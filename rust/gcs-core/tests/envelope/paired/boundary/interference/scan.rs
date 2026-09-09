//! Report the existing independent spherical-section check at successive resolutions.
//! Clearance sign classifies material; its magnitude is a polar arc, not normal distance.
use super::*;

struct Witness {
    clearance: f64,
    member: usize,
    face: f64,
    point: V,
}

pub(super) struct Scan {
    subdivisions: usize,
    face_intervals: usize,
    phase_intervals: usize,
    offset: f64,
    queries: usize,
    minima: Vec<Witness>,
    seconds: f64,
}

impl Scan {
    pub(super) fn minimum(&self) -> f64 {
        self.minima.iter().map(|w| w.clearance).fold(f64::INFINITY,f64::min)
    }

    fn json(&self) -> String {
        let rows: Vec<_> = self.minima.iter().enumerate().map(|(i,w)| format!(
            "{{\"tooth_fraction\":{},\"minimum_signed_polar_clearance_mm\":{},\"member\":{},\"face_fraction\":{},\"local_point_mm\":{:?}}}",
            i as f64/self.phase_intervals.max(1) as f64,w.clearance,w.member,w.face,w.point)).collect();
        format!("{{\"profile_subdivisions\":{},\"face_intervals\":{},\"phase_intervals\":{},\"gear_phase_offset_rad\":{},\"boundary_queries\":{},\"seconds\":{},\"minimum_signed_polar_clearance_mm\":{},\"phases\":[{}]}}",
            self.subdivisions,self.face_intervals,self.phase_intervals,self.offset,self.queries,
            self.seconds,self.minimum(),rows.join(","))
    }
}

pub(super) fn run(pair: &Pair,n: usize,faces: usize,phases: usize,offset: f64) -> Scan {
    assert!(n > 0 && faces > 0 && offset.is_finite());
    let started = std::time::Instant::now();
    let mut result = Scan {subdivisions:n,face_intervals:faces,phase_intervals:phases,
        offset,queries:0,minima:(0..=phases).map(|_| Witness {
            clearance:f64::INFINITY,member:0,face:0.,point:[0.;3]}).collect(),seconds:0.};
    for face in 0..=faces {
        let fraction = 0.9+0.2*face as f64/faces as f64;
        let rho = pair.rm*fraction;
        let exact = [ExactSection::read(pair,0,rho),ExactSection::read(pair,1,rho)];
        let samples = [Section::read(pair,0,rho,n),Section::read(pair,1,rho,n)];
        for member in 0..2 {
            // Numerical envelope points must independently lie on the analytic rim,
            // including both generated flanks, root fillets and tip/root connections.
            for &p in &samples[member].points {
                assert!((exact[member].outer(azimuth(p))-polar(p)).abs() < 1e-9);
            }
        }
        for phase in 0..=phases {
            let t = TAU/pair.teeth[0].hypot(pair.teeth[1])*phase as f64/phases.max(1) as f64;
            let bodies = [pair.body(0,t),rotate(2,offset,0.).then(pair.body(1,t))];
            for member in 0..2 {
                let transform = bodies[member].then(bodies[1-member].inverse());
                for &p in &samples[member].points {
                    // Check both rim and back-cone section boundaries. Interior points
                    // of the toe/heel faces and regions between stations remain unsampled.
                    for p in [p,spherical(rho,samples[member].back,azimuth(p))] {
                        let gap = exact[1-member].clearance(transform.point(p));
                        assert!(gap.is_finite());
                        result.queries += 1;
                        if gap < result.minima[phase].clearance {
                            result.minima[phase] = Witness {clearance:gap,member,face:fraction,point:p};
                        }
                    }
                }
            }
        }
    }
    result.seconds = started.elapsed().as_secs_f64();
    result
}

#[test]
#[ignore = "exports configured nominal engagement refinement and wrong-phase evidence"]
fn export_engagement_scan_for_cad_backend() {
    let path = std::env::var_os("SOLVENT_CAD_ENGAGEMENT_OUTPUT").expect("set output JSON path");
    let pair = Pair::read_cad_export();
    let mut levels = vec![];
    for (n,faces,phases) in [(8,4,16),(16,8,32),(32,8,64)] {
        let result = run(&pair,n,faces,phases,0.);
        eprintln!("engagement n={n}, faces={faces}, phases={phases}: {} mm, {} s",
            result.minimum(),result.seconds);
        assert!(result.minimum() > -1e-8*pair.module,"sampled nominal interference");
        levels.push(result.json());
    }
    let wrong = run(&pair,16,4,0,0.001);
    assert!(wrong.minimum() < -1e-4*pair.module,"wrong-phase control missed interference");
    std::fs::write(path,format!(
        "{{\"teeth\":{:?},\"module_mm\":{},\"mean_distance_mm\":{},\"scope\":\"Nominal boundary material signs on listed spherical stations and motion samples; no between-sample or CAD accuracy bound. Signed polar clearance is not normal penetration depth.\",\"passes_sampled_checks\":true,\"levels\":[{}],\"wrong_phase\":{}}}\n",
        pair.teeth,pair.module,pair.rm,levels.join(","),wrong.json())).unwrap();
}
