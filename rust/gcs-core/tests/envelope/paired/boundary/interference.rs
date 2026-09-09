//! Independent solid-side checks in spherical sections. Shaft rotations preserve rho.
//! This is a sampled check, not a certificate between stations or motion samples.
use super::*;

struct Curve {
    patch: RevolvedSurface,
    range: [f64;2],
    index: Motion,
}

impl Curve {
    fn point(&self, pair: &Pair, member: usize, rho: f64, s: f64) -> V {
        let u = self.range[0]+s*(self.range[1]-self.range[0]);
        self.index.point(pair.analytic(member,&self.patch,u,rho).contact.position)
    }
}

enum Boundary {
    Generated(Curve),
    Cone(f64),
}

// Conservative bounds over a whole straight-flank height interval. These establish
// that the sphere/circle branch and its selected roll remain continuous between the
// contact-window endpoints; endpoint coverage alone would otherwise be insufficient.
fn check_contact_interval(pair: &Pair, patch: &RevolvedSurface, h: [f64;2], rho: f64) {
    let v = patch.domain()[1][0];
    let a = patch.at(0.,v).unwrap();
    let r = h.map(|h| {
        let p = patch.at((h-a.position[2])/a.du[2],v).unwrap().position;
        (p[0]-pair.offset[0]).hypot(p[1]-pair.offset[1])
    });
    let rmin = r[0].min(r[1]); let rmax = r[0].max(r[1]);
    let hmax = h[0].abs().max(h[1].abs());
    let hmin = if h[0] <= 0. && h[1] >= 0. { 0. } else { h[0].abs().min(h[1].abs()) };
    let [cx,cy,_] = pair.offset;
    let c = cx.hypot(cy);
    assert!(rmin > 0. && rho > hmax);
    let nlo = rho*rho-hmax*hmax-c*c-rmax*rmax;
    let nhi = rho*rho-hmin*hmin-c*c-rmin*rmin;
    let dlo = 2.*c*rmin; let dhi = 2.*c*rmax;
    let clo = (nlo/dlo).min(nlo/dhi);
    let chi = (nhi/dlo).max(nhi/dhi);
    assert!(clo > -1.+1e-3 && chi < 1.-1e-3,"sphere branch can become tangent");
    let theta = [cy.atan2(cx)-clo.acos(),cy.atan2(cx)-chi.acos()];
    assert!(theta[0] > -FRAC_PI_2 && theta[1] < FRAC_PI_2);
    let px_min = cx+rmin*theta[0].cos().min(theta[1].cos());
    let n = envelope::contact(a,Motion::identity()).unwrap().normal;
    // A = nz*Px-h*nx has a fixed sign. Thus atan(-B/A) has no branch cut,
    // and neither the characteristic velocity equation nor its roll is singular.
    assert!(n[2].abs()*px_min-hmax*n[0].abs() > 0.1*pair.module);
}

struct ExactSection<'a> {
    pair: &'a Pair,
    member: usize,
    rho: f64,
    first: f64,
    ends: Vec<f64>,
    edges: Vec<Boundary>,
}

impl<'a> ExactSection<'a> {
    fn flank(&self, side: usize) -> &Curve {
        let Boundary::Generated(c) = &self.edges[if side == 0 { 3 } else { 1 }] else {
            unreachable!()
        };
        c
    }

    fn read(pair: &'a Pair, member: usize, rho: f64) -> Self {
        let pitch = TAU/pair.teeth[member];
        let tip_theta = pair.delta[member]+(pair.module*35f64.to_radians().cos()/rho).asin();
        let curves = |side| {
            let outer = member == side;
            let edge = if outer { "outer" } else { "inner" };
            let flank = pair.patch(member,side,edge);
            let round = pair.patch(member,side,&format!("{edge}_round"));
            let join = if outer { 0. } else { 1. };
            // Independent scalar bracketing of the addendum intersection, using the
            // closed-form characteristic rather than the envelope's Newton equations.
            let mut lo = 0.; let mut hi = 1.;
            let at = |s| pair.analytic(member,&flank,1.-join+s*(2.*join-1.),rho).contact.position;
            assert!(polar(at(lo)) < tip_theta && polar(at(hi)) > tip_theta);
            for _ in 0..48 {
                let mid = (lo+hi)/2.;
                if polar(at(mid)) < tip_theta { lo = mid; } else { hi = mid; }
            }
            let tip_u = 1.-join+(lo+hi)/2.*(2.*join-1.);
            let index = rotate(2,if member == 0 && side == 0 { pitch } else { 0. },0.);
            [Curve {patch:round,range:[1.-join,join],index},
                Curve {patch:flank,range:[1.-join,tip_u],index}]
        };
        let [lower_round,lower_flank] = curves(1);
        let [mut upper_round,mut upper_flank] = curves(0);
        upper_round.range.reverse(); upper_flank.range.reverse();
        let first_point = lower_round.point(pair,member,rho,0.);
        let first = azimuth(first_point);
        let edges = vec![Boundary::Generated(lower_round),Boundary::Generated(lower_flank),
            Boundary::Cone(tip_theta),Boundary::Generated(upper_flank),
            Boundary::Generated(upper_round),Boundary::Cone(polar(first_point))];
        let mut ends = vec![];
        for (i,edge) in edges.iter().enumerate() {
            let end = match edge {
                Boundary::Generated(c) => c.point(pair,member,rho,1.),
                Boundary::Cone(_) if i == 2 => match &edges[3] {
                    Boundary::Generated(c) => c.point(pair,member,rho,0.),
                    _ => unreachable!(),
                },
                Boundary::Cone(_) => rotate(2,pitch,0.).point(first_point),
            };
            ends.push((azimuth(end)-first).rem_euclid(TAU));
        }
        assert!(ends.windows(2).all(|a| a[1] > a[0]));
        assert!((ends[5]-pitch).abs() < 1e-10);
        ends[5] = pitch;
        Self {pair,member,rho,first,ends,edges}
    }

    fn outer(&self, phi: f64) -> f64 {
        let pitch = TAU/self.pair.teeth[self.member];
        let angle = (phi-self.first).rem_euclid(pitch);
        let k = self.ends.partition_point(|&end| end < angle);
        match &self.edges[k] {
            Boundary::Cone(theta) => *theta,
            Boundary::Generated(c) => {
                let mut lo = 0.; let mut hi = 1.;
                for _ in 0..44 {
                    let mid = (lo+hi)/2.;
                    let p = c.point(self.pair,self.member,self.rho,mid);
                    let a = (azimuth(p)-self.first).rem_euclid(TAU);
                    if a < angle { lo = mid; } else { hi = mid; }
                }
                polar(c.point(self.pair,self.member,self.rho,(lo+hi)/2.))
            }
        }
    }

    fn clearance(&self, p: V) -> f64 {
        let theta = polar(p);
        let normal_module = self.pair.module*35f64.to_radians().cos();
        let tip = self.pair.delta[self.member]+(normal_module/self.rho).asin();
        let back = self.pair.delta[self.member]-(4.*normal_module/self.rho).asin();
        if theta > tip { return (theta-tip)*self.rho; }
        if theta < back { return (back-theta)*self.rho; }
        (theta-self.outer(azimuth(p))).max(back-theta)*self.rho
    }
}

struct Section {
    points: Vec<V>,
    angles: Vec<f64>,
    theta: Vec<f64>,
    back: f64,
    rho: f64,
}

impl Section {
    fn read(pair: &Pair, member: usize, rho: f64, n: usize) -> Self {
        let points = pair.rim_section(member,rho,n);
        let first = azimuth(points[0]);
        let mut angles: Vec<f64> = points.iter()
            .map(|&p| (azimuth(p)-first).rem_euclid(TAU)).collect();
        angles.push(TAU);
        let mut theta: Vec<f64> = points.iter().map(|&p| polar(p)).collect();
        theta.push(theta[0]);
        let back = pair.delta[member]-(4.*pair.module*35f64.to_radians().cos()/rho).asin();
        Self {points,angles,theta,back,rho}
    }

    // Negative is inside the nominal rim. Interpolation error is assessed by refinement.
    fn clearance(&self, p: V) -> f64 {
        let a = (azimuth(p)-azimuth(self.points[0])).rem_euclid(TAU);
        let j = self.angles.partition_point(|&v| v <= a).saturating_sub(1);
        let f = (a-self.angles[j])/(self.angles[j+1]-self.angles[j]);
        let outer = self.theta[j]+f*(self.theta[j+1]-self.theta[j]);
        let theta = polar(p);
        (theta-outer).max(self.back-theta)*self.rho
    }
}

#[test]
fn assembled_rim_sections_through_one_tooth_period() {
    let pair = Pair::read([24,48],2.);
    for n in [16,32,64] {
        let mut worst = (f64::INFINITY,0,0.,0.,[0.;3]);
        for face in 0..=8 {
            let rho = pair.rm*(0.9+0.025*face as f64);
            let sections = [Section::read(&pair,0,rho,n),Section::read(&pair,1,rho,n)];
            for phase in 0..=32 {
                let t = TAU/pair.teeth[0].hypot(pair.teeth[1])*phase as f64/32.;
                for member in 0..2 {
                    let transform = pair.body(member,t).then(pair.body(1-member,t).inverse());
                    for &p in &sections[member].points {
                        let q = transform.point(p);
                        let gap = sections[1-member].clearance(q);
                        if gap < worst.0 { worst = (gap,member,rho,t,p); }
                    }
                }
            }
        }
        eprintln!("n={n} worst section clearance: {worst:?}");
        assert!(worst.0 > -1e-9*pair.module,"sampled interference: {worst:?}");
    }
}

#[test]
fn assembled_boundaries_stay_outside_the_exact_mating_rim_at_sampled_mesh_phases() {
    let pair = Pair::read([24,48],2.);
    let mut worst = (f64::INFINITY,0,0.,0.,[0.;3]);
    for face in 0..=8 {
        let rho = pair.rm*(0.9+0.025*face as f64);
        let exact = [ExactSection::read(&pair,0,rho),ExactSection::read(&pair,1,rho)];
        let samples = [Section::read(&pair,0,rho,32),Section::read(&pair,1,rho,32)];
        for member in 0..2 {
            // The independent boundary classifier must return the source rim's own
            // generating points as its boundary, including tips and root connections.
            for &p in &samples[member].points {
                assert!((exact[member].outer(azimuth(p))-polar(p)).abs() < 1e-9);
            }
        }
        for phase in 0..=32 {
            let t = TAU/pair.teeth[0].hypot(pair.teeth[1])*phase as f64/32.;
            for member in 0..2 {
                let transform = pair.body(member,t).then(pair.body(1-member,t).inverse());
                for &p in &samples[member].points {
                    let gap = exact[1-member].clearance(transform.point(p));
                    if gap < worst.0 { worst = (gap,member,rho,t,p); }
                }
            }
        }
    }
    eprintln!("worst exact mating-rim clearance: {worst:?}");
    assert!(worst.0 > -1e-8*pair.module,"sampled interference: {worst:?}");
}

#[test]
fn both_flanks_have_overlapping_contact_windows_covering_a_full_tooth_period() {
    let pair = Pair::read([24,48],2.);
    contact_windows(&pair,None);
}

#[test]
#[ignore = "exports configured analytical contacts for the CAD pair workflow"]
fn export_contact_windows_for_cad_backend() {
    let path = std::env::var_os("SOLVENT_CAD_CONTACTS_OUTPUT").expect("set output JSON path");
    contact_windows(&Pair::read_cad_export(),Some(path));
}

fn contact_windows(pair: &Pair, output: Option<std::ffi::OsString>) {
    let period = TAU/pair.teeth[0].hypot(pair.teeth[1]);
    let mut cad_contacts = vec![];
    for side in 0..2 {
        let mut windows = vec![];
        for face in [0.9,0.95,1.,1.05,1.1] {
            let rho = face*pair.rm;
            let exact = [ExactSection::read(&pair,0,rho),ExactSection::read(&pair,1,rho)];
            let flanks = [exact[0].flank(side),exact[1].flank(side)];
            let mut low = f64::NEG_INFINITY;
            let mut high = f64::INFINITY;
            for flank in flanks {
                let h = flank.range.map(|u| flank.patch.at(u,flank.patch.domain()[1][0]).unwrap().position[2]);
                low = low.max(h[0].min(h[1]));
                high = high.min(h[0].max(h[1]));
            }
            // Stay strictly inside both tooth-tip/root-transition trims.
            low += 1e-6*pair.module; high -= 1e-6*pair.module;
            assert!(high > low);
            check_contact_interval(&pair,&flanks[0].patch,[low,high],rho);
            let at = |member: usize,h| {
                let patch = &flanks[member].patch;
                let a = patch.at(0.,patch.domain()[1][0]).unwrap();
                pair.analytic(member,patch,(h-a.position[2])/a.du[2],rho)
            };
            let t0 = at(0,low).parameters[2];
            let t1 = at(0,high).parameters[2];
            windows.push([t0.min(t1),t0.max(t1)]);
            // On every window solve contact directly, then verify both indexed bodies
            // at the common assembly phase, independent of the reference's frame.
            for j in 0..=24 {
                let target = t0+(t1-t0)*j as f64/24.;
                let mut lo = low; let mut hi = high;
                for _ in 0..44 {
                    let h = (lo+hi)/2.;
                    if (at(0,h).parameters[2]-target)*(t1-t0) < 0. { lo = h; }
                    else { hi = h; }
                }
                let h = (lo+hi)/2.;
                let a = at(0,h); let b = at(1,h);
                assert!((a.parameters[2]-target).abs() < 1e-10);
                assert!((a.parameters[2]-b.parameters[2]).abs() < 1e-10);
                let phase = target.rem_euclid(period);
                let k = ((phase-target)/period).round();
                let pi = rotate(2,-k*TAU/pair.teeth[0],0.);
                let gi = rotate(2,k*TAU/pair.teeth[1],0.);
                let p = pi.point(a.contact.position); let g = gi.point(b.contact.position);
                near(pair.body(0,phase).point(p),pair.body(1,phase).point(g),1e-7);
                let pn = pair.body(0,phase).vector(pi.vector(a.contact.normal));
                let gn = pair.body(1,phase).vector(gi.vector(b.contact.normal));
                near(pn,gn.map(|v| -v),1e-8);
                let local_positions = [p,g];
                let local_normals = [pi.vector(a.contact.normal),gi.vector(b.contact.normal)];
                let position = pair.body(0,phase).point(p);
                let fraction = phase/period;
                // At toe/heel and nearly at the tip an offset can cross a second
                // boundary. Preserve those contacts for distance checks only.
                let probe = face > 0.9 && face < 1.1 && j > 0 && j < 24;
                cad_contacts.push(format!("{{\"side\":{side},\"face_fraction\":{face},\"sample\":{j},\"tooth_fraction\":{fraction},\"local_positions_mm\":{local_positions:?},\"local_normals\":{local_normals:?},\"world_position_mm\":{position:?},\"world_normal\":{pn:?},\"offset_probe\":{probe}}}"));
                assert!(exact[0].clearance(p).abs() < 1e-7);
                assert!(exact[1].clearance(g).abs() < 1e-7);
                if j == 12 {
                    // Negative control: a phase error must put one displaced probe
                    // inside material and the opposite probe outside it.
                    let gaps = [-0.001,0.001].map(|angle|
                        exact[1].clearance(rotate(2,angle,0.).point(g)));
                    assert!(gaps[0].min(gaps[1]) < -1e-4);
                    assert!(gaps[0].max(gaps[1]) > 1e-4);
                }
            }
        }
        // A continuous characteristic supplies every roll between its endpoints.
        // Repeat these intervals by the exact indexing period and require overlapping
        // coverage, rather than checking contact only at a finite set of phases.
        let mut intervals = vec![];
        for [lo,hi] in &windows {
            for k in -10..=10 { intervals.push([lo+k as f64*period,hi+k as f64*period]); }
        }
        intervals.sort_by(|a,b| a[0].total_cmp(&b[0]));
        let mut covered: f64 = 0.;
        for [lo,hi] in intervals {
            if hi < 0. { continue; }
            if lo > covered { break; }
            covered = covered.max(hi);
        }
        assert!(covered > period+1e-6,"side {side}: missing contact in windows {windows:?}");
        eprintln!("side {side} contact windows, crown radians: {windows:?}; period={period}");
    }
    if let Some(path) = output {
        std::fs::write(path,format!("{{\"teeth\":{:?},\"module_mm\":{},\"mean_distance_mm\":{},\"crown_period_rad\":{period},\"scope\":\"Analytical nominal contact samples; CAD incidence and global interference unchecked\",\"contacts\":[{}]}}\n",
            pair.teeth,pair.module,pair.rm,cad_contacts.join(","))).unwrap();
    }
}
