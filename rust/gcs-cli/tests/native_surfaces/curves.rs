//! Joined source charts reach native trimming without importing contact grids.
use super::*;
use gcs_core::solid::ContactCoverOptions;

// Exercise native cuts reachable from these joined intervals.
// The fixture's trim locator is sampled; this does not finish the retained gaps
// or claim that every interval reaches an existing trim at both ends.
pub(super) fn split_joined_intervals(cad: &Cad,sweep: &SweepContacts,joined: &gcs_core::solid::ContactCurves,
    faces: &[c_int],pose: gcs_core::envelope::Motion) -> Option<(usize,usize,f64)> {
    let mut cuts = std::collections::BTreeMap::<c_int,(Vec<c_int>,usize,f64)>::new();
    for curve in &joined.curves {
        let at = |s| Some(sweep.at_contact_curve(curve,s,1e-10).unwrap().contact.position);
        for &face in faces {
            let Ok(traces) = super::pcurves::clipped_test_traces(cad,face,&at) else { continue; };
            for uv in traces {
                if uv.windows(2).any(|p| (0..2).any(|k| (p[0][k]-p[1][k]).abs() > 0.5)) { continue; }
                let edge = match cad.0.pcurve(face,&uv,false,1e-7) {
                    Ok(edge) => edge,
                    Err(error) if error.contains("must end on face trims") => continue,
                    Err(error) => panic!("joined contact edge: {error}"),
                };
                let projector = sweep.patches()[curve.patch].projector().unwrap();
                let free = curve.dependent.free()[0];
                let [lo,hi] = curve.range;
                let mut worst = 0_f64;
                for i in 0..31 {
                    let p = cad.0.edge_point(edge,(i as f64+0.37)/31.).unwrap();
                    let q = projector.project(pose.inverse().point(p)).unwrap();
                    let value = q.parameters[free];
                    let value = if free == 1 && sweep.patches()[curve.patch].is_periodic() {
                        [value,value-1.,value+1.].into_iter().min_by(|a,b|
                            (a-a.clamp(lo,hi)).abs().total_cmp(&(b-b.clamp(lo,hi)).abs())).unwrap()
                    } else { value };
                    let s = ((value-lo)/(hi-lo)).clamp(0.,1.);
                    worst = worst.max(distance(p,at(s).unwrap()));
                }
                assert!(worst < 0.002,"joined contact fit error {worst} mm");
                let entry = cuts.entry(face).or_default();
                entry.0.push(edge); entry.1 += curve.cells.len(); entry.2 = entry.2.max(worst);
            }
        }
    }
    let mut result: Option<(usize,usize,f64)> = None;
    for (face,(edges,pieces,worst)) in cuts {
        let split = cad.0.split_pcurves(face,&edges).unwrap_or_else(|error|
            panic!("{error}: face {face}, {} joined edges",edges.len()));
        let fragments = cad.0.faces(split).unwrap();
        assert!(fragments.len() >= 2);
        let entry = result.get_or_insert((0,0,0.));
        entry.0 += 1; entry.1 += pieces; entry.2 = entry.2.max(worst);
    }
    result
}

#[test]
fn joined_contact_circle_splits_both_native_sphere_caps() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let source = include_str!("../../../examples/solid_generating_sweep.sv")
        .replace("removal: GeneratingCut(tool,", "\
line tilt_axis(std.origin, std.front.toward)\n\
motion tilt(about: tilt_axis,phase: 90deg)\n\
solid placed(tool,under: tilt,at: 0deg)\n\
removal: GeneratingCut(placed,");
    let e = read(&source,&base);
    let id = e.map.ent_named("removal.body").unwrap().i();
    let sweep = SweepContacts::read(&e.sketch,id,1e-10).unwrap();
    let cad = Cad::new();
    let caps = cad.0.sweep_caps(&e.sketch,id).unwrap();
    for cap in &caps.endpoints {
        let cover = sweep.cover_at(cap.parameter,ContactCoverOptions {max_depth:12,max_cells:12000}).unwrap();
        let joined = sweep.join_contact_curves(cover,1e-10).unwrap();
        assert_eq!(joined.curves.len(),1);
        let curve = &joined.curves[0];
        assert_eq!(curve.range,[0.,1.]);
        assert!(curve.cells.len() > 1);
        let points: Vec<_> = (0..=64).map(|i|
            sweep.at_contact_curve(curve,i as f64/64.,1e-10).unwrap().contact.position).collect();
        assert_eq!(cap.faces.len(),1);
        let face = cap.faces[0];
        // A full-turn source interval winds once around this periodic face.
        // Its UV endpoints reach opposite seam representatives of one point.
        let edge = cad.0.contact_edge(face,&points,false,1e-7).unwrap();
        let center = [3.*cap.parameter.cos(),3.*cap.parameter.sin(),0.];
        let plane = |p: [f64;3]| -cap.parameter.sin()*p[0]+cap.parameter.cos()*p[1];
        for i in 0..97 {
            let p = cad.0.edge_point(edge,(i as f64+0.37)/97.).unwrap();
            assert!(plane(p).abs() < 1e-6);
            assert!((distance(p,center)-1.).abs() < 1e-6);
        }
        let fragments = cad.0.faces(cad.0.split_pcurves(face,&[edge]).unwrap()).unwrap();
        assert_eq!(fragments.len(),2);
        let mut signs = Vec::new();
        for fragment in fragments {
            let mut sign = None;
            let mut checks = 0;
            for i in 0..11 { for j in 0..11 {
                let Some(p) = cad.0.face_point(fragment,(i as f64+0.27)/11.,(j as f64+0.61)/11.,1e-9).unwrap() else { continue; };
                let value = plane(p.position);
                if value.abs() < 1e-6 { continue; }
                if let Some(sign) = sign { assert_eq!(sign,value > 0.); } else { sign = Some(value > 0.); }
                checks += 1;
            } }
            assert!(checks > 16);
            signs.push(sign.unwrap());
        }
        assert!(signs.contains(&true) && signs.contains(&false));
        let shifted: Vec<_> = points[32..64].iter().chain(points[..=32].iter()).copied().collect();
        assert!(cad.0.contact_edge(face,&shifted,false,1e-7).unwrap_err().contains("face seam"));
        assert!(cad.0.contact_edge(face,&[[0.;3]],false,1e-7).is_err());
    }
}
