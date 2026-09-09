//! Joined source charts reach native trimming without importing contact grids.
use super::*;
use gcs_core::solid::ContactCoverOptions;

fn path_point(sweep: &SweepContacts,joined: &gcs_core::solid::ContactCurves,path: &gcs_core::solid::ContactPath,s: f64)
    -> Result<[f64;3],String> {
    let (i,t) = if s == 1. { (path.segments.len()-1,1.) }
        else { let p = s*path.segments.len() as f64; (p.floor() as usize,p.fract()) };
    let segment = path.segments[i];
    let [a,b] = segment.range;
    sweep.at_contact_curve(&joined.curves[segment.curve],a+(b-a)*t,1e-10).map(|p| p.contact.position)
}

// The locator is still sampled, so this checks attached candidate edges rather
// than claiming complete clipping or a partition of the entire cap.
pub(super) fn attach_path(cad: &Cad,sweep: &SweepContacts,joined: &gcs_core::solid::ContactCurves,
    path: &gcs_core::solid::ContactPath,faces: &[c_int],pose: gcs_core::envelope::Motion) -> (usize,f64) {
    let mut result = (0,f64::INFINITY);
    for n in [32,64,128,256] {
        result = attach_path_at_resolution(cad,sweep,joined,path,faces,pose,n*path.segments.len());
        if result.1 < 0.002 { return result; }
    }
    panic!("native multi-chart contact fit error {} mm",result.1)
}

fn attach_path_at_resolution(cad: &Cad,sweep: &SweepContacts,joined: &gcs_core::solid::ContactCurves,
    path: &gcs_core::solid::ContactPath,faces: &[c_int],pose: gcs_core::envelope::Motion,samples: usize) -> (usize,f64) {
    let source = &joined.curves[path.segments[0].curve];
    let projector = sweep.patches()[source.patch].projector().unwrap();
    let at = |s| Some(path_point(sweep,joined,path,s).unwrap());
    let (mut edges,mut worst) = (0,0_f64);
    let mut stages = [0;3];
    for &face in faces {
        let traces = match super::pcurves::clipped_test_traces_at_resolution(cad,face,&at,samples) {
            Ok(traces) => traces,
            Err(_) => { stages[0] += 1; continue; },
        };
        for uv in traces {
            stages[1] += 1;
            let edge = match cad.0.pcurve(face,&uv,false,1e-7) {
                Ok(edge) => edge,
                Err(error) if error.contains("must end on face trims") => { stages[2] += 1; continue; },
                Err(error) => panic!("native path edge: {error}"),
            };
            for i in 0..31 {
                let p = cad.0.edge_point(edge,(i as f64+0.37)/31.).unwrap();
                let q = projector.project(pose.inverse().point(p)).unwrap();
                let mut closest = f64::INFINITY;
                for segment in &path.segments {
                    let curve = &joined.curves[segment.curve];
                    assert_eq!(curve.patch,source.patch);
                    let [a,b] = curve.range;
                    let free = curve.dependent.free()[0];
                    let offsets: &[f64] = if free == 1 && sweep.patches()[curve.patch].is_periodic() { &[-1.,0.,1.] } else { &[0.] };
                    for &offset in offsets {
                        let s = ((q.parameters[free]+offset-a)/(b-a)).clamp(segment.range[0].min(segment.range[1]),segment.range[0].max(segment.range[1]));
                        let exact = sweep.at_contact_curve(curve,s,1e-10).unwrap().contact.position;
                        closest = closest.min(distance(p,exact));
                    }
                }
                worst = worst.max(closest);
            }
            edges += 1;
        }
    }
    eprintln!("path patch {}, {samples} samples: {edges} edges, fit error {worst:e} mm; projection errors/runs/interior endpoints {stages:?}",source.patch);
    if edges == 0 {
        let ends = [path.segments.first().unwrap(),path.segments.last().unwrap()];
        eprintln!("unfinished source ends {:?}",[0,1].map(|i|
            sweep.at_contact_curve(&joined.curves[ends[i].curve],ends[i].range[i],1e-10).unwrap().parameters));
    }
    (edges,worst)
}

#[test]
fn alternate_chart_path_trims_a_native_sphere_across_its_seam() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let source = include_str!("../../../examples/solid_generating_sweep.sv")
        .replace("start: -60deg", "start: 0deg")
        .replace("removal: GeneratingCut(tool,", "line tilt_axis(std.origin, std.front.toward)\n\
motion tilt(about: tilt_axis,phase: 15deg)\nsolid placed(tool,under: tilt,at: 0deg)\nremoval: GeneratingCut(placed,");
    let e = read(&source,&base);
    let id = e.map.ent_named("removal.body").unwrap().i();
    let sweep = SweepContacts::read(&e.sketch,id,1e-10).unwrap();
    let joined = sweep.join_contact_curves(sweep.cover_at(0.,ContactCoverOptions {max_depth:20,max_cells:30000}).unwrap(),1e-10).unwrap();
    let path = sweep.trace_contact_path(&joined,0,true,1e-10,1e-8).unwrap();
    assert!(path.closed && path.segments.len() > 2);
    let at = |s| path_point(&sweep,&joined,&path,s);
    let cad = Cad::new();
    let caps = cad.0.sweep_caps(&e.sketch,id).unwrap();
    let face = caps.endpoints[0].faces[0];
    let mut edge = None;
    for samples in [64,128,256] {
        let n = path.segments.len()*samples;
        let seeds: Vec<_> = (0..=n).map(|i| i as f64/n as f64).collect();
        let traces = cad.0.face_traces(face,&at,&seeds,true,1e-7).unwrap();
        assert_eq!(traces.len(),1);
        assert!(!traces[0].closed && traces[0].intervals.len() == 2);
        assert!((traces[0].intervals.iter().map(|p| p[1]-p[0]).sum::<f64>()-1.).abs() < 1e-12);
        let candidate = cad.0.pcurve(face,&traces[0].points,false,1e-7).unwrap();
        let mut error = 0_f64;
        for i in 0..97 {
            let p = cad.0.edge_point(candidate,(i as f64+0.37)/97.).unwrap();
            error = error.max(p[1].abs());
            assert!((distance(p,[3.,0.,0.])-1.).abs() < 1e-6);
        }
        eprintln!("{n} multi-chart samples: withheld contact-plane error {error:e} mm");
        if error < 1e-5 { edge = Some(candidate); break; }
    }
    let edge = edge.expect("native path fit must converge to the independent contact circle");
    let faces = cad.0.faces(cad.0.split_pcurves(face,&[edge]).unwrap()).unwrap();
    assert_eq!(faces.len(),2);
    let mut signs = Vec::new();
    for face in faces {
        let mut sign = None;
        let mut checks = 0;
        for i in 0..13 { for j in 0..13 {
            let Some(p) = cad.0.face_point(face,(i as f64+0.27)/13.,(j as f64+0.61)/13.,1e-9).unwrap() else { continue; };
            if p.position[1].abs() < 1e-5 { continue; }
            if let Some(sign) = sign { assert_eq!(sign,p.position[1] > 0.); } else { sign = Some(p.position[1] > 0.); }
            checks += 1;
        } }
        assert!(checks > 20);
        signs.push(sign.unwrap());
    }
    assert!(signs.contains(&true) && signs.contains(&false));
}

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
        let at = |s: f64| sweep.at_contact_curve(curve,(s+0.5)%1.,1e-10).map(|p| p.contact.position);
        let seeds: Vec<_> = (0..=64).map(|i| i as f64/64.).collect();
        let traces = cad.0.face_traces(face,&at,&seeds,true,1e-7).unwrap();
        assert_eq!(traces.len(),1);
        assert!(!traces[0].closed);
        assert_eq!(traces[0].intervals.len(),2,"the closed curve must be rebased at its interior seam");
        let edge = cad.0.pcurve(face,&traces[0].points,false,1e-7).unwrap();
        for i in 0..97 {
            let p = cad.0.edge_point(edge,(i as f64+0.37)/97.).unwrap();
            assert!(plane(p).abs() < 1e-6 && (distance(p,center)-1.).abs() < 1e-6);
        }
        assert_eq!(cad.0.faces(cad.0.split_pcurves(face,&[edge]).unwrap()).unwrap().len(),2);
    }
}
