use super::*;

#[test]
fn native_pcurves_partition_open_and_closed_regions_without_rebuilding_supports() {
    let cad = Cad::new();
    for closed in [false,true] {
        let face = cad.grid(4,|u,v| [2.*u-1.,2.*v-1.,0.]);
        let points: Vec<_> = if closed {
            (0..64).map(|i| { let t = i as f64*std::f64::consts::TAU/64.;
                [0.5+0.3*t.cos(),0.5+0.3*t.sin()] }).collect()
        } else {
            (0..=64).map(|i| { let v = i as f64/64.;
                [0.625-0.5*(2.*v-1.).powi(2),v] }).collect()
        };
        let edge = cad.0.pcurve(face,&points,closed,1e-7).unwrap();
        for i in 0..97 {
            let p = cad.0.edge_point(edge,(i as f64+0.37)/97.).unwrap();
            let value = if closed { p[0].hypot(p[1])-0.6 } else { p[0]+p[1]*p[1]-0.25 };
            assert!(value.abs() < 1e-5,"closed={closed}: curve residual {value}");
        }
        assert!(cad.0.edge_point(edge,-0.1).is_err());
        let fragments = cad.faces(cad.0.split_pcurves(face,&[edge]).unwrap());
        assert_eq!(fragments.len(),2);
        let mut signs = [None;2];
        for i in 0..19 { for j in 0..19 {
            let (u,v) = ((i as f64+0.27)/19.,(j as f64+0.61)/19.);
            let p = cad.at(face,u,v).unwrap().0;
            let value = if closed { p[0].hypot(p[1])-0.6 } else { p[0]+p[1]*p[1]-0.25 };
            if value.abs() < 1e-4 { continue; }
            let states: Vec<_> = fragments.iter().map(|&f| cad.contains(f,u,v).unwrap()).collect();
            assert_eq!(states.iter().filter(|s| **s == 1).count(),1,"{p:?}: {states:?}");
            let k = states.iter().position(|s| *s == 1).unwrap();
            assert!(distance(cad.at(fragments[k],u,v).unwrap().0,p) < 1e-12);
            if let Some(sign) = signs[k] { assert_eq!(sign,value > 0.); } else { signs[k] = Some(value > 0.); }
            let (uv,gap) = cad.0.face_parameters(fragments[k],p,1e-7).unwrap().unwrap();
            assert!(gap < 1e-12);
            let q = cad.0.face_point(fragments[k],uv[0],uv[1],1e-9).unwrap().unwrap();
            assert!(distance(q.position,p) < 1e-12);
        } }
        assert!(signs.contains(&Some(true)) && signs.contains(&Some(false)));
        assert_eq!(cad.contains(face,0.5,0.5).unwrap(),1,"source trims must be unchanged");
        assert_eq!(cad.faces(cad.0.split_pcurves(face,&[edge]).unwrap()).len(),2,
            "source faces and cutting edges must remain reusable");
        assert!(cad.0.pcurve(face,&[[0.2,0.2],[0.8,0.8]],false,1e-7).is_err());
        assert!(cad.0.pcurve(face,&[[0.2,0.2]],true,1e-7).is_err());
        assert!(cad.0.pcurve(face,&[[f64::NAN,0.],[0.5,1.]],false,1e-7).is_err());
        assert!(cad.0.split_pcurves(face,&[]).is_err());
        assert!(cad.0.split_pcurves(face,&[face]).is_err());
        assert!(cad.0.face_parameters(face,[0.,0.,1.],1e-7).unwrap().is_none());
        assert!(cad.0.face_parameters(face,[0.;3],0.).is_err());
    }
}

#[test]
fn alternate_source_charts_split_native_endpoint_faces() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let scale = e.sketch.units.length.unwrap().1;
    let cad = Cad::new();
    for member in ["pinion","gear"] {
        let id = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
        let sweep = SweepContacts::read(&e.sketch,id,1e-10/scale).unwrap();
        let caps = cad.0.sweep_caps(&e.sketch,id).unwrap();
        for cap in &caps.endpoints {
            let mut curves = std::collections::BTreeMap::<c_int,Vec<c_int>>::new();
            let mut unresolved_projections = 0;
            let mut stages = [0;4];
            let mut fit_error = 0_f64;
            // Try either exact source chart. Missing branches, poles and clipped
            // curves still need full event/trim tracing before cap coverage.
            for along_angle in [false,true] {
            let existing: std::collections::BTreeSet<_> = curves.keys().copied().collect();
            for patch in 0..sweep.patches().len() { for branch in 0..2 {
                let sample = |s| {
                    if along_angle {
                        sweep.at_angle(patch,s,cap.parameter,1e-10/scale).ok()?.into_iter()
                            .find(|r| r.branch == branch).map(|r| r.contact.position.map(|x| x*scale))
                    } else {
                        sweep.at(patch,s,cap.parameter,1e-10/scale).ok()?.into_iter()
                            .find(|r| r.branch == branch).map(|r| r.contact.position.map(|x| x*scale))
                    }
                };
                if !(0..=32).any(|i| sample(i as f64/32.).is_some()) { continue; }
                stages[0] += 1;
                let at = |s| sample(s).expect("a traced source branch disappeared");
                for &face in &cap.faces {
                    if existing.contains(&face) { continue; }
                    let traces = match clipped_test_traces(&cad,face,&sample) {
                        Ok(traces) => traces,
                        Err(_) => { unresolved_projections += 1; continue; },
                    };
                    for uv in traces {
                    stages[1] += 1;
                    if uv.windows(2).any(|p| (0..2).any(|k| (p[0][k]-p[1][k]).abs() > 0.5)) { continue; }
                    stages[2] += 1;
                    match cad.0.pcurve(face,&uv,false,1e-7) {
                        Ok(edge) => {
                            let projector = sweep.patches()[patch].projector().unwrap();
                            for j in 0..31 {
                                let p = cad.0.edge_point(edge,(j as f64+0.37)/31.).unwrap();
                                let source = cap.pose.inverse().point(p.map(|x| x/scale));
                                let projection = projector.project(source).unwrap();
                                let exact = at(projection.parameters[usize::from(along_angle)]);
                                fit_error = fit_error.max(distance(p,exact));
                            }
                            curves.entry(face).or_default().push(edge);
                        },
                        Err(error) if error.contains("must end on face trims") => { stages[3] += 1; continue; },
                        Err(error) => panic!("{member}, {} patch {patch}: {error}",cap.parameter),
                    }
                    }
                }
            } } }
            let mut split_faces = 0;
            let mut checks = 0;
            for (face,edges) in curves {
                let partition = cad.0.split_pcurves(face,&edges).unwrap_or_else(|error|
                    panic!("{member} endpoint {} face {face}, {} edges: {error}",cap.parameter,edges.len()));
                let fragments = cad.0.faces(partition).unwrap();
                let mut signs = vec![None;fragments.len()];
                let mut points = Vec::new();
                for probe in std::iter::once(face).chain(fragments.iter().copied()) {
                for i in 0..9 { for j in 0..9 {
                    let (u,v) = ((i as f64+0.27)/9.,(j as f64+0.61)/9.);
                    if let Some(p) = cad.0.face_point(probe,u,v,1e-9).unwrap() { points.push(p); }
                } } }
                for p in points {
                    let source = cap.pose.inverse().point(p.position.map(|x| x/scale));
                    let velocity = cap.pose.velocity(source);
                    let normal_velocity: f64 = (0..3).map(|k| p.normal[k]*velocity[k]).sum();
                    if normal_velocity.abs() < 1e-4/scale { continue; }
                    let mut owners = Vec::new();
                    for (k,&fragment) in fragments.iter().enumerate() {
                        if cad.0.face_parameters(fragment,p.position,1e-6).unwrap().is_some() { owners.push(k); }
                    }
                    assert_eq!(owners.len(),1,"{member}, {}: {owners:?}",cap.parameter);
                    let k = owners[0];
                    if let Some(sign) = signs[k] {
                        assert_eq!(sign,normal_velocity > 0.,"a cap fragment crosses an unsplit contact curve");
                    } else { signs[k] = Some(normal_velocity > 0.); }
                    checks += 1;
                }
                assert!(signs.contains(&Some(true)) && signs.contains(&Some(false)),
                    "{member} endpoint {}, face {face}: {signs:?}",cap.parameter);
                split_faces += 1;
            }
            eprintln!("{member} endpoint {}: {split_faces} faces split on source contact curves, {checks} sign/membership checks, {unresolved_projections} unresolved candidate-face projections",cap.parameter);
            eprintln!("sampled branches / on-face traces / unwrapped traces / interior endpoints: {stages:?}");
            eprintln!("withheld contact-edge error: {fit_error:e} mm");
            assert!(fit_error < 0.002,"{member}: native contact curve interpolation error {fit_error} mm");
            assert!(split_faces > 0,"{member}: endpoint {} has no native contact splits",cap.parameter);
        }
    }
}

// Integration-fixture search only: locate sampled on-face runs, then bracket
// their endpoints against native trims. This neither certifies that no narrow
// run was missed nor replaces full endpoint contact-event tracing.
fn clipped_test_traces(cad: &Cad,face: c_int,at: &impl Fn(f64)->Option<[f64;3]>) -> Result<Vec<Vec<[f64;2]>>,String> {
    let project = |u| match at(u) {
        Some(p) => cad.0.face_parameters(face,p,1e-6).map(|p| p.map(|(uv,_)| uv)),
        None => Ok(None),
    };
    const SEEDS: usize = 32;
    let samples = (0..=SEEDS).map(|i| project(i as f64/SEEDS as f64)).collect::<Result<Vec<_>,_>>()?;
    let endpoint = |mut inside: f64,mut outside: f64| -> Result<f64,String> {
        for _ in 0..52 {
            let mid = (inside+outside)*0.5;
            if mid == inside || mid == outside { break; }
            if project(mid)?.is_some() { inside = mid; } else { outside = mid; }
        }
        Ok(inside)
    };
    let mut result = Vec::new();
    let mut i = 0;
    while i < samples.len() {
        if samples[i].is_none() { i += 1; continue; }
        let start = i;
        while i+1 < samples.len() && samples[i+1].is_some() { i += 1; }
        let end = i; i += 1;
        // Incidence with an adjacent face only along its edge is not an
        // interior trimming curve on that face.
        let mut interior = false;
        for p in samples[start..=end].iter().flatten() {
            if cad.0.face_point(face,p[0],p[1],1e-9)?.is_some_and(|p| !p.on_trim) { interior = true; }
        }
        if !interior { continue; }
        let mut a = start as f64/SEEDS as f64; let mut b = end as f64/SEEDS as f64;
        if start > 0 { a = endpoint(a,(start-1) as f64/SEEDS as f64)?; }
        if end < SEEDS { b = endpoint(b,(end+1) as f64/SEEDS as f64)?; }
        if a == b { continue; }
        let mut points = Vec::new();
        for j in 0..=32 {
            let u = if j == 0 { a } else if j == 32 { b } else { a+(b-a)*j as f64/32. };
            let Some(p) = project(u)? else { return Err("fixture trace leaves its source face".into()); };
            points.push(p);
        }
        // A full turn can project its first point onto the opposite copy of the
        // native seam. Change only an endpoint representative, and only after
        // checking that the alternative is the same on-face spatial point.
        for (index,neighbor,s) in [(0,1,a),(32,31,b)] { for axis in 0..2 {
            let value = points[index][axis];
            if (value-points[neighbor][axis]).abs() <= 0.5 { continue; }
            let alias = if value < 1e-8 { 1. } else if value > 1.-1e-8 { 0. } else { continue; };
            let mut p = points[index]; p[axis] = alias;
            if cad.0.face_point(face,p[0],p[1],1e-9)?.is_some_and(|q| at(s).is_some_and(|p| distance(q.position,p) < 1e-6)) {
                points[index] = p;
            }
        } }
        result.push(points);
    }
    Ok(result)
}
