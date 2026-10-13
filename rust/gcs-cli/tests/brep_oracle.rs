//! The Rust B-rep kernel held to OCCT (docs/rust-kernel-plan.md: OCCT as the test-only oracle):
//! every node of every corpus object's CAD recipe that the kernel builds, built by both, the
//! volumes equal, faces equal by kind and the boundary valid; every object against the core's
//! faceted kernel; every body written as STEP and parsed back against itself (read back by OCCT as
//! a valid solid with its faces, in the slow tier), and meshed closed within its bar. A node the kernel refuses by name (an intersection it
//! does not trace yet) is counted, never passed over silently.
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
use gcs_core::brep::{geom::Surface,props::volume,recipe};
use gcs_core::json::{object,Json};
use gcs_core::space::{cross,norm,sub as minus3};
use std::collections::BTreeMap;

/// An edge tolerance OCCT's result declares past its operands' (1e-7 mm): it approximated a meeting
/// it could not resolve to them. And how little ours may move with its tolerance there.
const DECLARED: f64 = 1e-6;
const CONVERGED: f64 = 1e-9;

/// A whole revolution's volume by Pappus, where its profile is one loop of lines and arcs in a
/// plane through its axis: `2π ∬ s dA`, `s` the distance from the axis, by Green's theorem `∮ s²/2
/// dt` (`t` along the axis), each line's and arc's integral by Gauss–Legendre (a polynomial along a
/// line, a trigonometric one along an arc).
fn pappus(n: &Json) -> Option<f64> {
    use gcs_core::space::{add,dot,normalised,scale};
    type V = [f64;3];
    let v = |j: &Json| -> V { std::array::from_fn(|i| j.arr()[i].as_f64()) };
    if n.get("kind")?.as_str() != "revolve" || (n.get("angle")?.as_f64().abs()-2.*std::f64::consts::PI).abs() > 1e-12 { return None }
    let loops = n.get("profile")?.get("loops")?.arr();
    if loops.len() != 1 { return None }
    let (o,a) = (v(n.get("origin")?),normalised(v(n.get("axis")?))?);
    // each piece as a point and its derivative over [0, 1], in the loop's order
    let mut pieces: Vec<Box<dyn Fn(f64) -> (V,V)>> = Vec::new();
    let mut last: Option<V> = None;
    let entries = loops[0].arr();
    for (k,e) in entries.iter().enumerate() {
        match e.get("kind")?.as_str() {
            "line" => {
                let (p,q) = (v(e.get("start")?),v(e.get("end")?));
                let d = minus3(q,p);
                pieces.push(Box::new(move |u| (add(p,scale(d,u)),d)));
                last = Some(q);
            }
            "circle" => {
                let (c,nn,x,r) = (v(e.get("center")?),v(e.get("normal")?),v(e.get("x_dir")?),e.get("radius")?.as_f64());
                let y = cross(nn,x);
                let span = e.get("angles")?.arr();
                let (a0,a1) = (span[0].as_f64(),span[1].as_f64());
                let at = move |th: f64| add(c,add(scale(x,r*th.cos()),scale(y,r*th.sin())));
                // walked from the end that meets its neighbour
                let next = entries.get(k+1).or(entries.first()).and_then(|f| f.get("start")).map(&v);
                let far = |p: V,q: V| { let d = minus3(q,p); dot(d,d) };
                let from_a0 = match (last,next) {
                    (Some(p),_) => far(at(a0),p) <= far(at(a1),p),
                    (None,Some(q)) => far(at(a1),q) <= far(at(a0),q),
                    _ => true,
                };
                let (s0,s1) = if from_a0 { (a0,a1) } else { (a1,a0) };
                pieces.push(Box::new(move |u| {
                    let th = s0+(s1-s0)*u;
                    (at(th),scale(add(scale(x,-r*th.sin()),scale(y,r*th.cos())),s1-s0))
                }));
                last = Some(at(s1));
            }
            _ => return None,
        }
    }
    // the side the profile stands on, across the axis in its plane
    let across = |p: V| { let d = minus3(p,o); minus3(d,scale(a,dot(d,a))) };
    let side = pieces.iter().map(|f| across(f(0.5).0)).max_by(|p,q| dot(*p,*p).total_cmp(&dot(*q,*q)))?;
    let e = normalised(side)?;
    // each piece by Gauss–Legendre's eight points on each of eight panels: a polynomial along a
    // line, a trigonometric one along an arc, exact to rounding at this order
    let panels = 8;
    let mut moment = 0.;
    for f in &pieces {
        for j in 0..panels {
            for &(xi,wi) in &gcs_core::integral::GAUSS8 {
                let (p,dp) = f((j as f64+0.5*(xi+1.))/panels as f64);
                let s = dot(minus3(p,o),e);
                moment += 0.5*wi/panels as f64*(-0.5*s*s*dot(dp,a));
            }
        }
    }
    Some(2.*std::f64::consts::PI*moment.abs())
}

/// A body node's volume built operand by operand by the Boolean at a tolerance `k` of its size: its
/// stock, plus what is `on` it, less what `cut`s it, within what `bound`s it (none late).
fn rebuilt(n: &Json,ours: &BTreeMap<i64,gcs_core::brep::topo::Brep>,k: f64) -> Result<f64,String> {
    use gcs_core::brep::boolean::{boolean,Op};
    let ids = |key: &str| n.get(key).map(|l| l.arr().iter().map(|x| x.as_i64()).collect::<Vec<_>>()).unwrap_or_default();
    if !ids("on_late").is_empty() || !ids("cut_late").is_empty() { return Err("late operands".into()) }
    let of = |i: i64| ours.get(&i).ok_or_else(|| format!("no operand {i}"));
    let mut b = of(n.get("stock").ok_or("no stock")?.as_i64())?.clone();
    for (key,op) in [("on",Op::Union),("cut",Op::Cut),("bound",Op::Common)] {
        for i in ids(key) {
            let other = of(i)?;
            b = boolean(&b,other,op,k*b.size().max(other.size()))?;
        }
    }
    Ok(volume(&b))
}

#[test]
fn every_node_the_kernel_builds_is_occts() {
    let session = native::Session::new().unwrap();
    let (mut compared,mut skipped) = (0,BTreeMap::<String,usize>::new());
    let mut worst: f64 = 0.;
    let mut failures: Vec<String> = Vec::new();
    let mut refused: Vec<String> = Vec::new();
    let mut bodies = 0;
    for (name,e) in fixtures::examples() {
        // `BREP_ORACLE_ONLY=name` runs one example
        if std::env::var("BREP_ORACLE_ONLY").is_ok_and(|only| !name.contains(&only)) { continue }
        let Some(e) = e else { continue };
        if !e.ok() { continue }
        let mut sk = e.sketch.clone();
        gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
        for root in gcs_core::overview::objects(&sk) {
            let Ok(r) = gcs_core::solid::cad::recipe(&sk,root) else { continue };
            let root_id = r.get("root").unwrap().as_i64();
            let nodes = r.get("nodes").unwrap().arr();
            let mut ours = BTreeMap::new();
            for (i,n) in nodes.iter().enumerate() {
                let id = n.get("id").unwrap().as_i64();
                let label = format!("{name}: `{}` ({})",n.get("name").unwrap().as_str(),n.get("kind").unwrap().as_str());
                let names: std::collections::BTreeMap<i64,String> = nodes.iter().map(|m| (m.get("id").unwrap().as_i64(),m.get("name").unwrap().as_str().to_string())).collect();
                let b = match recipe::node_named(n,&ours,&names,0.) {
                    Ok(b) => b,
                    // a refusal by name is the kernel saying what it does not build yet
                    Err(err) if err.contains("not built yet") => { refused.push(format!("{label}: {err}")); continue }
                    Err(err) if n.get("kind").unwrap().as_str() == "body" => { failures.push(format!("{label}: {err}")); continue }
                    Err(err) => { *skipped.entry(err).or_default() += 1; continue }
                };
                let tol = 1e-9*b.size().max(1.);
                if let Err(err) = b.check(tol*10.) { failures.push(format!("{label}: {err}")); continue }
                let sub = object([("schema",1.into()),("units","mm".into()),("root",id.into()),
                    ("nodes",Json::Arr(nodes[..=i].to_vec()))]);
                let (v,faces) = (volume(&b),b.faces.len());
                // OCCT builds no lofts: for one (and what is built of it) the faceted kernel is the reference
                let theirs = match session.construct(&sub) {
                    Ok(t) => Some(t),
                    Err(err) if err.contains("along-guide lofts") => { eprintln!("{label}: {v:.9} mm³ (OCCT builds no lofts)"); None }
                    // nor a lowered round (`tests/fillet_oracle.rs` holds those to OCCT's fillet)
                    Err(err) if err.contains("lowered round") => { eprintln!("{label}: {v:.9} mm³ (OCCT builds no lowered rounds)"); None }
                    Err(err) => panic!("{label}: OCCT: {err}"),
                };
                if let Some(theirs) = theirs {
                let w = session.volume(theirs).unwrap();
                let their_faces = session.faces(theirs).unwrap().len();
                let rel = (v-w).abs()/w.abs();
                eprintln!("{label}: {v:.9} mm³ against {w:.9}, {rel:e}; {faces} faces against {their_faces}");
                worst = worst.max(rel);
                // a `through` prism's extent is its sources' box, which OCCT widens by its tolerances and,
                // about a spline face, takes from its poles, where this kernel samples the face: the
                // prism's depth is then a little different and nothing cut by it is
                let swept = |b: &gcs_core::brep::topo::Brep| b.faces.iter().any(|f| matches!(f.surface,Surface::Extrusion(..) | Surface::Revolution(..)));
                let through_splines = n.get("kind").unwrap().as_str() == "through"
                    && n.get("sources").unwrap().arr().iter().any(|s| ours.get(&s.as_i64()).is_some_and(swept));
                // OCCT integrates a face swept from a spline (its adaptive rule asked for 1e-9) to a few
                // parts in a million, where this kernel's Gauss rule is exact on each knot span: on
                // `solid_tooth` fitted within 1e-7 mm this kernel meets the involute's closed form to
                // 1e-9 and OCCT is 4e-8 off; fitted within 1e-4 mm, OCCT is 2e-6 off (`tests/brep.rs`
                // and `brep_census.rs` hold this kernel to the closed forms)
                let bar = if through_splines { 1e-3 } else if swept(&b) { 1e-5 } else { 1e-7 };
                if rel >= bar {
                    // a Boolean whose meeting OCCT approximates where it is ill-conditioned (two
                    // surfaces touching at a point, a traced meeting through it) and says so: its edges'
                    // tolerance grown far past its operands'. Its boundary is then only within that
                    // tolerance of the true one, so its volume within it times the boundary's area;
                    // and ours is held to itself, rebuilt at tolerances ten times either way, within
                    // `CONVERGED`
                    // a whole revolution of lines and arcs: its volume is Pappus's, which decides
                    if let Some(closed) = pappus(n) {
                        let off = ((v-closed)/closed).abs();
                        if off >= CONVERGED { failures.push(format!("{label}: volume {v} against Pappus's {closed} ({off:.1e})")); continue }
                        eprintln!("{label}: OCCT {rel:.1e} off; ours Pappus's {closed:.9} mm³ within {off:.1e}");
                    } else {
                    let [_,edge_tol,_] = session.tolerances(theirs).unwrap();
                    let area = gcs_core::brep::mesh::mesh(&b,1e-3*b.size(),0.2).map(|m| m.tris.iter().map(|t| {
                        let [p,q,r] = t.map(|i| m.pts[i as usize]);
                        0.5*norm(cross(minus3(q,p),minus3(r,p)))
                    }).sum::<f64>()).unwrap_or(f64::INFINITY);
                    let held = n.get("kind").unwrap().as_str() == "body" && edge_tol > DECLARED
                        && (v-w).abs() <= edge_tol*area
                        && [1e-8,1e-10].iter().all(|&k| rebuilt(n,&ours,k).is_ok_and(|u| ((u-v)/v).abs() < CONVERGED));
                    if !held { failures.push(format!("{label}: volume {v} against OCCT's {w}")); continue }
                    eprintln!("{label}: OCCT declares its edges good to {edge_tol:.1e} mm (its bound on the volume {:.1e} mm³, off by \
                        {:.1e}); ours converged within {CONVERGED:e} at tolerances ten times either way",edge_tol*area,(v-w).abs());
                    }
                }
                // faces by the kind of their surface (OCCT's `GeomAbs_SurfaceType` order): OCCT may split
                // a plane along a line where another face only touches it, which this kernel does not
                let kinds = |ks: &mut Vec<i32>| { ks.sort(); ks.clone() };
                let our_kinds = kinds(&mut b.faces.iter().map(|f| match f.surface {
                    Surface::Plane(_) => 0,Surface::Cylinder(..) => 1,Surface::Cone(..) => 2,Surface::Sphere(..) => 3,Surface::Torus(..) => 4,
                    Surface::Revolution(..) => 7,Surface::Extrusion(..) => 8,Surface::Blend(..) | Surface::BSpline(..) => 6,
                }).collect());
                let their_kinds = kinds(&mut session.faces(theirs).unwrap().into_iter().map(|g| session.face_kind(g).unwrap()).collect());
                if our_kinds != their_kinds {
                    let extra_planes = their_kinds.iter().filter(|&&k| k != 0).eq(our_kinds.iter().filter(|&&k| k != 0))
                        && their_kinds.iter().filter(|&&k| k == 0).count() > our_kinds.iter().filter(|&&k| k == 0).count();
                    // or OCCT builds a general surface (a B-spline, a surface of revolution) for an
                    // analytic one of ours, one for one, lying on our surfaces within the face
                    // tolerance OCCT declares (the drill's wheel's nose, a torus OCCT does not read as one)
                    let minus = |a: &[i32],b: &[i32]| { let mut rest = b.to_vec(); a.iter().filter(|k| {
                        match rest.iter().position(|j| j == *k) { Some(i) => { rest.remove(i); false } None => true }
                    }).copied().collect::<Vec<i32>>() };
                    let (extra,missing) = (minus(&their_kinds,&our_kinds),minus(&our_kinds,&their_kinds));
                    let general = |k: &i32| *k == 6 || *k == 7;
                    let stood_in = !extra.is_empty() && extra.len() == missing.len() && extra.iter().all(general)
                        && !missing.iter().any(general) && {
                        let face_tol = session.tolerances(theirs).unwrap()[0].max(1e-9);
                        let off = session.off_surfaces(theirs,&b,&|k| general(&k)).unwrap();
                        eprintln!("  (OCCT builds {extra:?} for our {missing:?}, on our surfaces within {off:.1e} mm, its faces' tolerance {face_tol:.1e})");
                        off <= face_tol
                    };
                    if extra_planes { eprintln!("  (OCCT splits a plane along a line it only touches: {their_kinds:?} against our {our_kinds:?})"); }
                    if !extra_planes && !stood_in {
                        failures.push(format!("{label}: faces by kind {our_kinds:?} against OCCT's {their_kinds:?}"));
                        continue
                    }
                }
                }
                let whole = n.get("kind").unwrap().as_str() == "body" || id == root_id;
                if whole && !b.pinches().is_empty() {
                    // a solid touching itself at a point has no manifold file (OCCT's export of one refuses too)
                    refused.push(format!("{label}: pinches at {:?}",b.pinches()[0]));
                } else if whole {
                    // our STEP, parsed back and checked against our solid face by face
                    let text = match gcs_core::brep::step::write(&b,"oracle",1e-5) {
                        Ok(text) => text,
                        Err(err) => { failures.push(format!("{label}: not written as STEP: {err}")); continue }
                    };
                    if let Err(err) = native::step_check::verify(&text,&native::step_check::Solid::of(&b)) {
                        failures.push(format!("{label}: our STEP does not describe it: {err}")); continue
                    }
                    // the slow tier: read back by OCCT as a valid solid with our faces. Its volume of the
                    // reading is only a check against gross misreading: the meter finds the file's faces
                    // on their surfaces exactly where OCCT's integration of it is 2e-4 off
                    if cfg!(feature="slow") {
                        let file = std::env::temp_dir().join(format!("solvent-brep-oracle-{}.step",std::process::id()));
                        std::fs::write(&file,&text).unwrap();
                        let read = session.read_step(file.to_str().unwrap());
                        let _ = std::fs::remove_file(&file);
                        let read = read.and_then(|r| session.validate(r).map(|_| r))
                            .and_then(|r| Ok((session.volume(r)?,session.faces(r)?.len())));
                        match read {
                            Ok((u,n)) if n == faces && ((u-v)/v).abs() < 1e-3 => {}
                            Ok((u,n)) => { failures.push(format!("{label}: our STEP reads back as {u} mm³ in {n} faces against {v} in {faces}")); continue }
                            Err(err) => { failures.push(format!("{label}: our STEP does not read back: {err}")); continue }
                        }
                    }
                    // our mesh: closed, within its bar
                    match gcs_core::brep::mesh::mesh(&b,0.01,0.2) {
                        Ok(m) if m.sag <= 0.01 => if let Err(err) = gcs_core::mesh::stl_shells(&gcs_core::mesh::stl_of(&m.triangles(),"oracle")) {
                            failures.push(format!("{label}: our mesh is not closed: {err}")); continue
                        },
                        Ok(m) => { failures.push(format!("{label}: our mesh sags {}",m.sag)); continue }
                        Err(err) => { failures.push(format!("{label}: not meshed: {err}")); continue }
                    }
                    bodies += 1;
                }
                // the object itself against the core's faceted kernel, cut as it cuts a mesh (its sagitta a
                // small fraction of the object's diagonal): to its faceting, a part in 200
                if id == root_id {
                    let scale = gcs_core::solid::cad::millimetres(&sk).unwrap();
                    let facets = sk.evaluated_solid(root,gcs_core::solid::ApproximationPolicy::Mesh).map(|e| e.volume()*scale.powi(3));
                    match facets {
                        Ok(w) if ((v-w)/v).abs() < 5e-3 => {}
                        Ok(w) => { failures.push(format!("{label}: the faceted kernel's volume {w} against {v}")); continue }
                        Err(err) => eprintln!("  (not faceted: {err})"),
                    }
                }
                ours.insert(id,b);
                compared += 1;
            }
        }
    }
    eprintln!("{compared} nodes compared, worst {worst:e}; {bodies} bodies written as STEP and checked (read back by OCCT in the slow tier), and meshed; not built: {skipped:?}");
    for r in &refused { eprintln!("refused {r}"); }
    for f in &failures { eprintln!("FAILED {f}"); }
    assert!(compared > 100 || std::env::var_os("BREP_ORACLE_ONLY").is_some(),"{compared}");
    assert!(failures.is_empty(),"{} failures",failures.len());
    assert!(refused.len() <= 8,"{} refused",refused.len());
}

/// Rational B-splines both ways through OCCT: solids turned from exact arcs (`nurbs::arc`), written
/// by our STEP writer as surfaces of revolution over rational curves and read by OCCT as valid
/// solids of their closed-form volumes; then OCCT's reading handed back through its dump, where its
/// faces arrive as rational nets, read by `brep::json` into a solid of the same volume. (Not a
/// prism of a closed spline: OCCT reads one, rational or not, 3.6% over; nor the ball's dump, whose
/// net with poles at both ends `brep::json` reads as volume 0, rational or not.)
#[test]
fn rational_solids_go_through_occt_and_back() {
    use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
    use gcs_core::brep::geom::{Frame,Surface};
    use gcs_core::brep::nurbs::arc;
    use std::f64::consts::{FRAC_PI_2,PI,TAU};
    let session = native::Session::new().unwrap();
    let spline = |c| ProfileEdge::Spline(std::sync::Arc::new(c));
    let xz = [0.,1.,0.];
    let solids = [
        ("ball",revolve(&Profile {names:vec![],origin:[0.;3],normal:xz,loops:vec![vec![spline(arc(&Frame::new([0.;3],xz,[1.,0.,0.]),2.,[-FRAC_PI_2,FRAC_PI_2])),
            ProfileEdge::Line {a:[0.,0.,-2.],b:[0.,0.,2.]}]]},[0.;3],[0.,0.,1.],TAU).unwrap(),4./3.*PI*8.,false),
        ("ring",revolve(&Profile {names:vec![],origin:[0.;3],normal:xz,loops:vec![vec![spline(arc(&Frame::new([3.,0.,1.],xz,[1.,0.,0.]),1.,[0.,TAU]))]]},[0.;3],[0.,0.,1.],TAU).unwrap(),TAU*3.*PI,true),
    ];
    for (what,b,whole,back) in solids {
        let text = gcs_core::brep::step::write(&b,what,1e-5).unwrap();
        assert!(text.contains("RATIONAL_B_SPLINE_CURVE"),"{what}: nothing rational written");
        let file = std::env::temp_dir().join(format!("solvent-rational-{what}-{}.step",std::process::id()));
        std::fs::write(&file,&text).unwrap();
        let read = session.read_step(file.to_str().unwrap());
        let _ = std::fs::remove_file(&file);
        let read = read.unwrap_or_else(|e| panic!("{what}: OCCT does not read our STEP: {e}"));
        session.validate(read).unwrap_or_else(|e| panic!("{what}: OCCT reads an invalid solid: {e}"));
        let theirs = session.volume(read).unwrap();
        eprintln!("{what}: OCCT reads {theirs:.9} mm³ against {whole:.9}");
        assert!(((theirs-whole)/whole).abs() < 1e-6,"{what}: OCCT reads {theirs} against {whole}");
        if !back { continue }
        let ours = gcs_core::brep::json::read(&session.brep_json(read).unwrap()).unwrap_or_else(|e| panic!("{what}: OCCT's dump not read: {e}"));
        assert!(ours.faces.iter().any(|f| matches!(&f.surface,Surface::BSpline(_,n) if n.is_rational())),"{what}: OCCT's reading carries no rational net");
        ours.check(1e-7).unwrap_or_else(|e| panic!("{what}: OCCT's dump read invalid: {e}"));
        let v = gcs_core::brep::props::volume(&ours);
        eprintln!("{what}: its dump read as {v:.12} mm³ ({:e})",(v-whole)/whole);
        assert!(((v-whole)/whole).abs() < 1e-9,"{what}: OCCT's dump read as {v} against {whole}");
    }
}
