//! The corpus's solids as the Rust kernel's rung 1 will meet them (docs/rust-kernel-plan.md,
//! phase 0 for the ladder): every object's CAD recipe, the analytic surfaces its operands bring,
//! and every Boolean whose operands share a surface — a flush bore, a boss `on` its stock, a mate,
//! coaxial cylinders of one radius. A report, ignored:
//! `cargo test brep_census -- --ignored --nocapture`.
use gcs_core::json::Json;
use gcs_core::model::Sketch;
use std::collections::BTreeMap;

type V = [f64; 3];
fn sub(a: V, b: V) -> V { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
fn dot(a: V, b: V) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
fn cross(a: V, b: V) -> V { [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]] }
fn norm(a: V) -> f64 { dot(a, a).sqrt() }
fn unit(a: V) -> V { let n = norm(a); [a[0] / n, a[1] / n, a[2] / n] }
fn scale(a: V, s: f64) -> V { [a[0] * s, a[1] * s, a[2] * s] }
fn add(a: V, b: V) -> V { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
fn vec3(j: &Json) -> V { let a = j.arr(); [a[0].as_f64(), a[1].as_f64(), a[2].as_f64()] }

/// An analytic surface an operand's boundary lies on.
#[derive(Clone, Debug)]
enum Surf {
    Plane { n: V, d: f64 },
    Cylinder { p: V, a: V, r: f64 },
    Cone { apex: V, a: V, half: f64 },
    Sphere { c: V, r: f64 },
    Torus { c: V, a: V, major: f64, minor: f64 },
    /// A `through:` cap, whose offset is known only after the solve.
    Cap,
}

impl Surf {
    fn kind(&self) -> &'static str {
        match self {
            Surf::Plane { .. } => "plane",
            Surf::Cylinder { .. } => "cylinder",
            Surf::Cone { .. } => "cone",
            Surf::Sphere { .. } => "sphere",
            Surf::Torus { .. } => "torus",
            Surf::Cap => "cap",
        }
    }
    fn moved(&self, m: &[f64]) -> Surf {
        let v = |x: V| [m[0] * x[0] + m[1] * x[1] + m[2] * x[2], m[4] * x[0] + m[5] * x[1] + m[6] * x[2],
            m[8] * x[0] + m[9] * x[1] + m[10] * x[2]];
        let p = |x: V| add(v(x), [m[3], m[7], m[11]]);
        match *self {
            Surf::Plane { n, d } => { let n2 = v(n); Surf::Plane { n: n2, d: d + dot(n2, [m[3], m[7], m[11]]) } }
            Surf::Cylinder { p: q, a, r } => Surf::Cylinder { p: p(q), a: v(a), r },
            Surf::Cone { apex, a, half } => Surf::Cone { apex: p(apex), a: v(a), half },
            Surf::Sphere { c, r } => Surf::Sphere { c: p(c), r },
            Surf::Torus { c, a, major, minor } => Surf::Torus { c: p(c), a: v(a), major, minor },
            Surf::Cap => Surf::Cap,
        }
    }
    /// The same surface, to `tol` (mm).
    fn same(&self, other: &Surf, tol: f64) -> bool {
        let parallel = |a: V, b: V| norm(cross(unit(a), unit(b))) < 1e-9;
        let on_axis = |p: V, q: V, a: V| norm(cross(sub(q, p), unit(a))) < tol;
        match (self, other) {
            (Surf::Plane { n, d }, Surf::Plane { n: m, d: e }) =>
                parallel(*n, *m) && (d - e * dot(*n, *m).signum()).abs() < tol,
            (Surf::Cylinder { p, a, r }, Surf::Cylinder { p: q, a: b, r: s }) =>
                parallel(*a, *b) && on_axis(*p, *q, *a) && (r - s).abs() < tol,
            (Surf::Cone { apex, a, half }, Surf::Cone { apex: b0, a: b, half: h }) =>
                parallel(*a, *b) && norm(sub(*apex, *b0)) < tol && (half - h).abs() < 1e-9,
            (Surf::Sphere { c, r }, Surf::Sphere { c: d, r: s }) => norm(sub(*c, *d)) < tol && (r - s).abs() < tol,
            (Surf::Torus { c, a, major, minor }, Surf::Torus { c: d, a: b, major: m2, minor: n2 }) =>
                parallel(*a, *b) && norm(sub(*c, *d)) < tol && (major - m2).abs() < tol && (minor - n2).abs() < tol,
            _ => false,
        }
    }
}

/// The surfaces of a profile swept along its normal (`from`/`to` known or not).
fn prism(profile: &Json, extent: Option<(f64, f64)>, out: &mut Vec<Surf>, edges: &mut BTreeMap<String, usize>) {
    let n = unit(vec3(profile.get("normal").unwrap()));
    let o = vec3(profile.get("origin").unwrap());
    for l in profile.get("loops").unwrap().arr() {
        for e in l.arr() {
            match e.get("kind").unwrap().as_str() {
                "line" => {
                    *edges.entry("line".into()).or_default() += 1;
                    let (s, t) = (vec3(e.get("start").unwrap()), vec3(e.get("end").unwrap()));
                    let side = unit(cross(sub(t, s), n));
                    out.push(Surf::Plane { n: side, d: dot(side, s) });
                }
                "circle" => {
                    *edges.entry(if e.get("angles").is_some() { "arc" } else { "circle" }.into()).or_default() += 1;
                    out.push(Surf::Cylinder { p: vec3(e.get("center").unwrap()), a: n,
                        r: e.get("radius").unwrap().as_f64() });
                }
                k => panic!("edge {k}"),
            }
        }
    }
    match extent {
        Some((from, to)) => for h in [from, to] { out.push(Surf::Plane { n, d: dot(n, add(o, scale(n, h))) }) },
        None => { out.push(Surf::Cap); out.push(Surf::Cap) }
    }
}

/// The surfaces of a profile turned about an axis in its plane.
fn revolve(node: &Json, out: &mut Vec<Surf>, edges: &mut BTreeMap<String, usize>) {
    let profile = node.get("profile").unwrap();
    let o = vec3(node.get("origin").unwrap());
    let a = unit(vec3(node.get("axis").unwrap()));
    let angle = node.get("angle").unwrap().as_f64();
    let rz = |p: V| { let z = dot(sub(p, o), a); (norm(sub(sub(p, o), scale(a, z))), z) };
    const EPS: f64 = 1e-9;
    for l in profile.get("loops").unwrap().arr() {
        for e in l.arr() {
            match e.get("kind").unwrap().as_str() {
                "line" => {
                    let ((r0, z0), (r1, z1)) = (rz(vec3(e.get("start").unwrap())), rz(vec3(e.get("end").unwrap())));
                    if (r0 - r1).abs() < EPS && r0 < EPS {
                        *edges.entry("line on the axis".into()).or_default() += 1;
                    } else if (z0 - z1).abs() < EPS {
                        *edges.entry("line square to the axis".into()).or_default() += 1;
                        out.push(Surf::Plane { n: a, d: dot(a, o) + z0 });
                    } else if (r0 - r1).abs() < EPS {
                        *edges.entry("line along the axis".into()).or_default() += 1;
                        out.push(Surf::Cylinder { p: o, a, r: r0 });
                    } else {
                        *edges.entry("line slanted to the axis".into()).or_default() += 1;
                        let zapex = z0 - r0 * (z1 - z0) / (r1 - r0);
                        out.push(Surf::Cone { apex: add(o, scale(a, zapex)), a,
                            half: ((r1 - r0) / (z1 - z0)).abs().atan() });
                    }
                }
                "circle" => {
                    let c = vec3(e.get("center").unwrap());
                    let (rc, zc) = rz(c);
                    let r = e.get("radius").unwrap().as_f64();
                    if rc < EPS {
                        *edges.entry("arc centred on the axis".into()).or_default() += 1;
                        out.push(Surf::Sphere { c, r });
                    } else {
                        *edges.entry("arc off the axis".into()).or_default() += 1;
                        out.push(Surf::Torus { c: add(o, scale(a, zc)), a, major: rc, minor: r });
                    }
                }
                k => panic!("edge {k}"),
            }
        }
    }
    if angle.abs() < std::f64::consts::TAU - 1e-12 {
        *edges.entry("partial revolution".into()).or_default() += 1;
        out.push(Surf::Cap);
        out.push(Surf::Cap);
    }
}

#[test]
#[ignore]
fn brep_census() {
    let mut refusals: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut nodes: BTreeMap<String, usize> = BTreeMap::new();
    let mut edges: BTreeMap<String, usize> = BTreeMap::new();
    let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
    let mut pairs: BTreeMap<String, usize> = BTreeMap::new();
    let mut shared: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let (mut built, mut objects, mut docs) = (0, 0, 0);
    for (name, e) in fixtures::examples() {
        let Some(e) = e else { continue };
        if !e.ok() { continue; }
        let mut solved = e.sketch.clone();
        gcs_core::solve::solve(&mut solved, gcs_core::solve::SolveOpts::default());
        let sk: &Sketch = &solved;
        let objs = gcs_core::overview::objects(sk);
        if objs.is_empty() { continue; }
        docs += 1;
        for root in objs {
            objects += 1;
            let solid = sk.solids[root].name.clone();
            let recipe = match gcs_core::solid::cad::recipe(sk, root) {
                Ok(r) => r,
                Err(err) => {
                    let key = err.split('`').enumerate().map(|(i, s)| if i % 2 == 1 { "…" } else { s })
                        .collect::<Vec<_>>().join("`");
                    refusals.entry(key).or_default().push(format!("{name}: {solid}"));
                    continue;
                }
            };
            built += 1;
            // every node's surfaces, operands' by id
            let mut surfs: BTreeMap<i64, Vec<Surf>> = BTreeMap::new();
            for node in recipe.get("nodes").unwrap().arr() {
                let id = node.get("id").unwrap().as_i64();
                let kind = node.get("kind").unwrap().as_str().to_string();
                *nodes.entry(kind.clone()).or_default() += 1;
                let mut out = Vec::new();
                match kind.as_str() {
                    "prism" => prism(node.get("profile").unwrap(), Some((node.get("from").unwrap().as_f64(),
                        node.get("to").unwrap().as_f64())), &mut out, &mut edges),
                    "through" => prism(node.get("profile").unwrap(), None, &mut out, &mut edges),
                    "revolve" => revolve(node, &mut out, &mut edges),
                    "placed" => {
                        let m: Vec<f64> = node.get("matrix").unwrap().arr().iter().map(Json::as_f64).collect();
                        out = surfs[&node.get("source").unwrap().as_i64()].iter().map(|s| s.moved(&m)).collect();
                    }
                    "body" => {
                        let stock = node.get("stock").unwrap().as_i64();
                        out = surfs[&stock].clone();
                        let mut operands = vec![("stock", stock)];
                        for op in ["on", "cut", "bound"] {
                            for x in node.get(op).unwrap().arr() { operands.push((op, x.as_i64())); }
                        }
                        for (i, &(op, x)) in operands.iter().enumerate().skip(1) {
                            // the operand's surfaces against everything before it
                            let mine = &surfs[&x];
                            let mut ks: Vec<&str> = mine.iter().map(Surf::kind).collect();
                            ks.sort(); ks.dedup();
                            let mut before: Vec<&Surf> = Vec::new();
                            for &(_, y) in &operands[..i] { before.extend(surfs[&y].iter()); }
                            let mut bk: Vec<&str> = before.iter().map(|s| s.kind()).collect();
                            bk.sort(); bk.dedup();
                            for a in &bk { for b in &ks {
                                let (p, q) = if a <= b { (a, b) } else { (b, a) };
                                *pairs.entry(format!("{p} × {q}")).or_default() += 1;
                            } }
                            for s in mine {
                                if let Some(t) = before.iter().find(|t| s.same(t, 1e-6)) {
                                    let _ = t;
                                    shared.entry(format!("{op}: {}", s.kind())).or_default()
                                        .push(format!("{name}: {}", node.get("name").unwrap().as_str()));
                                }
                            }
                            out.extend(mine.iter().cloned());
                        }
                    }
                    k => panic!("node {k}"),
                }
                for s in &out { if kind != "body" && kind != "placed" { *kinds.entry(s.kind().into()).or_default() += 1; } }
                surfs.insert(id, out);
            }
        }
    }
    println!("{docs} documents with objects, {objects} objects, {built} with a CAD recipe\n");
    println!("refused:");
    for (k, v) in &refusals { println!("  {} × {k}\n      {}", v.len(), v.join("; ")); }
    println!("\nrecipe nodes: {nodes:?}");
    println!("profile edges: {edges:?}");
    println!("primitive surfaces: {kinds:?}");
    println!("\nsurface kinds meeting in a Boolean (operand against what came before, by body): {pairs:?}");
    println!("\nshared surfaces (an operand's surface already among what came before):");
    for (k, v) in &shared {
        let mut v = v.clone(); v.dedup();
        println!("  {} × {k}: {}", v.len(), v.join("; "));
    }
}

/// A tool: one corpus body built by the Rust kernel a Boolean at a time, the first result that
/// fails its check (or the Boolean that refuses) printed face by face.
/// `BREP_FILE=vtwin/components/disc.sv BREP_SOLID=disc.body cargo test brep_body_debug -- --ignored --nocapture`
#[test]
#[ignore]
fn brep_body_debug() {
    use gcs_core::brep::boolean::{boolean,Op};
    use gcs_core::brep::topo::{Brep,EdgeCurve};
    let (file,solid) = (std::env::var("BREP_FILE").unwrap(),std::env::var("BREP_SOLID").unwrap());
    let (_,e) = fixtures::examples().into_iter().find(|(n,_)| *n == file).unwrap();
    let mut sk = e.unwrap().sketch;
    gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
    let root = (0..sk.solids.len()).find(|&i| sk.solids[i].name == solid).unwrap();
    let r = gcs_core::solid::cad::recipe(&sk,root).unwrap();
    let dump = |b: &Brep| {
        for (i,f) in b.faces.iter().enumerate() {
            eprintln!("  face {i} {} rev {} `{}`: {:?}",f.surface.kind(),f.reversed,f.name,f.loops.iter().map(|l| l.iter()
                .map(|c| format!("{}{}",if c.reversed { "-" } else { "+" },c.edge)).collect::<Vec<_>>()).collect::<Vec<_>>());
        }
        for (i,e) in b.edges.iter().enumerate() {
            let kind = match &e.curve { EdgeCurve::Curve(c) => c.kind(),_ => "pole" };
            eprintln!("  edge {i}: {kind} {:?} from {:?} to {:?}",e.t,b.vertices[e.v[0] as usize].p,b.vertices[e.v[1] as usize].p);
        }
    };
    // the solid meshed, timed, with any triangle of no area named
    let report = |solid: &Brep| {
        if std::env::var_os("BREP_DUMP").is_some() { eprintln!("result:"); dump(solid); }
        let started = std::time::Instant::now();
        match gcs_core::brep::mesh::mesh(solid,0.01,0.2) {
            Ok(m) => {
                eprintln!("meshed: {} triangles, sag {} ({:?})",m.tris.len(),m.sag,started.elapsed());
                for t in &m.tris {
                    let [a,b,c] = t.map(|i| m.pts[i as usize]);
                    // as an STL writes it, in single precision
                    let f = |p: [f64;3]| p.map(|x| x as f32 as f64);
                    if gcs_core::space::triangle_normal(f(a),f(b),f(c)).is_none() { eprintln!("  a triangle of no area (in f32): {a:?} {b:?} {c:?}"); }
                }
            }
            Err(err) => {
                eprintln!("NOT MESHED: {err}");
                dump(solid);
                for (i,f) in solid.faces.iter().enumerate() {
                    eprintln!("  face {i} loops in parameters: {:?}",f.loops.iter().map(|l| l.iter().map(|c| solid.uv_ends(f,c)[0]).collect::<Vec<_>>()).collect::<Vec<_>>());
                }
            }
        }
    };
    let mut built: std::collections::BTreeMap<i64,Brep> = Default::default();
    let names: std::collections::BTreeMap<i64,String> = r.get("nodes").unwrap().arr().iter()
        .map(|n| (n.get("id").unwrap().as_i64(),n.get("name").unwrap().as_str().to_string())).collect();
    for n in r.get("nodes").unwrap().arr() {
        let id = n.get("id").unwrap().as_i64();
        if n.get("kind").unwrap().as_str() != "body" {
            let b = gcs_core::brep::recipe::node(n,&built).unwrap();
            if id == r.get("root").unwrap().as_i64() { report(&b); }
            built.insert(id,b);
            continue
        }
        let mut solid = built[&n.get("stock").unwrap().as_i64()].clone();
        for (key,op) in [("on",Op::Union),("cut",Op::Cut),("bound",Op::Common)] {
            for x in n.get(key).unwrap().arr() {
                let operand = &built[&x.as_i64()];
                eprintln!("{} {key} `{}`",names[&id],names[&x.as_i64()]);
                let tol = 1e-9*solid.size().max(operand.size());
                let next = match boolean(&solid,operand,op,tol) {
                    Ok(b) => b,
                    Err(err) => { eprintln!("REFUSED: {err}\nstock:"); dump(&solid); eprintln!("operand:"); dump(operand); return }
                };
                if let Err(err) = next.check(10.*tol) {
                    eprintln!("INVALID: {err}\nstock:"); dump(&solid); eprintln!("operand:"); dump(operand); eprintln!("result:"); dump(&next); return
                }
                solid = next;
            }
        }
        report(&solid);
        built.insert(id,solid);
    }
    eprintln!("built");
}

/// `solid_tooth`: one involute tooth, its flanks stretches of `gear.Flank`'s involutes between the
/// root circle and the tip. An involute of base radius `Rb` has `x y' − y x' = Rb² u²`, so each
/// flank encloses `Rb² (u1³ − u0³) / 6` about the centre (Green's theorem), and the crown and the
/// line across the root their cross products. The exact kernel reads each flank as the B-spline
/// fitted within `FIT_MM`, which bounds its error by the flanks' length times the depth.
#[test]
fn an_involute_tooth_is_its_closed_form() {
    let (_,e) = fixtures::examples().into_iter().find(|(n,_)| n == "solid_tooth.sv").unwrap();
    let mut sk = e.unwrap().sketch;
    gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
    let root = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "tooth").unwrap();
    let b = gcs_core::brep::recipe::build(&gcs_core::solid::cad::recipe(&sk,root).unwrap()).unwrap();
    b.check(1e-9).unwrap();
    let (n,m,phi,depth) = (40.,2.,25f64.to_radians(),5.);
    let (r,rb) = (m*n/2.,m*n/2.*phi.cos());
    let half = (90./n).to_radians()+phi.tan()-phi;
    let (u0,u1) = (((r-1.25*m)/rb).powi(2)-1.,((r+m)/rb).powi(2)-1.);
    let (u0,u1) = (u0.sqrt(),u1.sqrt());
    let inv = |ph: f64,u: f64| [rb*((u+ph).cos()+u*(u+ph).sin()),rb*((u+ph).sin()-u*(u+ph).cos())];
    let cross = |a: [f64;2],b: [f64;2]| (a[0]*b[1]-a[1]*b[0])/2.;
    let area = 2.*rb*rb*(u1.powi(3)-u0.powi(3))/6.+cross(inv(-half,u1),inv(half,-u1))+cross(inv(half,-u0),inv(-half,u0));
    let flank = rb*(u1*u1-u0*u0)/2.;
    let v = gcs_core::brep::props::volume(&b);
    assert!((v-area.abs()*depth).abs() <= 2.*flank*depth*gcs_core::solid::cad::FIT_MM,"{v} against {}",area.abs()*depth);
    // and the faceted kernel, to its faceting
    let facets = sk.evaluated_solid(root,gcs_core::solid::ApproximationPolicy::Mesh).unwrap().volume();
    assert!((facets-v).abs() <= 5e-3*v,"{facets} against {v}");
}

/// Phase 1's tool: a native kernel's shape dumped as JSON (`SOLVENT_BREP_DUMP`), read into the
/// core's B-rep, checked, measured and meshed. `BREP_JSON=path cargo test … brep_json_debug -- --ignored --nocapture`;
/// `BREP_AGAINST=other` lists the faces whose fluxes differ most from the other dump's nearest
/// (OCCT's reading of a STEP, `PATH.read`), `BREP_SHIFT=d` measures it moved (which a closed
/// boundary's volume does not see), `BREP_BAR=mm` meshes it to that sag.
#[test]
#[ignore]
fn brep_json_debug() {
    let path = std::env::var("BREP_JSON").unwrap();
    let started = std::time::Instant::now();
    let b = gcs_core::brep::json::read(&std::fs::read_to_string(&path).unwrap()).unwrap();
    eprintln!("read {} faces, {} edges, {} vertices ({:?})",b.faces.len(),b.edges.len(),b.vertices.len(),started.elapsed());
    let worst = b.edges.iter().map(|e| e.tol).fold(0.,f64::max);
    eprintln!("edge tolerances measured: worst {worst:e} mm, {} over 1e-5 mm",b.edges.iter().filter(|e| e.tol > 1e-5).count());
    match b.check(1e-7) { Ok(()) => eprintln!("checked"), Err(e) => eprintln!("CHECK: {e}") }
    let started = std::time::Instant::now();
    eprintln!("volume {:.9} mm³ ({:?})",gcs_core::brep::props::volume(&b),started.elapsed());
    if let Ok(other) = std::env::var("BREP_AGAINST") {
        // each face's flux against the other B-rep's face nearest it (by a point of its first edge)
        let o = gcs_core::brep::json::read(&std::fs::read_to_string(&other).unwrap()).unwrap();
        let (fa,fb) = (gcs_core::brep::props::fluxes(&b),gcs_core::brep::props::fluxes(&o));
        let at = |b: &gcs_core::brep::topo::Brep,f: usize| -> Vec<[f64;3]> {
            b.faces[f].loops.iter().flatten().map(|c| { let e = &b.edges[c.edge as usize]; e.point((e.t[0]+e.t[1])/2.,&b.vertices) }).collect()
        };
        let mut worst = Vec::new();
        for i in 0..b.faces.len() {
            let pi = at(&b,i);
            let j = (0..o.faces.len()).min_by(|&j,&k| {
                let d = |j: usize| pi.iter().map(|p| at(&o,j).iter().map(|q| gcs_core::space::distance(*p,*q)).fold(f64::INFINITY,f64::min)).sum::<f64>();
                d(j).total_cmp(&d(k))
            }).unwrap();
            worst.push(((fa[i]-fb[j]).abs(),i,j,b.faces[i].surface.kind(),o.faces[j].surface.kind(),fa[i],fb[j],b.faces[i].reversed,o.faces[j].reversed));
        }
        worst.sort_by(|a,b| b.0.total_cmp(&a.0));
        for w in worst.iter().take(12) { eprintln!("flux {w:?}"); }
    }
    if let Ok(n) = std::env::var("BREP_PATTERN") {
        // the sector turned into `n` copies about its sphere's axis
        let f = b.faces.iter().find(|f| f.surface.kind() == "sphere").expect("a sphere gives the axis").surface.frame();
        let started = std::time::Instant::now();
        match gcs_core::brep::pattern::pattern(&b,f.o,f.z,n.parse().unwrap(),1e-4) {
            Err(e) => eprintln!("PATTERN: {e}"),
            Ok(p) => {
                let w = &p.solid;
                let kinds = w.faces.iter().fold(std::collections::BTreeMap::new(),|mut m,f| { *m.entry(f.surface.kind()).or_insert(0) += 1; m });
                eprintln!("pattern: sides {:?}, step {:.6}°, vertices matched within {:e}; {} faces {kinds:?}, {} edges, {} vertices ({:?})",
                    p.sides,p.step.to_degrees(),p.matched,w.faces.len(),w.edges.len(),w.vertices.len(),started.elapsed());
                eprintln!("pattern: edges within {:e}",w.edges.iter().map(|e| e.tol).fold(0.,f64::max));
                match w.check(1e-7) { Ok(()) => eprintln!("pattern: checked"),Err(e) => eprintln!("PATTERN CHECK: {e}") }
                if std::env::var("PATTERN_FACE").is_ok() {
                    let f = &w.faces[std::env::var("PATTERN_FACE").unwrap().parse::<usize>().unwrap()];
                    eprintln!("face {} reversed {} loops {:?}",f.surface.kind(),f.reversed,f.loops.iter().map(|l| l.len()).collect::<Vec<_>>());
                    for c in f.loops.iter().flatten().take(14) {
                        let e = &w.edges[c.edge as usize];
                        let (a,z) = (c.pcurve.at(e.t[0],e,&f.surface,&w.vertices),c.pcurve.at(e.t[1],e,&f.surface,&w.vertices));
                        eprintln!("  edge {} rev {} v {:?} uv {:?} -> {:?}",c.edge,c.reversed,e.v,a,z);
                    }
                }
                let (v,one) = (gcs_core::brep::props::volume(w),gcs_core::brep::props::volume(&b));
                eprintln!("pattern: volume {v:.9} against {n} sectors' {:.9} ({:e})",one*n.parse::<f64>().unwrap(),(v/(one*n.parse::<f64>().unwrap())-1.));
                match gcs_core::brep::mesh::mesh(w,0.01,0.2) {
                    Ok(m) => eprintln!("pattern: meshed whole {} triangles, sag {:e}, {} turned",m.tris.len(),m.sag,m.turned),
                    Err(e) => eprintln!("PATTERN MESH: {e}"),
                }
                let started = std::time::Instant::now();
                match p.mesh(0.01,0.2) {
                    Ok(m) => {
                        let stl = gcs_core::mesh::stl_of(&m.triangles(),"pattern");
                        let shells = gcs_core::mesh::stl_shells(&stl);
                        // the mesh's volume, by the divergence theorem over its triangles
                        let vol: f64 = m.tris.iter().map(|t| { let [a,b,c] = t.map(|i| m.pts[i as usize]);
                            gcs_core::space::dot(a,gcs_core::space::cross(b,c)) }).sum::<f64>()/6.;
                        eprintln!("pattern: meshed as copies {} triangles, sag {:e}, {} turned, shells {:?}, volume {vol:.6} ({:?})",
                            m.tris.len(),m.sag,m.turned,shells.map(|_| "ok"),started.elapsed());
                    }
                    Err(e) => eprintln!("PATTERN COPIES MESH: {e}"),
                }
            }
        }
    }
    if let Ok(d) = std::env::var("BREP_SHIFT") {
        let d: f64 = d.parse().unwrap();
        for m in [gcs_core::brep::geom::Rigid {t:[d,0.,0.],..gcs_core::brep::geom::Rigid::identity()},gcs_core::brep::geom::Rigid {t:[0.,d,d],..gcs_core::brep::geom::Rigid::identity()}] {
            eprintln!("moved: volume {:.9} mm³",gcs_core::brep::props::volume(&b.moved(&m)));
        }
    }
    let started = std::time::Instant::now();
    match gcs_core::brep::mesh::mesh(&b,std::env::var("BREP_BAR").map_or(0.01,|x| x.parse().unwrap()),0.2) {
        Ok(m) => eprintln!("meshed: {} triangles, sag {:e}, {} turned ({:?})",m.tris.len(),m.sag,m.turned,started.elapsed()),
        Err(e) => eprintln!("NOT MESHED: {e}"),
    }
}

/// Phase 3's tool: a member's split as the native export dumps it (`SOLVENT_SPLIT_DUMP=DIR`) done by
/// the core — the blank by its two sides, the wedge by each sheet in turn — against the kernel's
/// cells, by volume and faces. `BREP_SPLIT=DIR cargo test … brep_split_debug -- --ignored --nocapture`.
fn occt_first(dir: &str) -> gcs_core::brep::topo::Brep {
    use gcs_core::brep::{json,props::volume};
    (0..).map_while(|k| std::fs::read_to_string(format!("{dir}/cell{k}.json")).ok()).map(|t| json::read(&t).unwrap())
        .max_by(|a,b| volume(a).total_cmp(&volume(b))).unwrap()
}

#[test]
#[ignore]
fn brep_split_debug() {
    use gcs_core::brep::{boolean::split,json,props::volume,topo::Brep};
    let dir = std::env::var("BREP_SPLIT").unwrap();
    let tol: f64 = std::env::var("BREP_TOL").map_or(1e-6,|x| x.parse().unwrap());
    let read = |name: &str| json::read(&std::fs::read_to_string(format!("{dir}/{name}.json")).unwrap()).unwrap();
    let kinds = |b: &Brep| b.faces.iter().fold(std::collections::BTreeMap::new(),|mut m,f| { *m.entry(f.surface.kind()).or_insert(0) += 1; m });
    let said = |what: &str,cells: &[Brep],started: std::time::Instant| {
        let mut v: Vec<(f64,String)> = cells.iter().map(|c| (volume(c),format!("{:?} {}",kinds(c),match c.check(1e-6) { Ok(()) => "checked".into(),Err(e) => e }))).collect();
        v.sort_by(|a,b| b.0.total_cmp(&a.0));
        eprintln!("{what}: {} cells ({:?})",cells.len(),started.elapsed());
        for (x,k) in v { eprintln!("  {x:.9} {k}"); }
    };
    let blank = read("blank");
    let sides = read("side").joined(&read("other"));
    let started = std::time::Instant::now();
    let halves = split(&blank,&sides,tol).unwrap_or_else(|e| panic!("the sides: {e}"));
    said("the blank by its sides",&halves,started);
    let theirs = read("wedge");
    eprintln!("the kernel's wedge: {:.9} {:?}",volume(&theirs),kinds(&theirs));
    let wedge = halves.into_iter().min_by(|a,b| volume(a).total_cmp(&volume(b))).unwrap();
    let mut cells = vec![wedge];
    let only: Option<usize> = std::env::var("BREP_SHEETS").ok().map(|x| x.parse().unwrap());
    for k in 0.. {
        if only.is_some_and(|n| k >= n) { break }
        let Ok(text) = std::fs::read_to_string(format!("{dir}/sheet{k}.json")) else { break };
        let sheet = json::read(&text).unwrap();
        let started = std::time::Instant::now();
        let mut next = Vec::new();
        for c in &cells { next.extend(split(c,&sheet,tol).unwrap_or_else(|e| panic!("sheet {k}: {e}"))); }
        cells = next;
        said(&format!("and by sheet {k}"),&cells,started);
    }
    // the largest cell's short edges and near vertices
    if std::env::var_os("BREP_SHORT").is_some() {
        let c = cells.iter().max_by(|a,b| volume(a).total_cmp(&volume(b))).unwrap();
        eprintln!("the largest cell: {} faces, {} edges, {} vertices",c.faces.len(),c.edges.len(),c.vertices.len());
        let used: std::collections::BTreeSet<u32> = c.faces.iter().flat_map(|f| f.loops.iter().flatten().map(|co| co.edge)).collect();
        for &i in &used {
            let e = &c.edges[i as usize];
            let pts: Vec<[f64;3]> = (0..=8).map(|k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/8.,&c.vertices)).collect();
            let len: f64 = pts.windows(2).map(|w| ((w[1][0]-w[0][0]).powi(2)+(w[1][1]-w[0][1]).powi(2)+(w[1][2]-w[0][2]).powi(2)).sqrt()).sum();
            if len < 1e-2 { eprintln!("  edge {i}: {len:.3e} long, vertices {:?} at {:?}",e.v,pts[0]); }
        }
        for (who,b) in [("ours",c),("theirs",&occt_first(&dir))] {
            let m = gcs_core::brep::mesh::mesh(b,0.01,0.2).unwrap();
            let mut near: std::collections::BTreeMap<u32,usize> = std::collections::BTreeMap::new();
            let samples: Vec<(u32,[f64;3])> = b.faces.iter().flat_map(|f| f.loops.iter().flatten().map(|co| co.edge)).collect::<std::collections::BTreeSet<_>>()
                .into_iter().flat_map(|i| { let e = &b.edges[i as usize]; (0..=64).map(move |k| (i,e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/64.,&b.vertices))) }).collect();
            let mut tiny = 0;
            for t in &m.tris {
                let [a,bb,cc] = t.map(|i| m.pts[i as usize]);
                let u = [bb[0]-a[0],bb[1]-a[1],bb[2]-a[2]]; let v = [cc[0]-a[0],cc[1]-a[1],cc[2]-a[2]];
                let n = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
                if (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt()/2. >= 1e-6 { continue }
                tiny += 1;
                if who == "ours" { eprintln!("  tiny: {a:?} {bb:?} {cc:?}"); }
                let g = [(a[0]+bb[0]+cc[0])/3.,(a[1]+bb[1]+cc[1])/3.,(a[2]+bb[2]+cc[2])/3.];
                let best = samples.iter().min_by(|x,y| {
                    let d = |p: [f64;3]| (p[0]-g[0]).powi(2)+(p[1]-g[1]).powi(2)+(p[2]-g[2]).powi(2);
                    d(x.1).total_cmp(&d(y.1)) }).unwrap();
                *near.entry(best.0).or_insert(0) += 1;
            }
            eprintln!("{who}: {} triangles, {tiny} tiny, sag {:.3e}; nearest edges {near:?}",m.tris.len(),m.sag);
            for (&e,_) in &near { let e = &b.edges[e as usize]; eprintln!("  edge {:?} from {:?} to {:?}: {:?}",e.v,b.vertices[e.v[0] as usize].p,b.vertices[e.v[1] as usize].p,match &e.curve { gcs_core::brep::topo::EdgeCurve::Curve(c) => c.kind(),_ => "pole" }); }
        }
        let mut degree = vec![0;c.vertices.len()];
        for &i in &used { for v in c.edges[i as usize].v { degree[v as usize] += 1; } }
        for (v,d) in degree.iter().enumerate() { if *d != 3 { eprintln!("  vertex {v} of degree {d} at {:?}",c.vertices[v].p); } }
        let theirs = occt_first(&dir);
        let used_t: std::collections::BTreeSet<u32> = theirs.faces.iter().flat_map(|f| f.loops.iter().flatten().map(|co| co.edge)).collect();
        let mut degree = vec![0;theirs.vertices.len()];
        for &i in &used_t { for v in theirs.edges[i as usize].v { degree[v as usize] += 1; } }
        eprintln!("the kernel's: {} faces, {} edges, {} vertices",theirs.faces.len(),used_t.len(),degree.iter().filter(|&&d| d > 0).count());
        for (v,d) in degree.iter().enumerate() { if *d != 3 && *d > 0 { eprintln!("  their vertex {v} of degree {d} at {:?}",theirs.vertices[v].p); } }
        for i in 0..c.vertices.len() { for j in i+1..c.vertices.len() {
            let (p,q) = (c.vertices[i].p,c.vertices[j].p);
            let d = ((p[0]-q[0]).powi(2)+(p[1]-q[1]).powi(2)+(p[2]-q[2]).powi(2)).sqrt();
            if d < 1e-3 { eprintln!("  vertices {i} and {j}: {d:.3e} apart at {p:?}"); }
        } }
    }
    let mut occt: Vec<Brep> = (0..).map_while(|k| std::fs::read_to_string(format!("{dir}/cell{k}.json")).ok()).map(|t| json::read(&t).unwrap()).collect();
    occt.sort_by(|a,b| volume(b).total_cmp(&volume(a)));
    eprintln!("the kernel's cells:");
    for c in &occt { eprintln!("  {:.9} {:?}",volume(c),kinds(c)); }
    // points given, read against the kernel's cells: by our classifier and by a mesh's winding number
    if let Ok(text) = std::env::var("BREP_POINTS") {
        let given: Vec<[f64;3]> = text.split(';').map(|p| { let v: Vec<f64> = p.split(',').map(|x| x.trim().parse().unwrap()).collect(); [v[0],v[1],v[2]] }).collect();
        // each point and its six neighbours 0.01 off along the axes
        let pts: Vec<[f64;3]> = given.iter().flat_map(|p| (0..7).map(move |k| { let mut q = *p; if k > 0 { q[(k-1)/2] += if k % 2 == 0 { 0.01 } else { -0.01 }; } q })).collect();
        for (k,c) in occt.iter().enumerate() {
            let located = gcs_core::brep::query::Located::new(c,tol);
            let m = gcs_core::brep::mesh::mesh(c,0.002,0.1).unwrap();
            for p in &pts {
                let mut w = 0.;
                for t in &m.tris {
                    let [a,b,d] = t.map(|i| { let q = m.pts[i as usize]; [q[0]-p[0],q[1]-p[1],q[2]-p[2]] });
                    let n = |v: [f64;3]| (v[0]*v[0]+v[1]*v[1]+v[2]*v[2]).sqrt();
                    let (la,lb,ld) = (n(a),n(b),n(d));
                    let det = a[0]*(b[1]*d[2]-b[2]*d[1])-a[1]*(b[0]*d[2]-b[2]*d[0])+a[2]*(b[0]*d[1]-b[1]*d[0]);
                    let dot = |x: [f64;3],y: [f64;3]| x[0]*y[0]+x[1]*y[1]+x[2]*y[2];
                    w += 2.*det.atan2(la*lb*ld+dot(a,b)*ld+dot(b,d)*la+dot(d,a)*lb);
                }
                eprintln!("  cell {k} ({:.3}): {p:?} {:?}, winding {:.3}",volume(c),located.solid_place(*p),w/(4.*std::f64::consts::PI));
            }
        }
    }
    // the largest cells face by face: each face's flux and area against its nearest in the other
    cells.sort_by(|a,b| volume(b).total_cmp(&volume(a)));
    if let (Some(ours),Some(theirs)) = (cells.first(),occt.first()) {
        let (fa,fb) = (gcs_core::brep::props::fluxes(ours),gcs_core::brep::props::fluxes(theirs));
        let middle = |b: &Brep,f: usize| -> [f64;3] {
            let pts: Vec<[f64;3]> = b.faces[f].loops.iter().flatten().map(|c| { let e = &b.edges[c.edge as usize]; e.point((e.t[0]+e.t[1])/2.,&b.vertices) }).collect();
            let n = pts.len() as f64;
            [pts.iter().map(|p| p[0]).sum::<f64>()/n,pts.iter().map(|p| p[1]).sum::<f64>()/n,pts.iter().map(|p| p[2]).sum::<f64>()/n]
        };
        let mut measured = ours.clone();
        json::measure(&mut measured);
        if let Ok(k) = std::env::var("BREP_FACE") {
            let f = &measured.faces[k.parse::<usize>().unwrap()];
            for c in f.loops.iter().flatten() {
                let e = &measured.edges[c.edge as usize];
                let kind = match &e.curve { gcs_core::brep::topo::EdgeCurve::Curve(c) => c.kind(),_ => "pole" };
                let pk = match &c.pcurve { gcs_core::brep::topo::Pcurve::Line {..} => "line",gcs_core::brep::topo::Pcurve::Inverse {..} => "inverse",gcs_core::brep::topo::Pcurve::Curve(_) => "curve" };
                eprintln!("    edge {} {kind} pcurve {pk}: within {:e}",c.edge,e.tol);
            }
        }
        for i in 0..ours.faces.len() {
            let mi = middle(ours,i);
            let j = (0..theirs.faces.len()).filter(|&j| theirs.faces[j].surface.kind() == ours.faces[i].surface.kind())
                .min_by(|&x,&y| gcs_core::space::distance(middle(theirs,x),mi).total_cmp(&gcs_core::space::distance(middle(theirs,y),mi))).unwrap();
            if std::env::var("BREP_FACE").is_ok_and(|k| k.parse::<usize>().unwrap() == i) {
                let corners = |b: &Brep,f: usize| -> Vec<[f64;3]> { b.faces[f].loops.iter().flatten().map(|c| {
                    let e = &b.edges[c.edge as usize]; b.vertices[e.v[if c.reversed { 1 } else { 0 }] as usize].p }).collect() };
                let sides = |b: &Brep,f: usize| -> Vec<String> { b.faces[f].loops.iter().flatten().map(|c| {
                    let e = &b.edges[c.edge as usize];
                    let pts: Vec<[f64;3]> = (0..=64).map(|k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/64.,&b.vertices)).collect();
                    let len: f64 = pts.windows(2).map(|w| gcs_core::space::distance(w[0],w[1])).sum();
                    let mid = pts[32];
                    let across: Vec<usize> = (0..b.faces.len()).filter(|&g| g != f && b.faces[g].loops.iter().flatten().any(|u| u.edge == c.edge)).collect();
                    let kind = match &e.curve { gcs_core::brep::topo::EdgeCurve::Curve(c) => c.kind(),_ => "pole" };
                    let off = |g: usize| pts.iter().map(|&p| b.faces[g].surface.implicit(p).abs()).fold(0.,f64::max);
                    let offs: Vec<String> = std::iter::once(f).chain(across.iter().copied()).map(|g| format!("{:.1e}",off(g))).collect();
                    format!("{kind} len {len:.5} mid {:?} across {:?} off {offs:?}",mid.map(|x| (x*1e3).round()/1e3),across.iter().map(|&g| b.faces[g].surface.kind()).collect::<Vec<_>>())
                }).collect() };
                let net = |b: &Brep,f: usize| match &b.faces[f].surface { gcs_core::brep::geom::Surface::BSpline(_,n) => format!("{}x{} first pole {:?}",n.poles.len(),n.poles[0].len(),n.poles[0][0]),s => s.kind().into() };
                eprintln!("    nets: ours {} theirs {}",net(ours,i),net(theirs,j));
                // the area each face's loops enclose in the sheet's parameters, densely sampled
                let uv_area = |b: &Brep,f: usize| -> f64 {
                    let face = &b.faces[f];
                    let mut sum = 0.;
                    for c in face.loops.iter().flatten() {
                        let e = &b.edges[c.edge as usize];
                        let mut ts: Vec<f64> = (0..=4000).map(|k| e.t[0]+(e.t[1]-e.t[0])*k as f64/4000.).collect();
                        if c.reversed { ts.reverse(); }
                        for w in ts.windows(2) {
                            let (p,q) = (c.pcurve.at(w[0],e,&face.surface,&b.vertices),c.pcurve.at(w[1],e,&face.surface,&b.vertices));
                            sum += p[0]*q[1]-p[1]*q[0];
                        }
                    }
                    sum/2.
                };
                eprintln!("    areas in the parameters: ours {:.9} theirs {:.9}",uv_area(ours,i),uv_area(theirs,j));
                // the flux by positions alone: Σ G(middle) Δv, G = ∫ g du from the net's start by 64-point
                // Gauss–Legendre a knot span
                let flux_by_positions = |b: &Brep,f: usize| -> f64 {
                    let face = &b.faces[f];
                    let gcs_core::brep::geom::Surface::BSpline(_,n) = &face.surface else { return f64::NAN };
                    let [[u0,_],_] = n.domain();
                    let g = |u: f64,v: f64| { let (x,su,sv) = n.d1(u,v); gcs_core::space::dot(x,gcs_core::space::cross(su,sv)) };
                    let mut knots: Vec<f64> = n.uknots.clone(); knots.dedup();
                    let big_g = |u: f64,v: f64| -> f64 {
                        let mut cuts = vec![u0]; cuts.extend(knots.iter().copied().filter(|&k| k > u0 && k < u)); cuts.push(u);
                        cuts.windows(2).map(|w| { let (a,z) = (w[0],w[1]); let m = 400;
                            (0..m).map(|k| { let x = a+(z-a)*(k as f64+0.5)/m as f64; g(x,v) }).sum::<f64>()*(z-a)/m as f64 }).sum()
                    };
                    let mut sum = 0.;
                    for c in face.loops.iter().flatten() {
                        let e = &b.edges[c.edge as usize];
                        let mut ts: Vec<f64> = (0..=2000).map(|k| e.t[0]+(e.t[1]-e.t[0])*k as f64/2000.).collect();
                        if c.reversed { ts.reverse(); }
                        let mut part = 0.;
                        for w in ts.windows(2) {
                            let (p,q) = (c.pcurve.at(w[0],e,&face.surface,&b.vertices),c.pcurve.at(w[1],e,&face.surface,&b.vertices));
                            part += big_g((p[0]+q[0])/2.,(p[1]+q[1])/2.)*(q[1]-p[1]);
                        }
                        // the same by derivatives at the steps' middles, as `props` reads it
                        let by_derivative: f64 = ts.windows(2).map(|w| { let m = (w[0]+w[1])/2.;
                            let uv = c.pcurve.at(m,e,&face.surface,&b.vertices);
                            big_g(uv[0],uv[1])*c.pcurve.derivative(m,e,&face.surface,&b.vertices)[1]*(w[1]-w[0]) }).sum();
                        eprintln!("      use of edge {}: by positions {part:.6}, by derivatives {by_derivative:.6}",c.edge);
                        sum += part;
                    }
                    if face.reversed { -sum } else { sum }
                };
                eprintln!("    flux by positions: ours {:.6} theirs {:.6}",flux_by_positions(ours,i),flux_by_positions(theirs,j));
                // each of our sides against the nearest of theirs, densely sampled
                let dense = |b: &Brep,f: usize| -> Vec<Vec<[f64;3]>> { b.faces[f].loops.iter().flatten().map(|c| { let e = &b.edges[c.edge as usize];
                    (0..=2000).map(|k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/2000.,&b.vertices)).collect() }).collect() };
                let (mine,_their) = (dense(ours,i),dense(theirs,j));
                let their_edges: Vec<&gcs_core::brep::topo::Edge> = theirs.faces[j].loops.iter().flatten().map(|c| &theirs.edges[c.edge as usize]).collect();
                for (k,side) in mine.iter().enumerate() {
                    // the nearest of their edges by samples, then each point's foot on its curve
                    let m = side[side.len()/2];
                    let e = their_edges.iter().min_by(|a,b| {
                        let d = |e: &gcs_core::brep::topo::Edge| (0..=200).map(|q| gcs_core::space::distance(e.point(e.t[0]+(e.t[1]-e.t[0])*q as f64/200.,&theirs.vertices),m)).fold(f64::INFINITY,f64::min);
                        d(a).total_cmp(&d(b)) }).unwrap();
                    let gcs_core::brep::topo::EdgeCurve::Curve(c) = &e.curve else { continue };
                    let (mut far,mut at) = (0_f64,0);
                    for (q,&p) in side.iter().enumerate() { let d = gcs_core::space::distance(c.point(c.inverse(p).clamp(e.t[0],e.t[1])),p); if d > far { far = d; at = q; } }
                    let ts: Vec<f64> = side.iter().map(|&p| c.inverse(p).clamp(e.t[0],e.t[1])).collect();
                    let up = ts.windows(2).filter(|w| w[1] > w[0]).count();
                    let down = ts.windows(2).filter(|w| w[1] < w[0]).count();
                    eprintln!("    our side {k}: {far:.2e} from theirs at most, at {:.3} of it; along theirs {up} up {down} down",at as f64/2000.);
                }
                for x in sides(ours,i) { eprintln!("    ours side: {x}"); }
                for x in sides(theirs,j) { eprintln!("    their side: {x}"); }
                eprintln!("    ours:   {:?}",corners(ours,i).iter().map(|p| p.map(|x| (x*1e4).round()/1e4)).collect::<Vec<_>>());
                eprintln!("    theirs: {:?}",corners(theirs,j).iter().map(|p| p.map(|x| (x*1e4).round()/1e4)).collect::<Vec<_>>());
            }
            eprintln!("  face {i} {} ({} uses) flux {:.6} against {:.6} ({} uses): {:+.6}",ours.faces[i].surface.kind(),
                ours.faces[i].loops.iter().map(|l| l.len()).sum::<usize>(),fa[i],fb[j],theirs.faces[j].loops.iter().map(|l| l.len()).sum::<usize>(),fa[i]-fb[j]);
        }
    }
}

#[test]
#[ignore]
fn brep_meridian_debug() {
    // `BREP_MERIDIAN=recipe.json`: the recipe's meridian region by the core, with its pieces
    let text = std::fs::read_to_string(std::env::var("BREP_MERIDIAN").unwrap()).unwrap();
    let recipe = gcs_core::json::parse(&text).unwrap();
    let seam = [1.,0.,0.];
    match gcs_core::brep::recipe::meridian_region(&recipe,seam).unwrap() {
        Ok((p,o,a)) => eprintln!("region about {o:?} along {a:?}: {} loops of {:?} edges",p.loops.len(),p.loops.iter().map(|l| l.len()).collect::<Vec<_>>()),
        Err(e) => eprintln!("not a meridian: {e}"),
    }
}
