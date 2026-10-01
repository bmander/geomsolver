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
