//! **The tool read from its B-rep** (#721 F1, #828 F1a, F1c).
//!
//! Every sweep of the examples has its tool's recipe built into a B-rep and read by the kernel
//! (`Tool::read`): the sweep catalogue's (`rust/examples/solid_sweeps/`) in the fast suite, every
//! example's in the slow tier (`every_examples_tools_read_from_their_boundaries`). The drawing's
//! own reading, the oracle the B-rep reading was held to while the classes switched onto it (F1a,
//! F1b), is retired (F1c); the reading is held to the B-rep it reads:
//! - its **boundary**: every point of a face of the reading on the tool lies on the B-rep's
//!   boundary, and every point of a B-rep face lies on a face of the reading square to its normal;
//! - its **material**: the reading's agrees with the drawing's field of the tool, the semantic
//!   reference, which side of the boundary a point clear of it is on, and reads nought on it;
//! - its **refusals**: a face no reading takes is named with a point on it.
use gcs_core::brep::swept::read::Unread;
use gcs_core::brep::swept::tool::{Tool,ToolSurface};
use gcs_core::brep::query::{Located,Place};
use gcs_core::brep::topo::Brep;
use gcs_core::model::SolidDef;
use gcs_core::solid::cad;
use gcs_core::space::{cross,dot,norm,normalised,sub};
use std::sync::OnceLock;

type V = [f64;3];

/// One sweep of the examples: its tool's B-rep (millimetres) located, the model's millimetres a
/// unit, the tool read from it, with its size, what lengths are read against, and the drawing's
/// field of the tool where it reads one.
struct Case { what: String,brep: Located<'static>,mm: f64,read: Tool,size: f64,field: Option<gcs_core::solid::SpatialField> }

impl Case {
    fn place(&self,p: V) -> Place { self.brep.solid_place(p.map(|x| x*self.mm)) }
    fn b(&self) -> &Brep { &self.brep.b }
}

/// The sweeps read, and those whose B-rep reading refuses, with the B-rep and why.
type Cases = (Vec<Case>,Vec<(String,Brep,f64,Unread)>);

/// The sweep catalogue's (`rust/examples/solid_sweeps/`): a case of each motion family the classes
/// admit, read in a second or two.
fn catalogue() -> &'static Cases {
    static CASES: OnceLock<Cases> = OnceLock::new();
    CASES.get_or_init(|| read_cases(|name| name.starts_with("solid_sweeps/")))
}

/// Every sweep of every example whose tool builds a B-rep, read from it.
fn every() -> Cases { read_cases(|_| true) }

fn read_cases(within: impl Fn(&str) -> bool) -> Cases {
    let (mut cases,mut refused) = (Vec::new(),Vec::new());
    for name in fixtures::example_names().into_iter().filter(|n| within(n)) {
        let Some(e) = fixtures::example(&name) else { continue };
        if !e.sketch.solids.iter().any(|s| matches!(s.def,SolidDef::Swept {..})) { continue }
        let mut sk = e.sketch.clone();
        gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
        for swept in 0..sk.solids.len() {
            let SolidDef::Swept {source,..} = &sk.solids[swept].def else { continue };
            let what = format!("{name}: `{}`",sk.solids[swept].name);
            let Ok(recipe) = cad::recipe(&sk,*source as usize) else { continue };
            let Ok(brep) = gcs_core::brep::recipe::build(&recipe) else { continue };
            let mm = cad::millimetres(&sk);
            match Tool::read(&brep,mm) {
                Ok(read) => {
                    let size = read.faces.iter().filter_map(|f| f.at(0.5,0.5).ok()).map(|p| norm(p.position)).fold(1.,f64::max);
                    let tol = 1e-7*(1.+brep.size());
                    let field = gcs_core::solid::SpatialField::read(&sk,*source as usize,cad::AXIS_TOLERANCE).ok();
                    cases.push(Case {what,brep:Located::owned(brep,tol),mm,read,size,field})
                }
                Err(why) => refused.push((what,brep,mm,why)),
            }
        }
    }
    (cases,refused)
}

fn kind(s: &ToolSurface) -> &'static str {
    match s { ToolSurface::Revolved(_) => "a revolution",ToolSurface::Extruded(_) => "a strip",ToolSurface::Cap(_) => "a cap" }
}

/// The point of face `s` nearest `p`: its distance, its parameters and its unit normal there. A
/// grid over the face's domain, then Gauss–Newton steps held to it.
fn nearest(s: &ToolSurface,p: V) -> Option<(f64,[f64;2],V)> {
    let [[u0,u1],[v0,v1]] = s.domain();
    // a parameter whose two ends are one curve (a revolution's turn, a meridian closed on itself)
    // runs round, wrapped rather than held at an end
    let closed = |a: f64,b: f64,other: f64,u: bool| {
        let (x,y) = if u { (s.at(a,other),s.at(b,other)) } else { (s.at(other,a),s.at(other,b)) };
        matches!((x,y),(Ok(x),Ok(y)) if norm(sub(x.position,y.position)) <= 1e-12*(1.+norm(x.position)))
    };
    let (ru,rv) = (closed(u0,u1,0.5*(v0+v1),true),closed(v0,v1,0.5*(u0+u1),false));
    let hold = |x: f64,a: f64,b: f64,round: bool| if round { gcs_core::brep::geom::around(x,a,b-a) } else { x.clamp(a,b) };
    let at = |u: f64,v: f64| s.at(hold(u,u0,u1,ru),hold(v,v0,v1,rv)).ok();
    let n = 24;
    let mut best: Option<(f64,[f64;2])> = None;
    for i in 0..=n { for j in 0..=n {
        let (u,v) = (u0+(u1-u0)*i as f64/n as f64,v0+(v1-v0)*j as f64/n as f64);
        let Some(x) = at(u,v) else { continue };
        let d = norm(sub(x.position,p));
        if best.is_none_or(|(b,_)| d < b) { best = Some((d,[u,v])) }
    } }
    let (_,[mut u,mut v]) = best?;
    for _ in 0..40 {
        let x = at(u,v)?;
        let r = sub(p,x.position);
        let (a11,a12,a22) = (dot(x.du,x.du),dot(x.du,x.dv),dot(x.dv,x.dv));
        let (b1,b2) = (dot(x.du,r),dot(x.dv,r));
        let det = a11*a22-a12*a12;
        if !(det.abs() > 1e-300) { break }
        let (du,dv) = ((a22*b1-a12*b2)/det,(a11*b2-a12*b1)/det);
        u = hold(u+du,u0,u1,ru);
        v = hold(v+dv,v0,v1,rv);
        if du.abs()+dv.abs() < 1e-15 { break }
    }
    let x = at(u,v)?;
    Some((norm(sub(x.position,p)),[u,v],normalised(cross(x.du,x.dv))?))
}

/// The points of face `s` on a grid inside its domain where `tool` says they are on the tool.
fn on_tool(s: &ToolSurface,tool: &Tool,n: usize) -> Vec<(V,V)> {
    let [[u0,u1],[v0,v1]] = s.domain();
    let mut out = Vec::new();
    for i in 1..n { for j in 1..n {
        let Ok(x) = s.at(u0+(u1-u0)*i as f64/n as f64,v0+(v1-v0)*j as f64/n as f64) else { continue };
        if tool.material().value(x.position).abs() < 1e-6 {
            if let Some(n) = normalised(cross(x.du,x.dv)) { out.push((x.position,n)) }
        }
    } }
    out
}

/// The faces of `tool` that `p` lies on, square to `normal` (either way round).
fn faces_at(tool: &Tool,p: V,normal: V,size: f64) -> Vec<usize> {
    tool.faces.iter().enumerate().filter_map(|(k,f)| {
        let (d,_,n) = nearest(f,p)?;
        (d <= 1e-7*size && norm(cross(n,normal)) <= 1e-5).then_some(k)
    }).collect()
}

/// Every tool the B-rep reads, read, and of each kind; a refusal names a face of the B-rep with a
/// point on it: how many were read.
fn reads_every_tool((cases,refused): &Cases) -> usize {
    eprintln!("{} tools read, {} refused",cases.len(),refused.len());
    for (what,b,mm,why) in refused {
        eprintln!("refused: {what}: {why}");
        let (Unread::FreeForm {face,at,..} | Unread::Meridian {face,at,..}) = why else { continue };
        let f = b.faces.iter().find(|f| gcs_core::brep::recipe::joined(&f.name) == *face)
            .unwrap_or_else(|| panic!("{what}: the refusal names `{face}`, no face of the B-rep"));
        let q = at.map(|x| x*mm);
        assert!(norm(sub(f.surface.point(f.surface.inverse(q)),q)) <= 1e-6,"{what}: its witness {at:?} is off `{face}`");
    }
    let mut kinds = std::collections::BTreeMap::<&str,usize>::new();
    for c in cases { for f in &c.read.faces { *kinds.entry(kind(f)).or_default() += 1 } }
    eprintln!("faces read: {kinds:?}");
    for k in ["a revolution","a strip","a cap"] { assert!(kinds.contains_key(k),"no face read as {k}") }
    cases.len()
}

/// The reading's faces are the B-rep's boundary: every point of a read face on the tool lies on
/// the B-rep's boundary, and every point of a B-rep face lies on a read face square to its normal.
/// The points met.
fn bound_its_brep(cases: &[Case]) -> usize {
    let mut points = 0;
    for c in cases {
        let what = &c.what;
        for (k,f) in c.read.faces.iter().enumerate() {
            for (p,_) in on_tool(f,&c.read,8) {
                assert_eq!(c.place(p),Place::On,"{what}: the read face {k} `{}` at {p:?} is off the B-rep's boundary",f.name());
                points += 1;
            }
        }
        let b = c.b();
        for (fi,face) in b.faces.iter().enumerate() {
            let [[u0,u1],[v0,v1]] = gcs_core::brep::boolean::face_parameters(b,fi);
            // the face's edges, a point read on one of which is two faces' and passed over
            let rim: Vec<V> = face.loops.iter().flatten().flat_map(|e| {
                let e = &b.edges[e.edge as usize];
                (0..=32).map(move |k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/32.,&b.vertices))
            }).collect();
            // (off the middle of the face's box, where a disc's centre is its reading's pole)
            let n = 6;
            for i in 0..n { for j in 0..n {
                let (a,b) = ((i as f64+0.37)/n as f64,(j as f64+0.61)/n as f64);
                let q = face.surface.point([u0+(u1-u0)*a,v0+(v1-v0)*b]);
                if c.brep.solid_place(q) != Place::On || rim.iter().any(|r| norm(sub(*r,q)) < 1e-3*c.size*c.mm) { continue }
                let Some(normal) = face.outward(q) else { continue };
                let p = q.map(|x| x/c.mm);
                assert!(!faces_at(&c.read,p,normal,c.size).is_empty(),"{what}: the B-rep's face `{}` at {p:?} lies on no read face",
                    face.name);
                points += 1;
            } }
        }
    }
    eprintln!("{points} points on the reading's faces and the B-rep's");
    points
}

/// The reading's material and the drawing's field of the tool agree off the boundary, and the
/// material reads nought on it: the probes read.
fn its_tools_material(cases: &[Case]) -> usize {
    let mut probes = 0;
    for c in cases {
        let what = &c.what;
        let Some(field) = &c.field else { continue };
        // a grid over the box the tool's faces' points lie in, a little past it
        let pts: Vec<V> = c.read.faces.iter().flat_map(|f| on_tool(f,&c.read,4)).map(|(p,_)| p).collect();
        let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
        for p in &pts { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]) } }
        let n = 7;
        for i in 0..=n { for j in 0..=n { for l in 0..=n {
            let t = [i,j,l].map(|x| (x as f64+0.37)/n as f64*1.2-0.1);
            let p: V = std::array::from_fn(|k| lo[k]+(hi[k]-lo[k])*t[k]);
            let a = c.read.material().value(p);
            // clear of the boundary by a thousandth of the tool
            if a.abs() < 1e-3*c.size { continue }
            let b = field.value(p);
            if b.abs() < 1e-3*c.size { continue }
            assert_eq!(a < 0.,b < 0.,"{what}: at {p:?} the reading's material reads {a:e}, the drawing's field {b:e}");
            probes += 1;
        } } }
        for p in pts.iter().take(64) { assert!(c.read.material().value(*p).abs() < 1e-9*c.size,"{what}: {p:?} on a face reads off it") }
    }
    eprintln!("{probes} probes read alike");
    probes
}

#[test]
fn the_catalogues_tools_read_from_their_boundaries() {
    assert!(reads_every_tool(catalogue()) >= 30);
}

#[test]
fn the_reading_bounds_its_brep() {
    assert!(bound_its_brep(&catalogue().0) > 5000);
}

#[test]
fn the_reading_is_its_tools_material() {
    assert!(its_tools_material(&catalogue().0) > 5000);
}

/// The same over every example's sweeps: engines, gears, hypoid crowns standing on one plane about
/// two axes, twist drills.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 30 s: every example's sweeps elaborated and solved")]
fn every_examples_tools_read_from_their_boundaries() {
    let all = every();
    assert!(reads_every_tool(&all) >= 60);
    assert!(bound_its_brep(&all.0) > 20000);
    assert!(its_tools_material(&all.0) > 20000);
}

#[test]
fn a_face_no_closed_form_reads_is_refused_by_name() {
    // the Wankel's tool is a chamber bored by an epitrochoid: an extrusion of a curve no line or arc
    let e = fixtures::example("wankel/wankel.sv").expect("the Wankel");
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
    let swept = sk.solids.iter().position(|s| matches!(s.def,SolidDef::Swept {..})).expect("a sweep");
    let SolidDef::Swept {source,..} = &sk.solids[swept].def else { unreachable!() };
    let b = gcs_core::brep::recipe::build(&cad::recipe(&sk,*source as usize).unwrap()).unwrap();
    let mm = cad::millimetres(&sk);
    let why = Tool::read(&b,mm).expect_err("an extrusion of an epitrochoid is read");
    let Unread::FreeForm {face,kind,at} = &why else { panic!("refused as {why:?}") };
    assert_eq!(*kind,"extrusion");
    let f = b.faces.iter().find(|f| gcs_core::brep::recipe::joined(&f.name) == *face).expect("the face it names");
    let q = at.map(|x| x*mm);
    assert!(norm(sub(f.surface.point(f.surface.inverse(q)),q)) <= 1e-6,"its witness {at:?} is off `{face}`");
    assert_eq!(why.witness(),Some(*at));
    eprintln!("{why}");
}


/// **A cutter of revolution is sectioned from its B-rep as from its recipe** (#721 F1d): every
/// sweep whose tool's recipe is sectioned as revolutions (`Sectioned::read`) has its B-rep sectioned
/// so too where it is revolutions about one line (`Sectioned::of_boundary`), the two sections at
/// several half-planes one region — the same area, and every point of a grid over it in both or
/// in neither, read in space. A tool of revolutions about parallel lines (a spiral bevel's cutter)
/// the B-rep does not section yet, and it is named. Returns how many cutters were held, and how many
/// were refused.
fn sections_alike(within: impl Fn(&str) -> bool) -> (usize,usize) {
    use gcs_core::brep::section::Sectioned;
    let (mut held,mut refused) = (0,0);
    for name in fixtures::example_names().into_iter().filter(|n| within(n)) {
        let Some(e) = fixtures::example(&name) else { continue };
        if !e.sketch.solids.iter().any(|s| matches!(s.def,SolidDef::Swept {..})) { continue }
        let mut sk = e.sketch.clone();
        gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
        for swept in 0..sk.solids.len() {
            let SolidDef::Swept {source,..} = &sk.solids[swept].def else { continue };
            let what = format!("{name}: `{}`",sk.solids[swept].name);
            let Ok(recipe) = cad::recipe(&sk,*source as usize) else { continue };
            let Ok(drawn) = Sectioned::read(&recipe) else { continue };
            let Ok(brep) = gcs_core::brep::recipe::build(&recipe) else { continue };
            let read = match Sectioned::of_boundary(&brep) {
                Ok(read) => read,
                Err(why) => { eprintln!("{what}: not sectioned from its B-rep: {why}"); refused += 1; continue }
            };
            assert!(norm(cross(drawn.axis,read.axis)) <= 1e-9 && norm(cross(sub(read.origin,drawn.origin),drawn.axis)) <= 1e-6,
                "{what}: about one line");
            let (e1,e2) = gcs_core::space::across(drawn.axis);
            for k in 0..5 {
                let (s,c) = (0.7+1.3*k as f64).sin_cos();
                let side: V = std::array::from_fn(|i| c*e1[i]+s*e2[i]);
                let (a,b) = (drawn.section(side).unwrap(),read.section(side).unwrap());
                // each region's area by Green's theorem over its steps, exactly for lines and arcs
                // (its own polygon is coarse, and how a Boolean splits an arc moves its chords)
                let area = |r: &gcs_core::brep::planar::Region| r.loops.iter().flatten().map(|st| {
                    use gcs_core::brep::planar::Seg;
                    match (st.chart,st.seg) {
                        (None,Seg::Line {a,b}) => 0.5*(a[0]*b[1]-b[0]*a[1]),
                        (None,Seg::Arc {c,r,a0,sweep}) => {
                            let a1 = a0+sweep;
                            0.5*(r*r*sweep+r*(c[0]*(a1.sin()-a0.sin())-c[1]*(a1.cos()-a0.cos())))
                        }
                        _ => (0..4096).map(|i| { let (p,q) = (st.point(i as f64/4096.),st.point((i+1) as f64/4096.)); 0.5*(p[0]*q[1]-q[0]*p[1]) }).sum(),
                    }
                }).sum::<f64>().abs();
                let (x,y) = (area(&a),area(&b));
                assert!((x-y).abs() <= 1e-6*x.max(1.),"{what}: the section's area {y} from the B-rep, {x} from the recipe");
                // a grid over the recipe's section, each point read in the B-rep's section's coordinates
                let points: Vec<[f64;2]> = a.loops.iter().flatten().flat_map(|st| (0..8).map(move |i| st.point(i as f64/8.))).collect();
                let lo = points.iter().fold([f64::INFINITY;2],|m,p| [m[0].min(p[0]),m[1].min(p[1])]);
                let hi = points.iter().fold([f64::NEG_INFINITY;2],|m,p| [m[0].max(p[0]),m[1].max(p[1])]);
                for i in 0..=20 { for j in 0..=20 {
                    let p = [lo[0]+(hi[0]-lo[0])*(i as f64+0.37)/21.,lo[1]+(hi[1]-lo[1])*(j as f64+0.61)/21.];
                    let at: V = std::array::from_fn(|m| drawn.origin[m]+p[0]*side[m]+p[1]*drawn.axis[m]);
                    let q = [p[0],dot(sub(at,read.origin),read.axis)];
                    assert_eq!(a.contains(p),b.contains(q),"{what}: the point {at:?} of the half-plane toward {side:?}");
                } }
            }
            held += 1;
        }
    }
    (held,refused)
}

#[test]
fn the_catalogues_cutters_of_revolution_section_from_their_boundaries() {
    let (held,refused) = sections_alike(|name| name.starts_with("solid_sweeps/"));
    eprintln!("{held} cutters of revolution sectioned alike, {refused} not sectioned from the B-rep");
    assert!(held >= 5 && refused == 0,"{held} held, {refused} refused");
}

#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, every example's cutters of revolution")]
fn every_examples_cutters_of_revolution_section_from_their_boundaries() {
    let (held,refused) = sections_alike(|_| true);
    eprintln!("{held} cutters of revolution sectioned alike, {refused} not sectioned from the B-rep");
    assert!(held >= 5,"{held} held");
}
