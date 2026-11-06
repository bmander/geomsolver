//! **The tool read from its B-rep, held beside the drawing's reading** (#721 F1, #828 F1a).
//!
//! Every sweep of the examples whose tool the solver reads off its drawing (`SweepContacts::read`)
//! has that tool's recipe built into a B-rep and read by the kernel (`Tool::read`): the sweep
//! catalogue's (`rust/examples/solid_sweeps/`) in the fast suite, every example's in the slow tier
//! (`every_examples_tools_read_alike`). The two readings are held to one tool:
//! - its **boundary**: every point of a face of either reading that is on the tool lies on a face of
//!   the other, square to the same normal, and of the same kind (a revolution, a strip or a cap);
//! - its **material**: the two agree which side of the boundary a point clear of it is on;
//! - its **corners**: every sharp corner and crease of the drawing's reading on the tool is one of
//!   the B-rep's, and every sharp corner of the B-rep's lies on two faces of the drawing's;
//! - its **walk**: under a constant twist, the characteristic walked on each through the blank is
//!   one curve, or both are refused by one row.
//!
//! Where the two differ the B-rep reading reads more: a drafted prism's caps, which the drawing's
//! reading leaves to admission (`swept::admission::ends`), and the faces a Boolean of the tool splits.
use gcs_core::brep::swept::admission::{check_screw,Generating,Sampling};
use gcs_core::brep::swept::constant_twist::{Characteristic,Row};
use gcs_core::brep::swept::read::Unread;
use gcs_core::brep::swept::tool::{Tool,ToolSurface};
use gcs_core::brep::query::{Located,Place};
use gcs_core::brep::trajectory::Trajectory;
use gcs_core::model::{Sketch,SolidDef};
use gcs_core::solid::{cad,SweepContacts};
use gcs_core::space::{cross,dot,norm,normalised,segment_distance,sub};
use std::sync::OnceLock;

type V = [f64;3];


/// One sweep of the examples, its tool read both ways: off the drawing (`drawn`) and off its B-rep
/// (`read`), with its path, roll and blank for the walk.
struct Case {
    what: String,
    drawn: Tool,
    read: Tool,
    /// The sweep's path where it lowers, its roll, and whether it is a whole turn read once round.
    path: Option<Trajectory>,
    roll: [f64;2],
    whole: bool,
    /// The blank it cuts (`blank`).
    blank: Option<Blank>,
    /// The tool's size, what lengths are read against.
    size: f64,
}

/// The blank a sweep cuts: the static remainder of the first body it cuts, built by the kernel and
/// its points located, read in the sweep's frame in model units.
struct Blank { located: Located<'static>,pose: gcs_core::envelope::Motion,mm: f64 }

impl Blank {
    fn read(sk: &Sketch,swept: usize) -> Option<Blank> {
        let mm = cad::millimetres(sk);
        for root in 0..sk.solids.len() {
            if !matches!(sk.solids[root].def,SolidDef::Body {..}) { continue }
            let Ok(r) = cad::recipe_static(sk,root) else { continue };
            let Some(cut) = r.sweeps.iter().find(|c| c.swept == swept) else { continue };
            let b = gcs_core::brep::recipe::build(&r.recipe).ok()?;
            let tol = 1e-7*(1.+b.size());
            return Some(Blank {located:Located::owned(b,tol),pose:cut.pose,mm})
        }
        None
    }
    fn holds(&self,p: V) -> bool { self.located.solid_place(self.pose.point(p).map(|x| x*self.mm)) == Place::In }
}

/// The sweeps read both ways, and those whose B-rep reading refuses, with why.
type Cases = (Vec<Case>,Vec<(String,Unread)>);

/// The sweep catalogue's (`rust/examples/solid_sweeps/`): a case of each motion family the classes
/// admit, read in a second or two.
fn catalogue() -> &'static Cases {
    static CASES: OnceLock<Cases> = OnceLock::new();
    CASES.get_or_init(|| read_cases(|name| name.starts_with("solid_sweeps/")))
}

/// Every sweep of every example whose tool the drawing's reading reads, read from its B-rep too.
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
            let Ok(drawn) = SweepContacts::read(&sk,swept,cad::AXIS_TOLERANCE) else { continue };
            let what = format!("{name}: `{}`",sk.solids[swept].name);
            let brep = gcs_core::brep::recipe::build(&cad::recipe(&sk,*source as usize).unwrap())
                .unwrap_or_else(|m| panic!("{what}: the tool the drawing reads builds no B-rep: {m}"));
            match Tool::read(&brep,cad::millimetres(&sk)) {
                Ok(read) => {
                    let size = read.faces.iter().filter_map(|f| f.at(0.5,0.5).ok()).map(|p| norm(p.position)).fold(1.,f64::max);
                    // (the blank only where the walk is asked: under a constant twist)
                    let path = drawn.motion().path().ok().cloned();
                    let blank = path.as_ref().filter(|p| p.screw().is_some()).and_then(|_| Blank::read(&sk,swept));
                    cases.push(Case {what,drawn:drawn.tool().clone(),read,path,roll:drawn.domain(),
                        whole:drawn.whole(),blank,size})
                }
                Err(why) => refused.push((what,why)),
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

/// Every tool the drawing's reading reads, the boundary's reads too, and of each kind: how many.
fn reads_every_tool((cases,refused): &Cases) -> usize {
    eprintln!("{} tools read both ways",cases.len());
    for (what,why) in refused { eprintln!("refused: {what}: {why}") }
    assert!(refused.is_empty(),"{} refused by the B-rep reading",refused.len());
    let mut kinds = std::collections::BTreeMap::<&str,usize>::new();
    for c in cases { for f in &c.read.faces { *kinds.entry(kind(f)).or_default() += 1 } }
    eprintln!("faces read: {kinds:?}");
    for k in ["a revolution","a strip","a cap"] { assert!(kinds.contains_key(k),"no face read as {k}") }
    cases.len()
}

/// The two readings' faces are one boundary, face for face of one kind: the points met.
fn bound_one_tool(cases: &[Case]) -> usize {
    let (mut points,mut extra) = (0,0);
    for c in cases {
        let what = &c.what;
        let drafted_caps = !c.drawn.faces.iter().any(|f| f.cap().is_some());
        // every point of the B-rep's faces on the tool lies on a face of the drawing's reading of
        // its kind, but a drafted prism's caps, which that reading leaves to admission
        for (k,f) in c.read.faces.iter().enumerate() {
            for (p,n) in on_tool(f,&c.read,8) {
                let on = faces_at(&c.drawn,p,n,c.size);
                if on.is_empty() && f.cap().is_some() && drafted_caps { extra += 1; continue }
                assert!(!on.is_empty(),"{what}: the B-rep's face {k} `{}` at {p:?} lies on no face of the drawing's",f.name());
                assert!(on.iter().any(|&j| kind(&c.drawn.faces[j]) == kind(f)),
                    "{what}: the B-rep's face `{}` is {}, the drawing's there {}",f.name(),kind(f),kind(&c.drawn.faces[on[0]]));
                points += 1;
            }
        }
        // and every point of the drawing's on its tool lies on a face of the B-rep's
        for f in c.drawn.faces.iter() {
            for (p,n) in on_tool(f,&c.drawn,8) {
                assert!(!faces_at(&c.read,p,n,c.size).is_empty(),"{what}: the drawing's face `{}` at {p:?} lies on no face of the B-rep's",
                    f.name());
                points += 1;
            }
        }
    }
    eprintln!("{points} points on both readings' faces; {extra} on a drafted prism's caps, the B-rep's alone");
    points
}

/// The two readings' materials agree off the boundary, and the B-rep's reads nought on it: the
/// probes read.
fn one_material(cases: &[Case]) -> usize {
    let mut probes = 0;
    for c in cases {
        let what = &c.what;
        // a grid over the box the tool's faces' points lie in, a little past it
        let pts: Vec<V> = c.read.faces.iter().flat_map(|f| on_tool(f,&c.read,4)).map(|(p,_)| p).collect();
        let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
        for p in &pts { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]) } }
        let n = 7;
        for i in 0..=n { for j in 0..=n { for l in 0..=n {
            let t = [i,j,l].map(|x| (x as f64+0.37)/n as f64*1.2-0.1);
            let p: V = std::array::from_fn(|k| lo[k]+(hi[k]-lo[k])*t[k]);
            let (a,b) = (c.read.material().value(p),c.drawn.material().value(p));
            // clear of the boundary by a thousandth of the tool, as each reads it
            if a.abs() < 1e-3*c.size || b.abs() < 1e-3*c.size { continue }
            assert_eq!(a < 0.,b < 0.,"{what}: at {p:?} the B-rep's material reads {a:e}, the drawing's {b:e}");
            probes += 1;
        } } }
        // and on the boundary it reads nought
        for p in pts.iter().take(64) { assert!(c.read.material().value(*p).abs() < 1e-9*c.size,"{what}: {p:?} on a face reads off it") }
    }
    eprintln!("{probes} probes read alike");
    probes
}

/// The two readings' sharp corners and creases are one another's: the points of each met.
fn corners_alike(cases: &[Case]) -> (usize,usize) {
    let (mut sharp,mut creased) = (0,0);
    for c in cases {
        let what = &c.what;
        let (drawn,read) = (&c.drawn,&c.read);
        let tol = 1e-7*c.size;
        // the point of face `f` at `uv`, with its unit normal
        let at = |f: &ToolSurface,uv: [f64;2]| f.at(uv[0],uv[1]).ok().and_then(|x| Some((x.position,normalised(cross(x.du,x.dv))?)));
        // a corner's points along it, on the tool, where its two faces' normals part, with the two
        // normals there
        let corner = |tool: &Tool,e: &gcs_core::brep::swept::tool::PatchEdge| -> Vec<(V,V,V)> {
            let ([a,b],[ua,_]) = (e.patches,e.ends);
            let [v0,v1] = tool.faces[a].domain()[1];
            (0..=8).filter_map(|i| at(&tool.faces[a],[ua,v0+(v1-v0)*(i as f64+0.5)/9.])).filter_map(|(p,n)| {
                let (d,_,m) = nearest(&tool.faces[b],p)?;
                (d <= tol && tool.material().value(p).abs() < 1e-6 && norm(cross(n,m)) > 1e-3).then_some((p,n,m))
            }).collect()
        };
        for e in drawn.edges() {
            for (p,..) in corner(drawn,e) {
                let on = read.edges.iter().any(|f| f.patches.iter().zip(f.ends).all(|(&k,u)|
                    nearest(&read.faces[k],p).is_some_and(|(d,uv,_)| d <= tol && (uv[0]-u).abs() <= 1e-6)));
                assert!(on,"{what}: the drawing's corner {:?} at {p:?} is no corner of the B-rep's",e.patches);
                sharp += 1;
            }
        }
        for e in read.edges.iter() {
            for (p,n,m) in corner(read,e) {
                let on: std::collections::BTreeSet<usize> = faces_at(drawn,p,n,c.size).into_iter().chain(faces_at(drawn,p,m,c.size)).collect();
                assert!(on.len() >= 2,"{what}: the B-rep's corner {:?} at {p:?} lies on {} of the drawing's faces",e.patches,on.len());
            }
        }
        assert_eq!(drawn.creases().len(),read.creases.len(),"{what}: creases");
        for &[a,b] in drawn.creases() {
            for i in 0..=8 {
                let Some((p,_)) = at(&drawn.faces[a],[(i as f64+0.5)/9.,1.]) else { continue };
                let on = read.creases.iter().any(|&[x,y]| [(x,1.),(y,0.)].iter().all(|&(k,v)|
                    nearest(&read.faces[k],p).is_some_and(|(d,uv,_)| d <= tol && (uv[1]-v).abs() <= 1e-6)));
                assert!(on,"{what}: the drawing's crease {:?} at {p:?} is no crease of the B-rep's",[a,b]);
                creased += 1;
            }
        }
    }
    eprintln!("{sharp} points of sharp corners, {creased} of creases, met on both");
    (sharp,creased)
}

/// Under a constant twist, the two readings walk one characteristic through the blank, node for
/// node, or are refused by one row: how many walked, and how many were refused.
fn walk_alike(cases: &[Case]) -> (usize,usize) {
    let sampling = Sampling {rows:24,columns:96,coarse_rows:12,coarse_columns:96,root_tolerance:1e-9,least_factor:1e-3};
    let (mut walked,mut refused) = (0,0);
    for c in cases {
        let what = &c.what;
        let (Some(path),Some(blank)) = (&c.path,&c.blank) else { continue };
        if path.screw().is_none() { continue }
        let inside = |p: V| blank.holds(p);
        let walk = |tool: &Tool| check_screw(&Generating {tool,path,roll:c.roll,whole:c.whole,mm:1.},&inside,None,&sampling,None)
            .map(|(found,..)| found);
        match (walk(&c.drawn),walk(&c.read)) {
            (Err(a),Err(b)) => {
                // refused alike, or the B-rep's reading refused at a concave corner where a
                // Boolean of the tool meets, which the drawing's reading keeps whole
                let corner = b.0 == Row::Corner && c.read.edges.len() > c.drawn.edges.len();
                assert!(a.0 == b.0 || corner,"{what}: refused by {:?} drawn ({}), by {:?} read ({})",a.0,a.1,b.0,b.1);
                if a.0 != b.0 { eprintln!("{what}: refused by {:?} drawn, at a corner its Boolean makes read ({})",a.0,b.1) }
                refused += 1
            }
            (Ok(a),Ok(b)) => {
                // each walk's nodes in its reach on the other's curve
                let off = |x: &[V],curve: &[V]| x.iter().map(|&p| curve.windows(2).map(|w| segment_distance(p,w[0],w[1]))
                    .fold(f64::INFINITY,f64::min)).fold(0.,f64::max);
                let points = |n: &[gcs_core::brep::swept::constant_twist::Node]| n.iter().map(|n| n.position).collect::<Vec<V>>();
                let (reach,all) = (|f: &Characteristic| points(&f.nodes[f.reach[0]..=f.reach[1]]),|f: &Characteristic| points(&f.nodes));
                // (the charts start alike, so the walks step alike: to rounding)
                let bar = 1e-9*c.size;
                let (ab,ba) = (off(&reach(&a),&all(&b)),off(&reach(&b),&all(&a)));
                eprintln!("{what}: walked alike to {:.1e} and {:.1e} ({} and {} nodes)",ab,ba,a.nodes.len(),b.nodes.len());
                assert_eq!((a.nodes.len(),a.reach,a.closed),(b.nodes.len(),b.reach,b.closed),"{what}: the walks' nodes");
                assert!(ab <= bar && ba <= bar,"{what}: the walks part by {ab:e} and {ba:e} (against {bar:e})");
                walked += 1;
            }
            (a,b) => panic!("{what}: drawn {:?}, read {:?}",a.map(|_| ()).map_err(|e| (e.0,e.1)),b.map(|_| ()).map_err(|e| (e.0,e.1))),
        }
    }
    eprintln!("{walked} walked alike, {refused} refused alike");
    (walked,refused)
}

#[test]
fn the_catalogues_tools_read_from_their_boundaries() {
    assert!(reads_every_tool(catalogue()) >= 30);
}

#[test]
fn the_two_readings_bound_one_tool() {
    assert!(bound_one_tool(&catalogue().0) > 5000);
}

#[test]
fn the_two_readings_are_one_material() {
    assert!(one_material(&catalogue().0) > 5000);
}

#[test]
fn the_two_readings_turn_their_corners_alike() {
    let (sharp,creased) = corners_alike(&catalogue().0);
    assert!(sharp > 300 && creased > 50,"{sharp} corner points, {creased} crease points");
}

#[test]
fn the_two_readings_walk_alike() {
    let (walked,refused) = walk_alike(&catalogue().0);
    assert!(walked >= 10 && refused >= 5,"{walked} walked, {refused} refused");
}

/// The same over every example's sweeps: engines, gears, hypoid crowns standing on one plane about
/// two axes, twist drills.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 30 s: every example's sweeps elaborated and solved")]
fn every_examples_tools_read_alike() {
    let all = every();
    assert!(reads_every_tool(&all) >= 60);
    assert!(bound_one_tool(&all.0) > 20000);
    assert!(one_material(&all.0) > 20000);
    let (sharp,creased) = corners_alike(&all.0);
    assert!(sharp > 1000 && creased > 50,"{sharp} corner points, {creased} crease points");
    let (walked,refused) = walk_alike(&all.0);
    assert!(walked >= 13 && refused >= 10,"{walked} walked, {refused} refused");
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
    let f = b.faces.iter().find(|f| f.name == *face).expect("the face it names");
    let q = at.map(|x| x*mm);
    assert!(norm(sub(f.surface.point(f.surface.inverse(q)),q)) <= 1e-6,"its witness {at:?} is off `{face}`");
    assert_eq!(why.witness(),Some(*at));
    eprintln!("{why}");
}

