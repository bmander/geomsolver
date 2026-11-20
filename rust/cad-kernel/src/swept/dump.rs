//! **Dump, replay, reduce** (#777 V5): a closed build that fails writes what its arrangement was
//! given — the domain and every candidate solid, exactly, with the tolerance — where
//! `SOLVENT_SWEEP_DUMP` names a directory (`dump`), and the arrangement alone is run again from
//! those files (`replay`) in about a second, apart from the tool, the motion and the membership
//! that made them. A failure is then reduced to the smallest set of candidates whose arrangement
//! still fails (`reduce`), and that Boolean kept as a fast test before the failure is fixed
//! (docs/testing.md's first tier).
//!
//! The files are this kernel's own B-reps written whole (`write`/`read`): every vertex, edge and
//! use as it is, each curve and surface by its exact parameters, so a replay meets bit for bit what
//! the build met. Every surface and curve a closed build's candidates are made of is written: the
//! quadrics, the torus, a surface of revolution, a swept sheet (`Blend`), an affine image, a
//! B-spline, a face family's wall and the rails and rims along it (#942 F5); a kind none of them
//! is (a trace) is refused by name.
use crate::boolean::{Given,arrangement_reached};
use crate::complex::Complex;
use crate::geom::{Blend,Carry,Curve,Frame,Surface,V};
use crate::nurbs::{BSpline,Net};
use crate::topo::{Brep,Coedge,Edge,EdgeCurve,Face,Pcurve,Vertex};
use crate::util::json::{Json,object,parse};
use std::sync::Arc;

fn v3(v: V) -> Json { Json::Arr(v.iter().map(|&x| Json::Num(x)).collect()) }
fn reals(xs: &[f64]) -> Json { Json::Arr(xs.iter().map(|&x| Json::Num(x)).collect()) }
fn field<'a>(j: &'a Json,k: &str) -> Result<&'a Json,String> { j.get(k).ok_or_else(|| format!("sweep dump: no `{k}`")) }
fn rv3(j: &Json) -> V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }
fn rreals(j: &Json) -> Vec<f64> { j.arr().iter().map(Json::as_f64).collect() }
fn num(j: &Json,k: &str) -> Result<f64,String> { Ok(field(j,k)?.as_f64()) }

fn frame(f: &Frame) -> Json { object([("o",v3(f.o)),("x",v3(f.x)),("y",v3(f.y)),("z",v3(f.z))]) }
fn rframe(j: &Json) -> Result<Frame,String> {
    Ok(Frame {o:rv3(field(j,"o")?),x:rv3(field(j,"x")?),y:rv3(field(j,"y")?),z:rv3(field(j,"z")?)})
}

fn affine(a: &crate::affine::Affine) -> Json {
    object([("m",Json::Arr(a.m.iter().map(|r| reals(r)).collect())),("t",v3(a.t))])
}
fn raffine(j: &Json) -> Result<crate::affine::Affine,String> {
    let m = field(j,"m")?.arr();
    let row = |k: usize| -> [f64;3] { let r = rreals(&m[k]); [r[0],r[1],r[2]] };
    crate::affine::Affine::new([row(0),row(1),row(2)],rv3(field(j,"t")?))
}

fn bspline(b: &BSpline) -> Json {
    let mut o = object([("degree",b.degree.into()),("knots",reals(&b.knots)),("poles",Json::Arr(b.poles.iter().map(|&p| v3(p)).collect()))]);
    if let Some(w) = &b.weights { o.set("weights",reals(w)) }
    o
}
fn rbspline(j: &Json) -> Result<BSpline,String> {
    let (degree,knots) = (field(j,"degree")?.as_i64() as usize,rreals(field(j,"knots")?));
    let poles = field(j,"poles")?.arr().iter().map(rv3).collect();
    match j.get("weights") { Some(w) => BSpline::rational(degree,knots,poles,rreals(w)),None => BSpline::new(degree,knots,poles) }
}

fn blend(b: &Blend) -> Result<Json,String> {
    let carry = match b.carry {
        Carry::Line {delta} => object([("kind","line".into()),("delta",v3(delta))]),
        Carry::Screw {center,axis,angle,advance} => object([("kind","screw".into()),("center",v3(center)),("axis",v3(axis)),
            ("angle",angle.into()),("advance",advance.into())]),
    };
    Ok(object([("a",curve(&b.a)?),("ta",reals(&b.ta)),("b",curve(&b.b)?),("tb",reals(&b.tb)),("carry",carry),("closed",b.closed.into())]))
}
fn rblend(j: &Json) -> Result<Blend,String> {
    let c = field(j,"carry")?;
    let carry = match field(c,"kind")?.as_str() {
        "line" => Carry::Line {delta:rv3(field(c,"delta")?)},
        _ => Carry::Screw {center:rv3(field(c,"center")?),axis:rv3(field(c,"axis")?),angle:num(c,"angle")?,advance:num(c,"advance")?},
    };
    let two = |k: &str| -> Result<[f64;2],String> { let r = rreals(field(j,k)?); Ok([r[0],r[1]]) };
    Ok(Blend {a:rcurve(field(j,"a")?)?,ta:two("ta")?,b:rcurve(field(j,"b")?)?,tb:two("tb")?,carry,closed:field(j,"closed")?.as_bool()})
}

/// **A face family's wall by its numbers** (#942 F5): its run (a spline's degree, knots and
/// weights), its stations (the pieces and each piece's six nodes) and every node's numbers — what
/// `Sheet::new` makes it from, the rest (the stations' inverses, the pieces' boxes) made again as
/// it was.
fn sheet(w: &crate::family::Sheet) -> Json {
    use crate::family::Run;
    let run = match &w.run {
        Run::Line => object([("kind","line".into())]),
        Run::Round => object([("kind","round".into())]),
        Run::Spline {degree,knots,weights} => {
            let mut o = object([("kind","spline".into()),("degree",(*degree).into()),("knots",reals(knots))]);
            if let Some(w) = weights { o.set("weights",reals(w)) }
            o
        }
    };
    object([("run",run),("spans",Json::Arr(w.stations.spans.iter().map(|s| reals(s)).collect())),
        ("at",Json::Arr(w.stations.at.iter().map(|a| reals(a)).collect())),("nodes",Json::Arr(w.nodes.iter().map(|n| reals(n)).collect())),
        ("closed",w.closed.into())])
}
fn rsheet(j: &Json) -> Result<crate::family::Sheet,String> {
    use crate::family::{Run,Sheet,Stations};
    let r = field(j,"run")?;
    let run = match field(r,"kind")?.as_str() {
        "line" => Run::Line,
        "round" => Run::Round,
        "spline" => Run::Spline {degree:field(r,"degree")?.as_i64() as usize,knots:rreals(field(r,"knots")?),weights:r.get("weights").map(rreals)},
        k => return Err(format!("sweep dump: a face family's run of kind `{k}`")),
    };
    let spans = field(j,"spans")?.arr().iter().map(|s| { let s = rreals(s); [s[0],s[1]] }).collect();
    let at = field(j,"at")?.arr().iter().map(|a| { let a = rreals(a); std::array::from_fn(|k| a[k]) }).collect();
    let nodes = field(j,"nodes")?.arr().iter().map(rreals).collect();
    Sheet::new(run,Arc::new(Stations::new(spans,at)?),nodes,field(j,"closed")?.as_bool())
}

thread_local! {
    /// The walls written so far in this dump, by address: each one's place the number it is shared
    /// by (`shared`).
    static WRITTEN: std::cell::RefCell<Vec<usize>> = const { std::cell::RefCell::new(Vec::new()) };
    /// The walls read so far in this load, by the number they were written with (`rshared`).
    static READ: std::cell::RefCell<std::collections::BTreeMap<i64,Arc<crate::family::Sheet>>> =
        const { std::cell::RefCell::new(std::collections::BTreeMap::new()) };
}

/// **A wall written with the number it is shared by** (#942 T1d-cube): one wall borne by several
/// faces and curves — a fan's run, the rails restricted to it — is one wall read back, as it was
/// built. A wall's inverse starts from the last foot found on it (`warm`), so walls read back apart
/// read their feet from other starts, to the rounding otherwise, and a trim replayed was not the
/// trim refused bit for bit.
fn shared(w: &Arc<crate::family::Sheet>) -> Json {
    let at = Arc::as_ptr(w) as usize;
    let id = WRITTEN.with(|v| { let mut v = v.borrow_mut(); v.iter().position(|&p| p == at).unwrap_or_else(|| { v.push(at); v.len()-1 }) });
    let mut o = sheet(w);
    o.set("share",id.into());
    o
}
/// A wall `shared` wrote, the one already read where its number was (a dump written before #942
/// T1d-cube shares none).
fn rshared(j: &Json) -> Result<Arc<crate::family::Sheet>,String> {
    let Some(id) = j.get("share").map(Json::as_i64) else { return Ok(Arc::new(rsheet(j)?)) };
    if let Some(w) = READ.with(|m| m.borrow().get(&id).cloned()) { return Ok(w) }
    let w = Arc::new(rsheet(j)?);
    READ.with(|m| m.borrow_mut().insert(id,w.clone()));
    Ok(w)
}
thread_local! {
    /// Whether a dump or a load is under way, its files' walls numbered as one (`Walls`).
    static WHOLE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// The tables of walls shared, emptied.
fn fresh_walls() { WRITTEN.with(|v| v.borrow_mut().clear()); READ.with(|m| m.borrow_mut().clear()) }

/// **A dump or a load under way** (`shared`): its tables emptied as it starts, its every file's walls
/// numbered as one while it lasts; a brep written or read alone (`write`, `read`) numbers its own.
struct Walls;
impl Walls {
    fn whole() -> Walls { fresh_walls(); WHOLE.with(|w| w.set(true)); Walls }
    fn alone() { if !WHOLE.with(|w| w.get()) { fresh_walls() } }
}
impl Drop for Walls { fn drop(&mut self) { WHOLE.with(|w| w.set(false)); fresh_walls() } }

fn pair(j: &Json,k: &str) -> Result<[f64;2],String> { let r = rreals(field(j,k)?); Ok([r[0],r[1]]) }
fn contact(c: &crate::turned::Contact) -> Json {
    object([("o",v3(c.o)),("a",v3(c.a)),("r",c.r.into()),("sa",c.sa.into()),("ca",c.ca.into()),("side",c.side.into())])
}
fn rcontact(j: &Json) -> Result<crate::turned::Contact,String> {
    Ok(crate::turned::Contact {o:rv3(field(j,"o")?),a:rv3(field(j,"a")?),r:num(j,"r")?,sa:num(j,"sa")?,ca:num(j,"ca")?,side:num(j,"side")?})
}

/// A curve written by its exact parameters.
fn curve(c: &Curve) -> Result<Json,String> {
    Ok(match c {
        Curve::Line {p,d} => object([("kind","line".into()),("p",v3(*p)),("d",v3(*d))]),
        Curve::Circle(f,r) => object([("kind","circle".into()),("frame",frame(f)),("r",(*r).into())]),
        Curve::Ellipse(f,a,b) => object([("kind","ellipse".into()),("frame",frame(f)),("a",(*a).into()),("b",(*b).into())]),
        Curve::BSpline(b) => { let mut o = bspline(b); o.set("kind","bspline".into()); o }
        Curve::Iso(b,u) => object([("kind","iso".into()),("blend",blend(b)?),("u",(*u).into())]),
        Curve::Affine(m) => object([("kind","affine".into()),("map",affine(&m.map)),("of",curve(&m.of)?),("at",reals(&m.at))]),
        // each turned curve by its source's inputs, its derived readings made again as they were
        Curve::Turned(t) => {
            let mut o = match t.source {
                crate::turned::Source::Contact(c) => { let mut o = contact(&c); o.set("kind","turned".into()); o.set("posed",t.posed.into()); o }
                crate::turned::Source::Rim(c) =>
                    object([("kind","turned rim".into()),("o",v3(c.o)),("x",v3(c.x)),("y",v3(c.y)),("r",c.r.into())]),
                crate::turned::Source::Section(c) => {
                    let mut o = contact(&c.of);
                    for (k,v) in [("kind","turned section".into()),("n",v3(c.n)),("d",c.d.into()),("v",reals(&c.v))] { o.set(k,v) }
                    o
                }
                crate::turned::Source::Crossing(c) =>
                    object([("kind","turned crossing".into()),("c",v3(c.c)),("e1",v3(c.e1)),("e2",v3(c.e2)),("a",v3(c.a)),("r",c.r.into())]),
            };
            o.set("frame",frame(&t.frame));
            o.set("span",reals(&t.span));
            o
        }
        Curve::Slid(t) => { let c = &t.contact;
            object([("kind","slid".into()),("frame",frame(&t.frame)),("big",c.big.into()),("small",c.small.into()),("d",v3(c.d)),
                ("side",c.side.into()),("span",reals(&t.span)),("contour",t.contour.into())]) }
        Curve::Rail(w,u) => object([("kind","rail".into()),("sheet",shared(w)),("u",(*u).into())]),
        Curve::Rim(w,at) => object([("kind","rim".into()),("sheet",shared(w)),("s",(*at).into())]),
        other => return Err(format!("sweep dump: a {} curve is not written",other.kind())),
    })
}
fn rcurve(j: &Json) -> Result<Curve,String> {
    Ok(match field(j,"kind")?.as_str() {
        "line" => Curve::Line {p:rv3(field(j,"p")?),d:rv3(field(j,"d")?)},
        "circle" => Curve::Circle(rframe(field(j,"frame")?)?,num(j,"r")?),
        "ellipse" => Curve::Ellipse(rframe(field(j,"frame")?)?,num(j,"a")?,num(j,"b")?),
        "bspline" => Curve::BSpline(Arc::new(rbspline(j)?)),
        "iso" => Curve::Iso(Arc::new(rblend(field(j,"blend")?)?),num(j,"u")?),
        "affine" => { let at = rreals(field(j,"at")?);
            Curve::Affine(Arc::new(crate::affine::MappedCurve::new(raffine(field(j,"map")?)?,rcurve(field(j,"of")?)?,[at[0],at[1]]))) }
        kind @ ("turned" | "turned rim" | "turned section" | "turned crossing") => {
            use crate::turned::{Crossing,Rim,Section,Source,Turned};
            let (frame,span) = (rframe(field(j,"frame")?)?,pair(j,"span")?);
            let unread = || format!("sweep dump: a {kind} curve its inputs do not make");
            let (source,posed) = match kind {
                "turned" => (Source::Contact(rcontact(j)?),j.get("posed").is_some_and(|p| p.as_bool())),
                "turned rim" => (Source::Rim(Rim {o:rv3(field(j,"o")?),x:rv3(field(j,"x")?),y:rv3(field(j,"y")?),r:num(j,"r")?}),false),
                "turned section" => (Source::Section(Section::new(rcontact(j)?,rv3(field(j,"n")?),num(j,"d")?,pair(j,"v")?).ok_or_else(unread)?.0),true),
                _ => (Source::Crossing(Crossing::new(rv3(field(j,"c")?),rv3(field(j,"e1")?),rv3(field(j,"e2")?),rv3(field(j,"a")?),num(j,"r")?)
                    .ok_or_else(unread)?),true),
            };
            Curve::Turned(Arc::new(Turned {frame,source,span,posed}))
        }
        "slid" => { let span = rreals(field(j,"span")?);
            let contact = crate::slid::Contact {big:num(j,"big")?,small:num(j,"small")?,d:rv3(field(j,"d")?),side:num(j,"side")?};
            let contour = field(j,"contour")?.as_bool();
            Curve::Slid(Arc::new(crate::slid::Slid {frame:rframe(field(j,"frame")?)?,contact,span:[span[0],span[1]],contour})) }
        "rail" => Curve::Rail(rshared(field(j,"sheet")?)?,num(j,"u")?),
        "rim" => Curve::Rim(rshared(field(j,"sheet")?)?,num(j,"s")?),
        k => return Err(format!("sweep dump: a curve of kind `{k}`")),
    })
}

/// A surface written by its exact parameters.
fn surface(s: &Surface) -> Result<Json,String> {
    let with = |kind: &str,f: &Frame,more: Vec<(&str,Json)>| { let mut o = object([("kind",kind.into()),("frame",frame(f))]); for (k,v) in more { o.set(k,v) } o };
    Ok(match s {
        Surface::Plane(f) => with("plane",f,vec![]),
        Surface::Cylinder(f,r) => with("cylinder",f,vec![("r",(*r).into())]),
        Surface::Cone(f,r,a) => with("cone",f,vec![("r",(*r).into()),("angle",(*a).into())]),
        Surface::Sphere(f,r) => with("sphere",f,vec![("r",(*r).into())]),
        Surface::Torus(f,big,r) => with("torus",f,vec![("big",(*big).into()),("r",(*r).into())]),
        Surface::Extrusion(f,c) => with("extrusion",f,vec![("curve",curve(c)?)]),
        Surface::Revolution(f,c) => with("revolution",f,vec![("curve",curve(c)?)]),
        Surface::Blend(f,b) => with("blend",f,vec![("blend",blend(b)?)]),
        Surface::BSpline(f,n) => {
            let mut o = with("bspline",f,vec![("du",n.du.into()),("dv",n.dv.into()),("uknots",reals(&n.uknots)),("vknots",reals(&n.vknots)),
                ("poles",Json::Arr(n.poles.iter().map(|r| Json::Arr(r.iter().map(|&p| v3(p)).collect())).collect()))]);
            if let Some(w) = &n.weights { o.set("weights",Json::Arr(w.iter().map(|r| reals(r)).collect())) }
            o
        }
        Surface::Affine(m) => object([("kind","affine".into()),("map",affine(&m.map)),("of",surface(&m.of)?)]),
        Surface::Family(f,w) => with("family",f,vec![("sheet",shared(w))]),
        Surface::Offset(..) => return Err("sweep dump: an offset read by its signed distance is no face's surface".into()),
    })
}
fn rsurface(j: &Json) -> Result<Surface,String> {
    let kind = field(j,"kind")?.as_str();
    if kind == "affine" { return Ok(Surface::Affine(Arc::new(crate::affine::Mapped::new(raffine(field(j,"map")?)?,rsurface(field(j,"of")?)?)))) }
    let f = rframe(field(j,"frame")?)?;
    Ok(match kind {
        "plane" => Surface::Plane(f),
        "cylinder" => Surface::Cylinder(f,num(j,"r")?),
        "cone" => Surface::Cone(f,num(j,"r")?,num(j,"angle")?),
        "sphere" => Surface::Sphere(f,num(j,"r")?),
        "torus" => Surface::Torus(f,num(j,"big")?,num(j,"r")?),
        "extrusion" => Surface::Extrusion(f,Arc::new(rcurve(field(j,"curve")?)?)),
        "revolution" => Surface::Revolution(f,Arc::new(rcurve(field(j,"curve")?)?)),
        "blend" => Surface::Blend(f,Arc::new(rblend(field(j,"blend")?)?)),
        "family" => Surface::Family(f,rshared(field(j,"sheet")?)?),
        "bspline" => {
            let poles: Vec<Vec<V>> = field(j,"poles")?.arr().iter().map(|r| r.arr().iter().map(rv3).collect()).collect();
            let weights = j.get("weights").map(|w| w.arr().iter().map(rreals).collect());
            let net = Net {du:field(j,"du")?.as_i64() as usize,dv:field(j,"dv")?.as_i64() as usize,uknots:rreals(field(j,"uknots")?),
                vknots:rreals(field(j,"vknots")?),poles,weights};
            net.check()?;
            Surface::BSpline(f,Arc::new(net))
        }
        k => return Err(format!("sweep dump: a surface of kind `{k}`")),
    })
}

fn pcurve(p: &Pcurve) -> Result<Json,String> {
    let uv = |a: [f64;2]| reals(&a);
    Ok(match p {
        Pcurve::Line {a,b} => object([("kind","line".into()),("a",uv(*a)),("b",uv(*b))]),
        Pcurve::Inverse {a,b} => object([("kind","inverse".into()),("a",uv(*a)),("b",uv(*b))]),
        Pcurve::Curve(c) => object([("kind","curve".into()),("curve",curve(c)?)]),
    })
}
fn rpcurve(j: &Json) -> Result<Pcurve,String> {
    let uv = |k: &str| -> Result<[f64;2],String> { let r = rreals(field(j,k)?); Ok([r[0],r[1]]) };
    Ok(match field(j,"kind")?.as_str() {
        "line" => Pcurve::Line {a:uv("a")?,b:uv("b")?},
        "inverse" => Pcurve::Inverse {a:uv("a")?,b:uv("b")?},
        _ => Pcurve::Curve(Arc::new(rcurve(field(j,"curve")?)?)),
    })
}

/// **A B-rep written whole**: every vertex, edge and use as it is, each curve and surface by its
/// exact parameters (`read` gives it back bit for bit).
pub fn write(b: &Brep) -> Result<String,String> {
    Walls::alone();
    let vertices = Json::Arr(b.vertices.iter().map(|v| object([("p",v3(v.p)),("tol",v.tol.into())])).collect());
    let edges = b.edges.iter().map(|e| -> Result<Json,String> {
        let c = match &e.curve { EdgeCurve::Curve(c) => curve(c)?,EdgeCurve::Degenerate => Json::Null };
        Ok(object([("curve",c),("t",reals(&e.t)),("v",Json::Arr(vec![e.v[0].into(),e.v[1].into()])),("tol",e.tol.into())]))
    }).collect::<Result<Vec<_>,_>>()?;
    let faces = b.faces.iter().map(|f| -> Result<Json,String> {
        let loops = f.loops.iter().map(|l| -> Result<Json,String> {
            Ok(Json::Arr(l.iter().map(|c| -> Result<Json,String> {
                Ok(object([("edge",c.edge.into()),("reversed",c.reversed.into()),("pcurve",pcurve(&c.pcurve)?)]))
            }).collect::<Result<Vec<_>,_>>()?))
        }).collect::<Result<Vec<_>,_>>()?;
        Ok(object([("surface",surface(&f.surface)?),("reversed",f.reversed.into()),("name",f.name.clone().into()),("loops",Json::Arr(loops))]))
    }).collect::<Result<Vec<_>,_>>()?;
    Ok(object([("vertices",vertices),("edges",Json::Arr(edges)),("faces",Json::Arr(faces))]).dump(None))
}

/// The B-rep `write` wrote.
pub fn read(text: &str) -> Result<Brep,String> {
    Walls::alone();
    let j = parse(text)?;
    let vertices = field(&j,"vertices")?.arr().iter().map(|v| Ok(Vertex {p:rv3(field(v,"p")?),tol:num(v,"tol")?})).collect::<Result<Vec<_>,String>>()?;
    let edges = field(&j,"edges")?.arr().iter().map(|e| -> Result<Edge,String> {
        let c = field(e,"curve")?;
        let curve = if matches!(c,Json::Null) { EdgeCurve::Degenerate } else { EdgeCurve::Curve(rcurve(c)?) };
        let (t,v) = (rreals(field(e,"t")?),field(e,"v")?.arr());
        Ok(Edge {curve,t:[t[0],t[1]],v:[v[0].as_i64() as u32,v[1].as_i64() as u32],tol:num(e,"tol")?})
    }).collect::<Result<Vec<_>,_>>()?;
    let faces = field(&j,"faces")?.arr().iter().map(|f| -> Result<Face,String> {
        let loops = field(f,"loops")?.arr().iter().map(|l| l.arr().iter().map(|c| -> Result<Coedge,String> {
            Ok(Coedge {edge:field(c,"edge")?.as_i64() as u32,reversed:field(c,"reversed")?.as_bool(),pcurve:rpcurve(field(c,"pcurve")?)?})
        }).collect::<Result<Vec<_>,_>>()).collect::<Result<Vec<_>,_>>()?;
        Ok(Face {surface:rsurface(field(f,"surface")?)?,reversed:field(f,"reversed")?.as_bool(),loops,name:field(f,"name")?.as_str().to_string()})
    }).collect::<Result<Vec<_>,_>>()?;
    Ok(Brep {vertices,edges,faces})
}

/// What an arrangement was given: its domain, the reach its candidates were met within where they
/// were (#847, a sweep cut from a blank), its candidates, the meetings given it (#810) and its
/// tolerance.
#[derive(Clone,Debug)]
pub struct Arrangement { pub domain: Brep,pub reach: Option<Brep>,pub candidates: Vec<Brep>,pub given: Vec<Given>,pub tol: f64 }

impl Arrangement {
    /// The arrangement run again (`boolean::arrangement_given`, or `arrangement_reached` within its
    /// reach) over the candidates `keep` keeps, with the meetings given it.
    pub fn replay(&self,keep: &[usize]) -> Result<Complex,String> {
        let refs: Vec<&Brep> = keep.iter().map(|&k| &self.candidates[k]).collect();
        crate::util::warm::fresh(|| arrangement_reached(&self.domain,self.reach.as_ref(),&refs,&self.given,self.tol))
    }
    /// Every candidate's index.
    pub fn all(&self) -> Vec<usize> { (0..self.candidates.len()).collect() }
}

/// The meetings given an arrangement (#810), each by its two surfaces, its curves exactly, and the
/// points where the two touch beside them (#942 F3; none written where there are none).
fn givens(given: &[Given]) -> Result<Json,String> {
    Ok(Json::Arr(given.iter().map(|g| -> Result<Json,String> {
        let mut fields = vec![("surfaces",Json::Arr(vec![surface(&g.surfaces[0])?,surface(&g.surfaces[1])?])),
            ("curves",Json::Arr(g.curves.iter().map(curve).collect::<Result<Vec<_>,_>>()?))];
        if !g.touches.is_empty() { fields.push(("touches",Json::Arr(g.touches.iter().map(|&p| v3(p)).collect()))) }
        Ok(Json::Obj(fields.into_iter().map(|(k,v)| (k.to_string(),v)).collect()))
    }).collect::<Result<Vec<_>,_>>()?))
}
fn rgivens(j: &Json) -> Result<Vec<Given>,String> {
    j.arr().iter().map(|g| -> Result<Given,String> {
        let s = field(g,"surfaces")?.arr();
        let touches = g.get("touches").map_or(Vec::new(),|t| t.arr().iter().map(rv3).collect());
        Ok(Given {surfaces:[rsurface(&s[0])?,rsurface(&s[1])?],curves:field(g,"curves")?.arr().iter().map(rcurve).collect::<Result<_,_>>()?,touches})
    }).collect()
}

/// **The arrangement written into `dir`**: `domain.json`, `reach.json` where it was met within one,
/// `candidate-<k>.json` each, and `arrangement.json` (the tolerance, the count, the meetings given
/// it (#810) by their two surfaces and their curves exactly, and `why`, what the build was refused
/// for).
pub fn dump(dir: &std::path::Path,domain: &Brep,reach: Option<&Brep>,candidates: &[&Brep],given: &[Given],tol: f64,why: &str)
    -> Result<(),String> {
    let io = |e: std::io::Error| format!("sweep dump: {e}");
    let _walls = Walls::whole();
    std::fs::create_dir_all(dir).map_err(io)?;
    std::fs::write(dir.join("domain.json"),write(domain)?).map_err(io)?;
    if let Some(reach) = reach { std::fs::write(dir.join("reach.json"),write(reach)?).map_err(io)? }
    for (k,c) in candidates.iter().enumerate() { std::fs::write(dir.join(format!("candidate-{k}.json")),write(c)?).map_err(io)? }
    let meta = object([("tol",tol.into()),("candidates",candidates.len().into()),("given",givens(given)?),("why",why.into())]);
    std::fs::write(dir.join("arrangement.json"),meta.dump(Some(2))).map_err(io)
}

/// The arrangement `dump` wrote into `dir`.
pub fn load(dir: &std::path::Path) -> Result<Arrangement,String> {
    let io = |e: std::io::Error| format!("sweep dump: {}: {e}",dir.display());
    let _walls = Walls::whole();
    let meta = parse(&std::fs::read_to_string(dir.join("arrangement.json")).map_err(io)?)?;
    let n = field(&meta,"candidates")?.as_i64() as usize;
    let domain = read(&std::fs::read_to_string(dir.join("domain.json")).map_err(io)?)?;
    // (an arrangement met within no reach, or dumped before #847, has none)
    let reach = match std::fs::read_to_string(dir.join("reach.json")) {
        Ok(text) => Some(read(&text)?),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => return Err(io(e)),
    };
    let candidates = (0..n).map(|k| read(&std::fs::read_to_string(dir.join(format!("candidate-{k}.json"))).map_err(io)?)).collect::<Result<Vec<_>,_>>()?;
    // (an arrangement dumped before #810 gave none)
    let given = match meta.get("given") { None => Vec::new(),Some(g) => rgivens(g)? };
    Ok(Arrangement {domain,reach,candidates,given,tol:num(&meta,"tol")?})
}

/// **The fewest candidates whose arrangement still fails** (`fails` says of a replay whether it
/// does): each candidate dropped in turn and kept out wherever the arrangement fails without it,
/// until no one can be dropped — a set none of whose candidates can be left out, the smallest
/// Boolean to keep as a fast test. Starts from every candidate, which must fail.
pub fn reduce(a: &Arrangement,fails: &dyn Fn(&Result<Complex,String>) -> bool) -> Vec<usize> {
    let mut keep = a.all();
    if !fails(&a.replay(&keep)) { return keep }
    loop {
        let before = keep.len();
        let mut k = 0;
        while k < keep.len() {
            let trial: Vec<usize> = keep.iter().copied().enumerate().filter(|&(i,_)| i != k).map(|(_,c)| c).collect();
            if fails(&a.replay(&trial)) { keep = trial } else { k += 1 }
        }
        if keep.len() == before { return keep }
    }
}

/// Where `SOLVENT_SWEEP_DUMP` names a directory, a failed build's arrangement is written there.
pub fn dump_dir() -> Option<std::path::PathBuf> { std::env::var_os("SOLVENT_SWEEP_DUMP").map(std::path::PathBuf::from) }

/// A constant twist by its numbers.
fn screw(s: &crate::trajectory::Screw) -> Json {
    object([("origin",v3(s.origin)),("axis",v3(s.axis)),("ratio",s.ratio.into()),("advance",s.advance.into())])
}
fn rscrew(j: &Json) -> Result<crate::trajectory::Screw,String> {
    Ok(crate::trajectory::Screw {origin:rv3(field(j,"origin")?),axis:rv3(field(j,"axis")?),ratio:num(j,"ratio")?,advance:num(j,"advance")?})
}

/// **A trajectory by its structure** (#942 F5): a constant twist, one seen from another, and a twist
/// placed at a pose — what it is made of exactly. A path of pieces (its joins checked to a tolerance
/// not kept), a pace and a path read only by evaluating them are refused by name.
fn trajectory(t: &crate::trajectory::Trajectory) -> Result<Json,String> {
    use crate::trajectory::Trajectory;
    Ok(match t {
        Trajectory::Twist(s) => { let mut o = screw(s); o.set("kind","twist".into()); o }
        Trajectory::Relative {source,observer} => object([("kind","relative".into()),("source",trajectory(source)?),("observer",trajectory(observer)?)]),
        Trajectory::Placed {twist,at} => object([("kind","placed".into()),("twist",screw(twist)),
            ("r",Json::Arr(at.r.iter().map(|row| reals(row)).collect())),("p",v3(at.p))]),
        Trajectory::Pieces(_) => return Err("sweep dump: a path of pieces is not written".into()),
        Trajectory::Paced {..} => return Err("sweep dump: a paced path is not written".into()),
        Trajectory::General(p) => return Err(format!("sweep dump: a path read only by evaluating it ({p:?}) is not written")),
    })
}
fn rtrajectory(j: &Json) -> Result<crate::trajectory::Trajectory,String> {
    use crate::trajectory::{Pose,Trajectory};
    Ok(match field(j,"kind")?.as_str() {
        "twist" => Trajectory::Twist(rscrew(j)?),
        "relative" => Trajectory::Relative {source:Box::new(rtrajectory(field(j,"source")?)?),observer:Box::new(rtrajectory(field(j,"observer")?)?)},
        "placed" => {
            let rows = field(j,"r")?.arr();
            let r = std::array::from_fn(|k| { let x = rreals(&rows[k]); [x[0],x[1],x[2]] });
            Trajectory::Placed {twist:rscrew(field(j,"twist")?)?,at:Pose {r,p:rv3(field(j,"p")?)}}
        }
        k => return Err(format!("sweep dump: a trajectory of kind `{k}`")),
    })
}

/// **What a trim was given** (#942 F5): the tool, the motion and the roll membership reads the
/// sweep by, the candidates, the meetings given them, the bar the tool is meshed within, the
/// tolerance and how the trim reads (`trim::Trim`) — so a trim is run again apart from whatever made
/// its candidates (the general road's walls, a ring's fitted tubes, a fit of 100 s).
#[derive(Clone,Debug)]
pub struct Trimming {
    pub tool: Brep,pub path: crate::trajectory::Trajectory,pub roll: [f64;2],pub candidates: Vec<Brep>,pub given: Vec<Given>,
    pub bar: f64,pub tol: f64,pub trim: super::trim::Trim,
}

impl Trimming {
    /// The trim run again (`trim::trimmed`), checked and judged as the build was.
    pub fn replay(&self) -> Result<(Brep,super::trim::Report),super::closed::Unbuilt> {
        super::trim::trimmed(&self.tool,&self.path,self.roll,&self.candidates,&self.given,self.bar,self.tol,self.trim)
    }

    /// The candidates' faces whose surfaces pass within `within` of `p`, nearest first.
    pub fn faces_near(&self,p: V,within: f64) -> Vec<Near> {
        let mut out: Vec<Near> = self.candidates.iter().enumerate().flat_map(|(c,b)| b.faces.iter().enumerate().map(move |(f,face)| (c,f,face)))
            .filter_map(|(candidate,face,f)| {
                let off = f.surface.implicit(p);
                (off.abs() <= within).then(|| Near {candidate,face,kind:f.surface.kind(),name:f.name.clone(),off})
            }).collect();
        out.sort_by(|a,b| a.off.abs().total_cmp(&b.off.abs()));
        out
    }

    /// **Where the sweep's boundary crosses a line** (#942 F5): membership read at `steps` points
    /// along `at + s·dir`, `s` over `[−reach, reach]`; wherever it changes, the change halved to the
    /// tolerance, and the candidates' faces passing within a thousandth of `reach` of it — the surface
    /// the true boundary lies on, and how far each candidate stands off it.
    pub fn boundary_along(&self,at: V,dir: V,reach: f64,steps: usize) -> Result<Vec<Crossing>,String> {
        use super::member::{Solid,Swept};
        use crate::util::space::{add,scale};
        let solid = Solid::new(&self.tool,self.bar,1.)?;
        let swept = Swept {tool:&solid,path:&self.path,roll:self.roll};
        let point = |s: f64| add(at,scale(dir,s));
        let read = |s: f64| swept.member(point(s)).ok();
        let steps = steps.max(1);
        let mut out = Vec::new();
        let mut last = (-reach,read(-reach));
        for k in 1..=steps {
            let s = -reach+2.*reach*k as f64/steps as f64;
            let here = read(s);
            if here != last.1 {
                let (mut lo,mut hi) = (last.0,s);
                while hi-lo > self.tol { let m = 0.5*(lo+hi); if read(m) == last.1 { lo = m } else { hi = m } }
                let x = point(0.5*(lo+hi));
                out.push(Crossing {at:x,from:last.1,to:here,on:self.faces_near(x,1e-3*reach)});
            }
            last = (s,here);
        }
        Ok(out)
    }

    /// **What passes near `p`** (#942 F5): the candidates' faces within `within` of it
    /// (`faces_near`), their edges within `within` (each by its nearest of 256 points along it), and
    /// the given meetings' curves within `within` (each by its own foot of `p`).
    pub fn through(&self,p: V,within: f64) -> Through {
        use crate::util::space::distance;
        let mut edges = Vec::new();
        for (c,b) in self.candidates.iter().enumerate() {
            for (e,edge) in b.edges.iter().enumerate() {
                let EdgeCurve::Curve(cv) = &edge.curve else { continue };
                let d = (0..=256).map(|k| distance(cv.point(edge.t[0]+(edge.t[1]-edge.t[0])*k as f64/256.),p)).fold(f64::INFINITY,f64::min);
                if d <= within { edges.push((c,e,cv.kind(),d)) }
            }
        }
        let mut given = Vec::new();
        for (g,m) in self.given.iter().enumerate() {
            for c in &m.curves {
                let d = distance(c.point(c.inverse(p)),p);
                if d <= within { given.push((g,[m.surfaces[0].kind(),m.surfaces[1].kind()],c.kind(),d)) }
            }
        }
        Through {faces:self.faces_near(p,within),edges,given}
    }
}

/// A candidate's face passing near a point: which candidate and face, its surface's kind, its name,
/// and the surface's signed distance there (`Surface::implicit`).
#[derive(Clone,Debug)]
pub struct Near { pub candidate: usize,pub face: usize,pub kind: &'static str,pub name: String,pub off: f64 }

/// Where membership changes along a line (`Trimming::boundary_along`): the point, membership before
/// and after it (`None` where it did not settle), and the faces passing near it.
#[derive(Clone,Debug)]
pub struct Crossing { pub at: V,pub from: Option<bool>,pub to: Option<bool>,pub on: Vec<Near> }

/// What passes near a point (`Trimming::through`): the candidates' faces, their edges (candidate,
/// edge, the curve's kind, how near) and the given meetings' curves (meeting, its two surfaces'
/// kinds, the curve's kind, how near).
#[derive(Clone,Debug)]
pub struct Through { pub faces: Vec<Near>,pub edges: Vec<(usize,usize,&'static str,f64)>,pub given: Vec<(usize,[&'static str;2],&'static str,f64)> }

/// **A trim written into `dir`**: `tool.json`, `candidate-<k>.json` each, and `trim.json` (the
/// tolerance, the bar, the roll, the trim's reading, the trajectory, the meetings given and `why`,
/// what the build was refused for).
#[allow(clippy::too_many_arguments)]
pub fn dump_trim(dir: &std::path::Path,tool: &Brep,path: &crate::trajectory::Trajectory,roll: [f64;2],candidates: &[Brep],given: &[Given],
    bar: f64,tol: f64,trim: super::trim::Trim,why: &str) -> Result<(),String> {
    let io = |e: std::io::Error| format!("sweep dump: {e}");
    let _walls = Walls::whole();
    let path = trajectory(path)?;
    std::fs::create_dir_all(dir).map_err(io)?;
    std::fs::write(dir.join("tool.json"),write(tool)?).map_err(io)?;
    for (k,c) in candidates.iter().enumerate() { std::fs::write(dir.join(format!("candidate-{k}.json")),write(c)?).map_err(io)? }
    let reading = object([("resolution",trim.resolution.into()),("hair",trim.hair.into()),
        ("reach",trim.reach.map_or(Json::Null,|(lo,hi)| Json::Arr(vec![v3(lo),v3(hi)])))]);
    let meta = object([("tol",tol.into()),("bar",bar.into()),("roll",reals(&roll)),("trim",reading),("trajectory",path),
        ("candidates",candidates.len().into()),("given",givens(given)?),("why",why.into())]);
    std::fs::write(dir.join("trim.json"),meta.dump(Some(2))).map_err(io)
}

/// The trim `dump_trim` wrote into `dir`.
pub fn load_trim(dir: &std::path::Path) -> Result<Trimming,String> {
    let io = |e: std::io::Error| format!("sweep dump: {}: {e}",dir.display());
    let _walls = Walls::whole();
    let meta = parse(&std::fs::read_to_string(dir.join("trim.json")).map_err(io)?)?;
    let n = field(&meta,"candidates")?.as_i64() as usize;
    let tool = read(&std::fs::read_to_string(dir.join("tool.json")).map_err(io)?)?;
    let candidates = (0..n).map(|k| read(&std::fs::read_to_string(dir.join(format!("candidate-{k}.json"))).map_err(io)?)).collect::<Result<Vec<_>,_>>()?;
    let t = field(&meta,"trim")?;
    let reach = match field(t,"reach")? { Json::Null => None,r => { let r = r.arr(); Some((rv3(&r[0]),rv3(&r[1]))) } };
    let trim = super::trim::Trim {resolution:num(t,"resolution")?,hair:num(t,"hair")?,reach};
    Ok(Trimming {tool,path:rtrajectory(field(&meta,"trajectory")?)?,roll:pair(&meta,"roll")?,candidates,given:rgivens(field(&meta,"given")?)?,
        bar:num(&meta,"bar")?,tol:num(&meta,"tol")?,trim})
}
