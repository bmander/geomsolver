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
//! B-spline; a kind none of them is (a trace) is refused by name.
use crate::boolean::arrangement_within;
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
        Surface::Family(..) => return Err("sweep dump: a face family's wall is not written".into()),
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

/// What an arrangement was given: its domain, its candidates and its tolerance.
#[derive(Clone,Debug)]
pub struct Arrangement { pub domain: Brep,pub candidates: Vec<Brep>,pub tol: f64 }

impl Arrangement {
    /// The arrangement run again (`boolean::arrangement_within`) over the candidates `keep` keeps.
    pub fn replay(&self,keep: &[usize]) -> Result<Complex,String> {
        let refs: Vec<&Brep> = keep.iter().map(|&k| &self.candidates[k]).collect();
        crate::util::warm::fresh(|| arrangement_within(&self.domain,&refs,self.tol))
    }
    /// Every candidate's index.
    pub fn all(&self) -> Vec<usize> { (0..self.candidates.len()).collect() }
}

/// **The arrangement written into `dir`**: `domain.json`, `candidate-<k>.json` each, and
/// `arrangement.json` (the tolerance, the count, and `why`, what the build was refused for).
pub fn dump(dir: &std::path::Path,domain: &Brep,candidates: &[&Brep],tol: f64,why: &str) -> Result<(),String> {
    let io = |e: std::io::Error| format!("sweep dump: {e}");
    std::fs::create_dir_all(dir).map_err(io)?;
    std::fs::write(dir.join("domain.json"),write(domain)?).map_err(io)?;
    for (k,c) in candidates.iter().enumerate() { std::fs::write(dir.join(format!("candidate-{k}.json")),write(c)?).map_err(io)? }
    let meta = object([("tol",tol.into()),("candidates",candidates.len().into()),("why",why.into())]);
    std::fs::write(dir.join("arrangement.json"),meta.dump(Some(2))).map_err(io)
}

/// The arrangement `dump` wrote into `dir`.
pub fn load(dir: &std::path::Path) -> Result<Arrangement,String> {
    let io = |e: std::io::Error| format!("sweep dump: {}: {e}",dir.display());
    let meta = parse(&std::fs::read_to_string(dir.join("arrangement.json")).map_err(io)?)?;
    let n = field(&meta,"candidates")?.as_i64() as usize;
    let domain = read(&std::fs::read_to_string(dir.join("domain.json")).map_err(io)?)?;
    let candidates = (0..n).map(|k| read(&std::fs::read_to_string(dir.join(format!("candidate-{k}.json"))).map_err(io)?)).collect::<Result<Vec<_>,_>>()?;
    Ok(Arrangement {domain,candidates,tol:num(&meta,"tol")?})
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
