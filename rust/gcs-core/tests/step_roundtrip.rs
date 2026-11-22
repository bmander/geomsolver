//! **Every file the kernel writes reads back to the same solid** (#993 stage 1, the first clause
//! of its gate), stated as one property (`trip`) and held over what the kernel writes: the solids
//! of the example library (`census`, a tool of minutes) and a sample of them chosen to cover every
//! path of the writer (`the_sample_reads_back`, on every push).
//!
//! **The same solid.** `step::write` writes a `Brep` as one `MANIFOLD_SOLID_BREP` on one
//! `CLOSED_SHELL` holding every face, whatever its lumps and voids (a `Brep` is a bag of faces; the
//! writer says nothing of which shell bounds which): so a body of several lumps, or with a void,
//! is one solid of the file whose one shell is several closed ones, and "the same" is that one
//! solid, its volume the lumps' sum less the voids. For `b` and `text = write(b, …)`, `r` the one
//! solid `read(text)` gives is the same solid when:
//!
//! * a face for each face `b` wrote, in its order, read from the `ADVANCED_FACE` written for it,
//!   on a surface of the kind written and of the same sense (`step_check::Solid`), its numbers
//!   the written ones to 1e-12 (a swept surface's frame is the reader's own, its curve compared
//!   through its edges);
//! * an edge for each `EDGE_CURVE` written and a vertex for each `VERTEX_POINT`, each read from
//!   that entity and no other, the edge on a curve of the kind written running between the
//!   vertices written, its middle on the edge built within what the writer's fit allows, each
//!   vertex where it was written; and besides those only what the writer leaves out for a reader
//!   to lay (`Given` names them 0): a pole's degenerate edge for each `b` holds, and nothing else;
//! * each face's area and flux (`props::area`, `props::fluxes`) the built face's, and the solid's
//!   volume and area, within `Bars`: the measure's own noise (1e-9) where every curve and surface
//!   written is the kernel's own, and where the writer fits one, as far as a fit within its bar
//!   can move them (`step::EXACT` for a traced meeting written `Fit::Exact`, `Fit::bar` for one at
//!   a stated tolerance, `step::FIT` for a loft's or a family's net and the curves on them);
//! * `r`, written again, passes `step_check::verify` and reads again to the same solid at the
//!   bars of an exact solid: a second trip changes nothing more.
//!
//! What a solid comes to is an `Outcome`: the same; not written (the writer refused it, which is
//! the writer's say and no reading's); refused by `context`, by `topology` or by `sound` (for
//! `sound`, the solid `topology` read is held to the property anyway: the same solid there is a
//! false refusal, a different one a true refusal); read to a different solid (the worst); same
//! on the first trip and not on the second; a panic.
use gcs_core::brep::geom::{Curve,Surface};
use gcs_core::brep::props::{area,fluxes,volume};
use gcs_core::brep::step::{face_net,write,Fit,FIT};
use gcs_core::brep::step_check::{verify,Solid};
use gcs_core::brep::step_read::{context,sound,topology,Given,Refusal,Why};
use gcs_core::brep::topo::{Brep,EdgeCurve,Pcurve};
use gcs_core::space::distance;
use std::collections::{BTreeMap,BTreeSet};
use std::time::{Duration,Instant};

/// Which part of the reader refused a file.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Stage { Context,Topology,Sound }

/// What one solid's round trip came to.
#[derive(Clone,Debug)]
pub enum Outcome {
    Same,
    /// The writer refused it (its message).
    NotWritten(String),
    /// The reader refused it: by which part, its refusal, and for `sound`, whether the solid
    /// `topology` read is the same solid (`Ok`) or how it differs (`Err`).
    Refused { by: Stage,refusal: Refusal,unsound: Option<Result<(),String>> },
    /// Read to a different solid: what differs.
    Different(String),
    /// The same on the first trip; the solid read, written and read again, is not (why).
    SecondTrip(String),
    /// A panic, its message.
    Panicked(String),
    /// Not read within `BOUND` (what it was).
    Slow(String),
    /// The same point set and every written entity read, but the reader lays fewer degenerate
    /// edges than the solid held (laid, built): which ones it should lay is the convention.
    Degenerate(usize,usize),
}

impl Outcome {
    pub fn label(&self) -> String {
        match self {
            Outcome::Same => "same".into(),
            Outcome::NotWritten(_) => "not written".into(),
            // (the writer puts every face of a body of several lumps on one shell of one solid,
            // which `sound` refuses as a second solid: the file's defect, not the reader's)
            Outcome::Refused {by,refusal,unsound:Some(Ok(()))} if refusal.msg.contains("a second solid") =>
                format!("refused by {by:?}: several lumps written as one shell (read without it: the same solid)"),
            Outcome::Refused {by,unsound:Some(Ok(())),..} => format!("refused by {by:?}, falsely (read without it: the same solid)"),
            Outcome::Refused {by,unsound:Some(Err(_)),..} => format!("refused by {by:?}, truly (read without it: a different solid)"),
            Outcome::Refused {by,..} => format!("refused by {by:?}"),
            Outcome::Different(_) => "read to a different solid".into(),
            Outcome::SecondTrip(_) => "a second trip differs".into(),
            Outcome::Panicked(_) => "panicked".into(),
            Outcome::Slow(m) if m.starts_with("too large") => "written too large to read".into(),
            Outcome::Slow(_) => format!("not read within {BOUND:?}"),
            Outcome::Degenerate(..) => "same but for degenerate edges".into(),
        }
    }
    pub fn detail(&self) -> String {
        match self {
            Outcome::Same => String::new(),
            Outcome::Degenerate(l,b) => format!("{l} degenerate edges laid, {b} built"),
            Outcome::NotWritten(m) | Outcome::Different(m) | Outcome::SecondTrip(m) | Outcome::Panicked(m) | Outcome::Slow(m) => m.clone(),
            Outcome::Refused {refusal,unsound,..} => {
                let mut s = format!("{:?} {refusal}",refusal.why);
                if let Some(Err(d)) = unsound { s += &format!(" [read without sound: {d}]"); }
                s
            }
        }
    }
}

/// What a round trip found beside its outcome.
#[derive(Clone,Debug,Default)]
pub struct Found {
    /// The worst face's difference in units of its bar (area or flux, whichever is worse), the
    /// face, and its differences (area, flux), relative to the face's own.
    pub worst: Option<(f64,usize,f64,f64)>,
    /// The entities the reader laid itself (`Given`'s 0), by what they are.
    pub laid: BTreeMap<&'static str,usize>,
    /// The edges read looser than the file states (its measured tolerance, the stated one).
    pub loose: Vec<(usize,f64,f64)>,
    /// The vertices read looser than the file states, not at a pole.
    pub loose_vertices: Vec<(usize,f64,f64)>,
    /// The degenerate edges laid and built, where they differ and nothing else does.
    pub degenerate: Option<(usize,usize)>,
    pub write: Duration,
    pub read: Duration,
    pub measure: Duration,
}

/// The measures of `b`'s faces: each face's area and flux, and its boundary's length.
pub struct Measured { area: Vec<f64>,flux: Vec<f64>,length: Vec<f64>,volume: f64 }

pub fn measured(b: &Brep) -> Measured {
    let area: Vec<f64> = b.faces.iter().map(|f| area(b,f)).collect();
    let length = b.faces.iter().map(|f| f.loops.iter().flatten().map(|c| edge_length(b,c.edge as usize)).sum()).collect();
    Measured {area,flux:fluxes(b),length,volume:volume(b)}
}

/// An edge's length, by its chords at 64 points (a bar's scale, not a measure).
fn edge_length(b: &Brep,ei: usize) -> f64 {
    let e = &b.edges[ei];
    let p: Vec<_> = (0..=64).map(|k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/64.,&b.vertices)).collect();
    p.windows(2).map(|w| distance(w[0],w[1])).sum()
}

/// The size of `b`: its vertices' box's diagonal, and the farthest the box stands from the origin
/// (what a flux scales by).
fn size(b: &Brep) -> (f64,f64) {
    let pts: Vec<_> = b.edges.iter().flat_map(|e| (0..=8).map(move |k| e.t[0]+(e.t[1]-e.t[0])*k as f64/8.).map(move |t| (e,t)))
        .map(|(e,t)| e.point(t,&b.vertices)).chain(b.vertices.iter().map(|v| v.p)).collect();
    let lo: [f64;3] = std::array::from_fn(|k| pts.iter().map(|p| p[k]).fold(f64::INFINITY,f64::min));
    let hi: [f64;3] = std::array::from_fn(|k| pts.iter().map(|p| p[k]).fold(f64::NEG_INFINITY,f64::max));
    if pts.is_empty() { return (1.,1.) }
    let far = (0..8).map(|c| { let p: [f64;3] = std::array::from_fn(|k| if c>>k & 1 == 1 { hi[k] } else { lo[k] }); gcs_core::space::norm(p) })
        .fold(0.,f64::max);
    (distance(lo,hi).max(1e-9),far.max(distance(lo,hi)))
}

/// How far the file's curve of `b`'s edge `ei` may stand from the edge built (millimetres): none
/// where the writer writes it as itself, else the bar it fits it to.
fn edge_fit(b: &Brep,ei: usize,fit: Fit,tol: f64,polyline: bool) -> f64 {
    match &b.edges[ei].curve {
        EdgeCurve::Degenerate => 0.,
        EdgeCurve::Curve(c) => match c {
            Curve::Line {..} | Curve::Circle(..) | Curve::Ellipse(..) | Curve::BSpline(_) => 0.,
            // (the polyline a traced curve is written as where no cubic closes in: its bar the
            // stated uncertainty where no tolerance was asked, else the fit's)
            Curve::Traced(_) if polyline => match fit { Fit::Exact => tol.max(1e-7),Fit::Within(_) => fit.bar() },
            Curve::Traced(_) => fit.bar(),
            _ => FIT,
        },
    }
}

/// How far the file's surface of face `fi` may stand from the face built: none where it is
/// written as itself or as an exact net (a sheet, an affine image of a quadric, a revolution or
/// an extrusion of a spline), `FIT` where its net or its swept curve is fitted.
fn surface_fit(b: &Brep,fi: usize) -> f64 {
    fn fitted(s: &Surface) -> bool {
        match s {
            Surface::Blend(..) | Surface::Family(..) => true,
            Surface::Extrusion(_,c) | Surface::Revolution(_,c) => c.exact_span().is_some(),
            Surface::Affine(m) => fitted(&m.of),
            _ => false,
        }
    }
    if fitted(&b.faces[fi].surface) { FIT } else { 0. }
}

/// **The bars a reading of `b` is held to**, face by face: `area[i]`, `flux[i]`, and the volume's
/// and the total area's. A face of area `A`, its boundary `L` long, in a solid of size `s` whose
/// box stands at most `R` from the origin:
///
/// * the measure's own noise, 1e-9 of `A + L s` (the existing gate's bar where nothing is fitted;
///   `L s` so a sliver of a face is held to its solid's scale, not its own vanishing area);
/// * an edge whose written curve stands up to `δ` off the built one moves the face's trim by as
///   much along it: `2 δ L` (twice, as a fitted cubic's miss keeps its sign along a piece); and
///   so does a pcurve the writer fits (`Fit::bar` on the face), which a reader may trim by;
/// * a surface written within `δₛ` of the built one moves the face along its normal: its area by
///   `δₛ ∬ 2H`, held as `20 δₛ A / s` (its radius of curvature no less than a tenth of the solid's
///   size; a face curved tighter that fails this is examined, not taken as a defect outright);
/// * a flux is the area's bar times `R`, and `3 δₛ A` for the surface's move (`div X = 3`);
/// * the volume's is the fluxes' over 3 plus 1e-9 of it, the total area's the faces' sum.
pub struct Bars { area: Vec<f64>,flux: Vec<f64>,volume: f64,total: f64 }

fn bars(b: &Brep,m: &Measured,fit: Fit,tol: f64,polylines: &BTreeSet<usize>) -> Bars {
    let (s,far) = size(b);
    let edges: Vec<f64> = (0..b.edges.len()).map(|ei| edge_fit(b,ei,fit,tol,polylines.contains(&ei))).collect();
    let mut out = Bars {area:Vec::new(),flux:Vec::new(),volume:0.,total:0.};
    for (fi,f) in b.faces.iter().enumerate() {
        let ds = surface_fit(b,fi);
        // (each use's pcurve: exact where the writer writes it as a line, a plane's own conic or
        // spline, or the kernel's own spline; else a cubic within `fit`'s bar on the face, which a
        // reader may trim the face by)
        let pcurve = |c: &gcs_core::brep::topo::Coedge| -> f64 {
            let EdgeCurve::Curve(curve) = &b.edges[c.edge as usize].curve else { return 0. };
            let exact = match (&c.pcurve,&f.surface) {
                (Pcurve::Line {..},_) => true,
                (Pcurve::Curve(pc),_) if matches!(**pc,Curve::BSpline(_)) => true,
                (_,Surface::Plane(_)) => matches!(curve,Curve::Line {..} | Curve::Circle(..) | Curve::BSpline(_)),
                _ => false,
            };
            if exact { 0. } else { fit.bar() }
        };
        let de = f.loops.iter().flatten().map(|c| edges[c.edge as usize].max(pcurve(c))).fold(ds,f64::max);
        let (a,l) = (m.area[fi].abs(),m.length[fi]);
        let ba = 1e-9*(a+l*s)+2.*de*l+20.*ds*a/s;
        let bf = far*ba+3.*ds*a;
        out.area.push(ba);
        out.flux.push(bf);
        out.volume += bf/3.;
        out.total += ba;
    }
    out.volume += 1e-9*m.volume.abs();
    out
}

/// The `#n` of each entity of `kind` in `text`, in the file's order.
fn ids(text: &str,kind: &str) -> Vec<u32> {
    let head = format!(" = {kind}(");
    text.lines().filter(|l| l.contains(&head)).filter_map(|l| l.strip_prefix('#')?.split(' ').next()?.parse().ok()).collect()
}

/// The kind a written edge's curve is read as: a line, a conic and a spline as themselves, any
/// other curve as the spline the writer fits it with.
fn written_kind(c: &EdgeCurve) -> &'static str {
    match c { EdgeCurve::Curve(Curve::Line {..}) => "line",EdgeCurve::Curve(Curve::Circle(..)) => "circle",
        EdgeCurve::Curve(Curve::Ellipse(..)) => "ellipse",EdgeCurve::Degenerate => "degenerate",_ => "bspline" }
}

/// The edges of `b` whose traced curve the file writes as a polyline (a degree-one spline read).
fn polylines(b: &Brep,given: &Given,edge_ids: &[u32]) -> BTreeSet<usize> {
    let at: BTreeMap<u32,usize> = edge_ids.iter().enumerate().map(|(k,&id)| (id,k)).collect();
    let written: Vec<usize> = (0..b.edges.len()).filter(|&ei| matches!(b.edges[ei].curve,EdgeCurve::Curve(_))).collect();
    given.edges.iter().enumerate().filter_map(|(i,id)| {
        let k = *at.get(id)?;
        let ei = *written.get(k)?;
        let linear = matches!(&given.brep.edges[i].curve,EdgeCurve::Curve(Curve::BSpline(s)) if s.degree == 1);
        (linear && matches!(b.edges[ei].curve,EdgeCurve::Curve(Curve::Traced(_)))).then_some(ei)
    }).collect()
}

/// **Whether `r`, read from `text`, the file `write` made of `b`, is the same solid** — every
/// clause but the second trip's (see the file's head) — and what was found on the way: `Err`
/// says the first clause that fails, with its numbers.
pub fn same(b: &Brep,mb: &Measured,text: &str,given: &Given,fit: Fit,tol: f64,found: &mut Found) -> Result<(),String> {
    let r = &given.brep;
    let (s,_) = size(b);
    // the faces: one for each written, in its order, each from its own ADVANCED_FACE
    let face_ids = ids(text,"ADVANCED_FACE");
    if given.faces != face_ids {
        return Err(format!("{} faces read, {} written ({:?} against {:?})",given.faces.len(),face_ids.len(),
            &given.faces[..given.faces.len().min(6)],&face_ids[..face_ids.len().min(6)]))
    }
    let (written,got) = (Solid::of(b),Solid::of(r));
    for (k,(x,y)) in got.faces.iter().zip(&written.faces).enumerate() {
        if (x.kind,x.reversed) != (y.kind,y.reversed) {
            return Err(format!("face {k} (#{}) read of kind {} reversed {}, written {} reversed {}",face_ids[k],x.kind,x.reversed,y.kind,y.reversed))
        }
        if x.kind == 7 || x.kind == 8 { continue }
        if x.numbers.len() != y.numbers.len()
            || x.numbers.iter().zip(&y.numbers).any(|(p,q)| (p-q).abs() > 1e-12*q.abs().max(1.)) {
            let worst = x.numbers.iter().zip(&y.numbers).map(|(p,q)| (p-q).abs()/q.abs().max(1.)).fold(0.,f64::max);
            return Err(format!("face {k} (#{})'s surface read with other numbers ({} of them, {} written; worst {worst:.2e})",
                face_ids[k],x.numbers.len(),y.numbers.len()))
        }
    }
    // the edges and vertices: each written one once, from its own entity; besides them only
    // what the reader lays (0)
    let (edge_ids,vertex_ids) = (ids(text,"EDGE_CURVE"),ids(text,"VERTEX_POINT"));
    let named = |v: &[u32]| v.iter().copied().filter(|&e| e != 0).collect::<Vec<_>>();
    let (mut re,mut rv) = (named(&given.edges),named(&given.vertices));
    re.sort();
    rv.sort();
    if re != edge_ids { return Err(format!("{} edges read from the file's entities, {} written",re.len(),edge_ids.len())) }
    if rv != vertex_ids { return Err(format!("{} vertices read from the file's entities, {} written",rv.len(),vertex_ids.len())) }
    let written_edges: Vec<usize> = (0..b.edges.len()).filter(|&ei| matches!(b.edges[ei].curve,EdgeCurve::Curve(_))).collect();
    let used: Vec<usize> = { let mut u = vec![false;b.vertices.len()]; for &ei in &written_edges { for v in b.edges[ei].v { u[v as usize] = true; } }
        (0..b.vertices.len()).filter(|&v| u[v]).collect() };
    let edge_of: BTreeMap<u32,usize> = edge_ids.iter().zip(&written_edges).map(|(&id,&ei)| (id,ei)).collect();
    let vertex_of: BTreeMap<u32,usize> = vertex_ids.iter().zip(&used).map(|(&id,&v)| (id,v)).collect();
    for (i,&id) in given.vertices.iter().enumerate() {
        if id == 0 { *found.laid.entry("a pole's vertex").or_default() += 1; continue }
        let off = distance(r.vertices[i].p,b.vertices[vertex_of[&id]].p);
        if off > 1e-12*s { return Err(format!("vertex #{id} read {off:.2e} from where it was written")) }
    }
    let polys = polylines(b,given,&edge_ids);
    let mut laid_degenerate = 0;
    for (i,&id) in given.edges.iter().enumerate() {
        let e = &r.edges[i];
        if id == 0 {
            match e.curve {
                EdgeCurve::Degenerate => { laid_degenerate += 1; *found.laid.entry("a pole's degenerate edge").or_default() += 1; }
                EdgeCurve::Curve(_) => { *found.laid.entry("an edge on a curve").or_default() += 1; }
            }
            continue
        }
        let ei = edge_of[&id];
        let o = &b.edges[ei];
        if written_kind(&e.curve) != written_kind(&o.curve) {
            return Err(format!("edge #{id} read as a {}, written as a {}",written_kind(&e.curve),written_kind(&o.curve)))
        }
        let ends = e.v.map(|v| given.vertices[v as usize]);
        let built = o.v.map(|v| vertex_ids[used.iter().position(|&u| u == v as usize).unwrap()]);
        if ends != built { return Err(format!("edge #{id} read between #{} and #{}, written between #{} and #{}",ends[0],ends[1],built[0],built[1])) }
        let EdgeCurve::Curve(c) = &e.curve else { continue };
        let EdgeCurve::Curve(oc) = &o.curve else { continue };
        let bar = 2.*edge_fit(b,ei,fit,tol,polys.contains(&ei))+1e-8*s;
        for k in 1..4 {
            let m = c.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/4.);
            let off = distance(oc.point(oc.inverse(m)),m);
            if off > bar { return Err(format!("edge #{id} ({}) stands {off:.2e} off the edge built, past {bar:.1e}",oc.kind())) }
        }
    }
    // (a vertex no edge ends at is the kernel's bookkeeping, which no file writes)
    let degenerate = b.edges.iter().filter(|e| matches!(e.curve,EdgeCurve::Degenerate)).count();
    let ended = b.edges.iter().flat_map(|e| e.v).collect::<BTreeSet<_>>().len();
    // (a degenerate edge the writer leaves out where no pole of the surface lies, a net's side
    // collapsed to a point, which the reader does not lay again: the point set is the same, its
    // B-rep one degenerate edge short; noted and measured on)
    if laid_degenerate != degenerate && r.edges.len()+degenerate == b.edges.len()+laid_degenerate && r.vertices.len() == ended {
        found.degenerate = Some((laid_degenerate,degenerate));
    }
    else if laid_degenerate != degenerate || r.edges.len() != b.edges.len() || r.vertices.len() != ended {
        return Err(format!("{} edges and {} vertices read ({laid_degenerate} degenerate laid), {} and {ended} built ({degenerate} degenerate)",
            r.edges.len(),r.vertices.len(),b.edges.len()))
    }
    // the tolerances the reader measures, against the uncertainty the file states
    let stated = given.context.uncertainty.unwrap_or(0.);
    for (i,e) in r.edges.iter().enumerate() { if e.tol > stated*(1.+1e-6) { found.loose.push((i,e.tol,stated)); } }
    for (i,v) in r.vertices.iter().enumerate() {
        if v.tol > stated*(1.+1e-6) && given.vertices[i] != 0 { found.loose_vertices.push((i,v.tol,stated)); }
    }
    // the measures, face by face
    let started = Instant::now();
    let mr = measured(r);
    found.measure += started.elapsed();
    let bar = bars(b,mb,fit,tol,&polys);
    let mut worst: Option<(f64,usize,f64,f64)> = None;
    for fi in 0..b.faces.len() {
        let (da,df) = ((mr.area[fi]-mb.area[fi]).abs(),(mr.flux[fi]-mb.flux[fi]).abs());
        let ratio = (da/bar.area[fi]).max(df/bar.flux[fi]);
        if !(ratio <= worst.map_or(-1.,|w| w.0)) || ratio.is_nan() {
            worst = Some((ratio,fi,da/mb.area[fi].abs().max(1e-300),df/mb.flux[fi].abs().max(1e-300)));
        }
    }
    found.worst = worst;
    if let Some((ratio,fi,ra,rf)) = worst {
        if !(ratio <= 1.) {
            return Err(format!("face {fi} (#{}, a {}): area {:.12e} read, {:.12e} built ({ra:.1e}); flux {:.12e}, {:.12e} ({rf:.1e}); {ratio:.1} bars",
                face_ids[fi],b.faces[fi].surface.kind(),mr.area[fi],mb.area[fi],mr.flux[fi],mb.flux[fi]))
        }
    }
    let (ta,tb) = (mr.area.iter().sum::<f64>(),mb.area.iter().sum::<f64>());
    if !((mr.volume-mb.volume).abs() <= bar.volume) { return Err(format!("volume {:.12e} read, {:.12e} built",mr.volume,mb.volume)) }
    if !((ta-tb).abs() <= bar.total) { return Err(format!("area {ta:.12e} read, {tb:.12e} built")) }
    Ok(())
}

/// `text` read part by part, as `step_read::reading` reads it: the solids, or the part that
/// refused it and its refusal, and where `sound` refused, the solids `topology` read.
pub fn stages(text: &str) -> Result<Vec<Given>,(Stage,Refusal,Option<Vec<Given>>)> {
    let file = context::parse(text).map_err(|r| (Stage::Context,r,None))?;
    let shapes = context::shapes(&file).map_err(|r| (Stage::Context,r,None))?;
    if shapes.solids.is_empty() { return Err((Stage::Context,Refusal::new(Why::Empty,None,"the file represents no solid"),None)) }
    let mut out = Vec::new();
    let mut refused: Option<Refusal> = None;
    for f in shapes.solids {
        let placed = f.placed.clone();
        let mut given = topology::solid(&file,f).map_err(|r| (Stage::Topology,r,None))?;
        if let Err(u) = sound::sound(&given.brep,&given.context) { refused.get_or_insert(u.refusal(&given)); }
        if let Some(p) = placed { given.brep = given.brep.moved(&p.motion); }
        out.push(given);
    }
    match refused { Some(r) => Err((Stage::Sound,r,Some(out))),None => Ok(out) }
}

/// The longest a file the kernel writes may take to read: a minute. The slowest of the library
/// read in under half a second on this machine; a file past this is a defect of its size or of
/// the reader's pace, said apart.
pub const BOUND: Duration = Duration::from_secs(60);

/// The largest file the census reads: 64 MB. The library's largest besides the few past it is
/// some megabytes; past it, the file itself is the defect (a polyline's millions of points).
pub const LARGEST: usize = 64<<20;

/// The message of a panic.
fn said(p: Box<dyn std::any::Any + Send>) -> String {
    p.downcast_ref::<&str>().map(|s| s.to_string()).or_else(|| p.downcast_ref::<String>().cloned()).unwrap_or_else(|| "a panic".into())
}

/// **One solid's round trip**, written as `fit` says stating `tol`: its outcome, and what was found.
pub fn trip(b: &Brep,name: &str,tol: f64,fit: Fit) -> (Outcome,Found) {
    let mut found = Found::default();
    let got = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| trip_in(b,name,tol,fit,&mut found)));
    let outcome = got.unwrap_or_else(|p| Outcome::Panicked(said(p)));
    (outcome,found)
}

fn trip_in(b: &Brep,name: &str,tol: f64,fit: Fit,found: &mut Found) -> Outcome {
    let say = |what: &str| if std::env::var_os("STEP_ROUNDTRIP_VERBOSE").is_some() { eprintln!("  {name}: {what}") };
    say("writing");
    let started = Instant::now();
    let text = match write(b,name,tol,fit) { Ok(t) => t,Err(e) => return Outcome::NotWritten(e) };
    found.write = started.elapsed();
    say(&format!("written in {:?}, {} bytes; reading",found.write,text.len()));
    // (a file past `LARGEST` is said so and not read: a reader's time and memory grow with it, and
    // the census would take hours over a few such)
    if text.len() > LARGEST {
        return Outcome::Slow(format!("too large: a file of {} bytes, written in {:.1?}, not read",text.len(),found.write))
    }
    if let Ok(dir) = std::env::var("STEP_ROUNDTRIP_DUMP") {
        let file: String = name.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect();
        let _ = std::fs::write(std::path::Path::new(&dir).join(format!("{file}.step")),&text);
    }
    let started = Instant::now();
    // (read on a thread of its own, waited on no longer than `BOUND`: one past it is said slow, its
    // thread left to run)
    let read = {
        let (tx,rx) = std::sync::mpsc::channel();
        let owned = text.clone();
        std::thread::spawn(move || { let _ = tx.send(std::panic::catch_unwind(|| stages(&owned))); });
        match rx.recv_timeout(BOUND) {
            Ok(Ok(r)) => r,
            Ok(Err(p)) => return Outcome::Panicked(said(p)),
            Err(_) => return Outcome::Slow(format!("a file of {} bytes, written in {:.1?}, not read within {BOUND:?}",text.len(),found.write)),
        }
    };
    say(&format!("read in {:?}; measuring",started.elapsed()));
    found.read = started.elapsed();
    let started = Instant::now();
    let mb = measured(b);
    found.measure = started.elapsed();
    let given = match read {
        Ok(g) => g,
        Err((by,refusal,unsound)) => {
            let unsound = unsound.map(|g| if g.len() == 1 { same(b,&mb,&text,&g[0],fit,tol,&mut Found::default()) }
                else { Err(format!("{} solids", g.len())) });
            return Outcome::Refused {by,refusal,unsound}
        }
    };
    if given.len() != 1 { return Outcome::Different(format!("{} solids read, one written",given.len())) }
    say("compared; held to the property");
    if let Err(d) = same(b,&mb,&text,&given[0],fit,tol,found) { return Outcome::Different(d) }
    // the second trip: the solid read, written again, is a file of itself and reads to itself
    say("the second trip");
    let r = &given[0].brep;
    let again = match write(r,name,tol,fit) { Ok(t) => t,Err(e) => return Outcome::SecondTrip(format!("not written again: {e}")) };
    if let Err(e) = verify(&again,&Solid::of(r)) { return Outcome::SecondTrip(format!("written again, not verified: {e}")) }
    let twice = match stages(&again) { Ok(g) => g,Err((by,refusal,_)) => return Outcome::SecondTrip(format!("refused by {by:?}: {refusal}")) };
    if twice.len() != 1 { return Outcome::SecondTrip(format!("{} solids read again",twice.len())) }
    let mr = measured(r);
    if let Err(d) = same(r,&mr,&again,&twice[0],fit,tol,&mut Found::default()) { return Outcome::SecondTrip(d) }
    match found.degenerate { Some((l,b)) => Outcome::Degenerate(l,b),None => Outcome::Same }
}

/// **The writer's paths a solid takes** (`step::solid` and what it calls, read by what they branch
/// on): each face's surface by the form written (`step::Out::surface`, `face_net`), each edge's
/// curve (`step::solid`'s match), each pcurve by its face's surface entity, its form written and
/// the kernel's own pcurve it was written from (`step::pcurve`'s match), and the topology the
/// writer treats apart (a pole's degenerate edge, a closed edge, a seam, a face of several loops,
/// a face of none, several shells).
pub fn paths(b: &Brep,text: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    fn surface(s: &Surface) -> String {
        match s {
            Surface::Cone(_,_,a) => format!("cone {}",if *a < 0. { "opening against its axis" } else { "opening along its axis" }),
            Surface::BSpline(_,n) => format!("net{}",if n.is_rational() { ", rational" } else { "" }),
            Surface::Blend(_,bl) => format!("blend along a {}",match bl.carry { gcs_core::brep::geom::Carry::Line {..} => "line",_ => "screw" }),
            Surface::Extrusion(_,c) | Surface::Revolution(_,c) => format!("{} of a {}{}",s.kind(),c.kind(),
                if c.exact_span().is_some() { " (fitted)" } else if matches!(&**c,Curve::BSpline(b) if b.weights.is_some()) { ", rational" } else { "" }),
            Surface::Affine(m) => format!("affine image of a {}",surface(&m.of)),
            _ => s.kind().to_string(),
        }
    }
    for (fi,f) in b.faces.iter().enumerate() {
        out.insert(format!("surface: {}",surface(&f.surface)));
        if f.reversed { out.insert("face reversed".into()); }
        if f.loops.len() > 1 { out.insert("face of several loops".into()); }
        if f.loops.iter().all(|l| l.iter().all(|c| matches!(b.edges[c.edge as usize].curve,EdgeCurve::Degenerate))) {
            out.insert("face no written edge bounds".into());
        }
        if let Some(Err(_)) = face_net(b,fi) { out.insert("a net the writer refuses".into()); }
        for c in f.loops.iter().flatten() {
            let e = &b.edges[c.edge as usize];
            if matches!(e.curve,EdgeCurve::Degenerate) { continue }
            let p = match &c.pcurve { Pcurve::Line {..} => "line".to_string(),Pcurve::Inverse {..} => "inverse".into(),
                Pcurve::Curve(pc) => format!("{}",pc.kind()) };
            out.insert(format!("kernel pcurve: {p} on a {}",f.surface.kind()));
            let n = f.loops.iter().flatten().filter(|d| d.edge == c.edge).count();
            if n == 2 { out.insert(format!("seam on a {}",f.surface.kind())); }
        }
    }
    for e in &b.edges {
        match &e.curve {
            EdgeCurve::Degenerate => { out.insert("edge: degenerate (a pole's)".into()); }
            EdgeCurve::Curve(c) => {
                let rational = matches!(c,Curve::BSpline(s) if s.weights.is_some());
                out.insert(format!("edge: {}{}",c.kind(),if rational { ", rational" } else { "" }));
                if e.closed() { out.insert(format!("edge closed: {}",c.kind())); }
            }
        }
    }
    // the shells: faces joined by their edges
    let mut parent: Vec<usize> = (0..b.faces.len()).collect();
    fn root(p: &mut Vec<usize>,x: usize) -> usize { let mut x = x; while p[x] != x { p[x] = p[p[x]]; x = p[x]; } x }
    for uses in b.uses() { for w in uses.windows(2) { let (a,z) = (root(&mut parent,w[0].0),root(&mut parent,w[1].0)); parent[a] = z; } }
    let shells = (0..b.faces.len()).filter(|&f| root(&mut parent,f) == f).count();
    if shells > 1 { out.insert("several shells".into()); }
    // what the file writes for each pcurve: its form, on what surface entity
    let lines: BTreeMap<u32,&str> = text.lines().filter_map(|l| {
        let id = l.strip_prefix('#')?.split(' ').next()?.parse().ok()?;
        Some((id,l.split_once(" = ")?.1))
    }).collect();
    let kind = |id: u32| -> String {
        let Some(l) = lines.get(&id) else { return "?".into() };
        if l.starts_with("( ") { return if l.contains("RATIONAL_B_SPLINE_CURVE") { "rational spline".into() }
            else if l.contains("RATIONAL_B_SPLINE_SURFACE") { "B_SPLINE_SURFACE rational".into() } else { "complex".into() } }
        let k = l.split('(').next().unwrap_or("?");
        if k == "B_SPLINE_CURVE_WITH_KNOTS" { return format!("spline of degree {}",l.split(',').nth(1).unwrap_or("?")) }
        k.to_string()
    };
    let refs = |l: &str| -> Vec<u32> { l.split('#').skip(1).filter_map(|s| s.split(|c: char| !c.is_ascii_digit()).next()?.parse().ok()).collect() };
    for l in lines.values() {
        if let Some(rest) = l.strip_prefix("PCURVE(") {
            let r = refs(rest);
            let (Some(&s),Some(&rep)) = (r.first(),r.get(1)) else { continue };
            let Some(&curve) = lines.get(&rep).map(|d| refs(d)).as_ref().and_then(|v| v.first()) else { continue };
            out.insert(format!("pcurve written: {} on a {}",kind(curve),kind(s)));
        }
        if let Some(rest) = l.strip_prefix("SURFACE_CURVE(").or_else(|| l.strip_prefix("SEAM_CURVE(")) {
            if let Some(&c) = refs(rest).first() { out.insert(format!("edge curve written: {}",kind(c))); }
        }
        if let Some(rest) = l.strip_prefix("SURFACE_OF_LINEAR_EXTRUSION(").or_else(|| l.strip_prefix("SURFACE_OF_REVOLUTION(")) {
            if let Some(&c) = refs(rest).first() { out.insert(format!("swept curve written: {}",kind(c))); }
        }
    }
    out
}

/// The exact solids of the example library, each as `(example, solid, B-rep)`: the example
/// solved, and each object whose exact B-rep is built and checked (`sound_census`'s walk; a
/// swept body skipped where `swept` is `Some(true)`, only swept ones taken where `Some(false)`). Those not built are counted in `unbuilt`.
pub fn library(only: Option<&str>,swept: Option<bool>,unbuilt: &mut Vec<String>,each: &mut dyn FnMut(&str,&str,Brep,Duration)) {
    for (name,e) in fixtures::examples() {
        if only.is_some_and(|o| !name.contains(o)) { continue }
        if std::env::var("STEP_ROUNDTRIP_FROM").is_ok_and(|f| name.as_str() < f.as_str()) { continue }
        if std::env::var("STEP_ROUNDTRIP_UPTO").is_ok_and(|f| name.as_str() >= f.as_str()) { continue }
        if std::env::var("STEP_ROUNDTRIP_SKIP").is_ok_and(|f| f.split(',').any(|x| name.contains(x))) {
            unbuilt.push(format!("{name}: skipped")); continue
        }
        let Some(e) = e else { continue };
        if !e.ok() { continue }
        eprintln!("EXAMPLE {name}");
        let started = Instant::now();
        let built = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut solved = e.sketch.clone();
            gcs_core::solve::solve(&mut solved,gcs_core::solve::SolveOpts::default());
            let mut out = Vec::new();
            for i in gcs_core::overview::objects(&solved) {
                let label = solved.solid_name(i);
                if swept.is_some_and(|skip| skip == solved.is_swept(i)) {
                    out.push((label,Err(format!("{}, skipped",if solved.is_swept(i) { "swept" } else { "not swept" })))); continue
                }
                let x = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solved.exact_solid(i)));
                out.push((label,match x {
                    Ok(Ok(x)) => match x.brep.check(1e-7) { Ok(()) => Ok(x.brep.clone()),Err(e) => Err(format!("not checked: {e}")) },
                    Ok(Err(e)) => Err(format!("not built: {e}")),
                    Err(p) => Err(format!("panicked building: {}",said(p))),
                }));
            }
            out
        }));
        let took = started.elapsed();
        eprintln!("BUILT {name} in {took:.2?}");
        match built {
            Ok(solids) => for (label,b) in solids {
                match b { Ok(b) => each(&name,&label,b,took),Err(why) => unbuilt.push(format!("{name} `{label}`: {why}")) }
            },
            Err(p) => unbuilt.push(format!("{name}: panicked solving: {}",said(p))),
        }
    }
}

/// The tolerances the exports write at: the test's 1e-7 `Fit::Exact`; `solventc --step` and the
/// app with no tolerance, 1e-4 `Fit::Exact` (`export::step_accuracy(None)`); and at the
/// fabrication tolerance the app always asks and `--tolerance` takes bare (0.01 mm), a tenth of the
/// STL's half (`Tolerance::deflection`), 5e-4 stated and its curves within 2.5e-4.
pub fn tolerances() -> Vec<(&'static str,f64,Fit)> {
    let t = gcs_core::solid::export::Tolerance::new(0.01).unwrap();
    let (u,f) = gcs_core::brep::export::step_accuracy(Some(t));
    let (u0,f0) = gcs_core::brep::export::step_accuracy(None);
    vec![("1e-7 exact",1e-7,Fit::Exact),("export, no tolerance",u0,f0),("export at 0.01 mm",u,f)]
}

/// **Every exact solid of the example library, written and read back** (a tool, minutes): one line
/// a solid — its example and name, faces, outcome, the worst face's difference in its bars and
/// relative, and the time to write, to read and to measure — and a summary by outcome and cause.
/// `STEP_ROUNDTRIP_ONLY` narrows to the examples whose name holds it; `STEP_ROUNDTRIP_TOL` picks
/// a tolerance of `tolerances()` by its index (0 by default); `STEP_ROUNDTRIP_SKIP_SWEPT=1` skips
/// swept bodies; `STEP_ROUNDTRIP_PATHS=1` prints each solid's writer paths.
#[test]
#[ignore = "a tool, minutes: every example's exact solid written and read back"]
fn census() {
    let only = std::env::var("STEP_ROUNDTRIP_ONLY").ok();
    let which: usize = std::env::var("STEP_ROUNDTRIP_TOL").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let skip_swept = std::env::var("STEP_ROUNDTRIP_SKIP_SWEPT").is_ok();
    let only_swept = std::env::var("STEP_ROUNDTRIP_ONLY_SWEPT").is_ok();
    let show_paths = std::env::var("STEP_ROUNDTRIP_PATHS").is_ok();
    let (label,tol,fit) = tolerances()[which];
    eprintln!("census at {label}: tol {tol:e}, {fit:?}");
    let mut by: BTreeMap<String,Vec<String>> = BTreeMap::new();
    let mut causes: BTreeMap<String,usize> = BTreeMap::new();
    let mut laid: BTreeMap<&'static str,usize> = BTreeMap::new();
    let mut slowest: Vec<(Duration,String)> = Vec::new();
    let mut loose = Vec::new();
    let mut covered: BTreeMap<String,Vec<String>> = BTreeMap::new();
    let mut unbuilt = Vec::new();
    let mut n = 0;
    library(only.as_deref(),if only_swept { Some(false) } else if skip_swept { Some(true) } else { None },&mut unbuilt,&mut |example,solid,b,_| {
        n += 1;
        let who = format!("{example} `{solid}`");
        let (outcome,found) = trip(&b,&who,tol,fit);
        let worst = found.worst.map_or("-".into(),|(r,fi,a,f)| format!("{r:.2} bars at face {fi} (area {a:.1e}, flux {f:.1e})"));
        eprintln!("ROW\t{who}\t{}\t{}\t{worst}\t{:.0?}\t{:.0?}\t{:.0?}\t{}",b.faces.len(),outcome.label(),found.write,found.read,found.measure,outcome.detail());
        if show_paths || matches!(outcome,Outcome::Same) {
            if let Ok(text) = write(&b,&who,tol,fit) {
                for p in paths(&b,&text) { covered.entry(p).or_default().push(format!("{example}\t{solid}\t{}",b.faces.len())); }
            }
        }
        for (k,v) in &found.laid { *laid.entry(k).or_default() += v; }
        if !found.loose.is_empty() || !found.loose_vertices.is_empty() {
            let e = found.loose.iter().map(|x| x.1).fold(0.,f64::max);
            let v = found.loose_vertices.iter().map(|x| x.1).fold(0.,f64::max);
            loose.push(format!("{who}: {} edges (worst {e:.2e}) and {} vertices (worst {v:.2e}) looser than the {:.0e} stated",
                found.loose.len(),found.loose_vertices.len(),found.loose.first().or(found.loose_vertices.first()).map_or(0.,|x| x.2)));
        }
        slowest.push((found.read,who.clone()));
        let cause = match &outcome {
            Outcome::Refused {by,refusal,..} => format!("{by:?} {:?}: {}",refusal.why,generic(&refusal.msg)),
            Outcome::Different(d) | Outcome::SecondTrip(d) => generic(d),
            Outcome::NotWritten(m) | Outcome::Panicked(m) | Outcome::Slow(m) => generic(m),
            Outcome::Same | Outcome::Degenerate(..) => String::new(),
        };
        if !cause.is_empty() { *causes.entry(format!("{}: {cause}",outcome.label())).or_default() += 1; }
        by.entry(outcome.label()).or_default().push(who);
    });
    eprintln!("\n{n} solids written and read at {label}; {} not built:",unbuilt.len());
    for u in &unbuilt { eprintln!("  UNBUILT {u}"); }
    for (k,v) in &by { eprintln!("SUMMARY {:5}  {k}",v.len()); }
    for (k,v) in &causes { eprintln!("CAUSE {v:5}  {k}"); }
    for (k,v) in &laid { eprintln!("LAID {v:6}  {k}"); }
    for l in &loose { eprintln!("LOOSE {l}"); }
    slowest.sort();
    for (t,w) in slowest.iter().rev().take(10) { eprintln!("SLOW {t:.2?} {w}"); }
    for (p,who) in &covered { eprintln!("PATH {:4}  {p}  [{}]",who.len(),who[..who.len().min(3)].join(" | ")); }
}

/// A message with its numbers taken out, so refusals of one cause count together.
fn generic(m: &str) -> String {
    let mut out = String::new();
    let mut last = ' ';
    for ch in m.chars() {
        let c = if ch.is_ascii_digit() { '0' } else { ch };
        if !(c == '0' && last == '0') { out.push(c); }
        last = c;
    }
    let out = out.replace("0.0","0").replace("0e-0","0").replace("0e0","0").replace("-0","0");
    out.chars().take(140).collect()
}

/// **A cross-check of the numbers, not a gate**: the STEP files the kernel wrote for brep-bench's
/// cases (`$STEP_CORPUS/brep-bench/cad-kernel/`), each read against its row of the corpus's
/// reference (volume and area within the row's bar), and the solid read written again and read to
/// itself (the second trip). One line a file; a tool.
#[test]
#[ignore = "a tool: the kernel's brep-bench files read against the corpus's reference"]
fn bench_cross_check() {
    let Ok(root) = std::env::var("STEP_CORPUS") else { eprintln!("no STEP_CORPUS"); return };
    let table = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"),"/tests/step_corpus/reference.tsv")).unwrap();
    let mut lines = table.lines().filter(|l| !l.starts_with('#'));
    let head: Vec<&str> = lines.next().unwrap().split('\t').collect();
    let col = |n: &str| head.iter().position(|h| *h == n).unwrap();
    let (path,v,a,bv,ba,from) = (col("path"),col("ref_volume"),col("ref_area"),col("bar_volume"),col("bar_area"),col("ref_from"));
    let mut tally: BTreeMap<String,usize> = BTreeMap::new();
    for l in lines.filter(|l| l.starts_with("brep-bench/cad-kernel/")) {
        let f: Vec<&str> = l.split('\t').collect();
        let x = |i: usize| f[i].parse::<f64>().unwrap_or(f64::NAN);
        let Ok(text) = std::fs::read_to_string(std::path::Path::new(&root).join(f[path])) else { continue };
        let started = Instant::now();
        let got = std::panic::catch_unwind(|| -> Result<String,String> {
            let given = stages(&text).map_err(|(by,r,_)| format!("refused by {by:?}: {:?} {r}",r.why))?;
            let vol: f64 = given.iter().map(|g| volume(&g.brep)).sum();
            let ar: f64 = given.iter().map(|g| g.brep.faces.iter().map(|q| area(&g.brep,q)).sum::<f64>()).sum();
            let (dv,da) = ((vol-x(v)).abs()/x(v).abs(),(ar-x(a)).abs()/x(a).abs());
            if !(dv <= x(bv)) || !(da <= x(ba)) {
                return Err(format!("read wrong: volume {dv:.1e} off, area {da:.1e} off ({} bars {:e} {:e})",f[from],x(bv),x(ba)))
            }
            for g in &given {
                let tol = g.context.uncertainty.unwrap_or(1e-7);
                let again = write(&g.brep,"again",tol,Fit::Exact).map_err(|e| format!("not written again: {e}"))?;
                let twice = stages(&again).map_err(|(by,r,_)| format!("second trip refused by {by:?}: {r}"))?;
                let mr = measured(&g.brep);
                same(&g.brep,&mr,&again,&twice[0],Fit::Exact,tol,&mut Found::default()).map_err(|d| format!("second trip differs: {d}"))?;
            }
            Ok(format!("read (volume {dv:.1e}, area {da:.1e} off)"))
        });
        let said = match got { Ok(Ok(s)) => s,Ok(Err(s)) => s,Err(p) => format!("panicked: {}",self::said(p)) };
        let class = said.split([':','(']).next().unwrap_or("").trim().to_string();
        *tally.entry(class).or_default() += 1;
        eprintln!("BENCH\t{}\t{said}\t{:.0?}",f[path],started.elapsed());
    }
    for (k,n) in &tally { eprintln!("BENCH SUMMARY {n:4}  {k}"); }
}

/// Solids the kernel builds directly, each reaching a path of the writer the library's examples
/// may not: the cone either way and to its apex, a whole sphere and torus (poles and seams of
/// both parameters), a revolution of a rational spline and an extrusion of a plain one, a traced
/// meeting, affine images of a quadric, of a cylinder and of a torus, a blend along a line, a
/// solid with a void, and a body of two lumps.
fn kernel_solids() -> Vec<(&'static str,Brep)> {
    use gcs_core::brep::affine::Affine;
    use gcs_core::brep::boolean::{combined,Op};
    use gcs_core::brep::build::{cone,disc,ellipsoid,prism,revolve,sphere,square,torus,Profile,ProfileEdge};
    use gcs_core::brep::nurbs::BSpline;
    use std::f64::consts::{FRAC_1_SQRT_2,TAU};
    use std::sync::Arc;
    const Z: [f64;3] = [0.,0.,1.];
    let tilted = [0.2,-0.3,1.];
    let block = prism(&square([0.,0.,0.],Z,5.),0.,10.).unwrap();
    let pin = prism(&disc([0.,0.,-1.],Z,3.),0.,12.).unwrap();
    let along_x = prism(&disc([-15.,0.,0.],[1.,0.,0.],5.),0.,30.).unwrap();
    let across = prism(&disc([0.,-15.,1.],[0.,1.,0.],3.),0.,30.).unwrap();
    let (south,north) = ([0.,0.,-1.],[0.,0.,1.]);
    let arc = BSpline::rational(2,vec![0.,0.,0.,0.5,0.5,1.,1.,1.],vec![south,[2.,0.,-1.],[2.,0.,0.],[2.,0.,1.],north],
        vec![1.,FRAC_1_SQRT_2,1.,FRAC_1_SQRT_2,1.]).unwrap();
    let half = Profile {origin:south,normal:[0.,-1.,0.],names:vec![],
        loops:vec![vec![ProfileEdge::Spline(Arc::new(arc)),ProfileEdge::Line {a:north,b:south}]]};
    let bump = BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[2.,0.,0.],[2.,1.5,0.],[0.,1.5,0.],[0.,0.,0.]]).unwrap();
    let lobe = Profile {origin:[0.;3],normal:Z,names:vec![],
        loops:vec![vec![ProfileEdge::Line {a:[0.,0.,0.],b:[2.,0.,0.]},ProfileEdge::Spline(Arc::new(bump))]]};
    let shear = Affine::new([[1.,0.4,0.2],[0.,1.,-0.3],[0.1,0.,1.]],[1.,-2.,0.5]).unwrap();
    let boxed = prism(&square([0.,0.,0.],Z,2.),0.,3.).unwrap();
    let round = gcs_core::brep::fillet::roll(&boxed,[2.,2.,1.5],0.5,1e-9).unwrap();
    let rounded = combined(&boxed,&round.piece,if round.concave { Op::Union } else { Op::Cut },0.).unwrap();
    let hollow = combined(&prism(&square([0.,0.,0.],Z,5.),0.,10.).unwrap(),&sphere([0.,0.,5.],Z,2.).unwrap(),Op::Cut,0.).unwrap();
    let slab = gcs_core::brep::build::block([-5.,-5.,-5.],[5.,5.,0.]).unwrap()
        .moved(&gcs_core::brep::geom::Rigid::turn([0.;3],[1.,0.,0.],0.5));
    let aslant = combined(&cone([0.,0.,-3.],Z,1.,1.,6.).unwrap(),&slab,Op::Common,0.).unwrap();
    let two = block.joined(&prism(&square([20.,0.,0.],Z,2.),0.,3.).unwrap());
    vec![
        ("block drilled",combined(&block,&pin,Op::Cut,0.).unwrap()),
        ("cylinder",cone([0.,0.,0.],tilted,1.2,1.2,3.).unwrap()),
        ("cone with its apex",cone([0.,0.,0.],tilted,1.5,0.,2.).unwrap()),
        ("cone opening against its axis",cone([0.,0.,0.],tilted,0.6,1.5,2.).unwrap()),
        ("sphere",sphere([0.5,0.,0.],tilted,1.3).unwrap()),
        ("whole torus",torus([0.,0.,0.],tilted,3.,1.).unwrap()),
        ("spheroid of revolution",revolve(&half,[0.;3],Z,TAU).unwrap()),
        ("extruded spline",prism(&lobe,0.,1.).unwrap()),
        ("crossing cylinders",combined(&along_x,&across,Op::Union,0.).unwrap()),
        ("ellipsoid",ellipsoid([0.1,0.2,0.3],tilted,[1.,0.,0.],[2.,1.2,0.7]).unwrap()),
        ("sheared cylinder",cone([0.,0.,0.],Z,1.,1.,2.).unwrap().mapped(&shear)),
        ("sheared cone",cone([0.,0.,0.],Z,1.5,0.5,2.).unwrap().mapped(&shear)),
        ("sheared torus",torus([0.,0.,0.],Z,3.,1.).unwrap().mapped(&shear)),
        ("sheared extruded spline",prism(&lobe,0.,1.).unwrap().mapped(&shear)),
        ("rounded box",rounded),
        ("block with a spherical void",hollow),
        ("cylinder cut aslant",aslant),
        ("two blocks",two),
    ]
}

/// The kernel's own solids (`kernel_solids`) at every tolerance the exports write at, one line a
/// trip, with their writer's paths: a tool.
#[test]
#[ignore = "a tool: the kernel's own solids at every export tolerance"]
fn kernel_census() {
    for (name,b) in kernel_solids() {
        for (label,tol,fit) in tolerances() {
            let (o,f) = trip(&b,name,tol,fit);
            let worst = f.worst.map_or("-".into(),|(r,fi,a,x)| format!("{r:.2} bars at face {fi} (area {a:.1e}, flux {x:.1e})"));
            eprintln!("KROW\t{name}\t{label}\t{}\t{worst}\t{:?}\t{:?}\t{:?}\tloose {} {}\t{}",o.label(),f.write,f.read,f.measure,
                f.loose.len(),f.loose_vertices.len(),o.detail());
        }
        if let Ok(text) = write(&b,name,1e-7,Fit::Exact) { for p in paths(&b,&text) { eprintln!("KPATH\t{name}\t{p}"); } }
    }
}

/// Solid `solid` of the example `example`, solved and built exact (`library`'s walk, one solid).
fn example_solid(example: &str,solid: &str) -> Brep {
    let e = fixtures::example(example).unwrap_or_else(|| panic!("{example} does not parse"));
    assert!(e.ok(),"{example} does not elaborate");
    let mut solved = e.sketch.clone();
    gcs_core::solve::solve(&mut solved,gcs_core::solve::SolveOpts::default());
    let i = gcs_core::overview::objects(&solved).into_iter().find(|&i| solved.solid_name(i) == solid)
        .unwrap_or_else(|| panic!("{example} has no solid `{solid}`"));
    let x = solved.exact_solid(i).unwrap_or_else(|e| panic!("{example} `{solid}`: {e}"));
    x.brep.clone()
}

/// **`b` reads back to itself at every tolerance the exports write at** (`tolerances`): the same
/// solid, twice over, and no edge or vertex the reader measures looser than the file states.
fn holds(name: &str,b: &Brep) -> Result<(),String> {
    for (label,tol,fit) in tolerances() {
        let (o,f) = trip(b,name,tol,fit);
        if !matches!(o,Outcome::Same) { return Err(format!("{name} at {label}: {}: {}",o.label(),o.detail())) }
        if let Some(&(i,t,stated)) = f.loose.first().or(f.loose_vertices.first()) {
            return Err(format!("{name} at {label}: {} edges and {} vertices read looser than the file states (#{i}: {t:.3e} against {stated:e})",
                f.loose.len(),f.loose_vertices.len()))
        }
    }
    Ok(())
}
