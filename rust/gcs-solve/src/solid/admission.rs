//! The generating-sweep class (docs/generating-sweeps.md): which continuous sweeps a boundary
//! is constructed for, asked of the solved model before any construction. A tool that is a
//! revolution of lines and arcs, or a prism of them drafted or not (or an intersection of such),
//! turned by rotations about fixed axes at constant ratios, clear of the blank at both ends of its
//! roll, touching each blank point once, never folding and never crossing itself: that is what
//! generates a bevel or a hypoid, and everything else is refused by the condition it fails, with a
//! point that fails it. The conditions are **sampled**, never certified, and the evidence says so: a sweep this
//! admits has shown no failure at the samples it names, which is not a proof that none exists
//! between them. Material evaluation answers membership for every sweep, admitted or not; only
//! boundary construction is restricted.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{SpatialField,SweepContacts,cad};
use crate::{envelope::{Motion,SurfacePoint},model::{EntKind,Sketch,SolidDef}};
use std::{collections::BTreeMap,f64::consts::TAU,fmt};

use crate::space::{sub,cross,norm};

type V = [f64;3];

pub use cad_kernel::sweep::condition::Condition;

/// A sweep outside the class: which row, which sweep, what was seen, and where.
#[derive(Clone,Debug)]
pub struct Refusal {
    pub condition: Condition,
    pub sweep: String,
    pub message: String,
    /// A point in the body's coordinates where the condition fails, when there is one.
    pub witness: Option<V>,
    /// Which class's rows the sweep was asked: by its motion (a screw's) or its tool (prisms').
    pub asked: Asked,
}

/// The class a sweep's rows are read from.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Asked { Generating,ConstantTwist,Planar,Subtraction,Canal }

impl Refusal {
    /// The class the sweep was asked to join, by its motion and its tool.
    pub fn class(&self) -> &'static str {
        match self.asked { Asked::Generating => "generating-sweep class",Asked::ConstantTwist => "constant-twist class",
            Asked::Planar => "planar generating class",Asked::Subtraction => "planar subtraction class",
            Asked::Canal => "canal class" }
    }
}

impl fmt::Display for Refusal {
    fn fmt(&self,f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"`{}` is outside the {} ({}: {}): {}",
            self.sweep,self.class(),self.condition.code(),self.condition.rule(),self.message)?;
        if let Some(p) = self.witness { write!(f," at ({:.4}, {:.4}, {:.4})",p[0],p[1],p[2])?; }
        Ok(())
    }
}

#[derive(Clone,Debug)]
pub enum Error {
    /// Outside the class.
    Refused(Refusal),
    /// The body could not be read at all; nothing is said about the class.
    Unreadable(String),
}

impl fmt::Display for Error {
    fn fmt(&self,f: &mut fmt::Formatter) -> fmt::Result {
        match self { Error::Refused(r) => r.fmt(f), Error::Unreadable(m) => f.write_str(m) }
    }
}

/// Sampling density and the bars (the kernel's, `cad_kernel::sweep::admit`), and how the evidence
/// was obtained.
pub use cad_kernel::sweep::admit::{Basis,Options};
use cad_kernel::sweep::admit::{Evidence,Found,Lone,Reads};

/// One placement of a swept cut: its pose in the body, and whether its checks were skipped as
/// equivalent to an earlier placement's, which they are exactly when the blank is inside at
/// every point the earlier placement's checks read and outside at every other.
#[derive(Clone,Debug)]
pub struct Placement { pub pose: Motion, pub equivalent_to: Option<usize> }

/// How a sweep's later placements were found equivalent to its first.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Equivalence {
    /// From the solid graph: every operand of the blank is a full revolution about one line (a
    /// point of it and its unit direction), and each later placement is the first turned about
    /// that line, so the blank is the same under the turn and reads alike at every point.
    Revolved { origin: V,axis: V },
    /// Sampled: each later placement's blank read at every point the first placement's checks
    /// read, and found alike there (or checked itself where it was not).
    Sampled,
}

/// How near the lines and the turns must agree (relative to the blank's size) for the structural
/// equivalence: the solver's own noise, not a modelling tolerance.
const COINCIDENT: f64 = 1e-12;

#[derive(Clone,Debug)]
pub struct SweepEvidence {
    pub sweep: usize,
    pub name: String,
    pub placements: Vec<Placement>,
    /// Tool points sampled, and their contacts inside the blank, at the first placement.
    pub samples: usize,
    pub contacts: usize,
    /// Most neighbouring contacts are this close (the 95th percentile of their gaps).
    pub spacing: f64,
    pub least_area_factor: f64,
    /// Samples whose two contact times nearly coincide: the edge of the envelope, where the
    /// contact equation has a double root and is not solved there.
    pub near_double_roots: usize,
    /// Pairs of contacts from distant tool points lying on one another with the same normal:
    /// sheets touching tangentially, which is where a crossing begins.
    pub near_tangent_pairs: usize,
    pub basis: Basis,
    /// How the later placements' checks were stood for by the first's.
    pub equivalence: Equivalence,
    /// Which class admitted it, and what that class found.
    pub class: Class,
}

/// The class a sweep is admitted to: a relative rotation's (the generating class, rows M2–E4), a
/// screw's (constant twist, rows S1–S4), with the stretch of its characteristic that reaches the
/// blank at the first placement, or a planar motion's of a pocketed prism (rows P1–P5), with the
/// pocket's inner envelope.
#[derive(Clone,Debug)]
pub enum Class { Generating,ConstantTwist(Box<super::constant_twist::Characteristic>),Planar(Box<super::planar_class::Found>),
    Subtraction(Box<super::planar_subtraction::Found>),Capped(Box<Capped>) }

/// A generating sweep of a prism whose cap reaches the blank (#684 F, row T3), sectioned across:
/// the kernel's (`cad_kernel::sweep::sheet::Capped`).
pub use cad_kernel::sweep::sheet::Capped;

/// A body admitted to the class, and the only way to have one: `admit_body` makes it, so a
/// construction that takes one cannot be reached without the gate.
#[derive(Clone,Debug)]
pub struct Admission { body: usize,sweeps: Vec<SweepEvidence> }

impl Admission {
    /// The body admitted.
    pub fn body(&self) -> usize { self.body }
    /// Each distinct sweep the body cuts, with what its checks saw.
    pub fn sweeps(&self) -> &[SweepEvidence] { &self.sweeps }
}

/// The body's static remainder as a field, and its swept cuts with their poses. Swept material
/// is read only where it is cut from the body itself, directly or through placements. A host
/// constructing the swept boundary reads the blank here too: exact, and microseconds a point.
pub fn static_remainder(sk: &Sketch,root: usize,axis_tolerance: f64) -> Result<(SpatialField,Vec<cad::SweptCut>),String> {
    super::validate(sk,root)?;
    let solid = &sk.solids[root];
    let SolidDef::Body {stock,on,through,bound} = &solid.def else {
        return Err(format!("`{}`: admission reads a body with swept cuts",solid.name));
    };
    if let Some(&o) = std::iter::once(stock).chain(on).chain(bound).find(|&&o| cad::contains_sweep(sk,o as usize)) {
        return Err(format!("`{}`: only a `cut` may hold swept material for now (`{}`)",solid.name,sk.solids[o as usize].name));
    }
    let read = |i: u32| SpatialField::read(sk,i as usize,axis_tolerance);
    let mut field = read(*stock)?;
    for &o in on { field = field.union(read(o)?).map_err(|e| format!("{e:?}"))?; }
    for &b in bound { field = field.intersection(read(b)?).map_err(|e| format!("{e:?}"))?; }
    let mut sweeps = Vec::new();
    for &c in through {
        match cad::swept_cut(sk,c as usize)? {
            Some(sweep) => sweeps.push(sweep),
            None => field = field.difference(read(c)?).map_err(|e| format!("{e:?}"))?,
        }
    }
    Ok((field,sweeps))
}

/// What a full revolution is the same under: every turn about its line (a point of it and a
/// direction), or — a ball, its profile arcs of one circle about a point of the line and segments
/// of the line itself — every turn about any line through that centre.
#[derive(Clone,Copy,Debug)]
enum Turns { Line(V,V),Ball(V) }

/// What each operand of a static solid is the same under (`Turns`), or none where an operand is
/// not a full revolution. `tolerance` (a length) is how near a point must be to count as on a line.
fn revolution_turns(sk: &Sketch,id: usize,tolerance: f64) -> Option<Vec<Turns>> {
    match &sk.solids[id].def {
        SolidDef::Revolve {face,axis,sweep,..} => {
            if (sweep.value.abs()-TAU).abs() > 1e-9 { return None; }
            // the axis's two points lifted through the profile's frame, as the recipe lifts them
            let p = super::face_poly(sk,*face as usize,super::REPORT_UNIT)?;
            let lift = |i: usize| { let q = sk.point_xy(i); p.basis.lift(q.0,q.1) };
            let (a2,b2) = sk.revolve_axis(*face,*axis)?;
            let (a,b) = (p.basis.lift(a2.0,a2.1),p.basis.lift(b2.0,b2.1));
            let d = sub(b,a);
            if !(norm(d) > 0.) { return None; }
            let on_axis = |x: V| norm(cross(sub(x,a),d))/norm(d) <= tolerance;
            // a ball: every edge an arc or circle of one circle centred on the axis, or a segment of it
            let mut ball: Option<(V,f64)> = None;
            let mut round = true;
            for (edges,_) in sk.faces[*face as usize].boundaries() { for e in edges {
                let circle = match e.kind {
                    EntKind::Arc => { let c = &sk.arcs[e.i()]; Some((c.center,c.radius)) }
                    EntKind::Circle => { let c = &sk.circles[e.i()]; Some((c.center,c.radius)) }
                    EntKind::Line => { let l = &sk.lines[e.i()]; round &= on_axis(lift(l.p1 as usize)) && on_axis(lift(l.p2 as usize)); None }
                    _ => { round = false; None }
                };
                if let Some((centre,radius)) = circle {
                    let (c,r) = (lift(centre as usize),sk.params[radius as usize].value);
                    round &= on_axis(c) && ball.is_none_or(|(k,q)| norm(sub(k,c)) <= tolerance && (q-r).abs() <= tolerance);
                    ball.get_or_insert((c,r));
                }
            } }
            Some(vec![match ball { Some((c,_)) if round => Turns::Ball(c),_ => Turns::Line(a,d) }])
        }
        // a region: a revolution of its meridian about its axis — or, balls about one centre
        // alone, the same under every turn about it
        SolidDef::Region {terms,..} => {
            let m = super::region::meridian(sk,id).ok()?;
            let one = |t: &crate::model::RegionTerm| match t.shape {
                crate::model::RegionShape::Ball {center} => Some(sk.world_point(center as usize)),
                _ => None,
            };
            let centres: Option<Vec<V>> = terms.iter().map(one).collect();
            let alike = centres.as_ref().is_some_and(|c| c.iter().all(|&x| norm(sub(x,c[0])) <= tolerance));
            Some(vec![if alike { Turns::Ball(m.origin) } else { Turns::Line(m.origin,m.axis) }])
        }
        SolidDef::Placed {source,motion,at} => {
            let pose = crate::motion::Family::read(sk,*motion as usize).ok()?.at(at.value).ok()?;
            Some(revolution_turns(sk,*source as usize,tolerance)?.into_iter().map(|t| match t {
                Turns::Line(o,d) => Turns::Line(pose.point(o),pose.vector(d)),
                Turns::Ball(c) => Turns::Ball(pose.point(c)),
            }).collect())
        }
        SolidDef::Body {..} => {
            let mut turns = Vec::new();
            for o in sk.solids[id].operands() { turns.extend(revolution_turns(sk,o as usize,tolerance)?); }
            Some(turns)
        }
        SolidDef::Prism {..} | SolidDef::Through {..} | SolidDef::Swept {..}
            | SolidDef::Family {..} => None,
    }
}

/// The blank of `root` (its stock, what is put on it, what bounds it and its static cuts) the same
/// under every turn about one line — each operand a full revolution about it, or a ball centred on
/// it — and every placement of every sweep a turn of the sweep's first about that line: then each
/// later placement's checks read the blank exactly as the first's do (`Equivalence::Revolved`).
/// `size` is the blank's, for the tolerance.
fn structurally_alike(sk: &Sketch,root: usize,poses: &BTreeMap<usize,Vec<Motion>>,size: f64) -> Option<(V,V)> {
    let SolidDef::Body {stock,on,through,bound} = &sk.solids[root].def else { return None };
    let tolerance = COINCIDENT*size.max(1.);
    let mut blank = vec![*stock];
    blank.extend(on.iter().chain(bound));
    for &c in through { if cad::swept_cut(sk,c as usize).ok()?.is_none() { blank.push(c); } }
    let mut turns = Vec::new();
    for &o in &blank { turns.extend(revolution_turns(sk,o as usize,tolerance)?); }
    // the line: the first operand's that has one, every other line and every ball's centre on it
    let (origin,axis) = turns.iter().find_map(|t| match *t { Turns::Line(o,d) => Some((o,d.map(|x| x/norm(d)))),_ => None })?;
    let on_line = |x: V| norm(cross(sub(x,origin),axis)) <= tolerance;
    for t in &turns {
        let alike = match *t {
            Turns::Line(o,d) => norm(cross(axis,d))/norm(d) <= COINCIDENT && on_line(o),
            Turns::Ball(c) => on_line(c),
        };
        if !alike { return None; }
    }
    let far: V = std::array::from_fn(|k| origin[k]+size.max(1.)*axis[k]);
    let mut worst = 0f64;
    for placements in poses.values() {
        let Some((first,rest)) = placements.split_first() else { continue };
        let back = first.inverse();
        for pose in rest {
            let turn = back.then(*pose);
            for x in [origin,far] { worst = worst.max(norm(sub(turn.point(x),x))); }
            if worst > tolerance { return None; }
        }
    }
    if std::env::var_os("SOLVENT_ADMISSION_TIMES").is_some() {
        let off = turns.iter().map(|t| match *t { Turns::Line(o,d) => (norm(cross(axis,d))/norm(d)*size).max(norm(cross(sub(o,origin),axis))),
            Turns::Ball(c) => norm(cross(sub(c,origin),axis)) }).fold(0.,f64::max);
        eprintln!("admission: the blank is {} revolutions about one line (off it by {off:.1e} at most) and every placement a turn \
            about it (off by {worst:.1e}), against {tolerance:.1e}",turns.len());
    }
    Some((origin,axis))
}

/// T1, as a question about the tool's solid graph.
fn tool(sk: &Sketch,id: usize) -> Result<(),String> {
    let solid = &sk.solids[id];
    match &solid.def {
        SolidDef::Revolve {sweep,..} => if (sweep.value.abs()-TAU).abs() > 1e-9 {
            Err(format!("`{}` is a partial revolution",solid.name)) } else { Ok(()) },
        SolidDef::Placed {source,..} => tool(sk,*source as usize),
        // a region is a full revolution of its meridian
        SolidDef::Region {..} => super::region::meridian(sk,id).map(|_| ()),
        SolidDef::Body {stock,on,through,bound} => {
            // a union that is the whole tool is its members, each swept alone (`distributed`): one
            // here is within an intersection, which is not distributed
            if !on.is_empty() { return Err(format!("`{}` adds solids together within an intersection; a union is in the class \
                only as the whole tool, each member swept alone",solid.name)); }
            if !through.is_empty() { return Err(format!("`{}` cuts solids; only intersections are in the class",solid.name)); }
            std::iter::once(stock).chain(bound).try_for_each(|&o| tool(sk,o as usize))
        }
        // a drafted prism's sides lean (#684 G): each a plane or a cone, ruled between its sections
        SolidDef::Prism {draft:Some(_),..} => super::ExtrudedSurface::drafted(sk,id).map(|_| ())
            .map_err(|e| format!("`{}` is a drafted prism whose sides the class does not read: {e}",solid.name)),
        // a prism's sides are extrusions of lines and arcs, its contact set fixed by the motion
        SolidDef::Prism {..} => Ok(()),
        SolidDef::Family {..} => Err(format!("`{}` is a face family, not a revolution",solid.name)),
        SolidDef::Through {..} => Err(format!("`{}` is a through prism, not a revolution",solid.name)),
        SolidDef::Swept {..} => Err(format!("`{}` is itself a sweep",solid.name)),
    }
}

/// Admit or refuse every swept cut of the body `root` (the body it places, where `root` is a
/// placement). The sweeps are checked side by side and a placement's reads against the first
/// placement's on every core; what is admitted or refused, and with which evidence, is what
/// checking them one after another in order says.
pub fn admit_body(sk: &Sketch,root: usize,options: &Options) -> Result<Admission,Error> {
    // a body placed by a rigid motion is the body moved: the body it places is admitted (#619)
    let root = cad::placed_swept(sk,root).map_err(Error::Unreadable)?.map_or(root,|(inner,_)| inner);
    // a sweep under a driven joint is its joint's over the law's reach (#684 L), and a cut of a
    // union's sweep a cut of each member's (#684 C): the body is admitted as the copy of the drawing
    // that writes it so, each sweep by its own class (where every one is a ball's tube, the body is
    // static material there, and nothing is asked)
    if let Some(lowered) = super::lowered(sk,root) {
        if !cad::contains_sweep(&lowered,root) { return Ok(Admission {body:root,sweeps:Vec::new()}) }
        return admit_body(&lowered,root,options)
    }
    let (field,cuts) = static_remainder(sk,root,options.axis_tolerance).map_err(Error::Unreadable)?;
    let mut by_sweep: BTreeMap<usize,Vec<Motion>> = BTreeMap::new();
    let mut order = Vec::new();
    for cut in cuts {
        if !by_sweep.contains_key(&cut.swept) { order.push(cut.swept); }
        by_sweep.entry(cut.swept).or_default().push(cut.pose);
    }
    let names: BTreeMap<usize,String> = order.iter().map(|&s| (s,sk.solids[s].name.clone())).collect();
    // the first ball's sweep its tube's rows refuse (#684 D), along a path no closed form carries
    let canal = order.iter().find_map(|&s| super::canal::refusal(sk,s).map(|r| (s,r)));
    let asked: BTreeMap<usize,Asked> = order.iter().map(|&s| (s,if canal.as_ref().is_some_and(|(c,_)| *c == s) { Asked::Canal }
        else if super::planar_class::asks(sk,s) { Asked::Planar }
        else { match &sk.solids[s].def {
            SolidDef::Swept {motion,..} if crate::motion::Family::read(sk,*motion as usize).is_ok_and(|f| f.twist().is_some()) => Asked::ConstantTwist,
            _ => Asked::Generating,
        } })).collect();
    // a sweep of a member of a union tool is refused as the union's, saying the member (#684 C)
    let members: BTreeMap<usize,String> = order.iter().filter(|&&s| sk.distributed.contains_key(&(s as u32)))
        .filter_map(|&s| match &sk.solids[s].def { SolidDef::Swept {source,..} => Some((s,sk.solids[*source as usize].name.clone())),
            _ => None }).collect();
    let said = |swept: usize,message: String| match members.get(&swept) {
        Some(member) => format!("its tool's member `{member}`: {message}"),
        None => message,
    };
    let refuse = |swept: usize,condition,message: String,witness|
        Error::Refused(Refusal {condition,sweep:names[&swept].clone(),message:said(swept,message),witness,asked:asked[&swept]});
    if let Some((swept,r)) = canal {
        return Err(refuse(swept,r.condition,r.message,r.witness.map(|p| by_sweep[&swept][0].point(p))))
    }
    let field = &field;
    let bounds = field.support_bounds().ok().flatten();
    // a pocketed prism under a planar motion: rows P1–P5, every placement checked, the blank boxed
    // by its own boundary (a field's support is a construction's box, not the material's)
    let mut planar: Vec<SweepEvidence> = Vec::new();
    // built once, where a planar class first asks for it
    let boxed = std::cell::OnceCell::new();
    let blank_box = || -> Option<(V,V)> { *boxed.get_or_init(|| blank_box(sk,root)) };
    let planar_box = order.iter().any(|s| asked[s] == Asked::Planar).then(blank_box).flatten();
    for &swept in order.iter().filter(|s| asked[s] == Asked::Planar) {
        let refused = |(c,m,w): super::planar_class::Failure,pose: Motion| refuse(swept,c,m,w.map(|p| pose.point(p)));
        let class = super::planar_class::Planar::read(sk,swept,options).map_err(|f| refused(f,by_sweep[&swept][0]))?;
        let mut first = None;
        for &pose in &by_sweep[&swept] {
            let inside = move |p: V| field.value(pose.point(p)) < -options.margin;
            let checked = class.check(sk,pose,&inside,planar_box,options).map_err(|f| refused(f,pose))?;
            first.get_or_insert(checked);
        }
        let mut found = class.evidence(first.expect("a swept cut has a placement"));
        found.sweep = swept; found.name = names[&swept].clone();
        found.placements = by_sweep[&swept].iter().map(|&pose| Placement {pose,equivalent_to:None}).collect();
        planar.push(found);
    }
    order.retain(|s| asked[s] != Asked::Planar);
    // a placement's check by the class its motion is in
    let check = |contacts: &SweepContacts,lone: Option<&Lone>,pose: Motion,inside: &(dyn Fn(V) -> bool+Sync)| {
        let extent = contacts.motion().twist()
            .and_then(|screw| bounds.map(|b| screw.extent(b.map(|x| x.bounds()[0]),b.map(|x| x.bounds()[1]),pose.inverse())));
        cad_kernel::sweep::admit::admit(contacts,inside,extent,options,lone).map(|(found,reads)| (evidence(found),reads))
    };
    // a tool that is one prism, whose cap the class may section across (#684 F)
    let lones: Vec<Option<Lone>> = order.iter().map(|&swept| match &sk.solids[swept].def {
        SolidDef::Swept {source,..} => lone(sk,*source as usize),
        _ => None,
    }).collect();
    let inside_at = |pose: Motion| move |p: V| field.below(pose.point(p),-options.margin);
    // Each sweep's tool read, then its first placement checked.
    let tools: Vec<Result<SweepContacts,Error>> = order.iter().map(|&swept| {
        let SolidDef::Swept {source,..} = &sk.solids[swept].def else { unreachable!("a swept cut is a sweep") };
        tool(sk,*source as usize).map_err(|m| refuse(swept,Condition::Tool,m,None))?;
        let contacts = SweepContacts::read(sk,swept,options.axis_tolerance).map_err(|m| refuse(swept,Condition::Tool,m,None))?;
        if contacts.motion().twist().is_none() {
            let probe = SurfacePoint {position:[1.,2.,3.],du:[1.,0.,0.],dv:[0.,1.,0.]};
            contacts.motion().normal_velocity(probe).map_err(|m| refuse(swept,Condition::Motion,m,None))?;
        }
        Ok(contacts)
    }).collect();
    let clock = crate::clock::Instant::now();
    let checks = crate::par::indices(order.len(),|k| -> Option<Result<(SweepEvidence,Reads),Error>> {
        let contacts = tools[k].as_ref().ok()?;
        let (swept,pose) = (order[k],by_sweep[&order[k]][0]);
        Some(check(contacts,lones[k].as_ref(),pose,&inside_at(pose)).map_err(|(c,m,w)| refuse(swept,c,m,w.map(|p| pose.point(p)))))
    });
    let first: Vec<Result<(SweepContacts,SweepEvidence,Reads),Error>> = tools.into_iter().zip(checks).map(|(tool,check)| {
        let contacts = tool?;
        let (found,reads) = check.expect("a read tool is checked")?;
        Ok((contacts,found,reads))
    }).collect();
    if std::env::var_os("SOLVENT_ADMISSION_TIMES").is_some() { eprintln!("admission: checked in {:?}",clock.elapsed()); }
    let clock = crate::clock::Instant::now();
    // The blank a solid of revolution about the line every placement turns about: the later
    // placements read it as the first does, proved by the solid graph. Otherwise, whether each
    // later placement reads the blank as the first did, every point of it.
    let size = bounds.map_or(1.,|b| crate::space::box_centre_diagonal(&b).1);
    let revolved = structurally_alike(sk,root,&by_sweep,size);
    let later: Vec<(usize,usize)> = if revolved.is_some() { Vec::new() } else {
        first.iter().enumerate().filter(|(_,r)| r.is_ok())
            .flat_map(|(k,_)| (1..by_sweep[&order[k]].len()).map(move |p| (k,p))).collect()
    };
    let alike = crate::par::map(&later,|&(k,p)| {
        let inside = inside_at(by_sweep[&order[k]][p]);
        let Ok((_,_,reads)) = &first[k] else { unreachable!("only a checked sweep's placements are compared") };
        reads.points.iter().zip(&reads.inside).all(|(q,&i)| inside(*q) == i)
    });
    if std::env::var_os("SOLVENT_ADMISSION_TIMES").is_some() { eprintln!("admission: compared in {:?}",clock.elapsed()); }
    let mut admission = Admission {body:root,sweeps:planar};
    // each sweep's later placements, read alike or not
    let mut by: Vec<Vec<bool>> = vec![Vec::new();order.len()];
    for (&(k,_),a) in later.iter().zip(alike) { by[k].push(a) }
    let alike = by;
    // a prism sweep under a motion keeping its plane that the generating rows refuse, at any row and
    // any placement, is asked the planar subtraction class's rows instead (Q1–Q4), every placement
    // at once: what those rows guard against — a union within an intersection, a draft running both
    // ways from its plane, a concave corner, a roll never clear, a second contact — its swept region
    // does not need
    let subtract = |swept: usize| -> Result<SweepEvidence,Error> {
        let inside = |p: V| field.value(p) < -options.margin;
        super::planar_subtraction::admit(sk,swept,&by_sweep[&swept],&inside,blank_box()).map_err(|(condition,message,witness)|
            Error::Refused(Refusal {condition,sweep:names[&swept].clone(),message:said(swept,message),witness,
                asked:Asked::Subtraction}))
    };
    for (k,(result,&swept)) in first.into_iter().zip(&order).enumerate() {
        let generating = (|| -> Result<SweepEvidence,Error> {
            let (contacts,found,reads) = result?;
            let poses = &by_sweep[&swept];
            let mut placements = vec![Placement {pose:poses[0],equivalent_to:None}];
            let mut checked: Vec<(usize,Reads)> = vec![(0,reads)];
            let mut found = found;
            if let Some((origin,axis)) = revolved {
                placements.extend(poses.iter().skip(1).map(|pose| Placement {pose:*pose,equivalent_to:Some(0)}));
                found.sweep = swept; found.name = names[&swept].clone(); found.placements = placements;
                found.equivalence = Equivalence::Revolved {origin,axis};
                return Ok(found)
            }
            for (j,pose) in poses.iter().enumerate().skip(1) {
                if alike[k][j-1] { placements.push(Placement {pose:*pose,equivalent_to:Some(0)}); continue; }
                let inside = inside_at(*pose);
                let same = checked.iter().skip(1).find(|(_,reads)| reads.points.iter().zip(&reads.inside).all(|(p,&i)| inside(*p) == i));
                if let Some((i,_)) = same { placements.push(Placement {pose:*pose,equivalent_to:Some(*i)}); continue; }
                let (_,reads) = check(&contacts,lones[k].as_ref(),*pose,&inside).map_err(|(c,m,w)| refuse(swept,c,m,w.map(|p| pose.point(p))))?;
                checked.push((j,reads));
                placements.push(Placement {pose:*pose,equivalent_to:None});
            }
            found.sweep = swept; found.name = names[&swept].clone(); found.placements = placements;
            Ok(found)
        })();
        admission.sweeps.push(match generating {
            Err(Error::Refused(_)) if super::planar_subtraction::asks(sk,swept) => subtract(swept)?,
            other => other?,
        });
    }
    Ok(admission)
}

/// The box body `root`'s static blank lies in (model units, least and greatest corner), read off
/// its own boundary built by this kernel: a field's support is a construction's box, not the
/// material's. What the planar classes box the blank by.
pub fn blank_box(sk: &Sketch,root: usize) -> Option<(V,V)> {
    let mm = cad::millimetres(sk);
    let blank = crate::brep::recipe::build(&cad::recipe_static(sk,root).ok()?.recipe).ok()?;
    let (lo,hi) = blank.bounds();
    Some((lo.map(|x| x/mm),hi.map(|x| x/mm)))
}

/// A tool that is one prism standing alone, its profile one loop, as the class sections it across
/// where its cap reaches the blank (T3, #684 F): none where tool `source` is not one.
fn lone(sk: &Sketch,source: usize) -> Option<Lone> {
    let SolidDef::Prism {face,draft:None,..} = &sk.solids[source].def else { return None };
    if sk.faces.get(*face as usize)?.boundaries().count() != 1 { return None }
    Lone::new(crate::brep::sweep::cutter::Cutter::read(&cad::recipe(sk,source).ok()?).ok()?,cad::millimetres(sk))
}

/// What the kernel's rows saw of one placement, as the evidence of sweep 0 of no name, placed
/// nowhere yet and read by sampling: `admit_body` says which sweep, and where.
fn evidence(e: Evidence) -> SweepEvidence {
    let class = match e.class { Found::Generating => Class::Generating,Found::ConstantTwist(found) => Class::ConstantTwist(found),
        Found::Capped(capped) => Class::Capped(capped) };
    SweepEvidence {sweep:0,name:String::new(),placements:Vec::new(),samples:e.samples,contacts:e.contacts,spacing:e.spacing,
        least_area_factor:e.least_area_factor,near_double_roots:e.near_double_roots,near_tangent_pairs:e.near_tangent_pairs,
        basis:e.basis,equivalence:Equivalence::Sampled,class}
}
