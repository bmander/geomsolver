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
use super::{SpatialField,cad};
use crate::brep::swept::admission as rows;
use crate::brep::swept::dispatch;
use crate::{envelope::Motion,model::{EntKind,Sketch,SolidDef}};
use std::{collections::BTreeMap,f64::consts::TAU,fmt};

use crate::space::{sub,cross,norm};

type V = [f64;3];

/// The rows of the classes, in the order they are asked, are the kernel's (`dispatch::Row`, #721
/// F1c): their codes too.
pub use crate::brep::swept::dispatch::Row as Condition;

/// What row `c` asks, in the words the scope document uses.
pub fn rule(c: Condition) -> &'static str {
    match c {
        Condition::Tool => "the tool is read from its boundary, every face a face of revolution of lines and arcs, a ruled \
            side of lines or arcs (drafted or not) or a plane, its planes that are no side clear of the blank unless they are \
            the caps of one undrafted prism standing alone, or a union of tools each in the class",
        Condition::Corner => "every profile corner the sweep carries into the blank is convex or tangent",
        Condition::Motion => "the motion is rotations about fixed axes at constant ratios, or one screw, turn or slide (a joint driven by a law alone read as its joint over the values the law reaches)",
        Condition::Stationary => "the contact condition depends on the motion",
        Condition::Reach => "the sweep's contacts reach the blank",
        Condition::Clearance => "the tool is clear of the blank at both ends of the roll",
        Condition::Single => "each tool point touches the blank at most once in the roll",
        Condition::Fold => "the generated surface does not fold",
        Condition::Crossing => "the generated surface does not cross itself",
        Condition::Sections => "a prism whose cap reaches the blank is sectioned through its extrusion in a band clear of its \
            profile's corners and its ends",
        Condition::Regular => "the screw's characteristic is a regular curve through the blank",
        Condition::Once => "the characteristic reaches the blank in one stretch, one point a ring",
        Condition::Edgewise => "the characteristic never runs along the screw's path",
        Condition::Lead => "the sweep does not cross itself within a lead, or a turn",
        Condition::Plane => "the motion turns a plane in itself about axes square to it",
        Condition::Prisms => "the tool is a stock cut by one pocket, prisms square to that plane through the blank, the \
            pocket's profile one closed curve",
        Condition::Period => "the roll is a whole period of the motion",
        Condition::Envelope => "the pocket's inner envelope is one regular loop that neither folds nor crosses itself",
        Condition::Wall => "the stock's outer wall never reaches the blank",
        Condition::Members => "the tool stands along the normal of a plane the motion keeps: every face of its boundary a plane \
            square to it, or a plane, a cylinder or a cone standing along it",
        Condition::Slabs => "the blank is split at every cap of the tool within it, and where a side leans across a slab its \
            region keeps one shape through it",
        Condition::Closes => "the roll is a whole period of the motion, or the tool is clear of the blank at both its ends",
        Condition::Kept => "the swept region's boundary is regular: its pieces meet at corners or leave the blank's shadow, with no \
            fold and no crossing",
        Condition::Curvature => "the ball's centre's path never stops and turns nowhere tighter than its radius",
        Condition::Return => "the ball's centre's path comes back within its diameter of itself nowhere",
        Condition::Faces => "every face of the tool is a plane and every edge a line",
        Condition::Constant => "the motion is one slide, turn or screw, and under a screw every face and edge of the tool runs \
            along its axis or square to it",
        Condition::Settled => "every cell the sweep's candidate faces split the blank into is read in or out of the sweep at a \
            point clear of its boundary",
    }
}

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

/// The class a sweep's rows are read from is the kernel's (`dispatch::Asked`).
pub use crate::brep::swept::dispatch::Asked;

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
            self.sweep,self.class(),self.condition.code(),rule(self.condition),self.message)?;
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

/// Sampling density. Each revolved tool face is sampled `rows` along its profile by `columns`
/// around the band of its revolution that reaches the blank, the band found first by a
/// `coarse_rows` by `coarse_columns` pass over the whole revolution.
#[derive(Clone,Copy,Debug)]
pub struct Options {
    pub rows: usize,
    pub columns: usize,
    pub coarse_rows: usize,
    pub coarse_columns: usize,
    pub axis_tolerance: f64,
    /// How far inside the blank's field a point must be to count as in the blank.
    pub margin: f64,
    pub root_tolerance: f64,
    /// A generated area factor below this, relative to the tool's own, is a fold.
    pub least_factor: f64,
}

impl Options {
    /// How finely the kernel's rows sample the tool.
    fn sampling(&self) -> rows::Sampling {
        rows::Sampling {rows:self.rows,columns:self.columns,coarse_rows:self.coarse_rows,coarse_columns:self.coarse_columns,
            root_tolerance:self.root_tolerance,least_factor:self.least_factor}
    }
}

impl Default for Options {
    fn default() -> Self {
        Options {rows:100,columns:400,coarse_rows:24,coarse_columns:1440,axis_tolerance:cad::AXIS_TOLERANCE,
            margin:1e-6,root_tolerance:1e-9,least_factor:1e-3}
    }
}

/// How the evidence was obtained. Only sampling exists; a certified basis would be a second
/// variant, and a caller matching on this one learns of it when it arrives.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Basis { Sampled { rows: usize,columns: usize } }

/// One placement of a swept cut: its pose in the body, and whether its checks were skipped as
/// equivalent to an earlier placement's, which they are exactly when the blank is inside at
/// every point the earlier placement's checks read and outside at every other.
#[derive(Clone,Debug)]
pub struct Placement { pub pose: Motion, pub equivalent_to: Option<usize> }

/// How a sweep's later placements were found equivalent to its first is the kernel's
/// (`dispatch::Equivalence`): proved from the solid graph, the blank every turn about one line's
/// alike, or sampled at every point the first placement's checks read.
pub use crate::brep::swept::dispatch::Equivalence;

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
/// pocket's inner envelope; or, refused by those, a polyhedral tool's under one constant twist, built
/// closed by the kernel's one arrangement (rows C1–C3, `closed_class`).
#[derive(Clone,Debug)]
pub enum Class { Generating,ConstantTwist(Box<super::constant_twist::Characteristic>),Planar(Box<super::planar_class::Found>),
    Subtraction(Box<super::planar_subtraction::Found>),Capped(Box<Capped>),Closed(Box<super::closed_class::Closed>) }

/// A generating sweep of a prism whose cap reaches the blank, sectioned across (#684 F, row T3), is
/// the kernel's finding (`brep::swept::admission::Capped`).
pub use crate::brep::swept::admission::Capped;

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

/// The blank a body's sweeps cut, as the kernel's dispatch reads it (`dispatch::Blank`): its
/// static remainder's field, a point within `margin` of whose boundary is not in it, the box its
/// support stands in, and the box its own boundary stands in, built from its recipe the first time
/// a planar class asks it.
struct Blank<'a> { field: &'a SpatialField,margin: f64,extent: Option<(V,V)>,recipe: Option<crate::json::Json>,
    boxed: std::sync::OnceLock<Option<(V,V)>>,mm: f64 }

impl dispatch::Blank for Blank<'_> {
    fn inside(&self,p: V) -> bool { self.field.below(p,-self.margin) }
    fn extent(&self) -> Option<(V,V)> { self.extent }
    fn boxed(&self) -> Option<(V,V)> { *self.boxed.get_or_init(|| boxed(self.recipe.as_ref()?,self.mm)) }
}

/// What the drawing reads of one sweep for the kernel: its tool's B-rep, its trajectory and roll,
/// the names its refusals say, and its pocket where the drawing writes its tool as a stock cut by
/// one (the planar class's reading, #721 F1d) — or why the drawing reads none, refused at T1.
struct Read { tool: Result<std::sync::Arc<crate::brep::topo::Brep>,String>,path: crate::brep::trajectory::Trajectory,roll: [f64;2],
    names: crate::brep::swept::subtraction::Names,pocketed: bool,
    family: crate::motion::Family }

/// Sweep `swept` read off the drawing: everything the kernel's dispatch is handed, or the message
/// T1 refuses it with.
fn read(sk: &Sketch,swept: usize) -> Result<Read,String> {
    let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else { return Err(format!("`{}` is no sweep",sk.solids[swept].name)) };
    let family = crate::motion::Family::read(sk,*motion as usize)?;
    let path = family.trajectory()?;
    let roll = super::roll::interval(sk,swept)?;
    let tool = cad::tool_brep(sk,swept);
    let names = crate::brep::swept::subtraction::Names {sweep:sk.solids[swept].name.clone(),tool:sk.solids[*source as usize].name.clone(),
        motion:family.name.clone()};
    // whether the drawing writes the tool as a stock cut by one pocket: the planar class's refusal
    // stands for it, the pocket read from the tool's B-rep by the kernel (#721 F1d)
    let pocketed = super::planar_class::asks(sk,swept);
    Ok(Read {tool,path,roll,names,pocketed,family})
}

/// Admit or refuse every swept cut of the body `root` (the body it places, where `root` is a
/// placement): each sweep read off the drawing and handed to the kernel, which chooses its class
/// (`brep::swept::dispatch::sweep`, #721 F1c), the sweeps asked side by side. What is admitted or
/// refused, and with which evidence, is what asking them one after another in order says; what is
/// here is the drawing's reading and the refusals' words.
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
    // a sweep of a member of a union tool is refused as the union's, saying the member (#684 C)
    let said = |swept: usize,message: String| match sk.distributed.contains_key(&(swept as u32)) {
        true => match &sk.solids[swept].def {
            SolidDef::Swept {source,..} => format!("its tool's member `{}`: {message}",sk.solids[*source as usize].name),
            _ => message,
        },
        false => message,
    };
    let bounds = field.support_bounds().ok().flatten();
    let size = bounds.map_or(1.,|b| crate::space::box_centre_diagonal(&b).1);
    let mm = cad::millimetres(sk);
    let blank = Blank {field:&field,margin:options.margin,extent:bounds.map(|b| (b.map(|x| x.bounds()[0]),b.map(|x| x.bounds()[1]))),
        recipe:cad::recipe_static(sk,root).ok().map(|r| r.recipe),boxed:std::sync::OnceLock::new(),mm};
    // the blank a solid of revolution about the line every placement turns about: the later
    // placements read it as the first does, proved by the solid graph
    let revolved = structurally_alike(sk,root,&by_sweep,size);
    let reads: Vec<Result<Read,String>> = order.iter().map(|&swept| read(sk,swept)).collect();
    let clock = crate::clock::Instant::now();
    let asked = crate::par::indices(order.len(),|k| -> Option<Result<dispatch::Admitted,dispatch::Refused>> {
        let r = reads[k].as_ref().ok()?;
        let s = dispatch::Sweep {tool:r.tool.as_deref().map_err(String::as_str),path:&r.path,roll:r.roll,mm,names:&r.names,pocketed:r.pocketed};
        let ask = dispatch::Ask {poses:&by_sweep[&order[k]],revolved,sampling:options.sampling(),margin:options.margin};
        Some(dispatch::sweep(&s,&blank,&ask))
    });
    if std::env::var_os("SOLVENT_ADMISSION_TIMES").is_some() { eprintln!("admission: asked in {:?}",clock.elapsed()); }
    let mut admission = Admission {body:root,sweeps:Vec::new()};
    for ((&swept,read),asked) in order.iter().zip(reads).zip(asked) {
        let name = sk.solids[swept].name.clone();
        let poses = &by_sweep[&swept];
        let read = read.map_err(|m| Error::Refused(Refusal {condition:Condition::Tool,sweep:name.clone(),message:said(swept,m),
            witness:None,asked:Asked::Generating}))?;
        let refusal = |r: dispatch::Refused| {
            let message = match r.seen {
                dispatch::Seen::Words(m) => m,
                dispatch::Seen::Uncharted(_) => read.family.uncharted(),
                dispatch::Seen::Envelope(e) => crate::envelope::planar::said(e,&read.family.name).message,
                dispatch::Seen::Canal(c,radius) => match super::canal::said(crate::brep::swept::canal::Why::Refused(c),radius) {
                    super::canal::Why::Refused(r) => r.message,
                    super::canal::Why::Failed(m) => m,
                },
            };
            Refusal {condition:r.row,sweep:name.clone(),message:said(swept,message),witness:r.witness,asked:r.asked}
        };
        let admitted = asked.expect("a read sweep is asked").map_err(|r| Error::Refused(refusal(r)))?;
        let class = match admitted.class {
            dispatch::Class::Canal(_) => return Err(Error::Unreadable(match super::canal::built(sk,swept) {
                Some(Err(m)) => m,
                _ => format!("`{name}` is a ball's tube, which is built as static material, not cut as a sweep"),
            })),
            dispatch::Class::Planar(found) => Class::Planar(found),
            dispatch::Class::Subtraction(found) => Class::Subtraction(found),
            dispatch::Class::Generating(None) => Class::Generating,
            dispatch::Class::Generating(Some(capped)) => Class::Capped(capped),
            dispatch::Class::ConstantTwist(found) => Class::ConstantTwist(found),
            dispatch::Class::Closed(c) => Class::Closed(Box::new(super::closed_class::Closed {tool:c.tool,tool_name:read.names.tool.clone(),
                path:c.path,roll:c.roll,first:c.first.map(|r| refusal(*r))})),
        };
        admission.sweeps.push(SweepEvidence {sweep:swept,name,
            placements:poses.iter().zip(&admitted.placements).map(|(&pose,&equivalent_to)| Placement {pose,equivalent_to}).collect(),
            samples:admitted.samples,contacts:admitted.contacts,spacing:admitted.spacing,least_area_factor:admitted.least_area_factor,
            near_double_roots:admitted.near_double_roots,near_tangent_pairs:admitted.near_tangent_pairs,
            basis:Basis::Sampled {rows:admitted.basis[0],columns:admitted.basis[1]},equivalence:admitted.equivalence,class});
    }
    Ok(admission)
}

/// The box body `root`'s static blank lies in (model units, least and greatest corner), read off
/// its own boundary built by this kernel: a field's support is a construction's box, not the
/// material's. What the planar classes box the blank by.
pub fn blank_box(sk: &Sketch,root: usize) -> Option<(V,V)> {
    boxed(&cad::recipe_static(sk,root).ok()?.recipe,cad::millimetres(sk))
}

/// The box static recipe `recipe` (millimetres) stands in, built by this kernel, in model units
/// (`mm` a unit).
fn boxed(recipe: &crate::json::Json,mm: f64) -> Option<(V,V)> {
    let (lo,hi) = crate::brep::recipe::build(recipe).ok()?.bounds();
    Some((lo.map(|x| x/mm),hi.map(|x| x/mm)))
}
