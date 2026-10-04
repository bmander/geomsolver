//! The generating-sweep class (docs/generating-sweeps.md): which continuous sweeps a boundary
//! is constructed for, asked of the solved model before any construction. A tool that is a
//! revolution of lines and arcs (or an intersection of such), turned by rotations about fixed
//! axes at constant ratios, clear of the blank at both ends of its roll, touching each blank
//! point once, never folding and never crossing itself: that is what generates a bevel or a
//! hypoid, and everything else is refused by the condition it fails, with a point that fails
//! it. The conditions are **sampled**, never certified, and the evidence says so: a sweep this
//! admits has shown no failure at the samples it names, which is not a proof that none exists
//! between them. Material evaluation answers membership for every sweep, admitted or not; only
//! boundary construction is restricted.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{SpatialField,SweepContacts,TimedContact,cad};
use crate::{envelope::{Motion,SurfacePoint},model::{EntKind,Sketch,SolidDef}};
use std::{collections::BTreeMap,f64::consts::{PI,TAU},fmt};

use crate::space::{sub,dot,cross,norm};

type V = [f64;3];

/// The rows of the class, in the order they are asked.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Condition { Tool, Corner, Motion, Stationary, Reach, Clearance, Single, Fold, Crossing, Regular, Once, Edgewise, Lead,
    Plane, Prisms, Period, Envelope, Wall }

impl Condition {
    pub fn code(self) -> &'static str {
        match self { Condition::Tool => "T1", Condition::Corner => "T2", Condition::Motion => "M1",
            Condition::Stationary => "M2", Condition::Reach => "E0", Condition::Clearance => "E1", Condition::Single => "E2",
            Condition::Fold => "E3", Condition::Crossing => "E4", Condition::Regular => "S1", Condition::Once => "S2",
            Condition::Edgewise => "S3", Condition::Lead => "S4", Condition::Plane => "P1", Condition::Prisms => "P2",
            Condition::Period => "P3", Condition::Envelope => "P4", Condition::Wall => "P5" }
    }
    /// What the row asks, in the words the scope document uses.
    pub fn rule(self) -> &'static str {
        match self {
            Condition::Tool => "the tool is a full revolution, or a prism with its caps clear of the blank, of lines and arcs, \
                or an intersection of such",
            Condition::Corner => "every profile corner the sweep carries into the blank is convex or tangent",
            Condition::Motion => "the motion is rotations about fixed axes at constant ratios, or one screw",
            Condition::Stationary => "the contact condition depends on the motion",
            Condition::Reach => "the sweep's contacts reach the blank",
            Condition::Clearance => "the tool is clear of the blank at both ends of the roll",
            Condition::Single => "each tool point touches the blank at most once in the roll",
            Condition::Fold => "the generated surface does not fold",
            Condition::Crossing => "the generated surface does not cross itself",
            Condition::Regular => "the screw's characteristic is a regular curve through the blank",
            Condition::Once => "the characteristic reaches the blank in one stretch, one point a ring",
            Condition::Edgewise => "the characteristic never runs along the screw's path",
            Condition::Lead => "the screw's sweep does not cross itself within a lead",
            Condition::Plane => "the motion turns a plane in itself about axes square to it",
            Condition::Prisms => "the tool is a stock cut by one pocket, prisms square to that plane through the blank, the \
                pocket's profile one closed curve",
            Condition::Period => "the roll is a whole period of the motion",
            Condition::Envelope => "the pocket's inner envelope is one regular loop that neither folds nor crosses itself",
            Condition::Wall => "the stock's outer wall never reaches the blank",
        }
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

/// The class a sweep's rows are read from.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Asked { Generating,ConstantTwist,Planar }

impl Refusal {
    /// The class the sweep was asked to join, by its motion and its tool.
    pub fn class(&self) -> &'static str {
        match self.asked { Asked::Generating => "generating-sweep class",Asked::ConstantTwist => "constant-twist class",
            Asked::Planar => "planar generating class" }
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
pub enum Class { Generating,ConstantTwist(Box<super::constant_twist::Characteristic>),Planar(Box<super::planar_class::Found>) }

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
            let line = &sk.lines[*axis as usize];
            let (a,b) = (lift(line.p1 as usize),lift(line.p2 as usize));
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
        // a fillet's rings are not proved alike under a turn; its placements are sampled
        SolidDef::Fillet {..} => None,
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
        SolidDef::Prism {..} | SolidDef::Through {..} | SolidDef::Loft {..} | SolidDef::Swept {..} => None,
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
        SolidDef::Body {stock,on,through,bound} => {
            if !on.is_empty() { return Err(format!("`{}` adds solids together; only intersections are in the class",solid.name)); }
            if !through.is_empty() { return Err(format!("`{}` cuts solids; only intersections are in the class",solid.name)); }
            std::iter::once(stock).chain(bound).try_for_each(|&o| tool(sk,o as usize))
        }
        // a prism's sides are extrusions of lines and arcs, its contact set fixed by the motion
        SolidDef::Prism {..} => Ok(()),
        SolidDef::Loft {..} => Err(format!("`{}` is a loft, not a revolution",solid.name)),
        SolidDef::Through {..} => Err(format!("`{}` is a through prism, not a revolution",solid.name)),
        SolidDef::Swept {..} => Err(format!("`{}` is itself a sweep",solid.name)),
        SolidDef::Fillet {..} => Err(format!("`{}` is a fillet, not a revolution",solid.name)),
    }
}

/// Which tool points the checks read the blank at, and whether each was inside: two placements
/// agreeing on all of them would compute the same checks to the last number.
struct Reads { points: Vec<V>,inside: Vec<bool> }

impl Reads {
    fn new() -> Reads { Reads {points:Vec::new(),inside:Vec::new()} }
    /// Read the blank at `p`, and remember that it was read and what it said.
    fn read(&mut self,inside: &dyn Fn(V) -> bool,p: V) -> bool { let i = inside(p); self.points.push(p); self.inside.push(i); i }
    fn extend(&mut self,other: Reads) { self.points.extend(other.points); self.inside.extend(other.inside); }
    /// Remember a read made elsewhere.
    fn push(&mut self,p: V,inside: bool) { self.points.push(p); self.inside.push(inside); }
}

/// What one sample of a tool face says of itself alone, for the fine pass: its point, the rate of its
/// contact condition, and — where its roots could be found — each root, whether it is in the blank,
/// and for one that is, the generated surface's area factor there (`area_factor`).
struct Sampled { s: SurfacePoint,rate: Option<crate::motion::NormalVelocity>,roots: Option<Vec<(TimedContact,bool,Option<f64>)>> }

/// E3's area factor at a contact `t` of the tool point at (u, v) of face `patch`: the generated
/// surface's oriented area factor on the same branch, by central differences, against the tool
/// surface's own. None where a neighbouring root or point cannot be read, or the tool's own area
/// vanishes.
fn area_factor(surface: &super::ToolSurface,patch: usize,
    roots: &(dyn Fn(usize,f64,f64) -> Result<Vec<TimedContact>,String>+Sync),t: &TimedContact,u: f64,v: f64) -> Option<f64> {
    let h = 1e-5;
    let (ua,ub) = ((u-h).max(0.),(u+h).min(1.));
    // round a revolution `v` comes back; along a prism it stops at the caps
    let wrap = |vv: f64| if surface.periodic() { vv.rem_euclid(1.) } else { vv };
    let (va,vb) = if surface.periodic() { (v-h,v+h) } else { ((v-h).max(0.),(v+h).min(1.)) };
    let near = |uu: f64,vv: f64| -> Option<V> {
        roots(patch,uu,wrap(vv)).ok()?.into_iter()
            .filter(|x| x.root.branch == t.root.branch)
            .min_by(|a,b| (a.root.time-t.root.time).abs().total_cmp(&(b.root.time-t.root.time).abs()))
            .map(|x| x.contact.position)
    };
    let (Some(fa),Some(fb),Some(ga),Some(gb)) = (near(ua,v),near(ub,v),near(u,va),near(u,vb)) else { return None };
    let at = |uu: f64,vv: f64| surface.at(uu,wrap(vv)).map(|s| s.position).ok();
    let (Some(sa),Some(sb),Some(ta),Some(tb)) = (at(ua,v),at(ub,v),at(u,va),at(u,vb)) else { return None };
    let (du,dv) = (ub-ua,vb-va);
    let area = norm(cross(sub(sb,sa).map(|x| x/du),sub(tb,ta).map(|x| x/dv)));
    if area == 0. { return None; }
    Some(dot(cross(sub(fb,fa).map(|x| x/du),sub(gb,ga).map(|x| x/dv)),t.contact.normal)/area)
}

/// The coarse pass's `j`-th of `n` stations of `v` over a face: round a revolution `n` steps of a
/// turn, along a prism from one cap to the other, both ends included.
fn coarse_v(surface: &super::ToolSurface,j: usize,n: usize) -> f64 {
    if surface.periodic() { j as f64/n as f64 } else { coarse_at(j as f64,n) }
}

/// Station `at` (fractional) of `n` from one cap to the other.
fn coarse_at(at: f64,n: usize) -> f64 { (at/(n.max(2)-1) as f64).clamp(0.,1.) }

/// What the fine pass over one tool face found.
struct Fine { reads: Reads,samples: usize,near_double_roots: usize,hits: Vec<(Hit,usize,usize)>,gaps: Vec<f64>,least: f64,
    signs: BTreeMap<(usize,usize),([usize;2],[Option<V>;2])> }

/// Admit or refuse every swept cut of the body `root`. The sweeps are checked side by side and a
/// placement's reads against the first placement's on every core; what is admitted or refused, and
/// with which evidence, is what checking them one after another in order says.
pub fn admit_body(sk: &Sketch,root: usize,options: &Options) -> Result<Admission,Error> {
    let (field,cuts) = static_remainder(sk,root,options.axis_tolerance).map_err(Error::Unreadable)?;
    let mut by_sweep: BTreeMap<usize,Vec<Motion>> = BTreeMap::new();
    let mut order = Vec::new();
    for cut in cuts {
        if !by_sweep.contains_key(&cut.swept) { order.push(cut.swept); }
        by_sweep.entry(cut.swept).or_default().push(cut.pose);
    }
    let names: BTreeMap<usize,String> = order.iter().map(|&s| (s,sk.solids[s].name.clone())).collect();
    let asked: BTreeMap<usize,Asked> = order.iter().map(|&s| (s,if super::planar_class::asks(sk,s) { Asked::Planar }
        else { match &sk.solids[s].def {
            SolidDef::Swept {motion,..} if crate::motion::Family::read(sk,*motion as usize).is_ok_and(|f| f.screw().is_some()) => Asked::ConstantTwist,
            _ => Asked::Generating,
        } })).collect();
    let refuse = |swept: usize,condition,message: String,witness|
        Error::Refused(Refusal {condition,sweep:names[&swept].clone(),message,witness,asked:asked[&swept]});
    let field = &field;
    let bounds = field.support_bounds().ok().flatten();
    // a pocketed prism under a planar motion: rows P1–P5, every placement checked, the blank boxed
    // by its own boundary (a field's support is a construction's box, not the material's)
    let mut planar: Vec<SweepEvidence> = Vec::new();
    let planar_box = order.iter().any(|s| asked[s] == Asked::Planar).then(|| -> Option<(V,V)> {
        let mm = cad::millimetres(sk).ok()?;
        let blank = crate::brep::recipe::build(&cad::recipe_static(sk,root).ok()?.recipe).ok()?;
        let (lo,hi) = blank.bounds();
        Some((lo.map(|x| x/mm),hi.map(|x| x/mm)))
    }).flatten();
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
    let check = |contacts: &SweepContacts,pose: Motion,inside: &(dyn Fn(V) -> bool+Sync)| match contacts.motion().screw() {
        Some(screw) => check_screw(contacts,inside,bounds.map(|b| screw.extent(b.map(|x| x.bounds()[0]),b.map(|x| x.bounds()[1]),pose.inverse())),
            options),
        None => check(contacts,inside,options),
    };
    let inside_at = |pose: Motion| move |p: V| field.value(pose.point(p)) < -options.margin;
    // Each sweep's tool read, then its first placement checked.
    let tools: Vec<Result<SweepContacts,Error>> = order.iter().map(|&swept| {
        let SolidDef::Swept {source,..} = &sk.solids[swept].def else { unreachable!("a swept cut is a sweep") };
        tool(sk,*source as usize).map_err(|m| refuse(swept,Condition::Tool,m,None))?;
        let contacts = SweepContacts::read(sk,swept,options.axis_tolerance).map_err(|m| refuse(swept,Condition::Tool,m,None))?;
        if contacts.motion().screw().is_none() {
            let probe = SurfacePoint {position:[1.,2.,3.],du:[1.,0.,0.],dv:[0.,1.,0.]};
            contacts.motion().normal_velocity(probe).map_err(|m| refuse(swept,Condition::Motion,m,None))?;
        }
        Ok(contacts)
    }).collect();
    let clock = crate::clock::Instant::now();
    let checks = crate::par::indices(order.len(),|k| -> Option<Result<(SweepEvidence,Reads),Error>> {
        let contacts = tools[k].as_ref().ok()?;
        let (swept,pose) = (order[k],by_sweep[&order[k]][0]);
        Some(check(contacts,pose,&inside_at(pose)).map_err(|(c,m,w)| refuse(swept,c,m,w.map(|p| pose.point(p)))))
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
    let mut alike = alike.into_iter();
    for (result,&swept) in first.into_iter().zip(&order) {
        let (contacts,found,reads) = result?;
        let poses = &by_sweep[&swept];
        let mut placements = vec![Placement {pose:poses[0],equivalent_to:None}];
        let mut checked: Vec<(usize,Reads)> = vec![(0,reads)];
        let mut found = found;
        if let Some((origin,axis)) = revolved {
            placements.extend(poses.iter().skip(1).map(|pose| Placement {pose:*pose,equivalent_to:Some(0)}));
            found.sweep = swept; found.name = names[&swept].clone(); found.placements = placements;
            found.equivalence = Equivalence::Revolved {origin,axis};
            admission.sweeps.push(found);
            continue;
        }
        for (k,pose) in poses.iter().enumerate().skip(1) {
            if alike.next().expect("one answer a later placement") {
                placements.push(Placement {pose:*pose,equivalent_to:Some(0)}); continue;
            }
            let inside = inside_at(*pose);
            let same = checked.iter().skip(1).find(|(_,reads)| reads.points.iter().zip(&reads.inside).all(|(p,&i)| inside(*p) == i));
            if let Some((j,_)) = same { placements.push(Placement {pose:*pose,equivalent_to:Some(*j)}); continue; }
            let (_,reads) = check(&contacts,*pose,&inside).map_err(|(c,m,w)| refuse(swept,c,m,w.map(|p| pose.point(p))))?;
            checked.push((k,reads));
            placements.push(Placement {pose:*pose,equivalent_to:None});
        }
        found.sweep = swept; found.name = names[&swept].clone(); found.placements = placements;
        admission.sweeps.push(found);
    }
    Ok(admission)
}

/// One in-blank contact of the fine pass.
#[derive(Clone,Copy)]
struct Hit { source: V,position: V,normal: V }

use super::constant_twist::Failure;

/// T2 and E1 for one placement, asked of every class before anything that might be reported in
/// their place: the blank's reads, and the poses the roll is sampled at.
fn ends(c: &SweepContacts,inside: &(dyn Fn(V) -> bool+Sync),options: &Options) -> Result<(Reads,Vec<Motion>),Failure> {
    let tool = c.source_material();
    let on_tool = |p: V| tool.value(p).abs() < 1e-6;
    let domain = c.domain();
    let pose = |t: f64| c.motion().at(t).map_err(|m| (Condition::Motion,m,None));
    let limits = [pose(domain[0])?,pose(domain[1])?];
    let mut reads = Reads::new();
    // T2: a concave corner carried through the blank trims two envelopes against each other.
    // Asked first, since the crossing it makes would otherwise be reported in its place.
    let steps = 64;
    let poses: Vec<Motion> = (0..=steps).map(|k| pose(domain[0]+(domain[1]-domain[0])*k as f64/steps as f64))
        .collect::<Result<_,_>>()?;
    for edge in c.edges() {
        let ([a,b],[ua,ub]) = (edge.patches,edge.ends);
        let (Some(pa),Some(pb)) = (c.patches().get(a),c.patches().get(b)) else { continue };
        let h = 1e-4;
        let inward = |u: f64| if u < 0.5 { u+h } else { u-h };
        let (Ok(corner),Ok(sa),Ok(sb)) = (pa.at(ua,0.),pa.at(inward(ua),0.),pb.at(inward(ub),0.)) else { continue };
        if !on_tool(corner.position) { continue; }
        let (ta,tb) = (sub(sa.position,corner.position),sub(sb.position,corner.position));
        // Opposite tangents: the profile is smooth there.
        if dot(ta,tb) < -(1.-1e-9)*norm(ta)*norm(tb) { continue; }
        let chord: V = std::array::from_fn(|k| 0.5*(sa.position[k]+sb.position[k]));
        if tool.value(chord) <= 0. { continue; }
        for j in 0..options.coarse_columns {
            let Ok(point) = pa.at(ua,j as f64/options.coarse_columns as f64) else { continue };
            if let Some(p) = poses.iter().map(|m| m.point(point.position)).find(|&p| reads.read(inside,p)) {
                return Err((Condition::Corner,format!("`{}` meets `{}` at a concave corner",pa.name(),pb.name()),Some(p)));
            }
        }
    }
    // E1, over the whole tool, before anything that might be reported in its place.
    let clear = crate::par::map(c.patches(),|surface| -> Result<Reads,Failure> {
        let mut own = Reads::new();
        for i in 0..=options.coarse_rows { for j in 0..options.coarse_columns {
            let Ok(s) = surface.at(i as f64/options.coarse_rows as f64,coarse_v(surface,j,options.coarse_columns)) else { continue };
            if !on_tool(s.position) { continue; }
            for limit in &limits {
                let p = limit.point(s.position);
                if own.read(inside,p) { return Err((Condition::Clearance,
                    "the tool at a limit of its roll lies in the blank".into(),Some(p))); }
            }
        }}
        Ok(own)
    });
    for own in clear { reads.extend(own?); }
    // T1 for a prism: the class cuts with its sides, so neither cap may pass through the blank
    // (under a planar sweep a cap, square to the turn, is in contact at every time or none)
    for surface in c.patches().iter().filter(|s| !s.periodic()) {
        for v in [0.,1.] { for i in 0..=options.coarse_rows {
            let Ok(s) = surface.at(i as f64/options.coarse_rows as f64,v) else { continue };
            if let Some(p) = poses.iter().map(|m| m.point(s.position)).find(|&p| reads.read(inside,p)) {
                return Err((Condition::Tool,format!("a cap of the prism `{}` bounds passes through the blank, and the class cuts \
                    with a prism's sides only: make the prism longer than the blank is thick",surface.name()),Some(p)));
            }
        } }
    }
    Ok((reads,poses))
}

/// M2, E1–E4 and T2 for one placement, `inside` being the blank read in the sweep's frame. The
/// tool's faces are sampled side by side, each on its own core, and their findings taken in the
/// faces' order: the first failure in that order is the one reported, as one pass over them would.
fn check(c: &SweepContacts,inside: &(dyn Fn(V) -> bool+Sync),options: &Options) -> Result<(SweepEvidence,Reads),Failure> {
    let tool = c.source_material();
    let on_tool = |p: V| tool.value(p).abs() < 1e-6;
    let domain = c.domain();
    let middle = 0.5*(domain[0]+domain[1]);
    let pose = |t: f64| c.motion().at(t).map_err(|m| (Condition::Motion,m,None));
    let times = [pose(domain[0])?,pose(middle)?,pose(domain[1])?];
    let roots = |patch: usize,u: f64,v: f64| c.at_source_over(patch,u,v,domain,options.root_tolerance);
    let (mut reads,poses) = ends(c,inside,options)?;
    // M2 at the poles: where a face's profile meets the tool's axis the surface has no normal of
    // its own, so the sampled checks below cannot evaluate it. Its normal is the axis, the limit
    // along the profile; a pole whose contact condition is zero at every time and whose path
    // enters the blank is refused, as a sampled point would be.
    for surface in c.patches() {
        for end in [0.,1.] {
            let (Ok(s),Ok(near)) = (surface.at(end,0.),surface.at(if end == 0. { 1e-6 } else { 1.-1e-6 },0.)) else { continue };
            if norm(s.dv) > 1e-9*(1.+norm(s.position)) || !on_tool(s.position) { continue; }
            // The pole's normal is the revolution axis, oriented as the surface beside it: the
            // axis is square to the small circle a point beside the pole turns on.
            let inward = if end == 0. { 1e-6 } else { 1.-1e-6 };
            let ring: Vec<V> = [0.,1./3.,2./3.].iter().filter_map(|&v| surface.at(inward,v).ok().map(|q| q.position)).collect();
            if ring.len() < 3 { continue; }
            let axis = cross(sub(ring[1],ring[0]),sub(ring[2],ring[0]));
            let beside = cross(near.du,near.dv);
            if norm(axis) == 0. || norm(beside) == 0. { continue; }
            let n = axis.map(|x| x/norm(axis)*dot(axis,beside).signum());
            let seed = if n[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
            let a = cross(seed,n); let a = a.map(|x| x/norm(a));
            let pole = SurfacePoint {position:s.position,du:a,dv:cross(n,a)};
            let Ok(g) = c.motion().normal_velocity(pole) else { continue };
            let scale = 1.+norm(s.position);
            let values = [domain[0],middle,domain[1]].map(|t| g.at(t).unwrap_or(f64::NAN));
            if !values.iter().all(|x| x.abs() <= options.root_tolerance*scale) { continue; }
            if let Some(p) = poses.iter().map(|m| m.point(s.position)).find(|&p| reads.read(inside,p)) {
                return Err((Condition::Stationary,format!("the pole of `{}`, where its profile meets its axis, is in contact \
                    at every time",surface.name()),Some(p)));
            }
        }
    }
    // E3's sign: the area factor times the rate of the contact condition. The factor alone
    // runs through infinity and changes sign where a point's two contact times merge, which
    // is a fold of the tool's time chart and not of the surface; the product stays finite
    // there, and changes sign only where the generated surface itself folds.
    let fine = crate::par::indices(c.patches().len(),|patch| -> Result<Fine,Failure> {
        let mut own = Reads::new();
        let (mut samples,mut near_double_roots) = (0,0);
        let mut hits: Vec<(Hit,usize,usize)> = Vec::new();
        let mut gaps: Vec<f64> = Vec::new();
        let mut least = f64::INFINITY;
        let mut signs: BTreeMap<(usize,usize),([usize;2],[Option<V>;2])> = BTreeMap::new();
        let surface = &c.patches()[patch];
        let (cu,cv) = (options.coarse_rows,options.coarse_columns);
        let mut reach = vec![false;cv];
        // the coarse rows on every core, each its reads and which angles reach
        for (row,reads) in crate::par::indices(cu+1,|i| {
            let mut own = Reads::new();
            let mut row = vec![false;cv];
            for j in 0..cv {
                let (u,v) = (i as f64/cu as f64,coarse_v(surface,j,cv));
                let Ok(s) = surface.at(u,v) else { continue };
                if !on_tool(s.position) { continue; }
                if let Ok(list) = roots(patch,u,v) {
                    if list.iter().any(|t| own.read(inside,t.contact.position)) { row[j] = true; }
                }
            }
            (row,own)
        }) {
            for (r,x) in reach.iter_mut().zip(row) { *r |= x; }
            own.extend(reads);
        }
        let own = std::cell::RefCell::new(own);
        let read = |p: V| own.borrow_mut().read(inside,p);
        // The shortest arc of the revolution holding every reaching angle, widened a step.
        let reached: Vec<usize> = (0..cv).filter(|&j| reach[j]).collect();
        let band = if reached.is_empty() { None } else if !surface.periodic() {
            // along a prism the reaching stretch runs between its least and greatest, widened
            let (lo,hi) = (reached[0].saturating_sub(2),(reached[reached.len()-1]+2).min(cv-1));
            Some((lo,hi-lo))
        } else {
            let mut gap = (0,0);
            for w in 0..reached.len() {
                let (a,b) = (reached[w],reached[(w+1)%reached.len()]);
                let g = match (b+cv-a)%cv { 0 => cv, g => g };
                if g > gap.1 { gap = (w,g); }
            }
            Some(((reached[(gap.0+1)%reached.len()]+cv-2)%cv,(cv-gap.1+4).min(cv)))
        };
        let (nu,nv) = (options.rows,options.columns);
        // Without a band nothing of this patch reaches the blank in the roll; its clearance
        // and stationarity are still asked over a band of the whole revolution.
        let (start,width) = band.unwrap_or((0,cv));
        let v_at = |j: usize| {
            let at = start as f64+width as f64*j as f64/nv as f64;
            if surface.periodic() { (at/cv as f64).rem_euclid(1.) } else { coarse_at(at,cv) }
        };
        let mut previous: Vec<Option<V>> = vec![None;nv+1];
        // Each sample's contact equation, to find stationary points between samples.
        let mut equations: Vec<Option<([f64;3],V)>> = vec![None;nv+1];
        // What each sample says of itself alone — its point, the rate of its contact condition, its
        // roots with whether each is in the blank, and each such root's area factor — worked out on
        // every core a row at a time; what samples say of their neighbours is asked below, in order.
        let sampled: Vec<Vec<Option<Sampled>>> = crate::par::indices(nu+1,|i| {
            let u = i as f64/nu as f64;
            (0..=nv).map(|j| {
                let v = v_at(j);
                let s = surface.at(u,v).ok()?;
                if !on_tool(s.position) { return None; }
                let rate = c.motion().normal_velocity(s).ok();
                let roots = roots(patch,u,v).ok().map(|list| list.into_iter().map(|t| {
                    let within = inside(t.contact.position);
                    let factor = if within { area_factor(surface,patch,&roots,&t,u,v) } else { None };
                    (t,within,factor)
                }).collect());
                Some(Sampled {s,rate,roots})
            }).collect()
        });
        for sampled in sampled {
            let mut row: Vec<Option<V>> = vec![None;nv+1];
            let mut here: Vec<Option<([f64;3],V)>> = vec![None;nv+1];
            for (j,sample) in sampled.into_iter().enumerate() {
                let Some(Sampled {s,rate,roots:list}) = sample else { continue };
                samples += 1;
                // M2: where a point's contact condition does not change over the roll, the
                // point is on the boundary at every time or at none, and its time is no
                // parameter of the generated surface. A single rotation is that everywhere.
                // M2 between samples: the equation is C + a cos + b sin in the roll. Where (a, b)
                // turns right round between neighbours while roots exist on both sides, it passed
                // through zero with C: a point between them is in contact at every time, as on a
                // curve of such points, which no single sample is on.
                if let Some(g) = &rate {
                    let k = g.variation();
                    here[j] = Some((k,s.position));
                    // An isolated such point lies inside a sample cell rather than on a segment
                    // between two samples: there (a, b) winds right round the cell's corners.
                    if j > 0 { if let (Some(p0),Some(p1),Some(p2)) = (equations[j-1],equations[j],here[j-1]) {
                        let corners = [p0,p1,(k,s.position),p2];
                        let reached = corners.iter().all(|(m,_)| m[0].abs() <= m[1].dhypot(m[2]));
                        let mut turn = 0.;
                        for w in 0..4 {
                            let (m,n) = (corners[w].0,corners[(w+1)%4].0);
                            turn += (m[1]*n[2]-m[2]*n[1]).datan2(m[1]*n[1]+m[2]*n[2]);
                        }
                        if reached && turn.abs() > PI {
                            let centre: V = std::array::from_fn(|n| corners.iter().map(|c| c.1[n]).sum::<f64>()/4.);
                            if let Some(p) = times.iter().map(|m| m.point(centre)).find(|&p| read(p)) {
                                return Err((Condition::Stationary,format!("a tool point of `{}` inside a sample cell is in contact \
                                    at every time",surface.name()),Some(p)));
                            }
                        }
                    } }
                    for neighbour in [equations[j],if j > 0 { here[j-1] } else { None }].into_iter().flatten() {
                        let (m,q) = neighbour;
                        let (a1,a2) = (m[1].dhypot(m[2]),k[1].dhypot(k[2]));
                        let turned = m[1]*k[1]+m[2]*k[2] < -0.5*a1*a2;
                        if turned && m[0].abs() <= a1 && k[0].abs() <= a2 {
                            let between: V = std::array::from_fn(|n| 0.5*(q[n]+s.position[n]));
                            if let Some(p) = times.iter().map(|m| m.point(between)).find(|&p| read(p)) {
                                return Err((Condition::Stationary,format!("tool points between samples of `{}` are in contact at every time",
                                    surface.name()),Some(p)));
                            }
                        }
                    }
                }
                if let Some(g) = &rate {
                    let scale = 1.+norm(s.position);
                    let at = [domain[0],middle,domain[1]].map(|t| g.at(t).unwrap_or(f64::NAN));
                    if at.iter().all(|x| (x-at[1]).abs() <= options.root_tolerance*scale) {
                        if let Some(p) = times.iter().map(|m| m.point(s.position)).find(|&p| read(p)) {
                            return Err((Condition::Stationary,
                                "a tool point's contact condition does not change over the roll".into(),Some(p)));
                        }
                    }
                }
                let Some(list) = list else { near_double_roots += 1; continue };
                // every root read, as a pass over them reads them
                for (t,within,_) in &list { own.borrow_mut().push(t.contact.position,*within); }
                let found: Vec<&(TimedContact,bool,Option<f64>)> = list.iter().filter(|x| x.1).collect();
                if found.len() > 1 {
                    return Err((Condition::Single,format!("a tool point touches the blank at roll {:.4} and again at {:.4}",
                        found[0].0.root.time,found[1].0.root.time),Some(found[1].0.contact.position)));
                }
                for &(t,_,factor) in found {
                    let hit = Hit {source:s.position,position:t.contact.position,normal:t.contact.normal};
                    row[j] = Some(hit.position);
                    for q in [previous[j],if j > 0 { row[j-1] } else { None }].into_iter().flatten() {
                        gaps.push(norm(sub(q,hit.position)));
                    }
                    hits.push((hit,patch,t.root.branch));
                    // E3: the generated surface's oriented area factor on the same branch
                    // (`area_factor`), against the tool surface's own.
                    let Some(factor) = factor else { continue };
                    if factor.abs() < options.least_factor {
                        return Err((Condition::Fold,format!("the generated surface's area factor is {factor:.3e}"),Some(t.contact.position)));
                    }
                    least = least.min(factor.abs());
                    let e = 1e-6;
                    let Some(g) = &rate else { continue };
                    let (Ok(g0),Ok(g1)) = (g.at(t.root.time-e),g.at(t.root.time+e)) else { continue };
                    let side = usize::from(factor*(g1-g0) < 0.);
                    let entry = signs.entry((patch,t.root.branch)).or_insert(([0,0],[None,None]));
                    entry.0[side] += 1;
                    if entry.1[side].is_none() { entry.1[side] = Some(t.contact.position); }
                }
            }
            previous = row;
            equations = here;
        }
        Ok(Fine {reads:own.into_inner(),samples,near_double_roots,hits,gaps,least,signs})
    });
    let (mut samples,mut near_double_roots) = (0,0);
    let mut hits: Vec<(Hit,usize,usize)> = Vec::new();
    let mut gaps: Vec<f64> = Vec::new();
    let mut least = f64::INFINITY;
    let mut signs: BTreeMap<(usize,usize),([usize;2],[Option<V>;2])> = BTreeMap::new();
    for found in fine {
        let found = found?;
        reads.extend(found.reads);
        samples += found.samples; near_double_roots += found.near_double_roots;
        hits.extend(found.hits); gaps.extend(found.gaps);
        least = least.min(found.least);
        signs.extend(found.signs);
    }
    for ((patch,_),(counts,at)) in &signs {
        if counts[0] > 0 && counts[1] > 0 {
            let minority = usize::from(counts[1] < counts[0]);
            return Err((Condition::Fold,format!("the generated surface of `{}` turns back on itself ({} samples one way, {} the other)",
                c.patches()[*patch].name(),counts[0],counts[1]),at[minority]));
        }
    }
    if hits.is_empty() {
        return Err((Condition::Reach,"no contact of the tool within the roll lies in the blank".into(),None));
    }
    // E4: contacts closer than the sample spacing whose tool points lie far apart.
    gaps.sort_by(f64::total_cmp);
    let spacing = gaps.get(gaps.len()*95/100).copied().unwrap_or(0.);
    let mut near_tangent_pairs = 0;
    if spacing > 0. {
        let mut grid: BTreeMap<[i64;3],Vec<usize>> = BTreeMap::new();
        for (k,(h,_,_)) in hits.iter().enumerate() { grid.entry(h.position.map(|x| (x/spacing).floor() as i64)).or_default().push(k); }
        for (k,(a,_,_)) in hits.iter().enumerate() {
            let key = a.position.map(|x| (x/spacing).floor() as i64);
            for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
                let Some(list) = grid.get(&[key[0]+dx,key[1]+dy,key[2]+dz]) else { continue };
                for &m in list {
                    if m <= k { continue; }
                    let b = &hits[m].0;
                    if norm(sub(a.position,b.position)) > 0.5*spacing || norm(sub(a.source,b.source)) < 5.*spacing { continue; }
                    if dot(a.normal,b.normal).abs() > 0.999 { near_tangent_pairs += 1; continue; }
                    return Err((Condition::Crossing,"two distant parts of the tool's sweep meet here".into(),Some(a.position)));
                }
            }}}
        }
    }
    Ok((SweepEvidence {sweep:0,name:String::new(),placements:Vec::new(),samples,contacts:hits.len(),spacing,
        least_area_factor:least,near_double_roots,near_tangent_pairs,
        basis:Basis::Sampled {rows:options.rows,columns:options.columns},equivalence:Equivalence::Sampled,class:Class::Generating},reads))
}

/// T2 and E1, then S1–S4 (`constant_twist`), for one placement of a sweep under a screw, `inside`
/// being the blank read in the sweep's frame, `heights` how far along the screw's axis it reaches
/// there. A screw's twist is constant, so M2 does not apply: a tool point is on the boundary at
/// every time or at none, and the boundary is the characteristic carried along.
fn check_screw(c: &SweepContacts,inside: &(dyn Fn(V) -> bool+Sync),extent: Option<([f64;2],f64)>,options: &Options)
    -> Result<(SweepEvidence,Reads),Failure> {
    let (reads,_) = ends(c,inside,options)?;
    let reads = std::sync::Mutex::new(reads);
    // the blank read outside the lock, the faces' rings side by side
    let read = |p: V| { let i = inside(p); reads.lock().unwrap_or_else(|e| e.into_inner()).push(p,i); i };
    let ask = super::constant_twist::Ask {rows:options.rows,root_tolerance:options.root_tolerance,least_factor:options.least_factor,
        roll:c.domain(),heights:extent.map(|e| e.0),radius:extent.map(|e| e.1),inside:&read};
    let found = super::constant_twist::Characteristic::walk(c,&ask)?;
    let [first,last] = found.reach;
    let mut gaps: Vec<f64> = (first..last).map(|w| crate::space::distance(found.nodes[w].position,found.nodes[w+1].position)).collect();
    gaps.sort_by(f64::total_cmp);
    let evidence = SweepEvidence {sweep:0,name:String::new(),placements:Vec::new(),samples:found.samples,contacts:last-first+1,
        spacing:gaps.get(gaps.len()/2).copied().unwrap_or(0.),least_area_factor:found.least_factor,near_double_roots:0,
        near_tangent_pairs:0,basis:Basis::Sampled {rows:options.rows,columns:360},equivalence:Equivalence::Sampled,
        class:Class::ConstantTwist(Box::new(found))};
    Ok((evidence,reads.into_inner().unwrap_or_else(|e| e.into_inner())))
}

