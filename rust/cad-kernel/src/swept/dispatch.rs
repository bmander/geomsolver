//! **A sweep's method, chosen by the kernel** (#721 F1c): one `sweep` that reads the tool from its
//! B-rep and the trajectory by its structure, asks the classes in order and admits the sweep by the
//! first that takes it, or refuses it by a typed row with a witness.
//!
//! 1. **The canal** (B1, B2): a tool whose every face lies on one sphere (`canal::ball`) sweeps its
//!    ball's tube; one a row refuses is refused here. A screw's tube that would meet itself is the
//!    constant twist's.
//! 2. **The planar class** (P1–P5): a stock cut by one pocket under a motion turning its plane in
//!    itself, read by the drawing until the kernel reads its pocket's curve from the B-rep, which
//!    fits it (`Sweep::pocket`, #721 F1d). Every placement is asked; what it refuses stands.
//! 3. **The planar subtraction class** (P1, Q1–Q4): under one constant twist, a tool standing along
//!    a direction whose square plane the twist keeps (`subtraction::Subtraction::read`).
//! 4. **The constant twist** (S1–S4) under a screw, a turn or a slide, **the generating class**
//!    (M1–E4, T1–T3) under any other motion: the tool read from its B-rep (`Tool::read`; T1 its
//!    refusal), its first placement asked, each later one read alike or asked again. One these
//!    refuse that the planar subtraction class reads is asked that class instead.
//! 5. **The closed class** (C1, C2): a sweep every class it was asked refuses, whose tool and
//!    motion the kernel's one arrangement builds (`closed::reads`, `closed::reads_general`); where it
//!    cannot, the first refusal stands.
//!
//! The rows are sampled, and the evidence says how. What the caller reads is the drawing: the tool's
//! B-rep, the motion lowered, the roll, the blank (`Blank`) at its placements, and the names a
//! refusal says. What it says of a refusal is its words.
use crate::contact::SurfacePoint;
use crate::topo::Brep;
use crate::trajectory::{Motion,Trajectory};
use super::admission::{self as rows,Capped,Generating,Lone,Reads,Sampling};
use super::canal::{self,Tube};
use super::constant_twist::{once_round,twist_of,Characteristic};
use crate::trajectory::Screw;
use super::generating::{Form,Uncharted};
use super::pocket::{self,Pocket};
use super::subtraction::{self,Names,Subtraction};
use super::tool::Tool;

type V = [f64;3];

/// The rows of every class, in the order the scope document lists them.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Row { Tool,Corner,Motion,Stationary,Reach,Clearance,Single,Fold,Crossing,Sections,Regular,Once,Edgewise,Lead,
    Plane,Prisms,Period,Envelope,Wall,Members,Slabs,Closes,Kept,Curvature,Return,Faces,Constant,Settled }

impl Row {
    /// The row's code in the scope document.
    pub fn code(self) -> &'static str {
        match self { Row::Tool => "T1",Row::Corner => "T2",Row::Motion => "M1",
            Row::Stationary => "M2",Row::Reach => "E0",Row::Clearance => "E1",Row::Single => "E2",
            Row::Fold => "E3",Row::Crossing => "E4",Row::Sections => "T3",Row::Regular => "S1",Row::Once => "S2",
            Row::Edgewise => "S3",Row::Lead => "S4",Row::Plane => "P1",Row::Prisms => "P2",
            Row::Period => "P3",Row::Envelope => "P4",Row::Wall => "P5",Row::Members => "Q1",
            Row::Slabs => "Q2",Row::Closes => "Q3",Row::Kept => "Q4",Row::Curvature => "B1",
            Row::Return => "B2",Row::Faces => "C1",Row::Constant => "C2",Row::Settled => "C3" }
    }
}

impl From<super::constant_twist::Row> for Row {
    fn from(row: super::constant_twist::Row) -> Row {
        use super::constant_twist::Row as R;
        match row {
            R::Motion => Row::Motion,R::Reach => Row::Reach,R::Regular => Row::Regular,R::Once => Row::Once,
            R::Edgewise => Row::Edgewise,R::Lead => Row::Lead,R::Clearance => Row::Clearance,R::Tool => Row::Tool,
            R::Corner => Row::Corner,R::Stationary => Row::Stationary,R::Single => Row::Single,R::Fold => Row::Fold,
            R::Crossing => Row::Crossing,R::Sections => Row::Sections,
        }
    }
}

impl From<canal::Row> for Row {
    fn from(row: canal::Row) -> Row { match row { canal::Row::Curvature => Row::Curvature,canal::Row::Return => Row::Return } }
}

/// The class a sweep's rows were read from: by its motion (a screw's) or its tool (prisms', a
/// ball's).
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Asked { Generating,ConstantTwist,Planar,Subtraction,Canal }

/// What a refusing row saw, where the kernel does not say it in words: a ball's path (the canal's
/// rows, `canal::Refusal`, and the ball's radius) or a motion with no contact chart (`Uncharted`),
/// which the drawing says in its own.
#[derive(Clone,Debug)]
pub enum Seen { Words(String),Canal(canal::Refusal,f64),Uncharted(Uncharted) }

/// A sweep refused: the class asked, the row, what was seen, and a point of the body (model units)
/// where, when there is one.
#[derive(Clone,Debug)]
pub struct Refused { pub asked: Asked,pub row: Row,pub seen: Seen,pub witness: Option<V> }

impl Refused {
    fn said(asked: Asked,(row,message,witness): (Row,String,Option<V>)) -> Refused {
        Refused {asked,row,seen:Seen::Words(message),witness}
    }
}

/// A sweep as the kernel reads it: its tool's B-rep (millimetres; or why the drawing has none, the
/// words T1 refuses a sweep with where no class reads the tool otherwise), its trajectory and its roll
/// (model units, `mm` a model unit), the names a refusal says, and — read by the drawing until the
/// kernel reads it from the B-rep (#721 F1d) — the tool as a stock cut by one pocket, or why the
/// planar class refuses it as one, where the drawing writes it so.
pub struct Sweep<'a> { pub tool: Result<&'a Brep,&'a str>,pub path: &'a Trajectory,pub roll: [f64;2],pub mm: f64,pub names: &'a Names,
    pub pocket: Option<&'a Result<Pocket,pocket::Failure>> }

/// The blank a sweep cuts, in the body's frame (model units): whether a point is in it, the box its
/// material's support stands in, and the box its own boundary stands in (each least and greatest
/// corner, none where unknown), which the planar classes box it by.
pub trait Blank: Sync {
    fn inside(&self,p: V) -> bool;
    fn extent(&self) -> Option<(V,V)>;
    fn boxed(&self) -> Option<(V,V)>;
}

/// What the sweep is asked at: its placements (the sweep's frame in the body's, the first first),
/// the line about which every placement is a turn of the first and the blank the same under every
/// turn — so each reads the blank as the first does, where the drawing proves it (a point of it and
/// its unit direction) — how finely the rows sample, and how deep a point must be in the stock to
/// be read clear of its wall (P5).
pub struct Ask<'a> { pub poses: &'a [Motion],pub revolved: Option<(V,V)>,pub sampling: Sampling,pub margin: f64 }

/// A sweep the closed class builds: its tool's B-rep, its trajectory unpaced (millimetres) and
/// the roll it is carried over.
#[derive(Clone,Debug)]
pub struct Closed { pub tool: Brep,pub path: Trajectory,pub roll: [f64;2] }

/// The class a sweep is admitted to, and what it found.
#[derive(Clone,Debug)]
pub enum Class { Canal(Box<Tube>),Planar(Box<pocket::Found>),Subtraction(Box<subtraction::Found>),Generating(Option<Box<Capped>>),
    ConstantTwist(Box<Characteristic>),Closed(Box<Closed>) }

/// How the later placements were found to read the blank as the first: by the drawing's proof
/// (`Ask::revolved`), or read at every point the first placement's rows read.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Equivalence { Revolved { origin: V,axis: V },Sampled }

/// A sweep admitted: its class, each placement's first placement it reads the blank alike to (none
/// where it was asked itself), what the rows sampled — tool points, their contacts in the blank,
/// how close neighbouring contacts are, the least area factor, the near double roots and near
/// tangent pairs — the rows and columns they were sampled on, and how the placements were read.
#[derive(Clone,Debug)]
pub struct Admitted { pub class: Class,pub placements: Vec<Option<usize>>,pub samples: usize,pub contacts: usize,pub spacing: f64,
    pub least_area_factor: f64,pub near_double_roots: usize,pub near_tangent_pairs: usize,pub basis: [usize;2],
    pub equivalence: Equivalence }

impl Admitted {
    /// The class's finding, every placement asked itself, nothing sampled.
    fn whole(class: Class,poses: usize) -> Admitted {
        Admitted {class,placements:vec![None;poses],samples:0,contacts:0,spacing:0.,least_area_factor:1.,near_double_roots:0,
            near_tangent_pairs:0,basis:[0;2],equivalence:Equivalence::Sampled}
    }
}

/// **The sweep `s` cut from `blank` at `ask`'s placements**, admitted by the first class that takes
/// it, in the order the module names, or refused by the first class asked (the closed class's
/// refusal never stands for it).
pub fn sweep(s: &Sweep,blank: &dyn Blank,ask: &Ask) -> Result<Admitted,Refused> {
    let poses = ask.poses;
    let Some(&first) = poses.first() else {
        return Err(Refused::said(Asked::Generating,(Row::Reach,"the sweep cuts the blank at no placement".into(),None)))
    };
    // the canal: a ball's tube, or its row's refusal
    if let Some((centre,radius,face)) = s.tool.ok().and_then(|b| canal::ball(b,s.mm)) {
        match canal::tube(s.path,centre,radius,&face,s.roll) {
            Ok(Some(tube)) => return Ok(Admitted::whole(Class::Canal(Box::new(tube)),poses.len())),
            Err(canal::Why::Refused(r)) => return Err(Refused {asked:Asked::Canal,row:r.row.into(),
                witness:Some(first.point(r.witness)),seen:Seen::Canal(r,radius)}),
            // a screw's tube that meets itself, or a path the tube cannot follow: the other classes'
            Ok(None) | Err(_) => {}
        }
    }
    // the planar class: every placement, the blank boxed by its own boundary
    if let Some(pocket) = s.pocket {
        let at = |pose: Motion| move |(row,m,w): pocket::Failure| Refused::said(Asked::Planar,(row,m,w.map(|p| pose.point(p))));
        let pocket = pocket.as_ref().map_err(|f| at(first)(f.clone()))?;
        let side = ask.sampling.coarse_rows.max(8)*2;
        let mut checked = None;
        for &pose in poses {
            let inside = move |p: V| blank.inside(pose.point(p));
            let c = pocket.check(pose,&inside,blank.boxed(),side,ask.margin).map_err(at(pose))?;
            checked.get_or_insert(c);
        }
        let checked = checked.expect("a placement");
        let (found,contacts,spacing) = pocket.found(&checked);
        return Ok(Admitted {contacts,spacing,samples:checked.samples,basis:[checked.rows,checked.columns],
            ..Admitted::whole(Class::Planar(Box::new(found)),poses.len())})
    }
    let twist = twist_of(s.path);
    let asked = if twist.is_some() { Asked::ConstantTwist } else { Asked::Generating };
    let b = s.tool.map_err(|why| Refused::said(asked,(Row::Tool,why.to_string(),None)))?;
    // the planar subtraction class: outright under a constant twist, and in place of the other rows
    // where they refuse; every placement at once
    // (read once, where it is first asked)
    let read = std::cell::OnceCell::new();
    let subtraction = || read.get_or_init(|| Subtraction::read(b,s.path,s.roll,s.mm,s.names.clone())).as_ref().ok();
    let subtract = |class: &Subtraction| -> Result<Admitted,Refused> {
        let found = class.check(poses,&|p| blank.inside(p),blank.boxed()).map_err(|f| Refused::said(Asked::Subtraction,f))?;
        let sampled = class.sampled(&found);
        Ok(Admitted {samples:sampled.candidates,contacts:sampled.runs,spacing:(1./super::region::PER_DEGREE).to_radians(),
            basis:[sampled.candidates,sampled.steps],..Admitted::whole(Class::Subtraction(Box::new(found)),poses.len())})
    };
    let closed = |refused: Refused| closed(s,b,poses.len()).ok_or(refused);
    if twist.is_some() { if let Some(class) = subtraction() { return subtract(class).or_else(closed) } }
    rowed(s,b,blank,ask,twist).or_else(|refused| match subtraction() {
        Some(class) => subtract(class).or_else(closed),
        None => closed(refused),
    })
}

/// The closed class's admission of `s` (C1, C2): its trajectory unpaced, read in millimetres, a
/// constant twist's or any other's, the tool's structure what the build asks; none where it
/// refuses.
fn closed(s: &Sweep,b: &Brep,poses: usize) -> Option<Admitted> {
    // a sweep is its path's, not its pace's (#684 L)
    let (path,roll) = s.path.unpaced(s.roll).ok()?;
    let path = path.scaled(s.mm);
    match path.screw() {
        Some(_) => super::closed::reads(b,&path).map(|_| ()),
        None => super::closed::reads_general(b,&path,roll),
    }.ok()?;
    Some(Admitted::whole(Class::Closed(Box::new(Closed {tool:b.clone(),path,roll})),poses))
}

/// The constant twist's rows (S1–S4) under `twist`, the constant twist the motion is, or the
/// generating class's (M1–E4, T1–T3) where it is none: the tool read from its B-rep (T1), the
/// motion charted (M1), the first placement asked and each later one read alike to an earlier, or
/// asked itself.
fn rowed(s: &Sweep,b: &Brep,blank: &dyn Blank,ask: &Ask,twist: Option<Screw>) -> Result<Admitted,Refused> {
    let (poses,first) = (ask.poses,ask.poses[0]);
    let asked = if twist.is_some() { Asked::ConstantTwist } else { Asked::Generating };
    let refuse = |pose: Motion| move |(row,m,w): super::constant_twist::Failure|
        Refused::said(asked,(row.into(),m,w.map(|p| pose.point(p))));
    let tool = Tool::read(b,s.mm).map_err(|why| Refused::said(asked,(Row::Tool,format!("`{}` is read by its boundary, but \
        {why}",s.names.tool),why.witness().map(|p| first.point(p)))))?;
    let lone = Lone::read(b,&tool,s.mm);
    // M1: a motion no constant twist has a contact chart, or is refused by what it is
    if twist.is_none() {
        let form = Form::of(s.path).map_err(|u| Refused {asked,row:Row::Motion,seen:Seen::Uncharted(u),witness:None})?;
        let probe = SurfacePoint {position:[1.,2.,3.],du:[1.,0.,0.],dv:[0.,1.,0.]};
        form.normal_velocity(probe).map_err(|e| Refused::said(asked,(Row::Motion,e.to_string(),None)))?;
    }
    // a whole turn or more sweeps what one whole turn does (#684 N): read once round
    let once = twist.and_then(|screw| once_round(&screw,s.roll));
    let sweep = Generating {tool:&tool,path:s.path,roll:once.unwrap_or(s.roll),whole:once.is_some(),mm:s.mm};
    let sampling = &ask.sampling;
    let check = |pose: Motion| -> Result<(Admitted,Reads),Refused> {
        let inside = move |p: V| blank.inside(pose.point(p));
        match twist {
            Some(screw) => {
                let extent = blank.extent().map(|(lo,hi)| screw.extent(lo,hi,pose.inverse()));
                let (found,spacing,reads) = rows::check_screw(&sweep,&inside,extent,sampling,lone.as_ref()).map_err(refuse(pose))?;
                let [a,b] = found.reach;
                Ok((Admitted {samples:found.samples,contacts:b-a+1,spacing,least_area_factor:found.least_factor,basis:[sampling.rows,360],
                    ..Admitted::whole(Class::ConstantTwist(Box::new(found)),0)},reads))
            }
            None => {
                let (found,reads) = rows::check(&sweep,&inside,sampling,lone.as_ref()).map_err(refuse(pose))?;
                Ok((Admitted {samples:found.samples,contacts:found.contacts,spacing:found.spacing,least_area_factor:found.least_area_factor,
                    near_double_roots:found.near_double_roots,near_tangent_pairs:found.near_tangent_pairs,
                    basis:[sampling.rows,sampling.columns],..Admitted::whole(Class::Generating(found.capped.map(Box::new)),0)},reads))
            }
        }
    };
    let (mut admitted,reads) = check(first)?;
    admitted.placements = vec![None];
    // the blank the same under every turn about the line every placement turns about: each later
    // placement reads it as the first does, by the drawing's proof
    if let Some((origin,axis)) = ask.revolved {
        admitted.placements.extend(poses.iter().skip(1).map(|_| Some(0)));
        admitted.equivalence = Equivalence::Revolved {origin,axis};
        return Ok(admitted)
    }
    // otherwise whether each later placement reads the blank as the first did, every point of it,
    // side by side; one that does not is read against the others asked, or asked itself
    let read_alike = |pose: Motion,reads: &Reads| reads.points.iter().zip(&reads.inside).all(|(q,&i)| blank.inside(pose.point(*q)) == i);
    let alike = crate::util::par::map(&poses[1..],|&pose| read_alike(pose,&reads));
    let mut asked: Vec<(usize,Reads)> = vec![(0,reads)];
    for (j,(&pose,alike)) in poses.iter().zip(std::iter::once(true).chain(alike)).enumerate().skip(1) {
        if alike { admitted.placements.push(Some(0)); continue }
        if let Some((i,_)) = asked.iter().skip(1).find(|(_,reads)| read_alike(pose,reads)) {
            admitted.placements.push(Some(*i));
            continue
        }
        let (_,reads) = check(pose)?;
        asked.push((j,reads));
        admitted.placements.push(None);
    }
    Ok(admitted)
}
