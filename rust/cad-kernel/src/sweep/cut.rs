//! **A solid sweep without a document**: a tool (`Tool`: revolutions and prisms of lines and arcs,
//! their unions and intersections, placed), a rigid motion (`Motion`: a slide, a turn, a screw, or
//! one motion seen from another), a roll and a blank `Brep` — and the blank with everything the
//! tool passes through cut away, as exact B-rep, or a refusal naming the row of the class that
//! failed and a point where (`refusal::ExportRefusal`).
//!
//! It is the solver's pipeline, read from plain data: the tool's faces and contacts
//! (`SweepContacts::of_tool`), its material in closed form (`Tool::material`) and the blank's off
//! its boundary (`material::Blank`); the class's rows (`admit`); the sheet — the characteristic's
//! section carried by the motion exactly, under a screw, a turn or a slide
//! (`helical::carried_sheet`), or the contacts traced, under a relative rotation (`sheet`); and the
//! blank split by it, each cell judged by whether the tool passes through it (`cells`). A union is
//! swept member by member, as its sweep is theirs.
//!
//! Lengths are millimetres, as the bars and tolerances of the sheet fitter are. The roll is in
//! radians of the motion's parameter: a turn of ratio 1 turns a radian a radian of roll, a slide
//! moves `advance` a whole turn (`2π`) of roll.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::Say;
use super::admit::{self,Found,Lone,Options};
use super::cells::{self,Ball};
use super::condition::Condition;
use super::cutter::Cutter;
use super::envelope::{Motion as Pose,SurfacePoint};
use super::material::Blank;
use super::motion::{Family,Step};
use super::refusal::{AtStage,ExportRefusal,Stage,Tolerance};
use super::sheet::{Fitted,SweptCut};
use super::tool::{Material,SweepContacts};
use crate::topo::Brep;
use crate::util::json::{object,Json};

pub use super::tool::Tool;

type V = [f64;3];

/// **A rigid motion a sweep is read under**, by the roll `t` (radians): a slide along `direction`
/// (made unit) by `advance` a whole turn of the roll; a turn about the line through `origin` along
/// `axis`, `ratio` radians a radian (right-handed about `axis`); a screw, that turn advancing
/// `advance` along `axis` a whole turn of the roll; or `motion` as seen from a frame moving by
/// `observer` (`observer⁻¹ · motion`): a turn seen from a turn, a slide seen from a turn (a rack).
#[derive(Clone,Debug)]
pub enum Motion {
    Slide { direction: V,advance: f64 },
    Turn { origin: V,axis: V,ratio: f64 },
    Screw { origin: V,axis: V,ratio: f64,advance: f64 },
    SeenFrom { motion: Box<Motion>,observer: Box<Motion> },
}

impl Motion {
    /// `self` seen from a frame moving by `observer`.
    pub fn seen_from(self,observer: Motion) -> Motion { Motion::SeenFrom {motion:Box::new(self),observer:Box::new(observer)} }

    /// The steps of a motion family, appended to `steps`: the index of the one that is this motion.
    fn lower(&self,steps: &mut Vec<Step>) -> usize {
        let step = match self {
            Motion::Slide {direction,advance} =>
                Step::Translation {axis:crate::util::space::unit(*direction),advance:*advance,law:None},
            Motion::Turn {origin,axis,ratio} => Step::Rotation {origin:*origin,axis:*axis,ratio:*ratio,advance:0.,law:None},
            Motion::Screw {origin,axis,ratio,advance} => Step::Rotation {origin:*origin,axis:*axis,ratio:*ratio,advance:*advance,law:None},
            Motion::SeenFrom {motion,observer} => {
                let source = motion.lower(steps);
                let observer = observer.lower(steps);
                Step::Relative {source,observer}
            }
        };
        steps.push(step);
        steps.len()-1
    }

    /// The motion as a family of rigid motions (`motion::Family`), named `name`.
    pub fn family(&self,name: &str) -> Result<Family,String> {
        let mut steps = Vec::new();
        self.lower(&mut steps);
        Family::new(name.to_string(),steps)
    }
}

/// How a swept cut is asked: the name its refusals and its faces say (`sweep` by default), the
/// tolerance the sheet is held to (millimetres; `None` the fabrication tolerance, 0.01 mm), and
/// the class's sampling and bars (`admit::Options`, its `margin` how far inside the blank's
/// boundary a point must be to count as in it).
///
/// `carried`: a screw's, a turn's or a slide's sheet is its section carried by the motion exactly
/// (`helical::carried_sheet`: an extrusion, a revolution, a screwed curve), only its section
/// fitted, where the split takes it; the grid the solver's export fits (`helical_sheet`) where not,
/// or where `carried` is off. By default it is on.
#[derive(Clone,Debug)]
pub struct Ask { pub name: String,pub tolerance: Option<f64>,pub options: Options,pub carried: bool }

impl Default for Ask {
    fn default() -> Ask { Ask {name:"sweep".into(),tolerance:None,options:Options::default(),carried:true} }
}

/// The blank `blank` with everything `tool` passes through under `motion` over `roll` cut away,
/// its sheet held to `tolerance` (millimetres; `None` the fabrication tolerance, 0.01 mm); or a
/// refusal saying the stage, and at admission the row of the class and a point where.
pub fn swept_cut(blank: &Brep,tool: &Tool,motion: &Motion,roll: [f64;2],tolerance: Option<f64>) -> Result<Brep,ExportRefusal> {
    let quiet = Say {stage:&|_| {},mark:&|_| {}};
    swept_cut_with(blank,tool,motion,roll,&Ask {tolerance,..Ask::default()},&quiet)
}

/// A tool's members, each swept alone: a union's (through its placements), as a union's sweep is
/// its members'; the tool itself otherwise. Refused (T1) where a union stands within an
/// intersection, or the tool is faces a reader made.
fn members(tool: &Tool) -> Result<Vec<Tool>,String> {
    fn within(tool: &Tool) -> Result<(),String> {
        match tool {
            Tool::Union(_) => Err("it adds solids together within an intersection; a union is in the class only as the whole \
                tool, each member swept alone".into()),
            Tool::Intersection(tools) => tools.iter().try_for_each(within),
            Tool::Placed(_,tool) => within(tool),
            Tool::Read(_) => Err("a tool of faces a reader made is swept by that reader".into()),
            Tool::Revolved {..} | Tool::Prism {..} => Ok(()),
        }
    }
    match tool {
        Tool::Union(tools) => Ok(tools.iter().map(members).collect::<Result<Vec<_>,_>>()?.into_iter().flatten().collect()),
        Tool::Placed(m,tool) => Ok(members(tool)?.into_iter().map(|t| Tool::Placed(*m,Box::new(t))).collect()),
        _ => { within(tool)?; Ok(vec![tool.clone()]) }
    }
}

/// The name a member's faces and refusals carry: its primitive's, through its placements.
fn member_name(tool: &Tool) -> String {
    match tool {
        Tool::Revolved {name,..} | Tool::Prism {name,..} => name.clone(),
        Tool::Placed(_,tool) => member_name(tool),
        Tool::Intersection(tools) => tools.iter().map(member_name).collect::<Vec<_>>().join(" ∩ "),
        Tool::Union(_) | Tool::Read(_) => String::new(),
    }
}

impl Tool {
    /// The tool as a CAD recipe (`recipe`, millimetres), as `recipe::build` builds it and a
    /// cutter's sections read it: each primitive a node, each Boolean a body, each placement a
    /// placed node.
    pub fn recipe(&self) -> Result<Json,String> {
        fn write(tool: &Tool,nodes: &mut Vec<Json>) -> Result<i64,String> {
            let w = crate::recipe::Writer::world(1.);
            let triple = |v: V| crate::recipe::Writer::triple(v);
            let ids = |v: &[i64]| Json::Arr(v.iter().map(|&i| Json::from(i)).collect());
            let mut node = match tool {
                Tool::Revolved {name,profile,origin,axis} => object([("name",name.as_str().into()),("kind","revolve".into()),
                    ("profile",profile.to_json(&w)?),("origin",triple(*origin)),("axis",triple(*axis)),("angle",std::f64::consts::TAU.into())]),
                Tool::Prism {name,profile,from,to,draft} => {
                    let mut o = object([("name",name.as_str().into()),("kind","prism".into()),("profile",profile.to_json(&w)?),
                        ("from",(*from).into()),("to",(*to).into())]);
                    if let Some(d) = draft.filter(|d| *d != 0.) { o.set("draft",d.into()); }
                    o
                }
                Tool::Union(tools) | Tool::Intersection(tools) => {
                    let parts = tools.iter().map(|t| write(t,nodes)).collect::<Result<Vec<_>,_>>()?;
                    let [stock,rest @ ..] = &parts[..] else { return Err("a Boolean of no tools".into()) };
                    let (on,bound) = if matches!(tool,Tool::Union(_)) { (rest,&[][..]) } else { (&[][..],rest) };
                    object([("kind","body".into()),("stock",(*stock).into()),("on",ids(on)),("cut",ids(&[])),("bound",ids(bound))])
                }
                Tool::Placed(m,tool) => {
                    let source = write(tool,nodes)?;
                    let o = m.point([0.;3]);
                    let column = |k: usize| { let mut e = [0.;3]; e[k] = 1.; m.vector(e) };
                    let c = [column(0),column(1),column(2)];
                    let matrix: Vec<Json> = (0..12).map(|i| { let (r,k) = (i/4,i%4); Json::from(if k == 3 { o[r] } else { c[k][r] }) }).collect();
                    object([("kind","placed".into()),("source",source.into()),("matrix",Json::Arr(matrix))])
                }
                Tool::Read(_) => return Err("a tool of faces a reader made has no recipe of its own".into()),
            };
            let id = nodes.len() as i64;
            node.set("id",id.into());
            nodes.push(node);
            Ok(id)
        }
        let mut nodes = Vec::new();
        let root = write(self,&mut nodes)?;
        Ok(object([("root",root.into()),("nodes",Json::Arr(nodes))]))
    }

    /// The tool as this kernel builds it (`recipe::build`).
    pub fn brep(&self) -> Result<Brep,String> { crate::recipe::build(&self.recipe()?) }
}

/// A stretch of the roll waiting to be read, by the material at its middle: the least first.
struct Stretch { value: f64,at: [f64;2] }

impl PartialEq for Stretch { fn eq(&self,other: &Self) -> bool { self.cmp(other) == std::cmp::Ordering::Equal } }
impl Eq for Stretch {}
impl PartialOrd for Stretch { fn partial_cmp(&self,other: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(other)) } }
impl Ord for Stretch {
    fn cmp(&self,other: &Self) -> std::cmp::Ordering {
        other.value.total_cmp(&self.value).then_with(|| other.at[0].total_cmp(&self.at[0]))
    }
}

/// Whether the ball of radius `r` about `p` is swept by `tool` under `family` over `roll` — some
/// time carries all of it into the tool — or wholly clear of it at every time: the tool's material
/// (never more than the distance) read at `p` carried back to the tool's frame, the roll halved
/// where the bound on how fast a point carried back moves (`Family::inverse_point_speed_bound`)
/// leaves it undecided, the stretch reading least first. A stretch halved to nothing undecided is
/// left for the rest: one time that sweeps the ball says it is swept; `Other` where none does and
/// one was left, or the halving runs out.
fn swept_ball(tool: &dyn Material,family: &Family,roll: [f64;2],p: V,r: f64) -> Result<Ball,String> {
    let read = |[a,b]: [f64;2]| -> Result<Stretch,String> {
        Ok(Stretch {value:tool.value(family.at(0.5*(a+b))?.inverse().point(p)),at:[a,b]})
    };
    let mut waiting = std::collections::BinaryHeap::from([read(roll)?]);
    let (mut asked,mut left) = (0,None);
    while let Some(Stretch {value:d,at:[a,b]}) = waiting.pop() {
        if d < -r { return Ok(Ball::Removed) }
        let speed = crate::util::interval::Interval::new(a,b).and_then(|i| family.inverse_point_speed_bound(p,i))
            .map_err(|e| format!("{e:?}"))?;
        if d-speed*(b-a)/2. > r { continue }
        asked += 1;
        if asked > 20_000 { return Ok(Ball::Other(format!("undecided after {asked} stretches of the roll"))) }
        let m = 0.5*(a+b);
        if b-a <= 1e-9*(roll[1]-roll[0]).abs().max(1.) { left.get_or_insert((m,d)); continue }
        waiting.push(read([a,m])?); waiting.push(read([m,b])?);
    }
    Ok(match left {
        None => Ball::Material,
        Some((m,d)) => Ball::Other(format!("undecided: the tool passes {d:.2e} mm from it at roll {m:.6}, and never wholly over it")),
    })
}

/// `swept_cut`, asked by `ask`, saying its progress to `say`.
pub fn swept_cut_with(blank: &Brep,tool: &Tool,motion: &Motion,roll: [f64;2],ask: &Ask,say: &Say) -> Result<Brep,ExportRefusal> {
    build(blank,tool,motion,roll,ask,say,true)
}

/// **The solid a tool passes through**, not cut from anything: what `tool` sweeps under `motion`
/// over `roll`, held to `tolerance` (millimetres, as `swept_cut`'s). Built as what the sweep takes
/// from a box holding all of it, so asked the same rows: the classes build a sweep whose tool is
/// clear of its blank at both ends of the roll (E1), and the tool at either end lies within the
/// solid it sweeps — so only a turn carried round a whole turn or more, which has no ends (#684 N
/// in the solver), is built; any other is refused by that row, at a point of the tool at its start.
pub fn swept_solid(tool: &Tool,motion: &Motion,roll: [f64;2],tolerance: Option<f64>) -> Result<Brep,ExportRefusal> {
    let quiet = Say {stage:&|_| {},mark:&|_| {}};
    swept_solid_with(tool,motion,roll,&Ask {tolerance,..Ask::default()},&quiet)
}

/// `swept_solid`, asked by `ask`, saying its progress to `say`.
pub fn swept_solid_with(tool: &Tool,motion: &Motion,roll: [f64;2],ask: &Ask,say: &Say) -> Result<Brep,ExportRefusal> {
    let family = motion.family(&ask.name).at(Stage::Admission)?;
    let built = tool.brep().at(Stage::Blank)?;
    // E1: a sweep with ends keeps the tool at them, within what it sweeps
    let screw = family.twist();
    if screw.and_then(|s| super::characteristic::once_round(&s,roll)).is_none() {
        let at = built.vertices.first().map(|v| v.p).unwrap_or([0.;3]);
        let witness = family.at(roll[0]).map(|m| m.point(at)).ok();
        let condition = Condition::Clearance;
        let mut said = format!("`{}` is outside the {} ({}: {}): a solid swept with ends holds the tool at each end, so no blank \
            holding it is clear of the tool there; only a turn carried round a whole turn or more, which has no ends, is built",
            ask.name,if screw.is_some() { "constant-twist class" } else { "generating-sweep class" },condition.code(),condition.rule());
        if let Some(p) = witness { said += &format!(" at ({:.4}, {:.4}, {:.4})",p[0],p[1],p[2]); }
        return Err(ExportRefusal {stage:Stage::Admission,condition:Some(condition),witness,message:said})
    }
    // the tool's box carried over the roll a step at a time, grown by how far a step carries a
    // corner at most and by a tenth of its size
    let (lo,hi) = built.bounds();
    const STEPS: usize = 256;
    let (mut a,mut b) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    let mut reach: f64 = 0.;
    for k in 0..=STEPS {
        let t = roll[0]+(roll[1]-roll[0])*k as f64/STEPS as f64;
        let pose = family.at(t).at(Stage::Blank)?;
        for c in 0..8 {
            let corner: V = std::array::from_fn(|i| if c>>i & 1 == 0 { lo[i] } else { hi[i] });
            let p = pose.point(corner);
            for i in 0..3 { a[i] = a[i].min(p[i]); b[i] = b[i].max(p[i]); }
            reach = reach.max(crate::util::space::norm(pose.velocity(corner)));
        }
    }
    let grow = reach*(roll[1]-roll[0])/STEPS as f64+0.1*crate::util::space::distance(lo,hi);
    let (a,b) = (a.map(|x| x-grow),b.map(|x| x+grow));
    let corners = [[a[0],a[1],a[2]],[b[0],a[1],a[2]],[b[0],b[1],a[2]],[a[0],b[1],a[2]]];
    let floor = crate::build::Profile {origin:a,normal:[0.,0.,1.],names:Vec::new(),
        loops:vec![(0..4).map(|k| crate::build::ProfileEdge::Line {a:corners[k],b:corners[(k+1)%4]}).collect()]};
    let blank = crate::build::prism(&floor,0.,b[2]-a[2]).at(Stage::Blank)?;
    build(&blank,tool,motion,roll,ask,say,false)
}

/// A swept cut's pipeline, keeping what is left of the blank (`material`) or what the sweep took.
fn build(blank: &Brep,tool: &Tool,motion: &Motion,roll: [f64;2],ask: &Ask,say: &Say,material: bool) -> Result<Brep,ExportRefusal> {
    let name = &ask.name;
    let tolerance = Some(ask.tolerance.map_or(Ok(Tolerance::FABRICATION),Tolerance::new).at(Stage::Admission)?);
    let refused = |condition: Condition,class: &str,message: String,witness: Option<V>| {
        let mut said = format!("`{name}` is outside the {class} ({}: {}): {message}",condition.code(),condition.rule());
        if let Some(p) = witness { said += &format!(" at ({:.4}, {:.4}, {:.4})",p[0],p[1],p[2]); }
        ExportRefusal {stage:Stage::Admission,condition:Some(condition),witness,message:said}
    };
    let family = motion.family(name).map_err(|m| refused(Condition::Motion,"generating-sweep class",m,None))?;
    let twisted = family.twist().is_some();
    let class = if twisted { "constant-twist class" } else { "generating-sweep class" };
    if !(roll[0].is_finite() && roll[1].is_finite() && roll[0] < roll[1]) {
        return Err(ExportRefusal::at(Stage::Admission,format!("`{name}`: a roll runs from one number to a greater one, not {roll:?}")))
    }
    let members = members(tool).map_err(|m| refused(Condition::Tool,class,m,None))?;
    let distributed = members.len() > 1;
    let said = |k: usize,message: String| if distributed { format!("its tool's member `{}`: {message}",member_name(&members[k])) } else { message };
    let started = crate::util::clock::Instant::now();
    let read = Blank::new(blank,ask.options.margin).at(Stage::Blank)?;
    let (lo,hi) = read.bounds();
    (say.stage)(&format!("`{name}`: the blank read off its boundary: {} faces, its distance off a mesh within {:.1e} mm ({:?})",
        blank.faces.len(),read.sag(),started.elapsed()));
    (say.mark)(Stage::Blank);
    let inside = |p: V| read.inside(p);
    // each member's contacts and class, its first failing row refused in the members' order
    let mut cuts: Vec<(SweptCut,Found)> = Vec::new();
    for (k,member) in members.iter().enumerate() {
        let material = member.material().map_err(|m| refused(Condition::Tool,class,said(k,m),None))?;
        let contacts = SweepContacts::of_tool(member,material,family.clone(),roll).map_err(|m| refused(Condition::Tool,class,said(k,m),None))?;
        if !twisted {
            let probe = SurfacePoint {position:[1.,2.,3.],du:[1.,0.,0.],dv:[0.,1.,0.]};
            contacts.motion().normal_velocity(probe).map_err(|m| refused(Condition::Motion,class,said(k,m),None))?;
        }
        let recipe = member.recipe().map_err(|m| refused(Condition::Tool,class,said(k,m),None))?;
        let lone = match member {
            Tool::Prism {profile,draft,..} if profile.loops.len() == 1 && draft.is_none_or(|d| d == 0.) =>
                Cutter::read(&recipe).ok().and_then(|c| Lone::new(c,1.)),
            _ => None,
        };
        let extent = family.twist().map(|screw| screw.extent(lo,hi,Pose::identity()));
        let (evidence,_) = admit::admit(&contacts,&inside,extent,&ask.options,lone.as_ref())
            .map_err(|(c,m,w)| refused(c,class,said(k,m),w))?;
        (say.stage)(&format!("`{name}`: {}admitted to the {class}: {} contacts of {} samples in the blank",
            if distributed { format!("member `{}` ",member_name(member)) } else { String::new() },evidence.contacts,evidence.samples));
        let cut = SweptCut {name:name.clone(),scale:1.,recipe,family:family.clone(),limits:roll,contacts:Ok(contacts),routes:Default::default()};
        cuts.push((cut,evidence.class));
    }
    (say.mark)(Stage::Admission);
    // each member's sheet, side by side: a constant twist's carried exactly where asked
    let inside_all = |points: &[V]| -> Result<Vec<bool>,String> { Ok(points.iter().map(|&p| read.inside(p)).collect()) };
    let near = |p: V| read.distance(p);
    let sheets = |carried: bool| -> Result<(Vec<Fitted>,bool),ExportRefusal> {
        let built: Vec<(Fitted,bool)> = crate::util::par::indices(cuts.len(),|k| {
            let (cut,class) = &cuts[k];
            match class {
                Found::Generating => super::sheet::swept_sheet(cut,&inside_all,&near,tolerance,say).map(|f| (f,false)),
                Found::Capped(capped) => super::sheet::capped_sheet(cut,capped,&inside_all,&near,tolerance,say).map(|f| (f,false)),
                Found::ConstantTwist(found) => {
                    let heights = found.screw.extent(lo,hi,Pose::identity()).0;
                    let exact = if carried { super::helical::carried_sheet(cut,found,Some(heights),&inside_all,say) }
                        else { Err(ExportRefusal::at(Stage::Fit,"not asked")) };
                    match exact {
                        Ok(fitted) => Ok((fitted,true)),
                        Err(why) => {
                            if carried { (say.stage)(&format!("`{name}`: the sheet not carried exactly ({why}); fitted")); }
                            super::helical::helical_sheet(cut,found,Some(heights),&inside_all,&near,tolerance,say).map(|f| (f,false))
                        }
                    }
                }
            }
        }).into_iter().collect::<Result<_,_>>()?;
        let exact = built.iter().any(|(_,e)| *e);
        Ok((built.into_iter().map(|(f,_)| f).collect(),exact))
    };
    let whole = tool.material().at(Stage::Classify)?;
    let probe = |asked: &[(V,f64)]| crate::util::par::map(asked,|&(p,r)| swept_ball(&*whole,&family,roll,p,r));
    let placements = vec![1;cuts.len()];
    let cut = |sheets: &[Fitted]| -> Result<Brep,String> {
        // the blank split by every sheet, each cell judged by whether the tool passes through it
        let tools: Vec<(Brep,crate::geom::Box3)> = sheets.iter().enumerate().map(|(k,s)| {
            let key = if distributed { format!("{name}.{k}") } else { name.clone() };
            let faces = cells::keyed(s.faces.clone(),&key);
            let bounds = faces.bounds();
            (faces,bounds)
        }).collect();
        let tools = cells::reaching(blank,tools);
        let turned = cells::turned_away(blank,&tools)?;
        if material { cells::whole(&turned,tools,&placements,&probe,say) } else { cells::removed(&turned,tools,&placements,&probe,say) }
    };
    let (built,exact) = sheets(ask.carried)?;
    match cut(&built) {
        Ok(b) => Ok(b),
        Err(why) if exact => {
            (say.stage)(&format!("`{name}`: the blank not split by the sheets carried exactly ({why}); fitting them"));
            cut(&sheets(false)?.0).at(Stage::Split)
        }
        Err(why) => Err(ExportRefusal::at(Stage::Split,why)),
    }
}
