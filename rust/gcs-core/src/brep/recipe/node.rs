//! The CAD recipe read once: every node `solid::cad` writes, its fields typed, so a reader — this
//! kernel's build, its meridian and sections, the swept construction, the native host — matches a
//! `Node` and never pokes the JSON. A profile is kept as written (`Section`: its numbers as they
//! stand, an arc's angles and a normal unnormalised), since the native host hands them on as they
//! are; this kernel reads it into its own `Profile` (`Profile::of`). What cannot be read is a
//! `Kind::Recipe` error naming the field.
use crate::brep::geom::{Frame,Surface,V};
use crate::brep::{vec3,KernelError,Kind};
use crate::json::Json;

type Read<T> = Result<T,KernelError>;

/// A recipe that cannot be read, saying why.
fn unread(msg: String) -> KernelError { KernelError::new(Kind::Recipe,msg) }
/// A field of a recipe node, or an error naming it.
fn field<'a>(j: &'a Json,k: &str) -> Read<&'a Json> { j.get(k).ok_or_else(|| unread(format!("recipe: no `{k}`"))) }
/// A field of a recipe node's profile, or an error naming it.
fn of_profile<'a>(j: &'a Json,k: &str) -> Read<&'a Json> { j.get(k).ok_or_else(|| unread(format!("profile: no `{k}`"))) }
/// Three numbers of a node, by key.
fn point(j: &Json,k: &str) -> Read<V> { Ok(vec3(field(j,k)?)) }
/// A number of a node, by key.
fn number(j: &Json,k: &str) -> Read<f64> { Ok(field(j,k)?.as_f64()) }
/// A list of node ids, by key.
fn ids(j: &Json,k: &str) -> Read<Vec<i64>> { Ok(field(j,k)?.arr().iter().map(Json::as_i64).collect()) }
/// A list under `k`, empty where there is none.
fn listed<'a>(j: &'a Json,k: &str) -> &'a [Json] { j.get(k).map_or(&[][..],Json::arr) }
/// Each of a list read by `f`.
fn each<T>(list: &[Json],f: impl Fn(&Json) -> Read<T>) -> Read<Vec<T>> { list.iter().map(f).collect() }
/// Two numbers.
fn pair(j: &Json) -> [f64;2] { [j.arr()[0].as_f64(),j.arr()[1].as_f64()] }

/// A whole recipe: its nodes in the order they are built (each built from those before it) and
/// the id of the one that is the solid.
#[derive(Clone,Debug)]
pub struct Recipe { pub root: i64,pub nodes: Vec<Named> }

/// A node of a recipe's list: its id, its name (the solid's, as the document spells it) and what
/// it is.
#[derive(Clone,Debug)]
pub struct Named { pub id: i64,pub name: String,pub node: Node }

/// What a node makes.
#[derive(Clone,Debug)]
pub enum Node {
    /// A profile swept along its normal from `from` to `to` (along it from its plane), its sides
    /// tapered by `draft` (radians) where there is one (#245).
    Prism { profile: Section,from: f64,to: f64,draft: Option<f64> },
    /// A profile turned by `angle` (radians) about the line through `origin` along `axis`.
    Revolve { profile: Section,origin: V,axis: V,angle: f64 },
    /// The node `source` moved by the row-major 3x4 `matrix`.
    Placed { source: i64,matrix: [f64;12] },
    /// A prism spanning the material of the nodes `sources` along its profile's normal.
    Through { profile: Section,sources: Vec<i64> },
    /// The body rule over nodes built before it.
    Body(Body),
    /// A ball swept by a turn or a screw (`solid::canal`, #238): the ball of `radius` about `centre`
    /// carried about the line through `origin` along `axis`, turning by `angle` and stepping
    /// `advance`; `face` names the tube's faces.
    Canal { centre: V,radius: f64,origin: V,axis: V,angle: f64,advance: f64,face: String },
    /// A region in general position (`solid::bounded`, #238): the common of the pieces kept within,
    /// less those kept outside.
    Region(Vec<Kept>),
    /// A region set's pieces (`solid::lowering`, #194).
    Lowered(Lowered),
    /// A face family's walls (`solid::family`, #243 L2).
    Family(Family),
}

/// A face family as its walls (`brep::family`): the pieces of its interval, the six stations of
/// each, how short an edge's run at an end is to have vanished there (mm), and per loop of its
/// section, per edge in walk order, its wall.
#[derive(Clone,Debug)]
pub struct Family { pub spans: Vec<[f64;2]>,pub at: Vec<[f64;6]>,pub vanish: f64,
    pub loops: Vec<Vec<FamilyWall>>,
    /// The motion that carries the section, where the station's statements say so (#366): every
    /// row reading the running number a point carried by one motion to a roll affine in it.  Over
    /// the family's whole interval, its first rim the start.  `None`, read off the stations.
    pub carry: Option<crate::brep::geom::Carry> }

/// A wall of a `Family`: its name, its edge's kind, whether that edge runs round alone, and its
/// numbers at every station, piece by piece (`brep::family::Run`).
#[derive(Clone,Debug)]
pub struct FamilyWall { pub name: String,pub run: crate::brep::family::Run,pub closed: bool,pub nodes: Vec<Vec<f64>> }

/// A body: the stock, plus what is `on` it, less what `cut`s it, within what `bound`s it; then what
/// reads the faces that makes, fills (`on_late`) then cuts (`cut_late`) — empty where it has none.
#[derive(Clone,Debug)]
pub struct Body { pub stock: i64,pub on: Vec<i64>,pub cut: Vec<i64>,pub bound: Vec<i64>,
    pub on_late: Vec<i64>,pub cut_late: Vec<i64> }

/// A piece of a `Region`: a primitive, and whether the region keeps within it (else outside it).
#[derive(Clone,Debug)]
pub struct Kept { pub node: Node,pub keep: bool }

/// A `Lowered` node: its runs and rings, the corners' balls, the rolled runs, and which pieces
/// meet at a shared section (`joins`, indices into the pieces then the corners then the rolled).
/// `stratum` is the body a rolled run is rolled against, where there is one.
#[derive(Clone,Debug)]
pub struct Lowered { pub pieces: Vec<Run>,pub corners: Vec<Corner>,pub rolled: Vec<Rolled>,pub joins: Vec<[usize;2]>,
    pub stratum: Option<Body>,pub beyond: Vec<crate::brep::spill::Beyond> }

/// A lowered run or ring: a prism or a revolution, cut off at a slant behind each `within` plane
/// (through its `at`, its `normal` away from the run), and kept within or outside each solid a
/// curved face it ends on bounds (`kept`: that face's surface, and whether inside).
#[derive(Clone,Debug)]
pub struct Run { pub node: Node,pub within: Vec<(V,V)>,pub kept: Vec<(Surface,bool)> }

/// A corner's ball: of `radius` about `centre`, its patch at `vertex` between three faces.
#[derive(Clone,Debug)]
pub struct Corner { pub vertex: V,pub centre: V,pub radius: f64,pub faces: Vec<Surface> }

/// A ball rolled along a traced run from `at`, of `radius` — or, where `flat` gives its legs and a
/// point it lies `on`, a chamfer's flat cut along it — cut at each section of `cuts` (a point, the
/// section's normal, its reach), and kept within or outside each of `kept` (a seam's normal surface
/// a piece beyond takes the rest past, #422: a plane's half-space, a curved face's solid); a ball
/// rolled with the stretch between the planes of `wedge` cut away (each a point and its normal
/// into the stretch), a piece beyond taking it.
#[derive(Clone,Debug)]
pub struct Rolled { pub at: V,pub radius: f64,pub flat: Option<([f64;2],V)>,pub cuts: Vec<(V,V,f64)>,
    pub kept: Vec<(Surface,bool)>,pub wedge: Vec<(V,V)> }

/// A profile as a recipe writes it (millimetres): its plane, through `origin` square to `normal`
/// (as written, not made unit), and its loops of edges in written order.
#[derive(Clone,Debug)]
pub struct Section { pub origin: V,pub normal: V,pub loops: Vec<Vec<Edge>> }

/// An edge of a section, and its name (the face a sweep makes of it), empty where unnamed.
#[derive(Clone,Debug)]
pub struct Edge { pub name: String,pub shape: Stroke }

/// An edge's curve as written: a line, a circle (an arc from the first of `angles` to the second,
/// radians about `normal` from `x_dir`, where it has them) or a B-spline (rational where weighed).
#[derive(Clone,Debug)]
pub enum Stroke {
    Line { start: V,end: V },
    Circle { center: V,normal: V,x_dir: V,radius: f64,angles: Option<[f64;2]> },
    BSpline { degree: usize,knots: Vec<f64>,poles: Vec<V>,weights: Option<Vec<f64>> },
}

impl Recipe {
    /// A recipe's JSON read, or an error naming the first field it lacks.
    pub fn read(j: &Json) -> Read<Recipe> {
        let nodes = each(field(j,"nodes")?.arr(),|n| Ok(Named {id:field(n,"id")?.as_i64(),
            name:n.get("name").map_or("",Json::as_str).to_string(),node:Node::read(n)?}))?;
        Ok(Recipe {root:field(j,"root")?.as_i64(),nodes})
    }
}

impl Node {
    /// A node's JSON read: its `kind` and that kind's fields.
    pub fn read(n: &Json) -> Read<Node> {
        let profile = || Section::read(field(n,"profile")?);
        Ok(match field(n,"kind")?.as_str() {
            "prism" => Node::Prism {profile:profile()?,from:number(n,"from")?,to:number(n,"to")?,
                draft:n.get("draft").map(Json::as_f64)},
            "revolve" => Node::Revolve {profile:profile()?,origin:point(n,"origin")?,axis:point(n,"axis")?,
                angle:number(n,"angle")?},
            "placed" => {
                let m = field(n,"matrix")?.arr();
                if m.len() != 12 { return Err(unread("recipe: a placement's `matrix` is not three rows of four".into())) }
                Node::Placed {source:field(n,"source")?.as_i64(),matrix:std::array::from_fn(|k| m[k].as_f64())}
            }
            "through" => Node::Through {profile:profile()?,sources:ids(n,"sources")?},
            "body" => Node::Body(Body::read(n)?),
            "canal" => Node::Canal {centre:point(n,"centre")?,radius:number(n,"radius")?,origin:point(n,"origin")?,
                axis:point(n,"axis")?,angle:number(n,"angle")?,advance:number(n,"advance")?,
                face:field(n,"face")?.as_str().to_string()},
            "region" => Node::Region(each(field(n,"pieces")?.arr(),|p| Ok(Kept {node:Node::read(p)?,
                keep:matches!(field(p,"keep")?,Json::Bool(true))}))?),
            "lowered" => Node::Lowered(Lowered::read(n)?),
            "family" => Node::Family(Family::read(n)?),
            k => return Err(unread(format!("recipe: a node of kind `{k}`"))),
        })
    }

    /// The node's kind, as the recipe spells it.
    pub fn kind(&self) -> &'static str {
        match self {
            Node::Prism {..} => "prism",Node::Revolve {..} => "revolve",
            Node::Placed {..} => "placed",Node::Through {..} => "through",Node::Body(_) => "body",
            Node::Canal {..} => "canal",Node::Region(_) => "region",Node::Lowered(_) => "lowered",
            Node::Family(_) => "family",
        }
    }
}

impl Body {
    /// A body's sets read off `n` (a body node, or the stratum a lowered node carries): the late
    /// ones only where it has them.
    pub fn read(n: &Json) -> Read<Body> {
        let late = |k: &str| listed(n,k).iter().map(Json::as_i64).collect();
        Ok(Body {stock:field(n,"stock")?.as_i64(),on:ids(n,"on")?,cut:ids(n,"cut")?,bound:ids(n,"bound")?,
            on_late:late("on_late"),cut_late:late("cut_late")})
    }
    /// Each set after the stock, in the order the body applies them, by its key.
    pub fn sets(&self) -> [(&'static str,&[i64]);5] {
        [("on",&self.on),("cut",&self.cut),("bound",&self.bound),("on_late",&self.on_late),("cut_late",&self.cut_late)]
    }
}

impl Family {
    fn read(n: &Json) -> Read<Family> {
        use crate::brep::family::Run;
        let numbers = |j: &Json| -> Vec<f64> { j.arr().iter().map(Json::as_f64).collect() };
        let at = each(field(n,"at")?.arr(),|a| {
            let a = numbers(a);
            if a.len() != 6 { return Err(unread("recipe: a family's piece has not six stations".into())) }
            Ok(std::array::from_fn(|k| a[k]))
        })?;
        let wall = |w: &Json| -> Read<FamilyWall> {
            let run = match field(w,"run")?.as_str() {
                "line" => Run::Line,
                "round" => Run::Round,
                "spline" => Run::Spline {degree:number(w,"degree")? as usize,knots:numbers(field(w,"knots")?),
                    weights:w.get("weights").map(numbers)},
                k => return Err(unread(format!("recipe: a family's wall of kind `{k}`"))),
            };
            Ok(FamilyWall {name:w.get("name").map_or("",Json::as_str).to_string(),run,
                closed:matches!(field(w,"closed")?,Json::Bool(true)),nodes:field(w,"nodes")?.arr().iter().map(numbers).collect()})
        };
        let carry = match n.get("carry") {
            None => None,
            Some(c) => Some(match field(c,"kind")?.as_str() {
                "screw" => crate::brep::geom::Carry::Screw {center:point(c,"center")?,
                    axis:point(c,"axis")?,angle:number(c,"angle")?,advance:number(c,"advance")?},
                "line" => crate::brep::geom::Carry::Line {delta:point(c,"delta")?},
                k => return Err(unread(format!("recipe: a family carried by a `{k}`"))),
            }),
        };
        Ok(Family {spans:field(n,"spans")?.arr().iter().map(pair).collect(),at,vanish:number(n,"vanish")?,
            loops:each(field(n,"loops")?.arr(),|l| each(l.arr(),wall))?,carry})
    }
}

impl Lowered {
    fn read(n: &Json) -> Read<Lowered> {
        let pieces = each(field(n,"pieces")?.arr(),|p| Ok(Run {node:Node::read(p)?,
            within:each(listed(p,"within"),|h| Ok((point(h,"at")?,point(h,"normal")?)))?,
            kept:each(listed(p,"kept"),|k| Ok((surface(k)?,matches!(field(k,"inside")?,Json::Bool(true)))))?}))?;
        let corners = each(field(n,"corners")?.arr(),|c| Ok(Corner {vertex:point(c,"vertex")?,centre:point(c,"centre")?,
            radius:number(c,"radius")?,faces:each(field(c,"faces")?.arr(),surface)?}))?;
        let rolled = each(listed(n,"rolled"),|k| Ok(Rolled {at:point(k,"at")?,radius:number(k,"radius")?,
            flat:match k.get("legs") { Some(legs) => Some((pair(legs),point(k,"on")?)),None => None },
            cuts:each(listed(k,"cuts"),|c| Ok((point(c,"at")?,point(c,"normal")?,number(c,"reach")?)))?,
            kept:each(listed(k,"kept"),|j| Ok((surface(j)?,matches!(field(j,"inside")?,Json::Bool(true)))))?,
            wedge:each(listed(k,"wedge"),|j| Ok((point(j,"at")?,point(j,"normal")?)))?}))?;
        let joins = field(n,"joins")?.arr().iter()
            .map(|j| { let j = j.arr(); [j[0].as_i64() as usize,j[1].as_i64() as usize] }).collect();
        // a ring's section beyond a seam's normal surface (#422), counted after the rolled runs
        let beyond = each(listed(n,"beyond"),|b| {
            use crate::brep::spill::{Across,Seam};
            let seam = field(b,"seam")?;
            let ends = field(b,"ends")?.arr();
            let sides = field(b,"sides")?.arr();
            if ends.len() != 2 || sides.len() != 2 { return Err(unread("recipe: a piece beyond a seam, misread".into())) }
            let face = surface(field(field(b,"along")?,"face")?)?;
            let seam = match seam.get("point") {
                Some(_) => Seam::Line(point(seam,"point")?,point(seam,"direction")?),
                None => Seam::Circle(point(seam,"centre")?,point(seam,"normal")?,number(seam,"radius")?),
            };
            // a flat's legs and the surface it starts at, or a round's radius
            let section = match b.get("radius") {
                Some(_) => Across::Round {radius:number(b,"radius")?},
                None => Across::Flat {legs:pair(field(b,"legs")?),split:surface(field(b,"split")?)?},
            };
            Ok(crate::brep::spill::Beyond {face,toward:point(b,"toward")?,chain:surface(field(b,"chain")?)?,
                wall:surface(field(b,"wall")?)?,sides:[sides[0].as_f64(),sides[1].as_f64()],seam,section,
                ends:[vec3(&ends[0]),vec3(&ends[1])]})
        })?;
        Ok(Lowered {pieces,corners,rolled,joins,stratum:n.get("stratum").map(Body::read).transpose()?,beyond})
    }
}

/// A lowered piece's end face or a corner's face, as `solid::lowering` writes it (millimetres): a
/// plane through `origin` square to `normal`, a cylinder of `radius` about the line through `origin`
/// along `axis`, a sphere of `radius` about `origin`, or a cone about the line through `origin`
/// along `axis` of `radius` there, its `half` angle (radians).
fn surface(j: &Json) -> Read<Surface> {
    Ok(match field(j,"kind")?.as_str() {
        "plane" => Surface::Plane(Frame::about(point(j,"origin")?,point(j,"normal")?)),
        "cylinder" => Surface::Cylinder(Frame::about(point(j,"origin")?,point(j,"axis")?),number(j,"radius")?),
        "cone" => Surface::Cone(Frame::about(point(j,"origin")?,point(j,"axis")?),number(j,"radius")?,number(j,"half")?),
        "sphere" => Surface::Sphere(Frame::about(point(j,"origin")?,[0.,0.,1.]),number(j,"radius")?),
        "torus" => Surface::Torus(Frame::about(point(j,"origin")?,point(j,"axis")?),number(j,"big")?,number(j,"tube")?),
        k => return Err(unread(format!("recipe: a face on a surface of kind `{k}`"))),
    })
}

impl Section {
    /// A recipe's `profile` read.
    pub fn read(j: &Json) -> Read<Section> {
        let at = |e: &Json,k: &str| -> Read<V> { Ok(vec3(of_profile(e,k)?)) };
        let edge = |e: &Json| -> Read<Edge> {
            let shape = match of_profile(e,"kind")?.as_str() {
                "line" => Stroke::Line {start:at(e,"start")?,end:at(e,"end")?},
                "circle" => Stroke::Circle {center:at(e,"center")?,normal:at(e,"normal")?,x_dir:at(e,"x_dir")?,
                    radius:of_profile(e,"radius")?.as_f64(),angles:e.get("angles").map(pair)},
                "bspline" => Stroke::BSpline {degree:of_profile(e,"degree")?.as_f64() as usize,
                    knots:of_profile(e,"knots")?.arr().iter().map(Json::as_f64).collect(),
                    poles:of_profile(e,"poles")?.arr().iter().map(vec3).collect(),
                    weights:e.get("weights").map(|w| w.arr().iter().map(Json::as_f64).collect())},
                k => return Err(unread(format!("profile: an edge of kind `{k}`"))),
            };
            Ok(Edge {name:e.get("name").map_or(String::new(),|n| n.as_str().to_string()),shape})
        };
        let loops = each(of_profile(j,"loops")?.arr(),|l| each(l.arr(),edge))?;
        Ok(Section {origin:at(j,"origin")?,normal:at(j,"normal")?,loops})
    }
}
