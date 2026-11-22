//! The CAD recipe as the kernel builds it: every node `solid::cad` writes, its fields typed, so a
//! reader — this kernel's build, its meridian and sections, the swept construction, the native
//! host — matches a `Node`.  A writer makes these values directly; nothing is written as text and
//! read back.  A profile is kept as written (`Section`: its numbers as they stand, an arc's angles
//! and a normal unnormalised), since the native host hands them on as they are; this kernel reads
//! it into its own `Profile` (`Profile::of`).
use crate::geom::{Surface,V};

/// A whole recipe: its nodes in the order they are built (each built from those before it) and
/// the id of the one that is the solid.
#[derive(Clone,Debug,Default)]
pub struct Recipe { pub root: i64,pub nodes: Vec<Named> }

/// A node of a recipe's list: its id, its name (the solid's, as the document spells it) and what
/// it is.  The step it adds to a face's route is each body's to say (`Body::steps`).
#[derive(Clone,Debug)]
pub struct Named { pub id: i64,pub name: String,pub node: Node,
    /// What a failure to build it calls it, where its name is none the document wrote (`"said"`,
    /// #879): a round's solid is named by the use of the word that made it, which no author
    /// reads, so whoever wrote the recipe says it by the body it rounds and its label there.
    pub said: Option<String> }

impl Named {
    /// How a failure names the node: what its writer said, else its name between backticks
    /// (nothing for a node of no name) — what a body's `built_names` hold.
    pub fn called(&self) -> String {
        match &self.said {
            Some(said) => said.clone(),
            None if self.name.is_empty() => String::new(),
            None => format!("`{}`",self.name),
        }
    }
}

/// A body a `through` prism is applied to: the nodes `sources` its material is made of, and the
/// row-major 3x4 `matrix` the copy cut from it is placed by (the profile where it is written, none).
#[derive(Clone,Debug)]
pub struct Use { pub sources: Vec<i64>,pub matrix: Option<[f64;12]> }

impl Use {
    /// Where the use puts a profile through `origin` square to `normal`: its origin and normal.
    pub fn frame(&self,origin: V,normal: V) -> (V,V) {
        match &self.matrix {
            None => (origin,normal),
            Some(m) => {
                let r = crate::geom::Rigid::from_rows(m);
                (r.point(origin),r.vector(normal))
            }
        }
    }
}

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
    /// A prism through everything spanning, along its profile's normal, the material of each of
    /// its `uses`: the bodies it is cut from or kept in common with (#737), each read from where
    /// that use's placement puts the profile.
    Through { profile: Section,uses: Vec<Use> },
    /// The body rule over nodes built before it.
    Body(Body),
    /// A ball swept (`solid::canal`, #238, #684 D): the ball of `radius` about `centre` carried
    /// along its centre's path (`Spine`); `face` names the tube's faces.
    Canal { centre: V,radius: f64,spine: Spine,face: String },
    /// A region in general position (`solid::bounded`, #238): the common of the pieces kept within,
    /// less those kept outside.
    Region(Vec<Kept>),
    /// A region within reach of a body (`solid::grown`): the node `body`, and each piece glued to
    /// it where they share a face.
    Grown { body: i64,pieces: Vec<Node> },
    /// Pieces glued to one another where they share a face, the first to the next, and the faces
    /// they leave on one plane meeting edge to edge made one (`solid::grown`: a face's pieces
    /// around a boss's foot, glued to the body as one).
    Glued(Vec<Node>),
    /// A region set's pieces (`solid::lowering`, #194).
    Lowered(Lowered),
    /// A face family's walls (`solid::family`, #243 L2).
    Family(Family),
    /// A profile carried rigidly over an interval of a constant twist (`solid::carried`, #846): the
    /// profile moved by the row-major 3x4 `start` (the pose where the interval begins), then carried
    /// by `carry` from there to where it ends — a screw, a turn (a screw of no advance) or a slide.
    Carried { profile: Section,start: [f64;12],carry: crate::geom::Carry },
    /// The convex hull of `points` (`build::hull`), a plane face for each facet, every face called
    /// `face`: a join of sets read as the hull of finitely many points (`solid::joined`).
    Hull { points: Vec<V>,face: String },
    /// **A half-space bounded by the material it is kept with** (#953): the points behind the plane
    /// through `origin` whose unit normal `keep` points into them, as the block standing on that
    /// plane over the box of the nodes `sources` (built before it) — wide and deep enough to hold
    /// all of it, so nothing but the plane reaches what it is kept in common with.  Its every face
    /// called `face`.
    Half { origin: V,keep: V,sources: Vec<i64>,face: String },
    /// **Sheets and faces sewn into the solid they enclose** (#953): each piece a planar face of
    /// a profile or a B-spline net, named, joined where their boundaries are one curve to `tol`
    /// (`sew::sewn`) — a closed shell of panels and the faces that close it.
    Sewn { pieces: Vec<SewnPiece>,tol: f64 },
    /// **A B-rep given whole** (#993 stage 2): one a file gave (`step_read`), handed through
    /// untouched — its faces in their order, each on its own surface, its tolerances as measured.
    Given(Given),
}

/// A `Given` node's B-rep (millimetres, about the origin the recipe is written about) and a digest
/// of what it was read from: shared, never copied, and its `Debug` the digest and its size, since a
/// read B-rep may be megabytes and a node's `Debug` is read where nodes are told apart.
#[derive(Clone)]
pub struct Given { pub brep: std::sync::Arc<crate::topo::Brep>,pub digest: u64 }

impl std::fmt::Debug for Given {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f,"Given {{ digest: {:016x}, faces: {} }}",self.digest,self.brep.faces.len())
    }
}

/// One piece of a `Sewn` node: a planar face of a profile, or a sheet of a B-spline net — each
/// named, every face it makes called so.
#[derive(Clone,Debug)]
pub enum SewnPiece {
    Face { profile: Section,face: String },
    Net { net: crate::nurbs::Net,face: String },
}

/// The path a ball's centre runs (`Node::Canal`): carried by a screw about the line through
/// `origin` along `axis`, turning by `angle` and stepping `advance` (a turn with no step, a slide
/// with no turn), or along a path no closed form carries (#684 D), the ball's disc square to it a
/// face family's one round wall, its stations read off the motion (`tube`).
#[derive(Clone,Debug)]
pub enum Spine { Screw { origin: V,axis: V,angle: f64,advance: f64 },Tube(Family) }

/// A face family as its walls (`brep::family`): the pieces of its interval, the six stations of
/// each, how short an edge's run at an end is to have vanished there (mm), and per loop of its
/// section, per edge in walk order, its wall.
#[derive(Clone,Debug)]
pub struct Family { pub spans: Vec<[f64;2]>,pub at: Vec<[f64;6]>,pub vanish: f64,
    pub loops: Vec<Vec<FamilyWall>>,
    /// The motion that carries the section, where the station's statements say so (#366): every
    /// row reading the running number a point carried by one motion to a roll affine in it.  Over
    /// the family's whole interval, its first rim the start.  `None`, read off the stations.
    pub carry: Option<crate::geom::Carry> }

/// A wall of a `Family`: its name, its edge's kind, whether that edge runs round alone, and its
/// numbers at every station, piece by piece (`brep::family::Run`).
#[derive(Clone,Debug)]
pub struct FamilyWall { pub name: String,pub run: crate::family::Run,pub closed: bool,pub nodes: Vec<Vec<f64>> }

/// A body: the stock, plus what is `on` it, less what `cut`s it, within what `bound`s it; then what
/// reads the faces that makes, fills (`on_late`) then cuts (`cut_late`) — empty where it has none.
#[derive(Clone,Debug,Default)]
pub struct Body { pub stock: i64,pub on: Vec<i64>,pub cut: Vec<i64>,pub bound: Vec<i64>,
    pub on_late: Vec<i64>,pub cut_late: Vec<i64>,
    /// The step each operand adds to its faces' routes through this body, by the operand's id
    /// (#760: its path as the body's scope writes it, `hub.body`); an operand it does not list
    /// adds its name whole.
    pub steps: std::collections::BTreeMap<i64,String> }

/// A piece of a `Region`: a primitive, and whether the region keeps within it (else outside it).
#[derive(Clone,Debug)]
pub struct Kept { pub node: Node,pub keep: bool }

/// A `Lowered` node: its runs and rings, the corners' balls, the rolled runs, and which pieces
/// meet at a shared section (`joins`, indices into the pieces then the corners then the rolled).
/// `stratum` is the body a rolled run is rolled against, where there is one.
#[derive(Clone,Debug)]
pub struct Lowered { pub pieces: Vec<Run>,pub corners: Vec<Corner>,pub rolled: Vec<Rolled>,pub joins: Vec<[usize;2]>,
    /// For each join, how far apart the two sections it glues stand: at a seam where two faces meet
    /// tangent to the body's reading of tangent, its pieces' caps not one to a billionth (#962);
    /// else nothing.
    pub gaps: Vec<f64>,
    pub stratum: Option<Body>,pub beyond: Vec<crate::spill::Beyond> }

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
/// into the stretch), a piece beyond taking it; or (`beside`) where the ball stands still,
/// the part of its piece past that end of its run, a run going on from there holding it
/// (`fillet::roll_beside`, #962).
#[derive(Clone,Debug)]
pub struct Rolled { pub at: V,pub radius: f64,pub flat: Option<([f64;2],V)>,pub cuts: Vec<(V,V,f64)>,
    pub kept: Vec<(Surface,bool)>,pub wedge: Vec<(V,V)>,pub beside: bool }

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
    /// A point's path under a planar motion of turns and slides, exactly (#721 F1d,
    /// `trochoid::Trochoid`): its plane's `origin`, `x_dir` and `y_dir`, each term `[re a, im a, re b,
    /// im b, ω]` of `(a + b t) e^{iωt}` there, and the stretch of `t` it runs over.
    Trochoid { origin: V,x_dir: V,y_dir: V,terms: Vec<[f64;5]>,span: [f64;2] },
}

impl Node {
    /// The node's kind, a noun for what a message says of it.
    pub fn kind(&self) -> &'static str {
        match self {
            Node::Prism {..} => "prism",Node::Revolve {..} => "revolve",
            Node::Placed {..} => "placed",Node::Through {..} => "through",Node::Body(_) => "body",
            Node::Canal {..} => "canal",Node::Region(_) => "region",Node::Grown {..} => "grown",Node::Glued(_) => "glued",
            Node::Lowered(_) => "lowered",
            Node::Family(_) => "family",Node::Carried {..} => "carried",Node::Hull {..} => "hull",
            Node::Half {..} => "half",Node::Sewn {..} => "sewn",Node::Given(_) => "given",
        }
    }
}

impl Body {
    /// Each set after the stock, in the order the body applies them, by its key.
    pub fn sets(&self) -> [(&'static str,&[i64]);5] {
        [("on",&self.on),("cut",&self.cut),("bound",&self.bound),("on_late",&self.on_late),("cut_late",&self.cut_late)]
    }
}
