//! **The swept solid, closed** (#721 X6, docs/swept-solid-design.md): `sweep(S, M)` built as one
//! valid B-rep, independent of any blank, by the design's pipeline. The candidate faces of the
//! boundary are found, a box holding the sweep is split by them, each cell is classified by
//! membership (`member::Swept::member`, the definition read without the drawing), and the kept
//! cells are united.
//!
//! **Built here: a polyhedral tool** (every face a plane, every edge a line) **under a slide or a
//! turn**, however its edges lean to the axis, **and under a screw where its edges run along the
//! axis or square to it** (and so its faces); **and under a slide a tool whose faces are planes,
//! cylinders, cones, balls and tori (a whole ring at any lean) and whose edges are lines and
//! circles**, **and under a turn one
//! whose round faces stand about axes along the turn's, or are balls**. Under a turn, every line
//! turns into a surface of revolution held exactly over the heights its source spans (heights along
//! the axis are kept by the turn): a cylinder (a line along the axis), a plane square to the axis
//! (a line square to it), a hyperboloid (a line passing the axis by, its meridian a rational
//! quadratic) or a cone (a line meeting it). The candidates:
//! - under a slide `D`: each face at the start pose where it enters the slide (`n·D < 0`), at the
//!   end pose where it leaves it (`n·D > 0`), a face along the slide (`n·D = 0`) at either (one
//!   plane); each silhouette edge (its faces' `n·D` of both signs) swept, in the plane through it
//!   holding `D`; a cylinder's or a cone's face at both end poses, its own surface over the stretch
//!   it spans (the solid cylinder or frustum on it), and each of its rulings whose normal is square
//!   to `D` (two, one or none: on a cone `A cos u + B sin u = C`) swept into the plane tangent
//!   along it — or, a cylinder along the slide, the face swept along itself, one cylinder; a ball's
//!   face at both end poses (its ball) and its great circle square to `D` swept, the cylinder of
//!   its radius along the centre's path; each circle edge where its faces meet at an angle swept,
//!   an oblique cylinder (the circle's disc's cylinder sheared along `D`, an affine image the
//!   Booleans meet in the preimage's closed forms), or the circle's own plane where `D` lies in it;
//!   a torus's face (under a slide along its axis or square to it) at both end poses, and its
//!   contact circles swept: along its axis its equators, into coaxial cylinders; across it its top
//!   and bottom rings, within their level planes, and its meridians square to `D`, into cylinders
//!   along `D` (each touching the torus round its meridian, met in closed form, `ssi`); a whole
//!   ring's face under a slide leaning to its axis (#805) at both end poses, and its two contacts —
//!   closed curves smooth across the branch locus, where each latitude's two roots meet, read in the
//!   angle chart (`slid::Slid`) — each swept into the cylinder over its apparent contour, its solid
//!   the contour's region extruded along `D` from short of the ring at the start pose to past it at
//!   the end (`Cutter::Contour`), which the ring at each end pose touches along its contact there
//!   (met in closed form, `ssi`'s `slid_meeting`); refused by name where the inner contour folds
//!   (`Unbuilt::Folded`, `tan² β ≥ (R − r)/r`); a band of a torus's tube leaning to the slide
//!   (#810, `Band`: a face between two latitudes of the tube's outer half, each a smooth seam into
//!   its neighbour — a bull-nose's round) at both end poses, and its contact's arc on the band swept
//!   into the sheet over that arc of the contour (`Cutter::Sheet`), cut at the seams it crosses and
//!   run along `D` past the domain, which the plane its neighbour's ruling sweeps touches along the
//!   path of the point where the contact crosses the seam (given, below);
//! - under a turn: every face at both end poses; every edge turned; a face square to the axis, its
//!   own plane (kept in itself, O); every other face's contact line (its points whose normal passes
//!   through the axis; it meets the axis or runs along it) turned, over the stretch the face spans;
//!   a cylinder's or a cone's face about an axis along the turn's at both end poses (its solid
//!   cylinder or frustum) and its two generators in the plane through both axes turned (a cylinder
//!   or a cone about the turn's axis, the face itself where coaxial); a ball's face at both end poses
//!   and its great circle in the plane through the axis and its centre turned, a torus; a circle
//!   square to the axis, its own plane — and any rim lying in a plane square to the axis (an ellipse,
//!   a conic's spline: a leaning round face cut level), its own plane. A cylinder's or a cone's face
//!   about an axis leaning to the turn's (#780 C3) at both end poses (its own surface a fortieth past
//!   its span each way), and its contact — on each ring `A cos u + B sin u = C`, two curves along the
//!   face — each turned into the surface of revolution of its meridian, read exactly
//!   (`turned::Turned`, `Cutter::Turned`: the meridian a twentieth past the face's span each way,
//!   closed down the axis at both ends and turned once), which the face at each end pose touches
//!   along its contact turned there (in closed form, `ssi`'s `own_contact`); an axis in one plane
//!   with the turn's, along its two generators in that plane, turned into hyperboloids. A sharp
//!   circle or arc of one leaning to the axis, convex (#780 L2: its fan's contact lies on the surface
//!   its points sweep), turned into its circle's ring, whose meridian is read exactly
//!   (`turned::Rim`, an ellipse in `(ρ² + z², z)`), the side holding it at an end pose reaching past
//!   it beyond the loop where the two cross (`turned::Crossing`). A whole
//!   turn takes no face at its end poses: its sweep is the tool's solid of revolution, which those
//!   faces only touch. An edge where a round face runs on smoothly (a line into a plane, a circle
//!   into a ball, however it leans) bounds nothing, nor does a concave one (a stem entering a ball).
//! - under a screw: every face at both end poses; an edge along the axis, and a face along it, its
//!   contact line (its normal square to the screw's velocity there: its line nearest the axis, as
//!   under a turn), a cylinder about the axis over the heights the source spans, run on by the
//!   step the roll takes; an edge square to the axis, the helicoidal sheet it screws into
//!   (`screwed_sheet`). A face square to the axis has no contact (its normal meets the velocity by
//!   the step everywhere); an edge leaning to the axis, and a leaning face's contact line (its
//!   points where `n · v = 0`, a line of its plane), each screw into a patch of the ruled helicoid
//!   over the segment and the roll a twentieth past each, its rim inside the domain
//!   (`Cutter::Patch`, #777 M5). A cylinder's face about an axis along the screw's touches along
//!   its generators in the plane through both axes, screwed into coaxial cylinders; a cone's
//!   (#780 C2: an end mill's chamfer, a countersink) along the two rulings where its normal
//!   `cos α ρ̂ − sin α z` is square to the velocity, `A cos u + B sin u = C` whatever the height
//!   along them — turned off the plane through both axes by `asin(κ tan α / (ω |d|))`, or none (a
//!   cone about the screw's own axis) — each a line leaning to the axis, screwed into a patch,
//!   which the cone at each end pose touches along that ruling (met in closed form,
//!   `ssi::screwed_line_cone`); its rims' discs screw into their ramps, each meeting the cone at an
//!   end pose in closed form, its radius read at the height the rim has climbed to. A ball's great
//!   circle square to its centre's velocity screws into the pipe about the centre's helix
//!   (`Cutter::Pipe`); a pipe that comes back onto itself, past a turn at a lead short of the
//!   ball's diameter, is entered in stretches each meeting itself nowhere, met pairwise along the
//!   helices where it returns (#780 L1, X5: `twist::pipe_stretches`).
//!
//! Only the candidates that carry some of the boundary are kept (`carries`): read at their source's
//! points carried over the roll, inside the source and the roll, where membership a hair to each
//! side differs. **The box is split by every candidate at once** (#735,
//! `boolean::arrangement_within`): each candidate a solid one of whose faces lies on it across the
//! box — a plane a block standing on it, far wider than the box; a surface of revolution its solid
//! over the heights its source spans; a screwed line's sheet itself, an open face reaching past
//! the box (a box square to the axis, as tall as the sweep) — their pieces outside the box left
//! out; each cell of the
//! complex is kept where the deepest of its points whose membership settles is a member, and the
//! closed solid is the boundary between the kept cells and the rest (`Complex::select`): one
//! arrangement and one selection, with no separator, no order of cuts, no Boolean a cell and no
//! union. The candidates' touches are the arrangement's to hold: a face at an end pose touching its
//! own contact surface along a line, or holding the ruling of a hyperboloid it is tangent to at its
//! throat; a hyperboloid touching the cylinder of its throat's radius round a circle; a plane
//! crossing that circle, where the two meetings it lays are tangent — each laid as an edge, or cut
//! at the one vertex the tolerant rules make of it (`boolean`'s `touch_point`, `touch_root`,
//! `crossing_in_touch`). Under a screw, a level edge's sheet touches the cylinder its foot (its
//! point nearest the axis) runs on along that point's helix, and a face along the axis at an end
//! pose holds a ruling of the sheet of its own level edge, touching it at that foot: all three
//! meet in closed form (`ssi`'s screwed meetings, a plane's `Curve::SheetCut`), so the touch is a
//! vertex of three exact curves.
//!
//! **Touches given by construction** (#810, #780 R5): what the pipeline knows of where two of its
//! candidates touch, it hands the arrangement as given edges (`boolean::Given`, laid with no search
//! for that pair; `Touch`, each naming its two candidates): a band's seam points' paths on the sheet
//! and the planes beside it; the torus at each end pose of a slide along its contact on its
//! contour's cylinder or sheet (#805); a leaning round face at each end pose of a turn along its
//! contact turned (C3); a face holding a sharp circle at each end pose along its ring (L2). Each of
//! these `ssi` had recognised pair by pair, and each recognition can be bypassed
//! (`ssi::bypassing`), the build then meeting the pair as given. A solid of revolution among the
//! candidates is turned about its own axis so its seam stands clear of the touches (`seam_away`).
//!
//! **Never returned wrong, and failing fast** (#777 V): what the selection returns is judged by the
//! sweep's definition before it is returned (`judge`: the solid's exact point location against
//! membership at seeded points of the domain and at the cells' own points, and its sense), and a
//! disagreement is refused with its witness (`Unbuilt::Wrong`). The build's work is counted in
//! steps per stage against `BUDGET` (`util::meter`), and past it refused by the stage that ran away
//! (`Unbuilt::Budget`); `SOLVENT_SWEEP_PROFILE` prints the counts, and `SOLVENT_SWEEP_DUMP` writes a
//! failed build's arrangement to replay alone (`swept::dump`).
//!
//! **A path of pieces** (K1, `Trajectory::Pieces`) of slides and turns, each a constant twist or
//! one taken up where the piece before left the tool (`Trajectory::Placed`), is its pieces' sweeps
//! united: each piece's candidates, read on the tool where that piece takes it up over its stretch
//! of the roll, split one box at once, and each cell is read in the sweep of any piece
//! (`member::Swept::member`, piece by piece).
//!
//! A candidate whose solid stands clear of the domain's box (a curved tool's end pose clear of the
//! blank it passes wholly through) is left out of the arrangement, so its touches with the others
//! there are never met.
//!
//! **Any other rigid motion** (K2, #780 L3: a frame along a path, one motion seen from another) is
//! built from the sheet each instant's contact sweeps, fitted (`patches`), and the tool where it
//! starts and where it stops, whole, every meeting of the two given (`k2`): the tool a convex solid
//! of revolution, its contact read round a pencil of half-planes about its centre's velocity.
//!
//! **A prism under a motion that keeps a plane** (#880 Q2), a motion no constant twist takes
//! nor a path of pieces, is built before K2's road by its section's planar sweep, extruded through
//! the tool's slabs and judged (`planar_solid`); a tool that stands along no plane the motion keeps
//! goes on to K2.
//!
//! **A whole turn of any tool is built first by its shadow** (#880 Q1, `super::shadow`): its
//! candidates laid in a half-plane through the axis, arranged and kept there by membership, and
//! revolved, with no surface met in space; the arrangement here is asked only where that is refused.
//! **A convex polyhedral tool turned whole is built before it by its closed form** (`turned_whole`):
//! its solid of revolution, the radii each slice holds turned once, with no mesh and no Boolean.
//!
//! Anything else — under a
//! screw a round face about an axis leaning to it, or a torus's; under a turn a cylinder's or a
//! cone's about a leaning axis whose contact is no curve over the face (its two roots on a ring
//! meeting, or the curve reaching the axis) or whose meridian folds back along the axis, a ball
//! reaching across its axis, a torus's leaning to it, a sharp circle through it or whose normal lies
//! in the plane through it and its centre; under a screw a sharp circle leaning to its axis (#780
//! L2: its screwed sheet's meetings with the faces holding it at an end pose, and with a ball's pipe,
//! not yet in closed form); under a slide a torus's face leaning to its axis that is neither a whole
//! ring nor a band between smooth seams — a band meeting a neighbour in a sharp rim among them, its
//! fan not entered here (#810) — or a band whose contact crosses both its seams, or a whole ring's
//! whose inner contour folds (by name, `Folded`); a free-form face, a curve no line or circle, a
//! leaning edge under a screw; under any other motion a tool no convex solid of revolution, a
//! contact whose pieces change over the roll, a sheet folding or coming back onto itself
//! (`Folded`, `Returns`) — is refused by the closed class by what it is (`Unbuilt`), and under a
//! constant twist a `Curved` or `Folded` refusal is asked of the contact's road below.
//!
//! **Under a constant twist, what the closed class refuses as `Curved` or `Folded` goes the
//! contact's road** (#880 Q1b, `contacts_road`; T1 for the folds): each face's contact and each
//! sharp edge's fan in stretches, carried over the roll, with the end poses and every meeting
//! given, trimmed by membership and judged (`super::trim`) — a torus or a V wheel leaning to a
//! screw, a drum leaning to it, a round meeting its neighbour in a sharp rim under a slide; a ring
//! slid leaning past its fold, a ball screwed round a helix tighter than its radius, whose sheets
//! fold, the trim keeping of each what bounds the sweep (to about 1e-7 of it: near a cusp's path
//! the two leaves stand nearly tangent, the fold cut proper's to close). Where that road refuses
//! too, the closed class's refusal stands.
use crate::build::{Profile,ProfileEdge,disc,prism,square};
use crate::geom::{Curve,Frame,Surface};
use crate::turned::Exact;
use crate::topo::{Brep,EdgeCurve};
use crate::trajectory::{Pose,Screw,Trajectory};
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use super::member::{Solid,Swept};
#[allow(unused_imports)]
use crate::util::fmath::Det;

mod gather;
mod slide;
mod twist;

type V = [f64;3];

/// The legs a path is taken in (`legs`): each one's constant twist, its stretch of the roll, and
/// where it places the tool.
type Legs = Vec<(Screw,[f64;2],Option<Pose>)>;

/// Why the closed swept solid is not built.
#[derive(Clone,Debug,PartialEq)]
pub enum Unbuilt {
    /// The tool has a face that is no plane, or an edge that is no line, at `at` — under a slide,
    /// a face no plane, cylinder, cone, ball or torus, a torus's leaning to the slide that is
    /// neither a whole ring nor a band between smooth seams (#810: a sharp rim on it, at the rim),
    /// or an edge no line or circle; under a turn, a
    /// cylinder or a cone about an axis leaning to the turn's whose contact is no curve over the
    /// face or whose meridian folds back along the axis (or crossing the turn's axis within the
    /// face, #780 C3), a ball reaching across its axis, a torus leaning to it, a sharp edge no line
    /// or circle leaning to it, or a sharp circle through the axis or symmetric about the plane through
    /// it and its centre (#780 L2); under a screw, a cylinder or a cone about an axis leaning to the screw's, a
    /// torus, or a circle leaning to it: its envelope sheets are curved past what the candidates
    /// hold, the design's later steps.
    Curved { at: V },
    /// A contact's swept sheet folds (docs/swept-solid-design.md step 6, the fold locus): at `at`,
    /// a point of the tool's contact, the sheet would double back on itself, which its patch is not
    /// yet cut at — a torus slid at a lean steep enough that the apparent contour of its inner
    /// contact cusps (#805, `slid::Slid::folded`), or a ball screwed whose radius reaches its
    /// centre's helix's radius of curvature (#780 L1, X5). Never built wrong.
    Folded { at: V },
    /// A contact's swept sheet comes back onto itself under a motion no constant twist takes (#780
    /// L3, X5 under K2): at `at`, between two poses the tool's centre comes back within its reach
    /// of, after running more than half a turn of a circle of that reach — where the sheet is not
    /// yet entered in stretches, as a screwed ball's pipe is. Never built wrong.
    Returns { at: V },
    /// The motion is no slide, turn or screw, nor a path of slides and turns (each taken up where
    /// the one before left the tool, `Trajectory::Placed`), and K2's road was not asked (`reads`
    /// reads a constant twist; `reads_general` any other motion).
    Motion,
    /// A cell's membership could not be settled: its deepest point stands within the mesh's sag
    /// of the swept boundary.
    Unsettled { at: V },
    /// The build went past its budget (#777 V3, `util::meter`): the stage that ran away, a point
    /// where it was working, and what it was working on there (the surfaces, by kind). Counted in
    /// steps, so the same build is refused the same way every time.
    Budget { stage: crate::util::meter::Stage,at: V,what: String },
    /// The built solid disagrees with the sweep's definition (#777 V2): at the witness `at`, the
    /// solid holds it (`held`) where membership says the opposite. Never returned built wrong.
    Wrong { at: V,held: bool },
    /// The kernel could not do it.
    Failed(String),
}

impl From<String> for Unbuilt { fn from(m: String) -> Unbuilt { Unbuilt::Failed(m) } }

impl From<crate::KernelError> for Unbuilt { fn from(e: crate::KernelError) -> Unbuilt { Unbuilt::Failed(e.msg) } }

/// A build stopped past its budget is refused by it, naming the stage that ran away (#780 R3).
impl From<crate::util::meter::Stopped> for Unbuilt {
    fn from(crate::util::meter::Stopped(o): crate::util::meter::Stopped) -> Unbuilt { Unbuilt::Budget {stage:o.stage,at:o.at,what:o.what} }
}

/// A plane: a point of it and its unit normal.
type Plane = (V,V);

/// A candidate the box is split by, extended whole: a plane, or the cylinder of radius `r` about
/// the line through `o` along unit `axis`.
#[derive(Clone,Copy,Debug)]
enum Cutter { Plane(Plane),
    /// The cylinder of radius `r` about the unit `axis` through `o`, over the heights `span` along it
    /// from `o`.
    Cylinder { o: V,axis: V,r: f64,span: [f64;2] },
    /// The line through `p` along unit `e`, leaning to the unit `axis` through `o`, turned about it
    /// whole: a hyperboloid of one sheet, or where the line meets the axis a double cone; over the
    /// heights `span` along the axis from `o`.
    Revolved { o: V,axis: V,p: V,e: V,span: [f64;2] },
    /// The line through `p` along unit `e`, square to the screw's axis, screwed whole: a helicoidal
    /// sheet (a right helicoid where the line meets the axis).
    Screwed { p: V,e: V },
    /// The segment from `p` to `q`, leaning to the screw's axis, screwed over the roll (#777 M5): a
    /// patch of the ruled helicoid it sweeps, a little past the segment's ends and the roll's, its
    /// rim standing inside the domain (`patch`).
    Patch { p: V,q: V },
    /// The torus about `c` square to the unit `axis`, its tube's middle circle of radius `major`, its
    /// tube's radius `minor` (a ball's great circle through the turn's axis, turned).
    Torus { c: V,axis: V,major: f64,minor: f64 },
    /// The disc of radius `r` about `c` square to the screw's axis, screwed over the roll: the solid
    /// a round tool's rim sweeps under a screw, its side the sheet the rim screws into.
    ScrewedDisc { c: V,r: f64 },
    /// A round face's contact turned (#780 C3): the surface of revolution about the turn's axis
    /// whose meridian is where the contact of a cylinder or a cone about an axis leaning to the
    /// turn's goes in `(ρ, z)` (`turned::Turned`), its solid that meridian closed down the axis at
    /// both ends and turned once; or a sharp circle leaning to the turn's axis turned (#780 L2), its
    /// meridian closed, its solid a ring.
    Turned(crate::turned::Turned),
    /// The disc of radius `r` about `c` square to the unit `n` screwed over the stretch `span` of the
    /// roll: the solid a ball's great circle square to its centre's velocity sweeps under a screw,
    /// its side the pipe about the centre's helix — the whole roll, or one of the stretches a pipe
    /// coming back onto itself is entered in (#780 L1, X5: `twist::pipe_stretches`).
    Pipe { c: V,n: V,r: f64,span: [f64;2] },
    /// The ball of radius `r` about `c`, its poles on the unit `axis` (a ball's face at an end
    /// pose, its poles where the tool's are).
    Ball { c: V,axis: V,r: f64 },
    /// The circle of radius `r` about `c` square to the unit `a`, carried along `d`: an oblique
    /// cylinder, the solid the circle's disc sweeps (a cylinder's solid sheared along `d`).
    Oblique { c: V,a: V,r: f64,d: V },
    /// A slid torus's contact swept along the slide (#805): the cylinder over its apparent contour,
    /// its two halves (`slid::halves`, each read as the contour), over the heights `span` along the
    /// slide from the torus's centre at the start pose — its solid the contour's region extruded.
    Contour { halves: [crate::slid::Slid;2],span: [f64;2] },
    /// A slid torus band's contact swept along the slide (#810): the sheet over the arc `arc` of
    /// its outer contour (`slid::Slid`, read as the contour, its span the contact's arc on the band,
    /// from seam to seam), along the slide past the domain both ways — an open face, no solid, whose
    /// only rims within the domain are the lines along the slide through the seam points, where the
    /// planes the neighbours' rulings sweep touch it (given, `boolean::Given`).
    Sheet { arc: crate::slid::Slid } }

/// **The closed solid `tool` sweeps under `path` over `roll`** (the module's cases), its tool,
/// motion and solid in one set of units, built to `tol`; membership read on the tool meshed within
/// `bar`.
pub fn closed(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    built(tool,path,roll,bar,tol,None)
}

/// **The solid `tool` sweeps under `path` over `roll` cut from `blank`** (#699 C on #735's one
/// arrangement): the blank split by the same candidates `closed` splits its box by, all at once
/// (`boolean::arrangement_within`, the blank the domain), each cell kept where it is no member of
/// the sweep, and the blank's boundary with the swept cells the result (`boolean::selection`). No
/// closed solid is built and then subtracted: the blank and the sweep's candidates are one
/// arrangement, classified once — the closed solid is the same arrangement over a box, its cells
/// kept where they are members. A sweep missing the blank leaves it whole. (The contact's road,
/// `contacts_road`, and a prism under a motion keeping a plane, `planar_solid`, are the exceptions:
/// each closed solid is built alone and then cut from the blank.)
pub fn cut_from(blank: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    built(tool,path,roll,bar,tol,Some(blank))
}

/// **Whether the pipeline takes `tool` under `path`**, read off their structure before anything is
/// built (what an admission asks, #721's dispatch): the motion a slide, a turn or a screw that
/// moves something; every face of the tool a plane and every edge a line bounding two of them
/// (under a slide, a face a cylinder, a cone or a ball too, a torus about an axis along it or square
/// to it, or a whole ring at any lean short of its fold (#805) or a band of one between smooth seams
/// (#810), and an edge a circle; under a turn, a
/// cylinder or a cone about any axis (#780 C3), a ball, a sharp curve in a plane square to it, and a
/// sharp circle or arc of one leaning to it (#780 L2);
/// under a screw, a
/// cylinder or a cone about an axis along it, and a ball beside planes and such a cylinder). The
/// constant twist it is, or the first thing that is not, by what it is and where (a point of the
/// face or the edge).
pub fn reads(tool: &Brep,path: &Trajectory) -> Result<crate::trajectory::Screw,Unbuilt> { read(tool,path).map(|r| r.screw) }

/// What `reads` reads of a tool it takes: the constant twist, each face's plane (a point of it and
/// its outward unit normal; for a face no plane, a point of it and no normal), each straight
/// edge's two ends, two faces and their outward normals along it (the same all along a line on a
/// plane, a cylinder or a cone), each face no plane (`round`) and each circle edge where its faces
/// meet at an angle (`rims`): under a slide any, under a screw one square to its axis, under a turn
/// one leaning to it, whole or an arc (#780 L2).
struct Read { screw: crate::trajectory::Screw,faces: Vec<(V,V)>,edges: Vec<(V,V,[usize;2])>,normals: Vec<[V;2]>,
    round: Vec<usize>,rims: Vec<Rim>,
    /// Under a slide, each band of a torus leaning to it (#810): the face, the band, and where its
    /// contact lies on it.
    bands: Vec<(usize,Band,BandContact)>,
    /// Under a turn, each sharp edge no line lying in a plane square to the axis — a circle about a
    /// line along it, or a leaning round face's rim cut level, an ellipse or a conic's spline (#780
    /// C3) — read at its points: it turns within its own plane.
    levels: Vec<Vec<V>> }

/// A circle edge of the tool: its frame (centre and the circle's normal), radius, span of angle and
/// the two faces it bounds.
#[derive(Clone,Copy,Debug)]
struct Rim { frame: Frame,r: f64,span: [f64;2],faces: [usize;2] }

impl Rim {
    /// Its circle in the turn's frame `g` (`turned::Rim`).
    fn ring(&self,g: &Frame) -> crate::turned::Rim {
        crate::turned::Rim {o:g.local(self.frame.o),x:g.dir_local(unit(self.frame.x)),y:g.dir_local(unit(self.frame.y)),r:self.r.abs()}
    }
}

/// Whether two faces whose outward unit normals at a point of their edge are `n1` and `n2` meet
/// there smoothly: one normal, its fan a single way.
fn smooth(n1: V,n2: V) -> bool { norm(cross(n1,n2)) <= 1e-9 && dot(n1,n2) > 0. }

/// A face's outward unit normal at its point `p`.
fn outward(f: &crate::topo::Face,p: V) -> V {
    let n = unit(f.surface.normal_raw(f.surface.inverse(p)));
    if f.reversed { scale(n,-1.) } else { n }
}

/// `reads`, keeping what it read.
fn read(tool: &Brep,path: &Trajectory) -> Result<Read,Unbuilt> {
    let Some(screw) = path.screw().filter(|s| s.ratio != 0. || s.advance != 0.) else { return Err(Unbuilt::Motion) };
    // (the build's tolerance of length, as `build` takes it)
    let eps = { let (lo,hi) = tool.bounds(); 1e-9*(1.+norm(sub(hi,lo))) };
    let mut faces = Vec::with_capacity(tool.faces.len());
    let mut round = Vec::new();
    let mut bands = Vec::new();
    let uses = tool.uses();
    for (k,f) in tool.faces.iter().enumerate() {
        let at = f.loops.iter().flatten().next().map_or([0.;3],|c| tool.vertices[tool.ends(c)[0] as usize].p);
        match &f.surface {
            Surface::Plane(frame) => {
                // (a face leaning to a screw's axis touches along a leaning line, screwed into a patch
                // of a ruled helicoid bounded inside the domain, #777 M5)
                faces.push((frame.o,unit(if f.reversed { scale(frame.z,-1.) } else { frame.z })));
            }
            // under a slide a cylinder's or a cone's contact is along its rulings, each swept into
            // its tangent plane there, or (a cylinder along the slide) the whole face swept along
            // itself; a ball's is its great circle square to the slide, swept into a cylinder
            // (its entry here unread: its candidates are read off its surface)
            Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) if screw.is_slide() => {
                round.push(k);
                faces.push((at,[0.;3]));
            }
            // under a turn a cylinder's or a cone's about an axis along the turn's touches along its
            // generators in the plane through both axes, turned into a cylinder or a cone about the
            // turn's; about an axis leaning to it, along a curve on each ring where
            // `A cos u + B sin u = C`, turned into a surface of revolution whose meridian is read
            // exactly (#780 C3, `turned::Turned`)
            Surface::Cylinder(..) | Surface::Cone(..) if screw.is_turn() => {
                round.push(k);
                faces.push((at,[0.;3]));
            }
            // under a screw a cylinder's about an axis along it touches along its generators in the
            // plane through both axes, screwed into coaxial cylinders; a cone's along the two
            // rulings where its normal is square to the screw's velocity, turned off that plane,
            // each screwed into a patch of a ruled helicoid (#780 C2)
            Surface::Cylinder(g,_) | Surface::Cone(g,..) if !screw.is_turn() && !screw.is_slide() && norm(cross(g.z,screw.axis)) <= 1e-9 => {
                round.push(k);
                faces.push((at,[0.;3]));
            }
            // under a slide along a torus's axis or square to it, its contacts are whole circles: its
            // equators along it, swept into cylinders; across it its top and bottom rings, swept
            // into level planes, and its meridians square to the slide, swept into cylinders
            Surface::Torus(g,..) if screw.is_slide() && (dot(g.z,screw.axis).abs() <= 1e-9 || norm(cross(g.z,screw.axis)) <= 1e-9) => {
                round.push(k);
                faces.push((at,[0.;3]));
            }
            // under a slide leaning to a whole ring's axis (#805, X2 under a constant twist), its
            // contact is two closed curves, smooth across the branch locus where each latitude's two
            // roots meet, each swept along the slide into the cylinder over its apparent contour
            // (`slid::Slid`) — refused by name where the inner contour folds (`Unbuilt::Folded`)
            Surface::Torus(g,big,small) if screw.is_slide() && whole(f) => {
                if !(big.abs() > small.abs()) { return Err(Unbuilt::Curved {at}) }
                if let Some(at) = slid_fold(g,big.abs(),small.abs(),screw.axis) { return Err(Unbuilt::Folded {at}) }
                round.push(k);
                faces.push((at,[0.;3]));
            }
            // under a slide leaning to the axis of a torus's face that is no whole ring (#810, #780
            // R5: a bull-nose's round, a rounded drum's corner): a band of its tube's outer half
            // between two latitudes, each a smooth seam into its neighbour, its contact the outer
            // contact's arc on the band, crossing a seam into a cylinder's or a cone's contact there
            // (`Band`) — refused by name at a sharp rim, whose fan is not entered yet
            Surface::Torus(..) if screw.is_slide() => {
                let band = Band::of(tool,k,&uses)?;
                bands.push((k,band,band.contact(screw.axis)?));
                round.push(k);
                faces.push((at,[0.;3]));
            }
            // a ball's under a turn along its great circle in the plane through the axis and its
            // centre; under a screw along its great circle square to its centre's velocity, screwed
            // into the pipe about its centre's helix, beside planes and a side about an axis along
            // the screw's (a ball-end mill, whose pipe touches the side's cylinders along the
            // helices of the ball's equator, #777 M3 and M4)
            Surface::Sphere(..) if screw.is_turn() || !screw.is_slide()
                && tool.faces.iter().all(|g| match &g.surface {
                    Surface::Sphere(..) | Surface::Plane(_) => true,
                    Surface::Cylinder(c,_) => norm(cross(c.z,screw.axis)) <= 1e-9,
                    _ => false,
                }) => {
                round.push(k);
                faces.push((at,[0.;3]));
            }
            // under a turn a torus's about an axis along the turn's touches round its two meridians in
            // the plane through both axes, turned into tori about the turn's, and round its top and
            // bottom rings, which turn within their own planes
            Surface::Torus(g,..) if screw.is_turn() && norm(cross(g.z,screw.axis)) <= 1e-9 => {
                round.push(k);
                faces.push((at,[0.;3]));
            }
            _ => return Err(Unbuilt::Curved {at}),
        }
    }
    let mut edges = Vec::with_capacity(tool.edges.len());
    let mut normals = Vec::with_capacity(tool.edges.len());
    let mut rims = Vec::new();
    let mut levels = Vec::new();
    for (e,edge) in tool.edges.iter().enumerate() {
        if matches!(edge.curve,EdgeCurve::Degenerate) { continue }
        let (p,q) = (edge.point(edge.t[0],&tool.vertices),edge.point(edge.t[1],&tool.vertices));
        let mid = edge.point(0.5*(edge.t[0]+edge.t[1]),&tool.vertices);
        let [(f1,..),(f2,..)] = uses[e][..] else { return Err(Unbuilt::Failed(format!("edge {e} bounds {} faces",uses[e].len()))) };
        // a seam, one face on both sides of it: no edge of the solid
        if f1 == f2 { continue }
        // where its two faces meet smoothly, its fan one normal, the faces' own contacts'
        let (n1,n2) = (outward(&tool.faces[f1],mid),outward(&tool.faces[f2],mid));
        let smooth = smooth(n1,n2);
        match &edge.curve {
            EdgeCurve::Curve(Curve::Line {..}) => {
                // (a line where a round face runs on smoothly into a plane — a rounded key's flat
                // into its end — is no edge of the solid, #780 C3)
                if smooth && [f1,f2].iter().any(|&f| !matches!(tool.faces[f].surface,Surface::Plane(_))) { continue }
                edges.push((p,q,[f1,f2]));
                normals.push([n1,n2]);
            }
            // (a circle where two faces meet smoothly, a ball on a cylinder, is no edge of the
            // solid, however it leans)
            EdgeCurve::Curve(_) if smooth => continue,
            EdgeCurve::Curve(c) => {
                // convex, or concave: the two faces' material wrapping round it more than half a
                // turn (a stem entering a ball). Its first face's way in from it, square to the edge
                // (the face on the left of its use), leans along the second's outward normal there
                let convex = {
                    let use1 = &tool.faces[f1].loops[uses[e][0].1][uses[e][0].2];
                    let along = c.tangent(0.5*(edge.t[0]+edge.t[1]));
                    let along = if use1.reversed { scale(along,-1.) } else { along };
                    dot(cross(n1,along),n2) <= 1e-9
                };
                // its points, where they lie in one plane square to the axis
                let level = || -> Option<Vec<V>> {
                    let h = |p: V| dot(sub(p,screw.origin),screw.axis);
                    let points: Vec<V> = (0..=SITES).map(|i| edge.point(edge.t[0]+(edge.t[1]-edge.t[0])*within(i),&tool.vertices)).collect();
                    points.iter().all(|&p| (h(p)-h(mid)).abs() <= 1e-9*(1.+norm(sub(p,screw.origin)))).then_some(points)
                };
                match c {
                    // a circle under a slide, or under a screw square to its axis: swept, or screwed
                    // into the solid its disc sweeps
                    Curve::Circle(frame,r) if !screw.is_turn() && (screw.is_slide() || norm(cross(frame.z,screw.axis)) <= 1e-9) => {
                        rims.push(Rim {frame:*frame,r:*r,span:edge.t,faces:[f1,f2]})
                    }
                    // any curve in a level plane, under a turn (a circle about a line along the axis,
                    // a leaning round face's rim cut level, #780 C3): within its plane
                    _ if screw.is_turn() && let Some(points) = level() => levels.push(points),
                    // a sharp circle or arc of one leaning to a turn's axis, convex (#780 L2: its fan's
                    // contact, the circle's points whose fan holds a normal square to the turn's
                    // velocity, lies on the ring the circle turns into): turned, a ring (below)
                    // — read at its reading, so a sweep is not admitted that its build refuses: refused
                    // where it turns into no ring (`turned::Rim::simple`: through the axis, or its
                    // normal in the plane through the axis and its centre, as a pin along a radius has)
                    Curve::Circle(frame,r) if screw.is_turn() && convex => {
                        let rim = Rim {frame:*frame,r:*r,span:[edge.t[0].min(edge.t[1]),edge.t[0].max(edge.t[1])],faces:[f1,f2]};
                        if !rim.ring(&Frame::about(screw.origin,screw.axis)).simple(eps) { return Err(Unbuilt::Curved {at:frame.o}) }
                        rims.push(rim)
                    }
                    // a concave edge carries none of the swept boundary: what bounds a sweep near it
                    // is its faces' contacts
                    _ if !convex => {}
                    _ => return Err(Unbuilt::Curved {at:mid}),
                }
            }
            _ => return Err(Unbuilt::Curved {at:mid}),
        }
    }
    Ok(Read {screw,faces,edges,normals,round,rims,bands,levels})
}

/// The legs `path` is taken in over `roll`, each a constant twist over its stretch of the roll and
/// where it takes the tool up: the twist itself; a twist after a placement (`Trajectory::Placed`, a
/// turn after a slide); or each piece of a path of pieces (K1, a sweep its pieces' sweeps united)
/// over the stretch of the roll it holds, each a slide or a turn, placed or not (a screw is taken
/// only alone) — each leg's reading of the tool (`read`) taken.
fn legs(tool: &Brep,path: &Trajectory,roll: [f64;2]) -> Result<Legs,Unbuilt> {
    let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    // a piece's twist and placement, read on the tool where it stands
    let leg = |path: &Trajectory| -> Result<(crate::trajectory::Screw,Option<crate::trajectory::Pose>),Unbuilt> {
        let (twist,at) = match path {
            Trajectory::Twist(s) => (*s,None),
            Trajectory::Placed {twist,at} => (*twist,Some(*at)),
            _ => return Err(Unbuilt::Motion),
        };
        let placed = at.map(|q| tool.moved(&crate::geom::Rigid {r:q.r,t:q.p}));
        read(placed.as_ref().unwrap_or(tool),&Trajectory::Twist(twist))?;
        Ok((twist,at))
    };
    let legs = match path {
        Trajectory::Pieces(pieces) => {
            let mut out = Vec::new();
            for piece in pieces.pieces() {
                let (lo,hi) = (piece.over[0].max(a),piece.over[1].min(b));
                if !(hi > lo) { continue }
                let (screw,at) = leg(&piece.path)?;
                if !(screw.is_slide() || screw.is_turn()) { return Err(Unbuilt::Motion) }
                out.push((screw,[lo,hi],at));
            }
            out
        }
        Trajectory::Placed {..} => {
            let (screw,at) = leg(path)?;
            if !(screw.is_slide() || screw.is_turn()) { return Err(Unbuilt::Motion) }
            vec![(screw,[a,b],at)]
        }
        _ => { let (screw,at) = leg(path)?; vec![(screw,[a,b],at)] }
    };
    if legs.is_empty() { return Err(Unbuilt::Motion) }
    Ok(legs)
}

/// **The work a closed build may do** (#777 V3), by stage (`util::meter::Stage`): past what every
/// build the suite holds takes (each measured by its profile, `SOLVENT_SWEEP_PROFILE`), and short
/// of what one runs to where a generic fallback runs without a bound — refused naming the stage,
/// not after hours. The inverse map's is about four times the most a build takes (the drum
/// screwed beside the axis, 51 million; the leaning block 8.5 million), so the ball-end mill
/// screwed through a sector, whose sheet's inverse ran for hours, is refused in about two minutes.
pub const BUDGET: crate::util::meter::Budget = crate::util::meter::Budget([
    // arrangement: pairs of faces met
    400_000,
    // trace: steps marched along meetings with no closed form
    4_000_000,
    // ray: curve–surface searches and the samples sampled ones take
    2_000_000_000,
    // outline: chords faces' loops are cut in
    20_000_000,
    // inverse: a swept sheet's inverse map
    200_000_000,
    // membership: stretches of the roll read
    400_000_000,
    // judge: points judged
    100_000,
]);

/// `closed` within `budget` (`BUDGET` is `closed`'s): past it, refused by the stage that ran away
/// (`Unbuilt::Budget`).
pub fn closed_within(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,budget: crate::util::meter::Budget) -> Result<Brep,Unbuilt> {
    metered(tool,path,roll,bar,tol,None,budget)
}

/// `cut_from` within `budget`.
pub fn cut_from_within(blank: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,budget: crate::util::meter::Budget)
    -> Result<Brep,Unbuilt> {
    metered(tool,path,roll,bar,tol,Some(blank),budget)
}

/// `closed` over a box holding the sweep, or `cut_from` over `blank`, within `BUDGET`.
fn built(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Brep,Unbuilt> {
    metered(tool,path,roll,bar,tol,blank,BUDGET)
}

/// `build` with a meter counting its work against `budget` (`util::meter`): a build past it refused
/// by the stage that ran away, whatever else it ended in, and its profile printed where
/// `SOLVENT_SWEEP_PROFILE` is set (#777 V4).
fn metered(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>,budget: crate::util::meter::Budget)
    -> Result<Brep,Unbuilt> {
    use crate::util::meter;
    let m = meter::Meter::new(budget);
    let clock = crate::util::clock::Instant::now();
    let out = meter::within(&m,|| build(tool,path,roll,bar,tol,blank));
    if meter::profiling() {
        eprintln!("{}  took {:?}: {}",m.report(),clock.elapsed(),match &out { Ok(b) => format!("built, {} faces",b.faces.len()),Err(e) => format!("{e:?}") });
    }
    if let Some(o) = m.over() { return Err(meter::Stopped(o).into()) }
    out
}

/// Whether `screw` turns a whole turn or more over `roll`.
fn whole_turn(screw: &crate::trajectory::Screw,roll: [f64;2]) -> bool {
    (screw.ratio*(roll[1]-roll[0])).abs() >= std::f64::consts::TAU*(1.-1e-12)
}

/// `closed` over a box holding the sweep, or `cut_from` over `blank`: the pipeline itself.
fn build(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Brep,Unbuilt> {
    // a path of pieces cut from a blank, not every piece's candidates arranged at once: piece by
    // piece, each piece's sweep by its own cheapest road, smooth runs of them as one (`pieces`)
    if let Some(blank) = blank {
        if let Some(cut) = super::pieces::cut(blank,tool,path,roll,bar,tol) { return cut }
    }
    // a whole turn of any tool (#880 Q1): its shadow in a half-plane through the axis, arranged
    // there and revolved, with no surface met in space (`shadow`) — where it is refused by what the
    // tool is or a reading it cannot settle, the arrangement below is asked; a build judged wrong,
    // or past its budget, is refused as it is
    if let (Trajectory::Twist(s),None) = (path,blank) {
        if s.is_turn() && whole_turn(s,roll) {
            // a convex polyhedral tool by its slices' radial spans, in closed form: no mesh, no
            // membership, measured far faster than its shadow (#880 P2: a closed form stays where
            // it is faster)
            if let Ok(Read {faces,edges,round,rims,levels,..}) = read(tool,path) {
                let eps = 1e-9*(1.+tool.size());
                let convex = faces.iter().all(|&(o,n)| tool.vertices.iter().all(|v| dot(sub(v.p,o),n) <= eps));
                if convex && round.is_empty() && rims.is_empty() && levels.is_empty() {
                    return turned_whole(&faces,&edges,s.origin,s.axis,eps,tol)
                }
            }
            match super::shadow::turned(tool,s.origin,s.axis,bar,tol) {
                Ok(b) => return Ok(b),
                Err(e @ (Unbuilt::Wrong {..} | Unbuilt::Budget {..})) => return Err(e),
                Err(_) => {}
            }
        }
    }
    match build_by(tool,path,roll,bar,tol,blank,None) {
        // **a tool the closed class does not read, or whose sheet folds, under a constant twist**
        // (#880 Q1b, T1): its faces' contact carried over the roll and the end poses, trimmed by
        // membership (`contacts`, `trim`), which keeps of a folded sheet what bounds the sweep, and
        // cut from the blank — or refused as the closed class refused it, where the contact's road
        // refuses it too (a face in contact all over, a meeting it does not close) (the road
        // refusing too, the closed class's refusal stands: the first thing it does not read, and
        // where)
        Err(curved @ (Unbuilt::Curved {..} | Unbuilt::Folded {..})) if path.screw().is_some() => contacts_road(tool,path,roll,bar,tol,blank).or_else(|why| {
            if crate::debug() { eprintln!("swept: the contact's road refused too: {why:?}") }
            Err(curved)
        }),
        built => built,
    }
}

/// How far the box the road trims a sweep within is grown past the blank's, as a share of its
/// diagonal: no round number, so the box's planes stand clear of the blank's and the tool's.
const REACH: f64 = 0.0937;

/// **The contact's road** (#880 Q1b): `contacts::candidates` — each face's contact in stretches carried
/// over the roll, the end poses, every two given their whole meeting — trimmed by membership into the
/// closed sweep (`trim::trim`, judged there), and cut from `blank` where there is one. The hair a
/// piece's sides are read at is the larger of four tolerances, above the sections' fit, and a
/// millionth of the tool: a hair of a few tolerances at a fine one leaves a point within it of the
/// sweep within it of the tool over a spread of poses its square root wide, more stretches than
/// membership reads.
fn contacts_road(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Brep,Unbuilt> {
    let (candidates,given) = super::contacts::candidates(tool,path,roll,tol)?;
    let size = tool.size();
    let trim = super::trim::Trim {resolution:size/32.,hair:hair(tol,size),reach:None};
    // **the sweep within the blank's reach, where the whole does not close** (#880 T1c): cut from a
    // blank, the sweep is wanted only inside a box holding the blank — grown by `REACH` of its size
    // — and trimmed
    // there, its pieces outside dropped and the box's faces kept where the sweep holds them, judged
    // against membership inside the box; the cut is the same. Asked where the whole sweep's trim
    // refuses: the twist drill's steep flute wheel, run from below the stock to past its top, whose
    // flank's fold meets the wheel posed at the roll's ends far outside the stock (where the trim
    // does not close the meetings beside the cusp, #880 T1e). Not asked first: a torus screwed
    // through a rod, its stretches' rails crossing the box, is split there unlike its neighbours
    let swept = match super::trim::trim(tool,path,roll,&candidates,&given,bar,tol,trim) {
        Ok((swept,_)) => swept,
        // (a build stopped by its budget is not asked again)
        Err(whole) if !matches!(whole,Unbuilt::Budget {..}) => {
            let Some(b) = blank else { return Err(whole) };
            let (lo,hi) = b.bounds();
            let g = REACH*b.size();
            let reach = Some((lo.map(|x| x-g),hi.map(|x| x+g)));
            if crate::debug() { eprintln!("swept: the whole sweep's trim refused ({whole:?}): trimmed within the blank's reach") }
            super::trim::trim(tool,path,roll,&candidates,&given,bar,tol,super::trim::Trim {reach,..trim}).map_err(|_| whole)?.0
        }
        Err(whole) => return Err(whole),
    };
    cut_blank(swept,blank,tol)
}

/// A closed swept solid as the build returns it: alone, or cut from `blank` by the Boolean.
fn cut_blank(swept: Brep,blank: Option<&Brep>,tol: f64) -> Result<Brep,Unbuilt> {
    match blank {
        None => Ok(swept),
        Some(b) => crate::boolean::combined(b,&swept,crate::boolean::Op::Cut,tol).map_err(|e| Unbuilt::Failed(format!("the sweep cut from the blank: {e}"))),
    }
}

/// **`closed`'s candidates, trimmed** (#880 E1): the same candidates and touches `closed` makes,
/// built by trimming by membership (`trim`) rather than by their arrangement — the two set side by
/// side on every case `closed` builds. A whole turn is trimmed from its candidates here, not built by
/// its shadow or its slices as `closed` builds it; a motion `closed` takes K2's road for is refused
/// (`Unbuilt::Motion`): that road is no arrangement of candidates.
pub fn trimmed(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,trim: super::trim::Trim)
    -> Result<(Brep,super::trim::Report),Unbuilt> {
    use crate::util::meter;
    let m = meter::Meter::new(BUDGET);
    let clock = crate::util::clock::Instant::now();
    let report = std::cell::RefCell::new(None);
    let out = meter::within(&m,|| build_by(tool,path,roll,bar,tol,None,Some((trim,&report))));
    if meter::profiling() {
        eprintln!("{}  took {:?}: {}",m.report(),clock.elapsed(),match &out { Ok(b) => format!("trimmed, {} faces",b.faces.len()),Err(e) => format!("{e:?}") });
    }
    if let Some(o) = m.over() { return Err(meter::Stopped(o).into()) }
    let b = out?;
    let mut report = report.into_inner().ok_or(Unbuilt::Motion)?;
    report.meetings = m.meetings();
    report.membership = m.count(meter::Stage::Membership);
    Ok((b,report))
}

/// `build`, its candidates arranged, or trimmed (`trim`, its report written back) where asked.
#[allow(clippy::type_complexity)]
fn build_by(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>,
    trim: Option<(super::trim::Trim,&std::cell::RefCell<Option<super::trim::Report>>)>) -> Result<Brep,Unbuilt> {
    // the legs the path is taken in: one constant twist over the roll, or each piece of a path of
    // pieces over its stretch of it (K1: a sweep is its pieces' sweeps united), every one read —
    // or, a motion no constant twist and no such path takes, K2's road: each instant's contact swept
    // into a patch (#780 L3)
    let legs = match legs(tool,path,roll) {
        Err(Unbuilt::Motion) if path.screw().is_none() && trim.is_none() => {
            // a prism under a motion keeping the plane square to it: its section's planar sweep,
            // extruded (#880 Q2), and cut from the blank where there is one, before K2's road —
            // which is still asked where it refuses, its refusal standing unless K2 cannot read the
            // tool at all
            return match super::planar_solid::swept(tool,path,roll,bar,tol) {
                Ok(Some(swept)) => cut_blank(swept,blank,tol),
                Ok(None) => k2(tool,path,roll,bar,tol,blank),
                Err(why) => k2(tool,path,roll,bar,tol,blank).map_err(|k| if matches!(k,Unbuilt::Curved {..}) { why } else { k }),
            }
        }
        legs => legs?,
    };
    let whole_roll = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let size = { let (lo,hi) = tool.bounds(); norm(sub(hi,lo)) };
    let eps = 1e-9*(1.+size);
    let gather::Candidates {cutters,touches,seams,held,..} = gather::Candidates::gathered(tool,&legs,tol,eps,size)?;
    let domain = domain(tool,&legs,&held,tol)?;
    // only the candidates that carry some of the boundary: membership differing on their two sides
    // somewhere over the box (a candidate is kept where a point of it cannot be settled)
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll:whole_roll};
    // (each candidate's place among those carried, which a touch names it by)
    let carrying: Vec<(usize,Cutter)> = cutters.iter().enumerate().filter(|(_,(_,sites))| carries(&swept,sites,domain.half))
        .map(|(i,&(c,_))| (i,c)).collect();
    // a build past its budget goes no further (#777 V3: refused by `metered`, naming the stage)
    crate::util::meter::go_on()?;
    // **the box split by every candidate at once** (#735 A4, `boolean::arrangement_within`): each
    // candidate a solid one of whose faces lies on it across the box (`solids_of`), their pieces
    // outside the box left out. Where two candidates touch (a face at an end pose and its own
    // contact surface along a line, a hyperboloid and the cylinder of its throat's radius round a
    // circle) the arrangement lays the touch as an edge of both, so the cells on its two sides are
    // two
    let mut solids: Vec<Brep> = Vec::new();
    // the solids each candidate entered where it is carried (a hyperboloid may enter two), by its
    // index among `cutters`, as a touch names it
    let mut entered: Vec<Option<std::ops::Range<usize>>> = vec![None;cutters.len()];
    for &(i,c) in &carrying {
        let first = solids.len();
        solids.extend(solids_of(c,legs[0].0,legs[0].1,&domain,&seams,eps,size)?);
        entered[i] = Some(first..solids.len());
    }
    // (a candidate whose solid stands clear of the box parts none of it: left out, so its meetings
    // with the others there are never sought)
    let (within,reached) = match blank { Some(b) => (b,Some(&domain.solid)),None => (&domain.solid,None) };
    let refs = clear_of(&solids,&domain.solid,tol);
    let given = touches_given(&touches,&entered,&solids,tol);
    if let Some((t,report)) = trim {
        let owned: Vec<Brep> = refs.into_iter().cloned().collect();
        let (b,r) = super::trim::trim(tool,path,roll,&owned,&given,bar,tol,t)?;
        *report.borrow_mut() = Some(r);
        return Ok(b)
    }
    arranged_or_dumped(within,reached,&refs,&given,&swept,blank,tol)
}

/// **A box holding the sweep**, split by the candidates: its solid, its centre, half its side and,
/// under a screw, its heights along the axis and its farthest from it (`tall`).
struct Domain { solid: Brep,centre: V,half: f64,tall: Option<(f64,f64,f64)> }

/// The box holding the sweep: the tool's box at each leg's end poses and, under a turn, its reach
/// about the axis (`held`); under a screw (a lone leg), a box square to the axis and as tall along
/// it as the sweep, a little more (`tall`), so a sheet a level edge screws into leaves it within a
/// turn where the sweep climbs less than a turn's step. Cut from a blank, the box is the sweep's
/// reach (#847): every candidate reaches across it, and the blank is split by them within it alone
/// (`boolean::arrangement_reached`).
fn domain(tool: &Brep,legs: &Legs,held: &[V],tol: f64) -> Result<Domain,Unbuilt> {
    if legs.iter().all(|l| l.0.is_slide() || l.0.is_turn()) {
        let (solid,centre,half) = cube_holding(held,tol)?;
        return Ok(Domain {solid,centre,half,tall:None})
    }
    let (screw,[a,b]) = (legs[0].0,legs[0].1);
    let path = Trajectory::Twist(screw);
    let corners = corners_of(tool.bounds());
    // the tool's box carried over the roll, read at enough poses that the arcs its corners run
    // between them stand within the sagitta added
    let f = Frame::about(screw.origin,screw.axis);
    const STEPS: usize = 64;
    let mut bx = crate::geom::EMPTY;
    let mut reach = 0f64;
    for k in 0..=STEPS {
        let m = path.pose(a+(b-a)*k as f64/STEPS as f64).map_err(Unbuilt::from)?;
        for &p in &corners {
            let x = f.local(m.point(p));
            reach = reach.max(x[0].dhypot(x[1]));
            crate::geom::include(&mut bx,x);
        }
    }
    let (mut l,mut h) = bx;
    let sag = reach*(1.-(0.5*(screw.ratio*(b-a)).abs()/STEPS as f64).dcos());
    let margin = room((0..3).map(|i| h[i]-l[i]).fold(0.,f64::max),tol);
    let pad = [sag+margin,sag+margin,margin];
    for i in 0..3 { l[i] -= pad[i]; h[i] += pad[i] }
    let at = |x: f64,y: f64| f.at([x,y,l[2]]);
    let rim = [at(l[0],l[1]),at(h[0],l[1]),at(h[0],h[1]),at(l[0],h[1])];
    let section = Profile {origin:rim[0],normal:screw.axis,names:Vec::new(),
        loops:vec![(0..4).map(|k| ProfileEdge::Line {a:rim[k],b:rim[(k+1)%4]}).collect()]};
    let half = 0.5*(0..3).map(|i| h[i]-l[i]).fold(0.,f64::max);
    let far = l[0].abs().max(h[0].abs()).dhypot(l[1].abs().max(h[1].abs()));
    Ok(Domain {solid:prism(&section,0.,h[2]-l[2])?,centre:f.at(std::array::from_fn(|i| 0.5*(l[i]+h[i]))),half,tall:Some((l[2],h[2],far))})
}

/// **The solids a candidate splits the box by**, the first leg's twist `screw` over `[a, b]`: a
/// plane a block standing on it, far wider than the box; a surface of revolution its solid over the
/// heights its source spans (a hyperboloid may enter two); a screwed line's sheet itself, reaching
/// past the box — each a solid one of whose faces lies on the candidate across the box, its seam
/// laid away from `seams`.
fn solids_of(c: Cutter,screw: Screw,[a,b]: [f64;2],domain: &Domain,seams: &[V],eps: f64,size: f64) -> Result<Vec<Brep>,Unbuilt> {
    let path = Trajectory::Twist(screw);
    let pose_at = |t: f64| path.pose(t).map_err(Unbuilt::from);
    // the disc `disc` screwed over the stretch `[s0, s1]` of the roll
    let screwed = |disc: &Profile,[s0,s1]: [f64;2]| {
        let rate = screw.advance/std::f64::consts::TAU;
        crate::build::screwed(disc,screw.origin,screw.axis,screw.ratio*(s1-s0),rate*(s1-s0))
    };
    let (centre,half) = (domain.centre,domain.half);
    let reach = 4.*half;
    Ok(vec![match c {
        Cutter::Plane((o,n)) => {
            let foot = sub(centre,scale(n,dot(sub(centre,o),n)));
            prism(&square(foot,n,reach),0.,reach)?
        }
        Cutter::Cylinder {o,axis,r,span} => seam_away(prism(&disc(add(o,scale(axis,span[0])),axis,r),0.,span[1]-span[0])?,o,axis,seams),
        Cutter::Revolved {o,axis,p,e,span} => return Ok(revolved(o,axis,p,e,span,eps)?),
        Cutter::Turned(t) => turned_solid(&t)?,
        Cutter::Contour {halves,span} => contour_solid(&halves,span)?,
        Cutter::Sheet {arc} => {
            // (along the slide past the whole domain both ways, so no rim but its sides stands in it)
            let mid = dot(sub(centre,arc.frame.o),arc.slide());
            contour_sheet(&arc,[mid-2.*half,mid+2.*half])?
        }
        Cutter::Ball {c,axis,r} => crate::build::sphere(c,axis,r)?,
        Cutter::ScrewedDisc {c,r} => { let first = pose_at(a)?; screwed(&disc(first.point(c),screw.axis,r),[a,b])? }
        Cutter::Pipe {c,n,r,span} => { let first = pose_at(span[0])?; screwed(&disc(first.point(c),first.vector(n),r),span)? }
        Cutter::Torus {c,axis,major,minor} => seam_away(crate::build::torus(c,axis,major,minor)?,c,axis,seams),
        Cutter::Oblique {c,a,r,d} => {
            // the disc's cylinder along `a` as high as the slide climbs it, sheared
            // `x ↦ x + ((x − c)·a) w` so that `a h` goes to `d`
            let h = dot(d,a);
            let w = sub(scale(d,1./h),a);
            let m = std::array::from_fn(|i| std::array::from_fn(|j| if i == j { 1. } else { 0. }+w[i]*a[j]));
            let shear = crate::affine::Affine::new(m,scale(w,-dot(c,a)))?;
            seam_away(prism(&disc(c,a,r),0.,h)?,c,a,seams).mapped(&shear)
        }
        Cutter::Patch {p,q} => patch(&screw,&pose_at,p,q,[a,b])?,
        Cutter::Screwed {p,e} => screwed_sheet(&screw,&pose_at,p,e,domain.tall.expect("a level edge's sheet is a screw's, its box tall"),size)?,
    }])
}

/// **The touches given** (#810, #780 R5): each between the faces of its two candidates' solids that
/// hold it, laid with no search for it — where both candidates were carried (`entered`, the solids
/// each entered among `solids`).
fn touches_given(touches: &[Touch],entered: &[Option<std::ops::Range<usize>>],solids: &[Brep],tol: f64)
    -> Vec<crate::boolean::Given> {
    touches.iter().filter_map(|t| {
        let ranges = t.cutters.map(|c| entered[c].clone());
        let [Some(r0),Some(r1)] = ranges else { return None };
        let points = t.samples();
        let holds = |s: &&Surface| points.iter().all(|&x| s.implicit(x).abs() <= 64.*tol);
        let surface = |r: std::ops::Range<usize>| solids[r].iter().flat_map(|b| &b.faces).map(|f| &f.surface).find(holds).cloned();
        // (points of touching alone: every face of the first's solids holding them, each against the
        // second's surface holding them — the rim the points lie on bounds two faces of it, and an
        // edge is read off given meetings only where every face it bounds has one)
        if t.curves.is_empty() {
            let other = surface(r1)?;
            let faces: Vec<Surface> = solids[r0].iter().flat_map(|b| &b.faces).map(|f| &f.surface).filter(holds).cloned().collect();
            return Some(faces.into_iter().map(|s| crate::boolean::Given {surfaces:[s,other.clone()],curves:Vec::new(),touches:t.touches.clone()}).collect())
        }
        Some(vec![crate::boolean::Given {surfaces:[surface(r0)?,surface(r1)?],curves:t.curves.clone(),touches:Vec::new()}])
    }).flatten().collect()
}

/// A box's eight corners.
pub(super) fn corners_of((lo,hi): (V,V)) -> Vec<V> {
    (0..8).map(|k| std::array::from_fn(|i| if (k>>i)&1 == 0 { lo[i] } else { hi[i] })).collect()
}

/// A cube holding `points` with a fifth of its size to spare each way: the box, its centre and half
/// its side.
/// **The roll's grain at the tolerance**: the stretch of `[a, b]` over which no corner of the tool's
/// box moves `tol`, read at 4096 poses (#942 F6) — two instants nearer than this are one as the
/// tolerance reads the sweep. The whole roll where nothing moves.
pub(crate) fn grain(tool: &Brep,path: &Trajectory,[a,b]: [f64;2],tol: f64) -> Result<f64,Unbuilt> {
    const LOOKS: usize = 4096;
    let corners = { let (lo,hi) = tool.bounds(); (0..8).map(|k| std::array::from_fn(|i| if k>>i & 1 == 0 { lo[i] } else { hi[i] })).collect::<Vec<V>>() };
    let step = (b-a)/LOOKS as f64;
    let mut fastest = 0f64;
    let mut last = path.pose(a)?;
    for k in 1..=LOOKS {
        let now = path.pose(if k == LOOKS { b } else { a+step*k as f64 })?;
        for &c in &corners { fastest = fastest.max(crate::util::space::distance(now.point(c),last.point(c))/step) }
        last = now;
    }
    Ok(if fastest > 0. { tol/fastest } else { b-a })
}

/// **The hair a trim reads a piece's sides at** (#880 Q1b, every road's since #942 F6): the larger of
/// four tolerances, above the sections' fit, and a millionth of the tool — a hair of a few
/// tolerances at a fine one leaves a point within it of the sweep within it of the tool over a
/// spread of poses its square root wide, more stretches than membership reads.
fn hair(tol: f64,size: f64) -> f64 { (4.*tol).max(1e-6*size) }

/// **How far a box holding a sweep stands clear of it**: a twentieth of the box's largest extent and
/// the tolerance, every box a road holds its sweep in (#942 F6: the screw's box had a twentieth and
/// `cube_holding`'s a tenth; at a tenth the drum screwed a short lead through most of a turn was
/// built wrong, `swept_closed::a_drum_screwed_a_short_lead_through_most_of_a_turn_closes`).
pub(super) fn room(extent: f64,tol: f64) -> f64 { 0.05*extent+tol }

pub(super) fn cube_holding<'a>(points: impl IntoIterator<Item=&'a V>,tol: f64) -> Result<(Brep,V,f64),Unbuilt> {
    let (blo,bhi) = points.into_iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(l,h),p|
        (std::array::from_fn(|k| l[k].min(p[k])),std::array::from_fn(|k| h[k].max(p[k]))));
    let centre: V = std::array::from_fn(|k| 0.5*(blo[k]+bhi[k]));
    let extent = (0..3).map(|k| bhi[k]-blo[k]).fold(0.,f64::max);
    let half = 0.5*extent+room(extent,tol);
    Ok((prism(&square(add(centre,[0.,0.,-half]),[0.,0.,1.],half),0.,2.*half)?,centre,half))
}

/// The candidates' solids that reach the domain `within`'s box: one standing clear of it parts none
/// of it, so its meetings with the others there are never sought.
fn clear_of<'a>(solids: &'a [Brep],within: &Brep,tol: f64) -> Vec<&'a Brep> {
    let (dlo,dhi) = within.bounds();
    solids.iter().filter(|s| { let (lo,hi) = s.bounds(); (0..3).all(|k| lo[k] <= dhi[k]+tol && hi[k] >= dlo[k]-tol) }).collect()
}

/// `arranged`, a failed build's arrangement written where `SOLVENT_SWEEP_DUMP` asks (#777 V5), to be
/// replayed alone and reduced (`swept::dump`).
#[allow(clippy::too_many_arguments)]
fn arranged_or_dumped(within: &Brep,reach: Option<&Brep>,refs: &[&Brep],given: &[crate::boolean::Given],swept: &Swept,
    blank: Option<&Brep>,tol: f64) -> Result<Brep,Unbuilt> {
    let out = arranged(within,reach,refs,given,swept,blank,tol);
    if let (Err(why),Some(dir)) = (&out,super::dump::dump_dir()) {
        match super::dump::dump(&dir,within,reach,refs,given,tol,&format!("{why:?}")) {
            Ok(()) => eprintln!("sweep: the failed build's arrangement written to {}",dir.display()),
            Err(e) => eprintln!("sweep: the failed build's arrangement not written: {e}"),
        }
    }
    out
}

/// **Whether the pipeline takes `tool` under `path` over `roll`, a motion no constant twist takes**
/// (#780 L3): the tool a convex solid of revolution whose contact reads round the pencil at a few
/// instants and whose centre's path comes back nowhere (`patches::admits`), read before anything is
/// built, as `reads` reads a constant twist's.
pub fn reads_general(tool: &Brep,path: &Trajectory,roll: [f64;2]) -> Result<(),Unbuilt> {
    if path.screw().is_some() { return Err(Unbuilt::Motion) }
    super::patches::admits(tool,path,[roll[0].min(roll[1]),roll[0].max(roll[1])])
}

/// How finely K2's sheet is fitted, as a part of the arrangement's tolerance: its touches with the
/// tool at the end poses are laid as edges of both within that tolerance.
const K2_FIT: f64 = 0.1;

/// How many poses over the roll the box holding a K2 sweep is read at.
const K2_POSES: usize = 128;

/// **`build` under a rigid motion no constant twist takes** (#780 L3, K2): the candidates are the
/// tool at each end pose, whole, and the sheet its contact sweeps over the roll (`patches`); every
/// meeting between them given (`patches::entered`); the box (or the blank) split by them at once and
/// classified by membership, as every other build is.
fn k2(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Brep,Unbuilt> {
    let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    // (a tool no convex solid of revolution, every face a plane and every edge a line, is the
    // general road's (#880 T1d): its contact read face by face and edge by edge at every instant;
    // a tool it cannot read either keeps K2's refusal)
    let revolved = match super::patches::Revolved::read(tool) {
        Ok(r) => r,
        Err(why @ Unbuilt::Curved {..}) => return match general_built(tool,path,[a,b],bar,tol,blank) {
            Err(Unbuilt::Curved {..}) => Err(why),
            built => built,
        },
        Err(why) => return Err(why),
    };
    let fit = super::patches::fitted(&revolved,path,[a,b],K2_FIT*tol)?;
    crate::util::meter::go_on()?;
    let ends = [a,b].map(|t| super::patches::posed(tool,&revolved,path,&fit,t));
    let [start,end] = [ends[0].clone()?,ends[1].clone()?];
    let (sheet,given) = super::patches::entered(&fit,&revolved,[&start,&end])?;
    candidates(tool,path,[a,b],bar,tol,blank,[start,end],vec![sheet],&given)
}

/// **A polyhedral tool under a motion no constant twist takes, built closed** (`swept::general`,
/// #880 T1d): its candidates — each piece of the contact's wall over each stretch of the roll, and the
/// tool at either end — trimmed by membership and judged (`trim::trim`), and cut from `blank` where
/// there is one.
fn general_built(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Brep,Unbuilt> {
    let (candidates,given) = super::general::candidates(tool,path,roll,tol)?;
    crate::util::meter::go_on()?;
    let size = tool.size();
    let trim = super::trim::Trim {resolution:size/32.,hair:hair(tol,size),reach:None};
    let swept = trimmed_or_dumped(tool,path,roll,&candidates,&given,bar,tol,trim)?;
    cut_blank(swept,blank,tol)
}

/// The candidates trimmed by membership (`trim::trim`), and where that is refused and
/// `SOLVENT_SWEEP_DUMP` names a directory, what the trim was given written there (`dump_trim`,
/// #942 F5), so it is run again apart from what made its candidates.
#[allow(clippy::too_many_arguments)]
fn trimmed_or_dumped(tool: &Brep,path: &Trajectory,roll: [f64;2],candidates: &[Brep],given: &[crate::boolean::Given],bar: f64,tol: f64,
    trim: super::trim::Trim) -> Result<Brep,Unbuilt> {
    let out = super::trim::trim(tool,path,roll,candidates,given,bar,tol,trim);
    if let (Err(why),Some(dir)) = (&out,super::dump::dump_dir()) {
        match super::dump::dump_trim(&dir,tool,path,roll,candidates,given,bar,tol,trim,&format!("{why:?}")) {
            Ok(()) => eprintln!("sweep: the refused trim written to {}",dir.display()),
            Err(e) => eprintln!("sweep: the refused trim not written: {e}"),
        }
    }
    Ok(out?.0)
}

/// **A ring under a motion no constant twist takes, built closed** (`swept::rings`, steps 6 and 7
/// under K2; not yet a class the dispatch admits): the ring where it starts and where it stops and
/// the tube each of its contact curves sweeps — buried pieces and the bridges across its folds
/// with the touching ones — their meetings with the ring at the end poses read in the tubes' own
/// domains and given, the box split by them at once and classified by membership.
pub fn ring(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    ring_built(tool,path,roll,bar,tol,None)
}

/// **A ring's sweep cut from `blank`**, as `cut_from` cuts any other's: the blank split by the
/// candidates `ring` splits its box by, all at once, each cell kept where it is no member of the
/// sweep. No closed solid is built and then subtracted.
pub fn ring_cut_from(blank: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    ring_built(tool,path,roll,bar,tol,Some(blank))
}

/// `ring` and `ring_cut_from`: the ring's candidates, arranged in a box holding the sweep or in
/// `blank`.
fn ring_built(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Brep,Unbuilt> {
    let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let clock = crate::util::clock::Instant::now();
    // (each step's time and what it read, where `SOLVENT_BREP_TIME` is set: #942 F4)
    let mut mark = (crate::util::clock::Instant::now(),super::rings::spent::now());
    let mut phase = |what: &str| if crate::timing() {
        eprintln!("ring: {what} in {:?}; {}",mark.0.elapsed(),super::rings::spent::since(mark.1));
        mark = (crate::util::clock::Instant::now(),super::rings::spent::now());
    };
    let ring = super::rings::Ring::read(tool)?;
    // **the roll cut where the contact changes shape** (#880 T1d, `rings::changes`): a fold pair born
    // or vanishing partway — the tumbling torus turned whole, and thin — each stretch fitted on its
    // own, its rows half the roll's grain clear of the cuts (where the pair has no width; the two rows
    // either side of a cut so within the tolerance of one another, #942 F6), and the tubes of every
    // stretch sewn across them into one candidate per tube
    let (scale,cuts) = super::rings::changes(&ring,path,[a,b])?;
    let eps = 0.5*grain(tool,path,[a,b],tol)?;
    let bounds: Vec<f64> = std::iter::once(a).chain(cuts.iter().copied()).chain(std::iter::once(b)).collect();
    let stretches: Vec<[f64;2]> = bounds.windows(2).enumerate().map(|(k,w)| {
        [if k == 0 { w[0] } else { w[0]+eps },if k+2 == bounds.len() { w[1] } else { w[1]-eps }]
    }).collect();
    if crate::debug() && !cuts.is_empty() { eprintln!("ring: the contact changes shape at {cuts:?}") }
    phase(&format!("the roll cut where the contact changes shape, {} cuts,",cuts.len()));
    let mut fits: Vec<super::rings::Fitted> = stretches.iter().map(|&sp| super::rings::fitted_over(&ring,path,[a,b],sp,scale,K2_FIT*tol))
        .collect::<Result<_,_>>()?;
    phase(&format!("{} stretches fitted",fits.len()));
    // **each fold pair's double curve** (#880 T1d, `rings::double_curve`): traced over the whole roll
    // from where a stretch first finds the outer leaves crossing, once a pair, and cut into every
    // stretch it passes (`rings::cut_along`), its tail dropped there
    let mut curves: Vec<Vec<(f64,f64,f64)>> = Vec::new();
    for seed in fits.iter().flat_map(|f| f.seeds.clone()) {
        // (a seed on a curve traced already: its point within the tolerance of it, #942 F6)
        if curves.iter().any(|c| super::rings::on_double_curve(&ring,path,c,seed[0],tol)) { continue }
        curves.push(super::rings::double_curve(&ring,path,[a,b],seed,tol)?);
    }
    phase(&format!("{} double curves traced",curves.len()));
    // (and the roll cut again at each swallowtail point, the contact's shape the same either side:
    // there the double curve's two halves meet, one in space, and a face holding both would be glued
    // to itself along the crease — each stretch about it refitted in two)
    let apexes: Vec<f64> = curves.iter().flat_map(|c| c.iter().skip_while(|p| !p.0.is_nan()).skip(1).map(|p| p.1))
        .filter(|t| t.is_finite() && *t > a && *t < b).collect();
    for &t in &apexes {
        let Some(k) = fits.iter().position(|f| f.roll[0] < t && f.roll[1] > t) else { continue };
        let [lo,hi] = fits[k].roll;
        let parts = [[lo,t],[t,hi]].map(|sp| super::rings::fitted_over(&ring,path,[a,b],sp,scale,K2_FIT*tol));
        let [p0,p1] = parts;
        fits.splice(k..=k,[p0?,p1?]);
    }
    phase(&format!("{} stretches refitted about swallowtail points",2*apexes.len()));
    if !curves.is_empty() { for fit in &mut fits { super::rings::cut_along(&ring,path,[a,b],fit,scale,&curves,tol)? } }
    phase("the double curves cut into the stretches");
    if crate::timing() { eprintln!("ring: fitted in {:?}",clock.elapsed()) }
    crate::util::meter::go_on()?;
    let [start,end] = [super::rings::posed(tool,&ring,path,&fits[0],a)?,super::rings::posed(tool,&ring,path,&fits[fits.len()-1],b)?];
    let poses = [path.pose(a)?,path.pose(b)?];
    let last = fits.len()-1;
    let mut pieces = Vec::new();
    let mut given = Vec::new();
    for (k,fit) in fits.iter().enumerate() {
        // (both end poses read on every stretch: a tube carried on can cross either far from it)
        let at = [Some((&start,poses[0])),Some((&end,poses[1]))];
        let (s,g) = super::rings::entered_at(fit,&ring,at,[k == 0,k == last],tol)?;
        if crate::debug() { eprintln!("ring: stretch {k} over {:?}: pieces {}..{}, faces {:?}",fit.roll,pieces.len(),pieces.len()+s.len(),s.iter().map(|b: &Brep| b.faces.len()).collect::<Vec<_>>()) }
        phase(&format!("stretch {k} entered at the end poses"));
        pieces.extend(s);
        given.extend(g);
    }
    let sheets = if fits.len() == 1 { pieces } else {
        // (sewn only along the rows at the cuts: the double curve's chords on its two leaves are
        // one curve in space, and left to the trim)
        let rows: Vec<f64> = fits.windows(2).map(|w| 0.5*(w[0].roll[1]+w[1].roll[0])).collect();
        // (a parameter on a cut's row: within its stretches' clearance of the cut, twice over)
        let across = |uv: crate::geom::Uv| rows.iter().position(|&r| (uv[1]-r).abs() <= 2.*eps);
        let sewn = crate::sew::sewn_open(&pieces,tol,&across).map_err(|e| Unbuilt::Failed(format!("a ring's tubes not sewn across the cuts: {}",e.msg)))?;
        let parts = crate::sew::parts(&sewn);
        if crate::debug() {
            let free = |b: &Brep| { let u = b.uses(); u.iter().filter(|x| x.len() == 1).count() };
            eprintln!("ring: {} pieces over {} stretches sewn into {} candidates, {} boundary edges",pieces.len(),fits.len(),parts.len(),free(&sewn));
        }
        parts
    };
    phase("the stretches sewn");
    if crate::timing() { eprintln!("ring: entered in {:?}",clock.elapsed()) }
    // **trimmed by membership where the roll is cut** (#880 T1d): its tubes met only where both come
    // near the sweep's boundary (`trim`), as the contact's road meets its sheets — a turned whole
    // ring's two tubes cross at under a degree inside the sweep, which the arrangement meets and
    // cannot, and a thin one's run a hair apart all round, each pair searched for nothing
    if !cuts.is_empty() {
        let size = tool.size();
        let trim = super::trim::Trim {resolution:size/32.,hair:hair(tol,size),reach:None};
        let candidates: Vec<Brep> = [start,end].into_iter().chain(sheets).collect();
        let swept = trimmed_or_dumped(tool,path,[a,b],&candidates,&given,bar,tol,trim)?;
        return cut_blank(swept,blank,tol)
    }
    candidates(tool,path,[a,b],bar,tol,blank,[start,end],sheets,&given)
}

/// K2's build from its candidates: the tool at each end pose and the sheets, their meetings
/// `given`, a box holding the sweep (or the blank) split by them at once and classified.
#[allow(clippy::too_many_arguments)]
fn candidates(tool: &Brep,path: &Trajectory,[a,b]: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>,[start,end]: [Brep;2],sheets: Vec<Brep>,
    given: &[crate::boolean::Given]) -> Result<Brep,Unbuilt> {
    // a box holding the sweep: the tool's box at poses over the roll
    let corners = corners_of(tool.bounds());
    let mut points: Vec<V> = Vec::new();
    for k in 0..=K2_POSES {
        let m = path.pose(a+(b-a)*k as f64/K2_POSES as f64)?;
        points.extend(corners.iter().map(|&p| m.point(p)));
    }
    if let Some(bl) = blank { points.extend(corners_of(bl.bounds())) }
    let (domain,..) = cube_holding(&points,tol)?;
    let within = blank.unwrap_or(&domain);
    let solids: Vec<Brep> = [start,end].into_iter().chain(sheets).collect();
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll:[a,b]};
    arranged_or_dumped(within,None,&clear_of(&solids,within,tol),given,&swept,blank,tol)
}

/// The domain `within` split by the candidates `refs` at once — within `reach` alone where there
/// is one (a sweep cut from a blank, #847: `boolean::arrangement_reached`) — each cell kept by
/// membership, the kept cells' boundary selected and judged: `build` from its arrangement on.
#[allow(clippy::too_many_arguments)]
fn arranged(within: &Brep,reach: Option<&Brep>,refs: &[&Brep],given: &[crate::boolean::Given],swept: &Swept,blank: Option<&Brep>,
    tol: f64) -> Result<Brep,Unbuilt> {
    let clock = crate::util::clock::Instant::now();
    let complex = crate::boolean::arrangement_reached(within,reach,refs,given,tol)?;
    if crate::timing() { eprintln!("closed: arranged in {:?}",clock.elapsed()) }
    crate::util::meter::go_on()?;
    // each cell classified by membership at the deepest of its points that settles (a cusp-thin
    // cell's stand near the swept boundary, and the first may not), and the boundary between the
    // cells kept — the members, or cut from a blank the rest — and the others the result: one
    // selection, no union, the candidates that bound nothing healed out of its faces
    let found = complex.cells()?;
    let cells = found.len();
    if crate::timing() { eprintln!("closed: {cells} cells found in {:?}",clock.elapsed()) }
    // a cell a face of the reach bounds from outside it is outside the reach, which holds the sweep:
    // no member of it, and kept with nothing read (the reach is the complex's operand 1)
    let outside = |c: usize| reach.is_some() && complex.in_operand(&complex.cell_sides(found,c),1) == Some(false);
    // each cell read on its own, across the cores (`util::par`, in the cells' order): its points
    // certified by its own exact point location, the first whose membership settles classifying it,
    // and those past it kept as witnesses the built solid is judged at
    let read = crate::util::par::indices(cells.saturating_sub(1),|i| -> Result<Option<(bool,Vec<V>)>,Unbuilt> {
        let c = i+1;
        crate::util::meter::go_on()?;
        if outside(c) { return Ok(Some((true,Vec::new()))) }
        let at = crate::util::clock::Instant::now();
        let points = complex.inside_points(c,4)?;
        if crate::timing() { eprintln!("closed: cell {c}'s points in {:?}",at.elapsed()) }
        let Some(&first) = points.first() else { return Ok(None) };
        match points.iter().enumerate().find_map(|(k,&x)| swept.member(x).ok().map(|m| (k,m))) {
            Some((k,member)) => Ok(Some((member != blank.is_some(),points[k+1..].to_vec()))),
            None => Err(Unbuilt::Unsettled {at:first}),
        }
    });
    let mut keep = vec![false;cells];
    let mut witnesses: Vec<V> = Vec::new();
    for (i,r) in read.into_iter().enumerate() {
        if let Some((kept,more)) = r? { keep[i+1] = kept; witnesses.extend(more) }
    }
    if crate::timing() { eprintln!("closed: {} cells classified in {:?}",cells,clock.elapsed()) }
    if blank.is_none() && !keep.iter().any(|&k| k) { return Err(Unbuilt::Failed("no cell of the arrangement is swept".into())) }
    let out = crate::boolean::selection(&complex,&|c| keep[c])?;
    if let Err(e) = out.check(tol) {
        if crate::debug() {
            // each face's loops, where the check failed: their areas and their reach in parameters
            for (fi,f) in out.faces.iter().enumerate() {
                for (li,l) in f.loops.iter().enumerate() {
                    let uv: Vec<[f64;2]> = l.iter().flat_map(|c| out.uv_ends(f,c)).collect();
                    let (lo,hi) = uv.iter().fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(l,h),p| ([l[0].min(p[0]),l[1].min(p[1])],[h[0].max(p[0]),h[1].max(p[1])]));
                    eprintln!("closed: face {fi} ({} {}{}) loop {li}: {} uses, area {:.4e}, from {lo:?} to {hi:?}",f.surface.kind(),f.name,
                        if f.reversed { ", reversed" } else { "" },l.len(),out.loop_area(f,l));
                }
            }
        }
        return Err(e.into())
    }
    // **judged by its definition before it is returned** (#777 V2): never returned built wrong
    judge(&out,swept,within,blank,&witnesses,tol)?;
    Ok(out)
}

/// How many points of the domain each way a built solid is judged at (`judge`).
const JUDGED: usize = 8;

/// **The built solid judged by the sweep's definition** (#777 V2, the design bible's fourth
/// principle: built correctly or refused with a witness, never built wrong): at seeded points of
/// the domain — a jittered grid over its box, and the cells' own points past the one each was
/// classified by (`witnesses`) — the solid's exact point location (`query::Located`, V1) read
/// against the swept set's membership (`member::Swept::member`): built alone, inside the solid
/// exactly where the point is a member; cut from a blank, inside exactly where the blank holds it
/// and it is no member. A point either reading cannot settle (on the boundary, or within the tool
/// mesh's sag of it) says nothing. A disagreement is a refusal carrying the point
/// (`Unbuilt::Wrong`). The solid's sense is judged too: a boundary turned inside out (its volume
/// below nothing) places points as the right one does, but holds none of them, and is refused at
/// the first member it so leaves out. Public, so what a build returned can be judged again apart
/// from it.
pub fn judge(built: &Brep,swept: &Swept,domain: &Brep,blank: Option<&Brep>,witnesses: &[V],tol: f64) -> Result<(),Unbuilt> {
    use crate::query::{Located,Place};
    use crate::util::meter::{self,Stage};
    let (lo,hi) = domain.bounds();
    let mut rng = crate::util::rng::Rng::new(0x5ee7);
    let mut points: Vec<V> = Vec::with_capacity(JUDGED*JUDGED*JUDGED+witnesses.len());
    for k in 0..JUDGED*JUDGED*JUDGED {
        let ijk = [k%JUDGED,(k/JUDGED)%JUDGED,k/(JUDGED*JUDGED)];
        points.push(std::array::from_fn(|a| lo[a]+(hi[a]-lo[a])*(ijk[a] as f64+rng.uniform(0.15,0.85))/JUDGED as f64));
    }
    points.extend_from_slice(witnesses);
    // (the grid is within the domain's box; a blank is the domain cut from, read as the blank)
    let solid = Located::new(built,tol);
    let blank = blank.map(|b| Located::new(b,tol));
    let readings = crate::util::par::map(&points,|&p| -> Option<(V,bool)> {
        if !meter::charge(Stage::Judge,1,|| (p,"the built solid".into())) { return None }
        let held = match solid.solid_place(p) { Place::In => true,Place::Out => false,Place::On => return None };
        // (outside a blank, nothing is held whatever membership says: it is not read there)
        let should = match &blank {
            None => swept.member(p).ok()?,
            Some(b) => match b.solid_place(p) { Place::In => !swept.member(p).ok()?,Place::Out => false,Place::On => return None },
        };
        (held != should).then_some((p,held))
    });
    if let Some((at,held)) = readings.into_iter().flatten().next() { return Err(Unbuilt::Wrong {at,held}) }
    // each face turned the way its solid lies: a hair out along its outward normal the solid's own
    // point location reads out, a hair in reads in (a face turned against its neighbours places
    // points as the right one does, but its volume is off: the ring's sweep came out at 3.38 of
    // 25.13 with every point placed right) — each face read on its own across the cores, the first
    // turned in the faces' order refused (#780 R4)
    let size = { let (lo,hi) = built.bounds(); crate::util::space::distance(lo,hi) };
    let hair = (1e-4*size).max(64.*tol);
    let turned = crate::util::par::indices(built.faces.len(),|fi| -> Option<V> {
        let f = &built.faces[fi];
        let [[u0,u1],[v0,v1]] = crate::boolean::face_parameters(built,fi);
        let p = (1..4).flat_map(|i| (1..4).map(move |j| [u0+(u1-u0)*i as f64/4.,v0+(v1-v0)*j as f64/4.]))
            .map(|uv| f.surface.point(uv)).find(|&p| solid.face_place(fi,p) == Place::In)?;
        let n = f.outward(p)?;
        let (outside,inside) = (add(p,scale(n,hair)),sub(p,scale(n,hair)));
        (solid.solid_place(outside) == Place::In && solid.solid_place(inside) == Place::Out).then_some(inside)
    });
    if let Some(at) = turned.into_iter().flatten().next() { return Err(Unbuilt::Wrong {at,held:false}) }
    // turned inside out: every point it seems to hold, it leaves out
    if crate::props::volume(built) < 0. {
        let inside = points.iter().copied().find(|&p| solid.solid_place(p) == Place::In)
            .or_else(|| witnesses.first().copied()).unwrap_or_else(|| built.vertices.first().map_or([0.;3],|v| v.p));
        return Err(Unbuilt::Wrong {at:inside,held:false})
    }
    Ok(())
}

/// The span of a cylinder's or a cone's face along its rulings (its surface's `v`), read off its
/// boundary edges.
pub(super) fn rulings_span(tool: &Brep,k: usize) -> (f64,f64) {
    let face = &tool.faces[k];
    face.loops.iter().flatten().flat_map(|c| {
        let e = &tool.edges[c.edge as usize];
        (0..=8).map(move |i| e.point(e.t[0]+(e.t[1]-e.t[0])*i as f64/8.,&tool.vertices))
    }).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),p| { let v = face.surface.inverse(p)[1]; (lo.min(v),hi.max(v)) })
}

/// How many steps a candidate's source is read at each way (along it, and across the roll) for
/// whether it carries any of the boundary.
const SITES: usize = 24;

/// The `i`th of `SITES + 1` fractions a source is read at, inside it and the roll (a site at a
/// source's own edge reading its neighbours' boundary, not its own).
fn within(i: usize) -> f64 { (i as f64+0.5)/(SITES+1) as f64 }

/// Whether a candidate carries some of the swept boundary: at one of its `sites` (its source's
/// points carried over the roll, each with the candidate's normal there), membership a hair to one
/// side differs from a hair to the other, or cannot be settled. A candidate carries boundary only
/// within its source's image, so that is where it is read. Sampled: a candidate whose boundary
/// falls between the sites is dropped, and the cell it would have bounded reads both ways.
fn carries(swept: &Swept,sites: &[(V,V)],half: f64) -> bool {
    let hair = 1e-5*half;
    sites.iter().any(|&(p,normal)| match (swept.member(add(p,scale(normal,hair))),swept.member(sub(p,scale(normal,hair)))) {
        (Ok(x),Ok(y)) => x != y,
        _ => true,
    })
}

/// The closed solid a polyhedral tool sweeps under a slide (`closed`'s, refused for any other
/// motion).
pub fn slid(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    if !path.screw().is_some_and(|s| s.is_slide()) { return Err(Unbuilt::Motion) }
    closed(tool,path,roll,bar,tol)
}

/// **The sheet the line through `p` along unit `e` (square to the axis) screws into, across the
/// box** — the box standing over the heights `[z0, z1]` along the axis and reaching `far` from it
/// (`tall`): one open face, the line's stretch reaching past the box both ways screwed over the
/// roll that carries it from below the box to above it, a margin past each, so its rims (helices
/// farther from the axis than the box reaches) and its ends (rulings below and above it) all lie
/// outside, and within the box it parts each level along the line screwed there. A sheet has no
/// inside to classify, so a cell reaching round its end is no question; it is the face of a level
/// triangle on the line screwed (`build::screwed`), the triangle's other faces left off.
fn screwed_sheet(screw: &crate::trajectory::Screw,pose_at: &dyn Fn(f64) -> Result<crate::trajectory::Pose,Unbuilt>,
    p: V,e: V,(z0,z1,far): (f64,f64,f64),size: f64) -> Result<Brep,Unbuilt> {
    let (c,axis) = (screw.origin,screw.axis);
    let rate = screw.advance/std::f64::consts::TAU;
    // the line's foot nearest the axis, and the way from the axis to it (square to the line where
    // the line meets the axis)
    let foot = sub(p,scale(e,dot(sub(p,c),e)));
    let off = { let r = sub(foot,c); sub(r,scale(axis,dot(r,axis))) };
    let away = if norm(off) > 1e-9*(1.+size) { unit(off) } else { unit(cross(axis,e)) };
    let reach = 1.25*far+size;
    // the roll over which the line climbs from below the box to above it, a margin past each
    let margin = 0.1*(z1-z0);
    let (s0,s1) = { let (x,y) = (screw.time_to(p,z0-margin),screw.time_to(p,z1+margin)); (x.min(y),x.max(y)) };
    let first = pose_at(s0)?;
    let corners = [add(foot,scale(e,-reach)),add(foot,scale(e,reach)),add(foot,scale(away,reach))].map(|x| first.point(x));
    let triangle = Profile {origin:corners[0],normal:axis,names:vec![vec!["sheet".into(),String::new(),String::new()]],
        loops:vec![(0..3).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%3]}).collect()]};
    let sheet = crate::build::screwed(&triangle,c,axis,screw.ratio*(s1-s0),rate*(s1-s0))?;
    Ok(only_sheet(sheet))
}

/// **The patch a segment leaning to a screw's axis sweeps over the roll `[a, b]`** (#777 M5): one
/// open face of the ruled helicoid, the segment lengthened a twentieth past each end and the roll
/// a twentieth past each of its, so its rim stands just past the candidates that seal it, inside
/// the domain — a sheet bounded inside a cell, whose meetings stop partway across the faces it
/// meets. It is the face of a triangle on the segment screwed (`build::screwed`), the triangle's
/// other faces left off.
pub(crate) fn patch(screw: &crate::trajectory::Screw,pose_at: &dyn Fn(f64) -> Result<crate::trajectory::Pose,Unbuilt>,p: V,q: V,[a,b]: [f64;2])
    -> Result<Brep,Unbuilt> {
    let d = sub(q,p);
    let (p,q) = (sub(p,scale(d,PAST)),add(q,scale(d,PAST)));
    let delta = PAST*(b-a);
    let (s0,s1) = (a-delta,b+delta);
    let first = pose_at(s0)?;
    let across = { let w = cross(d,screw.axis); if norm(w) > 1e-12*norm(d) { unit(w) } else { unit(cross(d,[1.,0.,0.])) } };
    let corners = [p,q,add(p,scale(across,0.5*norm(d)))].map(|x| first.point(x));
    let normal = unit(cross(sub(corners[1],corners[0]),sub(corners[2],corners[0])));
    let triangle = Profile {origin:corners[0],normal,names:vec![vec!["sheet".into(),String::new(),String::new()]],
        loops:vec![(0..3).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%3]}).collect()]};
    let rate = screw.advance/std::f64::consts::TAU;
    let sheet = crate::build::screwed(&triangle,screw.origin,screw.axis,screw.ratio*(s1-s0),rate*(s1-s0))?;
    Ok(only_sheet(sheet))
}

/// The hyperboloid the line through `p` along unit `e` sweeps turned about the unit `axis` through
/// `o`, as its meridian: the throat radius, the height of the throat along the axis, and the lean
/// `k`, the radius at height `z` being `√(throat² + k² (z − height)²)` (a double cone where the
/// throat is none).
pub(super) fn meridian(o: V,axis: V,p: V,e: V) -> (f64,f64,f64) {
    let r = sub(p,o);
    let (r0,ez) = (sub(r,scale(axis,dot(r,axis))),dot(e,axis));
    let ep = sub(e,scale(axis,ez));
    // the radial offset as the line runs: |r0 + s e⊥|², least at s* = −r0·e⊥ / |e⊥|²
    let s = -dot(r0,ep)/dot(ep,ep);
    (norm(add(r0,scale(ep,s))),dot(r,axis)+s*ez,norm(ep)/ez.abs())
}

/// **The solid the line through `p` along unit `e` bounds turned about the unit `axis` through `o`
/// over the heights `span`**, held exactly: its meridian, `√(throat² + k² (z − height)²)`, closed
/// down the axis at both heights and turned once. A hyperboloid's meridian is one rational
/// quadratic (`meridian_arc`); a cone's a line, a solid on either side of its apex where the span
/// holds it.
pub fn revolved(o: V,axis: V,p: V,e: V,[z0,z1]: [f64;2],eps: f64) -> Result<Vec<Brep>,String> {
    let m = meridian(o,axis,p,e);
    let (throat,height,_) = m;
    if throat <= eps {
        // a cone: a frustum on each side of the apex the span reaches (its end there a point)
        let mut out = Vec::new();
        for (lo,hi) in [(z0,z1.min(height)),(z0.max(height),z1)] {
            if !(hi-lo > eps) { continue }
            let snap = |r: f64| if r <= eps { 0. } else { r };
            let (a,b) = (snap(radius(m,lo)),snap(radius(m,hi)));
            if a == 0. && b == 0. { continue }
            out.push(crate::build::cone(add(o,scale(axis,lo)),axis,a,b,hi-lo)?);
        }
        return Ok(out)
    }
    let f = Frame::about(o,axis);
    let at = |r: f64,z: f64| add(add(o,scale(f.x,r)),scale(axis,z));
    let (a,b) = ([radius(m,z0),z0],[radius(m,z1),z1]);
    let profile = Profile {origin:o,normal:f.y,names:Vec::new(),loops:vec![vec![
        ProfileEdge::Line {a:at(0.,a[1]),b:at(a[0],a[1])},meridian_arc(&at,m,a,b)?,
        ProfileEdge::Line {a:at(b[0],b[1]),b:at(0.,b[1])},ProfileEdge::Line {a:at(0.,b[1]),b:at(0.,a[1])}]]};
    Ok(vec![crate::build::revolve(&profile,o,axis,std::f64::consts::TAU)?])
}

/// **How far past its source's ends a candidate bounded inside the domain reaches**, as a fraction
/// of the source's stretch: a patch past its segment and its roll (#777 M5), a turned contact's
/// meridian past its face's span (#780 C3, the face's own surface at the end poses half as far), so
/// no end of one stands on another's.
const PAST: f64 = 0.05;

/// Where the line through `o` along the unit `a` passes nearest the line through `c` along the
/// unit `z`, as its distance along `a` from `o`; none where the two run parallel.
pub(super) fn foot_on(o: V,a: V,c: V,z: V) -> Option<f64> {
    let (b,w) = (dot(a,z),sub(o,c));
    let den = 1.-b*b;
    (den > 1e-12).then(|| (b*dot(w,z)-dot(w,a))/den)
}

/// **The solid a turned contact's meridian bounds** (#780 C3, `Cutter::Turned`): the meridian from
/// its span's start to its end, closed down the turn's axis by level lines at both ends, turned once
/// about the axis — exact, its side the surface of revolution of the meridian itself
/// (`geom::Curve::Turned`). A sharp circle's meridian is closed already (#780 L2): turned once, a
/// ring.
fn turned_solid(t: &crate::turned::Turned) -> Result<Brep,String> {
    let g = t.frame;
    // a closed meridian, a sharp circle's (#780 L2): turned once, a ring
    if t.period().is_some() {
        let profile = Profile {origin:g.o,normal:g.y,names:Vec::new(),loops:vec![vec![ProfileEdge::Exact(Curve::Turned(std::sync::Arc::new(*t)))]]};
        return crate::build::revolve(&profile,g.o,g.z,std::f64::consts::TAU)
    }
    let [a,b] = t.span.map(|v| { let l = g.local(t.d2(v).0); [l[0],l[2]] });
    let at = |r: f64,z: f64| g.at([r,0.,z]);
    let profile = Profile {origin:g.o,normal:g.y,names:Vec::new(),loops:vec![vec![
        ProfileEdge::Line {a:at(0.,a[1]),b:at(a[0],a[1])},ProfileEdge::Exact(Curve::Turned(std::sync::Arc::new(*t))),
        ProfileEdge::Line {a:at(b[0],b[1]),b:at(0.,b[1])},ProfileEdge::Line {a:at(0.,b[1]),b:at(0.,a[1])}]]};
    crate::build::revolve(&profile,g.o,g.z,std::f64::consts::TAU)
}

/// **The solid a slid torus's contact sweeps a cylinder of** (#805, `Cutter::Contour`): the region
/// within the contact's apparent contour, its two halves read in the plane through the torus's
/// centre square to the slide, extruded along the slide over `[h0, h1]` — exact, its side the
/// cylinder over the contour (`Surface::Extrusion` of `geom::Curve::Slid`).
pub(crate) fn contour_solid(halves: &[crate::slid::Slid;2],[h0,h1]: [f64;2]) -> Result<Brep,String> {
    let profile = Profile {origin:halves[0].frame.o,normal:halves[0].slide(),names:Vec::new(),
        loops:vec![halves.iter().map(|h| ProfileEdge::Exact(Curve::Slid(std::sync::Arc::new(*h)))).collect()]};
    prism(&profile,h0,h1)
}

/// **A solid of revolution about the unit `axis` through `o`, its seam turned away from the points
/// `touched`** (#810): turned about its own axis — the same solid — so its seam meridian stands in
/// the widest gap between the angles round the axis at which they stand, where it is laid at
/// nothing given (a touch's line along a candidate's seam is an edge lying along a touch, which no
/// arrangement holds by rounding). Left as it is where nothing is touched beside its axis.
pub(super) fn seam_away(solid: Brep,o: V,axis: V,touched: &[V]) -> Brep {
    let f = Frame::about(o,axis);
    let angles: Vec<f64> = touched.iter().filter_map(|&p| {
        let l = f.local(p);
        (l[0].dhypot(l[1]) > 1e-9*(1.+norm(sub(p,o)))).then(|| l[1].datan2(l[0]))
    }).collect();
    if angles.is_empty() { return solid }
    let (at,gap) = super::sector::widest(angles.into_iter(),std::f64::consts::TAU);
    solid.moved(&crate::geom::Rigid::turn(o,axis,at+0.5*gap))
}

/// A solid's face named `sheet` alone, its other faces left off: an open sheet the arrangement takes
/// as a candidate (`screwed_sheet`, `patch`, `contour_sheet`).
fn only_sheet(mut solid: Brep) -> Brep {
    solid.faces.retain(|f| f.name == "sheet");
    solid.compacted()
}

/// **The sheet a slid torus band's contact sweeps** (#810, `Cutter::Sheet`): the cylinder over the
/// arc `arc` of the contour, extruded along the slide over `[h0, h1]` — its one face, the other
/// faces of the prism on the arc and its chord left off, as a patch is (`patch`).
pub(crate) fn contour_sheet(arc: &crate::slid::Slid,[h0,h1]: [f64;2]) -> Result<Brep,String> {
    let [a,b] = arc.span;
    let (p,q) = (arc.d2(a).0,arc.d2(b).0);
    let profile = Profile {origin:arc.frame.o,normal:arc.slide(),names:vec![vec!["sheet".into(),String::new()]],
        loops:vec![vec![ProfileEdge::Exact(Curve::Slid(std::sync::Arc::new(*arc))),ProfileEdge::Line {a:q,b:p}]]};
    let sheet = prism(&profile,h0,h1)?;
    Ok(only_sheet(sheet))
}

/// **A touch known by construction** (#810, #780 R5): two candidates, by their index among the
/// build's (`cutters`), and the curves that are the whole meeting of the faces of their solids that
/// hold them (`boolean::Given`) — or, with none, the points they touch at and nowhere else (#942
/// F3: a tool's face at an end pose meeting the sheet its neighbour's contact sweeps where that
/// contact crosses into it, every face of its solid then given those points).
#[derive(Clone,Debug)]
struct Touch { cutters: [usize;2],curves: Vec<Curve>,touches: Vec<V> }

impl Touch {
    /// Points along its curves, over each one's own span (a line about its point), or its points of
    /// touching where it has no curves.
    fn samples(&self) -> Vec<V> {
        if self.curves.is_empty() { return self.touches.clone() }
        self.curves.iter().flat_map(|c| {
            let [a,b] = match c {
                Curve::Line {..} => [-1.,1.],
                _ => c.exact_span().or(c.period().map(|p| [0.,p])).unwrap_or([0.,1.]),
            };
            (0..=8).map(move |i| c.point(a+(b-a)*i as f64/8.))
        }).collect()
    }
}

/// **A band of a torus's tube** (#810, #780 R5): a torus's face that is no whole ring, bounded by
/// two latitudes of its tube's outer half — `v` round the tube in `[−π/2, π/2]`, read in the
/// torus's frame `frame` from its middle circle — each a smooth seam into its neighbour, and
/// curving out from its tube's middle (a round, not a fillet): a bull-nose's round, a rounded
/// drum's corner. Whether each latitude's neighbour is ruled (a cylinder or a cone, whose ruling
/// where a contact crosses the seam sweeps the plane the contour's sheet touches), low then high.
#[derive(Clone,Copy,Debug)]
struct Band { frame: Frame,big: f64,small: f64,v: [f64;2],ruled: [bool;2] }

/// Where a band's contact under a slide lies on it (`Band::contact`).
#[derive(Clone,Copy,Debug,PartialEq)]
enum BandContact {
    /// Nowhere on the band.
    None,
    /// The whole outer contact: the band reaches past it both ways.
    Whole,
    /// One arc, about the angle `centre` round the axis and `half` either side of it, ending on the
    /// latitude `v[seam]` at the points it crosses the seam into the neighbour's contact.
    Arc { centre: f64,half: f64,seam: usize },
}

impl Band {
    /// The band face `k` of `tool` is (`uses` the tool's edges' uses), or the first reason it is
    /// none, by name: an edge that is no latitude or seam, a sharp rim (its fan not entered yet), a
    /// face reaching into the tube's inner half or curving in.
    fn of(tool: &Brep,k: usize,uses: &[Vec<(usize,usize,usize)>]) -> Result<Band,Unbuilt> {
        let face = &tool.faces[k];
        let Surface::Torus(frame,big,small) = face.surface else { unreachable!("a band is a torus's face") };
        let (big,small) = (big.abs(),small.abs());
        let z = unit(frame.z);
        let seams = face.seams();
        let mut lats: Vec<(f64,usize)> = Vec::new();
        for c in face.loops.iter().flatten() {
            if seams.contains(&c.edge) { continue }
            let edge = &tool.edges[c.edge as usize];
            let mid = edge.point(0.5*(edge.t[0]+edge.t[1]),&tool.vertices);
            let at = Unbuilt::Curved {at:mid};
            // a latitude: a circle about the torus's axis
            let EdgeCurve::Curve(Curve::Circle(cf,_)) = &edge.curve else { return Err(at) };
            if norm(cross(unit(cf.z),z)) > 1e-9 || norm(cross(sub(cf.o,frame.o),z)) > 1e-9*(1.+big) { return Err(at) }
            let Some(&(other,..)) = uses[c.edge as usize].iter().find(|u| u.0 != k) else { return Err(at) };
            // a smooth seam: the neighbour's normal the band's there (a sharp rim's fan is not
            // entered yet, #810's negative control)
            let (n1,n2) = (outward(face,mid),outward(&tool.faces[other],mid));
            if !smooth(n1,n2) { return Err(at) }
            let l = frame.local(mid);
            lats.push((l[2].datan2(l[0].dhypot(l[1])-big),other));
        }
        let lo = lats.iter().copied().min_by(|a,b| a.0.total_cmp(&b.0));
        let hi = lats.iter().copied().max_by(|a,b| a.0.total_cmp(&b.0));
        let (Some(lo),Some(hi)) = (lo,hi) else { return Err(Unbuilt::Curved {at:frame.o}) };
        let half = std::f64::consts::FRAC_PI_2+1e-9;
        if !(hi.0-lo.0 > 1e-9) || lo.0 < -half || hi.0 > half { return Err(Unbuilt::Curved {at:frame.o}) }
        let ruled = |f: usize| matches!(tool.faces[f].surface,Surface::Cylinder(..) | Surface::Cone(..));
        let band = Band {frame,big,small,v:[lo.0,hi.0],ruled:[ruled(lo.1),ruled(hi.1)]};
        // curving out from the tube's middle: its outward normal there the tube's own
        let (x,n) = band.at(0.,0.5*(lo.0+hi.0));
        if dot(outward(face,x),n) <= 0. { return Err(Unbuilt::Curved {at:x}) }
        // every latitude one of the two, each with one neighbour all round
        if lats.iter().any(|&(v,f)| !((v-lo.0).abs() <= 1e-9 && f == lo.1 || (v-hi.0).abs() <= 1e-9 && f == hi.1)) {
            return Err(Unbuilt::Curved {at:frame.o})
        }
        Ok(band)
    }
    /// The band's surface point at `u` round the axis and `v` round the tube, and its outward unit
    /// normal there, in the tool's space.
    fn at(&self,u: f64,v: f64) -> (V,V) {
        let ((su,cu),(sv,cv)) = (u.dsin_cos(),v.dsin_cos());
        let q = self.big+self.small*cv;
        (self.frame.at([q*cu,q*su,self.small*sv]),self.frame.dir([cv*cu,cv*su,sv]))
    }
    /// **Where its contact under a slide along the unit `d` lies on it**: the outer contact,
    /// `tan v(u) = w(u) = −(d_x cos u + d_y sin u)/d_z` in its frame, reaches `±a = ±|d_xy|/|d_z|`;
    /// the band holds `tan v_lo ≤ w ≤ tan v_hi`. Nowhere, the whole contact, or one arc about the
    /// least `w` (the high seam crossed) or the greatest (the low) — refused by name where it is two
    /// arcs (both seams crossed), where the slide runs square to the axis (read as circles,
    /// elsewhere), or where a crossed seam's neighbour is no cylinder or cone.
    fn contact(&self,d: V) -> Result<BandContact,Unbuilt> {
        let l = self.frame.dir_local(unit(d));
        if !(l[2].abs() > 1e-9) { return Err(Unbuilt::Curved {at:self.frame.o}) }
        let a = l[0].dhypot(l[1])/l[2].abs();
        // the angle of the least `w`, where `d_xy` points with `d_z` above nought, else opposite
        let least = l[1].datan2(l[0])+if l[2] < 0. { std::f64::consts::PI } else { 0. };
        let tan = |v: f64| if v.abs() >= std::f64::consts::FRAC_PI_2-1e-12 { v.signum()*f64::INFINITY } else { v.dtan() };
        let (lo,hi) = (tan(self.v[0]),tan(self.v[1]));
        let (crosses_lo,crosses_hi) = (lo > -a,hi < a);
        if hi < -a || lo > a { return Ok(BandContact::None) }
        let (centre,half,seam) = match (crosses_lo,crosses_hi) {
            (false,false) => return Ok(BandContact::Whole),
            (true,true) => return Err(Unbuilt::Curved {at:self.at(least,self.v[1]).0}),
            // `w = −a cos(u − least)` at most `tan v_hi`
            (false,true) => (least,(-hi/a).clamp(-1.,1.).dacos(),1),
            // at least `tan v_lo`, about the greatest
            (true,false) => (least+std::f64::consts::PI,(lo/a).clamp(-1.,1.).dacos(),0),
        };
        // (an arc all but the whole contact: its ends all but meeting)
        if half >= 0.9*std::f64::consts::PI { return Err(Unbuilt::Curved {at:self.frame.o}) }
        if !self.ruled[seam] { return Err(Unbuilt::Curved {at:self.at(least,self.v[seam]).0}) }
        Ok(BandContact::Arc {centre,half,seam})
    }
    /// Its outer contact's arc under the slide along unit `d`, `half` either side of the angle
    /// `centre` round the axis, read as its contour (`slid::Slid`, in a frame turned to `centre`).
    fn arc(&self,d: V,centre: f64,half: f64) -> crate::slid::Slid {
        let (s,c) = centre.dsin_cos();
        let g = self.frame;
        let frame = Frame {o:g.o,x:add(scale(g.x,c),scale(g.y,s)),y:sub(scale(g.y,c),scale(g.x,s)),z:g.z};
        let t = (0.5*half).dtan();
        crate::slid::Slid {frame,contact:crate::slid::Contact {big:self.big,small:self.small,d:frame.dir_local(unit(d)),side:1.},
            span:[-t,t],contour:false}
    }
}

/// Whether a face is bounded by its seams alone (each edge of its loops used by it on both sides,
/// `Face::seams`): a whole ring.
fn whole(face: &crate::topo::Face) -> bool {
    let seams = face.seams();
    face.loops.iter().flatten().all(|c| seams.contains(&c.edge))
}

/// **Where a ring slid along the unit `d` folds** (#805, the fold locus): the torus about `g`'s axis,
/// its tube's middle radius `big` and radius `small`, its contacts' apparent contours read in the
/// angle chart (`slid::Slid`). The first point of a contact where its contour is not proved regular
/// (`Slid::folded`: the inner contour cusps once the slide leans `β` from the axis with
/// `tan² β ≥ (R − r)/r`), or where a line along the slide through it is not proved to meet the
/// torus there alone (`Slid::meets_again`, which comes only past the fold) — or none.
fn slid_fold(g: &Frame,big: f64,small: f64,d: V) -> Option<V> {
    [1.,-1.].into_iter().flat_map(|side| crate::slid::halves(*g,big,small,unit(d),side,false)).find_map(|h| {
        Some(h.d2(h.folded().or_else(|| h.meets_again())?).0)
    })
}

/// The radius `√(throat² + k² (z − height)²)` of the meridian `(throat, height, k)` at height `z`.
fn radius((throat,height,k): (f64,f64,f64),z: f64) -> f64 { (throat*throat+k*k*(z-height)*(z-height)).sqrt() }

/// How the nearest or farthest of a convex tool's slice about the axis runs with the height `z`
/// along it: through the slice's corner on an edge (`ρ² = throat² + k² (z − height)²`, the edge's
/// meridian), along the slice's side on a face (`ρ = α z + β`, the distance from the axis to the
/// face's line in the slice, the face's contact line), or on the axis itself (the slice holds it).
#[derive(Clone,Copy,Debug,PartialEq)]
enum Reach { Edge(usize),Face(usize),Axis }

/// **The closed solid a convex polyhedral tool sweeps turned whole about the unit `axis` through
/// `c`**: its solid of revolution. Each slice square to the axis is convex, so the radii it holds
/// are one span `[ρ_min(z), ρ_max(z)]`, and the sweep is that profile turned once. `ρ_max` is
/// reached at a slice's corner, on an edge: a hyperbola in `(ρ, z)` (the edge's meridian), a line
/// where the edge meets the axis or runs along it. `ρ_min` is zero where the slice holds the axis,
/// else reached on a side (the face's contact line, a line) or at a corner. Which one reaches is
/// fixed between the heights where two of them reach as far (their `ρ²`, each quadratic in `z`,
/// equal: a corner passing the foot of a side's perpendicular among them, where the two touch) or
/// the axis crosses a face, so the profile is cut there, each stretch read at its middle, and
/// turned whole (`build::revolve`): exact, its hyperbolas rational quadratics.
fn turned_whole(faces: &[(V,V)],edges: &[(V,V,[usize;2])],c: V,axis: V,eps: f64,tol: f64) -> Result<Brep,Unbuilt> {
    let f = Frame::about(c,axis);
    let height = |p: V| dot(sub(p,c),axis);
    let mut heights: Vec<f64> = edges.iter().flat_map(|&(p,q,_)| [height(p),height(q)]).collect();
    heights.sort_by(f64::total_cmp);
    heights.dedup_by(|x,y| (*x-*y).abs() <= eps);
    if heights.len() < 2 { return Err(Unbuilt::Failed("the tool has no height along the axis".into())) }
    // each edge's meridian (a line along the axis: a cylinder, its lean none)
    let meridians: Vec<(f64,f64,f64)> = edges.iter().map(|&(p,q,_)| {
        let e = unit(sub(q,p));
        if norm(cross(e,axis)) <= 1e-12 {
            let r = sub(p,c);
            (norm(sub(r,scale(axis,dot(r,axis)))),0.,0.)
        } else { meridian(c,axis,p,e) }
    }).collect();
    // each face's signed distance from the axis to its line in a slice, `α z + β` (positive where the
    // axis stands outside the face), none for a face square to the axis
    let lines: Vec<Option<(f64,f64)>> = faces.iter().map(|&(o,n)| {
        let across = norm(sub(n,scale(axis,dot(n,axis))));
        (across > 1e-12).then(|| (dot(n,axis)/across,dot(n,sub(c,o))/across))
    }).collect();
    // `ρ²` as a quadratic in `z`: its coefficients of `z²`, `z` and one
    let squared = |r: Reach| -> [f64;3] { match r {
        Reach::Edge(i) => { let (t,h,k) = meridians[i]; [k*k,-2.*k*k*h,t*t+k*k*h*h] }
        Reach::Face(f) => { let (al,be) = lines[f].unwrap(); [al*al,2.*al*be,be*be] }
        Reach::Axis => [0.;3],
    } };
    let rho = |r: Reach,z: f64| -> f64 { match r {
        Reach::Edge(i) => radius(meridians[i],z),
        Reach::Face(f) => { let (al,be) = lines[f].unwrap(); (al*z+be).abs() }
        Reach::Axis => 0.,
    } };
    // each stretch of height with what reaches farthest and what nearest over it
    let mut stretches: Vec<([f64;2],Reach,Reach)> = Vec::new();
    for w in heights.windows(2) {
        let (z0,z1) = (w[0],w[1]);
        let mid = 0.5*(z0+z1);
        // the edges the slab's slices cut (their corners) and the faces (their sides, each between
        // two of those edges)
        let cut: Vec<usize> = (0..edges.len()).filter(|&i| {
            let (h0,h1) = (height(edges[i].0),height(edges[i].1));
            h0.min(h1) < mid && h0.max(h1) > mid
        }).collect();
        let sides: Vec<(usize,[usize;2])> = (0..faces.len()).filter(|&f| lines[f].is_some()).filter_map(|f| {
            let on: Vec<usize> = cut.iter().copied().filter(|&i| edges[i].2.contains(&f)).collect();
            (on.len() == 2).then(|| (f,[on[0],on[1]]))
        }).collect();
        // where the reach may change: two candidates' `ρ²` equal (a root, or where they touch), or
        // the axis crossing a face's line
        let mut cuts = vec![z0,z1];
        let candidates: Vec<Reach> = cut.iter().map(|&i| Reach::Edge(i)).chain(sides.iter().map(|&(f,_)| Reach::Face(f))).collect();
        let inside = |z: f64| z > z0+eps && z < z1-eps;
        for (i,&r) in candidates.iter().enumerate() {
            if let Reach::Face(f) = r { let (al,be) = lines[f].unwrap(); if al != 0. { cuts.push(-be/al) } }
            if let Reach::Edge(e) = r { let (t,h,_) = meridians[e]; if t <= eps { cuts.push(h) } }
            for &s in &candidates[i+1..] {
                let ([a1,b1,c1],[a2,b2,c2]) = (squared(r),squared(s));
                let (qa,qb,qc) = (a1-a2,b1-b2,c1-c2);
                // either root, and where they touch (a double root)
                if qa.abs() > 1e-14*(qb.abs()+qc.abs()) { cuts.push(-qb/(2.*qa)) }
                cuts.extend(crate::util::roots::quadratic(qa,qb,qc));
            }
        }
        cuts.retain(|&z| z.is_finite() && (z == z0 || z == z1 || inside(z)));
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|x,y| (*x-*y).abs() <= eps);
        for v in cuts.windows(2) {
            let z = 0.5*(v[0]+v[1]);
            // the corners and sides of the slice at `z`
            let corner = |i: usize| -> [f64;2] {
                let (p,q,_) = edges[i];
                let (hp,hq) = (height(p),height(q));
                let x = add(p,scale(sub(q,p),(z-hp)/(hq-hp)));
                let r = sub(x,c);
                [dot(r,f.x),dot(r,f.y)]
            };
            let far = cut.iter().copied().max_by(|&i,&j| rho(Reach::Edge(i),z).total_cmp(&rho(Reach::Edge(j),z)))
                .ok_or_else(|| Unbuilt::Failed("a slab of the tool cuts no edge".into()))?;
            let holds = sides.iter().all(|&(f,_)| { let (al,be) = lines[f].unwrap(); al*z+be <= 0. });
            let near = if holds { Reach::Axis } else {
                // the nearest feature of the slice: a side where the axis's foot falls within it, else
                // a corner
                let mut best = (f64::INFINITY,Reach::Axis);
                for &(f,[i,j]) in &sides {
                    let (u,v) = (corner(i),corner(j));
                    let d = [v[0]-u[0],v[1]-u[1]];
                    let t = -(u[0]*d[0]+u[1]*d[1])/(d[0]*d[0]+d[1]*d[1]);
                    let (r,reach) = if t <= 0. { (rho(Reach::Edge(i),z),Reach::Edge(i)) }
                        else if t >= 1. { (rho(Reach::Edge(j),z),Reach::Edge(j)) }
                        else { (rho(Reach::Face(f),z),Reach::Face(f)) };
                    if r < best.0 { best = (r,reach) }
                }
                best.1
            };
            stretches.push(([v[0],v[1]],Reach::Edge(far),near));
        }
    }
    // each side's stretches run together where one candidate reaches over them
    let runs = |pick: &dyn Fn(&([f64;2],Reach,Reach)) -> Reach| -> Vec<([f64;2],Reach)> {
        let mut out: Vec<([f64;2],Reach)> = Vec::new();
        for s in &stretches {
            let r = pick(s);
            match out.last_mut() {
                Some((span,last)) if *last == r => span[1] = s.0[1],
                _ => out.push((s.0,r)),
            }
        }
        out
    };
    let (outer,inner) = (runs(&|s| s.1),runs(&|s| s.2));
    let at = |r: f64,z: f64| add(add(c,scale(f.x,r)),scale(axis,z));
    // the profile's corners: where two runs meet, the radius both reach (their mean, rounding apart)
    let joints = |runs: &[([f64;2],Reach)]| -> Vec<[f64;2]> {
        let mut out = vec![[rho(runs[0].1,runs[0].0[0]),runs[0].0[0]]];
        for w in runs.windows(2) {
            let z = w[0].0[1];
            out.push([0.5*(rho(w[0].1,z)+rho(w[1].1,z)),z]);
        }
        let last = runs[runs.len()-1];
        out.push([rho(last.1,last.0[1]),last.0[1]]);
        out
    };
    let (mut po,mut pi) = (joints(&outer),joints(&inner));
    // the profile closes at its bottom and top: where the farthest and nearest meet, one point
    for (k,j) in [(0,0),(po.len()-1,pi.len()-1)] {
        if (po[k][0]-pi[j][0]).abs() <= eps { pi[j] = po[k] }
    }
    for p in po.iter_mut().chain(pi.iter_mut()) { if p[0] <= eps { p[0] = 0. } }
    let mut loop_: Vec<ProfileEdge> = Vec::new();
    let mut piece = |r: Reach,a: [f64;2],b: [f64;2]| -> Result<(),Unbuilt> {
        if (a[0]-b[0]).abs() <= eps && (a[1]-b[1]).abs() <= eps { return Ok(()) }
        loop_.push(match r {
            Reach::Edge(i) => meridian_arc(&at,meridians[i],a,b)?,
            _ => ProfileEdge::Line {a:at(a[0],a[1]),b:at(b[0],b[1])},
        });
        Ok(())
    };
    for (k,&(_,r)) in outer.iter().enumerate() { piece(r,po[k],po[k+1])? }
    piece(Reach::Axis,po[po.len()-1],pi[pi.len()-1])?;
    for (k,&(_,r)) in inner.iter().enumerate().rev() { piece(r,pi[k+1],pi[k])? }
    piece(Reach::Axis,pi[0],po[0])?;
    let profile = Profile {origin:c,normal:f.y,names:Vec::new(),loops:vec![loop_]};
    let out = crate::build::revolve(&profile,c,axis,std::f64::consts::TAU)?;
    out.check(tol)?;
    Ok(out)
}

/// The meridian `ρ = √(throat² + k² (z − height)²)` from `a` to `b` (each `[ρ, z]`) as a profile
/// edge in the plane `at` maps `(ρ, z)` into: a line where it runs straight (a cylinder's, a cone's,
/// or its tangents parallel), else one rational quadratic, the conic through its ends, its end
/// tangents' meeting and its shoulder (the shoulder's weight `u / (1 − u)`).
pub(super) fn meridian_arc(at: &dyn Fn(f64,f64) -> V,(throat,height,k): (f64,f64,f64),a: [f64;2],b: [f64;2]) -> Result<ProfileEdge,String> {
    let line = ProfileEdge::Line {a:at(a[0],a[1]),b:at(b[0],b[1])};
    if k == 0. || throat <= 1e-12*(1.+a[0].max(b[0])) { return Ok(line) }
    let slope = |z: f64| k*k*(z-height)/radius((throat,height,k),z);
    let (ta,tb) = ([slope(a[1]),1.],[slope(b[1]),1.]);
    let det = -ta[0]*tb[1]+ta[1]*tb[0];
    if det.abs() <= 1e-12 { return Ok(line) }
    let t = (-(b[0]-a[0])*tb[1]+(b[1]-a[1])*tb[0])/det;
    let m = [a[0]+t*ta[0],a[1]+t*ta[1]];
    // the shoulder: where the line from the chord's middle to `m` meets the meridian, `u` along it
    let mid = [0.5*(a[0]+b[0]),0.5*(a[1]+b[1])];
    let (dr,dz,zr) = (m[0]-mid[0],m[1]-mid[1],mid[1]-height);
    let (qa,qb,qc) = (dr*dr-k*k*dz*dz,2.*(mid[0]*dr-k*k*zr*dz),mid[0]*mid[0]-throat*throat-k*k*zr*zr);
    let Some(u) = crate::util::roots::quadratic(qa,qb,qc).into_iter().find(|u| *u > 0. && *u < 1.) else {
        return Err("a hyperboloid's meridian has no shoulder between its chord and its tangents".into())
    };
    let conic = crate::nurbs::BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![at(a[0],a[1]),at(m[0],m[1]),at(b[0],b[1])],vec![1.,u/(1.-u),1.])?;
    Ok(ProfileEdge::Spline(std::sync::Arc::new(conic)))
}
