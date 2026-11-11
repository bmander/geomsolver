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
//!   helices where it returns (#780 L1, X5: `pipe_stretches`).
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
//! **A convex polyhedral tool turned whole is built by another road** (`turned_whole`): its solid
//! of revolution, the radii each slice holds turned once, with no Boolean. Anything else — under a
//! screw a round face about an axis leaning to it, or a torus's; under a turn a cylinder's or a
//! cone's about a leaning axis whose contact is no curve over the face (its two roots on a ring
//! meeting, or the curve reaching the axis) or whose meridian folds back along the axis, a ball
//! reaching across its axis, a torus's leaning to it, a sharp circle through it or whose normal lies
//! in the plane through it and its centre; under a screw a sharp circle leaning to its axis (#780
//! L2: its screwed sheet's meetings with the faces holding it at an end pose, and with a ball's pipe,
//! not yet in closed form); under a slide a torus's face leaning to its axis that is neither a whole
//! ring nor a band between smooth seams — a band meeting a neighbour in a sharp rim among them, its
//! fan not entered yet (#810) — or a band whose contact crosses both its seams, or a whole ring's
//! whose inner contour folds (by name, `Folded`); a free-form face, a curve no line
//! or circle, a leaning edge under a
//! screw; under any other motion a tool no convex solid of revolution, a contact whose pieces change
//! over the roll, a sheet folding or coming back onto itself (`Folded`, `Returns`) — is refused by
//! what it is (`Unbuilt`); those are the design's later steps.
use crate::build::{Profile,ProfileEdge,disc,prism,square};
use crate::geom::{Curve,Frame,Surface};
use crate::turned::Exact;
use crate::topo::{Brep,EdgeCurve};
use crate::trajectory::Trajectory;
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use super::member::{Solid,Swept};
#[allow(unused_imports)]
use crate::util::fmath::Det;

type V = [f64;3];

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
    /// coming back onto itself is entered in (#780 L1, X5: `pipe_stretches`).
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
/// kept where they are members. A sweep missing the blank leaves it whole.
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
fn legs(tool: &Brep,path: &Trajectory,roll: [f64;2]) -> Result<Vec<(crate::trajectory::Screw,[f64;2],Option<crate::trajectory::Pose>)>,Unbuilt> {
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

/// `closed` over a box holding the sweep, or `cut_from` over `blank`: the pipeline itself.
fn build(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Brep,Unbuilt> {
    // the legs the path is taken in: one constant twist over the roll, or each piece of a path of
    // pieces over its stretch of it (K1: a sweep is its pieces' sweeps united), every one read —
    // or, a motion no constant twist and no such path takes, K2's road: each instant's contact swept
    // into a patch (#780 L3)
    let legs = match legs(tool,path,roll) {
        Err(Unbuilt::Motion) if path.screw().is_none() => {
            // a prism under a motion keeping the plane square to it: its section's planar sweep,
            // extruded (#880 Q2), before K2's road
            if blank.is_none() { if let Some(b) = super::planar_solid::swept(tool,path,roll,bar,tol)? { return Ok(b) } }
            return k2(tool,path,roll,bar,tol,blank)
        }
        legs => legs?,
    };
    let whole_roll = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let size = { let (lo,hi) = tool.bounds(); norm(sub(hi,lo)) };
    let eps = 1e-9*(1.+size);
    // each candidate with the sites it is read at for whether it carries any of the boundary: the
    // points of its source carried over the roll, each with the candidate's normal there
    let mut cutters: Vec<(Cutter,Vec<(V,V)>)> = Vec::new();
    // the touches known by construction (#810, #780 R5), each naming two candidates by their index
    // among `cutters` (`add_cutter` answers the one it settled on) — and the points where a contact
    // crosses a seam, which no candidate's seam is laid at (`seam_away`)
    let mut touches: Vec<Touch> = Vec::new();
    let mut seams: Vec<V> = Vec::new();
    let mut add_cutter = |c: Cutter,sites: Vec<(V,V)>| -> usize {
        let same = |k: &Cutter| match (*k,c) {
            (Cutter::Plane((p,m)),Cutter::Plane((o,n))) => norm(cross(n,m)) <= 1e-9 && dot(sub(o,p),m).abs() <= eps,
            (Cutter::Ball {c:c1,r:r1,..},Cutter::Ball {c:c2,r:r2,..}) => norm(sub(c1,c2)) <= eps && (r1-r2).abs() <= eps,
            (Cutter::Cylinder {o:o1,axis:a1,r:r1,..},Cutter::Cylinder {o:o2,axis:a2,r:r2,..}) => (r1-r2).abs() <= eps
                && norm(cross(a1,a2)) <= 1e-9 && norm(cross(sub(o2,o1),a1)) <= eps,
            (Cutter::Revolved {o,axis,p:p1,e:e1,..},Cutter::Revolved {o:o2,axis:a2,p:p2,e:e2,..}) => {
                // the same surface: about one axis, the same throat at the same height, and the same
                // lean
                let (m1,m2) = (meridian(o,axis,p1,e1),meridian(o,axis,p2,e2));
                norm(cross(axis,a2)) <= 1e-9 && norm(cross(sub(o2,o),axis)) <= eps
                    && (m1.0-m2.0).abs() <= eps && (m1.1-m2.1).abs() <= eps && (m1.2-m2.2).abs() <= 1e-9
            }
            (Cutter::Torus {c:c1,axis:a1,major:m1,minor:n1},Cutter::Torus {c:c2,axis:a2,major:m2,minor:n2}) =>
                norm(sub(c1,c2)) <= eps && norm(cross(a1,a2)) <= 1e-9 && (m1-m2).abs() <= eps && (n1-n2).abs() <= eps,
            // two arcs of one sharp circle turned: one ring (#780 L2)
            (Cutter::Turned(t1),Cutter::Turned(t2)) => match (t1.source,t2.source) {
                (crate::turned::Source::Rim(x),crate::turned::Source::Rim(y)) => t1.frame == t2.frame && norm(sub(x.o,y.o)) <= eps
                    && (x.r-y.r).abs() <= eps && norm(cross(cross(x.x,x.y),cross(y.x,y.y))) <= 1e-9,
                _ => false,
            },
            _ => false,
        };
        match cutters.iter().position(|(k,_)| same(k)) {
            // one surface from two sources: over both their spans
            Some(i) => {
                let (k,known) = &mut cutters[i];
                if let (Cutter::Cylinder {o,axis,span,..} | Cutter::Revolved {o,axis,span,..},
                    Cutter::Cylinder {o:from,axis:way,span:more,..} | Cutter::Revolved {o:from,axis:way,span:more,..}) = (k,c) {
                    // the other's heights read from this one's origin along this one's axis
                    let (shift,sense) = (dot(sub(from,*o),*axis),dot(way,*axis).signum());
                    let more = [shift+sense*more[0],shift+sense*more[1]];
                    *span = [span[0].min(more[0].min(more[1])),span[1].max(more[0].max(more[1]))];
                }
                known.extend(sites);
                i
            }
            None => { cutters.push((c,sites)); cutters.len()-1 }
        }
    };
    // the points the box must hold: the tool's box at each leg's end poses and, about a turn's
    // axis, its reach
    let box_corners = corners_of;
    let corners = box_corners(tool.bounds());
    let mut held_points: Vec<V> = Vec::new();
    let one = legs.len() == 1;
    for &(screw,[a,b],place) in &legs {
        // the tool where the leg takes it up, read under the leg's twist
        let placed_tool = place.map(|q| tool.moved(&crate::geom::Rigid {r:q.r,t:q.p}));
        let tool: &Brep = placed_tool.as_ref().unwrap_or(tool);
        let path = &Trajectory::Twist(screw);
        let Read {faces,edges,normals,round,rims,bands,levels,..} = read(tool,path)?;
        // each face's corners in walk order, what its own sites are read over
        let corners_of: Vec<Vec<V>> = tool.faces.iter().map(|f| f.loops.iter().flatten()
            .map(|c| tool.vertices[tool.ends(c)[0] as usize].p).collect()).collect();
        // and, for a face with an edge no line (a drum's end, its loop one circle on one vertex),
        // its outline: each edge's start in walk order, and 32 points along each edge no line (#780
        // L2) — none where every edge is a line, whose corners are its outline
        let outline_of: Vec<Option<Vec<V>>> = tool.faces.iter().map(|f| {
            let straight = |c: &crate::topo::Coedge| matches!(tool.edges[c.edge as usize].curve,EdgeCurve::Curve(Curve::Line {..}));
            (!f.loops.iter().flatten().all(straight)).then(|| f.loops.iter().flatten().flat_map(|c| {
                let e = &tool.edges[c.edge as usize];
                let [t0,t1] = if c.reversed { [e.t[1],e.t[0]] } else { e.t };
                let n = if straight(c) { 1 } else { 32 };
                (0..n).map(move |i| e.point(t0+(t1-t0)*i as f64/n as f64,&tool.vertices))
            }).collect())
        }).collect();
        let corners = box_corners(tool.bounds());
        let pose_at = |t: f64| path.pose(t);
        let (start,end) = (pose_at(a)?,pose_at(b)?);
        held_points.extend(corners.iter().flat_map(|&p| [start.point(p),end.point(p)]));
        if screw.is_turn() {
            let reach = corners.iter().map(|&p| norm(sub(p,screw.origin))).fold(0.,f64::max);
            held_points.extend([1.,-1.].iter().flat_map(|&s| (0..3).map(move |k| { let mut v = [0.;3]; v[k] = s*reach; v }))
                .map(|v| add(screw.origin,v)));
        }
        // the poses the sites are carried by, across the roll
        // (inside the roll and inside each source, a site at a source's own edge reading its neighbours'
        // boundary, not its own)
        let poses: Vec<crate::trajectory::Pose> = (0..=SITES).map(|k| pose_at(a+(b-a)*within(k))).collect::<Result<_,_>>()?;
        // a face's own points, a grid over each triangle of a fan of its corners — or of its outline,
        // each grid as much finer as the fan has more triangles, so a face of many is read at about
        // as many points as a triangle
        let face_sites = |k: usize| -> Vec<V> {
            let (c,m) = match &outline_of[k] {
                Some(c) => (c,((SITES as f64/(c.len().saturating_sub(2).max(1) as f64).sqrt()).ceil() as usize).max(2)),
                None => (&corners_of[k],SITES),
            };
            let mut out = Vec::new();
            for w in 1..c.len().saturating_sub(1) {
                for i in 0..m { for j in 0..m-i {
                    let (u,v) = ((i as f64+1./3.)/(m+1) as f64,(j as f64+1./3.)/(m+1) as f64);
                    out.push(std::array::from_fn(|q| c[0][q]+u*(c[w][q]-c[0][q])+v*(c[w+1][q]-c[0][q])));
                } }
            }
            out
        };
        if screw.is_slide() {
            let d = sub(end.point([0.;3]),start.point([0.;3]));
            let located = crate::query::Located::new(tool,tol);
            for (k,&(o,n)) in faces.iter().enumerate() {
                if round.contains(&k) { continue }
                let s = dot(n,d);
                let pose = if s > eps { &end } else { &start };
                add_cutter(Cutter::Plane((pose.point(o),n)),face_sites(k).into_iter().map(|x| (pose.point(x),n)).collect());
            }
            for (&(p,q,_),&[n1,n2]) in edges.iter().zip(&normals) {
                let (s1,s2) = (dot(n1,d),dot(n2,d));
                if s1*s2 > 0. || (s1.abs() <= eps && s2.abs() <= eps) { continue }
                let n = cross(sub(q,p),d);
                if norm(n) <= eps*norm(d) { continue }
                let n = unit(n);
                let sites = (0..=SITES).flat_map(|i| poses.iter().map(move |m| (m.point(add(p,scale(sub(q,p),within(i)))),n))).collect();
                add_cutter(Cutter::Plane((start.point(p),n)),sites);
            }
            // a face no plane: at each end pose its own surface over the stretch it spans (a cylinder's
            // solid, a cone's frustum), and its contact swept: each ruling where its normal is square
            // to the slide swept into the plane tangent there, or a cylinder along the slide swept
            // along itself
            for &k in &round {
                let face = &tool.faces[k];
                // (a ball's face over every latitude, its own points those in it: its pole on no edge; a
                // torus's over every angle round its tube)
                let (lo,hi) = match face.surface {
                    Surface::Sphere(..) => (-std::f64::consts::FRAC_PI_2,std::f64::consts::FRAC_PI_2),
                    Surface::Torus(..) => (0.,std::f64::consts::TAU),
                    _ => rulings_span(tool,k),
                };
                let points = round_points(&located,tool,k,[lo,hi]);
                let sites_at = |m: &crate::trajectory::Pose| -> Vec<(V,V)> {
                    points.iter().map(|&x| (m.point(x),m.vector(outward(face,x)))).collect()
                };
                match face.surface {
                    Surface::Cylinder(f,r) => {
                        let a = unit(f.z);
                        let across = sub(d,scale(a,dot(d,a)));
                        for pose in [&start,&end] {
                            add_cutter(Cutter::Cylinder {o:pose.point(f.o),axis:a,r:r.abs(),span:[lo,hi]},sites_at(pose));
                        }
                        if norm(across) <= 1e-9*norm(d) {
                            let s = dot(d,a);
                            add_cutter(Cutter::Cylinder {o:start.point(f.o),axis:a,r:r.abs(),span:[lo+s.min(0.),hi+s.max(0.)]},
                                poses.iter().flat_map(|m| sites_at(m)).collect());
                            continue
                        }
                        let w = unit(cross(a,across));
                        for w in [w,scale(w,-1.)] {
                            let u = dot(w,f.y).datan2(dot(w,f.x));
                            tangent_plane(&mut add_cutter,&located,tool,k,u,[lo,hi],w,&start,&poses);
                        }
                    }
                    Surface::Cone(f,_,half) => {
                        let (sa,ca) = half.dsin_cos();
                        // the generator through the cone's point at `u = 0`, turned about its axis
                        let p0 = face.surface.point([0.,0.]);
                        let e = unit(add(scale(f.x,sa),scale(f.z,ca)));
                        for pose in [&start,&end] {
                            add_cutter(Cutter::Revolved {o:pose.point(f.o),axis:f.z,p:pose.point(p0),e,span:[lo*ca,hi*ca]},sites_at(pose));
                        }
                        // the normal `cos α ρ̂(u) − sin α z` square to the slide:
                        // `A cos u + B sin u = C`
                        let (aa,bb,cc) = (ca*dot(d,f.x),ca*dot(d,f.y),sa*dot(d,f.z));
                        let rr = aa.dhypot(bb);
                        if !(rr > 1e-12*norm(d)) || cc.abs() > rr { continue }
                        let (phi,delta) = (bb.datan2(aa),(cc/rr).clamp(-1.,1.).dacos());
                        for u in [phi+delta,phi-delta] {
                            let w = unit(face.surface.normal_raw([u,0.5*(lo+hi)]));
                            tangent_plane(&mut add_cutter,&located,tool,k,u,[lo,hi],w,&start,&poses);
                        }
                    }
                    Surface::Sphere(f,r) => {
                        let r = r.abs();
                        for pose in [&start,&end] {
                            add_cutter(Cutter::Ball {c:pose.point(f.o),axis:pose.vector(f.z),r},sites_at(pose));
                        }
                        // its great circle square to the slide, carried from the start to the end: the
                        // cylinder of its radius about the centre's path, over that path
                        let (len,dd) = (norm(d),unit(d));
                        let g = Frame::about(f.o,dd);
                        let sites: Vec<(V,V)> = (0..4*SITES).flat_map(|i| {
                            let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
                            let w = add(scale(g.x,cp),scale(g.y,sp));
                            let x = add(f.o,scale(w,r));
                            let on = located.face_place(k,x) != crate::query::Place::Out;
                            poses.iter().filter(move |_| on).map(move |m| (m.point(x),w))
                        }).collect();
                        add_cutter(Cutter::Cylinder {o:start.point(f.o),axis:dd,r,span:[0.,len]},sites);
                    }
                    Surface::Torus(f,big,small) => {
                        let (big,small,z) = (big.abs(),small.abs(),unit(f.z));
                        let tori = [&start,&end].map(|pose| {
                            (add_cutter(Cutter::Torus {c:pose.point(f.o),axis:pose.vector(z),major:big,minor:small},sites_at(pose)),pose)
                        });
                        // the torus at each end pose touches a contour's cylinder or sheet along the
                        // contact there, and only there short of the fold (#805, `Slid::meets_again`):
                        // given whole, each half or arc of it on its own face
                        let contacts = |sheet: usize,on: &[crate::slid::Slid]| -> Vec<Touch> {
                            tori.iter().flat_map(|&(torus,pose)| on.iter().map(move |h| Touch {cutters:[torus,sheet],
                                curves:vec![Curve::Slid(std::sync::Arc::new(h.at(pose.point(f.o),false)))]})).collect()
                        };
                        // the circle about `centre` of radius `r` square to `n`, read where it is on the
                        // face, each site with its way out from the centre
                        let circle = |centre: V,n: V,r: f64| -> Vec<(V,V)> {
                            let g = Frame::about(centre,n);
                            (0..4*SITES).flat_map(|i| {
                                let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
                                let w = add(scale(g.x,cp),scale(g.y,sp));
                                let x = add(centre,scale(w,r));
                                let on = located.face_place(k,x) != crate::query::Place::Out;
                                poses.iter().filter(move |_| on).map(move |m| (m.point(x),w))
                            }).collect()
                        };
                        let (len,dd) = (norm(d),unit(d));
                        if norm(cross(z,dd)) <= 1e-9 {
                            // along the axis: its equators, inner and outer, swept into coaxial cylinders
                            let s = dot(d,z);
                            for r in [big+small,big-small] {
                                add_cutter(Cutter::Cylinder {o:start.point(f.o),axis:z,r,span:[s.min(0.),s.max(0.)]},circle(f.o,z,r));
                            }
                        } else if dot(z,dd).abs() > 1e-9 {
                            // leaning to the axis: a whole ring's two contacts (#805), each a closed
                            // curve smooth across the branch locus, swept along the slide into the
                            // cylinder over its apparent contour (the contour's region extruded from
                            // short of the torus at the start pose to past it at the end) — or a
                            // band's (#810), its outer contact's cylinder where the band holds all
                            // of it, else its arc on the band swept into the sheet over that arc of
                            // the contour, cut at the seams it crosses (`Cutter::Sheet`), so no part
                            // of it stands inside the sweep, and at each seam that point's path
                            // along the slide given as the sheet's touch with the plane the
                            // neighbour's ruling there sweeps (`boolean::Given`), the sheet's rim on
                            // it. Each read at the contact's points on the face carried over the
                            // roll with the torus's normal there
                            let band = bands.iter().find(|b| b.0 == k).map(|&(_,band,contact)| (band,contact));
                            let carried = |h: &crate::slid::Slid| -> Vec<(V,V)> {
                                (0..=SITES).map(|i| h.d2(h.span[0]+(h.span[1]-h.span[0])*within(i)).0)
                                    .filter(|&x| located.face_place(k,x) != crate::query::Place::Out)
                                    .flat_map(|x| { let n = outward(face,x); poses.iter().map(move |m| (m.point(x),m.vector(n))) }).collect()
                            };
                            let sides: &[f64] = match band { None => &[1.,-1.],Some((_,BandContact::Whole)) => &[1.],Some(_) => &[] };
                            let reach = big*norm(sub(dd,scale(z,dot(dd,z))))+small;
                            let span = [-(1.+PAST)*reach,len+(1.+PAST)*reach];
                            for &side in sides {
                                let on = crate::slid::halves(f,big,small,dd,side,false);
                                let sites = on.iter().flat_map(|h| carried(h)).collect();
                                let contour = add_cutter(Cutter::Contour {halves:on.map(|h| h.at(start.point(f.o),true)),span},sites);
                                touches.extend(contacts(contour,&on));
                            }
                            if let Some((band,BandContact::Arc {centre,half,seam})) = band {
                                let arc = band.arc(dd,centre,half);
                                let sheet = add_cutter(Cutter::Sheet {arc:arc.at(start.point(f.o),true)},carried(&arc));
                                touches.extend(contacts(sheet,&[arc]));
                                // where the contact crosses the seam: its path along the slide, on the
                                // plane the neighbour's ruling there sweeps (entered by that
                                // neighbour, `tangent_plane`; here, with no sites, it is named)
                                for u in [centre-half,centre+half] {
                                    let (x,n) = band.at(u,band.v[seam]);
                                    let plane = add_cutter(Cutter::Plane((start.point(x),n)),Vec::new());
                                    touches.push(Touch {cutters:[plane,sheet],curves:vec![Curve::Line {p:start.point(x),d:dd}]});
                                    seams.extend([start.point(x),end.point(x)]);
                                }
                            }
                        } else {
                            // across the axis: its top and bottom rings swept within their level planes
                            for side in [1.,-1.] {
                                let o = add(f.o,scale(z,side*small));
                                let sites: Vec<(V,V)> = circle(o,z,big).into_iter().map(|(x,_)| (x,scale(z,side))).collect();
                                add_cutter(Cutter::Plane((start.point(o),z)),sites);
                            }
                            // and its meridians square to the slide swept into cylinders along it
                            let w = unit(cross(z,dd));
                            for side in [1.,-1.] {
                                let centre = add(f.o,scale(w,side*big));
                                add_cutter(Cutter::Cylinder {o:start.point(centre),axis:dd,r:small,span:[0.,len]},circle(centre,dd,small));
                            }
                        }
                    }
                    _ => unreachable!("`read` takes only cylinders, cones, balls and tori round"),
                }
            }
            // a circle edge swept: an oblique cylinder (the circle carried along the slide), or within
            // its own plane where the slide lies in it
            for rim in &rims {
                let (f,r) = (rim.frame,rim.r);
                let a = unit(f.z);
                let n_of = |phi: f64| -> V {
                    let (sp,cp) = phi.dsin_cos();
                    let t = add(scale(f.x,-sp),scale(f.y,cp));
                    let n = cross(t,d);
                    if norm(n) <= 1e-12*norm(d) { a } else { unit(n) }
                };
                let sites: Vec<(V,V)> = (0..=SITES).flat_map(|i| {
                    let phi = rim.span[0]+(rim.span[1]-rim.span[0])*within(i);
                    let (sp,cp) = phi.dsin_cos();
                    let x = f.at([r*cp,r*sp,0.]);
                    let n = n_of(phi);
                    poses.iter().map(move |m| (m.point(x),n))
                }).collect();
                let c = start.point(f.o);
                if dot(d,a).abs() <= 1e-9*norm(d) { add_cutter(Cutter::Plane((c,a)),sites); }
                else { add_cutter(Cutter::Oblique {c,a,r:r.abs(),d},sites); }
            }
        } else {
            let (c,axis) = (screw.origin,screw.axis);
            // the turn's frame a turned meridian is read in (#780 C3, L2)
            let g = Frame::about(c,axis);
            // a convex tool turned whole is the solid of revolution of its radial profile, built in
            // closed form with no Boolean
            let whole = (screw.ratio*(b-a)).abs() >= std::f64::consts::TAU*(1.-1e-12);
            // the poses each face is taken at: both ends of the roll — but a whole turn's sweep is
            // its tool's solid of revolution, bounded by its contacts, edges and rims turned alone,
            // which its faces at either end only touch (#780 C3: a leaning stem at an end pose
            // touching its contact's surface of revolution along the contact, crossed there by its
            // ball's torus, was a triple touch no arrangement holds by rounding), so none
            let ends: Vec<&crate::trajectory::Pose> = if screw.is_turn() && whole { vec![] } else { vec![&start,&end] };
            let convex = faces.iter().all(|&(o,n)| tool.vertices.iter().all(|v| dot(sub(v.p,o),n) <= eps));
            // each face's candidate at each end pose (by the pose's place in `ends`), where it holds
            // a touch given by construction (#810)
            let mut at_ends: std::collections::BTreeMap<(usize,usize),usize> = std::collections::BTreeMap::new();
            if one && screw.is_turn() && whole && convex && round.is_empty() && rims.is_empty() && levels.is_empty() && blank.is_none() {
                return turned_whole(&faces,&edges,c,axis,eps,tol)
            }
            // under a screw the heights a source spans run on by the step it takes over the roll, the
            // rest as under a turn (a cylinder about the axis keeps itself under both)
            let lift = { let rate = screw.advance/std::f64::consts::TAU; [(rate*a).min(rate*b),(rate*a).max(rate*b)] };
            // a line turned whole: a cylinder about the axis where it runs along it, a plane square to
            // the axis where it runs square to it, else a hyperboloid (a double cone where it meets the
            // axis)
            let turned = |p: V,e: V,q: V| -> Option<Cutter> {
                let e = unit(e);
                let (h0,h1) = (dot(sub(p,c),axis),dot(sub(q,c),axis));
                let span = [h0.min(h1)+lift[0],h0.max(h1)+lift[1]];
                if norm(cross(e,axis)) <= 1e-9 {
                    let r = { let r = sub(p,c); norm(sub(r,scale(axis,dot(r,axis)))) };
                    return (r > eps).then_some(Cutter::Cylinder {o:c,axis,r,span})
                }
                if dot(e,axis).abs() <= 1e-9 { return Some(Cutter::Plane((p,axis))) }
                Some(Cutter::Revolved {o:c,axis,p,e,span})
            };
            // a segment from `p` to `q` under the twist: turned (`turned`), or under a screw, leaning to
            // its axis, a patch of the ruled helicoid it sweeps (#777 M5), square to it the helicoidal
            // sheet it sweeps across the box
            let line_cutter = |p: V,q: V| -> Option<Cutter> {
                let e = unit(sub(q,p));
                if !screw.is_turn() && norm(cross(e,axis)) > 1e-9 {
                    return Some(if dot(e,axis).abs() > 1e-9 { Cutter::Patch {p,q} } else { Cutter::Screwed {p,e} })
                }
                turned(p,sub(q,p),q)
            };
            // the sites of a line from `p` to `q` turned: its points at each pose, with the normal of
            // the surface it turns into there
            let line_sites = |p: V,q: V,cutter: Cutter| -> Vec<(V,V)> {
                (0..=SITES).flat_map(|i| {
                    let x = add(p,scale(sub(q,p),within(i)));
                    let r = sub(x,c);
                    let radial = sub(r,scale(axis,dot(r,axis)));
                    let normal = match cutter {
                        Cutter::Plane((_,m)) => m,
                        Cutter::Cylinder {..} => unit(radial),
                        Cutter::Revolved {e,..} => {
                            let (rho,ez) = (norm(radial),dot(e,axis));
                            let slope = dot(radial,sub(e,scale(axis,ez)))/(ez*rho.max(1e-300));
                            unit(sub(scale(radial,1./rho.max(1e-300)),scale(axis,slope)))
                        }
                        // the sheet's normal: square to the line and to the screw's velocity there (which
                        // the screw carries as it carries the normal)
                        Cutter::Screwed {e,..} => unit(cross(e,screw.velocity(x))),
                        Cutter::Patch {p,q} => unit(cross(unit(sub(q,p)),screw.velocity(x))),
                        Cutter::Oblique {..} | Cutter::Ball {..} | Cutter::Torus {..} | Cutter::ScrewedDisc {..} | Cutter::Pipe {..}
                            | Cutter::Turned(_) | Cutter::Contour {..} | Cutter::Sheet {..} => unreachable!("a line turns into none of these"),
                    };
                    poses.iter().map(move |m| (m.point(x),m.vector(normal)))
                }).collect()
            };
            for (k,&(o,n)) in faces.iter().enumerate() {
                if round.contains(&k) { continue }
                let own = face_sites(k);
                for (j,pose) in ends.iter().enumerate() {
                    at_ends.insert((k,j),add_cutter(Cutter::Plane((pose.point(o),pose.vector(n))),own.iter().map(|&x| (pose.point(x),pose.vector(n))).collect()));
                }
                let m = cross(n,axis);
                // a face square to the axis: under a turn its own plane (kept in itself), under a screw no
                // contact at all (its normal meets the screw's velocity everywhere, by the step)
                if norm(m) <= 1e-9 {
                    if screw.is_turn() { add_cutter(Cutter::Plane((o,n)),poses.iter().flat_map(|pose| own.iter().map(|&x| (pose.point(x),n))).collect()); }
                    continue
                }
                // under a screw a face leaning to the axis touches where its normal is square to the
                // screw's velocity, `n · v(x) = 0`, affine in `x`: a line of its plane, leaning, screwed
                // into a patch of a ruled helicoid (#777 M5)
                if !screw.is_turn() && dot(n,axis).abs() > 1e-9 {
                    if let Some((from,to)) = leaning_contact(&screw,o,n,&corners_of[k],eps) {
                        let cutter = Cutter::Patch {p:from,q:to};
                        add_cutter(cutter,line_sites(from,to,cutter));
                    }
                    continue
                }
                // under a screw a face along the axis touches along its line nearest the axis, as under a
                // turn
                // the face's contact line: its plane's points whose normal passes through the axis, where
                // its plane meets the plane through the axis along `n × axis`; it meets the axis, or runs
                // along it
                if dot(n,sub(o,c)).abs() <= eps { continue }
                let (x0,e) = (add(c,scale(n,dot(n,sub(o,c)))),unit(cross(n,m)));
                // the stretch of the contact line the face spans: over its corners, or where a face
                // with an edge no line has one, the chord its outline cuts from the line, a twentieth
                // more each way (none where the line misses the face, #780 L2)
                let (t0,t1) = match &outline_of[k] {
                    None => corners_of[k].iter().map(|v| dot(sub(*v,x0),e)).fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),t| (l.min(t),h.max(t))),
                    Some(outline) => {
                        let across = unit(cross(n,e));
                        let side = |p: V| dot(sub(p,x0),across);
                        let cuts: Vec<f64> = (0..outline.len()).filter_map(|i| {
                            let (p,q) = (outline[i],outline[(i+1)%outline.len()]);
                            let (sp,sq) = (side(p),side(q));
                            (sp*sq <= 0. && sp != sq).then(|| dot(sub(add(p,scale(sub(q,p),sp/(sp-sq))),x0),e))
                        }).collect();
                        if cuts.is_empty() { continue }
                        let (lo,hi) = cuts.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),&t| (l.min(t),h.max(t)));
                        let past = PAST*(hi-lo);
                        (lo-past,hi+past)
                    }
                };
                let (from,to) = (add(x0,scale(e,t0)),add(x0,scale(e,t1)));
                if let Some(cutter) = turned(from,e,to) { add_cutter(cutter,line_sites(from,to,cutter)); }
            }
            // a round face (`reads` takes one only under a turn): at both end poses its own surface (the
            // cylinder, the frustum, the ball), and its contact turned. Its normal passes through the
            // axis where it lies in the plane through the axis and the face's own axis, or its centre:
            // a cylinder's or a cone's two generators there, each turned (a cylinder or a cone about the
            // axis, `turned`), or a ball's great circle there, turned into a torus
            let located = crate::query::Located::new(tool,tol);
            for &k in &round {
                let face = &tool.faces[k];
                let (lo,hi) = match face.surface {
                    Surface::Sphere(..) => (-std::f64::consts::FRAC_PI_2,std::f64::consts::FRAC_PI_2),
                    Surface::Torus(..) => (0.,std::f64::consts::TAU),
                    _ => rulings_span(tool,k),
                };
                let points = round_points(&located,tool,k,[lo,hi]);
                let sites_at = |m: &crate::trajectory::Pose| -> Vec<(V,V)> {
                    points.iter().map(|&x| (m.point(x),m.vector(outward(face,x)))).collect()
                };
                // the face's ruling at `u` round its axis, where it is on the face, under the twist
                let ruling = |u: f64| -> Option<(Cutter,Vec<(V,V)>)> {
                    let (p,q) = (face.surface.point([u,lo]),face.surface.point([u,hi]));
                    if located.face_place(k,face.surface.point([u,0.5*(lo+hi)])) == crate::query::Place::Out { return None }
                    let cutter = line_cutter(p,q)?;
                    Some((cutter,line_sites(p,q,cutter)))
                };
                // the way from the axis to the face's own axis or centre, square to the axis
                let away = |o: V| -> Option<V> { let r = sub(o,c); let w = sub(r,scale(axis,dot(r,axis))); (norm(w) > eps).then(|| unit(w)) };
                // a cylinder's or a cone's about an axis leaning to the turn's (#780 C3): its own
                // surface at both end poses half `PAST` past its span each way (no end of it on its
                // contact's meridian's, `PAST` past, nor on a face it meets at a concave edge — a
                // stem's rim on the ball it enters), and its contact on each ring turned
                let leaning = |f: &Frame| screw.is_turn() && norm(cross(f.z,axis)) > 1e-9;
                let own = |f: &Frame| if leaning(f) { let past = 0.5*PAST*(hi-lo); [lo-past,hi+past] } else { [lo,hi] };
                match face.surface {
                    Surface::Cylinder(f,r) => {
                        // past each sharp circle it holds that turns into a ring (#780 L2), beyond the
                        // loop along which its surface at an end pose crosses that ring
                        // (`turned::Crossing`, its heights from the circle the turn keeps): its ends
                        // there stand clear of that loop, which near the circle runs where the two
                        // all but touch
                        let mut span = own(&f);
                        if leaning(&f) {
                            let a = unit(f.z);
                            for rim in &rims {
                                let Some(k) = crate::turned::Crossing::held(&rim.ring(&g),g.dir_local(a),g.local(f.o),r.abs(),eps) else { continue };
                                let ([h0,h1],at) = (k.heights(),dot(sub(rim.frame.o,f.o),a));
                                let reach = 1.25*h0.abs().max(h1.abs());
                                span = [span[0].min(at-reach),span[1].max(at+reach)];
                            }
                        }
                        for (j,pose) in ends.iter().enumerate() {
                            at_ends.insert((k,j),add_cutter(Cutter::Cylinder {o:pose.point(f.o),axis:pose.vector(unit(f.z)),r:r.abs(),span},sites_at(pose)));
                        }
                        if leaning(&f) {
                            let turned = turned_contacts(&mut add_cutter,&located,face,k,&screw,(f,r.abs(),0.,1.),[lo,hi],&poses,&ruling,eps)?;
                            touches.extend(own_contacts(&face.surface,k,&turned,&ends,&at_ends,tol));
                            continue
                        }
                        let ways = match away(f.o) { Some(w) => vec![w,scale(w,-1.)],None => vec![] };
                        // coaxial, the face turns into itself
                        if ways.is_empty() { add_cutter(Cutter::Cylinder {o:f.o,axis:unit(f.z),r:r.abs(),span:[lo,hi]},poses.iter().flat_map(|m| sites_at(m)).collect()); }
                        for w in ways {
                            if let Some((cutter,sites)) = ruling(dot(w,f.y).datan2(dot(w,f.x))) { add_cutter(cutter,sites); }
                        }
                    }
                    Surface::Cone(f,r,half) => {
                        let (sa,ca) = half.dsin_cos();
                        let p0 = face.surface.point([0.,0.]);
                        let e = unit(add(scale(f.x,sa),scale(f.z,ca)));
                        let [l0,h0] = own(&f);
                        for (j,pose) in ends.iter().enumerate() {
                            at_ends.insert((k,j),add_cutter(Cutter::Revolved {o:pose.point(f.o),axis:pose.vector(f.z),p:pose.point(p0),
                                e:pose.vector(e),span:[l0*ca,h0*ca]},sites_at(pose)));
                        }
                        if leaning(&f) {
                            let turned = turned_contacts(&mut add_cutter,&located,face,k,&screw,(f,r,sa,ca),[lo,hi],&poses,&ruling,eps)?;
                            touches.extend(own_contacts(&face.surface,k,&turned,&ends,&at_ends,tol));
                            continue
                        }
                        if !screw.is_turn() {
                            // under a screw its normal `cos α ρ̂(u) − sin α z` is square to the
                            // velocity `ω axis × (x − c) + κ axis` where `A cos u + B sin u = C`,
                            // whatever the height along the ruling (the turn's part along `ρ̂` is
                            // `ω ρ̂ · (axis × (o − c))`, the climb's along `z` is `κ`): two rulings
                            // turned off the plane through both axes by `asin(κ tan α / (ω |d|))`,
                            // or none (a cone about the screw's own axis, or a climb too steep for
                            // its lean). Each is a line leaning to the axis, screwed into a patch
                            // of a ruled helicoid (#777 M5), which the cone at each end pose
                            // touches along that ruling
                            let w = cross(axis,sub(f.o,c));
                            let kappa = screw.advance/std::f64::consts::TAU*dot(f.z,axis).signum();
                            let (aa,bb,cc) = (screw.ratio*ca*dot(w,f.x),screw.ratio*ca*dot(w,f.y),sa*kappa);
                            let rr = aa.dhypot(bb);
                            if !(rr > 1e-12*(screw.ratio.abs()*size+kappa.abs())) || cc.abs() >= rr { continue }
                            let (phi,delta) = (bb.datan2(aa),(cc/rr).clamp(-1.,1.).dacos());
                            for u in [phi+delta,phi-delta] {
                                if let Some((cutter,sites)) = ruling(u) { add_cutter(cutter,sites); }
                            }
                            continue
                        }
                        let ways = match away(f.o) { Some(w) => vec![w,scale(w,-1.)],None => vec![] };
                        if ways.is_empty() {
                            add_cutter(Cutter::Revolved {o:f.o,axis:f.z,p:p0,e,span:[lo*ca,hi*ca]},poses.iter().flat_map(|m| sites_at(m)).collect());
                        }
                        for w in ways {
                            if let Some((cutter,sites)) = ruling(dot(w,f.y).datan2(dot(w,f.x))) { add_cutter(cutter,sites); }
                        }
                    }
                    Surface::Sphere(f,r) if !screw.is_turn() => {
                        let r = r.abs();
                        for pose in &ends {
                            add_cutter(Cutter::Ball {c:pose.point(f.o),axis:pose.vector(f.z),r},sites_at(pose));
                        }
                        // its great circle square to its centre's velocity, screwed: the pipe —
                        // entered in stretches of the roll each meeting itself nowhere where it
                        // comes back onto itself (#780 L1, X5)
                        let n = unit(screw.velocity(f.o));
                        let g = Frame::about(f.o,n);
                        // the circle's points on the face and their normals, carried over each
                        // stretch's own roll
                        let ring: Vec<(V,V)> = (0..4*SITES).filter_map(|i| {
                            let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
                            let w = add(scale(g.x,cp),scale(g.y,sp));
                            let x = add(f.o,scale(w,r));
                            (located.face_place(k,x) != crate::query::Place::Out).then_some((x,w))
                        }).collect();
                        for span in pipe_stretches(&screw,f.o,n,r,[a,b],eps)? {
                            let poses: Vec<crate::trajectory::Pose> = (0..=SITES)
                                .map(|i| pose_at(span[0]+(span[1]-span[0])*within(i))).collect::<Result<_,_>>()?;
                            let sites = ring.iter().flat_map(|&(x,w)| poses.iter().map(move |m| (m.point(x),m.vector(w)))).collect();
                            add_cutter(Cutter::Pipe {c:f.o,n,r,span},sites);
                        }
                    }
                    Surface::Sphere(f,r) => {
                        let r = r.abs();
                        for pose in &ends {
                            add_cutter(Cutter::Ball {c:pose.point(f.o),axis:pose.vector(f.z),r},sites_at(pose));
                        }
                        let height = dot(sub(f.o,c),axis);
                        let Some(w) = away(f.o) else {
                            // a ball about a point of the axis turns into itself
                            add_cutter(Cutter::Ball {c:f.o,axis:f.z,r},poses.iter().flat_map(|m| sites_at(m)).collect());
                            continue
                        };
                        let major = norm(sub(sub(f.o,c),scale(axis,height)));
                        // a ball reaching across the axis turns into no ring torus: not held yet
                        if !(major > r+eps) { return Err(Unbuilt::Curved {at:f.o}) }
                        let sites: Vec<(V,V)> = (0..4*SITES).flat_map(|i| {
                            let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
                            let n = add(scale(w,cp),scale(axis,sp));
                            let x = add(f.o,scale(n,r));
                            let on = located.face_place(k,x) != crate::query::Place::Out;
                            poses.iter().filter(move |_| on).map(move |m| (m.point(x),m.vector(n)))
                        }).collect();
                        add_cutter(Cutter::Torus {c:add(c,scale(axis,height)),axis,major,minor:r},sites);
                    }
                    Surface::Torus(f,big,small) => {
                        let (big,r) = (big.abs(),small.abs());
                        let z = unit(f.z);
                        for pose in &ends {
                            add_cutter(Cutter::Torus {c:pose.point(f.o),axis:pose.vector(z),major:big,minor:r},sites_at(pose));
                        }
                        // the face's points of a circle on the torus about `o` in the plane of `p` and
                        // `q`, each with its normal, carried over the roll
                        let circle_sites = |o: V,p: V,q: V,rad: f64| -> Vec<(V,V)> {
                            (0..4*SITES).flat_map(|i| {
                                let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
                                let x = add(o,add(scale(p,rad*cp),scale(q,rad*sp)));
                                let n = outward(face,x);
                                let on = located.face_place(k,x) != crate::query::Place::Out;
                                poses.iter().filter(move |_| on).map(move |m| (m.point(x),m.vector(n)))
                            }).collect()
                        };
                        // its top and bottom rings, their normals along the axis: each in its own plane
                        for side in [1.,-1.] {
                            let o = add(f.o,scale(z,side*r));
                            add_cutter(Cutter::Plane((o,scale(z,side))),circle_sites(o,f.x,cross(z,f.x),big));
                        }
                        let height = dot(sub(f.o,c),axis);
                        let Some(w) = away(f.o) else {
                            // a torus about the axis turns into itself
                            add_cutter(Cutter::Torus {c:f.o,axis:z,major:big,minor:r},poses.iter().flat_map(|m| sites_at(m)).collect());
                            continue
                        };
                        // its meridians in the plane through both axes, each turned into a torus about the
                        // axis (one reaching across the axis turns into no ring torus: not held yet)
                        for side in [1.,-1.] {
                            let m = add(f.o,scale(w,side*big));
                            let major = norm(sub(sub(m,c),scale(axis,dot(sub(m,c),axis))));
                            if !(major > r+eps) { return Err(Unbuilt::Curved {at:m}) }
                            add_cutter(Cutter::Torus {c:add(c,scale(axis,height)),axis,major,minor:r},circle_sites(m,w,axis,r));
                        }
                    }
                    _ => unreachable!("`read` takes only cylinders, cones, balls and tori round"),
                }
            }
            // under a turn a sharp circle leaning to the axis (#780 L2): its fan's contact — where
            // the fan between its faces' normals holds one square to the turn's velocity — lies on
            // the surface its points sweep, a ring whose meridian is read exactly
            // (`turned::Rim`, `Cutter::Turned`), its sites the circle's points carried over the
            // roll, each with the ring's normal there (square to the circle and to the velocity).
            // An arc of one sweeps a part of its circle's ring: the whole ring is its candidate, its
            // sites the arc's own points, and membership keeps what bounds the sweep; two arcs of
            // one circle (a rim a Boolean split) enter one ring. Part of a turn, the faces it bounds
            // at each end pose hold it, met in closed form where they are planes and cylinders
            // (`ssi`'s `plane_holding_rim`, `side_holding_rim`); beside any other round face (a
            // cone, a ball) it is refused by name, that face's meeting with the ring not yet in
            // closed form
            if screw.is_turn() {
                for rim in &rims {
                    let (f,r) = (rim.frame,rim.r.abs());
                    let source = rim.ring(&g);
                    let held = |k: &usize| matches!(tool.faces[*k].surface,Surface::Plane(_) | Surface::Cylinder(..));
                    if !source.simple(eps) || !whole && !rim.faces.iter().all(held) { return Err(Unbuilt::Curved {at:f.o}) }
                    let [w0,w1] = rim.span;
                    let sites: Vec<(V,V)> = (0..=4*SITES).flat_map(|i| {
                        let (sp,cp) = (w0+(w1-w0)*(i as f64+0.5)/(4*SITES+1) as f64).dsin_cos();
                        let x = f.at([r*cp,r*sp,0.]);
                        let n = unit(cross(add(scale(f.x,-sp),scale(f.y,cp)),screw.velocity(x)));
                        poses.iter().map(move |m| (m.point(x),m.vector(n)))
                    }).collect();
                    let ring = crate::turned::Turned::rim(g,source,[rim.span[0],rim.span[0]+std::f64::consts::TAU]);
                    let ring_at = add_cutter(Cutter::Turned(ring),sites);
                    // each face holding it at each end pose meets its ring in closed form, given (#810:
                    // a plane in two circles, `ssi::plane_holding_rim`; a cylinder in the circle and
                    // the loop crossing it, `ssi::side_holding_rim`)
                    let curve = Curve::Turned(std::sync::Arc::new(ring));
                    for (j,pose) in ends.iter().enumerate() {
                        for &fk in &rim.faces {
                            let Some(&at) = at_ends.get(&(fk,j)) else { continue };
                            let met = match tool.faces[fk].surface.moved(&crate::geom::Rigid {r:pose.r,t:pose.p}) {
                                Surface::Plane(pl) => crate::ssi::plane_holding_rim(&pl,&curve,tol),
                                Surface::Cylinder(cy,r) => crate::ssi::side_holding_rim(&cy,r.abs(),&curve,tol),
                                _ => None,
                            };
                            if let Some(curves) = met { touches.push(Touch {cutters:[at,ring_at],curves}) }
                        }
                    }
                }
            }
            // under a screw a circle square to the axis is screwed, the solid its disc sweeps
            // (`Cutter::ScrewedDisc`; under a turn it is a level curve, below)
            else { for rim in &rims {
                let (f,r) = (rim.frame,rim.r);
                // the screwed sheet's normal: square to the circle and to the screw's velocity
                let normal_at = |x: V| unit(cross(cross(axis,unit(sub(x,f.o))),screw.velocity(x)));
                let sites: Vec<(V,V)> = (0..=SITES).flat_map(|i| {
                    let (sp,cp) = (rim.span[0]+(rim.span[1]-rim.span[0])*within(i)).dsin_cos();
                    let x = f.at([r*cp,r*sp,0.]);
                    let n = normal_at(x);
                    poses.iter().map(move |m| (m.point(x),m.vector(n)))
                }).collect();
                add_cutter(Cutter::ScrewedDisc {c:f.o,r:r.abs()},sites);
            } }
            // under a turn a curve in a level plane (a circle about a line along the axis, an ellipse
            // or a conic a level plane cuts from a leaning round face, #780 C3) turns within it
            for points in &levels {
                let sites: Vec<(V,V)> = points.iter().flat_map(|&x| poses.iter().map(move |m| (m.point(x),axis))).collect();
                add_cutter(Cutter::Plane((points[0],axis)),sites);
            }
            for &(p,q,_) in &edges {
                if let Some(cutter) = line_cutter(p,q) { add_cutter(cutter,line_sites(p,q,cutter)); }
            }
        }
    }
    // a box holding the sweep: the tool's box at each leg's end poses and, under a turn, its reach
    // about the axis; under a screw (a lone leg), a box square to the axis and as tall along it as
    // the sweep, a little more (`tall`: its heights along the axis and its farthest from it), so a
    // sheet a level edge screws into leaves it within a turn where the sweep climbs less than a
    // turn's step. Cut from a blank, the box is the sweep's reach (#847): every candidate reaches
    // across it, and the blank is split by them within it alone (`boolean::arrangement_reached`)
    let (screw,[a,b]) = (legs[0].0,legs[0].1);
    let path_one = Trajectory::Twist(screw);
    let pose_at = |t: f64| path_one.pose(t).map_err(Unbuilt::from);
    let (domain,centre,half,tall) = if legs.iter().all(|l| l.0.is_slide() || l.0.is_turn()) {
        let (domain,centre,half) = cube_holding(&held_points,tol)?;
        (domain,centre,half,None)
    } else {
        // the tool's box carried over the roll, read at enough poses that the arcs its corners run
        // between them stand within the sagitta added
        let f = Frame::about(screw.origin,screw.axis);
        const STEPS: usize = 64;
        let (mut l,mut h) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
        let mut reach = 0f64;
        for k in 0..=STEPS {
            let m = pose_at(a+(b-a)*k as f64/STEPS as f64)?;
            for &p in &corners {
                let x = f.local(m.point(p));
                reach = reach.max(x[0].dhypot(x[1]));
                for i in 0..3 { l[i] = l[i].min(x[i]); h[i] = h[i].max(x[i]) }
            }
        }
        let sag = reach*(1.-(0.5*(screw.ratio*(b-a)).abs()/STEPS as f64).dcos());
        let margin = 0.05*(0..3).map(|i| h[i]-l[i]).fold(0.,f64::max)+tol;
        let pad = [sag+margin,sag+margin,margin];
        for i in 0..3 { l[i] -= pad[i]; h[i] += pad[i] }
        let at = |x: f64,y: f64| f.at([x,y,l[2]]);
        let rim = [at(l[0],l[1]),at(h[0],l[1]),at(h[0],h[1]),at(l[0],h[1])];
        let section = Profile {origin:rim[0],normal:screw.axis,names:Vec::new(),
            loops:vec![(0..4).map(|k| ProfileEdge::Line {a:rim[k],b:rim[(k+1)%4]}).collect()]};
        let half = 0.5*(0..3).map(|i| h[i]-l[i]).fold(0.,f64::max);
        let far = l[0].abs().max(h[0].abs()).dhypot(l[1].abs().max(h[1].abs()));
        (prism(&section,0.,h[2]-l[2])?,f.at(std::array::from_fn(|i| 0.5*(l[i]+h[i]))),half,Some((l[2],h[2],far)))
    };
    // only the candidates that carry some of the boundary: membership differing on their two sides
    // somewhere over the box (a candidate is kept where a point of it cannot be settled)
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll:whole_roll};
    // (each candidate's place among those carried, which a touch names it by)
    let carrying: Vec<(usize,Cutter)> = cutters.iter().enumerate().filter(|(_,(_,sites))| carries(&swept,sites,half))
        .map(|(i,&(c,_))| (i,c)).collect();
    // a build past its budget goes no further (#777 V3: refused by `metered`, naming the stage)
    crate::util::meter::go_on()?;
    // **the box split by every candidate at once** (#735 A4, `boolean::arrangement_within`): each
    // candidate a solid one of whose faces lies on it across the box — a plane a block standing on
    // it, far wider than the box; a surface of revolution its solid over the heights its source
    // spans; a screwed line's sheet itself, reaching past the box — their pieces outside the box
    // left out. Where two candidates touch (a face at an end
    // pose and its own contact surface along a line, a hyperboloid and the cylinder of its throat's
    // radius round a circle) the arrangement lays the touch as an edge of both, so the cells on its
    // two sides are two
    let reach = 4.*half;
    let mut solids: Vec<Brep> = Vec::new();
    // the solids each candidate entered where it is carried (a hyperboloid may enter two), by its
    // index among `cutters`, as a touch names it
    let mut entered: Vec<Option<std::ops::Range<usize>>> = vec![None;cutters.len()];
    for &(i,c) in &carrying {
        let first = solids.len();
        match c {
            Cutter::Plane((o,n)) => {
                let foot = sub(centre,scale(n,dot(sub(centre,o),n)));
                solids.push(prism(&square(foot,n,reach),0.,reach)?)
            }
            Cutter::Cylinder {o,axis,r,span} => solids.push(seam_away(prism(&disc(add(o,scale(axis,span[0])),axis,r),0.,span[1]-span[0])?,o,axis,&seams)),
            Cutter::Revolved {o,axis,p,e,span} => solids.extend(revolved(o,axis,p,e,span,eps)?),
            Cutter::Turned(t) => solids.push(turned_solid(&t)?),
            Cutter::Contour {halves,span} => solids.push(contour_solid(&halves,span)?),
            Cutter::Sheet {arc} => {
                // (along the slide past the whole domain both ways, so no rim but its sides stands in it)
                let mid = dot(sub(centre,arc.frame.o),arc.slide());
                solids.push(contour_sheet(&arc,[mid-2.*half,mid+2.*half])?)
            }
            Cutter::Ball {c,axis,r} => solids.push(crate::build::sphere(c,axis,r)?),
            Cutter::ScrewedDisc {c,r} => {
                let first = pose_at(a)?;
                let rate = screw.advance/std::f64::consts::TAU;
                solids.push(crate::build::screwed(&disc(first.point(c),screw.axis,r),screw.origin,screw.axis,screw.ratio*(b-a),rate*(b-a))?)
            }
            Cutter::Pipe {c,n,r,span:[s0,s1]} => {
                let first = pose_at(s0)?;
                let rate = screw.advance/std::f64::consts::TAU;
                solids.push(crate::build::screwed(&disc(first.point(c),first.vector(n),r),screw.origin,screw.axis,screw.ratio*(s1-s0),rate*(s1-s0))?)
            }
            Cutter::Torus {c,axis,major,minor} => solids.push(seam_away(crate::build::torus(c,axis,major,minor)?,c,axis,&seams)),
            Cutter::Oblique {c,a,r,d} => {
                // the disc's cylinder along `a` as high as the slide climbs it, sheared
                // `x ↦ x + ((x − c)·a) w` so that `a h` goes to `d`
                let h = dot(d,a);
                let w = sub(scale(d,1./h),a);
                let m = std::array::from_fn(|i| std::array::from_fn(|j| if i == j { 1. } else { 0. }+w[i]*a[j]));
                let shear = crate::affine::Affine::new(m,scale(w,-dot(c,a)))?;
                solids.push(seam_away(prism(&disc(c,a,r),0.,h)?,c,a,&seams).mapped(&shear))
            }
            Cutter::Patch {p,q} => solids.push(patch(&screw,&pose_at,p,q,[a,b])?),
            Cutter::Screwed {p,e} => {
                solids.push(screwed_sheet(&screw,&pose_at,p,e,tall.expect("a level edge's sheet is a screw's, its box tall"),size)?)
            }
        }
        entered[i] = Some(first..solids.len());
    }
    // (a candidate whose solid stands clear of the box parts none of it: left out, so its meetings
    // with the others there are never sought)
    let (within,reached) = match blank { Some(b) => (b,Some(&domain)),None => (&domain,None) };
    let refs = clear_of(&solids,&domain,tol);
    // **the touches given** (#810, #780 R5): each between the faces of its two candidates' solids
    // that hold it, laid with no search for it — where both candidates were carried
    let given: Vec<crate::boolean::Given> = touches.iter().filter_map(|t| {
        let ranges = t.cutters.map(|c| entered[c].clone());
        let [Some(r0),Some(r1)] = ranges else { return None };
        let points = t.samples();
        let holds = |s: &&Surface| points.iter().all(|&x| s.implicit(x).abs() <= 64.*tol);
        let surface = |r: std::ops::Range<usize>| solids[r].iter().flat_map(|b| &b.faces).map(|f| &f.surface).find(holds).cloned();
        Some(crate::boolean::Given {surfaces:[surface(r0)?,surface(r1)?],curves:t.curves.clone()})
    }).collect();
    arranged_or_dumped(within,reached,&refs,&given,&swept,blank,tol)
}

/// A box's eight corners.
fn corners_of((lo,hi): (V,V)) -> Vec<V> {
    (0..8).map(|k| std::array::from_fn(|i| if (k>>i)&1 == 0 { lo[i] } else { hi[i] })).collect()
}

/// A cube holding `points` with a fifth of its size to spare each way: the box, its centre and half
/// its side.
pub(super) fn cube_holding<'a>(points: impl IntoIterator<Item=&'a V>,tol: f64) -> Result<(Brep,V,f64),Unbuilt> {
    let (blo,bhi) = points.into_iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(l,h),p|
        (std::array::from_fn(|k| l[k].min(p[k])),std::array::from_fn(|k| h[k].max(p[k]))));
    let centre: V = std::array::from_fn(|k| 0.5*(blo[k]+bhi[k]));
    let half = (0..3).map(|k| bhi[k]-blo[k]).fold(0.,f64::max)*0.6+tol;
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
    let revolved = super::patches::Revolved::read(tool)?;
    let fit = super::patches::fitted(&revolved,path,[a,b],K2_FIT*tol)?;
    crate::util::meter::go_on()?;
    let ends = [a,b].map(|t| super::patches::posed(tool,&revolved,path,&fit,t));
    let [start,end] = [ends[0].clone()?,ends[1].clone()?];
    let (sheet,given) = super::patches::entered(&fit,&revolved,[&start,&end])?;
    candidates(tool,path,[a,b],bar,tol,blank,[start,end],vec![sheet],&given)
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
    let ring = super::rings::Ring::read(tool)?;
    let fit = super::rings::fitted(&ring,path,[a,b],K2_FIT*tol)?;
    if crate::timing() { eprintln!("ring: fitted in {:?}",clock.elapsed()) }
    crate::util::meter::go_on()?;
    let [start,end] = [super::rings::posed(tool,&ring,path,&fit,a)?,super::rings::posed(tool,&ring,path,&fit,b)?];
    let poses = [path.pose(a)?,path.pose(b)?];
    let (sheets,given) = super::rings::entered(&fit,&ring,[&start,&end],poses,tol)?;
    if crate::timing() { eprintln!("ring: entered in {:?}",clock.elapsed()) }
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
        let member = swept.member(p).ok()?;
        let should = match &blank {
            None => member,
            Some(b) => match b.solid_place(p) { Place::In => !member,Place::Out => false,Place::On => return None },
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
fn rulings_span(tool: &Brep,k: usize) -> (f64,f64) {
    let face = &tool.faces[k];
    face.loops.iter().flatten().flat_map(|c| {
        let e = &tool.edges[c.edge as usize];
        (0..=8).map(move |i| e.point(e.t[0]+(e.t[1]-e.t[0])*i as f64/8.,&tool.vertices))
    }).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),p| { let v = face.surface.inverse(p)[1]; (lo.min(v),hi.max(v)) })
}

/// A round face's own points: its surface over a grid of its parameters round the axis and along
/// its rulings over `span`, those in the face.
fn round_points(located: &crate::query::Located,tool: &Brep,k: usize,[lo,hi]: [f64;2]) -> Vec<V> {
    let surface = &tool.faces[k].surface;
    let mut out = Vec::new();
    for i in 0..2*SITES { for j in 0..SITES {
        let (u,v) = ((i as f64+0.5)*std::f64::consts::TAU/(2*SITES) as f64,lo+(hi-lo)*(j as f64+0.5)/SITES as f64);
        let p = surface.point([u,v]);
        if located.face_place(k,p) == crate::query::Place::In { out.push(p) }
    } }
    out
}

/// A round face's ruling at `u` over `span`, where its normal `w` is square to the slide, swept:
/// the plane tangent to the face there, holding the slide, read at the ruling's points carried over
/// the roll (none where the ruling is not on the face).
fn tangent_plane(add_cutter: &mut dyn FnMut(Cutter,Vec<(V,V)>) -> usize,located: &crate::query::Located,tool: &Brep,k: usize,u: f64,
    [lo,hi]: [f64;2],w: V,start: &crate::trajectory::Pose,poses: &[crate::trajectory::Pose]) {
    let surface = &tool.faces[k].surface;
    let at = |s: f64| surface.point([u,lo+(hi-lo)*s]);
    if located.face_place(k,at(0.5)) == crate::query::Place::Out { return }
    let sites = (0..=SITES).flat_map(|i| { let x = at((i as f64+0.5)/(SITES+1) as f64); poses.iter().map(move |m| (m.point(x),w)) }).collect();
    add_cutter(Cutter::Plane((start.point(at(0.5)),w)),sites);
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

/// **A plane face's contact under a screw, leaning to its axis** (#777 M5): its points where the
/// screw's velocity lies in the plane, `n · v(x) = 0` (affine in `x`, so a plane), met with the
/// face's own plane in a line, clipped to the face's polygon `corners` (convex): the chord's ends,
/// or `None` where the line misses the face.
fn leaning_contact(screw: &crate::trajectory::Screw,o: V,n: V,corners: &[V],eps: f64) -> Option<(V,V)> {
    let a = |x: V| dot(n,screw.velocity(x));
    let a0 = a(o);
    let g: V = std::array::from_fn(|i| { let mut e = [0.;3]; e[i] = 1.; a(add(o,e))-a0 });
    let e = cross(n,g);
    if norm(e) <= 1e-12*norm(g).max(1e-300) { return None }
    let e = unit(e);
    // the line's point: on the plane, on `a = 0`, and level with `o` along it
    let x0 = crate::ssi::solve3([n,g,e],[dot(n,o),dot(g,o)-a0,dot(e,o)]);
    // where it crosses each side of the polygon, its parameter along `e`
    let side = unit(cross(n,e));
    let mut ts: Vec<f64> = Vec::new();
    for k in 0..corners.len() {
        let (u,v) = (corners[k],corners[(k+1)%corners.len()]);
        let (du,dv) = (dot(sub(u,x0),side),dot(sub(v,x0),side));
        if (du > eps && dv > eps) || (du < -eps && dv < -eps) || (du-dv).abs() <= 1e-300 { continue }
        let s = (du/(du-dv)).clamp(0.,1.);
        ts.push(dot(sub(add(u,scale(sub(v,u),s)),x0),e));
    }
    let (t0,t1) = ts.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),&t| (l.min(t),h.max(t)));
    (t1-t0 > eps).then(|| (add(x0,scale(e,t0)),add(x0,scale(e,t1))))
}

/// **The patch a segment leaning to a screw's axis sweeps over the roll `[a, b]`** (#777 M5): one
/// open face of the ruled helicoid, the segment lengthened a twentieth past each end and the roll
/// a twentieth past each of its, so its rim stands just past the candidates that seal it, inside
/// the domain — a sheet bounded inside a cell, whose meetings stop partway across the faces it
/// meets. It is the face of a triangle on the segment screwed (`build::screwed`), the triangle's
/// other faces left off.
fn patch(screw: &crate::trajectory::Screw,pose_at: &dyn Fn(f64) -> Result<crate::trajectory::Pose,Unbuilt>,p: V,q: V,[a,b]: [f64;2])
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
fn meridian(o: V,axis: V,p: V,e: V) -> (f64,f64,f64) {
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

/// **A cylinder's or a cone's face `k` about an axis leaning to the turn `screw`'s, its contact
/// turned** (#780 C3): on each ring at slant `v` over `span` it touches where `A cos u + B sin u =
/// C` (`turned::Contact`), two curves along the face, each turned into a surface of revolution about
/// the turn's axis whose meridian is read exactly (`Cutter::Turned`), its sites the curve's points
/// on the face carried over the roll with the face's normal there (the turned surface's: it touches
/// the face there). An axis in one plane with the turn's touches along its two generators in that
/// plane, lines, each turned as any ruling is (`ruling`, a hyperboloid) — refused where it crosses the
/// turn's axis within the face, whose ring there touches all round. Refused by what it is
/// (`Unbuilt::Curved`) where a contact is no curve over the span (the two roots on a ring meeting,
/// or the curve reaching the axis) or its meridian folds back along the axis, which its solid,
/// closed down to the axis at both ends, cannot hold.
fn turned_contacts(add_cutter: &mut dyn FnMut(Cutter,Vec<(V,V)>) -> usize,located: &crate::query::Located,
    face: &crate::topo::Face,k: usize,screw: &crate::trajectory::Screw,(f,r,sa,ca): (Frame,f64,f64,f64),[lo,hi]: [f64;2],
    poses: &[crate::trajectory::Pose],ruling: &dyn Fn(f64) -> Option<(Cutter,Vec<(V,V)>)>,eps: f64)
    -> Result<Vec<(usize,crate::turned::Turned)>,Unbuilt> {
    use crate::turned::{Contact,Jet,Turned};
    let (c,axis) = (screw.origin,screw.axis);
    let a = unit(f.z);
    let carried = |x: V| -> Vec<(V,V)> {
        if located.face_place(k,x) == crate::query::Place::Out { return vec![] }
        let n = outward(face,x);
        poses.iter().map(|m| (m.point(x),m.vector(n))).collect()
    };
    // an axis in one plane with the turn's: its generators in that plane, turned (`ruling`)
    let np = cross(a,axis);
    if dot(unit(np),sub(f.o,c)).abs() <= eps {
        // where it crosses the turn's axis, along its own, within the face: a ring there touches
        // all round
        if foot_on(f.o,a,c,axis).is_some_and(|v| v >= lo*ca-eps && v <= hi*ca+eps) {
            return Err(Unbuilt::Curved {at:f.o})
        }
        let w = unit(cross(np,a));
        for w in [w,scale(w,-1.)] {
            if let Some((cutter,sites)) = ruling(dot(w,f.y).datan2(dot(w,f.x))) { add_cutter(cutter,sites); }
        }
        return Ok(Vec::new())
    }
    let g = Frame::about(c,axis);
    // the meridian `PAST` past the face's span each way, so its ends stand clear of the face's own
    // at the end poses (where the face's rim, the meridian's end and the touch along the contact
    // would meet at one point), as a patch's under a screw do (#777 M5)
    let past = PAST*(hi-lo);
    let span = [lo-past,hi+past];
    let mut entered = Vec::new();
    for side in [1.,-1.] {
        let contact = Contact {o:g.local(f.o),a:g.dir_local(a),r,sa,ca,side};
        let turned = Turned::meridian(g,contact,span);
        let at = g.at(contact.point(Jet::at(0.5*(lo+hi))).0.map(|j| j.v));
        if !turned.defined(span) { return Err(Unbuilt::Curved {at}) }
        let sites: Vec<(V,V)> = (0..=SITES).flat_map(|i| {
            let x = g.at(contact.point(Jet::at(lo+(hi-lo)*within(i))).0.map(|j| j.v));
            carried(x)
        }).collect();
        if !sites.is_empty() { entered.push((add_cutter(Cutter::Turned(turned),sites),turned)) }
    }
    Ok(entered)
}

/// **A leaning round face's touches with its own contact turned** (#780 C3, given by construction
/// since #810): the face (`surface`, the tool's face `k`) at each end pose `ends[j]`
/// (its candidate `at_ends[(k, j)]`) touches each surface of revolution its contact turns into
/// (`turned`: the candidates and their meridians) along that contact turned to the pose — the
/// curve `ssi`'s `own_contact` writes, given where it holds it whole, the face standing on one side
/// of the surface wherever both reach; where it does not, nothing is given and the two are traced.
fn own_contacts(surface: &Surface,k: usize,turned: &[(usize,crate::turned::Turned)],ends: &[&crate::trajectory::Pose],
    at_ends: &std::collections::BTreeMap<(usize,usize),usize>,tol: f64) -> Vec<Touch> {
    let mut out = Vec::new();
    for (j,pose) in ends.iter().enumerate() {
        let Some(&at) = at_ends.get(&(k,j)) else { continue };
        let posed = surface.moved(&crate::geom::Rigid {r:pose.r,t:pose.p});
        for &(i,t) in turned {
            if let Some(curves) = crate::ssi::own_contact(&posed,&Curve::Turned(std::sync::Arc::new(t)),tol) {
                out.push(Touch {cutters:[at,i],curves})
            }
        }
    }
    out
}

/// **The stretches of the roll `[a, b]` a ball's pipe is entered in** (#780 L1, X5), the ball of
/// radius `r` about `c` and its great circle square to the unit `n`, its centre's velocity under
/// `screw`. In the screw's helical coordinates `(ρ, ψ, ζ = z − κψ)` the screw is a shift in `ψ`, so
/// the pipe is the circle's points' helices, and two of its stretches `t` of a turn apart meet
/// where the circle holds two points at one `ρ` whose `ζ` differ by a whole number of turns' rise
/// (`geom::Pipe::returns`): `t` is then their `ψ` apart less those turns, at least `2π − 2w` (`w`
/// the circle's reach round the axis from its centre, `Pipe::spread`). A pipe whose roll is short of
/// that, or whose circle stands at no two such points, meets itself nowhere, and is entered whole,
/// as before; one that comes back onto itself, in stretches of at most half
/// a turn each, each meeting itself nowhere, which the arrangement meets pairwise like any two
/// candidates (`ssi`'s `pipes_meeting`: helices, and the circle where two abut). Refused, by what it
/// is, where the circle reaches the axis (`w` none: then two points less than a turn apart along
/// one helix may meet) or where two of its points stand on one helix less than a turn apart; and by
/// name (`Unbuilt::Folded`) where the ball's radius reaches the helix's radius of curvature and the
/// pipe folds (its fold locus, `(ρ² + κ²)/ρ`, the design's step after).
fn pipe_stretches(screw: &crate::trajectory::Screw,c: V,n: V,r: f64,[a,b]: [f64;2],eps: f64) -> Result<Vec<[f64;2]>,Unbuilt> {
    use std::f64::consts::{PI,TAU};
    let pipe = crate::geom::Pipe {carry:crate::geom::Carry::Screw {center:screw.origin,axis:unit(screw.axis),angle:screw.ratio,
        advance:screw.advance/TAU},frame:Frame::about(c,n),r};
    let refused = || Unbuilt::Curved {at:c};
    let (Some(rho),Some(crate::geom::Carry::Screw {advance:kappa,..})) = (pipe.offset(),pipe.radian()) else { return Err(refused()) };
    // the helix's radius of curvature, `(ρ² + κ²)/ρ`: a ball reaching it folds its pipe
    if !(r < (rho*rho+kappa*kappa)/rho.max(1e-300)) { return Err(Unbuilt::Folded {at:c}) }
    let Some(w) = pipe.spread() else { return Err(refused()) };
    let turned = (screw.ratio*(b-a)).abs();
    // nor may it meet itself within less than a turn, two of its circle's points at one `ρ` and
    // `ζ` (the circle's orbits crossing; past a thousandth of a radian, where only the circle's own
    // points stand, the helices crossing its plane all one way), within the roll
    let near = (2.*w).min(turned)+1e-3;
    if near > 2e-3 && pipe.returns([1e-3,near],eps).is_none_or(|r| !r.is_empty()) { return Err(refused()) }
    let back = TAU-2.*w;
    if turned < back-1e-9 { return Ok(vec![[a,b]]) }
    let returns: Vec<f64> = pipe.returns([back-1e-9,turned+1e-9],eps).ok_or_else(refused)?.into_iter().map(|[..,t]| t).collect();
    if returns.is_empty() { return Ok(vec![[a,b]]) }
    // the joints are the build's choice, not the sweep's: placed so that no joint stands within a
    // twentieth of a radian of a return's turn from an end or another joint, where the two
    // circles would cross one return's helix at one point (two stretches' joints a degenerate
    // vertex of three curves), which left a cell too thin to settle. Evenly spaced, those apart
    // are the spacing's multiples (the two ends' own distance, the whole roll, is no joint's)
    let clear = |n: usize| (1..n).all(|i| { let x = turned*i as f64/n as f64; returns.iter().all(|&t| (x-t.abs()).abs() > 0.05) });
    let fewest = (turned/PI).ceil().max(2.) as usize;
    let pieces = (fewest..fewest+16).find(|&n| clear(n)).ok_or_else(refused)?;
    Ok((0..pieces).map(|i| [a+(b-a)*i as f64/pieces as f64,a+(b-a)*(i+1) as f64/pieces as f64]).collect())
}

/// **How far past its source's ends a candidate bounded inside the domain reaches**, as a fraction
/// of the source's stretch: a patch past its segment and its roll (#777 M5), a turned contact's
/// meridian past its face's span (#780 C3, the face's own surface at the end poses half as far), so
/// no end of one stands on another's.
const PAST: f64 = 0.05;

/// Where the line through `o` along the unit `a` passes nearest the line through `c` along the
/// unit `z`, as its distance along `a` from `o`; none where the two run parallel.
fn foot_on(o: V,a: V,c: V,z: V) -> Option<f64> {
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
fn contour_solid(halves: &[crate::slid::Slid;2],[h0,h1]: [f64;2]) -> Result<Brep,String> {
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
fn contour_sheet(arc: &crate::slid::Slid,[h0,h1]: [f64;2]) -> Result<Brep,String> {
    let [a,b] = arc.span;
    let (p,q) = (arc.d2(a).0,arc.d2(b).0);
    let profile = Profile {origin:arc.frame.o,normal:arc.slide(),names:vec![vec!["sheet".into(),String::new()]],
        loops:vec![vec![ProfileEdge::Exact(Curve::Slid(std::sync::Arc::new(*arc))),ProfileEdge::Line {a:q,b:p}]]};
    let sheet = prism(&profile,h0,h1)?;
    Ok(only_sheet(sheet))
}

/// **A touch known by construction** (#810, #780 R5): two candidates, by their index among the
/// build's (`cutters`), and the curves that are the whole meeting of the faces of their solids that
/// hold them (`boolean::Given`).
#[derive(Clone,Debug)]
struct Touch { cutters: [usize;2],curves: Vec<Curve> }

impl Touch {
    /// Points along its curves, over each one's own span (a line about its point).
    fn samples(&self) -> Vec<V> {
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
fn meridian_arc(at: &dyn Fn(f64,f64) -> V,(throat,height,k): (f64,f64,f64),a: [f64;2],b: [f64;2]) -> Result<ProfileEdge,String> {
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
