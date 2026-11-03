//! **The swept solid, closed** (#721 X6, docs/swept-solid-design.md): `sweep(S, M)` built as one
//! valid B-rep, independent of any blank, by the design's pipeline. The candidate faces of the
//! boundary are found, a box holding the sweep is split by them, each cell is classified by
//! membership (`member::Swept::member`, the definition read without the drawing), and the kept
//! cells are united.
//!
//! **Built here: a polyhedral tool** (every face a plane, every edge a line) **under a slide or a
//! turn**, however its edges lean to the axis, **and under a screw where its edges run along the
//! axis or square to it** (and so its faces); **and under a slide a tool whose faces are planes,
//! cylinders, cones and balls and whose edges are lines and circles**, **and under a turn one
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
//!   along `D` (each touching the torus round its meridian, met in closed form, `ssi`);
//! - under a turn: every face at both end poses; every edge turned; a face square to the axis, its
//!   own plane (kept in itself, O); every other face's contact line (its points whose normal passes
//!   through the axis; it meets the axis or runs along it) turned, over the stretch the face spans;
//!   a cylinder's or a cone's face about an axis along the turn's at both end poses (its solid
//!   cylinder or frustum) and its two generators in the plane through both axes turned (a cylinder
//!   or a cone about the turn's axis, the face itself where coaxial); a ball's face at both end poses
//!   and its great circle in the plane through the axis and its centre turned, a torus; a circle
//!   square to the axis, its own plane.
//! - under a screw: every face at both end poses; an edge along the axis, and a face along it, its
//!   contact line (its normal square to the screw's velocity there: its line nearest the axis, as
//!   under a turn), a cylinder about the axis over the heights the source spans, run on by the
//!   step the roll takes; an edge square to the axis, the helicoidal sheet it screws into
//!   (`screwed_sheet`). A face square to the axis has no contact (its normal meets the velocity by
//!   the step everywhere); an edge or a face leaning to the axis screws into a surface no closed
//!   solid or sheet holds yet, and is refused (`Unbuilt::Leaning`).
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
//! **A convex polyhedral tool turned whole is built by another road** (`turned_whole`): its solid
//! of revolution, the radii each slice holds turned once, with no Boolean. Anything else — a curved
//! face under a screw, a cylinder's or a cone's about an axis leaning to a turn's, a ball reaching
//! across a turn's axis, a torus's under a turn or under a slide leaning to its axis, a free-form
//! face, a curve no line or circle,
//! a leaning edge under a screw, another motion — is refused by what it is (`Unbuilt`); those are
//! the design's later steps.
use crate::boolean::arrangement_within;
use crate::build::{Profile,ProfileEdge,disc,prism,square};
use crate::geom::{Curve,Frame,Surface};
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
    /// a face no plane, cylinder, cone or ball, or an edge no line or circle; under a turn, a
    /// cylinder or a cone about an axis leaning to the turn's, a ball reaching across its axis, or a
    /// circle leaning to it: its envelope sheets are curved past what the candidates hold, the
    /// design's later steps.
    Curved { at: V },
    /// The motion is no slide, turn or screw, nor a path of slides and turns (each taken up where
    /// the one before left the tool, `Trajectory::Placed`): the design's later steps.
    Motion,
    /// Under a screw, an edge or a face leaning to the axis (neither along it nor square to it), at
    /// `at`: its sheet a screw surface no closed solid yet holds, the design's later steps.
    Leaning { at: V },
    /// A cell's membership could not be settled: its deepest point stands within the mesh's sag
    /// of the swept boundary.
    Unsettled { at: V },
    /// The kernel could not do it.
    Failed(String),
}

impl From<String> for Unbuilt { fn from(m: String) -> Unbuilt { Unbuilt::Failed(m) } }

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
    /// The torus about `c` square to the unit `axis`, its tube's middle circle of radius `major`, its
    /// tube's radius `minor` (a ball's great circle through the turn's axis, turned).
    Torus { c: V,axis: V,major: f64,minor: f64 },
    /// The disc of radius `r` about `c` square to the screw's axis, screwed over the roll: the solid
    /// a round tool's rim sweeps under a screw, its side the sheet the rim screws into.
    ScrewedDisc { c: V,r: f64 },
    /// The disc of radius `r` about `c` square to the unit `n` screwed over the roll: the solid a
    /// ball's great circle square to its centre's velocity sweeps under a screw, its side the pipe
    /// about the centre's helix.
    Pipe { c: V,n: V,r: f64 },
    /// The ball of radius `r` about `c`, its poles on the unit `axis` (a ball's face at an end
    /// pose, its poles where the tool's are).
    Ball { c: V,axis: V,r: f64 },
    /// The circle of radius `r` about `c` square to the unit `a`, carried along `d`: an oblique
    /// cylinder, the solid the circle's disc sweeps (a cylinder's solid sheared along `d`).
    Oblique { c: V,a: V,r: f64,d: V } }

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
/// (under a slide, a face a cylinder, a cone or a ball too, and an edge a circle; under a turn, a
/// cylinder or a cone about an axis along it, a ball, and a circle square to it); and under a
/// screw every face and edge along the axis or square to it. The constant twist it is, or the first
/// thing that is not, by what it is and where (a point of the face or the edge).
pub fn reads(tool: &Brep,path: &Trajectory) -> Result<crate::trajectory::Screw,Unbuilt> { read(tool,path).map(|r| r.screw) }

/// What `reads` reads of a tool it takes: the constant twist, each face's plane (a point of it and
/// its outward unit normal; for a face no plane, a point of it and no normal), each straight
/// edge's two ends, two faces and their outward normals along it (the same all along a line on a
/// plane, a cylinder or a cone), each face no plane (`round`) and each circle edge where its faces
/// meet at an angle (`rims`): those two only under a slide.
struct Read { screw: crate::trajectory::Screw,faces: Vec<(V,V)>,edges: Vec<(V,V,[usize;2])>,normals: Vec<[V;2]>,
    round: Vec<usize>,rims: Vec<Rim> }

/// A circle edge of the tool: its frame (centre and the circle's normal), radius and span of angle.
#[derive(Clone,Copy,Debug)]
struct Rim { frame: Frame,r: f64,span: [f64;2] }

/// A face's outward unit normal at its point `p`.
fn outward(f: &crate::topo::Face,p: V) -> V {
    let n = unit(f.surface.normal_raw(f.surface.inverse(p)));
    if f.reversed { scale(n,-1.) } else { n }
}

/// `reads`, keeping what it read.
fn read(tool: &Brep,path: &Trajectory) -> Result<Read,Unbuilt> {
    let Some(screw) = path.screw().filter(|s| s.ratio != 0. || s.advance != 0.) else { return Err(Unbuilt::Motion) };
    // a way leaning to a screw's axis: neither along it nor square to it
    let leans = |d: V| !screw.is_turn() && !screw.is_slide() && norm(cross(d,screw.axis)) > 1e-9 && dot(d,screw.axis).abs() > 1e-9;
    let mut faces = Vec::with_capacity(tool.faces.len());
    let mut round = Vec::new();
    for (k,f) in tool.faces.iter().enumerate() {
        let at = f.loops.iter().flatten().next().map_or([0.;3],|c| tool.vertices[tool.ends(c)[0] as usize].p);
        match &f.surface {
            Surface::Plane(frame) => {
                let n = unit(if f.reversed { scale(frame.z,-1.) } else { frame.z });
                if leans(n) { return Err(Unbuilt::Leaning {at}) }
                faces.push((frame.o,n));
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
            // turn's; a ball's along its great circle in the plane through the axis and its centre,
            // turned into a torus
            Surface::Cylinder(g,_) | Surface::Cone(g,..) if screw.is_turn() && norm(cross(g.z,screw.axis)) <= 1e-9 => {
                round.push(k);
                faces.push((at,[0.;3]));
            }
            // under a screw a cylinder's about an axis along it touches along its generators in the
            // plane through both axes, screwed into coaxial cylinders
            Surface::Cylinder(g,_) if !screw.is_turn() && !screw.is_slide() && norm(cross(g.z,screw.axis)) <= 1e-9 => {
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
            // a ball's under a turn along its great circle in the plane through the axis and its
            // centre; under a screw along its great circle square to its centre's velocity, screwed
            // into the pipe about its centre's helix (a ball alone among round faces: beside a side
            // the pipe touches along the helices of the ball's equator, not yet held)
            Surface::Sphere(..) if screw.is_turn() || !screw.is_slide()
                && tool.faces.iter().all(|g| matches!(g.surface,Surface::Sphere(..) | Surface::Plane(_))) => {
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
    let uses = tool.uses();
    let mut edges = Vec::with_capacity(tool.edges.len());
    let mut normals = Vec::with_capacity(tool.edges.len());
    let mut rims = Vec::new();
    for (e,edge) in tool.edges.iter().enumerate() {
        if matches!(edge.curve,EdgeCurve::Degenerate) { continue }
        let (p,q) = (edge.point(edge.t[0],&tool.vertices),edge.point(edge.t[1],&tool.vertices));
        let mid = edge.point(0.5*(edge.t[0]+edge.t[1]),&tool.vertices);
        let [(f1,..),(f2,..)] = uses[e][..] else { return Err(Unbuilt::Failed(format!("edge {e} bounds {} faces",uses[e].len()))) };
        // a seam, one face on both sides of it: no edge of the solid
        if f1 == f2 { continue }
        match &edge.curve {
            EdgeCurve::Curve(Curve::Line {..}) => {
                if leans(unit(sub(q,p))) { return Err(Unbuilt::Leaning {at:mid}) }
                edges.push((p,q,[f1,f2]));
                normals.push([outward(&tool.faces[f1],mid),outward(&tool.faces[f2],mid)]);
            }
            // (a circle where two faces meet smoothly, a ball on a cylinder, is no edge of the
            // solid: its fan is one normal, the faces' own contacts')
            // (under a turn, a circle square to the axis only: it turns within its own plane)
            EdgeCurve::Curve(Curve::Circle(frame,r)) if screw.is_slide() || norm(cross(frame.z,screw.axis)) <= 1e-9 => {
                let (n1,n2) = (outward(&tool.faces[f1],mid),outward(&tool.faces[f2],mid));
                if norm(cross(n1,n2)) > 1e-9 || dot(n1,n2) < 0. { rims.push(Rim {frame:*frame,r:*r,span:edge.t}) }
            }
            _ => return Err(Unbuilt::Curved {at:mid}),
        }
    }
    Ok(Read {screw,faces,edges,normals,round,rims})
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

/// `closed` over a box holding the sweep, or `cut_from` over `blank`.
fn built(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Brep,Unbuilt> {
    // the legs the path is taken in: one constant twist over the roll, or each piece of a path of
    // pieces over its stretch of it (K1: a sweep is its pieces' sweeps united), every one read
    let legs = legs(tool,path,roll)?;
    let whole_roll = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let size = { let (lo,hi) = tool.bounds(); norm(sub(hi,lo)) };
    let eps = 1e-9*(1.+size);
    // each candidate with the sites it is read at for whether it carries any of the boundary: the
    // points of its source carried over the roll, each with the candidate's normal there
    let mut cutters: Vec<(Cutter,Vec<(V,V)>)> = Vec::new();
    let mut add_cutter = |c: Cutter,sites: Vec<(V,V)>| {
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
            _ => false,
        };
        match cutters.iter_mut().find(|(k,_)| same(k)) {
            // one surface from two sources: over both their spans
            Some((k,known)) => {
                if let (Cutter::Cylinder {o,axis,span,..} | Cutter::Revolved {o,axis,span,..},
                    Cutter::Cylinder {o:from,axis:way,span:more,..} | Cutter::Revolved {o:from,axis:way,span:more,..}) = (k,c) {
                    // the other's heights read from this one's origin along this one's axis
                    let (shift,sense) = (dot(sub(from,*o),*axis),dot(way,*axis).signum());
                    let more = [shift+sense*more[0],shift+sense*more[1]];
                    *span = [span[0].min(more[0].min(more[1])),span[1].max(more[0].max(more[1]))];
                }
                known.extend(sites)
            }
            None => cutters.push((c,sites)),
        }
    };
    // the points the box must hold: the tool's box at each leg's end poses and, about a turn's
    // axis, its reach
    let box_corners = |(lo,hi): (V,V)| -> Vec<V> { (0..8).map(|k| std::array::from_fn(|i| if (k>>i)&1 == 0 { lo[i] } else { hi[i] })).collect() };
    let corners = box_corners(tool.bounds());
    let mut held_points: Vec<V> = Vec::new();
    let one = legs.len() == 1;
    for &(screw,[a,b],place) in &legs {
        // the tool where the leg takes it up, read under the leg's twist
        let placed_tool = place.map(|q| tool.moved(&crate::geom::Rigid {r:q.r,t:q.p}));
        let tool: &Brep = placed_tool.as_ref().unwrap_or(tool);
        let path = &Trajectory::Twist(screw);
        let Read {faces,edges,normals,round,rims,..} = read(tool,path)?;
        // each face's corners in walk order, what its own sites are read over
        let corners_of: Vec<Vec<V>> = tool.faces.iter().map(|f| f.loops.iter().flatten()
            .map(|c| tool.vertices[tool.ends(c)[0] as usize].p).collect()).collect();
        let corners = box_corners(tool.bounds());
        let pose_at = |t: f64| path.pose(t).map_err(|e| Unbuilt::Failed(e.msg));
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
        let within = |i: usize| (i as f64+0.5)/(SITES+1) as f64;
        let poses: Vec<crate::trajectory::Pose> = (0..=SITES).map(|k| pose_at(a+(b-a)*within(k))).collect::<Result<_,_>>()?;
        // a face's own points, a grid over each triangle of a fan of its corners
        let face_sites = |k: usize| -> Vec<V> {
            let c = &corners_of[k];
            let mut out = Vec::new();
            for w in 1..c.len().saturating_sub(1) {
                for i in 0..SITES { for j in 0..SITES-i {
                    let (u,v) = ((i as f64+1./3.)/(SITES+1) as f64,(j as f64+1./3.)/(SITES+1) as f64);
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
                        let rr = aa.hypot(bb);
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
                        for pose in [&start,&end] {
                            add_cutter(Cutter::Torus {c:pose.point(f.o),axis:pose.vector(z),major:big,minor:small},sites_at(pose));
                        }
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
                if dot(d,a).abs() <= 1e-9*norm(d) { add_cutter(Cutter::Plane((c,a)),sites) }
                else { add_cutter(Cutter::Oblique {c,a,r:r.abs(),d},sites) }
            }
        } else {
            let (c,axis) = (screw.origin,screw.axis);
            // a convex tool turned whole is the solid of revolution of its radial profile, built in
            // closed form with no Boolean
            let whole = (screw.ratio*(b-a)).abs() >= std::f64::consts::TAU*(1.-1e-12);
            let convex = faces.iter().all(|&(o,n)| tool.vertices.iter().all(|v| dot(sub(v.p,o),n) <= eps));
            if one && screw.is_turn() && whole && convex && round.is_empty() && rims.is_empty() && blank.is_none() {
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
                        Cutter::Oblique {..} | Cutter::Ball {..} | Cutter::Torus {..} | Cutter::ScrewedDisc {..} | Cutter::Pipe {..} =>
                            unreachable!("a line turns into none of these"),
                    };
                    poses.iter().map(move |m| (m.point(x),m.vector(normal)))
                }).collect()
            };
            for (k,&(o,n)) in faces.iter().enumerate() {
                if round.contains(&k) { continue }
                let own = face_sites(k);
                for pose in [&start,&end] {
                    add_cutter(Cutter::Plane((pose.point(o),pose.vector(n))),own.iter().map(|&x| (pose.point(x),pose.vector(n))).collect());
                }
                let m = cross(n,axis);
                // a face square to the axis: under a turn its own plane (kept in itself), under a screw no
                // contact at all (its normal meets the screw's velocity everywhere, by the step)
                if norm(m) <= 1e-9 {
                    if screw.is_turn() { add_cutter(Cutter::Plane((o,n)),poses.iter().flat_map(|pose| own.iter().map(|&x| (pose.point(x),n))).collect()) }
                    continue
                }
                // under a screw a face along the axis touches along its line nearest the axis, as under a
                // turn (`reads` refuses a face leaning to it, along a line screwing into a sheet not yet
                // held)
                // the face's contact line: its plane's points whose normal passes through the axis, where
                // its plane meets the plane through the axis along `n × axis`; it meets the axis, or runs
                // along it
                if dot(n,sub(o,c)).abs() <= eps { continue }
                let (x0,e) = (add(c,scale(n,dot(n,sub(o,c)))),unit(cross(n,m)));
                // the stretch of the contact line the face spans
                let ts: Vec<f64> = corners_of[k].iter().map(|v| dot(sub(*v,x0),e)).collect();
                let (t0,t1) = ts.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),&t| (l.min(t),h.max(t)));
                let (from,to) = (add(x0,scale(e,t0)),add(x0,scale(e,t1)));
                if let Some(cutter) = turned(from,e,to) { add_cutter(cutter,line_sites(from,to,cutter)) }
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
                // the way from the axis to the face's own axis or centre, square to the axis
                let away = |o: V| -> Option<V> { let r = sub(o,c); let w = sub(r,scale(axis,dot(r,axis))); (norm(w) > eps).then(|| unit(w)) };
                match face.surface {
                    Surface::Cylinder(f,r) => {
                        for pose in [&start,&end] {
                            add_cutter(Cutter::Cylinder {o:pose.point(f.o),axis:pose.vector(unit(f.z)),r:r.abs(),span:[lo,hi]},sites_at(pose));
                        }
                        let ways = match away(f.o) { Some(w) => vec![w,scale(w,-1.)],None => vec![] };
                        // coaxial, the face turns into itself
                        if ways.is_empty() { add_cutter(Cutter::Cylinder {o:f.o,axis:unit(f.z),r:r.abs(),span:[lo,hi]},poses.iter().flat_map(|m| sites_at(m)).collect()) }
                        for w in ways {
                            let u = dot(w,f.y).datan2(dot(w,f.x));
                            let (p,q) = (face.surface.point([u,lo]),face.surface.point([u,hi]));
                            if located.face_place(k,face.surface.point([u,0.5*(lo+hi)])) == crate::query::Place::Out { continue }
                            if let Some(cutter) = turned(p,sub(q,p),q) { add_cutter(cutter,line_sites(p,q,cutter)) }
                        }
                    }
                    Surface::Cone(f,_,half) => {
                        let (sa,ca) = half.dsin_cos();
                        let p0 = face.surface.point([0.,0.]);
                        let e = unit(add(scale(f.x,sa),scale(f.z,ca)));
                        for pose in [&start,&end] {
                            add_cutter(Cutter::Revolved {o:pose.point(f.o),axis:pose.vector(f.z),p:pose.point(p0),e:pose.vector(e),span:[lo*ca,hi*ca]},
                                sites_at(pose));
                        }
                        let ways = match away(f.o) { Some(w) => vec![w,scale(w,-1.)],None => vec![] };
                        if ways.is_empty() {
                            add_cutter(Cutter::Revolved {o:f.o,axis:f.z,p:p0,e,span:[lo*ca,hi*ca]},poses.iter().flat_map(|m| sites_at(m)).collect())
                        }
                        for w in ways {
                            let u = dot(w,f.y).datan2(dot(w,f.x));
                            let (p,q) = (face.surface.point([u,lo]),face.surface.point([u,hi]));
                            if located.face_place(k,face.surface.point([u,0.5*(lo+hi)])) == crate::query::Place::Out { continue }
                            if let Some(cutter) = turned(p,sub(q,p),q) { add_cutter(cutter,line_sites(p,q,cutter)) }
                        }
                    }
                    Surface::Sphere(f,r) if !screw.is_turn() => {
                        let r = r.abs();
                        for pose in [&start,&end] {
                            add_cutter(Cutter::Ball {c:pose.point(f.o),axis:pose.vector(f.z),r},sites_at(pose));
                        }
                        // its great circle square to its centre's velocity, screwed: the pipe
                        let n = unit(screw.velocity(f.o));
                        let g = Frame::about(f.o,n);
                        let sites: Vec<(V,V)> = (0..4*SITES).flat_map(|i| {
                            let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
                            let w = add(scale(g.x,cp),scale(g.y,sp));
                            let x = add(f.o,scale(w,r));
                            let on = located.face_place(k,x) != crate::query::Place::Out;
                            poses.iter().filter(move |_| on).map(move |m| (m.point(x),m.vector(w)))
                        }).collect();
                        add_cutter(Cutter::Pipe {c:f.o,n,r},sites);
                    }
                    Surface::Sphere(f,r) => {
                        let r = r.abs();
                        for pose in [&start,&end] {
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
                        for pose in [&start,&end] {
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
            // a circle square to the axis turns within its own plane; under a screw it is screwed, the
            // solid its disc sweeps (`Cutter::ScrewedDisc`)
            for rim in &rims {
                let (f,r) = (rim.frame,rim.r);
                // the screwed sheet's normal: square to the circle and to the screw's velocity
                let normal_at = |x: V| if screw.is_turn() { axis } else { unit(cross(cross(axis,unit(sub(x,f.o))),screw.velocity(x))) };
                let sites: Vec<(V,V)> = (0..=SITES).flat_map(|i| {
                    let (sp,cp) = (rim.span[0]+(rim.span[1]-rim.span[0])*within(i)).dsin_cos();
                    let x = f.at([r*cp,r*sp,0.]);
                    let n = normal_at(x);
                    poses.iter().map(move |m| (m.point(x),m.vector(n)))
                }).collect();
                if screw.is_turn() { add_cutter(Cutter::Plane((f.o,axis)),sites) }
                else { add_cutter(Cutter::ScrewedDisc {c:f.o,r:r.abs()},sites) }
            }
            for &(p,q,_) in &edges {
                let e = unit(sub(q,p));
                if !screw.is_turn() && norm(cross(e,axis)) > 1e-9 {
                    // under a screw an edge square to the axis sweeps a helicoidal sheet (`reads` refuses
                    // one leaning to it, a screw surface not yet held)
                    let cutter = Cutter::Screwed {p,e};
                    add_cutter(cutter,line_sites(p,q,cutter));
                    continue
                }
                if let Some(cutter) = turned(p,sub(q,p),q) { add_cutter(cutter,line_sites(p,q,cutter)) }
            }
        }
    }
    // a box holding the sweep: the tool's box at each leg's end poses and, under a turn, its reach
    // about the axis; under a screw (a lone leg), a box square to the axis and as tall along it as
    // the sweep, a little more (`tall`: its heights along the axis and its farthest from it), so a
    // sheet a level edge screws into leaves it within a turn where the sweep climbs less than a
    // turn's step (cut from a blank, the box holds the blank too: every candidate reaching across it)
    let (screw,[a,b]) = (legs[0].0,legs[0].1);
    let path_one = Trajectory::Twist(screw);
    let pose_at = |t: f64| path_one.pose(t).map_err(|e| Unbuilt::Failed(e.msg));
    let held: Vec<V> = blank.map(|b| box_corners(b.bounds())).unwrap_or_default();
    let (domain,centre,half,tall) = if legs.iter().all(|l| l.0.is_slide() || l.0.is_turn()) {
        let points: Vec<V> = held_points.iter().copied().chain(held.iter().copied()).collect();
        let (blo,bhi) = points.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(l,h),p|
            (std::array::from_fn(|k| l[k].min(p[k])),std::array::from_fn(|k| h[k].max(p[k]))));
        let centre: V = std::array::from_fn(|k| 0.5*(blo[k]+bhi[k]));
        let half = (0..3).map(|k| bhi[k]-blo[k]).fold(0.,f64::max)*0.6+tol;
        (prism(&square(add(centre,[0.,0.,-half]),[0.,0.,1.],half),0.,2.*half)?,centre,half,None)
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
                reach = reach.max(x[0].hypot(x[1]));
                for i in 0..3 { l[i] = l[i].min(x[i]); h[i] = h[i].max(x[i]) }
            }
        }
        let sag = reach*(1.-(0.5*(screw.ratio*(b-a)).abs()/STEPS as f64).cos());
        for &p in &held {
            let x = f.local(p);
            reach = reach.max(x[0].hypot(x[1]));
            for i in 0..3 { l[i] = l[i].min(x[i]); h[i] = h[i].max(x[i]) }
        }
        let margin = 0.05*(0..3).map(|i| h[i]-l[i]).fold(0.,f64::max)+tol;
        let pad = [sag+margin,sag+margin,margin];
        for i in 0..3 { l[i] -= pad[i]; h[i] += pad[i] }
        let at = |x: f64,y: f64| f.at([x,y,l[2]]);
        let rim = [at(l[0],l[1]),at(h[0],l[1]),at(h[0],h[1]),at(l[0],h[1])];
        let section = Profile {origin:rim[0],normal:screw.axis,names:Vec::new(),
            loops:vec![(0..4).map(|k| ProfileEdge::Line {a:rim[k],b:rim[(k+1)%4]}).collect()]};
        let half = 0.5*(0..3).map(|i| h[i]-l[i]).fold(0.,f64::max);
        let far = l[0].abs().max(h[0].abs()).hypot(l[1].abs().max(h[1].abs()));
        (prism(&section,0.,h[2]-l[2])?,f.at(std::array::from_fn(|i| 0.5*(l[i]+h[i]))),half,Some((l[2],h[2],far)))
    };
    // only the candidates that carry some of the boundary: membership differing on their two sides
    // somewhere over the box (a candidate is kept where a point of it cannot be settled)
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll:whole_roll};
    let carrying: Vec<Cutter> = cutters.into_iter().filter(|(_,sites)| carries(&swept,sites,half)).map(|(c,_)| c).collect();
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
    for c in &carrying {
        match *c {
            Cutter::Plane((o,n)) => {
                let foot = sub(centre,scale(n,dot(sub(centre,o),n)));
                solids.push(prism(&square(foot,n,reach),0.,reach)?)
            }
            Cutter::Cylinder {o,axis,r,span} => solids.push(prism(&disc(add(o,scale(axis,span[0])),axis,r),0.,span[1]-span[0])?),
            Cutter::Revolved {o,axis,p,e,span} => solids.extend(revolved(o,axis,p,e,span,eps)?),
            Cutter::Ball {c,axis,r} => solids.push(crate::build::sphere(c,axis,r)?),
            Cutter::ScrewedDisc {c,r} => {
                let first = pose_at(a)?;
                let rate = screw.advance/std::f64::consts::TAU;
                solids.push(crate::build::screwed(&disc(first.point(c),screw.axis,r),screw.origin,screw.axis,screw.ratio*(b-a),rate*(b-a))?)
            }
            Cutter::Pipe {c,n,r} => {
                let first = pose_at(a)?;
                let rate = screw.advance/std::f64::consts::TAU;
                solids.push(crate::build::screwed(&disc(first.point(c),first.vector(n),r),screw.origin,screw.axis,screw.ratio*(b-a),rate*(b-a))?)
            }
            Cutter::Torus {c,axis,major,minor} => solids.push(crate::build::torus(c,axis,major,minor)?),
            Cutter::Oblique {c,a,r,d} => {
                // the disc's cylinder along `a` as high as the slide climbs it, sheared
                // `x ↦ x + ((x − c)·a) w` so that `a h` goes to `d`
                let h = dot(d,a);
                let w = sub(scale(d,1./h),a);
                let m = std::array::from_fn(|i| std::array::from_fn(|j| if i == j { 1. } else { 0. }+w[i]*a[j]));
                let shear = crate::affine::Affine::new(m,scale(w,-dot(c,a)))?;
                solids.push(prism(&disc(c,a,r),0.,h)?.mapped(&shear))
            }
            Cutter::Screwed {p,e} => {
                solids.push(screwed_sheet(&screw,&pose_at,p,e,tall.expect("a level edge's sheet is a screw's, its box tall"),size)?)
            }
        }
    }
    // (a candidate whose solid stands clear of the domain's box parts none of it: left out, so its
    // meetings with the others there are never sought)
    let within = blank.unwrap_or(&domain);
    let (dlo,dhi) = within.bounds();
    let refs: Vec<&Brep> = solids.iter().filter(|s| { let (lo,hi) = s.bounds(); (0..3).all(|k| lo[k] <= dhi[k]+tol && hi[k] >= dlo[k]-tol) }).collect();
    let complex = arrangement_within(within,&refs,tol)?;
    // each cell classified by membership at the deepest of its points that settles (a cusp-thin
    // cell's stand near the swept boundary, and the first may not), and the boundary between the
    // cells kept — the members, or cut from a blank the rest — and the others the result: one
    // selection, no union, the candidates that bound nothing healed out of its faces
    let cells = complex.cells()?.len();
    let mut keep = vec![false;cells];
    for (c,kept) in keep.iter_mut().enumerate().skip(1) {
        let points = complex.inside_points(c,4)?;
        let Some(&first) = points.first() else { continue };
        match points.iter().find_map(|&x| swept.member(x).ok()) {
            Some(m) => *kept = m != blank.is_some(),
            None => return Err(Unbuilt::Unsettled {at:first}),
        }
    }
    if blank.is_none() && !keep.iter().any(|&k| k) { return Err(Unbuilt::Failed("no cell of the arrangement is swept".into())) }
    let out = crate::boolean::selection(&complex,&|c| keep[c])?;
    out.check(tol).map_err(|e| Unbuilt::Failed(e.msg))?;
    Ok(out)
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
fn tangent_plane(add_cutter: &mut dyn FnMut(Cutter,Vec<(V,V)>),located: &crate::query::Located,tool: &Brep,k: usize,u: f64,
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
    let mut sheet = crate::build::screwed(&triangle,c,axis,screw.ratio*(s1-s0),rate*(s1-s0))?;
    sheet.faces.retain(|f| f.name == "sheet");
    Ok(sheet.compacted())
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
    out.check(tol).map_err(|e| Unbuilt::Failed(e.msg))?;
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
