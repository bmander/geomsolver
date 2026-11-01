//! **The swept solid, closed** (#721 X6, docs/swept-solid-design.md): `sweep(S, M)` built as one
//! valid B-rep, independent of any blank, by the design's pipeline. The candidate faces of the
//! boundary are found, a box holding the sweep is split by them, each cell is classified by
//! membership (`member::Swept::member`, the definition read without the drawing), and the kept
//! cells are united.
//!
//! **Built here: a polyhedral tool** (every face a plane, every edge a line) **under a slide, or
//! under a turn when its edges run along the axis or square to it.** The candidates, each extended
//! whole so that it splits the box through:
//! - under a slide `D`: each face at the start pose where it enters the slide (`n·D < 0`), at the
//!   end pose where it leaves it (`n·D > 0`), a face along the slide (`n·D = 0`) at either (one
//!   plane); each silhouette edge (its faces' `n·D` of both signs) swept, in the plane through it
//!   holding `D`;
//! - under a turn about an axis: every face at both end poses; an edge along the axis turned, the
//!   cylinder about the axis through it; an edge square to the axis turned, the plane square to the
//!   axis through it; a face square to the axis, its own plane (kept in itself, O); a face along
//!   the axis, the cylinder its plane touches (its contact line, where the plane's normal passes
//!   through the axis, turned).
//!
//! Only the candidates that carry some of the boundary are kept (`carries`, sampled); a face along
//! the axis touches its own contact cylinder at each end pose, and the plane through the axis and
//! that line is added to part the two sides of the touch, which would otherwise be one cell. The
//! arrangement is the candidates' (a plane by `kept` on each side, a cylinder by the solid cylinder
//! in common and cut away), and a cell is kept where its deepest interior point
//! (`query::interior`) is a member. Anything else — a curved face, an edge or a face leaning to the
//! axis (its line turns into a hyperboloid), a screw, another motion — is refused by what it is
//! (`Unbuilt`); those are the design's later steps.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::boolean::{Op,combined};
use crate::build::{Profile,ProfileEdge,kept,prism,square};
use crate::geom::{Curve,Frame,Surface};
use crate::topo::{Brep,EdgeCurve};
use crate::trajectory::Trajectory;
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use super::member::{Solid,Swept};

type V = [f64;3];

/// Why the closed swept solid is not built.
#[derive(Clone,Debug,PartialEq)]
pub enum Unbuilt {
    /// The tool has a face that is no plane, or an edge that is no line: its envelope sheets are
    /// curved, the design's later steps.
    Curved,
    /// The motion is no slide and no turn: the design's later steps.
    Motion,
    /// Under a turn, an edge or a face leans to the axis: its line turns into a hyperboloid, the
    /// design's later steps.
    Skew,
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
enum Cutter { Plane(Plane),Cylinder { o: V,axis: V,r: f64 } }

/// **The closed solid `tool` sweeps under `path` over `roll`** (the module's cases), its tool,
/// motion and solid in one set of units, built to `tol`; membership read on the tool meshed within
/// `bar`.
pub fn closed(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    let Some(screw) = path.screw().filter(|s| s.ratio != 0. || s.advance != 0.) else { return Err(Unbuilt::Motion) };
    let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let size = { let (lo,hi) = tool.bounds(); norm(sub(hi,lo)) };
    let eps = 1e-9*(1.+size);
    // each face's plane and outward normal, each edge's two ends and two faces
    let mut faces = Vec::with_capacity(tool.faces.len());
    for f in &tool.faces {
        let Surface::Plane(frame) = &f.surface else { return Err(Unbuilt::Curved) };
        faces.push((frame.o,unit(if f.reversed { scale(frame.z,-1.) } else { frame.z })));
    }
    let mut faces_of: Vec<Vec<usize>> = vec![Vec::new();tool.edges.len()];
    for (k,f) in tool.faces.iter().enumerate() {
        for c in f.loops.iter().flatten() { faces_of[c.edge as usize].push(k) }
    }
    let mut edges = Vec::with_capacity(tool.edges.len());
    for (e,edge) in tool.edges.iter().enumerate() {
        if !matches!(edge.curve,EdgeCurve::Curve(Curve::Line {..})) { return Err(Unbuilt::Curved) }
        let [f1,f2] = faces_of[e][..] else { return Err(Unbuilt::Failed(format!("edge {e} bounds {} faces",faces_of[e].len()))) };
        edges.push((edge.point(edge.t[0],&tool.vertices),edge.point(edge.t[1],&tool.vertices),[f1,f2]));
    }
    let mut cutters: Vec<Cutter> = Vec::new();
    // planes that carry none of the boundary but part what a touch between two candidates would
    // join into one cell, kept whatever they carry
    let mut separators: Vec<Plane> = Vec::new();
    let mut add_cutter = |c: Cutter| {
        let same = |k: &Cutter| match (*k,c) {
            (Cutter::Plane((p,m)),Cutter::Plane((o,n))) => norm(cross(n,m)) <= 1e-9 && dot(sub(o,p),m).abs() <= eps,
            (Cutter::Cylinder {r:r1,..},Cutter::Cylinder {r:r2,..}) => (r1-r2).abs() <= eps,
            _ => false,
        };
        if !cutters.iter().any(same) { cutters.push(c) }
    };
    let pose_at = |t: f64| path.pose(t).map_err(|e| Unbuilt::Failed(e.msg));
    let (start,end) = (pose_at(a)?,pose_at(b)?);
    if screw.is_slide() {
        let d = sub(end.point([0.;3]),start.point([0.;3]));
        for &(o,n) in &faces {
            let s = dot(n,d);
            add_cutter(Cutter::Plane((if s > eps { end.point(o) } else { start.point(o) },n)));
        }
        for &(p,q,[f1,f2]) in &edges {
            let (s1,s2) = (dot(faces[f1].1,d),dot(faces[f2].1,d));
            if s1*s2 > 0. || (s1.abs() <= eps && s2.abs() <= eps) { continue }
            let n = cross(sub(q,p),d);
            if norm(n) <= eps*norm(d) { continue }
            add_cutter(Cutter::Plane((start.point(p),unit(n))));
        }
    } else if screw.is_turn() {
        let (c,axis) = (screw.origin,screw.axis);
        let radial = |p: V| { let r = sub(p,c); norm(sub(r,scale(axis,dot(r,axis)))) };
        for &(o,n) in &faces {
            add_cutter(Cutter::Plane((start.point(o),start.vector(n))));
            add_cutter(Cutter::Plane((end.point(o),end.vector(n))));
            if norm(cross(n,axis)) <= 1e-9 { add_cutter(Cutter::Plane((o,n))) }
            else if dot(n,axis).abs() <= 1e-9 {
                let r = dot(n,sub(o,c)).abs();
                if r > eps {
                    add_cutter(Cutter::Cylinder {o:c,axis,r});
                    // the face at each end pose touches that cylinder along its contact line: the
                    // plane through the axis and that line parts the two sides of the touch
                    for pose in [&start,&end] { separators.push((c,unit(cross(axis,pose.vector(n))))) }
                }
            } else { return Err(Unbuilt::Skew) }
        }
        for &(p,q,_) in &edges {
            let e = unit(sub(q,p));
            if norm(cross(e,axis)) <= 1e-9 {
                let r = radial(p);
                if r > eps { add_cutter(Cutter::Cylinder {o:c,axis,r}) }
            } else if dot(e,axis).abs() <= 1e-9 { add_cutter(Cutter::Plane((p,axis))) }
            else { return Err(Unbuilt::Skew) }
        }
    } else { return Err(Unbuilt::Motion) }
    // a box holding the sweep: the tool's box at the end poses and, under a turn, its reach about
    // the axis
    let (lo,hi) = tool.bounds();
    let corners: Vec<V> = (0..8).map(|k| std::array::from_fn(|i| if (k>>i)&1 == 0 { lo[i] } else { hi[i] })).collect();
    let mut points: Vec<V> = corners.iter().flat_map(|&p| [start.point(p),end.point(p)]).collect();
    if screw.is_turn() {
        let reach = corners.iter().map(|&p| norm(sub(p,screw.origin))).fold(0.,f64::max);
        points.extend([1.,-1.].iter().flat_map(|&s| (0..3).map(move |k| { let mut v = [0.;3]; v[k] = s*reach; v }))
            .map(|v| add(screw.origin,v)));
    }
    let (blo,bhi) = points.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(l,h),p|
        (std::array::from_fn(|k| l[k].min(p[k])),std::array::from_fn(|k| h[k].max(p[k]))));
    let centre: V = std::array::from_fn(|k| 0.5*(blo[k]+bhi[k]));
    let half = (0..3).map(|k| bhi[k]-blo[k]).fold(0.,f64::max)*0.6+tol;
    let base = add(centre,[0.,0.,-half]);
    let reach = 4.*half;
    // only the candidates that carry some of the boundary: membership differing on their two sides
    // somewhere over the box (a candidate is kept where a point of it cannot be settled)
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll};
    let mut cutters: Vec<Cutter> = cutters.into_iter().filter(|&c| carries(&swept,c,centre,half)).collect();
    cutters.extend(separators.into_iter().map(Cutter::Plane));
    let mut cells = vec![prism(&square(base,[0.,0.,1.],half),0.,2.*half)?];
    for cutter in cutters {
        let mut next = Vec::with_capacity(cells.len()*2);
        for cell in cells {
            match cutter {
                Cutter::Plane((o,n)) => {
                    let side = |v: &crate::topo::Vertex| dot(sub(v.p,o),n);
                    let (low,high) = cell.vertices.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),v| (l.min(side(v)),h.max(side(v))));
                    if low >= -tol || high <= tol { next.push(cell); continue }
                    for keep in [n,scale(n,-1.)] {
                        let piece = kept(&cell,o,keep,tol)?;
                        if !piece.faces.is_empty() { next.push(piece) }
                    }
                }
                Cutter::Cylinder {o,axis,r} => {
                    let foot = sub(o,scale(axis,reach));
                    let disc = Profile {origin:foot,normal:axis,loops:vec![vec![ProfileEdge::Arc {frame:Frame::about(foot,axis),r,span:None}]],
                        names:Vec::new()};
                    let solid = prism(&disc,0.,2.*reach)?;
                    for op in [Op::Common,Op::Cut] {
                        let piece = combined(&cell,&solid,op,tol).map_err(|e| e.msg)?;
                        if !piece.faces.is_empty() { next.push(piece) }
                    }
                }
            }
        }
        cells = next;
    }
    // each cell classified by membership at its deepest point, the members united
    let mut out: Option<Brep> = None;
    for cell in &cells {
        let Some(&(x,_)) = crate::query::interior(cell,1)?.first() else { continue };
        match swept.member(x) {
            Ok(true) => out = Some(match out { None => cell.clone(),Some(acc) => combined(&acc,cell,Op::Union,tol).map_err(|e| e.msg)? }),
            Ok(false) => {}
            Err(_) => return Err(Unbuilt::Unsettled {at:x}),
        }
    }
    let out = out.ok_or_else(|| Unbuilt::Failed("no cell of the arrangement is swept".into()))?;
    out.check(tol).map_err(|e| Unbuilt::Failed(e.msg))?;
    Ok(out)
}

/// How many points a side a candidate is read at for whether it carries any of the boundary.
const CARRIES: usize = 24;

/// Whether `cutter` carries some of the swept boundary within the box of half-width `half` about
/// `centre`: a point of it, over a `CARRIES`-square grid, where membership a hair to one side
/// differs from a hair to the other, or cannot be settled. Sampled: a candidate whose boundary
/// falls between the samples is dropped, and the cell it would have bounded reads both ways, which
/// the classification refuses.
fn carries(swept: &Swept,cutter: Cutter,centre: V,half: f64) -> bool {
    let hair = 1e-5*half;
    let n = CARRIES;
    let at = |i: usize| -1.+2.*(i as f64+0.5)/n as f64;
    let differs = |p: V,normal: V| match (swept.member(add(p,scale(normal,hair))),swept.member(sub(p,scale(normal,hair)))) {
        (Ok(x),Ok(y)) => x != y,
        _ => true,
    };
    match cutter {
        Cutter::Plane((o,normal)) => {
            let f = Frame::new(sub(centre,scale(normal,dot(sub(centre,o),normal))),normal,crate::util::space::across(normal).0);
            (0..n).any(|i| (0..n).any(|j| differs(f.at([half*at(i),half*at(j),0.]),normal)))
        }
        Cutter::Cylinder {o,axis,r} => {
            let f = Frame::about(o,axis);
            let h = dot(sub(centre,o),axis);
            (0..n).any(|i| (0..n).any(|j| {
                let (sn,cs) = (std::f64::consts::PI*at(i)).dsin_cos();
                let radial = add(scale(f.x,cs),scale(f.y,sn));
                differs(add(f.at([0.,0.,h+half*at(j)]),scale(radial,r)),radial)
            }))
        }
    }
}

/// The closed solid a polyhedral tool sweeps under a slide (`closed`'s, refused for any other
/// motion).
pub fn slid(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    if !path.screw().is_some_and(|s| s.is_slide()) { return Err(Unbuilt::Motion) }
    closed(tool,path,roll,bar,tol)
}
