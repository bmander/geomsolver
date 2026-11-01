//! **The swept solid, closed** (#721 X6, docs/swept-solid-design.md): `sweep(S, M)` built as one
//! valid B-rep, independent of any blank, by the design's pipeline. The candidate faces of the
//! boundary are found, a box holding the sweep is split by them, each cell is classified by
//! membership (`member::Swept::member`, the definition read without the drawing), and the kept
//! cells are united.
//!
//! **Built here, the first case: a polyhedral tool under a slide.** Every face of the tool is a
//! plane and every edge a line, and the motion is a slide `D` over the roll. Every candidate is then
//! planar:
//! - each face at the start pose where it enters the slide (`n·D < 0`) and at the end pose where it
//!   leaves it (`n·D > 0`), a face along the slide (`n·D = 0`) at either, these being one plane;
//! - each edge where the slide turns its faces' normals from one sign of `n·D` to the other (a
//!   silhouette edge), swept, in the plane through it holding `D`.
//!
//! The arrangement is the planes' (`kept` on each side, a cell the plane misses left whole), and
//! a cell is kept where its deepest interior point (`query::interior`) is a member. Anything else
//! (a curved face, another motion) is refused by what it is (`Unbuilt`); those are the design's
//! later steps, not this one's.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::boolean::{Op,combined};
use crate::build::{kept,prism,square};
use crate::geom::{Curve,Surface};
use crate::topo::{Brep,EdgeCurve};
use crate::trajectory::Trajectory;
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use super::member::{Solid,Swept,Unsettled};

type V = [f64;3];

/// Why the closed swept solid is not built.
#[derive(Clone,Debug,PartialEq)]
pub enum Unbuilt {
    /// The tool has a face that is no plane, or an edge that is no line: its envelope sheets are
    /// curved, the design's later steps.
    Curved,
    /// The motion is no slide: the design's later steps.
    Motion,
    /// A cell's membership could not be settled: its deepest point stands within the mesh's sag
    /// of the swept boundary.
    Unsettled { at: V },
    /// The kernel could not do it.
    Failed(String),
}

impl From<String> for Unbuilt { fn from(m: String) -> Unbuilt { Unbuilt::Failed(m) } }

/// A plane: a point of it and its unit normal.
type Plane = (V,V);

/// **The closed solid a polyhedral tool `tool` sweeps under the slide `path` over `roll`**, its
/// tool, motion and solid in one set of units, built to `tol`; membership read on the tool meshed
/// within `bar`.
pub fn slid(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    let Some(screw) = path.screw().filter(|s| s.is_slide() && s.advance != 0.) else { return Err(Unbuilt::Motion) };
    let [a,b] = roll;
    let at = |t: f64| scale(screw.axis,screw.advance*t/std::f64::consts::TAU);
    let (start,end) = (at(a.min(b)),at(a.max(b)));
    let d = sub(end,start);
    let size = { let (lo,hi) = tool.bounds(); norm(sub(hi,lo)) };
    let eps = 1e-9*(1.+size);
    // each face's plane and outward normal
    let mut normals = Vec::with_capacity(tool.faces.len());
    for f in &tool.faces {
        let Surface::Plane(frame) = &f.surface else { return Err(Unbuilt::Curved) };
        let n = if f.reversed { scale(frame.z,-1.) } else { frame.z };
        normals.push((frame.o,unit(n)));
    }
    for e in &tool.edges {
        if !matches!(e.curve,EdgeCurve::Curve(Curve::Line {..})) { return Err(Unbuilt::Curved) }
    }
    let mut planes: Vec<Plane> = Vec::new();
    let mut add_plane = |o: V,n: V| {
        let n = unit(n);
        let same = |&(p,m): &Plane| norm(cross(n,m)) <= 1e-9 && dot(sub(o,p),m).abs() <= eps;
        if !planes.iter().any(same) { planes.push((o,n)) }
    };
    for &(o,n) in &normals {
        let s = dot(n,d);
        if s < -eps { add_plane(add(o,start),n) } else if s > eps { add_plane(add(o,end),n) } else { add_plane(add(o,start),n) }
    }
    // each edge's two faces, and the silhouette edges' planes
    let mut faces_of: Vec<Vec<usize>> = vec![Vec::new();tool.edges.len()];
    for (k,f) in tool.faces.iter().enumerate() {
        for c in f.loops.iter().flatten() { faces_of[c.edge as usize].push(k) }
    }
    for (e,edge) in tool.edges.iter().enumerate() {
        let [f1,f2] = faces_of[e][..] else { return Err(Unbuilt::Failed(format!("edge {e} bounds {} faces",faces_of[e].len()))) };
        let (s1,s2) = (dot(normals[f1].1,d),dot(normals[f2].1,d));
        if s1*s2 > 0. || (s1.abs() <= eps && s2.abs() <= eps) { continue }
        let p = edge.point(edge.t[0],&tool.vertices);
        let q = edge.point(edge.t[1],&tool.vertices);
        let n = cross(sub(q,p),d);
        if norm(n) <= eps*norm(d) { continue }
        add_plane(add(p,start),n);
    }
    // a box holding the sweep, split by every plane
    let (lo,hi) = tool.bounds();
    let (lo,hi) = ((0..3).map(|k| lo[k]+start[k].min(end[k])).collect::<Vec<_>>(),(0..3).map(|k| hi[k]+start[k].max(end[k])).collect::<Vec<_>>());
    let centre: V = std::array::from_fn(|k| 0.5*(lo[k]+hi[k]));
    let half = (0..3).map(|k| hi[k]-lo[k]).fold(0.,f64::max)*0.6+tol;
    let base = add(centre,[0.,0.,-half]);
    let mut cells = vec![prism(&square(base,[0.,0.,1.],half),0.,2.*half)?];
    for &(o,n) in &planes {
        let mut next = Vec::with_capacity(cells.len()*2);
        for cell in cells {
            let side = |v: &crate::topo::Vertex| dot(sub(v.p,o),n);
            let (low,high) = cell.vertices.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),v| (l.min(side(v)),h.max(side(v))));
            if low >= -tol || high <= tol { next.push(cell); continue }
            for keep in [n,scale(n,-1.)] {
                let piece = kept(&cell,o,keep,0.)?;
                if !piece.faces.is_empty() { next.push(piece) }
            }
        }
        cells = next;
    }
    // each cell classified by membership at its deepest point, the members united
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll};
    let mut out: Option<Brep> = None;
    for cell in &cells {
        let Some(&(x,_)) = crate::query::interior(cell,1)?.first() else { continue };
        match swept.member(x) {
            Ok(true) => out = Some(match out { None => cell.clone(),Some(acc) => combined(&acc,cell,Op::Union,0.).map_err(|e| e.msg)? }),
            Ok(false) => {}
            Err(Unsettled::Near {..}) | Err(_) => return Err(Unbuilt::Unsettled {at:x}),
        }
    }
    let out = out.ok_or_else(|| Unbuilt::Failed("no cell of the arrangement is swept".into()))?;
    out.check(tol).map_err(|e| Unbuilt::Failed(e.msg))?;
    Ok(out)
}

