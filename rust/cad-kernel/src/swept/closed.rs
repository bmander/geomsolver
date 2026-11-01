//! **The swept solid, closed** (#721 X6, docs/swept-solid-design.md): `sweep(S, M)` built as one
//! valid B-rep, independent of any blank, by the design's pipeline. The candidate faces of the
//! boundary are found, a box holding the sweep is split by them, each cell is classified by
//! membership (`member::Swept::member`, the definition read without the drawing), and the kept
//! cells are united.
//!
//! **Built here: a polyhedral tool** (every face a plane, every edge a line) **under a slide, or
//! under a turn when its edges run along the axis or square to it.** Under a turn, every line turns
//! into a surface of revolution held exactly over the heights its source spans (heights along the
//! axis are kept by the turn): a cylinder (a line along the axis), a plane square to the axis (a
//! line square to it), a hyperboloid (a line passing the axis by, its meridian a rational quadratic)
//! or a cone (a line meeting it). The candidates:
//! - under a slide `D`: each face at the start pose where it enters the slide (`n·D < 0`), at the
//!   end pose where it leaves it (`n·D > 0`), a face along the slide (`n·D = 0`) at either (one
//!   plane); each silhouette edge (its faces' `n·D` of both signs) swept, in the plane through it
//!   holding `D`;
//! - under a turn: every face at both end poses; every edge turned; a face square to the axis, its
//!   own plane (kept in itself, O); every other face's contact line (its points whose normal passes
//!   through the axis; it meets the axis or runs along it) turned, over the stretch the face spans.
//!
//! Only the candidates that carry some of the boundary are kept (`carries`): read at their source's
//! points carried over the roll, inside the source and the roll, where membership a hair to each
//! side differs. Two candidates touching along a line or a circle would join a cell across the touch,
//! so separators part them: the plane through the axis and a face's contact line at each end pose
//! (the face touches its contact surface there), and the plane square to the axis at a hyperboloid's
//! throat (where it touches the cylinder of that radius). The arrangement cuts by the separators,
//! then the planes, then the curved candidates (a plane by `kept` on each side, a solid of
//! revolution in common and cut away), and a cell is kept where its deepest interior point
//! (`query::interior`) is a member. A tool leaning to the axis splits and classifies, but its cells'
//! union does not yet hold where a hyperboloid's traced meeting with a leaning plane ends: the
//! Booleans' robustness (F4, #699 E). Anything else — a curved face, a screw, another motion — is
//! refused by what it is (`Unbuilt`); those are the design's later steps.
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
    /// Reserved: what a turn of a polyhedral tool cannot yet be built of.
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
enum Cutter { Plane(Plane),
    /// The cylinder of radius `r` about the unit `axis` through `o`, over the heights `span` along it
    /// from `o`.
    Cylinder { o: V,axis: V,r: f64,span: [f64;2] },
    /// The line through `p` along unit `e`, leaning to the unit `axis` through `o`, turned about it
    /// whole: a hyperboloid of one sheet, or where the line meets the axis a double cone; over the
    /// heights `span` along the axis from `o`.
    Revolved { o: V,axis: V,p: V,e: V,span: [f64;2] } }

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
    // each face's corners in walk order, what its own sites are read over
    let corners_of: Vec<Vec<V>> = tool.faces.iter().map(|f| f.loops.iter().flatten().map(|c| {
        let e = &tool.edges[c.edge as usize];
        let v = if c.reversed { e.v[1] } else { e.v[0] };
        tool.vertices[v as usize].p
    }).collect()).collect();
    let mut edges = Vec::with_capacity(tool.edges.len());
    for (e,edge) in tool.edges.iter().enumerate() {
        if !matches!(edge.curve,EdgeCurve::Curve(Curve::Line {..})) { return Err(Unbuilt::Curved) }
        let [f1,f2] = faces_of[e][..] else { return Err(Unbuilt::Failed(format!("edge {e} bounds {} faces",faces_of[e].len()))) };
        edges.push((edge.point(edge.t[0],&tool.vertices),edge.point(edge.t[1],&tool.vertices),[f1,f2]));
    }
    // each candidate with the sites it is read at for whether it carries any of the boundary: the
    // points of its source carried over the roll, each with the candidate's normal there
    let mut cutters: Vec<(Cutter,Vec<(V,V)>)> = Vec::new();
    // planes that carry none of the boundary but part what a touch between two candidates would
    // join into one cell, kept whatever they carry
    let mut separators: Vec<Plane> = Vec::new();
    let mut add_cutter = |c: Cutter,sites: Vec<(V,V)>| {
        let same = |k: &Cutter| match (*k,c) {
            (Cutter::Plane((p,m)),Cutter::Plane((o,n))) => norm(cross(n,m)) <= 1e-9 && dot(sub(o,p),m).abs() <= eps,
            (Cutter::Cylinder {r:r1,..},Cutter::Cylinder {r:r2,..}) => (r1-r2).abs() <= eps,
            (Cutter::Revolved {o,axis,p:p1,e:e1,..},Cutter::Revolved {p:p2,e:e2,..}) => {
                // the same surface: the same throat at the same height, and the same lean
                let (m1,m2) = (meridian(o,axis,p1,e1),meridian(o,axis,p2,e2));
                (m1.0-m2.0).abs() <= eps && (m1.1-m2.1).abs() <= eps && (m1.2-m2.2).abs() <= 1e-9
            }
            _ => false,
        };
        match cutters.iter_mut().find(|(k,_)| same(k)) {
            // one surface from two sources: over both their spans
            Some((k,known)) => {
                if let (Cutter::Cylinder {span,..} | Cutter::Revolved {span,..},Cutter::Cylinder {span:more,..} | Cutter::Revolved {span:more,..}) = (k,c) {
                    *span = [span[0].min(more[0]),span[1].max(more[1])];
                }
                known.extend(sites)
            }
            None => cutters.push((c,sites)),
        }
    };
    let pose_at = |t: f64| path.pose(t).map_err(|e| Unbuilt::Failed(e.msg));
    let (start,end) = (pose_at(a)?,pose_at(b)?);
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
        for (k,&(o,n)) in faces.iter().enumerate() {
            let s = dot(n,d);
            let pose = if s > eps { &end } else { &start };
            add_cutter(Cutter::Plane((pose.point(o),n)),face_sites(k).into_iter().map(|x| (pose.point(x),n)).collect());
        }
        for &(p,q,[f1,f2]) in &edges {
            let (s1,s2) = (dot(faces[f1].1,d),dot(faces[f2].1,d));
            if s1*s2 > 0. || (s1.abs() <= eps && s2.abs() <= eps) { continue }
            let n = cross(sub(q,p),d);
            if norm(n) <= eps*norm(d) { continue }
            let n = unit(n);
            let sites = (0..=SITES).flat_map(|i| poses.iter().map(move |m| (m.point(add(p,scale(sub(q,p),within(i)))),n))).collect();
            add_cutter(Cutter::Plane((start.point(p),n)),sites);
        }
    } else if screw.is_turn() {
        let (c,axis) = (screw.origin,screw.axis);
        // a line turned whole: a cylinder about the axis where it runs along it, a plane square to
        // the axis where it runs square to it, else a hyperboloid (a double cone where it meets the
        // axis)
        let turned = |p: V,e: V,q: V| -> Option<Cutter> {
            let e = unit(e);
            let (h0,h1) = (dot(sub(p,c),axis),dot(sub(q,c),axis));
            let span = [h0.min(h1),h0.max(h1)];
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
                };
                poses.iter().map(move |m| (m.point(x),m.vector(normal)))
            }).collect()
        };
        for (k,&(o,n)) in faces.iter().enumerate() {
            let own = face_sites(k);
            for pose in [&start,&end] {
                add_cutter(Cutter::Plane((pose.point(o),pose.vector(n))),own.iter().map(|&x| (pose.point(x),pose.vector(n))).collect());
            }
            let m = cross(n,axis);
            if norm(m) <= 1e-9 { add_cutter(Cutter::Plane((o,n)),poses.iter().flat_map(|pose| own.iter().map(|&x| (pose.point(x),n))).collect()); continue }
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
            // the face at each end pose touches that surface along the contact line: the plane
            // through the line square to the face parts the two sides of the touch. It holds the
            // axis and the face's normal, so its own normal is `axis × n`
            for pose in [&start,&end] { separators.push((c,unit(cross(axis,pose.vector(n))))) }
        }
        for &(p,q,_) in &edges {
            if let Some(cutter) = turned(p,sub(q,p),q) { add_cutter(cutter,line_sites(p,q,cutter)) }
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
    // only the candidates that carry some of the boundary: membership differing on their two sides
    // somewhere over the box (a candidate is kept where a point of it cannot be settled)
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll};
    let carrying: Vec<Cutter> = cutters.into_iter().filter(|(_,sites)| carries(&swept,sites,half)).map(|(c,_)| c).collect();
    // the separators first, then the planes, then the curved candidates: a curved candidate touching
    // a plane along a line then meets it along an edge the cells already have, which the Booleans
    // know (`ssi::trace_beside`)
    // a hyperboloid touches the cylinder of its own throat's radius along its throat circle (an
    // edge passing nearest the axis on the face whose contact line turns into that cylinder): the
    // plane square to the axis through the throat parts the two sides of that touch too
    for c in &carrying {
        if let Cutter::Revolved {o,axis,p,e,..} = *c {
            let (throat,height,_) = meridian(o,axis,p,e);
            if throat > eps { separators.push((add(o,scale(axis,height)),axis)) }
        }
    }
    let mut cutters: Vec<Cutter> = separators.into_iter().map(Cutter::Plane).collect();
    cutters.extend(carrying.iter().copied().filter(|c| matches!(c,Cutter::Plane(_))));
    cutters.extend(carrying.iter().copied().filter(|c| !matches!(c,Cutter::Plane(_))));
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
                Cutter::Revolved {o,axis,p,e,span} => {
                    // the solid the line's image over its span bounds, the cell split by each piece
                    let solids = revolved(o,axis,p,e,span,eps)?;
                    let mut pieces = vec![cell];
                    for solid in solids {
                        let mut split = Vec::new();
                        for piece in pieces {
                            for op in [Op::Common,Op::Cut] {
                                let part = combined(&piece,&solid,op,tol).map_err(|e| e.msg)?;
                                if !part.faces.is_empty() { split.push(part) }
                            }
                        }
                        pieces = split;
                    }
                    next.extend(pieces);
                }
                Cutter::Cylinder {o,axis,r,span} => {
                    let foot = add(o,scale(axis,span[0]));
                    let disc = Profile {origin:foot,normal:axis,loops:vec![vec![ProfileEdge::Arc {frame:Frame::about(foot,axis),r,span:None}]],
                        names:Vec::new()};
                    let solid = prism(&disc,0.,span[1]-span[0])?;
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
/// quadratic (the conic through its ends, its end tangents' meeting and its shoulder, the
/// shoulder's weight `u / (1 − u)`); a cone's a line, a solid on either side of its apex where the
/// span holds it.
fn revolved(o: V,axis: V,p: V,e: V,[z0,z1]: [f64;2],eps: f64) -> Result<Vec<Brep>,String> {
    let (throat,height,k) = meridian(o,axis,p,e);
    let f = Frame::about(o,axis);
    let at = |r: f64,z: f64| add(add(o,scale(f.x,r)),scale(axis,z));
    let turn = |meridian: ProfileEdge,a: [f64;2],b: [f64;2]| {
        let profile = Profile {origin:o,normal:f.y,names:Vec::new(),loops:vec![vec![
            ProfileEdge::Line {a:at(0.,a[1]),b:at(a[0],a[1])},meridian,
            ProfileEdge::Line {a:at(b[0],b[1]),b:at(0.,b[1])},ProfileEdge::Line {a:at(0.,b[1]),b:at(0.,a[1])}]]};
        crate::build::revolve(&profile,o,axis,std::f64::consts::TAU)
    };
    let rho = |z: f64| (throat*throat+k*k*(z-height)*(z-height)).sqrt();
    if throat <= eps {
        // a cone: a frustum on each side of the apex the span reaches
        let mut out = Vec::new();
        for (lo,hi) in [(z0,z1.min(height)),(z0.max(height),z1)] {
            if !(hi-lo > eps) { continue }
            let (a,b) = ([rho(lo),lo],[rho(hi),hi]);
            if a[0] <= eps && b[0] <= eps { continue }
            // a frustum's end at the apex is a point: its side's line closes onto the axis there
            if a[0] <= eps {
                let profile = Profile {origin:o,normal:f.y,names:Vec::new(),loops:vec![vec![
                    ProfileEdge::Line {a:at(0.,lo),b:at(b[0],hi)},ProfileEdge::Line {a:at(b[0],hi),b:at(0.,hi)},
                    ProfileEdge::Line {a:at(0.,hi),b:at(0.,lo)}]]};
                out.push(crate::build::revolve(&profile,o,axis,std::f64::consts::TAU)?);
            } else if b[0] <= eps {
                let profile = Profile {origin:o,normal:f.y,names:Vec::new(),loops:vec![vec![
                    ProfileEdge::Line {a:at(0.,lo),b:at(a[0],lo)},ProfileEdge::Line {a:at(a[0],lo),b:at(0.,hi)},
                    ProfileEdge::Line {a:at(0.,hi),b:at(0.,lo)}]]};
                out.push(crate::build::revolve(&profile,o,axis,std::f64::consts::TAU)?);
            } else {
                out.push(turn(ProfileEdge::Line {a:at(a[0],a[1]),b:at(b[0],b[1])},a,b)?);
            }
        }
        return Ok(out)
    }
    let slope = |z: f64| k*k*(z-height)/rho(z);
    let (a,b) = ([rho(z0),z0],[rho(z1),z1]);
    let (ta,tb) = ([slope(z0),1.],[slope(z1),1.]);
    let det = -ta[0]*tb[1]+ta[1]*tb[0];
    // a span the meridian runs straight over (its tangents parallel) is the chord itself
    if det.abs() <= 1e-12 { return Ok(vec![turn(ProfileEdge::Line {a:at(a[0],a[1]),b:at(b[0],b[1])},a,b)?]) }
    let t = (-(b[0]-a[0])*tb[1]+(b[1]-a[1])*tb[0])/det;
    let m = [a[0]+t*ta[0],a[1]+t*ta[1]];
    // the shoulder: where the line from the chord's middle to `m` meets the meridian, `u` along it
    let mid = [0.5*(a[0]+b[0]),0.5*(a[1]+b[1])];
    let (dr,dz,zr) = (m[0]-mid[0],m[1]-mid[1],mid[1]-height);
    let (qa,qb,qc) = (dr*dr-k*k*dz*dz,2.*(mid[0]*dr-k*k*zr*dz),mid[0]*mid[0]-throat*throat-k*k*zr*zr);
    let disc = qb*qb-4.*qa*qc;
    if !(disc >= 0.) { return Err("a hyperboloid's meridian has no shoulder".into()) }
    let u = [(-qb-disc.sqrt())/(2.*qa),(-qb+disc.sqrt())/(2.*qa)].into_iter().filter(|u| *u > 0. && *u < 1.)
        .fold(f64::NAN,|x,y| if x.is_nan() { y } else { x.min(y) });
    if !(u > 0. && u < 1.) { return Err("a hyperboloid's meridian's shoulder is not between its chord and its tangents".into()) }
    let conic = crate::nurbs::BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![at(a[0],a[1]),at(m[0],m[1]),at(b[0],b[1])],vec![1.,u/(1.-u),1.])?;
    Ok(vec![turn(ProfileEdge::Spline(std::sync::Arc::new(conic)),a,b)?])
}
