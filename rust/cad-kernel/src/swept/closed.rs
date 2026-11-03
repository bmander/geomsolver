//! **The swept solid, closed** (#721 X6, docs/swept-solid-design.md): `sweep(S, M)` built as one
//! valid B-rep, independent of any blank, by the design's pipeline. The candidate faces of the
//! boundary are found, a box holding the sweep is split by them, each cell is classified by
//! membership (`member::Swept::member`, the definition read without the drawing), and the kept
//! cells are united.
//!
//! **Built here: a polyhedral tool** (every face a plane, every edge a line) **under a slide or a
//! turn**, however its edges lean to the axis, **and under a screw where its edges run along the
//! axis or square to it** (and so its faces). Under a turn, every line turns
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
//! **A convex polyhedral tool turned whole is built by another road** (`turned_whole`): its solid
//! of revolution, the radii each slice holds turned once, with no Boolean. Anything else — a curved
//! face, a leaning edge under a screw, another motion — is refused by what it is (`Unbuilt`);
//! those are the design's later steps.
use crate::boolean::arrangement_within;
use crate::build::{Profile,ProfileEdge,disc,prism,square};
use crate::geom::{Curve,Frame,Surface};
use crate::topo::{Brep,EdgeCurve};
use crate::trajectory::Trajectory;
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use super::member::{Solid,Swept};

type V = [f64;3];

/// Why the closed swept solid is not built.
#[derive(Clone,Debug,PartialEq)]
pub enum Unbuilt {
    /// The tool has a face that is no plane, or an edge that is no line, at `at`: its envelope
    /// sheets are curved, the design's later steps.
    Curved { at: V },
    /// The motion is no slide, turn or screw: the design's later steps.
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
    Screwed { p: V,e: V } }

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
/// moves something; every face of the tool a plane and every edge a line bounding two of them;
/// and under a screw every face and edge along the axis or square to it. The constant twist it is,
/// or the first thing that is not, by what it is and where (a point of the face or the edge).
pub fn reads(tool: &Brep,path: &Trajectory) -> Result<crate::trajectory::Screw,Unbuilt> { read(tool,path).map(|r| r.screw) }

/// What `reads` reads of a tool it takes: the constant twist, each face's plane (a point of it and
/// its outward unit normal), and each edge's two ends and two faces.
struct Read { screw: crate::trajectory::Screw,faces: Vec<(V,V)>,edges: Vec<(V,V,[usize;2])> }

/// `reads`, keeping what it read.
fn read(tool: &Brep,path: &Trajectory) -> Result<Read,Unbuilt> {
    let Some(screw) = path.screw().filter(|s| s.ratio != 0. || s.advance != 0.) else { return Err(Unbuilt::Motion) };
    // a way leaning to a screw's axis: neither along it nor square to it
    let leans = |d: V| !screw.is_turn() && !screw.is_slide() && norm(cross(d,screw.axis)) > 1e-9 && dot(d,screw.axis).abs() > 1e-9;
    let mut faces = Vec::with_capacity(tool.faces.len());
    for f in &tool.faces {
        let at = f.loops.iter().flatten().next().map_or([0.;3],|c| tool.vertices[tool.ends(c)[0] as usize].p);
        let Surface::Plane(frame) = &f.surface else { return Err(Unbuilt::Curved {at}) };
        let n = unit(if f.reversed { scale(frame.z,-1.) } else { frame.z });
        if leans(n) { return Err(Unbuilt::Leaning {at}) }
        faces.push((frame.o,n));
    }
    let uses = tool.uses();
    let mut edges = Vec::with_capacity(tool.edges.len());
    for (e,edge) in tool.edges.iter().enumerate() {
        let (p,q) = (edge.point(edge.t[0],&tool.vertices),edge.point(edge.t[1],&tool.vertices));
        let mid = scale(add(p,q),0.5);
        if !matches!(edge.curve,EdgeCurve::Curve(Curve::Line {..})) { return Err(Unbuilt::Curved {at:mid}) }
        let [(f1,..),(f2,..)] = uses[e][..] else { return Err(Unbuilt::Failed(format!("edge {e} bounds {} faces",uses[e].len()))) };
        if leans(unit(sub(q,p))) { return Err(Unbuilt::Leaning {at:mid}) }
        edges.push((p,q,[f1,f2]));
    }
    Ok(Read {screw,faces,edges})
}

/// `closed` over a box holding the sweep, or `cut_from` over `blank`.
fn built(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Brep,Unbuilt> {
    let Read {screw,faces,edges} = read(tool,path)?;
    let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let size = { let (lo,hi) = tool.bounds(); norm(sub(hi,lo)) };
    let eps = 1e-9*(1.+size);
    // each face's corners in walk order, what its own sites are read over
    let corners_of: Vec<Vec<V>> = tool.faces.iter().map(|f| f.loops.iter().flatten()
        .map(|c| tool.vertices[tool.ends(c)[0] as usize].p).collect()).collect();
    // each candidate with the sites it is read at for whether it carries any of the boundary: the
    // points of its source carried over the roll, each with the candidate's normal there
    let mut cutters: Vec<(Cutter,Vec<(V,V)>)> = Vec::new();
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
    } else {
        let (c,axis) = (screw.origin,screw.axis);
        // a convex tool turned whole is the solid of revolution of its radial profile, built in
        // closed form with no Boolean
        let whole = (screw.ratio*(b-a)).abs() >= std::f64::consts::TAU*(1.-1e-12);
        let convex = faces.iter().all(|&(o,n)| tool.vertices.iter().all(|v| dot(sub(v.p,o),n) <= eps));
        if screw.is_turn() && whole && convex && blank.is_none() { return turned_whole(&faces,&edges,c,axis,eps,tol) }
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
    // a box holding the sweep: the tool's box at the end poses and, under a turn, its reach about
    // the axis; under a screw, a box square to the axis and as tall along it as the sweep, a little
    // more (`tall`: its heights along the axis and its farthest from it), so a sheet a level edge
    // screws into leaves it within a turn where the sweep climbs less than a turn's step
    // (cut from a blank, the box holds the blank too: every candidate reaching across it)
    let box_corners = |(lo,hi): (V,V)| -> Vec<V> { (0..8).map(|k| std::array::from_fn(|i| if (k>>i)&1 == 0 { lo[i] } else { hi[i] })).collect() };
    let corners = box_corners(tool.bounds());
    let held: Vec<V> = blank.map(|b| box_corners(b.bounds())).unwrap_or_default();
    let (domain,centre,half,tall) = if screw.is_slide() || screw.is_turn() {
        let mut points: Vec<V> = corners.iter().flat_map(|&p| [start.point(p),end.point(p)]).chain(held.iter().copied()).collect();
        if screw.is_turn() {
            let reach = corners.iter().map(|&p| norm(sub(p,screw.origin))).fold(0.,f64::max);
            points.extend([1.,-1.].iter().flat_map(|&s| (0..3).map(move |k| { let mut v = [0.;3]; v[k] = s*reach; v }))
                .map(|v| add(screw.origin,v)));
        }
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
    let swept = Swept {tool:&solid,path,roll};
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
            Cutter::Screwed {p,e} => {
                solids.push(screwed_sheet(&screw,&pose_at,p,e,tall.expect("a level edge's sheet is a screw's, its box tall"),size)?)
            }
        }
    }
    let refs: Vec<&Brep> = solids.iter().collect();
    let complex = arrangement_within(blank.unwrap_or(&domain),&refs,tol)?;
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
