//! **K2's patches** (#780 L3, docs/swept-solid-design.md step 7): the sheet each instant's contact
//! sweeps under a rigid motion no constant twist takes, fitted, and entered in the closed class's
//! one arrangement as any other candidate is.
//!
//! Under such a motion no contact is fixed on the tool: at each instant `t` the motion is a screw
//! (Mozzi–Chasles), and the tool touches its sweep where its outward normal is square to that
//! instant's velocity, `n · v_t(x) = 0` — on a face of revolution each ring's two roots of `A cos θ
//! + B sin θ + C = 0` (`swept::instant`). **The contact is read per tool, not per face**: a ring's
//! roots run on across a smooth seam into the neighbour's, and at a sharp edge the contact is the
//! edge's own points whose two faces' `n · v` differ in sign (L2's fan rule). For a convex tool of
//! revolution (a ball, a bull-nose, a drum) that is one closed curve at each instant, read in a
//! **pencil of half-planes** about the line through the tool's centre along its velocity
//! (`Revolved`, `Pencil`): each half-plane cuts the tool in a convex arc from its back to its
//! front, along which `n · v` changes sign once — the arc's own normal turns monotonely from the
//! back to the front, and the surface's normal leans out of the half-plane by a positive factor —
//! so its one root is the contact there, landing on an edge where the edge's fan holds it. So the
//! instant's contact is a closed curve `x(t, φ)`, `φ` the half-plane's angle, smooth across the
//! **branch locus**: this chart is transverse to it, where the ring chart `(t, h, ±)` turns back
//! (step 6's rule: read the patch in the chart transverse to the locus, not cut there).
//!
//! The curve runs over the tool's faces and edges in **pieces**, each on one face or one sharp
//! edge, joined where the contact crosses a seam or reaches an edge (found in `φ` by bisection, as
//! a label changing). A piece on a face is read over its own stretch of `φ` at even fractions of
//! it, so its points move smoothly with `t` though the joins move along the tool; a piece on an
//! edge (its fan) is read along the edge's own circle by its angle, since the pencil's angle runs
//! as the square root of the way along an edge where the edge runs along the velocity (where a
//! face's contact reaches it). The sheet is `M(t) · x` over (the fraction round the loop, `t`), a
//! B-spline net quintic both ways, continuous where pieces join, fitted until halfway points stand
//! within its tolerance of the sheet (`Fitted`). Its pieces keep their kinds over the roll, or the
//! build is refused by name (`Unbuilt::Curved` at a contact reaching an edge or a seam partway, the
//! patch not yet cut there).
//!
//! **Its touches given** (R5): the tool at each end pose touches the sheet along that instant's
//! contact, each face along the pieces it holds (an edge's fan crossing both its faces there) —
//! known before anything is met, so given as edges of both (`boolean::Given`), the net's iso-curves
//! at the end rolls, which are the sheet's rims. Those are the whole meetings of the tool at either
//! end pose with the sheet (`entered`), so nothing is sought between them. The sheet stops at its
//! rims, on the tool: carried on past them, its flap and the end pose's section on a face of the
//! blank (a block's top the tool stands across where it starts) are tangent there, and the cell
//! between them was misread (refused by the judge, never returned wrong).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::boolean::Given;
use crate::geom::{Curve,Frame,Surface};
use crate::nurbs::{BSpline,Net};
use crate::topo::{Brep,EdgeCurve};
use crate::trajectory::{Trajectory,Twist};
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use super::closed::Unbuilt;
use std::f64::consts::{PI,TAU};
use std::sync::Arc;

type V = [f64;3];
type P2 = [f64;2];

/// A piece of a tool's meridian `(ρ, z)`, walked counter-clockwise (the solid on its left): a
/// segment, or an arc about `c` of radius `r` from the angle `from` by `sweep` (positive: convex).
#[derive(Clone,Copy,Debug)]
pub enum Element { Line { a: P2,b: P2 },Arc { c: P2,r: f64,from: f64,sweep: f64 } }

impl Element {
    /// Where it starts and where it ends.
    fn ends(&self) -> [P2;2] {
        match *self {
            Element::Line {a,b} => [a,b],
            Element::Arc {c,r,from,sweep} => {
                let at = |w: f64| { let (s,co) = w.dsin_cos(); [c[0]+r*co,c[1]+r*s] };
                [at(from),at(from+sweep)]
            }
        }
    }
    /// Its outward unit normal in the meridian at its point `p`.
    fn normal(&self,p: P2) -> P2 {
        match *self {
            Element::Line {a,b} => { let (dr,dz) = (b[0]-a[0],b[1]-a[1]); let l = dr.dhypot(dz); [dz/l,-dr/l] }
            Element::Arc {c,r,..} => [(p[0]-c[0])/r,(p[1]-c[1])/r],
        }
    }
    /// Its tangent the way it is walked at its start (`0`) or its end (`1`).
    fn tangent(&self,end: usize) -> P2 {
        match *self {
            Element::Line {a,b} => { let (dr,dz) = (b[0]-a[0],b[1]-a[1]); let l = dr.dhypot(dz); [dr/l,dz/l] }
            Element::Arc {from,sweep,..} => {
                let (s,c) = (from+if end == 0 { 0. } else { sweep }).dsin_cos();
                [-s*sweep.signum(),c*sweep.signum()]
            }
        }
    }
    /// Where the ray from `(0, z0)` along `(cβ, sβ)` (`cβ ≥ 0`) meets it, by the length along the
    /// ray, and how far past its own stretch the meeting stands (nothing where it is on it).
    fn hit(&self,z0: f64,[cb,sb]: P2) -> Option<(f64,f64)> {
        match *self {
            Element::Line {a,b} => {
                // (s cβ, z0 + s sβ) = a + λ (b − a)
                let (dr,dz) = (b[0]-a[0],b[1]-a[1]);
                let det = cb*(-dz)-sb*(-dr);
                if det.abs() <= 1e-300 { return None }
                let (rr,rz) = (a[0],a[1]-z0);
                let s = (rr*(-dz)-rz*(-dr))/det;
                let lambda = (cb*rz-sb*rr)/det;
                (s > 0.).then(|| (s,(-lambda).max(lambda-1.).max(0.)*dr.dhypot(dz)))
            }
            Element::Arc {c,r,from,sweep} => {
                // |(s cβ, z0 + s sβ) − c|² = r²
                let (qr,qz) = (-c[0],z0-c[1]);
                let half_b = cb*qr+sb*qz;
                let cc = qr*qr+qz*qz-r*r;
                let disc = half_b*half_b-cc;
                if disc < 0. { return None }
                let root = disc.sqrt();
                let mut best: Option<(f64,f64)> = None;
                for s in [-half_b+root,-half_b-root] {
                    if !(s > 0.) { continue }
                    let w = (z0+s*sb-c[1]).datan2(s*cb-c[0]);
                    // the angle's place along the sweep, wrapped to the nearest turn
                    let mut x = (w-from)*sweep.signum();
                    x = x.rem_euclid(TAU);
                    let span = sweep.abs();
                    let off = if x <= span { 0. } else { (x-span).min(TAU-x) }*r;
                    if best.is_none_or(|(_,o)| off < o) { best = Some((s,off)) }
                }
                best
            }
        }
    }
}

/// What a point of the contact lies on: a face's piece of the meridian (by its index among the
/// elements), or a sharp corner between two of them (the edge it turns into, by the index of the
/// element it ends).
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum On { Face(usize),Edge(usize) }

/// **A convex tool of revolution**, read off its B-rep: its axis (`frame`, `z` along it), its
/// meridian's pieces in order from the axis below round to the axis above, the tool's face each
/// is (`faces`), and whether each joint between two is sharp (`sharp[k]`, after element `k`).
#[derive(Clone,Debug)]
pub struct Revolved {
    pub frame: Frame,
    pub elements: Vec<Element>,
    pub faces: Vec<usize>,
    pub sharp: Vec<bool>,
    /// The height on the axis the pencil's rays leave from, halfway up the tool.
    pub centre: f64,
    /// The tool's size: its meridian's farthest from the centre.
    pub size: f64,
}

impl Revolved {
    /// **The tool read as a convex solid of revolution**: every face a plane square to one axis, a
    /// cylinder, a cone or a torus about it, or a ball centred on it; its meridian one chain of
    /// their pieces from the axis round to the axis, turning one way all along (convex). Refused
    /// (`Unbuilt::Curved`, at a point of the face that is not) otherwise.
    pub fn read(tool: &Brep) -> Result<Revolved,Unbuilt> {
        let size = { let (lo,hi) = tool.bounds(); norm(sub(hi,lo)) };
        let eps = 1e-9*(1.+size);
        let at_face = |k: usize| -> V {
            tool.faces[k].loops.iter().flatten().next().map_or(tool.vertices.first().map_or([0.;3],|v| v.p),
                |c| tool.vertices[tool.ends(c)[0] as usize].p)
        };
        // the axis: a round face's own, or through a ball's centre along a plane's normal or the
        // ball's own pole
        let mut axis: Option<Frame> = None;
        for f in &tool.faces {
            if let Surface::Cylinder(g,_) | Surface::Cone(g,..) | Surface::Torus(g,..) = &f.surface { axis = Some(Frame::about(g.o,unit(g.z))); break }
        }
        if axis.is_none() {
            let ball = tool.faces.iter().find_map(|f| if let Surface::Sphere(g,_) = &f.surface { Some(*g) } else { None });
            let plane = tool.faces.iter().find_map(|f| if let Surface::Plane(g) = &f.surface { Some(*g) } else { None });
            axis = match (ball,plane) {
                (Some(b),Some(p)) => Some(Frame::about(b.o,unit(p.z))),
                (Some(b),None) => Some(Frame::about(b.o,unit(b.z))),
                _ => None,
            };
        }
        let Some(g) = axis else { return Err(Unbuilt::Curved {at:at_face(0)}) };
        let rz = |p: V| -> P2 { let l = g.local(p); [l[0].dhypot(l[1]),l[2]] };
        let on_axis = |o: V| { let l = g.local(o); l[0].dhypot(l[1]) <= eps };
        let along = |z: V| norm(cross(unit(z),g.z)) <= 1e-9;
        let uses = tool.uses();
        let located = crate::query::Located::new(tool,eps);
        let mut elements = Vec::new();
        let mut faces = Vec::new();
        for (k,f) in tool.faces.iter().enumerate() {
            let refused = Err(Unbuilt::Curved {at:at_face(k)});
            // the meridian points of its boundary: each edge's (a seam, one face both sides, none)
            let mut pts: Vec<P2> = Vec::new();
            for c in f.loops.iter().flatten() {
                let e = &tool.edges[c.edge as usize];
                if matches!(e.curve,EdgeCurve::Degenerate) { continue }
                if uses[c.edge as usize].iter().all(|u| u.0 == k) { continue }
                for i in 0..=4 { pts.push(rz(e.point(e.t[0]+(e.t[1]-e.t[0])*i as f64/4.,&tool.vertices))) }
            }
            let distinct = |pts: &[P2]| -> Vec<P2> {
                let mut out: Vec<P2> = Vec::new();
                for &p in pts { if !out.iter().any(|q| (p[0]-q[0]).dhypot(p[1]-q[1]) <= 1e-7*(1.+size)) { out.push(p) } }
                out
            };
            let ends = distinct(&pts);
            match &f.surface {
                Surface::Plane(h) => {
                    if !along(h.z) { return refused }
                    let z = rz(h.o)[1];
                    let rmax = ends.iter().map(|p| p[0]).fold(0.,f64::max);
                    let rmin = if f.loops.len() > 1 { ends.iter().map(|p| p[0]).fold(f64::INFINITY,f64::min) } else { 0. };
                    elements.push(Element::Line {a:[rmin,z],b:[rmax,z]});
                }
                Surface::Cylinder(h,_) | Surface::Cone(h,..) => {
                    if !along(h.z) || !on_axis(h.o) { return refused }
                    let [Some(lo),Some(hi)] = [ends.iter().copied().reduce(|a,b| if b[1] < a[1] { b } else { a }),
                        ends.iter().copied().reduce(|a,b| if b[1] > a[1] { b } else { a })] else { return refused };
                    elements.push(Element::Line {a:lo,b:hi});
                }
                Surface::Sphere(h,r) | Surface::Torus(h,_,r) => {
                    let c = match &f.surface { Surface::Torus(_,big,_) => [big.abs(),rz(h.o)[1]],_ => [0.,rz(h.o)[1]] };
                    if !on_axis(h.o) || matches!(f.surface,Surface::Torus(..)) && !along(h.z) { return refused }
                    let r = r.abs();
                    // the angles about the meridian's centre of its boundary, and the arc between
                    // them the face holds (tried at its middle); a ball with no boundary pole to
                    // pole
                    let angle = |p: P2| (p[1]-c[1]).datan2(p[0]-c[0]);
                    let mut ws: Vec<f64> = ends.iter().map(|&p| angle(p)).collect();
                    ws.sort_by(f64::total_cmp);
                    ws.dedup_by(|a,b| (*a-*b).abs() <= 1e-9);
                    let holds = |w: f64| -> bool {
                        let (s,co) = w.dsin_cos();
                        let (rho,z) = (c[0]+r*co,c[1]+r*s);
                        rho >= 0. && (0..8).any(|i| {
                            let (sa,ca) = (i as f64*TAU/8.+0.3).dsin_cos();
                            located.face_place(k,g.at([rho*ca,rho*sa,z])) == crate::query::Place::In
                        })
                    };
                    let (from,sweep) = match (ws.len(),&f.surface) {
                        (0,Surface::Sphere(..)) => (-PI/2.,PI),
                        (1,Surface::Sphere(..)) => {
                            let w = ws[0];
                            if holds(0.5*(w+PI/2.)) { (w,PI/2.-w) } else { (-PI/2.,w+PI/2.) }
                        }
                        (2,_) => {
                            let (w0,w1) = (ws[0],ws[1]);
                            if holds(0.5*(w0+w1)) { (w0,w1-w0) } else { (w1,TAU-(w1-w0)) }
                        }
                        _ => return refused,
                    };
                    elements.push(Element::Arc {c,r,from,sweep});
                }
                _ => return refused,
            }
            faces.push(k);
        }
        // the chain from the axis below round to the axis above: each piece walked so the solid is
        // on its left, each starting where the one before ends
        let n = elements.len();
        let near = |p: P2,q: P2| (p[0]-q[0]).dhypot(p[1]-q[1]) <= 1e-6*(1.+size);
        let reversed = |e: Element| match e {
            Element::Line {a,b} => Element::Line {a:b,b:a},
            Element::Arc {c,r,from,sweep} => Element::Arc {c,r,from:from+sweep,sweep:-sweep},
        };
        let bottom = (0..n).flat_map(|i| [(i,0),(i,1)]).filter(|&(i,e)| elements[i].ends()[e][0] <= 1e-6*(1.+size))
            .min_by(|&(i,e),&(j,f)| elements[i].ends()[e][1].total_cmp(&elements[j].ends()[f][1]));
        let Some((first,end)) = bottom else { return Err(Unbuilt::Curved {at:at_face(0)}) };
        let mut order = vec![(first,if end == 0 { elements[first] } else { reversed(elements[first]) })];
        let mut used = vec![false;n];
        used[first] = true;
        while order.len() < n {
            let tip = order.last().unwrap().1.ends()[1];
            let Some((i,e)) = (0..n).filter(|&i| !used[i]).flat_map(|i| [(i,0),(i,1)]).find(|&(i,e)| near(elements[i].ends()[e],tip))
                else { return Err(Unbuilt::Curved {at:at_face(faces[order.last().unwrap().0])}) };
            used[i] = true;
            order.push((i,if e == 0 { elements[i] } else { reversed(elements[i]) }));
        }
        if order.last().unwrap().1.ends()[1][0] > 1e-6*(1.+size) { return Err(Unbuilt::Curved {at:at_face(faces[order.last().unwrap().0])}) }
        // walked counter-clockwise from the bottom outward
        let faces: Vec<usize> = order.iter().map(|&(i,_)| faces[i]).collect();
        let mut elements: Vec<Element> = order.into_iter().map(|(_,e)| e).collect();
        // convex: every arc turning left, every joint turning left or running on
        let mut sharp = Vec::with_capacity(n);
        for k in 0..n {
            if let Element::Arc {sweep,..} = elements[k] { if !(sweep > 0.) { return Err(Unbuilt::Curved {at:at_face(faces[k])}) } }
            if k+1 < n {
                let (t0,t1) = (elements[k].tangent(1),elements[k+1].tangent(0));
                let turn = t0[0]*t1[1]-t0[1]*t1[0];
                if turn < -1e-9 { return Err(Unbuilt::Curved {at:at_face(faces[k])}) }
                sharp.push(turn > 1e-9 || t0[0]*t1[0]+t0[1]*t1[1] < 0.);
            }
        }
        let zs: Vec<f64> = elements.iter().flat_map(|e| e.ends()).map(|p| p[1]).collect();
        let (zlo,zhi) = zs.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),&z| (l.min(z),h.max(z)));
        let centre = 0.5*(zlo+zhi);
        let size = elements.iter().flat_map(|e| e.ends()).map(|p| p[0].dhypot(p[1]-centre)).fold(0.,f64::max);
        // (an arc's own far point counts too)
        let size = elements.iter().fold(size,|s,e| match *e { Element::Arc {c,r,..} => s.max(c[0].dhypot(c[1]-centre)+r),_ => s });
        elements.shrink_to_fit();
        Ok(Revolved {frame:g,elements,faces,sharp,centre,size})
    }

    /// The pencil's centre, in the tool's own coordinates.
    pub fn origin(&self) -> V { self.frame.at([0.,0.,self.centre]) }

    /// **Where the ray from the centre along the unit `d` (the tool's coordinates) leaves the
    /// tool**: the point, the meridian element it lies on and its outward unit normal there.
    pub fn cast(&self,d: V) -> (V,usize,V) {
        let l = self.frame.dir_local(d);
        let across = l[0].dhypot(l[1]);
        let (cb,sb) = (across,l[2]);
        let mut best: Option<(usize,f64,f64)> = None;
        for (k,e) in self.elements.iter().enumerate() {
            if let Some((s,off)) = e.hit(self.centre,[cb,sb]) {
                if best.is_none_or(|(_,_,o)| off < o) { best = Some((k,s,off)) }
            }
        }
        let (k,s,_) = best.unwrap_or((0,0.,0.));
        let m = [s*cb,self.centre+s*sb];
        let nm = self.elements[k].normal(m);
        let radial = if across > 1e-300 { [l[0]/across,l[1]/across] } else { [1.,0.] };
        let p = self.frame.at([m[0]*radial[0],m[0]*radial[1],m[1]]);
        let n = self.frame.dir([nm[0]*radial[0],nm[0]*radial[1],nm[1]]);
        (p,k,n)
    }
}

/// The pencil's half-planes at an instant: about the line through the tool's centre along its
/// velocity `w` there, each at the angle `φ` from `f1` toward `f2` (the tool's coordinates).
#[derive(Clone,Copy,Debug)]
struct Pencil { w: V,f1: V,f2: V }

impl Pencil {
    /// The pencil at the instant whose twist (on the tool) is `twist`, its angles measured from the
    /// direction `across`, square to every velocity the roll gives the centre.
    fn at(tool: &Revolved,twist: &Twist,across: V) -> Option<Pencil> {
        let v = twist.velocity(tool.origin());
        let w = crate::util::space::normalised(v)?;
        let f1 = crate::util::space::normalised(sub(across,scale(w,dot(across,w))))?;
        Some(Pencil {w,f1,f2:cross(w,f1)})
    }
}

/// How many of a half-plane's angles its root is first bracketed among.
const BRACKETS: usize = 16;

/// **The contact in the half-plane at `φ`**: the tool's point there whose outward normal is square
/// to the instant's velocity, and what it lies on — found as `n · v`'s one change of sign along the
/// tool's arc in the half-plane from its front (`α = 0`, along the velocity) to its back. Bisected
/// while the bracket stands across a corner of the meridian, to rounding where the root stands on
/// one (its fan holds it, or a smooth seam); within one face `n · v` runs smoothly along the arc and
/// its root is settled by the secant (`roots::bracketed_root`) — or, where only what it lies on is
/// asked (`label`), not settled at all. `None` where the arc does not change sign there once (the
/// motion turning the tool against its own advance).
fn contact(tool: &Revolved,twist: &Twist,pencil: &Pencil,phi: f64,label: bool) -> Option<(V,On)> {
    use crate::util::roots::{bisect,bracketed_root};
    let (s,c) = phi.dsin_cos();
    let e = add(scale(pencil.f1,c),scale(pencil.f2,s));
    let read = |alpha: f64| -> (f64,V,usize) {
        let (sa,ca) = alpha.dsin_cos();
        let (p,k,n) = tool.cast(add(scale(pencil.w,ca),scale(e,sa)));
        (dot(n,twist.velocity(p)),p,k)
    };
    let hs: Vec<(f64,(f64,V,usize))> = (0..=BRACKETS).map(|i| { let a = PI*i as f64/BRACKETS as f64; (a,read(a)) }).collect();
    let changes: Vec<usize> = (0..BRACKETS).filter(|&i| (hs[i].1.0 > 0.) != (hs[i+1].1.0 > 0.)).collect();
    let [i] = changes[..] else { return None };
    if !(hs[0].1.0 > 0. && hs[BRACKETS].1.0 <= 0.) { return None }
    let middle = |lo: (f64,(f64,V,usize)),hi: (f64,(f64,V,usize))| { let a = 0.5*(lo.0+hi.0); (a,read(a)) };
    let across = |lo: (f64,(f64,V,usize)),hi: (f64,(f64,V,usize))| {
        let a = 0.5*(lo.0+hi.0);
        lo.1.2 != hi.1.2 && a > lo.0 && a < hi.0
    };
    let (lo,hi) = bisect(hs[i],hs[i+1],middle,across,|m| m.1.0 > 0.);
    let ((_,(_,p,klo)),(_,(_,_,khi))) = (lo,hi);
    // a root standing where the arc turns a corner: on its sharp edge (the meridian's element
    // ending there, walked from the front), the fan's; or where it runs on smoothly, the seam
    if klo != khi {
        let k = klo.min(khi);
        return Some((p,if tool.sharp[k] && klo.abs_diff(khi) == 1 { On::Edge(k) } else { On::Face(klo) }))
    }
    if label { return Some((p,On::Face(klo))) }
    let root = bracketed_root(|a| Some(read(a).0),lo.0,lo.1.0,hi.0,hi.1.0,1e-15*PI);
    let (_,p,k) = read(root);
    Some((p,On::Face(k)))
}

/// One instant's contact read round the pencil: where its pieces join (`joins`, the angles at
/// which what it lies on changes, in order from the first), and what each piece lies on.
#[derive(Clone,Debug)]
struct Instant { twist: Twist,pencil: Pencil,joins: Vec<f64>,on: Vec<On> }

/// How many angles round the pencil an instant's pieces are first told apart at.
const ROUND: usize = 96;

/// The contact at the instant `twist`, its pieces found round the pencil, each join settled in the
/// angle to rounding. `Err` with a point of the tool where a half-plane holds no one root, or where
/// the contact crosses a sharp edge with no fan between its faces.
fn instant(tool: &Revolved,twist: Twist,across: V) -> Result<Instant,V> {
    let pencil = Pencil::at(tool,&twist,across).ok_or(tool.origin())?;
    let label = |phi: f64| contact(tool,&twist,&pencil,phi,false).map(|(_,on)| on).ok_or_else(|| tool.cast(pencil.f1).0);
    let phis: Vec<f64> = (0..ROUND).map(|i| TAU*i as f64/ROUND as f64).collect();
    let labels: Vec<On> = phis.iter().map(|&p| label(p)).collect::<Result<_,_>>()?;
    let mut joins = Vec::new();
    let mut on = Vec::new();
    for i in 0..ROUND {
        let (a,b) = (labels[i],labels[(i+1)%ROUND]);
        if a == b { continue }
        let mut missed = None;
        let (lo,hi) = crate::util::roots::bisect(phis[i],phis[i]+TAU/ROUND as f64,|l,h| 0.5*(l+h),|l,h| { let m = 0.5*(l+h); m > l && m < h },
            |m| label(m).map_or_else(|at| { missed.get_or_insert(at); true },|l| l == a));
        if let Some(at) = missed { return Err(at) }
        // a contact crossing a sharp edge from one face straight onto the other: its fan there has
        // no width, between a fan's piece and none — where the patch's pieces change, not yet cut
        if let (On::Face(x),On::Face(y)) = (a,b) {
            if x.abs_diff(y) == 1 && tool.sharp[x.min(y)] {
                return Err(contact(tool,&twist,&pencil,0.5*(lo+hi),true).map_or_else(|| tool.cast(pencil.f1).0,|(p,_)| p))
            }
        }
        joins.push(0.5*(lo+hi));
        on.push(b);
    }
    // a loop on one face all round: one piece, from the pencil's first angle
    if joins.is_empty() { joins.push(0.); on.push(labels[0]) }
    Ok(Instant {twist,pencil,joins,on})
}

/// The instant at the roll `t` (`instant`) and the pose there.
fn read_instant(tool: &Revolved,path: &Trajectory,across: V,t: f64) -> Result<(Instant,crate::trajectory::Pose),Unbuilt> {
    let (m,pose) = (path.at(t)?,path.pose(t)?);
    let i = instant(tool,Twist::body(m),across).map_err(|at| Unbuilt::Curved {at:pose.point(at)})?;
    Ok((i,pose))
}

/// The turns round the loop that carry the pieces `on` an instant reads onto those `first` read:
/// each `s` with `on[k + s] == first[k]` all round.
fn turns<'a>(on: &'a [On],first: &'a [On]) -> impl Iterator<Item=usize> + 'a {
    let n = first.len();
    (0..n).filter(move |&s| on.len() == n && (0..n).all(|k| on[(k+s)%n] == first[k]))
}

/// **The sheet K2's contact sweeps, fitted** over `roll`: the net over (the fraction round the
/// loop, the roll) — its loop parameter running over `[0, pieces]`, a piece a unit — what each
/// piece lies on, and the roll.
#[derive(Clone,Debug)]
pub struct Fitted { pub net: Net,pub on: Vec<On>,pub roll: [f64;2] }

/// A piece of an instant's contact: over a stretch of the pencil's angles, or along a sharp edge's
/// circle (its radius and height on the tool's axis) over a stretch of its own angle.
#[derive(Clone,Copy,Debug)]
enum Span { Phi(f64,f64),Rim { rho: f64,z: f64,from: f64,sweep: f64 } }

/// The least number of pieces the loop is entered in: each piece a face of the sheet, a closed one
/// at least two round.
const LEAST_PIECES: usize = 4;

/// The roll's first rows and each piece's first points, and the most either may be refined to.
const FIRST_ROWS: usize = 16;
const FIRST_POINTS: usize = 8;
const MOST_ROWS: usize = 1024;
const MOST_POINTS: usize = 1024;

/// A direction square to every velocity the trajectory gives the tool's centre over the roll, as
/// near as one is: the one of the tool's axes and the two square to it whose least distance from
/// any of those velocities' lines is the greatest.
fn across(tool: &Revolved,path: &Trajectory,[a,b]: [f64;2]) -> Result<V,Unbuilt> {
    let ws: Vec<V> = (0..=32).map(|i| {
        let t = a+(b-a)*i as f64/32.;
        Ok(unit(Twist::body(path.at(t)?).velocity(tool.origin())))
    }).collect::<Result<_,Unbuilt>>()?;
    let g = tool.frame;
    Ok([g.z,g.x,g.y].into_iter().max_by(|&p,&q| {
        let least = |c: V| ws.iter().map(|&w| norm(cross(c,w))).fold(f64::INFINITY,f64::min);
        least(p).total_cmp(&least(q))
    }).unwrap())
}

/// **The sheet `tool`'s contact sweeps under `path` over `roll`, fitted within `tol`** (the
/// module's reading): refused by name where an instant's contact is no one closed curve read round
/// the pencil (`Curved`, at a point of the tool), where its pieces change over the roll — a contact
/// reaching a seam or an edge partway, where the patch is not yet cut (`Curved`) — or where the fit
/// will not close within its rows and points.
pub fn fitted(tool: &Revolved,path: &Trajectory,[a,b]: [f64;2],tol: f64) -> Result<Fitted,Unbuilt> {
    let across = across(tool,path,[a,b])?;
    // the pieces at the first instant, each entered whole or cut evenly until there are enough
    let (first,_) = read_instant(tool,path,across,a)?;
    let on = first.on.clone();
    let m = on.len();
    let per = LEAST_PIECES.div_ceil(m);
    let pieces = m*per;
    // an instant's pieces' ends: its joins turned so the pieces stand in the first instant's order
    // (of the turns that do, the one whose first join stands nearest the first instant's), each
    // piece cut evenly into `per`
    let spans = |i: &Instant,pose: &crate::trajectory::Pose| -> Result<Vec<Span>,Unbuilt> {
        let refused = || Unbuilt::Curved {at:pose.point(tool.cast(i.pencil.f1).0)};
        let wrap = |x: f64| (x+PI).rem_euclid(TAU)-PI;
        let s = turns(&i.on,&on).min_by(|&p,&q| wrap(i.joins[p]-first.joins[0]).abs().total_cmp(&wrap(i.joins[q]-first.joins[0]).abs()))
            .ok_or_else(refused)?;
        let mut joins = i.joins.clone();
        joins.rotate_left(s);
        let mut out = Vec::with_capacity(pieces);
        for k in 0..m {
            let p = joins[k];
            let mut q = if k+1 < m { joins[k+1] } else { joins[0]+TAU };
            while q <= p { q += TAU }
            match on[k] {
                // a fan's piece read along its edge's own circle, by its angle: the pencil's angle
                // runs as the square root of the way along it where the edge runs along the
                // velocity
                On::Edge(e) => {
                    let corner = tool.elements[e].ends()[1];
                    let angle = |phi: f64| -> Result<f64,Unbuilt> {
                        let (x,_) = contact(tool,&i.twist,&i.pencil,phi,false).ok_or_else(refused)?;
                        let l = tool.frame.local(x);
                        Ok(l[1].datan2(l[0]))
                    };
                    let (w0,wm,w1) = (angle(p)?,angle(0.5*(p+q))?,angle(q)?);
                    // the way round from the first end through the middle to the last
                    let up = (wm-w0).rem_euclid(TAU) < (w1-w0).rem_euclid(TAU);
                    let sweep = if up { (w1-w0).rem_euclid(TAU) } else { -(w0-w1).rem_euclid(TAU) };
                    for j in 0..per {
                        out.push(Span::Rim {rho:corner[0],z:corner[1],from:w0+sweep*j as f64/per as f64,sweep:sweep/per as f64})
                    }
                }
                On::Face(_) => for j in 0..per { out.push(Span::Phi(p+(q-p)*j as f64/per as f64,p+(q-p)*(j+1) as f64/per as f64)) },
            }
        }
        Ok(out)
    };
    // each roll's instant, its pose and its pieces' ends, read once over every round of the fit
    // (a round's rolls are the last round's and their middles)
    let read = std::sync::Mutex::new(std::collections::BTreeMap::<u64,(Instant,crate::trajectory::Pose,Vec<Span>)>::new());
    let reading = |t: f64| -> Result<(Instant,crate::trajectory::Pose,Vec<Span>),Unbuilt> {
        if let Some(r) = read.lock().unwrap().get(&t.to_bits()) { return Ok(r.clone()) }
        let (i,pose) = read_instant(tool,path,across,t)?;
        let e = spans(&i,&pose)?;
        read.lock().unwrap().insert(t.to_bits(),(i.clone(),pose,e.clone()));
        Ok((i,pose,e))
    };
    // the sheet's points at the fractions `us` round the loop (`[0, pieces]`) at the roll `t`
    let row = |t: f64,us: &[f64]| -> Result<Vec<V>,Unbuilt> {
        let (i,pose,e) = reading(t)?;
        us.iter().map(|&u| {
            let k = (u.floor() as usize).min(pieces-1);
            let f = u-k as f64;
            let x = match e[k] {
                Span::Phi(p,q) => contact(tool,&i.twist,&i.pencil,p+(q-p)*f,false).ok_or_else(|| Unbuilt::Curved {at:pose.point(tool.origin())})?.0,
                Span::Rim {rho,z,from,sweep} => { let (sw,cw) = (from+sweep*f).dsin_cos(); tool.frame.at([rho*cw,rho*sw,z]) }
            };
            Ok(pose.point(x))
        }).collect()
    };
    let (mut rows,mut points) = (FIRST_ROWS,FIRST_POINTS);
    loop {
        crate::util::meter::go_on()?;
        let ts: Vec<f64> = (0..=rows).map(|j| a+(b-a)*j as f64/rows as f64).collect();
        let half_ts: Vec<f64> = (0..rows).map(|j| 0.5*(ts[j]+ts[j+1])).collect();
        // round the loop at the fitted fractions and halfway between them (the odd ones)
        let all: Vec<f64> = (0..=2*pieces*points).map(|i| i as f64/(2*points) as f64).collect();
        let us: Vec<f64> = all.iter().step_by(2).copied().collect();
        // every point read at once: the fitted rows and their halfways round the loop, and the
        // fitted fractions at the halfways along the roll
        let jobs: Vec<(f64,bool)> = ts.iter().map(|&t| (t,true)).chain(half_ts.iter().map(|&t| (t,false))).collect();
        let read_rows: Vec<Vec<V>> = crate::util::par::map(&jobs,|&(t,fitted)| row(t,if fitted { &all } else { &us }))
            .into_iter().collect::<Result<_,_>>()?;
        let (fitted,halfway) = read_rows.split_at(ts.len());
        let grid: Vec<Vec<V>> = fitted.iter().map(|r| r.iter().step_by(2).copied().collect()).collect();
        let net = interpolated(&grid,&us,&ts,pieces,points)?;
        // the most the net misses the sheet by halfway round the loop, and halfway along the roll
        let err_u = missed(&net,ts.iter().zip(fitted).flat_map(|(&t,r)| all.iter().zip(r).skip(1).step_by(2).map(move |(&u,&x)| (t,u,x))));
        let err_t = missed(&net,half_ts.iter().zip(halfway).flat_map(|(&t,r)| us.iter().zip(r).map(move |(&u,&x)| (t,u,x))));
        let (grow_u,grow_t) = (err_u > tol,err_t > tol);
        if !grow_u && !grow_t {
            folds(&net,tool,path,&us,&ts)?;
            return Ok(Fitted {net,on:(0..m).flat_map(|k| std::iter::repeat_n(on[k],per)).collect(),roll:[a,b]})
        }
        if grow_u && points >= MOST_POINTS || grow_t && rows >= MOST_ROWS {
            return Err(Unbuilt::Failed(format!("the contact's sheet fits within {} only to {} along the loop and {} along the roll",
                tol,err_u,err_t)))
        }
        if grow_u { points *= 2 }
        if grow_t { rows *= 2 }
    }
}

/// The most the net misses the sheet's points `(t, u, x)` by.
fn missed(net: &Net,points: impl Iterator<Item=(f64,f64,V)>) -> f64 {
    points.map(|(t,u,x)| norm(sub(x,net.point(u,t)))).fold(0.,f64::max)
}

/// How small a part of its two rates' lengths the sheet's area factor may fall to before it is read
/// as folding.
const FOLD: f64 = 1e-3;

/// **Whether the fitted sheet folds or comes back onto itself**, refused by name if it does: at
/// every fitted point its normal `Σ_u × Σ_t` faces the way it faces at the first, against the way
/// out from the tool's centre where it stands there, and its length keeps above `FOLD` of the
/// rates' (a fold: the contact running along the motion's flow, design step 6, `Folded`); and the
/// tool's centre, read at the fitted rolls and between, comes back nowhere within the tool's reach
/// (`returns`). Sampled, so a fold or a return between the points is the judge's to see.
fn folds(net: &Net,tool: &Revolved,path: &Trajectory,us: &[f64],ts: &[f64]) -> Result<(),Unbuilt> {
    let mut sense = 0.;
    for &t in ts {
        let centre = path.pose(t)?.point(tool.origin());
        for &u in us {
            let (x,du,dt) = net.d1(u,t);
            let n = cross(du,dt);
            let way = dot(n,sub(x,centre));
            if norm(n) <= FOLD*norm(du)*norm(dt) || sense*way < 0. { return Err(Unbuilt::Folded {at:x}) }
            if sense == 0. { sense = way.signum() }
        }
    }
    returns(tool,path,[ts[0],ts[ts.len()-1]],4*ts.len().max(64))
}

/// **Whether the tool's centre comes back on itself**, read at `n` rolls over `[a, b]`: within
/// twice the tool's reach after running half a turn of a circle of it (the canal's B2,
/// `canal::first_return`, for any convex tool), `Returns` at a point between the two passes.
fn returns(tool: &Revolved,path: &Trajectory,[a,b]: [f64;2],n: usize) -> Result<(),Unbuilt> {
    let centres: Vec<V> = (0..=n).map(|i| path.pose(a+(b-a)*i as f64/n as f64).map(|m| m.point(tool.origin())))
        .collect::<Result<_,_>>()?;
    let mut along = vec![0.];
    for w in centres.windows(2) { along.push(along.last().unwrap()+norm(sub(w[1],w[0]))) }
    match super::canal::first_return(&centres,&along,tool.size) {
        Some((i,j,_)) => Err(Unbuilt::Returns {at:scale(add(centres[i],centres[j]),0.5)}),
        None => Ok(()),
    }
}

/// How many rolls K2's admission reads the instant's contact at.
const ADMITTED_AT: usize = 65;

/// **Whether K2's road takes `tool` under `path` over `[a, b]`**, read before anything is fitted
/// (what an admission asks, as `closed::reads` asks of a constant twist): the tool a convex solid
/// of revolution (`Revolved::read`); at `ADMITTED_AT` rolls, the instant's contact one root in each
/// half-plane of the pencil, its pieces the first's (`Curved` where not); and the centre's path
/// coming back on itself nowhere (`Returns`).
pub fn admits(tool: &Brep,path: &Trajectory,[a,b]: [f64;2]) -> Result<(),Unbuilt> {
    let revolved = Revolved::read(tool)?;
    let across = across(&revolved,path,[a,b])?;
    let ts: Vec<f64> = (0..ADMITTED_AT).map(|k| a+(b-a)*k as f64/(ADMITTED_AT-1) as f64).collect();
    let read = crate::util::par::map(&ts,|&t| read_instant(&revolved,path,across,t));
    let mut first: Option<Vec<On>> = None;
    for r in read {
        let (i,pose) = r?;
        match &first {
            None => first = Some(i.on),
            Some(on) => if turns(&i.on,on).next().is_none() { return Err(Unbuilt::Curved {at:pose.point(revolved.cast(i.pencil.f1).0)}) },
        }
    }
    returns(&revolved,path,[a,b],256)
}

/// The degree along the roll, and round the loop.
const ALONG: usize = 5;
const ROUND_DEGREE: usize = 5;

/// The net through `grid` (a row a roll `ts`, a point a fraction `us` round the loop): each row's
/// pieces quintic through their points, joined by a knot five times over (continuous, each piece
/// its own); then each pole's column quintic along the roll.
fn interpolated(grid: &[Vec<V>],us: &[f64],ts: &[f64],pieces: usize,points: usize) -> Result<Net,Unbuilt> {
    let failed = || Unbuilt::Failed("the contact's sheet could not be interpolated".into());
    let mut rows: Vec<(Vec<f64>,Vec<V>)> = Vec::with_capacity(grid.len());
    for r in grid {
        let (mut knots,mut poles): (Vec<f64>,Vec<V>) = (Vec::new(),Vec::new());
        for k in 0..pieces {
            let (i0,i1) = (k*points,(k+1)*points);
            let s = crate::nurbs::interpolate(&r[i0..=i1],&us[i0..=i1],ROUND_DEGREE).ok_or_else(failed)?;
            let d = s.degree;
            let (from,to) = (if k == 0 { 0 } else { d+1 },s.knots.len()-if k+1 == pieces { 0 } else { 1 });
            knots.extend_from_slice(&s.knots[from..to]);
            poles.extend_from_slice(&s.poles[if k == 0 { 0 } else { 1 }..]);
        }
        rows.push((knots,poles));
    }
    let uknots = rows[0].0.clone();
    let nu = rows[0].1.len();
    let cols: Vec<BSpline> = (0..nu).map(|i| crate::nurbs::interpolate(&rows.iter().map(|r| r.1[i]).collect::<Vec<_>>(),ts,ALONG))
        .collect::<Option<_>>().ok_or_else(failed)?;
    let net = Net {du:ROUND_DEGREE,dv:cols[0].degree,uknots,vknots:cols[0].knots.clone(),poles:cols.into_iter().map(|c| c.poles).collect(),weights:None};
    net.check().map_err(Unbuilt::Failed)?;
    Ok(net)
}

/// **The tool at the pose the roll `t` (an end of the fit's roll) gives it**, as the arrangement takes
/// it: the tool moved, and turned about its own axis — the same solid — so its seams stand clear of
/// its contact there, which it is given touching the sheet along (#810's `seam_away`); and a ball,
/// whose axis is its own to choose, laid anew with its poles along its centre's velocity there, the
/// axis of the circle it touches its sheet round, so the touch passes through no pole of its
/// surface, where it would be laid on no face (`solid_sweeps/path.sv`'s ball, drawn turned about a
/// diameter square to its path, has its poles on that circle).
pub fn posed(tool: &Brep,revolved: &Revolved,path: &Trajectory,fit: &Fitted,t: f64) -> Result<Brep,Unbuilt> {
    let pose = path.pose(t)?;
    if let ([Element::Arc {c,r,sweep,..}],[face]) = (&revolved.elements[..],&tool.faces[..]) {
        if c[0] == 0. && (sweep-PI).abs() <= 1e-9 {
            let centre = revolved.frame.at([0.,0.,c[1]]);
            let w = Twist::body(path.at(t)?).velocity(centre);
            if norm(w) > 0. {
                let mut ball = crate::build::sphere(pose.point(centre),unit(pose.vector(w)),*r).map_err(Unbuilt::Failed)?;
                for f in &mut ball.faces { f.name = face.name.clone() }
                return Ok(ball)
            }
        }
    }
    let n = 8*fit.on.len();
    let touched: Vec<V> = (0..n).map(|i| fit.net.point(fit.on.len() as f64*i as f64/n as f64,t)).collect();
    let moved = tool.moved(&crate::geom::Rigid {r:pose.r,t:pose.p});
    Ok(super::closed::seam_away(moved,pose.point(revolved.origin()),pose.vector(revolved.frame.z),&touched))
}

/// **The sheet entered as a candidate**: the net cut round the loop at each piece's ends — a tube
/// of faces (`build::grid_sheet`) named `sheet`, open at both ends of the roll, its rims the
/// contact where the tool stands there — and every meeting of a face of the tool at an end pose
/// (`[start, end]`) with a face of the sheet given: the piece's iso-curve at that end where the
/// face holds the piece there (on its face, or on an edge of it, which the sheet of its fan
/// crosses), and nothing where it does not. The sheet stands nowhere inside the tool at either end
/// pose — every point of it on the sweep's boundary over a roll holding that pose, short of a fold
/// or a return the build refuses — so these are their whole meetings: a face beside a piece meets
/// it only at the end of a curve given (where the contact crosses the seam between them).
pub fn entered(fit: &Fitted,tool: &Revolved,ends: [&Brep;2]) -> Result<(Brep,Vec<Given>),Unbuilt> {
    let net = &fit.net;
    let pieces = fit.on.len();
    let us: Vec<f64> = (0..=pieces).map(|k| k as f64).collect();
    let sheet = crate::build::grid_sheet(net,&us,&fit.roll,&vec!["sheet".to_string();pieces],[true,false]).map_err(Unbuilt::Failed)?;
    let mut given = Vec::new();
    for (k,&on) in fit.on.iter().enumerate() {
        // the faces holding the piece where the tool stands: its face's, or both of its edge's
        let holding: Vec<usize> = match on {
            On::Face(e) => vec![tool.faces[e]],
            On::Edge(e) => vec![tool.faces[e],tool.faces[e+1]],
        };
        // the net's iso-curves at the ends of the roll: the piece's first column and its last
        let piece = net.segment([k as f64,k as f64+1.],fit.roll);
        for (pose,j) in ends.iter().zip([0,piece.poles[0].len()-1]) {
            let iso = Curve::BSpline(Arc::new(piece.column(j)));
            for (f,face) in pose.faces.iter().enumerate() {
                let curves = if holding.contains(&f) { vec![iso.clone()] } else { Vec::new() };
                given.push(Given {surfaces:[face.surface.clone(),sheet.faces[k].surface.clone()],curves});
            }
        }
    }
    Ok((sheet,given))
}
