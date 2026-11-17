//! **A ball-end mill's runs, in closed form** (brep-bench's `cnc/ball-end-surface-raster`): what
//! `swept::pieces` sweeps for a ball-end mill's slides at a slant one after another, its corners
//! laid by construction rather than met in a Boolean.
//!
//! The mill is read off its B-rep (`Mill::read`): a half ball below its centre, a cylinder of the
//! same radius about the axis through it from its equator up, and a flat top square to the axis.
//! Under a slide it is convex, so what it sweeps over a move is the convex hull of where it starts
//! and where it ends. A **run** is moves one after another whose steps share one way across the
//! axis (each forward along it, all in one plane holding the axis): over it the ball's centre runs
//! a polyline in that plane, and the sweep is
//!
//! - below the plane through the centre's path square to that plane, each step's half cylinder
//!   about the step, and at each corner where the path turns up (a valley) the ball's lune between
//!   the two steps' great circles square to them — where it turns down (a hill) the two half
//!   cylinders meet in the plane bisecting the turn, along an ellipse each cuts there (two
//!   cylinders of one radius whose axes meet are each other's mirror in it); at the run's start
//!   the ball's quarter behind the first step (or, where the piece before holds the tool there,
//!   the section it is shut by), at its end the quarter ahead of the last;
//! - above it, the walls the shank's sides sweep — two planes, a radius either side of the path's
//!   plane, standing on the half cylinders' sides — the shank's back half at the start and its
//!   front half at the end, and its top.
//!
//! Every shank's top stands above the blank, so what the sweep holds past the plane `top` along the
//! axis, a little above the blank and every ball centre of the run, takes nothing from the blank:
//! the sweep is cut off there by that plane, its top the stadium the shank's section sweeps in it.
//! A corner turning by less than a tolerance's lune (`r δ²/8` under `tol`) is a hill's, its lune
//! too thin to lay; steps along one line are one step; and a run ends where it would turn at a hill
//! too sharply for its half cylinders' ends to stand apart (`laid`).
use crate::build::{face,kept,prism,revolve,seam,sphere,Profile,ProfileEdge,SEAM};
use crate::geom::{Frame,Surface};
use crate::query::{Located,Place};
use crate::topo::Brep;
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use std::f64::consts::PI;

type V = [f64;3];

/// An upright ball-end mill as drawn: its ball's centre, the unit axis from its tip up its shank,
/// its radius, and the height of its flat top above the centre.
#[derive(Clone,Copy,Debug)]
pub struct Mill { pub c: V,pub axis: V,pub r: f64,pub h: f64 }

impl Mill {
    /// **The mill `tool` is, where it is one**: three faces — a sphere, a cylinder of its radius
    /// about an axis through its centre, a plane square to that axis a height `h` off the centre —
    /// the sphere's face the half below the centre (its tip on it, its top not), and the solid's
    /// volume the half ball's and the shank's, `2πr³/3 + πr²h`, to `1e-9` of it.
    pub fn read(tool: &Brep,tol: f64) -> Option<Mill> {
        if tool.faces.len() != 3 { return None }
        let (ks,c,r) = tool.faces.iter().enumerate().find_map(|(k,f)| match f.surface { Surface::Sphere(sf,r) => Some((k,sf.o,r.abs())),_ => None })?;
        let (cf,rc) = tool.faces.iter().find_map(|f| match f.surface { Surface::Cylinder(cf,rc) => Some((cf,rc.abs())),_ => None })?;
        let pf = tool.faces.iter().find_map(|f| match f.surface { Surface::Plane(pf) => Some(pf),_ => None })?;
        let eps = 1e-9*(1.+r);
        let along = unit(cf.z);
        let off = sub(c,cf.o);
        if (r-rc).abs() > eps || norm(sub(off,scale(along,dot(off,along)))) > eps { return None }
        if norm(cross(unit(pf.z),along)) > 1e-9 { return None }
        let s = dot(sub(pf.o,c),along);
        if !(s.abs() > eps) { return None }
        let axis = scale(along,s.signum());
        let h = s.abs();
        let volume = 2.*PI*r*r*r/3.+PI*r*r*h;
        if (crate::props::volume(tool)-volume).abs() > 1e-9*volume { return None }
        let located = Located::new(tool,tol);
        (located.face_place(ks,sub(c,scale(axis,r))) != Place::Out && located.face_place(ks,add(c,scale(axis,r))) == Place::Out)
            .then_some(Mill {c,axis,r,h})
    }
}

/// The corners of `cs` that lie on the line through their neighbours, to `eps`, left out: steps
/// along one line are one step.
fn straightened(cs: &[V],eps: f64) -> Vec<V> {
    let mut out: Vec<V> = vec![cs[0]];
    for k in 1..cs.len() {
        if k+1 < cs.len() {
            let (a,b) = (*out.last().unwrap(),cs[k+1]);
            let t = unit(sub(b,a));
            let off = sub(cs[k],a);
            if norm(sub(off,scale(t,dot(off,t)))) <= eps && dot(off,t) > 0. && dot(sub(b,cs[k]),t) > 0. { continue }
        }
        out.push(cs[k]);
    }
    out
}

/// The faces of `b` whose surfaces `which` takes, alone with the edges and vertices they use
/// (`offset::sheet_of`).
fn faces_of(b: &Brep,which: impl Fn(&Surface) -> bool) -> Result<Brep,String> {
    let faces: Vec<usize> = (0..b.faces.len()).filter(|&k| which(&b.faces[k].surface)).collect();
    if faces.is_empty() { return Err("a run's piece came out with no face on its surface".into()) }
    crate::offset::sheet_of(b,&faces)
}

/// `b`'s cylinder faces alone.
fn cylinders(b: &Brep) -> Result<Brep,String> { faces_of(b,|s| matches!(s,Surface::Cylinder(..))) }

/// The polygon on `pts` (in a plane square to `normal`).
fn polygon(pts: &[V],normal: V) -> Profile {
    let n = pts.len();
    Profile {origin:pts[0],normal,names:Vec::new(),loops:vec![(0..n).map(|k| ProfileEdge::Line {a:pts[k],b:pts[(k+1)%n]}).collect()]}
}

/// The disc of radius `r` about `c` square to `n`, its circle's seam where `x` turned about `n` by
/// `build::SEAM` points (`build::seam`), off every plane a run's piece is cut at.
fn disc(c: V,n: V,x: V,r: f64) -> Profile {
    let x = seam(unit(n),Frame::new(c,n,x).x);
    Profile {origin:c,normal:n,names:Vec::new(),loops:vec![vec![ProfileEdge::Arc {frame:Frame::new(c,n,x),r,span:None}]]}
}

/// The part of the sphere of radius `r` about `c` on the side of each plane through `c` its normal
/// in `cuts` points to: a ball whose poles stand off every one of those planes (leaning by
/// `build::SEAM` in three ways, the clearest taken), cut down (`build::kept`) — a plane through a
/// pole is a meeting the Booleans do not split.
fn lune(c: V,r: f64,cuts: &[V],[e1,e2,a]: [V;3]) -> Result<Brep,String> {
    let candidates = [[SEAM,0.5*SEAM,1.],[-0.5*SEAM,SEAM,1.],[0.5*SEAM,-SEAM,1.]]
        .map(|w| unit(add(add(scale(e1,w[0]),scale(e2,w[1])),scale(a,w[2]))));
    let clear = |w: V| cuts.iter().map(|n| dot(w,unit(*n)).abs()).fold(f64::INFINITY,f64::min);
    let w = candidates.into_iter().max_by(|x,y| clear(*x).total_cmp(&clear(*y))).unwrap();
    let mut b = sphere(c,w,r)?;
    for &n in cuts { b = kept(&b,c,unit(n),0.)?; }
    faces_of(&b,|s| matches!(s,Surface::Sphere(..)))
}

/// **A run's sweep, cut off at `top`** (the module's account): the ball's centre at `cs` (two or
/// more, along `across`, each step forward), the mill of radius `r` with its axis `a`.
fn run_sweep(cs: &[V],across: V,a: V,r: f64,top: f64,back: bool,tol: f64) -> Result<Brep,String> {
    let n = cs.len()-1;
    let e1 = across;
    let e2 = cross(a,e1);
    let frame = [e1,e2,a];
    let d: Vec<V> = (0..n).map(|k| sub(cs[k+1],cs[k])).collect();
    let t: Vec<V> = d.iter().map(|&x| unit(x)).collect();
    // each corner a valley (the path turning up past a tolerance's lune) or not; the plane each
    // half cylinder ends at there: square to its step at an end of the run and at a valley, else
    // the plane bisecting the turn
    let valley = valleys(cs,e1,a,r,tol);
    let bisector = |k: usize| unit(add(t[k-1],t[k]));
    let mut faces: Vec<Brep> = Vec::new();
    let below = scale(a,-1.);
    // the ball's quarter behind the first step and ahead of the last, its lune at each valley
    if back { faces.push(lune(cs[0],r,&[below,scale(t[0],-1.)],frame)?); }
    faces.push(lune(cs[n],r,&[below,t[n-1]],frame)?);
    for k in (1..n).filter(|&k| valley[k]) {
        faces.push(lune(cs[k],r,&[t[k-1],scale(t[k],-1.)],frame)?);
    }
    // each step's half cylinder below the plane of its axis and `e2`, between its two ends
    for k in 0..n {
        // (square to the step and to `e2`, downward: each step runs forward along `e1`)
        let low = unit(cross(e2,t[k]));
        let start = if k == 0 || valley[k] { t[k] } else { bisector(k) };
        let end = if k+1 == n || valley[k+1] { scale(t[k],-1.) } else { scale(bisector(k+1),-1.) };
        let cylinder = prism(&disc(sub(cs[k],scale(t[k],2.*r)),t[k],low,r),0.,norm(d[k])+4.*r)?;
        let mut b = kept(&cylinder,cs[k],low,0.)?;
        b = kept(&b,cs[k],start,0.)?;
        b = kept(&b,cs[k+1],end,0.)?;
        faces.push(cylinders(&b)?);
    }
    // the walls standing on the half cylinders' sides, up to `top`
    let up = |p: V| add(p,scale(a,top-dot(p,a)));
    for s in [1.,-1.] {
        let mut pts: Vec<V> = cs.iter().map(|&c| add(c,scale(e2,s*r))).collect();
        let (first,last) = (pts[0],pts[n]);
        pts.extend([up(last),up(first)]);
        faces.push(face(&polygon(&pts,e2))?);
    }
    // the shank's back half at the start and its front half at the end, up to `top`
    let shank = |c: V,from: V| -> Result<Brep,String> {
        let p = polygon(&[c,add(c,scale(from,r)),up(add(c,scale(from,r))),up(c)],cross(from,a));
        cylinders(&revolve(&p,c,a,PI)?)
    };
    let up_ = |p: V| add(p,scale(a,top-dot(p,a)));
    let (c0,side) = (cs[0],scale(e2,r));
    if back { faces.push(shank(c0,e2)?) } else {
        // shut where it starts, its half behind there left out (`back`): the half disc of the first
        // step's half cylinder's end, and the shank's section above it
        let low = unit(cross(e2,t[0]));
        faces.push(face(&Profile {origin:c0,normal:t[0],names:Vec::new(),loops:vec![vec![
            ProfileEdge::Arc {frame:Frame {o:c0,x:e2,y:low,z:cross(e2,low)},r,span:Some([0.,PI])},
            ProfileEdge::Line {a:sub(c0,side),b:add(c0,side)}]]})?);
        faces.push(face(&polygon(&[add(c0,side),up_(add(c0,side)),up_(sub(c0,side)),sub(c0,side)],e1))?);
    }
    faces.push(shank(cs[n],scale(e2,-1.))?);
    // the top: the stadium the shank's section sweeps in the plane `top`
    // (the back half round the start, along one side, the front half round the end, back along the
    // other)
    let (t0,t1) = (up(cs[0]),up(cs[n]));
    let stadium = [(t0,t1,e2),(t1,t0,scale(e2,-1.))].into_iter().enumerate().flat_map(|(k,(p,q,x))| [
        if k == 0 && !back { ProfileEdge::Line {a:add(p,scale(x,r)),b:add(p,scale(x,-r))} }
        else { ProfileEdge::Arc {frame:Frame::new(p,a,x),r,span:Some([0.,PI])} },
        ProfileEdge::Line {a:add(p,scale(x,-r)),b:add(q,scale(x,-r))}]).collect();
    faces.push(face(&Profile {origin:t0,normal:a,names:Vec::new(),loops:vec![stadium]})?);
    crate::sew::sewn(&faces,tol).map_err(|e| e.msg)
}

/// The bend at each corner of the run through `cs` (two or more, each step forward along `across`,
/// `a` the axis): how far the path turns up there (a valley past nought, a hill short of it),
/// nought at its ends.
fn bends(cs: &[V],across: V,a: V) -> Vec<f64> {
    let slope = |k: usize| { let d = sub(cs[k+1],cs[k]); dot(d,a).atan2(dot(d,across)) };
    (0..cs.len()).map(|k| if k == 0 || k+1 == cs.len() { 0. } else { slope(k)-slope(k-1) }).collect()
}

/// The least bend a valley's lune is laid at: one turning by less leaves a lune no thicker than
/// `r δ²/8`, short of the tolerance, and is a hill's.
fn least_bend(r: f64,tol: f64) -> f64 { (tol/r).sqrt() }

/// Which corners of the run through `cs` are valleys: turning up past `least_bend`.
fn valleys(cs: &[V],across: V,a: V,r: f64,tol: f64) -> Vec<bool> {
    bends(cs,across,a).into_iter().map(|b| b > least_bend(r,tol)).collect()
}

/// **A ball-end mill's run of slides at a slant**, its ball's centre through `cs` (two or more, in
/// one plane holding the axis `axis`, each step forward along one way across it): straightened
/// (steps along one line one step) and swept (`run_sweep`), cut off past `blank` and every centre, a
/// hundredth of the blank's size beyond the farthest of them along the axis — refused where the
/// mill's top at a corner stands short of that, or the run turns too sharply to lay (`laid`) — and,
/// where not `back`, shut where it starts by its section there, its half behind left out.
pub(super) fn run(cs: &[V],axis: V,m: &Mill,blank: &Brep,back: bool,tol: f64) -> Result<Brep,String> {
    let along = |p: &V| dot(*p,axis);
    let highest = super::closed::corners_of(blank.bounds()).iter().chain(cs).map(along).fold(f64::NEG_INFINITY,f64::max);
    let top = highest+1e-2*blank.size();
    if !(cs.iter().map(along).fold(f64::INFINITY,f64::min)+m.h > top) { return Err("the mill's top dips under the plane its sweep is cut off at".into()) }
    let d = sub(cs[1],cs[0]);
    let across = unit(sub(d,scale(axis,dot(d,axis))));
    let cs = straightened(cs,0.0625*tol);
    if !laid(&cs,across,axis,m.r,tol) { return Err("the run turns at a hill too sharply for its half cylinders' ends to stand apart".into()) }
    run_sweep(&cs,across,axis,m.r,top,back,tol)
}

/// Whether `run_sweep` lays the run through `cs` as it is: at every hill the turn short of a right
/// angle, and along every step the ellipses cutting its half cylinder at either end — each reaching
/// `r tan(β/2)` along it from its corner, `β` the hill's bend — clear of each other.
pub(super) fn laid(cs: &[V],across: V,a: V,r: f64,tol: f64) -> bool {
    let (bend,valley) = (bends(cs,across,a),valleys(cs,across,a,r,tol));
    let reach = |k: usize| if valley[k] { 0. } else { r*(0.5*bend[k].abs()).tan() };
    bend.iter().all(|b| b.abs() < 0.5*PI) && (0..cs.len()-1).all(|k| reach(k)+reach(k+1) < norm(sub(cs[k+1],cs[k])))
}
