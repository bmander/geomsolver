//! **A ball-end mill's runs, in closed form** (brep-bench's `cnc/ball-end-surface-raster`): a
//! three-axis program of straight moves cut from a blank, its sweep built run by run from the
//! surfaces it is made of rather than from an arrangement of every piece's candidates at once.
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
//!   the ball's quarter behind the first step, at its end the quarter ahead of the last;
//! - above it, the walls the shank's sides sweep — two planes, a radius either side of the path's
//!   plane, standing on the half cylinders' sides — the shank's back half at the start and its
//!   front half at the end, and its top.
//!
//! Every shank's top stands above the blank, so what the sweep holds past the plane `top` along the
//! axis, a little above the blank and every ball centre of a run reaching it, takes nothing from
//! the blank: the sweep is cut off there by that plane, its top the stadium the shank's section
//! sweeps in it. Each run is cut from the blank in turn —
//! the passes of a raster meet across their scallops only, each corner of a run being laid by
//! construction, not found by a Boolean where two moves' balls coincide — and a run reaching no
//! part of the blank's box is left out. A corner turning by less than a tolerance's lune
//! (`r δ²/8` under `tol`) is a hill's, its lune too thin to lay; steps along one line are one step.
//!
//! What the road takes (`cut`): a path of pieces, each a slide, of a tool `Mill::read` takes, whose
//! top stands past that plane at every pose of a run reaching the blank, each such run turning at
//! its hills gently enough for its half cylinders' ends to stand apart (`laid`). Anything else is
//! the closed class's (`None`), as is a path whose runs the road could not sweep or cut.
use crate::boolean::{combined_known,overlap,Known,Op};
use crate::build::{face,kept,prism,revolve,seam,sphere,Profile,ProfileEdge,SEAM};
use crate::geom::{Frame,Surface};
use crate::query::{Located,Place};
use crate::topo::Brep;
use crate::trajectory::Trajectory;
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use super::closed::Unbuilt;
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

/// A run: the ball's centre at each corner, in order (two or more), and the way across the axis
/// its steps run (none for a step along the axis, which is a run of its own).
struct Run { cs: Vec<V>,across: Option<V> }

/// The steps the ball's centre takes over `roll` along `path`, each a slide — `None` where a piece
/// is no slide, the path no path of pieces, or a pose turns the mill's axis off the first's.
fn steps(path: &Trajectory,roll: [f64;2],m: &Mill) -> Option<(Vec<[V;2]>,V)> {
    let Trajectory::Pieces(pieces) = path else { return None };
    let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let mut out = Vec::new();
    let mut axis: Option<V> = None;
    for piece in pieces.pieces() {
        let (lo,hi) = (piece.over[0].max(a),piece.over[1].min(b));
        if !(hi > lo) { continue }
        let slide = match &piece.path { Trajectory::Twist(s) | Trajectory::Placed {twist:s,..} => s.is_slide(),_ => false };
        if !slide { return None }
        let (p,q) = (piece.path.pose(lo).ok()?,piece.path.pose(hi).ok()?);
        let w = unit(p.vector(m.axis));
        if axis.is_some_and(|x| norm(cross(x,w)) > 1e-12 || dot(x,w) < 0.) { return None }
        axis.get_or_insert(w);
        out.push([p.point(m.c),q.point(m.c)]);
    }
    Some((out,axis?))
}

/// The steps in runs (`Run`): a step whose way across the axis is the run's goes on with it, any
/// other starts a run; a step along the axis is a run alone, and a step of no length none.
fn runs(steps: &[[V;2]],axis: V,eps: f64) -> Vec<Run> {
    let mut out: Vec<Run> = Vec::new();
    for &[p,q] in steps {
        let d = sub(q,p);
        if !(norm(d) > eps) { continue }
        let flat = sub(d,scale(axis,dot(d,axis)));
        let across = (norm(flat) > 1e-9*norm(d)).then(|| unit(flat));
        let goes_on = out.last().is_some_and(|r| match (r.across,across) {
            (Some(x),Some(y)) => dot(x,y) >= 1.-1e-12 && norm(sub(*r.cs.last().unwrap(),p)) <= eps,
            _ => false,
        });
        if goes_on { out.last_mut().unwrap().cs.push(q) } else { out.push(Run {cs:vec![p,q],across}) }
    }
    out
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
fn run_sweep(cs: &[V],across: V,a: V,r: f64,top: f64,tol: f64) -> Result<Brep,String> {
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
    faces.push(lune(cs[0],r,&[below,scale(t[0],-1.)],frame)?);
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
    faces.push(shank(cs[0],e2)?);
    faces.push(shank(cs[n],scale(e2,-1.))?);
    // the top: the stadium the shank's section sweeps in the plane `top`
    // (the back half round the start, along one side, the front half round the end, back along the
    // other)
    let (t0,t1) = (up(cs[0]),up(cs[n]));
    let stadium = [(t0,t1,e2),(t1,t0,scale(e2,-1.))].into_iter().flat_map(|(p,q,x)| [
        ProfileEdge::Arc {frame:Frame::new(p,a,x),r,span:Some([0.,PI])},
        ProfileEdge::Line {a:add(p,scale(x,-r)),b:add(q,scale(x,-r))}]).collect();
    faces.push(face(&Profile {origin:t0,normal:a,names:Vec::new(),loops:vec![stadium]})?);
    crate::sew::sewn(&faces,tol).map_err(|e| e.msg)
}

/// **A step along the axis, cut off at `top`**: the mill where its centre is lowest, `c`, its shank
/// run up to `top` — the ball's lower half, the cylinder and the disc at `top`.
fn plunge(c: V,a: V,r: f64,top: f64,tol: f64) -> Result<Brep,String> {
    let f = Frame::about(c,a);
    let ball = lune(c,r,&[scale(a,-1.)],[f.x,f.y,f.z])?;
    let side = cylinders(&prism(&disc(c,a,f.x,r),0.,top-dot(c,a))?)?;
    let cap = face(&disc(add(c,scale(a,top-dot(c,a))),a,f.x,r))?;
    crate::sew::sewn(&[ball,side,cap],tol).map_err(|e| e.msg)
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

/// Whether `run_sweep` lays the run through `cs` as it is: at every hill the turn short of a right
/// angle, and along every step the ellipses cutting its half cylinder at either end — each reaching
/// `r tan(β/2)` along it from its corner, `β` the hill's bend — clear of each other.
fn laid(cs: &[V],across: V,a: V,r: f64,tol: f64) -> bool {
    let (bend,valley) = (bends(cs,across,a),valleys(cs,across,a,r,tol));
    let reach = |k: usize| if valley[k] { 0. } else { r*(0.5*bend[k].abs()).tan() };
    bend.iter().all(|b| b.abs() < 0.5*PI) && (0..cs.len()-1).all(|k| reach(k)+reach(k+1) < norm(sub(cs[k+1],cs[k])))
}

/// **`blank` less what `tool` sweeps along `path` over `roll`, run by run** (the module's account),
/// judged as the closed class judges what it builds (`closed::judge`, its tool meshed to `bar`) —
/// or `None` where the road does not take the sweep: the tool no mill `Mill::read` takes, the path
/// no path of pieces each a slide keeping the mill's axis, a run reaching the blank turning too
/// sharply to lay (`laid`), or a pose of one leaving the mill's top short of the plane the sweep is
/// cut off at: a hundredth of the blank's size past the blank and every reaching run's ball centres.
pub fn cut(blank: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Option<Result<Brep,Unbuilt>> {
    if !matches!(path,Trajectory::Pieces(_)) { return None }
    let m = Mill::read(tool,tol)?;
    let (steps,a) = steps(path,roll,&m)?;
    let bounds = blank.bounds();
    if !bounds.0[0].is_finite() { return None }
    let eps = 1e-9*(1.+tool.size());
    // the runs, steps along one line made one, and of them those whose mill's box at its every pose
    // reaches the blank's (one that misses it takes nothing from it)
    let runs: Vec<(usize,Run)> = runs(&steps,a,eps).into_iter().map(|r| Run {cs:straightened(&r.cs,0.0625*tol),..r})
        .enumerate().filter(|(_,run)| {
            let reach = crate::geom::hull(run.cs.iter().flat_map(|&c| [c,add(c,scale(a,m.h))]));
            overlap(&reach,&bounds,m.r+tol)
        }).collect();
    if runs.iter().any(|(_,run)| run.across.is_some_and(|across| !laid(&run.cs,across,a,m.r,tol))) { return None }
    let along = |p: &V| dot(*p,a);
    let highest = super::closed::corners_of(bounds).iter().chain(runs.iter().flat_map(|(_,run)| &run.cs)).map(along)
        .fold(f64::NEG_INFINITY,f64::max);
    let top = highest+1e-2*blank.size();
    let lowest_top = runs.iter().flat_map(|(_,run)| &run.cs).map(|c| along(c)+m.h).fold(f64::INFINITY,f64::min);
    if !(lowest_top > top) { return None }
    // (a run the road could not sweep or cut leaves the path to the arrangement)
    match cut_runs(blank,tool,path,roll,m.r,a,&runs,top,bar,tol) {
        Err(Unbuilt::Failed(why)) => {
            if crate::debug() { eprintln!("raster: {why}; left to the arrangement") }
            None
        }
        cut => Some(cut),
    }
}

/// `cut`'s runs (each with its place in the path's) cut from the blank in turn, and judged: each
/// swept across the cores, every sweep being its own, then each cut in the path's order, what each
/// Boolean knows of the faces it hands on handed to the next (`boolean::Known`).
#[allow(clippy::too_many_arguments)]
fn cut_runs(blank: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2],r: f64,a: V,runs: &[(usize,Run)],top: f64,bar: f64,tol: f64)
    -> Result<Brep,Unbuilt> {
    let clock = crate::util::clock::Instant::now();
    let sweeps = crate::util::par::map(runs,|(i,run)| match run.across {
        Some(across) => run_sweep(&run.cs,across,a,r,top,tol),
        None => {
            let c = run.cs.iter().copied().min_by(|x,y| dot(*x,a).total_cmp(&dot(*y,a))).unwrap();
            plunge(c,a,r,top,tol)
        }
    }.map_err(|why| Unbuilt::Failed(format!("run {i} of the mill's path, from {:?}: its sweep not built: {why}",run.cs[0]))));
    let swept_at = clock.elapsed().as_secs_f64();
    let (mut out,mut known): (Option<Brep>,Option<Known>) = (None,None);
    for ((i,run),sweep) in runs.iter().zip(sweeps) {
        let (cut,next) = combined_known(out.as_ref().unwrap_or(blank),known.as_ref(),&sweep?,Op::Cut,tol)
            .map_err(|why| Unbuilt::Failed(format!("run {i} of the mill's path, from {:?}: not cut: {}",run.cs[0],why.msg)))?;
        (out,known) = (Some(cut),Some(next));
    }
    let out = out.unwrap_or_else(|| blank.clone());
    let cut_at = clock.elapsed().as_secs_f64();
    let solid = super::member::Solid::new(tool,bar,1.).map_err(Unbuilt::Failed)?;
    let swept = super::member::Swept {tool:&solid,path,roll};
    super::closed::judge(&out,&swept,blank,Some(blank),&[],tol)?;
    if crate::timing() {
        eprintln!("raster: {} runs reach the blank, swept in {swept_at:.2} s, cut by {cut_at:.2} s, judged by {:.2} s: {} faces",
            runs.len(),clock.elapsed().as_secs_f64(),out.faces.len());
    }
    Ok(out)
}
