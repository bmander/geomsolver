//! **A path of pieces cut from a blank, piece by piece** (#721 K1's sweep, its pieces' sweeps
//! united, taken one piece at a time): a toolpath of many moves — slides and turns in any direction,
//! overlapping each other and the blank — cut in the path's order, each move's sweep built alone by
//! the cheapest road that takes it, the moves that reach no part of the blank left out, and the cut
//! judged as the closed class judges what it builds.
//!
//! A tool of revolution (`patches::Revolved`: convex, its meridian a chain of segments and arcs
//! from its axis below round to its axis above) under one move sweeps, in closed form:
//!
//! - **slid along its axis**: the tool with its meridian parted where it reaches farthest from the
//!   axis and drawn out there by the slide — one solid of revolution;
//! - **slid square to its axis**: its contact is its section through the axis square to the slide,
//!   so the sweep is the tool's half behind that plane where it starts, the section carried along
//!   the slide (its edges' sheets: planes, cylinders about lines along the slide) and the half ahead
//!   where it ends;
//! - **turned about a line along its axis** that misses it (a G2 or G3 arc): likewise, its section
//!   through both axes turned about the line (planes, cylinders, cones and tori about it) between
//!   its halves at the two ends — a whole turn that section's solid of revolution alone;
//! - **a ball-end mill slid at a slant** (`raster`): its half cylinder, its ball's quarters and its
//!   shank's walls, cut off past the blank.
//!
//! Each is its faces sewn (`sew::sewn`), nothing intersected. Any other move is the closed class's
//! own build of that piece alone (`closed::closed`). A path whose tool the road does not read, or
//! that it could not sweep or cut, is the closed class's whole (`None`).
use crate::boolean::{combined_known,overlap,Known,Op};
use crate::build::{prism,revolve,Profile,ProfileEdge};
use crate::geom::{Frame,Surface};
use crate::topo::Brep;
use crate::trajectory::{Pose,Trajectory};
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use super::closed::Unbuilt;
use super::patches::{Element,Revolved};
use std::f64::consts::{PI,TAU};

type V = [f64;3];
type P2 = [f64;2];

/// What one piece sweeps, as this road builds it.
#[derive(Clone,Copy,Debug)]
enum Move {
    /// Slid along the axis by `s` (from the pose `at`).
    Along { at: Pose,s: f64 },
    /// Slid along the axis by `s` from the pose `at`, but for the tool where it stands lowest,
    /// which a chain beside it holds (`lift`).
    Lifted { at: Pose,s: f64 },
    /// Slid by `d`, square to the axis.
    Across { at: Pose,d: V },
    /// Turned by `angle` about the line through `q` along the unit `w`, which runs along the
    /// tool's axis and misses it.
    Turned { at: Pose,q: V,w: V,angle: f64 },
    /// A ball-end mill slid at a slant, its ball's centre from `from` to `to`, its axis `axis`.
    Slanted { from: V,to: V,axis: V },
    // (each but `Along` and `Closed` built `back` or not: with its half behind where it starts, or
    // shut there by its section, the piece before it holding that half — `cut`)
    /// Anything else: the closed class's build of the piece over `roll`.
    Closed { roll: [f64;2] },
}

/// The tool's meridian point `(ρ, z)` in the plane through `o` spanned by `u` (across) and `a`
/// (along the axis).
fn at(o: V,u: V,a: V,p: P2) -> V { add(o,add(scale(u,p[0]),scale(a,p[1]))) }

/// An element of the meridian drawn in that plane, mirrored across the axis where `mirror`.
fn edge(e: &Element,o: V,u: V,a: V,mirror: bool) -> ProfileEdge {
    let u = if mirror { scale(u,-1.) } else { u };
    match *e {
        Element::Line {a:p,b:q} => ProfileEdge::Line {a:at(o,u,a,p),b:at(o,u,a,q)},
        Element::Arc {c,r,from,sweep} => ProfileEdge::Arc {frame:Frame {o:at(o,u,a,c),x:u,y:a,z:cross(u,a)},r,span:Some([from,from+sweep])},
    }
}

/// Whether an element is a segment across the axis at one height (a flat end's disc).
fn flat(e: &Element,eps: f64) -> bool { matches!(*e,Element::Line {a,b} if (a[1]-b[1]).abs() <= eps) }

/// **The tool's whole section through its axis**, in the plane through `o` spanned by `u` and `a`:
/// its meridian on the side of `u` and mirrored on the other, a flat end's two halves one segment.
fn section(rev: &Revolved,o: V,u: V,a: V,eps: f64) -> Profile {
    let n = rev.elements.len();
    let mut edges = Vec::with_capacity(2*n);
    for (k,e) in rev.elements.iter().enumerate() {
        let end = k == 0 || k+1 == n;
        if end && flat(e,eps) {
            // the disc's segment from the axis, both sides at once
            let Element::Line {a:p,b:q} = *e else { unreachable!() };
            let r = p[0].max(q[0]);
            edges.push(ProfileEdge::Line {a:at(o,u,a,[-r,p[1]]),b:at(o,u,a,[r,p[1]])});
        } else if let (true,Element::Arc {c,r,from,sweep}) = (end,*e) {
            if c[0].abs() > eps { edges.push(edge(e,o,u,a,false)); edges.push(edge(e,o,u,a,true)); continue }
            // a ball's arc from the axis (or to it, at the top), both sides one arc through the axis
            let span = if k == 0 { [from-sweep,from+sweep] } else { [from,from+2.*sweep] };
            edges.push(ProfileEdge::Arc {frame:Frame {o:at(o,u,a,c),x:u,y:a,z:cross(u,a)},r,span:Some(span)});
        } else {
            edges.push(edge(e,o,u,a,false));
            edges.push(edge(e,o,u,a,true));
        }
    }
    Profile {origin:o,normal:cross(u,a),loops:vec![edges],names:Vec::new()}
}

/// **The tool's half on the side `m` points to**: its meridian in the plane through `o` holding the
/// axis `a` square to `m`, turned half a turn about the axis toward `m` — its faces, without the
/// two half sections it is shut by.
fn half(rev: &Revolved,o: V,a: V,m: V) -> Result<Brep,String> {
    let u = cross(m,a);
    let n = rev.elements.len();
    let top = rev.elements[n-1].ends()[1];
    let bottom = rev.elements[0].ends()[0];
    let mut edges: Vec<ProfileEdge> = rev.elements.iter().map(|e| edge(e,o,u,a,false)).collect();
    edges.push(ProfileEdge::Line {a:at(o,u,a,top),b:at(o,u,a,bottom)});
    let p = Profile {origin:o,normal:cross(u,a),loops:vec![edges],names:Vec::new()};
    // (turned right-handed about `a`, `u` goes to `a × u = m`)
    without_planes(&revolve(&p,o,a,PI)?,m)
}

/// `b`'s faces but those on a plane square to `m`: what is left of a section carried along `m`
/// without the section at either end.
fn without_planes(b: &Brep,m: V) -> Result<Brep,String> { faces_but(b,|n| norm(cross(n,m)) <= 1e-9) }

/// `b`'s faces but those on a plane holding the line along `w`: what is left of a section turned
/// about `w` without the section at either end.
fn without_planes_holding(b: &Brep,w: V) -> Result<Brep,String> { faces_but(b,|n| dot(n,w).abs() <= 1e-9) }

/// `b`'s faces but those on a plane whose unit normal `drop` takes.
fn faces_but(b: &Brep,drop: impl Fn(V) -> bool) -> Result<Brep,String> {
    let keep: Vec<usize> = (0..b.faces.len()).filter(|&k| !matches!(b.faces[k].surface,Surface::Plane(f) if drop(unit(f.z)))).collect();
    crate::offset::sheet_of(b,&keep)
}

/// **The tool slid along its axis by `s`** from the pose `at`: its meridian parted where it reaches
/// farthest from the axis (along a side running with the axis, its lower end) and drawn out there by
/// the slide, turned once.
fn along(rev: &Revolved,at_: Pose,s: f64,eps: f64) -> Result<Brep,String> {
    let (o,a) = (at_.point(rev.frame.o),at_.vector(rev.frame.z));
    let u = at_.vector(rev.frame.x);
    // slid down, the same as slid up from where it ends
    let (o,s) = if s < 0. { (add(o,scale(a,s)),-s) } else { (o,s) };
    let reach = rev.elements.iter().flat_map(|e| e.ends()).map(|p| p[0]).fold(0.,f64::max);
    let reach = rev.elements.iter().fold(reach,|r,e| match *e { Element::Arc {c,r:rr,..} => r.max(c[0]+rr),_ => r });
    let mut out: Vec<Element> = Vec::new();
    let mut lifted = false;
    let lift = |e: Element| match e {
        Element::Line {a,b} => Element::Line {a:[a[0],a[1]+s],b:[b[0],b[1]+s]},
        Element::Arc {c,r,from,sweep} => Element::Arc {c:[c[0],c[1]+s],r,from,sweep},
    };
    for &e in &rev.elements {
        if lifted { out.push(lift(e)); continue }
        match e {
            // its far side's lower end, or the arc through its far point (its angle nought)
            Element::Line {a:p,b:q} if (p[0]-reach).abs() <= eps && (q[0]-reach).abs() <= eps => {
                out.push(Element::Line {a:p,b:[q[0],q[1]+s]});
                lifted = true;
            }
            Element::Arc {c,r,from,sweep} if (c[0]+r-reach).abs() <= eps && from <= 1e-12 && from+sweep >= -1e-12 => {
                if -from > 1e-12 { out.push(Element::Arc {c,r,from,sweep:-from}); }
                out.push(Element::Line {a:[c[0]+r,c[1]],b:[c[0]+r,c[1]+s]});
                if from+sweep > 1e-12 { out.push(Element::Arc {c:[c[0],c[1]+s],r,from:0.,sweep:from+sweep}); }
                lifted = true;
            }
            _ => {
                out.push(e);
                if (e.ends()[1][0]-reach).abs() <= eps {
                    let p = e.ends()[1];
                    out.push(Element::Line {a:p,b:[p[0],p[1]+s]});
                    lifted = true;
                }
            }
        }
    }
    if !lifted { return Err("the tool's meridian reaches farthest nowhere it is read".into()) }
    // (a flat end's rim drawn out runs on into its side: one side)
    let mut merged: Vec<Element> = Vec::with_capacity(out.len());
    for e in out {
        if let (Some(Element::Line {a:p,b:q}),Element::Line {a:q2,b:r}) = (merged.last().copied(),e) {
            if (p[0]-q[0]).abs() <= eps && (q2[0]-r[0]).abs() <= eps && (p[0]-r[0]).abs() <= eps { *merged.last_mut().unwrap() = Element::Line {a:p,b:r}; continue }
        }
        merged.push(e);
    }
    let out = merged;
    let n = out.len();
    let mut edges: Vec<ProfileEdge> = out.iter().map(|e| edge(e,o,u,a,false)).collect();
    edges.push(ProfileEdge::Line {a:at(o,u,a,out[n-1].ends()[1]),b:at(o,u,a,out[0].ends()[0])});
    revolve(&Profile {origin:o,normal:cross(u,a),loops:vec![edges],names:Vec::new()},o,a,TAU)
}

/// The foot of the axis where a slide along it leaves the tool lowest, and the axis.
fn lowest(rev: &Revolved,at_: Pose,s: f64) -> [V;2] {
    let (o,a) = (at_.point(rev.frame.o),at_.vector(rev.frame.z));
    [if s < 0. { add(o,scale(a,s)) } else { o },a]
}

/// The height of the tool's top above the foot of its axis and its reach there, where the tool
/// ends in a flat top as wide as it reaches anywhere, its side running straight up to it — the
/// shank of every end mill and drill: what a slide along the axis covers past the tool where it
/// stands lowest is then the column of that width over its top (`lift`). None for any other top.
fn flat_top(rev: &Revolved) -> Option<(f64,f64)> {
    let n = rev.elements.len();
    let reach = rev.elements.iter().flat_map(|e| e.ends()).map(|p| p[0]).fold(0.,f64::max);
    let eps = 1e-9*(1.+rev.size);
    if rev.elements.iter().any(|e| matches!(*e,Element::Arc {c,r,..} if c[0]+r > reach+eps)) { return None }
    match (n >= 2).then(|| (rev.elements[n-2],rev.elements[n-1]))? {
        (Element::Line {a:p,b:q},Element::Line {a:r,b:t}) if (p[0]-reach).abs() <= eps && (q[0]-reach).abs() <= eps
            && (r[1]-t[1]).abs() <= eps && t[0].abs() <= eps => Some((q[1],reach)),
        _ => None,
    }
}

/// **A slide along the axis less the tool where it stands lowest**: the column the tool's flat top
/// (`flat_top`) sweeps over the height `s`, standing on that top.
fn lift(rev: &Revolved,at_: Pose,s: f64) -> Result<Brep,String> {
    let (top,reach) = flat_top(rev).ok_or("a tool with no flat top as wide as it reaches")?;
    let [o,a] = lowest(rev,at_,s);
    let x = at_.vector(rev.frame.x);
    let c = add(o,scale(a,top));
    let base = Profile {origin:c,normal:a,names:Vec::new(),loops:vec![vec![ProfileEdge::Arc {frame:Frame::new(c,a,x),r:reach,span:None}]]};
    prism(&base,0.,s.abs())
}

/// Where a move square to the tool's axis starts or ends: the foot of the axis there (the
/// meridian's origin), the axis, and the unit way the axis moves.
type Stand = [V;3];

/// **A move square to the tool's axis, as a chain takes it** (`Move::Across`, or `Move::Turned`
/// short of a whole turn): where it starts and where it ends (`Stand`) — or none for any other move.
fn stands(rev: &Revolved,mv: Move) -> Option<[Stand;2]> {
    match mv {
        Move::Across {at,d} => {
            let (o,a,m) = (at.point(rev.frame.o),at.vector(rev.frame.z),unit(d));
            Some([[o,a,m],[add(o,d),a,m]])
        }
        Move::Turned {at,q,w,angle} if angle.abs() < TAU*(1.-1e-12) => {
            let (o,a) = (at.point(rev.frame.o),at.vector(rev.frame.z));
            let off = sub(o,q);
            let u = unit(sub(off,scale(w,dot(off,w))));
            let m = scale(cross(w,u),angle.signum());
            let end = crate::geom::Rigid::turn(q,w,angle);
            Some([[o,a,m],[end.point(o),end.vector(a),end.vector(m)]])
        }
        _ => None,
    }
}

/// **A move's side**: its section through the axis square to the way it moves, carried along a
/// slide or turned about a turn's line, without the section at either end.
fn side(rev: &Revolved,mv: Move,eps: f64) -> Result<Brep,String> {
    match mv {
        Move::Across {at,d} => {
            let (o,a,m) = (at.point(rev.frame.o),at.vector(rev.frame.z),unit(d));
            without_planes(&prism(&section(rev,o,cross(a,m),a,eps),0.,norm(d))?,m)
        }
        Move::Turned {at,q,w,angle} => {
            let o = at.point(rev.frame.o);
            let off = sub(o,q);
            let u = unit(sub(off,scale(w,dot(off,w))));
            without_planes_holding(&revolve(&section(rev,o,u,at.vector(rev.frame.z),eps),q,w,angle)?,w)
        }
        _ => Err("a move along the axis or at a slant has no side of this kind".into()),
    }
}

/// **A chain's sweep** (`moves`, each square to the axis, each starting as the one before ends,
/// the way the axis moves running on smoothly): its moves' sides sewn along the sections they
/// share, between the tool's half behind where the chain starts — or, not `back`, its section
/// there — and its half ahead where it ends. A smooth path's moves are swept as one: no two of
/// their sweeps meet in a Boolean, where each would touch the other along that section.
fn chain(rev: &Revolved,moves: &[Move],back: bool,tol: f64) -> Result<Brep,String> {
    let eps = 1e-9*(1.+rev.size);
    let ([o,a,m],_) = stands(rev,moves[0]).map(|[s,e]| (s,e)).ok_or("a chain of no move square to the axis")?;
    let [_,[o1,a1,m1]] = stands(rev,moves[moves.len()-1]).ok_or("a chain of no move square to the axis")?;
    // (a chain closing on itself, ending where it starts the same way, is its sides alone: a ring)
    let closed = moves.len() > 1 && norm(sub(o1,o)) <= eps && dot(a,a1) >= 1.-1e-12 && dot(m,m1) >= 1.-1e-9;
    let mut faces = Vec::with_capacity(moves.len()+2);
    if !closed { faces.push(if back { half(rev,o,a,scale(m,-1.))? } else { crate::build::face(&section(rev,o,cross(a,m),a,eps))? }); }
    for &mv in moves { faces.push(side(rev,mv,eps)?); }
    if !closed { faces.push(half(rev,o1,a1,m1)?); }
    crate::sew::sewn(&faces,tol).map_err(|e| e.msg)
}

/// **The tool turned a whole turn or more about a line along its axis**: its section through both
/// axes' solid of revolution.
fn ring(rev: &Revolved,at_: Pose,q: V,w: V,angle: f64) -> Result<Brep,String> {
    let o = at_.point(rev.frame.o);
    let off = sub(o,q);
    let u = unit(sub(off,scale(w,dot(off,w))));
    revolve(&section(rev,o,u,at_.vector(rev.frame.z),1e-9*(1.+rev.size)),q,w,angle)
}

/// What `piece` over `[lo, hi]` sweeps, read: its move (`Move`), or none where it moves nothing.
fn read(rev: &Revolved,mill: Option<&super::raster::Mill>,path: &Trajectory,[lo,hi]: [f64;2]) -> Option<Move> {
    let closed = Some(Move::Closed {roll:[lo,hi]});
    let (Trajectory::Twist(s) | Trajectory::Placed {twist:s,..}) = path else { return closed };
    let (Ok(p),Ok(q)) = (path.pose(lo),path.pose(hi)) else { return closed };
    let a = unit(p.vector(rev.frame.z));
    if s.is_slide() {
        let d = sub(q.p,p.p);
        let l = norm(d);
        if !(l > 1e-12*(1.+rev.size)) { return None }
        let up = dot(d,a);
        let flat = sub(d,scale(a,up));
        if norm(flat) <= 1e-12*l { return Some(Move::Along {at:p,s:up}) }
        if up.abs() <= 1e-12*l { return Some(Move::Across {at:p,d}) }
        return match mill {
            Some(m) => Some(Move::Slanted {from:p.point(m.c),to:q.point(m.c),axis:unit(p.vector(m.axis))}),
            None => closed,
        }
    }
    if s.is_turn() {
        let w = unit(s.axis);
        let angle = s.ratio*(hi-lo);
        if angle == 0. { return None }
        let o = p.point(rev.frame.o);
        let off = sub(o,s.origin);
        let apart = norm(sub(off,scale(w,dot(off,w))));
        let reach = rev.elements.iter().flat_map(|e| e.ends()).map(|x| x[0]).fold(0.,f64::max);
        let reach = rev.elements.iter().fold(reach,|r,e| match *e { Element::Arc {c,r:rr,..} => r.max(c[0]+rr),_ => r });
        if norm(cross(w,a)) <= 1e-12 && apart > reach*(1.+1e-9) { return Some(Move::Turned {at:p,q:s.origin,w,angle}) }
    }
    closed
}

/// The box a piece's sweep stands in, by its tool's box at poses along it: at its two ends under a
/// slide; at every 64th of the piece and grown by the chord's sag under any other motion.
fn reach(tool: &Brep,path: &Trajectory,[lo,hi]: [f64;2]) -> Option<crate::geom::Box3> {
    let corners = super::closed::corners_of(tool.bounds());
    let slide = matches!(path,Trajectory::Twist(s) | Trajectory::Placed {twist:s,..} if s.is_slide());
    let steps = if slide { 1 } else { 64 };
    let mut pts = Vec::with_capacity(8*(steps+1));
    for k in 0..=steps {
        let pose = path.pose(lo+(hi-lo)*k as f64/steps as f64).ok()?;
        pts.extend(corners.iter().map(|&c| pose.point(c)));
    }
    let (mut l,mut h) = crate::geom::hull(pts);
    if !slide {
        // a step's chord against the arc it cuts: the farthest a corner runs, by the step's turn
        let size = tool.size();
        let far = size+norm(sub(path.pose(lo).ok()?.p,path.pose(hi).ok()?.p));
        let pad = far*(1./steps as f64)*(hi-lo).abs().min(TAU);
        for k in 0..3 { l[k] -= pad; h[k] += pad; }
    }
    Some((l,h))
}

/// **`blank` less what `tool` sweeps along `path` over `roll`, piece by piece** (the module's
/// account), judged as the closed class judges (`closed::judge`, the tool meshed to `bar`) — or
/// `None` where the road does not take it: no path of pieces, a tool no convex solid of revolution,
/// or a piece it could not sweep or cut.
pub fn cut(blank: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Option<Result<Brep,Unbuilt>> {
    let Trajectory::Pieces(pieces) = path else { return None };
    let rev = match Revolved::read(tool) {
        Ok(rev) => rev,
        Err(why) => { if crate::debug() { eprintln!("pieces: the tool read as no convex solid of revolution: {why:?}") } return None }
    };
    let clock = crate::util::clock::Instant::now();
    let mill = super::raster::Mill::read(tool,tol);
    let bounds = blank.bounds();
    if !bounds.0[0].is_finite() { return None }
    let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    // each piece's stretch of the roll, its move, those that reach the blank; each but the first
    // follows one that holds the tool where it starts, so needs no half behind there (`back`) —
    // that piece's own sweep holds it, or (left out) it stands clear of the blank
    let first = pieces.pieces().iter().position(|p| p.over[1].min(b) > p.over[0].max(a));
    let moves: Vec<(usize,&Trajectory,Move,bool)> = pieces.pieces().iter().enumerate().filter_map(|(i,piece)| {
        let (lo,hi) = (piece.over[0].max(a),piece.over[1].min(b));
        if !(hi > lo) { return None }
        let box_ = reach(tool,&piece.path,[lo,hi])?;
        if !overlap(&box_,&bounds,tol) { return None }
        Some((i,&piece.path,read(&rev,mill.as_ref(),&piece.path,[lo,hi])?,Some(i) == first))
    }).collect();
    let eps = 1e-9*(1.+rev.size);
    // slides along one axis line, one after another (a drill's pecks), are one slide over every
    // place they stand: the tool drawn out from the lowest to the highest
    let mut moves = moves;
    {
        let mut merged: Vec<(usize,&Trajectory,Move,bool)> = Vec::with_capacity(moves.len());
        for m in moves {
            if let (Some(last),(i,_,Move::Along {at,s},_)) = (merged.last_mut(),m) {
                if let (j,_,Move::Along {at:at0,s:s0},_) = *last {
                    let ([o0,a0],[o1,a1]) = ([at0.point(rev.frame.o),at0.vector(rev.frame.z)],[at.point(rev.frame.o),at.vector(rev.frame.z)]);
                    let off = sub(o1,o0);
                    if j+1 == i && dot(a0,a1) >= 1.-1e-12 && norm(sub(off,scale(a0,dot(off,a0)))) <= eps {
                        // each slide's span along the axis from `o0`, and their union
                        let (h1,h2) = (dot(off,a0),dot(off,a0)+s);
                        let (lo,hi) = (0f64.min(s0).min(h1).min(h2),0f64.max(s0).max(h1).max(h2));
                        let at_lo = Pose {r:at0.r,p:add(at0.p,scale(a0,lo))};
                        // (known by the last piece's place in the path, which a chain after it follows)
                        *last = (i,last.1,Move::Along {at:at_lo,s:hi-lo},last.3);
                        continue
                    }
                }
            }
            merged.push(m);
        }
        moves = merged;
    }
    // the moves in chains: one square to the axis goes on with the chain before it where it is the
    // next piece of the path and starts as that one ends, the axis moving on the same way
    let mut chains: Vec<Vec<(usize,&Trajectory,Move,bool)>> = Vec::new();
    // (a ball-end mill's slides at a slant chain where each starts where the last ends, the axis
    // the same, the way across it the same: a run in one plane holding the axis, its corners laid
    // by `raster`'s construction while it can lay them)
    let across = |from: V,to: V,a: V| { let d = sub(to,from); unit(sub(d,scale(a,dot(d,a)))) };
    for &(i,piece,mv,back) in &moves {
        let goes_on = chains.last().is_some_and(|c| {
            let &(j,_,last,_) = c.last().unwrap();
            j+1 == i && match (last,mv) {
                (Move::Slanted {from:f0,to:t0,axis:a0},Move::Slanted {from,to,axis}) => {
                    let mill = mill.as_ref().expect("a slanted move is read only of a mill");
                    let mut cs: Vec<V> = c.iter().filter_map(|x| match x.2 { Move::Slanted {from,..} => Some(from),_ => None }).collect();
                    cs.extend([t0,to]);
                    norm(sub(t0,from)) <= eps && dot(a0,axis) >= 1.-1e-12 && dot(across(f0,t0,a0),across(from,to,axis)) >= 1.-1e-12
                        && super::raster::laid(&cs,across(from,to,axis),axis,mill.r,tol)
                }
                _ => match (stands(&rev,last),stands(&rev,mv)) {
                    (Some([_,[o,a,m]]),Some([[p,b,n],_])) => norm(sub(o,p)) <= eps && dot(a,b) >= 1.-1e-12 && dot(m,n) >= 1.-1e-9,
                    _ => false,
                },
            }
        });
        if goes_on { chains.last_mut().unwrap().push((i,piece,mv,back)) } else { chains.push(vec![(i,piece,mv,back)]) }
    }
    // a slide along the axis ending where a chain starts (a plunge) or starting where one ends (a
    // lift out), the tool standing lowest there: that chain holds the tool there — its half behind
    // where it starts, `back` — so the slide is the column over the tool's top alone (`lift`), its
    // tool never met with the chain's sides, which touch it round their section
    if flat_top(&rev).is_some() {
        for k in 0..chains.len() {
            let [(i,piece,Move::Along {at,s},back)] = chains[k][..] else { continue };
            let [low,_] = lowest(&rev,at,s);
            // (where a chain starts or ends, or a whole turn about a line along the axis, whose ring
            // holds the tool where it starts and ends)
            let foot = |c: &[(usize,&Trajectory,Move,bool)],end: usize| -> Option<(usize,V)> {
                let (j,_,mv,_) = if end == 0 { c[0] } else { c[c.len()-1] };
                match mv {
                    Move::Turned {at,..} if c.len() == 1 && stands(&rev,mv).is_none() => Some((j,at.point(rev.frame.o))),
                    _ => stands(&rev,mv).map(|st| (j,st[end][0])),
                }
            };
            let plunged = s < 0. && chains.get(k+1).and_then(|c| foot(c,0)).is_some_and(|(j,o)| j == i+1 && norm(sub(o,low)) <= eps);
            let lifted = s > 0. && k > 0 && foot(&chains[k-1],1).is_some_and(|(j,o)| j+1 == i && norm(sub(o,low)) <= eps);
            if plunged { chains[k+1][0].3 = true }
            if plunged || lifted { chains[k] = vec![(i,piece,Move::Lifted {at,s},back)] }
        }
    }
    let sweeps = crate::util::par::map(&chains,|c| {
        let (i,piece,mv,back) = c[0];
        match (c.len(),mv) {
            (_,Move::Slanted {axis,..}) => {
                let mut cs: Vec<V> = c.iter().filter_map(|x| match x.2 { Move::Slanted {from,..} => Some(from),_ => None }).collect();
                if let Move::Slanted {to,..} = c[c.len()-1].2 { cs.push(to) }
                super::raster::run(&cs,axis,mill.as_ref().expect("a slanted move is read only of a mill"),blank,back,tol)
            }
            (1,_) => sweep(&rev,mill.as_ref(),tool,piece,mv,back,blank,bar,tol),
            _ => chain(&rev,&c.iter().map(|x| x.2).collect::<Vec<_>>(),back,tol),
        }.map_err(|why| Unbuilt::Failed(format!("piece {i} of the path: its sweep not built: {why}")))
    });
    let swept_at = clock.elapsed().as_secs_f64();
    let (mut out,mut known): (Option<Brep>,Option<Known>) = (None,None);
    for ((i,..),sweep) in chains.iter().map(|c| c[0]).zip(sweeps) {
        let sweep = match sweep { Ok(s) => s,Err(e) => return fallen(e) };
        match combined_known(out.as_ref().unwrap_or(blank),known.as_ref(),&sweep,Op::Cut,tol) {
            Ok((cut,next)) => (out,known) = (Some(cut),Some(next)),
            Err(why) => {
                // (the two operands written where `SOLVENT_SWEEP_DUMP` asks, to be replayed alone)
                if let Some(dir) = super::dump::dump_dir() {
                    let wrote = std::fs::create_dir_all(&dir).map_err(|e| e.to_string()).and_then(|()| {
                        for (name,b) in [("blank",out.as_ref().unwrap_or(blank)),("tool",&sweep)] {
                            std::fs::write(dir.join(format!("{name}.brep")),super::dump::write(b)?).map_err(|e| e.to_string())?;
                        }
                        Ok(())
                    });
                    if let Err(e) = wrote { eprintln!("pieces: the failed cut's operands not written: {e}") }
                }
                return fallen(Unbuilt::Failed(format!("piece {i} of the path: not cut: {}",why.msg)))
            }
        }
    }
    let out = out.unwrap_or_else(|| blank.clone());
    let cut_at = clock.elapsed().as_secs_f64();
    let judged = judged(out,blank,tool,path,roll,bar,tol);
    if crate::timing() {
        eprintln!("pieces: {} of {} pieces reach the blank, in {} sweeps: swept by {swept_at:.2} s, cut by {cut_at:.2} s, judged by {:.2} s",
            moves.len(),pieces.pieces().len(),chains.len(),clock.elapsed().as_secs_f64());
    }
    Some(judged)
}

/// The road's failure, said where the kernel narrates, the path left to the closed class.
fn fallen(why: Unbuilt) -> Option<Result<Brep,Unbuilt>> {
    if crate::debug() { eprintln!("pieces: {why:?}; left to the closed class") }
    None
}

/// `out` judged as the closed class judges its builds.
fn judged(out: Brep,blank: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    let solid = super::member::Solid::new(tool,bar,1.).map_err(Unbuilt::Failed)?;
    let swept = super::member::Swept {tool:&solid,path,roll};
    super::closed::judge(&out,&swept,blank,Some(blank),&[],tol)?;
    Ok(out)
}

/// What one piece sweeps, built by its move's road.
#[allow(clippy::too_many_arguments)]
fn sweep(rev: &Revolved,mill: Option<&super::raster::Mill>,tool: &Brep,piece: &Trajectory,mv: Move,back: bool,blank: &Brep,bar: f64,tol: f64)
    -> Result<Brep,String> {
    let eps = 1e-9*(1.+rev.size);
    match mv {
        Move::Along {at,s} => along(rev,at,s,eps),
        Move::Lifted {at,s} => lift(rev,at,s),
        Move::Across {..} => chain(rev,&[mv],back,tol),
        Move::Turned {at,q,w,angle} if angle.abs() >= TAU*(1.-1e-12) => ring(rev,at,q,w,angle),
        Move::Turned {..} => chain(rev,&[mv],back,tol),
        Move::Slanted {from,to,axis} => super::raster::run(&[from,to],axis,mill.expect("a slanted move is read only of a mill"),blank,back,tol),
        Move::Closed {roll} => super::closed::closed(tool,piece,roll,bar,tol).map_err(|e| format!("{e:?}")),
    }
}
