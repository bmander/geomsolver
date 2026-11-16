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

/// **The tool slid square to its axis by `d`** from the pose `at_`: its half behind, its section
/// through the axis square to `d` carried along it, and its half ahead where it ends — sewn.
fn across(rev: &Revolved,at_: Pose,d: V,back: bool,tol: f64) -> Result<Brep,String> {
    let (o,a) = (at_.point(rev.frame.o),at_.vector(rev.frame.z));
    let m = unit(d);
    let u = cross(a,m);
    let section = section(rev,o,u,a,1e-9*(1.+rev.size));
    let carried = prism(&section,0.,norm(d))?;
    let start = if back { half(rev,o,a,scale(m,-1.))? } else { crate::build::face(&section)? };
    let faces = [start,without_planes(&carried,m)?,half(rev,add(o,d),a,m)?];
    if crate::debug() {
        for (k,b) in faces.iter().enumerate() {
            for f in &b.faces {
                let es: Vec<String> = f.loops.iter().flatten().map(|c| { let e = &b.edges[c.edge as usize];
                    format!("{}{:?}->{:?}",e.curve.kind(),b.vertices[e.v[0] as usize].p.map(|x| (x*1e3).round()/1e3),b.vertices[e.v[1] as usize].p.map(|x| (x*1e3).round()/1e3)) }).collect();
                eprintln!("across piece {k}: {} {:?}",f.surface.kind(),es);
            }
        }
    }
    crate::sew::sewn(&faces,tol).map_err(|e| e.msg)
}

/// **The tool turned by `angle` about the line through `q` along `w`** (along its axis, missing
/// it) from the pose `at_`: its section through both axes turned about the line, between its half
/// behind where it starts and its half ahead where it ends — or, a whole turn or more, that
/// section's solid of revolution alone.
fn turned(rev: &Revolved,at_: Pose,q: V,w: V,angle: f64,back: bool,tol: f64) -> Result<Brep,String> {
    let (o,a) = (at_.point(rev.frame.o),at_.vector(rev.frame.z));
    let off = sub(o,q);
    let u = unit(sub(off,scale(w,dot(off,w))));
    let section = section(rev,o,u,a,1e-9*(1.+rev.size));
    let ring = revolve(&section,q,w,angle)?;
    if angle.abs() >= TAU*(1.-1e-12) { return Ok(ring) }
    // the axis's way at the start, and where the turn leaves it
    let m = scale(cross(w,u),angle.signum());
    let end = crate::geom::Rigid::turn(q,w,angle);
    let (o1,m1) = (end.point(o),end.vector(m));
    let start = if back { half(rev,o,a,scale(m,-1.))? } else { crate::build::face(&section)? };
    let faces = [start,without_planes_holding(&ring,w)?,half(rev,o1,end.vector(a),m1)?];
    crate::sew::sewn(&faces,tol).map_err(|e| e.msg)
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
    let rev = Revolved::read(tool).ok()?;
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
    let sweeps = crate::util::par::map(&moves,|&(i,piece,mv,back)| sweep(&rev,mill.as_ref(),tool,piece,mv,back,blank,bar,tol)
        .map_err(|why| Unbuilt::Failed(format!("piece {i} of the path: its sweep not built: {why}"))));
    let (mut out,mut known): (Option<Brep>,Option<Known>) = (None,None);
    for ((i,..),sweep) in moves.iter().zip(sweeps) {
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
    Some(judged(out,blank,tool,path,roll,bar,tol))
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
        Move::Across {at,d} => across(rev,at,d,back,tol),
        Move::Turned {at,q,w,angle} => turned(rev,at,q,w,angle,back,tol),
        Move::Slanted {from,to,axis} => super::raster::slanted(from,to,axis,mill.expect("a slanted move is read only of a mill"),blank,back,tol),
        Move::Closed {roll} => super::closed::closed(tool,piece,roll,bar,tol).map_err(|e| format!("{e:?}")),
    }
}
