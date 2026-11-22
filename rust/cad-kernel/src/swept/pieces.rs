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
//! Moves that join — a smooth chain (`chain`), slides square to the axis turning at corners the
//! run can mitre (`mitred`) — are one sweep, so no two of their sweeps touch in a Boolean round
//! the section where they join.
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

/// **A chain's run so far** (`cut`), kept as it grows: whether all its moves are slides square to
/// the axis, whether it turns at a sharp corner (so is `mitred`), its turn so far and the least and
/// most it has reached, and the turn at its last corner.
#[derive(Clone,Copy,Default)]
struct Run { across: bool,cornered: bool,at: f64,lo: f64,hi: f64,before: f64 }

/// The tool's meridian point `(ρ, z)` in the plane through `o` spanned by `u` (across) and `a`
/// (along the axis).
fn at(o: V,u: V,a: V,p: P2) -> V { add(o,add(scale(u,p[0]),scale(a,p[1]))) }

/// An element of the meridian drawn in that plane, mirrored across the axis where `mirror`.
fn edge(e: &Element,o: V,u: V,a: V,mirror: bool) -> ProfileEdge {
    let u = if mirror { scale(u,-1.) } else { u };
    match *e {
        Element::Line {a:p,b:q} => ProfileEdge::Line {a:at(o,u,a,p),b:at(o,u,a,q)},
        Element::Arc {c,r,from,sweep} => arc(at(o,u,a,c),u,a,r,[from,from+sweep]),
    }
}

/// An arc of radius `r` about `o`, in the plane of the unit `x` and `y` (square to one another),
/// over the angles `span` from `x` toward `y` — its frame turned so its circle's seam stands
/// opposite the arc's middle: no surface a section's arc is carried or turned into has its seam
/// along an edge, which two faces would read on either side of the seam.
fn arc(o: V,x: V,y: V,r: f64,[s0,s1]: [f64;2]) -> ProfileEdge {
    let turn = 0.5*(s0+s1)+PI;
    let (sn,cs) = turn.sin_cos();
    let (x2,y2) = (add(scale(x,cs),scale(y,sn)),add(scale(x,-sn),scale(y,cs)));
    ProfileEdge::Arc {frame:Frame {o,x:x2,y:y2,z:cross(x2,y2)},r,span:Some([s0-turn,s1-turn])}
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
            edges.push(arc(at(o,u,a,c),u,a,r,span));
        } else {
            edges.push(edge(e,o,u,a,false));
            edges.push(edge(e,o,u,a,true));
        }
    }
    Profile {origin:o,normal:cross(u,a),loops:vec![edges],names:Vec::new()}
}

/// **A sweep's end where the tool stands at `o`, the sweep beyond it along `m`**: the tool's half
/// there on the side `m` points to — its meridian in the plane through `o` holding the axis `a`
/// square to `m`, turned half a turn about the axis toward `m` (`wedge`) — or, not `whole` (a sweep
/// before it holds that half), the tool's section there.
fn cap(rev: &Revolved,o: V,a: V,m: V,whole: bool,eps: f64) -> Result<Vec<Brep>,String> {
    if whole { wedge(rev,o,cross(m,a),a,PI) } else { Ok(vec![crate::build::face(&section(rev,o,cross(a,m),a,eps))?]) }
}

/// A slide square to the axis read as a leg of a run: where it starts, its unit way and its length.
fn leg(rev: &Revolved,mv: Move) -> Option<(V,V,f64)> {
    match mv { Move::Across {at,d} => Some((at.point(rev.frame.o),unit(d),norm(d))),_ => None }
}

/// Whether a chain from `first` to `last` (moves square to the axis, more than one) closes on
/// itself: ending where it starts, the axis and the way it moves the same — its sides alone, a ring.
fn closes(rev: &Revolved,first: Move,last: Move,eps: f64) -> bool {
    match (stands(rev,first),stands(rev,last)) {
        (Some([[o,a,m],_]),Some([_,[o1,a1,m1]])) => norm(sub(o1,o)) <= eps && dot(a,a1) >= 1.-1e-12 && dot(m,m1) >= 1.-1e-9,
        _ => false,
    }
}

/// **The tool's wedge turned from `u` by `phi` about its axis `a`** (right-handed, through `o`):
/// its meridian on the side of `u` turned so — its faces, without the half sections it is shut by
/// at either end. A ball's quarter in it (its meridian's arc from the axis to the equator) is the
/// ball's part on the wedge's side of both sections and of the equator, its poles off all three
/// planes (`raster::lune`): turned about the axis, its pole would stand on both sections, where a
/// move after a corner touches the wedge along a great circle through it — a meeting through a
/// pole, which the Booleans do not split.
fn wedge(rev: &Revolved,o: V,u: V,a: V,phi: f64) -> Result<Vec<Brep>,String> {
    let quarters: Vec<(V,f64,V)> = rev.elements.iter().filter_map(|e| match *e {
        Element::Arc {c,r,from,sweep} if c[0].abs() <= 1e-9*(1.+rev.size) => {
            let [s0,s1] = [from.min(from+sweep),from.max(from+sweep)];
            let quarter = |x: f64,y: f64| (x-PI/2.).abs() <= 1e-9 && y.abs() <= 1e-9;
            // (below the equator for the bottom's quarter, above it for the top's)
            if quarter(-s0,s1) { Some((at(o,u,a,c),r,scale(a,-1.))) } else if quarter(s1,s0) { Some((at(o,u,a,c),r,a)) } else { None }
        }
        _ => None,
    }).collect();
    // the turned meridian, without its sections — nor its quarters' spheres, built apart below
    let turned = faces_but(&revolve(&half_section(rev,o,u,a,cross(u,a)),o,a,phi)?,
        |s| holding(s,a) || (!quarters.is_empty() && matches!(s,Surface::Sphere(..))))?;
    if quarters.is_empty() { return Ok(vec![turned]) }
    // the sections' sides the wedge lies on: from `u`'s plane toward where it turns, and back from
    // the plane it ends in (one plane, a half turn on)
    let ahead = scale(cross(a,u),phi.signum());
    let end = crate::geom::Rigid::turn(o,a,phi).vector(u);
    let behind = scale(cross(a,end),-phi.signum());
    let mut cuts = vec![ahead];
    if norm(sub(behind,ahead)) > 1e-9 { cuts.push(behind) }
    let mut out = vec![turned];
    for (c,r,w) in quarters {
        let mut planes = vec![w];
        planes.extend(&cuts);
        out.push(super::raster::lune(c,r,&planes,[ahead,u,a])?);
    }
    Ok(out)
}

/// **The tool's meridian on the side `u` of its axis**, in the plane through `o` holding the axis
/// `a`, shut along the axis: a half section, its plane facing `n` (`u × a` or against it).
fn half_section(rev: &Revolved,o: V,u: V,a: V,n: V) -> Profile {
    let k = rev.elements.len();
    let (bottom,top) = (rev.elements[0].ends()[0],rev.elements[k-1].ends()[1]);
    let mut edges: Vec<ProfileEdge> = rev.elements.iter().map(|e| edge(e,o,u,a,false)).collect();
    edges.push(ProfileEdge::Line {a:at(o,u,a,top),b:at(o,u,a,bottom)});
    Profile {origin:o,normal:n,loops:vec![edges],names:Vec::new()}
}

/// **A half section carried along its plane's normal between two planes through the axis**
/// (`ends`, each a point and a unit normal): the sheet its meridian's edges sweep, from where each
/// meets the plane it starts at to where it meets the one it ends at — a side cut square at a
/// section, or slanted at a mitre — its edge along the axis (through `o` along `a`) sweeping
/// nothing (the plane two halves share). A segment's side is a plane strip, an arc's a cylinder's,
/// each cut at a slanted end along a line or an ellipse.
fn strip(p: &Profile,o: V,a: V,ends: [[V;2];2],eps: f64) -> Result<Brep,String> {
    use crate::geom::Curve;
    use crate::topo::{Coedge,EdgeCurve,Face,Pcurve};
    let n = unit(p.normal);
    let plane = Frame::about(p.origin,n);
    let walks = crate::build::walks(p,&|q| { let l = plane.local(q); [l[0],l[1]] })?;
    let walk: Vec<crate::build::Seg> = walks.into_iter().next().unwrap_or_default().into_iter().map(|s| s.facing(n)).collect();
    let m = walk.len();
    // how far along `n` a point of the section's plane is carried to meet an end
    let height = |q: V,[c,k]: [V;2]| dot(sub(c,q),k)/dot(n,k);
    let hs: Vec<[f64;2]> = walk.iter().map(|s| ends.map(|e| height(s.start(),e))).collect();
    if hs.iter().any(|h| !(h[1]-h[0] > eps)) { return Err("a strip's ends cross".into()) }
    let lift = |q: V,h: f64| add(q,scale(n,h));
    let on_axis = |q: V| norm(cross(sub(q,o),a)) <= eps;
    let mut b = Brep::default();
    let bv: Vec<u32> = (0..m).map(|k| b.vertex(lift(walk[k].start(),hs[k][0]))).collect();
    let tv: Vec<u32> = (0..m).map(|k| b.vertex(lift(walk[k].start(),hs[k][1]))).collect();
    let up: Vec<u32> = (0..m).map(|k| b.edge(EdgeCurve::Curve(Curve::Line {p:walk[k].start(),d:n}),hs[k],[bv[k],tv[k]])).collect();
    for k in 0..m {
        let s = &walk[k];
        let k1 = (k+1)%m;
        if on_axis(s.start()) && on_axis(s.end()) { continue }
        // a step's edge at end `e`, from vertex `v0` to `v1` along the walk: its index, and whether
        // it is stored running the walk's way
        let rim = |e: usize,[v0,v1]: [u32;2],b: &mut Brep| -> Result<(u32,bool),String> {
            let [ha,hb] = [hs[k][e],hs[k1][e]];
            match s.curve {
                Curve::Line {..} => {
                    let (p0,p1) = (lift(s.start(),ha),lift(s.end(),hb));
                    Ok((b.edge(EdgeCurve::Curve(Curve::Line {p:p0,d:unit(sub(p1,p0))}),[0.,norm(sub(p1,p0))],[v0,v1]),true))
                }
                Curve::Circle(f,r) => {
                    // the circle carried onto the end's plane — itself, moved, at a square end;
                    // at a slanted one `c + cos t X + sin t Y`, read in its principal axes
                    let k_ = ends[e][1];
                    let c = lift(f.o,height(f.o,ends[e]));
                    let (curve,t0) = if dot(n,k_) >= 1.-1e-12 { (Curve::Circle(Frame {o:c,..f},r),0.) } else {
                        let g = scale(k_,-1./dot(n,k_));
                        let (x,y) = (scale(add(f.x,scale(n,dot(f.x,g))),r),scale(add(f.y,scale(n,dot(f.y,g))),r));
                        let t0 = 0.5*(2.*dot(x,y)).atan2(dot(x,x)-dot(y,y));
                        let (sn,cs) = t0.sin_cos();
                        let (xp,yp) = (add(scale(x,cs),scale(y,sn)),add(scale(x,-sn),scale(y,cs)));
                        (Curve::Ellipse(Frame {o:c,x:unit(xp),y:unit(yp),z:unit(cross(xp,yp))},norm(xp),norm(yp)),t0)
                    };
                    let [ta,tb] = s.span();
                    Ok((b.edge(EdgeCurve::Curve(curve),[ta-t0,tb-t0],if s.forward() { [v0,v1] } else { [v1,v0] }),s.forward()))
                }
                _ => Err(format!("a {} in a tool's section is not carried",s.curve.kind())),
            }
        };
        let (bottom,b_along) = rim(0,[bv[k],bv[k1]],&mut b)?;
        let (top,t_along) = rim(1,[tv[k],tv[k1]],&mut b)?;
        let (surface,reversed,curved) = match s.curve {
            Curve::Line {..} => {
                let d = unit(sub(s.end(),s.start()));
                (Surface::Plane(Frame::new(s.start(),cross(d,n),d)),false,false)
            }
            Curve::Circle(f,r) => (Surface::Cylinder(f,r),!s.forward(),true),
            _ => unreachable!("refused above"),
        };
        // a cylinder's pcurves its parameters (angle, height) at each edge's first and last; a
        // plane's, its inverse
        let pc = |uv: [[f64;2];2]| if curved { Pcurve::Inverse {a:uv[0],b:uv[1]} } else { Pcurve::Inverse {a:[0.;2],b:[0.;2]} };
        let rim_uv = |e: usize| { let w = [[s.t[0],hs[k][e]],[s.t[1],hs[k1][e]]]; pc(if s.forward() { w } else { [w[1],w[0]] }) };
        let (b_uv,t_uv) = (rim_uv(0),rim_uv(1));
        let t_at = |j: usize| if j == k { s.t[0] } else { s.t[1] };
        let ups = |j: usize| pc([[t_at(j),hs[j][0]],[t_at(j),hs[j][1]]]);
        let side = vec![
            Coedge {edge:bottom,reversed:!b_along,pcurve:b_uv},
            Coedge {edge:up[k1],reversed:false,pcurve:ups(k1)},
            Coedge {edge:top,reversed:t_along,pcurve:t_uv},
            Coedge {edge:up[k],reversed:true,pcurve:ups(k)},
        ];
        b.faces.push(Face {surface,reversed,loops:vec![side],name:String::new()});
    }
    Ok(b)
}

/// The turn about `a` (right-handed) from the way `d0` to the way `d1`, both square to it.
fn turn(a: V,d0: V,d1: V) -> f64 { dot(cross(d0,d1),a).atan2(dot(d0,d1)) }

/// The farthest the tool reaches from its axis.
fn reach_of(rev: &Revolved) -> f64 {
    let ends = rev.elements.iter().flat_map(|e| e.ends()).map(|p| p[0]).fold(0.,f64::max);
    rev.elements.iter().fold(ends,|r,e| match *e { Element::Arc {c,r:rr,..} => r.max(c[0]+rr),_ => r })
}

/// **A run of slides square to the axis turning at sharp corners** (`moves`, each starting where the
/// last ends, the axis the same): each slide's two halves carried along it, each cut where it
/// starts and ends square at the section — or, on the side a corner turns toward, slanted at the
/// plane through the axis bisecting the corner (a mitre), where the slide's half before it and
/// after it meet in one curve; at each corner the tool's wedge on the side it turns away from,
/// turned through the corner about the axis; between the tool's half behind where the run starts
/// (or, not `back`, its section) and its half ahead where it ends (or, not `front`, its section),
/// sewn. A run's sweeps are never met in a Boolean, where each would touch the next round the
/// section they share at the corner. (`cut` runs slides on only while the run is laid so: each
/// slide longer than its mitres take back on either side, and the run turning through no more than
/// a right angle, so no two of its slides meet but where they join.)
fn mitred(rev: &Revolved,moves: &[Move],back: bool,front: bool,tol: f64) -> Result<Brep,String> {
    let eps = 1e-9*(1.+rev.size);
    let legs: Vec<(V,V,f64)> = moves.iter().map(|&mv| leg(rev,mv)).collect::<Option<_>>().ok_or("a mitred run of a move not square to the axis")?;
    let Move::Across {at,..} = moves[0] else { unreachable!("read as a leg") };
    let (n,a) = (legs.len(),at.vector(rev.frame.z));
    // the turn where each slide starts and ends (none at the run's ends)
    let turns: Vec<f64> = (0..=n).map(|i| if i == 0 || i == n { 0. } else { turn(a,legs[i-1].1,legs[i].1) }).collect();
    // each slide's halves and the wedge at the corner it starts at, apart from every other's
    let pieces = crate::util::par::indices(n,|i| -> Result<Vec<Brep>,String> {
        let (o,d,len) = legs[i];
        let e = add(o,scale(d,len));
        let left = cross(a,d);
        let mut out = Vec::with_capacity(3);
        for (u,side) in [(left,1.),(scale(left,-1.),-1.)] {
            // (a corner turning toward this side takes it back to the mitre)
            let toward = |phi: f64| side*phi > 1e-12;
            let start = if toward(turns[i]) { [o,unit(add(legs[i-1].1,d))] } else { [o,d] };
            let end = if toward(turns[i+1]) { [e,unit(add(d,legs[i+1].1))] } else { [e,d] };
            out.push(strip(&half_section(rev,o,u,a,d),o,a,[start,end],eps)?);
        }
        if turns[i].abs() > 1e-12 {
            // the wedge on the side turned from: right of the slide before for a turn left
            let dp = legs[i-1].1;
            out.extend(wedge(rev,o,if turns[i] > 0. { cross(dp,a) } else { cross(a,dp) },a,turns[i])?);
        }
        Ok(out)
    });
    let mut faces: Vec<Brep> = Vec::with_capacity(3*n+2);
    for p in pieces { faces.extend(p?) }
    let ((o0,d0,_),(on,dn,ln)) = (legs[0],legs[n-1]);
    faces.extend(cap(rev,o0,a,scale(d0,-1.),back,eps)?);
    faces.extend(cap(rev,add(on,scale(dn,ln)),a,dn,front,eps)?);
    crate::sew::sewn(&faces,tol).map_err(|e| e.msg)
}

/// `b`'s faces but those on a plane square to `m`: what is left of a section carried along `m`
/// without the section at either end.
fn without_planes(b: &Brep,m: V) -> Result<Brep,String> { faces_but(b,|s| matches!(s,Surface::Plane(f) if norm(cross(unit(f.z),m)) <= 1e-9)) }

/// Whether a surface is a plane holding the line along `w`: a section turned about `w`.
fn holding(s: &Surface,w: V) -> bool { matches!(s,Surface::Plane(f) if dot(unit(f.z),w).abs() <= 1e-9) }

/// `b`'s faces but those on a plane holding the line along `w`: what is left of a section turned
/// about `w` without the section at either end.
fn without_planes_holding(b: &Brep,w: V) -> Result<Brep,String> { faces_but(b,|s| holding(s,w)) }

/// `b`'s faces but those on a surface `drop` takes.
fn faces_but(b: &Brep,drop: impl Fn(&Surface) -> bool) -> Result<Brep,String> {
    let keep: Vec<usize> = (0..b.faces.len()).filter(|&k| !drop(&b.faces[k].surface)).collect();
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
    let reach = reach_of(rev);
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
/// there — and its half ahead where it ends — or, not `front`, its section there. A smooth path's
/// moves are swept as one: no two of their sweeps meet in a Boolean, where each would touch the
/// other along that section.
fn chain(rev: &Revolved,moves: &[Move],back: bool,front: bool,tol: f64) -> Result<Brep,String> {
    let eps = 1e-9*(1.+rev.size);
    let ([o,a,m],_) = stands(rev,moves[0]).map(|[s,e]| (s,e)).ok_or("a chain of no move square to the axis")?;
    let [_,[o1,a1,m1]] = stands(rev,moves[moves.len()-1]).ok_or("a chain of no move square to the axis")?;
    // (a chain closing on itself is its sides alone: a ring)
    let closed = moves.len() > 1 && closes(rev,moves[0],moves[moves.len()-1],eps);
    let mut faces = Vec::with_capacity(moves.len()+2);
    if !closed { faces.extend(cap(rev,o,a,scale(m,-1.),back,eps)?) }
    for &mv in moves { faces.push(side(rev,mv,eps)?); }
    if !closed { faces.extend(cap(rev,o1,a1,m1,front,eps)?) }
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
        let reach = reach_of(rev);
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
    // a slide square to the axis turning at a sharp corner from a run of such slides goes on with it
    // (`mitred`) where the run stays laid: each slide longer than the mitres either side of it take
    // back on the side each corner turns toward (the tool's reach times the tangent of half the
    // turn), and the run turning through a right angle at most — so it runs on along one way, and
    // no two of its slides but those that join come within reach of one another
    // (each chain's run so far, kept as it grows: the turn at its last corner where it can be mitred)
    let far = reach_of(&rev);
    let mut runs: Vec<Run> = Vec::new();
    let laid = |run: &Run,last: Move,mv: Move| -> Option<f64> {
        let ((_,dl,ll),(_,dn,ln),Move::Across {at,..}) = (leg(&rev,last)?,leg(&rev,mv)?,mv) else { return None };
        let phi = turn(at.vector(rev.frame.z),dl,dn);
        let back = |phi: f64| far*(0.5*phi.abs()).tan();
        let taken = back(phi)+if run.before*phi > 0. { back(run.before) } else { 0. };
        let spare = 1e-3*far;
        (run.across && run.hi.max(run.at+phi)-run.lo.min(run.at+phi) <= 0.5*PI && taken < ll-spare && back(phi) < ln-spare).then_some(phi)
    };
    for &(i,piece,mv,back) in &moves {
        let mut corner = None;
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
                    (Some([_,[o,a,m]]),Some([[p,b,n],_])) if norm(sub(o,p)) <= eps && dot(a,b) >= 1.-1e-12 => {
                        let run = runs.last().unwrap();
                        // (a mitred run takes slides square to the axis alone)
                        if dot(m,n) >= 1.-1e-9 { !run.cornered || leg(&rev,mv).is_some() } else { corner = laid(run,last,mv); corner.is_some() }
                    }
                    _ => false,
                },
            }
        });
        if goes_on {
            chains.last_mut().unwrap().push((i,piece,mv,back));
            let run = runs.last_mut().unwrap();
            run.across &= leg(&rev,mv).is_some();
            run.before = corner.unwrap_or(0.);
            if let Some(phi) = corner { run.cornered = true; run.at += phi; (run.lo,run.hi) = (run.lo.min(run.at),run.hi.max(run.at)); }
        } else {
            chains.push(vec![(i,piece,mv,back)]);
            runs.push(Run {across:leg(&rev,mv).is_some(),..Run::default()});
        }
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
    // a chain ending where a sweep cut before it holds the whole tool (a closed path come back to
    // where it began) needs no half ahead there (`front`): that sweep's holds it, and the half's
    // surface met with that sweep's own — the same tool where both stand — would touch it there
    let holds = |c: &[(usize,&Trajectory,Move,bool)]| -> Vec<[V;2]> {
        let (_,_,first,back) = c[0];
        let (_,_,last,_) = c[c.len()-1];
        let mut at: Vec<[V;2]> = Vec::new();
        match (first,last) {
            (Move::Along {at:p,s},_) => {
                let (o,a) = (p.point(rev.frame.o),p.vector(rev.frame.z));
                at.extend([[o,a],[add(o,scale(a,s)),a]]);
            }
            (Move::Turned {at:p,..},_) if c.len() == 1 && stands(&rev,first).is_none() => at.push([p.point(rev.frame.o),p.vector(rev.frame.z)]),
            (Move::Slanted {from,axis,..},Move::Slanted {to,..}) => {
                if back { at.push([from,axis]) }
                at.push([to,axis]);
            }
            // (a chain closing on itself is its sides alone: it holds the tool nowhere whole)
            _ => if let (Some([s,_]),Some([_,e])) = (stands(&rev,first),stands(&rev,last)) {
                if c.len() > 1 && closes(&rev,first,last,eps) { return at }
                if back { at.push([s[0],s[1]]) }
                at.push([e[0],e[1]]);
            },
        }
        at
    };
    let mut held: Vec<[V;2]> = Vec::new();
    let fronts: Vec<bool> = chains.iter().map(|c| {
        let front = match stands(&rev,c[c.len()-1].2) {
            Some([_,[o,a,_]]) => !held.iter().any(|&[p,b]| norm(sub(p,o)) <= eps && dot(a,b) >= 1.-1e-12),
            None => true,
        };
        held.extend(holds(c));
        front
    }).collect();
    let jobs: Vec<(&Vec<(usize,&Trajectory,Move,bool)>,bool,bool)> = chains.iter().zip(fronts).zip(&runs).map(|((c,f),r)| (c,f,r.cornered)).collect();
    let sweeps = crate::util::par::map(&jobs,|&(c,front,cornered)| {
        let (i,piece,mv,back) = c[0];
        match (c.len(),mv) {
            (_,Move::Slanted {axis,..}) => {
                let mut cs: Vec<V> = c.iter().filter_map(|x| match x.2 { Move::Slanted {from,..} => Some(from),_ => None }).collect();
                if let Move::Slanted {to,..} = c[c.len()-1].2 { cs.push(to) }
                super::raster::run(&cs,axis,mill.as_ref().expect("a slanted move is read only of a mill"),blank,back,tol)
            }
            (1,_) => sweep(&rev,mill.as_ref(),tool,piece,mv,[back,front],blank,bar,tol),
            _ if cornered => mitred(&rev,&c.iter().map(|x| x.2).collect::<Vec<_>>(),back,front,tol),
            _ => chain(&rev,&c.iter().map(|x| x.2).collect::<Vec<_>>(),back,front,tol),
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
    let swept = super::member::Swept {sampled:false,tool:&solid,path,roll};
    super::closed::judge(&out,&swept,blank,Some(blank),&[],tol)?;
    Ok(out)
}

/// What one piece sweeps, built by its move's road.
#[allow(clippy::too_many_arguments)]
fn sweep(rev: &Revolved,mill: Option<&super::raster::Mill>,tool: &Brep,piece: &Trajectory,mv: Move,[back,front]: [bool;2],blank: &Brep,
    bar: f64,tol: f64)
    -> Result<Brep,String> {
    let eps = 1e-9*(1.+rev.size);
    match mv {
        Move::Along {at,s} => along(rev,at,s,eps),
        Move::Lifted {at,s} => lift(rev,at,s),
        Move::Across {..} => chain(rev,&[mv],back,front,tol),
        Move::Turned {at,q,w,angle} if angle.abs() >= TAU*(1.-1e-12) => ring(rev,at,q,w,angle),
        Move::Turned {..} => chain(rev,&[mv],back,front,tol),
        Move::Slanted {from,to,axis} => super::raster::run(&[from,to],axis,mill.expect("a slanted move is read only of a mill"),blank,back,tol),
        Move::Closed {roll} => super::closed::closed(tool,piece,roll,bar,tol).map_err(|e| format!("{e:?}")),
    }
}
