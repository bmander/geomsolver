//! A prism's sheet under a motion that carries its profile's plane within itself, built from the
//! planar envelope and not traced (issue #70, part 1): every station sections the prism in its
//! profile and the motion moves every station alike, so the contacts at every station are one
//! curve — the profile's envelope in the plane (`planar_cut`) — and the sheet is that curve extruded
//! along the prism. Each edge of the profile cuts as a line or an arc, and each convex corner as a
//! point over the fan between its sides' normals; the curve is their envelopes in walk order, an
//! edge's over the rolls its cut lies on it, a corner's from where the edge before reaches it to
//! where the edge after leaves it. Read exactly (`planar_cut::cut_at`: `C` and `C'` of a point, line
//! or arc are exact), interpolated by cubic Hermite spans halved until every span stays within the
//! fit's bars of the envelope at its quarter points, and extruded linearly over the prism's length.
//! A prism leaned along a rack's slide under its roll (#826, `Screwed`) is read the same way in the
//! plane square to the observer's axis, and its envelope carried up the helix every section's turns
//! along rather than extruded (`screwed`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::build::Seg;
use crate::geom::{Curve,Frame};
use crate::nurbs::Net;
use super::planar_cut::{Cut,Cutting,PlanarStep,cut_at};
use super::sheet::{AtStage,Fitted,Inside,Refused,Say,Sheet,Stage,Tolerance,UNNAMED};
use std::f64::consts::{PI,TAU};

type V = [f64;3];
type P = [f64;2];

/// How far the section curve may pass from the envelope between its nodes without a stated
/// tolerance (mm), and how far its tangent may turn from the envelope's (degrees). The curve is
/// exact where it is read and its spans converge at the fourth power of their length, so the
/// default is fine and cheap, not gross.
const FIT: f64 = 1e-4;
const TURN: f64 = 0.05;
/// Rolls a piece's cut is sought at across the declared roll.
const SCAN: usize = 720;
/// Spans a piece starts with, and the most the whole curve may be halved into.
const FIRST_SPANS: usize = 4;
const MOST_SPANS: usize = 8192;
/// Rolls of a piece's cut, and stations along the prism, the blank is asked about; the sheet's
/// contacts stand at the same stations.
const SAMPLES: usize = 64;
const STATIONS: usize = 9;
/// How far a leaned prism's section may turn at a corner (radians) and still run on smoothly
/// there, as it does at a round's ends: a corner with no fan, convex either way.
const SMOOTH: f64 = 1e-6;

/// One piece of the profile's loop as it cuts: an edge, a line or an arc, with the rolls its cut
/// lies on it (`run`), or the corner after it, a point.
struct Piece { tool: Cutting,program: Vec<PlanarStep>,kind: Kind,run: Option<Run> }

#[derive(Clone,Copy)]
enum Kind {
    Line,
    /// The arc's middle angle, the sign the walk turns it, and its sweep (radians).
    Arc { middle: f64,sense: f64,sweep: f64 },
    /// A corner, convex where the walk turns left.
    Corner { at: P,convex: bool },
}

impl Piece {
    fn new(tool: Cutting,program: &[PlanarStep],kind: Kind) -> Piece {
        Piece {tool,program:program.to_vec(),kind,run:None}
    }
    /// An edge, with the rolls within `roll` its cut lies on it.
    fn edge(tool: Cutting,program: &[PlanarStep],kind: Kind,roll: [f64;2]) -> Result<Piece,String> {
        let mut piece = Piece::new(tool,program,kind);
        piece.run = run(&piece,roll)?;
        Ok(piece)
    }
    /// The cut at roll `t` (degrees).
    fn cut(&self,t: f64) -> Option<Cut> {
        cut_at(self.tool,&self.program,t).filter(|c| c.c.iter().chain(&c.dc).all(|x| x.is_finite()))
    }
    /// Where along the edge the cut at roll `t` stands, as a fraction of the walk over it.
    fn fraction(&self,t: f64) -> Option<f64> {
        let s = self.cut(t)?.s;
        match self.kind {
            Kind::Line => Some(s),
            Kind::Arc {middle,sense,sweep} => Some(((sense*(s-middle)+PI).rem_euclid(TAU)-PI+sweep/2.)/sweep),
            Kind::Corner {..} => None,
        }
    }
}

/// Where an edge's cut runs within the declared roll: the rolls at its walk's start and end, each
/// with whether the cut reaches that end of the edge there (else the roll's limit stops it).
#[derive(Clone,Copy,Debug)]
struct Run { start: (f64,bool),end: (f64,bool) }

/// The rolls over which an edge's cut lies on it, within `[a, b]` (degrees): none where it never
/// does, refused where it does over two stretches.
fn run(piece: &Piece,[a,b]: [f64;2]) -> Result<Option<Run>,String> {
    let on = |t: f64| piece.fraction(t).is_some_and(|f| (0. ..=1.).contains(&f));
    let rolls: Vec<f64> = (0..=SCAN).map(|k| a+(b-a)*k as f64/SCAN as f64).collect();
    let marks: Vec<bool> = rolls.iter().map(|&t| on(t)).collect();
    let starts: Vec<usize> = (0..marks.len()).filter(|&k| marks[k] && (k == 0 || !marks[k-1])).collect();
    let [first] = starts[..] else {
        if starts.is_empty() { return Ok(None) }
        return Err("an edge's cut lies on it over two stretches of the roll".into())
    };
    let last = (first..marks.len()).take_while(|&k| marks[k]).last().expect("a mark");
    // the boundary between a roll on the edge and one off it, from the side on it, to the last bit
    let edge = |inner: f64,off: f64| crate::util::roots::bisect(inner,off,|x,y| 0.5*(x+y),
        |x,y| { let m = 0.5*(x+y); m != x && m != y },|t| on(t)).0;
    let lo = if first == 0 { (a,false) } else { (edge(rolls[first],rolls[first-1]),true) };
    let hi = if last == SCAN { (b,false) } else { (edge(rolls[last],rolls[last+1]),true) };
    // which end of the edge each boundary is: the fraction nearer 0 is the walk's start
    let f = |t: f64| piece.fraction(t).unwrap_or(0.5);
    Ok(Some(if f(lo.0) <= f(hi.0) { Run {start:lo,end:hi} } else { Run {start:hi,end:lo} }))
}

/// A node of the section curve: the roll, the point and `C'` per degree there.
#[derive(Clone,Copy)]
struct Node { t: f64,c: P,dc: P }

/// A Hermite span's point at `w` of its length in roll.
fn hermite(a: &Node,b: &Node,w: f64) -> (P,P) {
    let dt = b.t-a.t;
    let (h00,h10,h01,h11) = (2.*w*w*w-3.*w*w+1.,w*w*w-2.*w*w+w,-2.*w*w*w+3.*w*w,w*w*w-w*w);
    let (d00,d10,d01,d11) = (6.*w*w-6.*w,3.*w*w-4.*w+1.,-6.*w*w+6.*w,3.*w*w-2.*w);
    let p = std::array::from_fn(|k| h00*a.c[k]+h10*dt*a.dc[k]+h01*b.c[k]+h11*dt*b.dc[k]);
    let d = std::array::from_fn(|k| (d00*a.c[k]+d10*dt*a.dc[k]+d01*b.c[k]+d11*dt*b.dc[k])/dt);
    (p,d)
}

fn turn_between(a: P,b: P) -> f64 {
    let (la,lb) = (a[0].dhypot(a[1]),b[0].dhypot(b[1]));
    if !(la > 0. && lb > 0.) { return 0. }
    ((a[0]*b[1]-a[1]*b[0]).datan2(a[0]*b[0]+a[1]*b[1])).abs().to_degrees()
}

/// **The sheet of an extruded cutter under a motion that keeps its profile's plane** (#70, #721
/// F2c): the cut `name`'s prism, its profile's loops `walks` (each corner's convexity `convex`)
/// over `extent` of its extrusion along `frame`'s `z`, under `program` (the motion in the
/// profile's plane, `frame`'s) over `roll` (degrees); `inside` asks the blank (mm); `tolerance`
/// sets the fit's bar (half of it), else `FIT`.
#[allow(clippy::too_many_arguments)]
pub fn sheet(name: &str,walks: &[Vec<Seg>],convex: &[Vec<bool>],extent: [f64;2],frame: Frame,program: &[PlanarStep],roll: [f64;2],
    inside: Inside,tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,Refused> {
    laid(name,walks,convex,extent,frame,program,roll,inside,tolerance,Along::Line,say)
}

/// How a section's envelope is laid along the heights of its sheet: extruded straight along the
/// frame's `z`, or carried up the helix a leaned prism's sections turn along (#826).
#[derive(Clone,Copy,Debug)]
enum Along { Line,Helix(Helix) }

/// **The helix a leaned prism's envelope is carried along under a rack's roll** (#826): `centre`
/// the observer's axis in the frame's plane, the section read at height `base`, and as the height
/// rises by one millimetre the section carried `shift` across the plane (the extrusion's lean),
/// so cut by the roll `lag` degrees sooner and its envelope turned `rate` radians about the axis.
/// The envelope of the section at height `h` at roll `t − lag (h − base)` is the base's at `t`
/// turned by `rate (h − base)`: a helicoid of lead `2π / rate`.
#[derive(Clone,Copy,Debug)]
struct Helix { centre: P,rate: f64,base: f64,shift: P,lag: f64 }

impl Helix {
    /// The angle the section at height `h` is turned by from the base's.
    fn angle(&self,h: f64) -> f64 { self.rate*(h-self.base) }
    /// `p`, a point of the base's envelope, turned to the envelope at height `h`.
    fn turn(&self,p: P,h: f64) -> P { rotated(p,self.centre,self.angle(h)) }
}

/// `p` turned by `angle` (radians) about `centre`.
fn rotated(p: P,centre: P,angle: f64) -> P {
    let (sn,cs) = angle.dsin_cos();
    let q = [p[0]-centre[0],p[1]-centre[1]];
    [centre[0]+cs*q[0]-sn*q[1],centre[1]+sn*q[0]+cs*q[1]]
}

/// **A leaned prism under a rack's roll, read as a screw's sheet** (#826): a prism of lines and
/// arcs extruded along `axis` aslant `frame`'s plane, its lean in the plane along the rack's slide,
/// under a slide seen from a turn about `frame`'s `z` (a helical rack against the blank it cuts).
/// The prism is the same set slid along its own extrusion, so its section by a plane square to
/// the turn at any height is the base's section slid along the rack's line, and the rack's roll
/// carries a section slid along its line to its own envelope turned about the axis: every
/// section's envelope is the base's turned, and the sweep's side is that envelope carried up a
/// helix (`Helix`), as a helical gear's flank is its transverse involute carried up its helix.
/// What is kept is the section at the middle of the heights over which every edge of the prism
/// stands (`extent`, the band its sheet spans), its loop walked counter-clockwise about `frame`'s
/// `z`, each corner's convexity, and the helix.  An arc of the profile — a rack's tip round, which
/// every hob has (#904 A6) — is sectioned as an ellipse, the circle stretched along the lean, and
/// cuts in closed form as the circle does (`planar_cut::Cutting::Ellipse`): its envelope is the
/// root's trochoidal fillet, exact, carried up the helix with the flanks'.
pub struct Screwed { pub section: Vec<Vec<Seg>>,pub convex: Vec<Vec<bool>>,pub extent: [f64;2],pub frame: Frame,
    pub program: Vec<PlanarStep>,helix: Helix }

impl Screwed {
    /// The prism of one loop `walks` (in a plane square to the unit `axis`, its
    /// extrusion over `extent` along it) under `program` (the motion in `frame`'s plane) read as a
    /// screw's sheet; none where it is not one: a loop with an edge neither a line nor an arc,
    /// more than one loop, a motion that is no slide seen from a turn, an extrusion square to the turn's
    /// axis, along it (a prism the motion keeps the plane of, `sheet`'s) or leaning across the
    /// slide (its sections cut by the rack at another depth, so no turn of one another's envelope).
    pub fn read(walks: &[Vec<Seg>],axis: V,extent: [f64;2],frame: Frame,program: &[PlanarStep]) -> Option<Screwed> {
        use crate::util::space::{add,cross,distance,scale,sub};
        let [walk] = walks else { return None };
        let &[PlanarStep::Slide {direction,advance},PlanarStep::Turn {centre,ratio},PlanarStep::Relative] = program else { return None };
        if !(advance.abs() > 0. && ratio.abs() > 0.) || walk.iter().any(|s| !matches!(s.curve,Curve::Line {..} | Curve::Circle(..))) {
            return None
        }
        let length = direction[0].dhypot(direction[1]);
        if !(length > 0.) { return None }
        let d = [direction[0]/length,direction[1]/length];
        let e = frame.dir_local(axis);
        let lean = e[0].dhypot(e[1]);
        if !(e[2].abs() > 1e-6) || !(lean > 1e-9) || (e[0]*d[1]-e[1]*d[0]).abs() > 1e-9*lean { return None }
        // a point's foot on the plane at height `h` along the extrusion, and a direction's in it
        let at = |c: V,h: f64| { let w = (h-c[2])/e[2]; [c[0]+w*e[0],c[1]+w*e[1]] };
        let along = |v: V| [v[0]-v[2]*e[0]/e[2],v[1]-v[2]*e[1]/e[2]];
        // the heights of each edge's points, the lowest and the highest: a line's at its ends, an
        // arc's at its ends and where its height turns within its span
        let heights = |seg: &Seg| -> [f64;2] {
            let mut z: Vec<f64> = vec![frame.local(seg.start())[2],frame.local(seg.end())[2]];
            if let Curve::Circle(circle,_) = &seg.curve {
                let (x,y) = (frame.dir_local(circle.x)[2],frame.dir_local(circle.y)[2]);
                let [lo,hi] = seg.span();
                for turn in [0.,PI] {
                    let w = y.datan2(x)+turn;
                    // the turning angle's copy within the span, if there is one
                    let w = lo+(w-lo).rem_euclid(TAU);
                    if w <= hi { z.push(frame.local(seg.curve.point(w))[2]); }
                }
            }
            [z.iter().copied().fold(f64::INFINITY,f64::min),z.iter().copied().fold(f64::NEG_INFINITY,f64::max)]
        };
        // the heights over which every point's line along the extrusion runs within the prism's extent
        let reach: Vec<[f64;2]> = walk.iter().map(|seg| {
            let [lo,hi] = heights(seg);
            let [a,b] = extent.map(|w| w*e[2]);
            // a point at height z runs over [z + min, z + max]: every point of the edge, over the
            // highest's start to the lowest's end
            [hi+a.min(b),lo+a.max(b)]
        }).collect();
        let band = [reach.iter().map(|r| r[0]).fold(f64::NEG_INFINITY,f64::max),reach.iter().map(|r| r[1]).fold(f64::INFINITY,f64::min)];
        if !(band[0] < band[1]) { return None }
        let base = 0.5*(band[0]+band[1]);
        // each edge's section at the base: a line's between its ends' feet; an arc's an ellipse, the
        // affine image of its circle, on its principal axes and read at its circle's own angle less
        // the turn from the circle's axes to them
        let lift = |p: P| frame.at([p[0],p[1],base]);
        let mut section: Vec<Seg> = Vec::with_capacity(walk.len());
        for (k,seg) in walk.iter().enumerate() {
            section.push(match &seg.curve {
                Curve::Circle(circle,r) => {
                    let (u,v) = (along(frame.dir_local(circle.x)),along(frame.dir_local(circle.y)));
                    let (u,v) = ([r*u[0],r*u[1]],[r*v[0],r*v[1]]);
                    let turn = 0.5*(2.*(u[0]*v[0]+u[1]*v[1])).datan2(u[0]*u[0]+u[1]*u[1]-v[0]*v[0]-v[1]*v[1]);
                    let (sn,cs) = turn.dsin_cos();
                    let (p,q) = ([cs*u[0]+sn*v[0],cs*u[1]+sn*v[1]],[cs*v[0]-sn*u[0],cs*v[1]-sn*u[1]]);
                    let (a,b) = (p[0].dhypot(p[1]),q[0].dhypot(q[1]));
                    if !(a > 0. && b > 0.) { return None }
                    let (x,y) = (frame.dir([p[0]/a,p[1]/a,0.]),frame.dir([q[0]/b,q[1]/b,0.]));
                    let o = lift(at(frame.local(circle.o),base));
                    Seg {curve:Curve::Ellipse(Frame {o,x,y,z:cross(x,y)},a,b),t:seg.t.map(|t| t-turn),source:k}
                }
                _ => {
                    let (p,q) = (lift(at(frame.local(seg.start()),base)),lift(at(frame.local(seg.end()),base)));
                    let l = distance(p,q);
                    if !(l > 0.) { return None }
                    Seg {curve:Curve::Line {p,d:scale(sub(q,p),1./l)},t:[0.,l],source:k}
                }
            });
        }
        // walked counter-clockwise about the frame's `z` (an extrusion against it walks the other
        // way): the area its points enclose, an arc's sampled along it
        let points: Vec<P> = section.iter().flat_map(|seg| (0..8).map(move |j| seg.t[0]+(seg.t[1]-seg.t[0])*j as f64/8.)
            .map(move |t| { let l = frame.local(seg.curve.point(t)); [l[0],l[1]] })).collect();
        let area: f64 = (0..points.len()).map(|k| { let (p,q) = (points[k],points[(k+1)%points.len()]); p[0]*q[1]-p[1]*q[0] }).sum();
        if !(area.abs() > 0.) { return None }
        if area < 0. {
            section.reverse();
            for seg in &mut section { seg.t.reverse(); }
            // (a line runs from its own start: walked back, it starts at its end)
            for seg in &mut section {
                if let Curve::Line {p,d} = seg.curve {
                    let l = seg.t[0];
                    seg.curve = Curve::Line {p:add(p,scale(d,l)),d:scale(d,-1.)};
                    seg.t = [0.,l];
                }
            }
        }
        // convex where the walk turns left; where it runs on smoothly, as at a round's ends, too:
        // the corner's fan is empty, its sides' cuts meeting there
        let n = section.len();
        let convex: Vec<bool> = (0..n).map(|k| {
            let (a,b) = (&section[k],&section[(k+1)%n]);
            let tangent = |seg: &Seg,t: f64| { let l = frame.dir_local(seg.curve.tangent(t)); let w = (seg.t[1]-seg.t[0]).signum(); [w*l[0],w*l[1]] };
            let (p,q) = (tangent(a,a.t[1]),tangent(b,b.t[0]));
            let (lp,lq) = (p[0].dhypot(p[1]),q[0].dhypot(q[1]));
            let turn = (p[0]*q[1]-p[1]*q[0]).datan2(p[0]*q[0]+p[1]*q[1]);
            !(lp > 0. && lq > 0.) || turn > -SMOOTH
        }).collect();
        // the section at a height one millimetre up is slid `shift` across the plane, along the
        // rack's line; the rack reaches it `lag` degrees of the roll sooner, and its envelope is
        // turned by the observer's turn over those degrees
        let shift = [e[0]/e[2],e[1]/e[2]];
        let along = shift[0]*d[0]+shift[1]*d[1];
        let lag = along*360./advance;
        let rate = (ratio*std::f64::consts::PI/180.)*lag;
        Some(Screwed {section:vec![section],convex:vec![convex],extent:band,frame,program:program.to_vec(),
            helix:Helix {centre,rate,base,shift,lag}})
    }
    /// How far its sections turn about the axis a millimetre up (radians): `2π` over the lead.
    pub fn rate(&self) -> f64 { self.helix.rate }
}

/// **The sheet of a leaned prism under a rack's roll** (#826): the envelope of its section at the
/// middle of its band (`Screwed`), read exactly as `sheet` reads a prism's, carried up the helix
/// its sections turn along — the section's cubic spans by a cubic in the height fitted to the
/// helix within an eighth of the fit's bar, its poles each section pole turned and raised. The
/// helix is held to what it claims: at both ends of the band the section's own envelope, slid
/// and read at its lagging roll, is the base's turned. Refused where the sheet's ends at the
/// band's heights stand in the blank.
pub fn screwed(name: &str,screwed: &Screwed,roll: [f64;2],inside: Inside,tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,Refused> {
    laid(name,&screwed.section,&screwed.convex,screwed.extent,screwed.frame,&screwed.program,roll,inside,tolerance,
        Along::Helix(screwed.helix),say)
}

/// How many points along each span of the helix's fit its error is read at, and the most spans it
/// may take.
const HELIX_READS: usize = 16;
const MOST_HELIX_SPANS: usize = 768;

/// The helix of the band `[lo, hi]` as a cubic in the height: `(cos θ, sin θ, h)` at each height,
/// `θ` the section's turn there, interpolated at evenly spaced heights, doubled until a point
/// `reach` from the axis carried by it stands within `bar` of where the helix carries it — its
/// knots the heights, and the error read.
fn helix_fit(helix: &Helix,[lo,hi]: [f64;2],reach: f64,bar: f64) -> Result<(crate::nurbs::BSpline,f64),String> {
    let exact = |s: f64| { let h = lo+(hi-lo)*s; let (sn,cs) = helix.angle(h).dsin_cos(); [cs,sn,h] };
    let mut n = 3;
    loop {
        let t: Vec<f64> = (0..=n).map(|i| i as f64/n as f64).collect();
        let mut fit = crate::nurbs::interpolate(&t.iter().map(|&s| exact(s)).collect::<Vec<_>>(),&t,3)
            .ok_or("the helix could not be interpolated")?;
        let reads = HELIX_READS*n;
        let error = (0..=reads).map(|k| {
            let s = k as f64/reads as f64;
            let (p,q) = (fit.point(s),exact(s));
            reach*(p[0]-q[0]).dhypot(p[1]-q[1])+(p[2]-q[2]).abs()
        }).fold(0.,f64::max);
        if error <= bar {
            for k in &mut fit.knots { *k = lo+(hi-lo)**k; }
            return Ok((fit,error))
        }
        if n >= MOST_HELIX_SPANS { return Err(format!("the helix needs more than {MOST_HELIX_SPANS} spans to keep within {bar:.2e} mm")) }
        n *= 2;
    }
}

/// The section's envelope laid along its heights as `along` says: `sheet`'s, extruded, or
/// `screwed`'s, carried up a helix.
#[allow(clippy::too_many_arguments)]
fn laid(name: &str,walks: &[Vec<Seg>],convex: &[Vec<bool>],extent: [f64;2],frame: Frame,program: &[PlanarStep],roll: [f64;2],
    inside: Inside,tolerance: Option<Tolerance>,along: Along,say: &Say) -> Result<Fitted,Refused> {
    let refused = |stage: Stage,why: String| Refused::at(stage,format!("`{name}`: {why}"));
    let started = crate::util::clock::Instant::now();
    let local = |p: V| { let l = frame.local(p); [l[0],l[1]] };
    let lift = |p: P,h: f64| match along {
        Along::Line => frame.at([p[0],p[1],h]),
        Along::Helix(helix) => { let q = helix.turn(p,h); frame.at([q[0],q[1],h]) }
    };
    let stations: Vec<f64> = (0..STATIONS).map(|j| extent[0]+(extent[1]-extent[0])*j as f64/(STATIONS-1) as f64).collect();
    let in_blank = |points: &[P]| -> Result<bool,String> {
        let lifted: Vec<V> = points.iter().flat_map(|&p| stations.iter().map(move |&h| lift(p,h))).collect();
        Ok(inside(&lifted)?.into_iter().any(|b| b))
    };
    // each piece's stretch of the roll in walk order — an edge's where its cut lies on it, a
    // corner's from where the edge before reaches it to where the edge after leaves it — or why
    // it has none
    let stretch = |pieces: &[Piece],i: usize| -> Result<[f64;2],String> {
        let n = pieces.len();
        match pieces[i].kind {
            Kind::Corner {at,convex} => {
                let (Some(before),Some(after)) = (pieces[i-1].run,pieces[(i+1)%n].run) else {
                    return Err(format!("a side of the corner at {at:?} does not cut within the declared roll"))
                };
                if !(before.end.1 && after.start.1) {
                    return Err(format!("the cut of a side of the corner at {at:?} leaves the declared roll before it reaches the corner"))
                }
                if !convex {
                    return Err(format!("the profile's corner at {at:?} is concave: its sides' cuts cross there, which one sheet does \
                        not follow"))
                }
                Ok([before.end.0,after.start.0])
            }
            _ => pieces[i].run.map(|r| [r.start.0,r.end.0])
                .ok_or_else(|| format!("edge {} of the profile does not cut within the declared roll",i/2)),
        }
    };
    // each loop's pieces, edge k then the corner after it
    let mut chosen: Option<(Vec<Piece>,Vec<bool>)> = None;
    for (walk,convex) in walks.iter().zip(convex) {
        let mut pieces = Vec::with_capacity(2*walk.len());
        for (k,seg) in walk.iter().enumerate() {
            let (start,end) = (local(seg.start()),local(seg.end()));
            match &seg.curve {
                Curve::Line {..} => pieces.push(Piece::edge(Cutting::Line {a:start,b:end},program,Kind::Line,roll)
                    .at(Stage::Reach)?),
                Curve::Circle(..) | Curve::Ellipse(..) => {
                    let (tool,kind): (Box<dyn Fn(f64) -> Cutting>,Kind) = match seg.curve {
                        Curve::Circle(circle,r) => {
                            let c = local(circle.o);
                            let mid = local(seg.curve.point(0.5*(seg.t[0]+seg.t[1])));
                            let angle = |p: P| (p[1]-c[1]).datan2(p[0]-c[0]);
                            let sense = ((start[0]-c[0])*(mid[1]-c[1])-(start[1]-c[1])*(mid[0]-c[0])).signum();
                            (Box::new(move |side| Cutting::Circle {centre:c,radius:r,side}),
                                Kind::Arc {middle:angle(mid),sense,sweep:(seg.t[1]-seg.t[0]).abs()})
                        }
                        // a leaned prism's round in its section square to the axis (#904 A6): read
                        // by its own parameter, so the arc's middle, sense and sweep are its span's
                        _ => {
                            let Curve::Ellipse(ellipse,a,b) = seg.curve else { unreachable!() };
                            let axis = |d: V,k: f64| { let l = frame.dir_local(d); [k*l[0],k*l[1]] };
                            let (c,u,v) = (local(ellipse.o),axis(ellipse.x,a),axis(ellipse.y,b));
                            (Box::new(move |side| Cutting::Ellipse {centre:c,u,v,side}),
                                Kind::Arc {middle:0.5*(seg.t[0]+seg.t[1]),sense:(seg.t[1]-seg.t[0]).signum(),sweep:(seg.t[1]-seg.t[0]).abs()})
                        }
                    };
                    // an arc cuts on the side of its instant centre and the other: the arc's is the
                    // one whose cut lies on it
                    let mut sides: Vec<Piece> = [1.,-1.].into_iter().map(|side| Piece::edge(tool(side),program,kind,roll))
                        .collect::<Result<_,_>>().at(Stage::Reach)?;
                    if sides.iter().all(|p| p.run.is_some()) {
                        return Err(refused(Stage::Reach,"an arc of the profile cuts on both sides of the instant centre, which one \
                            sheet does not follow".into()))
                    }
                    let side = if sides[0].run.is_none() && sides[1].run.is_some() { 1 } else { 0 };
                    pieces.push(sides.swap_remove(side));
                }
                _ => return Err(refused(Stage::Reach,"the profile has an edge neither a line nor an arc".into())),
            }
            pieces.push(Piece::new(Cutting::Point(end),program,Kind::Corner {at:end,convex:convex[k]}));
        }
        let mut reaching = vec![false;pieces.len()];
        for i in 0..pieces.len() {
            let Ok([a,b]) = stretch(&pieces,i) else { continue };
            let points: Vec<P> = (0..=SAMPLES).filter_map(|j| pieces[i].cut(a+(b-a)*j as f64/SAMPLES as f64).map(|c| c.c)).collect();
            reaching[i] = in_blank(&points).at(Stage::Reach)?;
        }
        if reaching.iter().any(|r| *r) {
            if chosen.is_some() { return Err(refused(Stage::Reach,"two loops of the profile cut the blank".into())) }
            chosen = Some((pieces,reaching));
        }
    }
    let Some((pieces,reaching)) = chosen else {
        return Err(refused(Stage::Reach,"no cut of the profile reaches the blank within the declared roll".into()))
    };
    let n = pieces.len();
    if reaching.iter().all(|r| *r) {
        return Err(refused(Stage::Reach,"every piece of the profile cuts the blank, so its sheet would close".into()))
    }
    // the run of pieces cutting the blank: after the longest stretch of the loop that does not, from
    // an edge to an edge
    let gap = (0..n).filter(|&i| !reaching[i] && reaching[(i+n-1)%n])
        .map(|i| (i,(0..n).take_while(|k| !reaching[(i+k)%n]).count())).max_by_key(|&(_,length)| length).expect("a gap");
    let mut first = (gap.0+gap.1)%n;
    let mut last = (gap.0+n-1)%n;
    if first % 2 == 1 { first = (first+n-1)%n; }
    if last % 2 == 1 { last = (last+1)%n; }
    let order: Vec<usize> = (0..n).map(|k| (first+k)%n).take_while(|&i| i != (last+1)%n).collect();
    if order.len() == n {
        return Err(refused(Stage::Reach,"the profile's cut leaves the blank only along one edge, so its sheet would close".into()))
    }
    let stretches: Vec<(usize,[f64;2])> = order.iter().map(|&i| stretch(&pieces,i).map(|s| (i,s)))
        .collect::<Result<_,_>>().map_err(|why| refused(Stage::Reach,why))?;
    (say.stage)(&format!("`{name}`: the profile cuts the blank over {} pieces of its loop in the plane its motion keeps ({:?})",
        order.len(),started.elapsed()));
    (say.mark)(Stage::Reach);
    // the curve: each stretch's spans halved until each keeps to the envelope at its quarter points;
    // a span kept carries its exact middle, which the sheet withholds
    let bar = tolerance.map_or(FIT,|t| t.fit().min(FIT));
    let read = |i: usize,t: f64| -> Result<Node,Refused> {
        let c = pieces[i].cut(t).ok_or_else(|| refused(Stage::Sheet,format!("the envelope has no cut at roll {t:.6} degrees")))?;
        Ok(Node {t,c:c.c,dc:c.dc})
    };
    struct Span { a: Node,b: Node,middle: Option<Node> }
    let mut curve: Vec<(usize,Vec<Span>)> = Vec::new();
    let (mut error,mut turned,mut spans) = (0_f64,0_f64,0);
    for &(i,[a,b]) in &stretches {
        if !((b-a).abs() > 1e-12*(1.+a.abs())) { continue }
        let nodes: Vec<Node> = (0..=FIRST_SPANS).map(|j| read(i,a+(b-a)*j as f64/FIRST_SPANS as f64)).collect::<Result<_,_>>()?;
        let mut open: Vec<Span> = nodes.windows(2).rev().map(|w| Span {a:w[0],b:w[1],middle:None}).collect();
        let mut kept: Vec<Span> = Vec::new();
        // in walk order: the next span off the top, kept or halved back onto it
        while let Some(span) = open.pop() {
            let at = |q: f64| read(i,span.a.t+(span.b.t-span.a.t)*q);
            let quarters = [at(0.25)?,at(0.5)?,at(0.75)?];
            let (mut far,mut turn) = (0_f64,0_f64);
            for (q,exact) in [0.25,0.5,0.75].into_iter().zip(&quarters) {
                let (p,d) = hermite(&span.a,&span.b,q);
                (far,turn) = (far.max((p[0]-exact.c[0]).dhypot(p[1]-exact.c[1])),turn.max(turn_between(d,exact.dc)));
            }
            if far > bar || turn > TURN {
                open.push(Span {a:quarters[1],b:span.b,middle:None});
                open.push(Span {a:span.a,b:quarters[1],middle:None});
            } else {
                (error,turned) = (error.max(far),turned.max(turn));
                kept.push(Span {middle:Some(quarters[1]),..span});
            }
            if spans+kept.len()+open.len() > MOST_SPANS {
                return Err(refused(Stage::Withheld,format!("the envelope's curve needs more than {MOST_SPANS} spans to keep within \
                    {bar:.2e} mm and {TURN} degrees of it")))
            }
        }
        spans += kept.len();
        curve.push((i,kept));
    }
    if curve.is_empty() {
        return Err(refused(Stage::Sheet,"the envelope's curve runs over no roll".into()))
    }
    // its ends leave the blank, or the sheet would end inside it
    let ends = [curve[0].1[0].a.c,curve[curve.len()-1].1.last().expect("a span").b.c];
    if in_blank(&ends).at(Stage::Sheet)? {
        return Err(refused(Stage::Sheet,format!("the envelope's curve ends inside the blank (at {:?} or {:?})",ends[0],ends[1])))
    }
    (say.stage)(&format!("`{name}`: the envelope's curve in {spans} spans, within {error:.2e} mm and {turned:.3} degrees of it at their \
        quarter points ({:?})",started.elapsed()));
    // carried up a helix: held to what it claims at both ends of the band, and its ends there
    // clear of the blank (#826)
    if let Along::Helix(helix) = along {
        let reach = curve.iter().flat_map(|(_,kept)| kept.iter().map(|s| (s.a.c[0]-helix.centre[0]).dhypot(s.a.c[1]-helix.centre[1])))
            .fold(0.,f64::max);
        let (mut worst,mut at) = (0_f64,[0.;2]);
        for h in extent {
            let (slide,lag) = ([helix.shift[0]*(h-helix.base),helix.shift[1]*(h-helix.base)],helix.lag*(h-helix.base));
            for (i,kept) in &curve {
                let slid = match pieces[*i].tool {
                    Cutting::Point(p) => Cutting::Point([p[0]+slide[0],p[1]+slide[1]]),
                    Cutting::Line {a,b} => Cutting::Line {a:[a[0]+slide[0],a[1]+slide[1]],b:[b[0]+slide[0],b[1]+slide[1]]},
                    Cutting::Circle {centre,radius,side} => Cutting::Circle {centre:[centre[0]+slide[0],centre[1]+slide[1]],radius,side},
                    Cutting::Ellipse {centre,u,v,side} => Cutting::Ellipse {centre:[centre[0]+slide[0],centre[1]+slide[1]],u,v,side},
                };
                for node in kept.iter().flat_map(|s| [s.a,s.b]) {
                    let own = cut_at(slid,&pieces[*i].program,node.t-lag).ok_or_else(|| refused(Stage::Sheet,format!("the section at \
                        height {h:.4} mm has no cut at roll {:.6} degrees",node.t-lag)))?;
                    let turned = helix.turn(node.c,h);
                    let off = (own.c[0]-turned[0]).dhypot(own.c[1]-turned[1]);
                    if off > worst { (worst,at) = (off,turned); }
                }
            }
        }
        if worst > 1e-8*(1.+reach) {
            return Err(refused(Stage::Sheet,format!("the section's envelope at an end of its band stands {worst:.3e} mm from the base's \
                turned (at {at:?}): the prism's sections do not turn along one helix")))
        }
        let rims: Vec<V> = extent.iter().flat_map(|&h| curve.iter().flat_map(move |(_,kept)| kept.iter().map(move |s| (s.a.c,h))))
            .map(|(c,h)| lift(c,h)).collect();
        if inside(&rims).at(Stage::Sheet)?.into_iter().any(|b| b) {
            return Err(refused(Stage::Sheet,format!("the sheet's rim at a height of its band ({:.4} or {:.4} mm) stands in the blank",
                extent[0],extent[1])))
        }
        (say.stage)(&format!("`{name}`: the envelope of each section square to the axis is the base's turned {:.4} degrees a \
            millimetre up, within {worst:.1e} mm at the ends of its band",helix.rate.to_degrees()));
    }
    (say.mark)(Stage::Sheet);
    // the Hermite spans as one cubic B-spline, a piece's parameter running with its roll, extruded
    // linearly over the prism: knots of multiplicity three, the poles each span's Bézier's. Carried
    // up a helix, two spans of one piece meet at a knot of multiplicity two, its pole between theirs
    // left out: they share their node's derivative along the roll, which runs evenly with the
    // parameter over the piece, so the curve is as smooth there and the same (#826)
    let joined = matches!(along,Along::Helix(_));
    let mut knots = vec![0.;4];
    let mut poles: Vec<P> = vec![ends[0]];
    let mut u = 0.;
    for (_,kept) in &curve {
        let chord: f64 = kept.iter().map(|s| (s.b.c[0]-s.a.c[0]).dhypot(s.b.c[1]-s.a.c[1])).sum();
        let rate = chord.max(1e-12)/(kept[kept.len()-1].b.t-kept[0].a.t).abs();
        for (k,Span {a,b,..}) in kept.iter().enumerate() {
            let dt = b.t-a.t;
            poles.push(std::array::from_fn(|k| a.c[k]+a.dc[k]*dt/3.));
            poles.push(std::array::from_fn(|k| b.c[k]-b.dc[k]*dt/3.));
            u += dt.abs()*rate;
            if joined && k+1 < kept.len() { knots.extend([u;2]); continue }
            poles.push(b.c);
            knots.extend([u;3]);
        }
    }
    knots.push(u);
    let (net,error) = match along {
        Along::Line => (Net {du:3,dv:1,uknots:knots,vknots:vec![extent[0],extent[0],extent[1],extent[1]],
            poles:poles.iter().map(|&p| vec![lift(p,extent[0]),lift(p,extent[1])]).collect(),weights:None},error),
        // each pole of the section turned and raised by each of the helix's (#826): the net is the
        // section's curve carried by the helix's fit, which keeps a point within `reach` of the
        // axis within an eighth of the bar
        Along::Helix(helix) => {
            let reach = poles.iter().map(|p| (p[0]-helix.centre[0]).dhypot(p[1]-helix.centre[1])).fold(0.,f64::max);
            let (fit,carried) = helix_fit(&helix,extent,reach,bar/8.).map_err(|why| refused(Stage::Fit,why))?;
            let c = helix.centre;
            let poles = poles.iter().map(|&p| fit.poles.iter().map(|&[cs,sn,h]| {
                let q = [p[0]-c[0],p[1]-c[1]];
                frame.at([c[0]+cs*q[0]-sn*q[1],c[1]+sn*q[0]+cs*q[1],h])
            }).collect()).collect();
            (say.stage)(&format!("`{name}`: the helix over {:.2} mm in {} poles of degree {}, within {carried:.2e} mm {reach:.2} mm \
                out",extent[1]-extent[0],fit.poles.len(),fit.degree));
            (Net {du:3,dv:fit.degree,uknots:knots,vknots:fit.knots,poles,weights:None},error+carried)
        }
    };
    let face = crate::build::sheet(net).at(Stage::Fit)?;
    (say.mark)(Stage::Fit);
    // the contacts the sheet carries, for what reads it: its nodes at each station, and the
    // envelope at each span's middle withheld; a stretch's first node is the one before's last.
    // Carried up a helix, the normal is the section's tangent turned crossed with the helix's
    let normal = |d: P,forward: bool| { let l = d[0].dhypot(d[1]).max(1e-300)*if forward { 1. } else { -1. }; [d[1]/l,-d[0]/l] };
    let normal_at = |n: P,c: P,h: f64| match along {
        Along::Line => frame.dir([n[0],n[1],0.]),
        Along::Helix(helix) => {
            let (m,q) = (rotated(n,[0.;2],helix.angle(h)),helix.turn(c,h));
            let (a,b) = (-helix.rate*(q[1]-helix.centre[1]),helix.rate*(q[0]-helix.centre[0]));
            // (−m_y, m_x) is the tangent the normal `m` is square to; crossed with (a, b, 1)
            let z = m[0]*a+m[1]*b;
            let l = (m[0]*m[0]+m[1]*m[1]+z*z).sqrt();
            frame.dir([m[0]/l,m[1]/l,-z/l])
        }
    };
    let mut rows: Vec<(Node,P)> = Vec::new();
    let mut middles: Vec<(Node,P,usize)> = Vec::new();
    for (_,kept) in &curve {
        let forward = kept[kept.len()-1].b.t > kept[0].a.t;
        rows.pop();
        rows.push((kept[0].a,normal(kept[0].a.dc,forward)));
        for s in kept {
            let middle = s.middle.expect("a kept span's middle");
            middles.push((middle,normal(middle.dc,forward),rows.len()-1));
            rows.push((s.b,normal(s.b.dc,forward)));
        }
    }
    let mut sheet = Sheet {bands:vec![(0,UNNAMED)],points:Vec::new(),normals:Vec::new(),times:Vec::new(),rows:rows.len(),columns:STATIONS,withheld:Vec::new(),
        withheld_normals:Vec::new(),sites:Vec::new(),creases:Vec::new()};
    for (node,n) in &rows {
        for &h in &stations {
            sheet.points.push(lift(node.c,h));
            sheet.normals.push(normal_at(*n,node.c,h));
            sheet.times.push(node.t.to_radians());
        }
    }
    for (node,n,r) in &middles {
        for (c,&h) in stations.iter().enumerate() {
            sheet.withheld.push(lift(node.c,h));
            sheet.withheld_normals.push(normal_at(*n,node.c,h));
            sheet.sites.push([2*r+1,2*c]);
        }
    }
    (say.stage)(&format!("`{name}`: the sheet {} from the envelope's curve over {:.2} mm of the prism ({:?})",
        match along { Along::Line => "extruded",Along::Helix(_) => "carried up its helix" },extent[1]-extent[0],started.elapsed()));
    (say.mark)(Stage::Withheld);
    Ok(Fitted::whole(face,sheet,error))
}
