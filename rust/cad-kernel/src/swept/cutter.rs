//! **A swept cut's cutter** (#721 F2c), sectioned exactly at its stations: a solid of revolutions
//! about lines parallel to one axis (`section`) by the half-planes of that axis, a station an
//! angle; or a prism by planes square to its extrusion, a station a distance along it, every
//! section the profile itself (a rack's tooth) or, drafted, the profile moved into its material
//! (and, where the draft parts at the profile's plane, a fan of stations across the crease there).
//! Each section is a loop of pieces, every piece a stretch of one face (a revolution's meridian, a
//! prism side's edge) walked in augmented arc length — length along the pieces, and at a convex
//! corner the turning of the normal across its fan — and where the cutter's contacts within the
//! declared roll enter the blank fixes the band of stations and the stretch of the walk the sheet
//! spans (`Cutter::reach`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::build::{offset,walks as walks_of,Profile,Seg};
use crate::geom::Frame;
use crate::planar::Step;
use crate::section::Sectioned;
use super::trace::{Sample,TraceError,Tracer};
use crate::util::space::{add,cross,distance,dot,lerp,norm,scale,sub};
use std::collections::BTreeMap;
use std::f64::consts::TAU;

type V = [f64;3];

/// The augmented length of a corner: one radian of turning counts as this many millimetres of
/// profile, so a fan receives rows like an arc of that radius. Like the trace's spacings, it is a
/// cut's at unit one: a sheet's stations walk it `Tracer::unit` times over (`Loop::scaled`), so a
/// cutter twice the size turns through a fan in the same rows.
const TURN_SCALE: f64 = 0.05;
/// The same for a cap's rim in a section across a prism (`Across`): a quarter turn there carries a
/// contact from a cap's to a side's, which may be far apart in time, and the sheet's rows, laid at
/// `contact_trace::ROW_SPACING`, must step through it a radian of the motion at a time at most
/// (`contact_trace::charted`), so a rim's fan is laid as long as an arc of this radius.
const RIM_SCALE: f64 = 1.;
/// The same for the crease across a drafted prism's sides at its parting plane (#684 P), whose fan
/// runs across the stations: a radian of its turning counts as this many millimetres of the
/// extrusion. A draft's crease turns little, and its fan carries a contact from one half's time to
/// the other's only a little further along the crease's path, so its stretch of the sheet is about
/// as long in space as in stations; a fan laid longer leaves columns standing on one curve, a fold
/// of the fitted sheet. How far the fan carries a contact is the motion's, which the cutter does not
/// know: a fan laid as long as its contacts' path across it (admission reads that path) would hold
/// for any motion, where this holds for a draft's few degrees. It is laid when the cutter is read,
/// before any cut's unit is known, so it stays a millimetre at every size.
const CREASE_SCALE: f64 = 1.;
/// How near an end of one piece is to the axis to be on it (mm).
const ON_AXIS: f64 = 1e-5;
const SAMPLES_PER_PIECE: usize = 64;
/// How many places of every piece of a section loop the reach asks for contacts in the blank (and
/// the middle of every convex corner's fan): a piece as short as a tip's round is asked as often
/// as a flank, whatever the cutter's size, so the faces the walk runs over are the same for a
/// cutter scaled (#904 A3).
const REACH_SAMPLES: usize = 32;
/// How many stations the reach reads (`Cutter::reach`): round the axis, a step of 3.75 degrees.
const STATIONS: usize = 96;
/// How near the reach bisects the station where a cutter of revolutions' contacts first reach
/// the blank (radians): its band's start, which its stations are counted from (`Cutter::reach`).
const START: f64 = 1e-12;
/// Where a piece's face id keeps which meeting of its face it is, above the section's own tag.
const OCCURRENCE: u32 = 26;
/// A convex corner's fan as `Loop::starts` tags it: the face before it, with this bit.
pub const FAN: u32 = 1 << 31;

fn unit(a: V) -> Result<V,String> {
    let n = norm(a);
    if !(n > 0.) || !n.is_finite() { return Err("degenerate direction".into()) }
    Ok(a.map(|x| x/n))
}

/// What a piece is a stretch of: a step of a revolved section, edge `k` of a prism profile's
/// loop `l`, or a segment of a prism sectioned across (`Across`): from `a` to `b`, its outward
/// normal the same all along.
#[derive(Clone)]
enum Source { Meridian(Step),Edge(usize,usize),Segment { a: V,b: V,normal: V } }

/// Where a loop's station cuts the cutter: the half-plane of the axis towards a side, or the plane
/// square to a prism's extrusion at a distance along it.
#[derive(Clone,Copy)]
enum Cut { Side(V),Offset(f64) }

/// One piece of a section loop: what it is a stretch of, its face (a tag the same at every
/// station), and its points along the walk with their cumulative chord length (a drafted prism's,
/// its profile's: `Cutter::extruded_loops`).
#[derive(Clone)]
struct Piece { face: u32,reversed: bool,source: Source,points: Vec<V>,lengths: Vec<f64> }
impl Piece {
    fn length(&self) -> f64 { *self.lengths.last().unwrap() }
    /// The step's own parameter at arc length `s` along the walk.
    fn parameter(&self,s: f64) -> f64 {
        let n = self.lengths.len()-1;
        let s = s.clamp(0.,self.length());
        let i = self.lengths.partition_point(|l| *l < s).clamp(1,n);
        let (l0,l1) = (self.lengths[i-1],self.lengths[i]);
        let f = if l1 > l0 { (s-l0)/(l1-l0) } else { 0. };
        let t = (i as f64-1.+f)/n as f64;
        if self.reversed { 1.-t } else { t }
    }
}

/// A corner after a piece: where, the outward normals either side, and the turning between them
/// where it is convex (zero otherwise, and where the loop closes through the axis).
#[derive(Clone)]
struct Corner { position: V,normals: [V;2],angle: f64 }

/// One section loop at a station: pieces in walk order, corner `k` after piece `k` (cyclically),
/// cut where `cut` says, a radian of a corner's fan walked as `turn` millimetres.
#[derive(Clone)]
pub struct Loop { pieces: Vec<Piece>,corners: Vec<Corner>,cut: Cut,turn: f64 }

/// Where a place on a loop is: on a piece at arc length, or in a corner's fan.
enum Place { Piece {index: usize,s: f64},Fan {index: usize,fraction: f64} }

impl Loop {
    pub fn augmented_length(&self) -> f64 {
        self.pieces.iter().map(Piece::length).sum::<f64>()+self.corners.iter().map(|c| c.angle*self.turn).sum::<f64>()
    }
    fn start_of(&self,index: usize) -> f64 { (0..index).map(|k| self.pieces[k].length()+self.corners[k].angle*self.turn).sum() }
    fn locate(&self,s: f64) -> Place {
        let mut s = s.rem_euclid(self.augmented_length());
        for (k,piece) in self.pieces.iter().enumerate() {
            if s <= piece.length() { return Place::Piece {index:k,s} }
            s -= piece.length();
            let fan = self.corners[k].angle*self.turn;
            if s <= fan { return Place::Fan {index:k,fraction:if fan > 0. { s/fan } else { 0. }} }
            s -= fan;
        }
        Place::Piece {index:self.pieces.len()-1,s:self.pieces.last().unwrap().length()}
    }
    pub fn index_of(&self,face: u32) -> Option<usize> { self.pieces.iter().position(|p| p.face == face) }
    /// The loop with each corner's fan walked `unit` times as long (`TURN_SCALE` is a cut's at
    /// unit one).
    pub fn scaled(mut self,unit: f64) -> Loop { self.turn *= unit; self }
    /// The length of the pieces a reach's walk runs over, from its start face's to its end face's
    /// in loop order, each whole and no fan counted: the cut's own length (`Tracer::unit`), which
    /// a cutter scaled scales, and which moves with no fraction of a face a sampling reads.
    pub fn walked(&self,reach: &Reach) -> Result<f64,String> {
        let missing = "a profile face is missing at this station";
        let (first,last) = (self.index_of(reach.start.face).ok_or(missing)?,self.index_of(reach.end.face).ok_or(missing)?);
        let n = self.pieces.len();
        Ok((0..=(last+n-first)%n).map(|k| self.pieces[(first+k)%n].length()).sum())
    }
    /// The walk lengths the reach asks at: `per_piece` places evenly within each piece, and the
    /// middle of each convex corner's fan.
    fn places(&self,per_piece: usize) -> Vec<f64> {
        let mut out = Vec::new();
        for (k,piece) in self.pieces.iter().enumerate() {
            let start = self.start_of(k);
            out.extend((0..per_piece).map(|j| start+piece.length()*(j as f64+0.5)/per_piece as f64));
            let fan = self.corners[k].angle*self.turn;
            if fan > 0. { out.push(start+piece.length()+0.5*fan); }
        }
        out
    }
    /// Where along the walk each piece starts, and each convex corner's fan (`FAN` and the face
    /// before it), with its face: what a contact at a walk length was made by (#212).
    pub fn starts(&self) -> Vec<(f64,u32)> {
        let mut out = Vec::new();
        for k in 0..self.pieces.len() {
            let start = self.start_of(k);
            out.push((start,self.pieces[k].face));
            if self.corners[k].angle > 0. { out.push((start+self.pieces[k].length(),FAN | self.pieces[k].face)); }
        }
        out
    }
    /// Whether the corner after each piece is convex: where the walk turns left, carrying a fan.
    pub fn convex(&self) -> Vec<bool> { self.corners.iter().map(|c| c.angle > 0.).collect() }
    fn resolve(&self,anchor: Anchor) -> Result<f64,String> {
        let index = self.index_of(anchor.face).ok_or("a profile face is missing at this station")?;
        Ok(self.start_of(index)+anchor.fraction.clamp(0.,1.)*self.pieces[index].length())
    }
}

/// A place on the profile that survives its changing shape between stations: a face and a
/// fraction of its piece.
#[derive(Clone,Copy,Debug)]
pub struct Anchor { pub face: u32,fraction: f64 }

/// The band and walk chosen from where the declared roll's contacts enter the blank: station
/// angles (or distances), the walk's anchors in loop order, and the millimetres of space one unit
/// of station is worth across the band (the profile's mean distance from the axis; 1 along a
/// prism), for the columns' spacing, with where the search's time went (sections, contacts, blank
/// queries) and how many points it asked the blank about.
pub struct Reach { pub stations: [f64;2],pub start: Anchor,pub end: Anchor,pub faces: usize,pub radius: f64,
    pub spent: [f64;3],pub queried: usize }

/// A prism as its stations section it: its profile's loops walked in its plane (outer
/// counter-clockwise about the extrusion), and the extrusion's extent from that plane. A drafted
/// one's (#684 G) also what `Drafted` keeps.
struct Extruded { walks: Vec<Vec<Seg>>,extent: [f64;2],drafted: Option<Drafted> }

/// A drafted prism's halves, one each side of its parting plane that it runs into (#684 P), or the
/// one: each from one ordinate to the other, with its sections there in the profile's plane, each
/// walk step for step its profile's. Between them a side's point at a fraction of its edge moves
/// along a line, so a section between is the two's blend (`build::offset` builds only such
/// corners). Where there are two the sides fold between them at the plane, and the stations cross
/// that crease's fan over `fan` (`CREASE_SCALE` a radian of it): a station is the ordinate below
/// the plane, and `fan` more than the ordinate above it. And each step's cumulative chord lengths
/// along the profile itself, the same at every station.
struct Drafted { halves: Vec<Half>,fan: f64,lengths: Vec<Vec<Vec<f64>>> }

/// One half of a drafted prism: its ordinates and its sections at them.
struct Half { heights: [f64;2],ends: [Vec<Vec<Seg>>;2] }

/// Where a drafted prism's station is: in a half at an ordinate, or across the crease at the
/// parting plane, a fraction of the way through its fan.
enum Ordinate { Half(usize,f64),Fan(f64) }

impl Drafted {
    fn ordinate(&self,station: f64) -> Ordinate {
        if self.halves.len() == 1 || station <= 0. { return Ordinate::Half(0,station) }
        if station >= self.fan { Ordinate::Half(1,station-self.fan) } else { Ordinate::Fan(station/self.fan) }
    }
}

/// The cumulative chord lengths along `points`, from 0.
fn chords(points: &[V]) -> Vec<f64> {
    let mut lengths = vec![0.];
    for w in points.windows(2) { lengths.push(lengths.last().unwrap()+distance(w[0],w[1])); }
    lengths
}

/// Step `seg`'s point at fraction `t` of it.
fn point_at(seg: &Seg,t: f64) -> V { seg.curve.point(seg.t[0]+(seg.t[1]-seg.t[0])*t) }

impl Extruded {
    /// Edge `k` of loop `l` at fraction `t`, at station `h` along the extrusion `n` (an ordinate,
    /// but past a drafted prism's crease, `Drafted::ordinate`): its point, and the side face's
    /// outward normal (the walk's direction across `n`, since the material lies left; across the
    /// side's rise along a drafted one; across the crease, turned from the one half's to the other's).
    fn at(&self,n: V,l: usize,k: usize,h: f64,t: f64) -> (V,V) {
        let point = |walks: &[Vec<Seg>]| {
            let seg = &walks[l][k];
            let w = seg.t[0]+(seg.t[1]-seg.t[0])*t;
            (seg.curve.point(w),tangent(seg,w))
        };
        let Some(drafted) = &self.drafted else {
            let (p,d) = point(&self.walks);
            let normal = cross(d,n);
            return (std::array::from_fn(|i| p[i]+n[i]*h),scale(normal,1./norm(normal).max(1e-300)))
        };
        let side = |half: &Half,h: f64| {
            let ([near,far],[h0,h1]) = (&half.ends,half.heights);
            let ((p,d),(q,e)) = (point(near),point(far));
            let f = (h-h0)/(h1-h0);
            let normal = cross(lerp(d,e,f),add(scale(sub(q,p),1./(h1-h0)),n));
            (add(lerp(p,q,f),scale(n,h)),scale(normal,1./norm(normal).max(1e-300)))
        };
        match drafted.ordinate(h) {
            Ordinate::Half(i,h) => side(&drafted.halves[i],h),
            Ordinate::Fan(f) => {
                let ((p,a),(_,b)) = (side(&drafted.halves[0],0.),side(&drafted.halves[1],0.));
                let normal = lerp(a,b,f);
                (p,scale(normal,1./norm(normal).max(1e-300)))
            }
        }
    }

    /// The stations its sections are read at: its extent, and a drafted prism's crease's fan.
    fn stations(&self) -> [f64;2] { [self.extent[0],self.extent[1]+self.drafted.as_ref().map_or(0.,|d| d.fan)] }
}

/// A prism whose cap reaches the blank, sectioned through its extrusion (#684 F): a station is a
/// distance along `across` (a unit direction in the profile's plane) from the profile's origin, and
/// its section the plane square to `across` there: the profile's chord carried along the
/// extrusion, a ruling of the side at each end of the chord and the chord on each cap, four pieces
/// whose corners are the caps' rims, each a quarter turn of fan — one such loop for each chord where
/// a hole parts the section (#721 F1d); drafted, each side a ruling of each half, creased where they
/// meet. Its stretches along the extrusion and their end sections (`Ends`), walked counter-clockwise
/// about the extrusion; `across`; and the stations its sections are one family over. Each chord on
/// a cap is walked as its `chords` millimetres (the low cap's, the high cap's) whatever its length
/// at its station, so that a station's pieces start at the same walk lengths as its neighbours'
/// where the profile tapers: the sheet's rows are laid at walk lengths common to every column
/// (`contact_trace::Station`).
struct Across { halves: Vec<Ends>,across: V,range: [f64;2],chords: Vec<[f64;2]> }

/// A stretch of a prism along its extrusion whose sides run straight from one end's section to the
/// other's: its heights, and its sections there in the profile's plane, walk for walk and step for
/// step alike — an undrafted prism's one stretch, its profile at both ends; a drafted prism's of
/// lines each half (#721 F1d).
type Ends = ([f64;2],[Vec<Vec<Seg>>;2]);

/// A side's half above a drafted prism's parting plane, as its tag in a section across carries it.
const UPPER_HALF: u32 = 1 << 15;

/// A cap's tag in a section across (`Across`), the low cap's along the extrusion and the high's:
/// a side's tag is its edge's, `(loop + 1) << 16 | edge`, so these name no side.
const CAP_LOW: u32 = 1;
const CAP_HIGH: u32 = 2;

/// A section across `walk`'s plane by the plane `across·(x − origin) = c`: where it meets the
/// profile, each the edge it is on, its parameter there and the point (a point where two edges
/// meet counted once).
fn meets(walk: &[Seg],origin: V,across: V,c: f64) -> Vec<(usize,f64,V)> {
    let size = walk.iter().map(|s| distance(s.start(),origin)).fold(1_f64,f64::max);
    let close = crate::geom::tolerance(size);
    let mut out: Vec<(usize,f64,V)> = Vec::new();
    for (k,seg) in walk.iter().enumerate() {
        let [lo,hi] = seg.span();
        let tol = 1e-12*(1.+hi.abs()+lo.abs());
        let mut at = |w: f64| {
            let p = seg.curve.point(w);
            if !out.iter().any(|(_,_,q)| distance(*q,p) <= close) { out.push((k,w,p)); }
        };
        match &seg.curve {
            crate::geom::Curve::Line {p,d} => {
                let rate = dot(across,*d);
                if rate.abs() <= 1e-12 { continue }
                let w = (c-dot(across,sub(*p,origin)))/rate;
                if w >= lo-tol && w <= hi+tol { at(w.clamp(lo,hi)); }
            }
            crate::geom::Curve::Circle(f,r) => {
                let (a,b) = (r*dot(across,f.x),r*dot(across,f.y));
                let amplitude = a.dhypot(b);
                if !(amplitude > 0.) { continue }
                let cosine = (c-dot(across,sub(f.o,origin)))/amplitude;
                if cosine.abs() > 1. { continue }
                let (phase,turn) = (b.datan2(a),cosine.dacos());
                for w in [phase+turn,phase-turn] {
                    let w = crate::geom::around(w,lo,TAU);
                    let w = if w > hi+tol && (w-TAU-lo).abs() <= tol { lo } else { w };
                    if w <= hi+tol { at(w.min(hi)); }
                }
            }
            _ => {}
        }
    }
    out
}

/// A loop's least and greatest along `across` as `at` reads it, each with its point: at its edges'
/// ends, and where an arc turns along `across`.
fn extremes(walk: &[Seg],at: &dyn Fn(V) -> f64,across: V) -> [(f64,V);2] {
    let (mut lo,mut hi) = ((f64::INFINITY,[0.;3]),(f64::NEG_INFINITY,[0.;3]));
    let mut reach = |p: V| { let c = at(p); if c < lo.0 { lo = (c,p) } if c > hi.0 { hi = (c,p) } };
    for seg in walk {
        reach(seg.start());
        if let crate::geom::Curve::Circle(f,_) = &seg.curve {
            for (w,_) in crate::geom::turns(dot(across,f.x),dot(across,f.y),seg.span()) { reach(seg.curve.point(w)); }
        }
    }
    [lo,hi]
}

/// The tangent of a walked edge at its parameter `w`, in the walk's direction.
fn tangent(seg: &Seg,w: f64) -> V { let (_,d,_) = seg.curve.d2(w); scale(d,(seg.t[1]-seg.t[0]).signum()) }

/// Where a prism's sections across a direction cannot be one regular family (#684 F): the least and
/// greatest distance of its profile along the direction (where a section closes to a ruling of a
/// side), and each corner of its profile — where two edges meet at an angle, so a section through
/// it leaps from one side's normal to the other's — at its distance and its point (millimetres);
/// and the direction, made square to the extrusion and unit.
#[derive(Clone,Debug)]
pub struct Spread { pub across: V,pub range: [f64;2],pub corners: Vec<(f64,V)> }

/// What the stations section.
/// A cutter of revolutions' stations are angles from `side` about its axis, and the grid its band
/// is read on starts `phase` past where its contacts first reach the blank (`Cutter::reach`).
enum Kind { Revolved { side: V,sec: Sectioned,phase: f64 },Extruded(Extruded),Across(Across) }

/// A cutter as its stations section it: about `axis` through `origin` (a prism's extrusion from a
/// point of its profile's plane).
pub struct Cutter { pub origin: V,pub axis: V,kind: Kind,sections: std::sync::Mutex<BTreeMap<u64,Vec<Loop>>> }

impl std::fmt::Debug for Cutter {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let kind = match self.kind { Kind::Revolved {..} => "revolved",Kind::Extruded(_) => "extruded",Kind::Across(_) => "across" };
        write!(f,"Cutter {{ {kind}, origin: {:?}, axis: {:?} }}",self.origin,self.axis)
    }
}

impl Cutter {
    /// **The cutter of revolutions about parallel lines its B-rep `b` (millimetres) is** (#721 F1d):
    /// its section by every half-plane of the first line read from its faces
    /// (`Sectioned::of_boundary`), however it was drawn, or why it is not one. (No recipe is read:
    /// a prism's cutter is its B-rep's too, `admission::Lone::read`.)
    pub fn of_boundary(b: &crate::topo::Brep) -> Result<Cutter,String> {
        let sec = Sectioned::of_boundary(b)?;
        let (origin,axis) = (sec.origin,unit(sec.axis)?);
        // the half-plane its stations are counted from; the grid its band is read on is the reach's
        // own, not this one's (`reach`)
        let seed = if axis[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let side = unit(cross(cross(axis,seed),axis))?;
        Ok(Cutter {origin,axis,kind:Kind::Revolved {side,sec,phase:0.},sections:Default::default()})
    }
    /// This cutter of revolutions with the grid its band is read on (`reach`) turned `angle` further
    /// round its axis: the same cutter, its stations moved against it, which no sheet traced over
    /// them may depend on (#904 A4). None for a prism.
    pub fn turned(&self,angle: f64) -> Option<Cutter> {
        let Kind::Revolved {side,sec,phase} = &self.kind else { return None };
        Some(Cutter {origin:self.origin,axis:self.axis,kind:Kind::Revolved {side:*side,sec:sec.clone(),phase:phase+angle},
            sections:Default::default()})
    }
    /// The prism of `profile` (millimetres) from `from` to `to` along its normal, drafted `draft`
    /// where it is, as its sections: what a tool's B-rep read as one prism standing alone is
    /// (`admission::Lone::read`, #721 F1b, F1d).
    pub fn extruded(profile: Profile,from: f64,to: f64,draft: Option<f64>) -> Result<Cutter,String> {
        let axis = unit(profile.normal)?;
        let plane = Frame::about(profile.origin,axis);
        let coords = |q: V| { let l = plane.local(q); [l[0],l[1]] };
        let walks = walks_of(&profile,&coords)?;
        if walks.iter().flatten().any(|s| matches!(s.curve,crate::geom::Curve::BSpline(_))) {
            return Err("the core sections a prism of lines and arcs only".into())
        }
        let extent = [from.min(to),from.max(to)];
        let draft = draft.unwrap_or(0.);
        let drafted = if draft == 0. { None } else {
            let parted = extent[0] < 0. && extent[1] > 0.;
            // admission refuses such a tool at T1 (`ExtrudedSurface::drafted`); the kernel says so too
            if parted && draft < 0. {
                return Err("the core sections no drafted prism whose sides meet at its parting plane in a concave crease".into())
            }
            let taper = draft.dtan();
            // the step of a moved walk that is `s`, walked the same way
            let same = |moved: &[Seg],s: &Seg| moved.iter().find(|m| m.source == s.source && m.forward() == s.forward()).cloned()
                .ok_or_else(|| "a drafted section's walk is not its profile's".to_string());
            // each end's section walked, its steps put in the profile's walks' order
            let end = |h: f64| -> Result<Vec<Vec<Seg>>,String> {
                let moved = walks_of(&offset(&profile,h.abs()*taper)?,&coords)?;
                walks.iter().zip(moved).map(|(walk,moved)| walk.iter().map(|s| same(&moved,s)).collect()).collect()
            };
            let lengths = walks.iter().map(|walk| walk.iter().map(|s| {
                chords(&(0..=SAMPLES_PER_PIECE).map(|j| point_at(s,j as f64/SAMPLES_PER_PIECE as f64)).collect::<Vec<_>>())
            }).collect()).collect();
            // one half, or two meeting at the parting plane, where the profile is their section
            let heights = if parted { vec![[extent[0],0.],[0.,extent[1]]] } else { vec![extent] };
            let halves = heights.into_iter().map(|[h0,h1]| Ok(Half {heights:[h0,h1],ends:[end(h0)?,end(h1)?]}))
                .collect::<Result<_,String>>()?;
            Some(Drafted {halves,fan:if parted { 2.*draft*CREASE_SCALE } else { 0. },lengths})
        };
        Ok(Cutter {origin:profile.origin,axis,kind:Kind::Extruded(Extruded {walks,extent,drafted}),
            sections:Default::default()})
    }
    /// A prism's profile, its loops walked in its plane (outer counter-clockwise about the axis),
    /// and the extrusion's extent from that plane, where its section is the same at every station;
    /// none for a cutter of revolutions, a drafted prism or a prism sectioned across.
    pub fn prism_profile(&self) -> Option<(&[Vec<Seg>],[f64;2])> {
        match &self.kind { Kind::Extruded(x) if x.drafted.is_none() => Some((&x.walks,x.extent)),_ => None }
    }
    /// A prism's profile, its loops walked in its plane, the extrusion's extent from that plane, and
    /// whether it is drafted: what a prism read from a B-rep is held to (`admission::Lone`). None
    /// for a cutter of revolutions or a prism sectioned across.
    pub fn prism_walks(&self) -> Option<(&[Vec<Seg>],[f64;2],bool)> {
        match &self.kind { Kind::Extruded(x) => Some((&x.walks,x.extent,x.drafted.is_some())),_ => None }
    }
    /// Where this prism's sections across `across` (a direction in its profile's plane, made square
    /// to the extrusion and unit) cannot be one regular family (`Spread`): none for a cutter that is
    /// no undrafted prism of one loop.
    pub fn spread(&self,across: V) -> Option<Spread> {
        let ends = self.straight_ends()?;
        let across = unit(sub(across,scale(self.axis,dot(across,self.axis)))).ok()?;
        let at = |p: V| dot(across,sub(p,self.origin));
        // the stations every end section reaches; each loop's least and greatest along `across`
        // where it is a hole's, and each corner, of every end section
        let mut range = [f64::NEG_INFINITY,f64::INFINITY];
        let mut corners = Vec::new();
        for walks in ends.iter().flat_map(|(_,w)| w.iter()) {
            let mut reach = [f64::INFINITY,f64::NEG_INFINITY];
            for walk in walks {
                let [lo,hi] = extremes(walk,&at,across);
                reach = [reach[0].min(lo.0),reach[1].max(hi.0)];
                if walks.len() > 1 { corners.extend([lo,hi]) }
                corners.extend((0..walk.len()).filter_map(|k| {
                    let (a,b) = (&walk[k],&walk[(k+1)%walk.len()]);
                    let (ta,tb) = (unit(tangent(a,a.t[1])).ok()?,unit(tangent(b,b.t[0])).ok()?);
                    (dot(ta,tb).clamp(-1.,1.).dacos() > 1e-6).then(|| (at(a.end()),a.end()))
                }));
            }
            range = [range[0].max(reach[0]),range[1].min(reach[1])];
        }
        if !(range[0] < range[1]) { return None }
        // (an outline's own least and greatest are the range's ends, read there)
        corners.retain(|&(c,_)| c > range[0] && c < range[1]);
        Some(Spread {across,range,corners})
    }
    /// The prism's stretches along its extrusion whose sides run straight between their end
    /// sections (`Ends`): an undrafted prism's one, its profile at both ends; a drafted one's halves,
    /// where every edge of its profile is a line (a drafted arc's point moves along a conic, whose
    /// sections across are not built). None for any other cutter.
    fn straight_ends(&self) -> Option<Vec<Ends>> {
        let Kind::Extruded(x) = &self.kind else { return None };
        match &x.drafted {
            None => Some(vec![(x.extent,[x.walks.clone(),x.walks.clone()])]),
            Some(d) if x.walks.iter().flatten().all(|s| matches!(s.curve,crate::geom::Curve::Line {..})) =>
                Some(d.halves.iter().map(|h| (h.heights,h.ends.clone())).collect()),
            Some(_) => None,
        }
    }
    /// This prism sectioned across `across` (`Across`, #684 F): its stations distances along it,
    /// each section its chords carried along the extrusion, one a hole parts the section at each,
    /// every chord walked as long as it is at the station `reference` (the middle of the band the
    /// sheet spans). A prism undrafted, or drafted with sides of lines (#721 F1d).
    pub fn across(&self,across: V,reference: f64) -> Result<Cutter,String> {
        let Spread {across,range,..} = self.spread(across)
            .ok_or("a prism is sectioned across only undrafted, or drafted with sides of lines")?;
        let halves = self.straight_ends().expect("a spread is such a prism's");
        // the stations run between the nearest ends of the holes either side of `reference`, where
        // the section's chords part or join, as they run between the outline's ends
        let mut range = range;
        for walk in halves.iter().flat_map(|(_,w)| w.iter()).flatten() {
            let [(lo,_),(hi,_)] = extremes(walk,&|p| dot(across,sub(p,self.origin)),across);
            if !(lo > range[0] && hi < range[1]) { continue }
            if lo >= reference { range[1] = range[1].min(lo) } else if hi <= reference { range[0] = range[0].max(hi) }
            else { return Err(format!("the prism's section {reference:.4} mm across its profile passes through a hole")) }
        }
        let (first,last) = (&halves[0].1[0],&halves[halves.len()-1].1[1]);
        let (low,high) = (Self::chords(first,self.origin,self.axis,across,reference)?,Self::chords(last,self.origin,self.axis,across,reference)?);
        if low.len() != high.len() { return Err(format!("the prism's caps' sections {reference:.4} mm across its profile differ")) }
        let chords = low.iter().zip(&high).map(|(a,b)| [distance(a[0].2,a[1].2),distance(b[0].2,b[1].2)]).collect();
        Ok(Cutter {origin:self.origin,axis:self.axis,kind:Kind::Across(Across {halves,across,range,chords}),sections:Default::default()})
    }
    /// Where the plane `across·(x − origin) = c` meets `walks`' loops, paired into the chords the
    /// prism's material spans, in order along the extrusion crossed with `across`: each end its
    /// loop's side's tag (`(loop + 1) << 16 | edge`), its parameter and its point.
    fn chords(walks: &[Vec<Seg>],origin: V,axis: V,across: V,c: f64) -> Result<Vec<[(u32,f64,V);2]>,String> {
        let mut found: Vec<(u32,f64,V)> = walks.iter().enumerate()
            .flat_map(|(l,walk)| meets(walk,origin,across,c).into_iter().map(move |(k,w,p)| ((((l+1) as u32) << 16) | k as u32,w,p)))
            .collect();
        if found.is_empty() || found.len()%2 == 1 {
            return Err(format!("the prism's section {c:.4} mm across its profile meets it at {} points, no ends of chords",found.len()))
        }
        let along = cross(axis,across);
        found.sort_by(|a,b| dot(along,a.2).total_cmp(&dot(along,b.2)));
        Ok(found.chunks(2).map(|w| [w[0],w[1]]).collect())
    }
    /// The face a piece's tag (`Loop::starts`) is a stretch of: its recipe node's name and its
    /// own (#212), whichever meeting of it; none for a fan, which is no face of the cutter's, or a
    /// prism's side (its walk's order is not its profile's).
    pub fn face_of(&self,tag: u32) -> Option<(String,String)> {
        if tag & FAN != 0 { return None }
        match &self.kind { Kind::Revolved {sec,..} => sec.face_of(tag & ((1 << OCCURRENCE)-1)),_ => None }
    }
    /// Whether a station is an angle about the axis (else a distance along a prism).
    pub fn revolved(&self) -> bool { matches!(self.kind,Kind::Revolved {..}) }
    /// How far the band of stations may run before the sheet is said not to leave the blank: a
    /// whole turn about the axis, or a prism's own length (past it a cap would be in the blank).
    pub fn station_limit(&self) -> f64 {
        match &self.kind { Kind::Revolved {..} => TAU,Kind::Extruded(x) => { let [lo,hi] = x.stations(); hi-lo },
            Kind::Across(x) => x.range[1]-x.range[0] }
    }
    /// A piece's point at fraction `t` of its source, and the cutter's outward normal there.
    fn at(&self,cut: Cut,source: &Source,t: f64) -> (V,V) {
        match (&self.kind,cut,source) {
            (Kind::Revolved {sec,..},Cut::Side(side),Source::Meridian(step)) => sec.at(side,step,t),
            (Kind::Extruded(x),Cut::Offset(h),&Source::Edge(l,k)) => x.at(self.axis,l,k,h,t),
            (Kind::Across(_),_,&Source::Segment {a,b,normal}) => (lerp(a,b,t),normal),
            _ => unreachable!("a piece is cut as its cutter is"),
        }
    }
    fn side_at(side: V,axis: V,angle: f64) -> V {
        let other = cross(axis,side);
        std::array::from_fn(|k| side[k]*angle.dcos()+other[k]*angle.dsin())
    }
    /// The section loops at the station `angle`, each with its pieces oriented as the walk goes and
    /// its corners' fans (kept: the stations a sheet is traced at are asked for repeatedly).
    pub fn profile(&self,angle: f64) -> Result<Vec<Loop>,String> {
        if let Some(loops) = self.sections.lock().unwrap_or_else(|e| e.into_inner()).get(&angle.to_bits()) { return Ok(loops.clone()) }
        let loops = match &self.kind {
            Kind::Revolved {side,sec,..} => self.loops(sec,Cutter::side_at(*side,self.axis,angle))?,
            Kind::Extruded(x) => self.extruded_loops(x,angle),
            Kind::Across(x) => self.across_loops(x,angle)?,
        };
        self.sections.lock().unwrap_or_else(|e| e.into_inner()).insert(angle.to_bits(),loops.clone());
        Ok(loops)
    }
    /// A prism's section at `h` along its extrusion: its profile, each edge a face. A drafted one's
    /// is walked by its profile's lengths (`Drafted::lengths`), not its own: a walk length is then
    /// the same fraction of each edge at every station, a ruling of each side, as it is along a prism
    /// whose section is the same at every station (the sheet's rows are walk lengths read across the
    /// stations; a revolution's sections, which change with the station, are walked by their own
    /// lengths and their anchors warped across the band instead, `contact_trace::Anchors`).
    fn extruded_loops(&self,x: &Extruded,h: f64) -> Vec<Loop> {
        x.walks.iter().enumerate().map(|(l,walk)| {
            let cut = Cut::Offset(h);
            let pieces: Vec<Piece> = (0..walk.len()).map(|k| {
                let source = Source::Edge(l,k);
                let points: Vec<V> = (0..=SAMPLES_PER_PIECE)
                    .map(|j| self.at(cut,&source,j as f64/SAMPLES_PER_PIECE as f64).0).collect();
                let lengths = match &x.drafted { Some(d) => d.lengths[l][k].clone(),None => chords(&points) };
                Piece {face:((l as u32+1) << 16) | k as u32,reversed:false,source,points,lengths}
            }).collect();
            // convex where the walk turns left about the extrusion: the material is on its left
            let corners = (0..pieces.len()).map(|k| {
                let next = &pieces[(k+1)%pieces.len()];
                let position = *pieces[k].points.last().unwrap();
                let (a,b) = (self.at(cut,&pieces[k].source,1.).1,self.at(cut,&next.source,0.).1);
                let angle = dot(a,b).clamp(-1.,1.).dacos();
                let convex = angle > 1e-6 && dot(cross(a,b),self.axis) > 0.;
                Corner {position,normals:[a,b],angle:if convex { angle } else { 0. }}
            }).collect();
            Loop {pieces,corners,cut,turn:TURN_SCALE}
        }).collect()
    }
    /// A prism's section across at `c` along `across` (`Across`): the low cap's chord, the ruling
    /// at the chord's far end (along `along`), the high cap's chord back and the ruling at its near
    /// end, each corner a cap's rim and convex. Refused where the plane does not cut the profile in
    /// one chord.
    fn across_loops(&self,x: &Across,c: f64) -> Result<Vec<Loop>,String> {
        // each half's chords at its two ends, alike in number and in the sides they end on
        let ends: Vec<[Vec<[(u32,f64,V);2]>;2]> = x.halves.iter().map(|(_,w)| Ok([Self::chords(&w[0],self.origin,self.axis,x.across,c)?,
            Self::chords(&w[1],self.origin,self.axis,x.across,c)?])).collect::<Result<_,String>>()?;
        let alike = |a: &[[(u32,f64,V);2]],b: &[[(u32,f64,V);2]]| a.len() == b.len() && a.iter().zip(b).all(|(p,q)| p[0].0 == q[0].0 && p[1].0 == q[1].0);
        if ends.iter().any(|e| e[0].len() != x.chords.len() || !alike(&e[0],&e[1])) {
            return Err(format!("the prism's section {c:.4} mm across its profile is not the {} chords of its band",x.chords.len()))
        }
        let lift = |p: V,h: f64| add(p,scale(self.axis,h));
        let edge = |tag: u32,half: usize| &x.halves[half].1[0][((tag >> 16)-1) as usize][(tag & 0x7fff) as usize];
        let n = x.halves.len();
        let cut = Cut::Offset(c);
        (0..x.chords.len()).map(|i| {
            // each side's rulings up the halves at chord end `e`: its tag, its ends, its normal (the
            // edge's tangent crossed with the ruling up, outward as the walk turns counter-clockwise)
            let side = |e: usize,half: usize| -> Result<(u32,V,V,V),String> {
                let ([h0,h1],_) = x.halves[half];
                let ((tag,w,p),(_,_,q)) = (ends[half][0][i][e],ends[half][1][i][e]);
                let (a,b) = (lift(p,h0),lift(q,h1));
                let up = unit(sub(b,a))?;
                let tag = if half == 1 { tag | UPPER_HALF } else { tag };
                Ok((tag,a,b,unit(cross(tangent(edge(tag & !UPPER_HALF,half),w),up))?))
            };
            let (near,far): (Vec<_>,Vec<_>) = ((0..n).map(|h| side(0,h)).collect::<Result<_,_>>()?,(0..n).map(|h| side(1,h)).collect::<Result<_,_>>()?);
            // the low cap's chord, the far side up, the high cap's chord, the near side down: each
            // piece its face, its ends, its normal and its walked length (a chord the reference's)
            let mut segments: Vec<(u32,V,V,V,f64)> = vec![(CAP_LOW,near[0].1,far[0].1,scale(self.axis,-1.),x.chords[i][0])];
            segments.extend(far.iter().map(|&(tag,a,b,normal)| (tag,a,b,normal,distance(a,b))));
            segments.push((CAP_HIGH,far[n-1].2,near[n-1].2,self.axis,x.chords[i][1]));
            segments.extend(near.iter().rev().map(|&(tag,a,b,normal)| (tag,b,a,normal,distance(a,b))));
            if segments.iter().any(|s| !(distance(s.1,s.2) > 0.)) { return Err(format!("the prism's section {c:.4} mm across its profile has no width")) }
            let pieces: Vec<Piece> = segments.iter().map(|&(face,a,b,normal,walked)| {
                let points: Vec<V> = (0..=SAMPLES_PER_PIECE).map(|j| lerp(a,b,j as f64/SAMPLES_PER_PIECE as f64)).collect();
                let lengths = (0..=SAMPLES_PER_PIECE).map(|j| walked*j as f64/SAMPLES_PER_PIECE as f64).collect();
                Piece {face,reversed:false,source:Source::Segment {a,b,normal},points,lengths}
            }).collect();
            let m = segments.len();
            let corners = (0..m).map(|k| {
                let (a,b) = (segments[k].3,segments[(k+1)%m].3);
                Corner {position:segments[k].2,normals:[a,b],angle:dot(a,b).clamp(-1.,1.).dacos()}
            }).collect();
            Ok(Loop {pieces,corners,cut,turn:RIM_SCALE})
        }).collect()
    }
    fn loops(&self,sec: &Sectioned,side: V) -> Result<Vec<Loop>,String> {
        let region = sec.section(side)?;
        let cut = Cut::Side(side);
        let plane_normal = unit(cross(self.axis,side))?;
        let in_plane = |p: V| { let d = sub(p,self.origin); [dot(d,side),dot(d,self.axis)] };
        let on_axis = |p: [f64;2]| p[0].abs() <= ON_AXIS;
        let mut loops = Vec::new();
        for l in &region.loops {
            // a loop closing through the axis (a solid touching it) opens there: no face bounds it
            let mut steps: Vec<Step> = l.clone();
            let axis_closed = if let Some(k) = steps.iter().position(|s| on_axis(s.start()) && on_axis(s.end()) && on_axis(s.point(0.5))) {
                steps.rotate_left(k+1);
                steps.pop();
                true
            } else { false };
            let mut pieces: Vec<Piece> = Vec::new();
            let mut area = 0.;
            for step in &steps {
                let mut points = Vec::with_capacity(SAMPLES_PER_PIECE+1);
                let mut lengths = vec![0.];
                for j in 0..=SAMPLES_PER_PIECE {
                    let p = sec.at(side,step,j as f64/SAMPLES_PER_PIECE as f64).0;
                    if let Some(&last) = points.last() {
                        lengths.push(lengths.last().unwrap()+distance(p,last));
                        area += dot(plane_normal,cross(sub(last,self.origin),sub(p,self.origin)));
                    }
                    points.push(p);
                }
                pieces.push(Piece {face:step.tag,reversed:false,source:Source::Meridian(*step),points,lengths});
            }
            if area < 0. { reverse(&mut pieces); }
            // a face the section meets twice (a revolution about another line, which the half-plane
            // cuts in two) has a piece for each meeting: told apart by their order along the walk
            // from a canonical start — the lowest face met once, where the loop closes round (one
            // through the axis starts there already) — so neighbouring stations name them alike
            if pieces.iter().enumerate().any(|(i,p)| pieces[..i].iter().any(|q| q.face == p.face)) {
                let once = |f: u32| pieces.iter().filter(|p| p.face == f).count() == 1;
                if !axis_closed {
                    if let Some(k) = (0..pieces.len()).filter(|&k| once(pieces[k].face)).min_by_key(|&k| pieces[k].face) {
                        pieces.rotate_left(k);
                    }
                }
                let mut seen: BTreeMap<u32,u32> = BTreeMap::new();
                for piece in &mut pieces {
                    let n = seen.entry(piece.face).or_insert(0);
                    piece.face |= *n << OCCURRENCE;
                    *n += 1;
                }
            }
            let mut corners = Vec::new();
            for k in 0..pieces.len() {
                let next = (k+1)%pieces.len();
                let position = *pieces[k].points.last().unwrap();
                let normal = |p: &Piece,at_end: bool| { let t = if p.reversed != at_end { 1. } else { 0. }; self.at(cut,&p.source,t).1 };
                let a = normal(&pieces[k],true);
                if axis_closed && next == 0 { corners.push(Corner {position,normals:[a,a],angle:0.}); continue }
                let b = normal(&pieces[next],false);
                let angle = dot(a,b).clamp(-1.,1.).dacos();
                // convex where the bisector leaves the cutter: a concave corner's normals turn
                // through the material and sweep no exposed face
                let convex = angle > 1e-6 && {
                    let bisector = unit(std::array::from_fn(|i| a[i]+b[i]))?;
                    !region.contains(in_plane(std::array::from_fn(|i| position[i]+bisector[i]*1e-3)))
                };
                corners.push(Corner {position,normals:[a,b],angle:if convex { angle } else { 0. }});
            }
            loops.push(Loop {pieces,corners,cut,turn:TURN_SCALE});
        }
        Ok(loops)
    }
    /// The cutter's point and outward normal at augmented length `s` of a section loop. A fan whose
    /// normals cancel carries no direction: no one contact time there, like a degenerate root.
    pub fn sample(&self,profile: &Loop,s: f64) -> Result<Sample,TraceError> {
        match profile.locate(s) {
            Place::Piece {index,s} => {
                let piece = &profile.pieces[index];
                let (position,normal) = self.at(profile.cut,&piece.source,piece.parameter(s));
                Ok(Sample {position,normal})
            }
            Place::Fan {index,fraction} => {
                let corner = &profile.corners[index];
                let [a,b] = corner.normals;
                let normal = unit(std::array::from_fn(|i| (1.-fraction)*a[i]+fraction*b[i])).map_err(TraceError::Degenerate)?;
                Ok(Sample {position:corner.position,normal})
            }
        }
    }
    /// The places of one section loop whose contacts within the declared roll lie inside the blank,
    /// each its walk length and its sample, read at `REACH_SAMPLES` places a piece; what was spent
    /// sampling and asking the blank, and how many points it was asked, added to `spent` and
    /// `queried`.
    fn reached(&self,tracer: &Tracer,profile: &Loop,spent: &mut [f64;3],queried: &mut usize) -> Result<Vec<(f64,Sample)>,String> {
        let (chart,mm,inside) = (tracer.chart,tracer.scale,tracer.inside);
        let declared = chart.domain();
        let mut queries = Vec::new();
        let clock = crate::util::clock::Instant::now();
        for s in profile.places(REACH_SAMPLES) {
            let sample = self.sample(profile,s)?;
            match tracer.nearest(sample,declared,0.) {
                Ok(Some((p,_))) => queries.push((s,sample,p)),
                Ok(None) => {}
                // a point whose contact equation is degenerate (a pole, in contact at every
                // time) matters only if its path enters the blank
                Err(error) => {
                    let path: Vec<V> = (0..=64).map(|k| declared[0]+(declared[1]-declared[0])*k as f64/64.)
                        .map(|t| chart.path.at(t).map(|m| scale(m.point(scale(sample.position,1./mm)),mm)).map_err(|e| e.msg))
                        .collect::<Result<_,_>>()?;
                    if inside(&path)?.iter().any(|b| *b) { return Err(error.into()) }
                }
            }
        }
        spent[1] += clock.elapsed().as_secs_f64();
        if queries.is_empty() { return Ok(Vec::new()) }
        let clock = crate::util::clock::Instant::now();
        let states = inside(&queries.iter().map(|q| q.2).collect::<Vec<_>>())?;
        spent[2] += clock.elapsed().as_secs_f64(); *queried += queries.len();
        Ok(queries.into_iter().zip(states).filter(|(_,state)| *state).map(|((s,sample,_),_)| (s,sample)).collect())
    }
    /// Whether any section loop at the station `angle` reaches the blank (`reached`).
    fn reaches_at(&self,tracer: &Tracer,angle: f64) -> Result<bool,String> {
        let (mut spent,mut queried) = ([0.;3],0);
        for profile in &self.profile(angle)? {
            if !self.reached(tracer,profile,&mut spent,&mut queried)?.is_empty() { return Ok(true) }
        }
        Ok(false)
    }
    /// **The stations and the profile walk whose declared-roll contacts lie inside the blank**,
    /// read at `STATIONS` stations. Along a prism they are spread over its extent, half a step in
    /// from either end. Round a cutter of revolutions they are its band's own (#904 A4): read first
    /// a step apart from the half-plane the cutter is counted from, which a world axis picks, and
    /// again from where the contacts first reach the blank, the station before the band's first
    /// and the first bisected to it (`START`) — so where the cutter and the blank stand, turned
    /// together, the band and every station traced across it stand alike. Counted from that
    /// half-plane, the stations the traced sheet's grid falls on moved against the cutter with a
    /// turn of the pair, and the sheet was built at some of its phases and refused at others.
    pub fn reach(&self,tracer: &Tracer) -> Result<Reach,String> {
        let Kind::Revolved {phase,..} = self.kind else { return self.reach_on(tracer,None) };
        let coarse = self.reach_on(tracer,None)?;
        let (mut miss,mut hit) = (coarse.stations[0]-TAU/STATIONS as f64,coarse.stations[0]);
        // every station reaches the blank: the band is the whole turn, its start nowhere
        if self.reaches_at(tracer,miss)? { return Ok(coarse) }
        while hit-miss > START {
            let middle = 0.5*(miss+hit);
            if self.reaches_at(tracer,middle)? { hit = middle } else { miss = middle }
        }
        self.reach_on(tracer,Some(hit+phase))
    }
    /// `reach` read at `STATIONS` stations: round the axis from `start` a step apart, or from half a
    /// step off the cutter's own half-plane where none is given; along a prism, over its extent.
    fn reach_on(&self,tracer: &Tracer,start: Option<f64>) -> Result<Reach,String> {
        let stations = STATIONS;
        // half a step in from either end, round the axis or along the prism
        let (cyclic,station) = match &self.kind {
            Kind::Revolved {..} => (true,[0.,TAU]),
            Kind::Extruded(x) => (false,x.stations()),
            Kind::Across(x) => (false,x.range),
        };
        let station = |c: f64| match start {
            Some(start) => start+(station[1]-station[0])*c/stations as f64,
            None => station[0]+(station[1]-station[0])*(c+0.5)/stations as f64,
        };
        let mut inside_stations = Vec::new();
        let mut face_hits: BTreeMap<u32,(f64,f64)> = BTreeMap::new();
        let mut order: Option<Vec<(u32,f64)>> = None;
        let (mut radius_sum,mut radius_count) = (0.,0);
        let (mut spent,mut queried) = ([0.;3],0);
        // every station's section first, on every core
        let clock = crate::util::clock::Instant::now();
        let mut sections = crate::util::par::indices(stations,|c| self.profile(station(c as f64))).into_iter();
        spent[0] += clock.elapsed().as_secs_f64();
        for c in 0..stations {
            let mut hit_here = false;
            let loops = sections.next().expect("a section a station")?;
            for profile in &loops {
                let hits = self.reached(tracer,profile,&mut spent,&mut queried)?;
                for (s,sample) in &hits {
                    let radial = sub(sample.position,self.origin);
                    radius_sum += norm(sub(radial,scale(self.axis,dot(radial,self.axis)))); radius_count += 1;
                    if let Place::Piece {index,s} = profile.locate(*s) {
                        let piece = &profile.pieces[index];
                        let f = s/piece.length();
                        let entry = face_hits.entry(piece.face).or_insert((f64::INFINITY,f64::NEG_INFINITY));
                        entry.0 = entry.0.min(f); entry.1 = entry.1.max(f);
                    }
                }
                if !hits.is_empty() {
                    if hit_here { return Err("two section loops of the cutter reach the blank at one station".into()) }
                    hit_here = true;
                    if order.is_none() { order = Some(profile.pieces.iter().map(|p| (p.face,p.length())).collect()); }
                }
            }
            if hit_here { inside_stations.push(c); }
        }
        let order = order.ok_or("no contact of the cutter within its declared roll lies inside the blank")?;
        // the station band: round the axis, the complement of the largest gap between hits; along a
        // prism, from the first hit to the last
        let (first,span) = if cyclic {
            let mut gap = (0,0);
            for (i,&c) in inside_stations.iter().enumerate() {
                let next = inside_stations[(i+1)%inside_stations.len()];
                let width = (next as i64-c as i64).rem_euclid(stations as i64);
                if width > gap.0 { gap = (width,i); }
            }
            let (first,last) = (inside_stations[(gap.1+1)%inside_stations.len()],inside_stations[gap.1]);
            (first,(last as i64-first as i64).rem_euclid(stations as i64) as f64)
        } else {
            (inside_stations[0],(inside_stations[inside_stations.len()-1]-inside_stations[0]) as f64)
        };
        let (lo,hi) = (station(first as f64),station(first as f64+span));
        // a unit of station along a prism is a millimetre
        let radius = if cyclic { radius_sum/radius_count.max(1) as f64 } else { 1. };
        // the walk: the contiguous run of hit faces in loop order, the complement of the longest
        // stretch of faces missed between two hit (by their length: a face missed, and where its
        // stretch is no longer than another's, the walk turned the wrong way round the loop)
        let hit: Vec<usize> = order.iter().enumerate().filter(|(_,f)| face_hits.contains_key(&f.0)).map(|(i,_)| i).collect();
        let n = order.len();
        let mut biggest = (-1.,0);
        for (i,&h) in hit.iter().enumerate() {
            let next = hit[(i+1)%hit.len()];
            let width = (next as i64-h as i64).rem_euclid(n as i64) as usize;
            let missed: f64 = (1..width).map(|k| order[(h+k)%n].1).sum();
            if missed > biggest.0 { biggest = (missed,i); }
        }
        let (start_face,end_face) = (order[hit[(biggest.1+1)%hit.len()]].0,order[hit[biggest.1]].0);
        Ok(Reach {stations:[lo,hi],start:Anchor {face:start_face,fraction:face_hits[&start_face].0},
            end:Anchor {face:end_face,fraction:face_hits[&end_face].1},faces:hit.len(),radius,spent,queried})
    }
    /// The augmented range between the anchors on a station's loop, widened by `margin` each side.
    pub fn range(profile: &Loop,reach: &Reach,margin: f64) -> Result<[f64;2],String> {
        let s0 = profile.resolve(reach.start)?-margin;
        let mut s1 = profile.resolve(reach.end)?+margin;
        let total = profile.augmented_length();
        if s1 < s0 { s1 += total; }
        if s1-s0 >= total { return Err("the profile walk covers the whole section; the sheet cannot leave the blank".into()) }
        Ok([s0,s1])
    }
}

/// Pieces walked the other way round.
fn reverse(pieces: &mut [Piece]) {
    pieces.reverse();
    for piece in pieces.iter_mut() {
        piece.reversed = !piece.reversed;
        piece.points.reverse();
        let total = piece.length();
        let mut lengths: Vec<f64> = piece.lengths.iter().rev().map(|l| total-l).collect();
        lengths[0] = 0.;
        piece.lengths = lengths;
    }
}
