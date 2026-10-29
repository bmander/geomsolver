//! A swept cut's cutter, sectioned exactly at its stations: a solid of revolutions about lines
//! parallel to one axis (`brep::section`) by the half-planes of that axis, a station an angle; or
//! a prism by planes square to its extrusion, a station a distance along it, every section the
//! profile itself (a rack's tooth). Each section is a loop of pieces, every piece a stretch of one
//! face (a revolution's meridian, a prism side's edge) walked in augmented arc length — length
//! along the pieces, and at a convex corner the turning of the normal across its fan — and where
//! the cutter's contacts within the declared roll enter the blank fixes the band of stations and
//! the stretch of the walk the sheet spans (`Cutter::reach`).
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::brep::build::{walks,Profile,Seg};
use crate::brep::geom::Frame;
use crate::brep::planar::Step;
use crate::brep::recipe::{Node,Recipe};
use crate::brep::section::Sectioned;
use crate::json::Json;
use crate::solid::contact_trace::{Sample,TraceError,Tracer};
use crate::space::{cross,distance,dot,norm,scale,sub};
use std::collections::BTreeMap;
use std::f64::consts::TAU;

type V = [f64;3];

/// The augmented length of a corner: one radian of turning counts as this many millimetres of
/// profile, so a fan receives rows like an arc of that radius.
const TURN_SCALE: f64 = 0.05;
/// The same for a cap's rim in a section across a prism (`Across`): a quarter turn there carries a
/// contact from a cap's to a side's, which may be far apart in time, and the sheet's rows, laid at
/// `contact_trace::ROW_SPACING`, must step through it a radian of the motion at a time at most
/// (`contact_trace::charted`), so a rim's fan is laid as long as an arc of this radius.
const RIM_SCALE: f64 = 1.;
/// How near an end of one piece is to the axis to be on it (mm).
const ON_AXIS: f64 = 1e-5;
const SAMPLES_PER_PIECE: usize = 64;
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
/// station), and its points along the walk with their cumulative chord length.
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
/// counter-clockwise about the extrusion), and the extrusion's extent from that plane.
struct Extruded { walks: Vec<Vec<Seg>>,extent: [f64;2] }

impl Extruded {
    /// Edge `k` of loop `l` at fraction `t`, lifted `h` along the extrusion `n`: its point, and the
    /// side face's outward normal (the walk's direction across `n`, since the material lies left).
    fn at(&self,n: V,l: usize,k: usize,h: f64,t: f64) -> (V,V) {
        let seg = &self.walks[l][k];
        let w = seg.t[0]+(seg.t[1]-seg.t[0])*t;
        let (p,d,_) = seg.curve.d2(w);
        let d = scale(d,(seg.t[1]-seg.t[0]).signum());
        let normal = cross(d,n);
        (std::array::from_fn(|i| p[i]+n[i]*h),scale(normal,1./norm(normal).max(1e-300)))
    }
}

/// A prism whose cap reaches the blank, sectioned through its extrusion (#684 F): a station is a
/// distance along `across` (a unit direction in the profile's plane) from the profile's origin, and
/// its section the plane square to `across` there: the profile's chord carried along the
/// extrusion, a ruling of the side at each end of the chord and the chord on each cap, four pieces
/// whose corners are the caps' rims, each a quarter turn of fan. Its one loop of lines and arcs,
/// walked counter-clockwise about the extrusion; the extrusion's extent; `across` and `along` (the
/// extrusion crossed with it, so the section's walk is counter-clockwise about `across`); and the
/// stations its sections exist at, the profile's least and greatest distance along `across`.
/// Each chord is walked as `chord` millimetres whatever its length at its station, so that a
/// station's pieces start at the same walk lengths as its neighbours' where the profile tapers: the
/// sheet's rows are laid at walk lengths common to every column.
struct Across { walk: Vec<Seg>,extent: [f64;2],across: V,along: V,range: [f64;2],chord: f64 }

/// A cap's tag in a section across (`Across`), the low cap's along the extrusion and the high's:
/// a side's tag is its edge's, `(loop + 1) << 16 | edge`, so these name no side.
pub const CAP_LOW: u32 = 1;
pub const CAP_HIGH: u32 = 2;

/// A section across `walk`'s plane by the plane `across·(x − origin) = c`: where it meets the
/// profile, each the edge it is on, its parameter there and the point (a point where two edges
/// meet counted once).
fn meets(walk: &[Seg],origin: V,across: V,c: f64) -> Vec<(usize,f64,V)> {
    let size = walk.iter().map(|s| distance(s.start(),origin)).fold(1_f64,f64::max);
    let mut out: Vec<(usize,f64,V)> = Vec::new();
    for (k,seg) in walk.iter().enumerate() {
        let [lo,hi] = seg.span();
        let tol = 1e-12*(1.+hi.abs()+lo.abs());
        let mut at = |w: f64| {
            let p = seg.curve.point(w);
            if !out.iter().any(|(_,_,q)| distance(*q,p) <= 1e-9*size) { out.push((k,w,p)); }
        };
        match &seg.curve {
            crate::brep::geom::Curve::Line {p,d} => {
                let rate = dot(across,*d);
                if rate.abs() <= 1e-12 { continue }
                let w = (c-dot(across,sub(*p,origin)))/rate;
                if w >= lo-tol && w <= hi+tol { at(w.clamp(lo,hi)); }
            }
            crate::brep::geom::Curve::Circle(f,r) => {
                let (a,b) = (r*dot(across,f.x),r*dot(across,f.y));
                let amplitude = a.dhypot(b);
                if !(amplitude > 0.) { continue }
                let cosine = (c-dot(across,sub(f.o,origin)))/amplitude;
                if cosine.abs() > 1. { continue }
                let (phase,turn) = (b.datan2(a),cosine.dacos());
                for w in [phase+turn,phase-turn] {
                    let w = lo+(w-lo).rem_euclid(TAU);
                    let w = if w > hi+tol && (w-TAU-lo).abs() <= tol { lo } else { w };
                    if w <= hi+tol { at(w.min(hi)); }
                }
            }
            _ => {}
        }
    }
    out
}

/// The tangent of a walked edge at its parameter `w`, in the walk's direction.
fn tangent(seg: &Seg,w: f64) -> V { let (_,d,_) = seg.curve.d2(w); scale(d,(seg.t[1]-seg.t[0]).signum()) }

/// Where a prism's sections across a direction cannot be one regular family (#684 F): the least and
/// greatest distance of its profile along the direction (where a section closes to a ruling of a
/// side), and each corner of its profile — where two edges meet at an angle, so a section through
/// it leaps from one side's normal to the other's — at its distance and its point (millimetres).
#[derive(Clone,Debug)]
pub struct Spread { pub range: [f64;2],pub corners: Vec<(f64,V)> }

/// What the stations section.
enum Kind { Revolved { side: V,sec: Sectioned },Extruded(Extruded),Across(Across) }

/// A cutter as its stations section it: about `axis` through `origin` (a prism's extrusion from a
/// point of its profile's plane).
pub struct Cutter { pub origin: V,pub axis: V,kind: Kind,sections: std::sync::Mutex<BTreeMap<u64,Vec<Loop>>> }

impl Cutter {
    /// The cutter of `recipe` (millimetres), or why the core does not section it.
    pub fn read(recipe: &Json) -> Result<Cutter,String> {
        let recipe = Recipe::read(recipe)?;
        if let Some(cutter) = Cutter::prism(&recipe)? { return Ok(cutter) }
        let sec = match Sectioned::of(&recipe) {
            Ok(sec) => sec,
            Err(e) if e.kind == crate::brep::Kind::Refused => return Err(format!("the core sections a cutter of revolutions \
                about parallel lines, or a prism, only: {e}")),
            Err(e) => return Err(e.into()),
        };
        let (origin,axis) = (sec.origin,unit(sec.axis)?);
        let seed = if axis[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let side = unit(cross(cross(axis,seed),axis))?;
        Ok(Cutter {origin,axis,kind:Kind::Revolved {side,sec},sections:Default::default()})
    }
    /// A recipe whose root is one prism, as its sections: none for any other recipe.
    fn prism(recipe: &Recipe) -> Result<Option<Cutter>,String> {
        let Some(node) = recipe.nodes.iter().find(|n| n.id == recipe.root) else { return Ok(None) };
        let Node::Prism {profile,from,to,..} = &node.node else { return Ok(None) };
        let (profile,from,to) = (Profile::of(profile)?,*from,*to);
        let axis = unit(profile.normal)?;
        let plane = Frame::about(profile.origin,axis);
        let walks = walks(&profile,&|q| { let l = plane.local(q); [l[0],l[1]] })?;
        if walks.iter().flatten().any(|s| matches!(s.curve,crate::brep::geom::Curve::BSpline(_))) {
            return Err("the core sections a prism of lines and arcs only".into())
        }
        Ok(Some(Cutter {origin:profile.origin,axis,kind:Kind::Extruded(Extruded {walks,extent:[from.min(to),from.max(to)]}),
            sections:Default::default()}))
    }
    /// A prism's profile, its loops walked in its plane (outer counter-clockwise about the axis),
    /// and the extrusion's extent from that plane; none for a cutter of revolutions, or a prism
    /// sectioned across.
    pub fn prism_profile(&self) -> Option<(&[Vec<Seg>],[f64;2])> {
        match &self.kind { Kind::Extruded(x) => Some((&x.walks,x.extent)),_ => None }
    }
    /// Where this prism's sections across `across` (a direction in its profile's plane, made square
    /// to the extrusion and unit) cannot be one regular family (`Spread`): none for a cutter that is
    /// no prism of one loop.
    pub fn spread(&self,across: V) -> Option<Spread> {
        let Kind::Extruded(x) = &self.kind else { return None };
        let [walk] = &x.walks[..] else { return None };
        let across = unit(sub(across,scale(self.axis,dot(across,self.axis)))).ok()?;
        let at = |p: V| dot(across,sub(p,self.origin));
        let mut range = [f64::INFINITY,f64::NEG_INFINITY];
        let mut reach = |p: V| { let c = at(p); range = [range[0].min(c),range[1].max(c)]; };
        for seg in walk {
            reach(seg.start());
            if let crate::brep::geom::Curve::Circle(f,_) = &seg.curve {
                let [lo,hi] = seg.span();
                let phase = dot(across,f.y).datan2(dot(across,f.x));
                for w in [phase,phase+std::f64::consts::PI] {
                    let w = lo+(w-lo).rem_euclid(TAU);
                    if w <= hi { reach(seg.curve.point(w)); }
                }
            }
        }
        let corners = (0..walk.len()).filter_map(|k| {
            let (a,b) = (&walk[k],&walk[(k+1)%walk.len()]);
            let (ta,tb) = (unit(tangent(a,a.t[1])).ok()?,unit(tangent(b,b.t[0])).ok()?);
            (dot(ta,tb).clamp(-1.,1.).dacos() > 1e-6).then(|| (at(a.end()),a.end()))
        }).collect();
        Some(Spread {range,corners})
    }
    /// This prism sectioned across `across` (`Across`, #684 F): its stations distances along it,
    /// each section its chord carried along the extrusion, every chord walked as long as the one
    /// at the station `reference` (the middle of the band the sheet spans). A prism of one loop
    /// only.
    pub fn across(&self,across: V,reference: f64) -> Result<Cutter,String> {
        let Kind::Extruded(x) = &self.kind else { return Err("only a prism is sectioned across".into()) };
        let [walk] = &x.walks[..] else { return Err("a prism sectioned across has a profile of one loop".into()) };
        let spread = self.spread(across).ok_or("a prism sectioned across has a profile of one loop")?;
        let across = unit(sub(across,scale(self.axis,dot(across,self.axis))))?;
        let along = cross(self.axis,across);
        let ends = meets(walk,self.origin,across,reference);
        let [(_,_,a),(_,_,b)] = ends[..] else {
            return Err(format!("the prism's section {reference:.4} mm across its profile is no one chord"))
        };
        Ok(Cutter {origin:self.origin,axis:self.axis,kind:Kind::Across(Across {walk:walk.clone(),extent:x.extent,across,along,
            range:spread.range,chord:distance(a,b)}),sections:Default::default()})
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
        match &self.kind { Kind::Revolved {..} => TAU,Kind::Extruded(x) => x.extent[1]-x.extent[0],
            Kind::Across(x) => x.range[1]-x.range[0] }
    }
    /// A piece's point at fraction `t` of its source, and the cutter's outward normal there.
    fn at(&self,cut: Cut,source: &Source,t: f64) -> (V,V) {
        match (&self.kind,cut,source) {
            (Kind::Revolved {sec,..},Cut::Side(side),Source::Meridian(step)) => sec.at(side,step,t),
            (Kind::Extruded(x),Cut::Offset(h),&Source::Edge(l,k)) => x.at(self.axis,l,k,h,t),
            (Kind::Across(_),_,&Source::Segment {a,b,normal}) => (std::array::from_fn(|i| a[i]+(b[i]-a[i])*t),normal),
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
            Kind::Revolved {side,sec} => self.loops(sec,Cutter::side_at(*side,self.axis,angle))?,
            Kind::Extruded(x) => self.extruded_loops(x,angle),
            Kind::Across(x) => self.across_loops(x,angle)?,
        };
        self.sections.lock().unwrap_or_else(|e| e.into_inner()).insert(angle.to_bits(),loops.clone());
        Ok(loops)
    }
    /// A prism's section at `h` along its extrusion: its profile, each edge a face.
    fn extruded_loops(&self,x: &Extruded,h: f64) -> Vec<Loop> {
        x.walks.iter().enumerate().map(|(l,walk)| {
            let cut = Cut::Offset(h);
            let pieces: Vec<Piece> = (0..walk.len()).map(|k| {
                let source = Source::Edge(l,k);
                let points: Vec<V> = (0..=SAMPLES_PER_PIECE).map(|j| self.at(cut,&source,j as f64/SAMPLES_PER_PIECE as f64).0).collect();
                let mut lengths = vec![0.];
                for w in points.windows(2) { lengths.push(lengths.last().unwrap()+distance(w[0],w[1])); }
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
        let mut found = meets(&x.walk,self.origin,x.across,c);
        if found.len() != 2 {
            return Err(format!("the prism's section {c:.4} mm across its profile meets it at {} points, not at the two ends of                 one chord",found.len()))
        }
        found.sort_by(|a,b| dot(x.along,a.2).total_cmp(&dot(x.along,b.2)));
        let side = |&(k,w,p): &(usize,f64,V)| -> Result<(u32,V,V),String> {
            Ok((((1u32) << 16) | k as u32,p,unit(cross(tangent(&x.walk[k],w),self.axis))?))
        };
        let ((near_tag,near,near_normal),(far_tag,far,far_normal)) = (side(&found[0])?,side(&found[1])?);
        let lift = |p: V,h: f64| -> V { std::array::from_fn(|i| p[i]+self.axis[i]*h) };
        let [low,high] = x.extent;
        let down = scale(self.axis,-1.);
        let segments = [(CAP_LOW,lift(near,low),lift(far,low),down),(far_tag,lift(far,low),lift(far,high),far_normal),
            (CAP_HIGH,lift(far,high),lift(near,high),self.axis),(near_tag,lift(near,high),lift(near,low),near_normal)];
        let cut = Cut::Offset(c);
        let pieces: Vec<Piece> = segments.iter().map(|&(face,a,b,normal)| {
            let points: Vec<V> = (0..=SAMPLES_PER_PIECE).map(|j| { let t = j as f64/SAMPLES_PER_PIECE as f64;
                std::array::from_fn(|i| a[i]+(b[i]-a[i])*t) }).collect();
            // a chord walked as long as the reference's, a ruling as it is
            let walked = if face == CAP_LOW || face == CAP_HIGH { x.chord } else { distance(a,b) };
            let lengths = (0..=SAMPLES_PER_PIECE).map(|j| walked*j as f64/SAMPLES_PER_PIECE as f64).collect();
            Piece {face,reversed:false,source:Source::Segment {a,b,normal},points,lengths}
        }).collect();
        if segments.iter().any(|s| !(distance(s.1,s.2) > 0.)) { return Err(format!("the prism's section {c:.4} mm across its profile has no width")) }
        let corners = (0..4).map(|k| {
            let (a,b) = (segments[k].3,segments[(k+1)%4].3);
            Corner {position:segments[k].2,normals:[a,b],angle:dot(a,b).clamp(-1.,1.).dacos()}
        }).collect();
        Ok(vec![Loop {pieces,corners,cut,turn:RIM_SCALE}])
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
    /// The stations and the profile walk whose declared-roll contacts lie inside the blank.
    pub fn reach(&self,tracer: &Tracer) -> Result<Reach,String> {
        let (sweep,mm,inside) = (tracer.sweep,tracer.scale,tracer.inside);
        let declared = sweep.domain();
        let stations = 96;
        // half a step in from either end, round the axis or along the prism
        let (cyclic,station) = match &self.kind {
            Kind::Revolved {..} => (true,[0.,TAU]),
            Kind::Extruded(x) => (false,x.extent),
            Kind::Across(x) => (false,x.range),
        };
        let station = |c: f64| station[0]+(station[1]-station[0])*(c+0.5)/stations as f64;
        let mut inside_stations = Vec::new();
        let mut face_hits: BTreeMap<u32,(f64,f64)> = BTreeMap::new();
        let mut order: Option<Vec<u32>> = None;
        let (mut radius_sum,mut radius_count) = (0.,0);
        let (mut spent,mut queried) = ([0.;3],0);
        // half a step off the side, every station's section first, on every core
        let clock = crate::clock::Instant::now();
        let mut sections = crate::par::indices(stations,|c| self.profile(station(c as f64))).into_iter();
        spent[0] += clock.elapsed().as_secs_f64();
        for c in 0..stations {
            let mut hit_here = false;
            let loops = sections.next().expect("a section a station")?;
            for profile in &loops {
                let total = profile.augmented_length();
                let count = ((total/0.3).ceil() as usize).max(8);
                let mut queries = Vec::new();
                let clock = crate::clock::Instant::now();
                for i in 0..count {
                    let s = total*i as f64/count as f64;
                    let sample = self.sample(profile,s)?;
                    match tracer.nearest(sample,declared,0.) {
                        Ok(Some((p,_))) => queries.push((s,sample,p)),
                        Ok(None) => {}
                        // a point whose contact equation is degenerate (a pole, in contact at every
                        // time) matters only if its path enters the blank
                        Err(error) => {
                            let path: Vec<V> = (0..=64).map(|k| declared[0]+(declared[1]-declared[0])*k as f64/64.)
                                .map(|t| sweep.motion().at(t).map(|m| scale(m.point(scale(sample.position,1./mm)),mm)))
                                .collect::<Result<_,_>>()?;
                            if inside(&path)?.iter().any(|b| *b) { return Err(error.into()) }
                        }
                    }
                }
                spent[1] += clock.elapsed().as_secs_f64();
                if queries.is_empty() { continue }
                let clock = crate::clock::Instant::now();
                let states = inside(&queries.iter().map(|q| q.2).collect::<Vec<_>>())?;
                spent[2] += clock.elapsed().as_secs_f64(); queried += queries.len();
                let mut any = false;
                for ((s,sample,_),state) in queries.iter().zip(&states) {
                    if !*state { continue }
                    any = true;
                    let radial = sub(sample.position,self.origin);
                    radius_sum += norm(sub(radial,scale(self.axis,dot(radial,self.axis)))); radius_count += 1;
                    if let Place::Piece {index,s} = profile.locate(*s) {
                        let piece = &profile.pieces[index];
                        let f = s/piece.length();
                        let entry = face_hits.entry(piece.face).or_insert((f64::INFINITY,f64::NEG_INFINITY));
                        entry.0 = entry.0.min(f); entry.1 = entry.1.max(f);
                    }
                }
                if any {
                    if hit_here { return Err("two section loops of the cutter reach the blank at one station".into()) }
                    hit_here = true;
                    if order.is_none() { order = Some(profile.pieces.iter().map(|p| p.face).collect()); }
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
        // the walk: the contiguous run of hit faces in loop order, likewise
        let hit: Vec<usize> = order.iter().enumerate().filter(|(_,f)| face_hits.contains_key(f)).map(|(i,_)| i).collect();
        let n = order.len();
        let mut biggest = (0,0);
        for (i,&h) in hit.iter().enumerate() {
            let next = hit[(i+1)%hit.len()];
            let width = (next as i64-h as i64).rem_euclid(n as i64);
            if width > biggest.0 { biggest = (width,i); }
        }
        let (start_face,end_face) = (order[hit[(biggest.1+1)%hit.len()]],order[hit[biggest.1]]);
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
