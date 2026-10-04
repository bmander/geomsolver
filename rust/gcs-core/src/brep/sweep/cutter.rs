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
/// How near an end of one piece is to the axis to be on it (mm).
const ON_AXIS: f64 = 1e-5;
const SAMPLES_PER_PIECE: usize = 64;
/// Where a piece's face id keeps which meeting of its face it is, above the section's own tag.
const OCCURRENCE: u32 = 26;

fn unit(a: V) -> Result<V,String> {
    let n = norm(a);
    if !(n > 0.) || !n.is_finite() { return Err("degenerate direction".into()) }
    Ok(a.map(|x| x/n))
}

/// What a piece is a stretch of: a step of a revolved section, or edge `k` of a prism profile's
/// loop `l`.
#[derive(Clone)]
enum Source { Meridian(Step),Edge(usize,usize) }

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
/// cut where `cut` says.
#[derive(Clone)]
pub struct Loop { pieces: Vec<Piece>,corners: Vec<Corner>,cut: Cut }

/// Where a place on a loop is: on a piece at arc length, or in a corner's fan.
enum Place { Piece {index: usize,s: f64},Fan {index: usize,fraction: f64} }

impl Loop {
    pub fn augmented_length(&self) -> f64 {
        self.pieces.iter().map(Piece::length).sum::<f64>()+self.corners.iter().map(|c| c.angle*TURN_SCALE).sum::<f64>()
    }
    fn start_of(&self,index: usize) -> f64 { (0..index).map(|k| self.pieces[k].length()+self.corners[k].angle*TURN_SCALE).sum() }
    fn locate(&self,s: f64) -> Place {
        let mut s = s.rem_euclid(self.augmented_length());
        for (k,piece) in self.pieces.iter().enumerate() {
            if s <= piece.length() { return Place::Piece {index:k,s} }
            s -= piece.length();
            let fan = self.corners[k].angle*TURN_SCALE;
            if s <= fan { return Place::Fan {index:k,fraction:if fan > 0. { s/fan } else { 0. }} }
            s -= fan;
        }
        Place::Piece {index:self.pieces.len()-1,s:self.pieces.last().unwrap().length()}
    }
    pub fn index_of(&self,face: u32) -> Option<usize> { self.pieces.iter().position(|p| p.face == face) }
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

/// What the stations section.
enum Kind { Revolved { side: V,sec: Sectioned },Extruded(Extruded) }

/// A cutter as its stations section it: about `axis` through `origin` (a prism's extrusion from a
/// point of its profile's plane).
pub struct Cutter { pub origin: V,pub axis: V,kind: Kind,sections: std::sync::Mutex<BTreeMap<u64,Vec<Loop>>> }

impl Cutter {
    /// The cutter of `recipe` (millimetres), or why the core does not section it.
    pub fn read(recipe: &Json) -> Result<Cutter,String> {
        if let Some(cutter) = Cutter::prism(recipe)? { return Ok(cutter) }
        let sec = Sectioned::read(recipe)?.map_err(|e| format!("the core sections a cutter of revolutions about parallel lines, \
            or a prism, only: {e}"))?;
        let (origin,axis) = (sec.origin,unit(sec.axis)?);
        let seed = if axis[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let side = unit(cross(cross(axis,seed),axis))?;
        Ok(Cutter {origin,axis,kind:Kind::Revolved {side,sec},sections:Default::default()})
    }
    /// A recipe whose root is one prism, as its sections: none for any other recipe.
    fn prism(recipe: &Json) -> Result<Option<Cutter>,String> {
        let field = |j: &Json,k: &str| j.get(k).cloned().ok_or(format!("recipe: no `{k}`"));
        let root = field(recipe,"root")?.as_i64();
        let nodes = field(recipe,"nodes")?;
        let Some(node) = nodes.arr().iter().find(|n| n.get("id").is_some_and(|i| i.as_i64() == root)) else { return Ok(None) };
        if !node.get("kind").is_some_and(|k| k.as_str() == "prism") { return Ok(None) }
        let profile = Profile::from_json(&field(node,"profile")?)?;
        let (from,to) = (field(node,"from")?.as_f64(),field(node,"to")?.as_f64());
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
    /// and the extrusion's extent from that plane; none for a cutter of revolutions.
    pub fn prism_profile(&self) -> Option<(&[Vec<Seg>],[f64;2])> {
        match &self.kind { Kind::Extruded(x) => Some((&x.walks,x.extent)),Kind::Revolved {..} => None }
    }
    /// Whether a station is an angle about the axis (else a distance along a prism).
    pub fn revolved(&self) -> bool { matches!(self.kind,Kind::Revolved {..}) }
    /// How far the band of stations may run before the sheet is said not to leave the blank: a
    /// whole turn about the axis, or a prism's own length (past it a cap would be in the blank).
    pub fn station_limit(&self) -> f64 {
        match &self.kind { Kind::Revolved {..} => TAU,Kind::Extruded(x) => x.extent[1]-x.extent[0] }
    }
    /// A piece's point at fraction `t` of its source, and the cutter's outward normal there.
    fn at(&self,cut: Cut,source: &Source,t: f64) -> (V,V) {
        match (&self.kind,cut,source) {
            (Kind::Revolved {sec,..},Cut::Side(side),Source::Meridian(step)) => sec.at(side,step,t),
            (Kind::Extruded(x),Cut::Offset(h),&Source::Edge(l,k)) => x.at(self.axis,l,k,h,t),
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
            Loop {pieces,corners,cut}
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
            loops.push(Loop {pieces,corners,cut});
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
