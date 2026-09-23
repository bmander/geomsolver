//! A body with continuous swept cuts, by arrangement and classification.
//!
//! The static remainder of the body is constructed as usual and is the blank.
//! For each distinct sweep, the native cutter is sectioned by meridian half-planes
//! of its reference axis; each section loop is the profile, and a row of the
//! candidate sheet is a section point carrying its face's outward normal, whose
//! contact position under the declared motion is exact. Sharp convex corners of
//! the section contribute a fan of normals at zero position change, the sweep of
//! the cutter's edge. Rows are spaced in augmented arc length (position length
//! plus turning across corners), so a flat bottom narrowing into a crease keeps a
//! smooth row structure. The sheet is placed at every pose the body cuts it at,
//! the kernel splits the blank by all of them, every cell is judged by the
//! declared material field at points a measured distance inside it, and the
//! material cells are united. Nothing here trims, selects visibility or knows a
//! cutter's shape; an unresolved or mixed cell refuses the build.
use super::*;
use super::cells::Cell;
use gcs_core::{interval::{Interval,minimum::Options},model::{Sketch,SolidDef},
    motion::Family,solid::{cad,MaterialEvaluator,MaterialField,ProbeState,SweepContacts}};
use std::f64::consts::{PI,TAU};

fn sub(a: [f64;3],b: [f64;3]) -> [f64;3] { std::array::from_fn(|k| a[k]-b[k]) }
fn dot(a: [f64;3],b: [f64;3]) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: [f64;3],b: [f64;3]) -> [f64;3] { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: [f64;3]) -> f64 { dot(a,a).sqrt() }
fn unit(a: [f64;3]) -> Result<[f64;3],String> {
    let n = norm(a);
    if n <= 0. || !n.is_finite() { return Err("degenerate direction".into()); }
    Ok(a.map(|v| v/n))
}
fn scaled(a: [f64;3],s: f64) -> [f64;3] { a.map(|v| v*s) }
fn distance(a: [f64;3],b: [f64;3]) -> f64 { norm(sub(a,b)) }

/// The fit contract's bounds: how far the fitted sheet may pass from a withheld contact, and
/// how far its normal may turn from the contact's. Gross, not the accuracy budget: they
/// separate a fit that follows its contacts from one that does not.
const FIT_DISTANCE: f64 = 0.25;
const FIT_TURN: f64 = 20.;

/// Progress on stderr: a member takes minutes, and the JSON report owns stdout. Each line
/// carries the time since the first, so a whole export reads as one timeline.
static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
pub(crate) fn stage(message: &str) {
    let at = START.get_or_init(std::time::Instant::now).elapsed().as_secs_f64();
    eprintln!("solventc: [{at:7.1} s] {message}");
}

/// A stage completed, for a harness: with `SOLVENT_STAGE_TRACE` naming a file, one line
/// `key<TAB>seconds` is appended to it. The keys, in the order a swept export completes them:
/// admission, blank, clearance, reach, sheet, fit, withheld, split, classify, fuse, step, stl,
/// agreement, written. Independent of the prose lines, which may change.
pub(crate) fn mark(key: &str) {
    let Ok(path) = std::env::var("SOLVENT_STAGE_TRACE") else { return };
    let at = START.get_or_init(std::time::Instant::now).elapsed().as_secs_f64();
    use std::io::Write;
    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
        let _ = writeln!(file,"{key}\t{at:.3}");
    }
}

/// The augmented length scale of a corner: one radian of turning counts as this
/// many millimetres of profile, so a fan receives rows like an arc of that radius.
const TURN_SCALE: f64 = 0.5;
const CHAIN_TOLERANCE: f64 = 1e-5;
const SAMPLES_PER_EDGE: usize = 64;
/// Target node spacing along the profile and along the band, in millimetres.
const ROW_SPACING: f64 = 0.15;
const COLUMN_SPACING: f64 = 0.5;

/// One edge of a section loop on one native face, sampled along the walk.
struct Piece {
    edge: c_int,
    face: c_int,
    reversed: bool,
    /// Uniformly spaced walk parameters' positions and cumulative chord length.
    points: Vec<[f64;3]>,
    lengths: Vec<f64>,
}
impl Piece {
    fn length(&self) -> f64 { *self.lengths.last().unwrap() }
    /// Curve parameter at arc length `s` along the walk.
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

/// A corner between consecutive pieces. `angle` is the turning of the outward
/// normal there when the corner is convex, and zero otherwise.
struct Corner { position: [f64;3],normals: [[f64;3];2],angle: f64 }

/// One planar section loop, oriented consistently across stations: pieces in
/// walk order, corner k after piece k (cyclic).
struct Loop { pieces: Vec<Piece>,corners: Vec<Corner> }

/// Where a profile row is: on a piece at arc length, or in a corner's fan.
enum Place { Piece {index: usize,s: f64},Fan {index: usize,fraction: f64} }

impl Loop {
    fn augmented_length(&self) -> f64 {
        self.pieces.iter().map(Piece::length).sum::<f64>()+self.corners.iter().map(|c| c.angle*TURN_SCALE).sum::<f64>()
    }
    fn start_of(&self,index: usize) -> f64 {
        (0..index).map(|k| self.pieces[k].length()+self.corners[k].angle*TURN_SCALE).sum()
    }
    fn locate(&self,s: f64) -> Place {
        let total = self.augmented_length();
        let mut s = s.rem_euclid(total);
        for (k,piece) in self.pieces.iter().enumerate() {
            if s <= piece.length() { return Place::Piece {index:k,s}; }
            s -= piece.length();
            let fan = self.corners[k].angle*TURN_SCALE;
            if s <= fan { return Place::Fan {index:k,fraction:if fan > 0. { s/fan } else { 0. }}; }
            s -= fan;
        }
        Place::Piece {index:self.pieces.len()-1,s:self.pieces.last().unwrap().length()}
    }
    fn index_of(&self,face: c_int) -> Option<usize> { self.pieces.iter().position(|p| p.face == face) }
    /// Augmented length of an anchor on this loop.
    fn resolve(&self,anchor: Anchor) -> Result<f64,String> {
        let index = self.index_of(anchor.face).ok_or("a profile face is missing at this station")?;
        Ok(self.start_of(index)+anchor.fraction.clamp(0.,1.)*self.pieces[index].length())
    }
}

/// A cutter surface point with its outward normal (native millimetres).
#[derive(Clone,Copy)]
struct Sample { position: [f64;3],normal: [f64;3] }

/// A place on the profile that survives the section changing shape between
/// stations: a face and a fraction of its piece.
#[derive(Clone,Copy,Debug)]
struct Anchor { face: c_int,fraction: f64 }

/// The band and walk chosen from where the cutter's declared-roll contacts enter
/// the blank: station angles, the walk's anchors in loop order, and the profile's
/// mean distance from the axis for column spacing.
struct Reach { stations: [f64;2],start: Anchor,end: Anchor,faces: usize,radius: f64,
    /// Where its time went: sections, contacts, blank queries (seconds) and points queried.
    spent: [f64;3],queried: usize }

/// A candidate sheet: contact positions on a row-major grid with the outward
/// normal of the cutter at each contact, in native millimetres.
#[derive(Debug)]
pub(crate) struct Sheet {
    pub points: Vec<[f64;3]>,
    pub normals: Vec<[f64;3]>,
    /// Which root of each point's contact equation the grid took (`ContactTime::branch`).
    pub branches: Vec<usize>,
    /// Each point's contact time.
    pub times: Vec<f64>,
    pub rows: usize,
    pub columns: usize,
    /// Contacts at the centre of every grid cell, withheld from the fit, with their normals.
    pub withheld: Vec<[f64;3]>,
    pub withheld_normals: Vec<[f64;3]>,
}

impl Sheet {
    /// The chart contract: where the grid lies in the blank it samples one regular chart of the
    /// envelope. Down each column, consecutive contacts keep one root of the contact equation
    /// (the roots swap only where they merge, at a fold of the tool's time chart) and its time
    /// does not leap (the same root a turn away). Position is no test: near a fold a contact
    /// runs fast along the sweep for a small step along the profile, which is steep, not
    /// broken; whether the fit can follow it is the fit contract's question. Steps wholly
    /// outside the blank are the margin that carries the sheet's edge out, and bound nothing;
    /// a fault there that disturbs the fit inside is the fit contract's to find. The first
    /// violation, where it is.
    pub(crate) fn chart_fault(&self,inside: Inside) -> Result<Option<String>,String> {
        let within = inside(&self.points)?;
        let mut faults = Vec::new();
        for c in 0..self.columns {
            let at = |r: usize| r*self.columns+c;
            for r in 1..self.rows {
                if !within[at(r)] && !within[at(r-1)] { continue; }
                let p = self.points[at(r)].map(|x| (x*1e3).round()/1e3);
                if self.branches[at(r)] != self.branches[at(r-1)] {
                    faults.push(format!("the contact changes root between rows {} and {r} of column {c} at {p:?}",r-1));
                } else if (self.times[at(r)]-self.times[at(r-1)]).abs() > 1. {
                    faults.push(format!("the contact time leaps from {:.3} to {:.3} between rows {} and {r} of column {c} at {p:?}",
                        self.times[at(r-1)],self.times[at(r)],r-1));
                }
            }
        }
        Ok(faults.first().map(|f| format!("{f} ({} such steps in the blank, over {} columns)",faults.len(),self.columns)))
    }
}

/// Whether points lie inside the blank, in native millimetres.
pub(crate) type Inside<'a> = &'a dyn Fn(&[[f64;3]]) -> Result<Vec<bool>,String>;

/// The cutter as the section machinery needs it: the native solid, its faces in
/// the stable enumeration the sections index, and its axis.
struct Cutter { solid: c_int,faces: Vec<c_int>,origin: [f64;3],axis: [f64;3],side: [f64;3] }

impl Session {
    fn cutter(&self,sk: &Sketch,source: usize) -> Result<Cutter,String> {
        let recipe = cad::recipe(sk,source)?;
        let solid = self.construct(&recipe)?;
        let mut axes: Vec<([f64;3],[f64;3])> = Vec::new();
        for node in recipe.get("nodes").unwrap().arr() {
            let name = node.get("name").unwrap().as_str();
            match node.get("kind").unwrap().as_str() {
                "revolve" => {
                    let read = |key: &str| -> [f64;3] { let a = node.get(key).unwrap().arr(); std::array::from_fn(|i| a[i].as_f64()) };
                    axes.push((read("origin"),unit(read("axis"))?));
                }
                "placed" | "body" => {}
                kind => return Err(format!("`{name}`: swept cutters are currently built from revolutions, not a {kind}")),
            }
        }
        // Stations are meridian half-planes of the first revolution's axis. Other
        // faces, placed or about another axis, are simply sectioned obliquely;
        // a face meeting one section twice is refused below.
        let (origin,axis) = *axes.first().ok_or("a swept cutter needs at least one revolution")?;
        let seed = if axis[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let side = unit(cross(cross(axis,seed),axis))?;
        Ok(Cutter {solid,faces:self.faces(solid)?,origin,axis,side})
    }

    fn side_at(cutter: &Cutter,angle: f64) -> [f64;3] {
        let other = cross(cutter.axis,cutter.side);
        std::array::from_fn(|k| cutter.side[k]*angle.cos()+other[k]*angle.sin())
    }

    /// Outward normal of a cutter face (by stable index) at a point on it.
    fn normal_at(&self,cutter: &Cutter,face: c_int,point: [f64;3]) -> Result<[f64;3],String> {
        let handle = *cutter.faces.get((face-1) as usize).ok_or("section face index out of range")?;
        let (normal,gap) = self.face_normal(handle,point)?;
        if gap > 1e-4 { return Err(format!("a section point lies {gap:.2e} mm off its face")); }
        Ok(normal)
    }

    /// Section loops at one station: edges chained by endpoint identity, each
    /// loop oriented to positive area about the section plane's normal, with
    /// corner fans where consecutive faces meet at a convex dihedral.
    fn profile(&self,cutter: &Cutter,angle: f64) -> Result<Vec<Loop>,String> {
        let side = Self::side_at(cutter,angle);
        let plane_normal = unit(cross(cutter.axis,side))?;
        let rows = self.section(cutter.solid,cutter.origin,cutter.axis,side)?;
        if rows.is_empty() { return Ok(Vec::new()); }
        let ends: Vec<[[f64;3];2]> = rows.iter().map(|&(edge,_)| Ok([self.edge_point(edge,0.)?,self.edge_point(edge,1.)?]))
            .collect::<Result<_,String>>()?;
        let mut used = vec![false;rows.len()];
        let mut loops = Vec::new();
        for start in 0..rows.len() {
            if used[start] { continue; }
            let mut chain: Vec<(usize,bool)> = vec![(start,false)];
            used[start] = true;
            let mut tail = ends[start][1];
            loop {
                let next = (0..rows.len()).filter(|&i| !used[i]).find_map(|i| {
                    if distance(ends[i][0],tail) < CHAIN_TOLERANCE { Some((i,false)) }
                    else if distance(ends[i][1],tail) < CHAIN_TOLERANCE { Some((i,true)) } else { None }
                });
                match next {
                    Some((i,reversed)) => { used[i] = true; tail = ends[i][if reversed { 0 } else { 1 }]; chain.push((i,reversed)); }
                    None => break,
                }
            }
            // A solid touching its own axis sections into an arc from pole to
            // pole; it closes through the axis, where there is no boundary and
            // so no corner fan.
            let on_axis = |p: [f64;3]| {
                let r = sub(p,cutter.origin);
                norm(sub(r,scaled(cutter.axis,dot(r,cutter.axis)))) < CHAIN_TOLERANCE
            };
            let axis_closed = distance(tail,ends[start][0]) >= CHAIN_TOLERANCE;
            if axis_closed && !(on_axis(tail) && on_axis(ends[start][0])) {
                return Err("a section of the cutter did not close into a loop".into());
            }
            let mut pieces = Vec::new();
            let mut area = 0.;
            for &(i,reversed) in &chain {
                let (edge,face) = rows[i];
                if pieces.iter().any(|p: &Piece| p.face == face) {
                    return Err("a cutter face meets one section twice; such profiles are not supported yet".into());
                }
                let mut points = Vec::with_capacity(SAMPLES_PER_EDGE+1);
                let mut lengths = vec![0.];
                for j in 0..=SAMPLES_PER_EDGE {
                    let t = j as f64/SAMPLES_PER_EDGE as f64;
                    let p = self.edge_point(edge,if reversed { 1.-t } else { t })?;
                    if let Some(last) = points.last() {
                        lengths.push(lengths.last().unwrap()+distance(p,*last));
                        area += dot(plane_normal,cross(sub(*last,cutter.origin),sub(p,cutter.origin)));
                    }
                    points.push(p);
                }
                pieces.push(Piece {edge,face,reversed,points,lengths});
            }
            if area < 0. { reverse(&mut pieces); }
            let mut corners = Vec::new();
            for k in 0..pieces.len() {
                let next = (k+1)%pieces.len();
                let position = *pieces[k].points.last().unwrap();
                if axis_closed && next == 0 {
                    let a = self.normal_at(cutter,pieces[k].face,position)?;
                    corners.push(Corner {position,normals:[a,a],angle:0.});
                    continue;
                }
                let a = self.normal_at(cutter,pieces[k].face,position)?;
                let b = self.normal_at(cutter,pieces[next].face,position)?;
                let angle = dot(a,b).clamp(-1.,1.).acos();
                // Convex when the bisector leaves the cutter; a concave corner's
                // normals turn through the material and sweep no exposed face.
                let convex = angle > 1e-6 && {
                    let bisector = unit(std::array::from_fn(|i| a[i]+b[i]))?;
                    let probe: [f64;3] = std::array::from_fn(|i| position[i]+bisector[i]*1e-3);
                    self.solid_contains(cutter.solid,&[probe],1e-7)?[0] == 0
                };
                corners.push(Corner {position,normals:[a,b],angle:if convex { angle } else { 0. }});
            }
            loops.push(Loop {pieces,corners});
        }
        Ok(loops)
    }

    fn sample(&self,cutter: &Cutter,profile: &Loop,s: f64) -> Result<Sample,String> {
        match profile.locate(s) {
            Place::Piece {index,s} => {
                let piece = &profile.pieces[index];
                let position = self.edge_point(piece.edge,piece.parameter(s))?;
                Ok(Sample {position,normal:self.normal_at(cutter,piece.face,position)?})
            }
            Place::Fan {index,fraction} => {
                let corner = &profile.corners[index];
                let [a,b] = corner.normals;
                let normal = unit(std::array::from_fn(|i| (1.-fraction)*a[i]+fraction*b[i]))?;
                Ok(Sample {position:corner.position,normal})
            }
        }
    }

    /// Contact position of a cutter sample under the sweep, the root nearest
    /// `near`. Native millimetres in and out; the contact math runs in model units.
    fn contact(sweep: &SweepContacts,scale: f64,sample: Sample,interval: [f64;2],near: f64)
        -> Result<Option<([f64;3],f64)>,String> {
        Ok(Self::contact_full(sweep,scale,sample,interval,near)?.map(|(p,_,t,_)| (p,t)))
    }
    /// Contact position, outward normal and time.
    fn contact_full(sweep: &SweepContacts,scale: f64,sample: Sample,interval: [f64;2],near: f64)
        -> Result<Option<([f64;3],[f64;3],f64,usize)>,String> {
        let roots = sweep.at_point_normal_over(scaled(sample.position,1./scale),sample.normal,interval,1e-9)
            .map_err(|e| if e == "Degenerate" {
                // Constant zero (in contact at every time: a pole on the spin axis, or a revolution
                // about the motion's own axis) and a double root (a point grazing at one time) are
                // alike to the root finder; neither gives the per-point time this construction needs.
                format!("a cutter point's contact equation is degenerate (in contact at every time, or only grazing) \
                 at {:?} with normal {:?}; the section construction needs one contact time per point",
                 sample.position.map(|x| (x*1e4).round()/1e4),sample.normal.map(|x| (x*1e4).round()/1e4))
            } else { e })?;
        Ok(roots.iter().min_by(|x,y| (x.root.time-near).abs().total_cmp(&(y.root.time-near).abs()))
            .map(|r| (scaled(r.contact.position,scale),r.contact.normal,r.root.time,r.root.branch)))
    }

    /// Find the stations and the profile walk whose declared-roll contacts lie
    /// inside the blank.
    fn reach(&self,cutter: &Cutter,sweep: &SweepContacts,scale: f64,inside: Inside) -> Result<Reach,String> {
        let declared = sweep.domain();
        let stations = 96;
        let mut inside_stations = Vec::new();
        let mut face_hits: std::collections::BTreeMap<c_int,(f64,f64)> = Default::default();
        let mut order: Option<Vec<c_int>> = None;
        let (mut radius_sum,mut radius_count) = (0.,0);
        let (mut spent,mut queried) = ([0.;3],0);
        for c in 0..stations {
            let angle = TAU*c as f64/stations as f64;
            let mut hit_here = false;
            let clock = std::time::Instant::now();
            let loops = self.profile(cutter,angle)?;
            spent[0] += clock.elapsed().as_secs_f64();
            for profile in &loops {
                let total = profile.augmented_length();
                let count = ((total/0.3).ceil() as usize).max(8);
                let mut queries = Vec::new();
                let clock = std::time::Instant::now();
                for i in 0..count {
                    let s = total*i as f64/count as f64;
                    let sample = self.sample(cutter,profile,s)?;
                    match Self::contact(sweep,scale,sample,declared,0.) {
                        Ok(Some((p,_))) => queries.push((s,sample,p)),
                        Ok(None) => {}
                        // A point whose contact equation is degenerate (a pole, in contact at every
                        // time) matters only if its path enters the blank; one that never does is
                        // no part of the boundary there, and the section need not parametrize it.
                        Err(error) => {
                            let path: Vec<[f64;3]> = (0..=64).map(|k| declared[0]+(declared[1]-declared[0])*k as f64/64.)
                                .map(|t| sweep.motion().at(t).map(|m| scaled(m.point(scaled(sample.position,1./scale)),scale)))
                                .collect::<Result<_,_>>()?;
                            if inside(&path)?.iter().any(|b| *b) { return Err(error); }
                        }
                    }
                }
                spent[1] += clock.elapsed().as_secs_f64();
                if queries.is_empty() { continue; }
                let clock = std::time::Instant::now();
                let states = inside(&queries.iter().map(|q| q.2).collect::<Vec<_>>())?;
                spent[2] += clock.elapsed().as_secs_f64(); queried += queries.len();
                let mut any = false;
                for ((s,sample,_),state) in queries.iter().zip(&states) {
                    if !*state { continue; }
                    any = true;
                    let radial = sub(sample.position,cutter.origin);
                    radius_sum += norm(sub(radial,scaled(cutter.axis,dot(radial,cutter.axis)))); radius_count += 1;
                    if let Place::Piece {index,s} = profile.locate(*s) {
                        let piece = &profile.pieces[index];
                        let f = s/piece.length();
                        let entry = face_hits.entry(piece.face).or_insert((f64::INFINITY,f64::NEG_INFINITY));
                        entry.0 = entry.0.min(f); entry.1 = entry.1.max(f);
                    }
                }
                if any {
                    if hit_here { return Err("two section loops of the cutter reach the blank at one station".into()); }
                    hit_here = true;
                    if order.is_none() { order = Some(profile.pieces.iter().map(|p| p.face).collect()); }
                }
            }
            if hit_here { inside_stations.push(c); }
        }
        let order = order.ok_or("no contact of the cutter within its declared roll lies inside the blank")?;
        // The station band is the complement of the largest gap between hits.
        let mut gap = (0,0);
        for (i,&c) in inside_stations.iter().enumerate() {
            let next = inside_stations[(i+1)%inside_stations.len()];
            let width = (next as i64-c as i64).rem_euclid(stations as i64);
            if width > gap.0 { gap = (width,i); }
        }
        let first = inside_stations[(gap.1+1)%inside_stations.len()];
        let last = inside_stations[gap.1];
        let span = (last as i64-first as i64).rem_euclid(stations as i64) as f64;
        let lo = TAU*first as f64/stations as f64;
        // The walk is the contiguous run of hit faces in loop order, likewise.
        let hit: Vec<usize> = order.iter().enumerate().filter(|(_,f)| face_hits.contains_key(f)).map(|(i,_)| i).collect();
        let n = order.len();
        let mut biggest = (0,0);
        for (i,&h) in hit.iter().enumerate() {
            let next = hit[(i+1)%hit.len()];
            let width = (next as i64-h as i64).rem_euclid(n as i64);
            if width > biggest.0 { biggest = (width,i); }
        }
        let start_face = order[hit[(biggest.1+1)%hit.len()]];
        let end_face = order[hit[biggest.1]];
        Ok(Reach {
            stations:[lo,lo+TAU*span/stations as f64],
            start:Anchor {face:start_face,fraction:face_hits[&start_face].0},
            end:Anchor {face:end_face,fraction:face_hits[&end_face].1},
            faces:hit.len(),
            radius:radius_sum/radius_count.max(1) as f64,
            spent,queried,
        })
    }

    /// The augmented range between the anchors on a station's loop, widened by
    /// `margin` on each side.
    fn range(profile: &Loop,reach: &Reach,margin: f64) -> Result<[f64;2],String> {
        let s0 = profile.resolve(reach.start)?-margin;
        let mut s1 = profile.resolve(reach.end)?+margin;
        let total = profile.augmented_length();
        if s1 < s0 { s1 += total; }
        if s1-s0 >= total { return Err("the profile walk covers the whole section; the sheet cannot leave the blank".into()); }
        Ok([s0,s1])
    }

    /// The candidate sheet over the reach with the given margins. Contact times
    /// follow continuity down each column and across the first row; the root each
    /// point took is kept for the chart contract, and a contact at the centre of every
    /// cell (a section at each column's mid angle, at each row's mid length) is withheld.
    fn sheet(&self,cutter: &Cutter,sweep: &SweepContacts,scale: f64,reach: &Reach,margin: f64,station_margin: f64)
        -> Result<Sheet,String> {
        let [lo,hi] = reach.stations;
        let span = hi-lo+2.*station_margin;
        let columns = ((span*reach.radius/COLUMN_SPACING).ceil() as usize).clamp(24,200);
        let wide = [-PI,PI];
        let mut columns_data: Vec<Vec<([f64;3],[f64;3],f64,usize)>> = Vec::with_capacity(columns);
        let (mut withheld,mut withheld_normals) = (Vec::new(),Vec::new());
        let mut rows = 0;
        let column_at = |angle: f64| -> Result<(Loop,[f64;2]),String> {
            let loops = self.profile(cutter,angle)?;
            let profile = loops.into_iter().find(|l| l.index_of(reach.start.face).is_some() && l.index_of(reach.end.face).is_some())
                .ok_or("no section loop carries the profile at a station of the band")?;
            let range = Self::range(&profile,reach,margin)?;
            Ok((profile,range))
        };
        let missing = |s: f64,angle: f64| format!("the contact the sheet follows ends: no root of the contact equation at \
            station {:.3} degrees, {s:.3} along the walk",angle.to_degrees());
        for c in 0..columns {
            let angle = lo-station_margin+span*c as f64/(columns-1) as f64;
            let (profile,[s0,s1]) = column_at(angle)?;
            if c == 0 { rows = (((s1-s0)/ROW_SPACING).ceil() as usize).clamp(24,240); }
            let mut column = Vec::with_capacity(rows);
            let mut near = columns_data.last().map(|previous| previous[0].2).unwrap_or(0.);
            for r in 0..rows {
                let s = s0+(s1-s0)*r as f64/(rows-1) as f64;
                let found = Self::contact_full(sweep,scale,self.sample(cutter,&profile,s)?,wide,near)?.ok_or_else(|| missing(s,angle))?;
                near = found.2;
                column.push(found);
            }
            columns_data.push(column);
        }
        for c in 0..columns-1 {
            let angle = lo-station_margin+span*(c as f64+0.5)/(columns-1) as f64;
            let (profile,[s0,s1]) = column_at(angle)?;
            for r in 0..rows-1 {
                let s = s0+(s1-s0)*(r as f64+0.5)/(rows-1) as f64;
                let near = 0.5*(columns_data[c][r].2+columns_data[c+1][r+1].2);
                if let Some((q,n,_,_)) = Self::contact_full(sweep,scale,self.sample(cutter,&profile,s)?,wide,near)? {
                    withheld.push(q); withheld_normals.push(n);
                }
            }
        }
        let (mut points,mut normals,mut branches,mut times) = (Vec::new(),Vec::new(),Vec::new(),Vec::new());
        for r in 0..rows { for column in &columns_data {
            points.push(column[r].0); normals.push(column[r].1); times.push(column[r].2); branches.push(column[r].3);
        } }
        Ok(Sheet {points,normals,branches,times,rows,columns,withheld,withheld_normals})
    }
}

/// Reverse a chain of pieces in place, keeping each piece's own sampling.
fn reverse(pieces: &mut Vec<Piece>) {
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

/// Classify every cell of a partition against the material field. A cell is
/// judged at several interior points with measured boundary distances; every
/// point needs a ball certificate within that distance and all must agree, so
/// an unresolved point or a cell that a leaking sheet failed to separate
/// refuses the build instead of guessing.
pub(crate) fn classify(session: &Session,partition: c_int,material: &mut MaterialEvaluator)
    -> Result<(Vec<Cell>,Vec<Cell>),String> {
    let (mut kept,mut removed) = (Vec::new(),Vec::new());
    let (mut sampling,mut probing,mut probes) = (0.,0.,0);
    // Each cell's volume was measured when the partition was validated; its point is the
    // deepest interior sample measured here.
    for solid in session.solids(partition)? {
        let volume = session.volume(solid)?;
        let clock = std::time::Instant::now();
        let samples = session.samples(solid,4,12)?;
        sampling += clock.elapsed().as_secs_f64();
        if samples.is_empty() { return Err(format!("a cell of volume {volume} has no interior sample")); }
        let cell = Cell {solid,point:samples[0].0,margin:samples[0].1,volume};
        let mut verdict = None;
        for (point,boundary) in samples {
            let distance = (boundary*0.5).min(0.05);
            if distance <= 1e-4 { continue; }
            let clock = std::time::Instant::now();
            probes += 1;
            let probe = material.probe(point.map(|x| Interval::point(x).unwrap()),[1.,0.,0.],distance,
                Options {value_tolerance:distance/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))?;
            probing += clock.elapsed().as_secs_f64();
            let inside = match probe.state {
                ProbeState::InteriorBall => true,
                ProbeState::ExteriorBall => false,
                state => return Err(format!("the material at {point:?} ({boundary:.3} mm from a cell boundary) is {state:?}")),
            };
            match verdict {
                None => verdict = Some(inside),
                Some(previous) if previous != inside => return Err(format!(
                    "a cell of volume {} reads both material and removed: a sheet did not separate it",cell.volume)),
                _ => {}
            }
        }
        match verdict {
            Some(true) => kept.push(cell),
            Some(false) => removed.push(cell),
            None => return Err(format!("a cell of volume {} has no sample clear of its boundary",cell.volume)),
        }
    }
    stage(&format!("classification: interior samples {sampling:.1} s, {probes} field probes {probing:.1} s"));
    Ok((kept,removed))
}

/// The candidate sheet grid of one swept solid whose contacts enter the blank
/// `inside` describes, widened until its boundary lies outside. Exposed for the
/// mesh path and for tests of the construction itself.
pub(crate) fn swept_sheet_grid(session: &Session,sk: &Sketch,swept: usize,inside: Inside) -> Result<Sheet,String> {
    let SolidDef::Swept {source,..} = &sk.solids[swept].def else {
        return Err(format!("`{}` is not a continuous sweep",sk.solids[swept].name));
    };
    let scale = sk.units.length.ok_or("CAD construction requires an explicit length unit")?.1;
    let name = &sk.solids[swept].name;
    let cutter = session.cutter(sk,*source as usize)?;
    let sweep = SweepContacts::read(sk,swept,1e-10)?;
    let started = std::time::Instant::now();
    let reach = session.reach(&cutter,&sweep,scale,inside)?;
    stage(&format!("`{name}`: contacts reach the blank over {:.1} degrees of stations and {} profile faces ({:?}: \
        sections {:.1} s, contacts {:.1} s, {} blank queries {:.1} s)",
        (reach.stations[1]-reach.stations[0]).to_degrees(),reach.faces,started.elapsed(),
        reach.spent[0],reach.spent[1],reach.queried,reach.spent[2]));
    mark("reach");
    // The margins carry the sheet's edge out of the blank; then the grid must be one chart.
    let (mut margin,mut station_margin) = (1.,(reach.stations[1]-reach.stations[0])*0.15);
    for _ in 0..6 {
        let sheet = session.sheet(&cutter,&sweep,scale,&reach,margin,station_margin)?;
        let mut boundary = Vec::new();
        for r in 0..sheet.rows { for c in 0..sheet.columns {
            if r == 0 || r+1 == sheet.rows || c == 0 || c+1 == sheet.columns { boundary.push(sheet.points[r*sheet.columns+c]); }
        } }
        if inside(&boundary)?.iter().any(|b| *b) { margin *= 1.6; station_margin *= 1.6; continue; }
        if let Some(fault) = sheet.chart_fault(inside)? {
            return Err(format!("`{name}`: the sheet is not one regular chart: {fault}"));
        }
        stage(&format!("`{name}`: sheet {}x{}, one chart ({:?})",sheet.rows,sheet.columns,started.elapsed()));
        mark("sheet");
        return Ok(sheet);
    }
    Err(format!("`{name}`: the candidate sheet cannot be widened out of the blank"))
}

/// The candidate sheet of one swept solid against a native blank: the roll must
/// carry the cutter clear of the blank at both limits (no caps on this path),
/// and the grid is fitted as a native face with its withheld contact error.
/// Where contacts reach the blank is asked of `field`, the same blank as the
/// core's analytic field, not of the kernel: a point there is microseconds where
/// the kernel's classifier took 30 ms, and this question only sizes the sheet.
pub(crate) fn swept_sheet(session: &Session,sk: &Sketch,swept: usize,blank: c_int,field: &gcs_core::solid::SpatialField)
    -> Result<(c_int,Sheet,f64),String> {
    let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else {
        return Err(format!("`{}` is not a continuous sweep",sk.solids[swept].name));
    };
    let scale = sk.units.length.ok_or("CAD construction requires an explicit length unit")?.1;
    let name = &sk.solids[swept].name;
    let cutter = session.cutter(sk,*source as usize)?;
    let family = Family::read(sk,*motion as usize)?;
    let started = std::time::Instant::now();
    for (label,limit) in [("start",from.value),("end",to.value)] {
        let placed = session.place(cutter.solid,family.at(limit)?,scale)?;
        let overlap = session.common_volume(placed,blank)?;
        if overlap > 0. {
            return Err(format!("`{name}`: the declared roll leaves the cutter inside the blank at its {label} \
                ({:.1} degrees, {overlap:.3} mm³ overlap); declare a roll that carries it clear",limit.to_degrees()));
        }
    }
    stage(&format!("`{name}`: the roll carries the cutter clear of the blank at both limits ({:?})",started.elapsed()));
    mark("clearance");
    let inside = |points: &[[f64;3]]| Ok(points.iter().map(|p| field.value(p.map(|x| x/scale)) < 0.).collect());
    let sheet = swept_sheet_grid(session,sk,swept,&inside)?;
    let face = session.fit_sheet(&sheet.points,sheet.rows,sheet.columns)?;
    stage(&format!("`{name}`: fitted the sheet; measuring {} withheld contacts against it",sheet.withheld.len()));
    mark("fit");
    // The fit contract, at the centre of every cell: the fitted surface passes near the true
    // contact there, and its normal agrees with the contact's. A fit that oscillates or folds
    // between samples fails the second where the first can still pass.
    // Judged where it bounds the cut: in the blank, or within half a millimetre of it.
    let near: Vec<([f64;3],[f64;3])> = sheet.withheld.iter().zip(&sheet.withheld_normals)
        .filter(|(p,_)| field.value(p.map(|x| x/scale)) < 0.5/scale).map(|(p,n)| (*p,*n)).collect();
    let points: Vec<[f64;3]> = near.iter().map(|(p,_)| *p).collect();
    let (mut error,mut turn,mut worst) = (0_f64,0_f64,[0.;3]);
    for ((p,n),found) in near.iter().map(|(p,n)| (p,n)).zip(session.surface_feet(face,&points)?) {
        let Some((m,gap)) = found else { error = f64::INFINITY; worst = *p; continue };
        let angle = dot(m,*n).abs().min(1.).acos().to_degrees();
        if gap > error { error = gap; worst = *p; }
        turn = turn.max(angle);
    }
    stage(&format!("`{name}`: fitted sheet within {error:.2e} mm of the {} withheld contacts at the blank, normals within {turn:.2} degrees",
        points.len()));
    if error > FIT_DISTANCE || turn > FIT_TURN {
        return Err(format!("`{name}`: the fitted sheet leaves its contacts: {error:.3} mm (at {:?}) and {turn:.1} degrees, \
            against {FIT_DISTANCE} mm and {FIT_TURN} degrees",worst.map(|x| (x*1e3).round()/1e3)));
    }
    mark("withheld");
    Ok((face,sheet,error))
}

/// The cell contract after classification: one removed cell per placement of each sweep, the
/// cells of one sweep congruent (its placements are rigid copies, so a cell that differs was
/// cut by something else: a neighbour, a tangent face), and no cell below a size floor (a
/// sliver the kernel made of near-tangent surfaces). The removed volumes are compared as a
/// set, since a cell does not say which placement cut it; a body whose placements cut unequal
/// amounts from an asymmetric blank would need them told apart first.
fn cells_contract(kept: &[Cell],removed: &[Cell],sweeps: &[cad::SweptCut]) -> Result<(),String> {
    const FLOOR: f64 = 1e-3;
    if let Some(cell) = kept.iter().chain(removed).find(|c| c.volume < FLOOR) {
        return Err(format!("the split left a cell of {:.3e} mm³, under the {FLOOR} mm³ floor, near {:?}",
            cell.volume,cell.point.map(|x| (x*1e3).round()/1e3)));
    }
    if removed.len() != sweeps.len() {
        return Err(format!("the split removed {} cells for {} placements",removed.len(),sweeps.len()));
    }
    let (least,most) = removed.iter().fold((f64::INFINITY,0_f64),|(l,m),c| (l.min(c.volume),m.max(c.volume)));
    if most > least*(1.+1e-4) {
        return Err(format!("the removed cells are not congruent: {least:.6} to {most:.6} mm³"));
    }
    Ok(())
}

/// Construct a body whose cuts include continuous sweeps. Returns the native
/// solid handle in this session.
pub(crate) fn construct_swept_body(session: &Session,sk: &Sketch,body: usize,recipe: &cad::StaticRecipe) -> Result<c_int,String> {
    let scale = sk.units.length.ok_or("CAD construction requires an explicit length unit")?.1;
    let blank = session.construct(&recipe.recipe)?;
    let (field,_) = gcs_core::solid::admission::static_remainder(sk,body,1e-10)?;
    stage(&format!("`{}`: static blank of {} operations",sk.solids[body].name,recipe.recipe.get("nodes").unwrap().arr().len()));
    mark("blank");
    let mut tools = Vec::new();
    let mut distinct: Vec<usize> = recipe.sweeps.iter().map(|s| s.swept).collect();
    distinct.sort(); distinct.dedup();
    for swept in distinct {
        let (face,_,_) = swept_sheet(session,sk,swept,blank,&field)?;
        for cut in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            tools.push(session.place(face,cut.pose,scale)?);
        }
    }
    let started = std::time::Instant::now();
    let partition = session.split_solid(blank,&tools)?;
    stage(&format!("split the blank by {} sheets into {} cells ({:?})",tools.len(),session.solids(partition)?.len(),started.elapsed()));
    mark("split");
    let started = std::time::Instant::now();
    let mut material = MaterialField::read(sk,body,1e-10)?.evaluator(4096);
    let (kept,removed) = classify(session,partition,&mut material)?;
    stage(&format!("classified {} material and {} removed cells ({:?})",kept.len(),removed.len(),started.elapsed()));
    if kept.is_empty() { return Err("no cell of the blank is material".into()); }
    cells_contract(&kept,&removed,&recipe.sweeps)?;
    mark("classify");
    let started = std::time::Instant::now();
    let part = session.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>())?;
    let [vertex,edge,_] = session.tolerances(part)?;
    stage(&format!("united the material: {:.6} mm³, {} faces, tolerances {vertex:.1e}/{edge:.1e} mm ({:?})",
        session.volume(part)?,session.faces(part)?.len(),started.elapsed()));
    mark("fuse");
    Ok(part)
}

/// Native construction of a solid for export: the static recipe when it is
/// complete, the swept path when the body cuts continuous sweeps.
pub(crate) fn construct_solid(session: &Session,sk: &Sketch,solid: usize) -> Result<c_int,String> {
    let recipe = cad::recipe_static(sk,solid)?;
    if recipe.sweeps.is_empty() { session.construct(&recipe.recipe) }
    else { construct_swept_body(session,sk,solid,&recipe) }
}
