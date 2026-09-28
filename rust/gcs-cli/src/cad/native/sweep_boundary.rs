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
use super::kernel::Cell;
use gcs_core::{interval::{Interval,minimum::Options},model::{Sketch,SolidDef},motion::Family,
    solid::{admission::Admission,cad,contracts,MaterialEvaluator,MaterialField,ProbeState,SweepContacts}};
use gcs_core::solid::contact_trace::{Band,Layout,Sample,Station,TraceError,Tracer,Withheld,marked};
pub(crate) use gcs_core::solid::contact_trace::{Inside,Rows,Sheet};
use gcs_core::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use gcs_core::space::{sub,dot,cross,norm,scale as scaled,distance};
use std::f64::consts::TAU;

/// `a` over its length, refusing a zero or non-finite one.
fn unit(a: [f64;3]) -> Result<[f64;3],String> {
    let n = norm(a);
    if n <= 0. || !n.is_finite() { return Err("degenerate direction".into()); }
    Ok(a.map(|v| v/n))
}

/// The fit contract's bounds without a stated tolerance: how far the fitted sheet may pass from a
/// withheld contact, and how far its normal may turn from the contact's. Gross, not the accuracy
/// budget: they separate a fit that follows its contacts from one that does not. A tolerance
/// replaces them (`Tolerance::fit`, `Tolerance::turn`).
const FIT_DISTANCE: f64 = 0.25;
const FIT_TURN: f64 = 20.;
/// How far the fitted face's own normal may turn between points a quarter of a cell apart in the
/// blank: past a right angle it has folded back on itself, a pleat between the withheld contacts.
const FOLD_TURN: f64 = 90.;

/// `SOLVENT_TRACE_DEBUG=1` prints where a station's contact curve ends, leaves the root window or
/// runs away, and the edges of a section that does not close: the instruments that located the
/// faults of the traced construction, kept for the next one.
fn tracing() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_TRACE_DEBUG").is_some())
}

/// The augmented length scale of a corner: one radian of turning counts as this
/// many millimetres of profile, so a fan receives rows like an arc of that radius.
const TURN_SCALE: f64 = 0.05;
const CHAIN_TOLERANCE: f64 = 1e-5;
const SAMPLES_PER_EDGE: usize = 64;

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
            // A section closing through the axis is open, and the edge it started from need not
            // be at one of its ends: extend it backward from its head too.
            let mut head = ends[start][0];
            loop {
                let next = (0..rows.len()).filter(|&i| !used[i]).find_map(|i| {
                    if distance(ends[i][1],head) < CHAIN_TOLERANCE { Some((i,false)) }
                    else if distance(ends[i][0],head) < CHAIN_TOLERANCE { Some((i,true)) } else { None }
                });
                match next {
                    Some((i,reversed)) => { used[i] = true; head = ends[i][if reversed { 1 } else { 0 }]; chain.insert(0,(i,reversed)); }
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
            let axis_closed = distance(tail,head) >= CHAIN_TOLERANCE;
            if axis_closed && !(on_axis(tail) && on_axis(head)) {
                if tracing() {
                    eprintln!("section at {:.4}: chain {:?}; ends {:?}",angle,chain,ends.iter().zip(&rows).map(|(e,r)| (r.1,e.map(|p| p.map(|x| (x*1e4).round()/1e4)))).collect::<Vec<_>>());
                }
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

    /// The cutter point and its outward normal at augmented length `s` of a section. A fan whose
    /// normals cancel carries no direction: no one contact time there, like a degenerate root.
    fn sample(&self,cutter: &Cutter,profile: &Loop,s: f64) -> Result<Sample,TraceError> {
        match profile.locate(s) {
            Place::Piece {index,s} => {
                let piece = &profile.pieces[index];
                let position = self.edge_point(piece.edge,piece.parameter(s))?;
                Ok(Sample {position,normal:self.normal_at(cutter,piece.face,position)?})
            }
            Place::Fan {index,fraction} => {
                let corner = &profile.corners[index];
                let [a,b] = corner.normals;
                let normal = unit(std::array::from_fn(|i| (1.-fraction)*a[i]+fraction*b[i])).map_err(TraceError::Degenerate)?;
                Ok(Sample {position:corner.position,normal})
            }
        }
    }

    /// Find the stations and the profile walk whose declared-roll contacts lie
    /// inside the blank.
    fn reach(&self,cutter: &Cutter,tracer: &Tracer) -> Result<Reach,String> {
        let (sweep,scale,inside) = (tracer.sweep,tracer.scale,tracer.inside);
        let declared = sweep.domain();
        let stations = 96;
        let mut inside_stations = Vec::new();
        let mut face_hits: std::collections::BTreeMap<c_int,(f64,f64)> = Default::default();
        let mut order: Option<Vec<c_int>> = None;
        let (mut radius_sum,mut radius_count) = (0.,0);
        let (mut spent,mut queried) = ([0.;3],0);
        for c in 0..stations {
            // Half a step off the side: a revolution's seam lies in the plane of its profile, and
            // a section plane containing a seam loses that face's section.
            let angle = TAU*(c as f64+0.5)/stations as f64;
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
                    match tracer.nearest(sample,declared,0.) {
                        Ok(Some((p,_))) => queries.push((s,sample,p)),
                        Ok(None) => {}
                        // A point whose contact equation is degenerate (a pole, in contact at every
                        // time) matters only if its path enters the blank; one that never does is
                        // no part of the boundary there, and the section need not parametrize it.
                        Err(error) => {
                            let path: Vec<[f64;3]> = (0..=64).map(|k| declared[0]+(declared[1]-declared[0])*k as f64/64.)
                                .map(|t| sweep.motion().at(t).map(|m| scaled(m.point(scaled(sample.position,1./scale)),scale)))
                                .collect::<Result<_,_>>()?;
                            if inside(&path)?.iter().any(|b| *b) { return Err(error.into()); }
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
        let lo = TAU*(first as f64+0.5)/stations as f64;
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
        let cell = Cell {solid,point:samples[0].0,volume};
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

/// A sheet grid fitted as a native face and judged: the face, the grid, and its withheld error.
pub(crate) type Fitted = (c_int,Sheet,f64);

/// A fitted sheet's verdict at its withheld contacts: it fits (the face and its withheld error),
/// or, held to a tolerance, it misses at the contacts marked, which a finer grid may mend.
pub(crate) enum Judged { Fits(c_int,f64),Misses {over: Vec<bool>,refusal: ExportRefusal} }

/// How many times a sheet's grid is refined toward its tolerance, and the most nodes it may
/// have each way (the kernel interpolates at most 512): past either it is refused.
const MOST_REFINEMENTS: usize = 4;
const MOST_ROWS: usize = 480;
const MOST_COLUMNS: usize = 400;

/// The candidate sheet grid of one swept solid whose contacts enter the blank
/// `inside` describes, widened until its boundary lies outside: the core traces
/// (`solid::contact_trace`), the native cutter's sections are what it traces. Each row
/// placement in turn is handed to `fitted`; the first it accepts is the sheet, and a sheet it
/// refuses at the withheld contacts (a fit that misses or folds) passes to the next placement.
/// Held to a `tolerance`, a sheet also withholds the middles of its cells' sides, and one that
/// misses is refined where it misses (`contact_trace::marked`, `Grid::refined`) and fitted again,
/// until it fits or its refinement budget is spent.
pub(crate) fn swept_sheets(session: &Session,sk: &Sketch,swept: usize,inside: Inside,placements: &[Rows],
    tolerance: Option<Tolerance>,fitted: &mut dyn FnMut(&Sheet) -> Result<Judged,ExportRefusal>) -> Result<Fitted,ExportRefusal> {
    let SolidDef::Swept {source,..} = &sk.solids[swept].def else {
        return Err(ExportRefusal::at(Stage::Reach,format!("`{}` is not a continuous sweep",sk.solids[swept].name)));
    };
    let scale = cad::millimetres(sk).at(Stage::Reach)?;
    let name = &sk.solids[swept].name;
    let cutter = session.cutter(sk,*source as usize).at(Stage::Reach)?;
    let sweep = SweepContacts::read(sk,swept,cad::AXIS_TOLERANCE).at(Stage::Reach)?;
    let tracer = Tracer {sweep:&sweep,scale,inside,debug:tracing()};
    let started = std::time::Instant::now();
    let reach = session.reach(&cutter,&tracer).at(Stage::Reach)?;
    stage(&format!("`{name}`: contacts reach the blank over {:.1} degrees of stations and {} profile faces ({:?}: \
        sections {:.1} s, contacts {:.1} s, {} blank queries {:.1} s)",
        (reach.stations[1]-reach.stations[0]).to_degrees(),reach.faces,started.elapsed(),
        reach.spent[0],reach.spent[1],reach.queried,reach.spent[2]));
    mark(Stage::Reach);
    // A station: the section loop carrying the reach's walk, and the window between its anchors.
    let station_at = |angle: f64| -> Result<Station,TraceError> {
        let loops = session.profile(&cutter,angle)?;
        let profile = loops.into_iter().find(|l| l.index_of(reach.start.face).is_some() && l.index_of(reach.end.face).is_some())
            .ok_or("no section loop carries the profile at a station of the band")?;
        let window = Session::range(&profile,&reach,0.)?;
        let cutter = &cutter;
        Ok(Station {length:profile.augmented_length(),window,sample:Box::new(move |s| session.sample(cutter,&profile,s))})
    };
    let band = Band {stations:reach.stations,radius:reach.radius};
    let widened = |placement: Rows| -> Result<(Layout,Sheet),String> {
        // The margins carry the sheet's edge out of the blank; then the grid must be one chart.
        let (mut margin,mut station_margin) = (1.,(reach.stations[1]-reach.stations[0])*0.15);
        for _ in 0..6 {
            let layout = tracer.layout(&station_at,band,margin,station_margin,placement)?;
            let sheet = layout.sheet(&layout.grid,Withheld::Centres)?;
            let edges = [(0..sheet.columns).map(|c| sheet.points[c]).collect::<Vec<_>>(),
                (0..sheet.columns).map(|c| sheet.points[(sheet.rows-1)*sheet.columns+c]).collect(),
                (0..sheet.rows).map(|r| sheet.points[r*sheet.columns]).collect(),
                (0..sheet.rows).map(|r| sheet.points[r*sheet.columns+sheet.columns-1]).collect()];
            let within: Vec<usize> = edges.iter().map(|e| inside(e).map(|v| v.iter().filter(|b| **b).count())).collect::<Result<_,_>>()?;
            if within.iter().any(|&n| n > 0) {
                // An edge point in the blank at a time within the declared roll is where the sheet is
                // too small, and widening helps. One at a time outside it is the sheet, extended past
                // the roll so that it is rectangular, coming back into the blank: the cap at the roll's
                // limit is clear of the blank (admission's E1), so the true boundary is not there, and
                // widening only carries the extension further. That needs a sheet trimmed at the roll's
                // limits in its parameter space, which this construction does not build.
                let declared = sweep.domain();
                let mut late = None;
                let mut early = 0;
                for r in 0..sheet.rows { for c in 0..sheet.columns {
                    if !(r == 0 || r+1 == sheet.rows || c == 0 || c+1 == sheet.columns) { continue }
                    let k = r*sheet.columns+c;
                    if !inside(&[sheet.points[k]])?[0] { continue }
                    let t = sheet.times[k];
                    if t < declared[0] || t > declared[1] { late.get_or_insert((t,sheet.points[k])); } else { early += 1; }
                } }
                if early == 0 { if let Some((t,p)) = late {
                    return Err(format!("`{name}`: the sheet, extended past the declared roll, comes back into the blank at {:?} \
                        (time {:.1} degrees, the roll being {:.1} to {:.1}); it needs trimming at the roll's limits",
                        p.map(|x| (x*1e4).round()/1e4),t.to_degrees(),declared[0].to_degrees(),declared[1].to_degrees()));
                } }
                stage(&format!("`{name}`: the sheet's edge is in the blank (first row {}, last row {}, first column {}, last column {} \
                    points); widening its margins",within[0],within[1],within[2],within[3]));
                margin *= 1.6; station_margin *= 1.6; continue;
            }
            if let Some(fault) = sheet.chart_fault(inside)? {
                return Err(format!("`{name}`: the sheet is not one regular chart: {fault}"));
            }
            stage(&format!("`{name}`: sheet {}x{}, one chart ({:?})",sheet.rows,sheet.columns,started.elapsed()));
            return Ok((layout,sheet));
        }
        Err(format!("`{name}`: the candidate sheet cannot be widened out of the blank"))
    };
    let mut refused: Option<ExportRefusal> = None;
    for &placement in placements {
        let by = match placement { Rows::Walk => "walk length", Rows::Length => "length in space" };
        if let Some(refusal) = &refused {
            stage(&format!("{}; placing the sheet's rows by {by} instead",refusal.message));
        }
        let (layout,first) = widened(placement).at(Stage::Sheet)?;
        mark(Stage::Sheet);
        if tolerance.is_none() {
            match fitted(&first) {
                Ok(Judged::Fits(face,error)) => return Ok((face,first,error)),
                Ok(Judged::Misses {refusal,..}) => refused = Some(refusal),
                Err(refusal) if refusal.stage == Stage::Withheld => refused = Some(refusal),
                Err(refusal) => return Err(refusal),
            }
            continue;
        }
        // Held to a tolerance: refined where it misses until it fits, or refused.
        let mut grid = layout.grid.clone();
        for round in 0.. {
            let clock = std::time::Instant::now();
            let sheet = layout.sheet(&grid,Withheld::Sides).map_err(String::from).at(Stage::Sheet)?;
            stage(&format!("`{name}`: sheet {}x{} and {} withheld contacts read ({:?})",sheet.rows,sheet.columns,
                sheet.withheld.len(),clock.elapsed()));
            if let Some(fault) = sheet.chart_fault(inside).at(Stage::Sheet)? {
                return Err(ExportRefusal::at(Stage::Sheet,format!("`{name}`: the refined sheet is not one regular chart: {fault}")));
            }
            let (over,mut refusal) = match fitted(&sheet) {
                Ok(Judged::Fits(face,error)) => return Ok((face,sheet,error)),
                Ok(Judged::Misses {over,refusal}) => (over,refusal),
                Err(refusal) if refusal.stage == Stage::Withheld => { refused = Some(refusal); break }
                Err(refusal) => return Err(refusal),
            };
            let (rows,columns) = marked(&sheet.sites,&over,sheet.rows,sheet.columns);
            let next = grid.refined(&rows,&columns);
            if round >= MOST_REFINEMENTS || next.rows.len() > MOST_ROWS || next.columns.len() > MOST_COLUMNS {
                refusal.message = format!("{} with rows by {by}, after {round} refinements to {}x{} nodes (at most {MOST_REFINEMENTS} \
                    refinements and {MOST_ROWS}x{MOST_COLUMNS} nodes)",refusal.message,sheet.rows,sheet.columns);
                refused = Some(refusal);
                break;
            }
            stage(&format!("`{name}`: refining the sheet where it misses: {} of {} row intervals and {} of {} column intervals \
                split, {}x{} nodes",rows.iter().filter(|m| **m).count(),rows.len(),columns.iter().filter(|m| **m).count(),
                columns.len(),next.rows.len(),next.columns.len()));
            grid = next;
        }
    }
    Err(refused.unwrap_or_else(|| ExportRefusal::at(Stage::Sheet,format!("`{name}`: no row placement was offered"))))
}

/// The fit contract of a sheet grid fitted as a native face, judged where the face bounds the cut:
/// in the blank, or within half a millimetre of it (`near`). At every withheld contact the face
/// passes near the true contact there and its normal agrees with the contact's; and nowhere in the
/// blank does its own normal turn back between points a quarter of a cell apart, a pleat the
/// withheld contacts (one a cell) can straddle. Without a tolerance the bars are gross (0.25 mm,
/// 20°); held to one, the distance bar is its fit share and the normal bar follows from it, the gap
/// and the node spacing at each contact (`Tolerance::turn`), and a miss is a verdict the caller may
/// refine.
fn judged(session: &Session,name: &str,sheet: &Sheet,scale: f64,near: &dyn Fn([f64;3]) -> f64,tolerance: Option<Tolerance>)
    -> Result<Judged,ExportRefusal> {
    let at = |p: [f64;3]| p.map(|x| (x*1e3).round()/1e3);
    let (nu,nv) = (4*(sheet.rows-1)+1,4*(sheet.columns-1)+1);
    let folds = |face: c_int| -> Result<Vec<([f64;3],[f64;3])>,ExportRefusal> {
        let grid = session.surface_grid(face,nu,nv).at(Stage::Withheld)?;
        let (mut fold,mut folded) = (0_f64,[0.;3]);
        for i in 0..nu { for j in 0..nv {
            let (p,n) = grid[i*nv+j];
            let beside = [(i+1 < nu).then(|| grid[(i+1)*nv+j]),(j+1 < nv).then(|| grid[i*nv+j+1])];
            for (q,m) in beside.into_iter().flatten() {
                let angle = dot(n,m).clamp(-1.,1.).acos().to_degrees();
                if angle > fold && (near(p) < 0. || near(q) < 0.) { fold = angle; folded = p; }
            }
        } }
        if fold > FOLD_TURN {
            return Err(ExportRefusal {stage:Stage::Withheld,condition:None,witness:Some(folded.map(|x| x/scale)),
                message:format!("`{name}`: the fitted sheet folds in the blank: its normal turns {fold:.1} degrees between points a \
                quarter of a cell apart (at {:?}), against {FOLD_TURN} degrees",at(folded))});
        }
        Ok(grid)
    };
    let Some(tolerance) = tolerance else {
        let face = session.fit_sheet(&sheet.points,sheet.rows,sheet.columns).at(Stage::Fit)?;
        stage(&format!("`{name}`: fitted the sheet; measuring {} withheld contacts against it",sheet.withheld.len()));
        mark(Stage::Fit);
        let held: Vec<([f64;3],[f64;3])> = sheet.withheld.iter().zip(&sheet.withheld_normals)
            .filter(|(p,_)| near(**p) < 0.5).map(|(p,n)| (*p,*n)).collect();
        let points: Vec<[f64;3]> = held.iter().map(|(p,_)| *p).collect();
        let (mut error,mut turn,mut worst,mut turned) = (0_f64,0_f64,[0.;3],[0.;3]);
        for ((p,n),found) in held.iter().zip(session.surface_feet(face,&points).at(Stage::Withheld)?) {
            let Some((m,gap)) = found else { error = f64::INFINITY; worst = *p; continue };
            let angle = dot(m,*n).abs().min(1.).acos().to_degrees();
            if gap > error { error = gap; worst = *p; }
            if angle > turn { turn = angle; turned = *p; }
        }
        stage(&format!("`{name}`: fitted sheet within {error:.2e} mm of the {} withheld contacts at the blank, normals within {turn:.2} degrees",
            points.len()));
        if error > FIT_DISTANCE || turn > FIT_TURN {
            let witness = if error > FIT_DISTANCE { worst } else { turned };
            return Err(ExportRefusal {stage:Stage::Withheld,condition:None,witness:Some(witness.map(|x| x/scale)),
                message:format!("`{name}`: the fitted sheet leaves its contacts: {error:.3} mm (at {:?}) and {turn:.1} degrees (at {:?}), \
                against {FIT_DISTANCE} mm and {FIT_TURN} degrees",at(worst),at(turned))});
        }
        folds(face)?;
        mark(Stage::Withheld);
        return Ok(Judged::Fits(face,error));
    };
    let held: Vec<usize> = (0..sheet.withheld.len()).filter(|&i| near(sheet.withheld[i]) < 0.5).collect();
    let points: Vec<[f64;3]> = held.iter().map(|&i| sheet.withheld[i]).collect();
    let bar = tolerance.fit();
    // One interpolation of the grid, judged: a fold refuses it outright (refining would not unfold
    // it; another row placement may), otherwise every withheld contact is read against it.
    let fitted = |parametrization: c_int| -> Result<Candidate,ExportRefusal> {
        let clock = std::time::Instant::now();
        let face = session.fit_sheet_with(&sheet.points,sheet.rows,sheet.columns,parametrization).at(Stage::Fit)?;
        let grid = folds(face)?;
        // Each contact's foot is searched from the nearest point of the fold check's grid, a quarter
        // of a cell apart: a local search where a global one samples the whole face, and a local
        // extremum is never nearer than the nearest foot, so a gap it reads is never too small. A
        // foot farther than the bar is searched globally, so a miss is never a local search's.
        let guesses = nearest_on(&grid,nu,nv,&points);
        let feet = session.surface_feet_near(face,&points,&guesses,bar).at(Stage::Withheld)?;
        // Distance first: a sheet is refined where it misses its contacts by distance, then where its
        // normals turn past their bars once every distance holds. A normal's bar shrinks as its
        // contact's gap nears the tolerance, so a contact missing by distance misses by normal too,
        // and would mark the other direction for a miss the first already names.
        let mut c = Candidate {face,parametrization,far:vec![false;sheet.withheld.len()],turned:vec![false;sheet.withheld.len()],
            error:0.,worst:[0.;3],turn:(0.,f64::INFINITY,[0.;3]),distant:0,bent:0,spent:clock.elapsed()};
        for (&i,found) in held.iter().zip(feet) {
            let p = sheet.withheld[i];
            let (gap,angle) = match found { Some((m,gap)) => (gap,dot(m,sheet.withheld_normals[i]).abs().min(1.).acos().to_degrees()),
                None => (f64::INFINITY,180.) };
            let limit = tolerance.turn(gap,sheet.spacing(sheet.sites[i]));
            if gap > c.error { c.error = gap; c.worst = p; }
            if angle-limit > c.turn.0-c.turn.1 { c.turn = (angle,limit,p); }
            c.far[i] = gap > bar; c.turned[i] = angle > limit;
            if (c.far[i] || c.turned[i]) && tracing() {
                eprintln!("  parametrization {parametrization}: miss at site {:?}: {:.2} µm, {angle:.2} against {limit:.2} degrees, \
                    spacing {:.4} mm, blank {:.3}, at {:?}",sheet.sites[i],gap*1e3,sheet.spacing(sheet.sites[i]),near(p),at(p));
            }
        }
        (c.distant,c.bent) = (c.far.iter().filter(|m| **m).count(),c.turned.iter().filter(|m| **m).count());
        // The fold check at a sixteenth of a cell, for where a fit creases between the quarter-cell
        // points the check reads: the instrument that found the pinion's crease.
        if tracing() {
            let (fu,fv) = (16*(sheet.rows-1)+1,16*(sheet.columns-1)+1);
            let fine = session.surface_grid(face,fu,fv).at(Stage::Withheld)?;
            let mut turns: Vec<(f64,usize)> = Vec::new();
            for i in 0..fu { for j in 0..fv {
                let (p,n) = fine[i*fv+j];
                for (q,m) in [(i+1 < fu).then(|| fine[(i+1)*fv+j]),(j+1 < fv).then(|| fine[i*fv+j+1])].into_iter().flatten() {
                    let angle = dot(n,m).clamp(-1.,1.).acos().to_degrees();
                    if angle > 5. && (near(p) < 0. || near(q) < 0.) { turns.push((angle,i*fv+j)); }
                }
            } }
            turns.sort_by(|a,b| b.0.total_cmp(&a.0));
            eprintln!("  parametrization {parametrization}: fine fold check: {} neighbours in the blank turn over 5 degrees",turns.len());
            for (a,k) in turns.iter().take(8) {
                eprintln!("    {a:.2} degrees at row {:.3} column {:.3}, {:?}",(k/fv) as f64/16.,(k%fv) as f64/16.,at(fine[*k].0));
            }
        }
        stage(&format!("`{name}`: fitted sheet {}x{} ({}) within {:.2} µm of the {} withheld contacts at the blank (bar {:.2} µm), \
            normals within their bars by {:.2} degrees at worst ({:.2} against {:.2}); {} miss by distance, {} by normal ({:?})",
            sheet.rows,sheet.columns,["even","chord-length","centripetal"][parametrization as usize],c.error*1e3,points.len(),
            bar*1e3,c.turn.1-c.turn.0,c.turn.0,c.turn.1,c.distant,c.bent,c.spent));
        Ok(c)
    };
    // Both parametrizations interpolate the same exact contacts, and the withheld contacts say which
    // follows them: chord-length parameters (a step's share of its column's length, averaged over
    // the columns) stay consistent under refinement, since halving a step halves its share, but where
    // a sheet's contacts crowd on one face and stretch round the next (the pinion's flank into its
    // fillet) a column's own parameters depart from their average and the fit creased there, 31°
    // between points a sixteenth of a cell apart; centripetal ones (the square root of each step)
    // temper that departure, the pinion's first fit coming 7.6 µm from its contacts where chord
    // length came 16.5 with no crease past 5°, but give a halved step √2 of its share, so a locally
    // refined grid's parameters no longer match its spacing (the swept torus's fit went from 0.11 to
    // 3.1 µm on three rows added). Neither holds everywhere; the better fit is the sheet.
    let chosen = match (fitted(2),fitted(1)) {
        (Err(fold),Err(_)) => return Err(fold),
        (Ok(c),Err(_)) | (Err(_),Ok(c)) => c,
        (Ok(a),Ok(b)) => if (b.distant,b.bent,b.error) < (a.distant,a.bent,a.error) { b } else { a },
    };
    mark(Stage::Fit);
    let Candidate {face,far,turned,error,worst,turn,distant,bent,..} = chosen;
    if distant+bent > 0 {
        let witness = if distant > 0 { worst } else { turn.2 };
        return Ok(Judged::Misses {over:if distant > 0 { far } else { turned },refusal:ExportRefusal {stage:Stage::Withheld,
            condition:None,witness:Some(witness.map(|x| x/scale)),
            message:format!("`{name}`: the fitted sheet misses {distant} of its {} withheld contacts by distance, {:.2} µm from one \
            (at {:?}) against {:.2} µm, half the {} µm tolerance, and {bent} by normal, one {:.2} degrees off (at {:?}) against its \
            {:.2}",points.len(),error*1e3,at(worst),bar*1e3,tolerance.millimetres*1e3,turn.0,at(turn.2),turn.1)}});
    }
    stage(&format!("`{name}`: the {} fit holds",["even","chord-length","centripetal"][chosen.parametrization as usize]));
    mark(Stage::Withheld);
    Ok(Judged::Fits(face,error))
}

/// One interpolation of a sheet held to a tolerance and its reading at the withheld contacts: which
/// miss by distance and which by normal, the worst gap and where, and the normal furthest past its
/// own bar (turn, bar, where).
struct Candidate {
    face: c_int,parametrization: c_int,far: Vec<bool>,turned: Vec<bool>,error: f64,worst: [f64;3],
    turn: (f64,f64,[f64;3]),distant: usize,bent: usize,spent: std::time::Duration,
}

/// For each point, the parameters (fractions of the UV box) of the nearest point of an `nu` x `nv`
/// grid of a face's surface, found through cubes about as wide as the grid's steps; NaN where no
/// grid point lies in the cubes about a point.
fn nearest_on(grid: &[([f64;3],[f64;3])],nu: usize,nv: usize,points: &[[f64;3]]) -> Vec<[f64;2]> {
    let mut steps: Vec<f64> = (0..nu*nv).filter(|k| k%nv+1 < nv && k/nv+1 < nu)
        .flat_map(|k| [distance(grid[k].0,grid[k+1].0),distance(grid[k].0,grid[k+nv].0)]).collect();
    steps.sort_by(f64::total_cmp);
    let side = steps.get(steps.len()*19/20).copied().unwrap_or(1.).max(1e-9);
    let cube = |p: [f64;3]| p.map(|x| (x/side).floor() as i64);
    let mut cubes: std::collections::BTreeMap<[i64;3],Vec<usize>> = Default::default();
    for (k,(p,_)) in grid.iter().enumerate() { cubes.entry(cube(*p)).or_default().push(k); }
    points.iter().map(|&p| {
        let c = cube(p);
        let mut best = (f64::INFINITY,usize::MAX);
        for d in 0..27 {
            let key = [c[0]+d%3-1,c[1]+d/3%3-1,c[2]+d/9-1];
            for &k in cubes.get(&key).map_or(&[][..],Vec::as_slice) {
                let e = distance(grid[k].0,p);
                if e < best.0 { best = (e,k); }
            }
        }
        if best.1 == usize::MAX { [f64::NAN;2] } else { [(best.1/nv) as f64/(nu-1) as f64,(best.1%nv) as f64/(nv-1) as f64] }
    }).collect()
}

/// The candidate sheet of one swept solid against a native blank: the roll must
/// carry the cutter clear of the blank at both limits (no caps on this path),
/// and the grid is fitted as a native face with its withheld contact error.
/// Where contacts reach the blank is asked of `field`, the same blank as the
/// core's analytic field, not of the kernel: a point there is microseconds where
/// the kernel's classifier took 30 ms, and this question only sizes the sheet.
pub(crate) fn swept_sheet(session: &Session,sk: &Sketch,swept: usize,blank: c_int,field: &gcs_core::solid::SpatialField,
    tolerance: Option<Tolerance>) -> Result<(c_int,Sheet,f64),ExportRefusal> {
    let name = &sk.solids[swept].name;
    let scale = cad::millimetres(sk).at(Stage::Clearance)?;
    let clear = || -> Result<(),String> {
        let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else {
            return Err(format!("`{name}` is not a continuous sweep"));
        };
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
        Ok(())
    };
    clear().at(Stage::Clearance)?;
    mark(Stage::Clearance);
    let inside = |points: &[[f64;3]]| Ok(points.iter().map(|p| field.value(p.map(|x| x/scale)) < 0.).collect());
    let near = |p: [f64;3]| field.value(p.map(|x| x/scale))*scale;
    // Rows by walk length first, the placement the bevel pair and the pinion were recorded with;
    // by length where that fit misses or folds.
    swept_sheets(session,sk,swept,&inside,&[Rows::Walk,Rows::Length],tolerance,
        &mut |sheet| judged(session,name,sheet,scale,&near,tolerance))
}

/// Construct a body whose cuts include continuous sweeps, which only its admission to the
/// generating-sweep class allows. Returns the native solid handle in this session.
pub(crate) fn construct_swept_body(session: &Session,sk: &Sketch,body: usize,recipe: &cad::StaticRecipe,
    admission: &Admission,tolerance: Option<Tolerance>) -> Result<c_int,ExportRefusal> {
    if admission.body() != body {
        return Err(ExportRefusal::at(Stage::Admission,format!("`{}`: the admission presented is another body's",sk.solids[body].name)));
    }
    let scale = cad::millimetres(sk).at(Stage::Blank)?;
    let blank = session.construct(&recipe.recipe).at(Stage::Blank)?;
    let (field,_) = gcs_core::solid::admission::static_remainder(sk,body,cad::AXIS_TOLERANCE).at(Stage::Blank)?;
    stage(&format!("`{}`: static blank of {} operations",sk.solids[body].name,recipe.recipe.get("nodes").unwrap().arr().len()));
    mark(Stage::Blank);
    let mut tools = Vec::new();
    let mut distinct: Vec<usize> = recipe.sweeps.iter().map(|s| s.swept).collect();
    distinct.sort(); distinct.dedup();
    for swept in distinct {
        let (face,_,_) = swept_sheet(session,sk,swept,blank,&field,tolerance)?;
        for cut in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            tools.push(session.place(face,cut.pose,scale).at(Stage::Split)?);
        }
    }
    let started = std::time::Instant::now();
    let partition = session.split_solid(blank,&tools).at(Stage::Split)?;
    stage(&format!("split the blank by {} sheets into {} cells ({:?})",tools.len(),
        session.solids(partition).at(Stage::Split)?.len(),started.elapsed()));
    mark(Stage::Split);
    let started = std::time::Instant::now();
    let classified = || -> Result<(Vec<Cell>,Vec<Cell>),String> {
        let mut material = MaterialField::read(sk,body,cad::AXIS_TOLERANCE)?.evaluator(cad::POSE_CACHE);
        let (kept,removed) = classify(session,partition,&mut material)?;
        stage(&format!("classified {} material and {} removed cells ({:?})",kept.len(),removed.len(),started.elapsed()));
        if kept.is_empty() { return Err("no cell of the blank is material".into()); }
        let volumes = |cells: &[Cell]| cells.iter().map(|c| contracts::CellVolume {volume:c.volume,point:c.point}).collect::<Vec<_>>();
        contracts::cells(&volumes(&kept),&volumes(&removed),recipe.sweeps.len())?;
        Ok((kept,removed))
    };
    let (kept,_) = classified().at(Stage::Classify)?;
    mark(Stage::Classify);
    let started = std::time::Instant::now();
    let fused = || -> Result<c_int,String> {
        let part = session.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>())?;
        let [vertex,edge,_] = session.tolerances(part)?;
        stage(&format!("united the material: {:.6} mm³, {} faces, tolerances {vertex:.1e}/{edge:.1e} mm ({:?})",
            session.volume(part)?,session.faces(part)?.len(),started.elapsed()));
        Ok(part)
    };
    let part = fused().at(Stage::Fuse)?;
    mark(Stage::Fuse);
    Ok(part)
}

/// Native construction of a solid for export: the static recipe when it is
/// complete, the swept path when the body cuts continuous sweeps and was admitted.
pub(crate) fn construct_solid(session: &Session,sk: &Sketch,solid: usize,recipe: &cad::StaticRecipe,
    admission: Option<&Admission>,tolerance: Option<Tolerance>) -> Result<c_int,ExportRefusal> {
    if recipe.sweeps.is_empty() { return session.construct(&recipe.recipe).at(Stage::Blank); }
    let admission = admission.ok_or_else(|| ExportRefusal::at(Stage::Admission,format!("`{}`: a body with swept cuts is \
        built only once admitted to the generating-sweep class",sk.solids[solid].name)))?;
    construct_swept_body(session,sk,solid,recipe,admission,tolerance)
}
