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

/// Progress on stderr: a member takes minutes, and the JSON report owns stdout.
fn stage(message: &str) { eprintln!("solventc: {message}"); }

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
struct Reach { stations: [f64;2],start: Anchor,end: Anchor,faces: usize,radius: f64 }

/// A candidate sheet: contact positions on a row-major grid with the outward
/// normal of the cutter at each contact, in native millimetres.
#[derive(Debug)]
pub(crate) struct Sheet {
    pub points: Vec<[f64;3]>,
    pub normals: Vec<[f64;3]>,
    pub rows: usize,
    pub columns: usize,
    /// Contacts at row midpoints of every column, withheld from any fit.
    pub withheld: Vec<[f64;3]>,
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
        Ok(Self::contact_full(sweep,scale,sample,interval,near)?.map(|(p,_,t)| (p,t)))
    }
    /// Contact position, outward normal and time.
    fn contact_full(sweep: &SweepContacts,scale: f64,sample: Sample,interval: [f64;2],near: f64)
        -> Result<Option<([f64;3],[f64;3],f64)>,String> {
        let roots = sweep.at_point_normal_over(scaled(sample.position,1./scale),sample.normal,interval,1e-9)
            .map_err(|e| if e == "Degenerate" {
                "a cutter point's contact condition does not depend on the motion (the cutter is a revolution \
                 about an axis parallel to the motion's); such sweeps are not supported by the section construction".to_string()
            } else { e })?;
        Ok(roots.iter().min_by(|x,y| (x.root.time-near).abs().total_cmp(&(y.root.time-near).abs()))
            .map(|r| (scaled(r.contact.position,scale),r.contact.normal,r.root.time)))
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
        for c in 0..stations {
            let angle = TAU*c as f64/stations as f64;
            let mut hit_here = false;
            for profile in &self.profile(cutter,angle)? {
                let total = profile.augmented_length();
                let count = ((total/0.3).ceil() as usize).max(8);
                let mut queries = Vec::new();
                for i in 0..count {
                    let s = total*i as f64/count as f64;
                    let sample = self.sample(cutter,profile,s)?;
                    if let Some((p,_)) = Self::contact(sweep,scale,sample,declared,0.)? { queries.push((s,sample,p)); }
                }
                if queries.is_empty() { continue; }
                let states = inside(&queries.iter().map(|q| q.2).collect::<Vec<_>>())?;
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
    /// follow continuity down each column and across the first row.
    fn sheet(&self,cutter: &Cutter,sweep: &SweepContacts,scale: f64,reach: &Reach,margin: f64,station_margin: f64)
        -> Result<Sheet,String> {
        let [lo,hi] = reach.stations;
        let span = hi-lo+2.*station_margin;
        let columns = ((span*reach.radius/COLUMN_SPACING).ceil() as usize).clamp(24,200);
        let wide = [-PI,PI];
        let mut columns_data: Vec<Vec<([f64;3],[f64;3],f64)>> = Vec::with_capacity(columns);
        let mut withheld = Vec::new();
        let mut rows = 0;
        for c in 0..columns {
            let angle = lo-station_margin+span*c as f64/(columns-1) as f64;
            let loops = self.profile(cutter,angle)?;
            let profile = loops.iter().find(|l| l.index_of(reach.start.face).is_some() && l.index_of(reach.end.face).is_some())
                .ok_or("no section loop carries the profile at a station of the band")?;
            let [s0,s1] = Self::range(profile,reach,margin)?;
            if c == 0 { rows = (((s1-s0)/ROW_SPACING).ceil() as usize).clamp(24,240); }
            let mut column = Vec::with_capacity(rows);
            let mut near = columns_data.last().map(|previous| previous[0].2).unwrap_or(0.);
            for r in 0..rows {
                let s = s0+(s1-s0)*r as f64/(rows-1) as f64;
                let (p,n,time) = Self::contact_full(sweep,scale,self.sample(cutter,profile,s)?,wide,near)?
                    .ok_or("a cutter point has no contact time under the motion")?;
                if r % 3 == 1 && r+1 < rows {
                    let mid = self.sample(cutter,profile,s+(s1-s0)*0.5/(rows-1) as f64)?;
                    if let Some((q,_)) = Self::contact(sweep,scale,mid,wide,time)? { withheld.push(q); }
                }
                near = time;
                column.push((p,n,time));
            }
            columns_data.push(column);
        }
        let mut points = Vec::with_capacity(rows*columns);
        let mut normals = Vec::with_capacity(rows*columns);
        for r in 0..rows { for column in &columns_data { points.push(column[r].0); normals.push(column[r].1); } }
        Ok(Sheet {points,normals,rows,columns,withheld})
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
    for cell in session.cells(partition)? {
        let samples = session.samples(cell.solid,4,12)?;
        if samples.is_empty() { return Err(format!("a cell of volume {} has no interior sample",cell.volume)); }
        let mut verdict = None;
        for (point,boundary) in samples {
            let distance = (boundary*0.5).min(0.05);
            if distance <= 1e-4 { continue; }
            let probe = material.probe(point.map(|x| Interval::point(x).unwrap()),[1.,0.,0.],distance,
                Options {value_tolerance:distance/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))?;
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
    stage(&format!("`{name}`: contacts reach the blank over {:.1} degrees of stations and {} profile faces ({:?})",
        (reach.stations[1]-reach.stations[0]).to_degrees(),reach.faces,started.elapsed()));
    let (mut margin,mut station_margin) = (1.,(reach.stations[1]-reach.stations[0])*0.15);
    for _ in 0..6 {
        let sheet = session.sheet(&cutter,&sweep,scale,&reach,margin,station_margin)?;
        let mut boundary = Vec::new();
        for r in 0..sheet.rows { for c in 0..sheet.columns {
            if r == 0 || r+1 == sheet.rows || c == 0 || c+1 == sheet.columns { boundary.push(sheet.points[r*sheet.columns+c]); }
        } }
        if inside(&boundary)?.iter().any(|b| *b) { margin *= 1.6; station_margin *= 1.6; continue; }
        stage(&format!("`{name}`: sheet {}x{} ({:?})",sheet.rows,sheet.columns,started.elapsed()));
        return Ok(sheet);
    }
    Err(format!("`{name}`: the candidate sheet cannot be widened out of the blank"))
}

/// The candidate sheet of one swept solid against a native blank: the roll must
/// carry the cutter clear of the blank at both limits (no caps on this path),
/// and the grid is fitted as a native face with its withheld contact error.
pub(crate) fn swept_sheet(session: &Session,sk: &Sketch,swept: usize,blank: c_int) -> Result<(c_int,Sheet,f64),String> {
    let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else {
        return Err(format!("`{}` is not a continuous sweep",sk.solids[swept].name));
    };
    let scale = sk.units.length.ok_or("CAD construction requires an explicit length unit")?.1;
    let name = &sk.solids[swept].name;
    let cutter = session.cutter(sk,*source as usize)?;
    let family = Family::read(sk,*motion as usize)?;
    for (label,limit) in [("start",from.value),("end",to.value)] {
        let placed = session.place(cutter.solid,family.at(limit)?,scale)?;
        let overlap = session.common_volume(placed,blank)?;
        if overlap > 0. {
            return Err(format!("`{name}`: the declared roll leaves the cutter inside the blank at its {label} \
                ({:.1} degrees, {overlap:.3} mm³ overlap); declare a roll that carries it clear",limit.to_degrees()));
        }
    }
    let inside = |points: &[[f64;3]]| Ok(session.solid_contains(blank,points,1e-6)?.into_iter().map(|s| s == 1).collect());
    let sheet = swept_sheet_grid(session,sk,swept,&inside)?;
    let face = session.fit_sheet(&sheet.points,sheet.rows,sheet.columns)?;
    let mut error = 0_f64;
    for p in &sheet.withheld {
        error = error.max(session.face_parameters(face,*p,0.5)?.map(|(_,gap)| gap).unwrap_or(0.5));
    }
    stage(&format!("`{name}`: withheld contact error {error:.2e} mm"));
    Ok((face,sheet,error))
}

/// Construct a body whose cuts include continuous sweeps. Returns the native
/// solid handle in this session.
pub(crate) fn construct_swept_body(session: &Session,sk: &Sketch,body: usize,recipe: &cad::StaticRecipe) -> Result<c_int,String> {
    let scale = sk.units.length.ok_or("CAD construction requires an explicit length unit")?.1;
    let blank = session.construct(&recipe.recipe)?;
    stage(&format!("`{}`: static blank of {} operations",sk.solids[body].name,recipe.recipe.get("nodes").unwrap().arr().len()));
    let mut tools = Vec::new();
    let mut distinct: Vec<usize> = recipe.sweeps.iter().map(|s| s.swept).collect();
    distinct.sort(); distinct.dedup();
    for swept in distinct {
        let (face,_,_) = swept_sheet(session,sk,swept,blank)?;
        for cut in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            tools.push(session.place(face,cut.pose,scale)?);
        }
    }
    let started = std::time::Instant::now();
    let partition = session.split_solid(blank,&tools)?;
    stage(&format!("split the blank by {} sheets into {} cells ({:?})",tools.len(),session.solids(partition)?.len(),started.elapsed()));
    let started = std::time::Instant::now();
    let mut material = MaterialField::read(sk,body,1e-10)?.evaluator(4096);
    let (kept,removed) = classify(session,partition,&mut material)?;
    stage(&format!("classified {} material and {} removed cells ({:?})",kept.len(),removed.len(),started.elapsed()));
    if kept.is_empty() { return Err("no cell of the blank is material".into()); }
    let started = std::time::Instant::now();
    let part = session.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>())?;
    let [vertex,edge,_] = session.tolerances(part)?;
    stage(&format!("united the material: {:.6} mm³, {} faces, tolerances {vertex:.1e}/{edge:.1e} mm ({:?})",
        session.volume(part)?,session.faces(part)?.len(),started.elapsed()));
    Ok(part)
}

/// Native construction of a solid for export: the static recipe when it is
/// complete, the swept path when the body cuts continuous sweeps.
pub(crate) fn construct_solid(session: &Session,sk: &Sketch,solid: usize) -> Result<c_int,String> {
    let recipe = cad::recipe_static(sk,solid)?;
    if recipe.sweeps.is_empty() { session.construct(&recipe.recipe) }
    else { construct_swept_body(session,sk,solid,&recipe) }
}
