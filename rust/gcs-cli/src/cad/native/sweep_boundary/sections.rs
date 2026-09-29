//! The native cutter sectioned by meridian half-planes of its axis: each section loop is the
//! profile a station samples, walked in augmented arc length (position length plus turning across
//! a convex corner's fan), and where the cutter's declared-roll contacts enter the blank fixes the
//! band of stations and the walk the sheet spans.
use super::*;

/// `a` over its length, refusing a zero or non-finite one.
fn unit(a: [f64;3]) -> Result<[f64;3],String> {
    let n = norm(a);
    if n <= 0. || !n.is_finite() { return Err("degenerate direction".into()); }
    Ok(a.map(|v| v/n))
}

/// The augmented length scale of a corner: one radian of turning counts as this
/// many millimetres of profile, so a fan receives rows like an arc of that radius.
const TURN_SCALE: f64 = 0.05;
const CHAIN_TOLERANCE: f64 = 1e-5;
const SAMPLES_PER_EDGE: usize = 64;

/// One edge of a section loop on one native face, sampled along the walk.
#[derive(Clone)]
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

#[derive(Clone)]
/// A corner between consecutive pieces. `angle` is the turning of the outward
/// normal there when the corner is convex, and zero otherwise.
struct Corner { position: [f64;3],normals: [[f64;3];2],angle: f64 }

/// One planar section loop, oriented consistently across stations: pieces in
/// walk order, corner k after piece k (cyclic). A loop of a cutter that is a solid of revolution
/// about its own axis is its first section turned by `turn` (radians about the axis): its pieces'
/// edges are that section's, and a point read on one is turned.
#[derive(Clone)]
pub(super) struct Loop { pieces: Vec<Piece>,corners: Vec<Corner>,turn: Option<Turn> }

/// A turn about the cutter's axis: the line through `origin` along `axis` (unit), by `angle`.
#[derive(Clone,Copy)]
struct Turn { origin: [f64;3],axis: [f64;3],cos: f64,sin: f64 }
impl Turn {
    fn new(cutter: &Cutter,angle: f64) -> Turn { Turn {origin:cutter.origin,axis:cutter.axis,cos:angle.cos(),sin:angle.sin()} }
    /// A direction turned (Rodrigues).
    fn vector(&self,v: [f64;3]) -> [f64;3] {
        let (k,c,s) = (self.axis,self.cos,self.sin);
        let (kxv,kv) = (cross(k,v),dot(k,v));
        std::array::from_fn(|i| v[i]*c+kxv[i]*s+k[i]*kv*(1.-c))
    }
    fn point(&self,p: [f64;3]) -> [f64;3] {
        let r = self.vector(sub(p,self.origin));
        std::array::from_fn(|i| self.origin[i]+r[i])
    }
    fn apply(turn: Option<Turn>,p: [f64;3],direction: bool) -> [f64;3] {
        match turn { None => p, Some(t) => if direction { t.vector(p) } else { t.point(p) } }
    }
}

/// Where a profile row is: on a piece at arc length, or in a corner's fan.
enum Place { Piece {index: usize,s: f64},Fan {index: usize,fraction: f64} }

impl Loop {
    pub(super) fn augmented_length(&self) -> f64 {
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
    pub(super) fn index_of(&self,face: c_int) -> Option<usize> { self.pieces.iter().position(|p| p.face == face) }
    /// This loop (a first section) turned: its points and corners turned, its edges its own.
    fn turned(&self,turn: Turn) -> Loop {
        Loop {pieces:self.pieces.iter().map(|p| Piece {edge:p.edge,face:p.face,reversed:p.reversed,
            points:p.points.iter().map(|&q| turn.point(q)).collect(),lengths:p.lengths.clone()}).collect(),
            corners:self.corners.iter().map(|c| Corner {position:turn.point(c.position),normals:c.normals.map(|n| turn.vector(n)),
            angle:c.angle}).collect(),turn:Some(turn)}
    }
    /// Augmented length of an anchor on this loop.
    fn resolve(&self,anchor: Anchor) -> Result<f64,String> {
        let index = self.index_of(anchor.face).ok_or("a profile face is missing at this station")?;
        Ok(self.start_of(index)+anchor.fraction.clamp(0.,1.)*self.pieces[index].length())
    }
}

/// A place on the profile that survives the section changing shape between
/// stations: a face and a fraction of its piece.
#[derive(Clone,Copy,Debug)]
pub(super) struct Anchor { pub(super) face: c_int,fraction: f64 }

/// The band and walk chosen from where the cutter's declared-roll contacts enter
/// the blank: station angles, the walk's anchors in loop order, and the profile's
/// mean distance from the axis for column spacing.
pub(super) struct Reach { pub(super) stations: [f64;2],pub(super) start: Anchor,pub(super) end: Anchor,
    pub(super) faces: usize,pub(super) radius: f64,
    /// Where its time went: sections, contacts, blank queries (seconds) and points queried.
    pub(super) spent: [f64;3],pub(super) queried: usize }

/// The cutter as the section machinery needs it: the native solid, its faces in
/// the stable enumeration the sections index, and its axis. Where every revolution it is made of
/// is a whole one about that axis and every placement keeps the axis where it is, the solid is
/// one of revolution about the axis, and its section at any station is its section at `first`
/// turned: `first` is sectioned once, the first time a station is asked for.
pub(super) struct Cutter { pub(super) solid: c_int,faces: Vec<c_int>,origin: [f64;3],axis: [f64;3],side: [f64;3],
    revolution: bool,first: std::sync::OnceLock<(f64,Vec<Loop>)>,
    /// Every other cutter's sections, by station angle: a sheet's rows placed a second way are
    /// traced over the stations the first placement sectioned.
    sections: std::sync::Mutex<BTreeMap<u64,Vec<Loop>>> }

/// The station a solid of revolution is sectioned at: half a step of the reach's stations off the
/// side, where a revolution's seam (in the plane of its profile) is not.
const FIRST_STATION: f64 = TAU*0.5/96.;

impl Session {
    pub(super) fn cutter(&self,recipe: &gcs_core::json::Json) -> Result<Cutter,String> {
        let recipe = recipe.clone();
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
        let revolution = std::env::var("SOLVENT_SECTIONS").map_or(true,|v| v != "each") && of_revolution(&recipe,origin,axis);
        Ok(Cutter {solid,faces:self.faces(solid)?,origin,axis,side,revolution,first:std::sync::OnceLock::new(),
            sections:Default::default()})
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
    /// corner fans where consecutive faces meet at a convex dihedral. A cutter of revolution about
    /// its axis is sectioned once, and every station is that section turned.
    pub(super) fn profile(&self,cutter: &Cutter,angle: f64) -> Result<Vec<Loop>,String> {
        if !cutter.revolution {
            if let Some(loops) = cutter.sections.lock().unwrap_or_else(|e| e.into_inner()).get(&angle.to_bits()) { return Ok(loops.clone()); }
            let loops = self.section_loops(cutter,angle)?;
            if std::env::var_os("SOLVENT_SECTION_COUNT").is_some() { eprintln!("section at {angle:.17}"); }
            cutter.sections.lock().unwrap_or_else(|e| e.into_inner()).insert(angle.to_bits(),loops.clone());
            return Ok(loops);
        }
        if cutter.first.get().is_none() {
            let loops = self.section_loops(cutter,FIRST_STATION)?;
            let _ = cutter.first.set((FIRST_STATION,loops));
        }
        let (at,loops) = cutter.first.get().expect("the first section is kept");
        let turn = Turn::new(cutter,angle-at);
        Ok(loops.iter().map(|l| l.turned(turn)).collect())
    }

    /// The section loops the kernel cuts at one station.
    fn section_loops(&self,cutter: &Cutter,angle: f64) -> Result<Vec<Loop>,String> {
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
            loops.push(Loop {pieces,corners,turn:None});
        }
        Ok(loops)
    }

    /// The cutter point and its outward normal at augmented length `s` of a section. A fan whose
    /// normals cancel carries no direction: no one contact time there, like a degenerate root.
    pub(super) fn sample(&self,cutter: &Cutter,profile: &Loop,s: f64) -> Result<Sample,TraceError> {
        match profile.locate(s) {
            Place::Piece {index,s} => {
                let piece = &profile.pieces[index];
                let position = self.edge_point(piece.edge,piece.parameter(s))?;
                let normal = self.normal_at(cutter,piece.face,position)?;
                Ok(Sample {position:Turn::apply(profile.turn,position,false),normal:Turn::apply(profile.turn,normal,true)})
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
    pub(super) fn reach(&self,cutter: &Cutter,tracer: &Tracer) -> Result<Reach,String> {
        let (sweep,scale,inside) = (tracer.sweep,tracer.scale,tracer.inside);
        let declared = sweep.domain();
        let stations = 96;
        let mut inside_stations = Vec::new();
        let mut face_hits: std::collections::BTreeMap<c_int,(f64,f64)> = Default::default();
        let mut order: Option<Vec<c_int>> = None;
        let (mut radius_sum,mut radius_count) = (0.,0);
        let (mut spent,mut queried) = ([0.;3],0);
        // Half a step off the side: a revolution's seam lies in the plane of its profile, and
        // a section plane containing a seam loses that face's section. Every station's section
        // first, on every core.
        let clock = std::time::Instant::now();
        let mut sections = gcs_core::par::indices(stations,|c| self.profile(cutter,TAU*(c as f64+0.5)/stations as f64)).into_iter();
        spent[0] += clock.elapsed().as_secs_f64();
        for c in 0..stations {
            let mut hit_here = false;
            let loops = sections.next().expect("a section a station")?;
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
    pub(super) fn range(profile: &Loop,reach: &Reach,margin: f64) -> Result<[f64;2],String> {
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

/// Whether a cutter's recipe makes a solid of revolution about the line through `origin` along
/// `axis` (millimetres, unit): every revolution a whole turn about that line, every placement a
/// rigid motion keeping the line where it is, and nothing else but Booleans of them.
fn of_revolution(recipe: &gcs_core::json::Json,origin: [f64;3],axis: [f64;3]) -> bool {
    let read = |node: &gcs_core::json::Json,key: &str| -> Option<Vec<f64>> {
        Some(node.get(key)?.arr().iter().map(gcs_core::json::Json::as_f64).collect())
    };
    let size = 1.+norm(origin);
    let on_line = |p: [f64;3]| { let r = sub(p,origin); norm(sub(r,scaled(axis,dot(r,axis)))) <= 1e-9*size };
    let along = |d: [f64;3]| norm(cross(d,axis)) <= 1e-12*norm(d) && norm(d) > 0.;
    let Some(nodes) = recipe.get("nodes") else { return false };
    nodes.arr().iter().all(|node| match node.get("kind").map(gcs_core::json::Json::as_str) {
        Some("revolve") => {
            let (Some(o),Some(a),Some(angle)) = (read(node,"origin"),read(node,"axis"),node.get("angle")) else { return false };
            (angle.as_f64().abs()-TAU).abs() <= 1e-12 && on_line([o[0],o[1],o[2]]) && along([a[0],a[1],a[2]])
        }
        Some("placed") => {
            let Some(m) = read(node,"matrix") else { return false };
            if m.len() != 12 { return false }
            let map = |p: [f64;3],w: f64| -> [f64;3] { std::array::from_fn(|r| m[4*r]*p[0]+m[4*r+1]*p[1]+m[4*r+2]*p[2]+m[4*r+3]*w) };
            on_line(map(origin,1.)) && along(map(axis,0.))
        }
        Some("body") => true,
        _ => false,
    })
}
