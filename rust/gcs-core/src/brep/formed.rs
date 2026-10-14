//! A body cut by static copies of cutters placed at turns of one placement about its blank's axis —
//! a formed gear's spaces, copied round it with no roll — built as one sector and patterned, as a
//! body with swept cuts is (`sweep::sector`): the blank between a side through the gaps the cuts
//! leave and that side turned a pitch, less the copies that fall in it, turned round the axis
//! (`pattern`). Cut one by one into the whole blank, every copy is arranged against the faces all
//! the others left, so a gear's time grows with the square of its teeth (48 spaces and 48 tip
//! reliefs took 95 s); a sector's is the few copies that reach it.
//!
//! Where a premise fails — the cuts not one indexing of three or more, the blank no revolution
//! about that axis, no side clear of the cuts — the body is built cut by cut, as before;
//! `SOLVENT_SECTOR_DEBUG` says why.
#[allow(unused_imports)]
use crate::fmath::Det;
use std::collections::BTreeMap;
use super::boolean::{split,Op};
use super::geom::Rigid;
use super::props::volume;
use super::sweep::sector::{sphere_centres,turned_to};
use super::sweep::sheet::surface_grid;
use super::topo::Brep;
use crate::envelope::Motion;
use crate::json::Json;
use crate::solid::sector::{self,Boundary,Frame,Slices};
use crate::space::{add,scale,sub};

type V = [f64;3];

/// The side's grid across its slices, and how finely the built side is read back.
const COLUMNS: usize = 9;
const READ_BACK: usize = 4;
/// The least clearance the side keeps from every cut (mm).
const MARGIN: f64 = 0.05;
/// The tolerance the split works to (mm).
const SPLIT_TOL: f64 = 1e-6;
/// How near a sector's far side's vertex must be to its near side's turned for the pattern (mm).
const PATTERN_MATCH: f64 = 1e-6;
/// How finely a cutter is read for the gaps its copies leave: its mesh's sag (mm), and the
/// spacing of the points read over each triangle (mm).
const SAG: f64 = 0.01;
const SPACING: f64 = 0.2;
/// How many slices a copy's material is filled in over, and the angle between its points (radians).
const FILL: usize = 64;
const FILL_STEP: f64 = 0.002;

/// Body node `n` of a recipe (millimetres) built as one sector patterned, `nodes` the recipe's
/// nodes by id and `built` every node before it: `None` where the body is no such body, and is
/// built cut by cut.
pub fn body(n: &Json,nodes: &BTreeMap<i64,&Json>,built: &BTreeMap<i64,Brep>) -> Option<Brep> {
    match sector_of(n,nodes,built) {
        Ok(b) => Some(b),
        Err(why) => {
            if std::env::var_os("SOLVENT_SECTOR_DEBUG").is_some() {
                eprintln!("sector: `{}` built cut by cut: {why}",n.get("name").map_or("",Json::as_str));
            }
            None
        }
    }
}

/// The ids a body node lists under `key`.
fn ids(n: &Json,key: &str) -> Vec<i64> {
    match n.get(key) { Some(Json::Arr(l)) => l.iter().map(Json::as_i64).collect(),Some(x) => vec![x.as_i64()],None => vec![] }
}

/// The nodes `root` is made from, itself last, in the recipe's order: the recipe of one operand.
fn recipe_of(root: i64,nodes: &BTreeMap<i64,&Json>) -> Json {
    let mut wanted = vec![root];
    let mut at = 0;
    while at < wanted.len() {
        if let Some(n) = nodes.get(&wanted[at]) {
            for key in ["stock","on","cut","bound","on_late","cut_late","source","sources"] {
                for id in ids(n,key) { if !wanted.contains(&id) { wanted.push(id) } }
            }
        }
        at += 1;
    }
    wanted.sort();
    crate::json::object([("nodes",Json::Arr(wanted.iter().filter_map(|id| nodes.get(id).map(|&n| n.clone())).collect())),
        ("root",Json::Int(root))])
}

/// A solid of revolution's meridian, for telling where a point is: its profile's edges as segments
/// in (distance from the line, distance along it), a point inside where a ray from it out from the
/// line crosses them an odd number of times. Its edges may be listed in any order and direction.
struct Meridian { origin: V,axis: V,segments: Vec<[[f64;2];2]> }

impl Meridian {
    fn new(profile: &super::build::Profile,origin: V,axis: V) -> Result<Meridian,String> {
        use super::build::ProfileEdge;
        let at = |p: V| { let d = sub(p,origin); let z = crate::space::dot(d,axis); [crate::space::norm(sub(d,scale(axis,z))),z] };
        let mut segments = Vec::new();
        for e in profile.loops.iter().flatten() {
            let points: Vec<V> = match *e {
                ProfileEdge::Line {a,b} => vec![a,b],
                ProfileEdge::Arc {frame,r,span} => {
                    let [w0,w1] = span.unwrap_or([0.,std::f64::consts::TAU]);
                    let k = ((w1-w0).abs()*r/0.01).ceil().clamp(8.,4096.) as usize;
                    (0..=k).map(|j| { let w = w0+(w1-w0)*j as f64/k as f64;
                        add(frame.o,add(scale(frame.x,r*w.dcos()),scale(frame.y,r*w.dsin()))) }).collect()
                }
                ProfileEdge::Spline(ref c) => {
                    let [t0,t1] = c.domain();
                    let long: f64 = c.poles.windows(2).map(|w| crate::space::distance(w[0],w[1])).sum();
                    let k = (long/0.01).ceil().clamp(8.,4096.) as usize;
                    (0..=k).map(|j| c.d2(t0+(t1-t0)*j as f64/k as f64).0).collect()
                }
            };
            segments.extend(points.windows(2).map(|w| [at(w[0]),at(w[1])]));
        }
        Ok(Meridian {origin,axis,segments})
    }
    fn inside(&self,p: V) -> bool {
        let d = sub(p,self.origin);
        let z = crate::space::dot(d,self.axis);
        let r = crate::space::norm(sub(d,scale(self.axis,z)));
        self.segments.iter().filter(|[a,b]| (a[1] > z) != (b[1] > z) && a[0]+(z-a[1])/(b[1]-a[1])*(b[0]-a[0]) > r).count() % 2 == 1
    }
}

/// A copy's points with its material filled in: a cut's boundary in the blank leaves its middle
/// empty (a ball cutting a ring shows only its flanks there), but in each slice a copy's material
/// spans every angle between its extreme points, a copy being less than a pitch wide. So each of
/// `FILL` slices of the copy's span gets points every `FILL_STEP` across that span, at its least
/// place, for the side to keep out of.
fn filled(points: &[V],frame: Frame,slices: Slices) -> Vec<V> {
    let read: Vec<[f64;3]> = points.iter().map(|&p| frame.coordinates(slices,p)).collect();
    let Some(first) = read.first() else { return Vec::new() };
    // angles unwrapped about the first point's: a copy spans less than a turn
    let unwrap = |phi: f64| first[2]+(phi-first[2]+std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)-std::f64::consts::PI;
    let lo = read.iter().map(|r| r[0]).fold(f64::INFINITY,f64::min);
    let hi = read.iter().map(|r| r[0]).fold(f64::NEG_INFINITY,f64::max);
    let h = ((hi-lo)/FILL as f64).max(f64::MIN_POSITIVE);
    let mut bins = vec![(f64::INFINITY,f64::NEG_INFINITY,f64::INFINITY,0.,0usize);FILL];
    for r in &read {
        let k = (((r[0]-lo)/h) as usize).min(FILL-1);
        let phi = unwrap(r[2]);
        let b = &mut bins[k];
        *b = (b.0.min(phi),b.1.max(phi),b.2.min(r[1]),b.3+r[0],b.4+1);
    }
    let mut out = points.to_vec();
    for (a,z,w,s,n) in bins.into_iter().filter(|b| b.4 > 0) {
        let steps = ((z-a)/FILL_STEP).ceil() as usize;
        out.extend((1..steps).map(|j| frame.point(slices,s/n as f64,w,a+(z-a)*j as f64/steps as f64)));
    }
    out
}

/// A placement's matrix as a still pose.
fn pose(placed: &Json) -> Result<Motion,String> {
    let m: Vec<f64> = placed.get("matrix").ok_or("a placement has no matrix")?.arr().iter().map(Json::as_f64).collect();
    let r = Rigid::from_rows(&m);
    Ok(Motion::rigid(r.r,r.t))
}

/// Points over a cutter's boundary within the box `[lo, hi]`, every `SPACING` across each triangle of
/// its mesh that reaches the box.
fn over(cutter: &Brep,[lo,hi]: [V;2]) -> Result<Vec<V>,String> {
    let mesh = super::mesh::mesh(cutter,SAG,0.3)?;
    let within = |p: V| (0..3).all(|k| p[k] >= lo[k] && p[k] <= hi[k]);
    let mut points = Vec::new();
    for t in &mesh.tris {
        let [a,b,c] = t.map(|i| mesh.pts[i as usize]);
        if (0..3).any(|k| a[k].max(b[k]).max(c[k]) < lo[k] || a[k].min(b[k]).min(c[k]) > hi[k]) { continue }
        let long = [crate::space::distance(a,b),crate::space::distance(b,c),crate::space::distance(c,a)].into_iter().fold(0.,f64::max);
        let k = (long/SPACING).ceil().max(1.) as usize;
        for i in 0..=k { for j in 0..=k-i {
            let (u,v) = (i as f64/k as f64,j as f64/k as f64);
            let p = add(a,add(scale(sub(b,a),u),scale(sub(c,a),v)));
            if within(p) { points.push(p) }
        } }
    }
    Ok(points)
}

fn sector_of(n: &Json,nodes: &BTreeMap<i64,&Json>,built: &BTreeMap<i64,Brep>) -> Result<Brep,String> {
    let clock = crate::clock::Instant::now();
    let said = |what: &str| if std::env::var_os("SOLVENT_SECTOR_DEBUG").is_some() {
        eprintln!("sector: {what} ({:.2} s)",clock.elapsed().as_secs_f64());
    };
    if n.get("kind").map(Json::as_str) != Some("body") { return Err("not a body".into()) }
    if ["on","bound","on_late","cut_late"].iter().any(|k| !ids(n,k).is_empty()) {
        return Err("the body adds or bounds as well as cuts".into())
    }
    let stock = ids(n,"stock").first().copied().ok_or("a body with no stock")?;
    let cuts = ids(n,"cut");
    if cuts.len() < 3 { return Err(format!("cut {} times",cuts.len())) }
    // the cuts by their cutter: each a placement of one source
    let mut by_source: BTreeMap<i64,Vec<(i64,Motion)>> = BTreeMap::new();
    for &c in &cuts {
        let node = nodes.get(&c).ok_or("a cut the recipe does not hold")?;
        if node.get("kind").map(Json::as_str) != Some("placed") { return Err("a cut that is no placement".into()) }
        let source = node.get("source").ok_or("a placement with no source")?.as_i64();
        by_source.entry(source).or_default().push((c,pose(node)?));
    }
    let groups: Vec<Vec<(i64,Motion)>> = by_source.into_values().collect();
    let poses: Vec<Vec<Motion>> = groups.iter().map(|g| g.iter().map(|&(_,m)| m).collect()).collect();
    let blank = built.get(&stock).ok_or("the stock is not built")?;
    let (lo,hi) = blank.bounds();
    let size = crate::space::distance(lo,hi);
    let indexing = sector::indexing(&poses,size)?;
    let count = indexing.count;
    let frame = Frame::new(indexing.origin,indexing.axis);
    // the blank, a revolution about the indexing's line
    let blank_recipe = recipe_of(stock,nodes);
    let meridian = match super::recipe::meridian_region(&blank_recipe,frame.radial(0.))? {
        Ok((profile,o,a)) => {
            let off = sub(sub(o,frame.origin),scale(frame.axis,crate::space::dot(sub(o,frame.origin),frame.axis)));
            if crate::space::norm(crate::space::cross(a,frame.axis)) > 1e-9 || crate::space::norm(off) > 1e-6*size {
                return Err("the blank is a revolution about another line than the indexing's".into())
            }
            Meridian::new(&profile,o,scale(a,1./crate::space::norm(a)))?
        }
        Err(why) => return Err(format!("the blank is no revolution: {why}")),
    };
    let inside = |p: V| meridian.inside(p);
    // each cutter's points in the blank at its first placement
    let cuts: Vec<Vec<V>> = groups.iter().map(|g| -> Result<Vec<V>,String> {
        let cutter = built.get(&g[0].0).ok_or("a placement is not built")?;
        Ok(over(cutter,[lo,hi])?.into_iter().filter(|&p| inside(p)).collect())
    }).collect::<Result<_,_>>()?;
    if cuts.iter().any(Vec::is_empty) { return Err("a cutter does not reach the blank".into()) }
    said("cutters read in the blank");
    let mut candidates: Vec<Slices> = sphere_centres(&blank_recipe,frame,size).into_iter().map(Slices::Spheres).collect();
    candidates.push(Slices::Planes);
    // the side through the widest gap between the copies, read in each slicing with each copy's
    // material filled in across every slice
    let mut chosen: Option<Boundary> = None;
    let mut reasons = Vec::new();
    for slices in candidates {
        let filled: Vec<V> = cuts.iter().flat_map(|points| filled(points,frame,slices)).collect();
        match Boundary::choose(&indexing,&filled,&[slices]) {
            Ok(b) => if chosen.as_ref().is_none_or(|c| b.clearance > c.clearance) { chosen = Some(b) },
            Err(e) => reasons.push(e),
        }
    }
    let chosen = chosen.ok_or_else(|| reasons.join("; "))?;
    if chosen.clearance < MARGIN {
        return Err(format!("the cuts leave a gap of {:.4} mm between neighbours at least, under {MARGIN} mm",chosen.clearance))
    }
    said("side chosen");
    // the copy of each cutter that falls in the sector
    let mut tools = Vec::new();
    for (k,points) in cuts.iter().enumerate() {
        let sectors: std::collections::BTreeSet<usize> = points.iter().map(|&p| chosen.sector(p)).collect();
        let [first] = sectors.iter().copied().collect::<Vec<_>>()[..] else {
            return Err(format!("a cutter's copy lies in {} sectors",sectors.len()))
        };
        let index = (count-first) % count;
        let j = indexing.indices[k].iter().position(|&i| i == index).ok_or("no copy in the sector")?;
        tools.push(built.get(&groups[k][j].0).ok_or("a placement is not built")?);
    }
    // the side, built and read back against the gaps
    let (s,w) = sector::section_span(frame,chosen.slices,[lo,hi],&inside)?;
    let (grid,rows) = chosen.grid(s,w,COLUMNS);
    let side = super::build::sheet(super::nurbs::interpolate_net(&grid,rows,COLUMNS,super::nurbs::Parametrization::ChordLength)
        .ok_or("the side's grid could not be interpolated")?)?;
    let read: Vec<V> = surface_grid(&side,READ_BACK*rows,READ_BACK*COLUMNS).into_iter().map(|(p,_)| p).filter(|&p| inside(p)).collect();
    let clear = chosen.clearance_of(&read)?;
    if clear < MARGIN { return Err(format!("the side as built passes {clear:.4} mm from a cut, under {MARGIN} mm")) }
    said("side built");
    let turn = indexing.turn(1);
    let other = side.moved(&Rigid::from_rows(&crate::solid::cad::placement_matrix(turn,1.)));
    // the blank turned about the axis till its seams lie opposite the sector, split by the sides
    let across = chosen.angle_at((chosen.span[0]+chosen.span[1])/2.)+indexing.pitch()/2.+std::f64::consts::PI;
    let turned = turned_to(blank,frame.origin,frame.axis,frame.radial(across))?;
    let whole = volume(&turned);
    let share = whole/count as f64;
    let halves = split(&turned,&side.joined(&other),SPLIT_TOL)?;
    if halves.len() != 2 { return Err(format!("the sides split the blank into {} cells, not two",halves.len())) }
    let mut piece = halves.into_iter().find(|c| (volume(c)-share).abs() <= 1e-5*whole)
        .ok_or_else(|| format!("neither cell of the split blank ({whole:.6} mm³) is its {share:.6} mm³ share"))?;
    said("blank split");
    // its edges where the split's traces left them, then each copy cut to the split's tolerance
    super::json::measure(&mut piece);
    for tool in tools { piece = super::recipe::combined(&piece,tool,Op::Cut,SPLIT_TOL)?; }
    piece.check(1e-6).map_err(|e| format!("the sector is invalid: {e}"))?;
    super::json::measure(&mut piece);
    said("sector cut");
    Ok(super::pattern::pattern(&piece,frame.origin,frame.axis,count,PATTERN_MATCH)?.solid)
}
