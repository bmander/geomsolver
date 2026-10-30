//! The blank cut by a sweep's sheets, by this kernel alone: the blank split by every sheet placed
//! where it cuts (`split_by`, a batch of sheets that meet none of each other's boxes at a time),
//! each cell judged material or removed at its deepest interior samples (`judge`, by a probe the
//! caller hands it: its material field, or a sweep's closed forms), the cells held to the cell
//! contract (`cells`) and the one material cell kept — the body built `whole`, or, where its cuts
//! are turns of one placement about one axis and the blank reads alike under that turn, as one
//! `Sector` between a side through the gaps the cuts leave (`premises`, `construct`, over the
//! readings of `sector`) and that side turned by a pitch, which a caller patterns round the axis
//! (`pattern`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::Say;
use super::envelope::Motion;
use super::refusal::Stage;
use super::sector::{self as reading,Boundary,Frame,Slices};
use super::sheet::{Fitted,surface_grid};
use crate::boolean::{overlap,split};
use crate::geom::{Box3,Rigid};
use crate::props::volume;
use crate::query::interior;
use crate::recipe::{Node,Recipe,Stroke};
use crate::topo::Brep;
use crate::Kind;
use crate::util::space::{cross,distance,dot,norm,scale,sub};

type V = [f64;3];

/// What a probe says of a ball about a point: all material, all removed, or neither (what it
/// read instead).
#[derive(Clone,Debug,PartialEq)]
pub enum Ball { Material,Removed,Other(String) }

/// A caller's probe of the body's material: for each (point, radius) asked (millimetres), what
/// the ball of that radius about the point is, or why it could not be read.
pub type Probe<'a> = &'a (dyn Fn(&[(V,f64)]) -> Vec<Result<Ball,String>>+Sync);

/// The side's grid across its slices, and how finely the built side is read back (the native
/// host's sector too).
pub const COLUMNS: usize = 9;
pub const READ_BACK: usize = 4;
/// The least clearance the side keeps from every cut beyond twice the sheets' fit error (mm).
pub const MARGIN: f64 = 0.05;
/// The most points of the side the material field is asked about.
pub const PROBES: usize = 160;
/// The tolerance the split works to (mm).
const SPLIT_TOL: f64 = 1e-6;

/// The sector: its material, and the turn that patterns it (`count` copies a `pitch` apart about
/// the line through `origin`, mm, along `axis`).
pub struct Sector { pub piece: Brep,pub origin: V,pub axis: V,pub count: usize,pub pitch: f64 }

/// The centres of the spheres the recipe's revolutions make where they are on the axis: the
/// slicings a bevel gear's end spheres suggest (mm).
pub fn sphere_centres(recipe: &Recipe,frame: Frame,size: f64) -> Vec<V> {
    let on_axis = |p: V| { let d = sub(p,frame.origin); norm(sub(d,scale(frame.axis,dot(d,frame.axis)))) <= 1e-6*size };
    let mut centres: Vec<V> = Vec::new();
    for named in &recipe.nodes {
        let &Node::Revolve {ref profile,origin: o,axis: a,..} = &named.node else { continue };
        // a circle about a point of the revolution's own axis turns into a sphere about it
        let on_revolution = |p: V| norm(cross(sub(p,o),a)) <= 1e-6*size*norm(a);
        for edge in profile.loops.iter().flatten() {
            let Stroke::Circle {center: c,..} = edge.shape else { continue };
            if on_revolution(c) && on_axis(c) && !centres.iter().any(|&k| distance(k,c) <= 1e-6*size) { centres.push(c); }
        }
    }
    centres
}

/// A placed sheet's faces under the key of the cut that places it (its key), each
/// still named by the cutter's face that made it (#212): `indexed` and `flank`, as `solid::exact`
/// names a placed copy's.
pub fn keyed(mut sheet: Brep,key: &str) -> Brep {
    for f in &mut sheet.faces { f.name = crate::recipe::name_under(key,&f.name); }
    sheet
}

/// `cells` split by every sheet of `tools` (each with its box) a batch at a time, each sheet in the
/// first batch whose boxes all stand more than `gap` from its own: a batch's sheets meet none of
/// each other, so one split by all of them at once is the splits by each in turn, and a cell is
/// arranged against a batch where it was against a sheet. The cells, and the most sheets a split
/// took at once.
fn split_by(mut cells: Vec<Brep>,tools: &[(Brep,Box3)],gap: f64) -> Result<(Vec<Brep>,usize),String> {
    let mut batches: Vec<Vec<usize>> = Vec::new();
    for k in 0..tools.len() {
        match batches.iter_mut().find(|b| b.iter().all(|&j| !overlap(&tools[k].1,&tools[j].1,gap))) {
            Some(b) => b.push(k),
            None => batches.push(vec![k]),
        }
    }
    for batch in &batches {
        let mut sheet = tools[batch[0]].0.clone();
        for &k in &batch[1..] { sheet.join(&tools[k].0); }
        let parts = crate::util::par::map(&cells,|c| split(c,&sheet,SPLIT_TOL));
        cells = parts.into_iter().collect::<Result<Vec<_>,_>>()?.into_iter().flatten().collect();
    }
    Ok((cells,batches.iter().map(Vec::len).max().unwrap_or(0)))
}

/// The blank of `recipe` as its meridian region turned once from the half-plane along `seam`, or
/// by its Booleans turned about the axis till its seams lie there.
pub fn blank_from(recipe: &Recipe,origin: V,axis: V,seam: V) -> Result<Brep,String> {
    match recipe.meridian(seam) {
        Ok((b,o,a)) => {
            if norm(cross(a,axis)) > 1e-9 || norm(cross(sub(o,origin),axis)) > 1e-6 {
                return Err("the blank's meridian is about another line than the indexing's".into())
            }
            Ok(b)
        }
        Err(e) if e.kind == Kind::Refused => turned_to(&recipe.build()?,origin,axis,seam),
        Err(e) => Err(e.into()),
    }
}

/// A solid of revolution about the line through `origin` along `axis` turned about it until its
/// faces' seams lie in the half-plane along `seam`; refused where its seams are not in one.
pub fn turned_to(b: &Brep,origin: V,axis: V,seam: V) -> Result<Brep,String> {
    let mut found: Option<V> = None;
    for f in &b.faces {
        let uses: Vec<u32> = f.loops.iter().flatten().map(|c| c.edge).collect();
        for (k,&e) in uses.iter().enumerate() {
            if !uses[k+1..].contains(&e) { continue }
            let edge = &b.edges[e as usize];
            let crate::topo::EdgeCurve::Curve(_) = &edge.curve else { continue };
            let p = edge.point((edge.t[0]+edge.t[1])/2.,&b.vertices);
            let r = sub(p,origin);
            let r = sub(r,scale(axis,dot(r,axis)));
            let l = norm(r);
            if l < 1e-9 { continue }
            let r = r.map(|x| x/l);
            match found { None => found = Some(r),Some(q) if dot(q,r) < 1.-1e-12 => return Err("the blank's seams are not in one half-plane".into()),_ => {} }
        }
    }
    let Some(r) = found else { return Ok(b.clone()) };
    Ok(b.moved(&Rigid::turn(origin,axis,dot(axis,cross(r,seam)).datan2(dot(r,seam)))))
}

/// One cell judged: its volume, its deepest interior sample, its index.
pub struct Cell { pub index: usize,pub point: V,pub volume: f64 }

/// Cells of the given volumes judged by the material field at their interior samples (points with
/// a lower bound on their distance from the cell's boundary, deepest first): every sample far
/// enough from its cell's boundary is probed, at half that distance (at most 0.05 mm), and all of a
/// cell's must agree. An unresolved probe or a cell reading both ways refuses.
pub fn judge(volumes: &[f64],sampled: Vec<Vec<(V,f64)>>,probe: Probe) -> Result<(Vec<Cell>,Vec<Cell>),String> {
    let (mut kept,mut removed) = (Vec::new(),Vec::new());
    let asked: Vec<(V,f64)> = sampled.iter().flatten().map(|&(p,b)| (p,(b*0.5).min(0.05))).filter(|&(_,d)| d > 1e-4).collect();
    let mut answers = probe(&asked).into_iter();
    for (k,(&volume,samples)) in volumes.iter().zip(sampled).enumerate() {
        if samples.is_empty() { return Err(format!("a cell of volume {volume} has no interior sample")) }
        let cell = Cell {index:k,point:samples[0].0,volume};
        let mut verdict: Option<(bool,V)> = None;
        let deepest = samples[0];
        for (point,boundary) in samples {
            if (boundary*0.5).min(0.05) <= 1e-4 { continue }
            let probe = answers.next().expect("an answer a probe asked")?;
            let inside = match probe {
                Ball::Material => true,
                Ball::Removed => false,
                Ball::Other(state) => return Err(format!("the material at {point:?} ({boundary:.3} mm from a cell boundary) is {state}")),
            };
            match verdict {
                None => verdict = Some((inside,point)),
                Some((previous,at)) if previous != inside => return Err(format!("a cell of volume {} reads both material and removed: \
                    a sheet did not separate it (at {:?} and {:?})",cell.volume,at.map(|x| (x*1e4).round()/1e4),point.map(|x| (x*1e4).round()/1e4))),
                _ => {}
            }
        }
        match verdict {
            Some((true,_)) => kept.push(cell),
            Some((false,_)) => removed.push(cell),
            None => return Err(format!("a cell of volume {} has no sample clear of its boundary (the deepest {:.1e} mm in, at {:?})",
                cell.volume,deepest.1,deepest.0.map(|x| (x*1e4).round()/1e4))),
        }
    }
    Ok((kept,removed))
}

/// What a sector is cut from once its premises hold: the placements' turn, the two sides, and a
/// sheet of each sweep placed in the sector.
pub struct Premises { frame: Frame,n: usize,pitch: f64,across: f64,side: Brep,other: Brep,tools: Vec<Brep> }

/// The sweeps a body cuts, as a sector's premises read them: each distinct sweep's placements
/// (`poses`, in model units) with the key each places its faces under, its name, and its fitted
/// sheet; `scale` millimetres a model unit.
pub struct Cuts<'a> { pub poses: &'a [Vec<Motion>],pub keys: &'a [Vec<String>],pub names: &'a [String],pub sheets: &'a [Fitted],
    pub scale: f64 }

/// A placement's rigid motion in millimetres, its pose read in model units at `scale` millimetres
/// a unit.
pub fn placed(pose: Motion,scale: f64) -> Rigid {
    let m = |p: V| pose.point(p.map(|x| x/scale)).map(|x| x*scale);
    let o = m([0.;3]);
    let column = |k: usize| { let mut e = [0.;3]; e[k] = 1.; pose.vector(e) };
    let (x,y,z) = (column(0),column(1),column(2));
    Rigid {r:[[x[0],y[0],z[0]],[x[1],y[1],z[1]],[x[2],y[2],z[2]]],t:o}
}

/// Whether the body can be built as one sector, and what it is cut from, or why not — the
/// placements not turns of one about one axis, the blank not alike under the turn, no side clear of
/// the cuts through their gaps, a side the probe reads as not material. `blank` the static blank
/// (mm), `recipe` its recipe, `field` its signed field (mm).
pub fn premises(cuts: &Cuts,recipe: &Recipe,blank: &Brep,field: &(dyn Fn(V) -> f64+Sync),probe: Probe,say: &Say)
    -> Result<Premises,String> {
    let started = crate::util::clock::Instant::now();
    let (scale_mm,sheets,poses) = (cuts.scale,cuts.sheets,cuts.poses);
    let (lo,hi) = blank.bounds();
    let bounds = [lo,hi];
    let size = distance(lo,hi);
    let indexing = reading::indexing(poses,size/scale_mm)?;
    let n = indexing.count;
    let frame = Frame::new(indexing.origin.map(|x| x*scale_mm),indexing.axis);
    let mm = |m: Motion,p: V| m.point(p.map(|x| x/scale_mm)).map(|x| x*scale_mm);
    let inside = |p: V| field(p) < 0.;
    // the blank reads alike under one pitch's turn, at points spread over its box
    let turn = indexing.turn(1);
    let mut rng = crate::util::rng::Rng::new(0x5ec7);
    for _ in 0..256 {
        let p: V = std::array::from_fn(|k| rng.uniform(bounds[0][k],bounds[1][k]));
        let (a,b) = (field(p),field(mm(turn,p)));
        if (a-b).abs() > 1e-9*size { return Err(format!("the blank does not read alike turned by one pitch (at {p:?}, {a} against {b})")) }
    }
    // each sweep's contacts in the blank at its first placement
    let cut_points: Vec<Vec<V>> = sheets.iter().enumerate()
        .map(|(k,s)| s.sheet.points.iter().chain(&s.sheet.withheld).map(|&p| mm(poses[k][0],p)).filter(|&p| inside(p)).collect()).collect();
    let all: Vec<V> = cut_points.iter().flatten().copied().collect();
    let mut candidates: Vec<Slices> = sphere_centres(recipe,frame,size).into_iter().map(Slices::Spheres).collect();
    candidates.push(Slices::Planes);
    let chosen = Boundary::choose(&indexing,&all,&candidates)?;
    let fit = sheets.iter().map(|s| s.error).fold(0.,f64::max);
    let margin = MARGIN+2.*fit;
    (say.stage)(&format!("the sector: {n} placements a {:.4} degree pitch apart; its side sliced by {}, {:.4} mm clear of the cuts' \
        {} contacts at least (against {margin:.4} mm)",indexing.pitch().to_degrees(),
        match chosen.slices { Slices::Spheres(_) => "spheres about the axis",Slices::Planes => "planes square to the axis" },chosen.clearance,all.len()));
    if chosen.clearance < margin {
        return Err(format!("the cuts leave a gap of {:.4} mm between neighbours at least, under the {margin:.4} mm the side keeps",chosen.clearance))
    }
    // the placements whose cuts fall in the sector: one of each sweep
    let mut tools = Vec::new();
    for (k,points) in cut_points.iter().enumerate() {
        let sectors: std::collections::BTreeSet<usize> = points.iter().map(|&p| chosen.sector(p)).collect();
        let [first] = sectors.iter().copied().collect::<Vec<_>>()[..] else {
            return Err(format!("the cut of `{}` lies in {} sectors",cuts.names[k],sectors.len()))
        };
        let index = (n-first) % n;
        let j = indexing.indices[k].iter().position(|&i| i == index).ok_or("no placement in the sector")?;
        tools.push(keyed(sheets[k].faces.moved(&placed(poses[k][j],scale_mm)),&cuts.keys[k][j]));
    }
    // the side, built and read back against the gaps and the material
    let (s,w) = reading::section_span(frame,chosen.slices,bounds,&inside)?;
    let (grid,rows) = chosen.grid(s,w,COLUMNS);
    let side = crate::build::sheet(crate::nurbs::interpolate_net(&grid,rows,COLUMNS,crate::nurbs::Parametrization::ChordLength)
        .ok_or("the side's grid could not be interpolated")?)?;
    let read: Vec<V> = surface_grid(&side,READ_BACK*rows,READ_BACK*COLUMNS).into_iter().map(|(p,_)| p).filter(|&p| inside(p)).collect();
    let clear = chosen.clearance_of(&read)?;
    if clear < margin { return Err(format!("the side as built passes {clear:.4} mm from a cut, under {margin:.4} mm")) }
    let radius = margin/2.;
    let deep: Vec<V> = read.iter().copied().filter(|&p| field(p) < -radius).collect();
    let step = deep.len().div_ceil(PROBES).max(1);
    let probed_at: Vec<V> = deep.iter().copied().step_by(step).collect();
    let asked: Vec<(V,f64)> = probed_at.iter().map(|&p| (p,radius)).collect();
    for (&p,ball) in probed_at.iter().zip(probe(&asked)) {
        if ball? != Ball::Material {
            return Err(format!("the material field reads the side at {:?} as not material",p.map(|x| (x*1e3).round()/1e3)))
        }
    }
    (say.stage)(&format!("the side: {rows}x{COLUMNS} nodes, read back {clear:.4} mm clear of the cuts at {} points in the blank, \
        {} of them material by the field ({:?})",read.len(),probed_at.len(),started.elapsed()));
    let other = side.moved(&placed(turn,scale_mm));
    // the blank made again about the axis, its faces' parameters starting opposite the sector
    let across = chosen.angle_at((chosen.span[0]+chosen.span[1])/2.)+indexing.pitch()/2.+std::f64::consts::PI;
    Ok(Premises {frame,n,pitch:indexing.pitch(),across,side,other,tools})
}

/// The sector its premises describe: the blank of `recipe` made about the axis, split by the two
/// sides into the sector and it by the sheets, its cells judged by `probe` and the one material
/// cell kept, `sweeps` distinct sweeps each cutting it once.
pub fn construct(recipe: &Recipe,sweeps: usize,premises: Premises,probe: Probe,say: &Say) -> Result<Sector,String> {
    let Premises {frame,n,pitch,across,side,other,tools} = premises;
    let clock = crate::util::clock::Instant::now();
    let turned = blank_from(recipe,frame.origin,frame.axis,frame.radial(across))?;
    let whole = volume(&turned);
    let halves = split(&turned,&side.joined(&other),SPLIT_TOL)?;
    let volumes: Vec<f64> = halves.iter().map(volume).collect();
    let share = whole/n as f64;
    let Some(wedge) = halves.iter().zip(&volumes).find(|(_,v)| (*v-share).abs() <= 1e-5*whole).map(|(c,_)| c.clone()) else {
        return Err(format!("the sides split the blank ({whole:.6} mm³) into {} cells of {volumes:.6?} mm³, none its {share:.6} mm³ share",
            halves.len()))
    };
    if halves.len() != 2 { return Err(format!("the sides split the blank into {} cells, not two",halves.len())) }
    let (count,size) = (tools.len(),wedge.size());
    let (cells,_) = split_by(vec![wedge],&tools.into_iter().map(|t| { let b = t.bounds(); (t,b) }).collect::<Vec<_>>(),1e-3*size)?;
    for c in &cells { c.check(1e-6).map_err(|e| format!("a cell of the split is invalid: {e}"))?; }
    (say.stage)(&format!("split the blank by the sides into the sector of {share:.6} mm³, and it by {} sheets into {} cells ({:?})",
        count,cells.len(),clock.elapsed()));
    (say.mark)(Stage::Split);
    let piece = kept(cells,&vec![1;sweeps],"sector",true,probe,say)?;
    Ok(Sector {piece,origin:frame.origin,axis:frame.axis,count:n,pitch})
}

/// The placed sheets of `tools` (each with its box) whose boxes meet `blank`'s.
pub fn reaching(blank: &Brep,tools: Vec<(Brep,Box3)>) -> Vec<(Brep,Box3)> {
    let (lo,hi) = blank.bounds();
    tools.into_iter().filter(|(_,(a,b))| (0..3).all(|i| a[i] <= hi[i] && b[i] >= lo[i])).collect()
}

/// The blank of `recipe` (built already as `blank`) made again with its seams opposite the cuts,
/// where it is a solid of revolution, as a sector's blank is: a sheet across a face's seam is a
/// split this kernel does not finish. `tools` are the placed sheets with their boxes.
pub fn facing(recipe: &Recipe,blank: &Brep,tools: &[(Brep,Box3)]) -> Result<Brep,String> {
    Ok(match recipe.meridian([1.,0.,0.]) {
        Ok((_,origin,axis)) => turned_from(blank,origin,axis,tools,&|o,a,seam| blank_from(recipe,o,a,seam))?,
        Err(e) if e.kind == Kind::Refused => blank.clone(),
        Err(e) => return Err(e.into()),
    })
}

/// `blank`, a solid of revolution about the line through `origin` along `axis`, made again by
/// `again` with its seams in the half-plane opposite the middle of the cuts' boxes; itself where
/// that middle is on the axis.
fn turned_from(blank: &Brep,origin: V,axis: V,tools: &[(Brep,Box3)],again: &dyn Fn(V,V,V) -> Result<Brep,String>) -> Result<Brep,String> {
    let centres: Vec<V> = tools.iter().map(|(_,(a,b))| std::array::from_fn(|k| (a[k]+b[k])/2.)).collect();
    let c: V = std::array::from_fn(|k| centres.iter().map(|p| p[k]).sum::<f64>()/centres.len().max(1) as f64);
    let a = scale(axis,1./norm(axis));
    let r = sub(sub(c,origin),scale(a,dot(sub(c,origin),a)));
    if norm(r) > 1e-9*blank.size() { again(origin,a,scale(r,-1./norm(r))) } else { Ok(blank.clone()) }
}

/// A blank given only as its boundary turned about its own axis till its seams lie opposite the
/// cuts, where every face of it is a surface of revolution about one line (a plane square to it,
/// or a cylinder, cone, sphere or torus about it), as `facing` makes a recipe's again; otherwise as
/// it is.
pub fn turned_away(blank: &Brep,tools: &[(Brep,Box3)]) -> Result<Brep,String> {
    let size = blank.size();
    let mut line: Option<(V,V)> = None;
    let mut planes: Vec<V> = Vec::new();
    for f in &blank.faces {
        match (&f.surface,f.surface.axis()) {
            (crate::geom::Surface::Plane(frame),_) => planes.push(frame.z),
            (_,Some((o,a))) => match line {
                None => line = Some((o,a)),
                Some((p,b)) => if norm(cross(a,b)) > 1e-9 || norm(cross(sub(o,p),b)) > 1e-9*size { return Ok(blank.clone()) },
            },
            _ => return Ok(blank.clone()),
        }
    }
    let Some((origin,axis)) = line else { return Ok(blank.clone()) };
    if planes.iter().any(|&n| norm(cross(n,axis)) > 1e-9) { return Ok(blank.clone()) }
    turned_from(blank,origin,axis,tools,&|o,a,seam| turned_to(blank,o,a,seam))
}

/// A body whose premises for a sector do not hold, built whole: `blank` (its seams turned from the
/// cuts already, `facing`) split by every placed sheet of `tools` whose box meets its own, the cells
/// judged by `probe` and the one material cell kept, `placements` holding each distinct sweep's
/// count of placements.
pub fn whole(blank: &Brep,tools: Vec<(Brep,Box3)>,placements: &[usize],probe: Probe,say: &Say) -> Result<Brep,String> {
    split_whole(blank,tools,placements,true,probe,say)
}

/// What the sweeps take from `blank` (`whole`'s removed cell, where there is one): the solid a
/// sweep passes through, its blank a box holding all of it.
pub fn removed(blank: &Brep,tools: Vec<(Brep,Box3)>,placements: &[usize],probe: Probe,say: &Say) -> Result<Brep,String> {
    split_whole(blank,tools,placements,false,probe,say)
}

/// `whole`, keeping the material cell or (not `material`) the removed one.
fn split_whole(blank: &Brep,tools: Vec<(Brep,Box3)>,placements: &[usize],material: bool,probe: Probe,say: &Say) -> Result<Brep,String> {
    let clock = crate::util::clock::Instant::now();
    let tools = reaching(blank,tools);
    let (cells,widest) = split_by(vec![blank.clone()],&tools,1e-3*blank.size())?;
    for c in &cells { c.check(1e-6).map_err(|e| format!("a cell of the split is invalid: {e}"))?; }
    (say.stage)(&format!("split the blank of {:.6} mm³ by {} placed sheets, {} at once at most, into {} cells ({:?})",volume(blank),
        tools.len(),widest,cells.len(),clock.elapsed()));
    (say.mark)(Stage::Split);
    kept(cells,placements,"body",material,probe,say)
}

/// The material cell of a split's `cells` (or, not `material`, the removed one), judged by `probe`
/// and held to the cell contract (`placements` each distinct sweep's count), its tolerances
/// measured; refused where the contract fails or more than one cell of the `what` is the one asked.
fn kept(mut cells: Vec<Brep>,placements: &[usize],what: &str,material: bool,probe: Probe,say: &Say) -> Result<Brep,String> {
    let volumes: Vec<f64> = cells.iter().map(volume).collect();
    let clock = crate::util::clock::Instant::now();
    let sampled = crate::util::par::map(&cells,|c| interior(c,4)).into_iter().collect::<Result<Vec<_>,_>>()?;
    let (kept,removed) = judge(&volumes,sampled,probe)?;
    (say.stage)(&format!("classified {} material and {} removed cells ({:?})",kept.len(),removed.len(),clock.elapsed()));
    let measured = |cells: &[Cell]| cells.iter().map(|c| CellVolume {volume:c.volume,point:c.point}).collect::<Vec<_>>();
    self::cells(&measured(&kept),&measured(&removed),placements)?;
    (say.mark)(Stage::Classify);
    let (asked,said) = if material { (&kept,"material") } else { (&removed,"removed") };
    let [kept] = &asked[..] else { return Err(format!("{} cells of the {what} are {said}: this kernel does not unite them",asked.len())) };
    let mut piece = cells.swap_remove(kept.index);
    piece.measure_tolerances();
    Ok(piece)
}

/// A cell of a blank's partition as the cell contract reads it: its volume (mm³) and a point
/// inside it (mm).
#[derive(Clone,Copy,Debug)]
pub struct CellVolume { pub volume: f64,pub point: V }

/// No cell of a partition may be smaller than this (mm³): a sliver of near-tangent surfaces.
pub const LEAST_CELL: f64 = 1e-3;

/// The cell contract after classification: one removed cell per placement of each sweep, the
/// cells of one sweep congruent (its placements are rigid copies, so a cell that differs was
/// cut by something else: a neighbour, a tangent face), and no cell below a size floor (a
/// sliver the kernel made of near-tangent surfaces). The removed volumes are compared as a
/// set, since a cell does not say which placement cut it; a body whose placements cut unequal
/// amounts from an asymmetric blank would need them told apart first.
///
/// `placements` holds each sweep's count. Several sweeps placed at the same indices (a crown
/// and a relief beside it) cut each other's cells: their removed cells then fall into
/// congruence classes of one cell per index each, however many classes the sheets' crossings
/// make. Sweeps placed different numbers of times are not told apart and are refused.
pub fn cells(kept: &[CellVolume],removed: &[CellVolume],placements: &[usize]) -> Result<(),String> {
    if let Some(cell) = kept.iter().chain(removed).find(|c| c.volume < LEAST_CELL) {
        return Err(format!("the split left a cell of {:.3e} mm³, under the {LEAST_CELL} mm³ floor, near {:?}",
            cell.volume,cell.point.map(|x| (x*1e3).round()/1e3)));
    }
    let [each,..] = placements else { return Err("a split with no sweep".into()) };
    if placements.iter().any(|n| n != each) {
        return Err(format!("the sweeps are placed {placements:?} times: a cell contract needs one count"));
    }
    if placements.len() == 1 && removed.len() != *each {
        return Err(format!("the split removed {} cells for {} placements",removed.len(),each));
    }
    let mut volumes: Vec<f64> = removed.iter().map(|c| c.volume).collect();
    volumes.sort_by(f64::total_cmp);
    let mut classes: Vec<Vec<f64>> = Vec::new();
    for v in volumes {
        match classes.last_mut() {
            Some(class) if v <= class[0]*(1.+1e-4) => class.push(v),
            _ => classes.push(vec![v]),
        }
    }
    if placements.len() == 1 && classes.len() > 1 {
        return Err(format!("the removed cells are not congruent: {:.6} to {:.6} mm³",classes[0][0],
            classes.last().unwrap().last().unwrap()));
    }
    if let Some(class) = classes.iter().find(|c| c.len() != *each) {
        return Err(format!("the split removed {} cells in {} classes for {} placements of {} sweeps: {} of {:.6} mm³",
            removed.len(),classes.len(),each,placements.len(),class.len(),class[0]));
    }
    Ok(())
}
