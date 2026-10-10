//! A body built as one sector (docs/native-speed-plan.md, by this kernel alone): where every
//! swept cut is a turn of one placement about one axis and the blank reads alike under that turn,
//! the blank between a side through the gaps the cuts leave (`solid::sector`) and that side turned
//! by a pitch, cut by the placements that fall in it, its cells judged by the material field, and
//! the material cell returned to be patterned round the axis (`brep::pattern`).
#[allow(unused_imports)]
use crate::fmath::Det;
use super::Say;
use super::sheet::{Fitted,surface_grid};
use crate::brep::boolean::split;
use crate::brep::geom::Rigid;
use crate::brep::props::volume;
use crate::brep::query::interior;
use crate::brep::topo::Brep;
use crate::envelope::Motion;
use crate::interval::{Interval,minimum::Options};
use crate::json::Json;
use crate::model::Sketch;
use crate::solid::cad::{self,StaticRecipe};
use crate::solid::contracts;
use crate::solid::sector::{self,Boundary,Frame,Slices};
use crate::solid::{MaterialField,ProbeState,SpatialField};
use crate::space::{cross,distance,dot,norm,scale,sub};

type V = [f64;3];

/// The side's grid across its slices, and how finely the built side is read back.
const COLUMNS: usize = 9;
const READ_BACK: usize = 4;
/// The least clearance the side keeps from every cut beyond twice the sheets' fit error (mm).
const MARGIN: f64 = 0.05;
/// The most points of the side the material field is asked about.
const PROBES: usize = 160;
/// The tolerance the split works to (mm).
const SPLIT_TOL: f64 = 1e-6;

/// The sector: its material, and the turn that patterns it (`count` copies a `pitch` apart about
/// the line through `origin`, mm, along `axis`).
pub struct Sector { pub piece: Brep,pub origin: V,pub axis: V,pub count: usize,pub pitch: f64 }

/// The centres of the spheres the recipe's revolutions make where they are on the axis: the
/// slicings a bevel gear's end spheres suggest (mm).
fn sphere_centres(recipe: &Json,frame: Frame,size: f64) -> Vec<V> {
    let on_axis = |p: V| { let d = sub(p,frame.origin); norm(sub(d,scale(frame.axis,dot(d,frame.axis)))) <= 1e-6*size };
    let mut centres: Vec<V> = Vec::new();
    for node in recipe.get("nodes").map(Json::arr).unwrap_or_default() {
        if node.get("kind").map(Json::as_str) != Some("revolve") { continue }
        let (Some(o),Some(a)) = (node.get("origin"),node.get("axis")) else { continue };
        let [o,a]: [V;2] = [o,a].map(|j| std::array::from_fn(|i| j.arr()[i].as_f64()));
        let on_revolution = |p: V| norm(cross(sub(p,o),a)) <= 1e-6*size*norm(a);
        let Some(profile) = node.get("profile") else { continue };
        for edges in profile.get("loops").map(Json::arr).unwrap_or_default() { for edge in edges.arr() {
            if edge.get("kind").map(Json::as_str) != Some("circle") { continue }
            let Some(c) = edge.get("center") else { continue };
            let c: V = std::array::from_fn(|i| c.arr()[i].as_f64());
            if on_revolution(c) && on_axis(c) && !centres.iter().any(|&k| distance(k,c) <= 1e-6*size) { centres.push(c); }
        } }
    }
    centres
}

/// A solid's diagonal (mm).

/// A placement's rigid motion in millimetres.
pub(super) fn placed(pose: Motion,scale: f64) -> Rigid { Rigid::from_rows(&cad::placement_matrix(pose,scale)) }

/// The blank of `recipe` as its meridian region turned once from the half-plane along `seam`, or
/// by its Booleans turned about the axis till its seams lie there.
fn blank_from(recipe: &Json,origin: V,axis: V,seam: V) -> Result<Brep,String> {
    match crate::brep::recipe::meridian(recipe,seam)? {
        Ok((b,o,a)) => {
            if norm(cross(a,axis)) > 1e-9 || norm(cross(sub(o,origin),axis)) > 1e-6 {
                return Err("the blank's meridian is about another line than the indexing's".into())
            }
            Ok(b)
        }
        Err(_) => turned_to(&crate::brep::recipe::build(recipe)?,origin,axis,seam),
    }
}

/// A solid of revolution about the line through `origin` along `axis` turned about it until its
/// faces' seams lie in the half-plane along `seam`; refused where its seams are not in one.
fn turned_to(b: &Brep,origin: V,axis: V,seam: V) -> Result<Brep,String> {
    let mut found: Option<V> = None;
    for f in &b.faces {
        let uses: Vec<u32> = f.loops.iter().flatten().map(|c| c.edge).collect();
        for (k,&e) in uses.iter().enumerate() {
            if !uses[k+1..].contains(&e) { continue }
            let edge = &b.edges[e as usize];
            let crate::brep::topo::EdgeCurve::Curve(_) = &edge.curve else { continue };
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
struct Cell { index: usize,point: V,volume: f64 }

/// Cells of the given volumes judged by the material field at their interior samples (points with
/// a lower bound on their distance from the cell's boundary, deepest first): every sample far
/// enough from its cell's boundary is probed, at half that distance (at most 0.05 mm), and all of a
/// cell's must agree. An unresolved probe or a cell reading both ways refuses. The cells are in
/// millimetres and the field in the document's units, `scale_mm` millimetres each.
fn judge(volumes: &[f64],sampled: Vec<Vec<(V,f64)>>,field: &MaterialField,scale_mm: f64)
    -> Result<(Vec<Cell>,Vec<Cell>),String> {
    let (mut kept,mut removed) = (Vec::new(),Vec::new());
    let asked: Vec<(V,f64)> = sampled.iter().flatten().map(|&(p,b)| (p,(b*0.5).min(0.05))).filter(|&(_,d)| d > 1e-4).collect();
    let answers = crate::par::indices_with(asked.len(),|| field.evaluator(cad::POSE_CACHE),|material,i| {
        let (point,distance) = asked[i];
        material.probe(point.map(|x| Interval::point(x/scale_mm).unwrap()),[1.,0.,0.],distance/scale_mm,
            Options {value_tolerance:distance/scale_mm/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))
    });
    let mut answers = answers.into_iter();
    for (k,(&volume,samples)) in volumes.iter().zip(sampled).enumerate() {
        if samples.is_empty() { return Err(format!("a cell of volume {volume} has no interior sample")) }
        let cell = Cell {index:k,point:samples[0].0,volume};
        let mut verdict: Option<(bool,V)> = None;
        let deepest = samples[0];
        for (point,boundary) in samples {
            if (boundary*0.5).min(0.05) <= 1e-4 { continue }
            let probe = answers.next().expect("an answer a probe asked")?;
            let inside = match probe.state {
                ProbeState::InteriorBall => true,
                ProbeState::ExteriorBall => false,
                state => return Err(format!("the material at {point:?} ({boundary:.3} mm from a cell boundary) is {state:?}")),
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

/// What a sector is cut from once its premises hold: the placements' turn, the two sides, a sheet
/// of each sweep placed in the sector, and the body's material field (in the document's units,
/// `scale_mm` millimetres each).
pub struct Premises { frame: Frame,n: usize,pitch: f64,across: f64,side: Brep,other: Brep,tools: Vec<Brep>,body_field: MaterialField,
    scale_mm: f64 }

/// Whether `body` can be built as one sector, and what it is cut from, or why not — the placements
/// not turns of one about one axis, the blank not alike under the turn, no side clear of the cuts
/// through their gaps, a side the field reads as removed. `sheets` are the distinct sweeps' fitted
/// sheets in the order of `distinct`; `blank` the static blank (mm), `field` its analytic field.
#[allow(clippy::too_many_arguments)]
pub fn premises(sk: &Sketch,body: usize,recipe: &StaticRecipe,blank: &Brep,field: &SpatialField,distinct: &[usize],sheets: &[Fitted],
    scale_mm: f64,say: &Say) -> Result<Premises,String> {
    let started = crate::clock::Instant::now();
    let poses: Vec<Vec<Motion>> = distinct.iter().map(|&s| recipe.sweeps.iter().filter(|c| c.swept == s).map(|c| c.pose).collect()).collect();
    let (lo,hi) = blank.bounds();
    let bounds = [lo,hi];
    let size = distance(lo,hi);
    let indexing = sector::indexing(&poses,size/scale_mm)?;
    let n = indexing.count;
    let frame = Frame::new(indexing.origin.map(|x| x*scale_mm),indexing.axis);
    let mm = |m: Motion,p: V| m.point(p.map(|x| x/scale_mm)).map(|x| x*scale_mm);
    let inside = |p: V| field.value(p.map(|x| x/scale_mm)) < 0.;
    // the blank reads alike under one pitch's turn, at points spread over its box
    let turn = indexing.turn(1);
    let mut rng = crate::rng::Rng::new(0x5ec7);
    for _ in 0..256 {
        let p: V = std::array::from_fn(|k| rng.uniform(bounds[0][k],bounds[1][k]));
        let (a,b) = (field.value(p.map(|x| x/scale_mm)),field.value(mm(turn,p).map(|x| x/scale_mm)));
        if (a-b).abs()*scale_mm > 1e-9*size { return Err(format!("the blank does not read alike turned by one pitch (at {p:?}, {a} against {b})")) }
    }
    // each sweep's contacts in the blank at its first placement
    let cuts: Vec<Vec<V>> = sheets.iter().enumerate()
        .map(|(k,s)| s.sheet.points.iter().chain(&s.sheet.withheld).map(|&p| mm(poses[k][0],p)).filter(|&p| inside(p)).collect()).collect();
    let all: Vec<V> = cuts.iter().flatten().copied().collect();
    let mut candidates: Vec<Slices> = sphere_centres(&recipe.recipe,frame,size).into_iter().map(Slices::Spheres).collect();
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
    for (k,points) in cuts.iter().enumerate() {
        let sectors: std::collections::BTreeSet<usize> = points.iter().map(|&p| chosen.sector(p)).collect();
        let [first] = sectors.iter().copied().collect::<Vec<_>>()[..] else {
            return Err(format!("the cut of `{}` lies in {} sectors",sk.solids[distinct[k]].name,sectors.len()))
        };
        let index = (n-first) % n;
        let j = indexing.indices[k].iter().position(|&i| i == index).ok_or("no placement in the sector")?;
        tools.push(sheets[k].face.moved(&placed(poses[k][j],scale_mm)));
    }
    // the side, built and read back against the gaps and the material field
    let (s,w) = sector::section_span(frame,chosen.slices,bounds,&inside)?;
    let (grid,rows) = chosen.grid(s,w,COLUMNS);
    let side = crate::brep::build::sheet(crate::brep::nurbs::interpolate_net(&grid,rows,COLUMNS,crate::brep::nurbs::Parametrization::ChordLength)
        .ok_or("the side's grid could not be interpolated")?)?;
    let read: Vec<V> = surface_grid(&side,READ_BACK*rows,READ_BACK*COLUMNS).into_iter().map(|(p,_)| p).filter(|&p| inside(p)).collect();
    let clear = chosen.clearance_of(&read)?;
    if clear < margin { return Err(format!("the side as built passes {clear:.4} mm from a cut, under {margin:.4} mm")) }
    let body_field = MaterialField::read(sk,body,cad::AXIS_TOLERANCE)?;
    let radius = margin/2.;
    let deep: Vec<V> = read.iter().copied().filter(|p| field.value(p.map(|x| x/scale_mm))*scale_mm < -radius).collect();
    let step = deep.len().div_ceil(PROBES).max(1);
    let probed_at: Vec<V> = deep.iter().copied().step_by(step).collect();
    let probes = crate::par::indices_with(probed_at.len(),|| body_field.evaluator(cad::POSE_CACHE),|material,i| {
        material.probe(probed_at[i].map(|x| Interval::point(x/scale_mm).unwrap()),[1.,0.,0.],radius/scale_mm,
            Options {value_tolerance:radius/scale_mm/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))
    });
    for (&p,probe) in probed_at.iter().zip(probes) {
        if probe?.state != ProbeState::InteriorBall {
            return Err(format!("the material field reads the side at {:?} as not material",p.map(|x| (x*1e3).round()/1e3)))
        }
    }
    (say.stage)(&format!("the side: {rows}x{COLUMNS} nodes, read back {clear:.4} mm clear of the cuts at {} points in the blank, \
        {} of them material by the field ({:?})",read.len(),probed_at.len(),started.elapsed()));
    let other = side.moved(&placed(turn,scale_mm));
    // the blank made again about the axis, its faces' parameters starting opposite the sector
    let across = chosen.angle_at((chosen.span[0]+chosen.span[1])/2.)+indexing.pitch()/2.+std::f64::consts::PI;
    Ok(Premises {frame,n,pitch:indexing.pitch(),across,side,other,tools,body_field,scale_mm})
}

/// The sector its premises describe: the blank made about the axis, split by the two sides into the
/// sector and it by the sheets, its cells judged by the material field and the one material cell kept.
pub fn construct(recipe: &StaticRecipe,distinct: &[usize],premises: Premises,say: &Say) -> Result<Sector,String> {
    let Premises {frame,n,pitch,across,side,other,tools,body_field,scale_mm} = premises;
    let clock = crate::clock::Instant::now();
    let turned = blank_from(&recipe.recipe,frame.origin,frame.axis,frame.radial(across))?;
    let whole = volume(&turned);
    let halves = split(&turned,&side.joined(&other),SPLIT_TOL)?;
    let volumes: Vec<f64> = halves.iter().map(volume).collect();
    let share = whole/n as f64;
    let Some(wedge) = halves.iter().zip(&volumes).find(|(_,v)| (*v-share).abs() <= 1e-5*whole).map(|(c,_)| c.clone()) else {
        return Err(format!("the sides split the blank ({whole:.6} mm³) into {} cells of {volumes:.6?} mm³, none its {share:.6} mm³ share",
            halves.len()))
    };
    if halves.len() != 2 { return Err(format!("the sides split the blank into {} cells, not two",halves.len())) }
    let mut cells = vec![wedge];
    for sheet in &tools {
        let parts = crate::par::map(&cells,|c| split(c,sheet,SPLIT_TOL));
        cells = parts.into_iter().collect::<Result<Vec<_>,_>>()?.into_iter().flatten().collect();
    }
    for c in &cells { c.check(1e-6).map_err(|e| format!("a cell of the split is invalid: {e}"))?; }
    let volumes: Vec<f64> = cells.iter().map(volume).collect();
    (say.stage)(&format!("split the blank by the sides into the sector of {share:.6} mm³, and it by {} sheets into {} cells ({:?})",
        tools.len(),cells.len(),clock.elapsed()));
    (say.mark)(crate::solid::export::Stage::Split);
    let clock = crate::clock::Instant::now();
    let sampled = crate::par::map(&cells,|c| interior(c,4)).into_iter().collect::<Result<Vec<_>,_>>()?;
    let (kept,removed) = judge(&volumes,sampled,&body_field,scale_mm)?;
    (say.stage)(&format!("classified {} material and {} removed cells ({:?})",kept.len(),removed.len(),clock.elapsed()));
    let measured = |cells: &[Cell]| cells.iter().map(|c| contracts::CellVolume {volume:c.volume,point:c.point}).collect::<Vec<_>>();
    contracts::cells(&measured(&kept),&measured(&removed),&vec![1;distinct.len()])?;
    (say.mark)(crate::solid::export::Stage::Classify);
    let [kept] = &kept[..] else { return Err(format!("{} cells of the sector are material: this kernel does not unite them",kept.len())) };
    let mut piece = cells.swap_remove(kept.index);
    crate::brep::json::measure(&mut piece);
    Ok(Sector {piece,origin:frame.origin,axis:frame.axis,count:n,pitch})
}

/// A body whose premises for a sector do not hold, built whole: the blank split by every placement of
/// every sheet that reaches it, the cells judged by the material field and the one material cell
/// kept. `sheets` and `distinct` as for `premises`.
#[allow(clippy::too_many_arguments)]
pub fn whole(sk: &Sketch,body: usize,recipe: &StaticRecipe,blank: &Brep,distinct: &[usize],sheets: &[Fitted],scale_mm: f64,say: &Say)
    -> Result<Brep,String> {
    let clock = crate::clock::Instant::now();
    let (lo,hi) = blank.bounds();
    let placements: Vec<usize> = distinct.iter().map(|&s| recipe.sweeps.iter().filter(|c| c.swept == s).count()).collect();
    let mut tools = Vec::new();
    for (k,&s) in distinct.iter().enumerate() {
        for cut in recipe.sweeps.iter().filter(|c| c.swept == s) {
            let tool = sheets[k].face.moved(&placed(cut.pose,scale_mm));
            let (a,b) = tool.bounds();
            if (0..3).all(|i| a[i] <= hi[i] && b[i] >= lo[i]) { tools.push(tool); }
        }
    }
    // a solid of revolution made again with its seams opposite the cuts, as a sector's blank is: a
    // sheet across a face's seam is a split this kernel does not finish
    let blank = match crate::brep::recipe::meridian(&recipe.recipe,[1.,0.,0.])? {
        Ok((_,origin,axis)) => {
            let centres: Vec<V> = tools.iter().map(|t| { let (a,b) = t.bounds(); std::array::from_fn(|k| (a[k]+b[k])/2.) }).collect();
            let c: V = std::array::from_fn(|k| centres.iter().map(|p| p[k]).sum::<f64>()/centres.len().max(1) as f64);
            let a = scale(axis,1./norm(axis));
            let r = sub(sub(c,origin),scale(a,dot(sub(c,origin),a)));
            if norm(r) > 1e-9*blank.size() { blank_from(&recipe.recipe,origin,a,scale(r,-1./norm(r)))? } else { blank.clone() }
        }
        Err(_) => blank.clone(),
    };
    let mut cells = vec![blank.clone()];
    for sheet in &tools {
        let parts = crate::par::map(&cells,|c| split(c,sheet,SPLIT_TOL));
        cells = parts.into_iter().collect::<Result<Vec<_>,_>>()?.into_iter().flatten().collect();
    }
    for c in &cells { c.check(1e-6).map_err(|e| format!("a cell of the split is invalid: {e}"))?; }
    let volumes: Vec<f64> = cells.iter().map(volume).collect();
    (say.stage)(&format!("split the blank of {:.6} mm³ by {} placed sheets into {} cells ({:?})",volume(&blank),tools.len(),cells.len(),
        clock.elapsed()));
    (say.mark)(crate::solid::export::Stage::Split);
    let clock = crate::clock::Instant::now();
    let body_field = MaterialField::read(sk,body,cad::AXIS_TOLERANCE)?;
    let sampled = crate::par::map(&cells,|c| interior(c,4)).into_iter().collect::<Result<Vec<_>,_>>()?;
    let (kept,removed) = judge(&volumes,sampled,&body_field,scale_mm)?;
    (say.stage)(&format!("classified {} material and {} removed cells ({:?})",kept.len(),removed.len(),clock.elapsed()));
    let measured = |cells: &[Cell]| cells.iter().map(|c| contracts::CellVolume {volume:c.volume,point:c.point}).collect::<Vec<_>>();
    contracts::cells(&measured(&kept),&measured(&removed),&placements)?;
    (say.mark)(crate::solid::export::Stage::Classify);
    let [kept] = &kept[..] else { return Err(format!("{} cells of the body are material: this kernel does not unite them",kept.len())) };
    let mut solid = cells.swap_remove(kept.index);
    crate::brep::json::measure(&mut solid);
    Ok(solid)
}
