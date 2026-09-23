//! Phase 1 of docs/generating-sweeps-plan.md: where the configured pair stands
//! against the generating-sweep class (docs/generating-sweeps.md), measured on
//! the solved source by sampling. Nothing here constructs a boundary; these are
//! the numbers the plan's decision gate reads. Sampled, never certified.
//!
//! `SOLVENT_CLASS_OFFSETS=0,6,15` (degrees) chooses the offsets, and
//! `SOLVENT_CLASS_MEMBERS=pinion,gear` the members, and `SOLVENT_CLASS_ROLL` the
//! pinion's roll limit for the negative control.
mod support;
use gcs_core::{program,solid::{SweepContacts,SpatialField,TimedContact}};
use std::{collections::BTreeMap,path::Path};

type V = [f64;3];
fn sub(a: V,b: V) -> V { std::array::from_fn(|k| a[k]-b[k]) }
fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V) -> f64 { dot(a,a).sqrt() }

/// The pair at `offset` degrees, its members also publishing their blanks.
fn read(offset: f64) -> program::Elaborated {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    support::read_configured_with(&source,&base,&mut |name,text| match name {
        "configuration" => text.lines().map(|l| if l.trim_start().starts_with("param offset_angle") {
            format!("param offset_angle = {offset}deg\n") } else { format!("{l}\n") }).collect(),
        // `SOLVENT_CLASS_ROLL` (degrees) widens the pinion's roll: the negative
        // control, a tool left in the blank and revisiting it.
        "matched_pair" => text.replace("roll_limit: 35deg",&format!("roll_limit: {}deg",
            std::env::var("SOLVENT_CLASS_ROLL").unwrap_or("35".into()))).replace("  solid body(design.heel)\n",
            "  solid body(design.heel)\n  construction solid blank(design.heel)\n  design.tip bound blank\n  design.toe cut blank\n  design.back cut blank\n"),
        _ => text,
    })
}

/// One in-blank contact of the fine pass.
#[derive(Clone,Copy)]
struct Hit { patch: usize,source: V,position: V,normal: V,time: f64,branch: usize }

#[derive(Default)]
struct Report {
    boundary_samples: usize,
    root_errors: usize,
    hits: usize,
    /// E1: tool boundary samples inside the blank at each roll limit.
    at_limits: [usize;2],
    /// E2: source points with more than one in-blank contact time.
    multiple: usize,
    multiple_witness: Option<(usize,V,f64,f64)>,
    /// E3 per (patch, branch): positive and negative area factors, least |J| and where.
    factors: BTreeMap<(usize,usize),(usize,usize,f64,V)>,
    /// E4: pairs close in space whose source points are far apart, crossing and parallel.
    crossings: usize,
    parallel: usize,
    crossing_witness: Vec<(V,usize,usize,f64,f64)>,
    spacing: f64,
    largest_gap: f64,
    /// Where the class fails, whether the removal's own field puts the point
    /// strictly inside the swept material (buried), strictly outside, or neither.
    folds: Vec<V>,
    buried: [usize;3],
}

fn roots(c: &SweepContacts,patch: usize,u: f64,v: f64) -> Result<Vec<TimedContact>,String> {
    c.at_source_over(patch,u,v,c.domain(),1e-9)
}

fn measure(e: &program::Elaborated,member: &str) -> Report {
    let removal = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
    let blank_id = e.map.ent_named(&format!("pair.{member}.blank")).unwrap().i();
    let c = SweepContacts::read(&e.sketch,removal,1e-10).unwrap();
    let blank = SpatialField::read(&e.sketch,blank_id,1e-10).unwrap();
    let inside = |p: V| blank.value(p) < -1e-6;
    let tool = c.source_material();
    let on_tool = |p: V| tool.value(p).abs() < 1e-6;
    let domain = c.domain();
    let poses = domain.map(|t| c.motion().at(t).unwrap());
    let mut r = Report::default();
    let mut hits: Vec<(Hit,usize,usize)> = Vec::new();
    let mut gaps: Vec<f64> = Vec::new();
    for patch in 0..c.patches().len() {
        let surface = &c.patches()[patch];
        // Coarse pass: which revolution angles reach the blank at all.
        let (cu,cv) = (40,3600);
        let mut reach = vec![false;cv];
        for i in 0..=cu { for j in 0..cv {
            let (u,v) = (i as f64/cu as f64,j as f64/cv as f64);
            let Ok(s) = surface.at(u,v) else { continue };
            if !on_tool(s.position) { continue; }
            if let Ok(list) = roots(&c,patch,u,v) {
                if list.iter().any(|t| inside(t.contact.position)) { reach[j] = true; }
            }
        }}
        // The shortest circular arc holding every reaching angle, widened a step.
        let reached: Vec<usize> = (0..cv).filter(|&j| reach[j]).collect();
        if reached.is_empty() { continue; }
        let mut gap = (0,0);
        for w in 0..reached.len() {
            let (a,b) = (reached[w],reached[(w+1)%reached.len()]);
            let g = (b+cv-a)%cv; let g = if g == 0 { cv } else { g };
            if g > gap.1 { gap = (w,g); }
        }
        let start = (reached[(gap.0+1)%reached.len()]+cv-2)%cv;
        let width = (cv-gap.1+4).min(cv);
        let (nu,nv) = (300,1200);
        let v_at = |j: usize| ((start as f64+width as f64*j as f64/nv as f64)/cv as f64).rem_euclid(1.);
        let mut prev_row: Vec<Option<V>> = vec![None;nv+1];
        for i in 0..=nu {
            let u = i as f64/nu as f64;
            let mut row: Vec<Option<V>> = vec![None;nv+1];
            for j in 0..=nv {
                let v = v_at(j);
                let Ok(s) = surface.at(u,v) else { continue };
                if !on_tool(s.position) { continue; }
                r.boundary_samples += 1;
                for (k,pose) in poses.iter().enumerate() {
                    if inside(pose.point(s.position)) { r.at_limits[k] += 1; }
                }
                let list = match roots(&c,patch,u,v) { Ok(l) => l, Err(_) => { r.root_errors += 1; continue } };
                let found: Vec<&TimedContact> = list.iter().filter(|t| inside(t.contact.position)).collect();
                if found.len() > 1 {
                    r.multiple += 1;
                    if r.multiple_witness.is_none() {
                        r.multiple_witness = Some((patch,s.position,found[0].root.time,found[1].root.time));
                    }
                }
                for t in found {
                    r.hits += 1;
                    let hit = Hit {patch,source:s.position,position:t.contact.position,normal:t.contact.normal,
                        time:t.root.time,branch:t.root.branch};
                    row[j] = Some(hit.position);
                    for q in [prev_row[j],if j > 0 { row[j-1] } else { None }].into_iter().flatten() {
                        gaps.push(norm(sub(q,hit.position)));
                    }
                    hits.push((hit,i,j));
                    // E3: the generated surface's oriented area factor by central
                    // differences on the same branch, against the source's own.
                    let h = 1e-5;
                    let (ua,ub) = ((u-h).max(0.),(u+h).min(1.));
                    let near = |uu: f64,vv: f64| -> Option<V> {
                        roots(&c,patch,uu,vv.rem_euclid(1.)).ok()?.into_iter()
                            .filter(|x| x.root.branch == t.root.branch)
                            .min_by(|a,b| (a.root.time-t.root.time).abs().total_cmp(&(b.root.time-t.root.time).abs()))
                            .map(|x| x.contact.position)
                    };
                    let (Some(fa),Some(fb),Some(ga),Some(gb)) = (near(ua,v),near(ub,v),near(u,v-h),near(u,v+h)) else { continue };
                    let fu = sub(fb,fa).map(|x| x/(ub-ua)); let fv = sub(gb,ga).map(|x| x/(2.*h));
                    let su = surface.at(ub,v).unwrap().position; let sa = surface.at(ua,v).unwrap().position;
                    let tv = surface.at(u,(v+h).rem_euclid(1.)).unwrap().position; let ta = surface.at(u,(v-h).rem_euclid(1.)).unwrap().position;
                    let area = norm(cross(sub(su,sa).map(|x| x/(ub-ua)),sub(tv,ta).map(|x| x/(2.*h))));
                    if area == 0. { continue; }
                    let j_factor = dot(cross(fu,fv),t.contact.normal)/area;
                    let entry = r.factors.entry((patch,t.root.branch)).or_insert((0,0,f64::INFINITY,[0.;3]));
                    if j_factor > 0. { entry.0 += 1 } else { entry.1 += 1; r.folds.push(t.contact.position); }
                    if j_factor.abs() < entry.2 { entry.2 = j_factor.abs(); entry.3 = t.contact.position; }
                }
            }
            prev_row = row;
        }
        eprintln!("  patch {patch} ({}): {} in-blank contacts so far",surface.name,r.hits);
    }
    // The sample spacing: most neighbouring samples are this close. The largest
    // gap is reported apart, since a jump across a seam is not the spacing.
    gaps.sort_by(f64::total_cmp);
    r.largest_gap = gaps.last().copied().unwrap_or(0.);
    r.spacing = gaps.get(gaps.len()*95/100).copied().unwrap_or(0.);
    // E4: sheet points closer than the sample spacing whose source points are
    // far apart on the tool: the sheet passes through itself there.
    let cell = r.spacing.max(1e-9);
    let mut grid: BTreeMap<[i64;3],Vec<usize>> = BTreeMap::new();
    for (k,(h,_,_)) in hits.iter().enumerate() {
        grid.entry(h.position.map(|x| (x/cell).floor() as i64)).or_default().push(k);
    }
    for (k,(a,_,_)) in hits.iter().enumerate() {
        let key = a.position.map(|x| (x/cell).floor() as i64);
        for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
            let Some(list) = grid.get(&[key[0]+dx,key[1]+dy,key[2]+dz]) else { continue };
            for &m in list {
                if m <= k { continue; }
                let b = &hits[m].0;
                if norm(sub(a.position,b.position)) > 0.5*cell { continue; }
                if norm(sub(a.source,b.source)) < 5.*cell { continue; }
                if dot(a.normal,b.normal).abs() > 0.999 { r.parallel += 1; } else {
                    r.crossings += 1;
                    r.folds.push(a.position);
                    if r.crossing_witness.len() < 5 { r.crossing_witness.push((a.position,a.patch,b.patch,a.time,b.time)); }
                }
            }
        }}}
    }
    // Judge an even sample of the failing points against the removal's field.
    if !r.folds.is_empty() {
        let mut material = gcs_core::solid::MaterialField::read(&e.sketch,removal,1e-10).unwrap().evaluator(4096);
        let step = (r.folds.len()/200).max(1);
        for p in r.folds.iter().step_by(step) {
            let bounds = material.bounds(p.map(|x| gcs_core::interval::Interval::point(x).unwrap()),
                gcs_core::interval::minimum::Options {value_tolerance:0.002,max_evaluations:20000}).unwrap();
            let [lo,hi] = bounds.value.bounds();
            r.buried[if hi < 0. { 0 } else if lo > 0. { 1 } else { 2 }] += 1;
        }
    }
    r
}

#[test]
#[ignore]
fn where_the_pair_stands_against_the_generating_class() {
    let offsets: Vec<f64> = std::env::var("SOLVENT_CLASS_OFFSETS").unwrap_or("0,6,15,30,45".into())
        .split(',').map(|s| s.trim().parse().unwrap()).collect();
    let members: Vec<String> = std::env::var("SOLVENT_CLASS_MEMBERS").unwrap_or("pinion,gear".into())
        .split(',').map(|s| s.trim().to_string()).collect();
    for offset in offsets {
        let started = std::time::Instant::now();
        let e = read(offset);
        eprintln!("offset {offset}deg: read and solved in {:?}",started.elapsed());
        for member in &members {
            let started = std::time::Instant::now();
            let r = measure(&e,member);
            eprintln!("== offset {offset}deg {member} ({:?})",started.elapsed());
            eprintln!("  samples {} on the tool, {} root errors, {} in-blank contacts, spacing {:.4} (largest gap {:.4})",
                r.boundary_samples,r.root_errors,r.hits,r.spacing,r.largest_gap);
            eprintln!("  E1 tool samples inside the blank at the roll limits: {:?}",r.at_limits);
            eprintln!("  E2 source points with several in-blank contacts: {} {:?}",r.multiple,r.multiple_witness);
            for ((patch,branch),(pos,neg,least,at)) in &r.factors {
                eprintln!("  E3 patch {patch} branch {branch}: J>0 {pos}, J<0 {neg}, least |J| {least:.3e} at {at:?}");
            }
            eprintln!("  E4 far-source pairs within half a spacing: {} crossing, {} parallel",r.crossings,r.parallel);
            for w in &r.crossing_witness { eprintln!("    {w:?}"); }
            if !r.folds.is_empty() {
                eprintln!("  failing points against the removal's field: {} buried, {} outside, {} on the boundary",
                    r.buried[0],r.buried[1],r.buried[2]);
            }
        }
    }
}
