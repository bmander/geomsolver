//! Milestone 1: the field judges every vertex of the traced seed sheets.
//! What this establishes: on an outer branch the tracer's vertices are kept
//! by a bracket two queries wide; inner branches are labelled, not trusted;
//! and the cost of a near-boundary query is measured, since the whole
//! construction is priced in those.
use super::{harness::{self,distance,V3},motions,tools};
use gcs_core::solid::{MaterialField,SpatialField,SweepPatch,swept_boundary::{FieldJudge,Label,Labelled,label_sheets,seeds}};
use gcs_core::{model::SolidDef,motion::Family,program::Elaborated};

/// The tool's own field sampled over the roll at a world point, in plain
/// floats: the least value and the parameter attaining it. A diagnostic of
/// what the sweep does at a point the judge could not resolve.
pub(super) struct Sampler { pub(super) field: SpatialField, pub(super) family: Family, pub(super) domain: [f64;2] }
impl Sampler {
    pub(super) fn new(e: &Elaborated,swept: usize) -> Self {
        let SolidDef::Swept {source,motion,from,to} = &e.sketch.solids[swept].def else { panic!("not a sweep") };
        Self {field:SpatialField::read(&e.sketch,*source as usize,1e-10).unwrap(),family:Family::read(&e.sketch,*motion as usize).unwrap(),domain:[from.value,to.value]}
    }
    pub(super) fn least(&self,p: V3) -> (f64,f64) {
        (0..=400).map(|k| { let t = self.domain[0]+(self.domain[1]-self.domain[0])*k as f64/400.;
            (self.field.value(self.family.at(t).unwrap().inverse().point(p)),t) }).fold((f64::INFINITY,0.),|m,x| if x.0 < m.0 { x } else { m })
    }
}

const SPACING: f64 = 0.5;
const SAGITTA: f64 = 0.02;
const EPSILON: f64 = SAGITTA/4.;
const REACH: f64 = SAGITTA;

fn judge_and_label(source: &str,name: &str) -> (Vec<SweepPatch>,Vec<Labelled>,FieldJudge) {
    let e = harness::read(source);
    let swept = harness::solid(&e,name);
    let (_,sheets,_) = seeds(&e.sketch,swept,SPACING,SAGITTA,&|_| {}).unwrap();
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,EPSILON/2.,4000,1000,4096);
    let started = std::time::Instant::now();
    let labelled = label_sheets(&mut judge,&sheets,EPSILON,REACH).unwrap();
    report(name,&sheets,&labelled,&judge,started.elapsed(),&Sampler::new(&e,swept));
    (sheets,labelled,judge)
}

fn report(name: &str,sheets: &[SweepPatch],labelled: &[Labelled],judge: &FieldJudge,elapsed: std::time::Duration,sampler: &Sampler) {
    let counts = |l: Label| labelled.iter().map(|x| x.count(l)).sum::<usize>();
    eprintln!("{name}: {} sheets, {} vertices: {} kept, {} moved, {} inner, {} positive, {} unresolved in {:?}; {}",sheets.len(),
        sheets.iter().map(|s| s.points.len()).sum::<usize>(),counts(Label::Kept),counts(Label::Moved),counts(Label::Inner),
        counts(Label::Positive),counts(Label::Unresolved),elapsed,judge.stats.report());
    let along = |p: V3,m: V3,r: f64| -> V3 { std::array::from_fn(|k| p[k]+r*m[k]) };
    for (k,(sheet,l)) in sheets.iter().zip(labelled).enumerate() {
        for (i,r,enclosure) in l.unresolved.iter().take(3) {
            let (p,m) = (sheet.points[*i],l.directions[*i]);
            let (at,t_at) = sampler.least(p); let (off,t_off) = sampler.least(along(p,m,*r));
            eprintln!("  sheet {k} vertex {i} at {:?} (column {} of {}, t {:.4}) unresolved at {r:+.4} along {:?}: {enclosure:?}; sampled least {at:+.5} at t {t_at:.4} there, {off:+.5} at t {t_off:.4} offset",
                p.map(|x| (x*1e4).round()/1e4),sheet.column[*i],sheet.times.len()-1,sheet.times[sheet.column[*i] as usize],m.map(|x| (x*1e3).round()/1e3));
        }
        for (i,label) in l.labels.iter().enumerate().filter(|(_,l)| **l == Label::Positive).take(3) {
            let (p,m) = (sheet.points[i],l.directions[i]);
            let (at,t_at) = sampler.least(p); let (inward,t_in) = sampler.least(along(p,m,-REACH));
            eprintln!("  sheet {k} vertex {i} at {:?} (column {} of {}, t {:.4}) {label:?} along {:?}: sampled least {at:+.5} at t {t_at:.4} there, {inward:+.5} at t {t_in:.4} at -reach",
                p.map(|x| (x*1e4).round()/1e4),sheet.column[i],sheet.times.len()-1,sheet.times[sheet.column[i] as usize],m.map(|x| (x*1e3).round()/1e3));
        }
    }
}

#[test]
fn a_turned_spheres_retained_vertices_have_spatial_witnesses_on_the_torus() {
    let source = format!("{}{}{}",tools::SPHERE,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.));
    let (sheets,labelled,judge) = judge_and_label(&source,"swept");
    let mut worst: f64 = 0.;
    for l in &labelled {
        for (i,p) in l.points.iter().enumerate() {
            assert!(matches!(l.labels[i],Label::Kept | Label::Moved),"{i}: {:?}",l.labels[i]);
            let bracket = l.brackets[i].expect("retained label needs actual witnesses");
            assert!(bracket.inside().1[1] < 0. && bracket.outside().1[0] > 0.);
            assert!(bracket.radius_from(*p).unwrap() <= l.radius[i]);
            // the tube of radius 1 about the circle of radius 3 in z = 0
            let ring = (p[0]*p[0]+p[1]*p[1]).sqrt()-3.;
            worst = worst.max(((ring*ring+p[2]*p[2]).sqrt()-1.).abs());
        }
    }
    eprintln!("farthest kept vertex from the torus: {worst:.2e}");
    assert!(worst <= EPSILON,"a kept vertex is {worst:.2e} from the torus");
    assert!(judge.stats.near >= 2*sheets.iter().map(|s| s.points.len()).sum::<usize>());
}

/// The edges where a sheet leaves the boundary, per sheet: their midpoints
/// and their vertex pairs.
fn transitions(sheets: &[SweepPatch],labelled: &[Labelled]) -> Vec<Vec<(V3,(u32,u32))>> {
    sheets.iter().zip(labelled).map(|(s,l)| l.transitions(s).into_iter()
        .map(|(a,b)| { let (p,q) = (s.points[a as usize],s.points[b as usize]); (std::array::from_fn(|k| 0.5*(p[k]+q[k])),(a,b)) }).collect()).collect()
}

/// How far each transition is from the nearest transition that is not its
/// own neighbour on the rim: the crossing of two branches (of two sheets, or
/// of one sheet with itself across time) leaves a transition on both.
fn partner_distances(transitions: &[Vec<(V3,(u32,u32))>]) -> Vec<f64> {
    let mut out = Vec::new();
    for (i,mine) in transitions.iter().enumerate() {
        for (p,(a,b)) in mine {
            let nearest = transitions.iter().enumerate().flat_map(|(j,t)| t.iter().map(move |x| (j,x)))
                .filter(|(j,(_,(c,d)))| *j != i || (c != a && c != b && d != a && d != b))
                .map(|(_,(q,_))| distance(*p,*q)).fold(f64::INFINITY,f64::min);
            out.push(nearest);
        }
    }
    out
}

fn report_partners(name: &str,sheets: &[SweepPatch],labelled: &[Labelled]) -> Vec<f64> {
    let t = transitions(sheets,labelled);
    let d = partner_distances(&t);
    let mut sorted = d.clone(); sorted.sort_by(f64::total_cmp);
    let at = |f: f64| sorted.get(((sorted.len() as f64-1.)*f) as usize).copied().unwrap_or(f64::NAN);
    eprintln!("{name}: {} transitions on {} sheets; partner distance median {:.3}, 90% {:.3}, max {:.3}",
        d.len(),sheets.len(),at(0.5),at(0.9),at(1.));
    d
}

#[test]
fn a_tumbling_cylinders_unresolved_labels_do_not_claim_boundary_positions() {
    let source = format!("{}{}{}",tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-30.,30.));
    let (sheets,labelled,_) = judge_and_label(&source,"swept");
    for l in &labelled {

        for (i,label) in l.labels.iter().enumerate() {
            if *label == Label::Unresolved {
                assert!(!l.radius[i].is_finite());
                assert!(l.unresolved.iter().any(|(v,_,_)| *v == i));
            }
            if matches!(label,Label::Positive | Label::Inner) { assert!(!l.radius[i].is_finite()); }
        }
    }
    assert!(labelled.iter().any(|l| !l.unresolved.is_empty()));
    assert!(labelled.iter().any(|l| l.count(Label::Kept)+l.count(Label::Moved) > 0));
    report_partners("tumbling cylinder",&sheets,&labelled);
}

#[test]
fn a_turning_prisms_unresolved_labels_do_not_claim_boundary_positions() {
    let source = format!("{}{}{}",tools::TRIANGLE_PRISM,motions::TURN_OFFSET,motions::swept("turn",-50.,50.));
    let (sheets,labelled,_) = judge_and_label(&source,"swept");
    // Strict witnesses expose uncertainty inside columns too. It must not be
    // converted into a boundary position or an asserted paired trim curve.
    for l in &labelled {

        for (i,label) in l.labels.iter().enumerate() {
            if *label == Label::Unresolved {
                assert!(!l.radius[i].is_finite());
                assert!(l.unresolved.iter().any(|(v,_,_)| *v == i));
            }
            if matches!(label,Label::Positive | Label::Inner) { assert!(!l.radius[i].is_finite()); }
        }
    }
    assert!(labelled.iter().any(|l| !l.unresolved.is_empty()));
    assert!(labelled.iter().any(|l| l.count(Label::Kept)+l.count(Label::Moved) > 0));
    report_partners("turning prism",&sheets,&labelled);
}

/// The pinion cutter's document, one space, and its swept solid.
fn pinion_cutter() -> (Elaborated,usize) {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    // The 6-degree hypoid with the bevel pair's pressure angles and spiral, the
    // design these diagnostics were taken at.
    let mut one = |name: &str,text: String| match name {
        "matched_pair" => text.replace("repeat teeth as i {","repeat 1 as i {"),
        "configuration" => text.lines().filter(|l| !["param offset_angle","param pressure_shift","param spiral_angle"]
            .iter().any(|s| l.starts_with(s))).map(|l| format!("{l}\n")).collect::<String>()
            + "param offset_angle = 6deg\nparam pressure_shift = 0deg\nparam spiral_angle = 35deg\n",
        _ => text,
    };
    let mut resolver = harness::directory_resolver(&base,&mut one);
    let e = harness::read_resolving(&source,&mut resolver);
    let swept = harness::solid(&e,"pair.pinion.removal");
    (e,swept)
}

/// Contact points of the cutter at `steps` parameters across its roll, posed
/// to the world, with the contact normals: the sweep's candidate boundary
/// sampled without tracing the whole sweep, which without a crop is minutes.
fn cutter_contacts(e: &Elaborated,swept: usize,steps: usize,per_step: usize) -> (Vec<V3>,Vec<V3>) {
    let sweep = gcs_core::solid::SweepContacts::read(&e.sketch,swept,1e-10).unwrap();
    let [from,to] = sweep.domain();
    let sampler = Sampler::new(e,swept);
    let (mut points,mut normals) = (Vec::new(),Vec::new());
    for k in 0..steps {
        let t = from+(to-from)*(k as f64+0.5)/steps as f64;
        let pose = sampler.family.at(t).unwrap();
        let pieces = sweep.pieces_at(t,1e-9).unwrap();
        let total: usize = pieces.iter().map(|(_,c)| c.points.len()).sum();
        let stride = (total/per_step).max(1);
        for (_,c) in &pieces { for (p,n) in c.points.iter().zip(&c.normals).step_by(stride) { points.push(pose.point(*p)); normals.push(pose.vector(*n)); } }
    }
    (points,normals)
}

/// The spiral bevel pinion cutter, one space: the cost of a near query on a
/// deep Boolean tool under a relative rotation, and the labels a sample of
/// its contact points yields.
#[test]
#[ignore]
fn the_pinion_cutters_sheets_are_judged() {
    let (e,swept) = pinion_cutter();
    let sample: usize = std::env::var("SOLVENT_SAMPLE").ok().and_then(|v| v.parse().ok()).unwrap_or(120);
    let (points,normals) = cutter_contacts(&e,swept,6,sample/6);
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,EPSILON/2.,4000,1000,4096);
    let patch = SweepPatch {points:points.clone(),normals:normals.clone(),triangles:Vec::new(),column:vec![0;points.len()],times:vec![0.],closed:false,provenance:None};
    let started = std::time::Instant::now();
    let l = gcs_core::solid::swept_boundary::label_patch(&mut judge,&patch,&normals,EPSILON,REACH).unwrap();
    report(&format!("pinion cutter ({} contact points)",points.len()),&[patch],&[l],&judge,started.elapsed(),&Sampler::new(&e,swept));
}

/// What one near query on the pinion cutter is made of: the speed bound at a
/// contact point, one interval pose, one tool-field box evaluation, and a
/// query capped at a few hundred roll evaluations.
#[test]
#[ignore]
fn the_pinion_cutters_query_cost() {
    use gcs_core::interval::{Interval as I,minimum::Options};
    let (e,swept) = pinion_cutter();
    let started = std::time::Instant::now();
    let (points,_) = cutter_contacts(&e,swept,1,1);
    let p = points[points.len()/2];
    eprintln!("contact points in {:?}",started.elapsed());
    let sampler = Sampler::new(&e,swept);
    let domain = I::new(sampler.domain[0],sampler.domain[1]).unwrap();
    let point = p.map(|x| I::point(x).unwrap());
    let started = std::time::Instant::now();
    let speed = sampler.family.inverse_point_speed_bound_over(point,domain).unwrap();
    eprintln!("point {p:?}: speed bound {speed:.3} mm/rad ({:?})",started.elapsed());
    let started = std::time::Instant::now();
    for k in 0..100 { sampler.family.bounds(I::point(domain.bounds()[0]+k as f64*1e-3).unwrap()).unwrap(); }
    eprintln!("100 interval poses at a point angle: {:?}",started.elapsed());
    let pose = sampler.family.bounds(I::point(0.5*(domain.bounds()[0]+domain.bounds()[1])).unwrap()).unwrap();
    let local = pose.inverse_point(point).unwrap();
    let started = std::time::Instant::now();
    for _ in 0..100 { sampler.field.bounds(local).unwrap(); }
    eprintln!("100 tool-field box evaluations: {:?}",started.elapsed());
    let mut material = MaterialField::read(&e.sketch,swept,1e-10).unwrap().evaluator(4096);
    for budget in [64usize,256,1024,4000] {
        let started = std::time::Instant::now();
        let b = material.bounds_outside(point,I::point(0.).unwrap(),Options {value_tolerance:EPSILON/2.,max_evaluations:budget}).unwrap();
        let q = &b.sweeps[0].minimum;
        eprintln!("near query at the point, budget {budget}: {:?}, {} evaluations, {:?}, enclosure {:?}",started.elapsed(),q.evaluations,q.status,b.value.bounds());
    }
}
