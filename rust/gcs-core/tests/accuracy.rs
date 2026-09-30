//! The accuracy meter (`solid::accuracy`): an offset of the exact surface reads as that offset on
//! both routes, on the configured pinion's generated flanks (exact envelope contacts from the
//! cutter's own contact chart, which the meter's projection does not read) and on its blank.
use gcs_core::solid::{accuracy::{Meter,Options},SweepContacts,cad};
use gcs_core::space::{add,scale};

/// Offsets the checks read, millimetres.
const OFFSETS: [f64;3] = [0.,0.02,-0.02];

fn pinion() -> (gcs_core::program::Elaborated,usize,Meter) {
    let e = fixtures::gear::read_as_configured();
    let body = fixtures::solid(&e,"pair.pinion.body");
    let started = std::time::Instant::now();
    let meter = Meter::read(&e.sketch,body,Options::in_units(1.)).unwrap();
    eprintln!("meter read in {:?}: {} exact faces",started.elapsed(),meter.surfaces().len());
    (e,body,meter)
}

/// The side of a boundary point a direction points to, by the field: +1 out of the material.
fn outward(meter: &Meter,p: [f64;3],n: [f64;3]) -> [f64;3] {
    if meter.field(add(p,scale(n,1e-3))) > meter.field(add(p,scale(n,-1e-3))) { n } else { scale(n,-1.) }
}

fn reads_offsets(meter: &Meter,points: &[([f64;3],[f64;3])],generated: bool) {
    let mut worst = [0_f64;2];
    for &(p,n) in points {
        let n = outward(meter,p,n);
        for offset in OFFSETS {
            let m = meter.measure(add(p,scale(n,offset)));
            let a = m.analytic.unwrap_or_else(|| panic!("no exact face within reach of {p:?} offset {offset}"));
            assert_eq!(meter.surfaces()[a.surface].generated,generated,"{p:?} offset {offset} read on {}",
                meter.surfaces()[a.surface].name);
            worst[0] = worst[0].max((a.distance-offset).abs());
            worst[1] = worst[1].max((m.field-offset).abs());
        }
    }
    eprintln!("{} {} points: analytic off by at most {:.3e} mm, field by {:.3e} mm",points.len(),
        if generated { "generated" } else { "blank" },worst[0],worst[1]);
    assert!(worst[0] < 1e-6 && worst[1] < 1e-6,"offsets misread: {worst:?}");
}

#[test]
fn a_flank_offset_reads_as_its_offset() {
    let (e,body,meter) = pinion();
    let sk = &e.sketch;
    let recipe = cad::recipe_static(sk,body).unwrap();
    let cut = &recipe.sweeps[5];
    let contacts = SweepContacts::read(sk,cut.swept,cad::AXIS_TOLERANCE).unwrap();
    let mut points = Vec::new();
    for (k,patch) in contacts.patches().iter().enumerate() {
        let [v0,v1] = patch.domain()[1];
        for i in 1..6 { for j in 0..96 {
            let (u,v) = (i as f64/6.,v0+(v1-v0)*j as f64/96.);
            for r in contacts.at_source_over(k,u,v,contacts.domain(),1e-9).unwrap_or_default() {
                let p = cut.pose.point(r.contact.position);
                // on the body's boundary (not cut away by another index or roll), and a tenth of
                // a millimetre inside the blank, clear of its faces' edges
                if meter.field(p).abs() < 1e-7 && meter.blank(p) < -0.1 { points.push((p,cut.pose.vector(r.contact.normal))); }
            }
        } }
    }
    assert!(points.len() >= 20,"only {} exact flank points on the body",points.len());
    let step = (points.len()/40).max(1);
    let points: Vec<_> = points.into_iter().step_by(step).collect();
    let started = std::time::Instant::now();
    reads_offsets(&meter,&points,true);
    eprintln!("{} readings in {:?}",3*points.len(),started.elapsed());
}

#[test]
fn a_blank_offset_reads_as_its_offset() {
    let (e,body,meter) = pinion();
    let field = gcs_core::solid::MaterialField::read(&e.sketch,body,cad::AXIS_TOLERANCE).unwrap();
    // points about the blank, a fixed pseudo-random walk, each carried to the boundary down the
    // field; each nearest blank face's foot the field puts on the boundary is a point of the
    // exact blank surface
    let mut state = 0x9e3779b97f4a7c15_u64;
    let mut next = || { state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (state >> 11) as f64/(1u64 << 53) as f64 };
    let mut points = Vec::new();
    let mut tries = 0;
    while points.len() < 30 && tries < 2000 {
        tries += 1;
        let mut p: [f64;3] = std::array::from_fn(|_| -60.+120.*next());
        for _ in 0..6 {
            let r = field.reading(p);
            if !r.value.is_finite() || r.value.abs() > 200. { break; }
            p = add(p,scale(r.gradient,-r.value));
        }
        let Some(a) = meter.nearest(p) else { continue };
        if meter.surfaces()[a.surface].generated || meter.field(a.foot).abs() > 1e-9 { continue; }
        // clear of the face's edges: a tenth of a millimetre either side is still on it
        let clear = [-0.1,0.1].iter().all(|&d| meter.nearest(add(a.foot,scale(a.normal,d)))
            .is_some_and(|b| b.surface == a.surface && (b.distance-d).abs() < 1e-6));
        if !clear { continue; }
        points.push((a.foot,a.normal));
    }
    assert!(points.len() >= 10,"only {} blank points in {tries} tries",points.len());
    reads_offsets(&meter,&points,false);
}


/// A body with no swept cut is read through its exact B-rep: an offset of the cam's spline side
/// (`solid_spline`) reads as that offset on the analytic route (the B-rep's extrusion) and on the
/// field's (the spline as chords).
#[test]
fn a_static_spline_face_offset_reads_as_its_offset() {
    let (_,e) = fixtures::examples().into_iter().find(|(n,_)| n == "solid_spline.sv").unwrap();
    let mut sk = e.unwrap().sketch;
    gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
    let body = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "cam").unwrap();
    let meter = Meter::read(&sk,body,Options::in_units(1.)).unwrap();
    let b = gcs_core::brep::recipe::build(&cad::recipe(&sk,body).unwrap()).unwrap();
    let face = b.faces.iter().find(|f| f.surface.kind() == "extrusion").unwrap();
    // the side's parameters: along the spline, and across the depth its loop spans
    let vs: Vec<f64> = face.loops[0].iter().flat_map(|c| b.uv_ends(face,c)).map(|uv| uv[1]).collect();
    let (v0,v1) = (vs.iter().copied().fold(f64::INFINITY,f64::min),vs.iter().copied().fold(f64::NEG_INFINITY,f64::max));
    let mut worst = [0_f64;2];
    for i in 1..20 { for j in 1..6 {
        let (u,v) = (i as f64/20.,v0+(v1-v0)*j as f64/6.);
        let p = face.surface.point([u,v]);
        let n = face.surface.normal([u,v]).unwrap();
        let n = if face.reversed { scale(n,-1.) } else { n };
        for offset in OFFSETS {
            let m = meter.measure(add(p,scale(n,offset)));
            let a = m.analytic.unwrap();
            assert_eq!(meter.surfaces()[a.surface].name.split(' ').next(),Some("extrusion"));
            worst[0] = worst[0].max((a.distance-offset).abs());
            worst[1] = worst[1].max((m.field-offset).abs());
        }
    } }
    // the field reads the spline's chords, within `CHORD_SLACK` of the profile's reach of it
    let slack = gcs_core::solid::CHORD_SLACK*(1.+36.);
    assert!(worst[0] < 1e-9 && worst[1] <= slack,"offsets misread: {worst:?} (slack {slack:e})");
}
