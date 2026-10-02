//! Inspect the same native solids constructed by solventc, without exporting or
//! reconstructing their topology through a second host.
#[allow(dead_code)]
#[path="../src/cad/native.rs"]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
use gcs_core::{interval::Interval,solid::{cad,SpatialField}};
use std::{ffi::{c_void,c_int,c_char,CStr},path::Path};

extern "C" {
    fn solvent_cad_error(cad: *mut c_void) -> *const c_char;
    fn solvent_cad_boundary(cad: *mut c_void,source: c_int,rows: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_boundary_point(cad: *mut c_void,row: *const c_int,t: f64,output: *mut f64) -> c_int;
}
fn result(cad: &native::Session,value: c_int) -> Result<c_int,String> {
    if value < 0 { Err(unsafe { CStr::from_ptr(solvent_cad_error(cad.as_ptr())) }.to_string_lossy().into_owned()) }
    else { Ok(value) }
}
fn boundary(cad: &native::Session,solid: c_int) -> Vec<[c_int;4]> {
    let count = result(cad,unsafe { solvent_cad_boundary(cad.as_ptr(),solid,std::ptr::null_mut(),0) }).unwrap();
    assert!(count > 0);
    let mut rows = vec![[0;4];count as usize];
    assert_eq!(result(cad,unsafe { solvent_cad_boundary(cad.as_ptr(),solid,rows.as_mut_ptr().cast(),count) }).unwrap(),count);
    rows
}
fn sample(cad: &native::Session,row: &[c_int;4],t: f64) -> Result<[f64;15],String> {
    let mut values = [0.;15];
    result(cad,unsafe { solvent_cad_boundary_point(cad.as_ptr(),row.as_ptr(),t,values.as_mut_ptr()) })?;
    Ok(values)
}
fn v(sample: &[f64;15],i: usize) -> [f64;3] { [sample[i],sample[i+1],sample[i+2]] }
use gcs_core::space::{dot,norm};
fn frame(sample: &[f64;15]) {
    let t = v(sample,3);
    assert!((norm(t)-1.).abs() < 1e-12);
    for i in [6,9] {
        let n = v(sample,i);
        assert!((norm(n)-1.).abs() < 1e-12);
        assert!(dot(n,t).abs() < 1e-6,"normal/edge orthogonality {}",dot(n,t));
    }
    assert!(sample[12].max(sample[13]) < 1e-5,"edge incidence: {sample:?}");
}

const BOX: &str = "unit mm
use std
profile := std.CenteredRectangle(std.origin,w: 2mm,h: 2mm)
stock := solid(face(profile.loop),depth: 2mm)
body := solid(stock)
";

#[test]
fn a_blind_hole_has_a_concave_floor_edge() {
    let e = fixtures::gear::read(&format!("{BOX}c := circle(center: std.origin)\nradius(0.5mm) c\n\
        drill := solid(face(c),from: -1mm,to: 3mm)\ndrill cut body\n"),Path::new("."));
    let cad = native::Session::new().unwrap();
    let solid = cad.construct(&cad::recipe(&e.sketch,e.map.ent_named("body").unwrap().i()).unwrap()).unwrap();
    let rows = boundary(&cad,solid);
    let mut concave = 0;
    for row in &rows {
        let s = sample(&cad,row,0.37).unwrap();
        if s[14] > 1e-6 {
            assert!((s[14]-std::f64::consts::FRAC_PI_2).abs() < 1e-9);
            assert!((s[1]-1.).abs() < 1e-9 && (s[0].hypot(s[2])-0.5).abs() < 1e-9);
            concave += 1;
        }
    }
    assert_eq!(concave,1);
}

#[test]
fn trimmed_cube_and_hole_edges_have_material_normals() {
    let cad = native::Session::new().unwrap();
    for hole in [false,true] {
        let source = format!("{BOX}{}",if hole { "c := circle(center: std.origin)\nradius(0.5mm) c\n\
            drill := solid(face(c),from: -3mm,to: 3mm)\ndrill cut body\n" } else { "" });
        let e = fixtures::gear::read(&source,Path::new("."));
        let solid = cad.construct(&cad::recipe(&e.sketch,e.map.ent_named("body").unwrap().i()).unwrap()).unwrap();
        let rows = boundary(&cad,solid);
        assert_eq!(rows.len(),if hole { 15 } else { 12 });
        let mut circles = 0;
        for row in &rows {
            assert_eq!(row[3]&2,0);
            for t in [0.13,0.47,0.83] {
                let s = sample(&cad,row,t).unwrap(); frame(&s);
                let expected_angle = if row[3]&1 != 0 { 0. } else { -std::f64::consts::FRAC_PI_2 };
                assert!((s[14]-expected_angle).abs() < 1e-9,"dihedral {}, row={row:?}",s[14]);
                let p = v(&s,0); let center = [0.,1.,0.];
                for i in [6,9] {
                    let n = v(&s,i);
                    if n[1].abs() < 1e-8 && p[0].hypot(p[2]) < 0.6 {
                        assert!((dot(n,[p[0],0.,p[2]])+0.5).abs() < 1e-9,"hole normal must face the void");
                    } else {
                        assert!(dot(n,std::array::from_fn(|k| p[k]-center[k])) > 0.9,
                            "hole={hole}, row={row:?}, p={p:?}, n={n:?}");
                    }
                }
            }
            let s = sample(&cad,row,0.37).unwrap();
            if hole && row[3] == 0 && s[0].hypot(s[2]) < 0.6 {
                // These circles lie at the stock's ends, not the drill's original
                // ends (-3,3). They are edges made by the native Boolean operation.
                assert!(s[1].abs() < 1e-10 || (s[1]-2.).abs() < 1e-10);
                circles += 1;
            }
        }
        assert_eq!(circles,if hole { 2 } else { 0 });
        assert_eq!(rows.iter().filter(|r| r[3]&1 != 0).count(),usize::from(hole));
        assert!(sample(&cad,&rows[0],-0.1).is_err());
        let mut wrong = rows[0];
        wrong[1] = rows.iter().flat_map(|r| [r[1],r[2]])
            .find(|f| *f != rows[0][1] && *f != rows[0][2]).unwrap();
        assert!(sample(&cad,&wrong,0.5).unwrap_err().contains("not incident"));
        let mut small = [0;4];
        assert!(result(&cad,unsafe { solvent_cad_boundary(cad.as_ptr(),solid,small.as_mut_ptr(),1) }).is_err());
    }
}

#[test]
fn periodic_seams_and_collapsed_poles_remain_explicit() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let sphere = fixtures::gear::read(include_str!("../../examples/solid_generating_sweep.sv"),&base);
    let torus = fixtures::gear::read("unit mm\na := point hint(x: 0,y: 0)\nground a\n\
        b := point hint(x: 0,y: 1)\nground b\naxis := line(a,b)\n\
        c := point hint(x: 10,y: 2)\nground c\nring := circle(center: c)\nradius(2) ring\n\
        tool := solid(face(ring),about: axis)\n",Path::new("."));
    let cad = native::Session::new().unwrap();
    for (e,is_sphere) in [(&sphere,true),(&torus,false)] {
        let solid = cad.construct(&cad::recipe(&e.sketch,e.map.ent_named("tool").unwrap().i()).unwrap()).unwrap();
        let rows = boundary(&cad,solid);
        assert_eq!(rows.iter().filter(|r| r[3]&2 != 0).count(),if is_sphere { 2 } else { 0 });
        assert_eq!(rows.iter().filter(|r| r[3]&1 != 0).count(),if is_sphere { 1 } else { 2 });
        for row in &rows {
            if row[3]&2 != 0 { assert!(sample(&cad,row,0.5).is_err()); continue; }
            assert_eq!(row[1],row[2]);
            for t in [0.17,0.53,0.89] {
                let s = sample(&cad,row,t).unwrap(); frame(&s);
                let p = v(&s,0);
                let normal = if is_sphere { [p[0]-3.,p[1],p[2]] } else {
                    let r = p[0].hypot(p[1]);
                    [p[0]*(1.-10./r)/2.,p[1]*(1.-10./r)/2.,(p[2]-2.)/2.]
                };
                for i in [6,9] { assert!(dot(v(&s,i),normal) > 1.-1e-10); }
            }
        }
    }
}

#[test]
fn native_cutter_edges_agree_with_the_independent_source_material() {
    let base = fixtures::gear::project();
    let e = fixtures::gear::read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let cad = native::Session::new().unwrap();
    let scale = e.sketch.units.length.unwrap().1;
    for name in ["pair.reference.tooth.crown","pair.reference.gear_space.body"] {
        let id = e.map.ent_named(name).unwrap().i();
        let field = SpatialField::read(&e.sketch,id,1e-10).unwrap();
        let solid = cad.construct(&cad::recipe(&e.sketch,id).unwrap()).unwrap();
        let rows = boundary(&cad,solid);
        let mut checked = 0;
        for row in &rows {
            if row[3]&2 != 0 { assert!(sample(&cad,row,0.5).is_err()); continue; }
            for t in [0.17,0.47,0.83] {
                let s = sample(&cad,row,t).unwrap(); frame(&s);
                let p = v(&s,0);
                let normal: [f64;3] = std::array::from_fn(|k| s[6+k]+s[9+k]);
                let length = norm(normal); assert!(length > 1e-6);
                let normal = normal.map(|n| n/length);
                let value = |d| field.bounds(std::array::from_fn(|k|
                    Interval::point((p[k]+d*normal[k])/scale).unwrap())).unwrap().bounds();
                let boundary = value(0.);
                assert!(boundary[0].abs().max(boundary[1].abs()) < 1e-5,"{name}: {boundary:?}");
                assert!(value(-1e-3)[1] < 0.,"{name}: inward edge bisector");
                assert!(value(1e-3)[0] > 0.,"{name}: outward edge bisector");
                checked += 1;
            }
        }
        eprintln!("{name}: {} native edges, {checked} boundary/material checks",rows.len());
        assert!(checked >= 18);
    }
}

/// A sliver cell, a chevron 4 µm thick across 45 mm, has no candidate a fixed step in from its
/// faces reaches and no point of a grid over its box: its interior samples are the middles of
/// rays along its faces' inward normals, inside and clear of the 0.2 µm the classifier needs.
#[test]
fn a_sliver_is_sampled_between_its_faces() {
    let mut text = String::from("unit mm\n");
    for (name,x,y) in [("a",0.,0.),("b",20.,10.),("c",40.,0.),("d",40.,0.004),("e",20.,10.004),("f",0.,0.004)] {
        text += &format!("{name} := point hint(x: {x}, y: {y})\nground {name}\n");
    }
    text += "sliver := solid(face(a, b, c, d, e, f, -> close), depth: 10mm)\n";
    let e = fixtures::gear::read(&text,Path::new("."));
    let cad = native::Session::new().unwrap();
    let solid = cad.construct(&cad::recipe(&e.sketch,e.map.ent_named("sliver").unwrap().i()).unwrap()).unwrap();
    let started = std::time::Instant::now();
    let samples = cad.samples(solid,4,12).unwrap();
    eprintln!("{} samples in {:?}: {samples:?}",samples.len(),started.elapsed());
    assert_eq!(samples.len(),4);
    let half = 0.004*(20f64/500f64.sqrt())/2.;
    for (p,boundary) in &samples {
        assert!(*boundary > 2e-4 && *boundary <= half+1e-7,"{p:?} {boundary}");
    }
    let points: Vec<[f64;3]> = samples.iter().map(|s| s.0).collect();
    assert!(cad.solid_contains(solid,&points,1e-7).unwrap().iter().all(|&s| s == 1));
}
