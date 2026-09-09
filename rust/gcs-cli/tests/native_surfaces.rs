//! Native candidate faces from the ordinary source-contact evaluator. No gear grids
//! or Python host are involved; these checks do not claim a closed swept solid.
use gcs_core::{envelope::Contact,solid::SweepContacts};
use std::{ffi::{c_void,c_int,c_char,CStr},path::Path};

mod support;
use support::read;
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="native_surfaces/trimming.rs"]
mod trimming;
#[path="native_surfaces/coverage.rs"]
mod coverage;
#[path="native_surfaces/pcurves.rs"]
mod pcurves;
#[path="native_surfaces/curves.rs"]
mod curves;
#[path="native_surfaces/cells.rs"]
mod cells;

extern "C" {
    fn solvent_cad_error(cad: *mut c_void) -> *const c_char;
    fn solvent_cad_bspline_face(cad: *mut c_void,points: *const f64,nu: c_int,nv: c_int) -> c_int;
    fn solvent_cad_surface_point(cad: *mut c_void,id: c_int,u: f64,v: f64,output: *mut f64) -> c_int;
    fn solvent_cad_validate(cad: *mut c_void,id: c_int) -> c_int;
}
struct Cad(native::Session);
impl Cad {
    fn new() -> Self { Self(native::Session::new().unwrap()) }
    fn raw(&self) -> *mut c_void { self.0.as_ptr() }
    fn result(&self,id: c_int) -> Result<c_int,String> {
        if id < 0 { Err(unsafe { CStr::from_ptr(solvent_cad_error(self.raw())) }.to_string_lossy().into_owned()) }
        else { Ok(id) }
    }
    fn fit(&self,points: &[[f64;3]],nu: usize,nv: usize) -> Result<c_int,String> {
        assert_eq!(points.len(),nu*nv);
        assert!(nu <= 512 && nv <= 512);
        self.result(unsafe { solvent_cad_bspline_face(self.raw(),points.as_ptr().cast(),nu as c_int,nv as c_int) })
    }
    fn at(&self,id: c_int,u: f64,v: f64) -> Result<([f64;3],[f64;3]),String> {
        let mut p = [0.;6];
        self.result(unsafe { solvent_cad_surface_point(self.raw(),id,u,v,p.as_mut_ptr()) })?;
        Ok(([p[0],p[1],p[2]],[p[3],p[4],p[5]]))
    }
}
fn norm(p: [f64;3]) -> f64 { p[0].hypot(p[1]).hypot(p[2]) }
fn distance(a: [f64;3],b: [f64;3]) -> f64 { norm(std::array::from_fn(|k| a[k]-b[k])) }

#[test]
fn native_surface_interpolation_preserves_parameters_and_rejects_bad_data() {
    let cad = Cad::new();
    let points: Vec<_> = (0..=4).flat_map(|i| (0..=4).map(move |j| {
        let u = i as f64/4.; let v = j as f64/4.; [2.*u,3.*v,u*v]
    })).collect();
    let id = cad.fit(&points,5,5).unwrap();
    for (u,v) in [(0.13,0.79),(0.,1.),(0.61,0.37)] {
        let (p,n) = cad.at(id,u,v).unwrap();
        assert!(distance(p,[2.*u,3.*v,u*v]) < 1e-12);
        assert!((norm(n)-1.).abs() < 1e-12);
    }
    assert!(cad.at(id,-0.1,0.5).is_err());
    assert!(cad.at(-1,0.5,0.5).is_err());
    let mut bad = points.clone(); bad[4][1] = f64::NAN;
    assert!(cad.fit(&bad,5,5).unwrap_err().contains("nonfinite"));
    assert!(cad.fit(&[[0.;3]],1,1).is_err());
    // A valid candidate face must never pass the final solid-export check.
    assert!(cad.result(unsafe { solvent_cad_validate(cad.raw(),id) }).is_err());
}

fn check_chart(cad: &Cad,sweep: &SweepContacts,patch: usize,branch: usize,
    scale: f64,subdivisions: usize,roll: [f64;2]) -> (f64,f64) {
    // An interior regular chart, not a claim that the full sweep is regular.
    let contact = |u: f64,t: f64| -> Contact {
        sweep.at(patch,0.05+0.9*u,roll[0]+(roll[1]-roll[0])*t,1e-10).unwrap()
            .into_iter().find(|c| c.branch == branch).unwrap_or_else(|| panic!(
                "{} branch {branch} absent at u={}, roll={}",sweep.patches()[patch].name,
                0.05+0.9*u,roll[0]+(roll[1]-roll[0])*t)).contact
    };
    let mut grid = Vec::new();
    for i in 0..=subdivisions { for j in 0..=subdivisions {
        grid.push(contact(i as f64/subdivisions as f64,j as f64/subdivisions as f64)
            .position.map(|v| v*scale));
    } }
    let id = cad.fit(&grid,subdivisions+1,subdivisions+1).unwrap();
    let (mut error,mut normal_error) = (0_f64,0_f64);
    // Withheld samples between grid knots. Compare position and tangent plane,
    // allowing either orientation: global material orientation is a later step.
    for i in 0..subdivisions { for j in 0..subdivisions {
        let u = (i as f64+0.37)/subdivisions as f64;
        let t = (j as f64+0.63)/subdivisions as f64;
        let expected = contact(u,t);
        let (p,n) = cad.at(id,u,t).unwrap();
        error = error.max(distance(p,expected.position.map(|v| v*scale)));
        let dot = (0..3).map(|k| n[k]*expected.normal[k]).sum::<f64>();
        normal_error = normal_error.max((1.-dot.abs()).abs());
    } }
    (error,normal_error)
}

#[test]
fn native_contact_faces_follow_the_declarative_pair_and_neighbor_motion() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let cad = Cad::new();
    let scale = e.sketch.units.length.unwrap().1;
    let mut count = 0;
    let (mut worst,mut normal) = (0_f64,0_f64);
    for name in ["pinion","gear"] {
        let id = e.map.ent_named(&format!("pair.{name}.removal")).unwrap().i();
        let sweep = SweepContacts::read(&e.sketch,id,1e-10).unwrap();
        if name == "pinion" {
            // A former rectangular fit crossed the contact-domain boundary. Keep
            // the actual missing ring visible: it must not be bridged by a spline.
            let i = sweep.patches().iter().position(|p| p.name.ends_with(".inner")).unwrap();
            assert!(sweep.at(i,0.725,0.025,1e-10).unwrap().is_empty());
        }
        for (i,patch) in sweep.patches().iter().enumerate() {
            // Select the known flank/fillet fixtures for this fitting check.
            // The fitting API and source reader have no such name selection.
            if ![".inner",".outer",".inner_round",".outer_round"].iter()
                .any(|suffix| patch.name.ends_with(suffix)) { continue; }
            for branch in 0..2 {
                let (error,n) = check_chart(&cad,&sweep,i,branch,scale,16,[-0.3,-0.2]);
                assert!(error < 0.002,"{} branch {branch}: {error} mm",patch.name);
                assert!(n < 1e-4,"{} branch {branch}: normal error {n}",patch.name);
                worst = worst.max(error); normal = normal.max(n); count += 1;
            }
        }
    }
    assert_eq!(count,16);
    eprintln!("{count} native contact faces: withheld point error {worst:e} mm, tangent-plane error {normal:e}");
}

#[test]
fn source_parameter_contact_chart_crosses_the_temporal_fold() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let id = e.map.ent_named("pair.pinion.removal").unwrap().i();
    let sweep = SweepContacts::read(&e.sketch,id,1e-10).unwrap();
    let patch = sweep.patches().iter().position(|p| p.name.ends_with(".inner")).unwrap();
    let sample = |u: f64,v: f64| {
        let roots = sweep.at_source(patch,0.65+0.15*u,0.7+0.1*v,1e-10).unwrap();
        assert_eq!(roots.len(),1,"the fixture must stay on one temporal branch");
        roots[0]
    };
    // This source chart crosses the join of the two old (u,time) branches.
    let mut branches = Vec::new();
    for v in [0.,1.] {
        let c = sample(0.5,v);
        let roots = sweep.at(patch,0.725,c.root.time,1e-10).unwrap();
        let branch = roots.iter().find(|r| (r.v-(0.7+0.1*v)).abs() < 1e-9).unwrap();
        branches.push(branch.branch);
    }
    assert_ne!(branches[0],branches[1]);
    assert!(sample(0.5,0.5).root.time > sample(0.5,0.).root.time);
    assert!(sample(0.5,0.5).root.time > sample(0.5,1.).root.time);
    assert!(sweep.at(patch,0.725,0.025,1e-10).unwrap().is_empty());
    let cad = Cad::new();
    let grid: Vec<_> = (0..=32).flat_map(|i| {
        let sample = &sample;
        (0..=32).map(move |j| sample(i as f64/32.,j as f64/32.).contact.position)
    }).collect();
    let id = cad.fit(&grid,33,33).unwrap();
    let (mut error,mut normal_error) = (0_f64,0_f64);
    for i in 0..32 { for j in 0..32 {
        let u = (i as f64+0.37)/32.; let v = (j as f64+0.63)/32.;
        let expected = sample(u,v).contact;
        let (p,n) = cad.at(id,u,v).unwrap();
        error = error.max(distance(p,expected.position));
        let dot = (0..3).map(|k| n[k]*expected.normal[k]).sum::<f64>();
        normal_error = normal_error.max((1.-dot.abs()).abs());
    } }
    assert!(error < 1e-5,"fold-crossing position error {error} mm");
    assert!(normal_error < 1e-8,"fold-crossing tangent-plane error {normal_error}");
    eprintln!("native face across the contact-chart fold: position error {error:e} mm, tangent-plane error {normal_error:e}");
}

#[test]
fn native_sphere_sweep_faces_converge_to_an_independent_torus() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let e = read(include_str!("../../examples/solid_generating_sweep.sv"),&base);
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("removal.body").unwrap().i(),1e-10).unwrap();
    assert_eq!(sweep.patches().len(),1);
    let cad = Cad::new();
    let mut worst = 0_f64;
    for branch in 0..2 {
        let coarse = check_chart(&cad,&sweep,0,branch,1.,4,[-1.,1.]).0;
        let fine = check_chart(&cad,&sweep,0,branch,1.,32,[-1.,1.]).0;
        assert!(coarse > 0.002,"the coarse grid must expose interpolation error");
        assert!(fine < 0.00002 && fine < coarse/100.);
        // Fit a separate grid and evaluate the known torus equation in world
        // coordinates, independently of the contact evaluator at query points.
        let grid: Vec<_> = (0..=32).flat_map(|i| {
            let sweep = &sweep;
            (0..=32).map(move |j| sweep.at(0,0.05+0.9*i as f64/32.,
                -1.+2.*j as f64/32.,1e-10).unwrap().into_iter()
                .find(|c| c.branch == branch).unwrap().contact.position)
        }).collect();
        let id = cad.fit(&grid,33,33).unwrap();
        for i in 0..32 { for j in 0..32 {
            let (p,n) = cad.at(id,(i as f64+0.21)/32.,(j as f64+0.79)/32.).unwrap();
            let radial = p[0].hypot(p[1]);
            let residual = ((radial-3.).hypot(p[2])-1.).abs();
            worst = worst.max(residual);
            assert!(residual < 0.00002,"torus residual {residual}");
            let exact_normal = [p[0]*(1.-3./radial),p[1]*(1.-3./radial),p[2]];
            let dot = (0..3).map(|k| n[k]*exact_normal[k]).sum::<f64>()/norm(exact_normal);
            assert!((1.-dot.abs()).abs() < 1e-6);
        } }
    }
    eprintln!("native sphere sweep: independent torus residual {worst:e} mm");
}
