//! Native intersection curves trim candidate faces without changing their charts.
use super::*;
use gcs_core::{interval::{Interval,minimum::Options},solid::{cad,MaterialField,PlanarField,RevolvedField,SpatialField,ProbeState}};

extern "C" {
    fn solvent_cad_split_face(cad: *mut c_void,source: c_int,tools: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_faces(cad: *mut c_void,source: c_int,output: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_face_contains(cad: *mut c_void,face: c_int,u: f64,v: f64,tolerance: f64) -> c_int;
}
impl Cad {
    fn faces(&self,source: c_int) -> Vec<c_int> {
        let count = self.result(unsafe { solvent_cad_faces(self.raw(),source,std::ptr::null_mut(),0) }).unwrap();
        let mut faces = vec![-1;count as usize];
        assert_eq!(self.result(unsafe { solvent_cad_faces(self.raw(),source,faces.as_mut_ptr(),count) }).unwrap(),count);
        faces
    }
    fn split(&self,source: c_int,tools: &[c_int]) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_split_face(self.raw(),source,tools.as_ptr(),tools.len() as c_int) })
    }
    fn contains(&self,face: c_int,u: f64,v: f64) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_face_contains(self.raw(),face,u,v,1e-9) })
    }
    fn grid(&self,n: usize,at: impl Fn(f64,f64)->[f64;3]) -> c_int {
        let points: Vec<_> = (0..=n).flat_map(|i| {
            let at = &at;
            (0..=n).map(move |j| at(i as f64/n as f64,j as f64/n as f64))
        }).collect();
        self.fit(&points,n+1,n+1).unwrap()
    }
}

#[test]
fn native_curved_trim_partitions_the_original_chart_without_overlap_or_gaps() {
    let cad = Cad::new();
    let source = cad.grid(4,|u,v| [2.*u-1.,2.*v-1.,0.]);
    let tool = cad.grid(4,|u,v| { let y = 3.*u-1.5; [0.25-y*y,y,2.*v-1.] });
    let pieces = cad.faces(cad.split(source,&[tool]).unwrap());
    assert_eq!(pieces.len(),2);
    let mut sides = [None;2];
    let mut count = 0;
    for i in 0..17 { for j in 0..17 {
        let u = (i as f64+0.37)/17.; let v = (j as f64+0.63)/17.;
        let (p,_) = cad.at(source,u,v).unwrap();
        let signed = p[0]+p[1]*p[1]-0.25;
        assert!(signed.abs() > 1e-5);
        let membership: Vec<_> = pieces.iter().map(|f| cad.contains(*f,u,v).unwrap()).collect();
        assert_eq!(membership.iter().filter(|s| **s == 1).count(),1,"{p:?}: {membership:?}");
        assert!(membership.iter().all(|s| *s <= 1));
        let k = membership.iter().position(|s| *s == 1).unwrap();
        if let Some(side) = sides[k] { assert_eq!(side,signed > 0.); } else { sides[k] = Some(signed > 0.); }
        // Splitting keeps the original supporting surface and its parameterization.
        assert!(distance(cad.at(pieces[k],u,v).unwrap().0,p) < 1e-12);
        assert_eq!(cad.contains(source,u,v).unwrap(),1);
        count += 1;
    } }
    assert_ne!(sides[0],sides[1]);
    for y in [-0.7,0.,0.6] {
        let (u,v) = ((1.25-y*y)/2.,(y+1.)/2.);
        // Kernel intersection curves carry their own numerical tolerance. Check
        // the independent parabola is bracketed within 1e-5 mm, not exact UV equality.
        for f in &pieces {
            let a = cad.contains(*f,u-5e-6,v).unwrap();
            let b = cad.contains(*f,u+5e-6,v).unwrap();
            assert_eq!(a+b,1);
        }
    }
    assert_eq!(cad.contains(source,0.,0.5).unwrap(),2);
    assert!(cad.split(source,&[]).is_err());
    assert!(cad.split(source,&[-1]).is_err());
    assert!(cad.contains(source,f64::NAN,0.5).is_err());
    assert!(cad.result(unsafe { solvent_cad_validate(cad.raw(),pieces[0]) }).is_err());
    let remote = cad.grid(4,|u,v| [2.*u-1.,2.*v-1.,2.]);
    let unsplit = cad.faces(cad.split(source,&[remote]).unwrap());
    assert_eq!(unsplit.len(),1);
    assert_eq!(cad.contains(unsplit[0],0.5,0.5).unwrap(),1);
    eprintln!("curved native trim: {count} independent chart partition checks");
}

#[test]
fn a_closed_native_trim_retains_both_the_disk_and_the_face_with_a_hole() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let e = read(include_str!("../../../examples/solid_generating_sweep.sv"),&base);
    let cad = Cad::new();
    let tool = cad.0.construct(&cad::recipe(&e.sketch,e.map.ent_named("tool").unwrap().i()).unwrap()).unwrap();
    let source = cad.grid(4,|u,v| [1.+4.*u,-2.+4.*v,0.]);
    let split = cad.split(source,&cad.faces(tool)).unwrap();
    let pieces = cad.faces(split);
    assert_eq!(pieces.len(),2);
    let mut sides = [None;2];
    for i in 0..17 { for j in 0..17 {
        let u = (i as f64+0.37)/17.; let v = (j as f64+0.63)/17.;
        let p = cad.at(source,u,v).unwrap().0;
        let radius = (p[0]-3.).hypot(p[1]);
        assert!((radius-1.).abs() > 1e-5);
        let membership: Vec<_> = pieces.iter().map(|f| cad.contains(*f,u,v).unwrap()).collect();
        assert_eq!(membership.iter().filter(|s| **s == 1).count(),1,"{p:?}: {membership:?}");
        let k = membership.iter().position(|s| *s == 1).unwrap();
        if let Some(side) = sides[k] { assert_eq!(side,radius < 1.); } else { sides[k] = Some(radius < 1.); }
    } }
    assert!(sides.contains(&Some(true)) && sides.contains(&Some(false)));
    let disk = pieces[sides.iter().position(|s| *s == Some(true)).unwrap()];
    let hole = pieces[sides.iter().position(|s| *s == Some(false)).unwrap()];
    assert_eq!(cad.contains(disk,0.5,0.5).unwrap(),1);
    assert_eq!(cad.contains(hole,0.5,0.5).unwrap(),0);
    let mut small = [-1];
    assert!(cad.result(unsafe { solvent_cad_faces(cad.raw(),split,small.as_mut_ptr(),1) }).is_err());
    assert_eq!(small,[-1]);
    assert!(cad.split(source,&[split]).is_err());
    assert!(cad.result(unsafe { solvent_cad_face_contains(cad.raw(),source,0.5,0.5,0.) }).is_err());
}

#[test]
fn native_sweep_fragments_follow_the_complete_material_after_a_planar_cut() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let e = read(include_str!("../../../examples/solid_generating_sweep.sv"),&base);
    let id = e.map.ent_named("removal.body").unwrap().i();
    let sweep = SweepContacts::read(&e.sketch,id,1e-10).unwrap();
    let half = SpatialField::from(RevolvedField::new(
        PlanarField::half_plane([0.;2],[0.,-1.]).unwrap(),[0.;3],[0.,1.,0.]).unwrap());
    let mut material = MaterialField::read(&e.sketch,id,1e-10).unwrap()
        .intersection(half.into()).unwrap().evaluator(1024);
    let cad = Cad::new();
    let source = cad.grid(32,|u,v| sweep.at(0,0.05+0.9*u,-1.+2.*v,1e-10).unwrap()
        .into_iter().find(|c| c.branch == 0).unwrap().contact.position);
    let tool = cad.grid(4,|u,v| [12.*u-6.,0.,4.*v-2.]);
    let pieces = cad.faces(cad.split(source,&[tool]).unwrap());
    assert_eq!(pieces.len(),2);
    let mut states = [None;2];
    for u in [0.23,0.71] { for v in [0.19,0.37,0.63,0.81] {
        let membership: Vec<_> = pieces.iter().map(|f| cad.contains(*f,u,v).unwrap()).collect();
        assert_eq!(membership.iter().filter(|s| **s == 1).count(),1);
        let fragment = membership.iter().position(|s| *s == 1).unwrap();
        let (p,n) = cad.at(pieces[fragment],u,v).unwrap();
        assert!(p[1].abs() > 0.1);
        let probe = material.probe(p.map(|x| Interval::point(x).unwrap()),n,0.01,
            Options {value_tolerance:1e-4,max_evaluations:20000}).unwrap();
        if p[1] < 0. { assert_eq!(probe.state,ProbeState::ExteriorBall); }
        else { assert!(matches!(probe.state,ProbeState::OutwardBracket | ProbeState::InwardBracket)); }
        if let Some(state) = states[fragment] { assert_eq!(probe.state,state); }
        else { states[fragment] = Some(probe.state); }
    } }
    assert!(states.contains(&Some(ProbeState::ExteriorBall)));
    assert_ne!(states[0],states[1]);
}

#[test]
fn source_gear_charts_split_at_the_declared_toe_boundary() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let toe = e.map.ent_named("pair.reference.toe.carrier").unwrap().i();
    let field = gcs_core::solid::SpatialField::read(&e.sketch,toe,1e-10).unwrap();
    let cad = Cad::new();
    let solid = cad.0.construct(&cad::recipe(&e.sketch,toe).unwrap()).unwrap();
    let tools = cad.faces(solid);
    for member in ["pinion","gear"] {
        let id = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
        let sweep = SweepContacts::read(&e.sketch,id,1e-10).unwrap();
        // Locate a test chart where an active source flank crosses the toe sphere.
        // These motions preserve distance from the apex. This seed search is not
        // a coverage algorithm; the native split below constructs the trim curve.
        let mut seed = None;
        'patches: for (patch,surface) in sweep.patches().iter().enumerate() {
            if ![".inner",".outer"].iter().any(|s| surface.name.ends_with(s)) { continue; }
            let value = |v| {
                let p = surface.at(0.5,v).unwrap().position;
                field.bounds(p.map(|x| Interval::point(x).unwrap())).unwrap().bounds()[0]
            };
            for j in 0..64 {
                let mut lo = j as f64/64.; let mut hi = (j+1) as f64/64.;
                let positive = value(lo) > 0.;
                if positive == (value(hi) > 0.) { continue; }
                for _ in 0..40 {
                    let mid = (lo+hi)/2.;
                    if (value(mid) > 0.) == positive { lo = mid; } else { hi = mid; }
                }
                let v = (lo+hi)/2.;
                for contact in sweep.at_source(patch,0.5,v,1e-10).unwrap() {
                    let t = contact.root.time; let domain = sweep.domain();
                    if t > domain[0]+0.02 && t < domain[1]-0.02 {
                        seed = Some((patch,v,contact.root)); break 'patches;
                    }
                }
            }
        }
        let (patch,center,root) = seed.expect("an active source chart must cross the toe within the declared motion");
        let at = |u: f64,v: f64| {
            sweep.at_source(patch,0.49+0.02*u,center+0.01*(v-0.5),1e-10).unwrap()
                .into_iter().find(|c| c.root.branch == root.branch && c.root.turn == root.turn)
                .expect("the local test chart must retain its temporal branch").contact.position
        };
        let source = cad.grid(16,at);
        let pieces = cad.faces(cad.split(source,&tools).unwrap());
        assert_eq!(pieces.len(),2,"{member}: expected a toe crossing");
        let mut sides = [None;2];
        let mut worst = 0_f64;
        for i in 0..13 { for j in 0..13 {
            let u = (i as f64+0.37)/13.; let v = (j as f64+0.63)/13.;
            let (p,_) = cad.at(source,u,v).unwrap();
            worst = worst.max(distance(p,at(u,v)));
            let b = field.bounds(p.map(|x| Interval::point(x).unwrap())).unwrap().bounds();
            assert!(b[0] > 1e-5 || b[1] < -1e-5,"fixture sample too close to trim");
            let membership: Vec<_> = pieces.iter().map(|f| cad.contains(*f,u,v).unwrap()).collect();
            assert_eq!(membership.iter().filter(|s| **s == 1).count(),1,"{member} {p:?}: {membership:?}");
            let k = membership.iter().position(|s| *s == 1).unwrap();
            let outside = b[0] > 0.;
            if let Some(side) = sides[k] { assert_eq!(side,outside); } else { sides[k] = Some(outside); }
            assert!(distance(cad.at(pieces[k],u,v).unwrap().0,p) < 1e-10);
        } }
        assert!(sides.contains(&Some(true)) && sides.contains(&Some(false)));
        assert!(worst < 1e-4,"{member}: local chart interpolation error {worst}");
        eprintln!("{member}: native toe trim, 169 independent partition checks, withheld fit error {worst:e} mm");
    }
}
