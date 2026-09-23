use gcs_core::{io,model::{EntKind,MotionDef},motion,program,solve,syntax};

mod bounds;
mod contact;

const AXES: &str = "unit mm
point a hint(x: 2,y: 0)
point b hint(x: 2,y: 1)
point o hint(x: 0,y: 0)
point x hint(x: 1,y: 0)
ground a
ground b
ground o
ground x
line axis(a,b)
line other(o,x)
";

fn build(src: &str) -> program::Elaborated {
    let (p,errors) = syntax::parse(src);
    assert!(errors.is_empty(),"{errors:?}");
    program::elaborate(&p)
}
fn solved(src: &str) -> program::Elaborated {
    let mut e = build(src);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    e
}
fn near(a: [f64;3], b: [f64;3], tol: f64) {
    for i in 0..3 { assert!((a[i]-b[i]).abs() < tol,"{a:?} != {b:?}"); }
}

#[test]
fn inverse_motion_speed_bounds_cover_offset_axes_and_shared_relative_graphs() {
    let e = solved(&format!("{AXES}\n\
        motion turn(about: axis,ratio: 2,phase: 90deg)\n\
        motion observer(about: other,ratio: -3,phase: 20deg)\n\
        motion relative(turn,relative_to: observer)\n\
        motion nested(relative,relative_to: turn)\n"));
    for name in ["turn","observer","relative","nested"] {
        let family = motion::Family::read(&e.sketch,e.map.ent_named(name).unwrap().i()).unwrap();
        for p in [[0.;3],[0.,3.,4.],[3.,1.,4.],[-20.,30.,-5.]] {
            let upper = family.inverse_point_speed_bound(p,gcs_core::interval::Interval::new(-50.,50.).unwrap()).unwrap();
            for t in [-20.,-1.,0.,0.7,50.] {
                let v = family.at(t).unwrap().inverse().velocity(p);
                assert!(v[0].hypot(v[1]).hypot(v[2]) <= upper);
            }
            if name == "observer" && p == [0.,3.,4.] {
                // Rotation about x through the origin, |omega|=3 and perpendicular radius=5.
                assert!(upper >= 15. && upper < 15.+1e-10);
            }
        }
        assert!(family.inverse_point_speed_bound([f64::NAN,0.,0.],gcs_core::interval::Interval::new(-50.,50.).unwrap()).is_err());
        assert!(family.inverse_point_speed_bound([f64::MAX;3],gcs_core::interval::Interval::new(-50.,50.).unwrap()).is_err());
    }
}

#[test]
fn motions_follow_directed_axes_with_phase_and_exact_relative_velocity() {
    let e = solved(&format!("{AXES}motion relative(turn,relative_to: observer)\n\
        motion turn(about: axis,ratio: 2,phase: 90deg)\n\
        motion observer(about: other,ratio: -0.3,phase: 20deg)\n"));
    let index = |n| e.map.ent_named(n).unwrap().i();
    let turn = motion::evaluate(&e.sketch,index("turn"),0.).unwrap();
    near(turn.point([3.,0.,0.]),[2.,1.,0.],1e-12);
    near(turn.point([2.,0.,5.]),[2.,0.,5.],1e-12);
    let p = [3.,1.,4.];
    for t in [-1.,0.,0.7] {
        let relative = motion::evaluate(&e.sketch,index("relative"),t).unwrap();
        let source = motion::evaluate(&e.sketch,index("turn"),t).unwrap();
        let observer = motion::evaluate(&e.sketch,index("observer"),t).unwrap();
        near(observer.point(relative.point(p)),source.point(p),1e-12);
        let h = 1e-6;
        let before = motion::evaluate(&e.sketch,index("relative"),t-h).unwrap().point(p);
        let after = motion::evaluate(&e.sketch,index("relative"),t+h).unwrap().point(p);
        near(relative.velocity(p),std::array::from_fn(|k| (after[k]-before[k])/(2.*h)),1e-8);
    }
    assert!(motion::evaluate(&e.sketch,999,0.).is_err());
    assert!(motion::evaluate(&e.sketch,0,f64::NAN).is_err());
}

#[test]
fn motions_round_trip_and_remap_transitive_dependencies() {
    let src = format!("{AXES}motion a_relative(turn,relative_to: observer)\n\
        motion turn(about: axis)\nmotion observer(about: other,ratio: -2)\n");
    let e = solved(&src);
    let mut p = e.program.clone();
    let text = syntax::render_flat(&mut p).unwrap().to_string();
    assert_eq!(solved(&text).sketch.motions.len(),3);
    let relative = e.map.ent_named("a_relative").unwrap();
    assert!(e.sketch.entity_params(relative).is_empty());
    let clip = io::copy(&e.sketch,&[relative]);
    assert_eq!(clip.motions.len(),3);
    let mut dst = e.sketch.clone();
    io::paste(&mut dst,&clip,0.,0.);
    assert_eq!(dst.motions.len(),6);
    for i in 0..3 {
        near(motion::evaluate(&clip,i,0.3).unwrap().point([1.,2.,3.]),
            motion::evaluate(&dst,i+3,0.3).unwrap().point([1.,2.,3.]),1e-12);
    }
    let deleted = io::without(&e.sketch,&[e.map.ent_named("axis").unwrap()],&[]);
    assert_eq!(deleted.motions.len(),1);
    assert!(matches!(deleted.motions[0].def,MotionDef::Rotation {..}));
    assert!(io::copy(&e.sketch,&[e.map.ent_named("o").unwrap()]).motions.is_empty());
}

#[test]
fn motion_component_formals_preserve_private_dependencies() {
    let src = format!("{AXES}component Rotate(ax: line,rate: Scalar) {{\n\
        private motion spin(about: ax,ratio: rate)\n\
        motion out(spin,relative_to: spin)\n}}\n\
        component Observe(m: motion) {{ motion out(m,relative_to: m) }}\n\
        copy: Observe(part.out)\npart: Rotate(axis,rate: 2)\n");
    let e = solved(&src);
    assert!(e.map.entity_path(&e.sketch,"part.spin").is_none());
    let out = e.map.ent_named("copy.out").unwrap();
    assert_eq!(out.kind,EntKind::Motion);
    near(motion::evaluate(&e.sketch,out.i(),0.7).unwrap().point([1.,2.,3.]),[1.,2.,3.],1e-12);
    let bad = build(&format!("{src}motion bad(part.spin,relative_to: part.out)\n"));
    assert!(bad.errors().any(|d| d.message.contains("private member")),"{:?}",bad.diags);
}

#[test]
fn motions_reject_wrong_units_missing_references_and_cycles() {
    for (tail,want) in [
        ("motion bad(about: o)","directed line"),
        ("motion bad(about: missing)","no such motion axis"),
        ("motion bad(about: axis,ratio: 2mm)","ratio"),
        ("motion bad(about: axis,phase: 2mm)","phase"),
        ("motion bad(axis,relative_to: axis)","does not name a motion"),
        ("motion ma(mb,relative_to: mb)\nmotion mb(ma,relative_to: ma)","cycle"),
    ] {
        let e = build(&format!("{AXES}{tail}\n"));
        assert!(!e.ok() && e.errors().any(|d| d.message.contains(want)),"{:?}",e.diags);
    }
    for args in ["", "about: axis,relative_to: a", "about: axis,about: axis",
        "a,relative_to: a,ratio: 1", "bogus: axis"] {
        let (_,errors) = syntax::parse(&format!("motion bad({args})\n"));
        assert!(!errors.is_empty(),"accepted {args}");
    }
}

#[test]
fn motion_axes_use_world_geometry_and_follow_solved_point_edits() {
    let mut e = solved("unit mm
point o hint(x: 7,y: -3)
point q hint(x: 7,y: -2)
ground o
ground q
plane side(origin: o,toward: q,u: (1,0,0),v: (0,0,1))
in side {
  point a
  point b
  a distance(2mm,along: u) side
  a distance(0mm,along: v) side
  b distance(2mm,along: u) side
  b distance(1mm,along: v) side
  line axis(a,b)
}
motion turn(about: axis,phase: 90deg)
");
    let snapshot = motion::Family::read(&e.sketch,0).unwrap();
    let m = motion::evaluate(&e.sketch,0,0.).unwrap();
    near(m.point([3.,0.,0.]),[2.,1.,0.],1e-10);
    near(m.point([2.,0.,5.]),[2.,0.,5.],1e-10);
    // Editing the solved page points changes the next evaluation, without retaining a pose.
    for name in ["a","b"] {
        let i = e.map.ent_named(name).unwrap().i();
        let params = e.sketch.point_params(i);
        e.sketch.params[params[1] as usize].value += 1.;
    }
    near(snapshot.at(0.).unwrap().point([3.,0.,0.]),[2.,1.,0.],1e-10);
    let after = motion::evaluate(&e.sketch,0,0.).unwrap();
    near(after.point([4.,0.,0.]),[3.,1.,0.],1e-10);
    let a = e.map.ent_named("a").unwrap().i();
    let b = e.map.ent_named("b").unwrap().i();
    let xy = e.sketch.point_xy(a);
    let bp = e.sketch.point_params(b);
    e.sketch.params[bp[0] as usize].value = xy.0;
    e.sketch.params[bp[1] as usize].value = xy.1;
    assert!(motion::evaluate(&e.sketch,0,0.).unwrap_err().contains("nondegenerate"));
    assert!(program::solid_diagnostics(&e.sketch,&e.map).iter()
        .any(|d| d.message.contains("nondegenerate")));
}

#[test]
fn motion_dependency_depth_is_bounded_and_runtime_cycles_are_rejected() {
    let mut src = format!("{AXES}motion base(about: axis)\n");
    for i in 0..70 {
        let next = if i == 69 { "base".to_string() } else { format!("m{:02}",i+1) };
        src.push_str(&format!("motion m{i:02}({next},relative_to: base)\n"));
    }
    let e = build(&src);
    assert!(e.errors().any(|d| d.message.contains("64 levels")),"{:?}",e.diags);
    let mut e = solved(&format!("{AXES}motion base(about: axis)\n"));
    e.sketch.motions[0].def = MotionDef::Relative {source:0,observer:0};
    assert!(motion::evaluate(&e.sketch,0,0.).unwrap_err().contains("cycle"));
}

#[test]
fn cached_motion_dependencies_do_not_bypass_the_depth_limit() {
    let mut src = format!("{AXES}motion base(about: axis)\n");
    for i in 0..70 {
        let prev = if i == 0 { "base".to_string() } else { format!("m{:02}",i-1) };
        src.push_str(&format!("motion m{i:02}({prev},relative_to: base)\n"));
    }
    let e = build(&src);
    assert!(e.errors().any(|d| d.message.contains("64 levels")),"{:?}",e.diags);
    assert_eq!(e.sketch.motions.len(),64);
}

/// A screw advances along its axis by `advance` per turn; a translation moves
/// without turning. Poses, derivatives, interval bounds and the speed bound
/// are checked against closed forms.
#[test]
fn screws_and_translations_follow_their_closed_forms() {
    use gcs_core::interval::Interval as I;
    let e = solved(&format!("{AXES}\n\
        motion tap(about: axis,ratio: 2,phase: 30deg,advance: 3mm)\n\
        motion feed(along: other,advance: 10mm)\n\
        motion relative(tap,relative_to: feed)\n"));
    let tap = motion::Family::read(&e.sketch,e.map.ent_named("tap").unwrap().i()).unwrap();
    let feed = motion::Family::read(&e.sketch,e.map.ent_named("feed").unwrap().i()).unwrap();
    let relative = motion::Family::read(&e.sketch,e.map.ent_named("relative").unwrap().i()).unwrap();
    let (axis_a,axis_b) = ([e.sketch.lines[0].p1,e.sketch.lines[0].p2],[e.sketch.lines[1].p1,e.sketch.lines[1].p2]);
    let direction = |ends: [u32;2]| -> [f64;3] {
        let a = e.sketch.world_point(ends[0] as usize); let b = e.sketch.world_point(ends[1] as usize);
        let d: [f64;3] = std::array::from_fn(|k| b[k]-a[k]); let n = d[0].hypot(d[1]).hypot(d[2]); d.map(|v| v/n)
    };
    let (da,db) = (direction(axis_a),direction(axis_b));
    let origin = e.sketch.world_point(axis_a[0] as usize);
    for t in [-2.,0.,0.4,3.] {
        // The screw: the pure rotation's pose, then a slide of advance*t/tau along the axis.
        let pose = tap.at(t).unwrap();
        let turn = gcs_core::envelope::Motion::rotation(da,30_f64.to_radians()+2.*t,2.).unwrap();
        let slide = 3.*t/std::f64::consts::TAU;
        for p in [[0.;3],[1.,2.,3.],[-4.,0.5,2.]] {
            let q = pose.point(p);
            let r: [f64;3] = std::array::from_fn(|k| p[k]-origin[k]);
            let turned = turn.point(r);
            let expected: [f64;3] = std::array::from_fn(|k| turned[k]+origin[k]+da[k]*slide);
            assert!((0..3).all(|k| (q[k]-expected[k]).abs() < 1e-12));
            let v = pose.velocity(p);
            let expected_v: [f64;3] = { let w = turn.velocity(r); std::array::from_fn(|k| w[k]+da[k]*3./std::f64::consts::TAU) };
            assert!((0..3).all(|k| (v[k]-expected_v[k]).abs() < 1e-12));
            let moved = feed.at(t).unwrap();
            let f = moved.point(p);
            assert!((0..3).all(|k| (f[k]-(p[k]+db[k]*10.*t/std::f64::consts::TAU)).abs() < 1e-12));
            let fv = moved.velocity(p);
            assert!((0..3).all(|k| (fv[k]-db[k]*10./std::f64::consts::TAU).abs() < 1e-12));
        }
    }
    // Interval bounds contain the exact poses; the speed bound holds for the
    // inverse motion over the domain, including the relative composition.
    let domain = I::new(-2.,3.).unwrap();
    for family in [&tap,&feed,&relative] {
        let bounds = family.bounds(domain).unwrap();
        for p in [[0.;3],[1.,2.,3.],[-4.,0.5,2.]] {
            let upper = family.inverse_point_speed_bound(p,domain).unwrap();
            let boxed = bounds.point(p.map(|x| I::point(x).unwrap())).unwrap();
            for i in 0..=20 {
                let t = -2.+5.*i as f64/20.;
                let q = family.at(t).unwrap().point(p);
                assert!((0..3).all(|k| { let [lo,hi] = boxed[k].bounds(); lo <= q[k] && q[k] <= hi }),"{} pose outside its bounds",family.name);
                let v = family.at(t).unwrap().inverse().velocity(p);
                assert!(v[0].hypot(v[1]).hypot(v[2]) <= upper,"{} speed {} exceeds bound {upper}",family.name,v[0].hypot(v[1]).hypot(v[2]));
            }
        }
    }
    // The sinusoidal contact reduction is for pure rotations only.
    let surface = gcs_core::envelope::SurfacePoint {position:[1.,2.,3.],du:[1.,0.,0.],dv:[0.,1.,0.]};
    assert!(tap.normal_velocity(surface).is_err());
    assert!(feed.normal_velocity(surface).is_err());
}

/// The position-only pose is the full pose without its derivative, for every kind of step:
/// offset rotations, screws, translations and relative motions of them.
#[test]
fn a_pose_alone_is_the_motion_without_its_derivative() {
    let e = solved(&format!("{AXES}\n\
        motion tap(about: axis,ratio: 2,phase: 30deg,advance: 3mm)\n\
        motion feed(along: other,advance: 10mm)\n\
        motion turn(about: axis,ratio: 2,phase: 90deg)\n\
        motion observer(about: other,ratio: -3,phase: 20deg)\n\
        motion relative(tap,relative_to: feed)\n\
        motion rolled(turn,relative_to: observer)\n\
        motion nested(rolled,relative_to: turn)\n"));
    for name in ["tap","feed","turn","relative","rolled","nested"] {
        let family = motion::Family::read(&e.sketch,e.map.ent_named(name).unwrap().i()).unwrap();
        for k in -8..=8 {
            let angle = 0.37*k as f64;
            let (full,pose) = (family.at(angle).unwrap(),family.pose_at(angle).unwrap());
            for x in [[0.,0.,0.],[1.,-2.,3.5],[-4.,0.5,2.]] {
                near(full.point(x),pose.point(x),1e-12);
                near(full.inverse().point(x),pose.inverse().point(x),1e-12);
            }
        }
    }
}
