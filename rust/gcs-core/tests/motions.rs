use gcs_core::{io,model::{EntKind,EntRef,MotionDef,MotionE},motion,program,solve,syntax};
use gcs_core::program::Elaborated;
use crate::common::parsed::{build, solved};

mod bounds;
mod contact;

const AXES: &str = "\
unit mm
use std
in std.front {
a := point
b := point
o := point
x := point
fix((2, 0)) a
fix((2, 1)) b
fix((0, 0)) o
fix((1, 0)) x
ax := line(a,b)
other := line(o,x)
}
";

/// The motion node frame `name`'s joint was lowered to (#383).
fn joint(e: &Elaborated, name: &str) -> usize {
    let f = e.map.ent_named(name).unwrap_or_else(|| panic!("no frame `{name}`"));
    assert_eq!(f.kind,EntKind::Frame);
    let j = e.sketch.frames[f.i()].joint.as_ref()
        .unwrap_or_else(|| panic!("`{name}` has no joint"));
    j.motion.unwrap_or_else(|| panic!("`{name}`'s joint was not lowered")) as usize
}

/// A reading of frame `under` seen from frame `seen_from` — a point carried by it, `x` as the
/// relative's pose at roll 0 takes it — which is what mints the relative motion.
fn reading(under: &str, seen_from: &str) -> String {
    format!("seen_{under}_{seen_from} := point\n\
             seen_{under}_{seen_from} coincident(under: {under}, seen_from: {seen_from}, \
             t == 0deg) x\n")
}

/// The relative motion node of frame `under` seen from frame `seen_from`, minted by a reading.
fn seen(e: &Elaborated, under: &str, seen_from: &str) -> usize {
    let (s,o) = (joint(e,under) as u32,joint(e,seen_from) as u32);
    e.sketch.motions.iter().position(|m| matches!(m.def,
        MotionDef::Relative {source,observer} if source == s && observer == o))
        .unwrap_or_else(|| panic!("no reading of `{under}` seen from `{seen_from}`"))
}

/// A relative motion node pushed by hand: a document only reads one frame seen from another,
/// and the reader of nested relatives is held to its limits through these.
fn push_relative(sk: &mut gcs_core::model::Sketch, source: usize, observer: usize) -> usize {
    sk.motions.push(MotionE {
        def: MotionDef::Relative {source: source as u32,observer: observer as u32},
        name: format!("m{}",sk.motions.len()),class: Default::default(),measured: Vec::new()});
    sk.motions.len()-1
}

fn near(a: [f64;3], b: [f64;3], tol: f64) {
    for i in 0..3 { assert!((a[i]-b[i]).abs() < tol,"{a:?} != {b:?}"); }
}

#[test]
fn inverse_motion_speed_bounds_cover_offset_axes_and_shared_relative_graphs() {
    // the two relatives share both frames' nodes: each reads the other's observer as its source
    let e = solved(&format!("{AXES}\n\
        turn := frame\nstd.Turn(turn,about: ax,ratio: 2)\n\
        observer := frame\nstd.Turn(observer,about: other,ratio: -3)\n{}{}",
        reading("turn","observer"),reading("observer","turn")));
    let nodes = [("turn",joint(&e,"turn")),("observer",joint(&e,"observer")),
        ("relative",seen(&e,"turn","observer")),("reverse",seen(&e,"observer","turn"))];
    for (name,node) in nodes {
        let family = motion::Family::read(&e.sketch,node).unwrap();
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
fn motions_follow_directed_axes_and_exact_relative_velocity() {
    // the reading before the frames it reads
    let e = solved(&format!("{AXES}{}\
        turn := frame\nstd.Turn(turn,about: ax,ratio: 2)\n\
        observer := frame\nstd.Turn(observer,about: other,ratio: -0.3)\n",
        reading("turn","observer")));
    let (turn,observer,relative) =
        (joint(&e,"turn"),joint(&e,"observer"),seen(&e,"turn","observer"));
    // a quarter turn on, at roll 45°: about the axis's own direction, world z
    let quarter = motion::evaluate(&e.sketch,turn,std::f64::consts::FRAC_PI_4).unwrap();
    near(quarter.point([3.,0.,0.]),[2.,1.,0.],1e-12);
    near(quarter.point([2.,0.,5.]),[2.,0.,5.],1e-12);
    // roll 0 is as drawn
    near(motion::evaluate(&e.sketch,turn,0.).unwrap().point([3.,0.,0.]),[3.,0.,0.],1e-12);
    let p = [3.,1.,4.];
    for t in [-1.,0.,0.7] {
        let relative_at = motion::evaluate(&e.sketch,relative,t).unwrap();
        let source = motion::evaluate(&e.sketch,turn,t).unwrap();
        let observer = motion::evaluate(&e.sketch,observer,t).unwrap();
        near(observer.point(relative_at.point(p)),source.point(p),1e-12);
        let h = 1e-6;
        let before = motion::evaluate(&e.sketch,relative,t-h).unwrap().point(p);
        let after = motion::evaluate(&e.sketch,relative,t+h).unwrap().point(p);
        near(relative_at.velocity(p),std::array::from_fn(|k| (after[k]-before[k])/(2.*h)),1e-8);
    }
    assert!(motion::evaluate(&e.sketch,999,0.).is_err());
    assert!(motion::evaluate(&e.sketch,0,f64::NAN).is_err());
}

#[test]
fn motions_round_trip_and_remap_transitive_dependencies() {
    // each joint stated as the statements `std.Turn` writes: the canonical printer prints no
    // component instance
    let src = format!("{AXES}{}turn := frame\nax coincident turn.axis\n\
        fix(ratio == 1,advance == 0mm) turn\nobserver := frame\nother coincident observer.axis\n\
        fix(ratio == -2,advance == 0mm) observer\n",reading("turn","observer"));
    let e = solved(&src);
    let mut p = e.program.clone();
    let text = syntax::render_flat(&mut p).unwrap().to_string();
    assert_eq!(solved(&text).sketch.motions.len(),3);
    let relative = EntRef::new(EntKind::Motion,seen(&e,"turn","observer"));
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
    let deleted = io::without(&e.sketch,&[e.map.ent_named("ax").unwrap()],&[]);
    assert_eq!(deleted.motions.len(),1);
    assert!(matches!(deleted.motions[0].def,MotionDef::Rotation {..}));
    assert!(io::copy(&e.sketch,&[e.map.ent_named("o").unwrap()]).motions.is_empty());
}

#[test]
fn motion_component_formals_preserve_private_dependencies() {
    // a frame seen from itself stands still: the private one inside, the public one outside
    let src = format!("{AXES}component Rotate(ax: line,x: point,rate: Scalar) {{\n\
        private spin := frame\nstd.Turn(spin,about: ax,ratio: rate)\n\
        turn := frame\nstd.Turn(turn,about: ax,ratio: rate)\n\
        out := point\nout coincident(under: spin,seen_from: spin,t == 40deg) x\n}}\n\
        component Observe(m: frame,x: point) {{\n\
        out := point\nout coincident(under: m,seen_from: m,t == 40deg) x\n}}\n\
        copy := Observe(part.turn,x)\npart := Rotate(ax,x,rate: 2)\n");
    let e = solved(&src);
    assert!(e.map.entity_path(&e.sketch,"part.spin").is_none());
    for out in ["part.out","copy.out"] {
        near(crate::common::at(&e,out),[1.,0.,0.],1e-9);
    }
    let still = seen(&e,"part.turn","part.turn");
    near(motion::evaluate(&e.sketch,still,0.7).unwrap().point([1.,2.,3.]),[1.,2.,3.],1e-12);
    let bad = build(&format!("{src}bad := point\nbad coincident(under: part.spin,t == 0deg) x\n"));
    assert!(bad.errors().any(|d| d.message.contains("private member")
        || d.message.contains("no such frame: `part.spin`")),"{:?}",bad.diags);
}

/// A joint is held by relations, and what holds it wrongly — or reads a frame that has none, or
/// something that is not a frame — is refused saying so; `motion(…)` itself is retired, refused
/// naming what each of its spellings is now.
#[test]
fn motions_reject_wrong_units_missing_references_and_retired_spellings() {
    for (tail,want) in [
        ("bad := frame\no coincident bad.axis\nbad.axis parallel std.front.normal\n\
          fix(ratio == 1,advance == 2mm) bad","has no advance"),
        ("bad := frame\nstd.Turn(bad,about: missing,ratio: 1)","missing"),
        ("bad := frame\nstd.Turn(bad,about: ax,ratio: 2mm)","ratio"),
        ("bad := frame\nax coincident bad.axis","states no ratio"),
        ("bad := point\nbad coincident(under: ax,t == 0deg) x","a motion is read off a frame"),
        ("still := frame\nbad := point\nbad coincident(under: still,t == 0deg) x","does not move"),
    ] {
        let e = build(&format!("{AXES}{tail}\n"));
        assert!(!e.ok() && e.errors().any(|d| d.message.contains(want)),"{tail}: {:?}",e.diags);
    }
    for (args,want) in [
        ("about: ax,ratio: 2,phase: 30deg","roll 0 is as drawn"),
        ("turn,relative_to: observer","seen_from: b"),
        ("along: other,advance: 2mm","std.Slide(f, along: l, advance: a)"),
        ("about: ax,ratio: 2","std.Turn(f, about: l, ratio: r)"),
        ("about: o,ratio: 2","std.Spin(f, about: o, on: P, ratio: r)"),
    ] {
        let (_,errors) = crate::common::parse(&format!("{AXES}bad := motion({args})\n"));
        assert!(errors.iter()
            .any(|d| d.message.contains("is retired (#383)") && d.message.contains(want)),
            "motion({args}): {:?}",errors.iter().map(|d| &d.message).collect::<Vec<_>>());
    }
    let (_,errors) = crate::common::parse("component Observe(m: motion) {\n}\n");
    assert!(errors.iter().any(|d| d.message.contains("`m: frame`")),"{:?}",
        errors.iter().map(|d| &d.message).collect::<Vec<_>>());
}

#[test]
fn motion_axes_use_world_geometry_and_follow_solved_point_edits() {
    let mut e = solved("unit mm
use std
in std.front {
  a := point
  b := point
  a distance(2mm,along: u) std.front
  a level(v) std.front
  b distance(2mm,along: u) std.front
  b distance(1mm,along: v) std.front
  ax := line(a,b)
}
turn := frame
std.Turn(turn,about: ax,ratio: 1)
");
    // a quarter turn on, at roll 90°
    let (m,quarter) = (joint(&e,"turn"),std::f64::consts::FRAC_PI_2);
    let snapshot = motion::Family::read(&e.sketch,m).unwrap();
    let pose = motion::evaluate(&e.sketch,m,quarter).unwrap();
    near(pose.point([3.,0.,0.]),[2.,1.,0.],1e-10);
    near(pose.point([2.,0.,5.]),[2.,0.,5.],1e-10);
    // Editing the solved points changes the next evaluation, without retaining a pose: one
    // along the front's x is one along the world's.
    for name in ["a","b"] {
        let i = e.map.ent_named(name).unwrap().i();
        let params = e.sketch.point_params(i);
        e.sketch.params[params[0] as usize].value += 1.;
    }
    near(snapshot.at(quarter).unwrap().point([3.,0.,0.]),[2.,1.,0.],1e-10);
    let after = motion::evaluate(&e.sketch,m,quarter).unwrap();
    near(after.point([4.,0.,0.]),[3.,1.,0.],1e-10);
    let a = e.map.ent_named("a").unwrap().i();
    let b = e.map.ent_named("b").unwrap().i();
    let xy = e.sketch.point_xy(a);
    let bp = e.sketch.point_params(b);
    e.sketch.params[bp[0] as usize].value = xy.0;
    e.sketch.params[bp[1] as usize].value = xy.1;
    assert!(motion::evaluate(&e.sketch,m,0.).unwrap_err().contains("nondegenerate"));
    assert!(program::solid_diagnostics(&e.sketch,&e.map).iter()
        .any(|d| d.message.contains("nondegenerate")));
}

/// A document reads one frame seen from another and no deeper, so the reader's limits on nested
/// relatives — 64 levels, and no cycle — are held over nodes made by hand.
#[test]
fn motion_dependency_depth_is_bounded_and_runtime_cycles_are_rejected() {
    let mut e = solved(&format!("{AXES}base := frame\nstd.Turn(base,about: ax,ratio: 1)\n"));
    let base = joint(&e,"base");
    // m0 reads m1 reads … m69 reads base, each seen from base: the recursion is 70 deep
    let first = e.sketch.motions.len();
    for i in 0..70 {
        let next = if i == 69 { base } else { first+i+1 };
        push_relative(&mut e.sketch,next,base);
    }
    assert!(motion::evaluate(&e.sketch,first,0.).unwrap_err().contains("64 levels"));
    assert!(motion::evaluate(&e.sketch,first+70-63,0.).is_ok());
    e.sketch.motions[base].def = MotionDef::Relative {source:base as u32,observer:base as u32};
    assert!(motion::evaluate(&e.sketch,base,0.).unwrap_err().contains("cycle"));
}

/// Read the other way — each node over the one before it, so every step is found already read —
/// the cache does not hide the depth.
#[test]
fn cached_motion_dependencies_do_not_bypass_the_depth_limit() {
    let mut e = solved(&format!("{AXES}base := frame\nstd.Turn(base,about: ax,ratio: 1)\n"));
    let base = joint(&e,"base");
    let mut prev = base;
    for _ in 0..70 {
        prev = push_relative(&mut e.sketch,prev,base);
    }
    let first = prev+1-70;
    assert!(motion::evaluate(&e.sketch,first+62,0.).is_ok());
    for i in 63..70 {
        assert!(motion::evaluate(&e.sketch,first+i,0.).unwrap_err().contains("64 levels"),"{i}");
    }
}

/// A screw advances along its axis by `advance` per turn; a translation moves
/// without turning. Poses, derivatives, interval bounds and the speed bound
/// are checked against closed forms.
#[test]
fn screws_and_translations_follow_their_closed_forms() {
    use gcs_core::interval::Interval as I;
    let e = solved(&format!("{AXES}\n\
        tap := frame\nstd.Turn(tap,about: ax,ratio: 2,advance: 3mm)\n\
        feed := frame\nstd.Slide(feed,along: other,advance: 10mm)\n{}",reading("tap","feed")));
    let tap = motion::Family::read(&e.sketch,joint(&e,"tap")).unwrap();
    let feed = motion::Family::read(&e.sketch,joint(&e,"feed")).unwrap();
    let relative = motion::Family::read(&e.sketch,seen(&e,"tap","feed")).unwrap();
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
        let turn = gcs_core::envelope::Motion::rotation(da,2.*t,2.).unwrap();
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
    let mut e = solved(&format!("{AXES}\n\
        tap := frame\nstd.Turn(tap,about: ax,ratio: 2,advance: 3mm)\n\
        feed := frame\nstd.Slide(feed,along: other,advance: 10mm)\n\
        turn := frame\nstd.Turn(turn,about: ax,ratio: 2)\n\
        observer := frame\nstd.Turn(observer,about: other,ratio: -3)\n{}{}",
        reading("tap","feed"),reading("turn","observer")));
    // and a relative of a relative, made by hand: a document reads one frame seen from another
    let (rolled,turn) = (seen(&e,"turn","observer"),joint(&e,"turn"));
    let nested = push_relative(&mut e.sketch,rolled,turn);
    let nodes = [joint(&e,"tap"),joint(&e,"feed"),joint(&e,"turn"),seen(&e,"tap","feed"),
        rolled,nested];
    for node in nodes {
        let family = motion::Family::read(&e.sketch,node).unwrap();
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

/// **A turn about a point is a rotation about the line through it square to its view** — how
/// a planar motion reads in space, so the generators of the plane (§6.15.1) and of space (§6.14)
/// are one family.  At roll 0.7 past 15° a point one unit along x from the centre has turned
/// ratio times that about it, in the page.
#[test]
fn a_turn_about_a_point_is_a_rotation_square_to_its_view() {
    let e = solved(&format!("{AXES}turn := frame\n\
        std.Spin(turn,about: a,on: std.front,ratio: 2)\n"));
    // the page stands in space as the front view: its points are lifted through its basis; read
    // at 0.7 on from 15°, where a turn twice the roll stands 30° on
    let page = gcs_core::plane::Basis::page();
    let roll = 0.7 + 15f64.to_radians();
    let got = motion::evaluate(&e.sketch,joint(&e,"turn"),roll).unwrap().point(page.lift(3.,0.));
    let ang = 2.0*0.7 + 30f64.to_radians();
    near(got,page.lift(2.+ang.cos(),ang.sin()),1e-12);
}

/// The Wankel's planetary motion, read in the top view (x right, y away): the rotor's centre rides
/// the eccentric at `e` about the shaft, and the rotor turns at a third of the shaft's angle. Two
/// turns about parallel axes say it — the rotor about where its centre starts, at −2/3, seen by an
/// observer turning the other way about the shaft — so the apex, drawn at `R` from the rotor's
/// centre, traces the epitrochoid e(cos 3t, sin 3t) + R(cos t, sin t) with t = θ/3, and the housing
/// seen from the rotor is the inverse motion.
#[test]
fn two_turns_about_parallel_axes_are_the_wankel_rotors_planetary_motion() {
    let (r,e) = (105.,15.);
    let src = format!("\
unit mm
use std
in std.front {{
shaft := line(std.origin, hint((0, 1)))
fix((0, 1)) shaft.p2
        c0 := point
c1 := point
c0 distance({e}mm, along: u) std.front
c0 level(v) std.front
c1 distance({e}mm, along: u) std.front
c1 distance(1mm, along: v) std.front
rotor_axis := line(c0, c1)
}}
counter := frame
std.Turn(counter, about: shaft, ratio: -1)
spin := frame
std.Turn(spin, about: rotor_axis, ratio: -2/3)
rotor := point
rotor coincident(under: spin, seen_from: counter, t == 0deg) c0
housing := point
housing coincident(under: counter, seen_from: spin, t == 0deg) c0
");
    let (mut p,errors) = crate::common::parse(&src);
    assert!(errors.is_empty(),"{errors:?}");
    assert!(gcs_core::modules::link(&mut p,&mut gcs_core::library::resolve).is_empty());
    let mut el = program::elaborate(&p);
    assert!(el.ok(),"{:?}",el.diags);
    assert!(solve::solve(&mut el.sketch,Default::default()).success);
    // the rotor is the spin seen from the housing, and the housing the counter seen from it
    let family = |m| motion::Family::read(&el.sketch,m).unwrap();
    let (rotor,housing) = (family(seen(&el,"spin","counter")),family(seen(&el,"counter","spin")));
    let apex = [r+e,0.,0.];
    for k in 0..=36 {
        let theta = (k as f64*30.).to_radians();
        let t = theta/3.;
        let pose = rotor.at(theta).unwrap();
        // the rotor's centre on its orbit, and the apex on the epitrochoid
        near(pose.point([e,0.,0.]),[e*theta.cos(),e*theta.sin(),0.],1e-9);
        near(pose.point(apex),[e*(3.*t).cos()+r*t.cos(),e*(3.*t).sin()+r*t.sin(),0.],1e-9);
        // turned through t: a point a unit along x from the centre lies along (cos t, sin t)
        let (c,q) = (pose.point([e,0.,0.]),pose.point([e+1.,0.,0.]));
        near([q[0]-c[0],q[1]-c[1],q[2]-c[2]],[t.cos(),t.sin(),0.],1e-9);
        // the housing seen from the rotor undoes it
        near(housing.at(theta).unwrap().point(pose.point([3.,-7.,2.])),[3.,-7.,2.],1e-9);
    }
    // one whole period: three turns of the shaft bring the rotor back
    near(rotor.at(3.*std::f64::consts::TAU).unwrap().point([40.,25.,0.]),[40.,25.,0.],1e-9);
}
