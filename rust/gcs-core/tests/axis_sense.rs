//! **Which way an axis runs** (#535, Solvent 0.173): `parallel`, and `coincident` between two
//! directions, take `sense: same|opposite`, whose row has the stated root as its only one; an
//! axis a plane is built on along a drawn line runs from the line's start toward its end; and a
//! joint's axis runs the way the line or axis it is held to does.  Held here from seeds turned
//! round, where the unsensed row would keep the seed's sense.
use gcs_core::constraints::CKind;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};

use crate::common::{dof, ent, read, solved, unit};

/// An axis's direction as it stands.
fn dir(sk: &Sketch, axis: usize) -> [f64; 3] {
    sk.axes[axis].d.map(|p| sk.params[p as usize].value)
}

/// The direction a drawn line runs, from its start toward its end, in space.
fn runs(sk: &Sketch, line: EntRef) -> [f64; 3] {
    let l = &sk.lines[line.i()];
    unit(sub(sk.world_point(l.p2 as usize), sk.world_point(l.p1 as usize)))
}

/// The axis turned nearly round — 170° about a direction square to it — so a solve has to bring
/// it back the long way; turned exactly round, a turn onto it has no direction to start in.
fn turn_back(sk: &mut Sketch, axis: usize) {
    let d = unit(dir(sk, axis));
    let side = unit(cross(d, if d[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] }));
    let (c, s) = (170f64.to_radians().cos(), 170f64.to_radians().sin());
    let back = [0, 1, 2].map(|k| c * d[k] + s * side[k]);
    for (k, q) in sk.axes[axis].d.into_iter().enumerate() {
        sk.params[q as usize].value = back[k];
    }
}

/// A line held in space from (1, 2, 3) to (4, 6, 3), and the statement over it.
fn held_line(stated: &str, seed: &str) -> String {
    format!("unit mm\nuse std\np := point hint((1, 2, 3))\nq := point hint((4, 6, 3))\n\
             fix((1, 2, 3)) p\nfix((4, 6, 3)) q\nl := line(p, q)\n\
             t := axis hint(dir: ({seed}))\n{stated}\n")
}

/// `parallel(sense: same)` holds the axis the way the line runs, `opposite` the other way, from a
/// seed on either side and from one exactly turned round; unsaid, the seed keeps its side.
#[test]
fn an_axis_runs_the_way_its_parallel_says_from_any_seed() {
    for seed in ["0.6, 0.8, 0.1", "-0.6, -0.8, 0.1", "-0.6, -0.8, 0", "0, 0.1, 1"] {
        for (word, sign) in [("same", 1.0), ("opposite", -1.0)] {
            for stated in [format!("t parallel(sense: {word}) l"),
                           format!("l parallel(sense: {word}) t")] {
                let e = read(&held_line(&stated, seed));
                let sk = solved(&e);
                let (d, w) = (unit(dir(&sk, ent(&e, "t").i())), runs(&sk, ent(&e, "l")));
                assert!(norm(sub(d, w.map(|x| sign * x))) < 1e-9, "{stated} from ({seed}): {d:?}");
            }
        }
    }
    let e = read(&held_line("t parallel l", "-0.6, -0.8, 0.1"));
    let sk = solved(&e);
    assert!(dot(dir(&sk, ent(&e, "t").i()), [0.6, 0.8, 0.0]) < 0.0, "unsaid, the seed's side");
}

/// The same in a plane: two lines drawn there run alike or apart as `sense:` says, `angle`'s
/// kernel at no turn or a half; and the statement swapped is the same statement.
#[test]
fn two_lines_in_a_plane_run_the_way_their_parallel_says() {
    for (seed, word, want) in [((-8.0, 1.0), "same", (10.0, 0.0)), ((8.0, 1.0), "opposite", (-10.0, 0.0)),
                               ((0.0, 9.0), "same", (10.0, 0.0)), ((0.0, -9.0), "opposite", (-10.0, 0.0))] {
        let e = read(&format!("unit mm\nuse std (horizontal)\nin std.front {{\n\
            a := point\nb := point hint((10, 0))\nc := point hint((0, 5))\n\
            d := point hint(({}, {}))\nfix((0, 0)) a\nfix((10, 0)) b\nfix((0, 5)) c\n\
            ab := line(a, b)\ncd := line(c, d)\nc distance(10) d\n\
            ab parallel(sense: {word}) cd\n}}\n", seed.0, seed.1));
        let mut sk = solved(&e);
        let at = sk.point_xy(ent(&e, "d").i());
        assert!((at.0 - want.0).hypot(at.1 - 5.0 - want.1) < 1e-7, "{word} from {seed:?}: {at:?}");
        assert_eq!(dof(&mut sk), 0);
        let rows = sk.user_constraints().iter().filter(|c| c.kind == CKind::Parallel).count();
        assert_eq!(rows, 1);
    }
}

/// A line on an axis and two axes on one line take the sense too: the line's ends and the second
/// axis's place as unsensed, its direction the way it says.
#[test]
fn an_axis_coincident_a_line_or_an_axis_runs_the_way_it_says() {
    for (word, sign) in [("same", 1.0), ("opposite", -1.0)] {
        let e = read(&held_line(&format!("l coincident(sense: {word}) t"), "-0.6, -0.8, 0.2"));
        let mut sk = solved(&e);
        let t = ent(&e, "t").i();
        let d = unit(dir(&sk, t));
        assert!(norm(sub(d, [0.6 * sign, 0.8 * sign, 0.0])) < 1e-9, "{word}: {d:?}");
        assert_eq!(dof(&mut sk), 0);
        let src = format!("{}s := axis hint(dir: (0.5, 0.7, -0.4))\ns coincident(sense: {word}) t\n",
            held_line("l coincident(sense: same) t", "0.6, 0.8, 0"));
        let e = read(&src);
        let mut sk = solved(&e);
        let d = unit(dir(&sk, ent(&e, "s").i()));
        assert!(norm(sub(d, [0.6 * sign, 0.8 * sign, 0.0])) < 1e-9, "{word}: {d:?}");
        assert_eq!(dof(&mut sk), 0);
    }
}

/// **A plane built on drawn lines runs along them** (the rule): its `u` and `v` from each line's
/// start toward its end, which the rows say, so a solve that starts from the axes turned round
/// comes back to the lines — where through 0.172 only the seed said it, and the plane stayed
/// turned round.
#[test]
fn a_plane_on_two_lines_runs_along_them_whatever_its_start() {
    let e = read("unit mm\nuse std\na := point hint((0, 0, 0))\nb := point hint((10, 0, 2))\n\
                  c := point hint((0, 3, 10))\nfix((0, 0, 0)) a\nfix((10, 0, 2)) b\nfix((0, 3, 10)) c\n\
                  across := line(a, b)\nup := line(a, c)\nP := plane(u: across, v: up)\n");
    let mut sk = e.sketch.clone();
    let p = ent(&e, "P").i();
    let (u, v) = (sk.frames[p].u as usize, sk.frames[p].v as usize);
    let sensed = sk.constraints.iter()
        .filter(|c| c.intrinsic && c.kind == CKind::Parallel3 && c.side() == Some(1.0))
        .count();
    assert_eq!(sensed, 2, "each axis held along its line, the line's way");
    turn_back(&mut sk, u);
    turn_back(&mut sk, v);
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let (across, up) = (runs(&sk, ent(&e, "across")), runs(&sk, ent(&e, "up")));
    assert!(norm(sub(unit(dir(&sk, u)), across)) < 1e-9, "u runs {:?}", dir(&sk, u));
    assert!(norm(sub(unit(dir(&sk, v)), up)) < 1e-9, "v runs {:?}", dir(&sk, v));
    assert_eq!(dof(&mut sk), 0);
}

/// **A joint's axis runs the way its line does** (the rule, in `std`'s joint words): a frame
/// turning, screwing or sliding along a line has its axis from the line's start toward its end,
/// and one spinning about a point of a view, out of the view along its normal — so a positive
/// ratio turns the right-handed way about the line as it is drawn, from any start.
#[test]
fn a_joints_axis_runs_the_way_its_line_does_whatever_its_start() {
    for word in ["turns(1)", "screws(2mm)", "slides(3mm)"] {
        let name = word.split('(').next().unwrap();
        let e = read(&format!("unit mm\nuse std ({name})\np := point hint((1, 2, 3))\n\
            q := point hint((1, 2, 13))\nfix((1, 2, 3)) p\nfix((1, 2, 13)) q\nl := line(p, q)\n\
            f := frame\nf {word} l\n"));
        let mut sk = e.sketch.clone();
        let axis = sk.frames[ent(&e, "f").i()].joint.as_ref().expect("a joint").axis as usize;
        turn_back(&mut sk, axis);
        assert!(solve(&mut sk, SolveOpts::default()).success, "{word}");
        assert!(norm(sub(unit(dir(&sk, axis)), [0.0, 0.0, 1.0])) < 1e-9,
                "{word}: the joint's axis runs {:?}", dir(&sk, axis));
    }
    let e = read("unit mm\nuse std (spins)\nin std.front {\n  o := point\n  fix((5, 5)) o\n}\n\
                  f := frame\nf spins(1) o\n");
    let mut sk = e.sketch.clone();
    let axis = sk.frames[ent(&e, "f").i()].joint.as_ref().expect("a joint").axis as usize;
    turn_back(&mut sk, axis);
    assert!(solve(&mut sk, SolveOpts::default()).success);
    // the front's normal: out of the front, toward the eye that sees it
    let front = gcs_core::plane::Basis::page();
    let n = cross(front.u, front.v);
    assert!(norm(sub(unit(dir(&sk, axis)), n)) < 1e-9, "spins about {:?}", dir(&sk, axis));
}

/// A sense written on a relation that has none to say is refused at its key, and a word outside
/// `same` and `opposite` is too.
#[test]
fn a_sense_where_there_is_none_to_say_is_refused() {
    crate::common::refused(&held_line("t parallel(sense: ccw) l", "0.6, 0.8, 0"), "E040",
                           "`same` or `opposite`", "sense");
    crate::common::refused(&held_line("t perpendicular(sense: same) l", "0, 0, 1"), "E040",
                           "", "sense");
}

/// A sense prints back as written and reads back to the same pose: in a plane, and in space,
/// where the statement lifted names its axis by the direction it was given.
#[test]
fn a_sense_prints_back_as_written() {
    use gcs_core::program::to_program;
    for (src, said) in [
        ("unit mm\nuse std\nin std.front {\na := point\nb := point hint((10, 0))\n\
          c := point hint((0, 5))\nd := point hint((8, 6))\nfix((0, 0)) a\nfix((10, 0)) b\n\
          fix((0, 5)) c\nab := line(a, b)\ncd := line(c, d)\nc distance(10) d\n\
          ab parallel(sense: opposite) cd\n}\n", "parallel(sense: opposite)"),
        (&*held_line("l coincident(sense: opposite) t", "0.6, 0.8, 0"), "coincident(sense: opposite)"),
    ] {
        let e = read(src);
        let sk = solved(&e);
        let mut lifted = to_program(&e.sketch);
        let text = gcs_core::syntax::render_flat(&mut lifted).unwrap().to_string();
        assert!(text.contains(said), "{text}");
        let again = read(&text);
        let back = solved(&again);
        let pose = |sk: &Sketch| (0..sk.points.len()).map(|p| sk.world_point(p)).collect::<Vec<_>>();
        for (x, y) in pose(&sk).iter().zip(pose(&back)) {
            assert!(norm(sub(*x, y)) < 1e-7, "{said}: {x:?} read back at {y:?}\n{text}");
        }
    }
}
