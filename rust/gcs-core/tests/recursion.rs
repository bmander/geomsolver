//! **A component may instantiate itself** (§8): a call back into a component still being
//! expanded is walked like any other, and a `when` decided over the numbers it was given ends it
//! (§12.9).  Each level is an instance like any other, named under the one that made it
//! (`d.s.s.m`).  What never ends is refused (E003) at the call that closes the cycle: one given
//! the numbers its own expansion was given (every choice comes out the same again), or one
//! nesting past the flattener's cap.

use crate::common::{read, refused};
use gcs_core::solve::{solve, SolveOpts};

/// A segment halved `depth` times toward its start, one level per instance.
fn halving(depth: usize) -> String {
    format!(
        "use std\nuse std.place (horizontal)\n\
         component Half(a: point, b: point, len: Length, n: Int) {{\n\
           m := point hint((a.x + (b.x - a.x) / 2, a.y + (b.y - a.y) / 2))\n\
           a horizontal m\n\
           a distance(len / 2) m\n\
           when n > 1 {{\n\
             s := Half(a, m, len: len / 2, n: n - 1)\n\
           }}\n\
         }}\n\
         in std.front {{\n\
           a := point hint((0, 0))\n\
           b := point hint((10, 0))\n\
           fix((0, 0)) a\n\
           fix((10, 0)) b\n\
           d := Half(a, b, len: 10, n: {depth})\n\
         }}\n"
    )
}

/// **Each level is an instance, named under the last**, and the `when` ends it where the count
/// runs out: three levels make `d.m`, `d.s.m`, `d.s.s.m` and no fourth.
#[test]
fn a_component_recurses_until_its_when_ends_it() {
    let e = read(&halving(3));
    for name in ["d.m", "d.s.m", "d.s.s.m"] {
        assert!(e.map.ent_named(name).is_some(), "`{name}` is made");
    }
    assert!(e.map.ent_named("d.s.s.s.m").is_none(), "the `when` ends it at three");
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    for (name, x) in [("d.m", 5.0), ("d.s.m", 2.5), ("d.s.s.m", 1.25)] {
        let p = sk.point_xy(e.map.ent_named(name).unwrap().i());
        assert!((p.0 - x).abs() < 1e-9 && p.1.abs() < 1e-9, "{name} at {p:?}");
    }
}

/// Penrose's P3 by Robinson triangles, two components each calling the other
/// (`examples/penrose.sv`, the app's example), its ten outer points stated by `sun` and called
/// `v[…]` by the rest.
fn penrose_from(depth: usize, sun: &str, v: &str) -> String {
    format!(
        "use std\nuse std.place (horizontal)\nuse std.shapes\n\
         param depth := {depth}\n\
         param l := 100\n\
         component Acute(a: point, b: point, c: point, l: Length, n: Int) {{\n\
           when n == 0 {{\n\
             line(a, b)\n\
             line(a, c)\n\
           }}\n\
           when n > 0 {{\n\
             phi := (1 + sqrt(5)) / 2\n\
             p := point hint(at: a, toward: b, by: 1 / phi)\n\
             a distance(l / phi) p\n\
             c distance(l / phi) p\n\
             x := Acute(c, p, b, l: l / phi, n: n - 1)\n\
             y := Obtuse(p, c, a, s: l / phi, n: n - 1)\n\
           }}\n\
         }}\n\
         component Obtuse(a: point, b: point, c: point, s: Length, n: Int) {{\n\
           when n == 0 {{\n\
             line(a, b)\n\
             line(a, c)\n\
           }}\n\
           when n > 0 {{\n\
             phi := (1 + sqrt(5)) / 2\n\
             r := point hint(at: b, toward: c, by: 1 / phi)\n\
             q := point hint(at: b, toward: a, by: 1 / phi)\n\
             b distance(s) r\n\
             a distance(s / phi) r\n\
             r distance(s / phi) q\n\
             b distance(s / phi) q\n\
             x := Obtuse(r, c, a, s: s / phi, n: n - 1)\n\
             y := Obtuse(q, r, b, s: s / phi, n: n - 1)\n\
             z := Acute(r, q, a, l: s / phi, n: n - 1)\n\
           }}\n\
         }}\n\
         in std.front {{\n\
           o := point hint((0, 0))\n\
           fix((0, 0)) o\n\
           {sun}\n\
           repeat 5 as k {{\n\
             t := Acute(o, {v}[2 * k + 1], {v}[2 * k], l: l, n: depth)\n\
           }}\n\
           repeat 4 as k {{\n\
             m := Acute(o, {v}[2 * k + 1], {v}[2 * k + 2], l: l, n: depth)\n\
           }}\n\
           last := Acute(o, {v}[9], {v}[0], l: l, n: depth)\n\
         }}\n"
    )
}

type P = (f64, f64);

/// The same subdivision worked out directly (Robinson's rule, as Preshing writes it): the leaf
/// triangles, apex first.
fn subdivided(depth: usize) -> Vec<(bool, P, P, P)> {
    let phi = (1.0 + 5f64.sqrt()) / 2.0;
    let at = |p: P, q: P, f: f64| (p.0 + (q.0 - p.0) * f, p.1 + (q.1 - p.1) * f);
    let v = |j: usize| {
        let t = (36.0 * j as f64).to_radians();
        (100.0 * t.cos(), 100.0 * t.sin())
    };
    let o = (0.0, 0.0);
    let mut tris: Vec<(bool, P, P, P)> = (0..10)
        .map(|j| match j % 2 {
            0 => (true, o, v(j + 1), v(j)),
            _ => (true, o, v(j), v((j + 1) % 10)),
        })
        .collect();
    for _ in 0..depth {
        let mut next = Vec::new();
        for (acute, a, b, c) in tris {
            if acute {
                let p = at(a, b, 1.0 / phi);
                next.extend([(true, c, p, b), (false, p, c, a)]);
            } else {
                let q = at(b, a, 1.0 / phi);
                let r = at(b, c, 1.0 / phi);
                next.extend([(false, r, c, a), (false, q, r, b), (true, r, q, a)]);
            }
        }
        tris = next;
    }
    tris
}

/// **Mutual recursion draws Penrose's tiling exactly**: every leaf the solve places is a
/// triangle of the subdivision worked out directly, and every one of those is drawn — the
/// branch each point is seeded on included, which a count of lengths alone would not see.
#[test]
fn mutually_recursive_components_subdivide_a_penrose_sun() {
    tiles(include_str!("../../examples/penrose.sv"), 3, 0);
}

/// **The same with the sun a `ring`** (#225): `std.shapes.Polygon`'s ten corners, nine of them
/// turned copies of the first, each subdivided triangle seeding its points from them — so each seed
/// must read a copy where its turn puts it, or the deepest levels start on the wrong branch.
#[test]
fn a_penrose_sun_drawn_as_a_ring_subdivides_alike() {
    let sun = "q := point hint((l, 0))\n\
               ref := line(o, q)\n\
               horizontal ref\n\
               o distance(l) q\n\
               sun := std.shapes.Polygon(o, ref, n: 10, r: l, phase: 0deg)";
    // `ref` and the polygon's ten sides
    tiles(&penrose_from(3, sun, "sun.v"), 3, 11);
}

/// Every leaf the solve places is a triangle of `subdivided(depth)`, and every one is drawn;
/// `sun` lines are drawn before the leaves.
fn tiles(src: &str, depth: usize, sun: usize) {
    let e = read(src);
    let mut sk = e.sketch;
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{r:?}");
    let want = subdivided(depth);
    assert_eq!(want.len(), 130, "10 → 20 → 50 → 130 half rhombs");
    // each leaf drew its two legs, apex first, one after the other, past the sun's
    assert_eq!(sk.lines.len(), sun + 2 * want.len());
    let legs = &sk.lines[sun..];
    let drawn: Vec<(P, P, P)> = legs.chunks(2)
        .map(|legs| {
            assert_eq!(legs[0].p1, legs[1].p1, "a leaf's legs share its apex");
            let xy = |i: u32| sk.point_xy(i as usize);
            (xy(legs[0].p1), xy(legs[0].p2), xy(legs[1].p2))
        })
        .collect();
    let near = |p: P, q: P| (p.0 - q.0).hypot(p.1 - q.1) < 1e-6;
    for (_, a, b, c) in &want {
        assert!(
            drawn.iter().any(|(x, y, z)| near(*a, *x) && near(*b, *y) && near(*c, *z)),
            "no leaf drawn at {a:?} {b:?} {c:?}"
        );
    }
    for (x, y, z) in &drawn {
        assert!(
            want.iter().any(|(_, a, b, c)| near(*a, *x) && near(*b, *y) && near(*c, *z)),
            "a leaf drawn at {x:?} {y:?} {z:?} the subdivision has not"
        );
    }
}

/// **What never ends is refused, at the call that closes the cycle**: the numbers it was given
/// again (directly, or none at all), and a count running the wrong way past the cap.
#[test]
fn a_recursion_nothing_ends_is_refused() {
    let doc = |call: &str| {
        format!(
            "use std\ncomponent F(a: point, n: Int) {{\n  m := point\n  a level(v) m\n  \
             a distance(1, along: u) m\n  when n > 0 {{\n    \
             f := {call}\n  }}\n}}\nin std.front {{\n  o := point\n  x := F(o, n: 2)\n}}\n"
        )
    };
    refused(&doc("F(m, n: n)"), "E003", "`F` instantiates itself with the numbers it was given",
            "f := F(m, n: n)");
    refused(&doc("F(m, n: n + 1)"), "E003", "`F` instantiates itself more than 32 deep",
            "f := F(m, n: n + 1)");
    refused("use std\ncomponent G(a: point) {\n  m := point\n  g := G(m)\n}\n\
             in std.front {\n  o := point\n  x := G(o)\n}\n",
            "E003", "`G` instantiates itself with the numbers it was given", "g := G(m)");
    // two components each calling the other, said once and not as `A`, `A`, `A`, … through it
    refused("use std\ncomponent A(p: point, n: Int) {\n  when n > 0 {\n    b := B(p, n: n + 1)\n  }\n}\n\
             component B(p: point, n: Int) {\n  a := A(p, n: n)\n}\n\
             in std.front {\n  o := point\n  x := A(o, n: 1)\n}\n",
            "E003", "`B` instantiates itself more than 32 deep: its", "b := B(p, n: n + 1)");
    // and a count that runs out is no refusal at all
    read(&doc("F(m, n: n - 1)"));
}

/// **A recursion branching at every level is stopped by its count of instances**: a body of
/// nothing but calls emits no statement, so the statements' cap never reaches it, and three
/// calls a level twelve deep are half a million instances.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, ~5 s")]
fn a_recursion_branching_past_the_cap_is_refused() {
    refused("use std\ncomponent F(a: point, n: Int) {\n  when n > 0 {\n    x := F(a, n: n - 1)\n    \
             y := F(a, n: n - 1)\n    z := F(a, n: n - 1)\n  }\n}\n\
             in std.front {\n  o := point\n  f := F(o, n: 12)\n}\n",
            "E103", "instances once expanded", "x := F(a, n: n - 1)");
}
