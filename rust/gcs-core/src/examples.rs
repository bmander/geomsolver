//! Reference sketches used by the tests, the benchmarks and the app's case library.

use crate::constraints::{Arg, CKind, Constraint};
use crate::model::{EntRef, Sketch};
use crate::rng::Rng;

fn add(sk: &mut Sketch, c: Constraint) {
    sk.add(c);
}

fn dist(sk: &Sketch, a: usize, b: usize) -> f64 {
    let (ax, ay) = sk.point_xy(a);
    let (bx, by) = sk.point_xy(b);
    (ax - bx).hypot(ay - by)
}

fn tangent_arc_line(arc: usize, line: usize, at: &str) -> Constraint {
    Constraint::new(
        CKind::TangentArcLine,
        vec![Arg::Ent(EntRef::arc(arc)), Arg::Ent(EntRef::line(line)), Arg::Str(at.to_string())],
    )
}

fn equal_radius(a: EntRef, b: EntRef) -> Constraint {
    Constraint::new(CKind::EqualRadius, vec![Arg::Ent(a), Arg::Ent(b)])
}

/// Rectangle w x h with four equal fillets of radius r.  Fully constrained (0 DOF).
pub fn rect_fillets(w: f64, h: f64, r: f64, jitter: f64) -> Sketch {
    let mut sk = Sketch::new();
    let mut rng = Rng::new(0);
    let p = |sk: &mut Sketch, rng: &mut Rng, x: f64, y: f64, name: &str| {
        let jx = rng.uniform(-jitter, jitter);
        let jy = rng.uniform(-jitter, jitter);
        sk.point(x + jx, y + jy, false, name)
    };
    let b1 = p(&mut sk, &mut rng, r, 0.0, "b1");
    let b2 = p(&mut sk, &mut rng, w - r, 0.0, "b2");
    let bottom = sk.line(b1, b2);
    let r1 = p(&mut sk, &mut rng, w, r, "r1");
    let r2 = p(&mut sk, &mut rng, w, h - r, "r2");
    let right = sk.line(r1, r2);
    let t1 = p(&mut sk, &mut rng, w - r, h, "t1");
    let t2 = p(&mut sk, &mut rng, r, h, "t2");
    let top = sk.line(t1, t2);
    let l1 = p(&mut sk, &mut rng, 0.0, h - r, "l1");
    let l2 = p(&mut sk, &mut rng, 0.0, r, "l2");
    let left = sk.line(l1, l2);
    // arcs share endpoints with the lines (CCW start -> end)
    let c_br = p(&mut sk, &mut rng, w - r, r, "c_br");
    let a_br = sk.arc(c_br, b2, r1, "a_br");
    let c_tr = p(&mut sk, &mut rng, w - r, h - r, "c_tr");
    let a_tr = sk.arc(c_tr, r2, t1, "a_tr");
    let c_tl = p(&mut sk, &mut rng, r, h - r, "c_tl");
    let a_tl = sk.arc(c_tl, t2, l1, "a_tl");
    let c_bl = p(&mut sk, &mut rng, r, r, "c_bl");
    let a_bl = sk.arc(c_bl, l2, b1, "a_bl");

    for ln in [bottom, top] {
        add(&mut sk, Constraint::one_line(CKind::Horizontal, EntRef::line(ln)));
    }
    for ln in [left, right] {
        add(&mut sk, Constraint::one_line(CKind::Vertical, EntRef::line(ln)));
    }
    for (arc, l_in, l_out) in
        [(a_br, bottom, right), (a_tr, right, top), (a_tl, top, left), (a_bl, left, bottom)]
    {
        add(&mut sk, tangent_arc_line(arc, l_in, "start"));
        add(&mut sk, tangent_arc_line(arc, l_out, "end"));
    }
    for other in [a_tr, a_tl, a_bl] {
        add(&mut sk, equal_radius(EntRef::arc(a_br), EntRef::arc(other)));
    }
    add(&mut sk, Constraint::radius(EntRef::arc(a_bl), r));
    add(&mut sk, Constraint::distance(EntRef::point(b1), EntRef::point(b2), w - 2.0 * r));
    add(&mut sk, Constraint::distance(EntRef::point(l1), EntRef::point(l2), h - 2.0 * r));
    let centre = sk.arcs[a_bl].center as usize;
    sk.fix_point(centre, true);
    sk
}

/// Obround slot with two concentric holes.  Fully constrained (0 DOF).
pub fn slotted_link(length: f64, r: f64, hole_r: f64) -> Sketch {
    let mut sk = Sketch::new();
    let c1 = sk.point(0.0, 0.0, false, "c1");
    let c2 = sk.point(length, 0.0, false, "c2");
    let t1 = sk.point(0.0, r, false, "t1");
    let t2 = sk.point(length, r, false, "t2");
    let top = sk.line(t1, t2);
    let b1 = sk.point(length, -r, false, "b1");
    let b2 = sk.point(0.0, -r, false, "b2");
    let bottom = sk.line(b1, b2);
    let a_right = sk.arc(c2, b1, t2, "a_r");
    let a_left = sk.arc(c1, t1, b2, "a_l");
    let h1 = sk.circle(c1, hole_r, "h1");
    let h2 = sk.circle(c2, hole_r, "h2");
    add(&mut sk, tangent_arc_line(a_right, bottom, "start"));
    add(&mut sk, tangent_arc_line(a_right, top, "end"));
    add(&mut sk, tangent_arc_line(a_left, top, "start"));
    add(&mut sk, tangent_arc_line(a_left, bottom, "end"));
    add(&mut sk, equal_radius(EntRef::arc(a_left), EntRef::arc(a_right)));
    add(&mut sk, Constraint::radius(EntRef::arc(a_left), r));
    add(&mut sk, Constraint::radius(EntRef::circle(h1), hole_r));
    add(&mut sk, Constraint::radius(EntRef::circle(h2), hole_r));
    add(&mut sk, Constraint::distance(EntRef::point(c1), EntRef::point(c2), length));
    add(&mut sk, Constraint::one_line(CKind::Horizontal, EntRef::line(top)));
    sk.fix_point(c1, true);
    sk
}

/// Warren-style truss: bays+1 bottom nodes, bays top nodes, ~4*bays members.  With dims every
/// member gets a length constraint → rigid, 0 DOF after fixing the first node and making the
/// first chord horizontal.
pub fn truss(bays: usize, span: f64, height: f64, dims: bool) -> Sketch {
    let mut sk = Sketch::new();
    let bot: Vec<usize> =
        (0..=bays).map(|i| sk.point(i as f64 * span, 0.0, false, &format!("b{i}"))).collect();
    let top: Vec<usize> = (0..bays)
        .map(|i| sk.point((i as f64 + 0.5) * span, height, false, &format!("t{i}")))
        .collect();
    let mut members = Vec::new();
    for i in 0..bays {
        members.push(sk.line(bot[i], bot[i + 1]));
        members.push(sk.line(bot[i], top[i]));
        members.push(sk.line(top[i], bot[i + 1]));
        if i + 1 < bays {
            members.push(sk.line(top[i], top[i + 1]));
        }
    }
    if dims {
        for &m in &members {
            let (p1, p2) = (sk.lines[m].p1 as usize, sk.lines[m].p2 as usize);
            let d = dist(&sk, p1, p2);
            add(&mut sk, Constraint::distance(EntRef::point(p1), EntRef::point(p2), d));
        }
    }
    add(&mut sk, Constraint::one_line(CKind::Horizontal, EntRef::line(members[0])));
    sk.fix_point(bot[0], true);
    sk
}

/// Under-constrained: a closed n-gon of equal-length edges via Coincident joints.  The
/// EqualLength cycle is deliberately closed, so one equation is redundant-but-consistent.
pub fn polygon_chain(n: usize, radius: f64) -> Sketch {
    let mut sk = Sketch::new();
    let tau = 2.0 * std::f64::consts::PI;
    let lines: Vec<usize> = (0..n)
        .map(|i| {
            let a0 = tau * i as f64 / n as f64;
            let a1 = tau * (i + 1) as f64 / n as f64;
            sk.line_xy(
                radius * a0.cos(),
                radius * a0.sin(),
                radius * a1.cos(),
                radius * a1.sin(),
                &format!("e{i}"),
            )
        })
        .collect();
    for i in 0..n {
        let a = sk.lines[lines[i]].p2 as usize;
        let b = sk.lines[lines[(i + 1) % n]].p1 as usize;
        add(&mut sk, Constraint::coincident(EntRef::point(a), EntRef::point(b)));
        add(
            &mut sk,
            Constraint::two_line(
                CKind::EqualLength,
                EntRef::line(lines[i]),
                EntRef::line(lines[(i + 1) % n]),
            ),
        );
    }
    let first = sk.lines[lines[0]].p1 as usize;
    sk.fix_point(first, true);
    sk
}

/// Random Laman graph on n >= 2 vertices by Henneberg I (add a vertex and 2 edges) and II
/// (subdivide an edge and connect to a third vertex) moves — minimally rigid by construction.
pub fn henneberg_edges(n: usize, rng: &mut Rng) -> Vec<(usize, usize)> {
    let mut edges = vec![(0usize, 1usize)];
    for v in 2..n {
        if v == 2 || rng.next() < 0.6 {
            // type I
            let s = rng.sample(v, 2);
            edges.push((v, s[0]));
            edges.push((v, s[1]));
        } else {
            // type II
            let i = rng.int(edges.len());
            let (a, b) = edges.remove(i);
            let cands: Vec<usize> = (0..v).filter(|&w| w != a && w != b).collect();
            let c = rng.choice(&cands);
            edges.push((v, a));
            edges.push((v, b));
            edges.push((v, c));
        }
    }
    edges
}

/// Random minimally rigid framework with a horizontal member and a fixed node.
pub fn laman(n: usize, seed: u32, ground: bool) -> Sketch {
    let mut rng = Rng::new(seed);
    let mut sk = Sketch::new();
    let pts: Vec<usize> = (0..n)
        .map(|i| {
            let x = rng.uniform(0.0, 60.0);
            let y = rng.uniform(0.0, 60.0);
            sk.point(x, y, false, &format!("n{i}"))
        })
        .collect();
    for (a, b) in henneberg_edges(n, &mut rng) {
        let d = dist(&sk, pts[a], pts[b]);
        add(&mut sk, Constraint::distance(EntRef::point(pts[a]), EntRef::point(pts[b]), d));
    }
    if ground {
        sk.fix_point(pts[0], true);
        let l = sk.line(pts[0], pts[1]);
        add(&mut sk, Constraint::one_line(CKind::Horizontal, EntRef::line(l)));
    }
    sk
}

/// K3,3 bar framework: minimally rigid but triangle-free — no pair/triple cluster merge applies,
/// so the decomposition must isolate it as one core.
pub fn k33(seed: u32) -> Sketch {
    let mut rng = Rng::new(seed);
    let mut sk = Sketch::new();
    let pts: Vec<usize> = (0..6)
        .map(|i| {
            let x = rng.uniform(0.0, 40.0);
            let y = rng.uniform(0.0, 40.0);
            sk.point(x, y, false, &format!("k{i}"))
        })
        .collect();
    sk.fix_point(pts[0], true);
    for a in 0..3 {
        for b in 3..6 {
            let d = dist(&sk, pts[a], pts[b]);
            add(&mut sk, Constraint::distance(EntRef::point(pts[a]), EntRef::point(pts[b]), d));
        }
    }
    let l = sk.line(pts[0], pts[3]);
    add(&mut sk, Constraint::one_line(CKind::Horizontal, EntRef::line(l)));
    sk
}

/// Fillet rectangle with a second, contradicting width dimension (80 vs 50).
pub fn rect_fillets_conflict() -> Sketch {
    let mut sk = rect_fillets(100.0, 60.0, 10.0, 0.0);
    let l = &sk.lines[0];
    let (p1, p2) = (l.p1 as usize, l.p2 as usize);
    add(&mut sk, Constraint::distance(EntRef::point(p1), EntRef::point(p2), 50.0));
    sk
}

/// Fillet rectangle without its width dimension: the right side slides (1 DOF).
pub fn rect_fillets_under() -> Sketch {
    let mut sk = rect_fillets(100.0, 60.0, 10.0, 0.0);
    if let Some(c) = sk
        .constraints
        .iter()
        .find(|c| c.kind == CKind::Distance && (c.args[2].num() - 80.0).abs() < 1e-12)
    {
        let id = c.id;
        sk.remove(id);
    }
    sk
}

/// Truss with an extra, consistent member: structurally over-constrained but satisfiable.
pub fn truss_redundant() -> Sketch {
    let mut sk = truss(6, 20.0, 15.0, true);
    let d = dist(&sk, 0, 2);
    add(&mut sk, Constraint::distance(EntRef::point(0), EntRef::point(2), d));
    sk
}

/// Truss with an impossible member length (999 between nearby nodes).
pub fn truss_conflict() -> Sketch {
    let mut sk = truss(6, 20.0, 15.0, true);
    add(&mut sk, Constraint::distance(EntRef::point(0), EntRef::point(3), 999.0));
    sk
}

/// Rigid truss with nothing fixed: a free rigid body (3 DOF) — drag it around.
pub fn truss_floating(bays: usize) -> Sketch {
    let mut sk = truss(bays, 20.0, 15.0, true);
    for p in sk.params.iter_mut() {
        p.fixed = false;
    }
    sk.constraints.retain(|c| c.kind != CKind::Horizontal);
    sk
}

/// Structurally fine, geometrically impossible: sides 10, 1, 1 (the triangle inequality).
pub fn impossible_triangle() -> Sketch {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 0.0, false, "b");
    let c = sk.point(5.0, 5.0, false, "c");
    add(&mut sk, Constraint::distance(EntRef::point(a), EntRef::point(b), 10.0));
    add(&mut sk, Constraint::distance(EntRef::point(b), EntRef::point(c), 1.0));
    add(&mut sk, Constraint::distance(EntRef::point(a), EntRef::point(c), 1.0));
    let l = sk.line(a, b);
    add(&mut sk, Constraint::one_line(CKind::Horizontal, EntRef::line(l)));
    sk
}

/// Fixed triangle, three altitudes and a point on all three: structurally the third incidence
/// looks independent, but the altitudes concur — a theorem-type dependency only the witness
/// configuration method sees.
pub fn altitudes() -> Sketch {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "A");
    let b = sk.point(40.0, 0.0, true, "B");
    let c = sk.point(15.0, 30.0, true, "C");
    let ab = sk.line(a, b);
    let bc = sk.line(b, c);
    let ca = sk.line(c, a);
    let qa = sk.point(15.0, 5.0, false, "QA");
    let qb = sk.point(20.0, 10.0, false, "QB");
    let qc = sk.point(15.0, -5.0, false, "QC");
    let alt_a = sk.line(a, qa);
    let alt_b = sk.line(b, qb);
    let alt_c = sk.line(c, qc);
    for (u, v) in [(alt_a, bc), (alt_b, ca), (alt_c, ab)] {
        add(
            &mut sk,
            Constraint::two_line(CKind::Perpendicular, EntRef::line(u), EntRef::line(v)),
        );
    }
    let p = sk.point(15.0, 8.0, false, "P");
    for l in [alt_a, alt_b, alt_c] {
        add(
            &mut sk,
            Constraint::new(
                CKind::PointOnLine,
                vec![Arg::Ent(EntRef::point(p)), Arg::Ent(EntRef::line(l))],
            ),
        );
    }
    sk
}

/// Parallel / perpendicular / vertical lines with a few distances — exercises direction classes.
pub fn parallels() -> Sketch {
    let mut sk = Sketch::new();
    let o = sk.point(0.0, 0.0, true, "o");
    let e = sk.point(40.0, 0.0, true, "e");
    let base = sk.line(o, e);
    let a = sk.point(0.0, 15.0, false, "a");
    let b = sk.point(40.0, 15.0, false, "b");
    let l2 = sk.line(a, b);
    let c = sk.point(10.0, 15.0, false, "c");
    let d = sk.point(10.0, 35.0, false, "d");
    let l3 = sk.line(c, d);
    let f = sk.point(10.0, 35.0, false, "f");
    let g = sk.point(30.0, 30.0, false, "g");
    let l4 = sk.line(f, g);
    add(&mut sk, Constraint::two_line(CKind::Parallel, EntRef::line(base), EntRef::line(l2)));
    add(&mut sk, Constraint::distance(EntRef::point(o), EntRef::point(a), 15.0));
    add(&mut sk, Constraint::one_line(CKind::Vertical, EntRef::line(l3)));
    add(&mut sk, Constraint::coincident(EntRef::point(c), EntRef::point(a)));
    add(&mut sk, Constraint::distance(EntRef::point(c), EntRef::point(d), 20.0));
    add(&mut sk, Constraint::distance(EntRef::point(a), EntRef::point(b), 40.0));
    add(
        &mut sk,
        Constraint::two_line(CKind::Perpendicular, EntRef::line(l3), EntRef::line(l4)),
    );
    add(&mut sk, Constraint::coincident(EntRef::point(f), EntRef::point(d)));
    add(&mut sk, Constraint::distance(EntRef::point(f), EntRef::point(g), 20.0));
    sk
}

fn expr_distance(p: usize, q: usize, text: &str) -> Constraint {
    Constraint::new(
        CKind::Distance,
        vec![
            Arg::Ent(EntRef::point(p)),
            Arg::Ent(EntRef::point(q)),
            Arg::Expr(crate::expr::Expr::new(text, 0.0)),
        ],
    )
}

/// The graphical proof of the Pythagorean theorem, drawn with expressions.
///
/// A square of side `a + b` holds four copies of the right triangle with legs `a` and `b`, one
/// in each corner, turned a quarter each time; what they leave in the middle is a square on
/// their hypotenuses.  So (a + b)² = 4 · ab/2 + c², which is c² = a² + b².  The sketch states
/// `a` and `b` once, as named dimensions on the first triangle, and every other leg reads the
/// name; the inner square's side is dimensioned `c = hypot(a, b)` — an equation the figure
/// already satisfies, so the diagnosis reports it as redundant and consistent.  That is the
/// theorem: the dimension is true without being imposed, and stays true when `a` or `b` is
/// edited.  Corner O is fixed; everything else is determined (0 DOF).
pub fn pythagoras(a: f64, b: f64) -> Sketch {
    let mut sk = Sketch::new();
    let s = a + b;
    let o = sk.point(0.0, 0.0, true, "O");
    let lines = sk.rectangle(o, s, s, "sq");   // bottom, right, top, left; O, E, F, G round it
    let corner = |sk: &Sketch, i: usize| sk.lines[lines[i]].p1 as usize;
    let (e, f, g) = (corner(&sk, 1), corner(&sk, 2), corner(&sk, 3));
    add(&mut sk, Constraint::one_line(CKind::Horizontal, EntRef::line(lines[0])));
    add(
        &mut sk,
        Constraint::two_line(CKind::EqualLength, EntRef::line(lines[0]), EntRef::line(lines[3])),
    );
    // one point on each side, `a` from the corner it follows going round, so each corner holds
    // a triangle with legs a and b: P1 on the bottom, P2 on the right, P3 on the top, P4 on the left
    let p1 = sk.point(a, 0.0, false, "P1");
    let p2 = sk.point(s, a, false, "P2");
    let p3 = sk.point(b, s, false, "P3");
    let p4 = sk.point(0.0, b, false, "P4");
    for (p, l) in [(p1, lines[0]), (p2, lines[1]), (p3, lines[2]), (p4, lines[3])] {
        add(
            &mut sk,
            Constraint::new(
                CKind::PointOnLine,
                vec![Arg::Ent(EntRef::point(p)), Arg::Ent(EntRef::line(l))],
            ),
        );
    }
    add(&mut sk, expr_distance(o, p1, &format!("a = {}", crate::json::fmt_g(a, 6))));
    add(&mut sk, expr_distance(p1, e, &format!("b = {}", crate::json::fmt_g(b, 6))));
    add(&mut sk, expr_distance(e, p2, "a"));
    add(&mut sk, expr_distance(f, p3, "a"));
    add(&mut sk, expr_distance(g, p4, "a"));
    // the hypotenuses, which are the inner square
    sk.line(p1, p2);
    sk.line(p2, p3);
    sk.line(p3, p4);
    sk.line(p4, p1);
    add(&mut sk, expr_distance(p1, p2, "c = hypot(a, b)"));
    sk
}

/// The four sketches the regression suite parametrises over.
pub const EXAMPLES: [&str; 5] =
    ["rect_fillets", "slotted_link", "truss", "polygon_chain", "spline_follower"];

/// A cubic B-spline with a straight follower held tangent to it and a point riding on it.
///
/// What it is for is the two things a curve adds that no implicit primitive has.  The tangency
/// owns the parameter it touches at, so dragging a control point slides the contact along the
/// curve rather than breaking it — and it slides *past knots*, which changes which control
/// points the constraint's columns name and quietly recompiles.  The point on the curve is
/// dimensioned to a fixed anchor, so it has somewhere to be and the curve is not free to
/// swallow it.
pub fn spline_follower(n: usize) -> Sketch {
    let mut sk = Sketch::new();
    let n = n.max(4);
    let ctrl: Vec<usize> = (0..n)
        .map(|i| {
            let x = i as f64 * 20.0;
            let y = if i % 2 == 0 { 26.0 } else { 0.0 };
            sk.point(x, y, false, &format!("k{i}"))
        })
        .collect();
    sk.fix_point(ctrl[0], true);
    let sp = sk.spline(&ctrl).expect("four control points is a cubic");
    let spe = EntRef::spline(sp);

    // Both halves start where they already belong, so opening the case shows the curve as it is
    // drawn rather than the nearest configuration to it.  The follower rides where the curve
    // already dips lowest — an interior dip, not an end, so the contact has curve on both sides
    // of it to slide along.
    let (t0, t1) = crate::curve::domain(&sk, sp);
    let low = crate::curve::sample(&sk, sp, 64).into_iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let span = (n - 1) as f64 * 20.0;
    let a = sk.point(0.0, low, false, "f1");
    let b = sk.point(span, low, false, "f2");
    let face = sk.line(a, b);
    add(&mut sk, Constraint::one_line(CKind::Horizontal, EntRef::line(face)));
    let tangent = Constraint::spline_tangent_line(&sk, spe, EntRef::line(face));
    add(&mut sk, tangent);

    // and a point riding on the curve, held at a distance from a fixed anchor above it, so the
    // curve is not free to shrug it off
    let mid = crate::curve::point_at(&sk, sp, 0.5 * (t0 + t1));
    let rider = sk.point(mid.0, mid.1, false, "rider");
    let anchor = sk.point(mid.0, mid.1 + 60.0, true, "anchor");
    let on = Constraint::point_on_spline(&sk, EntRef::point(rider), spe);
    add(&mut sk, on);
    add(&mut sk, Constraint::distance(EntRef::point(anchor), EntRef::point(rider), 60.0));
    sk
}

/// Build a named example.  `None` for an unknown name.
/// `copies` disjoint chains of `n` points each, every segment alternately Vertical and Horizontal
/// and nothing else: a staircase with all its lengths free.  Every levelled line is in the ground
/// x-axis's direction class, which makes it the sketch that finds any cost that goes with the
/// *class* rather than with the geometry — and `copies` of it find any cost that goes with the
/// document rather than with the figure being dragged.
pub fn zigzag(n: usize, copies: usize) -> Sketch {
    let mut sk = Sketch::new();
    for c in 0..copies {
        let ox = c as f64 * (4.0 * n as f64);
        let mut prev = sk.point(ox, 0.0, false, &format!("z{c}_0"));
        for i in 1..n {
            let (px, py) = sk.point_xy(prev);
            let (x, y) = if i % 2 == 1 { (px, py + 5.0) } else { (px + 3.0, py) };
            let p = sk.point(x, y, false, &format!("z{c}_{i}"));
            let l = sk.line(prev, p);
            let kind = if i % 2 == 1 { CKind::Vertical } else { CKind::Horizontal };
            add(&mut sk, Constraint::one_line(kind, EntRef::line(l)));
            prev = p;
        }
    }
    sk
}

/// A belt segment over two pulleys, stated the way a draughtsman reaches for it: each end held
/// on its circle, the line tangent to each.  That pair says "tangent at the endpoint" as a
/// *double root* — the Jacobian is rank-deficient at every solution because each contact can
/// swim along the line to first order — so the figure is rigid while the numbers alone say it
/// has two degrees of freedom.  What the second-order screen is for; `TangentLineCircleAt` is
/// how the app states it now.
pub fn belt_tangency() -> Sketch {
    let mut sk = Sketch::new();
    let c1 = sk.point(0.0, 0.0, true, "c1");
    let c2 = sk.point(50.0, 0.0, true, "c2");
    let k1 = sk.circle(c1, 10.0, "k1");
    let k2 = sk.circle(c2, 10.0, "k2");
    let p = sk.point(0.0, 10.0, false, "p");
    let q = sk.point(50.0, 10.0, false, "q");
    let line = sk.line(p, q);
    sk.add(Constraint::radius(EntRef::circle(k1), 10.0));
    sk.add(Constraint::radius(EntRef::circle(k2), 10.0));
    sk.add(Constraint::point_on_circle(EntRef::point(p), EntRef::circle(k1), false));
    sk.add(Constraint::point_on_circle(EntRef::point(q), EntRef::circle(k2), false));
    for k in [k1, k2] {
        let t = Constraint::tangent_line_circle(&sk, EntRef::line(line), EntRef::circle(k), None);
        sk.add(t);
    }
    sk
}

pub fn example(name: &str) -> Option<Sketch> {
    Some(match name {
        "rect_fillets" => rect_fillets(100.0, 60.0, 10.0, 0.0),
        "slotted_link" => slotted_link(80.0, 15.0, 6.0),
        "truss" => truss(8, 20.0, 15.0, true),
        "polygon_chain" => polygon_chain(12, 50.0),
        "rect_fillets_conflict" => rect_fillets_conflict(),
        "rect_fillets_under" => rect_fillets_under(),
        "truss_redundant" => truss_redundant(),
        "truss_conflict" => truss_conflict(),
        "truss_floating" => truss_floating(8),
        "impossible_triangle" => impossible_triangle(),
        "altitudes" => altitudes(),
        "parallels" => parallels(),
        "pythagoras" => pythagoras(30.0, 40.0),
        "k33" => k33(3),
        "laman" => laman(10, 0, true),
        "zigzag" => zigzag(32, 3),
        "spline_follower" => spline_follower(7),
        "belt_tangency" => belt_tangency(),
        _ => return None,
    })
}

/// The case library shown in the app: (label, key, one-line description).
pub const CASES: [(&str, &str, &str); 22] = [
    ("Rectangle with fillets", "rect_fillets", "fully constrained; tangent arcs, equal radii, two dimensions"),
    ("Slotted link", "slotted_link", "obround slot with two holes; fully constrained"),
    ("Truss (8 bays)", "truss", "~30-entity Warren truss, every member dimensioned"),
    ("Truss (50 bays)", "truss50", "300 entities — drag a node"),
    ("Truss (200 bays)", "truss200", "1200 entities — solver/plan timing"),
    ("Truss, floating", "truss_floating", "rigid body with nothing fixed: 3 DOF, drag it around"),
    ("Polygon chain (12)", "polygon_chain", "under-constrained equal-length ring; the EqualLength cycle is a redundancy the graph can't see"),
    ("Rect, missing width", "rect_fillets_under", "under-constrained: the right side slides (null-space colouring)"),
    ("Rect, conflicting width", "rect_fillets_conflict", "conflict: two contradicting width dimensions"),
    ("Truss, redundant member", "truss_redundant", "structurally over-constrained but consistent (amber)"),
    ("Truss, impossible member", "truss_conflict", "conflict: a 999-long member; the minimal conflict set is a path plus it"),
    ("Impossible triangle", "impossible_triangle", "structurally fine, geometrically impossible (triangle inequality)"),
    ("K3,3 framework", "k33", "rigid but triangle-free: the decomposition needs a core merge"),
    ("Random Laman #0", "laman0", "Henneberg-built minimally rigid framework"),
    ("Random Laman #1", "laman1", "Henneberg-built; may need a core (Henneberg II)"),
    ("Concurrent altitudes", "altitudes", "theorem-type dependency: the third incidence is implied (Diagnose → witness); 3 DOF to animate"),
    ("Parallels & perpendiculars", "parallels", "direction classes: parallel/perpendicular/vertical (1 DOF left: slide along the base)"),
    ("Pythagoras, graphically", "pythagoras", "four a×b right triangles in a square of side a + b leave a square of side c; `c = hypot(a, b)` is redundant and consistent — edit a or b and it stays so"),
    ("Curve and follower", "spline_follower", "a cubic B-spline with a face held tangent to it and a point riding on it — drag a control point and the contact slides along the curve, across knots and all"),
    ("Belt over two pulleys", "belt_tangency", "each end on its circle and the line tangent to it — a double root: rank-deficient at every solution, yet nothing can move.  The second-order screen calls it rigid rather than 2 DOF"),
    ("Spur gear (30 teeth)", "gear", "written as a Solvent program: a curve family the document itself defines, one flank as a component, repeated round a cycle — open the Program panel (Edit ▸ Program) to read it"),
    ("Levelled zigzags (3×32)", "zigzag", "three separate staircases of free-length H/V segments — a drag costs one staircase, not three"),
];

/// A spur gear, written as a Solvent program rather than built here.
///
/// The one case in the library that is a *document* and not a function: a tooth is a component
/// and the wheel is that component round a cycle, which is what the language is for and what no
/// amount of `Sketch::add` says as clearly.  It is also the regression test for elaboration —
/// components, instances, parameters, `next`, and expressions worked out at elaboration time all
/// have to hold for it to come out round.
pub const GEAR: &str = include_str!("gear.sv");

pub fn gear() -> Sketch {
    let (p, errs) = crate::syntax::parse(GEAR);
    debug_assert!(errs.is_empty(), "the gear does not parse: {errs:?}");
    let e = crate::program::elaborate(&p);
    debug_assert!(e.ok(), "the gear does not elaborate");
    e.sketch
}

/// The *source* of a case, for the library that has one.
///
/// A case written as a document has a text somebody wrote, and that text — its comments, its
/// components, the reasons in it — is the case.  Lifting the sketch it elaborates to would print
/// a hundred and twenty `point` declarations and none of the explanation, which is a different
/// document about the same drawing.  Every other case is a function, so it has no source and the
/// caller lifts it.
pub fn source(key: &str) -> Option<&'static str> {
    match key.split(':').next().unwrap_or("") {
        "gear" => Some(GEAR),
        _ => None,
    }
}

/// The case library's factory.  Keys are either a plain name or `name:arg[:arg]`, so a front end
/// can ask for `truss:50` or `laman:12:1` without a table of its own.
pub fn case(key: &str) -> Option<Sketch> {
    let mut parts = key.split(':');
    let name = parts.next().unwrap_or("");
    let args: Vec<f64> = parts.filter_map(|p| p.parse().ok()).collect();
    let n = |i: usize, d: usize| args.get(i).map(|&v| v as usize).unwrap_or(d);
    let u = |i: usize, d: u32| args.get(i).map(|&v| v as u32).unwrap_or(d);
    Some(match name {
        "truss50" => truss(50, 20.0, 15.0, true),
        "truss200" => truss(200, 20.0, 15.0, true),
        "laman0" => laman(10, 0, true),
        "laman1" => laman(12, 1, true),
        "truss" if !args.is_empty() => truss(n(0, 8), 20.0, 15.0, true),
        "truss_floating" if !args.is_empty() => truss_floating(n(0, 8)),
        "polygon_chain" if !args.is_empty() => polygon_chain(n(0, 12), 50.0),
        "k33" if !args.is_empty() => k33(u(0, 3)),
        "gear" => gear(),
        "laman" => laman(n(0, 10), u(1, 0), true),
        "zigzag" if !args.is_empty() => zigzag(n(0, 32), n(1, 1)),
        "spline_follower" if !args.is_empty() => spline_follower(n(0, 7)),
        "rect_fillets" if args.len() >= 3 => rect_fillets(args[0], args[1], args[2], 0.0),
        "pythagoras" if args.len() >= 2 => pythagoras(args[0], args[1]),
        _ => return example(name),
    })
}
