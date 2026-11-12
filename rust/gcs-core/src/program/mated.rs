//! **A part mated by what it made starts where its own solve, placed whole, puts it** (#904 A5).
//!
//! An instance read as a frame (#548) stands in a square frame of its own, and a mate on its
//! datums seeds that frame where the mate puts it (#764, `views::place`).  A part placed instead
//! by relations on what it made — a gear's axis along a shaft, a pinion's square to another —
//! has nothing to seed it from before it is solved: its frame's six numbers are read, through
//! every point it draws, by every row of its body, so the document is one block of the part's
//! whole layout and its placement together, and a part with a delicate solve of its own (the
//! hypoid pair's face mill, balanced on envelopes) does not come out of it.  But a part is a
//! function of the frame it stands in (#461): solved once where its frame stands, it is placed
//! by a rigid motion, and the mate is six numbers.  So it is seeded as a designer would place it:
//!
//! 1. **alone**: its body's rows, and the document's rows over nothing but what it made (a
//!    balance its caller states of its margins), solved with its frame held where it stands —
//!    `boundary::split`'s reading of it, every input held;
//! 2. **its frame placed**: the document solved for its frame with everything it made that
//!    stands in space — a point's lift (what a relation in space reads of a point), a point in
//!    space, a plane's place, an axis's direction and foot — carried rigidly by the frame
//!    (`Sketch::rigid`, a fixed combination of the frame's place and axes, derived as a ring's
//!    copies are) and the rest of it held, its own rows set aside: the mate's six numbers;
//! 3. **the document solved** from there with its own coordinates held — a point's numbers in
//!    the plane or the frame it is drawn in, a radius, a number, a contact's place along what it
//!    is on — so what it made moves only as its frame carries it, and what a rigid motion does
//!    not carry (an axis's foot, the point of its line nearest the world's origin; a contact's
//!    place on a surface charted by the world's axes) is found again.
//!
//! Each is kept only where it settles, and nothing is kept unless all do: the seeds are then
//! the document's own answer, which its solve confirms, and otherwise stand as they were.  A part
//! that reads anything the document places besides its frame (an entity it is handed) is not a
//! function of its frame alone, and is left to the document's solve.
use crate::boundary::{columns, reduced, split, World};
use crate::model::{EntKind, EntRef, Sketch};
use crate::program::Elaborated;
use crate::solve::{solve, SolveOpts};
use std::collections::BTreeSet;

/// A part mated by what it made: its frame, that frame's numbers, its own, and the rows it is
/// solved alone by.
struct Mated {
    f: usize,
    frame: BTreeSet<u32>,
    own: BTreeSet<u32>,
    alone: BTreeSet<u32>,
}

/// Every part mated by relations on what it made, seeded solved and placed (see the module).
pub(crate) fn seat(e: &mut Elaborated) {
    let prog = &e.program;
    let mut drawn: Vec<_> = e.map.instances.iter()
        .filter(|i| i.drawn && prog.components[i.comp].braced())
        .filter(|i| e.map.ent_named(&crate::flatten::instance_frame(name(&i.prefix)))
            .is_some_and(|f| f.kind == EntKind::Frame))
        .collect();
    if drawn.is_empty() {
        return;
    }
    let parts = mated(e, &mut drawn);
    if parts.is_empty() {
        return;
    }
    if let Some(x) = placed(&e.sketch, &parts) {
        e.sketch.set_x(&x);
    }
}

/// The instances of `drawn` mated by what they made, the outermost first: a part inside one
/// taken is solved with it.
fn mated(e: &Elaborated, drawn: &mut [&crate::program::Instanced]) -> Vec<Mated> {
    let sk = &e.sketch;
    let world = World::of(sk, &e.map);
    let reads: Vec<(u32, Vec<u32>)> = sk.constraints.iter().filter(|c| c.acts())
        .map(|c| (c.id, columns(sk, c))).collect();
    drawn.sort_by_key(|i| i.path.0.len());
    let mut taken: Vec<&crate::program::Instanced> = Vec::new();
    let mut parts = Vec::new();
    for &inst in drawn.iter() {
        if taken.iter().any(|t| t.contains(&inst.path)) {
            continue;
        }
        let Some(f) = e.map.ent_named(&crate::flatten::instance_frame(name(&inst.prefix)))
        else { continue };
        if !sk.frames[f.i()].chart.as_ref().is_some_and(|c| c.square) {
            continue;
        }
        let frame = frame_params(sk, f.i());
        if frame.iter().all(|&p| sk.params[p as usize].fixed) {
            continue;
        }
        let s = split(&e.map, sk, &world, inst);
        // what it made, with the numbers its call left unknown (`mn`), which its rows find
        let mut own: BTreeSet<u32> = s.own.difference(&frame).copied().collect();
        own.extend(sk.free_vars.iter().filter(|(k, _)| k.starts_with(&inst.prefix))
            .map(|(_, &p)| p));
        let within = |p: &u32| own.contains(p) || frame.contains(p) || Some(*p) == sk.zero;
        // its rows read what it made, its frame and what it holds of its own, and nothing the
        // document places: it is a function of its frame alone
        let held = |p: &u32| sk.params[*p as usize].fixed
            && world.owner.get(p).is_none_or(|o| s.made.contains(o));
        if reads.iter().filter(|(id, _)| s.kept.contains(id))
            .any(|(_, cols)| !cols.iter().all(|p| within(p) || held(p))) {
            continue;
        }
        // the document's rows over it: those reading nothing else are its own as well (a balance
        // of its numbers), and the rest place it
        let mut alone = s.kept.clone();
        let mut mates = 0;
        for (id, cols) in reads.iter().filter(|(id, _)| !s.kept.contains(id)) {
            if !cols.iter().any(|p| own.contains(p)) {
                continue;
            }
            if cols.iter().all(within) {
                alone.insert(*id);
            } else {
                mates += 1;
            }
        }
        if mates == 0 {
            continue;
        }
        taken.push(inst);
        parts.push(Mated { f: f.i(), frame, own, alone });
    }
    parts
}

/// The numbers `sk` comes to with each of `parts` solved alone where its frame stands, its frame
/// placed with what it made carried rigidly, and the whole solved from there with the parts' own
/// coordinates held — `None` where a step does not settle.
fn placed(sk: &Sketch, parts: &[Mated]) -> Option<Vec<f64>> {
    let mut x = sk.get_x();
    // 1. each alone, its frame held where it stands
    for part in parts {
        let mut alone = reduced(sk, &part.alone, |p, fixed| fixed || !part.own.contains(&p));
        // and only its own stated orientations: another's seeds may break one, which the
        // document's solve crosses (`solve::steer`) and this one could not
        let its = |p: usize| sk.point_all_params(p).iter().any(|q| part.own.contains(q));
        alone.orientations.retain(|k, _| {
            crate::decompose::branch_key_points(k).is_some_and(|ps| ps.into_iter().any(its))
        });
        if !solve(&mut alone, SolveOpts::default()).settled() {
            return None;
        }
        for &p in &part.own {
            x[p as usize] = alone.params[p as usize].value;
        }
    }
    // 2. each frame placed, what each part made standing in space carried with it rigidly
    // (`Sketch::rigid`) — its points' lifts by their own rows — and the rest of it held, none of
    // its rows but those lifts' and those over its frame alone
    let mut framed = sk.clone();
    framed.set_x(&x);
    for part in parts {
        carry(&mut framed, part)?;
    }
    let lift = |c: &crate::constraints::Constraint| {
        matches!(c.kind, crate::constraints::CKind::Lift | crate::constraints::CKind::ChartLift)
    };
    framed.constraints.retain(|c| {
        lift(c) || parts.iter().all(|part| !part.alone.contains(&c.id)
            || columns(sk, c).iter().all(|p| part.frame.contains(p) || Some(*p) == sk.zero))
    });
    if !solve(&mut framed, SolveOpts::default()).settled() {
        return None;
    }
    // 3. the document solved from there with the parts' own coordinates held: what stands in
    // space is solved from where its frame carried it, and a contact's place on a surface the
    // world's axes chart is found again
    let mut placed = sk.clone();
    placed.set_x(&framed.get_x()[..sk.params.len()]);
    for part in parts {
        for p in coordinates(sk, &part.own) {
            placed.params[p as usize].fixed = true;
        }
    }
    solve(&mut placed, SolveOpts::default()).settled().then(|| placed.get_x())
}

/// `part`'s places in space made rigid with its frame in `sk` (`Sketch::rigid`), and the rest of
/// its own numbers held but its points' lifts, which their rows carry: each point in space, each
/// plane's place and each axis's direction and foot a fixed combination of the frame's place and
/// its axes' directions (its normal minted where the drawing has none), standing where it does.
/// `None` where the frame is itself derived or its axes are degenerate.
fn carry(sk: &mut Sketch, part: &Mated) -> Option<()> {
    use crate::model::Derivation;
    use crate::space::{cross, dot, sub};
    let derived = sk.derived_mask();
    let is_derived = |q: u32| derived.get(q as usize).copied().unwrap_or(false);
    if part.frame.iter().any(|&q| is_derived(q)) {
        return None;
    }
    let n = sk.plane_normal(part.f, "");
    let c = sk.frames[part.f].chart.clone()?;
    let e = [c.u as usize, c.v as usize, n].map(|a| sk.axes[a].d);
    let val = |sk: &Sketch, q: u32| sk.params[q as usize].value;
    let e0 = e.map(|d| d.map(|q| val(sk, q)));
    let o0 = c.o.map(|q| val(sk, q));
    // a vector's numbers along the axes as they stand: the rows of the basis's inverse
    let det = dot(e0[0], cross(e0[1], e0[2]));
    if !(det.abs() > 1e-12) {
        return None;
    }
    let inv = [cross(e0[1], e0[2]), cross(e0[2], e0[0]), cross(e0[0], e0[1])];
    let mut rigid = Vec::new();
    // a point's lift follows its own row, so a bound read of it can be crossed (`solve::steer`)
    let mut made: BTreeSet<u32> = sk.lifts.iter().flat_map(|l| l.x).collect();
    let mut add = |sk: &Sketch, qs: [u32; 3], point: bool| {
        let fit =
            |q: &u32| part.own.contains(q) && !sk.params[*q as usize].fixed && !is_derived(*q);
        if !qs.iter().all(fit) {
            return;
        }
        let w = qs.map(|q| val(sk, q));
        let w = if point { sub(w, o0) } else { w };
        let k = inv.map(|r| dot(r, w) / det);
        for i in 0..3 {
            let mut terms = if point { vec![(c.o[i], 1.0)] } else { Vec::new() };
            terms.extend((0..3).map(|j| (e[j][i], k[j])));
            rigid.push(Derivation { param: qs[i], terms });
        }
        made.extend(qs);
    };
    for p in &sk.points {
        if let Some(z) = p.z.filter(|_| p.plane.is_none() && p.chart.is_none()) {
            add(sk, [p.x, p.y, z], true);
        }
    }
    for f in sk.frames.iter().filter_map(|f| f.chart.as_ref()) {
        add(sk, f.o, true);
    }
    for a in &sk.axes {
        add(sk, a.d, false);
        add(sk, a.a, true);
    }
    for &q in part.own.iter().filter(|q| !made.contains(q) && !is_derived(**q)) {
        sk.params[q as usize].fixed = true;
    }
    sk.rigid.extend(rigid);
    Some(())
}

/// An instance's name from its prefix: `pair` of `pair.`.
fn name(prefix: &str) -> &str {
    prefix.trim_end_matches('.')
}

/// The numbers frame `f` stands by: its place and its axes', and those of the datums standing
/// where it does (`Place::Of`), with their origins.
fn frame_params(sk: &Sketch, f: usize) -> BTreeSet<u32> {
    let mut out = BTreeSet::new();
    for (g, fr) in sk.frames.iter().enumerate() {
        let Some(c) = &fr.chart else { continue };
        if g != f && !matches!(c.place, crate::model::Place::Of(h) if h as usize == f) {
            continue;
        }
        out.extend(sk.own_params(EntRef::frame(g)));
        out.extend(c.o);
        out.extend(sk.point_all_params(c.origin as usize));
        for a in c.axes() {
            out.extend(sk.own_params(EntRef::new(EntKind::Axis, a as usize)));
        }
    }
    out
}

/// Of `own`, the numbers a rigid motion of the frame they are drawn in leaves as they are: a
/// point's coordinates in the plane or the frame it is drawn in, a radius, a number of the
/// drawing's, a contact's place along what it is on, a joint's rate and advance.  Its places in
/// space — an axis, a plane's place, a point in space, a point's lift — are not among them.
fn coordinates(sk: &Sketch, own: &BTreeSet<u32>) -> Vec<u32> {
    let mut out: BTreeSet<u32> = BTreeSet::new();
    for (i, p) in sk.points.iter().enumerate() {
        if p.plane.is_some() || p.chart.is_some() {
            out.extend(sk.point_all_params(i));
        }
    }
    out.extend(sk.circles.iter().map(|c| c.radius));
    out.extend(sk.arcs.iter().map(|a| a.radius));
    out.extend(sk.free_vars.values().copied());
    out.extend(sk.constraints.iter().filter_map(|c| c.place()).map(|(_, t)| t));
    for j in sk.frames.iter().filter_map(|f| f.joint.as_ref()) {
        out.extend([j.ratio, j.advance]);
    }
    out.retain(|p| own.contains(p));
    out.into_iter().collect()
}
