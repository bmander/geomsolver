//! **The rebuild walk** (`graft`): deletion, copying, pasting and a drag's part are one walk
//! whose answer is a `Renumbering` — where each entity and each constraint of the source went.
//! The walk's rules for what an entity cannot be without are the model's own (`remapped` on each
//! value), and these hold them to the document corpus: a rebuild that removes nothing keeps
//! everything where it was, what a removal takes with it is what `Sketch::children` says depends
//! on what was removed, and the things the walk says as a whole — the unit a paste lands in, the
//! pictures, the numbers a drag writes back — come out as the document has them.
use crate::common::freedoms;
use gcs_core::constraints::CKind;
use gcs_core::graft::{self, Part};
use gcs_core::model::{CurveBody, DerivedE, EntKind, EntRef, Origin, Owner, RegionShape, SolidDef,
                      Sketch, Target};
use gcs_core::units::Units;
use gcs_core::par;

/// The example at `rel` under `rust/examples/`, elaborated and not solved — `None` for one that
/// does not elaborate.
fn example(rel: &str) -> Option<Sketch> {
    fixtures::example(rel).filter(|e| e.ok()).map(|e| e.sketch)
}

/// Every entity of `sk`, kind by kind.
fn entities(sk: &Sketch) -> Vec<EntRef> {
    EntKind::ALL.iter().flat_map(|&k| (0..sk.count(k)).map(move |i| EntRef::new(k, i))).collect()
}

/// **A rebuild that removes nothing keeps everything where it was**, in every document of the
/// corpus: each entity is itself, each constraint the document states has its copy, each
/// picture comes along.  Before the walk read every kind through one table, a region solid
/// whose term reads a face's side panicked here (every document with a rounded edge), and a
/// curve written over another curve was left behind (`wankel.sv`).
#[test]
fn a_rebuild_that_removes_nothing_keeps_everything() {
    let names = fixtures::example_names();
    let wrong: Vec<String> = par::map(&names, |name| {
        let Some(sk) = example(name) else { return Vec::new() };
        let (copy, r) = graft::rebuild(&sk, &|_| true, &[]);
        let mut wrong = Vec::new();
        for kind in EntKind::ALL {
            if copy.count(kind) != sk.count(kind) {
                wrong.push(format!("{name}: {} {kind:?} of {}", copy.count(kind), sk.count(kind)));
            }
        }
        for e in entities(&sk) {
            if r.ent(e) != Some(e) {
                wrong.push(format!("{name}: {e:?} became {:?}", r.ent(e)));
            }
        }
        for c in sk.user_constraints().into_iter().filter(|c| !c.kind.of_source()) {
            if r.constraint(c.id).is_none() {
                wrong.push(format!("{name}: constraint {} ({:?}) left behind", c.id, c.kind));
            }
        }
        if copy.derived.len() != sk.derived.len() {
            wrong.push(format!("{name}: {} pictures of {}", copy.derived.len(), sk.derived.len()));
        }
        wrong
    }).into_iter().flatten().collect();
    assert!(wrong.is_empty(), "{wrong:#?}");
}

/// What the walk reads of `e` that `Sketch::children` does not list, beside what it does: a
/// face's plane, a curve's trim, and the probe of a set a region's term measures from.  Each is
/// something `e` cannot be rebuilt without, so the walk drops `e` when it goes, while deletion
/// (`without`), copying (`copy`) and a drag's part (`Part::around_all`) walk `children` alone.
/// They are listed here so a gap is said where it is; closing one is a change to `children`.
fn read_beyond_children(sk: &Sketch, e: EntRef) -> Vec<EntRef> {
    fn probes(t: &Target, out: &mut Vec<EntRef>) {
        if let Target::Set { probe, terms } = t {
            out.push(EntRef::point(*probe as usize));
            for term in terms {
                shape_probes(&term.shape, out);
            }
        }
    }
    fn shape_probes(s: &RegionShape, out: &mut Vec<EntRef>) {
        match s {
            RegionShape::Measured { of } => of.iter().for_each(|(t, _)| probes(t, out)),
            RegionShape::Ends { edge } => probes(edge, out),
            _ => {}
        }
    }
    let mut out = Vec::new();
    match e.kind {
        EntKind::Face => out.extend(sk.faces[e.i()].plane.map(|p| EntRef::frame(p as usize))),
        EntKind::Curve => {
            if let Some(t) = sk.curves[e.i()].trim {
                out.extend([EntRef::new(EntKind::Curve, t.of as usize),
                            EntRef::point(t.from as usize), EntRef::point(t.to as usize)]);
            }
        }
        EntKind::Solid => {
            if let SolidDef::Region { terms, .. } = &sk.solids[e.i()].def {
                terms.iter().for_each(|t| shape_probes(&t.shape, &mut out));
            }
        }
        _ => {}
    }
    out
}

/// **What a removal takes with it is what depends on what was removed.**  Each entity of a set
/// of documents spanning the kinds — trimmed curves and faces in planes, region solids over
/// faces' sides and sets, motions, traced bodies, envelopes and meshings — is removed alone, the
/// walk left to follow (`keep` refuses that one entity and nothing else), and everything that
/// went with it must reach it through `children` and the gaps `read_beyond_children` names.
#[test]
fn what_a_removal_takes_is_what_depends_on_it() {
    let docs = ["solid_tooth.sv", "rounds/disc.sv", "rounds/blank.sv", "wankel/wankel.sv",
                "paths/frenet.sv", "generation/spur_mesh.sv", "spiral_bevel/meshing_problem.sv",
                "lofts/ribbon.sv", "conrod/rod.sv"];
    let wrong: Vec<String> = par::map(&docs, |name| {
        let sk = example(name).unwrap_or_else(|| panic!("{name} elaborates"));
        let all = entities(&sk);
        // everything each entity depends on, through any number of steps, worked out once
        let reach: std::collections::BTreeMap<EntRef, std::collections::BTreeSet<EntRef>> =
            all.iter().map(|&e| {
                let mut seen = std::collections::BTreeSet::new();
                let mut todo = vec![e];
                while let Some(d) = todo.pop() {
                    let mut next = sk.children(d);
                    next.extend(read_beyond_children(&sk, d));
                    for n in next {
                        if seen.insert(n) {
                            todo.push(n);
                        }
                    }
                }
                (e, seen)
            }).collect();
        let depends = |e: EntRef, on: EntRef| reach[&e].contains(&on);
        // every entity of a small document, an even sample of a large one
        let step = all.len().div_ceil(60).max(1);
        let mut wrong = Vec::new();
        for &x in all.iter().step_by(step) {
            let (_, r) = graft::rebuild(&sk, &|e| e != x, &[]);
            for &e in &all {
                if e != x && r.ent(e).is_none() && !depends(e, x) {
                    wrong.push(format!("{name}: removing {x:?} took {e:?}"));
                }
            }
        }
        wrong
    }).into_iter().flatten().collect();
    assert!(wrong.is_empty(), "{wrong:#?}");
}

/// **A paste lands in the document's unit**, an empty document's too.  The walk used to take an
/// empty destination for a rebuild and stamp the clipboard's unit on it, so a 1 in figure pasted
/// into a new millimetre document came out as 25.4 in.
#[test]
fn a_paste_into_an_empty_document_lands_in_its_unit() {
    let mut clip = Sketch::new();
    clip.units = Units::with_length("in").unwrap();
    clip.point(1.0, 0.0, false, "p0");
    for mut dst in [Sketch::new(), {
        let mut busy = Sketch::new();
        busy.point(0.0, 0.0, false, "q");
        busy
    }] {
        let made = graft::paste(&mut dst, &clip, 0.0, 0.0);
        assert_eq!(dst.units.name(), "mm");
        assert_eq!(made.len(), 1);
        assert!((dst.point_xy(made[0].i()).0 - 25.4).abs() < 1e-12);
    }
    // a rebuild is the document it came from, unit and all
    assert_eq!(graft::without(&clip, &[], &[]).units.name(), "in");
}

/// **The pictures a document asks for come with their body** (§6.11): `derived` is document
/// state, saved and grafted, and goes when its solid does.
#[test]
fn a_picture_comes_with_its_body() {
    let mut sk = example("solid_tooth.sv").unwrap();
    assert!(!sk.solids.is_empty());
    sk.derived.push(DerivedE { solid: 0, plane: None, at: None, dims: false, name: "front".into(),
                               class: Default::default() });
    let kept = graft::without(&sk, &[], &[]);
    assert_eq!(kept.derived.len(), sk.derived.len());
    assert_eq!(kept.derived.last().unwrap().name, "front");
    let gone = graft::without(&sk, &[EntRef::solid(0)], &[]);
    assert!(gone.derived.iter().all(|d| d.name != "front"));
}

/// **A drag's part writes back a contact's place on a traced body.**  The part pairs each
/// constraint's own unknowns with its copy's by the walk's constraint table, so a contact over
/// a kind the part's entity table once left out (a traced body, a motion) pairs too.
#[test]
fn a_part_pairs_a_contacts_place_on_a_traced_body() {
    let sk = example("paths/frenet.sv").unwrap();
    let contact = sk.constraints.iter().find(|c| c.kind == CKind::PointOnTraced)
        .expect("frenet.sv holds a point to its traced body");
    let point = contact.entities()[0];
    let mut part = Part::around(&sk, point);
    let theirs = part.sketch.constraints.iter().find(|c| c.kind == CKind::PointOnTraced)
        .expect("the contact came along")
        .aux_params();
    let ours = contact.aux_params();
    assert_eq!(theirs.len(), ours.len());
    for (k, &p) in theirs.iter().enumerate() {
        part.sketch.params[p as usize].value = 0.125 + k as f64;
    }
    let mut doc = sk.clone();
    part.write_back(&mut doc);
    for (k, &p) in ours.iter().enumerate() {
        assert_eq!(doc.params[p as usize].value, 0.125 + k as f64, "own unknown {k}");
    }
    assert_eq!(part.entity_in(contact.entities()[1]).map(|e| e.kind), Some(EntKind::Traced));
}

/// The intrinsic rows of `sk`, each by its owner renumbered through `map` and its kind, sorted:
/// what two copies of one document must agree on.
fn intrinsic_rows(sk: &Sketch, map: &dyn Fn(EntRef) -> Option<EntRef>) -> Vec<String> {
    let idx = |e: EntRef| map(e).map(|e| e.idx);
    let owner = |o: Owner| -> Option<Owner> {
        Some(match o {
            Owner::Axis(i) => Owner::Axis(idx(EntRef::new(EntKind::Axis, i as usize))?),
            Owner::Arc(i) => Owner::Arc(idx(EntRef::arc(i as usize))?),
            Owner::Frame(f) => Owner::Frame(idx(EntRef::frame(f as usize))?),
            Owner::Normal(f) => Owner::Normal(idx(EntRef::frame(f as usize))?),
            Owner::Lift(p) => Owner::Lift(idx(EntRef::point(p as usize))?),
            Owner::Twin(p, t) => Owner::Twin(idx(EntRef::point(p as usize))?,
                                             idx(EntRef::point(t as usize))?),
            Owner::Round(e) => Owner::Round(map(e)?),
            Owner::Developed(i) => Owner::Developed(idx(EntRef::new(EntKind::Curve, i as usize))?),
        })
    };
    let mut rows: Vec<String> = sk.constraints.iter()
        .filter(|c| c.along.is_none())
        .filter_map(|c| match c.origin {
            // the twin a statement's planes feed is one statement (`CKind::attitude_twin`)
            Origin::Intrinsic(o) => {
                Some(format!("{:?} {:?}", owner(o), c.kind.attitude_twin(false)))
            }
            Origin::Stated => None,
        })
        .collect();
    rows.sort();
    rows
}

/// Whether the JSON document carries every kind `sk` holds: a sketch of points, lines, circles,
/// arcs, splines, axes and frames, and no row of a kind it cannot name or a row the Solvent
/// source alone carries (#510 S1).
fn json_carries(sk: &Sketch) -> bool {
    let carried = [EntKind::Point, EntKind::Line, EntKind::Circle, EntKind::Arc, EntKind::Spline,
                   EntKind::Axis, EntKind::Frame];
    EntKind::ALL.iter().all(|&k| carried.contains(&k) || sk.count(k) == 0)
        && sk.constraints.iter().all(|c| {
            !c.kind.of_source() && c.entities().iter().all(|e| carried.contains(&e.kind))
        })
}

/// Whether a rebuild of `sk` leaves something behind on purpose, which the ledger says (`graft`):
/// a row the source alone carries (#510 S1), a station a chain's later leg carries, whose rows
/// and unknowns `solid::family::part_of` frees on its own copy (#363 S4), or an unknown the source
/// declared (`Sketch::declared`, parameters.md).  Its freedoms are not the source's, by design;
/// its intrinsic rows still are.
fn leaves_behind(sk: &Sketch) -> bool {
    !sk.carried.is_empty() || sk.constraints.iter().any(|c| c.kind.of_source())
        || sk.free_vars.keys().any(|n| sk.declared.contains_key(n))
}

/// **A rebuild keeps every freedom** (#831): a rebuild that removes nothing, and a JSON round
/// trip of every document the JSON carries, have the source's freedoms and rank, and the same
/// intrinsic rows by owner and kind, renumbered.  Copying the model copies entities and their
/// fields and mints no row of its own (`Sketch::derive_all`), so a structural fact recorded only
/// as a row would be lost here, whoever adds the next one: the independent check on the
/// derivation.  A followed point travels unlowered (`graft`'s `curves`), so a document with one
/// is not solved here, and a document whose rebuild leaves something behind on purpose
/// (`leaves_behind`) is held to its intrinsic rows alone.
#[test]
fn a_rebuild_keeps_every_freedom() {
    let names = fixtures::example_names();
    let wrong: Vec<String> = par::map(&names, |name| keeps_every_freedom(name))
        .into_iter().flatten().collect();
    assert!(wrong.is_empty(), "{wrong:#?}");
}

/// What `a_rebuild_keeps_every_freedom` finds wrong with one document.
fn keeps_every_freedom(name: &str) -> Vec<String> {
    let Some(sk) = example(name) else { return Vec::new() };
    if sk.curve_defs.iter().any(|d| matches!(d.body, CurveBody::Followed(_))) {
        return Vec::new();
    }
    let mut wrong = Vec::new();
    let solved = !leaves_behind(&sk);
    let want = solved.then(|| freedoms(&sk));
    let rows = intrinsic_rows(&sk, &|e| Some(e));
    let (copy, r) = graft::rebuild(&sk, &|_| true, &[]);
    if intrinsic_rows(&copy, &|e| Some(e)) != intrinsic_rows(&sk, &|e| r.ent(e)) {
        wrong.push(format!("{name}: rebuilt, its intrinsic rows differ"));
    }
    if let Some(want) = want {
        let got = freedoms(&copy);
        if got != want {
            wrong.push(format!("{name}: rebuilt, (dof, rank) {got:?} of {want:?}"));
        }
    }
    if json_carries(&sk) {
        match gcs_core::io::from_json(&gcs_core::io::to_json(&sk)) {
            Err(e) => wrong.push(format!("{name}: its JSON reads back as {e}")),
            Ok(back) => {
                if intrinsic_rows(&back, &|e| Some(e)) != rows {
                    wrong.push(format!("{name}: read back from JSON, its intrinsic rows differ"));
                }
                if let Some(want) = want {
                    let got = freedoms(&back);
                    if got != want {
                        wrong.push(format!("{name}: read back from JSON, (dof, rank) {got:?} \
                                            of {want:?}"));
                    }
                }
            }
        }
    }
    wrong
}


/// Where `sk`'s stored rows of an owner are not the rows `rows_of` derives for it now: each owner
/// whose cache disagrees with its fields, by name.
fn stale_owners(sk: &Sketch) -> Vec<String> {
    // the twin a statement's planes feed is one statement (`CKind::attitude_twin`)
    let key = |c: &gcs_core::constraints::Constraint| {
        format!("{:?} {:?}", c.kind.attitude_twin(false), c.args)
    };
    let mut wrong = Vec::new();
    for o in sk.owners() {
        let mut want: Vec<String> = sk.rows_of(o).iter().map(key).collect();
        let mut have: Vec<String> = sk.constraints.iter()
            .filter(|c| c.origin == Origin::Intrinsic(o) && c.along.is_none())
            .map(key)
            .collect();
        want.sort();
        have.sort();
        if want != have {
            wrong.push(format!("{o:?} stores {have:?}, derives {want:?}"));
        }
    }
    wrong
}

/// **A part keeps its owners' rows** (#831): the rows stored for each owner are the rows its
/// fields imply — in every document of a sample of the corpus as elaborated, and in the part
/// (`Part::around_all`) taken from each of a sample of its entities, a drag's and a station's
/// copy.  A row is a cache of `Sketch::rows_of`, and never the one place a fact is kept.
#[test]
fn a_part_keeps_its_owners_rows() {
    let names: Vec<String> = fixtures::example_names().into_iter().step_by(5).collect();
    let wrong: Vec<String> = par::map(&names, |name| {
        let Some(sk) = example(name) else { return Vec::new() };
        let mut wrong: Vec<String> =
            stale_owners(&sk).into_iter().map(|w| format!("{name}: {w}")).collect();
        let all = entities(&sk);
        let step = all.len().div_ceil(6).max(1);
        for &e in all.iter().step_by(step) {
            let part = Part::around_all(&sk, &[e]);
            wrong.extend(stale_owners(&part.sketch).into_iter()
                .map(|w| format!("{name}, the part around {e:?}: {w}")));
        }
        wrong
    }).into_iter().flatten().collect();
    assert!(wrong.is_empty(), "{wrong:#?}");
}
