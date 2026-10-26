//! **The rebuild walk** (`graft`): deletion, copying, pasting and a drag's part are one walk
//! whose answer is a `Renumbering` — where each entity and each constraint of the source went.
//! The walk's rules for what an entity cannot be without are the model's own (`remapped` on each
//! value), and these hold them to the document corpus: a rebuild that removes nothing keeps
//! everything where it was, what a removal takes with it is what `Sketch::children` says depends
//! on what was removed, and the things the walk says as a whole — the unit a paste lands in, the
//! pictures, the numbers a drag writes back — come out as the document has them.
use std::path::{Path, PathBuf};

use gcs_core::constraints::CKind;
use gcs_core::graft::{self, Part};
use gcs_core::model::{DerivedE, EntKind, EntRef, RegionShape, SolidDef, Sketch, Target};
use gcs_core::units::Units;
use gcs_core::{library, modules, par, program, syntax};

fn examples_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples")
}

/// The example at `rel` under `rust/examples/`, elaborated and not solved, its modules read as
/// `solventc` reads them — `None` for one that does not elaborate.
fn example(rel: &str) -> Option<Sketch> {
    let path = examples_root().join(rel);
    let text = std::fs::read_to_string(&path).unwrap();
    let dir = path.parent().unwrap().to_path_buf();
    let mut beside = |m: &str| -> Option<String> {
        modules::search_paths(m, rel).iter()
            .find_map(|r| std::fs::read_to_string(dir.join(r)).ok())
            .or_else(|| library::resolve(m))
    };
    let (mut prog, errs) = syntax::parse(&text);
    if !errs.is_empty() {
        return None;
    }
    modules::link(&mut prog, &mut beside);
    let e = program::elaborate(&prog);
    e.ok().then_some(e.sketch)
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
    let mut files = Vec::new();
    crate::every_example::sources(&examples_root(), &mut files);
    let root = examples_root();
    let names: Vec<String> = files.iter()
        .map(|p| p.strip_prefix(&root).unwrap().to_string_lossy().replace('\\', "/"))
        .collect();
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
