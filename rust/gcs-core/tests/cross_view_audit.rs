//! Every relation in the example corpus whose operands carry different memberships
//! (`docs/spatial-constraints-plan.md`, the P0 audit): the role rule reads each one as sheet
//! layout or as an ordinate in one view, never as a relation in space.  The gate holds the corpus
//! to that; the audit that found it is kept as a report, ignored:
//! `cargo test cross_view_audit -- --ignored --nocapture`.
use gcs_core::constraints::{Arg, CKind};
use gcs_core::model::{EntKind, EntRef, Sketch};
use gcs_core::program::Elaborated;
use std::path::{Path, PathBuf};

fn sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let mut entries: Vec<_> = std::fs::read_dir(dir).unwrap().flatten().map(|e| e.path()).collect();
    entries.sort();
    for p in entries {
        if p.is_dir() { sources(&p, out) }
        else if p.extension().is_some_and(|e| e == "sv") { out.push(p) }
    }
}

/// Every `.sv` in `rust/examples`, by its path there, elaborated with its modules — `None` for
/// one that does not parse.
fn corpus() -> Vec<(String, Option<Elaborated>)> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let mut files = Vec::new();
    sources(&root, &mut files);
    files.into_iter().map(|path| {
        let name = path.strip_prefix(&root).unwrap().to_string_lossy().to_string();
        let text = std::fs::read_to_string(&path).unwrap();
        let (mut prog, errs) = gcs_core::syntax::parse(&text);
        if !errs.is_empty() { return (name, None); }
        let dir = path.parent().unwrap().to_path_buf();
        let mut resolve = |m: &str| -> Option<String> {
            // beside the document, then its ancestors up to the examples, then the library
            gcs_core::modules::search_paths(m, &name).iter()
                .find_map(|rel| std::fs::read_to_string(dir.join(rel)).ok())
                .or_else(|| gcs_core::library::resolve(m))
        };
        let _ = gcs_core::modules::link(&mut prog, &mut resolve);
        let e = gcs_core::program::elaborate(&prog);
        (name, Some(e))
    }).collect()
}

/// The points an operand is drawn by: itself, or a line's, circle's or arc's children.
fn points(sk: &Sketch, e: EntRef) -> Vec<usize> {
    match e.kind {
        EntKind::Point => vec![e.i()],
        EntKind::Line | EntKind::Circle | EntKind::Arc => sk.children(e).iter()
            .filter(|c| c.kind == EntKind::Point).map(|c| c.i()).collect(),
        _ => Vec::new(),
    }
}

#[test]
#[ignore]
fn cross_view_audit() {
    let (mut total, mut docs, mut spatial) = (0, 0, 0);
    for (name, e) in corpus() {
        let Some(e) = e else { println!("{name}: does not parse"); continue };
        if !e.ok() { println!("{name}: does not elaborate"); continue; }
        let sk = &e.sketch;
        let view = |p: usize| sk.plane_of(p).map_or("page".to_string(), |i| {
            e.map.name_of(EntRef::plane(i)).cloned().unwrap_or(format!("plane#{i}"))
        });
        // the planes a point places on the sheet, as its origin or toward
        let places = |p: usize| -> Vec<usize> { (0..sk.planes.len()).filter(|&i| {
            let f = &sk.planes[i].frame;
            f.origin as usize == p || f.toward as usize == p
        }).collect() };
        let mut found = Vec::new();
        for c in sk.user_constraints() {
            if c.kind == CKind::Project { continue; }
            let pts: Vec<usize> = c.args.iter()
                .filter_map(|a| if let Arg::Ent(r) = a { Some(*r) } else { None })
                .flat_map(|r| points(sk, r)).collect();
            let mut views: Vec<Option<usize>> = pts.iter().map(|&p| sk.plane_of(p)).collect();
            views.sort();
            views.dedup();
            if views.len() < 2 { continue; }
            // every point places some view: a statement of where views sit on the sheet
            let layout = pts.iter().all(|&p| !places(p).is_empty());
            // one view, and every page point is that view's own datum: an ordinate measured
            // from the datum the view is drawn from
            let drawn: Vec<usize> = views.iter().flatten().copied().collect();
            let own = drawn.len() == 1 && pts.iter().filter(|&&p| sk.plane_of(p).is_none())
                .all(|&p| places(p).contains(&drawn[0]));
            let tag = if layout { "layout" } else if own { "own-datum" } else { "SPACE" };
            let label = |p: usize| {
                let v = view(p);
                if places(p).is_empty() { v } else { format!("{v} (datum of {})", places(p).iter()
                    .map(|&i| e.map.name_of(EntRef::plane(i)).cloned().unwrap_or(format!("#{i}")))
                    .collect::<Vec<_>>().join("/")) }
            };
            let mut labels: Vec<String> = pts.iter().map(|&p| label(p)).collect();
            labels.sort();
            labels.dedup();
            let said = gcs_core::io::describe_with(c, &|r| e.map.name_of(r).cloned());
            found.push((tag, format!("    {tag}: {said}   [{}]", labels.join(", "))));
        }
        if !found.is_empty() {
            docs += 1;
            total += found.len();
            let n = |t: &str| found.iter().filter(|f| f.0 == t).count();
            spatial += n("SPACE");
            println!("{name}: {} across views ({} layout, {} own-datum, {} SPACE)", found.len(),
                n("layout"), n("own-datum"), n("SPACE"));
            for f in found { println!("{}", f.1); }
        }
    }
    println!("{total} relations across views in {docs} documents, {spatial} neither layout nor \
        an ordinate from the view's own datum");
}

/// The documents written to be read in space — the app's spatial demos — which the audit's
/// corpus predates: each of them states relations across views on purpose.
const IN_SPACE: &[&str] = &["skew_axes.sv", "hypoid_pitch_cones.sv", "sphere_cone_cylinder.sv"];

/// The spiral bevel's step-by-step layout (`spiral_bevel/layout.sv` and its modules), written
/// after the audit in views folded from the pitch plane: its relations across views are read
/// in space where a step draws in two views, and its page-only previews (the blanks' limits, a
/// rack section) do not, so it is held to neither reading.
const LAYOUT: &[&str] = &["spiral_bevel/views.sv", "spiral_bevel/pitch/", "spiral_bevel/crown/",
    "spiral_bevel/blank/", "spiral_bevel/generation.sv", "spiral_bevel/layout.sv",
    "spiral_bevel/members.sv", "spiral_bevel/gears_layout.sv"];

/// **The gate the audit became**: with a relation across views meaning space, every
/// one of the corpus's 215 relations whose points carry different memberships still reads the
/// 2D kind it always did — the role rule reads each as sheet layout or as an ordinate in one view
/// — and no document is refused for one.  The spatial demos are the exception, and they are
/// held to the opposite: each does read in space.  The spiral bevel's layout is written in
/// space from the start and is left out (`LAYOUT`).
#[test]
fn every_cross_membership_relation_in_the_corpus_keeps_its_reading() {
    let mut across = 0;
    for (name, e) in corpus() {
        let e = e.unwrap_or_else(|| panic!("{name} does not parse"));
        assert!(!e.diags.iter().any(|d| d.code.as_str() == "E062"),
                "{name}: {:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
        if !e.ok() { continue; }
        let sk = &e.sketch;
        if IN_SPACE.contains(&name.as_str()) {
            assert!(sk.user_constraints().iter().any(|c| c.kind.spatial()), "{name} reads in space");
            continue;
        }
        if LAYOUT.iter().any(|l| name.starts_with(l)) { continue; }
        for c in sk.user_constraints() {
            assert!(!c.kind.spatial() || c.kind == CKind::ProjectSolved,
                    "{name}: {} reads in space", gcs_core::io::describe_with(c, &|r| e.map.name_of(r).cloned()));
            if c.kind == CKind::Project { continue; }
            let pts: Vec<usize> = c.args.iter()
                .filter_map(|a| if let Arg::Ent(r) = a { Some(*r) } else { None })
                .flat_map(|r| points(sk, r)).collect();
            let mut views: Vec<Option<usize>> = pts.iter().map(|&p| sk.plane_of(p)).collect();
            views.sort();
            views.dedup();
            if views.len() < 2 { continue; }
            across += 1;
            // and the reading the dispatch made of it: one view, or the page
            let mut read = gcs_core::program::reading_views(sk, &pts);
            read.sort();
            read.dedup();
            assert_eq!(read.len(), 1, "{name}: {} reads across views",
                       gcs_core::io::describe_with(c, &|r| e.map.name_of(r).cloned()));
        }
    }
    assert_eq!(across, 215, "the P0 audit's count");
}
