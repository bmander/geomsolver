//! The migration audit for spatial constraints (`docs/spatial-constraints-plan.md`, P0): every
//! relation in the example corpus whose operands are drawn in different views.  Today such a
//! relation is read on the page; once a relation across views means space, each one is either a
//! sheet-layout statement to rewrite on datum points or a genuine cross-view relation.  A report
//! and never a gate, so it is ignored: `cargo test cross_view_audit -- --ignored --nocapture`.
use gcs_core::constraints::{Arg, CKind};
use gcs_core::model::{EntKind, EntRef, Sketch};
use std::path::{Path, PathBuf};

fn sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let mut entries: Vec<_> = std::fs::read_dir(dir).unwrap().flatten().map(|e| e.path()).collect();
    entries.sort();
    for p in entries {
        if p.is_dir() { sources(&p, out) }
        else if p.extension().is_some_and(|e| e == "sv") { out.push(p) }
    }
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
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let mut files = Vec::new();
    sources(&root, &mut files);
    let (mut total, mut docs, mut spatial) = (0, 0, 0);
    for path in files {
        let name = path.strip_prefix(&root).unwrap().to_string_lossy().to_string();
        let text = std::fs::read_to_string(&path).unwrap();
        let (mut prog, errs) = gcs_core::syntax::parse(&text);
        if !errs.is_empty() { println!("{name}: does not parse"); continue; }
        let dir = path.parent().unwrap().to_path_buf();
        let mut resolve = |m: &str| -> Option<String> {
            // beside the document, then its ancestors up to the examples, then the library
            gcs_core::modules::search_paths(m, &name).iter()
                .find_map(|rel| std::fs::read_to_string(dir.join(rel)).ok())
                .or_else(|| gcs_core::library::resolve(m))
        };
        let _ = gcs_core::modules::link(&mut prog, &mut resolve);
        let e = gcs_core::program::elaborate(&prog);
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
