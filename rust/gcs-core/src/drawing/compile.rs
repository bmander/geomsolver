//! Host-independent loading and solving. File I/O belongs to the CLI/browser callback.
use super::*;
use std::collections::{BTreeMap, BTreeSet};

/// Compile a drawing and its model sources. `load(path, from)` resolves a path relative to
/// the source that imports it and returns its canonical identity and text. Canonical identities
/// make import cycles detectable and repeated model imports share one solved snapshot.
pub fn compile(
    text: &str,
    source: &str,
    sheet: Option<&str>,
    load: &mut dyn FnMut(&str, &str) -> Option<(String, String)>,
) -> Result<String, Error> {
    let mut doc = parse(text)?;
    let mut active = BTreeSet::new();
    let mut count = 0;
    styles(&mut doc, source, load, &mut active, &mut count)?;
    let mut solved = BTreeMap::new();
    let mut aliases = Vec::new();
    for m in &doc.models {
        let (path, text) = load(&m.path, source).ok_or_else(|| Error {
            span: m.span, message: format!("cannot load model `{}` from `{source}`", m.path),
        })?;
        if !solved.contains_key(&path) {
            let fail = |message: String| Error { span: m.span, message: format!("model `{path}`: {message}") };
            let (mut p, errs) = crate::syntax::parse(&text);
            if let Some(e) = errs.first() { return Err(fail(e.message.clone())) }
            let errs = crate::modules::link(&mut p, &mut |name| {
                load(&format!("{}.sv", name.replace('.', "/")), &path).map(|(_, t)| t)
                    .or_else(|| crate::library::resolve(name))
            });
            if let Some(e) = errs.iter().find(|e| e.severity() == crate::program::Severity::Error) {
                return Err(fail(e.message.clone()));
            }
            let mut e = crate::program::elaborate(&p);
            if let Some(d) = e.errors().next() { return Err(fail(d.message.clone())) }
            let r = crate::solve::solve(&mut e.sketch, crate::solve::SolveOpts::default());
            if !r.success { return Err(fail(format!("did not solve: {}", r.message))) }
            if let Some(d) = crate::program::solid_diagnostics(&e.sketch, &e.map).first() {
                return Err(fail(d.message.clone()));
            }
            solved.insert(path.clone(), e);
        }
        aliases.push((m.name.clone(), path));
    }
    let models = aliases.iter().map(|(name, path)| {
        let e = &solved[path];
        (name.clone(), Model { sketch: &e.sketch, names: &e.map })
    }).collect();
    render(&doc, &models, sheet)
}

fn styles(doc: &mut Document, source: &str,
    load: &mut dyn FnMut(&str, &str) -> Option<(String, String)>,
    active: &mut BTreeSet<String>, count: &mut usize) -> Result<(), Error>
{
    if active.len() >= 32 || *count >= 1024 {
        return Err(Error { span: Span::default(), message: "drawing import limit exceeded".into() });
    }
    *count += 1;
    active.insert(source.into());
    let mut imported = Vec::new();
    for path in &doc.imports {
        let (identity, text) = load(path, source).ok_or_else(|| Error {
            span: Span::default(), message: format!("cannot load style import `{path}` from `{source}`"),
        })?;
        if active.contains(&identity) {
            return Err(Error { span: Span::default(), message: format!("drawing import cycle at `{identity}`") });
        }
        let mut child = parse(&text).map_err(|e| Error {
            span: Span::default(), message: format!("style import `{identity}`: {}", e.message),
        })?;
        styles(&mut child, &identity, load, active, count)?;
        if !child.models.is_empty() || !child.sheets.is_empty() {
            return Err(Error { span: Span::default(), message: format!("`{identity}` must contain only styles and style imports") });
        }
        imported.extend(child.styles);
    }
    imported.append(&mut doc.styles);
    doc.styles = imported;
    active.remove(source);
    Ok(())
}
