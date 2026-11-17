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
    compile_with(text, source, sheet, &[], load).map(|c| c.svg)
}

/// A drawing compiled: the sheet's SVG, and for each model import (by its alias, in the order
/// written) the values it was given over the ones its file writes, by the drawing's `with` and by
/// the run's (#940).
#[derive(Debug)]
pub struct Compiled {
    pub svg: String,
    pub rebound: Vec<(String, Vec<crate::edit::Rebound>)>,
}

/// **`compile`, with values given for the run** (`solventc drawing.svd --set g_wf=2in`, #940):
/// each `(name, value)` is given to every model the drawing imports that binds `name` at its top
/// level, read there as `--set` reads one for a model (`edit::rebind`) — except an import whose
/// own `with` gives that name, the drawing's word for that one model being the narrower.  A name
/// no imported model binds is refused.
pub fn compile_with(
    text: &str,
    source: &str,
    sheet: Option<&str>,
    set: &[(String, String)],
    load: &mut dyn FnMut(&str, &str) -> Option<(String, String)>,
) -> Result<Compiled, Error> {
    let mut doc = parse(text)?;
    let mut active = BTreeSet::new();
    let mut count = 0;
    styles(&mut doc, source, load, &mut active, &mut count)?;
    let mut solved = BTreeMap::new();
    let mut aliases = Vec::new();
    let mut rebounds = Vec::new();
    let mut taken = vec![false; set.len()];
    for m in &doc.models {
        let (path, text) = load(&m.path, source).ok_or_else(|| Error {
            span: m.span, message: format!("cannot load model `{}` from `{source}`", m.path),
        })?;
        // the values it is given: its own `with`, then each of the run's it binds and its `with`
        // does not give
        let mut given: Vec<(String, String)> =
            m.with.iter().map(|g| (g.name.clone(), g.text.clone())).collect();
        for g in &m.with {
            crate::edit::rebind(&text, &[(g.name.clone(), g.text.clone())])
                .map_err(|why| Error { span: g.span, message: format!("model `{path}`: {why}") })?;
        }
        for (k, (name, value)) in set.iter().enumerate() {
            if m.with.iter().any(|g| g.name == name.trim()) { taken[k] = true; continue }
            match crate::edit::rebind(&text, &[(name.clone(), value.clone())]) {
                Ok(_) => { taken[k] = true; given.push((name.clone(), value.clone())); }
                Err(why) if crate::edit::binds_no_value(&why) => {}
                Err(why) => return Err(Error { span: m.span,
                    message: format!("--set: model `{path}`: {why}") }),
            }
        }
        let (text, rebound) = crate::edit::rebind(&text, &given)
            .map_err(|why| Error { span: m.span, message: format!("model `{path}`: {why}") })?;
        // a model shown at an instant is a snapshot of its own, the same file's drawn at home and
        // stood there (#398 L); one given values, the same file's with them (#940)
        let mut key = match &m.at {
            Some(at) => format!("{path}\u{0}{}={}", at.name, at.text.trim()),
            None => path.clone(),
        };
        for r in &rebound {
            key.push_str(&format!("\u{0}{}:={}", r.name, r.value));
        }
        rebounds.push((m.name.clone(), rebound));
        if !solved.contains_key(&key) {
            let fail = |message: String| Error { span: m.span, message: format!("model `{path}`: {message}") };
            let (mut p, errs) = crate::syntax::parse(&text);
            if let Some(e) = errs.first() { return Err(fail(e.message.clone())) }
            let errs = crate::modules::link(&mut p, &mut |name| {
                crate::modules::search_paths(name, &path).iter()
                    .find_map(|candidate| load(candidate, &path).map(|(_, t)| t))
                    .or_else(|| crate::library::resolve(name))
            });
            if let Some(e) = errs.iter().find(|e| e.severity() == crate::program::Severity::Error) {
                return Err(fail(e.message.clone()));
            }
            let mut e = crate::program::elaborate(&p);
            if let Some(d) = e.errors().next() { return Err(fail(d.message.clone())) }
            let r = crate::solve::solve(&mut e.sketch, crate::solve::SolveOpts::default());
            if !r.success { return Err(fail(format!("did not solve: {}", r.message))) }
            // a swept object drawn from its exact surface where one builds, as the page draws it
            crate::brep::export::supply_exact(&e.sketch);
            let refused = crate::program::solid_diagnostics(&e.sketch, &e.map, &e.program);
            if let Some(d) = refused.into_iter()
                .find(|d| d.severity() == crate::program::Severity::Error) {
                return Err(fail(d.message));
            }
            if let Some(at) = &m.at {
                let fail = |message: String| Error {
                    span: at.span, message: format!("model `{path}` at `{}`: {message}", at.name),
                };
                let t = crate::follow::instant(&e.sketch, &at.name, &at.text).map_err(fail)?;
                crate::follow::stand(&mut e.sketch, &at.name, t).map_err(fail)?;
            }
            solved.insert(key.clone(), e);
        }
        aliases.push((m.name.clone(), key));
    }
    if let Some(k) = taken.iter().position(|t| !t) {
        return Err(Error { span: Span::default(), message: format!(
            "--set: no model this drawing imports binds a value `{}` at its top level",
            set[k].0.trim()) });
    }
    let models = aliases.iter().map(|(name, key)| {
        let e = &solved[key];
        (name.clone(), Model { sketch: &e.sketch, names: &e.map })
    }).collect();
    Ok(Compiled { svg: render(&doc, &models, sheet)?, rebound: rebounds })
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

/// **The colours the drawing `text` gives the parts of a model** (#677), for a host that has the
/// model solved already (the workspace, `solventc --step-assembly`): its rules after its style
/// imports (`use`), read as `fills` reads them for every name the drawing imports the model by —
/// the model whose canonical identity, as `load` gives it, is `model`.  Nothing where the drawing
/// does not import that model.  The drawing's sheets are not rendered and its other models not
/// loaded: what the model is drawn in on paper is the paper's.
pub fn colours(
    text: &str,
    source: &str,
    model_path: &str,
    model: Model<'_>,
    load: &mut dyn FnMut(&str, &str) -> Option<(String, String)>,
) -> Result<BTreeMap<usize, [u8; 3]>, Error> {
    let mut doc = parse(text)?;
    let mut active = BTreeSet::new();
    let mut count = 0;
    styles(&mut doc, source, load, &mut active, &mut count)?;
    let aliases: Vec<&str> = doc.models.iter()
        .filter(|m| load(&m.path, source).is_some_and(|(identity, _)| identity == model_path))
        .map(|m| m.name.as_str()).collect();
    if aliases.is_empty() { return Ok(BTreeMap::new()) }
    fills(&doc, &aliases, model)
}
