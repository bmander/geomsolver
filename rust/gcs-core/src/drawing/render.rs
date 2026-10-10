#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::model::{EntKind, EntRef, Sketch};
use crate::plane::Basis;
use crate::program::SourceMap;
use crate::renderer::Renderer;
use crate::solid::{ApproximationPolicy, PageFrame};
use crate::style::{Classes, Display, Style};
use std::collections::{BTreeMap, BTreeSet};

/// A solved model and the public names from the same elaboration.
#[derive(Clone, Copy)]
pub struct Model<'a> {
    pub sketch: &'a Sketch,
    pub names: &'a SourceMap,
}

const PX_MM: f64 = 96.0 / 25.4;
fn error(span: Span, message: impl Into<String>) -> Error { Error { span, message: message.into() } }
fn n(v: f64) -> String { crate::json::fmt_g(v, 8) }
fn esc(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
        .replace('"', "&quot;").replace('\'', "&apos;")
}
fn split<'a>(target: &'a str, models: &BTreeMap<String, Model<'a>>, span: Span)
    -> Result<(&'a str, &'a str, Model<'a>), Error>
{
    if target.contains('#') {
        return Err(error(span, "drawing references use public names and indices, not internal statement identities"));
    }
    let (alias, path) = target.split_once('.').unwrap_or((target, ""));
    let model = models.get(alias).copied()
        .ok_or_else(|| error(span, format!("no model named `{alias}`")))?;
    Ok((alias, path, model))
}
/// The dimension `dimension m.NAME` asks for: the first, in statement order, of the document's
/// own dimensions whose number is written as the bare name — `distance(w)` over `param w := 60`.
fn written_as(sk: &Sketch, names: &SourceMap, name: &str) -> Option<u32> {
    sk.constraints.iter().filter_map(|c| {
        let site = names.site_of_constraint(c.id).filter(|s| s.path.0.is_empty())?;
        let (_, attr, _) = *c.dimensions().first()?;
        let text = c.written.as_deref().or_else(|| c.expr_text(attr))?;
        (text.trim() == name).then_some((site.span.lo, c.id))
    }).min().map(|(_, id)| id)
}
fn under(name: &str, prefix: &str) -> bool {
    name == prefix || name.strip_prefix(prefix).is_some_and(|s| s.starts_with('.') || s.starts_with('['))
}
fn entities(model: Model<'_>, path: &str) -> Vec<EntRef> {
    if model.names.is_private_path(path) { return Vec::new(); }
    if let Some(e) = model.names.entity_path(model.sketch, path) { return vec![e] }
    model.names.names.iter().filter(|(_, names)| path.is_empty() || names.iter().any(|n| under(&crate::program::public_path(n), path)))
        .map(|(&e, _)| e).collect()
}
fn basis(name: &str, models: &BTreeMap<String, Model<'_>>, span: Span) -> Result<Basis, Error> {
    let (u, v) = match name {
        "front" => ([1., 0., 0.], [0., 0., 1.]),
        "back" => ([-1., 0., 0.], [0., 0., 1.]),
        "right" => ([0., 1., 0.], [0., 0., 1.]),
        "left" => ([0., -1., 0.], [0., 0., 1.]),
        "top" => ([1., 0., 0.], [0., 1., 0.]),
        "bottom" => ([1., 0., 0.], [0., -1., 0.]),
        "isometric" => return Ok(Basis::explicit([1., -1., 0.], [1., 1., 2.])
            .expect("the built-in isometric axes span a plane")),
        _ => {
            let (_, path, model) = split(name, models, span)?;
            let e = model.names.entity_path(model.sketch, path).filter(|e| e.kind == EntKind::Plane)
                .ok_or_else(|| error(span, format!("`{name}` is not a model plane")))?;
            return Ok(model.sketch.basis(e.i()));
        }
    };
    Ok(Basis { u, v, o: [0.; 3] })
}

/// The plane a sketch view draws: a model plane by its path, and `front` the model's
/// `std.front` — or `None`, every plane's geometry, for a model that has no front plane (a
/// sketch built in code).
fn sketch_plane(name: &str, alias: &str, models: &BTreeMap<String, Model<'_>>, span: Span)
    -> Result<Option<usize>, Error>
{
    let target = if name == "front" { format!("{alias}.std.front") } else { name.to_string() };
    let (_, path, model) = split(&target, models, span)?;
    match model.names.entity_path(model.sketch, path).filter(|e| e.kind == EntKind::Plane) {
        Some(e) => Ok(Some(e.i())),
        None if name == "front" => Ok(None),
        None => Err(error(span, format!("`{name}` is not a model plane"))),
    }
}

fn point(model: Model<'_>, path: &str, span: Span) -> Result<((f64, f64), [f64; 3]), Error> {
    let e = model.names.entity_path(model.sketch, path).filter(|e| e.kind == EntKind::Point)
        .ok_or_else(|| error(span, format!("`{path}` is not a point")))?;
    let sk = model.sketch;
    Ok((sk.point_xy(e.i()), sk.world_point(e.i())))
}

/// Compile one sheet to SVG. This reads model snapshots, including names and measurements;
/// it never reparses, elaborates, or solves geometry. Paper coordinates are independent of
/// every plane/point in the model. SVG dimensions are physical mm, styles are CSS pixels.
pub fn render(doc: &Document, models: &BTreeMap<String, Model<'_>>, sheet: Option<&str>)
    -> Result<String, Error>
{
    let s = match sheet {
        Some(name) => doc.sheets.iter().find(|s| s.name == name)
            .ok_or_else(|| error(Span::default(), format!("no sheet named `{name}`")))?,
        None if doc.sheets.len() == 1 => &doc.sheets[0],
        None => return Err(error(Span::default(), "select a sheet when the drawing does not have exactly one")),
    };
    let rules: Vec<_> = doc.styles.iter().chain(&s.styles).collect();
    // An explicit selector is a reference, so misspellings cannot silently lose annotations.
    for r in &rules {
        if !r.selector.starts_with('.') {
            let (_, path, model) = split(&r.selector, models, r.span)?;
            if entities(model, path).is_empty() {
                return Err(error(r.span, format!("style selects no geometry: `{}`", r.selector)));
            }
        }
    }
    let mut out = format!("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}mm\" height=\"{}mm\" viewBox=\"0 0 {} {}\">\n<title>{}</title>\n",
        n(s.size.0), n(s.size.1), n(s.size.0 * PX_MM), n(s.size.1 * PX_MM), esc(&s.name));
    for v in &s.views {
        let (alias, path, model) = split(&v.target, models, v.span)?;
        let mut selected = entities(model, path);
        // a point drawn in several planes is selected with its twins, its images there (§6.7)
        let twins: Vec<EntRef> = model.sketch.twins.iter()
            .filter(|(&p, _)| selected.contains(&EntRef::point(p)))
            .flat_map(|(_, ts)| ts.iter().map(|&t| EntRef::point(t)))
            .collect();
        selected.extend(twins);
        if v.sketch {
            // a sketch draws one plane's geometry in that plane's own coordinates: `from` names
            // it, and the front plane is `std.front` where the model has one
            let plane = sketch_plane(&v.direction, alias, models, v.span)?;
            if let Some(p) = plane {
                let sk = model.sketch;
                let of = |e: EntRef| match e.kind {
                    EntKind::Curve => sk.curve_view(e.i()),
                    _ => crate::program::plane_of_entity(sk, e),
                };
                selected.retain(|&e| of(e) == Some(p));
            }
        }
        if selected.is_empty() { return Err(error(v.span, format!("no geometry named `{}`", v.target))) }
        let scale = v.scale.unwrap_or(s.scale) * model.sketch.units.length.map_or(1.0, |u| u.1) * PX_MM;
        if !scale.is_finite() || scale <= 0.0 { return Err(error(v.span, "view scale is out of range")) }
        let unit = 1.0 / scale;
        let at = |p: (f64, f64)| (v.at.0 * PX_MM + p.0 * scale, v.at.1 * PX_MM - p.1 * scale);
        // The existing sketch/callout painters take presentation on a Sketch. Adapt a private
        // snapshot; the model passed to us stays immutable, even when two drawings disagree.
        let mut sk = model.sketch.clone();
        sk.derived.clear(); sk.placements.clear(); sk.sheet.clear();
        sk.sheet.insert("point".into(), Style { display: Some(Display::None), ..Style::default() });
        sk.sheet.insert("construction".into(), Style { display: Some(Display::None), ..Style::default() });
        sk.sheet.insert("plane".into(), Style { display: Some(Display::None), ..Style::default() });
        sk.sheet.insert("_svd_unselected".into(), Style { display: Some(Display::None), ..Style::default() });
        for r in &rules {
            if let Some(class) = r.selector.strip_prefix('.') {
                sk.sheet.entry(class.into()).or_default().over(&r.style);
            }
        }
        let mut point_styles = BTreeMap::<EntRef, Style>::new();
        for (i, r) in rules.iter().enumerate().filter(|(_, r)| !r.selector.starts_with('.')) {
            let (owner, selector, _) = split(&r.selector, models, r.span)?;
            if owner != alias { continue }
            let class = format!("_svd{i}");
            sk.sheet.insert(class.clone(), r.style.clone());
            for e in entities(model, selector) {
                if e.kind == EntKind::Point {
                    point_styles.entry(e).or_insert_with(|| sk.style_of(e)).over(&r.style);
                } else { sk.set_class(e, &class, true); }
            }
        }
        out.push_str(&format!("<g id=\"{}\" fill=\"none\" stroke-linecap=\"round\">\n", esc(&v.name)));
        if v.sketch {
            let polys: Vec<_> = (0..sk.curves.len()).map(|i| sk.curve_polyline(i)).collect();
            let wanted: BTreeSet<_> = selected.iter().copied().collect();
            for e in sk.drawn().into_iter().filter(|e| wanted.contains(e)) {
                let style = point_styles.get(&e).cloned().unwrap_or_else(|| sk.style_of(e));
                if !style.shown() { continue }
                if e.kind == EntKind::Point {
                    let (x, y) = at(sk.point_xy(e.i()));
                    out.push_str(&format!("<circle cx=\"{}\" cy=\"{}\" r=\"2\" fill=\"{}\"/>\n",
                        n(x), n(y), esc(style.color.as_deref().unwrap_or("#000000"))));
                } else { crate::svg::entity(&mut out, &sk, e, unit, &at, &polys); }
            }
        } else {
            let frame = basis(&v.direction, models, v.span)?;
            let cut = v.cut.as_ref().map(|p| basis(p, models, v.span)).transpose()?;
            if cut.is_some_and(|c| crate::plane::dot(c.normal(), frame.normal()).abs() < 1.0 - 1e-9) {
                return Err(error(v.span, "a section must be viewed parallel to its cutting plane"));
            }
            let e = model.names.entity_path(model.sketch, path).filter(|e| e.kind == EntKind::Solid)
                .ok_or_else(|| error(v.span, "a solid view names a solid explicitly, such as `m.pis.body`; use `sketch` for 2D geometry"))?;
            {
                let solid = sk.evaluated_solid(e.i(), ApproximationPolicy::from_unit(unit))
                    .map_err(|m| error(v.span, m))?;
                let picture = Renderer::prepare(&solid).project(crate::renderer::View {
                    frame: PageFrame::new(frame), section: cut,
                });
                for stroke in picture.strokes {
                    let mut classes = Classes::one(if stroke.hidden { "hidden" } else { "visible" });
                    if cut.is_some() && !stroke.hidden { classes.0.push("section".into()); }
                    classes.0.extend(sk.class_of(e).0);
                    let style = crate::style::resolve(&sk.sheet, &classes);
                    if !style.shown() { continue }
                    let pts: Vec<_> = stroke.pts.into_iter().map(|p| { let p = at(p); format!("{},{}", n(p.0), n(p.1)) }).collect();
                    out.push_str(&format!("<polyline data-path=\"{}\" points=\"{}\" stroke=\"{}\" stroke-width=\"{}\"",
                        esc(&stroke.path), pts.join(" "), esc(style.color.as_deref().unwrap_or("#000000")), n(style.width.unwrap_or(1.8))));
                    if let Some(dash) = &style.dash {
                        out.push_str(&format!(" stroke-dasharray=\"{}\"", dash.iter().map(|v| n(*v)).collect::<Vec<_>>().join(" ")));
                    }
                    out.push_str("/>\n");
                }
                if v.dimensions {
                    let plane = sk.fixed_plane(frame, "_drawing");
                    sk.derived.push(crate::model::DerivedE { solid: e.idx, plane: Some(plane as u32),
                        at: None, dims: true, name: v.name.clone(), class: Classes::default() });
                }
            }
        }
        out.push_str("</g>\n");
        if !v.annotations.is_empty() && !v.sketch {
            return Err(error(v.span, "named constraint annotations belong in a sketch view; solid views support `dimensions in VIEW`"));
        }
        let mut chosen = BTreeSet::new();
        for a in &v.annotations {
            let (owner, path, _) = split(&a.target, models, a.span)?;
            if owner != alias { return Err(error(a.span, "dimension belongs to a different model")) }
            let id = written_as(&sk, model.names, path)
                .ok_or_else(|| error(a.span, format!("no dimension is written `{path}`")))?;
            chosen.insert(id);
            if let Some(p) = a.at { sk.placements.insert(id, p); }
        }
        for c in &mut sk.constraints {
            if !(chosen.contains(&c.id) || (v.sketch && v.dimensions
                && c.entities().iter().any(|e| selected.contains(e)))) {
                c.class.0.push("_svd_unselected".into());
            }
        }
        for c in crate::callout::layout(&sk, unit) {
            if crate::callout::ink(&sk, &c).0.dimensioned() {
                crate::svg::dimension(&mut out, &sk, &c, unit, &at);
            }
        }
        for m in &v.measurements {
            let (owner_a, a, _) = split(&m.points.0, models, m.span)?;
            let (owner_b, b, _) = split(&m.points.1, models, m.span)?;
            if owner_a != alias || owner_b != alias {
                return Err(error(m.span, "a measurement reads points from its view's model"));
            }
            let ((ax, ay), aw) = point(model, a, m.span)?;
            let ((bx, by), bw) = point(model, b, m.span)?;
            let value = (aw[0] - bw[0]).dhypot(aw[1] - bw[1]).dhypot(aw[2] - bw[2]);
            let (a, b) = if v.sketch { ((ax, ay), (bx, by)) } else {
                let frame = basis(&v.direction, models, v.span)?;
                (frame.view_coords(aw), frame.view_coords(bw))
            };
            if ((b.0 - a.0).dhypot(b.1 - a.1) - value).abs() > 1e-8 * value.max(1.0) {
                return Err(error(m.span, "this view foreshortens the measured distance; choose a view showing its true length"));
            }
            let c = crate::callout::measurement(&sk, unit, a, b, value, m.offset * PX_MM * unit)
                .ok_or_else(|| error(m.span, "a distance annotation needs two distinct projected points"))?;
            if crate::callout::ink(&sk, &c).0.dimensioned() {
                crate::svg::dimension(&mut out, &sk, &c, unit, &at);
            }
        }
    }
    for label in &s.labels {
        out.push_str(&format!("<text x=\"{}\" y=\"{}\" font-family=\"system-ui, sans-serif\" font-size=\"14\">{}</text>\n",
            n(label.at.0 * PX_MM), n(label.at.1 * PX_MM), esc(&label.text)));
    }
    out.push_str("</svg>\n");
    Ok(out)
}
