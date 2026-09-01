//! A drawing as SVG.
//!
//! It lives **here** rather than in the CLI for the reason callout layout does: an "export SVG"
//! button in the web app must not be a second implementation.  Everything it needs is already
//! the core's — a `Callout` is `solid`/`thin` segments, `arcs`, `arrows` and a label box with an
//! anchor and an angle; `curve::tessellate` gives a polyline; `Sketch::bbox` gives the extents a
//! `viewBox` wants; `style.rs` says what to stroke each thing with.
//!
//! **An SVG has no screen, so the export must choose a `unit`.**  `unit` is the world length of
//! one screen pixel, and everything drawn at a constant size goes through it — callout text and
//! arrowheads, `tessellate`'s flatness, and a sheet's dash lengths and stroke widths.  A page
//! width in pixels fixes it, and every constant size follows.
//!
//! The page's y runs downwards where the drawing's runs up, so world coordinates are converted
//! once, here, rather than by a group transform that would mirror every label.  That is the same
//! division `app/camera.ts` draws: one place writes a minus sign in front of a y.

use crate::callout::{self, Callout};
use crate::json::fmt_g;
use crate::model::{grow, EntKind, EntRef, Sketch};
use crate::style::Style;

/// White space round the drawing, in screen pixels.
const MARGIN_PX: f64 = 24.0;

/// The weight of geometry a sheet says nothing about.  The base sheet has no rule for unclassed
/// geometry, so each front end answers "what does a plain line look like" for itself and the two
/// must agree: `paint.ts`'s `strokeFor` holds the other copy.
const PLAIN_PX: f64 = 1.8;

/// Rim points for an ellipse.  A fixed count rather than `unit`-driven flatness: `ellipse.rs`
/// has no tessellator of its own yet, and this is the sweep every other consumer of it uses.

/// The ink for geometry a sheet says nothing about.  A drawing exported for print is black
/// unless the document says otherwise — the per-kind palette the app paints with is the *app's*
/// chrome, not the document's (spec §13.2).
const INK: &str = "#000000";

/// Render a solved sketch at a given page width, in pixels.
pub fn render(sk: &Sketch, width_px: f64) -> String {
    let width_px = width_px.max(16.0);
    // Each curve is swept **once**: its polyline fixes the page, and is then the thing drawn.
    // A curve's `bounds` *is* its polyline, and for a traced family that is a damped-Newton
    // march per point — asking for it twice was three quarters of an export on `gear_trace`.
    let polys: Vec<Vec<(f64, f64)>> = (0..sk.curves.len()).map(|i| sk.curve_polyline(i)).collect();
    // The geometry first, to fix `unit`; then the callouts, which are laid out *against* a unit
    // and may reach outside the geometry they measure.  `Sketch::drawn_bounds` is the primitives
    // — `callout::layout` measures the drawing by that same box, so a page sized by a second
    // answer would put the callouts somewhere the page is not — grown by what we swept.
    let mut geo = sk.drawn_bounds();
    for p in polys.iter().flatten() {
        grow(&mut geo, *p);
    }
    let span = (geo.2 - geo.0).max(geo.3 - geo.1).max(1e-9);
    let unit = span / (width_px - 2.0 * MARGIN_PX).max(1.0);
    let cs = callout::layout(sk, unit);

    let mut b = geo;
    for c in &cs {
        let segs = c.solid.iter().chain(&c.thin);
        for p in c.label.iter().copied().chain(segs.flat_map(|s| [s.0, s.1])) {
            grow(&mut b, p);
        }
        for a in &c.arcs {
            grow(&mut b, (a.c.0 - a.r, a.c.1 - a.r));
            grow(&mut b, (a.c.0 + a.r, a.c.1 + a.r));
        }
    }
    let m = MARGIN_PX * unit;
    let (x0, y0, x1, y1) = (b.0 - m, b.1 - m, b.2 + m, b.3 + m);
    let (w, h) = ((x1 - x0) / unit, (y1 - y0) / unit);
    // world -> page: the drawing's y runs up and the page's runs down
    let at = move |p: (f64, f64)| ((p.0 - x0) / unit, (y1 - p.1) / unit);

    let mut out = String::new();
    out.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}\" height=\"{}\" \
         viewBox=\"0 0 {} {}\">\n",
        n(w),
        n(h),
        n(w),
        n(h)
    ));
    out.push_str("<g fill=\"none\" stroke-linecap=\"round\">\n");
    for e in sk.drawn() {
        entity(&mut out, sk, e, unit, &at, &polys);
    }
    out.push_str("</g>\n");
    for c in &cs {
        dimension(&mut out, sk, c, unit, &at);
    }
    out.push_str("</svg>\n");
    out
}

/// A number, as short as it can be written without changing what it is.
fn n(v: f64) -> String {
    fmt_g(v, 6)
}

/// `stroke`, `stroke-width` and `stroke-dasharray` from a resolved style — the core's answer,
/// which is the whole point of the sheet: two front ends draw one drawing alike.
fn stroke(s: &Style) -> String {
    let mut out = format!(
        " stroke=\"{}\" stroke-width=\"{}\"",
        s.color.as_deref().unwrap_or(INK),
        n(s.width.unwrap_or(PLAIN_PX))
    );
    if let Some(d) = s.dash.as_ref().filter(|d| !d.is_empty()) {
        let parts: Vec<String> = d.iter().map(|&v| n(v)).collect();
        out.push_str(&format!(" stroke-dasharray=\"{}\"", parts.join(" ")));
    }
    out
}

/// `x,y x,y …` — a point list in page coordinates, which is what every `points=` attribute in
/// this file wants.  Written once so the number format is stated once, and written *into* the
/// output: a tessellated spline is hundreds of points, and a `String` per coordinate plus a
/// `Vec` to join them is four allocations each for a value that is appended and forgotten.
fn points(out: &mut String, pts: &[(f64, f64)], at: &dyn Fn((f64, f64)) -> (f64, f64)) {
    for (k, &p) in pts.iter().enumerate() {
        let (x, y) = at(p);
        if k > 0 {
            out.push(' ');
        }
        out.push_str(&n(x));
        out.push(',');
        out.push_str(&n(y));
    }
}

fn poly(out: &mut String, pts: &[(f64, f64)], at: &dyn Fn((f64, f64)) -> (f64, f64), attrs: &str) {
    if pts.len() < 2 {
        return;
    }
    out.push_str("<polyline points=\"");
    points(out, pts, at);
    out.push_str(&format!("\"{attrs}/>\n"));
}

/// One arc, as an SVG elliptical-arc path.  `a1 > a0` is counterclockwise in the drawing, which
/// on a page whose y runs down is the *sweep-flag 0* direction.
fn arc_path(
    out: &mut String,
    c: (f64, f64),
    r: f64,
    a0: f64,
    a1: f64,
    at: &dyn Fn((f64, f64)) -> (f64, f64),
    attrs: &str,
) {
    let (s, e) = (
        at((c.0 + r * a0.cos(), c.1 + r * a0.sin())),
        at((c.0 + r * a1.cos(), c.1 + r * a1.sin())),
    );
    let sweep = (a1 - a0).abs();
    let large = usize::from(sweep > std::f64::consts::PI);
    let ccw = a1 > a0;
    // the radius in page units: the transform is a uniform scale, so a length converts like a
    // point does, and asking `at` twice is one place fewer for the scale to be written down
    let rr = (at((c.0 + r, c.1)).0 - at(c).0).abs();
    out.push_str(&format!(
        "<path d=\"M {} {} A {} {} 0 {large} {} {} {}\"{attrs}/>\n",
        n(s.0),
        n(s.1),
        n(rr),
        n(rr),
        usize::from(!ccw),
        n(e.0),
        n(e.1)
    ));
}

fn entity(
    out: &mut String,
    sk: &Sketch,
    e: EntRef,
    unit: f64,
    at: &dyn Fn((f64, f64)) -> (f64, f64),
    polys: &[Vec<(f64, f64)>],
) {
    // resolved inside the arms that read it: a point is drawn in `INK` and a frame not at all,
    // and points are the majority of a sketch's entities — every line, circle and arc mints two
    // or three.  Resolving a style for each of them is a sheet cascade and a format for nothing.
    let ink = || stroke(&sk.style_of(e));
    let i = e.i();
    match e.kind {
        // a point is a place, not a stroke: a small filled dot, in the ink the sheet has no
        // rule for
        EntKind::Point => {
            let (x, y) = at(sk.point_xy(i));
            out.push_str(&format!(
                "<circle cx=\"{}\" cy=\"{}\" r=\"2\" fill=\"{INK}\"/>\n",
                n(x),
                n(y)
            ));
        }
        EntKind::Line => {
            let l = &sk.lines[i];
            let (a, b) = (at(sk.point_xy(l.p1 as usize)), at(sk.point_xy(l.p2 as usize)));
            let s = ink();
            out.push_str(&format!(
                "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"{s}/>\n",
                n(a.0),
                n(a.1),
                n(b.0),
                n(b.1)
            ));
        }
        EntKind::Circle => {
            let c = &sk.circles[i];
            let (cx, cy) = at(sk.point_xy(c.center as usize));
            let r = sk.params[c.radius as usize].value.abs() / unit;
            let s = ink();
            out.push_str(&format!(
                "<circle cx=\"{}\" cy=\"{}\" r=\"{}\"{s}/>\n",
                n(cx),
                n(cy),
                n(r)
            ));
        }
        EntKind::Arc => {
            let a = &sk.arcs[i];
            let c = sk.point_xy(a.center as usize);
            let r = sk.params[a.radius as usize].value.abs();
            let (a0, a1) = sk.arc_angles(i);
            arc_path(out, c, r, a0, a1, at, &ink());
        }
        EntKind::Ellipse => {
            // sampled rather than written as an SVG ellipse: the rotation would be a second
            // place the major axis's angle is worked out, and `ellipse.rs` already owns it —
            // which is `ellipse::sample`, so the rim is walked there and not here
            poly(out, &crate::ellipse::rim(sk, i), at, &ink());
        }
        EntKind::Spline => poly(out, &crate::curve::tessellate(sk, i, unit), at, &ink()),
        // the polyline `render` already swept to size the page
        EntKind::Curve => poly(out, &polys[i], at, &ink()),
        // a frame is a datum: it draws nothing, and its points are the click targets
        EntKind::Frame => {}
        // a plane is a datum with a glyph: its chord, and a tick along the frame's own y-axis
        // saying which side the view's second coordinate grows to.  No name — a `Sketch` holds
        // no source names, so the label is the app's to draw.
        EntKind::Plane => {
            let s = ink();
            for (from, to) in crate::plane::glyph(sk, i, unit) {
                let (a, b) = (at(from), at(to));
                out.push_str(&format!(
                    "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"{s}/>\n",
                    n(a.0),
                    n(a.1),
                    n(b.0),
                    n(b.1)
                ));
            }
        }
    }
}

fn dimension(
    out: &mut String,
    sk: &Sketch,
    c: &Callout,
    unit: f64,
    at: &dyn Fn((f64, f64)) -> (f64, f64),
) {
    // which classes a callout carries and how they compose is `callout::ink`'s: a rule each front
    // end resolved for itself is a rule each front end gets slightly differently
    let (ink, thin) = callout::ink(sk, c);
    let col = ink.color.clone().unwrap_or_else(|| INK.to_string());
    // one attribute string per style, not one per segment: a callout's parts are all stroked
    // the same two ways, and `stroke` allocates
    let (thin, solid) = (stroke(&thin), stroke(&Style { dash: None, ..ink }));
    for s in &c.thin {
        poly(out, &[s.0, s.1], at, &thin);
    }
    for s in &c.solid {
        poly(out, &[s.0, s.1], at, &solid);
    }
    for a in &c.arcs {
        arc_path(out, a.c, a.r, a.a0, a.a1, at, &solid);
    }
    for a in &c.arrows {
        // a filled triangle, in the shape the layout hands over — the core lays the figure out
        // and this only strokes it, the same bargain the label's box is drawn under
        out.push_str("<polygon points=\"");
        points(out, &callout::head(a, unit), at);
        out.push_str(&format!("\" fill=\"{col}\"/>\n"));
    }
    // the number, over a box that clears what is behind it — one dimension's number must not rub
    // out the next one's line, which is why the box is part of the layout
    out.push_str("<polygon points=\"");
    points(out, &c.label, at);
    out.push_str("\" fill=\"#ffffff\"/>\n");
    let (ax, ay) = at(c.anchor);
    // the layout turns counterclockwise; the page turns the other way, its y pointing down
    out.push_str(&format!(
        "<text x=\"0\" y=\"0\" transform=\"translate({} {}) rotate({})\" fill=\"{col}\" \
         font-family=\"system-ui, sans-serif\" font-size=\"{}\" text-anchor=\"middle\" \
         dominant-baseline=\"middle\">{}</text>\n",
        n(ax),
        n(ay),
        n(-c.angle.to_degrees()),
        n(callout::FONT_PX),
        escape(&c.text)
    ));
}

fn escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}
