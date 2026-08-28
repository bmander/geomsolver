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
use crate::model::{EntKind, EntRef, Sketch};
use crate::style::Style;

/// White space round the drawing, in screen pixels.
const MARGIN_PX: f64 = 24.0;

/// The ink for geometry a sheet says nothing about.  A drawing exported for print is black
/// unless the document says otherwise — the per-kind palette the app paints with is the *app's*
/// chrome, not the document's (spec §13.2).
const INK: &str = "#000000";

/// Render a solved sketch at a given page width, in pixels.
pub fn render(sk: &Sketch, width_px: f64) -> String {
    let width_px = width_px.max(16.0);
    // the geometry first, to fix `unit`; then the callouts, which are laid out *against* a unit
    // and may reach outside the geometry they measure
    let geo = drawn_bbox(sk);
    let span = (geo.2 - geo.0).max(geo.3 - geo.1).max(1e-9);
    let unit = span / (width_px - 2.0 * MARGIN_PX).max(1.0);
    let cs = callout::layout(sk, unit);

    let mut b = geo;
    for c in &cs {
        for p in c.label.iter().chain(c.solid.iter().flat_map(|s| [&s.0, &s.1])) {
            grow(&mut b, *p);
        }
        for s in &c.thin {
            grow(&mut b, s.0);
            grow(&mut b, s.1);
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
    for e in drawn(sk) {
        entity(&mut out, sk, e, unit, &at);
    }
    out.push_str("</g>\n");
    for c in &cs {
        dimension(&mut out, sk, c, unit, &at);
    }
    out.push_str("</svg>\n");
    out
}

/// Everything with something to draw.  `Sketch::primitives` stops short of curves — a curve is
/// written over the other kinds and is built and grafted after them — so they are added here,
/// which is the only place in this file that has to know the difference.
fn drawn(sk: &Sketch) -> Vec<EntRef> {
    let mut v = sk.primitives();
    v.extend((0..sk.curves.len()).map(|i| EntRef::new(EntKind::Curve, i)));
    v
}

/// The extents of what is actually *drawn* — a frame draws nothing and a point is a place, so
/// the box is over the geometry a reader sees.
fn drawn_bbox(sk: &Sketch) -> (f64, f64, f64, f64) {
    let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for e in drawn(sk) {
        if e.kind == EntKind::Frame {
            continue;
        }
        let (a, c, d, f) = sk.bounds(e);
        grow(&mut b, (a, c));
        grow(&mut b, (d, f));
    }
    if !b.0.is_finite() {
        return (0.0, 0.0, 1.0, 1.0);
    }
    b
}

fn grow(b: &mut (f64, f64, f64, f64), p: (f64, f64)) {
    b.0 = b.0.min(p.0);
    b.1 = b.1.min(p.1);
    b.2 = b.2.max(p.0);
    b.3 = b.3.max(p.1);
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
        n(s.width.unwrap_or(1.4))
    );
    if let Some(d) = s.dash.as_ref().filter(|d| !d.is_empty()) {
        let parts: Vec<String> = d.iter().map(|&v| n(v)).collect();
        out.push_str(&format!(" stroke-dasharray=\"{}\"", parts.join(" ")));
    }
    out
}

fn poly(out: &mut String, pts: &[(f64, f64)], at: &dyn Fn((f64, f64)) -> (f64, f64), attrs: &str) {
    if pts.len() < 2 {
        return;
    }
    let parts: Vec<String> = pts
        .iter()
        .map(|&p| {
            let (x, y) = at(p);
            format!("{},{}", n(x), n(y))
        })
        .collect();
    out.push_str(&format!("<polyline points=\"{}\"{attrs}/>\n", parts.join(" ")));
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
) {
    let s = stroke(&sk.style_of(e));
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
            arc_path(out, c, r, a0, a1, at, &s);
        }
        EntKind::Ellipse => {
            // sampled rather than written as an SVG ellipse: the rotation would be a second
            // place the major axis's angle is worked out, and `ellipse.rs` already owns it
            let el = &sk.ellipses[i];
            let c = sk.point_xy(el.center as usize);
            let m = sk.point_xy(el.major as usize);
            let b = sk.params[el.minor as usize].value.abs();
            let (dx, dy) = (m.0 - c.0, m.1 - c.1);
            let a = dx.hypot(dy).max(1e-12);
            let (ux, uy) = (dx / a, dy / a);
            let pts: Vec<(f64, f64)> = (0..=180)
                .map(|k| {
                    let t = std::f64::consts::TAU * k as f64 / 180.0;
                    let (p, q) = (a * t.cos(), b * t.sin());
                    (c.0 + p * ux - q * uy, c.1 + p * uy + q * ux)
                })
                .collect();
            poly(out, &pts, at, &s);
        }
        EntKind::Spline => poly(out, &crate::curve::tessellate(sk, i, unit), at, &s),
        EntKind::Curve => poly(out, &sk.curve_polyline(i), at, &s),
        // a frame is a datum: it draws nothing, and its points are the click targets
        EntKind::Frame => {}
    }
}

fn dimension(
    out: &mut String,
    sk: &Sketch,
    c: &Callout,
    _unit: f64,
    at: &dyn Fn((f64, f64)) -> (f64, f64),
) {
    let claimed = sk.constraint(c.id).map(|k| k.claim).unwrap_or(false);
    // a reference dimension *is* a dimension: the shared rule, and then the one that says how it
    // differs.  Asked for `reference` alone it would take neither the shared weight nor whatever
    // the document said about `.dimension`, which is `paint.ts`'s reason for the same list.
    let ink = sk.style_named(if claimed { "dimension reference" } else { "dimension" });
    let col = ink.color.clone().unwrap_or_else(|| INK.to_string());
    let thin = {
        let mut t = sk.style_named("extension");
        t.color = Some(col.clone());
        t.width = t.width.or(ink.width);
        t
    };
    let solid = Style { dash: None, ..ink.clone() };
    for s in &c.thin {
        poly(out, &[s.0, s.1], at, &stroke(&thin));
    }
    for s in &c.solid {
        poly(out, &[s.0, s.1], at, &stroke(&solid));
    }
    for a in &c.arcs {
        arc_path(out, a.c, a.r, a.a0, a.a1, at, &stroke(&solid));
    }
    for a in &c.arrows {
        // the head is a filled triangle: the tip, and two barbs a fixed number of pixels back
        let len = callout::ARROW_PX * _unit;
        let (bx, by) = (a.at.0 - a.dir.0 * len, a.at.1 - a.dir.1 * len);
        let (nx, ny) = (-a.dir.1 * len * callout::BARB, a.dir.0 * len * callout::BARB);
        let pts = [a.at, (bx + nx, by + ny), (bx - nx, by - ny)];
        let parts: Vec<String> = pts
            .iter()
            .map(|&p| {
                let (x, y) = at(p);
                format!("{},{}", n(x), n(y))
            })
            .collect();
        out.push_str(&format!(
            "<polygon points=\"{}\" fill=\"{col}\"/>\n",
            parts.join(" ")
        ));
    }
    // the number, over a box that clears what is behind it — one dimension's number must not rub
    // out the next one's line, which is why the box is part of the layout
    let box_pts: Vec<String> = c
        .label
        .iter()
        .map(|&p| {
            let (x, y) = at(p);
            format!("{},{}", n(x), n(y))
        })
        .collect();
    out.push_str(&format!(
        "<polygon points=\"{}\" fill=\"#ffffff\"/>\n",
        box_pts.join(" ")
    ));
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
