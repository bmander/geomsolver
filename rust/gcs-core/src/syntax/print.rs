//! Canonical source printing and editable declaration fragments.

use super::names::write_ref;
use super::{
    num, snake, Arg, CurveSpec, CurveTarget, Decl, InstArg, InstVal, Instance, Kid,
    KidSeed, OpArg, Program, Ref, Relation, Sense, Span, StmtKind, Sweep, Written,
};
use crate::constraints::{CKind, Fixity, SpecKind};
use crate::model::{EntKind, Field};
use crate::style::Style;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PrintError {
    pub construct: &'static str,
}

impl std::fmt::Display for PrintError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "canonical printing does not support {}; retain the original source text",
            self.construct
        )
    }
}

impl std::error::Error for PrintError {}

/// Export the flat subset and update component and statement spans.
/// This is canonical export, not source formatting; use `Program::text` to retain source.
/// Unsupported structure leaves the program unchanged.
pub fn render_flat(p: &mut Program) -> Result<&str, PrintError> {
    if p.preview.is_some() {
        return Err(PrintError { construct: "preview blocks" });
    }
    // a `use` is a line of its own; what it links is the host's, printed nowhere — the flat text
    // is unlinked, as a document is before its host links it, and read again it links again
    let own = |c: &super::Component| c.module.is_none();
    if p.components.iter().filter(|c| own(c)).count() > 1
        || p.components.iter().filter(|c| own(c)).any(|c| c.name.is_some() || !c.formals.is_empty())
    {
        return Err(PrintError { construct: "component definitions" });
    }
    if p.words.iter().any(|d| d.module.is_none()) {
        return Err(PrintError { construct: "relation word definitions" });
    }
    if p.interfaces.iter().any(|f| f.module.is_none()) {
        return Err(PrintError { construct: "interface definitions" });
    }
    for c in p.components.iter().filter(|c| own(c)) {
        for st in &c.body {
            printable(&st.kind)?;
            if matches!(st.kind, StmtKind::Instance(_)) {
                return Err(PrintError { construct: "component instances" });
            }
        }
    }
    let mut out = String::new();
    for u in &mut p.uses {
        let start = out.len();
        out.push_str(&format!("use {}", u.name));
        if !u.names.is_empty() {
            let names: Vec<&str> = u.names.iter().map(|n| n.text.as_str()).collect();
            out.push_str(&format!(" ({})", names.join(", ")));
        }
        u.span = Span::new(start, out.len());
        out.push('\n');
    }
    for comp in p.components.iter_mut().filter(|c| c.module.is_none()) {
        let lo = out.len();
        // a claim's statements are written in its block, after the rest (#460): the body is a
        // set, so the order printed is the order kept
        let blocks = claim_blocks(&mut comp.body);
        let loose = comp.body.len() - blocks.iter().sum::<usize>();
        // Separate declarations from the relations that follow them.
        let mut said_decl = false;
        for st in &mut comp.body[..loose] {
            let is_decl = matches!(st.kind, StmtKind::Decl(_));
            if said_decl && !is_decl {
                out.push('\n');
            }
            said_decl = is_decl;
            write_flat(&mut out, st);
            out.push('\n');
        }
        let mut at = loose;
        for n in blocks {
            out.push_str("claim {\n");
            for st in &mut comp.body[at..at + n] {
                out.push_str("  ");
                write_flat(&mut out, st);
                out.push('\n');
            }
            out.push_str("}\n");
            at += n;
        }
        comp.span = Span::new(lo, out.len());
    }
    p.components.retain(|c| c.module.is_none());
    p.words.clear();
    p.modules.clear();
    p.text = out;
    p.in_blocks.clear();
    Ok(&p.text)
}

/// One statement of the flat form, its span set to where it was written.
fn write_flat(out: &mut String, st: &mut super::Stmt) {
    let start = out.len();
    // a block's plane is said on each statement it gave one: the flat form has no blocks
    match &st.kind {
        StmtKind::Decl(d) if d.membership.plane().is_some() && d.membership.written().is_none() => {
            write_decl(out, d);
            out.push_str(" in ");
            write_ref(out, d.membership.plane().expect("just asked"));
        }
        k => write_stmt(out, k),
    }
    st.span = Span::new(start, out.len());
}

/// The `claim { … }` block a statement stands in (#460): a witness by its role, a relation by
/// its own mark — the same index either way, once the sketch is lifted.
fn in_claim(k: &StmtKind) -> Option<u32> {
    match k {
        StmtKind::Decl(d) => d.annotations.roles.witness,
        StmtKind::Relation(r) => r.witness.map(|w| w.0),
        _ => None,
    }
}

/// Put a body's `claim { … }` blocks last (#460), each its witnesses then its relations, and
/// say how many statements each holds.  The sort is stable, so everything else keeps its order.
fn claim_blocks(body: &mut [super::Stmt]) -> Vec<usize> {
    body.sort_by_key(|st| in_claim(&st.kind).map(|w| (w, !matches!(st.kind, StmtKind::Decl(_)))));
    let mut counts: Vec<(u32, usize)> = Vec::new();
    for w in body.iter().filter_map(|st| in_claim(&st.kind)) {
        match counts.last_mut() {
            Some((at, n)) if *at == w => *n += 1,
            _ => counts.push((w, 1)),
        }
    }
    counts.into_iter().map(|(_, n)| n).collect()
}

/// Append a supported declaration or statement fragment atomically.
pub fn write_stmt_to(out: &mut String, k: &StmtKind) -> Result<(), PrintError> {
    printable(k)?;
    // a claim's witness and what it claims of one stand in the claim's block (#460): printed
    // alone it would be written loose, where it restricts the drawing
    if in_claim(k).is_some() {
        return Err(PrintError { construct: "claim blocks" });
    }
    write_stmt(out, k);
    Ok(())
}

fn printable(k: &StmtKind) -> Result<(), PrintError> {
    match k {
        StmtKind::Chain(_) => Err(PrintError { construct: "named chains" }),
        // a round's arc is turned, seeded and its branch stated from its corner, which no
        // longhand says (#576)
        StmtKind::Decl(d) if d.round.is_some() => Err(PrintError { construct: "corner rounds" }),
        // a face carried by a map is a definition, written as one (#357)
        StmtKind::Block(b) if b.image.is_some() => Ok(()),
        StmtKind::Block(_) => Err(PrintError { construct: "repeat and cycle blocks" }),
        StmtKind::ClaimOver(_) => Err(PrintError { construct: "claim-over bodies" }),
        StmtKind::Set(_) => Err(PrintError { construct: "set literals" }),
        _ => Ok(()),
    }
}

fn write_stmt(out: &mut String, k: &StmtKind) {
    match k {
        StmtKind::Decl(d) => write_decl(out, d),
        StmtKind::Relation(r) => write_relation(out, r),
        StmtKind::Branch(b) => out.push_str(&format!("branch({}, {})", b.key, b.value)),
        StmtKind::Derived(d) => {
            if let Some(n) = d.name.written() {
                out.push_str(&n.text);
                out.push_str(" := ");
            }
            out.push_str(if d.dims {
                "dimensions"
            } else if d.at.is_some() {
                "section"
            } else {
                "view"
            });
            out.push('(');
            write_ref(out, &d.solid);
            if let Some(at) = &d.at {
                out.push_str(", at: ");
                write_ref(out, at);
            }
            out.push_str(") in ");
            write_ref(out, &d.plane);
            if !d.class.is_empty() {
                out.push_str(&format!(" class {}", d.class.0.join(" ")));
            }
        }
        StmtKind::SolidRel(r) => {
            write_ref(out, &r.what);
            out.push(' ');
            out.push_str(r.word.as_str());
            out.push(' ');
            write_ref(out, &r.body);
        }
        StmtKind::Instance(i) => {
            i.annotations.write(out);
            if !i.name.text.starts_with('#') {
                out.push_str(&format!("{} := ", i.name.text));
            }
            write_instance_call(out, i);
            // only a clause this statement wrote — `Membership::written` is the one guard
            if let Some(p) = i.membership.written() {
                out.push_str(" in ");
                write_ref(out, p);
            }
            if !i.class.is_empty() {
                out.push_str(" class ");
                out.push_str(&i.class.0.join(" "));
            }
            if let Some(m) = &i.mirror {
                out.push_str(&format!(" mirrored({})", m.plane.text));
            }
        }
        StmtKind::Param(p) => {
            if let Some(input) = &p.input {
                out.push_str("param ");
                out.push_str(&p.name.text);
                if let Some(word) = input.ty.and_then(|t| t.word()) {
                    out.push_str(": ");
                    out.push_str(word);
                }
                if let Some(o) = &input.over {
                    out.push_str(&format!(" over ({}, {})", o.from.0, o.to.0));
                }
            } else {
                out.push_str(&p.name.text);
            }
            if p.bound() {
                out.push_str(" := ");
                out.push_str(&p.text);
            }
            if let Some((seed, _)) = p.input.as_ref().and_then(|i| i.seed.as_ref()) {
                out.push_str(&format!(" hint({seed})"));
            }
        }
        StmtKind::Group(g) => {
            out.push_str(&format!("{} := ", g.name.text));
            if g.material.is_some() {
                out.push_str("material(");
                write_fields(out, &g.fields);
                out.push(')');
            } else {
                write_members(out, &g.fields);
            }
        }
        StmtKind::Unit(n) => out.push_str(&format!("unit {}", n.text)),
        StmtKind::Style(r) => {
            out.push_str(&format!("style .{} {{ ", r.name.text));
            let parts: Vec<String> = r
                .props
                .iter()
                .filter_map(|k| style_prop_text(&r.style, k).map(|v| format!("{k}: {v}")))
                .collect();
            out.push_str(&parts.join("; "));
            out.push_str(" }");
        }
        StmtKind::Measured(m) => {
            out.push_str(&format!("{} {} {}", m.lhs.text(), m.cmp.text(), m.rhs.text()));
        }
        StmtKind::Scenario(sc) => sc.write(out),
        StmtKind::Beam(b) => b.write(out),
        StmtKind::ValueClaim(c) => out.push_str(&format!("claim {}", c.text)),
        StmtKind::Minimize(m) => {
            write_ref(out, &m.curve);
            out.push_str(if m.maximize { " maximizes " } else { " minimizes " });
            for (i, t) in m.terms.iter().enumerate() {
                let c = if i == 0 { t.coef } else { t.coef.abs() };
                if i > 0 {
                    out.push_str(if t.coef < 0.0 { " - " } else { " + " });
                }
                match c {
                    1.0 => {}
                    -1.0 => out.push('-'),
                    _ => out.push_str(&format!("{} * ", super::num(c))),
                }
                let over = match &t.tangent {
                    Some(tn) => format!("({}, {})", t.point.text, tn.text),
                    None => t.point.text.clone(),
                };
                out.push_str(&format!("integral({} over {over})", t.body));
            }
        }
        // `sec := prof mapped(from -> a, to -> b) in n` (#357)
        StmtKind::Block(super::Block { image: Some(img), over: Some(over), .. }) => {
            out.push_str(&img.name.text);
            out.push_str(" := ");
            write_ref(out, &over.chain);
            out.push(' ');
            out.push_str(&img.word.text);
            let (parts, _) = written_parts(&img.args, true);
            out.push_str(&format!("({}) in ", parts.join(", ")));
            write_ref(out, &img.plane);
        }
        StmtKind::Block(_) | StmtKind::ClaimOver(_) | StmtKind::Chain(_) | StmtKind::Set(_) => {
            unreachable!("validated before printing")
        }
    }
}

/// Lines and splines use positional children; other kinds label child slots.
fn labels_children(k: EntKind) -> bool {
    !matches!(k, EntKind::Line | EntKind::Spline)
}

/// `{bore: 16mm, cyl: {axis: datum}}` — a group's members, nested groups in their braces.
fn write_members(out: &mut String, fields: &[InstArg]) {
    out.push('{');
    write_fields(out, fields);
    out.push('}');
}

/// `bore: 16mm, cyl: {axis: datum}` — a group's members, or a material's properties, between
/// whatever brackets hold them.
fn write_fields(out: &mut String, fields: &[InstArg]) {
    for (i, f) in fields.iter().enumerate() {
        if i > 0 { out.push_str(", "); }
        if let Some(label) = &f.label { out.push_str(&format!("{}: ", label.text)); }
        write_inst_val(out, &f.value);
    }
}

fn write_inst_val(out: &mut String, v: &InstVal) {
    match v {
        InstVal::Ref(r) => write_ref(out, r),
        InstVal::Expr(t) => out.push_str(t),
        InstVal::Hint(t, _) => out.push_str(&format!("hint({t})")),
        InstVal::Group(fields) => write_members(out, fields),
    }
}

/// `Tooth(base, a0: 30)` — a component and what it is given, the same spelling for an instance
/// statement and for an instance written in place inside a curve.
fn write_instance_call(out: &mut String, i: &Instance) {
    out.push_str(&format!("{}(", i.component.text));
    let parts: Vec<String> = i
        .args
        .iter()
        .map(|a| {
            let mut s = String::new();
            if let Some(l) = &a.label {
                s.push_str(&l.text);
                s.push_str(": ");
            }
            write_inst_val(&mut s, &a.value);
            s
        })
        .collect();
    out.push_str(&parts.join(", "));
    out.push(')');
}

/// `leg.toe over theta in (0, 360)` — what a curve is a curve of (§6.5).
fn write_curve_spec(out: &mut String, c: &CurveSpec) {
    match &c.target {
        CurveTarget::Drawn(r) => write_ref(out, r),
        CurveTarget::Anon(inst, point) => {
            write_instance_call(out, inst);
            out.push('.');
            write_ref(out, point);
        }
    }
    match &c.second {
        // over a declared parameter's own interval (#391 S4)
        None if c.domain.0.is_empty() => out.push_str(&format!(" over {}", c.swept.text)),
        None => out.push_str(&format!(" over {} in ({}, {})", c.swept.text, c.domain.0, c.domain.1)),
        Some((s, (a, b))) => out.push_str(&format!(" over ({} in ({}, {}), {} in ({a}, {b}))",
            c.swept.text, c.domain.0, c.domain.1, s.text)),
    }
}

fn write_decl(out: &mut String, d: &Decl) {
    d.annotations.write(out);
    // an anonymous declaration has no name to spell — its key is the elaboration's, not the
    // source's — so the value stands alone
    if let Some(n) = d.name.written() {
        out.push_str(&n.text);
        out.push_str(" := ");
    }
    // a reading at an instant is its source and the instant, as written (#391 S2)
    if let Some(c) = &d.carried {
        write_ref(out, &c.source);
        out.push_str(&match frame_drawn_in(&c.args) {
            // and one drawn in a jointed frame, at the frame's value (#391 S3)
            Some(f) => format!(" in {}", super::ref_text(f)),
            None => instant_text(&c.args),
        });
        write_class(out, d);
        return;
    }
    // and so is a solid placed at one (#391 S2): `tool at(under: m) θ`
    if let Some(Sweep::Placed { motion, at }) = &d.sweep {
        if let Some(Kid::Ref(tool)) = d.children.first().and_then(|k| k.first()) {
            write_ref(out, tool);
            out.push_str(&format!(" at{} {}", frames_text(motion.as_ref(), d.seen_from.as_ref()),
                                  dim(at)));
            write_class(out, d);
            return;
        }
    }
    // a sweep over an interval or through a body, a face family's among them, is `sweep(…)`
    if let Some(sweep @ (Sweep::Swept { .. } | Sweep::Rolled { .. } | Sweep::Family(_)))
        = &d.sweep
    {
        out.push_str(&sweep_text(d, sweep));
        write_class(out, d);
        return;
    }
    // a computed point is its formula, and carries no clause a solve could write
    if let Some([(x, _), (y, _)]) = &d.computed {
        out.push_str(&format!("point(x: {x}, y: {y})"));
        return;
    }
    if let Some(c) = &d.curve {
        // a point of a component followed as one of its formals runs (#391 S2)
        out.push_str("trace(");
        write_curve_spec(out, c);
        out.push(')');
    } else if d.bare {
        // a frame declared bare is its word alone (#383)
        out.push_str("frame");
    } else {
        out.push_str(d.kind.as_str());
        // a declaration written with no `hint(…)` prints none: "no clause" is the empty span
        if d.hint_span.is_some_and(|s| s.is_empty()) && d.seed_at.is_none() {
            out.push_str(&decl_args(d));
        } else {
            out.push_str(&decl_tail(d, &d.seed));
        }
    }
    if let Some(u) = &d.knots {
        out.push_str(" knots [");
        out.push_str(&u.iter().map(|&v| num(v)).collect::<Vec<_>>().join(", "));
        out.push(']');
    }
    if let Some(w) = &d.weights {
        out.push_str(" weights [");
        out.push_str(&w.iter().map(|w| w.text.clone().unwrap_or_else(|| num(w.value))).collect::<Vec<_>>().join(", "));
        out.push(']');
    }
    // what a solid is made of (#510 P3)
    if let Some(m) = &d.material {
        out.push_str(" of ");
        write_ref(out, m);
    }
    if !d.class.is_empty() {
        out.push_str(" class ");
        out.push_str(&d.class.0.join(" "));
    }
    // only a clause this statement wrote: a block's or an instance's is already written once
    if let Some(p) = d.membership.written() {
        out.push_str(" in ");
        write_ref(out, p);
        for q in d.membership.also() {
            out.push_str(", ");
            write_ref(out, q);
        }
    }
}

/// ` class a b` after a declaration that has one.
fn write_class(out: &mut String, d: &Decl) {
    if !d.class.is_empty() {
        out.push_str(" class ");
        out.push_str(&d.class.0.join(" "));
    }
}

/// `(under: m, seen_from: g)` on an instant, or nothing where it names neither (#391 S2).
fn frames_text(under: Option<&Ref>, seen_from: Option<&Ref>) -> String {
    let parts: Vec<String> = [("under", under), ("seen_from", seen_from)].into_iter()
        .filter_map(|(label, r)| r.map(|r| format!("{label}: {}", super::ref_text(r))))
        .collect();
    if parts.is_empty() { String::new() } else { format!("({})", parts.join(", ")) }
}

/// ` at τ`, or ` at(under: m, seen_from: g) τ` — an instant as its arguments hold it (#391 S2):
/// the frames by their labels, the roll the relation's number.
pub(crate) fn instant_text(args: &[OpArg]) -> String {
    let frame = |label: &str| args.iter().find_map(|a| match a {
        OpArg::Named(n, Arg::Ref(r)) if n.text == label => Some(r),
        _ => None,
    });
    let roll = args.iter().find_map(|a| match a {
        OpArg::Dim(t, _) => Some(t.as_str()),
        _ => None,
    }).unwrap_or_default();
    format!(" at{} {roll}", frames_text(frame("under"), frame("seen_from")))
}

/// The jointed frame a reading draws its source in, `x in f` (#391 S3), where it is one.
fn frame_drawn_in(args: &[OpArg]) -> Option<&super::Ref> {
    args.iter().find_map(|a| match a {
        OpArg::Named(n, Arg::Ref(r)) if n.text == super::DRAWN_IN => Some(r),
        _ => None,
    })
}

/// ` over t in (a, b)`, or ` over t` where it names a declared parameter (#391 S2).
fn over_text(over: &super::RunsOver, from: &Arg, to: &Arg) -> String {
    match over.declared {
        true => format!(" over {}", over.name.text),
        false => format!(" over {} in ({}, {})", over.name.text, dim(from), dim(to)),
    }
}

/// `sweep(…)` (#391 S2): a face family's instance written in place and its formal's interval,
/// or the tool a motion sweeps over an interval of the roll or through a body, and the frames.
fn sweep_text(d: &Decl, sweep: &Sweep) -> String {
    let tool = || match d.children.first().and_then(|k| k.first()) {
        Some(Kid::Ref(r)) => super::ref_text(r),
        _ => String::new(),
    };
    let mut parts = match sweep {
        Sweep::Family(_) => sweep_parts(sweep),
        Sweep::Swept { from, to, over, .. } => {
            vec![format!("{}{}", tool(), over_text(over, from, to))]
        }
        Sweep::Rolled { body, .. } => vec![tool(), format!("through: {}", super::ref_text(body))],
        _ => Vec::new(),
    };
    if let Sweep::Swept { motion, .. } | Sweep::Rolled { motion, .. } = sweep {
        parts.extend(motion.iter().map(under));
        parts.extend(d.seen_from.iter().map(|g| format!("seen_from: {}", super::ref_text(g))));
    }
    format!("sweep({})", parts.join(", "))
}

/// A document expression, kept as written rather than converted to a seed.
fn dim(a: &Arg) -> String {
    match a {
        Arg::Dim { text, .. } => text.clone(),
        other => write_arg("", crate::constraints::SpecKind::Float, other),
    }
}

/// `under: m`, as a sweep prints it.
fn under(m: &Ref) -> String {
    let mut text = String::from("under: ");
    write_ref(&mut text,m);
    text
}

/// Sweep arguments are document expressions, preserved through canonical printing.
fn sweep_parts(s: &Sweep) -> Vec<String> {
    let mut parts = match s {
        // a reading at an instant or over an interval is no `solid(…)`'s (#391 S2): `write_decl`
        // writes it whole
        Sweep::Swept { .. } | Sweep::Rolled { .. } | Sweep::Placed { .. } => Vec::new(),
        // a region's operand is the set the flattener applied to its probe: nothing to print
        Sweep::Body | Sweep::Region { .. } => Vec::new(),
        // the face family is the whole of the brackets: `Station(g).f over s`, `s` declared
        Sweep::Family(f) => {
            let mut text = String::new();
            write_instance_call(&mut text, &f.instance);
            text.push('.');
            write_ref(&mut text, &f.face);
            text.push_str(&format!(" over {}", f.swept.text));
            vec![text]
        }
        Sweep::Through { body } => {
            let mut text = String::from("through: ");
            write_ref(&mut text, body);
            vec![text]
        },
        Sweep::Prism { from, to, .. } => vec![format!("from: {}", dim(from)), format!("to: {}", dim(to))],
        Sweep::Depth { depth, .. } => vec![format!("depth: {}", dim(depth))],
        Sweep::Revolve { axis, sweep, sense } => {
            let mut about = String::from("about: ");
            write_ref(&mut about, axis);
            let mut parts = vec![about];
            if let Some(sweep) = sweep {
                parts.push(format!("sweep: {}", dim(sweep)));
            }
            if *sense == Sense::Cw {
                parts.push("sense: cw".into());
            }
            parts
        }
    };
    parts.extend(s.draft().map(|d| format!("draft: {}", dim(d))));
    parts
}

/// One property of a style, as a `style` block writes it.
fn style_prop_text(s: &Style, prop: &str) -> Option<String> {
    match prop {
        "dash" => s.dash.as_ref().map(|d| d.iter().map(|&v| num(v)).collect::<Vec<_>>().join(" ")),
        "width" => s.width.map(num),
        "color" => s.color.clone(),
        "display" => s.display.map(|d| d.as_str().to_string()),
        _ => None,
    }
}

/// `(center: p2)` — what a declaration says the thing is *made of*, or nothing when it names
/// none of it. A slot holds a name, a `hint(…)` seed, or a solid's inline face.
pub(crate) fn decl_args(d: &Decl) -> String {
    // A *gap* in the slots forces labels on: an unlabelled child counts into its slot by
    // position, so where an earlier slot stands empty — a corner the writeback left for the
    // chain's marker to thread again — a bare `line(hint(…))` would put the kept end in the
    // wrong slot on the next parse, and the pose committed for it would quietly reseed.
    let after_gap = d
        .children
        .iter()
        .rposition(|g| !g.is_empty())
        .is_some_and(|k| d.children[..k].iter().any(|g| g.is_empty()));
    let label = labels_children(d.kind) || after_gap;
    let mut parts: Vec<String> = Vec::new();
    let mut child = 0usize;
    // a face family's face is the instance's, written in its brackets (`sweep_parts`)
    let family = matches!(d.sweep, Some(Sweep::Family(_)));
    for (name, field) in d.kind.fields().iter().filter(|_| !family) {
        match field {
            Field::Child | Field::List => {
                let kids = d.children.get(child).map(|g| g.as_slice()).unwrap_or_default();
                child += 1;
                for (i, k) in kids.iter().enumerate() {
                    let mut s = String::new();
                    if (label && *field == Field::Child)
                        || (d.kind == EntKind::Face && *name == "holes" && i == 0) {
                        s.push_str(name);
                        s.push_str(": ");
                    }
                    match k {
                        Kid::Ref(r) => write_ref(&mut s, r),
                        Kid::Hint(k) => s.push_str(&kid_seed_text(k)),
                        Kid::Face { decl, .. } => write_decl(&mut s, decl),
                        Kid::Trim { curve, from, to, .. } => {
                            write_ref(&mut s, curve);
                            s.push_str(" from ");
                            write_ref(&mut s, from);
                            s.push_str(" to ");
                            write_ref(&mut s, to);
                        }
                    }
                    parts.push(s);
                }
            }
            // every scalar is a seed, and every seed is in the `hint(…)` clause: the brackets
            // after the name are what the thing is *made of*, and a radius is not that
            Field::Scalar => {}
        }
    }
    if let Some(sweep) = &d.sweep {
        parts.extend(sweep_parts(sweep));
    }
    if let Some(n) = &d.normal {
        parts.push(format!("normal: {}", super::ref_text(n)));
    }
    if let Some(s) = d.sense {
        parts.push(format!("sense: {}", if s == Sense::Cw { "cw" } else { "ccw" }));
    }
    if let Some(t) = &d.through {
        parts.push(format!("through: {}", super::ref_text(&t.chain)));
        for (label, r) in [("start", &t.start), ("end", &t.end)] {
            if let Some(r) = r {
                parts.push(format!("{label}: {}", super::ref_text(r)));
            }
        }
    }
    if let Some(e) = &d.angular_span {
        match &e.over {
            // over an interval of the roll, after the operand that runs (#391 S2): an envelope's
            // tool, a meshing's second flank
            Some(o) => {
                let k = usize::from(d.kind == EntKind::Meshing);
                if let Some(p) = parts.get_mut(k) {
                    p.push_str(&over_text(o, &e.from, &e.to));
                }
            }
            None => {
                parts.push(format!("from: {}",dim(&e.from)));
                parts.push(format!("to: {}",dim(&e.to)));
            }
        }
        if let Some(x) = &e.ease {
            parts.push(format!("ease: {}",super::ref_text(&x.about)));
            parts.push(format!("along: {}",super::ref_text(&x.along)));
            parts.push(format!("profile: {}",dim(&x.profile)));
            parts.push(format!("lengthwise: {}",dim(&x.lengthwise)));
        }
    }
    // the frame a motion is seen from, beside its `under:` (#383)
    if let Some(g) = &d.seen_from {
        parts.push(format!("seen_from: {}", super::ref_text(g)));
    }
    // and a face's loop seals last, where it was written (§6.8)
    if d.close.is_some() {
        parts.push("-> close".to_string());
    }
    if parts.is_empty() {
        String::new()
    } else {
        format!("({})", parts.join(", "))
    }
}

/// Declaration arguments and seeds, excluding order-independent trailers such as classes.
pub(crate) fn decl_tail(d: &Decl, seed: &[f64]) -> String {
    let mut out = decl_args(d);
    let hint = hint_clause(d, seed);
    if !hint.is_empty() {
        // the list glues to the name; the clause is a word of its own and brings its separator,
        // whether it follows the name or the bracket
        out.push(' ');
        out.push_str(&hint);
    }
    out
}

fn hint_of(parts: &[String]) -> String {
    if parts.is_empty() {
        String::new()
    } else {
        format!("hint({})", parts.join(", "))
    }
}

/// How some of an entity's own numbers are said.
pub(crate) enum Said {
    /// A vector, every component of it given: its key (`""` the point itself) and its members.
    Whole(&'static str, Vec<usize>),
    /// One member, by its index in `EntKind::members`.
    One(usize),
}

/// The numbers `given` (indices into `EntKind::members`, in order), said as the source says
/// them: each vector whole where every component of it is given — `(3, 4)`, `dir: (1, 0, 0)` —
/// else member by member, `y: 7`.  `point` is how many coordinates the point's own vector has
/// here: two in a plane, three in space.
pub(crate) fn said(kind: EntKind, point: usize, given: &[usize]) -> Vec<Said> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < kind.members().len() {
        let Some(&(key, first)) = kind.vectors().iter().find(|&&(_, f)| f == i) else {
            if given.contains(&i) {
                out.push(Said::One(i));
            }
            i += 1;
            continue;
        };
        let len = if key.is_empty() { point } else { 3 };
        let comps: Vec<usize> = (first..first + len).collect();
        if comps.iter().all(|c| given.contains(c)) {
            out.push(Said::Whole(key, comps));
        } else {
            out.extend((first..first + 3).filter(|c| given.contains(c)).map(Said::One));
        }
        i = first + 3;
    }
    out
}

/// The same, as text: `value(i)` the number of member `i` as written, `sep` what stands between
/// a key and its value (`: ` in a hint, ` == ` in a `fix`).
pub(crate) fn said_text(
    kind: EntKind,
    point: usize,
    given: &[usize],
    value: impl Fn(usize) -> String,
    sep: &str,
) -> Vec<String> {
    said(kind, point, given)
        .into_iter()
        .map(|s| match s {
            Said::Whole("", comps) => vector_text(comps.into_iter().map(&value)),
            Said::Whole(key, comps) => format!("{key}{sep}{}", vector_text(comps.into_iter().map(&value))),
            Said::One(i) => format!("{}{sep}{}", kind.members()[i], value(i)),
        })
        .collect()
}

/// A child's seed: a point's place in its plane, or an axis's direction (a plane's `u:` and
/// `v:`), as `hint((3, 4))` or `hint(dir: (1, 0, 0))`.
fn kid_hint(axis: bool, text: [Option<&str>; 3], v: [f64; 3]) -> String {
    let (kind, n) = if axis { (EntKind::Axis, 3) } else { (EntKind::Point, 2) };
    let given: Vec<usize> = (0..n).collect();
    let value = |i: usize| text[i].map_or_else(|| num(v[i]), str::to_string);
    hint_of(&said_text(kind, n, &given, value, ": "))
}

/// The same, for a place a solve arrived at: numbers, and no text anybody wrote — a point's
/// two, or an axis's three.
pub(crate) fn hint_numbers(v: &[f64]) -> String {
    let w = [0, 1, 2].map(|i| v.get(i).copied().unwrap_or(0.0));
    kid_hint(v.len() > 2, [None, None, None], w)
}

/// Print a child slot's seed as it was written, or as the numbers it holds.
pub(crate) fn kid_seed_text(k: &KidSeed) -> String {
    if let Some(d) = &k.dir {
        let named = |r: &Ref| {
            let mut text = String::new();
            write_ref(&mut text, r);
            text
        };
        let mut parts = vec![format!("dir: {}", named(&d.what))];
        if let Some((t, _)) = &d.turn {
            parts.push(format!("turn: {t}"));
        }
        if let Some(a) = &d.about {
            parts.push(format!("about: {}", named(a)));
        }
        return hint_of(&parts);
    }
    kid_hint(k.axis, [k.text[0].as_deref(), k.text[1].as_deref(), k.text[2].as_deref()], k.v)
}

/// Print owned scalar seeds using registry field names. Geometric seeds retain
/// their `at:` spelling; child seeds belong in the argument list.
pub(crate) fn hint_clause(d: &Decl, seed: &[f64]) -> String {
    if let Some(at) = &d.seed_at {
        let mut place = String::new();
        write_ref(&mut place, &at.what);
        let mut parts = vec![format!("at: {place}")];
        if let Some((b, _)) = &at.bearing {
            parts.push(format!("bearing: {b}"));
        }
        for (key, r) in [("toward", &at.toward), ("along", &at.along)] {
            if let Some(r) = r {
                let mut text = String::new();
                write_ref(&mut text, r);
                parts.push(format!("{key}: {text}"));
            }
        }
        for (key, text) in [("by", &at.by), ("turn", &at.turn), ("x", &at.x), ("y", &at.y)] {
            if let Some((t, _)) = text {
                parts.push(format!("{key}: {t}"));
            }
        }
        return hint_of(&parts);
    }
    let mut given: Vec<usize> = Vec::new();
    let text = |i: usize| match d.seed_text.get(i).and_then(|t| t.as_deref()) {
        Some(t) => t.to_string(),
        None => num(seed.get(i).copied().unwrap_or(0.0)),
    };
    for (i, member) in d.kind.members().iter().enumerate() {
        let v = seed.get(i).copied().unwrap_or(0.0);
        // a number the source wrote, though it be 0
        let said = d.seed_text.get(i).is_some_and(|t| t.is_some())
            || d.seed_spans.get(i).is_some_and(|s| !s.is_empty());
        // a number that is 0 where nobody wrote one says nothing: a point's height, an axis's
        // origin, where a plane stands — each 0 unless said
        let optional = matches!(d.kind, EntKind::Point | EntKind::Frame) && *member == "z"
            || member.starts_with("origin.");
        if optional && !said && v == 0.0 {
            continue;
        }
        given.push(i);
    }
    // where a plane stands is said whole, as its origin, where any of it is
    if d.kind == EntKind::Frame && !given.is_empty() {
        given = vec![0, 1, 2];
    }
    if d.kind == EntKind::Axis && given.iter().any(|&i| i >= 3) {
        given.retain(|&i| i < 3);
        given.extend(3..6);
    }
    let point = if given.contains(&2) { 3 } else { 2 };
    hint_of(&said_text(d.kind, point, &given, text, ": "))
}

fn write_relation(out: &mut String, r: &Relation) {
    // a row of the home alone is written in a block of its own (#339 D3)
    if r.initially {
        out.push_str("initially { ");
        write_relation(out, &Relation { initially: false, ..r.clone() });
        out.push_str(" }");
        return;
    }
    // in its block (#460) a claim's relation is the block's, and says no `claim` of its own
    if r.claim && r.witness.is_none() {
        out.push_str("claim ");
    }
    if let Some(n) = &r.name {
        out.push_str(&n.text);
        out.push_str(" := ");
    }
    match &r.form {
        super::RelationForm::Written(w) => write_written(out, w),
        super::RelationForm::Canonical { kind, args, bound } => {
            let hi = bound.as_ref().and_then(|(_, hi)| hi.as_ref());
            let args = bounded(*kind, args, bound.as_ref().map(|(cmp, _)| (*cmp, hi)));
            out.push_str(&operator_text(*kind, &args))
        }
    }

    if !r.class.is_empty() {
        out.push_str(" class ");
        out.push_str(&r.class.0.join(" "));
    }
    if let Some((t, rr)) = r.place {
        out.push_str(&format!(" at ({}, {})", num(t), num(rr)));
    }
}

/// Partition operator arguments between parentheses and the trailing hint clause.
pub(crate) fn written_parts(args: &[OpArg], pairs: bool) -> (Vec<String>, Vec<String>) {
    let mut parts: Vec<String> = Vec::new();
    // `mapped`'s pairs (#357): each source reference and the target after it, `from -> a`
    let mut source: Option<usize> = None;
    let mut hints: Vec<String> = Vec::new();
    let mut bound = None;
    for a in args {
        match a {
            OpArg::Bound { cmp, hi, .. } => bound = Some((*cmp, hi.as_ref().map(|h| &h.0))),
            OpArg::Named(n, v) => parts.push(format!("{}: {}", n.text, sel_text(v))),
            OpArg::Ent(r) => {
                let mut s = String::new();
                write_ref(&mut s, r);
                match source.take() {
                    Some(at) if pairs => parts[at] = format!("{} -> {s}", parts[at]),
                    _ => {
                        source = pairs.then_some(parts.len());
                        parts.push(s);
                    }
                }
            }
            OpArg::Dim(t, _) => parts.insert(0, t.clone()),
            OpArg::Within { key, lo, hi } => {
                parts.push(format!("{} in ({}, {})", key.text, lo.0, hi.0))
            }
            // the slot's own name, as it was written — never guessed from the kind.  The
            // same `slot_text` `operator_text` reads it off the spec with, so the two printers
            // cannot come to spell one slot differently.
            OpArg::Slot { key, arg } => {
                let (pin, hint) = slot_text(&key.text, arg);
                parts.extend(pin);
                hints.extend(hint);
            }
            OpArg::Vector { key, parts: vs, .. } => {
                let v = vector_text(vs.iter().map(value_text));
                parts.push(match key {
                    Some(k) => format!("{} == {v}", k.text),
                    None => v,
                });
            }
        }
    }
    // a bound is said before its number, which leads the parentheses (§9.6)
    if let (Some((cmp, hi)), Some(d)) = (bound, parts.first_mut()) {
        *d = bound_text(cmp, d, hi.map(|h| h.as_str()));
    }
    (parts, hints)
}

/// A bound as written (§9.6): `>= d`, `<= d`, `in: (d, hi)`.
fn bound_text(cmp: super::Cmp, lo: &str, hi: Option<&str>) -> String {
    match (cmp, hi) {
        (super::Cmp::In, Some(hi)) => format!("in: ({lo}, {hi})"),
        _ => format!("{} {lo}", cmp.text()),
    }
}

/// A kind's arguments with its number said as the bound it is, where it is one (§9.6) — what
/// `operator_text` is handed to print a bounded constraint, by a printer and `io::describe`.
pub fn bounded(
    kind: CKind,
    args: &[Option<Arg>],
    bound: Option<(super::Cmp, Option<&Arg>)>,
) -> Vec<Option<Arg>> {
    let mut args = args.to_vec();
    let Some((cmp, hi)) = bound else { return args };
    let at = kind.spec().iter().position(|(_, k)| k.is_dimension());
    if let Some(a) = at.and_then(|i| args.get_mut(i)).and_then(|a| a.as_mut()) {
        let text = bound_text(cmp, &dim_text(a), hi.map(dim_text).as_deref());
        *a = Arg::Dim { text, span: Span::default() };
    }
    args
}

fn write_written(out: &mut String, w: &Written) {
    // a point read at an instant (#391 S2): its roll and frames are the instant's, after the
    // operand it reads
    let instant = w.word.text == "coincident" && w.args.iter().any(|a| matches!(a, OpArg::Dim(..)));
    let of_instant = |a: &OpArg| matches!(a, OpArg::Dim(..))
        || matches!(a, OpArg::Named(n, _) if n.text == "under" || n.text == "seen_from");
    let args: Vec<OpArg> = w.args.iter().filter(|a| !(instant && of_instant(a))).cloned().collect();
    let (parts, hints) = written_parts(&args, w.word.text == "mapped");
    // an operand written as its rate is printed so (#339 D2)
    let operand = |out: &mut String, k: usize| {
        let Some(r) = w.ops.get(k) else { return };
        match w.rate_of(k) {
            Some(over) => {
                out.push_str("rate(");
                write_ref(out, r);
                out.push_str(&format!(" over {})", over.text));
            }
            None => write_ref(out, r),
        }
    };
    let infix = w.fixity == Fixity::Infix;
    if infix && !w.ops.is_empty() {
        operand(out, 0);
        out.push(' ');
    }
    out.push_str(&w.word.text);
    if !parts.is_empty() {
        out.push_str(&format!("({})", parts.join(", ")));
    }
    let last = if infix { 1 } else { 0 };
    if last < w.ops.len() {
        if let Some(c) = crate::constraints::connective(&w.word.text).filter(|_| infix) {
            out.push(' ');
            out.push_str(c);
        }
        out.push(' ');
        operand(out, last);
    }
    if instant {
        out.push_str(&instant_text(&w.args));
    }
    let hint = hint_of(&hints);
    if !hint.is_empty() {
        out.push(' ');
        out.push_str(&hint);
    }
}

/// A constraint the *library* holds, written as the operator it is spelled with (spec §9.1).
///
/// The inverse of parsing, and the one place it is done — `write_relation` prints a statement
/// somebody built rather than wrote, and `io::describe` prints one for a reader.  So the drawing,
/// the constraint list and the program panel cannot come to spell one constraint three ways.
pub fn operator_text(kind: CKind, args: &[Option<Arg>]) -> String {
    let Some((word, fixity)) = kind.operator() else {
        // nobody writes this one: a drag target, a lift, an axis's intrinsics
        return format!("{}(…)", snake(kind.name()));
    };
    if let Some(text) = ordinate_text(kind, args) {
        return text;
    }
    if let Some(text) = rate_text(kind, args) {
        return text;
    }
    let spec = kind.spec();
    let mut ents: Vec<String> = Vec::new();
    let mut parens: Vec<String> = Vec::new();
    let mut hints: Vec<String> = Vec::new();
    // a point read at an instant (#391 S2): the roll stated, and the frames it is read by
    let stated = kind.roll_slot().and_then(|k| args.get(k)).and_then(|a| a.as_ref())
        .is_some_and(|a| matches!(a, Arg::Dim { .. }));
    let mut instant: Vec<String> = Vec::new();
    let mut roll = String::new();
    let mut about = None;
    for (i, (name, sk)) in spec.iter().enumerate() {
        let Some(a) = args.get(i).and_then(|a| a.as_ref()) else { continue };
        // an entity slot the core infers — a projection's planes — is never spelled: the
        // source writes two points, and a lifted statement carries what the core filled in
        if sk.is_entity() && kind.infers_arg(i) {
            continue;
        }
        // nor is a skew distance's side (or a cylinder's tangency's, which is one): the word
        // is `distance`, and the side is the seed's `sign`
        if *name == "sign" && kind.infers_arg(i) {
            continue;
        }
        // a selector nobody wrote is not written: the empty word is what an omitted `side:` or
        // `sense:` holds, and it says the type's own default — that both sides are solutions and
        // the seed picks, or that the angle turns the way the language counts (issue #48, item 4)
        if matches!(a, Arg::Word(w) if w.is_empty()) {
            continue;
        }
        // a similarity keeping its turn says nothing of it (#357)
        if *name == "mirror" && matches!(a, Arg::Bool(false)) {
            continue;
        }
        // the point a tangent is at goes in the parentheses by its name (#251), as written:
        // unlabelled it would read as `symmetry`'s third entity
        if sk.takes_ref() && *name == "at" {
            parens.push(format!("at: {}", write_arg(name, *sk, a)));
        } else if sk.takes_ref() && *name == "by" {
            // and the tool a flank is cut by (#345)
            parens.push(format!("by: {}", write_arg(name, *sk, a)));
        } else if sk.takes_ref() && *name == "about" {
            // and the axis a turn is read about (#558), last, after the sense it reads
            about = Some(format!("about: {}", write_arg(name, *sk, a)));
        } else if sk.takes_ref() && *name == "under" && stated {
            // the motion a point is read by at an instant is the instant's (#391 S2)
            instant.push(format!("under: {}", write_arg(name, *sk, a)));
        } else if sk.takes_ref() && *name == "under" {
            // and the motion a point is carried by (#326)
            parens.push(format!("under: {}", write_arg(name, *sk, a)));
        } else if kind.roll_slot() == Some(i) && matches!(a, Arg::Dim { .. }) {
            // and the instant it is read at, after its operand (#391 S2)
            roll = dim_text(a);
        } else if sk.takes_ref() {
            ents.push(write_arg(name, *sk, a));
        } else if sk.is_param() {
            let (pin, hint) = slot_text(name, a);
            parens.extend(pin);
            hints.extend(hint);
        } else if sk.is_dimension() {
            // the number is written bare and first, wherever its slot stands: a kind has at most
            // one dimension, so there is nothing for a reader to confuse it with, and a selector
            // may sit after it in spec order (`distance(12, side: left)`)
            parens.insert(0, dim_text(a));
        } else {
            parens.push(write_arg(name, *sk, a));
        }
    }
    parens.extend(about);
    // the third entity of `symmetry` goes in the parentheses with everything else that is not
    // one of the two operands — and a call's operands all do, as does the second pair of an
    // angle made equal to another.  In spec order and ahead of any selector, which is where a
    // reader writes them: `l1 angle(l3, l4, sense: cw) l2`
    let outside = if fixity == Fixity::Call { 0 } else { 2 };
    if ents.len() > outside {
        let mut extra: Vec<String> = ents.drain(outside..).collect();
        // a map's pairs, each a point of the source and where it goes (#357)
        if matches!(kind, CKind::Mapped | CKind::MappedSimilar | CKind::MappedCircle) {
            extra = extra.chunks(2).map(|p| p.join(" -> ")).collect();
        }
        parens.splice(0..0, extra);
    }
    let mut out = String::new();
    if fixity == Fixity::Infix && !ents.is_empty() {
        out.push_str(&ents.remove(0));
        out.push(' ');
    }
    out.push_str(word);
    if !parens.is_empty() {
        out.push_str(&format!("({})", parens.join(", ")));
    }
    if let Some(c) = crate::constraints::connective(word).filter(|_| fixity == Fixity::Infix) {
        out.push(' ');
        out.push_str(c);
    }
    for e in &ents {
        out.push(' ');
        out.push_str(e);
    }
    if stated {
        let frames = match instant.is_empty() {
            true => String::new(),
            false => format!("({})", instant.join(", ")),
        };
        out.push_str(&format!(" at{frames} {roll}"));
    }
    let hint = hint_of(&hints);
    if !hint.is_empty() {
        out.push(' ');
        out.push_str(&hint);
    }
    out
}

/// **A rate of a contact, as a document writes it** (#365): `rate(p over t) parallel l`,
/// `perpendicular l`, `rate(p over t) distance(d, along: l)` — or, written at the point, a curve's
/// tangent `g tangent(at: p) l` and a surface's frame, `S perpendicular(at: p) l`, `S tangent(at:
/// p) P`, `S curvature(at: p) k`.  What the contact holds the point on is the contact's, and a rate never
/// names it; its slot is the contact's, written with the contact.  `None` for every other kind.
fn rate_text(kind: CKind, args: &[Option<Arg>]) -> Option<String> {
    let get = |i: usize| args.get(i).and_then(|a| a.as_ref());
    let text = |i: usize| get(i).map_or_else(String::new, |a| write_arg("", SpecKind::Point, a));
    let word = |i: usize| match get(i) {
        Some(Arg::Word(w)) if !w.is_empty() => w.clone(),
        _ => match kind.default_arg(i) {
            crate::constraints::Arg::Str(w) => w,
            _ => String::new(),
        },
    };
    match kind {
        CKind::Rate => {
            // a curve's tangent and a surface's frame are written at the point, the contact's
            // carrier first; a rate in one slot as the rate it is
            let at = match word(5).as_str() {
                "tangent" | "plane" | "in_plane" => "tangent",
                "normal" => "perpendicular",
                "curvature" | "centre" => "curvature",
                w => return Some(format!("rate({} over {}) {w} {}", text(2), word(4), text(1))),
            };
            Some(format!("{} {at}(at: {}) {}", text(0), text(2), text(1)))
        }
        CKind::RateDistance => {
            let d = get(3).map_or_else(String::new, dim_text);
            Some(format!("rate({} over {}) distance({d}, along: {})", text(2), word(5), text(1)))
        }
        _ => None,
    }
}

/// **An ordinate and its zero, as a document writes them** (`docs/ordinate-plan.md`): the
/// direction in the word that named it or as the reference it is, a plane's own word against the
/// plane its first point is the origin of (`q distance(d, along: u) P`), and a level in its
/// own word (`a level(up) b`) — the standard library's `a horizontal b` is a word a file must
/// import, and a constraint written with it says so itself (`Constraint::word`).  The form is
/// the core's and is never written.  `None` for every other kind.
fn ordinate_text(kind: CKind, args: &[Option<Arg>]) -> Option<String> {
    use crate::constraints::Toward;
    let ws = kind.word_slot()?;
    let get = |i: usize| args.get(i).and_then(|a| a.as_ref());
    let ent = |i: usize| get(i).map(|a| write_arg("", SpecKind::Point, a)).unwrap_or_default();
    let (p, q) = (ent(0), ent(1));
    let word = match get(ws) {
        Some(Arg::Word(w)) => w.clone(),
        _ => String::new(),
    };
    let toward = Toward::of(&word);
    let dir = if toward.is_some() { word.clone() } else { ent(2) };
    // a plane's own direction is measured from its origin: the plane is what the origin is of
    let plane = || p.strip_suffix(".origin").unwrap_or(&p).to_string();
    // `n` is only ever a plane's; `u` and `v` are where the core read them over the plane's frame
    // or as a coordinate from its origin, and between two points of a view the view's (#268) —
    // from a view's own origin the two readings are one ordinate, so either spelling says it.
    // With no form (a statement nobody has settled yet) the first point being an origin says it
    let form = kind.form_slot().and_then(get).and_then(|a| match a {
        Arg::Int(i) => Some(crate::constraints::OrdinateForm::from_int(*i)),
        _ => None,
    });
    let origin = p.ends_with(".origin");
    let of_plane = match (toward, form) {
        (Some(Toward::PlaneN), _) => true,
        (Some(t), Some(f)) if t.of_plane() => {
            use crate::constraints::OrdinateForm as F;
            f.of_frame() || (matches!(f, F::CoordU | F::CoordV) && origin)
        }
        (Some(t), None) => t.of_plane() && origin,
        _ => false,
    };
    Some(match kind {
        CKind::Ordinate => {
            let d = get(3).map(dim_text).unwrap_or_default();
            match of_plane {
                true => format!("{q} distance({d}, along: {word}) {}", plane()),
                false => format!("{p} distance({d}, along: {dir}) {q}"),
            }
        }
        _ => match toward {
            Some(_) if of_plane => format!("{q} level({word}) {}", plane()),
            _ => format!("{p} level({dir}) {q}"),
        },
    })
}

/// Format a slot: what goes in the argument list (a pin) and what in the trailing hint clause
/// (a seed).  A shared place is both — `t == s`, and where it starts.
fn slot_text(name: &str, a: &Arg) -> (Option<String>, Option<String>) {
    let (pinned, v) = match a {
        Arg::Seed { value, pinned } => (*pinned, num(*value)),
        Arg::SeedExpr { text, pinned, .. } => (*pinned, text.clone()),
        Arg::Tie { name: shared, seed, .. } => {
            let hint = seed.map(|v| format!("{name}: {}", num(v)));
            return (Some(format!("{name} == {shared}")), hint);
        }
        // a joint's value, an instant kept as the dimension it is (#391 S3)
        Arg::Dim { text, .. } => (true, text.clone()),
        _ => return (None, None),
    };
    match pinned {
        true => (Some(format!("{name} == {v}")), None),
        false => (None, Some(format!("{name}: {v}"))),
    }
}

/// A pinned or seeded number as written: the literal, or the expression it was written as.
fn value_text(a: &Arg) -> String {
    match a {
        Arg::Seed { value, .. } => num(*value),
        Arg::SeedExpr { text, .. } => text.clone(),
        Arg::Tie { name, .. } => name.clone(),
        other => sel_text(other),
    }
}

/// A vector, `(3, 4)`: its components in order, as the source writes one.
pub(crate) fn vector_text(parts: impl IntoIterator<Item = String>) -> String {
    format!("({})", parts.into_iter().collect::<Vec<_>>().join(", "))
}

/// A selector's value, as a `style` block writes one.
pub(crate) fn sel_text(a: &Arg) -> String {
    match a {
        Arg::Num(v) => num(*v),
        Arg::Int(v) => v.to_string(),
        Arg::Bool(b) => b.to_string(),
        Arg::Word(w) => w.clone(),
        // a direction written as a reference, `along: std.x`
        Arg::Ref(r) => {
            let mut s = String::new();
            write_ref(&mut s, r);
            s
        }
        // a defined word's argument, as written (§9.9)
        Arg::Dim { text, .. } => text.clone(),
        other => format!("{other:?}"),
    }
}

/// One argument.  Entity slots are positional — the statement's name says what they are — and
/// everything else is labelled, because `side: 1` says what `1` does not.
fn write_arg(name: &str, sk: SpecKind, a: &Arg) -> String {
    match a {
        Arg::Ref(r) => {
            let mut s = String::new();
            write_ref(&mut s, r);
            s
        }
        // only a *pin* reaches here: an unpinned slot is a seed and `write_relation` puts it in
        // the `hint(…)` clause, which is where every seed in the language is written
        Arg::Seed { value, .. } => format!("{name} == {}", num(*value)),
        Arg::Num(v) if sk == SpecKind::Angle => format!("{name}: {}", num(v.to_degrees())),
        Arg::Num(v) => format!("{name}: {}", num(*v)),
        Arg::Int(v) => format!("{name}: {v}"),
        Arg::Bool(b) => format!("{name}: {b}"),
        // **The language has no string literal** — a quote is the inch mark (spec §3.3) — so a
        // `Str` slot is written as the word it is (`at: start`), bare.
        Arg::Word(w) => format!("{name}: {w}"),
        Arg::Dim { text, .. } => format!("{name}: {text}"),
        Arg::SeedExpr { text, .. } => format!("{name} == {text}"),
        Arg::Tie { name: shared, .. } => format!("{name} == {shared}"),
    }
}

/// What goes after `==`: the dimension's own text, as written.
fn dim_text(a: &Arg) -> String {
    match a {
        Arg::Dim { text, .. } => text.clone(),
        Arg::Num(v) => num(*v),
        other => write_arg("", SpecKind::Length, other),
    }
}
