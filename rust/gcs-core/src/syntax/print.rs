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
        u.span = Span::new(start, out.len());
        out.push('\n');
    }
    for comp in p.components.iter_mut().filter(|c| c.module.is_none()) {
        let lo = out.len();
        // Separate declarations from the relations that follow them.
        let mut said_decl = false;
        for st in &mut comp.body {
            let is_decl = matches!(st.kind, StmtKind::Decl(_));
            if said_decl && !is_decl {
                out.push('\n');
            }
            said_decl = is_decl;
            let start = out.len();
            // a block's plane is said on each statement it gave one: the flat form has no blocks
            match &st.kind {
                StmtKind::Decl(d) if d.membership.plane().is_some() && d.membership.written().is_none() => {
                    write_decl(&mut out, d);
                    out.push_str(" in ");
                    write_ref(&mut out, d.membership.plane().expect("just asked"));
                }
                k => write_stmt(&mut out, k),
            }
            st.span = Span::new(start, out.len());
            out.push('\n');
        }
        comp.span = Span::new(lo, out.len());
    }
    p.components.retain(|c| c.module.is_none());
    p.modules.clear();
    p.text = out;
    p.in_blocks.clear();
    Ok(&p.text)
}

/// Append a supported declaration or statement fragment atomically.
pub fn write_stmt_to(out: &mut String, k: &StmtKind) -> Result<(), PrintError> {
    printable(k)?;
    write_stmt(out, k);
    Ok(())
}

fn printable(k: &StmtKind) -> Result<(), PrintError> {
    match k {
        StmtKind::Chain(_) => Err(PrintError { construct: "named chains" }),
        StmtKind::Block(_) => Err(PrintError { construct: "repeat and cycle blocks" }),
        StmtKind::ClaimOver(_) => Err(PrintError { construct: "claim-over bodies" }),
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
        }
        StmtKind::Param(p) => {
            if let Some(input) = &p.input {
                out.push_str("param ");
                out.push_str(&p.name.text);
                if let Some(word) = input.ty.and_then(|t| t.word()) {
                    out.push_str(": ");
                    out.push_str(word);
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
            write_members(out, &g.fields);
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
        StmtKind::Block(_) | StmtKind::ClaimOver(_) | StmtKind::Chain(_) => {
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
    for (i, f) in fields.iter().enumerate() {
        if i > 0 { out.push_str(", "); }
        if let Some(label) = &f.label { out.push_str(&format!("{}: ", label.text)); }
        write_inst_val(out, &f.value);
    }
    out.push('}');
}

fn write_inst_val(out: &mut String, v: &InstVal) {
    match v {
        InstVal::Ref(r) => write_ref(out, r),
        InstVal::Expr(t) => out.push_str(t),
        InstVal::Hint(t) => out.push_str(&format!("hint({t})")),
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
    out.push_str(&format!(" over {} in ({}, {})", c.swept.text, c.domain.0, c.domain.1));
}

fn write_decl(out: &mut String, d: &Decl) {
    d.annotations.write(out);
    // an anonymous declaration has no name to spell — its key is the elaboration's, not the
    // source's — so the value stands alone
    if let Some(n) = d.name.written() {
        out.push_str(&n.text);
        out.push_str(" := ");
    }
    // a computed point is its formula, and carries no clause a solve could write
    if let Some([(x, _), (y, _)]) = &d.computed {
        out.push_str(&format!("point(x: {x}, y: {y})"));
        return;
    }
    if let Some(c) = &d.curve {
        write_curve_spec(out, c);
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
    if !d.class.is_empty() {
        out.push_str(" class ");
        out.push_str(&d.class.0.join(" "));
    }
    // only a clause this statement wrote: a block's or an instance's is already written once
    if let Some(p) = d.membership.written() {
        out.push_str(" in ");
        write_ref(out, p);
    }
}

/// A document expression, kept as written rather than converted to a seed.
fn dim(a: &Arg) -> String {
    match a {
        Arg::Dim { text, .. } => text.clone(),
        other => write_arg("", crate::constraints::SpecKind::Float, other),
    }
}

/// Sweep arguments are document expressions, preserved through canonical printing.
fn sweep_parts(s: &Sweep) -> Vec<String> {
    match s {
        Sweep::Swept {motion,from,to} => {
            let mut text = String::from("under: ");
            write_ref(&mut text,motion);
            vec![text,format!("from: {}",dim(from)),format!("to: {}",dim(to))]
        }
        Sweep::Placed {motion,at} => {
            let mut text = String::from("under: ");
            write_ref(&mut text,motion);
            vec![text,format!("at: {}",dim(at))]
        }
        Sweep::Body => Vec::new(),
        Sweep::Along { guide } => {
            let mut text = String::from("along: ");
            write_ref(&mut text, guide);
            vec![text]
        },
        Sweep::Through { body } => {
            let mut text = String::from("through: ");
            write_ref(&mut text, body);
            vec![text]
        },
        Sweep::Prism { from, to } => vec![format!("from: {}", dim(from)), format!("to: {}", dim(to))],
        Sweep::Depth { depth } => vec![format!("depth: {}", dim(depth))],
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
    }
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
    for (name, field) in d.kind.fields() {
        match field {
            Field::Child | Field::List => {
                let kids = d.children.get(child).map(|g| g.as_slice()).unwrap_or_default();
                child += 1;
                for (i, k) in kids.iter().enumerate() {
                    let mut s = String::new();
                    if (label && *field == Field::Child) || d.kind == EntKind::Patch
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
    if let Some(e) = &d.angular_span {
        parts.push(format!("from: {}",dim(&e.from)));
        parts.push(format!("to: {}",dim(&e.to)));
    }
    if let Some(motion) = &d.motion {
        use super::MotionSpec;
        match motion {
            MotionSpec::Rotation {axis,ratio,phase,advance} => {
                parts.push(format!("about: {}",super::ref_text(axis)));
                if let Some(a) = ratio { parts.push(format!("ratio: {}",dim(a))); }
                if let Some(a) = phase { parts.push(format!("phase: {}",dim(a))); }
                if let Some(a) = advance { parts.push(format!("advance: {}",dim(a))); }
            }
            MotionSpec::Translation {axis,advance} => {
                parts.push(format!("along: {}",super::ref_text(axis)));
                parts.push(format!("advance: {}",dim(advance)));
            }
            MotionSpec::Relative {source,observer} => {
                parts.push(super::ref_text(source));
                parts.push(format!("relative_to: {}",super::ref_text(observer)));
            }
        }
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

/// Print owned scalar seeds using registry field names. Geometric seeds retain
/// their `at:` spelling; child seeds belong in the argument list.
fn point_hint(text: [Option<&str>; 2], v: [f64; 2]) -> String {
    // a child's place in its plane: x and y, never the height a point in space has
    let parts: Vec<String> = EntKind::Point
        .fields()
        .iter()
        .filter(|(_, f)| *f == Field::Scalar)
        .take(2)
        .enumerate()
        .map(|(i, (name, _))| match text.get(i).copied().flatten() {
            Some(t) => format!("{name}: {t}"),
            None => format!("{name}: {}", num(v.get(i).copied().unwrap_or(0.0))),
        })
        .collect();
    hint_of(&parts)
}

/// The same, for a place a solve arrived at: numbers, and no text anybody wrote.
pub(crate) fn hint_xy(x: f64, y: f64) -> String {
    point_hint([None, None], [x, y])
}

/// Print owned scalar seeds using registry field names. Geometric seeds retain
/// their `at:` spelling; child seeds belong in the argument list.
pub(crate) fn kid_seed_text(k: &KidSeed) -> String {
    point_hint([k.text[0].as_deref(), k.text[1].as_deref()], k.v)
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
    let mut parts: Vec<String> = Vec::new();
    let mut scalar = 0usize;
    for (name, field) in d.kind.fields() {
        if *field != Field::Scalar {
            continue;
        }
        let v = seed.get(scalar).copied().unwrap_or(0.0);
        let written = d.seed_text.get(scalar).and_then(|t| t.as_deref());
        // a number the source wrote, though it be 0
        let said = written.is_some() || d.seed_spans.get(scalar).is_some_and(|s| !s.is_empty());
        scalar += 1;
        // a number that is 0 where nobody wrote one says nothing: a point's height, an axis's
        // place, where a plane stands — each 0 unless said
        let optional = matches!(
            (d.kind, *name),
            (EntKind::Point, "z") | (EntKind::Axis, "px" | "py" | "pz") | (EntKind::Plane, _)
        );
        if optional && !said && v == 0.0 {
            continue;
        }
        let text = match written {
            Some(t) => t.to_string(),
            None => num(v),
        };
        parts.push(format!("{name}: {text}"));
    }
    hint_of(&parts)
}

fn write_relation(out: &mut String, r: &Relation) {
    if r.claim {
        out.push_str("claim ");
    }
    match &r.form {
        super::RelationForm::Written(w) => write_written(out, w),
        super::RelationForm::Canonical { kind, args } => out.push_str(&operator_text(*kind, args)),
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
fn written_parts(args: &[OpArg]) -> (Vec<String>, Vec<String>) {
    let mut parts: Vec<String> = Vec::new();
    let mut hints: Vec<String> = Vec::new();
    for a in args {
        match a {
            OpArg::Named(n, v) => parts.push(format!("{}: {}", n.text, sel_text(v))),
            OpArg::Ent(r) => {
                let mut s = String::new();
                write_ref(&mut s, r);
                parts.push(s);
            }
            OpArg::Dim(t, _) => parts.insert(0, t.clone()),
            // the slot's own name, as it was written — never guessed from the kind.  The
            // same `slot_text` `operator_text` reads it off the spec with, so the two printers
            // cannot come to spell one slot differently.
            OpArg::Slot { key, arg } => {
                let (pin, hint) = slot_text(&key.text, arg);
                parts.extend(pin);
                hints.extend(hint);
            }
        }
    }
    (parts, hints)
}

fn write_written(out: &mut String, w: &Written) {
    let (parts, hints) = written_parts(&w.args);
    let head = |out: &mut String, r: &Ref| {
        write_ref(out, r);
        out.push(' ');
    };
    if w.fixity == Fixity::Infix {
        if let Some(l) = w.ops.first() {
            head(out, l);
        }
    }
    out.push_str(&w.word.text);
    if !parts.is_empty() {
        out.push_str(&format!("({})", parts.join(", ")));
    }
    let last = if w.fixity == Fixity::Infix { w.ops.get(1) } else { w.ops.first() };
    if let Some(r) = last {
        out.push(' ');
        write_ref(out, r);
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
    let spec = kind.spec();
    let mut ents: Vec<String> = Vec::new();
    let mut parens: Vec<String> = Vec::new();
    let mut hints: Vec<String> = Vec::new();
    // which of the three a pair of points means is not in the kind's name but in `along:`, so
    // the axis is written back in whenever the slot itself says nothing — a run that names its
    // direction (`along: left`) carries the word in the slot and writes it there like any other
    let named_along =
        matches!(args.get(3).and_then(|a| a.as_ref()), Some(Arg::Word(w)) if !w.is_empty());
    if !named_along {
        match kind {
            CKind::HorizontalDistance => parens.push("along: x".to_string()),
            CKind::VerticalDistance => parens.push("along: y".to_string()),
            CKind::CoordinateU | CKind::Ordinate3U => parens.push("along: u".to_string()),
            CKind::CoordinateV | CKind::Ordinate3V => parens.push("along: v".to_string()),
            CKind::PointPlaneDistance => {
                parens.push("along: n".to_string())
            }
            _ => {}
        }
    }
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
        if sk.takes_ref() {
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
    // the third entity of `symmetry` goes in the parentheses with everything else that is not
    // one of the two operands — and a call's operands all do, as does the second pair of an
    // angle made equal to another.  In spec order and ahead of any selector, which is where a
    // reader writes them: `l1 angle(l3, l4, sense: cw) l2`
    let outside = if fixity == Fixity::Call { 0 } else { 2 };
    if ents.len() > outside {
        let extra: Vec<String> = ents.drain(outside..).collect();
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
    for e in &ents {
        out.push(' ');
        out.push_str(e);
    }
    let hint = hint_of(&hints);
    if !hint.is_empty() {
        out.push(' ');
        out.push_str(&hint);
    }
    out
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
        _ => return (None, None),
    };
    match pinned {
        true => (Some(format!("{name} == {v}")), None),
        false => (None, Some(format!("{name}: {v}"))),
    }
}

/// A selector's value, as a `style` block writes one.
fn sel_text(a: &Arg) -> String {
    match a {
        Arg::Num(v) => num(*v),
        Arg::Int(v) => v.to_string(),
        Arg::Bool(b) => b.to_string(),
        Arg::Word(w) => w.clone(),
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
