//! A gesture that draws: a point, an entity, a plane, a rectangle, a relation.

use super::*;

/// The rectangle the Rect tool draws: a **reusable component**, defined once per document, and
/// one instance per gesture.  The definition is the chain a person would write — four lines
/// welded corner to corner at right angles from a corner it is given, the first level and the
/// first two carrying the width and the height — so what a gesture leaves in the source is a
/// corner and one statement, `p0 := point` and `r0 := Rectangle(p0, w: 120, h: 60)`, and the
/// drawing owns a `Rectangle` any later statement may instance again.  A definition is a
/// function of what it is given (#562): the corner is the instance's one freedom, a point of the
/// drawing's own, and `names` is the instance's then the corner's, for the tool to seed at the
/// gesture.
pub fn add_rectangle(prog: &Program, w: f64, h: f64, plane: &str) -> Edit {
    // its first side is level, which a figure standing in space has no meaning for
    if plane.is_empty() {
        return Edit::none(prog, Some("a rectangle is drawn in a plane: choose one first".into()));
    }
    let (at, lead) = append_at(prog);
    let mut with = lead;
    let mut edits = Vec::new();
    // the document's own: a module's `Rectangle` is no name a bare call reaches (§14.4)
    if prog.component("Rectangle").is_none() {
        let definition = "component Rectangle(corner: point, w: Length, h: Length) {\n  \
             length(w) (l1 := line(corner)) -> perpendicular length(h) (l2 := line) -> \
             perpendicular (l3 := line) -> perpendicular (l4 := line) -> close\n  \
             horizontal l1\n}\n\n";
        if let Some(preview) = prog.preview {
            let start = preview.lo as usize;
            edits.push(Splice { at: Span::new(start, start), with: definition.into() });
        } else {
            if at.lo > 0 { with.push('\n'); }
            with.push_str(definition);
        }
    }
    // a fresh instance name and corner, past every name the document already binds
    let mut taken = taken_names(prog);
    let name = (0..).map(|i| format!("r{i}")).find(|n| !taken.contains(n)).unwrap_or_default();
    taken.insert(name.clone());
    let corner = next_name(&mut taken, EntKind::Point);
    // the corner the tool seeds, so written with no `hint(…)` (the empty clause), in the view
    // the instance joins
    let written = syntax::DeclName::Written(syntax::Name::new(&corner));
    let mut point = Decl::point(written, [0.0; 2], None);
    point.hint_span = Some(Span::default());
    point.membership = syntax::Membership::lifted(syntax::Ref::new(plane));
    let arg = |label: Option<&str>, value: syntax::InstVal| syntax::InstArg {
        label: label.map(syntax::Name::new),
        value,
        span: Span::default(),
    };
    let length = |label: &str, v: f64| arg(Some(label), syntax::InstVal::Expr(num(v)));
    let inst = syntax::Instance {
        annotations: Default::default(),
        name: syntax::Name::new(name.clone()),
        component: syntax::Name::new("Rectangle"),
        args: vec![
            arg(None, syntax::InstVal::Ref(syntax::Ref::new(&corner))),
            length("w", w),
            length("h", h),
        ],
        span: Span::default(),
        // drawn in the caller's view: the instance joins it whole (§6.7)
        membership: syntax::Membership::lifted(syntax::Ref::new(plane)),
        class: Default::default(),
        mirror: None,
    };
    let mut line = String::new();
    for (k, st) in [StmtKind::Decl(point), StmtKind::Instance(inst)].iter().enumerate() {
        if k > 0 {
            line.push('\n');
        }
        if let Err(e) = syntax::write_stmt_to(&mut line, st) {
            return Edit::none(prog, Some(e.to_string()));
        }
    }
    with.push_str(&line);
    edits.push(Splice { at, with });
    Edit {
        text: splice(prog.text(), edits),
        kind: Kind::Structural,
        names: vec![name, corner],
        refused: None,
        kept: 0,
    }
}

/// `pN := point hint((…, …))`
pub fn add_point(prog: &Program, x: f64, y: f64) -> Edit {
    let name = mint(prog, EntKind::Point);
    let d = Decl::point(syntax::DeclName::Written(syntax::Name::new(name.clone())), [x, y], None);
    append(prog, StmtKind::Decl(d), vec![name])
}

/// An entity built from names that already exist — a line from two points, a circle from a centre.
pub fn add_entity(prog: &Program, kind: EntKind, args: &[String], seed: &[f64]) -> Edit {
    add_entity_with(prog, kind, args, seed, None)
}

/// A plane over two axes or drawn lines, `args` their names — `plane(u: a, v: b)` — and, when
/// the caller has one, the name it asked for.  A name already in use is refused rather than
/// silently renamed: the caller is about to refer to it.
///
/// Held to the elaborator: two axes that do not meet, or run alike, make no plane (E067), and the
/// edit is refused in those words.  The language's own form — a gesture drops one by
/// `add_datum` instead, constrained to what is selected.
pub fn add_plane(prog: &Program, args: &[String], name: Option<&str>) -> Edit {
    Edit::refused_or(prog, entity_decl(prog, EntKind::Frame, args, &[], name)
        .and_then(|(d, name)| append_checked(prog, &[StmtKind::Decl(d)], vec![name])))
}

fn add_entity_with(
    prog: &Program,
    kind: EntKind,
    args: &[String],
    seed: &[f64],
    name: Option<&str>,
) -> Edit {
    match entity_decl(prog, kind, args, seed, name) {
        Ok((d, name)) => append(prog, StmtKind::Decl(d), vec![name]),
        Err(e) => Edit::none(prog, Some(e)),
    }
}

/// The declaration a gesture writes over names, and its name: each child slot takes the next
/// name, a list the rest.
fn entity_decl(
    prog: &Program,
    kind: EntKind,
    args: &[String],
    seed: &[f64],
    name: Option<&str>,
) -> Result<(Decl, String), String> {
    if kind == EntKind::Point || kind == EntKind::Curve {
        return Err(format!("{} is not built this way", kind.a()));
    }
    let name = chosen_name(&mut taken_names(prog), name, kind)?;
    let mut children: Vec<Vec<syntax::Kid>> = Vec::new();
    let mut taken = 0usize;
    for (_, f) in kind.fields() {
        match f {
            crate::model::Field::Child => {
                children.push(refs(args.get(taken)));
                taken += 1;
            }
            crate::model::Field::List => {
                children.push(refs(&args[taken.min(args.len())..]));
                taken = args.len();
            }
            crate::model::Field::Scalar => {}
        }
    }
    Ok((gesture_decl(kind, &name, children, seed), name))
}

/// The name a gesture asked for, or a fresh one: a name already spoken for, or one no statement
/// can say, is refused rather than silently changed, since the caller is about to refer to it.
pub(super) fn chosen_name(
    taken: &mut std::collections::BTreeSet<String>,
    name: Option<&str>,
    kind: EntKind,
) -> Result<String, String> {
    match name {
        Some(n) if taken.contains(n) => Err(format!("`{n}` is already a name in this document")),
        Some(n) if !crate::syntax::is_name(n) => {
            Err(format!("`{n}` is not a name a statement can say"))
        }
        Some(n) => {
            taken.insert(n.to_string());
            Ok(n.to_string())
        }
        None => Ok(next_name(taken, kind)),
    }
}

/// A declaration as a gesture writes it: a name, what it is made of, the seeds of its scalars,
/// and nothing a person adds by hand.
pub(super) fn gesture_decl(kind: EntKind, name: &str, children: Vec<Vec<syntax::Kid>>, seed: &[f64])
    -> Decl {
    let n_scalar = kind.fields().iter().filter(|(_, f)| *f == crate::model::Field::Scalar).count();
    Decl {
        annotations: Default::default(),
        kind,
        name: syntax::DeclName::Written(syntax::Name::new(name)),
        children,
        seed: (0..n_scalar).map(|i| seed.get(i).copied().unwrap_or(0.0)).collect(),
        seed_text: vec![None; n_scalar],
        seed_spans: Vec::new(),
        hint_span: None,
        knots: None,
        weights: None,
        curve: None,
        computed: None,
        class: Default::default(),
        class_span: Span::default(),
        seed_at: None,
        seed_names: Vec::new(),
        sweep: None, seen_from: None, angular_span: None,
        membership: Default::default(),
        list_span: Span::default(),
        close: None,
        mint_close: None,
        through: None,
        normal: None,
        sense: None,
        bare: false,
        carried: None,
        material: None,
        round: None,
    }
}

pub(super) fn refs<S: ToString>(names: impl IntoIterator<Item = S>) -> Vec<syntax::Kid> {
    names.into_iter().map(|n| syntax::Kid::Ref(syntax::Ref::new(n.to_string()))).collect()
}

/// One constraint, written the way the registry names it.
pub fn add_relation(prog: &Program, r: syntax::Relation) -> Edit {
    append(prog, StmtKind::Relation(r), Vec::new())
}
