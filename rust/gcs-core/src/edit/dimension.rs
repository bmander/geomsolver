//! Editing a number: a dimension's text, or the value it reads.

use super::*;

/* -- editing a number -------------------------------------------------------------- */

/// Write a dimension's text — a number, or an expression somebody typed.
///
/// `Kind::Numeric` when the text is a plain number and was one before: the topology cannot have
/// moved, so a compiled plan survives.  A text that names anything is `Structural`, because the
/// name may be an unknown, and that *is* a column.  A number typed over a dimension that reads a
/// `param` (`distance(w)`) is written into the param's own line.
pub fn set_dimension(e: &Elaborated, prog: &Program, cid: u32, attr: &str, text: &str) -> Edit {
    let Some(site) = e.map.of_constraint.get(&cid) else {
        return Edit::none(prog, Some("no such constraint".into()));
    };
    if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
        return Edit::none(prog, Some("that dimension is written inside a component".into()));
    }
    let Some(st) = prog.stmt(site.stmt) else { return Edit::none(prog, None) };
    // the number a statement states is the unlabelled thing in its operator's parentheses
    // (spec §9.1), which is where a *written* statement carries it — and a set drawn as a circle
    // states its radius as its body's distance (#105): written in place, the body's number; a
    // family's instance, the number its call gives the formal the body reads
    let dim = match &st.kind {
        StmtKind::Relation(rel) => match &rel.form {
            syntax::RelationForm::Written(w) => w.args.iter().find_map(|a| match a {
                syntax::OpArg::Dim(text, span) => Some((*span, text.clone())),
                _ => None,
            }).or_else(|| word_dimension(prog, w)),
            syntax::RelationForm::Canonical { kind, args, .. } => kind
                .spec()
                .iter()
                .position(|(n, _)| *n == attr)
                .and_then(|i| args.get(i).and_then(|a| a.as_ref()))
                .and_then(|a| match a {
                    syntax::Arg::Dim { span, text } => Some((*span, text.clone())),
                    _ => None,
                }),
        },
        StmtKind::Set(set) => crate::lowering::shape(&set.lit)
            .map(|Shape::Circle { radius, .. }| (radius.span, radius.text.to_string())),
        StmtKind::Instance(inst) => prog.resolve_component(&inst.component.text, None).ok()
            .and_then(|k| prog.components[k].set.as_ref())
            .and_then(crate::lowering::shape)
            .and_then(|Shape::Circle { radius, .. }| radius.at(Some(inst)))
            .map(|at| (at, at.slice(prog.text()).to_string())),
        _ => None,
    };
    let Some((span, was)) = dim else {
        return Edit::none(prog, Some("that argument is not a dimension".into()));
    };
    let was = was.as_str();
    // a dimension that reads a `param` (`distance(w)`) is edited where the number is: a plain
    // number typed over it is the param's new value, and every dimension reading it follows
    if writable_seed(text) {
        let param = prog.root().body.iter().find_map(|st| match &st.kind {
            StmtKind::Param(p) if p.name.text == was.trim() && p.bound() => Some(p),
            _ => None,
        });
        if let Some(p) = param {
            let value = Splice { at: p.span, with: text.trim().to_string() };
            return Edit::spliced(prog, vec![value], Kind::Structural);
        }
    }
    let plain = crate::expr::literal(text).is_some() && crate::expr::literal(was).is_some();
    let number = Splice { at: span, with: text.trim().to_string() };
    Edit::spliced(prog, vec![number], if plain { Kind::Numeric } else { Kind::Structural })
}

/// **A defined word's number, where the statement wrote it** (§9.9): where the word's body
/// states its dimension as one of its parameters (`a above(d) b := b distance(d, along: up) a`),
/// the argument the statement gave that parameter (`r above(d: 7) q`'s `7`) — the text the
/// callout draws, so the text an edit of it writes.
fn word_dimension(prog: &Program, w: &syntax::Written) -> Option<(Span, String)> {
    let k = prog.resolve_word(&w.word.text, w.fixity, None)?;
    let param = prog.words[k].body.iter().find_map(|st| match &st.kind {
        StmtKind::Relation(r) => r.form.written()?.args.iter().find_map(|a| match a {
            syntax::OpArg::Dim(t, _) => Some(t.trim()),
            _ => None,
        }),
        _ => None,
    })?;
    w.args.iter().find_map(|a| match a {
        syntax::OpArg::Named(n, syntax::Arg::Dim { text, span }) if n.text == param => {
            Some((*span, text.clone()))
        }
        _ => None,
    })
}
