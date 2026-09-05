//! Evaluate document expression dependencies and maintain free variables.

use super::{eval, literal, parse_in, to_arg_units, to_user_units, Aff, Expr, Free, Parsed};
use crate::constraints::{Arg, Constraint, SpecKind};
use crate::model::Sketch;
use crate::units::Dim;
use std::collections::{BTreeMap, BTreeSet};

/// Why an expression could not be used — sorted by what it means for the document, since the
/// three are not one kind of thing and were reported as one (#43.11).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    /// The number is not what its slot takes — `distance(45deg)`.  An error the spec names
    /// (§3.3: an angle is never coerced to a length or back), and the one place the checker
    /// used to degrade to a warning while every `param` got the error.
    Dimension,
    /// A claim's dimension names a free variable (§9.7).  A claim compiles to no rows, so the
    /// unknown would sit in no equation; warned and zeroed, the claim came back *refuted* by
    /// the number the warning had made up.
    ClaimFree,
    /// It would not compute — a cycle, a name defined twice, a non-number — and the last
    /// number stands, so the solver always has a constant.
    Uncomputable,
}

/// An expression's fault and the words for it.
#[derive(Clone, Debug, PartialEq)]
pub struct ExprError {
    pub fault: Fault,
    pub message: String,
}

impl ExprError {
    fn new(fault: Fault, message: impl Into<String>) -> ExprError {
        ExprError { fault, message: message.into() }
    }
}

/// A bare message is the ordinary fault: it would not compute.
impl From<String> for ExprError {
    fn from(message: String) -> ExprError {
        ExprError::new(Fault::Uncomputable, message)
    }
}

/// Read as its message where only the words matter, so `error.as_deref()` is the text.
impl std::ops::Deref for ExprError {
    type Target = str;
    fn deref(&self) -> &str {
        &self.message
    }
}

impl std::fmt::Display for ExprError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

/// One expression in the document, after evaluation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExprItem {
    /// The constraint it is an argument of, and which argument.
    pub id: u32,
    pub attr: &'static str,
    pub text: String,
    /// The name it defines, if any.
    pub name: Option<String>,
    /// Its value in the units a person reads (degrees for an angle) — the number it last
    /// evaluated to, when `error` is set.
    pub value: f64,
    /// The names it reads.
    pub deps: Vec<String>,
    /// The free names among them — the ones nothing defines, which are unknowns the solver
    /// moves rather than numbers.  At most one, since a dimension can only follow one; a list
    /// because that is what a reader wants to be handed, and because the deps beside it are one.
    pub free: Vec<String>,
    pub error: Option<ExprError>,
}

struct Node {
    ci: usize,
    ai: usize,
    kind: SpecKind,
    parsed: Result<Parsed, String>,
}

/// Evaluate dependencies in topological order, breaking ties by document order.
/// Uncomputable expressions keep their previous values and return diagnostics.
pub fn evaluate(sk: &mut Sketch) -> Vec<ExprItem> {
    let units = sk.units;
    let mut nodes: Vec<Node> = Vec::new();
    // Every binding to a free variable is worked out again from here, so a text that has stopped
    // reading one — or stopped parsing, or stopped being text at all — cannot leave a column
    // behind naming an unknown it no longer has anything to say about.  Clearing all of them and
    // not just the ones still carrying an expression is the point: the constraint that lost its
    // expression is exactly the one whose binding has to go.
    for (ci, c) in sk.constraints.iter_mut().enumerate() {
        c.free = None;
        for (ai, (_, kind)) in c.kind.spec().iter().enumerate() {
            if let Arg::Expr(e) = &c.args[ai] {
                nodes.push(Node { ci, ai, kind: *kind, parsed: parse_in(&e.text, units) });
            }
        }
    }
    let n = nodes.len();
    let mut errors: Vec<Option<ExprError>> = vec![None; n];
    for (i, nd) in nodes.iter().enumerate() {
        if let Err(e) = &nd.parsed {
            errors[i] = Some(e.clone().into());
        }
    }
    // who defines what; a name defined twice is nobody's, and every definer is told
    let mut definers: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    for (i, nd) in nodes.iter().enumerate() {
        if let Ok(Parsed { name: Some(name), .. }) = &nd.parsed {
            definers.entry(name.clone()).or_default().push(i);
        }
    }
    let mut def: BTreeMap<String, usize> = BTreeMap::new();
    for (name, who) in &definers {
        if who.len() == 1 {
            def.insert(name.clone(), who[0]);
        } else {
            for &i in who {
                errors[i] = Some(format!("`{name}` is defined more than once").into());
            }
        }
    }
    // edges: reader ← definer.  A name nothing defines at all is neither an edge nor an error:
    // it is a free variable, and what it is worth is the solver's business.  A name several
    // definitions claim is still an error — it is not undefined, it is ambiguous.
    let deps: Vec<Vec<String>> = nodes
        .iter()
        .map(|nd| match &nd.parsed {
            Ok(p) => p.body.deps().into_iter().collect(),
            Err(_) => Vec::new(),
        })
        .collect();
    let mut readers: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut indeg = vec![0usize; n];
    for i in 0..n {
        for name in &deps[i] {
            match def.get(name) {
                Some(&d) => {
                    readers[d].push(i);
                    indeg[i] += 1;
                }
                None => {
                    if errors[i].is_none() && definers.contains_key(name) {
                        errors[i] = Some(format!("`{name}` is defined more than once").into());
                    }
                }
            }
        }
    }
    // the walk
    let mut ready: BTreeSet<usize> = (0..n).filter(|&i| indeg[i] == 0).collect();
    let mut order: Vec<usize> = Vec::with_capacity(n);
    let mut env: BTreeMap<String, Aff> = BTreeMap::new();
    let mut values: Vec<f64> = nodes
        .iter()
        .map(|nd| to_user_units(nd.kind, sk.constraints[nd.ci].args[nd.ai].num()))
        .collect();
    let mut free_of: Vec<Vec<String>> = vec![Vec::new(); n];
    // the free names actually bound this time round, and what one unit of each is worth in world
    // length — the largest any of its readers makes it, since that is the motion a step buys
    let mut bound: BTreeMap<String, f64> = BTreeMap::new();
    // what each free name turned out to *be*, and the slot that first said so.  A name read by
    // a `Length` dimension and an `Angle` one is an error naming both — the one genuinely new
    // piece of analysis units bring, and the only place a dimension is deduced rather than read.
    let mut free_dim: BTreeMap<String, (Dim, &'static str)> = BTreeMap::new();
    while let Some(&i) = ready.iter().next() {
        ready.remove(&i);
        order.push(i);
        let nd = &nodes[i];
        if errors[i].is_none() {
            let parsed = nd.parsed.as_ref().unwrap();
            let unready =
                deps[i].iter().filter(|d| def.contains_key(*d)).find(|d| !env.contains_key(*d));
            if let Some(name) = unready {
                errors[i] = Some(format!("`{name}` could not be evaluated").into());
            } else {
                // work it out, check what it came to against its slot, write it: three steps
                // that fail the same way, so they are one chain and one error arm
                let done = (|| -> Result<(f64, Aff), ExprError> {
                    let a = eval(&parsed.body, &env)?;
                    check_dim(sk, nd, &a, &mut free_dim)?;
                    Ok((write_value(sk, nd, &a, &mut bound)?, a))
                })();
                match done {
                    Ok((v, a)) => {
                        values[i] = v;
                        free_of[i] = a.free.iter().cloned().collect();
                        if let Some(name) = &parsed.name {
                            // **A name is worth a number, and where that number is *used*
                            // decides what it is.**  `w = 80` in a `Length` slot does not make
                            // `w` a length: the same 80 may be a run, a rise or an angle, and a
                            // document with no `unit` line is in drawing units until something
                            // says otherwise.  `w = 80mm` is how a person says otherwise, and
                            // *that* travels.
                            env.insert(name.clone(), a);
                        }
                    }
                    Err(e) => errors[i] = Some(e),
                }
            }
        }
        for &r in &readers[i] {
            indeg[r] -= 1;
            if indeg[r] == 0 {
                ready.insert(r);
            }
        }
    }
    retire_free(sk, &bound);
    sk.free_dimensions = free_dim
        .into_iter()
        .filter(|(name, _)| bound.contains_key(name))
        .map(|(name, (dim, _))| (name, dim))
        .collect();
    // whatever never became ready is on a cycle, or downstream of one
    let stuck: Vec<usize> = (0..n).filter(|&i| indeg[i] > 0).collect();
    for &i in &stuck {
        if errors[i].is_none() {
            errors[i] = Some(cycle_text(i, &nodes, &deps, &def, &indeg).into());
        }
        order.push(i);
    }
    order
        .into_iter()
        .map(|i| {
            let nd = &nodes[i];
            let c = &sk.constraints[nd.ci];
            ExprItem {
                id: c.id,
                attr: c.spec()[nd.ai].0,
                text: match &c.args[nd.ai] {
                    Arg::Expr(e) => e.text.clone(),
                    _ => String::new(),
                },
                name: nd.parsed.as_ref().ok().and_then(|p| p.name.clone()),
                value: values[i],
                deps: deps[i].clone(),
                free: free_of[i].clone(),
                error: errors[i].clone(),
            }
        })
        .collect()
}

/// Check expression dimensions against the argument slot and infer a consistent
/// dimension for each free variable.
fn check_dim(
    sk: &Sketch,
    nd: &Node,
    a: &Aff,
    free_dim: &mut BTreeMap<String, (Dim, &'static str)>,
) -> Result<(), ExprError> {
    let want = nd.kind.dim();
    let attr = sk.constraints[nd.ci].spec()[nd.ai].0;
    let dim = |m: String| ExprError::new(Fault::Dimension, m);
    let Some(name) = a.free.clone() else {
        return a.dim.require(want, attr).map_err(dim);
    };
    let d = want.div(a.dim);
    if d != Dim::SCALAR && d != Dim::LENGTH && d != Dim::ANGLE {
        return Err(dim(format!(
            "`{name}` would have to be {} here, which is not a length, an angle or a plain number",
            d.name()
        )));
    }
    match free_dim.get(&name) {
        Some(&(was, first)) if was != d => Err(dim(format!(
            "`{name}` is {} in `{first}` and {} in `{attr}` — one free name, one dimension",
            was.name(),
            d.name()
        ))),
        _ => {
            free_dim.insert(name, (d, attr));
            Ok(())
        }
    }
}

/// Store a constant or affine result in argument units and return its user-unit
/// value. Allocate or reuse the solver parameter when the expression is free.
fn write_value(
    sk: &mut Sketch,
    nd: &Node,
    a: &Aff,
    bound: &mut BTreeMap<String, f64>,
) -> Result<f64, ExprError> {
    let (ci, ai, kind) = (nd.ci, nd.ai, nd.kind);
    let text = match &sk.constraints[ci].args[ai] {
        Arg::Expr(e) => e.text.clone(),
        _ => unreachable!("a node is an expression argument"),
    };
    let Some(name) = a.free.clone() else {
        if !a.c.is_finite() {
            return Err("does not evaluate to a number".to_string().into());
        }
        sk.constraints[ci].args[ai] = Arg::Expr(Expr::new(text, to_arg_units(kind, a.c)));
        return Ok(a.c);
    };
    if !a.m.is_finite() || !a.c.is_finite() {
        return Err("does not evaluate to a number".to_string().into());
    }
    // only a stated number can become an unknown, and only where there is a kernel to read it as
    // a column.  The two go together — `every_dimension_can_be_written_free` — so this is the
    // belt to that braces: an expression somewhere it was never meant to be says so rather than
    // selecting a kernel that does not exist.
    if !kind.is_dimension() || sk.constraints[ci].kind.free_kernel().is_none() {
        return Err(format!("`{name}` is free, and this is not a dimension it can be").into());
    }
    // a claim compiles to no rows, so an unknown bound here would sit in no equation at all — a
    // degree of freedom minted by a statement that promised to add nothing
    if sk.constraints[ci].claim {
        return Err(ExprError::new(
            Fault::ClaimFree,
            format!("`{name}` is free, and a claim may not bind an unknown"),
        ));
    }
    // a form that does not actually move with the variable states nothing about it, and there
    // would be no way back from the dimension to a value for it
    if a.m == 0.0 {
        return Err(format!("`{name}` does not affect this dimension").into());
    }
    let stated = sk.constraints[ci].args[ai].num();
    let seed = (to_user_units(kind, stated) - a.c) / a.m;
    let (param, fresh) = free_param(sk, &name, seed);
    let (m, c) = (to_arg_units(kind, a.m), to_arg_units(kind, a.c));
    // one unit of the variable is worth this much world length through this dimension: an angle
    // in degrees moves the drawing by the arm it turns, everything else is a length already
    let reach = m.abs() * if kind == SpecKind::Angle { sk.extent().max(1.0) } else { 1.0 };
    let was = bound.entry(name).or_insert(0.0);
    *was = was.max(reach);
    let free = Free { param, m, c };
    if fresh && stated == 0.0 {
        // the bound copy is what `settle` measures through: the columns and the constants it
        // asks for are the ones this constraint selects once the binding is in
        let mut bound_copy = sk.constraints[ci].clone();
        bound_copy.free = Some(free);
        settle(sk, &bound_copy, param, reach);
    }
    let value = a.at(sk.params[param as usize].value);
    sk.constraints[ci].free = Some(free);
    sk.constraints[ci].args[ai] = Arg::Expr(Expr::new(text, to_arg_units(kind, value)));
    Ok(value)
}

/// Seed a new free variable from its first dimension at the current pose.
/// If the Newton step is nonfinite, kick by one sketch extent in variable units.
fn settle(sk: &mut Sketch, c: &Constraint, param: u32, reach: f64) {
    let kid = c.kernel_id();
    if crate::kernels::kernel_by_id(kid).n_res != 1 {
        return;
    }
    let ps = c.params(sk);
    let col = ps.len() - 1; // the free column comes last — see `Constraint::params_on`
    let consts = c.consts(sk);
    let kick = sk.extent().max(1.0) / if reach.is_finite() && reach > 0.0 { reach } else { 1.0 };
    let err = |sk: &Sketch| {
        let v: Vec<f64> = ps.iter().map(|&p| sk.params[p as usize].value).collect();
        let (r, j) = crate::kernels::eval_one(kid, &v, &consts);
        (r[0], j[col])
    };
    let start = sk.params[param as usize].value;
    let start_err = err(sk).0.abs();
    for _ in 0..24 {
        let (r, dr) = err(sk);
        let here = sk.params[param as usize].value;
        let step = r / dr;
        sk.params[param as usize].value = if step.is_finite() { here - step } else { here + kick };
        if step.is_finite() && step.abs() <= 1e-12 * (1.0 + here.abs()) {
            break;
        }
    }
    // a seed is only a starting point, so a walk that ended worse than it began is discarded
    if !(err(sk).0.abs() < start_err) {
        sk.params[param as usize].value = start;
    }
}

/// Return a free parameter and whether it needs seeding (new or retired).
/// Active parameters retain their solver values across reevaluation.
fn free_param(sk: &mut Sketch, name: &str, seed: f64) -> (u32, bool) {
    if let Some(&p) = sk.free_vars.get(name) {
        let retired = sk.params[p as usize].fixed;
        sk.params[p as usize].fixed = false;
        if retired {
            sk.params[p as usize].value = seed;
        }
        return (p, retired);
    }
    let p = sk.param(seed, false, &format!("${name}")) as u32;
    sk.free_vars.insert(name.to_string(), p);
    (p, true)
}

/// Fix unread free parameters and update scales for active ones. Keep retired
/// names and slots for reuse; only a rebuild reclaims their indices.
fn retire_free(sk: &mut Sketch, bound: &BTreeMap<String, f64>) {
    let gone: Vec<u32> =
        sk.free_vars.iter().filter(|(n, _)| !bound.contains_key(*n)).map(|(_, &p)| p).collect();
    for p in gone {
        sk.params[p as usize].fixed = true;
    }
    for (name, &reach) in bound {
        if let Some(&p) = sk.free_vars.get(name) {
            sk.params[p as usize].scale =
                if reach.is_finite() && reach > 0.0 { reach } else { 1.0 };
        }
    }
}

/// Bring every dimension written in terms of a free variable up to the number that variable now
/// stands at.  The binding is what the kernels read, so a solve needs nothing from this; the
/// *text* of a dimension does, and so does anyone asking what it says without a sketch in hand.
pub fn sync_free(sk: &mut Sketch) {
    if sk.free_vars.is_empty() {
        return; // a document with no free variable in it pays nothing
    }
    let Sketch { constraints, params, .. } = sk;
    for c in constraints {
        let Some(f) = c.free else { continue };
        let v = f.m * params[f.param as usize].value + f.c;
        // a constraint carrying a binding has exactly one dimension, and it is written as text —
        // that is what having one means
        if let Some(Arg::Expr(e)) = c.args.iter_mut().find(|a| matches!(a, Arg::Expr(_))) {
            e.value = v;
        }
    }
}

/// `circular: w → h → w`, found by walking definitions from `i` through the stuck nodes; or,
/// for a node that only reads from a cycle without being on one, which name it waits for.
fn cycle_text(
    i: usize,
    nodes: &[Node],
    deps: &[Vec<String>],
    def: &BTreeMap<String, usize>,
    indeg: &[usize],
) -> String {
    // depth-first along unresolved definitions, looking for a way back to `i`
    let mut path: Vec<usize> = vec![i];
    let mut stack: Vec<(usize, usize)> = vec![(i, 0)]; // (node, next dep index)
    let mut seen: BTreeSet<usize> = BTreeSet::new();
    while let Some(&(u, k)) = stack.last() {
        if k >= deps[u].len() {
            stack.pop();
            path.pop();
            continue;
        }
        stack.last_mut().unwrap().1 += 1;
        let name = &deps[u][k];
        let Some(&d) = def.get(name) else { continue };
        if indeg[d] == 0 {
            continue; // resolved: not part of the tangle
        }
        if d == i {
            let names: Vec<String> = path
                .iter()
                .map(|&p| nodes[p].parsed.as_ref().ok().and_then(|q| q.name.clone()))
                .map(|n| n.unwrap_or_else(|| "?".to_string()))
                .collect();
            return format!("circular: {} → {}", names.join(" → "), names[0]);
        }
        if seen.insert(d) {
            path.push(d);
            stack.push((d, 0));
        }
    }
    let waiting = deps[i]
        .iter()
        .find(|n| def.get(*n).is_some_and(|&d| indeg[d] > 0))
        .cloned()
        .unwrap_or_default();
    format!("`{waiting}` could not be evaluated")
}

/// Write a dimension from text: a bare number becomes a constant (in the argument's units), and
/// anything else an expression, evaluated along with the rest of the document.  `Err` when the
/// text does not parse or names no dimension, and nothing is changed; `Ok(Some(why))` when it
/// was stored but could not be computed (a name nothing defines yet), so a caller can say so.
pub fn set_dimension(
    sk: &mut Sketch,
    id: u32,
    attr: &str,
    text: &str,
) -> Result<Option<String>, String> {
    let (i, kind, value) = {
        let c = sk.constraint(id).ok_or_else(|| format!("no constraint {id}"))?;
        let i = c.arg_index(attr).ok_or_else(|| format!("`{attr}` is not an argument"))?;
        let kind = c.spec()[i].1;
        if !kind.is_dimension() {
            return Err(format!("`{attr}` is not a dimension"));
        }
        (i, kind, c.args[i].num()) // the last number, until the expression is computed
    };
    let text = text.trim();
    if let Some(v) = literal(text) {
        if v < 0.0 && sk.constraint(id).is_some_and(|c| c.kind.magnitude()) {
            return Err(format!(
                "a {} is a magnitude and cannot be negative",
                crate::syntax::snake(sk.constraint(id).unwrap().kind.name())
            ));
        }
        sk.constraint_mut(id).unwrap().args[i] = Arg::Num(to_arg_units(kind, v));
        evaluate(sk); // whatever read a name this used to define
        return Ok(None);
    }
    // the document's units, so typing `6"` into the dimbox works with no change in the app —
    // `set_dimension` is the one write path for a dimension's text
    parse_in(text, sk.units)?;
    sk.constraint_mut(id).unwrap().args[i] = Arg::Expr(Expr::new(text, value));
    let mine = evaluate(sk).into_iter().find(|it| it.id == id && it.attr == attr);
    Ok(mine.and_then(|it| it.error).map(|e| e.message))
}
/// Whether a constraint carries any expression — what decides if adding it needs an evaluation.
pub fn has_expr(args: &[Arg]) -> bool {
    args.iter().any(|a| matches!(a, Arg::Expr(_)))
}
