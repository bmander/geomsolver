//! **A followed drawing a relation holds to** (#398 A): `trace(m over t)` as a curve the solve
//! works through.
//!
//! A point of the drawing followed along a declared parameter (`follow.rs`) is drawn from the
//! march.  To hold a relation to it — a point on it at an instant, a line tangent to it, a circle
//! osculating it — the solve needs the point at any instant `τ`, its derivatives in `τ`, and its
//! sensitivities to the drawing's own unknowns, as columns.  All three are what a curve traced
//! from a drawn component already gives (`locus.rs`): its point solved from the component's rows at
//! the place, its derivatives in the place by the implicit function theorem on those rows (to the
//! third order, off their Taylor forms, nothing differenced), its sensitivities to the numbers the
//! instance left unknown the same theorem's other columns.  So a followed point is **lowered to a
//! locus**, its block the drawing itself:
//!
//! - *its rows* are the drawing's relations that move with the parameter: those reading it
//!   (`follow::readers`) and every row joined to them through an unknown of the drawing, a row held
//!   to a followed point aside — a design that is solved through the trace, and not the trace;
//! - *its inner unknowns* (`q`) are those rows' unknowns, the coordinates and radii the march
//!   moves, each an entity's own scalar, so the drawing's pose there is the locus's pose;
//! - *its columns* (`θ`) are the drawing's free values those rows read (`leg.h`, an unbound formal
//!   of an instance or a declared unknown): numbers that do not run, the same at every instant,
//!   which a relation held to the trace may solve; a row that reads one does not join it to the
//!   rest, since it holds the number rather than moving it;
//! - *its values* are the fixed numbers those rows read (a fixed point's coordinates, a held
//!   number), as they stand;
//! - *its place* is the parameter: a row that reads it (`ground angle(t) crank`) is its kind's
//!   free twin over a tape of its text (`w`), as a component trace's dimension over its formal is.
//!
//! **One solve** (the issue's order problem): the drawing stands at the instant the parameter is
//! held at, and the locus is anchored there (`Sketch::curve_home`), its pose the drawing's own,
//! read off the sheet at every compile and refresh.  So a solve holding a relation to the trace
//! solves the drawing where it stands and the trace from where it stands together: the trace at `τ`
//! is a march from the pose to `τ`, remembered per contact and resumed (`locus::eval_at`), its
//! sensitivities columns of the contact's row, and none of the drawing's `q`.  A march of the
//! drawing (`follow::follow`) solves the held relation at each station as a row like any other,
//! anchored at that station — the same trace, read from where the drawing stands there.
//!
//! What is lowered once is the structure; every number the rows read but the parameter's is the
//! sketch's at the moment (the pose, the columns), and a fixed number's is baked: the drawing is
//! elaborated again when its text changes.

use std::collections::{BTreeMap, BTreeSet};

use crate::constraints::{Arg, Constraint};
use crate::kernels::KernelKey;
use crate::model::{CurveBody, CurveDef, EntKind, EntRef, Sketch};

/// The place-holder column a row reading the parameter is lowered over before its tape has a slot.
const PLACE: u32 = u32::MAX;

/// Whether constraint `c` holds to a point followed along a parameter.
fn holds_followed(sk: &Sketch, c: &Constraint) -> bool {
    c.args.iter().any(|a| match a {
        Arg::Ent(e) => {
            e.kind == EntKind::Curve && e.i() < sk.curves.len() && sk.curve_followed(e.i())
        }
        _ => false,
    })
}

/// **Lower curve `i`, a point followed along its parameter, to the locus a relation holds to**
/// (`CurveBody::Followed(Some(_))`): its own definition, and the curve's columns (the free values
/// its rows read, `CurveE::unknowns`), values and pose (each inner unknown's entity and scalar).
/// Why it cannot be, where it cannot: the curve is then left as drawn, held to by nothing.
pub fn lower(sk: &mut Sketch, i: usize) -> Result<(), String> {
    let cv = &sk.curves[i];
    let Some(&point) = cv.args.first().filter(|e| e.kind == EntKind::Point) else {
        return Err("a followed curve traces a point".into());
    };
    let name = sk.curve_defs[cv.def as usize].param.clone();
    let held = sk.held.get(&name).map(|h| h.param)
        .ok_or_else(|| format!("`{name}` is no parameter the drawing holds"))?;
    let readers: BTreeSet<u32> = crate::follow::readers(sk, &name).into_iter().collect();
    if readers.is_empty() {
        return Err(format!("nothing the drawing states reads `{name}`"));
    }
    let free: BTreeMap<u32, &String> = sk.free_vars.iter().map(|(n, &p)| (p, n)).collect();
    let derived: BTreeSet<u32> = sk.derived().iter().map(|d| d.param).collect();
    // what moves: an unknown of the drawing that is no free value, nor the parameter itself
    let moves = |p: u32| {
        let q = &sk.params[p as usize];
        !q.fixed && !free.contains_key(&p) && p != held
    };
    // the rows the drawing solves, but those held to a followed point and the soft ones a drag adds
    let rows: Vec<usize> = (0..sk.constraints.len()).filter(|&k| {
        let c = &sk.constraints[k];
        c.states_rows() && !c.soft && c.rows_in(sk) > 0 && !holds_followed(sk, c)
    }).collect();
    let cols: Vec<Vec<u32>> = rows.iter().map(|&k| sk.constraints[k].params(sk)).collect();
    // the rows joined to the readers through what moves: a march's whole block
    let mut by_param: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
    for (r, ps) in cols.iter().enumerate() {
        for &p in ps.iter().filter(|&&p| moves(p)) {
            by_param.entry(p).or_default().push(r);
        }
    }
    let mut taken = vec![false; rows.len()];
    let mut queue: Vec<usize> = (0..rows.len())
        .filter(|&r| readers.contains(&sk.constraints[rows[r]].id)).collect();
    for &r in &queue {
        taken[r] = true;
    }
    let mut moved: BTreeSet<u32> = BTreeSet::new();
    while let Some(r) = queue.pop() {
        for &p in cols[r].iter().filter(|&&p| moves(p)) {
            if moved.insert(p) {
                for &s in &by_param[&p] {
                    if !taken[s] {
                        taken[s] = true;
                        queue.push(s);
                    }
                }
            }
        }
    }
    let block: Vec<usize> = (0..rows.len()).filter(|&r| taken[r]).collect();
    // each inner unknown an entity's own scalar, in the drawing's order, so a point's two are
    // side by side and its pose reads off the sheet
    let mut pose: Vec<(EntRef, usize)> = Vec::new();
    let owners = (0..sk.points.len()).map(EntRef::point)
        .chain((0..sk.circles.len()).map(EntRef::circle))
        .chain((0..sk.arcs.len()).map(EntRef::arc));
    let mut slot: BTreeMap<u32, usize> = BTreeMap::new();
    for e in owners {
        for (j, p) in sk.own_params(e).into_iter().enumerate() {
            if moved.contains(&p) && !slot.contains_key(&p) {
                slot.insert(p, pose.len());
                pose.push((e, j));
            }
        }
    }
    if let Some(p) = moved.iter().find(|p| !slot.contains_key(p)) {
        return Err(format!("it moves `{}`, a number no point, circle or arc of the drawing owns, \
                            which a curve held to cannot carry yet", sk.params[*p as usize].name));
    }
    let traced = match sk.point_params(point.i()) {
        [x, y] if slot.get(&x).is_some_and(|&s| slot.get(&y) == Some(&(s + 1))) => slot[&x],
        _ => return Err(format!("the point it traces does not move with `{name}`")),
    };
    // the columns, the free values the block reads, by name; and the values, the fixed numbers
    let mut columns: BTreeSet<&String> = BTreeSet::new();
    let mut values: BTreeSet<u32> = BTreeSet::new();
    for &r in &block {
        for &p in &cols[r] {
            if derived.contains(&p) {
                return Err("it reads a turned copy's number, which a curve held to cannot carry \
                            yet".into());
            }
            if let Some(n) = free.get(&p) {
                columns.insert(n);
            } else if !moved.contains(&p) {
                values.insert(p);
            }
        }
    }
    let columns: Vec<String> = columns.into_iter().cloned().collect();
    let value_names: Vec<String> = values.iter().map(|p| format!("#value{p}")).collect();
    let vars: Vec<String> = std::iter::once(name.clone()).chain(columns.iter().cloned())
        .chain(value_names.iter().cloned()).collect();
    if vars.len() > crate::tape::MAX_VARS {
        return Err(format!("its rows read {} numbers that do not move with `{name}`, and a curve \
                            held to reads at most {}", vars.len() - 1, crate::tape::MAX_VARS - 1));
    }
    let n_outer = vars.len();
    let n_q = pose.len();
    let outer: BTreeMap<u32, usize> = columns.iter().enumerate()
        .map(|(k, n)| (sk.free_vars[n], 1 + k))
        .chain(values.iter().enumerate().map(|(k, &p)| (p, 1 + columns.len() + k)))
        .collect();
    let at = |p: u32| outer.get(&p).copied().or_else(|| slot.get(&p).map(|s| n_outer + s));
    let mut w: Vec<crate::tape::Tape> = Vec::new();
    let mut lowered: Vec<crate::locus::Row> = Vec::with_capacity(block.len());
    for &r in &block {
        let c = &sk.constraints[rows[r]];
        if !matches!(c.kernel_key(sk), KernelKey::Static(_)) || c.along.is_some() {
            return Err(format!("it holds {}, which a curve held to cannot carry yet",
                               crate::model::article(&crate::vocab::snake(c.kind.name()))));
        }
        // a row reading the parameter is its kind's free twin over its text's tape, the text in
        // the units a person writes and `(m, c)` the conversion, as a component trace's is
        let (c, tape) = match crate::follow::reading_slot(sk, c, &name) {
            Some(slot) => {
                if c.kind.dimension_slot() != Some(slot) || c.kind.free_kernel().is_none() {
                    let what = crate::model::article(&crate::vocab::snake(c.kind.name()));
                    return Err(format!("{what} reads `{name}` where no number of it runs"));
                }
                let Arg::Expr(e) = &c.args[slot] else { unreachable!("`reads` finds a text") };
                let ast = crate::expr::parse_in(&e.text, sk.units)?;
                let tape = crate::tape::Tape::compile(&ast.body, &vars)
                    .map_err(|m| format!("`{}`: {m}", e.text))?;
                let mut twin = c.clone();
                let m = crate::expr::to_arg_units(c.arg_kind(slot), 1.0);
                twin.free = Some(crate::expr::Free { param: PLACE, m, c: 0.0 });
                (twin, Some(tape))
            }
            None => (c.clone(), None),
        };
        let mut row_cols = Vec::new();
        for p in c.params(sk) {
            let s = match p {
                PLACE => n_outer + n_q + w.len(),
                p => at(p).ok_or("lowering lost a column")?,
            };
            row_cols.push(s as u32);
        }
        w.extend(tape);
        let consts = c.consts_on(sk, None);
        lowered.push(crate::locus::Row { kid: c.kernel_id(), cols: row_cols, consts });
    }
    let number = |v: f64| crate::expr::Ast::Num(v, crate::units::Dim::SCALAR);
    let seed = |v: f64| crate::tape::Tape::compile(&number(v), &vars)
        .expect("a number always compiles");
    let seeds: Vec<crate::tape::Tape> = pose.iter()
        .map(|&(e, j)| seed(sk.own_param(e, j).map_or(0.0, |p| sk.params[p as usize].value)))
        .collect();
    // the block places what moves, all of it: a number the rows leave free does not run, and is
    // the drawing's to solve through the trace as a free value, which no coordinate is
    let n_res: usize = lowered.iter().map(|r| crate::kernels::KERNELS[r.kid].n_res).sum();
    if n_res < n_q {
        let n = n_q - n_res;
        let them = if n == 1 { "it" } else { "each" };
        return Err(format!("the rows that move with `{name}` leave {n} of the numbers they place \
                            free: a number solved through the trace is a free value — declare \
                            {them} (`param l: Length`), or leave {them} an instance's unbound \
                            number"));
    }
    let rates = crate::locus::Rates { rows: Vec::new(), init: Vec::new(), pose: false };
    let locus = crate::locus::Locus::new(n_outer, columns.len(), n_q, traced, w, seeds, lowered,
                                         Vec::new(), rates)?;
    let value_numbers: Vec<f64> = values.iter().map(|&p| sk.params[p as usize].value).collect();
    let cv = &sk.curves[i];
    let shared = &sk.curve_defs[cv.def as usize];
    let def = CurveDef {
        name: format!("followed:{name}#{i}"),
        component: String::new(),
        port: String::new(),
        formals: shared.formals.clone(),
        columns: columns.clone(),
        values: value_names,
        param: name,
        turns: shared.turns,
        vars,
        body: CurveBody::Followed(Some(locus)),
        pose_of: Vec::new(),
        home: false,
    };
    let at = match sk.curve_defs.iter().position(|d| d.name == def.name) {
        Some(k) => {
            sk.curve_defs[k] = def;
            k
        }
        None => {
            sk.curve_defs.push(def);
            sk.curve_defs.len() - 1
        }
    };
    let cv = &mut sk.curves[i];
    cv.def = at as u32;
    cv.unknowns = columns;
    cv.values = value_numbers;
    cv.pose = pose;
    Ok(())
}

