//! **Parts** (#461, `docs/parts-plan.md` §3, §6): a component declared `part` is a function of
//! what it is given, and the elaborator holds it to that.  Each drawn instance is read as the
//! classifier reads one (`boundary::split`): what it made and the rows its body states, its inputs
//! — every other column those rows read, an unbound numeric formal among them — held.  Then, over
//! the structure of those rows alone, as the diagnosis reads a document's (Dulmage–Mendelsohn):
//!
//! * **PT2, its own unknowns are its inputs' to determine.**  A maximum matching of its rows on
//!   its own unknowns covers every unknown.  One left over is **E108** at the definition, naming
//!   what it moves: the parent would have to place it by reaching inside, and a part's freedom is
//!   a formal instead, which the parent binds or leaves unknown (FR2).
//! * **PT3, no fact about its operands alone.**  A row reading none of its own unknowns, or one
//!   past what they take (the matching's over-determined rows), is **E109** at the row: a verb
//!   fused to a noun, which belongs where the operands are given, or in a relation word (FR1).
//!
//! The structure, and not the numbers: deterministic and decided before any solve.  A claim, a
//! claim block's witness and a bound state no row and are not counted.  Nothing here changes a
//! row: a part is flattened and solved as any component is.  Each fault is said once for its
//! definition, against the first instance it was found in.
//!
//! **The diagnosis, read through the parts** (§6, `read`): what the flat solve's diagnosis says,
//! said again in a part's terms where a part is in it, from the flat solve alone.
//!
//! * **D1, an over-constraint through a part's outputs is the parent's.**  A conflict holding a
//!   part's rows and a parent's row over what the part makes names the parent's row as the one to
//!   remove: the part's rows are why, never the culprit, since they place what it makes from what
//!   it is given.
//! * **D2, a part that does not hold is named, with its inputs.**  Where the solve stopped with a
//!   part's rows unmet, the innermost such part is named with its inputs where they stood.
//! * **D3, a part singular at its inputs.**  Its rows place every unknown it makes structurally
//!   (PT2), and at the solved pose they may still not: that is said, with the rank.

use crate::boundary::{columns, reduced, split, World};
use crate::constraints::{Constraint, SpecKind};
use crate::diagnose::Diagnosis;
use crate::graph::dulmage_mendelsohn;
use crate::json::{self, Json};
use crate::model::{entity_name, EntKind, EntRef, Sketch};
use crate::program::{Code, Diag, Elaborated, Instanced};
use crate::system::{System, RANK_TOL};
use std::collections::{BTreeMap, BTreeSet};

/// How many of the unknowns left free, or of the rows, a message names before it counts the rest.
const NAMED: usize = 6;

/// One drawn instance of a part, read: its rows and unknowns set apart from the document.
struct Read<'e> {
    inst: &'e Instanced,
    /// Its own unknowns: what it made, the numbers no entity owns that its rows read, and the free
    /// variables under its name that are no formal of its own (an inner instance's, unbound).
    own: BTreeSet<u32>,
    /// Its acting rows, by constraint id.
    rows: BTreeSet<u32>,
    /// What else its rows read: what it is given.
    inputs: BTreeSet<u32>,
}

/// Every drawn instance of a part in `sk`, read.
fn instances<'e>(e: &'e Elaborated, sk: &Sketch) -> Vec<Read<'e>> {
    let prog = &e.program;
    if !prog.components.iter().any(|c| c.part) {
        return Vec::new();
    }
    let world = World::of(sk, &e.map);
    let by_id: BTreeMap<u32, &Constraint> = sk.constraints.iter().map(|c| (c.id, c)).collect();
    let zero = sk.zero;
    e.map.instances.iter().filter(|i| i.drawn && prog.components[i.comp].part).map(|inst| {
        let comp = &prog.components[inst.comp];
        let s = split(&e.map, sk, &world, inst);
        let rows: BTreeSet<u32> = s.kept.iter().copied()
            .filter(|id| by_id.get(id).is_some_and(|c| c.acts()))
            .collect();
        let formals: BTreeSet<String> =
            comp.formals.iter().map(|f| format!("{}{}", inst.prefix, f.name.text)).collect();
        let mut own = s.own;
        own.extend(sk.free_vars.iter()
            .filter(|(k, _)| k.starts_with(&inst.prefix) && !formals.contains(*k))
            .map(|(_, &p)| p));
        let inputs = rows.iter().filter_map(|id| by_id.get(id))
            .flat_map(|c| columns(sk, c))
            .filter(|p| !own.contains(p) && Some(*p) != zero)
            .collect();
        Read { inst, own, rows, inputs }
    }).collect()
}

/// The document with only part `r`'s rows, its inputs held.
fn alone(sk: &Sketch, r: &Read) -> Sketch {
    reduced(sk, &r.rows, |p, fixed| fixed || !r.own.contains(&p))
}

/// The entity that owns parameter `p`, by the document's name, or the free variable it is.
fn owner_name(e: &Elaborated, sk: &Sketch, world: &World, p: u32) -> Option<String> {
    if let Some(&r) = world.owner.get(&p) {
        return Some(e.map.name_of(r).cloned().unwrap_or_else(|| entity_name(r)));
    }
    sk.free_vars.iter().find(|(_, &q)| q == p).map(|(k, _)| k.clone())
}

/// `items` as a list, the first `NAMED` of them and how many more.
fn listed(items: Vec<String>) -> String {
    let n = items.len();
    let mut out = items.into_iter().take(NAMED).collect::<Vec<_>>().join(", ");
    if n > NAMED {
        out.push_str(&format!(" and {} more", n - NAMED));
    }
    out
}

/// What is wrong with each part the document instantiates, as diagnostics (PT2, PT3).
pub fn check(e: &Elaborated) -> Vec<Diag> {
    let prog = &e.program;
    let sk = &e.sketch;
    let map = &e.map;
    let read = instances(e, sk);
    if read.is_empty() {
        return Vec::new();
    }
    let world = World::of(sk, map);
    let name_of = |r: EntRef| map.name_of(r).cloned();
    // the document's structure, compiled once: each part's is its rows' over its own columns
    let sys = System::new(sk);
    let (all, all_c) = sys.structure();
    let mut rows_of: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
    for (k, &id) in all_c.iter().enumerate() {
        rows_of.entry(id).or_default().push(k);
    }
    let mut out: Vec<Diag> = Vec::new();
    let mut said: BTreeSet<(usize, &str, String)> = BTreeSet::new();
    for r in &read {
        let inst = r.inst;
        let comp = &prog.components[inst.comp];
        let word = prog.component_id(inst.comp);
        let at = comp.name.as_ref().map(|n| n.span).unwrap_or(comp.span);
        let instance = inst.prefix.trim_end_matches('.');
        // its own columns, numbered afresh, and its rows over them alone
        let cols: Vec<usize> = r.own.iter()
            .filter_map(|&p| usize::try_from(sys.col_of[p as usize]).ok())
            .collect::<BTreeSet<usize>>().into_iter().collect();
        let local: BTreeMap<usize, usize> = cols.iter().enumerate().map(|(k, &c)| (c, k)).collect();
        let mut adj: Vec<Vec<usize>> = Vec::new();
        let mut row_c: Vec<u32> = Vec::new();
        for &id in &r.rows {
            for &k in rows_of.get(&id).map(Vec::as_slice).unwrap_or(&[]) {
                adj.push(all[k].iter().filter_map(|c| local.get(c).copied()).collect());
                row_c.push(id);
            }
        }
        let dm = dulmage_mendelsohn(&adj, cols.len());

        // PT2: an own unknown no row reaches, the matching's unmatched columns
        let mut free: Vec<String> = Vec::new();
        let mut left = 0usize;
        for col in (0..cols.len()).filter(|&c| dm.mate_col[c] < 0) {
            left += 1;
            let what = owner_name(e, sk, &world, sys.free[cols[col]] as u32)
                .map_or("a number its rows read".to_string(), |n| format!("`{n}`"));
            if !free.contains(&what) {
                free.push(what);
            }
        }
        if left > 0 && said.insert((inst.comp, "E108", String::new())) {
            out.push(Diag::new(Code::E108, at, None, format!(
                "part `{word}` leaves {left} of its own unknowns free, moving {} (in \
                 `{instance}`): a part is a function of what it is given — state what places \
                 them inside it, or make the freedom a formal the parent binds or leaves unknown",
                listed(free))));
        }

        // PT3: a row of no own unknown, or past what they take
        let over: BTreeSet<usize> = dm.over_rows.iter().copied().collect();
        let mut fused: Vec<u32> = Vec::new();
        for (k, cols) in adj.iter().enumerate() {
            if (cols.is_empty() || over.contains(&k)) && !fused.contains(&row_c[k]) {
                fused.push(row_c[k]);
            }
        }
        for id in fused {
            let Some(c) = sk.constraint(id) else { continue };
            let text = crate::io::describe_with(c, &name_of);
            let site = map.site_of_constraint(id);
            let message = match columns(sk, c).iter().any(|p| r.own.contains(p)) {
                false => format!(
                    "part `{word}` states `{text}`, a fact about what it is given and not about \
                     anything it makes (in `{instance}`): state it where those are given, or as \
                     a relation word"),
                true => format!(
                    "part `{word}` states `{text}` past what its own unknowns take (in \
                     `{instance}`): a part's rows determine what it makes, and one more says \
                     something of what it is given"),
            };
            if said.insert((inst.comp, "E109", text)) {
                out.push(Diag::new(Code::E109, site.map_or(at, |s| s.span), site.map(|s| s.stmt),
                    message));
            }
        }
    }
    out
}

/// What the diagnosis says of a drawing's parts (D1–D3), in the core's words.
#[derive(Clone, Debug, Default)]
pub struct Reading {
    /// How many part instances the drawing has, and of how many definitions.
    pub instances: usize,
    pub definitions: usize,
    /// D1: each parent's row that over-constrains a part, said as the one to remove.
    pub over: Vec<String>,
    /// D2: each innermost part whose rows the solve left unmet, with its inputs where they stood.
    pub unmet: Vec<String>,
    /// D3: each part its rows do not place at its inputs as they stand, with the rank.
    pub singular: Vec<String>,
}

/// The diagnosis `d` of the solved sketch `sk`, read through `e`'s parts.  `settled` says whether
/// the solve settled: a part is judged singular only at a pose that is one.
pub fn read(e: &Elaborated, sk: &Sketch, d: &Diagnosis, settled: bool) -> Reading {
    let prog = &e.program;
    let read = instances(e, sk);
    let mut out = Reading { instances: read.len(), ..Reading::default() };
    if read.is_empty() {
        return out;
    }
    out.definitions = read.iter().map(|r| r.inst.comp).collect::<BTreeSet<_>>().len();
    let world = World::of(sk, &e.map);
    let name_of = |r: EntRef| e.map.name_of(r).cloned();
    let text = |id: u32| {
        sk.constraint(id).map(|c| format!("`{}`", crate::io::describe_with(c, &name_of)))
    };
    let label = |r: &Read| format!("part `{}` (`{}`)", r.inst.prefix.trim_end_matches('.'),
        prog.component_id(r.inst.comp));
    let inputs = |r: &Read| -> Vec<String> {
        let mut names: Vec<String> = Vec::new();
        for &p in &r.inputs {
            if let Some(n) = owner_name(e, sk, &world, p).map(|n| format!("`{n}`")) {
                if !names.contains(&n) {
                    names.push(n);
                }
            }
        }
        names
    };

    // D1: a conflict holding a part's rows and a row of the parent's over what it makes
    let conflict: Vec<u32> = d.conflicts.clone().unwrap_or_else(|| d.over.clone());
    let mut explained: BTreeSet<&str> = BTreeSet::new();
    for r in &read {
        let inside: Vec<u32> = conflict.iter().copied().filter(|id| r.rows.contains(id)).collect();
        if inside.is_empty() {
            continue;
        }
        let through: Vec<u32> = conflict.iter().copied()
            .filter(|id| !r.rows.contains(id))
            .filter(|&id| sk.constraint(id)
                .is_some_and(|c| columns(sk, c).iter().any(|p| r.own.contains(p))))
            .collect();
        for id in through {
            let Some(row) = text(id) else { continue };
            explained.insert(r.inst.prefix.as_str());
            out.over.push(format!(
                "{row} over-constrains {}: what it makes is placed by what it is given ({}), \
                 so remove this, not {}", label(r), listed(inputs(r)),
                listed(inside.iter().filter_map(|&k| text(k)).collect())));
        }
    }

    // D2: the innermost parts whose rows the solve left unmet, where no row of a parent's
    // over-constraining them is the reason already said
    let violated: BTreeSet<u32> = d.violated.iter().copied().collect();
    let unmet: Vec<&Read> = read.iter()
        .filter(|r| r.rows.iter().any(|id| violated.contains(id)))
        .filter(|r| !explained.contains(r.inst.prefix.as_str()))
        .collect();
    for r in &unmet {
        let nested = unmet.iter().any(|o| o.inst.path.0.len() > r.inst.path.0.len()
            && r.inst.contains(&o.inst.path));
        if nested {
            continue;
        }
        let rows: Vec<String> =
            r.rows.iter().filter(|id| violated.contains(id)).filter_map(|&id| text(id)).collect();
        let mut at: Vec<String> = Vec::new();
        let mut seen: BTreeSet<EntRef> = BTreeSet::new();
        for &p in &r.inputs {
            let Some(&ent) = world.owner.get(&p) else { continue };
            if !seen.insert(ent) {
                continue;
            }
            let n = e.map.name_of(ent).cloned().unwrap_or_else(|| entity_name(ent));
            let v = |x: f64| crate::expr::reading(SpecKind::Scalar, x);
            at.push(match ent.kind {
                EntKind::Point => {
                    let q = sk.world_point(ent.i());
                    format!("`{n}` at ({}, {}, {})", v(q[0]), v(q[1]), v(q[2]))
                }
                _ => format!("`{n}`"),
            });
        }
        out.unmet.push(format!("{} does not hold where the solve stopped: {}; what it is given \
            there: {}", label(r), listed(rows), listed(at)));
    }

    // D3: a part its rows do not place at its inputs as they stand
    if settled {
        for r in &read {
            let alone = alone(sk, r);
            let mut sys = System::new(&alone);
            let n = sys.n_free;
            if n == 0 || sys.hard_rows().is_empty() {
                continue;
            }
            let z = sys.z0(&alone);
            let rank = sys.conditioned(&z).rank_and_nullspace(RANK_TOL).rank.min(n);
            if rank < n {
                out.singular.push(format!("{} is singular at what it is given: its rows place \
                    {rank} of its {n} own unknowns here, so a small change to its inputs may move \
                    it far or find no pose", label(r)));
            }
        }
    }
    out
}

/// `read`'s lines, as `solventc` prints them.
pub fn text(r: &Reading) -> Vec<String> {
    if r.instances == 0 {
        return Vec::new();
    }
    let mut out = vec![format!("parts: {} instance{} of {} definition{}", r.instances,
        if r.instances == 1 { "" } else { "s" }, r.definitions,
        if r.definitions == 1 { "" } else { "s" })];
    out.extend(r.over.iter().map(|l| format!("part conflict: {l}")));
    out.extend(r.unmet.iter().map(|l| format!("part unmet: {l}")));
    out.extend(r.singular.iter().map(|l| format!("part singular: {l}")));
    out
}

/// `read` as JSON, under `--json`'s `"parts"`.
pub fn json(r: &Reading) -> Json {
    let lines = |v: &[String]| Json::Arr(v.iter().map(|s| Json::Str(s.clone())).collect());
    json::object([
        ("instances", Json::Int(r.instances as i64)),
        ("definitions", Json::Int(r.definitions as i64)),
        ("over", lines(&r.over)),
        ("unmet", lines(&r.unmet)),
        ("singular", lines(&r.singular)),
    ])
}
