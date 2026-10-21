//! **Parts** (#461, `docs/parts-plan.md` §3): a component declared `part` is a function of what it
//! is given, and the elaborator holds it to that.  Each drawn instance is read as the classifier
//! reads one (`boundary::split`): what it made and the rows its body states, its inputs — every
//! other column those rows read, an unbound numeric formal among them — held.  Then, over the
//! structure of those rows alone, as the diagnosis reads a document's (Dulmage–Mendelsohn):
//!
//! * **PT2, its own unknowns are its inputs' to determine.**  A maximum matching of its rows on
//!   its own unknowns covers every unknown.  One left over is **E108** at the definition, naming
//!   what it moves: the parent would have to place it by reaching inside, and a part's freedom is
//!   a formal instead, which the parent binds or leaves unknown (FR2).
//! * **PT3, no fact about its operands alone.**  A row reading none of its own unknowns, or one
//!   past what they take (the matching's over-determined rows), is **E109** at the row: a verb
//!   fused to a noun, which belongs where the operands are given, or in a relation word (FR1).
//!
//! The structure, and not the numbers: deterministic and decided before any solve.  A part
//! singular at its inputs is the diagnosis's to say, where it reads the solved pose.  A claim, a
//! claim block's witness and a bound state no row and are not counted.  Nothing here changes a
//! row: a part is flattened and solved as any component is.  Each fault is said once for its
//! definition, against the first instance it was found in.

use crate::boundary::{reduced, split, World};
use crate::graph::dulmage_mendelsohn;
use crate::model::{entity_name, EntRef};
use crate::program::{Code, Diag, Elaborated};
use crate::system::System;
use std::collections::{BTreeMap, BTreeSet};

/// How many of the unknowns left free, or of the rows, a message names before it counts the rest.
const NAMED: usize = 6;

/// What is wrong with each part the document instantiates, as diagnostics.
pub fn check(e: &Elaborated) -> Vec<Diag> {
    let prog = &e.program;
    let parts: BTreeSet<usize> =
        (0..prog.components.len()).filter(|&c| prog.components[c].part).collect();
    if parts.is_empty() {
        return Vec::new();
    }
    let sk = &e.sketch;
    let map = &e.map;
    let world = World::of(sk, map);
    let by_id: BTreeMap<u32, &crate::constraints::Constraint> =
        sk.constraints.iter().map(|c| (c.id, c)).collect();
    let name_of = |r: EntRef| map.name_of(r).cloned();
    let ent_name = |r: EntRef| name_of(r).unwrap_or_else(|| entity_name(r));
    let mut out: Vec<Diag> = Vec::new();
    let mut said: BTreeSet<(usize, &str, String)> = BTreeSet::new();
    for inst in map.instances.iter().filter(|i| i.drawn && parts.contains(&i.comp)) {
        let comp = &prog.components[inst.comp];
        let word = prog.component_id(inst.comp);
        let at = comp.name.as_ref().map(|n| n.span).unwrap_or(comp.span);
        let instance = inst.prefix.trim_end_matches('.');
        let s = split(map, sk, &world, inst);
        let acting: BTreeSet<u32> = s.kept.iter().copied()
            .filter(|id| by_id.get(id).is_some_and(|c| c.acts()))
            .collect();
        // its own unknowns: what it made, and the numbers its body declares unknown (`param t:
        // Angle`, or a formal of an instance inside left unbound) — its formals are inputs
        let formals: BTreeSet<String> =
            comp.formals.iter().map(|f| format!("{}{}", inst.prefix, f.name.text)).collect();
        let declared: BTreeSet<u32> = sk.free_vars.iter()
            .filter(|(k, _)| k.starts_with(&inst.prefix) && !formals.contains(*k))
            .map(|(_, &p)| p)
            .collect();
        let own = |p: u32| s.own.contains(&p) || declared.contains(&p);
        let alone = reduced(sk, &acting, |p, fixed| fixed || !own(p));
        let sys = System::new(&alone);
        let (adj, row_c) = sys.structure();
        let dm = dulmage_mendelsohn(&adj, sys.n_free);

        // PT2: an own unknown no row reaches, the matching's unmatched columns
        let mut free: Vec<String> = Vec::new();
        let mut left = 0usize;
        for col in (0..sys.n_free).filter(|&c| dm.mate_col[c] < 0) {
            left += 1;
            let p = sys.free[col] as u32;
            let what = match (world.owner.get(&p), sk.free_vars.iter().find(|(_, &q)| q == p)) {
                (Some(&r), _) => format!("`{}`", ent_name(r)),
                (None, Some((k, _))) => format!("`{k}`"),
                (None, None) => "a number its rows read".to_string(),
            };
            if !free.contains(&what) {
                free.push(what);
            }
        }
        if left > 0 {
            let n = free.len();
            let mut named = free.into_iter().take(NAMED).collect::<Vec<_>>().join(", ");
            if n > NAMED {
                named.push_str(&format!(" and {} more", n - NAMED));
            }
            let message = format!(
                "part `{word}` leaves {left} of its own unknowns free, moving {named} (in \
                 `{instance}`): a part is a function of what it is given — state what places \
                 them inside it, or make the freedom a formal the parent binds or leaves unknown");
            if said.insert((inst.comp, "E108", String::new())) {
                out.push(Diag::new(Code::E108, at, None, message));
            }
        }

        // PT3: a row of no own unknown, or past what they take
        let over: BTreeSet<usize> = dm.over_rows.iter().copied().collect();
        let mut rows: Vec<u32> = Vec::new();
        for (r, cols) in adj.iter().enumerate() {
            let id = row_c[r];
            if (cols.is_empty() || over.contains(&r)) && !rows.contains(&id) {
                rows.push(id);
            }
        }
        for id in rows {
            let Some(c) = by_id.get(&id) else { continue };
            let text = crate::io::describe_with(c, &name_of);
            let site = map.site_of_constraint(id);
            let reads_own = crate::boundary::columns(sk, c).iter().any(|&p| own(p));
            let message = match reads_own {
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
