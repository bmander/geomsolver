//! **A claim's witnesses** (§9.7, #460): the geometry a `claim { … }` block declares, made only
//! to state the claim.  The drawing holds every number a witness owns, so a witness is no
//! freedom of it and its block's relations (claims, no rows) are no equation of it: the drawing
//! solves, counts and diagnoses exactly as it would with the block deleted.  The diagnosis then
//! solves each block's witnesses by the block's relations, the drawing held where it landed
//! (`diagnose::witnessed`).

use super::*;
use crate::model::{EntRef, Sketch};

/// Hold every witness's numbers, and refuse a statement outside a witness's block that reads
/// it: a model row would make the witness part of the drawing again, and another block's would
/// make one claim's witness depend on how another's was solved (E040).
pub(super) fn hold(sk: &mut Sketch, map: &SourceMap, diags: &mut Vec<Diag>) {
    let witnesses: Vec<EntRef> =
        sk.roles.iter().filter(|(_, r)| r.witness).map(|(&e, _)| e).collect();
    if witnesses.is_empty() {
        return;
    }
    // each held number, by the witness that owns it
    let mut owner: BTreeMap<u32, EntRef> = BTreeMap::new();
    for &e in &witnesses {
        for p in sk.own_params(e) {
            owner.insert(p, e);
        }
    }
    for &p in owner.keys() {
        sk.params[p as usize].fixed = true;
    }
    // the block whose relations first read each witness
    let mut block: BTreeMap<EntRef, u32> = BTreeMap::new();
    for c in &sk.constraints {
        // an entity's own intrinsic rows (an arc's ends at its radius) are the witness's own,
        // held with it, and solved with it where its block is
        if c.intrinsic {
            continue;
        }
        let read: BTreeSet<EntRef> =
            c.params(sk).iter().filter_map(|p| owner.get(p).copied()).collect();
        for e in read {
            let name = map.name_of(e).cloned().unwrap_or_else(|| "a witness".into());
            let why = match c.witness {
                None => format!(
                    "`{name}` is a claim's witness, made only to state it: nothing outside its \
                     `claim {{ … }}` block may state anything of it"
                ),
                Some(w) if *block.entry(e).or_insert(w) != w => format!(
                    "`{name}` is another claim's witness: each `claim {{ … }}` block declares \
                     its own"
                ),
                Some(_) => continue,
            };
            let site = map.site_of_constraint(c.id);
            diags.push(Diag::new(
                Code::E040,
                site.map(|s| s.span).unwrap_or_default(),
                site.map(|s| s.stmt),
                why,
            ));
        }
    }
}
