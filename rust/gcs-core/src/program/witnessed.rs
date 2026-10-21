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
    // each held number, by the witness that owns it and that witness's block
    let mut owner: BTreeMap<u32, (EntRef, u32)> = BTreeMap::new();
    for (&e, r) in &sk.roles {
        let Some(w) = r.witness else { continue };
        for p in sk.own_params(e) {
            owner.insert(p, (e, w));
        }
    }
    if owner.is_empty() {
        return;
    }
    for &p in owner.keys() {
        sk.params[p as usize].fixed = true;
    }
    for c in &sk.constraints {
        // an entity's own intrinsic rows (an arc's ends at its radius) are the witness's own,
        // held with it, and solved with it where its block is
        if c.intrinsic {
            continue;
        }
        let read: BTreeSet<(EntRef, u32)> =
            c.params(sk).iter().filter_map(|p| owner.get(p).copied()).collect();
        for (e, _) in read.into_iter().filter(|&(_, w)| c.witness != Some(w)) {
            let name = map.name_of(e).cloned().unwrap_or_else(|| "a witness".into());
            let why = match c.witness {
                None => format!(
                    "`{name}` is a claim's witness, made only to state it: nothing outside its \
                     `claim {{ … }}` block may state anything of it"
                ),
                Some(_) => format!(
                    "`{name}` is another claim's witness: each `claim {{ … }}` block declares \
                     its own"
                ),
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
