//! Each solid a closed instance makes, stamped with its binding (#566): the key its instance's
//! binding gives it (`flatten::Bound`) with its own path in the body, the frame the instance
//! stands in, and the turn of the `ring` copy it is in.  `gcs-solve` reads the stamp and nothing
//! else of the language (`model::Binding`): solids of one key are one evaluation placed.

use super::resolve::Resolver;
use crate::flatten::{InstanceInfo, RingInfo};
use crate::model::{Binding, EntKind, EntRef, Sketch};

/// Stamp every solid made under a closed instance — the innermost one it is made in — once every
/// solid is built and the body rule has joined its tools.
pub(super) fn stamp(sk: &mut Sketch, res: &Resolver, instances: &[InstanceInfo], rings: &[RingInfo]) {
    // each closed instance with its key, its frame's index and its turn, innermost first: the
    // longest prefix a name starts with is the instance that made it
    let mut closed: Vec<(&InstanceInfo, &str, Option<u32>, Option<(EntRef, u32, u32)>)> =
        instances.iter().filter_map(|inst| {
            let bound = inst.binding.as_ref()?;
            let frame = match &bound.frame {
                Some(f) => Some(res.of.get(f).filter(|e| e.kind == EntKind::Frame)?.i() as u32),
                None => None,
            };
            // a copy of a ring's body past the first stands in its frame turned; one in two
            // rings' copies (a ring in an instance in a ring) is its own
            let turns: Vec<_> = rings.iter()
                .filter_map(|r| Some((r, super::rings::copy_of(r, &inst.prefix)?.0)))
                .filter(|&(_, k)| k > 0)
                .collect();
            let turn = match turns.as_slice() {
                [] => None,
                [(r, k)] => Some((res.entity(sk, &r.about)?, *k, r.n)),
                _ => return None,
            };
            Some((inst, bound.key.as_str(), frame, turn))
        }).collect();
    closed.sort_by_key(|(i, ..)| std::cmp::Reverse(i.prefix.len()));
    // which of the solids so named under an instance each is: a body and the stock the body rule
    // moved out from under its name share one
    let mut seen: std::collections::BTreeMap<(usize, String), u32> = Default::default();
    for si in 0..sk.solids.len() {
        let name = &sk.solids[si].name;
        let Some((n, &(inst, key, frame, turn))) =
            closed.iter().enumerate().find(|(_, c)| name.starts_with(&c.0.prefix)) else { continue };
        let path = name[inst.prefix.len()..].to_string();
        let nth = seen.entry((n, path.clone())).or_default();
        let key = format!("{key}\u{1e}{path}\u{1e}{nth}");
        *nth += 1;
        sk.solids[si].binding = Some(Binding { key, at: inst.prefix.clone(), frame, turn });
    }
}
