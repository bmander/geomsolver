//! Each solid a closed instance makes, stamped with its binding (#566): the key its instance's
//! binding gives it (`flatten::Bound`) with its own path in the body, the frame the instance
//! stands in, and the turn of the `ring` copy it is in.  `gcs-solve` reads the stamp and nothing
//! else of the language (`model::Binding`): solids of one key are one evaluation placed.

use super::resolve::Resolver;
use crate::flatten::{Bound, InstanceInfo, RingInfo};
use crate::model::{Binding, EntKind, EntRef, Sketch};

/// Stamp every solid made under a closed instance — the innermost one it is made in — once every
/// solid is built and the body rule has joined its tools.
pub(super) fn stamp(sk: &mut Sketch, res: &Resolver, instances: &[InstanceInfo], rings: &[RingInfo]) {
    // each closed instance with its binding, its frame's index and its turn, innermost first: the
    // longest prefix a name starts with is the instance that made it
    type Closed<'a> = (&'a InstanceInfo, &'a Bound, Option<u32>, Option<(EntRef, u32, u32)>);
    let mut closed: Vec<Closed> =
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
            Some((inst, bound, frame, turn))
        }).collect();
    closed.sort_by_key(|(i, ..)| std::cmp::Reverse(i.prefix.len()));
    // which of the solids so named under an instance each is: a body and the stock the body rule
    // moved out from under its name share one
    let mut seen: std::collections::BTreeMap<(usize, String), u32> = Default::default();
    for si in 0..sk.solids.len() {
        let name = &sk.solids[si].name;
        // a mirrored instance's every solid is its own, reflected, however deep it was made
        // (#578): the outermost mirrored instance it is under, else the innermost closed one
        let under = |p: &str| name.starts_with(p);
        let found = closed.iter().enumerate().rev()
            .find(|(_, c)| c.0.mirror.is_some() && under(&c.0.prefix))
            .or_else(|| closed.iter().enumerate().find(|(_, c)| under(&c.0.prefix)));
        let Some((n, &(inst, bound, frame, turn))) = found else { continue };
        let path = name[inst.prefix.len()..].to_string();
        let nth = seen.entry((n, path.clone())).or_default();
        let (key, part) = (format!("{}\u{1e}{path}\u{1e}{nth}", bound.key),
                           format!("{}\u{1e}{path}\u{1e}{nth}", bound.part));
        *nth += 1;
        let mirror = inst.mirror.as_ref().map(|m| m.axis);
        sk.solids[si].binding =
            Some(Binding { key, part, at: inst.prefix.clone(), frame, turn, mirror });
    }
}

