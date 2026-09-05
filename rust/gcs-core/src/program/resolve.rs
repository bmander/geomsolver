//! Resolve names and child paths, including forward references.

use crate::model::{EntKind, EntRef, Field, Sketch};
use crate::syntax::{Ref, Seg, Span};
use std::collections::BTreeMap;

/// Preallocated entity names and declared child slots for forward references.
#[derive(Default)]
pub(super) struct Resolver {
    pub(super) of: BTreeMap<String, EntRef>,
    pub(super) declared_at: BTreeMap<String, Span>,
    /// Each declaration's child slots, as the names it wrote — what `follow_building` reads
    /// where the entity itself is not built yet (build order is per kind, so a child slot that
    /// reaches into an entity of a later kind — `line t(p3, k.start)` with `k` an arc — has
    /// only the declaration to ask).  `None` where a slot holds a seed or nothing, since the
    /// point it mints does not exist until the parent is built.
    pub(super) kids: BTreeMap<String, Vec<Option<Ref>>>,
}

impl Resolver {
    pub(super) fn lookup(&self, r: &Ref) -> Option<EntRef> {
        self.of.get(&r.root.text).copied()
    }

    /// The name declaration `name` (of kind `kind`) wrote in its child field `f`, mirroring
    /// `follow`'s reading of the built entity — the same fields, the same refusals.
    fn kid(&self, name: &str, kind: EntKind, f: &str) -> Result<Option<Ref>, String> {
        let slots = self.kids.get(name).ok_or_else(|| format!("no such entity: `{name}`"))?;
        let mut at = 0usize;
        for (n, k) in kind.fields() {
            match k {
                Field::Scalar => {}
                Field::Child => {
                    if *n == f {
                        return Ok(slots.get(at).cloned().flatten());
                    }
                    at += 1;
                }
                Field::List => {
                    if *n == f {
                        return Err(format!("`{f}` is a list, so it needs an index"));
                    }
                    at += 1;
                }
            }
        }
        let named: Vec<&str> =
            kind.fields().iter().filter(|(_, k)| *k == Field::Child).map(|(n, _)| *n).collect();
        Err(if named.is_empty() {
            format!("a {} has no parts", kind.as_str())
        } else {
            format!("a {} has {}, not `{f}`", kind.as_str(), named.join(", "))
        })
    }
}

/// Follow child paths while building. Consult declared slots when a referenced
/// entity has not been constructed yet.
pub(super) fn follow_building(
    sk: &Sketch,
    res: &Resolver,
    e: EntRef,
    r: &Ref,
) -> Result<EntRef, String> {
    let mut e = e;
    let mut name = r.root.text.clone();
    let mut path: Vec<Seg> = r.path.clone();
    // two declarations naming their parts through each other would walk forever; the cap is
    // far past any real document, so hitting it is the cycle
    for _ in 0..64 {
        if path.is_empty() || e.i() < sk.count(e.kind) {
            return follow(sk, e, &path);
        }
        let Seg::Field(f) = &path[0] else {
            return Err("an index names a copy, not a part".to_string());
        };
        let Some(kid) = res.kid(&name, e.kind, &f.text)? else {
            return Err(format!(
                "`{name}` does not name its {}, and `{name}` is not built yet: name the point \
                 itself",
                f.text
            ));
        };
        let Some(ne) = res.lookup(&kid) else {
            return Err(format!("no such entity: `{}`", kid.root.text));
        };
        e = ne;
        name = kid.root.text.clone();
        path = kid.path.iter().chain(path[1..].iter()).cloned().collect();
    }
    Err(format!("`{}` names its parts in a circle", r.root.text))
}

/// Follow a reference's field path to the sub-entity it names — `root.center` is the circle's
/// centre point, and `a0.start` an arc's.
///
/// A `Scalar` field is not an entity and is not followed here: `c0.r` is a *number*, and the one
/// statement that names one is `fix`, which reads the path itself.
pub(super) fn follow(sk: &Sketch, mut e: EntRef, path: &[Seg]) -> Result<EntRef, String> {
    for seg in path {
        let Seg::Field(f) = seg else {
            return Err("an index names a copy, not a part".to_string());
        };
        let fields = e.kind.fields();
        let kids = sk.children(e);
        let mut at = 0usize;
        let mut found = None;
        for (name, kind) in fields {
            match kind {
                Field::Scalar => {}
                Field::Child => {
                    if *name == f.text {
                        found = kids.get(at).copied();
                    }
                    at += 1;
                }
                Field::List => {
                    if *name == f.text {
                        return Err(format!("`{}` is a list, so it needs an index", f.text));
                    }
                }
            }
        }
        match found {
            Some(k) => e = k,
            None => {
                let named: Vec<&str> =
                    fields.iter().filter(|(_, k)| *k == Field::Child).map(|(n, _)| *n).collect();
                return Err(if named.is_empty() {
                    format!("a {} has no parts", e.kind.as_str())
                } else {
                    format!("a {} has {}, not `{}`", e.kind.as_str(), named.join(", "), f.text)
                });
            }
        }
    }
    Ok(e)
}
