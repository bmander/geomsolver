//! **One thing stated twice is stated once** (#415).
//!
//! A family declared `symmetric(a, b)` is the same set whichever order its listed formals are
//! given in (`Component::symmetric`).  So an instance given the arguments of one made before, in
//! another of those orders, *is* that instance: the walk makes it a name for the first and expands
//! nothing (`Walk::same_instance`).  A region solid of a set that is one made before is the solid
//! made of it (`Walk::same_solid`), and a body statement over it that says what one over that solid
//! already said joins nothing more (`Sketch::join_body`).  `round(r) X` reaches every edge of `X`
//! in both orders and every corner in six: each is one set, one solid, one cut and one fill.
//!
//! Nothing here is a guess made after the fact: two instances are one only where the family says
//! the order does not matter, and only by what their arguments stand for, worked out as a `when`
//! works it out, and only where the two are stamped alike (roles, classes): an instance stamped
//! otherwise is its own.  Where an argument cannot be told yet (a name not found, a number left
//! unknown) the instance is its own.  And only a block's copies share a solid: a solid the
//! document names is its own statement, never taken for a copy's or a copy's for it.

use super::*;

/// What the walk found to be one thing named twice — see the module's words.
#[derive(Default)]
pub(super) struct Same {
    /// Each symmetric family's instance made, by its component, its arguments in the order the
    /// family's symmetry sorts them to (`canonical`) and its stamps, to the instance's absolute
    /// name.
    instances: BTreeMap<(usize, Vec<String>, String), String>,
    /// An instance that is one made before, by its absolute name, to that one's.
    pub(super) sets: BTreeMap<String, String>,
    /// The first region solid a block's copy made of each set, by the set's absolute name, to
    /// the solid's.
    solids: BTreeMap<String, String>,
}

/// The arguments `ids` (one per formal) with the symmetry's items in the order that sorts them:
/// every order of the items, each moving all its formals into the places of another, comes to
/// the same.
fn canonical(ids: &[String], items: &[Vec<usize>]) -> Vec<String> {
    let mut each: Vec<Vec<&String>> =
        items.iter().map(|item| item.iter().map(|&f| &ids[f]).collect()).collect();
    each.sort();
    let mut v = ids.to_vec();
    for (item, from) in items.iter().zip(&each) {
        for (&f, id) in item.iter().zip(from) {
            v[f] = (*id).clone();
        }
    }
    v
}

impl Walk<'_> {
    /// Whether the instance just bound under `key` (`{prefix}{name}.`) is one made before (#415):
    /// a symmetric family's, given what an earlier one was given in another order the family
    /// allows.  If so its name is made a name for that one, which is not made again; if not, it
    /// is remembered for the instances after it.  What `stamps` names (the roles and classes its
    /// parts would carry) is part of what it is: an instance stamped otherwise is its own.
    pub(super) fn same_instance(
        &mut self,
        comp: &Component,
        key: &str,
        scope: &Scope,
        given: &BTreeMap<String, Aff>,
        sides: &BTreeMap<String, String>,
        stamps: &Stamps,
    ) -> bool {
        let drawn_in = stamps.plane.is_some() || !stamps.rides.is_empty();
        if comp.symmetric.is_empty() || comp.set.is_none() || self.sym.is_some() || drawn_in {
            return false;
        }
        let Some(index) = self.instances.last().filter(|i| i.prefix == key).map(|i| i.comp) else {
            return false;
        };
        let at = |n: &Name| comp.formals.iter().position(|f| f.name.text == n.text);
        let Some(items) = comp.symmetric.iter()
            .map(|item| item.iter().map(at).collect::<Option<Vec<usize>>>())
            .collect::<Option<Vec<_>>>()
        else {
            return false;
        };
        // what each argument stands for: an entity, a set or a group by its absolute name, as a
        // `when` compares them; a number by its value
        let alias = super::resolve::Lazy::new(self);
        let root = Scope::absolute(scope.access.clone());
        let mut ids = Vec::with_capacity(comp.formals.len());
        for f in &comp.formals {
            let id = match f.ty {
                crate::syntax::Ty::Ent(_) | crate::syntax::Ty::Path | crate::syntax::Ty::Set
                | crate::syntax::Ty::Group => {
                    let r = Ref::dotted(&format!("{key}{}", f.name.text), f.name.span);
                    self.stands_for(&r, &root, &alias)
                }
                crate::syntax::Ty::Side => sides.get(&f.name.text).cloned(),
                // a number left unknown is the instance's own (`t1.w`), so it differs
                _ => given.get(&f.name.text).map(|v| format!("{v:?}")),
            };
            let Some(id) = id else { return false };
            ids.push(id);
        }
        let name = key.trim_end_matches('.').to_string();
        let stamped = format!("{:?} {:?} {}", stamps.roles, stamps.class.0, stamps.anonymous);
        let canon = (index, canonical(&ids, &items), stamped);
        let Some(first) = self.same.instances.get(&canon).cloned() else {
            self.same.instances.insert(canon, name);
            return false;
        };
        // the name stands for the first: read through it, as a formal is read through its actual
        self.names.remove(&name);
        self.group_names.remove(&name);
        self.instances.pop();
        let span = comp.name.as_ref().map_or(Span::default(), |n| n.span);
        let root = Scope::absolute(scope.access.clone());
        self.aliases.push((name.clone(), Ref::dotted(&first, span), root));
        self.same.sets.insert(name, first);
        true
    }

    /// Whether region solid `abs` (`taken := solid(run)`), declared in a block's copy, is of a set
    /// that is one made before, of which a copy's solid is made already: then `abs` is a name for
    /// that solid, and the declaration makes nothing.  A copy's solid of a set made once is
    /// remembered as its set's.  `set` is the solid's set as `set_named` reads it.
    pub(super) fn same_solid(&mut self, d: &Decl, abs: &str, set: Option<&(Ref, String, bool)>,
                             sc: &Scope) -> bool {
        // two solids the document names of one set are two statements it made, which this says
        // nothing about, nor is one of them a copy's
        let Some((_, set, through)) = set.filter(|_| sc.block.copies) else { return false };
        match self.same.solids.get(set).cloned() {
            // only where the set is the same by symmetry: a copy reaches it once for each order
            Some(first) if *through => {
                self.names.remove(abs);
                let at = d.name.span();
                self.aliases.push((abs.to_string(), Ref::dotted(&first, at),
                                   Scope::absolute(sc.access.clone())));
                true
            }
            Some(_) => false,
            None => {
                self.same.solids.insert(set.clone(), abs.to_string());
                false
            }
        }
    }
}
