//! **One thing stated twice is stated once** (#415).
//!
//! A family declared `symmetric(a, b)` is the same set whichever order its listed formals are
//! given in (`Component::symmetric`).  So an instance given the arguments of one made before, in
//! another of those orders, *is* that instance: the walk makes it a name for the first and expands
//! nothing (`Walk::same_instance`).  A region solid of a set that is one made before is the solid
//! made of it (`Walk::same_solid`), and a body statement over it that says what one over that solid
//! already said says nothing more (`Walk::same_body`).  `round(r) X` reaches every edge of `X` in
//! both orders and every corner in six: each is one set, one solid, one cut and one fill.
//!
//! Nothing here is a guess made after the fact: two instances are one only where the family says
//! the order does not matter, and only by what their arguments stand for, worked out as a `when`
//! works it out.  Where an argument cannot be told yet (a name not found, a number left unknown)
//! the instance is its own.

use super::*;

/// What the walk found to be one thing named twice — see the module's words.
#[derive(Default)]
pub(super) struct Same {
    /// Each symmetric family's instance made, by its component and its arguments in the order
    /// the family's symmetry reads first (`canonical`), to the instance's absolute name.
    instances: BTreeMap<(usize, Vec<String>), String>,
    /// An instance that is one made before, by its absolute name, to that one's.
    pub(super) sets: BTreeMap<String, String>,
    /// The first region solid made of each set, by the set's absolute name, to the solid's.
    solids: BTreeMap<String, String>,
    /// Those region solids, and the solids that are one of them: each to the one it is.
    regions: BTreeMap<String, String>,
    /// Each body statement over a region solid already made: its word, the solid, and what its
    /// body stands for.
    bodies: BTreeSet<(&'static str, String, String)>,
}

/// The orders `n` items can stand in, each as where item `i` takes its arguments from.
fn orders(n: usize) -> Vec<Vec<usize>> {
    if n == 0 {
        return vec![Vec::new()];
    }
    let mut out = Vec::new();
    for rest in orders(n - 1) {
        for at in 0..=rest.len() {
            let mut o = rest.clone();
            o.insert(at, n - 1);
            out.push(o);
        }
    }
    out
}

/// The arguments `ids` (one per formal) in the order of the symmetry's that comes first: each
/// order of the items moves every formal of an item into the matching place of another.
fn canonical(ids: &[String], items: &[Vec<usize>]) -> Vec<String> {
    let mut best: Option<Vec<String>> = None;
    for order in orders(items.len()) {
        let mut v = ids.to_vec();
        for (to, &from) in order.iter().enumerate() {
            for (k, &f) in items[to].iter().enumerate() {
                v[f] = ids[items[from][k]].clone();
            }
        }
        if best.as_ref().is_none_or(|b| v < *b) {
            best = Some(v);
        }
    }
    best.unwrap_or_else(|| ids.to_vec())
}

/// The most items a family may list: the orders of more are more than the walk asks after.
const MAX_ITEMS: usize = 6;

impl Walk<'_> {
    /// Whether the instance just bound under `key` (`{prefix}{name}.`) is one made before (#415):
    /// a symmetric family's, given what an earlier one was given in another order the family
    /// allows.  If so its name is made a name for that one, which is not made again; if not, it
    /// is remembered for the instances after it.
    pub(super) fn same_instance(
        &mut self,
        comp: &Component,
        key: &str,
        scope: &Scope,
        given: &BTreeMap<String, Aff>,
        sides: &BTreeMap<String, String>,
        drawn_in: bool,
    ) -> bool {
        if comp.symmetric.is_empty() || comp.set.is_none() || self.sym.is_some() || drawn_in
            || comp.symmetric.len() > MAX_ITEMS
        {
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
        let canon = (index, canonical(&ids, &items));
        let Some(first) = self.same.instances.get(&canon).cloned() else {
            self.same.instances.insert(canon, name);
            return false;
        };
        // the name stands for the first: read through it, as a formal is read through its actual
        self.names.remove(&name);
        self.group_names.remove(&name);
        self.instances.pop();
        let span = comp.name.as_ref().map_or(Span::default(), |n| n.span);
        self.aliases.push((name.clone(), Ref::dotted(&first, span), Scope::absolute(scope.access.clone())));
        self.same.sets.insert(name, first);
        true
    }

    /// Whether region solid `abs` (`taken := solid(run)`), declared in a block's copy, is of a set
    /// that is one made before, of which a solid is made already: then `abs` is a name for that
    /// solid, and the declaration makes nothing.  A solid of a set made once is remembered as its
    /// set's.
    pub(super) fn same_solid(&mut self, d: &Decl, abs: &str, sc: &Scope) -> bool {
        use crate::syntax::Sweep;
        if d.kind != crate::model::EntKind::Solid || !matches!(d.sweep, Some(Sweep::Body)) {
            return false;
        }
        let Some((_, set, through)) = self.set_named(d, 0, sc) else { return false };
        match self.same.solids.get(&set).cloned() {
            // only where the set is the same by symmetry, and the solid is a copy's, which a block
            // reaches once for each order: two solids the document names of one set are two
            // statements it made, which this says nothing about
            Some(first) if through && sc.block.copies => {
                self.names.remove(abs);
                let at = d.name.span();
                self.aliases.push((abs.to_string(), Ref::dotted(&first, at),
                                   Scope::absolute(sc.access.clone())));
                self.same.regions.insert(abs.to_string(), first);
                true
            }
            Some(_) => false,
            None => {
                self.same.solids.insert(set, abs.to_string());
                self.same.regions.insert(abs.to_string(), abs.to_string());
                false
            }
        }
    }

    /// Whether body statement `s` (`taken cut A.body`) says over a region solid what was said
    /// over it already: its solid one of the same solid's names (`same_solid`), its word the
    /// same and its body standing for the same.
    pub(super) fn same_body(&mut self, s: &crate::syntax::SolidRel, sc: &Scope) -> bool {
        if self.same.regions.is_empty() || !s.what.path.is_empty() {
            return false;
        }
        // the name as the walk's names find it, innermost first: a solid only if a region one
        let what = &s.what.root.text;
        let Some(solid) = sc.prefixes.iter()
            .map(|p| format!("{p}{what}"))
            .find(|abs| self.names.contains(abs) || self.same.regions.contains_key(abs))
            .and_then(|abs| self.same.regions.get(&abs).cloned())
        else {
            return false;
        };
        let alias = super::resolve::Lazy::new(self);
        let Some(body) = self.stands_for(&s.body, sc, &alias) else { return false };
        !self.same.bodies.insert((s.word.as_str(), solid, body))
    }
}
