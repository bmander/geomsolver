//! The numbers in force where a statement stands (`Scope::vals`), as frames read outward (#405).
//!
//! A body, a block's copy, a `when` that holds and a word's application each add a frame of only
//! the names they make — the binder, the body's `param`s, its instances' numbers — over the table
//! in force around them, which they share and never copy.  The document's root holds every
//! instance's numbers (`P0.w`), so a copy of it per block copy or per `when` is quadratic in the
//! assembly; a frame costs what it adds.  A name is read innermost frame first, so a frame's own
//! value hides one further out, exactly as inserting it into a copy of the whole would have.
//! The depth is how many blocks nest inside one instance — an instance's table starts afresh,
//! with what its call gave it — so bounded by `MAX_DEPTH`.

use crate::expr::{self, Aff};
use std::collections::{BTreeMap, BTreeSet};
use std::rc::Rc;

/// What a text's names are worked out against while elaborating: the walk's table (`Vals`), a
/// seed's view of it (`expand::Seeded`), or a plain map where a caller outside the walk has one.
pub(crate) trait Table: expr::Env {
    /// Whether `name` has a value here.
    fn has(&self, name: &str) -> bool;
    /// Whether some value here is affine in the drawing's unknown `unknown` (a formal left
    /// unbound, a `param` with no value, or one written over either).
    fn binds(&self, unknown: &str) -> bool;
    /// Every name under `prefix` (`o.` reads `o.x` and `o.y`), each with the value it has here.
    fn under(&self, prefix: &str) -> BTreeMap<String, Aff>;
}

impl Table for BTreeMap<String, Aff> {
    fn has(&self, name: &str) -> bool {
        self.contains_key(name)
    }

    fn binds(&self, unknown: &str) -> bool {
        self.values().any(|v| v.free.as_deref() == Some(unknown))
    }

    fn under(&self, prefix: &str) -> BTreeMap<String, Aff> {
        prefixed(self, prefix).map(|(k, v)| (k.clone(), v.clone())).collect()
    }
}

/// The entries of `m` whose names start with `prefix`, in order.
pub(crate) fn prefixed<'m, V>(m: &'m BTreeMap<String, V>, prefix: &'m str)
    -> impl Iterator<Item = (&'m String, &'m V)> {
    use std::ops::Bound;
    m.range::<str, _>((Bound::Included(prefix), Bound::Unbounded))
        .take_while(move |(k, _)| k.starts_with(prefix))
}

/// The numbers in force: a frame of names of its own over the table it was made in.
#[derive(Clone, Default, Debug)]
pub(crate) struct Vals {
    own: BTreeMap<String, Aff>,
    /// The names in `own` whose value is affine in each unknown, by the unknown: what `binds`
    /// asks without reading every value in force.
    unknowns: BTreeMap<String, BTreeSet<String>>,
    outer: Option<Rc<Vals>>,
}

impl Vals {
    /// A frame of its own over `outer`, holding nothing yet.
    pub(crate) fn over(outer: Rc<Vals>) -> Vals {
        Vals { outer: Some(outer), ..Vals::default() }
    }

    /// A table of `own` and nothing further out: an instance's, given its call's numbers.
    pub(crate) fn of(own: BTreeMap<String, Aff>) -> Vals {
        Vals::given(Vals::default(), own)
    }

    /// `own` in a frame over `outer`: a body's numbers over the constants of its file (#461).
    pub(crate) fn over_with(outer: Rc<Vals>, own: BTreeMap<String, Aff>) -> Vals {
        Vals::given(Vals::over(outer), own)
    }

    /// `t` with `own` put in its frame.
    fn given(mut t: Vals, own: BTreeMap<String, Aff>) -> Vals {
        for (k, v) in own {
            t.insert(k, v);
        }
        t
    }

    /// The frames, innermost first.
    fn frames(&self) -> impl Iterator<Item = &Vals> {
        std::iter::successors(Some(self), |t| t.outer.as_deref())
    }

    /// The value `name` has here.
    pub(crate) fn get(&self, name: &str) -> Option<&Aff> {
        self.frames().find_map(|t| t.own.get(name))
    }

    pub(crate) fn contains_key(&self, name: &str) -> bool {
        self.get(name).is_some()
    }

    /// `name` given `value` in this frame, over whatever it had further out.
    pub(crate) fn insert(&mut self, name: String, value: Aff) {
        self.forget(&name);
        if let Some(u) = &value.free {
            self.unknowns.entry(u.clone()).or_default().insert(name.clone());
        }
        self.own.insert(name, value);
    }

    /// `name` taken out of this frame's own.
    fn forget(&mut self, name: &str) {
        let Some(u) = self.own.remove(name).and_then(|v| v.free) else { return };
        if let Some(names) = self.unknowns.get_mut(&u) {
            names.remove(name);
            if names.is_empty() {
                self.unknowns.remove(&u);
            }
        }
    }

    /// `name` given `value` where nothing here has one yet.
    pub(crate) fn or_insert(&mut self, name: String, value: Aff) {
        if !self.contains_key(&name) {
            self.insert(name, value);
        }
    }

    /// This frame's own names, and nothing further out.
    pub(crate) fn own(&self) -> &BTreeMap<String, Aff> {
        &self.own
    }

    /// `own`, taken.
    pub(crate) fn into_own(self) -> BTreeMap<String, Aff> {
        self.own
    }

    /// Whether any value in force is affine in an unknown of the drawing: a name some frame
    /// indexed as one, whose value here — a frame inside may hide it — still is.
    pub(crate) fn any_unknown(&self) -> bool {
        self.frames().any(|t| t.unknowns.values().flatten()
            .any(|name| self.get(name).is_some_and(|v| v.free.is_some())))
    }
}

impl expr::Env for Vals {
    fn value(&self, name: &str) -> Option<Aff> {
        self.get(name).cloned()
    }
}

impl Table for Vals {
    fn has(&self, name: &str) -> bool {
        self.contains_key(name)
    }

    fn binds(&self, unknown: &str) -> bool {
        self.frames().any(|t| t.unknowns.get(unknown).into_iter().flatten()
            .any(|name| self.get(name).is_some_and(|v| v.free.as_deref() == Some(unknown))))
    }

    fn under(&self, prefix: &str) -> BTreeMap<String, Aff> {
        let mut out = BTreeMap::new();
        for t in self.frames() {
            for (k, v) in prefixed(&t.own, prefix) {
                out.entry(k.clone()).or_insert_with(|| v.clone());
            }
        }
        out
    }
}
