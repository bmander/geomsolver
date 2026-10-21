//! Geometry intent, independent of visibility and drawing style.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GeometryRoles {
    pub construction: bool,
    pub centerline: bool,
    /// Declared in a `claim { … }` block (§9.7, #460), by the block: geometry made only to state
    /// a claim, held while the drawing solves and solved after it, for its claim alone.  The
    /// parser stamps the block's statement id, the elaborator the block's index per copy
    /// (`Constraint::witness`'s).  Never written as a modifier, so `Annotations::write` leaves
    /// it out.
    pub witness: Option<u32>,
}

impl GeometryRoles {
    pub fn union(self, other: Self) -> Self {
        Self {
            construction: self.construction || other.construction,
            centerline: self.centerline || other.centerline,
            witness: self.witness.or(other.witness),
        }
    }

    /// Semantic selectors exposed to drawing styles; these are not arbitrary model classes.
    pub fn selectors(self) -> impl Iterator<Item = &'static str> {
        [
            (self.construction, "construction"),
            (self.centerline, "centerline"),
            (self.witness.is_some(), "witness"),
        ]
        .into_iter()
        .filter_map(|(yes, name)| yes.then_some(name))
    }
}

/// Access belongs to a declaration's name; roles belong to the geometry it creates.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Annotations {
    pub private: bool,
    pub roles: GeometryRoles,
}

impl Annotations {
    pub(crate) fn write(self, out: &mut String) {
        if self.private {
            out.push_str("private ");
        }
        // a witness is the block it stands in, not a word before it
        for role in self.roles.selectors().filter(|&r| r != "witness") {
            out.push_str(role);
            out.push(' ');
        }
    }
}
