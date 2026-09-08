//! Geometry intent, independent of visibility and drawing style.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GeometryRoles {
    pub construction: bool,
    pub centerline: bool,
}

impl GeometryRoles {
    pub fn union(self, other: Self) -> Self {
        Self {
            construction: self.construction || other.construction,
            centerline: self.centerline || other.centerline,
        }
    }

    /// Semantic selectors exposed to drawing styles; these are not arbitrary model classes.
    pub fn selectors(self) -> impl Iterator<Item = &'static str> {
        [
            (self.construction, "construction"),
            (self.centerline, "centerline"),
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
        for role in self.roles.selectors() {
            out.push_str(role);
            out.push(' ');
        }
    }
}
