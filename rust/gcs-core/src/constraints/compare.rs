//! Whether two constraints say the same thing.

use super::*;

fn same_args(a: &Constraint, b: &Constraint, swap: bool, want: impl Fn(SpecKind) -> bool)
    -> bool
{
    let spec = a.kind.spec();
    let mut order: Vec<usize> = (0..spec.len()).collect();
    if swap {
        let ents: Vec<usize> =
            spec.iter().enumerate().filter(|(_, (_, k))| k.is_entity()).map(|(i, _)| i).collect();
        if ents.len() < 2 {
            return false;
        }
        order.swap(ents[0], ents[1]);
    }
    // A hidden unknown is never part of what a constraint *says*: two contacts of the same point
    // on the same curve are the same statement however far apart their two seeds started, and a
    // duplicate that slipped through would add rank-free rows the matching cannot see.
    (0..spec.len()).filter(|&i| want(spec[i].1)).all(|i| a.args[i] == b.args[order[i]])
}

/// True when two constraints say exactly the same thing: same type, the same entities in the same
/// roles, the same values.  `commutative` types also match with their first two entities swapped.
///
/// An exact duplicate is worth keeping out of a sketch: it adds equations without adding rank, and
/// a structural matching cannot see that — two identical rows still match two different variables
/// — so it stays invisible until some unrelated edit tips the block into a (spurious)
/// over-constrained report.
pub fn same_constraint(a: &Constraint, b: &Constraint) -> bool {
    matches(a, b, |k| !k.is_param())
}

/// True when two constraints relate the same things in the same way, *whatever numbers they
/// state*: same type, same entities in the same roles, same flags, dimensions ignored.
///
/// `same_constraint` is this plus the values, which is what a duplicate is.  This is what an
/// *edit* would land on: the constraint a second `Distance` on the same pair would be rewriting
/// rather than adding to.  Whether that is what a caller wants is the caller's business — the
/// app states the second one and lets the diagnosis judge the pair.
pub fn same_relation(a: &Constraint, b: &Constraint) -> bool {
    matches(a, b, |k| !k.is_param() && !k.is_dimension())
}

fn matches(a: &Constraint, b: &Constraint, want: impl Fn(SpecKind) -> bool + Copy) -> bool {
    // a row and its derivative are two equations about the same entities, and a bound is no
    // equation at all (§9.6)
    if a.kind != b.kind || a.along != b.along || a.bound != b.bound {
        return false;
    }
    same_args(a, b, false, want) || (a.kind.commutative() && same_args(a, b, true, want))
}

/// Whether an entity can fill a spec slot of this kind.
pub fn kind_matches(spec: SpecKind, ent: EntKind) -> bool {
    match spec {
        SpecKind::Point => ent == EntKind::Point,
        SpecKind::Line => ent == EntKind::Line,
        SpecKind::Circle => ent == EntKind::Circle,
        SpecKind::Arc => ent == EntKind::Arc,
        SpecKind::CircleOrArc => ent == EntKind::Circle || ent == EntKind::Arc,
        SpecKind::Spline => ent == EntKind::Spline,
        SpecKind::Curve => ent == EntKind::Curve,
        SpecKind::Envelope => ent == EntKind::Envelope,
        SpecKind::Meshing => ent == EntKind::Meshing,
        SpecKind::Motion => ent == EntKind::Motion,
        SpecKind::Traced => ent == EntKind::Traced,
        SpecKind::Solid => ent == EntKind::Solid,
        SpecKind::Plane => ent == EntKind::Plane,
        SpecKind::Axis => ent == EntKind::Axis,
        SpecKind::Direction => ent == EntKind::Line || ent == EntKind::Axis,
        // what a contact holds a point on, whose slot a rate runs over (#365)
        SpecKind::Carrier => matches!(ent, EntKind::Spline | EntKind::Curve | EntKind::Traced
                                           | EntKind::Meshing | EntKind::Motion),
        SpecKind::Along => matches!(ent, EntKind::Line | EntKind::Axis | EntKind::Plane),
        _ => false,
    }
}
