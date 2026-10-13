//! **Hidden keys**: the names the flattener makes that the source cannot write, each spelled
//! here once.  A `#` is no character of a written name, so a key with one never meets a name the
//! source declares; what follows it says what made it.  Prefixes end in `.`, so what an
//! application or a copy makes is named under them (`{prefix}#{stmt}.0.p`).
//!
//! The bound variables an application binds to what its use wrote — the line a tangency runs
//! along, a reach's point and set, a word's operands — are names of the same kind, read under
//! the application's prefix (`apply::local`).

use crate::syntax::Member;

/// The line a tangency's contact runs along, bound to the use's line (`l tangent S`), or to the
/// line an inherited derivative runs along.
pub(super) const LINE: &str = "#line";

/// The point of an inherited derivative, bound to the use's (`Scope::twin`, `Relation::along`).
pub(super) const ALONG_POINT: &str = "#along.p";

/// A reach's point, `p` of `p distance(d, side: outside) S`.
pub(super) const POINT: &str = "#p";

/// A reach's set, `S` of `p distance(d, side: outside) S`.
pub(super) const SET: &str = "#set";

/// The number a set's side is grown by, given to the application (`Application::give`).
pub(super) const GROW: &str = "#grow";

/// The signed distance a reach's point stands from its foot, given to the application.
pub(super) const REACH: &str = "#reach";

/// The plane a face's edges are drawn in, after the face's name: `f0#plane`.
pub(super) const PLANE: &str = "#plane";

/// A word's `i`th operand as its use wrote it, read by the relations the application made.
pub(super) fn word_operand(i: usize) -> String {
    format!("#word.{i}")
}

/// One of a plane's own axes (`u`, `v`) a plane's tangency is differentiated along.
pub(super) fn plane_axis(axis: &str) -> String {
    format!("#{axis}")
}

/// The prefix of the `n`th application one use makes, `{where the use stands}#{statement}.{n}.`.
pub(super) fn application(prefix: &str, stmt: u32, n: usize) -> String {
    format!("{prefix}#{stmt}.{n}.")
}

/// The prefix a block's copies are made under, `{where the block stands}#{statement}.` — the
/// copy's index and a `.` after it.
pub(super) fn block(prefix: &str, stmt: u32) -> String {
    format!("{prefix}#{stmt}.")
}

/// The point a measurement's `k`th set is measured from, local to where the statement stands:
/// `#{statement}.m{k}`.
pub(super) fn measured(stmt: u32, k: usize) -> String {
    format!("#{stmt}.m{k}")
}

/// The prefix the `k`th measured set's incidence is applied under, `{prefix}#{statement}.m{k}.`.
pub(super) fn measured_prefix(prefix: &str, stmt: u32, k: usize) -> String {
    format!("{prefix}{}.", measured(stmt, k))
}

/// The derivative along the `k`th chart two sets touching share, `{prefix}#{statement}.t{k}.`.
pub(super) fn chart(prefix: &str, stmt: u32, k: u8) -> String {
    format!("{prefix}#{stmt}.t{k}.")
}

/// The derivative along a plane's `n`th axis, `{prefix}#{statement}.a{n}.`.
pub(super) fn plane_along(prefix: &str, stmt: u32, n: usize) -> String {
    format!("{prefix}#{stmt}.a{n}.")
}

/// The derivative along the `k`th chart at a reach's foot, under the application's own prefix:
/// `{application}t{k}.`.
pub(super) fn foot_chart(application: &str, k: u8) -> String {
    format!("{application}t{k}.")
}

/// A set's surface, made beside it: `{set}#surface`.
pub(super) fn surface(set: &str) -> String {
    format!("{set}{}", Member::Surface.key())
}

/// A region solid's probe: `{solid}#probe`.
pub(super) fn probe(solid: &str) -> String {
    format!("{solid}#probe")
}

/// The plane a face is drawn in: `{face}#plane`.
pub(super) fn face_plane(face: &str) -> String {
    format!("{face}{PLANE}")
}

/// A face written in place in a solid's statement, by the solid's name: `{solid}#face`.
pub(super) fn written_face(solid: &str) -> String {
    format!("{solid}#face")
}

/// The axis square to a side's edge and along its face's plane: `{face}#{edge}#m`.
pub(super) fn side_axis(face: &str, edge: &str) -> String {
    format!("{face}#{edge}#m")
}

/// What an interface-typed formal was given, whole (§8.1 [0.84]): `{formal}#given`, judged once
/// every name is known.
pub(super) const GIVEN: &str = "#given";

/// The argument an interface-typed formal was given, under the formal's key: `{formal}#given`.
pub(super) fn given(formal: &str) -> String {
    format!("{formal}{GIVEN}")
}
