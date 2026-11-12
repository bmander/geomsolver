//! **What a flank's regularity costs a solve** (#904): `envelope::regular`'s worst point is a
//! search — a grid over the tool's face and the roll, a shrinking grid about the best few, a
//! Newton polish — and a solve asks it at every iterate a row reading it is evaluated at.  What
//! it costs is counted (`regular::tally`) and held here: the face mill's balance in
//! `spiral_bevel/hypoid.sv` searches each flank from nothing a few times, not at every start a
//! block's root is searched from; and a search puts its places on the envelope without reading
//! each one's gradient, which only its polish asks.
use gcs_core::envelope::regular::{tally, Tally};

/// What has been asked since `t0`.
fn since(t0: Tally) -> Tally {
    let t = tally();
    Tally { searches: t.searches - t0.searches, warm: t.warm - t0.warm,
            remembered: t.remembered - t0.remembered, gradients: t.gradients - t0.gradients }
}

/// **A search puts a place on the envelope by its equation alone** (#904): the roll at each place
/// of the grid and the shrinking grid is found by Newton on the envelope's equation, read with its
/// derivative in the roll by the place's own frame; only the polish reads a place's gradient.  The
/// Formate pair's pinion flank, read at its configured shift: each search from nothing reads a few
/// hundred gradients, where every place it tried read its own at every Newton step (twenty-five
/// thousand a search).
#[test]
fn a_search_reads_gradients_only_where_it_polishes() {
    let text = crate::regular::pair(12.5, crate::regular::REGULAR, false);
    let before = tally();
    let e = fixtures::gear::read(&text);
    let cost = since(before);
    eprintln!("{cost:?}");
    assert!(cost.searches >= 1, "{cost:?}");
    assert!(cost.gradients <= 1000 * cost.searches, "{cost:?}");
    assert!(crate::regular::reading(&e) > 0.0);
}
