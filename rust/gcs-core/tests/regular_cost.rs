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

/// **The balance searches each flank a few times** (#904): the configured pair finds its pressure
/// shift by tying two flanks' least area factors, each of the four held from its fold.  The block
/// that finds the shift is torn at its unknowns first, as a block finding one from a contact is,
/// and the drawing solved a block at a time first: whole, the two rows read flanks their blanks do
/// not yet hold, and every start the block's root was searched from (its points scattered at the
/// drawing's size) searched both afresh — a hundred searches, twelve seconds.  Torn, each flank
/// is searched from nothing where its block first places it and where the shift's first step moves
/// its blade's apex beyond the memo's reach, and the two that bind nowhere once, by the verdict.
#[test]
fn the_balance_searches_each_flank_a_few_times() {
    let mut e = fixtures::gear::unsolved(&fixtures::gear::source());
    let before = tally();
    let r = gcs_core::solve::solve(&mut e.sketch, fixtures::accurate());
    let cost = since(before);
    eprintln!("{cost:?}");
    assert!(r.success, "{r:?}");
    let q = e.sketch.free_vars["pair.pressure_shift"];
    let shift = e.sketch.params[q as usize].value;
    assert!((shift - 8.854877).abs() < 1e-4, "the shift {shift}°");
    assert!(cost.searches <= 6, "{cost:?}");
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
