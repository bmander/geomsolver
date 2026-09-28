//! The block-triangular structure of a document's equations: a tool, not a gate — a report over
//! `System::block_order` (the rows matched to the free columns, the square part sorted into
//! strongly connected blocks, each solved on its own once the blocks it reads are; the gate is
//! `tests/block_order.rs`).  What it prints — how many blocks, how large, how deep the chain, and
//! what each large block is made of — says whether a document is one simultaneous system or a
//! sequence of small ones.
use gcs_core::{io, system::System};
use std::collections::BTreeMap;

/// The part of the layout a constraint belongs to: the component its first named entity is in.
fn part(name: &str) -> String {
    let segs: Vec<&str> = name.split('.').collect();
    let at = if segs.first() == Some(&"pair") && segs.get(1) == Some(&"reference") { 2 }
        else if segs.first() == Some(&"pair") { 1 } else { 0 };
    segs.get(at).map(|s| s.split('[').next().unwrap().to_string()).unwrap_or_else(|| "?".into())
}

fn report(label: &str, e: &gcs_core::program::Elaborated) {
    let sk = &e.sketch;
    let mut sys = System::new(sk);
    let order = sys.block_order();
    println!("\n== {label}: {} rows, {} free columns, over {} rows / {} cols, under {} rows / {} cols",
        sys.hard_rows().len(), sys.n_free, order.over_rows.len(), order.over_cols.len(),
        order.under_rows.len(), order.under_cols.len());
    let mut sizes: BTreeMap<usize, usize> = BTreeMap::new();
    for b in &order.blocks { *sizes.entry(b.rows.len()).or_default() += 1; }
    println!("  {} blocks, longest chain {} blocks; block sizes (rows: count): {:?}",
        order.blocks.len(), order.depth(), sizes);
    let by_id: BTreeMap<u32, &gcs_core::constraints::Constraint> = sk.constraints.iter().map(|c| (c.id, c)).collect();
    let name = |r: gcs_core::model::EntRef| e.map.name_of(r).cloned();
    let mut big: Vec<_> = order.blocks.iter().filter(|b| b.rows.len() >= 6).collect();
    big.sort_by_key(|b| std::cmp::Reverse(b.rows.len()));
    for b in big.iter().take(8) {
        let mut parts: BTreeMap<String, usize> = BTreeMap::new();
        let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
        let mut cids: Vec<u32> = b.instances.iter().map(|&(k, i)| sys.blocks[k].cids[i]).collect();
        cids.sort();
        for cid in &cids {
            let con = by_id[cid];
            let text = io::describe_with(con, &name);
            let first = text.split_whitespace().find(|w| w.contains('.')).unwrap_or("?");
            *parts.entry(part(first)).or_default() += 1;
            *kinds.entry(format!("{:?}", con.kind)).or_default() += 1;
        }
        println!("  block of {} rows ({} constraints): parts {:?}", b.rows.len(), cids.len(), parts);
        println!("      kinds {:?}", kinds);
    }
}

#[test]
#[ignore = "a tool: cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core block_structure -- --ignored --nocapture"]
fn the_hypoid_layouts_block_structure() {
    report("gears.sv, configured", &fixtures::gear::read_as_configured());
    let pair = std::fs::read_to_string(fixtures::gear::project().join("pair.sv")).unwrap();
    report("pair.sv, configured", &fixtures::read_beside(&pair, &fixtures::gear::project(), &mut |_, t| t));
    report("gears.sv, bevel", &fixtures::gear::read_configured_with(&mut |n, t| fixtures::gear::bevel(n, t)));
}
