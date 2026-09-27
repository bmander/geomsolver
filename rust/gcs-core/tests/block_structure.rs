//! The block-triangular structure of a document's equations: a tool, not a gate.  The rows the
//! solver compiles are matched to its free columns (Dulmage–Mendelsohn) and the square part is
//! sorted into strongly connected blocks (Tarjan), each of which could be solved on its own once
//! the blocks it reads are.  What it prints — how many blocks, how large, how deep the chain, and
//! what each large block is made of — says whether a document is one simultaneous system or a
//! sequence of small ones.
use gcs_core::{graph, io, system::System};
use std::collections::BTreeMap;

/// Strongly connected components of `succ`, each a list of nodes, in reverse topological order.
fn tarjan(succ: &[Vec<usize>]) -> Vec<Vec<usize>> {
    let n = succ.len();
    let (mut index, mut low, mut on) = (vec![usize::MAX; n], vec![0; n], vec![false; n]);
    let (mut stack, mut out, mut next) = (Vec::new(), Vec::new(), 0);
    for root in 0..n {
        if index[root] != usize::MAX { continue; }
        // iterative, since a layout's chains are long
        let mut work = vec![(root, 0usize)];
        while let Some(&mut (v, ref mut k)) = work.last_mut() {
            if *k == 0 { index[v] = next; low[v] = next; next += 1; stack.push(v); on[v] = true; }
            if *k < succ[v].len() {
                let w = succ[v][*k];
                *k += 1;
                if index[w] == usize::MAX { work.push((w, 0)); } else if on[w] { low[v] = low[v].min(index[w]); }
                continue;
            }
            work.pop();
            if let Some(&(u, _)) = work.last() { low[u] = low[u].min(low[v]); }
            if low[v] == index[v] {
                let mut comp = Vec::new();
                loop { let w = stack.pop().unwrap(); on[w] = false; comp.push(w); if w == v { break; } }
                out.push(comp);
            }
        }
    }
    out
}

/// The part of the layout a constraint belongs to: the component its first named entity is in.
fn part(name: &str) -> String {
    let segs: Vec<&str> = name.split('.').collect();
    let at = if segs.first() == Some(&"pair") && segs.get(1) == Some(&"reference") { 2 }
        else if segs.first() == Some(&"pair") { 1 } else { 0 };
    segs.get(at).map(|s| s.split('[').next().unwrap().to_string()).unwrap_or_else(|| "?".into())
}

fn report(label: &str, e: &gcs_core::program::Elaborated) {
    let sk = &e.sketch;
    let sys = System::new(sk);
    let (adj, row_c) = sys.structure();
    let n_cols = sys.n_free;
    let dm = graph::dulmage_mendelsohn(&adj, n_cols);
    println!("\n== {label}: {} rows, {} free columns, rank {}, over {} rows, under {} cols",
        adj.len(), n_cols, dm.rank, dm.over_rows.len(), dm.under_cols.len());
    // rows of the square part, and for each an edge to the row matched to every column it reads
    let well: Vec<usize> = dm.well_rows.clone();
    let pos: BTreeMap<usize, usize> = well.iter().enumerate().map(|(i, &r)| (r, i)).collect();
    let succ: Vec<Vec<usize>> = well.iter().map(|&r| adj[r].iter().filter_map(|&c| {
        let m = dm.mate_col[c];
        (m >= 0).then(|| pos.get(&(m as usize)).copied()).flatten()
    }).filter(|&w| w != pos[&r]).collect()).collect();
    let comps = tarjan(&succ);
    // depth: longest chain of blocks, over the condensed DAG (reverse topological order)
    let mut comp_of = vec![0; well.len()];
    for (k, c) in comps.iter().enumerate() { for &v in c { comp_of[v] = k; } }
    let mut depth = vec![1usize; comps.len()];
    for (k, c) in comps.iter().enumerate() {
        for &v in c { for &w in &succ[v] { let j = comp_of[w]; if j != k { depth[k] = depth[k].max(depth[j] + 1); } } }
    }
    let mut sizes: BTreeMap<usize, usize> = BTreeMap::new();
    for c in &comps { *sizes.entry(c.len()).or_default() += 1; }
    println!("  {} blocks, longest chain {} blocks; block sizes (rows: count): {:?}",
        comps.len(), depth.iter().max().unwrap_or(&0), sizes);
    let by_id: BTreeMap<u32, &gcs_core::constraints::Constraint> = sk.constraints.iter().map(|c| (c.id, c)).collect();
    let name = |r: gcs_core::model::EntRef| e.map.name_of(r).cloned();
    let mut big: Vec<&Vec<usize>> = comps.iter().filter(|c| c.len() >= 6).collect();
    big.sort_by_key(|c| std::cmp::Reverse(c.len()));
    for c in big.iter().take(8) {
        let mut parts: BTreeMap<String, usize> = BTreeMap::new();
        let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
        let mut cids: Vec<u32> = c.iter().map(|&v| row_c[well[v]]).collect();
        cids.sort(); cids.dedup();
        for cid in &cids {
            let con = by_id[cid];
            let text = io::describe_with(con, &name);
            let first = text.split_whitespace().find(|w| w.contains('.')).unwrap_or("?");
            *parts.entry(part(first)).or_default() += 1;
            *kinds.entry(format!("{:?}", con.kind)).or_default() += 1;
        }
        println!("  block of {} rows ({} constraints): parts {:?}", c.len(), cids.len(), parts);
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
