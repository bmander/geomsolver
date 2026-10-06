//! The block-triangular order of a system's equations (`System::block_order`, `graph::blocks`)
//! and the seam a block is solved through (`System::subset`): the order on documents whose
//! structure is known by construction, and on the spiral-bevel layout; the seam against the whole
//! system's own evaluation, to the bit.
use gcs_core::system::System;

use crate::common::{build, ent};

/// A chain of triangles off one grounded point: each new point is placed by two distances from
/// the two before it, so each is a block of its own, solved after the ones it reads.
const TRIANGLES: &str = "\
use std
in std.front {
a := point
b := point hint((10, 1))
c := point hint((5, 9))
d := point hint((15, 8))
e := point hint((20, 1))
fix((0, 0)) a
a distance(10) b
a horizontal b
a distance(10) c
b distance(10) c
b distance(10) d
c distance(10) d
c distance(10) e
d distance(10) e
}
";

/// Two cranks on grounded pivots joined by a level coupler: no point is placed before the other,
/// so the four equations are one block.
const LINKAGE: &str = "\
use std
in std.front {
o1 := point
o2 := point
a := point hint((2, 4))
b := point hint((12, 5))
fix((0, 0)) o1
fix((20, 0)) o2
o1 distance(5) a
o2 distance(12) b
a distance(9) b
a horizontal b
}
";

/// The free columns a point's coordinates are, ascending.
fn cols_of(sys: &System, e: &gcs_core::program::Elaborated, name: &str) -> Vec<usize> {
    let mut cols: Vec<usize> = e.sketch.entity_params(ent(e, name)).iter()
        .map(|&p| sys.col_of[p as usize]).filter(|&c| c >= 0).map(|c| c as usize).collect();
    cols.sort_unstable();
    cols
}

#[test]
fn a_chain_of_triangles_is_a_block_per_point_in_order() {
    let e = build(TRIANGLES);
    let mut sys = System::new(&e.sketch);
    let order = sys.block_order();
    assert_eq!(order.blocks.len(), 4);
    for (k, name) in ["b", "c", "d", "e"].iter().enumerate() {
        let b = &order.blocks[k];
        assert_eq!(b.rows.len(), 2, "{name}");
        assert_eq!(b.cols, cols_of(&sys, &e, name), "block {k} is {name}'s");
    }
    // b reads nothing, c reads b, d reads c, e reads d: a chain four deep
    assert_eq!(order.level, vec![1, 2, 3, 4]);
    assert_eq!(order.depth(), 4);
    assert!(order.over_rows.is_empty() && order.under_cols.is_empty());
    // memoised: the same order, not worked out again
    assert!(std::sync::Arc::ptr_eq(&order, &sys.block_order()));
}

#[test]
fn a_closed_linkage_is_one_block() {
    let e = build(LINKAGE);
    let mut sys = System::new(&e.sketch);
    let order = sys.block_order();
    assert_eq!(order.blocks.len(), 1);
    assert_eq!(order.blocks[0].rows.len(), 4);
    let mut both = cols_of(&sys, &e, "a");
    both.extend(cols_of(&sys, &e, "b"));
    both.sort_unstable();
    assert_eq!(order.blocks[0].cols, both);
    assert_eq!(order.depth(), 1);
}

/// What is not square is not a block: a free point's columns are the under-determined part, and
/// a length stated twice the over-determined one — with every row it reaches, which is why the
/// repeat is stated where nothing is upstream of it: the coarse decomposition puts every row a
/// redundant row reads, and every row those read, in the over-determined part.
#[test]
fn the_over_and_under_determined_parts_stand_apart() {
    let src = format!("{TRIANGLES}f := point hint((30, 30))\na distance(10) b\n");
    let e = build(&src);
    let mut sys = System::new(&e.sketch);
    let order = sys.block_order();
    assert_eq!(order.under_cols, cols_of(&sys, &e, "f"));
    assert!(order.under_rows.is_empty());
    // the repeated distance joins the rows placing b in the over-determined part
    assert_eq!(order.over_rows.len(), 3);
    assert_eq!(order.over_cols, cols_of(&sys, &e, "b"));
    assert_eq!(order.blocks.len(), 3);
    assert_eq!(order.blocks[0].cols, cols_of(&sys, &e, "c"));
    let in_blocks: usize = order.blocks.iter().map(|b| b.rows.len()).sum();
    assert_eq!(in_blocks + order.over_rows.len() + order.under_rows.len(), sys.hard_rows().len());
}

/// The configured hypoid's layout as recorded, with no backlash or tip relief; the relief's own
/// geometry is `the_tip_relief_orders_whole`'s.
#[test]
fn the_hypoid_layout_is_123_blocks_14_deep() {
    let e = fixtures::gear::read_configured_with(&mut |name, text| fixtures::gear::design(name, text, 25., 12.5, 25.));
    let mut sys = System::new(&e.sketch);
    let order = sys.block_order();
    assert_eq!((sys.hard_rows().len(), sys.n_free), (387, 387));
    assert!(order.over_rows.is_empty() && order.under_cols.is_empty());
    assert_eq!(order.blocks.len(), 123);
    assert_eq!(order.depth(), 14);
    let mut sizes = std::collections::BTreeMap::new();
    for b in &order.blocks {
        assert_eq!(b.rows.len(), b.cols.len());
        *sizes.entry(b.rows.len()).or_insert(0) += 1;
    }
    // a plane's origin where its two axes meet is a block of three
    let want = [(1, 18), (2, 69), (3, 20), (4, 6), (5, 4), (6, 1), (16, 3), (28, 1), (45, 1)];
    assert_eq!(sizes.into_iter().collect::<Vec<_>>(), want);
    // every block reads only the blocks before it: a column a block's rows touch that is not its
    // own is an earlier block's
    let (adj, _) = sys.structure();
    let hard = sys.hard_rows();
    let mut owner = vec![usize::MAX; sys.n_free];
    for (k, b) in order.blocks.iter().enumerate() {
        for &c in &b.cols {
            owner[c] = k;
        }
    }
    for (k, b) in order.blocks.iter().enumerate() {
        for &r in &b.rows {
            let h = hard.binary_search(&r).unwrap();
            for &c in &adj[h] {
                assert!(owner[c] <= k, "block {k} reads block {}'s column", owner[c]);
            }
        }
    }
}

/// The configured pair with its backlash and tip relief: the relief's chamfers, rounds and tops add
/// blocks of their own, and the order is still square with nothing over or under-determined.
#[test]
fn the_tip_relief_orders_whole() {
    let e = fixtures::gear::read_as_configured();
    let mut sys = System::new(&e.sketch);
    let order = sys.block_order();
    let mut sizes = std::collections::BTreeMap::new();
    for b in &order.blocks {
        assert_eq!(b.rows.len(), b.cols.len());
        *sizes.entry(b.rows.len()).or_insert(0) += 1;
    }
    println!("{} rows, {} free, {} blocks, {} deep, sizes {:?}", sys.hard_rows().len(), sys.n_free,
        order.blocks.len(), order.depth(), sizes);
    assert_eq!(sys.hard_rows().len(), sys.n_free);
    assert!(order.over_rows.is_empty() && order.under_cols.is_empty());
}

/// A subset's residuals and Jacobian are the whole system's rows and columns, bit for bit: every
/// block of the order, one scattered subset with every column and the same with half of them.
fn subsets_agree(label: &str, sk: &gcs_core::model::Sketch) {
    let mut sys = System::new(sk);
    let mut z = sys.z0(sk);
    // off the solution, so every residual has something to say
    for (k, v) in z.iter_mut().enumerate() {
        *v += 1e-3 * (1.0 + v.abs()) * (((k * 7919) % 13) as f64 / 13.0 - 0.5);
    }
    let full_r = sys.residuals(&z);
    let full_j = sys.jacobian_dense(&z);
    let order = sys.block_order();
    let mut cases: Vec<(Vec<(usize, usize)>, Vec<usize>)> =
        order.blocks.iter().map(|b| (b.instances.clone(), b.cols.clone())).collect();
    let every: Vec<(usize, usize)> = (0..sys.n_res).step_by(3).map(|r| sys.instance_of(r)).collect();
    let all: Vec<usize> = (0..sys.n_free).collect();
    let half: Vec<usize> = (0..sys.n_free).filter(|c| c % 2 == 1).collect();
    cases.push((every.clone(), all));
    cases.push((every, half));
    assert!(order.blocks.len() > 1, "{label}: a document with an order to test");
    for (instances, cols) in cases {
        let mut sub = sys.subset(&instances, &cols);
        let mut r = vec![0.0; sub.rows.len()];
        sys.subset_residuals(&mut sub, &z, &mut r);
        sys.subset_csr(&mut sub, &z);
        for (k, &row) in sub.rows.iter().enumerate() {
            assert_eq!(r[k].to_bits(), full_r[row].to_bits(), "{label}: row {row}");
            let mut dense = vec![0.0f64; cols.len()];
            for p in sub.indptr[k]..sub.indptr[k + 1] {
                dense[sub.indices[p as usize] as usize] = sub.data[p as usize];
            }
            for (j, &c) in cols.iter().enumerate() {
                assert_eq!(dense[j].to_bits(), full_j.at(row, c).to_bits(),
                    "{label}: J[{row}, {c}]");
            }
        }
    }
}

#[test]
fn a_subset_is_the_whole_systems_rows_to_the_bit() {
    subsets_agree("triangles", &build(TRIANGLES).sketch);
    // a traced curve with contacts on it: the kernel reads its constants where its memory is
    let gear = build(include_str!("../../examples/gear_trace.sv"));
    assert!(!gear.sketch.curve_defs.is_empty());
    subsets_agree("gear_trace", &gear.sketch);
    // spatial kinds, and a scaled system (rows over their units, columns over theirs)
    subsets_agree("hypoid_pitch_cones", &build(include_str!("../../examples/hypoid_pitch_cones.sv")).sketch);
    subsets_agree("hypoid layout", &fixtures::gear::read_as_configured().sketch);
}
