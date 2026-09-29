//! `par`: work spread over the cores and answered in order, whatever the threads did.
use gcs_core::par;

#[test]
fn a_map_is_the_serial_map_in_order() {
    let items: Vec<u64> = (0..1000).collect();
    let squares = par::map(&items,|&x| x*x);
    assert_eq!(squares,items.iter().map(|&x| x*x).collect::<Vec<_>>());
    assert!(par::indices(0,|i| i).is_empty());
}

#[test]
fn each_thread_keeps_its_own_state() {
    // a state counting the items its thread took: the counts over every thread cover the items once
    let taken = par::indices_with(500,|| 0_usize,|seen,i| { *seen += 1; (i,*seen) });
    assert_eq!(taken.iter().map(|(i,_)| *i).collect::<Vec<_>>(),(0..500).collect::<Vec<_>>());
    assert!(taken.iter().all(|&(_,seen)| seen >= 1));
    assert!(par::threads() >= 1);
}
