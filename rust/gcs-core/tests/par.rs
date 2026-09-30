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

#[test]
fn a_sort_on_every_core_is_the_serial_sort() {
    // a scrambled run of packed keys, long enough to be split, and lengths about the run's size
    let mut rng = gcs_core::rng::Rng::new(7);
    for n in [0,1,5,(1 << 16)-1,1 << 16,(1 << 16)+3,200_001,1 << 20] {
        let keys: Vec<u128> = (0..n).map(|i| ((rng.uniform(0.,1e6) as u128) << 32) | i as u128 % 977).collect();
        let (mut mine,mut serial) = (keys.clone(),keys);
        par::sort(&mut mine);
        serial.sort_unstable();
        assert_eq!(mine,serial,"{n} keys");
    }
}
