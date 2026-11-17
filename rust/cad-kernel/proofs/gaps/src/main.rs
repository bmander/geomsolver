//! Three runs of the kernel that depart from what its certified layers are documented to
//! guarantee, each found by writing the guarantee down exactly (`../README.md`). Each prints what
//! the kernel answers beside what is true. None is a drawing's case: they are inputs a contract
//! allows, chosen to show where the contract stops.
use cad_kernel::util::{interval::Iv,isolate,predicates};

/// 1. `predicates::orient` is the sign of its determinant only where no product in it underflows
/// or overflows: a positively oriented tetrahedron of side `s` reads as coplanar where `s³` does.
/// (`Predicates.v` states the predicates over a range, `tame`, for this reason.)
fn orient_out_of_range() {
    for s in [1.,1e-100,1e-108,1e103] {
        let sign = predicates::orient([0.,0.,0.],[s,0.,0.],[0.,s,0.],[0.,0.,s]);
        println!("orient, a tetrahedron of side {s:e}: {sign} (true: 1)");
    }
}

/// `F(x) = (x − 0.9) · max(0, |x − 0.5| − 0.1)`: nought on all of `[0.4, 0.6]`, and a crossing at
/// `0.9`. Its enclosures enclose; its run is the stretch it is nought over.
struct Flat;

fn flat(x: f64) -> f64 { (x-0.9)*((x-0.5).abs()-0.1).max(0.) }

impl isolate::System<1> for Flat {
    fn at(&self,x: [f64;1]) -> ([f64;1],[[f64;1];1]) {
        let h = 1e-7;
        ([flat(x[0])],[[(flat(x[0]+h)-flat(x[0]-h))/(2.*h)]])
    }
    fn value(&self,x: [f64;1]) -> [Iv;1] { self.over(&[[x[0],x[0]]]).0 }
    fn over(&self,bx: &[[f64;2];1]) -> ([Iv;1],[[Iv;1];1]) {
        let x = Iv::new(bx[0][0],bx[0][1]);
        let d = x-Iv::point(0.5);
        let least = if d.has_zero() { 0. } else { d.lo().abs().min(d.hi().abs()) };
        let g = Iv::new(least,d.mag())-Iv::point(0.1);
        let g = Iv::new(g.lo().max(0.),g.hi().max(0.));
        // (its slope between any two points of [0, 1] is within 0.9 + 0.4 of nought)
        ([(x-Iv::point(0.9))*g],[[Iv::new(-1.3,1.3)]])
    }
    fn run(&self,x: [f64;1]) -> Option<[f64;2]> { (x[0] >= 0.4 && x[0] <= 0.6).then_some([0.4,0.6]) }
}

/// 2. `isolate::solve` drops the box it is reading when a run is proved through its middle, though
/// the box reaches past the run: the crossing at 0.9 is lost over `[0, 1]`, whose middle is on the
/// run, and found over `[0.7, 1]`. (`Isolate.v`: `solve_misses_none`.)
fn solve_drops_a_box_for_a_run() {
    let near = |x: [f64;1]| flat(x[0]).abs() <= 1e-9;
    println!("solve over [0, 1]:   {:?} (true: a run over [0.4, 0.6] and a root at 0.9)",
        isolate::solve(&Flat,[[0.,1.]],1e-9,&near));
    println!("solve over [0.7, 1]: {:?}",isolate::solve(&Flat,[[0.7,1.]],1e-9,&near));
}

/// 3. `isolate::roots` answers `Ok` with no root for `g(t) = 1e6 (t − 0.3)` where its rate is not
/// enclosed (`Iv::ANY`, which `Function::slope` may answer): every stretch is unsure, and one too
/// narrow to halve that `near` does not accept is dropped, where `solve` would refuse it as
/// `GaveUp::Unsettled`. (`Isolate.v`: `roots_empty_means_none`.)
fn roots_drops_a_stretch_nothing_proved() {
    let g = |t: f64| 1e6*(t-0.3);
    let at = |t: f64| (g(t),1e6);
    let value = |t: f64| (Iv::point(t)-Iv::point(0.3))*Iv::point(1e6);
    let near = |t: f64| g(t).abs() <= 1e-9;
    let unknown = |_: f64,_: f64| Iv::ANY;
    let found = isolate::roots(&isolate::Parts {at:&at,value:&value,slope:&unknown},[0.29,0.3137],1e-6,&near);
    println!("roots, the rate unknown:  {found:?} (true: a root at 0.3)");
    let known = |_: f64,_: f64| Iv::new(1e6,1e6);
    let found = isolate::roots(&isolate::Parts {at:&at,value:&value,slope:&known},[0.29,0.3137],1e-6,&near);
    println!("roots, the rate enclosed: {found:?}");
}

fn main() {
    orient_out_of_range();
    solve_drops_a_box_for_a_run();
    roots_drops_a_stretch_nothing_proved();
}
