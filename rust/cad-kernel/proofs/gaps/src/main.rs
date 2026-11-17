//! Where the kernel answers outside what `../Predicates.v` states: the exact predicates are the
//! sign of their determinant only where no product in it underflows or overflows, which
//! `expansion.rs` assumes and nothing checks. A positively oriented tetrahedron of side `s` reads
//! as coplanar where `s³` does either. No drawing is near these scales; the statements carry the
//! range (`tame`) for this reason.
use cad_kernel::util::predicates;

fn main() {
    for s in [1.,1e-100,1e-108,1e103] {
        let sign = predicates::orient([0.,0.,0.],[s,0.,0.],[0.,s,0.],[0.,0.,s]);
        println!("orient, a tetrahedron of side {s:e}: {sign} (true: 1)");
    }
}
