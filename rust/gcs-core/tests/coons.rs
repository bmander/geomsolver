//! The bilinearly blended Coons patch of four curves (`nurbs::coons`), and the degree elevation
//! that makes opposite curves one space (`BSpline::elevated`): brep-bench's
//! fill/coons-patch-of-four-parabolas.

use gcs_core::brep::build::{face,sheet,Profile,ProfileEdge};
use gcs_core::brep::geom::V;
use gcs_core::brep::nurbs::{coons,distinct,BSpline};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::sew::sewn;
use gcs_core::brep::topo::Brep;
use gcs_core::space::distance;
use std::sync::Arc;

fn bezier(poles: &[V]) -> BSpline {
    let d = poles.len()-1;
    BSpline::new(d,[vec![0.;d+1],vec![1.;d+1]].concat(),poles.to_vec()).unwrap()
}

/// `n + 1` parameters evenly over `c`'s domain.
fn along(c: &BSpline,n: usize) -> Vec<f64> {
    let [a,b] = c.domain();
    (0..=n).map(|k| a+(b-a)*k as f64/n as f64).collect()
}

/// A cubic over `[0, 3]` with a simple knot and a double one, and a rational quarter circle.
fn curves() -> Vec<BSpline> {
    let cubic = BSpline::new(3,vec![0.,0.,0.,0.,1.,2.,2.,3.,3.,3.,3.],
        vec![[0.,0.,0.],[1.,2.,0.],[2.,-1.,1.],[3.,3.,0.],[4.,0.,2.],[5.,1.,-1.],[6.,0.,0.]]).unwrap();
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let quarter = BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]],vec![1.,h,1.]).unwrap();
    vec![cubic,quarter]
}

/// **A curve raised is the same curve**, point for point at the same parameters, its degree `by`
/// higher and each distinct knot `by` times more: as continuous at each as it was, so no more poles
/// than that asks. A circle raised is still a circle.
#[test]
fn a_curve_raised_is_the_same_curve_on_each_knot_once_more() {
    for c in curves() {
        for by in 1..=3 {
            let e = c.elevated(by).unwrap();
            assert_eq!(e.degree,c.degree+by);
            assert_eq!(e.domain(),c.domain());
            let (dc,de) = (distinct(&c.knots),distinct(&e.knots));
            assert_eq!(dc.len(),de.len());
            for (a,b) in dc.iter().zip(&de) { assert_eq!((b.0,b.1),(a.0,a.1+by)); }
            assert_eq!(e.knots.len(),e.poles.len()+e.degree+1);
            for t in along(&c,97) {
                let off = distance(c.point(t),e.point(t));
                assert!(off < 1e-13,"raised by {by}, {off:e} off at {t}");
            }
        }
    }
    let quarter = &curves()[1];
    let e = quarter.elevated(2).unwrap();
    for t in along(&e,50) { assert!((distance(e.point(t),[0.;3])-1.).abs() < 1e-14); }
}

/// The four parabolic arcs of fill/coons-patch-of-four-parabolas, end to end round the square.
fn arcs() -> [BSpline;4] {
    [
        bezier(&[[0.,0.,10.],[5.,0.,14.],[10.,0.,10.]]),
        bezier(&[[10.,0.,10.],[10.,5.,6.],[10.,10.,10.]]),
        bezier(&[[10.,10.,10.],[5.,10.,6.],[0.,10.,10.]]),
        bezier(&[[0.,10.,10.],[0.,5.,14.],[0.,0.,10.]]),
    ]
}

/// **The Coons patch of four parabolas is the saddle the case states**: at `(u, v)` it stands over
/// `(10u, 10v)` at `10 + 8u(1 − u)(1 − 2v) + 8v(1 − v)(1 − 2u)`, and its boundary rows are the arcs'
/// own poles, so its edges are the arcs bit for bit.
#[test]
fn the_coons_patch_of_four_parabolas_is_the_saddle() {
    let [a,b,c,d] = arcs();
    let net = coons([&a,&b,&c,&d]).unwrap();
    assert_eq!((net.du,net.dv),(2,2));
    for i in 0..=20 { for j in 0..=20 {
        let (u,v) = (i as f64/20.,j as f64/20.);
        let z = 10.+8.*u*(1.-u)*(1.-2.*v)+8.*v*(1.-v)*(1.-2.*u);
        let off = distance(net.point(u,v),[10.*u,10.*v,z]);
        assert!(off < 1e-13,"{off:e} off at ({u}, {v})");
    } }
    assert_eq!(net.column(0).poles,a.poles);
    assert_eq!(net.row(net.poles.len()-1).poles,b.poles);
    assert_eq!(net.column(net.poles[0].len()-1).poles,c.reversed().poles);
    assert_eq!(net.row(0).poles,d.reversed().poles);
}

/// **The patch of curves of other degrees and knots, over other domains, is the Coons formula**:
/// a cubic with an inner knot, a line, a quadratic and a quartic, each over its own interval, the
/// net equal at every `(u, v)` to `(1 − v) c₀(u) + v c₂(u) + (1 − u) d₀(v) + u d₁(v) − B(u, v)`
/// read off the curves themselves, each's parameter carried onto `[0, 1]`.
#[test]
fn a_coons_patch_of_unlike_curves_is_their_blend() {
    let (p00,p10,p11,p01) = ([0.,0.,0.],[8.,1.,1.],[9.,7.,-2.],[-1.,6.,3.]);
    let e0 = BSpline::new(3,vec![2.,2.,2.,2.,2.5,4.,4.,4.,4.],vec![p00,[2.,-2.,1.],[4.,3.,2.],[6.,-1.,0.],p10]).unwrap();
    let e1 = BSpline::new(1,vec![-1.,-1.,1.,1.],vec![p10,p11]).unwrap();
    let e2 = BSpline::new(2,vec![0.,0.,0.,5.,5.,5.],vec![p11,[4.,9.,0.],p01]).unwrap();
    let e3 = BSpline::new(4,vec![0.,0.,0.,0.,0.,1.,1.,1.,1.,1.],vec![p01,[-2.,4.,1.],[0.,3.,-2.],[1.,1.,4.],p00]).unwrap();
    let net = coons([&e0,&e1,&e2,&e3]).unwrap();
    assert_eq!((net.du,net.dv),(3,4));
    let at = |c: &BSpline,s: f64| { let [a,b] = c.domain(); c.point(a+(b-a)*s) };
    for i in 0..=16 { for j in 0..=16 {
        let (u,v) = (i as f64/16.,j as f64/16.);
        let want: V = std::array::from_fn(|k| (1.-v)*at(&e0,u)[k]+v*at(&e2,1.-u)[k]+(1.-u)*at(&e3,1.-v)[k]+u*at(&e1,v)[k]
            -((1.-u)*(1.-v)*p00[k]+u*(1.-v)*p10[k]+(1.-u)*v*p01[k]+u*v*p11[k]));
        let off = distance(net.point(u,v),want);
        assert!(off < 1e-12,"{off:e} off at ({u}, {v})");
    } }
}

/// **A loop that does not close, or of rational curves, is refused**, saying which.
#[test]
fn a_coons_patch_is_refused_an_open_loop_and_rational_curves() {
    let [a,b,c,_] = arcs();
    let short = bezier(&[[0.,10.,10.],[0.,5.,14.],[0.,0.5,10.]]);
    let e = coons([&a,&b,&c,&short]).unwrap_err();
    assert!(e.contains("open") && e.contains("curve 3"),"{e}");
    let quarter = &curves()[1];
    assert!(coons([quarter,&b,&c,&a]).unwrap_err().contains("rational"));
}

/// **The block the Coons patch of four parabolas caps** (fill/coons-patch-of-four-parabolas): the
/// patch's sheet sewn to the four upright walls under the arcs and the floor is a closed solid of
/// volume 1000, its rise and fall over the square cancelling, and of the area OCCT, next.BREP and
/// cadrs agree on.
#[test]
fn the_block_under_the_coons_patch_of_four_parabolas() {
    let arcs = arcs();
    let down = |p: V| [p[0],p[1],0.];
    let mut faces: Vec<Brep> = arcs.iter().map(|c| {
        let (a,b) = (c.poles[0],c.poles[2]);
        let out = [(0.5*(a[0]+b[0])-5.)/5.,(0.5*(a[1]+b[1])-5.)/5.,0.];
        face(&Profile {origin:down(a),normal:out,names:Vec::new(),loops:vec![vec![
            ProfileEdge::Spline(Arc::new(c.clone())),
            ProfileEdge::Line {a:b,b:down(b)},
            ProfileEdge::Line {a:down(b),b:down(a)},
            ProfileEdge::Line {a:down(a),b:a},
        ]]}).unwrap()
    }).collect();
    let floor = [[0.,0.,0.],[0.,10.,0.],[10.,10.,0.],[10.,0.,0.]];
    faces.push(face(&Profile {origin:floor[0],normal:[0.,0.,-1.],names:Vec::new(),
        loops:vec![(0..4).map(|k| ProfileEdge::Line {a:floor[k],b:floor[(k+1)%4]}).collect()]}).unwrap());
    let [a,b,c,d] = &arcs;
    faces.push(sheet(coons([a,b,c,d]).unwrap()).unwrap());
    let solid = sewn(&faces,1e-7).unwrap();
    solid.check(1e-9).unwrap();
    assert_eq!(solid.faces.len(),6);
    let v = volume(&solid);
    let s: f64 = solid.faces.iter().map(|f| area(&solid,f)).sum();
    assert!((v/1000.-1.).abs() < 1e-9,"volume {v}");
    assert!((s/614.1210047563-1.).abs() < 1e-9,"area {s}");
}
