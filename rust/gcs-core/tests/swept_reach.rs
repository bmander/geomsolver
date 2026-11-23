//! **The contact's road within the blank's reach about the motion's axis** (`brep::swept::closed`'s
//! clipped road, docs/contract/generating.md): cut from a blank under a screw, the tool clipped about
//! the axis a little past the blank, its candidates read with the clip's faces left out, the sweep
//! trimmed within a cylinder about the axis and cut from the blank — the same cut, held here to the
//! volume and area brep-bench's kernels agree on. And the meeting that region's side has with every
//! sheet the screw carries (`ssi::carried_round`): the sheet's paths of the section's crossings of
//! the cylinder, in closed form.
use gcs_core::brep::build::{disc,prism,Profile,ProfileEdge};
use gcs_core::brep::geom::{tolerance,Blend,Carry,Curve,Frame,Surface};
use gcs_core::brep::nurbs::BSpline;
use gcs_core::brep::ssi::{intersect,Ssi};
use gcs_core::brep::swept::closed::cut_from;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use std::sync::Arc;

type V = [f64;3];

/// The section of brep-bench's `swept/spline-bar-screwed-through-rod`, in (x, z), in order round
/// the loop.
const CONTROL: [[f64;2];7] = [[3.5,0.],[4.5,-1.5],[8.,-1.5],[12.,-1.],[12.,1.],[8.,1.5],[4.5,1.5]];

/// That section in the plane y = 0 as the closed uniform cubic B-spline it is, its poles read from
/// `first` round the loop — the first three repeated after the last, on uniform knots — so its seam
/// stands where the span from pole `first` begins.
fn section(first: usize) -> BSpline {
    let poles: Vec<V> = (0..CONTROL.len()+3).map(|k| { let [x,z] = CONTROL[(k+first)%CONTROL.len()]; [x,0.,z] }).collect();
    let knots = (0..poles.len()+4).map(|k| k as f64).collect();
    BSpline::new(3,knots,poles).unwrap()
}

/// How far `p` stands from the z axis.
fn off_axis(p: V) -> f64 { p[0].hypot(p[1]) }

/// The bar screwed through the rod, as brep-bench draws it (its seam at the span from its first pole,
/// beside the rod, where its contact runs): built by the road within the rod's reach, to the volume
/// and area three kernels agree on to 1e-9 — the bar clipped about the axis, so its far end, out to 12
/// from the axis, is never swept.
#[test]
fn a_bar_screwed_through_a_rod_is_cut_within_the_rods_reach() {
    let up = [0.,0.,1.];
    let rod = prism(&disc([0.;3],up,5.),0.,20.).unwrap();
    let section = Profile {origin:[0.;3],normal:[0.,1.,0.],loops:vec![vec![ProfileEdge::Spline(Arc::new(section(0)))]],names:Vec::new()};
    let bar = prism(&section,-8.,8.).unwrap();
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:up,ratio:1.,advance:6.});
    let tol = tolerance(rod.size().max(bar.size()));
    let cut = cut_from(&rod,&bar,&path,[-90f64.to_radians(),1350f64.to_radians()],0.01,tol)
        .unwrap_or_else(|why| panic!("the bar's sweep not cut from the rod: {why:?}"));
    cut.check(tol).unwrap();
    let volume = gcs_core::brep::props::volume(&cut);
    let area: f64 = cut.faces.iter().map(|f| gcs_core::brep::props::area(&cut,f)).sum();
    // (brep-bench's values, the mean of three kernels agreeing to 1e-9 and 3e-10)
    let (v,a) = (1335.319592427,810.568231802);
    assert!(((volume-v)/v).abs() <= 1e-8,"volume {volume}, not {v}");
    assert!(((area-a)/a).abs() <= 1e-8,"area {area}, not {a}");
}

/// The section carried a turn and a half up a screw about z: a cylinder about z meets the sheet in
/// the paths of the section's two crossings of it, each point on the cylinder to the last few bits —
/// and as many paths as the section crosses the cylinder, counted on 4096 samples of it.
#[test]
fn a_cylinder_about_the_screw_meets_a_carried_section_in_its_paths() {
    let curve = Curve::BSpline(Arc::new(section(0)));
    let carry = Carry::Screw {center:[0.;3],axis:[0.,0.,1.],angle:3.*std::f64::consts::PI,advance:9.};
    let sheet = Blend {a:curve.clone(),ta:[3.,10.],b:curve.clone(),tb:[3.,10.],carry,closed:true};
    let sheet = Surface::Blend(Frame::about([0.;3],[0.,0.,1.]),Arc::new(sheet));
    for r in [4.,5.,7.25,11.] {
        let cylinder = Surface::Cylinder(Frame::about([0.;3],[0.,0.,1.]),r);
        let Ssi::Curves(paths) = intersect(&cylinder,&sheet,1e-9) else { panic!("radius {r}: no paths read") };
        let crossings = (0..4096).filter(|&k| {
            let at = |k: usize| off_axis(curve.point(3.+7.*(k%4096) as f64/4096.))-r;
            (at(k) < 0.) != (at(k+1) < 0.)
        }).count();
        assert_eq!(paths.len(),crossings,"radius {r}");
        for c in &paths {
            assert!(matches!(c,Curve::Iso(..)),"radius {r}: a {} for a path",c.kind());
            for k in 0..=16 {
                let p = c.point(k as f64/16.);
                assert!((off_axis(p)-r).abs() <= 1e-12*(1.+r),"radius {r}: {p:?} off the cylinder by {:e}",off_axis(p)-r);
            }
        }
    }
}
