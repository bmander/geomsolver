//! **The depth trim on a ring's sweep** (#942, `swept::depth`): a candidate sheet is on the sweep's
//! boundary exactly where no instant but its own holds its point, read on brep-bench's ring carried
//! along a spline.
use gcs_core::brep::swept::depth::Depth;
use gcs_core::brep::swept::rings::Ring;
use gcs_core::brep::trajectory::Trajectory;
use std::f64::consts::{FRAC_PI_2,PI};

type V = [f64;3];

/// The frame brep-bench's runner carries the ring by: its `x` along the curve's tangent, its `z` up.
#[derive(Debug)]
struct Runner(gcs_core::brep::nurbs::BSpline);
impl gcs_core::brep::trajectory::Path for Runner {
    fn at(&self,t: f64) -> Result<gcs_core::brep::trajectory::Motion,gcs_core::brep::KernelError> {
        let (point,d,dd) = self.0.d2(t);
        let speed2 = d[0]*d[0]+d[1]*d[1];
        let (c,s) = (d[0]/speed2.sqrt(),d[1]/speed2.sqrt());
        let turning = (d[0]*dd[1]-d[1]*dd[0])/speed2;
        let r = [[c,-s,0.],[s,c,0.],[0.,0.,1.]];
        let dr = [[-s*turning,-c*turning,0.],[c*turning,-s*turning,0.],[0.;3]];
        Ok(gcs_core::brep::trajectory::Motion::of_parts(r,dr,point,d))
    }
}

/// The torus of radii 1.5 and 0.5 about `y` through (0, 0, 0.5), read as a ring, and the runner's
/// frame along the bench's cubic.
fn ring_along() -> (Ring,Trajectory) {
    let tool = gcs_core::brep::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = gcs_core::brep::nurbs::BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],
        vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    (Ring::read(&tool).unwrap(),Trajectory::General(std::sync::Arc::new(Runner(curve))))
}

fn add(a: V,b: V) -> V { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
fn sc(a: V,k: f64) -> V { [a[0]*k,a[1]*k,a[2]*k] }
fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }

/// The torus's point in its own coordinates at `α` round its axis (−π/2 its bottom meridian, θ*)
/// and `ψ` round its tube (0 away from the axis), and its normal there.
fn local(alpha: f64,psi: f64) -> (V,V) {
    let er = [alpha.cos(),0.,alpha.sin()];
    let n = add(sc(er,psi.cos()),[0.,psi.sin(),0.]);
    (add(add([0.,0.,0.5],sc(er,1.5)),sc(n,0.5)),n)
}

/// The contact parallel's `ψ₀` at the instant `t`, branch `j` half a turn on: `n · v = 0` on the
/// normal circle at `α = 0`.
fn psi0(path: &Trajectory,t: f64,j: usize) -> f64 {
    let m = path.at(t).unwrap();
    let v = m.velocity([1.5,0.,0.5]);
    (-dot(m.vector([1.,0.,0.]),v)).atan2(dot(m.vector([0.,1.,0.]),v))+j as f64*PI
}

/// **Each sheet's kept piece bounds the sweep, and the meridian's strip is kept whole** (#942, the
/// depth trim): a hair inside a point of each kept piece — the tool where the roll starts, behind
/// its bottom meridian and in the wedge before it beside the tube's rim, the strip swept by the
/// bottom meridian, each tube beyond its bridge — the whole roll's depth is that hair below nought,
/// a hair outside it that hair above. The strip's touching half has no basin but its own at all;
/// each tube is held from beyond on the far side of the meridian, by more than its stretch beside
/// θ* is (under 1e-7: the sliver, its fold and the far leaf there stand within that of one another).
#[test]
fn a_rings_kept_pieces_bound_its_sweep_by_depth() {
    let (ring,path) = ring_along();
    let depth = Depth::new(&ring,&path,[0.,1.],4000).unwrap();
    let place = |a: f64,p: f64,t: f64| { let (x,n) = local(a,p); let pose = path.pose(t).unwrap(); (pose.point(x),pose.vector(n)) };
    let least = |x: V| depth.minima(x).iter().map(|c| c.depth).fold(f64::INFINITY,f64::min);
    let h = 1e-6;
    let mut cases = vec![(-FRAC_PI_2-0.05,psi0(&path,0.,0)+1.,0.),(-FRAC_PI_2+0.1,psi0(&path,0.,0)-0.005,0.)];
    for t in [0.003,0.3,0.7,0.997] {
        cases.extend([(-FRAC_PI_2,psi0(&path,t,0)+1.,t),(-FRAC_PI_2+0.4,psi0(&path,t,0),t),(-FRAC_PI_2-0.4,psi0(&path,t,1),t)]);
    }
    for (a,p,t) in cases {
        let (x,n) = place(a,p,t);
        let (inward,on,outward) = (least(add(x,sc(n,-h))),least(x),least(add(x,sc(n,h))));
        assert!((inward+h).abs() < 1e-9 && on.abs() < 1e-12 && (outward-h).abs() < 1e-9,"α {a} ψ {p} t {t}: {inward} {on} {outward}");
    }
    for t in [0.1,0.5,0.9] {
        let p0 = psi0(&path,t,0);
        for k in 1..12 {
            let (x,_) = place(-FRAC_PI_2,p0+PI*k as f64/12.,t);
            assert!(depth.cover(x,t).is_none(),"the strip at t {t}, ψ − ψ0 {}",PI*k as f64/12.);
        }
        // tube 0 kept in front of θ*, held from behind it; its stretch beside θ* within 1e-7
        let (front,_) = place(-FRAC_PI_2+0.1,p0,t);
        let (behind,_) = place(-FRAC_PI_2-0.1,p0,t);
        assert!(depth.cover(front,t).is_none_or(|c| c.depth >= 0.),"t {t}");
        assert!(depth.cover(behind,t).is_some_and(|c| c.depth < -1e-6),"t {t}");
        for e in [0.001,0.005,0.01] {
            let (x,_) = place(-FRAC_PI_2+e,p0,t);
            assert!(depth.cover(x,t).is_none_or(|c| c.depth > -1e-7),"t {t} e {e}");
        }
    }
}

/// **The wedge the ring where it starts keeps beside its tube's rim is traced to its corner, each
/// point with its partner** (#942, the depth trim): along a path turning from its start, the ring
/// there stands out past the wall the later instants sweep, in a wedge in front of its bottom
/// meridian beside the tube's rim, held from beyond where the tube of an instant just after reaches
/// over it. That seam, traced as a graph over the angle from θ* (`Depth::seam_over`), runs into the
/// corner where the meridian meets the rim — the wedge's width there falls below the tolerance —
/// and each point carries the instant holding it from beyond and the tool's point there, which lies
/// on that instant's contact parallel: both faces' parameters by construction.
#[test]
fn the_start_wedges_seam_is_traced_into_its_corner_with_its_partner() {
    let (ring,path) = ring_along();
    let depth = Depth::new(&ring,&path,[0.,1.],4000).unwrap();
    let tol = 1e-5;
    let p0 = psi0(&path,0.,0);
    let pose = path.pose(0.).unwrap();
    let sheet = |e: f64,s: f64| (pose.point(local(-FRAC_PI_2+e,p0+s).0),0.);
    let seam = depth.seam_over(&sheet,[0.3,2.5e-4],&|_| [0.,-0.3],tol).unwrap();
    assert!(seam.len() >= 3 && seam.last().unwrap().e == 2.5e-4,"{} points",seam.len());
    for p in &seam {
        assert!(p.cover.depth >= 0. && p.cover.depth <= tol,"{p:?}");
        // its partner: an instant just after, the tool's point there on tube 0's parallel
        let at = p.cover.at;
        let rho = at[0].hypot(at[2]-0.5);
        let psi = at[1].atan2(rho-1.5);
        let d = (psi-psi0(&path,p.cover.t,0)+PI).rem_euclid(2.*PI)-PI;
        assert!(p.cover.t > 0. && p.cover.t < 0.05 && 0.5*d.abs() <= tol,"{p:?}: ψ off by {d}");
    }
    // the wedge narrowing to its corner: at the last point under the tolerance across
    let last = seam.last().unwrap();
    let rim = pose.point(local(-FRAC_PI_2+last.e,p0).0);
    assert!(gcs_core::brep::util::space::distance(rim,last.x) <= 2.*tol,"{last:?}");
}
