//! **Where two surfaces meet, as a topological answer** (#735 A2, `brep::meeting`): held complete
//! against an independent dense sampling — every pair of the kernel's surface kinds, placed at
//! seeded random, met over boxes of their parameters, and every point where a fine grid across
//! one crosses the other (within the other's box, and steeper than a trace is asked to follow) on
//! some branch of the meeting, so no branch is missed; and its singular points and its coincidences
//! classified where they are known: a ball resting on a plane touches it at one point alone, two
//! cylinders of one radius whose axes cross meet in branches crossing at two points, a plane on a
//! cylinder's side touches it along a line, and a surface met with itself is one surface.
use gcs_core::brep::geom::{Curve,Frame,Surface,V};
use gcs_core::brep::meeting::{meet,Contact,Singular};
use gcs_core::brep::nurbs::{fit_net,BSpline};
use gcs_core::space::{cross,distance,norm,unit};
use std::f64::consts::{PI,TAU};
use std::sync::Arc;

/// A surface of each kind the kernel builds faces on, and a box of its parameters, framed by `f`.
fn kinds(f: Frame) -> Vec<(&'static str,Surface,[[f64;2];2])> {
    let spline = BSpline::new(3,vec![0.,0.,0.,0.,0.5,1.,1.,1.,1.],
        [[-1.6,-0.4,0.],[-0.8,0.9,0.],[0.,-0.8,0.],[0.8,0.7,0.],[1.6,-0.3,0.]].map(|p| f.at(p)).to_vec()).unwrap();
    let net = fit_net(&|u,v| f.at([4.*u-2.,4.*v-2.,0.4*(2.*u+0.5).sin()*(3.*v).cos()]),3,3,8,8,1e-9).unwrap().0;
    vec![
        ("plane",Surface::Plane(f),[[-2.,2.],[-2.,2.]]),
        ("cylinder",Surface::Cylinder(f,1.),[[0.,TAU],[-2.,2.]]),
        ("cone",Surface::Cone(f,1.,0.4),[[0.,TAU],[-1.5,1.5]]),
        ("sphere",Surface::Sphere(f,1.2),[[0.,TAU],[-1.4,1.4]]),
        ("torus",Surface::Torus(f,1.4,0.5),[[0.,TAU],[0.,TAU]]),
        ("extrusion",Surface::Extrusion(f,Arc::new(Curve::BSpline(Arc::new(spline)))),[[0.,1.],[-2.,2.]]),
        ("sheet",Surface::BSpline(Frame::WORLD,Arc::new(net)),[[0.,1.],[0.,1.]]),
    ]
}

/// Whether `uv` lies within the box `d` of `s`'s parameters, a periodic one read round its period.
fn within(s: &Surface,d: [[f64;2];2],uv: [f64;2]) -> bool {
    let periods = s.periods();
    (0..2).all(|k| {
        let mut x = uv[k];
        if let Some(p) = periods[k] { x = d[k][0]+(x-d[k][0]).rem_euclid(p); }
        x >= d[k][0] && x <= d[k][1]
    })
}

/// Where a `n × n` grid across `a`'s box crosses `b` within `b`'s box: each grid line's sign
/// changes of `b`'s distance, bisected, kept where the two meet steeper than `steep`.
fn sampled(a: &Surface,da: [[f64;2];2],b: &Surface,db: [[f64;2];2],n: usize,steep: f64) -> Vec<V> {
    let at = |i: usize,j: usize| [da[0][0]+(da[0][1]-da[0][0])*i as f64/n as f64,da[1][0]+(da[1][1]-da[1][0])*j as f64/n as f64];
    let f = |uv: [f64;2]| b.implicit(a.point(uv));
    let mut out = Vec::new();
    let mut root = |p: [f64;2],q: [f64;2]| {
        let (mut p,mut q,mut fp) = (p,q,f(p));
        for _ in 0..60 {
            let m = [(p[0]+q[0])/2.,(p[1]+q[1])/2.];
            let fm = f(m);
            if (fm < 0.) == (fp < 0.) { p = m; fp = fm; } else { q = m; }
        }
        let x = a.point(p);
        let sine = norm(cross(unit(a.gradient(x)),unit(b.gradient(x))));
        // only where the two cross, on b inside its box (its implicit reads the whole surface, a
        // swept curve's past its ends along its end tangents)
        let uv = b.inverse(x);
        if sine >= steep && distance(b.point(uv),x) < 1e-9 && within(b,db,uv) { out.push(x); }
    };
    for i in 0..=n { for j in 0..=n {
        let here = f(at(i,j));
        if i < n && (here < 0.) != (f(at(i+1,j)) < 0.) { root(at(i,j),at(i+1,j)); }
        if j < n && (here < 0.) != (f(at(i,j+1)) < 0.) { root(at(i,j),at(i,j+1)); }
    } }
    out
}

/// **No branch of any two kinds' meeting is missed**: for the surface kind `i` and every kind
/// after it, in three seeded placements each, every point the dense sampling finds where they
/// cross lies on a branch of the meeting, within a ten-thousandth of their size.
fn complete_against_a_dense_sampling(i: usize) {
    let mut rng = gcs_core::rng::Rng::new(735+i as u32);
    let names: Vec<&str> = kinds(Frame::WORLD).iter().map(|k| k.0).collect();
    let mut asked = 0;
    for j in i..names.len() {
        for _ in 0..3 {
            let mut frame = || Frame::new([rng.uniform(-0.5,0.5),rng.uniform(-0.5,0.5),rng.uniform(-0.5,0.5)],
                [rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(0.2,1.)],[rng.uniform(-1.,1.),1.,rng.uniform(-1.,1.)]);
            let (fa,fb) = (frame(),frame());
            let (_,a,da) = kinds(fa).swap_remove(i);
            let (_,b,db) = kinds(fb).swap_remove(j);
            let m = match meet(&a,da,&b,db,1e-7) {
                Ok(m) => m,
                Err(e) => panic!("{} × {}: {e}",names[i],names[j]),
            };
            if m.same { continue }
            // sampled across the sheet, where one is, against the other's closed-form distance: a
            // sheet's distance is a foot found by Newton at every sample
            let sheet = |k: usize| names[k] == "sheet";
            let points = match (sheet(i),sheet(j)) {
                (false,true) => sampled(&b,db,&a,da,240,0.05),
                (true,true) => sampled(&a,da,&b,db,64,0.05),
                _ => sampled(&a,da,&b,db,240,0.05),
            };
            for p in points {
                let off = m.branches.iter().map(|br| distance(br.curve.point(br.curve.inverse(p)),p)).fold(f64::INFINITY,f64::min);
                assert!(off <= 1e-4,"{} × {}: the sampled crossing {p:?} is {off:e} from every one of {} branches",
                    names[i],names[j],m.branches.len());
                asked += 1;
            }
        }
    }
    assert!(asked > 100,"{asked} points sampled");
}

#[test]
fn a_planes_meeting_with_every_kind_is_complete() { complete_against_a_dense_sampling(0) }
#[test]
fn a_cylinders_meeting_with_every_kind_is_complete() { complete_against_a_dense_sampling(1) }
#[test]
fn a_cones_meeting_with_every_kind_is_complete() { complete_against_a_dense_sampling(2) }
#[test]
fn a_spheres_meeting_with_every_kind_is_complete() { complete_against_a_dense_sampling(3) }
#[test]
fn a_toruses_meeting_with_every_kind_is_complete() { complete_against_a_dense_sampling(4) }
#[test]
fn an_extrusions_meeting_with_every_kind_is_complete() { complete_against_a_dense_sampling(5) }
#[test]
fn a_sheets_meeting_with_a_sheet_is_complete() { complete_against_a_dense_sampling(6) }

/// **A ball resting on a plane touches it at one point alone**: a singular point of their meeting,
/// a touch, and no branch crossing anywhere.
#[test]
fn a_ball_on_a_plane_touches_it_at_one_point() {
    let plane = Surface::Plane(Frame::WORLD);
    let ball = Surface::Sphere(Frame::new([0.3,-0.2,1.],[0.,0.,1.],[1.,0.,0.]),1.);
    let m = meet(&plane,[[-2.,2.],[-2.,2.]],&ball,[[0.,TAU],[-1.5,1.5]],1e-9).unwrap();
    let touches: Vec<_> = m.singular.iter().filter(|s| s.kind == Singular::Touch).collect();
    assert_eq!(touches.len(),1,"{:?}",m.singular);
    assert!(distance(touches[0].at,[0.3,-0.2,0.]) <= 1e-6,"{:?}",touches[0].at);
}

/// **Two cylinders of one radius whose axes cross meet in two ellipses crossing where the two
/// touch**: two singular points, each a crossing of branches, at the ends of the common normal of
/// their axes' plane.
#[test]
fn two_equal_cylinders_crossing_meet_in_branches_crossing_twice() {
    let a = Surface::Cylinder(Frame::new([0.;3],[1.,0.,0.],[0.,1.,0.]),1.);
    let b = Surface::Cylinder(Frame::new([0.;3],[0.,1.,0.],[0.,0.,1.]),1.);
    let m = meet(&a,[[0.,TAU],[-2.,2.]],&b,[[0.,TAU],[-2.,2.]],1e-9).unwrap();
    let crossings: Vec<V> = m.singular.iter().filter(|s| matches!(s.kind,Singular::Crossing(_))).map(|s| s.at).collect();
    assert_eq!(crossings.len(),2,"{:?}",m.singular);
    for p in crossings { assert!((p[2].abs()-1.).abs() <= 1e-6 && p[0].abs() <= 1e-6 && p[1].abs() <= 1e-6,"{p:?}"); }
    assert!(m.branches.iter().all(|b| b.contact == Contact::Crossing));
}

/// **A plane on a cylinder's side touches it along a line**: one branch, a touching one; and a
/// cylinder met with itself is one surface.
#[test]
fn a_plane_on_a_cylinders_side_touches_it_along_a_line() {
    let cylinder = Surface::Cylinder(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),1.);
    let plane = Surface::Plane(Frame::new([1.,0.,0.],[1.,0.,0.],[0.,1.,0.]));
    let m = meet(&plane,[[-2.,2.],[-2.,2.]],&cylinder,[[0.,TAU],[-2.,2.]],1e-9).unwrap();
    assert_eq!(m.branches.len(),1);
    assert_eq!(m.branches[0].contact,Contact::Touching);
    assert!(meet(&cylinder,[[0.,PI],[-1.,1.]],&cylinder.clone(),[[0.,TAU],[-2.,2.]],1e-9).unwrap().same);
}
