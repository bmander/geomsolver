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

/// **A plane holding a hyperboloid's ruling meets it in that ruling and its partner, whatever the
/// tolerance** (`ssi::intersect`, #735): the face a leaning edge lies in, against the hyperboloid
/// the edge turns into, is tangent to it at the throat, and the constant the closed form finds at
/// the hyperbola's centre is rounding (some 1e-14) — read as nothing, not as a hyperbola whose
/// semi-axis, its square root, is some 1e-7 and splits from the lines at any tolerance finer.
#[test]
fn a_plane_holding_a_hyperboloids_ruling_meets_it_in_two_lines_at_any_tolerance() {
    use gcs_core::brep::ssi::{intersect,Ssi};
    let lean: f64 = 0.35;
    // a line in the plane x = 4, leaning to the z axis, turned about it
    let (p,e) = ([4.,0.,0.],[0.,-lean.sin(),lean.cos()]);
    let solid = &gcs_core::brep::swept::closed::revolved([0.;3],[0.,0.,1.],p,e,[-0.8,0.8],1e-9).unwrap()[0];
    let hyperboloid = solid.faces.iter().find(|f| matches!(f.surface,Surface::Revolution(..))).unwrap().surface.clone();
    let plane = Surface::Plane(Frame::about(p,[1.,0.,0.]));
    for tol in [1e-6,1e-7,1e-9] {
        let Ssi::Curves(curves) = intersect(&plane,&hyperboloid,tol) else { panic!("at {tol:e}: no closed form") };
        assert_eq!(curves.iter().map(|c| c.kind()).collect::<Vec<_>>(),["line","line"],"at {tol:e}");
        // the ruling the plane holds is one of them
        let on = |c: &Curve| (0..=8).all(|k| { let s = -0.8+0.2*k as f64; let x = [p[0]+s*e[0]/e[2],p[1]+s*e[1]/e[2],s];
            distance(c.point(c.inverse(x)),x) <= 1e-9 });
        assert!(curves.iter().any(on),"at {tol:e}");
    }
}

/// The sheet the level line through `p` along `e` screws into about the `z` axis — a turn of
/// `angle` and a rise of `advance` over it, from the line screwed back by `from` of that turn —
/// the face of a level triangle on the line screwed (`build::screwed`), its stretch of the line
/// `2 reach` long.
pub(crate) fn screwed_sheet(p: V,e: V,reach: f64,angle: f64,advance: f64,from: f64) -> Surface {
    screwed_sheet_brep(p,e,reach,angle,advance,from).faces[0].surface.clone()
}

/// `screwed_sheet` as a B-rep: the triangle's screwed face alone, its rails and rulings its edges.
pub(crate) fn screwed_sheet_brep(p: V,e: V,reach: f64,angle: f64,advance: f64,from: f64) -> gcs_core::brep::topo::Brep {
    use gcs_core::brep::build::{screwed,Profile,ProfileEdge};
    let off = [p[0]-e[0]*(p[0]*e[0]+p[1]*e[1]),p[1]-e[1]*(p[0]*e[0]+p[1]*e[1]),0.];
    let away = if norm(off) > 1e-9 { unit(off) } else { cross([0.,0.,1.],e) };
    let corners = [[p[0]-reach*e[0],p[1]-reach*e[1],p[2]],[p[0]+reach*e[0],p[1]+reach*e[1],p[2]],
        [p[0]+reach*away[0],p[1]+reach*away[1],p[2]]];
    let triangle = Profile {origin:corners[0],normal:[0.,0.,1.],names:vec![vec!["sheet".into(),String::new(),String::new()]],
        loops:vec![(0..3).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%3]}).collect()]};
    let back = gcs_core::brep::geom::Rigid::turn([0.;3],[0.,0.,1.],-from*angle)
        .then(&gcs_core::brep::geom::Rigid {t:[0.,0.,-from*advance],..gcs_core::brep::geom::Rigid::identity()});
    let mut b = screwed(&triangle.moved(&back),[0.;3],[0.,0.,1.],angle,advance).unwrap();
    b.faces.retain(|f| f.name == "sheet");
    b.compacted()
}

/// **A screwed line's sheet meets what keeps the screw in helices** (`ssi`'s screwed meetings,
/// #735): a cylinder about the axis where the line passes it, twice, or once where the line passes
/// at the cylinder's radius (the sheet touching it along the foot's helix, at any tolerance); a
/// sheet of the same screw where its line meets this one carried to its height, once, and nowhere
/// where the two run parallel at one height; a plane square to the axis in the one ruling at its
/// height. Every point of each read on both surfaces.
#[test]
fn a_screwed_lines_sheet_meets_what_keeps_the_screw_in_helices() {
    use gcs_core::brep::ssi::{intersect,Ssi};
    let (angle,advance) = (5.,1.6);
    // the line x = 2 at z = 0 (its foot (2, 0, 0)), and a line through the axis higher up
    let sheet = screwed_sheet([2.,0.5,0.],[0.,1.,0.],4.,angle,advance,0.);
    let other = screwed_sheet([1.,1.,0.3],unit([1.,1.,0.]),6.,angle,advance,0.);
    let apart = screwed_sheet([3.,0.5,0.],[0.,1.,0.],4.,angle,advance,0.);
    // read where both patches reach (the other sheet starts higher up)
    let on_both = |c: &Curve,a: &Surface,b: &Surface,what: &str| for k in 0..=16 {
        let x = c.point(0.1+0.6*k as f64/16.);
        assert!(a.implicit(x).abs() <= 1e-9 && b.implicit(x).abs() <= 1e-9,"{what}: {x:?} off them by {:e}, {:e}",a.implicit(x),b.implicit(x));
    };
    for tol in [1e-6,1e-7,1e-9] {
        for (r,n) in [(3.,2),(2.,1),(1.5,0)] {
            let cylinder = Surface::Cylinder(Frame::about([0.;3],[0.,0.,1.]),r);
            let Ssi::Curves(curves) = intersect(&sheet,&cylinder,tol) else { panic!("radius {r} at {tol:e}: no closed form") };
            assert_eq!(curves.len(),n,"radius {r} at {tol:e}");
            for c in &curves { assert_eq!(c.kind(),"iso"); on_both(c,&sheet,&cylinder,"a cylinder's helix") }
        }
        let Ssi::Curves(helix) = intersect(&sheet,&other,tol) else { panic!("two sheets at {tol:e}: no closed form") };
        assert_eq!(helix.len(),1,"two sheets at {tol:e}");
        on_both(&helix[0],&sheet,&other,"two sheets' helix");
        assert_eq!(intersect(&sheet,&apart,tol),Ssi::Curves(vec![]),"parallel at one height, at {tol:e}");
        let level = Surface::Plane(Frame::about([0.,0.,0.7],[0.,0.,1.]));
        let Ssi::Curves(ruling) = intersect(&sheet,&level,tol) else { panic!("a level plane at {tol:e}: no closed form") };
        assert_eq!(ruling.iter().map(|c| c.kind()).collect::<Vec<_>>(),["line"],"a level plane at {tol:e}");
        for k in 1..=15 { let x = ruling[0].point(k as f64*0.5); assert!(sheet.implicit(x).abs() <= 1e-9 && (x[2]-0.7).abs() <= 1e-12) }
    }
}

/// **A plane holding a ruling of a screwed line's sheet meets it in that ruling and a curve crossing
/// it where the two touch** (`Curve::SheetCut`, #735): a face along the axis holds its own level
/// edge, a ruling of the edge's sheet, and touches the sheet at the edge's foot, so the meeting there
/// is the ruling and the rest of the plane's cut, `S(−α/β, v)`, through the foot — read about the
/// held ruling by differences, so it stays on both surfaces to rounding however near the foot it is
/// read, and runs through the foot itself.
#[test]
fn a_plane_holding_a_screwed_sheets_ruling_meets_it_in_the_ruling_and_a_cut_through_the_touch() {
    use gcs_core::brep::ssi::{intersect,Ssi};
    let (angle,advance) = (5.,1.6);
    // from a fifth of a turn back, so the ruling the plane holds is inside the patch
    let sheet = screwed_sheet([2.,0.5,0.],[0.,1.,0.],4.,angle,advance,0.2);
    let plane = Surface::Plane(Frame::about([2.,0.,0.],[1.,0.,0.]));
    for tol in [1e-6,1e-7,1e-9] {
        let Ssi::Curves(curves) = intersect(&plane,&sheet,tol) else { panic!("at {tol:e}: no closed form") };
        assert_eq!(curves.iter().map(|c| c.kind()).collect::<Vec<_>>(),["line","sheet cut"],"at {tol:e}");
        let Curve::SheetCut(cut) = &curves[1] else { unreachable!() };
        let [held] = cut.held[..] else { panic!("at {tol:e}: held {:?}",cut.held) };
        // through the foot, and on both surfaces however near it
        assert!(distance(cut.point(held),[2.,0.,0.]) <= 1e-12,"at {tol:e}: {:?}",cut.point(held));
        for d in [0.,1e-12,1e-9,1e-7,1e-6,1e-4,1e-2,0.1] { for v in [held+d,held-d] {
            let x = cut.point(v);
            assert!((x[0]-2.).abs() <= 1e-12 && sheet.implicit(x).abs() <= 1e-10,"at {v}: {x:?}");
        } }
        // the ruling the plane holds is the line x = 2, z = 0
        assert!((0..=4).all(|k| { let x = [2.,k as f64-2.,0.]; distance(curves[0].point(curves[0].inverse(x)),x) <= 1e-12 }));
    }
}

/// **Two coaxial surfaces of revolution with spline meridians meet in circles, in closed form**
/// (#735 A2, `ssi::spline_spline`): two hyperboloids about z, each a leaning line turned (as
/// `swept::closed::revolved` builds a swept edge's, its meridian one rational quadratic), meet
/// where their radii agree, `t₁² + k₁² (z − h₁)² = t₂² + k₂² (z − h₂)²` — a quadratic in `z`, solved
/// here apart from the kernel — each a circle at that height of that radius, not a curve traced.
/// (Traced, a leaning bar turned through a ring sector spent 300 s crossing those traces.)
#[test]
fn two_coaxial_hyperboloids_meet_in_circles_in_closed_form() {
    use gcs_core::brep::ssi::{intersect,Ssi};
    let hyperboloid = |p: V,e: V| {
        let solids = gcs_core::brep::swept::closed::revolved([0.;3],[0.,0.,1.],p,unit(e),[-2.,2.],1e-9).unwrap();
        let [solid] = &solids[..] else { panic!("one solid") };
        solid.faces.iter().find(|f| f.surface.kind() == "revolution").expect("its hyperboloid").surface.clone()
    };
    // throat 2 at z = 0, lean 1; throat 1 at z = 0.3, lean 2
    let (a,b) = (hyperboloid([2.,0.,0.],[0.,1.,1.]),hyperboloid([1.,0.,0.3],[0.,2.,1.]));
    let Ssi::Curves(curves) = intersect(&a,&b,1e-9) else { panic!("in closed form") };
    // 4 + z² = 1 + 4 (z − 0.3)²: 3 z² − 2.4 z − 2.64 = 0
    let d = (2.4f64*2.4+4.*3.*2.64).sqrt();
    let expected = [(2.4-d)/6.,(2.4+d)/6.];
    let mut found: Vec<(f64,f64)> = curves.iter().map(|c| match c {
        Curve::Circle(f,r) => (f.o[2],*r),
        other => panic!("a circle, not {}",other.kind()),
    }).collect();
    found.sort_by(|x,y| x.0.total_cmp(&y.0));
    assert_eq!(found.len(),2,"{found:?}");
    for ((z,r),want) in found.into_iter().zip(expected) {
        assert!((z-want).abs() <= 1e-9,"a circle at {z} against {want}");
        assert!((r-(4.+want*want).sqrt()).abs() <= 1e-9,"of radius {r}");
    }
}

/// **Every kind touching along a curve, and crossing at under a degree, is met completely, with
/// and without the closed forms** (#778 T5's gate on the completeness gate): for each kind of
/// surface, a partner tangent to it along a curve — a plane on a cylinder's side and on a cone's
/// generator, a cylinder round a ball's equator and round a torus's, a plane on a vault's crown
/// (an extrusion) and on a fitted sheet's crest — and that partner moved to cross it at under a
/// degree. Touching, a branch of the meeting touches all along it (`Contact::Touching`), within a
/// ten-thousandth of the curve's points; crossing shallowly, every point a dense sampling finds
/// where they cross (at any angle) lies on a branch within a ten-thousandth. Each met by
/// `meeting::meet_with` with the closed forms taken and bypassed.
#[test]
fn every_kind_touching_along_a_curve_and_crossing_shallowly_is_met_completely() {
    use gcs_core::brep::meeting::meet_with;
    let z = [0.,0.,1.];
    let at = |o: V,n: V| Frame::about(o,n);
    let lean = 0.3f64;
    let (sl,cl) = lean.sin_cos();
    let arc = Surface::Extrusion(Frame::new([-12.,0.,0.],[1.,0.,0.],[0.,1.,0.]),Arc::new(Curve::BSpline(Arc::new(
        BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![[-12.,-12.,10.],[-12.,0.,26.],[-12.,12.,10.]],vec![1.,0.6,1.]).unwrap()))));
    let crest = Surface::BSpline(Frame::WORLD,Arc::new(fit_net(&|u,v| { let (x,y) = (4.*u-2.,4.*v-2.); [x,y,1.-y*y/4.] },3,3,8,8,1e-10).unwrap().0));
    // (each: what, the surface, its box, the partner touching it, its box, the touch's points, and
    // the partner moved off to cross at under a degree)
    let line = |p: V,d: V| (0..=8).map(|k| { let s = -1.+0.25*k as f64; [p[0]+s*d[0],p[1]+s*d[1],p[2]+s*d[2]] }).collect::<Vec<V>>();
    let circle = |c: V,r: f64| (0..16).map(|k| { let a = k as f64*TAU/16.; [c[0]+r*a.cos(),c[1]+r*a.sin(),c[2]] }).collect::<Vec<V>>();
    let full = [0.,TAU];
    let cases: Vec<(&str,Surface,[[f64;2];2],Surface,[[f64;2];2],Vec<V>,Surface)> = vec![
        ("a plane on a cylinder's side",Surface::Cylinder(at([0.;3],z),1.),[full,[-2.,2.]],Surface::Plane(at([1.,0.,0.],[1.,0.,0.])),[[-3.,3.],[-3.,3.]],
            line([1.,0.,0.],z),Surface::Plane(at([1.-1e-4,0.,0.],[1.,0.,0.]))),
        ("a plane on a cone's generator",Surface::Cone(at([0.;3],z),1.,lean),[full,[-1.5,1.5]],Surface::Plane(at([1.,0.,0.],[cl,0.,-sl])),[[-3.,3.],[-3.,3.]],
            line([1.,0.,0.],[sl,0.,cl]),Surface::Plane(at([1.-1e-4,0.,0.],[cl,0.,-sl]))),
        ("a cylinder round a ball's equator",Surface::Sphere(at([0.;3],z),1.),[full,[-1.4,1.4]],Surface::Cylinder(at([0.;3],z),1.),[full,[-2.,2.]],
            circle([0.;3],1.),Surface::Cylinder(at([0.;3],z),1.-1e-4)),
        ("a cylinder round a torus's equator",Surface::Torus(at([0.;3],z),1.4,0.5),[full,full],Surface::Cylinder(at([0.;3],z),1.9),[full,[-2.,2.]],
            circle([0.;3],1.9),Surface::Cylinder(at([0.;3],z),1.9-1e-4)),
        ("a plane on a vault's crown",arc,[[0.,1.],[0.,24.]],Surface::Plane(at([0.,0.,16.],z)),[[-13.,13.],[-13.,13.]],
            line([0.,0.,16.],[11.,0.,0.]),Surface::Plane(at([0.,0.,16.-1e-4],z))),
        ("a plane on a sheet's crest",crest,[[0.,1.],[0.,1.]],Surface::Plane(at([0.,0.,1.],z)),[[-3.,3.],[-3.,3.]],
            line([0.,0.,1.],[1.9,0.,0.]),Surface::Plane(at([0.,0.,1.-1e-4],z))),
    ];
    for (what,a,da,b,db,touch,shallow) in cases {
        for closed in [true,false] {
            let m = meet_with(&a,da,&b,db,1e-7,closed).unwrap_or_else(|e| panic!("{what}, closed forms {closed}: {e}"));
            let touching: Vec<_> = m.branches.iter().filter(|br| br.contact == Contact::Touching).collect();
            assert!(!touching.is_empty(),"{what}, closed forms {closed}: no touch among {} branches",m.branches.len());
            for &p in &touch {
                let off = touching.iter().map(|br| distance(br.curve.point(br.curve.inverse(p)),p)).fold(f64::INFINITY,f64::min);
                assert!(off <= 1e-4,"{what}, closed forms {closed}: the touch's point {p:?} is {off:e} from every touching branch");
            }
            // moved off a ten-thousandth, crossing at under a degree: every crossing sampled on a branch
            let m = meet_with(&a,da,&shallow,db,1e-7,closed).unwrap_or_else(|e| panic!("{what} moved off, closed forms {closed}: {e}"));
            let points = sampled(&a,da,&shallow,db,240,0.);
            assert!(!points.is_empty(),"{what} moved off: no crossing sampled");
            for p in points {
                let off = m.branches.iter().map(|br| distance(br.curve.point(br.curve.inverse(p)),p)).fold(f64::INFINITY,f64::min);
                assert!(off <= 1e-4,"{what} moved off, closed forms {closed}: the sampled crossing {p:?} is {off:e} from every one of {} branches: {:?}",
                    m.branches.len(),m.branches.iter().map(|br| (br.curve.kind(),br.curve.point(br.curve.inverse(p)))).collect::<Vec<_>>());
            }
        }
    }
}

/// **Two surfaces crossing at a fiftieth of a degree are met in their closed form** (#942 F1's
/// gate): a plane crossing a cylinder of radius 100 at 0.02° (`x = r cos θ`, its two lines `y =
/// ± r sin θ`) and a ball of radius 100 (`z = r cos θ`, its circle of radius `r sin θ`), met by
/// `meeting::meet_with` with the closed forms bypassed. Nearly flat beside each other, the two stand
/// sixty tolerances apart between their crossings: a crossing, not a touch. The trace refuses the
/// meeting as shallow, and it is marched in both surfaces' parameters (`parametric`): the two lines,
/// whole along the cylinder's box, and the circle, closed, each within 1e-9 of the closed form —
/// where the level curves in one surface's parameters, the fallback before, closed on themselves
/// about each seed a hair from the ball's pole (eight pieces 4.5e-4 off, none whole).
#[test]
fn two_surfaces_crossing_at_a_fiftieth_of_a_degree_are_met_in_their_closed_form() {
    use gcs_core::brep::meeting::meet_with;
    use gcs_core::brep::query::curve_domain;
    let (r,th,tol) = (100f64,0.02f64.to_radians(),1e-7);
    let along = |c: &Curve,n: usize| -> Vec<V> { let [t0,t1] = curve_domain(c).unwrap(); (0..=n).map(|k| c.point(t0+(t1-t0)*k as f64/n as f64)).collect() };
    // the plane and the cylinder: two lines, each the cylinder's height long
    let cylinder = Surface::Cylinder(Frame::about([0.;3],[0.,0.,1.]),r);
    let plane = Surface::Plane(Frame::about([r*th.cos(),0.,0.],[1.,0.,0.]));
    let m = meet_with(&cylinder,[[-3.*th,3.*th],[-2.,2.]],&plane,[[-3.,3.],[-3.,3.]],tol,false).unwrap();
    assert_eq!(m.branches.len(),2,"two lines: {:?}",m.branches.iter().map(|b| b.curve.kind()).collect::<Vec<_>>());
    let mut sides = Vec::new();
    for br in &m.branches {
        assert_eq!(br.contact,Contact::Crossing);
        let pts = along(&br.curve,400);
        for p in &pts { assert!((p[0]-r*th.cos()).abs() <= 1e-9 && (p[1].abs()-r*th.sin()).abs() <= 1e-9,"{p:?} off the lines") }
        let (z0,z1) = pts.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),p| (a.min(p[2]),b.max(p[2])));
        assert!(z0 <= -2.+1e-9 && z1 >= 2.-1e-9,"a line over [{z0}, {z1}], not the cylinder's height");
        sides.push(pts[0][1].signum());
    }
    assert!(sides[0] != sides[1],"both branches on one line");
    // the plane and the ball: a circle, closed
    let ball = Surface::Sphere(Frame::about([0.;3],[0.,0.,1.]),r);
    let lid = Surface::Plane(Frame::about([0.,0.,r*th.cos()],[0.,0.,1.]));
    let w = 3.*r*th;
    let m = meet_with(&ball,[[0.,TAU],[PI/2.-3.*th,PI/2.]],&lid,[[-w,w],[-w,w]],tol,false).unwrap();
    assert_eq!(m.branches.len(),1,"one circle");
    let pts = along(&m.branches[0].curve,800);
    for p in &pts { assert!((p[2]-r*th.cos()).abs() <= 1e-9 && (p[0].hypot(p[1])-r*th.sin()).abs() <= 1e-9,"{p:?} off the circle") }
    assert!(distance(pts[0],pts[pts.len()-1]) <= 1e-9,"the circle not closed");
    let mut angles: Vec<f64> = pts.iter().map(|p| p[1].atan2(p[0])).collect();
    angles.sort_by(f64::total_cmp);
    let gap = angles.windows(2).map(|w| w[1]-w[0]).fold(TAU-(angles[angles.len()-1]-angles[0]),f64::max);
    assert!(gap <= 0.05,"the circle not whole: a gap of {gap} rad");
}

/// **A cylinder touching a quadric or a torus from outside, its axis crossing the other's
/// straight ways, touches it at one point alone** (#996 §3): the meeting a rim's round has with a
/// blend whose foot it crosses. A cylinder of each radius 0.5, 2 and 8 laid on a cylinder, a
/// cone, a ball and a torus at a point of its outside, its axis along the other's parallel there
/// (square across a cylinder's or a cone's rulings, the rim's round on the blend) and turned 30°
/// and 60° from it: no branch, and one touch, at that point — the search for what no seed reaches
/// passing it (`meeting::meet_with`, `ssi::alone_over`), which found it, traced nothing from it and
/// refused the meeting, "a cylinder and a cone meet at … in a curve that could not be traced".
#[test]
fn a_cylinder_laid_across_a_surface_from_outside_touches_it_at_one_point_alone() {
    let mut failed: Vec<String> = Vec::new();
    let others: Vec<(&str,Surface,[[f64;2];2],[f64;2])> = vec![
        ("a cylinder",Surface::Cylinder(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),8.),[[0.,TAU],[-6.,6.]],[0.3,0.5]),
        ("a cone",Surface::Cone(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),4.,0.3),[[0.,TAU],[-3.,3.]],[0.3,0.5]),
        ("a ball",Surface::Sphere(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),6.),[[0.,TAU],[-1.5,1.5]],[0.3,0.2]),
        ("a torus",Surface::Torus(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),10.,3.),[[0.,TAU],[0.,TAU]],[0.3,0.1]),
    ];
    for (what,b,db,uv) in &others {
        let q = b.point(*uv);
        let n = unit(b.gradient(q));
        // the other's parallel there
        let (_,su,_) = b.d1(*uv);
        let e1 = unit(su);
        let e2 = cross(n,e1);
        for r in [0.5,2.,8.] { for turn in [0f64,30.,60.] {
            let (sn,cs) = turn.to_radians().sin_cos();
            let axis = [e1[0]*cs+e2[0]*sn,e1[1]*cs+e2[1]*sn,e1[2]*cs+e2[2]*sn];
            let o = [q[0]+r*n[0],q[1]+r*n[1],q[2]+r*n[2]];
            let a = Surface::Cylinder(Frame::new(o,axis,cross(axis,n)),r);
            let case = format!("radius {r} on {what}, turned {turn}°");
            match meet(&a,[[0.,TAU],[-3.,3.]],b,*db,1e-7) {
                Err(e) => failed.push(format!("{case}: {e}")),
                Ok(m) => {
                    let touches: Vec<V> = m.singular.iter().filter(|s| s.kind == Singular::Touch).map(|s| s.at).collect();
                    if !m.branches.is_empty() || touches.len() != 1 || distance(touches[0],q) > 1e-6 {
                        failed.push(format!("{case}: {} branches, touches {touches:?}, not one at {q:?}",m.branches.len()))
                    }
                }
            }
        } }
    }
    assert!(failed.is_empty(),"{}",failed.join("\n"));
}
