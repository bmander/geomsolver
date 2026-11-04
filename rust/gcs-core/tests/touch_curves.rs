//! **Touch curves found generally** (#778, #777 M3): a meeting's critical set classified by rank
//! (T1), its ridges traced and the touches along them returned as branches (T2), the crossings
//! beside a ridge built however shallow (T3), the Boolean reading them where its march finds two
//! surfaces nearly touching (T4), and each closed form for a touch held to the general method with
//! the closed forms bypassed (T5). Held on brep-bench's vault (a block under a rational barrel
//! vault of radius 15, crown z = 16, cut at and a hair below its crown and by planes through the
//! crown tilted), as a prism of the arc and as a rational B-spline sheet, each to its closed form.
use gcs_core::brep::build::{kept,lofted,prism,Profile,ProfileEdge};
use gcs_core::brep::geom::{Curve,Frame,Surface,V};
use gcs_core::brep::meeting::{critical_set,meet_with,Contact};
use gcs_core::brep::nurbs::{BSpline,Net};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::topo::Brep;
use gcs_core::space::{cross,distance,dot,norm,unit};
use std::f64::consts::TAU;
use std::sync::Arc;

const R: f64 = 15.;

/// The vault's arc in the plane `x`: the rational quadratic through (y, z) = (−12, 10), (0, 26),
/// (12, 10), weights 1, 3/5, 1 — a circle of radius 15 about (y, z) = (0, 1).
fn arch(x: f64) -> BSpline {
    BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![[x,-12.,10.],[x,0.,26.],[x,12.,10.]],vec![1.,0.6,1.]).unwrap()
}

/// The block under the vault, as the case's implementation builds it: the section at x = −12 (its
/// floor, its sides and the arc) swept to x = 12.
fn vault_prism() -> Brep {
    let at = |y: f64,z: f64| [-12.,y,z];
    let section = Profile {origin:at(0.,0.),normal:[1.,0.,0.],names:Vec::new(),loops:vec![vec![
        ProfileEdge::Line {a:at(-12.,0.),b:at(12.,0.)},ProfileEdge::Line {a:at(12.,0.),b:at(12.,10.)},
        ProfileEdge::Spline(Arc::new(arch(-12.))),ProfileEdge::Line {a:at(-12.,10.),b:at(-12.,0.)}]]};
    prism(&section,0.,24.).unwrap()
}

/// The block under the vault with its top the rational B-spline **sheet** the case describes,
/// degree 1 along x and 2 along y (`build::lofted`, each face a net running round the section and
/// along x).
fn vault_sheet() -> Brep {
    let clamped = |d: usize| [vec![0.;d+1],vec![1.;d+1]].concat();
    let line = |rows: [[V;2];2]| Net {du:1,dv:1,uknots:clamped(1),vknots:clamped(1),poles:rows.iter().map(|r| r.to_vec()).collect(),weights:None};
    let p = |x: f64,y: f64,z: f64| [x,y,z];
    let top = Net {du:2,dv:1,uknots:clamped(2),vknots:clamped(1),
        poles:vec![vec![p(-12.,12.,10.),p(12.,12.,10.)],vec![p(-12.,0.,26.),p(12.,0.,26.)],vec![p(-12.,-12.,10.),p(12.,-12.,10.)]],
        weights:Some(vec![vec![1.,1.],vec![0.6,0.6],vec![1.,1.]])};
    let nets = vec![
        line([[p(-12.,-12.,0.),p(12.,-12.,0.)],[p(-12.,12.,0.),p(12.,12.,0.)]]),
        line([[p(-12.,12.,0.),p(12.,12.,0.)],[p(-12.,12.,10.),p(12.,12.,10.)]]),
        top,
        line([[p(-12.,-12.,10.),p(12.,-12.,10.)],[p(-12.,-12.,0.),p(12.,-12.,0.)]]),
    ];
    lofted([-12.,0.,0.],[1.,0.,0.],[0.,24.],vec![nets]).unwrap()
}

/// The vault block's volume: `24 ∫₋₁₂¹² (1 + √(225 − y²)) dy = 3168 + 5400 asin(4/5)`.
fn whole() -> f64 { 3168.+5400.*(0.8f64).asin() }

/// The circular segment of the vault's section a plane `δ` below the crown cuts off:
/// `r² acos((r − δ)/r) − (r − δ) √(2rδ − δ²)`.
fn seg(delta: f64) -> f64 { R*R*((R-delta)/R).acos()-(R-delta)*(2.*R*delta-delta*delta).max(0.).sqrt() }

/// **The vault's crown on the plane z = 16 reads as a critical curve** (#778 T1's gate): the
/// plane's distance over the vault's parameters, and the vault's over the plane's, is critical
/// along the crown with a Hessian of rank 1, its null way along x — for the vault as an extrusion
/// of the arc and as the rational sheet. Two vaults crossed (#715) still read rank 2 at their crown.
#[test]
fn a_vaults_crown_on_its_tangent_plane_reads_as_a_critical_curve() {
    let plane = Surface::Plane(Frame::about([0.,0.,16.],[0.,0.,1.]));
    let extrusion = Surface::Extrusion(Frame::new([-12.,0.,0.],[1.,0.,0.],[0.,1.,0.]),Arc::new(Curve::BSpline(Arc::new(arch(-12.)))));
    let sheet = vault_sheet().faces.iter().find(|f| matches!(&f.surface,Surface::BSpline(_,n) if n.is_rational())).unwrap().surface.clone();
    let along_x = |s: &Surface,uv: [f64;2],e: [f64;2]| {
        let (_,su,sv) = s.d1(uv);
        let k = [norm(su),norm(sv)];
        let w = [e[0]/k[0],e[1]/k[1]];
        unit([su[0]*w[0]+sv[0]*w[1],su[1]*w[0]+sv[1]*w[1],su[2]*w[0]+sv[2]*w[1]])[0].abs()
    };
    for (what,vault,d) in [("the extrusion",&extrusion,[[0.,1.],[0.,24.]]),("the sheet",&sheet,[[0.,1.],[0.,1.]])] {
        let found = critical_set(vault,d,&plane,1e-7);
        let crown: Vec<_> = found.iter().filter(|c| c.value.abs() <= 1e-9).collect();
        assert!(!crown.is_empty(),"{what}: no critical point on the crown among {found:?}");
        for c in &crown {
            assert_eq!(c.rank,1,"{what}: {c:?}");
            assert!(along_x(vault,c.uv,c.e) > 0.999,"{what}: the null way {:?} is not along x",c.e);
        }
        // and from the plane's side
        let other = critical_set(&plane,[[-12.,12.],[-12.,12.]],vault,1e-7);
        assert!(other.iter().any(|c| c.value.abs() <= 1e-9 && c.rank == 1),"{what}, from the plane: {other:?}");
    }
    // two vaults crossed, each along its own axis, crown on crown: rank 2 at the crown
    let across = Surface::Extrusion(Frame::new([0.,-12.,0.],[0.,1.,0.],[-1.,0.,0.]),Arc::new(Curve::BSpline(Arc::new(
        BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![[12.,-12.,10.],[0.,-12.,26.],[-12.,-12.,10.]],vec![1.,0.6,1.]).unwrap()))));
    let found = critical_set(&extrusion,[[0.,1.],[0.,24.]],&across,1e-7);
    assert!(found.iter().any(|c| c.value.abs() <= 1e-9 && c.rank == 2),"two vaults crossed: {found:?}");
}

/// The vault `block` with everything past the plane through `o` along `out` cut away.
fn cut(block: &Brep,o: V,out: V) -> Result<Brep,String> { kept(block,o,[-out[0],-out[1],-out[2]],0.) }

/// **The vault cut at and a hair below its crown is the vault less its segment** (#778 T2, T3's
/// gate): the block, as a prism of the arc and as the rational sheet, less everything above z = 16 −
/// δ, for δ from 0 (the plane on the crown, touching it all along) to a tenth — the plane crossing
/// the vault at under a degree and more. Each is `V₀ − 24·seg(δ)`, the closed forms apart from the
/// kernel, to 1e-9; each valid.
#[test]
fn a_vault_cut_at_and_below_its_crown_is_the_vault_less_its_segment() {
    for (what,block) in [("the prism",vault_prism()),("the sheet",vault_sheet())] {
        assert!((volume(&block)-whole()).abs() <= 1e-9*whole(),"{what}: {} against {}",volume(&block),whole());
        for delta in [0.,1e-9,1e-7,1e-5,1e-3,1e-2,1e-1] {
            let kept = cut(&block,[0.,0.,16.-delta],[0.,0.,1.]).unwrap_or_else(|e| panic!("{what} at δ {delta:e}: {e}"));
            kept.check(1e-6).unwrap_or_else(|e| panic!("{what} at δ {delta:e}: {}",e.msg));
            let expected = whole()-24.*seg(delta);
            assert!((volume(&kept)-expected).abs() <= 1e-9*whole(),"{what} at δ {delta:e}: {} against {expected}",volume(&kept));
        }
    }
}

/// **The vault cut by a plane through its crown, tilted, is the vault less the wedge it cuts**
/// (#778 T3's gate): the plane through (0, 0, 16) leaning `α` about y, for α of 0.001°, 0.1° and 1°,
/// touches the crown where it crosses it and crosses the vault beside it ever more steeply. Apart
/// from the kernel, at each x the plane stands `−x tan α` below the crown (above it for x > 0), so
/// the cut takes `∫₋₁₂⁰ seg(−x tan α) dx`, by Gauss–Legendre. To 1e-9, both vaults.
#[test]
fn a_vault_cut_by_a_tilted_plane_through_its_crown_is_the_vault_less_its_wedge() {
    let nodes = [(-0.906179845938664,0.236926885056189),(-0.538469310105683,0.478628670499366),(0.,0.568888888888889),
        (0.538469310105683,0.478628670499366),(0.906179845938664,0.236926885056189)];
    let mut failed: Vec<String> = Vec::new();
    for (what,block) in [("the prism",vault_prism()),("the sheet",vault_sheet())] {
        for degrees in [0.001f64,0.1,1.] {
            let a = degrees.to_radians();
            // the plane's normal leant from z toward x: it stands below the crown where x < 0
            let out = [a.sin(),0.,a.cos()];
            let kept = match cut(&block,[0.,0.,16.],out) { Ok(k) => k,Err(e) => { failed.push(format!("{what} at {degrees}°: {e}")); continue } };
            if let Err(e) = kept.check(1e-6) { failed.push(format!("{what} at {degrees}°: {}",e.msg)); continue }
            let panels = 400;
            let mut wedge = 0.;
            for k in 0..panels {
                let (x0,x1) = (-12.+12.*k as f64/panels as f64,-12.+12.*(k+1) as f64/panels as f64);
                for (t,w) in nodes { let x = 0.5*(x0+x1)+0.5*(x1-x0)*t; wedge += 0.5*(x1-x0)*w*seg(-x*a.tan()) }
            }
            let expected = whole()-wedge;
            if (volume(&kept)-expected).abs() > 1e-9*whole() { failed.push(format!("{what} at {degrees}°: {} against {expected}",volume(&kept))) }
        }
    }
    assert!(failed.is_empty(),"{}",failed.join("\n"));
}

/// **A fitted sheet touching a plane along a wavy curve keeps whole below it** (#778 T2's gate: a
/// touch curve no closed form knows): the block under `z = 16 − (y − 0.2 sin x)² / 30`, fitted as a
/// bicubic sheet (`nurbs::fit_net`), cut by the plane z = 16, which touches it along `y = 0.2 sin x`.
/// Kept below the plane, the block is whole: its volume and its area to 1e-9.
#[test]
fn a_fitted_sheet_touching_a_plane_along_a_wavy_curve_keeps_whole_below_it() {
    let (x0,x1,y0,y1) = (-6.,6.,-4.,4.);
    let top = |u: f64,v: f64| { let (y,x) = (y1+(y0-y1)*u,x0+(x1-x0)*v); let w = y-0.2*x.sin(); [x,y,16.-w*w/30.] };
    let (net,_) = gcs_core::brep::nurbs::fit_net(&top,3,3,16,16,1e-10).unwrap();
    // the floor and the two sides from the sheet's own boundary rows, its x and y linear in its poles
    let row = |i: usize| net.poles[i].clone();
    let flat = |r: &[V]| r.iter().map(|p| [p[0],p[1],0.]).collect::<Vec<V>>();
    let clamped1 = vec![0.,0.,1.,1.];
    let side = |a: Vec<V>,b: Vec<V>| Net {du:1,dv:net.dv,uknots:clamped1.clone(),vknots:net.vknots.clone(),poles:vec![a,b],weights:None};
    let last = net.poles.len()-1;
    let nets = vec![side(flat(&row(last)),flat(&row(0))),side(flat(&row(0)),row(0)),net.clone(),side(row(last),flat(&row(last)))];
    let block = lofted([x0,0.,0.],[1.,0.,0.],[0.,x1-x0],vec![nets]).unwrap();
    let kept = cut(&block,[0.,0.,16.],[0.,0.,1.]).unwrap();
    kept.check(1e-6).unwrap_or_else(|e| panic!("{}",e.msg));
    let (v,a) = (volume(&block),block.faces.iter().map(|f| area(&block,f)).sum::<f64>());
    let (kv,ka) = (volume(&kept),kept.faces.iter().map(|f| area(&kept,f)).sum::<f64>());
    assert!((kv-v).abs() <= 1e-9*v,"volume {kv} against {v}");
    assert!((ka-a).abs() <= 1e-9*a,"area {ka} against {a}");
}

/// **Each closed form for a touch is held to the general method** (#778 T5): the meeting met with
/// the closed forms bypassed (`meeting::meet_with`, an argument, not a switch) finds, from the
/// critical set alone, a touch branch along the closed form's — two cones on parallel axes
/// (`parallel_touch`), a ball in a torus's tube (`ball_in_tube`), a cylinder round a torus's
/// equator (`tube_along_torus`), two tori round a meridian (`tori_touching`), a ball at a pipe's
/// end (`pipe_end`), a screwed rim's sheet and the cylinder of its farthest reach
/// (`screwed_circle_round`) and a plane on a cylinder's side (`plane_cylinder`'s touch) — every
/// point of it within 1e-5 of the closed form's touch: a touch is a double root, its general
/// reading `∂f/∂n = 0` by differences, so it is placed only to their rounding (a ball at a pipe's
/// end, on the sheet's rim, within 2e-5), which is why each closed form stays as the exact road; and
/// each is measured, and said.
#[test]
fn each_closed_form_for_a_touch_is_held_to_the_general_method() {
    use gcs_core::brep::ssi::{intersect,Ssi};
    let z = [0.,0.,1.];
    let about = |o: V| Frame::about(o,z);
    let lean = 0.2f64.atan();
    let rim = gcs_core::brep::build::screwed(&gcs_core::brep::build::disc([3.,0.,1.],z,1.),[0.;3],z,1.5,2./TAU).unwrap();
    let rim_sheet = rim.faces.iter().find(|f| f.surface.kind() == "blend").unwrap().surface.clone();
    let n = unit([0.,10.,0.5]);
    let pipe = gcs_core::brep::build::screwed(&gcs_core::brep::build::disc([10.,0.,0.],n,1.),[0.;3],z,1.5,0.75).unwrap();
    let pipe_sheet = pipe.faces.iter().find(|f| f.surface.kind() == "blend").unwrap().surface.clone();
    let full = [0.,TAU];
    let cases: Vec<(&str,Surface,[[f64;2];2],Surface,[[f64;2];2])> = vec![
        ("two cones on parallel axes",Surface::Cone(about([0.;3]),1.,lean),[full,[-2.,2.]],Surface::Cone(about([3.,0.,0.]),2.,-lean),[full,[-2.,2.]]),
        ("a ball in a torus's tube",Surface::Torus(about([0.;3]),2.,0.3),[full,full],Surface::Sphere(about([2.,0.,0.]),0.3),[full,[-1.5,1.5]]),
        ("a cylinder round a torus's equator",Surface::Cylinder(about([0.;3]),2.3),[full,[-1.,1.]],Surface::Torus(about([0.;3]),2.,0.3),[full,full]),
        ("two tori round a meridian",Surface::Torus(about([10.,0.,0.]),1.,0.3),[full,full],Surface::Torus(about([0.;3]),11.,0.3),[full,full]),
        ("a ball at a pipe's end",Surface::Sphere(Frame::about([10.,0.,0.],n),1.),[full,[-1.5,1.5]],pipe_sheet,[[0.,1.],[0.,1.]]),
        ("a screwed rim and its farthest cylinder",Surface::Cylinder(about([0.;3]),4.),[full,[0.,3.]],rim_sheet,[[0.,1.],[0.,1.]]),
        ("a plane on a cylinder's side",Surface::Cylinder(about([0.;3]),1.),[full,[-2.,2.]],Surface::Plane(Frame::about([1.,0.,0.],[1.,0.,0.])),[[-3.,3.],[-3.,3.]]),
    ];
    let tol = 1e-7;
    for (what,a,da,b,db) in cases {
        let clock = std::time::Instant::now();
        let Ssi::Curves(curves) = intersect(&a,&b,tol) else { panic!("{what}: no closed form") };
        let closed_form = clock.elapsed();
        let parallel = |p: V| norm(cross(unit(a.gradient(p)),unit(b.gradient(p))));
        let on = |c: &Curve| (1..16).map(|k| c.point(match c { Curve::Line {..} => -1.+k as f64/8.,Curve::Iso(..) => k as f64/16.,_ => k as f64*TAU/16. }))
            .all(|p| parallel(p) <= 1e-5 && a.implicit(p).abs() <= 1e-6);
        let touch = curves.iter().find(|c| on(c)).unwrap_or_else(|| panic!("{what}: no touch among the closed form's"));
        let clock = std::time::Instant::now();
        let general = meet_with(&a,da,&b,db,tol,false).unwrap_or_else(|e| panic!("{what}: the general method: {e}"));
        let searched = clock.elapsed();
        let touches: Vec<_> = general.branches.iter().filter(|br| br.contact == Contact::Touching).collect();
        assert!(!touches.is_empty(),"{what}: the general method found no touch among {} branches",general.branches.len());
        let mut read = 0;
        for br in touches {
            let Curve::BSpline(s) = &br.curve else { continue };
            let [t0,t1] = s.domain();
            for k in 0..=32 {
                let p = br.curve.point(t0+(t1-t0)*k as f64/32.);
                let off = distance(touch.point(touch.inverse(p)),p);
                // (on a sheet's rim, the one case read through a swept sheet's own parameters there, 2e-5)
                let within = if what.contains("pipe") { 2e-5 } else { 1e-5 };
                assert!(off <= within,"{what}: the general touch at {p:?} stands {off:e} off the closed form's");
                read += 1;
            }
        }
        assert!(read > 0,"{what}");
        eprintln!("{what}: in closed form {closed_form:?}, by the general method {searched:?}");
        let _ = dot;
    }
}

/// **The Boolean reads a touch and a shallow crossing through each of its roads** (#778 T4's
/// gate): the vault, as the prism and as the sheet, at its crown, a ten-thousandth and a tenth
/// below, and under a plane through its crown tilted a tenth of a degree, cut through
/// `boolean::combined` (less a block above the plane), `build::kept` (kept below it) and
/// `boolean::split` (parted by the plane's square, the piece below kept): each valid, the three
/// alike in volume to 1e-9, and the faces each gives as stated: six at the crown, eight below it,
/// seven tilted.
#[test]
fn the_boolean_reads_touches_and_shallow_crossings_through_each_road() {
    use gcs_core::brep::boolean::{combined,split,Op};
    use gcs_core::brep::build::square;
    let mut counts: Vec<String> = Vec::new();
    for (what,block) in [("the prism",vault_prism()),("the sheet",vault_sheet())] {
        for (case,o,out) in [("at the crown",[0.,0.,16.],[0.,0.,1.]),("1e-4 below",[0.,0.,16.-1e-4],[0.,0.,1.]),
            ("0.1 below",[0.,0.,15.9],[0.,0.,1.]),("tilted 0.1°",[0.,0.,16.],[0.1f64.to_radians().sin(),0.,0.1f64.to_radians().cos()])] {
            let by_kept = cut(&block,o,out).unwrap_or_else(|e| panic!("{what} {case}, kept: {e}"));
            let above = prism(&square(o,out,40.),0.,40.).unwrap();
            let by_combined = combined(&block,&above,Op::Cut,0.).unwrap_or_else(|e| panic!("{what} {case}, combined: {}",e.msg));
            let sheet = { let mut s = prism(&square(o,out,40.),0.,1.).unwrap(); let base = s.faces.iter().position(|f| {
                let n = f.outward(o).unwrap_or([0.;3]); f.surface.implicit(o).abs() < 1e-9 && dot(n,out) < 0. }).unwrap();
                let face = s.faces[base].clone(); s.faces = vec![face]; s.compacted() };
            let pieces = split(&block,&sheet,1e-7).unwrap_or_else(|e| panic!("{what} {case}, split: {e}"));
            // the piece below: the one with the block's floor
            let below = pieces.iter().find(|p| p.bounds().0[2] < 1.).unwrap_or_else(|| panic!("{what} {case}: no piece below"));
            for (road,b) in [("kept",&by_kept),("combined",&by_combined),("split",below)] {
                b.check(1e-6).unwrap_or_else(|e| panic!("{what} {case}, {road}: {}",e.msg));
                assert!((volume(b)-volume(&by_kept)).abs() <= 1e-9*whole(),"{what} {case}, {road}: {} against kept's {}",volume(b),volume(&by_kept));
            }
            // its face counts: at the crown the block's own six (the touch laid and healed out of
            // them), cut below eight (the vault parted in two and the plane's face), tilted seven
            let faces = match case { "at the crown" => 6,"tilted 0.1°" => 7,_ => 8 };
            for (road,b) in [("kept",&by_kept),("combined",&by_combined),("split",below)] {
                if b.faces.len() != faces { counts.push(format!("{what} {case}, {road}: {} faces, not {faces}",b.faces.len())) }
            }
        }
    }
    assert!(counts.is_empty(),"{}",counts.join("\n"));
}
