//! The kernel's offsets (`brep::offset`, docs/contract/brep.md "Offsets"): a solid grown or
//! shrunk by a distance, hollowed to a wall, a sheet thickened — each built exactly as the union of
//! the pieces its points are nearest (each face's normal sweep, each crease's tube or kite, each
//! corner's ball or parallelepiped), held to its closed form to 1e-9.  And the two Boolean and
//! mesher faults the brep-bench run found (#elliptic-hole, #horn-torus): a closed rational spline
//! cut square to its sweep, a torus whose chart pinches at the middle of its box.
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{cone,disc,hull,kept,prism,sphere,torus,Profile,ProfileEdge};
use gcs_core::brep::geom::{Frame,V};
use gcs_core::brep::nurbs::BSpline;
use gcs_core::brep::offset::{grown,normal_sweep,sheet_of,shelled,shrunk,thickened,Join,Side};
use gcs_core::brep::props::volume;
use gcs_core::brep::query::{Located,Place};
use gcs_core::brep::topo::Brep;
use std::f64::consts::{FRAC_1_SQRT_2,PI};
use std::sync::Arc;
use crate::brep::{block,mesh_volume,poly,total_area};
use gcs_core::space::{cross,sub};

fn close(what: &str,a: f64,b: f64) { assert!((a-b).abs() <= 1e-9*b.abs().max(1.),"{what}: {a} against {b} ({:e})",(a-b)/b); }

fn l_block() -> Brep {
    let section = Profile {names:vec![],origin:[0.;3],normal:[0.,0.,1.],
        loops:vec![poly(&[[0.,0.,0.],[20.,0.,0.],[20.,5.,0.],[5.,5.,0.],[5.,15.,0.],[0.,15.,0.]])]};
    prism(&section,0.,8.).unwrap()
}

/// The faces of `b` the points `on` lie inside, one each.
pub(crate) fn faces_through(b: &Brep,on: &[V]) -> Vec<usize> {
    let located = Located::new(b,1e-9*b.size());
    on.iter().map(|&p| (0..b.faces.len()).find(|&fi| {
        let q = b.faces[fi].surface.point(located.uv(fi,p));
        (0..3).all(|k| (q[k]-p[k]).abs() <= 1e-9) && located.face_place(fi,p) == Place::In
    }).unwrap_or_else(|| panic!("no face through {p:?}"))).collect()
}

/// Steiner's formula for the box `a × b × c` grown by `d`: its faces' slabs, its edges' quarter
/// cylinders, its corners' eighths of a ball.
fn steiner_box(a: f64,b: f64,c: f64,d: f64) -> f64 {
    a*b*c+2.*(a*b+b*c+c*a)*d+PI*(a+b+c)*d*d+4.*PI*d*d*d/3.
}

#[test]
fn a_box_grown_is_steiners_volume_and_sharp_its_moved_faces_box() {
    let b = block([0.,0.,0.],[10.,8.,6.]);
    let g = grown(&b,1.,Join::Round).unwrap();
    g.check(1e-7).unwrap();
    close("grown",volume(&g),steiner_box(10.,8.,6.,1.));
    close("its area",total_area(&g),2.*(80.+48.+60.)+2.*PI*24.+4.*PI);
    let s = grown(&b,1.,Join::Sharp).unwrap();
    close("grown sharp",volume(&s),12.*10.*8.);
    let k = shrunk(&b,1.,Join::Round).unwrap();
    close("shrunk",volume(&k),8.*6.*4.);
}

#[test]
fn an_l_grown_rounds_its_convex_edges_and_shrunk_its_concave_one() {
    let (a,p,t,d) = (150.,70.,8.,1.);
    let k = 5.*PI/4.-1.;
    close("grown",volume(&grown(&l_block(),d,Join::Round).unwrap()),
        t*(a+p*d+k*d*d)+2.*(a*d+PI*p*d*d/4.+2.*k*d*d*d/3.));
    let k = 5.-PI/4.;
    close("shrunk",volume(&shrunk(&l_block(),d,Join::Round).unwrap()),(t-2.*d)*(a-p*d+k*d*d));
    close("grown sharp",volume(&grown(&l_block(),d,Join::Sharp).unwrap()),(22.*7.+7.*10.)*10.);
}

/// The tetrahedron's corners' balls make one whole ball, its edges' cylinders turn through
/// π − acos(1/3) each.
#[test]
fn a_tetrahedron_grown_rounds_its_oblique_edges_and_corners() {
    let t = hull(&[[3.,3.,3.],[3.,-3.,-3.],[-3.,3.,-3.],[-3.,-3.,3.]]).unwrap();
    let e = 6.*2f64.sqrt();
    let want = 72.+72.*3f64.sqrt()+3.*e*(PI-(1f64/3.).acos())+4.*PI/3.;
    close("grown",volume(&grown(&t,1.,Join::Round).unwrap()),want);
}

/// Grown past its hole a torus is an apple: the tube's disc cut at the axis, turned.  Shrunk, the
/// tube narrows about the same middle circle.
#[test]
fn a_torus_grown_past_its_hole_is_an_apple_and_shrunk_a_thinner_torus() {
    let (big,s): (f64,f64) = (4.,5.);
    let c = (s*s-big*big).sqrt();
    let phi = (big/s).acos();
    let apple = grown(&torus([0.,0.,0.],[0.,0.,1.],big,2.).unwrap(),3.,Join::Round).unwrap();
    close("apple",volume(&apple),2.*PI*(PI*big*s*s+2.*c*c*c/3.-big*(s*s*phi-big*c)));
    let thin = shrunk(&torus([0.,0.,0.],[0.,0.,1.],10.,3.).unwrap(),1.5,Join::Round).unwrap();
    close("shrunk",volume(&thin),2.*PI*PI*10.*1.5*1.5);
}

/// A groove narrower than twice the growth is filled; a flange thinner than twice the shrinking
/// is taken away — the moved faces cross, and the union decides.
#[test]
fn a_groove_grown_shut_and_a_flange_shrunk_away() {
    let a: f64 = 0.8;
    let m = |s: f64| 20.*a-10.*(a*(1.-a*a).sqrt()+a.asin())+s*a*a*a/3.;
    let drum = prism(&disc([0.,0.,0.],[0.,0.,1.],10.),0.,10.).unwrap();
    let o = [0.,0.,4.2];
    let ring = Profile {origin:o,normal:[0.,0.,1.],names:Vec::new(),loops:vec![
        vec![ProfileEdge::Arc {frame:Frame::about(o,[0.,0.,1.]),r:12.,span:None}],
        vec![ProfileEdge::Arc {frame:Frame::about(o,[0.,0.,1.]),r:7.,span:None}]]};
    let grooved = combined(&drum,&prism(&ring,0.,1.6).unwrap(),Op::Cut,0.).unwrap();
    close("grooved, grown",volume(&grown(&grooved,1.,Join::Round).unwrap()),2.*PI*(705.+5.*PI+2./3.-m(1.)));
    let drum = prism(&disc([0.,0.,0.],[0.,0.,1.],8.),0.,6.).unwrap();
    let flange = prism(&disc([0.,0.,2.2],[0.,0.,1.],11.),0.,1.6).unwrap();
    let flanged = combined(&drum,&flange,Op::Union,0.).unwrap();
    let m = 16.*a-8.*(a*(1.-a*a).sqrt()+a.asin())-a*a*a/3.;
    close("flanged, shrunk",volume(&shrunk(&flanged,1.,Join::Round).unwrap()),PI*49.*4.+2.*PI*m);
}

/// A cone's apex grown is a cap of a ball: a corner of no crease.
#[test]
fn a_cone_grown_caps_its_apex_with_a_ball() {
    let (r,h,l,d) = (3.,4.,5.,1.);
    let beta = (4f64/3.).atan();
    let want = PI*r*r*h/3.+PI*(r*r+r*l)*d+PI*(h+r*(PI-beta))*d*d+4.*PI*d*d*d/3.;
    close("grown",volume(&grown(&cone([0.,0.,0.],[0.,0.,1.],r,0.,h).unwrap(),d,Join::Round).unwrap()),want);
}

/// The cylinder of radius `r` from `z = −h` up to the plane `z = x tan α` (brep-bench's
/// offset/cylinder-cut-at-thirty-degrees-grown and slant-cut-cylinder-grown).
fn cut_cylinder(r: f64,h: f64,alpha: f64) -> Brep {
    let cylinder = prism(&disc([0.,0.,-h],[0.,0.,1.],r),0.,2.*h).unwrap();
    kept(&cylinder,[0.,0.,0.],[alpha.sin(),0.,-alpha.cos()],0.).unwrap()
}

/// Steiner's formula for that convex body grown by `d`: its volume and area.  Its integrated mean
/// curvature is the side's `π h`, the bottom circle's right angle along it, `π² r / 2`, and half
/// the ellipse's turning angle along it, `J / 2`, with
/// `J = ∫ acos(−sin α cos φ) r √(1 + tan² α sin² φ) dφ` over a turn, which has no elementary form:
/// the trapezoid rule, exact to rounding for a smooth periodic integrand at this many points.
fn steiner_cut_cylinder(r: f64,h: f64,alpha: f64,d: f64) -> (f64,f64) {
    let n = 4096;
    let j = (0..n).map(|k| {
        let phi = 2.*PI*k as f64/n as f64;
        (-alpha.sin()*phi.cos()).acos()*r*(1.+(alpha.tan()*phi.sin()).powi(2)).sqrt()
    }).sum::<f64>()*2.*PI/n as f64;
    let (v0,s0) = (PI*r*r*h,2.*PI*r*h+PI*r*r+PI*r*r/alpha.cos());
    let m = PI*h+PI*PI*r/2.+j/2.;
    (v0+s0*d+m*d*d+4.*PI*d*d*d/3.,s0+2.*m*d+4.*PI*d*d)
}

/// A cylinder cut aslant has a face no parameter lines bound: its side meets the slanted top in an
/// ellipse.  The side's move along its normal is a stretch square to the axis, so its sweep is the
/// face carried between its moved copies, walled by the ruled sheet the ellipse sweeps along the
/// normals (exactly `60π (1 + d / 2r)` at d = 1); the top's is its face carried along its normal;
/// and the ellipse's own tube is the ball carried along it.  Grown, each is Steiner's formula.
/// Grown past the ellipse's least radius of curvature (`b² / a`, 2.12 at 45°) its tube would fold,
/// and the crease is joined by the sector of it its faces' normals span (`offset::sectors`), clear
/// of the fold; its side's wall, ruled along the normals from the ellipse, stands on the top's
/// plane along the ellipse and two rulings, which the plane meets in closed form
/// (`ssi::rail_in_plane`; traced, the ellipse's ends missed its vertices at 4).  Still Steiner's.
#[test]
fn a_cylinder_cut_aslant_grown_is_steiners() {
    let thirty = cut_cylinder(3.,10.,PI/6.);
    let side = faces_through(&thirty,&[[-3.,0.,-5.]])[0];
    close("the side swept",volume(&normal_sweep(&thirty,side,[0.,1.]).unwrap()),70.*PI);
    for (alpha,d) in [(PI/6.,1.),(PI/4.,2.),(PI/4.,2.5),(PI/4.,4.)] {
        let g = grown(&cut_cylinder(3.,10.,alpha),d,Join::Round).unwrap();
        g.check(1e-7).unwrap();
        let (v,a) = steiner_cut_cylinder(3.,10.,alpha,d);
        close(&format!("grown by {d} at {alpha}: its volume"),volume(&g),v);
        close(&format!("grown by {d} at {alpha}: its area"),total_area(&g),a);
    }
}

/// A round swept past its radius turns inside out: its piece stops at the centre of its section's
/// circle (a point past it is nearer the far side), so one offset step takes a round of any radius.
/// A puck of radius 6 and height 10, both rims rounded to 2, shrunk by 3 is the cylinder of
/// radius 3 from 3 to 7 (brep-bench's offset/rounded-puck-shrunk-past-its-rounds); a post on a
/// plate filleted at its foot to 2 and grown by 3 is the sharp post and plate grown by 3, the
/// fillet's moved face gone and the moved faces meeting in a sharp concave edge
/// (offset/filleted-post-grown-past-its-fillet); and a plate drilled through by a hole of radius 3
/// grown by 3, the rim's tube a horn torus touching the axis, fills the hole
/// (offset/drilled-plate-grown-by-its-hole).
#[test]
fn rounds_offset_past_their_radius_in_one_step() {
    use gcs_core::brep::fillet::roll;
    use gcs_core::brep::geom::tolerance;
    let up = [0.,0.,1.];
    let rounded = |part: &Brep,at: V,r: f64| {
        let round = roll(part,at,r,tolerance(part.size())).unwrap();
        combined(part,&round.piece,if round.concave { Op::Union } else { Op::Cut },0.).unwrap()
    };
    let cylinder = prism(&disc([0.;3],up,6.),0.,10.).unwrap();
    let puck = rounded(&rounded(&cylinder,[0.,6.,10.],2.),[0.,6.,0.],2.);
    let s = shrunk(&puck,3.,Join::Round).unwrap();
    s.check(1e-7).unwrap();
    close("the puck shrunk",volume(&s),36.*PI);
    let plate = prism(&disc([0.,0.,-4.],up,8.),0.,4.).unwrap();
    let post = prism(&disc([0.;3],up,3.),0.,8.).unwrap();
    let part = combined(&plate,&post,Op::Union,0.).unwrap();
    let g = grown(&rounded(&part,[0.,3.,0.],2.),3.,Join::Round).unwrap();
    g.check(1e-7).unwrap();
    let d = 3f64;
    let plate_grown = 256.*PI+192.*PI*d+(4.*PI+8.*PI*PI)*d*d+4.*PI*d*d*d/3.;
    let want = plate_grown+PI*(3.+d)*(3.+d)*(8.-d)+9.*PI*d+(PI*PI*d*d/2.)*(3.+4.*d/(3.*PI));
    close("the post grown",volume(&g),want);
    let drilled = combined(&block([-10.,-10.,0.],[10.,10.,4.]),&prism(&disc([0.,0.,-1.],up,3.),0.,6.).unwrap(),Op::Cut,0.).unwrap();
    let g = grown(&drilled,3.,Join::Round).unwrap();
    g.check(1e-7).unwrap();
    let (a,c,rho) = (20f64,4f64,3f64);
    let v0 = a*a*c+2.*(a*a+2.*a*c)*d+PI*(2.*a+c)*d*d+4.*PI*d*d*d/3.;
    close("the plate grown",volume(&g),v0-4.*PI*(d*rho*rho/2.-rho/2.*d*d*(rho/d).asin()+d*d*d/3.));
}

/// The ellipse x²/25 + y²/9 ≤ 1, a closed rational spline, swept up 10: its side, the spline
/// swept along a line (`Surface::Extrusion`), has no offset in closed form, but its normal stands
/// square to the sweep and alike all along each line of it, so its sweep is the face family of
/// the rectangle a line of it sweeps along its normal (`offset::extruded_sweep`), closed on itself
/// round the ellipse (`family::ring`).  Grown by 1 (Steiner's: the side's mean curvature `π h`
/// and the rims' right angles, `π P / 2`) and shrunk by 1 (the ellipse's inner parallel, its
/// area `π a b − P + π`, swept over 8), within its least radius of curvature 1.8; shrunk past it
/// the sweep would fold, and it is refused.
#[test]
fn an_elliptic_cylinder_grown_and_shrunk() {
    let at = |x: f64,y: f64| [5.*x,3.*y,0.];
    let poles = vec![at(1.,0.),at(1.,1.),at(0.,1.),at(-1.,1.),at(-1.,0.),at(-1.,-1.),at(0.,-1.),at(1.,-1.),at(1.,0.)];
    let h = FRAC_1_SQRT_2;
    let knots = vec![0.,0.,0.,0.25,0.25,0.5,0.5,0.75,0.75,1.,1.,1.];
    let rim = BSpline::rational(2,knots,poles,vec![1.,h,1.,h,1.,h,1.,h,1.]).unwrap();
    let section = Profile {origin:[0.;3],normal:[0.,0.,1.],loops:vec![vec![ProfileEdge::Spline(Arc::new(rim))]],names:Vec::new()};
    let solid = prism(&section,0.,10.).unwrap();
    // the ellipse's perimeter, by the trapezoid rule (exact to rounding for a periodic integrand)
    let n = 4096;
    let p = (0..n).map(|k| { let t = 2.*PI*k as f64/n as f64; (25.*t.sin().powi(2)+9.*t.cos().powi(2)).sqrt() }).sum::<f64>()*2.*PI/n as f64;
    let (v0,s0,m) = (150.*PI,30.*PI+10.*p,10.*PI+PI*p/2.);
    let g = grown(&solid,1.,Join::Round).unwrap();
    g.check(1e-7).unwrap();
    close("grown",volume(&g),v0+s0+m+4.*PI/3.);
    let s = shrunk(&solid,1.,Join::Round).unwrap();
    s.check(1e-7).unwrap();
    close("shrunk",volume(&s),(15.*PI-p+PI)*8.);
    let e = shrunk(&solid,2.5,Join::Round).unwrap_err();
    assert!(e.contains("folds"),"{e}");
}

/// A ball drawn as one rational biquadratic net with a pole at each end, its four patches spline
/// faces, has no offset in closed form as a spline — but the net lies on a sphere, which its
/// equation vanishing on a grid of each span proves (`offset::sphere_of`), so each face is put on
/// the sphere, its edges its own, and grown as a ball's: the ball of radius 7.  A net on no
/// sphere is left a spline, and refused (`a_spline_sheet_is_not_thickened`).
#[test]
fn a_spline_ball_grown_is_a_ball() {
    use gcs_core::brep::build::grid_sheet;
    use gcs_core::brep::nurbs::Net;
    let circle = [[1.,0.],[1.,1.],[0.,1.],[-1.,1.],[-1.,0.],[-1.,-1.],[0.,-1.],[1.,-1.],[1.,0.]];
    let meridian = [[0.,-5.],[5.,-5.],[5.,0.],[5.,5.],[0.,5.]];
    let (cw,mw) = ([1.,FRAC_1_SQRT_2,1.,FRAC_1_SQRT_2,1.,FRAC_1_SQRT_2,1.,FRAC_1_SQRT_2,1.],[1.,FRAC_1_SQRT_2,1.,FRAC_1_SQRT_2,1.]);
    let net = Net {du:2,dv:2,uknots:vec![0.,0.,0.,0.25,0.25,0.5,0.5,0.75,0.75,1.,1.,1.],vknots:vec![0.,0.,0.,0.5,0.5,1.,1.,1.],
        poles:circle.iter().map(|c| meridian.iter().map(|m| [m[0]*c[0],m[0]*c[1],m[1]]).collect()).collect(),
        weights:Some(cw.iter().map(|a| mw.iter().map(|b| a*b).collect()).collect())};
    let ball = grid_sheet(&net,&[0.,0.5,1.],&[0.,0.5,1.],&["east".into(),"west".into()],[true,false]).unwrap();
    let g = grown(&ball,2.,Join::Round).unwrap();
    g.check(1e-7).unwrap();
    close("grown",volume(&g),4.*PI*343./3.);
}

/// A ball cut by a plane leaning off its axis: the circle the plane cuts is no parallel of the
/// sphere's, and its face's sweep is the face scaled about the centre, walled by a cone's stretch.
#[test]
fn a_ball_cut_aslant_grown_is_steiners() {
    let (r,h,d) = (5f64,2f64,1f64);
    let n = { let n = [0.6f64.sin(),0.3,0.6f64.cos()]; let l = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt(); n.map(|x| x/l) };
    let cut = kept(&sphere([0.;3],[0.,0.,1.],r).unwrap(),n.map(|x| x*h),n.map(|x| -x),0.).unwrap();
    let g = grown(&cut,d,Join::Round).unwrap();
    g.check(1e-7).unwrap();
    // the ball less its cap past the plane, the circle of radius ρ turning by acos(h / r)
    let (c,rho) = (r-h,(r*r-h*h).sqrt());
    let (v0,s0) = (4.*PI*r*r*r/3.-PI*c*c*(3.*r-c)/3.,4.*PI*r*r-2.*PI*r*c+PI*rho*rho);
    let m = 4.*PI*r-2.*PI*c+(h/r).acos()*PI*rho;
    close("grown",volume(&g),v0+s0*d+m*d*d+4.*PI*d*d*d/3.);
    close("its area",total_area(&g),s0+2.*m*d+4.*PI*d*d);
}

/// The frustum `ρ = R₀ + m z` over `0 ≤ z ≤ 6` (`m = −1/3`, from 3 down to 1), cut by the plane
/// `z = 3 + x tan β` and kept below it.
pub(crate) fn cut_cone(beta: f64) -> Brep {
    let frustum = cone([0.;3],[0.,0.,1.],3.,1.,6.).unwrap();
    kept(&frustum,[0.,0.,3.],[beta.sin(),0.,-beta.cos()],0.).unwrap()
}

/// The trapezoid rule over a turn, exact to rounding for a smooth periodic integrand at this many
/// points.
fn turn(f: impl Fn(f64) -> f64) -> f64 {
    let n = 4096;
    (0..n).map(|k| f(2.*PI*k as f64/n as f64)).sum::<f64>()*2.*PI/n as f64
}

/// Steiner's formula for `cut_cone(β)` grown by `d`: its volume and area.  Over the polar angle
/// `φ`, the generator meets the plane at height `z_t(φ) = (c + R₀ T cos φ)/(1 − m T cos φ)`
/// (`T = tan β`), radius `ρ_t = R₀ + m z_t`.  Its volume by the divergence of `(0, 0, z)`: the
/// side's `−m (R₀ z_t²/2 + m z_t³/3)` and the top's `c ρ_t²/2 + T cos φ ρ_t³/3`, each over a turn;
/// its area the base's, the side's `√(1 + m²) (R₀ z_t + m z_t²/2)` and the top's ellipse's; its
/// integrated mean curvature the side's `z_t / 2` (its curvature `cos α / ρ`), the base circle's
/// angle between its faces' normals, `acos(m / √(1 + m²))`, along half of it, and the ellipse's
/// angle `acos(n_side · n_top)` along half of it — a convex body's.
fn steiner_cut_cone(beta: f64,d: f64) -> (f64,f64) {
    let (r0,m,c,t) = (3f64,-1f64/3.,3f64,beta.tan());
    let w = (1.+m*m).sqrt();
    let zt = |p: f64| (c+r0*t*p.cos())/(1.-m*t*p.cos());
    let dzt = |p: f64| {
        let (n,dn,dd) = (c+r0*t*p.cos(),-r0*t*p.sin(),m*t*p.sin());
        let den = 1.-m*t*p.cos();
        (dn*den-n*dd)/(den*den)
    };
    let v0 = turn(|p| { let (z,rho) = (zt(p),r0+m*zt(p)); -m*(r0*z*z/2.+m*z*z*z/3.)+c*rho*rho/2.+t*p.cos()*rho.powi(3)/3. });
    let s0 = PI*r0*r0+turn(|p| { let (z,rho) = (zt(p),r0+m*zt(p)); w*(r0*z+m*z*z/2.)+rho*rho/2./beta.cos() });
    let top = [-beta.sin(),0.,beta.cos()];
    let crease = turn(|p| {
        let (z,dz) = (zt(p),dzt(p));
        let (rho,drho) = (r0+m*z,m*dz);
        let e = [drho*p.cos()-rho*p.sin(),drho*p.sin()+rho*p.cos(),dz];
        let n = [p.cos()/w,p.sin()/w,-m/w];
        (n[0]*top[0]+n[1]*top[1]+n[2]*top[2]).acos()*(e[0]*e[0]+e[1]*e[1]+e[2]*e[2]).sqrt()
    });
    let mean = turn(|p| zt(p)/2.)+PI*r0*(m/w).acos()+crease/2.;
    (v0+s0*d+mean*d*d+4.*PI*d*d*d/3.,s0+2.*mean*d+4.*PI*d*d)
}

/// A frustum cut aslant: its side meets the slanted top in an ellipse no parameter line of the
/// cone is, and a cone's move along its normal is no map of space, so its side's sweep is the face
/// on its cone moved along the normal with its parameters kept (`Surface::normal_moved`, the
/// cone's origin stood back along its axis), its edges carried along the normal
/// (`Curve::Parallel`), walled by the ruled sheet of the normals along the ellipse.  Grown, it is
/// Steiner's formula, a convex body's.
#[test]
fn a_cone_cut_aslant_grown_is_steiners() {
    for (beta,d) in [(PI/6.,1.),(PI/5.,0.5)] {
        let cut = cut_cone(beta);
        close(&format!("cut at {beta}: its volume"),volume(&cut),steiner_cut_cone(beta,0.).0);
        let g = grown(&cut,d,Join::Round).unwrap();
        g.check(1e-7).unwrap();
        let (v,a) = steiner_cut_cone(beta,d);
        close(&format!("grown by {d} at {beta}: its volume"),volume(&g),v);
        close(&format!("grown by {d} at {beta}: its area"),total_area(&g),a);
    }
}

/// The torus about `z` of middle radius 5 and tube `r` kept below the plane `z = c + x tan β`,
/// which cuts every meridian circle (a ring, two spiric curves round it).
pub(crate) fn cut_torus(r: f64,beta: f64,c: f64) -> Brep {
    kept(&torus([0.;3],[0.,0.,1.],5.,r).unwrap(),[0.,0.,c],[beta.sin(),0.,-beta.cos()],0.).unwrap()
}

/// The ring `cut_torus(r, β, c)`'s volume, area and integrated mean curvature, in closed form over
/// its meridians.  At `u` the line `z = c + ρ k` (`k = T cos u`) leaves below it the arc
/// `v − φ ∈ [π − asin s, 2π + asin s]` of the meridian `ρ = R + r cos v`, `z = r sin v`
/// (`φ = atan k`, `s = (R k + c)/(r √(1 + k²))`).  Its volume by the divergence of `(0, 0, z)`:
/// the torus's `r² (R (v/2 − sin 2v/4) + r sin³ v/3)` across the arc and the plane's between its
/// two curves, `c (ρ_o² − ρ_i²)/2 + T cos u (ρ_o³ − ρ_i³)/3`; its area the torus's
/// `r (R v + r sin v)` across the arc and the plane's `(ρ_o² − ρ_i²)/2 / cos β`; its mean curvature
/// the torus's `(R v + 2 r sin v)/2` across the arc and, along each curve, half the angle between
/// its faces' normals, `|E'| = √(ρ² + r² v'²)`.
fn cut_torus_measures(r: f64,beta: f64,c: f64) -> (f64,f64,f64) {
    let (big,t) = (5f64,beta.tan());
    let arc = |u: f64| -> ([f64;2],[f64;2]) {
        let (k,dk) = (t*u.cos(),-t*u.sin());
        let q = 1.+k*k;
        let s = (big*k+c)/(r*q.sqrt());
        assert!(s.abs() < 1.,"the plane misses the meridian at {u}");
        let ds = dk*(big-c*k)/(r*q.powf(1.5))/(1.-s*s).sqrt();
        let (phi,dphi) = (k.atan(),dk/q);
        ([phi+PI-s.asin(),phi+2.*PI+s.asin()],[dphi-ds,dphi+ds])
    };
    let across = |g: &dyn Fn(f64) -> f64,u: f64| { let [a,b] = arc(u).0; g(b)-g(a) };
    let rho = |v: f64| big+r*v.cos();
    let plane = |u: f64| { let [i,o] = arc(u).0.map(rho); (i,o) };
    let v0 = turn(|u| across(&|v| r*r*(big*(v/2.-(2.*v).sin()/4.)+r*v.sin().powi(3)/3.),u))
        +turn(|u| { let (i,o) = plane(u); c*(o*o-i*i)/2.+t*u.cos()*(o.powi(3)-i.powi(3))/3. });
    let s0 = turn(|u| across(&|v| r*(big*v+r*v.sin()),u))+turn(|u| { let (i,o) = plane(u); (o*o-i*i)/2./beta.cos() });
    let top = [-beta.sin(),0.,beta.cos()];
    let crease = |u: f64,v: f64,dv: f64| {
        let n = [v.cos()*u.cos(),v.cos()*u.sin(),v.sin()];
        (n[0]*top[0]+n[1]*top[1]+n[2]*top[2]).acos()*(rho(v)*rho(v)+r*r*dv*dv).sqrt()
    };
    let mean = turn(|u| across(&|v| (big*v+2.*r*v.sin())/2.,u))
        +turn(|u| { let (v,dv) = arc(u); (crease(u,v[0],dv[0])+crease(u,v[1],dv[1]))/2. });
    (v0,s0,mean)
}

/// A torus cut aslant by a plane through every meridian is a ring bounded by two spiric curves,
/// no parameter line of the torus: its face's sweep is the face on the torus of the tube grown or
/// shrunk, its edges carried along the normal (`Curve::Parallel`), walled by the ruled sheets of
/// the normals along the curves.  Grown by less than its reach it is Steiner's formula, the ring's
/// `V + S d + M d²` (no cubic term: its Euler characteristic is nothing); shrunk, the ring of the
/// tube shrunk under the plane moved in, the eroded meet being the meet of the eroded sets.
#[test]
fn a_torus_cut_aslant_grown_and_shrunk() {
    let (r,beta,c) = (2.,12f64.to_radians(),0.4);
    let ring = cut_torus(r,beta,c);
    let (v0,s0,m) = cut_torus_measures(r,beta,c);
    close("the ring's volume",volume(&ring),v0);
    let d = 1.;
    let g = grown(&ring,d,Join::Round).unwrap();
    g.check(1e-7).unwrap();
    close("grown: its volume",volume(&g),v0+s0*d+m*d*d);
    close("grown: its area",total_area(&g),s0+2.*m*d);
    let d = 0.5;
    let s = shrunk(&ring,d,Join::Round).unwrap();
    s.check(1e-7).unwrap();
    close("shrunk: its volume",volume(&s),cut_torus_measures(r-d,beta,c-d/beta.cos()).0);
}

/// The parabolic trough's profile, `z = x²/8` over `|x| ≤ 4` as a quadratic Bézier closed by the
/// line `z = 2`, swept 10 along `y`, cut by the plane `y = 6 + 0.3 x + 0.2 z` and kept short of it:
/// its curved side a spline swept along a line (`Surface::Extrusion`), bounded at the cut by no
/// parameter line of it.
pub(crate) fn cut_trough() -> Brep {
    let parabola = BSpline::new(2,vec![0.,0.,0.,1.,1.,1.],vec![[-4.,0.,2.],[0.,0.,-2.],[4.,0.,2.]]).unwrap();
    let section = Profile {origin:[0.;3],normal:[0.,1.,0.],names:Vec::new(),
        loops:vec![vec![ProfileEdge::Spline(Arc::new(parabola)),ProfileEdge::Line {a:[4.,0.,2.],b:[-4.,0.,2.]}]]};
    let trough = prism(&section,0.,10.).unwrap();
    let n = [0.3f64,-1.,0.2];
    let l = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
    kept(&trough,[0.,6.,0.],n.map(|x| x/l),0.).unwrap()
}

/// A spline swept along a line, cut aslant so its boundary is no parameter line of it: its offset
/// keeps its parameters (`Surface::normal_moved`, the sweep read as its sheet), so the face lifted
/// out as a sheet and thickened by 1 toward its focus is the face moved on the sheet's exact
/// offset, its edges carried along the normal, walled by the normals along them — its volume
/// `∫ (t − t² κ / 2) dA`, the sheet's curvature `κ = (1/4) / (1 + x²/16)^{3/2}` across it and none
/// along it, `dA = √(1 + x²/16) (6 + 0.3 x + 0.2 x²/8) dx`, by Gauss's rule, independently of the
/// solid.  Thickened past its least radius of curvature, 4 at the vertex, it folds and is refused.
#[test]
fn a_spline_extrusion_cut_aslant_is_thickened() {
    let cut = cut_trough();
    let side = sheet_of(&cut,&faces_through(&cut,&[[0.,1.,0.]])).unwrap();
    assert_eq!(side.faces[0].surface.kind(),"extrusion");
    let th = 1.;
    let t = thickened(&side,toward_focus(&side,th),Join::Round).unwrap();
    t.check(1e-7).unwrap();
    let panels = 16;
    let h = 4./panels as f64;
    let want: f64 = (0..panels).flat_map(|k| gcs_core::integral::GAUSS8.map(|(x,w)| (-4.+h*(2*k+1) as f64+h*x,h*w))).map(|(x,w)| {
        let g = 1.+x*x/16.;
        w*(th-th*th*0.25/g.powf(1.5)/2.)*g.sqrt()*(6.+0.3*x+0.2*x*x/8.)
    }).sum();
    close("volume",volume(&t),want);
    let e = thickened(&side,toward_focus(&side,5.),Join::Round).unwrap_err();
    assert!(e.contains("folds"),"{e}");
}

#[test]
fn a_box_shelled_open_at_its_top_inward_outward_and_sharp() {
    let b = block([0.,0.,0.],[10.,8.,6.]);
    let top = faces_through(&b,&[[5.,4.,6.]]);
    close("inward",volume(&shelled(&b,&top,1.,Side::In,Join::Round).unwrap()),480.-8.*6.*5.);
    close("closed",volume(&shelled(&b,&[],1.,Side::In,Join::Round).unwrap()),480.-8.*6.*4.);
    close("walls meeting",volume(&shelled(&b,&top,4.,Side::In,Join::Round).unwrap()),480.);
    close("outward, sharp",volume(&shelled(&b,&top,1.,Side::Out,Join::Sharp).unwrap()),12.*10.*7.-480.);
    // outward: the box grown, cut off at the top's plane, less the box
    close("outward",volume(&shelled(&b,&top,1.,Side::Out,Join::Round).unwrap()),296.+47.*PI/3.);
}

/// The L of brep-bench's shell cases: (0, 0), (20, 0), (20, 20), (15, 20), (15, 5), (0, 5), 20 deep;
/// and its step, the face y = 5.
fn stepped_l() -> (Brep,Vec<usize>) {
    let section = Profile {names:vec![],origin:[0.;3],normal:[0.,0.,1.],
        loops:vec![poly(&[[0.,0.,0.],[20.,0.,0.],[20.,20.,0.],[15.,20.,0.],[15.,5.,0.],[0.,5.,0.]])]};
    let l = prism(&section,0.,20.).unwrap();
    let step = faces_through(&l,&[[7.5,5.,10.]]);
    (l,step)
}

/// An outward shell ends where its opening is, not across the opening's whole plane (brep-bench's
/// shell/l-block-open-at-its-step-outward-sharp): an L open at its step keeps the tall leg's walls
/// past the step's plane, and the riser's wall stands on the opening's edge.  Cut off at the whole
/// plane, the leg's walls went with it (904 for 1868).
#[test]
fn an_l_shelled_outward_open_at_its_step_keeps_the_walls_past_its_plane() {
    let (l,step) = stepped_l();
    // sharp: the moved faces' outline (−1, −1), (21, −1), (21, 21), (14, 21), (14, 5), (−1, 5),
    // 244 in area, from z = −1 to 21, less the L
    let sharp = shelled(&l,&step,1.,Side::Out,Join::Sharp).unwrap();
    close("sharp",volume(&sharp),22.*244.-3500.);
    close("sharp's area",total_area(&sharp),1670.+2094.+50.);
    // rounded: its walls' slabs (65 round the sides, 20 deep; the L's 175 above and below), the
    // quarter cylinders along the 65 of convex edges round the top and the bottom, the four convex
    // corners' quarter cylinders up the sides and eighths of a ball top and bottom; what the
    // step's own rounds keep on the L's side of its plane lies within these
    let round = shelled(&l,&step,1.,Side::Out,Join::Round).unwrap();
    close("round",volume(&round),1650.+323.*PI/6.);
}

/// A block whose top slopes, open at it: its walls meet the opening aslant.  Rounded, the wall that
/// runs past the plane is cut off at it and the one that falls short of it is met by the top's own
/// round: the block grown by 1 kept behind the top's plane, less the block.  Sharp, each moved wall
/// would have to be carried on to the plane, and is refused.
#[test]
fn a_shell_meeting_its_opening_aslant_is_cut_off_rounded_and_refused_sharp() {
    // the section in x and z: (0, 0), (10, 0), (10, 6), (0, 4), 8 deep along y
    let section = Profile {names:vec![],origin:[0.;3],normal:[0.,1.,0.],
        loops:vec![poly(&[[0.,0.,0.],[0.,0.,4.],[10.,0.,6.],[10.,0.,0.]])]};
    let b = prism(&section,0.,8.).unwrap();
    let top = faces_through(&b,&[[5.,4.,5.]]);
    // the section's area kept behind the top's line, grown by r: the section, the slabs of its
    // other three sides (the left one less what runs past the line), the bottom corners' quarter
    // discs and the right top corner's sector up to the line, atan 0.2 wide: 50 + 20r + c r²;
    // along y the slices, 8 of them at r = 1 and the ends' at r = √(1 − y²)
    let c = PI/2.+0.2f64.atan()/2.-0.1;
    let want = 8.*(70.+c)+2.*(50.+20.*PI/4.+c*2./3.)-8.*50.;
    close("round",volume(&shelled(&b,&top,1.,Side::Out,Join::Round).unwrap()),want);
    let refused = shelled(&b,&top,1.,Side::Out,Join::Sharp).unwrap_err();
    assert!(refused.contains("aslant"),"{refused}");
}

/// A rounded plate's corners swept in past their axis: what lies past it is its neighbours'.
#[test]
fn a_rounded_tray_shelled_past_its_rounds() {
    use std::f64::consts::FRAC_PI_2;
    let (lo,hi,r) = ([0.,0.],[20.,14.],2.);
    let corners = [([hi[0]-r,lo[1]+r],-FRAC_PI_2),([hi[0]-r,hi[1]-r],0.),([lo[0]+r,hi[1]-r],FRAC_PI_2),([lo[0]+r,lo[1]+r],PI)];
    let mut edges = Vec::new();
    for k in 0..4 {
        let (c,a) = corners[k];
        edges.push(ProfileEdge::Arc {frame:Frame::new([c[0],c[1],0.],[0.,0.,1.],[1.,0.,0.]),r,span:Some([a,a+FRAC_PI_2])});
        let (n,b) = corners[(k+1)%4];
        edges.push(ProfileEdge::Line {a:[c[0]+r*(a+FRAC_PI_2).cos(),c[1]+r*(a+FRAC_PI_2).sin(),0.],b:[n[0]+r*b.cos(),n[1]+r*b.sin(),0.]});
    }
    let plate = prism(&Profile {origin:[0.;3],normal:[0.,0.,1.],loops:vec![edges],names:Vec::new()},0.,10.).unwrap();
    let top = faces_through(&plate,&[[10.,7.,10.]]);
    close("rounds consumed",volume(&shelled(&plate,&top,3.,Side::In,Join::Round).unwrap()),1856.+40.*PI);
}

/// A moved wall runs on across a face left open until it meets it: inside a ball's cap the cavity
/// is the inner ball, which the opening never reaches.
#[test]
fn a_wall_runs_on_to_its_opening() {
    let cap = kept(&sphere([0.,0.,0.],[0.,0.,1.],5.).unwrap(),[0.,0.,4.5],[0.,0.,-1.],0.).unwrap();
    let flat = faces_through(&cap,&[[0.5,0.5,4.5]]);
    close("sealed",volume(&shelled(&cap,&flat,1.,Side::In,Join::Round).unwrap()),641.*PI/8.);
}

/// Sheets lifted off solids, thickened: an open box out of its cube with rounded and sharp
/// edges, and a bend thickened past its own radius, whose moved bend turns inside out.
#[test]
fn sheets_thickened() {
    let cube = block([0.,0.,0.],[10.,10.,10.]);
    let walls = faces_through(&cube,&[[5.,5.,0.],[0.,5.,5.],[10.,5.,5.],[5.,0.,5.],[5.,10.,5.]]);
    let tray = sheet_of(&cube,&walls).unwrap();
    close("rounded",volume(&thickened(&tray,[0.,1.],Join::Round).unwrap()),5.*100.+8.*PI*10./4.+4.*PI/6.);
    close("sharp",volume(&thickened(&tray,[0.,1.],Join::Sharp).unwrap()),12.*12.*11.-1000.);
    close("inward",volume(&thickened(&tray,[-1.,0.],Join::Round).unwrap()),1000.-8.*8.*9.);
    let bend = Frame::new([0.,0.,2.],[0.,1.,0.],[1.,0.,0.]);
    let section = Profile {origin:[0.;3],normal:[0.,1.,0.],names:Vec::new(),loops:vec![vec![
        ProfileEdge::Line {a:[-10.,0.,0.],b:[0.,0.,0.]},
        ProfileEdge::Arc {frame:bend,r:2.,span:Some([0.,std::f64::consts::FRAC_PI_2])},
        ProfileEdge::Line {a:[2.,0.,2.],b:[2.,0.,12.]},ProfileEdge::Line {a:[2.,0.,12.],b:[-10.,0.,12.]},
        ProfileEdge::Line {a:[-10.,0.,12.],b:[-10.,0.,0.]}]]};
    let solid = prism(&section,0.,8.).unwrap();
    let flange = sheet_of(&solid,&faces_through(&solid,&[[-5.,4.,0.],[2f64.sqrt(),4.,2.-2f64.sqrt()],[2.,4.,7.]])).unwrap();
    let (l,r,t,w) = (10.,2.,3.,8.);
    close("past its bend",volume(&thickened(&flange,[-t,0.],Join::Round).unwrap()),w*(2.*l*t-(t-r)*(t-r)+PI*r*r/4.));
    let hemi = kept(&sphere([0.,0.,0.],[0.,0.,1.],5.).unwrap(),[0.,0.,0.],[0.,0.,1.],0.).unwrap();
    let dome = sheet_of(&hemi,&faces_through(&hemi,&[[3.,0.,4.]])).unwrap();
    close("to its centre",volume(&thickened(&dome,[-5.,0.],Join::Round).unwrap()),2.*PI*125./3.);
}

/// `[0, h]` or `[−h, 0]`: a sheet's single face thickened by `h` toward the side `+z` its middle
/// faces, the focus of each sheet here.
fn toward_focus(sheet: &Brep,h: f64) -> [f64;2] {
    let f = &sheet.faces[0];
    if (f.surface.normal([0.5,0.5]).unwrap()[2] > 0.) != f.reversed { [0.,h] } else { [-h,0.] }
}

/// The paraboloid `z = (x² + y²)/8` over the square `|x|, |y| ≤ a`, a biquadratic Bézier sheet
/// (exact: each middle pole where the parabola's end tangents meet, `−a²/8`), thickened by `h`
/// toward its focus.
pub(crate) fn thickened_dish(a: f64,h: f64) -> Brep {
    use gcs_core::brep::nurbs::Net;
    let z = a*a/8.;
    let xs = [(-a,z),(0.,-z),(a,z)];
    let net = Net {du:2,dv:2,uknots:vec![0.,0.,0.,1.,1.,1.],vknots:vec![0.,0.,0.,1.,1.,1.],weights:None,
        poles:xs.iter().map(|&(x,zx)| xs.iter().map(|&(y,zy)| [x,y,zx+zy]).collect()).collect()};
    let dish = gcs_core::brep::build::sheet(net).unwrap();
    thickened(&dish,toward_focus(&dish,h),Join::Round).unwrap()
}

/// A spline sheet with no offset in closed form, the parabolic trough (brep-bench's
/// thicken/parabolic-trough: `z = x²/8` as a quadratic Bézier swept 10 along `y`), thickened by 1
/// toward its focus on its exact offset (`Surface::Offset`), not a fitted one: its moved face on
/// the sheet's offset, its edges the sheet's carried along its normal (`Curve::Parallel`), its ends
/// the plane strips between the parabola and its offset curve, its sides the normals' strips. Its
/// volume `h (t L − t² θ / 2)`, `L` the parabola's length and `θ` the turn of its tangent, and its
/// area the four faces' closed forms; thickened past its least radius of curvature, 4 at the
/// vertex, the sweep folds, and it is refused.
#[test]
fn a_spline_sheet_is_thickened_on_its_exact_offset() {
    use gcs_core::brep::nurbs::Net;
    let net = Net {du:2,dv:1,uknots:vec![0.,0.,0.,1.,1.,1.],vknots:vec![0.,0.,1.,1.],weights:None,
        poles:vec![vec![[-4.,0.,2.],[-4.,10.,2.]],vec![[0.,0.,-2.],[0.,10.,-2.]],vec![[4.,0.,2.],[4.,10.,2.]]]};
    let trough = gcs_core::brep::build::sheet(net).unwrap();
    let t = thickened(&trough,toward_focus(&trough,1.),Join::Round).unwrap();
    t.check(1e-7).unwrap();
    let mut kinds: Vec<&str> = t.faces.iter().map(|f| f.surface.kind()).collect();
    kinds.sort();
    assert_eq!(kinds,["bspline","offset","plane","plane","plane","plane"]);
    let (l,theta) = (4.*2f64.sqrt()+4.*1f64.asinh(),PI/2.);
    close("volume",volume(&t),10.*(l-theta/2.));
    close("area",total_area(&t),10.*(2.*l-theta)+2.*(l-theta/2.)+20.);
    let m = gcs_core::brep::mesh::mesh(&t,1e-3,0.1).unwrap();
    assert!((mesh_volume(&m)-volume(&t)).abs() < 1e-2*volume(&t),"meshed volume {}",mesh_volume(&m));
    let e = thickened(&trough,toward_focus(&trough,5.),Join::Round).unwrap_err();
    assert!(e.contains("folds"),"{e}");
}

/// A sheet curved both ways, the paraboloid `z = (x² + y²)/8` over the square `|x|, |y| ≤ 3` (a
/// biquadratic Bézier, exact), thickened by 1 toward its focus: its sides are no planes, each the
/// ruled wall of the sheet's normals along a parabola (a `Blend` between the edge and its
/// carried copy), and its volume is Steiner's, `∫ (t − t² (κ₁ + κ₂)/2 + t³ κ₁κ₂/3) dA` over the
/// sheet — here integrated by Gauss's rule over the square, independently of the solid.
#[test]
fn a_sheet_curved_both_ways_is_thickened_to_steiners_volume() {
    let (a,th) = (3f64,1f64);
    let t = thickened_dish(a,th);
    t.check(1e-7).unwrap();
    let mut kinds: Vec<&str> = t.faces.iter().map(|f| f.surface.kind()).collect();
    kinds.sort();
    assert_eq!(kinds,["blend","blend","blend","blend","bspline","offset"]);
    // Gauss–Legendre on 16 panels of 8 points each way
    let panels = 16;
    let h = a/panels as f64;
    let nodes: Vec<(f64,f64)> = (0..panels).flat_map(|k| gcs_core::integral::GAUSS8.map(|(x,w)| (-a+h*(2*k+1) as f64+h*x,h*w))).collect();
    let mut steiner = 0.;
    for &(x,wx) in &nodes { for &(y,wy) in &nodes {
        // `z = f(x, y)`: `f_x = x/4`, `f_xx = f_yy = 1/4`, `f_xy = 0`
        let (fx,fy) = (x/4.,y/4.);
        let w = (1.+fx*fx+fy*fy).sqrt();
        let mean2 = 0.25*((1.+fy*fy)+(1.+fx*fx))/w.powi(3);
        let gauss = 1./16./w.powi(4);
        steiner += wx*wy*w*(th-th*th*mean2/2.+th.powi(3)*gauss/3.);
    } }
    close("volume",volume(&t),steiner);
}

/// A plane cutting a rational spline's sweep square to it meets it in the spline carried along
/// the sweep — its weights kept — and that meeting, closed on one point with no other on it, is
/// one edge round: an elliptic bore cut through a plate, which was the plate untouched.
#[test]
fn an_elliptic_bore_cuts_through_its_plate() {
    let plate = block([-12.,-8.,0.],[12.,8.,8.]);
    let at = |x: f64,y: f64| [6.*x,3.*y,-1.];
    let poles = vec![at(1.,0.),at(1.,1.),at(0.,1.),at(-1.,1.),at(-1.,0.),at(-1.,-1.),at(0.,-1.),at(1.,-1.),at(1.,0.)];
    let h = FRAC_1_SQRT_2;
    let knots = vec![0.,0.,0.,0.25,0.25,0.5,0.5,0.75,0.75,1.,1.,1.];
    let rim = BSpline::rational(2,knots,poles,vec![1.,h,1.,h,1.,h,1.,h,1.]).unwrap();
    let section = Profile {origin:[0.,0.,-1.],normal:[0.,0.,1.],loops:vec![vec![ProfileEdge::Spline(Arc::new(rim))]],names:Vec::new()};
    let bored = combined(&plate,&prism(&section,0.,10.).unwrap(),Op::Cut,0.).unwrap();
    bored.check(1e-7).unwrap();
    close("bored",volume(&bored),3072.-144.*PI);
}

/// A horn torus's chart pinches at the middle of its box, where every `u` is one point: its
/// parameters scaled by their speed there would shrink `u` to nothing everywhere, and its mesh was
/// slivers across the whole turn (an area of 6.3e6 against 987).  Meshed, its facets bound its
/// volume and cover its area within the sag's share.
#[test]
fn a_horn_torus_meshes_within_its_sag() {
    let centre = [5.,0.,0.];
    let section = Profile {origin:centre,normal:[0.,-1.,0.],names:Vec::new(),
        loops:vec![vec![ProfileEdge::Arc {frame:Frame::new(centre,[0.,-1.,0.],[1.,0.,0.]),r:5.,span:None}]]};
    let horn = gcs_core::brep::build::revolve(&section,[0.;3],[0.,0.,1.],std::f64::consts::TAU).unwrap();
    let m = gcs_core::brep::mesh::mesh(&horn,1e-2,0.1).unwrap();
    let a: f64 = m.tris.iter().map(|t| {
        let [p,q,r] = t.map(|i| m.pts[i as usize]);
        0.5*gcs_core::space::norm(cross(sub(q,p),sub(r,p)))
    }).sum();
    let v = mesh_volume(&m);
    assert!((a-100.*PI*PI).abs() < 1e-2*100.*PI*PI,"meshed area {a} against {}",100.*PI*PI);
    assert!((v-250.*PI*PI).abs() < 1e-2*250.*PI*PI,"meshed volume {v} against {}",250.*PI*PI);
}

/// Two cylinders tangent inside at one point (brep-bench's tangent/tangent-tee-union): their
/// meeting is two closed fitted splines through that point, each with more points on it, and
/// each is split as a closed curve — its last stretch running round to its first — not dropped
/// past its last point, which left one use of the main's seam unmatched.
#[test]
fn a_tee_tangent_at_a_point_unites() {
    let main = prism(&gcs_core::brep::build::disc([-15.,0.,0.],[1.,0.,0.],5.),0.,30.).unwrap();
    let branch = prism(&gcs_core::brep::build::disc([0.,-15.,2.],[0.,1.,0.],3.),0.,30.).unwrap();
    let tee = combined(&main,&branch,Op::Union,0.).unwrap();
    tee.check(1e-7).unwrap();
    let l = (6f64.sqrt()/2.).asinh();
    assert!((volume(&tee)-(1020.*PI-96.*15f64.sqrt()+128.*l)).abs() < 1e-6*2964.68,"{}",volume(&tee));
}

/// Edges chamfered one after another, each run carried on past its ends until its section is
/// wholly past one cut there (brep-bench's chamfer/cube-all-edges-mitred and the octahedron's):
/// where two chamfers before it mitre at its end, each point of its section beyond one of the two
/// was taken for past, and the cube kept 13/30 of a corner too much at each of its last four edges
/// (809.73 against 808).  A leg is measured from the other face's plane: on the octahedron, the
/// line z = 2 of the face x + y + z = 10 stands 4/√3 off its neighbour's.
#[test]
fn edges_chamfered_one_after_another_mitre_at_their_corners() {
    use gcs_core::brep::fillet::flat_along;
    use gcs_core::brep::geom::tolerance;
    let chamfered = |mut b: Brep,edges: &[V],leg: f64| -> Brep {
        for &at in edges {
            for (i,f) in b.faces.iter_mut().enumerate() { f.name = format!("face {i}"); }
            let flat = flat_along(&b,at,[leg,leg],None,tolerance(b.size())).unwrap();
            b = combined(&b,&flat,Op::Cut,0.).unwrap();
        }
        b
    };
    let mut cube_edges = Vec::new();
    for axis in 0..3 { for a in [0.,10.] { for c in [0.,10.] {
        let mut at = [0.;3];
        (at[axis],at[(axis+1)%3],at[(axis+2)%3]) = (5.,a,c);
        cube_edges.push(at);
    } } }
    let cube = chamfered(block([0.;3],[10.;3]),&cube_edges,2.);
    cube.check(1e-7).unwrap();
    assert_eq!(cube.faces.len(),18);
    close("cube",volume(&cube),1000.-6.*10.*4.+6.*8.);
    let octahedron = hull(&[[10.,0.,0.],[-10.,0.,0.],[0.,10.,0.],[0.,-10.,0.],[0.,0.,10.],[0.,0.,-10.]]).unwrap();
    let mut edges = Vec::new();
    for (i,j) in [(0,1),(1,2),(2,0)] { for si in [-1.,1.] { for sj in [-1.,1.] {
        let mut at = [0.;3];
        (at[i],at[j]) = (si*5.,sj*5.);
        edges.push(at);
    } } }
    let octahedron = chamfered(octahedron,&edges,4./3f64.sqrt());
    assert_eq!(octahedron.faces.len(),20);
    close("octahedron",volume(&octahedron),4000./3.-8.*(52.-8.));
}
