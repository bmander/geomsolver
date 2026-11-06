//! **A ring slid at a lean to its axis** (#805, #780 L1's X2 under a constant twist,
//! `brep::slid`, docs/swept-solid-design.md step 6): the torus's contact is where its normal is
//! square to the slide, two roots on each latitude that meet at the silhouette's top and bottom (the
//! branch locus on the face), read in the chart transverse to it — the angle round the axis — as
//! two smooth closed curves, `tan v = −(D_x cos u + D_y sin u)/D_z` and that turned by `π`. Each,
//! swept along the slide, is the cylinder over its apparent contour; the inner contour folds once
//! the slide leans `β` from the axis with `tan² β ≥ (R − r)/r`, and is refused there by name.
//!
//! Held apart from the kernel: the contact read here in angles (`contact`), the contours' areas by
//! the shoelace sum over them (exact to rounding for a smooth closed curve), and every line along
//! the slide meeting the ring once or not at all — so the swept volume, each line's chord lengthened
//! by `|D|`, is `V + |D| A`, `A` the annulus between the two contours; a Monte Carlo of the sweep's
//! definition, each point's line back along the slide read against the ring; and membership.
use gcs_core::brep::build::torus;
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::{closed,Unbuilt};
use gcs_core::brep::geom::Frame;
use gcs_core::brep::turned::Exact;
use crate::swept_closed::slide;
use gcs_core::space::{add,cross,dot,norm,scale,sub,unit};
use std::f64::consts::{PI,TAU};

type V = [f64;3];

/// A ring about the origin: its tube's middle radius `big` and the tube's `small`, and the frame it
/// is read in, its `z` the ring's axis.
struct Ring { big: f64,small: f64,frame: Frame }

impl Ring {
    /// The ring leaning `beta` from `z` toward `x`.
    fn leaning(big: f64,small: f64,beta: f64) -> Ring {
        let (s,c) = beta.sin_cos();
        Ring {big,small,frame:Frame {o:[0.;3],x:[c,0.,-s],y:[0.,1.,0.],z:[s,0.,c]}}
    }
    /// Below nought inside the solid ring.
    fn inside(&self,p: V) -> f64 { let [x,y,z] = self.frame.local(p); (x.hypot(y)-self.big).hypot(z)-self.small }
    /// **The contact** with the slide `d` (unit), apart from `brep::slid`: at the angle `u` round
    /// the axis, the angle `v` round the tube where the normal `(cos v ρ̂, sin v)` is square to `d`,
    /// `tan v = −g/d_z`, `g = d_x cos u + d_y sin u` — the outer root (`cos v > 0`) or the inner —
    /// and its rate in `u`.
    fn contact(&self,d: V,u: f64,outer: bool) -> (V,V) {
        let dl = self.frame.local(d);
        let (su,cu) = u.sin_cos();
        let (g,dg) = (dl[0]*cu+dl[1]*su,dl[1]*cu-dl[0]*su);
        let v = (-g/dl[2]).atan()+if outer { 0. } else { PI };
        let dv = -dg*dl[2]/(dl[2]*dl[2]+g*g);
        let (sv,cv) = v.sin_cos();
        let (reach,dreach) = (self.big+self.small*cv,-self.small*sv*dv);
        (self.frame.at([reach*cu,reach*su,self.small*sv]),self.frame.at([dreach*cu-reach*su,dreach*su+reach*cu,self.small*cv*dv]))
    }
    /// The area within its contact's apparent contour along `d`, outer or inner: `½ ∮ (x y′ − y x′)
    /// du` in a frame square to `d`, by the trapezoid rule over 512 angles (exact to rounding for a
    /// smooth closed curve).
    fn contour_area(&self,d: V,outer: bool) -> f64 {
        let (e1,e2) = gcs_core::space::across(d);
        (0..512).map(|k| {
            let (p,dp) = self.contact(d,TAU*k as f64/512.,outer);
            dot(p,e1)*dot(dp,e2)-dot(p,e2)*dot(dp,e1)
        }).sum::<f64>().abs()/2.*TAU/512.
    }
    /// The annulus between the two contours along `d`.
    fn annulus(&self,d: V) -> f64 { self.contour_area(d,true)-self.contour_area(d,false) }
    fn volume(&self) -> f64 { 2.*PI*PI*self.big*self.small*self.small }
    /// How many stretches the line through `p` along the unit `d` stands inside the ring in, across
    /// its reach: stepped by the ring's distance from the line's point (its exact signed distance,
    /// so no step passes the surface), never less than 1e-4.
    fn stretches(&self,p: V,d: V) -> usize {
        let reach = self.big+self.small+1.;
        let (mut t,mut inside,mut count) = (-reach,false,0);
        while t <= reach {
            let g = self.inside(add(p,scale(d,t)));
            if g < 0. && !inside { count += 1 }
            inside = g < 0.;
            t += g.abs().max(1e-4);
        }
        count
    }
}

/// **The sweep's definition, read**: `p` is in it where some point of its line back along the
/// slide, `p − t D` for `t` in `[0, 1]`, is in the ring — stepped by the ring's exact distance from
/// the line's point, never less than a ten-thousandth of `D`.
fn swept(ring: &Ring,d: V,p: V) -> bool {
    let len = norm(d);
    let mut t = 0.;
    while t <= 1. {
        let g = ring.inside(sub(p,scale(d,t)));
        if g <= 0. { return true }
        t += (g/len).max(1e-4);
    }
    ring.inside(sub(p,d)) <= 0.
}

/// **The contact read by the kernel is the contact** (`brep::slid`): each half's point is on the
/// ring, its normal there square to the slide, and it agrees with the contact read here in angles;
/// its derivatives are its points' differences; the contour is the contact less its part along the
/// slide.
#[test]
fn the_slid_contact_is_where_the_ring_grazes_the_slide() {
    use gcs_core::brep::slid::halves;
    let ring = Ring::leaning(3.,1.,35f64.to_radians());
    let d = unit([0.3,-0.2,1.]);
    for (side,outer) in [(1.,true),(-1.,false)] {
        for (h,c) in halves(ring.frame,3.,1.,d,side,false).iter().zip(halves(ring.frame,3.,1.,d,side,true)) {
            for k in 0..=20 {
                let s = -1.+k as f64/10.;
                let (x,dx,ddx) = h.d2(s);
                assert!(ring.inside(x).abs() <= 1e-12,"on the ring: {}",ring.inside(x));
                let l = ring.frame.local(x);
                let rho = l[0].hypot(l[1]);
                let n = ring.frame.at([l[0]/rho*(rho-3.),l[1]/rho*(rho-3.),l[2]]);
                assert!(dot(n,d).abs() <= 1e-12,"square to the slide: {}",dot(n,d));
                let u = l[1].atan2(l[0]);
                assert!(norm(sub(x,ring.contact(d,u,outer).0)) <= 1e-12,"the contact read in angles");
                let e = 1e-5;
                let fd = scale(sub(h.d2(s+e).0,h.d2(s-e).0),0.5/e);
                let fdd = scale(sub(h.d2(s+e).1,h.d2(s-e).1),0.5/e);
                assert!(norm(sub(fd,dx)) <= 1e-7*(1.+norm(dx)) && norm(sub(fdd,ddx)) <= 1e-6*(1.+norm(ddx)),"its jets");
                assert!(norm(sub(c.d2(s).0,sub(x,scale(d,dot(x,d))))) <= 1e-12,"its contour");
            }
        }
    }
}

/// **The fold locus is where the inner contour cusps, `tan² β = (R − r)/r`** (#805): for rings of
/// three proportions, a lean a degree short of it is proved regular all along (`Slid::folded` none,
/// and each line along the slide through the contact meets the ring there alone, `meets_again`),
/// and a degree past it is not; at the lean itself the contour's speed reaches nought, at
/// `u = ±π/2`. The outer contour never folds.
#[test]
fn the_inner_contour_folds_where_the_slide_runs_along_an_asymptote() {
    use gcs_core::brep::slid::halves;
    for (big,small) in [(3.,1.),(2.,1.),(4.,0.5)] {
        let fold = ((big-small)/small as f64).sqrt().atan();
        for (beta,folds) in [(fold-1f64.to_radians(),false),(fold+1f64.to_radians(),true)] {
            let ring = Ring::leaning(big,small,beta);
            let d = [0.,0.,1.];
            let inner = halves(ring.frame,big,small,d,-1.,true);
            assert_eq!(inner.iter().any(|h| h.folded().is_some()),folds,"R {big}, r {small}, β {:.2}°",beta.to_degrees());
            let outer = halves(ring.frame,big,small,d,1.,true);
            assert!(outer.iter().all(|h| h.folded().is_none() && h.meets_again().is_none()));
            if !folds { assert!(inner.iter().all(|h| h.meets_again().is_none())) }
        }
        // at the fold itself, the inner contour's speed is nought across the lean
        let ring = Ring::leaning(big,small,fold);
        let [h,_] = halves(ring.frame,big,small,[0.,0.,1.],-1.,true);
        // `u = π/2` about the half's middle, `s = tan(π/4) = 1`
        assert!(norm(h.d2(1.).1) <= 1e-6,"R {big}, r {small}: the contour's speed at its cusp {}",norm(h.d2(1.).1));
    }
}

/// **Every line along the slide meets the ring once or not at all, short of the fold** — what makes
/// the swept volume `V + |D| A` — read over a grid of lines across the ring's shadow, at the leans
/// the gates slide at.
#[test]
fn a_line_along_the_slide_meets_the_leaning_ring_at_most_once() {
    for beta in [20f64,35.,50.] {
        let ring = Ring::leaning(3.,1.,beta.to_radians());
        let d = [0.,0.,1.];
        for i in 0..=40 { for j in 0..=40 {
            let p = [-4.2+8.4*i as f64/40.,-4.2+8.4*j as f64/40.,0.];
            assert!(ring.stretches(p,d) <= 1,"β {beta}: the line through {p:?}");
        } }
    }
}

/// **A ring slid at a lean closes into its sweep** (#805's gate, built alone): a ring of middle
/// radius 3 and tube 1, its axis leaning 35° from the slide, slid 6, its two end poses apart. Its
/// sweep is the ring at both end poses and its two contacts swept into the cylinders over their
/// contours, met in closed form; to `V + |D| A` within 1e-9, to membership and to a Monte Carlo of
/// the definition.
#[test]
fn a_ring_slid_at_a_lean_closes_into_its_sweep() { slid_closes(35.,[0.,0.,6.]) }

/// **A ring slid at a lean past less than its own reach closes into its sweep**: the ring of
/// `a_ring_slid_at_a_lean_closes_into_its_sweep` slid 2, its two end poses overlapping, so each
/// crosses the other and the contours' cylinders join them across their overlap.
#[test]
fn a_ring_slid_at_a_lean_by_less_than_its_reach_closes_into_its_sweep() { slid_closes(35.,[0.,0.,2.]) }

/// **A ring slid aslant its lean closes into its sweep**: `a_ring_slid_at_a_lean_closes_into_its_sweep`'s
/// ring at 20° slid 6 along a way leaning off the plane of the lean too, so neither the slide nor
/// its contours keep any of the ring's planes of symmetry.
#[test]
fn a_ring_slid_aslant_its_lean_closes_into_its_sweep() {
    slid_closes(20.,scale(unit([0.3,0.25,1.]),6.));
}

/// The ring of middle radius 3 and tube 1 leaning `lean` degrees from `z`, slid by `d`, built alone
/// and held to `V + |D| A` within 1e-9, to membership at seeded points of its box
/// (`swept_closed::agrees`), and to the sweep's definition read at 20000 points (`swept`) within four
/// standard errors of its volume.
fn slid_closes(lean: f64,d: V) {
    let ring = Ring::leaning(3.,1.,lean.to_radians());
    let tool = torus([0.;3],ring.frame.z,3.,1.).unwrap();
    let built = closed(&tool,&slide(d),[0.,1.],0.002,1e-6).unwrap_or_else(|e| panic!("slid {d:?}: {e:?}"));
    let expected = ring.volume()+norm(d)*ring.annulus(unit(d));
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"slid {d:?}: {} against {expected}",volume(&built));
    let path = slide(d);
    crate::swept_closed::agrees(&built,&tool,&path);
    let (lo,hi) = built.bounds();
    let (sampled,error) = crate::swept_closed::monte_carlo(lo.map(|x| x-0.1),hi.map(|x| x+0.1),20000,805,&|p| swept(&ring,d,p));
    assert!((sampled-volume(&built)).abs() <= 4.*error,"sampled {sampled} ± {error} against {}",volume(&built));
}

/// What a ring of middle radius `big` and tube `small`, its axis leaning `beta` from upright, pushed
/// straight down until wholly in a block, its centre `depth` under the block's top, takes from it:
/// each line down the push meets the ring once or not at all, and its depth to the ring's far side,
/// by the ring's symmetry about its centre, averages the centre's over the annulus `A` between the
/// contours — `depth A` — and half the ring's volume, `π² R r²` (`solid_sweeps/leaning_ring.sv`'s).
pub(crate) fn plunged_away(big: f64,small: f64,beta: f64,depth: f64) -> f64 {
    let ring = Ring::leaning(big,small,beta);
    depth*ring.annulus([0.,0.,1.])+ring.volume()/2.
}

/// **A ring slid leaning past its fold is refused by name** (#805's negative control): at 60° from
/// the slide, past `tan² β = (R − r)/r` (54.7°), the inner contour cusps — the fold locus, where the
/// sheet doubles back, which no patch is cut at yet — so the build is refused as `Folded` at a point
/// of the inner contact, never built wrong; and the admission (`closed::reads`) refuses it alike.
#[test]
fn a_ring_slid_past_its_fold_is_refused_by_name() {
    use gcs_core::brep::swept::closed::reads;
    let ring = Ring::leaning(3.,1.,60f64.to_radians());
    let tool = torus([0.;3],ring.frame.z,3.,1.).unwrap();
    let path = slide([0.,0.,6.]);
    for refused in [closed(&tool,&path,[0.,1.],0.002,1e-6).map(|_| ()),reads(&tool,&path).map(|_| ())] {
        match refused {
            Err(Unbuilt::Folded {at}) => {
                assert!(ring.inside(at).abs() <= 1e-9,"a point of the ring: {at:?}");
                let l = ring.frame.local(at);
                assert!(l[0].hypot(l[1]) < 3.,"of its inner contact: {at:?}");
            }
            other => panic!("refused by name as folded, not {:?}",other.err()),
        }
    }
}


/// The cylinders a ring's contacts sweep along the slide `d` (unit): each half of each contour,
/// outer (`side` 1) and inner, extruded along `d` (`Surface::Extrusion` of `Curve::Slid`).
fn contour_cylinders(ring: &Ring,d: V,side: f64) -> Vec<gcs_core::brep::geom::Surface> {
    use gcs_core::brep::geom::{Curve,Surface};
    use std::sync::Arc;
    gcs_core::brep::slid::halves(ring.frame,ring.big,ring.small,d,side,true).iter()
        .map(|h| Surface::Extrusion(Frame::about([0.;3],d),Arc::new(Curve::Slid(Arc::new(*h))))).collect()
}

/// **A line meets a slid contour's cylinder where a scan finds it** (#805, `query::line_slid`): the
/// line seen along the slide meets the contour's half in the plane square to it, its roots isolated
/// with the contour's jets bounded in outward intervals. Each crossing read lies on the cylinder, and
/// as many are read as the projected line crosses the half's contour over the stretch it is read
/// over, the contour read at 4096 chords.
#[test]
fn a_line_meets_a_slid_contours_cylinder_where_a_scan_finds_it() {
    use gcs_core::brep::geom::Curve;
    use gcs_core::brep::query::{Meets,curve_surface};
    let ring = Ring::leaning(3.,1.,35f64.to_radians());
    let d = unit([0.2,-0.1,1.]);
    let flat = |x: V| sub(x,scale(d,dot(x,d)));
    let mut rng = gcs_core::rng::Rng::new(805);
    let mut met = 0;
    for side in [1.,-1.] {
        for s in contour_cylinders(&ring,d,side) {
            let gcs_core::brep::geom::Surface::Extrusion(_,c) = &s else { unreachable!() };
            let chords: Vec<V> = (0..=4096).map(|k| flat(c.point(-1.+2.*k as f64/4096.))).collect();
            for _ in 0..40 {
                let (p,way): (V,V) = ([rng.uniform(-5.,5.),rng.uniform(-5.,5.),rng.uniform(-3.,3.)],unit([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)]));
                let Meets::At(found) = curve_surface(&Curve::Line {p,d:way},[-20.,20.],&s,1e-9) else { panic!("a line along it") };
                for &(t,_) in &found { assert!(s.implicit(add(p,scale(way,t))).abs() <= 1e-9,"a crossing off the cylinder") }
                // the projected line's side of each chord's ends
                let (q,e) = (flat(p),flat(way));
                let n = cross(d,e);
                let side_of = |x: V| dot(sub(x,q),n);
                // (each crossing within the stretch the line is read over)
                let scanned = chords.windows(2).filter(|w| side_of(w[0])*side_of(w[1]) < 0.).filter(|w| {
                    let x = add(w[0],scale(sub(w[1],w[0]),side_of(w[0])/(side_of(w[0])-side_of(w[1]))));
                    dot(sub(x,q),e).abs()/dot(e,e) <= 20.
                }).count();
                assert_eq!(found.iter().filter(|r| !r.1).count(),scanned,"through {p:?} along {way:?}: {found:?}");
                met += found.len();
            }
        }
    }
    assert!(met > 40,"{met} crossings read");
}

/// **A slid contour's cylinder meets its neighbours in closed form** (#805, `ssi`'s `slid_meeting`):
/// the ring at a pose along the slide touches each half along its contact there — on both, their
/// normals one; a plane along the slide meets it in lines along the slide on it; a plane leaning
/// to the slide in the contour sheared onto that plane, on both; and the outer contour's cylinder
/// and the inner's meet nowhere.
#[test]
fn a_slid_contours_cylinder_meets_its_neighbours_in_closed_form() {
    use gcs_core::brep::geom::Surface;
    use gcs_core::brep::ssi::{intersect,Ssi};
    let ring = Ring::leaning(3.,1.,35f64.to_radians());
    let d = unit([0.2,-0.1,1.]);
    let posed = Surface::Torus(Frame {o:scale(d,2.5),..ring.frame},3.,1.);
    let along = Surface::Plane(Frame::new([0.5,0.2,0.],unit(cross(d,[1.,0.3,0.])),d));
    let leaning = Surface::Plane(Frame::new([0.,0.,0.7],unit([0.3,0.1,1.]),[1.,0.,0.]));
    let inner = contour_cylinders(&ring,d,-1.);
    for (k,s) in contour_cylinders(&ring,d,1.).iter().chain(&inner).enumerate() {
        for (what,other,tangent) in [("the posed ring",&posed,true),("a plane along the slide",&along,false),("a plane leaning to it",&leaning,false)] {
            let Ssi::Curves(curves) = intersect(s,other,1e-9) else { panic!("{what}: in closed form") };
            assert!(!curves.is_empty() || what == "a plane along the slide","{what}: met");
            for c in &curves {
                let span = match c { gcs_core::brep::geom::Curve::Line {..} => [-3.,3.],_ => [-1.,1.] };
                for j in 0..=16 {
                    let x = c.point(span[0]+(span[1]-span[0])*j as f64/16.);
                    assert!(s.implicit(x).abs() <= 1e-9 && other.implicit(x).abs() <= 1e-9,"{what}: on both at {x:?}");
                    if tangent {
                        let (n,m) = (unit(s.gradient(x)),unit(other.gradient(x)));
                        assert!(norm(cross(n,m)) <= 1e-7,"{what}: touching at {x:?}");
                    }
                }
            }
        }
        if k < 2 { for i in &inner { assert_eq!(intersect(s,i,1e-9),Ssi::Curves(vec![]),"the outer and the inner contours") } }
    }
}
