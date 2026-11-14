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

/// **The sweep's definition, read**: `p` is in a tool slid by `d` where some point of its line back
/// along the slide, `p − t D` for `t` in `[0, 1]`, is in the tool — stepped by `apart`, at most the
/// distance from the tool (at or below nought in it), never less than a ten-thousandth of `D`.
fn swept(apart: &dyn Fn(V) -> f64,d: V,p: V) -> bool {
    let len = norm(d);
    let mut t = 0.;
    while t <= 1. {
        let g = apart(sub(p,scale(d,t)));
        if g <= 0. { return true }
        t += (g/len).max(1e-4);
    }
    apart(sub(p,d)) <= 0.
}

/// `∫ f` over `[lo, hi]`, by Gauss–Legendre's eight points (`integral::GAUSS8`) on each of `n`
/// pieces.
pub(crate) fn gauss(f: &dyn Fn(f64) -> f64,lo: f64,hi: f64,n: usize) -> f64 {
    (0..n).map(|k| {
        let (p,q) = (lo+(hi-lo)*k as f64/n as f64,lo+(hi-lo)*(k+1) as f64/n as f64);
        gcs_core::integral::GAUSS8.iter().map(|&(x,w)| w*0.5*(q-p)*f(0.5*(p+q)+0.5*(q-p)*x)).sum::<f64>()
    }).sum()
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
    let (sampled,error) = crate::swept_closed::monte_carlo(lo.map(|x| x-0.1),hi.map(|x| x+0.1),20000,805,&|p| swept(&|x| ring.inside(x),d,p));
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

/// **A ring slid leaning past its fold is swept by the contact's road** (#880 T1; #805's negative
/// control before it): at 60° from the slide, past `tan² β = (R − r)/r` (54.7°), the inner contour
/// cusps — the fold locus, where the sheet doubles back — which the closed class's admission still
/// refuses by name as `Folded` at a point of the inner contact (`closed::reads`); its build asks
/// the contact's road, whose trim keeps of the folded sheet what bounds the sweep. Held to the
/// sweep's volume line by line (`torus_pushed_by_lines`, apart from the kernel, to 1e-9) within
/// 1e-6 of it — 6.6e-8 today: near the cusps' paths the sheet's two leaves stand nearly tangent,
/// and the trim reads them no closer (the fold cut, #880 T1).
#[test]
fn a_ring_slid_past_its_fold_is_swept_by_the_contacts_road() {
    use gcs_core::brep::swept::closed::reads;
    let ring = Ring::leaning(3.,1.,60f64.to_radians());
    let tool = torus([0.;3],ring.frame.z,3.,1.).unwrap();
    let path = slide([0.,0.,6.]);
    match reads(&tool,&path) {
        Err(Unbuilt::Folded {at}) => {
            assert!(ring.inside(at).abs() <= 1e-9,"a point of the ring: {at:?}");
            let l = ring.frame.local(at);
            assert!(l[0].hypot(l[1]) < 3.,"of its inner contact: {at:?}");
        }
        other => panic!("read as folded, not {:?}",other.err()),
    }
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap_or_else(|e| panic!("{e:?}"));
    let expected = torus_pushed_by_lines([0.;3],ring.frame.z,3.,1.,6.,None,1e-9);
    assert!((volume(&built)-expected).abs() <= 1e-6*expected,"{} against {expected}",volume(&built));
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

/// **A mill with rounded corners** (#810, #780 R5): upright about `z` through the origin, its flat
/// end on `z = 0`, radius `big`, `h` tall; its bottom corner a quarter round of radius `c` (a band of
/// the torus of middle radius `big − c` about `z` at height `c`, tube `c`: a bull-nose), and its top
/// corner one of radius `top` (none: a sharp rim) — held apart from the kernel.
pub(crate) struct Mill { pub big: f64,pub c: f64,pub top: f64,pub h: f64 }

impl Mill {
    /// The tool, as the kernel builds it: its section turned once.
    pub(crate) fn tool(&self) -> gcs_core::brep::topo::Brep {
        use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
        let (big,c,top,h) = (self.big,self.c,self.top,self.h);
        let at = |x: f64,z: f64| [x,0.,z];
        let round = |o: V,r: f64,span: [f64;2]| ProfileEdge::Arc {frame:Frame::new(o,[0.,-1.,0.],[1.,0.,0.]),r,span:Some(span)};
        let mut edges = vec![ProfileEdge::Line {a:at(0.,0.),b:at(big-c,0.)},round(at(big-c,c),c,[-PI/2.,0.]),
            ProfileEdge::Line {a:at(big,c),b:at(big,h-top)}];
        if top > 0. { edges.push(round(at(big-top,h-top),top,[0.,PI/2.])) }
        edges.extend([ProfileEdge::Line {a:at(big-top,h),b:at(0.,h)},ProfileEdge::Line {a:at(0.,h),b:at(0.,0.)}]);
        revolve(&Profile {origin:at(0.,0.),normal:[0.,1.,0.],names:Vec::new(),loops:vec![edges]},[0.;3],[0.,0.,1.],TAU).unwrap()
    }
    /// Its volume: the drum less the rings its corners' sections `r² (1 − π/4)` turn, each about its
    /// centroid `r (10 − 3π)/(3 (4 − π))` in from the sharp corner each way, by Pappus.
    pub(crate) fn volume(&self) -> f64 {
        let ring = |r: f64| TAU*(self.big-r*(10.-3.*PI)/(3.*(4.-PI)))*r*r*(1.-PI/4.);
        PI*self.big*self.big*self.h-ring(self.c)-ring(self.top)
    }
    /// A bound below the distance from `p` to the tool, nought on it and inside: it is the infinite
    /// drum rounded at the bottom (its core `ρ ≤ big − c`, `z ≥ c`, grown by `c`) in common with the
    /// one rounded at the top, and its distance from a common part is at least each's.
    fn below(&self,p: V) -> f64 {
        let (r,z) = (p[0].hypot(p[1]),p[2]);
        let low = (r-(self.big-self.c)).max(0.).hypot((self.c-z).max(0.))-self.c;
        let high = (r-(self.big-self.top)).max(0.).hypot((z-(self.h-self.top)).max(0.))-self.top;
        low.max(high).max(0.)
    }
    /// **Its shadow along the unit `d`** (leaning off its axis and off square to it): the region its
    /// outline seen along `d` bounds, `½ ∮ (x y′ − y x′)` over that outline in a frame square to
    /// `d`. Seen along `d` the bottom and the top of the side are one ellipse shifted, and the
    /// outline is: the bottom round's outer contact where it lies on the round (`tan v = −(d_x cos u
    /// + d_y sin u)/d_z`, `v ≤ 0`: the half circle of `u` about the angle opposite where `d_xy`
    /// points with `d_z` above nought — `[a, b]`), the side's rulings at `b` and `a`, where the
    /// contact crosses the seam into the side's, and between them the top's: the top round's contact
    /// where it lies on the round (`v ≥ 0`), or the sharp top rim, over the other half `[b, a + 2π]`.
    /// Each arc by Gauss–Legendre over 256 pieces (`gauss`, exact to rounding), each ruling exactly.
    pub(crate) fn shadow(&self,d: V) -> f64 {
        let (big,h) = (self.big,self.h);
        let (e1,e2) = gcs_core::space::across(d);
        let flat = |p: V| [dot(p,e1),dot(p,e2)];
        // ½ ∫ (x y′ − y x′) over a piece `p(t)`, `t` in `[a, b]`, its derivative `dp`
        let swept = |p: &dyn Fn(f64) -> V,dp: &dyn Fn(f64) -> V,a: f64,b: f64| -> f64 {
            gauss(&|t| { let ([x0,y0],[x1,y1]) = (flat(p(t)),flat(dp(t))); x0*y1-y0*x1 },a,b,256)/2.
        };
        let seg = |p: V,q: V| { let ([x0,y0],[x1,y1]) = (flat(p),flat(q)); (x0*y1-y0*x1)/2. };
        let g = |u: f64| d[0]*u.cos()+d[1]*u.sin();
        let dg = |u: f64| -d[0]*u.sin()+d[1]*u.cos();
        // a round's outer contact, the round of radius `r` about the circle at height `z0`, and its rate
        let contact = |r: f64,z0: f64,u: f64| -> (V,V) {
            let w = -g(u)/d[2];
            let (v,dv) = (w.atan(),-dg(u)/d[2]/(1.+w*w));
            let (rho,drho) = (big-r+r*v.cos(),-r*v.sin()*dv);
            ([rho*u.cos(),rho*u.sin(),z0+r*v.sin()],[drho*u.cos()-rho*u.sin(),drho*u.sin()+rho*u.cos(),r*v.cos()*dv])
        };
        let centre = d[1].atan2(d[0])+if d[2] < 0. { PI } else { 0. };
        let (a,b) = (centre-PI/2.,centre+PI/2.);
        let (c,top) = (self.c,self.top);
        let low = swept(&|u| contact(c,c,u).0,&|u| contact(c,c,u).1,a,b);
        let high = if top > 0. { swept(&|u| contact(top,h-top,u).0,&|u| contact(top,h-top,u).1,b,a+TAU) }
            else { swept(&|u| [big*u.cos(),big*u.sin(),h],&|u| [-big*u.sin(),big*u.cos(),0.],b,a+TAU) };
        let side = |u: f64,z: f64| [big*u.cos(),big*u.sin(),z];
        (low+seg(side(b,c),side(b,h-top))+high+seg(side(a,h-top),side(a,c))).abs()
    }
    /// **The sweep's definition, read** (`swept`, stepped by `below`): whether `p` is in the mill slid
    /// by `d`.
    pub(crate) fn swept(&self,d: V,p: V) -> bool { swept(&|x| self.below(x),d,p) }
}

/// The mill `mill` slid by `d`, built alone (#810's gate) and held to `V + |D| A` within 1e-9 —
/// every line along the slide meets the convex mill once or not at all — to membership at seeded
/// points of its box (`swept_closed::agrees`), and to the sweep's definition read at 20000 points
/// (`Mill::swept`) within four standard errors of its volume.
fn mill_closes(mill: &Mill,d: V) {
    let tool = mill.tool();
    assert!((volume(&tool)-mill.volume()).abs() <= 1e-12*mill.volume(),"the mill: {} against {}",volume(&tool),mill.volume());
    let path = slide(d);
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap_or_else(|e| panic!("slid {d:?}: {e:?}"));
    let expected = mill.volume()+norm(d)*mill.shadow(unit(d));
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"slid {d:?}: {} against {expected}",volume(&built));
    crate::swept_closed::agrees(&built,&tool,&path);
    let (lo,hi) = built.bounds();
    let (sampled,error) = crate::swept_closed::monte_carlo(lo.map(|x| x-0.1),hi.map(|x| x+0.1),20000,810,&|p| mill.swept(d,p));
    assert!((sampled-volume(&built)).abs() <= 4.*error,"sampled {sampled} ± {error} against {}",volume(&built));
}

/// **A bull-nose mill slid at a lean closes into its sweep** (#810, #780 R5's first gate): radius 2,
/// its corner rounded 0.5, 3 tall, slid 5 down a way leaning 29° off its axis. Its round's contact is
/// the outer contact's half on the round (`v ≤ 0`), ending where it crosses the seam into the side's
/// two rulings whose normal is square to the slide; swept, it is the sheet over that arc of the
/// contour (`Cutter::Sheet`), which the planes the rulings sweep touch along the seam points' paths —
/// given to the arrangement (`boolean::Given`), not sought.
#[test]
fn a_bull_nose_slid_at_a_lean_closes_into_its_sweep() {
    mill_closes(&Mill {big:2.,c:0.5,top:0.,h:3.},scale(unit([0.5,0.2,-1.]),5.));
}

/// **A bull-nose mill slid up aslant by less than its reach closes into its sweep**: the mill of
/// `a_bull_nose_slid_at_a_lean_closes_into_its_sweep` slid 1.5 up a way leaning 27° off its axis
/// the other way, its end poses overlapping — where the deepest points of the cell between its top
/// at the two end poses stand on its top at the middle pose, whose membership the search took
/// depth first ran out of reads on (`member`, best first now).
#[test]
fn a_bull_nose_slid_up_aslant_by_less_than_its_reach_closes_into_its_sweep() {
    mill_closes(&Mill {big:2.,c:0.5,top:0.,h:3.},scale(unit([0.3,-0.4,1.]),1.5));
}

/// **A bull-nose mill whose contact crosses its seams on its own seam meridian closes into its
/// sweep**: slid across `y`, the contact reaches the side at `u = 0` and `π`, where a solid of
/// revolution's seam stands — the cylinder's, the torus's and the top rim's at the end poses, each
/// touched there by a plane a ruling sweeps; each is turned about its axis (`closed`'s `seam_away`)
/// so no touch runs along a seam.
#[test]
fn a_bull_nose_slid_across_its_seam_meridian_closes_into_its_sweep() {
    mill_closes(&Mill {big:2.,c:0.5,top:0.,h:3.},scale(unit([0.,0.7,-1.]),5.));
}

/// **A drum rounded at both corners slid at a lean closes into its sweep** (#810's dispatch: a
/// rounded drum's corners): each corner a band, the bottom's contact on its lower half and the
/// top's on its upper, each crossing its seam into the side's rulings on opposite sides — two
/// sheets, four touches given.
#[test]
fn a_drum_rounded_at_both_corners_slid_at_a_lean_closes_into_its_sweep() {
    mill_closes(&Mill {big:2.,c:0.5,top:0.5,h:3.},scale(unit([0.3,-0.4,1.]),5.));
}

/// **Membership reads its deepest pose first** (#810, `member::Swept::member`): a point on the
/// bull-nose's top at the middle pose of a slide up, inside it a little further on — the stretch of
/// roll before the middle, where it stands within the mesh's sag of the top, is halved to `FINEST`
/// last, after the stretch where it is inside is read.
#[test]
fn membership_reads_the_deepest_pose_first() {
    use gcs_core::brep::swept::member::{Solid,Swept};
    let mill = Mill {big:2.,c:0.5,top:0.,h:3.};
    let (tool,d) = (mill.tool(),scale(unit([0.3,-0.4,1.]),1.5));
    let path = slide(d);
    let solid = Solid::new(&tool,0.002,1.).unwrap();
    // on the top at the middle pose
    let p = add(scale(d,0.5),[0.76,0.28,3.]);
    assert!(mill.swept(d,p));
    assert_eq!((Swept {tool:&solid,path:&path,roll:[0.,1.]}).member(p),Ok(true));
}

/// **A round meeting its neighbour in a sharp rim at a lean builds through its fan** (#810's
/// negative control, turned by Q1b-S2): a mill whose corner round spans only the eighth of the tube
/// above `v = −π/4`, so the flat end meets it there at 45° — a sharp circle on the torus, whose fan
/// (the sheet its normals between the two faces' sweep) the patch road does not enter.  The
/// admission (`closed::reads`) still refuses it as `Curved` at a point of that rim; the build goes
/// the contacts road, which carries the rim's stretch where the faces' `n·v` differ in sign as a
/// sheet of its own, and holds the volume to the meridian's own reading (`advanced`).
#[test]
fn a_round_meeting_a_sharp_rim_at_a_lean_builds_through_its_fan() {
    use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
    use gcs_core::brep::swept::closed::reads;
    use gcs_core::brep::swept::patches::Revolved;
    use crate::swept_patches::advanced;
    let (big,c,h) = (2.,0.5,3.);
    let at = |x: f64,z: f64| [x,0.,z];
    let (s,co) = (-PI/4.).sin_cos();
    let rim = at(big-c+c*co,c+c*s);
    let edges = vec![ProfileEdge::Line {a:at(0.,rim[2]),b:rim},
        ProfileEdge::Arc {frame:Frame::new(at(big-c,c),[0.,-1.,0.],[1.,0.,0.]),r:c,span:Some([-PI/4.,0.])},
        ProfileEdge::Line {a:at(big,c),b:at(big,h)},ProfileEdge::Line {a:at(big,h),b:at(0.,h)},ProfileEdge::Line {a:at(0.,h),b:at(0.,rim[2])}];
    let tool = revolve(&Profile {origin:at(0.,0.),normal:[0.,1.,0.],names:Vec::new(),loops:vec![edges]},[0.;3],[0.,0.,1.],TAU).unwrap();
    let path = slide(scale(unit([0.5,0.2,-1.]),5.));
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).expect("built through its fan");
    let read = advanced(&Revolved::read(&tool).unwrap(),&tool,&path,[0.,1.],false);
    assert!((volume(&built)-read).abs() <= 1e-5*read,"{} against {read}",volume(&built));
    match reads(&tool,&path) {
        Err(Unbuilt::Curved {at}) => assert!((at[0].hypot(at[1])-rim[0]).abs() <= 1e-9 && (at[2]-rim[2]).abs() <= 1e-9,"on the sharp rim: {at:?}"),
        other => panic!("refused by name as curved, not {:?}",other.err()),
    }
}

impl Mill {
    /// **What the mill takes from a block pushed straight down into it** until wholly under the
    /// block's top (`z = 0`), its axis leaning `beta` from upright toward `x`, the middle of its
    /// flat end at `base` where it stops (`solid_sweeps/leaning_bullnose.sv`'s): every line down
    /// the push meets the convex mill once or not at all, so the block loses each line's depth to
    /// the mill's underside, `−∫ z_low dA` over its shadow — by the divergence theorem the flux of
    /// `(0, 0, z)` out of the faces turned down, `V − ∫ z n_z dS` over those turned up: its top
    /// disc whole (`n_z = cos β`), the side where its normal climbs (`ρ̂_z = −sin β cos u > 0`) and the
    /// round above the curve where the normal is level on it (`tan v* = −ρ̂_z / cos β`, `v ≥ v*`)
    /// — each by Gauss–Legendre over its parameters (smooth, so to rounding). A bottom round alone.
    pub(crate) fn plunged_away(&self,beta: f64,base: V) -> f64 {
        assert_eq!(self.top,0.,"a bull-nose");
        let (big,c,h) = (self.big,self.c,self.h);
        let (sb,cb) = beta.sin_cos();
        let x = [cb,0.,-sb];
        let rho = |u: f64| add(scale(x,u.cos()),[0.,u.sin(),0.]);
        let gl = gauss;
        let disc = cb*PI*big*big*(base[2]+h*cb);
        let (u0,u1) = (PI/2.,3.*PI/2.);
        let side = gl(&|u| {
            let up = rho(u)[2];
            gl(&|z| (base[2]+big*up+z*cb)*up*big,c,h,4)
        },u0,u1,64);
        let round = gl(&|u| {
            let up = rho(u)[2];
            let low = -(up/cb).atan();
            gl(&|v| {
                let (sv,cv) = v.sin_cos();
                let z = base[2]+(big-c+c*cv)*up+(c+c*sv)*cb;
                z*(cv*up+sv*cb)*(big-c+c*cv)*c
            },low,0.,16)
        },u0,u1,64);
        self.volume()-(disc+side+round)
    }
}

/// **A bull-nose mill pushed at a lean into a block stops inside it** (#810's gate, cut from a
/// blank): the mill of `a_bull_nose_slid_at_a_lean_closes_into_its_sweep`, its axis leaning 30°,
/// pushed 8 straight down from clear above a block 16 × 16 × 8 until its flat end's middle is 4 under
/// the top, wholly within: the block loses `Mill::plunged_away`, to 1e-9, which a Monte Carlo of the
/// sweep's definition agrees with.
#[test]
fn a_bull_nose_pushed_at_a_lean_into_a_block_stops_inside_it() {
    use gcs_core::brep::build::{prism,square};
    use gcs_core::brep::swept::closed::cut_from;
    let mill = Mill {big:2.,c:0.5,top:0.,h:3.};
    let beta = 30f64.to_radians();
    let lean = Frame {o:[0.,0.,4.],x:[beta.cos(),0.,-beta.sin()],y:[0.,1.,0.],z:[beta.sin(),0.,beta.cos()]};
    let tool = mill.tool().moved(&lean.placing());
    let block = prism(&square([0.,0.,-8.],[0.,0.,1.],8.),0.,8.).unwrap();
    let d = [0.,0.,-8.];
    let built = cut_from(&block,&tool,&slide(d),[0.,1.],0.002,1e-6).unwrap_or_else(|e| panic!("{e:?}"));
    let away = mill.plunged_away(beta,[0.,0.,-4.]);
    let expected = 16.*16.*8.-away;
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected} ({away} away)",volume(&built));
    // the definition: the block's points the mill, carried back up the push, holds somewhere
    let local = lean.dir_local(d);
    let (sampled,error) = crate::swept_closed::monte_carlo([-8.,-8.,-8.],[8.,8.,0.],20000,810,&|p| mill.swept(local,lean.local(p)));
    assert!((sampled-away).abs() <= 4.*error,"sampled {sampled} ± {error} against {away}");
}

/// **The slid touches build through their given touches** (#810, item 5): with `ssi`'s pair-by-pair
/// recognition of a torus at a pose of a slide against its contact's cylinder bypassed
/// (`Recognised::SlidTouch`), #805's ring slid at a lean and the bull-nose slid at a lean still close
/// into their sweeps to the same closed forms — the arrangement holds each touch as the closed
/// class gives it (`boolean::Given`), not as `intersect` recognised it.
#[test]
fn the_slid_touches_build_through_their_given_touches() {
    use gcs_core::brep::ssi::{Recognised,bypassing};
    bypassing(&[Recognised::SlidTouch],|| {
        slid_closes(35.,[0.,0.,6.]);
        mill_closes(&Mill {big:2.,c:0.5,top:0.,h:3.},scale(unit([0.5,0.2,-1.]),5.));
    });
}

/// `∫ f` over `[lo, hi]` within `tol`, by Gauss–Kronrod's 7–15 rule halved where its two estimates
/// part by more than their share of `tol` — never below the estimate's own rounding, nor past
/// twenty halvings, so a share `f`'s noise cannot meet stops there.
pub(crate) fn kronrod(f: &dyn Fn(f64) -> f64,lo: f64,hi: f64,tol: f64) -> f64 {
    const XK: [f64;8] = [0.991455371120812639,0.949107912342758525,0.864864423359769073,0.741531185599394440,
        0.586087235467691130,0.405845151377397167,0.207784955007898468,0.];
    const WK: [f64;8] = [0.022935322010529225,0.063092092629978553,0.104790010322250184,0.140653259715525919,
        0.169004726639267903,0.190350578064785410,0.204432940075298892,0.209482141084727828];
    const WG: [f64;4] = [0.129484966168869693,0.279705391489276668,0.381830050505118945,0.417959183673469388];
    fn go(f: &dyn Fn(f64) -> f64,lo: f64,hi: f64,tol: f64,depth: u32) -> f64 {
        let (m,h) = (0.5*(lo+hi),0.5*(hi-lo));
        let (mut k,mut g) = (WK[7]*f(m),WG[3]*f(m));
        for i in 0..7 {
            let s = f(m-h*XK[i])+f(m+h*XK[i]);
            k += WK[i]*s;
            if i % 2 == 1 { g += WG[i/2]*s }
        }
        let (k,g) = (k*h,g*h);
        if (k-g).abs() <= tol.max(1e-14*k.abs()) || depth >= 20 { return k }
        go(f,lo,m,tol/2.,depth+1)+go(f,m,hi,tol/2.,depth+1)
    }
    go(f,lo,hi,tol,0)
}

/// **What a torus pushed along `z` sweeps, line by line** (#880 T1: past its fold too), apart from
/// the kernel: the torus about `centre` along the unit `axis`, middle radius `big` and tube
/// `small`, carried `push` along `z`. The vertical line through `(x, y)` meets it in the intervals
/// between the real roots of a quartic in its height (`(s + w² + R² − r²)² = 4R² (s + w² − (b + n_z
/// w)²)`, `w` the height over the centre, `s` and `b` the line's own), each isolated between the
/// quartic's turning points and bisected; the push lengthens each by `push` the way it goes, and
/// the line keeps their union, clipped to `within` where given. Across `y` at each `x` that length
/// is smooth but where the line's make-up changes: where it crosses the outline seen down `z` (a
/// root pair born or lost, at the outline's crossings, found in its own parameter so none is missed
/// however narrow, as between a folded outline's cusps), and where two intervals run together or an
/// end crosses a clip (a scan of 400 lines, bisected); each smooth piece is read by
/// `swept_loci::integral`, its square-root ends tamed. Over `x`, by `kronrod` to `tol`, split where
/// the outline turns back in `x`.
pub(crate) fn torus_pushed_by_lines(centre: V,axis: V,big: f64,small: f64,push: f64,within: Option<[f64;2]>,tol: f64) -> f64 {
    let reach = big+small;
    // the swept length on the line, and its make-up: how many roots, intervals kept, ends clipped
    let line = |x: f64,y: f64| -> (f64,[u32;4]) {
        let (qx,qy) = (x-centre[0],y-centre[1]);
        let (s,b) = (qx*qx+qy*qy,qx*axis[0]+qy*axis[1]);
        let a = s+big*big-small*small;
        let r4 = 4.*big*big;
        // w⁴ + c2 w² + c1 w + c0
        let (c2,c1,c0) = (2.*a-r4*(1.-axis[2]*axis[2]),2.*r4*b*axis[2],a*a-r4*(s-b*b));
        let f = |w: f64| ((w*w+c2)*w+c1)*w+c0;
        // the quartic's turning points, the real roots of its derivative `4w³ + 2 c2 w + c1` (a
        // depressed cubic, by its trigonometric or Cardano form), cut the line into stretches each
        // holding at most one root of its own, found by bisection to rounding
        let (p,q) = (c2/2.,c1/4.);
        let disc = q*q/4.+p*p*p/27.;
        let mut turns: Vec<f64> = if disc > 0. {
            let u = (-q/2.+disc.sqrt()).cbrt();
            vec![u-if u != 0. { p/(3.*u) } else { 0. }]
        } else {
            let m = 2.*(-p/3.).sqrt();
            let phi = if m > 0. { (3.*q/(p*m)).clamp(-1.,1.).acos()/3. } else { 0. };
            (0..3).map(|k| m*(phi-2.*PI*k as f64/3.).cos()).collect()
        };
        turns.sort_by(f64::total_cmp);
        let far = 2.*(1.+c2.abs()+c1.abs()+c0.abs()).sqrt().sqrt()+reach;
        let mut ends = vec![-far];
        ends.extend(turns);
        ends.push(far);
        let mut roots: Vec<f64> = ends.windows(2).filter_map(|w| {
            let (mut a,mut b) = (w[0],w[1]);
            let (fa,fb) = (f(a),f(b));
            if fa == 0. { return Some(a) }
            if (fa < 0.) == (fb < 0.) { return None }
            for _ in 0..200 { let m = 0.5*(a+b); if m <= a || m >= b { break } if (f(m) < 0.) == (fa < 0.) { a = m } else { b = m } }
            Some(0.5*(a+b))
        }).collect();
        roots.sort_by(f64::total_cmp);
        let mut swept: Vec<[f64;2]> = roots.windows(2).filter(|p| f(0.5*(p[0]+p[1])) < 0.)
            .map(|p| [centre[2]+p[0]+push.min(0.),centre[2]+p[1]+push.max(0.)]).collect();
        swept.sort_by(|p,q| p[0].total_cmp(&q[0]));
        let mut merged: Vec<[f64;2]> = Vec::new();
        for iv in swept {
            match merged.last_mut() { Some(m) if iv[0] <= m[1] => m[1] = m[1].max(iv[1]),_ => merged.push(iv) }
        }
        let [lo,hi] = within.unwrap_or([f64::NEG_INFINITY,f64::INFINITY]);
        let clipped = |k: usize| merged.iter().map(|iv| iv[k]).filter(|&e| e < lo || e > hi).count() as u32;
        (merged.iter().map(|iv| (iv[1].min(hi)-iv[0].max(lo)).max(0.)).sum(),[roots.len() as u32,merged.len() as u32,clipped(0),clipped(1)])
    };
    // the outline seen along `z`: where the torus's normal is level, `tan v = −(cos u e1_z + sin u
    // e2_z)/n_z` and the half turn on, read at 20000 points of each branch
    let (e1,e2) = gcs_core::space::across(axis);
    let outline_at = |shift: f64,u: f64| -> [f64;2] {
        let (su,cu) = u.sin_cos();
        let v = (-(cu*e1[2]+su*e2[2])/axis[2]).atan()+shift;
        let (sv,cv) = v.sin_cos();
        let out = big+small*cv;
        [centre[0]+out*(cu*e1[0]+su*e2[0])+small*sv*axis[0],centre[1]+out*(cu*e1[1]+su*e2[1])+small*sv*axis[1]]
    };
    let outline: Vec<(f64,Vec<[f64;2]>)> = [0.,PI].iter().map(|&shift| (shift,(0..=20000).map(|k| outline_at(shift,TAU*k as f64/20000.)).collect())).collect();
    let across = |x: f64| -> f64 {
        let (y0,y1) = (centre[1]-reach,centre[1]+reach);
        let scan = 400;
        let ys: Vec<f64> = (0..=scan).map(|k| y0+(y1-y0)*k as f64/scan as f64).collect();
        let kinds: Vec<[u32;4]> = ys.iter().map(|&y| line(x,y).1).collect();
        let mut cuts = vec![y0];
        for k in 0..scan {
            if kinds[k] == kinds[k+1] { continue }
            let (mut a,mut b) = (ys[k],ys[k+1]);
            for _ in 0..80 { let m = 0.5*(a+b); if line(x,m).1 == kinds[k] { a = m } else { b = m } }
            cuts.push(0.5*(a+b));
        }
        // and where the outline crosses the line `x`: each sample pair astride it, bisected in `u`
        for (shift,branch) in &outline {
            for (k,w) in branch.windows(2).enumerate() {
                if (w[0][0] < x) == (w[1][0] < x) { continue }
                let (mut lo,mut hi) = (TAU*k as f64/20000.,TAU*(k+1) as f64/20000.);
                let below = w[0][0] < x;
                for _ in 0..60 { let m = 0.5*(lo+hi); if (outline_at(*shift,m)[0] < x) == below { lo = m } else { hi = m } }
                cuts.push(outline_at(*shift,0.5*(lo+hi))[1]);
            }
        }
        cuts.push(y1);
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|p,q| (*p-*q).abs() <= 1e-12);
        cuts.windows(2).map(|w| crate::swept_loci::integral(w[0],w[1],16,&|y| line(x,y).0)).sum()
    };
    // over `x`, split where the outline turns back in `x` (its extremes and its cusps: where the
    // number of its crossings with a line changes), each turn bisected in `u`
    let mut xs = vec![centre[0]-reach,centre[0]+reach];
    for (shift,branch) in &outline {
        let dx: Vec<f64> = branch.windows(2).map(|w| w[1][0]-w[0][0]).collect();
        for k in 0..dx.len()-1 {
            if (dx[k] < 0.) == (dx[k+1] < 0.) { continue }
            let rising = dx[k] > 0.;
            let (mut lo,mut hi) = (TAU*k as f64/20000.,TAU*(k+2) as f64/20000.);
            for _ in 0..60 {
                let (m1,m2) = (lo+(hi-lo)/3.,hi-(hi-lo)/3.);
                let (a,b) = (outline_at(*shift,m1)[0],outline_at(*shift,m2)[0]);
                if (a < b) == rising { lo = m1 } else { hi = m2 }
            }
            xs.push(outline_at(*shift,0.5*(lo+hi))[0]);
        }
    }
    xs.sort_by(f64::total_cmp);
    xs.dedup_by(|p,q| (*p-*q).abs() <= 1e-12);
    xs.windows(2).map(|w| kronrod(&across,w[0],w[1],tol*(w[1]-w[0])/(2.*reach))).sum()
}
