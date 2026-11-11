//! **Trimming by certified membership, face by face** (#880 E1, `brep::swept::trim`,
//! docs/swept-solid-design.md §12): the closed swept solid built from candidates entered by hand —
//! no cell of their arrangement built, only the meetings whose faces stand near the swept boundary
//! sought — held to a volume worked apart from the build, to the sweep's definition (`closed::judge`,
//! run before anything is returned) and to a closed STEP file; and `closed`'s own candidates trimmed,
//! the same solid as their arrangement's, from fewer meetings.
//!
//! E1's three refusals: the bar of `square-bar-screwed-through-rod` (the arrangement's "a cylinder's
//! pieces do not close") is built over the case's whole roll and, cut from the rod by the ordinary
//! Boolean, is the bench's settled solid; the ring of `ring-pushed-in-past-its-fold` (`Folded`)
//! is trimmed as far as the fold's cusp, where three curves tangent along the slide meet on the
//! boundary and no meeting is laid there (`a_ring_past_its_fold_is_trimmed_up_to_its_cusps`); the
//! cone of `cone-spun-about-a-line-through-it` (`Curved`) has no candidate to enter (its base rim's
//! sheet and its contact through the axis are surfaces the kernel does not hold yet).
use crate::brep::block;
use gcs_core::brep::build::{disc,prism,torus};
use gcs_core::brep::geom::{Curve,Frame,Rigid};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::swept::closed::{Unbuilt,closed};
use gcs_core::brep::swept::trim::{self,Report,Trim};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use std::f64::consts::{PI,TAU};

/// The bar of `square-bar-screwed-through-rod`: the box `4 ≤ x ≤ 12, −8 ≤ y ≤ 8, −1 ≤ z ≤ 1`.
fn bar() -> Brep { block([4.,-8.,-1.],[12.,8.,1.]) }

/// Its screw: about z, right-handed, rising 6 a turn, a radian a radian of the roll.
const SCREW: Screw = Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:6.};

/// **The bar's candidates, entered by hand**: the bar at each end pose, each of its faces a sheet in
/// its plane a little past its edges (`trim::planes_past`, so no edge of the bar is an edge of a
/// candidate); its edges along the axis and its long faces' contact lines (`y = 0` on `x = 4` and
/// on `x = 12`, where the face's normal is square to the screw's velocity) each the cylinder about
/// the axis it screws round, over the heights the sweep spans; and its edges square to the axis,
/// each the patch of the helicoid it screws into over the roll (`trim::screwed_segment`).
fn bar_candidates(roll: [f64;2]) -> Vec<Brep> {
    let tool = bar();
    let path = Trajectory::Twist(SCREW);
    let mut out = Vec::new();
    for t in roll {
        let m = path.pose(t).unwrap();
        out.extend(trim::planes_past(&tool.moved(&Rigid {r:m.r,t:m.p}),1.).unwrap());
    }
    let rise = |t: f64| SCREW.advance*t/TAU;
    let (z0,z1) = (-2.+rise(roll[0]),2.+rise(roll[1]));
    for r in [4.,12.,80f64.sqrt(),208f64.sqrt()] {
        let c = prism(&disc([0.,0.,z0],[0.,0.,1.],r),0.,z1-z0).unwrap();
        out.push(trim::sheet_of(&c,&|f| f.surface.kind() == "cylinder"));
    }
    for z in [-1.,1.] {
        for (p,q) in [([4.,-8.,z],[4.,8.,z]),([12.,-8.,z],[12.,8.,z]),([4.,-8.,z],[12.,-8.,z]),([4.,8.,z],[12.,8.,z])] {
            out.push(trim::screwed_segment(&SCREW,p,q,roll).unwrap());
        }
    }
    out
}

/// The bar screwed over `roll`, trimmed from its hand-entered candidates at `resolution`.
fn bar_trimmed(roll: [f64;2],resolution: f64) -> (Brep,Report) {
    let tool = bar();
    trim::trimmed(&tool,&Trajectory::Twist(SCREW),roll,&bar_candidates(roll),&[],tool.size(),1e-7,Trim {resolution,hair:1e-3})
        .unwrap_or_else(|e| panic!("the bar's sweep not trimmed: {e:?}"))
}

/// **The volume the bar sweeps screwed over `roll`, apart from the build**, in helical coordinates
/// `(ρ, ψ, ζ)`, a point `(ρ cos ψ, ρ sin ψ, ζ + k ψ)`, `k` the rise a radian: the volume element is
/// `ρ dρ dψ dζ`, and every point lies on one helix `(ρ, ζ)` with `ζ` in a period `[−πk, πk)` and
/// `ψ` over the whole line. The screw shifts `ψ`, so the sweep meets each helix in the bar's
/// meetings with it lengthened by the roll `Θ`. A helix meets the bar where it stands within the
/// bar's height, a window of `ψ` of `2/k` (less than a turn), and within the bar's slice at its
/// radius — one arc about `ψ = 0` up to `ρ = 12`, two past it (the bar's two far corners, as
/// horns): so one helix may meet it twice, and the roll, longer than the gap between them, joins
/// them. Each helix's share is then `ρ (span + Θ)`, `span` from the first meeting's start to the last
/// one's end. In `ζ` that is piecewise linear between the places where the window's ends cross the
/// arcs' ends, integrated exactly; in `ρ`, by adaptive Simpson between the radii where the arcs
/// change (4, √80, 12, √208).
fn bar_volume(roll: [f64;2]) -> f64 {
    let k = SCREW.advance/TAU;
    let theta = roll[1]-roll[0];
    let arcs = |r: f64| -> Vec<[f64;2]> {
        if r < 4. { return Vec::new() }
        let hi = (4./r).min(1.).acos().min((8./r).min(1.).asin());
        if r <= 12. { return vec![[-hi,hi]] }
        let lo = (12./r).acos();
        if lo >= hi { Vec::new() } else { vec![[-hi,-lo],[lo,hi]] }
    };
    let span = |r: f64,z0: f64| -> Option<f64> {
        let (w0,w1) = ((-1.-z0)/k,(1.-z0)/k);
        let mut found: Option<[f64;2]> = None;
        for [p,q] in arcs(r) { for n in -2..=2 {
            let (lo,hi) = ((p+TAU*n as f64).max(w0),(q+TAU*n as f64).min(w1));
            if lo < hi { found = Some(found.map_or([lo,hi],|[a,b]| [a.min(lo),b.max(hi)])) }
        } }
        found.map(|[a,b]| b-a)
    };
    let per_radius = |r: f64| -> f64 {
        let period = PI*k;
        let mut breaks = vec![-period,period];
        for [p,q] in arcs(r) { for n in -2..=2 { for e in [p,q] { for c in [-1.,1.] {
            let z = c-k*(e+TAU*n as f64);
            if z > -period && z < period { breaks.push(z) }
        } } } }
        breaks.sort_by(f64::total_cmp);
        breaks.windows(2).map(|w| {
            let (z0,z1) = (w[0],w[1]);
            let Some(m) = span(r,0.5*(z0+z1)) else { return 0. };
            let at = |z: f64| span(r,z).unwrap_or(m);
            let eps = 1e-12*(z1-z0);
            (z1-z0)*((at(z0+eps)+4.*m+at(z1-eps))/6.+theta)
        }).sum::<f64>()*r
    };
    fn adaptive(f: &dyn Fn(f64) -> f64,a: f64,b: f64,fa: f64,fm: f64,fb: f64,tol: f64,depth: usize) -> f64 {
        let m = 0.5*(a+b);
        let (l,r) = (0.5*(a+m),0.5*(m+b));
        let (fl,fr) = (f(l),f(r));
        let one = (b-a)/6.*(fa+4.*fm+fb);
        let two = (b-a)/12.*(fa+4.*fl+2.*fm+4.*fr+fb);
        if depth > 40 || (two-one).abs() <= 15.*tol { return two+(two-one)/15. }
        adaptive(f,a,m,fa,fl,fm,tol/2.,depth+1)+adaptive(f,m,b,fm,fr,fb,tol/2.,depth+1)
    }
    let radii = [4.,80f64.sqrt(),12.,208f64.sqrt()];
    radii.windows(2).map(|w| {
        let (a,b) = (w[0],w[1]);
        adaptive(&per_radius,a,b,per_radius(a),per_radius(0.5*(a+b)),per_radius(b),1e-10,0)
    }).sum()
}

/// `x` within `rel` of `want`, relatively.
fn near(x: f64,want: f64,rel: f64,what: &str) {
    assert!((x-want).abs() <= rel*want.abs(),"{what}: {x} where {want} (off {:.1e})",(x-want).abs()/want.abs());
}

/// **The bar screwed two turns, trimmed from its hand-entered candidates, is the solid the
/// arrangement of `closed`'s builds**: its volume the helical integral worked apart from the build,
/// judged by the sweep's definition before it was returned, and fewer pairs of faces met than the
/// candidates' boxes meet. The arrangement's own figure (4550.143322182, `closed` over the same roll)
/// is held here as a number, the arrangement taking ten seconds.
#[test]
fn the_bar_screwed_two_turns_is_trimmed_from_hand_entered_candidates() {
    let roll = [-90f64.to_radians(),630f64.to_radians()];
    let (b,report) = bar_trimmed(roll,1.);
    near(volume(&b),bar_volume(roll),1e-9,"the bar's sweep");
    near(volume(&b),4550.1433221825,1e-11,"the arrangement's figure");
    assert!(report.paired < report.boxed,"{report:?}");
    assert_eq!(report.meetings[1],0,"every meeting in closed form: {report:?}");
}

/// **The bar of `square-bar-screwed-through-rod`, screwed its whole roll (four turns), trimmed**
/// (E1): what the arrangement refuses ("a cylinder's pieces do not close", over the half-space
/// blocks it enters for the end poses' faces) is built from the candidates entered by hand — to
/// the helical integral apart from the build, judged before it is returned, written as a STEP
/// file that reads back as the solid — and, cut from the rod by the ordinary Boolean, it is the
/// bench's settled solid: volume 1300.5502131828668 and area 821.16715556389149 (brep-bench's
/// closed forms).
#[test]
fn the_bar_screwed_through_the_rod_is_trimmed_over_its_whole_roll() {
    let roll = [-90f64.to_radians(),1350f64.to_radians()];
    let (b,report) = bar_trimmed(roll,1.);
    near(volume(&b),bar_volume(roll),1e-9,"the bar's sweep");
    assert!(report.paired < report.boxed,"{report:?}");
    let text = gcs_core::brep::step::write(&b,"swept",1e-7).unwrap();
    gcs_core::brep::step_check::verify(&text,&gcs_core::brep::step_check::Solid::of(&b)).unwrap_or_else(|e| panic!("its STEP: {e}"));
    let rod = prism(&disc([0.;3],[0.,0.,1.],5.),0.,20.).unwrap();
    let cut = gcs_core::brep::boolean::combined(&rod,&b,gcs_core::brep::boolean::Op::Cut,1e-7).unwrap();
    near(volume(&cut),1300.5502131828668,1e-10,"the rod less the sweep");
    near(cut.faces.iter().map(|f| area(&cut,f)).sum(),821.16715556389149,1e-10,"its area");
}

/// **`closed`'s own candidates, trimmed, are the solid their arrangement is** (E1's comparison): a
/// box slid on a slant and a block turned a quarter turn about a line beside it, each built both
/// ways — the same volume — the trim meeting fewer pairs of faces than the candidates' boxes
/// meet, and seeking fewer meetings of surfaces than the arrangement (472 and 158, read off its
/// profile, `SOLVENT_SWEEP_PROFILE`).
#[test]
fn closeds_candidates_trimmed_are_their_arrangements_solid() {
    let slide = Trajectory::Twist(Screw {origin:[0.;3],axis:gcs_core::space::unit([1.,0.5,0.25]),ratio:0.,
        advance:gcs_core::space::norm([1.,0.5,0.25])*2.});
    let turn = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.});
    for (what,tool,path,roll,arranged_meetings) in [
        ("a box slid aslant",block([0.,0.,0.],[1.,2.,3.]),&slide,[0.,TAU],472),
        ("a block turned a quarter",block([2.,0.,0.],[3.,1.,1.]),&turn,[0.,PI/2.],158),
    ] {
        let arranged = closed(&tool,path,roll,1.,1e-7).unwrap();
        let (trimmed,report) = gcs_core::brep::swept::closed::trimmed(&tool,path,roll,1.,1e-7,Trim {resolution:tool.size()/32.,hair:1e-3})
            .unwrap_or_else(|e| panic!("{what}: {e:?}"));
        near(volume(&trimmed),volume(&arranged),1e-9,what);
        assert!(report.paired < report.boxed,"{what}: {report:?}");
        assert!(report.meetings.iter().sum::<u64>() < arranged_meetings,"{what}: {report:?}");
    }
}

/// **The ring of `ring-pushed-in-past-its-fold`, trimmed as far as its cusps** (E1): the torus at
/// each end pose (its seam turned off the contacts' halves), its outer contact's cylinder whole, and
/// its inner contact — whose apparent contour folds — entered in its regular arcs, cut at the four
/// cusps where the contact runs along the slide. Each torus touches each sheet along the contact
/// there, given; and past the fold a line along the slide through an inner contact point meets
/// the torus a second time, at `t = −2q ± √(2(2R²|D_xy|² − |p|² − R² + r²))` along the slide (the
/// quartic less its double root at the contact), so each inner sheet meets each torus again along
/// those points' curve, given too (fitted within 1e-10): it bounds the end pose's leading face
/// where the ring's own far side shades it. The trim zones, pairs and meets them; what stops it is
/// the cusp, where the contact, its second meeting and the line along the slide through it are
/// tangent to one another on the boundary, and no vertex is laid where they meet. Refused, never
/// built wrong.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, about 13 s: the folded ring trimmed to its cusps")]
fn a_ring_past_its_fold_is_trimmed_up_to_its_cusps() {
    use gcs_core::brep::turned::Exact;
    let lean = 65f64.to_radians();
    let (centre,axis,big,small) = ([0.,0.,4.],[lean.sin(),0.,lean.cos()],3.,1.);
    let ends = [centre,[0.,0.,-4.]];
    let tool = torus(centre,axis,big,small).unwrap();
    let path = Trajectory::Twist(Screw {origin:centre,axis:[0.,0.,-1.],ratio:0.,advance:8.});
    let d = [0.,0.,-1.];
    let tori: Vec<Brep> = ends.iter().map(|&c| torus(c,axis,big,small).unwrap().moved(&Rigid::turn(c,axis,PI/4.))).collect();
    let surface = |b: &Brep| b.faces[0].surface.clone();
    // the halves about the frame's y, across the lean, where the inner contour folds
    let f0 = Frame::about(centre,axis);
    let f = Frame {x:f0.y,y:gcs_core::space::scale(f0.x,-1.),..f0};
    let span = [-12.,12.];
    let mut candidates = tori.clone();
    let mut given = Vec::new();
    let outer = gcs_core::brep::slid::halves(f,big,small,d,1.,false);
    let cylinder = trim::contour_cylinder(&outer.map(|h| h.at(centre,true)),span).unwrap();
    for (k,c) in ends.iter().enumerate() {
        given.push(gcs_core::brep::boolean::Given {surfaces:[surface(&tori[k]),surface(&cylinder)],
            curves:outer.iter().map(|h| Curve::Slid(std::sync::Arc::new(h.at(*c,false)))).collect()});
    }
    candidates.push(cylinder);
    for h in gcs_core::brep::slid::halves(f,big,small,d,-1.,false) {
        let cusps = cusps(&h.at(centre,true));
        assert_eq!(cusps.len(),2,"two cusps a half past the fold: {cusps:?}");
        let cuts: Vec<f64> = std::iter::once(-1.).chain(cusps).chain(std::iter::once(1.)).collect();
        for w in cuts.windows(2) {
            let arc = gcs_core::brep::slid::Slid {span:[w[0],w[1]],..h};
            let sheet = trim::contour_arc(&arc.at(centre,true),span).unwrap();
            for (k,c) in ends.iter().enumerate() {
                let contact = arc.at(*c,false);
                let mut curves = vec![Curve::Slid(std::sync::Arc::new(contact))];
                curves.extend(again(&contact,*c,axis,big,small));
                given.push(gcs_core::brep::boolean::Given {surfaces:[surface(&tori[k]),surface(&sheet)],curves});
            }
            candidates.push(sheet);
        }
    }
    let out = trim::trimmed(&tool,&path,[0.,TAU],&candidates,&given,0.01,1e-5,Trim {resolution:0.5,hair:1e-3});
    match out {
        Ok((b,_)) => panic!("the folded ring trimmed: {:.6} — its cusps laid, this is a gate to write",volume(&b)),
        Err(Unbuilt::Wrong {at,..}) => panic!("built wrong at {at:?}"),
        Err(why) => assert!(matches!(why,Unbuilt::Failed(ref m) if m.contains("pieces do not close")),"{why:?}"),
    }

    /// Where a slid contour's speed falls to nought: its cusps, each the least of its speed between
    /// samples, golden-sectioned.
    fn cusps(c: &gcs_core::brep::slid::Slid) -> Vec<f64> {
        let speed = |s: f64| gcs_core::space::norm(c.d2(s).1);
        let [a,b] = c.span;
        let n = 4000;
        let at = |i: usize| a+(b-a)*i as f64/n as f64;
        (1..n).filter(|&i| speed(at(i)) < speed(at(i-1)) && speed(at(i)) <= speed(at(i+1)) && speed(at(i)) < 0.05).map(|i| {
            let (mut lo,mut hi) = (at(i-1),at(i+1));
            for _ in 0..200 {
                let (m1,m2) = (lo+(hi-lo)*0.382,lo+(hi-lo)*0.618);
                if speed(m1) < speed(m2) { hi = m2 } else { lo = m1 }
            }
            0.5*(lo+hi)
        }).collect()
    }

    /// Where the lines along the slide through a contact `arc` (its points at the torus about `c`)
    /// meet that torus again: `t = −2q ± √disc` along the slide, `disc = 2 (2R²|D_xy|² − |p|² − R² +
    /// r²)`, each stretch of the arc where `disc > 0` one curve through both roots — read through a
    /// root of `disc` by a parameter that takes its square root (`s = m + h cos θ` between two
    /// roots, `s = s₀ ± σ²` from one), so the curve is smooth there — fitted within 1e-10.
    fn again(arc: &gcs_core::brep::slid::Slid,c: [f64;3],axis: [f64;3],big: f64,small: f64) -> Vec<Curve> {
        use gcs_core::space::{add,dot,scale,sub};
        let d = arc.slide();
        let flat = 1.-dot(d,axis)*dot(d,axis);
        let read = |s: f64| { let x = arc.d2(s).0; let p = sub(x,c); (x,dot(p,d),2.*(2.*big*big*flat-dot(p,p)-(big*big-small*small))) };
        let disc = |s: f64| read(s).2;
        let root = |mut lo: f64,mut hi: f64| { for _ in 0..200 { let m = 0.5*(lo+hi); if (disc(m) > 0.) == (disc(lo) > 0.) { lo = m } else { hi = m } } 0.5*(lo+hi) };
        let [a,b] = arc.span;
        let n = 2000;
        let at = |i: usize| a+(b-a)*i as f64/n as f64;
        let mut stretches: Vec<((f64,bool),(f64,bool))> = Vec::new();
        let mut start = (disc(a) > 0.).then_some((a,false));
        for i in 0..n {
            let (p,q) = (disc(at(i)) > 0.,disc(at(i+1)) > 0.);
            if !p && q { start = Some((root(at(i),at(i+1)),true)) }
            if p && !q { if let Some(s) = start.take() { stretches.push((s,(root(at(i),at(i+1)),true))) } }
        }
        if let Some(s) = start { stretches.push((s,(b,false))) }
        let point = |s: f64,signed: f64| { let (x,q,_) = read(s); add(x,scale(d,-2.*q+signed)) };
        let fit = |f: &dyn Fn(f64) -> [f64;3]| gcs_core::brep::nurbs::fit_curve(f,3,64,1e-10)
            .map(|(bs,_)| Curve::BSpline(std::sync::Arc::new(bs)));
        let mut out = Vec::new();
        for ((lo,lr),(hi,hr)) in stretches {
            match (lr,hr) {
                (true,true) => {
                    let (m,h) = (0.5*(lo+hi),0.5*(hi-lo));
                    out.extend(fit(&|t| { let th = t*TAU; let s = m+h*th.cos();
                        point(s,th.sin()*h*(disc(s)/((s-lo)*(hi-s))).max(0.).sqrt()) }));
                }
                (true,false) | (false,true) => {
                    let (r0,dir) = if lr { (lo,1.) } else { (hi,-1.) };
                    let m = (hi-lo).sqrt();
                    out.extend(fit(&|t| { let sg = -m+2.*m*t; let s = r0+dir*sg*sg;
                        point(s,sg*(disc(s)/(dir*(s-r0))).max(0.).sqrt()) }));
                }
                (false,false) => for sign in [1.,-1.] {
                    out.extend(fit(&|t| { let s = lo+(hi-lo)*t; point(s,sign*disc(s).max(0.).sqrt()) }));
                }
            }
        }
        out
    }
}
