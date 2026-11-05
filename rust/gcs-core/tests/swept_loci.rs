//! **Loci in a patch's domain, and a sheet meeting itself** (#780 L1, docs/swept-solid-design.md
//! §3 and step 6, `brep::swept::closed`): a ball screwed through more than a turn at a lead short
//! of its diameter sweeps a pipe that comes back onto itself (X5). Entered stretch by stretch, each
//! meeting itself nowhere, the pipe's stretches meet pairwise in closed form — along the helices of
//! the points of its circle the screw carries onto one another a turn on (`geom::Pipe::returns`),
//! and round the circle where two abut — and the arrangement and membership do the rest. Held to a
//! volume worked apart from the build in the screw's helical coordinates, to a Monte Carlo of the
//! sweep's definition, and to membership.
use gcs_core::brep::geom::{Carry,Frame,Pipe};
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::{Unbuilt,closed};
use gcs_core::brep::trajectory::{Screw,Trajectory};
use std::f64::consts::{PI,TAU};

/// The screw about the world's `z` turning `theta` and rising `k` a radian over the roll `[0, 1]`.
fn screw(theta: f64,k: f64) -> Trajectory {
    Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:theta,advance:TAU*k*theta})
}

/// `∫ f` over `[a, b]` by Gauss–Legendre's sixteen points on each of `n` pieces, each read at
/// `a + (b − a)(1 − cos τ)/2` so that a square-root end is smooth in `τ`.
fn integral(a: f64,b: f64,n: usize,f: &dyn Fn(f64) -> f64) -> f64 {
    const X: [f64;8] = [0.0950125098376374,0.2816035507792589,0.4580167776572274,0.6178762444026438,
        0.7554044083550030,0.8656312023878318,0.9445750230732326,0.9894009349916499];
    const W: [f64;8] = [0.1894506104550685,0.1826034150449236,0.1691565193950025,0.1495959888165767,
        0.1246289712555339,0.0951585116824928,0.0622535239386479,0.0271524594117541];
    if !(b > a) { return 0. }
    let g = |t: f64| { let (s,c) = t.sin_cos(); f(0.5*(a+b)-0.5*(b-a)*c)*0.5*(b-a)*s };
    (0..n).map(|i| {
        let (lo,hi) = (PI*i as f64/n as f64,PI*(i+1) as f64/n as f64);
        let (m,h) = (0.5*(lo+hi),0.5*(hi-lo));
        (0..8).map(|j| W[j]*h*(g(m-h*X[j])+g(m+h*X[j]))).sum::<f64>()
    }).sum()
}

/// The most of a concave `f` over `[a, b]`, by golden sections.
fn most(a: f64,b: f64,f: &dyn Fn(f64) -> f64) -> f64 {
    let g = 0.5*(5f64.sqrt()-1.);
    let (mut a,mut b) = (a,b);
    for _ in 0..90 {
        let (c,d) = (b-g*(b-a),a+g*(b-a));
        if f(c) > f(d) { b = d } else { a = c }
    }
    f(0.5*(a+b)).max(f(a)).max(f(b))
}

/// **The volume a ball of radius `r` about `(d, 0, 0)` sweeps screwed about the world's `z` through
/// `theta` radians, rising `k` a radian** (`k > 0`), worked apart from the kernel in the screw's
/// helical coordinates. A point at `(ρ, ψ, z)` is swept where the ball holds it screwed back by
/// some `φ` in `[0, θ]`: where the ball's slice by the cylinder of radius `ρ`, `B_ρ` in `(ψ_b, z_b)`
/// (`ρ² + d² − 2ρd cos ψ_b + z_b² ≤ r²`, convex), holds `(ψ − φ, z − kφ)` up to whole turns of `ψ`.
/// So at each `(ρ, z)` the angles swept are `z/k + σ` for `σ = ψ_b − z_b/k` over the points of `B_ρ`
/// with `z_b` in `[z − kθ, z]` — one interval, the image of a convex set — and round the circle they
/// cover `min(2π, σ_max − σ_min)`: the pipe meeting itself is where that reaches `2π`. `σ_max` is
/// the most of the concave `φ_ρ(z_b) − z_b/k` over the strip (`φ_ρ` the slice's half-width, `acos((ρ²
/// + d² + z_b² − r²)/2ρd)`), `σ_min` the least of `−φ_ρ − z_b/k`. The volume is `∫ ρ ∫ min(2π, L) dz
/// dρ`: in `z` cut where the strip's ends cross the slice's top and bottom, where each extreme's
/// inner point enters and leaves the strip, and where `L` crosses `2π`, each piece read by
/// `integral`; in `ρ` read at `ρ = d + r sin θ`, Simpson's rule on 2000 steps.
fn swept_volume(r: f64,d: f64,k: f64,theta: f64) -> f64 { swept_volume_at(r,d,k,theta,4,16) }

fn swept_volume_at(r: f64,d: f64,k: f64,theta: f64,zp: usize,n: usize) -> f64 {
    let rise = k*theta;
    // (the angles a circle's slice covers are at most its own and the roll's: short of a turn
    // together, they never cover it, and `L` has no `2π` to cross)
    let returns = theta+2.*(r/d).asin() >= TAU;
    let at_rho = |rho: f64| -> f64 {
        let h2 = r*r-(rho-d)*(rho-d);
        if !(h2 > 0.) { return 0. }
        let h = h2.sqrt();
        let half = |zb: f64| ((rho*rho+d*d+zb*zb-r*r)/(2.*rho*d)).clamp(-1.,1.).acos();
        let strip = |z: f64| [(z-rise).max(-h),z.min(h)];
        let upper = |zb: f64| half(zb)-zb/k;
        let lower = |zb: f64| half(zb)+zb/k;
        let span = |z: f64| -> f64 {
            let [lo,hi] = strip(z);
            if !(hi > lo) { return 0. }
            (most(lo,hi,&upper)+most(lo,hi,&lower)).min(TAU)
        };
        // each extreme's inner point over the whole slice, where it is inside
        let inner = |f: &dyn Fn(f64) -> f64| -> f64 {
            let g = 0.5*(5f64.sqrt()-1.);
            let (mut a,mut b) = (-h,h);
            for _ in 0..90 { let (c,e) = (b-g*(b-a),a+g*(b-a)); if f(c) > f(e) { b = e } else { a = c } }
            0.5*(a+b)
        };
        let (iu,il) = (inner(&upper),inner(&lower));
        let mut cuts = vec![-h,h,-h+rise,h+rise,iu,iu+rise,il,il+rise];
        cuts.retain(|&z| z >= -h && z <= h+rise);
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|x,y| (*x-*y).abs() <= 1e-13);
        // where `L` crosses `2π`: bracketed among 64 samples of each piece, bisected
        let raw = |z: f64| { let [lo,hi] = strip(z); if hi > lo { most(lo,hi,&upper)+most(lo,hi,&lower)-TAU } else { -TAU } };
        let mut all = cuts.clone();
        for w in cuts.windows(2).filter(|_| returns) {
            let n = 64;
            for i in 0..n {
                let (mut a,mut b) = (w[0]+(w[1]-w[0])*i as f64/n as f64,w[0]+(w[1]-w[0])*(i+1) as f64/n as f64);
                let (fa,fb) = (raw(a),raw(b));
                if fa*fb >= 0. { continue }
                for _ in 0..80 { let m = 0.5*(a+b); if (raw(m) > 0.) == (fa > 0.) { a = m } else { b = m } }
                all.push(0.5*(a+b));
            }
        }
        all.sort_by(f64::total_cmp);
        all.windows(2).map(|w| integral(w[0],w[1],zp,&span)).sum::<f64>()*rho
    };
    // `∫ … dρ` cut where its integrand turns a corner: where the slice's height `2h` is the climb
    // (the strip's ends trade places), and where `L` first reaches `2π` somewhere up the slice
    // (the turns first cover a whole circle); each piece by `integral`, its ends smooth in `τ`
    let (lo,hi) = (d-r,d+r);
    let mut cuts = vec![lo,hi];
    if rise < 2.*r { let off = (r*r-0.25*rise*rise).sqrt(); cuts.extend([d-off,d+off]) }
    let whole = |rho: f64| -> f64 {
        // the most `L` reaches over the slice, less `2π`: sampled, and polished by golden sections
        let h = (r*r-(rho-d)*(rho-d)).max(0.).sqrt();
        let half = |zb: f64| ((rho*rho+d*d+zb*zb-r*r)/(2.*rho*d)).clamp(-1.,1.).acos();
        let l = |z: f64| { let (a,b) = ((z-rise).max(-h),z.min(h)); if b > a { most(a,b,&|zb| half(zb)-zb/k)+most(a,b,&|zb| half(zb)+zb/k) } else { 0. } };
        let zs: Vec<f64> = (0..=200).map(|i| -h+(2.*h+rise)*i as f64/200.).collect();
        let best = zs.iter().copied().max_by(|x,y| l(*x).total_cmp(&l(*y))).unwrap_or(0.);
        let step = (2.*h+rise)/200.;
        most(best-step,best+step,&l)-TAU
    };
    let rs: Vec<f64> = (0..=400).map(|i| lo+(hi-lo)*i as f64/400.).collect();
    for w in rs.windows(2).filter(|_| returns) {
        let (mut a,mut b) = (w[0],w[1]);
        let (fa,fb) = (whole(a),whole(b));
        if fa*fb >= 0. { continue }
        for _ in 0..60 { let m = 0.5*(a+b); if (whole(m) > 0.) == (fa > 0.) { a = m } else { b = m } }
        cuts.push(0.5*(a+b));
    }
    cuts.sort_by(f64::total_cmp);
    cuts.windows(2).map(|w| integral(w[0],w[1],n,&at_rho)).sum()
}

/// **The sweep's definition sampled**: whether the ball of radius `r` about `(d, 0, 0)`, screwed
/// back by some roll of the screw (`φ` radians about `z`, `k φ` along it, over `[0, θ]`), holds `p` —
/// the roll read at a step of a two-hundredth of the ball's angle at the axis, only where the helix
/// through `p` is within the ball's heights — apart from `swept::member`.
fn screwed_ball_holds(p: [f64;3],r: f64,d: f64,k: f64,theta: f64) -> bool {
    let rho = p[0].hypot(p[1]);
    if (rho-d).abs() > r { return false }
    let psi = p[1].atan2(p[0]);
    let step = (r/d).asin()/200.;
    // the rolls bringing `p` level with the ball: `z − kφ` within `[−r, r]`
    let (lo,hi) = (((p[2]-r)/k).max(0.),((p[2]+r)/k).min(theta));
    if hi < lo { return false }
    let n = ((hi-lo)/step).ceil() as usize+1;
    (0..=n).any(|i| {
        let phi = lo+(hi-lo)*i as f64/n as f64;
        let (s,c) = (psi-phi).sin_cos();
        let z = p[2]-k*phi;
        (rho*c-d).hypot(rho*s).hypot(z) <= r
    })
}

/// **A ball screwed through more than a turn at a lead short of its diameter comes back onto its own
/// pipe, and closes into its sweep** (#780 L1's X5 case): a ball of radius 1 three out from the
/// axis, screwed a turn and a half at 0.2 a radian (a lead of 1.26 against a diameter of 2), and
/// two and a half turns at 0.12 (where a point of the circle meets the helices of two others, a turn
/// and two turns on). Before L1 the pipe, entered whole, crossed itself and the judge refused the
/// build (`Unbuilt::Wrong`). Each is held to its volume worked apart (`swept_volume`) — and that
/// volume is short of the pipe's own by what the turns share, so the case is the pipe meeting
/// itself — to a Monte Carlo of the definition within four standard errors, and to membership.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 70 s: two balls screwed past a turn, each built, judged and sampled")]
fn a_ball_screwed_past_a_turn_meets_its_own_pipe_and_closes_into_its_sweep() {
    let (r,d) = (1.,3.);
    for (k,turns) in [(0.2,1.5),(0.12,2.5)] {
        let theta = turns*TAU;
        let tool = gcs_core::brep::build::sphere([d,0.,0.],[0.,0.,1.],r).unwrap();
        let path = screw(theta,k);
        let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap_or_else(|e| panic!("{turns} turns at {k}: {e:?}"));
        let expected = swept_volume(r,d,k,theta);
        // the pipe as though it met itself nowhere (Weyl's tube and the ball), which it does
        let unmet = 4./3.*PI*r*r*r+PI*r*r*theta*(d*d+k*k).sqrt();
        assert!(expected < unmet-0.5,"{turns} turns at {k}: the turns share nothing ({expected} against {unmet})");
        assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{turns} turns at {k}: {} against {expected}",volume(&built));
        let (lo,hi) = built.bounds();
        let (sampled,error) = crate::swept_closed::monte_carlo(lo,hi,20000,31,&|p| screwed_ball_holds(p,r,d,k,theta));
        assert!((sampled-expected).abs() <= 4.*error,"{turns} turns at {k}: sampled {sampled} ± {error} against {expected}");
        crate::swept_closed::agrees(&built,&tool,&path);
    }
}

/// **The worked volume is the pipe's where the pipe meets itself nowhere**: below a turn,
/// `swept_volume` is Weyl's tube and the ball, `4/3 π r³ + π r² θ √(d² + k²)`, which the closed class
/// already builds (`swept_closed::a_ball_screwed_closes_into_the_pipe_about_its_centres_helix`) —
/// the reference held to a closed form before it is held to a build.
#[test]
fn the_worked_volume_of_a_ball_screwed_short_of_a_turn_is_weyls_tube() {
    let (r,d,k,theta): (f64,f64,f64,f64) = (1.,3.,0.2,1.5);
    let weyl = 4./3.*PI*r*r*r+PI*r*r*theta*(d*d+k*k).sqrt();
    let worked = swept_volume(r,d,k,theta);
    assert!((worked-weyl).abs() <= 1e-10*weyl,"{worked} against {weyl}");
}

/// **Where a pipe comes back onto itself, in closed form** (`geom::Pipe::returns`): the ball of
/// radius 1 three out from the axis, screwed 0.2 a radian. Each root carries a point of the start's
/// circle onto another a turn and more on, to rounding: `c(θ₁) = S(t) c(θ₂)`, the two at one
/// distance from the axis and their heights a turn's rise apart once their angles round it are
/// taken off — and there are two (the helices where the turns cross, either side of the circle's
/// middle). A lead longer than the diameter meets nothing.
#[test]
fn a_pipe_comes_back_onto_itself_along_helices() {
    let (r,d) = (1.,3.);
    let pipe = |k: f64| {
        let n = gcs_core::space::unit([0.,d,k]);
        Pipe {carry:Carry::Screw {center:[0.;3],axis:[0.,0.,1.],angle:1.,advance:k},frame:Frame::about([d,0.,0.],n),r}
    };
    let w = pipe(0.2).spread().unwrap();
    let roots = pipe(0.2).returns([TAU-2.*w-1e-9,2.*TAU],1e-9).unwrap();
    assert_eq!(roots.len(),2,"{roots:?}");
    let p = pipe(0.2);
    let s = p.radian().unwrap();
    for [t1,t2,t] in roots {
        let (a,b) = (p.circle(t1),s.at(p.circle(t2),t));
        assert!(gcs_core::space::distance(a,b) <= 1e-12,"{a:?} against {b:?}");
        assert!((t-TAU).abs() < 2.*w,"a turn on: {t}");
    }
    let w = pipe(0.4).spread().unwrap();
    assert!(pipe(0.4).returns([TAU-2.*w-1e-9,2.*TAU],1e-9).unwrap().is_empty());
}

/// **A ball's pipe that would fold is refused by what it is**: a ball reaching past its centre's
/// helix's radius of curvature, `(ρ² + κ²)/ρ`, would fold its pipe (its fold locus, the design's
/// step after) — refused as `Curved` at the ball's centre, not built and not returned wrong.
#[test]
fn a_ball_screwed_tighter_than_its_radius_is_refused_by_name() {
    let tool = gcs_core::brep::build::sphere([0.5,0.,0.],[0.,0.,1.],1.2).unwrap();
    match closed(&tool,&screw(2.,0.1),[0.,1.],0.002,1e-6) {
        Err(Unbuilt::Curved {at}) => assert!(gcs_core::space::distance(at,[0.5,0.,0.]) <= 1e-12),
        other => panic!("{other:?}"),
    }
}


/// Two stretches of the pipe the ball of radius 1 three out from the axis sweeps screwed 0.2 a
/// radian, the first over the turns `[0, a]` and the second over `[a, b]`, each a disc screwed
/// (`build::screwed`, as the closed class enters them): each solid and its pipe's surface.
fn stretches(a: f64,b: f64) -> [(gcs_core::brep::topo::Brep,gcs_core::brep::geom::Surface);2] {
    use gcs_core::brep::build::{disc,screwed};
    let (r,d,k) = (1.,3.,0.2);
    let n = gcs_core::space::unit([0.,d,k]);
    let at = |phi: f64| {
        let (s,c) = phi.sin_cos();
        (([d*c,d*s,k*phi]),[-n[1]*s,n[1]*c,n[2]])
    };
    [[0.,a],[a,b]].map(|[from,to]| {
        let (c,m) = at(from);
        let solid = screwed(&disc(c,m,r),[0.;3],[0.,0.,1.],to-from,k*(to-from)).unwrap();
        let sheet = solid.faces.iter().find(|f| f.surface.kind() == "blend").unwrap().surface.clone();
        (solid,sheet)
    })
}

/// **Two stretches of one pipe meet in closed form** (`ssi`'s `pipes_meeting`): abutting at a turn
/// and a half, a turn and a quarter on from where the first starts, they meet round the circle they
/// share and along two helices where the second comes back over the first, every point of each on
/// both surfaces to rounding; read either way round alike. And a plane holding the circle they share
/// meets the first in that circle (`pipe_plane`), which lies in the plane.
#[test]
fn two_stretches_of_one_pipe_meet_round_their_circle_and_along_helices() {
    use gcs_core::brep::geom::{Curve,Surface};
    use gcs_core::brep::ssi::{Ssi,intersect};
    let [(_,first),(_,second)] = stretches(0.75*TAU,1.5*TAU);
    for (a,b) in [(&first,&second),(&second,&first)] {
        let Ssi::Curves(curves) = intersect(a,b,1e-9) else { panic!("not read in closed form") };
        let circles = curves.iter().filter(|c| matches!(c,Curve::Circle(..))).count();
        let helices = curves.iter().filter(|c| matches!(c,Curve::Iso(..))).count();
        assert_eq!((circles,helices),(1,2),"{curves:?}");
        // the circle all on both; each helix a rail of the first read, all on it, and on the
        // second over the stretch of it the second covers too (past that, on its surface extended)
        for c in &curves {
            let iso = matches!(c,Curve::Iso(..));
            let [t0,t1] = if iso { [0.,1.] } else { [0.,TAU] };
            let mut both = 0;
            for i in 0..=64 {
                let p = c.point(t0+(t1-t0)*i as f64/64.);
                assert!(a.implicit(p).abs() <= 1e-9,"off its own: {}",a.implicit(p));
                if b.implicit(p).abs() <= 1e-9 { both += 1 } else { assert!(iso,"the circle off the other: {}",b.implicit(p)) }
            }
            assert!(both >= 8,"{both} of a helix's points on both");
        }
    }
    let Ssi::Curves(curves) = intersect(&first,&second,1e-9) else { unreachable!() };
    let Some(Curve::Circle(f,_)) = curves.iter().find(|c| matches!(c,Curve::Circle(..))) else { unreachable!() };
    let plane = Surface::Plane(*f);
    let Ssi::Curves(cut) = intersect(&first,&plane,1e-9) else { panic!("not read in closed form") };
    assert!(cut.iter().any(|c| matches!(c,Curve::Circle(g,_) if gcs_core::space::distance(g.o,f.o) <= 1e-9)),"{cut:?}");
}

/// **A point past a patch's rim is in none of its face** (`query::Located::face_place`), however near
/// the rim's edge: the failure X5's build met, reduced. Two stretches of one pipe abut round a
/// circle, the second running on smoothly from the first; a point of the second's surface a
/// ten-thousandth past that circle lies on the first's surface extended too, and the first's inverse
/// clamps it onto its end circle — where it was read as in the first's face, so a ray crossing the
/// pipe beside the circle was counted on both stretches and the selected solid placed a point
/// outside it as in. It is out of the first's face and in the second's.
#[test]
fn a_point_past_a_patchs_rim_is_in_none_of_its_face() {
    use gcs_core::brep::query::{Located,Place};
    let [(first,_),(second,_)] = stretches(0.75*TAU,1.5*TAU);
    let fi = first.faces.iter().position(|f| f.surface.kind() == "blend").unwrap();
    let si = second.faces.iter().position(|f| f.surface.kind() == "blend").unwrap();
    let gcs_core::brep::geom::Surface::Blend(_,blend) = &second.faces[si].surface else { unreachable!() };
    // the second's points all round, from a hundred-thousandth to a thousandth of a radian of turn
    // on from where it starts, beside the first's end circle
    let span = 0.75*TAU;
    let gcs_core::brep::geom::Surface::Blend(_,earlier) = &first.faces[fi].surface else { unreachable!() };
    let (a,b) = (Located::new(&first,1e-6),Located::new(&second,1e-6));
    let mut read = 0;
    for i in 0..64 {
        for past in [1e-5,1e-4,1e-3] {
            // past the first's end, on the second; and short of the second's start, on the first
            let q = blend.d1([(i as f64+0.5)/64.,past/span]).0;
            assert_eq!(a.face_place(fi,q),Place::Out,"at {q:?}, {past} on");
            assert_eq!(b.face_place(si,q),Place::In,"at {q:?}, {past} on");
            let q = earlier.d1([(i as f64+0.5)/64.,1.-past/span]).0;
            assert_eq!(b.face_place(si,q),Place::Out,"at {q:?}, {past} short");
            assert_eq!(a.face_place(fi,q),Place::In,"at {q:?}, {past} short");
            read += 1;
        }
    }
    assert_eq!(read,192);
}

