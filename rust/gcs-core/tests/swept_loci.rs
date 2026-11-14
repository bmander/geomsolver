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
use gcs_core::brep::swept::closed::closed;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use std::f64::consts::{PI,TAU};

/// The screw about the world's `z` turning `theta` and rising `k` a radian over the roll `[0, 1]`.
fn screw(theta: f64,k: f64) -> Trajectory {
    Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:theta,advance:TAU*k*theta})
}

/// `∫ f` over `[a, b]` by Gauss–Legendre's eight points (`integral::GAUSS8`) on each of `n` pieces,
/// each read at `a + (b − a)(1 − cos τ)/2` so that a square-root end is smooth in `τ`.
fn integral(a: f64,b: f64,n: usize,f: &dyn Fn(f64) -> f64) -> f64 {
    if !(b > a) { return 0. }
    let g = |t: f64| { let (s,c) = t.sin_cos(); f(0.5*(a+b)-0.5*(b-a)*c)*0.5*(b-a)*s };
    (0..n).map(|i| {
        let (lo,hi) = (PI*i as f64/n as f64,PI*(i+1) as f64/n as f64);
        let (m,h) = (0.5*(lo+hi),0.5*(hi-lo));
        gcs_core::integral::GAUSS8.iter().map(|&(x,w)| w*h*g(m+h*x)).sum::<f64>()
    }).sum()
}

/// Where a concave `f` is most over `[a, b]`, and how much (`roots::brent` on `−f`, its ends read
/// too).
fn most(a: f64,b: f64,f: &dyn Fn(f64) -> f64) -> (f64,f64) {
    let least = gcs_core::roots::brent(&|x| -f(x),a,b,1e-14*(1.+a.abs().max(b.abs())),200,|_,_| false);
    [(least.at,-least.value),(a,f(a)),(b,f(b))].into_iter().fold((a,f64::NEG_INFINITY),|m,x| if x.1 > m.1 { x } else { m })
}

/// Where `f` changes sign between each two of `n + 1` points evenly over each of `pieces`, settled
/// within that bracket (`roots::bracketed_root`).
fn sign_changes(pieces: &[f64],n: usize,f: &dyn Fn(f64) -> f64) -> Vec<f64> {
    let mut out = Vec::new();
    for w in pieces.windows(2) {
        let xs: Vec<f64> = (0..=n).map(|i| w[0]+(w[1]-w[0])*i as f64/n as f64).collect();
        let fs: Vec<f64> = xs.iter().map(|&x| f(x)).collect();
        for i in 0..n {
            if fs[i]*fs[i+1] < 0. { out.push(gcs_core::roots::bracketed_root(|x| Some(f(x)),xs[i],fs[i],xs[i+1],fs[i+1],1e-14*(1.+xs[i].abs()))) }
        }
    }
    out
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
/// inner point enters and leaves the strip, and where `L` crosses `2π`; in `ρ` where the slice's
/// height is the climb and where `L` first reaches `2π` up a slice; each piece read by `integral`.
fn swept_volume(r: f64,d: f64,k: f64,theta: f64) -> f64 {
    let rise = k*theta;
    // (the angles a circle's slice covers are at most its own and the roll's: short of a turn
    // together, they never cover it, and `L` has no `2π` to cross)
    let returns = theta+2.*(r/d).asin() >= TAU;
    // at the radius `ρ`: the slice's half-height, and `L` at the height `z` (`span`, uncapped)
    let slice = |rho: f64| {
        let h = (r*r-(rho-d)*(rho-d)).max(0.).sqrt();
        let half = move |zb: f64| ((rho*rho+d*d+zb*zb-r*r)/(2.*rho*d)).clamp(-1.,1.).acos();
        let span = move |z: f64| -> f64 {
            let (lo,hi) = ((z-rise).max(-h),z.min(h));
            if !(hi > lo) { return 0. }
            most(lo,hi,&|zb| half(zb)-zb/k).1+most(lo,hi,&|zb| half(zb)+zb/k).1
        };
        (h,half,span)
    };
    let at_rho = |rho: f64| -> f64 {
        let (h,half,span) = slice(rho);
        if !(h > 0.) { return 0. }
        // each extreme's inner point over the whole slice
        let (iu,il) = (most(-h,h,&|zb| half(zb)-zb/k).0,most(-h,h,&|zb| half(zb)+zb/k).0);
        let mut cuts = vec![-h,h,-h+rise,h+rise,iu,iu+rise,il,il+rise];
        cuts.retain(|&z| z >= -h && z <= h+rise);
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|x,y| (*x-*y).abs() <= 1e-13);
        // and where `L` crosses `2π`
        if returns { let more = sign_changes(&cuts,64,&|z| span(z)-TAU); cuts.extend(more); cuts.sort_by(f64::total_cmp) }
        cuts.windows(2).map(|w| integral(w[0],w[1],8,&|z| span(z).min(TAU))).sum::<f64>()*rho
    };
    let (lo,hi) = (d-r,d+r);
    let mut cuts = vec![lo,hi];
    if rise < 2.*r { let off = (r*r-0.25*rise*rise).sqrt(); cuts.extend([d-off,d+off]) }
    if returns {
        // the most `L` reaches up the slice, less `2π`: sampled, and polished
        let reaches = |rho: f64| -> f64 {
            let (h,_,span) = slice(rho);
            let step = (2.*h+rise)/200.;
            let best = (0..=200).map(|i| -h+step*i as f64).max_by(|x,y| span(*x).total_cmp(&span(*y))).unwrap_or(0.);
            most(best-step,best+step,&span).1-TAU
        };
        cuts.extend(sign_changes(&[lo,hi],400,&reaches));
    }
    cuts.sort_by(f64::total_cmp);
    cuts.windows(2).map(|w| integral(w[0],w[1],32,&at_rho)).sum()
}

/// **The volume a ball of radius `r` about `(d, 0, 0)` sweeps screwed about the world's `z` through
/// `theta` radians, rising `k` a radian, by the circles about the axis** (#880 T1), apart from the
/// kernel and from `swept_volume`, which takes the ball clear of the axis: this one any ball, the
/// pipe folding or not. On the circle of radius `ρ` at height `z`, the ball screwed through `φ`
/// covers the arc about `φ` where `cos(ψ − φ) ≥ c(φ) = (ρ² + d² + (z − kφ)² − r²)/2ρd`, of
/// half-width `w = acos c`, every point where `c ≤ −1`, none where `c > 1`. Over the rolls with `c
/// ≤ 1`, one stretch, the arcs run together into `[min (φ − w), max (φ + w)]` of the unwrapped
/// angle, so the circle is covered `min(L, 2π)` of its `2π`: each extreme sampled at 256 rolls and
/// polished by Brent's method (`most`). The volume is `∫ ρ ∫ min(L, 2π) dz dρ`, each integral cut
/// where its integrand's make-up changes — over `z` where the rolls reach a circle's height, where
/// the end poses stop clamping them and where whole circles start and stop; over `ρ` at `|r − d|`
/// and where the stretches of circles covered all round change in number — and each piece's
/// square-root ends tamed (`swept_slid::tamed`). (Uncut, the halving was fooled by where a circle's
/// coverage begins, which moves with `ρ`, and by where a stretch covered all round is born: the
/// bench's ball read 1e-6 low, and the ball screwed tighter than its radius 4.2e-7 apart between
/// 1e-9 and 1e-10.)
pub(crate) fn swept_by_circles(r: f64,d: f64,k: f64,theta: f64,tol: f64) -> f64 {
    // `L`, the arcs' run unwrapped (`2·2π` where a roll covers the circle whole)
    let run = |rho: f64,z: f64| -> f64 {
        // the rolls with `c ≤ 1`: `(z − kφ)² ≤ r² − (ρ − d)²`
        let reach = r*r-(rho-d)*(rho-d);
        if !(reach > 0.) || !(rho > 0.) { return 0. }
        let s = reach.sqrt();
        let (lo,hi) = (((z-s)/k).max(0.),((z+s)/k).min(theta));
        if !(hi > lo) { return 0. }
        // a roll with `c ≤ −1` covers the whole circle: `(z − kφ)² ≤ r² − (ρ + d)²`
        let whole = r*r-(rho+d)*(rho+d);
        if whole >= 0. { let s = whole.sqrt(); if ((z-s)/k).max(0.) <= ((z+s)/k).min(theta) { return 2.*TAU } }
        let w = |phi: f64| { let zb = z-k*phi; ((rho*rho+d*d+zb*zb-r*r)/(2.*rho*d)).clamp(-1.,1.).acos() };
        let extreme = |f: &dyn Fn(f64) -> f64| -> f64 {
            let n = 256;
            let i = (0..=n).max_by(|&i,&j| f(lo+(hi-lo)*i as f64/n as f64).total_cmp(&f(lo+(hi-lo)*j as f64/n as f64))).unwrap();
            let at = |i: usize| lo+(hi-lo)*i.min(n) as f64/n as f64;
            most(at(i.saturating_sub(1)),at(i+1),f).1
        };
        extreme(&|phi| phi+w(phi))+extreme(&|phi| w(phi)-phi)
    };
    let covered = |rho: f64,z: f64| run(rho,z).min(TAU);
    // over `z`, cut where the circle's make-up changes: where the rolls first and last reach its
    // height (`±s` past the ends' heights), where the end poses stop clamping them, and where whole
    // circles start and stop — each piece's square-root ends tamed
    let along = |rho: f64,tol: f64| -> f64 {
        let reach = r*r-(rho-d)*(rho-d);
        if !(reach > 0.) { return 0. }
        let (s,top) = (reach.sqrt(),k*theta);
        let mut cuts = vec![-s,s,top-s,top+s];
        let whole = r*r-(rho+d)*(rho+d);
        if whole > 0. { let w = whole.sqrt(); cuts.extend([-w,w,top-w,top+w]) }
        cuts.retain(|&z| z >= -s && z <= top+s);
        cuts.sort_by(f64::total_cmp);
        cuts.windows(2).map(|w| crate::swept_slid::tamed(w[0],w[1],tol*(w[1]-w[0])/(top+2.*s),&|z| covered(rho,z))).sum()
    };
    // over `ρ`, cut at `|r − d|` and where the stretches along `z` of circles covered all round by
    // the arcs' run (`L ≥ 2π`) change in number — where one is born its contribution grows as a
    // power three halves, which no rule across it settles (the ball screwed tighter than its
    // radius at 0.8492, the bench's at 2.8651) — found among 400 radii, each counted at 400
    // heights, and bisected
    let stretches = |rho: f64| -> usize {
        let reach = r*r-(rho-d)*(rho-d);
        if !(reach > 0.) { return 0 }
        let (a,b) = (-reach.sqrt(),k*theta+reach.sqrt());
        let mut was = false;
        (0..=400).filter(|&i| { let on = run(rho,a+(b-a)*i as f64/400.) >= TAU; let born = on && !was; was = on; born }).count()
    };
    let rhos: Vec<f64> = (0..=400).map(|i| (d+r)*i as f64/400.).collect();
    let counts: Vec<usize> = rhos.iter().map(|&x| stretches(x)).collect();
    let mut cuts = vec![0.,(r-d).abs(),d+r];
    for i in 1..400 {
        if counts[i] == counts[i+1] { continue }
        let (mut a,mut b) = (rhos[i],rhos[i+1]);
        for _ in 0..50 { let m = 0.5*(a+b); if stretches(m) == counts[i] { a = m } else { b = m } }
        cuts.push(0.5*(a+b));
    }
    cuts.sort_by(f64::total_cmp);
    cuts.windows(2).map(|w| crate::swept_slid::tamed(w[0],w[1],0.5*tol,&|rho| rho*along(rho,0.1*tol/(d+r)))).sum()
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
    let p = pipe(0.2);
    let w = p.spread().unwrap();
    let roots = p.returns([TAU-2.*w-1e-9,2.*TAU],1e-9).unwrap();
    assert_eq!(roots.len(),2,"{roots:?}");
    let s = p.radian().unwrap();
    for [t1,t2,t] in roots {
        let (a,b) = (p.circle(t1).0,s.at(p.circle(t2).0,t));
        assert!(gcs_core::space::distance(a,b) <= 1e-12,"{a:?} against {b:?}");
        assert!((t-TAU).abs() < 2.*w,"a turn on: {t}");
    }
    let w = pipe(0.4).spread().unwrap();
    assert!(pipe(0.4).returns([TAU-2.*w-1e-9,2.*TAU],1e-9).unwrap().is_empty());
}

/// **A ball's pipe that folds is swept by the contact's road** (#880 T1; refused by name as
/// `Folded` before it, #805): a ball reaching past its centre's helix's radius of curvature, `(ρ² +
/// κ²)/ρ`, folds its pipe, which the closed class reads as `Folded` at the ball's centre; under a
/// constant twist that refusal is asked of the contact's road, whose trim keeps of the folded pipe
/// what bounds the sweep. Held within 1e-6 at 1e-6 (2.3e-7 of it; 1.7e-9 at the kernel's
/// tolerance) to the sweep's volume circle by circle, 11.7537013464 (`swept_by_circles`, apart
/// from the kernel, unchanged from 1e-8 to 1e-10; `the_tight_balls_reference_is_its_definition`).
#[test]
fn a_ball_screwed_tighter_than_its_radius_is_swept_by_the_contacts_road() {
    let tool = gcs_core::brep::build::sphere([0.5,0.,0.],[0.,0.,1.],1.2).unwrap();
    let built = closed(&tool,&screw(2.,0.1),[0.,1.],0.002,1e-6).unwrap_or_else(|e| panic!("{e:?}"));
    assert!((volume(&built)-TIGHT).abs() <= 1e-6*TIGHT,"{} against {TIGHT}",volume(&built));
}

/// **The tight ball at the kernel's tolerance** (#880 T1b): within its boundary's area times the
/// tolerance of its volume circle by circle — 2e-8 today, 1.7e-9 of it.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about ten seconds: the ball at the kernel's tolerance")]
fn the_tight_ball_holds_at_the_kernels_tolerance() {
    let tool = gcs_core::brep::build::sphere([0.5,0.,0.],[0.,0.,1.],1.2).unwrap();
    let tol = gcs_core::brep::geom::tolerance(tool.size());
    let built = closed(&tool,&screw(2.,0.1),[0.,1.],0.002,tol).unwrap_or_else(|e| panic!("{e:?}"));
    let within = tol*crate::brep::total_area(&built);
    assert!((volume(&built)-TIGHT).abs() <= within,"{} against {TIGHT}, within {within}",volume(&built));
}

/// The volume the ball of radius 1.2 half out from the axis sweeps screwed 0.1 a radian through 2
/// radians (`a_ball_screwed_tighter_than_its_radius_is_swept_by_the_contacts_road`).
const TIGHT: f64 = 11.7537013464;

/// **The tight ball's reference is its definition**, recomputed circle by circle to 1e-9.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about six minutes: the ball's circles to 1e-9")]
fn the_tight_balls_reference_is_its_definition() {
    let v = swept_by_circles(1.2,0.5,0.1,2.,1e-9);
    assert!((v-TIGHT).abs() <= 1e-9,"{v}");
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
    let mut shared = None;
    for (a,b) in [(&first,&second),(&second,&first)] {
        let Ssi::Curves(curves) = intersect(a,b,1e-9) else { panic!("not read in closed form") };
        if let Some(Curve::Circle(f,_)) = curves.iter().find(|c| matches!(c,Curve::Circle(..))) { shared = Some(*f) }
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
    let f = shared.expect("the circle they share");
    let plane = Surface::Plane(f);
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
    for i in 0..64 {
        for past in [1e-5,1e-4,1e-3] {
            // past the first's end, on the second; and short of the second's start, on the first
            let q = blend.d1([(i as f64+0.5)/64.,past/span]).0;
            assert_eq!(a.face_place(fi,q),Place::Out,"at {q:?}, {past} on");
            assert_eq!(b.face_place(si,q),Place::In,"at {q:?}, {past} on");
            let q = earlier.d1([(i as f64+0.5)/64.,1.-past/span]).0;
            assert_eq!(b.face_place(si,q),Place::Out,"at {q:?}, {past} short");
            assert_eq!(a.face_place(fi,q),Place::In,"at {q:?}, {past} short");
        }
    }
}

