//! The searches of one variable and the Newton step onto several zero sets (`roots.rs`), which the
//! field's readings, its creases and the refinement's crossings share: each against a function
//! whose answer is known in closed form.
use gcs_core::roots::{bisect,bracketed_root,brent,least_norm_step,newton_onto,sampled_roots,Least,Root};
use gcs_core::space::{dot,cross,norm,sub};

#[test]
fn brent_finds_the_minimum_of_a_parabola_and_of_a_quartic() {
    let Least {at:x,value:v} = brent(&|x: f64| (x-0.3)*(x-0.3)+2.,-1.,1.,1e-12,80,|_,_| false);
    assert!((x-0.3).abs() < 1e-8 && (v-2.).abs() < 1e-15,"{x} {v}");
    // a flat-bottomed minimum, where parabolic steps are least trusted
    let x = brent(&|x: f64| (x-0.7).powi(4),0.,1.,1e-12,200,|_,_| false).at;
    assert!((x-0.7).abs() < 1e-3,"{x}");
    // a minimum at the bracket's end is approached, never passed
    let x = brent(&|x: f64| x,0.,1.,1e-10,80,|_,_| false).at;
    assert!(x >= 0. && x < 1e-8,"{x}");
}

/// **A touch is a root of the derivative** (`sampled_roots`): a double root of `f` placed where
/// `df` changes sign, to its rounding — a least `|f|` stands only to the square root of `f`'s — and
/// found between two samples level either side of it, where neither is a sampled minimum alone.
#[test]
fn sampled_roots_place_a_touch_where_the_derivative_turns() {
    let samples = |n: usize| -> Vec<f64> { (0..=n).map(|i| i as f64/n as f64*10.).collect() };
    let read = |f: &dyn Fn(f64) -> f64,ts: &[f64]| -> Vec<f64> { ts.iter().map(|&t| f(t)).collect() };
    // a line tangent to a circle of radius 7 at t = 5: the distance off it, and its rate
    let (r,at) = (7.,5.);
    let f = |t: f64| (t-at).hypot(r)-r;
    let df = |t: f64| (t-at)/(t-at).hypot(r);
    // 23 steps: the touch midway between two samples of equal value
    let ts = samples(23);
    let roots = sampled_roots(f,df,&ts,&read(&f,&ts),1e-9,1e-15);
    assert_eq!(roots.len(),1,"{roots:?}");
    assert!(roots[0].touch && (roots[0].at-at).abs() <= 1e-12,"{roots:?}");
    // 20 steps: the touch on a sample, which stands there; 21: off every sample
    for n in [20,21] {
        let ts = samples(n);
        let roots = sampled_roots(f,df,&ts,&read(&f,&ts),1e-9,1e-15);
        assert!(roots.len() == 1 && roots[0].touch && (roots[0].at-at).abs() <= 1e-12,"{n}: {roots:?}");
    }
    // a sample within the tolerance of the touch but off it is moved to where it is
    let ts: Vec<f64> = vec![0.,2.,4.,at+1e-5,6.,8.];
    let roots = sampled_roots(f,df,&ts,&read(&f,&ts),1e-9,1e-15);
    assert!(roots.len() == 1 && roots[0].touch && (roots[0].at-at).abs() <= 1e-12,"{roots:?}");
}

/// A dip past zero between two samples is two crossings; one short of it, or a turn away from
/// zero, none; a crossing at a sample within the tolerance is refined between its neighbours.
#[test]
fn sampled_roots_read_a_turn_past_zero_and_a_crossing_at_a_sample() {
    let ts: Vec<f64> = (0..=10).map(|i| i as f64).collect();
    let read = |f: &dyn Fn(f64) -> f64| -> Vec<f64> { ts.iter().map(|&t| f(t)).collect() };
    // (t − 4.5)² − 0.01: crossings at 4.4 and 4.6, both between the samples 4 and 5
    let f = |t: f64| (t-4.5)*(t-4.5)-0.01;
    let df = |t: f64| 2.*(t-4.5);
    let roots = sampled_roots(f,df,&ts,&read(&f),1e-9,1e-15);
    assert_eq!(roots.len(),2,"{roots:?}");
    assert!((roots[0].at-4.4).abs() <= 1e-12 && (roots[1].at-4.6).abs() <= 1e-12 && !roots[0].touch && !roots[1].touch,"{roots:?}");
    // short of zero, and turning away from it: nothing
    let g = |t: f64| (t-4.5)*(t-4.5)+0.01;
    assert!(sampled_roots(g,df,&ts,&read(&g),1e-9,1e-15).is_empty());
    let h = |t: f64| 1.-(t-4.5)*(t-4.5)/100.;
    assert!(sampled_roots(h,|t| -(t-4.5)/50.,&ts,&read(&h),1e-9,1e-15).is_empty());
    // a line crossing at 3 + 1e-12, within the tolerance of the sample at 3
    let k = |t: f64| t-3.-1e-12;
    let roots = sampled_roots(k,|_| 1.,&ts,&read(&k),1e-9,1e-15);
    assert!(matches!(roots[..],[Root {at,touch:false}] if (at-3.-1e-12).abs() <= 1e-15),"{roots:?}");
}

#[test]
fn brent_stops_where_its_caller_says() {
    let calls = std::cell::Cell::new(0);
    let f = |x: f64| { calls.set(calls.get()+1); (x-0.25)*(x-0.25) };
    brent(&f,0.,1.,1e-14,80,|v,_| v < 1e-2);
    let early = calls.get();
    calls.set(0);
    brent(&f,0.,1.,1e-14,80,|_,_| false);
    assert!(early < calls.get(),"{early} against {}",calls.get());
    // the slack reported is infinite until a parabola holds water, then shrinks with the bracket
    let slacks = std::cell::RefCell::new(Vec::new());
    brent(&|x: f64| (x-0.25)*(x-0.25),0.,1.,1e-14,80,|_,slack| { slacks.borrow_mut().push(slack); false });
    let slacks = slacks.into_inner();
    assert!(slacks[0].is_infinite() && slacks.last().unwrap().is_finite(),"{slacks:?}");
}

#[test]
fn bracketed_root_keeps_its_bracket() {
    let x = bracketed_root(|x| Some(x*x*x-2.),0.,-2.,2.,6.,1e-14);
    assert!((x-2f64.cbrt()).abs() < 1e-12,"{x}");
    // a function the step leaves ends at the bracket's middle
    let x = bracketed_root(|_| None,0.,-1.,1.,1.,1e-14);
    assert_eq!(x,0.5);
}

#[test]
fn bisect_halves_scalars_and_points_alike() {
    let (lo,hi) = bisect(0.,1.,|a,b| 0.5*(a+b),|a,b| b-a > 1e-12,|m| m*m < 0.5);
    assert!(lo < 0.5f64.sqrt() && hi >= 0.5f64.sqrt() && hi-lo <= 1e-12,"{lo} {hi}");
    let mid = |a: [f64;3],b: [f64;3]| std::array::from_fn(|k| 0.5*(a[k]+b[k]));
    let (a,b) = bisect([0.;3],[2.,2.,2.],mid,|a,b| norm(sub(b,a)) > 1e-9,|m| m[0] < 1.3);
    assert!(a[0] < 1.3 && b[0] >= 1.3 && norm(sub(b,a)) <= 1e-9);
    // an already narrow bracket is not asked about at all
    let mut asked = 0;
    let _ = bisect(0.,1e-15,|a,b| 0.5*(a+b),|a,b| b-a > 1e-12,|_| { asked += 1; true });
    assert_eq!(asked,0);
}

#[test]
fn least_norm_step_satisfies_the_linearised_zero_sets_and_moves_least() {
    let (ga,gb) = ([1.,0.2,0.],[0.,1.,0.3]);
    let (va,vb) = (0.4,-0.7);
    let step = least_norm_step(&[ga,gb],&[va,vb]).unwrap();
    // J·step = F: the step subtracted puts both linearisations at zero
    assert!((dot(ga,step)-va).abs() < 1e-14 && (dot(gb,step)-vb).abs() < 1e-14);
    // least: it lies in the gradients' span, so has no part along their cross product
    assert!(dot(step,cross(ga,gb)).abs() < 1e-14);
    let rows = [[2.,0.,1.],[0.,1.,-1.],[1.,1.,1.]];
    let values = [1.,2.,3.];
    let step = least_norm_step(&rows,&values).unwrap();
    for k in 0..3 { assert!((dot(rows[k],step)-values[k]).abs() < 1e-12); }
    // dependent gradients fix no step
    assert!(least_norm_step(&[ga,ga.map(|x| 2.*x)],&[va,vb]).is_none());
    assert!(least_norm_step(&[ga,gb,[1.,1.2,0.3]],&[1.,2.,3.]).is_none());
    assert!(least_norm_step(&[ga],&[va]).is_none());
}

#[test]
fn newton_onto_two_spheres_lands_on_their_circle_and_three_on_their_point() {
    // two unit spheres a unit apart meet in a circle of radius √3/2 in the plane x = 1/2
    let sphere = |c: [f64;3]| move |p: [f64;3]| { let d = sub(p,c); let l = norm(d); (d.map(|x| x/l),l-1.) };
    let (a,b) = (sphere([0.;3]),sphere([1.,0.,0.]));
    let p = newton_onto([0.4,0.7,0.2],0.5,|p| Some([a(p),b(p)]),|at| at.iter().all(|r| r.1.abs() < 1e-13)).unwrap();
    assert!((p[0]-0.5).abs() < 1e-12 && ((p[1]*p[1]+p[2]*p[2]).sqrt()-0.75f64.sqrt()).abs() < 1e-12,"{p:?}");
    // a third sphere pins one point of that circle
    let c = sphere([0.,1.,0.]);
    let p = newton_onto([0.4,0.6,0.5],0.5,|p| Some([a(p),b(p),c(p)]),|at| at.iter().all(|r| r.1.abs() < 1e-13)).unwrap();
    for f in [&a,&b,&c] { assert!(f(p).1.abs() < 1e-13); }
    // spheres apart meet nowhere, and a reading that fails ends the search
    let far = sphere([3.,0.,0.]);
    assert!(newton_onto([1.,0.1,0.],0.5,|p| Some([a(p),far(p)]),|at| at.iter().all(|r| r.1.abs() < 1e-13)).is_none());
    assert!(newton_onto([0.;3],0.5,|_| None::<[([f64;3],f64);2]>,|_| true).is_none());
}
