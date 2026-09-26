//! The searches of one variable and the Newton step onto several zero sets (`roots.rs`), which the
//! field's readings, its creases and the refinement's crossings share: each against a function
//! whose answer is known in closed form.
use gcs_core::roots::{bisect,bracketed_root,brent,least_norm_step,newton_onto};
use gcs_core::space::{dot,cross,norm,sub};

#[test]
fn brent_finds_the_minimum_of_a_parabola_and_of_a_quartic() {
    let (v,x) = brent(&|x: f64| (x-0.3)*(x-0.3)+2.,-1.,1.,1e-12,80,|_,_| false);
    assert!((x-0.3).abs() < 1e-8 && (v-2.).abs() < 1e-15,"{x} {v}");
    // a flat-bottomed minimum, where parabolic steps are least trusted
    let (_,x) = brent(&|x: f64| (x-0.7).powi(4),0.,1.,1e-12,200,|_,_| false);
    assert!((x-0.7).abs() < 1e-3,"{x}");
    // a minimum at the bracket's end is approached, never passed
    let (_,x) = brent(&|x: f64| x,0.,1.,1e-10,80,|_,_| false);
    assert!(x >= 0. && x < 1e-8,"{x}");
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
