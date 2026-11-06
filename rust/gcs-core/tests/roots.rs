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

/// `f` read at each of `ts`.
fn read(f: &dyn Fn(f64) -> f64,ts: &[f64]) -> Vec<f64> { ts.iter().map(|&t| f(t)).collect() }

/// **A touch is a root of the derivative** (`sampled_roots`): a double root of `f` placed where
/// `df` changes sign, to its rounding — a least `|f|` stands only to the square root of `f`'s — and
/// found between two samples level either side of it, where neither is a sampled minimum alone.
#[test]
fn sampled_roots_place_a_touch_where_the_derivative_turns() {
    let samples = |n: usize| -> Vec<f64> { (0..=n).map(|i| i as f64/n as f64*10.).collect() };
    // a line tangent to a circle of radius 7 at t = 5: the distance off it, and its rate
    let (r,at) = (7.,5.);
    let f = |t: f64| (t-at).hypot(r)-r;
    let df = |t: f64| (t-at)/(t-at).hypot(r);
    // 23 steps: the touch midway between two samples of equal value; 20: on a sample; 21: off
    // every sample; and a sample within the tolerance of the touch but off it, moved to where it is
    for ts in [samples(23),samples(20),samples(21),vec![0.,2.,4.,at+1e-5,6.,8.]] {
        let roots = sampled_roots(f,df,&ts,&read(&f,&ts),1e-9,1e-15);
        assert!(roots.len() == 1 && roots[0].touch && (roots[0].at-at).abs() <= 1e-12,"{ts:?}: {roots:?}");
    }
}

/// A dip past zero between two samples is two crossings; one short of it, or a turn away from
/// zero, none; a crossing at a sample within the tolerance is refined between its neighbours.
#[test]
fn sampled_roots_read_a_turn_past_zero_and_a_crossing_at_a_sample() {
    let ts: Vec<f64> = (0..=10).map(|i| i as f64).collect();
    // (t − 4.5)² − 0.01: crossings at 4.4 and 4.6, both between the samples 4 and 5
    let f = |t: f64| (t-4.5)*(t-4.5)-0.01;
    let df = |t: f64| 2.*(t-4.5);
    let roots = sampled_roots(f,df,&ts,&read(&f,&ts),1e-9,1e-15);
    assert_eq!(roots.len(),2,"{roots:?}");
    assert!((roots[0].at-4.4).abs() <= 1e-12 && (roots[1].at-4.6).abs() <= 1e-12 && !roots[0].touch && !roots[1].touch,"{roots:?}");
    // short of zero, and turning away from it: nothing
    let g = |t: f64| (t-4.5)*(t-4.5)+0.01;
    assert!(sampled_roots(g,df,&ts,&read(&g,&ts),1e-9,1e-15).is_empty());
    let h = |t: f64| 1.-(t-4.5)*(t-4.5)/100.;
    assert!(sampled_roots(h,|t| -(t-4.5)/50.,&ts,&read(&h,&ts),1e-9,1e-15).is_empty());
    // a line crossing at 3 + 1e-12, within the tolerance of the sample at 3
    let k = |t: f64| t-3.-1e-12;
    let roots = sampled_roots(k,|_| 1.,&ts,&read(&k,&ts),1e-9,1e-15);
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

/// **A root certified by Kantorovich's theorem is the root, however flat the system** (#780 R2,
/// `isolate::solve`): a system all but linear — its second derivatives' bound a rounding's, as a
/// line's against a B-spline sheet fitted flat is — has its one root found where it is. The ball
/// the theorem holds it in was once read as `(1 − √(1 − 2h))/βK`, which cancels to nothing when
/// `βK` is rounding's: Newton's first step left the empty ball, and the box's middle came back as
/// a crossing a long way off (a slid bar's edge met its plane at 7.86 rather than 9).
#[test]
fn a_root_certified_over_a_flat_system_is_where_it_is() {
    use gcs_core::isolate::solve;
    let root = [0.27,0.81,0.64];
    let a = [[2.,-0.3,0.1],[0.4,-1.,0.2],[0.,0.5,-3.]];
    let f = |x: [f64;3]| -> ([f64;3],[[f64;3];3]) {
        (std::array::from_fn(|i| (0..3).map(|k| a[i][k]*(x[k]-root[k])).sum()),a)
    };
    for bend in [0.,1e-30,1e-17,1e-9] {
        let found = solve::<3>(&f,&|_| [bend;3],[[0.,1.];3],1e-12,&|_| false).expect("a search that stops");
        assert_eq!(found.len(),1,"bent by {bend}: {found:?}");
        assert!(!found[0].touch && (0..3).all(|k| (found[0].at[k]-root[k]).abs() <= 1e-12),"bent by {bend}: {found:?}");
    }
}

/// **A crossing on the face two halves of a box share is found, once** (#796, `isolate::solve`):
/// a line against a bilinear sheet (the wall of brep-bench's saddle drum, `y = 10` between the
/// floor `z = −5` and the saddle `z = 10 + xy/8`), read over stretches centred on where it crosses,
/// so the crossing stands on the face the search halves its first box along. Read as halves
/// meeting at that face, it stood on the edge of each: Kantorovich's ball about the middle reached
/// a hair past the face, so neither half certified it, and each was halved again with the crossing
/// still on its face until rounding put it a hair outside both and both were dropped — a ray from
/// inside the saddle block missed the wall, and the block's inside read as out. Each box is read
/// grown past its sides, so the crossing is inside both halves, and the two finding it are one.
#[test]
fn a_crossing_on_the_face_two_halves_share_is_found_once() {
    use gcs_core::brep::geom::{Curve,Frame,Surface};
    use gcs_core::brep::nurbs::Net;
    use gcs_core::brep::query::{curve_surface,Meets};
    let saddle = |x: f64,y: f64| 10.+x*y/8.;
    let l = vec![0.,0.,1.,1.];
    let wall = Net {du:1,dv:1,uknots:l.clone(),vknots:l,
        poles:vec![vec![[-10.,10.,-5.],[10.,10.,-5.]],vec![[-10.,10.,saddle(-10.,10.)],[10.,10.,saddle(10.,10.)]]],weights:None};
    let wall = Surface::BSpline(Frame::WORLD,std::sync::Arc::new(wall));
    let (mut failed,mut read) = (Vec::new(),0);
    for p in [[3.,3.,0.],[-9.,-9.,-1.],[0.,0.,5.],[-4.,2.,8.],[6.,-7.,-3.]] {
        for d in [[0.5773,0.6124,0.5402],[-0.3172,0.8123,-0.4893],[0.2,0.9,0.1],[0.05,1.,-0.3]] {
            let d = gcs_core::space::normalised(d).unwrap();
            let t = (10.-p[1])/d[1];
            let q = [p[0]+d[0]*t,10.,p[2]+d[2]*t];
            if q[0].abs() >= 10. || q[2] <= -5. || q[2] >= saddle(q[0],10.) { continue }
            for h in [3.26763806389766,1.,0.1,7.3] {
                read += 1;
                match curve_surface(&Curve::Line {p,d},[t-h,t+h],&wall,4e-11) {
                    Meets::At(r) if r.len() == 1 && !r[0].1 && (r[0].0-t).abs() <= 1e-9 => {}
                    m => failed.push(format!("from {p:?} along {d:?}, {h} either side of {t}: {m:?}")),
                }
            }
        }
    }
    assert!(read > 20 && failed.is_empty(),"{} of {read}:\n{}",failed.len(),failed.join("\n"));
}

/// **A crossing beside a sheet's kink is read on the sheet** (#812, `query::curve_sheet`): a flat
/// cubic sheet in two pieces meeting at a knot of the degree's multiplicity, off the middle of its
/// domain (where the search's first halving would part it), `x` running a fifth as fast past it as
/// before, so `S_u` jumps there (a gear's flank fitted patch by patch). Across the kink no bound
/// on the second derivatives holds, but the differences' net put the jump's at nought, and a box
/// straddling it was read by the slope at its middle alone: here a crossing a box held was dropped
/// as out of reach; on a 64-tooth gear's flank one was certified, Newton's step from the middle
/// left the theorem's ball and was returned where it stopped, 1e-3 off the flank — either way a
/// crossing lost, and the gear's material cell read as out. The bound refuses a box across a kink,
/// the sheet is searched kink to kink, and each crossing is where it is, by rays from either side.
#[test]
fn a_crossing_beside_a_sheets_kink_is_read_on_the_sheet() {
    use gcs_core::brep::geom::{Curve,Frame,Surface};
    use gcs_core::brep::nurbs::Net;
    use gcs_core::brep::query::{curve_surface,Meets};
    // x from 0 to 4 over u in [0, 0.7], on to 5 over [0.7, 2], each piece in step with u (its poles
    // evenly along it); y = 2v − 1; in the plane z = 0
    let xs = [0.,4./3.,8./3.,4.,13./3.,14./3.,5.];
    let net = Net {du:3,dv:1,uknots:vec![0.,0.,0.,0.,0.7,0.7,0.7,2.,2.,2.,2.],vknots:vec![0.,0.,1.,1.],
        poles:xs.iter().map(|&x| vec![[x,-1.,0.],[x,1.,0.]]).collect(),weights:None};
    net.check().unwrap();
    assert_eq!(net.kinks(0),vec![0.7]);
    assert!(net.kinks(1).is_empty());
    assert_eq!(net.second_bounds([0.5,1.5],[0.,1.]),None,"no bound across the kink");
    let flat = net.second_bounds([0.,0.7],[0.,1.]).expect("a bound up to it");
    assert!(flat.iter().all(|&b| b <= 1e-12),"flat up to it: {flat:?}");
    let sheet = Surface::BSpline(Frame::WORLD,std::sync::Arc::new(net));
    let (mut failed,mut read) = (Vec::new(),0);
    for x in [4.3,4.05,4.0004,3.99,3.9,2.,4.8] {
        for d in [[0.5773,0.6124,0.5402],[-0.3172,0.8123,0.4893],[0.1,-0.2,0.97]] {
            let d = gcs_core::space::normalised(d).unwrap();
            // from a point under the sheet, a long way back along the ray
            let t = 37.;
            let p = [x-d[0]*t,0.2-d[1]*t,-d[2]*t];
            for span in [[0.,200.],[t-1.,t+1.],[0.,t+0.5]] {
                read += 1;
                match curve_surface(&Curve::Line {p,d},span,&sheet,1e-9) {
                    Meets::At(r) if r.len() == 1 && !r[0].1 && (r[0].0-t).abs() <= 1e-9 => {}
                    m => failed.push(format!("crossing at x = {x} along {d:?} over {span:?}: {m:?}")),
                }
            }
        }
    }
    assert!(failed.is_empty(),"{} of {read}:\n{}",failed.len(),failed.join("\n"));
}

/// **A double root a stretch ends on is found** (#780 R2, `isolate::roots`): `g = −A(1 + ε) − A
/// sin t` over `[0, 2π]`, its turn at `3π/2` a hair below nought — the middle of the stretch's
/// second half, so halving leaves it at the end of the stretches either side. There `|g'(m)|` is
/// its bound `A h` to rounding (`sin h` against `h`), and claimed monotone by rounding, both ends
/// below nought, each side dropped it: a cone's rim circle tangent to a slid frustum's side, met
/// nowhere, built the frustum's sweep wrong. A touch, once, at the turn.
#[test]
fn a_double_root_a_stretch_ends_on_is_found() {
    use gcs_core::isolate::roots;
    use std::f64::consts::{PI,TAU};
    for amp in [1.8973665961010275,1.,3.7,0.25] {
        for eps in [4e-16,1e-15,1e-13,1e-10] {
            let c = -amp*(1.+eps);
            let g = |t: f64| c-amp*t.sin();
            let dg = |t: f64| -amp*t.cos();
            let found = roots(&g,&dg,&|_,_| [amp,amp],[0.,TAU],1e-12,&|t| g(t).abs() <= 1e-9).expect("a search that stops");
            assert_eq!(found.len(),1,"{amp}, {eps}: {found:?}");
            assert!(found[0].touch && (found[0].at-1.5*PI).abs() <= 1e-4,"{amp}, {eps}: {found:?}");
        }
    }
}

/// **A search says why it gave up** (#780 R3, `isolate::GaveUp`): one that cannot part a stretch
/// where the function stays nought reads past `MOST` stretches and gives up as `Most`, where its
/// caller reads the pair another way; one under a build already past its budget gives up at its
/// first stretch or box as `Stopped`, where its caller reads nothing more. Without either it finds
/// its root.
#[test]
fn a_search_says_why_it_gave_up() {
    use gcs_core::isolate::{roots,solve,GaveUp};
    use gcs_core::meter::{self as meter,Budget,Meter,Stage};
    let flat = roots(&|_| 0.,&|_| 0.,&|_,_| [1.,1.],[0.,1.],1e-15,&|_| false);
    assert!(matches!(flat,Err(GaveUp::Most)),"{flat:?}");
    let g = |t: f64| t-0.5;
    let line = |x: [f64;1]| ([x[0]-0.5],[[1.]]);
    let found = roots(&g,&|_| 1.,&|_,_| [1.,0.],[0.,1.],1e-12,&|_| false).expect("a search that stops");
    assert!(found.len() == 1 && (found[0].at-0.5).abs() <= 1e-12,"{found:?}");
    let m = Meter::new(Budget::UNLIMITED.with(Stage::Ray,0));
    let stopped = meter::within(&m,|| roots(&g,&|_| 1.,&|_,_| [1.,0.],[0.,1.],1e-12,&|_| false));
    assert!(matches!(stopped,Err(GaveUp::Stopped)),"{stopped:?}");
    let stopped = meter::within(&m,|| solve::<1>(&line,&|_| [0.],[[0.,1.]],1e-12,&|_| false));
    assert!(matches!(stopped,Err(GaveUp::Stopped)),"{stopped:?}");
    assert_eq!(m.count(Stage::Ray),2,"one stretch and one box read");
}
