//! **The folded patch fuzzed** (#1013 T1, T2): a block under a patch folded back over its own rim,
//! cut by a box, at random. The patch is `(s², v, h + a s⁴ + c s³ + b v)`, `s = 2u − 1`, given
//! exactly: half the cases `c = 0`, its layers `u ≥ 1/2` and `u ≤ 1/2` one graph, `z = h + a x² + b y`
//! (T1's); half `c` from 1e-5 to 3e-2, the layer `u ≤ 1/2` standing `2c x^{3/2}` beneath the face
//! (T2's); the face is `[u0, 1] × [v0, v1]`
//! of it, on the whole patch; the box `[x0, x1] × [y0, y1] × [−1, 3]`, its walls placed anywhere
//! across the face, its rim and the fold. Each of the three Booleans is held to the closed form
//! (`|A ∩ B|` the polynomial's integral over the rectangles' common part) and `check`ed, and
//! timed; a failure is said with the case's numbers, the same case on the patch cut down to the
//! face beside it.
//!
//! `cusp` (#1013 T3): the cuspidal patch `(s², v, h + k s³ + b v)`, its face `[u0, 1] × [v0, v1]`
//! trimmed as near as `u0 = 0.505` to the cusp `u = 1/2`; `pinch` (T4): `(s³, t³, h + st)`, its
//! face `[u0, 1] × [v0, 1]` as near as `0.505` to the pinch each way (half of them within 0.03).
//! Each is cut by a plane at random through a point within 0.003 of the cusp's line or of the pinch
//! (half the pinch's within 5e-4, and half its planes nearly upright), kept on each side
//! (`build::kept`): each side held to the same block on the patch cut down to its face (the
//! gates' control), and the two to `|A|`'s closed form.
//!
//! `cargo run --release -p cad-kernel --example patch_fuzz -- [cases] [seed] [fold|cusp|pinch]`
use cad_kernel::boolean::{boolean,Op};
use cad_kernel::nurbs::Net;
use cad_kernel::util::rng::Rng;
use std::time::Instant;
#[path = "support/patch.rs"]
mod patch;
use patch::under;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cases: usize = args.first().and_then(|s| s.parse().ok()).unwrap_or(50);
    let seed: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1013);
    if let Some(mode @ ("cusp"|"pinch")) = args.get(2).map(String::as_str) { return sliced(mode,cases,seed) }
    // (`CASE=n`: that case alone, the others drawn and passed over; `WHOLE`: the whole patch alone,
    // not the cut-down one beside it; `OP=name`: that Boolean alone)
    let only_case: Option<usize> = std::env::var("CASE").ok().and_then(|c| c.parse().ok());
    let whole_only = std::env::var_os("WHOLE").is_some();
    let only_op = std::env::var("OP").ok();
    let tol = 1e-7;
    let mut rng = Rng::new(seed);
    let (mut failed,mut slowest,mut total) = (0usize,(0.,0usize),0.);
    for case in 0..cases {
        let (h,a,b) = (rng.uniform(0.3,0.7),rng.uniform(0.1,0.5),rng.uniform(-0.2,0.2));
        // (half the patches covered twice exactly, half with their folded layer `2c x^{3/2}` beneath
        // the face, `c` from 1e-5 to 3e-2: T2's)
        let c = if rng.next() < 0.5 { 0. } else { 10f64.powf(rng.uniform(-5.,-1.5)) };
        let u0 = rng.uniform(0.53,0.85);
        let (v0,v1) = { let p = rng.uniform(0.,0.3); (p,rng.uniform(p+0.4,1.)) };
        let xr = (2.*u0-1.).powi(2);
        // the box: each wall somewhere from past the fold to past the face's far edge, or across
        // the face; the rim x = xr crossed by a wall along y often
        let x0 = if rng.next() < 0.5 { rng.uniform(-0.5,xr) } else { rng.uniform(xr,0.8) };
        let x1 = rng.uniform(x0+0.05,1.3);
        let y0 = rng.uniform(v0-0.4,v1-0.1);
        let y1 = rng.uniform(y0+0.05,v1+0.4);
        if only_case.is_some_and(|c| c != case) { continue }
        let zs: Vec<f64> = (0..5).map(|i| h+a*[1.,-1.,1.,-1.,1.][i]+c*[-1.,0.5,0.,-0.5,1.][i]).collect();
        let patch = Net {du:4,dv:1,uknots:vec![0.,0.,0.,0.,0.,1.,1.,1.,1.,1.],vknots:vec![0.,0.,1.,1.],weights:None,
            poles:(0..5).map(|i| { let x = [1.,0.,-1./3.,0.,1.][i]; vec![[x,0.,zs[i]],[x,1.,zs[i]+b]] }).collect()};
        // the block's volume over a rectangle of the plane: ∫∫ (h + a x² + c x^{3/2} + b y)
        let over = |xa: f64,xb: f64,ya: f64,yb: f64| if xb <= xa || yb <= ya { 0. } else {
            (yb-ya)*(h*(xb-xa)+a*(xb.powi(3)-xa.powi(3))/3.+0.4*c*(xb.powf(2.5)-xa.powf(2.5)))+b*(xb-xa)*(yb*yb-ya*ya)/2. };
        let va = over(xr,1.,v0,v1);
        let common = over(xr.max(x0),x1.min(1.),v0.max(y0),v1.min(y1));
        let vb = (x1-x0)*(y1-y0)*4.;
        let blk = cad_kernel::build::block([x0,y0,-1.],[x1,y1,3.]).unwrap();
        let said = format!("case {case}: h {h:.4} a {a:.4} b {b:.4} c {c:.2e} u0 {u0:.4} v [{v0:.4}, {v1:.4}] box x [{x0:.4}, {x1:.4}] y [{y0:.4}, {y1:.4}]");
        for whole in [true,false] {
            if !whole && whole_only { continue }
            let s = under(&patch,[u0,1.],[v0,v1],whole);
            for (name,op,want) in [("union",Op::Union,va+vb-common),("difference",Op::Cut,va-common),("intersection",Op::Common,common)] {
                if only_op.as_deref().is_some_and(|o| o != name) { continue }
                let clock = Instant::now();
                let r = boolean(&s,&blk,op,tol);
                let took = clock.elapsed().as_secs_f64();
                if whole { total += took; if took > slowest.0 { slowest = (took,case) } }
                let bad = match &r {
                    Ok(r) => { let v = cad_kernel::props::volume(r);
                        if (v-want).abs() > 1e-9*want.max(1e-3) { Some(format!("volume {v:.12} want {want:.12}")) }
                        else { r.check(tol).err().map(|e| format!("check: {e}")) } }
                    Err(e) => Some(format!("refused: {e}")),
                };
                if only_case.is_some() { println!("{} {name}: {took:.3} s",if whole { "whole" } else { "cut down" }) }
                if let Some(why) = bad {
                    if whole { failed += 1 }
                    println!("{} {said} {name}: {why} ({took:.2} s)",if whole { "FAIL" } else { "     (cut down)" });
                }
            }
        }
    }
    println!("{failed} of {} Booleans failed on the whole patch; {:.2} s each on average, slowest {:.2} s (case {})",3*cases,total/(3*cases) as f64,slowest.0,slowest.1);
}

/// The `cusp` and `pinch` modes (`main`'s doc).
fn sliced(mode: &str,cases: usize,seed: u32) {
    let only_case: Option<usize> = std::env::var("CASE").ok().and_then(|c| c.parse().ok());
    let tol = 1e-7;
    let mut rng = Rng::new(seed);
    let (mut failed,mut slowest,mut total,mut runs) = (0usize,(0.,0usize),0.,0usize);
    for case in 0..cases {
        let h = rng.uniform(0.3,0.7);
        let (patch,us,vs,va,singular,off) = if mode == "cusp" {
            let (k,b) = (rng.uniform(0.3,1.5),rng.uniform(-0.2,0.2));
            let u0 = rng.uniform(0.505,0.8);
            let (v0,v1) = { let p = rng.uniform(0.,0.3); (p,rng.uniform(p+0.4,1.)) };
            let x0 = (2.*u0-1.).powi(2);
            let poles = [(1.,-1.),(-1./3.,1.),(-1./3.,-1.),(1.,1.)].iter()
                .map(|&(x,z): &(f64,f64)| vec![[x,0.,h+k*z],[x,1.,h+k*z+b]]).collect();
            let net = Net {du:3,dv:1,uknots:vec![0.,0.,0.,0.,1.,1.,1.,1.],vknots:vec![0.,0.,1.,1.],weights:None,poles};
            // |A| = ∫∫ (h + k x^{3/2} + b y) over x0 ≤ x ≤ 1, v0 ≤ y ≤ v1; the cusp's line (0, y, h + b y)
            let va = (v1-v0)*(h*(1.-x0)+0.4*k*(1.-x0.powf(2.5)))+b*(1.-x0)*(v1*v1-v0*v0)/2.;
            let y = rng.uniform(v0,v1);
            (net,[u0,1.],[v0,v1],va,[0.,y,h+b*y],3e-3)
        } else {
            // (half the faces trimmed within 0.03 of the pinch each way, as T4's nearer gate)
            let near = rng.next() < 0.5;
            let (u0,v0) = if near { (rng.uniform(0.505,0.53),rng.uniform(0.505,0.53)) } else { (rng.uniform(0.505,0.7),rng.uniform(0.505,0.7)) };
            let (x,s) = ([-1.,1.,-1.,1.],[-1.,-1./3.,1./3.,1.]);
            let poles = (0..4).map(|i| (0..4).map(|j| [x[i],x[j],h+s[i]*s[j]]).collect()).collect();
            let net = Net {du:3,dv:3,uknots:vec![0.,0.,0.,0.,1.,1.,1.,1.],vknots:vec![0.,0.,0.,0.,1.,1.,1.,1.],weights:None,poles};
            let (x0,y0) = ((2.*u0-1.).powi(3),(2.*v0-1.).powi(3));
            // |A| = ∫∫ (h + ∛x ∛y) over x0 ≤ x ≤ 1, y0 ≤ y ≤ 1
            let va = h*(1.-x0)*(1.-y0)+0.5625*(1.-x0.powf(4./3.))*(1.-y0.powf(4./3.));
            (net,[u0,1.],[v0,1.],va,[0.,0.,h],if near { 5e-4 } else { 3e-3 })
        };
        // a plane through a point within 0.003 of the singularity, its normal at random but not
        // level (the floor's), each way kept
        let o = [singular[0]+rng.uniform(-off,off),singular[1]+rng.uniform(-off,off),singular[2]+rng.uniform(-off,off)];
        // (beside a pinch, half the planes nearly upright through it, `y ≈ a x`, as T4's gates)
        let n = if mode == "pinch" && rng.next() < 0.5 {
            let n = [-rng.uniform(0.5,3.),1.,rng.uniform(0.,0.1)];
            let l = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
            n.map(|x| x/l)
        } else { loop {
            let n = [rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)];
            let l = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
            if l > 0.2 && n[2].abs() < 0.9*l { break n.map(|x| x/l) }
        } };
        if only_case.is_some_and(|c| c != case) { continue }
        let said = format!("case {case}: h {h:.4} face u {us:?} v {vs:?} plane through {o:?} normal {n:?}");
        let control = under(&patch,us,vs,false);
        let whole = under(&patch,us,vs,true);
        let mut sides = [0.;2];
        for (k,keep) in [n,n.map(|x| -x)].into_iter().enumerate() {
            let want = cad_kernel::build::kept(&control,o,keep,0.).map(|r| cad_kernel::props::volume(&r));
            let clock = Instant::now();
            let r = cad_kernel::build::kept(&whole,o,keep,0.);
            let took = clock.elapsed().as_secs_f64();
            total += took; runs += 1;
            if took > slowest.0 { slowest = (took,case) }
            let bad = match (&r,&want) {
                (_,Err(e)) => { println!("     (cut down) {said} side {k}: refused: {e}"); None }
                (Ok(r),Ok(want)) => { let v = cad_kernel::props::volume(r); sides[k] = v;
                    // (to a billionth of the block's: a sliver a corner of the plane cuts off holds a
                    // thousandth of it or less, and both readings fit their traced edges to the
                    // tolerance)
                    if (v-want).abs() > 1e-9*va { Some(format!("volume {v:.12} want {want:.12}")) }
                    else { r.check(tol).err().map(|e| format!("check: {e}")) } }
                (Err(e),_) => Some(format!("refused: {e}")),
            };
            if only_case.is_some() { println!("side {k}: {took:.3} s") }
            if let Some(why) = bad { failed += 1; println!("FAIL {said} side {k}: {why} ({took:.2} s)") }
        }
        if sides.iter().all(|&v| v > 0.) && (sides[0]+sides[1]-va).abs() > 1e-9*va {
            failed += 1;
            println!("FAIL {said}: the sides {:.12} + {:.12} against |A| {va:.12}",sides[0],sides[1]);
        }
    }
    println!("{failed} of {runs} cuts failed on the whole patch; {:.3} s each on average, slowest {:.2} s (case {})",total/runs.max(1) as f64,slowest.0,slowest.1);
}
