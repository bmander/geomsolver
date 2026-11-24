//! **The folded patch fuzzed** (#1013 T1): a block under a patch folded back over its own rim,
//! cut by a box, at random. The patch is `(s², v, h + a s⁴ + b v)`, `s = 2u − 1`, given exactly (its
//! layers `u ≥ 1/2` and `u ≤ 1/2` one graph, `z = h + a x² + b y`); the face is `[u0, 1] × [v0, v1]`
//! of it, on the whole patch; the box `[x0, x1] × [y0, y1] × [−1, 3]`, its walls placed anywhere
//! across the face, its rim and the fold. Each of the three Booleans is held to the closed form
//! (`|A ∩ B|` the polynomial's integral over the rectangles' common part) and `check`ed, and
//! timed; a failure is said with the case's numbers, the same case on the patch cut down to the
//! face beside it.
//!
//! `cargo run --release -p cad-kernel --example patch_fuzz -- [cases] [seed]`
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
        let zs: Vec<f64> = [1.,-1.,1.,-1.,1.].iter().map(|k| h+a*k).collect();
        let patch = Net {du:4,dv:1,uknots:vec![0.,0.,0.,0.,0.,1.,1.,1.,1.,1.],vknots:vec![0.,0.,1.,1.],weights:None,
            poles:(0..5).map(|i| { let x = [1.,0.,-1./3.,0.,1.][i]; vec![[x,0.,zs[i]],[x,1.,zs[i]+b]] }).collect()};
        // the block's volume over a rectangle of the plane: ∫∫ (h + a x² + b y)
        let over = |xa: f64,xb: f64,ya: f64,yb: f64| if xb <= xa || yb <= ya { 0. } else {
            (yb-ya)*(h*(xb-xa)+a*(xb.powi(3)-xa.powi(3))/3.)+b*(xb-xa)*(yb*yb-ya*ya)/2. };
        let va = over(xr,1.,v0,v1);
        let common = over(xr.max(x0),x1.min(1.),v0.max(y0),v1.min(y1));
        let vb = (x1-x0)*(y1-y0)*4.;
        let blk = cad_kernel::build::block([x0,y0,-1.],[x1,y1,3.]).unwrap();
        let said = format!("case {case}: h {h:.4} a {a:.4} b {b:.4} u0 {u0:.4} v [{v0:.4}, {v1:.4}] box x [{x0:.4}, {x1:.4}] y [{y0:.4}, {y1:.4}]");
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
