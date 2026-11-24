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
use cad_kernel::geom::{Surface,V};
use cad_kernel::nurbs::Net;
use cad_kernel::topo::Brep;
use std::sync::Arc;
use std::time::Instant;

/// The block on `z = 0` under the part `us × vs` of `patch`, its top on the whole patch (`whole`)
/// or on the part.
fn under(patch: &Net,us: [f64;2],vs: [f64;2],whole: bool) -> Brep {
    let top = patch.segment(us,vs);
    let floored = |r: &Vec<V>| r.iter().map(|p| [p[0],p[1],0.]).collect();
    let floor = Net {poles:top.poles.iter().map(floored).collect(),..top.clone()};
    let wall = |pts: Vec<V>,knots: &[f64],d: usize| Net {du:d,dv:1,uknots:knots.to_vec(),
        vknots:vec![0.,0.,1.,1.],poles:pts.iter().map(|p| vec![[p[0],p[1],0.],*p]).collect(),weights:None};
    let (last,lastv) = (top.poles.len()-1,top.poles[0].len()-1);
    let walls = [
        wall(top.poles[0].clone(),&top.vknots,top.dv),
        wall(top.poles[last].clone(),&top.vknots,top.dv),
        wall(top.poles.iter().map(|r| r[0]).collect(),&top.uknots,top.du),
        wall(top.poles.iter().map(|r| r[lastv]).collect(),&top.uknots,top.du),
    ];
    let mut pieces = vec![cad_kernel::build::sheet(top.clone()).unwrap(),cad_kernel::build::sheet(floor).unwrap()];
    pieces[0].faces[0].name = "top".into();
    pieces.extend(walls.into_iter().map(|w| cad_kernel::build::sheet(w).unwrap()));
    let mut b = cad_kernel::sew::sewn(&pieces,1e-9).unwrap();
    if whole {
        let f = b.faces.iter_mut().find(|f| f.name == "top").unwrap();
        let Surface::BSpline(frame,_) = f.surface else { unreachable!() };
        f.surface = Surface::BSpline(frame,Arc::new(patch.clone()));
    }
    b
}

/// A small deterministic generator (xorshift), so a case is named by its seed.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 { self.0 ^= self.0 << 13; self.0 ^= self.0 >> 7; self.0 ^= self.0 << 17; (self.0 >> 11) as f64/(1u64 << 53) as f64 }
    fn in_(&mut self,a: f64,b: f64) -> f64 { a+(b-a)*self.next() }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cases: usize = args.first().and_then(|s| s.parse().ok()).unwrap_or(50);
    let seed: u64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1013);
    let tol = 1e-7;
    let mut rng = Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) | 1);
    let (mut failed,mut slowest,mut total) = (0usize,(0.,0usize),0.);
    for case in 0..cases {
        let (h,a,b) = (rng.in_(0.3,0.7),rng.in_(0.1,0.5),rng.in_(-0.2,0.2));
        let u0 = rng.in_(0.53,0.85);
        let (v0,v1) = { let p = rng.in_(0.,0.3); (p,rng.in_(p+0.4,1.)) };
        let xr = (2.*u0-1.).powi(2);
        // the box: each wall somewhere from past the fold to past the face's far edge, or across
        // the face; the rim x = xr crossed by a wall along y often
        let x0 = if rng.next() < 0.5 { rng.in_(-0.5,xr) } else { rng.in_(xr,0.8) };
        let x1 = rng.in_(x0+0.05,1.3);
        let y0 = rng.in_(v0-0.4,v1-0.1);
        let y1 = rng.in_(y0+0.05,v1+0.4);
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
        // (`CASE=n`: that case alone, the others drawn and passed over)
        if std::env::var("CASE").ok().and_then(|c| c.parse::<usize>().ok()).is_some_and(|c| c != case) { continue }
        let said = format!("case {case}: h {h:.4} a {a:.4} b {b:.4} u0 {u0:.4} v [{v0:.4}, {v1:.4}] box x [{x0:.4}, {x1:.4}] y [{y0:.4}, {y1:.4}]");
        // (`WHOLE`: the whole patch alone, not the cut-down one beside it)
        for whole in [true,false] {
            if !whole && std::env::var_os("WHOLE").is_some() { continue }
            let s = under(&patch,[u0,1.],[v0,v1],whole);
            for (name,op,want) in [("union",Op::Union,va+vb-common),("difference",Op::Cut,va-common),("intersection",Op::Common,common)] {
                // (`OP=name`: that one alone)
                if std::env::var("OP").ok().is_some_and(|o| o != name) { continue }
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
                if std::env::var_os("CASE").is_some() { println!("{} {name}: {took:.3} s",if whole { "whole" } else { "cut down" }) }
                if let Some(why) = bad {
                    if whole { failed += 1 }
                    println!("{} {said} {name}{}: {why} ({took:.2} s)",if whole { "FAIL" } else { "     (cut down)" },if whole { "" } else { "" });
                }
            }
        }
    }
    println!("{failed} of {} Booleans failed on the whole patch; {:.2} s each on average, slowest {:.2} s (case {})",3*cases,total/(3*cases) as f64,slowest.0,slowest.1);
}
