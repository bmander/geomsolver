//! **A block under a patch singular outside its face, cut** (#1013): the probe the gates of
//! `gcs-core/tests/patch_layers.rs` were found with. The block stands on `z = 0` under the part
//! `[u0, u1] × [v0, v1]` of a Bézier patch, its top face on the whole patch, trimmed (`PART`: on
//! the patch cut down to that part instead, the same face); it is cut by the box
//! `[bx0, bx1] × [by0, by1] × [−1, 3]` by all three Booleans (`OP=union|difference|intersection`
//! for one), each one's volume against the closed form (Gauss–Legendre), its area, `check`, time.
//!
//! `cargo run --release -p cad-kernel --example patch_probe -- NET u0 u1 v0 v1 bx0 bx1 by0 by1 [c]`
//!
//! NET: `twice` `(s², v, 1/2 + 3s⁴/10 + v/10 + c s³)`, `s = 2u − 1` (folded over its rim; `c` sets
//! its layers apart); `layers` `(s², v, 1/2 + v/10 + c s³)`; `cusp` `(s², v, 1/2 + s³ + v/10)`;
//! `point` `(s³, t³, LIFT + st)`, `t = 2v − 1` (singular at `u = v = 1/2`; `LIFT`, 1/2 unless given).
//! `BPLANE=ox,oy,oz,nx,ny,nz`: the slab under the plane through `o` with upward normal `n` in
//! place of the box; with `KEPT`, the block kept below the plane by `build::kept` instead, on the
//! whole patch and on its part (a unit `n`). `SOLVENT_BREP_DEBUG` says the Boolean's work.
use cad_kernel::boolean::{boolean,Op};
use cad_kernel::geom::V;
use cad_kernel::nurbs::Net;
use cad_kernel::util::quadrature::GAUSS8;
use std::time::Instant;
#[path = "support/patch.rs"]
mod patch;
use patch::under;

/// The Bézier net of degree (du, dv) over [0, 1]² interpolating `f` (a polynomial map of those
/// degrees, so exactly).
fn bezier(f: &dyn Fn(f64,f64) -> V,du: usize,dv: usize) -> Net {
    let bern = |d: usize,k: usize,t: f64| { let c = (1..=k).fold(1.,|a,i| a*(d+1-i) as f64/i as f64); c*t.powi(k as i32)*(1.-t).powi((d-k) as i32) };
    let n = (du+1)*(dv+1);
    let pts: Vec<(f64,f64)> = (0..=du).flat_map(|i| (0..=dv).map(move |j| (i as f64/du as f64,j as f64/dv as f64))).collect();
    let mut a = vec![vec![0.;n+3];n];
    for (r,&(u,v)) in pts.iter().enumerate() {
        for i in 0..=du { for j in 0..=dv { a[r][i*(dv+1)+j] = bern(du,i,u)*bern(dv,j,v) } }
        let p = f(u,v);
        for k in 0..3 { a[r][n+k] = p[k] }
    }
    for c in 0..n {
        let p = (c..n).max_by(|&x,&y| a[x][c].abs().total_cmp(&a[y][c].abs())).unwrap();
        a.swap(c,p);
        for r in 0..n { if r != c { let m = a[r][c]/a[c][c]; for k in c..n+3 { a[r][k] -= m*a[c][k] } } }
    }
    let poles = (0..=du).map(|i| (0..=dv).map(|j| { let r = i*(dv+1)+j; [a[r][n]/a[r][r],a[r][n+1]/a[r][r],a[r][n+2]/a[r][r]] }).collect()).collect();
    let clamped = |d: usize| [vec![0.;d+1],vec![1.;d+1]].concat();
    Net {du,dv,uknots:clamped(du),vknots:clamped(dv),poles,weights:None}
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let x = |i: usize| a[i].parse::<f64>().unwrap();
    let c = a.get(9).map_or(0.,|s| s.parse().unwrap());
    let (full,z): (Net,Box<dyn Fn(f64,f64) -> f64>) = match a[0].as_str() {
        // (its poles exactly: x the Bernstein form of s², z of 1/2 + 3s⁴/10 + c s³, v/10 added at v = 1)
        "twice" => (Net {du:4,dv:1,uknots:vec![0.,0.,0.,0.,0.,1.,1.,1.,1.,1.],vknots:vec![0.,0.,1.,1.],weights:None,
            poles:(0..5).map(|i| { let (x,z) = ([1.,0.,-1./3.,0.,1.][i],0.5+0.3*[1.,-1.,1.,-1.,1.][i]+c*[-1.,0.5,0.,-0.5,1.][i]);
                vec![[x,0.,z],[x,1.,z+0.1]] }).collect()},
            Box::new(move |x: f64,y: f64| 0.3*x*x+0.1*y+c*x.powf(1.5)+0.5)),
        "layers" => (bezier(&|u,v| { let s = 2.*u-1.; [s*s,v,0.5+0.1*v+c*s.powi(3)] },3,1),Box::new(move |x: f64,y: f64| 0.5+0.1*y+c*x.powf(1.5))),
        "cusp" => (Net {du:3,dv:1,uknots:vec![0.,0.,0.,0.,1.,1.,1.,1.],vknots:vec![0.,0.,1.,1.],weights:None,
            poles:[(1.,-0.5),(-1./3.,1.5),(-1./3.,-0.5),(1.,1.5)].iter().map(|&(x,z)| vec![[x,0.,z],[x,1.,z+0.1]]).collect()},
            Box::new(|x: f64,y: f64| x.powf(1.5)+0.1*y+0.5)),
        "point" => { let lift: f64 = std::env::var("LIFT").ok().map_or(0.5,|l| l.parse().unwrap());
            (bezier(&|u,v| { let (s,t) = (2.*u-1.,2.*v-1.); [s.powi(3),t.powi(3),s*t+lift] },3,3),
            Box::new(move |x: f64,y: f64| x.cbrt()*y.cbrt()+lift)) }
        n => panic!("{n}"),
    };
    let (u,v) = ([x(1),x(2)],[x(3),x(4)]);
    let whole = std::env::var_os("PART").is_none();
    let s = under(&full,u,v,whole);
    let tol = 1e-7;
    println!("A: {} faces, volume {:.12}, check {:?}",s.faces.len(),cad_kernel::props::volume(&s),s.check(tol).err());
    // the region in x, y of the face
    let pt = |uu: f64,vv: f64| full.point(uu,vv);
    let (xa,xb) = (pt(u[0],v[0])[0].min(pt(u[1],v[0])[0]),pt(u[0],v[0])[0].max(pt(u[1],v[0])[0]));
    let (ya,yb) = (pt(u[0],v[0])[1].min(pt(u[0],v[1])[1]),pt(u[0],v[0])[1].max(pt(u[0],v[1])[1]));
    // (`BPLANE=ox,oy,oz,nx,ny,nz`: B the slab under the plane through o with upward normal n, 12
    // wide and 8 deep, in place of the block)
    let plane: Option<Vec<f64>> = std::env::var("BPLANE").ok().map(|v| v.split(',').map(|x| x.parse().unwrap()).collect());
    let b = match &plane {
        Some(q) => cad_kernel::build::prism(&cad_kernel::build::square([q[0],q[1],q[2]],[q[3],q[4],q[5]],6.),-8.,0.).unwrap(),
        None => cad_kernel::build::block([x(5),x(7),-1.],[x(6),x(8),3.]).unwrap(),
    };
    // the height B allows over (x, y): its top plane's, or the block's in its rectangle
    let cap = |xx: f64,yy: f64| -> f64 { match &plane {
        Some(q) => q[2]-((xx-q[0])*q[3]+(yy-q[1])*q[4])/q[5],
        None => if xx >= x(5) && xx <= x(6) && yy >= x(7) && yy <= x(8) { 3. } else { f64::NEG_INFINITY },
    } };
    // Gauss–Legendre on the rectangle of A ∩ B in plan
    let g = GAUSS8;
    // ∫ over [y0, y1] of h(y), split where h's two parts cross (found on 64 cells, bisected)
    let line = |y0: f64,y1: f64,h: &dyn Fn(f64) -> f64,d: &dyn Fn(f64) -> f64| -> f64 {
        let mut cuts = vec![y0];
        let m = 64;
        for j in 0..m {
            let (mut a,mut b) = (y0+(y1-y0)*j as f64/m as f64,y0+(y1-y0)*(j+1) as f64/m as f64);
            if (d(a) < 0.) != (d(b) < 0.) {
                for _ in 0..60 { let c = 0.5*(a+b); if (d(c) < 0.) == (d(a) < 0.) { a = c } else { b = c } }
                cuts.push(0.5*(a+b));
            }
        }
        cuts.push(y1);
        let mut sum = 0.;
        for w in cuts.windows(2) {
            let n = 16; let k = (w[1]-w[0])/n as f64;
            for j in 0..n { for &(q,wq) in &g { sum += wq*h(w[0]+k*(j as f64+0.5+0.5*q))*k/2. } }
        }
        sum
    };
    let integral = |x0: f64,x1: f64,y0: f64,y1: f64,clip: bool| {
        if x1 <= x0 || y1 <= y0 { return 0. }
        let n = 2000;
        let h = (x1-x0)/n as f64;
        let mut sum = 0.;
        for i in 0..n { for &(p,wp) in &g {
            let xx = x0+h*(i as f64+0.5+0.5*p);
            let top = |yy: f64| if clip { z(xx,yy).min(cap(xx,yy)).max(0.) } else { z(xx,yy) };
            sum += wp*h/2.*line(y0,y1,&top,&|yy: f64| if clip { z(xx,yy)-cap(xx,yy) } else { 1. });
        }}
        sum
    };
    let va = integral(xa,xb,ya,yb,false);
    let (vab,vb) = match &plane {
        Some(_) => (integral(xa,xb,ya,yb,true),144.*8.),
        None => (integral(xa.max(x(5)),xb.min(x(6)),ya.max(x(7)),yb.min(x(8)),false),(x(6)-x(5))*(x(8)-x(7))*4.),
    };
    println!("A closed {va:.12}; A∩B {vab:.12}");
    // (`KEPT`: A cut by B's top plane alone, `build::kept`, keeping what lies below it)
    if let (Some(q),true) = (&plane,std::env::var_os("KEPT").is_some()) {
        let clock = Instant::now();
        for whole in [true,false] {
            let a = under(&full,u,v,whole);
            match cad_kernel::build::kept(&a,[q[0],q[1],q[2]],[-q[3],-q[4],-q[5]],0.) {
                Ok(r) => println!("kept (whole net {whole}): {} faces, volume {:.12}, area {:.12}, check {:?}, in {:?}",r.faces.len(),cad_kernel::props::volume(&r),
                    r.faces.iter().map(|f| cad_kernel::props::area(&r,f)).sum::<f64>(),r.check(tol).err(),clock.elapsed()),
                Err(e) => println!("kept (whole net {whole}): refused in {:?}: {e}",clock.elapsed()),
            }
        }
        return
    }
    let only = std::env::var("OP").ok();
    for (name,op,want) in [("union",Op::Union,va+vb-vab),("difference",Op::Cut,va-vab),("intersection",Op::Common,vab)] {
        if only.as_deref().is_some_and(|o| o != name) { continue }
        let clock = Instant::now();
        match boolean(&s,&b,op,tol) {
            Ok(r) => { let got = cad_kernel::props::volume(&r);
                println!("{name}: {} faces, volume {got:.12} want {want:.12} ({:.1e}), area {:.12}, check {:?}, in {:?}",r.faces.len(),(got-want)/want,
                    r.faces.iter().map(|f| cad_kernel::props::area(&r,f)).sum::<f64>(),r.check(tol).err(),clock.elapsed()) }
            Err(e) => println!("{name}: refused in {:?}: {e}",clock.elapsed()),
        }
    }
}
