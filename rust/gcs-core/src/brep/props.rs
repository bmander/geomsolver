//! What a B-rep measures, read off its loops: the volume by the divergence theorem, each face's
//! flux turned into a line integral round its loops in its own parameters (Green's theorem), so
//! nothing is meshed and a face of revolution is integrated as exactly as a plane.
//!
//! `V = ⅓ Σ_F ∬_F X·n dA`, and on a face `X·n dA = s g(u, v) du dv` with `g = S·(S_u × S_v)` and
//! `s = ±1` its sense. With `G(u, v) = ∫_0^u g(σ, v) dσ`, Green's theorem gives
//! `∬_D g du dv = ∮_∂D G dv` round the domain counter-clockwise — and a face's loops run
//! counter-clockwise exactly when it is not reversed, so `∮ G dv` round its loops as stored is
//! its flux, whichever its sense.
use super::geom::Curve;
use super::topo::{Brep,EdgeCurve};
use crate::space::{cross,dot};

/// Ten-point Gauss–Legendre nodes and weights on [−1, 1].
const GAUSS: [(f64,f64);10] = [
    (-0.9739065285171717,0.0666713443086881),(-0.8650633666889845,0.1494513491505806),
    (-0.6794095682990244,0.2190863625159820),(-0.4333953941292472,0.2692667193099963),
    (-0.1488743389816312,0.2955242247147529),(0.1488743389816312,0.2955242247147529),
    (0.4333953941292472,0.2692667193099963),(0.6794095682990244,0.2190863625159820),
    (0.8650633666889845,0.1494513491505806),(0.9739065285171717,0.0666713443086881),
];

/// `∫_a^b f` by ten-point Gauss–Legendre on `pieces` equal pieces.
fn gauss(a: f64,b: f64,pieces: usize,f: &mut dyn FnMut(f64) -> f64) -> f64 {
    let h = (b-a)/pieces as f64;
    let mut sum = 0.;
    for p in 0..pieces {
        let mid = a+h*(p as f64+0.5);
        for &(x,w) in &GAUSS { sum += w*f(mid+x*h/2.); }
    }
    sum*h/2.
}

/// Pieces for an angle-like span, a sixteenth of a turn each.
fn pieces(span: f64,periodic: bool) -> usize {
    if periodic { ((span.abs()/(std::f64::consts::PI/8.)).ceil() as usize).max(1) } else { 4 }
}

/// The volume the boundary encloses (negative if it is turned inside out).
pub fn volume(b: &Brep) -> f64 {
    let mut total = 0.;
    for f in &b.faces {
        let s = &f.surface;
        let [pu,pv] = s.periods();
        let g = |u: f64,v: f64| { let (x,su,sv) = s.d1([u,v]); dot(x,cross(su,sv)) };
        let big_g = |u: f64,v: f64| gauss(0.,u,pieces(u,pu.is_some()),&mut |w| g(w,v));
        for l in &f.loops {
            for c in l {
                let e = &b.edges[c.edge as usize];
                if matches!(e.curve,EdgeCurve::Degenerate) { continue }
                let span = e.t[1]-e.t[0];
                let periodic = pu.is_some() || pv.is_some() || matches!(&e.curve,EdgeCurve::Curve(c) if c.period().is_some());
                let mut integrand = |t: f64| {
                    let dv = c.pcurve.derivative(t,e,s,&b.vertices)[1];
                    if dv == 0. { return 0. }
                    let [u,v] = c.pcurve.at(t,e,s,&b.vertices);
                    big_g(u,v)*dv
                };
                let line = if let EdgeCurve::Curve(Curve::Traced(_)) = &e.curve {
                    // a traced curve is smooth between its points: integrated a stretch at a time
                    let mut cuts = vec![e.t[0]];
                    cuts.extend((e.t[0].floor() as i64+1..=e.t[1].ceil() as i64-1).map(|k| k as f64));
                    cuts.push(e.t[1]);
                    cuts.windows(2).map(|w| gauss(w[0],w[1],1,&mut integrand)).sum()
                } else { gauss(e.t[0],e.t[1],pieces(span,periodic),&mut integrand) };
                total += if c.reversed { -line } else { line };
            }
        }
    }
    total/3.
}
