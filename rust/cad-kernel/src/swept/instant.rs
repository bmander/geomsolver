//! **Contacts read at each instant** (#721 K2, docs/swept-solid-design.md §2 and step 7): under a
//! rigid motion no constant twist takes, no contact set is fixed on the tool, but each instant is a
//! screw (Mozzi–Chasles), and against it a face of revolution's contact is still in closed form on
//! each ring, `A cos θ + B sin θ + C = 0` (`Surface::ring_contact`), its coefficients the
//! trajectory's twist at that instant read on the tool (`Twist::body`). So a revolved face sweeps a
//! chart over `(t, h, ±)`: at each roll `t` and ring height `h`, its two roots, each carried by the
//! pose at `t`. The two are told apart by the sign of the contact's rate round the ring there,
//! `−A sin θ + B cos θ` (`+` where it rises through the root), which keeps its sign until they meet;
//! they meet on **the branch locus**, `A² + B² = C²`, where the ring's roots are one (`Double`) and
//! beyond which it has none. The chart is the patch the pipeline's arrangement takes (§2's K2 row);
//! the branch locus, a curve in `(t, h)`, is where the patch is cut (§3).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::contact::RingRoots;
use crate::geom::Surface;
use crate::trajectory::{Trajectory,Twist};
use crate::util::space::dot;

type V = [f64;3];

/// A revolved face's contacts over a grid of rolls and ring heights: at each node its `+` and `−`
/// roots round the ring (`None` where the ring has none there, or is in contact all round), each
/// with its point and outward unit normal carried to the pose at its roll.
#[derive(Clone,Debug)]
pub struct RingChart {
    pub times: Vec<f64>,
    pub heights: Vec<f64>,
    /// Row-major, a row a roll: `[+, −]`, each its angle round the ring, its point and its normal.
    pub roots: Vec<[Option<(f64,V,V)>;2]>,
}

/// `n ≥ 2` points evenly over `[a, b]`, its ends included.
fn grid([a,b]: [f64;2],n: usize) -> Vec<f64> { let n = n.max(2); (0..n).map(|i| a+(b-a)*i as f64/(n-1) as f64).collect() }

/// The trajectory's twist at roll `t`, read on what it carries.
fn twist_at(path: &Trajectory,t: f64) -> Result<Twist,String> { Ok(Twist::body(path.at(t).map_err(|e| e.msg)?)) }

/// **The face's chart** over `nt × nh` nodes of `roll × heights` (each end included), its roots
/// read to `tolerance` (a velocity along the normal, as `Sinusoid::roots` reads it). Refused where
/// the face is no surface of revolution, or the trajectory cannot be read.
pub fn ring_chart(face: &Surface,path: &Trajectory,roll: [f64;2],heights: [f64;2],nt: usize,nh: usize,tolerance: f64)
    -> Result<RingChart,String> {
    if !matches!(face.contact_form(),crate::contact::Form::Rings {..}) { return Err(format!("a {} has no rings",face.kind())) }
    let (times,hs) = (grid(roll,nt),grid(heights,nh));
    let mut roots = Vec::with_capacity(times.len()*hs.len());
    for &t in &times {
        let pose = path.pose(t).map_err(|e| e.msg)?;
        let twist = twist_at(path,t)?;
        for &h in &hs {
            let Some(k) = face.ring_contact(h,&twist) else { roots.push([None,None]); continue };
            let both = match k.roots(tolerance) {
                RingRoots::Two(r) => r.to_vec(),
                RingRoots::Double(u) => vec![u],
                RingRoots::None | RingRoots::All => vec![],
            };
            let mut node = [None,None];
            for u in both {
                let (Some(n),p) = (face.normal([u,h]),face.d1([u,h]).0) else { continue };
                // which root: the contact rising (`+`) or falling through it round the ring
                let (s,c) = u.dsin_cos();
                let rate = -k.a*s+k.b*c;
                let carried = (u,pose.point(p),pose.vector(n));
                if rate >= 0. && node[0].is_none() { node[0] = Some(carried) } else { node[1] = Some(carried) }
            }
            roots.push(node);
        }
    }
    Ok(RingChart {times,heights:hs,roots})
}

/// **The branch locus** of `face`'s chart: at each of `nt` rolls over `roll`, the heights within
/// `heights` where its ring's two roots meet, `A² + B² = C²`, found where `A² + B² − C²` changes
/// sign between `nh` heights and settled within that bracket (`roots::bracketed_root`), as
/// `(t, h)` pairs.
pub fn branch_locus(face: &Surface,path: &Trajectory,roll: [f64;2],heights: [f64;2],nt: usize,nh: usize)
    -> Result<Vec<(f64,f64)>,String> {
    let hs = grid(heights,nh);
    let mut out = Vec::new();
    for t in grid(roll,nt) {
        let twist = twist_at(path,t)?;
        let gap = |h: f64| face.ring_contact(h,&twist).map(|k| k.a*k.a+k.b*k.b-k.c*k.c);
        let gaps: Vec<Option<f64>> = hs.iter().map(|&h| gap(h)).collect();
        for j in 0..hs.len()-1 {
            let (Some(g0),Some(g1)) = (gaps[j],gaps[j+1]) else { continue };
            if g0 == 0. { out.push((t,hs[j])); continue }
            if g0*g1 >= 0. { continue }
            out.push((t,crate::util::roots::bracketed_root(gap,hs[j],g0,hs[j+1],g1,1e-14*(1.+hs[j].abs()))));
        }
    }
    Ok(out)
}

/// Whether a chart's point is grazing at its instant, read in the world apart from the chart: its
/// normal dotted with the world velocity of the motion there (`Twist::spatial`), over the speed.
pub fn grazing(path: &Trajectory,t: f64,p: V,n: V) -> Result<f64,String> {
    let v = Twist::spatial(path.at(t).map_err(|e| e.msg)?).velocity(p);
    Ok(dot(n,v)/(1e-300+dot(v,v).sqrt()))
}
