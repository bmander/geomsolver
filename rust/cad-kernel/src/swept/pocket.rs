//! **The planar generating class's rows asked of one placement** (#65, #721 F1c): a sweep under a
//! motion that turns a plane in itself, of a tool that is a stock cut by one pocket — prisms
//! standing square to that plane through the blank. Over a whole period what such a sweep leaves of
//! the blank is the blank within the pocket's inner envelope (`planar::InnerEnvelope`) carried along
//! the prisms: a Wankel rotor is its blank less the housing turned about it. The envelope holds P1,
//! P3 and P4; what is asked here of each placement is
//!
//! - P2: both prisms span the blank's height along the pocket's normal;
//! - P5: the stock's outer wall never reaches the blank — every point of the blank, at every pose
//!   sampled, lies within the stock — so the pocket alone decides what is kept.
//!
//! Sampled where the other classes are: P5 on a grid of the blank's section and of the roll. The
//! pocket (`Pocket`) is the drawing's reading until the kernel reads it from the tool's B-rep, which
//! fits the pocket's curve where the drawing has it exactly (#721 F1d).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::trajectory::Motion;
use crate::util::space::{cross,dot,norm};
use super::dispatch::Row;
use super::planar::InnerEnvelope;
use super::tool::Material;
use std::sync::Arc;

type V = [f64;3];

/// A row that fails, what was seen, and a point (the sweep's frame) where, when there is one.
pub type Failure = (Row,String,Option<V>);

/// A planar sweep's tool as the class reads it (model units): the pocket's inner envelope over the
/// roll, a direction in its plane and its unit normal, how far the stock and the pocket each span
/// along that normal, their names, the stock's material, and the inverse poses of a grid of the
/// roll that P5 reads the blank against.
#[derive(Clone)]
pub struct Pocket { pub envelope: InnerEnvelope,pub u: V,pub n: V,pub spans: [[f64;2];2],pub names: [String;2],
    pub stock: Arc<dyn Material>,pub backs: Vec<Motion> }

/// What the class found of one sweep: the pocket's inner envelope, and how far along its plane's
/// normal the blank reaches in the sweep's frame (model units, measured as the normal's ordinate
/// in space), which the envelope's prism must span.
#[derive(Clone,Debug)]
pub struct Found { pub envelope: InnerEnvelope,pub heights: [f64;2] }

/// One placement's rows held: how far the blank reaches along the normal (P2) and how P5 sampled.
#[derive(Clone,Copy,Debug)]
pub struct Checked { pub heights: [f64;2],pub samples: usize,pub rows: usize,pub columns: usize }

impl Pocket {
    /// Rows P2 and P5 for one placement (`pose`, the sweep's frame in the body's), the blank read in
    /// the sweep's frame by `inside` and boxed in the body's by `bounds` (lowest and highest
    /// corner), its section read on a grid of `side` steps each way, a point within `margin` of the
    /// stock's boundary counted as reaching its wall.
    pub fn check(&self,pose: Motion,inside: &(dyn Fn(V) -> bool+Sync),bounds: Option<(V,V)>,side: usize,margin: f64)
        -> Result<Checked,Failure> {
        let n = self.n;
        // the blank's corners in the sweep's frame, and how far they reach along the normal
        let Some(bounds) = bounds else { return Err((Row::Prisms,"the blank's extent is unknown".into(),None)) };
        let back = pose.inverse();
        let corners: Vec<V> = (0..8).map(|k| back.point(std::array::from_fn(|i| if (k>>i)&1 == 0 { bounds.0[i] } else { bounds.1[i] }))).collect();
        let heights = corners.iter().fold([f64::INFINITY,f64::NEG_INFINITY],|[a,b],c| [a.min(dot(*c,n)),b.max(dot(*c,n))]);
        for (span,what) in self.spans.iter().zip(&self.names) {
            if span[0] > heights[0] || span[1] < heights[1] {
                return Err((Row::Prisms,format!("`{what}` does not stand through the blank: it spans {:.4} to {:.4} along the \
                    pocket's normal, and the blank {:.4} to {:.4}",span[0],span[1],heights[0],heights[1]),None))
            }
        }
        // P5: the blank's section at mid-height read on a grid, every point of it within the stock at
        // every pose of a grid of the roll
        let mid = 0.5*(heights[0]+heights[1]);
        let (a,b) = { let u = self.u; let u = u.map(|x| x/norm(u)); (u,cross(n,u)) };
        let (lo,hi) = corners.iter().fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(lo,hi),c| {
            let (x,y) = (dot(*c,a),dot(*c,b));
            ([lo[0].min(x),lo[1].min(y)],[hi[0].max(x),hi[1].max(y)])
        });
        let points: Vec<V> = (0..=side).flat_map(|i| (0..=side).map(move |j| (i,j))).map(|(i,j)| {
            let (x,y) = (lo[0]+(hi[0]-lo[0])*i as f64/side as f64,lo[1]+(hi[1]-lo[1])*j as f64/side as f64);
            std::array::from_fn(|k| x*a[k]+y*b[k]+mid*n[k])
        }).filter(|&p| inside(p)).collect();
        let reached = crate::util::par::map(&points,|&p| self.backs.iter().find(|m| self.stock.value(m.point(p)) > -margin).map(|_| p));
        if let Some(p) = reached.into_iter().flatten().next() {
            return Err((Row::Wall,format!("`{}`'s outer wall reaches the blank as it turns",self.names[0]),Some(p)))
        }
        Ok(Checked {heights,samples:points.len(),rows:side+1,columns:self.backs.len()-1})
    }

    /// The class's finding, its envelope's, the blank reaching `checked`'s heights: with how many
    /// contacts the envelope holds and the widest gap between neighbours.
    pub fn found(&self,checked: &Checked) -> (Found,usize,f64) {
        let envelope = self.envelope.clone();
        let contacts = envelope.pieces.iter().map(Vec::len).sum();
        let spacing = envelope.pieces.iter().flat_map(|p| p.windows(2).map(|w| (w[1].at[0]-w[0].at[0]).dhypot(w[1].at[1]-w[0].at[1])))
            .fold(0.,f64::max);
        (Found {envelope,heights:checked.heights},contacts,spacing)
    }
}
