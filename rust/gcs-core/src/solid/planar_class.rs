//! The planar generating class (#65), rows P1–P5: a sweep under a motion that turns a plane in
//! itself, of a tool that is a stock cut by one pocket — prisms standing square to that plane
//! through the blank. Over a whole period what such a sweep leaves of the blank is the blank within
//! the pocket's inner envelope (`envelope::planar`) carried along the prisms: a Wankel rotor is its
//! blank less the housing turned about it. The rows hold that reading:
//!
//! - P1: the motion turns the plane in itself about axes square to it (the envelope's plane row);
//! - P2: the tool is a stock cut by one pocket, both prisms on the pocket's plane spanning the
//!   blank's height along its normal, the pocket's profile one closed curve;
//! - P3: the roll is a whole period, so the envelope is closed and nothing is cut at its ends;
//! - P4: the pocket's inner envelope is one regular loop: no fold, no crossing, no gap;
//! - P5: the stock's outer wall never reaches the blank — every point of the blank, at every pose
//!   sampled, lies within the stock — so the pocket alone decides what is kept.
//!
//! Sampled where the other classes are: P5 on a grid of the blank's section and of the roll, P4
//! on the envelope's own samples (`envelope::planar`).
#[allow(unused_imports)]
use crate::fmath::Det;
use super::admission::{Basis,Class,Condition,Equivalence,Options,SweepEvidence};
use crate::envelope::Motion;
use crate::envelope::planar::{InnerEnvelope,Row};
use crate::model::{EntKind,Sketch,SolidDef};
use crate::space::{cross,dot,norm};

type V = [f64;3];
pub type Failure = (Condition,String,Option<V>);

/// What the class found of one sweep: the pocket's inner envelope, and how far along its plane's
/// normal the blank reaches in the sweep's frame (model units, measured as the normal's ordinate
/// in space), which the envelope's prism must span.
#[derive(Clone,Debug)]
pub struct Found { pub envelope: InnerEnvelope,pub heights: [f64;2] }

/// Whether `swept`'s rows are this class's: its tool is a prism cut by a pocket. A prism alone (a
/// rack) cuts with its sides, as the generating class's rows ask of it.
pub fn asks(sk: &Sketch,swept: usize) -> bool {
    let SolidDef::Swept {source,..} = &sk.solids[swept].def else { return false };
    matches!(&sk.solids[*source as usize].def,
        SolidDef::Body {stock,through,..} if !through.is_empty() && matches!(sk.solids[*stock as usize].def,SolidDef::Prism {..}))
}

/// The stretch along unit `n` a prism standing square to it spans in space.
pub(crate) fn prism(sk: &Sketch,id: usize,n: V) -> Result<[f64;2],Failure> {
    let solid = &sk.solids[id];
    let SolidDef::Prism {face,from,to} = &solid.def else {
        return Err((Condition::Prisms,format!("`{}` is no prism",solid.name),None))
    };
    let basis = match sk.faces[*face as usize].plane() {
        Ok(Some(p)) => sk.basis(p as usize),
        Ok(None) => crate::plane::Basis::page(),
        Err(e) => return Err((Condition::Prisms,e,None)),
    };
    let m = basis.normal();
    let m = m.map(|x| x/norm(m));
    if norm(cross(m,n)) > 1e-9 {
        return Err((Condition::Prisms,format!("`{}` stands on another plane than the pocket's",solid.name),None))
    }
    let (o,s) = (dot(basis.o,n),dot(m,n));
    let (a,b) = (o+s*from.value,o+s*to.value);
    Ok([a.min(b),a.max(b)])
}

/// Rows P1–P5 for one placement (`pose`, the sweep's frame in the body's) of planar sweep `swept`,
/// the blank read in the sweep's frame by `inside` and boxed in the body's by `bounds` (lowest and
/// highest corner). The envelope is the sweep's, the same at every placement: read into `read` at
/// the first, and kept for the rest.
pub fn admit(sk: &Sketch,swept: usize,read: &mut Option<InnerEnvelope>,pose: Motion,inside: &(dyn Fn(V) -> bool+Sync),
    bounds: Option<(V,V)>,options: &Options) -> Result<SweepEvidence,Failure> {
    let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else {
        return Err((Condition::Prisms,"not a sweep".into(),None))
    };
    let tool = &sk.solids[*source as usize];
    let pocketed = |why: &str| (Condition::Prisms,format!("`{}` {why}: a planar sweep's tool is a stock cut by one pocket",tool.name),None);
    let (stock,pocket) = match &tool.def {
        SolidDef::Body {stock,on,through,bound} if on.is_empty() && bound.is_empty() && through.len() == 1 => (*stock as usize,through[0] as usize),
        _ => return Err(pocketed("is more than a stock and a pocket")),
    };
    // the pocket's profile: one closed curve, alone
    let SolidDef::Prism {face,..} = &sk.solids[pocket].def else { return Err(pocketed("is cut by no prism")) };
    let f = &sk.faces[*face as usize];
    let loops: Vec<_> = f.boundaries().collect();
    let curve = match loops.as_slice() {
        [(edges,_)] if edges.len() == 1 && edges[0].kind == EntKind::Curve => edges[0].i(),
        _ => return Err((Condition::Prisms,format!("`{}`'s profile is not one closed curve",sk.solids[pocket].name),None)),
    };
    let view = sk.curve_view(curve);
    let basis = view.map_or_else(crate::plane::Basis::page,|v| sk.basis(v));
    let n = basis.normal();
    let n = n.map(|x| x/norm(n));
    let (stock_span,pocket_span) = (prism(sk,stock,n)?,prism(sk,pocket,n)?);
    // the blank's corners in the sweep's frame, and how far they reach along the normal
    let Some(bounds) = bounds else { return Err((Condition::Prisms,"the blank's extent is unknown".into(),None)) };
    let back = pose.inverse();
    let corners: Vec<V> = (0..8).map(|k| back.point(std::array::from_fn(|i| if (k>>i)&1 == 0 { bounds.0[i] } else { bounds.1[i] }))).collect();
    let heights = corners.iter().fold([f64::INFINITY,f64::NEG_INFINITY],|[a,b],c| [a.min(dot(*c,n)),b.max(dot(*c,n))]);
    for (span,what) in [(stock_span,stock),(pocket_span,pocket)] {
        if span[0] > heights[0] || span[1] < heights[1] {
            return Err((Condition::Prisms,format!("`{}` does not stand through the blank: it spans {:.4} to {:.4} along the \
                pocket's normal, and the blank {:.4} to {:.4}",sk.solids[what].name,span[0],span[1],heights[0],heights[1]),None))
        }
    }
    // P1, P3, P4: the pocket's inner envelope, read once for the sweep
    if read.is_none() {
        *read = Some(InnerEnvelope::read(sk,curve,*motion as usize,[from.value,to.value]).map_err(|r| {
            let row = match r.row { Row::Plane => Condition::Plane,Row::Period => Condition::Period,Row::Source | Row::Regular => Condition::Envelope };
            let at = r.witness.map(|p| sk.world_in(view,(p[0],p[1])));
            (row,r.message,at)
        })?);
    }
    let envelope = read.clone().expect("read above");
    // P5: the blank's section at mid-height read on a grid, every point of it within the stock at
    // every pose of a grid of the roll
    let family = crate::motion::Family::read(sk,*motion as usize).map_err(|e| (Condition::Plane,e,None))?;
    let stock_field = super::SpatialField::read(sk,stock,options.axis_tolerance).map_err(|e| (Condition::Prisms,e,None))?;
    let mid = 0.5*(heights[0]+heights[1]);
    let (a,b) = { let u = basis.u; let u = u.map(|x| x/norm(u)); (u,cross(n,u)) };
    let (lo,hi) = corners.iter().fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(lo,hi),c| {
        let (x,y) = (dot(*c,a),dot(*c,b));
        ([lo[0].min(x),lo[1].min(y)],[hi[0].max(x),hi[1].max(y)])
    });
    let side = options.coarse_rows.max(8)*2;
    let points: Vec<V> = (0..=side).flat_map(|i| (0..=side).map(move |j| (i,j))).map(|(i,j)| {
        let (x,y) = (lo[0]+(hi[0]-lo[0])*i as f64/side as f64,lo[1]+(hi[1]-lo[1])*j as f64/side as f64);
        std::array::from_fn(|k| x*a[k]+y*b[k]+mid*n[k])
    }).filter(|&p| inside(p)).collect();
    let rolls = (((to.value-from.value)/std::f64::consts::TAU*360.).ceil() as usize).max(36);
    let poses: Vec<crate::motion::Pose> = (0..=rolls).map(|k| family.pose_at(from.value+(to.value-from.value)*k as f64/rolls as f64))
        .collect::<Result<_,_>>().map_err(|e| (Condition::Plane,e,None))?;
    let reached = crate::par::map(&points,|&p| poses.iter().find(|m| stock_field.value(m.inverse().point(p)) > -options.margin).map(|_| p));
    if let Some(p) = reached.into_iter().flatten().next() {
        return Err((Condition::Wall,format!("`{}`'s outer wall reaches the blank as it turns",sk.solids[stock].name),Some(p)))
    }
    let contacts = envelope.pieces.iter().map(Vec::len).sum();
    let spacing = envelope.pieces.iter().flat_map(|p| p.windows(2).map(|w| (w[1].at[0]-w[0].at[0]).dhypot(w[1].at[1]-w[0].at[1])))
        .fold(0.,f64::max);
    Ok(SweepEvidence {sweep:0,name:String::new(),placements:Vec::new(),samples:points.len(),contacts,spacing,least_area_factor:1.,
        near_double_roots:0,near_tangent_pairs:0,basis:Basis::Sampled {rows:side+1,columns:rolls},equivalence:Equivalence::Sampled,
        class:Class::Planar(Box::new(Found {envelope,heights}))})
}
