//! The inner envelope of a closed planar curve under a planar motion (#65), read off a drawing:
//! the envelope itself is the kernel's (`brep::swept::planar`, #721 F2c). What is here is the
//! drawing's reading — the curve read once off the solved sketch (`Source`), its view's page in
//! space (`frame`), its material in the plane (a `PlanarField` of its chords), the motion lowered —
//! and the kernel's refusals said in the drawing's words, by row.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::brep::swept::planar::{self as kernel,Region,Section,Seen,View};
use crate::model::Sketch;
use crate::motion::Family;
use crate::solid::PlanarField;
use std::fmt;

pub use kernel::{Contact,InnerEnvelope};

type P = [f64;2];

fn dist(a: P,b: P) -> f64 { (a[0]-b[0]).dhypot(a[1]-b[1]) }

pub use crate::solid::admission::Condition;

/// Why no envelope: the row, what was found, and a point of the curve's view where it was.
#[derive(Clone,Debug)]
pub struct Refusal { pub row: Condition,pub message: String,pub witness: Option<P> }

impl fmt::Display for Refusal {
    fn fmt(&self,f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"{}: {}",self.row.code(),self.message)?;
        if let Some(w) = self.witness { write!(f," (near ({:.6}, {:.6}))",w[0],w[1])?; }
        Ok(())
    }
}

fn refuse(row: Condition,message: impl Into<String>,witness: Option<P>) -> Refusal { Refusal {row,message:message.into(),witness} }

/// A view's page and space, as the affine maps between them (`brep::swept::planar::View`): what a
/// pose is read through without the sketch (which the threads reading contacts cannot share).
fn frame(sk: &Sketch,view: Option<usize>) -> View {
    let o = sk.world_in(view,(0.,0.));
    let cols = [crate::space::sub(sk.world_in(view,(1.,0.)),o),crate::space::sub(sk.world_in(view,(0.,1.)),o)];
    let (ax,ay) = sk.on_view_sheet([0.;3],view);
    let e = |k: usize| { let mut v = [0.;3]; v[k] = 1.; sk.page_vector_in(view,v) };
    let (ex,ey,ez) = (e(0),e(1),e(2));
    View {o,cols,at:[ax,ay],rows:[[ex.0,ey.0,ez.0],[ex.1,ey.1,ez.1]]}
}

impl Region for PlanarField {
    fn value(&self,p: P) -> f64 { PlanarField::value(self,p) }
}

/// The source curve, read once off the solved sketch so that an envelope point is solved from it
/// with no sketch at hand (a meter reads one from many threads): a formula's tapes and the numbers
/// they are read with, or a generated profile's encoding (`generate.rs`: a Wankel's bore is its
/// apex's envelope under the rotor's motion) and the coordinates it is written over.
#[derive(Clone,Debug)]
enum Source {
    Formula { x: crate::tape::Tape,y: crate::tape::Tape,vars: Vec<f64> },
    Generated { consts: Vec<f64>,theta: Vec<f64> },
}

impl Source {
    fn of(sk: &Sketch,curve: usize) -> Result<Source,Refusal> {
        use crate::model::{CurveBody,EntKind,EntRef};
        let cv = &sk.curves[curve];
        match &sk.curve_defs[cv.def as usize].body {
            CurveBody::Exprs {x,y} => Ok(Source::Formula {x:x.clone(),y:y.clone(),vars:sk.curve_vars(curve,0.)}),
            // what a contact on it carries: the roll its root is chosen at, its numbers, its encoding
            CurveBody::Envelope(g) => {
                let consts = g.contact_consts(sk,curve);
                let theta = sk.entity_params(EntRef::new(EntKind::Curve,curve)).into_iter()
                    .map(|p| sk.params[p as usize].value).collect();
                Ok(Source::Generated {consts,theta})
            }
            CurveBody::Trace(_) => Err(refuse(Condition::Envelope,"the inner envelope is read of a curve of a computed point or a \
                generated profile, not yet of a traced curve",None)),
            CurveBody::Extremal(_) => Err(refuse(Condition::Envelope,"the inner envelope is read of a curve of a computed point or a \
                generated profile, not of a free curve",None)),
            CurveBody::Followed(_) => Err(refuse(Condition::Envelope,"the inner envelope is read of a curve of a computed point or a \
                generated profile, not of a point traced over a parameter",None)),
            CurveBody::Developed(_) => Err(refuse(Condition::Envelope,"the inner envelope is read of a curve of a computed point or a \
                generated profile, not of a panel's rail laid flat",None)),
        }
    }

}

impl Section for Source {
    /// The point at `s` in the view's page coordinates and its derivative per unit of `s`.
    fn at(&self,s: f64) -> Option<(P,P)> {
        let out = match self {
            Source::Formula {x,y,vars} => {
                let mut v = vars.clone();
                v[0] = s;
                let mut scratch = crate::tape::Scratch::new();
                let (a,b) = (x.eval(&v,&mut scratch),y.eval(&v,&mut scratch));
                ([a.v,b.v],[a.d[0],b.d[0]])
            }
            Source::Generated {consts,theta} => {
                let v = crate::generate::kernel_eval(consts,s,theta,1,false);
                if !v.ok { return None }
                ([v.x,v.y],[v.dx[0],v.dy[0]])
            }
        };
        out.0.iter().chain(&out.1).all(|v| v.is_finite()).then_some(out)
    }
}

/// The inner envelope read off a drawing.
pub trait ReadEnvelope: Sized {
    /// The inner envelope of closed curve `curve` under motion `motion` over `roll` (radians), a
    /// whole period of it, in the curve's view.
    fn read(sk: &Sketch,curve: usize,motion: usize,roll: [f64;2]) -> Result<Self,Refusal>;
}

impl ReadEnvelope for InnerEnvelope {
    fn read(sk: &Sketch,curve: usize,motion: usize,roll: [f64;2]) -> Result<InnerEnvelope,Refusal> {
        if !(roll[0] < roll[1]) { return Err(refuse(Condition::Period,"the roll increases",None)) }
        let family = Family::read(sk,motion).map_err(|e| refuse(Condition::Envelope,e,None))?;
        if !sk.curve_closed(curve) {
            return Err(refuse(Condition::Envelope,"the curve does not come back to where it started: an envelope \
                is of a closed curve",None));
        }
        let view = sk.curve_view(curve);
        let (s0,s1) = sk.curve_domain(curve);
        let (s0,s1) = (s0.min(s1),s0.max(s1));
        // the source loop as chords, its signed distance the test of what lies inside it
        let poly = sk.curve_polyline(curve);
        let scale = poly.iter().fold(0_f64,|m,p| m.max(dist([p.0,p.1],[poly[0].0,poly[0].1]))).max(1e-300);
        let source = Source::of(sk,curve)?;
        let frame = frame(sk,view);
        let slack = 1e-7*(1.+scale);
        let ring: Vec<P> = sk.curve_polyline_within(curve,slack).into_iter().map(|(_,q)| [q.0,q.1]).collect();
        let edges: Vec<crate::solid::surface::Edge> = ring.windows(2)
            .map(|w| crate::solid::surface::Edge::Line {a:w[0],b:w[1],axis:false}).collect();
        let field = PlanarField::from_loops(&[edges],0.)
            .map_err(|e| refuse(Condition::Envelope,format!("the curve is no simple loop ({e})"),None))?;
        let path = family.trajectory().map_err(|e| refuse(Condition::Envelope,e,None))?;
        InnerEnvelope::new(std::sync::Arc::new(source),frame,path,&field,[s0,s1],scale,roll).map_err(|r| said(r,&family.name))
    }
}

/// The kernel's refusal of the envelope of a curve under the motion `name`, said in the drawing's
/// words.
fn said(r: kernel::Refusal,name: &str) -> Refusal {
    let row = match r.seen.row() {
        kernel::Row::Plane => Condition::Plane,
        kernel::Row::Period => Condition::Period,
        kernel::Row::Envelope => Condition::Envelope,
    };
    let message = match r.seen {
        Seen::NotInterval => "the roll is no interval".into(),
        Seen::LeavesPlane => format!("`{name}` carries the curve out of its plane, or turns it over: an envelope in the \
            plane is of a motion of the plane"),
        Seen::NotBack {from,to} => format!("from {:.4} to {:.4} degrees `{name}` does not bring the curve back: its inner \
            envelope is read over a whole period",from.to_degrees(),to.to_degrees()),
        Seen::Motion(e) => e.msg,
        Seen::Unread {s} => format!("the curve cannot be read with its tangent at {s}"),
        Seen::Folds => "the envelope folds back on itself: no rotor stays inside the curve along it".into(),
        Seen::NothingKept => "no contact is kept: the curve encloses nothing at every pose".into(),
        Seen::Open => "the envelope does not close: a piece of it ends at no corner of the curve".into(),
        Seen::Branches {meets} => format!("the envelope branches: {meets} pieces meet at one corner"),
        Seen::NotOneLoop => "the envelope does not close into one loop".into(),
        Seen::ManyLoops => "the envelope is more than one loop".into(),
        Seen::Crosses => "the envelope crosses itself".into(),
    };
    Refusal {row,message,witness:r.witness}
}
