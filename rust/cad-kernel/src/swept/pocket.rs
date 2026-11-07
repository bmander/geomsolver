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
use super::planar::{InnerEnvelope,Region};
use super::subtraction::Loops;
use crate::plane2::Edge;
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

/// Why a tool's B-rep, read as a stock cut by one pocket, has no planar class reading: a row its
/// shape fails (P2, in words with a point), or its pocket's inner envelope refused (P1, P3, P4),
/// which the solver says in the motion's words.
#[derive(Clone,Debug)]
pub enum Unread { Shape(Failure),Envelope(super::planar::Refusal) }

/// The pocket's curve, read off the B-rep's hole loop (millimetres) in the cap's plane: its pieces,
/// each a curve over the stretch of its parameter its edge runs (walked as the loop walks it); read
/// at `s`, one curve's own parameter where every piece is a stretch of one curve end to end (a
/// point's path's halves, read over its whole roll as the drawing reads it), else the `k`th piece
/// at `s − k`. In page coordinates of `view` (model units, `mm` millimetres a unit).
#[derive(Debug)]
struct Hole { pieces: Vec<(crate::geom::Curve,[f64;2])>,whole: bool,view: super::planar::View,mm: f64 }

impl super::planar::Section for Hole {
    fn at(&self,s: f64) -> Option<([f64;2],[f64;2])> {
        let (curve,t,rate) = match self.whole {
            true => {
                // the piece whose stretch holds `s`
                let (c,_) = self.pieces.iter().find(|(_,[a,b])| s >= a.min(*b)-1e-12 && s <= a.max(*b)+1e-12)?;
                (c,s,1.)
            }
            false => {
                let k = (s.floor().max(0.) as usize).min(self.pieces.len()-1);
                let (c,[a,b]) = &self.pieces[k];
                (c,a+(b-a)*(s-k as f64),b-a)
            }
        };
        let (p,d,_) = curve.d2(t);
        let p = self.view.page(p.map(|x| x/self.mm));
        let d = self.view.vector(d.map(|x| x*rate/self.mm));
        (p.iter().chain(&d).all(|x| x.is_finite())).then_some((p,d))
    }
}

/// The stock's material: its outline's prism — the outline loop's region (`outline`, in the page
/// of `view`) over its span along `n` — the tool with its pocket filled, whose wall P5 asks of.
#[derive(Debug)]
struct Stock { outline: Loops,view: super::planar::View,n: V,span: [f64;2] }

impl Material for Stock {
    fn value(&self,p: V) -> f64 {
        let h = dot(p,self.n);
        self.outline.value(self.view.page(p)).max(self.span[0]-h).max(h-self.span[1])
    }
}

impl Pocket {
    /// **A tool's B-rep read as a stock cut by one pocket** (#721 F1d): `b` (millimetres, `mm` a
    /// model unit) two caps, planes square to one direction each bounded by its outline and one
    /// hole, every other face standing along that direction — a plane, a cylinder or a sweep of a
    /// curve along it — so the tool is its outline's prism less its hole's. The pocket is the hole
    /// read off the B-rep, exactly where the B-rep carries its curve exactly (a point's path under
    /// turns, `geom::Curve::Trochoid`), its inner envelope under `path` over `roll` read
    /// (`InnerEnvelope::new`); the stock's material the tool's boundary's with the pocket filled;
    /// both prisms spanning the caps. `names` the stock's and the pocket's, as a refusal says them.
    /// None where the B-rep is no such tool (the class is not asked); a refusal by row where its
    /// envelope is none.
    pub fn read(b: &crate::topo::Brep,path: &crate::trajectory::Trajectory,roll: [f64;2],mm: f64) -> Option<Result<Pocket,Unread>> {
        use crate::geom::{Curve,Surface};
        use crate::recipe::named_parts;
        use crate::topo::EdgeCurve;
        // the caps: planes with an outline and one hole, square to one direction
        let caps: Vec<usize> = (0..b.faces.len()).filter(|&fi| matches!(b.faces[fi].surface,Surface::Plane(_)) && b.faces[fi].loops.len() == 2)
            .collect();
        let [c0,c1] = caps[..] else { return None };
        let normal = |fi: usize| match &b.faces[fi].surface { Surface::Plane(f) => f.z,_ => unreachable!("a plane") };
        let n = normal(c0);
        if norm(cross(n,normal(c1))) > 1e-9 { return None }
        // every other face stands along it
        let along = |z: V| norm(cross(z,n)) <= 1e-9;
        for (fi,f) in b.faces.iter().enumerate() {
            if fi == c0 || fi == c1 { continue }
            let standing = match &f.surface {
                Surface::Plane(fr) => dot(fr.z,n).abs() <= 1e-9,
                Surface::Cylinder(fr,_) | Surface::Extrusion(fr,_) => along(fr.z),
                _ => false,
            };
            if !standing { return None }
        }
        // the hole: the cap's loop enclosing the less (`Brep::loop_area`), its curves as it walks them
        let face = &b.faces[c0];
        let [first,second] = [&face.loops[0],&face.loops[1]];
        let (hole,outline) = if b.loop_area(face,first).abs() <= b.loop_area(face,second).abs() { (first,second) } else { (second,first) };
        let mut pieces: Vec<(Curve,[f64;2])> = Vec::new();
        for c in hole {
            let e = &b.edges[c.edge as usize];
            let EdgeCurve::Curve(curve) = &e.curve else { return None };
            pieces.push((curve.clone(),if c.reversed { [e.t[1],e.t[0]] } else { e.t }));
        }
        // read as one curve's parameter where its pieces are stretches of one curve end to end
        let one = pieces.windows(2).all(|w| w[0].0 == w[1].0) && {
            let mut spans: Vec<[f64;2]> = pieces.iter().map(|(_,[a,b])| [a.min(*b),a.max(*b)]).collect();
            spans.sort_by(|x,y| x[0].total_cmp(&y[0]));
            spans.windows(2).all(|w| (w[0][1]-w[1][0]).abs() <= 1e-9*(1.+w[0][1].abs()))
        };
        let whole = one.then(|| pieces.iter().fold([f64::INFINITY,f64::NEG_INFINITY],|[lo,hi],(_,[a,b])| [lo.min(a.min(*b)),hi.max(a.max(*b))]));
        // the cap's plane in model units, its page its own coordinates
        let o = b.vertices[b.edges[hole[0].edge as usize].v[0] as usize].p.map(|x| x/mm);
        // (laid as the curve's own frame is where it is a point's path, so the page turns as the
        // drawing's did; else square to the caps)
        let (x,y) = pieces.iter().find_map(|(c,_)| match c { Curve::Trochoid(t) => Some((t.frame.x,t.frame.y)),_ => None })
            .unwrap_or_else(|| crate::util::space::across(n));
        let view = super::planar::View {o,cols:[x,y],at:[-dot(x,o),-dot(y,o)],rows:[x,y]};
        let height = |fi: usize| dot(b.vertices[b.edges[b.faces[fi].loops[0][0].edge as usize].v[0] as usize].p,n)/mm;
        let (h0,h1) = (height(c0),height(c1));
        let span = [h0.min(h1),h0.max(h1)];
        // a loop's material in the cap's plane: its chords, its edges sampled finely as it walks them
        let region = |l: &Vec<crate::topo::Coedge>| -> (Vec<[f64;2]>,Loops) {
            let points: Vec<[f64;2]> = l.iter().flat_map(|c| { let e = &b.edges[c.edge as usize];
                let [t0,t1] = if c.reversed { [e.t[1],e.t[0]] } else { e.t };
                (0..2048).map(move |k| e.point(t0+(t1-t0)*k as f64/2048.,&b.vertices)) })
                .map(|p| view.page(p.map(|v| v/mm))).collect();
            let edges = (0..points.len()).map(|k| Edge::Line {a:points[k],b:points[(k+1)%points.len()],axis:false}).collect();
            (points,Loops::new(&[edges]))
        };
        let (points,ring) = region(hole);
        let scale = points.iter().fold(0_f64,|m,p| m.max(p[0].dhypot(p[1]))).max(1e-300);
        // the stock's and the pocket's names: the outline's walls' solid and the hole's
        let wall = |l: &Vec<crate::topo::Coedge>| l.iter().find_map(|c| b.uses()[c.edge as usize].iter().map(|u| u.0)
            .find(|&g| g != c0 && g != c1).map(|g| named_parts(&b.faces[g].name).0.to_string())).unwrap_or_default();
        let names = [wall(outline),wall(hole)];
        let roll_of = whole.unwrap_or([0.,pieces.len() as f64]);
        let source = Hole {pieces,whole:whole.is_some(),view,mm};
        let envelope = match InnerEnvelope::new(Arc::new(source),view,path.clone(),&ring,roll_of,scale,roll) {
            Ok(envelope) => envelope,
            Err(r) => return Some(Err(Unread::Envelope(r))),
        };
        let stock = Stock {outline:region(outline).1,view,n,span};
        let rolls = (((roll[1]-roll[0])/std::f64::consts::TAU*360.).ceil() as usize).max(36);
        let backs = (0..=rolls).map(|k| path.at(roll[0]+(roll[1]-roll[0])*k as f64/rolls as f64).map(|m| m.inverse()))
            .collect::<Result<Vec<_>,_>>();
        let backs = match backs { Ok(b) => b,Err(e) => return Some(Err(Unread::Shape((Row::Plane,e.msg,None)))) };
        Some(Ok(Pocket {envelope,u:x,n,spans:[span,span],names,stock:Arc::new(stock),backs}))
    }
}
