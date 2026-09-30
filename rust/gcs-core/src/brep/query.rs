//! The questions a Boolean asks of geometry and of a boundary: where a curve meets a surface,
//! whether a point of a face's surface is inside the face, and whether a point of space is inside
//! a solid.
use super::geom::{Curve,Surface,Uv,V};
use super::topo::{unwrap,Brep,EdgeCurve,Face};
use crate::space::{add,distance,norm,scale};

/// Where a point stands against a face or a solid.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Place { In,Out,On }

/// Where a curve meets a surface over a stretch of its parameter.
#[derive(Clone,Debug,PartialEq)]
pub enum Meets {
    /// The parameters where it crosses (`false`) or touches (`true`) the surface.
    At(Vec<(f64,bool)>),
    /// The whole stretch lies on the surface.
    Along,
}

/// Where `curve` over `[t0, t1]` meets `surface`, to `tol`: every sign change of the surface's
/// signed distance along it refined to a root, every sampled minimum of its size that comes within
/// `tol` refined to a touching point, and an end on the surface a root there.
pub fn curve_surface(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> Meets {
    let f = |t: f64| surface.implicit(curve.point(t));
    let length = (t1-t0)*curve.speed();
    let n = ((length/surface.feature()*16.).ceil() as usize)
        .max(if curve.period().is_some() { ((t1-t0)/(std::f64::consts::PI/32.)).ceil() as usize } else { 1 })
        .clamp(16,1<<14);
    let ts: Vec<f64> = (0..=n).map(|i| t0+(t1-t0)*i as f64/n as f64).collect();
    let fs: Vec<f64> = ts.iter().map(|&t| f(t)).collect();
    if fs.iter().all(|v| v.abs() <= tol) { return Meets::Along }
    let mut roots: Vec<(f64,bool)> = Vec::new();
    let push = |t: f64,touch: bool,roots: &mut Vec<(f64,bool)>| {
        if let Some(last) = roots.last() { if (t-last.0).abs()*curve.speed() <= tol { return } }
        roots.push((t,touch));
    };
    if fs[0].abs() <= tol { push(t0,false,&mut roots); }
    for i in 0..n {
        let (a,b) = (fs[i],fs[i+1]);
        if a.abs() > tol && b.abs() > tol && (a < 0.) != (b < 0.) {
            let t = crate::roots::bracketed_root(|t| Some(f(t)),ts[i],a,ts[i+1],b,1e-15*(t1-t0).abs().max(1.));
            push(t,false,&mut roots);
        } else if i+1 < n && b.abs() > tol && b.abs() < a.abs() && b.abs() < fs[i+2].abs() && (a < 0.) == (b < 0.)
            && (b < 0.) == (fs[i+2] < 0.) {
            // a sampled minimum of |f| on one side: does it come down to the surface?
            let (t,v) = crate::roots::brent(&|t| f(t).abs(),ts[i],ts[i+2],1e-15*(t1-t0).abs().max(1.),200,|_,_| false);
            if v <= tol { push(t,true,&mut roots); }
        } else if b.abs() <= tol && i+1 < n {
            // a sample on the surface: a crossing there, or a touch
            let (before,after) = (a,fs[i+2]);
            let touch = before.abs() > tol && after.abs() > tol && (before < 0.) == (after < 0.);
            push(ts[i+1],touch,&mut roots);
        }
    }
    if fs[n].abs() <= tol { push(t1,false,&mut roots); }
    Meets::At(roots)
}

/// A boundary with its faces' loops drawn in their parameters, for classifying points against it.
pub struct Located<'a> {
    pub b: &'a Brep,
    pub tol: f64,
    /// How far each loop's polygon may stray from its edges (a length): points farther than this
    /// from every edge are classified by the polygon's winding, nearer ones by the side of the
    /// nearest edge.
    coarse: f64,
    /// Each face's loops as closed polygons in its parameters.
    polys: Vec<Vec<Vec<Uv>>>,
    /// Each face's parameter box.
    boxes: Vec<[Uv;2]>,
}

impl<'a> Located<'a> {
    pub fn new(b: &'a Brep,tol: f64) -> Located<'a> {
        let coarse = (1e-4*b.size()).max(tol*8.);
        let mut polys = Vec::new();
        let mut boxes = Vec::new();
        for f in &b.faces {
            let loops: Vec<Vec<Uv>> = f.loops.iter().map(|l| {
                let mut pts = Vec::new();
                for c in l {
                    let e = &b.edges[c.edge as usize];
                    let at = |t: f64| c.pcurve.at(t,e,&f.surface,&b.vertices);
                    let mut run = vec![(e.t[0],at(e.t[0]))];
                    let mut stack = vec![(e.t[0],e.t[1],0)];
                    // subdivide until each chord's middle maps within a share of `coarse` of the edge
                    while let Some((a,z,depth)) = stack.pop() {
                        let (ua,uz) = (at(a),at(z));
                        let m = (a+z)/2.;
                        let chord = [(ua[0]+uz[0])/2.,(ua[1]+uz[1])/2.];
                        let off = distance(f.surface.point(chord),e.point(m,&b.vertices));
                        if depth < 2 || (off > coarse/8. && depth < 20) { stack.push((m,z,depth+1)); stack.push((a,m,depth+1)); }
                        else { run.push((z,uz)); }
                    }
                    if c.reversed { run.reverse(); }
                    pts.extend(run[..run.len()-1].iter().map(|p| p.1));
                }
                pts
            }).collect();
            let mut bx = [[f64::INFINITY;2],[f64::NEG_INFINITY;2]];
            for l in &loops { for p in l { for k in 0..2 { bx[0][k] = bx[0][k].min(p[k]); bx[1][k] = bx[1][k].max(p[k]); } } }
            polys.push(loops);
            boxes.push(bx);
        }
        Located {b,tol,coarse,polys,boxes}
    }

    /// The parameters of a point of face `fi`'s surface, on the branch of its periods its loops
    /// are drawn on.
    pub fn uv(&self,fi: usize,p: V) -> Uv {
        let f = &self.b.faces[fi];
        let bx = self.boxes[fi];
        let mid = [(bx[0][0]+bx[1][0])/2.,(bx[0][1]+bx[1][1])/2.];
        unwrap(f.surface.inverse(p),mid,f.surface.periods())
    }

    /// The point of an edge nearest `p`: its parameter and distance.
    fn nearest(&self,e: u32,p: V) -> (f64,f64) {
        let e = &self.b.edges[e as usize];
        let EdgeCurve::Curve(c) = &e.curve else { return (e.t[0],distance(p,self.b.vertices[e.v[0] as usize].p)) };
        let mut t = c.inverse(p);
        if let Some(period) = c.period() {
            // the representative nearest the edge's stretch
            let mid = (e.t[0]+e.t[1])/2.;
            t += ((mid-t)/period).round()*period;
        }
        let mut best = (t.clamp(e.t[0],e.t[1]),0.);
        best.1 = distance(p,c.point(best.0));
        for end in e.t { let d = distance(p,c.point(end)); if d < best.1 { best = (end,d); } }
        best
    }

    /// Where a point of face `fi`'s surface stands against the face: `On` within `tol` of its
    /// boundary.
    pub fn face_place(&self,fi: usize,p: V) -> Place {
        let f: &Face = &self.b.faces[fi];
        let mut closest: Option<(f64,&super::topo::Coedge,f64)> = None;
        for l in &f.loops { for c in l {
            let (t,d) = self.nearest(c.edge,p);
            if closest.is_none_or(|(best,_,_)| d < best) { closest = Some((d,c,t)); }
        } }
        let Some((d,c,t)) = closest else { return Place::Out };
        if d <= self.tol { return Place::On }
        let uv = self.uv(fi,p);
        let e = &self.b.edges[c.edge as usize];
        if d <= self.coarse && t > e.t[0] && t < e.t[1] {
            // near one edge's interior: which side of it, in the face's parameters — the point
            // on the branch beside each use of the edge (a seam is used twice, once each side of
            // the face), inside if beside any use it is on the face's side
            let raw = f.surface.inverse(p);
            let inside = f.loops.iter().flatten().filter(|u| u.edge == c.edge).any(|u| {
                let q = u.pcurve.at(t,e,&f.surface,&self.b.vertices);
                let mut dq = u.pcurve.derivative(t,e,&f.surface,&self.b.vertices);
                if u.reversed { dq = [-dq[0],-dq[1]]; }
                let near = unwrap(raw,q,f.surface.periods());
                let left = dq[0]*(near[1]-q[1])-dq[1]*(near[0]-q[0]) > 0.;
                left != f.reversed
            });
            return if inside { Place::In } else { Place::Out }
        }
        let mut winding = 0i32;
        for l in &self.polys[fi] {
            let n = l.len();
            for i in 0..n {
                let (a,b) = (l[i],l[(i+1)%n]);
                if a[1] <= uv[1] {
                    if b[1] > uv[1] && (b[0]-a[0])*(uv[1]-a[1])-(uv[0]-a[0])*(b[1]-a[1]) > 0. { winding += 1; }
                } else if b[1] <= uv[1] && (b[0]-a[0])*(uv[1]-a[1])-(uv[0]-a[0])*(b[1]-a[1]) < 0. { winding -= 1; }
            }
        }
        // a reversed face's loops turn the other way round
        let inside = if f.reversed { winding < 0 } else { winding > 0 };
        if inside { Place::In } else { Place::Out }
    }

    /// Where a point of space stands against the solid: `On` within `tol` of a face, otherwise by
    /// the parity of the faces a ray from it crosses (a ray grazing an edge or touching a face is
    /// abandoned for another direction).
    pub fn solid_place(&self,p: V) -> Place {
        for (fi,f) in self.b.faces.iter().enumerate() {
            if f.surface.implicit(p).abs() <= self.tol && self.face_place(fi,p) != Place::Out { return Place::On }
        }
        let reach = 4.*self.b.size()+distance(p,self.b.vertices.first().map_or(p,|v| v.p));
        const DIRECTIONS: [V;5] = [[0.5773,0.6124,0.5402],[-0.3172,0.8123,-0.4893],[0.7071,-0.3162,0.6325],
            [-0.6412,-0.5123,0.5712],[0.1234,-0.4321,-0.8934]];
        'dirs: for d in DIRECTIONS {
            let d = scale(d,1./norm(d));
            let ray = Curve::Line {p,d};
            let mut crossings = 0;
            for (fi,f) in self.b.faces.iter().enumerate() {
                match curve_surface(&ray,[0.,reach],&f.surface,self.tol*1e-3) {
                    Meets::Along => continue 'dirs,
                    Meets::At(roots) => for (t,touch) in roots {
                        if t <= self.tol { continue }
                        let q = add(p,scale(d,t));
                        match self.face_place(fi,q) {
                            Place::Out => {}
                            Place::On => continue 'dirs,
                            Place::In => { if touch { continue 'dirs } crossings += 1; }
                        }
                    },
                }
            }
            return if crossings%2 == 1 { Place::In } else { Place::Out }
        }
        Place::On
    }
}

/// Whether an edge is degenerate (a pole), which classifies nothing.
pub fn degenerate(b: &Brep,e: u32) -> bool { matches!(b.edges[e as usize].curve,EdgeCurve::Degenerate) }
