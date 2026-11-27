//! The primitives, built with their topology whole: a profile swept along its normal (a prism,
//! drafted or not), turned about a line in its plane (a revolution), or a ball's disc screwed along
//! a helix (a canal), from the loops of lines, arcs, circles and B-splines the CAD recipe carries
//! (`solid::cad::recipe`); the sphere, the cone and the torus by their numbers (`sphere`,
//! `cone`, `torus`), each its meridian turned; and the convex hull of points (`hull`), a face for
//! each of its facets.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::geom::{tolerance,BSpline,Blend,Carry,Curve,Frame,Rigid,Surface,Uv,V};
use std::sync::Arc;
use super::topo::{reverse_loop,Brep,Coedge,EdgeCurve,Face,Pcurve};
use super::recipe::Writer;
use crate::util::space::{add,cross,distance,dot,norm,scale,sub,unit};
mod hull;
pub use hull::hull;

const TAU: f64 = std::f64::consts::TAU;

/// One edge of a profile: a line, an arc (a whole circle where `span` is none) turning
/// counter-clockwise about its frame's `z` (the profile's normal or against it: a builder reads
/// it facing the normal, `Seg::facing`), or a B-spline whose
/// ends are its first and last poles.
#[derive(Clone,Debug)]
pub enum ProfileEdge { Line { a: V,b: V },Arc { frame: Frame,r: f64,span: Option<[f64;2]> },Spline(Arc<BSpline>),
    /// A curve read exactly in a frame over its own span, in the profile's plane (`turned::Exact`,
    /// `Curve::exact_span`): a turned contact's meridian (#780 C3), revolved only, or a slid torus's
    /// apparent contour (#805), swept along the profile's normal only — each refused by the other;
    /// or an ellipse (`Curve::Ellipse`), its whole turn, a loop of its own, swept or carried only.
    Exact(Curve) }

/// A planar region: an outer loop and its holes, each a closed walk of edges (listed in any
/// order and direction).
#[derive(Clone,Debug)]
pub struct Profile { pub origin: V,pub normal: V,pub loops: Vec<Vec<ProfileEdge>>,
    /// Each loop's edges' names, as the document spells them (`block.side_l`'s `side_l`): the face
    /// a sweep makes of an edge is named by it, as the faceted kernel names it. Empty where unnamed.
    pub names: Vec<Vec<String>> }

impl Profile {
    /// The same section carried by `m`.
    pub fn moved(&self,m: &Rigid) -> Profile {
        let loops = self.loops.iter().map(|l| l.iter().map(|e| match e {
            ProfileEdge::Line {a,b} => ProfileEdge::Line {a:m.point(*a),b:m.point(*b)},
            ProfileEdge::Arc {frame,r,span} => ProfileEdge::Arc {frame:frame.moved(m),r:*r,span:*span},
            ProfileEdge::Spline(b) => {
                let mut b = (**b).clone();
                b.poles = b.poles.iter().map(|p| m.point(*p)).collect();
                ProfileEdge::Spline(Arc::new(b))
            }
            ProfileEdge::Exact(curve) => ProfileEdge::Exact(curve.moved(m)),
        }).collect()).collect();
        Profile {origin:m.point(self.origin),normal:m.vector(self.normal),loops,names:self.names.clone()}
    }
    /// The name of loop `l`'s edge `k`, or nothing.
    pub fn name(&self,l: usize,k: usize) -> String { self.names.get(l).and_then(|n| n.get(k)).cloned().unwrap_or_default() }
    /// A recipe's profile as this kernel reads it (millimetres): each arc's angles as the span it
    /// turns through counter-clockwise, its normal made unit.
    pub fn of(s: &super::recipe::Section) -> Result<Profile,String> {
        use super::recipe::Stroke;
        let mut loops = Vec::new();
        let mut names = Vec::new();
        for l in &s.loops {
            let mut edges = Vec::new();
            names.push(l.iter().map(|e| e.name.clone()).collect());
            for e in l {
                edges.push(match &e.shape {
                    &Stroke::Line {start,end} => ProfileEdge::Line {a:start,b:end},
                    &Stroke::Circle {center,normal,x_dir,radius,angles} => {
                        let frame = Frame::new(center,normal,x_dir);
                        let span = angles.map(|[a0,a1]| {
                            let sweep = (a1-a0).rem_euclid(TAU);
                            [a0,a0+if sweep == 0. { TAU } else { sweep }]
                        });
                        ProfileEdge::Arc {frame,r:radius,span}
                    }
                    Stroke::BSpline {degree,knots,poles,weights} => {
                        let (degree,knots,poles) = (*degree,knots.clone(),poles.clone());
                        let b = match weights {
                            Some(w) => BSpline::rational(degree,knots,poles,w.clone()),
                            None => BSpline::new(degree,knots,poles),
                        };
                        ProfileEdge::Spline(Arc::new(b.map_err(|m| format!("profile: {m}"))?))
                    }
                    Stroke::Trochoid {origin,x_dir,y_dir,terms,span} => {
                        let (x,y) = (unit(*x_dir),unit(*y_dir));
                        let frame = Frame {o:*origin,x,y,z:unit(cross(x,y))};
                        let terms = terms.iter().map(|&[ar,ai,br,bi,w]| crate::trochoid::Term {a:[ar,ai],b:[br,bi],w}).collect();
                        ProfileEdge::Exact(Curve::Trochoid(Arc::new(crate::trochoid::Trochoid {frame,terms,span:*span})))
                    }
                });
            }
            loops.push(edges);
        }
        Ok(Profile {origin:s.origin,normal:unit(s.normal),loops,names})
    }

    /// As a recipe writes it (`of`'s inverse; its lengths model units, written by `w`): lines,
    /// arcs, B-splines (their weights where rational) and trochoids, each named where it is; any
    /// other exact curve has no recipe's form.
    pub fn section(&self,w: &Writer) -> Result<super::recipe::Section,String> {
        use super::recipe::{Edge,Section,Stroke};
        let scale = w.scale;
        let at = |p: V| w.moved(p.map(|x| x*scale));
        let mut loops = Vec::new();
        for (l,edges) in self.loops.iter().enumerate() {
            let mut out = Vec::new();
            for (k,e) in edges.iter().enumerate() {
                let shape = match *e {
                    ProfileEdge::Line {a,b} => Stroke::Line {start:at(a),end:at(b)},
                    ProfileEdge::Arc {frame,r,span} => Stroke::Circle {center:at(frame.o),normal:frame.z,x_dir:frame.x,
                        radius:r*scale,angles:span},
                    ProfileEdge::Spline(ref b) => Stroke::BSpline {degree:b.degree,knots:b.knots.clone(),
                        poles:b.poles.iter().map(|&p| at(p)).collect(),weights:b.weights.clone()},
                    ProfileEdge::Exact(Curve::Trochoid(ref t)) => {
                        let f = &t.frame;
                        // (each term's `a` and `b` lengths, scaled as points are; its rate not)
                        let terms = t.terms.iter().map(|q| [q.a[0]*scale,q.a[1]*scale,q.b[0]*scale,q.b[1]*scale,q.w]).collect();
                        Stroke::Trochoid {origin:at(f.o),x_dir:f.x,y_dir:f.y,terms,span:t.span}
                    }
                    ProfileEdge::Exact(ref curve) => return Err(format!("a {} curve has no recipe's form",curve.kind())),
                };
                out.push(Edge {name:self.name(l,k),shape});
            }
            loops.push(out);
        }
        Ok(Section {origin:w.plane_origin(self.origin.map(|x| x*scale),self.normal),normal:self.normal,loops})
    }

    /// A length the profile's tolerances are taken against.
    fn size(&self) -> f64 {
        let mut s: f64 = 0.;
        for l in &self.loops { for e in l { match *e {
            ProfileEdge::Line {a,b} => s = s.max(norm(sub(a,self.origin))).max(norm(sub(b,self.origin))),
            ProfileEdge::Arc {frame,r,..} => s = s.max(norm(sub(frame.o,self.origin))+r),
            ProfileEdge::Spline(ref b) => for &q in &b.poles { s = s.max(norm(sub(q,self.origin))) },
            ProfileEdge::Exact(ref curve) => for q in exact_points(curve) { s = s.max(norm(sub(q,self.origin))) },
        } } }
        s.max(1e-300)
    }
}

/// An exact profile edge's span (`Curve::exact_span`, an ellipse's whole turn; a profile holds no
/// other curve as one).
fn exact_span(c: &Curve) -> [f64;2] {
    match c { Curve::Ellipse(..) => [0.,TAU],_ => c.exact_span().expect("a curve read exactly in a frame") }
}

/// Seventeen points evenly along an exact profile edge's span.
fn exact_points(c: &Curve) -> Vec<V> {
    let [a,b] = exact_span(c);
    (0..=16).map(|k| c.point(a+(b-a)*k as f64/16.)).collect()
}

/// One step of a walk: a curve from `t[0]` to `t[1]` (either way round), and which of its loop's
/// edges (in written order) it is.
#[derive(Clone,Debug)]
pub struct Seg { pub curve: Curve,pub t: [f64;2],pub source: usize }

impl Seg {
    pub fn start(&self) -> V { self.curve.point(self.t[0]) }
    pub fn end(&self) -> V { self.curve.point(self.t[1]) }
    pub fn forward(&self) -> bool { self.t[1] > self.t[0] }
    /// The step's direction along the walk at its parameter `t`, unit.
    fn along(&self,t: f64) -> V { unit(scale(self.curve.tangent(t),if self.forward() { 1. } else { -1. })) }
    /// Its side into its material at `t`, in a walk about `n` with its material on its left.
    fn inward(&self,t: f64,n: V) -> V { cross(n,self.along(t)) }
    fn reversed(&self) -> Seg { Seg {curve:self.curve.clone(),t:[self.t[1],self.t[0]],source:self.source} }
    /// The edge's parameters, increasing.
    pub fn span(&self) -> [f64;2] { if self.forward() { self.t } else { [self.t[1],self.t[0]] } }
    /// The step with its circle's frame facing `n` (its `y` and `z` turned over and its angles
    /// negated where `z` faces away), the same points the same way: what a side carried along `n`
    /// reads its cylinder's parameters and its side off (an arc a caller drew facing the other way,
    /// a section's arc mirrored across an axis, stood its corner cylinder 80 off its edges).
    pub fn facing(self,n: V) -> Seg {
        match self.curve {
            Curve::Circle(f,r) if dot(f.z,n) < 0. => Seg {curve:Curve::Circle(Frame {o:f.o,x:f.x,y:scale(f.y,-1.),z:scale(f.z,-1.)},r),
                t:[-self.t[0],-self.t[1]],source:self.source},
            _ => self,
        }
    }
    /// How long the step is: a line's run, an arc's radius times its turn, and any other curve
    /// its widest chord among seventeen points along it — nothing only where they all coincide.
    /// A length, whatever the curve's parameter measures (an arc's is an angle).
    pub fn length(&self) -> f64 {
        match self.curve {
            Curve::Line {..} => (self.t[1]-self.t[0]).abs(),
            Curve::Circle(_,r) => r.abs()*(self.t[1]-self.t[0]).abs(),
            _ => {
                let q: Vec<V> = (0..=16).map(|k| self.curve.point(self.t[0]+(self.t[1]-self.t[0])*k as f64/16.)).collect();
                q.iter().map(|a| q.iter().map(|b| distance(*a,*b)).fold(0.,f64::max)).fold(0.,f64::max)
            }
        }
    }
    pub(crate) fn samples(&self,n: usize) -> impl Iterator<Item = V> + '_ {
        (0..n).map(move |j| self.curve.point(self.t[0]+(self.t[1]-self.t[0])*j as f64/n as f64))
    }
}

/// **Whether an edge `length` long has no length** in a profile of `size` (its reach from its
/// origin, as `Profile::size` reads it): within the profile's tolerance of nothing, where no walk
/// can tell its ends apart.  The one test, which `walks` refuses a loop by and a host asks of the
/// edges it draws before it hands them over, so the refusal is said in the host's terms (#962).
pub fn no_length(length: f64,size: f64) -> bool { !(length > tolerance(size)) }

/// Each loop of a profile as a closed walk, the outer one counter-clockwise in `(x, y)` and the
/// holes clockwise, where `coords` reads a point's coordinates in the plane.
pub fn walks(p: &Profile,coords: &dyn Fn(V) -> [f64;2]) -> Result<Vec<Vec<Seg>>,String> {
    let size = p.size();
    let tol = tolerance(size);
    let mut out = Vec::new();
    for (li,l) in p.loops.iter().enumerate() {
        let segs: Vec<Seg> = l.iter().enumerate().map(|(source,e)| match *e {
            ProfileEdge::Line {a,b} => {
                let d = sub(b,a);
                Seg {curve:Curve::Line {p:a,d:unit(d)},t:[0.,norm(d)],source}
            }
            ProfileEdge::Arc {frame,r,span} => Seg {curve:Curve::Circle(frame,r),t:span.unwrap_or([0.,TAU]),source},
            ProfileEdge::Spline(ref b) => Seg {curve:Curve::BSpline(b.clone()),t:b.domain(),source},
            ProfileEdge::Exact(ref curve) => Seg {curve:curve.clone(),t:exact_span(curve),source},
        }).collect();
        if let Some(s) = segs.iter().find(|s| no_length(s.length(),size)) {
            let name = p.name(li,s.source);
            let edge = if name.is_empty() { format!("edge {}",s.source) } else { format!("edge `{name}`") };
            return Err(format!("loop {li}'s {edge} has no length"))
        }
        let mut used = vec![false;segs.len()];
        let mut walk = vec![segs[0].clone()];
        used[0] = true;
        while walk.len() < segs.len() {
            let end = walk.last().unwrap().end();
            let next = (0..segs.len()).filter(|&i| !used[i]).find_map(|i| {
                if distance(segs[i].start(),end) <= tol { Some((i,segs[i].clone())) }
                else if distance(segs[i].end(),end) <= tol { Some((i,segs[i].reversed())) }
                else { None }
            });
            let Some((i,s)) = next else { return Err(format!("loop {li} does not close: nothing continues it")) };
            used[i] = true;
            walk.push(s);
        }
        if distance(walk.last().unwrap().end(),walk[0].start()) > tol { return Err(format!("loop {li} does not close")) }
        let pts: Vec<[f64;2]> = walk.iter().flat_map(|s| s.samples(64)).map(|q| coords(q)).collect();
        let n = pts.len();
        let area = (0..n).map(|i| pts[i][0]*pts[(i+1)%n][1]-pts[i][1]*pts[(i+1)%n][0]).sum::<f64>()/2.;
        if area == 0. { return Err(format!("loop {li} encloses nothing")) }
        if (area > 0.) != (li == 0) { walk = walk.iter().rev().map(Seg::reversed).collect(); }
        out.push(walk);
    }
    Ok(out)
}

/// The use of a walk's step in the walk's direction (`along`) or against it.
fn use_of(edge: u32,s: &Seg,along: bool,pcurve: Pcurve) -> Coedge { Coedge {edge,reversed:s.forward() != along,pcurve} }

/// A plane's pcurve for an edge: its inverse (a plane has no branches).
fn on_plane() -> Pcurve { Pcurve::Inverse {a:[0.;2],b:[0.;2]} }

/// The profile swept along its normal from `from` to `to` (ordinates along the normal).
pub fn prism(p: &Profile,from: f64,to: f64) -> Result<Brep,String> {
    let (h0,h1) = (from.min(to),from.max(to));
    if !(h1 > h0) { return Err("a prism of no depth".into()) }
    let n = unit(p.normal);
    let plane = Frame::about(p.origin,n);
    let walks: Vec<Vec<Seg>> = walks(p,&|q| { let l = plane.local(q); [l[0],l[1]] })?
        .into_iter().map(|w| w.into_iter().map(|s| s.facing(n)).collect()).collect();
    let named = |li: usize,s: &Seg| p.name(li,s.source);
    let mut b = Brep::default();
    let lift = |q: V,h: f64| add(q,scale(n,h));
    let (mut bottom_loops,mut top_loops) = (Vec::new(),Vec::new());
    for (li,walk) in walks.iter().enumerate() {
        let m = walk.len();
        let bv: Vec<u32> = walk.iter().map(|s| b.vertex(lift(s.start(),h0))).collect();
        let tv: Vec<u32> = walk.iter().map(|s| b.vertex(lift(s.start(),h1))).collect();
        let up: Vec<u32> = (0..m).map(|k| b.edge(EdgeCurve::Curve(Curve::Line {p:walk[k].start(),d:n}),[h0,h1],[bv[k],tv[k]])).collect();
        let rim = |h: f64,v: &[u32],b: &mut Brep| -> Vec<u32> { (0..m).map(|k| {
            let s = &walk[k];
            let curve = s.curve.moved(&Rigid {r:Rigid::identity().r,t:scale(n,h)});
            let ends = [v[k],v[(k+1)%m]];
            b.edge(EdgeCurve::Curve(curve),s.span(),if s.forward() { ends } else { [ends[1],ends[0]] })
        }).collect() };
        let bottom = rim(h0,&bv,&mut b);
        let top = rim(h1,&tv,&mut b);
        for k in 0..m {
            let s = walk[k].clone();
            let k1 = (k+1)%m;
            let (surface,reversed,u0,u1) = match s.curve {
                Curve::Line {..} => {
                    let d = unit(sub(s.end(),s.start()));
                    (Surface::Plane(Frame::new(s.start(),cross(d,n),d)),false,0.,distance(s.end(),s.start()))
                }
                Curve::Circle(f,r) => (Surface::Cylinder(f,r),!s.forward(),s.t[0],s.t[1]),
                // C(u) + v n: the step's tangent turned clockwise about n is its normal (an ellipse's
                // closed round it, as a file's elliptic cylinder is)
                Curve::BSpline(_) | Curve::Slid(_) | Curve::Trochoid(_) | Curve::Ellipse(..) =>
                    (Surface::Extrusion(plane,Arc::new(s.curve.clone())),!s.forward(),s.t[0],s.t[1]),
                Curve::Turned(..) => return Err("a turned contact's meridian is turned, not swept along a line".into()),
                Curve::Traced(..) | Curve::Iso(..) | Curve::Rail(..) | Curve::Rim(..) | Curve::Affine(..)
                | Curve::SheetCut(..) | Curve::Parallel(..) =>
                    unreachable!("a profile has only lines, circles, splines, ellipses and curves read exactly"),
            };
            // the side's parameters along the walk step, for the step's edge parameter
            let u_at = |t: f64| match s.curve { Curve::Line {..} => (t-s.t[0]).abs(),_ => t };
            let [ta,tb] = s.span();
            let side = vec![
                use_of(bottom[k],&s,true,Pcurve::Line {a:[u_at(ta),h0],b:[u_at(tb),h0]}),
                Coedge {edge:up[k1],reversed:false,pcurve:Pcurve::Line {a:[u1,h0],b:[u1,h1]}},
                use_of(top[k],&s,false,Pcurve::Line {a:[u_at(ta),h1],b:[u_at(tb),h1]}),
                Coedge {edge:up[k],reversed:true,pcurve:Pcurve::Line {a:[u0,h0],b:[u0,h1]}},
            ];
            b.faces.push(Face {surface,reversed,loops:vec![side],name:named(li,&s)});
        }
        bottom_loops.push((0..m).rev().map(|k| use_of(bottom[k],&walk[k],false,on_plane())).collect::<Vec<_>>());
        top_loops.push((0..m).map(|k| use_of(top[k],&walk[k],true,on_plane())).collect::<Vec<_>>());
    }
    let cap = |h: f64| Surface::Plane(Frame {o:lift(plane.o,h),..plane});
    // named as the document names them: `near` the cap toward the viewer (along the normal), as
    // the faceted kernel and `against` read them
    b.faces.push(Face {surface:cap(h0),reversed:true,loops:bottom_loops,name:"far".into()});
    b.faces.push(Face {surface:cap(h1),reversed:false,loops:top_loops,name:"near".into()});
    Ok(b)
}

/// What names a drafted prism's section at its parting plane while its halves are built: gone once
/// they are glued there.
const PARTING: &str = "#parting";

/// The profile swept along its normal from `from` to `to`, its sides tapered by `draft` (radians)
/// from its own plane, the parting plane (#245): the section at ordinate `h` is the profile moved
/// `|h| tan draft` into its material (`offset`), so a straight side is a plane and a circle's a
/// cone, each leaning away from the plane either side of it. A prism crossing the plane is two
/// pieces (`ruled_by`), one each side, glued at the section they share; its sides there are two
/// faces of one name, its caps named as a prism's. Each corner's edge between a piece's sections is
/// the path the corner moves along (`Moving`): straight where two lines meet or the sides go on
/// tangent, and where an arc meets a side at an angle the conic its cone and the side's plane (or
/// the two cones) meet in, exact (#962). A section the draft takes a side to nothing in, or parts a
/// corner's sides in, is refused, saying how far from the parting plane it comes to that.
pub fn drafted(p: &Profile,from: f64,to: f64,draft: f64) -> Result<Brep,String> {
    if draft == 0. { return prism(p,from,to) }
    let (h0,h1) = (from.min(to),from.max(to));
    if !(h1 > h0) { return Err("a prism of no depth".into()) }
    let n = unit(p.normal);
    let taper = draft.dtan();
    let (walks,moving) = movements(p)?;
    // every section between the ends is offset no further than the farther end's is (the offset
    // grows with |h|); where that one fails, the ordinate's size at which the failure begins
    if let Some((what,at)) = away_with(p,&walks,&moving,h0.abs().max(h1.abs()),taper) {
        return Err(format!("a draft {what} {at:.4} from the parting plane, the plane its face is drawn in, within the \
            prism's ordinates ({h0} to {h1})"))
    }
    let section = |h: f64| -> Result<(f64,Profile),String> {
        let q = offset_with(p,&walks,&moving,h.abs()*taper)?;
        Ok((h,q.moved(&Rigid {r:Rigid::identity().r,t:scale(n,h)})))
    };
    let piece = |(a,start): &(f64,Profile),(b,end): &(f64,Profile)| -> Result<Brep,String> {
        // within a piece the offset runs `kappa` a unit of ordinate: one sign of `h`
        let kappa = if a+b < 0. { -taper } else { taper };
        let rail = |li: usize,source: usize| -> Result<Option<Curve>,String> {
            moving[li].get(source).ok_or("a drafted section's corner is not its profile's")?.path(n,kappa,[*a,*b])
        };
        let mut s = ruled_by(start,end,Carry::Line {delta:scale(n,b-a)},Some(&rail))?;
        let k = s.faces.len();
        s.faces[k-2].name = if *a == h0 { "far".into() } else { PARTING.into() };
        s.faces[k-1].name = if *b == h1 { "near".into() } else { PARTING.into() };
        Ok(s)
    };
    let (lo,hi) = (section(h0)?,section(h1)?);
    if !(h0 < 0. && h1 > 0.) { return piece(&lo,&hi) }
    let tol = tolerance(p.size()+h1-h0);
    let parting = section(0.)?;
    let b = super::boolean::glued(&piece(&lo,&parting)?,&piece(&parting,&hi)?,tol)
        .ok_or("a drafted prism's halves do not meet at its parting plane")?;
    if b.faces.iter().any(|f| f.name == PARTING) { return Err("a drafted prism's halves meet across more than its parting plane".into()) }
    Ok(b)
}

/// A side of a profile as a draft moves it at a corner: a line through the corner along `t`, its
/// material toward `inward`, moved `d` along `inward`; or a circle about `o` of radius `r`, its
/// material inside it (`sigma` 1) or outside (−1), its radius moved to `r − sigma d`.
#[derive(Clone,Copy,Debug)]
enum Carrier { Line { t: V,inward: V },Circle { o: V,r: f64,sigma: f64 } }

impl Carrier {
    /// Step `s`'s carrier at its parameter `t`, in a walk about `n` (its material on its left).
    fn of(s: &Seg,t: f64,n: V) -> Carrier {
        match s.curve {
            Curve::Circle(f,r) => Carrier::Circle {o:f.o,r,sigma:-dot(s.inward(t,n),unit(sub(s.curve.point(t),f.o)))},
            _ => Carrier::Line {t:s.along(t),inward:s.inward(t,n)},
        }
    }
}

/// **How a drafted section's corner moves as the offset `d` grows** (#245, #962): where its moved
/// sides meet, in the profile's plane.
///
/// - Two lines meet in a point moving along a line, and so do sides going on tangent (square to
///   both): `Straight`, `c + m d`.
/// - A line meeting a circle at an angle: with the line through the corner `c` along `t`, its
///   material toward `n`, and the circle about `o` of radius `r` moved to `r − σd`, the moved line's
///   point `c + d n + s t` is on the moved circle where `(s + b)² = b² − 2d((c − o)·n + σr)` (`b =
///   (c − o)·t`, `|c − o| = r`): the square of `y = s + b` affine in `d`, a parabola.
/// - Two circles meeting at an angle: subtracting one moved circle's equation from the other's
///   leaves the line square to their centres' `x = x₀ + x₁ d` (`σ² = 1`, so `d²` cancels), and across
///   it `y² = (r₁ − σ₁d)² − x²`, quadratic in `d`: an ellipse, a parabola or a hyperbola.
///
/// So each is `Conic`: `o + d k + y e`, `y = sign √(q₀ + q₁ d + q₂ d²)` on the branch through the
/// drawn corner, and the corner's path in space, `(corner(|h| tan θ), h)`, an affine image of `(d,
/// y)`, the same conic in a plane: where the side's plane meets the cone (or two cones meet), exactly.
/// Where the square comes to nothing the moved sides part, touching there and missing beyond.
#[derive(Clone,Copy,Debug)]
enum Moving { Straight { c: V,m: V },Conic { o: V,k: V,e: V,q: [f64;3],sign: f64 } }

impl Moving {
    /// The corner where step `b` starts, after step `a` ends, in a walk about `n` (its material on
    /// its left), and how it moves; `named` says a step as the author does.
    fn of(a: &Seg,b: &Seg,n: V,named: &dyn Fn(&Seg) -> String) -> Result<Moving,String> {
        let back = || format!("{} turns straight back at a corner",named(b));
        let c = b.start();
        let (ta,tb) = (a.along(a.t[1]),b.along(b.t[0]));
        if norm(cross(ta,tb)) <= 1e-9 {
            if dot(ta,tb) > 0. { return Ok(Moving::Straight {c,m:b.inward(b.t[0],n)}) }
            return Err(back())
        }
        Ok(match (Carrier::of(a,a.t[1],n),Carrier::of(b,b.t[0],n)) {
            (Carrier::Line {t:ta,inward:na},Carrier::Line {t:tb,inward:nb}) => {
                // the moved lines, each through the corner moved square to it, crossing: a unit of
                // offset along `m` (`w` is `|ta × tb|` above 1e-9, both square to `n`)
                let w = dot(cross(ta,tb),n);
                Moving::Straight {c,m:add(na,scale(ta,dot(cross(sub(nb,na),tb),n)/w))}
            }
            (Carrier::Line {t,inward},Carrier::Circle {o,r,sigma}) | (Carrier::Circle {o,r,sigma},Carrier::Line {t,inward}) => {
                let w = sub(c,o);
                let b = dot(w,t);
                Moving::Conic {o:sub(c,scale(t,b)),k:inward,e:t,q:[b*b-dot(w,w)+r*r,-2.*(dot(w,inward)+sigma*r),0.],sign:b.signum()}
            }
            (Carrier::Circle {o:o1,r:r1,sigma:s1},Carrier::Circle {o:o2,r:r2,sigma:s2}) => {
                let between = sub(o2,o1);
                let dd = norm(between);
                // (concentric within the tolerance, their tangents can still stand up to about 2e-9 apart)
                if dd <= tolerance(r1+r2) { return Err(back()) }
                let e = scale(between,1./dd);
                let across = cross(n,e);
                let (x0,x1) = ((dd*dd+r1*r1-r2*r2)/(2.*dd),-(s1*r1-s2*r2)/dd);
                Moving::Conic {o:add(o1,scale(e,x0)),k:scale(e,x1),e:across,q:[r1*r1-x0*x0,-2.*s1*r1-2.*x0*x1,1.-x1*x1],
                    sign:dot(sub(c,o1),across).signum()}
            }
        })
    }

    /// The square `y²` at offset `d` (a conic's), and its rate.
    fn square(q: [f64;3],d: f64) -> (f64,f64) { (q[0]+d*(q[1]+d*q[2]),q[1]+2.*d*q[2]) }

    /// The corner moved `d`, or none where its moved sides have parted (or touch: no corner either
    /// side of it then).
    fn at(&self,d: f64) -> Option<V> {
        match *self {
            Moving::Straight {c,m} => Some(add(c,scale(m,d))),
            Moving::Conic {o,k,e,q,sign} => {
                let (y2,_) = Moving::square(q,d);
                (y2 > 0.).then(|| add(add(o,scale(k,d)),scale(e,sign*y2.sqrt())))
            }
        }
    }

    /// The corner's rate per unit of offset at `d` (none where it has none: its sides part there).
    fn rate(&self,d: f64) -> Option<V> {
        match *self {
            Moving::Straight {m,..} => Some(m),
            Moving::Conic {k,e,q,sign,..} => {
                let (y2,dy2) = Moving::square(q,d);
                (y2 > 0.).then(|| add(k,scale(e,sign*dy2/(2.*y2.sqrt()))))
            }
        }
    }

    /// Whether the moved sides keep meeting all the way between offsets `d0` and `d1`: the square
    /// stays above nothing at both and, where it turns between them, there too.
    fn holds(&self,d0: f64,d1: f64) -> bool {
        let Moving::Conic {q,..} = *self else { return true };
        let (lo,hi) = (d0.min(d1),d0.max(d1));
        let low = if q[2] > 0. { (-q[1]/(2.*q[2])).clamp(lo,hi) } else { lo };
        [lo,hi,low].iter().all(|&d| Moving::square(q,d).0 > 0.)
    }

    /// **The corner's edge between ordinates `h[0]` and `h[1]`** (one side of the parting plane, the
    /// offset `kappa h` along them), from the first to the second as the curve's parameter runs 0 to
    /// 1: none where it is straight (the caller's line between the sections' corners), else the
    /// conic as a rational quadratic Bézier — its ends the corner at each ordinate, its middle pole
    /// where their tangents cross, its weight `w` read off the corner halfway, whose barycentric
    /// coordinates `(α, β, γ)` against the poles have `β² = 4 w² α γ` on any such curve. A line
    /// meeting a circle moves along a parabola, which is the polynomial Bézier whose middle pole is
    /// `P₀ + Δh P′(h₀) |y₀| / (|y₀| + |y₁|)` (with `u = |y|` the parabola is quadratic in `u`, and `Δu
    /// = Δy²/(u₀ + u₁)`), read without the crossing of two near-parallel tangents.
    fn path(&self,n: V,kappa: f64,h: [f64;2]) -> Result<Option<Curve>,String> {
        let Moving::Conic {q,..} = *self else { return Ok(None) };
        let parted = || "a draft parts a corner's sides between its sections".to_string();
        if !self.holds(kappa*h[0],kappa*h[1]) { return Err(parted()) }
        let point = |h: f64| self.at(kappa*h).map(|c| add(c,scale(n,h))).ok_or_else(parted);
        let tangent = |h: f64| self.rate(kappa*h).map(|r| add(scale(r,kappa),n)).ok_or_else(parted);
        let (p0,p2) = (point(h[0])?,point(h[1])?);
        let (t0,t2) = (tangent(h[0])?,tangent(h[1])?);
        let curve = |poles: Vec<V>,weights: Option<Vec<f64>>| -> Result<Option<Curve>,String> {
            let knots = vec![0.,0.,0.,1.,1.,1.];
            let b = match weights { Some(w) => BSpline::rational(2,knots,poles,w),None => BSpline::new(2,knots,poles) };
            Ok(Some(Curve::BSpline(Arc::new(b.map_err(|m| format!("a drafted corner's edge: {m}"))?))))
        };
        if q[2] == 0. {
            let u = |h: f64| Moving::square(q,kappa*h).0.sqrt();
            let (u0,u1) = (u(h[0]),u(h[1]));
            return curve(vec![p0,add(p0,scale(t0,(h[1]-h[0])*u0/(u0+u1))),p2],None)
        }
        // the tangents' crossing, by least squares in the conic's plane
        let chord = sub(p2,p0);
        let (aa,ab,bb) = (dot(t0,t0),-dot(t0,t2),dot(t2,t2));
        let (ra,rb) = (dot(t0,chord),-dot(t2,chord));
        let det = aa*bb-ab*ab;
        let mid = point((h[0]+h[1])/2.)?;
        // a path turning less than its numbers can say is its chord
        let off = norm(sub(sub(mid,p0),scale(chord,dot(sub(mid,p0),chord)/dot(chord,chord))));
        if !(det > 1e-24*aa*bb) || off <= 1e-12*norm(chord) { return Ok(None) }
        let (lam,mu) = ((ra*bb-ab*rb)/det,(aa*rb-ab*ra)/det);
        let p1 = scale(add(add(p0,scale(t0,lam)),add(p2,scale(t2,mu))),0.5);
        // the middle point's barycentric coordinates against the poles, by least squares
        let (e0,e2,s) = (sub(p0,p1),sub(p2,p1),sub(mid,p1));
        let (g00,g02,g22) = (dot(e0,e0),dot(e0,e2),dot(e2,e2));
        let g = g00*g22-g02*g02;
        let (al,ga) = ((dot(s,e0)*g22-dot(s,e2)*g02)/g,(dot(s,e2)*g00-dot(s,e0)*g02)/g);
        let be = 1.-al-ga;
        let w = be/(2.*(al*ga).sqrt());
        if !(lam > 0. && mu < 0. && al > 0. && ga > 0. && w.is_finite() && w > 0.) {
            return Err("a drafted corner's edge is no conic arc between its sections".into())
        }
        curve(vec![p0,p1,p2],Some(vec![1.,w,1.]))
    }
}

/// Each loop of a profile walked with its material on its left (`walks` in its own plane), and how
/// each corner moves as a draft offsets it (`Moving`), the one where edge `source` starts at
/// `source`. Refused for a spline or an exact curve, and a walk turning straight back.
fn movements(p: &Profile) -> Result<(Vec<Vec<Seg>>,Vec<Vec<Moving>>),String> {
    let n = unit(p.normal);
    let frame = Frame::about(p.origin,n);
    let walks = walks(p,&|q| { let l = frame.local(q); [l[0],l[1]] })?;
    let mut moving = Vec::new();
    for (li,walk) in walks.iter().enumerate() {
        let m = walk.len();
        let named = |s: &Seg| said(&p.name(li,s.source));
        for s in walk {
            if let ProfileEdge::Exact(ref curve) = p.loops[li][s.source] { return Err(format!("a {} curve is not drafted",curve.kind())) }
            if !matches!(s.curve,Curve::Line {..} | Curve::Circle(..)) {
                return Err(format!("{}: a drafted side of a spline is not built yet",named(s)))
            }
        }
        // each corner, by the edge that starts there (a walk steps along every edge of its loop once)
        let mut corners = (0..m).map(|k| Ok((walk[k].source,Moving::of(&walk[(k+m-1)%m],&walk[k],n,&named)?)))
            .collect::<Result<Vec<_>,String>>()?;
        corners.sort_by_key(|&(source,_)| source);
        moving.push(corners.into_iter().map(|(_,c)| c).collect());
    }
    Ok((walks,moving))
}

/// Whether every corner of the profile moves along a line as a draft offsets it — where two lines
/// meet, or the sides go on tangent — so that a section between two is their blend, as a drafted
/// tool's sections are read (`swept::cutter`); not where an arc meets a side at an angle (a conic).
pub fn corners_move_straight(p: &Profile) -> Result<bool,String> {
    Ok(movements(p)?.1.iter().flatten().all(|m| matches!(m,Moving::Straight {..})))
}

/// The profile moved `d` into its material, each side along its own normal (#245): a line moved
/// square to itself, a circle's radius less `d` where its material is inside it (more where it is
/// outside, a hole's), each corner where the moved sides either side of it meet (`Moving`): two
/// lines' crossing, square to both where the sides go on tangent, and where an arc meets a side at
/// an angle the point the moved line and circle (or the two moved circles) meet at, on the branch
/// through the drawn corner — at a reflex corner as at a convex one, the sides' carriers met beyond
/// their ends as two lines' are. Refused for a spline, a side the move takes to nothing, and a
/// corner whose moved sides part.
pub fn offset(p: &Profile,d: f64) -> Result<Profile,String> {
    if d == 0. { return Ok(p.clone()) }
    let (walks,moving) = movements(p)?;
    offset_with(p,&walks,&moving,d)
}

/// `offset`, the profile's walks and corners' movements (`movements`) in hand.
fn offset_with(p: &Profile,walks: &[Vec<Seg>],moving: &[Vec<Moving>],d: f64) -> Result<Profile,String> {
    moved_with(p,walks,moving,d).map_err(|f| format!("a draft {}",f.said("")))
}

/// Why a profile is not moved: a side the move takes to nothing, or a corner whose moved sides
/// part (each side by its name, empty where unnamed).
enum Away { Gone(String),Parts(String,String) }

impl Away {
    /// What the move does, in the author's terms: `takes `x` to nothing` (`side` before a named
    /// side, `takes the side `x` to nothing`), `parts `a` from `b` at their corner`; a side unnamed
    /// is `a side`.
    fn said(&self,side: &str) -> String {
        match self {
            Away::Gone(s) if s.is_empty() => format!("takes {} to nothing",said(s)),
            Away::Gone(s) => format!("takes {side}{} to nothing",said(s)),
            Away::Parts(a,b) => format!("parts {} from {} at their corner",said(a),said(b)),
        }
    }
}

/// A side's name as a refusal says it.
fn said(name: &str) -> String { if name.is_empty() { "a side".into() } else { format!("`{name}`") } }

/// **Where a draft first takes the profile's sides to nothing or parts a corner's**, moving it
/// `taper` a unit of ordinate as far as ordinate `reach` (its size): what it does, in the author's
/// terms (`takes the side `x` to nothing`, `parts `a` from `b` at their corner`), and the ordinate's
/// size at which it begins — none where every section within `reach` keeps its sides and corners.
/// The section's rule is `offset`'s, the one the kernel lofts and the field reads, so a corner
/// sliding along an arc eats the arc as the set does, never a tessellation's chords. Refused, as
/// `offset` refuses it, for a profile the rule does not move at all (a spline, an exact curve, a
/// walk turning straight back).
pub fn drafted_away(p: &Profile,reach: f64,taper: f64) -> Result<Option<(String,f64)>,String> {
    let (walks,moving) = movements(p)?;
    Ok(away_with(p,&walks,&moving,reach,taper))
}

/// `drafted_away`, the profile's walks and corners' movements in hand.
fn away_with(p: &Profile,walks: &[Vec<Seg>],moving: &[Vec<Moving>],reach: f64,taper: f64) -> Option<(String,f64)> {
    let moved = |h: f64| moved_with(p,walks,moving,h*taper);
    let Err(away) = moved(reach) else { return None };
    // the ordinate's size where the sections begin to fail, halved until the middle stops moving
    // (or sixty times: a corner parting at once is not followed into the subnormals)
    let (ok,bad) = crate::util::roots::bisect(0.,reach,|a,b| (a+b)/2.,
        |a,b| { let mid = (a+b)/2.; mid > a && mid < b && b-a > reach*2f64.powi(-60) },|h| moved(h).is_ok());
    Some((away.said("the side "),(ok+bad)/2.))
}

/// `offset`, the profile's walks and corners' movements in hand, saying why not as `Away`.
fn moved_with(p: &Profile,walks: &[Vec<Seg>],moving: &[Vec<Moving>],d: f64) -> Result<Profile,Away> {
    if d == 0. { return Ok(p.clone()) }
    let n = unit(p.normal);
    let tol = tolerance(p.size());
    let mut out = p.clone();
    for (li,walk) in walks.iter().enumerate() {
        let m = walk.len();
        let named = |s: &Seg| p.name(li,s.source);
        // each corner, where step `k` starts
        let corners: Vec<V> = (0..m).map(|k| {
            let corner = &moving[li][walk[k].source];
            corner.at(d).filter(|_| corner.holds(0.,d)).ok_or_else(|| Away::Parts(named(&walk[(k+m-1)%m]),named(&walk[k])))
        }).collect::<Result<_,_>>()?;
        for (k,s) in walk.iter().enumerate() {
            let (start,end) = (corners[k],corners[(k+1)%m]);
            let gone = || Away::Gone(named(s));
            out.loops[li][s.source] = match p.loops[li][s.source] {
                ProfileEdge::Line {a,b} => {
                    let (a2,b2) = if s.forward() { (start,end) } else { (end,start) };
                    if dot(sub(b2,a2),sub(b,a)) <= tol*tol { return Err(gone()) }
                    ProfileEdge::Line {a:a2,b:b2}
                }
                ProfileEdge::Arc {frame:f,r,span} => {
                    let Carrier::Circle {sigma,..} = Carrier::of(s,s.t[0],n) else { unreachable!("an arc's carrier is its circle") };
                    let r2 = r-d*sigma;
                    if !(r2 > tol) { return Err(gone()) }
                    let span = match span {
                        None => None,
                        Some([a0,a1]) => {
                            let angle = |q: V,near: f64| super::geom::around(Curve::Circle(f,r2).inverse(q),near-0.5*TAU,TAU);
                            let (q0,q1) = if s.forward() { (start,end) } else { (end,start) };
                            let (b0,b1) = (angle(q0,a0),angle(q1,a1));
                            if !(b1 > b0) { return Err(gone()) }
                            Some([b0,b1])
                        }
                    };
                    ProfileEdge::Arc {frame:f,r:r2,span}
                }
                ProfileEdge::Spline(_) | ProfileEdge::Exact(_) => unreachable!("refused by `movements`"),
            };
        }
    }
    Ok(out)
}

/// What a profile step sweeps as it turns about the axis.
#[derive(Clone,Copy,Debug,PartialEq)]
enum Swept { Axis,Disk,Cylinder,Cone,Sphere,Torus,Spline }

/// The profile turned about the line through `origin` along `axis` by `angle` (right-handed; a
/// negative angle turns the other way, and anything past a turn is a whole one).
pub fn revolve(p: &Profile,origin: V,axis: V,angle: f64) -> Result<Brep,String> {
    let z = scale(unit(axis),angle.signum());
    let theta = angle.abs().min(TAU);
    if !(theta > 0.) { return Err("a revolution through no angle".into()) }
    let full = theta >= TAU-1e-12;
    let tol = tolerance(p.size());
    let normal = unit(p.normal);
    if dot(normal,z).abs() > 1e-9 || dot(sub(origin,p.origin),normal).abs() > tol {
        return Err("a revolution's axis must lie in its profile's plane".into())
    }
    // x: square to the axis in the profile's plane, toward the profile
    let mut x = unit(cross(z,normal));
    let far = p.loops.iter().flatten().flat_map(|e| match *e {
        ProfileEdge::Line {a,b} => vec![a,b],
        // points along the arc itself (its frame's x may lie along the axis: a semicircle's ends)
        ProfileEdge::Arc {frame,r,span} => {
            let [a0,a1] = span.unwrap_or([0.,TAU]);
            (0..=16).map(|k| { let t = a0+(a1-a0)*k as f64/16.; add(frame.o,add(scale(frame.x,r*t.dcos()),scale(frame.y,r*t.dsin()))) }).collect()
        }
        ProfileEdge::Spline(ref b) => b.poles.clone(),
        ProfileEdge::Exact(ref curve) => exact_points(curve),
    }).map(|q| dot(sub(q,origin),x)).fold(0.,|m: f64,v| if v.abs() > m.abs() { v } else { m });
    if far < 0. { x = scale(x,-1.); }
    let f = Frame::new(origin,z,x);
    let rz = |q: V| { let l = f.local(q); [l[0],l[2]] };
    let walks = walks(p,&rz)?;
    for s in walks.iter().flatten() {
        // (what a revolution turns: a line, a circle, a spline, a turned contact's meridian)
        if !matches!(s.curve,Curve::Line {..} | Curve::Circle(..) | Curve::BSpline(_) | Curve::Turned(_)) {
            return Err(format!("a {} curve is not turned",s.curve.kind()))
        }
        if s.samples(64).chain([s.end()]).any(|q| rz(q)[0] < -tol) { return Err("a revolution's profile crosses its axis".into()) }
    }
    let turn = Rigid::turn(origin,z,theta);
    let mut b = Brep::default();
    let (mut cap0,mut cap1) = (Vec::new(),Vec::new());
    for (li,walk) in walks.iter().enumerate() {
        let m = walk.len();
        let pts: Vec<V> = walk.iter().map(|s| s.start()).collect();
        let on_axis: Vec<bool> = pts.iter().map(|&q| rz(q)[0] <= tol).collect();
        let v0: Vec<u32> = pts.iter().map(|&q| b.vertex(q)).collect();
        let v1: Vec<u32> = (0..m).map(|k| if full || on_axis[k] { v0[k] } else { b.vertex(turn.point(pts[k])) }).collect();
        let kinds: Vec<Swept> = walk.iter().enumerate().map(|(k,s)| {
            // an end on the axis to the tolerance is on it: a cone placed at its apex has radius
            // zero there, never the rounding either side of it (a negative one no reader takes)
            let on = |r: f64| if r.abs() <= tol { 0. } else { r };
            let ([r0,z0],[r1,z1]) = (rz(s.start()),rz(s.end()));
            let (r0,r1) = (on(r0),on(r1));
            match &s.curve {
                Curve::Line {..} if on_axis[k] && on_axis[(k+1)%m] => Swept::Axis,
                Curve::Line {..} if (z0-z1).abs() <= tol => Swept::Disk,
                Curve::Line {..} if (r0-r1).abs() <= tol => Swept::Cylinder,
                Curve::Line {..} => Swept::Cone,
                Curve::Circle(c,_) if rz(c.o)[0].abs() <= tol => Swept::Sphere,
                Curve::BSpline(_) | Curve::Turned(_) => Swept::Spline,
                Curve::Circle(..) => Swept::Torus,
                _ => unreachable!("refused above"),
            }
        }).collect();
        // the circle each profile vertex off the axis turns through
        let circles: Vec<Option<u32>> = (0..m).map(|k| (!on_axis[k]).then(|| {
            let [r,h] = rz(pts[k]);
            let c = Curve::Circle(Frame {o:f.at([0.,0.,h]),..f},r);
            b.edge(EdgeCurve::Curve(c),[0.,theta],[v0[k],v1[k]])
        })).collect();
        // each step where it starts (φ = 0) and, turned whole, where it ends (φ = θ)
        let needs = |k: usize| !full || !matches!(kinds[k],Swept::Disk | Swept::Axis);
        let first: Vec<Option<u32>> = (0..m).map(|k| needs(k).then(|| {
            let s = &walk[k];
            let ends = [v0[k],v0[(k+1)%m]];
            b.edge(EdgeCurve::Curve(s.curve.clone()),s.span(),if s.forward() { ends } else { [ends[1],ends[0]] })
        })).collect();
        let last: Vec<Option<u32>> = (0..m).map(|k| if full || kinds[k] == Swept::Axis { first[k] } else {
            let s = &walk[k];
            let ends = [v1[k],v1[(k+1)%m]];
            Some(b.edge(EdgeCurve::Curve(s.curve.moved(&turn)),s.span(),if s.forward() { ends } else { [ends[1],ends[0]] }))
        }).collect();
        for k in 0..m {
            let s = walk[k].clone();
            let k1 = (k+1)%m;
            // an end on the axis to the tolerance is on it: a cone placed at its apex has radius
            // zero there, never the rounding either side of it (a negative one no reader takes)
            let on = |r: f64| if r.abs() <= tol { 0. } else { r };
            let ([r0,z0],[r1,z1]) = (rz(s.start()),rz(s.end()));
            let (r0,r1) = (on(r0),on(r1));
            // the material's outward normal at the step's middle, in (ρ, z): right of the walk
            let mid = (s.t[0]+s.t[1])/2.;
            let (q,dq) = (s.curve.point(mid),s.curve.tangent(mid));
            let d = { let l = f.dir_local(dq); [l[0],l[2]] };
            let sense = if s.forward() { 1. } else { -1. };
            let out = [d[1]*sense,-d[0]*sense];
            let [rq,zq] = rz(q);
            let (surface,normal,v_of): (Surface,[f64;2],Box<dyn Fn(f64) -> f64>) = match kinds[k] {
                Swept::Axis => continue,
                Swept::Disk => (Surface::Plane(Frame {o:f.at([0.,0.,z0]),..f}),[0.,1.],Box::new(|_| 0.)),
                Swept::Cylinder => { let c = s.curve.clone(); (Surface::Cylinder(f,r0),[1.,0.],Box::new(move |t| rz(c.point(t))[1])) }
                Swept::Cone => {
                    let a = ((r1-r0)/(z1-z0)).datan();
                    (Surface::Cone(Frame {o:f.at([0.,0.,z0]),..f},r0,a),[a.dcos(),-a.dsin()],
                        { let c = s.curve.clone(); Box::new(move |t| (rz(c.point(t))[1]-z0)/a.dcos()) })
                }
                Swept::Spline => {
                    // C(v) turned by u: in (ρ, z) its normal is the tangent turned clockwise
                    let l = d[0].dhypot(d[1]);
                    (Surface::Revolution(f,Arc::new(s.curve.clone())),[d[1]/l,-d[0]/l],Box::new(|t| t))
                }
                Swept::Sphere | Swept::Torus => {
                    let Curve::Circle(c,r) = s.curve.clone() else { unreachable!() };
                    let [rc,zc] = rz(c.o);
                    let big = if kinds[k] == Swept::Sphere { 0. } else { rc };
                    let surface = if kinds[k] == Swept::Sphere { Surface::Sphere(Frame {o:f.at([0.,0.,zc]),..f},r) }
                        else { Surface::Torus(Frame {o:f.at([0.,0.,zc]),..f},big,r) };
                    // v at the step's start, and which way it runs with the circle's parameter
                    let [ra,za] = rz(s.start());
                    let va = (za-zc).datan2(ra-big);
                    let dt = { let l = f.dir_local(s.curve.tangent(s.t[0])); [l[0],l[2]] };
                    let sigma = ((ra-big)*dt[1]-(za-zc)*dt[0]).signum();
                    let t0 = s.t[0];
                    // a sphere's v held to its poles: run a rounding past one, its chart turns over
                    let pole = if kinds[k] == Swept::Sphere { std::f64::consts::FRAC_PI_2 } else { f64::INFINITY };
                    (surface,[(rq-big)/r,(zq-zc)/r],Box::new(move |t| (va+sigma*(t-t0)).clamp(-pole,pole)))
                }
            };
            let reversed = out[0]*normal[0]+out[1]*normal[1] < 0.;
            let [ta,tb] = s.span();
            let mut l = Vec::new();
            if kinds[k] == Swept::Disk {
                if full {
                    // an annulus or a disk: the outer circle, the inner as a hole
                    let (outer,inner) = if r0 > r1 { (k,k1) } else { (k1,k) };
                    let rim = |i: usize,outer: bool| Coedge {edge:circles[i].unwrap(),reversed:(outer == reversed),pcurve:on_plane()};
                    let mut loops = vec![vec![rim(outer,true)]];
                    if let Some(_) = circles[inner] { loops.push(vec![rim(inner,false)]); }
                    b.faces.push(Face {surface,reversed,loops,name:p.name(li,s.source)});
                    continue;
                }
                l.push(use_of(first[k].unwrap(),&s,true,on_plane()));
                if let Some(c) = circles[k1] { l.push(Coedge {edge:c,reversed:false,pcurve:on_plane()}); }
                l.push(use_of(last[k].unwrap(),&s,false,on_plane()));
                if let Some(c) = circles[k] { l.push(Coedge {edge:c,reversed:true,pcurve:on_plane()}); }
            } else {
                let (va,vb) = (v_of(ta),v_of(tb));
                let (vs,ve) = (v_of(s.t[0]),v_of(s.t[1]));
                let rim = |i: usize,v: f64,reversed: bool,b: &mut Brep| -> Coedge {
                    let pcurve = Pcurve::Line {a:[0.,v],b:[theta,v]};
                    match circles[i] {
                        Some(c) => Coedge {edge:c,reversed,pcurve},
                        None => Coedge {edge:b.edge(EdgeCurve::Degenerate,[0.,theta],[v0[i],v0[i]]),reversed,pcurve},
                    }
                };
                l.push(use_of(first[k].unwrap(),&s,true,Pcurve::Line {a:[0.,va],b:[0.,vb]}));
                l.push(rim(k1,ve,false,&mut b));
                l.push(use_of(last[k].unwrap(),&s,false,Pcurve::Line {a:[theta,va],b:[theta,vb]}));
                l.push(rim(k,vs,true,&mut b));
            }
            let ccw = b.loop_area(&Face {surface:surface.clone(),reversed,loops:vec![],name:String::new()},&l) > 0.;
            if ccw == reversed { l = reverse_loop(l); }
            b.faces.push(Face {surface,reversed,loops:vec![l],name:p.name(li,s.source)});
        }
        if !full {
            cap0.push((0..m).map(|k| use_of(first[k].unwrap(),&walk[k],true,on_plane())).collect::<Vec<_>>());
            cap1.push((0..m).rev().map(|k| use_of(last[k].unwrap(),&walk[k],false,on_plane())).collect::<Vec<_>>());
        }
    }
    if !full {
        b.faces.push(Face {surface:Surface::Plane(Frame::new(origin,cross(x,z),x)),reversed:false,loops:cap0,name:"start".into()});
        let fx = turn.vector(x);
        b.faces.push(Face {surface:Surface::Plane(Frame::new(origin,cross(fx,z),fx)),reversed:true,loops:cap1,name:"end".into()});
    }
    Ok(b)
}

/// **The tube a ball sweeps** (#238): the ball of radius `r` about `c` carried by the screw about the
/// line through `origin` along `axis` — a turn by `angle` and a step of `advance` along it (none, a
/// turn; no turn, a slide, #684 D) — is the points within `r` of the path its centre runs, and
/// those are the ball's disc square to the path carried along it (a point's nearest point on the
/// path inside it lies in the disc there) with the ball where the path starts and where it ends:
/// the disc turned (a torus's stretch, a whole torus past a whole turn), carried along the helix to
/// its own image there (`ruled`, a screw's `Blend`) or along the line (a cylinder), each end's
/// half ball beyond its disc united with it. Every face is named `name`. The caller's to know the
/// tube meets itself nowhere: the path's centre stands off the axis by more than `r`, and a helix
/// of more than a turn climbs more than `2 r` a turn.
pub fn canal(c: V,r: f64,origin: V,axis: V,angle: f64,advance: f64,name: &str) -> Result<Brep,String> {
    let a = unit(axis);
    let radial = sub(c,add(origin,scale(a,dot(sub(c,origin),a))));
    // the centre's velocity at the start, square to the radial line
    let rate = add(scale(cross(a,radial),angle),scale(a,advance));
    // a centre that does not move sweeps the ball itself
    if norm(rate) <= 1e-12*r.max(1.) { return Ok(named(sphere(c,a,r)?,name)) }
    let t = unit(rate);
    // where the disc's circle starts, so where the tube's seams run: turned off the radial line by
    // an angle no design's symmetry gives (on it, the arc the seam's vertex sweeps lies in the plane
    // of the centre's path, which a face often does — a groove's floor's block top)
    let x = seam(t,if norm(radial) > 0. { unit(radial) } else { Frame::about(c,t).x });
    let w = poles(t,x);
    let disc = Profile {origin:c,normal:t,loops:vec![vec![ProfileEdge::Arc {frame:Frame::new(c,t,x),r,span:None}]],names:Vec::new()};
    if advance == 0. && angle.abs() >= TAU-1e-12 {
        return Ok(named(revolve(&disc,origin,a,TAU*angle.signum())?,name))
    }
    let end = screw(origin,a,angle,advance);
    let tube = if advance == 0. { revolve(&disc,origin,a,angle)? }
        else if angle == 0. { ruled(&disc,&disc.moved(&end),Carry::Line {delta:scale(a,advance)})? }
        else { ruled(&disc,&disc.moved(&end),Carry::Screw {center:origin,axis:a,angle,advance})? };
    ended(tube,half_ball(c,scale(t,-1.),w,r)?,half_ball(c,t,w,r)?.moved(&end),name)
}

/// **The solid `p` sweeps screwed about the line through `origin` along `axis`**: turned by `angle`
/// about it and stepped `advance` along it, both together (a screw, neither none). Its faces are the
/// section where it starts and where it ends and each edge's sheet between them, its rails the
/// screw's iso-curves (`ruled`'s `Blend`s), exact. The caller's to know it meets itself nowhere: a
/// section square to the axis never does, a helix crossing each level once.
pub fn screwed(p: &Profile,origin: V,axis: V,angle: f64,advance: f64) -> Result<Brep,String> {
    if angle == 0. || advance == 0. { return Err("a screw turns and steps both".into()) }
    let a = unit(axis);
    ruled(p,&p.moved(&screw(origin,a,angle,advance)),Carry::Screw {center:origin,axis:a,angle,advance})
}

/// **The solid `p` sweeps carried by `carry`** (#846): a screw's (`screwed`), a turn's — a screw of
/// no advance, its axis in the profile's plane (`revolve`) — or a slide's, a step along a line
/// crossing the profile's plane (`ruled`). Its faces are the section where it starts (`start`) and
/// where it ends (`end`) and each edge's sheet between them, named by the edge. The caller's to know
/// it meets itself nowhere: a section square to a screw's axis never does, nor one a slide crosses.
pub fn carried(p: &Profile,carry: Carry) -> Result<Brep,String> {
    match carry {
        Carry::Line {delta} => {
            if dot(delta,unit(p.normal)).abs() <= 1e-12*norm(delta) { return Err("a slide along its profile's plane sweeps no volume".into()) }
            ruled(p,&p.moved(&Rigid {r:Rigid::identity().r,t:delta}),carry)
        }
        Carry::Screw {axis,angle,advance,..} if angle == 0. => carried(p,Carry::Line {delta:scale(unit(axis),advance)}),
        Carry::Screw {center,axis,angle,advance} if advance == 0. => revolve(p,center,axis,angle),
        Carry::Screw {center,axis,angle,advance} => screwed(p,center,axis,angle,advance),
    }
}

/// **The tube a ball sweeps along a path no closed form carries** (#684 D): `tube`, the solid a face
/// family of the ball's disc square to the path bounds (`family`: its one round wall, `loops`, and
/// the discs where it starts and ends), united with the half ball of radius `r` beyond each end
/// disc, as `canal`'s are: each ball's poles leaning off the path toward the wall's seam, which the
/// disc's first radius vector names. Every face is named `name`. The caller's to know the tube
/// meets itself nowhere.
pub fn capped(tube: Brep,loops: &[Vec<super::recipe::FamilyWall>],r: f64,name: &str) -> Result<Brep,String> {
    let [walls] = loops else { return Err("a ball's tube is one round wall".into()) };
    let [wall] = &walls[..] else { return Err("a ball's tube is one round wall".into()) };
    let [first,..,last] = &wall.nodes[..] else { return Err("a ball's tube has no stations".into()) };
    // the half ball beyond an end disc, `out` along the path's tangent there (the disc turns
    // counter-clockwise about it): its centre, its tangent and its first radius vector, where the
    // wall's seam runs
    let half = |n: &[f64],out: f64| -> Result<Brep,String> {
        if n.len() != 10 { return Err("a ball's tube's wall is no round".into()) }
        let v = |k: usize| [n[k],n[k+1],n[k+2]];
        let t = scale(unit(cross(v(3),v(6))),n[9].signum());
        half_ball(v(0),scale(t,out),poles(t,unit(v(3))),r)
    };
    ended(tube,half(first,-1.)?,half(last,1.)?,name)
}

/// Every face of `b` named `name`.
fn named(mut b: Brep,name: &str) -> Brep {
    for f in &mut b.faces { f.name = name.to_string(); }
    b
}

/// A ball's `tube` united with the half balls beyond its two ends, every face named `name`. Each
/// half ball's flat face is the tube's end disc there, so the three are sewn where those discs were,
/// the discs left out (`sew::sewn`): nothing is met in space — united by the Boolean, the tube's wall
/// was traced against each disc's plane, which a coil's later turns cross again and again.
fn ended(tube: Brep,start: Brep,end: Brep,name: &str) -> Result<Brep,String> {
    let tol = tolerance(tube.size()+start.size()+end.size());
    // (the tube's discs its caps, "start" and "end", as every sweep of a section names them; a half
    // ball's its one flat face)
    let walls = |mut b: Brep,disc: &dyn Fn(&Face) -> bool| { b.faces.retain(|f| !disc(f)); named(b.compacted(),name) };
    let cap = |f: &Face| f.name == "start" || f.name == "end";
    let flat = |f: &Face| matches!(f.surface,Surface::Plane(..));
    super::sew::sewn(&[walls(tube,&cap),walls(start,&flat),walls(end,&flat)],tol).map_err(|e| e.msg)
}

/// Where a ball's tube's seam runs, square to the path's unit tangent `t`: the unit `x0` square to
/// it turned about it by `SEAM`.
pub fn seam(t: V,x0: V) -> V {
    let (s,k) = SEAM.dsin_cos();
    add(scale(x0,k),scale(cross(t,x0),s))
}

/// The axis through a half ball's poles: leaning off the path's tangent `t` by `SEAM`, toward the
/// seam `x`.
fn poles(t: V,x: V) -> V {
    let (s,k) = SEAM.dsin_cos();
    add(scale(t,k),scale(x,s))
}

/// How far round from the radial line a ball's tube's seams run (radians): the golden angle's
/// fraction of a turn, which no symmetry of a design's makes.
pub const SEAM: f64 = 0.6180339887498949;

/// **The ball** of radius `r` about `c`: a half disc turned once about the diameter along `axis`,
/// its poles where `axis` meets the sphere and its seam the half circle through them on the side
/// `Frame::about(c, axis)`'s `x` points to. One face, bounded by its seam and a degenerate edge at
/// each pole. A plane through a pole is a meeting this kernel's Booleans do not split, so a caller
/// lays `axis` off the planes the ball will meet.
pub fn sphere(c: V,axis: V,r: f64) -> Result<Brep,String> {
    if !(r > 0. && r.is_finite()) { return Err(format!("a sphere's radius must be positive, not {r}")) }
    if !(norm(axis) > 0.) { return Err("a sphere's axis has no direction".into()) }
    ball(&Frame::about(c,axis),r)
}

/// The ball of radius `r` about `f`'s origin, its poles on `f.z` and its seam on `f.x`'s side.
fn ball(f: &Frame,r: f64) -> Result<Brep,String> {
    let c = f.o;
    let arc = ProfileEdge::Arc {frame:Frame::new(c,f.y,f.z),r,span:Some([0.,std::f64::consts::PI])};
    let shut = ProfileEdge::Line {a:f.at([0.,0.,-r]),b:f.at([0.,0.,r])};
    revolve(&Profile {origin:c,normal:f.y,loops:vec![vec![arc,shut]],names:Vec::new()},c,f.z,TAU)
}

/// **The ellipsoid** about `c` of semi-axes `semi`, along the axes of `Frame::new(c, axis, x)`: the
/// first along `x` (made square to `axis`), the second along `axis × x`, the third along `axis`.
/// The unit ball on that frame stretched by the three (`affine::Affine::stretch`, `Brep::mapped`):
/// one face, the sphere's image, its poles where `axis` meets it, each a degenerate edge, and its
/// seam the half ellipse through them on the side `x` points to (a sphere where the three are
/// equal). As there, a plane through a pole is a meeting the Booleans do not split.
pub fn ellipsoid(c: V,axis: V,x: V,semi: [f64;3]) -> Result<Brep,String> {
    if !semi.iter().all(|e| *e > 0. && e.is_finite()) {
        return Err(format!("an ellipsoid's semi-axes must be positive, not {semi:?}"))
    }
    if !(norm(axis) > 0.) { return Err("an ellipsoid's axis has no direction".into()) }
    if !(norm(cross(axis,x)) > 1e-12*norm(axis)*norm(x)) {
        return Err("an ellipsoid's first axis lies along its third".into())
    }
    let f = Frame::new(c,axis,x);
    Ok(ball(&f,1.)?.mapped(&super::affine::Affine::stretch(&f,semi)?))
}

/// **The cone** on the line through `o` along `axis`: radius `r0` at `o` and `r1` a height `h` along
/// `axis` from it, its meridian turned once (OCCT's `MakeCone`). Either radius may be nothing, an
/// apex there; equal radii make a cylinder. Its side's seam lies in the half plane
/// `Frame::about(o, axis)`'s `x` points to.
pub fn cone(o: V,axis: V,r0: f64,r1: f64,h: f64) -> Result<Brep,String> {
    if !(r0 >= 0. && r1 >= 0. && r0.is_finite() && r1.is_finite()) {
        return Err(format!("a cone's radii must be positive or nothing, not {r0} and {r1}"))
    }
    if !(r0.max(r1) > 0.) { return Err("a cone with no radius at either end has no volume".into()) }
    if !(h > 0. && h.is_finite()) { return Err(format!("a cone's height must be positive, not {h}")) }
    if !(norm(axis) > 0.) { return Err("a cone's axis has no direction".into()) }
    let f = Frame::about(o,axis);
    // the meridian in the plane of the axis and `x`: down the axis, out along the base, up the
    // side, back along the top; an end of no radius is its apex, on the axis
    let mut corners = vec![o];
    if r0 > 0. { corners.push(f.at([r0,0.,0.])); }
    if r1 > 0. { corners.push(f.at([r1,0.,h])); }
    corners.push(f.at([0.,0.,h]));
    let n = corners.len();
    let edges = (0..n).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%n]}).collect();
    revolve(&Profile {origin:o,normal:f.y,loops:vec![edges],names:Vec::new()},o,f.z,TAU)
}

/// **The torus** about the line through `c` along `axis`: its tube's middle circle of radius
/// `major` about `c` square to `axis`, its tube of radius `minor` (OCCT's `MakeTorus`). A ring
/// torus only: `minor` under `major`, so the tube clears the axis. One face, its two seams the
/// tube's circle on the side `Frame::about(c, axis)`'s `x` points to and the outer equator.
pub fn torus(c: V,axis: V,major: f64,minor: f64) -> Result<Brep,String> {
    if !(minor > 0. && minor.is_finite()) { return Err(format!("a torus's tube's radius must be positive, not {minor}")) }
    if !(major > minor && major.is_finite()) {
        return Err(format!("a torus's middle radius {major} must exceed its tube's {minor}: the tube would cross the axis"))
    }
    if !(norm(axis) > 0.) { return Err("a torus's axis has no direction".into()) }
    let f = Frame::about(c,axis);
    let centre = f.at([major,0.,0.]);
    let section = ProfileEdge::Arc {frame:Frame::new(centre,f.y,f.x),r:minor,span:None};
    revolve(&Profile {origin:centre,normal:f.y,loops:vec![vec![section]],names:Vec::new()},c,f.z,TAU)
}

/// The half of the ball of radius `r` about `c` on the side the unit `n` points to: the ball turned
/// about the unit `w` kept beyond the plane through `c` square to `n`. `w` leans off `n` and off
/// that plane, so neither of the sphere's poles is on the disc's circle or on the line through the
/// centre along the path — which lies in the path's plane, as a face of the design often does,
/// and a plane through a pole is a meeting this kernel's Booleans do not split.
fn half_ball(c: V,n: V,w: V,r: f64) -> Result<Brep,String> {
    kept(&sphere(c,w,r)?,c,n,0.)
}

/// The screw about the line through `c` along the unit `a`: a turn by `angle` and a step of `advance`
/// along it.
fn screw(c: V,a: V,angle: f64,advance: f64) -> Rigid {
    Rigid::turn(c,a,angle).then(&Rigid {r:Rigid::identity().r,t:scale(a,advance)})
}

/// The solid `start` sweeps to `end` as `carry` takes it, `end` the start's own image edge for
/// edge — its offset (a drafted prism's next section) or itself screwed (a canal's disc): every
/// point of a start edge joined to the point of the edge it became at the same fraction of its own
/// parameter (an arc's by angle), blended while carried (`Blend`). Two sections drawn apart are no
/// input: what a section becomes along a path is the face family's to say (#243 L3). Carried along
/// a line, edges that are coplanar lines join in a plane and coaxial circles and arcs in step in a
/// cone or a cylinder, and the rails between them are straight; everything else is a `Blend`, and
/// screwed about an axis the rails are its iso-curves.
fn ruled(start: &Profile,end: &Profile,carry: Carry) -> Result<Brep,String> { ruled_by(start,end,carry,None) }

/// What a drafted piece's corner edges are: for loop `l`'s corner where its edge `source` starts,
/// the curve from the start section's corner to the end's over parameters 0 to 1, or none for the
/// line between them.
type Rails<'a> = &'a dyn Fn(usize,usize) -> Result<Option<Curve>,String>;

/// `ruled`, its corners' rails given (a drafted prism's, `Moving::path`) where they are not lines:
/// then each side is a plane or a cone along the carry, joined to its neighbours along those
/// rails, its ends at whatever angles the end section's corners stand (refused otherwise).
fn ruled_by(start: &Profile,end: &Profile,carry: Carry,given: Option<Rails>) -> Result<Brep,String> {
    let n = unit(start.normal);
    // what carries the end section back to the start, and the direction a line carries along
    let (back,line) = match carry {
        Carry::Line {delta} => (Rigid {r:Rigid::identity().r,t:scale(delta,-1.)},Some(unit(delta))),
        Carry::Screw {center,axis,angle,advance} => (screw(center,axis,-angle,-advance),None),
    };
    let tol = tolerance(start.size()+end.size()+norm(sub(end.origin,start.origin)));
    let drafting = given.is_some();
    let frame = Frame::about(start.origin,n);
    let starts = walks(start,&|q| { let l = frame.local(q); [l[0],l[1]] })?;
    let ends = walks(end,&|q| { let l = frame.local(back.point(q)); [l[0],l[1]] })?;
    if starts.len() != ends.len() { return Err("a swept section's image has another number of holes".into()) }
    let mut b = Brep::default();
    let (mut start_loops,mut end_loops) = (Vec::new(),Vec::new());
    for (li,(sw,ew)) in starts.iter().zip(&ends).enumerate() {
        let m = sw.len();
        if ew.len() != m { return Err(format!("a swept section's loop {li} has {} edges and its image {}",m,ew.len())) }
        // the image's steps in the start walk's order, each edge by the edge it is the image of
        // (a whole circle's image keeps its frame, so it starts where the circle does)
        let ew: Vec<Seg> = sw.iter().map(|s| ew.iter().find(|x| x.source == s.source).cloned()
            .ok_or("a swept section's image has lost an edge")).collect::<Result<_,_>>()?;
        for k in 0..m {
            if distance(ew[k].end(),ew[(k+1)%m].start()) > 1e3*tol {
                return Err("a swept section's image does not run edge for edge with it".into())
            }
        }
        let pv: Vec<u32> = sw.iter().map(|s| b.vertex(s.start())).collect();
        let qv: Vec<u32> = ew.iter().map(|s| b.vertex(s.start())).collect();
        let curve_edge = |b: &mut Brep,s: &Seg,v: &[u32],k: usize| {
            let ends = [v[k],v[(k+1)%m]];
            b.edge(EdgeCurve::Curve(s.curve.clone()),s.span(),if s.forward() { ends } else { [ends[1],ends[0]] })
        };
        let se: Vec<u32> = (0..m).map(|k| curve_edge(&mut b,&sw[k],&pv,k)).collect();
        let ee: Vec<u32> = (0..m).map(|k| curve_edge(&mut b,&ew[k],&qv,k)).collect();
        let blend = |k: usize| Blend {a:sw[k].curve.clone(),ta:sw[k].t,b:ew[k].curve.moved(&back),tb:ew[k].t,carry,
            closed:m == 1 && matches!(sw[k].curve,Curve::Circle(..)) && matches!(ew[k].curve,Curve::Circle(..))};
        // the rails, each where an edge of the start section begins: straight along a line, the
        // blend's own iso-curve turned about an axis
        let rails: Vec<u32> = (0..m).map(|k| -> Result<u32,String> {
            let (p,q) = (sw[k].start(),ew[k].start());
            if let Some(curve) = given.map(|rail| rail(li,sw[k].source)).transpose()?.flatten() {
                if distance(curve.point(0.),p) > 1e3*tol || distance(curve.point(1.),q) > 1e3*tol {
                    return Err("a drafted corner's edge does not run between its sections' corners".into())
                }
                return Ok(b.edge(EdgeCurve::Curve(curve),[0.,1.],[pv[k],qv[k]]))
            }
            Ok(match line {
                Some(_) => b.edge(EdgeCurve::Curve(Curve::Line {p,d:unit(sub(q,p))}),[0.,distance(p,q)],[pv[k],qv[k]]),
                None => b.edge(EdgeCurve::Curve(Curve::Iso(Arc::new(blend(k)),0.)),[0.,1.],[pv[k],qv[k]]),
            })
        }).collect::<Result<_,_>>()?;
        for k in 0..m {
            let (a,z) = (&sw[k],&ew[k]);
            let k1 = (k+1)%m;
            // outward, in the section's plane: right of a walk counter-clockwise about n
            let mid = (a.t[0]+a.t[1])/2.;
            let outward = cross(a.along(mid),n);
            let analytic = line.and_then(|e| analytic_ruled(a,z,e,outward,tol,drafting));
            if drafting && analytic.is_none() { return Err("a drafted side is neither a plane nor a cone".into()) }
            let (surface,mut l) = match analytic {
                Some((surface,rim)) => {
                    let at = |p: V| surface.inverse(p);
                    // the face's parameters at each corner, the rails at the arc's start (u = 0) and end;
                    // the end section's arc read by its own angles in the same frame where its ends
                    // stand elsewhere (a drafted arc meeting a side at an angle)
                    let e = line.unwrap();
                    let way = dot(match &a.curve { Curve::Circle(f,_) => f.z,_ => e },e);
                    let turn = if rim { (a.t[1]-a.t[0])*way } else { 0. };
                    let corner = |p: V,u: f64| { let mut uv = at(p); if rim { uv[0] = u; } uv };
                    let (v0,v1) = if drafting { ((z.t[0]-a.t[0])*way,(z.t[1]-a.t[0])*way) } else { (0.,turn) };
                    let (u_p0,u_p1,u_q0,u_q1) = (corner(a.start(),0.),corner(a.end(),turn),corner(z.start(),v0),corner(z.end(),v1));
                    let pc = |x: Uv,y: Uv| Pcurve::Inverse {a:x,b:y};
                    let edge_uv = |s: &Seg,first: Uv,last: Uv| if s.forward() { pc(first,last) } else { pc(last,first) };
                    (surface,vec![
                        Coedge {edge:se[k],reversed:!a.forward(),pcurve:edge_uv(a,u_p0,u_p1)},
                        Coedge {edge:rails[k1],reversed:false,pcurve:pc(u_p1,u_q1)},
                        Coedge {edge:ee[k],reversed:z.forward(),pcurve:edge_uv(z,u_q0,u_q1)},
                        Coedge {edge:rails[k],reversed:true,pcurve:pc(u_p0,u_q0)},
                    ])
                }
                None => {
                    // S(u, v) with u along the walk: an edge runs u forward where it is walked forward
                    let along = |s: &Seg,v: f64| if s.forward() { Pcurve::Line {a:[0.,v],b:[1.,v]} } else { Pcurve::Line {a:[1.,v],b:[0.,v]} };
                    (Surface::Blend(frame,Arc::new(blend(k))),vec![
                        Coedge {edge:se[k],reversed:!a.forward(),pcurve:along(a,0.)},
                        Coedge {edge:rails[k1],reversed:false,pcurve:Pcurve::Line {a:[1.,0.],b:[1.,1.]}},
                        Coedge {edge:ee[k],reversed:z.forward(),pcurve:along(z,1.)},
                        Coedge {edge:rails[k],reversed:true,pcurve:Pcurve::Line {a:[0.,0.],b:[0.,1.]}},
                    ])
                }
            };
            let reversed = match &surface {
                Surface::Plane(_) => false,
                // u along the walk, so the walk's middle is u = ½
                Surface::Blend(..) => dot(surface.normal_raw([0.5,0.]),outward) < 0.,
                _ => dot(surface.normal_raw(surface.inverse(a.curve.point(mid))),outward) < 0.,
            };
            let face = Face {surface,reversed,loops:vec![],name:start.name(li,a.source)};
            let ccw = b.loop_area(&face,&l) > 0.;
            if ccw == reversed { l = reverse_loop(l); }
            b.faces.push(Face {loops:vec![l],..face});
        }
        start_loops.push((0..m).map(|k| use_of(se[k],&sw[k],true,on_plane())).collect::<Vec<_>>());
        end_loops.push((0..m).map(|k| use_of(ee[k],&ew[k],true,on_plane())).collect::<Vec<_>>());
    }
    // the caps: the start's material the way the carry moves it, the end's behind it (that way
    // read at one of the section's corners, off any axis)
    let corner = starts[0][0].start();
    let forward = {
        let step = sub(carry.at(corner,1e-6),corner);
        if dot(n,step) < 0. { scale(n,-1.) } else { n }
    };
    let cap = |origin: V,normal: V,outward: V,loops: Vec<Vec<Coedge>>,name: &str,b: &Brep| {
        let surface = Surface::Plane(Frame::about(origin,normal));
        let reversed = dot(normal,outward) < 0.;
        let face = Face {surface,reversed,loops:vec![],name:name.into()};
        let loops = loops.into_iter().enumerate().map(|(li,l)| {
            let ccw = b.loop_area(&face,&l) > 0.;
            // the outer loop turns with the face's sense, its holes against it
            if (li == 0) != (ccw != reversed) { reverse_loop(l) } else { l }
        }).collect();
        Face {loops,..face}
    };
    let start_cap = cap(start.origin,n,scale(forward,-1.),start_loops,"start",&b);
    let end_cap = cap(end.origin,unit(end.normal),carry.dir(forward,1.),end_loops,"end",&b);
    b.faces.push(start_cap);
    b.faces.push(end_cap);
    Ok(b)
}

/// The analytic face joining a start edge to its end edge along the line `e`, where there is one
/// (and whether its parameters run round an axis): a plane for coplanar lines, a cone or cylinder
/// for coaxial circles and arcs in step.
fn analytic_ruled(a: &Seg,z: &Seg,e: V,outward: V,tol: f64,free: bool) -> Option<(Surface,bool)> {
    match (&a.curve,&z.curve) {
        (Curve::Line {..},Curve::Line {..}) => {
            let (p0,p1,q0,q1) = (a.start(),a.end(),z.start(),z.end());
            let mut normal = cross(sub(p1,p0),sub(q0,p0));
            if norm(normal) <= tol*tol { normal = cross(sub(p1,p0),sub(q1,p0)); }
            let normal = crate::util::space::normalised(normal)?;
            if dot(sub(q1,p0),normal).abs() > 1e3*tol || dot(sub(q0,p0),normal).abs() > 1e3*tol { return None }
            let normal = if dot(normal,outward) < 0. { scale(normal,-1.) } else { normal };
            Some((Surface::Plane(Frame::new(p0,normal,sub(p1,p0))),false))
        }
        (Curve::Circle(fa,ra),Curve::Circle(fb,rb)) => {
            // coaxial along the line they are carried along, and joined at the same angles
            let h = dot(sub(fb.o,fa.o),e);
            let off = norm(sub(sub(fb.o,fa.o),scale(e,h)));
            let radial = |p: V,c: V| { let d = sub(p,c); unit(sub(d,scale(e,dot(d,e)))) };
            let aligned = |sa: V,sz: V| norm(cross(radial(sa,fa.o),radial(sz,fb.o))) <= 1e-9
                && dot(radial(sa,fa.o),radial(sz,fb.o)) > 0.;
            let mid = (a.t[0]+a.t[1])/2.;
            // (where the rails are given, a drafted arc's ends may move round it: free)
            if norm(cross(fa.z,e)) > 1e-9 || norm(cross(fb.z,e)) > 1e-9 || off > 1e3*tol || h <= 0.
                || !free && (!aligned(a.start(),z.start()) || !aligned(a.end(),z.end())
                    || !aligned(a.curve.point(mid),z.curve.point((z.t[0]+z.t[1])/2.))) { return None }
            let f = Frame::new(fa.o,e,radial(a.start(),fa.o));
            Some((if (ra-rb).abs() <= tol { Surface::Cylinder(f,*ra) } else { Surface::Cone(f,*ra,((rb-ra)/h).datan()) },true))
        }
        _ => None,
    }
}

/// A sheet of several faces (#212): B-spline surfaces one after another along `u`, each face's
/// last iso-`u` curve the next one's first (each a segment of one net, `Net::segment`, at a shared
/// `v` domain), each named. The faces share those edges and their ends — the first's curve standing
/// for both — so a split by it never meets their seams; each face's sense is its net's own.
pub fn sheets(nets: Vec<(crate::nurbs::Net,String)>) -> Result<Brep,String> {
    let Some((first,_)) = nets.first() else { return Err("a sheet of no faces".into()) };
    let [_,[v0,v1]] = first.domain();
    let row = |net: &crate::nurbs::Net,i: usize| Curve::BSpline(Arc::new(net.row(i)));
    let column = |net: &crate::nurbs::Net,j: usize| Curve::BSpline(Arc::new(net.column(j)));
    let mut b = Brep::default();
    let [u0,_] = first.domain()[0];
    // the first face's first iso-u edge and its ends
    let mut low = [b.vertex(first.point(u0,v0)),b.vertex(first.point(u0,v1))];
    let mut iso = b.edge(EdgeCurve::Curve(row(first,0)),[v0,v1],low);
    let line = |a: [f64;2],z: [f64;2]| Pcurve::Line {a,b:z};
    for (net,name) in nets {
        let [[u0,u1],[w0,w1]] = net.domain();
        if (w0-v0).abs() > 1e-12*(1.+v0.abs()) || (w1-v1).abs() > 1e-12*(1.+v1.abs()) {
            return Err("a sheet's faces must share their v domain".into())
        }
        let (last_u,last_v) = (net.poles.len()-1,net.poles[0].len()-1);
        let high = [b.vertex(net.point(u1,v0)),b.vertex(net.point(u1,v1))];
        let e0 = b.edge(EdgeCurve::Curve(column(&net,0)),[u0,u1],[low[0],high[0]]);
        let e1 = b.edge(EdgeCurve::Curve(row(&net,last_u)),[v0,v1],high);
        let e2 = b.edge(EdgeCurve::Curve(column(&net,last_v)),[u0,u1],[low[1],high[1]]);
        let lp = vec![
            Coedge {edge:e0,reversed:false,pcurve:line([u0,v0],[u1,v0])},
            Coedge {edge:e1,reversed:false,pcurve:line([u1,v0],[u1,v1])},
            Coedge {edge:e2,reversed:true,pcurve:line([u0,v1],[u1,v1])},
            Coedge {edge:iso,reversed:true,pcurve:line([u0,v0],[u0,v1])},
        ];
        b.faces.push(Face {surface:Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net)),reversed:false,loops:vec![lp],name});
        (low,iso) = (high,e1);
    }
    Ok(b)
}

/// A sheet of one net cut into a grid of faces (#684 N): at the parameters `us` along `u` and `vs`
/// along `v`, each from its domain's start to its end, the faces of a band between two of `us`
/// named by `names`. Each face is the net's segment there (`Net::segment`); neighbours share their
/// edges and corners, so a split by it never meets a seam. `closed`, each way: along `u`, the net's
/// last iso-`u` curve is its first, as a sheet along a characteristic that closes on itself is
/// (#684 W), and the last band closes on the first's edges, a tube; along `v`, its last iso-`v`
/// curve is its first, as a sheet revolved a whole turn is, and the last faces along `v` close on
/// the first's edges, a ring. An iso-`v` curve that collapses to a point — a row of poles all at
/// one place, a sphere's pole — is that point: one vertex, and a degenerate edge in each face it
/// bounds, as `sew::sewn` makes a pole.
pub fn grid_sheet(net: &crate::nurbs::Net,us: &[f64],vs: &[f64],names: &[String],closed: [bool;2]) -> Result<Brep,String> {
    let (nu,nv) = (us.len().saturating_sub(1),vs.len().saturating_sub(1));
    if nu == 0 || nv == 0 || names.len() != nu { return Err("a sheet's grid of faces must have a face each way, and a name a band".into()) }
    if (closed[0] && nu < 2) || (closed[1] && nv < 2) { return Err("a closed sheet must have two faces round".into()) }
    // each band cut along u once, then round along v
    let nets: Vec<Vec<crate::nurbs::Net>> = (0..nu).map(|i| {
        let band = net.segment([us[i],us[i+1]],[vs[0],vs[nv]]);
        (0..nv).map(|j| band.segment([us[i],us[i+1]],[vs[j],vs[j+1]])).collect()
    }).collect();
    let curve = |b: BSpline| EdgeCurve::Curve(Curve::BSpline(Arc::new(b)));
    // the corners, and the edges along u at each of the `mv` distinct v and along v at each of the
    // `mu` distinct u (a closed way's last its first)
    let (mu,mv) = (if closed[0] { nu } else { nu+1 },if closed[1] { nv } else { nv+1 });
    let mut b = Brep::default();
    // each iso-`v` curve's poles over the whole of `u`, and whether they are all one point
    let iso_u = |i: usize,j: usize| if j < nv { nets[i][j].column(0) } else { let n = &nets[i][nv-1]; n.column(n.poles[0].len()-1) };
    // (the tolerance of the net's size, read as the farthest a pole stands from its first, twice)
    let tol = tolerance(2.*net.poles.iter().flatten().map(|&p| distance(p,net.poles[0][0])).fold(0.,f64::max));
    let pole: Vec<bool> = (0..mv).map(|j| {
        let at = net.point(us[0],vs[j]);
        (0..nu).all(|i| iso_u(i,j).poles.iter().all(|&q| distance(q,at) <= tol))
    }).collect();
    let corners: Vec<Vec<u32>> = {
        let mut corners = vec![vec![0;mv];mu];
        for j in 0..mv { for i in 0..mu {
            corners[i][j] = if pole[j] && i > 0 { corners[0][j] } else { b.vertex(net.point(us[i],vs[j])) };
        } }
        corners
    };
    let along_u: Vec<Vec<u32>> = (0..nu).map(|i| (0..mv).map(|j| {
        let ends = [corners[i][j],corners[(i+1)%mu][j]];
        if pole[j] { b.edge(EdgeCurve::Degenerate,[us[i],us[i+1]],ends) } else { b.edge(curve(iso_u(i,j)),[us[i],us[i+1]],ends) }
    }).collect()).collect();
    let along_v: Vec<Vec<u32>> = (0..mu).map(|i| (0..nv).map(|j| {
        let iso = if i < nu { nets[i][j].row(0) } else { let n = &nets[nu-1][j]; n.row(n.poles.len()-1) };
        b.edge(curve(iso),[vs[j],vs[j+1]],[corners[i][j],corners[i][(j+1)%mv]])
    }).collect()).collect();
    let line = |a: [f64;2],z: [f64;2]| Pcurve::Line {a,b:z};
    for (i,band) in nets.into_iter().enumerate() { for (j,net) in band.into_iter().enumerate() {
        let ([u0,u1],[v0,v1]) = ([us[i],us[i+1]],[vs[j],vs[j+1]]);
        let lp = vec![
            Coedge {edge:along_u[i][j],reversed:false,pcurve:line([u0,v0],[u1,v0])},
            Coedge {edge:along_v[(i+1)%mu][j],reversed:false,pcurve:line([u1,v0],[u1,v1])},
            Coedge {edge:along_u[i][(j+1)%mv],reversed:true,pcurve:line([u0,v1],[u1,v1])},
            Coedge {edge:along_v[i][j],reversed:true,pcurve:line([u0,v0],[u0,v1])},
        ];
        b.faces.push(Face {surface:Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net)),reversed:false,loops:vec![lp],
            name:names[i].clone()});
    } }
    Ok(b)
}

/// A solid lofted along a plane's normal (#607): each loop of its section a ring of B-spline
/// surfaces one after another along `u` — each net's last iso-`u` curve the next one's first, as
/// `sheets` joins them, and the last's the first's — with `v` running along `n` from the plane at
/// ordinate `h[0]` about `origin` to the one at `h[1]`, every net's first and last rows of poles in
/// those planes; capped by those planes. The first loop is the outer one, walked with the material
/// on its left (counter-clockwise about `n`), the rest its holes (clockwise).
pub fn lofted(origin: V,n: V,h: [f64;2],loops: Vec<Vec<crate::nurbs::Net>>) -> Result<Brep,String> {
    let n = unit(n);
    let plane = Frame::about(origin,n);
    let mut b = Brep::default();
    let row = |net: &crate::nurbs::Net,i: usize| -> Curve { Curve::BSpline(Arc::new(BSpline {degree:net.dv,knots:net.vknots.clone(),
        poles:net.poles[i].clone(),weights:None})) };
    // (a rational net's column carries its weights: a vault's arc stays the arc its face is)
    let column = |net: &crate::nurbs::Net,j: usize| -> Curve { Curve::BSpline(Arc::new(BSpline {degree:net.du,knots:net.uknots.clone(),
        poles:net.poles.iter().map(|r| r[j]).collect(),weights:net.weights.as_ref().map(|w| w.iter().map(|r| r[j]).collect())})) };
    let line = |a: [f64;2],z: [f64;2]| Pcurve::Line {a,b:z};
    let (mut bottom_loops,mut top_loops) = (Vec::new(),Vec::new());
    for nets in &loops {
        let m = nets.len();
        if m == 0 { return Err("a lofted loop of no faces".into()) }
        for net in nets { net.check()?; }
        let [_,[v0,v1]] = nets[0].domain();
        let bv: Vec<u32> = nets.iter().map(|net| { let [[u0,_],_] = net.domain(); b.vertex(net.point(u0,v0)) }).collect();
        let tv: Vec<u32> = nets.iter().map(|net| { let [[u0,_],_] = net.domain(); b.vertex(net.point(u0,v1)) }).collect();
        // the rails, each net's first iso-u curve; the last's last is the first's first
        let rails: Vec<u32> = (0..m).map(|k| b.edge(EdgeCurve::Curve(row(&nets[k],0)),[v0,v1],[bv[k],tv[k]])).collect();
        let mut bottom = Vec::new();
        let mut top = Vec::new();
        for k in 0..m {
            let net = &nets[k];
            let [[u0,u1],[w0,w1]] = net.domain();
            if (w0-v0).abs() > 1e-12*(1.+v0.abs()) || (w1-v1).abs() > 1e-12*(1.+v1.abs()) {
                return Err("a lofted loop's faces must share their v domain".into())
            }
            let k1 = (k+1)%m;
            let last = net.poles[0].len()-1;
            let e0 = b.edge(EdgeCurve::Curve(column(net,0)),[u0,u1],[bv[k],bv[k1]]);
            let e2 = b.edge(EdgeCurve::Curve(column(net,last)),[u0,u1],[tv[k],tv[k1]]);
            bottom.push(e0); top.push(e2);
            let lp = vec![
                Coedge {edge:e0,reversed:false,pcurve:line([u0,v0],[u1,v0])},
                Coedge {edge:rails[k1],reversed:false,pcurve:line([u1,v0],[u1,v1])},
                Coedge {edge:e2,reversed:true,pcurve:line([u0,v1],[u1,v1])},
                Coedge {edge:rails[k],reversed:true,pcurve:line([u0,v0],[u0,v1])},
            ];
            b.faces.push(Face {surface:Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net.clone())),reversed:false,
                loops:vec![lp],name:String::new()});
        }
        bottom_loops.push((0..m).rev().map(|k| Coedge {edge:bottom[k],reversed:true,pcurve:on_plane()}).collect::<Vec<_>>());
        top_loops.push((0..m).map(|k| Coedge {edge:top[k],reversed:false,pcurve:on_plane()}).collect::<Vec<_>>());
    }
    let cap = |at: f64| Surface::Plane(Frame {o:add(plane.o,scale(n,at)),..plane});
    b.faces.push(Face {surface:cap(h[0]),reversed:true,loops:bottom_loops,name:"far".into()});
    b.faces.push(Face {surface:cap(h[1]),reversed:false,loops:top_loops,name:"near".into()});
    Ok(b)
}

/// A sheet: the whole of a B-spline surface as one face, bounded by its four boundary curves
/// (each its net's outermost row or column of poles, exactly), its sense the net's own.
pub fn sheet(net: crate::nurbs::Net) -> Result<Brep,String> {
    net.check()?;
    let [[u0,u1],[v0,v1]] = net.domain();
    // a rational net's boundary rows and columns weighed as the net weighs them
    let row = |i: usize| Curve::BSpline(Arc::new(net.row(i)));
    let column = |j: usize| Curve::BSpline(Arc::new(net.column(j)));
    let (last_u,last_v) = (net.poles.len()-1,net.poles[0].len()-1);
    let mut b = Brep::default();
    let corner = [net.point(u0,v0),net.point(u1,v0),net.point(u1,v1),net.point(u0,v1)];
    let vs: Vec<u32> = corner.iter().map(|&p| b.vertex(p)).collect();
    // the loop u0v0 → u1v0 → u1v1 → u0v1 → back, counter-clockwise in (u, v)
    let e0 = b.edge(EdgeCurve::Curve(column(0)),[u0,u1],[vs[0],vs[1]]);
    let e1 = b.edge(EdgeCurve::Curve(row(last_u)),[v0,v1],[vs[1],vs[2]]);
    let e2 = b.edge(EdgeCurve::Curve(column(last_v)),[u0,u1],[vs[3],vs[2]]);
    let e3 = b.edge(EdgeCurve::Curve(row(0)),[v0,v1],[vs[0],vs[3]]);
    let line = |a: [f64;2],z: [f64;2]| Pcurve::Line {a,b:z};
    let lp = vec![
        Coedge {edge:e0,reversed:false,pcurve:line([u0,v0],[u1,v0])},
        Coedge {edge:e1,reversed:false,pcurve:line([u1,v0],[u1,v1])},
        Coedge {edge:e2,reversed:true,pcurve:line([u0,v1],[u1,v1])},
        Coedge {edge:e3,reversed:true,pcurve:line([u0,v0],[u0,v1])},
    ];
    b.faces.push(Face {surface:Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net)),reversed:false,loops:vec![lp],name:String::new()});
    Ok(b)
}

/// A face: the profile's region of its plane, alone, its material normal the profile's normal —
/// a sheet of one planar face, bounded by the profile's edges exactly, to be sewn to others
/// (`sew::sewn`) as a disc closes a cone's side.
pub fn face(p: &Profile) -> Result<Brep,String> {
    let n = unit(p.normal);
    if !n.iter().all(|x| x.is_finite()) { return Err("a face's profile has no normal".into()) }
    let plane = Frame::about(p.origin,n);
    let walks = walks(p,&|q| { let l = plane.local(q); [l[0],l[1]] })?;
    let mut b = Brep::default();
    let mut loops = Vec::new();
    for walk in &walks {
        let m = walk.len();
        let vs: Vec<u32> = walk.iter().map(|s| b.vertex(s.start())).collect();
        loops.push((0..m).map(|k| {
            let s = &walk[k];
            let ends = [vs[k],vs[(k+1)%m]];
            let e = b.edge(EdgeCurve::Curve(s.curve.clone()),s.span(),if s.forward() { ends } else { [ends[1],ends[0]] });
            use_of(e,s,true,on_plane())
        }).collect::<Vec<_>>());
    }
    b.faces.push(Face {surface:Surface::Plane(plane),reversed:false,loops,name:String::new()});
    Ok(b)
}

/// The box from `lo` to `hi`, its faces square to the world's axes.
pub fn block(lo: V,hi: V) -> Result<Brep,String> {
    let at = |x: f64,y: f64| [x,y,lo[2]];
    let corners = [at(lo[0],lo[1]),at(hi[0],lo[1]),at(hi[0],hi[1]),at(lo[0],hi[1])];
    let edges = (0..4).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%4]}).collect();
    prism(&Profile {origin:lo,normal:[0.,0.,1.],loops:vec![edges],names:Vec::new()},0.,hi[2]-lo[2])
}

/// The square on the plane through `o` square to the unit `n`, `half` from `o` along each of its
/// frame's axes (`Frame::about`): a block's section.
pub fn square(o: V,n: V,half: f64) -> Profile {
    let f = Frame::about(o,n);
    let at = |x: f64,y: f64| f.at([x,y,0.]);
    let corners = [at(-half,-half),at(half,-half),at(half,half),at(-half,half)];
    let edges = (0..4).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%4]}).collect();
    Profile {origin:o,normal:n,loops:vec![edges],names:Vec::new()}
}

/// The cylinder of radius `r` about the line through `o` along the unit `n`, from `o` to `depth`
/// along `n`, open at both ends: a sheet of `pieces` faces round it, each a stretch of the one
/// cylinder between two lines along it, its edges at the ends arcs (a prism of the disc's arcs, its
/// caps left off). A tool slid along its own axis sweeps one (#684 X).
pub fn tube(o: V,n: V,r: f64,depth: f64,pieces: usize) -> Result<Brep,String> {
    if pieces < 2 { return Err("a tube must have two faces round".into()) }
    let frame = Frame::about(o,unit(n));
    let step = std::f64::consts::TAU/pieces as f64;
    let arcs = (0..pieces).map(|k| ProfileEdge::Arc {frame,r,span:Some([k as f64*step,(k+1) as f64*step])}).collect();
    let mut b = prism(&Profile {origin:o,normal:unit(n),loops:vec![arcs],names:Vec::new()},0.,depth)?;
    // the prism's two caps are its last faces
    b.faces.truncate(pieces);
    Ok(b)
}

/// The disc of radius `r` about `o` square to `n`: a cylinder's or a frustum's section.
pub fn disc(o: V,n: V,r: f64) -> Profile {
    Profile {origin:o,normal:n,loops:vec![vec![ProfileEdge::Arc {frame:Frame::about(o,n),r,span:None}]],names:Vec::new()}
}

/// The half disc of the ball of radius `r` about `f.o` in `f`'s plane, on the side of `f.y` its `x`
/// points to, shut along `f.y`: turned once about the line through `f.o` along `f.y`, the ball.
pub fn ball_section(f: &Frame,r: f64) -> Profile {
    let arc = ProfileEdge::Arc {frame:Frame::new(f.o,f.z,f.y),r,span:Some([0.,std::f64::consts::PI])};
    let shut = ProfileEdge::Line {a:f.at([0.,-r,0.]),b:f.at([0.,r,0.])};
    Profile {origin:f.o,normal:f.z,loops:vec![vec![arc,shut]],names:Vec::new()}
}

/// `piece` kept on the side `keep` (a unit normal) of the plane through `o`: in common with a block
/// standing on the plane under the piece's middle, reaching past the piece on every other side.
pub fn kept(piece: &Brep,o: V,keep: V,floor: f64) -> Result<Brep,String> {
    let (lo,hi) = piece.bounds();
    if !lo[0].is_finite() { return Err("a piece of nothing cut off at a plane".into()) }
    let middle = scale(add(lo,hi),0.5);
    let foot = sub(middle,scale(keep,dot(sub(middle,o),keep)));
    let size = 2.*distance(lo,hi)+distance(middle,foot);
    let block = prism(&square(foot,keep,size),0.,size)?;
    Ok(super::boolean::combined(piece,&block,super::boolean::Op::Common,floor)?)
}

/// `piece` less what lies behind the plane through `o` (against `keep`, a unit normal) within
/// `reach` of `o`: a block standing on the plane, `reach` deep and as wide either way — a rolled
/// run cut short at the section of the corner at its end, the cell taking what lies between there
/// and the vertex, and nothing of the run elsewhere (a rim round a hole comes back past the plane).
pub fn cut_before(piece: &Brep,o: V,keep: V,reach: f64,floor: f64) -> Result<Brep,String> {
    let block = prism(&square(o,keep,reach),-reach,0.)?;
    Ok(super::boolean::combined(piece,&block,super::boolean::Op::Cut,floor)?)
}

/// `piece` kept within (`inside`) or outside the solid a cylinder, a cone, a sphere or an affine
/// image of one bounds: in common with it, or less it, the cylinder or the cone long enough to reach
/// past the piece either way (a cone short of its apex).
pub fn kept_by_solid(piece: &Brep,surface: &Surface,inside: bool,floor: f64) -> Result<Brep,String> {
    let op = if inside { super::boolean::Op::Common } else { super::boolean::Op::Cut };
    Ok(super::boolean::combined(piece,&bounded_by(piece.bounds(),surface)?,op,floor)?)
}

/// The solid `surface` bounds, reaching past the box `(lo, hi)` either way where it is unbounded:
/// an affine image's the image of its preimage's, reaching past the box carried back.
fn bounded_by((lo,hi): super::geom::Box3,surface: &Surface) -> Result<Brep,String> {
    if !lo[0].is_finite() { return Err("a piece of nothing cut off at a curved face".into()) }
    if let Surface::Affine(m) = surface { return Ok(bounded_by(super::geom::boxed(&m.back,(lo,hi)),&m.of)?.mapped(&m.map)) }
    let middle = scale(add(lo,hi),0.5);
    Ok(match *surface {
        Surface::Cylinder(f,radius) => {
            // the circle in the plane square to the axis through the piece's middle's foot
            let foot = add(f.o,scale(f.z,dot(sub(middle,f.o),f.z)));
            let reach = distance(lo,hi)+radius;
            prism(&disc(foot,f.z,radius),-reach,reach)?
        }
        Surface::Sphere(f,radius) => {
            // a half disc turned once about a line through the centre
            revolve(&ball_section(&f,radius),f.o,f.y,TAU)?
        }
        Surface::Cone(f,radius,half) => {
            // a frustum of the cone's solid over the heights along its axis the piece reaches, and
            // as far again either way, short of the apex: its section in a meridian turned once
            let slope = half.dtan();
            let corners = [lo,hi].into_iter().flat_map(|a| [lo,hi].into_iter().flat_map(move |b| [lo,hi].into_iter()
                .map(move |c| [a[0],b[1],c[2]])));
            let heights: Vec<f64> = corners.map(|p| dot(sub(p,f.o),f.z)).collect();
            let (h0,h1) = heights.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),&h| (a.min(h),b.max(h)));
            let span = h1-h0;
            let (mut from,mut to) = (h0-span,h1+span);
            let apex = if slope.abs() > 1e-12 { -radius/slope } else { f64::NAN };
            if slope > 1e-12 { from = from.max(apex+1e-6*span.max(1.)) }
            if slope < -1e-12 { to = to.min(apex-1e-6*span.max(1.)) }
            if !(to > from) { return Err("a piece kept within a cone it does not reach".into()) }
            let at = |rho: f64,h: f64| add(f.o,add(scale(f.x,rho),scale(f.z,h)));
            let (r0,r1) = (radius+slope*from,radius+slope*to);
            let pts = [at(0.,from),at(r0,from),at(r1,to),at(0.,to)];
            let edges = (0..4).map(|k| ProfileEdge::Line {a:pts[k],b:pts[(k+1)%4]}).collect();
            let section = Profile {origin:f.o,normal:f.y,loops:vec![edges],names:Vec::new()};
            revolve(&section,f.o,f.z,TAU)?
        }
        _ => return Err(format!("a piece kept within a {}",surface.kind())),
    })
}
