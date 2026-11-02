//! **Where two surfaces meet, as a topological answer** (#735 A2): not curves alone but what the
//! meeting is — its branches, each saying whether the surfaces cross along it or only touch; its
//! singular points, where its own topology changes (a point where the surfaces touch and meet in
//! nothing else near, or where branches cross); and whether the two are one surface over the
//! region. An arrangement computes each pair of surfaces' meeting once and shares it with every face
//! of those surfaces (`Shared`).
//!
//! **Complete, not just accurate.** Over two boxes of parameters `[da, db]` a meeting's branches
//! either reach a box's boundary or close inside it. Those reaching the boundary are seeded where
//! the boundary's four sides cross the other surface (each side a curve on its surface, its sign
//! changes against the other's signed distance found by bisection between samples finer than the
//! side's feature). A closed branch bounds a disc of the parameters on which the other surface's
//! signed distance `f = b ∘ a` has an interior extremum: so every critical point of `f` (a Morse
//! analysis of the meeting in `a`'s parameters, found by Newton from a grid) is a seed for the
//! branches round it, each reached by walking down `f`'s gradient to its zero. A critical point at
//! which `f` itself vanishes is where the two surfaces touch: a singular point of the meeting,
//! classified to second order (`ssi::touch`) as a touch alone or a crossing of branches. And between
//! analytic surfaces nothing seeded so is left unproved: `ssi::unseen` halves the box, each cell
//! shown clear of `b` by the signed distance's Lipschitz and second-order bounds or beside a branch
//! already traced, and any it cannot clear is traced from in turn, until none is left. Closed forms
//! (`ssi::intersect`: the quadric pairs, coaxial surfaces of revolution, a screwed line's sheets)
//! are taken where they exist; the search still runs over them for the critical points.
use super::geom::{Curve,Surface,Uv,V};
use super::ssi::{intersect,Ssi};
use crate::util::space::{cross,distance,norm,scale,sub};

/// How two surfaces stand along a branch of their meeting.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Contact {
    /// They cross along it, each passing from one side of the other to its other side.
    Crossing,
    /// They touch along it, their normals parallel all along: tangent, not crossing (a plane on a
    /// cylinder's side, two surfaces of revolution along a circle).
    Touching,
}

/// A branch of a meeting: its curve (whole, as a closed form gives it, or traced) and how the two
/// surfaces stand along it, read over the boxes asked about.
#[derive(Clone,Debug)]
pub struct Branch { pub curve: Curve,pub contact: Contact }

/// What a singular point of a meeting is.
#[derive(Clone,Debug,PartialEq)]
pub enum Singular {
    /// The two touch at the point and meet in nothing else near it (`f` has an extremum of zero there).
    Touch,
    /// Branches cross at it: a saddle of `f` at zero, or two branches of a closed form crossing,
    /// each going out along the directions given.
    Crossing(Vec<V>),
}

/// A meeting's singular point and what it is.
#[derive(Clone,Debug,PartialEq)]
pub struct Point { pub at: V,pub kind: Singular }

/// **What two surfaces share over two boxes of their parameters** (the module's account).
#[derive(Clone,Debug)]
pub struct Meeting {
    /// The two are one surface (within the tolerance), coinciding over the whole region.
    pub same: bool,
    /// Its branches.
    pub branches: Vec<Branch>,
    /// Its singular points: tangencies alone, and where branches cross.
    pub singular: Vec<Point>,
    /// Whether the branches came in closed form (else traced).
    pub closed_form: bool,
}

/// Samples along each side of a box of parameters, and the grid a box's critical points are
/// sought from: well past any feature of the surfaces this kernel builds over one face.
const SIDE: usize = 128;
const GRID: usize = 12;

/// The signed distance from `b` of `a`'s point at `uv`.
fn f(a: &Surface,b: &Surface,uv: Uv) -> f64 { b.implicit(a.point(uv)) }

/// The box of a box of parameters' points in space, sampled and grown by a sixteenth of its
/// diagonal (what a trace and a search keep to).
fn reach(s: &Surface,d: [[f64;2];2]) -> (V,V) {
    let (lo,hi) = super::geom::hull((0..=16).flat_map(|i| (0..=16).map(move |j|
        s.point([d[0][0]+(d[0][1]-d[0][0])*i as f64/16.,d[1][0]+(d[1][1]-d[1][0])*j as f64/16.]))));
    let grow = norm(sub(hi,lo))/16.;
    (lo.map(|x| x-grow),hi.map(|x| x+grow))
}

/// The two boxes' shared box in space.
fn shared(a: (V,V),b: (V,V)) -> (V,V) {
    (std::array::from_fn(|k| a.0[k].max(b.0[k])),std::array::from_fn(|k| a.1[k].min(b.1[k])))
}

/// **Where a box's boundary crosses the other surface**: along each of its four sides, the sign
/// changes of `b`'s signed distance between samples, each bracketed root found (`roots::bracketed_root`).
fn boundary_seeds(a: &Surface,d: [[f64;2];2],b: &Surface) -> Vec<V> {
    let mut out = Vec::new();
    let sides: [(Uv,Uv);4] = [([d[0][0],d[1][0]],[d[0][1],d[1][0]]),([d[0][1],d[1][0]],[d[0][1],d[1][1]]),
        ([d[0][1],d[1][1]],[d[0][0],d[1][1]]),([d[0][0],d[1][1]],[d[0][0],d[1][0]])];
    for (p,q) in sides {
        let at = |s: f64| [p[0]+(q[0]-p[0])*s,p[1]+(q[1]-p[1])*s];
        let mut last = (0.,f(a,b,at(0.)));
        for i in 1..=SIDE {
            let s = i as f64/SIDE as f64;
            let v = f(a,b,at(s));
            if v == 0. { out.push(a.point(at(s))); }
            else if (v < 0.) != (last.1 < 0.) && last.1 != 0. {
                let root = crate::util::roots::bracketed_root(|x| Some(f(a,b,at(x))),last.0,last.1,s,v,1e-12);
                out.push(a.point(at(root)));
            }
            last = (s,v);
        }
    }
    out
}

/// `f`'s gradient and Hessian in `a`'s parameters at `uv`, by central differences of step `h`.
fn jet(a: &Surface,b: &Surface,uv: Uv,h: [f64;2]) -> (f64,[f64;2],[[f64;2];2]) {
    let g = |du: f64,dv: f64| f(a,b,[uv[0]+du,uv[1]+dv]);
    let (f0,up,um,vp,vm) = (g(0.,0.),g(h[0],0.),g(-h[0],0.),g(0.,h[1]),g(0.,-h[1]));
    let grad = [(up-um)/(2.*h[0]),(vp-vm)/(2.*h[1])];
    let huu = (up-2.*f0+um)/(h[0]*h[0]);
    let hvv = (vp-2.*f0+vm)/(h[1]*h[1]);
    let huv = (g(h[0],h[1])-g(h[0],-h[1])-g(-h[0],h[1])+g(-h[0],-h[1]))/(4.*h[0]*h[1]);
    (f0,grad,[[huu,huv],[huv,hvv]])
}

/// **The critical points of `f` over the box** (the meeting's Morse analysis in `a`'s parameters):
/// Newton on its gradient from each point of a grid, kept where it settles inside the box, each
/// once. With `f`'s value there: zero at a singular point of the meeting.
pub fn critical_points(a: &Surface,d: [[f64;2];2],b: &Surface,tol: f64) -> Vec<(Uv,f64)> {
    let w = [d[0][1]-d[0][0],d[1][1]-d[1][0]];
    let h = [w[0]*1e-5,w[1]*1e-5];
    let mut found: Vec<(Uv,f64)> = Vec::new();
    for i in 0..GRID { for j in 0..GRID {
        let mut uv = [d[0][0]+w[0]*(i as f64+0.5)/GRID as f64,d[1][0]+w[1]*(j as f64+0.5)/GRID as f64];
        let mut settled = false;
        for _ in 0..40 {
            let (_,g,m) = jet(a,b,uv,h);
            let det = m[0][0]*m[1][1]-m[0][1]*m[1][0];
            if !(det.abs() > 1e-300) { break }
            let step = [(m[1][1]*g[0]-m[0][1]*g[1])/det,(m[0][0]*g[1]-m[1][0]*g[0])/det];
            uv = [uv[0]-step[0],uv[1]-step[1]];
            if !(uv[0] >= d[0][0] && uv[0] <= d[0][1] && uv[1] >= d[1][0] && uv[1] <= d[1][1]) { break }
            if step[0].abs() <= 1e-9*w[0] && step[1].abs() <= 1e-9*w[1] { settled = true; break }
        }
        if !settled { continue }
        let p = a.point(uv);
        if found.iter().any(|(q,_)| distance(a.point(*q),p) <= 1e3*tol.max(1e-9*norm(sub(a.point([d[0][1],d[1][1]]),a.point([d[0][0],d[1][0]]))))) { continue }
        found.push((uv,f(a,b,uv)));
    } }
    found
}

/// From a critical point of `f` that is no zero of it, the points of the meeting reached walking
/// down (or up) `|f|`'s gradient toward zero in eight ways out: a branch closing round it is met so.
fn descents(a: &Surface,d: [[f64;2];2],b: &Surface,uv: Uv,tol: f64) -> Vec<V> {
    let w = [d[0][1]-d[0][0],d[1][1]-d[1][0]];
    let h = [w[0]*1e-5,w[1]*1e-5];
    let mut out = Vec::new();
    for k in 0..8 {
        let ang = std::f64::consts::TAU*k as f64/8.;
        let mut x = [uv[0]+0.01*w[0]*ang.cos(),uv[1]+0.01*w[1]*ang.sin()];
        let sign = f(a,b,uv).signum();
        let mut last = f(a,b,x);
        for _ in 0..400 {
            if !(x[0] >= d[0][0] && x[0] <= d[0][1] && x[1] >= d[1][0] && x[1] <= d[1][1]) { break }
            let (fx,g,_) = jet(a,b,x,h);
            if fx.abs() <= tol || fx.signum() != sign {
                // bracketed between the last step and this one: bisect along the gradient line
                out.push(a.point(x));
                break
            }
            let gn = (g[0]*g[0]/(w[0]*w[0])+g[1]*g[1]/(w[1]*w[1])).sqrt().max(1e-300);
            // a step a fiftieth of the box across, scaled to the gradient, down |f|
            let s = 0.02/gn;
            x = [x[0]-sign*s*g[0]/(w[0]*w[0])*w[0]*w[0],x[1]-sign*s*g[1]/(w[1]*w[1])*w[1]*w[1]];
            let fx = f(a,b,x);
            if fx.abs() >= last.abs() && fx.signum() == sign { break }
            last = fx;
        }
    }
    out
}

/// How the two stand along a curve over the shared box: touching where `|∇a × ∇b|` stays under a
/// thousandth all along its points within it, else crossing.
fn contact(a: &Surface,b: &Surface,c: &Curve,(lo,hi): (V,V)) -> Contact {
    let span = match c.period() { Some(p) => [0.,p],None => match c {
        Curve::Line {p,d} => {
            // the stretch within the box: a line's
            let (mut t0,mut t1) = (f64::NEG_INFINITY,f64::INFINITY);
            for k in 0..3 {
                if d[k].abs() < 1e-300 { continue }
                let (x,y) = ((lo[k]-p[k])/d[k],(hi[k]-p[k])/d[k]);
                t0 = t0.max(x.min(y)); t1 = t1.min(x.max(y));
            }
            if !(t1 > t0) { return Contact::Crossing }
            [t0,t1]
        }
        Curve::BSpline(s) => s.domain(),
        Curve::Traced(t) => [0.,(t.pts.len()-1) as f64],
        _ => [0.,1.],
    } };
    let mut any = false;
    for j in 0..=64 {
        let p = c.point(span[0]+(span[1]-span[0])*j as f64/64.);
        if (0..3).any(|k| p[k] < lo[k] || p[k] > hi[k]) { continue }
        any = true;
        if norm(cross(unit_or(a.gradient(p)),unit_or(b.gradient(p)))) > 1e-3 { return Contact::Crossing }
    }
    if any { Contact::Touching } else { Contact::Crossing }
}

/// **The meeting of `a` over the box `da` of its parameters and `b` over `db`**, to `tol` (the
/// module's account).
pub fn meet(a: &Surface,da: [[f64;2];2],b: &Surface,db: [[f64;2];2],tol: f64) -> Result<Meeting,String> {
    if super::ssi::same(a,b,tol) { return Ok(Meeting {same:true,branches:Vec::new(),singular:Vec::new(),closed_form:true}) }
    let (ra,rb) = (reach(a,da),reach(b,db));
    let (lo,hi) = shared(ra,rb);
    let mut singular: Vec<Point> = Vec::new();
    // the Morse analysis in each surface's parameters: the critical points of the other's distance
    let mut critical: Vec<(V,bool)> = Vec::new();
    for (s,d,o) in [(a,da,b),(b,db,a)] {
        for (uv,value) in critical_points(s,d,o,tol) {
            let p = s.point(uv);
            if (0..3).any(|k| p[k] < lo[k] || p[k] > hi[k]) { continue }
            critical.push((p,value.abs() <= 8.*tol));
        }
    }
    for &(p,zero) in &critical {
        if !zero || singular.iter().any(|q| distance(q.at,p) <= 1e3*tol) { continue }
        match super::ssi::touch(a,b,p) {
            Some(t) if t.branches.is_empty() => singular.push(Point {at:p,kind:Singular::Touch}),
            Some(t) => singular.push(Point {at:p,kind:Singular::Crossing(t.branches)}),
            // degenerate to second order: the two touch along a curve through it, said by its branch
            None => {}
        }
    }
    let (curves,closed_form) = match intersect(a,b,tol) {
        Ssi::Same => return Ok(Meeting {same:true,branches:Vec::new(),singular:Vec::new(),closed_form:true}),
        Ssi::Curves(cs) => (cs,true),
        Ssi::Traced => {
            let mut seeds = boundary_seeds(a,da,b);
            seeds.extend(boundary_seeds(b,db,a));
            for (s,d,o) in [(a,da,b),(b,db,a)] {
                for (uv,value) in critical_points(s,d,o,tol) {
                    if value.abs() > 8.*tol { seeds.extend(descents(s,d,o,uv,tol)); }
                }
            }
            // a seed where they touch traces nothing; the touch is a singular point
            let touch_points: Vec<V> = singular.iter().map(|q| q.at).collect();
            seeds.retain(|&p| !touch_points.iter().any(|&q| distance(p,q) <= 1e3*tol));
            let mut traced = super::ssi::trace(a,b,&seeds,lo,hi,tol)?;
            // what no seed reached, proved absent cell by cell between analytic surfaces
            if super::ssi::searchable(a) && super::ssi::searchable(b) {
                let mut more = 0;
                while let Some(p) = super::ssi::unseen(a,da,b,&traced,lo,hi,tol)? {
                    more += 1;
                    if more > 64 { return Err(format!("a {} and a {} meet in more than 64 curves no seed reaches",a.kind(),b.kind())) }
                    let found = super::ssi::trace(a,b,&[p],lo,hi,tol)?;
                    if found.is_empty() {
                        return Err(format!("a {} and a {} meet at {p:?} in a curve that could not be traced",a.kind(),b.kind()))
                    }
                    traced.extend(found);
                }
            }
            (traced,false)
        }
    };
    // where two branches of a closed form cross (two cylinders of one radius whose axes cross)
    for i in 0..curves.len() { for j in i+1..curves.len() {
        let (Some(pi),Some(pj)) = (curves[i].period(),curves[j].period()) else { continue };
        for (_,q) in super::query::crossings_in(a,&curves[i],[0.,pi],&curves[j],[0.,pj],tol).unwrap_or_default() {
            if singular.iter().any(|s| distance(s.at,q) <= 1e3*tol) { continue }
            let ds = [curves[i].tangent(curves[i].inverse(q)),curves[j].tangent(curves[j].inverse(q))];
            singular.push(Point {at:q,kind:Singular::Crossing(ds.iter().flat_map(|&d| [d,scale(d,-1.)]).collect())});
        }
    } }
    let branches = curves.into_iter().map(|c| { let contact = contact(a,b,&c,(lo,hi)); Branch {curve:c,contact} }).collect();
    Ok(Meeting {same:false,branches,singular,closed_form})
}

/// **Meetings computed once and shared** (#735 A2): each distinct surface given an index, two faces
/// on surfaces the kernel holds the same (`ssi::same`) the same one, and each pair of indices'
/// closed-form meeting (`ssi::intersect`) found the first time any two faces on them are met and
/// handed to every other, so the same meeting curve bounds every face that uses it, bit for bit.
#[derive(Clone,Debug,Default)]
pub struct Shared {
    surfaces: Vec<Surface>,
    /// each kind's surfaces among them, which alone a surface of that kind is compared with
    of_kind: std::collections::BTreeMap<&'static str,Vec<usize>>,
    meetings: std::collections::BTreeMap<(usize,usize),Ssi>,
}

impl Shared {
    /// The index of `s` among the surfaces seen: an earlier one it is the same as (exactly, or
    /// within `tol` where both are of the same kind), else a new one.
    pub fn index(&mut self,s: &Surface,tol: f64) -> usize {
        let mine = self.of_kind.entry(s.kind()).or_default();
        if let Some(&i) = mine.iter().find(|&&i| self.surfaces[i] == *s) { return i }
        if let Some(&i) = mine.iter().find(|&&i| super::ssi::same(&self.surfaces[i],s,tol)) { return i }
        self.surfaces.push(s.clone());
        mine.push(self.surfaces.len()-1);
        self.surfaces.len()-1
    }
    /// The meeting of surfaces `i` and `j` (indices), found once: the first surface of each index
    /// met with the other's, in the order asked (a closed form's curves run as the order says).
    pub fn meeting(&mut self,i: usize,j: usize,tol: f64) -> Ssi {
        if let Some(m) = self.meetings.get(&(i,j)) { return m.clone() }
        let m = intersect(&self.surfaces[i],&self.surfaces[j],tol);
        self.meetings.insert((i,j),m.clone());
        m
    }
}

/// `v` made unit, or left as it is where it has no length.
fn unit_or(v: V) -> V { crate::util::space::normalised(v).unwrap_or(v) }


/// **Where two curves cross, whatever they are** (#735 A2: where branches of meetings cross, a
/// singular point of an arrangement), over their stretches `ta` and `tb`: each curve sampled
/// finely enough that its chords stray from it by far less than they are long, every two chords
/// whose boxes come within reach of one another the start of a polish — each curve's point nearest
/// the other's, in turn, until they stop moving — kept where the two come within `reach` of one
/// another inside both stretches, each crossing once. Each crossing's parameter on `a`, on `b`, and
/// its point. What `query::crossings_in` leaves (`None`: a spline's or a trace's crossing with
/// another), found by search, never assumed absent.
pub fn crossings(a: &Curve,ta: [f64;2],b: &Curve,tb: [f64;2],reach: f64) -> Vec<(f64,f64,V)> {
    use super::geom::around;
    use crate::util::bvh::{Bounds,Bvh};
    const N: usize = 512;
    let samples = |c: &Curve,t: [f64;2]| -> Vec<(f64,V)> { (0..=N).map(|k| { let s = t[0]+(t[1]-t[0])*k as f64/N as f64; (s,c.point(s)) }).collect() };
    let (sa,sb) = (samples(a,ta),samples(b,tb));
    let longest = |s: &[(f64,V)]| s.windows(2).map(|w| distance(w[0].1,w[1].1)).fold(0.,f64::max);
    let pad = reach+longest(&sa).max(longest(&sb));
    // each chord's box, `b`'s grown by `pad` and filed in a hierarchy each of `a`'s is asked of
    let chord = |w: &[(f64,V)],grow: f64| { let (lo,hi) = super::geom::hull([w[0].1,w[1].1]);
        Bounds {lo:lo.map(|x| x-grow),hi:hi.map(|x| x+grow)} };
    let boxes = Bvh::new(sb.windows(2).map(|w| chord(w,pad)));
    let wrap = |c: &Curve,t: f64,span: [f64;2]| match c.period() { Some(p) => around(t,span[0],p),None => t };
    let mut out: Vec<(f64,f64,V)> = Vec::new();
    for wa in sa.windows(2) {
        let mut near = Vec::new();
        boxes.query_box(chord(wa,0.),|j| near.push(j));
        near.sort_unstable();
        for j in near {
            // the polish, from the chords' middles: each curve's foot of the other's point
            let (mut s,mut t) = ((wa[0].0+wa[1].0)/2.,(sb[j].0+sb[j+1].0)/2.);
            let mut last = f64::INFINITY;
            for _ in 0..60 {
                t = wrap(b,b.inverse(a.point(s)),tb);
                s = wrap(a,a.inverse(b.point(t)),ta);
                let d = distance(a.point(s),b.point(t));
                if !(d < last*(1.-1e-9)) { last = d; break }
                last = d;
            }
            if !(last <= reach) || s < ta[0]-1e-12 || s > ta[1]+1e-12 || t < tb[0]-1e-12 || t > tb[1]+1e-12 { continue }
            let p = a.point(s);
            // two crossing at a shallow angle meet over a stretch `reach / sin θ` long: one crossing
            let (da,db) = (a.tangent(s),b.tangent(t));
            let sine = (norm(cross(da,db))/(norm(da)*norm(db)).max(1e-300)).max(1e-3);
            if out.iter().any(|x| distance(x.2,p) <= 2.*reach/sine) { continue }
            out.push((s,t,p));
        }
    }
    out
}
