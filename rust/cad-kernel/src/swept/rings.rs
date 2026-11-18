//! **A ring under any rigid motion, its sheet cut where it folds** (docs/swept-solid-design.md
//! steps 6 and 7 under K2): the sweep of a tool of revolution whose meridian is a loop clear of its
//! axis — a torus, a washer, any ring of lines and arcs — carried by a motion no constant twist
//! takes (a turn seen from a turn, a frame along a path), built closed on the one arrangement as
//! K2's convex tools are (`patches`), from the tool where it starts and where it stops and the
//! sheets its contact sweeps over the roll.
//!
//! **The contact is read meridian by meridian.** A ring has no pole: every half-plane through its
//! axis at the angle `θ` cuts it in the whole meridian, and on each piece of it the contact under
//! the instant's twist `v(x) = v₀ + ω × x` is in closed form. On an arc about `c` (a torus's tube,
//! a round) the normal is the arc's radius, so `n · v = n · (v₀ + ω × c)` — the term `ω × r n`
//! is square to `n` — and `n · v = A cos ψ + B sin ψ` has its two roots half a turn apart, never
//! one double: the two branches (`Label::branch`). On a segment the normal is fixed, so `n · v`
//! is affine along it and has one root. At a sharp corner turning outward the edge's fan holds the
//! root wherever its two faces' `n · v` differ in sign (L2's rule); a corner turning inward holds
//! none: the material about it covers every way it can move. So each instant's contact is a set of
//! closed curves, each a graph over `θ` (read at `AROUND` angles and joined where its points pass
//! from piece to piece of the meridian, found in `θ` by bisection), their number the same at every
//! angle — or the instant is refused by name (`Curved`).
//!
//! **Where the sheet folds** (step 6, the fold locus): a point `p` the tool touches at `t` is on the
//! sweep's boundary only if the tool does not hold it at the instants beside `t`. Its signed
//! distance from the tool there, `s(τ)`, has `s(t) = 0 = s'(t)` (the contact), and
//!
//! `s''(t) = κ_m (v · t_m)² + κ_p (v · e_θ)² + n · (ω × v) − n · v̇`
//!
//! read on the tool (`bend`): `κ_m` and `κ_p` the surface's principal curvatures along the
//! meridian `t_m` and the parallel `e_θ` (`κ_p = n_ρ / ρ`), `v̇ = v̇₀ + ω̇ × x` the twist's rate at
//! the point. Where `s'' < 0` the tool holds `p` at every instant beside `t`, so `p` is inside the
//! sweep: the sheet there is the far side of a fold, buried. Where `s''` changes sign the sheet's
//! area factor vanishes — the fold, a cuspidal edge in space. So each contact curve is cut into
//! pieces (`Side`): touching where `s''` stands above `KEPT` of its greatest at the instant, buried
//! below minus that, and between them, where it changes sign, a fold.
//!
//! **Each curve is entered whole, as a closed tube.** A sheet left open at its folds would end
//! inside the sweep in rims that cross what they meet there (the ring where it starts) in curves
//! ending inside a face, which no arrangement parts space by; so the buried pieces are entered too,
//! and the whole stretch between two touching pieces — a fold, the buried piece, the next fold —
//! dipped into the tool along its normal (`dip`): inside the tool at its instant and so inside the
//! sweep, regular where the cuspidal sheet is not, and crossing the tool where it starts and stops
//! rather than touching it, its crossing meeting the touching pieces' touches at the tube's own
//! vertices.
//! Each tube is its own candidate. Over the roll each instant's pieces must be the first's (their
//! kinds in order round each curve), or the build is refused by name (`Curved`): a fold born or
//! dying partway (a swallowtail point), or a contact reaching a corner partway, is not yet cut where
//! it starts. The tubes are fitted as K2's sheets are (`patches::interpolated`), but refined only
//! where they miss: each interval along the roll and round each face halved where the net misses at
//! its middle (`halved`), not every one at once.
//!
//! **A fold pair born or vanishing partway** (#880 T1d): a stretch read in the band about a fold
//! between two stretches alike is a fold pair only where `s''` takes the other sign within it — its
//! least sought along the band, each angle's root on the branch its nearest sample is on, and
//! clearly below nought, by a ten-millionth of the scale — else a dip toward nought that folds
//! nowhere. Where a pair is born or vanishes the curve's kinds change, and the roll is cut there
//! (`changes`: read at `SHAPE_LOOKS` rolls and halved to the rounding); each stretch is fitted on its
//! own at the whole roll's scale (`fitted_over`), its rows a billionth of the roll clear of the cuts,
//! entered against the tool only at the roll's own ends (`entered_at`), and the tubes of every
//! stretch sewn across the cuts (`sew::sewn_open`) into one candidate a tube (`sew::parts`), trimmed
//! by membership (`trim`). A sheet is refused as folding where its area factor turns over between
//! two neighbouring points, or vanishes, not where it is thin: just before a pair is born it is
//! squeezed toward nought where `s''` dips, and regular. A narrow pair's bridge — where `s''` stands
//! below the depth of its least — is the sheet itself, undipped, and its **double curve**, where its
//! outer leaves cross (`double_curve`), is traced from where a stretch first finds them crossing
//! over the whole roll, on the contact itself: in the tube's `(u, t)` one curve over `u` through the
//! swallowtail point, lowest before the pair is seen at an instant, the swallowtail's tail above it.
//! Each stretch is cut along it (`cut_along`), a curve leaving the stretch and coming back two chords,
//! its chords carrying their own curves in the face's parameters (the face's surface folds just past
//! them), and its tail dropped. Where one preimage leaves the roll before the other the double curve
//! ends on the tube's meeting with the tool at the end pose: a face's chords may cross or end on one
//! another (`split_chords`), its pieces the faces of their planar graph (`pieces_among`). **Not
//! yet:** the tumbling torus turned whole, stopped at 2.0, leaves a tube face a hole in no piece.
//!
//! **Its meetings with the tool where it starts and stops are read in its own domain** (`traces`):
//! at an end pose every piece's rim lies on the tool, touching it, and a tube that folds runs on
//! from its rim into the tool beside it (near each fold, and wherever the sweep comes back on the
//! ring where it started). A search for that meeting in space starts beside the touch, where the
//! two are all but tangent; read as the zero set of the ring's signed distance (`Ring::signed`) over
//! the face's parameters, the touch divided out, it is found exactly.
//!
//! **A tube that folds is entered only where it stands outside the tool at both end poses**
//! (`trimmed`): each face cut along those meetings and the pieces inside dropped — inside the tool
//! there, inside the sweep, parting nothing of it. Entered from its rim, a fold piece's sheet lay
//! over its own rim just past it (the contact sliding along itself near a fold), a meeting of the
//! tube with itself no arrangement seeks: the ring's face was left a slit along a touching piece's
//! rim, and the sweep's inside one cell with its outside. Trimmed, the tube's every free edge lies
//! on the tool where it starts or stops — a touching piece's rim, or a cut — and those are given as
//! its whole meeting with it.
//!
//! **Built**: the tumbling torus (`swept_tumble.sv`) closed and judged, its volume the definition's
//! to within its reading off the fitted sheets (`tests/swept_rings.rs`, slow tier). The road is not
//! yet a class the dispatch admits: `closed::ring` builds it alone, and `closed::ring_cut_from` cut
//! from a blank (a ring pushed into a block at a lean, a torus turned across one). A ring of several
//! faces whose sheet crosses the tool at an end pose across two of them is refused by name: the
//! crossing is not yet cut where it passes from face to face.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::boolean::Given;
use crate::geom::{Curve,Frame};
use crate::nurbs::Net;
use crate::topo::Brep;
use crate::trajectory::{Pose,Trajectory,Twist};
use crate::util::space::{add,cross,dot,norm,scale};
use super::closed::Unbuilt;
use super::patches::{Element,Meridian,On,meridian,missed};
use std::f64::consts::TAU;
use std::sync::Arc;

type V = [f64;3];
type P2 = [f64;2];

/// How the meridian turns where two of its pieces meet: running on smoothly, outward (a convex
/// sharp corner, an edge whose fan holds a contact) or inward (a concave one, which holds none).
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Corner { Smooth,Convex,Concave }

/// **A ring**: a tool of revolution whose meridian is one loop clear of its axis, read off its
/// B-rep — its axis (`frame`), the loop's pieces walked counter-clockwise in `(ρ, z)` (the solid on
/// their left), the tool's face each is, how the loop turns after each (`corners[k]`, between
/// piece `k` and the next round), its centre on the axis and its size.
#[derive(Clone,Debug)]
pub struct Ring { pub frame: Frame,pub elements: Vec<Element>,pub faces: Vec<usize>,pub corners: Vec<Corner>,pub centre: V,pub size: f64,
    /// How thick the ring is: the least of its meridian loop's extents across and along the axis.
    pub thickness: f64 }

/// A point of a meridian piece at the fraction `f` of the way along it.
fn along(e: &Element,f: f64) -> P2 {
    match *e {
        Element::Line {a,b} => [a[0]+(b[0]-a[0])*f,a[1]+(b[1]-a[1])*f],
        Element::Arc {c,r,from,sweep} => { let (s,co) = (from+sweep*f).dsin_cos(); [c[0]+r*co,c[1]+r*s] }
    }
}

/// The piece walked the other way.
fn reversed(e: Element) -> Element {
    match e {
        Element::Line {a,b} => Element::Line {a:b,b:a},
        Element::Arc {c,r,from,sweep} => Element::Arc {c,r,from:from+sweep,sweep:-sweep},
    }
}

impl Ring {
    /// **The tool read as a ring**: every face a plane square to one axis, a cylinder, a cone or a
    /// torus about it, or a ball centred on it (`patches::meridian`), their pieces one loop round
    /// the meridian that keeps clear of the axis. Refused (`Unbuilt::Curved`, at a point of the
    /// face) otherwise.
    pub fn read(tool: &Brep) -> Result<Ring,Unbuilt> {
        let Meridian {frame,elements,faces,at,size} = meridian(tool)?;
        let n = elements.len();
        let refused = |k: usize| Unbuilt::Curved {at:at[faces[k]]};
        if n == 0 { return Err(Unbuilt::Curved {at:at.first().copied().unwrap_or([0.;3])}) }
        let reach = 1e-6*(1.+size);
        // clear of the axis: no pole, every half-plane through it holding the whole meridian
        for (k,e) in elements.iter().enumerate() {
            if (0..=64).any(|i| along(e,i as f64/64.)[0] <= reach) { return Err(refused(k)) }
        }
        // walked round from the first piece as it is, each next starting where the last ends
        let near = |p: P2,q: P2| (p[0]-q[0]).dhypot(p[1]-q[1]) <= reach;
        let mut order = vec![(0,elements[0])];
        let mut used = vec![false;n];
        used[0] = true;
        while order.len() < n {
            let tip = order.last().unwrap().1.ends()[1];
            let Some((i,e)) = (0..n).filter(|&i| !used[i]).flat_map(|i| [(i,0),(i,1)]).find(|&(i,e)| near(elements[i].ends()[e],tip))
                else { return Err(refused(order.last().unwrap().0)) };
            used[i] = true;
            order.push((i,if e == 0 { elements[i] } else { reversed(elements[i]) }));
        }
        if !near(order.last().unwrap().1.ends()[1],order[0].1.ends()[0]) { return Err(refused(order.last().unwrap().0)) }
        // counter-clockwise, the solid on the left: the loop's area positive
        let mut area = 0.;
        for (_,e) in &order {
            let pts: Vec<P2> = (0..=32).map(|i| along(e,i as f64/32.)).collect();
            for w in pts.windows(2) { area += w[0][0]*w[1][1]-w[1][0]*w[0][1] }
        }
        if area < 0. {
            order.reverse();
            for o in &mut order { o.1 = reversed(o.1) }
        }
        let faces: Vec<usize> = order.iter().map(|&(i,_)| faces[i]).collect();
        let elements: Vec<Element> = order.into_iter().map(|(_,e)| e).collect();
        let corners = (0..n).map(|k| {
            let (t0,t1) = (elements[k].tangent(1),elements[(k+1)%n].tangent(0));
            let (turn,on) = (t0[0]*t1[1]-t0[1]*t1[0],t0[0]*t1[0]+t0[1]*t1[1]);
            if turn > 1e-9 || turn >= 0. && on < 0. { Corner::Convex } else if turn < -1e-9 { Corner::Concave } else { Corner::Smooth }
        }).collect();
        let zs: Vec<f64> = elements.iter().flat_map(|e| (0..=16).map(move |i| along(e,i as f64/16.)[1])).collect();
        let (zlo,zhi) = zs.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),&z| (l.min(z),h.max(z)));
        let rs: Vec<f64> = elements.iter().flat_map(|e| (0..=16).map(move |i| along(e,i as f64/16.)[0])).collect();
        let (rlo,rhi) = rs.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),&r| (l.min(r),h.max(r)));
        let middle = 0.5*(zlo+zhi);
        let size = elements.iter().flat_map(|e| (0..=16).map(move |i| along(e,i as f64/16.)))
            .map(|p| p[0].dhypot(p[1]-middle)).fold(0.,f64::max);
        Ok(Ring {frame,elements,faces,corners,centre:frame.at([0.,0.,middle]),size,thickness:(zhi-zlo).min(rhi-rlo)})
    }

    /// The directions at the angle `θ` about the axis: outward, round the axis and along it.
    fn axes(&self,theta: f64) -> (V,V,V) {
        let (s,c) = theta.dsin_cos();
        (self.frame.dir([c,s,0.]),self.frame.dir([-s,c,0.]),self.frame.z)
    }

    /// The point `(ρ, z)` of the meridian in the half-plane at `θ`.
    fn lift(&self,theta: f64,p: P2) -> V {
        let (s,c) = theta.dsin_cos();
        self.frame.at([p[0]*c,p[0]*s,p[1]])
    }

    /// **The contact labelled `l` in the half-plane at `θ`** under the twist `w` (read on the tool),
    /// in closed form: on an arc the root of its branch, on a segment its one root, on a convex
    /// corner's edge the fan's. `strict`: only within the piece's own stretch (half open, so a root
    /// on a join is counted once); otherwise wherever its closed form puts it, as a piece's end
    /// reads it.
    fn root(&self,w: &Twist,theta: f64,l: Label,strict: bool) -> Option<Root> {
        let (er,_,ez) = self.axes(theta);
        let n = self.elements.len();
        let lift3 = |m: P2| add(scale(er,m[0]),scale(ez,m[1]));
        match l.on {
            On::Face(k) => match self.elements[k] {
                Element::Arc {c,r,from,sweep} => {
                    let u = w.velocity(self.lift(theta,c));
                    let (a,b) = (dot(er,u),dot(ez,u));
                    if !(a.dhypot(b) > 0.) { return None }
                    let s = if l.branch { 1. } else { -1. };
                    let psi = (-a*s).datan2(b*s);
                    let x = ((psi-from)*sweep.signum()).rem_euclid(TAU);
                    let span = sweep.abs();
                    if strict && !(span >= TAU-1e-12 || x < span) { return None }
                    let (sp,cp) = psi.dsin_cos();
                    let g = sweep.signum();
                    let m = [c[0]+r*cp,c[1]+r*sp];
                    Some(Root {x:self.lift(theta,m),n:lift3([g*cp,g*sp]),sigma:k as f64+x.min(span)/span,
                        bend:Bend::Face {km:g/r,tm:lift3([-sp,cp]),kp:g*cp/m[0]}})
                }
                Element::Line {a,b} => {
                    let e = &self.elements[k];
                    let nm = e.normal(a);
                    let nn = lift3(nm);
                    let at = |f: f64| dot(nn,w.velocity(self.lift(theta,along(e,f))));
                    let (g0,g1) = (at(0.),at(1.));
                    if !(g0 != g1) { return None }
                    let f = g0/(g0-g1);
                    if strict && !(f >= 0. && f < 1.) || !f.is_finite() { return None }
                    let m = along(e,f);
                    let l = (b[0]-a[0]).dhypot(b[1]-a[1]);
                    Some(Root {x:self.lift(theta,m),n:nn,sigma:k as f64+f.clamp(0.,1.),
                        bend:Bend::Face {km:0.,tm:lift3([(b[0]-a[0])/l,(b[1]-a[1])/l]),kp:nm[0]/m[0]}})
                }
            },
            On::Edge(k) => {
                if self.corners[k] != Corner::Convex { return None }
                let m = self.elements[k].ends()[1];
                let x = self.lift(theta,m);
                let v = w.velocity(x);
                let (na,nb) = (self.elements[k].normal(m),self.elements[(k+1)%n].normal(m));
                let (aa,ab) = (na[1].datan2(na[0]),nb[1].datan2(nb[0]));
                let span = (ab-aa).rem_euclid(TAU);
                let (gr,gz) = (dot(er,v),dot(ez,v));
                if !(gr.dhypot(gz) > 0.) { return None }
                let alpha = (-gr).datan2(gz);
                // of the two roots half a turn apart, the one in the fan (or nearest it)
                let off = |al: f64| { let x = (al-aa).rem_euclid(TAU); if x <= span { 0. } else { (x-span).min(TAU-x) } };
                let alpha = if off(alpha) <= off(alpha+std::f64::consts::PI) { alpha } else { alpha+std::f64::consts::PI };
                let x_in = (alpha-aa).rem_euclid(TAU);
                if strict && !(x_in > 0. && x_in < span) { return None }
                let (sa,ca) = alpha.dsin_cos();
                Some(Root {x,n:lift3([ca,sa]),sigma:((k+1)%n) as f64,bend:Bend::Fan})
            }
        }
    }

    /// The greatest `|s''|` of the contacts on the faces `read` at the angles `thetas`.
    /// The scale `s''` is read against over `roll`: its greatest at the instant `t`.
    fn scale_at(&self,path: &Trajectory,t: f64,roll: [f64;2]) -> Result<f64,Unbuilt> {
        let (w,dw) = rates(path,t,roll)?;
        let thetas: Vec<f64> = (0..AROUND).map(|i| TAU*i as f64/AROUND as f64).collect();
        let read: Vec<Vec<(Label,Root)>> = thetas.iter().map(|&th| self.roots(&w,th)).collect();
        Ok(self.scale(&w,&dw,&thetas,&read))
    }

    fn scale(&self,w: &Twist,dw: &Twist,thetas: &[f64],read: &[Vec<(Label,Root)>]) -> f64 {
        let mut most = 0f64;
        for (i,r) in read.iter().enumerate() {
            for (_,root) in r { let b = self.bend(w,dw,thetas[i],root); if b.is_finite() { most = most.max(b.abs()) } }
        }
        most
    }

    /// Every contact in the half-plane at `θ`, in order round the meridian.
    fn roots(&self,w: &Twist,theta: f64) -> Vec<(Label,Root)> {
        let mut out: Vec<(Label,Root)> = Vec::new();
        for (k,e) in self.elements.iter().enumerate() {
            let branches: &[bool] = match e { Element::Arc {..} => &[true,false],Element::Line {..} => &[true] };
            for &branch in branches {
                let l = Label {on:On::Face(k),branch};
                if let Some(r) = self.root(w,theta,l,true) { out.push((l,r)) }
            }
            if self.corners[k] == Corner::Convex {
                let l = Label {on:On::Edge(k),branch:true};
                if let Some(r) = self.root(w,theta,l,true) { out.push((l,r)) }
            }
        }
        out.sort_by(|a,b| a.1.sigma.total_cmp(&b.1.sigma));
        out
    }

    /// **How the tool's distance from the point it touches at `r` bends over the roll there**,
    /// `s''`: positive where the tool holds the point at no instant beside (the sheet touching),
    /// negative where it holds it at every one (the sheet's far side of a fold, buried). A fan's
    /// is unbounded: an edge's normals turn through their fan in no length.
    fn bend(&self,w: &Twist,dw: &Twist,theta: f64,r: &Root) -> f64 {
        match r.bend {
            Bend::Fan => f64::INFINITY,
            Bend::Face {km,tm,kp} => {
                let (_,et,_) = self.axes(theta);
                let v = w.velocity(r.x);
                km*dot(v,tm)*dot(v,tm)+kp*dot(v,et)*dot(v,et)+dot(r.n,cross(w.omega,v))-dot(r.n,dw.velocity(r.x))
            }
        }
    }
}

/// What a contact lies on — a face's piece of the meridian or a sharp corner's edge — and, on an
/// arc, which of its two roots (`branch`: `+` the one along `(B, −A)`).
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub struct Label { pub on: On,pub branch: bool }

/// What `s''` is read from at a contact on a face: the meridian's curvature and its unit tangent,
/// and the parallel's curvature; a fan's is unbounded.
#[derive(Clone,Copy,Debug)]
enum Bend { Face { km: f64,tm: V,kp: f64 },Fan }

/// A contact in a half-plane: the point and its outward normal (the tool's coordinates), its place
/// round the meridian (`k + f` on piece `k`), and how it bends.
#[derive(Clone,Copy,Debug)]
struct Root { x: V,n: V,sigma: f64,bend: Bend }

/// How the sheet stands at a piece of the contact: touching (`s''` above `KEPT` of its greatest at
/// the instant, the sweep's boundary where nothing else holds it), the far side of a fold (below
/// minus that, buried), or within a hair of a fold between the two, where it is bridged.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Side { Touching,Fold,Buried }

/// A stretch of a contact curve: the angles `[from, to)` it runs over (`to > from`, a curve running
/// round more than once past `TAU`), what it lies on, and how the sheet stands there.
#[derive(Clone,Copy,Debug)]
struct Piece { from: f64,to: f64,label: Label,side: Side }

/// The kind of a piece: what it lies on and how the sheet stands there.
type Kind = (Label,Side);

/// One instant: its twist on the tool, and each contact curve's pieces in order.
#[derive(Clone,Debug)]
struct Instant { w: Twist,curves: Vec<Vec<Piece>> }

/// How many angles round the axis an instant's contacts are first read at.
const AROUND: usize = 256;

/// How near a fold, as a part of its greatest `|s''|` at the instant, a contact's `s''` may come and
/// its sheet still be entered as it is: nearer, its area factor vanishing at the fold, the sheet is
/// bridged (`bridged`).
const KEPT: f64 = 0.1;

/// The twist on the tool at `t` and its rate by `t`, read over `[a, b]` (the rate by differences
/// within it).
fn rates(path: &Trajectory,t: f64,[a,b]: [f64;2]) -> Result<(Twist,Twist),Unbuilt> {
    let w = |s: f64| -> Result<Twist,Unbuilt> { Ok(Twist::body(path.at(s)?)) };
    let h = 1e-4*(b-a).abs().max(1e-9);
    let w0 = w(t)?;
    let comb = |ws: [(f64,Twist);3]| Twist {
        omega:std::array::from_fn(|k| ws.iter().map(|(c,x)| c*x.omega[k]).sum()),
        v0:std::array::from_fn(|k| ws.iter().map(|(c,x)| c*x.v0[k]).sum()),
    };
    let dw = if t-h >= a.min(b) && t+h <= a.max(b) {
        comb([(-0.5/h,w(t-h)?),(0.,w0),(0.5/h,w(t+h)?)])
    } else if t+2.*h <= a.max(b) {
        comb([(-1.5/h,w0),(2./h,w(t+h)?),(-0.5/h,w(t+2.*h)?)])
    } else {
        comb([(1.5/h,w0),(-2./h,w(t-h)?),(0.5/h,w(t-2.*h)?)])
    };
    Ok((w0,dw))
}

impl Ring {
    /// **The contact at the instant `w` (its rate `dw`)**: each closed curve's pieces, in order
    /// along it, each join settled in `θ` to rounding. `Err` with a point of the tool where the
    /// number of contacts changes round the axis.
    fn instant(&self,w: Twist,dw: Twist,scale: Option<f64>) -> Result<Instant,V> {
        let thetas: Vec<f64> = (0..AROUND).map(|i| TAU*i as f64/AROUND as f64).collect();
        let read: Vec<Vec<(Label,Root)>> = thetas.iter().map(|&th| self.roots(&w,th)).collect();
        let m = read[0].len();
        let witness = || self.lift(0.,self.elements[0].ends()[0]);
        if m == 0 { return Err(witness()) }
        if let Some(i) = read.iter().position(|r| r.len() != m) { return Err(read[i].first().map_or_else(witness,|r| r.1.x)) }
        let nf = self.elements.len() as f64;
        let apart = |a: f64,b: f64| { let d = (a-b).rem_euclid(nf); d.min(nf-d) };
        // the turn round the meridian carrying each angle's contacts onto the next's
        let shifts: Vec<usize> = (0..AROUND).map(|i| {
            let (p,q) = (&read[i],&read[(i+1)%AROUND]);
            (0..m).min_by(|&s,&t| {
                let cost = |s: usize| (0..m).map(|j| apart(p[j].1.sigma,q[(j+s)%m].1.sigma)).sum::<f64>();
                cost(s).total_cmp(&cost(t))
            }).unwrap()
        }).collect();
        // the scale `s''` is read against: its greatest on the faces at the first instant, the same
        // at every one (read at each, a greatest that passes from one point to another would move
        // the folds' cuts unsmoothly along the roll)
        let scale_bend = scale.unwrap_or_else(|| self.scale(&w,&dw,&thetas,&read));
        let kind = |th: f64,(l,r): &(Label,Root)| -> Kind {
            let b = self.bend(&w,&dw,th,r);
            (*l,if b > KEPT*scale_bend { Side::Touching } else if b < -KEPT*scale_bend { Side::Buried } else { Side::Fold })
        };
        // each curve followed from a contact at the first angle, round as many turns as it takes
        let mut seen = vec![false;m];
        let mut curves = Vec::new();
        for start in 0..m {
            if seen[start] { continue }
            // each sample: its angle run on round the turns, the angle's index, and the contact's
            let mut samples: Vec<(f64,usize,usize)> = Vec::new();
            let (mut idx,mut turn) = (start,0);
            loop {
                seen[idx] = true;
                for i in 0..AROUND { samples.push((TAU*turn as f64+thetas[i],i,idx)); idx = (idx+shifts[i])%m }
                turn += 1;
                if idx == start { break }
            }
            let span = TAU*turn as f64;
            let at = |th: f64,sigma: f64| -> Option<(Label,Root)> {
                let rs = self.roots(&w,th.rem_euclid(TAU));
                rs.into_iter().min_by(|a,b| apart(a.1.sigma,sigma).total_cmp(&apart(b.1.sigma,sigma)))
            };
            let raw: Vec<Kind> = samples.iter().map(|&(_,i,j)| kind(thetas[i],&read[i][j])).collect();
            let s = samples.len();
            // a stretch near a fold is one only where the sheet turns there from touching to
            // buried: one between two stretches alike is theirs (a contact whose `s''` dips toward
            // nothing and rises again folds nowhere)
            let mut kinds = raw.clone();
            // the narrow pairs found (see below): each its stretch of angles, the branch of the samples
            // across it, the side about it, and its least and where
            let mut narrow: Vec<([f64;2],Vec<(f64,f64)>,f64,f64,f64)> = Vec::new();
            match (0..s).find(|&i| raw[i].1 != Side::Fold) {
                None => {
                    let side = if self.bend(&w,&dw,thetas[samples[0].1],&read[samples[0].1][samples[0].2].1) > 0. { Side::Touching } else { Side::Buried };
                    for k in &mut kinds { k.1 = side }
                }
                Some(first) => {
                    let mut i = first;
                    for _ in 0..s {
                        let k = (i+1)%s;
                        if raw[k].1 == Side::Fold {
                            let mut j = k;
                            while raw[(j+1)%s].1 == Side::Fold { j = (j+1)%s }
                            let (before,after) = (raw[i].1,raw[(j+1)%s].1);
                            // (unless `s''` takes the other sign within it: two folds and a sliver
                            // of the other side between them, narrower than the band a fold is read
                            // in — the tumbling torus, its sheet folding over a twentieth of a face its
                            // kinds read as touching all round, #880 T1d. Its least (greatest) is
                            // sought along the stretch, each step between samples read 64 times and
                            // the least of those settled by golden sections: a pair newly born is
                            // narrower than the samples)
                            let other = (before == after) && {
                                let sign = if before == Side::Touching { 1. } else { -1. };
                                let first = samples[(k+s-1)%s].0;
                                let mut last = samples[(j+1)%s].0;
                                if last <= first { last += span }
                                // (each angle's root the one nearest the branch its nearest sample is
                                // on: the curve followed, not the other root at a far angle)
                                let near: Vec<(f64,f64)> = (0..(j+s-k)%s+3).map(|x| {
                                    let (th,ia,ja) = samples[(k+s-1+x)%s];
                                    (if th < first { th+span } else { th },read[ia][ja].1.sigma)
                                }).collect();
                                let value = |th: f64| {
                                    let sigma = near.iter().min_by(|p,q| (p.0-th).abs().total_cmp(&(q.0-th).abs())).map_or(0.,|p| p.1);
                                    at(th,sigma).map_or(f64::INFINITY,|r| sign*self.bend(&w,&dw,th.rem_euclid(TAU),&r.1))
                                };
                                let looks = 64*((j+s-k)%s+3);
                                let step = (last-first)/looks as f64;
                                let (mut best,mut at_best) = (f64::INFINITY,first);
                                for x in 0..=looks { let th = first+step*x as f64; let v = value(th); if v < best { best = v; at_best = th } }
                                let (mut lo,mut hi) = ((at_best-step).max(first),(at_best+step).min(last));
                                let g = 0.5*(5f64.sqrt()-1.);
                                for _ in 0..60 {
                                    let (x1,x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
                                    if value(x1) < value(x2) { hi = x2 } else { lo = x1 }
                                }
                                // (clearly below nought, by a ten-millionth of the scale: so the
                                // instants a hair either side of where a pair is born read alike)
                                let mid = 0.5*(lo+hi);
                                let least = best.min(value(mid));
                                if least < -1e-7*scale_bend { narrow.push(([first,last],near.clone(),sign,mid,least)) }
                                least < -1e-7*scale_bend
                            };
                            // (a dip that folds nowhere is the stretches' about it; and a pair narrower
                            // than the band is read as touching here, its bridge put in below)
                            if before == after {
                                let mut x = k;
                                loop { kinds[x].1 = before; if x == j { break } x = (x+1)%s }
                            }
                            let _ = other;
                            i = (j+1)%s;
                        } else { i = k }
                        if i == first { break }
                    }
                }
            }
            let mut joins: Vec<(f64,Kind)> = Vec::new();
            for i in 0..s {
                let (mut a,b) = (kinds[i],kinds[(i+1)%s]);
                if a == b { continue }
                let mut lo = samples[i].0;
                let hi = if i+1 < s { samples[i+1].0 } else { span };
                let (_,ia,ja) = samples[i];
                let sig0 = read[ia][ja].1.sigma;
                // each change between the two angles in turn: a narrow stretch (a fold's bridge)
                // may stand wholly between them
                // (the sheet's side read only where the two stretches' differ)
                let alike = a.1 == b.1;
                let side = a.1;
                let seen = |th: f64,r: &(Label,Root)| -> Kind { let k = kind(th,r); if alike { (k.0,side) } else { k } };
                while a != b {
                    let mut missed = None;
                    let (l,h) = crate::util::roots::bisect(lo,hi,|l,h| 0.5*(l+h),|l,h| { let m = 0.5*(l+h); m > l && m < h },
                        |mid| match at(mid,sig0) {
                            Some(r) => seen(mid.rem_euclid(TAU),&r) == a,
                            None => { missed.get_or_insert(mid); true }
                        });
                    if missed.is_some() { return Err(read[ia][ja].1.x) }
                    let Some(next) = at(h,sig0).map(|r| seen(h.rem_euclid(TAU),&r)) else { return Err(read[ia][ja].1.x) };
                    if next == a { return Err(read[ia][ja].1.x) }
                    joins.push((0.5*(l+h),next));
                    (a,lo) = (next,h);
                }
            }
            // **a narrow pair's bridge** (#880 T1d): where `s''` stands below the depth of its least
            // below nought — about both folds, and short of where its outer leaves cross, at twice
            // that depth (the swallowtail's normal form: folds at `w` either side of its middle, the
            // leaves crossing at `√3 w`, `s''` there `2 |least|`)
            for ([first,last],near,sign,mid,least) in &narrow {
                let value = |th: f64| {
                    let sigma = near.iter().min_by(|p,q| (p.0-th).abs().total_cmp(&(q.0-th).abs())).map_or(0.,|p| p.1);
                    at(th,sigma).map(|r| (r.0,sign*self.bend(&w,&dw,th.rem_euclid(TAU),&r.1)))
                };
                let depth = least.abs();
                let edge = |from: f64,to: f64| -> Option<f64> {
                    let (fa,fb) = (value(from)?.1-depth,value(to)?.1-depth);
                    if (fa < 0.) == (fb < 0.) { return None }
                    let (x,y) = crate::util::roots::bisect(from,to,|x,y| 0.5*(x+y),|x,y| { let m = 0.5*(x+y); m > x && m < y },
                        |m| value(m).is_some_and(|v| (v.1-depth < 0.) == (fa < 0.)));
                    Some(0.5*(x+y))
                };
                let (Some(l),Some(r)) = (edge(*first,*mid),edge(*mid,*last)) else { return Err(read[0][0].1.x) };
                let side = if *sign > 0. { Side::Touching } else { Side::Buried };
                let (Some((ll,_)),Some((lr,_))) = (value(l),value(r)) else { return Err(read[0][0].1.x) };
                let wrap = |x: f64| x.rem_euclid(span);
                joins.push((wrap(l),(ll,Side::Fold)));
                joins.push((wrap(r),(lr,side)));
            }
            joins.sort_by(|x,y| x.0.total_cmp(&y.0));
            let pieces = if joins.is_empty() {
                vec![Piece {from:0.,to:span,label:kinds[0].0,side:kinds[0].1}]
            } else {
                (0..joins.len()).map(|j| {
                    let (from,k) = joins[j];
                    let to = if j+1 < joins.len() { joins[j+1].0 } else { joins[0].0+span };
                    Piece {from,to,label:k.0,side:k.1}
                }).collect()
            };
            curves.push(pieces);
        }
        Ok(Instant {w,curves})
    }
}

/// The kinds of a curve's pieces in order.
fn kinds(c: &[Piece]) -> Vec<Kind> { c.iter().map(|p| (p.label,p.side)).collect() }

/// The turns `s` carrying the pieces of `c` onto `first`'s kinds, `c[k + s]` standing for `first[k]`.
fn turns_onto(c: &[Piece],first: &[Kind]) -> Vec<usize> {
    let n = first.len();
    (0..n).filter(|&s| c.len() == n && (0..n).all(|k| (c[(k+s)%n].label,c[(k+s)%n].side) == first[k])).collect()
}

/// **A contact curve entered whole**: its index among the first instant's curves, and its pieces
/// (indices round it, in order) — touching, buried and the bridges across its folds alike, a tube
/// closed round with no free rim. A sheet left open where it folds would end inside the sweep in a
/// rim that crosses what it meets there (the ring where it starts) in curves ending inside a face,
/// which no arrangement parts space by; its far side, buried, closes it, and membership keeps none
/// of it.
#[derive(Clone,Debug)]
struct Run { curve: usize,pieces: Vec<usize> }

/// The curves of `first` that touch somewhere, each entered whole; one wholly buried left out.
fn runs(first: &[Vec<Piece>]) -> Vec<Run> {
    first.iter().enumerate().filter(|(_,c)| c.iter().any(|p| p.side == Side::Touching))
        .map(|(c,pieces)| Run {curve:c,pieces:(0..pieces.len()).collect()}).collect()
}

/// How deep the stretch of a contact curve between two touching pieces dips into the tool at its
/// deepest, as a part of the ring's thickness (`dip`).
const DEEP: f64 = 0.05;

/// How far into the roll from either end, as a part of it, a stretch's dip comes in (`dip`).
const RAMP: f64 = 0.05;

/// **How deep the stretch between two touching pieces dips into the tool**, as a part of its
/// deepest (`DEEP`), at the fraction `λ` across it (piece by piece, each of its pieces an equal
/// share, from where the one touching piece ends to where the next begins: a fold's narrow piece
/// dips as deep as a buried one's) and the roll `t` of `[a, b]`: `sin(π λ) ψ((t − a)/τ) ψ((b − t)/τ)`,
/// `ψ(x) = 1 − e^(−x)`, `τ` the `RAMP` of the roll — steep at either end, so beside a corner where
/// the stretch meets a touching piece at an end pose the dip's first order outruns the touching
/// piece's second, and smooth, so the roll is fitted as finely there as elsewhere. The stretch holds the folds at either end and the buried piece
/// between: inside the tool at its instant, so inside the sweep, it bounds nothing kept. Dipped, it
/// is a regular surface where the contact's sheet is cuspidal: at a fold the sheet's rates across
/// and along the roll are one way, and the dip's rate across, along the tool's normal, parts them
/// (and at either end of the roll, where that dip is nothing, its rate along the roll does). At
/// either end of the roll it is nothing, the stretch's rim the contact on the tool there as a
/// touching piece's is, and its rate along the roll carries the stretch into the tool across it —
/// a crossing, not a touch — so the tube meets the tool where it starts and stops in its touching
/// pieces' rims, touching, and the stretches' rims, crossing, joined at the tube's own vertices.
fn dip(lambda: f64,t: f64,[a,b]: [f64;2]) -> f64 {
    let tau = RAMP*(b-a);
    let ramp = |x: f64| 1.-(-x.max(0.)).exp();
    (std::f64::consts::PI*lambda).dsin()*ramp((t-a)/tau)*ramp((b-t)/tau)
}

/// One roll's reading: the instant, the pose and each first curve's pieces turned into the first
/// instant's order.
#[derive(Clone,Debug)]
struct Read { instant: Instant,pose: Pose,curves: Vec<Vec<Piece>> }



/// **A ring's sheets fitted**: each contact curve's net over (the fraction round it, a piece a unit,
/// the roll), closed round, and what each of its faces lies on and how the sheet stands there.
#[derive(Clone,Debug)]
pub struct Fitted { pub nets: Vec<Net>,pub on: Vec<Vec<On>>,pub sides: Vec<Vec<Side>>,pub roll: [f64;2],
    /// Per net, where its outer leaves cross about each narrow pair's bridge (#880 T1d): the double
    /// curve's preimage on the leaf before the bridge and on the one after, each a row's `(u, t)`.
    pub doubles: Vec<Vec<Vec<[f64;2]>>>,
    /// Where each narrow pair's outer leaves are first found crossing in this stretch: the two
    /// preimages as angles round the axis, rolls and places round the meridian (`double_curve`).
    pub seeds: Vec<[(f64,f64,f64);2]> }

/// A face of a tube: the piece of its curve it lies on (by its index among the run's), and which of
/// the `of` equal shares of the piece's stretch it is.
#[derive(Clone,Copy,Debug)]
struct Face { piece: usize,share: usize,of: usize }

/// The least number of faces a tube is entered in, and how finely its pieces are cut at least.
const LEAST_CLOSED: usize = 4;
const PER: usize = 2;

/// The first rows and points, and the most either may be refined to (as K2's).
const FIRST_ROWS: usize = 16;
const FIRST_POINTS: usize = 8;
const MOST_ROWS: usize = 1024;
const MOST_POINTS: usize = 1024;


/// The reader of a ring's instants over a roll, each roll read once.
struct Reader<'a> {
    ring: &'a Ring,path: &'a Trajectory,roll: [f64;2],first: Vec<Vec<Piece>>,
    /// The roll the twist's rate is read by differences within: the whole roll's, as its cuts are
    /// found (`changes`), whatever stretch of it the reader fits.
    whole: [f64;2],
    /// The scale `s''` is read against at every instant: its greatest at the first.
    scale: f64,
    cache: std::sync::Mutex<std::collections::BTreeMap<u64,Read>>,
}

impl<'a> Reader<'a> {
    fn new(ring: &'a Ring,path: &'a Trajectory,roll: [f64;2]) -> Result<Reader<'a>,Unbuilt> {
        let scale = ring.scale_at(path,roll[0],roll)?;
        Reader::within(ring,path,roll,roll,roll[0],scale)
    }

    /// The reader over `roll`, `s''` read against `scale`, its curves matched to those at `first`.
    fn within(ring: &'a Ring,path: &'a Trajectory,whole: [f64;2],roll: [f64;2],first: f64,scale: f64) -> Result<Reader<'a>,Unbuilt> {
        let (w,dw) = rates(path,first,whole)?;
        let pose = path.pose(first)?;
        let i = ring.instant(w,dw,Some(scale)).map_err(|at| Unbuilt::Curved {at:pose.point(at)})?;
        Ok(Reader {ring,path,roll,first:i.curves,whole,scale,cache:Default::default()})
    }

    /// The instant at `t`, its curves matched to the first instant's: each first curve by the one
    /// whose kinds turn onto its own, the nearest where several do; refused by name (`Curved`)
    /// where none does.
    fn read(&self,t: f64) -> Result<Read,Unbuilt> {
        if let Some(r) = self.cache.lock().unwrap().get(&t.to_bits()) { return Ok(r.clone()) }
        let (w,dw) = rates(self.path,t,self.whole)?;
        let pose = self.path.pose(t)?;
        let i = self.ring.instant(w,dw,Some(self.scale)).map_err(|at| Unbuilt::Curved {at:pose.point(at)})?;
        let refused = |c: &[Piece]| Unbuilt::Curved {at:pose.point(self.ring.root(&w,c[0].from.rem_euclid(TAU),c[0].label,false)
            .map_or(self.ring.centre,|r| r.x))};
        if std::env::var_os("SOLVENT_RING_DEBUG").is_some() && !same_shape(&self.first,&i.curves) {
            let show = |cs: &[Vec<Piece>]| cs.iter().map(|c| c.iter().map(|p| match p.side { Side::Touching => 'T',Side::Fold => 'F',Side::Buried => 'B' }).collect::<String>()).collect::<Vec<_>>().join("|");
            eprintln!("ring: at {t} over {:?} the shape {} against {}",self.roll,show(&i.curves),show(&self.first));
        }
        if i.curves.len() != self.first.len() { return Err(refused(&i.curves[0])) }
        let wrap = |x: f64| (x+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI;
        let mut taken = vec![false;i.curves.len()];
        let mut curves = Vec::with_capacity(self.first.len());
        for f in &self.first {
            let fk = kinds(f);
            // the curves whose kinds turn onto this one's, and the turn whose first join stands
            // nearest its own
            let best = i.curves.iter().enumerate().filter(|(j,_)| !taken[*j]).flat_map(|(j,c)| turns_onto(c,&fk).into_iter().map(move |s| (j,s)))
                .min_by(|&(j,s),&(k,r)| wrap(i.curves[j][s].from-f[0].from).abs().total_cmp(&wrap(i.curves[k][r].from-f[0].from).abs()));
            let Some((j,s)) = best else { return Err(refused(f)) };
            taken[j] = true;
            let mut c = i.curves[j].clone();
            c.rotate_left(s);
            // its angles unwrapped to run on from the first piece's
            let shift = TAU*((f[0].from-c[0].from)/TAU).round();
            let mut last = f64::NEG_INFINITY;
            for p in &mut c {
                p.from += shift; p.to += shift;
                while p.from < last { p.from += TAU; p.to += TAU }
                last = p.from;
            }
            curves.push(c);
        }
        let r = Read {instant:i,pose,curves};
        self.cache.lock().unwrap().insert(t.to_bits(),r.clone());
        Ok(r)
    }

    /// The point of the run `run` at `u` along it, at `t`: face `⌊u⌋` (`faces`, each a piece of
    /// the curve and its share of it) at the fraction `u − ⌊u⌋` across it.
    fn point(&self,run: &Run,faces: &[Face],u: f64,t: f64) -> Result<V,Unbuilt> {
        let r = self.read(t)?;
        let n = faces.len();
        let k = (u.floor().max(0.) as usize).min(n-1);
        let Face {piece,share,of} = faces[k];
        let f = (share as f64+(u-k as f64))/of as f64;
        let p = r.curves[run.curve][run.pieces[piece]];
        let at = |th: f64| self.ring.root(&r.instant.w,th.rem_euclid(TAU),p.label,false)
            .ok_or_else(|| Unbuilt::Curved {at:r.pose.point(self.ring.centre)});
        let th = p.from+(p.to-p.from)*f;
        let root = at(th)?;
        // (a narrow pair's bridge, between two touching pieces, is the sheet itself: the double curve
        // its outer leaves cross along passes through it, and its tail is cut off there, #880 T1d)
        let c = &r.curves[run.curve];
        let (n,i) = (c.len(),run.pieces[piece]);
        let narrow = p.side == Side::Fold && c[(i+n-1)%n].side == Side::Touching && c[(i+1)%n].side == Side::Touching;
        if p.side == Side::Touching || narrow { return Ok(r.pose.point(root.x)) }
        // the stretch between two touching pieces dipped into the tool along its normal (`dip`):
        // its ends where the touching piece before it ends and the one after it begins, round the
        // curve
        let mut j = i;
        while c[j].side != Side::Touching { j = (j+n-1)%n }
        let mut k = i;
        while c[k].side != Side::Touching { k = (k+1)%n }
        // its pieces from the one after `j` to the one before `k`, each an equal share
        let count = (k+n-j-1)%n;
        let place = (i+n-j-1)%n;
        let lambda = ((place as f64+f)/count as f64).clamp(0.,1.);
        Ok(r.pose.point(add(root.x,scale(root.n,-DEEP*self.ring.thickness*dip(lambda,t,self.roll)))))
    }

    /// Whether the run's sheet stays regular, read at its fitted points off the bridges (each
    /// regular by construction but at one point of each rim, `dip`): its area factor `Σ_u × Σ_t`
    /// keeping one sense between each two neighbours round the loop and along the roll, and nowhere
    /// below a billionth of its rates' (a fold the pieces were not cut at, `Folded`). A sheet squeezed
    /// thin where `s''` dips toward nought and rises again is regular, and kept (#880 T1d: the tumbling
    /// torus, 1.8e-4 of its rates just before a fold pair is born there).
    fn folds(&self,net: &Net,us: &[f64],ts: &[f64],bridged: &dyn Fn(f64) -> bool) -> Result<(),Unbuilt> {
        let read: Vec<Vec<Option<(V,V,f64)>>> = ts.iter().map(|&t| us.iter().map(|&u| (!bridged(u)).then(|| {
            let (x,du,dt) = net.d1(u,t);
            (x,cross(du,dt),norm(du)*norm(dt))
        })).collect()).collect();
        let flips = |p: &Option<(V,V,f64)>,q: &Option<(V,V,f64)>| match (p,q) {
            (Some((_,c,_)),Some((_,d,_))) => crate::util::space::dot(*c,*d) < 0.,
            _ => false,
        }.then(|| p.unwrap().0);
        for (j,row) in read.iter().enumerate() {
            for (i,p) in row.iter().enumerate() {
                if let Some((x,c,rates)) = p { if norm(*c) <= 1e-9*rates {
                    if std::env::var_os("SOLVENT_RING_DEBUG").is_some() { eprintln!("ring: no area at u {:.6} t {:.6} over {:?}: {:e} of {:e}",us[i],ts[j],self.roll,norm(*c),rates) }
                    return Err(Unbuilt::Folded {at:*x})
                } }
                let beside = [row.get(i+1),read.get(j+1).map(|r| &r[i])];
                if let Some(at) = beside.into_iter().flatten().find_map(|q| flips(p,q)) {
                    if std::env::var_os("SOLVENT_RING_DEBUG").is_some() { eprintln!("ring: folds at u {:.6} t {:.6} over {:?}",us[i],ts[j],self.roll) }
                    return Err(Unbuilt::Folded {at})
                }
            }
        }
        Ok(())
    }
}

/// **Places `x` (in order) with each interval `missed` halved**, and then each interval longer than
/// twice a neighbour's halved too, until none is: a quintic through places whose spacing jumps
/// rings between them.
fn halved(x: &[f64],missed: &[bool]) -> Vec<f64> {
    let mut out = vec![x[0]];
    for (i,w) in x.windows(2).enumerate() {
        if missed[i] { out.push(0.5*(w[0]+w[1])) }
        out.push(w[1]);
    }
    loop {
        let len: Vec<f64> = out.windows(2).map(|w| w[1]-w[0]).collect();
        let long: Vec<bool> = (0..len.len()).map(|i| {
            let near = [i.checked_sub(1),Some(i+1)].into_iter().flatten().filter_map(|j| len.get(j)).fold(f64::INFINITY,|m,&l| m.min(l));
            len[i] > 2.*near*(1.+1e-12)
        }).collect();
        if !long.contains(&true) { return out }
        let mut next = vec![out[0]];
        for (i,w) in out.windows(2).enumerate() {
            if long[i] { next.push(0.5*(w[0]+w[1])) }
            next.push(w[1]);
        }
        out = next;
    }
}

/// **The tubes a ring's contact curves sweep under `path` over `roll`, fitted within `tol`**, each
/// curve that touches anywhere entered whole: refused by name where an instant's contacts change in
/// number round the axis or in kind over the roll (`Curved`), where a tube folds off its bridges
/// (`Folded`), or where a fit will not close.
pub fn fitted(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],tol: f64) -> Result<Fitted,Unbuilt> {
    let scale = ring.scale_at(path,a,[a,b])?;
    fitted_over(ring,path,[a,b],[a,b],scale,tol)
}

/// `fitted` over the stretch `[a, b]` of the roll `whole`, `s''` read against `scale` (the whole
/// roll's, so a fold's band is the same in every stretch), the curves matched to those at the
/// stretch's middle.
pub fn fitted_over(ring: &Ring,path: &Trajectory,whole: [f64;2],[a,b]: [f64;2],scale: f64,tol: f64) -> Result<Fitted,Unbuilt> {
    let reader = Reader::within(ring,path,whole,[a,b],0.5*(a+b),scale)?;
    let runs = runs(&reader.first);
    if runs.is_empty() { return Err(Unbuilt::Failed("no contact of the ring touches its sweep's boundary".into())) }
    let mut nets = Vec::new();
    let mut on = Vec::new();
    let mut sides = Vec::new();
    let mut doubles = Vec::new();
    let mut seeds = Vec::new();
    for run in &runs {
        let per = LEAST_CLOSED.div_ceil(run.pieces.len()).max(PER);
        // each piece cut into `per` faces, but a bridge entered as one: no edge of the sheet runs
        // beside the point where its meeting with the tool at an end pose leaves its rim
        let faces: Vec<Face> = run.pieces.iter().enumerate().flat_map(|(i,&k)| {
            let of = if reader.first[run.curve][k].side == Side::Fold { 1 } else { per };
            (0..of).map(move |share| Face {piece:i,share,of})
        }).collect();
        let pieces = faces.len();
        // the rolls fitted at, and each face's fractions round the loop: each interval halved only
        // where the net misses at its middle (a quintic's miss falls some sixty-fold a halving, so
        // halving all of them at once overshoots the tolerance by as much wherever it was barely
        // missed), its neighbours kept within twice its length
        let mut ts: Vec<f64> = (0..=FIRST_ROWS).map(|j| a+(b-a)*j as f64/FIRST_ROWS as f64).collect();
        let mut fractions: Vec<Vec<f64>> = vec![(0..=FIRST_POINTS).map(|i| i as f64/FIRST_POINTS as f64).collect();pieces];
        let net = loop {
            crate::util::meter::go_on()?;
            let mid = |x: &[f64],i: usize| 0.5*(x[i]+x[i+1]);
            let half_ts: Vec<f64> = (0..ts.len()-1).map(|j| mid(&ts,j)).collect();
            // round the loop: each face's fitted fractions, and the middle of each interval between
            // them, its face's and its interval's
            let mut us: Vec<f64> = Vec::new();
            let mut middles: Vec<(f64,usize,usize)> = Vec::new();
            for (k,fr) in fractions.iter().enumerate() {
                for i in 0..fr.len()-1 {
                    us.push(k as f64+fr[i]);
                    middles.push((k as f64+mid(fr,i),k,i));
                }
            }
            us.push(pieces as f64);
            let at_us: Vec<f64> = us.iter().copied().chain(middles.iter().map(|m| m.0)).collect();
            let jobs: Vec<(f64,bool)> = ts.iter().map(|&t| (t,true)).chain(half_ts.iter().map(|&t| (t,false))).collect();
            let read_rows: Vec<Vec<V>> = crate::util::par::map(&jobs,|&(t,fitted)| -> Result<Vec<V>,Unbuilt> {
                if fitted { at_us.iter().map(|&u| reader.point(run,&faces,u,t)).collect() }
                else { us.iter().map(|&u| reader.point(run,&faces,u,t)).collect() }
            }).into_iter().collect::<Result<_,_>>()?;
            let (fitted,halfway) = read_rows.split_at(ts.len());
            let grid: Vec<Vec<V>> = fitted.iter().map(|r| r[..us.len()].to_vec()).collect();
            let counts: Vec<usize> = fractions.iter().map(|fr| fr.len()-1).collect();
            let net = interpolated_by(&grid,&us,&ts,&counts)?;
            // the most the net misses by at each interval's middle: round the loop over the fitted
            // rolls, and along the roll over the fitted points round it
            let mut err_u: Vec<Vec<f64>> = fractions.iter().map(|fr| vec![0.;fr.len()-1]).collect();
            for (&t,r) in ts.iter().zip(fitted) {
                for (&(u,k,i),&x) in middles.iter().zip(&r[us.len()..]) {
                    err_u[k][i] = err_u[k][i].max(norm(crate::util::space::sub(x,net.point(u,t))));
                }
            }
            let err_t: Vec<f64> = half_ts.iter().zip(halfway).map(|(&t,r)| missed(&net,us.iter().zip(r).map(|(&u,&x)| (t,u,x)))).collect();
            if std::env::var_os("SOLVENT_RING_DEBUG").is_some() {
                eprintln!("fit: curve {} over {:?}: {pieces} faces, {counts:?} points, {} rows: {:?} round, {:e} along",run.curve,reader.roll,ts.len()-1,
                    err_u.iter().map(|e| format!("{:.1e}",e.iter().fold(0f64,|m,&x| m.max(x)))).collect::<Vec<_>>(),err_t.iter().fold(0f64,|m,&x| m.max(x)));
            }
            let missed_u: Vec<Vec<bool>> = err_u.iter().map(|e| e.iter().map(|&x| x > tol).collect()).collect();
            let missed_t: Vec<bool> = err_t.iter().map(|&x| x > tol).collect();
            // a bridge's points, or on the join of one
            let bridge = |k: usize| reader.first[run.curve][run.pieces[faces[k%pieces].piece]].side == Side::Fold;
            let bridged = |u: f64| {
                // (the loop closing at `pieces`: the piece after its last join the first)
                let k = (u.floor().max(0.) as usize)%pieces;
                bridge(k) || (u-u.round()).abs() <= 1e-9 && bridge((u.round() as usize+pieces-1)%pieces)
            };
            if !missed_u.iter().flatten().any(|&m| m) && !missed_t.contains(&true) { reader.folds(&net,&us,&ts,&bridged)?; break net }
            let refined_u: Vec<Vec<f64>> = fractions.iter().zip(&missed_u).map(|(fr,m)| halved(fr,m)).collect();
            let refined_t = halved(&ts,&missed_t);
            if refined_u.iter().any(|fr| fr.len()-1 > MOST_POINTS) || refined_t.len()-1 > MOST_ROWS {
                return Err(Unbuilt::Failed(format!("a ring's contact's sheet fits within {tol} only to {:e} along the loop and \
                    {:e} along the roll",err_u.iter().flatten().fold(0f64,|m,&e| m.max(e)),err_t.iter().fold(0f64,|m,&e| m.max(e)))))
            }
            (fractions,ts) = (refined_u,refined_t);
        };
        // **where the outer leaves cross about each narrow pair's bridge** (#880 T1d): a bridge between
        // two touching pieces, at each fitted row the leaves' meeting `Σ(u₁, t) = Σ(u₂, t₂)` settled by
        // Newton's method on the net from beside the bridge's ends (the normal form puts it a fifth of
        // the bridge's half-width past them); a row where it settles on none is where the pair is
        // born, its bridge no wider than the rounding, both preimages the bridge's middle
        let kinds_of = |i: usize| reader.first[run.curve][run.pieces[i]].side;
        let n = run.pieces.len();
        let mut found = Vec::new();
        for (fk,f) in faces.iter().enumerate() {
            if kinds_of(f.piece) != Side::Fold || kinds_of((f.piece+n-1)%n) != Side::Touching || kinds_of((f.piece+1)%n) != Side::Touching { continue }
            let (k0,k1) = (fk as f64,fk as f64+1.);
            let mut last: Option<[f64;3]> = None;
            let mut seed: Option<[(f64,f64,f64);2]> = None;
            for &t in &ts {
                let r = reader.read(t)?;
                let c = &r.curves[run.curve];
                let (pl,pf,pr) = (&c[run.pieces[(f.piece+n-1)%n]],&c[run.pieces[f.piece]],&c[run.pieces[(f.piece+1)%n]]);
                let half = 0.5*(pf.to-pf.from);
                // (each neighbour's angle a unit of `u` spans: its stretch over its faces)
                let per = |i: usize| faces.iter().filter(|g| g.piece == i).count().max(1) as f64;
                let (dl,dr) = (0.22*half/((pl.to-pl.from)/per((f.piece+n-1)%n)).max(1e-300),0.22*half/((pr.to-pr.from)/per((f.piece+1)%n)).max(1e-300));
                let mut x = last.unwrap_or([k0-dl,k1+dr,t]);
                let mut settled = false;
                // (damped least squares, Levenberg–Marquardt, each unknown scaled by its column: near
                // where the pair is born the two leaves are all but tangent and the sheet's rates all
                // but parallel, and Newton's own steps flew off)
                // (each preimage within two of the bridge's half-widths past its end, where the
                // normal form puts it at √3 − 1 of one: past that a leaf meets another stretch of
                // the tube, which is the trim's to meet, not the pair's double curve)
                let lo = [(k0-per((f.piece+n-1)%n)).max(k0-dl*2./0.22),k1,a];
                let hi = [k0,(k1+per((f.piece+1)%n)).min(k1+dr*2./0.22),b];
                let residual = |x: [f64;3]| -> (V,[V;3]) {
                    let ((p1,du1,_),(p2,du2,dt2)) = (net.d1(x[0].rem_euclid(pieces as f64),t),net.d1(x[1].rem_euclid(pieces as f64),x[2]));
                    (crate::util::space::sub(p1,p2),[du1,du2.map(|v| -v),dt2.map(|v| -v)])
                };
                let mut lambda = 1e-3;
                let (mut f3,mut cols) = residual(x);
                for _ in 0..400 {
                    if norm(f3) <= 1e-3*tol { settled = true; break }
                    let w: [f64;3] = std::array::from_fn(|i| norm(cols[i]).max(1e-300));
                    let jtj: [[f64;3];3] = std::array::from_fn(|i| std::array::from_fn(|j| crate::util::space::dot(cols[i],cols[j])/(w[i]*w[j])));
                    let jtf: V = std::array::from_fn(|i| -crate::util::space::dot(cols[i],f3)/w[i]);
                    let mut stepped = false;
                    for _ in 0..30 {
                        let m: [V;3] = std::array::from_fn(|i| std::array::from_fn(|j| jtj[i][j]+if i == j { lambda } else { 0. }));
                        let d = crate::ssi::solve3(m,jtf);
                        if !d.iter().all(|v| v.is_finite()) { lambda *= 10.; continue }
                        let y: [f64;3] = std::array::from_fn(|i| (x[i]+d[i]/w[i]).clamp(lo[i],hi[i]));
                        let (g3,gcols) = residual(y);
                        if norm(g3) < norm(f3) { (x,f3,cols) = (y,g3,gcols); lambda = (lambda*0.3).max(1e-12); stepped = true; break }
                        lambda *= 10.;
                    }
                    if !stepped { break }
                }
                let apart = x[0] < k0 && x[1] > k1;
                if settled && apart {
                    // the two preimages as angles round the axis and the roots' places there
                    let angle = |u: f64,p: &Piece,face: usize| { let g = faces[face]; p.from+(p.to-p.from)*(g.share as f64+(u-face as f64))/g.of as f64 };
                    // (each wrapped round the loop first: the first preimage before the first face is
                    // on the last, not the bridge)
                    let (u0,u1) = (x[0].rem_euclid(pieces as f64),x[1].rem_euclid(pieces as f64));
                    let (fl,fr) = ((u0.floor() as usize).min(pieces-1),(u1.floor() as usize).min(pieces-1));
                    let r2 = reader.read(x[2])?;
                    let p2 = r2.curves[run.curve][run.pieces[faces[fr].piece]];
                    let (th1,th2) = (angle(u0,pl,fl),angle(u1,&p2,fr));
                    let s1 = self_root(ring,&r.instant.w,th1,pl.label).map(|q| q.sigma);
                    let s2 = self_root(ring,&r2.instant.w,th2,p2.label).map(|q| q.sigma);
                    if let (Some(s1),Some(s2)) = (s1,s2) { seed = Some([(th1,t,s1),(th2,x[2],s2)]); break }
                }
                if settled && apart { last = Some(x) }
            }
            if std::env::var_os("SOLVENT_RING_DEBUG").is_some() { eprintln!("ring: a narrow pair's leaves first cross at {seed:?}") }
            found.extend(seed);
        }
        seeds.extend(found);
        doubles.push(Vec::new());
        nets.push(net);
        on.push(faces.iter().map(|f| reader.first[run.curve][run.pieces[f.piece]].label.on).collect());
        sides.push(faces.iter().map(|f| reader.first[run.curve][run.pieces[f.piece]].side).collect());
    }
    Ok(Fitted {nets,on,sides,roll:[a,b],doubles,seeds})
}

/// Whether two instants' contacts are of one shape: as many curves, each of `b`'s turning onto one
/// of `a`'s kinds, each once.
fn same_shape(a: &[Vec<Piece>],b: &[Vec<Piece>]) -> bool {
    if a.len() != b.len() { return false }
    let mut taken = vec![false;a.len()];
    for c in b {
        let Some(i) = (0..a.len()).find(|&i| !taken[i] && !turns_onto(c,&kinds(&a[i])).is_empty()) else { return false };
        taken[i] = true;
    }
    true
}

/// How many rolls the instants a ring's contact changes shape at are sought between.
const SHAPE_LOOKS: usize = 4096;

/// **The scale and the instants over `[a, b]` where a ring's contact changes shape under `path`**
/// (#880 T1d): a fold pair born or vanishing at a point of a curve — where its least `s''` reaches
/// nought — the curve's kinds changing there. Read at `SHAPE_LOOKS` rolls, and between two that
/// differ halved to the rounding of the roll, as often as it changes there. `Curved` where an
/// instant's contacts change in number round the axis.
pub fn changes(ring: &Ring,path: &Trajectory,[a,b]: [f64;2]) -> Result<(f64,Vec<f64>),Unbuilt> {
    let scale = ring.scale_at(path,a,[a,b])?;
    let shape = |t: f64| -> Result<Vec<Vec<Piece>>,Unbuilt> {
        let (w,dw) = rates(path,t,[a,b])?;
        let pose = path.pose(t)?;
        Ok(ring.instant(w,dw,Some(scale)).map_err(|at| Unbuilt::Curved {at:pose.point(at)})?.curves)
    };
    let at = |k: usize| if k == SHAPE_LOOKS { b } else { a+(b-a)*k as f64/SHAPE_LOOKS as f64 };
    let mut cuts = Vec::new();
    let mut here = shape(a)?;
    let mut lo = a;
    for k in 1..=SHAPE_LOOKS {
        let hi = at(k);
        let there = shape(hi)?;
        let mut from = lo;
        while !same_shape(&here,&there) {
            let (mut x,mut y) = (from,hi);
            while y-x > 1e-12*(b-a) {
                let m = 0.5*(x+y);
                if same_shape(&here,&shape(m)?) { x = m } else { y = m }
            }
            cuts.push(0.5*(x+y));
            here = shape(y)?;
            from = y;
            if y >= hi { break }
        }
        here = there;
        lo = hi;
    }
    Ok((scale,cuts))
}

/// The ring's root at the angle `θ` with label `l` under twist `w` (`Ring::root`).
fn self_root(ring: &Ring,w: &Twist,theta: f64,l: Label) -> Option<Root> { ring.root(w,theta.rem_euclid(TAU),l,false) }

/// The sheet's point at the angle `θ` and roll `t`, the contact's root there nearest the place `σ`
/// round the meridian, and that root's place.
fn sheet_at(ring: &Ring,path: &Trajectory,theta: f64,t: f64,sigma: f64) -> Option<(V,f64)> {
    let w = Twist::body(path.at(t).ok()?);
    let nf = ring.elements.len() as f64;
    let apart = |x: f64| { let d = (x-sigma).rem_euclid(nf); d.min(nf-d) };
    let (_,r) = ring.roots(&w,theta.rem_euclid(TAU)).into_iter().min_by(|a,b| apart(a.1.sigma).total_cmp(&apart(b.1.sigma)))?;
    Some((path.pose(t).ok()?.point(r.x),r.sigma))
}

/// How far round the axis a double curve is stepped at once, as the angle its first preimage moves.
const DOUBLE_STEP: f64 = 2e-4;

/// **A ring's sheet's double curve about a fold pair, over `[a, b]`** (#880 T1d): where its outer
/// leaves cross, `Σ(θ₁, t₁) = Σ(θ₂, t₂)`, from `seed` — the first preimage's angle stepped, the other
/// three settled by Newton's method on the contact itself — toward the fold pair, until the two
/// preimages meet (the swallowtail point), and away, until either leaves the roll. The preimage in
/// order of angle: one leaf's points to the swallowtail point, the other's from it; each its angle,
/// roll and place round the meridian. In the tube's `(u, t)` it is one curve over `u` (the
/// swallowtail's normal form: a parabola through the swallowtail point, the folds' inside it), and
/// what lies above it is the swallowtail's tail, inside the sweep. After a NaN marker, its
/// swallowtail points: the pair's birth, and where the way away stops short of the roll with its
/// preimages all but met, the pair's end too — the curve then a loop, closed through it.
pub fn double_curve(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],seed: [(f64,f64,f64);2],tol: f64) -> Result<Vec<(f64,f64,f64)>,Unbuilt> {
    let failed = || Unbuilt::Failed("a ring's sheet's double curve about a fold pair not traced".into());
    // the three unknowns `t₁, θ₂, t₂` settled at `θ₁`, from `z`
    let settle = |th1: f64,z: [f64;3],s1: f64,s2: f64| -> Option<([f64;3],f64,f64)> {
        let g = |z: [f64;3]| -> Option<(V,f64,f64)> {
            let ((p,a1),(q,a2)) = (sheet_at(ring,path,th1,z[0],s1)?,sheet_at(ring,path,z[1],z[2],s2)?);
            Some((crate::util::space::sub(p,q),a1,a2))
        };
        let mut z = z;
        for _ in 0..40 {
            let (f,a1,a2) = g(z)?;
            if norm(f) <= 1e-3*tol { return Some((z,a1,a2)) }
            let h = [1e-7*(b-a),1e-7,1e-7*(b-a)];
            let cols: [V;3] = std::array::from_fn(|k| { let mut y = z; y[k] += h[k]; g(y).map_or([f64::NAN;3],|(fy,..)| scale(crate::util::space::sub(fy,f),1./h[k])) });
            let d = crate::ssi::solve3(std::array::from_fn(|i| [cols[0][i],cols[1][i],cols[2][i]]),scale(f,-1.));
            if !d.iter().all(|v| v.is_finite()) { return None }
            z = [z[0]+d[0],z[1]+d[1],z[2]+d[2]];
        }
        None
    };
    let [(th1,t1,s1),(th2,t2,s2)] = seed;
    // (the second angle beside the first, not across the wrap)
    let th2 = th1+(th2-th1+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI;
    let toward = (th2-th1).signum();
    // from the seed toward the pair, then away; each step's points
    let mut near: Vec<[(f64,f64,f64);2]> = Vec::new();
    let mut far: Vec<[(f64,f64,f64);2]> = Vec::new();
    // (whether the way away from the pair stopped short of the roll's ends: settling on no
    // meeting, where the preimages meet again — a pair born and later vanishing, its double curve
    // a loop through two swallowtail points, #880 T1d's thin torus)
    let mut short = false;
    for (dir,out) in [(toward,&mut near),(-toward,&mut far)] {
        let (mut th,mut z,mut sg1,mut sg2) = (th1,[t1,th2,t2],s1,s2);
        for _ in 0..200_000 {
            let next = th+dir*DOUBLE_STEP;
            let Some((y,a1,a2)) = settle(next,z,sg1,sg2) else {
                if std::env::var_os("SOLVENT_RING_DEBUG").is_some() {
                    eprintln!("ring: a double curve's way {} stops at θ₁ {th:.6} with (t₁, θ₂, t₂) {z:?}, σ {sg1:.4} {sg2:.4}",if dir == toward { "toward" } else { "away" });
                }
                short = dir != toward;
                break
            };
            let out_of = |y: [f64;3]| y[0] < a || y[0] > b || y[2] < a || y[2] > b;
            if out_of(y) {
                // (the step that leaves the roll halved to where a preimage reaches its end, and that
                // place kept: the curve ends on the stretch's rim, not short of it)
                let (mut lo,mut hi) = (th,next);
                let mut last = None;
                for _ in 0..60 {
                    let m = 0.5*(lo+hi);
                    match settle(m,z,sg1,sg2) { Some(r) if !out_of(r.0) => { last = Some((m,r)); lo = m } _ => hi = m }
                }
                if let Some((m,(y,a1,a2))) = last {
                    let edge = |x: f64| if (x-a).abs() < (x-b).abs() { a } else { b };
                    let y = if (y[0]-edge(y[0])).abs() < (y[2]-edge(y[2])).abs() { [edge(y[0]),y[1],y[2]] } else { [y[0],y[1],edge(y[2])] };
                    out.push([(m,y[0],a1),(y[1],y[2],a2)]);
                }
                break
            }
            // (the preimages met: the swallowtail point)
            if dir == toward && (y[1]-next)*toward <= 0. { break }
            (th,z,sg1,sg2) = (next,y,a1,a2);
            out.push([(th,y[0],a1),(y[1],y[2],a2)]);
        }
    }
    if near.is_empty() && far.is_empty() { return Err(failed()) }
    // in order of angle: the first preimage from far to near, then the second from near to far
    let first: Vec<(f64,f64,f64)> = far.iter().rev().map(|p| p[0]).chain(std::iter::once((th1,t1,s1))).chain(near.iter().map(|p| p[0])).collect();
    let second: Vec<(f64,f64,f64)> = near.iter().rev().map(|p| p[1]).chain(std::iter::once((th2,t2,s2))).chain(far.iter().map(|p| p[1])).collect();
    // (the second taken from where it passes the first's last angle: the step that found the
    // preimages met stood a hair past the swallowtail point)
    let last = first.last().map_or(f64::NAN,|p| p.0);
    let second: Vec<(f64,f64,f64)> = second.into_iter().skip_while(|p| (p.0-last)*toward <= 0.).collect();
    let apex = first.len();
    let mut out: Vec<(f64,f64,f64)> = first.into_iter().chain(second).collect();
    // (the swallowtail point: between the two leaves' preimages, put at the end of the list)
    let ap = if apex > 0 && apex < out.len() { let (p,q) = (out[apex-1],out[apex]); (0.5*(p.0+q.0),0.5*(p.1+q.1),p.2) } else { out[apex.min(out.len()-1)] };
    // (and where the way away stopped short with its preimages within a twentieth of a radian, the
    // second: the curve closed through it, from the second preimage's far end to the first's)
    let far_apex = far.last().filter(|l| short && (l[1].0-l[0].0).abs() <= 0.05)
        .map(|l| (0.5*(l[0].0+l[1].0),0.5*(l[0].1+l[1].1),l[0].2));
    if let Some(q) = far_apex { let first = out[0]; out.push(q); out.push(first) }
    if toward < 0. { out.reverse() }
    // the swallowtail points after a marker
    out.push((f64::NAN,f64::NAN,f64::NAN));
    out.push(ap);
    out.extend(far_apex);
    if std::env::var_os("SOLVENT_RING_DEBUG").is_some() {
        eprintln!("ring: a double curve of {} points from {:?} to {:?}",out.len(),out.first(),out.last());
    }
    Ok(out)
}

/// **The double curves cut into a stretch's fit** (`Fitted::doubles`): each curve's points within the
/// stretch on each tube, in that tube's `(u, t)` — those whose root is the tube's curve's at their
/// angle — with where it crosses the stretch's ends.
pub fn cut_along(ring: &Ring,path: &Trajectory,whole: [f64;2],fit: &mut Fitted,scale: f64,curves: &[Vec<(f64,f64,f64)>]) -> Result<(),Unbuilt> {
    let [a,b] = fit.roll;
    let reader = Reader::within(ring,path,whole,[a,b],0.5*(a+b),scale)?;
    let runs = runs(&reader.first);
    let nf = ring.elements.len() as f64;
    for (ri,run) in runs.iter().enumerate() {
        let per = LEAST_CLOSED.div_ceil(run.pieces.len()).max(PER);
        let faces: Vec<Face> = run.pieces.iter().enumerate().flat_map(|(i,&k)| {
            let of = if reader.first[run.curve][k].side == Side::Fold { 1 } else { per };
            (0..of).map(move |share| Face {piece:i,share,of})
        }).collect();
        // `(θ, t, σ)`'s `u` on this tube, where its root there is the tube's
        let u_of = |(th,t,sg): (f64,f64,f64)| -> Result<Option<f64>,Unbuilt> {
            let r = reader.read(t)?;
            let c = &r.curves[run.curve];
            for (fk,g) in faces.iter().enumerate() {
                let p = c[run.pieces[g.piece]];
                let f = ((th-p.from).rem_euclid(TAU))/(p.to-p.from);
                let lo = g.share as f64/g.of as f64;
                let hi = (g.share+1) as f64/g.of as f64;
                if !(f >= lo && f <= hi) { continue }
                let Some(root) = self_root(ring,&r.instant.w,th,p.label) else { continue };
                let d = (root.sigma-sg).rem_euclid(nf);
                // (the tube's own branch: another stands some half a piece of the meridian away)
                if d.min(nf-d) > 1e-2 { return Ok(None) }
                return Ok(Some(fk as f64+(f-lo)*g.of as f64))
            }
            Ok(None)
        };
        let mut out = Vec::new();
        for full in curves {
            // (the curve, and its swallowtail points after the marker)
            let mark = full.iter().position(|p| p.0.is_nan()).unwrap_or(full.len());
            let curve = &full[..mark];
            let mut line: Vec<[f64;2]> = Vec::new();
            for w in 0..curve.len() {
                let (th,t,sg) = curve[w];
                let inside = t >= a && t <= b;
                // (where it crosses an end of the stretch: between this point and the one before; going
                // out, the line ends there, and coming back in a new one begins)
                if w > 0 {
                    let (th0,t0,sg0) = curve[w-1];
                    for edge in [a,b] {
                        if (t0-edge)*(t-edge) < 0. {
                            let f = (edge-t0)/(t-t0);
                            if let Some(u) = u_of((th0+(th-th0)*f,edge,if f < 0.5 { sg0 } else { sg }))? { line.push([u,edge]) }
                            if !inside && line.len() >= 2 { out.push(std::mem::take(&mut line)) } else if !inside { line.clear() }
                        }
                    }
                }
                if inside { if let Some(u) = u_of((th,t,sg))? { line.push([u,t]) } }
            }
            if line.len() >= 2 { out.push(line) }
        }
        if std::env::var_os("SOLVENT_RING_DEBUG").is_some() {
            for l in &out { eprintln!("ring: cut over {:?}, tube {ri}: a line of {} from {:?} to {:?}",[a,b],l.len(),l[0],l[l.len()-1]) }
        }
        fit.doubles[ri] = out;
    }
    Ok(())
}

/// How many rolls a ring's admission reads the instant's contact at.
const ADMITTED_AT: usize = 65;

/// **Whether the ring road takes `tool` under `path` over `[a, b]`**, read before anything is
/// fitted: the tool a ring (`Ring::read`), and at `ADMITTED_AT` rolls the instant's contacts the
/// first's, curve by curve and piece by piece (`Curved` where not).
pub fn admits(tool: &Brep,path: &Trajectory,[a,b]: [f64;2]) -> Result<(),Unbuilt> {
    let ring = Ring::read(tool)?;
    let reader = Reader::new(&ring,path,[a,b])?;
    if runs(&reader.first).is_empty() { return Err(Unbuilt::Failed("no contact of the ring touches its sweep's boundary".into())) }
    let ts: Vec<f64> = (0..ADMITTED_AT).map(|k| a+(b-a)*k as f64/(ADMITTED_AT-1) as f64).collect();
    for r in crate::util::par::map(&ts,|&t| reader.read(t).map(|_| ())) { r? }
    Ok(())
}

/// **The ring at the pose the roll `t` gives it**, turned about its own axis — the same solid — so
/// its seam meridian stands in the widest gap among the angles at which its sheets touch it there
/// (`closed::seam_away`).
pub fn posed(tool: &Brep,ring: &Ring,path: &Trajectory,fit: &Fitted,t: f64) -> Result<Brep,Unbuilt> {
    let pose = path.pose(t)?;
    let touched: Vec<V> = fit.nets.iter().zip(&fit.on).flat_map(|(net,on)| {
        let n = 8*on.len();
        (0..n).map(move |i| net.point(on.len() as f64*i as f64/n as f64,t))
    }).collect();
    let moved = tool.moved(&crate::geom::Rigid {r:pose.r,t:pose.p});
    Ok(super::closed::seam_away(moved,pose.point(ring.centre),pose.vector(ring.frame.z),&touched))
}

/// **Each contact curve's sheet entered as a candidate of its own, where it stands outside the tool
/// at both end poses** (`trimmed`), with its meetings with the tool where it starts and stops given.
/// At an end pose a tube that folds runs on from its rim into the tool — a dipped stretch at once,
/// and anywhere the sweep comes back on the ring where it started — and what lies inside the tool
/// there is inside the sweep, and parts nothing of it. Entered from its rim, it overlaid itself
/// there: near a fold the contact slides along itself, and a fold piece's sheet just past its rim
/// lay over the rim, inside the tool — a meeting of the tube with itself no arrangement seeks, which
/// left the ring's face a slit along a touching piece's rim and the sweep's inside one cell with its
/// outside (the tumbling torus, at the fold corners where it starts). So each face is cut where it
/// crosses the tool at either end pose (`traces`, read in its own domain, each face on its own core)
/// and only its pieces outside both are kept; a tube touching all round crosses it nowhere, and is
/// its net cut round at each piece's ends. A touching piece's rim — the piece's iso-curve at that end,
/// on the tool's faces holding it — and the curves a face is cut along — each on the face it lies on
/// — all lie on the tool, and are each face's whole meeting with it, given.
pub fn entered(fit: &Fitted,ring: &Ring,ends: [&Brep;2],poses: [Pose;2],tol: f64) -> Result<(Vec<Brep>,Vec<Given>),Unbuilt> {
    entered_at(fit,ring,[Some((ends[0],poses[0])),Some((ends[1],poses[1]))],[true,true],tol)
}

/// `entered`, where an end of the fit's stretch of the roll may be no end of the roll (`rimmed`
/// false): a cut where the contact changes shape (`changes`, #880 T1d), the tube there running on into
/// the next stretch's along its rim, which lies on no tool. Such a stretch is still cut where it
/// crosses the tool at either end pose given (`at`), read plainly (no rim of it touches that tool): a
/// tube carried on past where the roll's end pose stands — the thin torus, its double curve ending
/// where one leaf's partner reaches the end of the roll, on that pose's tool, partway along another
/// stretch.
pub fn entered_at(fit: &Fitted,ring: &Ring,at: [Option<(&Brep,Pose)>;2],rimmed: [bool;2],tol: f64) -> Result<(Vec<Brep>,Vec<Given>),Unbuilt> {
    let mut sheets = Vec::new();
    let mut given = Vec::new();
    let n = ring.elements.len();
    // each end pose carrying a point in space back into the tool where it stands there
    let backs = at.map(|x| x.map(|(_,p)| p.inverse()));
    for (((net,on),sides),doubles) in fit.nets.iter().zip(&fit.on).zip(&fit.sides).zip(&fit.doubles) {
        let pieces = on.len();
        let [a,b] = fit.roll;
        let y_of = |t: f64| (t-a)/(b-a);
        // each double curve's preimages cut at the seams between faces: per face its chords
        let mut cuts_of: Vec<Vec<Vec<P2>>> = vec![Vec::new();pieces];
        for side in doubles {
            // (its `u` unwrapped along it, and each step cut at every seam it crosses)
            let np = pieces as f64;
            let mut un: Vec<[f64;2]> = Vec::new();
            for &[u,t] in side {
                let u = match un.last() { Some(&[v,_]) => v+(u-v+0.5*np).rem_euclid(np)-0.5*np,None => u };
                un.push([u,t]);
            }
            let face_of = |u: f64| (u.rem_euclid(np).floor() as usize).min(pieces-1);
            let mut run: Vec<P2> = vec![[un[0][0]-face_of(un[0][0]+1e-12) as f64-(un[0][0]/np).floor()*np,y_of(un[0][1])]];
            let mut face = face_of(un[0][0]+1e-12);
            for w in un.windows(2) {
                let ([u0,t0],[u1,t1]) = (w[0],w[1]);
                let (lo,hi) = (u0.min(u1),u0.max(u1));
                let mut seams: Vec<f64> = ((lo.floor() as i64+1)..=(hi.ceil() as i64-1)).map(|m| m as f64).filter(|&m| m > lo && m < hi).collect();
                if u1 < u0 { seams.reverse() }
                for m in seams {
                    let f = (m-u0)/(u1-u0);
                    let y = y_of(t0+(t1-t0)*f);
                    let up = u1 > u0;
                    run.push([if up { 1. } else { 0. },y]);
                    if run.len() >= 2 { cuts_of[face].push(std::mem::take(&mut run)) } else { run.clear() }
                    face = if up { (face+1)%pieces } else { (face+pieces-1)%pieces };
                    run.push([if up { 0. } else { 1. },y]);
                }
                let x = u1-(u1/np).floor()*np-face as f64;
                run.push([x.clamp(0.,1.),y_of(t1)]);
            }
            if run.len() >= 2 { cuts_of[face].push(run) }
        }
        // whether `(u, t)` is in a narrow pair's tail: above its double curve, a curve over `u`
        let tail = |u: f64,t: f64| doubles.iter().any(|line| {
            let mid = 0.5*(line[0][0]+line[line.len()-1][0]);
            let u = mid+(u-mid+0.5*pieces as f64).rem_euclid(pieces as f64)-0.5*pieces as f64;
            let i = line.partition_point(|p| p[0] < u);
            if i == 0 || i == line.len() { return false }
            let ([u0,t0],[u1,t1]) = (line[i-1],line[i]);
            let at = if u1 == u0 { t0 } else { t0+(t1-t0)*(u-u0)/(u1-u0) };
            t > at
        });
        let holding: Vec<Vec<usize>> = on.iter().map(|&o| match o {
            On::Face(e) => vec![ring.faces[e]],
            On::Edge(e) => vec![ring.faces[e],ring.faces[(e+1)%n]],
        }).collect();
        let faces: Vec<Trimmed> = crate::util::par::indices(pieces,|k| {
            // a touching piece's rim touches the tool, a dipped stretch's crosses it
            let order = if sides[k] == Side::Touching { 2 } else { 1 };
            let readings: [Option<Reading>;2] = std::array::from_fn(|e| backs[e].map(|back| Reading {net,ring,back,u:[k as f64,k as f64+1.],
                roll:fit.roll,end:if rimmed[e] { e as f64 } else { f64::NAN },order:if rimmed[e] { order } else { 0 }}));
            let mut chords: Vec<(usize,Vec<P2>)> = (0..2).flat_map(|e| readings[e].as_ref().map_or(Vec::new(),traces).into_iter().map(move |c| (e,c))).collect();
            chords.extend(cuts_of[k].iter().map(|c| (2,c.clone())));
            let tail: Box<dyn Fn(P2) -> bool+Sync+Send+'_> = Box::new(move |[x,y]: P2| tail(k as f64+x,a+(b-a)*y));
            Trimmed {readings,chords,tail}
        });
        let (sheet,cut) = trimmed(net,fit.roll,faces,tol)?;
        for (k,(surface,(rims,chords))) in cut.surfaces.iter().zip(cut.rims.iter().zip(&cut.chords)).enumerate() {
            for (e,(pose,_)) in at.iter().enumerate().filter_map(|(e,x)| x.map(|x| (e,x))) {
                let back = backs[e].unwrap();
                // the tool's face each curve it is cut along here lies on (a crossing passing from
                // one of the tool's faces to the next is not yet cut where it does: refused, not
                // given to one of them alone)
                let on: Vec<usize> = chords[e].iter().map(|(_,xs)| {
                    let mut on = xs.iter().map(|&x| ring.faces[ring.signed(back.point(x)).1]);
                    let first = on.next().unwrap_or(usize::MAX);
                    if on.all(|g| g == first) { Ok(first) } else { Err(Unbuilt::Failed("a ring's sheet crosses the tool where it starts or \
                        stops across two of its faces: not yet cut there".into())) }
                }).collect::<Result<_,_>>()?;
                for (f,face) in pose.faces.iter().enumerate() {
                    let mut curves: Vec<Curve> = Vec::new();
                    if rimmed[e] && holding[k].contains(&f) { curves.extend(rims[e].iter().cloned()) }
                    curves.extend(chords[e].iter().zip(&on).filter(|&(_,&g)| g == f).map(|((c,_),_)| c.clone()));
                    given.push(Given {surfaces:[face.surface.clone(),surface.clone()],curves});
                }
            }
        }
        sheets.push(sheet);
    }
    Ok((sheets,given))
}

/// How many points along each rim of a face its meeting with the tool is first bracketed at
/// (`traces`), crowded toward the ends of the roll; the step a trace of it takes across the face, as
/// a part of the face's domain each way; and the most steps it takes.
const MET_RIM: usize = 512;
const MET_STEP: f64 = 2e-3;
const MET_MOST: usize = 20_000;

/// **A tube's face read against the tool at one end pose, in its own domain**: the face of `net`
/// over `u` × `roll`, normalised (`x` across it, `y` along the roll), and the ring's signed distance
/// at its points carried `back` into it (`Ring::signed`), `F`. At the end of the roll `end` (0 or
/// 1, normalised) the face's rim lies on the tool — touching it to the `order` 2 where the face is a
/// touching piece's, the rim the contact where the tool stands, or crossing it, `order` 1, where it
/// is a dipped stretch's — so that zero is divided out: `G = (F(x, y) − F(x, end)) / (y −
/// end)^order` (the rim's own fitting error taken off), whose sign is the sheet's side of the tool
/// beside its rim, carried onto the rim itself from just off it. A face none of whose rims lies on
/// the tool (a stretch of the roll ending at no end pose) is read with `order` 0: `G = F`.
struct Reading<'a> { net: &'a Net,ring: &'a Ring,back: Pose,u: [f64;2],roll: [f64;2],end: f64,order: i32 }

impl Reading<'_> {
    /// The face's point at `(x, y)` and its parameters.
    fn at(&self,x: f64,y: f64) -> (V,[f64;2]) {
        let (u,t) = (self.u[0]+(self.u[1]-self.u[0])*x,self.roll[0]+(self.roll[1]-self.roll[0])*y);
        (self.net.point(u,t),[u,t])
    }
    fn f(&self,x: f64,y: f64) -> f64 { self.ring.signed(self.back.point(self.at(x,y).0)).0 }
    fn g(&self,x: f64,y: f64) -> f64 {
        // (no rim of the face on this tool: read plainly)
        if self.order == 0 { return self.f(x,y) }
        // how near the touching rim the division is not made but carried from just off it
        let near = 1e-4;
        let (e,order) = (self.end,self.order);
        let rim = self.f(x,e);
        let raw = |y: f64| (self.f(x,y)-rim)/(y-e).powi(order);
        let d = y-e;
        if d.abs() >= near { return raw(y) }
        let w = if e < 0.5 { 1. } else { -1. };
        let (g1,g2) = (raw(e+w*near),raw(e+2.*w*near));
        g1+(g1-g2)*(d-w*near)/(w*near)
    }
    /// Whether the face stands outside the tool at `(x, y)`: `F`'s sign, read off `G`'s (the touch
    /// at the rim divided out, so it holds beside the rim).
    fn outside(&self,[x,y]: P2) -> bool {
        let g = self.g(x,y);
        if self.order % 2 == 1 && y < self.end { g < 0. } else { g > 0. }
    }
}

/// The way into the face's normalised square from each of its sides, in the order `perimeter`
/// walks them: the rim at the start, the far seam, the rim at the stop, the near seam.
const INWARD: [P2;4] = [[0.,1.],[-1.,0.],[0.,-1.],[1.,0.]];

/// **Where a face crosses the tool at an end pose** (`Reading`): `G`'s zero set, whose curves reach
/// the rim at that end only at the face's corners — where a dipped stretch's rim meets a touching
/// piece's — and a trace ending beside one ends at it. Each place it crosses a rim of the face is
/// bracketed among `MET_RIM` points (crowded toward the ends of the roll, where a crossing near a
/// fold runs close beside the touch) and settled by bisection; from each, the zero set is traced
/// across the face in its normalised domain — a step along it, then Newton back onto it along its
/// gradient — until it leaves the face, where it is settled on the rim it crosses; each end met is
/// used once. A closed curve inside the face, crossing no rim, is not read (the judge sees what it
/// misses). Each curve's points in the normalised domain, from one rim to another.
fn traces(r: &Reading) -> Vec<Vec<P2>> {
    let g = |x: f64,y: f64| r.g(x,y);
    let ye = r.end;
    // the gradient in the normalised domain, by differences
    let grad = |x: f64,y: f64| -> [f64;2] {
        let h = 1e-6;
        [(g((x+h).min(1.),y)-g((x-h).max(0.),y))/((x+h).min(1.)-(x-h).max(0.)),
         (g(x,(y+h).min(1.))-g(x,(y-h).max(0.)))/((y+h).min(1.)-(y-h).max(0.))]
    };
    // a point settled on the zero set between `p` and `q`, whose values differ in sign (halved until
    // the middle is one of the two, a rounding apart)
    let settle = |p: [f64;2],q: [f64;2]| -> [f64;2] {
        let s0 = g(p[0],p[1]) > 0.;
        let (mut lo,mut hi) = (0_f64,1_f64);
        loop {
            let m = 0.5*(lo+hi);
            if m <= lo || m >= hi { break }
            if (g(p[0]+(q[0]-p[0])*m,p[1]+(q[1]-p[1])*m) > 0.) == s0 { lo = m } else { hi = m }
        }
        let m = 0.5*(lo+hi);
        [p[0]+(q[0]-p[0])*m,p[1]+(q[1]-p[1])*m]
    };
    // the rims, each a way round the domain's boundary: along the roll crowded at both ends
    let crowd = |k: usize| 0.5*(1.-(std::f64::consts::PI*k as f64/MET_RIM as f64).dcos());
    let rims: [Box<dyn Fn(f64) -> [f64;2]>;4] = [Box::new(|s| [s,0.]),Box::new(|s| [1.,s]),Box::new(|s| [s,1.]),Box::new(|s| [0.,s])];
    // each crossing of a rim, and the way into the face from that rim
    let mut ends: Vec<([f64;2],[f64;2])> = Vec::new();
    for (k,rim) in rims.iter().enumerate() {
        // the rim on the end the zero is divided out at holds none of it but at its corners
        if k%2 == 0 && (rim(0.)[1]-ye).abs() < 0.5 { continue }
        let ss: Vec<f64> = (0..=MET_RIM).map(|i| if k%2 == 1 { crowd(i) } else { i as f64/MET_RIM as f64 }).collect();
        let vs: Vec<f64> = ss.iter().map(|&s| { let p = rim(s); g(p[0],p[1]) }).collect();
        for i in 0..MET_RIM {
            if (vs[i] > 0.) != (vs[i+1] > 0.) { ends.push((settle(rim(ss[i]),rim(ss[i+1])),INWARD[k])) }
        }
    }
    let inside = |p: [f64;2]| p[0] >= 0. && p[0] <= 1. && p[1] >= 0. && p[1] <= 1.;
    let apart = |p: [f64;2],q: [f64;2]| (p[0]-q[0]).hypot(p[1]-q[1]);
    let debug = std::env::var_os("SOLVENT_RING_DEBUG").is_some();
    if debug { eprintln!("met: face {:?} end {}: rim crossings {:?}",r.u,r.end,ends.iter().map(|e| e.0).collect::<Vec<_>>()) }
    let mut used = vec![false;ends.len()];
    let mut out = Vec::new();
    for start in 0..ends.len() {
        if used[start] { continue }
        used[start] = true;
        let (p0,into) = ends[start];
        let mut pts = vec![p0];
        let tangent = |p: [f64;2]| { let d = grad(p[0],p[1]); let l = d[0].hypot(d[1]); [-d[1]/l,d[0]/l] };
        let t0 = tangent(p0);
        // the way along the zero set into the face
        let mut dir = if t0[0]*into[0]+t0[1]*into[1] >= 0. { t0 } else { [-t0[0],-t0[1]] };
        let mut p = p0;
        let mut closed_at = None;
        for _ in 0..MET_MOST {
            // back onto the zero set along the gradient: until on it, or the step is a rounding's
            // (near a rim `G` is a difference divided by a small power, and holds no finer)
            let mut q = [p[0]+dir[0]*MET_STEP,p[1]+dir[1]*MET_STEP];
            for _ in 0..8 {
                let v = g(q[0],q[1]);
                if v.abs() <= 1e-13 { break }
                let d = grad(q[0],q[1]);
                let n2 = d[0]*d[0]+d[1]*d[1];
                if !(n2 > 0.) { break }
                let step = [v*d[0]/n2,v*d[1]/n2];
                q = [q[0]-step[0],q[1]-step[1]];
                if step[0].hypot(step[1]) <= 1e-14 { break }
            }
            if !inside(q) {
                // it leaves the face: at a crossing of the rim already found beside it, used, or
                // where it crosses the rim, settled along it
                let c = [q[0].clamp(0.,1.),q[1].clamp(0.,1.)];
                // beside a corner of the face, at the corner: where a dipped stretch's rim, crossing
                // the tool, meets a touching piece's, touching it, the zero set runs into the
                // tube's own vertex
                let corner = [c[0].round(),c[1].round()];
                if apart(corner,c) <= 4.*MET_STEP && (corner[1]-ye).abs() < 0.5 {
                    pts.push(corner);
                    break
                }
                let near = (0..ends.len()).filter(|&k| !used[k] && apart(ends[k].0,c) <= 4.*MET_STEP)
                    .min_by(|&i,&j| apart(ends[i].0,c).total_cmp(&apart(ends[j].0,c)));
                match near {
                    Some(k) => { used[k] = true; pts.push(ends[k].0) }
                    None => {
                        // along the rim the step left by, either side of where it left
                        let along = if c[1] == 0. || c[1] == 1. { [1.,0.] } else { [0.,1.] };
                        let w = 4.*MET_STEP;
                        let (x,y) = ([(c[0]-along[0]*w).clamp(0.,1.),(c[1]-along[1]*w).clamp(0.,1.)],
                            [(c[0]+along[0]*w).clamp(0.,1.),(c[1]+along[1]*w).clamp(0.,1.)]);
                        pts.push(if (g(x[0],x[1]) > 0.) != (g(y[0],y[1]) > 0.) { settle(x,y) } else { c });
                    }
                }
                break
            }
            let next = tangent(q);
            dir = if next[0]*dir[0]+next[1]*dir[1] >= 0. { next } else { [-next[0],-next[1]] };
            pts.push(q);
            p = q;
            // back where it began: a loop through the rim at one point is not read
            if pts.len() > 4 && apart(p,p0) < 0.5*MET_STEP { closed_at = Some(()); break }
        }
        if debug { eprintln!("met:   traced {} points from {:?} to {:?}",pts.len(),pts[0],pts[pts.len()-1]) }
        if closed_at.is_some() || pts.len() < 2 { continue }
        out.push(pts);
    }
    out
}

/// **A trace of a meeting in space**: the face's points along it (`pts`, in its normalised domain;
/// on the face exactly and on the tool to rounding), interpolated — a cubic through every so many of
/// them, the coarsest within a quarter of `tol` of all — and those points in space.
fn chord_curve(r: &Reading,pts: &[P2],tol: f64) -> Result<(Curve,Vec<V>),Unbuilt> {
    curve_through(pts.iter().map(|&[x,y]| r.at(x,y).0).collect(),tol)
}

/// The cubic through points `xs` by chord length (`chord_curve`'s), and the points.
fn curve_through(xs: Vec<V>,tol: f64) -> Result<(Curve,Vec<V>),Unbuilt> {
    let mut params = vec![0.];
    for w in xs.windows(2) { params.push(params.last().unwrap()+norm(crate::util::space::sub(w[1],w[0])).max(1e-300)) }
    let length = *params.last().unwrap();
    let params: Vec<f64> = params.iter().map(|p| p/length.max(1e-300)).collect();
    let thinned = |stride: usize| -> Option<crate::nurbs::BSpline> {
        let mut keep: Vec<usize> = (0..xs.len()).step_by(stride).collect();
        if *keep.last().unwrap() != xs.len()-1 { keep.push(xs.len()-1) }
        if keep.len() < 2 { return None }
        let c = crate::nurbs::interpolate(&keep.iter().map(|&i| xs[i]).collect::<Vec<_>>(),&keep.iter().map(|&i| params[i]).collect::<Vec<_>>(),3)?;
        xs.iter().zip(&params).all(|(&x,&t)| norm(crate::util::space::sub(c.point(t),x)) <= 0.25*tol).then_some(c)
    };
    let curve = [64,32,16,8,4,2,1].into_iter().find_map(thinned).ok_or_else(|| Unbuilt::Failed("a ring's sheet's meeting with the \
        tool could not be interpolated".into()))?;
    Ok((Curve::BSpline(Arc::new(curve)),xs))
}

/// A tube's face read at both end poses (`Reading`) and the curves it crosses the tool along at
/// each (`traces`), each with the end pose it is read at.
struct Trimmed<'a> { readings: [Option<Reading<'a>>;2],chords: Vec<(usize,Vec<P2>)>,
    /// Whether a place of the face's square is in a narrow pair's tail (#880 T1d): inside the sweep,
    /// and dropped. A chord read at `2` is where the tail is cut off, along its double curve.
    tail: Box<dyn Fn(P2) -> bool+Sync+Send+'a> }

/// What a trimmed tube was cut along: each face's surface; its rim at each end where it is kept
/// (the touch there); and the curves it is cut along at each end, each with the points it was
/// traced through.
struct Cut { surfaces: Vec<crate::geom::Surface>,rims: Vec<[Option<Curve>;2]>,chords: Vec<[Vec<(Curve,Vec<V>)>;2]> }

/// One way round a kept piece of a face: along its square's side from one place round it to the
/// next (`perimeter`), or along a chord, forward or back.
#[derive(Clone,Copy,Debug)]
enum Walk { Side(f64,f64),Chord(usize,bool) }

/// How near a side of the face's normalised square a point is on it: a trace ends on its rims
/// exactly (`traces` settles it there).
const ON_SIDE: f64 = 1e-12;

/// A place on the face's normalised square, as a way round it counter-clockwise from `(0, 0)`:
/// along the rim at the start, up the far seam, back along the rim at the stop, down the near seam.
fn perimeter([x,y]: P2) -> f64 {
    if y <= ON_SIDE { x } else if x >= 1.-ON_SIDE { 1.+y } else if y >= 1.-ON_SIDE { 2.+(1.-x) } else { 3.+(1.-y) }
}

/// The point that far round the square.
fn round_square(s: f64) -> P2 {
    let s = s.rem_euclid(4.);
    if s <= 1. { [s,0.] } else if s <= 2. { [1.,s-1.] } else if s <= 3. { [3.-s,1.] } else { [0.,4.-s] }
}

/// **A square's pieces, parted by chords**: the face's normalised square cut by curves from one
/// place on its rim to another (crossing none of the others), each piece walked counter-clockwise —
/// along the rim to a chord's end, along it to its other end, on round from there — a disc's pieces
/// between chords that do not cross, each side and each way along a chord walked once.
fn pieces_of_square(chords: &[(usize,Vec<P2>)]) -> Result<Vec<Vec<Walk>>,Unbuilt> {
    // the places marked round it: the corners, and each chord's ends (an end at a corner the corner)
    let mut marks: Vec<(f64,Option<(usize,usize)>)> = Vec::new();
    for (i,(_,c)) in chords.iter().enumerate() {
        for (k,p) in [c[0],c[c.len()-1]].into_iter().enumerate() { marks.push((perimeter(p),Some((i,k)))) }
    }
    for corner in 0..4 { if !marks.iter().any(|m| (m.0-corner as f64).abs() < ON_SIDE) { marks.push((corner as f64,None)) } }
    marks.sort_by(|a,b| a.0.total_cmp(&b.0));
    if marks.windows(2).any(|w| w[1].0-w[0].0 < ON_SIDE) {
        return Err(Unbuilt::Failed("two of a ring's sheet's crossings of the tool end at one place on a face's rim".into()))
    }
    let n = marks.len();
    let place = |end: (usize,usize)| marks.iter().position(|m| m.1 == Some(end)).unwrap();
    let mut walked = vec![false;n];
    let mut out = Vec::new();
    for first in 0..n {
        if walked[first] { continue }
        let mut walk = Vec::new();
        let mut j = first;
        loop {
            if walked[j] {
                if j == first { break }
                return Err(Unbuilt::Failed("a ring's sheet's face is cut by crossings of the tool that cross".into()))
            }
            walked[j] = true;
            let next = (j+1)%n;
            walk.push(Walk::Side(marks[j].0,if next == 0 { 4. } else { marks[next].0 }));
            j = match marks[next].1 {
                // along the chord to its other end, then on round from there
                Some((i,k)) => { walk.push(Walk::Chord(i,k == 0)); place((i,1-k)) }
                None => next,
            };
        }
        out.push(walk);
    }
    Ok(out)
}

/// **A square's chords split where they cross or one ends on another** (#880 T1d: a double curve
/// ending on a tube's meeting with the tool at an end pose): each chord cut at every place another
/// crosses it, and at every end of another standing within `SNAP_ON` of it (that end put there),
/// each piece keeping its chord's kind.
fn split_chords(chords: &[(usize,Vec<P2>)]) -> Vec<(usize,Vec<P2>)> {
    const SNAP_ON: f64 = 1e-3;
    let on_rim = |p: P2| p[0] <= ON_SIDE || p[0] >= 1.-ON_SIDE || p[1] <= ON_SIDE || p[1] >= 1.-ON_SIDE;
    let mut lines: Vec<(usize,Vec<P2>)> = chords.to_vec();
    // each chord's cuts: (segment, fraction along it, place)
    let mut cuts: Vec<Vec<(usize,f64,P2)>> = vec![Vec::new();lines.len()];
    for i in 0..lines.len() {
        // an end of `i` inside the square standing on another chord: put there, and that chord cut
        let ends: Vec<usize> = [0,lines[i].1.len()-1].into_iter().filter(|&e| !on_rim(lines[i].1[e])).collect();
        for e in ends {
            let p = lines[i].1[e];
            let best = (0..lines.len()).filter(|&j| j != i).flat_map(|j| (0..lines[j].1.len()-1).map(move |k| (j,k))).map(|(j,k)| {
                let (a,b) = (lines[j].1[k],lines[j].1[k+1]);
                let d = [b[0]-a[0],b[1]-a[1]];
                let f = (((p[0]-a[0])*d[0]+(p[1]-a[1])*d[1])/(d[0]*d[0]+d[1]*d[1]).max(1e-300)).clamp(0.,1.);
                let q = [a[0]+d[0]*f,a[1]+d[1]*f];
                ((q[0]-p[0]).hypot(q[1]-p[1]),j,k,f,q)
            }).min_by(|x,y| x.0.total_cmp(&y.0));
            if let Some((dist,j,k,f,q)) = best { if dist <= SNAP_ON { lines[i].1[e] = q; cuts[j].push((k,f,q)) } }
        }
    }
    for i in 0..lines.len() { for j in i+1..lines.len() {
        for k in 0..lines[i].1.len()-1 { for l in 0..lines[j].1.len()-1 {
            let (a,b,c,d) = (lines[i].1[k],lines[i].1[k+1],lines[j].1[l],lines[j].1[l+1]);
            let (r,s2) = ([b[0]-a[0],b[1]-a[1]],[d[0]-c[0],d[1]-c[1]]);
            let rxs = r[0]*s2[1]-r[1]*s2[0];
            if !(rxs.abs() > 1e-300) { continue }
            let q = [c[0]-a[0],c[1]-a[1]];
            let (t,u) = ((q[0]*s2[1]-q[1]*s2[0])/rxs,(q[0]*r[1]-q[1]*r[0])/rxs);
            // (strictly inside both: an end on the other is cut above)
            if t > 1e-9 && t < 1.-1e-9 && u > 1e-9 && u < 1.-1e-9 {
                let x = [a[0]+r[0]*t,a[1]+r[1]*t];
                cuts[i].push((k,t,x));
                cuts[j].push((l,u,x));
            }
        } }
    } }
    let mut out = Vec::new();
    for (i,(e,pts)) in lines.into_iter().enumerate() {
        let mut c = cuts[i].clone();
        c.sort_by(|x,y| (x.0,x.1).partial_cmp(&(y.0,y.1)).unwrap());
        let mut piece = vec![pts[0]];
        let mut at = 0;
        for k in 0..pts.len()-1 {
            while at < c.len() && c[at].0 == k {
                let q = c[at].2;
                if (q[0]-piece[piece.len()-1][0]).hypot(q[1]-piece[piece.len()-1][1]) > 1e-12 { piece.push(q) }
                if piece.len() >= 2 { out.push((e,std::mem::take(&mut piece))) }
                piece = vec![q];
                at += 1;
            }
            piece.push(pts[k+1]);
        }
        if piece.len() >= 2 { out.push((e,piece)) }
    }
    out
}

/// **A square's pieces among chords that may cross or end on one another** (`split_chords` first):
/// the planar graph of the chords' pieces and the square's sides between their places, each piece
/// walked with it on the left — along a side counter-clockwise, along a chord either way — turning at
/// each place to the first way out clockwise from the way back.
fn pieces_among(chords: &[(usize,Vec<P2>)]) -> Result<Vec<Vec<Walk>>,Unbuilt> {
    let on_rim = |p: P2| p[0] <= ON_SIDE || p[0] >= 1.-ON_SIDE || p[1] <= ON_SIDE || p[1] >= 1.-ON_SIDE;
    // the places: each a point
    let mut nodes: Vec<P2> = Vec::new();
    let mut node = |p: P2| -> usize {
        if let Some(i) = nodes.iter().position(|q| (q[0]-p[0]).hypot(q[1]-p[1]) <= ON_SIDE) { return i }
        nodes.push(p);
        nodes.len()-1
    };
    // the ways out of each place: (angle, what, to): a chord's `Walk::Chord(i, forward)`, a side's
    let mut ends: Vec<[usize;2]> = Vec::new();
    for (_,c) in chords { ends.push([node(c[0]),node(c[c.len()-1])]) }
    for corner in [[0.,0.],[1.,0.],[1.,1.],[0.,1.]] { node(corner); }
    let rim: Vec<usize> = { let mut r: Vec<usize> = (0..nodes.len()).filter(|&i| on_rim(nodes[i])).collect();
        r.sort_by(|&x,&y| perimeter(nodes[x]).total_cmp(&perimeter(nodes[y]))); r };
    let mut out: Vec<Vec<(f64,Walk,usize)>> = vec![Vec::new();nodes.len()];
    let angle = |a: P2,b: P2| (b[1]-a[1]).atan2(b[0]-a[0]);
    for (i,(_,c)) in chords.iter().enumerate() {
        let [p,q] = ends[i];
        out[p].push((angle(c[0],c[1]),Walk::Chord(i,true),q));
        out[q].push((angle(c[c.len()-1],c[c.len()-2]),Walk::Chord(i,false),p));
    }
    for w in 0..rim.len() {
        let (p,q) = (rim[w],rim[(w+1)%rim.len()]);
        let (s0,s1) = (perimeter(nodes[p]),if w+1 == rim.len() { 4. } else { perimeter(nodes[q]) });
        // (the way out along the side, read a hair along it)
        let ahead = round_square(s0+1e-6);
        out[p].push((angle(nodes[p],ahead),Walk::Side(s0,s1),q));
    }
    // the way back into each place along each way, to turn from
    let back = |w: &Walk| -> Option<f64> {
        match *w {
            Walk::Side(_,s1) => { let at = round_square(s1); Some(angle(at,round_square(s1-1e-6))) }
            Walk::Chord(i,f) => { let c = &chords[i].1; Some(if f { angle(c[c.len()-1],c[c.len()-2]) } else { angle(c[0],c[1]) }) }
        }
    };
    let key = |w: &Walk| -> (u8,u64,u64) { match *w { Walk::Side(a,_) => (0,a.to_bits(),0),Walk::Chord(i,f) => (1,i as u64,f as u64) } };
    let mut walked: std::collections::BTreeSet<(u8,u64,u64)> = std::collections::BTreeSet::new();
    let mut pieces = Vec::new();
    for start in 0..nodes.len() {
        for &(_,first,to) in &out[start].clone() {
            if walked.contains(&key(&first)) { continue }
            let mut walk = Vec::new();
            let (mut w,mut at) = (first,to);
            loop {
                if !walked.insert(key(&w)) {
                    if key(&w) == key(&first) { break }
                    return Err(Unbuilt::Failed("a ring's sheet's face's pieces among its chords do not close".into()))
                }
                walk.push(w);
                if walk.len() > 4*(chords.len()+nodes.len())+8 { return Err(Unbuilt::Failed("a ring's sheet's face's piece runs on".into())) }
                // the first way out clockwise from the way back
                let into = back(&w).unwrap();
                let turn = |a: f64| (into-a).rem_euclid(std::f64::consts::TAU);
                let Some(&(_,next,to)) = out[at].iter().filter(|o| key(&o.1) != key(&match w { Walk::Chord(i,f) => Walk::Chord(i,!f),x => x }))
                    .min_by(|x,y| { let (a,b) = (turn(x.0),turn(y.0)); (if a <= 1e-12 { std::f64::consts::TAU } else { a }).total_cmp(&(if b <= 1e-12 { std::f64::consts::TAU } else { b })) })
                    else { return Err(Unbuilt::Failed("a ring's sheet's face's piece ends at a place with no way on".into())) };
                if key(&next) == key(&first) { break }
                (w,at) = (next,to);
            }
            pieces.push(walk);
        }
    }
    Ok(pieces)
}

/// **A tube's net cut to where its faces stand outside the tool at both end poses**: each face's
/// square cut by the curves it crosses the tool along at either end (`pieces_of_square`), and the
/// pieces kept that stand outside at both (`Reading::outside`, read just inside a side of each), as
/// faces of one sheet named `sheet` — along the face's rims, the net's iso-curves at the ends of the
/// roll; along its seams, those at the pieces' ends, each shared with the face beside it (the last
/// face's far seam the first's near one, a tube); along its chords, the crossing interpolated in
/// space (`chord_curve`). A crossing ending on a seam ends where the face beside it finds its own
/// (the two read the one curve of the net; within `SNAP` of one another they are one place).
fn trimmed(net: &Net,roll: [f64;2],mut faces: Vec<Trimmed>,tol: f64) -> Result<(Brep,Cut),Unbuilt> {
    use crate::topo::{Coedge,EdgeCurve,Face,Pcurve};
    const SNAP: f64 = 1e-7;
    let pieces = faces.len();
    let [a,b] = roll;
    let t_of = |y: f64| a+(b-a)*y;
    // the seam a place on face `k`'s square stands on, if any: its near one, or its far one (the
    // next face's near one)
    let seam_of = |k: usize,x: f64| if x <= ON_SIDE { Some(k) } else if x >= 1.-ON_SIDE { Some((k+1)%pieces) } else { None };
    // the seams' places, both faces' ends merged, and each crossing's end on a seam put there
    let mut seams: Vec<Vec<f64>> = vec![vec![0.,1.];pieces];
    // (the chords as each face's pieces are walked about: split where they cross or one ends on
    // another, #880 T1d)
    let splits: Vec<Vec<(usize,Vec<P2>)>> = faces.iter().map(|f| split_chords(&f.chords)).collect();
    for (k,face) in faces.iter().enumerate() {
        for (_,c) in face.chords.iter().chain(&splits[k]) { for p in [c[0],c[c.len()-1]] { if let Some(s) = seam_of(k,p[0]) { seams[s].push(p[1]) } } }
    }
    for s in &mut seams {
        s.sort_by(f64::total_cmp);
        s.dedup_by(|x,y| (*x-*y).abs() <= SNAP);
    }
    // (the nearest place within a side's reach: an end a hair from a corner is the corner's)
    let place = |s: usize,y: f64| -> Result<usize,Unbuilt> { seams[s].iter().enumerate().filter(|(_,&q)| (q-y).abs() <= ON_SIDE)
        .min_by(|x,z| (x.1-y).abs().total_cmp(&(z.1-y).abs())).map(|(i,_)| i)
        .ok_or_else(|| {
            if std::env::var_os("SOLVENT_RING_DEBUG").is_some() { eprintln!("ring: seam {s} has no place at {y}: {:?}",seams[s]) }
            Unbuilt::Failed("a ring's sheet's crossing of the tool ends on a seam where the face beside finds none".into())
        }) };
    for (k,face) in faces.iter_mut().enumerate() {
        for (_,c) in &mut face.chords {
            let last = c.len()-1;
            for i in [0,last] { if let Some(s) = seam_of(k,c[i][0]) { c[i] = [c[i][0].round(),seams[s][place(s,c[i][1])?]] } }
        }
    }
    let mut out = Brep::default();
    let frame = Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]);
    // the vertices: a seam's at its places, a rim's at a chord's end on it
    let mut at_seam: std::collections::BTreeMap<(usize,usize),u32> = std::collections::BTreeMap::new();
    let mut at_rim: Vec<(usize,usize,f64,u32)> = Vec::new();
    let mut inside: Vec<(usize,P2,u32)> = Vec::new();
    let mut vertex = |out: &mut Brep,k: usize,[x,y]: P2| -> Result<u32,Unbuilt> {
        Ok(match seam_of(k,x) {
            Some(s) => {
                let i = place(s,y)?;
                *at_seam.entry((s,i)).or_insert_with(|| out.vertex(net.point(s as f64,t_of(seams[s][i]))))
            }
            None if y > ON_SIDE && y < 1.-ON_SIDE => {
                // (a place inside the square, where chords meet)
                if let Some(&(_,_,v)) = inside.iter().find(|q: &&(usize,P2,u32)| q.0 == k && (q.1[0]-x).hypot(q.1[1]-y) <= ON_SIDE) { return Ok(v) }
                let v = out.vertex(net.point(k as f64+x,t_of(y)));
                inside.push((k,[x,y],v));
                v
            }
            None => {
                let end = usize::from(y > 0.5);
                if let Some(&(_,_,_,v)) = at_rim.iter().find(|r| r.0 == k && r.1 == end && (r.2-x).abs() <= ON_SIDE) { return Ok(v) }
                let v = out.vertex(net.point(k as f64+x,t_of(y)));
                at_rim.push((k,end,x,v));
                v
            }
        })
    };
    // each face's net, its near seam's iso-curve, and the seams' edges between places, made as they
    // are first walked
    let nets: Vec<Net> = (0..pieces).map(|k| net.segment([k as f64,k as f64+1.],roll)).collect();
    let seam_curve: Vec<Curve> = nets.iter().map(|n| Curve::BSpline(Arc::new(n.row(0)))).collect();
    let mut seam_edges: std::collections::BTreeMap<(usize,usize),u32> = std::collections::BTreeMap::new();
    let mut cut = Cut {surfaces:Vec::new(),rims:Vec::new(),chords:Vec::new()};
    for ((k,face),piece) in faces.iter().enumerate().zip(nets) {
        let rim: [Curve;2] = [Curve::BSpline(Arc::new(piece.column(0))),Curve::BSpline(Arc::new(piece.column(piece.poles[0].len()-1)))];
        let surface = crate::geom::Surface::BSpline(frame,Arc::new(piece));
        let (u0,u1) = (k as f64,k as f64+1.);
        // (its chords split where they cross or one ends on another: a double curve ending on the
        // tube's meeting with the tool at an end pose, #880 T1d)
        let split = split_chords(&face.chords);
        let simple = split.len() == face.chords.len() && split.iter().zip(&face.chords).all(|(x,y)| x.1.len() == y.1.len());
        let walks = if simple { pieces_of_square(&face.chords)? } else { pieces_among(&split)? };
        let chords: &[(usize,Vec<P2>)] = if simple { &face.chords } else { &split };
        // the chords' edges, made once, each walked by the two pieces either side
        let mut chord_edges: Vec<Option<u32>> = vec![None;chords.len()];
        // (a double curve's chord carries its own curve in the face's parameters, on its curve's
        // chord-length parameter: the face's surface still holds the fold just past it, where an
        // inversion onto it may land on the leaf beyond, #880 T1d)
        let mut chord_pcurves: Vec<Option<Pcurve>> = vec![None;chords.len()];
        let mut rim_kept = [false;2];
        let mut kept_chords: [Vec<(Curve,Vec<V>)>;2] = [Vec::new(),Vec::new()];
        for walk in walks {
            // kept where it stands outside the tool at both ends: read just inside its longest side
            let Some((s0,s1)) = walk.iter().filter_map(|w| match *w { Walk::Side(s0,s1) => Some((s0,s1)),_ => None })
                .max_by(|x,y| (x.1-x.0).total_cmp(&(y.1-y.0))) else { continue };
            let m = round_square(0.5*(s0+s1));
            let into = INWARD[(s0.floor() as usize).min(3)];
            // no farther in than half the way to the nearest chord
            let clear = chords.iter().flat_map(|(_,c)| c).map(|p| (p[0]-m[0]).hypot(p[1]-m[1])).fold(f64::INFINITY,f64::min);
            let depth = (0.5*clear).min(1e-3).min(0.25*(s1-s0));
            let probe = [m[0]+into[0]*depth,m[1]+into[1]*depth];
            if !face.readings.iter().flatten().all(|r| r.outside(probe)) || (face.tail)(probe) { continue }
            let mut lp: Vec<Coedge> = Vec::new();
            for w in walk {
                match w {
                    Walk::Side(s0,s1) => {
                        let (p,q) = (round_square(s0),round_square(s1));
                        let (vp,vq) = (vertex(&mut out,k,p)?,vertex(&mut out,k,q)?);
                        match s0.floor() as usize {
                            // along a rim: the iso-curve there, from one place to the other
                            side @ (0|2) => {
                                let e = side/2;
                                let (lo,hi) = if p[0] <= q[0] { ((p,vp),(q,vq)) } else { ((q,vq),(p,vp)) };
                                let edge = out.edge(EdgeCurve::Curve(rim[e].clone()),[u0+lo.0[0],u0+hi.0[0]],[lo.1,hi.1]);
                                let t = t_of(e as f64);
                                lp.push(Coedge {edge,reversed:p[0] > q[0],pcurve:Pcurve::Line {a:[u0+lo.0[0],t],b:[u0+hi.0[0],t]}});
                                rim_kept[e] = true;
                            }
                            // along a seam: its edge between the two places, shared with the face beside
                            side => {
                                let (s,u) = if side == 1 { ((k+1)%pieces,u1) } else { (k,u0) };
                                let (lo,hi) = if p[1] <= q[1] { ((p,vp),(q,vq)) } else { ((q,vq),(p,vp)) };
                                let edge = match seam_edges.entry((s,place(s,lo.0[1])?)) {
                                    std::collections::btree_map::Entry::Occupied(o) => *o.get(),
                                    std::collections::btree_map::Entry::Vacant(v) => *v.insert(out.edge(EdgeCurve::Curve(seam_curve[s].clone()),
                                        [t_of(lo.0[1]),t_of(hi.0[1])],[lo.1,hi.1])),
                                };
                                lp.push(Coedge {edge,reversed:p[1] > q[1],pcurve:Pcurve::Line {a:[u,t_of(lo.0[1])],b:[u,t_of(hi.0[1])]}});
                            }
                        }
                    }
                    Walk::Chord(i,forward) => {
                        let (e,pts) = &chords[i];
                        let edge = match chord_edges[i] {
                            Some(edge) => edge,
                            None => {
                                let (curve,xs) = match face.readings.get(*e).and_then(|r| r.as_ref()) {
                                    Some(r) => chord_curve(r,pts,tol)?,
                                    // (the double curve: through the net's points along it)
                                    None => {
                                        // (refined until the surface between each two points stands within a
                                        // tenth of the tolerance of the chord between them: beside a fold the
                                        // sheet bends sharply, and points 2e-4 radians apart sagged 3.6e-6)
                                        let at = |p: P2| net.point(k as f64+p[0],a+(b-a)*p[1]);
                                        let mut fine: Vec<P2> = vec![pts[0]];
                                        for w in pts.windows(2) {
                                            let mut todo = vec![(w[0],w[1],0)];
                                            while let Some((p,q,depth)) = todo.pop() {
                                                let m = [0.5*(p[0]+q[0]),0.5*(p[1]+q[1])];
                                                let sag = norm(crate::util::space::sub(at(m),scale(add(at(p),at(q)),0.5)));
                                                if sag > 0.1*tol && depth < 20 { todo.push((m,q,depth+1)); todo.push((p,m,depth+1)) } else { fine.push(q) }
                                            }
                                        }
                                        let pts = &fine;
                                        let xs: Vec<V> = pts.iter().map(|&p| at(p)).collect();
                                        let mut params = vec![0.];
                                        for w in xs.windows(2) { params.push(params.last().unwrap()+norm(crate::util::space::sub(w[1],w[0])).max(1e-300)) }
                                        let length = *params.last().unwrap();
                                        let params: Vec<f64> = params.iter().map(|p| p/length.max(1e-300)).collect();
                                        let uvs: Vec<V> = pts.iter().map(|&[x,y]| [u0+x,t_of(y),0.]).collect();
                                        // (both through every point at one parameter, a polyline each: they agree at
                                        // every point and part only by a chord's sag)
                                        let (Some(pc),Some(c3)) = (crate::nurbs::interpolate(&uvs,&params,1),crate::nurbs::interpolate(&xs,&params,1))
                                            else { return Err(Unbuilt::Failed("a ring's sheet's double curve could not be drawn".into())) };
                                        chord_pcurves[i] = Some(Pcurve::Curve(Arc::new(Curve::BSpline(Arc::new(pc)))));
                                        (Curve::BSpline(Arc::new(c3)),xs)
                                    }
                                };
                                let (v0,v1) = (vertex(&mut out,k,pts[0])?,vertex(&mut out,k,pts[pts.len()-1])?);
                                let edge = out.edge(EdgeCurve::Curve(curve.clone()),[0.,1.],[v0,v1]);
                                if *e < 2 { kept_chords[*e].push((curve,xs)) }
                                chord_edges[i] = Some(edge);
                                edge
                            }
                        };
                        let uv = |p: P2| [u0+p[0],t_of(p[1])];
                        let pcurve = chord_pcurves[i].clone().unwrap_or(Pcurve::Inverse {a:uv(pts[0]),b:uv(pts[pts.len()-1])});
                        lp.push(Coedge {edge,reversed:!forward,pcurve});
                    }
                }
            }
            out.faces.push(Face {surface:surface.clone(),reversed:false,loops:vec![lp],name:"sheet".into()});
        }
        cut.surfaces.push(surface);
        cut.rims.push(std::array::from_fn(|e| rim_kept[e].then(|| rim[e].clone())));
        cut.chords.push(kept_chords);
    }
    Ok((out.compacted(),cut))
}

impl Ring {
    /// **The ring's signed distance at the point `x`** (its own coordinates), and the piece of its
    /// meridian nearest it: its `(ρ, z)`'s distance from the loop, less than nought inside it — on
    /// the side of the nearest piece the solid is on (its left, the loop walked counter-clockwise),
    /// which near the loop is the side the point is on. A torus's is its tube's distance less its
    /// radius; each piece's the tool face it is (`faces`).
    pub fn signed(&self,x: V) -> (f64,usize) {
        let l = self.frame.local(x);
        let q = [l[0].dhypot(l[1]),l[2]];
        let mut best = (f64::INFINITY,0,false);
        for (k,e) in self.elements.iter().enumerate() {
            let (d,inside) = match *e {
                Element::Line {a,b} => {
                    let (ab,aq) = ([b[0]-a[0],b[1]-a[1]],[q[0]-a[0],q[1]-a[1]]);
                    let s = ((aq[0]*ab[0]+aq[1]*ab[1])/(ab[0]*ab[0]+ab[1]*ab[1]).max(1e-300)).clamp(0.,1.);
                    ((q[0]-a[0]-s*ab[0]).dhypot(q[1]-a[1]-s*ab[1]),ab[0]*aq[1]-ab[1]*aq[0] > 0.)
                }
                Element::Arc {c,r,from,sweep} => {
                    // how far round from `from` the way it sweeps, past its end to the nearer end
                    let w = ((q[1]-c[1]).datan2(q[0]-c[0])-from)*sweep.signum();
                    let w = w.rem_euclid(TAU);
                    let rho = (q[0]-c[0]).dhypot(q[1]-c[1]);
                    let d = if w <= sweep.abs() { (rho-r).abs() } else {
                        e.ends().iter().map(|p| (q[0]-p[0]).dhypot(q[1]-p[1])).fold(f64::INFINITY,f64::min)
                    };
                    (d,if sweep > 0. { rho < r } else { rho > r })
                }
            };
            if d < best.0 { best = (d,k,inside) }
        }
        (if best.2 { -best.0 } else { best.0 },best.1)
    }

    /// **Whether the ring holds the point `x`** (its own coordinates): its `(ρ, z)` within the
    /// meridian loop, read against the loop drawn finely (each piece in 512 chords) by its winding.
    pub fn holds(&self,x: V) -> bool {
        let l = self.frame.local(x);
        let q = [l[0].dhypot(l[1]),l[2]];
        let pts: Vec<P2> = self.elements.iter().flat_map(|e| (0..512).map(move |i| along(e,i as f64/512.))).collect();
        let mut inside = false;
        for i in 0..pts.len() {
            let (p,r) = (pts[i],pts[(i+1)%pts.len()]);
            if (p[1] > q[1]) != (r[1] > q[1]) && q[0] < p[0]+(r[0]-p[0])*(q[1]-p[1])/(r[1]-p[1]) { inside = !inside }
        }
        inside
    }
}

/// **The ring's contact at the roll `t` of `[a, b]`, as the road reads it**: each contact curve's
/// pieces, matched to those at `a`, sampled at `n` points each (at the middles of `n` equal shares
/// of its stretch), each point where the pose at `t` puts it with how the sheet stands there.
pub fn contacts(ring: &Ring,path: &Trajectory,roll: [f64;2],t: f64,n: usize) -> Result<Vec<Vec<(V,Side)>>,Unbuilt> {
    let reader = Reader::new(ring,path,roll)?;
    let r = reader.read(t)?;
    r.curves.iter().map(|c| c.iter().flat_map(|p| (0..n).map(move |k| (p,(k as f64+0.5)/n as f64))).map(|(p,f)| {
        let root = ring.root(&r.instant.w,(p.from+(p.to-p.from)*f).rem_euclid(TAU),p.label,false)
            .ok_or_else(|| Unbuilt::Curved {at:r.pose.point(ring.centre)})?;
        Ok((r.pose.point(root.x),p.side))
    }).collect()).collect()
}

/// `patches::interpolated` with each piece's own count of points (`counts`): the net through `grid`
/// (a row a roll `ts`, a point a fraction `us` round the loop), each row's pieces quintic through
/// their points, joined by a knot five times over; then each pole's column quintic along the roll.
fn interpolated_by(grid: &[Vec<V>],us: &[f64],ts: &[f64],counts: &[usize]) -> Result<Net,Unbuilt> {
    let failed = || Unbuilt::Failed("a ring's contact's sheet could not be interpolated".into());
    let starts: Vec<usize> = counts.iter().scan(0,|at,&n| { let s = *at; *at += n; Some(s) }).collect();
    let pieces = counts.len();
    let mut rows: Vec<(Vec<f64>,Vec<V>)> = Vec::with_capacity(grid.len());
    for r in grid {
        let (mut knots,mut poles): (Vec<f64>,Vec<V>) = (Vec::new(),Vec::new());
        for k in 0..pieces {
            let (i0,i1) = (starts[k],starts[k]+counts[k]);
            let s = crate::nurbs::interpolate(&r[i0..=i1],&us[i0..=i1],5).ok_or_else(failed)?;
            let d = s.degree;
            let (from,to) = (if k == 0 { 0 } else { d+1 },s.knots.len()-if k+1 == pieces { 0 } else { 1 });
            knots.extend_from_slice(&s.knots[from..to]);
            poles.extend_from_slice(&s.poles[if k == 0 { 0 } else { 1 }..]);
        }
        rows.push((knots,poles));
    }
    let uknots = rows[0].0.clone();
    let nu = rows[0].1.len();
    let cols: Vec<crate::nurbs::BSpline> = (0..nu).map(|i| crate::nurbs::interpolate(&rows.iter().map(|r| r.1[i]).collect::<Vec<_>>(),ts,5))
        .collect::<Option<_>>().ok_or_else(failed)?;
    let net = Net {du:5,dv:cols[0].degree,uknots,vknots:cols[0].knots.clone(),poles:cols.into_iter().map(|c| c.poles).collect(),weights:None};
    net.check().map_err(Unbuilt::Failed)?;
    Ok(net)
}
