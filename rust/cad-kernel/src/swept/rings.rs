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
//! yet a class the kernel's dispatch admits: `closed::ring` builds it alone, `closed::ring_cut_from`
//! cut from a blank (a ring pushed into a block at a lean, a torus turned across one), and `closed`'s
//! own build takes a ring there where K2 and the general road refuse it (#942 T1d-ring, `k2`). A
//! meridian touched whole over the roll — a ring carried in a plane its axis lies in — sweeps a sheet
//! no contact curve's tube holds: its touching half is fitted between the tubes' crossings on it
//! (`Strip`) and entered with them, but the corners at the end poses are not yet arranged, and it is
//! refused by name (`closed::ring_entered`). A ring of several
//! faces whose sheet crosses the tool at an end pose across two of them is refused by name: the
//! crossing is not yet cut where it passes from face to face.
#[allow(unused_imports)]
use crate::util::fmath::{Det,HALF_READ,THIRD_READ};
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

/// `SOLVENT_RING_DEBUG` is set: the ring road narrates its fits, its double curves and its cuts
/// (read once, as `crate::debug`).
pub(crate) fn ring_debug() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_RING_DEBUG").is_some())
}

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
    pub(crate) fn root(&self,w: &Twist,theta: f64,l: Label,strict: bool) -> Option<Root> {
        spent::ROOTS.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
        let (er,_,ez) = self.axes(theta);
        let n = self.elements.len();
        let lift3 = |m: P2| add(scale(er,m[0]),scale(ez,m[1]));
        match l.on {
            On::Face(k) => match self.elements[k] {
                Element::Arc {c,r,from,sweep} => {
                    let u = w.velocity(self.lift(theta,c));
                    let (mut a,mut b) = (dot(er,u),dot(ez,u));
                    // (a meridian the whole of whose circle the instant touches — the ring's top and
                    // bottom carried along a path in the plane square to them, its velocity along
                    // the path there, #880 T1d M2 — has `A` and `B` nought, and beside it both are
                    // all but nought, read off a velocity whole by its rounding: there they are read
                    // in closed form. With `e_r(θ) = cos θ e₁ + sin θ e₂`, `A = P · e_r(θ)` and
                    // `B = q + Q · e_r(θ)` (the arc's centre's velocity `U` less its turn about the
                    // axis: `P = U` across, `q = e_z · U`, `Q = ρ_c (e_z × ω)`); where both vanish at
                    // `θ*` each is `2 sin((θ − θ*)/2)` times `(−P₁ sin m + P₂ cos m)`, `m` the half sum,
                    // and the branch roots turn on that, smooth through `θ*`)
                    let (pp,q,qq,size) = self.terms(w,c);
                    // (read so where `A` and `B` are no more than their rounding's half the scale's
                    // digits, and at the nearer of the two angles `A` vanishes at where `B` does too,
                    // #942 F6)
                    if a.dhypot(b) <= HALF_READ*size {
                        use std::f64::consts::PI;
                        let base = (-pp[0]).datan2(pp[1]);
                        // (θ's way from an angle, wrapped into (−π, π])
                        let way = |st: f64| (theta-st+PI).rem_euclid(TAU)-PI;
                        let st = if way(base).abs() <= way(base+PI).abs() { base } else { base+PI };
                        let (ss,cs) = st.dsin_cos();
                        if (q+qq[0]*cs+qq[1]*ss).abs() <= HALF_READ*size {
                            let d = way(st);
                            let (sm,cm) = (theta-0.5*d).dsin_cos();
                            let half = if d < 0. { -1. } else { 1. };
                            (a,b) = (half*(-pp[0]*sm+pp[1]*cm),half*(-qq[0]*sm+qq[1]*cm));
                        }
                    }
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
    pub(crate) fn roots(&self,w: &Twist,theta: f64) -> Vec<(Label,Root)> {
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

    /// **The contact in the half-plane at `θ` nearest the place `σ` round the meridian** (`roots`'),
    /// its places round the meridian read a whole turn of its pieces apart as one.
    pub(crate) fn root_near(&self,w: &Twist,theta: f64,sigma: f64) -> Option<(Label,Root)> {
        let nf = self.elements.len() as f64;
        let apart = |x: f64| { let d = (x-sigma).rem_euclid(nf); d.min(nf-d) };
        self.roots(w,theta).into_iter().min_by(|a,b| apart(a.1.sigma).total_cmp(&apart(b.1.sigma)))
    }

    /// **How the tool's distance from the point it touches at `r` bends over the roll there**,
    /// `s''`: positive where the tool holds the point at no instant beside (the sheet touching),
    /// negative where it holds it at every one (the sheet's far side of a fold, buried). A fan's
    /// is unbounded: an edge's normals turn through their fan in no length.
    pub(crate) fn bend(&self,w: &Twist,dw: &Twist,theta: f64,r: &Root) -> f64 {
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
pub(crate) struct Root { pub(crate) x: V,pub(crate) n: V,pub(crate) sigma: f64,bend: Bend }

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
pub(crate) fn rates(path: &Trajectory,t: f64,[a,b]: [f64;2]) -> Result<(Twist,Twist),Unbuilt> {
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

/// **What the ring road spends its time on** (#942 F4): every instant's contact read
/// (`Ring::instant`, a root on each of `AROUND` meridians and the joins between them settled), the
/// time those reads took, and every root read alone (`Ring::root`) — counted for the whole process,
/// read before and after a step and printed where `SOLVENT_BREP_TIME` is set (`crate::timing`).
pub(crate) mod spent {
    use std::sync::atomic::{AtomicU64,Ordering::Relaxed};
    pub(crate) static INSTANTS: AtomicU64 = AtomicU64::new(0);
    pub(crate) static INSTANT_NANOS: AtomicU64 = AtomicU64::new(0);
    pub(crate) static ROOTS: AtomicU64 = AtomicU64::new(0);
    /// The counts so far: instants read, their nanoseconds, roots read.
    pub(crate) fn now() -> [u64;3] { [INSTANTS.load(Relaxed),INSTANT_NANOS.load(Relaxed),ROOTS.load(Relaxed)] }
    /// What was spent since `before`, said in words.
    pub(crate) fn since(before: [u64;3]) -> String {
        let n = now();
        format!("{} instants in {:.2} s (of the cores' time), {} roots alone",n[0]-before[0],(n[1]-before[1]) as f64*1e-9,n[2]-before[2])
    }
}

impl Ring {
    /// **The contact at the instant `w` (its rate `dw`)**: each closed curve's pieces, in order
    /// along it, each join settled in `θ` to rounding. `Err` with a point of the tool where the
    /// number of contacts changes round the axis.
    fn instant(&self,w: Twist,dw: Twist,scale: Option<f64>) -> Result<Instant,V> { self.instant_born(w,dw,scale,None) }

    /// `instant`, where `unborn` names an angle round the axis with each curve's narrow pair nearest
    /// it not put in (`Reader::unborn`, the swallowtail point of a pair born since): read so, the pair
    /// is the touching pieces either side of it, as a dip of `s''` that folds nowhere is.
    fn instant_born(&self,w: Twist,dw: Twist,scale: Option<f64>,unborn: Option<f64>) -> Result<Instant,V> {
        let clock = crate::util::clock::Instant::now();
        let out = self.instant_read(w,dw,scale,unborn);
        spent::INSTANTS.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
        spent::INSTANT_NANOS.fetch_add(clock.elapsed().as_nanos() as u64,std::sync::atomic::Ordering::Relaxed);
        out
    }

    /// `instant_born`, uncounted.
    fn instant_read(&self,w: Twist,dw: Twist,scale: Option<f64>,unborn: Option<f64>) -> Result<Instant,V> {
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
                                // (clearly below nought, by more than its rounding leaves unread of the
                                // scale: so the instants a hair either side of where a pair is born
                                // read alike, #942 F6)
                                let mid = 0.5*(lo+hi);
                                let least = best.min(value(mid));
                                if least < -HALF_READ*scale_bend { narrow.push(([first,last],near.clone(),sign,mid,least)) }
                                least < -HALF_READ*scale_bend
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
            // (the pair nearest the angle `unborn` names read as not yet born)
            if let Some(th) = unborn {
                let apart = |x: f64| { let d = (x-th).rem_euclid(TAU); d.min(TAU-d) };
                if let Some(k) = (0..narrow.len()).min_by(|&i,&j| apart(narrow[i].3).total_cmp(&apart(narrow[j].3))) { narrow.remove(k); }
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
            let mut bridges: Vec<[f64;3]> = Vec::new();
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
                bridges.push([wrap(l),wrap(r),wrap(*mid)]);
            }
            joins.sort_by(|x,y| x.0.total_cmp(&y.0));
            // (a join inside a bridge — the branch's label turning over on a meridian the instant
            // touches whole, where one of the pair's folds stands, #880 T1d M2 — is the bridge's: its
            // piece a fold, on its own label)
            let within = |x: f64,l: f64,r: f64| if l <= r { x > l && x < r } else { x > l || x < r };
            for j in &mut joins {
                let inside = bridges.iter().any(|&[l,r,_]| within(j.0,l,r));
                if inside { j.1.1 = Side::Fold }
            }
            // **the side of such a bridge the meridian bounds clear of the sliver is touching**
            // (#942 T1d-ring): where an instant touches a meridian whole, the contact curve crosses
            // it, and `s''` changes sign there without the sheet folding — on one side touching up
            // to the meridian itself, on the other the sliver (its least) out to the fold beyond. The
            // touching side is entered as the sheet, its rim on the meridian the crossing, which the
            // meridian's own sheet holds exactly; the sliver's side alone is dipped
            for &[l,r,mid] in &bridges {
                let Some(at) = joins.iter().position(|j| within(j.0,l,r)) else { continue };
                if joins.iter().any(|j| j.0 == l && j.1.0 == joins[at].1.0) { continue }
                let past = within(mid,joins[at].0,r);
                let touching = if past { joins.iter().position(|j| j.0 == l) } else { Some(at) };
                if let Some(k) = touching { joins[k].1.1 = Side::Touching }
            }
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

/// **Whether the piece `i` of the curve `c` is a narrow pair's bridge** (#880 T1d): a fold piece
/// between two touching pieces on its own branch. A bridge's piece beside a meridian the instant
/// touches whole (#942 T1d-ring), the branch's label turning over there, is the dipped stretch
/// across the sliver the meridian bounds, not the sheet itself.
fn narrow(c: &[Piece],i: usize) -> bool {
    let n = c.len();
    let (before,after) = (c[(i+n-1)%n],c[(i+1)%n]);
    c[i].side == Side::Fold && before.side == Side::Touching && after.side == Side::Touching && before.label == c[i].label && after.label == c[i].label
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
    /// curve's preimage on the leaf before the bridge and on the one after, each a row's `(u, t)`,
    /// its `u` continuous along the line — unwrapped across the seam where the tube's faces close
    /// round, read modulo their number (#942 F6).
    pub doubles: Vec<Vec<Vec<[f64;2]>>>,
    /// Per net, each double curve crossing it whole, over the whole roll, as its points' angles
    /// round the axis and rolls: it is traced a graph over its first preimage's angle, and its tail
    /// is above it there (#942 T1d-tori) — not along `u`, which the faces' seams shear as the pieces
    /// they share move, nor over the stretch's part alone, past whose ends it runs on.
    pub tails: Vec<Vec<Vec<[f64;2]>>>,
    /// The motion the stretch is fitted under: a place on a face read as an angle round the tool's
    /// axis at its roll.
    pub path: Trajectory,
    /// Where each narrow pair's outer leaves are first found crossing in this stretch: the two
    /// preimages as angles round the axis, rolls and places round the meridian (`double_curve`).
    pub seeds: Vec<[(f64,f64,f64);2]>,
    /// The swallowtail points of each narrow pair's folds in this stretch (#942 T1d-tori,
    /// `swallowtails`): each double curve built outward from its own (`double_curve_from_apex`).
    pub apexes: Vec<Apex>,
    /// The swallowtail points of the narrow pairs whose leaves the rows find crossing in this
    /// stretch (#942 T1d-tori): the double curve traced from the rows' seed toward the pair is
    /// joined to its own (`double_curve`).
    pub ends: Vec<Apex>,
    /// Per net, where it crosses a meridian the instants touch whole (#942 T1d-ring).
    pub crossings: Vec<Vec<Crossing>>,
    /// Each meridian touched whole over the stretch, its touching half's sheet (`Strip`).
    pub strips: Vec<Strip>,
    /// Where the stretch was fitted with a fold pair born within it read as not yet born
    /// (`fitted_unborn`): the end of the stretch its pieces are matched to, which has no such pair,
    /// and the pair's swallowtail point's angle round the axis.
    pub unborn: Option<(f64,f64)>,
    /// Each bridge's end that gives way to a double curve over the stretch (`give_way`): the face
    /// ends on the curve's preimage itself, and the curve is laid there as no chord.
    pub ways: Vec<Way> }

/// **A bridge's end given way to a double curve** (#942 T1d-tori, `give_way`): the bridge `piece` of
/// the instant's curve `curve` (its index among the first instant's), its end `side` (0 its start
/// round the curve, 1 its end), and the branch of the double curve that end is put on — its
/// preimage's points `(θ, t, σ)` in order of roll, and the second derivatives of its angle over the
/// roll at them (`bends`, the natural cubic spline's) — and which points of which traced curve the
/// branch is (`of`: the curve's index and the range of its points), laid as no chord there.
#[derive(Clone,Debug)]
pub struct Way { pub curve: usize,pub piece: usize,pub side: usize,pub branch: Vec<(f64,f64,f64)>,pub bends: Vec<f64>,pub of: (usize,[usize;2]) }

impl Way {
    /// The way `curve`'s `piece`'s end `side` gives to `branch` (in order of roll), its spline read.
    fn new(curve: usize,piece: usize,side: usize,branch: Vec<(f64,f64,f64)>,of: (usize,[usize;2])) -> Way {
        // (the spline's second derivatives, run out parabolically — each end's that of the point
        // beside it, not nought: the thin torus's branch ends at its swallowtail point, where the
        // angle's second derivative is 130 — by a tridiagonal solve, Thomas's)
        let n = branch.len();
        let (x,y): (Vec<f64>,Vec<f64>) = branch.iter().map(|p| (p.1,p.0)).unzip();
        let mut bends = vec![0.;n];
        if n > 2 {
            let (mut c,mut d) = (vec![0.;n],vec![0.;n]);
            for i in 1..n-1 {
                let (h0,h1) = (x[i]-x[i-1],x[i+1]-x[i]);
                let r = 6.*((y[i+1]-y[i])/h1-(y[i]-y[i-1])/h0);
                let (lo,hi) = (if i == 1 { h0 } else { 0. },if i+2 == n { h1 } else { 0. });
                let m = 2.*(h0+h1)+lo+hi-if i == 1 { 0. } else { h0*c[i-1] };
                c[i] = if i+2 == n { 0. } else { h1/m };
                d[i] = (r-if i == 1 { 0. } else { h0*d[i-1] })/m;
            }
            for i in (1..n-1).rev() { bends[i] = d[i]-c[i]*bends[i+1] }
            (bends[0],bends[n-1]) = (bends[1],bends[n-2]);
        }
        Way {curve,piece,side,branch,bends,of}
    }

    /// **The branch's preimage at the roll `t`**, as an angle round the axis: the cubic spline of
    /// its angle over the roll through its traced points. Not settled row by row: beside
    /// a swallowtail point the two leaves cross all but tangentially and the row's pair is not well
    /// posed (the thin torus's, 6e-4 of the roll above its point, settled to two families 1.5e-6 of
    /// an angle apart, and the end read so zig-zagged by 2e-6 in space, which no fit closes), and
    /// read straight between the traced points, 5e-4 of the roll apart there with the angle's second
    /// derivative 130, it misses by 4e-6. `None` past the branch's rolls.
    pub fn at(&self,t: f64) -> Option<f64> {
        let br = &self.branch;
        let k = (0..br.len().saturating_sub(1)).find(|&k| (br[k].1-t)*(br[k+1].1-t) <= 0.)?;
        let (x0,x1,y0,y1,m0,m1) = (br[k].1,br[k+1].1,br[k].0,br[k+1].0,self.bends[k],self.bends[k+1]);
        let h = x1-x0;
        if !(h > 0.) { return Some(y0) }
        let (a,b) = ((x1-t)/h,(t-x0)/h);
        Some(a*y0+b*y1+((a*a*a-a)*m0+(b*b*b-b)*m1)*h*h/6.)
    }
}

/// **A swallowtail point of a ring's sheet** (#942 T1d-tori): where a fold's way along the tube is
/// the way the sheet does not move — the kernel of its rates `Σ_θ, Σ_t` tangent to the fold — and its
/// double curve begins. Its angle round the axis, roll and place round the meridian, and the kernel's
/// way there as the rate of the roll by the angle, `dt/dθ`.
#[derive(Clone,Copy,Debug)]
pub struct Apex { pub at: (f64,f64,f64),pub way: f64 }

/// **A meridian's touching half swept** (#942 T1d-ring): where the instants touch a meridian whole,
/// its arc sweeps a sheet of its own — the canal of the ball about the arc's centre — of which the
/// half where `s''` stands above nought bounds the sweep, between the two contact curves crossing
/// the meridian. Its net over (the fraction along that half from one crossing to the other, the
/// roll), the arc it is swept from, and the two crossings bounding it, each a tube (its net's
/// index) and where on it (`Crossing`), the first at the net's `u = 0`.
#[derive(Clone,Debug)]
pub struct Strip { pub net: Net,pub arc: usize,pub sides: [(usize,Crossing);2] }

/// **Where a tube crosses a meridian the instants touch whole** (#942 T1d-ring): the seam (a face
/// boundary, its `u`) on the meridian, the touching piece on one side of it and the dipped stretch
/// across the sliver on the other, that stretch's far seam, and the meridian's angle at the first
/// instant read.
#[derive(Clone,Copy,Debug)]
pub struct Crossing { pub seam: usize,pub far: usize,pub theta: f64 }

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
    /// Where a fold pair the first instant does not have is read as not yet born (`fitted_unborn`):
    /// its swallowtail point's angle round the axis, an instant whose shape is not the first's read
    /// again with each curve's pair nearest it not put in.
    unborn: Option<f64>,
    /// Where reading unborn, each roll's narrow pairs as the instant read whole has them: their
    /// labels and angles (`newborn`).
    born: std::sync::Mutex<std::collections::BTreeMap<u64,Vec<(Label,[f64;2])>>>,
    /// The bridges' ends given way to double curves (`give_way`): each read on its branch's preimage
    /// at every roll the branch spans.
    ways: Vec<Way>,
    cache: std::sync::Mutex<std::collections::BTreeMap<u64,Read>>,
}

impl<'a> Reader<'a> {
    fn new(ring: &'a Ring,path: &'a Trajectory,roll: [f64;2]) -> Result<Reader<'a>,Unbuilt> {
        let scale = ring.scale_at(path,roll[0],roll)?;
        Reader::within(ring,path,roll,roll,roll[0],scale)
    }

    /// The reader a stretch is fitted by (`Fitted::unborn`): its curves matched to those at its
    /// middle, or at the end `unborn` names with the pair born since read as not yet born.
    fn of(ring: &'a Ring,path: &'a Trajectory,whole: [f64;2],[a,b]: [f64;2],scale: f64,unborn: Option<(f64,f64)>) -> Result<Reader<'a>,Unbuilt> {
        let mut r = Reader::within(ring,path,whole,[a,b],unborn.map_or(0.5*(a+b),|u| u.0),scale)?;
        r.unborn = unborn.map(|u| u.1);
        Ok(r)
    }

    /// The reader over `roll`, `s''` read against `scale`, its curves matched to those at `first`.
    fn within(ring: &'a Ring,path: &'a Trajectory,whole: [f64;2],roll: [f64;2],first: f64,scale: f64) -> Result<Reader<'a>,Unbuilt> {
        let (w,dw) = rates(path,first,whole)?;
        let pose = path.pose(first)?;
        let i = ring.instant(w,dw,Some(scale)).map_err(|at| Unbuilt::Curved {at:pose.point(at)})?;
        Ok(Reader {ring,path,roll,first:i.curves,whole,scale,unborn:None,born:Default::default(),ways:Vec::new(),
            cache:Default::default()})
    }

    /// The instant at `t`, its curves matched to the first instant's: each first curve by the one
    /// whose kinds turn onto its own, the nearest where several do; refused by name (`Curved`)
    /// where none does.
    fn read(&self,t: f64) -> Result<Read,Unbuilt> {
        if let Some(r) = self.cache.lock().unwrap().get(&t.to_bits()) { return Ok(r.clone()) }
        let (w,dw) = rates(self.path,t,self.whole)?;
        let pose = self.path.pose(t)?;
        let mut i = self.ring.instant(w,dw,Some(self.scale)).map_err(|at| Unbuilt::Curved {at:pose.point(at)})?;
        if self.unborn.is_some() && !same_shape(&self.first,&i.curves) {
            i = self.ring.instant_born(w,dw,Some(self.scale),self.unborn).map_err(|at| Unbuilt::Curved {at:pose.point(at)})?;
        }
        let refused = |c: &[Piece]| Unbuilt::Curved {at:pose.point(self.ring.root(&w,c[0].from.rem_euclid(TAU),c[0].label,false)
            .map_or(self.ring.centre,|r| r.x))};
        if ring_debug() && !same_shape(&self.first,&i.curves) {
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
        // (each bridge's end given way to a double curve put on its preimage at this roll, the
        // touching piece beside ending there too)
        for way in &self.ways {
            let Some(th) = way.at(t) else { continue };
            let c = &mut curves[way.curve];
            let (n,j) = (c.len(),way.piece);
            let old = if way.side == 0 { c[j].from } else { c[j].to };
            let th = old+((th-old+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI);
            if way.side == 0 { c[j].from = th; c[(j+n-1)%n].to += th-old } else { c[j].to = th; c[(j+1)%n].from += th-old }
            let ordered = c.iter().all(|p| p.to > p.from);
            if !ordered { return Err(Unbuilt::Failed(format!("a bridge's end given way to a double curve at {t} passes the piece beside it"))) }
        }
        let r = Read {instant:i,pose,curves};
        self.cache.lock().unwrap().insert(t.to_bits(),r.clone());
        Ok(r)
    }

    /// **Whether the run's point at `u`, `t` lies on a fold pair read as not yet born** (`unborn`):
    /// within the bridge the instant read whole puts in there, on the piece's own label. Its sheet
    /// folds there as a bridge's does, and the fold lies in the tail its double curve cuts off
    /// (`fitted_unborn`): read for folds no more than a bridge is.
    fn newborn(&self,run: &Run,faces: &[Face],u: f64,t: f64) -> bool {
        let Some(apex) = self.unborn else { return false };
        let Ok(r) = self.read(t) else { return false };
        let known = self.born.lock().unwrap().get(&t.to_bits()).cloned();
        let bridges = match known {
            Some(b) => b,
            None => {
                let Ok((w,dw)) = rates(self.path,t,self.whole) else { return false };
                let Ok(born) = self.ring.instant(w,dw,Some(self.scale)) else { return false };
                // (each curve's bridge nearest the swallowtail point's angle, as the unborn read drops)
                let apart = |p: &Piece| { let d = (0.5*(p.from+p.to)-apex).rem_euclid(TAU); d.min(TAU-d) };
                let b: Vec<(Label,[f64;2])> = if same_shape(&self.first,&born.curves) { Vec::new() } else {
                    born.curves.iter().filter_map(|c| (0..c.len()).filter(|&i| narrow(c,i)).min_by(|&i,&j| apart(&c[i]).total_cmp(&apart(&c[j])))
                        .map(|i| (c[i].label,[c[i].from.rem_euclid(TAU),c[i].to.rem_euclid(TAU)]))).collect()
                };
                self.born.lock().unwrap().insert(t.to_bits(),b.clone());
                b
            }
        };
        let k = (u.floor().max(0.) as usize).min(faces.len()-1);
        let Face {piece,share,of} = faces[k];
        let p = r.curves[run.curve][run.pieces[piece]];
        let th = (p.from+(p.to-p.from)*(share as f64+(u-k as f64))/of as f64).rem_euclid(TAU);
        bridges.iter().any(|&(l,[from,to])| l == p.label && if from <= to { th >= from && th <= to } else { th >= from || th <= to })
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
        // (a piece ending on a meridian the instant touches whole, where `A` and `B` pass through
        // nought and each branch's root turns half about, is read there on its own side: its root a
        // hair inside it where that stands apart from the end's, #880 T1d M2 — elsewhere the end's
        // own, the one the face beside reads its seam at)
        let th = p.from+(p.to-p.from)*f;
        let mut root = at(th)?;
        if f <= 0. || f >= 1. {
            let hair = (1e-10f64).min(0.25*(p.to-p.from).abs());
            let inner = at(th+if f <= 0. { hair } else { -hair }*(p.to-p.from).signum())?;
            if crate::util::space::distance(inner.x,root.x) > HALF_READ*self.ring.thickness { root = inner }
        }
        // (a narrow pair's bridge, between two touching pieces, is the sheet itself: the double curve
        // its outer leaves cross along passes through it, and its tail is cut off there, #880 T1d)
        let c = &r.curves[run.curve];
        let (n,i) = (c.len(),run.pieces[piece]);
        let narrow = narrow(c,i);
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
    fn folds(&self,net: &Net,us: &[f64],ts: &[f64],bridged: &dyn Fn(f64,f64) -> bool) -> Result<(),Unbuilt> {
        let read: Vec<Vec<Option<(V,V,f64)>>> = ts.iter().map(|&t| us.iter().map(|&u| (!bridged(u,t)).then(|| {
            let (x,du,dt) = net.d1(u,t);
            (x,cross(du,dt),norm(du)*norm(dt))
        })).collect()).collect();
        let flips = |p: &Option<(V,V,f64)>,q: &Option<(V,V,f64)>| match (p,q) {
            (Some((_,c,_)),Some((_,d,_))) => crate::util::space::dot(*c,*d) < 0.,
            _ => false,
        }.then(|| p.unwrap().0);
        for (j,row) in read.iter().enumerate() {
            for (i,p) in row.iter().enumerate() {
                if let Some((x,c,rates)) = p { if norm(*c) <= HALF_READ*rates {
                    if ring_debug() { eprintln!("ring: no area at u {:.6} t {:.6} over {:?}: {:e} of {:e}",us[i],ts[j],self.roll,norm(*c),rates) }
                    return Err(Unbuilt::Folded {at:*x})
                } }
                let beside = [row.get(i+1),read.get(j+1).map(|r| &r[i])];
                if let Some(at) = beside.into_iter().flatten().find_map(|q| flips(p,q)) {
                    if ring_debug() { eprintln!("ring: folds at u {:.6} t {:.6} over {:?}",us[i],ts[j],self.roll) }
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
    fitted_as(ring,path,whole,[a,b],scale,tol,None,Vec::new())
}

/// **`fitted_over` with the fold pairs born within the stretch read as not yet born** (#942
/// T1d-tori): its pieces those at the end `from` (the end before the birth), and an instant whose
/// shape is not theirs read with the pair nearest the swallowtail point's angle `apex` not put in
/// (each curve's: a pair born before the stretch began is put in as ever) — its bridge and the
/// touching pieces either side one touching piece across its sliver. The road fits so the stretch
/// from before a pair's birth to its swallowtail point (`closed::ring_fitted`), where its double
/// curve begins: the pair's sliver over it lies above the double curve, in the tail its cut drops,
/// and bounds nothing.
pub fn fitted_unborn(ring: &Ring,path: &Trajectory,whole: [f64;2],[a,b]: [f64;2],scale: f64,tol: f64,from: f64,apex: f64) -> Result<Fitted,Unbuilt> {
    fitted_as(ring,path,whole,[a,b],scale,tol,Some((from,apex)),Vec::new())
}

/// **`fit` fitted again with its bridges' ends given way** (`give_way`): as it was fitted (over its
/// stretch, read unborn as it was), each end in `ways` read on its double curve's preimage.
pub fn fitted_giving_way(ring: &Ring,path: &Trajectory,whole: [f64;2],fit: &Fitted,scale: f64,tol: f64,ways: Vec<Way>) -> Result<Fitted,Unbuilt> {
    fitted_as(ring,path,whole,fit.roll,scale,tol,fit.unborn,ways)
}

/// `fitted_over`, `fitted_unborn` and `fitted_giving_way`.
#[allow(clippy::too_many_arguments)]
fn fitted_as(ring: &Ring,path: &Trajectory,whole: [f64;2],[a,b]: [f64;2],scale: f64,tol: f64,unborn: Option<(f64,f64)>,ways: Vec<Way>) -> Result<Fitted,Unbuilt> {
    let mut reader = Reader::of(ring,path,whole,[a,b],scale,unborn)?;
    reader.ways = ways;
    let runs = runs(&reader.first);
    if runs.is_empty() { return Err(Unbuilt::Failed("no contact of the ring touches its sweep's boundary".into())) }
    let mut nets = Vec::new();
    let mut on = Vec::new();
    let mut sides = Vec::new();
    let mut doubles = Vec::new();
    let mut seeds = Vec::new();
    let mut apexes = Vec::new();
    let mut ends = Vec::new();
    let mut crossings = Vec::new();
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
        let (fit_clock,fit_before) = (crate::util::clock::Instant::now(),spent::now());
        let mut rounds = 0;
        let net = loop {
            crate::util::meter::go_on()?;
            rounds += 1;
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
            if ring_debug() {
                eprintln!("fit: curve {} over {:?}: {pieces} faces, {counts:?} points, {} rows: {:?} round, {:e} along",run.curve,reader.roll,ts.len()-1,
                    err_u.iter().map(|e| format!("{:.1e}",e.iter().fold(0f64,|m,&x| m.max(x)))).collect::<Vec<_>>(),err_t.iter().fold(0f64,|m,&x| m.max(x)));
                if let Some(j) = (0..err_t.len()).max_by(|&i,&j| err_t[i].total_cmp(&err_t[j])) {
                    let worst = us.iter().zip(&halfway[j]).map(|(&u,&x)| (norm(crate::util::space::sub(x,net.point(u,half_ts[j]))),u)).fold((0f64,0f64),|m,p| if p.0 > m.0 { p } else { m });
                    eprintln!("fit: the most along at t {} (rows {} to {}), u {}",half_ts[j],ts[j],ts[j+1],worst.1);
                }
            }
            let missed_u: Vec<Vec<bool>> = err_u.iter().map(|e| e.iter().map(|&x| x > tol).collect()).collect();
            let missed_t: Vec<bool> = err_t.iter().map(|&x| x > tol).collect();
            // a bridge's points, or on the join of one
            let bridge = |k: usize| reader.first[run.curve][run.pieces[faces[k%pieces].piece]].side == Side::Fold;
            let bridged = |u: f64,t: f64| {
                // (the loop closing at `pieces`: the piece after its last join the first)
                let k = (u.floor().max(0.) as usize)%pieces;
                bridge(k) || (u-u.round()).abs() <= 1e-9 && bridge((u.round() as usize+pieces-1)%pieces)
                    || reader.newborn(run,&faces,u,t)
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
        // Newton's method on the net from the bridge's ends (the normal form `θ³ − 3h²θ` puts the
        // preimages at `±√3 h`, `√3 − 1` of the half-width `h` past the folds, and the bridge, its
        // band within a hair of the folds, is no narrower: so within that of its ends, #942 F6); a
        // row where it settles on none is where the pair is born, its bridge no wider than the
        // rounding, both preimages the bridge's middle
        if crate::timing() {
            eprintln!("ring: curve {} over {:?} fitted in {:?}: {rounds} rounds, {} faces, {} rows; {}",run.curve,[a,b],fit_clock.elapsed(),pieces,ts.len()-1,
                spent::since(fit_before))
        }
        let (seed_clock,seed_before) = (crate::util::clock::Instant::now(),spent::now());
        let n = run.pieces.len();
        let mut found = Vec::new();
        for (fk,f) in faces.iter().enumerate() {
            if !narrow(&reader.first[run.curve],run.pieces[f.piece]) { continue }
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
                let past = (3f64.sqrt()-1.)*half;
                let (dl,dr) = (past/((pl.to-pl.from)/per((f.piece+n-1)%n)).max(1e-300),past/((pr.to-pr.from)/per((f.piece+1)%n)).max(1e-300));
                let mut x = last.unwrap_or([k0,k1,t]);
                let mut settled = false;
                // (damped least squares, Levenberg–Marquardt, each unknown scaled by its column: near
                // where the pair is born the two leaves are all but tangent and the sheet's rates all
                // but parallel, and Newton's own steps flew off)
                // (each preimage within `√3 − 1` of the bridge's half-width past its end: past that a
                // leaf meets another stretch of the tube, which is the trim's to meet, not the pair's
                // double curve)
                let lo = [(k0-per((f.piece+n-1)%n)).max(k0-dl),k1,a];
                let hi = [k0,(k1+per((f.piece+1)%n)).min(k1+dr),b];
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
            if ring_debug() { eprintln!("ring: a narrow pair's leaves first cross at {seed:?}") }
            // (and where the rows find none — the crossing's other preimage in the stretch before
            // a pair's birth, or within its bridge — the swallowtail point its double curve is built
            // from, where its folds meet the sheet's kernel within the stretch, #942 T1d-tori; and
            // where they do, the pair's swallowtail point all the same, the curve traced from the
            // seed joined to it: beside the thin torus's points the first preimage's angle turns back
            // within 2e-4, and built from the point, stepped by that angle, the curve went no further)
            let st = swallowtails(&reader,run,f.piece,[a,b])?;
            if ring_debug() { eprintln!("ring: a narrow pair's swallowtail points over {:?}: {st:?}",[a,b]) }
            if seed.is_none() { apexes.extend(st) } else { ends.extend(st) }
            found.extend(seed);
        }
        if crate::timing() { eprintln!("ring: curve {} over {:?}: its narrow pairs' crossings sought in {:?}; {}",run.curve,[a,b],seed_clock.elapsed(),spent::since(seed_before)) }
        seeds.extend(found);
        doubles.push(Vec::new());
        // (each seam where the branch's label turns over between a touching piece and a dipped one:
        // a meridian the instant touches whole, #942 T1d-ring)
        let first = &reader.first[run.curve];
        crossings.push((0..pieces).filter_map(|fk| {
            let before = (fk+pieces-1)%pieces;
            let (p,q) = (first[run.pieces[faces[before].piece]],first[run.pieces[faces[fk].piece]]);
            let turned = p.label.on == q.label.on && p.label.branch != q.label.branch;
            let fin = match (p.side,q.side) { (Side::Touching,Side::Fold) => fk,(Side::Fold,Side::Touching) => before,_ => return None };
            turned.then(|| Crossing {seam:fk,far:if fin == fk { (fk+1)%pieces } else { before },theta:q.from.rem_euclid(TAU)})
        }).collect());
        nets.push(net);
        on.push(faces.iter().map(|f| reader.first[run.curve][run.pieces[f.piece]].label.on).collect());
        sides.push(faces.iter().map(|f| reader.first[run.curve][run.pieces[f.piece]].side).collect());
    }
    let tails = vec![Vec::new();doubles.len()];
    let strips = strips(ring,path,[a,b],&nets,&crossings,tol)?;
    Ok(Fitted {nets,on,sides,roll:[a,b],doubles,tails,path:path.clone(),seeds,apexes,ends,crossings,strips,unborn,ways:reader.ways})
}

impl Ring {
    /// **The meridian the twist `w` touches whole nearest the angle `θ`** (#942 T1d-ring), as
    /// `touched_whole` reads one, and its arc: of the two angles either arc's `A` vanishes at, the
    /// one `B` vanishes at too nearest `θ`.
    fn touched_near(&self,w: &Twist,theta: f64) -> Option<(f64,usize)> {
        let way = |st: f64| (theta-st+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI;
        self.elements.iter().enumerate().filter_map(|(k,e)| {
            let Element::Arc {c,..} = *e else { return None };
            let (pp,q,qq,size) = self.terms(w,c);
            let base = (-pp[0]).datan2(pp[1]);
            [base,base+std::f64::consts::PI].into_iter().filter(|&st| { let (ss,cs) = st.dsin_cos(); (q+qq[0]*cs+qq[1]*ss).abs() <= HALF_READ*size })
                .map(|st| (st.rem_euclid(TAU),k)).min_by(|x,y| way(x.0).abs().total_cmp(&way(y.0).abs()))
        }).min_by(|x,y| way(x.0).abs().total_cmp(&way(y.0).abs()))
    }

    /// The point of the arc `k` at `ψ` round its centre in the half-plane at `θ`, as a contact
    /// there (`root`'s), so its `s''` is read as any contact's.
    fn on_arc(&self,theta: f64,k: usize,psi: f64) -> Option<Root> {
        let Element::Arc {c,r,sweep,..} = self.elements[k] else { return None };
        let (er,_,ez) = self.axes(theta);
        let lift3 = |m: P2| add(scale(er,m[0]),scale(ez,m[1]));
        let (sp,cp) = psi.dsin_cos();
        let g = sweep.signum();
        let m = [c[0]+r*cp,c[1]+r*sp];
        Some(Root {x:self.lift(theta,m),n:lift3([g*cp,g*sp]),sigma:k as f64,bend:Bend::Face {km:g/r,tm:lift3([-sp,cp]),kp:g*cp/m[0]}})
    }

    /// The angle round the arc `k`'s centre of the point `x` (the tool's coordinates).
    fn arc_angle(&self,k: usize,x: V) -> f64 {
        let Element::Arc {c,..} = self.elements[k] else { return f64::NAN };
        let l = self.frame.local(x);
        (l[2]-c[1]).datan2(l[0].dhypot(l[1])-c[0])
    }
}

/// **Each meridian the stretch `[a, b]` touches whole, its touching half fitted within `tol`**
/// (#942 T1d-ring, `Strip`): the crossings the tubes record (`Crossing`) paired by the meridian
/// they lie on — two to a meridian, or refused by name (`Curved`). At each roll the meridian's angle
/// is read again (`touched_near`) and the two crossings on it — the arc's two roots a hair off it,
/// the tubes' own reading of their seams there (`Reader::point`) — and the sheet is the arc between
/// them on the side where `s''` stands above nought at the first roll, kept so, fitted as a tube
/// is: rows and points halved where the net misses at their middles.
fn strips(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],nets: &[Net],crossings: &[Vec<Crossing>],tol: f64) -> Result<Vec<Strip>,Unbuilt> {
    let mut all: Vec<(usize,Crossing)> = crossings.iter().enumerate().flat_map(|(i,c)| c.iter().map(move |&x| (i,x))).collect();
    let mut out = Vec::new();
    while let Some(x) = all.pop() {
        let way = |p: f64,q: f64| ((p-q+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI).abs();
        // (the other crossing on its meridian: within a hair of its angle, the meridians a ring's
        // axis touches whole half a turn apart)
        let Some(j) = all.iter().position(|y| y.0 != x.0 && way(y.1.theta,x.1.theta) < 0.25*std::f64::consts::PI) else {
            return Err(Unbuilt::Curved {at:path.pose(a)?.point(nets[x.0].point(x.1.seam as f64,a))})
        };
        let y = all.remove(j);
        let pair = [x,y];
        // each roll's meridian, the crossings' angles round its arc in the tubes' order, and the
        // way round from the first to the second
        let read = |t: f64| -> Result<(f64,usize,[f64;2],Twist,Twist),Unbuilt> {
            let (w,dw) = rates(path,t,[a,b])?;
            let curved = || Unbuilt::Curved {at:nets[x.0].point(x.1.seam as f64,t)};
            let (theta,k) = ring.touched_near(&w,x.1.theta).ok_or_else(curved)?;
            let hair = 1e-10;
            let roots: Vec<V> = [true,false].iter().filter_map(|&branch| ring.root(&w,theta+hair,Label {on:On::Face(k),branch},false)).map(|r| r.x).collect();
            if roots.len() != 2 { return Err(curved()) }
            let pose = path.pose(t)?;
            let seams = pair.map(|(i,c)| nets[i].point(c.seam as f64,t));
            let near = |s: V| (0..2).min_by(|&m,&n| crate::util::space::distance(pose.point(roots[m]),s)
                .total_cmp(&crate::util::space::distance(pose.point(roots[n]),s))).unwrap();
            let (m0,m1) = (near(seams[0]),near(seams[1]));
            if m0 == m1 { return Err(curved()) }
            Ok((theta,k,[ring.arc_angle(k,roots[m0]),ring.arc_angle(k,roots[m1])],w,dw))
        };
        let (theta,k,[p0,p1],w,dw) = read(0.5*(a+b))?;
        let up = (p1-p0).rem_euclid(TAU);
        let mid = |sense: f64| ring.on_arc(theta,k,p0+sense*0.5*(if sense > 0. { up } else { TAU-up })).map_or(f64::NAN,|r| ring.bend(&w,&dw,theta,&r));
        let sense = if mid(1.) > 0. { 1. } else if mid(-1.) > 0. { -1. } else { return Err(Unbuilt::Curved {at:path.pose(a)?.point(ring.meridian_point(theta,k))}) };
        let arc = k;
        let point = |v: f64,t: f64| -> Result<V,Unbuilt> {
            let (theta,k,[p0,p1],..) = read(t)?;
            if k != arc { return Err(Unbuilt::Curved {at:path.pose(t)?.point(ring.meridian_point(theta,k))}) }
            let span = if sense > 0. { (p1-p0).rem_euclid(TAU) } else { -(p0-p1).rem_euclid(TAU) };
            let r = ring.on_arc(theta,k,p0+v*span).ok_or(Unbuilt::Failed("a ring's whole-touched meridian is no arc".into()))?;
            Ok(path.pose(t)?.point(r.x))
        };
        let mut ts: Vec<f64> = (0..=FIRST_ROWS).map(|j| a+(b-a)*j as f64/FIRST_ROWS as f64).collect();
        let mut us: Vec<f64> = (0..=FIRST_POINTS).map(|i| i as f64/FIRST_POINTS as f64).collect();
        let net = loop {
            crate::util::meter::go_on()?;
            let mid_u: Vec<f64> = us.windows(2).map(|w| 0.5*(w[0]+w[1])).collect();
            let mid_t: Vec<f64> = ts.windows(2).map(|w| 0.5*(w[0]+w[1])).collect();
            let rows: Vec<(f64,bool)> = ts.iter().map(|&t| (t,true)).chain(mid_t.iter().map(|&t| (t,false))).collect();
            let read_rows: Vec<Vec<V>> = crate::util::par::map(&rows,|&(t,fitted)| -> Result<Vec<V>,Unbuilt> {
                if fitted { us.iter().chain(&mid_u).map(|&v| point(v,t)).collect() } else { us.iter().map(|&v| point(v,t)).collect() }
            }).into_iter().collect::<Result<_,_>>()?;
            let (fitted,halfway) = read_rows.split_at(ts.len());
            let grid: Vec<Vec<V>> = fitted.iter().map(|r| r[..us.len()].to_vec()).collect();
            let net = interpolated_by(&grid,&us,&ts,&[us.len()-1])?;
            let mut err_u = vec![0f64;mid_u.len()];
            for (&t,r) in ts.iter().zip(fitted) {
                for (i,(&v,&x)) in mid_u.iter().zip(&r[us.len()..]).enumerate() { err_u[i] = err_u[i].max(norm(crate::util::space::sub(x,net.point(v,t)))) }
            }
            let err_t: Vec<f64> = mid_t.iter().zip(halfway).map(|(&t,r)| missed(&net,us.iter().zip(r).map(|(&v,&x)| (t,v,x)))).collect();
            let (miss_u,miss_t): (Vec<bool>,Vec<bool>) = (err_u.iter().map(|&e| e > tol).collect(),err_t.iter().map(|&e| e > tol).collect());
            if !miss_u.contains(&true) && !miss_t.contains(&true) { break net }
            let (nu,nt) = (halved(&us,&miss_u),halved(&ts,&miss_t));
            if nu.len()-1 > MOST_POINTS || nt.len()-1 > MOST_ROWS {
                return Err(Unbuilt::Failed(format!("a ring's whole-touched meridian's sheet fits within {tol} only to {:e} round it and {:e} along the roll",
                    err_u.iter().fold(0f64,|m,&e| m.max(e)),err_t.iter().fold(0f64,|m,&e| m.max(e)))))
            }
            (us,ts) = (nu,nt);
        };
        out.push(Strip {net,arc,sides:pair});
    }
    Ok(out)
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

/// **The swallowtail points of a narrow pair's folds over the stretch `[a, b]`** (#942 T1d-tori):
/// the bridge `piece` of `run`, read by `reader`. Where a pair is born at an instant its folds are
/// the two branches of one curve in `(θ, t)`, `s'' = 0`, tangent there to the instant's own contact,
/// whose rate `Σ_θ` is no kernel of the sheet's: so the birth is an ordinary point of a cuspidal
/// edge, and the double curve does not begin there. It begins where a fold runs along the kernel
/// of the sheet's rates — `k = (−Σ_θ · Σ_t / |Σ_θ|², 1)` at the fold, where `Σ_θ` and `Σ_t` are one
/// way — the swallowtail point: `∇s'' · k`, `−∂s''/∂t` at the birth, passes nought along one
/// branch as the fold turns from the instant's way toward the roll's (the half turn's left fold,
/// 0.0072 of the roll after the pair is born, the kernel's `dθ/dt` −2.06). Each branch read at the
/// stretch's ends (the rate's differences inside the whole roll) and halved, to the rounding, where
/// it changes sign; the fold at each roll halved to the rounding from the pair's least `s''`
/// (golden sections over the bridge) toward the bridge's end, where `s''` is the depth of its least
/// above nought. Every rate a central difference.
fn swallowtails(reader: &Reader,run: &Run,piece: usize,[a,b]: [f64;2]) -> Result<Vec<Apex>,Unbuilt> {
    let (ring,path,whole) = (reader.ring,reader.path,reader.whole);
    // (a central difference's step: the cube root of the rounding, where the difference's rounding
    // and its truncation balance, of an angle and of the roll)
    let h = THIRD_READ;
    let ht = h*(whole[1]-whole[0]);
    // the bridge's label and angles at `t`; `s''` and the sheet's point at `(θ, t)` on its label
    let bridge = |t: f64| -> Result<(Label,[f64;2]),Unbuilt> {
        let r = reader.read(t)?;
        let p = r.curves[run.curve][run.pieces[piece]];
        Ok((p.label,[p.from,p.to]))
    };
    let bend = |l: Label,th: f64,t: f64| -> Result<Option<f64>,Unbuilt> {
        let (w,dw) = rates(path,t,whole)?;
        let th = th.rem_euclid(TAU);
        Ok(ring.root(&w,th,l,false).map(|r| ring.bend(&w,&dw,th,&r)))
    };
    let point = |l: Label,th: f64,t: f64| -> Result<Option<(V,f64)>,Unbuilt> {
        let w = Twist::body(path.at(t)?);
        let pose = path.pose(t)?;
        Ok(ring.root(&w,th.rem_euclid(TAU),l,false).map(|r| (pose.point(r.x),r.sigma)))
    };
    // the fold on `side` of the pair at `t`
    let fold = |t: f64,side: f64| -> Result<Option<(Label,f64)>,Unbuilt> {
        let (l,[from,to]) = bridge(t)?;
        let v = |th: f64| -> Result<f64,Unbuilt> { Ok(bend(l,th,t)?.unwrap_or(f64::INFINITY)) };
        let g = 0.5*(5f64.sqrt()-1.);
        let (mut lo,mut hi) = (from,to);
        loop {
            let (x1,x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
            if !(x1 > lo && x2 < hi && x1 < x2) { break }
            if v(x1)? < v(x2)? { hi = x2 } else { lo = x1 }
        }
        let least = 0.5*(lo+hi);
        let end = if side < 0. { from } else { to };
        if !(v(least)? < 0.) || !(v(end)? > 0.) { return Ok(None) }
        let (mut inside,mut out) = (least,end);
        loop {
            let m = 0.5*(inside+out);
            if m == inside || m == out { break }
            if v(m)? < 0. { inside = m } else { out = m }
        }
        Ok(Some((l,0.5*(inside+out))))
    };
    // `∇s'' · k` at the fold on `side` at `t`, its label, angle and the kernel's `dθ/dt`
    let gauge = |t: f64,side: f64| -> Result<Option<(f64,Label,f64,f64)>,Unbuilt> {
        let Some((l,th)) = fold(t,side)? else { return Ok(None) };
        let (Some(sp),Some(sm),Some(tp),Some(tm)) = (bend(l,th+h,t)?,bend(l,th-h,t)?,bend(l,th,t+ht)?,bend(l,th,t-ht)?) else { return Ok(None) };
        let (Some(xp),Some(xm),Some(yp),Some(ym)) = (point(l,th+h,t)?,point(l,th-h,t)?,point(l,th,t+ht)?,point(l,th,t-ht)?) else { return Ok(None) };
        let (s_th,s_t) = (scale(crate::util::space::sub(xp.0,xm.0),0.5/h),scale(crate::util::space::sub(yp.0,ym.0),0.5/ht));
        let ak = -dot(s_th,s_t)/dot(s_th,s_th);
        Ok(Some(((sp-sm)*0.5/h*ak+(tp-tm)*0.5/ht,l,th,ak)))
    };
    let (lo,hi) = (a.max(whole[0]+ht),b.min(whole[1]-ht));
    let mut out = Vec::new();
    for side in [-1.,1.] {
        let (Some(gl),Some(gh)) = (gauge(lo,side)?,gauge(hi,side)?) else { continue };
        if (gl.0 < 0.) == (gh.0 < 0.) { continue }
        let (mut x,mut y) = (lo,hi);
        loop {
            let m = 0.5*(x+y);
            if !(m > x && m < y) { break }
            let Some(gm) = gauge(m,side)? else { break };
            if (gm.0 < 0.) == (gl.0 < 0.) { x = m } else { y = m }
        }
        let t = 0.5*(x+y);
        let Some((_,l,th,ak)) = gauge(t,side)? else { continue };
        // (a kernel along the roll alone: the double curve no graph over its first preimage's
        // angle, which `double_curve` steps — not yet built)
        if !(ak.abs() > 0.) { continue }
        let Some((_,sigma)) = point(l,th,t)? else { continue };
        out.push(Apex {at:(th,t,sigma),way:1./ak});
    }
    Ok(out)
}

/// The ring's root at the angle `θ` with label `l` under twist `w` (`Ring::root`).
fn self_root(ring: &Ring,w: &Twist,theta: f64,l: Label) -> Option<Root> { ring.root(w,theta.rem_euclid(TAU),l,false) }

/// **Whether `at` (`(θ, t, σ)`, a contact's point at an instant) lies on the double curve `curve`**
/// (`double_curve`'s points): its point within twice `tol` of the curve's polyline in space — each
/// traced point on the curve, and the chords between them sagging within `tol` (its step, #942 F6).
pub fn on_double_curve(ring: &Ring,path: &Trajectory,curve: &[(f64,f64,f64)],at: (f64,f64,f64),tol: f64) -> bool {
    use crate::util::space::{distance,segment_distance};
    let Some((x,_)) = sheet_at(ring,path,at.0,at.1,at.2) else { return false };
    // (read along the curve, the first point or chord within reach the answer)
    let mut last: Option<V> = None;
    for &(th,t,sg) in curve {
        let p = if th.is_finite() && t.is_finite() { sheet_at(ring,path,th,t,sg).map(|p| p.0) } else { None };
        if let Some(p) = p {
            if distance(x,p) <= 2.*tol || last.is_some_and(|q| segment_distance(x,q,p) <= 2.*tol) { return true }
        }
        last = p;
    }
    false
}

/// The sheet's point at the angle `θ` and roll `t`, the contact's root there nearest the place `σ`
/// round the meridian, and that root's place.
pub(crate) fn sheet_at(ring: &Ring,path: &Trajectory,theta: f64,t: f64,sigma: f64) -> Option<(V,f64)> {
    let w = Twist::body(path.at(t).ok()?);
    let (_,r) = ring.root_near(&w,theta.rem_euclid(TAU),sigma)?;
    Some((path.pose(t).ok()?.point(r.x),r.sigma))
}

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
pub fn double_curve(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],seed: [(f64,f64,f64);2],ends: &[Apex],tol: f64) -> Result<Vec<(f64,f64,f64)>,Unbuilt> {
    traced(ring,path,[a,b],seed,None,ends,tol)
}

/// **The three unknowns `t₁, θ₂, t₂` of a double curve's pair settled at `θ₁`**, from `z`, by
/// Newton's method on the contact itself, each root the one nearest its place `σ₁`, `σ₂` round the
/// meridian: the pair and the roots' places, or `None` where it settles on none.
#[allow(clippy::too_many_arguments)]
fn settled_pair(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],th1: f64,z: [f64;3],s1: f64,s2: f64,tol: f64) -> Option<([f64;3],f64,f64)> {
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
}

/// **A ring's sheet's double curve built outward from its swallowtail point** (#942 T1d-tori):
/// `double_curve`, its first pair from the swallowtail's normal form — about the point, its two
/// preimages stand either way along the sheet's kernel, `(1, dt/dθ)`, meeting there to the third
/// order (`(x, ±τ)` with `x = −2τ²`, the folds at `x = −6τ²`), so the pair `θ ∓ s` along it is
/// settled by Newton's method on the contact, `s` halved from half the double curve's greatest step
/// until it settles across the point and the chord from the point sags within the tolerance at its
/// middle — then marched out until either preimage leaves the roll. Not traced toward the point:
/// there its two preimages meet and the sheets cross ever more shallowly, and beside a fold pair's
/// birth one preimage lies in the stretch before the cut, which no stretch's net reaches (the half
/// turn's, #942 T1d-tori). The point itself is on the curve, between its two preimages' points.
pub fn double_curve_from_apex(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],apex: Apex,tol: f64) -> Result<Vec<(f64,f64,f64)>,Unbuilt> {
    let failed = || Unbuilt::Failed("a ring's sheet's double curve not built from its swallowtail point".into());
    // (the double curve's own steps, `double_curve`'s: at most the angle over which the meridian's
    // chord sags the tolerance, at least the angle over which the meridian moves it)
    let (most,least) = ((8.*tol/ring.thickness).sqrt(),tol/ring.thickness);
    let (th,t,sg) = apex.at;
    let Some((x0,_)) = sheet_at(ring,path,th,t,sg) else { return Err(failed()) };
    // (the second preimage across the point, not the first itself — the pair of no distance every
    // point of the sheet is — and both within the roll)
    let pair = |s: f64| settled_pair(ring,path,[a,b],th-s,[t-apex.way*s,th+s,t+apex.way*s],sg,sg,tol)
        .filter(|(y,..)| y[1] > th && [y[0],y[2]].iter().all(|r| *r >= a && *r <= b));
    let mut s = 0.5*most;
    let seed = loop {
        let near = pair(s).and_then(|(y,a1,a2)| {
            let (ym,am,_) = pair(0.5*s)?;
            let (p,pm) = (sheet_at(ring,path,th-s,y[0],a1)?.0,sheet_at(ring,path,th-0.5*s,ym[0],am)?.0);
            (crate::util::space::distance(pm,scale(add(x0,p),0.5)) <= tol).then_some([(th-s,y[0],a1),(y[1],y[2],a2)])
        });
        if let Some(seed) = near { break seed }
        if 0.5*s < least { return Err(failed()) }
        s *= 0.5;
    };
    if ring_debug() {
        eprintln!("ring: a double curve built from its swallowtail point {:?}, its first pair {seed:?}",apex.at)
    }
    traced(ring,path,[a,b],seed,Some(apex.at),&[],tol)
}

/// **A double curve's pair joined to its swallowtail point** (#942 T1d-tori): beside the point the
/// curve's two leaves stand within the tolerance of one another over a band — the region the
/// curve bounds is covered there less than the tolerance deep (the thin torus's, some 2e-3 of the
/// roll about its points) — so the curve is fixed there to the tolerance alone, and neither a march
/// toward the point nor one out from it settles on it: toward it, the thin torus's preimages slid
/// onto a fold's inner leaf 2e-3 short of its point and ran on along it. So the pair `pair`, traced
/// from far, is joined to the point `apex` along the swallowtail's normal form — each preimage's
/// offset from the point along the sheet's kernel `k = (1, dt/dθ)` taken linearly and across it
/// quadratically (`(x, ±τ)` with `x = −2τ²`) — once it stands within the pair's parting of the point
/// (the normal form's `τ` within its `2τ`) and every point of the join, its two leaves' points at
/// each fraction, stands within the tolerance of the other and on an outer leaf (`s''` above
/// nought); its points from the pair toward the point, each chord of the join no longer in space
/// than the double curve's own step allows (`√(8 tol r)`, the chord the tube's bend sags the
/// tolerance), the point itself not among them, and the point its angle unwrapped as the pair's.
/// `None` where it does not hold.
fn joined(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],pair: [(f64,f64,f64);2],apex: &Apex,tol: f64) -> Option<((f64,f64,f64),Vec<[(f64,f64,f64);2]>)> {
    let (tha,ta,sa) = apex.at;
    // the offsets in `(θ, t)` from the point, the angle taken beside it — and the point's angle
    // unwrapped as the curve's is
    let off = |(th,t,_): (f64,f64,f64)| [(th-tha+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI,t-ta];
    let [d1,d2] = pair.map(off);
    let tha = pair[0].0-d1[0];
    if d1[0].hypot(d1[1]) > (d2[0]-d1[0]).hypot(d2[1]-d1[1]) { return None }
    let kn = 1f64.hypot(apex.way);
    let (k,n) = ([1./kn,apex.way/kn],[-apex.way/kn,1./kn]);
    let parts = |d: [f64;2]| (d[0]*k[0]+d[1]*k[1],d[0]*n[0]+d[1]*n[1]);
    let (p1,p2) = (parts(d1),parts(d2));
    let at = |(al,be): (f64,f64),f: f64,sg: f64| (tha+f*al*k[0]+f*f*be*n[0],ta+f*al*k[1]+f*f*be*n[1],sa+f*(sg-sa));
    // (as many chords as the double curve's longest step takes of the join's length in space,
    // each read at its middle too)
    let (x,_) = sheet_at(ring,path,pair[0].0,pair[0].1,pair[0].2)?;
    let (xa,_) = sheet_at(ring,path,tha,ta,sa)?;
    let first = (crate::util::space::distance(x,xa)/(8.*tol*ring.thickness).sqrt()).ceil().max(1.) as usize;
    let outer = |(th,t,sg): (f64,f64,f64)| -> Option<V> {
        let (w,dw) = rates(path,t,[a,b]).ok()?;
        let th = th.rem_euclid(TAU);
        let (_,r) = ring.root_near(&w,th,sg)?;
        (ring.bend(&w,&dw,th,&r) > 0.).then(|| path.pose(t).ok().map(|m| m.point(r.x)))?
    };
    // (and as many again, twice over, until each leaf's chord straight in its `(θ, t)` — what a
    // tube's face draws between two of its points, #942 the full turn — stands at its middle within
    // the tolerance of the curve's own middle: across the kernel each preimage is the parabola
    // `x = −2τ²`, and the two leaves bend opposite ways, so their straight chords part, the full
    // turn's by 3.8e-4 beside its point at the cut 2.2318; doubled no further than chords no longer
    // in space than the tolerance, whose middles stand within it of anything between their ends)
    let mid = |q: (f64,f64,f64),r: (f64,f64,f64)| (0.5*(q.0+r.0),0.5*(q.1+r.1),0.5*(q.2+r.2));
    let mut chords = first;
    loop {
        let mut out = Vec::new();
        let mut straight = true;
        let mut last: Option<[(f64,f64,f64);2]> = None;
        for j in (1..2*chords).rev() {
            let f = j as f64/(2*chords) as f64;
            let (q1,q2) = (at(p1,f,pair[0].2),at(p2,f,pair[1].2));
            let (y1,y2) = (outer(q1)?,outer(q2)?);
            if crate::util::space::distance(y1,y2) > tol { return None }
            if j%2 == 0 { out.push([q1,q2]); last = Some([q1,q2]); continue }
            // (the middle of the chord from the point after it, or the pair, to the point before)
            let after = last.unwrap_or(pair);
            let g = (j-1) as f64/(2*chords) as f64;
            let before = if j == 1 { [(tha,ta,sa);2] } else { [at(p1,g,pair[0].2),at(p2,g,pair[1].2)] };
            for (k,y) in [y1,y2].into_iter().enumerate() {
                if outer(mid(after[k],before[k])).is_none_or(|z| crate::util::space::distance(z,y) > tol) { straight = false }
            }
        }
        if ring_debug() { eprintln!("ring: joined to {:?} in {chords} chords, straight {straight}",apex.at) }
        if straight || crate::util::space::distance(x,xa) <= tol*chords as f64 { return Some(((tha,ta,sa),out)) }
        chords *= 2;
    }
}

/// `double_curve` from `seed`, toward the pair unless it is built from its swallowtail point
/// `apex`, which then stands between the seed's two preimages.
fn traced(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],seed: [(f64,f64,f64);2],apex: Option<(f64,f64,f64)>,ends: &[Apex],tol: f64)
    -> Result<Vec<(f64,f64,f64)>,Unbuilt> {
    let failed = || Unbuilt::Failed("a ring's sheet's double curve about a fold pair not traced".into());
    // (stepped round the axis, as the angle its first preimage moves, by as much as the angle over
    // which the meridian's chord sags the tolerance, `√(8 tol / r)`, and halved where the curve's
    // own chord sags more at its middle, down to the angle over which the meridian moves the
    // tolerance, `tol / r` — nearer, two points are one to the tolerance, #942 F6)
    let (most,least) = ((8.*tol/ring.thickness).sqrt(),tol/ring.thickness);
    let settle = |th1: f64,z: [f64;3],s1: f64,s2: f64| settled_pair(ring,path,[a,b],th1,z,s1,s2,tol);
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
    let mut stopped_at = most;
    let mut joined_at: Option<(f64,f64,f64)> = None;
    for (dir,out) in [(toward,&mut near),(-toward,&mut far)] {
        // (from a swallowtail point, away alone)
        if apex.is_some() && dir == toward { continue }
        let (mut th,mut z,mut sg1,mut sg2) = (th1,[t1,th2,t2],s1,s2);
        // (the pair before, the swallowtail point itself where the curve is built from it: where the
        // last pair alone does not settle the next, it is sought from the line through the two —
        // beside the point the preimages part either way at once, and from the last alone the step
        // found none but the pair of no distance)
        let mut prev: Option<(f64,[f64;3])> = apex.filter(|_| dir != toward).map(|p| (p.0,[p.1,p.0,p.1]));
        let mut step = most;
        for _ in 0..200_000 {
            // (toward the pair, joined to a swallowtail point `ends` names where the join holds)
            if dir == toward {
                let here = [(th,z[0],sg1),(z[1],z[2],sg2)];
                if let Some((e,join)) = ends.iter().find_map(|e| joined(ring,path,[a,b],here,e,tol)) {
                    out.extend(join);
                    joined_at = Some(e);
                    break
                }
            }
            let next = th+dir*step;
            // (away from the pair, a pair whose preimages have met or crossed — nearer than the least
            // step, one point to the tolerance — is none: the pair of no distance every point of the
            // sheet is, which the march from beside a swallowtail point slid onto and ran along, the
            // thin torus's, #942 T1d-tori; where the preimages meet the step halves, and the curve
            // stops short of a far swallowtail point. Nor, built from a swallowtail point, is one
            // whose second preimage comes back: beside the half turn's, where the two sheets cross
            // at an angle of the pair's width, the pair settled within its residual 1e-6 of the
            // angle back, and once with both preimages on one side. A curve traced from a row's
            // seed is not yet held so: the turned torus's second preimage comes back over its
            // angle, and held so its trim was refused sooner)
            let reach = |at: f64| -> Option<([f64;3],f64,f64)> {
                let apart = |r: &([f64;3],f64,f64)| dir == toward || (r.0[1]-at)*toward > least && (apex.is_none() || (r.0[1]-z[1])*toward > 0.);
                settle(at,z,sg1,sg2).filter(apart).or_else(|| {
                    let (tp,zp) = prev?;
                    let f = (at-th)/(th-tp);
                    settle(at,std::array::from_fn(|i| z[i]+(z[i]-zp[i])*f),sg1,sg2).filter(apart)
                })
            };
            let settled = reach(next);
            // (the step halved where it settles on no meeting, or where the curve's chord over it
            // sags more than the tolerance at its middle)
            let sag = settled.and_then(|(y,a1,_)| {
                let mid = th+dir*0.5*step;
                let (ym,am,_) = reach(mid)?;
                let (p0,p1,pm) = (sheet_at(ring,path,th,z[0],sg1)?.0,sheet_at(ring,path,next,y[0],a1)?.0,sheet_at(ring,path,mid,ym[0],am)?.0);
                Some(crate::util::space::distance(pm,scale(crate::util::space::add(p0,p1),0.5)))
            });
            if sag.is_none_or(|g| g > tol) && 0.5*step >= least { step *= 0.5; continue }
            stopped_at = step;
            let Some((y,a1,a2)) = settled else {
                if ring_debug() {
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
            prev = Some((th,z));
            (th,z,sg1,sg2) = (next,y,a1,a2);
            out.push([(th,y[0],a1),(y[1],y[2],a2)]);
            // (grown again where the chord sagged a quarter of the tolerance or less)
            if sag.is_some_and(|g| g <= 0.25*tol) { step = (2.*step).min(most) }
        }
    }
    if near.is_empty() && far.is_empty() { return Err(failed()) }
    // in order of angle: the first preimage from far to near, then the second from near to far
    let first: Vec<(f64,f64,f64)> = far.iter().rev().map(|p| p[0]).chain(std::iter::once((th1,t1,s1))).chain(near.iter().map(|p| p[0])).collect();
    let second: Vec<(f64,f64,f64)> = near.iter().rev().map(|p| p[1]).chain(std::iter::once((th2,t2,s2))).chain(far.iter().map(|p| p[1])).collect();
    // (the second taken from where it passes the first's last angle: the step that found the
    // preimages met stood a hair past the swallowtail point)
    // (not where it is joined to its swallowtail point: the join ends at the point itself)
    let last = first.last().map_or(f64::NAN,|p| p.0);
    let second: Vec<(f64,f64,f64)> = second.into_iter().skip_while(|p| joined_at.is_none() && (p.0-last)*toward <= 0.).collect();
    let at = first.len();
    // (the swallowtail point: between the two leaves' preimages — the one it was built from or
    // joined to in the curve itself — put at the end of the list)
    let apex = apex.or(joined_at);
    let mut out: Vec<(f64,f64,f64)> = first.into_iter().chain(apex).chain(second).collect();
    let ap = if let Some(p) = apex { p } else if at > 0 && at < out.len() { let (p,q) = (out[at-1],out[at]); (0.5*(p.0+q.0),0.5*(p.1+q.1),p.2) }
        else { out[at.min(out.len()-1)] };
    // (and where the way away stopped short with its preimages within the step it stopped at either
    // side of one another, the second: the curve closed through it, from the second preimage's far
    // end to the first's)
    let far_apex = far.last().filter(|l| short && (l[1].0-l[0].0).abs() <= 2.*stopped_at)
        .map(|l| (0.5*(l[0].0+l[1].0),0.5*(l[0].1+l[1].1),l[0].2));
    if let Some(q) = far_apex { let first = out[0]; out.push(q); out.push(first) }
    if toward < 0. { out.reverse() }
    // the swallowtail points after a marker
    out.push((f64::NAN,f64::NAN,f64::NAN));
    out.push(ap);
    out.extend(far_apex);
    if ring_debug() {
        eprintln!("ring: a double curve of {} points from {:?} to {:?}",out.len(),out.first(),out.last());
    }
    Ok(out)
}

/// **The double curves cut into a stretch's fit** (`Fitted::doubles`): each curve's points within the
/// stretch on each tube, in that tube's `(u, t)` — those whose root is the tube's curve's at their
/// angle, its point within `tol` of the tube's fitted net — with where it crosses the stretch's ends.
pub fn cut_along(ring: &Ring,path: &Trajectory,whole: [f64;2],fit: &mut Fitted,scale: f64,curves: &[Vec<(f64,f64,f64)>],tol: f64) -> Result<(),Unbuilt> {
    let [a,b] = fit.roll;
    let reader = Reader::of(ring,path,whole,[a,b],scale,fit.unborn)?;
    let runs = runs(&reader.first);
    for (ri,run) in runs.iter().enumerate() {
        let per = LEAST_CLOSED.div_ceil(run.pieces.len()).max(PER);
        let faces: Vec<Face> = run.pieces.iter().enumerate().flat_map(|(i,&k)| {
            let of = if reader.first[run.curve][k].side == Side::Fold { 1 } else { per };
            (0..of).map(move |share| Face {piece:i,share,of})
        }).collect();
        // `(θ, t, σ)`'s `u` on this tube, where its root there is the tube's: the face whose seams'
        // angles at `t` hold `θ` — each seam's point of the fitted net read round the ring's axis in
        // the tool (every point the fit is made of stands in its meridian's half-plane, a dipped
        // one moved along the tool's normal, which lies in it), the face's share of its piece linear
        // in `θ` as the fit placed it. One instant read per point of a double curve, to find the
        // joins of the piece holding it, was 70% of the turned torus's build (#942 F4: 19 314
        // instants, 74 s of 106)
        let net = &fit.nets[ri];
        let n = faces.len();
        let angle = |u: f64,t: f64| -> Result<f64,Unbuilt> {
            let q = path.pose(t)?.inverse().point(net.point(u,t));
            let l = ring.frame.local(q);
            Ok(l[1].atan2(l[0]))
        };
        let u_of = |(th,t,sg): (f64,f64,f64)| -> Result<Option<f64>,Unbuilt> {
            // the seams' angles at `t`, each run on from the one before round the loop
            let mut seams = Vec::with_capacity(n+1);
            for k in 0..=n {
                let mut x = angle(k as f64,t)?;
                if let Some(&last) = seams.last() { while x < last { x += TAU } }
                seams.push(x);
            }
            let th = seams[0]+(th-seams[0]).rem_euclid(TAU);
            let Some(k) = (0..n).find(|&k| th >= seams[k] && th <= seams[k+1]) else { return Ok(None) };
            let u = k as f64+(th-seams[k])/(seams[k+1]-seams[k]).max(1e-300);
            // (the tube's own branch: within the tolerance of its net, which fits it within less,
            // where another stands some half a piece of the meridian away, #942 F6)
            let Some((x,_)) = sheet_at(ring,path,th,t,sg) else { return Ok(None) };
            Ok((crate::util::space::distance(x,net.point(u,t)) <= tol).then_some(u))
        };
        // (each line's `u` unwrapped along it as it is laid: a line crossing the tube's seam, read in
        // `[0, n)`, jumps from one end of the tube to the other, #942 F6)
        let laid = |line: &mut Vec<[f64;2]>,u: f64,t: f64| {
            let nf = n as f64;
            let u = match line.last() { Some(&[v,_]) => v+(u-v+0.5*nf).rem_euclid(nf)-0.5*nf,None => u };
            line.push([u,t]);
        };
        let mut out = Vec::new();
        let mut tails = Vec::new();
        for (ci,full) in curves.iter().enumerate() {
            // (the curve, and its swallowtail points after the marker)
            let mark = full.iter().position(|p| p.0.is_nan()).unwrap_or(full.len());
            let curve = &full[..mark];
            let mut line: Vec<[f64;2]> = Vec::new();
            let lines = out.len();
            // (a branch a bridge's end on this tube gives way to is the face's seam: laid as no chord)
            let seam = |w: usize| fit.ways.iter().any(|y| y.curve == run.curve && y.of.0 == ci && w >= y.of.1[0] && w < y.of.1[1]);
            let yielded = fit.ways.iter().any(|y| y.curve == run.curve && y.of.0 == ci);
            for w in 0..curve.len() {
                let (th,t,sg) = curve[w];
                let inside = t >= a && t <= b && !seam(w);
                // (where it crosses an end of the stretch: between this point and the one before; going
                // out, the line ends there, and coming back in a new one begins)
                if w > 0 && !seam(w) && !seam(w-1) {
                    let (th0,t0,sg0) = curve[w-1];
                    for edge in [a,b] {
                        if (t0-edge)*(t-edge) < 0. {
                            let f = (edge-t0)/(t-t0);
                            if let Some(u) = u_of((th0+(th-th0)*f,edge,if f < 0.5 { sg0 } else { sg }))? { laid(&mut line,u,edge) }
                            if !inside && line.len() >= 2 { out.push(std::mem::take(&mut line)) } else if !inside { line.clear() }
                        }
                    }
                }
                if inside { if let Some(u) = u_of((th,t,sg))? { laid(&mut line,u,t) } }
                // (and any point past an end ends the line, though the one before stood on that end:
                // a swallowtail point on its row is such a point, and a preimage leaving there and
                // coming back across the row further on was joined to it along the row, the double
                // curve laid on the row's own edge — the thin torus's second preimage, 0.0316 of an
                // angle along, #942 T1d-tori)
                else if line.len() >= 2 { out.push(std::mem::take(&mut line)) } else { line.clear() }
            }
            if line.len() >= 2 { out.push(line) }
            // (the whole curve where any of it is cut into this tube here)
            if out.len() > lines || yielded { tails.push(curve.iter().map(|p| [p.0,p.1]).collect()) }
        }
        if ring_debug() {
            for l in &out { eprintln!("ring: cut over {:?}, tube {ri}: a line of {} from {:?} to {:?}",[a,b],l.len(),l[0],l[l.len()-1]) }
        }
        fit.doubles[ri] = out;
        fit.tails[ri] = tails;
    }
    Ok(())
}

/// **Where a bridge's end gives way to a double curve** (#942 T1d-tori): a bridge's end is a
/// construction line — where the contact's `s''` leaves the band a fold is read in, at the depth of
/// the pair's least — and its double curve is geometry, so where a double curve's preimage comes
/// within the arrangement's band of an end (`tolerant::ON_CURVE` tolerances, within which the
/// arrangement takes two curves for one), the end is not put there: over the stretch it is the
/// preimage itself (`Way`, read row by row by `Reader::read`), the face before the bridge ending on
/// the double curve and the bridge beginning there — one seam by construction, laid as no chord
/// (`cut_along`). A preimage beside its swallowtail point starts on the fold (`s''` nought) and
/// leaves the bridge as `s''` climbs past its least's depth: the thin torus's first double curve's
/// first preimage reads `s''` at 0.08 of that depth by its point, crosses it 0.036 of the roll
/// above, and stands at three times it where the roll ends, the swallowtail's normal form's 2
/// between — and where it crosses, the sheet's rates all but one way beside the pair, it runs within
/// 8.8e-7 of the end in space (three tolerances) over 9.9e-4 of its length. Each branch of each
/// curve `curves` (its points between its swallowtail points, `double_curve`'s marker) is read over
/// the stretch `fit`: its points within the stretch one run whose roll only climbs or only falls
/// (a graph over the roll there, as the end is), beside the bridge's end nearest its middle point,
/// each point's distance from that end's polyline (read at the points' own rolls) its nearness. A
/// branch within the band of its end but not spanning the stretch's rolls cannot be the end over
/// the whole stretch and gives no way (said where `SOLVENT_RING_DEBUG` is set); a branch that gives
/// way in one stretch is read in every stretch it spans beside the same end, the end continuous
/// across the rows they are sewn along.
///
/// `giving` is the stretch's reader and bridges (`Giving::new`) and `points` each curve's points in
/// space (`curve_points`), each read once for every pass.
pub fn give_way(giving: &mut Giving,fit: &Fitted,curves: &[Vec<(f64,f64,f64)>],points: &std::cell::OnceCell<Vec<Vec<Option<V>>>>,
    given: &[(usize,usize)],tol: f64) -> Result<Vec<Way>,Unbuilt> {
    use crate::util::space::{distance,segment_distance};
    let [a,b] = fit.roll;
    let Giving {reader,bridges,near: nearness} = giving;
    let (reader,bridges) = (&*reader,&*bridges);
    let ring = reader.ring;
    if bridges.is_empty() { return Ok(Vec::new()) }
    let points = points.get_or_init(|| curve_points(ring,reader.path,curves));
    let band = crate::tolerant::ON_CURVE*tol;
    let debug = ring_debug();
    // a bridge's end in space at the roll `t`
    let end = |(c,j): (usize,usize),side: usize,t: f64| -> Result<Option<V>,Unbuilt> {
        let r = reader.read(t)?;
        let p = r.curves[c][j];
        let th = if side == 0 { p.from } else { p.to };
        Ok(ring.root(&r.instant.w,th.rem_euclid(TAU),p.label,false).map(|q| r.pose.point(q.x)))
    };
    let mut ways: Vec<Way> = Vec::new();
    for (ci,full) in curves.iter().enumerate() {
        let mark = full.iter().position(|p| p.0.is_nan()).unwrap_or(full.len());
        let curve = &full[..mark];
        if curve.len() < 2 { continue }
        // (its branches: the curve parted at its swallowtail points)
        let mut parts: Vec<usize> = full[(mark+1).min(full.len())..].iter().filter_map(|ap| curve.iter().position(|p| p.0 == ap.0 && p.1 == ap.1)).collect();
        parts.sort_unstable();
        let bounds: Vec<usize> = std::iter::once(0).chain(parts).chain(std::iter::once(curve.len()-1)).collect();
        let points = &points[ci];
        for w in bounds.windows(2) {
            let (lo,hi) = (w[0],w[1]);
            if hi <= lo { continue }
            let inside: Vec<usize> = (lo..=hi).filter(|&k| curve[k].1 >= a && curve[k].1 <= b).collect();
            let (Some(&first),Some(&last)) = (inside.first(),inside.last()) else { continue };
            if last-first+1 != inside.len() { continue }
            // (bracketed by its points either side of the stretch)
            let (from,to) = (first.saturating_sub(1).max(lo),(last+1).min(hi));
            let run = &curve[from..=to];
            let (rising,falling) = (run.windows(2).all(|p| p[1].1 > p[0].1),run.windows(2).all(|p| p[1].1 < p[0].1));
            if !(rising || falling) { continue }
            let (t_lo,t_hi) = (run[0].1.min(run[run.len()-1].1),run[0].1.max(run[run.len()-1].1));
            // the bridge's end nearest its middle point
            let m = inside[inside.len()/2];
            let Some(xm) = points[m] else { continue };
            let mut best: Option<(f64,(usize,usize),usize)> = None;
            for &br in bridges { for side in 0..2 {
                if let Some(y) = end(br,side,curve[m].1)? { let d = distance(xm,y); if best.is_none_or(|e| d < e.0) { best = Some((d,br,side)) } }
            } }
            let Some((_,br,side)) = best else { continue };
            let known = given.contains(&(ci,lo));
            if !known {
                // how near the end it comes: each point's distance from the end's polyline (no
                // branch given changes it: read in the first pass, kept for the next)
                let near = match nearness.get(&(ci,lo)) {
                    Some(&n) => n,
                    None => {
                        let ys: Vec<V> = inside.iter().map(|&k| end(br,side,curve[k].1)).collect::<Result<Vec<_>,_>>()?.into_iter().flatten().collect();
                        let n = inside.iter().filter_map(|&k| points[k])
                            .map(|x| ys.windows(2).map(|s| segment_distance(x,s[0],s[1])).fold(f64::INFINITY,f64::min)).fold(f64::INFINITY,f64::min);
                        nearness.insert((ci,lo),n);
                        n
                    }
                };
                if debug { eprintln!("ring: curve {ci}'s branch {lo}..{hi} over {:?} beside bridge {br:?}'s end {side}: {near:.3e} from it at its nearest",[a,b]) }
                if !(near <= band) { continue }
            }
            if !(t_lo <= a && t_hi >= b) {
                if debug { eprintln!("ring: curve {ci}'s branch {lo}..{hi} within the band of bridge {br:?}'s end {side} but over {:?} of {:?}: no way given",[t_lo,t_hi],[a,b]) }
                continue
            }
            if ways.iter().any(|y| (y.curve,y.piece,y.side) == (br.0,br.1,side)) { continue }
            let mut order: Vec<usize> = (from..=to).collect();
            if falling { order.reverse() }
            ways.push(Way::new(br.0,br.1,side,order.iter().map(|&k| curve[k]).collect(),(ci,[from,to+1])));
            if debug { eprintln!("ring: curve {ci}'s branch {lo}..{hi} gives bridge {br:?}'s end {side} way over {:?}",[a,b]) }
        }
    }
    Ok(ways)
}

/// **A stretch's reader and its bridges, for `give_way`** (#942 T1d-tori): built once and read in
/// each of its passes (`closed::ring_fitted` reads every stretch, then again with the branches that
/// gave way anywhere) — each roll's instant read once, and each branch's nearness to the end beside
/// it, which no branch given changes, once.
pub struct Giving<'a> { reader: Reader<'a>,bridges: Vec<(usize,usize)>,near: std::collections::BTreeMap<(usize,usize),f64> }

impl<'a> Giving<'a> {
    /// The reader `fit` is read by, and its bridges: each narrow pair of its first instant.
    pub fn new(ring: &'a Ring,path: &'a Trajectory,whole: [f64;2],fit: &Fitted,scale: f64) -> Result<Giving<'a>,Unbuilt> {
        let reader = Reader::of(ring,path,whole,fit.roll,scale,fit.unborn)?;
        let bridges: Vec<(usize,usize)> = runs(&reader.first).iter()
            .flat_map(|r| r.pieces.iter().filter(|&&j| narrow(&reader.first[r.curve],j)).map(move |&j| (r.curve,j))).collect();
        Ok(Giving {reader,bridges,near:Default::default()})
    }
}

/// Each double curve's points in space (`rings::double_curve`'s, up to its marker), on the sheet
/// its place round the meridian names (`give_way`'s).
pub fn curve_points(ring: &Ring,path: &Trajectory,curves: &[Vec<(f64,f64,f64)>]) -> Vec<Vec<Option<V>>> {
    curves.iter().map(|full| {
        let mark = full.iter().position(|p| p.0.is_nan()).unwrap_or(full.len());
        full[..mark].iter().map(|&(th,t,sg)| sheet_at(ring,path,th,t,sg).map(|x| x.0)).collect()
    }).collect()
}

/// **A sheet with its seams first** (#942 T1d-tori): each edge every use of which is a straight
/// line in its face's parameters at one `u` — a seam between two of a tube's faces, run along the
/// roll — before the rest, in order otherwise (the rows keep theirs). Where the arrangement takes two
/// of the sheet's edges for one it keeps the first (`boolean::merged`) and reads every other use as
/// the surface's inverse of it; a bridge's end given way to a double curve (`give_way`) is such a
/// seam, met by the chord the double curve is laid as on the other leaf, and the face given way ends
/// on the fold at the swallowtail point, where that inverse is ill posed (the thin torus's foot found
/// 1.2e-5 off on the leaf folded back, at an input one ulp off the bench's, and its rate in `v` read
/// −0.997 against its feet's −5e-6). So kept, the seam stays the face's own line, and the chord's
/// leaf, clear of the fold, takes the inverse. (All of a sheet's lines put first, the rows' pieces
/// beside the swallowtail points, sewn across no row there, were kept the other way round as well,
/// and those read inverse on the fold instead.)
pub fn seams_first(mut b: Brep) -> Brep {
    use crate::topo::Pcurve;
    let mut line = vec![true;b.edges.len()];
    for c in b.faces.iter().flat_map(|f| f.loops.iter().flatten()) {
        if !matches!(c.pcurve,Pcurve::Line {a,b} if a[0] == b[0]) { line[c.edge as usize] = false }
    }
    let order: Vec<usize> = (0..b.edges.len()).filter(|&e| line[e]).chain((0..b.edges.len()).filter(|&e| !line[e])).collect();
    let mut to = vec![0u32;b.edges.len()];
    for (k,&e) in order.iter().enumerate() { to[e] = k as u32 }
    b.edges = order.iter().map(|&e| b.edges[e].clone()).collect();
    for c in b.faces.iter_mut().flat_map(|f| f.loops.iter_mut().flatten()) { c.edge = to[c.edge as usize] }
    b
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
    entered_at(fit,ring,[Some((ends[0],poses[0])),Some((ends[1],poses[1]))],[true,true],&|_,_| false,tol)
}

/// `entered`, where an end of the fit's stretch of the roll may be no end of the roll (`rimmed`
/// false): a cut where the contact changes shape (`changes`, #880 T1d), the tube there running on into
/// the next stretch's along its rim, which lies on no tool. Such a stretch is still cut where it
/// crosses the tool at either end pose given (`at`), read plainly (no rim of it touches that tool): a
/// tube carried on past where the roll's end pose stands — the thin torus, its double curve ending
/// where one leaf's partner reaches the end of the roll, on that pose's tool, partway along another
/// stretch.
pub fn entered_at(fit: &Fitted,ring: &Ring,at: [Option<(&Brep,Pose)>;2],rimmed: [bool;2],covered: &(dyn Fn(V,f64) -> bool+Sync),tol: f64)
    -> Result<(Vec<Brep>,Vec<Given>),Unbuilt> {
    let mut sheets = Vec::new();
    let mut surfaces: Vec<Vec<crate::geom::Surface>> = Vec::new();
    let mut given = Vec::new();
    let n = ring.elements.len();
    // each end pose carrying a point in space back into the tool where it stands there
    let backs = at.map(|x| x.map(|(_,p)| p.inverse()));
    for ((((net,on),sides),doubles),tails) in fit.nets.iter().zip(&fit.on).zip(&fit.sides).zip(&fit.doubles).zip(&fit.tails) {
        let pieces = on.len();
        let [a,b] = fit.roll;
        let y_of = |t: f64| (t-a)/(b-a);
        // each double curve's preimages cut at the seams between faces: per face its chords
        let mut cuts_of: Vec<Vec<Vec<P2>>> = vec![Vec::new();pieces];
        let np = pieces as f64;
        for un in doubles {
            // (each step cut at every seam it crosses, its `u` continuous along it)
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
        // whether `(u, t)` is in a narrow pair's tail: where an instant other than its own covers
        // the place (`covered`, the depth-based trim's reading, #942 T1d-tori). Read as above the
        // double curve over the angle round the axis its first preimage is stepped by, the thin
        // torus's curve joined to its swallowtail point, whose first preimage's angle turns back
        // within 2e-4 of the point, held a piece beside it as tail that only its own instant
        // touches; and the full turn's places above the curve where it dips below the stretch and
        // comes back stood on the boundary to 1e-8, past the curve's end on an end pose's tool
        let tail = |u: f64,t: f64| -> bool { !tails.is_empty() && covered(net.point(u,t),t) };
        let holding: Vec<Vec<usize>> = on.iter().map(|&o| match o {
            On::Face(e) => vec![ring.faces[e]],
            On::Edge(e) => vec![ring.faces[e],ring.faces[(e+1)%n]],
        }).collect();
        let faces: Vec<Trimmed> = crate::util::par::indices(pieces,|k| {
            // a touching piece's rim touches the tool, a dipped stretch's crosses it
            let order = if sides[k] == Side::Touching { 2 } else { 1 };
            let readings: [Option<Reading>;2] = std::array::from_fn(|e| backs[e].map(|back| Reading {net,ring,back,tol,u:[k as f64,k as f64+1.],
                roll:fit.roll,end:if rimmed[e] { e as f64 } else { f64::NAN },order:if rimmed[e] { order } else { 0 }}));
            let mut chords: Vec<(usize,Vec<P2>)> = Vec::new();
            // (a trace walked from both its ends — its far end matched to no rim crossing, the other
            // traced back along it — is one chord: two along one another split each other into a
            // thousand pieces, #880 T1d M2's ring beside its whole-touched meridian)
            // (where a curve cut into the face ends inside it on the tool — its corner where the
            // covering instant reaches the roll's end, on that pose — a seed of a closed trace there,
            // read within the tolerance of the tool)
            let seeds: [Vec<P2>;2] = std::array::from_fn(|e| readings[e].as_ref().map_or(Vec::new(),|r| cuts_of[k].iter()
                .flat_map(|c| [c[0],c[c.len()-1]]).filter(|p| p[0] > ON_SIDE && p[0] < 1.-ON_SIDE && p[1] > ON_SIDE && p[1] < 1.-ON_SIDE)
                .filter(|p| r.f(p[0],p[1]).abs() <= tol).collect()));
            for (e,c) in (0..2).flat_map(|e| readings[e].as_ref().map_or(Vec::new(),|r| traces(r,&seeds[e])).into_iter().map(move |c| (e,c))) {
                // (a trace of no length — all its points within `ON_SIDE` of its first, one place on
                // the face's square — bounds nothing: the tool touching the face there, not crossing
                // it; the near-orthogonal tori's, two points 2.4e-13 apart on the rim where a crossing
                // of the face ends, #942 F6. A trace along a side is kept: a rim lying on the tool)
                if no_length(&c) { continue }
                // (its ends within the trace's step of the other's: a trace's end is placed to it)
                let near = |p: P2,q: P2| (p[0]-q[0]).hypot(p[1]-q[1]) <= MET_STEP;
                let mid = c[c.len()/2];
                let again = chords.iter().any(|(f,d)| *f == e && (near(d[0],c[0]) && near(d[d.len()-1],c[c.len()-1]) || near(d[0],c[c.len()-1]) && near(d[d.len()-1],c[0]))
                    && d.iter().any(|&p| (p[0]-mid[0]).hypot(p[1]-mid[1]) <= 4.*MET_STEP));
                if !again { chords.push((e,c)) }
            }
            chords.extend(cuts_of[k].iter().map(|c| (2,c.clone())));
            let tail: Box<dyn Fn(P2) -> bool+Sync+Send+'_> = Box::new(move |[x,y]: P2| tail(k as f64+x,a+(b-a)*y));
            Trimmed {readings,chords,tail}
        });
        let (sheet,cut) = trimmed(net,fit.roll,faces,tol)?;
        surfaces.push(cut.surfaces.clone());
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
                    given.push(Given {surfaces:[face.surface.clone(),surface.clone()],curves,touches:Vec::new()});
                }
            }
        }
        sheets.push(sheet);
    }
    for strip in &fit.strips {
        let (sheet,g) = strip_entered(fit,ring,strip,&surfaces,at,rimmed)?;
        sheets.push(sheet);
        given.extend(g);
    }
    Ok((sheets,given))
}

/// **A whole-touched meridian's touching half entered** (#942 T1d-ring, `Strip`): one face, its
/// rims where the roll starts and stops — on the ring there, the meridian it touches whole — and
/// its sides the two tubes' seams on the meridian, each the very curve the tube's faces either side
/// of it are bounded by (`trimmed`'s seam curve), so the strip and the tube share it. What each
/// meets is known by construction and given whole: the ring at either end in the rim, each tube's
/// touching face beside its crossing in the seam, the dipped stretch across the sliver beyond in
/// the seam and in its far seam — the far side of the sliver, which the stretch rises back to,
/// lying on the strip within the tolerance — and the touching face beyond that in the far seam:
/// past the fold the contact curve's sheet stands over the strip's edge no farther off it than its
/// fit, and no search between two sheets all but tangent there is asked.
fn strip_entered(fit: &Fitted,ring: &Ring,strip: &Strip,surfaces: &[Vec<crate::geom::Surface>],at: [Option<(&Brep,Pose)>;2],rimmed: [bool;2])
    -> Result<(Brep,Vec<Given>),Unbuilt> {
    use crate::topo::{Coedge,EdgeCurve,Face,Pcurve};
    if rimmed != [true,true] { return Err(Unbuilt::Failed("a ring's whole-touched meridian over a stretch cut where the contact changes \
        shape: not yet entered".into())) }
    let [a,b] = fit.roll;
    let seam = |i: usize,u: usize| -> Curve {
        let pieces = fit.on[i].len();
        let s = u%pieces;
        Curve::BSpline(Arc::new(fit.nets[i].segment([s as f64,s as f64+1.],fit.roll).row(0)))
    };
    let sides = strip.sides.map(|(i,c)| seam(i,c.seam));
    let net = &strip.net;
    let ends = [a,b];
    let rims = [Curve::BSpline(Arc::new(net.column(0))),Curve::BSpline(Arc::new(net.column(net.poles[0].len()-1)))];
    let frame = Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]);
    let surface = crate::geom::Surface::BSpline(frame,Arc::new(net.clone()));
    let poses: [Pose;2] = std::array::from_fn(|e| at[e].map(|x| x.1).unwrap());
    // **where a point stands against the strip's rim at each end** (in its meridian's half-plane
    // there): its angle about the ring's axis at that end pose, against the rim's, on the side the
    // strip runs on into the roll (its rate along the roll, across the half-plane)
    let angle = |e: usize,x: V| { let l = ring.frame.local(poses[e].inverse().point(x)); l[1].datan2(l[0]) };
    let wrap = |x: f64| (x+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI;
    let star: [f64;2] = std::array::from_fn(|e| angle(e,net.point(0.5,ends[e])));
    let into: [f64;2] = std::array::from_fn(|e| {
        let (_,_,dt) = net.d1(0.5,ends[e]);
        let (_,et,_) = ring.axes(star[e]);
        dot(poses[e].vector(et),dt)*if e == 0 { 1. } else { -1. }
    });
    let inside = |e: usize,x: V| wrap(angle(e,x)-star[e])*into[e] > 0.;
    // each side's far seam, the stretch of it over the strip, and where it crosses a rim: past the
    // fold the contact's sheet lies over the strip's edge a little along the roll from the crossing,
    // so at one end the far seam leaves the strip across its rim, and at the other it ends on it,
    // where the dipped stretch's rim on the ring there closes the strip's edge off
    struct Far { curve: Curve,crosses: [bool;2],fin: usize,touching: usize,beyond: usize }
    let fars: Vec<Far> = strip.sides.iter().map(|&(i,c)| {
        let pieces = fit.on[i].len();
        let Curve::BSpline(whole) = seam(i,c.far) else { unreachable!() };
        let (fin,touching,beyond) = if c.far == (c.seam+1)%pieces { (c.seam,(c.seam+pieces-1)%pieces,c.far) }
            else { ((c.seam+pieces-1)%pieces,c.seam,(c.far+pieces-1)%pieces) };
        let mut span = [a,b];
        let mut crosses = [false;2];
        for e in 0..2 {
            if inside(e,whole.point(ends[e])) { continue }
            // (halved to the rounding between the end and the middle of the roll)
            let mid = 0.5*(a+b);
            if !inside(e,whole.point(mid)) { return Err(Unbuilt::Failed("a ring's whole-touched meridian's strip holds no stretch of a \
                tube's far seam".into())) }
            let (_,hi) = crate::util::roots::bisect(ends[e],mid,|l,h| 0.5*(l+h),|l,h| { let m = 0.5*(l+h); m != l && m != h },
                |m| !inside(e,whole.point(m)));
            span[e] = hi;
            crosses[e] = true;
        }
        let curve = Curve::BSpline(Arc::new(if span == [a,b] { (*whole).clone() } else { whole.segment(span) }));
        Ok(Far {curve,crosses,fin,touching,beyond})
    }).collect::<Result<_,_>>()?;
    let mut out = Brep::default();
    // the corners: each side's ends, the tube's own
    let corner = |e: usize,t: f64| { let (i,c) = strip.sides[e]; fit.nets[i].point(c.seam as f64,t) };
    let v = [[corner(0,a),corner(1,a)],[corner(0,b),corner(1,b)]].map(|r| r.map(|x| out.vertex(x)));
    let mut lp = Vec::new();
    for e in 0..2 {
        // the rim at the start forward, at the stop back (where a far seam crosses it, 3e-5 off the
        // corner along a rim the strip and the ring touch along, the arrangement's reach there takes
        // the two for one place: the rim is not cut there)
        let places: Vec<(f64,u32)> = vec![(0.,v[e][0]),(1.,v[e][1])];
        let t = ends[e];
        let mut rim: Vec<Coedge> = places.windows(2).map(|w| {
            let edge = out.edge(EdgeCurve::Curve(rims[e].clone()),[w[0].0,w[1].0],[w[0].1,w[1].1]);
            Coedge {edge,reversed:e == 1,pcurve:Pcurve::Line {a:[w[0].0,t],b:[w[1].0,t]}}
        }).collect();
        if e == 1 { rim.reverse() }
        lp.extend(rim);
        if e == 0 {
            let side1 = out.edge(EdgeCurve::Curve(sides[1].clone()),[a,b],[v[0][1],v[1][1]]);
            lp.push(Coedge {edge:side1,reversed:false,pcurve:Pcurve::Line {a:[1.,a],b:[1.,b]}});
        }
    }
    let side0 = out.edge(EdgeCurve::Curve(sides[0].clone()),[a,b],[v[0][0],v[1][0]]);
    lp.push(Coedge {edge:side0,reversed:true,pcurve:Pcurve::Line {a:[0.,a],b:[0.,b]}});
    out.faces.push(Face {surface:surface.clone(),reversed:false,loops:vec![lp],name:"sheet".into()});
    let mut given = Vec::new();
    for (e,(pose,_)) in at.iter().enumerate().filter_map(|(e,x)| x.map(|x| (e,x))) {
        for (f,face) in pose.faces.iter().enumerate() {
            let mut curves = Vec::new();
            if ring.faces[strip.arc] == f {
                curves.push(rims[e].clone());
                // (where a far seam ends on the strip, the dipped stretch's rim there, on the ring,
                // over the strip within the tolerance from the crossing out to the far seam's end)
                for (k,far) in fars.iter().enumerate() {
                    if far.crosses[e] { continue }
                    let i = strip.sides[k].0;
                    let fin = fit.nets[i].segment([far.fin as f64,far.fin as f64+1.],fit.roll);
                    curves.push(Curve::BSpline(Arc::new(if e == 0 { fin.column(0) } else { fin.column(fin.poles[0].len()-1) })));
                }
            }
            given.push(Given {surfaces:[face.surface.clone(),surface.clone()],curves,touches:Vec::new()});
        }
    }
    for (e,(&(i,_),far)) in strip.sides.iter().zip(&fars).enumerate() {
        let pieces = fit.on[i].len();
        // (every other face of the tube meets the strip nowhere: off its crossing the tube stands on
        // the meridian's buried side, past the strip's edge)
        for k in 0..pieces {
            let curves = if k == far.fin { vec![sides[e].clone(),far.curve.clone()] } else if k == far.touching { vec![sides[e].clone()] }
                else if k == far.beyond { vec![far.curve.clone()] } else { Vec::new() };
            given.push(Given {surfaces:[surface.clone(),surfaces[i][k].clone()],curves,touches:Vec::new()});
        }
    }
    Ok((out.compacted(),given))
}

/// How many points along each rim of a face its meeting with the tool is first bracketed at
/// (`traces`), crowded toward the ends of the roll; the step a trace of it takes across the face, as
/// a part of the face's domain each way; and the most steps it takes.
const MET_RIM: usize = 512;
const MET_STEP: f64 = 2e-3;
/// How near the rim at its end `G` is not divided but carried on from just off it (#942 F6): where
/// what its division by `(y − e)^order` leaves to the rounding, `ε / d^order`, is what carrying it
/// on straight from `d` and `2d` leaves of its curvature, `d²` — `ε^(1/(order + 2))`, a touching
/// rim's (`order` 2) `ε^¼` and a crossing one's `ε^⅓`, the face's roll its unit.
fn g_carried(order: i32) -> f64 { f64::EPSILON.powf(1./(order as f64+2.)) }
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
struct Reading<'a> { net: &'a Net,ring: &'a Ring,back: Pose,tol: f64,u: [f64;2],roll: [f64;2],end: f64,order: i32 }

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
        let near = g_carried(self.order);
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
/// used once. A closed curve inside the face, crossing no rim, is read only through a place of it
/// the caller knows (`seeds`: where a curve cut into the face ends inside it on the tool, #942 the
/// full turn — the end torus's trace across the tube in the middle of the roll, where the tube's
/// crossing with itself ends), settled onto the zero set and traced round until it closes on it;
/// any other is not read (the judge sees what it misses). Each curve's points in the normalised
/// domain, from one rim to another, or round from a seed back to it.
fn traces(r: &Reading,seeds: &[P2]) -> Vec<Vec<P2>> {
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
            if (vs[i] > 0.) != (vs[i+1] > 0.) {
                let p = settle(rim(ss[i]),rim(ss[i+1]));
                // **a seam's crossing beside the rim the tool stands on, the seam between the two on
                // the tool within the tolerance, is the corner** (#942 T1d-ring): there the tube and
                // the tool are one within the tolerance all the way, and where along it `G` changes
                // sign is the fit's error, not the meeting's — beside a corner where a touching
                // piece's rim meets a dipped one's, the fit's slope across a rim, 2.7e-5, put a
                // crossing 4e-4 of the roll up the seam, and the chord from it zigzagged off the
                // tool beside a sliver face. The trace reaching it runs into the tube's own vertex
                // there (below), as it does where the face's corner is nearer.
                if k%2 == 1 && r.order > 0 && ss.iter().zip(&vs).filter(|&(&s,_)| (s-ye).abs() <= (p[1]-ye).abs())
                    .all(|(&s,_)| { let q = rim(s); r.f(q[0],q[1]).abs() <= r.tol }) { continue }
                ends.push((p,INWARD[k]))
            }
        }
    }
    let inside = |p: [f64;2]| p[0] >= 0. && p[0] <= 1. && p[1] >= 0. && p[1] <= 1.;
    let apart = |p: [f64;2],q: [f64;2]| (p[0]-q[0]).hypot(p[1]-q[1]);
    let debug = ring_debug();
    if debug { eprintln!("met: face {:?} end {}: rim crossings {:?}",r.u,r.end,ends.iter().map(|e| e.0).collect::<Vec<_>>()) }
    let mut used = vec![false;ends.len()];
    let mut out: Vec<Vec<P2>> = Vec::new();
    let tangent = |p: [f64;2]| { let d = grad(p[0],p[1]); let l = d[0].hypot(d[1]); [-d[1]/l,d[0]/l] };
    for start in 0..ends.len()+seeds.len() {
        // (past the rims' crossings, each seed inside the face: settled onto the zero set, unless a
        // curve traced already passes within a step of it)
        let seeded = start >= ends.len();
        let (p0,into) = if seeded {
            let Some(p) = settled(r,seeds[start-ends.len()]) else { continue };
            if out.iter().flatten().any(|&q| apart(p,q) <= MET_STEP) { continue }
            (p,tangent(p))
        } else {
            if used[start] { continue }
            used[start] = true;
            ends[start]
        };
        let mut pts = vec![p0];
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
                if debug { eprintln!("met:   leaves at {c:?}, last {p:?}") }
                let near = (0..ends.len()).filter(|&k| !used[k] && apart(ends[k].0,c) <= 4.*MET_STEP)
                    .min_by(|&i,&j| apart(ends[i].0,c).total_cmp(&apart(ends[j].0,c)));
                // (unless a crossing of the rim is nearer: beside a meridian an instant touches whole
                // two crossings stand a step from the vertex, #880 T1d M2)
                if apart(corner,c) <= 4.*MET_STEP && (corner[1]-ye).abs() < 0.5
                    && near.is_none_or(|k| apart(corner,c) < apart(ends[k].0,c)) {
                    pts.push(corner);
                    break
                }
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
            // back where it began: a loop through the rim at one point is not read; a loop from a
            // seed inside the face is the curve, closed on its first point (within the step ahead)
            if seeded && pts.len() > 4 && apart(p,p0) <= MET_STEP && (p0[0]-p[0])*dir[0]+(p0[1]-p[1])*dir[1] >= 0. {
                pts.push(p0);
                closed_at = Some(());
                break
            }
            if !seeded && pts.len() > 4 && apart(p,p0) < 0.5*MET_STEP { closed_at = Some(()); break }
        }
        if debug { eprintln!("met:   traced {} points from {:?} to {:?}{}",pts.len(),pts[0],pts[pts.len()-1],if seeded { ", seeded" } else { "" }) }
        // (a seed's curve that leaves the face crosses a rim, and is traced from there)
        if closed_at.is_some() != seeded || pts.len() < 2 { continue }
        // (a point within `G`'s carried reach of the rim settled again on the tool itself, across
        // the face at its own roll: `G` there is drawn on from just off the rim)
        for p in pts.iter_mut() { *p = polished(r,*p) }
        out.push(pts);
    }
    out
}

/// **A trace of a meeting in space**: the face's points along it (`pts`, in its normalised domain;
/// on the face exactly and on the tool to rounding), interpolated — a cubic through every so many of
/// them, the coarsest within a quarter of `tol` of all — and those points in space; and the places
/// on the face the curve passes, each at its own parameter of the curve (#942 F3): the traced
/// points, and the meeting's own point settled halfway between each two, where the curve passes it
/// nearest — its pcurve, by construction.
fn chord_curve(r: &Reading,pts: &[P2],tol: f64) -> Result<(Curve,Vec<V>,Vec<(f64,P2)>),Unbuilt> {
    // (and between each two of them, where the cubic strays from the meeting's own point halfway
    // — settled on the tool across the face — by more than a quarter of `tol`, that point too: a
    // trace of three points into a vertex beside a meridian the end pose touches whole bowed 6e-8
    // off the tool between them, #880 T1d M2)
    let mut pts = pts.to_vec();
    for _ in 0..8 {
        let xs: Vec<V> = pts.iter().map(|&[x,y]| r.at(x,y).0).collect();
        let (curve,xs) = curve_through(xs,tol)?;
        let mut params = vec![0.];
        for w in xs.windows(2) { params.push(params.last().unwrap()+norm(crate::util::space::sub(w[1],w[0])).max(1e-300)) }
        let length = *params.last().unwrap();
        let mut more: Vec<(usize,P2)> = Vec::new();
        let mut on: Vec<(f64,P2)> = vec![(0.,pts[0])];
        for i in 0..pts.len()-1 {
            let half = [0.5*(pts[i][0]+pts[i+1][0]),0.5*(pts[i][1]+pts[i+1][1])];
            // (a settling that ran a quarter of the way between the two off the middle met another
            // branch of the zero set, not this one)
            let Some(m) = settled(r,half).filter(|m| (m[0]-half[0]).hypot(m[1]-half[1])
                <= 0.25*(pts[i][0]-pts[i+1][0]).hypot(pts[i][1]-pts[i+1][1])) else { on.push((params[i+1]/length,pts[i+1])); continue };
            let x = r.at(m[0],m[1]).0;
            // (the cubic's nearest point to it between the two, by golden sections)
            let off = |t: f64| norm(crate::util::space::sub(curve.point(t),x));
            let (mut lo,mut hi) = (params[i]/length,params[i+1]/length);
            for _ in 0..60 {
                let (m1,m2) = (hi-(hi-lo)*0.618_033_988_749_895,lo+(hi-lo)*0.618_033_988_749_895);
                if off(m1) <= off(m2) { hi = m2 } else { lo = m1 }
            }
            if off(0.5*(lo+hi)) > 0.25*tol { more.push((i+1,m)) }
            on.push((0.5*(lo+hi),m));
            on.push((params[i+1]/length,pts[i+1]));
        }
        if more.is_empty() { return Ok((curve,xs,on)) }
        for (i,m) in more.into_iter().rev() { pts.insert(i,m) }
    }
    let (curve,xs) = curve_through(pts.iter().map(|&[x,y]| r.at(x,y).0).collect(),tol)?;
    let mut params = vec![0.];
    for w in xs.windows(2) { params.push(params.last().unwrap()+norm(crate::util::space::sub(w[1],w[0])).max(1e-300)) }
    let length = *params.last().unwrap();
    Ok((curve,xs,params.iter().map(|p| p/length.max(1e-300)).zip(pts).collect()))
}

/// A place on a face's normalised square moved onto `G`'s zero set by Newton's steps along its
/// gradient (`traces`' own), and `polished`; none where it leaves the square.
fn settled(r: &Reading,p: P2) -> Option<P2> {
    let g = |x: f64,y: f64| r.g(x,y);
    let mut q = p;
    for _ in 0..8 {
        let v = g(q[0],q[1]);
        if v.abs() <= 1e-13 { break }
        let h = 1e-6;
        let d = [(g((q[0]+h).min(1.),q[1])-g((q[0]-h).max(0.),q[1]))/((q[0]+h).min(1.)-(q[0]-h).max(0.)),
            (g(q[0],(q[1]+h).min(1.))-g(q[0],(q[1]-h).max(0.)))/((q[1]+h).min(1.)-(q[1]-h).max(0.))];
        let n2 = d[0]*d[0]+d[1]*d[1];
        if !(n2 > 0.) { break }
        let step = [v*d[0]/n2,v*d[1]/n2];
        q = [q[0]-step[0],q[1]-step[1]];
        if step[0].hypot(step[1]) <= 1e-14 { break }
    }
    (q[0] >= 0. && q[0] <= 1. && q[1] >= 0. && q[1] <= 1.).then(|| polished(r,q))
}

/// A place within `G`'s carried reach of the rim at its end (`G_CARRIED`) settled again on the tool
/// itself, across the face at its own roll — `G` there is drawn on from just off the rim, and a
/// meeting ending at a vertex beside a meridian the end pose touches whole stood off the tool, #880
/// T1d M2 — where the tool's side changes within four steps; any other place as it is.
fn polished(r: &Reading,p: P2) -> P2 {
    let (x0,y) = (p[0],p[1]);
    if r.order == 0 || !(x0 > 0. && x0 < 1.) || !((y-r.end).abs() > 0. && (y-r.end).abs() < g_carried(r.order)) { return p }
    let f = |x: f64| r.f(x,y);
    let f0 = f(x0);
    let mut h = 1e-12;
    while h <= 4.*MET_STEP {
        for side in [-1.,1.] {
            let x1 = (x0+side*h).clamp(0.,1.);
            if (f(x1) > 0.) != (f0 > 0.) {
                let (mut lo,mut hi) = (x0,x1);
                for _ in 0..100 {
                    let m = 0.5*(lo+hi);
                    if m == lo || m == hi { break }
                    if (f(m) > 0.) == (f0 > 0.) { lo = m } else { hi = m }
                }
                return [0.5*(lo+hi),y]
            }
        }
        h *= 4.;
    }
    p
}

/// The cubic through points `xs` by chord length (`chord_curve`'s), and the points.
pub(crate) fn curve_through(xs: Vec<V>,tol: f64) -> Result<(Curve,Vec<V>),Unbuilt> {
    let mut params = vec![0.];
    for w in xs.windows(2) { params.push(params.last().unwrap()+norm(crate::util::space::sub(w[1],w[0])).max(1e-300)) }
    let length = *params.last().unwrap();
    let params: Vec<f64> = params.iter().map(|p| p/length.max(1e-300)).collect();
    curve_through_at(xs,&params,tol)
}

/// `curve_through` at the parameters `params` given each point (increasing, from 0 to 1).
fn curve_through_at(xs: Vec<V>,params: &[f64],tol: f64) -> Result<(Curve,Vec<V>),Unbuilt> {
    let thinned = |stride: usize| -> Option<crate::nurbs::BSpline> {
        let mut keep: Vec<usize> = (0..xs.len()).step_by(stride).collect();
        if *keep.last().unwrap() != xs.len()-1 { keep.push(xs.len()-1) }
        if keep.len() < 2 { return None }
        let c = crate::nurbs::interpolate(&keep.iter().map(|&i| xs[i]).collect::<Vec<_>>(),&keep.iter().map(|&i| params[i]).collect::<Vec<_>>(),3)?;
        xs.iter().zip(params).all(|(&x,&t)| norm(crate::util::space::sub(c.point(t),x)) <= 0.25*tol).then_some(c)
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

/// Whether a run of places on a face's square has no length: every one within `ON_SIDE` of its
/// first, one place — bounding nothing (#942 F6).
fn no_length(c: &[P2]) -> bool { c.iter().all(|&p| (p[0]-c[0][0]).hypot(p[1]-c[0][1]) <= ON_SIDE) }

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
/// crosses it, and at every end of another standing within `SNAP_ON` of it, each piece keeping its
/// chord's kind. A trace's end is placed to its step (`MET_STEP`), so an end within a step of another
/// chord ends on it (#942 F6), put there. **A double curve's end is the tube's own place** (#942
/// T1d-tori, by construction): where one of its preimages reaches the roll's end, on the tool, it
/// ends there on the other's face too, so it stays, and the trace it ends on is cut at it — put on
/// the trace's polyline instead, it stood 1e-6 off the vertex its other preimage ends at on the rim
/// (the thin torus: the touching rim and the crossing beside it two vertices, the rim a slit in the
/// tool's face; the full turn's 2.4e-6 inside the end torus, two tolerances from the same point on
/// the leaf whose rim it is, and the two never met).
fn split_chords(chords: &[(usize,Vec<P2>)]) -> Vec<(usize,Vec<P2>)> {
    const SNAP_ON: f64 = MET_STEP;
    let on_rim = |p: P2| p[0] <= ON_SIDE || p[0] >= 1.-ON_SIDE || p[1] <= ON_SIDE || p[1] >= 1.-ON_SIDE;
    let mut lines: Vec<(usize,Vec<P2>)> = chords.to_vec();
    // each chord's cuts: (segment, fraction along it, place)
    let mut cuts: Vec<Vec<(usize,f64,P2)>> = vec![Vec::new();lines.len()];
    // (each end left where it stands on a trace, and the trace: their crossing beside it is that end)
    let mut kept: Vec<(usize,usize,P2)> = Vec::new();
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
            if let Some((dist,j,k,f,q)) = best { if dist <= SNAP_ON {
                // (a double curve's on a trace: the trace cut at the end itself)
                if lines[i].0 == 2 && lines[j].0 < 2 { cuts[j].push((k,f,p)); kept.push((i,j,p)) }
                else { lines[i].1[e] = q; cuts[j].push((k,f,q)) }
            } }
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
                if kept.iter().any(|&(m,n,p)| ((m,n) == (i,j) || (m,n) == (j,i)) && (p[0]-x[0]).hypot(p[1]-x[1]) <= SNAP_ON) { continue }
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
            // (a cut at a point of the chord's own is that point once: no step of no length, whose
            // way is none)
            let last = piece[piece.len()-1];
            if (pts[k+1][0]-last[0]).hypot(pts[k+1][1]-last[1]) > 1e-12 { piece.push(pts[k+1]) }
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
    // (a side of the square is straight: its way is its own, read off no step along it, #942 F6 —
    // counter-clockwise round the square from `(0, 0)`, the side `⌊s⌋` from `s` on)
    let side_way = |k: i64| [[1.,0.],[0.,1.],[-1.,0.],[0.,-1.]][k.rem_euclid(4) as usize];
    let way = |d: [f64;2]| d[1].atan2(d[0]);
    for (i,(_,c)) in chords.iter().enumerate() {
        let [p,q] = ends[i];
        out[p].push((angle(c[0],c[1]),Walk::Chord(i,true),q));
        out[q].push((angle(c[c.len()-1],c[c.len()-2]),Walk::Chord(i,false),p));
    }
    for w in 0..rim.len() {
        let (p,q) = (rim[w],rim[(w+1)%rim.len()]);
        let (s0,s1) = (perimeter(nodes[p]),if w+1 == rim.len() { 4. } else { perimeter(nodes[q]) });
        // (the way out along the side it runs on from here)
        out[p].push((way(side_way(s0.floor() as i64)),Walk::Side(s0,s1),q));
    }
    // the way back into each place along each way, to turn from
    let back = |w: &Walk| -> Option<f64> {
        match *w {
            // (back along the side it arrives on: the one before a corner it arrives at)
            Walk::Side(_,s1) => { let d = side_way(s1.ceil() as i64-1); Some(way([-d[0],-d[1]])) }
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
                    else {
                        if ring_debug() {
                            eprintln!("ring: a face's piece ends at {:?} with no way on; its chords:",nodes[at]);
                            for (k,c) in chords { eprintln!("ring:   kind {k}, {} points from {:?} to {:?}",c.len(),c[0],c[c.len()-1]) }
                        }
                        return Err(Unbuilt::Failed("a ring's sheet's face's piece ends at a place with no way on".into()))
                    };
                if key(&next) == key(&first) { break }
                (w,at) = (next,to);
            }
            pieces.push(walk);
        }
    }
    Ok(pieces)
}

/// A way round a piece of a face's square as the places it passes, in order: along a side its ends
/// and the corners between, along a chord its points.
fn walk_points(walk: &[Walk],chords: &[(usize,Vec<P2>)]) -> Vec<P2> {
    let mut out = Vec::new();
    for w in walk {
        match *w {
            Walk::Side(s0,s1) => {
                out.push(round_square(s0));
                let mut c = s0.floor()+1.;
                while c < s1 { out.push(round_square(c)); c += 1. }
            }
            Walk::Chord(i,forward) => {
                let c = &chords[i].1;
                if forward { out.extend(c.iter().copied()) } else { out.extend(c.iter().rev().copied()) }
            }
        }
    }
    out
}

/// Whether the place `x` is inside the closed run `pts` (by the crossings of a ray along `+x`).
fn within(pts: &[P2],x: P2) -> bool {
    let mut inside = false;
    for i in 0..pts.len() {
        let (p,q) = (pts[i],pts[(i+1)%pts.len()]);
        if (p[1] > x[1]) != (q[1] > x[1]) && x[0] < p[0]+(q[0]-p[0])*(x[1]-p[1])/(q[1]-p[1]) { inside = !inside }
    }
    inside
}

/// **A face's pieces with their holes** (#942, the full turn): the planar graph's walks
/// (`pieces_among`) go round each piece with it on their left, so a component of chords touching
/// no side of the square — the end torus's closed trace across the tube in the middle of the roll,
/// with the tube's crossing with itself ending on it — is walked once round its outside clockwise:
/// the hole it makes in the piece about it. Each such walk (its enclosed area below nought) is
/// given to the least piece walked counter-clockwise that holds it — larger than the hole (the
/// component's own pieces are no larger together) and holding a place of it — as an inner loop.
fn with_holes(walks: Vec<Vec<Walk>>,chords: &[(usize,Vec<P2>)]) -> Vec<(Vec<Walk>,Vec<Vec<Walk>>)> {
    let pts: Vec<Vec<P2>> = walks.iter().map(|w| walk_points(w,chords)).collect();
    let area: Vec<f64> = pts.iter().map(|p| crate::planar::signed_area(p)).collect();
    let mut out: Vec<(Vec<Walk>,Vec<Vec<Walk>>)> = Vec::new();
    let mut at: Vec<Option<usize>> = vec![None;walks.len()];
    for (i,w) in walks.iter().enumerate() { if area[i] > 0. { at[i] = Some(out.len()); out.push((w.clone(),Vec::new())) } }
    for (i,w) in walks.into_iter().enumerate() {
        if area[i] > 0. { continue }
        // (larger by more than the rounding of the sum of the component's pieces)
        let hole = -area[i];
        let holder = (0..pts.len()).filter(|&j| area[j] > hole*(1.+1e-9) && within(&pts[j],pts[i][0]))
            .min_by(|&x,&y| area[x].total_cmp(&area[y]));
        if let Some(j) = holder { out[at[j].unwrap()].1.push(w) }
    }
    out
}

/// **A face's normalised square cut by chords, as each piece's loops of places** (#942, the full
/// turn): the chords split where they cross or one ends on another (`split_chords`), the planar
/// graph's pieces walked (`pieces_among`) and each component touching no side given as a hole to
/// the piece about it (`with_holes`) — per piece its outer loop, then its holes, each the places it
/// passes. What the ring road's trim cuts a tube's face into, read alone.
pub fn loops_of_square(chords: &[(usize,Vec<P2>)]) -> Result<Vec<Vec<Vec<P2>>>,Unbuilt> {
    let split = split_chords(chords);
    let walks = with_holes(pieces_among(&split)?,&split);
    Ok(walks.iter().map(|(outer,holes)| std::iter::once(outer).chain(holes).map(|w| walk_points(w,&split)).collect()).collect())
}

/// A place just to the left of the longest chord of a piece walked with it on its left (`walk`),
/// beside the chord's middle point: no farther from it than half the way to any other chord's or
/// the square's side, nor than a thousandth of the square; none where the piece walks no chord.
fn beside_chord(walk: &[Walk],chords: &[(usize,Vec<P2>)]) -> Option<P2> {
    let long = |c: &[P2]| c.windows(2).map(|w| (w[1][0]-w[0][0]).hypot(w[1][1]-w[0][1])).sum::<f64>();
    let (i,forward) = walk.iter().filter_map(|w| match *w { Walk::Chord(i,f) => Some((i,f)),_ => None })
        .max_by(|x,y| long(&chords[x.0].1).total_cmp(&long(&chords[y.0].1)))?;
    let c = &chords[i].1;
    if c.len() < 2 { return None }
    let j = (c.len()/2).min(c.len()-2);
    let (p,q) = if forward { (c[j],c[j+1]) } else { (c[j+1],c[j]) };
    let m = [0.5*(p[0]+q[0]),0.5*(p[1]+q[1])];
    let l = (q[0]-p[0]).hypot(q[1]-p[1]);
    if !(l > 0.) { return None }
    let left = [-(q[1]-p[1])/l,(q[0]-p[0])/l];
    let others = chords.iter().enumerate().filter(|&(k,_)| k != i).flat_map(|(_,c)| &c.1).map(|r| (r[0]-m[0]).hypot(r[1]-m[1]));
    let side = m[0].min(1.-m[0]).min(m[1]).min(1.-m[1]);
    let clear = others.fold(side,f64::min);
    let depth = (0.5*clear).min(1e-3).min(0.25*l);
    Some([m[0]+left[0]*depth,m[1]+left[1]*depth])
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
    // (a crossing of a seam both faces beside it trace straight back to the seam's end at the one
    // corner is a cap about the tube's vertex, no meeting: beside a meridian an instant touches whole
    // the end pose touches the sheet along its rim, and the rounding there draws loops a hair across,
    // #880 T1d M2 — each such chord dropped)
    let corner = |p: P2| [p[0],p[1]].iter().all(|&x| x <= ON_SIDE || x >= 1.-ON_SIDE);
    let cap = |k: usize,c: &[P2]| -> Option<(usize,f64,f64)> {
        let (p,q) = (c[0],c[c.len()-1]);
        [(p,q),(q,p)].into_iter().find_map(|(x,y)| (corner(y) && !corner(x) && seam_of(k,x[0]).is_some() && seam_of(k,x[0]) == seam_of(k,y[0]))
            .then(|| (seam_of(k,x[0]).unwrap(),x[1],y[1].round())))
    };
    let caps: Vec<Vec<(usize,f64,f64)>> = faces.iter().enumerate().map(|(k,f)| f.chords.iter().filter(|c| c.0 < 2).filter_map(|c| cap(k,&c.1)).collect()).collect();
    for k in 0..pieces {
        let others: Vec<(usize,f64,f64)> = [(k+pieces-1)%pieces,(k+1)%pieces].into_iter().filter(|&j| j != k).flat_map(|j| caps[j].clone()).collect();
        faces[k].chords.retain(|c| c.0 >= 2 || !cap(k,&c.1).is_some_and(|(s,y,e)| others.iter().any(|&(s2,y2,e2)| s2 == s && e2 == e && (y2-y).abs() <= SNAP)));
    }
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
    // (the nearest place within a side's reach: an end a hair from a corner is the corner's, and
    // one the face beside read within `SNAP` its place)
    let place = |s: usize,y: f64| -> Result<usize,Unbuilt> { seams[s].iter().enumerate().filter(|(_,&q)| (q-y).abs() <= SNAP)
        .min_by(|x,z| (x.1-y).abs().total_cmp(&(z.1-y).abs())).map(|(i,_)| i)
        .ok_or_else(|| {
            if ring_debug() { eprintln!("ring: seam {s} has no place at {y}: {:?}",seams[s]) }
            Unbuilt::Failed("a ring's sheet's crossing of the tool ends on a seam where the face beside finds none".into())
        }) };
    for (k,face) in faces.iter_mut().enumerate() {
        for (_,c) in &mut face.chords {
            let last = c.len()-1;
            for i in [0,last] { if let Some(s) = seam_of(k,c[i][0]) { c[i] = [c[i][0].round(),seams[s][place(s,c[i][1])?]] } }
        }
        // (and one its ends' places put at one place has no length, and bounds nothing, #942 F6)
        face.chords.retain(|(_,c)| !no_length(c));
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
        // (and where two end at one place — at the tube's vertex, each run into it, #942 T1d-ring —
        // its pieces are the planar graph's too: no way round the square alone tells them apart)
        let ends: Vec<f64> = face.chords.iter().flat_map(|(_,c)| [perimeter(c[0]),perimeter(c[c.len()-1])]).collect();
        let shared = ends.iter().enumerate().any(|(i,a)| ends[i+1..].iter().any(|b| (a-b).abs() < ON_SIDE));
        // (and a chord ending inside the square — a closed trace, or a curve ending on one — leaves
        // the planar graph pieces with holes, which no way round the square finds)
        let on_rim = |p: P2| p[0] <= ON_SIDE || p[0] >= 1.-ON_SIDE || p[1] <= ON_SIDE || p[1] >= 1.-ON_SIDE;
        let rimmed = face.chords.iter().all(|(_,c)| on_rim(c[0]) && on_rim(c[c.len()-1]));
        let simple = rimmed && !shared && split.len() == face.chords.len() && split.iter().zip(&face.chords).all(|(x,y)| x.1.len() == y.1.len());
        let told = |_: &Unbuilt| if ring_debug() {
            for (e,c) in &face.chords { eprintln!("ring: face {k} unsplit chord of kind {e} from {:?} to {:?}, {} points",c[0],c[c.len()-1],c.len()) }
        };
        let chords: &[(usize,Vec<P2>)] = if simple { &face.chords } else { &split };
        let walks = if simple { pieces_of_square(&face.chords).inspect_err(told)? } else { pieces_among(&split).inspect_err(told)? };
        let walks = with_holes(walks,chords);
        if ring_debug() {
            eprintln!("trim: face {k} over {roll:?} chords {:?}",chords.iter().map(|(e,c)| (*e,c[0],c[c.len()-1])).collect::<Vec<_>>());
            for w in &walks { eprintln!("trim:   walk {w:?}") }
        }
        // the chords' edges, made once, each walked by the two pieces either side
        let mut chord_edges: Vec<Option<u32>> = vec![None;chords.len()];
        // (a double curve's chord carries its own curve in the face's parameters, on its curve's
        // chord-length parameter: the face's surface still holds the fold just past it, where an
        // inversion onto it may land on the leaf beyond, #880 T1d)
        let mut chord_pcurves: Vec<Option<Pcurve>> = vec![None;chords.len()];
        let mut rim_kept = [false;2];
        let mut kept_chords: [Vec<(Curve,Vec<V>)>;2] = [Vec::new(),Vec::new()];
        for (walk,holes) in walks {
            // kept where it stands outside the tool at both ends: read just inside its longest side,
            // or, where it has none (a piece inside the face, among closed chords), just to the left
            // of its longest chord's middle — each piece walked with it on its left
            let probe = match walk.iter().filter_map(|w| match *w { Walk::Side(s0,s1) => Some((s0,s1)),_ => None })
                .max_by(|x,y| (x.1-x.0).total_cmp(&(y.1-y.0))) {
                Some((s0,s1)) => {
                    let m = round_square(0.5*(s0+s1));
                    let into = INWARD[(s0.floor() as usize).min(3)];
                    // no farther in than half the way to the nearest chord
                    let clear = chords.iter().flat_map(|(_,c)| c).map(|p| (p[0]-m[0]).hypot(p[1]-m[1])).fold(f64::INFINITY,f64::min);
                    let depth = (0.5*clear).min(1e-3).min(0.25*(s1-s0));
                    [m[0]+into[0]*depth,m[1]+into[1]*depth]
                }
                None => {
                    let Some(p) = beside_chord(&walk,chords) else { continue };
                    p
                }
            };
            if ring_debug() {
                let x = net.point(k as f64+probe[0],roll[0]+(roll[1]-roll[0])*probe[1]);
                eprintln!("trim:   probe {probe:?} at {x:?}: outside {}, tail {}, holes {}",face.readings.iter().flatten().all(|r| r.outside(probe)),
                    (face.tail)(probe),holes.len());
            }
            if !face.readings.iter().flatten().all(|r| r.outside(probe)) || (face.tail)(probe) { continue }
            // (its outer loop, then each hole in it)
            let mut loops: Vec<Vec<Coedge>> = Vec::new();
            for lw in std::iter::once(walk).chain(holes) {
                let mut lp: Vec<Coedge> = Vec::new();
                for w in lw {
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
                                        Some(r) => {
                                            let (curve,xs,on) = chord_curve(r,pts,tol)?;
                                            // **its pcurve the places on the face it passes** (#942 F3, by
                                            // construction): each at the curve's own parameter there, not the curve
                                            // inverted onto the face from the line between its ends — beside a fold
                                            // pair's birth the face's sheet folds back on itself within 2e-4, and the
                                            // inverse landed on the other fold (the tumbling torus's half turn)
                                            let uvs: Vec<V> = on.iter().map(|&(_,[x,y])| [u0+x,t_of(y),0.]).collect();
                                            let ts: Vec<f64> = on.iter().map(|&(t,_)| t).collect();
                                            // (the coarsest cubic through every so many of them — as the curve is
                                            // drawn — whose image holds the curve's own bar, a quarter of `tol`, at
                                            // each of them and at 64 places along it: beside the birth itself, where
                                            // the leaf has no width, the places the trace passes are no guide, none
                                            // holds, and the curve is inverted as before)
                                            let holds = |pc: &crate::nurbs::BSpline| {
                                                let at = |t: f64| { let q = pc.point(t); norm(crate::util::space::sub(surface.point([q[0],q[1]]),curve.point(t))) };
                                                ts.iter().all(|&t| at(t) <= 0.25*tol) && (0..=64).all(|j| at(j as f64/64.) <= 0.25*tol)
                                            };
                                            let pc = ts.windows(2).all(|w| w[1] > w[0]).then(|| [64,32,16,8,4,2,1].into_iter().find_map(|stride: usize| {
                                                let mut keep: Vec<usize> = (0..uvs.len()).step_by(stride).collect();
                                                if *keep.last().unwrap() != uvs.len()-1 { keep.push(uvs.len()-1) }
                                                if keep.len() < 2 { return None }
                                                let pc = crate::nurbs::interpolate(&keep.iter().map(|&k| uvs[k]).collect::<Vec<_>>(),
                                                    &keep.iter().map(|&k| ts[k]).collect::<Vec<_>>(),3.min(keep.len()-1))?;
                                                holds(&pc).then_some(pc)
                                            })).flatten();
                                            if ring_debug() && pc.is_none() {
                                            eprintln!("trim:   chord {i} from {:?} to {:?}: no pcurve through its places holds, inverted",pts[0],pts[pts.len()-1])
                                        }
                                        if let Some(pc) = pc { chord_pcurves[i] = Some(Pcurve::Curve(Arc::new(Curve::BSpline(Arc::new(pc))))) }
                                            (curve,xs)
                                        }
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
                loops.push(lp);
            }
            out.faces.push(Face {surface:surface.clone(),reversed:false,loops,name:"sheet".into()});
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

    /// **An arc's contact terms under the twist `w`** (`root`'s): `A = P · e_r(θ)` and
    /// `B = q + Q · e_r(θ)` about the arc's centre `c`, as `(P, q, Q)`, and the scale they are read
    /// against.
    fn terms(&self,w: &Twist,c: P2) -> ([f64;2],f64,[f64;2],f64) {
        let (e1,e2,ez) = (self.axes(0.).0,self.axes(std::f64::consts::FRAC_PI_2).0,self.axes(0.).2);
        let uc = w.velocity(self.lift(0.,[0.,c[1]]));
        let cross_z = cross(ez,w.omega);
        let (pp,q,qq) = ([dot(uc,e1),dot(uc,e2)],dot(ez,uc),[c[0]*dot(cross_z,e1),c[0]*dot(cross_z,e2)]);
        (pp,q,qq,pp[0].dhypot(pp[1])+q.abs()+qq[0].dhypot(qq[1]))
    }

    /// The point of the meridian's arc `k` in the half-plane at `θ` farthest from the ring's axis.
    pub fn meridian_point(&self,theta: f64,k: usize) -> V {
        let m = match self.elements[k] { Element::Arc {c,r,..} => [c[0]+r,c[1]],Element::Line {a,b} => if a[0] >= b[0] { a } else { b } };
        self.lift(theta,m)
    }

    /// **The instants a meridian is touched whole at** (`touched_whole`), of `looks + 1` equal steps
    /// over the roll `[a, b]` along `path`: each with the meridian's angle and its arc.
    pub(crate) fn touched_over(&self,path: &Trajectory,[a,b]: [f64;2],looks: usize) -> Result<Vec<(f64,f64,usize)>,Unbuilt> {
        let mut out = Vec::new();
        for k in 0..=looks {
            let t = a+(b-a)*k as f64/looks as f64;
            if let Some((theta,arc)) = self.touched_whole(&Twist::body(path.at(t)?)) { out.push((t,theta,arc)) }
        }
        Ok(out)
    }

    /// **A meridian the instant touches whole** (#942 T1d-ring): the angle `θ*` of the half-plane
    /// whose arc of the meridian the twist `w` moves along itself all round — `A` and `B` both nought
    /// there, to their rounding's half the scale's digits (`root`'s reading) — and the arc; none
    /// where no arc's is.
    pub fn touched_whole(&self,w: &Twist) -> Option<(f64,usize)> {
        self.elements.iter().enumerate().find_map(|(k,e)| {
            let Element::Arc {c,..} = *e else { return None };
            let (pp,q,qq,size) = self.terms(w,c);
            let base = (-pp[0]).datan2(pp[1]);
            [base,base+std::f64::consts::PI].into_iter().find(|&st| { let (ss,cs) = st.dsin_cos(); (q+qq[0]*cs+qq[1]*ss).abs() <= HALF_READ*size })
                .map(|st| (st.rem_euclid(TAU),k))
        })
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
/// At roll `t`, the root of the arc's branch `branch` (face 0) at each of `thetas` and its `s''`,
/// and the point in space (a probe of the contact beside a whole-touched meridian).
pub fn branch_at(ring: &Ring,path: &Trajectory,roll: [f64;2],t: f64,branch: bool,thetas: &[f64]) -> Result<Vec<(f64,V,f64)>,Unbuilt> {
    let (w,dw) = rates(path,t,roll)?;
    let pose = path.pose(t)?;
    let l = Label {on:On::Face(0),branch};
    Ok(thetas.iter().filter_map(|&th| ring.root(&w,th,l,false).map(|r| (th,pose.point(r.x),ring.bend(&w,&dw,th,&r)))).collect())
}

/// Each contact curve's pieces at roll `t` under `path` over `roll`: each piece's angles round the
/// axis and its kind (a probe of the reader).
pub fn pieces_at(ring: &Ring,path: &Trajectory,roll: [f64;2],t: f64) -> Result<Vec<Vec<(f64,f64,Label,Side)>>,Unbuilt> {
    let reader = Reader::new(ring,path,roll)?;
    let r = reader.read(t)?;
    Ok(r.curves.iter().map(|c| c.iter().map(|p| (p.from,p.to,p.label,p.side)).collect()).collect())
}

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
