//! **Certified root isolation** (#777 M1, #814): the zeros of a function over a stretch, or of a
//! system over a box, found from enclosures of it there — never from samples, so no root is
//! missed for falling between two of them, however close two roots stand (a thin sheet's two
//! crossings a hair apart) — and every root returned proved, in outward-rounded arithmetic, or the
//! search refused with the box it could not settle (`GaveUp`).
//!
//! **What proves a stretch or a box is one choice among several, behind one boundary** (#814): a
//! search halves, budgets, polishes and reads each root once whatever decides a box, so that
//! decision is a `Test` (several unknowns) or a `Test1` (one), and a caller names the one it reads
//! by. Two of each are written:
//!
//! * **`Krawczyk`** (the default): a box `X` about its middle `m` holds no root where `0 ∉ F(m) +
//!   J(X)(X − m)` (the mean-value form, `J(X)` an enclosure of the Jacobian over the box) or where
//!   `K(X) = m − Y F(m) + (I − Y J(X))(X − m)` misses `X`, `Y` the inverse of `J`'s middle; and
//!   exactly one where `K(X) ⊂ int X`. Only first derivatives are enclosed, and the mean-value
//!   inclusion holds for any Lipschitz function — across a C⁰ knot the enclosure is the hull of both
//!   one-sided slopes — so a kink costs nothing (#812). `F(m)` and `J(X)` come from the system in
//!   outward-rounded intervals (`util::interval::Iv`), so rounding is inside the proof (#796, #780
//!   R2) rather than a margin added after it. The root is settled by Krawczyk's own contraction
//!   `X ← K(X) ∩ X`, then Newton's method within what is left.
//! * **`Kantorovich`**: the theorem of Kantorovich at the box's middle, from bounds on the second
//!   derivatives the caller writes by hand (`Hᵢ`, the sum of equation `i`'s Hessian's entries'
//!   magnitudes): equation `i` is off nought where `|Fᵢ(m)| > Σₖ |∂ₖFᵢ(m)| eₖ + ½ Hᵢ r²`; with `β =
//!   ‖J⁻¹‖`, `η = ‖J⁻¹ F(m)‖` and `K = r² maxᵢ Hᵢ` in the norm the box is the unit ball of, `h = β K η
//!   ≤ ½` puts exactly one root within `ρ₋ = 2η/(1 + √(1 − 2h))` of `m` and no other within `ρ₊ = (1
//!   + √(1 − 2h))/(β K)`. It is only as sound as the bound it is handed (#812: a bound of nought
//!   across a kink lost a crossing), which is why no reading in the kernel uses it; it is kept so a
//!   search can be read both ways and held to the same roots.
//! * **`Slope`** (one unknown, the default): a stretch is off nought where `0 ∉ g(m) + g'(X)(X −
//!   m)`, and monotone — one root at most, where its ends differ in sign, settled by bisection —
//!   where `0 ∉ g'(X)`: interval Newton's existence test in one variable.
//! * **`Bounded`** (one unknown): the same from hand bounds on `|g'|` and `|g''|`.
//!
//! A box or a stretch neither test decides is halved; one that cannot be parted before it is
//! `width` wide holds a singular root — a touch — where `near` accepts its middle. Each box is read
//! grown a little past its sides (`GROW`), so a root on the face two halves share is inside both.
//! A stretch's sign at its ends is proved by its value's enclosure, not read in floats (rounding's
//! noise beside a double root made a touch dozens of crossings), and two crossings a hair apart
//! the opposite ways are kept as the pair they are (one crossing in their place flips a ray's
//! count).
//!
//! **A run is proved, not parted** (#814): where the function stays within the caller's tolerance
//! of nought along a stretch — a curve lying along a surface for a while — no box can part its
//! roots nor exclude them. A `Function` says when a whole stretch is within the tolerance
//! (`within`), a `System` proves a run through a point its tolerance accepts (`run`); the search
//! reads its ends as touches, which the caller settles as one meeting, and no box inside it again.
//! A system may also offer an orthogonal frame its equations are read in besides their own
//! (`frame`): a curve all but tangent to a surface is told off it along the surface's normal,
//! where its world axes' components are not.
//!
//! **Not proved means refused** (#814 P0): a root a test certifies is returned only once it is
//! settled within the box it was certified in and `near` accepts it there; a Newton iterate that
//! leaves that box, a polish that does not settle, or a box that cannot be parted where no touch is
//! (its enclosure left its domain) gives up as `GaveUp::Unsettled`, with the box — never the last
//! iterate presented as a root.
//!
//! Each stretch or box read is counted against the build's budget (`util::meter`, the ray stage, or
//! the stage the caller names): past it, or past `MOST` boxes, the search gives up and says which
//! (`GaveUp`), never a guess.
use super::interval::{Iv,Jet,Num};

type Root = super::roots::Root;

/// The most stretches or boxes one search reads.
pub const MOST: usize = 1 << 16;

/// **Why a search gave up** (#780 R3, #814), apart from finding no root: a caller left with no
/// answer knows whether the build it serves has stopped, and so whether to read the pair any other
/// way, and where a box could not be settled, which box (the witness its refusal carries).
#[derive(Clone,Debug,PartialEq)]
pub enum GaveUp {
    /// It read more than `MOST` stretches or boxes.
    Most,
    /// The build it is counted against went past its budget (`util::meter`).
    Stopped,
    /// A box (each unknown's stretch) it could not settle: one certified to hold a root whose
    /// polish did not settle within it, or one too narrow to part whose enclosure proved nothing.
    Unsettled { at: Vec<[f64;2]> },
}

/// One more stretch or box read: `read` of them so far, the search giving up past `MOST` or under
/// a stopped build, told where (`at`) should it stop there.
fn read_at(read: &mut usize,stage: super::meter::Stage,at: &dyn Fn() -> ([f64;3],String)) -> Result<(),GaveUp> {
    *read += 1;
    if *read > MOST { return Err(GaveUp::Most) }
    if !super::meter::charge(stage,1,at) { return Err(GaveUp::Stopped) }
    Ok(())
}

/// Nowhere to name.
fn nowhere() -> ([f64;3],String) { ([f64::NAN;3],String::new()) }

// ---------------------------------------------------------------------------------------------
// One unknown

/// **A function of one unknown as a search reads it**: its value and rate at a point in floats
/// (bracketing and polishing), its value at a point and its rate over a stretch enclosed in
/// outward-rounded intervals (the proof).
pub trait Function {
    /// `g(t)` and `g'(t)`.
    fn at(&self,t: f64) -> (f64,f64);
    /// An enclosure of `g(t)`, its rounding inside it.
    fn value(&self,t: f64) -> Iv;
    /// An enclosure of `g'` over `[a, b]` (`Iv::ANY` where none is known).
    fn slope(&self,a: f64,b: f64) -> Iv;
    /// Whether `g` is proved within the caller's tolerance of nought over all of `[a, b]`: a run
    /// along its zero set, which no stretch can part and none exclude (a trace arriving flatly on a
    /// plane it lies all but in). Its ends are read as touches, and the caller settles the run as
    /// one meeting. By default none is.
    fn within(&self,_a: f64,_b: f64) -> bool { false }
}

/// **A function written once over `Num`** (`util::interval`): its jet at a jet of `t`, read in
/// floats for its value and rate and in intervals for their enclosures — one formula, so the
/// enclosure cannot contradict the function (`Written`).
pub trait Formula { fn jet<T: Num>(&self,t: Jet<T>) -> Jet<T>; }

/// A `Formula` read as a `Function`.
pub struct Written<F>(pub F);

impl<F: Formula> Function for Written<F> {
    fn at(&self,t: f64) -> (f64,f64) { let j = self.0.jet(Jet::at(t)); (j.v,j.d) }
    fn value(&self,t: f64) -> Iv { self.0.jet(Jet::at(Iv::point(t))).v }
    fn slope(&self,a: f64,b: f64) -> Iv { self.0.jet(Jet::at(Iv::new(a,b))).d }
}

/// **A function from its parts**: `g` and `g'` in floats, `g` enclosed at a point and `g'` over a
/// stretch — for a caller whose function is not one formula (a meridian read through its own
/// jets, a polynomial).
pub struct Parts<'a> {
    pub at: &'a dyn Fn(f64) -> (f64,f64),
    pub value: &'a dyn Fn(f64) -> Iv,
    pub slope: &'a dyn Fn(f64,f64) -> Iv,
}

impl Function for Parts<'_> {
    fn at(&self,t: f64) -> (f64,f64) { (self.at)(t) }
    fn value(&self,t: f64) -> Iv { (self.value)(t) }
    fn slope(&self,a: f64,b: f64) -> Iv { (self.slope)(a,b) }
}

/// A function its search reads, each point's value enclosed once (`roots_at`).
struct Held<'a> { g: &'a dyn Function,values: std::cell::RefCell<std::collections::BTreeMap<u64,Iv>> }

impl Function for Held<'_> {
    fn at(&self,t: f64) -> (f64,f64) { self.g.at(t) }
    fn value(&self,t: f64) -> Iv {
        if let Some(&v) = self.values.borrow().get(&t.to_bits()) { return v }
        let v = self.g.value(t);
        self.values.borrow_mut().insert(t.to_bits(),v);
        v
    }
    fn slope(&self,a: f64,b: f64) -> Iv { self.g.slope(a,b) }
    fn within(&self,a: f64,b: f64) -> bool { self.g.within(a,b) }
}

/// **A polynomial** (coefficients lowest first) as a `Function`: Horner's rule in floats and in
/// intervals.
pub struct Poly<'a>(pub &'a [f64]);

impl Function for Poly<'_> {
    fn at(&self,t: f64) -> (f64,f64) { (poly(self.0,t),poly_d(self.0,t)) }
    fn value(&self,t: f64) -> Iv { poly_iv(self.0,Iv::point(t)) }
    fn slope(&self,a: f64,b: f64) -> Iv {
        // (each coefficient `k cₖ` rounded: enclosed a hair outward)
        let t = Iv::new(a,b);
        self.0.iter().enumerate().skip(1).rev()
            .fold(Iv::point(0.),|acc,(k,&x)| { let d = k as f64*x; acc*t+Iv::new(d.next_down(),d.next_up()) })
    }
}

/// The polynomial `c` over the interval `t`, by Horner's rule.
fn poly_iv(c: &[f64],t: Iv) -> Iv { c.iter().rev().fold(Iv::point(0.),|acc,&x| acc*t+Iv::point(x)) }

/// What a one-unknown test makes of a stretch.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Stretch {
    /// The function is off nought across it.
    Off,
    /// The function runs one way across it: one root at most, where its ends differ in sign; with
    /// the enclosure of its rate the test proved it by, where it has one (`Iv::ANY` where not).
    Monotone(Iv),
    /// Neither, proved: halved.
    Unsure,
}

/// **What decides a stretch** (#814's boundary for one unknown): off nought, monotone, or neither,
/// and how far the function can move from its middle across it (what reads a touch beside a
/// stretch it is off: a line passing a hair from tangent).
pub trait Test1 {
    fn read(&self,g: &dyn Function,a: f64,b: f64) -> (Stretch,f64);
}

/// **The interval slope** (interval Newton in one unknown): off where `0 ∉ g(m) + g'(X)(X − m)`,
/// monotone where `0 ∉ g'(X)`.
pub struct Slope;

impl Test1 for Slope {
    fn read(&self,g: &dyn Function,a: f64,b: f64) -> (Stretch,f64) {
        let m = 0.5*(a+b);
        let s = g.slope(a,b);
        let reach = s*(Iv::new(a,b)-Iv::point(m));
        // (monotone read first: its ends' values its neighbours share, where the middle's is its own)
        if !s.has_zero() { return (Stretch::Monotone(s),reach.mag()) }
        let around = g.value(m)+reach;
        (if !around.has_zero() { Stretch::Off } else { Stretch::Unsure },reach.mag())
    }
}

/// **Hand bounds** `[|g'|, |g''|]` over a stretch (the way every reading was proved before #814;
/// kept so a search can be read both ways): off where `|g(m)| > D₁ h`, monotone where `|g'(m)| >
/// D₂ h`, by a margin past rounding (#780 R2).
pub struct Bounded<'a>(pub &'a dyn Fn(f64,f64) -> [f64;2]);

impl Test1 for Bounded<'_> {
    fn read(&self,g: &dyn Function,a: f64,b: f64) -> (Stretch,f64) {
        let (m,h) = (0.5*(a+b),0.5*(b-a));
        let [d1,d2] = (self.0)(a,b);
        let (gm,dm) = g.at(m);
        let verdict = if gm.abs() > d1*h { Stretch::Off } else if dm.abs() > d2*h*(1.+1e-9)+1e-12*d1 { Stretch::Monotone(Iv::ANY) } else { Stretch::Unsure };
        (verdict,d1*h)
    }
}

/// **The roots of `g` over `[t0, t1]`**, each proved by the interval slope (`Slope`). A stretch
/// narrower than `width` that cannot be parted holds a touch where `near(t)` says `g` is nought
/// there. Each root once, in order; an end of the stretch where `g` is nought a root there.
/// `GaveUp` where the search read more than `MOST` stretches or the build stopped.
pub fn roots(g: &dyn Function,[t0,t1]: [f64;2],width: f64,near: &dyn Fn(f64) -> bool) -> Result<Vec<Root>,GaveUp> {
    roots_by(&Slope,super::meter::Stage::Ray,g,[t0,t1],width,near)
}

/// `roots`, each stretch decided by `test` and each read charged to `stage` (a swept sheet's
/// inverse map isolates its foot on its spine this way, and its work is the inverse stage's, #780
/// C1).
pub fn roots_by(test: &dyn Test1,stage: super::meter::Stage,g: &dyn Function,[t0,t1]: [f64;2],width: f64,
    near: &dyn Fn(f64) -> bool) -> Result<Vec<Root>,GaveUp> {
    roots_at(test,stage,&nowhere,g,[t0,t1],width,near)
}

/// `roots_by`, a build stopped by this search told where (`at`: a point and what was sought there).
pub fn roots_at(test: &dyn Test1,stage: super::meter::Stage,at: &dyn Fn() -> ([f64;3],String),g: &dyn Function,[t0,t1]: [f64;2],
    width: f64,near: &dyn Fn(f64) -> bool) -> Result<Vec<Root>,GaveUp> {
    // (each point's value enclosed once: a stretch's ends and middle are its neighbours' and its
    // halves' too)
    let g = &Held {g,values:std::cell::RefCell::new(std::collections::BTreeMap::new())};
    let mut out: Vec<Root> = Vec::new();
    if !(t1 >= t0) { return Ok(out) }
    let width = width.max(1e-15*(t0.abs()+t1.abs()).max(1e-300));
    let dg = |t: f64| g.at(t).1;
    let gv = |t: f64| g.at(t).0;
    let mut stack = vec![(t0,t1,0usize)];
    let mut read = 0;
    while let Some((a,b,depth)) = stack.pop() {
        read_at(&mut read,stage,at)?;
        let m = 0.5*(a+b);
        if g.within(a,b) {
            out.push(Root {at:a,touch:true});
            out.push(Root {at:b,touch:true});
            continue
        }
        let (verdict,reach) = test.read(g,a,b);
        let sign = |v: Iv| if v.lo() > 0. { 1 } else if v.hi() < 0. { -1 } else { 0 };
        // monotone across it: one root at most, where its ends differ in sign — each end's sign
        // proved by its value's enclosure (#814: read in floats, rounding's noise beside a double
        // root flipped them, and a touch came back as dozens of crossings a hair apart): from the
        // middle's value and the slope (`g(a) ∈ g(m) + g'(X)(a − m)`), each end's own where that
        // proves no sign. A stretch whose two ends are both within rounding of nought is unsure.
        let ends = match verdict {
            Stretch::Monotone(slope) => {
                let gm = if slope.0.is_some() { g.value(m) } else { Iv::ANY };
                let end = |x: f64| { let v = gm+slope*(Iv::point(x)-Iv::point(m)); if sign(v) != 0 { sign(v) } else { sign(g.value(x)) } };
                Some((end(a),end(b))).filter(|&e| e != (0,0))
            }
            _ => None,
        };
        let halve = |stack: &mut Vec<(f64,f64,usize)>| if b-a > width && depth < 96 { stack.push((m,b,depth+1)); stack.push((a,m,depth+1)); true } else { false };
        match (verdict,ends) {
            // nought nowhere across the stretch: but where it comes within the tolerance of nought
            // and turns back there (`near` at the turn, `g'` changing sign), a touch to the
            // tolerance — a line passing a hair from tangent, which the caller reads as touching
            (Stretch::Off,_) => {
                let (da,db) = (dg(a),dg(b));
                if (da < 0.) != (db < 0.) && (gv(m).abs() <= reach*4. || near(m)) {
                    let at = super::roots::bracketed_root(|t| Some(dg(t)),a,da,b,db,width*1e-3);
                    if near(at) { out.push(Root {at,touch:true}) }
                }
            }
            (Stretch::Monotone(_),Some((sa,sb))) => {
                let up = dg(m) > 0.;
                let (lo,hi) = if up { (-1,1) } else { (1,-1) };
                // an end within rounding of nought, the function running on past it: a crossing
                // where it is proved on the other side of nought a stretch's width back (or the end
                // is the search's own), a touch where it is proved on this side, halved otherwise —
                // so a root on an end two stretches share is read by one
                if sa == 0 && sb == hi {
                    let back = if a <= t0 { lo } else { sign(g.value((a-(b-a)).max(t0))) };
                    if back == lo || a <= t0 { out.push(Root {at:a,touch:false}) }
                    else if (back == hi || !halve(&mut stack)) && near(a) { out.push(Root {at:a,touch:true}) }
                }
                else if sa == lo && sb == hi {
                    let (ga,gb) = (gv(a),gv(b));
                    let at = if ga*gb < 0. { super::roots::bracketed_root(|t| Some(gv(t)),a,ga,b,gb,4e-16*a.abs().max(b.abs())) } else {
                        // (floats and the enclosures disagreeing at an end: halved by the proved signs)
                        let (mut x,mut y) = (a,b);
                        for _ in 0..80 { let c = 0.5*(x+y); if sign(g.value(c)) == lo { x = c } else { y = c } }
                        0.5*(x+y)
                    };
                    out.push(Root {at,touch:false});
                }
                else if sa == lo && sb == 0 && b == t1 { out.push(Root {at:b,touch:false}) }
            }
            // unsure, or too narrow to part: a double root where the function is nought there
            _ => if !halve(&mut stack) && near(m) { out.push(Root {at:m,touch:true}) },
        }
    }
    // an end where the function is nought is a root there
    if gv(t1) == 0. { out.push(Root {at:t1,touch:false}) }
    out.sort_by(|x,y| x.at.total_cmp(&y.at));
    // one root once: the narrow stretches a touch leaves beside one another are one touch, placed
    // where `g'` changes sign among them (a double root is a simple root of the derivative, placed
    // to its rounding, never a least `|g|`), and a crossing within a width of one the same way is
    // that crossing
    let mut kept: Vec<Root> = Vec::new();
    let mut k = 0;
    while k < out.len() {
        if !out[k].touch {
            // (two crossings a hair apart the same way are one found twice; the opposite ways, a
            // pair — the function out past nought and back — which the caller reads as one touch,
            // and merged into one crossing it flipped the side a ray counts)
            let same = |l: &Root| !l.touch && (out[k].at-l.at).abs() <= 2.*width && (dg(out[k].at) > 0.) == (dg(l.at) > 0.);
            if !kept.last().is_some_and(same) { kept.push(out[k]) }
            else if let Some(l) = kept.last_mut() { *l = out[k] }
            k += 1;
            continue
        }
        let mut j = k;
        while j+1 < out.len() && out[j+1].touch && out[j+1].at-out[j].at <= 8.*width { j += 1 }
        let (a,b) = (out[k].at-width,out[j].at+width);
        let (da,db) = (dg(a),dg(b));
        let at = if (da < 0.) != (db < 0.) {
            super::roots::bracketed_root(|t| Some(dg(t)),a,da,b,db,width*1e-3)
        } else { 0.5*(out[k].at+out[j].at) };
        if !kept.last().is_some_and(|l| (at-l.at).abs() <= 2.*width) { kept.push(Root {at,touch:true}) }
        k = j+1;
    }
    Ok(kept)
}

// ---------------------------------------------------------------------------------------------
// Several unknowns

/// A root of a system: where it is, and whether it is singular (a touch) rather than certified
/// simple.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Point<const N: usize> { pub at: [f64;N],pub touch: bool }

/// **A system of `N` equations in `N` unknowns as a search reads it**: its values and Jacobian at a
/// point in floats (Newton's polish), its values at a point, and its values and Jacobian over a box,
/// enclosed in outward-rounded intervals (the proof).
pub trait System<const N: usize> {
    /// `F(x)` and its Jacobian (rows the equations).
    fn at(&self,x: [f64;N]) -> ([f64;N],[[f64;N];N]);
    /// An enclosure of `F(x)`, its rounding inside it.
    fn value(&self,x: [f64;N]) -> [Iv;N];
    /// Enclosures of `F` and its Jacobian over the box `bx`, read together (a curve's and a
    /// surface's points over a box come with their derivatives): a box whose values miss nought is
    /// dropped before any mean-value form is read. An entry `Iv::ANY` where none is known.
    fn over(&self,bx: &[[f64;2];N]) -> ([Iv;N],[[Iv;N];N]);
    /// **An orthogonal frame its equations are read in besides their own**, at the box's middle `m`:
    /// `F = 0` where `R F = 0`, and where its Jacobian is all but singular there (a curve running
    /// all but tangent to a surface), its components along the frame — the surface's normal first —
    /// are off nought across boxes their own axes' components are not. By default none.
    fn frame(&self,_m: [f64;N]) -> Option<[[f64;N];N]> { None }
    /// **A run through `x`**, a point the caller's tolerance accepts: the stretch of the first
    /// unknown over which the system is proved to stay within that tolerance of nought (a curve
    /// lying along a surface for a while, which no box can part and none exclude), or `None`. The
    /// search reads its ends as touches — the caller settles the run as one meeting — and reads no
    /// box within it again. By default none is proved.
    fn run(&self,_x: [f64;N]) -> Option<[f64;2]> { None }
}

/// **A system from its parts**, for a caller that writes its readings apart: `over` encloses `F`
/// and its Jacobian over a box, and `F` at a point is its reading over that point's box.
pub struct Closures<'a,const N: usize> {
    pub at: &'a dyn Fn([f64;N]) -> ([f64;N],[[f64;N];N]),
    pub over: &'a dyn Fn(&[[f64;2];N]) -> ([Iv;N],[[Iv;N];N]),
}

impl<const N: usize> System<N> for Closures<'_,N> {
    fn at(&self,x: [f64;N]) -> ([f64;N],[[f64;N];N]) { (self.at)(x) }
    fn value(&self,x: [f64;N]) -> [Iv;N] { (self.over)(&x.map(|v| [v,v])).0 }
    fn over(&self,bx: &[[f64;2];N]) -> ([Iv;N],[[Iv;N];N]) { (self.over)(bx) }
}

/// What a test makes of a box (read grown, `GROW`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Verdict<const N: usize> {
    /// No root in the box.
    Off,
    /// Exactly one root in `unique` (the box read), and it within `within`.
    One { within: [[f64;2];N],unique: [[f64;2];N] },
    /// Neither, proved: halved.
    Unsure,
}

/// **What decides a box** (#814's boundary for several unknowns): no root, exactly one, or neither;
/// and how a root it certified is settled within the box it was certified in (`None` where it
/// cannot be: the search then gives up rather than return an unproved point).
pub trait Test<const N: usize> {
    fn read(&self,sys: &dyn System<N>,bx: &[[f64;2];N]) -> Verdict<N>;
    fn settle(&self,sys: &dyn System<N>,within: [[f64;2];N]) -> Option<[f64;N]> { newton_within(sys,within) }
}

/// **The Krawczyk operator** (the default test; the module's).
pub struct Krawczyk;

/// The box's middle and the reach of each unknown from it, enclosed.
fn about<const N: usize>(bx: &[[f64;2];N]) -> ([f64;N],[Iv;N]) {
    let m: [f64;N] = std::array::from_fn(|k| 0.5*(bx[k][0]+bx[k][1]));
    (m,std::array::from_fn(|k| Iv::new(bx[k][0],bx[k][1])-Iv::point(m[k])))
}

/// `K(X)` over the box `bx`, or `None` where the mean-value form proves the box holds no root;
/// `Err(())` where the Jacobian's enclosure or its middle's inverse is wanting.
fn krawczyk<const N: usize>(sys: &dyn System<N>,bx: &[[f64;2];N]) -> Result<Option<[Iv;N]>,()> {
    let (m,r) = about(bx);
    let (range,j) = sys.over(bx);
    // off nought across the box, an equation at a time: its values over the box, or `Fᵢ(m) +
    // Σₖ Jᵢₖ(X)(Xₖ − mₖ)`, missing nought
    if range.iter().any(|x| !x.has_zero()) { return Ok(None) }
    let fm = sys.value(m);
    for i in 0..N {
        let s = (0..N).fold(fm[i],|acc,k| acc+j[i][k]*r[k]);
        if !s.has_zero() { return Ok(None) }
    }
    // and in the frame the system offers: `(R F)(m) + (R J(X))(X − m)`
    if let Some(rot) = sys.frame(m) {
        for row in rot {
            let at = (0..N).fold(Iv::point(0.),|acc,l| acc+Iv::point(row[l])*fm[l]);
            let s = (0..N).fold(at,|acc,k| acc+(0..N).fold(Iv::point(0.),|e,l| e+Iv::point(row[l])*j[l][k])*r[k]);
            if !s.has_zero() { return Ok(None) }
        }
    }
    if j.iter().flatten().any(|x| x.0.is_none()) || fm.iter().any(|x| x.0.is_none()) { return Err(()) }
    let middle: [[f64;N];N] = std::array::from_fn(|i| std::array::from_fn(|k| j[i][k].mid()));
    let y = inverse(middle).ok_or(())?;
    let k: [Iv;N] = std::array::from_fn(|i| {
        let step = (0..N).fold(Iv::point(0.),|acc,l| acc+Iv::point(y[i][l])*fm[l]);
        let bend = (0..N).fold(Iv::point(0.),|acc,c| {
            let e = (0..N).fold(Iv::point(if i == c { 1. } else { 0. }),|e,l| e-Iv::point(y[i][l])*j[l][c]);
            acc+e*r[c]
        });
        Iv::point(m[i])-step+bend
    });
    if k.iter().any(|x| x.0.is_none()) { return Err(()) }
    Ok(Some(k))
}

impl<const N: usize> Test<N> for Krawczyk {
    fn read(&self,sys: &dyn System<N>,bx: &[[f64;2];N]) -> Verdict<N> {
        let k = match krawczyk(sys,bx) { Ok(Some(k)) => k,Ok(None) => return Verdict::Off,Err(()) => return Verdict::Unsure };
        // `K(X)` holds every root `X` does: missing it, `X` holds none
        if (0..N).any(|i| k[i].hi() < bx[i][0] || k[i].lo() > bx[i][1]) { return Verdict::Off }
        // within its inside, exactly one
        if (0..N).all(|i| k[i].lo() > bx[i][0] && k[i].hi() < bx[i][1]) {
            return Verdict::One {within:std::array::from_fn(|i| [k[i].lo(),k[i].hi()]),unique:*bx}
        }
        Verdict::Unsure
    }
    /// Krawczyk's contraction `X ← K(X) ∩ X` while it shrinks the box, each step as proved as the
    /// first, then Newton's method within what is left (a step out of it stops, the last point
    /// inside kept: the root is in the box either way).
    fn settle(&self,sys: &dyn System<N>,within: [[f64;2];N]) -> Option<[f64;N]> {
        let mut x = within;
        let wide = |b: &[[f64;2];N]| (0..N).fold(0f64,|a,i| a.max(b[i][1]-b[i][0]));
        for _ in 0..40 {
            let Ok(Some(k)) = krawczyk(sys,&x) else { break };
            let next: [[f64;2];N] = std::array::from_fn(|i| [k[i].lo().max(x[i][0]),k[i].hi().min(x[i][1])]);
            if next.iter().any(|s| !(s[0] <= s[1])) { return None }
            let shrunk = wide(&next) < 0.5*wide(&x);
            x = next;
            if !shrunk { break }
        }
        let inside = |p: &[f64;N]| (0..N).all(|i| p[i] >= x[i][0] && p[i] <= x[i][1]);
        let mut p: [f64;N] = std::array::from_fn(|i| 0.5*(x[i][0]+x[i][1]));
        for _ in 0..8 {
            let (fx,jx) = sys.at(p);
            let Some(ix) = inverse(jx) else { break };
            let next: [f64;N] = std::array::from_fn(|i| p[i]-(0..N).map(|k| ix[i][k]*fx[k]).sum::<f64>());
            if !inside(&next) { break }
            let moved = (0..N).fold(0f64,|a,i| a.max((next[i]-p[i]).abs()));
            p = next;
            if moved == 0. { break }
        }
        Some(p)
    }
}

/// **Kantorovich's theorem from hand bounds on the second derivatives** (the module's): `second(bx)`
/// bounds each equation's over a box.
pub struct Kantorovich<'a,const N: usize>(pub &'a dyn Fn(&[[f64;2];N]) -> [f64;N]);

impl<const N: usize> Test<N> for Kantorovich<'_,N> {
    fn read(&self,sys: &dyn System<N>,bx: &[[f64;2];N]) -> Verdict<N> {
        let (m,_) = about(bx);
        // how far each unknown reaches from the middle across the box (`e`), the farthest `r`
        let e: [f64;N] = std::array::from_fn(|k| (m[k]-bx[k][0]).max(bx[k][1]-m[k]).max(1e-300));
        let r = e.iter().fold(0.,|a: f64,&x| a.max(x));
        let (fm,jm) = sys.at(m);
        let hs = (self.0)(bx);
        let off = (0..N).any(|i| {
            let reach: f64 = (0..N).map(|k| jm[i][k].abs()*e[k]).sum();
            fm[i].abs() > reach+0.5*hs[i]*r*r
        });
        if off { return Verdict::Off }
        let Some(inv) = inverse(jm) else { return Verdict::Unsure };
        // preconditioned: a box whose Newton step reaches past it on any unknown holds no root
        let step: [f64;N] = std::array::from_fn(|i| (0..N).map(|k| inv[i][k]*fm[k]).sum());
        let far = (0..N).any(|i| {
            let bend: f64 = (0..N).map(|k| inv[i][k].abs()*hs[k]).sum();
            step[i].abs() > e[i]+0.5*bend*r*r
        });
        if far { return Verdict::Off }
        let beta = (0..N).map(|i| inv[i].iter().map(|x| x.abs()).sum::<f64>()/e[i]).fold(0f64,f64::max);
        let eta = (0..N).map(|i| step[i].abs()/e[i]).fold(0f64,f64::max);
        let big = hs.iter().fold(0f64,|a,&h| a.max(h))*r*r;
        let bk = beta*big;
        let h = bk*eta;
        // (the nearer radius `(1 − s)/βK` read as `2η/(1 + s)`, which is it exactly and never short
        // of the first step `η`: over a sheet all but flat `βK` is rounding's and `1 − s` cancelled
        // to nothing)
        let (lo,hi) = if bk > 0. {
            let s = (1.-2.*h).max(0.).sqrt();
            (2.*eta/(1.+s),(1.+s)/bk)
        } else { (eta,f64::INFINITY) };
        if h <= 0.5 && lo <= 1. && 1. <= hi {
            let ball = |k: usize| lo*e[k]*(1.+1e-9)+1e-300;
            return Verdict::One {within:std::array::from_fn(|k| [m[k]-ball(k),m[k]+ball(k)]),unique:*bx}
        }
        Verdict::Unsure
    }
}

/// **Newton's method from the middle of `within`, held in it** (#814 P0): the root, once a step
/// moves it no more than rounding; `None` where a step leaves the box or it does not settle.
fn newton_within<const N: usize>(sys: &dyn System<N>,within: [[f64;2];N]) -> Option<[f64;N]> {
    let mut x: [f64;N] = std::array::from_fn(|k| 0.5*(within[k][0]+within[k][1]));
    for _ in 0..60 {
        let (fx,jx) = sys.at(x);
        let ix = inverse(jx)?;
        let next: [f64;N] = std::array::from_fn(|i| x[i]-(0..N).map(|k| ix[i][k]*fx[k]).sum::<f64>());
        if (0..N).any(|i| !(next[i] >= within[i][0] && next[i] <= within[i][1])) { return None }
        let moved = (0..N).fold(0f64,|a,i| a.max((next[i]-x[i]).abs()));
        x = next;
        if moved <= 1e-15*(1.+x.iter().fold(0f64,|a,&v| a.max(v.abs()))) { return Some(x) }
    }
    None
}

/// **How far past its sides a search reads a box**, as a fraction of its width along each (within
/// the domain), so the boxes a box is halved into overlap: a root on the face two of them share
/// lies inside both by this much. Read as halves meeting at a face, such a root stood on the edge
/// of each: never certified there, each halved again with the root still on its face, until
/// rounding read it a hair outside both and both were dropped (#796: a ray from inside a saddle
/// block met the block's side at its stretch's middle, the crossing was lost, and the block's
/// inside read as out). Far past rounding at the widths a root is certified at, far short of a
/// box's own width.
pub const GROW: f64 = 1e-4;

/// **The roots of the system `sys` over the box `domain`**, each proved by the Krawczyk operator
/// (`Krawczyk`). A box narrower than `width` (each side) that cannot be parted holds a singular
/// root where `near(x)` accepts its middle. Each root once: one found in the box another was
/// certified the only root of is that root. `GaveUp` where the search read more than `MOST` boxes,
/// the build stopped, or a box could not be settled.
pub fn solve<const N: usize>(sys: &dyn System<N>,domain: [[f64;2];N],width: f64,near: &dyn Fn([f64;N]) -> bool) -> Result<Vec<Point<N>>,GaveUp> {
    solve_by(&Krawczyk,sys,domain,width,near)
}

/// `solve`, each box decided by `test`.
pub fn solve_by<const N: usize>(test: &dyn Test<N>,sys: &dyn System<N>,domain: [[f64;2];N],width: f64,
    near: &dyn Fn([f64;N]) -> bool) -> Result<Vec<Point<N>>,GaveUp> {
    // each root, and for one certified simple the box it was certified the only root of
    let mut out: Vec<(Point<N>,Option<[[f64;2];N]>)> = Vec::new();
    if domain.iter().any(|s| !(s[1] >= s[0])) { return Ok(Vec::new()) }
    let mut stack = vec![(domain,0usize)];
    let mut read = 0;
    // the stretches of the first unknown proved runs, and no box within one read again
    let mut runs: Vec<[f64;2]> = Vec::new();
    let mut tried: Vec<f64> = Vec::new();
    while let Some((bx,depth)) = stack.pop() {
        if runs.iter().any(|r| bx[0][0] >= r[0] && bx[0][1] <= r[1]) { continue }
        read_at(&mut read,super::meter::Stage::Ray,&nowhere)?;
        let m: [f64;N] = std::array::from_fn(|k| 0.5*(bx[k][0]+bx[k][1]));
        // the box read: grown by `GROW` of its width past each side, within the domain
        let grown: [[f64;2];N] = std::array::from_fn(|k| {
            let g = GROW*(bx[k][1]-bx[k][0]);
            [(bx[k][0]-g).max(domain[k][0]),(bx[k][1]+g).min(domain[k][1])]
        });
        match test.read(sys,&grown) {
            Verdict::Off => continue,
            Verdict::One {within,unique} => match test.settle(sys,within) {
                Some(x) if near(x) => { out.push((Point {at:x,touch:false},Some(unique))); continue }
                // certified, but not settled where the caller's tolerance accepts it: refused
                _ => return Err(GaveUp::Unsettled {at:within.to_vec()}),
            },
            Verdict::Unsure => {}
        }
        // a run through it, tried once a place: what no box beside it could part
        if near(m) && !tried.iter().any(|&x: &f64| (x-m[0]).abs() <= (bx[0][1]-bx[0][0]).max(width)) {
            tried.push(m[0]);
            if let Some(r) = sys.run(m).filter(|r| r[1]-r[0] > width) {
                for end in r { let mut at = m; at[0] = end; out.push((Point {at,touch:true},None)) }
                runs.push(r);
                continue
            }
        }
        if (0..N).all(|k| bx[k][1]-bx[k][0] <= width) || depth >= 200 {
            if near(m) { out.push((Point {at:m,touch:true},None)); continue }
            // too narrow to part and off the caller's tolerance, yet not proved off: the
            // enclosure proved nothing here (it left its domain), and no root is no answer
            return Err(GaveUp::Unsettled {at:grown.to_vec()})
        }
        // halved across the side it smears farthest along: each side's width times how far that
        // unknown carries the system there (its Jacobian's column at the middle), among the sides
        // still wider than `width` — so a box stays about as wide in what it reads every way, where
        // halving the widest side in the unknowns' own scales left it long along one that moves
        // the system far (a short edge read against a whole sheet)
        let (_,jm) = sys.at(m);
        let w: [f64;N] = std::array::from_fn(|i| bx[i][1]-bx[i][0]);
        let smear: [f64;N] = std::array::from_fn(|i| w[i]*(0..N).map(|r| jm[r][i].abs()).fold(0f64,f64::max));
        let wide = |i: &usize| w[*i] > width;
        let k = if (0..N).filter(wide).all(|i| smear[i].is_finite() && smear[i] > 0.) {
            (0..N).filter(wide).max_by(|&i,&j| smear[i].total_cmp(&smear[j]))
        } else { None };
        let k = k.unwrap_or_else(|| (0..N).max_by(|&i,&j| w[i].total_cmp(&w[j])).unwrap_or(0));
        let (mut a,mut b) = (bx,bx);
        a[k][1] = m[k];
        b[k][0] = m[k];
        stack.push((b,depth+1));
        stack.push((a,depth+1));
    }
    // a root on the face two boxes share is found in both: once — within a width of one another,
    // or either in the box the other was certified the only root of — and singular only where
    // both are
    let held = |b: &Option<[[f64;2];N]>,x: &[f64;N]| b.is_some_and(|b| (0..N).all(|k| x[k] >= b[k][0] && x[k] <= b[k][1]));
    let mut kept: Vec<(Point<N>,Option<[[f64;2];N]>)> = Vec::new();
    for (p,bp) in out {
        match kept.iter_mut().find(|(q,bq)| (0..N).all(|k| (q.at[k]-p.at[k]).abs() <= 2.*width) || held(bq,&p.at) || held(&bp,&q.at)) {
            Some((q,bq)) => { if q.touch && !p.touch { *q = p; *bq = bp; } }
            None => kept.push((p,bp)),
        }
    }
    Ok(kept.into_iter().map(|(p,_)| p).collect())
}

/// The largest row sum of `a`'s magnitudes: its norm on the largest-coordinate norm.
fn norm_inf<const N: usize>(a: &[[f64;N];N]) -> f64 { a.iter().map(|r| r.iter().map(|x| x.abs()).sum::<f64>()).fold(0.,f64::max) }

/// `a`'s inverse by elimination with partial pivoting, `None` where it is singular to rounding.
fn inverse<const N: usize>(a: [[f64;N];N]) -> Option<[[f64;N];N]> {
    let mut m = a;
    let mut inv: [[f64;N];N] = std::array::from_fn(|i| std::array::from_fn(|j| if i == j { 1. } else { 0. }));
    let scale = norm_inf(&a);
    if !(scale > 0.) || !scale.is_finite() { return None }
    for c in 0..N {
        let p = (c..N).max_by(|&i,&j| m[i][c].abs().total_cmp(&m[j][c].abs()))?;
        if !(m[p][c].abs() > 1e-14*scale) { return None }
        m.swap(c,p);
        inv.swap(c,p);
        let d = m[c][c];
        for j in 0..N { m[c][j] /= d; inv[c][j] /= d; }
        for i in 0..N {
            if i == c { continue }
            let f = m[i][c];
            if f == 0. { continue }
            for j in 0..N { m[i][j] -= f*m[c][j]; inv[i][j] -= f*inv[c][j]; }
        }
    }
    Some(inv)
}

/// The polynomial `c` (coefficients lowest first) at `t`.
pub fn poly(c: &[f64],t: f64) -> f64 { c.iter().rev().fold(0.,|acc,&x| acc*t+x) }

/// The polynomial `c`'s derivative at `t`.
pub fn poly_d(c: &[f64],t: f64) -> f64 { c.iter().enumerate().skip(1).rev().fold(0.,|acc,(k,&x)| acc*t+k as f64*x) }

/// The product of two polynomials (coefficients lowest first).
pub fn poly_mul(a: &[f64],b: &[f64]) -> Vec<f64> {
    let mut out = vec![0.;(a.len()+b.len()).saturating_sub(1)];
    for (i,&x) in a.iter().enumerate() { for (j,&y) in b.iter().enumerate() { out[i+j] += x*y } }
    out
}

/// The sum of polynomials, each times its weight.
pub fn poly_sum(terms: &[(f64,&[f64])]) -> Vec<f64> {
    let n = terms.iter().map(|t| t.1.len()).max().unwrap_or(0);
    let mut out = vec![0.;n];
    for &(w,p) in terms { for (k,&x) in p.iter().enumerate() { out[k] += w*x } }
    out
}
