//! `Constraint`: a kind and its arguments, its kernel and its rows.

use super::*;

#[derive(Clone, Debug)]
pub struct Constraint {
    /// Document-stable identity, assigned by `Sketch::add` (0 until then).
    pub id: u32,
    pub kind: CKind,
    pub args: Vec<Arg>,
    /// Soft constraints (drag targets) do not count toward convergence.
    pub soft: bool,
    /// Implied by a primitive's definition (an arc's endpoints sit at its radius).
    pub intrinsic: bool,
    /// A `claim` (Solvent §9.7): stated as *expected to add no rank*.  A claim is no equation —
    /// the solve, the decomposition and the drag walk all leave it out, so it can never move the
    /// geometry or weld two figures — and the diagnosis alone judges it, against the drawing the
    /// rest of the document made: a theorem when it holds and adds no rank, `violated` when it
    /// does not hold, `consuming` when enforcing it would have taken a freedom.  It travels like
    /// any flag: through `graft`, the document and the JSON.
    pub claim: bool,
    /// The unknown this constraint's number is written in terms of, when its dimension names a
    /// free variable — see `expr::Free`.  At most one, which is why it lives here and not on the
    /// argument: one appended column, one `(m, c)` pair, one twin kernel.  Derived state, written
    /// only by `expr::evaluate`, so `Constraint::new` and every rebuild leave it empty.
    pub free: Option<Free>,
    /// The classes the statement carries (§13.2) — a dimension's callout is drawn in them over
    /// `.dimension`, and `display: none` on one leaves the callout out of the layout altogether.
    /// Presentation: nothing that solves, diagnoses or decomposes reads it, and `same_constraint`
    /// does not compare it.
    pub class: crate::style::Classes,
    /// Its dimension **as the statement wrote it**, where that is not the text the number is
    /// worked out from: `l1 distance(w) r2` under `w := 100` reaches the sketch as `100`,
    /// since the flattener settles a `param` to its number, and the callout draws `w`.  Set by
    /// the elaborator where the statement is the document's own text — at the root, or in a
    /// component body written in this file, whose formula over its formals is true of every
    /// instance (`program::relations::written`); a module's body and a block's copies draw the
    /// number.  Presentation, like `class`: nothing that solves reads it, and writing a number
    /// (`set_num`, `expr::set_dimension`) drops it.
    pub written: Option<String>,
    /// A dimension **another copy of its block already states**: `repeat n { radius(hole_r)
    /// circle hole … }` is one statement, and six holes are drawn with one callout and not six.
    /// Set by the elaborator (`program::relations::repeated`) on every copy after the first
    /// that draws the same label; the full callout layout leaves it out, and asking for it by id
    /// (editing that one) still draws it.  Presentation, like `class`.
    pub repeated: bool,
    /// The defined word the statement was written with (§9.9), where it was one: `a horizontal
    /// b` reaches the sketch as the `Level` its body states, and is described as written.
    /// Presentation, like `class`; `graft` carries it while its operands survive.
    pub word: Option<WordUse>,
    /// **Stated as its derivative** (§6.21): this row's rate as a tangency's use moves along the
    /// set — the index of its `Sketch::duals` entry, which says what moves each column it reads.
    /// What `l tangent S` makes of each row of a set's body, at the contact on `l`.  `None` for
    /// every constraint stated as itself.  Not presentation: it picks the kernel
    /// (`kernels::dual_kernel`) and appends a tangent column per column and the line's ends to
    /// the columns, as `free` appends its column.
    pub along: Option<usize>,
    /// Its number is a **bound** (§9.6): `distance(>= d)`, `distance(<= d)`, `distance(in: (d,
    /// hi))`.  A bound is no equation — like a claim it compiles no row, so `acts()` leaves it
    /// out — and outside a set body it chooses a branch: read on the solution (`reading`), and
    /// the solve steered to the root where it holds (`solve::steer`).  Travels like `claim`.
    pub bound: Option<Bound>,
}

/// A defined relation word as a statement wrote it — see `Constraint::word`.
#[derive(Clone, Debug, PartialEq)]
pub struct WordUse {
    pub word: String,
    /// The entities the statement's operands named, in written order: two of an infix word —
    /// but for a set (§6.21), which is no entity and stands in `set`.
    pub ops: Vec<EntRef>,
    /// The parentheses' text as written (`d: 5mm`), or empty.
    pub args: String,
    /// The operands that are sets, each by its place among the operands and its name as written.
    pub sets: Vec<(usize, String)>,
}

/// `+1` and `−1` as the words a statement writes them with — the one place the two meet, read by
/// the constructor that infers a tangency's side from the geometry and by the document reader
/// that finds a number where a word now stands (issue #48, item 4).
pub fn side_word(sign: i64) -> &'static str {
    if sign < 0 {
        "right"
    } else {
        "left"
    }
}

impl Constraint {
    /// Whether this constraint *acts* on the drawing — the one predicate behind "everything that
    /// must be satisfied".  A soft one is a transient the solve may miss; a claim is a question
    /// about the drawing rather than part of it, and neither is something a consumer asking for
    /// the constraints that determine the figure wants back.
    pub fn acts(&self) -> bool {
        !self.soft && self.states_rows()
    }

    /// Whether it states rows at all: a claim (§9.7) is judged and a bound (§9.6) read, never
    /// solved for, so neither compiles one — the question every seam that compiles, counts or
    /// welds rows asks, where `acts` also leaves out the soft rows a drag does compile.
    pub fn states_rows(&self) -> bool {
        !self.claim && self.bound.is_none()
    }

    /// What `reading` can come to: a magnitude's at least zero (a distance between points, a
    /// point's from a line read either side), an unsigned angle's within half a turn, a signed
    /// measure's anything.
    pub fn reading_range(&self) -> (f64, f64) {
        match self.kind {
            CKind::Angle3 => (0.0, std::f64::consts::PI),
            k if k.magnitude() && self.side().is_none() => (0.0, f64::INFINITY),
            _ => (f64::NEG_INFINITY, f64::INFINITY),
        }
    }

    /// The number the drawing reads where this constraint's stands: the one that would make its
    /// row hold now, as the statement measures it (its word's sign, `side:`, `along:`) — what a
    /// bound is checked against (§9.6).  Found along the number from the one stated by secant
    /// steps, exact in one for the kinds whose residual is affine in it (an ordinate, a distance
    /// from a line) and a few for a point–point distance's squared form.  `None` for a kind
    /// outside `CKind::boundable`.
    pub fn reading(&self, sk: &Sketch) -> Option<f64> {
        if !self.kind.boundable() {
            return None;
        }
        let at = self.kind.dimension_slot()?;
        // a number reading an unknown (`distance(<= 2 * m)`, `m` free) is read at the unknown's
        // value: the row as a plain one at that number, so stepping the number moves the residual
        let plain = self.free.is_some().then(|| Constraint { free: None, ..self.clone() });
        let this = plain.as_ref().unwrap_or(self);
        let v = this.local_values(sk);
        // the unsigned angle in space is read by its cosine (`cos3 - cos θ`), which is even and
        // periodic in θ: read straight off it, never searched along it
        if self.kind == CKind::Angle3 {
            let d = self.args[at].num();
            return Some((this.residual(sk, &v)[0] + d.dcos()).clamp(-1.0, 1.0).dacos());
        }
        let mut c = this.clone();
        let mut residual = |d: f64| {
            c.args[at] = Arg::Num(d);
            c.residual(sk, &v)[0]
        };
        let d = self.args[at].num();
        let (mut d0, mut d1) = (d, d + d.abs().max(1.0));
        let (mut r0, mut r1) = (residual(d0), residual(d1));
        for _ in 0..40 {
            if r1 == r0 || r1 == 0.0 || !r1.is_finite() {
                break;
            }
            let d2 = d1 - r1 * (d1 - d0) / (r1 - r0);
            (d0, r0) = (d1, r1);
            d1 = d2;
            r1 = residual(d1);
            if (d1 - d0).abs() <= 1e-15 * d1.abs().max(1.0) {
                break;
            }
        }
        Some(d1)
    }

    /// A constraint of `kind` over the arguments given, **and the defaults for any the caller
    /// stopped short of**.
    ///
    /// One rule, in the one place every builder goes through: a slot appended to a type — a
    /// selector saying which side a magnitude is on (issue #48, item 4) — is a slot no existing
    /// caller knew to fill, and its default is what the type already meant without it.  A caller
    /// that gives *more* than the type has is a mistake, and still one.
    pub fn new(kind: CKind, args: Vec<Arg>) -> Constraint {
        let spec = kind.spec();
        debug_assert!(args.len() <= spec.len(), "{:?} arity", kind);
        let mut args = args;
        for i in args.len()..spec.len() {
            args.push(kind.default_arg(i));
        }
        Constraint {
            id: 0,
            kind,
            args,
            soft: false,
            intrinsic: false,
            claim: false,
            free: None,
            class: Default::default(),
            written: None,
            repeated: false,
            word: None,
            along: None,
            bound: None,
        }
    }

    pub fn coincident(p: EntRef, q: EntRef) -> Constraint {
        Constraint::new(CKind::Coincident, vec![Arg::Ent(p), Arg::Ent(q)])
    }

    pub fn distance(p: EntRef, q: EntRef, d: f64) -> Constraint {
        Constraint::new(CKind::Distance, vec![Arg::Ent(p), Arg::Ent(q), Arg::Num(d)])
    }

    pub fn one_line(kind: CKind, line: EntRef) -> Constraint {
        Constraint::new(kind, vec![Arg::Ent(line)])
    }

    pub fn two_line(kind: CKind, l1: EntRef, l2: EntRef) -> Constraint {
        Constraint::new(kind, vec![Arg::Ent(l1), Arg::Ent(l2)])
    }

    pub fn point_on_circle(p: EntRef, circle: EntRef, intrinsic: bool) -> Constraint {
        let mut c = Constraint::new(CKind::PointOnCircle, vec![Arg::Ent(p), Arg::Ent(circle)]);
        c.intrinsic = intrinsic;
        c
    }

    pub fn radius(circle: EntRef, r: f64) -> Constraint {
        Constraint::new(CKind::Radius, vec![Arg::Ent(circle), Arg::Num(r)])
    }

    pub fn drag_target(p: EntRef, tx: f64, ty: f64, weight: f64) -> Constraint {
        let mut c = Constraint::new(
            CKind::DragTarget,
            vec![Arg::Ent(p), Arg::Num(tx), Arg::Num(ty), Arg::Num(weight)],
        );
        c.soft = true;
        c
    }

    /// The soft target of a drag of a point in space, seen by the eye at bearing `az` and
    /// elevation `el` at (`tx`, `ty`) on its picture plane.
    /// `depth` is how far along the eye's line of sight (`right × up`) the point is held, weakly.
    pub fn drag_seen(p: EntRef, tx: f64, ty: f64, weight: f64, (az, el): (f64, f64), depth: f64)
        -> Constraint {
        let mut c = Constraint::new(
            CKind::DragSeen,
            vec![Arg::Ent(p), Arg::Num(tx), Arg::Num(ty), Arg::Num(weight), Arg::Num(az),
                 Arg::Num(el), Arg::Num(depth)],
        );
        c.soft = true;
        c
    }

    /// `TangentLineCircle` with the chirality flag read off the current geometry when `side` is
    /// `None`, so the solver keeps the circle on the side it already is.
    pub fn tangent_line_circle(
        sk: &Sketch,
        line: EntRef,
        circle: EntRef,
        side: Option<i64>,
    ) -> Constraint {
        let s = side.unwrap_or_else(|| {
            let l = &sk.lines[line.i()];
            let (ax, ay) = sk.point_xy(l.p1 as usize);
            let (bx, by) = sk.point_xy(l.p2 as usize);
            let (cx, cy) = sk.point_xy(sk.round_center(circle));
            let (dx, dy) = (bx - ax, by - ay);
            let (wx, wy) = (cx - ax, cy - ay);
            if dx * wy - dy * wx >= 0.0 {
                1
            } else {
                -1
            }
        });
        Constraint::new(
            CKind::TangentLineCircle,
            vec![Arg::Ent(line), Arg::Ent(circle), Arg::Str(side_word(s).to_string())],
        )
    }

    /// `TangentCircleCircle` with the sense read off the current geometry when `external` is
    /// `None`: whichever of |c1−c2| = r1+r2 (outside) and |c1−c2| = |r1−r2| (inside) the circles
    /// are already nearer to, so the solver keeps the arrangement the user drew.
    pub fn tangent_circle_circle(
        sk: &Sketch,
        c1: EntRef,
        c2: EntRef,
        external: Option<bool>,
    ) -> Constraint {
        let e = external.unwrap_or_else(|| {
            let (ax, ay) = sk.point_xy(sk.round_center(c1));
            let (bx, by) = sk.point_xy(sk.round_center(c2));
            let d = (ax - bx).dhypot(ay - by);
            let (r1, r2) = (sk.radius_value(c1).abs(), sk.radius_value(c2).abs());
            (d - (r1 + r2)).abs() <= (d - (r1 - r2).abs()).abs()
        });
        Constraint::new(
            CKind::TangentCircleCircle,
            vec![Arg::Ent(c1), Arg::Ent(c2), Arg::Bool(e)],
        )
    }

    /// A point on a curve, starting at the curve parameter nearest where the point already is.
    pub fn point_on_spline(sk: &Sketch, p: EntRef, spline: EntRef) -> Constraint {
        Constraint::contact(sk, CKind::PointOnSpline, Arg::Ent(p), Arg::Ent(spline))
    }

    /// A line tangent to a curve, starting where the curve already comes nearest that line.
    pub fn spline_tangent_line(sk: &Sketch, spline: EntRef, line: EntRef) -> Constraint {
        Constraint::contact(sk, CKind::SplineTangentLine, Arg::Ent(spline), Arg::Ent(line))
    }

    /// A circle that osculates a curve — the curve's own radius there — starting at the place
    /// the circle's centre is already nearest.
    pub fn spline_curvature(sk: &Sketch, spline: EntRef, circle: EntRef) -> Constraint {
        Constraint::contact(sk, CKind::SplineCurvature, Arg::Ent(spline), Arg::Ent(circle))
    }

    /// Two points as images of one point in space — the planes read off their memberships,
    /// through the same seam the document readers use (`io::seed_omitted`), so a Rust caller
    /// and a document are refused by one rule.
    pub fn project(sk: &Sketch, a: EntRef, b: EntRef) -> Result<Constraint, String> {
        let mut args = vec![Arg::Ent(a), Arg::Ent(b), Arg::Num(0.0), Arg::Num(0.0)];
        crate::io::seed_omitted(sk, CKind::Project, &mut args, |i| i >= 2)?;
        Ok(Constraint::new(CKind::Project, args))
    }

    /// A relation in space over drawn entities: the number where the kind states one, and what
    /// the core reads off the geometry (a skew distance's side) filled in and checked through
    /// `io::seed_omitted`, the one rule a document and a Rust caller are refused by.  The hidden
    /// points are minted when the constraint is added.
    pub fn in_space(sk: &Sketch, kind: CKind, ents: &[EntRef], value: Option<f64>)
        -> Result<Constraint, String>
    {
        let spec = kind.spec();
        let mut ents = ents.iter();
        let mut args: Vec<Arg> = Vec::with_capacity(spec.len());
        for (i, (_, k)) in spec.iter().enumerate() {
            args.push(match k {
                k if k.is_entity() => Arg::Ent(*ents.next().ok_or("too few operands")?),
                k if k.is_dimension() => Arg::Num(value.ok_or("this relation states a number")?),
                _ => kind.default_arg(i),
            });
        }
        crate::io::seed_omitted(sk, kind, &mut args, |i| kind.infers_arg(i))?;
        Ok(Constraint::new(kind, args))
    }

    /// An ordinate (`docs/ordinate-plan.md`): how far `q` stands from `p` along `t` — an axis,
    /// a line, or a plane standing for its normal.  Its form is read when it is added.
    pub fn ordinate(p: EntRef, q: EntRef, t: EntRef, d: f64) -> Constraint {
        Constraint::new(CKind::Ordinate, vec![Arg::Ent(p), Arg::Ent(q), Arg::Ent(t), Arg::Num(d)])
    }

    /// The two level along `t`: the ordinate's zero.
    pub fn level(p: EntRef, q: EntRef, t: EntRef) -> Constraint {
        Constraint::new(CKind::Level, vec![Arg::Ent(p), Arg::Ent(q), Arg::Ent(t)])
    }

    /// A two-entity curve contact whose parameter starts where the geometry puts it.
    fn contact(sk: &Sketch, kind: CKind, a: Arg, b: Arg) -> Constraint {
        let mut args = vec![a, b, Arg::Num(0.0)];
        args[2] = Arg::Num(seed_param(sk, kind, &args, 2));
        Constraint::new(kind, args)
    }

    pub fn kernel_id(&self) -> usize {
        self.kernel() as usize
    }

    /// Which kernel evaluates this constraint, when the sketch is at hand.
    ///
    /// The static one (`kernel_id`) for every type but those whose kernel is built from the
    /// sketch.  A curve contact's belongs to the curve's *definition* — different families read
    /// different numbers of coordinates, so they cannot share a block — a spline's length to its
    /// control-point count, and a derivative to its row's kernel.  `System` builds one kernel per
    /// key present (`system::build_kernel`) and orders its blocks by key, so the derived order
    /// keeps every static block ahead of every built one.
    pub fn kernel_key(&self, sk: &Sketch) -> kernels::KernelKey {
        use kernels::KernelKey as Key;
        // a derivative's kernel is its row's kernel's twin (§6.21)
        if self.along.is_some() {
            return Key::Dual(self.kernel_id());
        }
        if self.kind == CKind::SplineLength {
            let n = sk.splines[self.args[0].ent().i()].ctrl.len();
            return Key::SplineLength { n, free: self.free.is_some() };
        }
        // a touch compiles no row (`rows_in`), and is keyed with its curve's energy
        if matches!(self.kind, CKind::Stationary | CKind::CurveTouchesLine) {
            return Key::Stationary(self.id);
        }
        // an envelope contact's kernel is its block's widths' (#283)
        if let Some(c) = self.kind.envelope_contact() {
            let parts = self.envelope_parts(sk);
            let (n_par, n_const) = parts.map_or((0, 0), |p| (p.width(c), p.n_const()));
            return Key::Envelope { fk: c as u8, n_par, n_const };
        }
        if let Some(c) = self.kind.meshing_contact() {
            let parts = self.meshing_parts(sk);
            let (n_par, n_const) = parts.map_or((0, 0), |p| (p.width(sk, c.lead()), p.n_const(sk)));
            return Key::Meshing { contact: c as u8, n_par, n_const };
        }
        // a contact with a traced body, at its block's widths (#326)
        if let Some(c) = self.kind.traced_contact() {
            let t = self.traced_of().and_then(|e| sk.traced.get(e.i()));
            let (n_par, n_const) = t.map_or((0, 0), |t| {
                (c.lead() + crate::traced::PLACE + t.body.n_theta(), crate::traced::consts_of(t).len())
            });
            return Key::Traced { contact: c as u8, n_par, n_const };
        }
        // a motion's twist read, at its block's widths (#326)
        if let Some(r) = self.kind.motion_reading() {
            let parts = self.motion_parts(sk);
            let (n_par, n_const) = parts.map_or((0, 0), |p| {
                (r.lead() + p.width(), crate::envelope::screw::n_const(&p.steps))
            });
            return Key::Motion { reading: r as u8, n_par, n_const };
        }
        match (self.kind.family_kernel(), self.curve_of()) {
            (Some(fk), Some(e)) => Key::Family { def: sk.curves[e.i()].def as usize, fk: fk as u8 },
            _ => Key::Static(self.kernel_id()),
        }
    }

    /// The kernel itself (`kernel_key`, built where it must be).
    pub fn kernel_in(&self, sk: &Sketch) -> Kernel {
        crate::system::build_kernel(sk, self.kernel_key(sk))
    }

    /// Rows this constraint compiles to, when the sketch is at hand: `n_residuals` for every kind
    /// whose rows are a fact about it; an energy's — its curve's length stationary, one row on its
    /// first statement where nothing holds the length — and a peg's, none (`variational.rs`).
    pub fn rows_in(&self, sk: &Sketch) -> usize {
        match self.kind {
            CKind::Stationary => crate::variational::rows(sk, self.id),
            // a held point a free curve passes is its problem's, not a row of the drawing's
            CKind::PointOnCurve if sk.is_peg(self.id) => 0,
            _ => self.n_residuals(),
        }
    }

    /// The envelope a contact with one is on (#283) — the spec's `Envelope` slot.
    pub fn envelope_of(&self) -> Option<EntRef> {
        let (e, _) = self.kind.contact_on(SpecKind::Envelope)?;
        match self.args.get(e)? {
            Arg::Ent(e) if e.kind == EntKind::Envelope => Some(*e),
            _ => None,
        }
    }

    /// The meshing a point on one is held to (#283) — the spec's `Meshing` slot.
    pub fn meshing_of(&self) -> Option<EntRef> {
        let (e, _) = self.kind.contact_on(SpecKind::Meshing)?;
        match self.args.get(e)? {
            Arg::Ent(e) if e.kind == EntKind::Meshing => Some(*e),
            _ => None,
        }
    }

    /// What a point on a meshing reads of it (`envelope::meshing::Parts`), where the solve can
    /// read it.
    pub fn meshing_parts(&self, sk: &Sketch) -> Option<crate::envelope::meshing::Parts> {
        crate::envelope::meshing::Parts::of(sk, self.meshing_of()?.i()).ok()
    }

    /// The traced body a contact with one is on (#326) — the spec's `Traced` slot.
    pub fn traced_of(&self) -> Option<EntRef> {
        let i = self.kind.spec().iter().position(|&(_, k)| k == SpecKind::Traced)?;
        match self.args.get(i)? {
            Arg::Ent(e) if e.kind == EntKind::Traced => Some(*e),
            _ => None,
        }
    }

    /// The motion a relation reading its twist names (#326) — the spec's `Motion` slot.
    pub fn motion_of(&self) -> Option<EntRef> {
        let i = self.kind.spec().iter().position(|&(_, k)| k == SpecKind::Motion)?;
        match self.args.get(i)? {
            Arg::Ent(e) if e.kind == EntKind::Motion => Some(*e),
            _ => None,
        }
    }

    /// What a relation reading a motion reads of it (`envelope::held::Moving`), where the solve can
    /// read it (#326).
    pub fn motion_parts(&self, sk: &Sketch) -> Option<crate::envelope::held::Moving> {
        crate::envelope::held::Moving::of_any(sk, self.motion_of()?.i()).ok()
    }

    /// What a contact reads of its envelope (`envelope::held::Parts`), where the solve can read it.
    pub fn envelope_parts(&self, sk: &Sketch) -> Option<crate::envelope::held::Parts> {
        crate::envelope::held::Parts::of(sk, self.envelope_of()?.i()).ok()
    }

    /// The curve a per-definition kernel is over — the spec's `Curve` slot, wherever it stands.
    pub fn curve_of(&self) -> Option<EntRef> {
        let (e, _) = self.kind.contact_on(SpecKind::Curve)?;
        Some(self.args[e].ent())
    }

    /// Which kernel evaluates this constraint: its type's, or the free-variable twin when the
    /// number it states is an unknown rather than a constant.
    pub(crate) fn kernel(&self) -> K {
        let free = self.free.is_some();
        // an ordinate is read by the kernel its operands give it (`OrdinateForm`), and its zero
        // by the same at no distance — but along a view's own axis by the line kernel that
        // levels a pair of points
        if let Some(form) = self.form() {
            use OrdinateForm as F;
            return match (self.kind, form, free) {
                (CKind::Level, F::PageU, _) => K::Vertical,
                (CKind::Level, F::PageV, _) => K::Horizontal,
                (_, F::CoordU | F::CoordV, false) => K::Radius,
                (_, F::CoordU | F::CoordV, true) => K::RadiusFree,
                (_, F::PageU, false) => K::OrdinateU,
                (_, F::PageU, true) => K::OrdinateUFree,
                (_, F::PageV, false) => K::OrdinateV,
                (_, F::PageV, true) => K::OrdinateVFree,
                (_, F::InView, false) => K::OrdinateLine,
                (_, F::InView, true) => K::OrdinateLineFree,
                (_, F::Space, false) => K::OrdinateSpace,
                (_, F::Space, true) => K::OrdinateSpaceFree,
                (_, F::FrameU, false) => K::OrdinateFrameU,
                (_, F::FrameU, true) => K::OrdinateFrameUFree,
                (_, F::FrameV, false) => K::OrdinateFrameV,
                (_, F::FrameV, true) => K::OrdinateFrameVFree,
                (_, F::FrameN, false) => K::OrdinateFrameN,
                (_, F::FrameN, true) => K::OrdinateFrameNFree,
            };
        }
        // a side left unsaid is the magnitude form: both sides are solutions, and where the solve
        // lands among them is the seed's business (§9.2)
        if self.side().is_none() {
            if let Some(k) = self.kind.magnitude_kernel(free) {
                return k;
            }
        }
        match free {
            true => self.kind.free_kernel().expect("a dimension has a free kernel"),
            false => self.kind.kernel(),
        }
    }

    /// The number a *cluster* has to place this dimension at: what it states, signed.
    ///
    /// The cluster vocabulary is signed where the language is not: a PL edge places a point at
    /// `n·p − c = v`, so it needs to know which side of the line to put it on.  Where the
    /// statement pins a side, that is the answer; where it does not, both sides are solutions and
    /// the one the drawing is *on* is the answer, because a plan moves a figure that already
    /// satisfies its constraints rather than choosing among their solutions (§9.2).  Every other
    /// type states its own number and reads it straight.
    pub fn signed_gap(&self, sk: &Sketch) -> f64 {
        let d = self.dimensions().first().map_or(0.0, |&(i, _, _)| self.args[i].num());
        // an ordinate is signed already, its word turning it: there is no line to read a side off
        if self.kind == CKind::Ordinate {
            return self.side().unwrap_or(1.0) * d;
        }
        let Some(_) = self.kind.side_slot() else { return d };
        if let Some(s) = self.side() {
            return s * d;
        }
        // the same reading each kernel takes: a point against its line, and for a pair of lines
        // the second's first endpoint against the first
        let (p, line) = match self.kind {
            CKind::PointLineDistance => (self.args[0].ent().i(), self.args[1].ent().i()),
            _ => (sk.lines[self.args[1].ent().i()].p1 as usize, self.args[0].ent().i()),
        };
        let (px, py) = sk.point_xy(p);
        let now = crate::model::signed_point_to_line(sk, px, py, line);
        if now < 0.0 {
            -d
        } else {
            d
        }
    }

    /// An ordinate's form (`OrdinateForm`), or `None` for every other kind.
    pub fn form(&self) -> Option<OrdinateForm> {
        let i = self.kind.form_slot()?;
        Some(match self.args.get(i) {
            Some(Arg::Int(f)) => OrdinateForm::from_int(*f),
            _ => OrdinateForm::Space,
        })
    }

    /// Whether this statement reads its points where they stand in space — a relation in space,
    /// or an ordinate whose form is one — so `Sketch::add` mints their lifts.
    pub fn reads_space(&self) -> bool {
        self.kind.spatial() || self.form().is_some_and(|f| f.in_space())
    }

    /// Which way an angle turns: `sense: cw` is the minus a drawing no longer writes (§9.4), and
    /// an angle with no sense written turns the way the language counts, counter-clockwise.
    pub fn sense(&self) -> f64 {
        self.side().unwrap_or(1.0)
    }

    /// Which side the statement named, as the sign the signed kernel wants: `None` where it named
    /// none, and where the type has no side to name.
    pub fn side(&self) -> Option<f64> {
        let (i, table) = self.kind.side_words()?;
        let Arg::Str(w) = &self.args[i] else { return None };
        table.iter().find(|(n, _)| n == w).map(|(_, s)| *s)
    }

    pub fn n_residuals(&self) -> usize {
        // a curve's kernel belongs to its definition, so its row count is a fact about the kind
        match self.kind.family_kernel() {
            Some(fk) => fk.n_res(),
            None if self.kind == CKind::SplineLength => 1,
            // its curve's, which only the sketch can say: `rows_in`
            None if self.kind == CKind::Stationary => 0,
            // its curve's problem's: never a row of the drawing's
            None if self.kind == CKind::CurveTouchesLine => 0,
            None => match self.kind.envelope_contact() {
                // an envelope contact's: a fact about the contact it states (#283)
                Some(c) => c.rows(),
                None if self.kind.meshing_contact().is_some() => {
                    self.kind.meshing_contact().map_or(0, |c| c.rows())
                }
                None if self.kind.traced_contact().is_some() => {
                    self.kind.traced_contact().map_or(0, |c| c.rows())
                }
                None if self.kind.motion_reading().is_some() => {
                    self.kind.motion_reading().map_or(0, |r| r.rows())
                }
                None => kernels::kernel(self.kernel()).n_res,
            },
        }
    }

    pub fn spec(&self) -> Spec {
        self.kind.spec()
    }

    pub fn type_name(&self) -> &'static str {
        self.kind.name()
    }

    /// Entities this constraint references directly, in spec order.
    pub fn entities(&self) -> Vec<EntRef> {
        self.kind
            .spec()
            .iter()
            .zip(&self.args)
            .filter(|((_, k), _)| k.is_entity())
            .map(|(_, a)| a.ent())
            .collect()
    }

    /// A tangent at a point already on its curve (#251): the slot holding its place, the point
    /// and the curve — the contact holding that point there owns the place (`Sketch::add`).
    pub fn tangent_at(&self) -> Option<(&'static [usize], usize, EntRef)> {
        let slots: &'static [usize] = match self.kind {
            CKind::SplineTangentAt | CKind::CurveTangentAt => &[3],
            // an envelope's place is three numbers, all the contact's (#283)
            CKind::EnvelopeTangentAt
            | CKind::EnvelopeNormalAt
            | CKind::EnvelopePlaneAt
            | CKind::EnvelopeCurvatureAt => &[3, 4, 5],
            // a meshing's is its phase, its lag and a place on each flank (#345)
            CKind::MeshingNormalAt | CKind::MeshingTangentAt => &[3, 4, 5, 6, 7, 8, 9, 10],
            // a traced body's place is its two formals' (#326)
            CKind::TracedTangentAt
            | CKind::TracedSurfaceTangentAt
            | CKind::TracedNormalAt
            | CKind::TracedPlaneAt
            | CKind::TracedCurvatureAt => &[3, 4],
            _ => return None,
        };
        match (&self.args[0], &self.args[2]) {
            (Arg::Ent(on), Arg::Ent(at)) => Some((slots, at.i(), *on)),
            _ => None,
        }
    }

    /// A contact holding a point on a spline, a curve, an envelope or a meshing — the point, what
    /// it is on and its place, once allocated: what a tangent at that point reads its place from
    /// (#251).
    pub fn holds_on(&self) -> Option<(usize, EntRef, Vec<u32>)> {
        if !self.states_rows() {
            return None;
        }
        match (self.kind, &self.args[..]) {
            (CKind::PointOnSpline | CKind::PointOnCurve, [Arg::Ent(at), Arg::Ent(on), Arg::Param(p), ..]) => {
                Some((at.i(), *on, vec![*p]))
            }
            (CKind::PointOnEnvelope, [Arg::Ent(at), Arg::Ent(on), Arg::Param(u), Arg::Param(v), Arg::Param(t)]) => {
                Some((at.i(), *on, vec![*u, *v, *t]))
            }
            (CKind::PointOnMeshing, [Arg::Ent(at), Arg::Ent(on), place @ ..]) => {
                let place: Option<Vec<u32>> = place.iter().map(|a| match a {
                    Arg::Param(p) => Some(*p),
                    _ => None,
                }).collect();
                Some((at.i(), *on, place?))
            }
            (CKind::PointOnTraced, [Arg::Ent(at), Arg::Ent(on), Arg::Param(u), Arg::Param(s)]) => {
                Some((at.i(), *on, vec![*u, *s]))
            }
            _ => None,
        }
    }

    /// Whether this constraint owns the Param `p` — `aux_params` asked of one index, building
    /// nothing.
    pub fn owns(&self, p: u32) -> bool {
        let spec = self.kind.spec();
        spec.iter().zip(&self.args).any(|((_, k), a)| k.is_param() && *a == Arg::Param(p))
    }

    /// The Params this constraint owns — empty until `Sketch::add` has allocated them.
    pub fn aux_params(&self) -> Vec<u32> {
        self.kind
            .param_slots()
            .iter()
            .filter_map(|&(i, _)| match self.args[i] {
                Arg::Param(p) => Some(p),
                _ => None,
            })
            .collect()
    }

    /// The (index, name, kind) of this constraint's dimension values.
    /// Every dimension slot, by index, name and kind — a kind has at most one
    /// (`CKind::dimension_slot`).
    pub fn dimensions(&self) -> Vec<(usize, &'static str, SpecKind)> {
        self.kind
            .spec()
            .iter()
            .enumerate()
            .filter(|(_, (_, k))| k.is_dimension())
            .map(|(i, (n, k))| (i, *n, *k))
            .collect()
    }

    pub fn arg_index(&self, name: &str) -> Option<usize> {
        self.kind.spec().iter().position(|(n, _)| *n == name)
    }

    pub fn get_num(&self, name: &str) -> Option<f64> {
        self.arg_index(name).map(|i| self.args[i].num())
    }

    /// Set a numeric-ish argument by name — a dimension, a flag, a count.  `false` if there is no
    /// such argument or it is not one a number can express, rather than overwriting a string
    /// argument with `NaN`.  A dimension written as an expression becomes this plain number:
    /// whoever sets a number means the number, not the formula it replaces.
    ///
    /// This is the write on the constraint alone, and dropping an expression is a change to the
    /// *document*: the unknown it read is retired only when nothing else reads it.
    /// `Sketch::set_constraint_num` is the path that settles that, and is what a caller holding a
    /// sketch should use; this one is for a constraint that has no document behind it yet, or an
    /// argument no expression can reach (a soft drag target's own number).
    pub fn set_num(&mut self, name: &str, v: f64) -> bool {
        let Some(i) = self.arg_index(name) else { return false };
        self.args[i] = match self.args[i] {
            Arg::Int(_) => Arg::Int(v as i64),
            Arg::Bool(_) => Arg::Bool(v != 0.0),
            Arg::Num(_) | Arg::Expr(_) => Arg::Num(v),
            _ => return false,
        };
        // no expression left to read a name, so no unknown to be written in terms of: a binding
        // that outlived its text would have this constraint compiled against a column it no
        // longer has anything to say about
        self.free = None;
        self.written = None;
        true
    }

    /// The expression text behind a dimension argument, if it was written as one.
    pub fn expr_text(&self, name: &str) -> Option<&str> {
        match self.arg_index(name).map(|i| &self.args[i]) {
            Some(Arg::Expr(e)) => Some(&e.text),
            _ => None,
        }
    }

    /// Set a string argument by name (an arc tangency's end).  `false` if there is no such
    /// argument or it is not a string.
    pub fn set_str(&mut self, name: &str, v: &str) -> bool {
        let Some(i) = self.arg_index(name) else { return false };
        if !matches!(self.args[i], Arg::Str(_)) {
            return false;
        }
        self.args[i] = Arg::Str(v.to_string());
        true
    }

    /// Move a drag target's target point.  No other kind has one, and several are shorter than
    /// three arguments, so the kind is checked rather than the write being attempted blind.
    pub fn set_target(&mut self, tx: f64, ty: f64) -> bool {
        if !matches!(self.kind, CKind::DragTarget | CKind::DragSeen) {
            return false;
        }
        self.args[1] = Arg::Num(tx);
        self.args[2] = Arg::Num(ty);
        true
    }

    pub fn local_values(&self, sk: &Sketch) -> Vec<f64> {
        self.params(sk).iter().map(|&i| sk.params[i as usize].value).collect()
    }

    /// Current residual norm — convenience for reporting and tests.
    pub fn error(&self, sk: &Sketch) -> f64 {
        let v = self.local_values(sk);
        let (r, _) = self.eval(sk, &v);
        crate::linalg::norm(&r)
    }

    pub fn residual(&self, sk: &Sketch, v: &[f64]) -> Vec<f64> {
        // the residual alone: a finite difference asks for it once a column, and a Jacobian can
        // cost many residuals (a path of contact's, #345)
        let k = self.kernel_in(sk);
        let mut r = vec![0.0; k.n_res];
        (k.res)(1, v, &self.consts(sk), &mut r);
        r
    }

    /// n_res x n_par, row-major.
    pub fn jacobian(&self, sk: &Sketch, v: &[f64]) -> Vec<f64> {
        self.eval(sk, v).1
    }

    /// The residual and Jacobian at `v`, by the kernel this row runs: its own, or its
    /// derivative's (§6.21).
    fn eval(&self, sk: &Sketch, v: &[f64]) -> (Vec<f64>, Vec<f64>) {
        kernels::eval_with(&self.kernel_in(sk), v, &self.consts(sk))
    }
}
