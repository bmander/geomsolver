//! **An equation of integrals, stated or claimed of the drawing** (§6.25): `integral(density(p)
//! over p in arm) == 10kg`, `claim integral(1 over p in web) <= 42cm^3` — and every word
//! `std.measures` writes in them (`mass(10kg) arm`, `g centroid hull`).  Each answers to no load,
//! so it is no scenario's: it is a statement of the drawing, read once every beam and solid is
//! built (`elaborate::finish`), since a beam is read last and is no entity an operand could
//! otherwise name.  Stated, it is a row (`CKind::Integral`) the solve holds; claimed, it is judged
//! on the solved drawing (`measured::read`, `report::measure_answers`).
//!
//! Each integral is read into the model: what it is over (a solid, a face of one, a beam or a
//! line one is drawn on, an instance's bodies), its body as an expression over its point's
//! numbers, the density there and what it reads of the drawing (`model::Reading`), and its
//! dimension, its body's times its set's measure's; each side an expression over the integrals,
//! numbers and the unknowns the drawing declares, its dimension agreeing with the other's.

use super::resolve::{Operand, Resolver};
use super::source_map::{Made, SourceMap};
use super::{Code, Diag};
use crate::constraints::{Arg, CKind, Constraint};
use crate::expr::{Ast, Measure, Op};
use crate::ir::{Operation as StmtKind, Statement};
use crate::model::{integral_var, integrand_var, Applied, Domain, EntKind, EntRef, Integral,
                   Measured, MeshedKey, Reading, Sketch, POINT_VARS};
use crate::syntax::{Measurement, Measuring, Ref, Span, StmtId};
use crate::units::Dim;

/// **An equation and where it was written**: its statement, what a report calls it, its index
/// on `Sketch::measured`, and its row where it drives one.
#[derive(Clone, Debug)]
pub struct MeasureStated {
    pub stmt: StmtId,
    pub head: String,
    pub index: usize,
    pub driven: Option<u32>,
}

/// Why an equation cannot be read, and where.
type Refused = (Option<Span>, String);

/// **Every equation of integrals the drawing states**, in the order written: each read onto
/// `Sketch::measured`, a stated one compiled to its row (refused where the solve cannot read it,
/// **E040**), a claimed one left to be judged.  What cannot be read is said where it is written.
pub(super) fn measures(
    sk: &mut Sketch,
    res: &Resolver,
    map: &mut SourceMap,
    stating: &[&Statement],
    diags: &mut Vec<Diag>,
) -> Vec<MeasureStated> {
    let mut out = Vec::new();
    // equations over the same bodies fill them once where the drawing stands
    let mut fills = crate::measured::Fills::default();
    for st in stating {
        let StmtKind::Measured(m) = &st.kind else { continue };
        if !m.integrates() {
            continue;
        }
        let measured = match read(sk, res, m) {
            Ok(x) => x,
            Err((span, message)) => {
                diags.push(Diag::within(Code::E040, span.unwrap_or(st.span), st.id, message));
                continue;
            }
        };
        let head = m.head.clone().unwrap_or_else(|| {
            format!("{} {} {}", m.lhs.text(), m.cmp_text(), m.rhs.text())
        });
        let (index, claim) = (sk.measured.len(), measured.claim);
        sk.measured.push(measured);
        let driven = match claim {
            true => None,
            false => match drive(sk, map, st, index, &head, &mut fills, diags) {
                Some(id) => Some(id),
                None => continue,
            },
        };
        out.push(MeasureStated { stmt: st.id, head, index, driven });
    }
    if out.iter().any(|m| m.driven.is_some()) {
        // the unknowns the rows read are bound once the rows read them
        crate::expr::evaluate(sk);
    }
    // a claim reads what the drawing determines, and binds nothing itself
    out.retain(|ms| {
        let m = &sk.measured[ms.index];
        let Some(u) = m.unknowns.iter().find(|u| m.claim && !sk.free_vars.contains_key(*u))
        else { return true };
        let st = stating.iter().find(|st| st.id == ms.stmt).expect("its statement");
        diags.push(Diag::on(Code::E040, st, format!(
            "`{}` reads `{u}`, an unknown nothing determines, and a claim binds none: state it, \
             or claim it of what the drawing determines", ms.head)));
        false
    });
    out
}

/// **An equation's statement read into the model**, or why it cannot be: each integral over a
/// set the solve can read, its body over its point; each side over the integrals, numbers and
/// declared unknowns, the two of one dimension; a bound only where it is claimed — a bound on an
/// integral chooses no root.
fn read(sk: &Sketch, res: &Resolver, m: &crate::syntax::Measured) -> Result<Measured, Refused> {
    let compare = match m.cmp {
        None => crate::expr::Cmp::Eq,
        Some(_) if !m.claim => return Err((None, "a bound on an integral chooses no root: claim \
            it, or state it equal".into())),
        Some(crate::syntax::Cmp::Le) => crate::expr::Cmp::Le,
        Some(crate::syntax::Cmp::Ge) => crate::expr::Cmp::Ge,
        Some(crate::syntax::Cmp::In) => return Err((None, "an integral is compared one-sided, \
                                                           `<=` or `>=`".into())),
    };
    let mut r = Reader { sk, res, integrals: Vec::new(), unknowns: Vec::new() };
    let (lhs, ld) = r.side(&m.lhs)?;
    let (rhs, rd) = r.side(&m.rhs)?;
    let dim = agree(ld, rd).ok_or_else(|| (Some(m.span), format!(
        "its sides are of two dimensions: `{}` is {} and `{}` is {}", m.lhs.text(), ld.name(),
        m.rhs.text(), rd.name())))?;
    let Reader { integrals, unknowns, .. } = r;
    if integrals.len() + unknowns.len() > crate::tape::MAX_VARS {
        return Err((Some(m.span), format!("it reads {} integrals and unknowns, and a row reads at \
                                           most {}", integrals.len() + unknowns.len(),
                                          crate::tape::MAX_VARS)));
    }
    Ok(Measured { integrals, lhs, rhs, unknowns, compare, claim: m.claim, dim })
}

/// Two dimensions that may be compared or added: equal, or one a bare number, which stands
/// anywhere; the one they come to.
fn agree(a: Dim, b: Dim) -> Option<Dim> {
    match (a.is_scalar(), b.is_scalar()) {
        (true, _) => Some(b),
        (_, true) => Some(a),
        _ => (a == b).then_some(a),
    }
}

/// An expression's dimension, its names the drawing's declared unknowns.
fn dim_of(sk: &Sketch, a: &Ast) -> Result<Dim, String> {
    Ok(match a {
        Ast::Num(_, d) => *d,
        Ast::Var(n) => sk.declared.get(n).map(|d| d.dim)
            .ok_or_else(|| format!("`{n}` is no number of the drawing"))?,
        Ast::Neg(x) => dim_of(sk, x)?,
        Ast::Bin(op, x, y) => binary(*op, dim_of(sk, x)?, dim_of(sk, y)?, y)?,
        Ast::Call(f, args) => call(f, &args.iter().map(|x| dim_of(sk, x)).collect::<Result<
            Vec<_>, _>>()?)?,
        Ast::Measure(..) | Ast::If(..) => return Err("an equation's number is arithmetic over \
                                                       numbers and unknowns".into()),
    })
}

/// What an operator makes of its operands' dimensions, `y` the right's expression (a power's
/// exponent, a number).
fn binary(op: Op, a: Dim, b: Dim, y: &Ast) -> Result<Dim, String> {
    match op {
        Op::Add | Op::Sub => agree(a, b).ok_or_else(|| {
            format!("a sum of {} and {}", a.name(), b.name())
        }),
        Op::Mul => Ok(a.mul(b)),
        Op::Div => Ok(a.div(b)),
        Op::Pow => match constant(y) {
            Some(k) if b.is_scalar() => a.powf(k).ok_or_else(|| {
                format!("{} to the power {k}", a.name())
            }),
            _ if a.is_scalar() && b.is_scalar() => Ok(Dim::SCALAR),
            _ => Err("a power of a dimension is to a number".into()),
        },
    }
}

/// An expression's value where it is a plain number's arithmetic (an exponent, `(-3)`).
fn constant(a: &Ast) -> Option<f64> {
    match a {
        Ast::Num(v, d) if d.is_scalar() => Some(*v),
        Ast::Neg(x) => Some(-constant(x)?),
        Ast::Bin(op, x, y) => {
            let (x, y) = (constant(x)?, constant(y)?);
            Some(match op {
                Op::Add => x + y,
                Op::Sub => x - y,
                Op::Mul => x * y,
                Op::Div => x / y,
                Op::Pow => x.powf(y),
            })
        }
        _ => None,
    }
}

/// What a function makes of its arguments' dimensions: a root halves, a size keeps the first's,
/// anything else is of plain numbers.
fn call(f: &str, args: &[Dim]) -> Result<Dim, String> {
    match (f, args) {
        ("sqrt", [a]) => Ok(a.sqrt()),
        ("abs" | "min" | "max" | "hypot", [a, ..]) => Ok(*a),
        (_, args) if args.iter().all(|a| a.is_scalar()) => Ok(Dim::SCALAR),
        _ => Err(format!("`{f}` is of plain numbers")),
    }
}

/// **What reads an equation**: the drawing and its names, and the integrals and unknowns its
/// sides have read so far, in the order written.
struct Reader<'a> {
    sk: &'a Sketch,
    res: &'a Resolver,
    integrals: Vec<Integral>,
    unknowns: Vec<String>,
}

impl Reader<'_> {
    /// **One side**: an expression over its integrals (`#0`, `#1`, …), numbers and unknowns,
    /// and its dimension.
    fn side(&mut self, m: &Measuring) -> Result<(Ast, Dim), Refused> {
        match m {
            Measuring::Number { text, span } => {
                let a = self.number(text, *span)?;
                for n in a.deps() {
                    if !self.unknowns.contains(&n) {
                        self.unknowns.push(n);
                    }
                }
                let d = dim_of(self.sk, &a).map_err(|e| (Some(*span), format!("`{text}`: {e}")))?;
                Ok((a, d))
            }
            Measuring::Integral(i) => {
                let j = self.integrals.len();
                let (integral, d) = self.integral(i)?;
                self.integrals.push(integral);
                Ok((Ast::Var(integral_var(j)), d))
            }
            Measuring::Neg(a) => {
                let (a, d) = self.side(a)?;
                Ok((Ast::Neg(Box::new(a)), d))
            }
            Measuring::Bin(op, a, b) => {
                let (x, dx) = self.side(a)?;
                let (y, dy) = self.side(b)?;
                let d = binary(*op, dx, dy, &y)
                    .map_err(|e| (None, format!("`{}`: {e}", m.text())))?;
                Ok((Ast::Bin(*op, Box::new(x), Box::new(y)), d))
            }
            Measuring::Call(f, args) => {
                let mut asts = Vec::new();
                let mut dims = Vec::new();
                for a in args {
                    let (x, d) = self.side(a)?;
                    asts.push(x);
                    dims.push(d);
                }
                let d = call(&f.text, &dims).map_err(|e| (Some(f.span), e))?;
                Ok((Ast::Call(f.text.clone(), asts), d))
            }
            Measuring::Measure(x) => Err((Some(x.span), format!(
                "`{}` measures the drawing outside an integral: an equation of integrals compares \
                 integrals and numbers", x.text()))),
            Measuring::Density(_, span) => Err((Some(*span), "a density is read at an integral's \
                point: `integral(density(p) over p in B)`".into())),
        }
    }

    /// A number's text read as an expression in the document's units.
    fn number(&self, text: &str, span: Span) -> Result<Ast, Refused> {
        crate::expr::parse_in(text, self.sk.units).map(|p| p.body)
            .map_err(|e| (Some(span), format!("`{text}`: {e}")))
    }

    /// **One integral read**: what it is over, its body over its point and what the body reads —
    /// and its dimension, the body's times its set's measure's.
    fn integral(&mut self, i: &crate::syntax::Integration) -> Result<(Integral, Dim), Refused> {
        let over = self.domain(&i.over)?;
        let measure = match over.face() {
            Some(_) => Dim::LENGTH.mul(Dim::LENGTH),
            None => Dim::LENGTH.mul(Dim::LENGTH).mul(Dim::LENGTH),
        };
        let mut reads = Vec::new();
        let (integrand, d) = self.body(&i.body, &i.var.text, &mut reads)?;
        Ok((Integral { over, integrand, reads }, d.mul(measure)))
    }

    /// **What an integral is over**: a face of a solid, a beam, a solid or a line a beam is drawn
    /// on — or, where the name is none of the drawing's, every body under the instance it names.
    fn domain(&self, r: &Ref) -> Result<Domain, Refused> {
        let named = crate::syntax::ref_text(r);
        let what = |o: &Operand| match (o, o.ent()) {
            (Operand::Face(_), _) => "a face of no solid".to_string(),
            (_, Some(e)) => e.kind.a().to_string(),
            _ => "nothing an integral is over".to_string(),
        };
        match self.res.operand(self.sk, r) {
            Ok(Operand::Face(f)) => Ok(Domain::Of(Applied::Face(f))),
            Ok(Operand::Beam(b)) => Ok(Domain::Of(Applied::Beam(b))),
            Ok(Operand::Ent(e) | Operand::Body(e)) if e.kind == EntKind::Solid
                || (e.kind == EntKind::Line
                    && !crate::beams::members(self.sk, &Applied::Ent(e)).is_empty()) =>
            {
                Ok(Domain::Of(Applied::Ent(e)))
            }
            Ok(o) => Err((Some(r.span), format!(
                "an integral is over a solid, a face of one, a beam, a line a beam is drawn on or \
                 an instance's bodies, and `{named}` is {}", what(&o)))),
            Err(m) => match r.path.is_empty() {
                true => match self.res.bodies_under(self.sk, &r.root.text) {
                    b if !b.is_empty() => Ok(Domain::Bodies(b)),
                    _ => Err((Some(r.span), m)),
                },
                false => Err((Some(r.span), m)),
            },
        }
    }

    /// **An integral's body** over its point `var`: an expression over the point's numbers
    /// (`#x`, `#y`, `#z`), the density there (`#rho`) and the numbers of what it reads (`#r…`,
    /// each reading in `reads` once), and its dimension.
    fn body(&self, m: &Measuring, var: &str, reads: &mut Vec<Reading>)
        -> Result<(Ast, Dim), Refused>
    {
        match m {
            Measuring::Number { text, span } => {
                let a = self.number(text, *span)?;
                if let Some(n) = a.deps().into_iter().next() {
                    return Err((Some(*span), format!("an integral's body reads its point and \
                        numbers the drawing states, and `{n}` is an unknown: put it outside the \
                        integral")));
                }
                let d = dim_of(self.sk, &a).map_err(|e| (Some(*span), format!("`{text}`: {e}")))?;
                Ok((a, d))
            }
            Measuring::Density(r, span) => match is(r, var) {
                true => Ok((Ast::Var(integrand_var(3)), Dim::DENSITY)),
                false => Err((Some(*span), format!("a density is read at the integral's point: \
                                                    `density({var})`"))),
            },
            Measuring::Measure(x) => self.measurement(x, var, reads, false),
            // a distance squared is its square, never a root squared: smooth where it is nothing
            Measuring::Bin(Op::Pow, a, b)
                if matches!(&**a, Measuring::Measure(x) if x.measure == Measure::Distance
                            && x.along.is_none())
                    && matches!(&**b, Measuring::Number { text, .. } if text.trim() == "2") =>
            {
                let Measuring::Measure(x) = &**a else { unreachable!("a distance") };
                self.measurement(x, var, reads, true)
            }
            Measuring::Neg(a) => {
                let (a, d) = self.body(a, var, reads)?;
                Ok((Ast::Neg(Box::new(a)), d))
            }
            Measuring::Bin(op, a, b) => {
                let (x, dx) = self.body(a, var, reads)?;
                let (y, dy) = self.body(b, var, reads)?;
                let d = binary(*op, dx, dy, &y)
                    .map_err(|e| (None, format!("`{}`: {e}", m.text())))?;
                Ok((Ast::Bin(*op, Box::new(x), Box::new(y)), d))
            }
            Measuring::Call(f, args) => {
                let mut asts = Vec::new();
                let mut dims = Vec::new();
                for a in args {
                    let (x, d) = self.body(a, var, reads)?;
                    asts.push(x);
                    dims.push(d);
                }
                let d = call(&f.text, &dims).map_err(|e| (Some(f.span), e))?;
                Ok((Ast::Call(f.text.clone(), asts), d))
            }
            Measuring::Integral(i) => Err((Some(i.span), "an integral's body integrates nothing \
                more: write each integral on a side".into())),
        }
    }

    /// **A distance read at the integral's point** (§6.25), `squared` where its square is what
    /// is written: from the point to a point, a line or an axis, or a plane — or, `along:` a
    /// direction, the ordinate of its second point from its first.
    fn measurement(&self, x: &Measurement, var: &str, reads: &mut Vec<Reading>, squared: bool)
        -> Result<(Ast, Dim), Refused>
    {
        let refuse = |m: String| Err((Some(x.span), m));
        if x.measure != Measure::Distance {
            return refuse(format!("in an integral's body a measurement is a distance from its \
                                   point, and `{}` is none", x.text()));
        }
        let [a, b] = &x.args[..] else { return refuse("a distance is between two".into()) };
        if let Some(d) = &x.along {
            // the ordinate of `b` from `a` along `d`, made unit
            let e = self.entity(d)?;
            if !matches!(e.kind, EntKind::Axis | EntKind::Line | EntKind::Frame) {
                return refuse(format!("`along: {}` is no direction: an axis, a line or a plane's \
                                       normal", crate::syntax::ref_text(d)));
            }
            let dir = Reading::slot(reads, Reading::Direction(e));
            let pa = self.place(a, var, reads)?;
            let pb = self.place(b, var, reads)?;
            if !(is(a, var) || is(b, var)) {
                return refuse(format!("a distance in an integral's body is from its point, \
                                       `{var}`"));
            }
            let terms = (0..3).map(|c| mul(sub(pb[c].clone(), pa[c].clone()), read_var(dir + c)));
            return Ok((sum(terms), Dim::LENGTH));
        }
        let other = match (is(a, var), is(b, var)) {
            (true, false) => b,
            (false, true) => a,
            _ => return refuse(format!("a distance in an integral's body is from its point, \
                                        `{var}`, to something of the drawing")),
        };
        let p = point();
        let e = self.entity(other)?;
        let (sq, plane) = match e.kind {
            EntKind::Point => {
                let g = Reading::slot(reads, Reading::Point(e.i()));
                (sum((0..3).map(|c| square(sub(p[c].clone(), read_var(g + c))))), None)
            }
            EntKind::Line | EntKind::Axis | EntKind::Frame => {
                // the line's or the plane's point, and its direction or normal, made unit
                let at = Reading::slot(reads, Reading::Line(e));
                let q: Vec<Ast> = (0..3).map(|c| sub(p[c].clone(), read_var(at + c))).collect();
                let along = sum((0..3).map(|c| mul(q[c].clone(), read_var(at + 3 + c))));
                match e.kind {
                    EntKind::Frame => (square(along.clone()), Some(along)),
                    _ => (sub(sum(q.into_iter().map(square)), square(along)), None),
                }
            }
            k => return refuse(format!("a distance from an integral's point is to a point, a \
                                        line, an axis or a plane, and `{}` is {}",
                                       crate::syntax::ref_text(other), k.a())),
        };
        let a = match (squared, plane) {
            (true, _) => sq,
            (false, Some(n)) => Ast::Call("abs".into(), vec![n]),
            (false, None) => Ast::Call("sqrt".into(), vec![sq]),
        };
        Ok((a, if squared { Dim::LENGTH.mul(Dim::LENGTH) } else { Dim::LENGTH }))
    }

    /// A distance's end as three numbers: the integral's point, or a point of the drawing read.
    fn place(&self, r: &Ref, var: &str, reads: &mut Vec<Reading>) -> Result<[Ast; 3], Refused> {
        if is(r, var) {
            return Ok(point());
        }
        let e = self.entity(r)?;
        if e.kind != EntKind::Point {
            return Err((Some(r.span), format!("`{}` is no point: an ordinate is of a point from \
                                               a point", crate::syntax::ref_text(r))));
        }
        let g = Reading::slot(reads, Reading::Point(e.i()));
        Ok([read_var(g), read_var(g + 1), read_var(g + 2)])
    }

    /// The entity a reference names.
    fn entity(&self, r: &Ref) -> Result<EntRef, Refused> {
        match self.res.operand(self.sk, r).map_err(|m| (Some(r.span), m))? {
            Operand::Ent(e) => Ok(e),
            _ => Err((Some(r.span), format!("`{}` is none of the drawing's points, lines, axes or \
                                             planes", crate::syntax::ref_text(r)))),
        }
    }
}

/// Whether reference `r` names the integral's point `var`.
fn is(r: &Ref, var: &str) -> bool {
    r.path.is_empty() && r.root.text == var
}

/// The integral's point's three numbers.
fn point() -> [Ast; 3] {
    [0, 1, 2].map(|k| Ast::Var(integrand_var(k)))
}

/// A reading's number `k`.
fn read_var(k: usize) -> Ast {
    Ast::Var(integrand_var(POINT_VARS + k))
}

fn sub(a: Ast, b: Ast) -> Ast {
    Ast::Bin(Op::Sub, Box::new(a), Box::new(b))
}

fn mul(a: Ast, b: Ast) -> Ast {
    Ast::Bin(Op::Mul, Box::new(a), Box::new(b))
}

fn square(a: Ast) -> Ast {
    mul(a.clone(), a)
}

fn sum(terms: impl IntoIterator<Item = Ast>) -> Ast {
    terms.into_iter().reduce(|a, b| Ast::Bin(Op::Add, Box::new(a), Box::new(b)))
        .unwrap_or(Ast::Num(0.0, Dim::SCALAR))
}

/// An equation compiled to its row, `Sketch::measured[index]`: its mesh made once, here, and
/// held before the row is validated so the validation reads that mesh — or refused where it is
/// written (**E040**), the solve unable to read it.
fn drive(
    sk: &mut Sketch,
    map: &mut SourceMap,
    st: &Statement,
    index: usize,
    head: &str,
    fills: &mut crate::measured::Fills,
    diags: &mut Vec<Diag>,
) -> Option<u32> {
    let kind = CKind::Integral;
    let args = vec![Arg::Int(index as i64)];
    let key = MeshedKey::Measure(index);
    // the unknowns the row reads are its columns, bound from their seeds before it is made — a
    // waterline's height a volume sizes; `evaluate` keeps them bound once the row reads them
    let c = Constraint::new(kind, args.clone());
    for name in c.frame_unknowns(sk).unwrap_or_default() {
        if let Some(seed) = sk.declared.get(&name).and_then(|d| d.seed) {
            crate::expr::bind_free(sk, &name, seed);
        }
    }
    if let Ok(crate::constraints::InteriorRow::Meshed(row)) =
        crate::measured::row_in(sk, index, fills)
    {
        sk.meshed.insert(key, row);
    }
    let name = |e: EntRef| map.name_of(e).cloned().unwrap_or_else(|| crate::model::entity_name(e));
    if let Err(m) = crate::constraints::validate(sk, kind, &args, &name) {
        sk.meshed.remove(&key);
        diags.push(Diag::on(Code::E040, st, format!("`{head}` cannot drive the drawing: {m}")));
        return None;
    }
    let id = sk.add_quiet(Constraint::new(kind, args));
    map.record(st, Made::Con(id));
    Some(id)
}
