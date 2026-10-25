//! **Developable panels, recognised** (#645): what `std.Developable` and `std.Development` say,
//! read off a body by what it says — `std`'s, or one written by hand — and never by its name, as
//! the lowering table recognises a round (`solid/lowering`).
//!
//! * **A sheet** is a set whose body puts its point on the ruling between two points of its own
//!   (`p coincident(t in (0, 1)) line(m, n)`), each on a path (`m coincident a`, `n coincident b`),
//!   with one plane of its own touching both paths there (`T tangent(at: m) a`, `T tangent(at: n,
//!   sense: same) b`).  Its rails are `a` and `b` in that order — the order the panel's sides are
//!   read by — `b` run the other way where it runs through the plane against `a` (`sense:
//!   opposite`).  A band of it at a region solid's probe is built exactly (`RegionShape::Sheet`),
//!   never applied as a body.
//! * **A development** is a component handed a set, a point and a length whose body lays a ruling
//!   of the set in its plane by an isometry along rulings: the rates of two points of its own
//!   equal to the rates of two others (`rate(f over u) equal rate(m over u)`), and the first's
//!   angle to the ruling's image equal to the other's to the ruling (`rate(f over u)
//!   angle(rate(m over u), r) line(f, g)`).  Given a sheet, it is laid by the core
//!   (`model::Developed`): `f` and `g` drawn in its plane, the home ruling's ends, and the laid
//!   rails `a` and `b` curves of the drawing, with the end rulings between them.
use super::*;
use super::sets::written_of;
use crate::syntax::{OpArg, SetLit, Ty};

/// Whether a set's body holds its point on a stretch of a curve by an interval contact, `p
/// coincident(t in (a, b)) l` — the body's whole is then each use's (`instance_part`).
pub(super) fn within_point(lit: &SetLit) -> bool {
    lit.body.iter().filter_map(written_of).any(|w| {
        w.word.text == "coincident"
            && w.ops.first().is_some_and(|r| r.root.text == lit.bound.text && r.path.is_empty())
            && w.args.iter().any(|a| matches!(a, OpArg::Within { .. }))
    })
}

/// A sheet's body, read: its two rails as the body names them, and whether the second runs
/// through the plane against the first (`sense: opposite`).
#[derive(Clone, Debug)]
pub(super) struct SheetBody {
    pub a: Ref,
    pub b: Ref,
    pub opposite: bool,
}

/// A bare name, as a body names its own declarations: no path.
fn bare(r: &Ref) -> Option<&str> {
    r.path.is_empty().then_some(r.root.text.as_str())
}

/// **What a body says, if it says a sheet** (`std.Developable`'s body, or one like it): `None`
/// where it says anything else, or more.
pub(super) fn sheet_body(lit: &SetLit) -> Option<SheetBody> {
    use crate::model::EntKind;
    let p = lit.bound.text.as_str();
    // the body's own declarations, by name: their kinds and, of a line, its two ends
    let mut decls: BTreeMap<&str, (EntKind, Option<[&str; 2]>)> = BTreeMap::new();
    let mut relations: Vec<&crate::syntax::Written> = Vec::new();
    for st in &lit.body {
        match &st.kind {
            StmtKind::Decl(d) => {
                let ends = match d.children.as_slice() {
                    [a, b] if d.kind == EntKind::Line => match (a.as_slice(), b.as_slice()) {
                        ([Kid::Ref(a)], [Kid::Ref(b)]) => Some([bare(a)?, bare(b)?]),
                        _ => return None,
                    },
                    _ => None,
                };
                decls.insert(d.name.key().text.as_str(), (d.kind, ends));
            }
            StmtKind::Relation(r) if !r.claim => relations.push(r.form.written()?),
            _ => return None,
        }
    }
    let kind = |n: &str| decls.get(n).map(|d| d.0);
    // the point on the ruling: `p coincident(t in (0, 1)) line(m, n)`
    let (mut m, mut n) = (None, None);
    let mut rest = Vec::new();
    for w in &relations {
        let on_ruling = w.word.text == "coincident" && w.ops.len() == 2
            && bare(&w.ops[0]) == Some(p)
            && matches!(w.args.as_slice(), [OpArg::Within { key, lo, hi }]
                if key.text == "t" && lo.0.trim() == "0" && hi.0.trim() == "1");
        if on_ruling && m.is_none() {
            let [x, y] = decls.get(bare(&w.ops[1])?)?.1?;
            m = Some(x);
            n = Some(y);
        } else {
            rest.push(*w);
        }
    }
    let (m, n) = (m?, n?);
    if kind(m) != Some(EntKind::Point) || kind(n) != Some(EntKind::Point) || m == n {
        return None;
    }
    // each on a path the body does not declare, and one plane of its own touching both there
    let mut on: BTreeMap<&str, &Ref> = BTreeMap::new();
    let mut touches: Vec<(&str, &str, &Ref, Option<String>)> = Vec::new();
    for w in rest {
        match w.word.text.as_str() {
            "coincident" if w.args.is_empty() && w.ops.len() == 2 => {
                let x = bare(&w.ops[0])?;
                if ![m, n].contains(&x) || on.contains_key(x) || decls.contains_key(w.ops[1].root.text.as_str()) {
                    return None;
                }
                on.insert(x, &w.ops[1]);
            }
            "tangent" if w.ops.len() == 2 => {
                let plane = bare(&w.ops[0])?;
                let mut at = None;
                let mut sense = None;
                for a in &w.args {
                    match a {
                        OpArg::Named(k, crate::syntax::Arg::Ref(r)) if k.text == "at" => at = bare(r),
                        OpArg::Named(k, crate::syntax::Arg::Word(v)) if k.text == "sense" => {
                            sense = Some(v.clone());
                        }
                        _ => return None,
                    }
                }
                touches.push((plane, at?, &w.ops[1], sense));
            }
            _ => return None,
        }
    }
    let (a, b) = (*on.get(m)?, *on.get(n)?);
    let [(pa, ma, ra, sa), (pb, mb, rb, sb)] = touches.as_slice() else { return None };
    // the tangencies in either order: the one at `m` touches `a`, the one at `n` touches `b`
    let ((ta, at_a, rail_a, sense_a), (tb, at_b, rail_b, sense_b)) = if *ma == m {
        ((pa, ma, ra, sa), (pb, mb, rb, sb))
    } else {
        ((pb, mb, rb, sb), (pa, ma, ra, sa))
    };
    let same_ref = |x: &Ref, y: &Ref| crate::syntax::ref_text(x) == crate::syntax::ref_text(y);
    if ta != tb || *at_a != m || *at_b != n || kind(ta) != Some(EntKind::Frame)
        || !same_ref(rail_a, a) || !same_ref(rail_b, b) || sense_a.is_some()
    {
        return None;
    }
    // which root: `b` through the plane the way `a` runs, or against it
    let opposite = match sense_b.as_deref() {
        Some("same") => false,
        Some("opposite") => true,
        _ => return None,
    };
    // nothing else declared: the two points, the plane and the ruling
    if decls.len() != 4 {
        return None;
    }
    Some(SheetBody { a: a.clone(), b: b.clone(), opposite })
}

/// A development's component, read: which of its formals is the set, the home point and the
/// layer, and its points the ruling is laid between (`f`, `g`).
#[derive(Clone, Debug)]
pub struct DevelopmentBody {
    pub set: String,
    pub home: String,
    pub layer: String,
    pub f: String,
    pub g: String,
}

/// **What a component says, if it says a development** (`std.Development`'s body, or one like
/// it): a `set`, a `point` and a `Length` among its formals, and an isometry along rulings in its
/// body — two rate rows equal to two others' and an angle row reading the first's rate.
pub fn development_body(comp: &crate::syntax::Component) -> Option<DevelopmentBody> {
    if comp.set.is_some() {
        return None;
    }
    let formal = |ty: Ty| -> Option<String> {
        let mut it = comp.formals.iter().filter(|f| f.ty == ty);
        let one = it.next()?.name.text.clone();
        it.next().is_none().then_some(one)
    };
    let (set, home, layer) = (formal(Ty::Set)?, formal(Ty::Ent(crate::model::EntKind::Point))?,
                              formal(Ty::Length)?);
    let mut equal: Vec<String> = Vec::new();
    let mut angled = false;
    for w in comp.body.iter().filter_map(written_of) {
        match w.word.text.as_str() {
            "equal" if w.rates.len() == 2 && w.ops.len() == 2 => {
                equal.push(bare(&w.ops[0])?.to_string());
            }
            "angle" if w.rates.len() == 2 => angled = true,
            _ => {}
        }
    }
    let [f, g] = equal.as_slice() else { return None };
    angled.then(|| DevelopmentBody { set, home, layer, f: f.clone(), g: g.clone() })
}

/// A rail as the flattener resolved it: the path or the named chain's links by absolute name,
/// whether a chain, whether it comes round, and whether it is run the other way.
#[derive(Clone, Debug, PartialEq)]
pub struct RailInfo {
    pub links: Vec<String>,
    pub chain: bool,
    pub closed: bool,
    pub reversed: bool,
}

/// **A band of a recognised sheet at a region solid's probe** (#645): the probe by absolute key,
/// the rails, which side the band is on (1 the outside, −1 the inside), how thick, in the file's
/// lengths, and the set's name — what the solid's faces are named under.
#[derive(Clone, Debug)]
pub struct SheetInfo {
    pub probe: String,
    pub rails: [RailInfo; 2],
    pub side: f64,
    pub d: f64,
    pub name: String,
    pub span: Span,
}

/// **A recognised development** (#645): its instance's prefix, the panel's rails, the home point
/// on `a` by absolute name, the layer, and whether the panel is a band (both rails closed).
#[derive(Clone, Debug)]
pub struct DevelopedInfo {
    pub prefix: String,
    pub rails: [RailInfo; 2],
    pub home: String,
    pub d: f64,
    pub span: Span,
}

/// A development met in the walk, its set resolved once every set is known (`Walk::developments`).
#[derive(Clone)]
pub(super) struct Pending {
    prefix: String,
    set: Ref,
    home: Ref,
    d: f64,
    caller: Scope,
    inner: Scope,
    st: Stmt,
    path: Vec<PathStep>,
    depth: usize,
}

impl Walk<'_> {
    /// **A development's instance** (#645): not walked as its body, which states rates of a march
    /// the core lays exactly — instead `f` and `g`, the home ruling's ends, drawn where the
    /// instance is, the laid rails' ends held on them (`a` and `b`, made by the elaborator), and the
    /// end rulings; the set resolved once every set is known.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn walk_development(
        &mut self,
        dev: &DevelopmentBody,
        comp: &crate::syntax::Component,
        inst: &crate::syntax::Instance,
        st: &Stmt,
        caller: &Scope,
        sc: &Scope,
        vals: &mut Vals,
        path: &[PathStep],
        depth: usize,
    ) {
        let at = inst.name.span;
        let arg = |name: &str| -> Option<Ref> {
            let fo = comp.formals.iter().find(|f| f.name.text == name)?;
            let k = super::bindings::arg_index(comp, inst, fo)?;
            match &inst.args.get(k)?.value {
                crate::syntax::InstVal::Ref(r) => Some(r.clone()),
                _ => None,
            }
        };
        let (Some(set), Some(home)) = (arg(&dev.set), arg(&dev.home)) else {
            let m = format!("`{}` lays a set given its `{}` from the point given its `{}`",
                            comp.name.as_ref().map_or("", |n| n.text.as_str()), dev.set, dev.home);
            return self.once(Code::E103, at, m);
        };
        let d = match value_aff(&dev.layer, vals, self.units) {
            Ok(a) if a.dim.fits(crate::units::Dim::LENGTH) => {
                match a.as_dim(crate::units::Dim::LENGTH).number() {
                    Some(d) => d,
                    None => return self.once(Code::E040, at, format!("`{}`, the layer a \
                        development lays, is a number the drawing states", dev.layer)),
                }
            }
            _ => return self.once(Code::E103, at, format!("`{}` is a length", dev.layer)),
        };
        let prefix = sc.prefix().to_string();
        for curve in ["a", "b"] {
            self.names.insert(format!("{prefix}{curve}"));
        }
        let span = st.span;
        let name = |n: &str| crate::syntax::DeclName::Written(Name { text: n.to_string(), span: at });
        let point = |n: &str, seed: [f64; 2], hidden: bool, by: Option<&str>| {
            let mut d = crate::syntax::Decl::point(name(n), seed,
                by.map(|r| crate::syntax::AtRef::at(Ref::new(r))));
            d.annotations.private = hidden;
            StmtKind::Decl(d)
        };
        let pinned = |p: &str, t: f64, curve: &str| {
            let slot = OpArg::Slot { key: Name { text: "t".into(), span: at },
                                     arg: crate::syntax::Arg::Seed { value: t, pinned: true } };
            StmtKind::Relation(super::sets::relation(("coincident", at), crate::constraints::Fixity::Infix,
                vec![Ref::new(p), Ref::new(curve)], vec![slot], span))
        };
        let line = |n: &str, a: &str, b: &str| {
            StmtKind::Decl(crate::syntax::Decl::line(name(n), Ref::new(a), Ref::new(b),
                                                     Default::default()))
        };
        let kinds = vec![
            point(&dev.f, [0.0, 0.0], false, None),
            point(&dev.g, [100.0, 0.0], false, None),
            point("a0", [0.0; 2], true, Some(&dev.f)),
            point("b0", [0.0; 2], true, Some(&dev.g)),
            point("a1", [0.0; 2], true, Some(&dev.f)),
            point("b1", [0.0; 2], true, Some(&dev.g)),
            pinned("a0", 0.0, "a"),
            pinned("b0", 0.0, "b"),
            pinned("a1", 1.0, "a"),
            pinned("b1", 1.0, "b"),
            line("start", "a0", "b0"),
            line("end", "a1", "b1"),
        ];
        let body: Vec<Stmt> = kinds.into_iter()
            .map(|kind| Stmt { id: st.id, kind, span, chained: crate::syntax::Chained::No })
            .collect();
        self.body(&body, sc, vals, path, depth);
        self.developing.push(Pending {
            prefix, set, home, d, caller: caller.clone(), inner: sc.clone(), st: st.clone(),
            path: path.to_vec(), depth,
        });
    }

    /// Each development met, its set read once every set is known: its rails, and — a band — its
    /// seam and where it ends drawn too.
    pub(super) fn developments(&mut self) {
        let alias = self.alias_table();
        for p in std::mem::take(&mut self.developing) {
            let at = p.st.span;
            let found = self.look(&alias).raw(&p.set, &p.caller)
                .filter(|(_, rest)| rest.is_empty())
                .and_then(|(abs, _)| self.sets.made.get(&abs).cloned().map(|s| (abs, s)));
            let Some((abs, site)) = found else {
                let m = format!("`{}` is not a set: a development lays a panel a plane touches \
                    along each ruling (`std.Developable`)", crate::syntax::ref_text(&p.set));
                self.once(Code::E040, at, m);
                continue;
            };
            let Some(rails) = self.sheet_rails(&site, &alias) else {
                let m = format!("`{}` is no panel a plane touches along each ruling, so nothing \
                    lays it flat: a development lays `std.Developable`'s set, or a set whose body \
                    says what that one's does", crate::program::public_path(&abs));
                self.once(Code::E040, at, m);
                continue;
            };
            let Some((home, rest)) = self.look(&alias).find(&p.home, &p.caller) else { continue };
            let home = std::iter::once(home).chain(rest).collect::<Vec<_>>().join(".");
            if rails[0].closed && rails[1].closed {
                // a band cut at its seam: the home ruling, and where the march comes round to it
                let at_name = p.st.span;
                let name = |n: &str| crate::syntax::DeclName::Written(Name { text: n.to_string(), span: at_name });
                let kinds = [("seam", "a0", "b0"), ("seam_end", "a1", "b1")].map(|(n, a, b)| {
                    StmtKind::Decl(crate::syntax::Decl::line(name(n), Ref::new(a), Ref::new(b),
                                                             Default::default()))
                });
                let body: Vec<Stmt> = kinds.into_iter()
                    .map(|kind| Stmt { id: p.st.id, kind, span: at, chained: crate::syntax::Chained::No })
                    .collect();
                let mut vals = Vals::default();
                self.body(&body, &p.inner, &mut vals, &p.path, p.depth);
            }
            self.developed.push(DevelopedInfo { prefix: p.prefix, rails, home, d: p.d, span: at });
        }
    }

    /// A recognised sheet's rails, resolved from where its body is read, or `None` for any other
    /// set (or rails that name nothing a path is).
    pub(super) fn sheet_rails(&self, site: &super::sets::Site, alias: &BTreeMap<String, String>)
        -> Option<[RailInfo; 2]> {
        let body = sheet_body(site.lit())?;
        let rail = |r: &Ref, reversed: bool| -> Option<RailInfo> {
            let (abs, rest) = self.look(alias).find(r, site.scope())?;
            let abs = std::iter::once(abs).chain(rest).collect::<Vec<_>>().join(".");
            // a named chain: its links by absolute name, in order
            let chain = self.out.iter().find_map(|(st, _, sc)| match &st.kind {
                StmtKind::Chain(c) if c.name.key().text == abs => Some((c.clone(), sc.clone())),
                _ => None,
            });
            if let Some((c, sc)) = chain {
                let links = c.links.iter().map(|l| {
                    let (t, rest) = self.look(alias).find(l, &sc)?;
                    Some(std::iter::once(t).chain(rest).collect::<Vec<_>>().join("."))
                }).collect::<Option<Vec<_>>>()?;
                return Some(RailInfo { links, chain: true, closed: c.closed, reversed });
            }
            // one path: closed where it is a circle, or a spline written closed
            let closed = self.out.iter().any(|(st, _, _)| matches!(&st.kind,
                StmtKind::Decl(d) if d.name.key().text == abs
                    && (d.kind == crate::model::EntKind::Circle || d.close.is_some())));
            Some(RailInfo { links: vec![abs], chain: false, closed, reversed })
        };
        Some([rail(&body.a, false)?, rail(&body.b, body.opposite)?])
    }
}

impl Walk<'_> {
    /// **A band of a recognised sheet** (#645): `q distance(<= d, side: outside) S` at a region
    /// solid's probe, recorded as the band it is (`SheetInfo`) and not applied as a body.  A
    /// distance from a sheet reads a foot on it along its normal, which only the band built from
    /// its rulings has: from a drawn point, and as anything but a bound within `d`, it is refused.
    pub(super) fn reach_sheet(&mut self, su: &super::sets::SetUse, lowered: &crate::syntax::Written,
                              rails: [RailInfo; 2], abs: &str, alias: &BTreeMap<String, String>) {
        let (k, at) = (su.set().0, su.u.at);
        let set = crate::program::public_path(abs);
        let probe = self.look(alias).raw(&su.w.ops[1 - k], su.u.scope)
            .filter(|(p, rest)| rest.is_empty() && self.sets.probes.contains(p)).map(|(p, _)| p);
        let Some(probe) = probe else {
            let m = format!("`{set}` is a sheet, a panel a plane touches along each ruling: a \
                distance from it is measured along its normal, and is a band of a region solid, \
                `solid({{ q | q distance(<= d, side: outside) {set} }})`");
            return self.once(Code::E040, at, m);
        };
        let side = match lowered.sel("side").as_deref() {
            Some("outside") => 1.0,
            Some("inside") => -1.0,
            _ => {
                let m = format!("a distance from `{set}` says its side: `side: outside` or `side: \
                    inside`");
                return self.once(Code::E040, at, m);
            }
        };
        let (mut cmp, mut text) = (None, None);
        for a in &lowered.args {
            match a {
                OpArg::Bound { cmp: c, hi: None, grow: None, .. } => cmp = Some(*c),
                OpArg::Dim(t, _) => text = Some(t.clone()),
                _ => {}
            }
        }
        let (Some(crate::syntax::Cmp::Le), Some(text)) = (cmp, text) else {
            let m = format!("a band of `{set}` is the points within a distance of it on one side: \
                `distance(<= d, side: outside) {set}`");
            return self.once(Code::E040, at, m);
        };
        let d = match value_read(&text, &*su.u.scope.vals, &self.unknowns, self.units) {
            Ok(v) if v.dim.fits(crate::units::Dim::LENGTH) => {
                v.as_dim(crate::units::Dim::LENGTH).number()
            }
            _ => None,
        };
        let Some(d) = d.filter(|d| *d > 0.0) else {
            let m = format!("`{text}` is how thick a band of `{set}` is: a length more than \
                nothing the drawing states");
            return self.once(Code::E040, at, m);
        };
        self.sets.sheets.push(SheetInfo { probe, rails, side, d, name: abs.to_string(), span: at });
    }
}
