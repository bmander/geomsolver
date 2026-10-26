//! **A relation as written** (§9.2): the word, its operands in written order and what its
//! parentheses hold — a selector, an entity, a pin or a seed, a vector, the number and the bound
//! it may be (§9.6) — kept through resolution for printing and source edits, and assembled into a
//! kind's arguments in spec order (`Written::assemble`).

use super::{num, Arg, Cmp, Name, Ref, Seg, Span};
use crate::constraints::{CKind, Fixity};

/// One written operator argument: a selector, entity, owned slot, or dimension.
#[derive(Clone, Debug)]
pub enum OpArg {
    /// `side: -1`, `at: start`, `along: u`, `external: true`
    Named(Name, Arg),
    /// the third entity, unlabelled: `a symmetry(l) b` — or every operand of a call, `ccw(a, b, c)`
    Ent(Ref),
    /// A named constraint slot. Pins (`t == 0.4`) constrain the solution; values
    /// inside `hint(…)` only seed it. Selectors such as `end: start` use `Named`.
    Slot { key: Name, arg: Arg },
    /// A whole vector pinned at once: `fix((0, 0)) p`, the point itself, or `fix(dir == (1, 0,
    /// 0)) t`, a vector the entity has.  Its components fill the slots of the members they are
    /// (`Written::assemble`), and it is kept whole here so that what a vector was written as —
    /// how many components — is still known where the entity is (`apply_gauge`).
    Vector { key: Option<Name>, parts: Vec<Arg>, span: Span },
    /// the number, as written — `80`, `x = 7`, `h = w / 2`, `1' 3"`
    Dim(String, Span),
    /// The number beside it is a **bound** (§9.6): `distance(>= 5)`, `distance(<= d)`, or the low
    /// end of `distance(in: (a, b))`, whose high end is `hi`.  `span` is the `>=` or `in:` written.
    /// Outside a set body a bound is a branch choice: no row, checked on the solution.  `grow` is
    /// an offset no source writes: a set's use (`p distance(<= d, side: outside) S`, §6.21) puts it
    /// on a cone's number, so the bound is on the distance from its surface (`model::RegionTerm`).
    Bound { cmp: Cmp, span: Span, hi: Option<(String, Span)>, grow: Option<(String, Span)> },
    /// **A contact over an interval** (#645): `p coincident(t in (0, 1)) line(m, n)`, the slot
    /// `key` held within `(lo, hi)` — on a line the segment between its ends.  Where on the curve,
    /// never a distance: no set it stands in is made a region by it.
    Within { key: Name, lo: (String, Span), hi: (String, Span) },
}

/// An operator spelling retained through resolution, with operands and arguments
/// in written order for printing and source edits.
#[derive(Clone, Debug)]
pub struct Written {
    pub word: Name,
    pub fixity: Fixity,
    /// One (prefix) or two (infix), in written order.  **Order carries meaning**: `arc tangent
    /// line` is `TangentArcLine` and `line tangent circle` is `TangentLineCircle`.
    pub ops: Vec<Ref>,
    pub args: Vec<OpArg>,
    pub span: Span,
    /// The operands written as a **rate** (#339 D2): `rate(p over s)` is operand `op` read as
    /// its rate along the formal `over`, never as itself.  Empty for every other relation.
    pub rates: Vec<RateOf>,
}

/// Where a rate in a relation's parentheses (`angle(rate(m over u), l)`, #645) is numbered from:
/// past the two operands, the `k`th entity there `RateOf::op` `ARG_RATES + k`.
pub const ARG_RATES: usize = 2;

/// The selector the flattener marks a contact's interval with (#645), beside its low end and its
/// `in:` bound: `p coincident(t in (a, b)) l`'s place bounded, which no source can spell (`#`), so
/// a bound written `coincident(in: (a, b))` is never read as one.
pub const WITHIN: &str = "#within";

impl Written {
    /// Where the operand `RateOf::op` names is: `Ok` an operand's index, `Err` the index in
    /// `args` of an entity in the parentheses.
    pub fn rated_at(&self, op: usize) -> Option<Result<usize, usize>> {
        if op < ARG_RATES {
            return (op < self.ops.len()).then_some(Ok(op));
        }
        self.args.iter().enumerate().filter(|(_, a)| matches!(a, OpArg::Ent(_)))
            .nth(op - ARG_RATES).map(|(i, _)| Err(i))
    }
}

/// One operand of a relation written as its rate along a formal, `rate(x over s)` (#339 D2):
/// the operand's index and the formal.  Of a point, its velocity; of a line, its direction's.
#[derive(Clone, Debug, PartialEq)]
pub struct RateOf {
    pub op: usize,
    pub over: Name,
    /// Read along **the slot of the contact holding the point** rather than a formal (#365):
    /// `rate(Q over t)` of `Q coincident(t == 0deg) contact`.  Set where the flattener finds the
    /// name no formal in scope (`Walk::rate_over`); written as the parser wrote it either way.
    pub slot: bool,
}

impl Written {
    /// The formal operand `k` is a rate along, where it is written as one.
    pub fn rate_of(&self, k: usize) -> Option<&Name> {
        self.rates.iter().find(|r| r.op == k).map(|r| &r.over)
    }

    /// Whether any operand is written as a rate.
    pub fn reads_rate(&self) -> bool {
        !self.rates.is_empty()
    }

    /// Whether it is a step, `step(u: s) l` (#330): the core's prefix word the flattener expands
    /// where it stands (`flatten::steps`).
    pub fn is_step(&self) -> bool {
        self.fixity == Fixity::Prefix && crate::constraints::step_word(&self.word.text)
    }

    /// The bound the number is, where it is one (§9.6): its direction and an interval's high
    /// end.
    pub fn bound(&self) -> Option<(Cmp, Option<&(String, Span)>)> {
        self.args.iter().find_map(|a| match a {
            OpArg::Bound { cmp, hi, .. } => Some((*cmp, hi.as_ref())),
            _ => None,
        })
    }

    /// Its number, as written, and where: the one `OpArg::Dim` — a kind has at most one, so it
    /// is *the* number in the parentheses.
    pub fn dim(&self) -> Option<(&str, Span)> {
        self.dim_index().map(|k| match &self.args[k] {
            OpArg::Dim(t, s) => (t.as_str(), *s),
            _ => unreachable!("a number"),
        })
    }

    /// Where its number stands among its arguments.
    pub fn dim_index(&self) -> Option<usize> {
        self.args.iter().position(|a| matches!(a, OpArg::Dim(..)))
    }

    /// The offset its bound carries, where a set's use put one (§6.21).
    pub fn grow(&self) -> Option<&(String, Span)> {
        self.args.iter().find_map(|a| match a {
            OpArg::Bound { grow, .. } => grow.as_ref(),
            _ => None,
        })
    }

    /// **A side, said as a bound** (§9.6, §6.21; #192), in place: `p inside X` is `p
    /// coincident(<=) X` and `p outside X` `p coincident(>=) X` — beside `p distance(<= d, side:
    /// outside) X`, a side grown, the one form a set's body carries its sides in (`inside(d)` is
    /// retired, #266), so a side turned round is its bound's `Cmp::turned` and its surface the
    /// bound taken off, as every other bound's.  Any other relation is left as it stands.
    pub fn lower_side(&mut self) {
        let Some(cmp) = crate::constraints::plane_side_word(&self.word.text) else { return };
        if self.ops.len() != 2 || !self.args.is_empty() {
            return;
        }
        self.word.text = "coincident".to_string();
        self.args = vec![OpArg::Bound { cmp, span: self.word.span, hi: None, grow: None }];
    }

    /// Which side of its second operand's surface it puts its point on (§9.6, §6.21), read in
    /// `lower_side`'s one form: the bound on the signed distance from the surface toward its
    /// outside, and that distance where one is written (`-(d)` for `side: inside`).  `None` for a
    /// relation stating no side, an equality (`distance(d, side: outside)`) or an interval.
    pub fn side(&self) -> Option<(Cmp, Option<(String, Span)>)> {
        let mut w = self.clone();
        w.lower_side();
        let (cmp, None) = w.bound()? else { return None };
        if w.ops.len() != 2 {
            return None;
        }
        let dim = w.dim().map(|(t, s)| (t.to_string(), s));
        match (w.word.text.as_str(), dim) {
            ("coincident", None) => Some((cmp, None)),
            ("distance", Some((t, s))) => match w.sel("side").as_deref() {
                Some("outside") => Some((cmp, Some((t, s)))),
                Some("inside") => Some((cmp.turned()?, Some((format!("-({t})"), s)))),
                _ => None,
            },
            _ => None,
        }
    }

    /// Its bound turned round (§9.6): the other side of the same number — `None` for an
    /// interval, either side of which is no one bound, or a relation stating none.
    pub fn turn_bound(&mut self) -> Option<()> {
        self.args.iter_mut().find_map(|a| match a {
            OpArg::Bound { cmp, .. } => Some(cmp),
            _ => None,
        }).and_then(|cmp| {
            *cmp = cmp.turned()?;
            Some(())
        })
    }

    /// Its one-sided bound made the equality at its number — a ball's sphere, a side's surface —
    /// returning which way the bound ran; `None` where it states none, or an interval.
    pub fn equality(&mut self) -> Option<Cmp> {
        let k = self.args.iter().position(|a| matches!(a, OpArg::Bound { hi: None, .. }))?;
        let OpArg::Bound { cmp, .. } = self.args.remove(k) else { unreachable!("a bound") };
        Some(cmp)
    }

    /// Its equality held as a bound at its number, `in: (d, d)` (§9.6) — what a set measured
    /// from states of its point, a term and never a row (#192).  `None` where it states a
    /// bound already, or no number.
    pub fn held(&mut self) -> Option<()> {
        if self.bound().is_some() {
            return None;
        }
        let k = self.dim_index()?;
        let (text, span) = self.dim().map(|(t, s)| (t.to_string(), s))?;
        self.args.insert(k, OpArg::Bound { cmp: Cmp::In, span, hi: Some((text, span)), grow: None });
        Some(())
    }

    /// Its `side:` turned round, `inside` for `outside` and `outside` for any other word: a
    /// distance from a set's outside read from the set it is the outside of (#209).
    pub fn turn_side(&mut self) {
        for a in self.args.iter_mut() {
            if let OpArg::Named(n, Arg::Word(x)) = a {
                if n.text == "side" {
                    *x = if x == "outside" { "inside".into() } else { "outside".into() };
                }
            }
        }
    }

    /// Whether it states a bound (§9.6): its number is one, or it is `inside` / `outside` a
    /// plane, lowered to one later — either way no row, and no derivative.
    pub fn states_bound(&self) -> bool {
        self.bound().is_some() || crate::constraints::plane_side_word(&self.word.text).is_some()
    }

    /// One selector by name — what `constraints::infix_op` reads to tell `distance … along: u`
    /// from a plain one, and a tangency at a named end from the bare pair.
    pub fn sel(&self, name: &str) -> Option<String> {
        self.args.iter().find_map(|a| match a {
            OpArg::Named(n, v) if n.text == name => Some(match v {
                Arg::Word(w) => w.clone(),
                Arg::Int(i) => i.to_string(),
                Arg::Bool(b) => b.to_string(),
                Arg::Num(x) => num(*x),
                _ => String::new(),
            }),
            _ => None,
        })
    }

    /// Where a selector's key was written — what a diagnostic about its *value* points at, since
    /// a value carries no span of its own (`Arg::Word` is a bare `String`).
    pub fn key_span(&self, name: &str) -> Option<Span> {
        self.args.iter().find_map(|a| match a {
            OpArg::Named(n, _) if n.text == name => Some(n.span),
            _ => None,
        })
    }

    /// Every pinned or seeded slot, a vector's components each under the member it is — `x`,
    /// `dir.y` — and where its key was written: what `assemble` fills and a gauge's key reads.
    pub fn slots(&self) -> impl Iterator<Item = (Name, &Arg)> + '_ {
        self.args.iter().flat_map(Self::fills)
    }

    /// The slots one argument fills: a pin or a seed its own, a vector one per member.
    fn fills(a: &OpArg) -> Vec<(Name, &Arg)> {
        match a {
            OpArg::Slot { key, arg } => vec![(key.clone(), arg)],
            OpArg::Vector { key, parts, span } => {
                let span = key.as_ref().map_or(*span, |k| k.span);
                let member = |c: &str| match key {
                    Some(k) => format!("{}.{c}", k.text),
                    None => c.to_string(),
                };
                let names = ["x", "y", "z"].map(member);
                names.into_iter().zip(parts).map(|(text, arg)| (Name { text, span }, arg)).collect()
            }
            _ => Vec::new(),
        }
    }

    /// Assemble arguments in registry order, rejecting unknown slot and selector names.
    /// Missing arguments remain `None` for elaboration to validate.
    pub fn assemble(&self, kind: CKind) -> Result<Vec<Option<Arg>>, (Span, String)> {
        let spec = kind.spec();
        // a seed or a pin names the slot it fills, and a name the kind does not have is a typo
        // rather than something to fill the first slot with: filled by position, the wrong
        // word here would silently pin the right slot at the wrong
        // number.  Checked before anything is assembled, so the message is about what was
        // written and not about what it came to.
        for (key, _) in self.slots() {
            if !spec.iter().any(|(n, k)| k.is_param() && *n == Self::slot_of(kind, &key.text)) {
                let word = &self.word.text;
                let m = format!("`{word}` has no slot `{}` to seed", key.text);
                return Err((key.span, m));
            }
        }
        // and a *selector* naming nothing was dropped in silence, which is the same mistake one
        // layer down (issue #48, item 4): `a distance(80, sied: x) b` settled as a plain distance
        // and the argument went nowhere.  A level's direction stands in its parentheses
        // unlabelled, so `along:` there is a second spelling and refused as one.
        for a in &self.args {
            let OpArg::Named(key, _) = a else { continue };
            if kind == CKind::Level && key.text == "along" {
                let m = "`level` takes its direction in its parentheses: `a level(up) b`, \
                         `a level(std.z) b`".to_string();
                return Err((key.span, m));
            }
            if !spec.iter().any(|(n, _)| *n == key.text) {
                let word = &self.word.text;
                // two directions in space turn neither way, but about an axis they do (#558)
                let m = match (kind, key.text.as_str()) {
                    (CKind::Angle3 | CKind::Perpendicular3, "sense") => format!(
                        "two directions in space turn neither way, so `{word}` takes a `sense` \
                         only with an axis to read the turn about: `{word}(…, sense: ccw, about: \
                         a)`"
                    ),
                    // how a path runs through a plane another touches (#645)
                    (CKind::PlaneTangentLine | CKind::PlaneTangentRound | CKind::Rate, "sense") => {
                        "`sense:` is how a second path runs through a plane the first touches, read \
                         between the two where a panel's body states both \
                         (`std.panels.Developable`): of one plane and one path it says nothing"
                            .to_string()
                    }
                    _ => format!("`{word}` takes no `{}`", key.text),
                };
                return Err((key.span, m));
            }
        }
        self.given_once(kind)?;
        let mut out: Vec<Option<Arg>> = vec![None; spec.len()];
        let mut ents: Vec<Ref> = self.ops.clone();
        // `length line1` is the distance between the line's own ends — the one prefix word
        // that is sugar for a statement about something else's parts (#269)
        if kind == CKind::Distance && self.ops.len() == 1 {
            let r = &self.ops[0];
            ents = ["p1", "p2"]
                .iter()
                .map(|f| Ref {
                    root: r.root.clone(),
                    path: vec![Seg::Field(Name::new(*f))],
                    span: r.span,
                })
                .collect();
        }
        ents.extend(self.args.iter().filter_map(|a| match a {
            OpArg::Ent(r) => Some(r.clone()),
            _ => None,
        }));
        let mut next = ents.into_iter();
        for (i, (name, sk)) in spec.iter().enumerate() {
            out[i] = if sk.takes_ref() {
                // a reference given by its slot's name (`at: m`) fills that slot; the operands
                // and the unlabelled references fill the rest in order
                let named = self.args.iter().find_map(|a| match a {
                    OpArg::Named(n, Arg::Ref(r)) if n.text == *name => Some(Arg::Ref(r.clone())),
                    _ => None,
                });
                named.or_else(|| next.next().map(Arg::Ref))
            } else if sk.is_param() {
                let slot = || {
                    let named = |key: &Name| Self::slot_of(kind, &key.text) == *name;
                    self.slots().filter(move |(key, _)| named(key)).map(|(_, arg)| arg)
                };
                // a shared parameter is pinned to an unknown and seeded where the unknown is
                // declared (`param s: Angle hint(330)`): a `hint(t: …)` beside the pin would be a
                // second seed for one unknown, and the contacts sharing it would disagree
                let tie = slot().find(|a| matches!(a, Arg::Tie { .. })).cloned();
                if tie.is_some() {
                    if let Some(key) = self.args.iter().find_map(|a| match a {
                        OpArg::Slot { key, arg: Arg::Seed { pinned: false, .. } }
                            if Self::slot_of(kind, &key.text) == *name => Some(key),
                        _ => None,
                    }) {
                        let m = format!(
                            "`{}` is pinned to an unknown, which is seeded where it is declared",
                            key.text
                        );
                        return Err((key.span, m));
                    }
                }
                // a carried point's roll is its own unknown or the angle stated, `at: τ` (#366):
                // one slot, said once
                let stated = if kind.roll_slot() == Some(i) { self.dim() } else { None };
                if stated.is_some() {
                    if let Some((key, _)) =
                        self.slots().find(|(k, _)| Self::slot_of(kind, &k.text) == *name)
                    {
                        let m = format!("`{}` is given twice: the roll is the relation's own `t` \
                                         or the angle stated, `at:`", key.text);
                        return Err((key.span, m));
                    }
                }
                tie.or_else(|| slot().next().cloned()).or_else(|| {
                    stated.map(|(t, span)| Arg::Dim { text: t.to_string(), span })
                })
            } else if sk.is_dimension() {
                // the number, wherever the dimension slot stands: a kind has at most one, so it
                // is *the* number in the parentheses, and a selector may follow it in spec order
                // (`distance(12, side: left)` — issue #48, item 4)
                self.dim().map(|(t, span)| Arg::Dim { text: t.to_string(), span })
            } else {
                self.args.iter().find_map(|a| match a {
                    OpArg::Named(n, v) if n.text == *name => Some(v.clone()),
                    _ => None,
                })
            };
        }
        if let Some(w) = kind.word_slot() {
            Self::direction(w, &mut out);
        }
        Ok(out)
    }

    /// The slot a key fills: its own name, but for a point on a curve traced in space, whose
    /// place `t` is the trace's own formal, `u`, as a spline's and a line's is their `t` (#363 S1).
    fn slot_of(kind: CKind, key: &str) -> &str {
        match (kind, key) {
            (CKind::PointOnTraced, "t") => "u",
            _ => key,
        }
    }

    /// **A slot is given once** (issue #112): assembled, a slot takes the first argument naming
    /// it, so a second — `fix((3, 4), (1, 1)) a`, `fix((3, 4), x == 1) a`, two pins of one
    /// contact — went nowhere and said nothing.  Refused at the second, saying which.  A pin to
    /// an unknown beside a seed for it has a message of its own (`assemble`'s tie).
    fn given_once(&self, kind: CKind) -> Result<(), (Span, String)> {
        let seed = |x: &Arg| {
            matches!(x, Arg::Seed { pinned: false, .. } | Arg::SeedExpr { pinned: false, .. })
        };
        let tie = |x: &Arg| matches!(x, Arg::Tie { .. });
        // each slot given so far, and whether by a place written whole (`(3, 4)`)
        let mut said: Vec<(String, &Arg, bool)> = Vec::new();
        for a in &self.args {
            let whole = matches!(a, OpArg::Vector { key: None, .. });
            for (key, arg) in Self::fills(a) {
                let slot = Self::slot_of(kind, &key.text).to_string();
                let before = said.iter().find(|(n, b, _)| {
                    *n == slot && !(tie(arg) && seed(b) || seed(arg) && tie(b))
                });
                if let Some(&(_, _, was_whole)) = before {
                    let word = &self.word.text;
                    let m = if whole && was_whole {
                        format!("`{word}` is given its place twice: a point is one vector")
                    } else {
                        format!("`{}` is given twice: a number is said once", key.text)
                    };
                    return Err((key.span, m));
                }
                said.push((slot, arg, whole));
            }
        }
        Ok(())
    }

    /// **An ordinate's direction, where it was written** (`docs/ordinate-plan.md`): `along:` an
    /// ordinate is measured on and the parentheses of `level` hold a word of
    /// `constraints::ALONG` or a reference.  The word goes to the kind's word slot — the core
    /// infers the axis it names — and a reference to its direction slot.
    fn direction(w: usize, out: &mut [Option<Arg>]) {
        let t = 2;
        match out[w].take() {
            Some(Arg::Ref(r)) => out[t] = Some(Arg::Ref(r)),
            other => out[w] = other,
        }
        if let Some(Arg::Ref(r)) = &out[t] {
            if let Some(v) = r.direction_word().map(str::to_string) {
                out[t] = None;
                out[w] = Some(Arg::Word(v));
            }
        }
    }
}
