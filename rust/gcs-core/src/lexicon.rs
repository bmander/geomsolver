//! **The lexicon**: every word a document can be written with, and the slots each one takes.
//!
//! A front end offers the language as its words — `line`, `distance`, `std.place.right_of`,
//! `std.solids.Prism` — and asks for each word's arguments by what they are: a point, a line or
//! an axis, a `Length`, one of `left`/`right`.  What a word takes is the language's own
//! knowledge, so it is read here off the tables the elaborator settles statements by, never kept
//! as a second list beside them:
//!
//! * an **entity**'s slots are its kind's `fields()`;
//! * a **built-in relation word**'s operands are every pair of kinds `constraints::infix_op` (or
//!   `prefix_op`) settles the word over, and what goes in its parentheses is the spec of every
//!   constraint those pairs settle to, read the way `syntax::operator_text` prints it;
//! * a **component**'s slots are its formals, typed as written; and
//! * a **relation word** a module defines (`b right_of(d) a`) takes what its body hands its
//!   operands and parameters to: `a level(up) b` makes `a` and `b` points.
//!
//! `statement` is the inverse: a word and what fills its slots, written as the statement a person
//! would write.  `edit::write_word` appends it, imports what it needs and holds it to the
//! elaborator, so a front end composes no source.

use crate::constraints::{infix_op, prefix_op, CKind, Fixity, SpecKind};
use crate::json::{object, Json};
use crate::model::{EntKind, Field};
use crate::syntax::{self, Arg, OpArg, Program, StmtKind, Ty};
use std::collections::{BTreeMap, BTreeSet};

/// How a slot is written in its statement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// The name the statement gives what it makes: `NAME := …`.
    Name,
    /// An infix word's operands, before and after it.
    Left,
    Right,
    /// A prefix word's operand, after its parentheses.
    Operand,
    /// An entity in the parentheses, by position: `symmetry(l)`, a call's operands, `line(a, b)`.
    Arg,
    /// The word's number, bare and first in its parentheses: `distance(80)`.
    Value,
    /// Labelled in the parentheses: `side: left`, a component's number `w: 20`.
    Option,
    /// A slot pinned, `t == 0.4`.
    Pin,
    /// A seed after the statement, `hint(r: 25)`; with no key, a point's place, `hint((3, 4))`.
    Seed,
}

impl Role {
    pub fn as_str(self) -> &'static str {
        match self {
            Role::Name => "name",
            Role::Left => "left",
            Role::Right => "right",
            Role::Operand => "operand",
            Role::Arg => "arg",
            Role::Value => "value",
            Role::Option => "option",
            Role::Pin => "pin",
            Role::Seed => "seed",
        }
    }
}

/// One hole of a word.
#[derive(Clone, Debug)]
pub struct Slot {
    pub key: String,
    pub role: Role,
    /// The kinds a click may fill it with, as the language names them (`point`, `plane`,
    /// `solid`, `face`); `any` is anything drawn.  Empty for a slot only typed into.
    pub kinds: Vec<String>,
    /// What is typed into it: `Length`, `Angle`, `Scalar`, `Int`, `Bool`, `expr` (a number or
    /// a name), `vector`, `place`, `name`.  `None` for a slot only clicked.
    pub ty: Option<String>,
    /// The words it takes, where it takes words (`left`, `right`; `up`).
    pub words: Vec<String>,
    pub optional: bool,
    /// Several, in order: a spline's control points, a face's edges.
    pub many: bool,
    /// A point slot a click on nothing fills with a new point (`line`'s ends).
    pub mints: bool,
    /// What a formal is given when the call leaves it out.
    pub default: Option<String>,
}

impl Slot {
    fn new(key: &str, role: Role) -> Slot {
        Slot {
            key: key.into(), role, kinds: Vec::new(), ty: None, words: Vec::new(), optional: false,
            many: false, mints: false, default: None,
        }
    }
    fn kinds(mut self, ks: &[&str]) -> Slot {
        self.kinds = ks.iter().map(|k| k.to_string()).collect();
        self
    }
    fn ty(mut self, t: &str) -> Slot {
        self.ty = Some(t.into());
        self
    }
    fn words(mut self, ws: &[&str]) -> Slot {
        self.words = ws.iter().map(|w| w.to_string()).collect();
        self
    }
    fn optional(mut self) -> Slot {
        self.optional = true;
        self
    }
    fn many(mut self) -> Slot {
        self.many = true;
        self
    }
    fn mints(mut self) -> Slot {
        self.mints = true;
        self
    }
}

/// How the statement a word makes is laid out.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Form {
    /// `NAME := kind(children) hint(seed)`.
    Declare,
    /// `a word(…) b`.
    Infix,
    /// `word(…) a`.
    Prefix,
    /// `word(a, b, c)`.
    Call,
    /// `NAME := path(entities, label: number)`, or `NAME := solid(path(…))` for a solid's set.
    Instance { solid: bool },
    /// `param NAME: TYPE := VALUE`, or with no value an unknown.
    Param,
    /// `NAME := VALUE`.
    Value,
}

impl Form {
    pub fn as_str(&self) -> &'static str {
        match self {
            Form::Declare => "declare",
            Form::Infix => "infix",
            Form::Prefix => "prefix",
            Form::Call => "call",
            Form::Instance { .. } => "instance",
            Form::Param => "param",
            Form::Value => "value",
        }
    }
}

/// One word of the lexicon.
#[derive(Clone, Debug)]
pub struct Entry {
    /// Unique across the lexicon: `line`, `distance`, `std.place.right_of`, `std.solids.Prism`,
    /// a document's own `Lug`.
    pub id: String,
    /// What is written: `distance`, `right_of`, `std.solids.Prism`.
    pub word: String,
    /// The module it is read from: `""` for the language's own, `"."` for the document's.
    pub module: String,
    /// What the word is for, by what it makes or says: `Geometry`, `Relations`, `Dimensions`,
    /// `Placement`, `Sets`, `Faces & solids`, …
    pub group: String,
    pub form: Form,
    /// One sentence on what it means, the language's or its definition's comment.
    pub doc: String,
    /// The kind of thing it declares, if it declares one (`line`, `solid`, `set`, `instance`).
    pub makes: Option<String>,
    pub slots: Vec<Slot>,
    /// A built-in infix word's operand kinds, pair by pair, as the elaborator settles them — so
    /// a front end narrows the second operand once the first is known — each with the selectors
    /// the pair is settled by only when they are given (`tangent` between two lines is at a point
    /// named, `at:`; a point's distance from a plane is `along:` its normal).
    pub pairs: Vec<(String, String, Vec<String>)>,
    /// The `use` it needs: the module, and the word imported bare where it is a relation word.
    pub uses: Option<(String, Option<String>)>,
}

impl Entry {
    fn new(id: &str, word: &str, module: &str, group: &str, form: Form, doc: &str) -> Entry {
        Entry {
            id: id.into(), word: word.into(), module: module.into(), group: group.into(), form,
            doc: doc.into(), makes: None, slots: Vec::new(), pairs: Vec::new(), uses: None,
        }
    }
}

/* -- the language's own words ------------------------------------------------------- */

/// What each of the language's words means, in a sentence.  Read by a person choosing one.
const DOCS: &[(&str, &str)] = &[
    ("point", "A point. Where it is clicked is where its solve begins, never a constraint."),
    ("line", "A line between two points: click a point, or empty space to place one."),
    ("circle", "A circle about its centre; its radius is a seed the solve may move."),
    ("arc", "An arc about its centre, counter-clockwise from its start to its end."),
    ("spline", "A spline over named control points, in order."),
    ("curve", "A free curve between two points, its shape settled by an energy."),
    ("axis", "A directed line in space."),
    ("plane", "A plane over two axes or lines."),
    ("face", "A planar region bounded by the edges given, in order, with holes."),
    ("solid", "The points of a set, or another body, as a body."),
    ("coincident", "Two points meet, or a point lies on a line, circle, arc, curve or plane."),
    ("distance", "A length between two things: points, a point and a line, two planes. With \
                  along, a signed run; with side, which side of a line."),
    ("tangent", "Two things touch. Order carries meaning: an arc tangent to a line is at the arc's end."),
    ("equal", "Two lines the same length, or two circles or arcs the same radius."),
    ("curvature", "A circle osculates a curve where they touch."),
    ("level", "Two points share an ordinate along a direction."),
    ("angle", "Directed, counter-clockwise from the first line to the second; or one angle equal \
               to another."),
    ("midpoint", "A point halfway along a line."),
    ("project", "Two points drawn in two views are images of one point in space."),
    ("parallel", "Two lines, axes or planes run alike."),
    ("perpendicular", "Square to one another; sense says which quarter turn."),
    ("symmetry", "Two points mirrored in a line, or turned a half turn about an axis."),
    ("mapped", "A point where the map taking the sources to the targets carries another: two \
                pairs the similarity, three the affine map."),
    ("horizontal", "A line is level in its view."),
    ("vertical", "A line is plumb in its view."),
    ("radius", "The radius of a circle or an arc."),
    ("length", "How long one path is: a line, an arc, a spline or a free curve."),
    ("fix", "Hold an entity's own numbers where they are stated: (0, 0), or r == 25."),
    ("ccw", "The third point is left of the ray from the first to the second: a root chosen, \
             no equation."),
    ("cw", "The third point is right of the ray from the first to the second: a root chosen, \
            no equation."),
    ("param", "A number the document states: an input, or with no value an unknown the solve \
               finds."),
    ("value", "A named number, worked out while elaborating."),
];

fn doc_of(word: &str) -> &'static str {
    DOCS.iter().find(|(w, _)| *w == word).map_or("", |(_, d)| d)
}

/// The kinds a person clicks on, in the order a list of them is read.
const DRAWN: [EntKind; 10] = [
    EntKind::Point, EntKind::Line, EntKind::Circle, EntKind::Arc, EntKind::Spline,
    EntKind::Curve, EntKind::Frame, EntKind::Axis, EntKind::Face, EntKind::Solid,
];

/// A path: what a point runs along.
const PATH: [&str; 5] = ["line", "arc", "circle", "spline", "curve"];

/// The built-in words a statement relates two things with, and what each is for.
const INFIX: [(&str, &str); 13] = [
    ("coincident", "Relations"), ("tangent", "Relations"), ("parallel", "Relations"),
    ("perpendicular", "Relations"), ("equal", "Relations"), ("midpoint", "Relations"),
    ("symmetry", "Relations"), ("level", "Relations"), ("curvature", "Relations"),
    ("project", "Relations"), ("mapped", "Relations"), ("distance", "Dimensions"),
    ("angle", "Dimensions"),
];
const PREFIX: [(&str, &str); 4] = [
    ("horizontal", "Relations"), ("vertical", "Relations"), ("radius", "Dimensions"),
    ("length", "Dimensions"),
];

/// The selectors `infix_op` reads to tell one sense of a word from another, each as a statement
/// would give it: a word is every constraint it settles to under any of them.
const SELECTORS: [&[(&str, &str)]; 7] = [
    &[], &[("t", "0.5")], &[("along", "u")], &[("at", "")], &[("at", "start")],
    &[("about", "x")], &[("side", "left")],
];

/// A constraint the workspace has nothing to click for: what a motion, an envelope or a meshing
/// is held to, written in the program panel.
fn exotic(k: CKind) -> bool {
    k.spec().iter().any(|(_, s)| matches!(s, SpecKind::Envelope | SpecKind::Meshing
        | SpecKind::Motion | SpecKind::Traced | SpecKind::Carrier | SpecKind::Against))
        || matches!(k, CKind::CarriedPoint | CKind::CarriedPointIn | CKind::LineFraction)
}

/// What a constraint slot of this kind may be clicked on with.
fn spec_kinds(s: SpecKind) -> &'static [&'static str] {
    match s {
        SpecKind::Point => &["point"],
        SpecKind::Line => &["line"],
        SpecKind::Circle => &["circle"],
        SpecKind::Arc => &["arc"],
        SpecKind::CircleOrArc => &["circle", "arc"],
        SpecKind::Spline => &["spline"],
        SpecKind::Curve => &["curve"],
        SpecKind::Plane => &["plane"],
        SpecKind::Axis => &["axis"],
        SpecKind::Direction => &["line", "axis"],
        SpecKind::Along => &["line", "axis", "plane"],
        SpecKind::Scalar => &["point", "line", "circle", "arc", "axis", "plane"],
        SpecKind::Solid => &["solid"],
        _ => &[],
    }
}

fn kind_word(k: EntKind) -> &'static str {
    k.as_str()
}

/// How one constraint's spec is written around its word: `syntax::operator_text`'s reading.
struct Shape {
    /// The entities written by position, operands first.
    ents: Vec<(&'static str, SpecKind, usize)>,
    value: Option<SpecKind>,
    /// `(label, kinds, words, pin)` for a labelled slot in the parentheses.
    labelled: Vec<(&'static str, Vec<&'static str>, Vec<String>, Option<&'static str>, bool)>,
}

fn shape(k: CKind, word: &str) -> Shape {
    let mut s = Shape { ents: Vec::new(), value: None, labelled: Vec::new() };
    for (i, &(name, sk)) in k.spec().iter().enumerate() {
        // what the core reads off the geometry and nobody writes: a projection's planes, a skew
        // distance's sign, an ordinate's form
        if (sk.is_entity() || matches!(name, "sign" | "form")) && k.infers_arg(i) {
            continue;
        }
        let words: Vec<String> = k.words(i).map(|ws| ws.iter().map(|w| w.to_string()).collect())
            .unwrap_or_default();
        let labelled_ref = matches!(name, "at" | "by" | "about" | "under" | "seen_from")
            || (name == "along" && word == "distance");
        if sk.takes_ref() && labelled_ref {
            s.labelled.push((name, spec_kinds(sk).to_vec(), words, None, false));
        } else if sk.takes_ref() {
            s.ents.push((name, sk, i));
        } else if word == "level" && name == "along" {
            // `a level(up) b`: the direction stands bare, as a third entity would
            s.ents.push((name, sk, i));
        } else if sk.is_param() {
            s.labelled.push((name, Vec::new(), words, Some("Scalar"), true));
        } else if sk.is_dimension() {
            s.value = Some(sk);
        } else {
            let (ty, words) = match sk {
                SpecKind::Bool => ("Bool", vec!["true".to_string(), "false".to_string()]),
                SpecKind::Int => ("Int", words),
                SpecKind::Float => ("Scalar", words),
                _ => ("word", words),
            };
            s.labelled.push((name, Vec::new(), words, Some(ty), false));
        }
    }
    s
}

fn dim_ty(s: SpecKind) -> &'static str {
    match s {
        SpecKind::Angle => "Angle",
        SpecKind::Ratio => "Scalar",
        _ => "Length",
    }
}

fn push_unique(v: &mut Vec<String>, xs: impl IntoIterator<Item = impl ToString>) {
    for x in xs {
        let x = x.to_string();
        if !v.contains(&x) {
            v.push(x);
        }
    }
}

/// A built-in relation word, read off the tables that settle it.
fn builtin(word: &str, group: &str, fixity: Fixity) -> Option<Entry> {
    let form = match fixity {
        Fixity::Infix => Form::Infix,
        Fixity::Prefix => Form::Prefix,
        Fixity::Call => Form::Call,
    };
    let mut e = Entry::new(word, word, "", group, form, doc_of(word));
    let mut kinds: Vec<CKind> = Vec::new();
    let (mut left, mut right): (Vec<String>, Vec<String>) = (Vec::new(), Vec::new());
    for a in DRAWN {
        if fixity == Fixity::Prefix {
            if let Some(k) = prefix_op(word, a).filter(|&k| !exotic(k)) {
                push_unique(&mut left, [kind_word(a)]);
                if !kinds.contains(&k) {
                    kinds.push(k);
                }
            }
            continue;
        }
        for b in DRAWN {
            for sel in SELECTORS {
                let pick = |key: &str| sel.iter().find(|(k, _)| *k == key).map(|(_, v)| v.to_string());
                let Some(k) = infix_op(word, a, b, &pick).filter(|&k| !exotic(k)) else { continue };
                let (x, y) = (kind_word(a), kind_word(b));
                // the first selectors that settle it, none before any: what the pair needs said
                if !e.pairs.iter().any(|(p, q, _)| p == x && q == y) {
                    let needs = sel.iter().map(|(key, _)| key.to_string()).collect();
                    e.pairs.push((x.to_string(), y.to_string(), needs));
                }
                push_unique(&mut left, [kind_word(a)]);
                push_unique(&mut right, [kind_word(b)]);
                if !kinds.contains(&k) {
                    kinds.push(k);
                }
            }
        }
    }
    if kinds.is_empty() {
        return None;
    }
    // an angle made equal to another is chosen by the two lines in its parentheses
    // (`program::relations`), which no selector says
    if word == "angle" {
        kinds.push(CKind::EqualAngle);
    }
    let shapes: Vec<Shape> = kinds.iter().map(|&k| shape(k, word)).collect();
    let operands = if fixity == Fixity::Infix { 2 } else { 1 };
    // the operands named as the word's first sense names them: `p distance q`, `l1 angle l2`
    let named = |j: usize| shapes[0].ents.get(j).map_or(if j == 0 { "a" } else { "b" }, |x| x.0);
    match fixity {
        Fixity::Infix => e.slots.push(Slot { kinds: left, ..Slot::new(named(0), Role::Left) }),
        Fixity::Prefix => e.slots.push(Slot { kinds: left, ..Slot::new(named(0), Role::Operand) }),
        Fixity::Call => {}
    }
    // the number, where the word has one: optional where some sense of it has none
    let values: Vec<SpecKind> = shapes.iter().filter_map(|s| s.value).collect();
    if !values.is_empty() {
        let mut v = Slot::new("value", Role::Value);
        let tys: Vec<&str> = values.iter().map(|&s| dim_ty(s)).collect();
        push_unique(&mut v.kinds, std::iter::empty::<String>());
        v.ty = Some(if tys.iter().all(|t| *t == tys[0]) { tys[0] } else { "expr" }.to_string());
        v.optional = values.len() < shapes.len();
        e.slots.push(v);
    }
    // the entities beyond the operands, by position (a mirror's line, an angle's other pair)
    let skip = if fixity == Fixity::Call { 0 } else { operands };
    let extra = shapes.iter().map(|s| s.ents.len().saturating_sub(skip)).max().unwrap_or(0);
    for j in 0..extra {
        let mut slot = Slot::new("", Role::Arg);
        let mut present = 0;
        for (s, &k) in shapes.iter().zip(&kinds) {
            if let Some(&(name, sk, i)) = s.ents.get(skip + j) {
                present += 1;
                if slot.key.is_empty() {
                    slot.key = name.to_string();
                }
                push_unique(&mut slot.kinds, spec_kinds(sk).iter());
                if let Some(ws) = k.words(i) {
                    push_unique(&mut slot.words, ws.iter());
                }
            }
        }
        slot.optional = present < shapes.len();
        e.slots.push(slot);
    }
    // what goes labelled in the parentheses: a pin, a selector, an `at:` point
    let mut labelled: Vec<Slot> = Vec::new();
    for s in &shapes {
        for (name, ks, ws, ty, pin) in &s.labelled {
            if matches!(*name, "under" | "seen_from" | "mirror" | "sign") {
                continue;
            }
            let at = match labelled.iter().position(|l| l.key == *name) {
                Some(at) => at,
                None => {
                    let role = if *pin { Role::Pin } else { Role::Option };
                    labelled.push(Slot::new(name, role).optional());
                    labelled.len() - 1
                }
            };
            let l = &mut labelled[at];
            push_unique(&mut l.kinds, ks.iter());
            push_unique(&mut l.words, ws.iter());
            if l.ty.is_none() {
                l.ty = ty.map(|t| t.to_string());
            }
        }
    }
    e.slots.extend(labelled);
    if fixity == Fixity::Infix {
        e.slots.push(Slot { kinds: right, ..Slot::new(named(1), Role::Right) });
    }
    Some(e)
}

/// `ccw(a, b, c)` and `cw(a, b, c)`: three points, a root chosen.
fn orientation(word: &str) -> Entry {
    let mut e = Entry::new(word, word, "", "Relations", Form::Call, doc_of(word));
    for key in ["a", "b", "c"] {
        e.slots.push(Slot::new(key, Role::Arg).kinds(&["point"]));
    }
    e
}

/// `fix(…) e`: what is held is the entity's own numbers, said whole or by member.
fn fix() -> Entry {
    let mut e = Entry::new("fix", "fix", "", "Relations", Form::Prefix, doc_of("fix"));
    e.slots.push(Slot::new("of", Role::Operand).kinds(&["point", "circle", "arc", "axis", "plane"]));
    e.slots.push(Slot::new("value", Role::Value).ty("expr"));
    e
}

/// An entity declared: its kind's children and the numbers it is seeded with.
fn declaration(kind: EntKind) -> Entry {
    let word = kind_word(kind);
    let group = if matches!(kind, EntKind::Face | EntKind::Solid) { "Faces & solids" } else { "Geometry" };
    let mut e = Entry::new(word, word, "", group, Form::Declare, doc_of(word));
    e.makes = Some(word.into());
    e.slots.push(Slot::new("name", Role::Name).ty("name"));
    match kind {
        EntKind::Point => e.slots.push(Slot::new("", Role::Seed).ty("place").mints()),
        EntKind::Spline => e.slots.push(Slot::new("ctrl", Role::Arg).kinds(&["point"]).many().mints()),
        EntKind::Curve => {
            e.slots.push(Slot::new("a", Role::Arg).kinds(&["point"]).mints());
            e.slots.push(Slot::new("b", Role::Arg).kinds(&["point"]).mints());
        }
        EntKind::Face => {
            e.slots.push(Slot::new("edges", Role::Arg).kinds(&PATH).many());
            e.slots.push(Slot::new("holes", Role::Option).kinds(&["circle", "face"]).optional().many());
        }
        EntKind::Solid => e.slots.push(Slot::new("of", Role::Arg).kinds(&["solid", "face"]).ty("expr")),
        EntKind::Axis => e.slots.push(Slot::new("dir", Role::Seed).ty("vector").optional()),
        _ => {
            let line = kind == EntKind::Line;
            for &(name, f) in kind.fields() {
                match f {
                    Field::Child if kind == EntKind::Frame && name == "origin" => {}
                    Field::Child => {
                        let ks: &[&str] = match kind {
                            EntKind::Frame => &["axis", "line"],
                            _ => &["point"],
                        };
                        let role = if line { Role::Arg } else { Role::Option };
                        let mut s = Slot::new(name, role).kinds(ks);
                        if kind != EntKind::Frame {
                            s = s.mints();
                        }
                        e.slots.push(s);
                    }
                    Field::Scalar if name == "r" => {
                        e.slots.push(Slot::new("r", Role::Seed).ty("Length").optional());
                    }
                    _ => {}
                }
            }
            if kind == EntKind::Arc {
                e.slots.push(Slot::new("sense", Role::Option).ty("word").words(&["ccw", "cw"]).optional());
            }
            // what a circle about a point in space stands square to (#363 S6): an axis or a line,
            // or its own seeded along a direction, `hint(dir: (0, 0, 1))`
            if kind == EntKind::Circle {
                e.slots.push(Slot::new("normal", Role::Option).kinds(&["axis", "line"]).optional());
            }
        }
    }
    e
}

fn language() -> Vec<Entry> {
    let mut out: Vec<Entry> = [
        EntKind::Point, EntKind::Line, EntKind::Circle, EntKind::Arc, EntKind::Spline,
        EntKind::Curve, EntKind::Axis, EntKind::Frame, EntKind::Face, EntKind::Solid,
    ].into_iter().map(declaration).collect();
    for (w, g) in INFIX {
        out.extend(builtin(w, g, Fixity::Infix));
    }
    for (w, g) in PREFIX {
        out.extend(builtin(w, g, Fixity::Prefix));
    }
    out.push(fix());
    out.push(orientation("ccw"));
    out.push(orientation("cw"));
    let mut param = Entry::new("param", "param", "", "Values", Form::Param, doc_of("param"));
    param.slots.push(Slot::new("name", Role::Name).ty("name"));
    param.slots.push(Slot { default: Some("Length".into()), ..Slot::new("type", Role::Option)
        .ty("word").words(&["Length", "Angle", "Scalar", "Int"]) });
    param.slots.push(Slot::new("value", Role::Value).ty("expr").optional());
    out.push(param);
    let mut value = Entry::new("value", ":=", "", "Values", Form::Value, doc_of("value"));
    value.slots.push(Slot::new("name", Role::Name).ty("name"));
    value.slots.push(Slot::new("value", Role::Value).ty("expr"));
    out.push(value);
    out
}

/* -- the words a module or the document defines ------------------------------------- */

/// The modules a person draws with: the standard library and the hardware module.
const VOCABULARY: [(&str, &str); 14] = [
    ("std", "Geometry"), ("std.place", "Placement"), ("std.shapes", "Geometry"),
    ("std.sets", "Sets"), ("std.solids", "Faces & solids"), ("std.rounds", "Rounds"),
    ("std.panels", "Sets"), ("std.stations", "Stations"), ("std.motion", "Motion"),
    ("std.ports", "Parts"), ("std.bearings", "Parts"), ("std.retaining", "Parts"),
    ("std.measures", "Dimensions"), ("hardware", "Parts"),
];

/// The first sentence of the comment written above a definition at `lo` in `text`.
fn comment_above(text: &str, lo: usize) -> String {
    let lo = lo.min(text.len());
    let head = &text[..text[..lo].rfind('\n').map_or(0, |k| k + 1)];
    let mut lines: Vec<&str> = Vec::new();
    for line in head.lines().rev() {
        let t = line.trim();
        let Some(c) = t.strip_prefix("//") else { break };
        lines.push(c.trim());
    }
    lines.reverse();
    // a section's banner (`// -- a frame's joint -- …`) is not what the definition means
    let mut said = lines.iter().skip_while(|l| l.starts_with("--")).copied()
        .collect::<Vec<_>>().join(" ");
    said = said.trim_start_matches(':').trim().to_string();
    let cut = said.char_indices()
        .find(|&(i, c)| c == '.' && said[i + 1..].starts_with(' ')
            && said[i + 1..].trim_start().chars().next().is_some_and(char::is_uppercase))
        .map_or(said.len(), |(i, _)| i + 1);
    let mut s = said[..cut].trim().to_string();
    if s.chars().count() > 220 {
        s = s.chars().take(219).collect::<String>() + "…";
    }
    s
}

/// A formal's slot: its type as a click or a typed number fills it.
fn formal_slot(f: &syntax::Formal) -> Slot {
    let key = f.name.text.as_str();
    let mut s = match f.ty {
        Ty::Ent(k) if f.frame => Slot::new(key, Role::Arg).kinds(&[kind_word(k), "frame"]),
        Ty::Ent(k) => Slot::new(key, Role::Arg).kinds(&[kind_word(k)]),
        Ty::Path => Slot::new(key, Role::Arg).kinds(&PATH),
        Ty::Set => Slot::new(key, Role::Arg).kinds(&["solid", "face"]).ty("expr"),
        Ty::Group => Slot::new(key, Role::Arg).ty("expr"),
        Ty::Side => Slot::new(key, Role::Option).ty("word").words(&["left", "right"]),
        Ty::Int => Slot::new(key, Role::Option).ty("Int"),
        Ty::Scalar => Slot::new(key, Role::Option).ty("Scalar"),
        Ty::Length => Slot::new(key, Role::Option).ty("Length"),
        Ty::Angle => Slot::new(key, Role::Option).ty("Angle"),
    };
    if let Some(i) = &f.interface {
        s.ty = Some(i.text.clone());
    }
    if let Some((d, _)) = &f.default {
        s.default = Some(d.clone());
        s.optional = true;
    }
    if f.within.is_some() || f.frame {
        s.optional = true;
    }
    s
}

/// A component's entry: an instance of it, its formals its slots.
fn component(c: &syntax::Component, module: &str, group: &str, doc: String) -> Option<Entry> {
    let name = c.name.as_ref()?.text.clone();
    let path = if module == "." { name.clone() } else { format!("{module}.{name}") };
    let solid = module == "std.solids";
    let mut e = Entry::new(&path, &path, module, group, Form::Instance { solid }, &doc);
    e.makes = Some(if solid { "solid" } else if c.set.is_some() { "set" } else { "instance" }.into());
    e.slots.push(Slot::new("name", Role::Name).ty("name"));
    e.slots.extend(c.formals.iter().map(formal_slot));
    if module != "." {
        e.uses = Some((module.to_string(), None));
    }
    Some(e)
}

/// What the language's words at hand say of their operands and options, by word: the kinds a
/// relation word's body hands an operand or a parameter to.
#[derive(Clone, Default)]
struct Takes {
    left: Vec<String>,
    right: Vec<String>,
    value: Option<String>,
    args: Vec<Vec<String>>,
    labelled: BTreeMap<String, (Vec<String>, Vec<String>, Option<String>)>,
}

fn takes_of(e: &Entry) -> Takes {
    let mut t = Takes::default();
    for s in &e.slots {
        match s.role {
            Role::Left | Role::Operand => t.left = s.kinds.clone(),
            Role::Right => t.right = s.kinds.clone(),
            Role::Value => t.value = s.ty.clone(),
            Role::Arg => t.args.push(s.kinds.clone()),
            Role::Option | Role::Pin => {
                t.labelled.insert(s.key.clone(), (s.kinds.clone(), s.words.clone(), s.ty.clone()));
            }
            Role::Name | Role::Seed => {}
        }
    }
    t
}

/// Narrow `have` by what one use says; `None` is not yet narrowed.
fn narrow(have: &mut Option<Vec<String>>, by: &[String]) {
    if by.is_empty() || by.iter().any(|k| k == "any") {
        return;
    }
    *have = Some(match have.take() {
        None => by.to_vec(),
        Some(h) => h.into_iter().filter(|k| by.contains(k)).collect(),
    });
}

/// A relation word a module or the document defines: its operands and parameters typed by
/// what its body does with them.
fn relation_word(w: &syntax::WordDef, module: &str, group: &str, doc: String,
                 known: &BTreeMap<String, Takes>) -> Entry {
    let word = w.word.text.clone();
    let id = if module == "." { word.clone() } else { format!("{module}.{word}") };
    let form = match w.fixity {
        Fixity::Infix => Form::Infix,
        Fixity::Prefix => Form::Prefix,
        Fixity::Call => Form::Call,
    };
    let mut e = Entry::new(&id, &word, module, group, form, &doc);
    let mut ops: Vec<Option<Vec<String>>> = vec![None; w.operands.len()];
    let mut params: Vec<(Option<Vec<String>>, Vec<String>, Option<String>)> =
        vec![(None, Vec::new(), None); w.params.len()];
    let bare = |r: &syntax::Ref, n: &str| r.path.is_empty() && r.root.text == n;
    let param_at = |s: &str| w.params.iter().position(|p| p.text == s.trim());
    for st in &w.body {
        let StmtKind::Relation(r) = &st.kind else { continue };
        let Some(wr) = r.form.written() else { continue };
        let Some(t) = known.get(&wr.word.text) else { continue };
        for (j, op) in wr.ops.iter().enumerate() {
            let by = if wr.fixity == Fixity::Infix && j == 1 { &t.right } else { &t.left };
            for (i, name) in w.operands.iter().enumerate() {
                if bare(op, &name.text) {
                    narrow(&mut ops[i], by);
                }
            }
        }
        let mut positional = 0;
        for a in &wr.args {
            let (found, kinds, words, ty): (Option<usize>, Vec<String>, Vec<String>, Option<String>) =
                match a {
                    OpArg::Dim(text, _) => (param_at(text), Vec::new(), Vec::new(), t.value.clone()),
                    OpArg::Named(label, arg) => {
                        let at = match arg {
                            Arg::Ref(r) if r.path.is_empty() => param_at(&r.root.text),
                            Arg::Word(s) => param_at(s),
                            Arg::Dim { text, .. } => param_at(text),
                            _ => None,
                        };
                        let (k, ws, ty) = t.labelled.get(&label.text).cloned().unwrap_or_default();
                        (at, k, ws, ty)
                    }
                    OpArg::Ent(r) => {
                        positional += 1;
                        let k = t.args.get(positional - 1).cloned().unwrap_or_default();
                        (r.path.is_empty().then(|| param_at(&r.root.text)).flatten(), k, Vec::new(), None)
                    }
                    _ => (None, Vec::new(), Vec::new(), None),
                };
            if let Some(i) = found {
                narrow(&mut params[i].0, &kinds);
                push_unique(&mut params[i].1, words);
                if params[i].2.is_none() {
                    params[i].2 = ty;
                }
            }
        }
    }
    let operand = |i: usize, role: Role| {
        let mut s = Slot::new(&w.operands[i].text, role);
        s.kinds = match (&ops[i], w.types.get(i).and_then(|t| t.as_ref())) {
            (_, Some(interface)) => {
                s.ty = Some(interface.text.clone());
                vec!["any".into()]
            }
            (Some(k), _) if !k.is_empty() => k.clone(),
            _ => vec!["any".into()],
        };
        s
    };
    match w.fixity {
        Fixity::Infix if w.operands.len() == 2 => e.slots.push(operand(0, Role::Left)),
        Fixity::Prefix if !w.operands.is_empty() => {}
        _ => {}
    }
    for (p, (kinds, words, ty)) in w.params.iter().zip(params) {
        let mut s = Slot::new(&p.text, Role::Option);
        s.kinds = kinds.unwrap_or_default();
        s.words = words;
        s.ty = Some(match ty {
            Some(t) if t != "word" => t,
            _ if s.kinds.is_empty() && s.words.is_empty() => "expr".into(),
            _ => "expr".into(),
        });
        e.slots.push(s);
    }
    match w.fixity {
        Fixity::Infix if w.operands.len() == 2 => e.slots.push(operand(1, Role::Right)),
        Fixity::Prefix if !w.operands.is_empty() => e.slots.push(operand(0, Role::Operand)),
        _ => {}
    }
    if module != "." {
        e.uses = Some((module.to_string(), Some(word)));
    }
    e
}

/// The words of every module in `prog` that `keep` admits, then the document's own, in the order
/// each text writes them.
fn defined(prog: &Program, keep: &dyn Fn(&str) -> bool, own: bool,
           known: &mut BTreeMap<String, Takes>) -> Vec<Entry> {
    // (module index or none, where, component or word index)
    let mut at: Vec<(Option<usize>, usize, Result<usize, usize>)> = Vec::new();
    for (i, c) in prog.components.iter().enumerate() {
        if c.name.is_some() {
            at.push((c.module, c.span.lo as usize, Ok(i)));
        }
    }
    for (i, w) in prog.words.iter().enumerate() {
        at.push((w.module, w.span.lo as usize, Err(i)));
    }
    at.sort_by_key(|&(m, lo, _)| (m.map_or(usize::MAX, |m| m), lo));
    let mut out = Vec::new();
    for (m, lo, which) in at {
        let (module, group, text, local) = match m {
            Some(k) => {
                let md = &prog.modules[k];
                if !keep(&md.name) {
                    continue;
                }
                let group = VOCABULARY.iter().find(|(n, _)| *n == md.name).map_or("Parts", |(_, g)| g);
                (md.name.clone(), group, md.text.as_str(), lo.saturating_sub(md.base))
            }
            None if own => (".".to_string(), "This document", prog.text(), lo),
            None => continue,
        };
        let doc = comment_above(text, local);
        match which {
            Ok(i) => out.extend(component(&prog.components[i], &module, group, doc)),
            Err(i) => {
                let mut e = relation_word(&prog.words[i], &module, group, doc, known);
                known.insert(e.word.clone(), takes_of(&e));
                // a word defined both between two things and of one (`A round(r) B`, `round(r)
                // X`) is two entries, the second told by its form
                if out.iter().any(|x: &Entry| x.id == e.id) {
                    e.id = format!("{} ({})", e.id, e.form.as_str());
                }
                out.push(e);
            }
        }
    }
    out
}

/// The library's vocabulary, linked once: a program that `use`s every module of it.
fn library() -> Program {
    let src: String = VOCABULARY.iter().map(|(m, _)| format!("use {m}\n")).collect();
    crate::library::parse_linked(&src).0
}

/// **Every word**: the language's, the library's, any other module the document links, and the
/// document's own — `prog` being the document, or `None` for the words any document has.
pub fn lexicon(prog: Option<&Program>) -> Vec<Entry> {
    let mut out = language();
    let mut known: BTreeMap<String, Takes> =
        out.iter().filter(|e| matches!(e.form, Form::Infix | Form::Prefix | Form::Call))
            .map(|e| (e.word.clone(), takes_of(e))).collect();
    let vocabulary: BTreeSet<&str> = VOCABULARY.iter().map(|(m, _)| *m).collect();
    out.extend(defined(&library(), &|m| vocabulary.contains(m), false, &mut known));
    if let Some(p) = prog {
        out.extend(defined(p, &|m| !vocabulary.contains(m), true, &mut known));
    }
    out
}

/// The word `id` names, among those `prog` can be written with.
pub fn entry(prog: Option<&Program>, id: &str) -> Option<Entry> {
    lexicon(prog).into_iter().find(|e| e.id == id)
}

fn strs(v: &[String]) -> Json {
    Json::Arr(v.iter().map(|s| Json::Str(s.clone())).collect())
}

fn opt_str(s: &Option<String>) -> Json {
    s.as_ref().map_or(Json::Null, |s| Json::Str(s.clone()))
}

pub fn slot_json(s: &Slot) -> Json {
    object([
        ("key", s.key.as_str().into()),
        ("role", s.role.as_str().into()),
        ("kinds", strs(&s.kinds)),
        ("type", opt_str(&s.ty)),
        ("words", strs(&s.words)),
        ("optional", s.optional.into()),
        ("many", s.many.into()),
        ("mints", s.mints.into()),
        ("default", opt_str(&s.default)),
    ])
}

pub fn entry_json(e: &Entry) -> Json {
    object([
        ("id", e.id.as_str().into()),
        ("word", e.word.as_str().into()),
        ("module", e.module.as_str().into()),
        ("group", e.group.as_str().into()),
        ("form", e.form.as_str().into()),
        ("doc", e.doc.as_str().into()),
        ("makes", opt_str(&e.makes)),
        ("slots", Json::Arr(e.slots.iter().map(slot_json).collect())),
        ("pairs", Json::Arr(e.pairs.iter()
            .map(|(a, b, needs)| Json::Arr(vec![Json::Str(a.clone()), Json::Str(b.clone()), strs(needs)]))
            .collect())),
    ])
}

/// The lexicon as a front end reads it: `[{id, word, module, group, form, doc, makes, slots,
/// pairs}]`.
pub fn lexicon_json(prog: Option<&Program>) -> Json {
    Json::Arr(lexicon(prog).iter().map(entry_json).collect())
}

/* -- writing a word ------------------------------------------------------------------ */

/// **The statement a word makes**, its slots filled from `args` — each slot's text by its key, a
/// slot that takes several a comma-separated list — and the plane a drawn declaration is drawn
/// in, if any.  Refused, in a sentence, for a slot required and empty.
pub fn statement(e: &Entry, args: &BTreeMap<String, String>, plane: Option<&str>)
    -> Result<String, String> {
    let get = |s: &Slot| args.get(&s.key).map(|v| v.trim()).filter(|v| !v.is_empty());
    for s in &e.slots {
        if !s.optional && s.role != Role::Name && get(s).is_none() && s.default.is_none() {
            let what = if s.key.is_empty() { "a place".to_string() } else { format!("`{}`", s.key) };
            return Err(format!("{} still needs {what}", e.word));
        }
    }
    let name = e.slots.iter().find(|s| s.role == Role::Name).and_then(|s| get(s));
    let named = |body: String| match name {
        Some(n) => format!("{n} := {body}"),
        None => body,
    };
    let of = |role: Role| e.slots.iter().filter(move |s| s.role == role);
    let first = |role: Role| of(role).find_map(|s| get(s).map(str::to_string));
    // the parentheses: the number, the entities by position, the pins, the labelled
    let mut parens: Vec<String> = Vec::new();
    // a map's pairs, each a point and where it goes: `from -> a`
    let mut from: Option<&str> = None;
    for s in &e.slots {
        let Some(v) = get(s) else { continue };
        match s.role {
            Role::Value if e.form != Form::Param && e.form != Form::Value => parens.insert(0, v.into()),
            Role::Arg if e.word == "mapped" => match from.take() {
                Some(f) => parens.push(format!("{f} -> {v}")),
                None => from = Some(v),
            },
            Role::Arg => parens.push(v.into()),
            Role::Pin => parens.push(format!("{} == {v}", s.key)),
            Role::Option if e.form != Form::Param => parens.push(format!("{}: {v}", s.key)),
            _ => {}
        }
    }
    let wrapped = |w: &str| match parens.is_empty() {
        true => w.to_string(),
        false => format!("{w}({})", parens.join(", ")),
    };
    let seeds: Vec<String> = of(Role::Seed).filter_map(|s| get(s).map(|v| match s.key.as_str() {
        "" => v.to_string(),
        k => format!("{k}: {v}"),
    })).collect();
    let hint = match seeds.is_empty() {
        true => String::new(),
        false => format!(" hint({})", seeds.join(", ")),
    };
    Ok(match &e.form {
        Form::Declare => {
            let drawn = matches!(e.word.as_str(), "point" | "line" | "circle" | "arc" | "spline");
            let within = match plane.filter(|p| drawn && !p.is_empty()) {
                Some(p) => format!(" in {p}"),
                None => String::new(),
            };
            named(format!("{}{hint}{within}", wrapped(&e.word)))
        }
        Form::Infix => format!("{} {} {}", first(Role::Left).unwrap_or_default(), wrapped(&e.word),
                               first(Role::Right).unwrap_or_default()),
        Form::Prefix => format!("{} {}", wrapped(&e.word), first(Role::Operand).unwrap_or_default()),
        Form::Call => wrapped(&e.word),
        Form::Instance { solid } => {
            let call = format!("{}({})", e.word, parens.join(", "));
            named(if *solid { format!("solid({call})") } else { call })
        }
        Form::Param => {
            let ty = args.get("type").map(|t| t.trim()).filter(|t| !t.is_empty()).unwrap_or("Length");
            let n = name.unwrap_or_default();
            match first(Role::Value) {
                Some(v) => format!("param {n}: {ty} := {v}"),
                None => format!("param {n}: {ty}"),
            }
        }
        Form::Value => named(first(Role::Value).unwrap_or_default()),
    })
}

/// The kind initial a word's name is minted in: `l0` for a line, `body0` for a solid.
pub fn name_stem(e: &Entry) -> String {
    match e.makes.as_deref() {
        Some(k) => match EntKind::parse(k) {
            Some(kind) => syntax::kind_initial(kind).to_string(),
            None if k == "solid" => "body".into(),
            None if k == "set" => "region".into(),
            None => e.word.rsplit('.').next().unwrap_or("x").to_lowercase(),
        },
        None => match e.form {
            Form::Param | Form::Value => "w".into(),
            _ => "x".into(),
        },
    }
}
