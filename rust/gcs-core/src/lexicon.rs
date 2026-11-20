//! **The lexicon**: every word a document can be written with, and the slots each one takes.
//!
//! A front end offers the language as its words — `line`, `distance`, `std.place.right_of`,
//! `std.solids.Prism` — and asks for each word's arguments by what they are: a point, a line or
//! an axis, a `Length`, one of `left`/`right`.  What a word takes is the language's own
//! knowledge, so it is read here off the tables the elaborator settles statements by, never kept
//! as a second list beside them:
//!
//! * an **entity**'s slots are its kind's `fields()`;
//! * a **built-in relation word** is every constraint kind `CKind::operator` spells with it, its
//!   operands every pair of kinds `constraints::infix_op` (or `prefix_op`) settles it over, and
//!   what goes in its parentheses each kind's spec as `syntax::place_of` places it — the reading
//!   `operator_text` prints by;
//! * a **component**'s slots are its formals, typed as written; and
//! * a **relation word** a module defines (`b right_of(d) a`) takes what its body hands its
//!   operands and parameters to: `a level(up) b` makes `a` and `b` points.
//!
//! `statement` is the inverse: a word and what fills its slots, written as the statement a person
//! would write — a relation by the relation printer itself.  `edit::write_word` appends it,
//! imports what it needs and holds it to the elaborator, so a front end composes no source.
//!
//! The language's and the library's words are the same for every document, and are read once
//! (`library_words`); a document adds its own and those of the other modules it links
//! (`own_words`).

use crate::constraints::{builtin_word, infix_op, kind_matches, prefix_op, CKind, Fixity, SpecKind,
                         ALL_KINDS, OPERATORS};
use crate::json::{object, Json};
use crate::model::{EntKind, Field};
use crate::syntax::{self, Arg, OpArg, Place, Program, StmtKind, Ty};
use std::collections::BTreeMap;

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
            Form::Infix | Form::Prefix | Form::Call => self.fixity().map_or("", Fixity::as_str),
            Form::Instance { .. } => "instance",
            Form::Param => "param",
            Form::Value => "value",
        }
    }

    /// Where a relation's word stands to its operands; `None` for any other statement.
    fn fixity(&self) -> Option<Fixity> {
        match self {
            Form::Infix => Some(Fixity::Infix),
            Form::Prefix => Some(Fixity::Prefix),
            Form::Call => Some(Fixity::Call),
            _ => None,
        }
    }
}

impl From<Fixity> for Form {
    fn from(f: Fixity) -> Form {
        match f {
            Fixity::Infix => Form::Infix,
            Fixity::Prefix => Form::Prefix,
            Fixity::Call => Form::Call,
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
    /// A declaration drawn on a plane — a point, a line, a circle, an arc, a spline — written
    /// `in` the plane being drawn on.
    pub drawn: bool,
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
            doc: doc.into(), makes: None, drawn: false, slots: Vec::new(), pairs: Vec::new(),
            uses: None,
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
    ("tangent", "Two things touch. Order carries meaning: an arc tangent to a line is at the arc's \
                 end."),
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
    ("touches", "A free curve pressed onto a line: a corner held somewhere on it, sliding to where \
                 the pull along the line balances."),
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

/// The kinds a person declares and clicks on, in the order a list of them is read.
const DRAWN: [EntKind; 10] = [
    EntKind::Point, EntKind::Line, EntKind::Circle, EntKind::Arc, EntKind::Spline,
    EntKind::Curve, EntKind::Axis, EntKind::Frame, EntKind::Face, EntKind::Solid,
];

/// A path: what a point runs along.
const PATH: [&str; 5] = ["line", "arc", "circle", "spline", "curve"];

/// The selectors `infix_op` reads to tell one sense of a word from another, each as a statement
/// would give it: a pair of kinds a word settles over only when one is given needs it said.
const SELECTORS: [&[(&str, &str)]; 8] = [
    &[], &[("t", "0.5")], &[("along", "u")], &[("at", "")], &[("at", "start")],
    &[("about", "x")], &[("pairs", "2")], &[("fraction", "0.5")],
];

/// A constraint the workspace has nothing to click for: what a motion, an envelope, a meshing or
/// a traced body is held to, and where on a line a contact over an interval is (the flattener's
/// own), written in the program panel.
fn exotic(k: CKind) -> bool {
    k.spec().iter().any(|(_, s)| matches!(s, SpecKind::Envelope | SpecKind::Meshing
        | SpecKind::Motion | SpecKind::Traced | SpecKind::Carrier | SpecKind::Against))
        || k == CKind::LineFraction
}

/// What a constraint slot of this kind may be clicked on with, by `constraints::kind_matches` —
/// and for the operand of `fix`, any entity whose own numbers it holds.
fn spec_kinds(s: SpecKind) -> Vec<String> {
    let all = |k: &EntKind| s == SpecKind::Scalar || kind_matches(s, *k);
    let drawable =
        |k: &EntKind| !matches!(k, EntKind::Face | EntKind::Solid) || s == SpecKind::Solid;
    DRAWN.iter().filter(|k| all(k) && drawable(k)).map(|k| k.as_str().to_string()).collect()
}

fn dim_ty(s: SpecKind) -> &'static str {
    match s {
        SpecKind::Angle => "Angle",
        SpecKind::Ratio => "Scalar",
        _ => "Length",
    }
}

fn push_unique<T: PartialEq>(v: &mut Vec<T>, xs: impl IntoIterator<Item = T>) {
    for x in xs {
        if !v.contains(&x) {
            v.push(x);
        }
    }
}

/// The slot `key` in `slots`, made with `role` where there is none yet.
fn slot_named<'a>(slots: &'a mut Vec<Slot>, key: &str, role: Role) -> &'a mut Slot {
    match slots.iter().position(|s| s.key == key) {
        Some(at) => &mut slots[at],
        None => {
            slots.push(Slot::new(key, role).optional());
            slots.last_mut().unwrap()
        }
    }
}

/// A built-in relation word, read off the tables that settle it: the kinds `CKind::operator`
/// spells with it, the pairs of drawn kinds it settles over, and each kind's spec as `place_of`
/// places it.  `None` for a word the workspace has nothing to click for.
fn builtin(word: &str, fixity: Fixity) -> Option<Entry> {
    let kinds: Vec<CKind> = ALL_KINDS.iter().copied()
        .filter(|k| k.operator() == Some((word, fixity)) && !exotic(*k)).collect();
    let mut pairs: Vec<(String, String, Vec<String>)> = Vec::new();
    let (mut left, mut right): (Vec<String>, Vec<String>) = (Vec::new(), Vec::new());
    for a in DRAWN {
        if fixity == Fixity::Prefix {
            if prefix_op(word, a).is_some_and(|k| !exotic(k)) {
                push_unique(&mut left, [a.as_str().to_string()]);
            }
            continue;
        }
        for b in DRAWN {
            for sel in SELECTORS {
                let pick =
                    |key: &str| sel.iter().find(|(k, _)| *k == key).map(|(_, v)| v.to_string());
                if !infix_op(word, a, b, &pick).is_some_and(|k| !exotic(k)) {
                    continue;
                }
                let (x, y) = (a.as_str().to_string(), b.as_str().to_string());
                // the first selectors that settle it, none before any: what the pair needs said
                if !pairs.iter().any(|(p, q, _)| *p == x && *q == y) {
                    let needs = sel.iter().map(|(k, _)| k.to_string()).collect();
                    pairs.push((x.clone(), y.clone(), needs));
                }
                push_unique(&mut left, [x]);
                push_unique(&mut right, [y]);
            }
        }
    }
    if kinds.is_empty() || left.is_empty() {
        return None;
    }
    let dimensioned = kinds.iter().any(|k| k.spec().iter().any(|(_, s)| s.is_dimension()));
    let group = if dimensioned { "Dimensions" } else { "Relations" };
    let mut e = Entry::new(word, word, "", group, fixity.into(), doc_of(word));
    e.pairs = pairs;
    // each kind's slots by place: its positional entities, its number, its labelled and pinned
    let operands = if fixity == Fixity::Infix { 2 } else { 1 };
    let mut positional: Vec<Slot> = Vec::new();
    let mut labelled: Vec<Slot> = Vec::new();
    let mut value: Option<Slot> = None;
    let mut numbered = 0;
    // how many of the word's senses write each positional entity: one some sense lacks is optional
    let mut present: Vec<usize> = Vec::new();
    for &k in &kinds {
        let mut j = 0;
        let mut has_number = false;
        for (i, &(name, sk)) in k.spec().iter().enumerate() {
            let words = k.words(i).unwrap_or(&[]).iter().map(|w| w.to_string());
            match syntax::place_of(k, i) {
                Place::Unwritten => {}
                Place::Positional => {
                    if positional.len() == j {
                        positional.push(Slot::new(name, Role::Arg));
                        present.push(0);
                    }
                    present[j] += 1;
                    push_unique(&mut positional[j].kinds, spec_kinds(sk));
                    push_unique(&mut positional[j].words, words);
                    j += 1;
                }
                Place::Number => {
                    has_number = true;
                    let v = value
                        .get_or_insert_with(|| Slot::new("value", Role::Value).ty(dim_ty(sk)));
                    if v.ty.as_deref() != Some(dim_ty(sk)) {
                        v.ty = Some("expr".into());
                    }
                }
                place => {
                    let role = if place == Place::Pin { Role::Pin } else { Role::Option };
                    let l = slot_named(&mut labelled, name, role);
                    if sk.takes_ref() {
                        push_unique(&mut l.kinds, spec_kinds(sk));
                    }
                    push_unique(&mut l.words, words);
                    if l.ty.is_none() && !sk.takes_ref() {
                        l.ty = Some(match sk {
                            SpecKind::Bool => {
                                push_unique(&mut l.words, ["true", "false"].map(String::from));
                                "Bool"
                            }
                            SpecKind::Int => "Int",
                            SpecKind::Float | SpecKind::Param => "Scalar",
                            _ => "word",
                        }.into());
                    }
                }
            }
        }
        numbered += has_number as usize;
    }
    for (p, n) in positional.iter_mut().zip(present) {
        p.optional = n < kinds.len();
    }
    // the operands named as the word's first sense names them: `p distance q`, `l1 angle l2`;
    // what the pairs say they take, the kinds
    let rest = positional.split_off(operands.min(positional.len()));
    let mut ops = positional.into_iter();
    let first = ops.next().map_or("a".to_string(), |s| s.key);
    match fixity {
        Fixity::Infix => e.slots.push(Slot { kinds: left, ..Slot::new(&first, Role::Left) }),
        _ => e.slots.push(Slot { kinds: left, ..Slot::new(&first, Role::Operand) }),
    }
    if let Some(mut v) = value {
        // optional where some sense of the word has no number (an angle made equal to another)
        v.optional = numbered < kinds.len();
        e.slots.push(v);
    }
    e.slots.extend(rest);
    e.slots.extend(labelled);
    if fixity == Fixity::Infix {
        let second = ops.next().map_or("b".to_string(), |s| s.key);
        e.slots.push(Slot { kinds: right, ..Slot::new(&second, Role::Right) });
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
    let held = ["point", "circle", "arc", "axis", "plane"];
    e.slots.push(Slot::new("of", Role::Operand).kinds(&held));
    e.slots.push(Slot::new("value", Role::Value).ty("expr"));
    e
}

/// An entity declared: its kind's children and the numbers it is seeded with.
fn declaration(kind: EntKind) -> Entry {
    let word = kind.as_str();
    let group = match kind {
        EntKind::Face | EntKind::Solid => "Faces & solids",
        _ => "Geometry",
    };
    let mut e = Entry::new(word, word, "", group, Form::Declare, doc_of(word));
    e.makes = Some(word.into());
    e.drawn = matches!(kind, EntKind::Point | EntKind::Line | EntKind::Circle | EntKind::Arc
        | EntKind::Spline);
    e.slots.push(Slot::new("name", Role::Name).ty("name"));
    match kind {
        EntKind::Point => e.slots.push(Slot::new("", Role::Seed).ty("place").mints()),
        EntKind::Spline => {
            e.slots.push(Slot::new("ctrl", Role::Arg).kinds(&["point"]).many().mints());
        }
        EntKind::Curve => {
            e.slots.push(Slot::new("a", Role::Arg).kinds(&["point"]).mints());
            e.slots.push(Slot::new("b", Role::Arg).kinds(&["point"]).mints());
        }
        EntKind::Face => {
            e.slots.push(Slot::new("edges", Role::Arg).kinds(&PATH).many());
            let holes = Slot::new("holes", Role::Option).kinds(&["circle", "face"]);
            e.slots.push(holes.optional().many());
        }
        EntKind::Solid => {
            e.slots.push(Slot::new("of", Role::Arg).kinds(&["solid", "face"]).ty("expr"));
        }
        EntKind::Axis => e.slots.push(Slot::new("dir", Role::Seed).ty("vector").optional()),
        _ => {
            for &(name, f) in kind.fields() {
                match f {
                    Field::Child if kind == EntKind::Frame && name == "origin" => {}
                    Field::Child if kind == EntKind::Frame => {
                        e.slots.push(Slot::new(name, Role::Option).kinds(&["axis", "line"]));
                    }
                    // a line's two ends by position, every other kind's children by name
                    Field::Child => {
                        let role = if kind == EntKind::Line { Role::Arg } else { Role::Option };
                        e.slots.push(Slot::new(name, role).kinds(&["point"]).mints());
                    }
                    Field::Scalar if name == "r" => {
                        e.slots.push(Slot::new("r", Role::Seed).ty("Length").optional());
                    }
                    _ => {}
                }
            }
            if kind == EntKind::Arc {
                let sense = Slot::new("sense", Role::Option).ty("word").words(&["ccw", "cw"]);
                e.slots.push(sense.optional());
            }
        }
    }
    e
}

fn language() -> Vec<Entry> {
    let mut out: Vec<Entry> = DRAWN.into_iter().map(declaration).collect();
    for word in OPERATORS {
        for fixity in [Fixity::Infix, Fixity::Prefix] {
            if builtin_word(word, fixity) {
                out.extend(builtin(word, fixity));
            }
        }
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

/// A word of `module` by its path, `module.name`, or bare where it is the document's own.
fn qualified(module: &str, name: &str) -> String {
    if module == "." { name.to_string() } else { format!("{module}.{name}") }
}

/// The first sentence of the comment written above a definition at `lo` in `text`.
fn comment_above(text: &str, lo: usize) -> String {
    let lo = lo.min(text.len());
    let head = &text[..text[..lo].rfind('\n').map_or(0, |k| k + 1)];
    let mut lines: Vec<&str> = Vec::new();
    for line in head.lines().rev() {
        let Some(c) = line.trim().strip_prefix("//") else { break };
        lines.push(c.trim());
    }
    lines.reverse();
    // a section's banner (`// -- a frame's joint -- …`) is not what the definition means
    let said = lines.iter().skip_while(|l| l.starts_with("--")).copied()
        .collect::<Vec<_>>().join(" ");
    let said = said.trim_start_matches(':').trim();
    let cut = said.char_indices()
        .find(|&(i, c)| c == '.' && said[i + 1..].starts_with(' ')
            && said[i + 1..].trim_start().chars().next().is_some_and(char::is_uppercase))
        .map_or(said.len(), |(i, _)| i + 1);
    let s = said[..cut].trim();
    match s.chars().count() > 220 {
        true => s.chars().take(219).collect::<String>() + "…",
        false => s.to_string(),
    }
}

/// A formal's slot: its type as a click or a typed number fills it.
fn formal_slot(f: &syntax::Formal) -> Slot {
    let key = f.name.text.as_str();
    let mut s = match f.ty {
        Ty::Ent(k) => Slot::new(key, Role::Arg).kinds(&[k.as_str()]),
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
    s.optional |= f.within.is_some() || f.frame;
    s
}

/// A component's entry: an instance of it, its formals its slots.
fn component(c: &syntax::Component, module: &str, group: &str, doc: String) -> Option<Entry> {
    let path = qualified(module, &c.name.as_ref()?.text);
    let solid = module == "std.solids";
    let mut e = Entry::new(&path, &path, module, group, Form::Instance { solid }, &doc);
    let makes = if solid { "solid" } else if c.set.is_some() { "set" } else { "instance" };
    e.makes = Some(makes.into());
    e.slots.push(Slot::new("name", Role::Name).ty("name"));
    e.slots.extend(c.formals.iter().map(formal_slot));
    if module != "." {
        e.uses = Some((module.to_string(), None));
    }
    Some(e)
}

/// What the words at hand say of their operands and options, by word: what a relation word's
/// body hands an operand or a parameter to.
#[derive(Clone, Default)]
struct Takes {
    left: Vec<String>,
    right: Vec<String>,
    value: Option<String>,
    args: Vec<Vec<String>>,
    labelled: BTreeMap<String, Slot>,
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
                t.labelled.insert(s.key.clone(), s.clone());
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
    let mut e = Entry::new(&qualified(module, &word), &word, module, group, w.fixity.into(), &doc);
    let mut ops: Vec<Option<Vec<String>>> = vec![None; w.operands.len()];
    // each parameter as a slot labelled by its name, its kinds narrowed by every use
    let mut params: Vec<(Option<Vec<String>>, Slot)> = w.params.iter()
        .map(|p| (None, Slot::new(&p.text, Role::Option))).collect();
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
            let (found, like): (Option<usize>, Slot) = match a {
                OpArg::Dim(text, _) => {
                    (param_at(text), Slot { ty: t.value.clone(), ..Slot::new("", Role::Value) })
                }
                OpArg::Named(label, arg) => {
                    let at = match arg {
                        Arg::Ref(r) if r.path.is_empty() => param_at(&r.root.text),
                        Arg::Word(s) | Arg::Dim { text: s, .. } => param_at(s),
                        _ => None,
                    };
                    let like = t.labelled.get(&label.text).cloned();
                    (at, like.unwrap_or(Slot::new("", Role::Option)))
                }
                OpArg::Ent(r) => {
                    positional += 1;
                    let kinds = t.args.get(positional - 1).cloned().unwrap_or_default();
                    let at = if r.path.is_empty() { param_at(&r.root.text) } else { None };
                    (at, Slot { kinds, ..Slot::new("", Role::Arg) })
                }
                _ => continue,
            };
            if let Some(i) = found {
                let (kinds, slot) = &mut params[i];
                narrow(kinds, &like.kinds);
                push_unique(&mut slot.words, like.words);
                if slot.ty.is_none() {
                    slot.ty = like.ty.filter(|t| t != "word");
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
    let infix = w.fixity == Fixity::Infix && w.operands.len() == 2;
    if infix {
        e.slots.push(operand(0, Role::Left));
    }
    for (kinds, mut s) in params {
        s.kinds = kinds.unwrap_or_default();
        s.ty.get_or_insert_with(|| "expr".into());
        e.slots.push(s);
    }
    if infix {
        e.slots.push(operand(1, Role::Right));
    } else if w.fixity == Fixity::Prefix && !w.operands.is_empty() {
        e.slots.push(operand(0, Role::Operand));
    }
    if module != "." {
        e.uses = Some((module.to_string(), Some(word)));
    }
    e
}

/// The words of every module in `prog` that `keep` admits, then the document's own where `own`,
/// in the order each text writes them; each relation word read joins `known`, so a later one
/// written over it is typed by it.
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
    at.sort_by_key(|&(m, lo, _)| (m.unwrap_or(usize::MAX), lo));
    let mut out = Vec::new();
    for (m, lo, which) in at {
        let (module, group, text, local) = match m {
            Some(k) => {
                let md = &prog.modules[k];
                if !keep(&md.name) {
                    continue;
                }
                let group =
                    VOCABULARY.iter().find(|(n, _)| *n == md.name).map_or("Parts", |(_, g)| g);
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

fn vocabulary(module: &str) -> bool {
    VOCABULARY.iter().any(|(m, _)| *m == module)
}

/// The words every document has, read once: the language's, and the library's vocabulary — with
/// what each relation word says of its operands, for a document's own words to be typed by.
fn every_document() -> &'static (Vec<Entry>, BTreeMap<String, Takes>) {
    static WORDS: std::sync::OnceLock<(Vec<Entry>, BTreeMap<String, Takes>)> =
        std::sync::OnceLock::new();
    WORDS.get_or_init(|| {
        let mut out = language();
        let mut known: BTreeMap<String, Takes> = out.iter().filter(|e| e.form.fixity().is_some())
            .map(|e| (e.word.clone(), takes_of(e))).collect();
        out.extend(defined(crate::library::linked(), &vocabulary, false, &mut known));
        (out, known)
    })
}

/// **The words every document can be written with**: the language's own and the standard
/// library's, the same for each.
pub fn library_words() -> &'static [Entry] {
    &every_document().0
}

/// **The words `prog` adds**: its own components and relation words, and those of any module it
/// links beyond the library's vocabulary.
pub fn own_words(prog: &Program) -> Vec<Entry> {
    let mut known = every_document().1.clone();
    defined(prog, &|m| !vocabulary(m), true, &mut known)
}

/// **Every word** `prog` can be written with — `None` for the words any document has.
pub fn lexicon(prog: Option<&Program>) -> Vec<Entry> {
    let mut out = library_words().to_vec();
    out.extend(prog.map(own_words).unwrap_or_default());
    out
}

/// The word `id` names, among those `prog` can be written with.
pub fn entry(prog: Option<&Program>, id: &str) -> Option<Entry> {
    library_words().iter().find(|e| e.id == id).cloned()
        .or_else(|| own_words(prog?).into_iter().find(|e| e.id == id))
}

pub fn slot_json(s: &Slot) -> Json {
    object([
        ("key", s.key.as_str().into()),
        ("role", s.role.as_str().into()),
        ("kinds", s.kinds.clone().into()),
        ("type", s.ty.clone().into()),
        ("words", s.words.clone().into()),
        ("optional", s.optional.into()),
        ("many", s.many.into()),
        ("mints", s.mints.into()),
        ("default", s.default.clone().into()),
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
        ("makes", e.makes.clone().into()),
        ("drawn", e.drawn.into()),
        ("slots", Json::Arr(e.slots.iter().map(slot_json).collect())),
        ("pairs", Json::Arr(e.pairs.iter()
            .map(|(a, b, needs)| {
                Json::Arr(vec![a.as_str().into(), b.as_str().into(), needs.clone().into()])
            })
            .collect())),
    ])
}

/// Words as a front end reads them: `[{id, word, module, group, form, doc, makes, drawn, slots,
/// pairs}]`.
pub fn words_json(words: &[Entry]) -> Json {
    Json::Arr(words.iter().map(entry_json).collect())
}

/* -- writing a word ------------------------------------------------------------------ */

/// **The statement a word makes**, its slots filled from `args` — each slot's text by its key, a
/// slot that takes several a comma-separated list — and the plane a drawn declaration is drawn
/// in, if any.  A relation is written by the relation printer (`syntax::write_stmt_to`), so it is
/// spelled as every relation is.  Refused, in a sentence, for a slot required and empty.
pub fn statement(e: &Entry, args: &BTreeMap<String, String>, plane: Option<&str>)
    -> Result<String, String> {
    let get = |s: &Slot| args.get(&s.key).map(|v| v.trim()).filter(|v| !v.is_empty());
    for s in &e.slots {
        if !s.optional && s.role != Role::Name && get(s).is_none() && s.default.is_none() {
            let what = match s.key.is_empty() {
                true => "a place".to_string(),
                false => format!("`{}`", s.key),
            };
            return Err(format!("{} still needs {what}", e.word));
        }
    }
    let first = |role: Role| e.slots.iter().filter(|s| s.role == role).find_map(get);
    if let Some(fixity) = e.form.fixity() {
        let mut ops = Vec::new();
        let mut written = Vec::new();
        for s in &e.slots {
            let Some(v) = get(s) else { continue };
            match s.role {
                Role::Left | Role::Right | Role::Operand => ops.push(syntax::Ref::new(v)),
                Role::Value => written.push(OpArg::Dim(v.into(), Default::default())),
                Role::Arg => written.push(OpArg::Ent(syntax::Ref::new(v))),
                Role::Pin => written.push(OpArg::Slot {
                    key: syntax::Name::new(s.key.as_str()),
                    arg: Arg::Dim { text: v.into(), span: Default::default() },
                }),
                _ => written.push(OpArg::Named(syntax::Name::new(s.key.as_str()),
                                               Arg::Word(v.into()))),
            }
        }
        let rel = syntax::Relation::of(syntax::RelationForm::Written(syntax::Written {
            word: syntax::Name::new(e.word.as_str()),
            fixity,
            ops,
            args: written,
            span: Default::default(),
            rates: Vec::new(),
            lowered: syntax::Lowered::No,
        }));
        let mut out = String::new();
        syntax::write_stmt_to(&mut out, &StmtKind::Relation(rel)).map_err(|e| e.to_string())?;
        return Ok(out);
    }
    let name = e.slots.iter().find(|s| s.role == Role::Name).and_then(get);
    let named = |body: String| match name {
        Some(n) => format!("{n} := {body}"),
        None => body,
    };
    // a declaration's or a call's parentheses: its entities by position, the rest by label
    let parens: Vec<String> = e.slots.iter().filter_map(|s| {
        let v = get(s)?;
        match s.role {
            Role::Arg => Some(v.to_string()),
            Role::Option if e.form != Form::Param => Some(format!("{}: {v}", s.key)),
            _ => None,
        }
    }).collect();
    let call = |w: &str| match parens.is_empty() {
        true => w.to_string(),
        false => format!("{w}({})", parens.join(", ")),
    };
    let seed = |s: &Slot, v: &str| match s.key.is_empty() {
        true => v.to_string(),
        false => format!("{}: {v}", s.key),
    };
    let seeds: Vec<String> = e.slots.iter().filter(|s| s.role == Role::Seed)
        .filter_map(|s| get(s).map(|v| seed(s, v))).collect();
    Ok(match &e.form {
        Form::Declare => {
            let hint = match seeds.is_empty() {
                true => String::new(),
                false => format!(" hint({})", seeds.join(", ")),
            };
            let within = plane.filter(|p| e.drawn && !p.is_empty()).map(|p| format!(" in {p}"));
            named(format!("{}{hint}{}", call(&e.word), within.unwrap_or_default()))
        }
        Form::Instance { solid } => {
            let c = format!("{}({})", e.word, parens.join(", "));
            named(if *solid { format!("solid({c})") } else { c })
        }
        Form::Param => {
            let ty = args.get("type").map(|t| t.trim()).filter(|t| !t.is_empty());
            let ty = ty.unwrap_or("Length");
            let n = name.unwrap_or_default();
            match first(Role::Value) {
                Some(v) => format!("param {n}: {ty} := {v}"),
                None => format!("param {n}: {ty}"),
            }
        }
        Form::Value => named(first(Role::Value).unwrap_or_default().to_string()),
        Form::Infix | Form::Prefix | Form::Call => unreachable!("a relation is printed above"),
    })
}

/// The stem a name for what a word makes is minted in, where it makes no entity (an entity's is
/// its kind's initial, `edit::mint`): `region0` for a set, `hex0` for an instance of `Hex`.
pub fn name_stem(e: &Entry) -> String {
    match e.makes.as_deref() {
        Some("set") => "region".into(),
        Some(_) => e.word.rsplit('.').next().unwrap_or_default().to_lowercase(),
        None => "w".into(),
    }
}
