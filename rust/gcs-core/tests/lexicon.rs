//! **The lexicon** (`lexicon.rs`): every word a document can be written with and the slots each
//! takes, read off the tables the elaborator settles statements by — and `edit::write_word`, the
//! statement a word and its filled slots make, appended with the `use` it needs and held to the
//! elaborator.  The app's Words menu and its argument box are built on both.
use gcs_core::edit::{self, WordRequest};
use gcs_core::lexicon::{self, Entry, Form, Role, Slot};
use gcs_core::library::{parse_linked, MODULES};
use gcs_core::syntax::parse;
use std::collections::BTreeMap;

/// A drawing with one of each kind a word is written over: points, lines, a circle, an arc, a
/// spline and a free curve drawn on the front plane, a face and a solid of it, and in space an
/// axis and the standard planes.
const DOC: &str = "unit mm\nuse std\nuse std.solids\nin std.front {\n\
    p := point hint((0, 0))\n  q := point hint((40, 0))\n  r := point hint((40, 30))\n\
    s := point hint((0, 30))\n  t := point hint((15, 12))\n  u := point hint((25, 18))\n\
    l := line(p, q)\n  m := line(q, r)\n  lc := line(r, s)\n  o := line(s, p)\n\
    g := line(t, u)\n  h := line(hint((60, 0)), hint((70, 10)))\n\
    c := point hint((20, 15))\n  k := circle(center: c) hint(r: 5)\n\
    e := point hint((80, 20))\n  j := circle(center: e) hint(r: 4)\n\
    a1 := point hint((26, 15))\n  a2 := point hint((20, 21))\n  z := point hint((5, 5)) in std.top\n\
    w := arc(center: c, start: a1, end: a2)\n  kk := circle(center: c) hint(r: 9)\n\
    k0 := point hint((90, 0))\n  k1 := point hint((95, 10))\n  k2 := point hint((100, 0))\n\
    k3 := point hint((105, 10))\n  sp := spline(k0, k1, k2, k3)\n\
    v0 := point hint((0, -20))\n  v1 := point hint((30, -20))\n  rope := curve(v0, v1)\n\
    f := face(l, m, lc, o)\n}\nax := axis hint(dir: (0, 1, 0))\nbx := axis hint(dir: (0, 0, 1))\n\
    body := solid(std.solids.Prism(f, from: 0mm, to: 8mm))\n";

fn request(id: &str, args: &[(&str, &str)], plane: Option<&str>) -> WordRequest {
    WordRequest {
        id: id.into(),
        args: args.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        plane: plane.map(str::to_string),
    }
}

/// `id` written with `args`, the source it leaves or the elaborator's refusal.
fn write(src: &str, id: &str, args: &[(&str, &str)]) -> Result<edit::Edit, String> {
    let prog = parse_linked(src).0;
    let e = edit::write_word(&prog, &request(id, args, None));
    match &e.refused {
        Some(why) => Err(why.clone()),
        None => Ok(e),
    }
}

fn entry(id: &str) -> Entry {
    lexicon::entry(None, id).unwrap_or_else(|| panic!("no word `{id}` in the lexicon"))
}

fn slot<'a>(e: &'a Entry, key: &str) -> &'a Slot {
    e.slots.iter().find(|s| s.key == key).unwrap_or_else(|| panic!("`{}` has no slot `{key}`", e.id))
}

fn statement(id: &str, args: &[(&str, &str)], plane: Option<&str>) -> String {
    let e = entry(id);
    let args: BTreeMap<String, String> =
        args.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect();
    lexicon::statement(&e, &args, plane).unwrap_or_else(|why| panic!("{id}: {why}"))
}

/// The modules a person draws with, each one's components and relation words: every one is a
/// word of the lexicon, by its path, with a sentence saying what it is.
#[test]
fn every_library_word_is_in_the_lexicon() {
    let words = lexicon::lexicon(None);
    let has = |id: &str| words.iter().find(|e| e.id == id);
    let mut missing = Vec::new();
    let mut unsaid = Vec::new();
    let mut seen = 0;
    for (module, text) in MODULES.iter().filter(|(m, _)| m.starts_with("std") || *m == "hardware") {
        let (prog, _) = parse(text);
        let ids = prog.components.iter().filter_map(|c| c.name.as_ref())
            .map(|n| format!("{module}.{}", n.text))
            .chain(prog.words.iter().map(|w| format!("{module}.{}", w.word.text)));
        for id in ids {
            seen += 1;
            match has(&id) {
                None => missing.push(id),
                Some(e) if e.doc.is_empty() => unsaid.push(id),
                Some(_) => {}
            }
        }
    }
    assert!(seen > 100, "only {seen} library words read");
    assert!(missing.is_empty(), "not in the lexicon: {missing:?}");
    // a definition the library left uncommented says nothing, and that is a fault of the library
    // the menu shows: at most a handful
    assert!(unsaid.len() <= seen / 5, "no sentence for {} of {seen}: {unsaid:?}", unsaid.len());
    // and the language's own words, each said
    for e in words.iter().filter(|e| e.module.is_empty()) {
        assert!(!e.doc.is_empty(), "the language's `{}` says nothing", e.id);
    }
    // one entry per id
    let mut ids: Vec<&str> = words.iter().map(|e| e.id.as_str()).collect();
    ids.sort();
    let twice: Vec<&str> = ids.windows(2).filter(|w| w[0] == w[1]).map(|w| w[0]).collect();
    assert!(twice.is_empty(), "ids given twice: {twice:?}");
}

/// A built-in word's operands are the pairs `infix_op` settles it over, and its parentheses the
/// specs of what those settle to: one word, read off one table.
#[test]
fn a_built_in_word_is_read_off_the_tables_that_settle_it() {
    let pair = |e: &Entry, a: &str, b: &str| e.pairs.iter().any(|(x, y, _)| x == a && y == b);
    let needs = |e: &Entry, a: &str, b: &str| {
        e.pairs.iter().find(|(x, y, _)| x == a && y == b).map(|p| p.2.clone()).unwrap_or_default()
    };
    let co = entry("coincident");
    assert_eq!(co.form, Form::Infix);
    for (a, b) in [("point", "point"), ("point", "line"), ("point", "circle"), ("point", "plane"),
                   ("line", "axis"), ("axis", "plane")] {
        assert!(pair(&co, a, b), "coincident takes {a} and {b}");
    }
    assert!(!pair(&co, "circle", "point"), "a circle is not coincident with a point, that way round");
    assert_eq!(slot(&co, "t").role, Role::Pin);
    // two lines touch only at a point named on one, and a point is off a plane along its normal
    assert_eq!(needs(&entry("tangent"), "line", "line"), ["at"]);
    assert_eq!(needs(&entry("distance"), "point", "plane"), ["along"]);
    assert!(needs(&entry("distance"), "point", "point").is_empty());
    let d = entry("distance");
    let v = slot(&d, "value");
    assert_eq!((v.role, v.ty.as_deref(), v.optional), (Role::Value, Some("Length"), false));
    assert!(slot(&d, "side").words.contains(&"left".to_string()));
    assert!(slot(&d, "along").words.contains(&"u".to_string()));
    // an angle made equal to another has no number of its own
    let an = entry("angle");
    assert!(slot(&an, "value").optional);
    // a level's direction stands bare in its parentheses
    let lv = slot(&entry("level"), "along").clone();
    assert_eq!(lv.role, Role::Arg);
    assert!(lv.words.contains(&"up".to_string()));
    // a tangency at a point named, or at an end by its word
    let at = slot(&entry("tangent"), "at").clone();
    assert!(at.kinds.contains(&"point".to_string()) && at.words.contains(&"start".to_string()));
    assert_eq!(slot(&entry("horizontal"), "line").kinds, ["line"]);
    let r = slot(&entry("radius"), "circle").kinds.clone();
    assert!(r.contains(&"circle".to_string()) && r.contains(&"arc".to_string()) && r.len() == 2);
}

/// A word a module defines takes what its body hands its operands and parameters to: `b
/// right_of(d) a` is `a distance(d, along: right) b` and `a level(up) b`, so both are points and
/// `d` a length.
#[test]
fn a_module_word_takes_what_its_body_gives() {
    let w = entry("std.place.right_of");
    assert_eq!(w.form, Form::Infix);
    assert_eq!(slot(&w, "b").kinds[0], "point");
    assert_eq!(slot(&w, "a").kinds[0], "point");
    assert_eq!(slot(&w, "d").ty.as_deref(), Some("Length"));
    assert_eq!(slot(&entry("std.place.horizontal"), "a").kinds[0], "point");
    // a word defined both ways is two words, told apart by form
    assert_eq!(entry("std.rounds.round").form, Form::Infix);
    assert_eq!(entry("std.rounds.round (prefix)").form, Form::Prefix);
    // a component's formals, typed as written
    let prism = entry("std.solids.Prism");
    assert_eq!(prism.form, Form::Instance { solid: true });
    assert_eq!(slot(&prism, "section").kinds, ["face"]);
    assert_eq!(slot(&prism, "to").ty.as_deref(), Some("Length"));
    let rev = entry("std.solids.Revolution");
    assert_eq!(slot(&rev, "from").default.as_deref(), Some("0deg"));
    assert!(slot(&rev, "from").optional);
    let hex = entry("std.shapes.Hex");
    assert!(!hex.doc.is_empty());
    assert_eq!(slot(&hex, "c").kinds, ["point"]);
}

/// Each word written as a person writes it: operands either side, the number bare and first, a
/// selector labelled, entities of a call by position and its numbers by label.
#[test]
fn a_statement_is_written_as_a_person_writes_it() {
    let cases: &[(&str, &[(&str, &str)], Option<&str>, &str)] = &[
        ("distance", &[("p", "p"), ("q", "q"), ("value", "80")], None, "p distance(80) q"),
        ("distance", &[("p", "p"), ("q", "l"), ("value", "5"), ("side", "left")], None,
         "p distance(5, side: left) l"),
        ("level", &[("p", "p"), ("q", "q"), ("along", "up")], None, "p level(up) q"),
        ("coincident", &[("p", "p"), ("q", "l"), ("t", "0.25")], None, "p coincident(t == 0.25) l"),
        ("symmetry", &[("p", "p"), ("q", "q"), ("line", "g")], None, "p symmetry(g) q"),
        ("angle", &[("l1", "l"), ("l2", "m"), ("l3", "n"), ("l4", "o")], None, "l angle(n, o) m"),
        ("mapped", &[("q", "s"), ("p", "t"), ("from", "p"), ("a", "q"), ("to", "r"), ("b", "s")],
         None, "s mapped(p -> q, r -> s) t"),
        ("radius", &[("circle", "k"), ("value", "8")], None, "radius(8) k"),
        ("horizontal", &[("line", "l")], None, "horizontal l"),
        ("ccw", &[("a", "p"), ("b", "q"), ("c", "r")], None, "ccw(p, q, r)"),
        ("fix", &[("of", "p"), ("value", "(0, 0)")], None, "fix((0, 0)) p"),
        ("line", &[("name", "l9"), ("p1", "p"), ("p2", "q")], Some("std.front"),
         "l9 := line(p, q) in std.front"),
        ("circle", &[("name", "k9"), ("center", "c"), ("r", "5")], Some("std.front"),
         "k9 := circle(center: c) hint(r: 5) in std.front"),
        ("arc", &[("name", "w9"), ("center", "c"), ("start", "a1"), ("end", "a2"), ("sense", "cw")],
         None, "w9 := arc(center: c, start: a1, end: a2, sense: cw)"),
        ("spline", &[("name", "s9"), ("ctrl", "k0, k1, k2, k3")], None, "s9 := spline(k0, k1, k2, k3)"),
        ("face", &[("name", "f9"), ("edges", "l, m, n, o"), ("holes", "k")], None,
         "f9 := face(l, m, n, o, holes: k)"),
        ("std.place.right_of", &[("b", "q"), ("a", "p"), ("d", "30")], None, "q right_of(d: 30) p"),
        ("std.solids.Prism", &[("name", "b9"), ("section", "f"), ("from", "0mm"), ("to", "8mm")],
         None, "b9 := solid(std.solids.Prism(f, from: 0mm, to: 8mm))"),
        ("std.shapes.Hex", &[("name", "hx"), ("c", "c"), ("ref", "l"), ("af", "10"), ("phase", "0deg")],
         None, "hx := std.shapes.Hex(c, l, af: 10, phase: 0deg)"),
        ("std.sets.Ball", &[("name", "ball"), ("center", "c"), ("r", "12mm")], None,
         "ball := std.sets.Ball(c, r: 12mm)"),
        ("param", &[("name", "w"), ("type", "Length"), ("value", "100")], None, "param w: Length := 100"),
        ("param", &[("name", "a"), ("type", "Angle")], None, "param a: Angle"),
        ("value", &[("name", "half"), ("value", "w / 2")], None, "half := w / 2"),
    ];
    for (id, args, plane, want) in cases {
        assert_eq!(statement(id, args, *plane), *want, "{id}");
    }
    // a required slot left empty is said, by its key
    let e = entry("distance");
    let args: BTreeMap<String, String> = [("p".to_string(), "p".to_string())].into();
    assert_eq!(lexicon::statement(&e, &args, None).unwrap_err(), "distance still needs `value`");
}

/// Written into a document, a word brings its module in — a relation word bare — mints a name
/// for what it declares, and is refused in the elaborator's words where what it says is wrong.
#[test]
fn writing_a_word_imports_it_and_holds_it_to_the_elaborator() {
    let e = write(DOC, "std.place.right_of", &[("b", "q"), ("a", "p"), ("d", "30")]).unwrap();
    assert!(e.text.contains("use std.place (right_of)\n"), "{}", e.text);
    assert!(e.text.trim_end().ends_with("q right_of(d: 30) p"), "{}", e.text);
    // a second word of the module joins the first's `use`
    let again = write(&e.text, "std.place.above", &[("b", "r"), ("a", "q"), ("d", "30")]).unwrap();
    assert!(again.text.contains("use std.place (right_of, above)\n"), "{}", again.text);
    // a component of a module the document already uses needs nothing more
    let prism = write(DOC, "std.solids.Prism", &[("section", "f"), ("from", "0mm"), ("to", "3mm")])
        .unwrap();
    assert_eq!(prism.text.matches("use std.solids").count(), 1);
    assert_eq!(prism.names, ["b0"], "a solid's name is minted in its kind's alphabet");
    // and a set family brings its module in
    let ball = write(DOC, "std.sets.Ball", &[("center", "c"), ("r", "12mm")]).unwrap();
    assert!(ball.text.contains("use std.sets\n"), "{}", ball.text);
    // an entity's name in its kind's alphabet
    let line = write(DOC, "line", &[("p1", "t"), ("p2", "r")]).unwrap();
    assert_eq!(line.names, ["l0"]);
    // refused: a slot left empty, a name taken, and what the elaborator will not have
    assert_eq!(write(DOC, "distance", &[("p", "p"), ("q", "q")]).unwrap_err(),
               "distance still needs `value`");
    assert_eq!(write(DOC, "line", &[("name", "l"), ("p1", "p"), ("p2", "q")]).unwrap_err(),
               "`l` is already a name in this document");
    assert!(write(DOC, "radius", &[("circle", "l"), ("value", "5")]).is_err());
    assert!(write(DOC, "distance", &[("p", "p"), ("q", "nowhere"), ("value", "5")]).is_err());
    // the text a front end shows is the text written
    let prog = parse_linked(DOC).0;
    let r = request("distance", &[("p", "p"), ("q", "q"), ("value", "40")], None);
    assert_eq!(edit::word_text(&prog, &r).unwrap(), "p distance(40) q");
}

/// **Every pair a built-in word publishes writes**: each word, over two things of each pair of
/// kinds it says it takes, its number and its third entities given, elaborates without an error
/// in what it wrote — so the menu offers nothing the language refuses.
#[test]
fn every_built_in_pair_writes() {
    let of_kind: &[(&str, &[&str])] = &[
        ("point", &["t", "u"]), ("line", &["g", "h"]), ("circle", &["k", "kk"]), ("arc", &["w", "w"]),
        ("spline", &["sp", "sp"]), ("curve", &["rope", "rope"]), ("plane", &["std.top", "std.side"]),
        ("axis", &["ax", "bx"]), ("face", &["f", "f"]), ("solid", &["body", "body"]),
    ];
    let named = |k: &str, i: usize| of_kind.iter().find(|(n, _)| *n == k).map(|(_, v)| v[i]);
    let mut refused = Vec::new();
    let mut written = 0;
    for e in lexicon::lexicon(None).iter().filter(|e| e.module.is_empty() && e.form == Form::Infix) {
        for (a, b, needs) in &e.pairs {
            let (Some(x), Some(mut y)) = (named(a, 0), named(b, 1)) else { continue };
            // two images of one point are drawn in two views
            if e.id == "project" {
                y = "z";
            }
            let key = |r: &str| e.slots.iter().find(|s| s.role.as_str() == r).map(|s| s.key.as_str()).unwrap();
            let mut args: Vec<(&str, String)> = vec![(key("left"), x.into()), (key("right"), y.into())];
            for key in needs {
                args.push((key, match key.as_str() {
                    "at" => "t".into(),
                    "along" => "n".into(),
                    "t" => "0.5".into(),
                    "about" => "bx".into(),
                    _ => "left".into(),
                }));
            }
            for s in &e.slots {
                // a number is given wherever the word has one; any other optional slot is not
                if s.optional && s.role != Role::Value {
                    continue;
                }
                match s.role {
                    Role::Value => args.push((&s.key, if s.ty.as_deref() == Some("Angle") {
                        "30deg".into()
                    } else {
                        "5".into()
                    })),
                    Role::Arg if !s.words.is_empty() => args.push((&s.key, s.words[0].clone())),
                    Role::Arg => args.push((&s.key, named(&s.kinds[0], 0).unwrap_or("p").into())),
                    _ => {}
                }
            }
            let args: Vec<(&str, &str)> = args.iter().map(|(k, v)| (*k, v.as_str())).collect();
            written += 1;
            if let Err(why) = write(DOC, &e.id, &args) {
                refused.push(format!("{} {a} {b}: {why}", e.id));
            }
        }
    }
    assert!(written > 40, "only {written} pairs written");
    assert!(refused.is_empty(), "{} of {written} refused:\n{}", refused.len(), refused.join("\n"));
}

/// The document's own components and words join the lexicon, after the library's.
#[test]
fn a_document_s_own_words_are_words() {
    let src = format!("{DOC}component Lug(c: point, d: Length) {{\n  x := point\n  \
        c distance(d) x\n}}\na spaced(d) b := a distance(d) b\n");
    let prog = parse_linked(&src).0;
    let words = lexicon::lexicon(Some(&prog));
    let lug = words.iter().find(|e| e.id == "Lug").expect("Lug is a word");
    assert_eq!(lug.module, ".");
    assert_eq!(slot(lug, "c").kinds, ["point"]);
    let spaced = words.iter().find(|e| e.id == "spaced").expect("spaced is a word");
    assert_eq!(slot(spaced, "a").kinds, ["point", "line", "circle", "arc", "plane"]);
    assert_eq!(slot(spaced, "d").ty.as_deref(), Some("Length"));
    let w = edit::write_word(&prog, &request("spaced", &[("a", "p"), ("b", "q"), ("d", "12")], None));
    assert!(w.refused.is_none(), "{:?}", w.refused);
    assert!(w.text.contains("\np spaced(d: 12) q\n"), "{}", w.text);
}
