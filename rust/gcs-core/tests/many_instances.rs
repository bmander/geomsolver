//! An assembly of many instances elaborates in work linear in how many (#403).  The walk carried
//! the numbers in force on every statement it emitted, and the root's table holds every
//! instance's numbers by name (`P0.w`), so a copy per statement was quadratic in the instances:
//! a thousand plates ran a 15 GB machine out of memory while the same statements written inline
//! elaborated in under a second.  The measure is the bytes the elaboration allocates, counted on
//! the thread doing it, which is deterministic where a clock is not.
//!
//! The table itself is then the cost (#405): a block's copy, a `when` and a seed read with any
//! number in force free each built a copy of the root's, so a `repeat` beside the plates, or one
//! `param` with no value, made the work the plates times the copies, or the plates squared.
//! Each adds a frame of its own over the table now, and a seed reads through it.
//!
//! The count is the allocator of the whole core suite, which is one binary (`tests/main.rs`):
//! `System`'s, with a per-thread tally beside it, so a test on another thread does not count.
use gcs_core::{library, program};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::fmt::Write;

struct Counted;

thread_local! {
    // `const`, with no destructor: reading it allocates nothing, so the allocator may read it
    static ALLOCATED: Cell<usize> = const { Cell::new(0) };
}

unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATED.with(|n| n.set(n.get().wrapping_add(layout.size())));
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        ALLOCATED.with(|n| n.set(n.get().wrapping_add(size.saturating_sub(layout.size()))));
        unsafe { System.realloc(ptr, layout, size) }
    }
}

#[global_allocator]
static COUNTED: Counted = Counted;

/// What stands beside the plates (`with`).
#[derive(Clone, Copy)]
enum Beside {
    /// Nothing: the plates alone.
    Nothing,
    /// The hole margin `e` a `param` with no value, so an unknown every plate reads.
    FreeMargin,
    /// A `repeat` of this many copies at the root, each a point.
    Repeat(usize),
    /// This many `when`s at the root, each holding and each a point.
    Whens(usize),
}

/// `n` plates, each an instance of one component — a dimensioned rectangle with four equal
/// holes, held by its own three numbers — and each placed off its neighbour: the issue's
/// document, without its jitter.  Each instance is written and named by hand (`P0`, `P1`, …),
/// not by a `repeat`: the root's table holds a named instance's numbers (`P0.w`), and that table
/// is what was copied.  `beside` stands with them.
fn plates(n: usize, beside: Beside) -> String {
    let (w, h, e, r, cols) = (80.0, 50.0, 10.0, 4.0, 40);
    let margin = match beside {
        Beside::FreeMargin => format!("param e: Length hint({e})"),
        _ => format!("e := {e}"),
    };
    let mut out = format!("\
use std
w := {w}
h := {h}
{margin}
component Plate(a: point, b: point, c: point, d: point,
                h1: circle, h2: circle, h3: circle, h4: circle,
                w: Length, h: Length, e: Length) {{
  horizontal (bot := line(a, b)) -> vertical (rt := line(b, c)) ->
    horizontal (top := line(c, d)) -> vertical (lf := line(d, a)) -> close
  length(w) bot
  length(h) rt
  radius({r}) h1
  h2 equal h1
  h3 equal h1
  h4 equal h1
  h1.center distance(e) bot
  h1.center distance(e) lf
  h2.center distance(e) bot
  h2.center distance(e) rt
  h3.center distance(e) top
  h3.center distance(e) rt
  h4.center distance(e) top
  h4.center distance(e) lf
}}
in std.front {{
");
    for i in 0..n {
        let x0 = (i % cols) as f64 * (w + 20.0);
        let y0 = (i / cols) as f64 * (h + 20.0);
        let corners = [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)];
        for (k, (x, y)) in ["a", "b", "c", "d"].iter().zip(corners) {
            writeln!(out, "  {k}{i} := point hint(({x}, {y}))").unwrap();
        }
        let holes = [(x0 + e, y0 + e), (x0 + w - e, y0 + e), (x0 + w - e, y0 + h - e),
                     (x0 + e, y0 + h - e)];
        for (k, (x, y)) in (1..).zip(holes) {
            writeln!(out, "  c{i}_{k} := point hint(({x}, {y}))").unwrap();
            writeln!(out, "  h{i}_{k} := circle(c{i}_{k}) hint(r: {r})").unwrap();
        }
        writeln!(out, "  P{i} := Plate(a{i}, b{i}, c{i}, d{i}, h{i}_1, h{i}_2, h{i}_3, h{i}_4, \
                       w: w, h: h, e: e)").unwrap();
        if i == 0 {
            writeln!(out, "  fix((0, 0)) a0").unwrap();
        } else {
            // off the plate before it in the row, or the one below it in the column
            let (prev, gap, way) = match i % cols {
                0 => (i - cols, h + 20.0, "up"),
                _ => (i - 1, w + 20.0, "right"),
            };
            writeln!(out, "  a{prev} distance({gap}) a{i}\n  a{prev} level({way}) a{i}").unwrap();
        }
    }
    match beside {
        Beside::Repeat(k) => {
            writeln!(out, "  repeat {k} as i {{\n    q := point hint((i * 5, -100))\n  }}").unwrap();
        }
        Beside::Whens(k) => {
            for j in 0..k {
                writeln!(out, "  when w > 0 {{\n    q{j} := point hint(({}, -100))\n  }}", j * 5)
                    .unwrap();
            }
        }
        Beside::Nothing | Beside::FreeMargin => {}
    }
    out + "}\n"
}

/// The bytes the elaboration of `n` plates allocates, and how many unknowns it made.  Parsed
/// here rather than by `common::linked::build`, which parses and elaborates in one call: only the
/// elaboration is counted.
fn elaborated(n: usize) -> (usize, usize) {
    elaborated_with(n, Beside::Nothing)
}

/// `elaborated`, with `beside` beside the plates.
fn elaborated_with(n: usize, beside: Beside) -> (usize, usize) {
    let (p, errors, linked) = library::parse_linked(&plates(n, beside));
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    let before = ALLOCATED.with(Cell::get);
    let e = program::elaborate(&p);
    let bytes = ALLOCATED.with(Cell::get).wrapping_sub(before);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(e.map.ent_named(&format!("P{}.bot", n - 1)).is_some(), "the last plate drawn");
    (bytes, e.sketch.params.iter().filter(|q| !q.fixed).count())
}

/// **Four times the plates is four times the work**: the bytes elaborating 200 plates allocates
/// are about four times 50's, where a table copied per statement made them sixteen.
#[test]
fn many_instances_elaborate_in_linear_work() {
    let (small, small_params) = elaborated(50);
    let (large, large_params) = elaborated(200);
    // the same drawing four times over: twenty unknowns a plate, two less the one held
    assert_eq!(small_params, 20 * 50 - 2);
    assert_eq!(large_params, 20 * 200 - 2);
    let ratio = large as f64 / small as f64;
    assert!(ratio < 5.0, "200 plates allocate {ratio:.1}× what 50 do ({large} bytes, {small})");
}

/// **One free number at the root is not a copy of the root per declaration** (#405): with the
/// margin a `param` with no value, every seed was read through a table built from the whole root
/// — every plate's numbers — for every declaration, so four times the plates allocated sixteen
/// times the bytes.
#[test]
fn a_free_number_at_the_root_elaborates_in_linear_work() {
    let (small, small_params) = elaborated_with(50, Beside::FreeMargin);
    let (large, large_params) = elaborated_with(200, Beside::FreeMargin);
    // the margin is the drawing's one unknown more
    assert_eq!(small_params, 20 * 50 - 1);
    assert_eq!(large_params, 20 * 200 - 1);
    let ratio = large as f64 / small as f64;
    assert!(ratio < 5.0, "200 plates allocate {ratio:.1}× what 50 do ({large} bytes, {small})");
}

/// **A block beside the instances costs its copies, not its copies times the instances** (#405):
/// each copy of a root `repeat`, and each root `when`, copied the root's table — every plate's
/// numbers — so four times the plates and four times the copies allocated sixteen times the
/// bytes.  Each is a frame over the root's table now.
#[test]
fn blocks_beside_many_instances_elaborate_in_linear_work() {
    for (block, beside) in [("repeat", Beside::Repeat as fn(usize) -> Beside), ("when", Beside::Whens)] {
        let (small, small_params) = elaborated_with(50, beside(50));
        let (large, large_params) = elaborated_with(200, beside(200));
        // a point of two unknowns per copy, beside the plates'
        assert_eq!(small_params, 20 * 50 - 2 + 2 * 50, "{block}");
        assert_eq!(large_params, 20 * 200 - 2 + 2 * 200, "{block}");
        let ratio = large as f64 / small as f64;
        assert!(ratio < 5.0, "{block}: 200 plates and 200 copies allocate {ratio:.1}× what 50 \
                              and 50 do ({large} bytes, {small})");
    }
}
