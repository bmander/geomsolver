//! **The row's kernel** (`KernelKey::Interior`): one row read off its columns and constants
//! (`Row`), its structure solved there, its output less what it states and its adjoint rate —
//! the last meshed row kept, since a step's accepted trial is where the next rate is taken.

use super::consts::{self, Drive, Mover};
use super::{hull, moved, Hull, Moved, State, Structure, SECTION};
use crate::diagnose::analysis::{self, Reach};
use crate::kernels::{curve_widths, Kernel};

/// The kernel at a block's widths (`system::build_kernel`, `KernelKey::Interior`): one row, the
/// output less what it states, `J(x) − J₀` or `J(x) − (m·a + c)`.  A displacement, so degree 1;
/// a measure's row (`Reads::Volume`, `Reads::Mass`) is the same kernel, no solve made, its degree
/// still 1 — the residual of a number a scenario states, not of a length the drawing draws.
/// A structure with no displacement to give (`Failure`), and constants that are no structure,
/// read NaN, which the step rejects.
pub fn kernel(n_par: usize, n_const: usize) -> Kernel {
    Kernel { name: "interior", n_res: 1, n_par, n_const, degree: 1, res, jac, const_jac: None,
             form: None }
}

/// One row read off its columns `v` and constants `k`: the structure, the columns it is solved
/// at, what it states, how many columns it reads before the unknown that may set its output,
/// whether one does, and how its columns drive it.
struct Row {
    s: Structure,
    x: Vec<f64>,
    stated: Vec<f64>,
    cols: usize,
    free: bool,
    driven: Driven,
}

/// **A row's drive read where its columns stand** (`Drive`): its sections each with its value and
/// rate in the unknowns, or its nodes placed with what their rate is carried by.
enum Driven {
    Stated,
    Sized(usize, Vec<[crate::tape::Value; SECTION]>),
    Plane(Moved, moved::Placed),
    Hull(Hull, hull::Placed),
}

impl Row {
    fn read(v: &[f64], k: &[f64]) -> Option<Row> {
        // the unknown's column is one more than the columns the head says the structure reads
        let cols = consts::columns(k)?;
        let free = match v.len().checked_sub(cols)? {
            0 => false,
            1 => true,
            _ => return None,
        };
        let (s, stated, drive) = Structure::read(k, free)?;
        let stated = stated.to_vec();
        // a measure reads the boundary alone: its inner nodes need no harmonic map
        let measure = s.reads.measure();
        // the places move into the row: the rate reads its factors alone
        let (x, driven) = match drive {
            Drive::Stated => (v[..cols].to_vec(), Driven::Stated),
            Drive::Sized(z) => {
                let direct = s.section_at(0);
                let mut x = v[..direct].to_vec();
                let mut sc = crate::tape::Scratch::new();
                let sections: Vec<_> =
                    (0..s.beams.len()).map(|i| z.section(i, &v[direct..cols], &mut sc)).collect();
                x.extend(sections.iter().flatten().map(|t| t.v));
                (x, Driven::Sized(z.unknowns, sections))
            }
            Drive::Moved(Mover::Plane(m)) => {
                let mut placed =
                    if measure { m.boundary(&v[..cols]) } else { m.places(&v[..cols])? };
                (std::mem::take(&mut placed.x), Driven::Plane(m, placed))
            }
            Drive::Moved(Mover::Hull(h)) => {
                let tets: Vec<[usize; 10]> = s.tets.iter().map(|t| t.n).collect();
                let mut placed = h.places(&v[..cols], &tets, !measure)?;
                (std::mem::take(&mut placed.x), Driven::Hull(h, placed))
            }
        };
        Some(Row { s, x, stated, cols, free, driven })
    }

    /// Whether its nodes move with the drawing: a region's, a prism's, a filled body's.
    fn meshed(&self) -> bool {
        matches!(self.driven, Driven::Plane(..) | Driven::Hull(..))
    }

    /// What the row states the output is, at the unknown's column `v` reads.
    fn target(&self, v: &[f64]) -> f64 {
        match self.free {
            true => self.stated[0] * v[self.cols] + self.stated[1],
            false => self.stated[0],
        }
    }
}

fn res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    for i in 0..n {
        let (v, k) = (&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)]);
        r[i] = reading(v, k).map_or(f64::NAN, |(output, target)| output - target);
    }
}

fn jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    for i in 0..n {
        let (v, k) = (&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)]);
        let row = &mut j[n_par * i..n_par * (i + 1)];
        row.fill(0.0);
        let Some(got) = solved(v, k) else {
            row.fill(f64::NAN);
            continue;
        };
        got.row.rate(&got.state, v, row);
    }
}

impl Row {
    /// What the row reads where it stands, solved at `st`: the displacement `Σ weight · u`, or
    /// what the structure is made of (`Reads::Volume`, `Reads::Mass`), which no solve reads.
    fn output(&self, st: &Option<State>) -> f64 {
        match st {
            Some(st) => self.s.output(st),
            None => self.s.measure(&self.x),
        }
    }

    /// The row's rate in its columns `v`, solved at `st`, into `row` (zeroed): the structure's
    /// adjoint gradient — or a measure's own, where nothing is solved — carried to the drawing's
    /// columns, and the unknown's where it sets what the row states.
    fn rate(&self, st: &Option<State>, v: &[f64], row: &mut [f64]) {
        let s = &self.s;
        let cols = self.cols;
        let gradient = |g: &mut [f64]| match st {
            Some(st) => s.gradient(st, &self.x, g),
            None => s.measure_gradient(&self.x, g),
        };
        match &self.driven {
            // the nodes' rates carried through how they move with the drawing (#510 S2)
            Driven::Plane(m, placed) => {
                let mut g = vec![0.0; s.columns()];
                gradient(&mut g);
                m.chain(&v[..cols], placed, &g, &mut row[..cols]);
            }
            // a filled body's, through its sheets (#567)
            Driven::Hull(h, placed) => {
                let mut g = vec![0.0; s.columns()];
                gradient(&mut g);
                if h.chain(&v[..cols], placed, &g, &mut row[..cols]).is_none() {
                    row.fill(f64::NAN);
                    return;
                }
            }
            Driven::Stated => gradient(&mut row[..cols]),
            // a section's rate carried to the unknowns it is read over
            Driven::Sized(unknowns, sections) => {
                let mut g = vec![0.0; s.columns()];
                gradient(&mut g);
                let direct = s.section_at(0);
                row[..direct].copy_from_slice(&g[..direct]);
                for (b, four) in sections.iter().enumerate() {
                    for (c, t) in four.iter().enumerate() {
                        for u in 0..*unknowns {
                            row[direct + u] += g[s.section_at(b) + c] * t.d[u];
                        }
                    }
                }
            }
        }
        if self.free {
            row[cols] = -self.stated[0];
        }
    }
}

/// **A row read and solved** (`solved`): the row, and its structure's displacements (none for a
/// measure's).
struct Solved {
    row: Row,
    state: Option<State>,
}

thread_local! {
    /// **The last meshed row solved on this thread** (#567): its columns, its constants, and the
    /// row read and solved there — what `jac` reads again where `res` last stood, since a step's
    /// trial that is accepted is where the next step's rate is taken.  Kept for a row over a mesh
    /// moved with the drawing (a region's, a prism's, a filled body's), whose reading is its
    /// placement and its structure's solve; the same numbers give the same row to the bit, so what
    /// is kept changes no answer.  Let go when a document solve ends (`forget`).
    static LAST: std::cell::RefCell<Option<(Vec<f64>, Vec<f64>, std::rc::Rc<Solved>)>> =
        const { std::cell::RefCell::new(None) };
}

/// **The last row solved on this thread let go** (`LAST`): a filled body's holds its mesh, its
/// placement's factors and its stiffness, so a document solve lets it go when it ends.
pub fn forget() {
    LAST.with(|last| *last.borrow_mut() = None);
}

/// One row read off its columns `v` and constants `k`, and its structure solved there — a
/// measure's left unsolved (`None`), since it reads what the structure is made of; `None` where it
/// cannot be read or solved.  A meshed row read where the last one was is that one (`LAST`).
fn solved(v: &[f64], k: &[f64]) -> Option<std::rc::Rc<Solved>> {
    let same = |a: &[f64], b: &[f64]| {
        a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
    };
    let kept = LAST.with(|last| {
        last.borrow().as_ref().filter(|(lv, lk, _)| same(lv, v) && same(lk, k))
            .map(|(_, _, got)| got.clone())
    });
    if kept.is_some() {
        return kept;
    }
    let row = Row::read(v, k)?;
    let meshed = row.meshed();
    let state = if row.s.reads.measure() { None } else { Some(row.s.solve(&row.x).ok()?) };
    let got = std::rc::Rc::new(Solved { row, state });
    if meshed {
        LAST.with(|last| *last.borrow_mut() = Some((v.to_vec(), k.to_vec(), got.clone())));
    }
    Some(got)
}

/// One row's output where its columns `v` stand, and what it states there; `None` where the row
/// cannot be read or its structure solved there.
pub fn reading(v: &[f64], k: &[f64]) -> Option<(f64, f64)> {
    let got = solved(v, k)?;
    let (read, st) = (&got.row, &got.state);
    Some((read.output(st), read.target(v)))
}

/// **Whether one row is out of reach where its columns `v` stand** (#510 P7): its output and
/// what it states there, and its rate in its columns — the rate the solve steps on (`jac`) —
/// judged by `analysis::reach` over the columns `free` says the solve moves, each moved by its
/// `scale`; with the output.  One structure solved for all three.  `None` where the row cannot
/// be read or its structure solved there.
pub fn reach_of(v: &[f64], k: &[f64], free: &[bool], scale: &[f64]) -> Option<(Reach, f64)> {
    let got = solved(v, k)?;
    let (read, st) = (&got.row, &got.state);
    let mut g = vec![0.0; v.len()];
    read.rate(st, v, &mut g);
    if !g.iter().all(|g| g.is_finite()) {
        return None;
    }
    let output = read.output(st);
    Some((analysis::reach(output, read.target(v), &g, scale, free), output))
}
