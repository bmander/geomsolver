//! **A row's constants** (`KernelKey::Interior`): the structure an analysis row is solved over,
//! what it states, and what drives its columns (`Drive`), as a kernel's constants.  The layout is
//! stated once: one walk writes it (`Structure::write`), into a vector where the constants are
//! kept (`Structure::consts`) or into a count where only their width is wanted
//! (`Structure::widths`), and one walk reads it back in the same order (`Structure::read`), over a
//! cursor whose every step is bounded (`Take`) — a document's constants are untrusted input, and
//! the wasm core aborts on a panic.  The head is named (`Head`), so neither the reader nor the
//! kernel (`columns`) reads a slot by its number.

use super::{Bar, Beam, Borne, Hull, Laid, Moved, Reads, Spread, Structure, Tet10, Tri6, SECTION};

/// **What drives a row's columns**: the structure's own numbers, each a column (`Stated`); the
/// beams' sections read over unknowns (`Sized`, #510 S1); or the nodes read over the drawing's
/// columns (`Moved`: a region's, a prism's, a filled body's, #510 S2 and #567), with no bars and
/// no beams.  A row has one, written as one tag in its head and its own payload after the
/// structure.
#[derive(Clone, Debug, PartialEq)]
pub enum Drive {
    Stated,
    Sized(Sizing),
    Moved(Mover),
}

impl Drive {
    /// Its tag in the head.
    fn tag(&self) -> Tag {
        match self {
            Drive::Stated => Tag::Stated,
            Drive::Sized(_) => Tag::Sized,
            Drive::Moved(Mover::Plane(m)) if m.stack.is_some() => Tag::Stacked,
            Drive::Moved(Mover::Plane(_)) => Tag::Plane,
            Drive::Moved(Mover::Hull(_)) => Tag::Hull,
        }
    }

    /// How many columns it reads in the structure's place: the unknowns the sections are read
    /// over, or the drawing's columns the nodes are; none where the structure's numbers are the
    /// columns.
    fn over(&self) -> usize {
        match self {
            Drive::Stated => 0,
            Drive::Sized(z) => z.unknowns,
            Drive::Moved(m) => m.columns(),
        }
    }
}

/// A drive as its head says it: `Plane` a region's nodes, `Stacked` a prism's over its face's,
/// `Hull` a filled body's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Tag {
    Stated,
    Sized,
    Plane,
    Stacked,
    Hull,
}

/// How many constants the head is (`Head`).
pub const HEAD: usize = 12;

/// **The constants before the structure**, in this order: `dim` and `nodes`; how many bars,
/// beams, spread loads, plane elements and loads laid on them (#510 S2), solid elements and loads
/// they bear (#510 S3); what the row reads (`Reads`: 0 a displacement, 1 the volume, 2 the mass);
/// the drive's tag (`Tag`: 0 stated, 1 sized, 2 a region's nodes moved, 3 a prism's, 4 a filled
/// body's); and how many columns the drive reads (`Drive::over`, 0 where the row is stated) —
/// then the output stated, `J₀`, or where an unknown sets it, its `(m, c)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Head {
    dim: usize,
    nodes: usize,
    bars: usize,
    beams: usize,
    spread: usize,
    tris: usize,
    laid: usize,
    tets: usize,
    borne: usize,
    reads: Reads,
    drive: Tag,
    over: usize,
}

impl Head {
    fn of(s: &Structure, drive: &Drive) -> Head {
        Head { dim: s.dim, nodes: s.nodes, bars: s.bars.len(), beams: s.beams.len(),
               spread: s.spread.len(), tris: s.tris.len(), laid: s.laid.len(), tets: s.tets.len(),
               borne: s.borne.len(), reads: s.reads, drive: drive.tag(), over: drive.over() }
    }

    fn write(&self, k: &mut impl Sink) {
        k.put([self.dim, self.nodes, self.bars, self.beams, self.spread, self.tris, self.laid,
               self.tets, self.borne, self.reads as usize, self.drive as usize, self.over]
            .map(|n| n as f64));
    }

    /// What `write` wrote, in its order (a struct's fields are read as they are listed).
    fn read(r: &mut Take) -> Option<Head> {
        Some(Head {
            dim: r.count()?,
            nodes: r.count()?,
            bars: r.count()?,
            beams: r.count()?,
            spread: r.count()?,
            tris: r.count()?,
            laid: r.count()?,
            tets: r.count()?,
            borne: r.count()?,
            reads: match r.count()? {
                0 => Reads::Displacement,
                1 => Reads::Volume,
                2 => Reads::Mass,
                _ => return None,
            },
            drive: match r.count()? {
                0 => Tag::Stated,
                1 => Tag::Sized,
                2 => Tag::Plane,
                3 => Tag::Stacked,
                4 => Tag::Hull,
                _ => return None,
            },
            over: r.count()?,
        })
    }

    /// How many columns a row over the structure reads before the unknown that may set its
    /// output: the nodes' coordinates, the bars' areas, then the beams' sections or the unknowns
    /// they are read over; or the drawing's columns the nodes are read over.  `None` where the
    /// count does not fit.
    fn columns(&self) -> Option<usize> {
        let direct = self.dim.checked_mul(self.nodes)?.checked_add(self.bars)?;
        match self.drive {
            Tag::Stated => direct.checked_add(SECTION.checked_mul(self.beams)?),
            Tag::Sized => direct.checked_add(self.over),
            Tag::Plane | Tag::Stacked | Tag::Hull => Some(self.over),
        }
    }
}

/// How many columns a row's constants `k` say its structure reads before the unknown that may
/// set its output (`Head::columns`); `None` where there is no head.
pub(super) fn columns(k: &[f64]) -> Option<usize> {
    Head::read(&mut Take::new(k))?.columns()
}

/// **How a row's nodes move with the drawing**: a region's (and a prism's stacked over it,
/// `Moved`), or a filled body's (`Hull`, #567).
#[derive(Clone, Debug, PartialEq)]
pub enum Mover {
    Plane(Moved),
    Hull(Hull),
}

impl Mover {
    /// How many columns of the drawing the nodes are read over.
    pub fn columns(&self) -> usize {
        match self {
            Mover::Plane(m) => m.columns,
            Mover::Hull(h) => h.columns,
        }
    }

    /// Its constants.
    pub fn consts(&self) -> Vec<f64> {
        match self {
            Mover::Plane(m) => m.consts(),
            Mover::Hull(h) => h.consts(),
        }
    }

    /// How many constants it is.
    pub fn width(&self) -> usize {
        match self {
            Mover::Plane(m) => m.width(),
            Mover::Hull(h) => h.width(),
        }
    }
}

/// **A row's sections read over its unknowns** (#510 S1): a beam sized by a requirement states
/// its section over unknowns of the drawing (`Iz: b * h^3 / 12`, `h` declared), so the row's
/// columns after the bars' areas are those `unknowns`, and each section's number is a function
/// of them — a flat tape (`tape::Tape::flat`), per beam in `SECTION`'s order, a stated number a
/// tape of one constant.  The structure is solved at the sections the tapes give, and its
/// gradient in a section's number is carried to the unknowns by the tape's own.
#[derive(Clone, Debug, PartialEq)]
pub struct Sizing {
    pub unknowns: usize,
    pub sections: Vec<[Vec<f64>; SECTION]>,
}

impl Sizing {
    /// Beam `i`'s section at the unknowns `u`, each number with its gradient in them.
    pub(super) fn section(&self, i: usize, u: &[f64], sc: &mut crate::tape::Scratch)
        -> [crate::tape::Value; SECTION]
    {
        self.sections[i].each_ref().map(|t| crate::tape::eval_flat(t, self.unknowns, u, sc))
    }
}

/// A spread load's constants before the beams it lies along: how many, whether it is a force an
/// area, and its force.
const SPREAD: usize = 5;

/// A beam's constants: its ends, its moduli, its density and its section's z.
const BEAM: usize = 8;

/// A plane element's constants: its six nodes, its modulus, ratio, density and depth, and
/// whether it is read in plane strain.
const TRI: usize = 11;

/// A laid load's constants before what it lies on: its kind, how many sides or elements, and
/// two numbers (a pressure and its depth, or a force).
const LAID: usize = 4;

/// A solid element's constants: its ten nodes, its modulus, ratio and density.
const TET: usize = 13;

/// A borne load's constants before what it lies on: its kind, how many facets or elements, and
/// three numbers (a pressure and two zeros, or a force).
const BORNE: usize = 5;

/// **Where a row's constants are written**: kept (`Vec<f64>`, `Structure::consts`), or only
/// counted (`usize`, `Structure::widths`) — so a row's width is what its writer writes.
trait Sink {
    fn put(&mut self, v: impl IntoIterator<Item = f64>);

    /// A block that writes itself (`Mover::consts`), `width` constants, made only where kept.
    fn block(&mut self, width: usize, consts: impl FnOnce() -> Vec<f64>);
}

impl Sink for Vec<f64> {
    fn put(&mut self, v: impl IntoIterator<Item = f64>) {
        self.extend(v);
    }

    fn block(&mut self, width: usize, consts: impl FnOnce() -> Vec<f64>) {
        let k = consts();
        debug_assert_eq!(k.len(), width, "a block's constants are its width");
        self.extend(k);
    }
}

impl Sink for usize {
    fn put(&mut self, v: impl IntoIterator<Item = f64>) {
        *self += v.into_iter().count();
    }

    fn block(&mut self, width: usize, _: impl FnOnce() -> Vec<f64>) {
        *self += width;
    }
}

/// **A walk over a row's constants** in the order they are written, every step bounded: what
/// is not there reads `None`, never past the end.
struct Take<'a> {
    k: &'a [f64],
    at: usize,
}

impl<'a> Take<'a> {
    fn new(k: &'a [f64]) -> Take<'a> {
        Take { k, at: 0 }
    }

    /// The next `n` constants.
    fn take(&mut self, n: usize) -> Option<&'a [f64]> {
        let out = self.k.get(self.at..self.at.checked_add(n)?)?;
        self.at += n;
        Some(out)
    }

    /// The next `n` records of `W` constants each.
    fn records<const W: usize>(&mut self, n: usize) -> Option<&'a [[f64; W]]> {
        Some(self.take(n.checked_mul(W)?)?.as_chunks::<W>().0)
    }

    /// The next constant, a count.
    fn count(&mut self) -> Option<usize> {
        count(*self.take(1)?.first()?)
    }

    /// A block that reads itself off what is left (`Moved::read`, `Hull::read`), saying how many
    /// constants it read.
    fn block<T>(&mut self, read: impl FnOnce(&'a [f64]) -> Option<(T, usize)>) -> Option<T> {
        let (t, used) = read(self.k.get(self.at..)?)?;
        self.take(used)?;
        Some(t)
    }

    /// Whether every constant is read.
    fn done(&self) -> bool {
        self.at == self.k.len()
    }
}

/// A constant that is a count: whole, not negative, and under a billion.
fn count(v: f64) -> Option<usize> {
    (v >= 0.0 && v.fract() == 0.0 && v < 1e9).then_some(v as usize)
}

impl Borne {
    /// Its constants: `[kind, count, a, b, c]`, then each facet's six nodes or each element.
    fn write(&self, k: &mut impl Sink) {
        let (kind, head) = match self {
            Borne::Pressure { p, .. } => (0.0, [*p, 0.0, 0.0]),
            Borne::Across { force, .. } => (1.0, *force),
            Borne::Through { force, .. } => (2.0, *force),
            Borne::Body { force, .. } => (3.0, *force),
        };
        let count = self.facets().len() + self.elements().len();
        k.put([kind, count as f64]);
        k.put(head);
        k.put(self.facets().iter().flatten().map(|&n| n as f64));
        k.put(self.elements().iter().map(|&e| e as f64));
    }

    /// What `write` wrote, its nodes under `nodes` and its elements under `tets`.
    fn read(r: &mut Take, nodes: usize, tets: usize) -> Option<Borne> {
        let c = r.records::<BORNE>(1)?[0];
        let (kind, n, f) = (count(c[0])?, count(c[1])?, [c[2], c[3], c[4]]);
        let node = |v: f64| count(v).filter(|&i| i < nodes);
        Some(if kind < 2 {
            let facets: Vec<[usize; 6]> = r.records::<6>(n)?.iter().map(|s| {
                let s = s.map(node);
                Some([s[0]?, s[1]?, s[2]?, s[3]?, s[4]?, s[5]?])
            }).collect::<Option<_>>()?;
            match kind {
                0 => Borne::Pressure { facets, p: f[0] },
                _ => Borne::Across { facets, force: f },
            }
        } else {
            let elements: Vec<usize> = r.take(n)?.iter()
                .map(|&v| count(v).filter(|&i| i < tets)).collect::<Option<_>>()?;
            match kind {
                2 => Borne::Through { elements, force: f },
                3 => Borne::Body { elements, force: f },
                _ => return None,
            }
        })
    }
}

impl Laid {
    /// Its constants: `[kind, count, a, b]`, then each side's three nodes or each element.
    fn write(&self, k: &mut impl Sink) {
        let (kind, head) = match self {
            Laid::Pressure { p, depth, .. } => (0.0, [*p, *depth]),
            Laid::Along { force, .. } => (1.0, *force),
            Laid::Over { force, .. } => (2.0, *force),
            Laid::Body { force, .. } => (3.0, *force),
        };
        let count = self.sides().len() + self.elements().len();
        k.put([kind, count as f64]);
        k.put(head);
        k.put(self.sides().iter().flatten().map(|&n| n as f64));
        k.put(self.elements().iter().map(|&e| e as f64));
    }

    /// What `write` wrote, its nodes under `nodes` and its elements under `tris`.
    fn read(r: &mut Take, nodes: usize, tris: usize) -> Option<Laid> {
        let c = r.records::<LAID>(1)?[0];
        let (kind, n, a, b) = (count(c[0])?, count(c[1])?, c[2], c[3]);
        let node = |v: f64| count(v).filter(|&i| i < nodes);
        Some(if kind < 2 {
            let sides: Vec<[usize; 3]> = r.records::<3>(n)?.iter()
                .map(|s| Some([node(s[0])?, node(s[1])?, node(s[2])?])).collect::<Option<_>>()?;
            match kind {
                0 => Laid::Pressure { sides, p: a, depth: b },
                _ => Laid::Along { sides, force: [a, b] },
            }
        } else {
            let elements: Vec<usize> = r.take(n)?.iter()
                .map(|&v| count(v).filter(|&i| i < tris)).collect::<Option<_>>()?;
            match kind {
                2 => Laid::Over { elements, force: [a, b] },
                3 => Laid::Body { elements, force: [a, b] },
                _ => return None,
            }
        })
    }
}

impl Structure {
    /// A row's constants (`Head`, then the structure): what `stated` is — `[J₀]`, or `[m, c]` for
    /// an output an unknown sets — every bar's `(a, b, E)`, every beam's `(a, b, E, G, ρ, z)`,
    /// every spread load's `(count, per_area, force, beams…)`, every plane element and laid load,
    /// every solid element and borne load, every degree of freedom's `(held, load, weight)`, then
    /// the drive's payload: where the sections are sized, every beam's four tapes, each
    /// `(length, flat…)`; where the nodes are moved, how (`Mover::consts`).
    pub fn consts(&self, stated: &[f64], drive: &Drive) -> Vec<f64> {
        let mut k = Vec::with_capacity(self.widths(stated.len() == 2, drive).1);
        self.write(stated, drive, &mut k);
        k
    }

    /// A row's widths, `(n_par, n_const)`: its columns, and the unknown's where it has one; and
    /// how many constants `consts` writes, counted by writing them.
    pub fn widths(&self, free: bool, drive: &Drive) -> (usize, usize) {
        let mut n = 0;
        self.write(&[0.0; 2][..1 + free as usize], drive, &mut n);
        (self.row_columns(drive) + free as usize, n)
    }

    /// How many columns a row over the structure, driven by `drive`, reads before the unknown
    /// that may set its output (`Head::columns`).
    pub fn row_columns(&self, drive: &Drive) -> usize {
        Head::of(self, drive).columns().expect("a structure's columns are counted")
    }

    /// The one walk of a row's constants, into `k`; `read` reads them in its order.
    fn write(&self, stated: &[f64], drive: &Drive, k: &mut impl Sink) {
        debug_assert!(match drive {
            Drive::Sized(z) => z.sections.len() == self.beams.len(),
            Drive::Moved(_) => self.bars.is_empty() && self.beams.is_empty(),
            Drive::Stated => true,
        });
        Head::of(self, drive).write(k);
        k.put(stated.iter().copied());
        for b in &self.bars {
            k.put([b.a as f64, b.b as f64, b.e]);
        }
        for b in &self.beams {
            k.put([b.a as f64, b.b as f64, b.e, b.g, b.rho]);
            k.put(b.z);
        }
        for s in &self.spread {
            k.put([s.beams.len() as f64, s.per_area as u8 as f64]);
            k.put(s.force);
            k.put(s.beams.iter().map(|&i| i as f64));
        }
        for t in &self.tris {
            k.put(t.n.map(|n| n as f64));
            k.put([t.e, t.nu, t.rho, t.depth, t.strain as u8 as f64]);
        }
        for l in &self.laid {
            l.write(k);
        }
        for t in &self.tets {
            k.put(t.n.map(|n| n as f64));
            k.put([t.e, t.nu, t.rho]);
        }
        for l in &self.borne {
            l.write(k);
        }
        for i in 0..self.fixed.len() {
            k.put([self.fixed[i] as u8 as f64, self.load[i], self.weight[i]]);
        }
        match drive {
            Drive::Stated => {}
            Drive::Sized(z) => {
                for t in z.sections.iter().flatten() {
                    k.put([t.len() as f64]);
                    k.put(t.iter().copied());
                }
            }
            Drive::Moved(m) => k.block(m.width(), || m.consts()),
        }
    }

    /// What `consts` wrote, `free` where an unknown sets the output (the row has its column, as
    /// `integral::length_widths` reads a length's): the structure, what it states and its drive
    /// — `None` for constants that are not whole (a document is untrusted input): a count out of
    /// range, a node, a beam or an element named past its count, a tape reading past its
    /// unknowns, a mover that does not fit the structure, or a constant left over.
    pub fn read(k: &[f64], free: bool) -> Option<(Structure, &[f64], Drive)> {
        let mut r = Take::new(k);
        let h = Head::read(&mut r)?;
        let stated = r.take(1 + free as usize)?;
        let node = |v: f64| count(v).filter(|&i| i < h.nodes);
        let bars = r.records::<3>(h.bars)?.iter()
            .map(|c| Some(Bar { a: node(c[0])?, b: node(c[1])?, e: c[2] }))
            .collect::<Option<_>>()?;
        let beams = r.records::<BEAM>(h.beams)?.iter()
            .map(|c| Some(Beam { a: node(c[0])?, b: node(c[1])?, e: c[2], g: c[3], rho: c[4],
                                 z: [c[5], c[6], c[7]] }))
            .collect::<Option<_>>()?;
        let spread = (0..h.spread).map(|_| {
            let c = r.records::<SPREAD>(1)?[0];
            let along: Vec<usize> = r.take(count(c[0])?)?.iter()
                .map(|&v| count(v).filter(|&i| i < h.beams)).collect::<Option<_>>()?;
            Some(Spread { beams: along, force: [c[2], c[3], c[4]], per_area: c[1] != 0.0 })
        }).collect::<Option<_>>()?;
        let tris = r.records::<TRI>(h.tris)?.iter().map(|c| {
            let n: Option<Vec<usize>> = c[..6].iter().map(|&v| node(v)).collect();
            Some(Tri6 { n: n?.try_into().ok()?, e: c[6], nu: c[7], rho: c[8], depth: c[9],
                        strain: c[10] != 0.0 })
        }).collect::<Option<_>>()?;
        let laid = (0..h.laid).map(|_| Laid::read(&mut r, h.nodes, h.tris))
            .collect::<Option<_>>()?;
        let tets = r.records::<TET>(h.tets)?.iter().map(|c| {
            let n: Option<Vec<usize>> = c[..10].iter().map(|&v| node(v)).collect();
            Some(Tet10 { n: n?.try_into().ok()?, e: c[10], nu: c[11], rho: c[12] })
        }).collect::<Option<_>>()?;
        let borne = (0..h.borne).map(|_| Borne::read(&mut r, h.nodes, h.tets))
            .collect::<Option<_>>()?;
        let mut s = Structure { dim: h.dim, nodes: h.nodes, bars, beams, fixed: Vec::new(),
                                load: Vec::new(), weight: Vec::new(), spread, tris, laid, tets,
                                borne, reads: h.reads };
        // a plane element is of the plane, a solid one of space, each moves without turning, and
        // neither is a member's nor the other's
        let mixed = !s.beams.is_empty() || !s.bars.is_empty();
        if !(s.dim == 2 || s.dim == 3) || (!s.tris.is_empty() && (s.dim != 2 || mixed))
            || (!s.tets.is_empty() && (s.dim != 3 || mixed || !s.tris.is_empty()))
        {
            return None;
        }
        let per = r.records::<3>(s.per().checked_mul(s.nodes)?)?;
        s.fixed = per.iter().map(|c| c[0] != 0.0).collect();
        s.load = per.iter().map(|c| c[1]).collect();
        s.weight = per.iter().map(|c| c[2]).collect();
        let drive = match h.drive {
            Tag::Stated => (h.over == 0).then_some(Drive::Stated)?,
            Tag::Sized => {
                let unknowns = Some(h.over).filter(|&n| n <= crate::tape::MAX_VARS)?;
                let sections = (0..h.beams).map(|_| {
                    let mut four: [Vec<f64>; SECTION] = Default::default();
                    for t in &mut four {
                        let n = r.count()?;
                        let flat = r.take(n)?;
                        if !crate::tape::reads_within(flat, unknowns) {
                            return None;
                        }
                        *t = flat.to_vec();
                    }
                    Some(four)
                }).collect::<Option<_>>()?;
                Drive::Sized(Sizing { unknowns, sections })
            }
            // a plane structure's nodes, read over the drawing: no areas and no sections
            _ if mixed => return None,
            Tag::Plane | Tag::Stacked => {
                let m = r.block(|k| Moved::read(k, h.drive == Tag::Stacked))?;
                let plane = m.nodes.len();
                let fits = m.x0.len() == 2 * plane && h.over == m.columns
                    && match &m.stack {
                        None => plane == s.nodes && s.dim == 2,
                        Some(st) => st.of.len() == s.nodes && s.dim == 3,
                    };
                Drive::Moved(Mover::Plane(fits.then_some(m)?))
            }
            // a filled body's nodes, moved by its sheets (#567): every node of the solid's
            Tag::Hull => {
                let hull = r.block(Hull::read)?;
                let n = hull.nodes.len();
                let fits = n == s.nodes && s.dim == 3 && hull.x0.len() == 3 * n
                    && hull.feet0.len() == 3 * n && h.over == hull.columns;
                Drive::Moved(Mover::Hull(fits.then_some(hull)?))
            }
        };
        r.done().then_some((s, stated, drive))
    }
}
