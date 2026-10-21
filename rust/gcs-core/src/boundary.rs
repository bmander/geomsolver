//! **What each component is, against the contract for a reusable solve boundary** (#452,
//! `docs/solve-boundaries-plan.md`).  Advisory only: it proves nothing about caching, and nothing
//! is refused on its word.  `solventc --components` prints it.
//!
//! A component is read through one instance of it in a solved document: what that instance made
//! (every entity and row whose site lies under its path) against what it was given (every other
//! column its rows read).  The instance's own unknowns are `y`, its inputs `x`, and its rows the
//! acting statements of its body, a nested instance's included.  Everything else in the document
//! is set aside: the parent's statements over the instance's outputs are the parent's (IO2), and
//! the inputs are held where they stand.
//!
//! * **Asserts only**: it declares nothing a parent reads (none, or only `private` geometry), and
//!   its rows, or its `fix`es, say something of its operands (`std.Spin`, `std.Turn`).  A candidate
//!   verb.
//! * **Leaves freedoms**: rank-deficient against its own unknowns at a generic input, the inputs
//!   moved from the instance's by a seeded draw (DT1 fails).  The freedoms are named: a parent
//!   would have to own them, and cannot name them.
//! * **Locally determined**: full rank there (DT1), and a second root found from scattered starts
//!   that every root choice it states admits: which it returns is the seeds' choice.
//! * **Branch chosen**: locally determined, and no second root the stated choices admit was found.
//!   A heuristic, and said to be one: a root not found is not shown absent (DT2).
//! * **Reads the world**, beside any of those: a row that reads its plane's own axes (`horizontal`,
//!   `level(up)`, `along: u`), a seed in coordinates or by a bearing, a coordinate read (`p.x`) or
//!   a `fix` at coordinates — what fails the spin check of EQ2, found from the text and the rows
//!   rather than by moving anything.
//!
//! What is not read is said so: a family of sets, a component only ever traced in a curve, one
//! nothing instantiates, and one whose instance owns no unknown and says nothing of its operands.
//!
//! The rank is judged as the diagnosis judges it: `System::conditioned` against `RANK_TOL`, at
//! the document's scale rather than the boundary's own (CL1 asks for the latter; #455 sets it).

use crate::constraints::{CKind, Constraint, OrdinateForm};
use crate::json::{self, Json};
use crate::model::{EntKind, EntRef, Sketch};
use crate::space;
use crate::program::{Elaborated, InstPath, Instanced, SourceMap};
use crate::solve::{solve, BlockMode, SolveOpts};
use crate::syntax::{Program, Stmt, StmtKind, Ty};
use crate::system::{System, RANK_TOL};
use std::collections::{BTreeMap, BTreeSet};

pub mod measure;

/// How many inputs moved from the instance's the rank is read at, beside the instance's own.
pub const PERTURBED: usize = 3;
/// How many scattered starts the second root is looked for from.
pub const STARTS: usize = 12;
/// How many iterations a probe's solve is given (`probe`).
pub const PROBE_ITERATIONS: i32 = 50;
/// Past this many unknowns of its own, no second root is looked for: too large to search here.
pub const SEARCHED: usize = 120;

/// What a component is, by the first of the contract's tests it fails or passes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Class {
    /// A family of sets (§6.21): an instance is the set, and makes nothing of its own to solve.
    Set,
    /// Nothing in the document instantiates it, so there is nothing to read it through.
    Unused,
    /// Instantiated only in place inside a curve: its body is a trace's, solved by the locus
    /// (`locus.rs`) at each place along the curve, and never drawn on the sheet to be read here.
    Traced,
    /// Its instance owns no unknown and states nothing of its operands: it builds faces or
    /// solids, or nothing at all.
    NothingToSolve,
    AssertsOnly,
    LeavesFreedoms,
    LocallyDetermined,
    BranchChosen,
}

impl Class {
    pub fn as_str(self) -> &'static str {
        match self {
            Class::Set => "a set",
            Class::Unused => "not instantiated",
            Class::Traced => "a curve's body",
            Class::NothingToSolve => "nothing to solve",
            Class::AssertsOnly => "asserts only",
            Class::LeavesFreedoms => "leaves freedoms",
            Class::LocallyDetermined => "locally determined",
            Class::BranchChosen => "branch chosen",
        }
    }
}

/// One component definition, read through the first of its instances.
#[derive(Clone, Debug)]
pub struct Classified {
    /// Which of the program's components (`Program::components`).
    pub comp: usize,
    /// Its identity across the program (`Program::component_id`): `Rung`, `std.Spin`.
    pub name: String,
    /// How many instances of it the document expanded.
    pub instances: usize,
    pub class: Class,
    /// What was found, where an instance was read.
    pub analysis: Option<Analysis>,
}

/// What one instance came to.
#[derive(Clone, Debug, Default)]
pub struct Analysis {
    /// The instance read, as its names start (`one`, `drive.upper`).
    pub instance: String,
    /// Its own unknowns: the free numbers of what it made.
    pub unknowns: usize,
    /// The rank of its rows against them, at the instance's own inputs.
    pub rank_here: usize,
    /// The most it came to there or at an input moved from them (`PERTURBED`): its generic rank.
    pub rank: usize,
    /// Whether it declares anything a parent may read (a member not `private`).
    pub public: bool,
    /// The rank its rows add over its operands beyond what its own unknowns take up: how many
    /// independent facts it states about what it was given (FR1).
    pub facts: usize,
    /// Its rows whose every column is an operand's, as the source says them.
    pub fact_rows: Vec<String>,
    /// The operands whose numbers its `fix`es hold.
    pub held: Vec<String>,
    /// What moves, where it leaves freedoms: the entities its rows' null space moves.
    pub freedoms: Vec<String>,
    /// The numeric formals the instance left unbound, which it reads as inputs: the freedoms it
    /// exports, the parent's unknowns (FR2).
    pub exported: Vec<String>,
    /// The `Int` formals among those: a count unbound, which no boundary may leave (CL3).
    pub counts: Vec<String>,
    /// The root search, where it ran.
    pub roots: Option<Roots>,
    /// How many root choices (`ccw`, `cw`) its body states.
    pub stated: usize,
    /// What it reads of the world, each in words, by how it reads it.
    pub world: Vec<(Reads, String)>,
}

impl Analysis {
    /// The class its findings put it in.
    pub fn class(&self) -> Class {
        let asserts = self.facts > 0 || !self.held.is_empty();
        if self.unknowns == 0 && !asserts {
            return Class::NothingToSolve;
        }
        if !self.public && asserts {
            return Class::AssertsOnly;
        }
        if self.rank < self.unknowns {
            return Class::LeavesFreedoms;
        }
        match self.roots {
            Some(r) if r.admitted > 1 => Class::LocallyDetermined,
            // too large to search: what was shown is DT1, and no more
            None if self.unknowns > SEARCHED => Class::LocallyDetermined,
            _ => Class::BranchChosen,
        }
    }
}

/// How a body reads the world (EQ2): what fails the check that spins a carrier's frame in its
/// plane, the operands held.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reads {
    /// A row over its plane's own axes: `horizontal`, `level(up)`, `along: u`.
    Axes,
    /// A `fix` holding a point or an axis of its own at coordinates.
    Held,
    /// A seed in coordinates, in a plane's frame or by a bearing from its x axis.
    Seed,
    /// A coordinate read: `p.x`.
    Coordinate,
}

impl Reads {
    pub fn as_str(self) -> &'static str {
        match self {
            Reads::Axes => "axes",
            Reads::Held => "held",
            Reads::Seed => "seed",
            Reads::Coordinate => "coordinate",
        }
    }
}

/// The root search of DT2: scattered starts at the instance's own inputs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Roots {
    pub starts: usize,
    /// How many of them solved.
    pub solved: usize,
    /// The distinct roots its stated choices admit, the instance's own among them.
    pub admitted: usize,
    /// The distinct roots its stated choices refuse.
    pub refused: usize,
    /// Whether it ran at inputs moved from the instance's, where the instance's own were singular.
    pub moved: bool,
}

/// Every component the document defines and every one of a module's it instantiates, in that
/// order, each read through its first instance in `solved` (the document's sketch after its solve).
pub fn classify(e: &Elaborated, solved: &Sketch) -> Vec<Classified> {
    let prog = &e.program;
    let map = &e.map;
    let mut defs: Vec<usize> = (0..prog.components.len())
        .filter(|&i| prog.components[i].name.is_some() && prog.components[i].module.is_none())
        .collect();
    let mut used: Vec<usize> = map.instances.iter().map(|i| i.comp)
        .filter(|&c| prog.components[c].module.is_some()).collect();
    used.sort_by_key(|&c| prog.component_id(c));
    used.dedup();
    defs.extend(used);
    let world = World::of(solved, map);
    defs.into_iter().map(|c| {
        let name = prog.component_id(c);
        let instances: Vec<&Instanced> = map.instances.iter().filter(|i| i.comp == c).collect();
        let mut out = Classified { comp: c, name, instances: instances.len(), class: Class::Unused,
                                   analysis: None };
        if prog.components[c].set.is_some() {
            out.class = Class::Set;
            return out;
        }
        let Some(first) = instances.iter().find(|i| i.drawn) else {
            if !instances.is_empty() {
                out.class = Class::Traced;
            }
            return out;
        };
        let a = analyse(prog, map, solved, &world, first);
        out.class = a.class();
        out.analysis = Some(a);
        out
    }).collect()
}

/// What every instance's reading shares: which parameters are whose, which are where a point is.
pub(crate) struct World {
    /// Each parameter an entity with a site owns, to that entity.
    pub(crate) owner: BTreeMap<u32, EntRef>,
    /// The parameters that are a place or a length: a point's coordinates, a radius, an axis's
    /// origin — moved by a length, where every other number is moved by its own size.
    lengths: BTreeSet<u32>,
    /// The free variables' parameters.
    free: BTreeSet<u32>,
    /// Every entity and every statement (by its index among the sketch's) by the instance path
    /// it is sited under, ascending, and the statements sited nowhere: what an instance made and
    /// states are a range of each (`under`), not a reading of the whole document per instance.
    ents: Vec<(InstPath, EntRef)>,
    cons: Vec<(InstPath, usize)>,
    unsited: Vec<usize>,
}

/// The entries of `list` (ascending by path) sited at `at` or under it: one range, since every
/// path that starts with `at` sorts after it and before any that does not.
fn under<'a, T: Copy>(list: &'a [(InstPath, T)], at: &'a InstPath)
    -> impl Iterator<Item = T> + 'a {
    let lo = list.partition_point(|(p, _)| p < at);
    list[lo..].iter().take_while(move |(p, _)| p.0.starts_with(&at.0)).map(|&(_, t)| t)
}

impl World {
    pub(crate) fn of(sk: &Sketch, map: &SourceMap) -> World {
        let mut owner = BTreeMap::new();
        for &e in map.of_entity.keys() {
            for p in sk.own_params(e) {
                owner.entry(p).or_insert(e);
            }
        }
        let mut lengths = BTreeSet::new();
        for i in 0..sk.points.len() {
            lengths.extend(sk.point_all_params(i));
        }
        lengths.extend(sk.circles.iter().map(|c| c.radius));
        lengths.extend(sk.arcs.iter().map(|a| a.radius));
        for a in &sk.axes {
            lengths.extend(a.a);
        }
        let free = sk.free_vars.values().copied().collect();
        let mut ents: Vec<(InstPath, EntRef)> =
            map.of_entity.iter().map(|(&e, s)| (s.path.clone(), e)).collect();
        ents.sort();
        let (mut cons, mut unsited) = (Vec::new(), Vec::new());
        for (k, c) in sk.constraints.iter().enumerate() {
            match map.site_of_constraint(c.id) {
                Some(s) => cons.push((s.path.clone(), k)),
                None => unsited.push(k),
            }
        }
        cons.sort();
        World { owner, lengths, free, ents, cons, unsited }
    }
}

/// Every column a row reads: its kernel's, and the free variable its number is written in.
pub(crate) fn columns(sk: &Sketch, c: &Constraint) -> Vec<u32> {
    let mut cols = c.params(sk);
    cols.extend(c.free.as_ref().map(|f| f.param));
    cols
}

/// What an instance made and what it states, set apart from the document: every entity sited
/// under it (and a twin of one), its own unknowns, and its rows.
pub(crate) struct Split {
    pub(crate) made: BTreeSet<EntRef>,
    /// The numbers of what it made, with the numbers no entity owns that its rows read.
    pub(crate) own: BTreeSet<u32>,
    /// Its rows, by constraint id.
    pub(crate) kept: BTreeSet<u32>,
}

pub(crate) fn split(map: &SourceMap, sk: &Sketch, world: &World, inst: &Instanced) -> Split {
    let inside = |id: u32| map.site_of_constraint(id).is_some_and(|s| inst.contains(&s.path));
    // what it made: every entity sited under it, and a twin of one
    let mut made: BTreeSet<EntRef> = under(&world.ents, &inst.path).collect();
    for (&twin, &of) in &map.twin_of {
        if made.contains(&of) {
            made.insert(twin);
        }
    }
    let mine: BTreeSet<u32> = made.iter().flat_map(|&e| sk.own_params(e)).collect();
    // its rows, the numbers no entity owns that they read (a contact's place, a lift) — its own
    // too — and the rows no statement owns that reach only those or what it made (a lift's, an
    // arc's radius, a twin's tie).  A lift of a point drawn in its plane that none of its rows
    // reads is the document's, though its row reads the plane.
    let zero = sk.zero;
    let nobodys = |p: u32| !world.owner.contains_key(&p) && !world.free.contains(&p);
    let unowned = |p: u32| nobodys(p) && Some(p) != zero && !sk.params[p as usize].fixed;
    let mut aux: BTreeSet<u32> = BTreeSet::new();
    let mut kept: BTreeSet<u32> = BTreeSet::new();
    // a statement sited elsewhere is never its own: the rest are those sited under it and those
    // sited nowhere, in the sketch's order (the rows kept grow to one fixed point in any order)
    let mut candidates: Vec<usize> =
        under(&world.cons, &inst.path).chain(world.unsited.iter().copied()).collect();
    candidates.sort_unstable();
    loop {
        let before = (kept.len(), aux.len());
        for c in candidates.iter().map(|&k| &sk.constraints[k]) {
            if kept.contains(&c.id) {
                continue;
            }
            let cols = columns(sk, c);
            // an arc's, an axis's or a plane's own rows (its ends at its radius, a unit
            // direction, a plane through its axes) are what it made, whatever else they read: an
            // arc about an operand's centre places its ends from it (#461)
            let own_row = c.intrinsic && c.entities().iter()
                .any(|e| e.kind != EntKind::Point && made.contains(e));
            let ours = inside(c.id) || map.site_of_constraint(c.id).is_none() && (
                own_row
                || cols.iter().any(|p| aux.contains(p))
                || cols.iter().any(|p| mine.contains(p))
                    && cols.iter().all(|&p| mine.contains(&p) || nobodys(p)));
            if !ours {
                continue;
            }
            kept.insert(c.id);
            aux.extend(cols.into_iter().filter(|&p| unowned(p)));
        }
        if (kept.len(), aux.len()) == before {
            break;
        }
    }
    let own: BTreeSet<u32> = mine.union(&aux).copied().collect();
    Split { made, own, kept }
}

/// The instance named `name` (as its members' names start: `upper`, `drive.upper`) as a boundary
/// a parent solves through (#454, `solve::solve_through`): its own unknowns what it made, and its
/// rows its statements, read as the classifier reads them.  No syntax marks it: the experiment
/// names it from Rust.  `None` where no instance drawn on the sheet has that name.
pub fn instance(e: &Elaborated, sk: &Sketch, name: &str) -> Option<crate::solve::Boundary> {
    instances(e, sk, &[name]).pop().flatten()
}

/// `instance` for each of `names`, in order, the document read once for all of them.
pub fn instances(e: &Elaborated, sk: &Sketch, names: &[&str])
    -> Vec<Option<crate::solve::Boundary>> {
    let world = World::of(sk, &e.map);
    let drawn: BTreeMap<&str, &Instanced> =
        e.map.instances.iter().rev().filter(|i| i.drawn).map(|i| (i.prefix.as_str(), i)).collect();
    names.iter().map(|&name| {
        let inst = drawn.get(format!("{name}.").as_str())?;
        let Split { own, kept, .. } = split(&e.map, sk, &world, inst);
        Some(crate::solve::Boundary { name: name.to_string(), own, rows: kept })
    }).collect()
}

/// One instance, read: its rows and unknowns set apart from the document, ranked at its inputs
/// and at inputs moved from them, its roots searched for, and its text read for the world.
fn analyse(
    prog: &Program,
    map: &SourceMap,
    sk: &Sketch,
    world: &World,
    inst: &Instanced,
) -> Analysis {
    let comp = &prog.components[inst.comp];
    let name_of = |e: EntRef| map.name_of(e).cloned();
    let ent_name = |e: EntRef| name_of(e).unwrap_or_else(|| crate::model::entity_name(e));
    let inside = |id: u32| map.site_of_constraint(id).is_some_and(|s| inst.contains(&s.path));
    let zero = sk.zero;
    let Split { made, own, kept } = split(map, sk, world, inst);
    let rows: Vec<&Constraint> = sk.constraints.iter().filter(|c| kept.contains(&c.id)).collect();
    let inputs: BTreeSet<u32> = rows.iter().flat_map(|c| columns(sk, c))
        .filter(|p| !own.contains(p) && Some(*p) != zero).collect();
    let describe = |c: &Constraint| crate::io::describe_with(c, &name_of);
    let mut a = Analysis {
        instance: inst.prefix.trim_end_matches('.').to_string(),
        ..Analysis::default()
    };
    // read off the definition, not the names: an instance under an anonymous key (a face
    // family's station) shows none of its members' names, private or not
    a.public = !made.is_empty() && declares_public(&comp.body);

    // its unknowns against its rows, the inputs held; and both free, for what it says of them
    let mut alone = reduced(sk, &kept, |p, fixed| fixed || !own.contains(&p));
    // an operand the document holds is still one a fact can be about: freed, as every input is
    let both = reduced(sk, &kept, |p, fixed| !inputs.contains(&p) && (fixed || !own.contains(&p)));
    let (n, rank_here, null_here) = rank_of(&alone);
    a.unknowns = n;
    a.rank_here = rank_here;
    a.facts = rank_of(&both).1.saturating_sub(rank_here);
    a.fact_rows = rows.iter()
        .filter(|c| inside(c.id) && c.acts())
        .filter(|c| columns(sk, c).iter().all(|p| inputs.contains(p)))
        .map(|c| describe(c)).collect();

    // the instance's size: how far its points stand from their middle
    let size = size_of(sk, &own, &inputs);
    let base = alone.get_x();
    let mut rng = crate::rng::Rng::new(0x452);
    a.rank = rank_here;
    let mut null = null_here;
    // where the most rank was read: the instance's inputs, or the moved ones that gave more
    let mut generic = base.clone();
    for _ in 0..PERTURBED {
        alone.set_x(&base);
        nudge(&mut alone, inputs.iter().copied(), world, &mut rng, 0.05 * size, 0.05);
        solve(&mut alone, probe());
        let (_, r, nl) = rank_of(&alone);
        if r > a.rank {
            a.rank = r;
            null = nl;
            generic = alone.get_x();
        }
    }
    alone.set_x(&generic);
    if a.rank < a.unknowns {
        a.freedoms = moving(&alone, &null, world, &ent_name);
    }

    // the numbers it left unbound, read as inputs; a count among them is no input at all
    for f in comp.formals.iter().filter(|f| f.ty.number()) {
        let key = format!("{}{}", inst.prefix, f.name.text);
        let Some(&p) = sk.free_vars.get(&key) else { continue };
        if !inputs.contains(&p) && !own.contains(&p) {
            continue;
        }
        match f.ty {
            Ty::Int => a.counts.push(key),
            _ => a.exported.push(key),
        }
    }

    // what its `fix`es held: an operand's numbers are a fact about it, and its own place held at
    // coordinates is the world's
    let mut held: Vec<String> = Vec::new();
    let mut placed: Vec<String> = Vec::new();
    for (p, path) in &map.holds {
        if !inst.contains(path) {
            continue;
        }
        let Some(&e) = world.owner.get(p) else { continue };
        let what = format!("`{}`", ent_name(e));
        if !made.contains(&e) {
            if !held.contains(&what) {
                held.push(what);
            }
        } else if matches!(e.kind, EntKind::Point | EntKind::Axis) && !placed.contains(&what) {
            placed.push(what);
        }
    }
    a.held = held;

    // the root choices it states, and the roots they admit: read here of each root a probe finds,
    // since a probe is no document solve and is steered by none of them (`solve::steer`, #157).
    // Its orientations, and its bounds (§9.6), which `alone` keeps among its rows
    let choices: Vec<(&String, [usize; 3], i32)> = map.chosen.iter()
        .filter(|(_, path)| inst.contains(path))
        .filter_map(|(k, _)| {
            Some((k, crate::decompose::branch_key_points(k)?, *sk.orientations.get(k)?))
        })
        .collect();
    a.stated = choices.len() + rows.iter().filter(|c| c.bound.is_some()).count();
    if a.rank == a.unknowns && a.unknowns > 0 && a.unknowns <= SEARCHED {
        let admits = |s: &Sketch| crate::solve::bounds_hold(s)
            && choices.iter().all(|&(_, [p, q, r], v)| {
                crate::model::orientation(s, p, q, r) * v as f64 > 0.0
            });
        let mut roots = search(&mut alone, &own, &made, world, size, &admits);
        roots.moved = generic != base;
        a.roots = Some(roots);
    }
    alone.set_x(&base);

    // what it reads of the world
    for c in &rows {
        if !inside(c.id) || c.intrinsic {
            continue;
        }
        if let Some(what) = reads_axes(c) {
            a.world.push((Reads::Axes, format!("`{}` reads {what}", describe(c))));
        }
    }
    for what in placed {
        a.world.push((Reads::Held, format!("`fix` holds {what} at coordinates")));
    }
    // the bodies it expanded: its own, then each nested definition's once
    let mut bodies = vec![inst.comp];
    for i in &map.instances {
        if i.drawn && i.path.0.len() > inst.path.0.len() && inst.contains(&i.path)
            && !bodies.contains(&i.comp) {
            bodies.push(i.comp);
        }
    }
    for &b in &bodies {
        let mut said = Vec::new();
        written(prog, &prog.components[b].body, &mut said);
        let within = (b != inst.comp).then(|| prog.component_id(b));
        for (k, s) in said {
            let s = match &within {
                Some(w) => (k, format!("{s} (in `{w}`)")),
                None => (k, s),
            };
            if !a.world.contains(&s) {
                a.world.push(s);
            }
        }
    }
    a
}

/// How each probe is solved: one DogLeg of `PROBE_ITERATIONS`, no retry and no block pass.  A
/// probe is a question, and a start that does not settle soon answers it; the block pass's own
/// root search would carry a start back to the root it already has.
fn probe() -> SolveOpts {
    SolveOpts {
        max_iter: PROBE_ITERATIONS,
        retry: false,
        blocks: BlockMode::Off,
        ..SolveOpts::default()
    }
}

/// The document with only the rows `kept`, each parameter held as `fixed` says from its index
/// and whether the document held it.
pub(crate) fn reduced(sk: &Sketch, kept: &BTreeSet<u32>, fixed: impl Fn(u32, bool) -> bool)
    -> Sketch {
    let mut out = sk.clone();
    out.constraints.retain(|c| kept.contains(&c.id));
    out.solid_claims.clear();
    for (i, p) in out.params.iter_mut().enumerate() {
        p.fixed = fixed(i as u32, p.fixed);
    }
    out
}

/// Move each of `params` by a seeded draw: a place or a length by `length` world units, any other
/// number by `relative` of its own size (one, where it is smaller).
fn nudge(
    sk: &mut Sketch,
    params: impl Iterator<Item = u32>,
    world: &World,
    rng: &mut crate::rng::Rng,
    length: f64,
    relative: f64,
) {
    for p in params {
        let q = &mut sk.params[p as usize];
        let step = if world.lengths.contains(&p) {
            rng.normal(0.0, length) / q.scale
        } else {
            rng.normal(0.0, relative) * q.value.abs().max(1.0)
        };
        q.value += step;
    }
    sk.settle_turns();
}

/// The free columns, the rank of the hard rows over them and their null space, at the pose.
fn rank_of(sk: &Sketch) -> (usize, usize, Option<(Vec<i32>, crate::linalg::Mat)>) {
    let mut sys = System::new(sk);
    let n = sys.n_free;
    if n == 0 {
        return (0, 0, None);
    }
    if sys.hard_rows().is_empty() {
        return (n, 0, Some((sys.free.clone(), crate::linalg::Mat::identity(n))));
    }
    let z = sys.z0(sk);
    let rn = sys.conditioned(&z).rank_and_nullspace(RANK_TOL);
    (n, rn.rank.min(n), Some((sys.free.clone(), rn.null())))
}

/// The entities a null space moves, by their names, most moved first; a number no entity owns by
/// its own name.
fn moving(
    sk: &Sketch,
    null: &Option<(Vec<i32>, crate::linalg::Mat)>,
    world: &World,
    name: &dyn Fn(EntRef) -> String,
) -> Vec<String> {
    let Some((free, m)) = null else { return Vec::new() };
    let mut weight: BTreeMap<String, f64> = BTreeMap::new();
    for (c, &p) in free.iter().enumerate() {
        let w: f64 = (0..m.cols).map(|k| m.at(c, k).powi(2)).sum();
        if w <= 1e-6 {
            continue;
        }
        let who = match world.owner.get(&(p as u32)) {
            Some(&e) => name(e),
            None => sk.params[p as usize].name.clone(),
        };
        let at = weight.entry(who).or_insert(0.0);
        *at = at.max(w);
    }
    let mut out: Vec<(String, f64)> = weight.into_iter().collect();
    out.sort_by(|a, b| b.1.total_cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
    out.into_iter().map(|(n, _)| n).collect()
}

/// How far the instance's points stand from their middle: the size its starts and its moved
/// inputs are drawn at.  The drawing's extent where it has no two points apart.
fn size_of(sk: &Sketch, own: &BTreeSet<u32>, inputs: &BTreeSet<u32>) -> f64 {
    let pts: Vec<[f64; 3]> = (0..sk.points.len())
        .filter(|&i| sk.point_all_params(i).iter().any(|p| own.contains(p) || inputs.contains(p)))
        .map(|i| sk.world_point(i))
        .collect();
    let n = pts.len().max(1) as f64;
    let mid = space::scale(pts.iter().fold([0.0; 3], |a, &w| space::add(a, w)), 1.0 / n);
    let size = pts.iter().map(|&w| space::distance(w, mid)).fold(0.0, f64::max);
    if size > 0.0 { size } else { sk.extent().max(1.0) }
}

/// The figure an instance made, as numbers two roots are compared by: each point where it stands
/// in space, each radius's size and each axis's direction and place.  Not its parameters: an
/// angle a turn on, or a frame's attitude written the other way, is the same figure.
fn figure(sk: &Sketch, made: &BTreeSet<EntRef>) -> Vec<f64> {
    let mut out = Vec::new();
    for e in made {
        match e.kind {
            EntKind::Point => out.extend(sk.world_point(e.i())),
            EntKind::Circle => out.push(sk.params[sk.circles[e.i()].radius as usize].value.abs()),
            EntKind::Arc => out.push(sk.params[sk.arcs[e.i()].radius as usize].value.abs()),
            EntKind::Axis => {
                let r = &sk.axes[e.i()];
                let d = r.d.map(|p| sk.params[p as usize].value);
                out.extend(space::normalised(d).unwrap_or(d));
                out.extend(r.a.map(|p| sk.params[p as usize].value));
            }
            _ => {}
        }
    }
    out
}

/// DT2 by refutation: `STARTS` starts scattered about the instance's solution at twice its size,
/// each solved with the inputs held, and the distinct roots counted, the instance's own first.
/// Two roots are distinct where their figures are (`figure`).
fn search(
    sk: &mut Sketch,
    own: &BTreeSet<u32>,
    made: &BTreeSet<EntRef>,
    world: &World,
    size: f64,
    admits: &dyn Fn(&Sketch) -> bool,
) -> Roots {
    let base = sk.get_x();
    let free: Vec<u32> = sk.free_indices().into_iter().map(|p| p as u32)
        .filter(|p| own.contains(p)).collect();
    let mut roots: Vec<(Vec<f64>, bool)> = vec![(figure(sk, made), admits(sk))];
    let mut out = Roots { starts: STARTS, ..Roots::default() };
    for k in 0..STARTS {
        sk.set_x(&base);
        let mut rng = crate::rng::Rng::new(0x4520_0000 ^ (k as u32).wrapping_mul(2_654_435_761));
        nudge(sk, free.iter().copied(), world, &mut rng, 2.0 * size, 1.0);
        let r = solve(sk, probe());
        if !r.settled() {
            continue;
        }
        out.solved += 1;
        let y = figure(sk, made);
        // lengths at the instance's size, a direction's parts as if it were a length
        let tol = 1e-4 * size.max(1.0);
        let apart = |r: &Vec<f64>| r.iter().zip(&y).any(|(a, b)| (a - b).abs() > tol);
        if roots.iter().all(|(r, _)| apart(r)) {
            roots.push((y, admits(sk)));
        }
    }
    sk.set_x(&base);
    out.admitted = roots.iter().filter(|r| r.1).count();
    out.refused = roots.len() - out.admitted;
    out
}

/// Whether a row reads the axes of the plane it is drawn in, and which: a line held level in it,
/// or an ordinate along the plane's own `u` or `v`.
fn reads_axes(c: &Constraint) -> Option<&'static str> {
    match c.kind {
        CKind::Horizontal | CKind::Vertical => Some("its plane's axes"),
        CKind::Level | CKind::Ordinate => match c.form()? {
            OrdinateForm::PageU | OrdinateForm::PageV | OrdinateForm::CoordU
            | OrdinateForm::CoordV => Some("its plane's axes"),
            OrdinateForm::FrameU | OrdinateForm::FrameV => Some("a plane's own axes"),
            _ => None,
        },
        _ => None,
    }
}

/// Whether a body declares anything a parent may read: a declaration or a named instance not
/// `private`, in it or in a block of it.
fn declares_public(body: &[Stmt]) -> bool {
    body.iter().any(|st| match &st.kind {
        StmtKind::Decl(d) => !d.annotations.private && !d.name.key().text.starts_with('#'),
        StmtKind::Instance(i) => !i.annotations.private && !i.name.text.starts_with('#'),
        StmtKind::Block(b) => declares_public(&b.body),
        _ => false,
    })
}

/// What a body's text reads of the world, each in words: seeds in coordinates or by a bearing,
/// and coordinates read (`c.x`).  Nested blocks are walked; a nested instance's body is its own
/// component's, walked by the caller.
fn written(prog: &Program, body: &[Stmt], out: &mut Vec<(Reads, String)>) {
    for st in body {
        match &st.kind {
            StmtKind::Decl(d) => {
                let n = &d.name.key().text;
                let who = match n.starts_with('#') {
                    true => format!("a {}", d.kind.as_str()),
                    false => format!("`{n}`"),
                };
                let hinted = d.hint_span.is_some_and(|s| !s.is_empty());
                let seed = match &d.seed_at {
                    Some(at) if at.bearing.is_some() => {
                        Some("by a bearing from its plane's x axis".to_string())
                    }
                    Some(at) if at.x.is_some() => Some(format!(
                        "at coordinates in `{}`'s frame", crate::syntax::ref_text(&at.what))),
                    None if hinted && d.seed_spans.iter().any(|s| !s.is_empty()) => {
                        Some("at coordinates in its plane".to_string())
                    }
                    _ => None,
                };
                out.extend(seed.map(|how| (Reads::Seed, format!("seeds {who} {how}"))));
            }
            StmtKind::Block(b) => written(prog, &b.body, out),
            _ => {}
        }
        if let Some(text) = prog.span_text(st.span) {
            for read in coordinate_reads(text) {
                let s = (Reads::Coordinate, format!("reads a coordinate, `{read}`"));
                if !out.contains(&s) {
                    out.push(s);
                }
            }
        }
    }
}

/// Each `name.x`, `name.y` and `name.z` a statement's text reads, comments left out.
fn coordinate_reads(text: &str) -> Vec<String> {
    let ident = |c: char| c.is_alphanumeric() || c == '_';
    let mut out: Vec<String> = Vec::new();
    for line in text.lines() {
        let line = line.split("//").next().unwrap_or("");
        let cs: Vec<char> = line.chars().collect();
        for i in 1..cs.len() {
            let field = matches!(cs.get(i + 1), Some('x' | 'y' | 'z'))
                && !cs.get(i + 2).is_some_and(|&c| ident(c));
            if cs[i] != '.' || !ident(cs[i - 1]) || !field {
                continue;
            }
            let mut lo = i;
            while lo > 0 && (ident(cs[lo - 1]) || cs[lo - 1] == '.') {
                lo -= 1;
            }
            let read: String = cs[lo..i + 2].iter().collect();
            // a number's decimal point is no field: `1.x` is not a read
            if read.starts_with(|c: char| c.is_ascii_digit()) || out.contains(&read) {
                continue;
            }
            out.push(read);
        }
    }
    out
}

/// One component as `solventc --components` says it: its class on the first line, and what
/// was found below it, indented.
pub fn text(c: &Classified) -> Vec<String> {
    let instances = match (c.instances, &c.analysis) {
        (0, _) => String::new(),
        (n, Some(a)) => format!(" ({}, `{}` read)", count(n, "instance"), a.instance),
        (n, None) => format!(" ({})", count(n, "instance")),
    };
    let class = match c.class {
        Class::LocallyDetermined => "locally determined, branch not chosen",
        Class::BranchChosen => "locally determined, branch chosen (heuristic)",
        k => k.as_str(),
    };
    let mut out = vec![format!("component `{}`{instances}: {class}", c.name)];
    let Some(a) = &c.analysis else { return out };
    if a.unknowns > 0 {
        let unknowns = count(a.unknowns, "unknown");
        let at = if a.rank == a.rank_here {
            format!("rank {} at its inputs and at {PERTURBED} moved from them", a.rank)
        } else {
            format!("rank {} at its inputs, {} at one moved from them", a.rank_here, a.rank)
        };
        out.push(format!("  {unknowns} of its own, {at}"));
    } else {
        out.push("  no unknown of its own".to_string());
    }
    let quoted = |xs: &[String]| xs.iter().map(|x| format!("`{x}`")).collect::<Vec<_>>();
    if !a.freedoms.is_empty() {
        out.push(format!("  leaves {} free, moving {}", a.unknowns - a.rank,
                         list(&quoted(&a.freedoms), 6)));
    }
    if a.facts > 0 || !a.held.is_empty() {
        let mut says = Vec::new();
        if a.facts > 0 {
            says.push(format!("states {} about its operands", count(a.facts, "fact")));
        }
        if !a.held.is_empty() {
            says.push(format!("holds numbers of {}", a.held.join(", ")));
        }
        out.push(format!("  {}", says.join("; ")));
        for r in &a.fact_rows {
            out.push(format!("    `{r}` reads only its operands"));
        }
    }
    if !a.public && a.unknowns > 0 {
        out.push("  declares nothing a parent reads: every member is `private`".to_string());
    }
    if !a.exported.is_empty() {
        out.push(format!("  exports {}: a formal left unbound, the parent's unknown",
                         list(&quoted(&a.exported), 6)));
    }
    if !a.counts.is_empty() {
        out.push(format!("  not a boundary: a count is unbound ({})",
                         quoted(&a.counts).join(", ")));
    }
    if let Some(r) = a.roots {
        let at = if r.moved { " at inputs moved from its own" } else { "" };
        let mut line = format!("  {} from {} of {} scattered starts{at}",
                               count(r.admitted, "root"), r.solved, r.starts);
        if a.stated > 0 {
            line += &format!(", by its {}", count(a.stated, "stated root choice"));
            if r.refused > 0 {
                line += &format!(" (which refuse {}; the solve holds the drawing to them)",
                                 count(r.refused, "root"));
            }
        }
        line += match r.admitted {
            0 | 1 => ": a heuristic, which finds no second root and proves none absent",
            _ => ": which it returns is the seeds' choice",
        };
        out.push(line);
    } else if a.rank == a.unknowns && a.unknowns > SEARCHED {
        out.push(format!("  no second root looked for: more than {SEARCHED} unknowns"));
    }
    if !a.world.is_empty() {
        out.push(format!("  reads the world: {}", world_text(&a.world)));
    }
    out
}

/// What a body reads of the world, said by kind: how many of each, and the first few.
fn world_text(world: &[(Reads, String)]) -> String {
    let mut said = Vec::new();
    for k in [Reads::Axes, Reads::Held, Reads::Seed, Reads::Coordinate] {
        let all: Vec<String> = world.iter().filter(|w| w.0 == k).map(|w| w.1.clone()).collect();
        if all.is_empty() {
            continue;
        }
        let n = all.len();
        said.push(match k {
            Reads::Axes if n == 1 => all[0].clone(),
            Reads::Axes => format!("{n} rows read a plane's axes ({})", list(&all, 2)),
            Reads::Seed if n > 2 => format!("{n} seeds in coordinates or by a bearing ({})",
                                            list(&all, 2)),
            Reads::Coordinate if n > 2 => format!("{n} coordinates read ({})", list(&all, 2)),
            _ => all.join("; "),
        });
    }
    said.join("; ")
}

/// `1 root`, `2 roots`.
pub(crate) fn count(n: usize, what: &str) -> String {
    match n {
        1 => format!("1 {what}"),
        _ => format!("{n} {what}s"),
    }
}

/// The first `most`, and how many more.
pub(crate) fn list(xs: &[String], most: usize) -> String {
    let shown = xs.iter().take(most).cloned().collect::<Vec<_>>().join(", ");
    match xs.len().saturating_sub(most) {
        0 => shown,
        k => format!("{shown} and {k} more"),
    }
}

/// One component as `--json` reports it.
pub fn json(c: &Classified) -> Json {
    let strs = |v: &[String]| Json::Arr(v.iter().map(|s| Json::Str(s.clone())).collect());
    let analysis = match &c.analysis {
        None => Json::Null,
        Some(a) => json::object([
            ("instance", Json::Str(a.instance.clone())),
            ("unknowns", Json::Int(a.unknowns as i64)),
            ("rank_here", Json::Int(a.rank_here as i64)),
            ("rank", Json::Int(a.rank as i64)),
            ("public", Json::Bool(a.public)),
            ("facts", Json::Int(a.facts as i64)),
            ("fact_rows", strs(&a.fact_rows)),
            ("held", strs(&a.held)),
            ("freedoms", strs(&a.freedoms)),
            ("exported", strs(&a.exported)),
            ("counts", strs(&a.counts)),
            ("stated", Json::Int(a.stated as i64)),
            ("roots", match a.roots {
                None => Json::Null,
                Some(r) => json::object([
                    ("starts", Json::Int(r.starts as i64)),
                    ("solved", Json::Int(r.solved as i64)),
                    ("admitted", Json::Int(r.admitted as i64)),
                    ("refused", Json::Int(r.refused as i64)),
                    ("moved", Json::Bool(r.moved)),
                ]),
            }),
            ("world", Json::Arr(a.world.iter().map(|(k, w)| json::object([
                ("reads", Json::Str(k.as_str().to_string())),
                ("what", Json::Str(w.clone())),
            ])).collect())),
        ]),
    };
    json::object([
        ("name", Json::Str(c.name.clone())),
        ("instances", Json::Int(c.instances as i64)),
        ("class", Json::Str(c.class.as_str().to_string())),
        ("reads_world", Json::Bool(c.analysis.as_ref().is_some_and(|a| !a.world.is_empty()))),
        ("analysis", analysis),
    ])
}
