//! **A scenario read over the drawing's beams** (#510 S0): a frame's answers, as claims.
//!
//! The drawing is the mesh.  Every line a `beam(…)` names is a member, an Euler–Bernoulli beam
//! rigid at its ends (`interior::Beam`), and members join where their ends meet or where an end
//! lies on another member: a **joint**, found by place (`JOINED` of the frame's extent).  A point
//! the scenario names that lies on a member — a load, a support, a deflection asked for — cuts it
//! there into a joint of its own.  Between joints a member is cut into `PIECES` cubic elements:
//! under loads at its joints one would be exact, but a mode, a buckling load and a load spread
//! along it are the cubic's to approximate, to the fourth power of the piece.  Nothing else is
//! meshed, and the frame stands where the solved drawing puts it.
//!
//! A **frame** is the members joined to one another; a requirement reads the frame of what it
//! names — a beam, a member's line, a point at a joint.  The scenario's supports and loads are
//! read where they fall on that frame's joints and members.  What it says is the requirement's
//! value, or why there is none: an analysis cannot be read where the supports leave the frame
//! free (`Structure::free`, said in the solver's words), where a material does not state what is
//! read of it, or where an output is asked of what it is not read of yet.  The value is in the
//! document's units, the material read back into them (`Units::from_si`).
//!
//! **A driving deflection is a row** (#510 S1, `Row`): `deflection(tip, along: down) == 2mm`
//! compiles to the interior's kernel (`interior::kernel`) over the frame its point is a joint of,
//! meshed with one element between joints (exact for the joints' moves under loads at joints
//! and spread evenly, which is all a static deflection reads), its nodes' columns the lifts of
//! the drawing's points there and its sections read over the unknowns they state (`Sizing`).
//! Every other requirement is judged after the solve, never solved for.

use crate::diagnose::analysis::direction;
use crate::eigen::{self, Pairs};
use crate::graph::UnionFind;
use crate::interior::{self, Beam as Element, Failure, Sizing, Spread, State, Structure};
use crate::model::{
    Applied, EntKind, EntRef, Hold, LoadKind, LoadValue, Output, Property, Requirement, Scenario,
    SectionProp, Sketch,
};
use crate::solid::Box3;
use crate::space::{distance, dot, lerp, norm, normalised, scale, sub};
use crate::sparse::Sym;
use crate::vocab::all_of;
use std::cell::OnceCell;
use std::collections::BTreeMap;

type V = [f64; 3];

/// How many cubic elements a member is cut into between two joints.
pub const PIECES: usize = 8;
/// Two places within this of the frame's extent are one joint; a point within it of a member's
/// line, between its ends, lies on the member.  Far below any member's length, far above what
/// "solved" leaves between two points held coincident.
pub const JOINED: f64 = 1e-6;

/// **What a scenario's frames say**: words, each a sentence — a frame the supports leave free —
/// and per requirement its value, or why it has none; `None` for one about no frame (a solid's,
/// which nothing analyses yet).  Per support, whether it holds a frame (names a joint of one), so
/// the body a solid's supports hold is not read with it.
#[derive(Clone, Debug, Default)]
pub struct Read {
    pub words: Vec<String>,
    pub answers: Vec<Option<Result<f64, String>>>,
    pub holds_frame: Vec<bool>,
}

impl Read {
    /// Whether it read a frame at all: a support holding one, or a requirement about one.
    pub fn reads_a_frame(&self) -> bool {
        self.holds_frame.iter().any(|&h| h) || self.answers.iter().any(Option::is_some)
    }
}

/// A member: a line of a beam, between two places, its section's z the normal of the plane the
/// line is drawn in.
struct Member {
    beam: usize,
    line: usize,
    ends: [V; 2],
    z: V,
}

/// The drawing's beams as joints and elements: every node — the joints first, then each
/// member's pieces' inner ends — the elements between them by member, and the frame (connected
/// part) each node is in.
struct Mesh {
    nodes: Vec<V>,
    joints: usize,
    /// Each joint's point of the drawing: the first member's end or named point found there.
    points: Vec<usize>,
    /// Each element: its member, and its ends (nodes).  A member's elements are consecutive.
    elements: Vec<(usize, [usize; 2])>,
    members: Vec<Member>,
    frame: Vec<usize>,
    tol: f64,
}

impl Mesh {
    /// The drawing's beams meshed, cut at the `named` points that lie on a member and into
    /// `pieces` elements between joints.
    fn new(sk: &Sketch, named: &[usize], pieces: usize) -> Result<Mesh, String> {
        let mut members = Vec::new();
        let mut bounds = Box3::empty();
        let mut ends: Vec<(V, usize)> = Vec::new();
        for (b, beam) in sk.beams.iter().enumerate() {
            for &l in &beam.lines {
                let at = sk.line_ends(l);
                let places = at.map(|p| sk.world_point(p));
                places.iter().for_each(|&p| bounds.add(p));
                ends.extend([(places[0], at[0]), (places[1], at[1])]);
                members.push(Member { beam: b, line: l, ends: places, z: plane_normal(sk, l) });
            }
        }
        let extent = (0..3).map(|k| bounds.hi[k] - bounds.lo[k]).fold(0.0, f64::max);
        let tol = JOINED * extent.max(f64::MIN_POSITIVE);
        let mut mesh = Mesh { nodes: Vec::new(), joints: 0, points: Vec::new(),
                              elements: Vec::new(), members, frame: Vec::new(), tol };
        // the joints: every member's ends, then every place an end or a named point cuts one
        for &(p, point) in &ends {
            mesh.joint(p, point);
        }
        let named: Vec<(V, usize)> = named.iter().map(|&i| (sk.world_point(i), i)).collect();
        let mut cuts: Vec<Vec<(f64, usize)>> = Vec::new();
        for m in 0..mesh.members.len() {
            let [p, q] = mesh.members[m].ends;
            let d = sub(q, p);
            let l = norm(d);
            if !(l > tol) {
                let b = &sk.beams[mesh.members[m].beam];
                return Err(format!("a line of `{}` has no length, so it is no member", b.name));
            }
            let [pa, pb] = sk.line_ends(mesh.members[m].line);
            let mut at: Vec<(f64, usize)> = vec![(0.0, mesh.joint(p, pa)), (1.0, mesh.joint(q, pb))];
            for &(c, point) in ends.iter().chain(&named) {
                let t = dot(sub(c, p), d) / (l * l);
                if t * l > tol && (1.0 - t) * l > tol && distance(c, lerp(p, q, t)) <= tol {
                    at.push((t, mesh.joint(c, point)));
                }
            }
            at.sort_by(|a, b| a.0.total_cmp(&b.0));
            at.dedup_by(|a, b| a.1 == b.1);
            cuts.push(at);
        }
        for (m, at) in cuts.iter().enumerate() {
            let [p, q] = mesh.members[m].ends;
            for w in at.windows(2) {
                let ((t0, a), (t1, b)) = (w[0], w[1]);
                let mut prev = a;
                for k in 1..=pieces {
                    let next = if k == pieces {
                        b
                    } else {
                        mesh.nodes.push(lerp(p, q, t0 + (t1 - t0) * k as f64 / pieces as f64));
                        mesh.nodes.len() - 1
                    };
                    mesh.elements.push((m, [prev, next]));
                    prev = next;
                }
            }
        }
        // the frames: the nodes the elements join
        let mut uf = UnionFind::new(mesh.nodes.len());
        for &(_, [a, b]) in &mesh.elements {
            uf.union(a, b);
        }
        mesh.frame = uf.labels().0;
        Ok(mesh)
    }

    /// The joint of the drawing's point `point`, at `p`: the one it already is, else the one
    /// at its place, made where there is none.
    fn joint(&mut self, p: V, point: usize) -> usize {
        if let Some(j) = self.points.iter().position(|&q| q == point) {
            return j;
        }
        self.at(p).unwrap_or_else(|| {
            self.nodes.push(p);
            self.points.push(point);
            self.joints += 1;
            self.joints - 1
        })
    }

    /// The joint at `p`, if there is one.
    fn at(&self, p: V) -> Option<usize> {
        (0..self.joints).find(|&i| distance(self.nodes[i], p) <= self.tol)
    }

    /// The joint the drawing's point `i` is: the one it was made at, else the one at its place.
    fn at_point(&self, sk: &Sketch, i: usize) -> Option<usize> {
        self.points.iter().position(|&q| q == i).or_else(|| self.at(sk.world_point(i)))
    }

    /// The members `a` names: a beam's every line's, or a line's.
    fn members_of(&self, a: &Applied) -> Vec<usize> {
        let named = |m: &Member| match a {
            Applied::Beam(b) => m.beam == *b,
            Applied::Ent(e) if e.kind == EntKind::Line => m.line == e.i(),
            _ => false,
        };
        (0..self.members.len()).filter(|&m| named(&self.members[m])).collect()
    }

    /// The nodes `a` names on the frame: a point's joint, or every node along the members a
    /// line or a beam names.
    fn nodes_of(&self, sk: &Sketch, a: &Applied) -> Vec<usize> {
        if let Applied::Ent(e) = a {
            if e.kind == EntKind::Point {
                return self.at_point(sk, e.i()).into_iter().collect();
            }
        }
        let ms = self.members_of(a);
        let mut out: Vec<usize> = self.elements.iter().filter(|(m, _)| ms.contains(m))
            .flat_map(|&(_, ends)| ends).collect();
        out.sort();
        out.dedup();
        out
    }
}

/// The normal of the plane line `l` is drawn in: its section's z.
fn plane_normal(sk: &Sketch, l: usize) -> V {
    let [p, _] = sk.line_ends(l);
    match sk.plane_of(p) {
        Some(pl) => normalised(sk.basis(pl).normal()).unwrap_or([0.0, 0.0, 1.0]),
        None => [0.0, 0.0, 1.0],
    }
}

/// The elements (by index) whose member is one of `ms`, `member` each element's.
fn elements_of(member: &[usize], ms: &[usize]) -> Vec<usize> {
    (0..member.len()).filter(|&e| ms.contains(&member[e])).collect()
}

/// One frame of the mesh under a scenario, as a structure: its columns (its sections the
/// drawing's), its elements' members, where each node of the mesh is in it, and the beams it is
/// made of, named.  Its loads are the scenario's at its joints, and spread along its members
/// (`interior::Spread`).
struct Framed {
    s: Structure,
    x: Vec<f64>,
    member: Vec<usize>,
    /// Each node of the mesh, its own number in the frame, where it is in it.
    local: Vec<Option<usize>>,
    names: String,
}

/// One frame of the scenario, analysed: the frame, its loads spread along each element (in the
/// world's axes, a force a length), the solved displacements, and what the requirements read of
/// it, worked out once.
struct Analysed {
    s: Structure,
    x: Vec<f64>,
    st: State,
    spread: Vec<V>,
    member: Vec<usize>,
    local: Vec<Option<usize>>,
    /// How many modes the requirements on the frame ask for, at most.
    modes: usize,
    stiffness: OnceCell<Result<Sym, String>>,
    pairs: OnceCell<Result<Pairs, String>>,
    buckled: OnceCell<Result<f64, String>>,
    actions: OnceCell<Vec<Option<[f64; 12]>>>,
}

/// Why a frame cannot be read: the supports leave it free (the beams it is made of, named, and
/// what they are free to do, in the solver's words), or anything else, said.
enum Unread {
    Free { names: String, free: String },
    Why(String),
}

impl From<String> for Unread {
    fn from(m: String) -> Unread {
        Unread::Why(m)
    }
}

impl Unread {
    fn text(&self) -> String {
        match self {
            Unread::Free { names, free } => format!("{names} is {free}"),
            Unread::Why(m) => m.clone(),
        }
    }
}

/// A material's property, read back into the document's units, or why there is none.
fn property(sk: &Sketch, beam: usize, p: Property) -> Result<f64, String> {
    let b = &sk.beams[beam];
    let name = b.material.as_ref().ok_or_else(|| {
        format!("`{}` is made of no material it can be read by: say what, `beam(…) of steel`",
                b.name)
    })?;
    let m = &sk.materials[name];
    let si = m.get(p).ok_or_else(|| {
        format!("`{}` is made of `{name}`, which states no `{}`", b.name, p.name())
    })?;
    Ok(sk.units.from_si(si, p.dim()))
}

impl Framed {
    /// The frame `frame` of `mesh` under scenario `sc`, or why it cannot be read.
    fn new(sk: &Sketch, sc: &Scenario, mesh: &Mesh, frame: usize) -> Result<Framed, Unread> {
        let mut local = vec![None; mesh.nodes.len()];
        let mut n = 0;
        for i in (0..mesh.nodes.len()).filter(|&i| mesh.frame[i] == frame) {
            local[i] = Some(n);
            n += 1;
        }
        let els: Vec<usize> =
            (0..mesh.elements.len()).filter(|&e| local[mesh.elements[e].1[0]].is_some()).collect();
        let names = frame_names(sk, mesh, &els);
        // every beam's moduli and section, read once
        let mut moduli: BTreeMap<usize, (f64, f64, f64, [f64; 4])> = BTreeMap::new();
        for &e in &els {
            let beam = mesh.members[mesh.elements[e].0].beam;
            if let std::collections::btree_map::Entry::Vacant(v) = moduli.entry(beam) {
                let young = property(sk, beam, Property::E)?;
                let nu = property(sk, beam, Property::Nu).map_err(|m| {
                    format!("{m}: a frame twists, and its shear modulus is E / 2(1 + nu)")
                })?;
                let rho = property(sk, beam, Property::Rho).unwrap_or(f64::NAN);
                let section = sk.beams[beam].section.stiffness(sk)?;
                v.insert((young, young / (2.0 * (1.0 + nu)), rho, section));
            }
        }
        let mut x = vec![0.0; 3 * n];
        for (i, k) in local.iter().enumerate() {
            if let Some(k) = *k {
                x[3 * k..3 * k + 3].copy_from_slice(&mesh.nodes[i]);
            }
        }
        let (mut elements, mut member) = (Vec::new(), Vec::new());
        for &e in &els {
            let (m, [a, b]) = mesh.elements[e];
            let (young, g, rho, section) = moduli[&mesh.members[m].beam];
            let (a, b) = (local[a].unwrap(), local[b].unwrap());
            elements.push(Element { a, b, e: young, g, rho, z: mesh.members[m].z });
            member.push(m);
            x.extend(section);
        }
        let mut s = Structure { dim: 3, nodes: n, bars: Vec::new(), beams: elements,
                                fixed: Vec::new(), load: Vec::new(), weight: Vec::new(),
                                spread: Vec::new() };
        let dofs = s.per() * n;
        (s.fixed, s.load, s.weight) = (vec![false; dofs], vec![0.0; dofs], vec![0.0; dofs]);
        let on = |a: &Applied| -> Vec<usize> {
            mesh.nodes_of(sk, a).into_iter().filter_map(|i| local[i]).collect()
        };
        // the supports that fall on this frame
        for sup in &sc.supports {
            let at = on(&sup.at);
            if at.is_empty() {
                continue;
            }
            let moves: Vec<usize> = match sup.along {
                None => vec![0, 1, 2],
                Some(d) => {
                    let d = direction(sk, d)?;
                    match (0..3).find(|&k| (d[k].abs() - 1.0).abs() <= 1e-9) {
                        Some(k) => vec![k],
                        None => return Err(Unread::Why(format!(
                            "a support holds {names} along a direction off the world's axes, \
                             which a frame is not read under yet"))),
                    }
                }
            };
            let turns: &[usize] = if sup.hold == Hold::Fixed { &[3, 4, 5] } else { &[] };
            for node in at {
                for &c in moves.iter().chain(turns) {
                    let dof = s.dof(node, c).unwrap();
                    s.fixed[dof] = true;
                }
            }
        }
        // the loads: a force at a joint, or spread along what it names
        for load in &sc.loads {
            let v = match &load.value {
                LoadValue::Vector(v) => *v,
                LoadValue::Along { magnitude, along } => scale(direction(sk, *along)?, *magnitude),
                LoadValue::Scalar(_) => continue, // a pressure, on a face
            };
            let at_point = matches!(load.at, Applied::Ent(e) if e.kind == EntKind::Point);
            match load.kind {
                LoadKind::Force if at_point => {
                    let Some(&node) = on(&load.at).first() else { continue };
                    for c in 0..3 {
                        let dof = s.dof(node, c).unwrap();
                        s.load[dof] += v[c];
                    }
                }
                LoadKind::Force => {
                    let along = elements_of(&member, &mesh.members_of(&load.at));
                    if !along.is_empty() {
                        s.spread.push(Spread { beams: along, force: v, per_area: false });
                    }
                }
                // a body load: the density times the acceleration, a force a volume, spread
                // along each beam named by its own density
                LoadKind::Acceleration => {
                    let along = elements_of(&member, &mesh.members_of(&load.at));
                    let mut by_beam: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
                    for e in along {
                        by_beam.entry(mesh.members[member[e]].beam).or_default().push(e);
                    }
                    for (beam, along) in by_beam {
                        let rho = property(sk, beam, Property::Rho)?;
                        s.spread.push(Spread { beams: along, force: scale(v, rho),
                                               per_area: true });
                    }
                }
                LoadKind::Pressure => {}
            }
        }
        Ok(Framed { s, x, member, local, names })
    }
}

impl Analysed {
    /// The frame `frame` of `mesh` under scenario `sc`, solved, or why it cannot be; `modes` the
    /// most modes a requirement asks of it.
    fn new(sk: &Sketch, sc: &Scenario, mesh: &Mesh, frame: usize, modes: usize)
        -> Result<Analysed, Unread>
    {
        let Framed { s, x, member, local, names } = Framed::new(sk, sc, mesh, frame)?;
        let st = s.solve(&x).map_err(|why| unsolved(&s, &x, &names, why))?;
        let spread = s.spread_at(&x);
        Ok(Analysed { s, x, st, spread, member, local, modes, stiffness: OnceCell::new(),
                      pairs: OnceCell::new(), buckled: OnceCell::new(), actions: OnceCell::new() })
    }

    /// The displacement of node `i` (the mesh's numbering).
    fn moved(&self, i: usize) -> V {
        let k = self.local[i].unwrap();
        [0, 1, 2].map(|c| self.st.u[self.s.dof(k, c).unwrap()])
    }

    /// Element `e`'s density, or why its material states none.
    fn density(&self, sk: &Sketch, mesh: &Mesh, e: usize) -> Result<f64, String> {
        property(sk, mesh.members[self.member[e]].beam, Property::Rho)
    }

    /// `K`, assembled once.
    fn stiffness(&self) -> Result<&Sym, String> {
        self.stiffness.get_or_init(|| self.s.stiffness(&self.x).map_err(|_| "no stiffness".into()))
            .as_ref().map_err(Clone::clone)
    }

    /// The modes the requirements ask for, solved once for the most of them.
    fn pairs(&self, sk: &Sketch, mesh: &Mesh) -> Result<&Pairs, String> {
        self.pairs.get_or_init(|| {
            for e in 0..self.s.beams.len() {
                self.density(sk, mesh, e)?;
            }
            let m = self.s.mass(&self.x).map_err(|_| "no mass".to_string())?;
            eigen::lowest(self.stiffness()?, &m, self.modes, &eigen::Opts::default())
                .map_err(|why| format!("its modes are not found: {why:?}"))
        }).as_ref().map_err(Clone::clone)
    }

    /// Every element's end actions, worked out once.
    fn actions(&self) -> &[Option<[f64; 12]>] {
        self.actions.get_or_init(|| self.s.end_actions(&self.st.u, &self.x))
    }

    /// Element `e`'s end actions on it at its near end, in its own axes, the load spread along
    /// it taken out: the forces and moments its near node holds it by; and that load in its axes,
    /// and its length.
    fn acting(&self, e: usize) -> ([f64; 12], [f64; 3], f64) {
        let (h, axes) = self.s.beam_axes(e, &self.x).unwrap();
        let q = interior::into_axes(&axes, self.spread[e]);
        let fe = fixed_end(q, h);
        let mut r = self.actions()[e].unwrap();
        for k in 0..12 {
            r[k] -= fe[k];
        }
        (r, q, h)
    }

    /// The least factor on the loads the frame buckles at, solved once.
    fn buckled(&self) -> Result<f64, String> {
        self.buckled.get_or_init(|| {
            let pulls: Vec<f64> = (0..self.s.beams.len()).map(|e| {
                let (r, q, h) = self.acting(e);
                -(r[0] + q[0] * h / 2.0)
            }).collect();
            let kg = self.s.geometric(&self.x, &pulls).map_err(|_| "no stiffness".to_string())?;
            let pairs = eigen::buckling(self.stiffness()?, &kg, 1, &eigen::Opts::default())
                .map_err(|why| format!("its buckling is not found: {why:?}"))?;
            pairs.values.first().copied()
                .ok_or_else(|| "the loads press no member, so nothing buckles".to_string())
        }).clone()
    }
}

/// Why a frame's structure gave no displacement, in the solver's words.
fn unsolved(s: &Structure, x: &[f64], names: &str, why: Failure) -> Unread {
    match why {
        Failure::Free => {
            let free = s.free(x).text(3).unwrap_or_else(|| "free".to_string());
            Unread::Free { names: names.to_string(), free }
        }
        Failure::Collapsed => Unread::Why(format!("a member of {names} has no length")),
        Failure::Unstiff => Unread::Why(format!("a member of {names} has no stiffness")),
    }
}

/* -- a driving row (#510 S1) ------------------------------------------------------------------ */

/// **What a driving deflection's row reads** (#510 S1): the frame its point is a joint of under
/// the scenario, one element between joints — so every node is a joint, and each is a point of
/// the drawing whose lift is its three columns (`points`, a node each) — with the deflection's
/// weights on the point's moves along its direction, and its sections read over the unknowns
/// they state (`unknowns`, by name, the row's columns after the nodes').
#[derive(Clone, Debug, PartialEq)]
pub struct Row {
    pub structure: Structure,
    pub sizing: Sizing,
    pub points: Vec<usize>,
    pub unknowns: Vec<String>,
    /// Where the structure stands: the nodes' places and the sections, at the drawing.
    pub x: Vec<f64>,
}

impl Row {
    /// **The row of `deflection(at, along: along)` under scenario `sc`**, or why the solve
    /// cannot read it: `at` names no joint of a frame, a direction the solve moves (the weights
    /// and the loads are read as constants of the row), more unknowns than a tape reads, or a
    /// frame that cannot be read at all.
    pub fn of(sk: &Sketch, sc: &Scenario, at: usize, along: EntRef) -> Result<Row, String> {
        for d in [Some(along)].into_iter().chain(sc.loads.iter().map(|l| match l.value {
            LoadValue::Along { along, .. } => Some(along),
            _ => None,
        })).flatten() {
            if !held_direction(sk, d) {
                return Err("its direction moves in the solve, and a driving row reads every \
                            direction as it stands: name an axis the drawing holds, or a line \
                            whose ends it holds".to_string());
            }
        }
        let mut named: Vec<usize> = sc.applied().filter_map(|a| match a {
            Applied::Ent(e) if e.kind == EntKind::Point => Some(e.i()),
            _ => None,
        }).collect();
        named.push(at);
        let mesh = Mesh::new(sk, &named, 1)?;
        let joint = mesh.at_point(sk, at).ok_or_else(|| {
            "it is of a point that is no joint of a frame: a driving deflection is read at a \
             member's end, or a point on one".to_string()
        })?;
        let framed = Framed::new(sk, sc, &mesh, mesh.frame[joint]).map_err(|u| u.text())?;
        let Framed { mut s, x, member, local, .. } = framed;
        let d = direction(sk, along)?;
        let node = local[joint].unwrap();
        for c in 0..3 {
            let dof = s.dof(node, c).unwrap();
            s.weight[dof] = d[c];
        }
        let mut points = vec![0; s.nodes];
        for (i, k) in local.iter().enumerate() {
            if let Some(k) = *k {
                points[k] = mesh.points[i];
            }
        }
        // the unknowns every beam of the frame reads, each once
        let mut unknowns: Vec<String> = Vec::new();
        for &m in &member {
            for n in sk.beams[mesh.members[m].beam].section.unknowns() {
                if !unknowns.contains(&n) {
                    unknowns.push(n);
                }
            }
        }
        if unknowns.len() > crate::tape::MAX_VARS {
            return Err(format!("its frame's sections read {} unknowns, and a row reads at most \
                                {}", unknowns.len(), crate::tape::MAX_VARS));
        }
        let mut sections = Vec::with_capacity(member.len());
        for (e, &m) in member.iter().enumerate() {
            let section = &sk.beams[mesh.members[m].beam].section;
            let mut four: [Vec<f64>; interior::SECTION] = Default::default();
            for ((t, p), v) in four.iter_mut().zip(SectionProp::STIFFNESS).zip(s.section(e, &x)) {
                *t = match section.over.get(&p) {
                    Some(over) => over.tape(&unknowns, sk.units)?.flat,
                    None => crate::tape::Tape::constant(*v).flat,
                };
            }
            sections.push(four);
        }
        let sizing = Sizing { unknowns: unknowns.len(), sections };
        Ok(Row { structure: s, sizing, points, unknowns, x })
    }

    /// Whether the structure gives a displacement where the drawing stands, or why not — what
    /// the row is refused for where it is stated (a frame the supports leave free).
    pub fn check(&self) -> Result<(), String> {
        self.structure.solve(&self.x).map(|_| ())
            .map_err(|why| unsolved(&self.structure, &self.x, "the frame", why).text())
    }

    /// Its constants, stating the deflection `d`.
    pub fn consts(&self, d: f64) -> Vec<f64> {
        self.structure.consts_sized(&[d], Some(&self.sizing))
    }

    /// Its widths, `(n_par, n_const)`.
    pub fn widths(&self) -> (usize, usize) {
        self.structure.widths_sized(false, Some(&self.sizing))
    }
}

/// **Whether what a requirement names is of a frame** (#510 S1): a joint of one, or a beam or
/// a member's line — what a driving row may be compiled over, where the drawing stands.
pub fn of_frame(sk: &Sketch, sc: &Scenario, of: &Applied) -> bool {
    let named: Vec<usize> = sc.applied().chain([of]).filter_map(|a| match a {
        Applied::Ent(e) if e.kind == EntKind::Point => Some(e.i()),
        _ => None,
    }).collect();
    !sk.beams.is_empty()
        && Mesh::new(sk, &named, 1).is_ok_and(|mesh| !mesh.nodes_of(sk, of).is_empty())
}

/// Whether direction `d` is one the solve cannot move: an axis whose direction is held, or a line
/// whose ends are held in a plane that is.
fn held_direction(sk: &Sketch, d: EntRef) -> bool {
    let held = |ps: &[u32]| ps.iter().all(|&p| sk.params[p as usize].fixed);
    match d.kind {
        EntKind::Axis => held(&sk.axes[d.i()].d),
        EntKind::Line => {
            let [a, _] = sk.line_ends(d.i());
            held(&sk.line_params(d.i())) && sk.plane_of(a).is_none_or(|v| sk.plane_fixed(v))
        }
        _ => false,
    }
}

/// The consistent load of `q` (a force a length, in the element's axes) along an element of
/// length `h`: half at each end, and the cubic's end moments `q h²/12` in each plane it bends in.
fn fixed_end(q: V, h: f64) -> [f64; 12] {
    let (m2, m3) = (q[2] * h * h / 12.0, q[1] * h * h / 12.0);
    [
        q[0] * h / 2.0, q[1] * h / 2.0, q[2] * h / 2.0, 0.0, -m2, m3,
        q[0] * h / 2.0, q[1] * h / 2.0, q[2] * h / 2.0, 0.0, m2, -m3,
    ]
}

/// The beams a frame's elements are of, named: `` `arm` `` or `` `arm` and `post` ``.
fn frame_names(sk: &Sketch, mesh: &Mesh, els: &[usize]) -> String {
    let mut beams: Vec<usize> = els.iter().map(|&e| mesh.members[mesh.elements[e].0].beam)
        .collect();
    beams.sort();
    beams.dedup();
    let names: Vec<String> = beams.iter().map(|&b| format!("`{}`", sk.beams[b].name)).collect();
    all_of(&names)
}

/// The frame a requirement reads, as a node of it: a beam's, a member's line's, a joint's.
/// `None` for one about no frame; `Err` for a beam whose members are not joined.
fn frame_of(sk: &Sketch, mesh: &Mesh, of: &Applied) -> Option<Result<usize, String>> {
    let nodes = mesh.nodes_of(sk, of);
    let first = *nodes.first()?;
    let frame = mesh.frame[first];
    if nodes.iter().any(|&n| mesh.frame[n] != frame) {
        let what = match of {
            Applied::Beam(b) => format!("`{}`", sk.beams[*b].name),
            _ => "what it names".to_string(),
        };
        return Some(Err(format!("{what} is more than one frame: its members are not joined")));
    }
    Some(Ok(frame))
}

/// **A scenario read over the drawing's beams** at the solved drawing: per requirement its
/// value or why none, the words its frames say, and which supports hold a frame.  Each frame is
/// analysed once, and its modes solved once for the most any requirement asks for.
pub fn read(sk: &Sketch, sc: &Scenario) -> Read {
    let mut out = Read {
        words: Vec::new(),
        answers: vec![None; sc.requirements.len()],
        holds_frame: vec![false; sc.supports.len()],
    };
    if sk.beams.is_empty() {
        return out;
    }
    let named: Vec<usize> = sc.applied().filter_map(|a| match a {
        Applied::Ent(e) if e.kind == EntKind::Point => Some(e.i()),
        _ => None,
    }).collect();
    let mesh = match Mesh::new(sk, &named, PIECES) {
        Ok(m) => m,
        Err(m) => {
            out.words.push(format!("scenario `{}`: {m}", sc.name));
            return out;
        }
    };
    for (k, s) in sc.supports.iter().enumerate() {
        out.holds_frame[k] = !mesh.nodes_of(sk, &s.at).is_empty();
    }
    let of: Vec<_> = sc.requirements.iter().map(|r| frame_of(sk, &mesh, &r.of)).collect();
    let mut modes: BTreeMap<usize, usize> = BTreeMap::new();
    for (r, f) in sc.requirements.iter().zip(&of) {
        if let (Output::Mode, Some(Ok(f))) = (r.output, f) {
            let n = modes.entry(*f).or_default();
            *n = (*n).max(r.count.unwrap_or(1) as usize);
        }
    }
    let mut frames: BTreeMap<usize, Result<Analysed, Unread>> = BTreeMap::new();
    for (k, (r, f)) in sc.requirements.iter().zip(of).enumerate() {
        out.answers[k] = match f {
            None => continue,
            Some(Err(m)) => Some(Err(m)),
            Some(Ok(f)) => {
                let wanted = modes.get(&f).copied().unwrap_or(0);
                match frames.entry(f).or_insert_with(|| Analysed::new(sk, sc, &mesh, f, wanted)) {
                    Ok(a) => Some(answer(sk, &mesh, a, r)),
                    Err(m) => Some(Err(m.text())),
                }
            }
        };
    }
    // a frame the supports leave free is said once, in the solver's words
    for a in frames.values() {
        if let Err(Unread::Free { names, free }) = a {
            out.words.push(format!("scenario `{}` leaves {names} {free}", sc.name));
        }
    }
    out
}

/// A requirement's value over its frame `a`, or why it has none.
fn answer(sk: &Sketch, mesh: &Mesh, a: &Analysed, r: &Requirement) -> Result<f64, String> {
    match r.output {
        Output::Deflection => {
            let along = r.along.map(|d| direction(sk, d)).transpose()?;
            let mut worst: Option<f64> = None;
            for i in mesh.nodes_of(sk, &r.of) {
                let u = a.moved(i);
                let v = match along {
                    Some(d) => dot(u, d),
                    None => norm(u),
                };
                if worst.is_none_or(|w| v.abs() > w.abs()) {
                    worst = Some(v);
                }
            }
            worst.ok_or_else(|| "it names no node of a frame".to_string())
        }
        Output::Compliance => Ok(dot_all(&a.s.forces(&a.x), &a.st.u)),
        Output::Mass => {
            let ms = mesh.members_of(&r.of);
            if ms.is_empty() {
                return Err("a mass is of a beam or a member, and this names a point".into());
            }
            let mut m = 0.0;
            for e in elements_of(&a.member, &ms) {
                let length = a.s.beam_axes(e, &a.x).map_or(0.0, |x| x.0);
                m += a.density(sk, mesh, e)? * a.s.section(e, &a.x)[0] * length;
            }
            Ok(m)
        }
        Output::Mode => {
            let n = r.count.unwrap_or(1) as usize;
            let v = *a.pairs(sk, mesh)?.values.get(n - 1).ok_or("it has fewer modes than that")?;
            Ok(v.max(0.0).sqrt() / (2.0 * std::f64::consts::PI))
        }
        Output::Buckling => a.buckled(),
        Output::Safety => {
            let ms = mesh.members_of(&r.of);
            if ms.is_empty() {
                return Err("a factor of safety is of a beam or a member".into());
            }
            let mut worst = 0.0f64;
            let mut yields = f64::INFINITY;
            for e in elements_of(&a.member, &ms) {
                let beam = mesh.members[a.member[e]].beam;
                let b = &sk.beams[beam];
                let (sy, sz) = (b.section.read(SectionProp::Sy, sk).transpose()?,
                                b.section.read(SectionProp::Sz, sk).transpose()?);
                let (Some(sy), Some(sz)) = (sy, sz) else {
                    let states = if sy.is_some() || sz.is_some() { "one" } else { "neither" };
                    return Err(format!("a factor of safety reads `{}`'s section moduli, `Sy` \
                                        and `Sz`, and it states {states}", b.name));
                };
                yields = yields.min(property(sk, beam, Property::Yield)?);
                let (r, q, h) = a.acting(e);
                worst = worst.max(stress(&r, q, h, a.s.section(e, &a.x)[0], sy, sz));
            }
            if !(worst > 0.0) {
                let what = match &r.of {
                    Applied::Beam(b) => format!("`{}`", sk.beams[*b].name),
                    _ => "it".to_string(),
                };
                return Err(format!("the loads stress {what} nowhere"));
            }
            Ok(yields / worst)
        }
    }
}

/// `Σ aᵢbᵢ`.
fn dot_all(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// **The greatest normal stress along an element**, its near end's actions `r` (its own axes,
/// the spread load taken out), `q` spread along it, its length `h`: the stretch and the two
/// bendings at the section's farthest fibres, `|N|/A + |My|/Sy + |Mz|/Sz` — each a quadratic
/// along it (`N = −(r₀ + q₀s)`, `My = −r₄ − s r₂ − s²q₂/2`, `Mz = −r₅ + s r₁ + s²q₁/2`), so the
/// greatest of the sum of their sizes is the greatest of the eight sums with signs, each at an
/// end or its vertex.  Euler–Bernoulli's: no shear, and the twist's is not read.
fn stress(r: &[f64; 12], q: V, h: f64, a: f64, sy: f64, sz: f64) -> f64 {
    // each as c0 + c1 s + c2 s²
    let n = [-r[0] / a, -q[0] / a, 0.0];
    let my = [-r[4] / sy, -r[2] / sy, -q[2] / (2.0 * sy)];
    let mz = [-r[5] / sz, r[1] / sz, q[1] / (2.0 * sz)];
    let mut worst = 0.0f64;
    for signs in 0..8 {
        let sgn = |k: u32| if signs >> k & 1 == 0 { 1.0 } else { -1.0 };
        let c: [f64; 3] = [0, 1, 2].map(|i| sgn(0) * n[i] + sgn(1) * my[i] + sgn(2) * mz[i]);
        let at = |s: f64| c[0] + c[1] * s + c[2] * s * s;
        let mut best = at(0.0).max(at(h));
        if c[2] != 0.0 {
            let s = -c[1] / (2.0 * c[2]);
            if s > 0.0 && s < h {
                best = best.max(at(s));
            }
        }
        worst = worst.max(best);
    }
    worst
}

/// **A requirement's value as a reader reads it**, in the document's units: a deflection in its
/// length (`1.6mm`), a frequency in hertz, a mass in kilograms, a compliance in joules, a factor
/// as a plain number — six figures.
pub fn value_text(sk: &Sketch, output: Output, v: f64) -> String {
    let fig = |v: f64| crate::json::fmt_g(v, 6);
    match output {
        Output::Deflection => format!("{}{}", fig(v), sk.units.name()),
        Output::Mode => format!("{}Hz", fig(v)),
        Output::Mass => format!("{}kg", fig(v)),
        Output::Compliance => format!("{}J", fig(sk.units.to_si(v, output.dim()))),
        Output::Safety | Output::Buckling => fig(v),
    }
}

/// Whether value `v` keeps requirement `r`: within a hair of rounding (1e-9 of what it states)
/// of an equality, and on its side of an inequality.
pub fn keeps(r: &Requirement, v: f64) -> bool {
    let slack = 1e-9 * r.value.abs().max(v.abs());
    match r.compare {
        crate::expr::Cmp::Eq => (v - r.value).abs() <= slack,
        crate::expr::Cmp::Le | crate::expr::Cmp::Lt => v <= r.value + slack,
        crate::expr::Cmp::Ge | crate::expr::Cmp::Gt => v >= r.value - slack,
        _ => false,
    }
}
