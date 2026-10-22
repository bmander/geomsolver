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
//! document's units, the material read back into them (`Units::from_si`).  Nothing here is a row:
//! a requirement is judged after the solve, never solved for (S1 drives).

use crate::eigen;
use crate::interior::{Beam as Element, Failure, State, Structure};
use crate::model::{
    Applied, EntKind, EntRef, Hold, LoadKind, LoadValue, Output, Property, Requirement, Scenario,
    SectionProp, Sketch,
};
use crate::space::{add, dot, norm, normalised, scale, sub};
use crate::vocab::all_of;
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

/// A member: a line of a beam, between two places.
struct Member {
    beam: usize,
    line: usize,
    ends: [V; 2],
}

/// The drawing's beams as joints and elements: every node, the elements between them, the
/// member each element is of, and the frame (connected part) each node is in.
struct Mesh {
    nodes: Vec<V>,
    /// How many of `nodes` are joints: the rest are a member's pieces' inner ends.
    joints: usize,
    /// Each element: its member, its ends (nodes) and its section's z.
    elements: Vec<(usize, [usize; 2], V)>,
    members: Vec<Member>,
    /// Each member's nodes along it, its ends and every joint between.
    along: Vec<Vec<usize>>,
    frame: Vec<usize>,
    tol: f64,
}

impl Mesh {
    fn new(sk: &Sketch, named: &[V]) -> Result<Mesh, String> {
        let mut members = Vec::new();
        for (b, beam) in sk.beams.iter().enumerate() {
            for &l in &beam.lines {
                let ends = sk.line_ends(l).map(|p| sk.world_point(p));
                members.push(Member { beam: b, line: l, ends });
            }
        }
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for p in members.iter().flat_map(|m| m.ends) {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let extent = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
        let tol = JOINED * extent.max(f64::MIN_POSITIVE);
        let mut mesh = Mesh { nodes: Vec::new(), joints: 0, elements: Vec::new(), members,
                              along: Vec::new(), frame: Vec::new(), tol };
        // the joints: every member's ends, then every place an end or a named point cuts one
        let ends: Vec<V> = mesh.members.iter().flat_map(|m| m.ends).collect();
        for &p in &ends {
            mesh.joint(p);
        }
        let mut cuts: Vec<Vec<(f64, usize)>> = Vec::new();
        for m in 0..mesh.members.len() {
            let [p, q] = mesh.members[m].ends;
            let d = sub(q, p);
            let l = norm(d);
            if !(l > tol) {
                let b = &sk.beams[mesh.members[m].beam];
                return Err(format!("a line of `{}` has no length, so it is no member", b.name));
            }
            let mut at: Vec<(f64, usize)> = vec![(0.0, mesh.joint(p)), (1.0, mesh.joint(q))];
            for &c in ends.iter().chain(named) {
                let t = dot(sub(c, p), d) / (l * l);
                let off = norm(sub(c, add(p, scale(d, t))));
                if t * l > tol && (1.0 - t) * l > tol && off <= tol {
                    at.push((t, mesh.joint(c)));
                }
            }
            at.sort_by(|a, b| a.0.total_cmp(&b.0));
            at.dedup_by(|a, b| a.1 == b.1);
            cuts.push(at);
        }
        mesh.joints = mesh.nodes.len();
        for (m, at) in cuts.iter().enumerate() {
            let [p, q] = mesh.members[m].ends;
            let line = mesh.members[m].line;
            let z = plane_normal(sk, line);
            let mut along = Vec::new();
            for w in at.windows(2) {
                let ((t0, a), (t1, b)) = (w[0], w[1]);
                let mut prev = a;
                along.push(a);
                for k in 1..=PIECES {
                    let next = if k == PIECES {
                        b
                    } else {
                        let t = t0 + (t1 - t0) * k as f64 / PIECES as f64;
                        mesh.nodes.push(add(p, scale(sub(q, p), t)));
                        mesh.nodes.len() - 1
                    };
                    mesh.elements.push((m, [prev, next], z));
                    prev = next;
                }
            }
            along.push(at.last().unwrap().1);
            mesh.along.push(along);
        }
        // the frames: the nodes the elements join
        let mut parent: Vec<usize> = (0..mesh.nodes.len()).collect();
        fn root(p: &mut [usize], mut i: usize) -> usize {
            while p[i] != i {
                p[i] = p[p[i]];
                i = p[i];
            }
            i
        }
        for &(_, [a, b], _) in &mesh.elements {
            let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
            if ra != rb {
                parent[ra.max(rb)] = ra.min(rb);
            }
        }
        mesh.frame = (0..mesh.nodes.len()).map(|i| root(&mut parent, i)).collect();
        Ok(mesh)
    }

    /// The joint at `p`, made where there is none.
    fn joint(&mut self, p: V) -> usize {
        match self.at(p) {
            Some(i) => i,
            None => {
                self.nodes.push(p);
                self.nodes.len() - 1
            }
        }
    }

    /// The joint at `p`, if there is one.
    fn at(&self, p: V) -> Option<usize> {
        let n = if self.joints > 0 { self.joints } else { self.nodes.len() };
        (0..n).find(|&i| norm(sub(self.nodes[i], p)) <= self.tol)
    }

    /// The members drawn on line `l`.
    fn members_on(&self, l: usize) -> Vec<usize> {
        (0..self.members.len()).filter(|&m| self.members[m].line == l).collect()
    }

    /// The members `a` names: a beam's every line's, or a line's.
    fn members_of(&self, a: &Applied) -> Vec<usize> {
        match a {
            Applied::Beam(b) => {
                (0..self.members.len()).filter(|&m| self.members[m].beam == *b).collect()
            }
            Applied::Ent(e) if e.kind == EntKind::Line => self.members_on(e.i()),
            _ => Vec::new(),
        }
    }

    /// The nodes `a` names on the frame: a point's joint, or every node along the members a
    /// line or a beam names.
    fn nodes_of(&self, sk: &Sketch, a: &Applied) -> Vec<usize> {
        if let Applied::Ent(e) = a {
            if e.kind == EntKind::Point {
                return self.at(sk.world_point(e.i())).into_iter().collect();
            }
        }
        let mut out: Vec<usize> = Vec::new();
        for m in self.members_of(a) {
            for &(em, [x, y], _) in &self.elements {
                if em == m {
                    out.extend([x, y]);
                }
            }
        }
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

/// The unit direction an `along:` names: an axis's, or a line's from its first end.
fn direction(sk: &Sketch, d: EntRef) -> Option<V> {
    let v = match d.kind {
        EntKind::Axis => sk.axes[d.i()].d.map(|p| sk.params[p as usize].value),
        EntKind::Line => {
            let [a, b] = sk.line_ends(d.i());
            sub(sk.world_point(b), sk.world_point(a))
        }
        _ => return None,
    };
    normalised(v)
}

/// The places a scenario names that may cut a member: the points it holds, loads and asks of.
fn named_points(sk: &Sketch, sc: &Scenario) -> Vec<V> {
    let named = sc.supports.iter().map(|s| &s.at)
        .chain(sc.loads.iter().map(|l| &l.at))
        .chain(sc.requirements.iter().map(|r| &r.of));
    named.filter_map(|a| match a {
        Applied::Ent(e) if e.kind == EntKind::Point => Some(sk.world_point(e.i())),
        _ => None,
    }).collect()
}

/// One frame of the scenario, analysed: its structure and columns, the solved displacements,
/// the load each degree of freedom carries, and per element the load spread along it (in the
/// world's axes, a force a length) and the element's member.
struct Analysed {
    s: Structure,
    x: Vec<f64>,
    st: State,
    f: Vec<f64>,
    spread: Vec<V>,
    member: Vec<usize>,
    /// Each node of the frame by the mesh's numbering, to its own.
    local: BTreeMap<usize, usize>,
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

impl From<&str> for Unread {
    fn from(m: &str) -> Unread {
        Unread::Why(m.to_string())
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

impl Analysed {
    /// The frame `frame` of `mesh` under scenario `sc`, solved, or why it cannot be.
    fn new(sk: &Sketch, sc: &Scenario, mesh: &Mesh, frame: usize) -> Result<Analysed, Unread>
    {
        let local: BTreeMap<usize, usize> = (0..mesh.nodes.len())
            .filter(|&i| mesh.frame[i] == frame)
            .enumerate().map(|(k, i)| (i, k)).collect();
        let els: Vec<usize> =
            (0..mesh.elements.len()).filter(|&e| local.contains_key(&mesh.elements[e].1[0]))
                .collect();
        let names = frame_names(sk, mesh, &els);
        // every beam's moduli, read once
        let mut moduli: BTreeMap<usize, (f64, f64, f64)> = BTreeMap::new();
        for &e in &els {
            let beam = mesh.members[mesh.elements[e].0].beam;
            if let std::collections::btree_map::Entry::Vacant(v) = moduli.entry(beam) {
                let young = property(sk, beam, Property::E)?;
                let nu = property(sk, beam, Property::Nu).map_err(|m| {
                    format!("{m}: a frame twists, and its shear modulus is E / 2(1 + nu)")
                })?;
                let rho = property(sk, beam, Property::Rho).unwrap_or(f64::NAN);
                v.insert((young, young / (2.0 * (1.0 + nu)), rho));
            }
        }
        let mut x = vec![0.0; 3 * local.len()];
        for (&i, &k) in &local {
            x[3 * k..3 * k + 3].copy_from_slice(&mesh.nodes[i]);
        }
        let mut elements = Vec::new();
        let mut member = Vec::new();
        for &e in &els {
            let (m, [a, b], z) = mesh.elements[e];
            let beam = mesh.members[m].beam;
            let (young, g, rho) = moduli[&beam];
            elements.push(Element { a: local[&a], b: local[&b], e: young, g, rho, z });
            member.push(m);
        }
        for &e in &els {
            x.extend(sk.beams[mesh.members[mesh.elements[e].0].beam].section.stiffness());
        }
        let n = local.len();
        let mut s = Structure { dim: 3, nodes: n, bars: Vec::new(), beams: elements,
                                fixed: vec![false; 6 * n], load: vec![0.0; 6 * n],
                                weight: vec![0.0; 6 * n] };
        // the supports that fall on this frame
        for sup in &sc.supports {
            let at: Vec<usize> =
                mesh.nodes_of(sk, &sup.at).into_iter().filter_map(|i| local.get(&i).copied())
                    .collect();
            if at.is_empty() {
                continue;
            }
            let moves: Vec<usize> = match sup.along {
                None => vec![0, 1, 2],
                Some(d) => {
                    let d = direction(sk, d).ok_or("`along:` names a direction of no length")?;
                    match (0..3).find(|&k| (d[k].abs() - 1.0).abs() <= 1e-9) {
                        Some(k) => vec![k],
                        None => return Err(Unread::Why(format!(
                            "a support holds {names} along a direction off the world's axes, \
                             which a frame is not read under yet"))),
                    }
                }
            };
            for node in at {
                for &c in &moves {
                    s.fixed[6 * node + c] = true;
                }
                if sup.hold == Hold::Fixed {
                    for c in 3..6 {
                        s.fixed[6 * node + c] = true;
                    }
                }
            }
        }
        // the loads: a force at a joint, or spread along what it names
        let mut spread = vec![[0.0; 3]; s.beams.len()];
        for load in &sc.loads {
            let v = match (&load.value, load.kind) {
                (LoadValue::Vector(v), _) => *v,
                (LoadValue::Along { magnitude, along }, _) => {
                    let d = direction(sk, *along).ok_or("`along:` names a direction of no length")?;
                    scale(d, *magnitude)
                }
                (LoadValue::Scalar(_), _) => continue, // a pressure, on a face
            };
            let at_point = matches!(load.at, Applied::Ent(e) if e.kind == EntKind::Point);
            match load.kind {
                LoadKind::Force if at_point => {
                    let Some(node) = mesh.nodes_of(sk, &load.at).first()
                        .and_then(|i| local.get(i).copied()) else { continue };
                    for c in 0..3 {
                        s.load[6 * node + c] += v[c];
                    }
                }
                LoadKind::Force | LoadKind::Acceleration => {
                    let ms = mesh.members_of(&load.at);
                    let on: Vec<usize> = (0..s.beams.len()).filter(|&e| ms.contains(&member[e]))
                        .collect();
                    if on.is_empty() {
                        continue;
                    }
                    let length: f64 = on.iter().map(|&e| s.beam_axes(e, &x).map_or(0.0, |a| a.0))
                        .sum();
                    for e in on {
                        let q = if load.kind == LoadKind::Force {
                            scale(v, 1.0 / length)
                        } else {
                            let rho = s.beams[e].rho;
                            if !rho.is_finite() {
                                let b = mesh.members[member[e]].beam;
                                return Err(property(sk, b, Property::Rho).unwrap_err().into());
                            }
                            scale(v, rho * x[3 * n + 4 * e])
                        };
                        spread[e] = add(spread[e], q);
                    }
                }
                LoadKind::Pressure => {}
            }
        }
        // a load spread along an element is its consistent load at its ends
        let mut f = s.load.clone();
        for e in 0..s.beams.len() {
            if spread[e] == [0.0; 3] {
                continue;
            }
            let (h, axes) = s.beam_axes(e, &x).ok_or("a member of no length")?;
            let fl = fixed_end(local_q(&axes, spread[e]), h);
            let [a, b] = [s.beams[e].a, s.beams[e].b];
            for (end, node) in [(0, a), (1, b)] {
                for k in 0..3 {
                    for block in 0..2 {
                        let w: f64 = (0..3).map(|j| axes[j][k] * fl[6 * end + 3 * block + j])
                            .sum();
                        f[6 * node + 3 * block + k] += w;
                    }
                }
            }
        }
        s.load = f.clone();
        let st = s.solve(&x).map_err(|why| match why {
            Failure::Free => {
                let free = s.free(&x).text(3).unwrap_or_else(|| "free".to_string());
                Unread::Free { names: names.clone(), free }
            }
            Failure::Collapsed => Unread::Why(format!("a member of {names} has no length")),
            Failure::Unstiff => Unread::Why(format!("a member of {names} has no stiffness")),
        })?;
        Ok(Analysed { s, x, st, f, spread, member, local })
    }

    /// The displacement of node `i` (the mesh's numbering).
    fn moved(&self, i: usize) -> V {
        let k = self.local[&i];
        [0, 1, 2].map(|c| self.st.u[6 * k + c])
    }

    /// The elements of members `ms`.
    fn elements_of(&self, ms: &[usize]) -> Vec<usize> {
        (0..self.s.beams.len()).filter(|&e| ms.contains(&self.member[e])).collect()
    }

    /// `[A, Iy, Iz, J]` of element `e`.
    fn section(&self, e: usize) -> &[f64] {
        let at = 3 * self.s.nodes + 4 * e;
        &self.x[at..at + 4]
    }

    /// Element `e`'s end actions on it at its near end, in its own axes, the load spread along
    /// it taken out: the forces and moments its near node holds it by.
    fn actions(&self, e: usize, all: &[Option<[f64; 12]>]) -> ([f64; 12], [f64; 3], f64) {
        let (h, axes) = self.s.beam_axes(e, &self.x).unwrap();
        let q = local_q(&axes, self.spread[e]);
        let fe = fixed_end(q, h);
        let mut r = all[e].unwrap();
        for k in 0..12 {
            r[k] -= fe[k];
        }
        (r, q, h)
    }

    /// Each element's pull at its middle (negative pressed).
    fn pulls(&self) -> Vec<f64> {
        let all = self.s.end_actions(&self.st.u, &self.x);
        (0..self.s.beams.len()).map(|e| {
            let (r, q, h) = self.actions(e, &all);
            -(r[0] + q[0] * h / 2.0)
        }).collect()
    }
}

/// A load a length in the world's axes, in an element's own.
fn local_q(axes: &[[f64; 3]; 3], q: V) -> V {
    [0, 1, 2].map(|j| dot(axes[j], q))
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
/// value or why none, the words its frames say, and which supports hold a frame.
pub fn read(sk: &Sketch, sc: &Scenario) -> Read {
    let mut out = Read {
        words: Vec::new(),
        answers: vec![None; sc.requirements.len()],
        holds_frame: vec![false; sc.supports.len()],
    };
    if sk.beams.is_empty() {
        return out;
    }
    let mesh = match Mesh::new(sk, &named_points(sk, sc)) {
        Ok(m) => m,
        Err(m) => {
            out.words.push(format!("scenario `{}`: {m}", sc.name));
            return out;
        }
    };
    for (k, s) in sc.supports.iter().enumerate() {
        out.holds_frame[k] = !mesh.nodes_of(sk, &s.at).is_empty();
    }
    let mut frames: BTreeMap<usize, Result<Analysed, Unread>> = BTreeMap::new();
    for (k, r) in sc.requirements.iter().enumerate() {
        let Some(frame) = frame_of(sk, &mesh, &r.of) else { continue };
        let frame = match frame {
            Ok(f) => f,
            Err(m) => {
                out.answers[k] = Some(Err(m));
                continue;
            }
        };
        let a = frames.entry(frame)
            .or_insert_with(|| Analysed::new(sk, sc, &mesh, frame));
        out.answers[k] = Some(match a {
            Ok(a) => answer(sk, &mesh, a, r),
            Err(m) => Err(m.text()),
        });
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
    let what = |of: &Applied| match of {
        Applied::Beam(b) => format!("`{}`", sk.beams[*b].name),
        _ => "it".to_string(),
    };
    match r.output {
        Output::Deflection => {
            let along = r.along.map(|d| direction(sk, d).ok_or("`along:` names no direction"))
                .transpose()?;
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
        Output::Compliance => Ok(a.f.iter().zip(&a.st.u).map(|(f, u)| f * u).sum()),
        Output::Mass => {
            let ms = mesh.members_of(&r.of);
            if ms.is_empty() {
                return Err("a mass is of a beam or a member, and this names a point".into());
            }
            let mut m = 0.0;
            for e in a.elements_of(&ms) {
                let rho = a.s.beams[e].rho;
                if !rho.is_finite() {
                    return Err(property(sk, mesh.members[a.member[e]].beam, Property::Rho)
                        .unwrap_err());
                }
                m += rho * a.section(e)[0] * a.s.beam_axes(e, &a.x).map_or(0.0, |x| x.0);
            }
            Ok(m)
        }
        Output::Mode => {
            if let Some(e) = (0..a.s.beams.len()).find(|&e| !a.s.beams[e].rho.is_finite()) {
                return Err(property(sk, mesh.members[a.member[e]].beam, Property::Rho)
                    .unwrap_err());
            }
            let n = r.count.unwrap_or(1) as usize;
            let k = a.s.stiffness(&a.x).map_err(|_| "no stiffness".to_string())?;
            let m = a.s.mass(&a.x).map_err(|_| "no mass".to_string())?;
            let pairs = eigen::lowest(&k, &m, n, &eigen::Opts::default())
                .map_err(|why| format!("its modes are not found: {why:?}"))?;
            let v = *pairs.values.get(n - 1).ok_or("it has fewer modes than that")?;
            Ok(v.max(0.0).sqrt() / (2.0 * std::f64::consts::PI))
        }
        Output::Buckling => {
            let k = a.s.stiffness(&a.x).map_err(|_| "no stiffness".to_string())?;
            let kg = a.s.geometric(&a.x, &a.pulls()).map_err(|_| "no stiffness".to_string())?;
            let pairs = eigen::buckling(&k, &kg, 1, &eigen::Opts::default())
                .map_err(|why| format!("its buckling is not found: {why:?}"))?;
            pairs.values.first().copied()
                .ok_or_else(|| "the loads press no member, so nothing buckles".to_string())
        }
        Output::Safety => {
            let ms = mesh.members_of(&r.of);
            if ms.is_empty() {
                return Err("a factor of safety is of a beam or a member".into());
            }
            let all = a.s.end_actions(&a.st.u, &a.x);
            let mut worst = 0.0f64;
            let mut yields = f64::INFINITY;
            for e in a.elements_of(&ms) {
                let beam = mesh.members[a.member[e]].beam;
                let b = &sk.beams[beam];
                let (sy, sz) = (b.section.get(SectionProp::Sy), b.section.get(SectionProp::Sz));
                let (Some(sy), Some(sz)) = (sy, sz) else {
                    let states = if sy.is_some() || sz.is_some() { "one" } else { "neither" };
                    return Err(format!("a factor of safety reads `{}`'s section moduli, `Sy` \
                                        and `Sz`, and it states {states}", b.name));
                };
                yields = yields.min(property(sk, beam, Property::Yield)?);
                let (r, q, h) = a.actions(e, &all);
                worst = worst.max(stress(&r, q, h, a.section(e)[0], sy, sz));
            }
            if !(worst > 0.0) {
                return Err(format!("the loads stress {} nowhere", what(&r.of)));
            }
            Ok(yields / worst)
        }
    }
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
