//! **CalculiX as the corpus's oracle** (#510 P8), as OCCT is the B-rep kernel's: an external
//! solver, in the tests alone, judging every claim the corpus states.  Each case's solid is built
//! by this kernel and written as STEP; gmsh meshes it in quadratic tetrahedra (C3D10), or meshes
//! the face it sweeps in quadratic triangles (CPS6) where the case is read in the plane; the
//! scenario's supports and loads are laid on the mesh by which of the solid's faces each boundary
//! facet lies on, read by this kernel's own surfaces; and CalculiX's answer is held to the
//! document's bracket.  Nothing here is the analysis #510 builds: it is what that analysis will
//! be compared with, and the evidence that each case says what its references say.
//!
//! Gated on `--features gcs-core/ccx` (`make test-ccx`), with `ccx` and `gmsh` on the path.
use super::*;
use gcs_core::brep::geom::V;
use gcs_core::model::{EntKind, EntRef, Hold, Scenario, Sketch};
use gcs_core::space::{cross, distance, dot, lerp, norm, sub, unit};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

/// The tolerance the STEP file is written to, millimetres: how far gmsh's nodes may stand off this
/// kernel's surfaces.
const STEP_TOL: f64 = 1e-4;

/// A quadratic element's edges by their corners, in Abaqus's order: a triangle's are the first
/// three, and the midside node of edge `i` follows the corners, at `corners + i`.
const EDGES: [(usize, usize); 6] = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)];

/// A quadratic triangle's or tetrahedron's shape functions at barycentric coordinates `l`: each
/// corner's `l(2l − 1)`, then each edge's `4 lₐ l_b`.
fn quadratic(l: &[f64]) -> Vec<f64> {
    let edges = if l.len() == 3 { 3 } else { 6 };
    let mut n: Vec<f64> = l.iter().map(|&x| x * (2.0 * x - 1.0)).collect();
    n.extend(EDGES[..edges].iter().map(|&(a, b)| 4.0 * l[a] * l[b]));
    n
}

/// The sum of `points` weighted by `w`.
fn weighted(w: &[f64], points: impl Fn(usize) -> V) -> V {
    let mut at = [0.0; 3];
    for (k, &wk) in w.iter().enumerate() {
        let p = points(k);
        for c in 0..3 {
            at[c] += wk * p[c];
        }
    }
    at
}

/// A mesh in the B-rep's frame (millimetres about the solid's origin): its nodes, and its
/// elements as node indices in Abaqus's order, corners first.
struct Mesh {
    nodes: Vec<V>,
    elements: Vec<Vec<usize>>,
    plane: bool,
}

/// A facet of the mesh's boundary: a quadratic triangle (corners, then the midpoints of 0–1,
/// 1–2 and 2–0) of a solid, or a quadratic edge (its ends, then its middle) of a plane mesh,
/// with the corner of its element it faces away from.
struct Facet {
    nodes: Vec<usize>,
    inside: usize,
}

impl Mesh {
    fn corners(&self) -> usize {
        if self.plane { 3 } else { 4 }
    }

    /// The boundary: each facet one element has and no other.
    fn boundary(&self) -> Vec<Facet> {
        // a C3D10's faces and a CPS6's edges, each with the corner opposite
        const TET: [([usize; 6], usize); 4] = [
            ([0, 1, 2, 4, 5, 6], 3),
            ([0, 3, 1, 7, 8, 4], 2),
            ([1, 3, 2, 8, 9, 5], 0),
            ([2, 3, 0, 9, 7, 6], 1),
        ];
        const TRI: [([usize; 3], usize); 3] = [([0, 1, 3], 2), ([1, 2, 4], 0), ([2, 0, 5], 1)];
        let faces: Vec<(&[usize], usize)> = if self.plane {
            TRI.iter().map(|(f, o)| (&f[..], *o)).collect()
        } else {
            TET.iter().map(|(f, o)| (&f[..], *o)).collect()
        };
        let mut seen: BTreeMap<Vec<usize>, (usize, Facet)> = BTreeMap::new();
        for el in &self.elements {
            for &(f, o) in &faces {
                let nodes: Vec<usize> = f.iter().map(|&k| el[k]).collect();
                let mut key = nodes[..self.corners() - 1].to_vec();
                key.sort();
                seen.entry(key).or_insert((0, Facet { nodes, inside: el[o] })).0 += 1;
            }
        }
        seen.into_values().filter(|(n, _)| *n == 1).map(|(_, f)| f).collect()
    }

    /// Every midside node is near the middle of its edge: the element's order is Abaqus's.
    fn check_order(&self) {
        let edges = if self.plane { 3 } else { 6 };
        for el in &self.elements {
            for (i, &(a, b)) in EDGES[..edges].iter().enumerate() {
                let (pa, pb) = (self.nodes[el[a]], self.nodes[el[b]]);
                let pm = self.nodes[el[self.corners() + i]];
                assert!(distance(lerp(pa, pb, 0.5), pm) < 0.25 * distance(pa, pb),
                        "a midside node off its edge: {el:?}");
            }
        }
    }
}

/// A quadratic triangle's shape functions and their derivatives in `(r, s)`, nodes as `Facet`.
fn tri6(r: f64, s: f64) -> (Vec<f64>, [[f64; 2]; 6]) {
    let (l1, l2, l3) = (1.0 - r - s, r, s);
    let d = [
        [-(4.0 * l1 - 1.0), -(4.0 * l1 - 1.0)],
        [4.0 * l2 - 1.0, 0.0],
        [0.0, 4.0 * l3 - 1.0],
        [4.0 * (l1 - l2), -4.0 * l2],
        [4.0 * l3, 4.0 * l2],
        [-4.0 * l3, 4.0 * (l1 - l3)],
    ];
    (quadratic(&[l1, l2, l3]), d)
}

/// Strang and Fix's six points `(r, s, weight)` on the reference triangle, exact to degree four,
/// the weights summing to its area.
const TRI_RULE: [(f64, f64, f64); 6] = {
    const A: f64 = 0.445948490915965;
    const WA: f64 = 0.5 * 0.223381589678011;
    const B: f64 = 0.091576213509771;
    const WB: f64 = 0.5 * 0.109951743655322;
    [(A, A, WA), (1.0 - 2.0 * A, A, WA), (A, 1.0 - 2.0 * A, WA),
     (B, B, WB), (1.0 - 2.0 * B, B, WB), (B, 1.0 - 2.0 * B, WB)]
};

/// `∫ N_i n dA` over a facet, for each of its nodes: `n` its outward normal (out of the element),
/// `dA` its area — a plane mesh's edges times the thickness.  A uniform traction `t` lays `t`
/// times the area part on each node, and a pressure `p` lays `−p` times the vector part.
fn integrals(mesh: &Mesh, f: &Facet, thickness: f64) -> Vec<(f64, V)> {
    let x = |k: usize| mesh.nodes[f.nodes[k]];
    let mut out = vec![(0.0, [0.0; 3]); f.nodes.len()];
    let away = |n: V, at: V| if dot(n, sub(at, mesh.nodes[f.inside])) < 0.0 { -1.0 } else { 1.0 };
    let mut add = |n: &[f64], w: f64, normal: V, area: f64, at: V| {
        let sign = away(normal, at);
        for k in 0..n.len() {
            out[k].0 += w * n[k] * area;
            for c in 0..3 {
                out[k].1[c] += w * n[k] * sign * normal[c];
            }
        }
    };
    if mesh.plane {
        let g = (3.0f64 / 5.0).sqrt();
        for (xi, w) in [(-g, 5.0 / 9.0), (0.0, 8.0 / 9.0), (g, 5.0 / 9.0)] {
            let n = [xi * (xi - 1.0) / 2.0, xi * (xi + 1.0) / 2.0, 1.0 - xi * xi];
            let t = weighted(&[xi - 0.5, xi + 0.5, -2.0 * xi], x);
            // the profile lies in x and z (`swept_across_y`): the edge's normal in it, times the
            // depth, and its length times the depth
            let normal = [t[2] * thickness, 0.0, -t[0] * thickness];
            add(&n, w, normal, norm(normal), weighted(&n, x));
        }
    } else {
        for (r, s, w) in TRI_RULE {
            let (n, d) = tri6(r, s);
            let xr = weighted(&d.map(|dk| dk[0]), x);
            let xs = weighted(&d.map(|dk| dk[1]), x);
            let normal = cross(xr, xs);
            add(&n, w, normal, norm(normal), weighted(&n, x));
        }
    }
    out
}

/// The point on a facet its shape functions put at `(1/3, 1/3)` — on the curved facet, not its
/// chord — or a plane edge's middle node.
fn facet_middle(mesh: &Mesh, f: &Facet) -> V {
    if mesh.plane {
        return mesh.nodes[f.nodes[2]];
    }
    weighted(&tri6(1.0 / 3.0, 1.0 / 3.0).0, |k| mesh.nodes[f.nodes[k]])
}

/// How far `p` is from face `f`'s surface.
fn off(x: &gcs_core::solid::Exact, f: usize, p: V) -> f64 {
    let s = &x.brep.faces[f].surface;
    distance(s.point(s.inverse(p)), p)
}

/// The face of the solid each boundary facet lies on, by its own name (`root` of `beam.root`;
/// every case is one primitive): the nearest surface to the facet's middle, every node of it on
/// that surface to the STEP file's tolerance.  A plane mesh's caps — the faces the whole profile
/// lies in — are no facet's.
fn classify(x: &gcs_core::solid::Exact, mesh: &Mesh, facets: &[Facet]) -> Vec<String> {
    assert!(x.named.iter().all(|n| n.0 == x.named[0].0), "the oracle reads one primitive's faces");
    let on = 10.0 * STEP_TOL;
    let probes = [0, mesh.nodes.len() / 2, mesh.nodes.len() - 1];
    let faces: Vec<usize> = (0..x.brep.faces.len())
        .filter(|&f| !mesh.plane || !probes.iter().all(|&n| off(x, f, mesh.nodes[n]) < on))
        .collect();
    facets.iter().map(|fa| {
        let mid = facet_middle(mesh, fa);
        let (_, f) = faces.iter().map(|&f| (off(x, f, mid), f))
            .min_by(|a, b| a.0.total_cmp(&b.0)).unwrap();
        for &n in &fa.nodes {
            let d = off(x, f, mesh.nodes[n]);
            assert!(d < on, "a facet {d} mm off `{}`", x.brep.faces[f].name);
        }
        // the face's own name in the primitive it is a piece of, as a face path names it
        x.named[f].1.clone()
    }).collect()
}

/// A plane case is drawn in the front and swept across y: its caps (`near`, `far`) are square to
/// y, so its profile lies in x and z, the axes the plane reading is written in.
fn swept_across_y(x: &gcs_core::solid::Exact) {
    let (lo, hi) = x.brep.bounds();
    let middle = lerp(lo, hi, 0.5);
    let caps = x.named.iter().enumerate().filter(|(_, n)| n.1 == "near" || n.1 == "far");
    for (f, (_, name)) in caps {
        let s = &x.brep.faces[f].surface;
        let n = s.normal(s.inverse(middle)).expect("a cap's normal");
        assert!((n[1].abs() - 1.0).abs() < 1e-12, "the oracle reads a plane case swept across y: \
                                                  `{name}` faces {n:?}");
    }
}

/// The B-rep's frame, millimetres about the solid's origin, of a point of the drawing.
fn framed(e: &Elaborated, x: &gcs_core::solid::Exact, p: EntRef) -> V {
    let w = e.sketch.world_point(p.i());
    std::array::from_fn(|k| (w[k] - x.origin[k]) * x.mm)
}

/// The node nearest a point, in the profile's plane where the mesh is one.
fn nearest(mesh: &Mesh, p: V) -> usize {
    let flat = |q: V| if mesh.plane { [q[0], 0.0, q[2]] } else { q };
    let d = |n: usize| distance(flat(mesh.nodes[n]), flat(p));
    (0..mesh.nodes.len()).map(|n| (d(n), n)).min_by(|a, b| a.0.total_cmp(&b.0)).unwrap().1
}

/// The unit direction an `along:` names.
fn direction(sk: &Sketch, d: EntRef) -> V {
    assert_eq!(d.kind, EntKind::Axis, "the oracle reads `along:` an axis");
    unit(sk.axes[d.i()].d.map(|p| sk.params[p as usize].value))
}

/// The world axis an `along:` names: its index, 0 for x.
fn axis_of(sk: &Sketch, d: EntRef) -> usize {
    let dir = direction(sk, d);
    let k = (0..3).max_by(|&a, &b| dir[a].abs().total_cmp(&dir[b].abs())).unwrap();
    assert!((dir[k].abs() - 1.0).abs() < 1e-12, "the oracle holds along the world's axes: {dir:?}");
    k
}

/// What a deck holds: each node's held directions and its load, newtons.
struct Laid {
    held: BTreeMap<usize, [bool; 3]>,
    force: BTreeMap<usize, V>,
}

/// The scenario's supports and loads, laid on the mesh.
fn lay(e: &Elaborated, x: &gcs_core::solid::Exact, s: &Scenario, mesh: &Mesh, facets: &[Facet],
       on: &[String], thickness: f64) -> Laid {
    let sk = &e.sketch;
    let si = |v: f64, dim: Dim| sk.units.to_si(v, dim);
    // the facets on the face a path names
    let on_face = |f: &gcs_core::model::FaceRef| -> Vec<&Facet> {
        let name = f.path.last().unwrap();
        let mine: Vec<&Facet> =
            facets.iter().zip(on).filter(|(_, n)| *n == name).map(|(fa, _)| fa).collect();
        assert!(!mine.is_empty(), "no facet on `{name}`");
        mine
    };
    let nodes_of = |a: &Applied| -> Vec<usize> {
        match a {
            Applied::Face(f) => {
                let mut ns: Vec<usize> =
                    on_face(f).iter().flat_map(|fa| fa.nodes.clone()).collect();
                ns.sort();
                ns.dedup();
                ns
            }
            Applied::Ent(p) if p.kind == EntKind::Point => vec![nearest(mesh, framed(e, x, *p))],
            other => panic!("the oracle lays nothing on {other:?}"),
        }
    };
    let mut held: BTreeMap<usize, [bool; 3]> = BTreeMap::new();
    for sup in &s.supports {
        assert!(matches!(sup.hold, Hold::Fixed | Hold::Pinned));
        let dofs: Vec<usize> = match sup.along {
            Some(d) => vec![axis_of(sk, d)],
            None => vec![0, 1, 2],
        };
        // in the plane nothing moves across it: a point held across the thickness is the solid
        // reading's, taking out what the cuts leave free, and nothing else may be held only so
        if mesh.plane && dofs == [1] {
            assert!(matches!(sup.at, Applied::Ent(_)),
                    "a plane reading drops only a point held across the thickness: {sup:?}");
            continue;
        }
        let dofs: Vec<usize> = dofs.into_iter().filter(|&k| !mesh.plane || k != 1).collect();
        for n in nodes_of(&sup.at) {
            for &k in &dofs {
                held.entry(n).or_default()[k] = true;
            }
        }
    }
    let mut force: BTreeMap<usize, V> = BTreeMap::new();
    let mut lay_on = |n: usize, f: V| {
        let at = force.entry(n).or_default();
        for c in 0..3 {
            at[c] += f[c];
        }
    };
    for l in &s.loads {
        match (l.kind, &l.at, &l.value) {
            (LoadKind::Force, at, value) => {
                let total: V = match value {
                    LoadValue::Vector(v) => v.map(|c| si(c, l.kind.dim())),
                    LoadValue::Along { magnitude, along } => {
                        direction(sk, *along).map(|c| c * si(*magnitude, l.kind.dim()))
                    }
                    LoadValue::Scalar(_) => unreachable!(),
                };
                if let Applied::Face(f) = at {
                    // a uniform traction: the force over the face's area
                    let mine = on_face(f);
                    let parts: Vec<_> =
                        mine.iter().map(|f| integrals(mesh, f, thickness)).collect();
                    let area: f64 = parts.iter().flatten().map(|(a, _)| a).sum();
                    for (f, part) in mine.iter().zip(&parts) {
                        for (k, (a, _)) in part.iter().enumerate() {
                            lay_on(f.nodes[k], total.map(|c| c * a / area));
                        }
                    }
                } else {
                    for n in nodes_of(at) {
                        lay_on(n, total);
                    }
                }
            }
            (LoadKind::Pressure, Applied::Face(f), LoadValue::Scalar(p)) => {
                let p = si(*p, l.kind.dim());
                for fa in on_face(f) {
                    for (k, (_, nv)) in integrals(mesh, fa, thickness).into_iter().enumerate() {
                        // the integrals are in square millimetres
                        lay_on(fa.nodes[k], nv.map(|c| -p * c * 1e-6));
                    }
                }
            }
            (LoadKind::Acceleration, ..) => panic!("the oracle lays no body load yet"),
            other => unreachable!("{other:?}"),
        }
    }
    Laid { held, force }
}

/// The analysis a bracket needs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Kind {
    Static,
    Frequency,
    Buckle,
}

fn kind(o: Output) -> Option<Kind> {
    match o {
        Output::Deflection | Output::Safety => Some(Kind::Static),
        Output::Mode => Some(Kind::Frequency),
        Output::Buckling => Some(Kind::Buckle),
        Output::Mass | Output::Compliance => None,
    }
}

/// The mesh in metres, written once and included by each deck: a plane mesh's z is CalculiX's y,
/// its plane elements lying in x and y.
fn mesh_deck(mesh: &Mesh) -> String {
    use std::fmt::Write;
    let mut d = String::from("*NODE, NSET=NALL\n");
    for (i, p) in mesh.nodes.iter().enumerate() {
        let [x, y, z] = if mesh.plane { [p[0], p[2], 0.0] } else { *p }.map(|c| c * 1e-3);
        writeln!(d, "{}, {x:.12e}, {y:.12e}, {z:.12e}", i + 1).unwrap();
    }
    let ty = if mesh.plane { "CPS6" } else { "C3D10" };
    writeln!(d, "*ELEMENT, TYPE={ty}, ELSET=EALL").unwrap();
    for (i, el) in mesh.elements.iter().enumerate() {
        let ns: Vec<String> = el.iter().map(|n| (n + 1).to_string()).collect();
        writeln!(d, "{}, {}", i + 1, ns.join(", ")).unwrap();
    }
    d
}

/// A deck: the mesh, the material in SI, the section, the supports, and one step asking for what
/// the brackets read — the nodes' displacements, their stresses, or `modes` frequencies.
fn deck(m: &Material, plane: bool, laid: &Laid, thickness: f64, k: Kind, modes: u32,
        out: &[&str]) -> String {
    use std::fmt::Write;
    let mut d = String::from("*HEADING\nSolvent's verification corpus\n*INCLUDE, INPUT=mesh.inp\n");
    let (young, nu) = (m.get(Property::E).unwrap(), m.get(Property::Nu).unwrap());
    writeln!(d, "*MATERIAL, NAME=M\n*ELASTIC\n{young:.12e}, {nu}").unwrap();
    if let Some(rho) = m.get(Property::Rho) {
        writeln!(d, "*DENSITY\n{rho:.12e}").unwrap();
    }
    d.push_str("*SOLID SECTION, ELSET=EALL, MATERIAL=M\n");
    if plane {
        writeln!(d, "{:.12e}", thickness * 1e-3).unwrap();
    }
    // the deck's own axes: a plane mesh's z is CalculiX's y (its y, across the thickness, is
    // never held or loaded)
    let dof = |c: usize| if plane { [1, 0, 2][c] } else { c + 1 };
    d.push_str("*BOUNDARY\n");
    for (n, h) in &laid.held {
        for c in (0..3).filter(|&c| h[c]) {
            writeln!(d, "{}, {}, {}", n + 1, dof(c), dof(c)).unwrap();
        }
    }
    match k {
        Kind::Static => d.push_str("*STEP\n*STATIC\n"),
        Kind::Frequency => writeln!(d, "*STEP\n*FREQUENCY, SOLVER=SPOOLES\n{modes}").unwrap(),
        Kind::Buckle => d.push_str("*STEP\n*BUCKLE, SOLVER=SPOOLES\n2\n"),
    }
    if k != Kind::Frequency {
        d.push_str("*CLOAD\n");
        for (n, f) in &laid.force {
            for c in (0..3).filter(|&c| f[c] != 0.0 && !(plane && c == 1)) {
                writeln!(d, "{}, {}, {:.12e}", n + 1, dof(c), f[c]).unwrap();
            }
        }
    }
    for o in out {
        writeln!(d, "{o}").unwrap();
    }
    d.push_str("*END STEP\n");
    d
}

/// Runs a program in `dir`, failing the test with its output where it fails.
fn run(dir: &Path, program: &str, args: &[&str]) {
    let out = Command::new(program).args(args).current_dir(dir).output()
        .unwrap_or_else(|e| panic!("`{program}` does not run ({e}): the oracle needs it on the \
                                    path"));
    assert!(out.status.success(), "`{program} {}` failed:\n{}\n{}", args.join(" "),
            String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
}

/// The case's solid meshed by gmsh from its STEP file.
fn mesh(case: &Case, x: &gcs_core::solid::Exact, dir: &Path) -> Mesh {
    let step = gcs_core::brep::step::write(&x.brep, case.name, STEP_TOL).expect("STEP");
    std::fs::write(dir.join("solid.step"), step).unwrap();
    let (size, curvature) = case.mesh;
    let plane = case.reading == Reading::Plane;
    let (lo, hi) = x.brep.bounds();
    let e = 1e-3 * x.brep.size();
    let group = if plane {
        swept_across_y(x);
        // the cap at the least y: the face the solid sweeps
        format!("s() = Surface In BoundingBox{{{}, {}, {}, {}, {}, {}}};\n\
                 Physical Surface(1) = {{s()}};\n",
                lo[0] - e, lo[1] - e, lo[2] - e, hi[0] + e, lo[1] + e, hi[2] + e)
    } else {
        "Physical Volume(1) = Volume{:};\n".to_string()
    };
    let geo = format!("Merge \"solid.step\";\n{group}Mesh.ElementOrder = 2;\n\
                       Mesh.SecondOrderLinear = 0;\nMesh.MeshSizeMax = {size};\n\
                       Mesh.MeshSizeFromCurvature = {curvature};\nMesh.Optimize = 1;\n");
    std::fs::write(dir.join("mesh.geo"), geo).unwrap();
    let dim = if plane { "-2" } else { "-3" };
    run(dir, "gmsh", &["mesh.geo", dim, "-format", "inp", "-o", "gmsh.inp", "-v", "2"]);
    read_inp(&std::fs::read_to_string(dir.join("gmsh.inp")).unwrap(), plane)
}

/// gmsh's Abaqus file: its nodes and the elements of the one type wanted, renumbered from zero
/// over the nodes they use.
fn read_inp(text: &str, plane: bool) -> Mesh {
    let want = if plane { "CPS6" } else { "C3D10" };
    let (mut at, mut raw): (BTreeMap<usize, V>, Vec<Vec<usize>>) = (BTreeMap::new(), Vec::new());
    let mut section = "";
    let mut pending: Vec<usize> = Vec::new();
    for line in text.lines() {
        let l = line.trim();
        if l.starts_with('*') {
            let up = l.to_uppercase();
            section = if up.starts_with("*NODE") {
                "node"
            } else if up.starts_with("*ELEMENT") && up.contains(&format!("TYPE={want}")) {
                "element"
            } else {
                ""
            };
            continue;
        }
        let nums: Vec<&str> = l.split(',').map(str::trim).filter(|t| !t.is_empty()).collect();
        match section {
            "node" if nums.len() >= 4 => {
                let v: Vec<f64> = nums[1..4].iter().map(|t| t.parse().unwrap()).collect();
                at.insert(nums[0].parse().unwrap(), [v[0], v[1], v[2]]);
            }
            "element" => {
                pending.extend(nums.iter().map(|t| t.parse::<usize>().unwrap()));
                if !l.ends_with(',') {
                    raw.push(pending[1..].to_vec());
                    pending.clear();
                }
            }
            _ => {}
        }
    }
    let mut index: BTreeMap<usize, usize> = BTreeMap::new();
    let mut nodes = Vec::new();
    let elements = raw.iter().map(|el| el.iter().map(|n| *index.entry(*n).or_insert_with(|| {
        nodes.push(at[n]);
        nodes.len() - 1
    })).collect()).collect();
    let m = Mesh { nodes, elements, plane };
    assert!(!m.elements.is_empty(), "gmsh wrote no {want}");
    m.check_order();
    m
}

/// The numbers in a line, where every token is one.
fn numbers(l: &str) -> Option<Vec<f64>> {
    l.split_whitespace().map(|t| t.parse::<f64>().ok()).collect()
}

/// The table under a heading of CalculiX's `.dat`: its rows of `width` numbers, the first a
/// count from 1, taken in each block the heading opens.
fn table(dat: &str, heading: &str, width: usize) -> Vec<Vec<f64>> {
    let mut out = Vec::new();
    let mut inside = false;
    for l in dat.lines() {
        if l.contains(heading) {
            inside = true;
            continue;
        }
        if !inside {
            continue;
        }
        match numbers(l) {
            Some(v) if v.len() == width && v[0] >= 1.0 && v[0].fract() == 0.0 => out.push(v),
            _ if !out.is_empty() && l.trim().is_empty() => inside = false,
            _ => {}
        }
    }
    out
}

/// The last block of each result in CalculiX's `.frd` (`DISP`, `STRESS`), node by node.
fn frd(text: &str) -> BTreeMap<String, BTreeMap<usize, Vec<f64>>> {
    let mut out: BTreeMap<String, BTreeMap<usize, Vec<f64>>> = BTreeMap::new();
    let mut inside: Option<String> = None;
    for l in text.lines() {
        if l.starts_with(" -4") {
            let name = l[3..].split_whitespace().next().unwrap().to_string();
            out.insert(name.clone(), BTreeMap::new());
            inside = Some(name);
        } else if l.starts_with(" -3") {
            inside = None;
        } else if let (Some(name), true) = (&inside, l.starts_with(" -1")) {
            let node: usize = l[3..13].trim().parse().unwrap();
            let vals: Vec<f64> = (13..l.len()).step_by(12)
                .map(|k| l[k..(k + 12).min(l.len())].trim().parse().unwrap())
                .collect();
            out.get_mut(name).unwrap().insert(node, vals);
        }
    }
    out
}

/// The displacement at a point, metres: the shape functions of the element that holds it.
fn displacement(mesh: &Mesh, u: &BTreeMap<usize, Vec<f64>>, p: V) -> V {
    // each element's barycentric coordinates of `p` from its corners
    let bary = |el: &Vec<usize>| -> Vec<f64> {
        let c: Vec<V> = (0..mesh.corners()).map(|k| mesh.nodes[el[k]]).collect();
        let q = sub(p, c[0]);
        if mesh.plane {
            let (a, b) = (sub(c[1], c[0]), sub(c[2], c[0]));
            let det = a[0] * b[2] - a[2] * b[0];
            let l2 = (q[0] * b[2] - q[2] * b[0]) / det;
            let l3 = (a[0] * q[2] - a[2] * q[0]) / det;
            vec![1.0 - l2 - l3, l2, l3]
        } else {
            let (a, b, cc) = (sub(c[1], c[0]), sub(c[2], c[0]), sub(c[3], c[0]));
            let det = dot(a, cross(b, cc));
            let l2 = dot(q, cross(b, cc)) / det;
            let l3 = dot(a, cross(q, cc)) / det;
            let l4 = dot(a, cross(b, q)) / det;
            vec![1.0 - l2 - l3 - l4, l2, l3, l4]
        }
    };
    // how far outside an element `p` is
    let outside = |l: &[f64]| l.iter().fold(0.0f64, |m, &x| m.max(-x));
    let (l, el) = mesh.elements.iter().map(|el| (bary(el), el))
        .min_by(|a, b| outside(&a.0).total_cmp(&outside(&b.0))).unwrap();
    assert!(outside(&l) < 1e-6, "{p:?} is in no element");
    weighted(&quadratic(&l), |k| {
        let v = &u[&(el[k] + 1)];
        // a plane deck's y is the drawing's z
        if mesh.plane { [v[0], 0.0, v[1]] } else { [v[0], v[1], v[2]] }
    })
}

/// The greatest von Mises stress at a node, pascals.
fn mises(s: &BTreeMap<usize, Vec<f64>>) -> f64 {
    s.values().map(|v| {
        let [xx, yy, zz, xy, yz, zx] = [v[0], v[1], v[2], v[3], v[4], v[5]];
        let normal = (xx - yy).powi(2) + (yy - zz).powi(2) + (zz - xx).powi(2);
        (0.5 * normal + 3.0 * (xy * xy + yz * yz + zx * zx)).sqrt()
    }).fold(0.0, f64::max)
}

/// A fresh directory for a case's files.
fn scratch(case: &Case) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("gcs-ccx-{}-{}", std::process::id(), case.name));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// What CalculiX answered one scenario: each result of a static step, its frequencies and its
/// buckling factors.
#[derive(Default)]
struct Answered {
    fields: BTreeMap<String, BTreeMap<usize, Vec<f64>>>,
    modes: Vec<Vec<f64>>,
    buckling: Vec<Vec<f64>>,
}

/// **The oracle's value of every bracket a case claims**, in the document's own units.
fn judge(case: &Case, e: &Elaborated) -> Vec<(Bracket, f64)> {
    let sk = &e.sketch;
    let x = exact(case, e);
    let dir = scratch(case);
    let m = mesh(case, &x, &dir);
    std::fs::write(dir.join("mesh.inp"), mesh_deck(&m)).unwrap();
    let facets = m.boundary();
    let on = classify(&x, &m, &facets);
    let thickness = extent(&x)[1] * x.mm;
    let material = material(case, e);
    let doc = |v: f64, dim: Dim| v / sk.units.to_si(1.0, dim);
    let bs = brackets(e);
    let mut out = Vec::new();
    for s in e.scenarios.iter().map(|s| &s.scenario) {
        let mine: Vec<&Bracket> = bs.iter().filter(|b| b.scenario == s.name).collect();
        let laid = lay(e, &x, s, &m, &facets, &on, thickness);
        let modes = mine.iter().filter_map(|b| b.count).max().unwrap_or(1) + 4;
        let mut answered = Answered::default();
        for k in [Kind::Static, Kind::Frequency, Kind::Buckle] {
            let wants = |o: Output| mine.iter().any(|b| b.output == o);
            if !mine.iter().any(|b| kind(b.output) == Some(k)) {
                continue;
            }
            let job = format!("{}_{:?}", s.name, k).to_lowercase();
            let mut asked = Vec::new();
            if wants(Output::Deflection) && k == Kind::Static {
                asked.push("*NODE FILE\nU");
            }
            if wants(Output::Safety) && k == Kind::Static {
                asked.push("*EL FILE\nS");
            }
            let text = deck(material, m.plane, &laid, thickness, k, modes, &asked);
            std::fs::write(dir.join(format!("{job}.inp")), text).unwrap();
            run(&dir, "ccx", &[&job]);
            let file =
                |ext: &str| std::fs::read_to_string(dir.join(format!("{job}.{ext}"))).unwrap();
            match k {
                Kind::Static => answered.fields = frd(&file("frd")),
                Kind::Frequency => {
                    answered.modes = table(&file("dat"), "E I G E N V A L U E   O U T P U T", 5)
                }
                Kind::Buckle => {
                    let heading = "B U C K L I N G   F A C T O R   O U T P U T";
                    answered.buckling = table(&file("dat"), heading, 2)
                }
            }
        }
        for b in mine {
            let v = match b.output {
                Output::Deflection => {
                    let Applied::Ent(p) = &b.of else { panic!("a deflection at a point") };
                    let u = displacement(&m, &answered.fields["DISP"], framed(e, &x, *p));
                    let along = b.along.map(|d| direction(sk, d));
                    doc(along.map_or(norm(u), |d| dot(u, d)), Dim::LENGTH)
                }
                Output::Safety => {
                    material.get(Property::Yield).unwrap() / mises(&answered.fields["STRESS"])
                }
                Output::Mode => answered.modes[b.count.unwrap() as usize - 1][3],
                Output::Buckling => answered.buckling[0][1],
                // the B-rep's volume, not CalculiX's: the one output with no analysis in it
                Output::Mass => {
                    // a B-rep is in millimetres, a mass in kilograms in any document
                    material.get(Property::Rho).unwrap() * volume(&x) * (x.mm * 1e-3).powi(3)
                }
                Output::Compliance => panic!("no case claims a compliance yet"),
            };
            out.push((b.clone(), v));
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
    out
}

/// **CalculiX keeps every claim the corpus states.**  Each case's answer by an independent solver,
/// on a mesh this kernel's solid gives, inside the bracket the document claims — the evidence
/// that the documents say what their references say, and what #510's own analysis will be held
/// to beside them.  The cases are independent and run side by side, each in its own thread and
/// its own directory.
#[test]
#[cfg_attr(not(feature = "ccx"),
           ignore = "the CalculiX oracle: `make test-ccx`, with ccx and gmsh")]
fn calculix_keeps_every_claim() {
    let reports: Vec<Vec<(String, bool)>> = std::thread::scope(|scope| {
        let running: Vec<_> = CASES.iter().map(|case| scope.spawn(move || {
            let e = read(case);
            let refs = (case.answers)(case, &e);
            judge(case, &e).into_iter().map(|(b, v)| {
                let r = refs.iter().find(|a| a.key() == b.key()).expect("an answer").reference;
                let n = b.count.map_or(String::new(), |n| format!(" {n}"));
                let line = format!("{:<12} {:<9} {:?}{n} = {v:.6e} ({:+.3}% of {r:.6e})",
                                   case.name, b.scenario, b.output, 100.0 * (v / r - 1.0));
                if b.holds(v) { (line, true) } else {
                    (format!("{line}: outside [{:?}, {:?}]", b.lo, b.hi), false)
                }
            }).collect()
        })).collect();
        running.into_iter().map(|t| t.join().unwrap()).collect()
    });
    let mut broken = Vec::new();
    for (line, kept) in reports.into_iter().flatten() {
        eprintln!("{line}");
        if !kept {
            broken.push(line);
        }
    }
    assert!(broken.is_empty(), "{}", broken.join("\n"));
}
