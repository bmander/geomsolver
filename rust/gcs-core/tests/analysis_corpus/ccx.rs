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
use gcs_core::model::{EntKind, EntRef, Hold, Scenario};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

/// How finely each case is meshed: the largest element (millimetres, as the B-rep is) and gmsh's
/// elements a full turn of a curve.
fn fineness(name: &str) -> (f64, u32) {
    match name {
        "cantilever" => (6.0, 0),
        "column" => (4.0, 0),
        "lame" => (3.0, 72),
        "kirsch" => (8.0, 180),
        "plate_modes" => (20.0, 0),
        "nafems_le1" => (40.0, 120),
        "nafems_fv32" => (150.0, 0),
        other => panic!("no fineness for `{other}`"),
    }
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
        let mut seen: BTreeMap<Vec<usize>, (usize, Facet)> = BTreeMap::new();
        let mut add = |nodes: Vec<usize>, inside: usize, corners: usize| {
            let mut key = nodes[..corners].to_vec();
            key.sort();
            seen.entry(key).or_insert((0, Facet { nodes, inside })).0 += 1;
        };
        for el in &self.elements {
            if self.plane {
                for (f, o) in TRI {
                    add(f.iter().map(|&k| el[k]).collect(), el[o], 2);
                }
            } else {
                for (f, o) in TET {
                    add(f.iter().map(|&k| el[k]).collect(), el[o], 3);
                }
            }
        }
        seen.into_values().filter(|(n, _)| *n == 1).map(|(_, f)| f).collect()
    }

    /// Every midside node is near the middle of its edge: the element's order is Abaqus's.
    fn check_order(&self) {
        let pairs: &[(usize, usize, usize)] = if self.plane {
            &[(0, 1, 3), (1, 2, 4), (2, 0, 5)]
        } else {
            &[(0, 1, 4), (1, 2, 5), (2, 0, 6), (0, 3, 7), (1, 3, 8), (2, 3, 9)]
        };
        for el in &self.elements {
            for &(a, b, m) in pairs {
                let (pa, pb, pm) = (self.nodes[el[a]], self.nodes[el[b]], self.nodes[el[m]]);
                let mid = std::array::from_fn(|k| 0.5 * (pa[k] + pb[k]));
                assert!(dist(mid, pm) < 0.25 * dist(pa, pb), "a midside node off its edge: {el:?}");
            }
        }
    }
}

fn dist(a: V, b: V) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// A quadratic triangle's shape functions and their derivatives in `(r, s)`, nodes as `Facet`.
fn tri6(r: f64, s: f64) -> ([f64; 6], [[f64; 2]; 6]) {
    let (l1, l2, l3) = (1.0 - r - s, r, s);
    let n = [
        l1 * (2.0 * l1 - 1.0),
        l2 * (2.0 * l2 - 1.0),
        l3 * (2.0 * l3 - 1.0),
        4.0 * l1 * l2,
        4.0 * l2 * l3,
        4.0 * l3 * l1,
    ];
    let d = [
        [-(4.0 * l1 - 1.0), -(4.0 * l1 - 1.0)],
        [4.0 * l2 - 1.0, 0.0],
        [0.0, 4.0 * l3 - 1.0],
        [4.0 * (l1 - l2), -4.0 * l2],
        [4.0 * l3, 4.0 * l2],
        [-4.0 * l3, 4.0 * (l1 - l3)],
    ];
    (n, d)
}

/// Strang and Fix's six points on the reference triangle, exact to degree four, weights summing
/// to its area.
fn tri_rule() -> Vec<(f64, f64, f64)> {
    let a = (0.445948490915965, 0.223381589678011);
    let b = (0.091576213509771, 0.109951743655322);
    let mut out = Vec::new();
    for (x, w) in [a, b] {
        for (r, s) in [(x, x), (1.0 - 2.0 * x, x), (x, 1.0 - 2.0 * x)] {
            out.push((r, s, 0.5 * w));
        }
    }
    out
}

/// `∫ N_i n dA` over a facet, for each of its nodes: `n` its outward normal (out of the element),
/// `dA` its area — a plane mesh's edges times the thickness.  A uniform traction `t` lays `t`
/// times the area part on each node, and a pressure `p` lays `−p` times the vector part.
fn integrals(mesh: &Mesh, f: &Facet, thickness: f64) -> Vec<(f64, V)> {
    let x = |k: usize| mesh.nodes[f.nodes[k]];
    let mut out = vec![(0.0, [0.0; 3]); f.nodes.len()];
    let away = |n: V, at: V| if dot(n, sub(at, mesh.nodes[f.inside])) < 0.0 { -1.0 } else { 1.0 };
    if mesh.plane {
        let g = (3.0f64 / 5.0).sqrt();
        for (xi, w) in [(-g, 5.0 / 9.0), (0.0, 8.0 / 9.0), (g, 5.0 / 9.0)] {
            let n = [xi * (xi - 1.0) / 2.0, xi * (xi + 1.0) / 2.0, 1.0 - xi * xi];
            let dn = [xi - 0.5, xi + 0.5, -2.0 * xi];
            let mut at = [0.0; 3];
            let mut t = [0.0; 3];
            for k in 0..3 {
                for c in 0..3 {
                    at[c] += n[k] * x(k)[c];
                    t[c] += dn[k] * x(k)[c];
                }
            }
            // the profile lies in x and z: the edge's normal in it, its length times the depth
            let normal = [t[2], 0.0, -t[0]];
            let sign = away(normal, at);
            let len = (t[0] * t[0] + t[2] * t[2]).sqrt();
            for k in 0..3 {
                out[k].0 += w * n[k] * len * thickness;
                for c in 0..3 {
                    out[k].1[c] += w * n[k] * sign * normal[c] * thickness;
                }
            }
        }
    } else {
        for (r, s, w) in tri_rule() {
            let (n, d) = tri6(r, s);
            let (mut at, mut xr, mut xs) = ([0.0; 3], [0.0; 3], [0.0; 3]);
            for k in 0..6 {
                for c in 0..3 {
                    at[c] += n[k] * x(k)[c];
                    xr[c] += d[k][0] * x(k)[c];
                    xs[c] += d[k][1] * x(k)[c];
                }
            }
            let normal = cross(xr, xs);
            let sign = away(normal, at);
            let area = dot(normal, normal).sqrt();
            for k in 0..6 {
                out[k].0 += w * n[k] * area;
                for c in 0..3 {
                    out[k].1[c] += w * n[k] * sign * normal[c];
                }
            }
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
    let (n, _) = tri6(1.0 / 3.0, 1.0 / 3.0);
    let mut at = [0.0; 3];
    for k in 0..6 {
        for c in 0..3 {
            at[c] += n[k] * mesh.nodes[f.nodes[k]][c];
        }
    }
    at
}

/// How far `p` is from face `f`'s surface.
fn off(x: &gcs_core::solid::Exact, f: usize, p: V) -> f64 {
    let s = &x.brep.faces[f].surface;
    dist(s.point(s.inverse(p)), p)
}

/// The face of the solid each boundary facet lies on, by its own name (`root` of `beam.root`;
/// every case is one primitive): the nearest surface to
/// the facet's middle, every node of it on that surface.  A plane mesh's caps — the faces the
/// whole profile lies in — are no facet's.
fn classify(x: &gcs_core::solid::Exact, mesh: &Mesh, facets: &[Facet]) -> Vec<String> {
    assert!(x.named.iter().all(|n| n.0 == x.named[0].0), "the oracle reads one primitive's faces");
    let tol = 1e-5 * x.brep.size();
    let probes = [0, mesh.nodes.len() / 2, mesh.nodes.len() - 1];
    let faces: Vec<usize> = (0..x.brep.faces.len())
        .filter(|&f| !mesh.plane || !probes.iter().all(|&n| off(x, f, mesh.nodes[n]) < tol))
        .collect();
    facets.iter().map(|fa| {
        let mid = facet_middle(mesh, fa);
        let &f = faces.iter().min_by(|&&a, &&b| off(x, a, mid).total_cmp(&off(x, b, mid))).unwrap();
        for &n in &fa.nodes {
            // a node on a curved face is placed on it by gmsh, to the STEP's precision
            assert!(off(x, f, mesh.nodes[n]) < 1e3 * tol, "a facet off `{}`", x.brep.faces[f].name);
        }
        // the face's own name in the primitive it is a piece of, as a face path names it
        x.named[f].1.clone()
    }).collect()
}

/// The B-rep's frame, millimetres about the solid's origin, of a point of the drawing.
fn framed(e: &Elaborated, x: &gcs_core::solid::Exact, p: EntRef) -> V {
    let w = e.sketch.world_point(p.i());
    std::array::from_fn(|k| (w[k] - x.origin[k]) * x.mm)
}

/// The node nearest a point, in the profile's plane where the mesh is one.
fn nearest(mesh: &Mesh, p: V) -> usize {
    let flat = |q: V| if mesh.plane { [q[0], 0.0, q[2]] } else { q };
    (0..mesh.nodes.len()).min_by(|&a, &b| {
        dist(flat(mesh.nodes[a]), flat(p)).total_cmp(&dist(flat(mesh.nodes[b]), flat(p)))
    }).unwrap()
}

/// The world axis an `along:` names: its index, 0 for x.
fn axis_of(sk: &Sketch, d: EntRef) -> usize {
    assert_eq!(d.kind, EntKind::Axis, "the oracle reads `along:` an axis");
    let dir = sk.axes[d.i()].d.map(|p| sk.params[p as usize].value);
    let k = (0..3).max_by(|&a, &b| dir[a].abs().total_cmp(&dir[b].abs())).unwrap();
    assert!((dir[k].abs() - 1.0).abs() < 1e-12, "the oracle holds along the world's axes: {dir:?}");
    k
}

/// The unit direction an `along:` names.
fn direction(sk: &Sketch, d: EntRef) -> V {
    let dir = sk.axes[d.i()].d.map(|p| sk.params[p as usize].value);
    let n = dot(dir, dir).sqrt();
    dir.map(|c| c / n)
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
    let nodes_of = |a: &Applied| -> Vec<usize> {
        match a {
            Applied::Face(f) => {
                let name = f.path.last().unwrap();
                let mut ns: Vec<usize> = facets.iter().zip(on).filter(|(_, n)| *n == name)
                    .flat_map(|(fa, _)| fa.nodes.clone()).collect();
                ns.sort();
                ns.dedup();
                assert!(!ns.is_empty(), "no facet on `{name}`");
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
        // in the plane, nothing moves across it: a hold across the thickness is the solid's
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
        match l.kind {
            LoadKind::Force => {
                let total: V = match &l.value {
                    LoadValue::Vector(v) => v.map(|c| si(c, l.kind.dim())),
                    LoadValue::Along { magnitude, along } => {
                        direction(sk, *along).map(|c| c * si(*magnitude, l.kind.dim()))
                    }
                    LoadValue::Scalar(_) => unreachable!(),
                };
                match &l.at {
                    Applied::Face(f) => {
                        // a uniform traction: the force over the face's area
                        let name = f.path.last().unwrap();
                        let mine: Vec<_> = facets.iter().zip(on).filter(|(_, n)| *n == name)
                            .map(|(f, _)| f).collect();
                        let parts: Vec<_> =
                            mine.iter().map(|f| integrals(mesh, f, thickness)).collect();
                        let area: f64 = parts.iter().flatten().map(|(a, _)| a).sum();
                        for (f, part) in mine.iter().zip(&parts) {
                            for (k, (a, _)) in part.iter().enumerate() {
                                lay_on(f.nodes[k], total.map(|c| c * a / area));
                            }
                        }
                    }
                    a => {
                        for n in nodes_of(a) {
                            lay_on(n, total);
                        }
                    }
                }
            }
            LoadKind::Pressure => {
                let (Applied::Face(f), LoadValue::Scalar(p)) = (&l.at, &l.value) else {
                    unreachable!()
                };
                let p = si(*p, l.kind.dim());
                let name = f.path.last().unwrap();
                for (fa, _) in facets.iter().zip(on).filter(|(_, n)| *n == name) {
                    for (k, (_, nv)) in integrals(mesh, fa, thickness).into_iter().enumerate() {
                        // the integrals are in square millimetres
                        lay_on(fa.nodes[k], nv.map(|c| -p * c * 1e-6));
                    }
                }
            }
            LoadKind::Acceleration => panic!("the oracle lays no body load yet"),
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
        Output::Deflection | Output::Safety | Output::Compliance => Some(Kind::Static),
        Output::Mode => Some(Kind::Frequency),
        Output::Buckling => Some(Kind::Buckle),
        Output::Mass => None,
    }
}

/// The deck: the mesh in metres, the material in SI, the section, the supports, and one step.
fn deck(case: &Case, sk: &Sketch, mesh: &Mesh, laid: &Laid, thickness: f64, k: Kind, modes: u32)
    -> String {
    use std::fmt::Write;
    let mut d = String::from("*HEADING\nSolvent's verification corpus\n*NODE, NSET=NALL\n");
    for (i, p) in mesh.nodes.iter().enumerate() {
        let [x, y, z] = p.map(|c| c * 1e-3);
        writeln!(d, "{}, {x:.12e}, {y:.12e}, {z:.12e}", i + 1).unwrap();
    }
    let ty = if mesh.plane { "CPS6" } else { "C3D10" };
    writeln!(d, "*ELEMENT, TYPE={ty}, ELSET=EALL").unwrap();
    for (i, el) in mesh.elements.iter().enumerate() {
        let ns: Vec<String> = el.iter().map(|n| (n + 1).to_string()).collect();
        writeln!(d, "{}, {}", i + 1, ns.join(", ")).unwrap();
    }
    let name = sk.solids[solid(sk, case.solid)].material.clone().unwrap();
    let m = &sk.materials[&name];
    let (young, nu) = (m.get(Property::E).unwrap(), m.get(Property::Nu).unwrap());
    writeln!(d, "*MATERIAL, NAME=M\n*ELASTIC\n{young:.12e}, {nu}").unwrap();
    if let Some(rho) = m.get(Property::Rho) {
        writeln!(d, "*DENSITY\n{rho:.12e}").unwrap();
    }
    if mesh.plane {
        writeln!(d, "*SOLID SECTION, ELSET=EALL, MATERIAL=M\n{:.12e}", thickness * 1e-3).unwrap();
    } else {
        d.push_str("*SOLID SECTION, ELSET=EALL, MATERIAL=M\n");
    }
    // the deck's own axes: a plane mesh's z is CalculiX's y (its y, across the thickness, is
    // never held or loaded)
    let dof = |c: usize| if mesh.plane { [1, 0, 2][c] } else { c + 1 };
    d.push_str("*BOUNDARY\n");
    for (n, h) in &laid.held {
        for c in (0..3).filter(|&c| h[c]) {
            writeln!(d, "{}, {}, {}", n + 1, dof(c), dof(c)).unwrap();
        }
    }
    match k {
        Kind::Static => d.push_str("*STEP\n*STATIC\n"),
        Kind::Frequency => writeln!(d, "*STEP\n*FREQUENCY, SOLVER=SPOOLES\n{modes}").unwrap(),
        Kind::Buckle => d.push_str("*STEP\n*BUCKLE, SOLVER=SPOOLES\n4\n"),
    }
    if k != Kind::Frequency {
        d.push_str("*CLOAD\n");
        for (n, f) in &laid.force {
            for c in (0..3).filter(|&c| f[c] != 0.0 && !(mesh.plane && c == 1)) {
                writeln!(d, "{}, {}, {:.12e}", n + 1, dof(c), f[c]).unwrap();
            }
        }
    }
    if k == Kind::Static {
        d.push_str("*NODE FILE\nU\n*EL FILE\nS\n");
    }
    d.push_str("*END STEP\n");
    d
}

/// A plane mesh's nodes as CalculiX reads them: z turned into y.
fn turned(mesh: &Mesh) -> Mesh {
    Mesh {
        nodes: mesh.nodes.iter().map(|p| if mesh.plane { [p[0], p[2], 0.0] } else { *p }).collect(),
        elements: mesh.elements.clone(),
        plane: mesh.plane,
    }
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
    let step = gcs_core::brep::step::write(&x.brep, case.name, 1e-4).expect("STEP");
    std::fs::write(dir.join("solid.step"), step).unwrap();
    let (size, curvature) = fineness(case.name);
    let plane = case.reading == Reading::Plane;
    let (lo, hi) = x.brep.bounds();
    let e = 1e-3 * x.brep.size();
    let group = if plane {
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
    run(dir, "gmsh", &["mesh.geo", dim, "-format", "inp", "-o", "mesh.inp", "-v", "2"]);
    read_inp(&std::fs::read_to_string(dir.join("mesh.inp")).unwrap(), plane)
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

/// The last block of a result in CalculiX's `.frd` (`DISP`, `STRESS`), node by node.
fn frd(text: &str, name: &str) -> BTreeMap<usize, Vec<f64>> {
    let mut out = BTreeMap::new();
    let mut inside = false;
    for l in text.lines() {
        if l.starts_with(" -4") {
            inside = l[3..].trim_start().starts_with(name);
            if inside {
                out.clear();
            }
            continue;
        }
        if inside && l.starts_with(" -1") {
            let node: usize = l[3..13].trim().parse().unwrap();
            let vals: Vec<f64> = (13..l.len()).step_by(12)
                .map(|k| l[k..(k + 12).min(l.len())].trim().parse().unwrap())
                .collect();
            out.insert(node, vals);
        }
        if l.starts_with(" -3") {
            inside = false;
        }
    }
    assert!(!out.is_empty(), "no {name} in the .frd");
    out
}

/// The displacement at a point, metres: the shape functions of the element that holds it.
fn displacement(mesh: &Mesh, u: &BTreeMap<usize, Vec<f64>>, p: V) -> V {
    let corners = if mesh.plane { 3 } else { 4 };
    // each element's barycentric coordinates of `p` from its corners, and how far outside it is
    let bary = |el: &Vec<usize>| -> Vec<f64> {
        let c: Vec<V> = (0..corners).map(|k| mesh.nodes[el[k]]).collect();
        if mesh.plane {
            let (a, b) = (sub(c[1], c[0]), sub(c[2], c[0]));
            let q = sub(p, c[0]);
            let det = a[0] * b[2] - a[2] * b[0];
            let l2 = (q[0] * b[2] - q[2] * b[0]) / det;
            let l3 = (a[0] * q[2] - a[2] * q[0]) / det;
            vec![1.0 - l2 - l3, l2, l3]
        } else {
            let (a, b, cc) = (sub(c[1], c[0]), sub(c[2], c[0]), sub(c[3], c[0]));
            let q = sub(p, c[0]);
            let det = dot(a, cross(b, cc));
            let l2 = dot(q, cross(b, cc)) / det;
            let l3 = dot(a, cross(q, cc)) / det;
            let l4 = dot(a, cross(b, q)) / det;
            vec![1.0 - l2 - l3 - l4, l2, l3, l4]
        }
    };
    let outside = |l: &Vec<f64>| l.iter().fold(0.0f64, |m, &x| m.max(-x));
    let el = mesh.elements.iter()
        .min_by(|a, b| outside(&bary(a)).total_cmp(&outside(&bary(b))))
        .unwrap();
    let l = bary(el);
    assert!(outside(&l) < 1e-6, "{p:?} is in no element");
    let n: Vec<f64> = if mesh.plane {
        let mut n: Vec<f64> = (0..3).map(|k| l[k] * (2.0 * l[k] - 1.0)).collect();
        for (a, b) in [(0, 1), (1, 2), (2, 0)] {
            n.push(4.0 * l[a] * l[b]);
        }
        n
    } else {
        let mut n: Vec<f64> = (0..4).map(|k| l[k] * (2.0 * l[k] - 1.0)).collect();
        for (a, b) in [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)] {
            n.push(4.0 * l[a] * l[b]);
        }
        n
    };
    let mut out = [0.0; 3];
    for (k, &node) in el.iter().enumerate() {
        let v = &u[&(node + 1)];
        // a plane deck's y is the drawing's z
        let v: V = if mesh.plane { [v[0], 0.0, v[1]] } else { [v[0], v[1], v[2]] };
        for c in 0..3 {
            out[c] += n[k] * v[c];
        }
    }
    out
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

/// **The oracle's value of every bracket a case claims**, in the document's own units.
fn judge(case: &Case) -> Vec<(Bracket, f64)> {
    let e = read(case);
    let sk = &e.sketch;
    let x = exact(case, sk);
    let dir = scratch(case);
    let m = mesh(case, &x, &dir);
    let facets = m.boundary();
    let on = classify(&x, &m, &facets);
    let thickness = extent(&x)[1] * x.mm;
    let doc = |v: f64, dim: Dim| v / sk.units.to_si(1.0, dim);
    let mut out = Vec::new();
    let bs = brackets(&e);
    let scenarios: Vec<&Scenario> = e.scenarios.iter().map(|s| &s.scenario).collect();
    for s in &scenarios {
        let mine: Vec<&Bracket> = bs.iter().filter(|b| b.scenario == s.name).collect();
        let laid = lay(&e, &x, s, &m, &facets, &on, thickness);
        let mut kinds: Vec<Kind> = mine.iter().filter_map(|b| kind(b.output)).collect();
        kinds.sort();
        kinds.dedup();
        let mut results: BTreeMap<Kind, String> = BTreeMap::new();
        let deck_mesh = turned(&m);
        for k in kinds {
            let modes = mine.iter().filter_map(|b| b.count).max().unwrap_or(1) + 4;
            let job = format!("{}_{:?}", s.name, k).to_lowercase();
            let text = deck(case, sk, &deck_mesh, &laid, thickness, k, modes);
            std::fs::write(dir.join(format!("{job}.inp")), text).unwrap();
            run(&dir, "ccx", &[&job]);
            results.insert(k, job);
        }
        for b in mine {
            let file = |ext: &str, k: Kind| {
                std::fs::read_to_string(dir.join(format!("{}.{ext}", results[&k]))).unwrap()
            };
            let v = match b.output {
                Output::Deflection => {
                    let Applied::Ent(p) = &b.of else { panic!("a deflection at a point") };
                    let u = frd(&file("frd", Kind::Static), "DISP");
                    let u = displacement(&m, &u, framed(&e, &x, *p));
                    let along = b.along.map(|d| direction(sk, d));
                    doc(along.map_or(dot(u, u).sqrt(), |d| dot(u, d)), Dim::LENGTH)
                }
                Output::Safety => {
                    let name = sk.solids[solid(sk, case.solid)].material.clone().unwrap();
                    let y = sk.materials[&name].get(Property::Yield).unwrap();
                    y / mises(&frd(&file("frd", Kind::Static), "STRESS"))
                }
                Output::Mode => {
                    let heading = "E I G E N V A L U E   O U T P U T";
                    let rows = table(&file("dat", Kind::Frequency), heading, 5);
                    rows[b.count.unwrap() as usize - 1][3]
                }
                Output::Buckling => {
                    let heading = "B U C K L I N G   F A C T O R   O U T P U T";
                    let rows = table(&file("dat", Kind::Buckle), heading, 2);
                    rows[0][1]
                }
                Output::Mass => {
                    // the B-rep's volume, not CalculiX's: the one output with no analysis in it
                    let name = sk.solids[solid(sk, case.solid)].material.clone().unwrap();
                    let rho = sk.materials[&name].get(Property::Rho).unwrap();
                    rho * gcs_core::brep::props::volume(&x.brep) * 1e-9
                }
                Output::Compliance => panic!("no case claims a compliance yet"),
            };
            out.push((b.clone(), v));
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
    out
}

/// The answer a bracket is about.
fn bracket_answer<'a>(refs: &'a [Answer], b: &Bracket) -> &'a Answer {
    let key = (b.scenario.as_str(), b.output, b.count);
    refs.iter().find(|a| (a.scenario, a.output, a.count) == key).expect("an answer")
}

/// **CalculiX keeps every claim the corpus states.**  Each case's answer by an independent solver,
/// on a mesh this kernel's solid gives, inside the bracket the document claims — the evidence
/// that the documents say what their references say, and what #510's own analysis will be held
/// to beside them.
#[test]
#[cfg_attr(not(feature = "ccx"),
           ignore = "the CalculiX oracle: `make test-ccx`, with ccx and gmsh")]
fn calculix_keeps_every_claim() {
    let mut broken = Vec::new();
    for case in &CASES {
        let refs = answers(case, &read(case));
        for (b, v) in judge(case) {
            let r = bracket_answer(&refs, &b).reference;
            let n = b.count.map_or(String::new(), |n| format!(" {n}"));
            let line = format!("{:<12} {:<9} {:?}{n} = {v:.6e} ({:+.3}% of {r:.6e})", case.name,
                               b.scenario, b.output, 100.0 * (v / r - 1.0));
            eprintln!("{line}");
            if !b.holds(v) {
                broken.push(format!("{line}: outside [{:?}, {:?}]", b.lo, b.hi));
            }
        }
    }
    assert!(broken.is_empty(), "{}", broken.join("\n"));
}
