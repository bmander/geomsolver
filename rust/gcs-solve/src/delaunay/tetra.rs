//! A tetrahedron's measures, as an analysis judges a mesh by them (#510 P6): its circumsphere,
//! its six dihedral angles and the ratio of its circumradius to its shortest edge; and the
//! quadratic tetrahedron (TET10), whose Jacobian determinant is bounded over the whole element
//! by its Bernstein coefficients.
//!
//! A TET10's nodes are its four corners, then one node on each edge in the order CalculiX and
//! Abaqus number them (`EDGES`): 0–1, 1–2, 2–0, 0–3, 1–3, 2–3. In barycentric coordinates `λ` the
//! corner shape functions are `λᵢ(2λᵢ − 1)` and an edge's `4λᵢλⱼ`, so the place `x(λ)` is
//! quadratic and every column of its Jacobian `∂x/∂ξₖ` (`ξ = (λ₁, λ₂, λ₃)`) is linear in `λ`.
//! Their determinant is a cubic, and a cubic over a tetrahedron is a combination of the twenty
//! cubic Bernstein polynomials, each positive inside it and together summing to one: the least
//! coefficient bounds the determinant from below everywhere in the element, and the coefficients
//! at the corners are its values there (Johnen, Remacle and Geuzaine, "Geometrical validity of
//! curvilinear finite elements", 2013). All positive proves the element is not inverted anywhere.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::space::{sub,dot,cross,distance_squared as dist2};

type P = [f64;3];

/// The edges of a tetrahedron as its TET10 numbers their nodes, node `4 + k` on edge `EDGES[k]`.
pub const EDGES: [(usize,usize);6] = [(0,1),(1,2),(2,0),(0,3),(1,3),(2,3)];

/// Six times the signed volume: positive when `d` lies on the side of `abc` that `(b − a) × (c − a)`
/// points to, the orientation `predicates::orient` judges exactly.
pub fn volume6(t: [P;4]) -> f64 {
    let [a,b,c,d] = t;
    dot(sub(b,a),cross(sub(c,a),sub(d,a)))
}

/// The centre of the sphere through the four corners, and its radius; none for a flat one.
pub fn circumsphere(t: [P;4]) -> Option<(P,f64)> {
    let [a,b,c,d] = t;
    let (u,v,w) = (sub(b,a),sub(c,a),sub(d,a));
    let det = 2.*dot(u,cross(v,w));
    if !(det.abs() > 0.) { return None; }
    let (vw,wu,uv) = (cross(v,w),cross(w,u),cross(u,v));
    let (ru,rv,rw) = (dot(u,u),dot(v,v),dot(w,w));
    let o: P = std::array::from_fn(|k| (ru*vw[k]+rv*wu[k]+rw*uv[k])/det);
    let centre = [a[0]+o[0],a[1]+o[1],a[2]+o[2]];
    centre.iter().all(|x| x.is_finite()).then(|| (centre,dot(o,o).sqrt()))
}

/// The shortest of the six edges.
pub fn shortest_edge(t: [P;4]) -> f64 {
    EDGES.iter().map(|&(i,j)| dist2(t[i],t[j])).fold(f64::INFINITY,f64::min).sqrt()
}

/// The circumradius over the shortest edge: `1/√(8/3)` ≈ 0.61 for the regular tetrahedron, and
/// unbounded as one flattens, except for a sliver — four points near a circle's equator, its
/// ratio fair and its volume none — which only its dihedral angles see.
pub fn radius_edge(t: [P;4]) -> f64 {
    circumsphere(t).map_or(f64::INFINITY,|(_,r)| r/shortest_edge(t))
}

/// The dihedral angle along each edge, in degrees, in `EDGES`' order: the angle inside the
/// tetrahedron between the two faces that meet there. A flat tetrahedron has angles of 0 and 180.
pub fn dihedral_angles(t: [P;4]) -> [f64;6] {
    // The outward normal of the face opposite each corner.
    let normals: [P;4] = std::array::from_fn(|k| {
        let f: Vec<P> = (0..4).filter(|&j| j != k).map(|j| t[j]).collect();
        let n = cross(sub(f[1],f[0]),sub(f[2],f[0]));
        if dot(n,sub(t[k],f[0])) > 0. { n.map(|x| -x) } else { n }
    });
    EDGES.map(|(i,j)| {
        // The two faces at edge ij are those opposite the other two corners.
        let mut other = (0..4).filter(|&k| k != i && k != j);
        let (k,l) = (other.next().unwrap(),other.next().unwrap());
        let (m,n) = (normals[k],normals[l]);
        let l2 = dot(m,m)*dot(n,n);
        if !(l2 > 0.) { return 0.; }
        180.-(dot(m,n)/l2.sqrt()).clamp(-1.,1.).dacos().to_degrees()
    })
}

/// The least dihedral angle, in degrees.
pub fn least_dihedral(t: [P;4]) -> f64 { dihedral_angles(t).iter().copied().fold(180.,f64::min) }

/// The multi-indices of the cubic Bernstein polynomials over a tetrahedron, in a fixed order:
/// the four corners' first (`3eᵢ`, whose coefficient is the value there).
pub fn cubic_indices() -> [[u8;4];20] {
    let mut out = [[0u8;4];20];
    let mut n = 0;
    for i in 0..4 { let mut a = [0;4]; a[i] = 3; out[n] = a; n += 1; }
    for a in 0..=3u8 { for b in 0..=3-a { for c in 0..=3-a-b {
        let d = 3-a-b-c;
        let alpha = [a,b,c,d];
        if alpha.iter().any(|&x| x == 3) { continue; }
        out[n] = alpha;
        n += 1;
    } } }
    out
}

/// A quadratic tetrahedron: its ten nodes, corners first, then one an edge in `EDGES`' order.
#[derive(Clone,Copy,Debug)]
pub struct Tet10 { pub nodes: [P;10] }

impl Tet10 {
    /// The straight element on four corners: every edge node at its edge's middle.
    pub fn straight(t: [P;4]) -> Self {
        let mut nodes = [[0.;3];10];
        nodes[..4].copy_from_slice(&t);
        for (k,&(i,j)) in EDGES.iter().enumerate() { nodes[4+k] = std::array::from_fn(|c| 0.5*(t[i][c]+t[j][c])); }
        Self {nodes}
    }

    /// The node on edge `(i, j)`, either way round.
    fn edge(&self,i: usize,j: usize) -> P {
        let k = EDGES.iter().position(|&(a,b)| (a,b) == (i,j) || (a,b) == (j,i)).expect("an edge of the tetrahedron");
        self.nodes[4+k]
    }

    /// The place at barycentric `λ`.
    pub fn at(&self,l: [f64;4]) -> P {
        let mut x = [0.;3];
        for i in 0..4 { for c in 0..3 { x[c] += l[i]*(2.*l[i]-1.)*self.nodes[i][c]; } }
        for (k,&(i,j)) in EDGES.iter().enumerate() { for c in 0..3 { x[c] += 4.*l[i]*l[j]*self.nodes[4+k][c]; } }
        x
    }

    /// `∂x/∂λᵢ` at corner `v` (the derivative is linear in `λ`, so its corner values are its
    /// Bernstein coefficients): `3xᵢ` at its own corner, `4xᵢᵥ − xᵢ` at another.
    fn d_lambda(&self,i: usize,v: usize) -> P {
        let xi = self.nodes[i];
        if i == v { return xi.map(|x| 3.*x); }
        let m = self.edge(i,v);
        std::array::from_fn(|c| 4.*m[c]-xi[c])
    }

    /// The Bernstein coefficients of the Jacobian determinant `det ∂x/∂ξ`, in `cubic_indices`'
    /// order: `det J = Σ c_α B_α(λ)`. For the straight element every one is `6V`.
    pub fn jacobian_bernstein(&self) -> [f64;20] {
        // Column k of J at corner v: ∂x/∂λₖ − ∂x/∂λ₀ there (λ₀ = 1 − ξ₁ − ξ₂ − ξ₃).
        let col = |k: usize,v: usize| sub(self.d_lambda(k,v),self.d_lambda(0,v));
        let indices = cubic_indices();
        let slot = |a: [u8;4]| indices.iter().position(|&b| b == a).unwrap();
        let mut sum = [0.;20];
        // det J(λ) = Σ_{ijk} λᵢλⱼλₖ det(J₁(eᵢ), J₂(eⱼ), J₃(eₖ)), and λ^α = (α!/3!) B_α.
        for i in 0..4 { for j in 0..4 { for k in 0..4 {
            let mut a = [0u8;4];
            a[i] += 1; a[j] += 1; a[k] += 1;
            sum[slot(a)] += dot(col(1,i),cross(col(2,j),col(3,k)));
        } } }
        let factorial = |n: u8| [1.,1.,2.,6.][n as usize];
        std::array::from_fn(|s| sum[s]*indices[s].iter().map(|&n| factorial(n)).product::<f64>()/6.)
    }

    /// The element's volume, exactly: every cubic Bernstein polynomial integrates to a twentieth
    /// of the reference tetrahedron's sixth.
    pub fn volume(&self) -> f64 { self.jacobian_bernstein().iter().sum::<f64>()/120. }

    /// The least Bernstein coefficient over the straight element's Jacobian, `6V`: a lower bound
    /// on the least of `det J / 6V` over the element, 1 for a straight one. Positive proves the
    /// curved element is nowhere inverted.
    pub fn jacobian_ratio(&self) -> f64 {
        let straight = volume6([self.nodes[0],self.nodes[1],self.nodes[2],self.nodes[3]]);
        self.jacobian_bernstein().iter().copied().fold(f64::INFINITY,f64::min)/straight
    }
}
