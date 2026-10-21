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
use crate::space::{add,sub,dot,cross,scale,distance_squared as dist2};
use super::predicates::Weighted;

type P = [f64;3];

/// The edges of a tetrahedron as its TET10 numbers their nodes, node `4 + k` on edge `EDGES[k]`.
pub const EDGES: [(usize,usize);6] = [(0,1),(1,2),(2,0),(0,3),(1,3),(2,3)];

/// Six times the signed volume: positive when `d` lies on the side of `abc` that `(b − a) × (c − a)`
/// points to, the orientation `predicates::orient` judges exactly.
pub fn volume6(t: [P;4]) -> f64 {
    let [a,b,c,d] = t;
    dot(sub(b,a),cross(sub(c,a),sub(d,a)))
}

/// The centre of the sphere orthogonal to four weighted points, and the square of its radius
/// (`|x − p|² − w` equal at all four), in floating point and in the order given: the
/// circumsphere for zero weights. Infinite or NaN for a flat tetrahedron.
pub fn orthosphere(corners: [Weighted;4]) -> (P,f64) {
    let [a,b,c,d] = corners;
    let from_a = |q: Weighted| [0,1,2].map(|k| q.p[k]-a.p[k]);
    let (u,v,x) = (from_a(b),from_a(c),from_a(d));
    let (ru,rv,rx) = (dot(u,u)-(b.w-a.w),dot(v,v)-(c.w-a.w),dot(x,x)-(d.w-a.w));
    let (vx,xu,uv) = (cross(v,x),cross(x,u),cross(u,v));
    let det = 2.*dot(u,vx);
    let o: P = std::array::from_fn(|k| (ru*vx[k]+rv*xu[k]+rx*uv[k])/det);
    ([a.p[0]+o[0],a.p[1]+o[1],a.p[2]+o[2]],dot(o,o)-a.w)
}

/// The centre of the sphere through the four corners, and its radius; none for a flat one.
pub fn circumsphere(t: [P;4]) -> Option<(P,f64)> {
    let (centre,r2) = orthosphere(t.map(|p| Weighted {p,w:0.}));
    (centre.iter().all(|x| x.is_finite()) && r2.is_finite()).then(|| (centre,r2.sqrt()))
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

/// The cosine between the outward normals of the two faces at each edge, in `EDGES`' order: the
/// dihedral angle there is 180° less its arccosine. 1 where a face has no area.
fn normal_cosines(t: [P;4]) -> [f64;6] {
    // The outward normal of the face opposite each corner.
    let normals: [P;4] = std::array::from_fn(|k| {
        let f = [t[(k+1)%4],t[(k+2)%4],t[(k+3)%4]];
        let n = cross(sub(f[1],f[0]),sub(f[2],f[0]));
        if dot(n,sub(t[k],f[0])) > 0. { n.map(|x| -x) } else { n }
    });
    // The two faces at edge ij are those opposite the other two corners.
    EDGES.map(|(i,j)| {
        let (k,l) = (OTHERS[i][j],6-i-j-OTHERS[i][j]);
        let (m,n) = (normals[k],normals[l]);
        let l2 = dot(m,m)*dot(n,n);
        if l2 > 0. { (dot(m,n)/l2.sqrt()).clamp(-1.,1.) } else { 1. }
    })
}

/// For corners `i ≠ j`, one of the other two (the other is `6 − i − j` less it).
const OTHERS: [[usize;4];4] = [[0,2,1,1],[2,0,0,0],[1,0,0,0],[1,0,0,0]];

/// The dihedral angle along each edge, in degrees, in `EDGES`' order: the angle inside the
/// tetrahedron between the two faces that meet there. A flat tetrahedron has angles of 0 and 180.
pub fn dihedral_angles(t: [P;4]) -> [f64;6] { normal_cosines(t).map(|c| 180.-c.dacos().to_degrees()) }

/// The least dihedral angle, in degrees: one arccosine, of the least cosine.
pub fn least_dihedral(t: [P;4]) -> f64 {
    180.-normal_cosines(t).iter().copied().fold(1.,f64::min).dacos().to_degrees()
}

/// The multi-indices of the cubic Bernstein polynomials over a tetrahedron, in a fixed order:
/// the four corners' first (`3eᵢ`, whose coefficient is the value there).
pub const CUBIC: [[u8;4];20] = [[3,0,0,0],[0,3,0,0],[0,0,3,0],[0,0,0,3],
    [0,0,1,2],[0,0,2,1],[0,1,0,2],[0,1,1,1],[0,1,2,0],[0,2,0,1],[0,2,1,0],[1,0,0,2],
    [1,0,1,1],[1,0,2,0],[1,1,0,1],[1,1,1,0],[1,2,0,0],[2,0,0,1],[2,0,1,0],[2,1,0,0]];

/// The node on each edge, by its two corners: `4 +` its place in `EDGES` (the diagonal unused).
const EDGE_NODE: [[usize;4];4] = [[0,4,6,7],[4,0,5,8],[6,5,0,9],[7,8,9,0]];

/// A quadratic tetrahedron: its ten nodes, corners first, then one an edge in `EDGES`' order.
#[derive(Clone,Copy,Debug)]
pub struct Tet10 { pub nodes: [P;10] }

impl Tet10 {
    /// The straight element on four corners: every edge node at its edge's middle, the same bits
    /// whichever way round the edge is taken.
    pub fn straight(t: [P;4]) -> Self {
        let mut nodes = [[0.;3];10];
        nodes[..4].copy_from_slice(&t);
        for (k,&(i,j)) in EDGES.iter().enumerate() { nodes[4+k] = scale(add(t[i],t[j]),0.5); }
        Self {nodes}
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
        let m = self.nodes[EDGE_NODE[i][v]];
        std::array::from_fn(|c| 4.*m[c]-xi[c])
    }

    /// The Bernstein coefficients of the Jacobian determinant `det ∂x/∂ξ`, in `CUBIC`'s order:
    /// `det J = Σ c_α B_α(λ)`. For the straight element every one is `6V`.
    pub fn jacobian_bernstein(&self) -> [f64;20] {
        // Column k of J at corner v: ∂x/∂λₖ − ∂x/∂λ₀ there (λ₀ = 1 − ξ₁ − ξ₂ − ξ₃).
        let col: [[P;4];3] = std::array::from_fn(|k| std::array::from_fn(|v| sub(self.d_lambda(k+1,v),self.d_lambda(0,v))));
        let mut sum = [0.;20];
        // det J(λ) = Σ_{ijk} λᵢλⱼλₖ det(J₁(eᵢ), J₂(eⱼ), J₃(eₖ)), and λ^α = (α!/3!) B_α.
        for j in 0..4 { for k in 0..4 {
            let jk = cross(col[1][j],col[2][k]);
            for i in 0..4 {
                let mut a = [0u8;4];
                a[i] += 1; a[j] += 1; a[k] += 1;
                let slot = CUBIC.iter().position(|&b| b == a).unwrap();
                sum[slot] += dot(col[0][i],jk);
            }
        } }
        let factorial = |n: u8| [1.,1.,2.,6.][n as usize];
        std::array::from_fn(|s| sum[s]*CUBIC[s].iter().map(|&n| factorial(n)).product::<f64>()/6.)
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
