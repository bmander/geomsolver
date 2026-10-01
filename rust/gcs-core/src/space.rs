//! Points and triangles in space: the vector arithmetic and the few triangle measures the core,
//! the CLI's native construction and their tests share, written once.
//!
//! Each helper is one fixed expression, and callers rely on its bits: a mesh is compared
//! byte for byte across refactors, so a helper is not rewritten into an equal-in-exact-arithmetic
//! form (`norm` is `sqrt(dot)`, and `length` — by `hypot` — is a different number).

#[allow(unused_imports)]
use crate::fmath::Det;
type V3 = [f64;3];

pub fn sub(a: V3,b: V3) -> V3 { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
pub fn add(a: V3,b: V3) -> V3 { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
pub fn scale(a: V3,s: f64) -> V3 { a.map(|x| x*s) }
pub fn dot(a: V3,b: V3) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
pub fn cross(a: V3,b: V3) -> V3 { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
/// The Euclidean length, as `sqrt(dot(a, a))`.
pub fn norm(a: V3) -> f64 { dot(a,a).sqrt() }
/// The Euclidean length by `hypot`, which neither overflows nor underflows where the squares
/// would: not bit-identical to `norm`.
pub fn length(a: V3) -> f64 { a[0].dhypot(a[1]).dhypot(a[2]) }
pub fn distance(a: V3,b: V3) -> f64 { norm(sub(a,b)) }
pub fn distance_squared(a: V3,b: V3) -> f64 { let d = sub(a,b); dot(d,d) }
/// The unit vector along `a`, or none for a zero vector.
pub fn normalised(a: V3) -> Option<V3> { let l = norm(a); (l > 0.).then(|| a.map(|x| x/l)) }
/// The point a fraction `t` of the way from `a` to `b`, as `a + t (b − a)`.
pub fn lerp(a: V3,b: V3,t: f64) -> V3 { [a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),a[2]+t*(b[2]-a[2])] }

/// The nearest point of the segment `ab` to `p` (`a` for a segment of no length).
pub fn closest_on_segment(p: V3,a: V3,b: V3) -> V3 {
    let (d,w) = (sub(b,a),sub(p,a));
    let l = dot(d,d);
    let s = if l > 0. { (dot(w,d)/l).clamp(0.,1.) } else { 0. };
    [a[0]+s*d[0],a[1]+s*d[1],a[2]+s*d[2]]
}
/// The distance from `p` to the segment `ab`.
pub fn segment_distance(p: V3,a: V3,b: V3) -> f64 { distance(p,closest_on_segment(p,a,b)) }
/// A polyline's length: the sum of its sides, in order.
pub fn polyline_length(c: &[V3]) -> f64 { c.windows(2).map(|w| distance(w[0],w[1])).sum::<f64>() }

/// The point of a triangle's plane of equal power `|x − p|² − w` to its three weighted corners
/// (for zero weights, the circumcentre), with the triangle's normal `(b − a) × (c − a)`; none
/// for a triangle with no area.
pub fn orthocentre(a: V3,b: V3,c: V3,w: [f64;3]) -> Option<(V3,V3)> {
    let (u,v) = (sub(b,a),sub(c,a));
    let n = cross(u,v);
    let nn = dot(n,n);
    if !(nn > 0.) { return None; }
    // a + (ru v × n + rv n × u) / (2 |n|²), with ru = |u|² − (wb − wa), rv = |v|² − (wc − wa)
    let (ru,rv) = (dot(u,u)-(w[1]-w[0]),dot(v,v)-(w[2]-w[0]));
    let (vn,nu) = (cross(v,n),cross(n,u));
    Some((std::array::from_fn(|k| a[k]+(ru*vn[k]+rv*nu[k])/(2.*nn)),n))
}
/// The centre of the circle through a triangle's corners, or none for a triangle with no area.
pub fn circumcentre(a: V3,b: V3,c: V3) -> Option<V3> { orthocentre(a,b,c,[0.;3]).map(|(o,_)| o) }

/// A box's centre and the length of its diagonal (`[Interval; 3]`, as the fields bound things).
pub fn box_centre_diagonal(b: &[crate::interval::Interval;3]) -> (V3,f64) {
    let [lo,hi] = [0,1].map(|k| b.map(|x| x.bounds()[k]));
    (std::array::from_fn(|k| 0.5*(lo[k]+hi[k])),(0..3).map(|k| (hi[k]-lo[k])*(hi[k]-lo[k])).sum::<f64>().sqrt())
}

/// The unit normal of a triangle by its winding, or none.
pub fn triangle_normal(a: V3,b: V3,c: V3) -> Option<V3> { normalised(cross(sub(b,a),sub(c,a))) }

/// Whether a triangle is flat beyond what rounding can decide: its least height no more than a
/// ten-billionth of the size of its coordinates and its longest side. The tracer's points carry
/// about 1e-12 of noise at unit coordinates, so a triangle made flat exactly (three points of a
/// fixed point of the motion) and the same triangle moved by that noise are treated alike.
pub fn degenerate(a: V3,b: V3,c: V3) -> bool {
    let magnitude = [a,b,c].iter().flatten().fold(0_f64,|m,x| m.max(x.abs()));
    let longest = distance(a,b).max(distance(b,c)).max(distance(c,a));
    altitude(a,b,c) <= 1e-10*(magnitude+longest)
}

/// The unit normal of a triangle by its winding, or none for a `degenerate` one: the normal a
/// construction decides by.
pub fn stable_normal(a: V3,b: V3,c: V3) -> Option<V3> { if degenerate(a,b,c) { None } else { triangle_normal(a,b,c) } }

/// A triangle's least height: twice its area over its longest side, zero for a triangle with
/// no area.
pub fn altitude(a: V3,b: V3,c: V3) -> f64 {
    let area2 = norm(cross(sub(b,a),sub(c,a)));
    let longest = distance(a,b).max(distance(b,c)).max(distance(c,a));
    if area2 > 0. && longest > 0. { area2/longest } else { 0. }
}

/// Which feature of a triangle its nearest point to a query lies on.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Region { Vertex, Edge, Face }

/// The nearest point of a triangle to `p` and the feature it lies on
/// (Ericson's Voronoi-region walk).
pub fn closest_on_triangle(p: V3,a: V3,b: V3,c: V3) -> (V3,Region) {
    let (ab,ac,ap) = (sub(b,a),sub(c,a),sub(p,a));
    let (d1,d2) = (dot(ab,ap),dot(ac,ap));
    if d1 <= 0. && d2 <= 0. { return (a,Region::Vertex); }
    let bp = sub(p,b); let (d3,d4) = (dot(ab,bp),dot(ac,bp));
    if d3 >= 0. && d4 <= d3 { return (b,Region::Vertex); }
    let vc = d1*d4-d3*d2;
    if vc <= 0. && d1 >= 0. && d3 <= 0. { let v = d1/(d1-d3); return (add(a,scale(ab,v)),Region::Edge); }
    let cp = sub(p,c); let (d5,d6) = (dot(ab,cp),dot(ac,cp));
    if d6 >= 0. && d5 <= d6 { return (c,Region::Vertex); }
    let vb = d5*d2-d1*d6;
    if vb <= 0. && d2 >= 0. && d6 <= 0. { let w = d2/(d2-d6); return (add(a,scale(ac,w)),Region::Edge); }
    let va = d3*d6-d5*d4;
    if va <= 0. && d4-d3 >= 0. && d5-d6 >= 0. { let w = (d4-d3)/((d4-d3)+(d5-d6)); return (add(b,scale(sub(c,b),w)),Region::Edge); }
    let denom = 1./(va+vb+vc); let (v,w) = (vb*denom,vc*denom);
    (add(a,add(scale(ab,v),scale(ac,w))),Region::Face)
}

/// A sparse uniform grid: ids filed under the cubic cells (of side `cell`) their points or boxes
/// fall in. Cells are visited in x, then y, then z order and a cell's ids in the order they were
/// filed, so whatever is read from a grid is deterministic.
#[derive(Clone,Debug)]
pub struct Grid { cell: f64,cells: std::collections::BTreeMap<[i64;3],Vec<u32>> }

impl Grid {
    pub fn new(cell: f64) -> Grid { Grid {cell,cells:Default::default()} }
    pub fn cell(&self) -> f64 { self.cell }
    pub fn key(&self,p: V3) -> [i64;3] { p.map(|x| (x/self.cell).floor() as i64) }
    /// File `id` under the cell holding `p`.
    pub fn insert(&mut self,p: V3,id: u32) { let k = self.key(p); self.cells.entry(k).or_default().push(id); }
    /// File `id` under every cell the box from `lo` to `hi` meets.
    pub fn insert_box(&mut self,lo: V3,hi: V3,id: u32) {
        let (a,b) = (self.key(lo),self.key(hi));
        for x in a[0]..=b[0] { for y in a[1]..=b[1] { for z in a[2]..=b[2] { self.cells.entry([x,y,z]).or_default().push(id); } } }
    }
    /// The ids filed under one cell.
    pub fn at(&self,key: [i64;3]) -> &[u32] { self.cells.get(&key).map_or(&[],|l| &l[..]) }
    /// Every occupied cell with its ids.
    pub fn cells(&self) -> impl Iterator<Item = (&[i64;3],&[u32])> { self.cells.iter().map(|(k,l)| (k,&l[..])) }
    /// The ids in the cells from key `a` to key `b`, cell by cell.
    pub fn in_keys(&self,a: [i64;3],b: [i64;3],mut visit: impl FnMut(u32)) {
        let span = (0..3).map(|k| (b[k]-a[k]+1).max(0) as u128).product::<u128>();
        if span > self.cells.len() as u128 {
            // a box wider than the grid is occupied: the occupied cells in the box, in the same order
            for (k,list) in &self.cells { if (0..3).all(|i| k[i] >= a[i] && k[i] <= b[i]) { for &id in list { visit(id); } } }
        } else {
            for x in a[0]..=b[0] { for y in a[1]..=b[1] { for z in a[2]..=b[2] { for &id in self.at([x,y,z]) { visit(id); } } } }
        }
    }
    /// The ids in the cells the box from `lo` to `hi` meets, cell by cell.
    pub fn in_box(&self,lo: V3,hi: V3,visit: impl FnMut(u32)) { self.in_keys(self.key(lo),self.key(hi),visit); }
    /// The ids in the 27 cells round the one holding `p`, cell by cell.
    pub fn around(&self,p: V3,visit: impl FnMut(u32)) { let k = self.key(p); self.in_keys(k.map(|x| x-1),k.map(|x| x+1),visit); }
}

impl Grid {
    /// The grid, filed as it stands, packed for reading only.
    pub fn pack(&self) -> PackedGrid {
        let n = self.cells.len();
        let (mut lo,mut hi) = ([i64::MAX;3],[i64::MIN;3]);
        for k in self.cells.keys() { for i in 0..3 { lo[i] = lo[i].min(k[i]); hi[i] = hi[i].max(k[i]); } }
        let total: usize = self.cells.values().map(Vec::len).sum();
        let dims: [u128;3] = std::array::from_fn(|i| if n == 0 { 0 } else { (hi[i]-lo[i]+1) as u128 });
        let dense = n > 0 && dims[0]*dims[1]*dims[2] <= 8*total as u128+64;
        let mut packed = PackedGrid {cell:self.cell,lo,hi,dims:dims.map(|d| d as usize),dense,keys:Vec::new(),starts:vec![0],ids:Vec::with_capacity(total)};
        if dense {
            // every cell of the occupied box, in the order the tree already holds them
            let cells = packed.dims.iter().product::<usize>();
            packed.starts = vec![0;cells+1];
            for (k,list) in &self.cells { let c = packed.index(*k); packed.starts[c+1] = list.len() as u32; }
            for c in 0..cells { packed.starts[c+1] += packed.starts[c]; }
            for list in self.cells.values() { packed.ids.extend_from_slice(list); }
        } else {
            for (k,list) in &self.cells { packed.keys.push(*k); packed.ids.extend_from_slice(list); packed.starts.push(packed.ids.len() as u32); }
        }
        packed
    }
}

/// A `Grid` packed for reading only: the same cells, visited in the same order with the same
/// ids, in one dense table over the occupied cells' box where that box is small and in sorted
/// runs otherwise.
#[derive(Clone,Debug)]
pub struct PackedGrid { cell: f64,lo: [i64;3],hi: [i64;3],dims: [usize;3],dense: bool,keys: Vec<[i64;3]>,starts: Vec<u32>,ids: Vec<u32> }

impl PackedGrid {
    pub fn key(&self,p: V3) -> [i64;3] { p.map(|x| (x/self.cell).floor() as i64) }
    fn index(&self,k: [i64;3]) -> usize { (((k[0]-self.lo[0]) as usize*self.dims[1])+(k[1]-self.lo[1]) as usize)*self.dims[2]+(k[2]-self.lo[2]) as usize }
    fn run(&self,c: usize) -> &[u32] { &self.ids[self.starts[c] as usize..self.starts[c+1] as usize] }
    /// The ids filed under one cell.
    pub fn at(&self,k: [i64;3]) -> &[u32] {
        if (0..3).any(|i| k[i] < self.lo[i] || k[i] > self.hi[i]) { return &[]; }
        if self.dense { self.run(self.index(k)) } else { self.keys.binary_search(&k).map_or(&[],|c| self.run(c)) }
    }
    /// The ids in the cells from key `a` to key `b`, cell by cell.
    pub fn in_keys(&self,a: [i64;3],b: [i64;3],mut visit: impl FnMut(u32)) {
        // no cell outside the occupied box holds anything
        let (a,b): ([i64;3],[i64;3]) = (std::array::from_fn(|i| a[i].max(self.lo[i])),std::array::from_fn(|i| b[i].min(self.hi[i])));
        if (0..3).any(|i| a[i] > b[i]) { return; }
        let span = (0..3).map(|k| (b[k]-a[k]+1) as u128).product::<u128>();
        if !self.dense && span > self.keys.len() as u128 {
            for (c,k) in self.keys.iter().enumerate() { if (0..3).all(|i| k[i] >= a[i] && k[i] <= b[i]) { for &id in self.run(c) { visit(id); } } }
        } else {
            for x in a[0]..=b[0] { for y in a[1]..=b[1] { for z in a[2]..=b[2] { for &id in self.at([x,y,z]) { visit(id); } } } }
        }
    }
    /// The ids in the cells the box from `lo` to `hi` meets, cell by cell.
    pub fn in_box(&self,lo: V3,hi: V3,visit: impl FnMut(u32)) { self.in_keys(self.key(lo),self.key(hi),visit); }
    /// The ids in the 27 cells round the one holding `p`, cell by cell.
    pub fn around(&self,p: V3,visit: impl FnMut(u32)) { let k = self.key(p); self.in_keys(k.map(|x| x-1),k.map(|x| x+1),visit); }
}
