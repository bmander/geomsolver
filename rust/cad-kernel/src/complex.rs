//! **The cell complex a set of surfaces divides space into** (#735 A1): faces, each parting two
//! cells, and edges each bounding any number of faces, taken in order round it (the radial edge,
//! Weiler 1986; a selective geometric complex, Rossignac & O'Connor 1989). A touch along a curve —
//! two pieces of material meeting along an edge and nowhere else near — or at a point is a state it
//! holds, not a `Brep::check` failure: four faces round an edge, two cells pinched at a vertex.
//!
//! **Sides and cells.** A face's material normal is its outward one (`topo`'s orientation): its
//! *back* side (`2f`) faces against it, its *front* (`2f + 1`) along it. Round each edge the faces
//! are sorted by the way into each from the edge; the wedge between two neighbours is one cell,
//! bounded by the side of each facing it. A cell is a class of sides joined so, and each class a
//! shell: wound positively (its sides' faces turned to face out of it bound a positive volume) the
//! outer shell of a bounded cell, otherwise the boundary of a void in the smallest cell holding it,
//! or of the unbounded cell, `0`.
//!
//! **Selection.** A solid is the boundary between cells kept and cells not (`Complex::select`): the
//! faces whose two cells are kept differently, each turned to face out of the kept one. Where that
//! boundary touches itself along an edge (kept cells meeting there across unkept ones), each kept
//! wedge's two faces take an edge of their own, so the result is a valid boundary, and it says so
//! (`Touch`); where it touches itself at a point, it says that too.
use super::geom::{Surface,V};
use super::props::volume;
use super::query::{Located,Place};
use super::topo::{reverse_loop,Brep,EdgeCurve,Face};
use crate::util::fmath::Det;
use crate::util::space::{cross,dot,norm,normalised,scale,sub};
use std::collections::BTreeMap;

const TAU: f64 = std::f64::consts::TAU;

/// Two ways into faces from an edge closer than this (radians) are read again a step into each
/// face, where their surfaces' curvatures part them (faces tangent along the edge).
const TANGENT: f64 = 1e-6;

/// Where a face of the complex came from: an operand of the arrangement, its face there, and
/// whether the complex's face faces the way that one did.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub struct Origin { pub operand: usize,pub face: usize,pub alike: bool }

/// One use of an edge by a face's loop, in its place round the edge.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Use {
    /// The face, its loop and the use's place in it.
    pub face: usize,pub lp: usize,pub at: usize,
    /// The way into the face from the edge, as an angle about the edge's tangent at its middle
    /// (not measured, `NaN`, where two uses alone meet at the edge: they need no order).
    pub angle: f64,
    /// The shell bounding the wedge turning on from this use about the edge, and the one behind it.
    pub ahead: usize,pub behind: usize,
}

/// A cell: the sides of faces bounding it (each face and whether its front faces into the cell),
/// shell by shell, its outer shell first (none for the unbounded cell), and its volume (infinite
/// for the unbounded cell).
#[derive(Clone,Debug,Default)]
pub struct Cell { pub shells: Vec<usize>,pub volume: f64 }

/// The cells shells bound (`Complex::cells`): each cell, and each shell's cell.
#[derive(Clone,Debug)]
struct Cells { cells: Vec<Cell>,of_shell: Vec<usize>,volumes: Vec<f64> }

/// The cell complex (the module's account).
#[derive(Clone,Debug)]
pub struct Complex {
    /// Every face of the complex, each once, with the edges and vertices they use: a boundary that
    /// need be no manifold.
    pub b: Brep,
    /// Each face's two shells: behind it, then in front of it.
    pub shell_sides: Vec<[usize;2]>,
    /// The shells: classes of sides joined round edges, each side a face and whether its front
    /// faces into the shell's cell.
    pub shells: Vec<Vec<(usize,bool)>>,
    /// Each edge's uses in order round it (none for a degenerate edge).
    pub radial: Vec<Vec<Use>>,
    /// Where each face came from: every operand's face that lies there (coincident faces merged
    /// into one are each named).
    pub origins: Vec<Vec<Origin>>,
    /// The tolerance it was built to (a length).
    pub tol: f64,
    /// The cells the shells bound, found the first time they are asked for: a shell's volume and
    /// which shell holds a void are measured, which a Boolean selecting by shells never asks.
    cells: std::sync::OnceLock<Result<Cells,String>>,
}

/// Where a boundary selected from a complex touches itself.
#[derive(Clone,Debug,PartialEq)]
pub enum Touch {
    /// Along an edge: its ends, and how many faces of the result meet there (four or more).
    Along { ends: [V;2],faces: usize },
    /// At a point: material meeting there and nowhere else near (a pinch).
    At(V),
}

/// A selection's faces before they are assembled (`Complex::select_parts`).
#[derive(Clone,Debug)]
pub struct Parts {
    /// Each kept face: the complex's face it is, it faced out of the kept shell, and whether an
    /// edge of it was given one of `twins` (it touches the result elsewhere along it).
    pub faces: Vec<(usize,Face,bool)>,
    /// One edge for each kept wedge beyond the first round an edge the result touches itself
    /// along, numbered after the complex's edges.
    pub twins: Vec<super::topo::Edge>,
    /// Where it touches itself along an edge.
    pub touches: Vec<Touch>,
}

/// A boundary selected from a complex (`Complex::select`), and where it touches itself.
#[derive(Clone,Debug)]
pub struct Selected { pub brep: Brep,pub touches: Vec<Touch> }

/// A face turned to face the other way: its loops walked the other way round.
pub fn turned(f: &Face) -> Face {
    Face {surface:f.surface.clone(),reversed:!f.reversed,loops:f.loops.iter().cloned().map(reverse_loop).collect(),name:f.name.clone()}
}

/// `f` as it is, or turned where `turn`.
pub fn faced(f: &Face,turn: bool) -> Face { if turn { turned(f) } else { f.clone() } }

/// A face's outward normal at its parameters `uv`, unit; none where the surface has none.
fn outward(f: &Face,uv: super::geom::Uv) -> Option<V> {
    let n = normalised(f.surface.normal_raw(uv))?;
    Some(if f.reversed { scale(n,-1.) } else { n })
}

/// `d` in the surface's parameters at `uv` (`topo::in_parameters`), where that is finite.
fn in_parameters(s: &Surface,uv: super::geom::Uv,d: V) -> Option<super::geom::Uv> {
    Some(super::topo::in_parameters(s,uv,d)).filter(|x| x.iter().all(|v| v.is_finite()))
}

/// `a` wrapped into `(-π, π]`.
fn wrapped(a: f64) -> f64 {
    let mut a = a%TAU;
    if a > std::f64::consts::PI { a -= TAU } else if a <= -std::f64::consts::PI { a += TAU }
    a
}

impl Complex {
    /// **The complex of faces sharing edges and vertices by construction** (`b`): each face's
    /// origins as `origins` says (one per face), the uses round each edge ordered and the sides
    /// joined into cells, to `tol`. Refused where two faces round an edge cannot be ordered (they
    /// coincide there, which the caller should have merged into one face).
    pub fn new(b: Brep,origins: Vec<Vec<Origin>>,tol: f64) -> Result<Complex,String> {
        Complex::try_new(b,origins,tol).map_err(|(e,_)| e)
    }

    /// `new`, handing `b` back where it is refused: a caller with another road for the faces (a
    /// Boolean keeping each by its own sides) takes them on without a copy.
    pub(crate) fn try_new(b: Brep,origins: Vec<Vec<Origin>>,tol: f64) -> Result<Complex,(String,Brep)> {
        assert_eq!(origins.len(),b.faces.len(),"one list of origins a face");
        match Complex::ordered(&b,tol) {
            Ok((shell_sides,shells,radial)) => Ok(Complex {b,shell_sides,shells,radial,origins,tol,cells:std::sync::OnceLock::new()}),
            Err(e) => Err((e,b)),
        }
    }

    /// `new`'s work: each face's two shells, the shells, and each edge's uses in order round it.
    #[allow(clippy::type_complexity)]
    fn ordered(b: &Brep,tol: f64) -> Result<(Vec<[usize;2]>,Vec<Vec<(usize,bool)>>,Vec<Vec<Use>>),String> {
        let nf = b.faces.len();
        // a step into each face, where two faces tangent along an edge have parted by their
        // curvatures' difference times its square: far past the tolerance, well inside the scale
        // (measured only where an edge has more than two uses)
        let mut step = None;
        let mut uses: Vec<Vec<(usize,usize,usize)>> = vec![Vec::new();b.edges.len()];
        for (fi,f) in b.faces.iter().enumerate() {
            for (li,l) in f.loops.iter().enumerate() {
                for (ci,c) in l.iter().enumerate() { uses[c.edge as usize].push((fi,li,ci)); }
            }
        }
        let mut sides_joined = crate::util::union_find::UnionFind::new(2*nf);
        let mut radial: Vec<Vec<Use>> = vec![Vec::new();b.edges.len()];
        // each sorted use's side facing the wedge ahead of it
        let mut wedges: Vec<Vec<usize>> = vec![Vec::new();b.edges.len()];
        for e in 0..b.edges.len() {
            let edge = &b.edges[e];
            let EdgeCurve::Curve(curve) = &edge.curve else { continue };
            if uses[e].is_empty() { continue }
            let tm = 0.5*(edge.t[0]+edge.t[1]);
            if uses[e].len() <= 2 {
                // one use (a face hanging free there) joins its own two sides; two join back to
                // back and front to front where they walk the edge opposite ways, back to front
                // where the same way: no geometry tells more
                let (u,w) = (uses[e][0],uses[e][uses[e].len()-1]);
                let r = |(f,l,c): (usize,usize,usize)| b.faces[f].loops[l][c].reversed;
                let opposite = uses[e].len() == 1 || r(u) != r(w);
                let pairs = if uses[e].len() == 1 { vec![(2*u.0,2*u.0+1)] }
                    else if opposite { vec![(2*u.0,2*w.0),(2*u.0+1,2*w.0+1)] } else { vec![(2*u.0,2*w.0+1),(2*u.0+1,2*w.0)] };
                for (x,y) in pairs { sides_joined.union(x,y); }
                // the wedge ahead of the first is behind one of its sides; which, no order says, so
                // the first's back is taken to face it (a selection asks only where kept faces
                // meet more than twice)
                radial[e] = uses[e].iter().map(|&(face,lp,at)| Use {face,lp,at,angle:f64::NAN,ahead:0,behind:0}).collect();
                wedges[e] = if uses[e].len() == 1 { vec![2*u.0] } else if opposite { vec![2*u.0,2*w.0+1] } else { vec![2*u.0,2*w.0] };
                continue
            }
            let step = *step.get_or_insert_with(|| (1e-3*b.size().max(tol)).max(64.*tol));
            let t = normalised(curve.tangent(tm)).ok_or_else(|| format!("edge {e} has no direction"))?;
            let m = curve.point(tm);
            let (b1,b2) = crate::util::space::across(t);
            // each use: its way in at the edge (first order) and a step in (second order), its normal
            let mut round: Vec<(f64,f64,usize,V,V)> = Vec::with_capacity(uses[e].len());
            for &(fi,li,ci) in &uses[e] {
                let f = &b.faces[fi];
                let co = &f.loops[li][ci];
                let uv = co.pcurve.at(tm,edge,&f.surface,&b.vertices);
                let n = outward(f,uv).ok_or_else(|| format!("edge {e}: a {} with no normal at {m:?}",f.surface.kind()))?;
                let walk = if co.reversed { scale(t,-1.) } else { t };
                let d = cross(n,walk);
                let d = sub(d,scale(t,dot(d,t)));
                let angle = dot(d,b2).datan2(dot(d,b1));
                let deep = match in_parameters(&f.surface,uv,d) {
                    Some(x) => {
                        let p = f.surface.point([uv[0]+step*x[0],uv[1]+step*x[1]]);
                        let q = sub(p,m);
                        let q = sub(q,scale(t,dot(q,t)));
                        if norm(q) > 0. { dot(q,b2).datan2(dot(q,b1)) } else { angle }
                    }
                    None => angle,
                };
                round.push((angle,deep,round.len(),d,n));
            }
            round.sort_by(|x,y| x.0.total_cmp(&y.0).then(x.2.cmp(&y.2)));
            // runs of ways in closer than `TANGENT` (wrapping round): ordered by the way a step in
            let k = round.len();
            let gap = |r: &[(f64,f64,usize,V,V)],i: usize| { let g = r[(i+1)%k].0-r[i].0; if i+1 == k { g+TAU } else { g } };
            let Some(s) = (0..k).find(|&i| gap(&round,i) > TANGENT) else {
                return Err(format!("edge {e} near {m:?}: every face round it is tangent to every other"))
            };
            // start after a gap wider than `TANGENT`, so no run is cut by the wrap
            round.rotate_left((s+1)%k);
            let mut i = 0;
            while i < k {
                let mut j = i+1;
                while j < k && wrapped(round[j].0-round[j-1].0).abs() <= TANGENT { j += 1; }
                if j-i > 1 {
                    let base = round[i].0;
                    let run = &mut round[i..j];
                    run.sort_by(|x,y| wrapped(x.1-base).total_cmp(&wrapped(y.1-base)).then(x.2.cmp(&y.2)));
                    for w in run.windows(2) {
                        if (wrapped(w[1].1-base)-wrapped(w[0].1-base)).abs() <= 1e-9 {
                            let (f,g) = (uses[e][w[0].2].0,uses[e][w[1].2].0);
                            if f != g {
                                return Err(format!("edge {e} near {m:?}: faces {f} and {g} (a {} and a {}) coincide along it",
                                    b.faces[f].surface.kind(),b.faces[g].surface.kind()))
                            }
                        }
                    }
                }
                i = j;
            }
            let mut sides = Vec::with_capacity(k);
            for i in 0..k {
                let (_,_,ui,d,n) = round[i];
                let (_,_,uj,dj,nj) = round[(i+1)%k];
                // the wedge from i's way in to j's, turning about the edge: i bounds it by the side
                // facing the turn, j by the side facing back against it
                let r = cross(t,d);
                let rj = cross(t,dj);
                let (fi,_,_) = uses[e][ui];
                let (fj,_,_) = uses[e][uj];
                let side_i = 2*fi+usize::from(dot(n,r) > 0.);
                let side_j = 2*fj+usize::from(dot(nj,rj) < 0.);
                sides_joined.union(side_i,side_j);
                sides.push(side_i);
            }
            radial[e] = round.iter().map(|&(angle,_,u,_,_)| {
                let (face,lp,at) = uses[e][u];
                Use {face,lp,at,angle,ahead:0,behind:0}
            }).collect();
            wedges[e] = sides;
        }
        // the classes of sides, in order of their least side: the shells
        let mut shell_of_root: BTreeMap<usize,usize> = BTreeMap::new();
        let mut shells: Vec<Vec<(usize,bool)>> = Vec::new();
        let mut shell_of_side = vec![0usize;2*nf];
        for side in 0..2*nf {
            let r = sides_joined.find(side);
            let c = *shell_of_root.entry(r).or_insert_with(|| { shells.push(Vec::new()); shells.len()-1 });
            shells[c].push((side/2,side%2 == 1));
            shell_of_side[side] = c;
        }
        for e in 0..b.edges.len() {
            for (u,&ahead) in radial[e].iter_mut().zip(&wedges[e]) { u.ahead = shell_of_side[ahead]; }
            let k = radial[e].len();
            for i in 0..k { let a = radial[e][(i+k-1)%k].ahead; radial[e][i].behind = a; }
        }
        let shell_sides = (0..nf).map(|f| [shell_of_side[2*f],shell_of_side[2*f+1]]).collect();
        Ok((shell_sides,shells,radial))
    }

    /// Shell `k` as a solid's boundary: its sides' faces turned to face out of its cell.
    pub fn shell(&self,k: usize) -> Brep { self.boundary_of(&self.shells[k]) }

    /// The boundary of `sides`, each face turned where its front faces in.
    fn boundary_of(&self,sides: &[(usize,bool)]) -> Brep {
        let faces = sides.iter().map(|&(f,front)| faced(&self.b.faces[f],front)).collect();
        Brep {vertices:self.b.vertices.clone(),edges:self.b.edges.clone(),faces}.compacted()
    }

    /// Bounded cell `c`'s sides, every shell's.
    pub(crate) fn cell_sides(&self,cells: &[Cell],c: usize) -> Vec<(usize,bool)> {
        cells[c].shells.iter().flat_map(|&k| self.shells[k].iter().copied()).collect()
    }

    /// **The cells the shells bound**, found once: each shell wound positively (its faces turned
    /// out of it bound a positive volume) the outer shell of a bounded cell; each other the boundary
    /// of a void in the smallest such holding a point of it, or of the unbounded cell, `0`.
    fn cells_found(&self) -> Result<&Cells,String> {
        self.cells.get_or_init(|| {
            let size = self.b.size().max(self.tol);
            let shells: Vec<Brep> = (0..self.shells.len()).map(|k| self.shell(k)).collect();
            let volumes: Vec<f64> = shells.iter().map(volume).collect();
            let floor = 1e-12*size*size*size;
            let outer: Vec<usize> = (0..shells.len()).filter(|&c| volumes[c] > floor).collect();
            let is_outer: Vec<bool> = volumes.iter().map(|&v| v > floor).collect();
            // smallest first, so a void finds its tightest host first
            let mut order = outer.clone();
            order.sort_by(|&x,&y| volumes[x].total_cmp(&volumes[y]).then(x.cmp(&y)));
            let mut cells: Vec<Cell> = vec![Cell {shells:Vec::new(),volume:f64::INFINITY}];
            let mut of_shell = vec![0usize;shells.len()];
            for &c in &outer {
                cells.push(Cell {shells:vec![c],volume:volumes[c]});
                of_shell[c] = cells.len()-1;
            }
            let located: BTreeMap<usize,Located> = order.iter().map(|&c| (c,Located::new(&shells[c],self.tol))).collect();
            for c in 0..shells.len() {
                if is_outer[c] { continue }
                // a void's points read at its edges' middles, one off every outer shell's boundary
                // (a void touching its host at a point)
                let probes: Vec<V> = shells[c].edges.iter().filter_map(|e| match &e.curve {
                    EdgeCurve::Curve(cv) => Some(cv.point(0.5*(e.t[0]+e.t[1]))),
                    EdgeCurve::Degenerate => None,
                }).collect();
                let mut host = 0;
                'hosts: for &o in &order {
                    for &p in &probes {
                        match located[&o].solid_place(p) {
                            Place::In => { host = of_shell[o]; break 'hosts }
                            Place::Out => continue 'hosts,
                            Place::On => continue,
                        }
                    }
                }
                of_shell[c] = host;
                cells[host].shells.push(c);
                if host != 0 { cells[host].volume += volumes[c]; }
            }
            Ok(Cells {cells,of_shell,volumes})
        }).as_ref().map_err(|e| e.clone())
    }

    /// The cells (`0` the unbounded one), each its shells and volume.
    pub fn cells(&self) -> Result<&[Cell],String> { Ok(&self.cells_found()?.cells) }

    /// The cell shell `k` bounds.
    pub fn cell_of_shell(&self,k: usize) -> Result<usize,String> { Ok(self.cells_found()?.of_shell[k]) }

    /// Face `f`'s two cells: behind it, then in front of it.
    pub fn sides(&self,f: usize) -> Result<[usize;2],String> {
        let c = self.cells_found()?;
        Ok(self.shell_sides[f].map(|k| c.of_shell[k]))
    }

    /// **The complex one boundary parts space into** (a solid's, a manifold or not): its faces as
    /// they are, each its own origin (operand 0).
    pub fn of(b: &Brep,tol: f64) -> Result<Complex,String> {
        let origins = (0..b.faces.len()).map(|face| vec![Origin {operand:0,face,alike:true}]).collect();
        Complex::new(b.clone(),origins,tol)
    }

    /// The bounded cells' indices.
    pub fn bounded(&self) -> Result<std::ops::Range<usize>,String> { Ok(1..self.cells()?.len()) }

    /// **Everything the complex promises, checked**: every loop closed; every non-degenerate edge
    /// used by some face; each wedge round an edge one shell, bounded by the sides facing it; every
    /// shell's boundary closed — each edge it uses used as often along as against by its sides;
    /// every bounded cell's volume positive; and the bounded cells filling, together, what the
    /// unbounded cell's shells enclose (to `tol`, a length, over their area).
    pub fn check(&self,tol: f64) -> Result<(),String> {
        let b = &self.b;
        for (fi,f) in b.faces.iter().enumerate() {
            if f.loops.is_empty() { return Err(format!("face {fi} has no loop")) }
            for (li,l) in f.loops.iter().enumerate() {
                for (k,c) in l.iter().enumerate() {
                    if b.ends(c)[1] != b.ends(&l[(k+1)%l.len()])[0] { return Err(format!("face {fi}: loop {li} breaks after use {k}")) }
                }
            }
        }
        let mut used = vec![false;b.edges.len()];
        for c in b.faces.iter().flat_map(|f| f.loops.iter().flatten()) { used[c.edge as usize] = true; }
        if let Some(e) = (0..b.edges.len()).find(|&e| !used[e]) { return Err(format!("edge {e} bounds no face")) }
        for (e,round) in self.radial.iter().enumerate() {
            for (i,u) in round.iter().enumerate() {
                let next = &round[(i+1)%round.len()];
                if u.ahead != next.behind { return Err(format!("edge {e}: the wedge after use {i} is two shells")) }
            }
        }
        for (k,shell) in self.shells.iter().enumerate() {
            let mut count: BTreeMap<u32,(usize,usize)> = BTreeMap::new();
            for &(f,front) in shell {
                for c in self.b.faces[f].loops.iter().flatten() {
                    if matches!(b.edges[c.edge as usize].curve,EdgeCurve::Degenerate) { continue }
                    let n = count.entry(c.edge).or_default();
                    if c.reversed == front { n.0 += 1 } else { n.1 += 1 }
                }
            }
            if let Some((e,n)) = count.iter().find(|(_,n)| n.0 != n.1) {
                return Err(format!("shell {k}: its boundary is open at edge {e}, used {} times along and {} against",n.0,n.1))
            }
        }
        let found = self.cells_found()?;
        for (ci,cell) in found.cells.iter().enumerate().skip(1) {
            if !(cell.volume > 0.) { return Err(format!("cell {ci} has volume {:e}",cell.volume)) }
        }
        let filled: f64 = found.cells[1..].iter().map(|c| c.volume).sum();
        let enclosed: f64 = -found.cells[0].shells.iter().map(|&k| found.volumes[k]).sum::<f64>();
        let area: f64 = b.faces.iter().map(|f| super::props::area(b,f)).sum();
        if (filled-enclosed).abs() > 1e-9*enclosed.abs()+tol*area {
            return Err(format!("the bounded cells fill {filled:e}, their outer boundary encloses {enclosed:e}"))
        }
        Ok(())
    }

    /// The cell a point lies in: the smallest bounded cell whose outer shell holds it and none of
    /// whose voids does, else the unbounded one (`None` where it lies on the complex's faces, or
    /// the cells could not be found).
    pub fn cell_at(&self,p: V) -> Option<usize> {
        let cells = self.cells().ok()?;
        let place = |k: usize,turn: bool| {
            let sides: Vec<(usize,bool)> = self.shells[k].iter().map(|&(f,front)| (f,front != turn)).collect();
            Located::owned(self.boundary_of(&sides),self.tol).solid_place(p)
        };
        let mut best: Option<(f64,usize)> = None;
        for (c,cell) in cells.iter().enumerate().skip(1) {
            match place(cell.shells[0],false) {
                Place::On => return None,
                Place::Out => continue,
                Place::In => {}
            }
            let mut inside = true;
            // a void's sides face into the cell; turned, they bound the void
            for &void in &cell.shells[1..] {
                match place(void,true) { Place::On => return None,Place::In => { inside = false; break }
                    Place::Out => {} }
            }
            if inside && best.is_none_or(|(v,_)| cell.volume < v) { best = Some((cell.volume,c)); }
        }
        Some(best.map_or(0,|(_,c)| c))
    }

    /// **The boundary between the cells `keep` keeps and the rest** (the module's account), the
    /// cells found first (`cells`).
    pub fn select(&self,keep: &dyn Fn(usize) -> bool) -> Result<Selected,String> {
        let found = self.cells_found()?;
        Ok(self.select_shells(&|k| keep(found.of_shell[k])))
    }

    /// **The boundary between the shells `keep` keeps and the rest**: each face whose two shells
    /// are kept differently, faced out of the kept one, in the faces' order; at an edge where it
    /// touches itself, each kept wedge's two faces given an edge of their own (the first keeping
    /// it), and every such edge and every pinch said. A shell's cell is kept or not as its shells
    /// all are, so this is a selection of cells wherever the caller reads a shell's cell from its
    /// sides — as a Boolean does — without finding the cells.
    pub fn select_shells(&self,keep: &dyn Fn(usize) -> bool) -> Selected {
        let parts = self.select_parts(keep);
        let edges = self.b.edges.iter().cloned().chain(parts.twins).collect();
        let b = Brep {vertices:self.b.vertices.clone(),edges,faces:parts.faces.into_iter().map(|(_,f,_)| f).collect()}.compacted();
        let mut touches = parts.touches;
        for p in b.pinches() {
            let along = touches.iter().any(|t| matches!(t,Touch::Along {ends,..}
                if ends.iter().any(|&q| crate::util::space::distance(p,q) <= self.tol)));
            if !along { touches.push(Touch::At(p)); }
        }
        Selected {brep:b,touches}
    }

    /// `select_shells`'s faces before they are assembled: each kept face with the complex's face
    /// it is, the edges each touch gives its kept wedges, and the touches along edges (pinches at
    /// points are `select_shells`'s, read on the assembled boundary).
    pub fn select_parts(&self,keep: &dyn Fn(usize) -> bool) -> Parts {
        let kept: Vec<bool> = (0..self.shells.len()).map(keep).collect();
        let mut twins: Vec<super::topo::Edge> = Vec::new();
        let mut at = vec![usize::MAX;self.b.faces.len()];
        let mut faces = Vec::new();
        for (f,face) in self.b.faces.iter().enumerate() {
            let [behind,front] = self.shell_sides[f];
            if kept[behind] == kept[front] { continue }
            at[f] = faces.len();
            faces.push((f,faced(face,!kept[behind]),false));
        }
        let mut touches = Vec::new();
        for (e,round) in self.radial.iter().enumerate() {
            let mine: Vec<&Use> = round.iter().filter(|u| at[u.face] != usize::MAX).collect();
            if mine.len() <= 2 { continue }
            let k = mine.len();
            let pairs: Vec<(&Use,&Use)> = (0..k).filter(|&i| kept[mine[i].ahead]).map(|i| (mine[i],mine[(i+1)%k])).collect();
            let ends = self.b.edges[e].v.map(|v| self.b.vertices[v as usize].p);
            touches.push(Touch::Along {ends,faces:k});
            for &(x,y) in pairs.iter().skip(1) {
                twins.push(self.b.edges[e].clone());
                let new = (self.b.edges.len()+twins.len()-1) as u32;
                for u in [x,y] {
                    // the use's place in the kept face: its loop is the same, turned or not
                    faces[at[u.face]].2 = true;
                    let face = &mut faces[at[u.face]].1;
                    let l = &mut face.loops[u.lp];
                    let n = l.len();
                    let place = if face.reversed == self.b.faces[u.face].reversed { u.at } else { n-1-u.at };
                    l[place].edge = new;
                }
            }
        }
        Parts {faces,twins,touches}
    }

    /// The bounded cells, each its own solid: its shells' faces turned to face out of it.
    pub fn solids(&self) -> Result<Vec<Brep>,String> {
        let cells = self.cells()?;
        Ok((1..cells.len()).map(|c| self.boundary_of(&self.cell_sides(cells,c))).collect())
    }

    /// A point deep inside bounded cell `c`: the deepest of its points `inside_points` reads.
    pub fn inside(&self,c: usize) -> Result<Option<V>,String> { Ok(self.inside_points(c,1)?.first().copied()) }

    /// **Points inside bounded cell `c`**, the deepest first: its solid's interior points
    /// (`query::interior`, read off its mesh); and where its boundary cannot be meshed — a cell
    /// thinner than the mesh's bar, a cusp between two surfaces tangent at its end — points of its
    /// faces stepped into it by a quarter of its size, then a quarter of that and so on, each kept
    /// where the cell's own boundary reads it inside.
    pub fn inside_points(&self,c: usize,capacity: usize) -> Result<Vec<V>,String> {
        let cells = self.cells()?;
        let solid = self.boundary_of(&self.cell_sides(cells,c));
        if let Ok(found) = super::query::interior(&solid,capacity) { if !found.is_empty() { return Ok(found.into_iter().map(|(p,_)| p).collect()) } }
        let located = Located::new(&solid,self.tol);
        let size = solid.size();
        let mut out = Vec::new();
        for (fi,f) in solid.faces.iter().enumerate() {
            // a point of the face: the middle of a use of it nudged in, else a grid of its parameters
            let [[u0,u1],[v0,v1]] = super::boolean::face_parameters(&solid,fi);
            const N: usize = 9;
            for i in 1..N { for j in 1..N {
                let uv = [u0+(u1-u0)*i as f64/N as f64,v0+(v1-v0)*j as f64/N as f64];
                let p = f.surface.point(uv);
                if located.face_place(fi,p) != Place::In { continue }
                let Some(n) = outward(f,uv) else { continue };
                let mut d = size/4.;
                while d > 16.*self.tol {
                    let q = crate::util::space::sub(p,scale(n,d));
                    if located.solid_place(q) == Place::In { out.push((d,q)); break }
                    d /= 4.;
                }
            } }
        }
        out.sort_by(|a,b| b.0.total_cmp(&a.0));
        Ok(out.into_iter().take(capacity.max(1)*8).map(|(_,q)| q).collect())
    }
}
