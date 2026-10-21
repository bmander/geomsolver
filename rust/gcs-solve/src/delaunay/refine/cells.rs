//! Refining the cells inside the domain once its surface is done (#510 P6): the tetrahedra
//! whose orthocentre the domain holds, judged by their circumradius over their shortest edge,
//! their circumradius and their least dihedral angle, and refined worst first.
//!
//! - **A cell too big or too long for its shortest edge** is refined by its orthocentre, the
//!   centre of its empty ball (Shewchuk's tetrahedral Delaunay refinement): the point is at least
//!   that ball's radius from every vertex, so refinement ends.
//! - **A point in a restricted facet's surface ball refines the facet instead** (Mesh_3's rule,
//!   `Bad::forced`): inserted, it would take the facet out of the surface and leave a vertex just
//!   inside it. The facet's surface centre goes in, and the cell is judged again, in this round
//!   or the next.
//! - **A sliver** — fair radius–edge ratio, four corners near one circle, its least dihedral
//!   angle below `Cells::dihedral` — has its orthocentre near its own corners' circle and no
//!   orthocentre to refine it by alone. It is refined by a point of its **picking region**, a
//!   ball of `PICK` times its radius about the orthocentre (Li and Teng, "Generating well-shaped
//!   Delaunay meshes in 3D", 2001): of `CANDIDATES` seeded draws, the one whose new tetrahedra's
//!   least dihedral angle is largest, among those the domain holds and that reach no facet's
//!   surface ball.
//! - **A point in a protecting ball, or a cell whose ball is under the refinement margin**, is
//!   left and counted (`CellReport`): no cell refinement shrinks a ball.
//!
//! Points are inserted as the surface's are, so the facets they make are judged and refined
//! first, and the surface's criteria hold to the end.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::delaunay::tetra;

/// What the cells inside must satisfy, in the domain's units but the angle.
#[derive(Clone,Debug)]
pub struct Cells {
    /// The largest ratio of a cell's circumradius to its shortest edge. Refinement provably meets
    /// 2 away from small input angles (Shewchuk); a regular tetrahedron's is 0.61.
    pub radius_edge: f64,
    /// The largest circumradius a cell may have.
    pub size: f64,
    /// The least dihedral angle a cell may keep, in degrees: below it a cell is a sliver.
    pub dihedral: f64,
}

/// What cell refinement did.
#[derive(Clone,Debug,Default)]
pub struct CellReport {
    /// Cell points inserted, and of them those picked for a sliver.
    pub inserted: usize,
    pub slivers: usize,
    /// Cell points left because they fell in a protecting ball.
    pub in_balls: usize,
    /// Cells left because their ball was under the refinement margin.
    pub too_small: usize,
    /// Cell points that reached a restricted facet's surface ball, the facet refined instead.
    pub encroaching: usize,
    /// Rounds of judging every cell (`Progressive`'s filling).
    pub rounds: usize,
}

/// The seed of a sliver's draws: one triangulation, one mesh.
pub(super) const SEED: u32 = 0x510;
/// A sliver's picking region, as a fraction of its circumradius about its orthocentre.
const PICK: f64 = 0.3;
/// Draws from a picking region.
const CANDIDATES: usize = 24;

/// A cell waiting to be refined, as `Bad` is a facet.
#[derive(PartialEq)]
pub(super) struct BadCell { badness: f64,t: u32,key: [u32;4] }
impl BadCell { pub(super) fn badness(&self) -> f64 { self.badness } }
impl Eq for BadCell {}
impl PartialOrd for BadCell { fn partial_cmp(&self,o: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) } }
impl Ord for BadCell { fn cmp(&self,o: &Self) -> std::cmp::Ordering { self.badness.total_cmp(&o.badness) } }

impl Cells {
    /// Whether the criteria can be met at all.
    pub(super) fn check(&self) -> Result<(),String> {
        if !(self.radius_edge >= 1.) { return Err(format!("a cell's radius–edge bound must be at least 1, not {}",self.radius_edge)); }
        if !(self.size > 0.) { return Err(format!("a cell's size must be positive, not {}",self.size)); }
        if !(self.dihedral >= 0. && self.dihedral < 30.) {
            return Err(format!("a cell's least dihedral angle must be from 0° to under 30°, not {}°",self.dihedral));
        }
        Ok(())
    }

    /// How badly the tetrahedron `q` breaks the criteria (above 1 is bad), and whether its shape
    /// and size pass, so that only its angles do not.
    fn badness(&self,q: [P;4]) -> (f64,bool) {
        let shape = match tetra::circumsphere(q) {
            Some((_,r)) => (r/tetra::shortest_edge(q)/self.radius_edge).max(r/self.size),
            None => f64::INFINITY,
        };
        let least = tetra::least_dihedral(q);
        let angle = if least < self.dihedral { 1.+(self.dihedral-least)/self.dihedral } else { 0. };
        (shape.max(angle),shape <= 1.)
    }
}

impl Refiner<'_> {
    fn corners(&self,t: u32) -> [P;4] { self.tri.tet(t).v.map(|v| self.tri.points()[v as usize].p) }

    /// Whether `t` is a cell of the volume: on no enclosing vertex, its orthocentre inside.
    pub(super) fn inside(&mut self,t: u32) -> bool {
        self.tri.alive(t) && !self.tri.touches_enclosure(t) && self.tet_side(t) < 0
    }

    /// Queue cell `t` if it is inside and breaks the criteria.
    pub(super) fn judge_cell(&mut self,t: u32) {
        let Some(cells) = self.cells.as_ref() else { return };
        let (badness,_) = cells.badness(self.corners(t));
        if badness <= 1. || !self.inside(t) { return; }
        let mut key = self.tri.tet(t).v;
        key.sort_unstable();
        self.cell_queue.push(BadCell {badness,t,key});
    }

    /// Judge every cell again: a round of filling (`Progressive`).
    pub(super) fn judge_cells(&mut self) {
        self.cell_report.rounds += 1;
        let tets: Vec<u32> = self.tri.tets().collect();
        for t in tets { self.judge_cell(t); }
    }

    /// Refine at most about `budget` points, waiting facets first, then the worst cell: whether
    /// both queues are empty.
    pub(super) fn fill_some(&mut self,budget: usize) -> Result<bool,String> {
        let mut spent = 0;
        while spent < budget {
            spent += 1;
            if !self.queue.is_empty() { self.refine_some(1)?; continue; }
            let Some(bad) = self.cell_queue.pop() else { return Ok(true) };
            if !self.tri.alive(bad.t) { continue; }
            let mut key = self.tri.tet(bad.t).v;
            key.sort_unstable();
            if key != bad.key { continue; }
            let (badness,sliver) = self.cells.as_ref().expect("criteria while filling").badness(self.corners(bad.t));
            if badness <= 1. { continue; }
            self.refine_cell(bad.t,sliver)?;
        }
        Ok(self.queue.is_empty() && self.cell_queue.is_empty())
    }

    fn refine_cell(&mut self,t: u32,sliver: bool) -> Result<(),String> {
        let (o,r2) = self.tri.orthosphere(t);
        let margin = self.margin();
        if !o.iter().all(|x| x.is_finite()) || !(r2*(1.-PICK)*(1.-PICK) > margin*margin) {
            self.cell_report.too_small += 1;
            return Ok(());
        }
        let picked = if sliver { self.pick(t,o,r2.sqrt())? } else { None };
        let p = picked.unwrap_or(o);
        if self.ball_at(p).is_some() { self.cell_report.in_balls += 1; return Ok(()); }
        let reached = self.encroached(p,t)?;
        if !reached.is_empty() {
            self.cell_report.encroaching += 1;
            for (u,i,key) in reached { self.queue.push(Bad {badness:f64::INFINITY,t:u,i,key,forced:true}); }
            return Ok(());
        }
        if self.insert(p)? {
            self.cell_report.inserted += 1;
            if picked.is_some() { self.cell_report.slivers += 1; }
        }
        Ok(())
    }

    /// The restricted facets whose surface ball holds `p`, among those its insertion would remake.
    fn encroached(&mut self,p: P,hint: u32) -> Result<Vec<(u32,usize,[u32;3])>,String> {
        let region = self.tri.conflicts(p,0.,Some(hint))?;
        let mut out: Vec<(u32,usize,[u32;3])> = Vec::new();
        for &u in &region {
            for i in 0..4 {
                let Some((key,centre)) = self.restricted(u,i) else { continue };
                if out.iter().any(|f| f.2 == key) { continue; }
                let a = self.tri.points()[key[0] as usize];
                if dist2(p,centre) < dist2(centre,a.p)-a.w { out.push((u,i,key)); }
            }
        }
        Ok(out)
    }

    /// A sliver's refinement point from its picking region: the orthocentre and `CANDIDATES`
    /// draws in the ball of `PICK × radius` about it, best first by the least dihedral angle of
    /// the tetrahedra each would make, the first the domain holds and that reaches no surface
    /// ball and no protecting ball. `None` when no draw qualifies.
    fn pick(&mut self,t: u32,o: P,radius: f64) -> Result<Option<P>,String> {
        let mut candidates = vec![o];
        while candidates.len() <= CANDIDATES {
            let d: P = std::array::from_fn(|_| self.rng.uniform(-1.,1.));
            if dot(d,d) > 1. { continue; }
            candidates.push(std::array::from_fn(|k| o[k]+PICK*radius*d[k]));
        }
        let mut scored = Vec::with_capacity(candidates.len());
        for (k,&p) in candidates.iter().enumerate() {
            if self.ball_at(p).is_some() { continue; }
            scored.push((self.star_quality(p,t)?,k));
        }
        // best first, the draw order breaking ties
        scored.sort_by(|a,b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        for (_,k) in scored {
            let p = candidates[k];
            if self.side(p) > 0 || !self.encroached(p,t)?.is_empty() { continue; }
            return Ok(Some(p));
        }
        Ok(None)
    }

    /// The least dihedral angle of the tetrahedra `p` would make with its conflict region's
    /// boundary faces, those on no enclosing vertex; 0 where it is in conflict with nothing.
    fn star_quality(&mut self,p: P,hint: u32) -> Result<f64,String> {
        let mut region = self.tri.conflicts(p,0.,Some(hint))?;
        if region.is_empty() { return Ok(0.); }
        region.sort_unstable();
        let mut least = 180f64;
        for &u in &region {
            let tet = self.tri.tet(u);
            for i in 0..4 {
                let n = tet.n[i];
                if n != NONE && region.binary_search(&n).is_ok() { continue; }
                if tet.v.iter().enumerate().any(|(j,&v)| j != i && v < ENCLOSING) { continue; }
                let mut q = self.corners(u);
                q[i] = p;
                least = least.min(tetra::least_dihedral(q));
            }
        }
        Ok(least)
    }
}
