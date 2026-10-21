//! The refinement a step at a time (`Progressive`): build, refine, repair, and rebuild with
//! shrunk balls where they blocked a repair or held a facet off the surface.
use super::*;
use super::manifold::{repair,standing_off};
use super::protect::{protect,shrink,Sizing,LEAST,REBUILDS};
use super::seed::{lattice,seed};
use crate::space::polyline_length;

/// A refinement's state for a host to show (`Progressive::progress`). `worst` is the badness of
/// the worst facet waiting — above 1 by how far it misses the criteria, 1 with none waiting — and
/// falls toward 1 as refinement goes on, worst first; it is the one measure of how near the end is.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Progress {
    pub stage: Stage,
    pub rebuild: usize,
    pub queued: usize,
    pub worst: f64,
    pub inserted: usize,
    pub queries: usize,
    pub readings: usize,
}

/// Which stage a refinement is at (`Progress::stage`), and the word a host shows for it.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Stage { Building, Refining, Repairing, Done, Failed }

impl Stage {
    /// The word for the stage: `building`, `refining`, `repairing`, `done` or `failed`.
    pub fn word(self) -> &'static str {
        match self {
            Stage::Building => "building",
            Stage::Refining => "refining",
            Stage::Repairing => "repairing",
            Stage::Done => "done",
            Stage::Failed => "failed",
        }
    }
}

/// Where a `Progressive` is, holding what it finished with.
enum State {
    /// Protect, seed or re-insert, and judge every facet.
    Build,
    /// Refine the queued facets, worst first.
    Refine,
    /// Repair the manifold, check what the balls hold off the surface, and finish or rebuild.
    Repair,
    Done(Result<Mesh,String>),
}

/// **A refinement that can be stopped and looked at**: `step` does a bounded amount of work, and
/// `snapshot` gives the restricted facets as they stand — worst first means the first snapshots
/// are the coarse shape and every later one a finer version of it, which is what a preview shows
/// while the refinement goes on. A snapshot before the end may be open or not manifold.
///
/// Refinement that a ball blocks is the ball's to give way: the curves are sampled twice as
/// finely where balls blocked a manifold repair, graded back to their spacing, and the
/// triangulation rebuilt with the new balls and every point kept so far (Mesh_3 shrinks balls in
/// place; this triangulation removes no vertex, and a rebuild costs a few microseconds a point,
/// the domain's answers being remembered).
pub struct Progressive<'a> {
    /// The domain until the first build hands it to its refiner, which every rebuild takes it from.
    domain: Option<Box<dyn Domain + 'a>>,
    refiner: Option<Refiner<'a>>,
    centre: P,
    radius: f64,
    curves: Vec<Vec<P>>,
    criteria: Criteria,
    sizing: Vec<Sizing>,
    kept: Vec<P>,
    memo: HashMap<[u64;3],i8>,
    crossings: HashMap<[u64;7],P>,
    queries: usize,
    rebuild: usize,
    state: State,
    /// The last closed, manifold surface a build reached, kept while balls are shrunk for facets
    /// that stood off it: shrinking is an improvement, and one that fails leaves this standing.
    manifold: Option<Mesh>,
}

impl<'a> Progressive<'a> {
    pub fn new(domain: Box<dyn Domain + 'a>,centre: P,radius: f64,curves: Vec<Vec<P>>,criteria: Criteria) -> Self {
        let sizing = curves.iter().map(|c| Sizing {base:polyline_length(c).min(criteria.edge_size).max(criteria.edge_size*1e-3),local:Vec::new()}).collect();
        Self {domain:Some(domain),refiner:None,centre,radius,curves,criteria,sizing,kept:Vec::new(),
            memo:HashMap::new(),crossings:HashMap::new(),queries:0,rebuild:0,state:State::Build,manifold:None}
    }

    /// Do at most about `budget` refinements (a build or a repair runs whole): whether the
    /// refinement has finished, well or not — `finished` says which.
    pub fn step(&mut self,budget: usize) -> Result<bool,String> {
        match self.state {
            State::Done(_) => return Ok(true),
            State::Build => { self.build()?; self.state = State::Refine; }
            State::Refine => {
                let r = self.refiner.as_mut().expect("a refiner while refining");
                if r.refine_some(budget)? { self.state = State::Repair; }
            }
            State::Repair => {
                let outcome = self.repair();
                if let Some(result) = outcome { self.state = State::Done(result); }
            }
        }
        Ok(matches!(self.state,State::Done(_)))
    }

    /// Whether the refinement has finished, well or not.
    pub fn done(&self) -> bool { matches!(self.state,State::Done(_)) }

    /// The refinement's counts so far (the build in hand's; `queries` over every build).
    pub fn report(&self) -> Report {
        self.refiner.as_ref().map_or(Report {queries:self.queries,..Report::default()},|r| r.report.clone())
    }

    /// Where the refinement stands, for a host to show: which stage, how many rebuilds, the
    /// facets waiting and the worst of them, and the work done. No total is known in advance.
    pub fn progress(&self) -> Progress {
        let stage = match self.state {
            State::Build => Stage::Building,
            State::Refine => Stage::Refining,
            State::Repair => Stage::Repairing,
            State::Done(Ok(_)) => Stage::Done,
            State::Done(Err(_)) => Stage::Failed,
        };
        let (queued,worst,report) = match &self.refiner {
            Some(r) => (r.queue.len(),r.queue.peek().map_or(1.,|b| b.badness),r.report.clone()),
            None => (0,1.,Report {queries:self.queries,..Report::default()}),
        };
        Progress {stage,rebuild:self.rebuild,queued,worst,inserted:report.inserted,queries:report.queries,
            readings:report.readings}
    }

    /// Why a finished refinement could not make a mesh, if it could not.
    pub fn done_error(&self) -> Option<String> {
        match &self.state { State::Done(Err(e)) => Some(e.clone()), _ => None }
    }

    /// The mesh a finished refinement made, or why it could not.
    pub fn finished(self) -> Result<Mesh,String> {
        match self.state {
            State::Done(result) => result,
            _ => Err("the refinement has not finished".into()),
        }
    }

    /// The restricted facets as they stand, oriented outward; empty before the first build.
    pub fn snapshot(&mut self) -> Mesh {
        if let State::Done(Ok(m)) = &self.state { return m.clone(); }
        let Some(r) = self.refiner.as_mut() else { return Mesh {vertices:Vec::new(),triangles:Vec::new(),report:Report::default()} };
        let facets = r.extract();
        let (vertices,triangles) = collect(r,&facets);
        Mesh {vertices,triangles,report:r.report.clone()}
    }

    fn build(&mut self) -> Result<(),String> {
        let criteria = &self.criteria;
        let (balls,owners) = protect(&self.curves,criteria.edge_size,&mut self.sizing)?;
        let cell = balls.iter().map(|b| b.1).fold(criteria.edge_size,f64::max).max(self.radius*1e-9);
        let domain = match self.refiner.take() {
            Some(old) => {
                (self.kept,self.memo,self.crossings,self.queries) = (old.kept,old.memo,old.crossings,old.report.queries);
                old.domain
            }
            None => self.domain.take().expect("the domain before the first build"),
        };
        let mut r = Refiner {domain,centre:self.centre,radius:self.radius,criteria:criteria.clone(),
            tri:Regular::new(self.centre,self.radius),sign:Vec::new(),balls:Vec::new(),owners:Vec::new(),
            ball_of:HashMap::new(),kept:Vec::new(),memo:std::mem::take(&mut self.memo),normals:HashMap::new(),
            crossings:std::mem::take(&mut self.crossings),blocking:Vec::new(),
            grid:HashMap::new(),cell,queue:BinaryHeap::new(),report:Report {queries:self.queries,..Report::default()}};
        // Protecting balls first, in spatial order.
        let order = crate::delaunay::spatial_order(&balls.iter().map(|b| b.0).collect::<Vec<_>>());
        for &k in &order {
            let (p,rad) = balls[k];
            if let Inserted::Vertex(v) = r.tri.insert(p,rad*rad)? { r.ball_of.insert(v,r.balls.len()); }
            let key = r.cell_of(p);
            r.grid.entry(key).or_default().push(r.balls.len());
            r.balls.push((p,rad));
            r.owners.push(owners[k].clone());
        }
        r.report.balls = r.balls.len();
        r.report.rebuilds = self.rebuild;
        if self.rebuild == 0 {
            seed(&mut r)?;
            // judged, so that whether the rays found any surface is known
            let tets: Vec<u32> = r.tri.tets().collect();
            for t in tets { for i in 0..4 { r.judge(t,i); } }
            if r.balls.is_empty() || (r.queue.is_empty() && r.extract().is_empty()) { lattice(&mut r)?; }
            r.queue.clear();          // judged again below, with whatever the lattice added
        } else {
            let kept = std::mem::take(&mut self.kept);
            for &k in &crate::delaunay::spatial_order(&kept) { r.insert_judging(kept[k],false)?; }
            r.blocking.clear();
        }
        // Every facet now; refinement follows in steps.
        let tets: Vec<u32> = r.tri.tets().collect();
        for t in tets { for i in 0..4 { r.judge(t,i); } }
        self.refiner = Some(r);
        Ok(())
    }

    /// Repair and judge the refined surface: the finished mesh, a refusal, or `None` after
    /// arranging a rebuild with shrunk balls.
    fn repair(&mut self) -> Option<Result<Mesh,String>> {
        let rebuild = self.rebuild;
        let r = self.refiner.as_mut().expect("a refiner while repairing");
        let manifold = match repair(r) { Ok(m) => m, Err(e) => return Some(Err(e)) };
        if manifold {
            let facets = r.extract();
            // A facet with a protecting ball for a vertex may stand off the surface where no
            // refinement could reach it, its surface centre inside the ball: those balls are too
            // big for the surface's curvature there, and shrink as a repair's blocking balls do.
            let off = standing_off(r,&facets);
            r.report.coarse = off.len();
            r.trace_standing_off(rebuild,off.len());
            let blocking: Vec<usize> = off.iter().flatten().copied().collect();
            let (vertices,triangles) = collect(r,&facets);
            let mesh = Mesh {vertices,triangles,report:r.report.clone()};
            // Shrinking that left no fewer facets standing off than before has not helped, and
            // another round would only pay for the whole surface again: keep the earlier one.
            if let Some(earlier) = self.manifold.take_if(|m| m.report.coarse <= off.len()) { return Some(Ok(earlier)); }
            if blocking.is_empty() || rebuild+1 == REBUILDS || !shrink(r,&blocking,&mut self.sizing,self.criteria.edge_size/LEAST) {
                return Some(Ok(mesh));
            }
            self.manifold = Some(mesh);
        } else {
            let mut blocking = r.blocking.clone();
            blocking.sort_unstable();
            blocking.dedup();
            let shrunk = shrink(r,&blocking,&mut self.sizing,self.criteria.edge_size/LEAST);
            r.trace_faults(rebuild,&blocking);
            // A closed surface reached before is better than this one: the shrinking that followed
            // it was for facets standing off it, and has opened the surface instead.
            if self.manifold.is_some() { return Some(Ok(self.manifold.take().unwrap())); }
            if !shrunk {
                return Some(Err("the boundary is not a manifold, and no ball blocking its repair can shrink".into()));
            }
            if rebuild+1 == REBUILDS {
                return Some(Err(format!("the boundary is not a manifold after refining the protecting balls {REBUILDS} times")));
            }
        }
        self.rebuild += 1;
        self.state = State::Build;
        None
    }
}
