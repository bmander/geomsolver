//! `SOLVENT_REFINE_TRACE=1`: what a refinement says as it goes, for someone finding out why one
//! will not finish — every line of it here, and nothing it says read by anything else.
use super::*;
use super::manifold::non_manifold;

/// Whether the trace is on (read once).
fn on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_REFINE_TRACE").is_some())
}

impl Refiner<'_> {
    /// A point inserted within twice the bisection of a vertex its tetrahedra hold.
    pub(super) fn trace_inserted(&self,p: P,created: &[u32],judging: bool) {
        if !on() { return; }
        let near = created.iter().flat_map(|&t| self.tri.tet(t).v).filter(|&v| v >= ENCLOSING)
            .map(|v| self.tri.points()[v as usize].p).filter(|&q| q != p)
            .map(|q| dist2(p,q)).fold(f64::INFINITY,f64::min).sqrt();
        if near <= 2.*self.criteria.bisection {
            eprintln!("refine: inserted {p:?} {near:.3e} from a vertex (judging {judging}, {} inserted)",self.report.inserted);
        }
    }

    /// Every 250 insertions, the facet being refined.
    pub(super) fn trace_refining(&self,key: [u32;3],badness: f64,centre: P) {
        if !on() || self.report.inserted % 250 != 0 { return; }
        let [a,b,c] = key.map(|v| self.tri.points()[v as usize].p);
        let edges = [dist2(a,b).sqrt(),dist2(b,c).sqrt(),dist2(c,a).sqrt()];
        eprintln!("refine: {} inserted, {} in balls, queue {}: facet edges {:?}, badness {badness:.2} at {:?}",
            self.report.inserted,self.report.in_balls,self.queue.len(),edges.map(|x| (x*1e4).round()/1e4),
            centre.map(|x| (x*1e4).round()/1e4));
    }

    /// How many facets of a closed surface stand off it.
    pub(super) fn trace_standing_off(&self,rebuild: usize,off: usize) {
        if on() && off > 0 { eprintln!("refine: rebuild {rebuild}: {off} facets stand off the surface"); }
    }

    /// A surface left not manifold: its faults and the balls that blocked their repair.
    pub(super) fn trace_faults(&mut self,rebuild: usize,blocking: &[usize]) {
        if !on() { return; }
        let faults = non_manifold(&self.extract());
        eprintln!("refine: rebuild {rebuild}: {} faults (first at {:?}), {} points, {} queries; {} blocking balls, {} balls",
            faults.len(),faults.first().map(|f| self.tri.orthosphere(f.0).0),self.kept.len(),self.report.queries,blocking.len(),
            self.balls.len());
        for &b in blocking.iter().take(6) { eprintln!("refine:   ball {b} at {:?} r {:.3e}",self.balls[b].0,self.balls[b].1); }
        for &(t,i,key) in faults.iter().take(8) {
            let n = self.tri.tet(t).n[i];
            eprintln!("refine:   fault {key:?} sides {} {}: {:?}",self.tet_side(t),self.tet_side(n),
                key.map(|v| { let q = self.tri.points()[v as usize]; (q.p.map(|x| (x*1e4).round()/1e4),(q.w.max(0.).sqrt()*1e4).round()/1e4) }));
        }
    }
}
