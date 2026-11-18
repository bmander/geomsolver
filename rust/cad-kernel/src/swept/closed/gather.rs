//! **The candidates a closed sweep's box is split by, gathered** (`build_by`): each leg of the path
//! read (`Leg`), and every surface its tool's faces, edges and rims sweep entered once
//! (`Candidates`), each with the sites it is read at and the touches known by construction between
//! them — under a slide (`super::slide`), or under a turn or a screw (`super::twist`).
use crate::geom::{Curve,Surface};
use crate::query::{Located,Place};
use crate::topo::{Brep,EdgeCurve};
use crate::trajectory::{Pose,Screw,Trajectory};
use crate::util::space::{add,cross,dot,norm,scale,sub};
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::{Cutter,Legs,Read,Touch,Unbuilt,V,SITES,corners_of,meridian,outward,read,rulings_span,within};

/// **The candidates gathered**, over every leg: the surfaces the box is split by, the touches known
/// between them, and what the box must hold.
pub(super) struct Candidates {
    /// Each candidate with the sites it is read at for whether it carries any of the boundary: the
    /// points of its source carried over the roll, each with the candidate's normal there.
    pub(super) cutters: Vec<(Cutter,Vec<(V,V)>)>,
    /// The touches known by construction (#810, #780 R5), each naming two candidates by their index
    /// among `cutters` (`add` answers the one it settled on).
    pub(super) touches: Vec<Touch>,
    /// The points where a contact crosses a seam, which no candidate's seam is laid at
    /// (`seam_away`).
    pub(super) seams: Vec<V>,
    /// The points the box must hold: the tool's box at each leg's end poses and, about a turn's
    /// axis, its reach.
    pub(super) held: Vec<V>,
    eps: f64,
}

impl Candidates {
    /// Every candidate the legs' tools sweep, each leg read in turn.
    pub(super) fn gathered(tool: &Brep,legs: &Legs,tol: f64,eps: f64,size: f64) -> Result<Candidates,Unbuilt> {
        let mut out = Candidates {cutters:Vec::new(),touches:Vec::new(),seams:Vec::new(),held:Vec::new(),eps};
        for &(screw,roll,place) in legs {
            // the tool where the leg takes it up, read under the leg's twist
            let placed = place.map(|q| tool.moved(&crate::geom::Rigid {r:q.r,t:q.p}));
            let leg = Leg::new(placed.as_ref().unwrap_or(tool),screw,roll,tol,eps,size)?;
            out.held.extend(leg.held());
            if screw.is_slide() { super::slide::gather(&leg,&mut out) } else { super::twist::gather(&leg,&mut out)? }
        }
        Ok(out)
    }

    /// Enters the candidate `c` read at `sites`, answering its index among `cutters`: one surface
    /// already entered from another source (`same`) is that one, read at both's sites, over both
    /// their spans.
    pub(super) fn add(&mut self,c: Cutter,sites: Vec<(V,V)>) -> usize {
        match self.cutters.iter().position(|(k,_)| same(k,c,self.eps)) {
            Some(i) => {
                let (k,known) = &mut self.cutters[i];
                if let (Cutter::Cylinder {o,axis,span,..} | Cutter::Revolved {o,axis,span,..},
                    Cutter::Cylinder {o:from,axis:way,span:more,..} | Cutter::Revolved {o:from,axis:way,span:more,..}) = (k,c) {
                    // the other's heights read from this one's origin along this one's axis
                    let (shift,sense) = (dot(sub(from,*o),*axis),dot(way,*axis).signum());
                    let more = [shift+sense*more[0],shift+sense*more[1]];
                    *span = [span[0].min(more[0].min(more[1])),span[1].max(more[0].max(more[1]))];
                }
                known.extend(sites);
                i
            }
            None => { self.cutters.push((c,sites)); self.cutters.len()-1 }
        }
    }
}

/// Whether the candidate `k` already entered and `c` are one surface, to `eps` along and `1e-9`
/// across.
fn same(k: &Cutter,c: Cutter,eps: f64) -> bool {
    match (*k,c) {
        (Cutter::Plane((p,m)),Cutter::Plane((o,n))) => norm(cross(n,m)) <= 1e-9 && dot(sub(o,p),m).abs() <= eps,
        (Cutter::Ball {c:c1,r:r1,..},Cutter::Ball {c:c2,r:r2,..}) => norm(sub(c1,c2)) <= eps && (r1-r2).abs() <= eps,
        (Cutter::Cylinder {o:o1,axis:a1,r:r1,..},Cutter::Cylinder {o:o2,axis:a2,r:r2,..}) => (r1-r2).abs() <= eps
            && norm(cross(a1,a2)) <= 1e-9 && norm(cross(sub(o2,o1),a1)) <= eps,
        (Cutter::Revolved {o,axis,p:p1,e:e1,..},Cutter::Revolved {o:o2,axis:a2,p:p2,e:e2,..}) => {
            // the same surface: about one axis, the same throat at the same height, and the same
            // lean
            let (m1,m2) = (meridian(o,axis,p1,e1),meridian(o,axis,p2,e2));
            norm(cross(axis,a2)) <= 1e-9 && norm(cross(sub(o2,o),axis)) <= eps
                && (m1.0-m2.0).abs() <= eps && (m1.1-m2.1).abs() <= eps && (m1.2-m2.2).abs() <= 1e-9
        }
        (Cutter::Torus {c:c1,axis:a1,major:m1,minor:n1},Cutter::Torus {c:c2,axis:a2,major:m2,minor:n2}) =>
            norm(sub(c1,c2)) <= eps && norm(cross(a1,a2)) <= 1e-9 && (m1-m2).abs() <= eps && (n1-n2).abs() <= eps,
        // two arcs of one sharp circle turned: one ring (#780 L2)
        (Cutter::Turned(t1),Cutter::Turned(t2)) => match (t1.source,t2.source) {
            (crate::turned::Source::Rim(x),crate::turned::Source::Rim(y)) => t1.frame == t2.frame && norm(sub(x.o,y.o)) <= eps
                && (x.r-y.r).abs() <= eps && norm(cross(cross(x.x,x.y),cross(y.x,y.y))) <= 1e-9,
            _ => false,
        },
        _ => false,
    }
}

/// **A leg of the path, read** (`legs`): one constant twist over its stretch of the roll, the tool
/// where the leg takes it up, what `read` reads of it, and the poses its sources are carried by.
pub(super) struct Leg<'a> {
    /// The tool, placed where the leg takes it up.
    pub(super) tool: &'a Brep,
    /// Where a point stands on the tool's faces, at `tol`.
    pub(super) located: Located<'a>,
    pub(super) screw: Screw,
    /// The leg's stretch of the roll, `[a, b]`.
    pub(super) roll: [f64;2],
    pub(super) read: Read,
    /// Each face's corners in walk order, what its own sites are read over.
    pub(super) corners: Vec<Vec<V>>,
    /// For a face with an edge no line (a drum's end, its loop one circle on one vertex), its
    /// outline: each edge's start in walk order, and 32 points along each edge no line (#780 L2) —
    /// none where every edge is a line, whose corners are its outline.
    pub(super) outlines: Vec<Option<Vec<V>>>,
    /// The poses at the leg's ends.
    pub(super) start: Pose,
    pub(super) end: Pose,
    /// The poses the sites are carried by, across the roll (inside the roll and inside each
    /// source, a site at a source's own edge reading its neighbours' boundary, not its own).
    pub(super) poses: Vec<Pose>,
    pub(super) tol: f64,
    /// The whole tool's tolerance of length and size (not the placed tool's: one for every leg).
    pub(super) eps: f64,
    pub(super) size: f64,
}

impl<'a> Leg<'a> {
    fn new(tool: &'a Brep,screw: Screw,roll: [f64;2],tol: f64,eps: f64,size: f64) -> Result<Leg<'a>,Unbuilt> {
        let path = Trajectory::Twist(screw);
        let read = read(tool,&path)?;
        let corners = tool.faces.iter().map(|f| f.loops.iter().flatten()
            .map(|c| tool.vertices[tool.ends(c)[0] as usize].p).collect()).collect();
        let outlines = tool.faces.iter().map(|f| {
            let straight = |c: &crate::topo::Coedge| matches!(tool.edges[c.edge as usize].curve,EdgeCurve::Curve(Curve::Line {..}));
            (!f.loops.iter().flatten().all(straight)).then(|| f.loops.iter().flatten().flat_map(|c| {
                let e = &tool.edges[c.edge as usize];
                let [t0,t1] = if c.reversed { [e.t[1],e.t[0]] } else { e.t };
                let n = if straight(c) { 1 } else { 32 };
                (0..n).map(move |i| e.point(t0+(t1-t0)*i as f64/n as f64,&tool.vertices))
            }).collect())
        }).collect();
        let [a,b] = roll;
        let (start,end) = (path.pose(a)?,path.pose(b)?);
        let poses = (0..=SITES).map(|k| path.pose(a+(b-a)*within(k))).collect::<Result<_,_>>()?;
        Ok(Leg {tool,located:Located::new(tool,tol),screw,roll,read,corners,outlines,start,end,poses,tol,eps,size})
    }

    /// The pose at `t` of the roll.
    pub(super) fn pose_at(&self,t: f64) -> Result<Pose,Unbuilt> { Ok(Trajectory::Twist(self.screw).pose(t)?) }

    /// The point `x` and the vector `n` carried by each of the leg's poses.
    pub(super) fn carry(&self,x: V,n: V) -> impl Iterator<Item=(V,V)> + '_ {
        self.poses.iter().map(move |m| (m.point(x),m.vector(n)))
    }

    /// The points the box must hold for this leg: the tool's box at its end poses and, about a
    /// turn's axis, its reach.
    fn held(&self) -> Vec<V> {
        let corners = corners_of(self.tool.bounds());
        let mut out: Vec<V> = corners.iter().flat_map(|&p| [self.start.point(p),self.end.point(p)]).collect();
        let screw = &self.screw;
        if screw.is_turn() {
            let reach = corners.iter().map(|&p| norm(sub(p,screw.origin))).fold(0.,f64::max);
            out.extend([1.,-1.].iter().flat_map(|&s| (0..3).map(move |k| { let mut v = [0.;3]; v[k] = s*reach; v }))
                .map(|v| add(screw.origin,v)));
        }
        out
    }

    /// A plane face's own points, a grid over each triangle of a fan of its corners — or of its
    /// outline, each grid as much finer as the fan has more triangles, so a face of many is read at
    /// about as many points as a triangle.
    pub(super) fn face_sites(&self,k: usize) -> Vec<V> {
        let (c,m) = match &self.outlines[k] {
            Some(c) => (c,((SITES as f64/(c.len().saturating_sub(2).max(1) as f64).sqrt()).ceil() as usize).max(2)),
            None => (&self.corners[k],SITES),
        };
        let mut out = Vec::new();
        for w in 1..c.len().saturating_sub(1) {
            for i in 0..m { for j in 0..m-i {
                let (u,v) = ((i as f64+1./3.)/(m+1) as f64,(j as f64+1./3.)/(m+1) as f64);
                out.push(std::array::from_fn(|q| c[0][q]+u*(c[w][q]-c[0][q])+v*(c[w+1][q]-c[0][q])));
            } }
        }
        out
    }

    /// The points of the circle about `centre` of radius `r` in the plane of the unit `u` and `v`
    /// that are on the tool's face `k`, each with its way out from the centre.
    pub(super) fn ring(&self,k: usize,centre: V,u: V,v: V,r: f64) -> Vec<(V,V)> {
        (0..4*SITES).filter_map(|i| {
            let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
            let w = add(scale(u,cp),scale(v,sp));
            let x = add(centre,scale(w,r));
            (self.located.face_place(k,x) != Place::Out).then_some((x,w))
        }).collect()
    }
}

/// **A round face of a leg's tool, read**: its index, the span its own points are read over (a
/// ball's every latitude, its pole on no edge; a torus's every angle round its tube; a cylinder's
/// or a cone's along its rulings, `rulings_span`), and those points (`round_points`).
pub(super) struct Round<'a> {
    pub(super) k: usize,
    pub(super) face: &'a crate::topo::Face,
    pub(super) span: [f64;2],
    points: Vec<V>,
}

impl<'a> Round<'a> {
    pub(super) fn new(leg: &Leg<'a>,k: usize) -> Round<'a> {
        let tool: &'a Brep = leg.tool;
        let face = &tool.faces[k];
        let (lo,hi) = match face.surface {
            Surface::Sphere(..) => (-std::f64::consts::FRAC_PI_2,std::f64::consts::FRAC_PI_2),
            Surface::Torus(..) => (0.,std::f64::consts::TAU),
            _ => rulings_span(tool,k),
        };
        Round {k,face,span:[lo,hi],points:round_points(&leg.located,tool,k,[lo,hi])}
    }

    /// Its own points at the pose `m`, each with its outward normal there.
    pub(super) fn sites_at(&self,m: &Pose) -> Vec<(V,V)> {
        self.points.iter().map(|&x| (m.point(x),m.vector(outward(self.face,x)))).collect()
    }

    /// Its own points at each of `poses` in turn, as `sites_at`.
    pub(super) fn sites_over(&self,poses: &[Pose]) -> Vec<(V,V)> { poses.iter().flat_map(|m| self.sites_at(m)).collect() }
}

/// A round face's own points: its surface over a grid of its parameters round the axis and along
/// its rulings over `span`, those in the face.
fn round_points(located: &Located,tool: &Brep,k: usize,[lo,hi]: [f64;2]) -> Vec<V> {
    let surface = &tool.faces[k].surface;
    let mut out = Vec::new();
    for i in 0..2*SITES { for j in 0..SITES {
        let (u,v) = ((i as f64+0.5)*std::f64::consts::TAU/(2*SITES) as f64,lo+(hi-lo)*(j as f64+0.5)/SITES as f64);
        let p = surface.point([u,v]);
        if located.face_place(k,p) == Place::In { out.push(p) }
    } }
    out
}
