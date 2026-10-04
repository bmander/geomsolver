//! Sweeps of constant twist (docs/generating-sweeps.md): a tool carried by a screw. A screw moves
//! every point of the tool the same way, relative to the tool, at every time. So whether a tool
//! point is on the swept boundary does not depend on the time. The points that are form the
//! tool's **characteristic**, a curve fixed on the tool, and the boundary is that curve carried by
//! the screw, S(s, t) = M(t)·c(s), whose normal is the tool's there. On a revolved face, the
//! characteristic's ring at meridian station u is A cos θ + B sin θ + C = 0
//! (`RevolvedSurface::contact_coefficients`): two roots, or none. Nothing is traced.
//!
//! `Characteristic::walk` finds the stretch of it that reaches the blank by walking one root
//! from ring to ring and across the faces of the tool's profile. It asks the class's rows of
//! that stretch as it goes: regular (S1), one stretch and one root a ring (S2), no fold (S3),
//! and no crossing within a lead (S4). The sheet is built from the same walk
//! (`brep::sweep::helical`). Like the generating class's checks, these are **sampled**: rings
//! `rows` apart along each face, and each point's path through the blank at a degree's turn.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::admission::Condition;
use super::{RevolvedSurface,SweepContacts};
use crate::envelope::{self,Motion};
use crate::motion::{Family,Screw};
use crate::space::{sub,dot,cross,norm,distance};

type V = [f64;3];

/// A row of the class that fails, what was seen and where: in the sweep's frame, at a time the
/// tool point is in the blank.
pub(super) type Failure = (Condition,String,Option<V>);

/// How many rings past the last that reaches the blank the walk goes on each side: the sheet's
/// margin, which carries its edge out of the blank.
const MARGIN: usize = 3;
/// The least sine of the half angle between a ring's two roots for a characteristic to be
/// regular there: below it the roots are about to meet, where the curve turns back on the ring.
const LEAST_SPREAD: f64 = 1e-3;
/// How far apart, in turns of the screw, a point's path through the blank is read.
const PATH_STEP: f64 = 1./360.;

/// One root of one ring: where it is on the tool, its unit normal (the face's, as its parameters
/// orient it), the sine of its spread from the other root, and when its path first enters the blank.
#[derive(Clone,Copy,Debug)]
struct Root { position: V,normal: V,spread: f64,reach: Option<f64> }

/// A face's ring at one station: its roots, or why it has none to give.
#[derive(Clone,Debug)]
enum Ring { Roots(Vec<Root>),Degenerate(String) }

/// A point of the walk: the face, the station on it, and the station the step to it began at on
/// the same face (the station of the edge it crossed onto this face from, after a crossing), and
/// the first time its path enters the blank, if it does.
#[derive(Clone,Copy,Debug)]
pub struct Node { pub patch: usize,pub u: f64,pub from: f64,pub position: V,pub normal: V,pub reach: Option<f64> }

/// The stretch of a screw sweep's characteristic that reaches the blank, with `MARGIN` rings of
/// it beyond each end. The parameter `s` runs from 0 to `nodes.len() - 1`: whole values are the
/// walk's nodes, and between two the station runs linearly on the later node's face.
#[derive(Clone,Debug)]
pub struct Characteristic {
    pub screw: Screw,
    pub nodes: Vec<Node>,
    /// The first and last nodes that reach the blank.
    pub reach: [usize;2],
    /// The least sine between the characteristic's tangent and the screw's velocity in the reach.
    pub least_factor: f64,
    /// How many rings were sampled, over every face.
    pub samples: usize,
    patches: Vec<RevolvedSurface>,
    tool: super::SpatialField,
    motion: Motion,
    tolerance: f64,
}

/// What the walk is asked with: the rings a face is sampled at, the bars, the roll, and how far
/// along the screw's axis the blank reaches (in the sweep's frame), where a path is read.
pub struct Ask<'a> {
    pub rows: usize,
    pub root_tolerance: f64,
    pub least_factor: f64,
    pub roll: [f64;2],
    pub heights: Option<[f64;2]>,
    /// How far from the screw's axis the blank reaches, where known: a point farther never enters.
    pub radius: Option<f64>,
    pub inside: &'a (dyn Fn(V) -> bool+Sync),
}

/// What every pass of the walk reads: the sweep, the ask, the screw and the twist the rings are
/// solved under, the tool's material, its size and the roots' tolerance.
struct Walker<'a> { contacts: &'a SweepContacts,patches: Vec<RevolvedSurface>,ask: &'a Ask<'a>,screw: Screw,motion: Motion,
    tool: &'a super::SpatialField,scale: f64,tolerance: f64 }

/// One pass of the walk: the stations each face was sampled at, its rings there, and the stretch
/// walked through the blank (face, ring, root), in order along the characteristic.
struct Pass { stations: Vec<Vec<f64>>,rings: Vec<Vec<Ring>>,order: Vec<(usize,usize,usize)> }

impl Pass {
    fn root(&self,x: (usize,usize,usize)) -> &Root { root(&self.rings,x) }
    fn samples(&self) -> usize { self.rings.iter().map(Vec::len).sum() }
}

impl Walker<'_> {
    /// Sample each face's rings at its stations, and walk the stretch that reaches the blank from
    /// its first point to `MARGIN` rings past its last each way: S2 asked of every ring and of the
    /// stretch, and the walk's own end (a double point, a jump at a corner) refused as S1.
    fn pass(&self,stations: Vec<Vec<f64>>) -> Result<Pass,Failure> {
        let (contacts,ask,screw,family) = (self.contacts,self.ask,&self.screw,self.contacts.motion());
        let patches = &self.patches;
        let on_tool = |p: V| self.tool.value(p).abs() < 1e-6;
        let outward = |p: V,n: V| outward(self.tool,p,n);
        // where each ring's roots are, and whether each one's path enters the blank: on every core
        let rings: Vec<Vec<Ring>> = crate::par::indices(patches.len(),|k| stations[k].iter().map(|&u|
            ring(&patches[k],u,self.motion,self.tolerance,&outward,&|p| on_tool(p).then(|| reach(family,screw,p,ask)).flatten())).collect());
        // S2, ring by ring: two roots of one ring in the blank are two sheets over one station
        for (k,face) in rings.iter().enumerate() { for ring in face {
            let Ring::Roots(roots) = ring else { continue };
            let reaching: Vec<&Root> = roots.iter().filter(|r| r.reach.is_some()).collect();
            if reaching.len() > 1 {
                return Err((Condition::Once,format!("both of a ring's characteristic points on `{}` reach the blank",patches[k].name),
                    witness(family,reaching[1].reach,reaching[1].position)))
            }
        } }
        let reaching: Vec<(usize,usize,usize)> = rings.iter().enumerate().flat_map(|(k,face)| face.iter().enumerate()
            .flat_map(move |(i,ring)| match ring { Ring::Roots(roots) => roots.iter().enumerate()
                .filter(|(_,r)| r.reach.is_some()).map(|(j,_)| (k,i,j)).collect::<Vec<_>>(),_ => Vec::new() })).collect();
        let Some(&seed) = reaching.first() else {
            return Err((Condition::Reach,"no point of the tool's characteristic passes through the blank in the roll".into(),None))
        };
        let root = |x| root(&rings,x);
        // the walk each way from the seed, to MARGIN rings past the last that reaches
        let walk = |dir: isize| -> Result<Vec<(usize,usize,usize)>,Failure> {
            let mut out = Vec::new();
            let (mut at,mut dir) = (seed,dir);
            let mut since = 0;
            loop {
                match step(contacts,&stations,&rings,at,dir,self.scale) {
                    Step::Next(next,d) => {
                        if next == seed { return Err((Condition::Regular,format!("the characteristic closes on itself through \
                            `{}` within the blank's reach",patches[seed.0].name),witness(family,root(seed).reach,root(seed).position))) }
                        let duplicate = next.0 != at.0;
                        out.push(next);
                        at = next; dir = d;
                        if duplicate { continue }
                        if root(at).reach.is_some() { since = 0 } else { since += 1 }
                        if since == MARGIN { return Ok(out) }
                    }
                    Step::End(why) => {
                        let last = out.iter().rev().chain(std::iter::once(&seed)).find(|&&x| root(x).reach.is_some()).copied().unwrap_or(seed);
                        return Err((Condition::Regular,format!("the characteristic on `{}` {why} within {MARGIN} rings of the blank",
                            patches[at.0].name),witness(family,root(last).reach,root(last).position)))
                    }
                }
            }
        };
        let (back,ahead) = (walk(-1)?,walk(1)?);
        let order: Vec<(usize,usize,usize)> = back.into_iter().rev().chain(std::iter::once(seed)).chain(ahead).collect();
        // S2: every ring point that reaches the blank is on this one stretch
        if let Some(&other) = reaching.iter().find(|x| !order.contains(x)) {
            return Err((Condition::Once,format!("the characteristic reaches the blank in more than one stretch (again on `{}`)",
                patches[other.0].name),witness(family,root(other).reach,root(other).position)))
        }
        Ok(Pass {stations,rings,order})
    }
}

impl Characteristic {
    /// Walk the characteristic of `contacts`' tool under its screw through the blank `ask.inside`
    /// reads, or refuse with the row it fails. A coarse pass, `rows` rings along every face, finds
    /// the stretch that reaches the blank; a fine pass then samples that stretch alone `rows` times
    /// across the reach, so the rows are asked as finely whether the reach is a whole face or a
    /// sliver of a large one.
    pub fn walk(contacts: &SweepContacts,ask: &Ask) -> Result<Characteristic,Failure> {
        let family = contacts.motion();
        let screw = family.screw().ok_or((Condition::Motion,"the motion is not a screw".to_string(),None))?;
        let motion = family.at(ask.roll[0]).map_err(|m| (Condition::Motion,m,None))?;
        let tool = contacts.source_material();
        // a screw's characteristic is read ring by ring of a revolved face; a prism's side has none
        let patches: Vec<RevolvedSurface> = contacts.patches().iter().map(|s| s.revolved().cloned().ok_or_else(|| (Condition::Tool,
            format!("a screw's tool is a full revolution: `{}` is a prism's side",s.name()),None))).collect::<Result<_,_>>()?;
        let scale = patches.iter().filter_map(|s| s.at(0.,0.).ok()).map(|s| norm(s.position)).fold(1_f64,f64::max);
        let tolerance = ask.root_tolerance*scale;
        let walker = Walker {contacts,patches:patches.clone(),ask,screw,motion,tool,scale,tolerance};
        let n = ask.rows;
        let coarse = walker.pass((0..patches.len()).map(|_| (0..=n).map(|i| i as f64/n as f64).collect()).collect())?;
        let mut samples = coarse.samples();
        let reached = coarse.order.iter().filter(|&&x| coarse.root(x).reach.is_some()).count();
        let fine = if reached >= n { coarse } else {
            // the stretch walked, each face's part widened a coarse ring, sampled about `rows` times
            // across the reach (never more coarsely than the coarse pass)
            let along = |a: (usize,usize,usize),b: (usize,usize,usize)| distance(coarse.root(a).position,coarse.root(b).position);
            let reach_length: f64 = coarse.order.windows(2).filter(|w| coarse.root(w[0]).reach.is_some() || coarse.root(w[1]).reach.is_some())
                .map(|w| along(w[0],w[1])).sum();
            let step = (reach_length/n as f64).max(1e-12);
            let mut windows: Vec<Option<[f64;2]>> = vec![None;patches.len()];
            let mut lengths = vec![0.;patches.len()];
            for (w,&(k,i,_)) in coarse.order.iter().enumerate() {
                let u = coarse.stations[k][i];
                let window = windows[k].get_or_insert([u,u]);
                window[0] = window[0].min(u); window[1] = window[1].max(u);
                if w > 0 && coarse.order[w-1].0 == k { lengths[k] += along(coarse.order[w-1],coarse.order[w]); }
            }
            let stations = windows.iter().zip(&lengths).map(|(window,&length)| match *window {
                None => Vec::new(),
                Some([lo,hi]) => {
                    let widen = 1./n as f64;
                    let (lo,hi) = (if lo == 0. { 0. } else { (lo-widen).max(0.) },if hi == 1. { 1. } else { (hi+widen).min(1.) });
                    let count = ((length/step).ceil() as usize).max(((hi-lo)*n as f64).round() as usize).max(2);
                    (0..=count).map(|i| lo+(hi-lo)*i as f64/count as f64).collect()
                }
            }).collect();
            let fine = walker.pass(stations)?;
            samples += fine.samples();
            fine
        };
        let Pass {stations,rings,order} = fine;
        let root = |x| root(&rings,x);
        let seed = *order.iter().find(|&&x| root(x).reach.is_some()).expect("a pass reaches");
        // the nodes, a crossing's second point dropped (its station starts the next step)
        let mut nodes: Vec<Node> = Vec::new();
        let mut from = None;
        for (w,&x) in order.iter().enumerate() {
            let (k,i,_) = x;
            let u = stations[k][i];
            let r = root(x);
            if w > 0 && order[w-1].0 != k && from.is_none() && nodes.last().is_some_and(|m: &Node| distance(m.position,r.position) <= 1e-7*scale) {
                from = Some(u); continue
            }
            let start = from.take().unwrap_or_else(|| nodes.last().map_or(u,|m| m.u));
            nodes.push(Node {patch:k,u,from:start,position:r.position,normal:r.normal,reach:r.reach});
        }
        let first = nodes.iter().position(|m| m.reach.is_some()).expect("the seed reaches");
        let last = nodes.iter().rposition(|m| m.reach.is_some()).expect("the seed reaches");
        // S1: no ring within the reach and its margin is about to lose its two roots
        for x in &order {
            let r = root(*x);
            if r.spread < LEAST_SPREAD {
                return Err((Condition::Regular,format!("the characteristic on `{}` turns back on a ring (its two points {:.1e} \
                    apart in angle)",patches[x.0].name,r.spread),witness(family,r.reach,r.position).or_else(|| witness(family,root(seed).reach,root(seed).position))))
            }
        }
        // S3: the characteristic is nowhere along the screw's velocity, where its sweep would fold.
        // Signed by the face's normal, as E3 is, so a fold passed between two samples is seen by the
        // sign it turns: the factor's size alone would pass through zero unseen.
        let mut least = f64::INFINITY;
        let mut sign = 0.;
        for w in first.max(1)..=last.min(nodes.len()-2) {
            let tangent = sub(nodes[w+1].position,nodes[w-1].position);
            let velocity = screw.velocity(nodes[w].position);
            let factor = dot(cross(tangent,velocity),nodes[w].normal)/(norm(tangent)*norm(velocity));
            if !(factor.abs() >= ask.least_factor) || factor*sign < 0. {
                let at = witness(family,nodes[w].reach.or(nodes[w-1].reach),nodes[w].position);
                return Err((Condition::Edgewise,format!("the characteristic on `{}` runs along the screw's path, where its sweep folds: \
                    the sweep's area factor is {factor:.3e}{}",patches[nodes[w].patch].name,if factor*sign < 0. { " and has turned its sign" } else { "" }),at))
            }
            sign = factor.signum();
            least = least.min(factor.abs());
        }
        let found = Characteristic {screw,nodes,reach:[first,last],least_factor:least,samples,patches:patches.to_vec(),tool:tool.clone(),motion,tolerance};
        // S4: its section square to the axis is simple
        found.simple(family)?;
        Ok(found)
    }

    /// The point of the characteristic at `s` (0 to `nodes.len() - 1`) on the tool, and its normal.
    pub fn at(&self,s: f64) -> Result<(V,V),String> {
        let last = self.nodes.len()-1;
        let s = s.clamp(0.,last as f64);
        let j = (s.ceil() as usize).max(1).min(last);
        let f = s-(j-1) as f64;
        let (a,b) = (&self.nodes[j-1],&self.nodes[j]);
        if f <= 0. { return Ok((a.position,a.normal)) }
        if f >= 1. { return Ok((b.position,b.normal)) }
        let u = b.from+(b.u-b.from)*f;
        // the root nearest the chord between the two nodes: labels may change where a ring's
        // amplitude turns, positions do not jump
        let guess = crate::space::lerp(a.position,b.position,f);
        let Ring::Roots(roots) = ring(&self.patches[b.patch],u,self.motion,self.tolerance,&|p,n| outward(&self.tool,p,n),&|_| None) else {
            return Err(format!("the characteristic on `{}` has no regular ring at {u:.6}",self.patches[b.patch].name))
        };
        roots.iter().min_by(|x,y| distance(x.position,guess).total_cmp(&distance(y.position,guess)))
            .map(|r| (r.position,r.normal)).ok_or_else(|| format!("the characteristic on `{}` has no point at {u:.6}",self.patches[b.patch].name))
    }

    /// The point at `s` carried along its path to `height`, and its normal turned with it: the
    /// sheet's section square to the axis there, which is as smooth as the sheet is. (The
    /// characteristic itself is not: where the tool's profile changes curvature its direction
    /// jumps, by a step along the screw's path, which the sweep does not see.)
    pub fn on_section(&self,s: f64,height: f64) -> Result<(V,V),String> {
        let (p,n) = self.at(s)?;
        let t = self.screw.time_to(p,height);
        Ok((self.screw.carry(p,t),self.screw.turn(n,t)))
    }

    /// S4: the reach's section square to the axis does not cross itself. The sheet is the same at
    /// every height but turned and raised, so two of its points meet exactly where two points of
    /// one section do.
    fn simple(&self,family: &Family) -> Result<(),Failure> {
        let [first,last] = self.reach;
        let height = self.screw.height(self.nodes[first].position);
        let points: Vec<V> = (first..=last).map(|w| self.on_section(w as f64,height).map(|(p,_)| p)).collect::<Result<_,_>>()
            .map_err(|m| (Condition::Motion,m,None))?;
        let crate::brep::geom::Frame {x:e1,y:e2,..} = crate::brep::geom::Frame::about([0.;3],self.screw.axis);
        let flat: Vec<[f64;2]> = points.iter().map(|p| [dot(*p,e1),dot(*p,e2)]).collect();
        if let Some((i,j)) = crossing(&flat) {
            let p: V = std::array::from_fn(|k| 0.5*(points[i][k]+points[i+1][k]));
            let at = witness(family,self.nodes[i+first].reach,self.nodes[i+first].position);
            return Err((Condition::Lead,format!("the sweep crosses itself within a lead: its section square to the axis meets \
                itself between the characteristic's points {i} and {j}"),at.or(Some(p))))
        }
        Ok(())
    }
}

/// The first pair of segments of a polyline that meet without being neighbours, by the index of
/// each one's first point: crossing, touching at a point, or overlapping along a line.
pub fn crossing(points: &[[f64;2]]) -> Option<(usize,usize)> {
    let side = |a: [f64;2],b: [f64;2],c: [f64;2]| (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
    // whether c, on the line through a and b, lies within the segment's box
    let within = |a: [f64;2],b: [f64;2],c: [f64;2]| c[0] >= a[0].min(b[0]) && c[0] <= a[0].max(b[0]) && c[1] >= a[1].min(b[1]) && c[1] <= a[1].max(b[1]);
    let meet = |a: [f64;2],b: [f64;2],c: [f64;2],d: [f64;2]| {
        let (d1,d2,d3,d4) = (side(a,b,c),side(a,b,d),side(c,d,a),side(c,d,b));
        if d1*d2 < 0. && d3*d4 < 0. { return true }
        (d1 == 0. && within(a,b,c)) || (d2 == 0. && within(a,b,d)) || (d3 == 0. && within(c,d,a)) || (d4 == 0. && within(c,d,b))
    };
    let n = points.len();
    for i in 0..n.saturating_sub(1) {
        for j in i+2..n.saturating_sub(1) {
            if meet(points[i],points[i+1],points[j],points[j+1]) { return Some((i,j)) }
        }
    }
    None
}

/// The roots of one ring of `surface` at station `u` under the twist `motion` gives, each with
/// the time its path first enters the blank (`reach`; none for a point not on the tool).
fn ring(surface: &RevolvedSurface,u: f64,motion: Motion,tolerance: f64,outward: &dyn Fn(V,V) -> V,reach: &dyn Fn(V) -> Option<f64>) -> Ring {
    let (a,b,c) = match surface.contact_coefficients(u,motion) { Ok(k) => k,Err(e) => return Ring::Degenerate(format!("{e:?}")) };
    let amplitude = a.dhypot(b);
    let spread = (1.-(c/amplitude).dpowi(2)).max(0.).sqrt();
    match surface.contacts(u,motion,tolerance) {
        Ok(found) => Ring::Roots(found.into_iter().filter_map(|r| {
            let s = surface.at(u,r.v).ok()?;
            let normal = outward(s.position,envelope::contact(s,Motion::identity()).ok()?.normal);
            Some(Root {position:s.position,normal,spread,reach:reach(s.position)})
        }).collect()),
        Err(envelope::Error::Degenerate) => Ring::Degenerate(if amplitude <= tolerance && c.abs() <= tolerance {
            "is in contact all round a ring".into() } else { "has a double point on a ring".into() }),
        Err(e) => Ring::Degenerate(format!("{e:?}")),
    }
}

/// `n` turned, if need be, to point out of the tool's material at `p`: the faces' parameters
/// orient their normals each its own way, and the class's signs read the tool's outside.
fn outward(tool: &super::SpatialField,p: V,n: V) -> V {
    let h = 1e-6*(1.+norm(p));
    let step = |k: f64| std::array::from_fn(|i| p[i]+k*h*n[i]);
    if tool.value(step(1.)) < tool.value(step(-1.)) { n.map(|x| -x) } else { n }
}

/// The first time within the roll at which the screw carries `p` into the blank: its path read a
/// degree of turn apart over the stretch of the roll that passes the blank's heights.
fn reach(family: &Family,screw: &Screw,p: V,ask: &Ask) -> Option<f64> {
    let off = sub(sub(p,screw.origin),crate::space::scale(screw.axis,screw.height(p)));
    if ask.radius.is_some_and(|r| norm(off) > r) { return None }
    let [lo,hi] = match ask.heights {
        Some([a,b]) => {
            let (x,y) = (screw.time_to(p,a),screw.time_to(p,b));
            [x.min(y).max(ask.roll[0]),x.max(y).min(ask.roll[1])]
        }
        None => ask.roll,
    };
    if !(lo <= hi) { return None }
    let step = std::f64::consts::TAU*PATH_STEP/screw.ratio.abs();
    let count = ((hi-lo)/step).ceil().max(1.) as usize;
    (0..=count).map(|k| lo+(hi-lo)*k as f64/count as f64)
        .find(|&t| family.pose_at(t).is_ok_and(|pose| (ask.inside)(pose.point(p))))
}

/// The root of `rings` the walk stands on (face, ring, root).
fn root(rings: &[Vec<Ring>],(k,i,j): (usize,usize,usize)) -> &Root {
    let Ring::Roots(r) = &rings[k][i] else { unreachable!("the walk stands on a root") };
    &r[j]
}

/// A tool point carried to the time `reach` its path first enters the blank, if it does.
fn witness(family: &Family,reach: Option<f64>,p: V) -> Option<V> {
    reach.and_then(|t| family.pose_at(t).ok()).map(|pose| pose.point(p))
}

/// One step of the walk: the next ring's root and the direction to go on in, or why there is none.
enum Step { Next((usize,usize,usize),isize),End(String) }

/// The root after `(face, ring, root)` going `dir` along the face's stations, or across the edge
/// the face shares with the next one in its profile, onto the root at the same point there.
fn step(contacts: &SweepContacts,stations: &[Vec<f64>],rings: &[Vec<Ring>],(k,i,j): (usize,usize,usize),dir: isize,scale: f64) -> Step {
    let here = *root(rings,(k,i,j));
    let n = rings[k].len()-1;
    let next = i as isize+dir;
    if (0..=n as isize).contains(&next) {
        let next = next as usize;
        return match &rings[k][next] {
            Ring::Degenerate(why) => Step::End(why.clone()),
            Ring::Roots(roots) if roots.is_empty() => Step::End("leaves its face's rings".into()),
            // the nearest root of the next ring: a label may turn where the amplitude does
            Ring::Roots(roots) => {
                let m = (0..roots.len()).min_by(|&x,&y| distance(roots[x].position,here.position)
                    .total_cmp(&distance(roots[y].position,here.position))).expect("a root");
                Step::Next((k,next,m),dir)
            }
        }
    }
    // across the edge this end of the face shares with its neighbour in the profile, where the
    // stations reach the face's end
    let end = if dir > 0 { 1. } else { 0. };
    if stations[k][i] != end { return Step::End("runs past the stretch the coarse pass sampled".into()) }
    let Some((other,their)) = contacts.edges().iter().find_map(|e| (0..2).find(|&s| e.patches[s] == k && e.ends[s] == end)
        .map(|s| (e.patches[1-s],e.ends[1-s]))) else { return Step::End("reaches the end of the tool's profile".into()) };
    let at = if their == 0. { 0 } else { stations[other].len().wrapping_sub(1) };
    if stations[other].get(at) != Some(&their) { return Step::End("crosses onto a face the coarse pass did not reach".into()) }
    let Ring::Roots(roots) = &rings[other][at] else { return Step::End("meets a degenerate ring where two faces join".into()) };
    match roots.iter().position(|r| distance(r.position,here.position) <= 1e-7*scale) {
        Some(x) => Step::Next((other,at,x),if their == 0. { 1 } else { -1 }),
        None => Step::End("jumps at a corner of the profile (a convex corner's sweep is not built for a screw)".into()),
    }
}
