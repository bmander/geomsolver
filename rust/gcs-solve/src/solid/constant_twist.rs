//! Sweeps of constant twist (docs/generating-sweeps.md): a tool carried by a screw, or by either
//! of its limits, a turn about one axis and a slide along one line (#684). Each moves every point
//! of the tool the same way, relative to the tool, at every time. So whether a tool point is on the
//! swept boundary does not depend on the time. The points that are form the tool's
//! **characteristic**, a curve fixed on the tool, and the boundary is that curve carried by the
//! motion, S(s, t) = M(t)·c(s), whose normal is the tool's there. On a revolved face, the
//! characteristic's ring at meridian station u is A cos θ + B sin θ + C = 0
//! (`RevolvedSurface::contact_coefficients`): two roots, or none. On a prism's side, the
//! characteristic's point on each ruling is the one root of a contact linear along it (#684 M,
//! `Ruled`), or under a slide the whole ruling at each point of its silhouette (`Line`). Nothing
//! is traced. The sheet is read along its **section**: square to the axis for a screw and a slide,
//! which carry one such plane to the next, and a half-plane through the axis for a turn, which
//! carries one round to the next (`Screw::station`); a turn's sheet is its section revolved, a
//! slide's its section extruded.
//!
//! `Characteristic::walk` finds the stretch of it that reaches the blank by walking one root
//! from ring to ring and across the faces of the tool's profile, and round a convex corner's fan
//! where the faces' roots part at it (`Piece::Corner`, #684 E): the corner's circle, its stations
//! the normals between its faces' (a prism's corner, its ruling). It asks the class's rows of
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
/// How far apart, in turns of the screw, a point's path through the blank is read: a slide's, in
/// the same number of steps across the stretch of the roll that passes the blank's heights.
const PATH_STEP: f64 = 1./360.;
/// How far apart, in radians, two faces' normals turn where they meet for the join to be a corner,
/// with a fan between them: below it the profile is smooth there (a circle's own ends, a tangent
/// drawn to within the solve's tolerance).
const LEAST_TURN: f64 = 1e-6;

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
    /// Whether the motion is a turn carried round a whole turn or more, read as one whole turn: its
    /// sheet is its section revolved all the way round, closing on itself (#684 N).
    pub whole: bool,
    pieces: Vec<Piece>,
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
struct Walker<'a> { contacts: &'a SweepContacts,pieces: Vec<Piece>,ask: &'a Ask<'a>,screw: Screw,motion: Motion,
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
        let pieces = &self.pieces;
        let on_tool = |p: V| self.tool.value(p).abs() < 1e-6;
        let outward = |p: V,n: V| outward(self.tool,p,n);
        // where each ring's roots are, and whether each one's path enters the blank: on every core
        let rings: Vec<Vec<Ring>> = crate::par::indices(pieces.len(),|k| stations[k].iter().map(|&u|
            ring(&pieces[k],u,self.motion,self.tolerance,&outward,&|p| on_tool(p).then(|| reach(family,screw,p,ask)).flatten())).collect());
        // S2, ring by ring: two roots of one ring in the blank are two sheets over one station
        for (k,face) in rings.iter().enumerate() { for ring in face {
            let Ring::Roots(roots) = ring else { continue };
            let reaching: Vec<&Root> = roots.iter().filter(|r| r.reach.is_some()).collect();
            if reaching.len() > 1 {
                return Err((Condition::Once,format!("both of a ring's characteristic points on {} reach the blank",pieces[k].named()),
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
                match step(contacts,pieces,&stations,&rings,at,dir,self.scale) {
                    Step::Next(next,d) => {
                        if next == seed { return Err((Condition::Regular,format!("the characteristic closes on itself through \
                            {} within the blank's reach",pieces[seed.0].named()),witness(family,root(seed).reach,root(seed).position))) }
                        let duplicate = next.0 != at.0;
                        out.push(next);
                        at = next; dir = d;
                        if duplicate { continue }
                        if root(at).reach.is_some() { since = 0 } else { since += 1 }
                        if since == MARGIN { return Ok(out) }
                    }
                    Step::End(why) => {
                        let last = out.iter().rev().chain(std::iter::once(&seed)).find(|&&x| root(x).reach.is_some()).copied().unwrap_or(seed);
                        return Err((Condition::Regular,format!("the characteristic on {} {why} within {MARGIN} rings of the blank",
                            pieces[at.0].named()),witness(family,root(last).reach,root(last).position)))
                    }
                }
            }
        };
        let (back,ahead) = (walk(-1)?,walk(1)?);
        let order: Vec<(usize,usize,usize)> = back.into_iter().rev().chain(std::iter::once(seed)).chain(ahead).collect();
        // S2: every ring point that reaches the blank is on this one stretch
        if let Some(&other) = reaching.iter().find(|x| !order.contains(x)) {
            return Err((Condition::Once,format!("the characteristic reaches the blank in more than one stretch (again on {})",
                pieces[other.0].named()),witness(family,root(other).reach,root(other).position)))
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
        let screw = family.twist().ok_or((Condition::Motion,"the motion is no screw, turn or slide".to_string(),None))?;
        let motion = family.at(ask.roll[0]).map_err(|m| (Condition::Motion,m,None))?;
        let tool = contacts.source_material();
        let scale = contacts.patches().iter().filter_map(|s| s.at(0.,0.).ok()).map(|s| norm(s.position)).fold(1_f64,f64::max);
        let tolerance = ask.root_tolerance*scale;
        // how fast the tool's points move, what a contact equation's size is read against
        let speed = contacts.patches().iter().filter_map(|s| s.at(0.5,0.5).ok()).map(|s| norm(motion.velocity(s.position)))
            .fold(0_f64,f64::max).max(f64::MIN_POSITIVE);
        let on_tool = |p: V| tool.value(p).abs() < 1e-6;
        let reaching = |p: V| on_tool(p).then(|| reach(family,&screw,p,ask)).flatten();
        // a revolution's faces, then the convex corners where two of them meet: a corner's fan is
        // walked where the characteristic jumps across it; then a prism's sides and corners, read
        // along their rulings (#684 M)
        let mut pieces: Vec<Piece> = contacts.patches().iter().enumerate()
            .filter_map(|(k,s)| s.revolved().map(|s| Piece::Face(k,s.clone()))).collect();
        pieces.extend(corners(contacts,tool,motion).into_iter().map(Piece::Corner));
        pieces.extend(prism(contacts,tool,motion,speed,&reaching)?);
        let walker = Walker {contacts,pieces,ask,screw,motion,tool,scale,tolerance};
        let pieces = &walker.pieces;
        let n = ask.rows;
        // S1 of a prism: no ruling in contact all along it where its path enters the blank
        rulings(pieces,n,motion,speed,&reaching,family)?;
        let coarse = walker.pass((0..pieces.len()).map(|_| (0..=n).map(|i| i as f64/n as f64).collect()).collect())?;
        let mut samples = coarse.samples();
        let reached = coarse.order.iter().filter(|&&x| coarse.root(x).reach.is_some()).count();
        let fine = if reached >= n { coarse } else {
            // the stretch walked, each face's part widened a coarse ring, sampled about `rows` times
            // across the reach (never more coarsely than the coarse pass)
            let along = |a: (usize,usize,usize),b: (usize,usize,usize)| distance(coarse.root(a).position,coarse.root(b).position);
            let reach_length: f64 = coarse.order.windows(2).filter(|w| coarse.root(w[0]).reach.is_some() || coarse.root(w[1]).reach.is_some())
                .map(|w| along(w[0],w[1])).sum();
            let step = (reach_length/n as f64).max(1e-12);
            let mut windows: Vec<Option<[f64;2]>> = vec![None;pieces.len()];
            let mut lengths = vec![0.;pieces.len()];
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
        // S1: no ring the walk passes is in contact all round between its stations
        stationary(pieces,&stations,&order,motion,family,&screw,ask)?;
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
                return Err((Condition::Regular,format!("the characteristic on {} turns back on a ring (its two points {:.1e} \
                    apart in angle)",pieces[x.0].named(),r.spread),witness(family,r.reach,r.position).or_else(|| witness(family,root(seed).reach,root(seed).position))))
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
                return Err((Condition::Edgewise,format!("the characteristic on {} runs along the screw's path, where its sweep folds: \
                    the sweep's area factor is {factor:.3e}{}",pieces[nodes[w].patch].named(),if factor*sign < 0. { " and has turned its sign" } else { "" }),at))
            }
            sign = factor.signum();
            least = least.min(factor.abs());
        }
        let found = Characteristic {screw,nodes,reach:[first,last],least_factor:least,samples,whole:contacts.whole(),pieces:walker.pieces,
            tool:tool.clone(),motion,tolerance};
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
        let Ring::Roots(roots) = ring(&self.pieces[b.patch],u,self.motion,self.tolerance,&|p,n| outward(&self.tool,p,n),&|_| None) else {
            return Err(format!("the characteristic on {} has no regular ring at {u:.6}",self.pieces[b.patch].named()))
        };
        roots.iter().min_by(|x,y| distance(x.position,guess).total_cmp(&distance(y.position,guess)))
            .map(|r| (r.position,r.normal)).ok_or_else(|| format!("the characteristic on {} has no point at {u:.6}",self.pieces[b.patch].named()))
    }

    /// The convex corners whose fans the characteristic runs round, each as a message names it.
    pub fn fans(&self) -> Vec<String> {
        let mut walked: Vec<usize> = self.nodes.iter().map(|m| m.patch).filter(|&k| self.pieces[k].fan().is_some()).collect();
        walked.dedup();
        walked.into_iter().map(|k| self.pieces[k].named()).collect()
    }

    /// Whether the stretch from node `w − 1` to node `w` runs round a convex corner's fan (it lies
    /// on the later node's piece).
    pub fn on_fan(&self,w: usize) -> bool { self.pieces[self.nodes[w].patch].fan().is_some() }

    /// Where along the characteristic (`s`, as `at` reads it) it runs onto or off a convex
    /// corner's fan, in order: a fan is the stretch between a face's last node and its own last.
    pub fn fan_ends(&self) -> Vec<f64> {
        (1..self.nodes.len()).filter(|&w| self.on_fan(w-1) != self.on_fan(w)).map(|w| (w-1) as f64).collect()
    }

    /// Which motion it is, as a report says it: a screw, a slide, a turn or a whole turn.
    pub fn kind(&self) -> &'static str { if self.whole { "a whole turn" } else { self.screw.kind() } }

    /// How each point's path through the blank was read, as a report says it.
    pub fn path_reading(&self) -> &'static str {
        if self.screw.is_slide() { "read in 360 steps" } else { "a degree of turn apart" }
    }

    /// The point at `s` carried along its path to `station` (`Screw::station`), and its normal
    /// turned with it: the sheet's section there, which is as smooth as the sheet is. (The
    /// characteristic itself is not: where the tool's profile changes curvature its direction
    /// jumps, by a step along the motion's path, which the sweep does not see.)
    pub fn on_section(&self,s: f64,station: f64) -> Result<(V,V),String> {
        let (p,n) = self.at(s)?;
        let t = self.screw.time_to_station(p,station,0.);
        Ok((self.screw.carry(p,t),self.screw.turn(n,t)))
    }

    /// S4: the reach's section does not cross itself. The sheet is the same at every station but
    /// turned and raised (a screw's), raised (a slide's) or turned (a turn's), so two of its points
    /// meet exactly where two points of one section do. A turn's section is read in its half-plane,
    /// by radius and height.
    fn simple(&self,family: &Family) -> Result<(),Failure> {
        let [first,last] = self.reach;
        let station = self.screw.station(self.nodes[first].position);
        let points: Vec<V> = (first..=last).map(|w| self.on_section(w as f64,station).map(|(p,_)| p)).collect::<Result<_,_>>()
            .map_err(|m| (Condition::Motion,m,None))?;
        let crate::brep::geom::Frame {x:e1,y:e2,..} = crate::brep::geom::Frame::about([0.;3],self.screw.axis);
        let flat: Vec<[f64;2]> = points.iter().map(|&p| if self.screw.is_turn() { [self.screw.radius(p),self.screw.height(p)] }
            else { [dot(p,e1),dot(p,e2)] }).collect();
        if let Some((i,j)) = crossing(&flat) {
            let p: V = std::array::from_fn(|k| 0.5*(points[i][k]+points[i+1][k]));
            let at = witness(family,self.nodes[i+first].reach,self.nodes[i+first].position);
            let within = if self.screw.is_turn() { "where its section in a half-plane through the axis" }
                else if self.screw.is_slide() { "where its section square to the slide" } else { "within a lead: its section square to the axis" };
            return Err((Condition::Lead,format!("the sweep crosses itself {within} meets itself between the characteristic's \
                points {i} and {j}"),at.or(Some(p))))
        }
        Ok(())
    }
}

/// S1 between the walk's stations: a ring in contact all round, every point of it on the
/// boundary, is where a characteristic branches. Under a turn about an axis that the tool's own
/// axis meets, the ring whose normals all pass through the point where they meet is one; under a
/// slide along the tool's axis, a ring whose normals are all square to it. Either way the
/// contact equation's amplitude turns right round as the walk passes it while its constant term
/// stays nought, and the roots it walks need not move, so a station need not fall on it: where
/// the amplitude turns more than a quarter turn between two of the walk's stations on a face, the
/// least of the ring's three coefficients between them is sought (`roots::brent`), and a ring
/// found in contact all round with a point whose path enters the blank is refused.
fn stationary(pieces: &[Piece],stations: &[Vec<f64>],order: &[(usize,usize,usize)],motion: Motion,family: &Family,screw: &Screw,
    ask: &Ask) -> Result<(),Failure> {
    let coefficients = |k: usize,u: f64| pieces[k].coefficients(u,motion).ok();
    for w in order.windows(2) {
        let ((k,i,_),(l,j,_)) = (w[0],w[1]);
        if k != l || i == j { continue }
        let (u0,u1) = (stations[k][i],stations[k][j]);
        let (Some((a0,b0,c0)),Some((a1,b1,c1))) = (coefficients(k,u0),coefficients(k,u1)) else { continue };
        if a0*a1+b0*b1 >= 0. { continue }
        let size = |(a,b,c): (f64,f64,f64)| a.dhypot(b).dhypot(c);
        let scale = size((a0,b0,c0)).max(size((a1,b1,c1)));
        let at = |u: f64| coefficients(k,u).map_or(f64::INFINITY,size);
        let (least,u) = crate::roots::brent(&at,u0.min(u1),u0.max(u1),1e-12*(u1-u0).abs(),80,|_,_| false);
        if !(least <= 1e-6*scale) { continue }
        let entering = (0..64).filter_map(|m| pieces[k].around(u,m as f64/64.))
            .find_map(|p| reach(family,screw,p,ask).map(|t| (t,p)));
        if let Some((t,p)) = entering {
            let why = if matches!(pieces[k],Piece::Ruled(_)) { format!("a ruling of {} is in contact all along it, where the \
                characteristic runs along the ruling, which the class does not sweep",pieces[k].named()) } else {
                format!("a ring of {} is in contact all round, where the characteristic branches: the tool's normals there all pass \
                through the motion's axis, or all lie square to its slide",pieces[k].named()) };
            return Err((Condition::Regular,why,witness(family,Some(t),p)))
        }
    }
    Ok(())
}

/// Where `screw` is a turn the `roll` carries round a whole turn or more, the roll once round from
/// its start (#684 N): a sweep is its path's, so that is all it sweeps.
pub(super) fn once_round(screw: &Screw,roll: [f64;2]) -> Option<[f64;2]> {
    (screw.is_turn() && roll[1]-roll[0] >= screw.period()*(1.-1e-12)).then(|| [roll[0],roll[0]+screw.period()])
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

/// The roots of one ring of `piece` at station `u` under the twist `motion` gives, each with
/// the time its path first enters the blank (`reach`; none for a point not on the tool).
fn ring(piece: &Piece,u: f64,motion: Motion,tolerance: f64,outward: &dyn Fn(V,V) -> V,reach: &dyn Fn(V) -> Option<f64>) -> Ring {
    // a prism's ruling has one root, where its contact, linear along it, is nought (none on the
    // tool where that lies past a cap); a silhouette ruling is all root (#684 M)
    let root = |position: V,normal: V| Root {position,normal,spread:1.,reach:reach(position)};
    match piece {
        Piece::Ruled(ruled) => return match ruled.root(u,motion) {
            Ok(found) => Ring::Roots(found.map(|(p,n)| root(p,n)).into_iter().collect()),
            Err(why) => Ring::Degenerate(why),
        },
        Piece::Line(line) => return Ring::Roots(vec![root(crate::space::lerp(line.ends[0],line.ends[1],u),line.normal)]),
        Piece::Face(..) | Piece::Corner(_) => {}
    }
    let (a,b,c) = match piece.coefficients(u,motion) { Ok(k) => k,Err(e) => return Ring::Degenerate(format!("{e:?}")) };
    let amplitude = a.dhypot(b);
    let spread = (1.-(c/amplitude).dpowi(2)).max(0.).sqrt();
    // each root's point and outward normal
    let found: Result<Vec<(V,V)>,envelope::Error> = match piece {
        Piece::Ruled(_) | Piece::Line(_) => unreachable!("read above"),
        Piece::Face(_,surface) => surface.contacts(u,motion,tolerance).map(|found| found.into_iter().filter_map(|r| {
            let s = surface.at(u,r.v).ok()?;
            Some((s.position,outward(s.position,envelope::contact(s,Motion::identity()).ok()?.normal)))
        }).collect()),
        Piece::Corner(corner) => corner.ring.roots((a,b,c),tolerance).map(|angles| angles.into_iter().map(|t| corner.at(u,t)).collect()),
    };
    match found {
        Ok(found) => Ring::Roots(found.into_iter().map(|(position,normal)| Root {spread,..root(position,normal)}).collect()),
        Err(envelope::Error::Degenerate) => Ring::Degenerate(if amplitude <= tolerance && c.abs() <= tolerance {
            "is in contact all round a ring".into() } else { "has a double point on a ring".into() }),
        Err(e) => Ring::Degenerate(format!("{e:?}")),
    }
}

/// `n` turned, if need be, to point out of the tool's material at `p`: the faces' parameters
/// orient their normals each its own way, and the class's signs read the tool's outside.
pub(super) fn outward(tool: &super::SpatialField,p: V,n: V) -> V {
    let h = 1e-6*(1.+norm(p));
    let step = |k: f64| std::array::from_fn(|i| p[i]+k*h*n[i]);
    if tool.value(step(1.)) < tool.value(step(-1.)) { n.map(|x| -x) } else { n }
}

/// The first time within the roll at which the motion carries `p` into the blank: its path read a
/// degree of turn apart (a slide's, `1/PATH_STEP` steps along) over the stretch of the roll that
/// passes the blank's heights (`Screw::passing`). A turn keeps a point at its height and its
/// radius, so one that stands beyond the blank in either never enters it.
fn reach(family: &Family,screw: &Screw,p: V,ask: &Ask) -> Option<f64> {
    if ask.radius.is_some_and(|r| screw.radius(p) > r) { return None }
    let [lo,hi] = match ask.heights {
        Some(band) => { let [x,y] = screw.passing(p,band)?; [x.max(ask.roll[0]),y.min(ask.roll[1])] }
        None => ask.roll,
    };
    if !(lo <= hi) { return None }
    let count = if screw.is_slide() { (1./PATH_STEP).round() as usize }
        else { ((hi-lo)*screw.ratio.abs()/(std::f64::consts::TAU*PATH_STEP)).ceil().max(1.) as usize };
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

/// The root after `(piece, ring, root)` going `dir` along the piece's stations, or across the edge
/// a face shares with the next one in its profile, onto the root at the same point there — or,
/// where the characteristic jumps at a convex corner, onto the end of the corner's fan; and off
/// a fan's end onto the face beyond.
fn step(contacts: &SweepContacts,pieces: &[Piece],stations: &[Vec<f64>],rings: &[Vec<Ring>],(k,i,j): (usize,usize,usize),dir: isize,
    scale: f64) -> Step {
    let here = *root(rings,(k,i,j));
    let n = rings[k].len()-1;
    let next = i as isize+dir;
    if (0..=n as isize).contains(&next) {
        let next = next as usize;
        return match &rings[k][next] {
            Ring::Degenerate(why) => Step::End(why.clone()),
            Ring::Roots(roots) if roots.is_empty() && matches!(pieces[k],Piece::Ruled(_)) => Step::End("runs over a cap of the \
                prism, which the class does not sweep under a screw, a turn or a slide".into()),
            Ring::Roots(roots) if roots.is_empty() => Step::End("leaves its face's rings".into()),
            // the nearest root of the next ring: a label may turn where the amplitude does
            Ring::Roots(roots) => {
                let m = (0..roots.len()).min_by(|&x,&y| distance(roots[x].position,here.position)
                    .total_cmp(&distance(roots[y].position,here.position))).expect("a root");
                Step::Next((k,next,m),dir)
            }
        }
    }
    // across the edge this end of the face shares with its neighbour in the profile (or off this
    // end of a fan onto its face), where the stations reach the piece's end
    let end = if dir > 0 { 1. } else { 0. };
    if stations[k][i] != end { return Step::End("runs past the stretch the coarse pass sampled".into()) }
    // the root of `other`'s ring at `their` end at this same point, and the way on along it; none
    // where the characteristic jumps there
    let onto = |other: usize,their: f64| -> Option<Step> {
        let at = if their == 0. { 0 } else { stations[other].len().wrapping_sub(1) };
        if stations[other].get(at) != Some(&their) { return Some(Step::End("crosses onto a face the coarse pass did not reach".into())) }
        let Ring::Roots(roots) = &rings[other][at] else { return Some(Step::End("meets a degenerate ring where two faces join".into())) };
        roots.iter().position(|r| distance(r.position,here.position) <= 1e-7*scale)
            .map(|x| Step::Next((other,at,x),if their == 0. { 1 } else { -1 }))
    };
    let jumps = || Step::End("jumps at a corner of the profile".into());
    // the piece that is a face of the tool (`SweepContacts::patches`)
    let face = |patch: usize| pieces.iter().position(|p| p.patch() == Some(patch));
    if let Piece::Line(_) = &pieces[k] {
        return Step::End("runs onto a cap of the prism, which the class does not sweep under a screw, a turn or a slide".into())
    }
    if let Some(edge) = pieces[k].fan() {
        let (e,side) = (&contacts.edges()[edge],end as usize);
        let Some(other) = face(e.patches[side]) else { return jumps() };
        return onto(other,e.ends[side]).unwrap_or_else(jumps)
    }
    let patch = pieces[k].patch().expect("a piece is a face or a fan");
    let Some((edge,e,s)) = contacts.edges().iter().enumerate().find_map(|(m,e)| (0..2).find(|&s| e.patches[s] == patch && e.ends[s] == end)
        .map(|s| (m,e,s))) else { return Step::End("reaches the end of the tool's profile".into()) };
    let fan = pieces.iter().position(|p| p.fan() == Some(edge));
    let Some(next) = face(e.patches[1-s]) else { return jumps() };
    match (onto(next,e.ends[1-s]),fan) {
        // both faces' characteristics meet the corner's circle at one point: every normal of its
        // fan is in contact there, and the sweep has a sharp edge along that point's path
        (Some(Step::Next(..)),Some(_)) => Step::End("meets a convex corner of the profile at one point from both of its faces, \
            where the sweep has a sharp edge along that point's path (not built)".into()),
        // the faces' points differ where they meet: the characteristic runs round the corner's fan
        (_,Some(fan)) => onto(fan,s as f64).unwrap_or_else(jumps),
        (Some(next),None) => next,
        (None,None) => Step::End("jumps at a concave corner of the profile, where two sheets would be trimmed against each other \
            (not built)".into()),
    }
}

/// A piece of the tool's boundary the characteristic is walked over: a revolved face (patch `k` of
/// `SweepContacts::patches`), its rings stations along its meridian; a convex corner where two
/// faces of its profile meet; a prism's side or convex corner read along its rulings (#684 M); or
/// a prism's ruling all of which is in contact, as a slide's silhouette is.
#[derive(Clone,Debug)]
enum Piece { Face(usize,RevolvedSurface),Corner(Corner),Ruled(Ruled),Line(Line) }

impl Piece {
    /// The piece as a message names it.
    fn named(&self) -> String {
        match self {
            Piece::Face(_,s) => format!("`{}`",s.name),
            Piece::Corner(Corner {names,..}) | Piece::Ruled(Ruled {ruling:Ruling::Rib {names,..},..}) =>
                format!("the corner of `{}` and `{}`",names[0],names[1]),
            Piece::Ruled(Ruled {ruling:Ruling::Side {surface,..},..}) => format!("`{}`",surface.name),
            Piece::Line(line) => line.named.clone(),
        }
    }

    /// The face of the tool it is (`SweepContacts::patches`), where it is one.
    fn patch(&self) -> Option<usize> {
        match self {
            Piece::Face(k,_) | Piece::Ruled(Ruled {ruling:Ruling::Side {patch:k,..},..}) => Some(*k),
            _ => None,
        }
    }

    /// The edge of `SweepContacts::edges` whose convex corner's fan it is, where it is one.
    fn fan(&self) -> Option<usize> {
        match self {
            Piece::Corner(Corner {edge,..}) | Piece::Ruled(Ruled {ruling:Ruling::Rib {edge,..},..}) => Some(*edge),
            _ => None,
        }
    }

    /// The coefficients of the contact equation of the piece's ring at station `u` under `motion`
    /// (a corner's, under the walk's motion its fan was read at): `a cos + b sin + c` round a
    /// ring, and along a prism's ruling, `a v + c` (`b` nought).
    fn coefficients(&self,u: f64,motion: Motion) -> Result<(f64,f64,f64),envelope::Error> {
        match self {
            Piece::Face(_,s) => s.contact_coefficients(u,motion),
            Piece::Corner(c) => {
                let [nr,nz] = c.normal(u);
                let [r,z] = c.parts;
                Ok((nr*r[0]+nz*z[0],nr*r[1]+nz*z[1],nr*r[2]+nz*z[2]))
            }
            Piece::Ruled(r) => { let [c0,c1] = r.contact(u,motion).ok_or(envelope::Error::OutsideDomain)?; Ok((c1-c0,0.,c0)) }
            Piece::Line(_) => Err(envelope::Error::Degenerate),
        }
    }

    /// The point of the piece's ring at station `u` a fraction `turn` of the way round it (along
    /// a prism's ruling, from one cap to the other).
    fn around(&self,u: f64,turn: f64) -> Option<V> {
        match self {
            Piece::Face(_,s) => { let [v0,v1] = s.domain()[1]; s.at(u,v0+(v1-v0)*turn).ok().map(|p| p.position) }
            Piece::Corner(c) => Some(c.ring.at(std::f64::consts::TAU*turn).0),
            Piece::Ruled(r) => r.line(r.parameter(u)).map(|(a,b,_)| crate::space::lerp(a,b,turn)),
            Piece::Line(line) => Some(crate::space::lerp(line.ends[0],line.ends[1],u)),
        }
    }
}

/// A prism's side or convex corner read ruling by ruling (#684 M): a side's rulings are the lines
/// along it from one cap to the other, one through each point of its edge; a corner's one ruling
/// is the line its two sides share, carrying every normal between theirs, a station on it a normal
/// of its fan as a revolution's corner's is. A ruling's points move as one affine field moves
/// them and its normal is one (a drafted side's too, #684 G), so the contact `n·v` is linear along
/// it: one root, where its values at the ruling's two ends say, or none on the tool where that
/// lies past a cap. Its stations are spread along the characteristic by length (`lengths`, read
/// at `RULED_READS` parameters, a point's root held a quarter of the ruling past either cap, and
/// each turn of its normal counted as the ruling's length over a half turn), so a root running out
/// along the rulings toward a cap, as a corner's does where its fan's normal turns toward one
/// square to the motion, is sampled as finely there as anywhere.
#[derive(Clone,Debug)]
struct Ruled { ruling: Ruling,lengths: Vec<f64> }

#[derive(Clone,Debug)]
enum Ruling {
    /// Patch `patch`, a side, its normal `du × dv` turned out of the tool where `flip` says.
    Side { patch: usize,surface: super::ExtrudedSurface,flip: bool },
    /// The convex corner edge `edge` of `SweepContacts::edges` is: its sides' names, its ruling's
    /// ends (at the first side's `v` 0 and 1) and the outward normals of its sides there.
    Rib { edge: usize,names: [String;2],ends: [V;2],normals: [V;2] },
}

/// How many parameters along a ruled piece its stations' lengths are read at.
const RULED_READS: usize = 256;

impl Ruled {
    /// A piece, its stations spread along its characteristic under `motion`.
    fn new(ruling: Ruling,motion: Motion) -> Ruled {
        let mut ruled = Ruled {ruling,lengths:Vec::new()};
        let lines: Vec<Option<(V,V,V)>> = (0..=RULED_READS).map(|i| ruled.line(i as f64/RULED_READS as f64)).collect();
        let size = lines.iter().flatten().map(|(a,b,_)| distance(*a,*b)).fold(0_f64,f64::max);
        let point = |(a,b,n): (V,V,V)| {
            let [c0,c1] = [a,b].map(|p| dot(motion.vector(n),motion.velocity(p)));
            let v = if c0 == c1 { if c0 == 0. { 0.5 } else { f64::INFINITY } } else { c0/(c0-c1) };
            (crate::space::lerp(a,b,v.clamp(-0.25,1.25)),n)
        };
        let mut lengths = vec![0.];
        for w in lines.windows(2) {
            let step = match (w[0],w[1]) {
                (Some(x),Some(y)) => { let ((p,m),(q,n)) = (point(x),point(y));
                    distance(p,q)+size/std::f64::consts::PI*dot(m,n).clamp(-1.,1.).dacos() }
                _ => 0.,
            };
            lengths.push(lengths.last().expect("a length")+step+1e-3*size/RULED_READS as f64);
        }
        let total = *lengths.last().expect("a length");
        ruled.lengths = lengths.into_iter().map(|l| l/total).collect();
        ruled
    }

    /// The parameter along the piece (its side's edge, or its fan's turn) at station `u`.
    fn parameter(&self,u: f64) -> f64 {
        if self.lengths.is_empty() || u <= 0. { return u.max(0.) }
        if u >= 1. { return 1. }
        let i = self.lengths.partition_point(|&l| l <= u).clamp(1,self.lengths.len()-1);
        let (a,b) = (self.lengths[i-1],self.lengths[i]);
        ((i-1) as f64+((u-a)/(b-a)).clamp(0.,1.))/RULED_READS as f64
    }

    /// The ruling at parameter `x` (not a station): its ends, and the outward normal along it.
    fn line(&self,x: f64) -> Option<(V,V,V)> {
        match &self.ruling {
            Ruling::Side {surface,flip,..} => {
                let (a,b) = (surface.at(x,0.).ok()?,surface.at(x,1.).ok()?);
                let n = crate::space::unit(cross(a.du,a.dv));
                Some((a.position,b.position,if *flip { n.map(|c| -c) } else { n }))
            }
            Ruling::Rib {ends,normals,..} => Some((ends[0],ends[1],between(*normals,x))),
        }
    }

    /// The contact `n·v` at the two ends of the ruling at station `u` under `motion`.
    fn contact(&self,u: f64,motion: Motion) -> Option<[f64;2]> {
        let (a,b,n) = self.line(self.parameter(u))?;
        Some([a,b].map(|p| dot(motion.vector(n),motion.velocity(p))))
    }

    /// The root of the ruling at station `u`, and its normal: none where it lies past a cap, and
    /// refused where the whole ruling is in contact (S1 says where that is in the blank).
    fn root(&self,u: f64,motion: Motion) -> Result<Option<(V,V)>,String> {
        let (a,b,n) = self.line(self.parameter(u)).ok_or("is read off its side")?;
        let [c0,c1] = [a,b].map(|p| dot(motion.vector(n),motion.velocity(p)));
        let size = c0.abs().max(c1.abs()).max(norm(motion.velocity(a))).max(norm(motion.velocity(b)));
        if c0.abs().max(c1.abs()) <= 1e-12*size { return Err("is in contact all along a ruling".into()) }
        if c0 == c1 { return Ok(None) }
        let v = c0/(c0-c1);
        Ok((-1e-12..=1.+1e-12).contains(&v).then(|| (crate::space::lerp(a,b,v.clamp(0.,1.)),n)))
    }
}

/// The unit vector a fraction `x` of the way round from `a` to `b`, in their plane.
fn between([a,b]: [V;2],x: f64) -> V {
    let angle = dot(a,b).clamp(-1.,1.).dacos();
    let s = angle.dsin();
    if s <= 1e-12 { return a }
    let (p,q) = (((1.-x)*angle).dsin()/s,(x*angle).dsin()/s);
    std::array::from_fn(|k| p*a[k]+q*b[k])
}

/// A prism's ruling every point of which is in contact (#684 M): where the motion moves each
/// ruling's points alike (a slide; a screw along the prism's own extrusion), a side or a corner's
/// fan is in contact along the whole ruling at each root of `n·v` across it, the silhouette, and
/// nowhere else. A station on it is the fraction of the way from the first cap to the second.
#[derive(Clone,Debug)]
struct Line { named: String,ends: [V;2],normal: V }

/// A convex corner of the tool's profile, turned to a circle: every normal between its two faces'
/// there is the tool's, so a point of the circle is on the boundary wherever one of them is square
/// to its velocity (`SweepContacts::at_point_normal_over` says the same for a moving point). Its
/// one ring is that circle, and a station on it `u` is a normal of the fan, turned from the first
/// face's outward normal (0) to the second's (1) in the plane through the axis: the ring's points
/// are the roots of the same `A cos θ + B sin θ + C = 0`, its coefficients the fan normal's
/// (`RingFrame::coefficients`). Where the two faces' characteristics meet the circle at different
/// points, the characteristic runs between them round the fan, and the sheet the fan makes is the
/// circle's own sweep, tangent to both faces' sheets.
#[derive(Clone,Debug)]
struct Corner {
    /// The edge of `SweepContacts::edges` it is, and its two faces' names.
    edge: usize,
    names: [String;2],
    ring: crate::solid::surface::RingFrame,
    /// The first face's outward normal's angle in the plane through the axis, from the radial
    /// direction toward the axis's, and how far the fan turns from it to the second's.
    from: f64,
    turn: f64,
    /// The ring's coefficients for a normal straight out from the axis and for one along it: a
    /// fan normal's are theirs in its two parts, the contact equation being linear in the normal.
    parts: [[f64;3];2],
}

impl Corner {
    /// The fan's normal at station `u`, away from the axis and along it.
    fn normal(&self,u: f64) -> [f64;2] {
        let (s,c) = (self.from+self.turn*u).dsin_cos();
        [c,s]
    }

    /// The point of the circle at angle `theta`, and the fan's normal at station `u` there.
    fn at(&self,u: f64,theta: f64) -> (V,V) {
        let [nr,nz] = self.normal(u);
        let (position,e) = self.ring.at(theta);
        (position,std::array::from_fn(|k| nr*e[k]+nz*self.ring.axis[k]))
    }
}

/// The convex corners of the tool's profile, where two of its faces meet at an angle with the
/// tool's material within it: their normals there more than `LEAST_TURN` apart, and the chord of
/// `SweepContacts::corner`'s probe in the material. A smooth join has no fan, and a concave
/// corner's sweep is no part of the boundary (T2 refuses one that reaches the blank); a corner on
/// the axis turns to no circle. Each corner's ring is read under `motion`, the walk's.
fn corners(contacts: &SweepContacts,tool: &super::SpatialField,motion: Motion) -> Vec<Corner> {
    let normal = |s: &RevolvedSurface,u: f64| -> Option<(V,V)> {
        let p = s.at(u,0.).ok()?;
        Some((p.position,envelope::contact(p,Motion::identity()).ok()?.normal))
    };
    contacts.edges().iter().enumerate().filter_map(|(edge,e)| {
        let ([a,b],[ua,ub]) = (e.patches,e.ends);
        let (sa,sb) = (contacts.patches().get(a)?.revolved()?,contacts.patches().get(b)?.revolved()?);
        let ring = sa.ring(ua).ok()?;
        // each face's normal at the corner, turned out of the tool as it is in the face's middle,
        // where nothing else is near: the chart orients it the same way all along the face
        let out = |s: &RevolvedSurface,u: f64| -> Option<f64> {
            let (_,n) = normal(s,u)?;
            let (p,m) = normal(s,0.5)?;
            let n = if dot(outward(tool,p,m),m) < 0. { n.map(|x| -x) } else { n };
            Some(dot(n,ring.axis).datan2(dot(n,ring.radial)))
        };
        let (from,to) = (out(sa,ua)?,out(sb,ub)?);
        let turn = (to-from+std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)-std::f64::consts::PI;
        if turn.abs() <= LEAST_TURN { return None }
        let (_,[pa,pb]) = contacts.corner(e)?;
        let chord: V = std::array::from_fn(|k| 0.5*(pa[k]+pb[k]));
        if !(tool.value(chord) < 0.) { return None }
        let part = |nr: f64,nz: f64| { let (a,b,c) = ring.coefficients(nr,nz,motion); [a,b,c] };
        Some(Corner {edge,names:[sa.name.clone(),sb.name.clone()],ring,from,turn,parts:[part(1.,0.),part(0.,1.)]})
    }).collect()
}

/// A prism's pieces (#684 M): its sides (`ToolSurface::Extruded`, a drafted prism's too) and the
/// convex corners where two of them meet, their normals more than `LEAST_TURN` apart and the chord
/// just inside them halfway between the caps (`SweepContacts::corner`) in the material (a concave
/// corner's sweep is no part of the boundary, and T2 refuses one that reaches the blank). Its caps
/// are not walked: the class asks that they stay clear of the blank (T1, `admission::ends`).
///
/// Under a turn or a screw about any line but the prism's own extrusion, each piece is read ruling
/// by ruling (`Ruled`). Where the motion moves every point of each ruling alike — a slide, or a
/// screw along the extrusion — the contact is the same all along each ruling, so the rulings in
/// contact are whole: the silhouette, a ruling at each root of `n·v` across each side and each
/// corner's fan (`Line`), read at `RULED_READS` parameters and refined between two that differ in
/// sign. A piece in contact all over there (a side along the slide, a corner moving along itself)
/// is refused where its path enters the blank (S1): every point of it is on the boundary, the
/// face kept in its own plane.
fn prism(contacts: &SweepContacts,tool: &super::SpatialField,motion: Motion,speed: f64,reaching: &dyn Fn(V) -> Option<f64>)
    -> Result<Vec<Piece>,Failure> {
    let sides: Vec<(usize,&super::ExtrudedSurface)> = contacts.patches().iter().enumerate()
        .filter_map(|(k,s)| match s { super::ToolSurface::Extruded(e) => Some((k,e)),_ => None }).collect();
    if sides.is_empty() { return Ok(Vec::new()) }
    let unruled = |ruling: &Ruling| Ruled {ruling:ruling.clone(),lengths:Vec::new()};
    // each side's normal turned out of the tool as it is in the side's middle, where nothing else
    // is near: the chart orients it the same way all along the side
    let mut rulings: Vec<Ruling> = sides.iter().map(|&(patch,surface)| {
        let flip = surface.at(0.5,0.5).is_ok_and(|p| { let n = crate::space::unit(cross(p.du,p.dv));
            dot(outward(tool,p.position,n),n) < 0. });
        Ruling::Side {patch,surface:surface.clone(),flip}
    }).collect();
    for (edge,e) in contacts.edges().iter().enumerate() {
        let ([a,b],[ua,ub]) = (e.patches,e.ends);
        let (Some(ia),Some(ib)) = (sides.iter().position(|(k,_)| *k == a),sides.iter().position(|(k,_)| *k == b)) else { continue };
        let (Some((p0,p1,na)),Some((_,_,nb))) = (unruled(&rulings[ia]).line(ua),unruled(&rulings[ib]).line(ub)) else { continue };
        if dot(na,nb).clamp(-1.,1.).dacos() <= LEAST_TURN { continue }
        let Some((_,[pa,pb])) = contacts.corner(e) else { continue };
        if !(tool.value(crate::space::lerp(pa,pb,0.5)) < 0.) { continue }
        rulings.push(Ruling::Rib {edge,names:[sides[ia].1.name.clone(),sides[ib].1.name.clone()],ends:[p0,p1],normals:[na,nb]});
    }
    // whether the motion moves every point of each side's rulings alike
    let alike = sides.iter().all(|(_,s)| [0.,0.5,1.].iter().all(|&x| match (s.at(x,0.),s.at(x,1.)) {
        (Ok(a),Ok(b)) => norm(sub(motion.velocity(a.position),motion.velocity(b.position))) <= 1e-9*speed,
        _ => false,
    }));
    if !alike { return Ok(rulings.into_iter().map(|r| Piece::Ruled(Ruled::new(r,motion))).collect()) }
    // the silhouette: each piece's rulings where `n·v`, the same all along each, is nought
    let mut lines: Vec<Line> = Vec::new();
    for ruling in &rulings {
        let piece = unruled(ruling);
        let named = Piece::Ruled(piece.clone()).named();
        let read: Vec<Option<(f64,(V,V,V))>> = (0..=RULED_READS).map(|i| { let x = i as f64/RULED_READS as f64;
            piece.line(x).map(|(a,b,n)| (dot(motion.vector(n),motion.velocity(a)),(a,b,n))) }).collect();
        if read.iter().flatten().all(|(c,_)| c.abs() <= 1e-9*speed) {
            let entering = read.iter().flatten().flat_map(|&(_,(a,b,_))| (0..=16).map(move |j| crate::space::lerp(a,b,j as f64/16.)))
                .find_map(|p| reaching(p).map(|t| (t,p)));
            if let Some((t,p)) = entering {
                return Err((Condition::Regular,format!("{named} moves along itself, every point of it in contact: the motion keeps \
                    it in its own plane, which the class does not sweep"),witness(contacts.motion(),Some(t),p)))
            }
            continue
        }
        let contact = |x: f64| piece.line(x).map(|(a,_,n)| dot(motion.vector(n),motion.velocity(a)));
        for i in 0..RULED_READS {
            let (Some((c0,_)),Some((c1,_))) = (read[i],read[i+1]) else { continue };
            let (x0,x1) = (i as f64/RULED_READS as f64,(i+1) as f64/RULED_READS as f64);
            // a nought at a read parameter is the bracket's that ends there
            let x = if c1 == 0. { x1 } else if c0 == 0. && i == 0 { x0 } else if c0*c1 < 0. {
                crate::roots::bracketed_root(contact,x0,c0,x1,c1,1e-14)
            } else { continue };
            let Some((a,b,n)) = piece.line(x) else { continue };
            let size = distance(a,b);
            if lines.iter().any(|l| distance(l.ends[0],a) <= 1e-9*size && distance(l.ends[1],b) <= 1e-9*size) { continue }
            lines.push(Line {named:named.clone(),ends:[a,b],normal:n});
        }
    }
    Ok(lines.into_iter().map(Piece::Line).collect())
}

/// S1 of a prism read ruling by ruling (#684 M): no ruling in contact all along it where its path
/// enters the blank. There the characteristic runs along the ruling, off the walked stretch, which
/// crosses each ruling once; a side every ruling of which is in contact is a face the motion keeps
/// in its own plane (a side square to a turn). A ruling's contact is nought at both its ends there,
/// so between two of `rows` stations across each piece where both ends' contacts change sign (or
/// are nought), the least of the two together is sought (`roots::brent`), and a ruling found in
/// contact all along with a point whose path enters the blank is refused.
fn rulings(pieces: &[Piece],rows: usize,motion: Motion,speed: f64,reaching: &dyn Fn(V) -> Option<f64>,family: &Family)
    -> Result<(),Failure> {
    for piece in pieces {
        let Piece::Ruled(ruled) = piece else { continue };
        let size = |u: f64| ruled.contact(u,motion).map_or(f64::INFINITY,|[c0,c1]| c0.dhypot(c1));
        let nought = |u: f64| size(u) <= 1e-6*speed;
        for w in (0..=rows).map(|i| i as f64/rows as f64).collect::<Vec<_>>().windows(2) {
            // each end's contact at either station, nought within the bar
            let read = |u: f64| ruled.contact(u,motion).map(|c| c.map(|c| if c.abs() <= 1e-6*speed { 0. } else { c }));
            let (Some([a0,a1]),Some([b0,b1])) = (read(w[0]),read(w[1])) else { continue };
            if a0*b0 > 0. || a1*b1 > 0. { continue }
            let (_,u) = crate::roots::brent(&size,w[0],w[1],1e-12,80,|_,_| false);
            let entering = [w[0],u,w[1]].into_iter().filter(|&u| nought(u))
                .flat_map(|u| (0..=64).filter_map(move |j| piece.around(u,j as f64/64.))).find_map(|p| reaching(p).map(|t| (t,p)));
            let Some((t,p)) = entering else { continue };
            let why = if [0.,0.5,1.].into_iter().all(nought) { format!("{} is in contact all over: the motion keeps it in its own \
                plane, which the class does not sweep",piece.named()) } else { format!("a ruling of {} is in contact all along it, \
                where the characteristic runs along the ruling, which the class does not sweep",piece.named()) };
            return Err((Condition::Regular,why,witness(family,Some(t),p)))
        }
    }
    Ok(())
}
