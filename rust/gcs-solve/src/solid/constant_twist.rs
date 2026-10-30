//! Sweeps of constant twist (docs/generating-sweeps.md): a tool carried by a screw, or by either
//! of its limits, a turn about one axis and a slide along one line (#684). Each moves every point
//! of the tool the same way, relative to the tool, at every time. So whether a tool point is on the
//! swept boundary does not depend on the time. The points that are form the tool's
//! **characteristic**, a curve fixed on the tool, and the boundary is that curve carried by the
//! motion, S(s, t) = M(t)·c(s), whose normal is the tool's there. On a revolved face, the
//! characteristic's ring at meridian station u is A cos θ + B sin θ + C = 0
//! (`RevolvedSurface::contact_coefficients`): two roots, or none. On a prism's side, the
//! characteristic's point on each ruling is the one root of a contact linear along it (#684 M,
//! `Ruled`), or under a slide the whole ruling at each point of its silhouette (`Line`); on a
//! prism's cap, a line, the contact being linear across its plane (#684 AE). Nothing is traced.
//! The sheet is read along its **section**: square to the axis for a screw and a slide, which
//! carry one such plane to the next, and a half-plane through the axis for a turn, which carries
//! one round to the next (`Screw::station`); a turn's sheet is its section revolved, a slide's its
//! section extruded.
//!
//! `Characteristic::walk` finds the stretch of it that reaches the blank by walking one root
//! from ring to ring and across the faces of the tool's profile, and round a convex corner's fan
//! where the faces' roots part at it (`Piece::Corner`, #684 E): the corner's circle, its stations
//! the normals between its faces' (a prism's corner, its ruling; a cap's rim or a drafted crease,
//! its line, stepped onto partway along a side, #684 AE, AG). It asks the class's rows of
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
use crate::space::{sub,dot,cross,norm,scale,distance};

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
    /// Whether the tool's caps are walked: a prism standing alone or placed, its profile one loop,
    /// whose cap may reach the blank (#684 AE, `admission::Lone`).
    pub caps: bool,
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
        let outward = |p: V,n: V| outward(self.tool,p,n);
        // where each ring's roots are, and whether each one's path enters the blank: on every core
        let rings: Vec<Vec<Ring>> = crate::par::indices(pieces.len(),|k| stations[k].iter().map(|&u|
            ring(&pieces[k],u,self.motion,self.tolerance,&outward,&|p| entering(self.tool,family,screw,ask,p))).collect());
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
        let enters = |p: V| entering(tool,family,&screw,ask,p);
        // a revolution's faces, then the convex corners where two of them meet: a corner's fan is
        // walked where the characteristic jumps across it; then a prism's sides and corners, read
        // along their rulings (#684 M)
        let mut pieces: Vec<Piece> = contacts.patches().iter().enumerate()
            .filter_map(|(k,s)| s.revolved().map(|s| Piece::Face(k,s.clone()))).collect();
        pieces.extend(corners(contacts,tool,motion).into_iter().map(Piece::Corner));
        pieces.extend(prism(contacts,tool,motion,&screw,speed,ask.caps,&enters)?);
        let walker = Walker {contacts,pieces,ask,screw,motion,tool,scale,tolerance};
        let pieces = &walker.pieces;
        let n = ask.rows;
        // S1 of a prism: no ruling in contact all along it where its path enters the blank
        rulings(pieces,n,motion,&enters,family)?;
        let coarse = walker.pass((0..pieces.len()).map(|k| crossed(&pieces[k],(0..=n).map(|i| i as f64/n as f64).collect()))
            .collect())?;
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
            }).enumerate().map(|(k,stations)| crossed(&pieces[k],stations)).collect();
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
        let mut walked: Vec<usize> = self.nodes.iter().map(|m| m.patch).filter(|&k| self.pieces[k].fan()).collect();
        walked.dedup();
        walked.into_iter().map(|k| self.pieces[k].named()).collect()
    }

    /// Whether the stretch from node `w − 1` to node `w` runs round a convex corner's fan (it lies
    /// on the later node's piece).
    pub fn on_fan(&self,w: usize) -> bool { self.pieces[self.nodes[w].patch].fan() }

    /// Where along the characteristic (`s`, as `at` reads it) it runs onto or off a convex
    /// corner's fan, in order: a fan is the stretch between a face's last node and its own last.
    pub fn fan_ends(&self) -> Vec<f64> {
        (1..self.nodes.len()).filter(|&w| self.on_fan(w-1) != self.on_fan(w)).map(|w| (w-1) as f64).collect()
    }

    /// How far the normal turns over the fan whose end `fan_ends` gives at `s`: between that end
    /// and the fan's other (the walk's own end, where it starts or ends on the fan).
    pub fn fan_turn(&self,s: f64) -> f64 {
        let ends = self.fan_ends();
        let Some(k) = ends.iter().position(|&e| e == s) else { return 0. };
        // a fan runs from an end onto it to the next end off it
        let onto = |e: f64| self.on_fan(e as usize+1);
        let other = if onto(s) { ends.get(k+1).copied().unwrap_or((self.nodes.len()-1) as f64) }
            else { k.checked_sub(1).map_or(0.,|j| ends[j]) };
        dot(self.nodes[s as usize].normal,self.nodes[other as usize].normal).clamp(-1.,1.).dacos()
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
            return Err((Condition::Regular,format!("a ring of {} is in contact all round, where the characteristic branches: \
                the tool's normals there all pass through the motion's axis, or all lie square to its slide",pieces[k].named()),
                witness(family,Some(t),p)))
        }
    }
    Ok(())
}

/// A piece's stations with those where its root crosses onto a cap's rim or a crease's fan
/// (`Ruled::crossings`) put among them, within the stretch they span: the walk steps onto the fan
/// from a station, and the root reaches it partway along the piece, not at its end (#684 AE, AG).
/// A station within `CROSSING_GAP` of a crossing gives way to it.
fn crossed(piece: &Piece,mut stations: Vec<f64>) -> Vec<f64> {
    let Piece::Ruled(ruled) = piece else { return stations };
    let (Some(&lo),Some(&hi)) = (stations.first(),stations.last()) else { return stations };
    for &u in ruled.crossings.iter().filter(|&&u| u > lo+CROSSING_GAP && u < hi-CROSSING_GAP) {
        stations.retain(|&s| (s-u).abs() > CROSSING_GAP);
        stations.push(u);
    }
    stations.sort_by(f64::total_cmp);
    stations
}

/// How near a station stands to a crossing to give way to it (`crossed`).
const CROSSING_GAP: f64 = 1e-9;

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
    let root = |position: V,normal: V,spread: f64| Root {position,normal,spread,reach:reach(position)};
    // a revolution's ring, `a cos + b sin + c`: its roots, each with its point and outward normal
    let round = |found: &dyn Fn((f64,f64,f64)) -> Result<Vec<(V,V)>,envelope::Error>| -> Ring {
        let (a,b,c) = match piece.coefficients(u,motion) { Ok(k) => k,Err(e) => return Ring::Degenerate(format!("{e:?}")) };
        let amplitude = a.dhypot(b);
        let spread = (1.-(c/amplitude).dpowi(2)).max(0.).sqrt();
        match found((a,b,c)) {
            Ok(found) => Ring::Roots(found.into_iter().map(|(position,normal)| root(position,normal,spread)).collect()),
            Err(envelope::Error::Degenerate) => Ring::Degenerate(if amplitude <= tolerance && c.abs() <= tolerance {
                "is in contact all round a ring".into() } else { "has a double point on a ring".into() }),
            Err(e) => Ring::Degenerate(format!("{e:?}")),
        }
    };
    match piece {
        Piece::Face(_,surface) => round(&|_| surface.contacts(u,motion,tolerance).map(|found| found.into_iter().filter_map(|r| {
            let s = surface.at(u,r.v).ok()?;
            Some((s.position,outward(s.position,envelope::contact(s,Motion::identity()).ok()?.normal)))
        }).collect())),
        Piece::Corner(corner) => round(&|k| corner.ring.roots(k,tolerance).map(|angles| angles.into_iter().map(|t| corner.at(u,t)).collect())),
        // a prism's ruling has one root, where its contact, linear along it, is nought (none on the
        // tool where that lies past a cap); a silhouette ruling is all root (#684 M)
        Piece::Ruled(ruled) => match ruled.root(u,motion) {
            Ok(found) => Ring::Roots(found.map(|(p,n)| root(p,n,1.)).into_iter().collect()),
            Err(why) => Ring::Degenerate(why),
        },
        Piece::Line(line) => Ring::Roots(vec![root(crate::space::lerp(line.ends[0],line.ends[1],u),line.normal,1.)]),
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

/// When the path of `p` first enters the blank (`reach`), where `p` is a point of the tool: none
/// for a point of a face's chart off the tool (past a trim, or hidden by another operand).
fn entering(tool: &super::SpatialField,family: &Family,screw: &Screw,ask: &Ask,p: V) -> Option<f64> {
    (tool.value(p).abs() < 1e-6).then(|| reach(family,screw,p,ask)).flatten()
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
/// a fan's end onto the face beyond. A prism's side whose root reaches a ruling's end at a cap's
/// rim or a crease steps onto that fan there, partway along the side, and off the fan's far end
/// onto the cap's contact line or the other half's side (#684 AE, AG).
fn step(contacts: &SweepContacts,pieces: &[Piece],stations: &[Vec<f64>],rings: &[Vec<Ring>],(k,i,j): (usize,usize,usize),dir: isize,
    scale: f64) -> Step {
    let here = *root(rings,(k,i,j));
    let at_here = |roots: &[Root]| roots.iter().position(|r| distance(r.position,here.position) <= 1e-7*scale);
    // the root of `other`'s ring at `their` end at this same point, and the way on along it; none
    // where the characteristic jumps there
    let onto = |other: usize,their: f64| -> Option<Step> {
        let at = if their == 0. { 0 } else { stations[other].len().wrapping_sub(1) };
        if stations[other].get(at) != Some(&their) { return Some(Step::End("crosses onto a face the coarse pass did not reach".into())) }
        let Ring::Roots(roots) = &rings[other][at] else { return Some(Step::End("meets a degenerate ring where two faces join".into())) };
        at_here(roots).map(|x| Step::Next((other,at,x),if their == 0. { 1 } else { -1 }))
    };
    // the first of `candidates` (a piece and its end) whose root there is this point, else the
    // first that could not be read
    let first = |candidates: &mut dyn Iterator<Item=(usize,f64)>| -> Option<Step> {
        let mut ended = None;
        for (other,their) in candidates {
            match onto(other,their) {
                Some(Step::End(why)) => { ended.get_or_insert(Step::End(why)); }
                found @ Some(_) => return found,
                None => {}
            }
        }
        ended
    };
    // the side `patch` at the station whose root is this point, going on the way its root does:
    // where the walk crosses onto it off a cap's rim or a crease, partway along it
    let side_at = |patch: usize| -> Option<Step> {
        let s = pieces.iter().position(|p| p.patch() == Some(patch))?;
        let found = rings[s].iter().enumerate().find_map(|(m,ring)| match ring { Ring::Roots(roots) => at_here(roots).map(|x| (m,x)),
            _ => None });
        let Some((m,x)) = found else {
            return stations[s].is_empty().then(|| Step::End("crosses onto a face the coarse pass did not reach".into()))
        };
        let rooted = |d: isize| (m as isize+d).try_into().ok().and_then(|q: usize| rings[s].get(q))
            .is_some_and(|ring| matches!(ring,Ring::Roots(roots) if !roots.is_empty()));
        Some(match (rooted(1),rooted(-1)) {
            (true,false) => Step::Next((s,m,x),1),
            (false,true) => Step::Next((s,m,x),-1),
            _ => Step::End(format!("meets {} at one point of it, where the class does not cross it",pieces[s].named())),
        })
    };
    // a side's root running off its rulings: over a cap, or across a crease where its halves meet
    let off = || -> String {
        if let Piece::Ruled(ruled @ Ruled {ruling:Ruling::Side {patch,..},..}) = &pieces[k] {
            let end = ruled.line(stations[k][i])
                .map_or(0,|(a,b,_)| { let d = sub(b,a); (dot(sub(here.position,a),d) > 0.5*dot(d,d)) as usize });
            if contacts.creases().iter().any(|crease| crease[1-end] == *patch) { return OVER_CREASE.into() }
        }
        pieces[k].off().into()
    };
    let n = rings[k].len()-1;
    let next = i as isize+dir;
    if (0..=n as isize).contains(&next) {
        let next = next as usize;
        return match &rings[k][next] {
            Ring::Degenerate(why) => Step::End(why.clone()),
            // a side's root reaching a ruling's end at a cap's rim or a crease: onto that fan
            Ring::Roots(roots) if roots.is_empty() => match pieces[k].patch().filter(|_| matches!(pieces[k],Piece::Ruled(_))) {
                Some(patch) => first(&mut pieces.iter().enumerate().filter_map(|(r,p)| p.rim_of(patch).map(|end| (r,end))))
                    .unwrap_or_else(|| Step::End(off())),
                None => Step::End(off()),
            },
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
    match &pieces[k] {
        // a rim's or a crease's fan: off its first end onto its side, off its last onto the cap's
        // contact line or the other half's side
        Piece::Ruled(Ruled {ruling:Ruling::Rim {side,across,..},..}) => return if end == 0. { side_at(*side) } else { match *across {
            Across::Cap(c) => first(&mut pieces.iter().enumerate().filter(|(_,p)| p.cap() == Some(c)).flat_map(|(l,_)| [(l,0.),(l,1.)])),
            Across::Half(other) => side_at(other),
        } }.unwrap_or_else(|| Step::End("runs off its fan where no root of the face beyond takes it on".into())),
        // a cap's contact line: off either end onto the fan of the rim it meets there
        Piece::Line(Line {cap:Some(c),..}) => return first(&mut pieces.iter().enumerate()
            .filter(|(_,p)| matches!(p,Piece::Ruled(Ruled {ruling:Ruling::Rim {across:Across::Cap(d),..},..}) if d == c)).map(|(r,_)| (r,1.)))
            .unwrap_or_else(|| Step::End(OFF_CAP.into())),
        _ => {}
    }
    let jumps = || Step::End("jumps at a corner of the profile".into());
    // the piece that is a face of the tool (`SweepContacts::patches`)
    let face = |patch: usize| pieces.iter().position(|p| p.patch() == Some(patch));
    if let Some(edge) = pieces[k].edge() {
        let (e,side) = (&contacts.edges()[edge],end as usize);
        let Some(other) = face(e.patches[side]) else { return jumps() };
        return onto(other,e.ends[side]).unwrap_or_else(jumps)
    }
    // a silhouette ruling's ends are the prism's caps
    let Some(patch) = pieces[k].patch() else { return Step::End(SILHOUETTE_OVER_CAP.into()) };
    let Some((edge,e,s)) = contacts.edges().iter().enumerate().find_map(|(m,e)| (0..2).find(|&s| e.patches[s] == patch && e.ends[s] == end)
        .map(|s| (m,e,s))) else { return Step::End("reaches the end of the tool's profile".into()) };
    let fan = pieces.iter().position(|p| p.edge() == Some(edge));
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
/// faces of its profile meet; a prism's side or convex corner read along its rulings (#684 M), or
/// a cap's rim or a crease read along its line (#684 AE, AG); or a line all of which is in
/// contact: a slide's silhouette ruling, or a cap's contact line (#684 AE).
#[derive(Clone,Debug)]
enum Piece { Face(usize,RevolvedSurface),Corner(Corner),Ruled(Ruled),Line(Line) }

/// Why the walk ends where a prism's characteristic runs off its side over a cap it does not walk.
const OVER_CAP: &str = "runs over a cap of the prism, which the class walks only across a rim that is a line, of a prism \
    standing alone as the tool or placed, its profile one loop";
/// Why it ends where it runs off a half of a drafted prism's side across the crease at its parting
/// plane, where the side is no line.
const OVER_CREASE: &str = "runs across the crease where the prism's halves meet at its parting plane, on a side that is no \
    line, which the class does not walk";
/// Why it ends where a silhouette ruling's end is reached: under a slide, or a screw along the
/// extrusion, a cap is in contact all over or nowhere, and its rims' silhouettes meet the sides' at
/// the prism's vertices.
const SILHOUETTE_OVER_CAP: &str = "runs over a cap of the prism, which the class does not walk under a slide, or a screw along \
    the prism's extrusion: there the silhouette turns at a vertex of the prism, a sharp edge of the sweep (not built)";
/// Why it ends where a corner's, a rim's or a crease's root runs off its line.
const VERTEX: &str = "runs into a vertex of the prism, where a corner meets a cap or a crease, along whose path the sweep has \
    a sharp edge (not built)";
/// Why it ends where a cap's contact line meets the cap's edge but no rim the walk reads.
const OFF_CAP: &str = "leaves the cap's contact line at a vertex of the profile, or over the rim of a side that is no line, \
    which the class does not walk";

/// A corner of the profile between its faces `names`, as a message names it.
fn corner_named(names: &[String;2]) -> String { format!("the corner of `{}` and `{}`",names[0],names[1]) }

impl Piece {
    /// The piece as a message names it.
    fn named(&self) -> String {
        match self {
            Piece::Face(_,s) => format!("`{}`",s.name),
            Piece::Corner(c) => corner_named(&c.names),
            Piece::Ruled(r) => r.ruling.named(),
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

    /// Which cap's contact line it is (0 the prism's `start`, 1 its `end`), where it is one.
    fn cap(&self) -> Option<usize> { match self { Piece::Line(Line {cap,..}) => *cap,_ => None } }

    /// Whether it is a fan: a convex corner's, a cap's rim's or a crease's, every normal between
    /// two faces' carried at its points.
    fn fan(&self) -> bool { matches!(self,Piece::Corner(_) | Piece::Ruled(Ruled {ruling:Ruling::Rib {..} | Ruling::Rim {..},..})) }

    /// The edge of `SweepContacts::edges` whose convex corner's fan it is, where it is one.
    fn edge(&self) -> Option<usize> {
        match self {
            Piece::Corner(Corner {edge,..}) | Piece::Ruled(Ruled {ruling:Ruling::Rib {edge,..},..}) => Some(*edge),
            _ => None,
        }
    }

    /// Which end of it (its first station's, 0, or its last's, 1) meets side `patch`, where it is
    /// a cap's rim or a crease that side runs to (#684 AE, AG).
    fn rim_of(&self,patch: usize) -> Option<f64> {
        match self {
            Piece::Ruled(Ruled {ruling:Ruling::Rim {side,across,..},..}) =>
                if *side == patch { Some(0.) } else if *across == Across::Half(patch) { Some(1.) } else { None },
            _ => None,
        }
    }

    /// Why the walk ends where a ring of the piece has no root: past a revolved face's rings, over
    /// a prism's cap, or at a vertex of the prism where a corner's, a rim's or a crease's root runs
    /// off its line.
    fn off(&self) -> &'static str {
        match self {
            Piece::Ruled(Ruled {ruling:Ruling::Side {..},..}) => OVER_CAP,
            Piece::Ruled(_) => VERTEX,
            Piece::Line(_) => SILHOUETTE_OVER_CAP,
            _ => "leaves its face's rings",
        }
    }

    /// The coefficients of the contact equation `a cos + b sin + c` of a revolution's ring at
    /// station `u` under `motion` (a corner's, under the walk's motion its fan was read at). A
    /// prism's piece has none: its ruling's contact is linear along it (`Ruled::root`, S1 `rulings`).
    fn coefficients(&self,u: f64,motion: Motion) -> Result<(f64,f64,f64),envelope::Error> {
        match self {
            Piece::Face(_,s) => s.contact_coefficients(u,motion),
            Piece::Corner(c) => {
                let [nr,nz] = c.normal(u);
                let [r,z] = c.parts;
                Ok((nr*r[0]+nz*z[0],nr*r[1]+nz*z[1],nr*r[2]+nz*z[2]))
            }
            Piece::Ruled(_) | Piece::Line(_) => Err(envelope::Error::Degenerate),
        }
    }

    /// The point of a revolution's ring at station `u` a fraction `turn` of the way round it.
    fn around(&self,u: f64,turn: f64) -> Option<V> {
        match self {
            Piece::Face(_,s) => { let [v0,v1] = s.domain()[1]; s.at(u,v0+(v1-v0)*turn).ok().map(|p| p.position) }
            Piece::Corner(c) => Some(c.ring.at(std::f64::consts::TAU*turn).0),
            Piece::Ruled(_) | Piece::Line(_) => None,
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
/// square to the motion, is sampled as finely there as anywhere. `speed` is how fast the tool's
/// points move, what a contact is nought against (`NOUGHT`). `crossings` are the stations where a
/// side's root reaches a ruling's end at a cap's rim or a crease the walk crosses (#684 AE, AG).
#[derive(Clone,Debug)]
struct Ruled { ruling: Ruling,speed: f64,lengths: Vec<f64>,crossings: Vec<f64> }

#[derive(Clone,Debug)]
enum Ruling {
    /// Patch `patch`, a side, its normal `du × dv` turned out of the tool where `flip` says.
    Side { patch: usize,surface: super::ExtrudedSurface,flip: bool },
    /// The convex corner edge `edge` of `SweepContacts::edges` is: its sides' names, its ruling's
    /// ends (at the first side's `v` 0 and 1) and the outward normals of its sides there.
    Rib { edge: usize,names: [String;2],ends: [V;2],normals: [V;2] },
    /// A convex edge where a plane side `side` ends at its `v` = `end` (#684 AE, AG): a cap's rim,
    /// where the side meets the prism's cap, or a crease, where a drafted prism's half meets the
    /// other half at its parting plane. Its one ruling is the side's edge there, its stations the
    /// normals of its fan, turned from the side's (0) to the cap's or the other half's (1), as a
    /// corner's are.
    Rim { side: usize,end: usize,across: Across,names: [String;2],ends: [V;2],normals: [V;2] },
}

/// What a rim's fan turns onto from its side: cap `0` (the prism's `start`) or `1` (its `end`), or
/// the half of a drafted side beyond its parting plane, patch `usize`, at its `v` 0.
#[derive(Clone,Copy,Debug,PartialEq)]
enum Across { Cap(usize),Half(usize) }

/// How many parameters along a ruled piece its stations' lengths are read at.
const RULED_READS: usize = 256;
/// How small a ruling's contact is, against how fast the tool's points move, to read as nought.
const NOUGHT: f64 = 1e-9;

impl Ruling {
    /// The piece as a message names it.
    fn named(&self) -> String {
        match self {
            Ruling::Side {surface,..} => format!("`{}`",surface.name),
            Ruling::Rib {names,..} => corner_named(names),
            Ruling::Rim {names,across:Across::Cap(_),..} => format!("the rim where `{}` meets `{}`",names[0],names[1]),
            Ruling::Rim {names,across:Across::Half(_),..} => format!("the crease across `{}`",names[0]),
        }
    }

    /// The ruling at parameter `x` along the piece (its side's edge, or its fan's turn): its ends,
    /// and the outward normal along it.
    fn line(&self,x: f64) -> Option<(V,V,V)> {
        match self {
            Ruling::Side {surface,flip,..} => {
                let (a,b) = (surface.at(x,0.).ok()?,surface.at(x,1.).ok()?);
                let n = envelope::contact(a,Motion::identity()).ok()?.normal;
                Some((a.position,b.position,if *flip { n.map(|c| -c) } else { n }))
            }
            Ruling::Rib {ends,normals,..} | Ruling::Rim {ends,normals,..} =>
                Some((ends[0],ends[1],crate::space::slerp(normals[0],normals[1],x))),
        }
    }
}

/// The contact `n·v` under `motion` at the two ends `a` and `b` of a ruling whose normal is `n`.
fn contact(motion: Motion,(a,b,n): (V,V,V)) -> [f64;2] {
    let n = motion.vector(n);
    [a,b].map(|p| dot(n,motion.velocity(p)))
}

/// Where along a ruling its contact, `c` at its two ends and linear between, is nought: none where
/// it is the same all along.
fn nought_at(c: [f64;2]) -> Option<f64> { (c[0] != c[1]).then(|| c[0]/(c[0]-c[1])) }

impl Ruled {
    /// A piece, its stations spread along its characteristic under `motion`.
    fn new(ruling: Ruling,motion: Motion,speed: f64) -> Ruled {
        let lines: Vec<Option<(V,V,V)>> = (0..=RULED_READS).map(|i| ruling.line(i as f64/RULED_READS as f64)).collect();
        let size = lines.iter().flatten().map(|(a,b,_)| distance(*a,*b)).fold(0_f64,f64::max);
        // each ruling's root, held a quarter of the ruling past either cap, and its normal
        let points: Vec<Option<(V,V)>> = lines.into_iter().map(|line| line.map(|(a,b,n)| {
            let v = nought_at(contact(motion,(a,b,n))).unwrap_or(f64::INFINITY);
            (crate::space::lerp(a,b,v.clamp(-0.25,1.25)),n)
        })).collect();
        let mut lengths = vec![0.];
        for w in points.windows(2) {
            let step = match (w[0],w[1]) {
                (Some((p,m)),Some((q,n))) => distance(p,q)+size/std::f64::consts::PI*dot(m,n).clamp(-1.,1.).dacos(),
                _ => 0.,
            };
            lengths.push(lengths.last().expect("a length")+step+1e-3*size/RULED_READS as f64);
        }
        let total = *lengths.last().expect("a length");
        Ruled {ruling,speed,lengths:lengths.into_iter().map(|l| l/total).collect(),crossings:Vec::new()}
    }

    /// The station at parameter `x` along the piece, as `parameter` reads it back.
    fn station(&self,x: f64) -> f64 {
        let f = (x*RULED_READS as f64).clamp(0.,RULED_READS as f64);
        let i = (f.floor() as usize).min(RULED_READS-1);
        self.lengths[i]+(self.lengths[i+1]-self.lengths[i])*(f-i as f64)
    }

    /// The piece with the stations where its root reaches its rulings' ends `ends` (each 0 or 1,
    /// a `v` of its side) among its `crossings`: where the contact at that end is nought, read at
    /// `RULED_READS` parameters and refined between two that differ in sign.
    fn crossing(mut self,motion: Motion,ends: &[usize]) -> Ruled {
        for &end in ends {
            let at = |x: f64| self.ruling.line(x).map(|line| contact(motion,line)[end]);
            let found: Vec<f64> = noughts(&at).into_iter().map(|x| self.station(x)).collect();
            self.crossings.extend(found);
        }
        self.crossings.sort_by(f64::total_cmp);
        self.crossings.dedup();
        self
    }

    /// The parameter along the piece (its side's edge, or its fan's turn) at station `u`.
    fn parameter(&self,u: f64) -> f64 {
        if u <= 0. { return 0. }
        if u >= 1. { return 1. }
        let i = self.lengths.partition_point(|&l| l <= u).clamp(1,self.lengths.len()-1);
        let (a,b) = (self.lengths[i-1],self.lengths[i]);
        ((i-1) as f64+((u-a)/(b-a)).clamp(0.,1.))/RULED_READS as f64
    }

    /// The ruling at station `u`: its ends, and the outward normal along it.
    fn line(&self,u: f64) -> Option<(V,V,V)> { self.ruling.line(self.parameter(u)) }

    /// The contact `n·v` at the two ends of the ruling at station `u` under `motion`, each nought
    /// where it is within `NOUGHT` of it.
    fn contact(&self,u: f64,motion: Motion) -> Option<[f64;2]> {
        self.line(u).map(|line| contact(motion,line).map(|c| if c.abs() <= NOUGHT*self.speed { 0. } else { c }))
    }

    /// The root of the ruling at station `u`, and its normal: none where it lies past a cap, and
    /// refused where the whole ruling is in contact (S1 says where that is in the blank).
    fn root(&self,u: f64,motion: Motion) -> Result<Option<(V,V)>,String> {
        let (a,b,n) = self.line(u).ok_or("is read off its side")?;
        let c = self.contact(u,motion).ok_or("is read off its side")?;
        if c == [0.,0.] { return Err("is in contact all along a ruling".into()) }
        Ok(nought_at(c).filter(|v| (-1e-12..=1.+1e-12).contains(v)).map(|v| (crate::space::lerp(a,b,v.clamp(0.,1.)),n)))
    }
}

/// A line of a prism every point of which is in contact: where the motion moves each ruling's
/// points alike (a slide; a screw along the prism's own extrusion), a side or a corner's fan is in
/// contact along the whole ruling at each root of `n·v` across it, the silhouette, and nowhere else
/// (#684 M); and on a cap, whose contact is linear across its plane, the stretch of the line where
/// that is nought within the profile, from one rim to another (`cap`, which cap it is on, #684 AE).
/// A station on it is the fraction of the way from its first end to its second.
#[derive(Clone,Debug)]
struct Line { named: String,ends: [V;2],normal: V,cap: Option<usize> }

/// The parameters of `[0, 1]` where `at` is nought: read at `RULED_READS` parameters, each a nought
/// read there or one refined between two that differ in sign, and none twice.
fn noughts(at: &dyn Fn(f64) -> Option<f64>) -> Vec<f64> {
    let x = |i: usize| i as f64/RULED_READS as f64;
    let read: Vec<Option<f64>> = (0..=RULED_READS).map(|i| at(x(i))).collect();
    let mut found = Vec::new();
    for i in 0..RULED_READS {
        let (Some(c0),Some(c1)) = (read[i],read[i+1]) else { continue };
        // a nought at a read parameter is the bracket's that ends there
        if c1 == 0. { found.push(x(i+1)) } else if c0 == 0. && i == 0 { found.push(x(0)) } else if c0*c1 < 0. {
            found.push(crate::roots::bracketed_root(at,x(i),c0,x(i+1),c1,1e-14))
        }
    }
    found
}

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
/// corner's sweep is no part of the boundary, and T2 refuses one that reaches the blank).
///
/// Under a turn or a screw about any line but the prism's own extrusion, each piece is read ruling
/// by ruling (`Ruled`), and the folds a side's root runs onto partway along it are pieces too, each
/// its line read as a corner's ruling is (`Ruling::Rim`): where a drafted prism runs both ways
/// from its parting plane, the convex crease across each plane side there, whose fan turns from
/// the half below's normal to the half above's (#684 AG); and where the tool is a prism standing
/// alone (`caps`), the rims where its plane sides meet its caps, and on each cap its contact line,
/// `n·v` being linear across the cap's plane, within the profile from rim to rim (#684 AE). A
/// side's stations take in those where its root reaches a ruling's end at such a fold
/// (`Ruled::crossing`), so the walk steps onto it there. Otherwise a cap is not walked: T1 asks
/// that it stay clear of the blank (`admission::ends`).
///
/// Where the motion moves every point of each ruling alike — a slide, or a screw along the
/// extrusion — the contact is the same all along each ruling, so the rulings in contact are whole:
/// the silhouette, a ruling at each root of `n·v` across each side and each corner's fan (`Line`),
/// read at `RULED_READS` parameters and refined between two that differ in sign. A piece in
/// contact all over there (a side along the slide, a corner moving along itself) is refused where
/// its path enters the blank (S1): every point of it is on the boundary, the face kept in its own
/// plane.
fn prism(contacts: &SweepContacts,tool: &super::SpatialField,motion: Motion,screw: &Screw,speed: f64,caps: bool,
    enters: &dyn Fn(V) -> Option<f64>) -> Result<Vec<Piece>,Failure> {
    let sides: Vec<(usize,&super::ExtrudedSurface)> = contacts.patches().iter().enumerate()
        .filter_map(|(k,s)| match s { super::ToolSurface::Extruded(e) => Some((k,e)),_ => None }).collect();
    if sides.is_empty() { return Ok(Vec::new()) }
    // each side's normal turned out of the tool as it is in the side's middle, where nothing else
    // is near: the chart orients it the same way all along the side
    let mut rulings: Vec<Ruling> = sides.iter().map(|&(patch,surface)| {
        let flip = surface.at(0.5,0.5).ok().and_then(|p| envelope::contact(p,Motion::identity()).ok()
            .map(|c| dot(outward(tool,p.position,c.normal),c.normal) < 0.)).unwrap_or(false);
        Ruling::Side {patch,surface:surface.clone(),flip}
    }).collect();
    for (edge,e) in contacts.edges().iter().enumerate() {
        let ([a,b],[ua,ub]) = (e.patches,e.ends);
        let (Some(ia),Some(ib)) = (sides.iter().position(|(k,_)| *k == a),sides.iter().position(|(k,_)| *k == b)) else { continue };
        let (Some((p0,p1,na)),Some((_,_,nb))) = (rulings[ia].line(ua),rulings[ib].line(ub)) else { continue };
        if dot(na,nb).clamp(-1.,1.).dacos() <= LEAST_TURN { continue }
        let Some((_,[pa,pb])) = contacts.corner(e) else { continue };
        if !(tool.value(crate::space::lerp(pa,pb,0.5)) < 0.) { continue }
        rulings.push(Ruling::Rib {edge,names:[sides[ia].1.name.clone(),sides[ib].1.name.clone()],ends:[p0,p1],normals:[na,nb]});
    }
    // the motion moves a ruling's points alike where it turns nothing about any line across it:
    // `v(b) − v(a) = ω × (b − a)`, nought under a slide or for a ruling along the screw's axis
    let alike = screw.is_slide() || sides.iter().all(|(_,s)| [0.,0.5,1.].iter().all(|&x| match (s.at(x,0.),s.at(x,1.)) {
        (Ok(a),Ok(b)) => { let d = sub(b.position,a.position); norm(cross(screw.axis,d)) <= NOUGHT*norm(d) }
        _ => false,
    }));
    if alike { return silhouette(contacts,&rulings,motion,speed,enters) }
    // a plane side's outward normal, the same all along it
    let normal = |patch: usize| rulings.iter().find(|r| matches!(r,Ruling::Side {patch:p,..} if *p == patch)).and_then(|r| r.line(0.5))
        .map(|(_,_,n)| n);
    let mut rims: Vec<Ruling> = Vec::new();
    // each crease where a drafted prism's halves meet at its parting plane, the half below's edge
    // at its `v = 1`, its fan turned from that half's normal to the half above's (#684 AG)
    for &[below,above] in contacts.creases() {
        let Some(&(_,s)) = sides.iter().find(|(k,_)| *k == below) else { continue };
        let (Some(na),Some(nb)) = (normal(below),normal(above)) else { continue };
        if !s.straight() || dot(na,nb).clamp(-1.,1.).dacos() <= LEAST_TURN { continue }
        let (Ok(a),Ok(b)) = (s.at(0.,1.),s.at(1.,1.)) else { continue };
        rims.push(Ruling::Rim {side:below,end:1,across:Across::Half(above),names:[s.name.clone(),s.name.clone()],
            ends:[a.position,b.position],normals:[na,nb]});
    }
    // a lone prism's caps (#684 AE): each plane side's rim on each, the side's edge at its `v` 0 or
    // 1, its fan turned from the side's normal to the cap's; and each cap's contact line
    let mut lines: Vec<Line> = Vec::new();
    if caps { for surface in contacts.patches() {
        let (super::ToolSurface::Cap(cap),Some(c)) = (surface,surface.cap()) else { continue };
        let Ok(p) = surface.at(0.5,0.5) else { continue };
        // `du × dv` is a cap's outward normal (`CapSurface`)
        let n = crate::space::unit(cross(p.du,p.dv));
        for &(patch,s) in &sides {
            let Some(ns) = normal(patch).filter(|_| s.straight()) else { continue };
            let (Ok(a),Ok(b)) = (s.at(0.,c as f64),s.at(1.,c as f64)) else { continue };
            rims.push(Ruling::Rim {side:patch,end:c,across:Across::Cap(c),names:[s.name.clone(),cap.name.clone()],
                ends:[a.position,b.position],normals:[ns,n]});
        }
        lines.extend(cap_lines(tool,surface,c,n,&sides,motion));
    } }
    // each side's stations where its root reaches a rim or a crease above
    let ends = |patch: usize| -> Vec<usize> { rims.iter().filter_map(|r| match *r {
        Ruling::Rim {side,end,..} if side == patch => Some(end),
        Ruling::Rim {across:Across::Half(other),..} if other == patch => Some(0),
        _ => None,
    }).collect() };
    let mut pieces: Vec<Piece> = rulings.into_iter().map(|r| {
        let ends = match &r { Ruling::Side {patch,..} => ends(*patch),_ => Vec::new() };
        Piece::Ruled(Ruled::new(r,motion,speed).crossing(motion,&ends))
    }).collect();
    pieces.extend(rims.into_iter().map(|r| Piece::Ruled(Ruled::new(r,motion,speed))));
    pieces.extend(lines.into_iter().map(Piece::Line));
    Ok(pieces)
}

/// The silhouette of a prism under a motion that moves each ruling's points alike (`prism`): each
/// piece's rulings where `n·v`, the same all along each, is nought. A drafted prism running both
/// ways from its parting plane is refused at T1: its silhouette turns at the crease, at a vertex
/// of the prism or a point of a crease that is no line, a sharp edge of the sweep.
fn silhouette(contacts: &SweepContacts,rulings: &[Ruling],motion: Motion,speed: f64,enters: &dyn Fn(V) -> Option<f64>)
    -> Result<Vec<Piece>,Failure> {
    if let Some(&[below,_]) = contacts.creases().first() {
        return Err((Condition::Tool,format!("`{}` runs both ways from its parting plane, where a crease crosses each side, which \
            the class does not walk under a slide, or a screw along the prism's extrusion: its silhouette turns at the crease, a \
            sharp edge of the sweep",contacts.patches()[below].name()),None))
    }
    let mut lines: Vec<Line> = Vec::new();
    for ruling in rulings {
        let at = |x: f64| ruling.line(x).map(|line| contact(motion,line)[0]);
        if (0..=RULED_READS).filter_map(|i| at(i as f64/RULED_READS as f64)).all(|c| c.abs() <= NOUGHT*speed) {
            // a face moving in its own plane, read across a coarse net of its rulings
            let entering = (0..=16).filter_map(|i| ruling.line(i as f64/16.))
                .flat_map(|(a,b,_)| (0..=16).map(move |j| crate::space::lerp(a,b,j as f64/16.))).find_map(|p| enters(p).map(|t| (t,p)));
            if let Some((t,p)) = entering {
                return Err((Condition::Regular,format!("{} moves along itself, every point of it in contact: the motion keeps \
                    it in its own plane, which the class does not sweep",ruling.named()),witness(contacts.motion(),Some(t),p)))
            }
            continue
        }
        for x in noughts(&at) {
            let Some((a,b,n)) = ruling.line(x) else { continue };
            let size = distance(a,b);
            if lines.iter().any(|l| distance(l.ends[0],a) <= 1e-9*size && distance(l.ends[1],b) <= 1e-9*size) { continue }
            lines.push(Line {named:ruling.named(),ends:[a,b],normal:n,cap:None});
        }
    }
    Ok(lines.into_iter().map(Piece::Line).collect())
}

/// Cap `c`'s contact lines (#684 AE), `n` its outward normal: its contact `n·v` is linear across its
/// plane, so it is nought along a line (the motion turns about a line across the cap's normal,
/// as `prism` reads it), and each stretch of that line within the profile, between two of the
/// points where it crosses the sides' edges there (each read as `noughts` reads a ruling's, in
/// order along the line, and paired where the stretch between lies on the tool), is a piece.
fn cap_lines(tool: &super::SpatialField,surface: &super::ToolSurface,c: usize,n: V,sides: &[(usize,&super::ExtrudedSurface)],
    motion: Motion) -> Vec<Line> {
    let f = |q: V| contact(motion,(q,q,n))[0];
    let corners: Vec<V> = [(0.,0.),(1.,0.),(0.,1.)].iter().filter_map(|&(u,v)| surface.at(u,v).ok().map(|s| s.position)).collect();
    let [o,a,b] = corners[..] else { return Vec::new() };
    // the line's direction in the cap's plane, square to the contact's gradient there
    let (fo,du,dv) = (f(o),sub(a,o),sub(b,o));
    let gradient = crate::space::add(scale(du,(f(a)-fo)/dot(du,du)),scale(dv,(f(b)-fo)/dot(dv,dv)));
    let along = cross(n,gradient);
    let size = norm(du).max(norm(dv));
    let mut crossed: Vec<V> = Vec::new();
    for &(_,s) in sides {
        let at = |x: f64| s.at(x,c as f64).ok().map(|q| f(q.position));
        for x in noughts(&at) {
            let Ok(q) = s.at(x,c as f64) else { continue };
            if crossed.iter().all(|&r| distance(r,q.position) > 1e-9*size) { crossed.push(q.position) }
        }
    }
    crossed.sort_by(|&p,&q| dot(p,along).total_cmp(&dot(q,along)));
    crossed.windows(2).filter(|w| {
        let middle = crate::space::lerp(w[0],w[1],0.5);
        tool.value(sub(middle,scale(n,1e-6*(1.+norm(middle))))) < 0.
    }).map(|w| Line {named:format!("`{}`",surface.name()),ends:[w[0],w[1]],normal:n,cap:Some(c)}).collect()
}

/// S1 of a prism read ruling by ruling (#684 M): no ruling in contact all along it where its path
/// enters the blank. There the characteristic runs along the ruling, off the walked stretch, which
/// crosses each ruling once; a side every ruling of which is in contact is a face the motion keeps
/// in its own plane (a side square to a turn). A ruling's contact is nought at both its ends there
/// (`Ruled::contact`), so between two of `rows` stations across each piece where both ends'
/// contacts change sign (or are nought), the least of the two together is sought
/// (`roots::brent`), and a ruling found in contact all along with a point whose path enters the
/// blank is refused. Asked before the walk, whose ring there is degenerate, so the row is named.
fn rulings(pieces: &[Piece],rows: usize,motion: Motion,enters: &dyn Fn(V) -> Option<f64>,family: &Family) -> Result<(),Failure> {
    let station = |i: usize| i as f64/rows as f64;
    for piece in pieces {
        let Piece::Ruled(ruled) = piece else { continue };
        let read: Vec<Option<[f64;2]>> = (0..=rows).map(|i| ruled.contact(station(i),motion)).collect();
        let size = |u: f64| ruled.contact(u,motion).map_or(f64::INFINITY,|[c0,c1]| c0.dhypot(c1));
        let whole = read.iter().all(|c| *c == Some([0.,0.]));
        // the rulings already read for a point that enters, each a station's or a window's least
        let mut asked = vec![false;rows+1];
        for i in 0..rows {
            let (Some([a0,a1]),Some([b0,b1])) = (read[i],read[i+1]) else { continue };
            if a0*b0 > 0. || a1*b1 > 0. { continue }
            let (least,u) = crate::roots::brent(&size,station(i),station(i+1),1e-12,80,|_,_| false);
            let mut candidates = Vec::new();
            for k in [i,i+1] { if read[k] == Some([0.,0.]) && !std::mem::replace(&mut asked[k],true) { candidates.push(station(k)) } }
            if least == 0. && !candidates.contains(&u) { candidates.push(u) }
            let entering = candidates.into_iter().filter_map(|u| ruled.line(u))
                .flat_map(|(a,b,_)| (0..=64).map(move |j| crate::space::lerp(a,b,j as f64/64.))).find_map(|p| enters(p).map(|t| (t,p)));
            let Some((t,p)) = entering else { continue };
            let why = if whole { format!("{} is in contact all over: the motion keeps it in its own plane, which the class does not \
                sweep",piece.named()) } else { format!("a ruling of {} is in contact all along it, where the characteristic runs along \
                the ruling, which the class does not sweep",piece.named()) };
            return Err((Condition::Regular,why,witness(family,Some(t),p)))
        }
    }
    Ok(())
}
