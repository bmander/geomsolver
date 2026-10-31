//! Sweeps of constant twist (docs/generating-sweeps.md): a tool carried by a screw, or by either
//! of its limits, a turn about one axis and a slide along one line (#684). Each moves every point
//! of the tool the same way, relative to the tool, at every time. So whether a tool point is on the
//! swept boundary does not depend on the time. The points that are form the tool's
//! **characteristic**, a curve fixed on the tool, and the boundary is that curve carried by the
//! motion, S(s, t) = M(t)·c(s), whose normal is the tool's there. On a revolved face, the
//! characteristic's ring at meridian station u is A cos θ + B sin θ + C = 0
//! (`RevolvedSurface::contact_coefficients`): two roots, or none. On a prism's side, the
//! characteristic's point on each ruling is the one root of a contact linear along it (#684 M,
//! `Ruled`), or under a slide the whole ruling at each point of its silhouette (`Line`), as on a
//! side each of whose rulings the motion moves alike, where its contact is nought (#684 AF); on a
//! prism's cap, a line, the contact being linear across its plane (#684 AE); and on a flat face
//! the motion keeps in its own plane, the whole face, read as the line across it from where it
//! stands least across the motion's sections to where it stands most (`Flat`, #684 O). Nothing is
//! traced.
//! The sheet is read along its **section**: square to the axis for a screw and a slide, which
//! carry one such plane to the next, and a half-plane through the axis for a turn, which carries
//! one round to the next (`Screw::station`); a turn's sheet is its section revolved, a slide's its
//! section extruded.
//!
//! `Characteristic::walk` finds the stretch of it that reaches the blank by walking one root
//! from ring to ring and across the faces of the tool's profile, and round a convex corner's fan
//! where the faces' roots part at it (`Piece::Corner`, #684 E): the corner's circle, its stations
//! the normals between its faces' (a prism's corner, its ruling; a cap's rim or a drafted crease,
//! its line, stepped onto partway along a side, #684 AE, AG). Where both faces' roots meet the
//! corner at one point, or a fan it runs round stands at one point, every normal of the fan is in
//! contact there: it steps straight across, and the sheet is creased along that point's path
//! (`Characteristic::sharp`, #684 V). Where it comes round to where it began without leaving the
//! blank, the characteristic closes on itself, a tool's passed wholly through the blank: the
//! stretch is the whole loop, begun again at a crease (`Characteristic::closed`, #684 W). Under a
//! slide along the tool's own axis every ring touches nowhere or all round, and the characteristic
//! is the ring of greatest radius alone, closed, its sheet the cylinder it sweeps
//! (`along_axis`, #684 X); under a turn about an axis the tool's own meets, the ring whose normals
//! pass through where they meet touches all round and sweeps a zone of the sphere there, and the
//! walk turns onto it half round and off onto the branch that leaves the zone, the other branch
//! trimmed (`rounds`, `Walker::sides`, `Walker::under`, #684 X). It asks
//! the class's rows of that stretch as it goes: regular (S1), one stretch and one root a ring (S2),
//! no fold (S3), and no crossing within a lead (S4). The sheet is built from the same walk
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

/// Why a revolution's ring has no roots where every point of it is in contact: a smooth join to a
/// flat the motion keeps in its plane reads it by this (#684 O).
const ALL_ROUND: &str = "is in contact all round a ring";

/// Why a characteristic closed on itself with no crease is refused (#684 W): its sheet's first and
/// last rows, one curve, would meet where the sheet is smooth, and the class closes a sheet along
/// the characteristic only where it is creased, its fit joined there by a knot standing three times.
const SMOOTH_LOOP: &str = "where its sheet would close on itself smoothly, which the class closes only at a sharp edge (not built)";
/// Why one under a whole turn is refused (#684 W): its sheet would close round the turn as well.
const WHOLE_LOOP: &str = "which the class does not split a blank by (not built)";

/// Why the walk ends where a revolution's face meets the tool's axis at an angle: there every
/// normal of the cone round the apex is the tool's, and the sweep has a sharp edge along the
/// apex's path, which the class builds only at a corner that turns to a circle (#684 V).
const APEX: &str = "meets the tool's axis at an apex of its profile, where every normal of the cone round it is in \
    contact and the sweep has a sharp edge along the apex's path (not built)";

/// Why the walk ends where a revolution's face meets the tool's axis square to it, at a pole (a disc's
/// centre): the ring there is a point, and the characteristic would run on across the axis, onto
/// the other root of the rings beside it, which the walk does not step to (#684 X: under a turn
/// about an axis the tool's meets, every ring's two roots lie in the plane of the two axes, and a
/// face's characteristic runs through its pole).
const POLE: &str = "meets the tool's axis at a pole of its profile, where its characteristic runs on across the axis to \
    the other root of each ring beside it, which the walk does not cross to (not built)";

/// Whether revolved face `s` meets its axis at station `u`: its ring there no circle, or one of no
/// radius to `tolerance`.
fn at_axis(s: &RevolvedSurface,u: f64,tolerance: f64) -> bool { s.ring(u).map_or(true,|r| r.radius <= tolerance) }

/// Whether revolved face `s` meets its axis at station `u` at an apex (`at_axis`), its normal beside
/// it (a millionth of the meridian in) not along the axis (where it is, at a smooth pole, no normal
/// but the axis's is the tool's).
fn apex(s: &RevolvedSurface,u: f64,tolerance: f64) -> bool {
    if !at_axis(s,u,tolerance) { return false }
    let beside = if u > 0.5 { u-1e-6 } else { u+1e-6 };
    let Some(n) = s.at(beside,0.).ok().and_then(|p| envelope::contact(p,Motion::identity()).ok()).map(|c| c.normal) else { return false };
    dot(n,s.axis_direction()).abs() < 1.-1e-9
}

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
    /// The first and last nodes that reach the blank (of a closed characteristic, its first and last).
    pub reach: [usize;2],
    /// The least sine between the characteristic's tangent and the screw's velocity in the reach.
    pub least_factor: f64,
    /// How many rings were sampled, over every face.
    pub samples: usize,
    /// Whether the motion is a turn carried round a whole turn or more, read as one whole turn: its
    /// sheet is its section revolved all the way round, closing on itself (#684 N).
    pub whole: bool,
    /// Whether the characteristic closes on itself within the blank's reach, as a tool's does that is
    /// passed wholly through the blank (#684 W): its last node is its first, at a crease, and `reach`
    /// is all of them. Its sheet is a tube, closing on itself along the characteristic.
    pub closed: bool,
    /// The nodes at which the characteristic passes a convex corner, a rim or a crease whose fan
    /// stands at one point, the node itself (#684 V), each with that fan's piece, in order.
    sharp: Vec<(usize,usize)>,
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
    tool: &'a super::SpatialField,scale: f64,tolerance: f64,meets: bool }

/// One pass of the walk: the stations each face was sampled at, its rings there, the stretch
/// walked through the blank (face, ring, root), in order along the characteristic, and the steps
/// it took straight across a convex corner whose fan stands at one point (`Step::Sharp`): the two
/// roots either side and the fan.
/// Where the characteristic closes on itself (`closed`, #684 W), `order` is the loop, its last root
/// its first.
/// `sides` are which way the boundary goes on from each end of each ring in contact all round (#684 X).
struct Pass { stations: Vec<Vec<f64>>,rings: Vec<Vec<Ring>>,order: Vec<(usize,usize,usize)>,sharp: Vec<([(usize,usize,usize);2],usize)>,
    closed: bool,sides: Sides }

impl Pass {
    fn root(&self,x: (usize,usize,usize)) -> &Root { root(&self.rings,x) }
    fn samples(&self) -> usize { self.rings.iter().map(Vec::len).sum() }
}

impl Walker<'_> {
    /// Whether the tool has a flat face the motion keeps in its own plane, read across (#684 O).
    fn flats(&self) -> bool { self.pieces.iter().any(|p| matches!(p,Piece::Flat(Flat {line:Ok(_),..}))) }

    /// Whether the tool has a ring in contact all round under a turn, walked half round, or the turn's
    /// axis meets a revolved face's, so that every ring's two roots lie in the plane of the two axes
    /// and the walk runs from the one to the other round the profile (#684 X).
    fn rounds(&self) -> bool { self.meets || self.pieces.iter().any(|p| matches!(p,Piece::Round(Round {of:Some(_),..}))) }

    /// Which way from each end of each ring in contact all round under a turn the boundary goes on
    /// (#684 X), by the ring's piece and its end: the way along the piece it is a ring of. The ring
    /// sweeps a zone of the sphere about the pole, which the turn keeps; read in a half-plane through
    /// the turn's axis, each branch through an end goes on one way (`arm`, read where it has moved
    /// off the end, `moved`), and the boundary's is the one that leaves the zone's angles from the
    /// axis (`Round::lean`): from its highest end the one that rises more for its change of distance
    /// from the pole, from its lowest the one that falls more. The other runs back within them, under
    /// the zone, and the ring trims it.
    fn sides(&self,stations: &[Vec<f64>],rings: &[Vec<Ring>]) -> Sides {
        let mut sides = Sides::new();
        for (r,piece) in self.pieces.iter().enumerate() {
            let Piece::Round(round @ Round {of:Some(_),..}) = piece else { continue };
            for end in 0..2 {
                let leaning: Vec<(isize,f64)> = [-1,1].into_iter().filter_map(|dir| {
                    let start = arm(stations,rings,round,end,dir)?;
                    Some((dir,round.lean(self.moved(stations,rings,start,round.at(end as f64).0)?,end)))
                }).collect();
                if let [(a,x),(b,y)] = leaning[..] { sides.insert((r,end),if (x < y) == (end == 0) { a } else { b }); }
            }
        }
        sides
    }

    /// Where the walk from `start` first stands off `q`: a branch through an end of a ring in contact
    /// all round, which stands there along a corner's fan until it runs onto a face.
    fn moved(&self,stations: &[Vec<f64>],rings: &[Vec<Ring>],(mut at,mut dir): ((usize,usize,usize),isize),q: V) -> Option<V> {
        for _ in 0..4*self.ask.rows {
            let p = root(rings,at).position;
            if distance(p,q) > 1e-6*self.scale { return Some(p) }
            match step(self.contacts,&self.pieces,stations,rings,at,dir,self.scale,&Arms::new()) {
                Step::Next(next,d) | Step::Sharp(next,d,_) => { at = next; dir = d }
                Step::End(_) => return None,
            }
        }
        None
    }

    /// From each end of each ring in contact all round, the root the walk steps onto the way `sides`
    /// says and the way on; and each branch the other way, which the ring trims, with the ring: walked
    /// from that end until it comes to an end of such a ring (once off its own), ends, or runs `MARGIN`
    /// rings past the blank.
    fn arms(&self,stations: &[Vec<f64>],rings: &[Vec<Ring>],sides: &Sides) -> (Arms,Vec<(usize,Vec<(usize,usize,usize)>)>) {
        let (mut arms,mut trimmed) = (Arms::new(),Vec::new());
        let ends: Vec<V> = self.pieces.iter().filter_map(|p| match p { Piece::Round(r @ Round {of:Some(_),..}) => Some([r.at(0.).0,r.at(1.).0]),
            _ => None }).flatten().collect();
        let near = |p: V,q: V| distance(p,q) <= 1e-7*self.scale;
        for (&(r,end),&dir) in sides {
            let Piece::Round(round) = &self.pieces[r] else { continue };
            if let Some(a) = arm(stations,rings,round,end,dir) { arms.insert((r,end),a); }
            let Some((mut at,mut d)) = arm(stations,rings,round,end,-dir) else { continue };
            let q = round.at(end as f64).0;
            let (mut walked,mut since,mut off) = (Vec::new(),0,false);
            for _ in 0..rings.iter().map(Vec::len).sum::<usize>() {
                let p = root(rings,at).position;
                off |= !near(p,q);
                if walked.contains(&at) || (off && ends.iter().any(|&e| near(p,e))) { break }
                walked.push(at);
                if root(rings,at).reach.is_some() { since = 0 } else { since += 1; if since == MARGIN { break } }
                match step(self.contacts,&self.pieces,stations,rings,at,d,self.scale,&Arms::new()) {
                    Step::Next(next,dd) | Step::Sharp(next,dd,_) => { at = next; d = dd }
                    Step::End(_) => break,
                }
            }
            trimmed.push((r,walked));
        }
        (arms,trimmed)
    }

    /// S2 of the branches the rings in contact all round trim (#684 X): each root of one that reaches
    /// the blank (off its ring's ends), read in a half-plane through the turn's axis, lies within the
    /// zone of a ring that trims it — nearer the pole than the ring where the tool lies within the
    /// sphere there, farther where without — its normal facing the zone, and the line from it to the
    /// zone, straight away from the pole and ending within the zone's angles, crosses no other part of
    /// the characteristic's section. The tool's material lies behind it, and before it what the tool
    /// sweeps just under the zone, nothing of the boundary between: the branch is within the sweep.
    fn under(&self,order: &[(usize,usize,usize)],rings: &[Vec<Ring>],trimmed: &[(usize,Vec<(usize,usize,usize)>)]) -> Result<(),Failure> {
        let (screw,family) = (&self.screw,self.contacts.motion());
        let flat = |p: V| [screw.radius(p),screw.height(p)];
        let round = |k: usize| matches!(self.pieces[k],Piece::Round(_));
        let segments = |walked: &[(usize,usize,usize)]| walked.windows(2).map(|w| [flat(root(rings,w[0]).position),flat(root(rings,w[1]).position)])
            .collect::<Vec<_>>();
        // the walk's section but along a ring all round (whose section the zone is), and each trimmed branch's
        let walked: Vec<[[f64;2];2]> = order.windows(2).filter(|w| !(round(w[0].0) && round(w[1].0)))
            .map(|w| [flat(root(rings,w[0]).position),flat(root(rings,w[1]).position)]).collect();
        let branches: Vec<Vec<[[f64;2];2]>> = trimmed.iter().map(|(_,w)| segments(w)).collect();
        let crosses = |a: [f64;2],b: [f64;2],[c,d]: [[f64;2];2]| {
            let side = |p: [f64;2],q: [f64;2],x: [f64;2]| (q[0]-p[0])*(x[1]-p[1])-(q[1]-p[1])*(x[0]-p[0]);
            side(a,b,c)*side(a,b,d) < 0. && side(c,d,a)*side(c,d,b) < 0.
        };
        // whether branch `b`'s ring holds its root `w` within its zone
        let holds = |b: usize,w: usize| -> bool {
            let (r,walked_b) = (&trimmed[b].0,&trimmed[b].1);
            let Piece::Round(ring) = &self.pieces[*r] else { return false };
            let t = root(rings,walked_b[w]);
            let (top,bottom) = (ring.at(0.).0,ring.at(1.).0);
            let far = distance(top,ring.pole);
            let pole = flat(ring.pole);
            let heights = { let (a,b) = (screw.height(top),screw.height(bottom)); [a.min(b),a.max(b)] };
            let s = flat(t.position);
            let d = (s[0]-pole[0]).dhypot(s[1]-pole[1]);
            let out = [(s[0]-pole[0])/d,(s[1]-pole[1])/d];
            // the normal's parts away from the turn's axis and along it
            let q = sub(t.position,screw.origin);
            let radial = crate::space::unit(sub(q,scale(screw.axis,dot(q,screw.axis))));
            let m = [dot(t.normal,radial),dot(t.normal,screw.axis)];
            let inside = ring.support == Some(true);
            let g = [pole[0]+far*out[0],pole[1]+far*out[1]];
            (d < far) == inside && (m[0]*out[0]+m[1]*out[1] > 0.) == inside && g[1] >= heights[0] && g[1] <= heights[1]
                && !walked.iter().any(|&line| crosses(s,g,line))
                && !branches.iter().enumerate().any(|(c,lines)| lines.iter().enumerate().any(|(l,&line)| !(c == b && (l+1 == w || l == w))
                    && crosses(s,g,line)))
        };
        let ends: Vec<V> = trimmed.iter().filter_map(|(r,_)| match &self.pieces[*r] { Piece::Round(ring) => Some([ring.at(0.).0,ring.at(1.).0]),
            _ => None }).flatten().collect();
        for (b,(_,walked_b)) in trimmed.iter().enumerate() { for &x in walked_b {
            let t = root(rings,x);
            if t.reach.is_none() || ends.iter().any(|&e| distance(t.position,e) <= 1e-7*self.scale) { continue }
            // held by a ring that trims it: this branch's, or another's that runs through it too
            let by = trimmed.iter().enumerate().filter_map(|(c,(_,other))| other.iter().position(|&y| y == x).map(|v| (c,v)));
            if by.clone().any(|(c,v)| holds(c,v)) { continue }
            let Piece::Round(ring) = &self.pieces[trimmed[b].0] else { continue };
            return Err((Condition::Once,format!("the characteristic on {} runs on where {} trims it, and the class does not \
                read it within the zone the ring sweeps",self.pieces[x.0].named(),ring.named),witness(family,t.reach,t.position)))
        } }
        Ok(())
    }

    /// Sample each face's rings at its stations, and walk the stretch that reaches the blank from
    /// its first point to `MARGIN` rings past its last each way: S2 asked of every ring and of the
    /// stretch, and the walk's own end (a double point, a jump at a corner) refused as S1.
    fn pass(&self,stations: Vec<Vec<f64>>,sides: Option<&Sides>) -> Result<Pass,Failure> {
        let (contacts,ask,screw,family) = (self.contacts,self.ask,&self.screw,self.contacts.motion());
        let pieces = &self.pieces;
        let outward = |p: V,n: V| outward(self.tool,p,n);
        // where each ring's roots are, and whether each one's path enters the blank: on every core
        let rings: Vec<Vec<Ring>> = crate::par::indices(pieces.len(),|k| stations[k].iter().map(|&u|
            ring(&pieces[k],u,self.motion,self.tolerance,&outward,&|p| entering(self.tool,family,screw,ask,p))).collect());
        // a revolution's face joined smoothly to a flat disc the motion keeps in its own plane (a
        // bull-nose's round to its flat end): its ring at the join is in contact all round, its
        // normals the flat's, and the characteristic runs on from the flat's line across it at the
        // line's two ends, which lie on the disc's rim, the join (#684 O)
        let mut rings = rings;
        let face = |patch: usize| pieces.iter().position(|p| p.patch() == Some(patch));
        for e in contacts.edges() { for s in 0..2 {
            let (Some(k),Some(l)) = (face(e.patches[s]),face(e.patches[1-s])) else { continue };
            let (Piece::Face(..),Piece::Flat(Flat {line:Ok(ends),normal,..})) = (&pieces[k],&pieces[l]) else { continue };
            let at = if e.ends[s] == 0. { 0 } else { stations[k].len().wrapping_sub(1) };
            if stations[k].get(at) != Some(&e.ends[s]) || !matches!(&rings[k][at],Ring::Degenerate(why) if why == ALL_ROUND) { continue }
            rings[k][at] = Ring::Roots(ends.iter().map(|&p| Root {position:p,normal:*normal,spread:1.,
                reach:entering(self.tool,family,screw,ask,p)}).collect());
        } }
        // S2, ring by ring: two roots of one ring in the blank are two sheets over one station, but
        // where the walk runs from the one across a flat face to the other (asked once it is walked)
        let mut doubled: Vec<(usize,usize,[usize;2])> = Vec::new();
        for (k,face) in rings.iter().enumerate() { for (i,ring) in face.iter().enumerate() {
            let Ring::Roots(roots) = ring else { continue };
            let reaching: Vec<usize> = (0..roots.len()).filter(|&j| roots[j].reach.is_some()).collect();
            if reaching.len() > 1 {
                let r = &roots[reaching[1]];
                if !self.flats() && !self.rounds() { return Err((Condition::Once,format!("both of a ring's characteristic points on {} reach the blank",
                    pieces[k].named()),witness(family,r.reach,r.position))) }
                doubled.push((k,i,[reaching[0],reaching[1]]));
            }
        } }
        let reaching: Vec<(usize,usize,usize)> = rings.iter().enumerate().flat_map(|(k,face)| face.iter().enumerate()
            .flat_map(move |(i,ring)| match ring { Ring::Roots(roots) => roots.iter().enumerate()
                .filter(|(_,r)| r.reach.is_some()).map(|(j,_)| (k,i,j)).collect::<Vec<_>>(),_ => Vec::new() })).collect();
        // each ring in contact all round under a turn (#684 X): which way on from each of its ends is
        // the boundary's (the coarse pass's, read again by the fine), the root that way, and the
        // branch the other way, which the ring trims and the walk does not begin on
        let sides = match sides { Some(sides) => sides.clone(),None => self.sides(&stations,&rings) };
        let (arms,trimmed) = self.arms(&stations,&rings,&sides);
        let cut = |x: &(usize,usize,usize)| trimmed.iter().any(|(_,walked)| walked.contains(x));
        let Some(&seed) = reaching.iter().find(|x| !cut(x)) else {
            return Err((Condition::Reach,"no point of the tool's characteristic passes through the blank in the roll".into(),None))
        };
        let root = |x| root(&rings,x);
        // the walk each way from the seed, to MARGIN rings past the last that reaches; or round to
        // where it began, the seed or a root the walk the other way took (`met`), where the
        // characteristic closes on itself (#684 W): the step onto it is the walk's last, and how many
        // of `met` the loop keeps is given
        type Walked = (Vec<(usize,usize,usize)>,Vec<([(usize,usize,usize);2],usize)>,Option<usize>);
        let walk = |dir: isize,met: &[(usize,usize,usize)]| -> Result<Walked,Failure> {
            let (mut out,mut sharp) = (Vec::new(),Vec::new());
            let (mut at,mut dir) = (seed,dir);
            let mut since = 0;
            loop {
                let taken = step(contacts,pieces,&stations,&rings,at,dir,self.scale,&arms);
                if let Step::Sharp(next,_,fan) = taken { sharp.push(([at,next],fan)) }
                match taken {
                    Step::Next(next,d) | Step::Sharp(next,d,_) => {
                        if next == seed { return Ok((out,sharp,Some(0))) }
                        if let Some(m) = met.iter().position(|&x| x == next) { return Ok((out,sharp,Some(m+1))) }
                        let duplicate = next.0 != at.0;
                        out.push(next);
                        at = next; dir = d;
                        if duplicate { continue }
                        if root(at).reach.is_some() { since = 0 } else { since += 1 }
                        if since == MARGIN { return Ok((out,sharp,None)) }
                    }
                    Step::End(why) => {
                        let last = out.iter().rev().chain(std::iter::once(&seed)).find(|&&x| root(x).reach.is_some()).copied().unwrap_or(seed);
                        return Err((Condition::Regular,format!("the characteristic on {} {why} within {MARGIN} rings of the blank",
                            pieces[at.0].named()),witness(family,root(last).reach,root(last).position)))
                    }
                }
            }
        };
        // back first: where it comes round to the seed, the loop is the seed, what it took the other
        // way round, and the seed again, closing it; else ahead, which comes round onto the roots
        // back took where the loop leaves the blank for fewer rings than the two margins together,
        // and the loop is the seed, ahead's, those of back's it reached the other way round, and the
        // seed again
        let (back,mut sharp,closed) = walk(-1,&[])?;
        let (order,closed) = if closed.is_some() {
            (std::iter::once(seed).chain(back.into_iter().rev()).chain(std::iter::once(seed)).collect::<Vec<_>>(),true)
        } else {
            let (ahead,sharp_ahead,met) = walk(1,&back)?;
            sharp.extend(sharp_ahead);
            match met {
                Some(m) => (std::iter::once(seed).chain(ahead).chain(back[..m].iter().rev().copied())
                    .chain(std::iter::once(seed)).collect(),true),
                None => (back.into_iter().rev().chain(std::iter::once(seed)).chain(ahead).collect(),false),
            }
        };
        // S2: a ring's two roots in the blank both on the walk, a flat face's line or a ring in contact
        // all round between them (#684 X), or either on a branch such a ring trims
        for &(k,i,[a,b]) in &doubled {
            let (x,y) = (order.iter().position(|&w| w == (k,i,a)),order.iter().position(|&w| w == (k,i,b)));
            let across = match (x,y) { (Some(x),Some(y)) => order[x.min(y)..x.max(y)].iter()
                .any(|w| matches!(pieces[w.0],Piece::Flat(_) | Piece::Round(_))),_ => cut(&(k,i,a)) || cut(&(k,i,b)) };
            if !across {
                let r = root((k,i,b));
                return Err((Condition::Once,format!("both of a ring's characteristic points on {} reach the blank",pieces[k].named()),
                    witness(family,r.reach,r.position)))
            }
        }
        // S2: every ring point that reaches the blank is on this one stretch, but a root of a fan
        // the walk stepped straight across where it stands at one point, that point (#684 V)
        let stood = |x: &(usize,usize,usize)| sharp.iter().any(|&([a,_],fan)| x.0 == fan
            && distance(root(*x).position,root(a).position) <= 1e-7*self.scale);
        if let Some(&other) = reaching.iter().find(|x| !order.contains(x) && !stood(x) && !cut(x)) {
            return Err((Condition::Once,format!("the characteristic reaches the blank in more than one stretch (again on {})",
                pieces[other.0].named()),witness(family,root(other).reach,root(other).position)))
        }
        // S2: what the rings in contact all round trim lies within what they sweep (#684 X)
        self.under(&order,&rings,&trimmed)?;
        Ok(Pass {stations,rings,order,sharp,closed,sides})
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
        // (a flat face the motion keeps in its own plane walked across its line, #684 O)
        let mut pieces: Vec<Piece> = Vec::new();
        for (k,s) in contacts.patches().iter().enumerate() {
            let Some(s) = s.revolved() else { continue };
            match revolved_flat(k,s,tool,motion,&screw,speed) {
                Some(flat) => {
                    let [v0,v1] = s.domain()[1];
                    unread_clear(&flat,&|u,v| s.at(u,v0+(v1-v0)*v).ok().map(|p| p.position),&enters,family)?;
                    pieces.push(Piece::Flat(flat));
                }
                None => pieces.push(Piece::Face(k,s.clone())),
            }
        }
        let flats: Vec<usize> = pieces.iter().filter_map(|p| match p { Piece::Flat(f) => Some(f.patch),_ => None }).collect();
        pieces.extend(corners(contacts,tool,motion,&flats).into_iter().map(Piece::Corner));
        pieces.extend(prism(contacts,tool,motion,&screw,speed,ask.caps,&enters)?);
        // the rings in contact all round under a turn about an axis the tool's meets (#684 X)
        let found = rounds(&pieces,motion,&screw,tool,scale);
        pieces.extend(found.into_iter().map(Piece::Round));
        // whether the turn's axis meets a revolved face's own (#684 X)
        let meets = screw.is_turn() && pieces.iter().any(|p| match p {
            Piece::Face(_,s) => s.ring(0.5).is_ok_and(|ring| {
                let (a,w) = (ring.axis,screw.axis);
                let across = cross(a,w);
                norm(across) > 1e-9 && dot(sub(ring.center,screw.origin),across).abs() <= 1e-9*scale*norm(across)
            }),
            _ => false,
        });
        let walker = Walker {contacts,pieces,ask,screw,motion,tool,scale,tolerance,meets};
        let pieces = &walker.pieces;
        let n = ask.rows;
        // a slide along the tool's own axis: its ring of greatest radius in contact all round, the
        // characteristic that ring alone, closed on itself (#684 X)
        if let Some(found) = along_axis(contacts,pieces,&screw,motion,ask,scale,tolerance) { return found }
        // S1 of a prism: no ruling in contact all along it where its path enters the blank
        rulings(pieces,n,motion,&enters,family)?;
        let coarse = walker.pass((0..pieces.len()).map(|k| crossed(&pieces[k],(0..=n).map(|i| i as f64/n as f64).collect()))
            .collect(),None)?;
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
            let fine = walker.pass(stations,Some(&coarse.sides))?;
            samples += fine.samples();
            fine
        };
        let Pass {stations,rings,mut order,sharp: stepped,closed,..} = fine;
        // S1: no ring the walk passes is in contact all round between its stations
        stationary(pieces,&stations,&order,motion,family,&screw,ask)?;
        let root = |x| root(&rings,x);
        let seed = *order.iter().find(|&&x| root(x).reach.is_some()).expect("a pass reaches");
        // a loop begun at a root within one piece's stretch (the root before it on the same piece),
        // on no fan, so that it neither starts on a crossing nor at a fan the nodes below drop
        if closed {
            let cycle = &order[..order.len()-1];
            let within = |r: usize| !pieces[cycle[r].0].fan() && cycle[(r+cycle.len()-1)%cycle.len()].0 == cycle[r].0;
            if let Some(r) = (0..cycle.len()).find(|&r| within(r)) {
                order = cycle[r..].iter().chain(&cycle[..=r]).copied().collect();
            }
        }
        // the nodes, a crossing's second point dropped (its station starts the next step); a step
        // straight across a convex corner whose fan stands at one point a crease at the node before
        // it, with the fan (#684 V)
        let mut nodes: Vec<Node> = Vec::new();
        let mut sharp: Vec<(usize,usize)> = Vec::new();
        let mut from = None;
        for (w,&x) in order.iter().enumerate() {
            let (k,i,_) = x;
            let u = stations[k][i];
            let r = root(x);
            if let Some(&(_,fan)) = stepped.iter().find(|(pair,_)| w > 0 && (*pair == [order[w-1],x] || *pair == [x,order[w-1]])) {
                if let Some(at) = nodes.len().checked_sub(1) { sharp.push((at,fan)) }
            }
            if w > 0 && order[w-1].0 != k && from.is_none() && nodes.last().is_some_and(|m: &Node| distance(m.position,r.position) <= 1e-7*scale) {
                from = Some(u); continue
            }
            let start = from.take().unwrap_or_else(|| nodes.last().map_or(u,|m| m.u));
            nodes.push(Node {patch:k,u,from:start,position:r.position,normal:r.normal,reach:r.reach});
        }
        // a fan the walk runs round that stands at one point, from the node it runs onto it at to
        // its last: there the fan's edge runs along that point's velocity, every normal between its
        // faces is in contact at it, and the sweep has a sharp edge along its path (as where a lined
        // side's ruling under a turn, in a plane through the axis, meets a rim square to that plane).
        // Between two faces' nodes it is that point, a crease, as where the walk steps straight
        // across a corner (#684 V): its nodes are dropped, the crease the node it runs onto it at. At
        // the walk's end, among its last `MARGIN` rings past the blank, it is where the walk ends:
        // its nodes are that point once, the face's beside it. (S1: one at the end that reaches the
        // blank, which the margin leaves none, is refused.)
        let mut w = 0;
        while w < nodes.len() {
            let k = nodes[w].patch;
            if !pieces[k].fan() { w += 1; continue }
            let onto = w;
            while w+1 < nodes.len() && nodes[w+1].patch == k { w += 1 }
            let start = onto.saturating_sub(1);
            let run = &nodes[start..=w];
            if !(run.len() > 1 && run.iter().all(|m| distance(m.position,run[0].position) <= 1e-7*scale)) { w += 1; continue }
            let shift = |sharp: &mut Vec<(usize,usize)>,from: usize,by: usize| for c in sharp.iter_mut().filter(|c| c.0 >= from) { c.0 -= by };
            let between = onto > 0 && w+1 < nodes.len();
            if let (false,Some(at)) = (between,run.iter().find_map(|m| witness(family,m.reach,m.position))) {
                return Err((Condition::Regular,format!("the characteristic on {} stands at one point all round its fan at the end \
                    of the walk, in the blank: the sweep has a sharp edge along that point's path, which the class builds only \
                    between two faces",pieces[k].named()),Some(at)))
            }
            if between {
                nodes.drain(onto..=w);
                shift(&mut sharp,onto,w+1-onto);
                sharp.push((start,k));
                w = onto;
            } else if onto > 0 {
                nodes.truncate(onto);
            } else {
                // the walk starts on the fan: one node at its point, the face's after it as the
                // crossing's dropped point would have been, its station where that face's step starts
                let mut first = nodes[w];
                let next = nodes.get(w+1).copied();
                if let Some(next) = next { first = Node {patch:next.patch,u:next.from,from:next.from,..first} }
                nodes.splice(0..=w,std::iter::once(first));
                shift(&mut sharp,1,w);
                w = 1;
            }
        }
        sharp.sort_by_key(|c| c.0);
        if nodes.len() < 2 {
            return Err((Condition::Regular,format!("the characteristic stands at one point, on {}: the sweep is that point's \
                path, no sheet",pieces[nodes[0].patch].named()),witness(family,root(seed).reach,root(seed).position)))
        }
        // a characteristic closed on itself (#684 W), its last node its first: begun again at a crease,
        // so that the sheet's first and last rows, one curve, meet where it has a sharp edge and is
        // cut and its fit creased anyway. One with no crease would close smoothly on itself, and one
        // turned a whole turn would close round the turn too, a sheet with no edge: refused by name
        if closed {
            let at = witness(family,root(seed).reach,root(seed).position);
            if contacts.whole() {
                return Err((Condition::Regular,format!("the characteristic closes on itself through {} under a whole turn: \
                    the sheet would close both ways, a ring with no edge, {WHOLE_LOOP}",pieces[seed.0].named()),at))
            }
            let n = nodes.len()-1;
            let Some(&w) = creases(&nodes,pieces,&sharp).first() else {
                return Err((Condition::Regular,format!("the characteristic closes on itself through {} with no crease, {SMOOTH_LOOP}",
                    pieces[seed.0].named()),at))
            };
            nodes = nodes[w..=n].iter().chain(&nodes[1..=w]).copied().collect();
            for c in &mut sharp { c.0 = if c.0 >= w { (c.0-w).min(n-w) } else if c.0 >= 1 { c.0+n-w } else { n-w } }
            sharp.sort_by_key(|c| c.0);
        }
        let (first,last) = if closed { (0,nodes.len()-1) } else {
            (nodes.iter().position(|m| m.reach.is_some()).expect("the seed reaches"),
             nodes.iter().rposition(|m| m.reach.is_some()).expect("the seed reaches"))
        };
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
        let creased = creases(&nodes,pieces,&sharp);
        for w in first.max(1)..=last.min(nodes.len()-2) {
            // where the walk runs onto or off a flat face, or straight across a convex corner, the
            // sweep has a sharp edge, its normal turned there from the one face's to the other's,
            // whose sides are read either side (#684 O, V)
            if creased.contains(&w) { continue }
            // at a ring in contact all round's ends the characteristic turns along the motion's path,
            // the edge of the zone the ring sweeps, which is no fold (#684 X)
            let round = |w: usize| matches!(pieces[nodes[w].patch],Piece::Round(_));
            if round(w-1) != round(w) || round(w) != round(w+1) { continue }
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
        let found = Characteristic {screw,nodes,reach:[first,last],least_factor:least,samples,whole:contacts.whole(),closed,sharp,
            pieces:walker.pieces,tool:tool.clone(),motion,tolerance};
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
        let mut walked: Vec<usize> = self.walked().iter().map(|m| m.patch).filter(|&k| self.pieces[k].fan()).collect();
        walked.dedup();
        walked.into_iter().map(|k| self.pieces[k].named()).collect()
    }

    /// The lined sides whose ruling in contact all along it the characteristic runs along (#684 AF),
    /// each as a message names it.
    pub fn lined(&self) -> Vec<String> {
        let mut walked: Vec<usize> = self.walked().iter().map(|m| m.patch).filter(|&k| self.pieces[k].lined().is_some()).collect();
        walked.dedup();
        walked.into_iter().map(|k| self.pieces[k].named()).collect()
    }

    /// The flat faces the motion keeps in their own planes that the characteristic runs across
    /// (#684 O), each as a message names it.
    pub fn flats(&self) -> Vec<String> {
        let mut walked: Vec<usize> = self.walked().iter().map(|m| m.patch).filter(|&k| self.on_flat_piece(k)).collect();
        walked.dedup();
        walked.into_iter().map(|k| self.pieces[k].named()).collect()
    }

    fn on_flat_piece(&self,k: usize) -> bool { matches!(self.pieces[k],Piece::Flat(_)) }

    /// Where the characteristic is one ring all round under a slide (or a screw) along the tool's own
    /// axis (`along_axis`, #684 X): the ring's centre, the axis and its radius. Its sheet is the cylinder
    /// of that radius about the axis, written exactly.
    pub fn cylinder(&self) -> Option<(V,V,f64)> {
        if !(self.closed && !self.screw.is_turn()) { return None }
        let k = self.nodes.first()?.patch;
        let Piece::Round(round) = &self.pieces[k] else { return None };
        (self.nodes.iter().all(|m| m.patch == k) && round.turn == std::f64::consts::TAU && round.normal == [1.,0.])
            .then_some((round.centre,round.axis,round.radius))
    }

    /// The rings in contact all round that the characteristic runs round (#684 X), each as a message
    /// names it.
    pub fn rounds(&self) -> Vec<String> {
        let mut walked: Vec<usize> = self.walked().iter().map(|m| m.patch).filter(|&k| matches!(self.pieces[k],Piece::Round(_))).collect();
        walked.dedup();
        walked.into_iter().map(|k| self.pieces[k].named()).collect()
    }

    /// The nodes each of which ends a stretch of the walk on its piece: all of them, but a closed
    /// characteristic's first, which is its last (#684 W).
    fn walked(&self) -> &[Node] { if self.closed { &self.nodes[1..] } else { &self.nodes } }

    /// Where along the characteristic (`s`, a node's) it runs onto or off a flat face the motion
    /// keeps in its own plane (#684 O), or from one face to the next across a convex corner whose
    /// fan stands at one point (#684 V): the sweep has a sharp edge along that point's path, between
    /// the strip the flat sweeps in its plane and the face beside it, or between the two faces'
    /// sheets, so the sheet is creased there and cut into faces.
    pub fn creases(&self) -> Vec<f64> { creases(&self.nodes,&self.pieces,&self.sharp).into_iter().map(|w| w as f64).collect() }

    /// The convex corners, rims and creases of the tool whose fans stand at one point where the
    /// characteristic passes them, the sheet creased along that point's path (#684 V), each as a
    /// message names it.
    pub fn sharp(&self) -> Vec<String> {
        let mut walked: Vec<usize> = self.sharp.iter().map(|&(_,k)| k).collect();
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

    /// S4: the reach's section does not cross itself (a closed characteristic's read as a closed
    /// polygon, #684 W). The sheet is the same at every station but turned and raised (a screw's),
    /// raised (a slide's) or turned (a turn's), so two of its points meet exactly where two points
    /// of one section do. A turn's section is read in its half-plane, by radius and height.
    fn simple(&self,family: &Family) -> Result<(),Failure> {
        let [first,last] = self.reach;
        let station = self.screw.station(self.nodes[first].position);
        let points: Vec<V> = (first..=last).map(|w| self.on_section(w as f64,station).map(|(p,_)| p)).collect::<Result<_,_>>()
            .map_err(|m| (Condition::Motion,m,None))?;
        let crate::brep::geom::Frame {x:e1,y:e2,..} = crate::brep::geom::Frame::about([0.;3],self.screw.axis);
        let flat: Vec<[f64;2]> = points.iter().map(|&p| if self.screw.is_turn() { [self.screw.radius(p),self.screw.height(p)] }
            else { [dot(p,e1),dot(p,e2)] }).collect();
        // (a closed characteristic's section a closed polygon, its last point its first: its first
        // and last segments neighbours, #684 W)
        if let Some((i,j)) = crossing_of(&flat,self.closed) {
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

/// The nodes at which the sweep is creased, in order: where the walk runs onto or off a flat face
/// the motion keeps in its own plane (#684 O), and the nodes of `sharp`, where it passes a fan that
/// stands at one point (#684 V).
fn creases(nodes: &[Node],pieces: &[Piece],sharp: &[(usize,usize)]) -> Vec<usize> {
    let flat = |w: usize| matches!(pieces[nodes[w].patch],Piece::Flat(_));
    let mut found: Vec<usize> = (0..nodes.len()-1).filter(|&w| flat(w) != flat(w+1)).chain(sharp.iter().map(|&(w,_)| w)).collect();
    found.sort_unstable();
    found.dedup();
    found
}

/// S1 between the walk's stations: a ring in contact all round, every point of it on the
/// boundary, is where a characteristic branches. Under a turn about an axis that the tool's own
/// axis meets, the ring whose normals all pass through the point where they meet is one (found
/// before the walk and walked onto, `rounds`, #684 X: this row refuses one it missed); under a
/// slide square to the tool's axis, or a turn about one parallel to it, a ring whose normals all lie
/// along it, in a plane the motion keeps (a slide along the tool's axis touches only on such rings,
/// read whole by `along_axis`). Either way the
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

/// Under a slide along the tool's own axis (#684 X), the tool's characteristic, where every face of it
/// is a revolution about one line along the slide; else none. A screw about that line is read so too:
/// the turn carries each ring into itself, and each point's normal turns with it, so its contact is
/// the slide's alone. Each ring's normal makes the same
/// angle with the slide all round it, so a ring touches nowhere or, where its normal lies square to
/// the slide, all round: the walk finds no roots. The ring of greatest radius `R` is one, and it
/// sweeps the cylinder of that radius about the axis, every point of which is on the boundary where
/// the blank is swept within it. The characteristic is that ring alone, closed on itself (a
/// `Round`), and its sheet that cylinder, a tube, where two rows hold (each read from the faces'
/// points on the tool at `ROUND_READS` stations along each, as its samples):
///
/// - **S2.** The tool's shadow along the slide is its whole disc: the radii of its faces' points,
///   each stretch between two stations on one face covering at least what its ends' do, run from
///   its axis to `R` with no gap. The sweep is then every point within `R` the tool passes, and the
///   rings of lesser radius in contact all round (a cylinder's every ring, a torus's inner ring)
///   sweep cylinders within it. Where it is a ring, its least radius's ring sweeps a second wall.
/// - **E1.** The tool passes wholly through the blank's heights along the slide: every point of it
///   comes to them after the roll's start and leaves them before its end. Then every point of the
///   blank within `R` is passed by the tool point at its radius and angle, and none beyond `R` is.
///
/// What is removed is the blank within the cylinder, exactly.
fn along_axis(contacts: &SweepContacts,pieces: &[Piece],screw: &Screw,motion: Motion,ask: &Ask,size: f64,tolerance: f64)
    -> Option<Result<Characteristic,Failure>> {
    if screw.is_turn() { return None }
    let faces: Vec<&RevolvedSurface> = contacts.patches().iter().map(|s| s.revolved()).collect::<Option<_>>()?;
    let axis = faces.first()?.axis_direction();
    if norm(cross(axis,screw.axis)) > 1e-9 { return None }
    // a point of the tool's axis, the centre of a ring of its first face, and how far a point stands from it
    let centre = (0..=8).find_map(|i| faces[0].ring(i as f64/8.).ok())?.center;
    let off = |p: V| { let q = sub(p,centre); norm(sub(q,scale(axis,dot(q,axis)))) };
    if faces.iter().any(|s| norm(cross(s.axis_direction(),axis)) > 1e-9
        || (0..=8).filter_map(|i| s.ring(i as f64/8.).ok()).any(|r| off(r.center) > 1e-9*size)) { return None }
    // a screw turns the tool about its own axis as it slides, which moves no ring off itself: the
    // same sweep as the slide's (a drill fed as it turns)
    if !screw.is_slide() && off(screw.origin) > 1e-9*size { return None }
    let (family,tool) = (contacts.motion(),contacts.source_material());
    // each face's points at its stations, where they are on the tool (a face of an intersection may
    // run past it), with their radii
    let point = |s: &RevolvedSurface,u: f64| s.at(u,s.domain()[1][0]).ok().map(|p| p.position).filter(|&p| tool.value(p).abs() < 1e-6);
    let read: Vec<Vec<Option<(f64,V)>>> = faces.iter().map(|s| (0..=ROUND_READS).map(|i| point(s,i as f64/ROUND_READS as f64)
        .map(|p| (off(p),p))).collect()).collect();
    let samples = read.iter().map(Vec::len).sum();
    // the greatest radius, refined between the stations either side of where it is read greatest
    let (mut most,mut at,mut best) = (f64::NEG_INFINITY,[0.;3],None);
    for (k,face) in read.iter().enumerate() { for (i,x) in face.iter().enumerate() {
        if let Some((r,p)) = *x { if r > most { (most,at,best) = (r,p,Some((k,i))) } }
    } }
    if let Some((k,i)) = best {
        let step = 1./ROUND_READS as f64;
        let u = i as f64*step;
        let (least,v) = crate::roots::brent(&|v| point(faces[k],v).map_or(f64::INFINITY,|p| -off(p)),(u-step).max(0.),(u+step).min(1.),
            1e-14,100,|_,_| false);
        if -least > most { (most,at) = (-least,point(faces[k],v).expect("read on the tool")) }
    }
    if !(most > 0.) { return None }
    // S2: the faces' radii run from the axis to the greatest with no gap
    let mut spans: Vec<[f64;2]> = read.iter().flat_map(|face| face.windows(2).filter_map(|w| match (w[0],w[1]) {
        (Some((a,_)),Some((b,_))) => Some([a.min(b),a.max(b)]),_ => None })).collect();
    spans.sort_by(|a,b| a[0].total_cmp(&b[0]));
    let gap = 1e-9*size;
    let mut covered = 0.;
    let mut hole = (spans.first().map_or(most,|s| s[0]) > gap).then_some([0.,spans.first().map_or(most,|s| s[0])]);
    for s in &spans { if hole.is_none() && s[0] > covered+gap { hole = Some([covered,s[0]]) } covered = f64::max(covered,s[1]) }
    if let Some([lo,hi]) = hole {
        let p = read.iter().flatten().flatten().min_by(|a,b| (a.0-hi).abs().total_cmp(&(b.0-hi).abs())).map_or(at,|x| x.1);
        return Some(Err((Condition::Once,format!("under a slide along its own axis, the tool's shadow along the slide is no whole \
            disc: none of it stands between {lo:.4} and {hi:.4} from its axis, where a ring of it in contact all round sweeps a \
            second wall within the cylinder its greatest radius's sweeps (not built)"),witness(family,reach(family,screw,p,ask),p))))
    }
    // E1: every point of the tool passes the blank's heights within the roll
    let Some(band) = ask.heights else {
        return Some(Err((Condition::Clearance,"under a slide along its own axis, how far along the slide the blank reaches is not \
            known, so the tool is not read as passing wholly through it".into(),None)))
    };
    let roll = ask.roll;
    if let Some(&(_,p)) = read.iter().flatten().flatten().find(|(_,p)| screw.passing(*p,band).is_none_or(|[a,b]| !(a > roll[0] && b < roll[1]))) {
        let end = family.pose_at(roll[1]).ok().map(|pose| pose.point(p));
        return Some(Err((Condition::Clearance,"under a slide along its own axis, the tool does not pass wholly through the blank's \
            heights along the slide over the roll, which the class reads the sweep by: there its ring of greatest radius, in contact \
            all round, sweeps its cylinder through every point of the blank within it".into(),end)))
    }
    // the ring of greatest radius, from where it was read, round the axis
    let foot: V = { let q = sub(at,centre); std::array::from_fn(|k| centre[k]+axis[k]*dot(q,axis)) };
    let round = Round {named:"the ring of the tool's greatest radius".into(),centre:foot,axis,radial:crate::space::unit(sub(at,foot)),
        radius:most,normal:[1.,0.],turn:std::f64::consts::TAU,of:None,pole:foot,about:axis,support:Some(true)};
    let n = ask.rows.max(8);
    let patch = pieces.len();
    let mut nodes: Vec<Node> = (0..=n).map(|i| {
        let u = i as f64/n as f64;
        let (position,normal) = round.at(u);
        Node {patch,u,from:if i == 0 { 0. } else { (i-1) as f64/n as f64 },position,normal,reach:reach(family,screw,position,ask)}
    }).collect();
    nodes[n].position = nodes[0].position;
    nodes[n].normal = nodes[0].normal;
    if nodes.iter().all(|m| m.reach.is_none()) {
        return Some(Err((Condition::Reach,"no point of the tool's characteristic passes through the blank in the roll".into(),None)))
    }
    // S3, of every node (the ring's tangent goes round the axis, square to the slide and to its normal)
    let least = (1..n).map(|w| {
        let (tangent,velocity) = (sub(nodes[w+1].position,nodes[w-1].position),screw.velocity(nodes[w].position));
        (dot(cross(tangent,velocity),nodes[w].normal)/(norm(tangent)*norm(velocity))).abs()
    }).fold(f64::INFINITY,f64::min);
    let mut pieces = pieces.to_vec();
    pieces.push(Piece::Round(round));
    let found = Characteristic {screw:*screw,nodes,reach:[0,n],least_factor:least,samples,whole:false,closed:true,sharp:Vec::new(),
        pieces,tool:tool.clone(),motion,tolerance};
    Some(found.simple(family).map(|()| found))
}

/// How many stations along each face a slide along the tool's own axis reads it at (`along_axis`).
const ROUND_READS: usize = 400;

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
pub fn crossing(points: &[[f64;2]]) -> Option<(usize,usize)> { crossing_of(points,false) }

/// `crossing`, of a closed polygon where `closed` says so, its last point its first: there its
/// first and last segments are neighbours too, meeting at that point.
fn crossing_of(points: &[[f64;2]],closed: bool) -> Option<(usize,usize)> {
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
            if closed && i == 0 && j+2 == n { continue }
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
        let (a,b,c) = match piece.coefficients(u,motion) { Ok(k) => k,Err(e) => return Ring::Degenerate(match piece {
            Piece::Face(_,surface) if apex(surface,u,tolerance) => APEX.into(),
            Piece::Face(_,surface) if at_axis(surface,u,tolerance) => POLE.into(),
            _ => format!("{e:?}"),
        }) };
        let amplitude = a.dhypot(b);
        let spread = (1.-(c/amplitude).dpowi(2)).max(0.).sqrt();
        match found((a,b,c)) {
            Ok(found) => Ring::Roots(found.into_iter().map(|(position,normal)| root(position,normal,spread)).collect()),
            // (a ring of no radius, where a face meets its axis, is no ring in contact all round)
            Err(envelope::Error::Degenerate) => Ring::Degenerate(if amplitude <= tolerance && c.abs() <= tolerance { match piece {
                Piece::Face(_,surface) if at_axis(surface,u,tolerance) => if apex(surface,u,tolerance) { APEX } else { POLE },
                _ => ALL_ROUND,
            }.into() } else { "has a double point on a ring".into() }),
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
        // a flat face's line across it, all in contact (#684 O)
        Piece::Flat(flat) => match &flat.line {
            Ok([a,b]) => Ring::Roots(vec![root(crate::space::lerp(*a,*b,u),flat.normal,1.)]),
            Err(why) => Ring::Degenerate(why.clone()),
        },
        // a ring in contact all round, read round it (#684 X)
        Piece::Round(round) => { let (p,n) = round.at(u); Ring::Roots(vec![root(p,n,1.)]) }
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
/// A step across a convex corner where both faces' characteristics meet its circle at one point
/// (`Sharp`) also names the corner's fan, which stands at that point (#684 V).
enum Step { Next((usize,usize,usize),isize),Sharp((usize,usize,usize),isize,usize),End(String) }

/// The root after `(piece, ring, root)` going `dir` along the piece's stations, or across the edge
/// a face shares with the next one in its profile, onto the root at the same point there — or,
/// where the characteristic jumps at a convex corner, onto the end of the corner's fan; and off
/// a fan's end onto the face beyond. A prism's side whose root reaches a ruling's end at a cap's
/// rim or a crease steps onto that fan there, partway along the side, and off the fan's far end
/// onto the cap's contact line or the other half's side (#684 AE, AG).
fn step(contacts: &SweepContacts,pieces: &[Piece],stations: &[Vec<f64>],rings: &[Vec<Ring>],(k,i,j): (usize,usize,usize),dir: isize,
    scale: f64,arms: &Arms) -> Step {
    let here = *root(rings,(k,i,j));
    // a ring of this piece in contact all round under a turn between this station and the next (#684
    // X): onto it at the end this root is at, half round to its other end, where it comes along the
    // branch the boundary takes there (`Walker::sides`); off its far end onto the boundary's branch
    // there (below)
    let ahead = usize::try_from(i as isize+dir).ok().filter(|&q| q < stations[k].len());
    if let Some((r,round)) = ahead.and_then(|q| { let (u0,u1) = (stations[k][i],stations[k][q]);
        pieces.iter().enumerate().find_map(|(r,p)| match p {
            Piece::Round(round @ Round {of:Some((of,u)),..}) if *of == k && *u != u0 && (u-u0)*(u-u1) <= 0. => Some((r,round)),
            _ => None,
        }) }) {
        let end = if distance(round.at(0.).0,here.position) <= distance(round.at(1.).0,here.position) { 0 } else { 1 };
        if round.support.is_none() { return Step::End(CROSSED.into()) }
        return match arms.get(&(r,end)) {
            None => Step::End(UNREAD_ARMS.into()),
            Some(&(_,way)) if way != -dir => Step::End(TRIMMED.into()),
            Some(_) => {
                let at = if end == 0 { 0 } else { stations[r].len().wrapping_sub(1) };
                if stations[r].get(at) != Some(&(end as f64)) { return Step::End("crosses onto a face the coarse pass did not reach".into()) }
                Step::Next((r,at,0),if end == 0 { 1 } else { -1 })
            }
        }
    }
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
    // flat face `l`'s line from whichever of its ends is this point (#684 O): a face's
    // characteristic meets the flat's edge where the flat stands least or most across the motion's
    // sections, so elsewhere the class does not cross it
    let onto_flat = |l: usize| -> Step {
        let Piece::Flat(Flat {line,..}) = &pieces[l] else { unreachable!("a flat face's piece") };
        let ends = match line { Ok(ends) => ends,Err(why) => return Step::End(why.clone()) };
        let Some(end) = ends.iter().position(|&p| distance(p,here.position) <= 1e-7*scale) else {
            return Step::End(format!("meets {} away from where it stands least or most across the motion's sections, where \
                the class does not cross it",pieces[l].named()))
        };
        let at = if end == 0 { 0 } else { stations[l].len().wrapping_sub(1) };
        if stations[l].get(at) != Some(&(end as f64)) { return Step::End("crosses onto a face the coarse pass did not reach".into()) }
        Step::Next((l,at,0),if end == 0 { 1 } else { -1 })
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
    // the side `patch` where the walk runs off a rim's or a crease's fan onto it: at the station
    // whose root is this point, or where it is lined, onto its ruling in contact all along it whose
    // end this point is (#684 AF)
    let onto_side = |patch: usize| -> Option<Step> {
        let lined = pieces.iter().any(|p| matches!(p,Piece::Ruled(Ruled {ruling:Ruling::Side {patch:q,..},lined:true,..}) if *q == patch));
        if !lined { return side_at(patch) }
        first(&mut pieces.iter().enumerate().filter(|(_,p)| p.lined() == Some(patch)).flat_map(|(l,_)| [(l,0.),(l,1.)]))
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
    // the piece that is a face of the tool (`SweepContacts::patches`)
    let face = |patch: usize| pieces.iter().position(|p| p.patch() == Some(patch));
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
        // contact line or the other half's side (onto a lined side's ruling, where it is one)
        Piece::Ruled(Ruled {ruling:Ruling::Rim {side,across,..},..}) => return if end == 0. { onto_side(*side) }
            else { match *across {
                Across::Cap(c) => first(&mut pieces.iter().enumerate().filter(|(_,p)| p.cap() == Some(c)).flat_map(|(l,_)| [(l,0.),(l,1.)])),
                Across::Half(other) => onto_side(other),
            } }.unwrap_or_else(|| Step::End("runs off its fan where no root of the face beyond takes it on".into())),
        // a lined side's ruling: off either end, at a cap or a crease, onto the fan of the rim or
        // the crease its side meets there (#684 AF)
        Piece::Line(Line {side:Some(patch),..}) => return first(&mut pieces.iter().enumerate().filter_map(|(r,p)| p.rim_of(*patch)
            .map(|end| (r,end)))).unwrap_or_else(|| Step::End(OVER_CAP.into())),
        // a cap's contact line: off either end onto the fan of the rim it meets there
        Piece::Line(Line {cap:Some(c),..}) => return first(&mut pieces.iter().enumerate()
            .filter(|(_,p)| matches!(p,Piece::Ruled(Ruled {ruling:Ruling::Rim {across:Across::Cap(d),..},..}) if d == c)).map(|(r,_)| (r,1.)))
            .unwrap_or_else(|| Step::End(OFF_CAP.into())),
        // a ring in contact all round: off either end onto the branch the boundary takes there (#684 X)
        Piece::Round(_) => return arms.get(&(k,end as usize)).map(|&(x,d)| Step::Next(x,d)).unwrap_or_else(|| Step::End(UNREAD_ARMS.into())),
        // a flat face's line: off either end onto a face beside it whose characteristic meets the
        // flat's edge there, at that face's end (#684 O)
        Piece::Flat(Flat {patch,..}) => {
            let beside: Vec<(usize,f64)> = contacts.edges().iter().filter_map(|e| (0..2).find(|&s| e.patches[s] == *patch)
                .and_then(|s| face(e.patches[1-s]).map(|other| (other,e.ends[1-s])))).collect();
            if let Some(&(other,_)) = beside.iter().find(|&&(other,_)| matches!(&pieces[other],Piece::Flat(Flat {line:Ok(ends),..})
                if ends.iter().any(|&p| distance(p,here.position) <= 1e-7*scale))) { return onto_flat(other) }
            return first(&mut beside.into_iter().filter(|&(other,_)| !matches!(pieces[other],Piece::Flat(_))))
                .unwrap_or_else(|| Step::End(OFF_FLAT.into()))
        }
        _ => {}
    }
    let jumps = || Step::End("jumps at a corner of the profile".into());
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
    // onto a flat face the motion keeps in its own plane: a sharp edge along this point's path (#684 O)
    if matches!(pieces[next],Piece::Flat(_)) { return onto_flat(next) }
    match (onto(next,e.ends[1-s]),fan) {
        // both faces' characteristics meet the corner's circle at one point: every normal of its
        // fan is in contact there, and the sweep has a sharp edge along that point's path, the
        // sheet creased there (#684 V)
        // (but where the fan holds a ring in contact all round, it is walked onto that ring, #684 X)
        (Some(Step::Next(next,d)),Some(fan)) if !pieces.iter().any(|p| matches!(p,Piece::Round(Round {of:Some((f,_)),..}) if *f == fan))
            => Step::Sharp(next,d,fan),
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
enum Piece { Face(usize,RevolvedSurface),Corner(Corner),Ruled(Ruled),Line(Line),Flat(Flat),Round(Round) }

/// A ring of a revolution in contact all round, every point of it on the boundary (#684 X): under a
/// slide along the tool's own axis, the ring of its greatest radius, whose normals all lie square
/// to the slide; under a turn about an axis the tool's own axis meets, a ring whose normals all pass
/// through `pole`, where the axes meet, at station `of.1` of piece `of.0` (`rounds`). A station on
/// it is the fraction of `turn` round it from `radial`, about `axis`; its normal there is
/// `normal`'s parts away from the axis and along it, the same all round. Under a turn it is walked
/// half round, from its highest point about the turn's axis `about` (station 0, `radial` toward the
/// turn's axis's way) to its lowest (1), both in the plane of the two axes, through which every
/// other ring's two roots pass; `support` says whether the tool lies within the sphere about `pole`
/// through it there (`Some(true)`), without it (`Some(false)`), or neither.
#[derive(Clone,Debug)]
struct Round { named: String,centre: V,axis: V,radial: V,radius: f64,normal: [f64;2],turn: f64,of: Option<(usize,f64)>,pole: V,about: V,
    support: Option<bool> }

impl Round {
    /// The ring's point at station `u`, and its normal there.
    fn at(&self,u: f64) -> (V,V) {
        let (s,c) = (self.turn*u).dsin_cos();
        let e: V = std::array::from_fn(|k| c*self.radial[k]+s*cross(self.axis,self.radial)[k]);
        (std::array::from_fn(|k| self.centre[k]+self.radius*e[k]),std::array::from_fn(|k| self.normal[0]*e[k]+self.normal[1]*self.axis[k]))
    }

    /// How a point `p` beside the end `end` of the ring leans from it, read about the pole in a
    /// half-plane through the turn's axis: the angle of its step from the end, in angle from the
    /// axis's way against in distance from the pole. The boundary goes on from the highest end (0)
    /// along the branch that leans least, from the lowest (1) along the one that leans most.
    fn lean(&self,p: V,end: usize) -> f64 {
        let q = self.at(end as f64).0;
        let from = |x: V| { let d = sub(x,self.pole); (dot(d,self.about)/norm(d)).clamp(-1.,1.).dacos() };
        (from(p)-from(q)).datan2((distance(p,self.pole)-distance(q,self.pole)).abs())
    }
}

/// Where the walk goes on from each end of each ring in contact all round under a turn (#684 X), by
/// the ring's piece and its end: the way along the piece it is a ring of (`Sides`), and the root it
/// steps onto that way and the way on (`Arms`).
type Sides = std::collections::BTreeMap<(usize,usize),isize>;
type Arms = std::collections::BTreeMap<(usize,usize),((usize,usize,usize),isize)>;

/// Why the walk ends at a ring in contact all round that the tool crosses there, lying both within
/// and without the sphere about the pole through it: the ring's zone lies within what the tool
/// sweeps, and the branch walked crosses it.
const CROSSED: &str = "meets a ring in contact all round that the tool crosses, lying both within and without the sphere \
    through it about the point where the turn's axis meets the tool's, where the characteristic runs on across the ring (not \
    built)";
/// Why it ends where it comes onto such a ring along the branch the ring trims.
const TRIMMED: &str = "comes onto a ring in contact all round along the branch the ring trims, which runs within the zone \
    the ring sweeps (not built)";
/// Why it ends where the branches through an end of such a ring cannot be read either way.
const UNREAD_ARMS: &str = "meets a ring in contact all round where the branches through its ends cannot both be read \
    beside it (not built)";

/// The rings in contact all round of `pieces` under a turn about an axis the tool's own axis meets
/// (#684 X): where a revolved face's or a convex corner's ring has the coefficients of its contact
/// equation all nought, its amplitude turning right round between two of `ROUND_READS` stations
/// along the piece and their least sought between them (`roots::brent`) as S1 `stationary` seeks
/// one, and its normals meeting the tool's axis at a point of the turn's. Each is a `Round` walked
/// half round, between its points in the plane of the two axes; `support` reads the face a
/// thousandth of the way along it either side of the ring (a corner's fan is always within its
/// tangent plane, so on the side of the sphere its normal says).
fn rounds(pieces: &[Piece],motion: Motion,screw: &Screw,tool: &super::SpatialField,size: f64) -> Vec<Round> {
    if !screw.is_turn() { return Vec::new() }
    let mut found = Vec::new();
    for (k,piece) in pieces.iter().enumerate() {
        if !matches!(piece,Piece::Face(..) | Piece::Corner(_)) { continue }
        let coefficients = |u: f64| piece.coefficients(u,motion).ok();
        let size_of = |(a,b,c): (f64,f64,f64)| a.dhypot(b).dhypot(c);
        let read: Vec<Option<(f64,f64,f64)>> = (0..=ROUND_READS).map(|i| coefficients(i as f64/ROUND_READS as f64)).collect();
        for i in 0..ROUND_READS {
            let (Some(x),Some(y)) = (read[i],read[i+1]) else { continue };
            if x.0*y.0+x.1*y.1 >= 0. { continue }
            let (u0,u1) = (i as f64/ROUND_READS as f64,(i+1) as f64/ROUND_READS as f64);
            let (least,u) = crate::roots::brent(&|u| coefficients(u).map_or(f64::INFINITY,size_of),u0,u1,1e-14,100,|_,_| false);
            if !(least <= 1e-6*size_of(x).max(size_of(y))) { continue }
            // the ring there and its outward normal's parts away from its axis and along it
            let (ring,[nr,nz]) = match piece {
                Piece::Face(_,s) => {
                    let (Ok(ring),Ok(at)) = (s.ring(u),s.at(u,0.)) else { continue };
                    let Ok(c) = envelope::contact(at,Motion::identity()) else { continue };
                    let n = outward(tool,at.position,c.normal);
                    (ring,[dot(n,ring.radial),dot(n,ring.axis)])
                }
                Piece::Corner(c) => (c.ring,c.normal(u)),
                _ => continue,
            };
            // where its normals meet its axis, on the turn's; and the way along the turn's axis
            // square to the ring's, toward its highest point (none where the axes lie along each other)
            if nr.abs() <= 1e-9 { continue }
            let pole: V = std::array::from_fn(|m| ring.center[m]-ring.radius*nz/nr*ring.axis[m]);
            if screw.radius(pole) > 1e-7*size { continue }
            let across = sub(screw.axis,scale(ring.axis,dot(screw.axis,ring.axis)));
            if norm(across) <= 1e-9 { continue }
            let mut round = Round {named:format!("a ring of {} in contact all round",piece.named()),centre:ring.center,axis:ring.axis,
                radial:crate::space::unit(across),radius:ring.radius,normal:[nr,nz],turn:std::f64::consts::PI,of:Some((k,u)),pole,
                about:screw.axis,support:None};
            let (q,n) = round.at(0.);
            let away = dot(n,sub(q,pole)) > 0.;
            round.support = match piece {
                Piece::Face(_,s) => {
                    let far = distance(q,pole);
                    let beside: Vec<f64> = [u-1e-3,u+1e-3].into_iter().filter(|w| (0. ..=1.).contains(w))
                        .filter_map(|w| s.at(w,0.).ok().map(|p| distance(p.position,pole)-far)).collect();
                    (!beside.is_empty() && beside.iter().all(|&d| (d < 0.) == away && d != 0.)).then_some(away)
                }
                _ => Some(away),
            };
            found.push(round);
        }
    }
    found
}

/// The root of `round`'s piece at its station next beyond the ring from the ring's end `end` going
/// `dir` along the piece, the one at that end, and the way on: where a branch through that end goes.
fn arm(stations: &[Vec<f64>],rings: &[Vec<Ring>],round: &Round,end: usize,dir: isize) -> Option<((usize,usize,usize),isize)> {
    let (k,u) = round.of?;
    let q = round.at(end as f64).0;
    let i = if dir > 0 { stations[k].iter().position(|&s| s > u)? } else { stations[k].iter().rposition(|&s| s < u)? };
    let Ring::Roots(roots) = &rings[k][i] else { return None };
    let j = (0..roots.len()).min_by(|&a,&b| distance(roots[a].position,q).total_cmp(&distance(roots[b].position,q)))?;
    Some(((k,i,j),dir))
}

/// A flat face the motion keeps in its own plane (#684 O): a revolution's disc square to its axis
/// under a turn about a line along that axis or a slide square to it, or a prism's plane side
/// square to a turn's axis. Its contact `n·v` is nought all over it, so every point of it is in
/// contact and what it sweeps is a region of its own plane. Each of its points is carried onto
/// the motion's section where it stands across the sections (`across`: its distance from a turn's
/// axis, its offset along the face square to a slide), so the section reads the face as the
/// stretch of a line between where it stands least and most across them, and any line on the
/// face between those two points is a characteristic of it: the straight `line` the walk takes,
/// its stations the fraction of the way from its first end to its second, its sweep a strip of the
/// face's plane. A face beside it meets it in a sharp edge along the path of the point where the
/// face's own characteristic meets the flat's edge, which is an end of `line` (the fan between
/// them is in contact at that one point all round it, so no piece: `corners`, `prism`). Where
/// there is no such line, `line` says why: a ring, its hole across it; a revolution only partway
/// round; a face a turn's axis passes through.
#[derive(Clone,Debug)]
struct Flat { patch: usize,named: String,line: Result<[V;2],String>,normal: V }

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
/// Why it ends where a flat face's line runs off it onto no face whose characteristic meets it there.
const OFF_FLAT: &str = "runs off the flat face's line where no face beyond it takes the characteristic on (a vertex of \
    the prism, or a face that does not touch there), which the class does not walk";
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
            Piece::Flat(flat) => flat.named.clone(),
            Piece::Round(round) => round.named.clone(),
        }
    }

    /// The face of the tool it is (`SweepContacts::patches`), where it is one.
    fn patch(&self) -> Option<usize> {
        match self {
            Piece::Face(k,_) | Piece::Ruled(Ruled {ruling:Ruling::Side {patch:k,..},..}) | Piece::Flat(Flat {patch:k,..}) => Some(*k),
            _ => None,
        }
    }

    /// Which cap's contact line it is (0 the prism's `start`, 1 its `end`), where it is one.
    fn cap(&self) -> Option<usize> { match self { Piece::Line(Line {cap,..}) => *cap,_ => None } }

    /// The lined side (`Ruled::lined`) whose ruling in contact all along it it is, where it is one.
    fn lined(&self) -> Option<usize> { match self { Piece::Line(Line {side,..}) => *side,_ => None } }

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
            Piece::Flat(_) => OFF_FLAT,
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
            Piece::Ruled(_) | Piece::Line(_) | Piece::Flat(_) | Piece::Round(_) => Err(envelope::Error::Degenerate),
        }
    }

    /// The point of a revolution's ring at station `u` a fraction `turn` of the way round it.
    fn around(&self,u: f64,turn: f64) -> Option<V> {
        match self {
            Piece::Face(_,s) => { let [v0,v1] = s.domain()[1]; s.at(u,v0+(v1-v0)*turn).ok().map(|p| p.position) }
            Piece::Corner(c) => Some(c.ring.at(std::f64::consts::TAU*turn).0),
            Piece::Ruled(_) | Piece::Line(_) | Piece::Flat(_) | Piece::Round(_) => None,
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
/// A side is `lined` where the motion moves each of its rulings square to its normal alike: its
/// contact is the same all along each ruling, so no ruling has one root, and it touches along the
/// whole rulings where that is nought, each walked as a `Line` (#684 AF).
#[derive(Clone,Debug)]
struct Ruled { ruling: Ruling,speed: f64,lengths: Vec<f64>,crossings: Vec<f64>,lined: bool }

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

/// How far either side of a ruling in contact all along it S1 reads the rulings beside it, in
/// stations (`rulings`): far enough that their contacts stand clear of `NOUGHT`.
const BESIDE: f64 = 1e-4;
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
        Ruled {ruling,speed,lengths:lengths.into_iter().map(|l| l/total).collect(),crossings:Vec::new(),lined:false}
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

    /// The root of the ruling at station `u`, and its normal: none where it lies past a cap, nor on
    /// a lined side (its contact is walked along its `Line`s), and refused where the whole ruling is
    /// in contact (S1 says where that is in the blank).
    fn root(&self,u: f64,motion: Motion) -> Result<Option<(V,V)>,String> {
        if self.lined { return Ok(None) }
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
/// that is nought within the profile, from one rim to another (`cap`, which cap it is on, #684 AE);
/// and on a lined side (`Ruled::lined`), a ruling in contact all along it, from cap to cap (`side`,
/// the side's patch, #684 AF). A station on it is the fraction of the way from its first end to its
/// second.
#[derive(Clone,Debug)]
struct Line { named: String,ends: [V;2],normal: V,cap: Option<usize>,side: Option<usize> }

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
/// the axis turns to no circle; and a corner beside one of `flats`, faces the motion keeps in their
/// own planes, is in contact at one point all round its fan or nowhere, a sharp edge of the sweep
/// the walk steps across (#684 O). Each corner's ring is read under `motion`, the walk's.
fn corners(contacts: &SweepContacts,tool: &super::SpatialField,motion: Motion,flats: &[usize]) -> Vec<Corner> {
    let normal = |s: &RevolvedSurface,u: f64| -> Option<(V,V)> {
        let p = s.at(u,0.).ok()?;
        Some((p.position,envelope::contact(p,Motion::identity()).ok()?.normal))
    };
    contacts.edges().iter().enumerate().filter_map(|(edge,e)| {
        let ([a,b],[ua,ub]) = (e.patches,e.ends);
        // beside a flat face the motion keeps in its own plane, the fan is in contact at one point
        // all round it, or nowhere (`Flat`): the sharp edge between the flat's sweep and the face's
        if flats.contains(&a) || flats.contains(&b) { return None }
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

/// Whether the plane face through `points` (three not on one line say it of the whole plane),
/// whose normal is `n`, is kept in its own plane by `motion`: its contact `n·v` nought at each,
/// against how fast the tool's points move.
fn kept(motion: Motion,points: &[V],n: V,speed: f64) -> bool {
    points.len() >= 3 && points.iter().all(|&p| contact(motion,(p,p,n))[0].abs() <= NOUGHT*speed)
}

/// How far across the motion's sections the point `p` of a flat face whose normal is `n` stands:
/// its distance from a turn's axis, or its offset along the face square to a slide.
fn across(screw: &Screw,n: V,p: V) -> f64 {
    if screw.is_turn() { screw.radius(p) } else { dot(p,crate::space::unit(cross(screw.axis,n))) }
}

/// Why a flat face a turn's axis passes through has no line across it.
const THROUGH: &str = "is kept in its own plane with the turn's axis through it, so it sweeps across the axis, which the \
    class does not read across";

/// Revolution `k`'s face `s` as a flat one `motion` keeps in its own plane (`Flat`), where it is
/// one: a disc, its line the diameter from where it stands least across the motion's sections to
/// where it stands most — square to a slide, or along the line from a turn's axis through its
/// centre. A ring, a revolution only partway round and a disc a turn's axis passes through have
/// no line (`Flat::line`).
fn revolved_flat(k: usize,s: &RevolvedSurface,tool: &super::SpatialField,motion: Motion,screw: &Screw,speed: f64) -> Option<Flat> {
    let (centre,axis,radii,whole) = s.flat()?;
    let [v0,v1] = s.domain()[1];
    let rim = if radii[1] >= radii[0] { 1. } else { 0. };
    let points: Vec<V> = [0.,1./3.,2./3.].iter().filter_map(|&v| s.at(rim,v0+(v1-v0)*v).ok().map(|p| p.position)).collect();
    if !kept(motion,&points,axis,speed) { return None }
    let middle = s.at(0.5,v0+(v1-v0)*0.5).ok()?.position;
    let normal = outward(tool,middle,axis);
    let radius = radii[0].max(radii[1]);
    let line = if !whole { Err("is kept in its own plane but turned only partway round, which the class does not read across".into()) }
        else if radii[0].min(radii[1]) > 1e-9*radius { Err("is a ring kept in its own plane, the line across it crossing its hole, \
            which the class does not read across".into()) }
        else {
            let w = if screw.is_turn() { let q = sub(centre,screw.origin); sub(q,scale(screw.axis,dot(q,screw.axis))) }
                else { cross(screw.axis,normal) };
            if screw.is_turn() && !(norm(w) > radius*(1.+1e-9)) { Err(THROUGH.into()) } else {
                let w = scale(crate::space::unit(w),radius);
                Ok([sub(centre,w),crate::space::add(centre,w)])
            }
        };
    Some(Flat {patch:k,named:format!("`{}`",s.name),line,normal})
}

/// A prism's plane side, its corners `q` in order round it and its outward normal `n`, as a flat
/// face the motion keeps in its own plane (`Flat`): its line from its point nearest a turn's axis
/// (on one of its edges, the axis passing outside it) to its corner farthest, or from its corner
/// least across a slide to its corner most. Every point between is on the side (it is convex),
/// and the line's distance from the axis only grows along it.
fn side_line(screw: &Screw,n: V,q: [V;4]) -> Result<[V;2],String> {
    let at = |p: V| across(screw,n,p);
    let far = q.iter().copied().fold(q[0],|a,b| if at(b) > at(a) { b } else { a });
    if !screw.is_turn() { return Ok([q.iter().copied().fold(q[0],|a,b| if at(b) < at(a) { b } else { a }),far]) }
    // the axis's foot in the side's plane, outside the side
    let foot = crate::space::add(screw.origin,scale(screw.axis,screw.height(q[0])));
    let sides: Vec<f64> = (0..4).map(|k| dot(cross(sub(q[(k+1)%4],q[k]),sub(foot,q[k])),n)).collect();
    if sides.iter().all(|&x| x >= 0.) || sides.iter().all(|&x| x <= 0.) { return Err(THROUGH.into()) }
    let nearest = (0..4).map(|k| {
        let (a,d) = (q[k],sub(q[(k+1)%4],q[k]));
        crate::space::add(a,scale(d,(dot(sub(foot,a),d)/dot(d,d)).clamp(0.,1.)))
    }).fold(q[0],|a,b| if distance(b,foot) < distance(a,foot) { b } else { a });
    Ok([nearest,far])
}

/// S1 of a flat face the motion keeps in its own plane that the walk cannot read across
/// (`Flat::line`): refused where a point of it (`at`, read across a 17 × 17 net of its parameters)
/// enters the blank; one clear of it is no part of the boundary there.
fn unread_clear(flat: &Flat,at: &dyn Fn(f64,f64) -> Option<V>,enters: &dyn Fn(V) -> Option<f64>,family: &Family) -> Result<(),Failure> {
    let Err(why) = &flat.line else { return Ok(()) };
    let entering = (0..=16).flat_map(|i| (0..=16).map(move |j| (i as f64/16.,j as f64/16.))).filter_map(|(u,v)| at(u,v))
        .find_map(|p| enters(p).map(|t| (t,p)));
    match entering {
        Some((t,p)) => Err((Condition::Regular,format!("{} {why}",flat.named),witness(family,Some(t),p))),
        None => Ok(()),
    }
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
/// that it stay clear of the blank (`admission::ends`). A plane side whose normal lies in the plane
/// of the axis and its rulings is lined (`Ruled::lined`): the motion moves each ruling square to its
/// normal alike, so each ruling's contact is the same all along it, and the side touches along the
/// whole rulings where that is nought, each a `Line` from cap to cap, walked onto off a rim's fan
/// and off onto one (#684 AF).
///
/// A plane side a turn keeps in its own plane, square to its axis, is a `Flat` walked across its
/// line (`side_line`), and no rib, rim or crease beside it is a piece: its fan is in contact at one
/// point all round, or nowhere (#684 O).
///
/// Where the motion moves every point of each ruling alike — a slide, or a screw along the
/// extrusion — the contact is the same all along each ruling, so the rulings in contact are whole:
/// the silhouette, a ruling at each root of `n·v` across each side and each corner's fan (`Line`),
/// read at `RULED_READS` parameters and refined between two that differ in sign. A piece in
/// contact all over there (a side along the slide, a corner moving along itself) is refused where
/// its path enters the blank (S1): every point of it is on the boundary, the face kept in its own
/// plane, whose line across the slide runs between vertices of the prism.
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
    // the motion moves a ruling's points alike where it turns nothing about any line across it:
    // `v(b) − v(a) = ω × (b − a)`, nought under a slide or for a ruling along the screw's axis
    let alike = screw.is_slide() || sides.iter().all(|(_,s)| [0.,0.5,1.].iter().all(|&x| match (s.at(x,0.),s.at(x,1.)) {
        (Ok(a),Ok(b)) => { let d = sub(b.position,a.position); norm(cross(screw.axis,d)) <= NOUGHT*norm(d) }
        _ => false,
    }));
    // the plane sides the motion keeps in their own planes, a turn's axis square to them (#684 O):
    // walked across their lines, and no fan beside one walked (under a slide, `silhouette` refuses
    // one in the blank)
    let flat: Vec<usize> = if alike { Vec::new() } else { rulings.iter().filter_map(|r| {
        let Ruling::Side {patch,surface,..} = r else { return None };
        let (Some((a,b,n)),Some((c,_,_))) = (r.line(0.),r.line(1.)) else { return None };
        (surface.straight() && kept(motion,&[a,b,c],n,speed)).then_some(*patch)
    }).collect() };
    for (edge,e) in contacts.edges().iter().enumerate() {
        let ([a,b],[ua,ub]) = (e.patches,e.ends);
        if flat.contains(&a) || flat.contains(&b) { continue }
        let (Some(ia),Some(ib)) = (sides.iter().position(|(k,_)| *k == a),sides.iter().position(|(k,_)| *k == b)) else { continue };
        let (Some((p0,p1,na)),Some((_,_,nb))) = (rulings[ia].line(ua),rulings[ib].line(ub)) else { continue };
        if dot(na,nb).clamp(-1.,1.).dacos() <= LEAST_TURN { continue }
        let Some((_,[pa,pb])) = contacts.corner(e) else { continue };
        if !(tool.value(crate::space::lerp(pa,pb,0.5)) < 0.) { continue }
        rulings.push(Ruling::Rib {edge,names:[sides[ia].1.name.clone(),sides[ib].1.name.clone()],ends:[p0,p1],normals:[na,nb]});
    }
    if alike { return silhouette(contacts,&rulings,motion,speed,enters) }
    // a lined side (#684 AF): a plane side each of whose rulings the motion moves square to its
    // normal alike, `n·(ω × e)` nought, so its contact is the same all along each ruling and nought
    // along whole rulings, each a `Line` from cap to cap; a side in contact all over is left to S1
    let mut lines: Vec<Line> = Vec::new();
    let mut lined: Vec<usize> = Vec::new();
    for ruling in &rulings {
        let Ruling::Side {patch,..} = *ruling else { continue };
        let at = |x: f64| ruling.line(x).map(|line| contact(motion,line));
        let read: Vec<[f64;2]> = (0..=RULED_READS).filter_map(|i| at(i as f64/RULED_READS as f64)).collect();
        if !read.iter().all(|c| (c[0]-c[1]).abs() <= NOUGHT*speed) || read.iter().all(|c| c[0].abs() <= NOUGHT*speed) { continue }
        lined.push(patch);
        for x in noughts(&|x| at(x).map(|c| c[0])) {
            let Some((a,b,n)) = ruling.line(x) else { continue };
            lines.push(Line {named:ruling.named(),ends:[a,b],normal:n,cap:None,side:Some(patch)});
        }
    }
    // a plane side's outward normal, the same all along it
    let normal = |patch: usize| rulings.iter().find(|r| matches!(r,Ruling::Side {patch:p,..} if *p == patch)).and_then(|r| r.line(0.5))
        .map(|(_,_,n)| n);
    let mut rims: Vec<Ruling> = Vec::new();
    // each crease where a drafted prism's halves meet at its parting plane, the half below's edge
    // at its `v = 1`, its fan turned from that half's normal to the half above's (#684 AG)
    for &[below,above] in contacts.creases() {
        let Some(&(_,s)) = sides.iter().find(|(k,_)| *k == below).filter(|_| !flat.contains(&below) && !flat.contains(&above)) else { continue };
        let (Some(na),Some(nb)) = (normal(below),normal(above)) else { continue };
        if !s.straight() || dot(na,nb).clamp(-1.,1.).dacos() <= LEAST_TURN { continue }
        let (Ok(a),Ok(b)) = (s.at(0.,1.),s.at(1.,1.)) else { continue };
        rims.push(Ruling::Rim {side:below,end:1,across:Across::Half(above),names:[s.name.clone(),s.name.clone()],
            ends:[a.position,b.position],normals:[na,nb]});
    }
    // a lone prism's caps (#684 AE): each plane side's rim on each, the side's edge at its `v` 0 or
    // 1, its fan turned from the side's normal to the cap's; and each cap's contact line
    if caps { for surface in contacts.patches() {
        let (super::ToolSurface::Cap(cap),Some(c)) = (surface,surface.cap()) else { continue };
        let Ok(p) = surface.at(0.5,0.5) else { continue };
        // `du × dv` is a cap's outward normal (`CapSurface`)
        let n = crate::space::unit(cross(p.du,p.dv));
        for &(patch,s) in sides.iter().filter(|(k,_)| !flat.contains(k)) {
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
    let mut pieces: Vec<Piece> = Vec::with_capacity(rulings.len());
    for r in rulings { pieces.push(match r {
        // a flat side: its line from its point nearest the turn's axis to its corner farthest
        Ruling::Side {patch,ref surface,..} if flat.contains(&patch) => {
            let Some((_,_,n)) = r.line(0.5) else { continue };
            let corners = [(0.,0.),(1.,0.),(1.,1.),(0.,1.)].map(|(x,y)| surface.at(x,y).map(|p| p.position));
            let line = match corners { [Ok(a),Ok(b),Ok(c),Ok(d)] => side_line(screw,n,[a,b,c,d]),_ => Err("is read off its side".into()) };
            let flat = Flat {patch,named:r.named(),line,normal:n};
            unread_clear(&flat,&|x,y| surface.at(x,y).ok().map(|p| p.position),enters,contacts.motion())?;
            Piece::Flat(flat)
        }
        Ruling::Side {patch,..} if lined.contains(&patch) => Piece::Ruled(Ruled {lined:true,..Ruled::new(r,motion,speed)}),
        Ruling::Side {patch,..} => { let ends = ends(patch); Piece::Ruled(Ruled::new(r,motion,speed).crossing(motion,&ends)) }
        r => Piece::Ruled(Ruled::new(r,motion,speed)),
    }) }
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
                    it in its own plane, which the class reads of a prism's side only under a turn (under a slide its line across \
                    the slide runs between vertices of the prism, where a cap meets it)",ruling.named()),
                    witness(contacts.motion(),Some(t),p)))
            }
            continue
        }
        for x in noughts(&at) {
            let Some((a,b,n)) = ruling.line(x) else { continue };
            let size = distance(a,b);
            if lines.iter().any(|l| distance(l.ends[0],a) <= 1e-9*size && distance(l.ends[1],b) <= 1e-9*size) { continue }
            lines.push(Line {named:ruling.named(),ends:[a,b],normal:n,cap:None,side:None});
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
    }).map(|w| Line {named:format!("`{}`",surface.name()),ends:[w[0],w[1]],normal:n,cap:Some(c),side:None}).collect()
}

/// S1 of a prism read ruling by ruling (#684 M): no ruling in contact all along it where its path
/// enters the blank, but a lined side's (`Ruled::lined`), which the walk runs along (#684 AF).
/// Elsewhere the characteristic would run along the ruling off the walked stretch, which crosses
/// each ruling once: where the rulings either side have roots on the tool, as at a corner whose
/// fan holds a normal along a turn's axis, the walk crosses the ruling at one point and the
/// characteristic branches there (a plane side every ruling of which is in contact is a face the
/// motion keeps in its own plane, a `Flat`, #684 O, and no `Ruled` piece). A ruling's contact is
/// nought at both its ends there (`Ruled::contact`), so between two of `rows` stations across each
/// piece where
/// both ends' contacts change sign (or are nought), the least of the two together is sought
/// (`roots::brent`), and a ruling found in contact all along with a point whose path enters the
/// blank is refused. Asked before the walk, whose ring there is degenerate, so the row is named.
fn rulings(pieces: &[Piece],rows: usize,motion: Motion,enters: &dyn Fn(V) -> Option<f64>,family: &Family) -> Result<(),Failure> {
    let station = |i: usize| i as f64/rows as f64;
    for piece in pieces {
        let Piece::Ruled(ruled) = piece else { continue };
        if ruled.lined { continue }
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
            let entering = candidates.into_iter().filter_map(|u| ruled.line(u).map(|line| (u,line)))
                .flat_map(|(u,(a,b,_))| (0..=64).map(move |j| (u,crate::space::lerp(a,b,j as f64/64.))))
                .find_map(|(u,p)| enters(p).map(|t| (u,t,p)));
            let Some((u,t,p)) = entering else { continue };
            // the rulings just either side of it: a root on the tool on either is the walk crossing it
            let crossed = [u-BESIDE,u+BESIDE].iter().any(|&w| (0. ..=1.).contains(&w) && matches!(ruled.root(w,motion),Ok(Some(_))));
            let why = if whole { format!("{} is in contact all over: the motion keeps it in its own plane, which the class does not \
                sweep",piece.named()) } else if crossed { format!("a ruling of {} is in contact all along it, where the characteristic \
                branches: the walk crosses the ruling at one point, and the ruling's own sweep runs off it there, which the class does \
                not sweep",piece.named()) } else { format!("a ruling of {} is in contact all along it, where the characteristic runs \
                along the ruling, which the class walks only on a plane side whose every ruling's contact is the same at both its \
                ends",piece.named()) };
            return Err((Condition::Regular,why,witness(family,Some(t),p)))
        }
    }
    Ok(())
}
