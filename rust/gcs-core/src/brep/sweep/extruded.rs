//! A prism's sheet under a motion that carries its profile's plane within itself, built from the
//! planar envelope and not traced (issue #70, part 1): every station sections the prism in its
//! profile and the motion moves every station alike, so the contacts at every station are one
//! curve — the profile's envelope in the plane (`generate`) — and the sheet is that curve extruded
//! along the prism. Each edge of the profile cuts as a line or an arc, and each convex corner as a
//! point over the fan between its sides' normals; the curve is their envelopes in walk order, an
//! edge's over the rolls its cut lies on it, a corner's from where the edge before reaches it to
//! where the edge after leaves it. Read exactly (`generate::cut_at`: `C` and `C'` of a point, line
//! or arc are exact), interpolated by cubic Hermite spans halved until every span stays within the
//! fit's bars of the envelope at its quarter points, and extruded linearly over the prism's length.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::cutter::Cutter;
use super::sheet::{Fitted,SweptCut};
use super::Say;
use crate::brep::build::Seg;
use crate::brep::geom::{Curve,Frame};
use crate::brep::nurbs::Net;
use crate::generate::{self,Generated,Tool};
use crate::motion::PlanarStep;
use crate::solid::contact_trace::{Inside,Sheet};
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use std::f64::consts::{PI,TAU};

type V = [f64;3];
type P = [f64;2];

/// How far the section curve may pass from the envelope between its nodes without a stated
/// tolerance (mm), and how far its tangent may turn from the envelope's (degrees). The curve is
/// exact where it is read and its spans converge at the fourth power of their length, so the
/// default is fine and cheap, not gross.
const FIT: f64 = 1e-4;
const TURN: f64 = 0.05;
/// Rolls a piece's cut is sought at across the declared roll.
const SCAN: usize = 720;
/// Spans a piece starts with, and the most the whole curve may be halved into.
const FIRST_SPANS: usize = 4;
const MOST_SPANS: usize = 8192;
/// Rolls of a piece's cut, and stations along the prism, the blank is asked about; the sheet's
/// contacts stand at the same stations.
const SAMPLES: usize = 64;
const STATIONS: usize = 9;

/// One piece of the profile's loop as it cuts: an edge, a line or an arc, with the rolls its cut
/// lies on it (`run`), or the corner after it, a point.
struct Piece { generated: Generated,outer: Vec<f64>,kind: Kind,run: Option<Run> }

#[derive(Clone,Copy)]
enum Kind {
    Line,
    /// The arc's middle angle, the sign the walk turns it, and its sweep (radians).
    Arc { middle: f64,sense: f64,sweep: f64 },
    /// A corner, convex where the walk turns left.
    Corner { at: P,convex: bool },
}

impl Piece {
    fn new(tool: Tool,columns: &[f64],side: f64,program: &[PlanarStep],kind: Kind) -> Piece {
        let (generated,outer) = generate::planar(tool,columns,side,program);
        Piece {generated,outer,kind,run:None}
    }
    /// An edge, with the rolls within `roll` its cut lies on it.
    fn edge(tool: Tool,columns: &[f64],side: f64,program: &[PlanarStep],kind: Kind,roll: [f64;2]) -> Result<Piece,String> {
        let mut piece = Piece::new(tool,columns,side,program,kind);
        piece.run = run(&piece,roll)?;
        Ok(piece)
    }
    /// The cut at roll `t` (degrees).
    fn cut(&self,t: f64) -> Option<generate::Cut> {
        let mut outer = self.outer.clone();
        outer[0] = t;
        generate::cut_at(&self.generated.flat,&outer,t).filter(|c| c.c.iter().chain(&c.dc).all(|x| x.is_finite()))
    }
    /// Where along the edge the cut at roll `t` stands, as a fraction of the walk over it.
    fn fraction(&self,t: f64) -> Option<f64> {
        let s = self.cut(t)?.s;
        match self.kind {
            Kind::Line => Some(s),
            Kind::Arc {middle,sense,sweep} => Some(((sense*(s-middle)+PI).rem_euclid(TAU)-PI+sweep/2.)/sweep),
            Kind::Corner {..} => None,
        }
    }
}

/// Where an edge's cut runs within the declared roll: the rolls at its walk's start and end, each
/// with whether the cut reaches that end of the edge there (else the roll's limit stops it).
#[derive(Clone,Copy,Debug)]
struct Run { start: (f64,bool),end: (f64,bool) }

/// The rolls over which an edge's cut lies on it, within `[a, b]` (degrees): none where it never
/// does, refused where it does over two stretches.
fn run(piece: &Piece,[a,b]: [f64;2]) -> Result<Option<Run>,String> {
    let on = |t: f64| piece.fraction(t).is_some_and(|f| (0. ..=1.).contains(&f));
    let rolls: Vec<f64> = (0..=SCAN).map(|k| a+(b-a)*k as f64/SCAN as f64).collect();
    let marks: Vec<bool> = rolls.iter().map(|&t| on(t)).collect();
    let starts: Vec<usize> = (0..marks.len()).filter(|&k| marks[k] && (k == 0 || !marks[k-1])).collect();
    let [first] = starts[..] else {
        if starts.is_empty() { return Ok(None) }
        return Err("an edge's cut lies on it over two stretches of the roll".into())
    };
    let last = (first..marks.len()).take_while(|&k| marks[k]).last().expect("a mark");
    // the boundary between a roll on the edge and one off it, from the side on it, to the last bit
    let edge = |inner: f64,off: f64| crate::roots::bisect(inner,off,|x,y| 0.5*(x+y),
        |x,y| { let m = 0.5*(x+y); m != x && m != y },|t| on(t)).0;
    let lo = if first == 0 { (a,false) } else { (edge(rolls[first],rolls[first-1]),true) };
    let hi = if last == SCAN { (b,false) } else { (edge(rolls[last],rolls[last+1]),true) };
    // which end of the edge each boundary is: the fraction nearer 0 is the walk's start
    let f = |t: f64| piece.fraction(t).unwrap_or(0.5);
    Ok(Some(if f(lo.0) <= f(hi.0) { Run {start:lo,end:hi} } else { Run {start:hi,end:lo} }))
}

/// A node of the section curve: the roll, the point and `C'` per degree there.
#[derive(Clone,Copy)]
struct Node { t: f64,c: P,dc: P }

/// A Hermite span's point at `w` of its length in roll.
fn hermite(a: &Node,b: &Node,w: f64) -> (P,P) {
    let dt = b.t-a.t;
    let (h00,h10,h01,h11) = (2.*w*w*w-3.*w*w+1.,w*w*w-2.*w*w+w,-2.*w*w*w+3.*w*w,w*w*w-w*w);
    let (d00,d10,d01,d11) = (6.*w*w-6.*w,3.*w*w-4.*w+1.,-6.*w*w+6.*w,3.*w*w-2.*w);
    let p = std::array::from_fn(|k| h00*a.c[k]+h10*dt*a.dc[k]+h01*b.c[k]+h11*dt*b.dc[k]);
    let d = std::array::from_fn(|k| (d00*a.c[k]+d10*dt*a.dc[k]+d01*b.c[k]+d11*dt*b.dc[k])/dt);
    (p,d)
}

fn turn_between(a: P,b: P) -> f64 {
    let (la,lb) = (a[0].dhypot(a[1]),b[0].dhypot(b[1]));
    if !(la > 0. && lb > 0.) { return 0. }
    ((a[0]*b[1]-a[1]*b[0]).datan2(a[0]*b[0]+a[1]*b[1])).abs().to_degrees()
}

/// The sheet of `cut`, built from its profile's planar envelope, when its cutter is a prism and its
/// motion carries the profile's plane within itself; none otherwise (it is traced). `inside` asks
/// the blank (mm); `tolerance` sets the fit's bar (half of it), else `FIT`.
pub fn planar_sheet(cut: &SweptCut,inside: Inside,tolerance: Option<Tolerance>,say: &Say) -> Option<Result<Fitted,ExportRefusal>> {
    let cutter = Cutter::read(&cut.recipe).ok()?;
    let (walks,extent) = cutter.prism_profile()?;
    let frame = Frame::about(cutter.origin,cutter.axis);
    let program = cut.family.in_frame(&frame,cut.scale)?;
    // which corners are convex is the cutter's own reading, as admission's
    let convex: Vec<Vec<bool>> = match cutter.profile(0.) {
        Ok(loops) => loops.iter().map(|l| l.convex()).collect(),
        Err(e) => return Some(Err(ExportRefusal::at(Stage::Reach,e))),
    };
    Some(build(cut,walks,&convex,extent,frame,&program,inside,tolerance,say))
}

#[allow(clippy::too_many_arguments)]
fn build(cut: &SweptCut,walks: &[Vec<Seg>],convex: &[Vec<bool>],extent: [f64;2],frame: Frame,program: &[PlanarStep],inside: Inside,
    tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,ExportRefusal> {
    let name = &cut.name;
    let refused = |stage: Stage,why: String| ExportRefusal::at(stage,format!("`{name}`: {why}"));
    let started = crate::clock::Instant::now();
    let local = |p: V| { let l = frame.local(p); [l[0],l[1]] };
    let lift = |p: P,h: f64| frame.at([p[0],p[1],h]);
    let roll = cut.limits.map(f64::to_degrees);
    let stations: Vec<f64> = (0..STATIONS).map(|j| extent[0]+(extent[1]-extent[0])*j as f64/(STATIONS-1) as f64).collect();
    let in_blank = |points: &[P]| -> Result<bool,String> {
        let lifted: Vec<V> = points.iter().flat_map(|&p| stations.iter().map(move |&h| lift(p,h))).collect();
        Ok(inside(&lifted)?.into_iter().any(|b| b))
    };
    // each piece's stretch of the roll in walk order — an edge's where its cut lies on it, a
    // corner's from where the edge before reaches it to where the edge after leaves it — or why
    // it has none
    let stretch = |pieces: &[Piece],i: usize| -> Result<[f64;2],String> {
        let n = pieces.len();
        match pieces[i].kind {
            Kind::Corner {at,convex} => {
                let (Some(before),Some(after)) = (pieces[i-1].run,pieces[(i+1)%n].run) else {
                    return Err(format!("a side of the corner at {at:?} does not cut within the declared roll"))
                };
                if !(before.end.1 && after.start.1) {
                    return Err(format!("the cut of a side of the corner at {at:?} leaves the declared roll before it reaches the corner"))
                }
                if !convex {
                    return Err(format!("the profile's corner at {at:?} is concave: its sides' cuts cross there, which one sheet does \
                        not follow"))
                }
                Ok([before.end.0,after.start.0])
            }
            _ => pieces[i].run.map(|r| [r.start.0,r.end.0])
                .ok_or_else(|| format!("edge {} of the profile does not cut within the declared roll",i/2)),
        }
    };
    // each loop's pieces, edge k then the corner after it
    let mut chosen: Option<(Vec<Piece>,Vec<bool>)> = None;
    for (walk,convex) in walks.iter().zip(convex) {
        let mut pieces = Vec::with_capacity(2*walk.len());
        for (k,seg) in walk.iter().enumerate() {
            let (start,end) = (local(seg.start()),local(seg.end()));
            match &seg.curve {
                Curve::Line {..} => pieces.push(Piece::edge(Tool::Line,&[start[0],start[1],end[0],end[1]],1.,program,Kind::Line,roll)
                    .at(Stage::Reach)?),
                Curve::Circle(circle,r) => {
                    let c = local(circle.o);
                    let mid = local(seg.curve.point(0.5*(seg.t[0]+seg.t[1])));
                    let angle = |p: P| (p[1]-c[1]).datan2(p[0]-c[0]);
                    let sense = ((start[0]-c[0])*(mid[1]-c[1])-(start[1]-c[1])*(mid[0]-c[0])).signum();
                    let kind = Kind::Arc {middle:angle(mid),sense,sweep:(seg.t[1]-seg.t[0]).abs()};
                    // a circle cuts on the side of its instant centre and the other: the arc's is the
                    // one whose cut lies on it
                    let mut sides: Vec<Piece> = [1.,-1.].into_iter()
                        .map(|side| Piece::edge(Tool::Arc,&[c[0],c[1],0.,0.,0.,0.,*r],side,program,kind,roll))
                        .collect::<Result<_,_>>().at(Stage::Reach)?;
                    if sides.iter().all(|p| p.run.is_some()) {
                        return Err(refused(Stage::Reach,"an arc of the profile cuts on both sides of the instant centre, which one \
                            sheet does not follow".into()))
                    }
                    let side = if sides[0].run.is_none() && sides[1].run.is_some() { 1 } else { 0 };
                    pieces.push(sides.swap_remove(side));
                }
                _ => return Err(refused(Stage::Reach,"the profile has an edge neither a line nor an arc".into())),
            }
            pieces.push(Piece::new(Tool::Point,&end,1.,program,Kind::Corner {at:end,convex:convex[k]}));
        }
        let mut reaching = vec![false;pieces.len()];
        for i in 0..pieces.len() {
            let Ok([a,b]) = stretch(&pieces,i) else { continue };
            let points: Vec<P> = (0..=SAMPLES).filter_map(|j| pieces[i].cut(a+(b-a)*j as f64/SAMPLES as f64).map(|c| c.c)).collect();
            reaching[i] = in_blank(&points).at(Stage::Reach)?;
        }
        if reaching.iter().any(|r| *r) {
            if chosen.is_some() { return Err(refused(Stage::Reach,"two loops of the profile cut the blank".into())) }
            chosen = Some((pieces,reaching));
        }
    }
    let Some((pieces,reaching)) = chosen else {
        return Err(refused(Stage::Reach,"no cut of the profile reaches the blank within the declared roll".into()))
    };
    let n = pieces.len();
    if reaching.iter().all(|r| *r) {
        return Err(refused(Stage::Reach,"every piece of the profile cuts the blank, so its sheet would close".into()))
    }
    // the run of pieces cutting the blank: after the longest stretch of the loop that does not, from
    // an edge to an edge
    let gap = (0..n).filter(|&i| !reaching[i] && reaching[(i+n-1)%n])
        .map(|i| (i,(0..n).take_while(|k| !reaching[(i+k)%n]).count())).max_by_key(|&(_,length)| length).expect("a gap");
    let mut first = (gap.0+gap.1)%n;
    let mut last = (gap.0+n-1)%n;
    if first % 2 == 1 { first = (first+n-1)%n; }
    if last % 2 == 1 { last = (last+1)%n; }
    let order: Vec<usize> = (0..n).map(|k| (first+k)%n).take_while(|&i| i != (last+1)%n).collect();
    if order.len() == n {
        return Err(refused(Stage::Reach,"the profile's cut leaves the blank only along one edge, so its sheet would close".into()))
    }
    let stretches: Vec<(usize,[f64;2])> = order.iter().map(|&i| stretch(&pieces,i).map(|s| (i,s)))
        .collect::<Result<_,_>>().map_err(|why| refused(Stage::Reach,why))?;
    (say.stage)(&format!("`{name}`: the profile cuts the blank over {} pieces of its loop in the plane its motion keeps ({:?})",
        order.len(),started.elapsed()));
    (say.mark)(Stage::Reach);
    // the curve: each stretch's spans halved until each keeps to the envelope at its quarter points;
    // a span kept carries its exact middle, which the sheet withholds
    let bar = tolerance.map_or(FIT,|t| t.fit().min(FIT));
    let read = |i: usize,t: f64| -> Result<Node,ExportRefusal> {
        let c = pieces[i].cut(t).ok_or_else(|| refused(Stage::Sheet,format!("the envelope has no cut at roll {t:.6} degrees")))?;
        Ok(Node {t,c:c.c,dc:c.dc})
    };
    struct Span { a: Node,b: Node,middle: Option<Node> }
    let mut curve: Vec<(usize,Vec<Span>)> = Vec::new();
    let (mut error,mut turned,mut spans) = (0_f64,0_f64,0);
    for &(i,[a,b]) in &stretches {
        if !((b-a).abs() > 1e-12*(1.+a.abs())) { continue }
        let nodes: Vec<Node> = (0..=FIRST_SPANS).map(|j| read(i,a+(b-a)*j as f64/FIRST_SPANS as f64)).collect::<Result<_,_>>()?;
        let mut open: Vec<Span> = nodes.windows(2).rev().map(|w| Span {a:w[0],b:w[1],middle:None}).collect();
        let mut kept: Vec<Span> = Vec::new();
        // in walk order: the next span off the top, kept or halved back onto it
        while let Some(span) = open.pop() {
            let at = |q: f64| read(i,span.a.t+(span.b.t-span.a.t)*q);
            let quarters = [at(0.25)?,at(0.5)?,at(0.75)?];
            let (mut far,mut turn) = (0_f64,0_f64);
            for (q,exact) in [0.25,0.5,0.75].into_iter().zip(&quarters) {
                let (p,d) = hermite(&span.a,&span.b,q);
                (far,turn) = (far.max((p[0]-exact.c[0]).dhypot(p[1]-exact.c[1])),turn.max(turn_between(d,exact.dc)));
            }
            if far > bar || turn > TURN {
                open.push(Span {a:quarters[1],b:span.b,middle:None});
                open.push(Span {a:span.a,b:quarters[1],middle:None});
            } else {
                (error,turned) = (error.max(far),turned.max(turn));
                kept.push(Span {middle:Some(quarters[1]),..span});
            }
            if spans+kept.len()+open.len() > MOST_SPANS {
                return Err(refused(Stage::Withheld,format!("the envelope's curve needs more than {MOST_SPANS} spans to keep within \
                    {bar:.2e} mm and {TURN} degrees of it")))
            }
        }
        spans += kept.len();
        curve.push((i,kept));
    }
    if curve.is_empty() {
        return Err(refused(Stage::Sheet,"the envelope's curve runs over no roll".into()))
    }
    // its ends leave the blank, or the sheet would end inside it
    let ends = [curve[0].1[0].a.c,curve[curve.len()-1].1.last().expect("a span").b.c];
    if in_blank(&ends).at(Stage::Sheet)? {
        return Err(refused(Stage::Sheet,format!("the envelope's curve ends inside the blank (at {:?} or {:?})",ends[0],ends[1])))
    }
    (say.stage)(&format!("`{name}`: the envelope's curve in {spans} spans, within {error:.2e} mm and {turned:.3} degrees of it at their \
        quarter points ({:?})",started.elapsed()));
    (say.mark)(Stage::Sheet);
    // the Hermite spans as one cubic B-spline, a piece's parameter running with its roll, extruded
    // linearly over the prism: knots of multiplicity three, the poles each span's Bézier's
    let mut knots = vec![0.;4];
    let mut poles: Vec<P> = vec![ends[0]];
    let mut u = 0.;
    for (_,kept) in &curve {
        let chord: f64 = kept.iter().map(|s| (s.b.c[0]-s.a.c[0]).dhypot(s.b.c[1]-s.a.c[1])).sum();
        let rate = chord.max(1e-12)/(kept[kept.len()-1].b.t-kept[0].a.t).abs();
        for Span {a,b,..} in kept {
            let dt = b.t-a.t;
            poles.push(std::array::from_fn(|k| a.c[k]+a.dc[k]*dt/3.));
            poles.push(std::array::from_fn(|k| b.c[k]-b.dc[k]*dt/3.));
            poles.push(b.c);
            u += dt.abs()*rate;
            knots.extend([u;3]);
        }
    }
    knots.push(u);
    let net = Net {du:3,dv:1,uknots:knots,vknots:vec![extent[0],extent[0],extent[1],extent[1]],
        poles:poles.iter().map(|&p| vec![lift(p,extent[0]),lift(p,extent[1])]).collect(),weights:None};
    let face = crate::brep::build::sheet(net).at(Stage::Fit)?;
    (say.mark)(Stage::Fit);
    // the contacts the sheet carries, for what reads it: its nodes at each station, and the
    // envelope at each span's middle withheld; a stretch's first node is the one before's last
    let normal = |d: P,forward: bool| { let l = d[0].dhypot(d[1]).max(1e-300)*if forward { 1. } else { -1. }; frame.dir([d[1]/l,-d[0]/l,0.]) };
    let mut rows: Vec<(Node,V)> = Vec::new();
    let mut middles: Vec<(Node,V,usize)> = Vec::new();
    for (_,kept) in &curve {
        let forward = kept[kept.len()-1].b.t > kept[0].a.t;
        rows.pop();
        rows.push((kept[0].a,normal(kept[0].a.dc,forward)));
        for s in kept {
            let middle = s.middle.expect("a kept span's middle");
            middles.push((middle,normal(middle.dc,forward),rows.len()-1));
            rows.push((s.b,normal(s.b.dc,forward)));
        }
    }
    let mut sheet = Sheet {points:Vec::new(),normals:Vec::new(),times:Vec::new(),rows:rows.len(),columns:STATIONS,withheld:Vec::new(),
        withheld_normals:Vec::new(),sites:Vec::new()};
    for (node,n) in &rows {
        for &h in &stations {
            sheet.points.push(lift(node.c,h));
            sheet.normals.push(*n);
            sheet.times.push(node.t.to_radians());
        }
    }
    for (node,n,r) in &middles {
        for (c,&h) in stations.iter().enumerate() {
            sheet.withheld.push(lift(node.c,h));
            sheet.withheld_normals.push(*n);
            sheet.sites.push([2*r+1,2*c]);
        }
    }
    (say.stage)(&format!("`{name}`: the sheet extruded from the envelope's curve over {:.2} mm of the prism ({:?})",extent[1]-extent[0],
        started.elapsed()));
    (say.mark)(Stage::Withheld);
    Ok(Fitted {face,sheet,error})
}
