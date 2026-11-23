//! **A slide through a blank, by its shadow**: a tool slid without turning from wholly before a
//! blank to wholly past it cuts from it the tool's **shadow** along the slide — the points whose
//! line along the slide meets the tool — and nothing else. Let `h` be the height along the slide,
//! the tool `[l, u]` high along each such line where it starts and the slide `L` long: a point of
//! the blank on a line meeting the tool stands above `u` (the tool starts wholly before it) and below
//! `l + L` (it ends wholly past), so the tool slid holds it at some instant; on a line missing the
//! tool nothing holds it. So the cut is the blank less a prism over the shadow, built in the plane
//! square to the slide, with no surface met in space but the prism's with the blank:
//!
//! 1. **Candidates** (`Plan`): every curve of the plane the shadow's boundary may run along, each
//!    the image of something on the tool carried along the slide onto the plane — each face's
//!    contact (its points whose normal is square to the slide, `contacts::face_contact`, fitted as
//!    `contacts::piece_curve` fits it, the parts buried in the tool left out), and each sharp edge,
//!    and each edge of a face the slide keeps in its own surface (in contact all over: a plane or a
//!    cylinder along the slide), whole. Carried along a line is affine, so a B-spline carried is the
//!    B-spline of its poles carried, exactly; a line's image is a line, a circle's the rational arc's
//!    (`nurbs::arc`) carried.
//! 2. **Arranged** and **kept by membership** as a turn's shadow is (`shadow::arranged`): a piece of
//!    the arrangement is on the boundary where the shadow lies on one side of it alone, read by
//!    whether the line along the slide through the point meets the tool, meshed
//!    (`member::Solid::line`).
//! 3. **Judged** in the plane: at points of a grid over the shadow's box clear of its boundary, the
//!    loops' parity against the membership; a disagreement refuses the road.
//! 4. **Extruded and cut**: each region and its holes made a profile in the plane (each kept piece
//!    its candidate's stretch, its ends held to the vertices the arrangement placed), extruded along
//!    the slide (`build::prism`) past the blank's heights over the shadow either way — its walls
//!    extrusions of the images — and the prism cut from the blank by the Boolean.
//!
//! `cut` takes a slide only, and a tool wholly before the blank where it starts and wholly past it
//! where it ends, each read on boxes: the tool's, and the blank's box within the prism standing on
//! the tool's box seen along the slide (`span_within`). Anything else is left to the other roads
//! (`None`), as is a tool with an edge of a curve it does not carry (no line, circle or B-spline),
//! a contact it does not fit, or a shadow it does not arrange or judge.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::build::{Profile,ProfileEdge,prism};
use crate::geom::{Curve,Frame};
use crate::nurbs::BSpline;
use crate::query::Located;
use crate::topo::{Brep,EdgeCurve};
use crate::trajectory::{Screw,Trajectory};
use crate::plane2::dist;
use crate::util::space::{distance,dot,norm,sub,unit};
use super::closed::Unbuilt;
use super::contacts::{Contact,CELLS};
use super::member::Solid;
use super::shadow::{Arranged,Candidate,arranged};
use std::sync::Arc;

type V = [f64;3];
type P = [f64;2];

/// How many samples a candidate is read at for its crossings, at least, and how many to each span
/// of a B-spline.
const SAMPLES: usize = 160;
const PER_SPAN: usize = 2;
/// The grid the shadow is judged on, each way.
const JUDGED: usize = 24;

/// **A candidate of the shadow's boundary**, in the plane square to the slide, in that plane's
/// coordinates (`x`, `y`; its third coordinate nought).
enum Plan {
    /// A B-spline: a contact's image, an edge's.
    Spline(BSpline),
    /// The straight stretch from `a` to `b` (its parameter from 0 to 1): a line's image.
    Segment { a: P,b: P },
}

impl Candidate for Plan {
    fn span(&self) -> [f64;2] {
        match self { Plan::Spline(s) => s.domain(),Plan::Segment {..} => [0.,1.] }
    }
    fn at(&self,t: f64) -> (P,P) {
        match self {
            Plan::Spline(s) => { let (p,d) = s.d1(t); ([p[0],p[1]],[d[0],d[1]]) }
            &Plan::Segment {a,b} => ([a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])],[b[0]-a[0],b[1]-a[1]]),
        }
    }
    fn samples(&self) -> Vec<(f64,P)> {
        let n = match self { Plan::Spline(s) => SAMPLES.max(PER_SPAN*s.poles.len()),Plan::Segment {..} => 1 };
        let [lo,hi] = self.span();
        (0..=n).map(|k| { let t = lo+(hi-lo)*k as f64/n as f64; (t,self.point(t)) }).collect()
    }
}

impl Plan {
    /// A B-spline in space carried along the slide onto the plane `g` square to it, in the plane's
    /// coordinates: its poles carried, exactly.
    fn carried(s: &BSpline,g: &Frame) -> Plan {
        Plan::Spline(BSpline {poles:s.poles.iter().map(|&p| { let l = g.local(p); [l[0],l[1],0.] }).collect(),..s.clone()})
    }
    /// Its stretch over `[t0, t1]` from the vertex `a` to the vertex `b`, as an edge of a profile in
    /// the plane `g`: a B-spline's own stretch (`BSpline::segment`), its end poles set on the two
    /// vertices — where it crosses another, Newton placed it within a rounding; where it runs into
    /// another's end, within the two fits' tolerance — so the loop closes exactly.
    fn edge(&self,[t0,t1]: [f64;2],a: P,b: P,g: &Frame) -> ProfileEdge {
        let at = |p: P| g.at([p[0],p[1],0.]);
        match self {
            Plan::Segment {..} => ProfileEdge::Line {a:at(a),b:at(b)},
            Plan::Spline(s) => {
                let (lo,hi,first,last) = if t0 <= t1 { (t0,t1,a,b) } else { (t1,t0,b,a) };
                let mut piece = s.segment([lo,hi]);
                let n = piece.poles.len();
                piece.poles[0] = [first[0],first[1],0.];
                piece.poles[n-1] = [last[0],last[1],0.];
                piece.poles = piece.poles.iter().map(|&p| at([p[0],p[1]])).collect();
                ProfileEdge::Spline(Arc::new(piece))
            }
        }
    }
}

/// **The span of heights along `g.z` of the box `(lo, hi)` within the prism standing on the
/// rectangle `rect` of `g`'s `x` and `y`**: each of the box's faces clipped to the prism, the
/// least and the greatest height among what is left (the prism's sides are parallel to `g.z`, so
/// every corner of the two's common part lies on a face of the box). `None` where they share nothing.
fn span_within((lo,hi): (V,V),g: &Frame,rect: [P;2]) -> Option<[f64;2]> {
    let corner = |k: usize| -> V { std::array::from_fn(|i| if (k>>i)&1 == 0 { lo[i] } else { hi[i] }) };
    // each face of the box, its four corners round it
    let faces: [[usize;4];6] = [[0,2,6,4],[1,3,7,5],[0,1,5,4],[2,3,7,6],[0,1,3,2],[4,5,7,6]];
    let mut span = [f64::INFINITY,f64::NEG_INFINITY];
    for f in faces {
        let mut poly: Vec<V> = f.iter().map(|&k| g.local(corner(k))).collect();
        // clipped by each of the four half-planes `±(coordinate i − bound) ≤ 0`
        for (i,bound,sign) in [(0,rect[0][0],-1.),(0,rect[1][0],1.),(1,rect[0][1],-1.),(1,rect[1][1],1.)] {
            let past = |p: &V| sign*(p[i]-bound);
            let mut out = Vec::new();
            for k in 0..poly.len() {
                let (p,q) = (poly[k],poly[(k+1)%poly.len()]);
                let (fp,fq) = (past(&p),past(&q));
                if fp <= 0. { out.push(p) }
                if (fp <= 0.) != (fq <= 0.) {
                    let s = fp/(fp-fq);
                    out.push(std::array::from_fn(|j| p[j]+s*(q[j]-p[j])));
                }
            }
            poly = out;
            if poly.is_empty() { break }
        }
        for p in poly { span = [span[0].min(p[2]),span[1].max(p[2])] }
    }
    (span[0] <= span[1]).then_some(span)
}

/// **The blank less what the tool passes through, slid along `path` over `roll`**, by its shadow
/// (the module's); `None` where this road does not take it, or refuses it, for the others to read.
pub fn cut(blank: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Option<Result<Brep,Unbuilt>> {
    let screw = path.screw()?;
    if !screw.is_slide() { return None }
    let (start,end) = (path.pose(roll[0]).ok()?,path.pose(roll[1]).ok()?);
    let step = sub(end.p,start.p);
    let length = norm(step);
    if !(length > 0.) { return None }
    let d = unit(step);
    let (tlo,thi) = tool.bounds();
    if !tlo[0].is_finite() { return None }
    let size = distance(tlo,thi);
    let g = Frame::about(start.point(crate::util::space::lerp(tlo,thi,0.5)),d);
    // the tool's box (drawn) read along the slide: its heights, and the rectangle holding its shadow
    let corners: Vec<V> = (0..8).map(|k| g.local(std::array::from_fn(|i| if (k>>i)&1 == 0 { tlo[i] } else { thi[i] }))).collect();
    let rect = corners.iter().fold([[f64::INFINITY;2],[f64::NEG_INFINITY;2]],|[l,h],c| [[l[0].min(c[0]),l[1].min(c[1])],[h[0].max(c[0]),h[1].max(c[1])]]);
    let [low,high] = corners.iter().fold([f64::INFINITY,f64::NEG_INFINITY],|[l,h],c| [l.min(c[2]),h.max(c[2])]);
    // the tool where it starts and where it ends: the drawn tool's heights moved by each pose's step
    // along the slide (a slide moves the plane's coordinates by nothing)
    let ([s0,s1],[e0,e1]) = ([low,high].map(|h| h+dot(start.p,d)),[low,high].map(|h| h+dot(end.p,d)));
    let pad = 1e-6*(1.+size);
    let Some([b0,b1]) = span_within(blank.bounds(),&g,rect) else {
        // the tool's shadow misses the blank: nothing is cut
        return Some(Ok(blank.clone()))
    };
    if !(s1 < b0-pad && e0 > b1+pad) {
        if crate::debug() { eprintln!("through: the slide does not carry the tool wholly through the blank ({s0}..{s1} → {e0}..{e1} past {b0}..{b1})") }
        return None
    }
    match shadow(tool,&screw,&g,bar,tol) {
        Ok(regions) => {
            let reach = 0.1*(b1-b0).max(size);
            let mut prisms = Brep::default();
            for profile in regions {
                match prism(&profile,b0-reach,b1+reach) {
                    Ok(p) => prisms.join(&p),
                    Err(why) => { if crate::debug() { eprintln!("through: the shadow's prism not built: {why}") } return None }
                }
            }
            Some(crate::boolean::combined(blank,&prisms,crate::boolean::Op::Cut,tol)
                .map_err(|e| Unbuilt::Failed(format!("the shadow's prism cut from the blank: {e}"))))
        }
        Err(why @ Unbuilt::Budget {..}) => Some(Err(why)),
        Err(why) => { if crate::debug() { eprintln!("through: the shadow refused ({why:?}): the other roads taken") } None }
    }
}

/// **The tool's shadow along the slide `screw`**, in the plane `g` square to it: each region with
/// its holes, a profile in the plane (the module's steps 1 to 3), membership read on the tool meshed
/// within `bar` (and an eighth of it, where that leaves a reading unsettled).
fn shadow(tool: &Brep,screw: &Screw,g: &Frame,bar: f64,tol: f64) -> Result<Vec<Profile>,Unbuilt> {
    let size = tool.size();
    let twist = screw.twist();
    // (1) the candidates: each face's contact, and the edges that may bound the shadow
    let located = Located::new(tool,tol);
    // (each piece's points held to a ten-thousandth of the tool chord by chord: its curve is fitted
    // through points settled on it, within the tolerance, `piece_curve`)
    let contacts: Vec<Contact> = crate::util::par::indices(tool.faces.len(),|fi|
        super::contacts::face_contact_within(tool,&located,fi,&twist,tol,CELLS,(1e-4*size).max(tol)));
    let whole: Vec<bool> = contacts.iter().map(|c| matches!(c,Contact::Whole)).collect();
    let pieces: Vec<super::contacts::Piece> = contacts.into_iter().flat_map(|c| match c { Contact::Pieces(p) => p,Contact::Whole => Vec::new() })
        .filter(|p| !super::contacts::buried_on(tool,size,&p.points,&[p.face],screw)).collect();
    let fitted: Vec<Option<Curve>> = crate::util::par::indices(pieces.len(),|k| super::contacts::piece_curve(tool,&located,&pieces[k],&twist,tol));
    let mut cands: Vec<Plan> = Vec::new();
    for c in fitted {
        match c {
            Some(Curve::BSpline(s)) => cands.push(Plan::carried(&s,g)),
            _ => return Err(Unbuilt::Failed("a piece of contact not fitted".into())),
        }
    }
    let sharp: std::collections::BTreeSet<usize> = super::contacts::sharp_edges(tool).map_err(|at| Unbuilt::Curved {at})?.into_iter().map(|(e,_)| e).collect();
    let uses = tool.uses();
    for (ei,e) in tool.edges.iter().enumerate() {
        let EdgeCurve::Curve(curve) = &e.curve else { continue };
        if !sharp.contains(&ei) && !uses[ei].iter().any(|u| whole[u.0]) { continue }
        let flat = |p: V| { let l = g.local(p); [l[0],l[1]] };
        match curve {
            Curve::Line {..} => {
                let (a,b) = (flat(e.point(e.t[0],&tool.vertices)),flat(e.point(e.t[1],&tool.vertices)));
                // (a line along the slide is a point of the plane, bounding nothing)
                if dist(a,b) > 1e-9*(1.+size) { cands.push(Plan::Segment {a,b}) }
            }
            &Curve::Circle(f,r) if r > 0. => cands.push(Plan::carried(&crate::nurbs::arc(&f,r,e.t),g)),
            Curve::BSpline(s) => cands.push(Plan::carried(&s.segment(e.t),g)),
            _ => return Err(Unbuilt::Curved {at:e.point(0.5*(e.t[0]+e.t[1]),&tool.vertices)}),
        }
    }
    if crate::debug() { eprintln!("through: {} candidates of the shadow's boundary",cands.len()) }
    // (2) arranged, each piece kept by the membership of the line through it along the slide
    let solid = Solid::new(tool,bar,1.).map_err(Unbuilt::Failed)?;
    let member = |solid: &Solid,p: P| solid.line(g.at([p[0],p[1],0.]),g.z);
    let mut finer: Option<Solid> = None;
    let mut line = |p: P| -> Result<bool,Unbuilt> {
        if let Ok(b) = member(&solid,p) { return Ok(b) }
        if finer.is_none() { finer = Some(Solid::new(tool,bar/8.,1.).map_err(Unbuilt::Failed)?) }
        member(finer.as_ref().unwrap(),p).map_err(|_| Unbuilt::Unsettled {at:g.at([p[0],p[1],0.])})
    };
    let near = 1e-7*(1.+size);
    let Arranged {vertices,kept,loops,polys,regions} = arranged(&cands,near,solid.mesh_sag(),size,&mut line)?;
    if crate::debug() { eprintln!("through: {} pieces kept in {} loops",kept.len(),loops.len()) }
    // (3) judged: the loops' parity against the membership, at points clear of the boundary
    let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for p in polys.iter().flatten() { for k in 0..2 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]) } }
    let margin = (16.*solid.mesh_sag()).max(1e-6*(1.+size));
    let chords: Vec<[P;2]> = polys.iter().flat_map(|p| (0..p.len()).map(move |i| [p[i],p[(i+1)%p.len()]])).collect();
    let flat3 = |q: P| [q[0],q[1],0.];
    for i in 0..JUDGED {
        for j in 0..JUDGED {
            // (the grid set an irrational part of a cell off the box's corner)
            let q = [lo[0]+(hi[0]-lo[0])*(i as f64+0.381966)/JUDGED as f64,lo[1]+(hi[1]-lo[1])*(j as f64+0.381966)/JUDGED as f64];
            if chords.iter().any(|[a,b]| crate::util::space::segment_distance(flat3(q),flat3(*a),flat3(*b)) <= margin) { continue }
            let inside = polys.iter().filter(|p| super::region::parity(p,q)).count()%2 == 1;
            let Ok(is) = line(q) else { continue };
            if inside != is {
                return Err(Unbuilt::Wrong {at:g.at([q[0],q[1],0.]),held:inside})
            }
        }
    }
    // (4) each region a profile in the plane
    let edges = |l: &[usize]| -> Vec<ProfileEdge> {
        l.iter().map(|&k| { let p = &kept[k]; cands[p.m].edge(p.t,vertices[p.ends[0]],vertices[p.ends[1]],g) }).collect()
    };
    Ok(regions.iter().map(|(o,holes)| {
        let mut ls = vec![edges(&loops[*o])];
        for &h in holes { ls.push(edges(&loops[h])) }
        Profile {origin:g.o,normal:g.z,names:Vec::new(),loops:ls}
    }).collect())
}
