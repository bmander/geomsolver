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
use super::shadow::{Arranged,Candidate,Meshed,arranged};
use std::sync::Arc;

type V = [f64;3];
type P = [f64;2];

/// How many stretches a B-spline candidate is sampled in for its crossings, at least, and how many
/// to each of its poles.
const SAMPLES: usize = 160;
const PER_POLE: usize = 2;
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
    fn sample_count(&self) -> usize {
        match self { Plan::Spline(s) => SAMPLES.max(PER_POLE*s.poles.len()),Plan::Segment {..} => 1 }
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
fn span_within(bounds: (V,V),g: &Frame,rect: (V,V)) -> Option<[f64;2]> {
    let corners: Vec<V> = super::closed::corners_of(bounds).into_iter().map(|c| g.local(c)).collect();
    // each face of the box, its four corners round it
    let faces: [[usize;4];6] = [[0,2,6,4],[1,3,7,5],[0,1,5,4],[2,3,7,6],[0,1,3,2],[4,5,7,6]];
    let mut span = [f64::INFINITY,f64::NEG_INFINITY];
    for f in faces {
        let mut poly: Vec<V> = f.iter().map(|&k| corners[k]).collect();
        // clipped to each of the four half-planes `±(coordinate i − bound) ≥ 0`
        for (i,bound,sign) in [(0,rect.0[0],1.),(0,rect.1[0],-1.),(1,rect.0[1],1.),(1,rect.1[1],-1.)] {
            poly = crate::offset::clipped(&poly,&|p: V| sign*(p[i]-bound));
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
    if !(norm(step) > 0.) { return None }
    let d = unit(step);
    let (tlo,thi) = tool.bounds();
    if !tlo[0].is_finite() { return None }
    let size = distance(tlo,thi);
    let g = Frame::about(start.point(crate::util::space::lerp(tlo,thi,0.5)),d);
    // the tool's box (drawn) read along the slide: the rectangle holding its shadow (`x`, `y`) and its
    // heights (`z`)
    let rect = crate::geom::hull(super::closed::corners_of((tlo,thi)).into_iter().map(|c| g.local(c)));
    let (low,high) = (rect.0[2],rect.1[2]);
    // the tool's top where it starts and its bottom where it ends: the drawn tool's heights moved by
    // each pose's step along the slide (a slide moves the plane's coordinates by nothing)
    let (top,bottom) = (high+dot(start.p,d),low+dot(end.p,d));
    let pad = 1e-6*(1.+size);
    let Some([b0,b1]) = span_within(blank.bounds(),&g,rect) else {
        // the tool's shadow misses the blank: nothing is cut
        return Some(Ok(blank.clone()))
    };
    if !(top < b0-pad && bottom > b1+pad) {
        if crate::debug() { eprintln!("through: the slide does not carry the tool wholly through the blank (its top {top} where it starts, its bottom {bottom} where it ends, the blank {b0}..{b1})") }
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
/// within `bar` (`shadow::Meshed`).
fn shadow(tool: &Brep,screw: &Screw,g: &Frame,bar: f64,tol: f64) -> Result<Vec<Profile>,Unbuilt> {
    let size = tool.size();
    let twist = screw.twist();
    let located = Located::new(tool,tol);
    // (1) the candidates: each face's contact (each piece's points held to a ten-thousandth of the
    // tool chord by chord: its curve is fitted through points settled on it, within the tolerance,
    // `piece_curve`), the faces read one after another and each over the cores, and the edges that
    // may bound the shadow — read first, as an edge of a curve not carried refuses the road
    let contacts: Vec<Contact> = (0..tool.faces.len())
        .map(|fi| super::contacts::face_contact_within(tool,&located,fi,&twist,tol,CELLS,(1e-4*size).max(tol))).collect();
    let whole: Vec<bool> = contacts.iter().map(|c| matches!(c,Contact::Whole)).collect();
    let sharp: std::collections::BTreeSet<usize> = super::contacts::sharp_edges(tool).map_err(|at| Unbuilt::Curved {at})?.into_iter().map(|(e,_)| e).collect();
    let uses = tool.uses();
    let flat = |p: V| { let l = g.local(p); [l[0],l[1]] };
    let mut cands: Vec<Plan> = Vec::new();
    for (ei,e) in tool.edges.iter().enumerate() {
        let EdgeCurve::Curve(curve) = &e.curve else { continue };
        if !sharp.contains(&ei) && !uses[ei].iter().any(|u| whole[u.0]) { continue }
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
    let pieces: Vec<super::contacts::Piece> = contacts.into_iter().flat_map(|c| match c { Contact::Pieces(p) => p,Contact::Whole => Vec::new() })
        .filter(|p| !super::contacts::buried_on(tool,size,&p.points,&[p.face],screw)).collect();
    let fitted = crate::util::par::indices(pieces.len(),|k| match super::contacts::piece_curve(tool,&located,&pieces[k],&twist,tol) {
        Some(Curve::BSpline(s)) => Ok(Plan::carried(&s,g)),
        _ => Err(Unbuilt::Failed("a piece of contact not fitted".into())),
    });
    cands.extend(fitted.into_iter().collect::<Result<Vec<_>,_>>()?);
    if crate::debug() { eprintln!("through: {} candidates of the shadow's boundary",cands.len()) }
    // (2) arranged, each piece kept by whether the line along the slide through it meets the tool
    let mut meshed = Meshed::new(tool,bar)?;
    let sag = meshed.coarse.mesh_sag;
    let mut line = |p: P| { let x = g.at([p[0],p[1],0.]); meshed.read(x,|s| s.line(x,g.z)) };
    let Arranged {vertices,kept,loops,polys,regions} = arranged(&cands,1e-7*(1.+size),sag,size,&mut line)?;
    if crate::debug() { eprintln!("through: {} pieces kept in {} loops",kept.len(),loops.len()) }
    // (3) judged: the loops' parity against the membership, at points of a grid over the loops' box
    // clear of them (their signed distance, `subtraction::Loops`)
    let chords: Vec<crate::plane2::Edge> = polys.iter().flat_map(|p| (0..p.len()).map(move |i| crate::plane2::Edge::Line {a:p[i],b:p[(i+1)%p.len()],axis:false})).collect();
    let field = super::subtraction::Loops::new(&[chords]);
    let ([lo0,lo1],[hi0,hi1]) = polys.iter().flatten().fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(l,h),p| ([l[0].min(p[0]),l[1].min(p[1])],[h[0].max(p[0]),h[1].max(p[1])]));
    let margin = (16.*sag).max(1e-6*(1.+size));
    for i in 0..JUDGED {
        for j in 0..JUDGED {
            // (the grid set an irrational part of a cell off the box's corner)
            let q = [lo0+(hi0-lo0)*(i as f64+0.381966)/JUDGED as f64,lo1+(hi1-lo1)*(j as f64+0.381966)/JUDGED as f64];
            let reading = super::planar::Region::value(&field,q);
            if reading.abs() <= margin { continue }
            let Ok(is) = line(q) else { continue };
            if (reading < 0.) != is { return Err(Unbuilt::Wrong {at:g.at([q[0],q[1],0.]),held:reading < 0.}) }
        }
    }
    // (4) each region a profile in the plane
    let edges = |l: &[usize]| -> Vec<ProfileEdge> {
        l.iter().map(|&k| { let p = &kept[k]; cands[p.m].edge(p.t,vertices[p.ends[0]],vertices[p.ends[1]],g) }).collect()
    };
    Ok(regions.iter().map(|region| Profile {origin:g.o,normal:g.z,names:Vec::new(),loops:region.iter().map(|&l| edges(&loops[l])).collect()}).collect())
}
