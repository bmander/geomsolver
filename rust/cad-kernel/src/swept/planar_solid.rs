//! **A prism under a motion that keeps a plane, swept closed** (#880 Q2): where every pose of the
//! motion turns one plane in itself, the same face up, and the tool stands along that plane's
//! normal — every face a cap square to it or a side along it, however it was drawn — each point of
//! the tool moves within its own height. So the swept solid is, slab by slab between the tool's
//! caps, the region the section there sweeps in the plane, `R = ∪ₜ poseₜ(section)`, carried
//! through the slab's heights: no blank read, nothing outside the solid computed.
//!
//! The region is the planar classes' reading (`region::SweptRegion`): its boundary's candidates —
//! each side's contact, each corner's path (a trochoid), each edge at an end pose where the roll is
//! no whole period — kept where they stand outside the section at every pose of the roll, and
//! linked into loops. Here it is read within a box that holds the whole region (every pose's
//! section with a quarter of its size to spare), so no stretch of it is the box's: a loop that
//! runs along the box is refused, never built. Each loop is fitted, a cubic B-spline a stretch
//! from corner to corner (`region::profiles`) within a tenth of the tolerance, and extruded
//! through its slab (`build::prism`); the slabs, and the separate pieces of one, are united.
//!
//! **Judged before it is returned**, as every closed build is (`closed::judge`): the solid's exact
//! point location against the sweep's membership at seeded points of a box holding it, and every
//! face's sense. A section whose sides lean (a drafted tool, its section moving with the height),
//! a motion that keeps no plane the tool stands along, are not this method's: `read` says so and
//! the sweep goes on to the others. Its refusals are the region's, named and with a point
//! (`Unbuilt::Folded` where the boundary folds back on itself; `Unbuilt::Failed` saying what was
//! seen otherwise).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::build::{Profile,prism};
use crate::topo::Brep;
use crate::trajectory::Trajectory;
use super::closed::Unbuilt;
use super::member::{Solid,Swept};
use super::region::{profiles,Piece,Setting,SweptRegion};
use super::subtraction::{kept_normals,Prismatic};

type V = [f64;3];
type P = [f64;2];

/// How many poses, at least, the plane is checked kept at before the region is read (which checks
/// it at its own grid), and how many a degree the box holding the region is read at.
const KEPT_POSES: usize = 17;
const BOX_PER_DEGREE: f64 = 4.;

/// How finely each stretch of the region is fitted, as a part of the tolerance.
const FIT: f64 = 0.1;

/// **Whether the motion keeps a plane the tool stands along**: the first normal of a plane face of
/// `tool` whose plane every pose of `path` over the increasing `roll` keeps (`kept_normals`, at
/// `KEPT_POSES`), the tool read as standing along it (`subtraction::Prismatic`), and its slabs
/// between its caps, none with a side leaning. None where there is no such plane, or the tool
/// stands along none of them, or its section changes with the height.
fn read(tool: &Brep,path: &Trajectory,[a,b]: [f64;2]) -> Option<(Prismatic,Vec<[f64;2]>)> {
    if !(a < b) { return None }
    let poses: Vec<_> = (0..KEPT_POSES).map(|k| path.at(a+(b-a)*k as f64/(KEPT_POSES-1) as f64)).collect::<Result<_,_>>().ok()?;
    kept_normals(tool,&poses,1.).into_iter().find_map(|n| {
        let tool = Prismatic::along(tool,n,1.).ok()?;
        let slabs = tool.slabs();
        (!slabs.iter().any(|s| tool.leans(0.5*(s[0]+s[1])))).then_some((tool,slabs))
    })
}

/// **The closed solid `tool` sweeps under `path` over `roll`, where the motion keeps a plane the
/// tool stands along** (the module's method), built to `tol` and judged by membership read on the
/// tool meshed within `bar`; none where `read` finds no such plane.
pub fn swept(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Option<Brep>,Unbuilt> {
    let roll = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let Some((standing,slabs)) = read(tool,path,roll) else { return Ok(None) };
    let plane = standing.plane;
    let mut solids: Vec<Brep> = Vec::new();
    let mut held: Vec<V> = Vec::new();
    for &[h0,h1] in &slabs {
        let h = 0.5*(h0+h1);
        let setting = standing.setting(h,true,path,roll).map_err(|e| Unbuilt::Failed(format!("the tool's section at height {h}: {e}")))?;
        if setting.section.is_empty() { continue }
        let whole = setting.alike(roll[0],roll[1]);
        let bounds = holding(&setting,roll);
        let region = SweptRegion::read(setting,bounds,whole).map_err(|r| {
            let at = r.witness.map(|p| plane.world(p,h));
            match (&r.seen,at) {
                (super::region::Seen::Folds,Some(at)) => Unbuilt::Folded {at},
                _ => Unbuilt::Failed(format!("the region the tool's section at height {h:.6} sweeps: {}{}",r.words("the motion"),
                    at.map_or(String::new(),|p| format!(", at ({:.6}, {:.6}, {:.6})",p[0],p[1],p[2])))),
            }
        })?;
        crate::util::meter::go_on()?;
        // a loop along the box is the box's, not the region's: the box was to hold it all
        if let Some(p) = region.loops.iter().flat_map(|l| &l.pieces).find_map(|p| match *p { Piece::Side {a,..} => Some(a),_ => None }) {
            return Err(Unbuilt::Failed(format!("the region the tool's section at height {h:.6} sweeps reaches past the box held \
                about every pose of it, at {:?}",plane.world(p,h))))
        }
        if region.loops.is_empty() { return Err(Unbuilt::Failed(format!("the tool's section at height {h:.6} sweeps no region"))) }
        for profile in profiles(&region,1.,FIT*tol,"the sweep")? {
            solids.push(prism(&Profile::from_json(&profile)?,h0,h1)?);
        }
        held.extend([bounds[0],bounds[1]].iter().flat_map(|&p| [plane.world(p,h0),plane.world(p,h1)]));
    }
    let out = solids.into_iter().map(Ok).reduce(|a: Result<Brep,Unbuilt>,b| Ok(crate::boolean::combined(&a?,&b?,crate::boolean::Op::Union,tol)?))
        .ok_or_else(|| Unbuilt::Failed("the tool has no section".into()))??;
    out.check(tol)?;
    // **judged by its definition before it is returned** (#777 V2), over a box holding it
    let (domain,..) = super::closed::cube_holding(&held,tol)?;
    let solid = Solid::new(tool,bar,1.)?;
    super::closed::judge(&out,&Swept {tool:&solid,path,roll},&domain,None,&[],tol)?;
    Ok(Some(out))
}

/// A box of the plane holding the region `setting`'s section sweeps over `roll`: the section's
/// points carried by poses `BOX_PER_DEGREE` a degree, and a quarter of the section's size and the
/// farthest one of them moved between two poses to spare.
fn holding(setting: &Setting,roll: [f64;2]) -> [P;2] {
    let n = (((roll[1]-roll[0]).to_degrees()*BOX_PER_DEGREE).ceil() as usize).max(64);
    // the section's points read once, each pose carrying them
    let local = setting.points(4);
    let pl = &setting.plane;
    let poses: Vec<Vec<P>> = crate::util::par::indices(n+1,|k| setting.path.pose(roll[0]+(roll[1]-roll[0])*k as f64/n as f64)
        .map_or(Vec::new(),|m| local.iter().map(|&x| pl.page(m.point(pl.world(x,0.)))).collect()));
    let span = |ps: &mut dyn Iterator<Item=&P>| ps.fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(l,h),p| ([l[0].min(p[0]),l[1].min(p[1])],[h[0].max(p[0]),h[1].max(p[1])]));
    let (lo,hi) = span(&mut poses.iter().flatten());
    let (slo,shi) = span(&mut local.iter());
    let size = (shi[0]-slo[0]).dhypot(shi[1]-slo[1]);
    let moved = poses.windows(2).flat_map(|w| w[0].iter().zip(&w[1]).map(|(p,q)| (p[0]-q[0]).dhypot(p[1]-q[1]))).fold(0.,f64::max);
    let pad = 0.25*size+moved+1e-6*(1.+setting.scale);
    [[lo[0]-pad,lo[1]-pad],[hi[0]+pad,hi[1]+pad]]
}
