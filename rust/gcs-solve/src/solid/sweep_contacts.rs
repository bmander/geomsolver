//! The contacts of a continuous sweep's tool, read from an ordinary swept-solid definition. The
//! tool's faces, their contacts and the sweep's contacts are the kernel's
//! (`cad_kernel::sweep::tool`); here is what reads a sweep's tool off the model — its revolutions'
//! and prisms' faces placed as its Booleans and placements put them, its motion and its roll —
//! the kernel's `Source` for the sketch, and the tool's material field as the kernel's `Material`.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{SpatialField,RevolvedSurface};
pub use cad_kernel::sweep::tool::*;
use crate::{envelope::Motion,model::{EntRef,Sketch,SolidDef,EntKind},motion::Family};
use crate::space::{add,scale,sub};
use std::{collections::BTreeSet,f64::consts::TAU,sync::Arc};

impl Material for SpatialField {
    fn value(&self,p: [f64;3]) -> f64 { SpatialField::value(self,p) }
}

impl Source for Sketch {
    type Edge = EntRef;
    fn contacts(&self,solid: usize,axis_tolerance: f64) -> Result<SweepContacts,String> { contacts(self,solid,axis_tolerance) }
    fn extruded(&self,solid: usize,edge: EntRef) -> Result<ExtrudedSurface,String> { extruded(self,solid,edge) }
    fn drafted(&self,solid: usize) -> Result<Vec<Vec<Vec<ExtrudedSurface>>>,String> { drafted(self,solid) }
    fn caps(&self,solid: usize) -> Result<[CapSurface;2],String> { caps(self,solid) }
}

/// Prism `solid`'s caps, its `start` then its `end`.
fn caps(sk: &Sketch,solid: usize) -> Result<[CapSurface;2],String> {
    let s = sk.solids.get(solid).ok_or("no such solid")?;
    let SolidDef::Prism {face,from,to,draft:None} = &s.def else { return Err("a cap is read of a prism with square sides".into()) };
    let p = super::face_poly(sk,*face as usize,super::REPORT_UNIT).ok_or("invalid prism profile")?;
    let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for &(x,y) in &p.pts { lo = [lo[0].min(x),lo[1].min(y)]; hi = [hi[0].max(x),hi[1].max(y)]; }
    if !(hi[0] > lo[0] && hi[1] > lo[1]) { return Err("a prism's profile of no area".into()) }
    // a little past the profile's points, which its arcs bulge beyond
    let pad = 0.02*(hi[0]-lo[0]).max(hi[1]-lo[1]);
    let (lo,hi) = (lo.map(|x| x-pad),hi.map(|x| x+pad));
    let normal = p.basis.normal();
    let (u,v) = (scale(p.basis.u,hi[0]-lo[0]),scale(p.basis.v,hi[1]-lo[1]));
    let at = |h: f64| add(p.basis.lift(lo[0],lo[1]),scale(normal,h));
    // `u × v` is the profile's normal: outward at the farther cap along it, and at the nearer
    // `v` runs back
    let cap = |h: f64,other: f64,end: bool| {
        let farther = h >= other;
        CapSurface::new(format!("{}.{}",s.name,if end { "end" } else { "start" }),end,
            if farther { at(h) } else { add(at(h),v) },u,if farther { v } else { scale(v,-1.) })
    };
    Ok([cap(from.value,to.value,false),cap(to.value,from.value,true)])
}

/// Edge `edge` of prism `solid`'s profile, its face's lifted as the prism's recipe lifts it.
fn extruded(sk: &Sketch,solid: usize,edge: EntRef) -> Result<ExtrudedSurface,String> {
    let solid = sk.solids.get(solid).ok_or("no such solid")?;
    let SolidDef::Prism {face,from,to,draft:None} = &solid.def else { return Err("an extruded patch requires a prism with square sides".into()) };
    let profile = sk.faces.get(*face as usize).ok_or("no such profile")?;
    let name = profile.boundaries().flat_map(|(e,n)| e.iter().zip(n))
        .find_map(|(e,n)| (*e == edge).then_some(n))
        .ok_or("the requested edge is not a boundary of this prism's profile")?;
    let p = super::face_poly(sk,*face as usize,super::REPORT_UNIT).ok_or("invalid prism profile")?;
    let lift = |q: (f64,f64)| p.basis.lift(q.0,q.1);
    let vector = |x: f64,y: f64| add(scale(p.basis.u,x),scale(p.basis.v,y));
    let normal = p.basis.normal();
    let base = scale(normal,from.value);
    let edge = match edge.kind {
        EntKind::Line => {
            let line = sk.lines.get(edge.i()).ok_or("no such profile line")?;
            let start = lift(sk.point_xy(line.p1 as usize));
            ProfileEdge::Line {start:add(start,base),delta:sub(lift(sk.point_xy(line.p2 as usize)),start)}
        }
        EntKind::Arc | EntKind::Circle => {
            let (center,radius,start,sweep) = if edge.kind == EntKind::Arc {
                let a = sk.arcs.get(edge.i()).ok_or("no such profile arc")?;
                let (start,end) = sk.arc_angles(edge.i());
                (a.center,a.radius,start,end-start)
            } else {
                let c = sk.circles.get(edge.i()).ok_or("no such profile circle")?;
                (c.center,c.radius,0.,TAU)
            };
            let r = sk.params[radius as usize].value;
            if !r.is_finite() || r <= 0. || !sweep.is_finite() || sweep <= 0. {
                return Err("a round profile edge needs positive finite radius and sweep".into());
            }
            ProfileEdge::round(add(lift(sk.point_xy(center as usize)),base),r,start,sweep,vector)
        }
        _ => return Err("an extruded patch requires a line, arc or circle".into()),
    };
    let depth = scale(normal,to.value-from.value);
    if !(depth[0].dhypot(depth[1]).dhypot(depth[2]) > 0.) { return Err("a prism of no depth".into()) }
    Ok(ExtrudedSurface::new(format!("{}.{}",solid.name,name),edge,Side::Straight(depth)))
}

/// The sides of drafted prism `solid` (#684 G), half by half and loop by loop, each loop walked
/// with its material on its left: each side the ruled surface between its edge in the section at
/// one end of its half and in the section at the other, each section the profile moved `|h| tan
/// draft` into its material (`field::Sections`, the section `build::drafted` builds). That is the
/// side exactly where every point at a fixed fraction of its edge moves along a line as the draft
/// deepens: where each corner joins two lines or goes on tangent (`Sections::linear`). A prism on
/// one side of its parting plane is one half, from `from` to `to`; one running both ways from it
/// (#684 P) two, from `from` to the plane and from the plane to `to`, side `k` of loop `l` of the
/// first meeting the second's at the profile itself, a convex crease across the side whose fan
/// turns its normal from the one half's to the other's. Refused otherwise, saying why.
fn drafted(sk: &Sketch,solid: usize) -> Result<Vec<Vec<Vec<ExtrudedSurface>>>,String> {
    use super::surface::Edge as Drawn;
    let solid = sk.solids.get(solid).ok_or("no such solid")?;
    let SolidDef::Prism {face,from,to,draft:Some(draft)} = &solid.def else {
        return Err("a drafted prism's sides require a drafted prism".into())
    };
    let (from,to) = (from.value,to.value);
    let parted = from.min(to) < 0. && from.max(to) > 0.;
    if parted && draft.value < 0. {
        return Err("it runs both ways from its profile's plane, drafted outward, so its sides meet there in a concave crease, \
            which trims the two halves' envelopes against each other".into())
    }
    let profile = sk.faces.get(*face as usize).ok_or("no such profile")?;
    let named: Vec<(&EntRef,&String)> = profile.boundaries().flat_map(|(e,n)| e.iter().zip(n)).collect();
    if let Some((_,n)) = named.iter().find(|(e,_)| !matches!(e.kind,EntKind::Line | EntKind::Arc | EntKind::Circle)) {
        return Err(format!("its side `{n}` is no line, arc or circle"))
    }
    // each drawn edge, a line, arc or circle, read as one loop edge in the profile's order
    let (basis,loops) = super::field::prism_loops(sk,*face as usize)?;
    let drawn: Vec<(&Drawn,&String)> = loops.iter().flatten().zip(named.iter().map(|(_,n)| *n)).collect();
    let sections = super::field::Sections::new(&loops,draft.value,[from,to]).map_err(|e| e.replace("the field reads ",""))?;
    if !sections.linear() {
        return Err("an arc meets a side at an angle, and a draft moves that corner along the curve its cone and the side \
            meet in, not along a line".into())
    }
    let normal = basis.normal();
    let lift = |q: [f64;2],h: f64| add(basis.lift(q[0],q[1]),scale(normal,h));
    let vector = |x: f64,y: f64| add(scale(basis.u,x),scale(basis.v,y));
    let edge = |e: &Drawn,h: f64| match *e {
        Drawn::Line {a,b,..} => ProfileEdge::Line {start:lift(a,h),delta:vector(b[0]-a[0],b[1]-a[1])},
        Drawn::Arc {center,radius,start,sweep,..} => ProfileEdge::round(lift(center,h),radius,start,sweep,vector),
        Drawn::Conic {..} => unreachable!("refused when read"),
    };
    // each side named by the drawn edge it walks: of its kind, through its ends, an arc about its centre
    let same = |p: [f64;2],q: [f64;2]| (p[0]-q[0]).dhypot(p[1]-q[1]) <= 1e-9*(1.+p[0].abs().max(p[1].abs()));
    let name = |e: &Drawn| drawn.iter().find(|(d,_)| {
        let ([a,b],[p,q]) = (e.endpoints(),d.endpoints());
        ((same(a,p) && same(b,q)) || (same(a,q) && same(b,p))) && match (e,d) {
            (Drawn::Line {..},Drawn::Line {..}) => true,
            (Drawn::Arc {center:c,..},Drawn::Arc {center:o,..}) => same(*c,*o),
            _ => false,
        }
    }).map(|(_,n)| n.as_str());
    // each half from one height to the other
    let halves = if parted { vec![[from,0.],[0.,to]] } else { vec![[from,to]] };
    halves.into_iter().map(|[h0,h1]| {
        let (near,far) = (sections.section_at(h0)?,sections.section_at(h1)?);
        sections.loops().iter().enumerate().map(|(l,lp)| lp.iter().enumerate().map(|(k,e)| {
            let name = name(e).ok_or("a drafted side is no drawn edge of its profile")?;
            Ok(ExtrudedSurface::new(format!("{}.{}",solid.name,name),edge(&near[l][k],h0),
                Side::Leaning(edge(&far[l][k],h1))))
        }).collect()).collect()
    }).collect()
}

fn contacts(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<SweepContacts,String> {
    super::validate(sk,solid)?;
    let SolidDef::Swept {source,motion,..} = &sk.solids[solid].def else {
        return Err("contact construction requires a continuous swept solid".into());
    };
    let source_field = SpatialField::read(sk,*source as usize,axis_tolerance)?;
    let mut pending = vec![(*source as usize,Motion::identity())];
    let mut seen = BTreeSet::new();
    let mut patches = Vec::new();
    let mut loops = Vec::new();
    let mut creases: Vec<[usize;2]> = Vec::new();
    while let Some((id,pose)) = pending.pop() {
        let columns: [[u64;3];4] = std::array::from_fn(|i| {
            let p = if i == 3 { pose.point([0.;3]) } else {
                let mut p = [0.;3]; p[i] = 1.; pose.vector(p)
            }; p.map(f64::to_bits)
        });
        if !seen.insert((id,columns)) { continue; }
        match &sk.solids[id].def {
            SolidDef::Revolve {face,..} => {
                for (edges,_) in sk.faces[*face as usize].boundaries() {
                    let mut order = Vec::with_capacity(edges.len());
                    for &edge in edges {
                        let surface = RevolvedSurface::read(sk,id,edge)?.placed(pose);
                        let surface = ToolSurface::Revolved(surface);
                        if edge.kind == EntKind::Line {
                            let radius = |u| surface.at(u,0.).map(|p|
                                p.dv[0].dhypot(p.dv[1]).dhypot(p.dv[2])/TAU);
                            // A diameter on the axis disappears in a full revolution.
                            if radius(0.).map_err(|e| format!("{e:?}"))? <= axis_tolerance
                                && radius(1.).map_err(|e| format!("{e:?}"))? <= axis_tolerance { order.push(None); continue; }
                        }
                        order.push(Some(patches.len()));
                        patches.push(surface);
                    }
                    loops.push(order);
                }
            }
            // a region's faces, each step of its meridian turned (§6.21): its profile is the
            // meridian, read as a revolution's
            SolidDef::Region {..} => {
                let m = super::region::meridian(sk,id)?;
                for lp in RevolvedSurface::meridian_loops(&m,&sk.solids[id].name) {
                    loops.push(lp.into_iter().map(|s| s.map(|s| {
                        patches.push(ToolSurface::Revolved(s.placed(pose)));
                        patches.len()-1
                    })).collect());
                }
            }
            // a drafted prism's sides, each a plane or a cone between its two sections (#684 G),
            // and where it runs both ways from its parting plane each side's halves' crease there
            // (#684 P); its caps are admission's to keep out of the blank (`admission::ends`)
            SolidDef::Prism {draft:Some(_),..} => {
                let (halves,start) = (drafted(sk,id)?,patches.len());
                for sides in halves.iter().flatten() {
                    loops.push(sides.iter().map(|side| {
                        patches.push(ToolSurface::Extruded(side.placed(pose)));
                        Some(patches.len()-1)
                    }).collect());
                }
                // the halves side for side alike, the second's sides the first's as many on
                if halves.len() == 2 {
                    let n = (patches.len()-start)/2;
                    creases.extend((start..start+n).map(|a| [a,a+n]));
                }
            }
            // a prism's sides, one an edge of its profile, then its caps, square to the
            // extrusion, which admission asks only where one reaches the blank (#684 F)
            SolidDef::Prism {face,..} => {
                for (edges,_) in sk.faces[*face as usize].boundaries() {
                    let mut order = Vec::with_capacity(edges.len());
                    for &edge in edges {
                        order.push(Some(patches.len()));
                        patches.push(ToolSurface::Extruded(extruded(sk,id,edge)?.placed(pose)));
                    }
                    loops.push(order);
                }
                patches.extend(caps(sk,id)?.map(|c| ToolSurface::Cap(c.placed(pose))));
            }
            SolidDef::Placed {source,motion,at} => pending.push((*source as usize,
                Family::read(sk,*motion as usize)?.at(at.value)?.then(pose))),
            SolidDef::Body {..} => pending.extend(sk.solids[id].operands().into_iter()
                .rev().map(|id| (id as usize,pose))),
            _ => return Err("contact construction requires static revolutions, prisms and Boolean operands".into()),
        }
    }
    let edges = PatchEdge::of_loops(&patches,&loops)?;
    // a whole turn or more sweeps what one whole turn does, a solid of revolution (#684 N): every
    // reader of its contacts reads it once round
    let (motion,roll) = (Family::read(sk,*motion as usize)?,super::roll::interval(sk,solid)?);
    let once = motion.twist().and_then(|screw| super::constant_twist::once_round(&screw,roll));
    Ok(SweepContacts::new(Arc::new(source_field),patches,edges,creases,motion,once.unwrap_or(roll),once.is_some()))
}
