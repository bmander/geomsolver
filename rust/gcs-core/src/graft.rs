//! The rebuild walk: a sketch copied through a filter, every surviving entity renumbered and
//! every reference to it following.  Deletion (`without`), copying (`copy`), pasting (`paste`)
//! and the part of a document one gesture can move (`Part`) are all this one walk, so they cannot
//! disagree about what a constraint belongs to.

use crate::constraints::{Arg, CKind, Constraint};
use crate::decompose;
use crate::expr;
use crate::model::{EntKind, EntRef, Sketch};
use std::collections::{BTreeMap, BTreeSet};

/// Graft the entities of `src` that `keep` accepts onto `dst`, moved by `offset`, and return
/// what that made, in `src` order.
///
/// This is the one rebuild walk in the project: every surviving entity is renumbered into `dst`
/// and every reference to it follows — which is what deletion, copying and pasting all are,
/// differing only in what they keep, where they land and what they land on.  A constraint comes
/// along exactly when every entity it names came along, so the three operations cannot disagree
/// about what a constraint belongs to.
fn graft(dst: &mut Sketch, src: &Sketch, keep: &dyn Fn(EntRef) -> bool, drop_c: &[u32],
         offset: (f64, f64)) -> Vec<EntRef> {
    let base = dst.points.len();
    let fresh = base == 0;   // a rebuild owns the whole document; a paste only adds to it
    if fresh {
        // a rebuild, a deletion and a copy all keep the document they came from — which is what
        // makes a clipboard say what its numbers are in, and `paste` able to convert them
        dst.units = src.units;
        dst.sheet = src.sheet.clone();
    }
    let mut made = Vec::new();
    // a face family's running numbers, before any contact pinned to one is added (#243 L1)
    for (name, h) in &src.held {
        if !dst.held.contains_key(name) {
            dst.hold(name, src.params[h.param as usize].value, h.dim);
        }
    }
    // a paste onto a plane the document already has: one held where the clipboard's is held and
    // facing the same way *is* that plane, so what was drawn in it is drawn in it again and no
    // second plane stands on the first; its axes and its origin are the document's
    let mut reuse: Vec<Option<usize>> = vec![None; src.frames.len()];
    if !fresh {
        let same = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9);
        for (i, r) in reuse.iter_mut().enumerate() {
            if !keep(EntRef::frame(i)) || !src.plane_fixed(i) || src.frames[i].chart.is_none() {
                continue;
            }
            let b = src.basis(i);
            let planar = src.frames[i].planar();
            *r = dst.charted().into_iter().find(|&j| {
                let c = dst.basis(j);
                dst.frames[j].planar() == planar && dst.plane_fixed(j)
                    && same(b.u, c.u) && same(b.v, c.v) && same(b.o, c.o)
            });
        }
    }
    let reused_origin: BTreeMap<usize, usize> = reuse.iter().enumerate()
        .filter_map(|(i, r)| r.map(|j| (src.frames[i].origin as usize, dst.frames[j].origin as usize)))
        .collect();
    let mut keep_pts = Vec::new();
    let mut pt_map: Vec<Option<usize>> = vec![None; src.points.len()];
    for i in 0..src.points.len() {
        if let Some(&o) = reused_origin.get(&i) {
            pt_map[i] = Some(o);
        } else if keep(EntRef::point(i)) {
            pt_map[i] = Some(base + keep_pts.len());
            keep_pts.push(i);
        }
    }
    let pt_index = |i: usize| pt_map[i];
    for &i in &keep_pts {
        let (x, y) = src.point_xy(i);
        let n = dst.point(x + offset.0, y + offset.1, src.point_fixed(i),
                          &format!("p{}", dst.points.len()));
        if let Some(z) = src.points[i].z {
            dst.give_place(n, src.params[z as usize].value);
        }
        made.push(EntRef::point(n));
    }
    let mut line_map: Vec<Option<usize>> = vec![None; src.lines.len()];
    for i in 0..src.lines.len() {
        if !keep(EntRef::line(i)) {
            continue;
        }
        let l = &src.lines[i];
        let (Some(p1), Some(p2)) = (pt_index(l.p1 as usize), pt_index(l.p2 as usize)) else {
            continue;
        };
        let ni = dst.line(p1, p2);
        dst.lines[ni].class = l.class.clone();
        line_map[i] = Some(ni);
        made.push(EntRef::line(ni));
    }
    let mut circle_map: Vec<Option<usize>> = vec![None; src.circles.len()];
    for i in 0..src.circles.len() {
        if !keep(EntRef::circle(i)) {
            continue;
        }
        let c = &src.circles[i];
        let Some(centre) = pt_index(c.center as usize) else { continue };
        let ni = dst.circle(centre, src.params[c.radius as usize].value, "");
        let rp = dst.circles[ni].radius as usize;
        dst.params[rp].fixed = src.params[c.radius as usize].fixed;
        dst.circles[ni].class = c.class.clone();
        circle_map[i] = Some(ni);
        made.push(EntRef::circle(ni));
    }
    let mut arc_map: Vec<Option<usize>> = vec![None; src.arcs.len()];
    for i in 0..src.arcs.len() {
        if !keep(EntRef::arc(i)) {
            continue;
        }
        let a = &src.arcs[i];
        let (Some(c), Some(st), Some(e)) = (
            pt_index(a.center as usize),
            pt_index(a.start as usize),
            pt_index(a.end as usize),
        ) else {
            continue;
        };
        let ni = dst.arc(c, st, e, "");
        let rp = dst.arcs[ni].radius as usize;
        dst.params[rp].value = src.params[a.radius as usize].value;
        dst.params[rp].fixed = src.params[a.radius as usize].fixed;
        dst.arcs[ni].class = a.class.clone();
        arc_map[i] = Some(ni);
        made.push(EntRef::arc(ni));
    }
    let mut spline_map: Vec<Option<usize>> = vec![None; src.splines.len()];
    for i in 0..src.splines.len() {
        if !keep(EntRef::spline(i)) {
            continue;
        }
        let sp = &src.splines[i];
        // the control points that came along, and the ones that did not: a curve that has lost
        // some is shortened, one interior knot going with each, and only dies when too few are
        // left to draw with
        let (mut ctrl, mut gone) = (Vec::new(), Vec::new());
        for (k, &c) in sp.ctrl.iter().enumerate() {
            match pt_index(c as usize) {
                Some(n) => ctrl.push(n),
                None => gone.push(k),
            }
        }
        // with nothing gone this gives back the knots unchanged, so there is no case to split;
        // a weight goes with its control point
        let knots = crate::curve::knots_without(&sp.knots, &gone, ctrl.len());
        let weights = sp.weights.as_ref().map(|w| {
            w.iter().enumerate().filter(|(k, _)| !gone.contains(k)).map(|(_, &x)| x).collect()
        });
        let class = sp.class.clone();
        let Some(ni) = dst.spline_weighted(&ctrl, Some(knots), weights) else { continue };
        dst.splines[ni].class = class;
        spline_map[i] = Some(ni);
        made.push(EntRef::spline(ni));
    }
    // the axes, each its own: a direction, its fixed flags and its place — but a reused plane's,
    // which are the document's plane's
    let mut axis_map: Vec<Option<usize>> = vec![None; src.axes.len()];
    for (i, r) in reuse.iter().enumerate() {
        if let Some(j) = r {
            axis_map[src.frames[i].u as usize] = Some(dst.frames[*j].u as usize);
            axis_map[src.frames[i].v as usize] = Some(dst.frames[*j].v as usize);
        }
    }
    let mut places = Vec::new();
    for i in 0..src.axes.len() {
        if !keep(EntRef::new(EntKind::Axis, i)) || axis_map[i].is_some() {
            continue;
        }
        let r = &src.axes[i];
        let value = |q: u32| src.params[q as usize].value;
        let ni = dst.axis_restored(r.d.map(value), r.d.map(|q| src.params[q as usize].fixed),
                                  r.class.clone());
        // a place held while nothing reads it is no `fix`'s (`Sketch::place_axis`)
        places.push((ni, r.a.map(value), r.a.map(|q| r.placed && src.params[q as usize].fixed)));
        axis_map[i] = Some(ni);
        made.push(EntRef::new(EntKind::Axis, ni));
    }
    let mut plane_map: Vec<Option<usize>> = vec![None; src.frames.len()];
    for i in 0..src.frames.len() {
        if let Some(j) = reuse[i] {
            plane_map[i] = Some(j);
            continue;
        }
        if !keep(EntRef::frame(i)) {
            continue;
        }
        // a frame declared bare comes as it is, owning nothing (#383)
        let Some(p) = src.frames[i].chart.as_ref() else {
            dst.frames.push(crate::model::FrameE { class: src.frames[i].class.clone(),
                                                   ..crate::model::FrameE::bare() });
            plane_map[i] = Some(dst.frames.len() - 1);
            made.push(EntRef::frame(dst.frames.len() - 1));
            continue;
        };
        let (Some(u), Some(v), Some(origin)) =
            (axis_map[p.u as usize], axis_map[p.v as usize], pt_index(p.origin as usize))
        else {
            continue;
        };
        let ni = dst.plane_restored(u, v, p.o.map(|k| src.params[k as usize].value),
                                    p.o.map(|k| src.params[k as usize].fixed), origin,
                                    src.frames[i].class.clone());
        if !p.planar {
            dst.spatial_chart(ni);
        }
        plane_map[i] = Some(ni);
        made.push(EntRef::frame(ni));
        // its normal, where it came along too
        if let Some(n) = p.normal.and_then(|n| axis_map[n as usize]) {
            dst.stand_normal(ni, n);
        }
    }
    for (ni, a, held) in places {
        dst.place_restored(ni, a, held);
    }
    // a membership follows its plane across, and a plane that did not come — deleted, or
    // missing a point — takes the memberships that named it with it
    for i in 0..src.points.len() {
        if let (Some(ni), Some(p)) = (pt_index(i), src.plane_of(i)) {
            dst.set_plane(ni, plane_map[p]);
        }
        // and a point drawn in a spatial frame's chart, its three numbers kept (#383)
        if let (Some(ni), Some(f)) = (pt_index(i), src.points[i].chart) {
            if let Some(g) = plane_map[f as usize].filter(|&g| !dst.frames[g].planar()
                && dst.frames[g].chart.is_some() && dst.frames[g].origin as usize != ni)
            {
                dst.draw_in_chart(ni, g);
            }
        }
    }
    // a hidden point comes with the point it lifts, seeded where that point lifts to now
    for l in &src.lifts {
        if let Some(ni) = pt_index(l.point as usize) {
            dst.lift_point(ni);
        }
    }
    // curves last: a curve's arguments may be of any other kind, so every map it reads has to
    // be filled before this one is built.  The *definition* travels with it — a document that
    // came apart from the curve family it is written in would be a document that cannot be drawn.
    let mut curve_map: Vec<Option<usize>> = vec![None; src.curves.len()];
    for (i, cv) in src.curves.iter().enumerate() {
        let mut args = Vec::with_capacity(cv.args.len());
        let mut whole = true;
        for &a in &cv.args {
            match remap_early(&pt_index, &line_map, &circle_map, &arc_map, &spline_map,
                              &plane_map, a) {
                Some(r) => args.push(r),
                None => whole = false,
            }
        }
        // a trim follows its curve and both its points, or goes with them (it is minted after
        // the curve it trims, so that curve's place is already known)
        let trim = match cv.trim {
            None => None,
            Some(t) => match (curve_map.get(t.of as usize).copied().flatten(), pt_index(t.from as usize), pt_index(t.to as usize)) {
                (Some(of), Some(from), Some(to)) => Some(crate::model::Trim { of: of as u32, from: from as u32, to: to as u32 }),
                _ => { whole = false; None }
            },
        };
        if !whole || !keep(EntRef::new(EntKind::Curve, i)) {
            continue;
        }
        let def = &src.curve_defs[cv.def as usize];
        // a followed point's lowering is the whole drawing's (#398 A): it travels unlowered, as
        // drawn, until the document it lands in is elaborated again
        let lowered = matches!(def.body, crate::model::CurveBody::Followed(Some(_)));
        let def = match lowered {
            true => crate::model::CurveDef {
                name: format!("followed:{}", def.param),
                columns: Vec::new(),
                values: Vec::new(),
                vars: Vec::new(),
                body: crate::model::CurveBody::Followed(None),
                ..def.clone()
            },
            false => def.clone(),
        };
        let at = match dst.curve_defs.iter().position(|d| d.name == def.name) {
            Some(k) => k,
            None => {
                dst.curve_defs.push(def);
                dst.curve_defs.len() - 1
            }
        };
        // the home pose follows the entities it is read from, and is whole or nothing: a
        // pose with a hole in it is no pose, and the seeds stand in
        let pose: Vec<_> = cv
            .pose
            .iter()
            .filter_map(|&(e, j)| {
                remap_early(&pt_index, &line_map, &circle_map, &arc_map, &spline_map,
                            &plane_map, e)
                    .map(|r| (r, j))
            })
            .collect();
        // a lowered followed point carries none of what it was lowered to
        let (unknowns, values, pose) = match lowered {
            true => Default::default(),
            false => {
                (cv.unknowns.clone(), cv.values.clone(), crate::model::whole(pose, cv.pose.len()))
            }
        };
        // a free curve's length is its own number, and comes as a circle's radius does; its
        // pegs are the energy's to settle again
        let length = cv.length.map(|l| {
            let p = &src.params[l as usize];
            dst.param(p.value, p.fixed, &p.name) as u32
        });
        dst.curves.push(crate::model::CurveE {
            def: at as u32,
            args,
            unknowns,
            values,
            over: cv.over.clone(),
            home: cv.home.clone(),
            pose,
            class: cv.class.clone(),
            trim,
            extrusion: cv.extrusion,
            length,
            pegs: Vec::new(),
            slides: Vec::new(),
        });
        curve_map[i] = Some(dst.curves.len() - 1);
        made.push(EntRef::new(EntKind::Curve, dst.curves.len() - 1));
    }
    // Faces must retain every edge. Solids must retain their geometry and every referenced
    // solid; allocate the surviving graph before remapping its possibly forward references.
    let mut face_map: Vec<Option<usize>> = vec![None; src.faces.len()];
    for (i, f) in src.faces.iter().enumerate() {
        let profile_plane = f.plane;
        if !keep(EntRef::face(i)) {
            continue;
        }
        let mut edges = Vec::with_capacity(f.edges.len());
        let mut whole = true;
        for &e in &f.edges {
            match remap_early(&pt_index, &line_map, &circle_map, &arc_map, &spline_map,
                              &plane_map, e) {
                Some(r) => edges.push(r),
                None => whole = false,
            }
        }
        let holes = f.holes.iter().map(|h| {
            let edges = h.edges.iter().filter_map(|&e| {
                let r = remap_early(&pt_index, &line_map, &circle_map, &arc_map, &spline_map, &plane_map, e);
                if r.is_none() { whole = false; }
                r
            }).collect();
            crate::model::FaceLoop { edges, edge_names: h.edge_names.clone() }
        }).collect();
        let plane = match profile_plane {
            Some(p) => match plane_map[p as usize] {
                Some(n) => Some(n as u32),
                None => {
                    whole = false;
                    None
                }
            },
            None => None,
        };
        if !whole {
            continue;
        }
        dst.faces.push(crate::model::FaceE {
            edges,
            edge_names: f.edge_names.clone(),
            holes,
            plane,
            name: f.name.clone(),
            class: f.class.clone(),
        });
        face_map[i] = Some(dst.faces.len() - 1);
        made.push(EntRef::face(dst.faces.len() - 1));
    }
    let mut retained: Vec<bool> = src.motions.iter().enumerate().map(|(i,m)| {
        // a motion goes with what it is written over, as a constraint goes with its entities
        keep(EntRef::new(EntKind::Motion,i)) && match m.def {
            crate::model::MotionDef::Rotation {axis,..} | crate::model::MotionDef::Translation {axis,..} =>
                line_map[axis as usize].is_some(),
            crate::model::MotionDef::Turn {centre,..} => pt_index(centre as usize).is_some(),
            crate::model::MotionDef::Relative {..} | crate::model::MotionDef::Inverse {..}
                | crate::model::MotionDef::Driven {..} => true,
            crate::model::MotionDef::Joint {axis,..} => axis_map[axis as usize].is_some(),
            // a frame along a path is read off the whole drawing's path, and a frame the drawing
            // carries off the whole drawing's march: a graft carries neither
            crate::model::MotionDef::Guided {..} | crate::model::MotionDef::Followed {..} => false,
        }
    }).collect();
    loop {
        let mut changed = false;
        for (i,m) in src.motions.iter().enumerate() {
            let needs: &[u32] = match m.def {
                crate::model::MotionDef::Relative {ref source,ref observer} => &[*source,*observer],
                crate::model::MotionDef::Inverse {ref of} | crate::model::MotionDef::Driven {ref of,..} =>
                    std::slice::from_ref(of),
                _ => &[],
            };
            if retained[i] && needs.iter().any(|&k| !retained[k as usize]) {
                retained[i] = false; changed = true;
            }
        }
        if !changed { break; }
    }
    let mut next_motion = dst.motions.len();
    let motion_map: Vec<_> = retained.iter().map(|&keep| {
        if keep { let i = next_motion; next_motion += 1; Some(i) } else { None }
    }).collect();
    for (i,m) in src.motions.iter().enumerate() {
        let Some(next) = motion_map[i] else { continue; };
        let def = match m.def {
            crate::model::MotionDef::Rotation {axis,ratio,advance} =>
                crate::model::MotionDef::Rotation {
                    axis:line_map[axis as usize].unwrap() as u32,ratio,advance,
                },
            crate::model::MotionDef::Translation {axis,advance} =>
                crate::model::MotionDef::Translation {
                    axis:line_map[axis as usize].unwrap() as u32,advance,
                },
            crate::model::MotionDef::Turn {centre,ratio} =>
                crate::model::MotionDef::Turn {
                    centre:pt_index(centre as usize).unwrap() as u32,ratio,
                },
            crate::model::MotionDef::Relative {source,observer} =>
                crate::model::MotionDef::Relative {
                    source:motion_map[source as usize].unwrap() as u32,
                    observer:motion_map[observer as usize].unwrap() as u32,
                },
            crate::model::MotionDef::Inverse {of} =>
                crate::model::MotionDef::Inverse {of:motion_map[of as usize].unwrap() as u32},
            crate::model::MotionDef::Joint {axis,ratio,advance} =>
                crate::model::MotionDef::Joint {axis:axis_map[axis as usize].unwrap() as u32,
                    ratio,advance},
            // its law reads the numbers it was written over by name, as a dimension's text does
            crate::model::MotionDef::Driven {of,ref law,ref over} =>
                crate::model::MotionDef::Driven {of:motion_map[of as usize].unwrap() as u32,
                    law:law.clone(),over:over.clone()},
            crate::model::MotionDef::Guided {..} | crate::model::MotionDef::Followed {..} =>
                unreachable!("a graft carries no guided or carried frame"),
        };
        // a joint's numbers the solve finds are the frame's, renumbered with its joint below
        dst.motions.push(crate::model::MotionE {def,name:m.name.clone(),class:m.class.clone(),
                                                numbers:None});
        made.push(EntRef::new(EntKind::Motion,next));
    }
    // each frame's joint (#383): its axis, its two numbers, and the node it was lowered to
    for (i, f) in src.frames.iter().enumerate() {
        let (Some(j), Some(ni)) = (f.joint.as_ref(), plane_map[i]) else { continue };
        if reuse[i].is_some() || dst.frames[ni].joint.is_some() {
            continue;
        }
        let Some(axis) = axis_map[j.axis as usize] else { continue };
        let number = |dst: &mut Sketch, q: u32| {
            let p = &src.params[q as usize];
            dst.param(p.value, p.fixed, &p.name) as u32
        };
        let (ratio, advance) = (number(dst, j.ratio), number(dst, j.advance));
        let motion = j.motion.and_then(|m| motion_map[m as usize]).map(|m| m as u32);
        dst.frames[ni].joint =
            Some(crate::model::Joint { axis: axis as u32, ratio, advance, motion });
        if let (Some(m), Some(n)) = (motion, j.motion.map(|m| &src.motions[m as usize])) {
            dst.motions[m as usize].numbers = n.numbers.map(|_| [ratio, advance]);
        }
    }
    // a face family's station as copied: its drawn geometry, `None` where some was left behind
    let station_ent = |e: EntRef| -> Option<EntRef> {
        let i = match e.kind {
            EntKind::Point => pt_map[e.i()],
            EntKind::Line => line_map[e.i()],
            EntKind::Circle => circle_map[e.i()],
            EntKind::Arc => arc_map[e.i()],
            EntKind::Spline => spline_map[e.i()],
            EntKind::Axis => axis_map[e.i()],
            EntKind::Frame => plane_map[e.i()],
            EntKind::Curve => curve_map[e.i()],
            EntKind::Face => face_map[e.i()],
            _ => None,
        };
        i.map(|i| EntRef::new(e.kind, i))
    };
    // Allocate the retained graph before remapping it: extent targets and Boolean operands
    // may be forward references, including a cutter referring to the body that subtracts it.
    let mut retained: Vec<bool> = src.solids.iter().enumerate().map(|(i, s)| {
        keep(EntRef::solid(i)) && s.face().is_none_or(|f| face_map[f as usize].is_some())
            && match s.def {
                crate::model::SolidDef::Placed { motion, .. } | crate::model::SolidDef::Swept { motion, .. } =>
                    motion_map[motion as usize].is_some(),
                crate::model::SolidDef::Revolve { axis, .. } => match axis.kind {
                    EntKind::Line => line_map[axis.i()].is_some(),
                    EntKind::Axis => axis_map[axis.i()].is_some(),
                    _ => false,
                },
                // a family comes with its whole station or not at all
                crate::model::SolidDef::Family { face, ref station, ref legs, .. } =>
                    face_map[face as usize].is_some()
                        && legs.iter().all(|l| face_map[l.face as usize].is_some())
                        && station.iter().all(|&e| station_ent(e).is_some()),
                _ => true,
            }
    }).collect();
    loop {
        let mut changed = false;
        for (i, s) in src.solids.iter().enumerate() {
            let required = match s.def {
                crate::model::SolidDef::Through { body, .. } => vec![body],
                crate::model::SolidDef::Swept { source, roll: crate::model::Roll::Through { body }, .. } =>
                    vec![source, body],
                _ => s.operands(),
            };
            if retained[i] && required.iter().any(|&r| !retained[r as usize]) {
                retained[i] = false; changed = true;
            }
        }
        if !changed { break; }
    }
    let mut next_solid = dst.solids.len();
    let solid_map: Vec<Option<usize>> = retained.iter().map(|&keep| {
        if keep { let i = next_solid; next_solid += 1; Some(i) } else { None }
    }).collect();
    for (i, so) in src.solids.iter().enumerate() {
        if solid_map[i].is_none() {
            continue;
        }
        // Retention above guarantees every reference exists. Do not silently discard an
        // operand here: that would change the solid and invalidate the preallocated indices.
        let face = |f: u32| face_map[f as usize].unwrap() as u32;
        let sol = |s: &u32| solid_map[*s as usize].unwrap() as u32;
        let def = match &so.def {
            crate::model::SolidDef::Swept { source, motion, roll } => {
                crate::model::SolidDef::Swept {source:sol(source),
                    motion:motion_map[*motion as usize].unwrap() as u32,roll:match roll {
                        crate::model::Roll::Through { body } => crate::model::Roll::Through { body: sol(body) },
                        stated => stated.clone(),
                    }}
            }
            crate::model::SolidDef::Placed { source, motion, at } => {
                crate::model::SolidDef::Placed { source:sol(source),
                    motion:motion_map[*motion as usize].unwrap() as u32,at:at.clone() }
            }
            crate::model::SolidDef::Prism { face: f, from, to, draft } => {
                crate::model::SolidDef::Prism { face: face(*f), from: from.clone(), to: to.clone(), draft: draft.clone() }
            }
            crate::model::SolidDef::Family { face: f, station, over, legs } => {
                let station = station.iter()
                    .map(|&e| station_ent(e).expect("retention kept the whole station"))
                    .collect();
                let legs = legs.iter().map(|l| crate::model::FamilyLeg {
                    face: face(l.face),
                    ents: l.ents.iter()
                        .map(|&e| station_ent(e).expect("retention kept the whole station"))
                        .collect(),
                    ..l.clone()
                }).collect();
                crate::model::SolidDef::Family { face: face(*f), station, over: over.clone(), legs }
            }
            crate::model::SolidDef::Through { face: f, body } => {
                crate::model::SolidDef::Through { face: face(*f), body: sol(body) }
            }
            crate::model::SolidDef::Revolve { face: f, axis, sweep, sense } => {
                crate::model::SolidDef::Revolve {
                    face: face(*f), axis: match axis.kind {
                        EntKind::Line => EntRef::line(line_map[axis.i()].unwrap()),
                        _ => EntRef::new(EntKind::Axis, axis_map[axis.i()].unwrap()),
                    },
                    sweep: sweep.clone(), sense: *sense,
                }
            }
            crate::model::SolidDef::Body { stock, on, through, bound } => {
                crate::model::SolidDef::Body {
                    stock: sol(stock), on: on.iter().map(sol).collect(),
                    through: through.iter().map(sol).collect(),
                    bound: bound.iter().map(sol).collect(),
                }
            }
            // a region's probe and the entities its terms read, which retention kept with it
            crate::model::SolidDef::Region { probe, terms } => {
                let ent = |e: EntRef| match e.kind {
                    EntKind::Point => pt_index(e.i()).map(EntRef::point),
                    EntKind::Line => line_map[e.i()].map(EntRef::line),
                    EntKind::Frame => plane_map[e.i()].map(EntRef::frame),
                    _ => None,
                };
                crate::model::SolidDef::Region {
                    probe: pt_index(*probe as usize).expect("a region's probe comes with it")
                        as u32,
                    terms: terms.iter().map(|t| crate::model::RegionTerm {
                        shape: t.shape.remapped(&ent).expect("a region's terms come with it"),
                        ..t.clone()
                    }).collect(),
                }
            }
        };
        dst.solids.push(crate::model::SolidE {
            def,
            name: so.name.clone(),
            class: so.class.clone(),
        });
        debug_assert_eq!(solid_map[i], Some(dst.solids.len() - 1));
        made.push(EntRef::solid(dst.solids.len() - 1));
    }
    let mut surface_map = vec![None;src.surfaces.len()];
    for (i,s) in src.surfaces.iter().enumerate() {
        if !keep(EntRef::new(EntKind::Surface,i)) { continue; }
        let Some(solid) = solid_map[s.solid as usize] else { continue };
        let edge = match s.edge.kind {
            EntKind::Line => line_map[s.edge.i()].map(EntRef::line),
            EntKind::Arc => arc_map[s.edge.i()].map(EntRef::arc),
            EntKind::Circle => circle_map[s.edge.i()].map(EntRef::circle),
            _ => None,
        };
        let Some(edge) = edge else { continue };
        let next = dst.surfaces.len();
        dst.surfaces.push(crate::model::SurfaceE {
            solid:solid as u32,edge,span:s.span,name:s.name.clone(),class:s.class.clone(),
        });
        surface_map[i] = Some(next);
        made.push(EntRef::new(EntKind::Surface,next));
    }
    // a drawn entity, renumbered: a point, a line, a circle or an arc
    let drawn_ent = |e: EntRef| -> Option<EntRef> {
        match e.kind {
            EntKind::Point => pt_index(e.i()).map(EntRef::point),
            EntKind::Line => line_map[e.i()].map(EntRef::line),
            EntKind::Circle => circle_map[e.i()].map(EntRef::circle),
            EntKind::Arc => arc_map[e.i()].map(EntRef::arc),
            _ => None,
        }
    };
    // what the solve reads of a source, in the copy's own geometry; a part left behind leaves
    // the copy with nothing it can read, said where a relation names it
    let remap_held = |h: &crate::model::Held| -> crate::model::Held {
        let line = |l: u32| line_map[l as usize].map(|x| x as u32);
        let gone = || crate::model::Held::Not("its source was left behind when it was copied".into());
        match h {
            crate::model::Held::Not(why) => crate::model::Held::Not(why.clone()),
            crate::model::Held::Revolution {axis,edge} => {
                let edge = drawn_ent(*edge);
                let axis = match axis.kind {
                    EntKind::Line => line_map[axis.i()].map(EntRef::line),
                    EntKind::Axis => axis_map[axis.i()].map(EntRef::axis),
                    _ => None,
                };
                match (axis,edge) {
                    (Some(axis),Some(edge)) => crate::model::Held::Revolution {axis,edge},
                    _ => gone(),
                }
            }
            crate::model::Held::Set {probe,shape} => {
                use crate::model::SetShape;
                let shape = shape.as_ref()
                    .map(|s| {
                        let origin = match s.origin {
                            Some(o) => Some(pt_map[o as usize].ok_or(())? as u32),
                            None => None,
                        };
                        line(s.axis).map(|axis| SetShape {axis,origin,..s.clone()}).ok_or(())
                    })
                    .transpose();
                match (pt_map[*probe as usize],shape) {
                    (Some(probe),Ok(shape)) => crate::model::Held::Set {probe:probe as u32,shape},
                    _ => gone(),
                }
            }
        }
    };
    let mut envelope_map = vec![None;src.envelopes.len()];
    for (i,e) in src.envelopes.iter().enumerate() {
        if !keep(EntRef::new(EntKind::Envelope,i)) { continue; }
        let surface = match e.surface {
            Some(s) => match surface_map[s as usize] { Some(s) => Some(s as u32), None => continue },
            None => None,
        };
        let Some(motion) = motion_map[e.motion as usize] else { continue; };
        let held = remap_held(&e.held);
        let next = dst.envelopes.len();
        dst.envelopes.push(crate::model::EnvelopeE {
            surface,motion:motion as u32,roll:e.roll.clone(),
            name:e.name.clone(),class:e.class.clone(),held,
        });
        envelope_map[i] = Some(next);
        made.push(EntRef::new(EntKind::Envelope,next));
    }
    // a traced body, where everything its component was given came along (#326)
    let mut traced_map = vec![None;src.traced.len()];
    for (i,t) in src.traced.iter().enumerate() {
        if !keep(EntRef::new(EntKind::Traced,i)) { continue; }
        let args: Option<Vec<EntRef>> = t.args.iter().map(|&e| match e.kind {
            EntKind::Point => pt_index(e.i()).map(EntRef::point),
            EntKind::Line => line_map[e.i()].map(EntRef::line),
            EntKind::Motion => motion_map[e.i()].map(|m| EntRef::new(EntKind::Motion,m)),
            // the curves a marched body's Cauchy data is read off (#376)
            EntKind::Spline => spline_map[e.i()].map(EntRef::spline),
            EntKind::Curve => curve_map[e.i()].map(|c| EntRef::new(EntKind::Curve,c)),
            EntKind::Traced => traced_map[e.i()].map(|t| EntRef::new(EntKind::Traced,t)),
            _ => None,
        }).collect();
        let Some(args) = args else { continue; };
        let next = dst.traced.len();
        dst.traced.push(crate::model::TracedE {args,..t.clone()});
        traced_map[i] = Some(next);
        made.push(EntRef::new(EntKind::Traced,next));
    }
    // a meshing's flanks, each an envelope copied with it or a source read in the copy's own
    // geometry (#283)
    let mut meshing_map = vec![None;src.meshings.len()];
    for (i,e) in src.meshings.iter().enumerate() {
        if !keep(EntRef::new(EntKind::Meshing,i)) { continue; }
        let Some(motion) = motion_map[e.motion as usize] else { continue; };
        let flank = |f: &crate::model::Flank| match f {
            crate::model::Flank::Envelope(v) => envelope_map[*v as usize].map(|v| crate::model::Flank::Envelope(v as u32)),
            crate::model::Flank::Formed(h) => Some(crate::model::Flank::Formed(remap_held(h))),
            crate::model::Flank::Traced(t) => traced_map[*t as usize].map(|t| crate::model::Flank::Traced(t as u32)),
        };
        let (Some(a),Some(b)) = (flank(&e.flanks[0]),flank(&e.flanks[1])) else { continue; };
        // an ease's point and line, copied with it
        let ease = match &e.ease {
            Some(x) => match (pt_map[x.about as usize],line_map[x.along as usize]) {
                (Some(about),Some(along)) => {
                    Some(crate::model::Ease {about:about as u32,along:along as u32,..x.clone()})
                }
                _ => continue,
            },
            None => None,
        };
        let next = dst.meshings.len();
        dst.meshings.push(crate::model::MeshingE {flanks:[a,b],motion:motion as u32,
            roll:e.roll.clone(),name:e.name.clone(),class:e.class.clone(),ease});
        meshing_map[i] = Some(next);
        made.push(EntRef::new(EntKind::Meshing,next));
    }
    let remap = |e: EntRef| -> Option<EntRef> {
        match e.kind {
            EntKind::Point => pt_index(e.i()).map(EntRef::point),
            EntKind::Line => line_map[e.i()].map(EntRef::line),
            EntKind::Circle => circle_map[e.i()].map(EntRef::circle),
            EntKind::Axis => axis_map[e.i()].map(|i| EntRef::new(EntKind::Axis, i)),
            EntKind::Arc => arc_map[e.i()].map(EntRef::arc),
            EntKind::Spline => spline_map[e.i()].map(EntRef::spline),
            EntKind::Frame => plane_map[e.i()].map(EntRef::frame),
            EntKind::Curve => curve_map[e.i()].map(|i| EntRef::new(EntKind::Curve, i)),
            EntKind::Face => face_map[e.i()].map(EntRef::face),
            EntKind::Solid => solid_map[e.i()].map(EntRef::solid),
            EntKind::Surface => surface_map[e.i()].map(|i| EntRef::new(EntKind::Surface,i)),
            EntKind::Motion => motion_map[e.i()].map(|i| EntRef::new(EntKind::Motion,i)),
            EntKind::Envelope => envelope_map[e.i()].map(|i| EntRef::new(EntKind::Envelope,i)),
            EntKind::Meshing => meshing_map[e.i()].map(|i| EntRef::new(EntKind::Meshing,i)),
            EntKind::Traced => traced_map[e.i()].map(|i| EntRef::new(EntKind::Traced,i)),
        }
    };
    for (&e, &roles) in &src.roles {
        if let Some(target) = remap(e) { dst.roles.insert(target, roles); }
    }
    // each tangency's derivative, while its point and its line came along, with the geometry
    // the use made that did (§6.21)
    let mut dual_map: Vec<Option<usize>> = vec![None; src.duals.len()];
    for (i, d) in src.duals.iter().enumerate() {
        let Some(point) = remap(EntRef::point(d.point)) else { continue };
        let toward = match d.toward {
            crate::model::Toward::Line(l) => match remap(EntRef::line(l)) {
                Some(l) => crate::model::Toward::Line(l.i()),
                None => continue,
            },
            crate::model::Toward::Axis(a) => match remap(EntRef::new(EntKind::Axis, a)) {
                Some(a) => crate::model::Toward::Axis(a.i()),
                None => continue,
            },
            chart => chart,
        };
        let owned = d.owned.iter().filter_map(|&e| remap(e)).collect();
        dual_map[i] = Some(dst.add_dual(point.i(), toward, owned));
    }
    let mut expr = false;
    for c in src.user_constraints() {
        if drop_c.contains(&c.id) {
            continue;
        }
        let mut args = Vec::with_capacity(c.args.len());
        let mut ok = true;
        for a in &c.args {
            match a {
                Arg::Ent(e) => match remap(*e) {
                    Some(n) => args.push(Arg::Ent(n)),
                    None => {
                        ok = false;
                        break;
                    }
                },
                // the destination allocates its own: a Param index is this sketch's name for
                // it, and the pin rides along in the seed — or the name, where the unknown is
                // shared, so the contacts that owned one own one again
                Arg::Param(p) => args.push(src.owned_arg(*p)),
                other => args.push(other.clone()),
            }
        }
        if ok {
            expr |= crate::expr::has_expr(&args);
            // `add_quiet`: the walk evaluates once at the end, not once per constraint
            let mut nc = Constraint::new(c.kind, args);
            nc.claim = c.claim;
            nc.bound = c.bound;
            nc.initially = c.initially;
            nc.class = c.class.clone();
            nc.written = c.written.clone();
            nc.repeated = c.repeated;
            nc.elsewhere = c.elsewhere;
            // the word it was written with, while every operand it named came along (§9.9)
            nc.word = c.word.as_ref().and_then(|w| {
                let ops = w.ops.iter().map(|&e| remap(e)).collect::<Option<Vec<_>>>()?;
                Some(crate::constraints::WordUse { ops, ..w.clone() })
            });
            // a derivative comes with its dual, or not at all (§6.21)
            if let Some(d) = c.along {
                let Some(d) = dual_map[d] else { continue };
                nc.along = Some(d);
            }
            let id = dst.add_quiet(nc);
            if let Some(&place) = src.placements.get(&c.id) {
                dst.placements.insert(id, place);   // a dimension keeps where it was dragged to
            }
        }
    }
    // a plane held along drawn lines (`program::entities::axes_along`) came over as a plane of its
    // own: its axes' rows to the lines, and the point its origin stands at in place of its own
    // `PlaneAxis` rows, are the elaborator's and not a plane's build, so they are carried across
    // as they stand, or the copy would turn free of the lines it was drawn along
    {
        use crate::constraints::CKind;
        let along: BTreeSet<usize> = src.constraints.iter()
            .filter(|c| c.intrinsic && c.kind == CKind::Parallel3)
            .filter(|c| c.args[0].ent().kind == EntKind::Axis)
            .filter(|c| c.args[1].ent().kind == EntKind::Line)
            .map(|c| c.args[0].ent().i())
            .collect();
        let carried = |c: &&Constraint| match c.kind {
            CKind::Parallel3 | CKind::PointOnAxis => c.entities().iter()
                .any(|e| e.kind == EntKind::Axis && along.contains(&e.i())),
            CKind::Coincident3 => src.plane_of_origin(c.args[0].ent().i()).is_some_and(|p| {
                let pl = &src.frames[p];
                along.contains(&(pl.u as usize)) && along.contains(&(pl.v as usize))
            }),
            _ => false,
        };
        let rows: Vec<&Constraint> =
            src.constraints.iter().filter(|c| c.intrinsic).filter(carried).collect();
        for c in rows {
            let Some(ents) = c.entities().into_iter().map(remap).collect::<Option<Vec<_>>>() else {
                continue;
            };
            // a plane standing at a point has no `PlaneAxis` rows of its own for its in-plane
            // axes: they go, as `axes_along` takes them (its normal's stays)
            if c.kind == CKind::Coincident3 {
                if let Some(p) = dst.plane_of_origin(ents[0].i()) {
                    let (u, v) = (dst.frames[p].u as usize, dst.frames[p].v as usize);
                    let gone: Vec<u32> = dst.constraints.iter()
                        .filter(|d| d.intrinsic && d.kind == CKind::PlaneAxis)
                        .filter(|d| d.args[0].ent() == EntRef::frame(p))
                        .filter(|d| [u, v].contains(&d.args[1].ent().i()))
                        .map(|d| d.id)
                        .collect();
                    for id in gone {
                        dst.remove(id);
                    }
                }
            }
            let mut nc = Constraint::new(c.kind, ents.into_iter().map(Arg::Ent).collect());
            nc.intrinsic = true;
            dst.add_quiet(nc);
        }
    }
    // a rate row comes along with both the entities it names (#339 D3)
    for r in &src.rates {
        if let (Some(of), Some(to)) = (remap(r.of), remap(r.to)) {
            dst.rates.push(crate::model::RateRow { of, to, ..r.clone() });
        }
    }
    // a held number that reads the geometry reads the copy of it, where that came along
    for (name, h) in &src.held {
        let reads = h.reads.and_then(|(e, f)| Some((remap(e)?, f)));
        if let Some(d) = dst.held.get_mut(name) {
            d.reads = reads;
        }
    }
    // each twin that came along with its point, tied again after the relations (§6.7)
    for (&p, ts) in &src.twins {
        let Some(p) = remap(EntRef::point(p)) else { continue };
        for &t in ts {
            if let Some(t) = remap(EntRef::point(t)) {
                dst.tie_twin(p.i(), t.i());
            }
        }
    }
    dst.hold_twins();
    if expr {
        expr::evaluate(dst);
    }
    // recorded root choices are keyed by sketch point index; grafting renumbers points, so the
    // keys travel with them.  One naming a point that did not come is dropped — replaying it
    // would apply a chirality to whatever triangle inherited those indices.
    for (k, &v) in &src.branches {
        match decompose::branch_key_points(k) {
            None => {
                if fresh {
                    dst.branches.insert(k.clone(), v);   // not a triangle: nothing to renumber
                }
            }
            Some(pts) => {
                let mapped: Option<Vec<usize>> = pts.iter().map(|&p| pt_index(p)).collect();
                if let Some(m) = mapped {
                    // renumbering can reorder the three, and the sign is read against the key's
                    // own order — so it is re-recorded rather than carried across
                    let (k, v) = decompose::branch_record([m[0], m[1], m[2]], v >= 0);
                    dst.branches.insert(k, v);
                }
            }
        }
    }
    // a ring's turns, where every entity of one came along; a copy whose representative or
    // centre did not stands free where it is
    for tn in &src.turns {
        if let (Some(copy), Some(rep), Some(about)) = (remap(tn.copy), remap(tn.rep), remap(tn.about)) {
            dst.turn(crate::model::Turn { copy, rep, about, ..*tn });
        }
    }
    dst.settle_turns();
    // each free curve's energy, its pegs and its definition, read again from what came (#144)
    dst.settle_variational();
    made
}

/// Copy of the sketch with the given entities/constraints removed, plus everything that depends on
/// a removed entity.  Deletion by rebuild — simple, and keeps `Sketch`'s invariants trivially true.
pub fn without(sk: &Sketch, entities: &[EntRef], constraints: &[u32]) -> Sketch {
    let dead: Vec<EntRef> = entities.to_vec();
    // An entity survives while enough of its children do.  For a line, a circle or an arc that
    // is all of them, which is the old rule; a spline is defined by a list, so losing one
    // control point shortens the curve rather than deleting it.
    let alive = |e: EntRef| {
        if dead.contains(&e) {
            return false;
        }
        // a plane's origin is the plane's, and goes with it
        if e.kind == EntKind::Point {
            if let Some(pl) = sk.plane_of_origin(e.i()) {
                if dead.contains(&EntRef::frame(pl)) {
                    return false;
                }
            }
        }
        let kids = sk.children(e);
        kids.iter().filter(|c| !dead.contains(c)).count() >= sk.min_children(e, &kids)
    };
    let mut tmp = Sketch::new();
    graft(&mut tmp, sk, &alive, constraints, (0.0, 0.0));
    tmp
}

/// The selection as a sketch of its own: every entity picked, the points that define it, and
/// every constraint all of whose entities came along.  Copying is keeping, which is deleting
/// everything else — so it goes through the same rule, and a constraint that survives a copy is
/// exactly one that would have survived deleting the rest.
pub fn copy(sk: &Sketch, entities: &[EntRef]) -> Sketch {
    let mut keep = BTreeSet::new();
    let mut pending = entities.to_vec();
    while let Some(e) = pending.pop() {
        if !keep.insert(e) { continue; }
        pending.extend(sk.children(e));
        if e.kind == EntKind::Point {
            if let Some(p) = sk.plane_of(e.i()) { pending.push(EntRef::frame(p)); }
        }
    }
    let mut candidates = sk.primitives();
    for kind in [EntKind::Curve,EntKind::Face,EntKind::Solid,EntKind::Surface,EntKind::Motion,EntKind::Envelope,EntKind::Meshing] {
        candidates.extend((0..sk.count(kind)).map(|i| EntRef::new(kind,i)));
    }
    let drop: Vec<EntRef> = candidates.into_iter().filter(|e| !keep.contains(e)).collect();
    without(sk, &drop, &[])
}

/// Add everything in `clip` to `sk`, moved by (dx, dy), and return what that made.  The pasted
/// geometry brings its own constraints and nothing else: it is not joined to what is already
/// there, so it can be put where it belongs before being tied down.
/// Paste a figure in, **converting it** where the two documents are in different units.
///
/// A figure copied out of a drawing in inches and pasted into one in millimetres is the same
/// figure, and 2 inches is 50.8 mm: converting is what keeps it so.  The scaling is on a copy of
/// the clipboard, so a clipboard may be pasted into any number of documents and is unchanged by
/// each.  A document that says nothing about its unit is in millimetres, and is converted as
/// one; two in the same unit are not scaled.
pub fn paste(sk: &mut Sketch, clip: &Sketch, dx: f64, dy: f64) -> Vec<EntRef> {
    match (clip.units.mm_per(), sk.units.mm_per()) {
        (from, to) if from != to => {
            let mut scaled = clip.clone();
            scaled.rescale(from / to);
            graft(sk, &scaled, &|_| true, &[], (dx, dy))
        }
        _ => graft(sk, clip, &|_| true, &[], (dx, dy)),
    }
}

/// The part of a sketch one gesture can move, as a sketch of its own.
///
/// Dragging a point can only move what is connected to it: the entities it belongs to, what
/// shares their points, what a constraint ties to those — and nothing past a fixed entity,
/// which does not move whatever is done on its other side.  Everything else in the document is
/// unaffected by the drag, so it is not worth compiling, decomposing or solving for it; a drag
/// on a sketch of many separate figures costs what the one figure costs.
///
/// The part is made by the same rebuild walk as deletion, copying and pasting (`graft`), so it is
/// an ordinary sketch: the plan and the systems are built on it unchanged, and the gesture writes
/// the moved parameters back through `write_back`.  Point indices differ between the two, so the
/// things a drag exchanges with its caller by point index — guard triangles, recorded flips,
/// branch keys — go through `point_in` / `point_out`.
pub struct Part {
    pub sketch: Sketch,
    /// part point index → document point index
    to_doc: Vec<usize>,
    /// document point index → part point index, where it came along
    to_part: Vec<Option<usize>>,
    /// (part param, document param) for every parameter of the part
    params: Vec<(usize, usize)>,
    /// document entity → part entity, for every primitive, curve and face that came along
    ents: BTreeMap<EntRef, EntRef>,
}

impl Part {
    /// The part around `seed`.  A fixed entity is a wall: it comes along (a constraint naming
    /// it must keep all of its entities) but nothing is reached through it.
    pub fn around(sk: &Sketch, seed: EntRef) -> Part {
        Part::around_all(sk, &[seed])
    }

    /// The part around every one of `seeds` — a face family's station, from its face's edges
    /// (`solid::family`); a face among them comes along when its edges do.
    pub fn around_all(sk: &Sketch, seeds: &[EntRef]) -> Part {
        let prims = sk.primitives();
        // who contains a point, and which constraints name an entity
        let mut parents: Vec<Vec<EntRef>> = vec![Vec::new(); sk.points.len()];
        // a curve is no primitive, but it moves with the points it is written over — a free
        // curve's ends among them (#144)
        let curves = (0..sk.curves.len()).map(|i| EntRef::new(EntKind::Curve, i));
        for e in prims.iter().copied().chain(curves) {
            for c in sk.children(e).into_iter().filter(|c| c.kind == EntKind::Point) {
                parents[c.i()].push(e);
            }
        }
        // Which constraints name an entity, and — because two dimensions written in terms of the
        // same free variable share an unknown, which is as real a tie as a shared point — which
        // of them read each free variable.  The walk follows both, so a part that reaches one of
        // a tied group reaches all of it.
        let mut named: BTreeMap<EntRef, Vec<usize>> = BTreeMap::new();
        let mut by_free: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
        // A curve written over a number its drawn instance left unknown (a rod's length) has that
        // unknown as a column, so it is tied to the dimensions reading it just as they are to
        // each other, and a contact on the curve may move it.
        let free_param = |name: &String| sk.free_vars.get(name).copied();
        let mut curves_by_free: BTreeMap<u32, Vec<EntRef>> = BTreeMap::new();
        for (i, cv) in sk.curves.iter().enumerate() {
            for f in cv.unknowns.iter().filter_map(free_param) {
                curves_by_free.entry(f).or_default().push(EntRef::new(EntKind::Curve, i));
            }
        }
        for (ci, c) in sk.constraints.iter().enumerate() {
            // a claim constrains nothing, so it welds nothing: two figures a claim spans stay
            // two parts, and a drag of one costs the other nothing — nor does a bound (§9.6)
            if !c.states_rows() {
                continue;
            }
            for e in c.entities() {
                named.entry(e).or_default().push(ci);
            }
            if let Some(f) = c.free {
                by_free.entry(f.param).or_default().push(ci);
            }
        }
        // a ring's turned copy moves with its representative and its centre, and they with it
        let mut turned: BTreeMap<EntRef, Vec<EntRef>> = BTreeMap::new();
        for tn in &sk.turns {
            for (a, b) in [(tn.copy, tn.rep), (tn.copy, tn.about)] {
                turned.entry(a).or_default().push(b);
                turned.entry(b).or_default().push(a);
            }
        }
        // a plane is a wall when it cannot move: its axes held as well as where it stands
        let wall = |e: EntRef| match e.kind {
            EntKind::Frame => sk.plane_fixed(e.i()),
            // a plane's origin is held at its own (0, 0) and stands wherever the plane does: what
            // is tied to it moves with a plane that moves
            EntKind::Point if let Some(pl) = sk.plane_of_origin(e.i()) => sk.plane_fixed(pl),
            // a motion owns no number, and moves with what it is written over: its axes (#366)
            EntKind::Motion => false,
            _ => sk.entity_params(e).iter().all(|&p| sk.params[p as usize].fixed),
        };
        let mut keep: BTreeSet<EntRef> = BTreeSet::new();
        let mut followed: BTreeSet<u32> = BTreeSet::new();
        let mut queue = seeds.to_vec();
        keep.extend(seeds.iter().copied());
        // a tied group — the dimensions and curves reading one free variable — only has to be
        // opened once, however many of its members the walk arrives at
        let mut open = |f: u32, next: &mut Vec<EntRef>| {
            if followed.insert(f) {
                for &cj in by_free.get(&f).map(|v| v.as_slice()).unwrap_or(&[]) {
                    next.extend(sk.constraints[cj].entities());
                }
                next.extend(curves_by_free.get(&f).map(|v| v.as_slice()).unwrap_or(&[]));
            }
        };
        while let Some(e) = queue.pop() {
            // a point comes with the plane it is drawn in, which places it in space — a held
            // point too, which is drawn there all the same
            if let Some(pl) = (e.kind == EntKind::Point).then(|| sk.plane_of(e.i())).flatten() {
                if keep.insert(EntRef::frame(pl)) {
                    queue.push(EntRef::frame(pl));
                }
            }
            if wall(e) {
                // a held entity is followed no further, but it is rebuilt over what it is made of
                // — a held spline over its control points (#134) — and those are held with it, so
                // they stop here in turn, each with its plane.  A plane's own are kept below.  A
                // body traced in space owns no parameter and is read over what its component was
                // given, so a station standing on one keeps that too (#363 S1).
                let made_of = !sk.entity_params(e).is_empty() || e.kind == EntKind::Traced;
                if e.kind != EntKind::Frame && made_of {
                    for c in sk.children(e) {
                        if keep.insert(c) {
                            queue.push(c);
                        }
                    }
                }
                continue;
            }
            let mut next = sk.children(e);
            if e.kind == EntKind::Point {
                next.extend(parents[e.i()].iter().copied());
            }
            next.extend(turned.get(&e).map(|v| v.as_slice()).unwrap_or(&[]));
            if e.kind == EntKind::Curve {
                for f in sk.curves[e.i()].unknowns.iter().filter_map(free_param) {
                    open(f, &mut next);
                }
            }
            for &ci in named.get(&e).map(|v| v.as_slice()).unwrap_or(&[]) {
                next.extend(sk.constraints[ci].entities());
                if let Some(f) = sk.constraints[ci].free {
                    open(f.param, &mut next);
                }
            }
            for n in next {
                if keep.insert(n) {
                    queue.push(n);
                }
            }
        }
        // a plane is rebuilt over its axes and its origin, wherever the walk stopped at it
        for i in 0..sk.frames.len() {
            if keep.contains(&EntRef::frame(i)) {
                keep.extend(sk.children(EntRef::frame(i)));
            }
        }
        let mut sketch = Sketch::new();
        let mut made = graft(&mut sketch, sk, &|e| keep.contains(&e), &[], (0.0, 0.0));
        // `graft` makes entities kind by kind in document order, which is `primitives` order,
        // then the curves, paired below
        let srcs: Vec<EntRef> = prims.into_iter().filter(|e| keep.contains(e)).collect();
        made.truncate(srcs.len());
        debug_assert!(srcs.iter().zip(&made).all(|(s, m)| s.kind == m.kind));
        let mut to_doc = Vec::new();
        let mut to_part = vec![None; sk.points.len()];
        let mut params = Vec::new();
        for (&s, &m) in srcs.iter().zip(&made) {
            if s.kind == EntKind::Point {
                to_part[s.i()] = Some(m.i());
                to_doc.push(s.i());
            }
            for (a, b) in sketch.entity_params(m).into_iter().zip(sk.entity_params(s)) {
                params.push((a as usize, b as usize));
            }
        }
        // a free curve's length is its own number (#144): the curves come after the primitives,
        // the ones kept in document order
        let mut ent: BTreeMap<EntRef, EntRef> = srcs.iter().copied().zip(made.iter().copied()).collect();
        let kept = (0..sk.curves.len()).filter(|&i| keep.contains(&EntRef::new(EntKind::Curve, i)));
        for (doc, part) in kept.zip(0..sketch.curves.len()) {
            ent.insert(EntRef::new(EntKind::Curve, doc), EntRef::new(EntKind::Curve, part));
            if let (Some(a), Some(b)) = (sketch.curves[part].length, sk.curves[doc].length) {
                params.push((a as usize, b as usize));
            }
        }
        // and a constraint's own unknowns (a contact's place along its curve) with its copy's:
        // the constraint of the same kind over the same entities, each copy paired once
        let mut copies: Vec<Option<(CKind, Vec<EntRef>, Vec<u32>)>> = sketch
            .constraints
            .iter()
            .map(|c| Some((c.kind, c.entities(), c.aux_params())).filter(|(_, _, own)| !own.is_empty()))
            .collect();
        for c in &sk.constraints {
            let own = c.aux_params();
            if own.is_empty() {
                continue;
            }
            let Some(over) = c.entities().iter().map(|e| ent.get(e).copied()).collect::<Option<Vec<_>>>() else {
                continue;
            };
            let found = copies.iter_mut().find(|k| k.as_ref().is_some_and(|(kind, ents, _)| *kind == c.kind && *ents == over));
            if let Some((_, _, theirs)) = found.and_then(Option::take) {
                params.extend(theirs.into_iter().zip(own).map(|(a, b)| (a as usize, b as usize)));
            }
        }
        // and a hidden point moves with the view point it lifts
        for l in &sk.lifts {
            if let Some(k) = to_part[l.point as usize].and_then(|p| sketch.lift_of(p)) {
                for (x, y) in sketch.lifts[k].x.iter().zip(&l.x) {
                    params.push((*x as usize, *y as usize));
                }
            }
        }
        // the free variables came along by name: the rebuild allocated the part's own unknown
        // for each, seeded off the number the dimension carried, so the two are already in step
        for (name, &doc) in &sk.free_vars {
            if let Some(&mine) = sketch.free_vars.get(name) {
                params.push((mine as usize, doc as usize));
            }
        }
        // and the faces among the seeds, which come after the curves, in document order
        let faces = (0..sk.faces.len()).filter(|&i| keep.contains(&EntRef::face(i)));
        for (doc, part) in faces.zip(0..sketch.faces.len()) {
            ent.insert(EntRef::face(doc), EntRef::face(part));
        }
        Part { sketch, to_doc, to_part, params, ents: ent }
    }

    /// A document entity as the part's, if it came along.
    pub fn entity_in(&self, doc: EntRef) -> Option<EntRef> {
        self.ents.get(&doc).copied()
    }

    /// A document point's index in the part, if it came along.
    pub fn point_in(&self, doc: usize) -> Option<usize> {
        self.to_part.get(doc).copied().flatten()
    }

    /// A part point's index in the document.
    pub fn point_out(&self, part: usize) -> usize {
        self.to_doc[part]
    }

    /// A triangle of document points as part points — `None` if any did not come along.
    pub fn triangle_in(&self, t: (usize, usize, usize)) -> Option<(usize, usize, usize)> {
        Some((self.point_in(t.0)?, self.point_in(t.1)?, self.point_in(t.2)?))
    }

    /// The triangles of `ts` that came along, as part points.  One that did not is dropped: it
    /// names geometry this part cannot move, so there is no orientation here for it to guard.
    pub fn triangles_in(&self, ts: &[(usize, usize, usize)]) -> Vec<(usize, usize, usize)> {
        ts.iter().filter_map(|&t| self.triangle_in(t)).collect()
    }

    pub fn triangle_out(&self, t: (usize, usize, usize)) -> (usize, usize, usize) {
        (self.point_out(t.0), self.point_out(t.1), self.point_out(t.2))
    }

    /// Recorded root choices of the part, keyed as the document keys them.
    pub fn branches_out(&self, b: &BTreeMap<String, i32>) -> BTreeMap<String, i32> {
        b.iter()
            .map(|(k, &v)| match decompose::branch_key_points(k) {
                Some(p) => decompose::branch_record(
                    [self.point_out(p[0]), self.point_out(p[1]), self.point_out(p[2])],
                    v >= 0,
                ),
                None => (k.clone(), v),
            })
            .collect()
    }

    /// Copy the part's parameter values back into the document it came from.
    pub fn write_back(&self, sk: &mut Sketch) {
        for &(a, b) in &self.params {
            sk.params[b].value = self.sketch.params[a].value;
        }
        // this is a parameter write of its own — it does not go through `Sketch::set_x` — so the
        // dimensions written in terms of a free variable are brought up to date here too, or a
        // plan drag would leave the constraint list reading the number from before the gesture
        expr::sync_free(sk);
    }
}

/// The same lookup `graft`'s `remap` makes, for the one pass that has to run before it exists:
/// a curve's arguments, which may be of any other kind and so must be resolved once every other
/// kind has been grafted.
#[allow(clippy::too_many_arguments)]
fn remap_early(
    pt_index: &dyn Fn(usize) -> Option<usize>,
    line_map: &[Option<usize>],
    circle_map: &[Option<usize>],
    arc_map: &[Option<usize>],
    spline_map: &[Option<usize>],
    plane_map: &[Option<usize>],
    e: EntRef,
) -> Option<EntRef> {
    match e.kind {
        EntKind::Point => pt_index(e.i()).map(EntRef::point),
        EntKind::Line => line_map[e.i()].map(EntRef::line),
        EntKind::Circle => circle_map[e.i()].map(EntRef::circle),
        EntKind::Arc => arc_map[e.i()].map(EntRef::arc),
        EntKind::Spline => spline_map[e.i()].map(EntRef::spline),
        EntKind::Frame => plane_map[e.i()].map(EntRef::frame),
        // a curve is never another curve's argument: nothing in the language says so, and a
        // face and a solid are built after every curve, so neither is one either
        EntKind::Curve | EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Meshing
        | EntKind::Traced | EntKind::Axis => None,
    }
}
