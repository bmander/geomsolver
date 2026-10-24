//! A mirrored instance's images (#578).
//!
//! The flattener walks a mirrored instance's body once, as it walks any closed instance's: its
//! entities, its *preimage*, are solved by its rows where the body stands unmirrored, in the
//! right-handed frame its call stands in.  What makes it mirrored is here: every point, line,
//! circle, arc and spline the body made gets an **image** — its representative's numbers, some
//! negated, in the image of what it is drawn in (`model::Mirror`) — bound to the key a reference
//! from outside the instance reads it by (`flatten::keys::image`), so a parent, a claim and the
//! drawing read the mirrored figure; and the frame's datums get images too, each a plane on the
//! frame's own axes with the mirrored one reversed, which the image's points are drawn in and
//! `b.std.D` names from outside (`flatten::keys::image_datum`).  The preimage goes by its own
//! names and is not drawn; an image goes by its preimage's.  Its solids are the body's
//! evaluation reflected (`model::Binding::mirror`), and no row is added or taken away.

use super::resolve::Resolver;
use super::rings::called;
use super::source_map::{public_path, SourceMap};
use super::{Code, Diag};
use crate::flatten::InstanceInfo;
use crate::model::{EntKind, EntRef, Mirror, Sketch};
use std::collections::BTreeMap;

/// Each standard datum plane by its basis in its frame's axes: which axis (0 `x`, 1 `y`, 2 `z`)
/// and which way, for its `u` and its `v` — as `std.sv`'s `StandardDatums` and `FrameDatums`
/// write them.
const PLANES: [(&str, [(usize, bool); 2]); 5] = [
    ("top", [(0, true), (1, true)]),
    ("front", [(0, true), (2, true)]),
    ("side", [(1, true), (2, true)]),
    ("behind", [(1, false), (2, true)]),
    ("up", [(2, true), (0, false)]),
];

/// Each standard datum axis by the frame's axis it runs along, and which way.
const AXES: [(&str, (usize, bool)); 4] = [("x", (0, true)), ("y", (1, true)), ("z", (2, true)),
                                          ("back", (0, false))];

fn refuse(diags: &mut Vec<Diag>, span: crate::syntax::Span, message: String) {
    Diag::once(diags, Code::E112, span, message);
}

/// `key` bound to `e`, by no name a reader is shown: what a reference from outside reads.
fn publish(res: &mut Resolver, map: &mut SourceMap, key: String, e: EntRef) {
    map.bind(&key, e, crate::syntax::Named::No);
    res.of.insert(key, e);
}

/// One mirrored instance's frame read for its images: its axes, the plane its image datums stand
/// where (the frame itself, or the world's front), the frame's own datum planes by index, and the
/// axis its image reverses.
struct Mirroring<'m> {
    prefix: &'m str,
    name: String,
    axis: usize,
    /// The frame's `x`, `y` and `z`: a spatial frame's `u`, `v` and normal, or the world's axes.
    axes: [usize; 3],
    /// The frame, `None` for the world.
    frame: Option<usize>,
    /// Where an image datum stands: the frame, or the world's front.
    place: usize,
    /// Each of the frame's datum planes, by index, to its name, and its `std.origin`.
    datums: BTreeMap<usize, &'static str>,
    origin: Option<usize>,
    reversed: BTreeMap<usize, usize>,
    planes: BTreeMap<&'static str, usize>,
}

impl Mirroring<'_> {
    /// The axis along the frame's axis `k`, its way flipped where it is the one the image reverses.
    fn axis(&mut self, sk: &mut Sketch, (k, along): (usize, bool)) -> usize {
        let base = self.axes[k];
        if along != (k == self.axis) {
            return base;
        }
        *self.reversed.entry(base).or_insert_with(|| {
            sk.image_axis(base, &crate::flatten::image_datum(self.prefix, &format!("reversed{k}")))
        })
    }

    /// Datum plane `name`'s image (one of `PLANES`): the plane over its axes' images, where the
    /// frame stands.
    fn plane(&mut self, sk: &mut Sketch, name: &'static str) -> usize {
        if let Some(&p) = self.planes.get(name) {
            return p;
        }
        let (_, [a, b]) = PLANES.iter().find(|(n, _)| *n == name).expect("a datum plane");
        let (u, v) = (self.axis(sk, *a), self.axis(sk, *b));
        // standing where the datum it is the image of does, where that is made
        let of = self.datums.iter().find(|(_, &d)| d == name).map_or(self.place, |(&p, _)| p);
        let p = sk.image_plane(u, v, of, &crate::flatten::image_datum(self.prefix, name));
        self.planes.insert(name, p);
        p
    }
}

/// Every mirrored instance's images — after the holds and the rings' turns, which a preimage's
/// numbers are worked out by first, and before any relation reads an image.
pub(super) fn mirrors(
    sk: &mut Sketch,
    res: &mut Resolver,
    map: &mut SourceMap,
    instances: &[InstanceInfo],
    diags: &mut Vec<Diag>,
) {
    for inst in instances {
        let (Some(m), Some(bound)) = (&inst.mirror, &inst.binding) else { continue };
        let name = public_path(inst.prefix.trim_end_matches('.'));
        let frame = match &bound.frame {
            Some(f) => match res.of.get(f) {
                Some(e) if e.kind == EntKind::Frame && sk.frames[e.i()].chart.is_some() => {
                    Some((e.i(), f.as_str()))
                }
                _ => continue,
            },
            None => None,
        };
        // the frame's own axes, and its datums by index
        let world = |key: &str| res.of.get(key).copied();
        let (axes, place) = match frame {
            Some((f, key)) => {
                let (u, v) = sk.frames[f].chart.as_ref().map(|c| (c.u, c.v)).expect("charted");
                let n = sk.plane_normal(f, &format!("{key}.normal"));
                ([u as usize, v as usize, n], f)
            }
            None => match (world("std.x"), world("std.y"), world("std.z"), world("std.front")) {
                (Some(x), Some(y), Some(z), Some(front)) => ([x.i(), y.i(), z.i()], front.i()),
                _ => {
                    refuse(diags, m.span, format!("`{name}` is mirrored in the world's datums, \
                        which a document has where it says `use std`"));
                    continue;
                }
            },
        };
        let datum_key = |d: &str| match &bound.frame {
            Some(f) => format!("{}{d}", crate::flatten::frame_datums(f)),
            None => format!("std.{d}"),
        };
        let datums = PLANES.iter().filter_map(|(d, _)| {
            let e = res.of.get(&datum_key(d)).filter(|e| e.kind == EntKind::Frame)?;
            Some((e.i(), *d))
        }).collect();
        let mut at = Mirroring {
            prefix: &inst.prefix, name, axis: m.axis as usize, axes, frame: frame.map(|f| f.0),
            place, datums, origin: res.of.get(&datum_key("origin")).map(|e| e.i()),
            reversed: BTreeMap::new(), planes: BTreeMap::new(),
        };
        images(sk, res, map, &mut at, diags);
        // its datums, as `b.std.D` reads them from outside (#548): only where the instance stands
        // in a frame of its own, which nothing else does
        let own = bound.frame.as_deref().and_then(crate::flatten::frame_instance)
            .is_some_and(|i| format!("{i}.") == inst.prefix);
        if own {
            let key = |d: &str| crate::flatten::image_datum(&inst.prefix, d);
            for (d, _) in PLANES {
                let p = at.plane(sk, d);
                publish(res, map, key(d), EntRef::frame(p));
            }
            // its origin, the front's
            let front = at.plane(sk, "front");
            let origin = sk.frames[front].origin;
            publish(res, map, key("origin"), EntRef::point(origin as usize));
            for (d, along) in AXES {
                let e = EntRef::new(EntKind::Axis, at.axis(sk, along));
                publish(res, map, key(d), e);
            }
        }
    }
    sk.settle_turns();
}

/// Every entity mirrored instance `at` made, imaged: points first, then what is drawn over them.
fn images(sk: &mut Sketch, res: &mut Resolver, map: &mut SourceMap, at: &mut Mirroring<'_>,
          diags: &mut Vec<Diag>) {
    // every key under the instance, by the entity it names
    let mut made: BTreeMap<EntRef, Vec<String>> = BTreeMap::new();
    for (key, e) in map.keys() {
        if key.starts_with(at.prefix) {
            made.entry(e).or_default().push(key.clone());
        }
    }
    let mut image: BTreeMap<EntRef, EntRef> = BTreeMap::new();
    // what the body draws over and did not make: its frame's datums' points, held, each imaged as
    // the point at its numbers in its plane's image — the image datum's origin where it stands
    // there (`std.origin`, `std.side.origin`)
    let drawn = [EntKind::Line, EntKind::Circle, EntKind::Arc, EntKind::Spline];
    let origins = at.datums.keys().map(|&d| EntRef::point(sk.frames[d].origin as usize))
        .chain(at.origin.map(EntRef::point));
    let over: Vec<EntRef> = made.keys().filter(|e| drawn.contains(&e.kind))
        .flat_map(|&e| sk.children(e)).chain(origins).filter(|p| !made.contains_key(p)).collect();
    for p in over {
        if image.contains_key(&p) {
            continue;
        }
        let held = sk.point_all_params(p.i()).iter().all(|&k| sk.params[k as usize].fixed);
        let Some(&d) = sk.points[p.i()].plane.filter(|_| held)
            .and_then(|pl| at.datums.get(&(pl as usize)))
        else {
            continue;
        };
        let plane = at.plane(sk, d);
        let (x, y) = sk.point_xy(p.i());
        let copy = match sk.frames[plane].origin as usize {
            o if (x, y) == (0.0, 0.0) => o,
            _ => {
                let c = sk.point(x, y, true, &format!("{}#image", sk.point_label(p.i())));
                sk.points[c].plane = Some(plane as u32);
                c
            }
        };
        // recorded, so what the body draws over it is drawn over the image (`Sketch::images`)
        let copy = EntRef::point(copy);
        sk.mirror(Mirror { copy, rep: p, negated: 0, datum: true });
        image.insert(p, copy);
    }
    let order = [EntKind::Point, EntKind::Line, EntKind::Circle, EntKind::Arc, EntKind::Spline];
    let span = |map: &SourceMap, e| map.site_of(e).map_or(crate::syntax::Span::default(), |s| s.span);
    for e in made.keys().copied().filter(|e| !order.contains(&e.kind)) {
        let why = match e.kind {
            EntKind::Frame | EntKind::Axis => "whose numbers are the world's, so its image is no \
                linear function of them: declare it outside, or draw in the frame's own datums",
            EntKind::Curve => "and a curve is not reflected yet: draw it as a spline over points",
            EntKind::Motion | EntKind::Surface | EntKind::Envelope | EntKind::Meshing
                | EntKind::Traced => "which is not reflected yet",
            // what is evaluated, and published reflected
            _ => continue,
        };
        refuse(diags, span(map, e), format!("`{}` is mirrored, and its body declares {}, {why}",
                                            at.name, called(map, e)));
    }
    for kind in order {
        for &e in made.keys().filter(|e| e.kind == kind) {
            let made = match kind {
                EntKind::Point => point(sk, at, e.i()),
                EntKind::Line => {
                    let l = sk.lines[e.i()].clone();
                    let end = |p: u32| image.get(&EntRef::point(p as usize));
                    match (end(l.p1), end(l.p2)) {
                        (Some(a), Some(b)) => {
                            let i = sk.line(a.i(), b.i());
                            sk.lines[i].class = l.class;
                            Ok(EntRef::line(i))
                        }
                        _ => Err("its ends have no image".to_string()),
                    }
                }
                EntKind::Circle => {
                    let c = sk.circles[e.i()].clone();
                    match image.get(&EntRef::point(c.center as usize)) {
                        Some(o) => {
                            let r = sk.params[c.radius as usize].value;
                            let label = format!("{}#image", sk.point_label(c.center as usize));
                            let i = sk.circle(o.i(), r, &label);
                            sk.circles[i].class = c.class;
                            Ok(EntRef::circle(i))
                        }
                        None => Err("its centre has no image".to_string()),
                    }
                }
                EntKind::Arc => {
                    let a = sk.arcs[e.i()].clone();
                    let ends = [a.center, a.start, a.end]
                        .map(|p| image.get(&EntRef::point(p as usize)).copied());
                    match ends {
                        [Some(c), Some(s), Some(t)] => {
                            let i = sk.arc(c.i(), s.i(), t.i(), "#image");
                            sk.arcs[i].class = a.class;
                            Ok(EntRef::arc(i))
                        }
                        _ => Err("its centre and ends have no image".to_string()),
                    }
                }
                EntKind::Spline => {
                    let s = sk.splines[e.i()].clone();
                    let ctrl: Option<Vec<usize>> = s.ctrl.iter()
                        .map(|&p| image.get(&EntRef::point(p as usize)).map(|q| q.i())).collect();
                    match ctrl.and_then(|c| sk.spline(&c)) {
                        Some(i) => {
                            sk.splines[i].knots = s.knots;
                            sk.splines[i].weights = s.weights;
                            sk.splines[i].class = s.class;
                            Ok(EntRef::spline(i))
                        }
                        None => Err("its control points have no image".to_string()),
                    }
                }
                _ => unreachable!(),
            };
            let copy = match made {
                Ok(copy) => copy,
                Err(why) => {
                    let what = called(map, e);
                    refuse(diags, span(map, e), format!("`{}` is mirrored, and {what} {why}", at.name));
                    continue;
                }
            };
            // a point's numbers were derived as it was made; a round's radius, here, and a line
            // or a spline, which owns none, recorded so its preimage is drawn as it
            if kind != EntKind::Point {
                sk.mirror(Mirror { copy, rep: e, negated: 0, datum: false });
            }
            image.insert(e, copy);
        }
    }
    // each image read from outside by its preimage's every key, and called by its name
    for (e, keys) in &made {
        let Some(&copy) = image.get(e) else { continue };
        for key in keys {
            publish(res, map, crate::flatten::image(key), copy);
        }
        map.image_of.insert(copy, *e);
        map.also_made(*e, copy);
    }
}

/// Point `q`'s image, or why it has none: in its plane's image, where that plane is one of the
/// frame's datums; in the frame itself, its three numbers there; in space, where the frame is
/// the world's.
fn point(sk: &mut Sketch, at: &mut Mirroring<'_>, q: usize) -> Result<EntRef, String> {
    let rep = sk.points[q].clone();
    let (x, y) = sk.point_xy(q);
    let value = |p: u32| sk.params[p as usize].value;
    let label = format!("{}#image", sk.point_label(q));
    let (p, negated) = match (rep.plane, rep.chart, rep.z) {
        (Some(plane), _, _) => {
            let Some(&d) = at.datums.get(&(plane as usize)) else {
                return Err("is drawn in a plane that is none of the datums of the frame it stands \
                    in, which a mirrored body draws in".to_string());
            };
            let image = at.plane(sk, d);
            let p = sk.point(x, y, false, &label);
            sk.points[p].plane = Some(image as u32);
            (p, 0)
        }
        (None, Some(chart), Some(z)) if Some(chart as usize) == at.frame => {
            let p = sk.point3([x, y, value(z)], false, &label);
            sk.draw_in_chart(p, chart as usize);
            (p, 1 << at.axis)
        }
        (None, None, Some(z)) if at.frame.is_none() => {
            (sk.point3([x, y, value(z)], false, &label), 1 << at.axis)
        }
        _ => return Err("stands where its image is no linear function of its numbers: draw it in \
                         the frame's datums, or in the frame".to_string()),
    };
    sk.mirror(Mirror { copy: EntRef::point(p), rep: EntRef::point(q), negated, datum: false });
    Ok(EntRef::point(p))
}
