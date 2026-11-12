//! The spiral-bevel pair laid out step by step (`spiral_bevel/hypoid.sv`: the pitch cones, the
//! trace and the crown built from lines, circles and regions), held to the pair as it was computed
//! with trigonometry before the layout replaced it: every named quantity the layout plan lists
//! (docs/spiral-bevel-layout-plan.md), at the configured hypoid, the bevel pair, the six-millimetre
//! hypoid and the nine sizes and ratios the paired-envelope checks read, and the material both
//! made at the configured design and the bevel.
//!
//! The recorded numbers are that pair's (`tests/fixtures/hypoid_layout.tsv` and
//! `hypoid_layout_material.tsv`), taken when the layout and the pair were compared side by side
//! and agreed to 1e-9: they are the proof that the layout is the pair, so they are not
//! re-recorded from the layout itself.  The exception is every design off the bevel pair: the
//! layout states the shafts and is a true hypoid, which the old pair never was, so the configured
//! and six-millimetre columns are the layout's own (`record`); the bevel pair's and the nine
//! sizes' still hold it to the old pair.  Nothing is compared by name alone: points in space,
//! lines by where they are and which way they run, sections by their meridian coordinates about
//! the cutter's axis, motions by the poses they give.
use gcs_core::{motion,program::Elaborated,solid::MaterialField};

type V = [f64;3];

fn sub(a: V,b: V) -> V { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V) -> f64 { dot(a,a).sqrt() }
fn unit(a: V) -> V { let n = norm(a); a.map(|x| x/n) }
fn dist(a: V,b: V) -> f64 { norm(sub(a,b)) }
/// The unsigned angle between two directions, in degrees.
fn angle(a: V,b: V) -> f64 { norm(cross(a,b)).atan2(dot(a,b)).to_degrees() }
/// The angle between a direction and a line whichever way the line runs, in degrees.
fn slant(a: V,b: V) -> f64 { angle(a,b).min(180.-angle(a,b)) }
/// The distance of `p` from the line through `o` along `d`.
fn off_line(p: V,o: V,d: V) -> f64 { norm(cross(sub(p,o),unit(d))) }

/// A design: the pair's document at it.
struct Design { label: String, text: String }

impl Design {
    /// A design of its own (`fixtures::gear::configuration`).
    fn at(label: String,teeth: [u32;2],module: f64,offset: f64,shift: f64,spiral: f64) -> Self {
        Design { label, text: fixtures::gear::at(teeth,module,offset,shift,spiral) }
    }
    /// The pair (`hypoid.sv`) at this design, solved.
    fn read(&self) -> Elaborated { fixtures::gear::read(&self.text) }
}

/// The recorded designs (`fixtures::gear::designs`): the configured hypoid, the bevel pair, the
/// six-millimetre hypoid and the nine sizes and ratios of `tests/envelope/paired.rs`.
fn designs() -> Vec<Design> {
    fixtures::gear::designs().into_iter().map(|(label,text)| Design { label,text }).collect()
}

fn index(e: &Elaborated,name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}
/// A line's ends in space.
fn line(e: &Elaborated,name: &str) -> (V,V) {
    let l = &e.sketch.lines[index(e,name)];
    (e.sketch.world_point(l.p1 as usize),e.sketch.world_point(l.p2 as usize))
}
/// A point in space, or a line's end where the end is another's point by alias.
fn point(e: &Elaborated,name: &str) -> V {
    if e.map.ent_named(name).is_none() {
        if let Some((l,end)) = name.rsplit_once('.').filter(|(_,end)| ["p1","p2"].contains(end)) {
            let (a,b) = line(e,l);
            return if end == "p1" { a } else { b };
        }
    }
    e.sketch.world_point(index(e,name))
}
/// The motion node of the pair's frame `name` (`crown_roll`, …): its joint's.
fn joint(e: &Elaborated,name: &str) -> usize {
    let f = index(e,&format!("pair.{name}"));
    e.sketch.frames[f].joint.as_ref().and_then(|j| j.motion)
        .unwrap_or_else(|| panic!("`{name}` has no joint")) as usize
}
/// A motion of the pair by the recorded rows' label, and the roll it is read from: a frame's
/// joint at its own roll; a member's generation, the crown's roll seen from the member's (the
/// node `under: crown_roll, seen_from: pinion_roll` minted); the crown's neighbour, the crown's
/// roll read one crown pitch round, where the gear's space places its inner crown (`Space`'s
/// `neighbor`).
fn motion(e: &Elaborated,name: &str) -> (usize,f64) {
    let seen = |member: &str| {
        let (source,observer) = (joint(e,"crown_roll") as u32,joint(e,member) as u32);
        e.sketch.motions.iter().position(|m| matches!(m.def,
            gcs_core::model::MotionDef::Relative {source: s,observer: o}
                if s == source && o == observer))
            .unwrap_or_else(|| panic!("nothing reads the crown's roll seen from `{member}`"))
    };
    match name {
        "pinion_generation" => (seen("pinion_roll"),0.),
        "gear_generation" => (seen("gear_roll"),0.),
        "crown_neighbor" => {
            let s = e.sketch.solids.iter().find(|s| s.name == "pair.gear_space.neighbor")
                .expect("the gear space's neighbour");
            let gcs_core::model::SolidDef::Placed {motion,at,..} = &s.def else {
                panic!("the neighbour is no placed solid")
            };
            assert_eq!(*motion as usize,joint(e,"crown_roll"));
            (*motion as usize,at.value)
        }
        _ => (joint(e,name),0.),
    }
}

/// What the plan's gate compares, read off one elaborated pair: numbers with a name.
struct Reading { values: Vec<(String,f64)> }

impl Reading {
    fn push(&mut self,name: &str,v: f64) { self.values.push((name.into(),v)); }
    fn point(&mut self,name: &str,p: V) {
        for (k,c) in ["x","y","z"].iter().zip(p) { self.push(&format!("{name}.{k}"),c); }
    }
}

/// Each crown region's corners about the cutter's axis (base, joins, tip ends), each where two of
/// its faces meet, named by side, with the faces: the crown tooth's six and the gear's space
/// cutter's, its outer mate's inner flank (closed at the reach's cap) and its inner mate's outer
/// flank (closed at the axis), unturned.  The recorded pair's sections drew both mates whole; the
/// space cutter has the flank of each that cuts.  Then its tip roundings, by the corner its
/// centre is recorded as.
type Corners = [(&'static str,&'static str,&'static str)];
const CROWNS: [(&str,&str,&Corners,&[(&str,&str)]);3] = [
    ("tooth","pair.tooth.crown",&[("bi","inner","base"),("bo","outer","base"),("ij","inner","inner_round"),
        ("oj","outer","outer_round"),("it","inner_round","tip"),("ot","outer_round","tip")],
        &[("ci","inner_round"),("co","outer_round")]),
    ("mate outer","pair.gear_space.outer_crown",&[("bi","inner","base"),("ij","inner","inner_round"),
        ("it","inner_round","tip")],&[("ci","inner_round")]),
    ("mate inner","pair.gear_space.inner_crown",&[("bo","outer","base"),("oj","outer","outer_round"),
        ("ot","outer_round","tip")],&[("co","outer_round")]),
];

/// A region solid's meridian, its steps in space: each step's ends, its circle's centre and
/// radius where it is an arc, and the face it bounds (`""` a step on the axis).
struct Steps { steps: Vec<(String,V,V,Option<(V,f64)>)> }

impl Steps {
    /// `None` where the pose leaves no meridian to read (a rough start's stop, its terms apart).
    fn read(e: &Elaborated,name: &str) -> Option<Steps> {
        use gcs_core::brep::planar::Seg;
        let m = gcs_core::solid::region::meridian(&e.sketch,index(e,name)).ok()?;
        let basis = gcs_core::plane::Basis { u: m.seam,v: m.axis,o: m.origin };
        let at = |q: [f64;2]| basis.lift(q[0],q[1]);
        let steps = m.region.loops.iter().flatten().map(|s| {
            let round = match s.seg { Seg::Arc {c,r,..} => Some((at(c),r)),_ => None };
            (m.name(s.tag),at(s.seg.start()),at(s.seg.end()),round)
        }).collect();
        Some(Steps { steps })
    }
    /// The steps bounding face `face` (its name's last segment).
    fn of(&self,face: &str) -> Vec<&(String,V,V,Option<(V,f64)>)> {
        let found: Vec<_> = self.steps.iter().filter(|s| s.0.rsplit('.').next() == Some(face)).collect();
        assert!(!found.is_empty(),"no `{face}` among {:?}",self.steps.iter().map(|s| &s.0).collect::<Vec<_>>());
        found
    }
    /// Where faces `a` and `b` meet: the end the steps share.
    fn corner(&self,a: &str,b: &str) -> V {
        let ends = |f: &str| self.of(f).iter().flat_map(|s| [s.1,s.2]).collect::<Vec<V>>();
        let (ea,eb) = (ends(a),ends(b));
        let (_,p) = ea.iter().flat_map(|p| eb.iter().map(move |q| (dist(*p,*q),*p)))
            .min_by(|x,y| x.0.total_cmp(&y.0)).unwrap();
        p
    }
    /// The round of face `face`: its centre and radius.
    fn round(&self,face: &str) -> (V,f64) { self.of(face)[0].3.unwrap_or_else(|| panic!("`{face}` is no arc")) }
    /// Where face `face` meets the axis: its end there.
    fn on_axis(&self,face: &str,up: (V,V)) -> V {
        self.of(face).iter().flat_map(|s| [s.1,s.2]).min_by(|p,q|
            off_line(*p,up.0,up.1).total_cmp(&off_line(*q,up.0,up.1))).unwrap()
    }
}

/// The motions, under `generation`.
const MOTIONS: [&str;8] = ["crown_roll","pinion_roll","gear_roll","pinion_generation",
    "gear_generation","pinion_index","gear_index","crown_neighbor"];

/// Every quantity of one pair; `probe` sizes the points the motions are asked to carry.
fn read_pair(e: &Elaborated,probe: f64) -> Reading {
    let o = point(e,"pair.gear_cone.O");
    let (g0,g1) = line(e,"pair.gear_cone.ax");
    let (c0,c1) = line(e,"pair.gear_cone.crown_ax");
    let a = point(e,"pair.pinion_cone.A");
    let (p0,p1) = line(e,"pair.pinion_cone.ax");
    let m = point(e,"pair.gear_cone.M");
    let r = dist(m,o);
    let mut out = Reading { values: Vec::new() };
    out.push("R",r);
    out.point("O",o);
    out.point("M",m);
    out.point("pinion apex",a);
    out.push("gear pitch angle",angle(sub(g1,g0),sub(m,o)));
    out.push("pinion pitch angle",angle(sub(p1,p0),sub(m,a)));
    out.point("gear axis",unit(sub(g1,g0)));
    out.push("gear axis off O",off_line(o,g0,sub(g1,g0)));
    out.point("crown axis",unit(sub(c1,c0)));
    out.push("crown axis off O",off_line(o,c0,sub(c1,c0)));
    out.point("pinion axis",unit(sub(p1,p0)));
    out.push("pinion axis off apex",off_line(a,p0,sub(p1,p0)));
    out.push("pinion distance",dist(m,a));
    out.push("shaft angle",angle(sub(g1,g0),sub(p1,p0)));
    let skew = unit(cross(sub(g1,g0),sub(p1,p0)));
    out.push("offset",dot(sub(p0,g0),skew).abs());
    // The cutter: its centre and axis, and the trace through M about it.
    let (u0,u1) = line(e,"pair.mill.ax");
    out.point("C",u0);
    out.point("cutter axis",unit(sub(u1,u0)));
    let heading = unit(cross([0.,0.,1.],sub(m,u0)));
    out.push("pinion spiral angle",slant(heading,sub(m,a)));
    out.push("gear spiral angle",slant(heading,sub(m,o)));
    // Sections, about the cutter's axis: distance from it, and height.
    let up = unit(sub(u1,u0));
    let meridian = |p: V| (off_line(p,u0,up),dot(sub(p,u0),up));
    let lp = point(e,"pair.mill.inner_pitch");
    let rp = point(e,"pair.mill.outer_pitch");
    out.push("section width",dist(lp,rp));
    out.push("section radius",(meridian(lp).0+meridian(rp).0)/2.);
    let corner = |out: &mut Reading,name: String,p: V| {
        let (rho,h) = meridian(p);
        out.push(&format!("{name} radial"),rho);
        out.push(&format!("{name} height"),h);
    };
    // a pose a meridian cannot be read at reads nowhere, every corner of it apart from the pair
    let nowhere = [f64::NAN;3];
    let read: Vec<Option<Steps>> = CROWNS.iter().map(|(_,solid,_,_)| Steps::read(e,solid)).collect();
    for ((label,_,corners,rounds),steps) in CROWNS.iter().zip(&read) {
        for (k,a,b) in corners.iter() {
            corner(&mut out,format!("{label}.{k}"),steps.as_ref().map_or(nowhere,|s| s.corner(a,b)));
        }
        for (k,face) in rounds.iter() {
            corner(&mut out,format!("{label}.{k}"),steps.as_ref().map_or(nowhere,|s| s.round(face).0));
        }
        out.push(&format!("{label} rounding"),steps.as_ref().map_or(f64::NAN,|s| s.round(rounds[0].1).1));
    }
    let [_,outer,inner] = &read[..] else { unreachable!("three crowns") };
    for (end,face) in [("base_end","base"),("tip_end","tip")] {
        corner(&mut out,format!("space outer {end}"),outer.as_ref().map_or(nowhere,|s| s.corner(face,"cap")));
        corner(&mut out,format!("space inner {end}"),inner.as_ref().map_or(nowhere,|s| s.on_axis(face,(u0,up))));
    }
    // The motions: their numbers, and where they carry three points at two angles.
    // A phase is the roll the motion is read from: roll 0 is as drawn.
    for label in MOTIONS {
        let (i,phase) = motion(e,label);
        let (ratio,advance) = e.sketch.motions[i].rotation(&e.sketch).unwrap_or_default();
        out.push(&format!("{label} ratio"),ratio);
        out.push(&format!("{label} phase"),phase);
        out.push(&format!("{label} advance"),advance);
        let family = motion::Family::read(&e.sketch,i).unwrap();
        for t in [-0.6,0.25] {
            let pose = family.at(t+phase/if ratio == 0. { 1. } else { ratio }).unwrap();
            for (k,x) in [[0.,0.,0.],[1.,0.,0.],[0.3,-0.2,0.5]].iter().enumerate() {
                out.point(&format!("{label}({t}) of {k}"),pose.point(x.map(|c| c*probe)));
            }
        }
    }
    out
}

/// The recorded quantities: each name, and its value at every design, in `designs()` order.
fn recorded() -> (Vec<String>,Vec<(String,Vec<f64>)>) {
    let text = include_str!("fixtures/hypoid_layout.tsv");
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    let labels = lines.next().unwrap().split('\t').skip(1).map(String::from).collect();
    let rows = lines.map(|l| {
        let mut fields = l.split('\t');
        (fields.next().unwrap().to_string(),fields.map(|v| v.parse().unwrap()).collect())
    }).collect();
    (labels,rows)
}

#[test]
fn the_layout_reproduces_the_pairs_named_quantities() {
    let (labels,rows) = recorded();
    let all = designs();
    assert_eq!(labels,all.iter().map(|d| d.label.clone()).collect::<Vec<_>>());
    let mut worst: std::collections::BTreeMap<String,(f64,String)> = Default::default();
    let recorded: std::collections::BTreeMap<&str,&Vec<f64>> =
        rows.iter().map(|(name,values)| (name.as_str(),values)).collect();
    for (k,design) in all.iter().enumerate() {
        let e = design.read();
        // the motions carry points at the pair's own scale, the recorded R
        let scale = rows[0].1[k];
        assert_eq!(rows[0].0,"R");
        let reading = read_pair(&e,scale);
        assert_eq!(reading.values.len(),rows.len());
        for (name,v) in &reading.values {
            let values = recorded.get(name.as_str())
                .unwrap_or_else(|| panic!("`{name}` is not recorded"));
            // a length relative to the cone distance; an angle in degrees, a direction's
            // component, a ratio and a phase in radians as they are
            let plain = ["angle","ratio","phase","ax."].iter().any(|w| name.contains(w));
            let d = (v-values[k]).abs()/if plain { 1. } else { scale };
            // the quantity, whichever of its points or poses
            let key = name.split(['(','.']).next().unwrap().trim().to_string();
            let entry = worst.entry(key).or_insert((0.,String::new()));
            if d > entry.0 || entry.1.is_empty() {
                *entry = (d,format!("{} ({name})",design.label));
            }
        }
    }
    let mut failed = Vec::new();
    for (key,(d,at)) in &worst {
        println!("{key:32} {d:9.2e}  {at}");
        if !(*d <= 1e-9) { failed.push(format!("{key}: {d:e} at {at}")); }
    }
    assert!(failed.is_empty(),"{failed:#?}");
}

/// The static solids: the crown tooth and the gear's space cutter by the names the pair gives
/// them (under `pair`), each the label its recorded rows carry.
const SOLIDS: [&str;2] = ["tooth.crown","gear_space.body"];

/// One of `SOLIDS` as a field.
fn static_field(e: &Elaborated,name: &str,r: f64) -> MaterialField {
    MaterialField::read(&e.sketch,fixtures::solid(e,&format!("pair.{name}")),1e-10*r)
        .unwrap_or_else(|err| panic!("{name}: {err}"))
}

/// Each member's blank's limits as the pair drew them when its material was recorded: the heel
/// sphere's and the tip cone's carriers the blank lies within, the toe sphere's and the back
/// cone's the blank lies outside — the recorded rows (`{member}_limits.{limit}.carrier`) the
/// region blank (`hypoid.sv`'s `Blank`) is held to, by the body rule over them.
const LIMITS: [(&str,f64);4] = [("heel",1.),("tip",1.),("toe",-1.),("back",-1.)];

/// A grid of `n` points a side over the box `lo`..`hi`.
fn grid(lo: V,hi: V,n: usize) -> Vec<V> {
    let at = |k: usize,i: usize| lo[k]+(hi[k]-lo[k])*i as f64/(n-1) as f64;
    (0..n).flat_map(|i| (0..n).flat_map(move |j| (0..n).map(move |k| [at(0,i),at(1,j),at(2,k)])))
        .collect()
}

/// Where the material is recorded: a coarse grid over both members and a finer one over the
/// tooth band about the mean point, from the design's cone distance `r`.
fn material_points(r: f64) -> Vec<V> {
    let mut points = grid([-0.3*r,-0.6*r,-0.8*r],[1.4*r,0.9*r,0.8*r],5);
    points.extend(grid([0.93*r,-0.06*r,-0.025*r],[1.07*r,0.06*r,0.025*r],3));
    points
}

/// The rolls the members' material was recorded at, in degrees (pinion, gear), by design.  Each
/// is found through its blank (#271), ending at the first step its tool is proved clear over; at
/// the bevel pinion's upper end the tool grazes the blank within the proof's depth, so a pose a
/// few bits off ends it a step sooner (#283).  A reading inside the cut is a magnitude as well as
/// a sign, which the sweep's extent moves, so the members are read rolled as recorded.
const RECORDED_ROLLS: [(&str,[[f64;2];2]);2] =
    [("configured",[[-24.,22.],[-35.,44.]]),("bevel",[[-24.,26.],[-36.,46.]])];

/// The material: every solid the pair evaluated — the crowns, the space cutter, the blank
/// limits, and the members with their swept cuts — read as fields at the recorded points, at
/// the configured hypoid and the bevel pair.
#[test]
fn the_layout_makes_the_pairs_material() {
    let (_,rows) = recorded();
    let text = include_str!("fixtures/hypoid_layout_material.tsv");
    let mut recorded = std::collections::BTreeMap::new();
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let mut fields = line.split('\t');
        let key = (fields.next().unwrap().to_string(),fields.next().unwrap().to_string());
        recorded.insert(key,fields.map(|v| v.parse::<f64>().unwrap()).collect::<Vec<_>>());
    }
    for (k,design) in designs().into_iter().enumerate().take(2) {
        let (label,[pinion,gear]) = RECORDED_ROLLS[k];
        assert_eq!(label,design.label);
        let rolled = fixtures::gear::rolled(&design.text,"pinion",pinion);
        let e = fixtures::gear::read(&fixtures::gear::rolled(&rolled,"gear",gear));
        let r = rows[0].1[k];
        let read = |name: &str| {
            MaterialField::read(&e.sketch,fixtures::solid(&e,&format!("pair.{name}")),1e-10*r)
                .unwrap_or_else(|err| panic!("{}: {name}: {err}",design.label))
        };
        let points = material_points(r);
        let values = |name: &str| &recorded[&(design.label.clone(),name.to_string())];
        // Every static solid, its value over the whole of both members and over the teeth.
        for name in SOLIDS {
            let (field,want) = (static_field(&e,name,r),values(name));
            assert_eq!(want.len(),points.len());
            let worst = points.iter().zip(want).map(|(&p,w)| (field.side(p)-w).abs())
                .fold(0.,f64::max);
            println!("{:10} {name:32} {:9.2e}",design.label,worst/r);
            assert!(worst <= 1e-9*r,"{}: {name}: the field is {worst} off the pair's",design.label);
        }
        // Each member's blank, a region: its material the recorded limits' by the body rule, at
        // every point they decide (the carriers' fields one-Lipschitz, not distances, so held by
        // sign alone, and only where every limit is clear of its surface).
        for member in ["gear","pinion"] {
            let field = MaterialField::read(&e.sketch,fixtures::solid(&e,&format!("pair.{member}_blank.material")),1e-10*r)
                .unwrap_or_else(|err| panic!("{}: {member}'s blank: {err}",design.label));
            let limits: Vec<(&Vec<f64>,f64)> = LIMITS.iter().map(|(limit,sense)|
                (values(&format!("{member}_limits.{limit}.carrier")),*sense)).collect();
            let (mut decided,mut inside) = (0,0);
            for (k,&p) in points.iter().enumerate() {
                if limits.iter().any(|(v,_)| v[k].abs() < 1e-6*r) { continue }
                let held = limits.iter().all(|(v,sense)| v[k]*sense < 0.);
                let side = field.side(p);
                if side.abs() < 1e-6*r { continue }
                assert_eq!(side < 0.,held,"{}: {member}'s blank at {p:?}: {side} where the limits say {held}",design.label);
                decided += 1;
                inside += usize::from(held);
            }
            println!("{:10} {member} blank {decided} points decided, {inside} inside",design.label);
            assert!(decided > points.len()/2 && inside > 0,"{}: {member}'s blank: {decided} decided, {inside} inside",design.label);
        }
        // The members, each blank less its swept cuts, read by branch and bound (`side` is a
        // sign, whose magnitude is wherever the search stopped). A reading is exact to its
        // query's accuracy near the boundary and to a thousandth of itself away from it
        // (`Query::at`), so every point is held to that, on the recorded side. The crowns are
        // regions since #214: the same surfaces, read through their meridians, so a reading off
        // the boundary may stop elsewhere within its accuracy than the drawn sections' did; the
        // band within a hundredth of the cone distance of the surface is reported.
        for member in ["pinion.body","gear.body"] {
            let (field,want) = (read(member),values(member));
            let (mut inside,mut near,mut far) = (0,0_f64,0_f64);
            for (&p,&a) in points.iter().zip(want) {
                let b = field.reading(p).value;
                let d = (a-b).abs();
                if a < 0. { inside += 1; }
                let accuracy = 2e-10*(1.+norm(p))+1e-3*a.abs().max(b.abs());
                assert!(d <= accuracy && (a < 0.) == (b < 0.),"{}: {member} at {p:?}: {a} recorded, {b} read",
                    design.label);
                if a.abs().max(b.abs()) < 0.01*r { near = near.max(d); } else { far = far.max(d/a.abs()); }
            }
            println!("{:10} {member:32} {:9.2e}  (elsewhere {far:.1e} of the reading; {inside} of {} \
                inside)",design.label,near/r,points.len());
            assert!(inside > 0 && inside < points.len(),"{}: {member}: the grid misses it",
                design.label);
        }
    }
}

/// A tool, not a check: re-records designs' columns from the layout.
/// `SOLVENT_RECORD=configured,hypoid6 cargo test -p gcs-core --test core hypoid_layout::record --
/// --ignored --nocapture` prints each named design's quantities as a column and its material as
/// rows, in the fixtures' own format, to be pasted over the old. Only a design off the bevel pair
/// is recorded from the layout; the bevel pair's and the sizes' columns are the old pair's and
/// stay as they are. The blank's limits' carrier rows (`LIMITS`) are not printed: the limits are
/// drawn no more, and a re-record keeps those rows.
#[test]
#[ignore = "a tool: SOLVENT_RECORD names the designs to re-record"]
fn record() {
    let wanted = std::env::var("SOLVENT_RECORD").expect("SOLVENT_RECORD");
    let (_,rows) = recorded();
    for (k,design) in designs().into_iter().enumerate() {
        if !wanted.split(',').any(|w| w == design.label) { continue; }
        let e = design.read();
        let reading = read_pair(&e,rows[0].1[k]);
        println!("column\t{}",design.label);
        for (name,_) in &rows {
            let v = reading.values.iter().find(|(n,_)| n == name).map_or(f64::NAN,|(_,v)| *v);
            println!("{name}\t{v:.12e}");
        }
        if k >= 2 { continue; }
        let r = reading.values[0].1;
        let points = material_points(r);
        let read = |name: &str|
            MaterialField::read(&e.sketch,fixtures::solid(&e,&format!("pair.{name}")),1e-10*r).unwrap();
        let row = |name: &str,values: Vec<f64>| println!("material\t{}\t{name}\t{}",design.label,
            values.iter().map(|v| format!("{v:.12e}")).collect::<Vec<_>>().join("\t"));
        for name in SOLIDS {
            let field = static_field(&e,name,r);
            row(name,points.iter().map(|&p| field.side(p)).collect());
        }
        for member in ["pinion.body","gear.body"] {
            let field = read(member);
            row(member,points.iter().map(|&p| field.reading(p).value).collect());
        }
    }
}

/// The true hypoid (docs/spiral-bevel-layout-plan.md, the true hypoid), read off the solved
/// layout at designs across size, ratio and offset: the shafts square and the offset apart, the
/// two pitch cones tangent to one pitch plane along their generators through M, the pinion's
/// pitch radius the equal normal pitch's, and each member rolling on the common crown at the
/// crown's tooth count over its own, N_c / N = (2R / m) / N. The pinion's pitch angle is solved,
/// so its cone's own ratio, one over the sine of it, is not that ratio: the layout measures the
/// pinion's roll off the gear's triangle (`hypoid.sv`'s step 5), whose hypotenuse is R and whose
/// short leg is N_p m / 2, and this holds it to the tooth counts.
#[test]
fn the_true_hypoid_is_square_offset_on_one_pitch_plane_and_rolls_at_the_tooth_ratio() {
    // teeth, module, offset (mm), shift and spiral (degrees)
    let designs = [([24,48],2.,0.,0.,35.),([24,48],2.,5.7,0.,35.),([24,48],2.,25.,12.5,25.),
        ([28,49],0.2,1.,0.,35.),([32,32],25.4,100.,5.,30.)];
    for (teeth,module,offset,shift,spiral) in designs {
        let design = Design::at(format!("{}x{} m{module} E{offset}",teeth[0],teeth[1]),
            teeth,module,offset,shift,spiral);
        let e = design.read();
        let label = &design.label;
        let [np,ng] = teeth.map(f64::from);
        let (o,m,a) = (point(&e,"pair.gear_cone.O"),point(&e,"pair.gear_cone.M"),
            point(&e,"pair.pinion_cone.A"));
        let (g0,g1) = line(&e,"pair.gear_cone.ax");
        let (p0,p1) = line(&e,"pair.pinion_cone.ax");
        let (c0,c1) = line(&e,"pair.gear_cone.crown_ax");
        let (g,p,n) = (unit(sub(g1,g0)),unit(sub(p1,p0)),unit(sub(c1,c0)));
        let r = dist(m,o);
        let ct = (np*np+ng*ng).sqrt();
        assert!((r-module*ct/2.).abs() < 1e-9*r,"{label}: R {r}");
        // the shafts
        let shaft = angle(g,p);
        let skew = dot(sub(p0,g0),unit(cross(g,p))).abs();
        println!("{label:24} shaft {shaft:.12} offset {skew:.12}");
        assert!((shaft-90.).abs() < 1e-9,"{label}: shaft angle {shaft}");
        assert!((skew-offset).abs() < 1e-9*r,"{label}: offset {skew}");
        // one pitch plane: both apexes and M on it, each axis in the plane square to it
        // through its generator, so each cone touches it along that generator
        for (what,x) in [("M",m),("the pinion's apex",a)] {
            assert!(dot(sub(x,o),n).abs() < 1e-9*r,"{label}: {what} off the pitch plane");
        }
        for (what,axis,apex) in [("gear",g,o),("pinion",p,a)] {
            let generator = unit(sub(m,apex));
            assert!(dot(n,cross(axis,generator)).abs() < 1e-10,
                "{label}: the {what}'s cone leaves P");
            let (pitch,incline) = (angle(axis,generator),90.-angle(axis,n).min(180.-angle(axis,n)));
            assert!((pitch-incline).abs() < 1e-8,
                "{label}: the {what}'s pitch angle {pitch} against {incline}");
        }
        // the equal normal pitch: each member's pitch radius at M times the cosine of its spiral
        // is its teeth times half the normal module
        let c = point(&e,"pair.mill.ax.p1");
        let heading = unit(cross(n,sub(m,c)));
        let normal_module = module*spiral.to_radians().cos();
        for (what,axis,axis_point,apex,teeth) in [("gear",g,g0,o,ng),("pinion",p,p0,a,np)] {
            let radius = off_line(m,axis_point,axis);
            let psi = slant(heading,sub(m,apex)).to_radians();
            let pitch = radius*psi.cos();
            assert!((pitch-teeth*normal_module/2.).abs() < 1e-9*r,
                "{label}: the {what}'s normal pitch radius {pitch}");
        }
        // the rolls: each member's ratio is the crown's tooth count over its own, and the crown's
        // neighbour is one crown pitch, 2 pi / N_c
        let rotation = |name: &str| e.sketch.motions[joint(&e,name)].rotation(&e.sketch).unwrap();
        let pinion = rotation("pinion_roll").0;
        let gear = rotation("gear_roll").0;
        let pitch = motion(&e,"crown_neighbor").1;
        println!("{label:24} pinion {pinion:.15} against {:.15}, gear {gear:.15} against {:.15}",
            ct/np,-ct/ng);
        assert!((pinion-ct/np).abs() < 1e-12*ct,"{label}: the pinion rolls at {pinion}");
        assert!((gear+ct/ng).abs() < 1e-12*ct,"{label}: the gear rolls at {gear}");
        assert!((pitch+std::f64::consts::TAU/ct).abs() < 1e-12,"{label}: a crown pitch is {pitch}");
        // off the bevel, the pinion's own cone would roll at another ratio
        let cone = 1./angle(p,sub(m,a)).to_radians().sin();
        println!("{label:24} the pinion's cone alone {cone:.6}");
        if offset > 0. { assert!((cone-ct/np).abs() > 1e-3,"{label}: {cone}"); }
    }
}

/// How far a reading is from the recorded column `k`: the worst difference, a length relative
/// to the cone distance and an angle, a direction's component, a ratio or a phase as it is, and
/// the quantity it is at.
fn off_recorded(reading: &Reading,rows: &[(String,Vec<f64>)],k: usize) -> (f64,String) {
    let scale = rows[0].1[k];
    let mut worst = (0.,String::new());
    for (name,v) in &reading.values {
        let values = &rows.iter().find(|(recorded,_)| recorded == name)
            .unwrap_or_else(|| panic!("`{name}` is not recorded")).1;
        let plain = ["angle","ratio","phase","ax."].iter().any(|w| name.contains(w));
        let d = (v-values[k]).abs()/if plain { 1. } else { scale };
        if !(d <= worst.0) { worst = (d,name.clone()); }
    }
    worst
}

impl Design {
    /// The pair elaborated and not solved: where its words put it, no point seeded.
    fn unsolved(&self) -> Elaborated { fixtures::gear::unsolved(&self.text) }
    /// `sk`, this design's pair, started from the same layout solved at `f` times its size
    /// (`scaled`): right in shape and wrong in size, which is what a seed a person writes by eye
    /// is.
    fn reseed(&self,sk: &mut gcs_core::model::Sketch,f: f64) {
        let rough = fixtures::gear::read(&scaled(&self.text,f));
        assert_eq!(sk.params.len(),rough.sketch.params.len());
        for (p,q) in sk.params.iter_mut().zip(&rough.sketch.params) {
            if !p.fixed { p.value = q.value; }
        }
    }
}

/// The input `name` of the pair's text, in its document's unit.
fn input(text: &str,name: &str) -> f64 {
    let line = text.lines().find_map(|l| l.strip_prefix(&format!("param {name} := ")))
        .unwrap_or_else(|| panic!("no input `{name}`"));
    line.trim().trim_end_matches("mm").parse().unwrap()
}

/// The pair with its lengths — the mean module and the offset — scaled by `f`.
fn scaled(text: &str,f: f64) -> String {
    let params = ["mean_module","offset"].map(|p| (p,input(text,p)*f));
    fixtures::gear::configure(text,&params)
}

/// `sk` with point `name` carried across the pitch plane's u axis (in P) or through P (in space):
/// its last coordinate turned, and its lift's with it.
fn crossed(e: &mut Elaborated,name: &str) {
    let p = index(e,name);
    let sk = &mut e.sketch;
    let own = sk.point_all_params(p);
    let last = *own.last().unwrap() as usize;
    sk.params[last].value = -sk.params[last].value;
    if let Some(k) = sk.lift_of(p) {
        let w = sk.world_point(p);
        for (i,v) in sk.lifts[k].x.into_iter().zip(w) {
            sk.params[i as usize].value = v;
        }
    }
}

/// **Words say which root** (§9.6, #109, #217): the gear's axis below the pitch plane and the
/// pinion's offset hand (`foot inside P`, `A distance(>= 0, along: v) P`) were once said
/// by the side their seeds stood on.  Every design solved, then its gear axis's foot carried
/// above P and its pinion's apex to the offset's other hand — the start a seed on the wrong side
/// of both made — and solved again, lands on the recorded pair.
#[test]
fn a_start_on_the_wrong_side_lands_on_the_recorded_pair() {
    let (_,rows) = recorded();
    for (k,design) in designs().into_iter().enumerate() {
        let mut e = design.read();
        crossed(&mut e,"pair.gear_cone.foot");
        crossed(&mut e,"pair.pinion_cone.A");
        let res = gcs_core::solve::solve(&mut e.sketch,fixtures::accurate());
        assert!(res.success,"{}: {res:?}",design.label);
        let (d,at) = off_recorded(&read_pair(&e,rows[0].1[k]),&rows,k);
        println!("{:12} from the wrong side: {} {d:.1e} off at {at}",design.label,res.method);
        assert!(d <= 1e-9,"{}: {d:e} off the recorded pair at {at}",design.label);
    }
}

/// **The layout solves from its words alone** (#217): nothing seeds a point, so every design's
/// equations start where the elaborator puts unseeded geometry, about the origin at the size of a
/// unit, and a document solve gets to the recorded pair by the block-triangular order
/// (docs/block-triangular-solve-plan.md) — each block from what the blocks before it made, its
/// root searched for where it comes to none or to one a bound it decides refuses (`Search`), and
/// polished whole — to the 1e-9 the named-quantities gate asks, `Rescue` (the default: the pass
/// after a failed whole-system DogLeg) and `First` (the pass before it) alike.  And from rough
/// starts, the same layout solved at two fifths and at two and a half times its size: right in
/// shape and wrong in size, where a whole-system DogLeg that settles comes out of `Rescue` the
/// same solve.  The whole-system solve alone (`Off`, DogLeg then LM) is reported, not held: from
/// the words alone it reaches the pair from no design.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 36 s: the hypoid layout solved")]
fn the_layout_solves_from_its_words_alone_in_block_order() {
    use gcs_core::solve::{self,BlockMode,SolveOpts};
    let (_,rows) = recorded();
    let opts = |blocks| SolveOpts {blocks,..fixtures::accurate()};
    let mut whole_reached = 0;
    for (k,design) in designs().into_iter().enumerate() {
        let r = rows[0].1[k];
        let mut e = design.unsolved();
        let start = e.sketch.clone();
        for f in [None,Some(0.4),Some(2.5)] {
            let mut rough = start.clone();
            if let Some(f) = f { design.reseed(&mut rough,f); }
            let from = f.map_or("its words".to_string(),|f| format!("x{f}"));
            let mut line = format!("{:12} from {from:10}",design.label);
            let mut whole = None;
            for mode in [BlockMode::Off,BlockMode::Rescue,BlockMode::First] {
                e.sketch = rough.clone();
                let res = solve::solve(&mut e.sketch,opts(mode));
                let off = res.success.then(|| off_recorded(&read_pair(&e,r),&rows,k));
                line += &format!(" | {mode:?}: {} by {} ({:.1e})",if res.success { "solved" }
                    else { "FAILED" },res.method,res.max_residual);
                if let Some((d,at)) = &off { line += &format!(", {d:.1e} off at {at}"); }
                let label = format!("{} from {from}, {mode:?}",design.label);
                let bits = crate::common::bits(&e.sketch);
                match (mode,&whole) {
                    (BlockMode::Off,_) => {
                        if off.as_ref().is_some_and(|(d,_)| *d <= 1e-9) { whole_reached += 1; }
                        whole = Some((res.settled() && res.method == "dogleg",bits));
                    }
                    // a whole-system DogLeg that settles here comes out of `Rescue` the same solve
                    (BlockMode::Rescue,Some((true,x))) => assert!(&bits == x,"{label} {line}"),
                    _ => match off {
                        Some((d,_)) if d <= 1e-9 => {}
                        None => panic!("{label}: {res:?}"),
                        Some((d,at)) => panic!("{label}: {d:e} off the recorded pair at {at}"),
                    },
                }
            }
            println!("{line}");
        }
    }
    println!("the whole-system solve reached the recorded pair from {whole_reached} starts of 36");
}

/// The pair opens: `hypoid.sv` solved as solventc solves it (the default options, an interactive
/// acceptance of 1e-6) **converges**, not merely under that acceptance — a whole-system DogLeg that
/// runs out of iterations at a residual of 1e-7 is a success by the interactive measure, and the
/// app would draw a pose that is not the solution.
#[test]
fn the_pair_converges_as_solventc_solves_it() {
    use gcs_core::solve::{self,SolveOpts};
    let mut e = fixtures::gear::unsolved(&fixtures::gear::source());
    let res = solve::solve(&mut e.sketch,SolveOpts::default());
    println!("hypoid.sv {} by {} ({:.1e})",res.success,res.method,res.max_residual);
    assert!(res.success && res.max_residual <= 1e-10,"{res:?}");
}

/// The block path decides nothing by the order its work happens to be done in, nor by a draw:
/// the same start — the words alone, and a rough one — solved twice by it comes out the same to
/// the bit, its root search and all.
#[test]
fn a_start_solves_to_the_same_bits_twice() {
    use gcs_core::solve::{self,BlockMode,SolveOpts};
    let designs = designs();
    for design in [&designs[0],&designs[5]] {
        for rough in [false,true] {
            let mut e = design.unsolved();
            if rough { design.reseed(&mut e.sketch,2.5); }
            let (mut a,mut b) = (e.sketch.clone(),e.sketch.clone());
            for blocks in [BlockMode::First,BlockMode::Rescue] {
                let opts = SolveOpts {blocks,..fixtures::accurate()};
                let (ra,rb) = (solve::solve(&mut a,opts),solve::solve(&mut b,opts));
                assert!(ra.success,"{}: {ra:?}",design.label);
                assert_eq!((ra.nfev,ra.njev,ra.iterations),(rb.nfev,rb.njev,rb.iterations));
                assert_eq!(crate::common::bits(&a),crate::common::bits(&b));
            }
        }
    }
}

/// The design labelled `label`, by its index among `designs()` and solved as solventc solves it:
/// the settled pose the rough starts below are made from.
fn solved_design(label: &str) -> (usize,Elaborated) {
    use gcs_core::solve::{self,SolveOpts};
    let designs = designs();
    let k = designs.iter().position(|d| d.label == label).unwrap();
    let mut e = designs[k].unsolved();
    assert!(solve::solve(&mut e.sketch,SolveOpts::default()).settled(),"{label}");
    (k,e)
}

/// **Where a rescue scenario starts is looked for, not recorded**
/// (docs/iteration-limit-rescue-plan.md).  A scenario is a jittered start that stops a certain
/// way; which jitters do is a fact about the layout's rows, and any change to them moves it.  So
/// each test walks the same starts in order — `jitters`, each at the first `SEARCH` seeds — and
/// asks `attempt` of each: `None` where the start does not stop as the scenario asks, else
/// whether the scenario's own claim holds there, having checked at every such stop what the
/// rescue promises of all of them.  The first start where the claim holds ends the walk; none
/// means the layout has moved past the search.
fn first_start(scenario: &str, jitters: &[f64], mut attempt: impl FnMut(f64,u32) -> Option<bool>)
    -> (f64,u32) {
    const SEARCH: u32 = 64;
    let mut stops = 0;
    for &jitter in jitters { for seed in 0..SEARCH {
        match attempt(jitter,seed) {
            Some(true) => {
                println!("{scenario}: jittered {jitter} at seed {seed}, after {stops} other stops");
                return (jitter,seed);
            }
            Some(false) => stops += 1,
            None => {}
        }
    }}
    panic!("{scenario}: no start among the first {SEARCH} seeds of {jitters:?} ({stops} stopped as \
            asked without it): widen the search")
}

/// **A solve that stops on its iteration limit is rescued.** A design solved as solventc solves
/// it and then jittered by a fifth of its extent is back under the interactive acceptance within
/// `LIMIT` iterations and not at its solution: without the block rescue (`BlockMode::Off`) the
/// solve succeeds on status 4, short of the recorded pair.  `LIMIT` is half the default: since a
/// direction's row reads a sine or a cosine (#479) and a recovery is kept only where it settles
/// (#449), no jitter of the first 64 at a fifth, a tenth or five others stops so within a
/// hundred.  By default such a stop is not *settled* and the rescue runs: what it returns has
/// settled, or is the stop bit for bit, unless a bound then breaks (status 5) — at every stop
/// met.  And from some, the block pass settles it.  Which root it settles on is the layout's; the
/// rough starts' gates hold that.  The bevel pair: since the heading's square from O is said as
/// parallel to MC (#217) the configured hypoid converges inside the limit from every jitter of a
/// fifth tried, and so does every design at a few hundredths; and since the grip's plane runs
/// along its lines by its rows (#535) so does the bevel from every jitter of a fifth of the first
/// 64, so the walk goes on to a tenth.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 16 s: the hypoid layout rescued")]
fn a_stop_on_the_iteration_limit_is_rescued() {
    use gcs_core::solve::{self,BlockMode,SolveOpts};
    // half the default hundred, both solves alike: the block pass works to the same limit
    const LIMIT: i32 = 50;
    let (_,rows) = recorded();
    let (k,mut e) = solved_design("bevel");
    let reference = e.sketch.clone();
    first_start("a limit stop the block pass settles",&[0.2,0.1],|jitter,seed| {
        let start = crate::common::jittered(&reference,jitter,seed);
        e.sketch = start.clone();
        let off = SolveOpts {blocks:BlockMode::Off,max_iter:LIMIT,..SolveOpts::default()};
        let stop = solve::solve(&mut e.sketch,off);
        // a start that failed, or settled, is not the scenario, and may stand where the pair's
        // faces are not to be read
        if !(stop.success && stop.status == 4) { return None; }
        let (short,_) = off_recorded(&read_pair(&e,rows[0].1[k]),&rows,k);
        if !(short > 1e-9) { return None; }
        let stopped = crate::common::bits(&e.sketch);
        e.sketch = start;
        let res = solve::solve(&mut e.sketch,SolveOpts {max_iter:LIMIT,..SolveOpts::default()});
        let kept = crate::common::bits(&e.sketch) == stopped && res.status == stop.status;
        let broken = res.status == solve::BOUND_BROKEN;
        assert!(res.settled() || kept || broken,"seed {seed}: {res:?}");
        let (d,at) = off_recorded(&read_pair(&e,rows[0].1[k]),&rows,k);
        println!("seed {seed}: stopped {short:.1e} off, {} by {} {d:.1e} off at {at}",
            if res.settled() { "settled" } else { "unsettled" },res.method);
        Some(res.settled() && res.method == "blocks")
    });
}

/// A tool, not a check: which path each design's solve takes from the layout's own words — the
/// whole-system DogLeg (`dogleg`), the block rescue (`blocks`) or the LM retry (`lm`) — under
/// each `BlockMode`, how far from the recorded pair it lands, and how long it takes.
#[test]
#[ignore = "a tool: cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
    hypoid_layout::solve_paths -- --ignored --nocapture"]
fn solve_paths() {
    use gcs_core::solve::{self,BlockMode,SolveOpts};
    let (_,rows) = recorded();
    for (k,design) in designs().into_iter().enumerate() {
        let mut line = format!("{:12}",design.label);
        for blocks in [BlockMode::Off,BlockMode::Rescue,BlockMode::First] {
            let mut e = design.unsolved();
            let clock = std::time::Instant::now();
            let res = solve::solve(&mut e.sketch,SolveOpts {blocks,..fixtures::accurate()});
            let ms = clock.elapsed().as_secs_f64()*1e3;
            line += &format!(" | {blocks:?}: {} {} ({:.1e}, {ms:.0} ms)",
                if res.success { "solved" } else { "FAILED" },res.method,res.max_residual);
            if res.success {
                let (d,at) = off_recorded(&read_pair(&e,rows[0].1[k]),&rows,k);
                line += &format!(" {d:.1e} off at {at}");
            }
        }
        println!("{line}");
    }
}

/// A tool, not a check: the layout at 480 designs (three tooth pairs, modules 2 and 25.4,
/// offsets 0 to 30 modules / 2, shifts 0 to 15 degrees, spirals 20 to 35) solved as solventc
/// solves it, against the block path held to 1e-12; it prints each design where the default
/// solve does not converge to the same pose (a whole-system DogLeg that stalls just under the
/// interactive acceptance counts as a success there), or where the block path fails.
#[test]
#[ignore = "a tool: cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
    hypoid_layout::design_sweep -- --ignored --nocapture"]
fn design_sweep() {
    use gcs_core::solve::{self,BlockMode,SolveOpts};
    let (mut bad,mut n) = (0,0);
    for teeth in [[24,48],[32,32],[13,40]] { for module in [2.,25.4] {
    for offset in [0.,10.,20.,25.,30.] { for shift in [0.,7.5,12.5,15.] { for spiral in [20.,25.,30.,35.] {
        let design = Design::at(String::new(),teeth,module,offset*module/2.,shift,spiral);
        let (mut a,mut b) = (design.unsolved(),design.unsolved());
        let ra = solve::solve(&mut a.sketch,SolveOpts::default());
        let rb = solve::solve(&mut b.sketch,SolveOpts {blocks:BlockMode::First,..fixtures::accurate()});
        let diff = crate::common::apart(&a.sketch,&b.sketch);
        n += 1;
        if !(ra.success && ra.max_residual < 1e-10 && diff < 1e-8) || !rb.success {
            bad += 1;
            println!("{teeth:?} m{module} E{} shift {shift} spiral {spiral}: default {} by {} \
                ({:.1e}, {}); blocks {} ({:.1e}); {diff:.1e} apart",offset*module/2.,ra.success,
                ra.method,ra.max_residual,ra.message,rb.success,rb.max_residual);
        }
    }}}}}
    println!("{bad} of {n} designs");
}


