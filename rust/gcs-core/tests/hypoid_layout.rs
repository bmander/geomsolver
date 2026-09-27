//! The spiral-bevel pair laid out step by step (`spiral_bevel/gears_layout.sv`: the pitch
//! cones, the trace and the crown built from lines, circles and folds) against the pair as
//! `gears.sv` computes it with trigonometry: every named quantity the layout plan lists
//! (docs/spiral-bevel-layout-plan.md, migration step 2), at the configured hypoid, the bevel
//! pair, the six-degree hypoid and the nine sizes and ratios the paired-envelope checks read,
//! and the material both make at the configured design and the bevel.
//!
//! The two are written in different views (the layout's are folds of the pitch plane through
//! the mean point M), so nothing is compared by name alone: points in space, lines by where
//! they are and which way they run, sections by their meridian coordinates about the cutter's
//! axis, motions by the poses they give.
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

/// A design: the configuration module's text.
struct Design { label: String, configuration: String }

impl Design {
    /// The configured pair with the offset, shift and spiral rewritten, as the suites do.
    fn configured(label: &str,offset: f64,shift: f64,spiral: f64) -> Self {
        let path = fixtures::gear::project().join("configuration.sv");
        let text = std::fs::read_to_string(path).unwrap();
        let configuration = fixtures::gear::design("configuration",text,offset,shift,spiral);
        Design { label: label.into(), configuration }
    }
    /// A size and ratio case of `tests/envelope/paired.rs`: the bevel pair at those teeth.
    fn sized(teeth: [u32;2],module: f64) -> Self {
        Design { label: format!("{}x{} m{module}",teeth[0],teeth[1]), configuration: format!(
            "param pinion_teeth = {}\nparam gear_teeth = {}\nparam mean_module = {module}mm\n\
             param offset_angle = 0deg\nparam pressure_shift = 0deg\nparam spiral_angle = 35deg\n",
            teeth[0],teeth[1]) }
    }
    /// `entry` in the project, its modules beside it (a dotted name in a subdirectory), the
    /// configuration replaced.
    fn read(&self,entry: &str) -> Elaborated {
        let base = fixtures::gear::project();
        let source = std::fs::read_to_string(base.join(entry)).unwrap();
        fixtures::elaborate(&source,&mut |name: &str| {
            if name == "configuration" { return Some(self.configuration.clone()); }
            std::fs::read_to_string(base.join(format!("{}.sv",name.replace('.',"/")))).ok()
                .or_else(|| gcs_core::library::resolve(name))
        })
    }
}

fn designs() -> Vec<Design> {
    let mut all = vec![Design::configured("configured",25.,10.,25.),
        Design::configured("bevel",0.,0.,35.),Design::configured("hypoid6",6.,0.,35.)];
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] { all.push(Design::sized(teeth,module)); }
    }
    all
}

/// Both pairs name their reference geometry under this; the members stand beside it.
const REF: &str = "pair.reference";

fn index(e: &Elaborated,name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}
/// A line's ends in space.
fn line(e: &Elaborated,name: &str) -> (V,V) {
    let l = &e.sketch.lines[index(e,&format!("{REF}.{name}"))];
    (e.sketch.world_point(l.p1 as usize),e.sketch.world_point(l.p2 as usize))
}
/// A point in space, or a line's end where the end is another's point by alias.
fn point(e: &Elaborated,name: &str) -> V {
    let full = format!("{REF}.{name}");
    if e.map.ent_named(&full).is_none() {
        if let Some((l,end)) = name.rsplit_once('.').filter(|(_,end)| ["p1","p2"].contains(end)) {
            let (a,b) = line(e,l);
            return if end == "p1" { a } else { b };
        }
    }
    e.sketch.world_point(index(e,&full))
}
fn motion(e: &Elaborated,name: &str) -> usize {
    let name = format!("{REF}.{name}");
    e.sketch.motions.iter().position(|m| m.name == name)
        .unwrap_or_else(|| panic!("no motion `{name}`"))
}

/// What the plan's gate compares, read off one elaborated pair: numbers with a name, and
/// the scale their tolerance is relative to.
struct Reading { values: Vec<(String,f64)>, scale: f64 }

impl Reading {
    fn push(&mut self,name: &str,v: f64) { self.values.push((name.into(),v)); }
    fn point(&mut self,name: &str,p: V) {
        for (k,c) in ["x","y","z"].iter().zip(p) { self.push(&format!("{name}.{k}"),c); }
    }
}

/// The names each model gives one thing under `REF`: the old pair's, then the layout's.
struct Names {
    apex: &'static str, mean: Option<&'static str>, gear_axis: &'static str,
    crown_axis: &'static str, pinion_apex: &'static str, pinion_axis: &'static str,
    cutter_up: &'static str, cutter_down: &'static str,
    tooth: &'static str, outer: &'static str, inner: &'static str,
    /// Each section's corners (base inner and outer, the joins, the tip's ends, the rounding
    /// centres): the mate walks the other way round and names its corners by side.
    tooth_corners: [&'static str;8], mate_corners: [&'static str;8],
    /// Each member's blank limits: the tip, root and back cones, the toe and heel spheres.
    gear_limits: [&'static str;5], pinion_limits: [&'static str;5],
    motions: [&'static str;8],
}

const CORNERS: [&str;8] = ["bl","br","lj","rj","lt","rt","cl","cr"];

const OLD: Names = Names {
    apex: "gear_axis.p1", mean: None, gear_axis: "gear_axis", crown_axis: "front_axis",
    pinion_apex: "pinion_apex", pinion_axis: "pinion_axis",
    cutter_up: "crown_front_axis", cutter_down: "crown_back_axis",
    tooth: "pinion", outer: "gear_outer", inner: "gear_inner",
    tooth_corners: CORNERS, mate_corners: CORNERS,
    gear_limits: ["gear_tip_boundary","gear_root_boundary","gear_back_boundary","toe","heel"],
    pinion_limits: ["pinion_tip_boundary","pinion_root_boundary","pinion_back_boundary",
        "pinion_toe","pinion_heel"],
    motions: ["crown_roll","pinion_roll","gear_roll","pinion_generation","gear_generation",
        "pinion_index","gear_index","crown_neighbor"],
};

const NEW: Names = Names {
    apex: "gear.O", mean: Some("gear.M"), gear_axis: "gear.axis", crown_axis: "gear.crown_axis",
    pinion_apex: "pinion.A", pinion_axis: "pinion.axis",
    cutter_up: "tooth.axis", cutter_down: "mate.axis",
    tooth: "tooth.rack", outer: "mate.outer", inner: "mate.inner",
    tooth_corners: CORNERS, mate_corners: ["bi","bo","ij","oj","it","ot","ci","co"],
    gear_limits: ["gear_blank.tip","gear_blank.root","gear_blank.back","gear_blank.toe",
        "gear_blank.heel"],
    pinion_limits: ["pinion_blank.tip","pinion_blank.root","pinion_blank.back",
        "pinion_blank.toe","pinion_blank.heel"],
    motions: ["generation.crown_roll","generation.pinion_roll","generation.gear_roll",
        "generation.pinion_generation","generation.gear_generation","generation.pinion_index",
        "generation.gear_index","generation.crown_neighbor"],
};

/// Every quantity of one pair; `probe` sizes the points the motions are asked to carry.
fn read_pair(e: &Elaborated,n: &Names,probe: f64) -> Reading {
    let o = point(e,n.apex);
    let (g0,g1) = line(e,n.gear_axis);
    let (c0,c1) = line(e,n.crown_axis);
    let a = point(e,n.pinion_apex);
    let (p0,p1) = line(e,n.pinion_axis);
    // The old pair names no mean point: it stands on +x at the cone distance, which its gear
    // axis is drawn as long as.
    let m = n.mean.map_or([dist(g1,o),0.,0.],|name| point(e,name));
    let r = dist(m,o);
    let mut out = Reading { values: Vec::new(), scale: r };
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
    // The cutter: its centre and axes, and the trace through M about it.
    let (u0,u1) = line(e,n.cutter_up);
    let (d0,d1) = line(e,n.cutter_down);
    out.point("C",u0);
    out.point("cutter axis",unit(sub(u1,u0)));
    out.point("mate axis",unit(sub(d1,d0)));
    out.push("mate axis off C",off_line(u0,d0,sub(d1,d0)));
    let heading = unit(cross([0.,0.,1.],sub(m,u0)));
    out.push("pinion spiral angle",slant(heading,sub(m,a)));
    out.push("gear spiral angle",slant(heading,sub(m,o)));
    // Sections, about the cutter's axis: distance from it, and height.
    let up = unit(sub(u1,u0));
    let meridian = |p: V| (off_line(p,u0,up),dot(sub(p,u0),up));
    let lp = point(e,&format!("{}.pitch.p1",n.tooth));
    let rp = point(e,&format!("{}.pitch.p2",n.tooth));
    out.push("section width",dist(lp,rp));
    out.push("section radius",(meridian(lp).0+meridian(rp).0)/2.);
    for (label,section,names) in [("tooth",n.tooth,&n.tooth_corners),
        ("mate outer",n.outer,&n.mate_corners),("mate inner",n.inner,&n.mate_corners)] {
        for (k,c) in CORNERS.iter().zip(names) {
            let (rho,h) = meridian(point(e,&format!("{section}.{c}")));
            out.push(&format!("{label}.{k} radial"),rho);
            out.push(&format!("{label}.{k} height"),h);
        }
        let round = &e.sketch.arcs[index(e,&format!("{REF}.{section}.outer_round"))];
        out.push(&format!("{label} rounding"),e.sketch.params[round.radius as usize].value);
    }
    for side in ["outer","inner"] {
        for end in ["base_end","tip_end"] {
            let (rho,h) = meridian(point(e,&format!("gear_space.{side}.{end}")));
            out.push(&format!("space {side} {end} radial"),rho);
            out.push(&format!("space {side} {end} height"),h);
        }
    }
    // The blank limits: each cone's quadrilateral and each sphere's poles, in space.
    for (member,limits) in [("gear",&n.gear_limits),("pinion",&n.pinion_limits)] {
        for (cone,name) in ["tip","root","back"].iter().zip(&limits[..3]) {
            for c in ["a","b","p","q"] {
                out.point(&format!("{member} {cone} cone {c}"),point(e,&format!("{name}.{c}")));
            }
        }
        for (sphere,name) in ["toe","heel"].iter().zip(&limits[3..]) {
            for c in ["top","bottom"] {
                out.point(&format!("{member} {sphere} {c}"),point(e,&format!("{name}.{c}")));
            }
        }
    }
    // The motions: their numbers, and where they carry three points at five angles.
    for (label,name) in OLD.motions.iter().zip(n.motions) {
        let i = motion(e,name);
        let (ratio,phase,advance) = e.sketch.motions[i].rotation(&e.sketch).unwrap_or_default();
        out.push(&format!("{label} ratio"),ratio);
        out.push(&format!("{label} phase"),phase);
        out.push(&format!("{label} advance"),advance);
        let family = motion::Family::read(&e.sketch,i).unwrap();
        for t in [-0.6,-0.1,0.,0.25,0.6] {
            let pose = family.at(t).unwrap();
            for (k,x) in [[0.,0.,0.],[1.,0.,0.],[0.3,-0.2,0.5]].iter().enumerate() {
                out.point(&format!("{label}({t}) of {k}"),pose.point(x.map(|c| c*probe)));
            }
        }
    }
    out
}

/// Each quantity's deviation between the pair and the layout at one design, over its scale:
/// a length relative to the cone distance; an angle in degrees, a direction's component, a
/// ratio and a phase in radians as they are.
fn compare(old: &Reading,new: &Reading) -> Vec<(String,f64)> {
    assert_eq!(old.values.len(),new.values.len());
    old.values.iter().zip(&new.values).map(|((name,a),(other,b))| {
        assert_eq!(name,other);
        let plain = ["angle","ratio","phase","axis."].iter().any(|w| name.contains(w));
        (name.clone(),(a-b).abs()/if plain { 1. } else { old.scale })
    }).collect()
}

#[test]
fn the_layout_reproduces_the_pairs_named_quantities() {
    let mut worst: std::collections::BTreeMap<String,(f64,String)> = Default::default();
    for design in designs() {
        let (old,new) = (design.read("gears.sv"),design.read("gears_layout.sv"));
        // the motions carry points at the old pair's own scale, so both are asked the same
        let probe = dist(line(&old,OLD.gear_axis).1,point(&old,OLD.apex));
        for (name,d) in compare(&read_pair(&old,&OLD,probe),&read_pair(&new,&NEW,probe)) {
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

/// The static solids both make, by the names each gives them under `REF`: the crowns, the
/// gear's space cutter and every blank limit's carrier.
const SOLIDS: [(&str,&str);14] = [
    ("pinion_crown","tooth.crown"),
    ("gear_outer_crown","mate.outer_crown"),
    ("gear_inner_crown","mate.inner_crown"),
    ("gear_space.body","gear_space.body"),
    ("gear_tip_boundary.carrier","gear_blank.tip.carrier"),
    ("gear_root_boundary.carrier","gear_blank.root.carrier"),
    ("gear_back_boundary.carrier","gear_blank.back.carrier"),
    ("toe.carrier","gear_blank.toe.carrier"),
    ("heel.carrier","gear_blank.heel.carrier"),
    ("pinion_tip_boundary.carrier","pinion_blank.tip.carrier"),
    ("pinion_root_boundary.carrier","pinion_blank.root.carrier"),
    ("pinion_back_boundary.carrier","pinion_blank.back.carrier"),
    ("pinion_toe.carrier","pinion_blank.toe.carrier"),
    ("pinion_heel.carrier","pinion_blank.heel.carrier"),
];

/// A grid of `n` points a side over the box `lo`..`hi`.
fn grid(lo: V,hi: V,n: usize) -> Vec<V> {
    let at = |k: usize,i: usize| lo[k]+(hi[k]-lo[k])*i as f64/(n-1) as f64;
    (0..n).flat_map(|i| (0..n).flat_map(move |j| (0..n).map(move |k| [at(0,i),at(1,j),at(2,k)])))
        .collect()
}

/// The material: every solid either pair evaluates — the crowns, the space cutter, the blank
/// limits, and the members with their swept cuts — read as fields over the members and the
/// tooth band, at the configured hypoid and the bevel pair.
#[test]
fn the_layout_makes_the_pairs_material() {
    let designs = [Design::configured("configured",25.,10.,25.),
        Design::configured("bevel",0.,0.,35.)];
    for design in designs {
        let (old,new) = (design.read("gears.sv"),design.read("gears_layout.sv"));
        let r = dist(point(&new,"gear.M"),point(&new,"gear.O"));
        let read = |e: &Elaborated,name: &str| {
            MaterialField::read(&e.sketch,fixtures::solid(e,name),1e-10*r)
                .unwrap_or_else(|err| panic!("{}: {name}: {err}",design.label))
        };
        // Every static solid, its value over the whole of both members and over the teeth.
        let mut points = grid([-0.3*r,-0.6*r,-0.8*r],[1.4*r,0.9*r,0.8*r],13);
        points.extend(grid([0.93*r,-0.06*r,-0.025*r],[1.07*r,0.06*r,0.025*r],11));
        for (a,b) in SOLIDS {
            let (fa,fb) = (read(&old,&format!("{REF}.{a}")),read(&new,&format!("{REF}.{b}")));
            let worst = points.iter().map(|&p| (fa.side(p)-fb.side(p)).abs()).fold(0.,f64::max);
            println!("{:10} {b:32} {:9.2e}",design.label,worst/r);
            assert!(worst <= 1e-9*r,"{}: {b}: the fields differ by {worst}",design.label);
        }
        // The members, each blank less its swept cuts, read by branch and bound (`side` is a
        // sign, whose magnitude is wherever the search stopped). A reading is exact to its
        // query's accuracy near the boundary and to a thousandth of itself away from it
        // (`Query::at`), so both are held to that, and the band within a hundredth of the cone
        // distance of the surface to the named quantities' 1e-9.
        for member in ["pair.pinion.body","pair.gear.body"] {
            let (fa,fb) = (read(&old,member),read(&new,member));
            let (mut inside,mut band,mut near,mut far) = (0,0,0_f64,0_f64);
            for &p in &points {
                let (a,b) = (fa.reading(p).value,fb.reading(p).value);
                let d = (a-b).abs();
                if a < 0. { inside += 1; }
                if a.abs().max(b.abs()) < 0.01*r { band += 1; near = near.max(d); } else {
                    let accuracy = 2e-10*(1.+norm(p))+1e-3*a.abs().max(b.abs());
                    assert!(d <= accuracy,"{}: {member} at {p:?}: {a} against {b}",design.label);
                    far = far.max(d/a.abs());
                }
            }
            println!("{:10} {member:32} {:9.2e}  ({band} points near the surface; elsewhere \
                {far:.1e} of the reading; {inside} of {} inside)",design.label,near/r,points.len());
            assert!(inside > 0 && inside < points.len() && band > 0,
                "{}: {member}: the grid misses it",design.label);
            assert!(near <= 1e-9*r,"{}: {member}: {near} apart near the surface",design.label);
        }
    }
}
