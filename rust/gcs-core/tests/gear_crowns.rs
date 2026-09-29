//! The two crowns that generate the spiral-bevel pair are one crown and its mate: the pinion's
//! crown tooth and the gear's space cutter meet flank on flank, so at any roll no point is kept
//! by both members and none is cut by both.  A pressure shift gives the crown tooth unequal
//! flanks, and its mate must carry them on the opposite sides; the layout draws the mate's
//! flanks on the tooth's own flank lines (`crown/mate_section.sv`), so it does by construction.
//! A mate whose flanks stood at the tooth's own angles instead — its section shared with the
//! tooth's, as an earlier model had it — crossed the mating flanks at the pitch line, 20 degrees
//! apart, and the configured hypoid interfered by 19 mm³ over the whole face width.
//!
//! Backlash keeps the shared flank lines and stands each cutter's flanks a quarter of it outside
//! them (`crown/section.sv`): the crowns overlap by half the backlash across each shared line, so
//! each generated flank is a quarter of it inside its conjugate one. Tip relief is a second cut
//! beside each crown (`crown/relief.sv`) whose chamfer leaves the crown's flank at a kink, so the
//! two cuts together are the crown turned there. End relief chamfers the blank itself, the tip
//! cone's corner with each end sphere (`blank/ends.sv`).
use gcs_core::solid::MaterialField;

/// The pinion's crown tooth and the gear's space cutter at a design, in their generating pose.
fn crowns(shift: f64,rewrite: &dyn Fn(&str,String) -> String) -> (MaterialField,MaterialField) {
    let e = fixtures::gear::read_configured_with(&mut |name,text|
        rewrite(name,fixtures::gear::design(name,text,25.,shift,25.)));
    let read = |name: &str| MaterialField::read(&e.sketch,fixtures::solid(&e,name),1e-10).unwrap();
    (read("pair.reference.tooth.crown"),read("pair.reference.gear_space.body"))
}

/// Points of a box about the mean point, in the flank band clear of both crowns' tips and
/// bases, that both crowns take (`overlap`), and points within `reach` of both that neither
/// takes (`gap`): a sliver between facing flanks, each beyond `tol`.  The space cutter is the
/// one mating tooth beyond the crown tooth's outer flank; the gear's indexing makes the rest,
/// so a point beside the inner flank is near the tooth alone and is no gap.
fn disagreements(tooth: &MaterialField,space: &MaterialField) -> (usize,usize,usize) {
    let (tol,reach) = (1e-3,0.4);
    let (mut inside,mut overlap,mut gap) = (0,0,0);
    for i in 0..=30 { for j in 0..=24 { for k in 0..=12 {
        let p = [50.+0.25*i as f64,-3.+0.25*j as f64,-1.2+0.2*k as f64];
        let (a,b) = (tooth.side(p),space.side(p));
        if a < -tol { inside += 1; if b < -tol { overlap += 1; } }
        if (tol..reach).contains(&a) && (tol..reach).contains(&b) { gap += 1; }
    }}}
    (inside,overlap,gap)
}

#[test]
fn the_generating_crowns_mate_flank_on_flank_with_a_pressure_shift() {
    for shift in [0.,10.] {
        let (tooth,space) = crowns(shift,&|_,text| text);
        let (inside,overlap,gap) = disagreements(&tooth,&space);
        assert!(inside > 500,"shift {shift}: the box misses the crown tooth ({inside} points in it)");
        assert_eq!((overlap,gap),(0,0),"shift {shift}: {inside} points in the tooth");
    }
    // The control: each mate flank turned twice the shift off the tooth's flank line, the
    // mate's section at the tooth's own pressure angles.
    let shared = |name: &str,text: String| if name == "crown.mate_section" {
        let lines = "  inner_along angle(180deg) inner\n  outer_along angle(180deg) outer\n";
        assert!(text.contains(lines));
        text.replace(lines,"  inner_along angle(180deg + 2 * design.shift) inner\n  \
            outer_along angle(180deg + 2 * design.shift) outer\n") } else { text };
    let (tooth,space) = crowns(10.,&shared);
    let (_,overlap,gap) = disagreements(&tooth,&space);
    assert!(overlap > 100 && gap > 100,"the shared section went unnoticed: {overlap} overlapping, {gap} missed");
}

/// The crown tooth and the space cutter at the configured hypoid with `backlash` millimetres of
/// normal backlash and no tip relief.
fn crowns_with_backlash(backlash: f64) -> (MaterialField,MaterialField) {
    let e = fixtures::gear::read_configured_with(&mut |name,text|
        fixtures::gear::fabricated(name,text,[25.,12.5,25.],backlash,0.,0.));
    let read = |name: &str| MaterialField::read(&e.sketch,fixtures::solid(&e,name),1e-10).unwrap();
    (read("pair.reference.tooth.crown"),read("pair.reference.gear_space.body"))
}

/// Backlash, proved on the cutters: every point of a conjugate flank (the crowns without backlash)
/// in the flank band lies exactly a quarter of the backlash inside both cutters with backlash
/// wherever it bounded them, and each cutter's flank stands exactly that far out along the same
/// normal. Across the flank the two share, the cutters overlap by half the backlash; each member's
/// flank, the envelope of its cutter's, is the conjugate flank offset a quarter of it into the
/// member, so the pair's normal clearance is half the backlash at each flank, all of it once the
/// other flanks touch. The readings are exact (a revolution's meridian distance), so the tolerance
/// is the solve's.
#[test]
fn backlash_stands_each_cutter_a_quarter_of_it_off_the_shared_flanks() {
    let b = 0.2;
    let (tooth0,space0) = crowns_with_backlash(0.);
    let (tooth,space) = crowns_with_backlash(b);
    let value = |f: &MaterialField,p: [f64;3]| f.reading(p).value;
    let along = |p: [f64;3],n: [f64;3],d: f64| std::array::from_fn(|k| p[k]+d*n[k]);
    // Points on the conjugate crowns' flanks in the band clear of tips and bases, the same box as
    // `disagreements`: a grid point moved onto the nearer cutter's flank along its normal, kept
    // where that cutter reads as a straight flank a tenth of a millimetre either side.
    let (mut shared,mut tooth_only,mut space_only) = (0,0,0);
    let mut worst: f64 = 0.;
    for i in 0..=30 { for j in 0..=24 { for k in 0..=12 {
        let p = [50.+0.25*i as f64,-3.+0.25*j as f64,-1.2+0.2*k as f64];
        for (conjugate,other) in [(&tooth0,&space0),(&space0,&tooth0)] {
            let r = conjugate.reading(p);
            if r.value.abs() > 0.3 { continue; }
            let g = r.gradient;
            let len = (g[0]*g[0]+g[1]*g[1]+g[2]*g[2]).sqrt();
            let n = g.map(|x| x/len);
            let q = along(p,n,-r.value);
            if q[2].abs() > 1.2 { continue; }
            let straight = [-0.1,0.1].iter().all(|&d| (value(conjugate,along(q,n,d))-d).abs() < 1e-9);
            if !straight { continue; }
            let on_other = value(other,q).abs() < 1e-9;
            let (with,without) = if std::ptr::eq(conjugate,&tooth0) { (&tooth,&space) } else { (&space,&tooth) };
            // The cutter with backlash takes the conjugate flank a quarter of it deep, its own flank
            // a quarter of it out along the same normal.
            worst = worst.max((value(with,q)+b/4.).abs()).max(value(with,along(q,n,b/4.)).abs());
            if on_other {
                // The shared flank: the other cutter also takes it a quarter deep, from its side.
                worst = worst.max((value(without,q)+b/4.).abs()).max(value(without,along(q,n,-b/4.)).abs());
                shared += 1;
            } else if std::ptr::eq(conjugate,&tooth0) { tooth_only += 1; } else { space_only += 1; }
        }
    }}}
    eprintln!("{shared} shared flank points, {tooth_only} on the tooth's other flank, {space_only} on the space's; worst {worst:e} mm");
    assert!(shared > 100 && tooth_only > 100,"too few flank points: {shared} shared, {tooth_only} tooth only");
    assert!(worst < 1e-9,"a flank stands {worst} mm off a quarter of the backlash");
    // And the band itself: points both cutters take, none either misses beside a flank.
    let (inside,overlap,gap) = disagreements(&tooth,&space);
    eprintln!("backlash {b}: {inside} points in the tooth, {overlap} in both cutters, {gap} in neither");
    assert!(overlap > 0 && gap == 0);
}

/// Tip relief, proved on the cutters (`crown/relief.sv`'s preview, at the configured design): the
/// crown with its relief is the crown's section turned at a kink by the relief angle. Along each
/// chamfer below its kink the relief bounds (reads zero) and the crown lies behind it by
/// sin(relief angle) per millimetre; along the flank above the kink the crown bounds and the relief
/// lies behind it by the same. In the section, the kink stands the tip relief short of where the
/// flank generates the member's tip, an addendum from the pitch line, and the chamfer turns the
/// relief angle from the flank. For the gear, each side of the space cutter against its own
/// relief side, before the neighbour's indexing.
#[test]
fn the_tip_relief_turns_each_crown_flank_at_its_kink() {
    let project = fixtures::gear::project();
    let source = std::fs::read_to_string(project.join("crown/relief.sv")).unwrap();
    let e = fixtures::read_beside(&source,&project,&mut |_,text| text);
    let sk = &e.sketch;
    let entity = |name: &str| e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i();
    let field = |name: &str| MaterialField::read(sk,entity(name),1e-10).unwrap();
    let (turn,tip_relief,addendum,base) = (30f64.to_radians(),0.2,1.,2.);
    let xy = |name: &str| { let (x,y) = sk.point_xy(entity(name)); [x,y] };
    // A line's ends, by the line's name: in the section, and in space.
    let ends = |line: &str| { let l = &sk.lines[entity(line)]; [l.p1 as usize,l.p2 as usize] };
    let ends_xy = |line: &str| ends(line).map(|i| { let (x,y) = sk.point_xy(i); [x,y] });
    let sub2 = |a: [f64;2],b: [f64;2]| [a[0]-b[0],a[1]-b[1]];
    let cross2 = |a: [f64;2],b: [f64;2]| a[0]*b[1]-a[1]*b[0];
    let norm2 = |a: [f64;2]| a[0].hypot(a[1]);
    // The distance from a point to a line of the section, by name.
    let off = |p: [f64;2],line: &str| {
        let [a,b] = ends_xy(line);
        (cross2(sub2(b,a),sub2(p,a))/norm2(sub2(b,a))).abs()
    };
    let unit = |a: [f64;3],b: [f64;3]| { let d: [f64;3] = std::array::from_fn(|k| b[k]-a[k]);
        let n = (d[0]*d[0]+d[1]*d[1]+d[2]*d[2]).sqrt(); d.map(|x| x/n) };
    let at = |p: [f64;3],u: [f64;3],t: f64| -> [f64;3] { std::array::from_fn(|k| p[k]+t*u[k]) };
    let mut worst: f64 = 0.;
    for (crown,relief,chamfer,flank,section) in [
        ("tooth.crown","tooth_relief.outer_side","tooth_relief.outer_chamfer","tooth.rack.outer","tooth.rack"),
        ("tooth.crown","tooth_relief.inner_side","tooth_relief.inner_chamfer","tooth.rack.inner","tooth.rack"),
        ("space.outer_crown","space_relief.outer_side","space_relief.outer_chamfer","mate.outer.inner","mate.outer"),
        ("space.inner_crown","space_relief.inner_side","space_relief.inner_chamfer","mate.inner.outer","mate.inner"),
    ] {
        let (crown,relief) = (field(crown),field(relief));
        let value = |f: &MaterialField,p: [f64;3]| f.reading(p).value;
        // The section: the kink's depth and the chamfer's turn.
        let (kink,foot) = (xy(&format!("{chamfer}.at.kink")),xy(&format!("{chamfer}.foot")));
        let pitch = format!("{section}.pitch");
        let normal_module = off(ends_xy(&pitch)[0],&format!("{section}.base"))/base;
        let depth = off(kink,&pitch);
        let [a,b] = ends_xy(flank);
        let angle = (cross2(sub2(b,a),sub2(foot,kink))/(norm2(sub2(b,a))*norm2(sub2(foot,kink)))).abs().asin();
        eprintln!("{chamfer}: kink {depth:.9} deep (addendum {:.9} less {tip_relief}), turned {:.9} degrees",
            addendum*normal_module,angle.to_degrees());
        worst = worst.max((depth-(addendum*normal_module-tip_relief)).abs()).max((angle-turn).abs());
        // In space: down the chamfer from the kink, and up the flank from it.
        let world = |name: &str| sk.world_point(entity(name));
        let k = world(&format!("{chamfer}.at.kink"));
        let down = unit(k,world(&format!("{chamfer}.foot")));
        // The flank's end farther from its base line runs toward the tip.
        let tip_end = ends(flank)[if off(a,&format!("{section}.base")) > off(b,&format!("{section}.base")) { 0 } else { 1 }];
        let up = unit(k,sk.world_point(tip_end));
        worst = worst.max(value(&crown,k).abs()).max(value(&relief,k).abs());
        for t in [0.05,0.1,0.2,0.4,0.8] {
            let p = at(k,down,t);
            worst = worst.max(value(&relief,p).abs()).max((value(&crown,p)-t*turn.sin()).abs());
        }
        for s in [0.05,0.1,0.2,0.3] {
            let p = at(k,up,s);
            worst = worst.max(value(&crown,p).abs()).max((value(&relief,p)-s*turn.sin()).abs());
        }
    }
    assert!(worst < 1e-9,"the relief leaves its stated chamfer by {worst}");
}

/// A tip relief of zero is no relief: the pair then has no relief cut at all, and each member is
/// its blank less its crown's sweep alone, as before the allowance existed.
#[test]
fn a_tip_relief_of_zero_cuts_nothing() {
    let relief_solids = |e: &gcs_core::program::Elaborated| e.sketch.solids.iter().filter(|s| s.name.contains("relief")).count();
    let without = fixtures::gear::read_configured_with(&mut |name,text| fixtures::gear::fabricated(name,text,[25.,12.5,25.],0.05,0.,0.));
    assert_eq!(relief_solids(&without),0);
    // As configured: the two relief tools (three and four solids), a sweep of each, and a
    // placement at every tooth of each member.
    let with = fixtures::gear::read_as_configured();
    assert_eq!(relief_solids(&with),3+4+2+24+48);
}

/// End relief, proved on the blanks (`blank/ends.sv`, as configured): at each member's toe and heel
/// the corner where the tip cone meets the sphere is chamfered the end relief each way, the
/// chamfer's end on the tip cone that much farther along the cone distance toward the tooth and its
/// end on the sphere that much down from the tip cone; and the ring each end cuts takes from the
/// blank the corner beyond the chamfer and nothing else. In the member's axial plane about the
/// corner, a point of the blank is in the ring exactly when it is on the corner's side of the
/// chamfer.
#[test]
fn the_end_relief_chamfers_each_tip_corner() {
    let e = fixtures::gear::read_as_configured();
    let sk = &e.sketch;
    let entity = |name: &str| e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i();
    // An end's entities are one copy of the limits' `repeat`, named under the copy's key.
    let copied = |limits: &str,end: &str,part: &str| e.map.names.values().flatten()
        .find(|n| n.starts_with(limits) && n.ends_with(&format!(".{end}_end.{part}")))
        .unwrap_or_else(|| panic!("no `{part}` of the {end} end of `{limits}`")).clone();
    let world = |name: &str| sk.world_point(entity(name));
    let field = |name: &str| MaterialField::read(sk,entity(name),1e-10).unwrap();
    let value = |f: &MaterialField,p: [f64;3]| f.reading(p).value;
    let sub = |a: [f64;3],b: [f64;3]| -> [f64;3] { std::array::from_fn(|k| a[k]-b[k]) };
    let dist = |a: [f64;3],b: [f64;3]| { let d = sub(a,b); (d[0]*d[0]+d[1]*d[1]+d[2]*d[2]).sqrt() };
    let size = 0.2;
    let (mut worst,mut kept,mut cut): (f64,usize,usize) = (0.,0,0);
    for member in ["pinion","gear"] {
        let limits = format!("pair.reference.{member}_blank");
        let apex = sk.world_point(sk.lines[entity(&format!("pair.reference.{member}.pitch_line"))].p1 as usize);
        let solid = |part: &str| field(&format!("{limits}.{part}.carrier"));
        let [heel,tip,toe,back] = ["heel","tip","toe","back"].map(solid);
        // the blank before its ends are relieved: the heel within the tip, less the toe and the back
        let blank = |p| value(&heel,p).max(value(&tip,p)).max(-value(&toe,p)).max(-value(&back,p));
        for (end,sphere,inward) in [("toe",&toe,1.),("heel",&heel,-1.)] {
            let at = |point: &str| world(&copied(&limits,end,point));
            let (corner,along,down) = (at("corner"),at("along_tip"),at("down_end"));
            let radius = dist(world(&format!("{limits}.span.{end}")),apex);
            for p in [corner,down] { worst = worst.max((dist(p,apex)-radius).abs()).max(value(sphere,p).abs()); }
            for p in [corner,along] { worst = worst.max(value(&tip,p).abs()); }
            worst = worst.max((inward*(dist(along,apex)-radius)-size).abs()).max((value(&tip,down)+size).abs());
            let ring = field(&copied(&limits,end,"ring"));
            let (a,d) = (sub(along,corner),sub(down,corner));
            for i in -15..=25 { for j in -15..=25 {
                let (s,t) = (i as f64/10.+0.013,j as f64/10.+0.007);
                let p: [f64;3] = std::array::from_fn(|k| corner[k]+s*a[k]+t*d[k]);
                if blank(p) > -1e-6 || (s+t-1.).abs() < 1e-3 { continue; }
                let (inside,beyond) = (value(&ring,p) < 0.,s+t < 1.);
                assert_eq!(inside,beyond,"{member} {end} at ({s}, {t}): in the ring {inside}, beyond the chamfer {beyond}");
                if inside { cut += 1; } else { kept += 1; }
            }}
        }
    }
    eprintln!("end relief: chamfers within {worst:.3e} mm; {cut} blank points cut by the rings, {kept} kept");
    assert!(worst < 1e-9,"the end relief leaves its stated chamfer by {worst}");
    assert!(cut > 20 && kept > 500);
}
