//! The two crowns that generate the spiral-bevel pair are one crown and its mate: the pinion's
//! crown tooth and the gear's space cutter meet flank on flank, so at any roll no point is kept
//! by both members and none is cut by both.  A pressure shift gives the crown tooth unequal
//! flanks, and its mate must carry them on the opposite sides; the layout makes both crowns
//! regions on the same two flank cones (`crown/cutter.sv`), so it does by construction.  A mate
//! whose flanks stood at the tooth's own angles instead — its section shared with the tooth's, as
//! an earlier model had it — crossed the mating flanks at the pitch line, 20 degrees apart, and
//! the configured hypoid interfered by 19 mm³ over the whole face width.
//!
//! Backlash grows each crown across the shared flank cones by a quarter of it (`crown/tooth.sv`):
//! the crowns overlap by half the backlash across each shared cone, so each generated flank is a
//! quarter of it inside its conjugate one. Tip relief is a second cut beside each crown
//! (`crown/relief.sv`) whose topping leaves the crown's flank at a kink, so the two cuts together
//! are the crown turned there. End relief chamfers the blank itself, the tip cone's corner with
//! each end sphere, in the blank's own region (`blank/member.sv`).
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
    // The control: each mate flank turned twice the shift off the tooth's flank cone, the mate's
    // flanks at the tooth's own pressure angles.
    let shared = |name: &str,text: String| if name == "layout" {
        let space = "  construction gear_space := crown.mate.Space(cutter.outer_flank, cutter.inner_flank,\n";
        assert!(text.contains(space));
        text.replace(space,"  shared_outer := std.ConeThrough(cutter.ax, thickness.outer_pitch,\n    \
            lean: -design.pressure - design.shift)\n  \
            shared_inner := std.ConeThrough(cutter.ax, thickness.inner_pitch,\n    \
            lean: design.pressure - design.shift)\n  \
            construction gear_space := crown.mate.Space(shared_outer, shared_inner,\n") } else { text };
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
/// crown with its relief is the crown's meridian turned at a kink by the relief angle. Along each
/// topping's generator below its kink the relief bounds (reads zero) and the crown lies behind it
/// by sin(relief angle) per millimetre; along the flank's above the kink the crown bounds and the
/// relief lies behind it by the same. The kink stands the tip relief short of where the flank
/// generates the member's tip, an addendum from the pitch plane along the cutter's axis, and the
/// topping's cone turns the relief angle from the flank's in the meridian. For the gear, each side
/// of the space cutter against its own relief side, before the neighbour's indexing.
#[test]
fn the_tip_relief_turns_each_crown_flank_at_its_kink() {
    let project = fixtures::gear::project();
    let source = std::fs::read_to_string(project.join("crown/relief.sv")).unwrap();
    let e = fixtures::read_beside(&source,&project,&mut |_,text| text);
    let sk = &e.sketch;
    let entity = |name: &str| e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i();
    let field = |name: &str| MaterialField::read(sk,entity(name),1e-10).unwrap();
    let (turn,tip_relief,addendum) = (30f64.to_radians(),0.2,1.);
    let world = |name: &str| sk.world_point(entity(name));
    let unit = |a: [f64;3],b: [f64;3]| { let d: [f64;3] = std::array::from_fn(|k| b[k]-a[k]);
        let n = (d[0]*d[0]+d[1]*d[1]+d[2]*d[2]).sqrt(); d.map(|x| x/n) };
    let dot = |a: [f64;3],b: [f64;3]| (0..3).map(|k| a[k]*b[k]).sum::<f64>();
    let at = |p: [f64;3],u: [f64;3],t: f64| -> [f64;3] { std::array::from_fn(|k| p[k]+t*u[k]) };
    let normal_module = sk.params[sk.free_vars["trace.normal_module"] as usize].value;
    // the cutter's axis, up from C
    let ax = &sk.lines[entity("cutter.ax")];
    let (c,up_axis) = (sk.world_point(ax.p1 as usize),unit(sk.world_point(ax.p1 as usize),sk.world_point(ax.p2 as usize)));
    let mut worst: f64 = 0.;
    // each side: the crown, its relief, the topping, the flank cone it leaves and its pitch point,
    // and which way along the cutter's axis the crown's base lies
    for (crown,relief,topping,flank,pitch,base) in [
        ("tooth.crown","tooth_relief.relief.crown","tooth_relief.outer_topping","cutter.outer_flank","thickness.outer_pitch",-1.),
        ("tooth.crown","tooth_relief.relief.crown","tooth_relief.inner_topping","cutter.inner_flank","thickness.inner_pitch",-1.),
        ("gear_space.outer_crown","space_relief.relief.outer_crown","space_relief.outer_topping","cutter.outer_flank","thickness.outer_pitch",1.),
        ("gear_space.inner_crown","space_relief.relief.inner_crown","space_relief.inner_topping","cutter.inner_flank","thickness.inner_pitch",1.),
    ] {
        let (crown,relief) = (field(crown),field(relief));
        let value = |f: &MaterialField,p: [f64;3]| f.reading(p).value;
        let k = world(&format!("{topping}.kink"));
        // the kink's depth along the axis from the pitch plane, and the topping's turn from the
        // flank: the angle between their generators in the meridian through the kink
        let depth = dot(up_axis,[k[0]-c[0],k[1]-c[1],k[2]-c[2]]).abs();
        let flank_line = unit(world(&format!("{flank}.apex")),world(pitch));
        let topping_line = unit(world(&format!("{topping}.cone.apex")),k);
        let angle = dot(flank_line,topping_line).abs().min(1.).acos();
        eprintln!("{topping}: kink {depth:.9} deep (addendum {:.9} less {tip_relief}), turned {:.9} degrees",
            addendum*normal_module,angle.to_degrees());
        worst = worst.max((depth-(addendum*normal_module-tip_relief)).abs()).max((angle-turn).abs());
        // In space: down the topping's generator from the kink toward the crown's base, and up
        // the flank's from it toward the tip.
        let baseward = up_axis.map(|x| x*base);
        let down = topping_line.map(|x| x*dot(topping_line,baseward).signum());
        let up = flank_line.map(|x| -x*dot(flank_line,baseward).signum());
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
    // As configured: the two relief tools (the pinion's topping, one region solid; the gear's
    // space, two and its neighbour and their body), a sweep of each, and a placement at every
    // tooth of each member.
    let with = fixtures::gear::read_as_configured();
    assert_eq!(relief_solids(&with),1+4+2+24+48);
}

/// End relief, proved on the blanks (`blank/member.sv`, as configured): at each member's toe and
/// heel the corner where the tip cone meets the sphere is chamfered in the blank's region
/// (`std.Chamfered`), lowered into its meridian as a conic's arc: on it the distances from the tip
/// cone and from the sphere, each over the end relief, sum to one; its end on the tip cone is the
/// end relief inside the sphere and its end on the sphere the end relief below the tip cone.
#[test]
fn the_end_relief_chamfers_each_tip_corner() {
    use gcs_core::brep::planar::Seg;
    use gcs_core::model::{RegionShape,SolidDef};
    use gcs_core::syntax::Cmp;
    let e = fixtures::gear::read_as_configured();
    let sk = &e.sketch;
    let size = 0.2;
    let mut worst: f64 = 0.;
    for member in ["pinion","gear"] {
        let solid = fixtures::solid(&e,&format!("pair.reference.{member}_blank.material"));
        assert!(!gcs_core::solid::region::in_general(sk,solid),"{member}'s blank is lowered");
        let m = gcs_core::solid::region::meridian(sk,solid).unwrap();
        let SolidDef::Region {terms,..} = &sk.solids[solid].def else { panic!("a region") };
        let tag = |ball: bool,cmp: Cmp| terms.iter().position(|t| t.cmp == cmp
            && if ball { matches!(t.shape,RegionShape::Ball {..}) } else { matches!(t.shape,RegionShape::Cone {..}) })
            .unwrap() as u32+1;
        let step = |t: u32| m.region.loops.iter().flatten().find(|s| s.tag == t).unwrap().seg;
        let Seg::Line {a: ta,b: tb} = step(tag(false,Cmp::Le)) else { panic!("the tip is a line") };
        let len = ((tb[0]-ta[0]).powi(2)+(tb[1]-ta[1]).powi(2)).sqrt();
        let to_tip = |q: [f64;2]| ((tb[0]-ta[0])*(q[1]-ta[1])-(tb[1]-ta[1])*(q[0]-ta[0]))/len;
        let conics: Vec<Seg> = m.region.loops.iter().flatten().map(|s| s.seg)
            .filter(|s| matches!(s,Seg::Conic {..})).collect();
        assert_eq!(conics.len(),2,"{member}: a flat at each end");
        for end in [tag(true,Cmp::Ge),tag(true,Cmp::Le)] {
            let Seg::Arc {c,r,sweep,..} = step(end) else { panic!("an end is an arc") };
            let to_sphere = |q: [f64;2]| sweep.signum()*(r-((q[0]-c[0]).powi(2)+(q[1]-c[1]).powi(2)).sqrt());
            let conic = conics.iter().find(|s| [s.start(),s.end()].iter().any(|&p| to_sphere(p).abs() < 1e-9))
                .unwrap_or_else(|| panic!("{member}: no flat on the sphere {end}"));
            for k in 0..=100 {
                let q = conic.point(k as f64/100.);
                worst = worst.max((to_tip(q)/size+to_sphere(q)/size-1.).abs());
            }
            for p in [conic.start(),conic.end()] {
                let (t,s) = (to_tip(p),to_sphere(p));
                worst = worst.max(if t.abs() < s.abs() { t.abs()+(s-size).abs() } else { s.abs()+(t-size).abs() });
            }
        }
    }
    eprintln!("end relief: the flats within {worst:.3e} mm of their sums and legs");
    assert!(worst < 1e-9,"the end relief leaves its stated chamfer by {worst}");
}

/// Each member of the pair built by this kernel (`export::exact`) from the crowns at `rewrite`'s
/// design, at the export tolerance `tolerance` where one is given: its volume, its blank's and
/// its face count.
fn built(rewrite: &dyn Fn(&str,String) -> String,tolerance: Option<f64>) -> [(f64,f64,usize);2] {
    use gcs_core::brep::{export,props,sweep::Say};
    let say = Say {stage:&|_: &str| {},mark:&|_| {}};
    let e = fixtures::gear::read_configured_with(&mut |name,text| rewrite(name,text));
    ["pinion","gear"].map(|member| {
        let body = fixtures::solid(&e,&format!("pair.{member}.body"));
        let blank = e.sketch.exact_solid(fixtures::solid(&e,&format!("pair.reference.{member}_blank.material"))).unwrap();
        let tolerance = tolerance.map(|t| gcs_core::solid::export::Tolerance::new(t).unwrap());
        let exact = export::exact(&e.sketch,body,None,tolerance,&say)
            .unwrap_or_else(|r| panic!("{member}: refused at {:?}: {}",r.stage,r.message));
        (props::volume(&exact.solid),props::volume(&blank.brep)/blank.mm.powi(3),exact.solid.faces.len())
    })
}

/// **The crowns as regions cut the tooth spaces the drawn sections did** (#214): the bevel pair,
/// one tooth space each, built by this kernel, takes from each blank the space
/// `native_surfaces::gear_cells` records of OCCT's construction (120.708817 and 117.137321 mm³),
/// to the same 1e-3 — so the check stands where the native build is not made.  The drawn sections
/// took 120.675849 and 117.134204 mm³ by this kernel, which the regions take to 1e-7.
#[test]
fn the_regions_cut_the_recorded_tooth_spaces() {
    let spaces = built(&|name,text| { let text = fixtures::gear::bevel(name,text);
        if name == "members" { fixtures::gear::one_space(&text) } else { text } },None);
    for ((member,(volume,blank,_)),(recorded,drawn)) in ["pinion","gear"].iter().zip(spaces)
        .zip([(120.708817,120.675849),(117.137321,117.134204)]) {
        let space = blank-volume;
        eprintln!("{member}: tooth space {space:.6} mm³, OCCT's {recorded}, the drawn sections' {drawn}");
        assert!((space-recorded).abs() < 1e-3*recorded,"{member}: {space} against {recorded}");
        assert!((space-drawn).abs() < 1e-7*drawn,"{member}: {space} against the drawn sections' {drawn}");
    }
}

/// **The pair as fabricated agrees with its export from the drawn sections** (#214): the
/// configured hypoid with its backlash, tip relief and end relief, each member built whole by this
/// kernel at the export tolerance (10 µm), has the faces the drawn sections' export had and their
/// volume to 1e-6 — a ten-micron band over either member is a thousand times that.  Recorded
/// from the drawn sections before they were retired.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 40 s: both members built whole as fabricated")]
fn the_fabricated_pair_is_the_drawn_sections_export() {
    let members = built(&|_,text| text,Some(0.01));
    for ((member,(volume,_,faces)),(recorded,recorded_faces)) in ["pinion","gear"].iter().zip(members)
        .zip([(14643.571918,243),(22261.893669,291)]) {
        eprintln!("{member}: {volume:.6} mm³ in {faces} faces, the drawn sections' {recorded} in {recorded_faces}");
        assert_eq!(faces,recorded_faces,"{member}");
        assert!((volume-recorded).abs() < 1e-6*recorded,"{member}: {volume} against {recorded}");
    }
}
