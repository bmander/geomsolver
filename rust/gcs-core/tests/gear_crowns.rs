//! The two crowns that generate the spiral-bevel pair are one crown and its mate: the pinion's
//! crown tooth and the gear's space cutter meet flank on flank, so at any roll no point is kept
//! by both members and none is cut by both.  A pressure shift gives the crown tooth unequal
//! flanks, and its mate must carry them on the opposite sides; the layout draws the mate's
//! flanks on the tooth's own flank lines (`crown/mate.sv`), so it does by construction.  A mate
//! whose flanks stood at the tooth's own angles instead — its section shared with the tooth's,
//! as an earlier model had it — crossed the mating flanks at the pitch line, 20 degrees apart,
//! and the configured hypoid interfered by 19 mm³ over the whole face width.
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
    let shared = |name: &str,text: String| if name == "crown.mate" {
        let lines = "  inner_along angle(180deg) inner\n  outer_along angle(180deg) outer\n";
        assert!(text.contains(lines));
        text.replace(lines,"  inner_along angle(180deg + 2 * design.shift) inner\n  \
            outer_along angle(180deg + 2 * design.shift) outer\n") } else { text };
    let (tooth,space) = crowns(10.,&shared);
    let (_,overlap,gap) = disagreements(&tooth,&space);
    assert!(overlap > 100 && gap > 100,"the shared section went unnoticed: {overlap} overlapping, {gap} missed");
}
