//! The configured hypoid pinion cut by the face mill its balance finds (#904 A1/A2), the mill stated:
//! its pressure shift written into the pair's call as a number beside the cutter's radius the
//! document states (`fixtures::gear::as_found`), so what the kernel builds there is judged apart
//! from the balance that finds the shift (`hypoid_machine`).
use crate::common::{built,saying};
use gcs_core::brep::{export,props,sweep};

/// What the pinion's cut says of its sheet at the mill its balance finds, at any scale: laid 69 by
/// 39, one chart, its faces named by the cutter's.
pub(crate) fn the_pinions_sheet(said: &str) {
    assert!(said.contains("`pair.pinion.cut.removal`: sheet 69x39, one chart"),"{said}");
    assert!(said.contains("`pair.pinion.cut.removal`: the sheet's faces, by the cutter's: inner, inner_round, tip, outer_round, outer"),
        "{said}");
}

/// The pinion built by sectors: the sector turned into its 24 copies, each the blank's pieces, the
/// space's five faces and the relief's five, 243 in all, and the solid sound.
pub(crate) fn the_pinion_by_sectors(exact: &export::Exact,said: &str) {
    assert!(said.contains("turned the sector into 24 copies"),"{said}");
    assert_eq!(exact.solid.faces.len(),243,"{said}");
    exact.solid.check(1e-6).unwrap();
}

/// **The pinion's cut at the mill its balance finds is one chart** (#904 A1/A2): its sheet is traced,
/// fitted within the gross bars and named by the cutter's faces. At the start of its band the fold
/// of the stations' time chart, where the two roots of a point of the crown's inner flank meet near
/// the flank's end at its round, ends in a cusp among the stations, outside the blank: a station
/// before it folds and the one after does not. The station just before the cusp holds contacts in
/// the blank on both sides of its fold's gap; walked from the ones past the round, it turned onto
/// the other root at the gap and ran on a turn round the pinion, its rows torn from its neighbour's,
/// and the fit left its contacts by 0.155 mm and turned 89.7° from them. The walk now crosses a
/// fold's gap outside the blank to where its own root resumes (`brep::swept::trace::Tracer::trace`).
#[test]
fn the_pinions_cut_at_the_found_mill_is_one_chart() {
    use gcs_core::solid::admission::{admit_body,Options};
    let e = fixtures::gear::read_as_found();
    let pinion = fixtures::solid(&e,"pair.pinion.body");
    let admission = admit_body(&e.sketch,pinion,&Options::default()).unwrap_or_else(|r| panic!("refused: {r}"));
    let recipe = gcs_core::solid::cad::recipe_static(&e.sketch,pinion).unwrap();
    let (fitted,said) = saying(|say| {
        let prepared = sweep::prepare(&e.sketch,pinion,&recipe,&admission,say).unwrap_or_else(|r| panic!("{}",r.message));
        let k = (0..prepared.sweeps()).find(|&k| prepared.sweep_name(&e.sketch,k) == "pair.pinion.cut.removal")
            .expect("the pinion's cut");
        sweep::traced(&prepared,k,None,say)
    });
    let fitted = fitted.unwrap_or_else(|r| panic!("{}\n{said}",r.message));
    the_pinions_sheet(&said);
    assert!(fitted.error < 0.01,"the fit is {} mm from its contacts\n{said}",fitted.error);
}

/// **The pinion at the mill its balance finds is built by sectors** (#904 A1/A2): its cut one chart
/// (`the_pinions_cut_at_the_found_mill_is_one_chart`), each sector the blank's pieces, the space's
/// five faces and the relief's five, as at the mill it was recorded at
/// (`brep_export::the_pinion_as_written_is_built_by_sectors`). At this mill the tip relief the inner
/// flank keeps is about a hundredth of a millimetre thick at the tip (0.026 mm³ a sector, where the
/// recorded mill left 0.084), thinner than the sag of the mesh its interior was sought on, and the
/// cell was refused for having no interior sample; it is read again on a finer mesh
/// (`brep::query::interior`).
#[test]
fn the_pinion_at_the_found_mill_is_built_by_sectors() {
    let e = fixtures::gear::read_as_found();
    let (exact,said) = built(&e,"pair.pinion.body");
    assert!(said.contains("classified 1 material and 4 removed cells"),"{said}");
    the_pinion_by_sectors(&exact,&said);
    let volume = props::volume(&exact.solid);
    assert!((volume-14664.2385).abs() < 1e-3,"{volume} mm³");
}
