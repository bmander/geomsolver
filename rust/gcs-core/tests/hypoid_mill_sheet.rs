//! The configured hypoid pinion cut by the face mill its balance finds (#904 A1/A2), the mill stated:
//! its pressure shift and its cutter's radius written into the pair's call as numbers
//! (`fixtures::gear::machine`), so what the kernel builds there is judged apart from the balance that
//! finds them (`hypoid_machine`).
use gcs_core::brep::sweep::{self,Say};
use std::sync::Mutex;

/// The face mill the configured pair's balance finds: its pressure shift (degrees), where the
/// pinion's flank the inner blade cuts ties the gear's the outer blade cuts, and the cutter's radius
/// the document states (mm).
const MILL: [f64;2] = [8.8549,75.6122];

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
    let e = fixtures::gear::read(&fixtures::gear::machine(&fixtures::gear::source(),MILL[0],MILL[1]));
    let pinion = fixtures::solid(&e,"pair.pinion.body");
    let admission = admit_body(&e.sketch,pinion,&Options::default()).unwrap_or_else(|r| panic!("refused: {r}"));
    let recipe = gcs_core::solid::cad::recipe_static(&e.sketch,pinion).unwrap();
    let lines = Mutex::new(Vec::new());
    let say = Say {stage:&|l: &str| lines.lock().unwrap().push(l.to_string()),mark:&|_| {}};
    let prepared = sweep::prepare(&e.sketch,pinion,&recipe,&admission,&say).unwrap_or_else(|r| panic!("{}",r.message));
    let k = (0..prepared.sweeps()).find(|&k| prepared.sweep_name(&e.sketch,k) == "pair.pinion.cut.removal").expect("the pinion's cut");
    let fitted = sweep::traced(&prepared,k,None,&say);
    let said = lines.lock().unwrap().join("\n");
    let fitted = fitted.unwrap_or_else(|r| panic!("{}\n{said}",r.message));
    assert!(said.contains("`pair.pinion.cut.removal`: sheet 69x39, one chart"),"{said}");
    assert!(said.contains("`pair.pinion.cut.removal`: the sheet's faces, by the cutter's: inner, inner_round, tip, outer_round, outer"),
        "{said}");
    assert!(fitted.error < 0.01,"the fit is {} mm from its contacts\n{said}",fitted.error);
}
