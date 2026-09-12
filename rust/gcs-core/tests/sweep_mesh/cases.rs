//! The case table: a case is one struct literal, and `sweep_cases!` makes a
//! `#[test]` of each row, so it is filterable and runs beside the others. Every
//! row is built, held to a closed shell with every triangle certified, and
//! measured against a closed form in `forms` that knows nothing of the
//! construction. A case that does not close yet is not a row: it keeps its own
//! `#[ignore]`d test with the evidence of what it is waiting for.
use super::{closed::{closed,shell_at,volume},creases,forms,motions,tools};

const SAGITTA: f64 = 0.02;

// A row's document, where no other module already holds it. The three crease cases take theirs
// from `creases`, which shares them with the export.
fn slid_box() -> String { format!("{}{}{}",tools::BOX,motions::slide_x(10.),motions::swept("feed",0.,360.)) }

/// What a case's volume is judged against, and how closely.
pub(super) enum Reference {
    /// A straight-edged sweep: the mesh is the solid, to rounding.
    Exact(f64),
    /// A round sweep, whose mesh is inscribed: its columns are inscribed
    /// polygons and its seams chamfer a column's width, so it falls short by
    /// at most three sagittas per unit of the least radius, and never exceeds
    /// the form.
    Inscribed { volume: f64,least_radius: f64 },
}

/// One case: where its document comes from, the chord error it is built to,
/// and the truth it is measured against.
pub(super) struct Case {
    pub name: &'static str,
    pub source: fn() -> String,
    pub sagitta: f64,
    /// A function, not a number: a form costs a quadrature, and a row that is
    /// filtered out must not pay for it.
    pub reference: fn() -> Reference,
}

/// Build the case, hold it to a closed shell with every triangle certified,
/// and measure its volume against its reference.
pub(super) fn run(case: &Case) {
    let source = (case.source)();
    let (mesh,certificate) = shell_at(&source,case.sagitta);
    let certificate = certificate.unwrap_or_else(|e| panic!("{}: construction refused: {e:?}",case.name));
    assert!(certificate.is_complete(),"{}: {} triangles failed",case.name,certificate.failures.len());
    closed(&mesh).unwrap_or_else(|e| panic!("{}: not a closed shell: {e}",case.name));
    let v = volume(&mesh);
    match (case.reference)() {
        Reference::Exact(expected) => {
            assert!((v-expected).abs() <= 1e-9*expected,"{}: volume {v} against {expected}",case.name);
        }
        Reference::Inscribed {volume: expected,least_radius} => {
            eprintln!("{}: volume {v:.4}, expected {expected:.4}",case.name);
            assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*case.sagitta/least_radius),
                "{}: volume {v} against {expected}",case.name);
        }
    }
}

macro_rules! sweep_cases {
    ($($test:ident: $case:expr;)*) => { $(
        #[test]
        fn $test() { run(&$case); }
    )* };
}

sweep_cases! {
    // straight-edged, so the mesh is the solid: the 2 x 3 x 2 box advanced 10 along x is
    // 12 + 6 * 10, and every line along x meets the tool in one segment
    a_box_translated_along_x_sweeps_a_longer_box: Case {
        name:"box slid along x",source:slid_box,sagitta:SAGITTA,
        reference:|| Reference::Exact(72.) };
    a_turning_prism_closes_along_its_creases: Case {
        name:"turning prism",source:creases::turning_prism,sagitta:SAGITTA,
        reference:|| Reference::Inscribed {volume:forms::turning_prism_volume(),least_radius:0.5} };
    a_box_turned_about_its_face_centre_closes_along_its_creases: Case {
        name:"turned box",source:creases::turned_box,sagitta:SAGITTA,
        reference:|| Reference::Inscribed {volume:forms::turned_box_volume(),least_radius:1.} };
    a_lens_turned_about_the_spindle_closes_along_its_crease: Case {
        name:"turned lens",source:creases::turned_lens,sagitta:SAGITTA,
        reference:|| Reference::Inscribed {volume:forms::turned_lens_volume(),least_radius:0.5} };
}
