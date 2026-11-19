//! **Where a body is made, and a face's measures** (#953): a solid kept within a half-space is
//! the body in common with it, built exactly (`solid::defined::trimmed`, the kernel's `half`
//! node); its plane's height may be an unknown a measure drives (`interior::Sheet::Ordinate`); and
//! the drawing states where a body's volume or mass is centred, `g centroid B`, `g centerofmass
//! B`, a face's area, `area(A) F`, and its second moment about a line, `moment(I, about: l) F` —
//! each a row of the drawing over the body's filled mesh moved by its sheets, or a claim judged on
//! the solved drawing.  Each is held to its closed form: a box barge, a cylinder floating at a
//! draft, a wedge heeled about its apex.  The rules are `docs/contract/analysis.md`'s.
use gcs_core::constraints::CKind;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;

/// A box barge `L` long and `B` in beam, 100 deep, on the top plane its keel's middle at the
/// origin, kept within the water below the waterline `T` above it (an unknown), and after it
/// `lines` of the drawing's own.
pub fn barge(lines: &str) -> String {
    format!("\
unit mm
use std
use std.solids
use std.sets
param T: Length hint(30mm)
L := 200mm
B := 60mm
in std.top {{
  a := point
  b := point
  c := point
  d := point
  fix((0mm, -B / 2)) a
  fix((L, -B / 2)) b
  fix((L, B / 2)) c
  fix((0mm, B / 2)) d
  side := line(a, b)
  stern := line(b, c)
  other := line(c, d)
  bow := line(d, a)
  plan := face(side, stern, other, bow)
}}
hull := solid(std.solids.Prism(plan, from: 0mm, to: 100mm))
keel := point
fix((0mm, 0mm, 0mm)) keel
water := std.sets.HalfSpace(keel, std.z, d: T)
immersed := solid({{ p | p in hull; p in water }})
fore := point
aft := point
fix((0mm, 0mm, 0mm)) fore
fix((L, 0mm, 0mm)) aft
centreline := line(fore, aft)
{lines}
")
}

/// `src` elaborated with no error and solved.
fn sized(src: &str) -> (Elaborated, Sketch) {
    let e = crate::common::read(src);
    let sk = crate::common::solved(&e);
    (e, sk)
}

/// What the unknown `name` came to.
fn unknown(sk: &Sketch, name: &str) -> f64 {
    sk.params[sk.free_vars[name] as usize].value
}

/// What the report says of each measure the drawing states, each its sentence.
fn said(e: &Elaborated, sk: &Sketch) -> Vec<String> {
    gcs_core::report::measure_answers(sk, &e.measures).into_iter().map(|a| a.text).collect()
}

#[test]
fn probe() {
    let src = barge("volume(L * B * 40mm) immersed\ng := point hint((100, 0, 20))\ng centroid immersed\n");
    let (e, sk) = sized(&src);
    eprintln!("T = {}", unknown(&sk, "T"));
    for c in sk.constraints.iter().filter(|c| c.kind.interior()) {
        let v = c.local_values(&sk);
        let r = c.residual(&sk, &v);
        let ja = c.jacobian(&sk, &v);
        let jn = gcs_core::fdcheck::fd_jacobian(|w| c.residual(&sk, w), &v, 1e-3);
        eprintln!("{:?} {:?}: r {:?}\n  v {:?}\n  ja {:?}\n  jn {:?}", c.kind, c.args, r, v, ja,
                  jn.data);
    }
    eprintln!("{:#?}", said(&e, &sk));
    panic!();
}
