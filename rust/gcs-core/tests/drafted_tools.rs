//! **Drafted prisms in the generating class** (#684 G, docs/generating-sweeps.md): a drafted
//! prism's side is a plane where its edge is a line and a cone where it is an arc, the ruled
//! surface between its edge's two sections (`ExtrudedSurface::drafted`), and a point of it has a
//! normal of its own as an undrafted side's has, so its contact is read in closed form as theirs
//! is. The rack's tooth drafted cuts a tooth space that narrows through the blank; leaned across
//! the blank as a helical rack's teeth are (`solid_sweeps/helical.sv`, held in
//! `solid_sweeps_examples.rs`), its motion keeps no plane of the tool, so only this class builds
//! it. What each removes is computed here — each section of the blank square to its axis is cut
//! by the tooth's section there, a quadrilateral read from the tooth's four drafted planes, and
//! swept by `rack_cut::space_area`'s arithmetic — and held to the exact solid, whose STL is held
//! to the field. A drafted prism running both ways from its plane (#684 P,
//! `solid_sweeps/parted.sv`) folds there, each side's two halves meeting in a convex crease whose
//! fan carries the contact from one half's envelope to the other's; its sections are each the
//! tooth's eight half-planes, every side's material below the plane and above it. Drafted outward
//! both ways, the crease is concave, and an arc meeting a side at an angle moves its corner along a
//! curve: both are refused at T1 by name.
use gcs_core::solid::ReadExtruded;
use gcs_core::solid::admission::{self,Class,Condition,Options};
use gcs_core::solid::{ExtrudedSurface,SweepContacts,ToolSurface};
use gcs_core::space::{cross,dot,unit};
use crate::rack_cut::{space_area,TOOTH};
use crate::turns_and_slides::{exported,refused};
use std::f64::consts::PI;

/// The rack-cut tooth space (`rack_cut::SPACE`), its tooth drafted `draft` from its own plane, the
/// front, and run from it to 9 behind; the blank from 1 to 7 behind the front.
fn drafted_space(draft: &str) -> String {
    crate::rack_cut::SPACE
        .replace("blank := solid(disc, depth: 6mm)","blank := solid(disc, from: -7mm, to: -1mm)")
        .replace("solid(tooth, from: -8mm, to: 2mm)",&format!("solid(tooth, from: -9mm, to: 0mm, draft: {draft})"))
}

/// The rack-cut tooth space, its tooth drafted `draft` both ways from its own plane, the front, and
/// run 7 either side of it; the blank 3 either side of the front.
fn parted_space(draft: &str) -> String {
    crate::rack_cut::SPACE
        .replace("blank := solid(disc, depth: 6mm)","blank := solid(disc, from: -3mm, to: 3mm)")
        .replace("solid(tooth, from: -8mm, to: 2mm)",&format!("solid(tooth, from: -7mm, to: 7mm, draft: {draft})"))
}

/// `solid_sweeps/helical.sv`: the tooth leaned 15° across the blank and drafted 2°, the blank from
/// 2 to 8 behind the front.
fn helical() -> String { crate::solid_sweeps_examples::text("helical.sv") }

/// The tooth's section by the plane at `y` (in the front's coordinates, x and z), clockwise: the
/// tooth drawn in a plane through the origin leaned `lean` degrees about x from the front (its v
/// turned from z toward -y), drafted `draft` degrees on its normal's negative side. A point's place
/// in that plane is (x, -y sin + z cos) and its depth along the normal -y cos - z sin, so each side's
/// material — where the drawn side's line moved in by the depth times the draft's tangent holds it —
/// is a half-space, and the plane cuts it in a half-plane; the four meet in the section.
fn section(lean: f64,draft: f64,y: f64) -> Vec<(f64,f64)> {
    let (sides,n) = (half_planes(lean,draft,y,-1.),TOOTH.len());
    let mut corners: Vec<(f64,f64)> = (0..n).map(|i| {
        let ([a1,b1,c1],[a2,b2,c2]) = (sides[(i+n-1)%n],sides[i]);
        let w = a1*b2-a2*b1;
        ((b1*c2-b2*c1)/w,(a2*c1-a1*c2)/w)
    }).collect();
    if twice_area(&corners) > 0. { corners.reverse(); }
    corners
}

/// Each side's material, in the plane at `y` from the tooth's own (whose point at its origin's
/// depth is `y` along the front's normal), as α x + γ z + κ >= 0: m·(place − a) − way·tan·depth >= 0,
/// m the side's inward normal — the side moved in by the depth (`way` 1) or by the depth's opposite
/// (`way` −1, the material on the normal's negative side, where the depth is negative).
fn half_planes(lean: f64,draft: f64,y: f64,way: f64) -> Vec<[f64;3]> {
    let (s,c) = lean.to_radians().sin_cos();
    let t = draft.to_radians().tan();
    let n = TOOTH.len();
    let (cu,cv) = (TOOTH.iter().map(|p| p.0).sum::<f64>()/n as f64,TOOTH.iter().map(|p| p.1).sum::<f64>()/n as f64);
    (0..n).map(|i| {
        let ((au,av),(bu,bv)) = (TOOTH[i],TOOTH[(i+1)%n]);
        let l = (bu-au).hypot(bv-av);
        let (mut mu,mut mv) = (-(bv-av)/l,(bu-au)/l);
        if mu*(cu-au)+mv*(cv-av) < 0. { (mu,mv) = (-mu,-mv); }
        [mu,mv*c+way*t*s,-mu*au+mv*(-y*s-av)+way*t*y*c]
    }).collect()
}

/// Twice a polygon's signed area, counter-clockwise positive.
fn twice_area(p: &[(f64,f64)]) -> f64 { (0..p.len()).map(|i| p[i].0*p[(i+1)%p.len()].1-p[(i+1)%p.len()].0*p[i].1).sum() }

/// The space removed between the sections at `ys`: each section's area, which the tooth's planes
/// make smooth in `y`, by four-point Gauss–Legendre across them.
pub(crate) fn space_volume(lean: f64,draft: f64,ys: [f64;2]) -> f64 {
    let (m,h) = (0.5*(ys[0]+ys[1]),0.5*(ys[1]-ys[0]));
    crate::common::rounds::gauss_legendre(4).into_iter().map(|(x,w)| h*w*space_area(&section(lean,draft,m+h*x),400,4000)).sum()
}

/// The tooth's section by the plane at `y`, clockwise, where it is drawn in the plane through
/// (0, `parting`, 0) leaned `lean` degrees as `section`'s, and drafted `draft` degrees both ways
/// from it (#684 P). A point's depth `h` from that plane is `-(y - parting) cos - z sin`, and each
/// side's material is where the drawn side's line moved in by `|h|` times the draft's tangent holds
/// it: both half-spaces, the one moved in by `h` and the one by `-h`. The eight cut the plane in
/// half-planes, and the section is what of a large square all eight keep.
fn parted_section(lean: f64,draft: f64,parting: f64,y: f64) -> Vec<(f64,f64)> {
    let mut kept = vec![(-100.,-100.),(100.,-100.),(100.,100.),(-100.,100.)];
    for [a,g,k] in [1.,-1.].into_iter().flat_map(|way| half_planes(lean,draft,y-parting,way)) {
        let f = |p: (f64,f64)| a*p.0+g*p.1+k;
        // Sutherland–Hodgman: what of the polygon the half-plane keeps
        let mut next = Vec::new();
        for j in 0..kept.len() {
            let (p,q) = (kept[j],kept[(j+1)%kept.len()]);
            let (fp,fq) = (f(p),f(q));
            if fp >= 0. { next.push(p); }
            if (fp >= 0.) != (fq >= 0.) {
                let r = fp/(fp-fq);
                next.push((p.0+r*(q.0-p.0),p.1+r*(q.1-p.1)));
            }
        }
        kept = next;
    }
    if twice_area(&kept) > 0. { kept.reverse(); }
    kept
}

/// The space the tooth drafted both ways from its plane through (0, `parting`, 0) removes between
/// the sections at `ys` (`parted_section`): each section's area by four-point Gauss–Legendre, over
/// pieces of `ys` split where the plane's crease across the tooth passes a corner of its profile —
/// at `parting − v sin lean` for each corner's `v` — between which the section's sides do not
/// change and its area is smooth in `y`. Each area is read at four times `space_volume`'s radii: the
/// draft lifts the tooth's tip off the root circle, a step in what each radius loses that the
/// radii's midpoints read at first order (at 400 a both-ways section's space errs by 5e-4, at 1600
/// by 1e-4); and the sections side by side, on every core. Unleaned, the crease passes every corner
/// at once, at the plane, and the pieces are its two sides.
pub(crate) fn parted_volume(lean: f64,draft: f64,parting: f64,ys: [f64;2]) -> f64 {
    let s = lean.to_radians().sin();
    let mut breaks: Vec<f64> = TOOTH.iter().map(|p| parting-p.1*s).filter(|&y| y > ys[0] && y < ys[1]).collect();
    breaks.extend(ys);
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    let nodes: Vec<(f64,f64)> = breaks.windows(2).flat_map(|w| {
        let (m,h) = (0.5*(w[0]+w[1]),0.5*(w[1]-w[0]));
        crate::common::rounds::gauss_legendre(4).into_iter().map(move |(x,w)| (m+h*x,h*w))
    }).collect();
    gcs_core::par::map(&nodes,|&(y,w)| w*space_area(&parted_section(lean,draft,parting,y),1600,4000)).into_iter().sum()
}

/// **A section drafted both ways is the one-way section mirrored in depth**: with the tooth
/// unleaned, a section at depth `d` from the parting plane is the one-way section `section` reads
/// at depth `|d|` (its own arithmetic, four half-planes meeting at its corners), either side.
#[test]
fn a_section_drafted_both_ways_mirrors_the_one_way_section() {
    let area = |p: &[(f64,f64)]| -twice_area(p)/2.;
    for d in [0.5,2.,4.] {
        let one = area(&section(0.,2.,d));
        for y in [5.-d,5.+d] {
            let both = parted_section(0.,2.,5.,y);
            assert_eq!(both.len(),4,"{both:?}");
            assert!((area(&both)-one).abs() < 1e-12*one,"at {y}: {} against {one}",area(&both));
        }
    }
}

/// **The drafted rack's tooth space** (#684 G): the generating class, where its draft was refused
/// at T1; its sheet traced (no longer its profile's planar envelope extruded),
/// the tooth's sections walked by its profile's lengths so a row is a ruling of each side. The
/// space narrows as the tooth thins: 72.06 mm³, where the undrafted tooth's through the same 6 mm
/// is 80.68.
#[test]
fn a_drafted_rack_tooth_cuts_a_narrowing_space() {
    let e = fixtures::read(&drafted_space("2deg"));
    assert!(matches!(crate::solid_sweeps_examples::class(&e,"gear"),Class::Generating),"the generating class");
    let removed = PI*22.*22.*6.-exported(&e,"gear");
    // undrafted, every section is the drawn tooth
    let (want,square) = (space_volume(0.,2.,[1.,7.]),6.*space_area(&TOOTH,400,4000));
    eprintln!("removed {removed:.5} mm³, the drafted space {want:.5}, the undrafted {square:.5}");
    assert!(square-want > 8.,"the draft narrows the space");
    assert!((removed-want).abs() < 2e-4*want,"removed {removed:.5} mm³, the space being {want:.5}");
}

/// **The rack's tooth drafted both ways from its plane** (#684 P): the generating class, where T1
/// refused it and the planar subtraction class read it instead; its sides fold at the plane, the
/// middle of the blank, each half's envelope joined to the other's by the crease's fan, the
/// stations crossing it between the halves. The motion keeps the tooth's plane, so every normal of
/// the fan is in contact at the instant both halves are: the fan sweeps nothing, and the halves'
/// envelopes meet in a ridge the fitted sheet is refined across. The space is widest at the middle
/// and narrows toward either face, each section the one-way tooth's at the same depth from the
/// plane: 77.40 mm³, where the undrafted tooth's is 80.68.
#[test]
fn a_rack_tooth_drafted_both_ways_cuts_a_space_widest_at_its_parting_plane() {
    let e = fixtures::read(&parted_space("2deg"));
    assert!(matches!(crate::solid_sweeps_examples::class(&e,"gear"),Class::Generating),"the generating class");
    let contacts = SweepContacts::read(&e.sketch,fixtures::solid(&e,"space"),1e-10).unwrap();
    assert_eq!((contacts.patches().len(),contacts.creases().len()),(8,4),"each side two halves, folding between them");
    let removed = PI*22.*22.*6.-exported(&e,"gear");
    let want = parted_volume(0.,2.,0.,[-3.,3.]);
    eprintln!("removed {removed:.5} mm³, the space {want:.5}, the undrafted {:.5}",6.*space_area(&TOOTH,400,4000));
    assert!((removed-want).abs() < 2e-4*want,"removed {removed:.5} mm³, the space being {want:.5}");
}

/// **A drafted side is a plane or a cone of the tool** (`ExtrudedSurface::drafted`): a slot drafted
/// 2° from its plane, two lines and two half circles going on tangent, has four sides, each the
/// ruled surface between its two sections. Every point read on one lies on the tool's boundary (the
/// field reads a drafted prism's sides exactly) with the field's normal, and a line's normal is
/// one direction, leaning the draft from the plane, where an arc's turns round its cone.
#[test]
fn a_drafted_side_is_a_plane_or_a_cone_of_the_tool() {
    let src = "\
unit mm
use std
use std.motion (spins, slides)
in std.front {
  o := point
  fix((0, 0)) o
  c0 := point
  c1 := point
  fix((16, 0)) c0
  fix((26, 0)) c1
  s0 := point hint((16, -3))
  s1 := point hint((26, -3))
  s2 := point hint((26, 3))
  s3 := point hint((16, 3))
  outline := (lower := line(s0, s1)) -> tangent (east := arc(center: c1)) ->
             tangent (upper := line(s2, s3)) -> tangent (west := arc(center: c0)) -> close
  radius(3mm) east
  radius(3mm) west
  horizontal lower
  p0 := point hint((20, 0))
  p1 := point hint((20, 10))
  fix((20, 0)) p0
  fix((20, 10)) p1
  pitch_line := line(p0, p1)
}
construction slot := solid(outline, from: -12mm, to: 0mm, draft: 2deg)
turn := frame
turn spins(1) o
rack := frame
rack slides(2 * pi * 20mm) pitch_line
construction space := sweep(slot over t in (-60deg, 60deg), under: rack, seen_from: turn)
";
    let e = fixtures::read(src);
    let sides = ExtrudedSurface::drafted(&e.sketch,fixtures::solid(&e,"slot")).unwrap();
    let mut names: Vec<&str> = sides.iter().flatten().flatten().map(|s| s.name.as_str()).collect();
    names.sort();
    assert_eq!(names,["slot.east","slot.lower","slot.upper","slot.west"]);
    let contacts = SweepContacts::read(&e.sketch,fixtures::solid(&e,"space"),1e-10).unwrap();
    let tool = contacts.source_material();
    let lean = 2f64.to_radians();
    for surface in contacts.patches() {
        let ToolSurface::Extruded(side) = surface else { panic!("a prism's side") };
        let mut normals = Vec::new();
        // short of the caps, where the field's gradient is a cap's too
        for i in 0..=8 { for j in 1..8 {
            let p = side.at(i as f64/8.,j as f64/8.).unwrap();
            let n = unit(cross(p.du,p.dv));
            let h = 1e-6;
            let grad = unit(std::array::from_fn(|k| {
                let (mut a,mut b) = (p.position,p.position);
                a[k] += h; b[k] -= h;
                (tool.value(a)-tool.value(b))/(2.*h)
            }));
            assert!(tool.value(p.position).abs() < 1e-9,"{} at ({i}, {j}) is off the tool by {}",side.name,tool.value(p.position));
            assert!(dot(n,grad).abs() > 1.-1e-6,"{} at ({i}, {j}): its normal {n:?} against the field's {grad:?}",side.name);
            normals.push(n);
        }}
        // the side leans the draft from the page's normal (-y), a cone's all round as a plane's
        assert!(normals.iter().all(|n| (n[1].abs()-lean.sin()).abs() < 1e-9),"{}: {normals:?}",side.name);
        let turned = normals.iter().any(|&n| dot(n,normals[0]) < 1.-1e-6);
        assert_eq!(turned,side.name.ends_with("east") || side.name.ends_with("west"),"{}",side.name);
    }
}

/// **What a drafted tool the class does not read is refused at T1**, saying why: `parted.sv`'s
/// tooth drafted outward both ways from its plane, its sides meeting there in a concave crease that
/// would trim the two halves' envelopes against each other, and the tooth drawn as a half disc, its
/// arc meeting its diameter at a right angle, a corner a draft moves along a curve. (Drafted 0.2°:
/// the faceted reading of the half disc refuses a deeper draft, which eats the arc's facet beside
/// that corner, before any class is asked.)
#[test]
fn a_drafted_tool_the_class_does_not_read_is_refused_at_t1() {
    let outward = crate::solid_sweeps_examples::text("parted.sv").replace("draft: 2deg","draft: -2deg");
    let r = refused({
        let e = fixtures::read(&outward);
        admission::admit_body(&e.sketch,fixtures::solid(&e,"gear"),&Options::default())
    });
    assert_eq!(r.condition,Condition::Tool);
    assert!(r.message.starts_with("`tooth` is a drafted prism whose sides the class does not read: it runs both ways from its \
        profile's plane, drafted outward"),"{}",r.message);
    let half_disc = helical()
        .replace("  t0 := point hint((18, -0.84))\n","")
        .replace("  t1 := point hint((18, 0.84))\n","")
        .replace("  fix((18, -0.842856)) t0\n","")
        .replace("  fix((18, 0.842856)) t1\n","")
        .replace("solid(face(t0, t1, t2, t3, -> close), from: -12mm, to: 0mm, draft: 2deg)",
            "solid(face(chord, dome), from: -12mm, to: 0mm, draft: 0.2deg)")
        .replace("// the blank 6 thick","in inclined {\n  hub := point\n  fix((24, 0)) hub\n  dome := arc(center: hub, start: t2, end: t3)\n  \
            chord := line(t3, t2)\n}\n// the blank 6 thick");
    let r = refused({
        let e = fixtures::read(&half_disc);
        admission::admit_body(&e.sketch,fixtures::solid(&e,"gear"),&Options::default())
    });
    assert_eq!((r.condition,r.class()),(Condition::Tool,"generating-sweep class"));
    assert!(r.message.contains("an arc meets a side at an angle"),"{}",r.message);
}
