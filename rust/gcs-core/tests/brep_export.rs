//! A solid's exact export by this kernel (`brep::export`, phase 5 of docs/rust-kernel-plan.md), the
//! functions `solventc` and the browser's worker both call: a body whose swept cut is placed once is
//! no sector, and is built whole — the blank split by the placed sheet, its cells judged by the
//! material field — then written, parsed back, meshed and held to the field.
use crate::common::{built,saying};
use crate::hypoid_mill_sheet::{the_pinion_by_sectors,the_pinions_sheet};
use gcs_core::brep::{export,props,sweep::Say};
use gcs_core::model::Sketch;

fn example(name: &str) -> Sketch {
    let (_,e) = fixtures::examples().into_iter().find(|(n,_)| n == name).unwrap();
    let mut sk = e.unwrap().sketch;
    gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
    sk
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 19 s: a placed sweep exported")]
fn a_sweep_placed_once_is_built_whole_and_exported() {
    let sk = example("swept_torus.sv");
    let body = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "part").unwrap();
    let lines = std::sync::Mutex::new(Vec::new());
    let say = Say {stage:&|l: &str| lines.lock().unwrap().push(l.to_string()),mark:&|_| {}};
    let exact = export::exact(&sk,body,None,None,&say).unwrap();
    let said = lines.lock().unwrap().join("\n");
    assert!(exact.swept && exact.pattern.is_none() && exact.indexed.is_none(),"{said}");
    assert!(said.contains("built whole, not as one sector: the sweep is placed 1 times"),"{said}");
    assert!(said.contains("classified 1 material and 1 removed cells"),"{said}");
    exact.solid.check(1e-6).unwrap();
    // the blank (0.753982 mm³) less the removal, which OCCT's reading of the file measures as
    // 0.447407 mm³ at 0.1 µm
    let volume = props::volume(&exact.solid);
    assert!((volume-0.447407).abs() < 2e-5,"{volume}");
    let step = export::step(&exact,"part",None,&say).unwrap();
    assert!(step.starts_with("ISO-10303-21;"));
    let stl = export::stl(&sk,body,&exact,None,&say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    let said = lines.lock().unwrap().join("\n");
    assert!(said.contains("0 disagree"),"{said}");
}

/// The page's exact surface of a swept body (`export::Builder`), built a stage at a time — each
/// stage saying what the next does, as a worker shows it — and supplied to a sketch in place of
/// the field's surface: its volume the B-rep's, its faces meeting at creases.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 17 s: a sweep built by stages")]
fn a_swept_bodys_exact_surface_is_built_by_stages_and_supplied() {
    let sk = example("swept_torus.sv");
    let body = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "part").unwrap();
    let mut builder = export::Builder::new(&sk,body).unwrap();
    let mut doing = vec![builder.doing()];
    let (_,total) = builder.stages();
    while !builder.step().unwrap() { doing.push(builder.doing()); }
    assert_eq!(builder.stages(),(total,total),"{doing:?}");
    assert_eq!(doing[0],"admitting its sweeps to the generating class");
    assert!(doing.iter().any(|d| d.starts_with("fitting the sheet of `removal`")),"{doing:?}");
    assert!(doing.iter().any(|d| d == "cutting the blank by its sheets"),"{doing:?}");
    assert!(builder.said().is_some());
    let d = builder.display().unwrap().clone();
    assert_eq!(d.faces.of.len(),d.triangles.len());
    assert!((d.faces.volume-0.447407).abs() < 2e-5,"{}",d.faces.volume);
    // supplied in place of the field's surface, the sketch draws it and never meshes the field
    sk.field_meshing.set(gcs_core::solid::FieldMeshing::Deferred);
    sk.supply_field(body,d.surface()).unwrap();
    let solid = sk.evaluated_solid(body,gcs_core::solid::ApproximationPolicy::Mesh).unwrap();
    assert!(!solid.provisional());
    assert_eq!(solid.volume(),d.faces.volume);
    // each face by the name its build gave it
    assert!(!solid.surviving_faces().is_empty() && solid.surviving_faces().iter().all(|f| d.faces.names.contains(f)),
        "{:?} against {:?}",solid.surviving_faces(),d.faces.names);
    // its faces meet at creases (the cut's rim on the post), and a curved face's seams are smooth
    assert!(solid.edges().iter().any(|e| !e.smooth));
    assert!(solid.edges().iter().any(|e| e.smooth));
    // a provisional exact surface is refused
    let bad = gcs_core::solid::FieldSurface {provisional:true,..sk.supplied_field(body).unwrap().as_ref().clone()};
    assert!(sk.supply_field(body,bad).unwrap_err().contains("never provisional"));
}

/// A 2.03 × 2 × 2 mm box whose near face stands at x = `x0` mm (issue #60).
fn far_box(x0: f64) -> Sketch {
    let x1 = x0+2.03;
    let src = format!("\
unit mm
use std
use std.solids (cut, common)
in std.front {{
p0 := point
fix(({x0}, 0)) p0
p1 := point
fix(({x1}, 0)) p1
p2 := point
fix(({x1}, 2)) p2
p3 := point
fix(({x0}, 2)) p3
}}
section := face(p0, p1, p2, p3, -> close)
body := solid(std.solids.Prism(section, from: 0mm, to: 2mm))
");
    let (p,errors) = crate::common::parse(&src);
    assert!(errors.is_empty(),"{errors:?}");
    let e = gcs_core::program::elaborate(&p);
    assert!(e.ok(),"{:?}",e.diags);
    e.sketch
}

/// The x extent of a binary STL's written coordinates, decoded from its bytes.
fn written_x(stl: &[u8]) -> (f64,f64) {
    let n = u32::from_le_bytes(stl[80..84].try_into().unwrap()) as usize;
    let xs = (0..n).flat_map(|i| (0..3).map(move |k| {
        let at = 84+50*i+12+12*k;
        f32::from_le_bytes(stl[at..at+4].try_into().unwrap()) as f64
    }));
    xs.fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),x| (lo.min(x),hi.max(x)))
}

/// The tolerance is about the surface written, and binary STL writes float32: a box far from the
/// origin, whose mesh sags nothing, is moved by the encoding alone (30 µm at x = 10⁶ mm). The
/// rounding is counted with the sag, so an STL is written within its share of the tolerance —
/// its faces read back from the bytes where the model has them — or refused, and never given
/// back moved further.
#[test]
fn float32_rounding_is_spent_from_the_tolerance_of_the_written_stl() {
    let say = Say {stage:&|_| {},mark:&|_| {}};
    for (x0,tolerance,written) in [(0.03,0.001,true),(100_000.003,0.001,false),(100_000.003,0.01,true),
        (1_000_000.03,0.001,false),(1_000_000.03,0.01,false)] {
        let sk = far_box(x0);
        let tolerance = gcs_core::solid::export::Tolerance::new(tolerance).unwrap();
        let exact = export::exact(&sk,0,None,Some(tolerance),&say).unwrap();
        let what = format!("x = {x0} mm within {} µm",tolerance.millimetres*1e3);
        match export::stl(&sk,0,&exact,Some(tolerance),&say) {
            Ok(stl) => {
                assert!(written,"{what}: written");
                let rounding = gcs_core::mesh::stl_rounding(&stl).unwrap();
                assert!(rounding <= tolerance.deflection(),"{what}: {rounding}");
                let (lo,hi) = written_x(&stl);
                let off = (lo-x0).abs().max((hi-(x0+2.03)).abs());
                assert!(off <= tolerance.deflection(),"{what}: the faces written at {lo} and {hi}, {off} mm off");
            }
            Err(e) => {
                assert!(!written,"{what}: {e}");
                assert_eq!(e.stage,gcs_core::solid::export::Stage::Stl,"{what}: {e}");
                assert!(e.message.contains("float32"),"{what}: {e}");
            }
        }
    }
    // the bound is half a float32 step: 1/32 mm on one axis at 10⁶ mm, nothing at the origin
    let one = |p: [f64;3]| gcs_core::mesh::stl_rounding(&gcs_core::mesh::stl_of(&[p,[p[0]+1.,p[1],p[2]],[p[0],p[1]+1.,p[2]]]
        .concat(),"t")).unwrap();
    assert!((one([1_000_000.03,0.,0.])-1./32.).abs() < 1e-9);
    assert!(one([0.,0.,0.]) < 1e-7);
}

/// Issue #59's stock: a 10 mm disc 2 mm thick about z, its plane's axes `u`, `v` (turned 11.25°
/// where written so), cut by a 2 mm strip `cutter` (a `std.solids` word over its face `a, b, c, d`)
/// along x through it — `std.solids.Through` — or from −11 to 11 mm.
fn strip_cut(frame: [[f64; 3]; 2], cutter: &str, cut: bool) -> String {
    let [u, v] = frame;
    format!("unit mm
use std
use std.solids (cut, common)
su := axis hint(dir: ({}, {}, {}))
fix(dir == ({}, {}, {})) su
sv := axis hint(dir: ({}, {}, {}))
fix(dir == ({}, {}, {})) sv
stock_plane := plane(u: su, v: sv)
fix(origin == (0, 0, 0)) stock_plane
tool_plane := plane(u: std.y, v: std.z)
fix(origin == (0, 0, 0)) tool_plane
in stock_plane {{
  rim := radius(10mm) circle(center: stock_plane.origin) hint(r: 10)
  stock := solid(std.solids.Prism(face(rim), from: 0mm, to: 2mm))
}}
in tool_plane {{
  a := point hint((-1, -1))
  b := point hint((1, -1))
  c := point hint((1, 3))
  d := point hint((-1, 3))
  fix((-1, -1)) a
  fix((1, -1)) b
  fix((1, 3)) c
  fix((-1, 3)) d
  cutter := solid({cutter})
}}
body := solid(stock)
{}
", u[0], u[1], u[2], u[0], u[1], u[2], v[0], v[1], v[2], v[0], v[1], v[2],
        if cut { "cutter cut body" } else { "" })
}

/// A `through:` cutter spans its stock wherever the stock's circle starts (issue #59: turned by
/// 11.25°, every sample of the old box missed the rim's extremes and the cutter stopped 0.19 mm
/// short, leaving a bridge at each tip and the part in one piece). Volume, a point in the bridge
/// and the STL's shells, against an explicit cutter and the controls.
#[test]
fn a_through_cut_crosses_its_stock_whichever_way_the_stocks_circle_starts() {
    let turned = [[0.9807852804032304, 0.19509032201612825, 0.], [-0.19509032201612825, 0.9807852804032304, 0.]];
    let aligned = [[1., 0., 0.], [0., 1., 0.]];
    // the disc less the strip |y| ≤ 1, closed form
    let want = 200.*std::f64::consts::PI-4.*(99f64.sqrt()+100.*0.1f64.asin());
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let built = |src: &str| {
        let (prog,errs) = crate::common::parse(src);
        assert!(errs.is_empty(),"{errs:?}");
        let mut sk = gcs_core::program::elaborate(&prog).sketch;
        gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
        let body = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "body").unwrap();
        let exact = export::exact(&sk,body,None,None,&say).unwrap();
        exact.solid.check(1e-6).unwrap();
        let shells = gcs_core::mesh::stl_shells(&export::stl(&sk,body,&exact,None,&say).unwrap()).unwrap().len();
        (exact.solid,shells)
    };
    const THROUGH: &str = "std.solids.Through(face(a, b, c, d, -> close))";
    const BETWEEN: &str = "std.solids.Prism(face(a, b, c, d, -> close), from: -11mm, to: 11mm)";
    for (frame,cutter) in [(turned,THROUGH),(aligned,THROUGH),(turned,BETWEEN)] {
        let (solid,shells) = built(&strip_cut(frame,cutter,true));
        let volume = props::volume(&solid);
        assert!((volume-want).abs() < 1e-9*want,"{frame:?}, {cutter}: {volume} against {want}");
        assert_eq!(shells,2,"{frame:?}, {cutter}");
        let at = gcs_core::brep::query::Located::new(&solid,1e-9);
        assert_eq!(at.solid_place([9.95,0.,1.]),gcs_core::brep::query::Place::Out,"{frame:?}, {cutter}");
    }
    // uncut, the turned frame changes nothing
    let (solid,shells) = built(&strip_cut(turned,BETWEEN,false));
    let volume = props::volume(&solid);
    assert!((volume-200.*std::f64::consts::PI).abs() < 1e-9*volume,"{volume}");
    assert_eq!(shells,1);
}

/// The configured hypoid pinion, its end relief lowered into its blank's meridian as two conics
/// turned (#196, `Surface::Revolution` of a rational quadratic B-spline): its sector is split by
/// the sides and turned into every copy — a traced edge of the band ends where it leaves the
/// band's patch (`Surface::off_patch`), and the band is kept by the turns about the axis, not
/// taken for a side (`brep::pattern`).  Cut by the mill it was recorded at
/// (`fixtures::gear::recorded_machine`): the mill the pair finds (#283) has a larger cutter
/// (`the_pinion_as_written_is_built_by_sectors`).
#[test]
fn the_relieved_pinion_is_built_by_sectors() {
    let e = fixtures::gear::read(&fixtures::gear::recorded_machine(&fixtures::gear::source(),12.5));
    let (exact,said) = built(&e,"pair.pinion.body");
    // each sector: the blank's pieces, the space's five faces and the relief's five, named by
    // the cutter faces that made them (#212)
    the_pinion_by_sectors(&exact,&said);
    assert_eq!(exact.solid.faces.iter().filter(|f| f.surface.kind() == "revolution").count(),24*2,
        "the two conic bands, a piece in each sector");
}

/// **The pinion as its document writes it is built** (the page's `?example=spiral_bevel`): at the
/// mill the pair finds (#283) its cut's sheet is laid by walk length and named by the cutter's
/// faces, each sector the blank's pieces, the space's five faces and the relief's five, as at the
/// mill it was recorded at; built, turned into every copy, and checked. Until its stations were read
/// from where its contacts first reach the blank, and a margin column leaping beside the blank cost
/// the sheet that column rather than its rows (#904 A4), the sheet was fitted there as one face, a
/// cubic net of 240 × 66 poles laid by length in space, which the spline edges of the sector's side
/// hugged; from #814 the search over that net halved past any search's boxes until it was read by
/// its restricted Bézier patches (`enclosures::a_spline_edge_hugging_a_fine_net_is_found_where_it_crosses`
/// holds that case apart from the pinion). Solved as written, the mill its balance finds and all,
/// it is the pinion built at that mill stated
/// (`hypoid_mill_sheet::the_pinion_at_the_found_mill_is_built_by_sectors`, the fast gate).
#[test]
fn the_pinion_as_written_is_built_by_sectors() {
    let e = fixtures::gear::read_as_configured();
    let (exact,said) = built(&e,"pair.pinion.body");
    the_pinions_sheet(&said);
    the_pinion_by_sectors(&exact,&said);
    let volume = props::volume(&exact.solid);
    assert!((volume-14664.2384).abs() < 1e-3,"{volume} mm³");
}

/// **The pinion scaled is built as the pinion is** (#904 A3): every length of the pair 0.625 times
/// as long — a 1.25 mm module, the offset and the allowances — and cut by the mill its balance finds
/// there (the configured mill scaled, stated: `hypoid_machine::the_face_mill_scaled_is_found_scaled`),
/// the pinion is built by sectors with the faces it has at its own size. Its cut is traced in its
/// own scale (`trace::Tracer::unit`): the crown tooth's faces it walks are 0.625 times as long, and
/// its sheet is the pinion's, scaled. While the construction's lengths were millimetres the reach,
/// asking every 0.3 mm, missed the tooth's 0.24 mm tip and walked the wrong way round the tooth,
/// and a station's contact curve ended inside the blank.
///
/// The configured pinion's cut at that mill reaches the blank over 11.3 degrees of stations and
/// 13.702 mm of its crown tooth's faces (`CONFIGURED_WALK`); the construction's lengths were set at
/// the 13.7855 mm it walked at the mill the balance of profile radii found (`REFERENCE_WALK`).
#[test]
fn the_pinion_scaled_is_built_as_the_pinion_is() {
    const CONFIGURED_WALK: f64 = 13.702;
    let s = 0.625;
    let e = fixtures::gear::read(&fixtures::gear::scaled_as_found(s));
    let (exact,said) = built(&e,"pair.pinion.body");
    let walk = CONFIGURED_WALK*s;
    assert!(said.contains(&format!("`pair.pinion.cut.removal`: contacts reach the blank over 11.3 degrees of stations and 5 \
        profile faces, {walk:.3} mm of them")),"{said}");
    the_pinions_sheet(&said);
    the_pinion_by_sectors(&exact,&said);
}

/// **Each member of the pair is built alike at every scale** (#904 A3): the pinion at half its size
/// and at 0.75 and 1.5 times, and the gear at 0.625 and 1.5 times, each cut by its balance's mill
/// there (`fixtures::gear::scaled_as_found`), are built with the faces each has at its own size.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute: five members of scaled pairs built")]
fn each_member_of_the_pair_is_built_alike_at_every_scale() {
    for (member,s,faces) in [("pinion",0.5,243),("pinion",0.75,243),("pinion",1.5,243),("gear",0.625,291),("gear",1.5,291)] {
        let e = fixtures::gear::read(&fixtures::gear::scaled_as_found(s));
        let body = fixtures::solid(&e,&format!("pair.{member}.body"));
        let (exact,said) = saying(|say| export::exact(&e.sketch,body,None,None,say));
        let exact = exact.unwrap_or_else(|r| panic!("the {member} at {s}: {r:?}\n{said}"));
        assert_eq!(exact.solid.faces.len(),faces,"the {member} at {s}: {said}");
        exact.solid.check(1e-6).unwrap();
    }
}

/// The pair at 0.625 of its size, cut by the mill the balance of profile radii found there (11.2527°,
/// the cutter 47.2576 mm): at the gear's heel a corner of the cutter grazes the blank's end relief
/// at some stations and is a fan the sheet's rows stop before at others (`contact_trace::charted`),
/// so the cut's fitted sheet ends a few microns inside the blank, where it divides nothing. Carried
/// on past it by its own surface (`sweep::sheet`), the sector splits into its cells, is turned into
/// every copy, and agrees with its field. (The 20-tooth pair's sheet did, until it was laid in its
/// own scale, #904 A3, and the 21-tooth pair's, until its stations were read from where its
/// contacts first reach the blank, #904 A4: theirs now leave the blank there. So does this pair's
/// at the mill its balance finds since #904, `fixtures::gear::scaled_as_found`, so the mill this
/// case was found at is stated.)
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 10 s: the scaled pair's gear built and measured")]
fn a_sheet_ending_in_the_blank_is_carried_out_of_it() {
    let s = 0.625;
    let scaled = fixtures::gear::scaled(&fixtures::gear::source(),s);
    let e = fixtures::gear::read(&fixtures::gear::machine(&scaled,11.2527));
    let body = fixtures::solid(&e,"pair.gear.body");
    let (exact,said) = built(&e,"pair.gear.body");
    assert!(said.contains("`pair.gear.cut.removal`: the fitted sheet's edge is in the blank by up to"),"{said}");
    assert!(said.contains("carried on past it by its own surface"),"{said}");
    assert!(said.contains("classified 1 material and 4 removed cells"),"{said}");
    assert!(said.contains("turned the sector into 48 copies"),"{said}");
    exact.solid.check(1e-6).unwrap();
    let (stl,more) = saying(|say| export::stl(&e.sketch,body,&exact,None,say).unwrap());
    gcs_core::mesh::stl_shells(&stl).unwrap();
    let said = format!("{said}\n{more}");
    assert!(said.contains(" 0 disagree"),"{said}");
}

/// **An empty body is refused as one** (#617): two blocks apart share nothing, and the export
/// of what they share names the solid and the statement that emptied it — not a float32 rounding
/// read off an empty solid's infinite bounds, nor a STEP file of no faces.  A body standing on it
/// is refused for its stock.
#[test]
fn an_empty_body_is_refused_naming_the_statement_that_emptied_it() {
    let src = "unit mm\nuse std\nuse std.solids (cut, common)\nuse std.place (coords)\nuse std.shapes\nin std.top {\n  ca := point hint((0, 0))\n  \
               fix((0mm, 0mm)) ca\n  sa := std.shapes.CenteredRectangle(ca, w: 10mm, h: 10mm)\n  \
               cb := point hint((20, 0))\n  ca distance(20mm, along: u) cb\n  ca level(v) cb\n  \
               sb := std.shapes.CenteredRectangle(cb, w: 10mm, h: 10mm)\n}\n\
               a := solid(std.solids.Prism(face(sa.loop), from: 0mm, to: 10mm))\n\
               b := solid(std.solids.Prism(face(sb.loop), from: 0mm, to: 10mm))\n\
               shared := solid(a)\nb common shared\nover := solid(shared)\n";
    let mut sk = crate::common::read(src).sketch;
    assert!(gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default()).success);
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let at = |name: &str| (0..sk.solids.len()).find(|&i| sk.solids[i].name == name).unwrap();
    for (name,why) in [("shared","`b common shared` leaves no material"),("over","its stock `shared` holds no material")] {
        let e = export::exact(&sk,at(name),None,None,&say).err().unwrap_or_else(|| panic!("{name} built"));
        assert_eq!(e.message,format!("`{name}` is empty: {why}, so there is nothing to write"));
    }
    // and what is not empty is written as before
    let b = export::exact(&sk,at("b"),None,None,&say).unwrap();
    export::stl(&sk,at("b"),&b,None,&say).unwrap();
}

/// A 40 × 30 × 20 mm block standing `z0` mm along the front plane's normal, cut from below by a
/// 20 × 10 pocket to `z0` and a 10 × 20 bore crossing it to `step` beyond: the bore's walls stand
/// `step` tall over the pocket's ceiling where they cross it.
fn stepped_pockets(z0: f64,step: f64) -> Sketch {
    let src = format!("\
unit mm
use std
use std.solids (cut)
use std.shapes
in std.front {{
  outer := std.shapes.CenteredRectangle(std.origin, w: 40mm, h: 30mm)
  wide := std.shapes.CenteredRectangle(std.origin, w: 20mm, h: 10mm)
  tall := std.shapes.CenteredRectangle(std.origin, w: 10mm, h: 20mm)
}}
block := solid(std.solids.Prism(face(outer.loop), from: {}mm, to: {}mm))
pocket := solid(std.solids.Prism(face(wide.loop), from: {}mm, to: {z0}mm))
bore := solid(std.solids.Prism(face(tall.loop), from: {}mm, to: {z0}mm + {step}mm))
pocket cut block
bore cut block
base := solid(block)
",z0-10.,z0+10.,z0-20.,z0-20.);
    let mut sk = crate::common::read(&src).sketch;
    assert!(gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default()).success);
    sk
}

/// **A triangle float32 flattens is said where it is** (#821): two pockets ending 200 nm apart ten
/// metres out, where float32 steps 977 nm, leave the bore's walls a strip 200 nm tall that binary
/// STL cannot write — farther apart than the Booleans read as flush (`brep::flush`, a part in a
/// million of the block's size, 54 nm), so the B-rep holds it. The refusal names the solid, the
/// faces its flat triangles are of and the face beside the first, a point near it (and the witness
/// there), how thin it is and that float32 flattened it, and the likely cause — where it said only
/// that the file had a degenerate triangle. The same step near the origin, or 10 µm out there, is
/// written.
#[test]
fn a_sliver_float32_flattens_is_refused_naming_its_faces_and_where() {
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let stl = |z0: f64,step: f64| { let (sk,body,exact) = stepped_base(z0,step); export::stl(&sk,body,&exact,None,&say) };
    let e = stl(10000.,2e-4).err().expect("a strip float32 flattens is refused");
    assert_eq!(e.stage,gcs_core::solid::export::Stage::Stl,"{e}");
    for said in ["`base`: ","STL triangles have no area as written, on `bore.","no area once float32 rounds its corners",
        "nm thick) is near (","beside `pocket.near`","most often a sliver","here `bore."] {
        assert!(e.message.contains(said),"{said}: {e}");
    }
    // on the bore's walls, across the pocket's ceiling, ten metres out
    let w = e.witness.expect("a point near the flat triangle");
    let out = w.iter().map(|x| x.abs()).fold(0.,f64::max);
    assert!((out-10000.).abs() < 1e-3,"{w:?}");
    assert!(w.iter().any(|x| (x.abs()-5.).abs() < 1e-6),"{w:?}: on a wall of the bore, 5 mm from its middle");
    // float32 writes the strip near the origin, and a step it can tell apart out there
    gcs_core::mesh::stl_shells(&stl(0.,2e-4).unwrap()).unwrap();
    gcs_core::mesh::stl_shells(&stl(1000.,0.01).unwrap()).unwrap();
}

/// The base of `stepped_pockets(z0, step)` built exactly, and its index.
fn stepped_base(z0: f64,step: f64) -> (Sketch,usize,export::Exact) {
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let sk = stepped_pockets(z0,step);
    let body = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "base").unwrap();
    let exact = export::exact(&sk,body,None,None,&say).unwrap();
    (sk,body,exact)
}

/// **An end all but flush with a face is on it** (#836, the issue's own case): the bore ending
/// 10 nm over the pocket's ceiling a thousand millimetres out — or 10 nm under it — is cut as if it
/// ended on the ceiling (`brep::flush`): the same faces and volume as the bore ending there exactly,
/// no corner standing off the ceiling, no sliver to say, and its STEP and STL written. Where the
/// float32 STL refused it, and a STEP of 19 faces held the strip silently, the file is what was meant.
#[test]
fn an_end_all_but_flush_with_a_face_is_cut_on_it() {
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let (_,_,flush) = stepped_base(1000.,0.);
    let b = &flush.solid;
    for step in [1e-5,-1e-5] {
        let (sk,body,exact) = stepped_base(1000.,step);
        let x = &exact.solid;
        x.check(1e-6).unwrap();
        assert_eq!((x.faces.len(),x.edges.len(),x.vertices.len()),(b.faces.len(),b.edges.len(),b.vertices.len()),"{step:e}");
        assert!((props::volume(x)-props::volume(b)).abs() <= 1e-9*props::volume(b),"{step:e}");
        // every corner on a plane the drawing states: none 10 nm off the ceiling
        for v in &x.vertices {
            let off = (v.p.iter().map(|k| k.abs()).fold(0.,f64::max)-1000.).abs();
            assert!(off <= 1e-9 || off >= 9.9,"{step:e}: {:?}",v.p);
        }
        assert!(export::slivers(&sk,body,&exact,None).is_none(),"{step:e}");
        export::step(&exact,"base",None,&say).unwrap();
        gcs_core::mesh::stl_shells(&export::stl(&sk,body,&exact,None,&say).unwrap()).unwrap();
    }
}

/// **A sliver the B-rep holds is said wherever it stands** (#836): the bore ending 1 µm over the
/// pocket's ceiling — farther than the Booleans read as flush, so built as drawn, the strip of wall
/// held exactly — is said by the core in the same words at the origin and a thousand millimetres
/// out, where float32 tells the ends apart and the STL is written: the solid, how many faces are
/// thinner than the export's tolerance, the first by name and how thin, the faces it stands between
/// and a point on it (the witness there). An export asked to 0.5 µm finds nothing so thin, and a
/// step of 20 µm is none.
#[test]
fn a_sliver_the_brep_holds_is_said_wherever_it_stands() {
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let (_,_,flush) = stepped_base(0.,0.);
    for z0 in [0.,1000.] {
        let (sk,body,exact) = stepped_base(z0,1e-3);
        // the strip held: the bore's 200 mm² a micrometre deeper than flush
        let deeper = props::volume(&flush.solid)-props::volume(&exact.solid);
        assert!((deeper-0.2).abs() <= 1e-6,"{z0}: {deeper}");
        let s = export::slivers(&sk,body,&exact,None).expect("a strip 1 µm tall is a sliver");
        for said in ["`base` holds 2 faces thinner than the export's 10.000 µm: the first, `bore.","1.000 µm across, beside \
            `pocket.near` and `bore.near`, near (","move that end onto the face or clear of it"] {
            assert!(s.message.contains(said),"{said}: {}",s.message);
        }
        // on a wall of the bore, midway up the strip
        let w = s.witness;
        assert!(w.iter().any(|x| (x.abs()-5.).abs() < 1e-9),"{w:?}");
        assert!(w.iter().any(|x| (x.abs()-z0-5e-4).abs() < 1e-6),"{w:?}");
        // the files are written: the STEP of the exact solid, and the STL float32 can write
        export::step(&exact,"base",None,&say).unwrap();
        export::stl(&sk,body,&exact,None,&say).unwrap();
        assert!(export::slivers(&sk,body,&exact,Some(gcs_core::solid::export::Tolerance::new(5e-4).unwrap())).is_none());
    }
    let (sk,body,exact) = stepped_base(1000.,0.02);
    assert!(export::slivers(&sk,body,&exact,None).is_none());
}

/// A binary STL refused by its reader says which triangle and where (#821), not only that one is
/// degenerate.
#[test]
fn an_stl_reader_refusing_a_flat_triangle_says_which() {
    let t = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[3.,1.,2.],[4.,1.,2.],[5.,1.,2.]].concat();
    let e = gcs_core::mesh::stl_shells(&gcs_core::mesh::stl_of(&t,"t")).unwrap_err();
    assert_eq!(e,"binary STL triangle 2 of 2, at (4.0000, 1.0000, 2.0000), has no area");
}

/// **A cube trimmed by its midsphere** (`midsphere_common.sv`, brep-bench's
/// tangent/midsphere-common): the ball's radius is the solve's, through a corner of the cube's
/// middle section, and the exact common is the closed forms' — six discs and eight patches of the
/// sphere meeting two and two at the twelve edges' middles — written and meshed closed.
#[test]
fn a_cube_trimmed_by_its_midsphere_is_built_exactly() {
    use crate::common::{close,dof,solid_named,solved};
    use std::f64::consts::{PI,SQRT_2};
    // read alone: `example` elaborates every example to find one
    let e = fixtures::example("midsphere_common.sv").unwrap();
    let sk = solved(&e);
    assert_eq!(dof(&mut sk.clone()),0);
    let body = solid_named(&e,"body");
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let exact = export::exact(&sk,body,None,None,&say).unwrap();
    exact.solid.check(1e-6).unwrap();
    let b = &exact.solid;
    assert_eq!((b.faces.len(),b.edges.len(),b.vertices.len()),(14,24,12));
    close(props::volume(b),PI*(1250.-2000.*SQRT_2/3.));
    close(b.faces.iter().map(|f| props::area(b,f)).sum(),PI*(300.*SQRT_2-250.));
    let stl = export::stl(&sk,body,&exact,None,&say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
}

/// **A groin vault** (`groin_vault.sv`, brep-bench's nurbs/groin-vault): two blocks under weighted
/// spline vaults crossed at right angles, built exactly — four pieces of vault meeting along the
/// diagonals in arches that end at the crown, where the vaults touch, on the floor and four sides —
/// to the closed forms' volume and area, written and meshed closed.
#[test]
fn a_groin_vault_is_built_exactly() {
    use crate::common::{close,dof,solid_named,solved};
    let e = fixtures::example("groin_vault.sv").unwrap();
    let sk = solved(&e);
    assert_eq!(dof(&mut sk.clone()),0);
    let vault = solid_named(&e,"vault");
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let exact = export::exact(&sk,vault,None,None,&say).unwrap();
    exact.solid.check(1e-6).unwrap();
    let b = &exact.solid;
    assert_eq!((b.faces.len(),b.edges.len(),b.vertices.len()),(9,16,9));
    close(props::volume(b),7632.);
    close(b.faces.iter().map(|f| props::area(b,f)).sum(),2256.);
    let stl = export::stl(&sk,vault,&exact,None,&say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
}

/// A quarter-turn-style elbow's document: a section of radius `r` about a point `bend` from the
/// bend axis (the line x = 0, z = `bend`, along y), clipped by a chord on the axis where it reaches
/// it (`r > bend`, a spindle torus; one arc of 308° at r 20), or whole; turned `turn` degrees; a
/// leg of radius 20 standing up from where the turn starts and one running on from where it ends,
/// tangent to it there as a pipe's are. `two` splits the arc at its far point; `ends` places the
/// arc's ends by hand, with no radius said (the turn's tube then as wide as they put it); `order`
/// the body's first solid and the two joined to it. `legs` is the two legs alone.
fn elbow(r: f64,bend: f64,turn: f64,two: bool,ends: Option<f64>,order: [&str;3]) -> String {
    let h = (r*r-bend*bend).max(0.).sqrt();
    let section = if r <= bend {
        format!("  a0 := circle(center: c) hint(r: {r})\n  radius({r}mm) a0\n  prof := face(a0)\n")
    } else {
        let y = ends.unwrap_or(h);
        let mut s = format!("  s := point hint((0, {y}))\n  fix(x == 0mm) s\n  e := point hint((0, {}))\n  fix(x == 0mm) e\n",-y);
        if ends.is_some() { s += &format!("  fix(y == {y}mm) s\n  fix(y == {}mm) e\n",-y); }
        if two {
            s += &format!("  f := point\n  fix(({}, 0)) f\n  a0 := arc(center: c, start: s, end: f)\n  a1 := arc(center: c, start: f, end: e)\n",-bend-r);
            s += "  ch := line(e, s)\n  prof := face(a0, a1, ch)\n";
        } else {
            s += "  a0 := arc(center: c, start: s, end: e)\n";
            if ends.is_none() { s += &format!("  radius({r}mm) a0\n"); }
            s += "  ch := line(e, s)\n  prof := face(a0, ch)\n";
        }
        s
    };
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\n\
        bend := plane\nfix(origin == (0mm, 0mm, {bend}mm)) bend\nfix(dir == (1, 0, 0)) bend.u\nfix(dir == (0, 1, 0)) bend.v\n\
        in bend {{\n  c := point\n  fix(({nb}, 0)) c\n{section}  ax0 := point\n  ax1 := point\n  fix((0, 10)) ax0\n  fix((0, -10)) ax1\n  axis_l := line(ax0, ax1)\n}}\n\
        elbow := solid(std.solids.Revolution(prof, about: axis_l, from: 0deg, to: {turn}deg))\n\
        in std.top {{\n  vc := point\n  fix(({nb}, 0)) vc\n  v := circle(center: vc) hint(r: 20)\n  radius(20mm) v\n}}\n\
        leg_v := solid(std.solids.Prism(face(v), from: {bend}mm, to: {top}mm))\n\
        out := plane\nfix(origin == (0mm, 0mm, {bend}mm)) out\nfix(dir == (-cos({turn}deg), 0, -sin({turn}deg))) out.u\nfix(dir == (0, 1, 0)) out.v\n\
        in out {{\n  hc := point\n  fix(({bend}, 0)) hc\n  h := circle(center: hc) hint(r: 20)\n  radius(20mm) h\n}}\n\
        leg_h := solid(std.solids.Prism(face(h), from: 0mm, to: 40mm))\n\
        body := solid({})\n{} union body\n{} union body\nlegs := solid(leg_v)\nleg_h union legs\n",
        order[0],order[1],order[2],nb = -bend,top = bend+42.)
}

/// **A swept elbow joins its straight legs at their tangent seams** (#996): a section turned about
/// an axis and two cylinders of its radius whose ends meet the turn's ends tangentially, as a
/// pipe's do — a ring torus's section whole (r 15 at 18), a horn's touching the axis (r 18), and a
/// spindle's clipped by a chord on the axis (r 18.3, 20 and 25 at 18: one arc of up to 308°, its
/// faces reaching the axis at the turn's poles), turned 45°, 90° and 135°, the legs joined either
/// way round or the elbow first. The body is the turn's volume by Pappus — the clipped disc's
/// moment about the axis times the angle, in closed form — and the two legs' union, the turn
/// standing beside both: the same written as one arc or as two split at its far point, valid
/// each time. A turn's caps stand in planes through its axis, which meet its torus in its
/// meridians in closed form (the two-arc build took 3.3 s, tracing them), and a cell of the
/// search beside a spindle's lemon is clear of the face (`ssi::disc_reach`). A tube a thousandth
/// wider than its legs (the arc's ends placed, no radius said) leaves a crescent of its cap 308°
/// long and a thousandth wide, whose loop's area settles its sense (`Brep::loop_area`).
#[test]
fn a_swept_elbow_joins_its_legs_at_their_tangent_seams() {
    let vh = ["leg_v","elbow","leg_h"];
    let cases: Vec<(f64,f64,bool,Option<f64>,[&str;3])> = vec![
        (20.,90.,false,None,vh),(20.,90.,true,None,vh),
        (20.,90.,false,None,["leg_h","elbow","leg_v"]),(20.,90.,false,None,["elbow","leg_v","leg_h"]),
        (20.,45.,false,None,vh),(20.,135.,false,None,vh),(20.,135.,true,None,vh),
        (15.,90.,false,None,vh),(18.,90.,false,None,vh),(18.3,90.,false,None,vh),(25.,90.,false,None,vh),
        (20.,90.,false,Some(8.72),vh),
    ];
    let mut one_and_two: Vec<((f64,f64),f64)> = Vec::new();
    for (r,turn,two,ends,order) in cases {
        let what = format!("r {r}, {turn}°, {}, ends {ends:?}, {order:?}",if two { "two arcs" } else { "one arc" });
        let (e,sk) = crate::common::read_solved(&elbow(r,18.,turn,two,ends,order));
        let (body,legs) = (exact_named(&e,&sk,"body",&what),exact_named(&e,&sk,"legs",&what));
        body.check(1e-6).unwrap_or_else(|why| panic!("{what}: {why:?}"));
        let tube = ends.map_or(r,|y| (18f64*18.+y*y).sqrt());
        let want = turn.to_radians()*moment(tube,18.)+props::volume(&legs);
        let got = props::volume(&body);
        assert!((got-want).abs() <= 1e-8*want,"{what}: {got} against {want} ({:e})",(got-want)/want);
        if ends.is_none() && order == vh { one_and_two.push(((r,turn),got)); }
    }
    for w in one_and_two.windows(2).filter(|w| w[0].0 == w[1].0) {
        assert!((w[0].1-w[1].1).abs() <= 1e-9*w[0].1,"{:?}: one arc {} against two {}",w[0].0,w[0].1,w[1].1);
    }
}

/// The elbow of `elbow` with its tube `20 + δ` (the arc's ends placed where that tube crosses the
/// axis) — or, `hinted`, the issue's own document, its ends' hints alone saying where — its legs of
/// radius `legs`, and `extra` statements after.
fn near_miss(delta: f64,hinted: bool,legs: f64,extra: &str) -> String {
    let tube = 20.+delta;
    let ends = if hinted { 8.7178 } else { (tube*tube-18.*18.).sqrt() };
    let src = elbow(20.,18.,90.,false,Some(ends),["leg_v","elbow","leg_h"])
        .replace("use std.solids (union)","use std.solids (union, cut)")
        .replace("radius(20mm) v",&format!("radius({legs}mm) v")).replace("radius(20mm) h",&format!("radius({legs}mm) h"));
    let src = if hinted {
        src.lines().filter(|l| !l.trim_start().starts_with("fix(y ==")).collect::<Vec<_>>().join("\n")+"\n"
    } else { src };
    src+extra
}

/// The solid `name` of `src` built exactly, checked valid, and its volume and the tolerance's band
/// over its boundary (`geom::tolerance`, a part in 10⁹ of its size, times its area).
fn built_near_miss(src: &str,name: &str,what: &str) -> (f64,f64) {
    let (e,sk) = crate::common::read_solved(src);
    let solid = exact_named(&e,&sk,name,what);
    solid.check(1e-6).unwrap_or_else(|why| panic!("{what}: {name}: {why:?}"));
    let area: f64 = solid.faces.iter().map(|f| props::area(&solid,f)).sum();
    (props::volume(&solid),gcs_core::brep::geom::tolerance(solid.size())*area)
}

/// The near-misses a sweep reads: by decades from 1e-8 to a hundredth, and two, three, five and
/// seven of each of the decades where the tolerance's readings once parted (1e-8 to 1e-6), either
/// way.
fn near_misses() -> Vec<f64> {
    let mut out: Vec<f64> = (-8..=-2).map(|k| 10f64.powi(k)).collect();
    for k in -8..=-6 { for m in [2.,3.,5.,7.] { out.push(m*10f64.powi(k)); } }
    out.iter().flat_map(|&d| [d,-d]).collect()
}

/// The turn's volume by Pappus: the moment about the axis of the disc of radius `r` about 18 from
/// it, clipped where it crosses the axis, times a quarter turn.
fn turn_volume(r: f64) -> f64 { std::f64::consts::PI/2.*moment(r,18.) }

/// The moment about the axis of the disc of radius `r` about `bend` from it, on its side and
/// clipped where it crosses the axis: `∫ ρ · 2√(r² − (ρ − bend)²) dρ` over `ρ ≥ 0` — a turn's
/// volume per radian by Pappus.
fn moment(r: f64,bend: f64) -> f64 {
    if r <= bend { return std::f64::consts::PI*r*r*bend }
    let f = |t: f64| bend*(t*(r*r-t*t).sqrt()+r*r*(t/r).asin())-2./3.*(r*r-t*t).powf(1.5);
    f(r)-f(-bend)
}

/// The solid the document calls `name` built exactly at the export's own tolerance, or the test
/// fails saying `what` and what the build said.
fn exact_named(e: &gcs_core::program::Elaborated,sk: &Sketch,name: &str,what: &str) -> gcs_core::brep::topo::Brep {
    let (out,said) = saying(|say| export::exact(sk,fixtures::solid(e,name),None,None,say));
    out.unwrap_or_else(|why| panic!("{what}: {name}: {why:?}\n{said}")).solid
}

/// **A turn a near-miss wider or narrower than its legs is the tangent turn, or built with its
/// step** (#996): the elbow's tube drawn `20 ± δ` against legs of 20, over `near_misses`, and the
/// issue's own document (its arc's radius stated by no dimension, its ends' hints putting the tube
/// 9.2e-7 over the legs). Within a part in a million of the solid's size (`flush::FLUSH` of the
/// Boolean's tolerance) a leg is moved onto the turn's tube where it stands on it
/// (`flush::tubes_flushed`), and the solid is the tangent turn and its legs at the tube's radius —
/// the same solid drawn so, to the tolerance's band; past it the leg stays as drawn and the step is
/// built: the turn's volume by Pappus and the legs' own union. Each valid. Between a tolerance and a
/// thousandth the arrangement's readings of the seam had parted, and one near-miss in three was
/// refused, or (at 1e-7) built 2.6 times too large. At 1e-4, at the reach, either may be read.
#[test]
fn a_turn_a_near_miss_off_its_legs_is_the_tangent_turn_or_built_with_its_step() {
    let mut cases: Vec<(f64,bool)> = near_misses().into_iter().map(|d| (d,false)).collect();
    cases.push(((18f64*18.+8.7178*8.7178).sqrt()-20.,true));
    for (delta,hinted) in cases {
        let what = format!("δ {delta:e}{}",if hinted { ", the issue's document" } else { "" });
        let tube = 20.+delta;
        let (got,band) = built_near_miss(&near_miss(delta,hinted,20.,""),"body",&what);
        // the tangent turn at the tube's radius, its legs drawn so, and its own closed form
        let tangent = near_miss(delta,hinted,tube,"");
        let (moved,_) = built_near_miss(&tangent,"body",&what);
        let (legs_moved,_) = built_near_miss(&tangent,"legs",&what);
        assert!((moved-turn_volume(tube)-legs_moved).abs() <= band,"{what}: the tangent turn {moved}");
        let (legs,_) = built_near_miss(&near_miss(delta,hinted,20.,""),"legs",&what);
        let stepped = turn_volume(tube)+legs;
        let near = |w: f64| (got-w).abs() <= band;
        if delta.abs() < 1e-4 { assert!(near(moved),"{what}: {got} against the tangent turn's {moved}") }
        else if delta.abs() > 1e-4 { assert!(near(stepped),"{what}: {got} against the step's {stepped}") }
        else { assert!(got >= moved.min(stepped)-band && got <= moved.max(stepped)+band,"{what}: {got}") }
    }
}

/// **…and so it is with a solid joined or cut through the turn's poles, or across its bend**
/// (#996): an axle along the bend axis joined through both poles, a slab cut along a plane through
/// the axis at 45°, and a bore across the bend, over `near_misses`: a valid solid whose volume is
/// the same statements' over the tangent turn at the tube's radius within a part in a million (the
/// legs moved onto it), and past that off the tangent turn's by no more than the step the tube adds.
/// The fixture's own refusal ("loop 0 breaks in parameters" at a pole, its collapsed parameter
/// half a turn round) was the slab's at 1e-8 and the turn's alone at −1e-7; "a shell of the
/// arrangement is kept on one side … and dropped on another" any Boolean's on the issue's
/// document after the two legs.
#[test]
fn a_solid_through_a_near_miss_turns_poles_is_built() {
    let axle = "in std.front {\n  ac := point\n  fix((0, 18)) ac\n  ak := circle(center: ac) hint(r: 3)\n  \
        radius(3mm) ak\n}\naxle := solid(std.solids.Prism(face(ak), from: -30mm, to: 30mm))\naxle union body\n";
    let slab = "wp := plane\nfix(origin == (0mm, 0mm, 18mm)) wp\nfix(dir == (-cos(45deg), 0, -sin(45deg))) wp.u\n\
        fix(dir == (0, 1, 0)) wp.v\nin wp {\n  w0 := point\n  w1 := point\n  w2 := point\n  w3 := point\n  \
        fix((-5, -30)) w0\n  fix((60, -30)) w1\n  fix((60, 30)) w2\n  fix((-5, 30)) w3\n  wl0 := line(w0, w1)\n  \
        wl1 := line(w1, w2)\n  wl2 := line(w2, w3)\n  wl3 := line(w3, w0)\n}\n\
        slab := solid(std.solids.Prism(face(wl0, wl1, wl2, wl3), from: 0mm, to: 70mm))\nslab cut body\n";
    let bore = "in std.front {\n  bc := point\n  fix((-12, 6)) bc\n  b := circle(center: bc) hint(r: 5)\n  \
        radius(5mm) b\n}\nbore := solid(std.solids.Prism(face(b), from: -60mm, to: 60mm))\nbore cut body\n";
    for extra in [axle,slab,bore] {
        let (tangent,_) = built_near_miss(&near_miss(0.,false,20.,extra),"body","δ 0");
        let mut cases: Vec<(f64,bool)> = near_misses().into_iter().map(|d| (d,false)).collect();
        cases.push(((18f64*18.+8.7178*8.7178).sqrt()-20.,true));
        for (delta,hinted) in cases {
            let what = format!("δ {delta:e}{}: {}",if hinted { ", the issue's document" } else { "" },extra.lines().last().unwrap());
            let (v,band) = built_near_miss(&near_miss(delta,hinted,20.,extra),"body",&what);
            if delta.abs() < 1e-4 {
                let (moved,_) = built_near_miss(&near_miss(delta,hinted,20.+delta,extra),"body",&what);
                assert!((v-moved).abs() <= band,"{what}: {v} against the tangent turn's {moved}");
            } else {
                let step = (turn_volume(20.+delta)-turn_volume(20.)).abs();
                assert!((v-tangent).abs() <= step+band,"{what}: {v} against the tangent turn's {tangent}");
            }
        }
    }
}

/// **The fixture's shape: the issue's elbow cored by a bent passage, a flange at each end** (#996,
/// CADGenBench fixture 104, "a bent Ø34 passage over a web"): the issue's document as written, a
/// square flange 60 × 60 × 6 joined at each leg's end, and cut by a passage of radius 17 that
/// follows the bend — a quarter turn of a circle about the same axis and two straight runs out
/// through the flanges. Its volume is the tangent turn's and its legs' at the tube's radius (the
/// legs moved onto it, `flush::tubes_flushed`), the flanges', less the passage's by Pappus and its
/// runs' through the solid: closed forms all but the legs' union, which the kernel builds alone.
#[test]
fn the_fixtures_elbow_cored_and_flanged_is_built() {
    use std::f64::consts::PI;
    let flanges_and_passage = "\
in std.top {\n  f0 := point\n  f1 := point\n  f2 := point\n  f3 := point\n  fix((-48, -30)) f0\n  fix((12, -30)) f1\n  \
fix((12, 30)) f2\n  fix((-48, 30)) f3\n  fa := line(f0, f1)\n  fb := line(f1, f2)\n  fc := line(f2, f3)\n  fd := line(f3, f0)\n}\n\
top_flange := solid(std.solids.Prism(face(fa, fb, fc, fd), from: 60mm, to: 66mm))\n\
out_flange := plane\nfix(origin == (40mm, 0mm, 0mm)) out_flange\nfix(dir == (0, 1, 0)) out_flange.u\nfix(dir == (0, 0, 1)) out_flange.v\n\
in out_flange {\n  g0 := point\n  g1 := point\n  g2 := point\n  g3 := point\n  fix((-30, -30)) g0\n  fix((30, -30)) g1\n  \
fix((30, 30)) g2\n  fix((-30, 30)) g3\n  ga := line(g0, g1)\n  gb := line(g1, g2)\n  gc := line(g2, g3)\n  gd := line(g3, g0)\n}\n\
side_flange := solid(std.solids.Prism(face(ga, gb, gc, gd), from: 0mm, to: 6mm))\n\
in bend {\n  pc := point\n  fix((-18, 0)) pc\n  pk := circle(center: pc) hint(r: 17)\n  radius(17mm) pk\n}\n\
core_turn := solid(std.solids.Revolution(face(pk), about: axis_l, from: 0deg, to: 90deg))\n\
in std.top {\n  pvc := point\n  fix((-18, 0)) pvc\n  pv := circle(center: pvc) hint(r: 17)\n  radius(17mm) pv\n}\n\
core_v := solid(std.solids.Prism(face(pv), from: 18mm, to: 80mm))\n\
in out {\n  phc := point\n  fix((18, 0)) phc\n  ph := circle(center: phc) hint(r: 17)\n  radius(17mm) ph\n}\n\
core_h := solid(std.solids.Prism(face(ph), from: 0mm, to: 60mm))\n\
top_flange union body\nside_flange union body\ncore_turn cut body\ncore_v cut body\ncore_h cut body\n";
    let delta = (18f64*18.+8.7178*8.7178).sqrt()-20.;
    let tube = 20.+delta;
    let (v,band) = built_near_miss(&near_miss(delta,true,20.,flanges_and_passage),"body","the fixture");
    let (legs,_) = built_near_miss(&near_miss(delta,true,tube,""),"legs","the legs at the tube's radius");
    let r2 = 17f64*17.;
    let passage = PI*PI/2.*r2*18.+PI*r2*(48.+46.);
    let want = turn_volume(tube)+legs+2.*60.*60.*6.-passage;
    assert!((v-want).abs() <= band,"the fixture: {v} against {want} ({:e}, the band {band:e})",v-want);
}
