//! The spiral-bevel pair's face mill found rather than stated (`spiral_bevel/hypoid.sv`, #283,
//! #904): its pressure shift is a formal of `HypoidPair` the call leaves to the solve and its
//! cutter's radius is stated, and `Balanced` reads the four flanks the two blades cut by their
//! least area factors within their blanks; the document ties the two nearest undercut by holding
//! both to one unknown margin, `pair.least`.  The rolls are the crown's tooth count over each
//! member's, written as counts.
//!
//! The numbers recorded here were taken of the document as it stands; the rest is the
//! balance's own words, read back off the solve.
use gcs_core::program::Elaborated;

type V = [f64;3];

fn sub(a: V,b: V) -> V { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn dist(a: V,b: V) -> f64 { let d = sub(a,b); (d[0]*d[0]+d[1]*d[1]+d[2]*d[2]).sqrt() }

/// A number the document leaves unknown, where the solve put it, in the unit the document
/// writes it in (an angle in degrees).
fn unknown(e: &Elaborated,name: &str) -> f64 {
    let q = *e.sketch.free_vars.get(name)
        .unwrap_or_else(|| panic!("no unknown `{name}` in {:?}",e.sketch.free_vars.keys()));
    e.sketch.params[q as usize].value
}

/// The face mill as found at the configured hypoid: its pressure shift in degrees and its
/// cutter's radius in millimetres, as the document states it.
const SHIFT: f64 = fixtures::gear::FOUND_MILL[0];
const CUTTER: f64 = fixtures::gear::FOUND_MILL[1];
/// The least margin the configured pair's balance finds, the area factor the two flanks it ties
/// share there.
const LEAST: f64 = 0.676717;

/// **The pair scaled finds the same shift** (#904 A3): with every length of the pair (the module,
/// the offset, the allowances and the cutter's radius) 0.625, 0.75 or 1.5 times as long, the
/// balance finds the same pressure shift and the same least margin — an area factor is a ratio of
/// rates, so a pair scaled whole stands as far from undercut.
#[test]
fn the_face_mill_scaled_is_found_scaled() {
    for s in [0.625,0.75,1.5] {
        let text = fixtures::gear::scaled(&fixtures::gear::source(),s);
        let cutter: f64 = text.lines().find_map(|l| l.strip_prefix("param cutter_radius := "))
            .and_then(|v| v.trim().strip_suffix("mm")).expect("the pair states its cutter").parse().unwrap();
        assert!((cutter-CUTTER*s).abs() < 1e-9*s,"at {s}: the cutter radius {cutter} mm against {} mm",CUTTER*s);
        let e = fixtures::gear::read(&text);
        let (shift,least) = (unknown(&e,"pair.pressure_shift"),unknown(&e,"pair.least"));
        assert!((shift-SHIFT).abs() < 1e-4,"at {s}: the pressure shift {shift}°");
        assert!((least-LEAST).abs() < 1e-5,"at {s}: the least margin {least}");
    }
}

#[test]
fn the_rolls_are_the_crowns_tooth_count_over_each_members() {
    // the bevel pair, its mill stated: the rolls read nothing the balance finds
    let e = fixtures::gear::read(&fixtures::gear::bevel(&fixtures::gear::source()));
    let crown = (24f64*24.+48.*48.).sqrt();
    for (member,ratio) in [("pinion_roll",crown/24.),("gear_roll",-crown/48.)] {
        let at = e.sketch.motions.iter().position(|m| m.name == format!("pair.{member}"))
            .unwrap_or_else(|| panic!("no motion `{member}`"));
        let read = e.sketch.motions[at].rotation(&e.sketch).unwrap().0;
        assert!((read-ratio).abs() < 1e-12*ratio.abs(),"{member}: {read} against {ratio}");
    }
}

#[test]
fn the_recorded_machine_states_the_mill_the_suites_were_cut_by() {
    // as configured, the pair leaves its pressure shift and the least margin to the solve and
    // states its cutter; a design the suites recorded states both settings (the shift it was
    // recorded at, the cutter 0.8 R) and drops the balance, with the lines that tie its margins
    let text = fixtures::gear::source();
    let found = fixtures::gear::unsolved(&text);
    for name in ["pair.pressure_shift","pair.least"] {
        assert!(found.sketch.free_vars.contains_key(name),"`{name}` is unknown as configured");
    }
    assert!(!found.sketch.free_vars.contains_key("pair.cutter_radius"),"the cutter is stated");
    assert!(text.contains(&format!("param cutter_radius := {CUTTER}mm")),"the cutter as found is stated");
    let stated = fixtures::gear::bevel(&text);
    assert!(!stated.contains("balance := Balanced(") && !stated.lines().any(|l| l.starts_with("pair.margins.")));
    let e = fixtures::gear::unsolved(&stated);
    for name in ["pair.pressure_shift","pair.cutter_radius"] {
        assert!(!e.sketch.free_vars.contains_key(name),"`{name}` is stated at a recorded design");
    }
    assert!(e.map.ent_named("pair.balance.pinion_inner").is_none());
}

/// The pair at 0.625 of its size, its mill stated, standing in its own frame or, `turned`, in the
/// frame `shaft_drive/drive.sv` holds it in: the bicycle's bottom bracket, its pitch plane at an
/// attitude no world axis lies in. The mill is the one the drive's pair was cut by when these were
/// found, the balance of profile radii's at that size (11.2527°, the cutter 47.2576 mm), not the
/// one the balance finds now (`FOUND_MILL`): each test here holds what was refused there.
fn scaled_pair(turned: bool) -> String {
    let call = "pair := HypoidPair(pinion_teeth: 24, gear_teeth: 48, mean_module: 1.25mm,\n  \
        shaft_angle: 90deg, offset: 15.625mm, spiral_angle: 25deg, backlash: 0.03125mm,\n  \
        tip_relief: 0.125mm, end_relief: 0.125mm, pressure_shift: 11.2527deg,\n  \
        cutter_radius: 47.2576mm)\n";
    let head = "unit mm\nuse std\nuse spiral_bevel.hypoid (HypoidPair)\n";
    if !turned { return format!("{head}{call}") }
    let ([u0,u1,u2],[v0,v1,v2],[o0,o1,o2]) = (TURNED_U,TURNED_V,TURNED_ORIGIN);
    format!("{head}place := frame\n\
        fix(dir == ({u0}, {u1}, {u2})) place.u\n\
        fix(dir == ({v0}, {v1}, {v2})) place.v\n\
        fix(origin == ({o0}mm, {o1}mm, {o2}mm)) place\nin place {{\n{call}}}\n")
}

/// The frame `scaled_pair` turns the pair into: its u, its v and its origin.
const TURNED_U: V = [-0.870151820509,0.447213595500,-0.206968135870];
const TURNED_V: V = [0.231397410491,0.,-0.972859310701];
const TURNED_ORIGIN: V = [0.,-34.184481591980,0.];

/// **Whether a member's tool builds does not depend on where the pair stands** (#904 A4): the
/// gear's generating crown, its tooth space and the neighbour's tooth unioned
/// (`pair.gear_space.body`), is built in the pair's own frame and in a frame turned to it, the
/// document solved as `solventc` solves it, and is the same solid. The neighbour's floor circle
/// lies in the plane tangent to the space's round and touches the round's torus twice; in the
/// turned frame that reading was refused ("its search read more boxes than any search may", the
/// run beside each touch proved only a hair wide: `enclosures::a_circle_touching_a_torus_is_read_alike_at_every_attitude`),
/// and the gear fell back to its field.
#[test]
fn the_gears_tool_builds_in_a_frame_turned_to_the_pair() {
    let volume = |turned: bool| -> f64 {
        let e = fixtures::solved_in_examples(&scaled_pair(turned));
        let tool = fixtures::solid(&e,"pair.gear_space.body");
        let built = gcs_core::solid::cad::recipe(&e.sketch,tool).and_then(|j| gcs_core::brep::recipe::build(&j))
            .unwrap_or_else(|e| panic!("turned {turned}: {e}"));
        gcs_core::brep::props::volume(&built)
    };
    let (own,turned) = (volume(false),volume(true));
    assert!((own-turned).abs() <= 1e-9*own,"{own} mm³ in its own frame, {turned} mm³ turned");
}

/// The gear's cut of `scaled_pair`, its sheet traced (`sweep::traced`), or with the grid its cutter's
/// band is read on turned each of `phases` past the band's start (`sweep::traced_turned`).
fn gear_sheets(turned: bool,phases: &[Option<f64>]) -> Vec<Result<gcs_core::brep::sweep::sheet::Fitted,String>> {
    use gcs_core::brep::sweep::{self,Say};
    use gcs_core::solid::admission::{admit_body,Options};
    let e = fixtures::solved_in_examples(&scaled_pair(turned));
    let gear = fixtures::solid(&e,"pair.gear.body");
    let admission = admit_body(&e.sketch,gear,&Options::default()).unwrap_or_else(|e| panic!("refused: {e}"));
    let recipe = gcs_core::solid::cad::recipe_static(&e.sketch,gear).unwrap();
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let prepared = sweep::prepare(&e.sketch,gear,&recipe,&admission,&say).unwrap_or_else(|r| panic!("{}",r.message));
    let k = (0..prepared.sweeps()).find(|&k| prepared.sweep_name(&e.sketch,k) == "pair.gear.cut.removal").expect("the gear's cut");
    phases.iter().map(|phase| match phase {
        None => sweep::traced(&prepared,k,None,&say),
        Some(phase) => sweep::traced_turned(&prepared,k,*phase,None,&say),
    }.map_err(|r| r.message)).collect()
}

/// **The gear's sheet is traced alike wherever its pair stands** (#904 A4): the gear's cut of
/// `scaled_pair`, traced in the pair's own frame and in the frame turned to it, is the same sheet —
/// as many rows and columns, each point the other's carried by the turn. The reach read its band on
/// a grid a world axis set, which a turn of the pair moved against the cutter; it now reads it from
/// where the contacts first reach the blank (`Cutter::reach`), and the rows are counted alike where
/// their span is a whole number of them to rounding (`trace::Tracer::columns`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 20 s: the pair solved and its gear's cut traced in two frames")]
fn the_gears_sheet_is_traced_alike_in_a_frame_turned_to_the_pair() {
    let [own,turned] = [false,true].map(|turned| gear_sheets(turned,&[None]).remove(0).unwrap());
    let (own,turned) = (&own.sheet,&turned.sheet);
    assert_eq!((own.rows,own.columns),(turned.rows,turned.columns));
    // the frame `scaled_pair` turns the pair into, carrying a point of the pair's own frame there
    let (u,v) = (TURNED_U,TURNED_V);
    let w = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
    let carried = |p: V| -> V { std::array::from_fn(|k| TURNED_ORIGIN[k]+p[0]*u[k]+p[1]*v[k]+p[2]*w[k]) };
    let apart = own.points.iter().zip(&turned.points).map(|(&p,&q)| dist(carried(p),q)).fold(0.,f64::max);
    assert!(apart < 1e-6,"the sheets' points are {apart} mm apart");
}

/// **Whether the gear builds exactly does not depend on where its pair stands** (#904 A4): the
/// gear of `scaled_pair`, built by the kernel in the pair's own frame and in the frame turned to
/// it, is the same solid. Its tool builds in both (`the_gears_tool_builds_in_a_frame_turned_to_the_pair`),
/// but the gear's cut was refused turned: "the candidate sheet cannot be widened out of the blank".
/// A revolved cutter's stations were read every 3.75 degrees round its axis from a half-plane a
/// world axis chose, so a turn of the pair moved that grid against the cutter, and the sheet traced
/// over it was refused at four of eleven phases of the grid within one step, in either frame
/// (`the_gears_sheet_is_traced_at_every_phase_of_its_stations`). The grid now starts where the
/// contacts first reach the blank (`brep::swept::cutter::Cutter::reach`), and the sheet's rows are
/// counted alike where its span is a whole number of them to rounding (`swept::trace`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 35 s: the gear built twice")]
fn the_gear_builds_alike_in_a_frame_turned_to_the_pair() {
    use gcs_core::brep::{export,sweep::Say};
    let built = |turned: bool| {
        let e = fixtures::solved_in_examples(&scaled_pair(turned));
        let say = Say {stage:&|_| {},mark:&|_| {}};
        export::exact(&e.sketch,fixtures::solid(&e,"pair.gear.body"),None,None,&say)
            .map(|x| (x.solid.faces.len(),gcs_core::brep::props::volume(&x.solid))).map_err(|r| r.message)
    };
    let (own,turned) = (built(false),built(true));
    let (Ok((faces,volume)),Ok((turned_faces,turned_volume))) = (&own,&turned) else {
        panic!("in its own frame {own:?}, turned {turned:?}")
    };
    assert_eq!(faces,turned_faces);
    assert!((volume-turned_volume).abs() <= 1e-9*volume,"{volume} mm³ in its own frame, {turned_volume} mm³ turned");
}

/// **The gear's sheet is traced at every phase of its cutter's stations** (#904 A4): the cut of
/// `scaled_pair`'s gear traced with the grid its cutter's band is read on turned eleven ways across
/// one step of it (`Cutter::turned`) is built at every one. A margin column's curve leaps into a far
/// corner's fan a row nearer the blank with each column further out; trimmed back to before that
/// leap, every column's rows stopped in the blank wherever the grid put a column leaping on the row
/// past the blank's last, and the sheet could not be widened out of it, at four of the eleven. Such
/// a column goes instead, with those beyond it (`brep::swept::trace::charted`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 14 s: eleven sheets traced")]
fn the_gears_sheet_is_traced_at_every_phase_of_its_stations() {
    let phases: Vec<f64> = (0..11).map(|i| 3.75*i as f64/11.).collect();
    let refused: Vec<String> = phases.iter().zip(gear_sheets(false,&phases.iter().map(|p| Some(p.to_radians())).collect::<Vec<_>>()))
        .filter_map(|(phase,sheet)| sheet.err().map(|r| format!("{phase:.3} degrees: {r}"))).collect();
    assert!(refused.is_empty(),"refused at {} of 11 phases: {refused:#?}",refused.len());
}

/// Each flank's margin against undercut as the pair reads it — its least area factor within its
/// blank, the `>= 0` each flank is held to — by the flank's name in `pair.balance`.
fn margins(e: &Elaborated) -> Vec<(String,f64)> {
    use gcs_core::constraints::{Arg,CKind};
    let sk = &e.sketch;
    sk.constraints.iter().filter(|c| c.kind == CKind::RegularWithin && c.free.is_none()).map(|c| {
        let Arg::Ent(f) = c.args[0] else { panic!("a regularity names its flank") };
        let name = sk.envelopes[f.i()].name.trim_start_matches("pair.balance.").to_string();
        (name,c.reading(sk).expect("a regularity reads its least area factor"))
    }).collect()
}

/// The pinion's and the gear's sweeps admitted: each cut's least area factor (E3's).
fn admitted(e: &Elaborated) -> [f64;2] {
    use gcs_core::solid::admission::{admit_body,Options};
    ["pinion","gear"].map(|m| {
        let a = admit_body(&e.sketch,fixtures::solid(e,&format!("pair.{m}.body")),&Options::default())
            .unwrap_or_else(|r| panic!("the {m} is refused: {r}"));
        a.sweeps().iter().find(|s| s.name == format!("pair.{m}.cut.removal")).unwrap().least_area_factor
    })
}

/// The pair's balance held: the two flanks it ties at `pair.least`, every flank no nearer its
/// fold, and each member's cut admitted at the least margin, as E3 samples it.
fn balanced(e: &Elaborated,label: &str,tied: [&str;2]) -> (f64,f64) {
    let (shift,least) = (unknown(e,"pair.pressure_shift"),unknown(e,"pair.least"));
    let read = margins(e);
    eprintln!("{label}: the shift {shift:.6}°, the least margin {least:.6}: {read:?}");
    assert_eq!(read.len(),4,"{label}: every flank the two blades cut is read");
    for (name,m) in &read {
        if tied.contains(&name.as_str()) {
            assert!((m-least).abs() < 1e-9,"{label}: {name} reads {m} against {least}");
        } else {
            assert!(*m > least,"{label}: {name} reads {m}, nearer its fold than the tie, {least}");
        }
    }
    let [pinion,gear] = admitted(e);
    for (m,a) in [("pinion",pinion),("gear",gear)] {
        assert!(a > 0. && a >= least-5e-3,"{label}: the {m}'s cut admitted at {a} against {least}");
    }
    (shift,least)
}

/// **The face mill's shift ties the flanks nearest undercut** (#904): with the cutter stated, the
/// pressure shift is found where the pinion's flank the inner blade cuts and the gear's the outer
/// cuts have one least area factor, `pair.least`, within their blanks; the pinion's other flank
/// and the gear's stand further from their folds, and each member's cut is admitted at that
/// least, as E3 samples it. As found here: 8.8549° (`FOUND_MILL`), the least 0.6767, with the
/// cutter the document states (`FOUND_MILL`'s).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 15 s: the balance solved and both cuts admitted")]
fn the_shift_ties_the_flanks_nearest_undercut() {
    let e = fixtures::gear::read_as_configured();
    let (shift,least) = balanced(&e,"24/48",["pinion_inner","gear_outer"]);
    assert!((shift-SHIFT).abs() < 1e-4 && (least-LEAST).abs() < 1e-5,"{shift}° {least}");
}

/// **The pair re-targeted finds its shift** (#904 A1): at 12/24 (12.5 mm between the shafts),
/// 13/27 (12 mm) and 16/32 at a 1.5 mm module (12 mm) — the issue's three, which the balance of
/// profile radii had no root at — and at a ten-tooth pinion, the document as written finds the
/// shift that ties the same two flanks, the others further from their folds, and both cuts are
/// admitted.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 90 s: four pairs balanced and admitted")]
fn the_pair_retargeted_finds_its_shift() {
    for (teeth,module,offset) in [([12,24],2.,12.5),([13,27],2.,12.),([16,32],1.5,12.),([10,48],2.,25.)] {
        let text = fixtures::gear::configure(&fixtures::gear::source(),&[("pinion_teeth",teeth[0].into()),
            ("gear_teeth",teeth[1].into()),("mean_module",module),("offset",offset)]);
        let label = format!("{}/{} m{module} E{offset}",teeth[0],teeth[1]);
        let (shift,least) = balanced(&fixtures::gear::read(&text),&label,["pinion_inner","gear_outer"]);
        assert!(shift > 0. && least > 0.5,"{label}: {shift}° {least}");
    }
}

/// **At no offset the hypoid's tie is a bound broken, and the pinion's own tie cuts it** (#904
/// A2): with no offset to protect it, the pinion's flank the outer blade cuts folds at the shift
/// that ties the pinion's inner flank to the gear's outer (8.3°), and the solve says the bound is
/// broken rather than cut an undercut pinion: where it stops, the sweep's admission refuses the
/// pinion's cut for that fold, on the crown's `outer` face. (The balance of profile radii found
/// 11.68° there and cut it.) Tied instead to the pinion's outer flank — the two flanks of the
/// pinion, the pair that binds with no offset — the shift is −1.195°, every flank held from its
/// fold, and the pinion's cut is admitted.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 5 min: the balance solved twice at no offset")]
fn at_no_offset_the_pinions_two_flanks_tie() {
    let text = fixtures::gear::configure(&fixtures::gear::source(),&[("offset",0.)]);
    let mut e = fixtures::gear::unsolved(&text);
    let r = gcs_core::solve::solve(&mut e.sketch,fixtures::accurate());
    assert!(!r.success && r.status == gcs_core::solve::BOUND_BROKEN,"{r:?}");
    let folded = margins(&e).into_iter().find(|(name,_)| name == "pinion_outer").unwrap().1;
    assert!(folded < 0.,"the pinion's outer flank reads {folded}");
    {
        use gcs_core::solid::admission::{admit_body,Condition,Error,Options};
        match admit_body(&e.sketch,fixtures::solid(&e,"pair.pinion.body"),&Options::default()) {
            Err(Error::Refused(r)) => {
                eprintln!("{r}");
                assert_eq!(r.condition,Condition::Fold,"{r}");
                assert!(r.message.contains("`pair.tooth.crown.outer`"),"the fold is said on the outer blade's face: {r}");
            }
            Ok(_) => panic!("the undercut pinion's cut was admitted"),
            Err(e) => panic!("{e}"),
        }
    }
    let tie = "pair.margins.gear_outer regular(pair.least, by: pair.margins.gear_outer_cut) within pair.margins.gear";
    assert!(text.contains(tie),"the document ties the gear's outer flank");
    let text = text.replace(tie,"pair.margins.pinion_outer regular(pair.least, by: pair.margins.pinion_cut) within pair.margins.pinion")
        .replace("pressure_shift: hint(10deg)","pressure_shift: hint(0deg)");
    let (shift,least) = balanced(&fixtures::gear::read(&text),"24/48 E0",["pinion_inner","pinion_outer"]);
    assert!((shift+1.195399).abs() < 1e-4 && (least-0.423065).abs() < 1e-5,"{shift}° {least}");
}
