//! **One stepped exact build for every object** (#670, #671): `brep::export::Builder` builds a
//! static body as the export builds it, a stage at a time like a swept one, and hands back its
//! `Display` — the mesh, its faces named, the B-rep's edges — which crosses between cores as one
//! buffer and is supplied to a sketch that builds no object of its own (`FieldMeshing::Supplied`):
//! one job a binding (`Sketch::object_jobs`), an object on its way until its build arrives, its
//! refusal said in the build's words, and an earlier drawing's build drawn, stale, till the new.
use gcs_core::brep::export::{self,Builder,Display};
use gcs_core::brep::sweep::Say;
use gcs_core::model::{objects,Sketch};
use gcs_core::solid::{ApproximationPolicy,ObjectState};
use std::f64::consts::PI;

fn solved(name: &str) -> Option<Sketch> {
    let e = fixtures::example(name)?;
    let mut sk = e.sketch;
    gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
    Some(sk)
}

fn document(text: &str) -> Sketch { fixtures::read(text).sketch }

/// Solid `i` built by the stepped builder to the end: its display, or the refusal's words.
fn stepped(sk: &Sketch,i: usize) -> Result<(Display,Vec<String>),String> {
    let mut b = Builder::new(sk,i).map_err(|r| r.message)?;
    let mut doing = vec![b.doing()];
    while !b.step().map_err(|r| r.message)? { doing.push(b.doing()); }
    let (done,total) = b.stages();
    assert_eq!(done,total,"`{}`: {doing:?}",sk.solids[i].name);
    Ok((b.display().unwrap().clone(),doing))
}

/// What the export builds of solid `i`: its face count and volume (model units), or its refusal.
fn exported(sk: &Sketch,i: usize) -> Result<(usize,f64),String> {
    let say = Say {stage:&|_: &str| {},mark:&|_| {}};
    let x = export::exact(sk,i,None,None,&say).map_err(|r| r.message)?;
    let mm = sk.units.mm_per();
    Ok((x.solid.faces.len(),gcs_core::brep::props::volume(&x.solid)/(mm*mm*mm)))
}

/// A display's surface area (model units squared).
fn area(d: &Display) -> f64 {
    use gcs_core::space::{cross,norm,sub};
    d.triangles.iter().map(|t| { let [a,b,c] = t.map(|k| d.vertices[k as usize]); 0.5*norm(cross(sub(b,a),sub(c,a))) }).sum()
}

/// Every object the documents `names` hold (swept or not, as `swept` says): built by the
/// stepped builder as the export builds it, with as many faces (a static one), each named, and the
/// same volume — a swept one's within the export's gross bar, since the display admits it at fewer
/// samples; or refused in the same words.  How many were built.
fn corpus(names: &[String],swept: bool) -> usize {
    names.iter()
        .map(|name| solved(name).map_or(0,|sk| objects_of(name,&sk,|s| s == swept)[swept as usize]))
        .sum()
}

/// **Solid `i` of document `name` (drawn as `sk`) built by the stepped builder as the export built
/// it** (`exact`): as `corpus` holds it, for a test that has the export's build in hand already and
/// would not build it twice
/// (`solid_sweeps_examples::the_bull_nose_turned_partway_stops_inside_the_ring`).
pub(crate) fn builds_as_exported(name: &str,sk: &Sketch,i: usize,exact: &export::Exact) {
    let mm = sk.units.mm_per();
    let volume = gcs_core::brep::props::volume(&exact.solid)/(mm*mm*mm);
    let (d,_) = stepped(sk,i).unwrap_or_else(|why| panic!("{name}: the export built it, the builder: {why}"));
    assert_eq!(d.faces.names.len(),d.faces.smooth.len(),"{name}: the faces named");
    let within = if sk.is_swept(i) { export::GROSS/mm*area(&d) } else { 1e-9*volume.abs().max(1.) };
    assert!((d.faces.volume-volume).abs() <= within,"{name}: {} against {volume}, within {within}",d.faces.volume);
}

/// **The kinds of construction object `i` is built of**: every solid under it by its form — a
/// prism (drafted or not), a prism through, a revolution (whole or part of a turn), a body (with
/// what it unites, cuts and keeps in common), a region (by its terms' shapes), a face family, a
/// placed solid — and each sweep it cuts by the class admission reads it in and its motion (or
/// its refusal).  What decides which way the export and the stepped builder go, so a document
/// holding a kind holds every document's object of that kind to the comparison.
fn kinds(sk: &Sketch,i: usize) -> std::collections::BTreeSet<String> {
    use gcs_core::model::SolidDef;
    use gcs_core::solid::admission::{admit_body,Class,Options};
    let variant = |d: &dyn std::fmt::Debug| {
        let t = format!("{d:?}");
        t.split(|c: char| !c.is_alphanumeric()).next().unwrap_or("").to_string()
    };
    let mut out = std::collections::BTreeSet::new();
    let mut todo = vec![i];
    let mut seen = std::collections::BTreeSet::new();
    while let Some(k) = todo.pop() {
        if !seen.insert(k) { continue }
        let Some(solid) = sk.solids.get(k) else { continue };
        match &solid.def {
            SolidDef::Placed {source,..} => { out.insert("placed".into()); todo.push(*source as usize) }
            SolidDef::Swept {source,..} => { out.insert("swept material".into()); todo.push(*source as usize) }
            SolidDef::Prism {draft,..} => { out.insert(if draft.is_some() { "prism, drafted" } else { "prism" }.into()); }
            SolidDef::Through {..} => { out.insert("prism through".into()); }
            SolidDef::Revolve {sweep,..} => {
                let whole = (sweep.value.abs()-std::f64::consts::TAU).abs() < 1e-9;
                out.insert(if whole { "revolution" } else { "revolution, part of a turn" }.into());
            }
            SolidDef::Body {stock,on,through,bound} => {
                for (what,set) in [("unites",on),("cuts",through),("keeps in common",bound)] {
                    if !set.is_empty() { out.insert(format!("body {what}")); }
                }
                todo.push(*stock as usize);
                todo.extend(on.iter().chain(through).chain(bound).map(|&o| o as usize));
            }
            SolidDef::Region {terms,..} => {
                out.insert("region".into());
                out.extend(terms.iter().map(|t| format!("region term {}",variant(&t.shape))));
            }
            SolidDef::Family {legs,..} => { out.insert(if legs.is_empty() { "face family" } else { "face family, chained" }.into()); }
        }
    }
    if sk.is_swept(i) {
        match admit_body(sk,i,&Options::default()) {
            Ok(a) => for e in a.sweeps() {
                out.insert(match &e.class {
                    Class::Generating => "sweep generating".to_string(),
                    Class::ConstantTwist(c) => format!("sweep constant twist, {}",c.kind()),
                    Class::Planar(_) => "sweep planar".into(),
                    Class::Subtraction(_) => "sweep planar subtraction".into(),
                    Class::Capped(_) => "sweep capped".into(),
                    Class::Closed(c) => format!("sweep closed, {}",c.kind()),
                });
            },
            Err(_) => { out.insert("sweep refused".into()); }
        }
    }
    out
}

/// The objects of document `name` (drawn as `sk`) whose sweeping `keep` admits, each held as
/// `corpus` holds it.  How many were built: the static ones, and the swept.
fn objects_of(name: &str,sk: &Sketch,keep: impl Fn(bool) -> bool) -> [usize; 2] {
    let mut built = [0,0];
    for i in objects(sk).into_iter().filter(|&i| keep(sk.is_swept(i))) {
        let swept = sk.is_swept(i);
        let solid = &sk.solids[i].name;
        match (exported(sk,i),stepped(sk,i)) {
            (Ok((faces,volume)),Ok((d,doing))) => {
                assert_eq!(d.faces.names.len(),d.faces.smooth.len(),"{name}: `{solid}`'s faces named");
                if !swept {
                    assert_eq!(doing[0],"building the solid from its recipe","{name}: `{solid}`");
                    assert_eq!(d.faces.smooth.len(),faces,"{name}: `{solid}`'s faces");
                }
                // a static body is the export's own build; a swept one is admitted at the
                // display's samples, and comes out within the export's gross bar of it: no
                // more volume than its surface swept that far
                let within = if swept { export::GROSS/sk.units.mm_per()*area(&d) } else { 1e-9*volume.abs().max(1.) };
                assert!((d.faces.volume-volume).abs() <= within,"{name}: `{solid}`: {} against {volume}, within {within}",d.faces.volume);
                built[swept as usize] += 1;
            }
            (Err(a),Err(b)) => assert_eq!(a,b,"{name}: `{solid}`"),
            (a,b) => panic!("{name}: `{solid}`: the export {:?}, the builder {:?}",a.map(|_| ()),b.map(|_| ())),
        }
    }
    built
}

/// **Every object the export builds, the builder builds the same; one it refuses, it refuses in
/// the same words** — here over documents of every kind of static body, and over a document of
/// every kind of object the library builds (`every_kind_of_object_builds_as_the_export_builds_it`).
#[test]
fn every_static_object_builds_as_the_export_builds_it() {
    let names: Vec<String> = ["solid_flange.sv","turned_shaft.sv","star_handle.sv","rounded_bracket.sv","solid_tray.sv",
        "pin_lug.sv","solid_elbow.sv"].map(String::from).to_vec();
    let built = corpus(&names,false);
    assert!(built >= names.len(),"{built} built");
}

/// The documents whose swept objects a test of their own holds — below (`corpus(&[…], true)`), or
/// an example's that builds it once for its volume and this (`builds_as_exported`): they hold the
/// kinds of those objects for `the_corpus_holds_every_kind_the_library_builds`.
const NAMED: [&str; 22] = ["solid_sweeps/vthread.sv","solid_sweeps/bar_end.sv",
    "solid_sweeps/next_tooth.sv","solid_sweeps/parted.sv","solid_sweeps/whole_turn.sv",
    "solid_sweeps/square_thread.sv","solid_sweeps/rocked.sv","solid_sweeps/flat_end.sv",
    "solid_sweeps/creased.sv","solid_sweeps/facing.sv","solid_sweeps/slot.sv","solid_sweeps/vgroove.sv",
    "solid_sweeps/punch.sv","solid_sweeps/drill.sv","solid_sweeps/spoke.sv","solid_sweeps/plunge.sv",
    "solid_sweeps/stopped_turn.sv","solid_sweeps/stopped_punch.sv","solid_sweeps/stopped_drum.sv",
    "solid_sweeps/helical_groove.sv","solid_sweeps/stopped_bullnose.sv","solid_sweeps/ball_end_mill.sv"];

/// Where each of `NAMED` is held, its sweep built by the stepped builder as the export builds it:
/// this file, or an example's own test that builds it once for both (`builds_as_exported`).
const HELD_IN: [&str; 2] = [include_str!("object_builder.rs"),include_str!("solid_sweeps_examples.rs")];

/// **A document of every kind of object the library builds** (`kinds`), besides those `NAMED`'s
/// tests hold: chosen from the library as the fewest and quickest documents that, with the swept
/// objects of `NAMED`, hold every kind it has (11 documents, about five seconds of building where
/// the library's 282 took minutes).  `the_corpus_holds_every_kind_the_library_builds` says when
/// one is missing.
const CORPUS: [&str; 11] = ["rounds/by_hand.sv","rounds/disc.sv","rounds/blank.sv",
    "vtwin/components/piston.sv","developable/frustum.sv","solid_twist.sv","swept_tumble.sv",
    "lofts/squares.sv","generation/cam_plate.sv","turbine_blade.sv","wankel/wankel.sv"];

/// **Every kind of object the library builds, the builder builds as the export does**
/// (`every_static_object_builds_as_the_export_builds_it` over the `CORPUS`): each document solved
/// once and its objects of both kinds held, the documents side by side.
#[test]
fn every_kind_of_object_builds_as_the_export_builds_it() {
    let built = gcs_core::par::map(&CORPUS,|name| solved(name).map_or([0,0],|sk| objects_of(name,&sk,|_| true)));
    let [statics,swept] = built.iter().fold([0,0],|t,b| [t[0]+b[0],t[1]+b[1]]);
    assert!(statics >= 8 && swept >= 2,"{statics} static objects built, {swept} swept");
}

/// **The corpus holds every kind of object in the library** (`kinds`): every document solved
/// (one whose solve takes minutes, `examples::SLOW`, read as it is drawn), each object's kinds
/// read — no object built — and each kind found in an object of `CORPUS`, or in a swept object of
/// one of `NAMED` (whose tests hold its sweeps).  A document with a kind none of them has is said,
/// to be added to the corpus.  It stands for the whole library built twice, which this test did
/// until #780 R1 (23 minutes; see `docs/testing.md`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, half a minute: every document solved and its objects' kinds read")]
fn the_corpus_holds_every_kind_the_library_builds() {
    use std::collections::{BTreeMap,BTreeSet};
    // each of `NAMED` held by a test of this file or an example's, or its sweep would be held
    // nowhere
    for name in NAMED {
        let file = name.rsplit('/').next().unwrap();
        assert!(HELD_IN[0].contains(&format!("\"{name}\".to_string()"))
            || HELD_IN[1].contains(&format!("builds_as_exported(\"{file}\"")),"`{name}` is held by no test");
    }
    let names = fixtures::example_names();
    // each document's objects' kinds, each with whether its object is swept
    let read = gcs_core::par::map(&names,|name| {
        let stem = name.rsplit('/').next().unwrap().trim_end_matches(".sv");
        let sk = match gcs_core::examples::SLOW.contains(&stem) {
            true => document_unsolved(name),
            false => solved(name),
        };
        sk.map_or(Vec::new(),|sk| objects(&sk).into_iter().flat_map(|i| {
            let swept = sk.is_swept(i);
            kinds(&sk,i).into_iter().map(move |k| (k,swept))
        }).collect::<Vec<_>>())
    });
    let mut held = BTreeSet::new();
    let mut found: BTreeMap<String,BTreeSet<&str>> = BTreeMap::new();
    for (name,ks) in names.iter().zip(&read) {
        let (corpus,named) = (CORPUS.contains(&name.as_str()),NAMED.contains(&name.as_str()));
        for (k,swept) in ks {
            found.entry(k.clone()).or_default().insert(name);
            if corpus || (named && *swept) { held.insert(k.clone()); }
        }
    }
    assert!(found.len() >= 30,"{} kinds in the library",found.len());
    let missing: Vec<String> = found.iter().filter(|(k,_)| !held.contains(*k))
        .map(|(k,docs)| format!("{k}: {docs:?}")).collect();
    assert!(missing.is_empty(),"kinds no document of the corpus holds:\n{}",missing.join("\n"));
}

/// Document `name` as it is drawn, not solved.
fn document_unsolved(name: &str) -> Option<Sketch> { fixtures::example(name).map(|e| e.sketch) }

/// **A sharp rim's fan builds at the display's samples** (`?example=solid_sweeps`, `vthread.sv`):
/// the rod's end planes meet the fan the V wheel's rim sweeps in a curve 0.09 mm long, shorter than
/// a march's step there (0.22 mm). Dropping every point within a quarter of a step of an end cut it
/// to a chord across the rim's tip, the meeting was traced again from a seed that chord missed, and
/// the split took the copy whose fit ran 3.4e-6 off the fan, refusing a cell. Each of the trace's
/// three rules builds it alone (`docs/contract/brep.md`): the turn's steps kept, the fit held to
/// both surfaces between its points, and one curve kept of a meeting traced twice.
#[test]
fn a_sharp_rims_fan_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/vthread.sv".to_string()],true),1);
}

/// **A prism's cap in the blank builds at the display's samples** (`bar_end.sv`, #684 F): the bar's
/// end, sectioned across, built by the page's stepped builder as the export builds it.
#[test]
fn a_bars_end_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/bar_end.sv".to_string()],true),1);
}

/// **A placed prism's cap builds at the display's samples** (`next_tooth.sv`, #684 AC): the bar
/// placed along the rack, its sections carried by the placement, built by the page's stepped
/// builder as the export builds it.
#[test]
fn a_placed_bars_end_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/next_tooth.sv".to_string()],true),1);
}

/// **A prism drafted both ways builds at the display's samples** (`parted.sv`, #684 P): the tooth's
/// sides folding at its parting plane, the stations crossing the crease's fan between its halves,
/// built by the page's stepped builder as the export builds it.
#[test]
fn a_tooth_drafted_both_ways_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/parted.sv".to_string()],true),1);
}

/// **A whole turn builds at the display's samples** (`whole_turn.sv`, #684 N): its sheet closed round
/// the turn, cut into faces both ways (round the turn, and where it runs onto and off the rim's
/// fan), built by the page's stepped builder as the export builds it.
#[test]
fn a_whole_turn_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/whole_turn.sv".to_string()],true),1);
}

/// **A prism under a screw builds at the display's samples** (`square_thread.sv`, #684 M): the bar's
/// characteristic walked along its rulings and round its corners' fans, carried along the helix,
/// built by the page's stepped builder as the export builds it.
#[test]
fn a_square_thread_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/square_thread.sv".to_string()],true),1);
}

/// **A drive builds at the display's samples** (`rocked.sv`, #684 L): the turn rocked out and back
/// by a law, read as the turn over the law's reach, built by the page's stepped builder as the
/// export builds it.
#[test]
fn a_rocked_turn_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/rocked.sv".to_string()],true),1);
}

/// **A prism's flat end turned builds at the display's samples** (`flat_end.sv`, #684 AE): the walk
/// from the bar's inner side round its end's rim and across the end, carried round the turn, built
/// by the page's stepped builder as the export builds it.
#[test]
fn a_flat_end_turned_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/flat_end.sv".to_string()],true),1);
}

/// **A prism drafted both ways turned builds at the display's samples** (`creased.sv`, #684 AG):
/// the walk from one half of the bar's inner side round the crease to the other, built by the
/// page's stepped builder as the export builds it.
#[test]
fn a_creased_bar_turned_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/creased.sv".to_string()],true),1);
}

/// **A ruling in contact all along it turned builds at the display's samples** (`facing.sv`,
/// #684 AF): the walk along the bar's inner side's ruling in the plane through the axis, carried
/// round the turn, built by the page's stepped builder as the export builds it.
#[test]
fn a_ruling_in_contact_all_along_it_turned_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/facing.sv".to_string()],true),1);
}

/// **A flat face kept in its own plane slid builds at the display's samples** (`slot.sv`, #684 O):
/// the walk down one wall, across the end mill's flat bottom and up the other, its sheet creased
/// where it runs onto and off the bottom, built by the page's stepped builder as the export builds
/// it.
#[test]
fn a_flat_face_kept_in_its_plane_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/slot.sv".to_string()],true),1);
}

/// **A sharp edge along one point's path slid builds at the display's samples** (`vgroove.sv`,
/// #684 V): the walk down one flank of the V wheel and straight onto the other at the rim's one
/// point, its sheet creased there, built by the page's stepped builder as the export builds it.
#[test]
fn a_sharp_edge_along_one_points_path_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/vgroove.sv".to_string()],true),1);
}

/// **A characteristic closed on itself builds at the display's samples** (`punch.sv`, #684 W): the
/// walk round the drum, along each side's line and across each flat end, closing where it began,
/// its sheet a tube cut at its four creases, built by the page's stepped builder as the export
/// builds it.
#[test]
fn a_closed_characteristic_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/punch.sv".to_string()],true),1);
}

/// **A ring in contact all round builds at the display's samples** (`drill.sv`, #684 X): the drill
/// slid along its own axis, its characteristic its ring of greatest radius, its sheet the cylinder
/// that ring sweeps, written exactly, built by the page's stepped builder as the export builds it.
#[test]
fn a_ring_in_contact_all_round_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/drill.sv".to_string()],true),1);
}

/// **A ring in contact all round under a turn builds at the display's samples** (`spoke.sv`, #684 X):
/// the pin swung along a radius, the walk turning onto its rim's ring half round and off onto its
/// bottom line, the branch across its end trimmed, built by the page's stepped builder as the export
/// builds it.
#[test]
fn a_ring_in_contact_all_round_turned_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/spoke.sv".to_string()],true),1);
}

/// **A sweep of the closed class builds at the display's samples** (`plunge.sv`, `stopped_turn.sv`,
/// `stopped_punch.sv`, `stopped_drum.sv`, #721): a polyhedral tool's slide and turn and a drum's
/// slide and turn, each stopping inside its blank, cut from it on one arrangement by the page's
/// stepped builder as the export builds it — admitted, its blank built, and cut in the stage after,
/// with no sheet, plan or sector.
#[test]
fn a_closed_sweep_builds_at_the_displays_samples() {
    assert_eq!(corpus(&["solid_sweeps/plunge.sv".to_string(),"solid_sweeps/stopped_turn.sv".to_string(),
        "solid_sweeps/stopped_punch.sv".to_string(),"solid_sweeps/stopped_drum.sv".to_string()],true),4);
}

const BLOCK: &str = "unit mm\nuse std\nuse std.solids (union)\nuse std.rounds (round)\nin std.front {\n  a := point hint((0, 0))\n  b := point hint((40, 0))\n  \
    c := point hint((40, 20))\n  d := point hint((0, 20))\n  fix((0, 0)) a\n  l1 := line(a, b)\n  l2 := line(b, c)\n  \
    l3 := line(c, d)\n  l4 := line(d, a)\n  horizontal l1\n  horizontal l3\n  vertical l2\n  vertical l4\n  \
    length(40) l1\n  length(20) l2\n}\nblock := solid(std.solids.Prism(face(l1, l2, l3, l4), from: -10, to: 0))\n";

/// A static body is a builder of two stages — its recipe built, then meshed — which says what it
/// is doing as a swept one does, and its display names each face and draws each edge: a block's
/// twelve, every one a crease.
#[test]
fn a_static_body_is_built_by_stages_with_its_faces_named_and_its_edges_drawn() {
    let sk = document(BLOCK);
    let block = objects(&sk)[0];
    let mut b = Builder::new(&sk,block).unwrap();
    assert_eq!(b.stages(),(0,2));
    assert_eq!(b.doing(),"building the solid from its recipe");
    assert!(!b.step().unwrap());
    assert_eq!(b.doing(),"meshing the exact surface");
    assert!(b.said().unwrap().contains("built the solid by the Rust kernel"),"{:?}",b.said());
    assert!(b.step().unwrap());
    assert_eq!(b.stages(),(2,2));
    let d = b.display().unwrap();
    assert!((d.faces.volume-8000.).abs() < 1e-6,"{}",d.faces.volume);
    assert_eq!(d.faces.smooth,vec![false;6]);
    assert_eq!(d.faces.names.len(),6);
    assert!(d.faces.names.iter().all(|n| n.starts_with("block.")),"{:?}",d.faces.names);
    assert_eq!(d.faces.edges.len(),12,"{:?}",d.faces.edges);
    assert!(d.faces.edges.iter().all(|e| !e.smooth && e.faces[0] < e.faces[1] && e.points.len() >= 2));
    // each edge runs between two corners of the block, 40, 20 or 10 long
    for e in &d.faces.edges {
        let [p,q] = [e.points[0],*e.points.last().unwrap()].map(|k| d.vertices[k as usize]);
        let l = gcs_core::space::distance(p,q);
        assert!([40.,20.,10.].iter().any(|w| (l-w).abs() < 1e-9),"{l}");
    }
}

/// A round's edges meet its faces tangent, and are drawn smooth; the rest of the body's are
/// creases.
#[test]
fn a_rounds_tangent_edges_are_smooth() {
    let sk = document(&format!("{BLOCK}block.near round(r: 2mm) block\n"));
    let body = objects(&sk)[0];
    let (d,_) = stepped(&sk,body).unwrap();
    assert!(d.faces.edges.iter().any(|e| e.smooth),"{:?}",d.faces.edges.iter().map(|e| e.smooth).collect::<Vec<_>>());
    assert!(d.faces.edges.iter().any(|e| !e.smooth));
}

/// A display crosses between cores as one buffer, read back as it was written; a buffer cut short,
/// of another format or naming a vertex that is not there is refused, never read as a surface.
#[test]
fn a_display_crosses_as_one_buffer_and_a_bad_one_is_refused() {
    let sk = document(BLOCK);
    let (d,_) = stepped(&sk,objects(&sk)[0]).unwrap();
    let bytes = d.bytes();
    let back = Display::read(&bytes).unwrap();
    assert_eq!(back.vertices,d.vertices);
    assert_eq!(back.triangles,d.triangles);
    assert_eq!(back.faces.of,d.faces.of);
    assert_eq!(back.faces.smooth,d.faces.smooth);
    assert_eq!(back.faces.names,d.faces.names);
    assert_eq!(back.faces.edges,d.faces.edges);
    assert_eq!(back.faces.volume,d.faces.volume);
    assert!(Display::read(&bytes[..bytes.len()-1]).unwrap_err().contains("not as long as it says"));
    let mut other = bytes.clone();
    other[..8].copy_from_slice(&99f64.to_le_bytes());
    assert!(Display::read(&other).unwrap_err().contains("another format"));
    let mut wild = d.clone();
    wild.triangles[0][0] = wild.vertices.len() as u32;
    assert!(Display::read(&wild.bytes()).unwrap_err().contains("vertex past"));
    assert!(Display::read(&[]).is_err());
}

/// Two closed instances of one component, standing apart: one binding, placed twice.
const PAIR: &str = "unit mm\nuse std\nuse std.solids (union)\nuse std.place (offset)\ncomponent Plate(w: Length) {\n  in std.front {\n    \
    a := point hint((1, 1))\n    b := point hint((w, 1))\n    c := point hint((w, w))\n    e := point hint((1, w))\n    \
    a offset(dx: 1mm, dy: 1mm) std.origin\n    b offset(dx: w, dy: 1mm) std.origin\n    c offset(dx: w, dy: w) std.origin\n    \
    e offset(dx: 1mm, dy: w) std.origin\n    ab := line(a, b)\n    bc := line(b, c)\n    ce := line(c, e)\n    \
    ea := line(e, a)\n    body := solid(std.solids.Prism(face(ab, bc, ce, ea), from: 0mm, to: 2mm))\n  }\n}\n\
    in (f := frame) { one := Plate(w: 11mm) }\nfix(dir == (1, 0, 0)) f.u\nfix(dir == (0, 1, 0)) f.v\nfix(origin == (0, 0, 0)) f\n\
    in (g := frame) { two := Plate(w: 11mm) }\nfix(dir == (1, 0, 0)) g.u\nfix(dir == (0, 1, 0)) g.v\nfix(origin == (30, 0, 0)) g\n";

/// **A sketch that builds no object of its own** (#671): every object is on its way until its
/// build is supplied — refused here, never built on the thread that draws, and not unbuilt — then
/// drawn from the build; a refused build is unbuilt in its words; a changed drawing draws the
/// earlier build, stale, till the new one arrives.
#[test]
fn a_sketch_supplied_its_objects_builds_none_of_its_own() {
    let sk = document(BLOCK);
    let block = objects(&sk)[0];
    sk.supply_objects(true);
    let jobs = sk.object_jobs();
    assert_eq!(jobs.len(),1);
    assert_eq!((jobs[0].solid,jobs[0].objects.clone(),jobs[0].swept),(block,vec![block],false));
    assert_eq!(sk.object_state(block),ObjectState::Building);
    let refused = sk.evaluated_solid(block,ApproximationPolicy::Mesh).err().unwrap();
    assert!(refused.contains("still being built"),"{refused}");
    assert!(gcs_core::overview::unbuilt(&sk).is_empty(),"on its way, not unbuilt");
    // its build supplied, as a worker hands it over
    let (d,_) = stepped(&sk,block).unwrap();
    assert!(sk.supply_object(block,Display::read(&d.bytes()).unwrap()).unwrap());
    assert_eq!(sk.object_state(block),ObjectState::Built);
    let solid = sk.evaluated_solid(block,ApproximationPolicy::Mesh).unwrap();
    assert!((solid.volume()-8000.).abs() < 1e-6);
    assert!(!solid.stale());
    assert!(solid.surviving_faces().iter().any(|f| f.starts_with("block.")),"{:?}",solid.surviving_faces());
    assert!(solid.edges().iter().any(|e| !e.smooth));
    // the drawing changed: the earlier build is drawn till the new one arrives
    let mut moved = sk.clone();
    let x: Vec<f64> = moved.get_x();
    moved.set_x(&x.iter().map(|v| v*1.5).collect::<Vec<_>>());
    if gcs_core::solid::reads(&moved,block,0.) != gcs_core::solid::reads(&sk,block,0.) {
        moved.supply_objects(true);
        assert_eq!(moved.object_state(block),ObjectState::Rebuilding);
        assert!(moved.evaluated_solid(block,ApproximationPolicy::Mesh).unwrap().stale());
    }
    // a refusal is the build's words
    sk.refuse_object(block,"`block`: the built boundary is invalid").unwrap();
    assert_eq!(sk.object_state(block),ObjectState::Refused("`block`: the built boundary is invalid".into()));
    assert_eq!(gcs_core::overview::unbuilt(&sk),vec![(block,"`block`: the built boundary is invalid".to_string())]);
}

/// **One job a binding** (#671): two placements of one component's solid are one build, of the
/// first, and each is drawn from it where it stands.
#[test]
fn the_placements_of_one_binding_are_one_job() {
    let sk = document(PAIR);
    let objs = objects(&sk);
    assert_eq!(objs.len(),2,"{:?}",objs.iter().map(|&i| &sk.solids[i].name).collect::<Vec<_>>());
    sk.supply_objects(true);
    let jobs = sk.object_jobs();
    assert_eq!(jobs.len(),1,"{jobs:?}");
    assert_eq!(jobs[0].objects,objs);
    let (d,_) = stepped(&sk,jobs[0].solid).unwrap();
    assert!(sk.supply_object(jobs[0].solid,d).unwrap());
    let boxes: Vec<_> = objs.iter().map(|&i| {
        assert_eq!(sk.object_state(i),ObjectState::Built,"`{}`",sk.solids[i].name);
        let s = sk.evaluated_solid(i,ApproximationPolicy::Mesh).unwrap();
        assert!((s.volume()-200.).abs() < 1e-6,"{}",s.volume());
        s.world_bounds()
    }).collect();
    assert!((boxes[1].lo[0]-boxes[0].lo[0]-30.).abs() < 1e-6,"{boxes:?}");
}

/// **A face family's job keeps its key through the page's write-back** (#671): the page solves,
/// writes the drawing back into its source (`edit::reconcile`, which here drops the seeds the
/// fixed frame points do not need) and hands a worker that text with its values.  The family's
/// key reads its running number's name, so that name is the family's count and not where it
/// stands: the worker names the job as the page did, though the write-back moved the family.
#[test]
fn a_face_familys_job_keeps_its_key_through_the_write_back() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"),"/../examples/paths/coil.sv")).unwrap();
    let mut page = fixtures::read(&text);
    gcs_core::solve::solve(&mut page.sketch,gcs_core::solve::SolveOpts::default());
    page.sketch.supply_objects(true);
    let jobs = page.sketch.object_jobs();
    assert_eq!(jobs.iter().map(|j| j.name.as_str()).collect::<Vec<_>>(),["coil"]);
    let sk = std::mem::take(&mut page.sketch);
    let written = gcs_core::edit::reconcile(&mut page,&sk);
    assert_eq!(written.refused,None);
    let sweep = |t: &str| t.find("coil := solid(").unwrap();
    assert_ne!(sweep(&written.text),sweep(&text),"the write-back moved the family:\n{}",written.text);
    let mut worker = document(&written.text);
    assert!(worker.set_x(&sk.get_x()));
    assert_eq!(worker.object_jobs(),jobs,"the worker's job is the page's");
}

/// **`exact_solid` and the builder agree** (#536): a body a generating sweep cuts is built by
/// `exact_solid` as the export builds it, where the sketch builds its own solids; where a host
/// builds them, the refusal names the builder that does.
#[test]
fn exact_solid_builds_a_generated_body_as_the_export_does() {
    let sk = solved("swept_torus.sv").unwrap();
    let part = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "part").unwrap();
    let x = sk.exact_solid(part).unwrap();
    let (faces,volume) = exported(&sk,part).unwrap();
    assert_eq!(x.brep.faces.len(),faces);
    assert!((gcs_core::brep::props::volume(&x.brep)/x.mm.powi(3)-volume).abs() < 1e-9*volume);
    let hosted = solved("swept_torus.sv").unwrap();
    hosted.supply_objects(true);
    let why = hosted.exact_solid(part).unwrap_err();
    assert!(why.contains("`brep::export::Builder`"),"{why}");
}

/// **A build is current while what its solid reads is what the build read, within a solve's own
/// tolerance** (#671): a solve settling the drawing again changes nothing a build reads, and the
/// supply says it is current, so a host files it under the job's key and asks nothing again; an
/// edit leaves the build stale, drawn till the new one arrives.
#[test]
fn a_build_is_current_within_a_solves_tolerance() {
    let sk = document(BLOCK);
    let block = objects(&sk)[0];
    sk.supply_objects(true);
    let key = sk.object_jobs()[0].key;
    let (d,_) = stepped(&sk,block).unwrap();
    assert_eq!(gcs_core::solid::reads_key(&d.reads),key,"the build read what the drawing reads");
    // what a solve settling the block again leaves: its numbers a hair from the build's
    let nudged = Display {reads:d.reads.iter().map(|x| x*(1.+1e-14)).collect(),..d.clone()};
    assert!(sk.supply_object(block,nudged.clone()).unwrap());
    assert_eq!(sk.object_state(block),ObjectState::Built);
    assert_eq!(sk.object_jobs()[0].key,key,"the drawing's own key");
    // an earlier drawing's build: stale, and the drawing's own key asked
    let earlier = Display {reads:d.reads.iter().map(|x| x*1.01+1.).collect(),..d.clone()};
    assert!(!sk.supply_object(block,earlier).unwrap(),"an earlier drawing's");
    assert_eq!(sk.object_state(block),ObjectState::Rebuilding);
    assert!(sk.evaluated_solid(block,ApproximationPolicy::Mesh).unwrap().stale());
}

/// **A body built from swept material is built as the export composes it, to any depth** (the
/// radiator bot's compounds): a shaped gear united with a copy of itself screwed along its axis —
/// two swept operands, one a placement — and a body made of that compound, whose stock holds the
/// sweeps a level down; each operand built by its own stages, the compound combined, then meshed.
#[test]
fn a_body_of_swept_bodies_is_built_by_stages_as_the_export_composes_it() {
    let src = crate::rack_cut::SPACE.to_string() + "\
screw := frame
use std.motion (screws)
screw screws(48mm) turn.axis
construction lifted := gear at(under: screw) 30deg
construction compound := solid(gear)
lifted union compound
nested := solid(compound)
";
    let mut e = crate::common::build(&src);
    let r = gcs_core::solve::solve(&mut e.sketch,gcs_core::solve::SolveOpts::default());
    assert!(r.success,"{}",r.message);
    let sk = e.sketch;
    let nested = e.map.ent_named("nested").unwrap().i();
    let (faces,volume) = exported(&sk,nested).unwrap();
    let (d,doing) = stepped(&sk,nested).unwrap();
    assert!(doing.iter().filter(|d| d.starts_with("admitting its sweeps")).count() == 2,"{doing:?}");
    assert!(doing.iter().any(|d| d == "adding the rest of the body"),"{doing:?}");
    let within = export::GROSS/sk.units.mm_per()*area(&d);
    assert!((d.faces.volume-volume).abs() <= within,"{} against {volume}, within {within}",d.faces.volume);
    // the two gears overlap 2 mm of their 6, each turned off the other's tooth space
    let gear = PI*(22.*22.-0.)*6.;
    assert!(volume > gear && volume < 2.*gear,"{volume} ({faces} faces)");
    // each gear's tooth space named for the sweep that cut it
    assert_eq!(d.faces.names.iter().filter(|n| *n == "space").count(),2,"{:?}",d.faces.names);
}

