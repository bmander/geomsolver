//! **OCCT's fillet as the oracle of the rounds the lowering builds** (#218's 8, #194): for each
//! rounds example whose body this kernel builds and exports, the same body without its rounds (the
//! stratum: its recipe less the lowered regions) built by OCCT, the same edges rounded by OCCT's own
//! fillet (`BRepFilletAPI_MakeFillet`) or chamfered (`BRepFilletAPI_MakeChamfer`), and the two
//! held to one another: the volume (`EXACT` where both are analytic, else OCCT's own accuracy,
//! `APPROXIMATE`, not ours: `tests/round_lowering.rs` holds ours to the closed forms), and the
//! kinds of surface their faces lie on — OCCT approximating by a B-spline some analytic face this
//! kernel builds exactly (a round between two cylinders, the knob's neck). The edges are read off
//! the lowering's pieces (a point on each edge and its size), so OCCT is told nothing a person
//! would not: which edges, how big. Where OCCT's fillet fails, or makes no valid solid (this
//! kernel's own export of each is held by `tests/cli.rs` and `round_lowering.rs`), that is said
//! and counted, never passed over silently.
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
use gcs_core::brep::{export,geom::Rigid,props::volume,sweep::Say};
use gcs_core::json::Json;
use gcs_core::model::{RegionShape,SolidDef};
use gcs_core::solid::lowering::{self,Piece,Run,Stroke};
use gcs_core::space::{add,distance as dist,norm,scale};
use std::collections::BTreeSet;

/// How near OCCT's volume ours must be where both build every face analytically.
const EXACT: f64 = 1e-9;

/// And where either fits one by a B-spline: OCCT's fit (and its integration of one) is good to a
/// few parts in a million (this kernel's own rolled pieces are held to #85's quadrature to 1e-6).
const APPROXIMATE: f64 = 2e-5;

type V = [f64;3];

/// The example's source, beside the rest of the corpus.
fn source(file: &str) -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/rounds").join(format!("{file}.sv"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}",path.display()))
}

/// What a lowered piece says of the edge it rounds or chamfers: a point on the edge, the size
/// (a round's radius, a chamfer's leg) and whether it is a chamfer. A corner's cell says nothing
/// its three edges do not.
fn edge_of(p: &Piece) -> Option<(V,f64,bool)> {
    // a section's size: its ball's arc, or the leg from the edge along its first face
    let size = |r: &Run| -> (f64,bool) {
        match r.strokes.iter().find_map(|(s,n)| match s { Stroke::Arc {radius,..} if *n == "round" => Some(*radius),_ => None }) {
            Some(radius) => (radius,false),
            None => match &r.strokes[0].0 {
                Stroke::Line {from,to} => (dist(*from,*to),true),
                Stroke::Arc {radius,angles,..} => (radius*(angles[1]-angles[0]).abs(),true),
                Stroke::Conic {..} => unreachable!(),
            },
        }
    };
    match p {
        Piece::Run(r) => {
            let (s,chamfer) = size(r);
            // a little way along the edge from its first end
            Some((add(r.origin,scale(r.along,0.5*s)),s,chamfer))
        }
        Piece::Ring(g) => {
            let (s,chamfer) = size(&g.section);
            // half way round its turn
            Some((Rigid::turn(g.axis_origin,g.axis,0.5*g.angle).point(g.section.origin),s,chamfer))
        }
        Piece::Rolled(k) => Some(match k.flat { None => (k.at,k.r,false),Some(([la,_],_)) => (k.at,la,true) }),
        Piece::Corner(_) => None,
    }
}


/// The body's recipe less its lowered regions: what the rounds read.
fn stratum(recipe: &Json) -> Json {
    let nodes = recipe.get("nodes").unwrap().arr();
    let lowered: BTreeSet<i64> = nodes.iter().filter(|n| n.get("kind").unwrap().as_str() == "lowered")
        .map(|n| n.get("id").unwrap().as_i64()).collect();
    let kept = nodes.iter().filter(|n| !lowered.contains(&n.get("id").unwrap().as_i64())).map(|n| {
        let mut n = n.clone();
        for key in ["on","cut","bound","on_late","cut_late"] {
            if let Some(Json::Arr(list)) = n.get(key) {
                let list = list.iter().filter(|x| !lowered.contains(&x.as_i64())).cloned().collect();
                n.set(key,Json::Arr(list));
            }
        }
        n
    }).collect();
    let mut out = recipe.clone();
    out.set("nodes",Json::Arr(kept));
    out
}

/// OCCT's `GeomAbs_SurfaceType` as this kernel names its surfaces.
fn occt_kind(k: i32) -> &'static str {
    ["plane","cylinder","cone","sphere","torus","bezier","bspline","revolution","extrusion","offset","other"]
        .get(k as usize).copied().unwrap_or("other")
}

/// Where two solids disagree, sampled over ours's box: how many of the points read inside one and
/// outside the other, and the first few.
fn apart(session: &native::Session,ours: &gcs_core::brep::topo::Brep,theirs: std::os::raw::c_int) -> String {
    use gcs_core::brep::query::{Located,Place};
    let (lo,hi) = ours.bounds();
    let located = Located::owned(ours.clone(),1e-9*ours.size());
    let mut rng = gcs_core::rng::Rng::new(218);
    let points: Vec<V> = (0..20000).map(|_| std::array::from_fn(|j| rng.uniform(lo[j],hi[j]))).collect();
    let states = session.solid_contains(theirs,&points,1e-9).unwrap();
    let apart: Vec<(&V,&str)> = points.iter().zip(&states).filter_map(|(p,s)| match located.solid_place(*p) {
        Place::In if *s == 0 => Some((p,"ours")),Place::Out if *s == 1 => Some((p,"OCCT's")),_ => None,
    }).collect();
    format!("; {} of {} points apart, as {:?}",apart.len(),points.len(),&apart[..apart.len().min(4)])
}

#[test]
fn every_rounds_example_is_occts_fillet() {
    let session = native::Session::new().unwrap();
    let examples = [("block","block"),("by_hand","block"),("pocket","dish"),("boss","pad"),("slot","part"),("pad","pad"),
        ("rail","rail"),("knob","knob"),("tee","tee"),("bore","pipe"),("runout","lug"),("runout","vent"),("twin","bar"),
        ("post","stand"),("dshaft","shaft")];
    let (mut failures,mut refused) = (Vec::new(),0);
    for (file,name) in examples {
        let (prog,errs,linked) = gcs_core::library::parse_linked(&source(file));
        assert!(errs.is_empty() && linked.is_empty(),"{file}: {errs:?} {linked:?}");
        let mut e = gcs_core::program::elaborate(&prog);
        assert!(e.ok(),"{file}: does not elaborate");
        assert!(gcs_core::solve::solve(&mut e.sketch,gcs_core::solve::SolveOpts::default()).success,"{file}: does not solve");
        let sk = &e.sketch;
        let body = e.map.ent_named(name).unwrap_or_else(|| panic!("{file}: no `{name}`")).i();
        // ours: the body as this kernel exports it
        let say = Say {stage: &|_: &str| {},mark: &|_| {}};
        let exact = export::exact(sk,body,None,None,&say).unwrap_or_else(|r| panic!("{file}: {r:?}"));
        let ours = volume(&exact.solid);
        let our_kinds: BTreeSet<&str> = exact.solid.faces.iter().map(|f| f.surface.kind()).collect();
        // the edges its regions round, each once
        let mut rounds: Vec<(V,f64)> = Vec::new();
        let mut flats: Vec<(V,f64)> = Vec::new();
        // (of the regions over this body's faces)
        let over = |i: usize| matches!(&sk.solids[i].def,SolidDef::Region {terms,..} if terms.iter()
            .any(|t| matches!(&t.shape,RegionShape::FaceSide {face} if face.solid as usize == body)));
        for i in (0..sk.solids.len()).filter(|&i| over(i)) {
            let Some(Ok(pieces)) = lowering::lowered(sk,i) else { continue };
            for (at,size,chamfer) in pieces.iter().filter_map(edge_of) {
                let list = if chamfer { &mut flats } else { &mut rounds };
                if !list.iter().any(|(p,_)| dist(*p,at) <= 1e-9*(1.+norm(at))) { list.push((at,size)); }
            }
        }
        // theirs: OCCT's body less the rounds, rounded by OCCT
        let recipe = gcs_core::solid::cad::recipe(sk,body).unwrap();
        let theirs = session.construct(&stratum(&recipe)).and_then(|mut id| {
            if !rounds.is_empty() { id = session.round(id,&rounds,false)?; }
            if !flats.is_empty() { id = session.round(id,&flats,true)?; }
            session.validate(id).map_err(|e| format!("its fillet is no valid solid: {e}"))?;
            Ok(id)
        });
        let theirs = match theirs {
            Ok(id) => id,
            Err(err) => { eprintln!("{file} `{name}`: OCCT does not round it: {err}"); refused += 1; continue }
        };
        let w = session.volume(theirs).unwrap();
        let their_kinds: BTreeSet<&str> = session.faces(theirs).unwrap().into_iter()
            .map(|f| occt_kind(session.face_kind(f).unwrap())).collect();
        let rel = (ours-w).abs()/w.abs();
        eprintln!("{file} `{name}`: {} rounds, {} flats; {ours:.9} mm³ against OCCT's {w:.9} ({rel:.1e}); faces {our_kinds:?} against \
            {their_kinds:?}",rounds.len(),flats.len());
        let fitted = our_kinds.contains("bspline") || their_kinds.contains("bspline");
        if rel > if fitted { APPROXIMATE } else { EXACT } {
            failures.push(format!("{file} `{name}`: volume {ours} against OCCT's {w} ({rel:.1e}){}",
                apart(&session,&exact.solid,theirs)));
        }
        // every kind of OCCT's ours, and every one of ours OCCT's or a B-spline OCCT fits in its place
        let alike = their_kinds.iter().all(|k| *k == "bspline" || our_kinds.contains(k))
            && our_kinds.iter().all(|k| their_kinds.contains(k) || their_kinds.contains("bspline"));
        if !alike { failures.push(format!("{file} `{name}`: faces {our_kinds:?} against OCCT's {their_kinds:?}")); }
    }
    eprintln!("{} compared, {refused} OCCT does not round",examples.len()-refused);
    assert!(failures.is_empty(),"{} disagreements:\n{}",failures.len(),failures.join("\n"));
}
