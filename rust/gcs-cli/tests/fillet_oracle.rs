//! **OCCT's fillet as the oracle of the rounds the lowering builds** (#218's 8, #194): for each
//! rounds example whose body this kernel builds and exports, the same body without its rounds (the
//! stratum: its recipe less the lowered regions) built by OCCT, the same edges rounded by OCCT, and
//! the two held to one another: the volume (`EXACT` where both are analytic, else OCCT's own
//! accuracy, `APPROXIMATE`, not ours: `tests/round_lowering.rs` holds ours to the closed forms), and
//! the kinds of surface their faces lie on — every one of ours OCCT's too, and OCCT's ours but for a
//! B-spline it fits where this kernel builds a face exactly (a round between two cylinders, the
//! knob's neck). The edges are read off the lowering's pieces (a point on each edge and its size),
//! so OCCT is told nothing a person would not: which edges, how big. OCCT rounds them by `How`:
//! its fillet or chamfer (`BRepFilletAPI_MakeFillet`, `BRepFilletAPI_MakeChamfer`), asked again of
//! the same solid with its seams turned (`seams_turned`) where it fails on an edge along or ending
//! on a periodic surface's seam; a convex body rounded all over by its offsets (its fillet fills a
//! corner of three rounds beside a rolled one by a surface of its own, its offsets by the ball); and
//! a chamfer beside a curved face — where OCCT's is ruled between legs along the faces and
//! `std.Flat`'s is where the faces' distances, each over the other's leg, sum to one — by a loft
//! through std's flat's own sections, worked out in closed form, ours also read by OCCT's own
//! distances (`flats_by_occt`).
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
use gcs_core::space::{add,cross,distance as dist,dot,norm,scale,sub};
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
fn edge_of(p: &Piece) -> Option<(V,f64,bool)> { edge_and_kind(p).map(|(at,size,chamfer,_)| (at,size,chamfer)) }

/// `edge_of`, and whether a chamfer's flat stands beside a curved face (#218's 5, 6): there
/// `std.Flat`'s flat is where the two faces' distances, each over the other's leg, sum to one —
/// its legs measured from the other face's surface — and OCCT's chamfer is ruled between legs
/// measured along the faces from the edge: two definitions, which only between planes agree.
fn edge_and_kind(p: &Piece) -> Option<(V,f64,bool,bool)> {
    let curved = |r: &Run| r.strokes.iter().any(|(s,_)| !matches!(s,Stroke::Line {..}));
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
            Some((add(r.origin,scale(r.along,0.5*s)),s,chamfer,chamfer && curved(r)))
        }
        Piece::Ring(g) => {
            let (s,chamfer) = size(&g.section);
            // half way round its turn
            Some((Rigid::turn(g.axis_origin,g.axis,0.5*g.angle).point(g.section.origin),s,chamfer,chamfer && curved(&g.section)))
        }
        Piece::Rolled(k) => Some(match k.flat { None => (k.at,k.r,false,false),Some(([la,_],_)) => (k.at,la,true,true) }),
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

/// `v` turned through `angle` about the unit `k` (Rodrigues).
fn turned(v: V,k: V,angle: f64) -> V {
    let (sn,cs) = angle.sin_cos();
    add(add(scale(v,cs),scale(cross(k,v),sn)),scale(k,dot(k,v)*(1.-cs)))
}

fn vector_of(j: &Json) -> V { std::array::from_fn(|i| j.arr()[i].as_f64()) }
fn json_of(v: V) -> Json { Json::Arr(v.iter().map(|&x| Json::Num(x)).collect()) }

/// A profile's every point and direction turned through `angle` about the line through `o` along
/// the unit `k`, by what its keys say they are.
fn profile_turned(j: &mut Json,o: V,k: V,angle: f64) {
    match j {
        Json::Arr(list) => for x in list { profile_turned(x,o,k,angle) },
        Json::Obj(pairs) => for (key,x) in pairs.iter_mut() {
            match key.as_str() {
                "start" | "end" | "center" | "origin" => *x = json_of(add(o,turned(sub(vector_of(x),o),k,angle))),
                "normal" | "x_dir" => *x = json_of(turned(vector_of(x),k,angle)),
                "poles" => for p in match x { Json::Arr(l) => l,_ => unreachable!() } {
                    *p = json_of(add(o,turned(sub(vector_of(p),o),k,angle)));
                },
                _ => profile_turned(x,o,k,angle),
            }
        },
        _ => {}
    }
}

/// A whole circle's start turned round it, `angle` about its normal.
fn circles_turned(j: &mut Json,angle: f64) {
    match j {
        Json::Arr(list) => for x in list { circles_turned(x,angle) },
        Json::Obj(_) => {
            if j.get("kind").is_some_and(|k| *k == Json::Str("circle".into())) && j.get("angles").is_none() {
                let (n,x) = (vector_of(j.get("normal").unwrap()),vector_of(j.get("x_dir").unwrap()));
                j.set("x_dir",json_of(turned(x,scale(n,1./norm(n)),angle)));
            }
            if let Json::Obj(pairs) = j { for (_,x) in pairs.iter_mut() { circles_turned(x,angle) } }
        }
        _ => {}
    }
}

/// The same solid with every seam turned off where it was: each whole circle's start a radian
/// round it, each whole revolution's profile a radian round its axis. OCCT's fillet fails, or
/// makes a face it cannot orient, where an edge it rounds ends on a seam or lies along it (a
/// rod's seam where a plate's top meets it, a frustum's along the plane that halves it), and the
/// mirror image of that edge, off the seam, it rounds: the oracle asks it again of the same solid
/// described from another start.
fn seams_turned(recipe: &Json) -> Json {
    let mut out = recipe.clone();
    let Some(Json::Arr(nodes)) = out.get("nodes").cloned() else { return out };
    let nodes = nodes.into_iter().map(|mut n| {
        let whole = n.get("kind").is_some_and(|k| *k == Json::Str("revolve".into()))
            && (n.get("angle").unwrap().as_f64().abs() - 2.*std::f64::consts::PI).abs() < 1e-9;
        if whole {
            let (o,k) = (vector_of(n.get("origin").unwrap()),vector_of(n.get("axis").unwrap()));
            let mut profile = n.get("profile").unwrap().clone();
            profile_turned(&mut profile,o,scale(k,1./norm(k)),1.);
            n.set("profile",profile);
        }
        if let Some(mut profile) = n.get("profile").cloned() {
            circles_turned(&mut profile,1.);
            n.set("profile",profile);
        }
        n
    }).collect();
    out.set("nodes",Json::Arr(nodes));
    out
}

/// How near one the faces' distances, each over the other's leg, must sum on our flat: its fit's.
const FLAT_FIT: f64 = 1e-6;

/// Our flats along traced edges held to their meaning by OCCT's geometry: of each point of a
/// fitted face of ours (a grid of its parameters, off the fit's nodes), the distance from each of the two faces of OCCT's own
/// body (`body`, the stratum) that meet at the flat's edge, the second the one the first leg is
/// on. The number of points read, and the worst `|d_a / l_b + d_b / l_a − 1|`.
fn flats_by_occt(session: &native::Session,body: &Json,ours: &gcs_core::brep::topo::Brep,flats: &[(V,[f64;2],V)])
    -> Result<(usize,f64),String> {
    let theirs = session.construct(body)?;
    let faces = session.faces(theirs)?;
    let distance = |face: std::os::raw::c_int,points: &[V]| -> Result<Vec<f64>,String> {
        Ok(session.surface_feet_near(face,points,&vec![[f64::NAN;2];points.len()],0.)?.into_iter()
            .map(|f| f.map_or(f64::INFINITY,|(_,d)| d)).collect())
    };
    // the fitted faces of ours, read on a grid of their parameters off every node of the fit
    let mut points: Vec<V> = Vec::new();
    for face in &ours.faces {
        let gcs_core::brep::geom::Surface::BSpline(_,net) = &face.surface else { continue };
        let [[u0,u1],[v0,v1]] = net.domain();
        let n = 64;
        for i in 0..n {
            for j in 0..n {
                let at = |k: usize,[a,b]: [f64;2]| a+(b-a)*(k as f64+0.371)/n as f64;
                points.push(face.surface.point([at(i,[u0,u1]),at(j,[v0,v1])]));
            }
        }
    }
    if points.is_empty() { return Err("no fitted face of ours to hold to OCCT".into()) }
    let mut worst: f64 = 0.;
    for &(at,[la,lb],on) in flats {
        // the two faces meeting at the edge, the first the one `on` stands beside
        let mut meeting = Vec::new();
        for &f in &faces {
            if distance(f,&[at])?[0] <= 1e-6*(1.+norm(at)) { meeting.push((distance(f,&[on])?[0],f)); }
        }
        meeting.sort_by(|a,b| a.0.total_cmp(&b.0));
        let [(_,a),(_,b)] = meeting[..] else { return Err(format!("{} of OCCT's faces meet at {at:?}",meeting.len())) };
        let (da,db) = (distance(a,&points)?,distance(b,&points)?);
        // (of the points of this flat: those within its legs of both faces)
        let within: Vec<(f64,f64)> = da.iter().zip(&db).map(|(x,y)| (*x,*y))
            .filter(|(x,y)| *x <= lb*(1.+1e-6) && *y <= la*(1.+1e-6)).collect();
        // (and of those, enough off its legs to read the fit between them)
        let inner = within.iter().filter(|(x,y)| x/lb > 0.01 && y/la > 0.01).count();
        if inner < 100 { return Err(format!("only {inner} points of our flat stand between its legs")) }
        for (x,y) in within {
            worst = worst.max((x/lb+y/la-1.).abs());
        }
    }
    if worst > FLAT_FIT { return Err(format!("OCCT's distances of our flat sum to one only within {worst:e}")) }
    Ok((points.len(),worst))
}

/// OCCT's `GeomAbs_SurfaceType` as this kernel names its surfaces.
fn occt_kind(k: i32) -> &'static str {
    ["plane","cylinder","cone","sphere","torus","bezier","bspline","revolution","extrusion","offset","other"]
        .get(k as usize).copied().unwrap_or("other")
}

/// Where two solids disagree, sampled at `count` points over ours's box: the points inside one
/// and outside the other, and whose.
fn apart(session: &native::Session,ours: &gcs_core::brep::topo::Brep,theirs: std::os::raw::c_int,count: usize)
    -> Vec<(V,&'static str)> {
    use gcs_core::brep::query::{Located,Place};
    let (lo,hi) = ours.bounds();
    let located = Located::owned(ours.clone(),1e-9*ours.size());
    let mut rng = gcs_core::rng::Rng::new(218);
    let points: Vec<V> = (0..count).map(|_| std::array::from_fn(|j| rng.uniform(lo[j],hi[j]))).collect();
    let states = session.solid_contains(theirs,&points,1e-9).unwrap();
    points.iter().zip(&states).filter_map(|(p,s)| match located.solid_place(*p) {
        Place::In if *s == 0 => Some((*p,"ours")),Place::Out if *s == 1 => Some((*p,"OCCT's")),_ => None,
    }).collect()
}


/// How OCCT builds what an example rounds.
#[derive(Clone,Copy)]
enum How {
    /// OCCT's fillet or chamfer of the same edges.
    Fillet,
    /// A convex body rounded all over by one radius: OCCT's offsets, the body shrunk by it and
    /// grown by it with round joins (`Session::opened`). OCCT's fillet fills a corner of three
    /// rounds beside a rolled one by a surface of its own; its offsets make the ball's.
    Opened(f64),
    /// Where OCCT's chamfer is another surface (beside a curved face it is ruled between legs along
    /// the faces): what the flat adds, lofted by OCCT through std's flat's own sections, worked out
    /// in closed form, and OCCT's body less the rounds beside it.
    Lofted(fn(&native::Session) -> Result<(f64,BTreeSet<&'static str>),String>),
}

/// The tee's crotch chamfered (`crotch.sv`): its main pipe of 10 about x, its branch of 6 about z
/// standing on it, the flat of legs 1 where the walls' distances sum to 1, i.e. `ρₓ + ρ_z = 17`.
/// In each half-plane through the branch's axis at `φ` (a distance `s` from it), what the flat adds
/// lies between the main wall `z = √(10² − s² sin²φ)` and the flat `z = √((17 − s)² − s² sin²φ)`,
/// from the branch's wall `s = 6` to where the two meet at `s = 7`: lofted by OCCT through those
/// sections over a quarter turn, the tee being its own mirror both ways.
fn crotch_flat(session: &native::Session) -> Result<(f64,BTreeSet<&'static str>),String> {
    let (big,small,leg) = (10.,6.,1.);
    let c = big+small+leg;
    let (n,p) = (33,33);
    let sections: Vec<Vec<Vec<V>>> = (0..n).map(|i| {
        let (sn,cs) = (std::f64::consts::FRAC_PI_2*i as f64/(n-1) as f64).sin_cos();
        let at = |s: f64,z: f64| [s*cs,s*sn,z];
        let main = |s: f64| (big*big-s*s*sn*sn).sqrt();
        let flat = |s: f64| ((c-s)*(c-s)-s*s*sn*sn).sqrt();
        let along = |k: usize| small+leg*k as f64/(p-1) as f64;
        vec![(0..p).map(|k| at(along(k),main(along(k)))).collect(),
            (0..p).map(|k| at(along(p-1-k),flat(along(p-1-k)))).collect(),
            (0..p).map(|k| at(small,flat(small)+(main(small)-flat(small))*k as f64/(p-1) as f64)).collect()]
    }).collect();
    let id = session.loft(&sections)?;
    session.validate(id)?;
    let kinds = session.faces(id)?.into_iter().map(|f| session.face_kind(f).map(occt_kind)).collect::<Result<_,_>>()?;
    Ok((4.*session.volume(id)?,kinds))
}

#[test]
fn every_rounds_example_is_occts_fillet() {
    let session = native::Session::new().unwrap();
    // (each with an edit of its source, where it is asked of part of what the example rounds)
    let generators = Some(("round(r: r) half\n","half.cutter round(r: r) half.cone.bc\n"));
    use How::*;
    let examples = [("block","block",None,Fillet),("by_hand","block",None,Fillet),("pocket","dish",None,Fillet),
        ("boss","pad",None,Fillet),("slot","part",None,Fillet),("pad","pad",None,Fillet),("rail","rail",None,Fillet),
        ("knob","knob",None,Fillet),("tee","tee",None,Fillet),("bore","pipe",None,Fillet),("runout","lug",None,Fillet),
        ("runout","vent",None,Fillet),("twin","bar",None,Fillet),("post","stand",None,Fillet),("dshaft","shaft",None,Fillet),
        ("groove","part",None,Fillet),("bores","slab",None,Fillet),("frustum","half",None,Opened(1.)),
        ("frustum","half",generators,Fillet),("crotch","tee",None,Lofted(crotch_flat)),("bullnose","part",None,Fillet)];
    let mut failures = Vec::new();
    for (file,name,edit,how) in examples {
        let text = match edit {
            None => source(file),
            Some((from,to)) => { let s = source(file); assert!(s.contains(from),"{file}: {from}"); s.replace(from,to) }
        };
        let file = if edit.is_some() { format!("{file} (in part)") } else { file.to_string() };
        let (prog,errs,linked) = gcs_core::library::parse_linked(&text);
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
        let mut beside_curved = false;
        // the flats rolled along a traced edge: (a point of the edge, the legs, a point beside the
        // face the first leg is on)
        let mut traced_flats: Vec<(V,[f64;2],V)> = Vec::new();
        // (of the regions over this body's faces)
        let over = |i: usize| matches!(&sk.solids[i].def,SolidDef::Region {terms,..} if terms.iter()
            .any(|t| matches!(&t.shape,RegionShape::FaceSide {face} if face.solid as usize == body)));
        for i in (0..sk.solids.len()).filter(|&i| over(i)) {
            let Some(Ok(pieces)) = lowering::lowered(sk,i) else { continue };
            beside_curved |= pieces.iter().filter_map(edge_and_kind).any(|(_,_,_,curved)| curved);
            traced_flats.extend(pieces.iter().filter_map(|p| match p { Piece::Rolled(k) => k.flat.map(|(legs,on)| (k.at,legs,on)),_ => None }));
            for (at,size,chamfer) in pieces.iter().filter_map(edge_of) {
                let list = if chamfer { &mut flats } else { &mut rounds };
                if !list.iter().any(|(p,_)| dist(*p,at) <= 1e-9*(1.+norm(at))) { list.push((at,size)); }
            }
        }
        let recipe = gcs_core::solid::cad::recipe(sk,body).unwrap();
        // OCCT's fillet fails on an edge along or ending on a seam: asked again with them turned
        let turning = |build: &dyn Fn(&Json) -> Result<std::os::raw::c_int,String>| build(&stratum(&recipe)).or_else(|first| {
            let turned = build(&seams_turned(&stratum(&recipe)));
            if turned.is_ok() { eprintln!("{file} `{name}`: OCCT builds it with its seams turned ({first})"); }
            turned
        });
        let valid = |id: std::os::raw::c_int| session.validate(id).map(|_| id).map_err(|e| format!("no valid solid: {e}"));
        let kinds_of = |id: std::os::raw::c_int| -> BTreeSet<&'static str> {
            session.faces(id).unwrap().into_iter().map(|f| occt_kind(session.face_kind(f).unwrap())).collect()
        };
        // theirs: its volume and the kinds of its faces, and the solid where OCCT made one whole
        let theirs: Result<(f64,BTreeSet<&str>,Option<std::os::raw::c_int>),String> = match how {
            Fillet if beside_curved => Err("a chamfer beside a curved face is std's flat, not OCCT's chamfer".into()),
            Fillet => turning(&|recipe| session.construct(recipe).and_then(|mut id| {
                if !rounds.is_empty() { id = session.round(id,&rounds,false)?; }
                if !flats.is_empty() { id = session.round(id,&flats,true)?; }
                valid(id)
            })).map(|id| (session.volume(id).unwrap(),kinds_of(id),Some(id))),
            // (offset, an edge along a seam is misread whole and the solid valid: so never asked so)
            Opened(r) => session.construct(&seams_turned(&stratum(&recipe))).and_then(|id| valid(session.opened(id,r)?))
                .map(|id| (session.volume(id).unwrap(),kinds_of(id),Some(id))),
            Lofted(added) => session.construct(&stratum(&recipe)).and_then(|id| {
                let (v,kinds) = added(&session)?;
                let sharp = session.volume(id)?;
                // what the flat adds, alone
                let rel = ((ours-sharp)-v).abs()/v;
                eprintln!("{file} `{name}`: the flat adds {:.9} mm³ against OCCT's loft's {v:.9} ({rel:.1e})",ours-sharp);
                if rel > APPROXIMATE { return Err(format!("the flat adds {} against OCCT's {v} ({rel:.1e})",ours-sharp)) }
                // and of ours, OCCT's distances from its walls
                let (points,worst) = flats_by_occt(&session,&stratum(&recipe),&exact.solid,&traced_flats)?;
                eprintln!("{file} `{name}`: OCCT's distances of {points} points of our flat sum to one within {worst:.1e}");
                Ok((sharp+v,kinds_of(id).union(&kinds).copied().collect(),None))
            }),
        };
        let (w,their_kinds,solid) = match theirs {
            Ok(t) => t,
            Err(err) => { failures.push(format!("{file} `{name}`: OCCT does not build it: {err}")); continue }
        };
        let rel = (ours-w).abs()/w.abs();
        eprintln!("{file} `{name}`: {} rounds, {} flats; {ours:.9} mm³ against OCCT's {w:.9} ({rel:.1e}); faces {our_kinds:?} against \
            {their_kinds:?}",rounds.len(),flats.len());
        let fitted = our_kinds.contains("bspline") || their_kinds.contains("bspline");
        if rel > if fitted { APPROXIMATE } else { EXACT } {
            let apart = solid.map(|id| apart(&session,&exact.solid,id,20_000)).unwrap_or_default();
            failures.push(format!("{file} `{name}`: volume {ours} against OCCT's {w} ({rel:.1e}); {} of 20000 points apart, as {:?}",
                apart.len(),&apart[..apart.len().min(4)]));
        }
        // every kind of ours OCCT's, and OCCT's the same but for a B-spline it fits where this kernel
        // builds a face exactly (a round between two cylinders, the knob's neck)
        let alike = our_kinds.is_subset(&their_kinds) && their_kinds.iter().all(|k| *k == "bspline" || our_kinds.contains(k));
        if !alike { failures.push(format!("{file} `{name}`: faces {our_kinds:?} against OCCT's {their_kinds:?}")); }
    }
    eprintln!("{} compared",examples.len());
    assert!(failures.is_empty(),"{} disagreements:\n{}",failures.len(),failures.join("\n"));
}
