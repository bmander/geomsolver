//! The generating-system checks: the pair's members built at one tooth space each (`gears.sv`),
//! each face the space leaves held to the analytic envelope its name says (#212) — the crown face
//! that made it, under its member's generating roll — with independent contact and indexing checks.
use super::*;
use gcs_core::{diagnose,modules,program,solid::RevolvedSurface,solve,space};
use std::f64::consts::{FRAC_PI_2,PI,TAU};

mod boundary;
mod analytic;
mod intervals;
mod faces;
mod caps;
mod swept;

/// The layout's names in the pair: `HypoidPair` keeps it under its private `reference`.
const REF: &str = "pair.reference";

/// The checks' roll: the envelopes are verified over ±35 degrees; each member's removal sweep
/// (members.sv) rolls at least that far.
const ROLL_SPAN: f64 = 35.;

struct Pair {
    model: program::Elaborated,
    teeth: [f64;2],
    module: f64,
    delta: [f64;2],
    rm: f64,
    offset: [f64;3],
    motion_families: [gcs_core::motion::Family;2],
    /// Each member's blank's faces by their terms (`tip`, `heel`, `toe`, `back`), read off the
    /// built member's faces (`material.tip`), and the blank's region solid.
    walls: [std::collections::BTreeMap<String,RevolvedSurface>;2],
    blanks: [usize;2],
    regions: std::collections::BTreeMap<String,gcs_core::patch::TrimmedPatch>,
    envelopes: std::collections::BTreeMap<String,envelope::GeneratedEnvelope>,
    seams: std::collections::BTreeMap<String,gcs_core::seam::EnvelopeSeam>,
    boundary_seams: std::collections::BTreeMap<String,gcs_core::seam::BoundarySeam>,
    surface_seams: std::collections::BTreeMap<String,gcs_core::seam::SurfaceSeam>,
    corners: std::collections::BTreeMap<String,gcs_core::vertex::SolvedVertex>,
    edges: std::collections::BTreeMap<String,gcs_core::edge::SpatialEdge>,
    faces: std::collections::BTreeMap<String,gcs_core::spatial_face::SpatialFaceBoundary>,
    /// Each cutter face the checks read, by what they call it (`faces.pinion_outer`,
    /// `faces.gear_inner_tip`): its surface (named so), the region it is a face of, and its arc's
    /// sweep in the region's meridian where it is a round (`cutter_faces`).
    surfaces: std::collections::BTreeMap<String,(RevolvedSurface,usize,Option<f64>)>,
}

fn read_model(src: &str,teeth: [u32;2],module: f64) -> program::Elaborated { read_model_with(src,teeth,module,false) }

/// `read_model`, each member cut at its first tooth space only where `one_space` says: the faces
/// a space leaves, every space's alike.
fn read_model_with(src: &str,teeth: [u32;2],module: f64,one_space: bool) -> program::Elaborated {
    let (mut p,errors) = crate::common::parse(&src);
    let link = modules::link(&mut p,&mut |name| match name {
        // The reference checks assume the common apex, so the offset is zero here.
        "configuration" => Some(fixtures::gear::configuration(teeth,module,0.,0.,35.)),
        "members" if one_space => fixtures::gear::module(name).map(|text| fixtures::gear::one_space(&text)),
        _ => fixtures::gear::module(name),
    });
    assert!(errors.is_empty() && link.is_empty(),"{errors:?} {link:?}");
    let mut model = program::elaborate(&p);
    assert!(model.ok(),"{:?}",model.diags);
    for i in 0..model.sketch.points.len() {
        for p in model.sketch.point_params(i) {
            if !model.sketch.params[p as usize].fixed {
                model.sketch.params[p as usize].value += module*0.001*(i as f64).sin();
            }
        }
    }
    // Shared analytic boundaries need tighter accuracy than interactive editing.
    // The solver owns method selection and retry; geometry still checks its own
    // position, normal and incidence errors independently of equation residuals.
    let solved = solve::solve(&mut model.sketch,solve::SolveOpts {
        tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(solved.success,
        "teeth {teeth:?}, module {module}: {solved:?}");
    assert_eq!(diagnose::diagnose(&mut model.sketch,Default::default()).dof,0);
    model
}

/// What the checks read off a member as built (`BodyFaces`): each face's name, which faces meet
/// along an edge, and which meet at each vertex. The design is proportional to its module
/// (design.sv: every depth in modules, every length in the cone distance; no offset or allowance
/// here), so a member's faces and how they meet are the same at every module for its tooth
/// counts: they are read off the member built at `BUILT_MODULE`, where this kernel fits the cut's
/// sheet (its rows are spaced in millimetres, and a tenth of that tooth is a few rows deep), and
/// remembered by tooth counts, since the suite asks the same designs many times.
#[derive(Clone)]
struct Built { paths: Vec<gcs_core::solid::faces::FacePath>,meets: std::collections::BTreeSet<(usize,usize)>,corners: Vec<Vec<usize>> }

/// The module the members' faces are read off at (`Built`).
const BUILT_MODULE: f64 = 2.;

impl Built {
    /// Member `member`'s faces, both members built once per tooth counts, whichever test asks
    /// first (the others wait for it).
    fn read(member: &str,teeth: [u32;2]) -> Built {
        use std::{collections::BTreeMap,sync::{Arc,Mutex,OnceLock}};
        static BUILT: Mutex<BTreeMap<[u32;2],Arc<OnceLock<[Built;2]>>>> = Mutex::new(BTreeMap::new());
        let cell = BUILT.lock().unwrap().entry(teeth).or_default().clone();
        let both = cell.get_or_init(|| {
            let model = read_model_with(include_str!("../../../examples/spiral_bevel/gears.sv"),teeth,BUILT_MODULE,true);
            ["pinion","gear"].map(|member| Built::of(&model,member,teeth))
        });
        both[usize::from(member == "gear")].clone()
    }

    fn of(model: &program::Elaborated,member: &str,teeth: [u32;2]) -> Built {
        let body = model.map.ent_named(&format!("pair.{member}.body")).unwrap().i();
        let faces = gcs_core::solid::faces::BodyFaces::read(&model.sketch,body)
            .unwrap_or_else(|why| panic!("teeth {teeth:?}: {why}"));
        let n = faces.paths().len();
        let meets = (0..n).flat_map(|a| (a+1..n).map(move |b| (a,b))).filter(|&(a,b)| !faces.edges_between(&[a],&[b]).is_empty()).collect();
        let corners = (0..faces.brep().0.vertices.len() as u32).map(|v| faces.faces_at(v,&[])).collect();
        Built {paths:faces.paths().to_vec(),meets,corners}
    }
}

/// What a face of a built member is to the checks: a face its cut left, by what they call its
/// cutter face, or one of its blank's, by its term.
#[derive(Clone,Debug,PartialEq)]
enum Role { Cut(String,usize),Wall(String) }

/// A face of a region cutter as the checks read it: its name's last segment, its surface, and
/// the sweep of its meridian's arc where it is a round.
type CutterFace = (String,RevolvedSurface,Option<f64>);

/// A region cutter's faces (`RevolvedSurface::meridian_loops`; a step on the axis `None`), walked
/// counterclockwise in (radius, height) about the cutter's axis (`cutter.ax`, up from C) — the
/// crown tooth's base, outer flank, round, tip, round and inner flank; its mate's base, inner
/// flank, round, tip and so on — each face's `u` along the walk, as the drawn sections' were; and
/// turned about the axis so each chart starts where the section stood in the normal view, on the
/// axis's far side from the mean point, as a drawn section's revolution's did.
fn cutter_faces(model: &program::Elaborated,solid: usize) -> Vec<Option<CutterFace>> {
    use gcs_core::brep::planar::Seg;
    let sk = &model.sketch;
    let ax = &sk.lines[model.map.ent_named(&format!("{REF}.cutter.ax")).unwrap().i()];
    let up = space::sub(sk.world_point(ax.p2 as usize),sk.world_point(ax.p1 as usize));
    let mean = sk.world_point(model.map.ent_named(&format!("{REF}.gear.M")).unwrap().i());
    let m = gcs_core::solid::region::meridian(sk,solid).unwrap();
    let surfaces = RevolvedSurface::meridian_loops(&m,&sk.solids[solid].name);
    let ([steps],[faces]) = (m.region.loops.as_slice(),surfaces.as_slice()) else { panic!("a cutter is one loop") };
    // (radius, height up the axis) of a meridian point, its height signed by the cutter's axis
    let sense = space::dot(m.axis,up).signum();
    let rh = |q: [f64;2]| [q[0],sense*q[1]];
    let area: f64 = steps.iter().map(|s| { let (a,b) = (rh(s.seg.start()),rh(s.seg.end())); a[0]*b[1]-a[1]*b[0] }).sum();
    let mut walk: Vec<(Seg,Option<RevolvedSurface>)> = steps.iter().map(|s| s.seg).zip(faces.iter().cloned()).collect();
    if area < 0. { walk.reverse(); }
    let basis = gcs_core::plane::Basis {u: m.seam,v: m.axis,o: m.origin};
    // the turn about the axis from the meridian's seam to the far side from the mean point
    let axis = space::scale(m.axis,1./space::norm(m.axis));
    let off = space::sub(mean,m.origin);
    let away = space::sub(space::scale(axis,space::dot(off,axis)),off);
    let angle = space::dot(space::cross(m.seam,away),axis).atan2(space::dot(m.seam,away));
    let pose = Motion::translation(space::scale(m.origin,-1.),[0.;3]).unwrap()
        .then(Motion::rotation(axis,angle,0.).unwrap()).then(Motion::translation(m.origin,[0.;3]).unwrap());
    walk.into_iter().map(|(seg,surface)| {
        let surface = surface?;
        let first = if area < 0. { seg.end() } else { seg.start() };
        let start = basis.lift(first[0],first[1]);
        let ends = [0.,1.].map(|u| space::distance(surface.at(u,0.).unwrap().position,start));
        let surface = if ends[0] <= ends[1] { surface } else { surface.reversed() };
        let sweep = match seg { Seg::Arc {sweep,..} => Some(sweep.abs()),_ => None };
        let name = surface.name.rsplit('.').next().unwrap().to_string();
        Some((name,surface.placed(pose),sweep))
    }).collect()
}

impl Pair {
    // Only explicit CAD export entry points consume these overrides; ordinary
    // regression tests keep their declared configurations.
    fn read_cad_export() -> Self {
        let integer = |name,default: u32| std::env::var(name).map(|s|
            s.parse::<u32>().expect("CAD tooth count must be an integer")).unwrap_or(default);
        let teeth = [integer("SOLVENT_CAD_PINION_TEETH",24),integer("SOLVENT_CAD_GEAR_TEETH",48)];
        let module = std::env::var("SOLVENT_CAD_MODULE_MM").map(|s|
            s.parse::<f64>().expect("CAD module must be a number")).unwrap_or(2.);
        assert!(teeth.iter().all(|&n| n > 0) && module.is_finite() && module > 0.,
            "CAD tooth counts and module must be positive and finite");
        Self::read(teeth,module)
    }

    /// The pair at `teeth` and `module`, each member built at one tooth space (`gears.sv`), and
    /// what the checks read off it: each face the space leaves (`BodyFaces`), held to the analytic
    /// surface its name says (`face_source`) — every seam, corner, edge and face boundary between
    /// them made of those analytic parts, never of the kernel's fit of the faces.
    fn read(teeth: [u32;2], module: f64) -> Self {
        use gcs_core::solid::faces::{face_source,FaceSource};
        use gcs_core::seam::{BoundarySeam,EnvelopeSeam,SeamTolerance,SurfaceSeam};
        use gcs_core::vertex::{BoundaryVertex,JunctionVertex};
        use gcs_core::edge::{EdgePoint,EdgeTolerance,SpatialEdge};
        use gcs_core::spatial_face::{SpatialFaceBoundary,Support,Use};
        use std::collections::BTreeMap;
        let src = include_str!("../../../examples/spiral_bevel/gears.sv");
        let model = read_model_with(src,teeth,module,true);
        let sk = &model.sketch;
        let teeth_f = teeth.map(|n| n as f64);
        let delta = [teeth_f[0].atan2(teeth_f[1]),teeth_f[1].atan2(teeth_f[0])];
        let rm = module*teeth_f[0].hypot(teeth_f[1])/2.;
        let theta = (35f64-90.).to_radians();
        let offset = [rm-0.8*rm*theta.cos(),-0.8*rm*theta.sin(),0.];
        let motion_families = ["pinion_generation","gear_generation"].map(|name|
            gcs_core::motion::Family::read(sk,model.map.ent_named(&format!("{REF}.generation.{name}")).unwrap().i()).unwrap());
        let tolerance = SeamTolerance {position:module*1e-8,normal:1e-8,axis:module*1e-10};
        let edge_tolerance = EdgeTolerance {junction:tolerance,point:gcs_core::seam::BoundarySeamTolerance {
            normal_velocity:module*1e-10,incidence:module*1e-10,trim:module*1e-8}};
        let roll = ROLL_SPAN.to_radians();
        let mut walls: [BTreeMap<String,RevolvedSurface>;2] = Default::default();
        let mut blanks = [0;2];
        let (mut regions,mut envelopes,mut surfaces) = (BTreeMap::new(),BTreeMap::new(),BTreeMap::new());
        let (mut seams,mut boundary_seams,mut surface_seams) = (BTreeMap::new(),BTreeMap::new(),BTreeMap::new());
        let (mut corners,mut edges,mut faces) = (BTreeMap::new(),BTreeMap::new(),BTreeMap::new());
        for (m,member) in ["pinion","gear"].into_iter().enumerate() {
            let body = model.map.ent_named(&format!("pair.{member}.body")).unwrap().i();
            let built = Built::read(member,teeth);
            // each built face's role: a cut face by what the checks call its cutter face, a blank
            // face by its term
            let mut roles = Vec::new();
            for path in &built.paths {
                let source = face_source(sk,body,path).unwrap_or_else(|why| panic!("`{}`: {why}",path.text()));
                roles.push(match source {
                    FaceSource::Revolved {surface,solid} => {
                        blanks[m] = solid;
                        walls[m].insert(path.face.clone(),surface.clone());
                        Role::Wall(path.face.clone())
                    }
                    FaceSource::Swept {solid,roll: swept,..} => {
                        assert!(swept[0] <= -roll && swept[1] >= roll,"{member}: the removal rolls {swept:?}, under ±{ROLL_SPAN} degrees");
                        // the checks' names: the crown tooth's section for the pinion, the gear's
                        // outer and inner mates' (the space cutter's own crown and its neighbour)
                        let rack = if m == 0 { "pinion" } else if path.face.starts_with("outer_crown.") { "gear_outer" } else { "gear_inner" };
                        let face = path.face.rsplit('.').next().unwrap();
                        Role::Cut(format!("faces.{rack}_{face}"),solid)
                    }
                });
            }
            // each side: its flank (a cut face meeting the blank's tip), its round (the cut face the
            // flank meets) and its root (the cutter's face its round runs on to, read off the
            // cutter's profile: the round's neighbour there), each read on the cutter's revolution
            // where the cutter's own placements put it before they turn it — the gear's inner mate
            // is its space cutter's neighbour, one crown pitch round, so its faces are the envelopes
            // of the space one tooth over, by the symmetry the indexing turns through
            let fi_of = |r: &Role| (0..roles.len()).filter(|&i| roles[i] == *r).collect::<Vec<_>>();
            let meets = |a: &[usize],b: &[usize]| a.iter().any(|&x| b.iter().any(|&y| built.meets.contains(&(x.min(y),x.max(y)))));
            let tips = fi_of(&Role::Wall("tip".into()));
            let cut: Vec<(usize,String,usize)> = roles.iter().enumerate().filter_map(|(i,r)| match r {
                Role::Cut(label,solid) => Some((i,label.clone(),*solid)),
                _ => None,
            }).collect();
            // each cutter's faces, its region's meridian's steps in their walk (`cutter_faces`)
            let cutters: std::collections::BTreeMap<usize,Vec<Option<CutterFace>>> = cut.iter()
                .map(|&(_,_,solid)| (solid,cutter_faces(&model,solid))).collect();
            let surface_of = |solid: usize,name: &str,label: &str| -> (RevolvedSurface,usize,Option<f64>) {
                let faces = &cutters[&solid];
                let k = faces.iter().position(|f| f.as_ref().is_some_and(|(n,..)| n == name))
                    .unwrap_or_else(|| panic!("`{}` has no `{name}`",sk.solids[solid].name));
                let (_,s,sweep) = faces[k].as_ref().unwrap();
                let mut s = s.spanned([90f64.to_radians(),270f64.to_radians()]).unwrap();
                s.name = label.to_string();
                (s,k,*sweep)
            };
            let profile_name = |solid: usize,k: usize| -> String {
                cutters[&solid][k].as_ref().map(|f| f.0.clone()).unwrap_or_default()
            };
            let neighbours = |solid: usize,k: usize| -> Vec<(usize,usize,usize)> {
                // the faces either side of step `k` in the meridian, each with which end of
                // step `k` and which of its own (its `u`, 0 or 1) is the corner they share
                let faces = &cutters[&solid];
                let ends = |f: &RevolvedSurface| [0.,1.].map(|u| f.at(u,0.).unwrap().position);
                let this = ends(&faces[k].as_ref().unwrap().1);
                let n = faces.len();
                [(k+n-1)%n,(k+1)%n].into_iter().filter_map(|j| {
                    let that = ends(&faces[j].as_ref()?.1);
                    let (_,a,b) = (0..2).flat_map(|a| (0..2).map(move |b| (a,b)))
                        .map(|(a,b)| (space::distance(this[a],that[b]),a,b))
                        .min_by(|x,y| x.0.total_cmp(&y.0)).unwrap();
                    Some((j,a,b))
                }).collect()
            };
            let blank = gcs_core::solid::RevolvedRegion::read(sk,blanks[m],module*1e-10).unwrap();
            let mut corner_ids: BTreeMap<String,usize> = BTreeMap::new();
            let mut root_face = None;
            for (side,flank_name) in (if m == 0 { ["outer","inner"] } else { ["inner","outer"] }).into_iter().enumerate() {
                let rack = if m == 0 { "pinion" } else { ["gear_outer","gear_inner"][side] };
                let label = format!("faces.{rack}_{flank_name}");
                let &(flank_fi,_,solid) = cut.iter().find(|c| c.1 == label).unwrap_or_else(|| panic!("{member}: no `{label}` among {roles:?}"));
                assert!(meets(&[flank_fi],&tips),"{member}: `{label}` meets no tip");
                let (flank,flank_edge,flank_sweep) = surface_of(solid,flank_name,&label);
                // the round: the flank's neighbour in the profile that is a face of the cut it meets
                let round_label = format!("faces.{rack}_{flank_name}_round");
                let &(round_fi,..) = cut.iter().find(|c| c.1 == round_label).unwrap();
                assert!(meets(&[flank_fi],&[round_fi]),"{member}: `{label}` meets no round");
                let (round_edge,flank_end,round_end) = neighbours(solid,flank_edge).into_iter()
                    .find(|(e,..)| profile_name(solid,*e) == format!("{flank_name}_round")).unwrap();
                let (round,_,round_sweep) = surface_of(solid,&format!("{flank_name}_round"),&round_label);
                // the root: the round's other neighbour in the profile, whose face the space's
                // bottom is (the gear's two mates' tips lie in one plane, the space's one face)
                let (root_edge,round_root_end,root_end) = neighbours(solid,round_edge).into_iter()
                    .find(|(e,..)| *e != flank_edge).unwrap();
                let root_label = format!("faces.{rack}_tip");
                let (root,_,root_sweep) = surface_of(solid,&profile_name(solid,root_edge),&root_label);
                let bottom: Vec<usize> = cut.iter().filter(|c| c.0 != flank_fi && c.0 != round_fi && meets(&[c.0],&[round_fi]))
                    .map(|c| c.0).collect();
                assert_eq!(bottom.len(),1,"{member}: `{round_label}` meets {bottom:?}");
                root_face.get_or_insert(bottom[0]);
                assert_eq!(root_face,Some(bottom[0]),"{member}: the space has one bottom");
                for (l,s,e) in [(&label,&flank,flank_sweep),(&round_label,&round,round_sweep),(&root_label,&root,root_sweep)] {
                    let env = envelope::GeneratedEnvelope::new(format!("{l}_envelope"),s.clone(),motion_families[m].clone(),[-roll,roll]).unwrap();
                    regions.insert(l.clone(),gcs_core::patch::TrimmedPatch::of_envelope(format!("{l}_region"),env.clone(),vec![blank.clone()],vec![]));
                    envelopes.insert(l.clone(),env);
                    surfaces.insert(l.clone(),(s.clone(),solid,e));
                }
                let join = EnvelopeSeam::new(format!("{label}.join"),[&regions[&label],&regions[&round_label]],
                    [flank_end as f64,round_end as f64],tolerance).unwrap();
                let root_join = EnvelopeSeam::new(format!("{label}.root_join"),[&regions[&round_label],&regions[&root_label]],
                    [round_root_end as f64,root_end as f64],tolerance).unwrap();
                // the boundary seams: each the B-rep says the face meets the blank's face along
                let wall = |w: &str| walls[m][w].projector().unwrap();
                let mut boundary = BTreeMap::new();
                for (face,fi,prefix) in [(&label,flank_fi,""),(&round_label,round_fi,"round_")] {
                    for w in ["tip","toe","heel","back"] {
                        if !meets(&[fi],&fi_of(&Role::Wall(w.into()))) { continue }
                        let s = BoundarySeam::new(format!("{label}.{prefix}{w}_edge"),&regions[face],wall(w)).unwrap();
                        boundary.insert(format!("{prefix}{w}"),s);
                    }
                }
                assert_eq!(boundary.keys().map(String::as_str).collect::<Vec<_>>(),["heel","round_heel","round_toe","tip","toe"],
                    "{member}: `{label}` meets the blank's faces");
                // its corners, where three of the faces meet in the B-rep: solved on the analytic ones
                let around = |a: usize,b: &[usize],c: &[usize]| -> bool {
                    built.corners.iter().any(|f| f.contains(&a) && b.iter().any(|x| f.contains(x)) && c.iter().any(|x| f.contains(x)))
                };
                let tol_options = |bounds: [[f64;2];3]| gcs_core::envelope::IntersectionOptions {
                    bounds,parameter_scale:[1.;3],residual_tolerance:[module*1e-10;3],max_iterations:100};
                let mut corner = |name: &str,solved: gcs_core::vertex::SolvedVertex| {
                    corner_ids.insert(format!("{label}.{name}"),corner_ids.len());
                    corners.insert(format!("{label}.{name}"),solved);
                };
                for end in ["toe","heel"] {
                    let ends = fi_of(&Role::Wall(end.into()));
                    assert!(around(flank_fi,&tips,&ends),"{member}: `{label}` has no tip {end} corner");
                    let v = BoundaryVertex::new(format!("{label}.tip_{end}"),&regions[&label],[wall("tip"),wall(end)]).unwrap();
                    let bounds = v.domain();
                    corner(&format!("tip_{end}"),v.solve(bounds.map(|b| (b[0]+b[1])/2.),tol_options(bounds),module*1e-8)
                        .unwrap_or_else(|e| panic!("{label}.tip_{end}, module {module}: {e:?}")));
                    for (name,seam,a,b) in [("join",&join,flank_fi,round_fi),("root",&root_join,round_fi,bottom[0])] {
                        assert!(around(a,&[b],&ends),"{member}: `{label}` has no {name} {end} corner");
                        let v = JunctionVertex::new(format!("{label}.{name}_{end}"),seam.clone(),wall(end),0).unwrap();
                        let [_,vb,tb] = v.domain();
                        corner(&format!("{name}_{end}"),v.solve([vb,tb].map(|b| (b[0]+b[1])/2.),gcs_core::seam::SeamIntersectionOptions {
                            bounds:[vb,tb],parameter_scale:[1.;2],normal_tolerance:module*1e-10,section_tolerance:module*1e-10,
                            max_iterations:100},module*1e-8).unwrap_or_else(|e| panic!("{label}.{name}_{end}, module {module}: {e:?}")));
                    }
                }
                // its edges, between those corners along the seams, sliced along the member's axis
                let axis = &sk.lines[model.map.ent_named(&format!("{REF}.{member}.ax")).unwrap().i()];
                let along: [f64;3] = { let (a,b) = (sk.world_point(axis.p1 as usize),sk.world_point(axis.p2 as usize)); std::array::from_fn(|k| b[k]-a[k]) };
                let point = |name: &str,u: Option<f64>| {
                    let c = corners[&format!("{label}.{name}")];
                    let mut parameters = c.parameters;
                    if let Some(u) = u { parameters[0] = u; }
                    EdgePoint {position:c.position,parameters}
                };
                use gcs_core::edge::Source::{Boundary,Junction};
                let mut span = |name: &str,source: gcs_core::edge::Source,[a,b]: [(&str,Option<f64>);2]| {
                    let e = SpatialEdge::new(format!("{label}.{name}"),source,[corner_ids[&format!("{label}.{}",a.0)],corner_ids[&format!("{label}.{}",b.0)]],
                        [point(a.0,a.1),point(b.0,b.1)],along,edge_tolerance).unwrap_or_else(|e| panic!("{label}.{name}: {e}"));
                    let id = edges.len();
                    edges.insert(format!("{label}.{name}"),e.clone());
                    (e,id)
                };
                // a junction's corner read on its second face takes that face's end
                let on_round = Some(round_end as f64);
                let tip_span = span("tip_span",Boundary(boundary["tip"].clone()),[("tip_toe",None),("tip_heel",None)]);
                let join_span = span("join_span",Junction(join.clone()),[("join_toe",None),("join_heel",None)]);
                let root_span = span("root_span",Junction(root_join.clone()),[("root_toe",None),("root_heel",None)]);
                let toe_span = span("toe_span",Boundary(boundary["toe"].clone()),[("join_toe",None),("tip_toe",None)]);
                let heel_span = span("heel_span",Boundary(boundary["heel"].clone()),[("join_heel",None),("tip_heel",None)]);
                let round_toe = span("round_toe_span",Boundary(boundary["round_toe"].clone()),[("root_toe",None),("join_toe",on_round)]);
                let round_heel = span("round_heel_span",Boundary(boundary["round_heel"].clone()),[("root_heel",None),("join_heel",on_round)]);
                // its two faces, each loop closed on the corners' identities
                let looped = |edges: Vec<((SpatialEdge,usize),Option<f64>)>| -> Vec<Use> {
                    let mut uses: Vec<Use> = Vec::new();
                    let mut end = None;
                    for ((edge,id),endpoint_u) in edges {
                        let [a,b] = edge.vertices();
                        let forward = match end { None => true,Some(e) => a == e };
                        end = Some(if forward { b } else { a });
                        uses.push(Use {edge,id,direction:if forward { gcs_core::topology::Direction::Forward }
                            else { gcs_core::topology::Direction::Reverse },endpoint_u});
                    }
                    uses
                };
                let working = SpatialFaceBoundary::new(format!("{label}.working"),Support::Envelope(regions[&label].clone()),
                    looped(vec![(toe_span,None),(tip_span,None),(heel_span,None),(join_span.clone(),None)]),edge_tolerance)
                    .unwrap_or_else(|e| panic!("{label}.working: {e}"));
                let transition = SpatialFaceBoundary::new(format!("{label}.transition"),Support::Envelope(regions[&round_label].clone()),
                    looped(vec![(round_toe,None),(join_span,on_round),(round_heel,None),(root_span,None)]),edge_tolerance)
                    .unwrap_or_else(|e| panic!("{label}.transition: {e}"));
                faces.insert(working.name.clone(),working);
                faces.insert(transition.name.clone(),transition);
                for (k,s) in boundary { boundary_seams.insert(format!("{label}.{}_edge",k),s); }
                seams.insert(join.name.clone(),join);
                seams.insert(root_join.name.clone(),root_join);
            }
            // where the blank's faces meet one another: its end circles, each cone's with each sphere
            for (role,cone) in ["tip","back"].map(|w| (w,&walls[m][w])) {
                for (end,sphere) in [("near","toe"),("far","heel")] {
                    assert!(meets(&fi_of(&Role::Wall(role.into())),&fi_of(&Role::Wall(sphere.into()))),"{member}: {role} meets no {sphere}");
                    let name = format!("faces.{member}_ends.{role}.{end}");
                    surface_seams.insert(name.clone(),SurfaceSeam::new(name,cone.clone(),walls[m][sphere].projector().unwrap()));
                }
            }
        }
        Self {model,teeth:teeth_f,module,delta,rm,offset,motion_families,walls,blanks,regions,envelopes,seams,boundary_seams,
            surface_seams,corners,edges,faces,surfaces}
    }

    /// A member's blank face by its term (`tip`, `heel`, `toe`, `back`).
    fn wall(&self,member: usize,term: &str) -> &RevolvedSurface { &self.walls[member][term] }

    /// Where the generator of a blank's cone (its straight meridian at the chart's start, read past
    /// the face's own stretch between the end spheres) meets the sphere of radius `rho` about the
    /// apex: the one point near the face.
    fn cone_on_sphere(&self,member: usize,term: &str,rho: f64) -> [f64;3] {
        let s = self.wall(member,term);
        let v = s.domain()[1][0];
        let (a,b) = (s.at(0.,v).unwrap().position,s.at(1.,v).unwrap().position);
        let d: [f64;3] = std::array::from_fn(|k| b[k]-a[k]);
        let (dd,ad,aa) = ((0..3).map(|k| d[k]*d[k]).sum::<f64>(),(0..3).map(|k| a[k]*d[k]).sum::<f64>(),(0..3).map(|k| a[k]*a[k]).sum::<f64>());
        let disc = ad*ad-dd*(aa-rho*rho);
        assert!(disc >= 0.,"the {term} cone misses the sphere of radius {rho}");
        let roots: Vec<f64> = [(-ad-disc.sqrt())/dd,(-ad+disc.sqrt())/dd].into_iter().filter(|t| *t > -0.5 && *t < 1.5).collect();
        assert_eq!(roots.len(),1,"the {term} cone meets the sphere of radius {rho} once near its face");
        std::array::from_fn(|k| a[k]+roots[0]*d[k])
    }

    /// The revolution a cutter face the checks read is a face of.
    fn crown_of(&self,surface: &RevolvedSurface) -> usize { self.surfaces[&surface.name].1 }

    /// The sweep of the meridian's arc a round the checks read is the revolution of.
    fn round_sweep(&self,surface: &RevolvedSurface) -> f64 {
        self.surfaces[&surface.name].2.unwrap_or_else(|| panic!("`{}` is no round",surface.name))
    }

    /// Where the layout keeps what the checks call a rack section's crown, `pinion`, `gear_outer`
    /// or `gear_inner`: the crown tooth, and its mate's two sides of the gear's space cutter (the
    /// outer mate's region, closed at the reach's cap, and the inner's, its neighbour unturned).
    fn crown(rack: &str) -> &'static str {
        match rack { "pinion" => "tooth.crown", "gear_outer" => "gear_space.outer_crown",
            "gear_inner" => "gear_space.inner_crown", _ => panic!("no rack section `{rack}`") }
    }

    /// The flank's `u` where it meets its round, which is also the round's where it meets the
    /// tip: each crown's faces run along its meridian's counterclockwise walk about the cutter's
    /// axis (`cutter_faces`), the tooth's from its base up its outer flank, its mate's (tip down)
    /// from its base down its inner — so the tooth's outer flank ends at its round and its inner
    /// begins there, and its mate's the other way round.
    fn flank_join(member: usize,outer: bool) -> f64 { if outer == (member == 0) { 1. } else { 0. } }

    /// Which way a crown surface's chart normal points against its crown's material, +1 out of
    /// it: a chart's normal follows its edge and its revolution, and the mate's run the other way
    /// round, so the sign is read rather than assumed.
    fn outward(&self,patch: &RevolvedSurface) -> f64 {
        let region = gcs_core::solid::RevolvedRegion::read(&self.model.sketch,self.crown_of(patch),self.module*1e-10).unwrap();
        let [from,to] = patch.domain()[1];
        let s = patch.at(0.5,(from+to)/2.).unwrap();
        let position = s.position;
        let n = envelope::contact(s,Motion::identity()).unwrap().normal;
        let p: [f64;3] = std::array::from_fn(|k| position[k]+1e-3*self.module*n[k]);
        region.classify(p,0.).unwrap().signed_distance.signum()
    }

    fn patch(&self, member: usize, side: usize, edge: &str) -> RevolvedSurface {
        let rack = if member == 0 { "pinion" }
            else if side == 0 { "gear_outer" } else { "gear_inner" };
        self.surfaces[&format!("faces.{rack}_{edge}")].0.clone()
    }

    /// What the checks call a cutter face, `faces.pinion_outer`, which is its surface's name — a
    /// flank's the prefix of the seams, corners, edges and faces its side has.

    fn body(&self, member: usize, t: f64) -> Motion {
        let d = self.delta[member];
        let sign = if member == 0 { 1. } else { -1. };
        rotate(2,sign*t/d.sin(),sign/d.sin())
            .then(rotate(1,FRAC_PI_2-sign*d,0.))
    }

    fn region(&self,surface: &RevolvedSurface) -> &gcs_core::patch::TrimmedPatch {
        &self.regions[&surface.name]
    }

    fn seam(&self,flank: &RevolvedSurface,root: bool) -> &gcs_core::seam::EnvelopeSeam {
        &self.seams[&format!("{}.{}join",flank.name,if root { "root_" } else { "" })]
    }

    fn seam_at(&self,member: usize,seam: &gcs_core::seam::EnvelopeSeam,rho: f64)
        -> envelope::Intersection {
        let [_,v,roll] = seam.domain();
        let bounds = [v,roll];
        let mut result = seam.intersect(|c| c.position[0].hypot(c.position[1]).hypot(c.position[2])-rho,
            bounds.map(|b| (b[0]+b[1])/2.),gcs_core::seam::SeamIntersectionOptions {
                bounds,parameter_scale:[1.;2],normal_tolerance:1e-10*self.module,
                section_tolerance:1e-10*self.module,max_iterations:100},
            1e-8*self.module).unwrap_or_else(|e| panic!("{}, rho {rho}: {e:?}",seam.name));
        let frame = self.local_frame(member);
        result.contact = envelope::Contact {position:frame.point(result.contact.position),
            normal:frame.vector(result.contact.normal),velocity:frame.vector(result.contact.velocity),
            normal_velocity:result.contact.normal_velocity};
        result
    }

    fn domain(&self,surface: &RevolvedSurface) -> [[f64;2];3] {
        self.envelopes[&surface.name].domain()
    }

    fn check_trim(&self,surface: &RevolvedSurface,parameters: [f64;3]) {
        self.region(surface).envelope_at(parameters,1e-8*self.module,1e-8*self.module)
            .unwrap_or_else(|e| panic!("{} at {parameters:?}: {e:?}",surface.name));
    }

    fn local_frame(&self, member: usize) -> Motion {
        let sign = if member == 0 { 1. } else { -1. };
        rotate(1,FRAC_PI_2-sign*self.delta[member],0.).inverse()
    }

    fn motion(&self, member: usize, t: f64) -> Motion {
        // A coordinate convention only: geometry and rolling come from the source.
        self.motion_families[member].at(t).unwrap().then(self.local_frame(member))
    }

    fn intersection(&self, member: usize, patch: &RevolvedSurface,
        section: impl Fn([f64;3],&envelope::Contact) -> [f64;2],
        seed: [f64;3], options: IntersectionOptions) -> envelope::Intersection {
        let generated = &self.envelopes[&patch.name];
        let frame = self.local_frame(member);
        let local = |c: &envelope::Contact| envelope::Contact {
            position:frame.point(c.position),normal:frame.vector(c.normal),
            velocity:frame.vector(c.velocity),normal_velocity:c.normal_velocity,
        };
        let mut result = generated.intersect(section,seed,options)
            .unwrap_or_else(|e| panic!("member {member}, {} at {seed:?}: {e:?}",patch.name));
        result.contact = local(&result.contact);
        result
    }

    fn at(&self, member: usize, patch: &RevolvedSurface, u: f64, rho: f64)
        -> envelope::Intersection {
        let [from,to] = patch.domain()[1];
        let v = (from+to)/2.;
        self.at_seed(member,patch,u,rho,[u,v,0.])
    }

    fn at_seed(&self, member: usize, patch: &RevolvedSurface, u: f64, rho: f64,
        mut seed: [f64;3]) -> envelope::Intersection {
        seed[0] = u;
        self.intersection(member,patch,
            |p,c| [p[0]-u,c.position[0].hypot(c.position[1]).hypot(c.position[2])-rho],
            seed,IntersectionOptions {
                bounds: [[u,u],self.domain(patch)[1],self.domain(patch)[2]],
                parameter_scale: [1.;3],
                residual_tolerance: [1e-10*self.module,1e-11,1e-10*self.module],
                max_iterations: 100,
            })
    }

    fn at_height(&self, member: usize, patch: &RevolvedSurface, height: f64, rho: f64)
        -> envelope::Intersection {
        let a = patch.at(0.,patch.domain()[1][0]).unwrap();
        self.at(member,patch,(height-a.position[2])/a.du[2],rho)
    }
}

#[test]
fn finite_edges_follow_independent_characteristics_and_declared_axial_slices() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            assert_eq!(pair.edges.len(),28);
            for member in 0..2 {
                let frame = pair.local_frame(member);
                for side in 0..2 {
                    let outer = member == side;
                    let name = if outer { "outer" } else { "inner" };
                    let flank = pair.patch(member,side,name);
                    let round = pair.patch(member,side,&format!("{name}_round"));
                    for role in ["tip","join","root","toe","heel","round_toe","round_heel"] {
                        let edge = &pair.edges[&format!("{}.{role}_span",flank.name)];
                        let source = if role == "root" || role.starts_with("round_") { &round } else { &flank };
                        let [a,b] = edge.endpoints().map(|p| frame.point(p.position));
                        for fraction in [0.,0.25,0.5,0.75,1.] {
                            let p = edge.sample(fraction,100)
                                .unwrap_or_else(|e| panic!("{} at {fraction}: {e:?}",edge.name));
                            let actual = frame.point(p.position);
                            let rho = if role.ends_with("toe") { 0.9*pair.rm }
                                else if role.ends_with("heel") { 1.1*pair.rm }
                                else { actual[0].hypot(actual[1]).hypot(actual[2]) };
                            assert!(rho >= 0.9*pair.rm-module*1e-8 && rho <= 1.1*pair.rm+module*1e-8);
                            let expected = pair.analytic(member,source,p.parameters[0],rho);
                            near(p.parameters,expected.parameters,1e-8);
                            near(actual,expected.contact.position,module*1e-7);
                            assert!((actual[2]-((1.-fraction)*a[2]+fraction*b[2])).abs() < module*1e-8);
                            if role == "tip" {
                                let angle = pair.delta[member]+(module*35f64.to_radians().cos()/rho).asin();
                                assert!((actual[0].hypot(actual[1]).atan2(actual[2])-angle).abs() < 1e-8);
                            } else if role == "join" || role == "root" {
                                assert_eq!(p.parameters[0],Pair::flank_join(member,outer));
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn declared_boundary_seams_match_independent_spheres_and_tip_cones_across_sizes_and_ratios() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            assert_eq!(pair.boundary_seams.len(),20);
            assert_eq!(pair.corners.len(),24);
            for member in 0..2 {
                for side in 0..2 {
                    let outer = member == side;
                    let edge = if outer { "outer" } else { "inner" };
                    let flank = pair.patch(member,side,edge);
                    let round = pair.patch(member,side,&format!("{edge}_round"));
                    let join = Pair::flank_join(member,outer);
                    for fraction in [0.9,1.,1.1] {
                        let rho = fraction*pair.rm;
                        // Independent scalar bracketing of the addendum cone on
                        // the closed-form characteristic; no boundary projector.
                        let theta = pair.delta[member]+(module*35f64.to_radians().cos()/rho).asin();
                        let at = |s| pair.analytic(member,&flank,join+s*(1.-2.*join),rho);
                        let polar = |p: [f64;3]| p[0].hypot(p[1]).atan2(p[2]);
                        let (mut lo,mut hi) = (0.,1.);
                        assert!(polar(at(lo).contact.position) < theta);
                        assert!(polar(at(hi).contact.position) > theta);
                        for _ in 0..48 {
                            let mid = (lo+hi)/2.;
                            if polar(at(mid).contact.position) < theta { lo = mid; } else { hi = mid; }
                        }
                        let expected = at((lo+hi)/2.);
                        let seed = pair.seam_at(member,pair.seam(&flank,false),rho).parameters;
                        let tip = pair.tip(member,&flank,rho,seed);
                        near(tip.parameters,expected.parameters,1e-8);
                        near(tip.contact.position,expected.contact.position,module*1e-7);
                        if fraction == 1. { continue; }
                        let end = if fraction < 1. { "toe" } else { "heel" };
                        let corner = pair.corners[&format!("{}.tip_{end}",flank.name)];
                        near(corner.parameters,expected.parameters,1e-8);
                        near(pair.local_frame(member).point(corner.position),expected.contact.position,module*1e-7);
                        for source in [&flank,&round] {
                            let role = if source.name == flank.name { "" } else { "round_" };
                            let seam = &pair.boundary_seams[&format!("{}.{role}{end}_edge",flank.name)];
                            for s in [0.2,0.5,0.8] {
                                let u = if source.name == flank.name {
                                    join+s*(expected.parameters[0]-join)
                                } else { s };
                                let expected = pair.analytic(member,source,u,rho);
                                let frame = pair.local_frame(member);
                                let bounds = seam.domain();
                                let seed = std::array::from_fn(|i|
                                    (expected.parameters[i]+0.002).clamp(bounds[i][0],bounds[i][1]));
                                let point = seam.intersect(|c|
                                    frame.point(c.position)[2]-expected.contact.position[2],seed,
                                    IntersectionOptions {bounds,parameter_scale:[1.;3],
                                        residual_tolerance:[1e-10*module;3],max_iterations:100},
                                    1e-8*module).unwrap_or_else(|e| panic!("{}: {e:?}",seam.name));
                                near(point.parameters,expected.parameters,1e-8);
                                near(frame.point(point.contact.position),expected.contact.position,module*1e-7);
                                near(frame.vector(point.contact.normal),expected.contact.normal,1e-8);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn declared_seams_agree_with_independent_characteristics_across_ratios_and_sizes() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            assert_eq!(pair.seams.len(),8);
            for member in 0..2 {
                for side in 0..2 {
                    let outer = member == side;
                    let edge = if outer { "outer" } else { "inner" };
                    let flank = pair.patch(member,side,edge);
                    for root in [false,true] {
                        let seam = pair.seam(&flank,root);
                        let source = pair.patch(member,side,
                            &if root { format!("{edge}_round") } else { edge.into() });
                        let u = Pair::flank_join(member,outer);
                        // the flank's end and the round's opposite; the root's whichever way its
                        // edge runs in the cutter's profile (the gear's space cutter's tip is a
                        // run its face closes, the crown tooth's a line of its section)
                        let [first,second] = seam.endpoint_parameters();
                        assert_eq!(first,u);
                        if root { assert!(second == 0. || second == 1.) } else { assert_eq!(second,1.-u) }
                        for fraction in [0.9,1.,1.1] {
                            let rho = fraction*pair.rm;
                            let actual = pair.seam_at(member,seam,rho);
                            let expected = pair.analytic(member,&source,u,rho);
                            near(actual.parameters,expected.parameters,1e-8);
                            near(actual.contact.position,expected.contact.position,module*1e-7);
                            near(actual.contact.normal,expected.contact.normal,1e-8);
                            if fraction != 1. {
                                let end = if fraction < 1. { "toe" } else { "heel" };
                                let role = if root { "root" } else { "join" };
                                let corner = pair.corners[&format!("{}.{role}_{end}",flank.name)];
                                near(corner.parameters,expected.parameters,1e-8);
                                near(pair.local_frame(member).point(corner.position),expected.contact.position,module*1e-7);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn declarative_mates_have_opposing_normals_and_exact_half_pitch_spacing() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            let mut angles = [[0.;2];2];
            for (side,edges) in [["outer","inner"],["inner","outer"]].iter().enumerate() {
                let p = pair.patch(0,side,edges[0]);
                let g = pair.patch(1,side,edges[1]);
                // each normal out of its own crown's material
                let (sp,sg) = (pair.outward(&p),pair.outward(&g));
                for face in [0.9,1.,1.1] {
                    for height in [-0.2,0.,0.2] {
                        let a = pair.at_height(0,&p,height*module,face*pair.rm);
                        let b = pair.at_height(1,&g,height*module,face*pair.rm);
                        let t = a.parameters[2];
                        assert!((t-b.parameters[2]).abs() < 1e-8);
                        let ap = pair.body(0,t).point(a.contact.position);
                        let bp = pair.body(1,t).point(b.contact.position);
                        near(ap,bp,module*1e-7);
                        let an = pair.body(0,t).vector(a.contact.normal);
                        let bn = pair.body(1,t).vector(b.contact.normal);
                        near(an.map(|x| sp*x),bn.map(|x| -sg*x),1e-7);
                        // Actual shaft velocities, independent of the crown-envelope residual.
                        let av = pair.body(0,t).velocity(a.contact.position);
                        let bv = pair.body(1,t).velocity(b.contact.position);
                        let relative_normal = (0..3).map(|k| an[k]*(av[k]-bv[k])).sum::<f64>();
                        assert!(relative_normal.abs() < module*1e-8,
                            "{teeth:?}, module {module}, side {side}, face {face}, height {height}: {relative_normal}");
                        if face == 1. && height == 0. {
                            for (m,c) in [a.contact,b.contact].iter().enumerate() {
                                angles[m][side] = c.position[1].atan2(c.position[0]);
                            }
                        }
                    }
                }
            }
            for member in 0..2 {
                let width = (angles[member][1]-angles[member][0]+PI).rem_euclid(TAU)-PI;
                assert!((width.abs()-PI/pair.teeth[member]).abs() < 1e-8,
                    "{teeth:?} module {module}: angular width {width}");
            }
        }
    }
}

#[test]
fn both_members_have_tangent_generated_root_transitions_on_both_sides() {
    let pair = Pair::read([24,48],2.);
    for member in 0..2 {
        for side in 0..2 {
            let outer = member == side;
            let edge = if outer { "outer" } else { "inner" };
            let flank = pair.patch(member,side,edge);
            let round = pair.patch(member,side,&format!("{edge}_round"));
            let tip = pair.patch(member,side,"tip");
            let join = 1.-Pair::flank_join(member,outer);
            for face in [0.9,1.,1.1] {
                let rho = face*pair.rm;
                let a = pair.at(member,&flank,1.-join,rho);
                let b = pair.at_seed(member,&round,join,rho,a.parameters);
                near(a.contact.position,b.contact.position,1e-7);
                near(a.contact.normal,b.contact.normal,1e-7);
                let mut previous = b.contact.position;
                let mut seed = b.parameters;
                let mut min_segment = f64::INFINITY;
                for i in 1..=40 {
                    let u = join+(1.-2.*join)*i as f64/40.;
                    let point = pair.at_seed(member,&round,u,rho,seed);
                    seed = point.parameters;
                    let c = point.contact;
                    let segment = (0..3).map(|k| (c.position[k]-previous[k]).powi(2))
                        .sum::<f64>().sqrt();
                    min_segment = min_segment.min(segment);
                    previous = c.position;
                }
                assert!(min_segment > 0.001,"member {member} side {side}: {min_segment}");
                let end = pair.at_seed(member,&round,1.-join,rho,seed);
                // the tip's own end at the round, whichever way its edge runs
                let tip_end = pair.seam(&flank,true).endpoint_parameters()[1];
                let root = pair.at_seed(member,&tip,tip_end,rho,seed);
                near(end.contact.position,root.contact.position,1e-7);
                // tangent: one plane, its normal either way (a chart's normal follows its edge,
                // and the gear's space cutter walks its tip the other way)
                let sense = (0..3).map(|k| end.contact.normal[k]*root.contact.normal[k]).sum::<f64>().signum();
                near(end.contact.normal,root.contact.normal.map(|x| sense*x),1e-7);
                let p = end.contact.position;
                let d = pair.delta[member];
                let depth = p[0].hypot(p[1])*d.cos()-p[2]*d.sin();
                let declared = tip.at(0.,tip.domain()[1][0]).unwrap().position[2].abs();
                assert!((depth+declared).abs() < 1e-7,"root depth {depth}, declared {declared}");
            }
        }
    }
}

/// The gear's inner mate is its space cutter's neighbour (crown/mate.sv): the inner crown turned
/// a crown pitch about the crown's axis, which the generating roll turns about too, so the
/// neighbour's face generates the inner mate's envelope turned a gear tooth about the gear's
/// axis — the face the indexed member's previous space leaves, which the checks read. Held here
/// at points of the unplaced envelope: each, turned a tooth, lies on the placed one, the face the
/// built space has (`face_source`).
#[test]
fn the_gears_inner_mate_is_its_space_cutters_neighbour_one_tooth_over() {
    use gcs_core::solid::faces::{face_source,FacePath,FaceSource};
    let pair = Pair::read([24,48],2.);
    let sk = &pair.model.sketch;
    let body = pair.model.map.ent_named("pair.gear.body").unwrap().i();
    let FaceSource::Swept {surface,motion,within,..} = face_source(sk,body,&FacePath::new("indexed","neighbor.outer")).unwrap()
        else { panic!("the neighbour's flank is a cut face") };
    assert!(within.point([1.,2.,3.]).iter().zip([1.,2.,3.]).any(|(a,b)| (a-b).abs() > 1e-3),"the neighbour is turned");
    let placed = envelope::GeneratedEnvelope::new("placed".into(),surface,motion,[-PI,PI]).unwrap();
    let unplaced = pair.patch(1,1,"outer");
    let frame = pair.local_frame(1);
    let pitch = TAU/pair.teeth[1];
    let [_,v,roll] = placed.domain();
    // turned a tooth one way or the other about the gear's axis (local z): the first point says
    // which, and every other is held to it
    let mut senses = vec![-1.,1.];
    for u in [0.2,0.5,0.8] {
        for rho in [0.95*pair.rm,1.05*pair.rm] {
            let c = pair.analytic(1,&unplaced,u,rho);
            let off = |sense: f64| {
                let q = frame.inverse().point(rotate(2,sense*pitch,0.).point(c.contact.position));
                let (e1,e2) = space::across(frame.inverse().vector(rotate(2,sense*pitch,0.).vector(c.contact.normal)));
                (0..=72).flat_map(|i| (0..=24).map(move |j| (i,j))).find_map(|(i,j)| {
                    let seed = [u,v[0]+(v[1]-v[0])*i as f64/72.,roll[0]+(roll[1]-roll[0])*j as f64/24.];
                    placed.intersect(|_,c| { let x = space::sub(c.position,q); [space::dot(x,e1),space::dot(x,e2)] },seed,
                        IntersectionOptions {bounds:[[u,u],v,roll],parameter_scale:[1.;3],residual_tolerance:[1e-10,1e-11,1e-10],
                        max_iterations:50}).ok()
                        .map(|found| space::distance(found.contact.position,q)).filter(|&d| d < 1e-7)
                }).unwrap_or(f64::INFINITY)
            };
            let d: Vec<(f64,f64)> = senses.iter().map(|&s| (s,off(s))).collect();
            let best = d.iter().copied().min_by(|a,b| a.1.total_cmp(&b.1)).unwrap();
            assert!(best.1 < 1e-7,"u {u}, rho {rho}: {d:?} off the placed neighbour");
            senses = vec![best.0];
        }
    }
}

#[test]
fn named_rolling_motions_match_independent_shaft_kinematics() {
    for teeth in [[24,48],[32,32],[28,49]] {
        let pair = Pair::read(teeth,2.);
        for member in 0..2 {
            for t in [-0.2,0.,0.3] {
                let old = rotate(2,t,1.).then(pair.body(member,t).inverse());
                let named = pair.motion(member,t);
                for p in [[0.;3],[1.,2.,3.],[-40.,11.,5.]] {
                    near(named.point(p),old.point(p),1e-8);
                    near(named.vector(p),old.vector(p),1e-8);
                    near(named.velocity(p),old.velocity(p),1e-8);
                }
            }
        }
    }
}

/// Each member's blank is its built faces (`material.toe`, …): the end spheres about the apex
/// through 0.9 and 1.1 of the mean cone distance, the tip cone the pitch cone moved out by the
/// addendum and the back cone moved in by the back depth, each its blank's material on the side
/// its term bounds — held to the closed forms of the layout, with no drawn limit.
#[test]
fn the_blanks_faces_are_independent_spheres_and_parallel_pitch_cones() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            for member in 0..2 {
                let material = gcs_core::solid::RevolvedRegion::read(&pair.model.sketch,pair.blanks[member],module*1e-10).unwrap();
                // the material is outside the toe and inside the heel
                for (name,fraction,outward) in [("toe",0.9,-1.),("heel",1.1,1.)] {
                    let s = pair.wall(member,name);
                    let projector = s.projector().unwrap();
                    for u in [0.,0.2,0.5,0.8,1.] {
                        for v in [0.,0.2,0.7,1.] {
                            let p = s.at(u,v).unwrap().position;
                            let r = p[0].hypot(p[1]).hypot(p[2]);
                            assert!((r-fraction*pair.rm).abs() < module*1e-8,"{name}: {r}");
                            let on = projector.project(p).unwrap();
                            assert!(on.incidence_error < module*1e-8,"{name}: {on:?}");
                            if u == 0. || u == 1. { continue; }
                            for offset in [-0.01,0.01] {
                                let q = p.map(|x| x*(1.+offset*module/r));
                                let sample = material.classify(q,module*1e-10).unwrap();
                                assert!((sample.signed_distance-outward*offset*module).abs() < module*1e-8,
                                    "{name} at {u}, {v}: {} for {offset}",sample.signed_distance);
                            }
                        }
                    }
                }
                let d = pair.delta[member];
                // the material is within the tip cone and outside the back cone
                for (role,depth,outward) in [("tip",1.,1.),("back",-4.,-1.)] {
                    let s = pair.wall(member,role);
                    let projector = s.projector().unwrap();
                    for u in [0.,0.3,0.8,1.] {
                        for v in [0.,0.2,0.6,1.] {
                            let sample = s.at(u,v).unwrap();
                            let p = pair.local_frame(member).point(sample.position);
                            let value = p[0].hypot(p[1])*d.cos()-p[2]*d.sin();
                            assert!((value-depth*module*35f64.to_radians().cos()).abs() < module*1e-8,"{role}: {value}");
                            let on = projector.project(sample.position).unwrap();
                            assert!(on.incidence_error < module*1e-8,"{role}: {on:?}");
                            let normal = envelope::contact(sample,Motion::identity()).unwrap().normal;
                            near(on.normal,normal,1e-8);
                            if u > 0. && u < 1. {
                                let phi = p[1].atan2(p[0]);
                                let away = [d.cos()*phi.cos(),d.cos()*phi.sin(),-d.sin()];
                                for offset in [-0.01,0.01] {
                                    let local = std::array::from_fn(|i| p[i]+offset*module*away[i]);
                                    let q = pair.local_frame(member).inverse().point(local);
                                    let value = material.classify(q,module*1e-10).unwrap();
                                    assert!((value.signed_distance-outward*offset*module).abs() < module*1e-8,
                                        "{role} at {u}, {v}: {} for {offset}",value.signed_distance);
                                }
                            }
                        }
                    }
                    for fraction in [0.9,1.,1.1] {
                        let p = pair.cone_on_sphere(member,role,fraction*pair.rm);
                        assert!((p[0].hypot(p[1]).hypot(p[2])-fraction*pair.rm).abs() < module*1e-8);
                    }
                }
            }
        }
    }
}

#[test]
fn named_boundary_intersections_find_tooth_corners_and_refuse_surface_continuations() {
    let pair = Pair::read([24,48],2.);
    for member in 0..2 {
        for side in 0..2 {
            let edge = if member == side { "outer" } else { "inner" };
            let patch = pair.patch(member,side,edge);
            let generated = &pair.envelopes[&patch.name];
            let tip = pair.wall(member,"tip").projector().unwrap();
            for end in 0..2 {
                let sphere = pair.wall(member,["toe","heel"][end]);
                let face = sphere.projector().unwrap();
                let p = sphere.at(0.5,0.).unwrap().position;
                let rho = p[0].hypot(p[1]).hypot(p[2]);
                let seed = pair.at(member,&patch,0.5,rho).parameters;
                let options = IntersectionOptions {
                    bounds:pair.domain(&patch),
                    parameter_scale:[1.;3],residual_tolerance:[1e-9*pair.module;3],max_iterations:100,
                };
                let corner = pair.region(&patch).intersect_boundaries([&tip,&face],seed,options,
                    1e-8*pair.module).unwrap();
                let expected = pair.tip(member,&patch,rho,seed);
                near(pair.local_frame(member).point(corner.contact.position),expected.contact.position,1e-7);
                if member == 0 && side == 0 && end == 0 {
                    // Same sphere equation, but a narrow angular patch away from the corner.
                    let narrow = sphere.spanned([0.,0.001]).unwrap().projector().unwrap();
                    assert!(narrow.project(corner.contact.position).unwrap().incidence_error > 0.01);
                    assert_eq!(generated.intersect_boundaries([&tip,&narrow],seed,options).unwrap_err(),
                        envelope::Error::OutsideDomain);
                }
            }
        }
    }
}
