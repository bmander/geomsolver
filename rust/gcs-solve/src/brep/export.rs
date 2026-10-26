//! A solid's exact export by this kernel, as every host makes it (phase 5 of
//! docs/rust-kernel-plan.md): the terminal's `solventc` and the browser's worker call the same
//! functions, so a file is one file wherever it is written. A body is built from its CAD recipe,
//! or — with swept cuts of the generating class, admitted — as one sector patterned
//! (`brep::sweep`); its STEP is written and parsed back against it (`step_check`); its STL is meshed
//! within the tolerance's deflection (the measured sag, a bound and not a control), its shells
//! checked, and a swept body's judged against its material field (`agreement`), nothing given back
//! unless it agrees.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::pattern::Built as Patterned;
use super::sweep::Say;
use super::boolean::Op;
use super::topo::Brep;
use super::geom::Frame;
use crate::model::Sketch;
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use crate::solid::{admission,agreement,cad,contracts,MaterialField};

/// The mesh's deflection with no tolerance given: the export's gross bar (mm).
pub const GROSS: f64 = 0.01;
/// The mesh's angular control (radians).
pub const ANGULAR: f64 = 0.2;

/// A body built exactly: its B-rep, and, built as one sector, the pattern that meshes it and the
/// turn its field agreement reads each probe in.
#[derive(Clone,Debug)]
pub struct Exact { pub solid: Brep,pub pattern: Option<Patterned>,pub indexed: Option<([f64;3],[f64;3],usize)>,
    /// Whether the body cuts sweeps, so its mesh is held to its material field.
    pub swept: bool,
    /// Built with no face, the statement that left it no material, as the document writes it (#617).
    pub emptied: Option<String> }

impl Exact {
    /// The body carried by the rigid motion `m` (mm): its pattern and the turn its agreement folds
    /// by moved with it.
    pub fn moved(&self,m: &super::geom::Rigid) -> Exact {
        Exact {solid:self.solid.moved(m),pattern:self.pattern.as_ref().map(|p| p.moved(m)),
            indexed:self.indexed.map(|(o,a,n)| (m.point(o),m.vector(a),n)),swept:self.swept,
            emptied:self.emptied.clone()}
    }
}

/// Solid `body` of the sketch built by this kernel. A body with swept cuts is admitted to the
/// generating-sweep class first (`admission` where the host made it, else here). A body built with
/// no face is refused as empty (`emptied`), however it was built, so no file is written of it.
pub fn exact(sk: &Sketch,body: usize,admitted: Option<&admission::Admission>,tolerance: Option<Tolerance>,say: &Say) -> Result<Exact,ExportRefusal> {
    let built = build(sk,body,admitted,tolerance,say)?;
    if built.solid.faces.is_empty() {
        return Err(match &built.emptied { Some(why) => empty(&sk.solids[body].name,why),None => emptied(sk,body) })
    }
    Ok(built)
}

/// Solid `body` built by whichever path its operands take (`exact`).
fn build(sk: &Sketch,body: usize,admitted: Option<&admission::Admission>,tolerance: Option<Tolerance>,say: &Say) -> Result<Exact,ExportRefusal> {
    // a mirrored instance's solid, or one built of one, is the core's exact B-rep, which reads a
    // mirrored solid as it is published, reflected (#578)
    let mirrored = mirrored_within(sk,body);
    if mirrored && cad::contains_sweep(sk,body) {
        return Err(ExportRefusal::at(Stage::Blank,format!("`{}` reads a mirrored instance's solid, and a sweep is not reflected yet",
            sk.solids[body].name)))
    }
    // a body placed by a rigid motion (`x in f`) is the body it places, built once and moved
    // (#619): its volume the body's, its pattern and the turn its agreement folds by moved with it
    if let Some((inner,pose)) = cad::placed_swept(sk,body).at(Stage::Blank)? {
        let m = super::sweep::sector::placed(pose,cad::millimetres(sk));
        let built = exact(sk,inner,admitted,tolerance,say)?;
        (say.stage)(&format!("`{}` is `{}` moved",sk.solids[body].name,sk.solids[inner].name));
        return Ok(built.moved(&m))
    }
    // a closed instance's swept body is its binding's representative's build placed (#566): built
    // once for the drawing and the tolerance, and moved where each copy stands, its faces renamed
    // from the representative's instance to the copy's
    let bound = sk.bound_where(body,true);
    let rep = bound.as_ref().map_or(body,|b| b.rep);
    if cad::contains_sweep(sk,body) && sk.shared_where(body,true) && sk.solids[rep].binding.as_ref().is_some_and(|b| b.mirror.is_none()) {
        let key = crate::solid::MemoKey::of(sk,rep);
        let tol = tolerance.map(|t| t.millimetres.to_bits());
        let held = sk.caches.swept_exact.get(rep,&key).filter(|(t,_)| *t == tol).map(|(_,e)| e);
        let built = match held {
            Some(e) => e,
            None => {
                let e = std::rc::Rc::new(own(sk,rep,if rep == body { admitted } else { None },false,tolerance,say)?);
                sk.caches.swept_exact.put(rep,key,(tol,e.clone()));
                e
            }
        };
        let Some(b) = bound else { return Ok((*built).clone()) };
        let mm = cad::millimetres(sk);
        let mut placed = built.moved(&super::geom::Rigid {r:b.raw.r,t:b.raw.t.map(|k| k*mm)});
        for f in &mut placed.solid.faces {
            if let Some(rest) = f.name.strip_prefix(b.from.as_str()) { f.name = format!("{}{rest}",b.to); }
        }
        (say.stage)(&format!("`{}` is `{}` placed: one build of their binding",sk.solids[body].name,sk.solids[rep].name));
        return Ok(placed)
    }
    own(sk,body,admitted,mirrored,tolerance,say)
}

/// Solid `body` built as its own: from swept material (`compose`), with swept cuts of its own, or
/// from its recipe (as published, where it reads a `mirrored` solid).
fn own(sk: &Sketch,body: usize,admitted: Option<&admission::Admission>,mirrored: bool,tolerance: Option<Tolerance>,say: &Say)
    -> Result<Exact,ExportRefusal> {
    if !cad::swept_operands(sk,body).is_empty() { return compose(sk,body,tolerance,say) }
    let recipe = cad::recipe_static(sk,body).at(Stage::Blank)?;
    if !recipe.sweeps.is_empty() {
        let made;
        let admission = match admitted {
            Some(a) => a,
            None => { made = admission::admit_body(sk,body,&admission::Options::default())?; (say.mark)(Stage::Admission); &made }
        };
        return Ok(match super::sweep::build(sk,body,&recipe,admission,tolerance,say)? {
            super::sweep::Built::Sector {sector,pattern} => {
                (say.stage)(&format!("the solid by the Rust kernel: {} sectors, {} faces, {} edges",sector.count,pattern.solid.faces.len(),
                    pattern.solid.edges.len()));
                Exact {solid:pattern.solid.clone(),pattern:Some(pattern),indexed:Some((sector.origin,sector.axis,sector.count)),swept:true,emptied:None}
            }
            super::sweep::Built::Whole(solid) => {
                (say.stage)(&format!("the solid by the Rust kernel: {} faces, {} edges",solid.faces.len(),solid.edges.len()));
                Exact {solid,pattern:None,indexed:None,swept:true,emptied:None}
            }
        })
    }
    let started = crate::clock::Instant::now();
    let solid = match mirrored {
        true => {
            let x = sk.exact_solid(body).at(Stage::Blank)?;
            x.brep.moved(&super::geom::Rigid {t:x.origin.map(|k| k*x.mm),..super::geom::Rigid::identity()})
        }
        false => super::recipe::build(&recipe.recipe).at(Stage::Blank)?,
    };
    let tol = 1e-9*solid.size().max(1.);
    solid.check(10.*tol).map_err(|e| ExportRefusal::at(Stage::Blank,format!("the built boundary is invalid: {e}")))?;
    if let Some([a,b]) = solid.self_touching(tol).first() {
        return Err(ExportRefusal::at(Stage::Blank,format!("the solid touches itself along a line from [{:.4}, {:.4}, {:.4}] to \
            [{:.4}, {:.4}, {:.4}] mm (two of its pieces meet there and nowhere else near, as rounds of separate statements leave \
            them where each reads the body before the other): no manifold file describes it",a[0],a[1],a[2],b[0],b[1],b[2])))
    }
    if let Some(p) = solid.pinches().first() {
        return Err(ExportRefusal::at(Stage::Blank,format!("the solid pinches to a point at [{:.4}, {:.4}, {:.4}] mm (two of its \
            walls touch there without an edge between them): no manifold file describes it",p[0],p[1],p[2])))
    }
    (say.stage)(&format!("built the solid by the Rust kernel: {:.6} mm³, {} faces, {} edges ({:?})",super::props::volume(&solid),
        solid.faces.len(),solid.edges.len(),started.elapsed()));
    Ok(Exact {solid,pattern:None,indexed:None,swept:false,emptied:None})
}

/// Whether solid `body` is a mirrored instance's, or reaches one among its operands (#578).
fn mirrored_within(sk: &Sketch,body: usize) -> bool {
    let mut pending = vec![body];
    let mut seen = std::collections::BTreeSet::new();
    while let Some(i) = pending.pop() {
        if !seen.insert(i) { continue }
        if sk.solids[i].binding.as_ref().is_some_and(|b| b.mirror.is_some()) { return true }
        pending.extend(sk.solids[i].operands().into_iter().map(|o| o as usize));
    }
    false
}

/// A body built from swept material (`cad::swept_operands`): each operand built exactly — a body with
/// swept cuts by its own construction, the rest by its recipe — and combined by the body rule
/// (`combine`).
fn compose(sk: &Sketch,body: usize,tolerance: Option<Tolerance>,say: &Say) -> Result<Exact,ExportRefusal> {
    let started = crate::clock::Instant::now();
    let (built,emptied) = combine(sk,body,&mut |o| if cad::contains_sweep(sk,o) { Ok(exact(sk,o,None,tolerance,say)?.solid) } else { built_static(sk,o) })?;
    (say.stage)(&format!("`{}` built from its swept material by this kernel: {:.6} mm³, {} faces ({:?})",sk.solids[body].name,
        super::props::volume(&built),built.faces.len(),started.elapsed()));
    (say.mark)(Stage::Fuse);
    Ok(Exact {solid:built,pattern:None,indexed:None,swept:true,emptied})
}

/// A static solid built from its recipe by this kernel.
fn built_static(sk: &Sketch,o: usize) -> Result<Brep,ExportRefusal> {
    super::recipe::build(&cad::recipe_static(sk,o).at(Stage::Blank)?.recipe).at(Stage::Blank)
}

/// The body rule over operands `part` builds (millimetres): the stock, plus what is put on it, minus
/// what cuts it, within what bounds it, by this kernel's Booleans. A sweep cut from it directly is
/// refused: the swept material is named apart, as `fluted` is from `drill`.  A body left with no
/// material is built empty, with why (#617): no later statement puts material back, and an export
/// refuses it where a drawing reads it, its volume 0 (#618).
fn combine(sk: &Sketch,body: usize,part: &mut dyn FnMut(usize) -> Result<Brep,ExportRefusal>)
    -> Result<(Brep,Option<String>),ExportRefusal> {
    let solid = &sk.solids[body];
    let crate::model::SolidDef::Body {stock,on,through,bound} = &solid.def else { unreachable!("a composed solid is a body") };
    if let Some(&c) = through.iter().find(|&&c| cad::contains_sweep(sk,c as usize)) {
        return Err(ExportRefusal::at(Stage::Blank,format!("`{}` cuts the sweep `{}` from swept material: name the body the sweep is \
            cut from, and build `{}` from it",solid.name,sk.solids[c as usize].name,solid.name)))
    }
    let mut built = part(*stock as usize)?;
    if built.faces.is_empty() {
        return Ok((built,Some(format!("its stock `{}` holds no material",sk.solids[*stock as usize].name))))
    }
    for (operands,op,word) in [(on,Op::Union,"union"),(through,Op::Cut,"cut"),(bound,Op::Common,"common")] {
        for &o in operands {
            let operand = part(o as usize)?;
            let name = &sk.solids[o as usize].name;
            built = super::boolean::combined(&built,&operand,op,0.)
                .map_err(|e| ExportRefusal::at(Stage::Fuse,format!("`{}`: {word} `{name}`: {e}",solid.name)))?;
            // the statement that left nothing, named as the document writes it (#617)
            if built.faces.is_empty() {
                return Ok((built,Some(format!("`{name} {word} {}` leaves no material",solid.name))))
            }
        }
    }
    Ok((built,None))
}

/// **An empty body refused as a fact of the document** (#617): a Boolean that leaves no material
/// is what was drawn, not a limit of the encoding, so its export says which solid is empty and
/// why — never a float32 rounding read off the empty solid's infinite bounds, nor a file of no
/// faces.
fn empty(name: &str,why: &str) -> ExportRefusal {
    ExportRefusal::at(Stage::Fuse,format!("`{name}` is empty: {why}, so there is nothing to write"))
}

/// Why solid `body`, built empty, holds nothing. A body built from swept material was told so by
/// `combine`; a static one has its operands combined again one statement at a time until one
/// leaves no material — only once the whole is known empty, so a solid that builds pays nothing.
fn emptied(sk: &Sketch,body: usize) -> ExportRefusal {
    if matches!(sk.solids[body].def,crate::model::SolidDef::Body {..}) && !cad::contains_sweep(sk,body) {
        match combine(sk,body,&mut |o| built_static(sk,o)) {
            Ok((_,Some(why))) => return empty(&sk.solids[body].name,&why),
            Err(e) => return e,
            Ok((_,None)) => {}
        }
    }
    empty(&sk.solids[body].name,"it leaves no material")
}

/// The STEP file of a built solid, parsed back and held to it (every reference, the topology's
/// counts, units, and each face's surface and its numbers), with what the check counted.
pub fn step(exact: &Exact,name: &str,tolerance: Option<Tolerance>,say: &Say) -> Result<String,ExportRefusal> {
    let started = crate::clock::Instant::now();
    let text = super::step::write(&exact.solid,name,tolerance.map_or(1e-4,|t| t.deflection()*0.1)).at(Stage::Step)?;
    let verified = super::step_check::verify(&text,&super::step_check::Solid::of(&exact.solid))
        .map_err(|e| ExportRefusal::at(Stage::Step,format!("the STEP file does not describe the solid: {e}")))?;
    (say.stage)(&format!("the STEP output: {} entities; {} faces, {} edges and {} vertices, each the solid's ({:?})",
        verified.entities,verified.faces,verified.edges,verified.vertices,started.elapsed()));
    (say.mark)(Stage::Step);
    Ok(text)
}

/// **The objects `objects` of the sketch as an assembly** (`step::Assembly`), `name` its root
/// product's: a part of each object, a sub-assembly of each step of their names as a reader writes
/// them (`wheel.hub` is `hub` in `wheel`, placed where the root's axes are; `p[0].body` a ring's
/// copy's), and the objects of one binding one part, built once — its representative's
/// (`Sketch::bound`), named for the first object it is — and used where each instance stands. A
/// solid a closed instance made is written in its instance's frame (`Sketch::binding_pose`) and
/// used placed by that frame; a mirrored one is its own part, its reflection in its solid and its
/// placement proper; any other is written where it stands, and used at the root's axes. Each part
/// built is given to `built`, as the world has it. A refusal says which solid's build refused.
pub fn assembly(sk: &Sketch,objects: &[usize],name: &str,tolerance: Option<Tolerance>,say: &Say,built: &mut dyn FnMut(usize,&Exact))
    -> Result<super::step::Assembly,(usize,ExportRefusal)> {
    use super::step::{Assembly,Product,Usage};
    use crate::space::{dot,norm};
    let mm = cad::millimetres(sk);
    // the frame a solid's instance stands in (millimetres), where it has one a placement can say:
    // `u` and `v` unit and square, so `u × v` is too
    let frame = |i: usize| sk.solids[i].binding.as_ref().and(sk.binding_pose(i))
        .filter(|b| (norm(b.u)-1.).abs() <= 1e-9 && (norm(b.v)-1.).abs() <= 1e-9 && dot(b.u,b.v).abs() <= 1e-9)
        .map(|b| Frame {o:b.o.map(|x| x*mm),x:b.u,y:b.v,z:b.normal()});
    // each object's part — the solid it is built as: an unmirrored object of a binding its
    // representative's, whose build as the world has it is the binding's own evaluation unless
    // the representative is mirrored; then the first such object's
    let reps: Vec<Option<usize>> = objects.iter().map(|&i| sk.bound(i)
        .filter(|b| b.mirror.is_none() && b.rep != i && frame(i).is_some() && frame(b.rep).is_some()).map(|b| b.rep)).collect();
    let unmirrored = |j: usize| sk.solids[j].binding.as_ref().is_none_or(|b| b.mirror.is_none());
    let source = |k: usize| match reps[k] {
        Some(r) if unmirrored(r) => r,
        Some(r) => objects[reps.iter().position(|&x| x == Some(r)).unwrap_or(k)],
        None => objects[k],
    };
    // what the source calls each solid, as a reader writes it (`p[0].body`, a ring's copy)
    let named = |i: usize| crate::vocab::public_path(&sk.solids[i].name);
    let mut products = vec![Product {name:name.to_string(),solid:None}];
    let mut uses = Vec::new();
    // the sub-assemblies first, each step of a name after the one it is in
    let mut groups: Vec<String> = Vec::new();
    let group = |groups: &[String],path: &str| groups.iter().position(|g| g == path).map_or(0,|g| g+1);
    for &i in objects {
        let name = named(i);
        let steps: Vec<&str> = name.split('.').collect();
        for k in 1..steps.len() {
            let path = steps[..k].join(".");
            if groups.contains(&path) { continue }
            let parent = group(&groups,&steps[..k-1].join("."));
            groups.push(path.clone());
            uses.push(Usage {parent,child:products.len(),name:path.clone(),placement:Frame::WORLD});
            products.push(Product {name:path,solid:None});
        }
    }
    // then the parts, each built once, and each object a use of its part in its group
    let mut parts: std::collections::BTreeMap<usize,usize> = Default::default();
    for (k,&i) in objects.iter().enumerate() {
        let source = source(k);
        let part = match parts.get(&source) {
            Some(&p) => p,
            None => {
                let x = exact(sk,source,None,tolerance,say).map_err(|e| (source,e))?;
                built(source,&x);
                // about its instance's frame where it has one, and named for the first object it
                // is, which the file holds where the source may not
                let solid = match frame(source) { Some(f) => x.solid.moved(&f.unplacing()),None => x.solid };
                products.push(Product {name:named(i),solid:Some(solid)});
                parts.insert(source,products.len()-1);
                products.len()-1
            }
        };
        let name = named(i);
        let parent = name.rsplit_once('.').map_or(0,|(g,_)| group(&groups,g));
        uses.push(Usage {parent,child:part,name,placement:frame(i).unwrap_or(Frame::WORLD)});
    }
    (say.stage)(&format!("the assembly: {} parts, {} sub-assemblies, {} objects placed",parts.len(),groups.len(),objects.len()));
    Ok(Assembly {products,uses})
}

/// The STEP file of an assembly, parsed back and held to it as `step` holds a solid's: each part's
/// solid, and every use of a product where the assembly places it (`step_check::verify_assembly`).
pub fn assembly_step(a: &super::step::Assembly,name: &str,tolerance: Option<Tolerance>,say: &Say) -> Result<String,ExportRefusal> {
    let started = crate::clock::Instant::now();
    let text = super::step::write_assembly(a,name,tolerance.map_or(1e-4,|t| t.deflection()*0.1)).at(Stage::Step)?;
    let verified = super::step_check::verify_assembly(&text,&super::step_check::Structure::of(a))
        .map_err(|e| ExportRefusal::at(Stage::Step,format!("the STEP file does not describe the assembly: {e}")))?;
    (say.stage)(&format!("the STEP output: {} entities; {} products and {} uses, each the assembly's; {} faces, {} edges and {} vertices, \
        each a part's ({:?})",verified.entities,a.products.len(),a.uses.len(),verified.faces,verified.edges,verified.vertices,started.elapsed()));
    (say.mark)(Stage::Step);
    Ok(text)
}

/// The binary STL of a built solid: meshed within the tolerance's deflection (a sector's mesh
/// turned into its copies), no triangle turned against its surface, its shells closed — and a
/// swept body's held to its material field, refused where they disagree.
pub fn stl(sk: &Sketch,body: usize,exact: &Exact,tolerance: Option<Tolerance>,say: &Say) -> Result<Vec<u8>,ExportRefusal> {
    let started = crate::clock::Instant::now();
    let bar = tolerance.map_or(GROSS,|t| t.deflection());
    // meshed within what float32 coordinates leave of the bar (never finer than an eighth of it),
    // so the written file is within it; refused at once where the encoding alone passes it
    let (lo,hi) = exact.solid.bounds();
    let rounding = crate::mesh::f32_rounding_within(lo,hi);
    if rounding >= bar {
        return Err(ExportRefusal::at(Stage::Stl,format!("float32 STL coordinates may move the surface {:.3} µm at this position, \
            which passes {:.3} µm; move the solid nearer the origin or ask a coarser tolerance",rounding*1e3,bar*1e3)))
    }
    let sag = (bar-rounding).max(bar/8.);
    let m = match &exact.pattern { Some(p) => p.mesh(sag,ANGULAR),None => super::mesh::mesh(&exact.solid,sag,ANGULAR) }.at(Stage::Mesh)?;
    if m.turned > 0 { return Err(ExportRefusal::at(Stage::Mesh,format!("the mesh has {} triangles facing against their surfaces",m.turned))) }
    if m.sag > bar { return Err(ExportRefusal::at(Stage::Mesh,format!("the mesh sags {:.3} µm against {:.3} µm",m.sag*1e3,bar*1e3))) }
    let bytes = crate::mesh::stl_of(&m.triangles(),&sk.solids[body].name);
    let rounding = written_within(&bytes,m.sag,bar)?;
    crate::mesh::stl_shells(&bytes).at(Stage::Stl)?;
    (say.stage)(&format!("the STL output: {} triangles, sagging {:.3} µm at most and moved {:.3} µm at most by float32, against \
        {:.3} µm, its shells checked ({:?})",m.tris.len(),m.sag*1e3,rounding*1e3,bar*1e3,started.elapsed()));
    (say.mark)(Stage::Stl);
    if exact.swept { field_agreement(sk,body,&bytes,tolerance,exact.indexed,say)?; }
    Ok(bytes)
}

/// How far float32 encoding may have moved the written STL (`mesh::stl_rounding`), refused where
/// that and the mesh's `sag` together pass the STL's share of the tolerance (`bar`): the bar is
/// about the written surface, and binary STL cannot place a vertex far from the origin finely.
pub fn written_within(bytes: &[u8],sag: f64,bar: f64) -> Result<f64,ExportRefusal> {
    let rounding = crate::mesh::stl_rounding(bytes).at(Stage::Stl)?;
    if sag+rounding > bar {
        return Err(ExportRefusal::at(Stage::Stl,format!("float32 STL coordinates may move the surface {:.3} µm at this \
            position, which with the mesh's sag of {:.3} µm passes {:.3} µm; move the solid nearer the origin or ask a coarser \
            tolerance",rounding*1e3,sag*1e3,bar*1e3)))
    }
    Ok(rounding)
}

/// A swept body's STL held to its material field: no cluster of microscopic triangles, and every
/// probe a little off each side of the mesh read on the side the mesh says (an indexed body's turned
/// into one sector and read by the cuts that reach it).
pub fn field_agreement(sk: &Sketch,body: usize,stl: &[u8],tolerance: Option<Tolerance>,indexed: Option<([f64;3],[f64;3],usize)>,say: &Say)
    -> Result<(),ExportRefusal> {
    let mesh = |message: String| ExportRefusal::at(Stage::Mesh,message);
    let refused = |message: String| ExportRefusal::at(Stage::Agreement,message);
    let scale = cad::millimetres(sk);
    let started = crate::clock::Instant::now();
    let (vertices,triangles) = agreement::stl_triangles(stl,scale).map_err(mesh)?;
    let under = tolerance.map_or(contracts::TINY_AREA,|t| (t.deflection()/10.).dpowi(2));
    let tiny = contracts::tiny_triangles_under(&vertices,&triangles,scale,under);
    (say.stage)(&format!("mesh: {} of {} triangles under {:.1e} mm², at most {} in a millimetre cube",tiny.count,tiny.total,
        tiny.under,tiny.densest));
    tiny.clustered().map_err(mesh)?;
    (say.mark)(Stage::Mesh);
    let field = MaterialField::read(sk,body,cad::AXIS_TOLERANCE).map_err(refused)?;
    let [offset,confirm,value_tolerance] = tolerance.map_or([0.1,0.025,0.02],|t| t.probe());
    let options = agreement::Options {offset:offset/scale,confirm:confirm/scale,value_tolerance:value_tolerance/scale,..Default::default()};
    let (report,how) = match indexed {
        Some((origin,axis,count)) => {
            let at = agreement::Indexed {origin:origin.map(|x| x/scale),axis,count};
            let (report,folding) = agreement::of_triangles_indexed(&vertices,&triangles,&field,at,cad::POSE_CACHE,&options).map_err(refused)?;
            (report,if folding.alike {
                format!(", each turned into one sector and read there by {} of its {} cuts (the rest proved clear of {} boxes about it; \
                    {} probes outside them read where they stand)",folding.kept,folding.operands,folding.cells,folding.whole)
            } else { ", each where it stands (the field did not read alike turned by a pitch)".into() })
        }
        None => (agreement::of_triangles_parallel(&vertices,&triangles,&field,cad::POSE_CACHE,&options).map_err(refused)?,String::new()),
    };
    let off = if tolerance.is_some() { format!("{:.4}",options.offset*scale) } else { format!("{:.2}",options.offset*scale) };
    (say.stage)(&format!("field agreement: {} of {} triangles probed {off} mm off each side{how}, {} probes unresolved, \
        {} withdrawn beside another face, {} disagree ({:?})",report.probed_triangles,report.triangles,
        report.unresolved,report.withdrawn,report.disagreements.len(),started.elapsed()));
    if report.agrees() { (say.mark)(Stage::Agreement); return Ok(()) }
    Err(ExportRefusal {stage:Stage::Agreement,condition:None,witness:report.disagreements.first().map(|d| d.point),
        message:format!("`{}`: the exported surface disagrees with the material field at {} of {} probes; nothing was written",
        sk.solids[body].name,report.disagreements.len(),report.probes)})
}

/// A swept body's exact surface as the page draws it: its patterned (or whole) B-rep meshed at the
/// solid's own mesh unit, in model units and world coordinates, each triangle with the B-rep face
/// it is of (`of`, the sector's for a pattern, so every copy of a face is one face), whether each
/// face is curved, and the solid's exact volume (model units cubed).
#[derive(Clone,Debug,Default)]
pub struct Display { pub vertices: Vec<[f64;3]>,pub triangles: Vec<[u32;3]>,pub of: Vec<u32>,pub smooth: Vec<bool>,pub volume: f64 }

/// Where a swept body's exact build stands for its display, a stage at a time (`Builder::step`).
enum State {
    Admit,
    Prepare(admission::Admission),
    Sheets(super::sweep::Prepared,Vec<super::sweep::sheet::Fitted>),
    Plan(super::sweep::Prepared,Vec<super::sweep::sheet::Fitted>),
    Cut(super::sweep::Prepared,Vec<super::sweep::sheet::Fitted>,super::sweep::Plan),
    Finish(super::sweep::Cut),
    Compose(super::sweep::Built),
    Mesh(super::sweep::Built),
    Done(Display),
    Failed,
}

/// **A swept body's exact surface, built a stage at a time** — admitted, its blank, each sweep's
/// sheet, its sector's premises, the cut, the pattern, the mesh — so a host that cannot hear from
/// a call while it runs (a page's worker: the core's module has no imports) can say between stages
/// what it is doing, and stop between them for a newer drawing. The stages are `export::exact`'s,
/// without a file written or the field's agreement asked: what it builds is for looking at.
pub struct Builder { sk: Sketch,body: usize,recipe: cad::StaticRecipe,state: State,lines: std::sync::Mutex<Vec<String>>,done: usize,
    /// The body built from the swept body, when `body` is that (`holds_sweep`): the stages build
    /// the swept body, and a last one combines it with the rest by the body rule.
    outer: Option<usize>,
    /// The operand of `outer` the swept body stands for: `body`, or a placement of it.
    operand: usize,
    /// Where `body` is placed (#619): the motion its build is moved by (mm), and the one the
    /// combined body is moved by after, where a placement places that.
    moved: [Option<super::geom::Rigid>;2] }

impl Builder {
    /// The build of swept solid `body` of `sk` (a copy of the sketch is kept), or why it has none:
    /// a solid with no swept cut is the static path's.
    pub fn new(sk: &Sketch,body: usize) -> Result<Builder,ExportRefusal> {
        if body >= sk.solids.len() { return Err(ExportRefusal::at(Stage::Blank,"no such solid in this drawing")) }
        // a placement of a swept body (`x in f`): the body it places, built and moved (#619)
        let placed = |i: usize| -> Result<(usize,Option<super::geom::Rigid>),ExportRefusal> {
            Ok(cad::placed_swept(sk,i).at(Stage::Blank)?
                .map_or((i,None),|(inner,pose)| (inner,Some(super::sweep::sector::placed(pose,cad::millimetres(sk))))))
        };
        let (body,whole) = placed(body)?;
        // a body built from swept material: its one operand holding sweeps built by the stages
        let swept = cad::swept_operands(sk,body);
        let (body,outer,operand,moved) = if !swept.is_empty() {
            let [operand] = swept[..] else {
                return Err(ExportRefusal::at(Stage::Blank,format!("`{}` is built from {} swept bodies; its display is built from one",
                    sk.solids[body].name,swept.len())))
            };
            let (inner,moved) = placed(operand)?;
            (inner,Some(body),operand,[moved,whole])
        } else { (body,None,body,[whole,None]) };
        let recipe = cad::recipe_static(sk,body).at(Stage::Blank)?;
        if recipe.sweeps.is_empty() {
            return Err(ExportRefusal::at(Stage::Blank,format!("`{}` cuts no sweep",sk.solids[body].name)))
        }
        Ok(Builder {sk:sk.clone(),body,recipe,state:State::Admit,lines:Default::default(),done:0,outer,operand,moved})
    }
    /// Run the next stage: `Ok(true)` once the surface is built, or the refusal that stopped it.
    pub fn step(&mut self) -> Result<bool,ExportRefusal> {
        let lines = &self.lines;
        let say = Say {stage:&|line: &str| lines.lock().unwrap_or_else(|e| e.into_inner()).push(line.to_string()),mark:&|_| {}};
        let (sk,body,recipe) = (&self.sk,self.body,&self.recipe);
        let state = std::mem::replace(&mut self.state,State::Failed);
        let next = (|| -> Result<State,ExportRefusal> { Ok(match state {
            State::Admit => State::Prepare(admission::admit_body(sk,body,&DISPLAY_ADMISSION)?),
            State::Prepare(admitted) => {
                let prepared = super::sweep::prepare(sk,body,recipe,&admitted,&say)?;
                // a planar sweep's body is its blank within its envelope: no sheet, no sector
                if super::sweep::planar::all(&prepared) {
                    State::Finish(super::sweep::Cut::Whole(super::sweep::planar::cut(sk,body,recipe,&prepared,None,&say)?))
                } else if super::sweep::subtraction::all(&prepared) {
                    State::Finish(super::sweep::Cut::Whole(super::sweep::subtraction::cut(sk,body,recipe,&prepared,None,&say)?))
                } else { State::Sheets(prepared,Vec::new()) }
            }
            State::Sheets(prepared,mut sheets) => {
                sheets.push(super::sweep::sheet(&prepared,sheets.len(),None,&say)?);
                if sheets.len() == prepared.sweeps() { State::Plan(prepared,sheets) } else { State::Sheets(prepared,sheets) }
            }
            State::Plan(prepared,sheets) => { let plan = super::sweep::plan(sk,body,recipe,&prepared,&sheets,&say); State::Cut(prepared,sheets,plan) }
            State::Cut(prepared,sheets,plan) => State::Finish(super::sweep::cut(sk,body,recipe,&prepared,&sheets,plan,&say)?),
            State::Finish(cut) => {
                let mut built = super::sweep::finish(cut,&say)?;
                if let Some(m) = &self.moved[0] { built = built.moved(m); }
                if self.outer.is_some() { State::Compose(built) } else { State::Mesh(built) }
            }
            State::Compose(built) => {
                let outer = self.outer.expect("a composed build has its body");
                let mut inner = Some(built.into_solid());
                let (solid,emptied) = combine(sk,outer,&mut |o| if o == self.operand {
                    inner.take().ok_or_else(|| ExportRefusal::at(Stage::Fuse,"the swept body is used twice")) } else { built_static(sk,o) })?;
                if let Some(why) = emptied { return Err(empty(&sk.solids[outer].name,&why)) }
                (say.stage)(&format!("`{}` combined from its swept material",sk.solids[outer].name));
                State::Mesh(super::sweep::Built::Whole(match &self.moved[1] { Some(m) => solid.moved(m),None => solid }))
            }
            State::Mesh(built) => State::Done(display(sk,&built)?),
            done @ State::Done(_) => done,
            State::Failed => return Err(ExportRefusal::at(Stage::Blank,"the build has failed already")),
        }) })();
        match next {
            Ok(state) => { self.done += 1; self.state = state; Ok(matches!(self.state,State::Done(_))) }
            Err(refusal) => Err(refusal),
        }
    }
    /// What the next stage does, in a page's words.
    pub fn doing(&self) -> String {
        let name = |k: usize,p: &super::sweep::Prepared| format!("fitting the sheet of `{}` ({} of {})",p.sweep_name(&self.sk,k),k+1,p.sweeps());
        match &self.state {
            State::Admit => "admitting its sweeps to the generating class".into(),
            State::Prepare(_) => "building the blank".into(),
            State::Sheets(p,sheets) => name(sheets.len(),p),
            State::Plan(..) => "laying out the sector".into(),
            State::Cut(_,_,super::sweep::Plan::Sector(_)) => "cutting the sector by its sheets".into(),
            State::Cut(_,_,super::sweep::Plan::Whole(_)) => "cutting the blank by its sheets".into(),
            State::Finish(_) => "turning the sector round".into(),
            State::Compose(_) => "adding the rest of the body".into(),
            State::Mesh(_) => "meshing the exact surface".into(),
            State::Done(_) => "built".into(),
            State::Failed => "failed".into(),
        }
    }
    /// The stages run, and their number in all: admission, the blank, a sheet a distinct sweep, the
    /// plan, the cut, the pattern and the mesh.
    pub fn stages(&self) -> (usize,usize) {
        let mut sweeps: Vec<usize> = self.recipe.sweeps.iter().map(|s| s.swept).collect();
        sweeps.sort(); sweeps.dedup();
        (self.done,6+sweeps.len()+usize::from(self.outer.is_some()))
    }
    /// The last thing a stage said (its own words: what it built, and in what time).
    pub fn said(&self) -> Option<String> { self.lines.lock().unwrap_or_else(|e| e.into_inner()).last().cloned() }
    /// The surface, once built.
    pub fn display(&self) -> Option<&Display> { match &self.state { State::Done(d) => Some(d),_ => None } }
}

/// Every swept object of `sk` given its exact B-rep where this kernel builds one (`exact`, at the
/// gross bars), so that what reads the object's surface — a drawing's views and sections, its
/// silhouettes traced on the surfaces — reads the B-rep, as it reads a static solid's; a field is
/// meshed only for an object whose build is refused.
pub fn supply_exact(sk: &Sketch) { supply_each(sk,sk.field_jobs().into_iter().map(|j| j.solid)) }

/// Give the drawing `body`'s exact B-rep as an export `built` it, where it holds none yet: what a
/// host reads of the body after, a picture, then reads that build rather than building it again
/// or meshing its field (#592).  A swept body's is about the world's origin, as `supply_exact`
/// gives it; a static one's about its own (`solid::exact::about_origin`), and none is given for a
/// solid of a binding, which its representative's build places, or one reaching a mirrored solid.
/// The B-rep is the export's: the same solid as the drawing's own build, its faces named by its
/// recipe, so a host gives it to what only looks at the solid.
pub fn supply(sk: &Sketch,body: usize,built: &Exact) {
    if sk.supplied_exact_solid(body).is_some() { return }
    if built.swept {
        let mm = cad::millimetres(sk);
        sk.supply_exact_solid(body,crate::solid::exact::named(built.solid.clone(),mm,[0.;3],Default::default()));
    } else if sk.solids[body].binding.is_none() && !mirrored_within(sk,body) {
        sk.supply_exact_solid(body,crate::solid::exact::about_origin(sk,body,&built.solid));
    }
}

/// `supply_exact` for the swept solids `wanted` takes (by solid index), objects or not: a report
/// asked of one solid builds no other, and one asked of a body other solids are made from builds
/// it too — the wheel a frame stands (`x in f`) as well as the wheel standing, so the two read one
/// B-rep, moved, and neither a field (#619).
pub fn supply_exact_where(sk: &Sketch,wanted: &dyn Fn(usize) -> bool) {
    supply_each(sk,(0..sk.solids.len()).filter(|&i| sk.is_swept(i) && wanted(i)))
}

/// Swept solid `i` given its exact B-rep where this kernel builds one and it holds none yet: what a
/// claim about it measures (#615), as a drawing reads it. A solid with no sweep reads its own build
/// already, and a refused build leaves its field to be meshed.
pub fn supply_exact_of(sk: &Sketch,i: usize) {
    supply_each(sk,std::iter::once(i).filter(|&i| i < sk.solids.len() && sk.is_swept(i)))
}

/// Each of the swept `solids` given its exact B-rep where this kernel builds one and it holds none.
/// One built with no face is given too: what the drawing reads of it is a body with no material,
/// its volume 0 (#618), where only a file of it is refused as empty (`exact`, #617).
fn supply_each(sk: &Sketch,solids: impl Iterator<Item=usize>) {
    // a sketch that leaves meshing to its host has its surfaces supplied by it
    if sk.field_meshing.get() != crate::solid::FieldMeshing::Now { return }
    for i in solids { supply_one(sk,i) }
}

/// Swept solid `i` given its exact B-rep where it holds none (`supply_each`).
fn supply_one(sk: &Sketch,i: usize) {
    if sk.supplied_exact_solid(i).is_some() { return }
    let mm = cad::millimetres(sk);
    // a body placed by a rigid motion is the body it places, supplied once and moved (#619): a
    // claim along a roll places a wheel at each pose, and each is that one build, not its own.
    // A supplied B-rep stands where it was built, about no origin, so the pose moves it whole
    if let Ok(Some((inner,pose))) = cad::placed_swept(sk,i) {
        if sk.is_swept(inner) { supply_one(sk,inner) }
        if let Some(x) = sk.supplied_exact_solid(inner) {
            let brep = x.brep.moved(&super::sweep::sector::placed(pose,mm));
            sk.supply_exact_solid(i,crate::solid::Exact {brep,mm,origin:[0.;3],leading:x.leading.clone(),named:x.named.clone()});
            return
        }
    }
    // admitted as a display is, which is looked at and not made
    let say = Say {stage:&|_: &str| {},mark:&|_| {}};
    let admitted = cad::swept_operands(sk,i).is_empty().then(|| admission::admit_body(sk,i,&DISPLAY_ADMISSION).ok()).flatten();
    let Ok(built) = build(sk,i,admitted.as_ref(),None,&say) else { return };
    sk.supply_exact_solid(i,crate::solid::exact::named(built.solid,mm,[0.;3],Default::default()));
}

/// The admission a display asks: a quarter of the export's samples each way, on both its grids. A
/// display is looked at, not made: a design the export's sampling would refuse is still refused
/// there, and the build's own stages refuse what they cannot construct. On a gear member it is a
/// second in the browser's core where the export's took six.
const DISPLAY_ADMISSION: admission::Options = admission::Options {rows:25,columns:100,coarse_rows:6,coarse_columns:360,
    axis_tolerance:cad::AXIS_TOLERANCE,margin:1e-6,root_tolerance:1e-9,least_factor:1e-3};

/// How far a display's mesh may sag, as a fraction of the solid's diagonal (a gear's 0.2 mm), and
/// how far its chords and facets may turn (radians): a gear's half-millimetre fillets are what an
/// export's 0.2 radians cut finest, and what a screen shows least. A gear member is then 17–26
/// thousand triangles, as many as its field's preview: at 0.05 mm and 0.5 rad it was 39–63
/// thousand, and a page panning and zooming the pair could not keep up.
const DISPLAY_SAG: f64 = 2e-3;
const DISPLAY_ANGULAR: f64 = 0.8;

/// A built swept body meshed for display, within `DISPLAY_SAG` of its size.
fn display(sk: &Sketch,built: &super::sweep::Built) -> Result<Display,ExportRefusal> {
    let mm = cad::millimetres(sk);
    let size = match built { super::sweep::Built::Sector {pattern,..} => pattern.solid.size(),super::sweep::Built::Whole(solid) => solid.size() };
    let bar = DISPLAY_SAG*size;
    // (a pattern's volume is its sector's times its copies, which are turns of it: measured whole,
    // a gear's 291 faces took seconds where its sector's 11 take a few hundredths)
    let (m,faces,volume) = match built {
        super::sweep::Built::Sector {sector,pattern} => (pattern.mesh(bar,DISPLAY_ANGULAR).at(Stage::Mesh)?,&pattern.sector().faces,
            super::props::volume(&sector.piece)*sector.count as f64),
        super::sweep::Built::Whole(solid) => (super::mesh::mesh(solid,bar,DISPLAY_ANGULAR).at(Stage::Mesh)?,&solid.faces,
            super::props::volume(solid)),
    };
    Ok(Display {
        vertices: m.pts.iter().map(|p| p.map(|x| x/mm)).collect(),
        triangles: m.tris.clone(),
        of: m.of.clone(),
        smooth: faces.iter().map(|f| !matches!(f.surface,super::geom::Surface::Plane(_))).collect(),
        volume: volume/(mm*mm*mm),
    })
}
