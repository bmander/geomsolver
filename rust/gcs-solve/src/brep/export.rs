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
    // a sweep under a joint driven by a law is its joint's over the law's range (#684 L): built from
    // the copy of the drawing that writes it so, and held to the driven sweep's field
    if let Some(lowered) = crate::solid::paced::lowered(sk,body) {
        (say.stage)(&format!("`{}` cuts a sweep under a joint driven by a law: the joint's sweep over the law's range cut",
            sk.solids[body].name));
        return Ok(Exact {swept:true,..own(&lowered,body,admitted,mirrored,tolerance,say)?})
    }
    // a cut of a union's sweep is a cut of each member's (#684 C): built from the copy of the
    // drawing that writes it so, and held to the union's field as a swept body (where every
    // member's sweep is a ball's tube, the copy's body is static material)
    if let Some(lowered) = crate::solid::distributed::lowered(sk,body) {
        (say.stage)(&format!("`{}` cuts the sweep of a union: each member's sweep cut",sk.solids[body].name));
        return Ok(Exact {swept:true,..own(&lowered,body,admitted,mirrored,tolerance,say)?})
    }
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
    no_swept_cut(sk,body)?;
    let solid = &sk.solids[body];
    let crate::model::SolidDef::Body {stock,on,through,bound} = &solid.def else { unreachable!("a composed solid is a body") };
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

/// Body `body` of swept material refused where a sweep is cut from it directly (`combine`).
fn no_swept_cut(sk: &Sketch,body: usize) -> Result<(),ExportRefusal> {
    let solid = &sk.solids[body];
    let crate::model::SolidDef::Body {through,..} = &solid.def else { unreachable!("a composed solid is a body") };
    match through.iter().find(|&&c| cad::contains_sweep(sk,c as usize)) {
        Some(&c) => Err(ExportRefusal::at(Stage::Blank,format!("`{}` cuts the sweep `{}` from swept material: name the body the sweep is \
            cut from, and build `{}` from it",solid.name,sk.solids[c as usize].name,solid.name))),
        None => Ok(()),
    }
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
/// built is given to `built`, as the world has it, and coloured `colour` of the first object it is
/// built for (by its solid's index; the objects of one part are one colour). A refusal says which
/// solid's build refused.
pub fn assembly(sk: &Sketch,objects: &[usize],name: &str,tolerance: Option<Tolerance>,colour: &dyn Fn(usize) -> Option<[f64;3]>,
    say: &Say,built: &mut dyn FnMut(usize,&Exact)) -> Result<super::step::Assembly,(usize,ExportRefusal)> {
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
    let mut products = vec![Product {name:name.to_string(),solid:None,colour:None}];
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
            products.push(Product {name:path,solid:None,colour:None});
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
                products.push(Product {name:named(i),solid:Some(solid),colour:colour(i)});
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

/// **An object's exact surface as the page draws it** (#670): its B-rep (a pattern's patterned, a
/// swept body's whole, a static body's as its recipe builds it) meshed at the solid's own mesh
/// unit, in model units and world coordinates; what the mesh says of itself (`faces`: each
/// triangle's B-rep face — the sector's for a pattern, so every copy of a face is one face —
/// whether each face is curved, each face's name as `solid::exact::named` writes it, the B-rep's
/// edges as polylines through the vertices and the solid's exact volume, model units cubed); and
/// what the solid read when it was built (`solid::reads`), which a sketch it is supplied to holds
/// it current against (`Sketch::supply_object`).
#[derive(Clone,Debug,Default)]
pub struct Display { pub vertices: Vec<[f64;3]>,pub triangles: Vec<[u32;3]>,pub faces: crate::solid::ExactFaces,pub reads: Vec<f64> }

/// What `Display::bytes` writes first: the format's number, which `Display::read` holds a buffer to.
const DISPLAY_FORMAT: f64 = 2.;
/// How many doubles head the buffer.
const DISPLAY_HEAD: usize = 9;

impl Display {
    /// **The surface as one buffer** (little-endian), the form it crosses between cores and is kept
    /// in by a host: nine doubles — the format, the vertices, triangles, faces and edges counted,
    /// the volume, the names' bytes, the edges' words and the numbers read — then three doubles a
    /// vertex and the numbers read; then 32-bit words: three a triangle, a face a triangle, 0 or 1
    /// a face (curved), and each edge as its points counted, its two faces, 0 or 1 (smooth) and
    /// its points; then the faces' names in UTF-8, one a line.
    pub fn bytes(&self) -> Vec<u8> {
        let f = &self.faces;
        let names = f.names.join("\n");
        let words: usize = f.edges.iter().map(|e| 4+e.points.len()).sum();
        let mut out = Vec::with_capacity(8*(DISPLAY_HEAD+3*self.vertices.len()+self.reads.len())
            +4*(4*self.triangles.len()+f.smooth.len()+words)+names.len());
        let head = [DISPLAY_FORMAT,self.vertices.len() as f64,self.triangles.len() as f64,f.smooth.len() as f64,
            f.edges.len() as f64,f.volume,names.len() as f64,words as f64,self.reads.len() as f64];
        for x in head.iter().chain(self.vertices.iter().flatten()).chain(&self.reads) { out.extend(x.to_le_bytes()); }
        let mut word = |w: u32| out.extend(w.to_le_bytes());
        for &i in self.triangles.iter().flatten() { word(i) }
        for &k in &f.of { word(k) }
        for &m in &f.smooth { word(m as u32) }
        for e in &f.edges {
            word(e.points.len() as u32); word(e.faces[0]); word(e.faces[1]); word(e.smooth as u32);
            for &p in &e.points { word(p) }
        }
        out.extend(names.as_bytes());
        out
    }

    /// The surface `bytes` wrote, or why `b` is none: a buffer crosses from another core or a
    /// host's store, so every count and index is held to what is there.
    pub fn read(b: &[u8]) -> Result<Display,String> {
        let bad = |what: &str| format!("an exact surface's buffer {what}");
        let double = |k: usize| -> Option<f64> { Some(f64::from_le_bytes(b.get(8*k..8*k+8)?.try_into().ok()?)) };
        let count = |k: usize| double(k).filter(|x| *x >= 0. && x.fract() == 0. && *x <= u32::MAX as f64).map(|x| x as usize);
        if double(0) != Some(DISPLAY_FORMAT) { return Err(bad("is of another format")) }
        let [nv,nt,nf,ne,nn,nw,nr] = [1,2,3,4,6,7,8].map(count);
        let (Some(nv),Some(nt),Some(nf),Some(ne),Some(nn),Some(nw),Some(nr)) = (nv,nt,nf,ne,nn,nw,nr) else {
            return Err(bad("counts what is no count")) };
        let volume = double(5).filter(|v| v.is_finite()).ok_or_else(|| bad("holds a volume that is no number"))?;
        let words = 3*nt+nt+nf+nw;
        let at = 8*(DISPLAY_HEAD+3*nv+nr);
        if b.len() != at+4*words+nn { return Err(bad("is not as long as it says")) }
        let vertices: Vec<[f64;3]> = (0..nv).map(|v| std::array::from_fn(|k| double(DISPLAY_HEAD+3*v+k).unwrap_or(f64::NAN))).collect();
        if vertices.iter().flatten().any(|x| !x.is_finite()) { return Err(bad("places a vertex nowhere")) }
        let reads: Vec<f64> = (0..nr).map(|k| double(DISPLAY_HEAD+3*nv+k).unwrap_or(f64::NAN)).collect();
        let w: Vec<u32> = b[at..at+4*words].chunks_exact(4).map(|c| u32::from_le_bytes([c[0],c[1],c[2],c[3]])).collect();
        let triangles: Vec<[u32;3]> = w[..3*nt].chunks_exact(3).map(|c| [c[0],c[1],c[2]]).collect();
        let of = w[3*nt..4*nt].to_vec();
        let smooth = w[4*nt..4*nt+nf].iter().map(|&m| m != 0).collect();
        let mut edges = Vec::with_capacity(ne);
        let mut k = 4*nt+nf;
        for _ in 0..ne {
            let n = *w.get(k).ok_or_else(|| bad("ends inside an edge"))? as usize;
            let e = w.get(k+1..k+4+n).ok_or_else(|| bad("ends inside an edge"))?;
            edges.push(crate::solid::ExactEdge {points:e[3..].to_vec(),faces:[e[0],e[1]],smooth:e[2] != 0});
            k += 4+n;
        }
        if k != words { return Err(bad("holds more edge words than its edges")) }
        let names = std::str::from_utf8(&b[at+4*words..]).map_err(|_| bad("names a face in what is no text"))?;
        let names: Vec<String> = if names.is_empty() && nf != 1 { Vec::new() } else { names.split('\n').map(str::to_string).collect() };
        let faces = crate::solid::ExactFaces {of,smooth,volume,names,edges};
        crate::solid::field_mesh::exact_checked(&vertices,&triangles,&faces).map_err(|e| bad(&e))?;
        Ok(Display {vertices,triangles,faces,reads})
    }

    /// The surface as a sketch is given it (`Sketch::supply_field`): finished, exact.
    pub fn surface(&self) -> crate::solid::FieldSurface { self.clone().into_surface().0 }

    /// `surface`, and what the solid read, moved out rather than copied.
    pub fn into_surface(self) -> (crate::solid::FieldSurface,Vec<f64>) {
        (crate::solid::FieldSurface {vertices:self.vertices,triangles:self.triangles,provisional:false,exact:Some(self.faces)},self.reads)
    }
}

/// Where a body's exact build stands, a stage at a time (`Builder::step`).
enum State {
    /// A body with no sweep: built from its recipe as the export builds it (`exact`), at once.
    Static,
    Admit,
    Prepare(admission::Admission),
    Sheets(super::sweep::Prepared,Vec<super::sweep::sheet::Fitted>),
    Plan(super::sweep::Prepared,Vec<super::sweep::sheet::Fitted>),
    Cut(super::sweep::Prepared,Vec<super::sweep::sheet::Fitted>,super::sweep::Plan),
    Finish(super::sweep::Cut),
    /// A body built from swept material: its operands that hold sweeps built in turn, each by a
    /// build of its own (`Build::parts`), then combined with the rest by the body rule.
    Parts,
    Compose,
    Mesh(super::sweep::Built),
    /// An operand's build finished, its B-rep moved where it stands, for the body built from it.
    Built(Brep),
    Done(Display),
    Failed,
}

/// **An object's exact surface, built a stage at a time** (#670) — a swept body admitted, its
/// blank, each sweep's sheet, its sector's premises, the cut, the pattern, the mesh; a static body
/// built from its recipe and meshed — so a host that cannot hear from a call while it runs (a
/// page's worker: the core's module has no imports) can say between stages what it is doing, and
/// stop between them for a newer drawing. Every object is built by it, one builder whatever the
/// body: the stages are `export::exact`'s, a static body's that one call, without a file written
/// or the field's agreement asked — what it builds is for looking at — and a refusal is the
/// export's, its stage and its words, never a field meshed in its place. A placement of a binding
/// is its representative's build moved: a host asks one build a binding (`Sketch::object_jobs`).
pub struct Builder { sk: Sketch,build: Build,lines: std::sync::Mutex<Vec<String>> }

/// One body's stages (`Builder`): the object's, or, where a body is built from swept material,
/// each of its operands that hold sweeps, as `compose` builds them — a compound of two shaped
/// gears, a body made of one made of a shaped blank — each to its B-rep, to any depth.
struct Build {
    /// The solid this build was asked for: the object, or an operand of the body built from it.
    asked: usize,
    /// The body its stages build: `asked`, or the swept body it places (`x in f`, #619).
    body: usize,
    recipe: cad::StaticRecipe,
    state: State,
    /// The stages this build has run itself, its parts' aside.
    done: usize,
    /// Where `asked` places the body built (mm), where it is a placement.
    moved: Option<super::geom::Rigid>,
    /// Whether its sweeps were all planar or subtractions, built within their envelopes: no
    /// sheet, plan or cut stage is run, nor counted.
    enveloped: bool,
    /// The builds of the operands of `body` that hold swept material, where it is built from it.
    parts: Vec<Build>,
    /// Whether the build ends at its B-rep (`State::Built`), for the body built from it, unmeshed.
    part: bool,
    /// What the solid asked for read (`solid::reads`), which its display carries.
    reads: Vec<f64>,
    /// The copy of the drawing its stages read where `body` cuts a union's sweep, the cut written
    /// as each member's (`solid::distributed`, #684 C); the builder's own drawing otherwise.
    lowered: Option<Sketch>,
}

impl Builder {
    /// The build of solid `body` of `sk` (a copy of the sketch is kept), or why it has none.
    pub fn new(sk: &Sketch,body: usize) -> Result<Builder,ExportRefusal> {
        if body >= sk.solids.len() { return Err(ExportRefusal::at(Stage::Blank,"no such solid in this drawing")) }
        let mut build = Build::new(sk,body,false)?;
        build.reads = crate::solid::reads(sk,body,0.);
        Ok(Builder {sk:sk.clone(),build,lines:Default::default()})
    }
    /// Run the next stage: `Ok(true)` once the surface is built, or the refusal that stopped it.
    pub fn step(&mut self) -> Result<bool,ExportRefusal> {
        let lines = &self.lines;
        let say = Say {stage:&|line: &str| lines.lock().unwrap_or_else(|e| e.into_inner()).push(line.to_string()),mark:&|_| {}};
        self.build.step(&self.sk,&say)?;
        Ok(matches!(self.build.state,State::Done(_)))
    }
    /// What the next stage does, in a page's words.
    pub fn doing(&self) -> String { self.build.doing(&self.sk) }
    /// The stages run, and their number in all: admission, the blank, a sheet a distinct sweep, the
    /// plan, the cut, the pattern and the mesh — a body whose sweeps are built within their
    /// envelopes runs no sheet, plan or cut, and counts none once that is known; a static body's
    /// build and its mesh; a body built from swept material, each of its swept operands' stages
    /// but the mesh, its combining and its mesh.
    pub fn stages(&self) -> (usize,usize) { self.build.stages() }
    /// The last thing a stage said (its own words: what it built, and in what time).
    pub fn said(&self) -> Option<String> { self.lines.lock().unwrap_or_else(|e| e.into_inner()).last().cloned() }
    /// The surface, once built.
    pub fn display(&self) -> Option<&Display> { match &self.build.state { State::Done(d) => Some(d),_ => None } }
}

impl Build {
    /// The build of solid `asked`: an object's (meshed), or a `part`'s (its B-rep).
    fn new(sk: &Sketch,asked: usize,part: bool) -> Result<Build,ExportRefusal> {
        let build = |body,state,recipe,moved,parts| Build {asked,body,recipe,state,done:0,moved,enveloped:false,parts,part,reads:Vec::new(),
            lowered:None};
        let none = || cad::StaticRecipe {recipe:crate::json::Json::Null,sweeps:Vec::new()};
        // a body no sweep reaches is built as the export builds it, a stage, then meshed
        if !cad::contains_sweep(sk,asked) { return Ok(build(asked,State::Static,none(),None,Vec::new())) }
        // a placement of a swept body (`x in f`): the body it places, built and moved (#619)
        let (body,moved) = cad::placed_swept(sk,asked).at(Stage::Blank)?
            .map_or((asked,None),|(inner,pose)| (inner,Some(super::sweep::sector::placed(pose,cad::millimetres(sk)))));
        // a body built from swept material: each of its operands holding sweeps built, then combined
        let swept = cad::swept_operands(sk,body);
        if !swept.is_empty() {
            no_swept_cut(sk,body)?;
            let parts = swept.into_iter().map(|o| Build::new(sk,o,true)).collect::<Result<Vec<_>,_>>()?;
            return Ok(build(body,State::Parts,none(),moved,parts))
        }
        // a sweep under a driven joint is its joint's over the law's range (#684 L), and a cut of a
        // union's sweep a cut of each member's (#684 C): the stages read the copy of the drawing
        // that writes it so, and where every member's sweep is a ball's tube the body is static
        // material there, built as the export builds it
        let paced = crate::solid::paced::lowered(sk,body);
        let lowered = crate::solid::distributed::lowered(paced.as_ref().unwrap_or(sk),body).or(paced);
        let state = match &lowered { Some(l) if !cad::contains_sweep(l,body) => State::Static,_ => State::Admit };
        let recipe = match state {
            State::Static => none(),
            _ => cad::recipe_static(lowered.as_ref().unwrap_or(sk),body).at(Stage::Blank)?,
        };
        if matches!(state,State::Admit) && recipe.sweeps.is_empty() {
            return Err(ExportRefusal::at(Stage::Blank,format!("`{}` cuts no sweep",sk.solids[body].name)))
        }
        // a static copy builds the solid asked, its placement as the export reads it
        let (body,moved) = if matches!(state,State::Static) { (asked,None) } else { (body,moved) };
        Ok(Build {lowered,..build(body,state,recipe,moved,Vec::new())})
    }
    /// Whether the build has come to its end: its display, or a part's B-rep.
    fn finished(&self) -> bool { matches!(self.state,State::Done(_) | State::Built(_)) }
    /// Run the next stage: a part's, where its parts are still being built.
    fn step(&mut self,sk: &Sketch,say: &Say) -> Result<(),ExportRefusal> {
        if let State::Parts = self.state {
            let part = self.parts.iter_mut().find(|p| !p.finished()).expect("a part left to build");
            part.step(sk,say)?;
            if self.parts.iter().all(Build::finished) { self.state = State::Compose; }
            return Ok(())
        }
        let state = std::mem::replace(&mut self.state,State::Failed);
        let was_prepare = matches!(state,State::Prepare(_));
        let lowered = self.lowered.take();
        let next = self.advance(lowered.as_ref().unwrap_or(sk),state,say);
        self.lowered = lowered;
        let next = next?;
        self.enveloped |= was_prepare && matches!(next,State::Finish(_));
        self.done += 1;
        self.state = next;
        Ok(())
    }
    /// The state after `state`'s stage.
    fn advance(&mut self,sk: &Sketch,state: State,say: &Say) -> Result<State,ExportRefusal> {
        let (body,recipe) = (self.body,&self.recipe);
        Ok(match state {
            State::Static => {
                let solid = exact(sk,body,None,None,say)?.solid;
                // (a part is static only where a union it cuts is distributed into balls' tubes)
                if self.part { State::Built(solid) } else { State::Mesh(super::sweep::Built::Whole(solid)) }
            }
            State::Admit => State::Prepare(admission::admit_body(sk,body,&DISPLAY_ADMISSION)?),
            State::Prepare(admitted) => {
                let prepared = super::sweep::prepare(sk,body,recipe,&admitted,say)?;
                // a planar sweep's body is its blank within its envelope: no sheet, no sector
                if super::sweep::planar::all(&prepared) {
                    State::Finish(super::sweep::Cut::Whole(super::sweep::planar::cut(sk,body,recipe,&prepared,None,say)?))
                } else if super::sweep::subtraction::all(&prepared) {
                    State::Finish(super::sweep::Cut::Whole(super::sweep::subtraction::cut(sk,body,recipe,&prepared,None,say)?))
                } else { State::Sheets(prepared,Vec::new()) }
            }
            State::Sheets(prepared,mut sheets) => {
                sheets.push(super::sweep::sheet(&prepared,sheets.len(),None,say)?);
                if sheets.len() == prepared.sweeps() { State::Plan(prepared,sheets) } else { State::Sheets(prepared,sheets) }
            }
            State::Plan(prepared,sheets) => { let plan = super::sweep::plan(sk,body,recipe,&prepared,&sheets,say); State::Cut(prepared,sheets,plan) }
            State::Cut(prepared,sheets,plan) => State::Finish(super::sweep::cut(sk,body,recipe,&prepared,&sheets,plan,say)?),
            State::Finish(cut) => {
                let mut built = super::sweep::finish(cut,say)?;
                if let Some(m) = &self.moved { built = built.moved(m); }
                if !self.part { State::Mesh(built) } else {
                    let solid = built.into_solid();
                    if solid.faces.is_empty() { return Err(emptied(sk,body)) }
                    State::Built(solid)
                }
            }
            State::Compose => {
                let mut parts: Vec<(usize,Option<Brep>)> = self.parts.iter_mut().map(|p| (p.asked,p.take())).collect();
                let (solid,emptied) = combine(sk,body,&mut |o| match parts.iter_mut().find(|(k,b)| *k == o && b.is_some()) {
                    Some((_,b)) => Ok(b.take().expect("a part's build")),
                    None if cad::contains_sweep(sk,o) => Err(ExportRefusal::at(Stage::Fuse,"the swept body is used twice")),
                    None => built_static(sk,o),
                })?;
                if let Some(why) = emptied { return Err(empty(&sk.solids[body].name,&why)) }
                (say.stage)(&format!("`{}` combined from its swept material",sk.solids[body].name));
                let solid = match &self.moved { Some(m) => solid.moved(m),None => solid };
                if self.part { State::Built(solid) } else { State::Mesh(super::sweep::Built::Whole(solid)) }
            }
            State::Mesh(built) => State::Done(display(sk,&built,self.reads.clone())?),
            State::Parts => unreachable!("a body's parts are stepped by `step`"),
            done @ (State::Done(_) | State::Built(_)) => done,
            State::Failed => return Err(ExportRefusal::at(Stage::Blank,"the build has failed already")),
        })
    }
    /// A finished part's B-rep, taken for the body built from it.
    fn take(&mut self) -> Option<Brep> {
        match std::mem::replace(&mut self.state,State::Failed) { State::Built(solid) => Some(solid),other => { self.state = other; None } }
    }
    /// What the next stage does, in a page's words.
    fn doing(&self,sk: &Sketch) -> String {
        let own = self.lowered.as_ref().unwrap_or(sk);
        let name = |k: usize,p: &super::sweep::Prepared| format!("fitting the sheet of `{}` ({} of {})",p.sweep_name(own,k),k+1,p.sweeps());
        match &self.state {
            State::Static => "building the solid from its recipe".into(),
            State::Admit => "admitting its sweeps to the generating class".into(),
            State::Prepare(_) => "building the blank".into(),
            State::Sheets(p,sheets) => name(sheets.len(),p),
            State::Plan(..) => "laying out the sector".into(),
            State::Cut(_,_,super::sweep::Plan::Sector(_)) => "cutting the sector by its sheets".into(),
            State::Cut(_,_,super::sweep::Plan::Whole(_)) => "cutting the blank by its sheets".into(),
            State::Finish(_) => "turning the sector round".into(),
            State::Parts => self.parts.iter().find(|p| !p.finished()).map_or_else(|| "adding the rest of the body".into(),|p| p.doing(sk)),
            State::Compose => "adding the rest of the body".into(),
            State::Mesh(_) => "meshing the exact surface".into(),
            State::Done(_) | State::Built(_) => "built".into(),
            State::Failed => "failed".into(),
        }
    }
    /// The stages run, and their number in all (`Builder::stages`).
    fn stages(&self) -> (usize,usize) {
        let mesh = usize::from(!self.part);
        if !self.parts.is_empty() {
            let (done,total) = self.parts.iter().map(Build::stages).fold((0,0),|(a,b),(c,d)| (a+c,b+d));
            return (self.done+done,total+1+mesh)
        }
        // (a static body's recipe lists no sweep: a swept one's always does)
        if self.recipe.sweeps.is_empty() { return (self.done,1+mesh) }
        if self.enveloped { return (self.done,3+mesh) }
        let mut sweeps: Vec<usize> = self.recipe.sweeps.iter().map(|s| s.swept).collect();
        sweeps.sort(); sweeps.dedup();
        (self.done,5+sweeps.len()+mesh)
    }
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

/// How far two faces' normals may turn across an edge for the edge to be drawn as smooth
/// (radians): a round's tangent edge meets its faces within a kernel's rounding of each other, and
/// a crease worth seeing turns a degree and more.
const SMOOTH_EDGE: f64 = 1e-2;

/// A built body meshed for display, within `DISPLAY_SAG` of its size: its faces named as
/// `solid::exact::named` names them, and its edges read off the mesh (`display_edges`).
fn display(sk: &Sketch,built: &super::sweep::Built,reads: Vec<f64>) -> Result<Display,ExportRefusal> {
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
    // where a face's normal points at `p` (mm) on the copy triangle `t` is of: a pattern's turned
    // back into its sector, read there, and turned out again
    let copies = match built { super::sweep::Built::Sector {sector,pattern} => Some((pattern,m.tris.len()/sector.count.max(1))),_ => None };
    let normal = |f: u32,t: usize,p: [f64;3]| -> Option<[f64;3]> {
        let face = faces.get(f as usize)?;
        let angle = copies.map(|(pattern,per)| pattern.step*(t/per.max(1)) as f64);
        let turn = |by: f64| copies.map(|(pattern,_)| pattern.turn(by));
        let q = match angle.and_then(|by| turn(-by)) { Some(back) => back.point(p),None => p };
        let n = face.outward(q)?;
        Some(match angle.and_then(turn) { Some(out) => out.vector(n),None => n })
    };
    let edges = display_edges(&m.pts,&m.tris,&m.of,&normal);
    Ok(Display {
        vertices: m.pts.iter().map(|p| p.map(|x| x/mm)).collect(),
        faces: crate::solid::ExactFaces {
            smooth: faces.iter().map(|f| !matches!(f.surface,super::geom::Surface::Plane(_))).collect(),
            volume: volume/(mm*mm*mm),
            names: faces.iter().map(|f| crate::solid::exact::joined(&f.name)).collect(),
            edges,
            of: m.of,
        },
        triangles: m.tris,
        reads,
    })
}

/// **The B-rep's edges, read off its mesh** (#670): the mesh shares each edge's samples between
/// the faces it bounds, so a side of the mesh between triangles of two faces is a stretch of an
/// edge, and the stretches of one pair of faces chained end to end are its polylines — within the
/// mesh's bars, as the faces are. Each is smooth where the faces' own normals (`normal`, of a face
/// as triangle `t` has it, at a point in mm) meet within `SMOOTH_EDGE` at its middle, sharp where
/// they turn more or either is not read.
fn display_edges(pts: &[[f64;3]],tris: &[[u32;3]],of: &[u32],normal: &dyn Fn(u32,usize,[f64;3]) -> Option<[f64;3]>)
    -> Vec<crate::solid::ExactEdge> {
    use std::collections::{BTreeMap,BTreeSet};
    let key = |a: u32,b: u32| (a.min(b),a.max(b));
    // each side of the mesh beside the triangle it is of, sorted so a side's triangles are together
    let mut sides: Vec<((u32,u32),usize)> = tris.iter().enumerate()
        .flat_map(|(t,tri)| (0..3).map(move |k| (key(tri[k],tri[(k+1)%3]),t))).collect();
    sides.sort_unstable();
    // the stretches between two faces, by the pair of faces, each with a triangle of either face
    let mut pairs: BTreeMap<(u32,u32),BTreeMap<(u32,u32),[usize;2]>> = BTreeMap::new();
    for run in sides.chunk_by(|x,y| x.0 == y.0) {
        let &[(side,s),(_,t)] = run else { continue };
        if of[s] == of[t] { continue }
        let (a,b) = if of[s] < of[t] { (s,t) } else { (t,s) };
        pairs.entry((of[a],of[b])).or_default().insert(side,[a,b]);
    }
    let mut out = Vec::new();
    for ((fa,fb),stretches) in pairs {
        let mut next: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
        for &(a,b) in stretches.keys() { next.entry(a).or_default().push(b); next.entry(b).or_default().push(a); }
        let mut used: BTreeSet<(u32,u32)> = BTreeSet::new();
        // a chain starts where it ends (a point not two stretches meet at), then what is left is loops
        let starts: Vec<u32> = next.iter().filter(|(_,n)| n.len() != 2).map(|(&v,_)| v).chain(next.keys().copied()).collect();
        for start in starts {
            for &first in &next[&start] {
                if used.contains(&key(start,first)) { continue }
                let mut line = vec![start];
                let (mut at,mut to) = (start,first);
                loop {
                    used.insert(key(at,to));
                    line.push(to);
                    if next[&to].len() != 2 { break }
                    let Some(&w) = next[&to].iter().find(|&&w| !used.contains(&key(to,w))) else { break };
                    (at,to) = (to,w);
                }
                // smooth where both faces' normals agree at its middle stretch
                let k = (line.len()-1)/2;
                let [ta,tb] = stretches[&key(line[k],line[k+1])];
                let p = std::array::from_fn(|c| 0.5*(pts[line[k] as usize][c]+pts[line[k+1] as usize][c]));
                let smooth = match (normal(fa,ta,p),normal(fb,tb,p)) {
                    (Some(na),Some(nb)) => crate::space::dot(na,nb) >= SMOOTH_EDGE.cos(),
                    _ => false,
                };
                out.push(crate::solid::ExactEdge {points:line,faces:[fa,fb],smooth});
            }
        }
    }
    out
}
