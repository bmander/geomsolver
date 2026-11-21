//! The solids a file represents, and the context each is represented in: its units, as
//! millimetres and radians to a unit of the file's, and the uncertainty the file states.
//!
//! **The schema.** A file is read under AP203 (`CONFIG_CONTROL_DESIGN`, and edition 2's
//! `AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF`), AP214
//! (`AUTOMOTIVE_DESIGN`, and its draft `AUTOMOTIVE_DESIGN_CC2`) or AP242
//! (`AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF`); a file naming any other schema is refused
//! `Unsupported`, naming it. An entity means what its schema says it does, and a schema the
//! reader was not written against may give a name it knows another meaning, or place a solid by
//! an entity it does not know to look for: read under a guess, such a file could be read wrong.
//!
//! **Units.** A unit of length or of plane angle is an SI unit under any SI prefix, or a
//! conversion-based unit read by following its conversion factor (a measure with a unit, that unit
//! another of the same kind, to any depth) down to the SI unit — never by its name, so the inch,
//! the foot and the degree are what their factors say. A unit the reader cannot resolve (a
//! context-dependent unit, a derived one, an unknown prefix) is refused naming it. A solid angle's
//! unit, and a unit of any other quantity, is not read. The uncertainty is the least length the
//! context states (the one called `distance_accuracy_value` where there is one), in the unit its
//! measure names; a context may state none. Each context is read once, however many solids it
//! holds.
//!
//! **Which solids.** A `MANIFOLD_SOLID_BREP` or a `BREP_WITH_VOIDS` that is an item of an
//! `ADVANCED_BREP_SHAPE_REPRESENTATION` or of a plain `SHAPE_REPRESENTATION`, simple or complex; a
//! solid in a representation of another kind is refused naming the representation, and a solid
//! in none is refused. Its name is its product's, walked from the `SHAPE_DEFINITION_REPRESENTATION`
//! of its representation — or of one a `SHAPE_REPRESENTATION_RELATIONSHIP` without a
//! transformation ties it to — through `PRODUCT_DEFINITION_SHAPE`, `PRODUCT_DEFINITION` and
//! `PRODUCT_DEFINITION_FORMATION` to the `PRODUCT`. **A placed solid is refused, never read as if
//! unplaced** (assemblies wait until a single body reads, #993): a file with an assembly's usage
//! occurrence, a representation relationship with a transformation or a mapped item reaching the
//! solid's representation, or an `AXIS2_PLACEMENT_3D` item of it (or of a representation tied to
//! it) that is not the identity, is refused `Unsupported` naming that entity. A file that holds
//! no solid but a shell, a surface model, a faceted B-rep, wireframe or tessellated geometry is
//! refused saying what it holds, naming the entity; beside a solid, a surface model or a faceted
//! B-rep (a body the reader would leave out) is refused, and tessellated or wireframe geometry (a
//! picture of a body, or construction) is not read.
use std::collections::{BTreeMap,BTreeSet};
use super::{Refusal,Why};
use crate::step_file::{Entity,File,Value};

/// A representation's context: what the file's numbers measure.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Context {
    /// The context's entity.
    pub entity: u32,
    /// Millimetres to the file's unit of length.
    pub length: f64,
    /// Radians to the file's unit of plane angle.
    pub angle: f64,
    /// The uncertainty the file states for its lengths, in millimetres; none where it states
    /// none.
    pub uncertainty: Option<f64>,
}

/// A solid a file represents: its entity (a `MANIFOLD_SOLID_BREP` or a `BREP_WITH_VOIDS`), its
/// product's name (empty where the file gives none), and its representation's context.
#[derive(Clone,Debug,PartialEq)]
pub struct Found { pub solid: u32,pub name: String,pub context: Context }

/// The application protocols a file is read under.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Schema { Ap203,Ap214,Ap242 }

/// The protocol a schema's name (`Header::schema_names`) is, where the reader reads it.
pub fn protocol(name: &str) -> Option<Schema> {
    Some(match name {
        "CONFIG_CONTROL_DESIGN" | "AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF" => Schema::Ap203,
        "AUTOMOTIVE_DESIGN" | "AUTOMOTIVE_DESIGN_CC2" => Schema::Ap214,
        "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF" => Schema::Ap242,
        _ => return None,
    })
}

/// The protocol the file's `FILE_SCHEMA` names; refused where it names none, or one (of several)
/// the reader does not read.
pub fn schema(file: &File) -> Result<Schema,Refusal> {
    let names = file.header.schema_names();
    let mut read = None;
    for n in &names {
        match protocol(n) {
            Some(s) => { read.get_or_insert(s); }
            None => return Err(Refusal::new(Why::Unsupported,None,format!("the file's schema is {n}, which the reader does not read \
                (AP203's CONFIG_CONTROL_DESIGN, AP214's AUTOMOTIVE_DESIGN and AP242's AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF are read)"))),
        }
    }
    read.ok_or_else(|| Refusal::new(Why::Malformed,None,"the file's header names no schema (FILE_SCHEMA)"))
}

/// `text` parsed (`step_file::File::read`), refused where it does not parse: `Unsupported` where
/// the text uses what the reader does not read (a SCOPE, a second DATA section), else
/// `Malformed`, naming the instance where there is one and the line where there is a place.
pub fn parse(text: &str) -> Result<File<'_>,Refusal> {
    File::read(text,crate::util::par::threads()*4,&|_| {}).map_err(|e| {
        let why = if e.unsupported { Why::Unsupported } else { Why::Malformed };
        Refusal::new(why,e.entity,crate::step_file::Error {entity:None,..e}.to_string())
    })
}

/// What a unit measures.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Quantity { Length,Angle,SolidAngle }

/// A unit resolved: what it measures, and its size in the SI unit of that (metres, radians,
/// steradians).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Unit { pub quantity: Quantity,pub si: f64 }

/// An SI prefix's factor.
fn prefix(word: &str) -> Option<f64> {
    Some(match word {
        "EXA" => 1e18,"PETA" => 1e15,"TERA" => 1e12,"GIGA" => 1e9,"MEGA" => 1e6,"KILO" => 1e3,"HECTO" => 1e2,"DECA" => 1e1,
        "DECI" => 1e-1,"CENTI" => 1e-2,"MILLI" => 1e-3,"MICRO" => 1e-6,"NANO" => 1e-9,"PICO" => 1e-12,"FEMTO" => 1e-15,"ATTO" => 1e-18,
        _ => return None,
    })
}

/// What the unit `e` says it measures: by its kind, else by its SI unit's name; another quantity
/// (a mass, a time) is `Err` with its kind, and a unit that says nothing is `Err` with none.
fn quantity(e: &Entity) -> Result<Quantity,Option<String>> {
    if e.is("LENGTH_UNIT") { return Ok(Quantity::Length) }
    if e.is("PLANE_ANGLE_UNIT") { return Ok(Quantity::Angle) }
    if e.is("SOLID_ANGLE_UNIT") { return Ok(Quantity::SolidAngle) }
    match e.part("SI_UNIT").and_then(|v| v.get(1)).and_then(Value::word) {
        Some("METRE") => return Ok(Quantity::Length),
        Some("RADIAN") => return Ok(Quantity::Angle),
        Some("STERADIAN") => return Ok(Quantity::SolidAngle),
        _ => {}
    }
    const GENERIC: [&str;5] = ["NAMED_UNIT","SI_UNIT","CONVERSION_BASED_UNIT","CONTEXT_DEPENDENT_UNIT","DERIVED_UNIT"];
    Err(e.parts.iter().map(|p| p.0).find(|n| n.ends_with("_UNIT") && !GENERIC.contains(n)).map(str::to_string))
}

/// The parameters of `e`'s part that is one of `kinds` (its first, for a complex entity).
fn params<'e,'a>(e: &'e Entity<'a>,kinds: &[&str]) -> Option<&'e [Value<'a>]> {
    e.parts.iter().find(|(n,_)| kinds.contains(n)).map(|(_,v)| &v[..])
}

/// The unit `#u`, resolved to SI; refused naming the unit where it cannot be.
pub fn unit(file: &File,u: u32) -> Result<Unit,Refusal> { resolve(file,u,0) }

fn resolve(file: &File,u: u32,depth: usize) -> Result<Unit,Refusal> {
    let e = file.get(u);
    let quantity = quantity(e).map_err(|other| match other {
        Some(k) => Refusal::unsupported(u,format!("a unit of another quantity ({k}) where a length or an angle is measured")),
        None => Refusal::unsupported(u,format!("a unit ({}) that says nothing of what it measures",e.kinds())),
    })?;
    let si = if let Some(v) = e.part("SI_UNIT") {
        let [p,name] = v else { return Err(Refusal::malformed(u,"an SI unit without its prefix and name")) };
        let want = match quantity { Quantity::Length => "METRE",Quantity::Angle => "RADIAN",Quantity::SolidAngle => "STERADIAN" };
        if name.word() != Some(want) { return Err(Refusal::malformed(u,format!("a unit of {quantity:?} whose SI unit is {name:?}, not {want}"))) }
        match p {
            Value::Unset => 1.,
            Value::Enum(w) => prefix(w).ok_or_else(|| Refusal::unsupported(u,format!("an SI prefix .{w}. the reader does not know")))?,
            _ => return Err(Refusal::malformed(u,"an SI unit whose prefix is no enumeration")),
        }
    } else if let Some(v) = e.part("CONVERSION_BASED_UNIT") {
        if depth > 16 { return Err(Refusal::malformed(u,"a conversion-based unit whose conversions do not end")) }
        let name = v.first().and_then(Value::name).unwrap_or_default();
        let f = v.get(1).and_then(Value::reference).ok_or_else(|| Refusal::malformed(u,format!("the unit '{name}' without its conversion factor")))?;
        let m = file.get(f);
        let mv = m.parts.iter().find(|(n,v)| n.ends_with("MEASURE_WITH_UNIT") && v.len() >= 2).map(|(_,v)| v)
            .ok_or_else(|| Refusal::malformed(f,format!("the conversion factor of the unit '{name}' (#{u}) is no measure with a unit")))?;
        let x = mv[0].measure().filter(|x| x.is_finite() && *x > 0.)
            .ok_or_else(|| Refusal::malformed(f,format!("the conversion factor of the unit '{name}' (#{u}) is no positive number")))?;
        let inner = mv[1].reference().ok_or_else(|| Refusal::malformed(f,format!("the conversion factor of the unit '{name}' (#{u}) names no unit")))?;
        let to = resolve(file,inner,depth+1)?;
        if to.quantity != quantity { return Err(Refusal::malformed(f,format!("the unit '{name}' (#{u}), a {quantity:?}, converted to a {:?}",to.quantity))) }
        x*to.si
    } else if let Some(v) = e.part("CONTEXT_DEPENDENT_UNIT") {
        let name = v.first().and_then(Value::name).unwrap_or_default();
        return Err(Refusal::unsupported(u,format!("a context-dependent unit '{name}', which has no conversion to SI")))
    } else {
        return Err(Refusal::unsupported(u,format!("a unit ({}) the reader does not resolve to SI",e.kinds())))
    };
    if !(si.is_finite() && si > 0.) { return Err(Refusal::malformed(u,"a unit whose size is no positive number")) }
    Ok(Unit {quantity,si})
}

/// The context `#r`: its units of length and plane angle (`unit`) and its least stated
/// uncertainty of length; refused where it is no three-dimensional geometric context, where it
/// lacks either unit or states one twice, or where a unit or an uncertainty cannot be read.
pub fn context(file: &File,r: u32) -> Result<Context,Refusal> {
    let e = file.get(r);
    let dimension = e.part("GEOMETRIC_REPRESENTATION_CONTEXT").and_then(|v| v.first()).and_then(Value::int)
        .ok_or_else(|| Refusal::malformed(r,"a solid's representation context that is no geometric one"))?;
    if dimension != 3 { return Err(Refusal::malformed(r,format!("a solid's representation context of {dimension} dimensions"))) }
    let units = e.part("GLOBAL_UNIT_ASSIGNED_CONTEXT").and_then(|v| v.first()).and_then(Value::list)
        .ok_or_else(|| Refusal::malformed(r,"a representation context that assigns no units"))?;
    let (mut length,mut angle) = (None,None);
    for v in units {
        let u = v.reference().ok_or_else(|| Refusal::malformed(r,"a representation context's unit that is no reference"))?;
        // a solid angle's unit, or another quantity's, measures nothing the reader reads
        if matches!(quantity(file.get(u)),Ok(Quantity::SolidAngle) | Err(Some(_))) { continue }
        let unit = unit(file,u)?;
        let slot = if unit.quantity == Quantity::Length { &mut length } else { &mut angle };
        if slot.is_some() { return Err(Refusal::malformed(r,format!("a representation context with two units of {:?}",unit.quantity))) }
        *slot = Some(unit.si);
    }
    let length = length.ok_or_else(|| Refusal::malformed(r,"a representation context without a unit of length"))?*1e3;
    let angle = angle.ok_or_else(|| Refusal::malformed(r,"a representation context without a unit of plane angle"))?;
    // the uncertainty, each in the unit its measure names
    let mut stated: Vec<(bool,f64)> = Vec::new();
    for v in e.part("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT").and_then(|v| v.first()).and_then(Value::list).unwrap_or(&[]) {
        let Some(m) = v.reference() else { return Err(Refusal::malformed(r,"an uncertainty that is no reference")) };
        let me = file.get(m);
        let Some(p) = params(me,&["UNCERTAINTY_MEASURE_WITH_UNIT","MEASURE_WITH_UNIT"]).filter(|p| p.len() >= 2)
            else { return Err(Refusal::malformed(m,format!("an uncertainty ({}) that is no measure with a unit",me.kinds()))) };
        let u = p[1].reference().ok_or_else(|| Refusal::malformed(m,"an uncertainty that names no unit"))?;
        if !matches!(quantity(file.get(u)),Ok(Quantity::Length)) { continue }
        let x = p[0].measure().filter(|x| x.is_finite() && *x > 0.).ok_or_else(|| Refusal::malformed(m,"an uncertainty that is no positive number"))?;
        let named = me.part("UNCERTAINTY_MEASURE_WITH_UNIT").and_then(|p| p.get(2)).and_then(Value::name).is_some_and(|n| n == "distance_accuracy_value");
        stated.push((named,x*unit(file,u)?.si*1e3));
    }
    let named = stated.iter().any(|s| s.0);
    let uncertainty = stated.iter().filter(|s| s.0 == named).map(|s| s.1).reduce(f64::min);
    Ok(Context {entity:r,length,angle,uncertainty})
}

/// What a representation lists: its items and its context, for one whose parameters are a
/// representation's (`name, items, context`).
fn representation<'e,'a>(e: &'e Entity<'a>) -> Option<(&'e [Value<'a>],u32)> {
    e.parts.iter().find(|(n,v)| (*n == "REPRESENTATION" || n.ends_with("_REPRESENTATION")) && v.len() == 3)
        .and_then(|(_,v)| Some((v[1].list()?,v[2].reference()?)))
}

/// The kinds a representation holding a solid may be.
const HOLDERS: [&str;3] = ["ADVANCED_BREP_SHAPE_REPRESENTATION","SHAPE_REPRESENTATION","REPRESENTATION"];

/// The kinds of a usage occurrence, an assembly's.
const USAGES: [&str;4] = ["NEXT_ASSEMBLY_USAGE_OCCURRENCE","ASSEMBLY_COMPONENT_USAGE","SPECIFIED_HIGHER_USAGE_OCCURRENCE","PROMISSORY_USAGE_OCCURRENCE"];

/// What a file may hold that is no solid, as a refusal says it, the bodies first.
const HELD: [(&str,&str);20] = [
    ("SHELL_BASED_SURFACE_MODEL","a shell-based surface model"),("FACETED_BREP","a faceted B-rep"),
    ("MANIFOLD_SURFACE_SHAPE_REPRESENTATION","a manifold surface representation"),
    ("GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION","geometrically bounded surfaces"),
    ("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION","a geometrically bounded wireframe"),
    ("EDGE_BASED_WIREFRAME_MODEL","an edge-based wireframe"),("SHELL_BASED_WIREFRAME_MODEL","a shell-based wireframe"),
    ("TESSELLATED_SHAPE_REPRESENTATION","tessellated geometry"),("TESSELLATED_SOLID","a tessellated solid"),
    ("TESSELLATED_SHELL","a tessellated shell"),("TRIANGULATED_FACE","a triangulated face"),
    ("COMPLEX_TRIANGULATED_FACE","a triangulated face"),("TRIANGULATED_SURFACE_SET","a triangulated surface set"),
    ("COMPLEX_TRIANGULATED_SURFACE_SET","a triangulated surface set"),("CLOSED_SHELL","a closed shell in no solid"),
    ("OPEN_SHELL","an open shell"),("ADVANCED_FACE","faces in no solid"),("GEOMETRIC_CURVE_SET","a set of curves"),
    ("GEOMETRIC_SET","a geometric set"),("POLY_LOOP","polygons"),
];

/// The bodies that, beside a solid, the reader would leave out.
const BESIDE: [&str;2] = ["SHELL_BASED_SURFACE_MODEL","FACETED_BREP"];

fn is_solid(e: &Entity) -> bool { e.is("MANIFOLD_SOLID_BREP") || e.is("BREP_WITH_VOIDS") }

/// Whether the placement `#p` is no `AXIS2_PLACEMENT_3D`, or is the identity: at the origin,
/// its axis `z` (or unset) and its reference direction `x` (or unset, or along `z` off `x`'s
/// plane by nothing).
fn identity(file: &File,p: u32) -> bool {
    let Some(("AXIS2_PLACEMENT_3D",v)) = file.get(p).simple() else { return true };
    let numbers = |r: Option<&Value>,kind: &str| -> Option<Vec<f64>> {
        let r = r?.reference()?;
        let (_,w) = file.of(Some(r),&[kind],"").ok()?;
        w.get(1)?.list()?.iter().map(Value::number).collect()
    };
    let unit = |d: Vec<f64>| { let n = d.iter().map(|x| x*x).sum::<f64>().sqrt(); (n > 0.).then(|| d.iter().map(|x| x/n).collect::<Vec<_>>()) };
    let near = |a: &[f64],b: [f64;3]| a.len() == 3 && a.iter().zip(b).all(|(x,y)| (x-y).abs() <= 1e-12);
    let Some(o) = numbers(v.get(1),"CARTESIAN_POINT") else { return false };
    if !(o.len() == 3 && o.iter().all(|x| x.abs() <= 1e-12)) { return false }
    let z = match v.get(2) { Some(Value::Unset) => Some(vec![0.,0.,1.]),r => numbers(r,"DIRECTION").and_then(unit) };
    let x = match v.get(3) { Some(Value::Unset) => Some(vec![1.,0.,0.]),r => numbers(r,"DIRECTION").and_then(unit) };
    match (z,x) {
        (Some(z),Some(x)) if near(&z,[0.,0.,1.]) => near(&x,[1.,0.,0.]) || (x[1].abs() <= 1e-12 && x[0] > 0.),
        _ => false,
    }
}

/// The product's name a shape definition representation `#sdr` is of, where the chain to it is
/// whole.
fn product_name(file: &File,sdr: u32) -> Option<String> {
    let at = |e: &Entity,kinds: &[&str],k: usize| params(e,kinds)?.get(k)?.reference();
    let pds = at(file.get(sdr),&["SHAPE_DEFINITION_REPRESENTATION","PROPERTY_DEFINITION_REPRESENTATION"],0)?;
    let pd = at(file.get(pds),&["PRODUCT_DEFINITION_SHAPE","PROPERTY_DEFINITION"],2)?;
    let pdf = at(file.get(pd),&["PRODUCT_DEFINITION","PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS"],2)?;
    let p = at(file.get(pdf),&["PRODUCT_DEFINITION_FORMATION","PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE"],2)?;
    params(file.get(p),&["PRODUCT"])?.get(1)?.name()
}

/// Every solid the file represents, in the file's order (see the module's account of which are
/// read and which refused).
pub fn solids(file: &File) -> Result<Vec<Found>,Refusal> {
    schema(file)?;
    if let Some(e) = file.entities.iter().find(|e| USAGES.iter().any(|k| e.is(k))) {
        return Err(Refusal::unsupported(e.id,format!("an assembly's component ({}): assemblies are not read yet",e.kinds())))
    }
    let order: Vec<u32> = file.entities.iter().filter(|e| is_solid(e)).map(|e| e.id).collect();
    let solids: BTreeSet<u32> = order.iter().copied().collect();
    if solids.is_empty() {
        for (kind,what) in HELD {
            if let Some(e) = file.entities.iter().find(|e| e.is(kind)) {
                return Err(Refusal::unsupported(e.id,format!("the file holds {what} ({kind}) and no solid")))
            }
        }
        return Ok(Vec::new())
    }
    for kind in BESIDE {
        if let Some(e) = file.entities.iter().find(|e| e.is(kind)) {
            return Err(Refusal::unsupported(e.id,format!("beside its solids the file holds a body the reader does not read ({kind})")))
        }
    }
    // the representations, relationships, maps and shape definitions, in one pass
    let mut holders: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
    let mut ties: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
    let mut moved: Vec<(u32,u32,u32)> = Vec::new();
    let mut maps: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
    let mut mapped: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
    let mut defined: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
    for e in &file.entities {
        if let Some((items,_)) = representation(e) {
            for i in items.iter().filter_map(Value::reference).filter(|i| solids.contains(i)) { holders.entry(i).or_default().push(e.id); }
        }
        if let Some(v) = e.parts.iter().find(|(n,v)| n.ends_with("REPRESENTATION_RELATIONSHIP") && v.len() == 4).map(|(_,v)| v) {
            if let (Some(a),Some(b)) = (v[2].reference(),v[3].reference()) {
                if e.is("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") { moved.push((e.id,a,b)); }
                else { ties.entry(a).or_default().push(b); ties.entry(b).or_default().push(a); }
            }
        }
        if let Some(v) = params(e,&["REPRESENTATION_MAP"]) {
            if let Some(r) = v.get(1).and_then(Value::reference) { maps.entry(r).or_default().push(e.id); }
        }
        if let Some(v) = params(e,&["MAPPED_ITEM"]) {
            if let Some(m) = v.get(1).and_then(Value::reference) { mapped.entry(m).or_default().push(e.id); }
        }
        if let Some(v) = params(e,&["SHAPE_DEFINITION_REPRESENTATION"]) {
            if let Some(r) = v.get(1).and_then(Value::reference) { defined.entry(r).or_default().push(e.id); }
        }
    }
    let mut contexts: BTreeMap<u32,Context> = BTreeMap::new();
    let mut out = Vec::new();
    for &solid in &order {
        let reps = holders.get(&solid).map(Vec::as_slice).unwrap_or(&[]);
        if reps.is_empty() { return Err(Refusal::unsupported(solid,"a solid that is an item of no shape representation")) }
        let mut context = None;
        for &rep in reps {
            let e = file.get(rep);
            if let Some(k) = e.parts.iter().map(|p| p.0).find(|k| !HOLDERS.contains(k)) {
                return Err(Refusal::unsupported(rep,format!("a solid (#{solid}) is an item of a {k}, which the reader does not read solids from")))
            }
            let at = representation(e).unwrap().1;
            let c = match contexts.get(&at) { Some(&c) => c,None => { let c = self::context(file,at)?; contexts.insert(at,c); c } };
            match context {
                None => context = Some(c),
                Some(was) if (Context {entity:at,..was}) != c => return Err(Refusal::unsupported(rep,
                    format!("the solid #{solid} is represented in two contexts that measure differently (#{} and #{at})",was.entity))),
                _ => {}
            }
        }
        // the representations tied to the solid's without a transformation, nearest first
        let mut tied: Vec<u32> = reps.to_vec();
        let mut seen: BTreeSet<u32> = tied.iter().copied().collect();
        let mut k = 0;
        while k < tied.len() {
            for &t in ties.get(&tied[k]).map(Vec::as_slice).unwrap_or(&[]) { if seen.insert(t) { tied.push(t); } }
            k += 1;
        }
        if let Some(&(r,_,_)) = moved.iter().find(|(_,a,b)| seen.contains(a) || seen.contains(b)) {
            return Err(Refusal::unsupported(r,format!("a transformation places the solid #{solid}'s representation: placed solids are not read yet")))
        }
        for &rep in &tied {
            if let Some(&m) = maps.get(&rep).into_iter().flatten().flat_map(|m| mapped.get(m)).flatten().next() {
                return Err(Refusal::unsupported(m,format!("a mapped item places the solid #{solid}'s representation (#{rep}): placed solids are not read yet")))
            }
            if let Some((items,_)) = file.find(rep).and_then(representation) {
                if let Some(p) = items.iter().filter_map(Value::reference).find(|&p| !identity(file,p)) {
                    return Err(Refusal::unsupported(p,format!("a placement other than the identity in the solid #{solid}'s representation (#{rep}): placed solids are not read yet")))
                }
            }
        }
        let name = tied.iter().flat_map(|r| defined.get(r)).flatten().find_map(|&sdr| product_name(file,sdr)).unwrap_or_default();
        out.push(Found {solid,name,context:context.unwrap()});
    }
    Ok(out)
}
