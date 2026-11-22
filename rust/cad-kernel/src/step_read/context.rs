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
//! in none is refused.
//!
//! **Where it stands.** A solid stands in its product's space, the space of the representation the
//! product's `SHAPE_DEFINITION_REPRESENTATION` names (through its `PRODUCT_DEFINITION_SHAPE`), and
//! its name is that product's. From that representation the reader walks (ISO 10303-43): a
//! representation relationship without a transformation shares the space; a `MAPPED_ITEM` among
//! a representation's items places its map's representation by the rigid motion taking the map's
//! origin onto the item's target; a `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` places its
//! `rep_1` in its `rep_2` by the motion taking its `ITEM_DEFINED_TRANSFORMATION`'s first item onto
//! its second (rep_1 the placed, as the standard's rule on the items and Open CASCADE's
//! assemblies have it). Each axis is read in its own representation's units, so a part in inches
//! mapped into a product in millimetres is placed, not scaled; two `AXIS2_PLACEMENT_3D`s are
//! right-handed, so the motion is proper. A solid so placed is read in its own representation's
//! context and moved (`Found::placed`, `Brep::moved`): exact. What places nothing: an
//! `AXIS2_PLACEMENT_3D` that is merely an item of a representation (a datum, the part's frame: a
//! representation's items share its context's space), and a mapped item in a `DRAUGHTING_MODEL`
//! (the shape presented in a view of its annotation). Refused `Unsupported`, naming the entity, as
//! product structure, which is not read yet: an assembly's usage occurrence anywhere in the file;
//! a solid placed twice in its product's shape (an instance used more than once); solids in the
//! shapes of two products; a transformation by anything but two axis placements (a cartesian
//! transformation operator, which may scale or mirror). A solid in no product's shape is read in
//! its representation's space, named through a shape definition of a representation tied to it,
//! unless a mapped item or a transformation places it somewhere, which is refused.
//!
//! **What is beside.** A file that holds no solid but a shell, a surface model, a faceted B-rep,
//! wireframe or tessellated geometry is refused saying what it holds, naming the entity. Beside a
//! solid, what the reader does not give (surface models, wireframes, sets of curves, tessellated
//! bodies, construction geometry, annotation) is said, each by its entity (`Beside`), never
//! refused and never hidden.
use std::collections::{BTreeMap,BTreeSet};
use super::{Refusal,Why};
use crate::geom::{Rigid,V};
use crate::step_file::{Entity,File,Value};
use crate::util::space::{cross,dot,norm,normalised,scale,sub};

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
/// product's name (empty where the file gives none), its representation's context, and where its
/// product's shape places it (none where its representation's space is its product's).
#[derive(Clone,Debug,PartialEq)]
pub struct Found { pub solid: u32,pub name: String,pub context: Context,pub placed: Option<Placed> }

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

/// The items that, held by a representation beside the solids, are a body or a set of geometry
/// the reader does not give, and what each is.
const BODIES: [(&str,&str);14] = [
    ("SHELL_BASED_SURFACE_MODEL","a shell-based surface model"),("FACETED_BREP","a faceted B-rep"),
    ("EDGE_BASED_WIREFRAME_MODEL","an edge-based wireframe"),("SHELL_BASED_WIREFRAME_MODEL","a shell-based wireframe"),
    ("GEOMETRIC_CURVE_SET","a set of curves"),("GEOMETRIC_SET","a geometric set"),
    ("TESSELLATED_SOLID","a tessellated solid"),("TESSELLATED_SHELL","a tessellated shell"),
    ("TESSELLATED_SURFACE_SET","a tessellated surface set"),("TRIANGULATED_SURFACE_SET","a triangulated surface set"),
    ("COMPLEX_TRIANGULATED_SURFACE_SET","a triangulated surface set"),("TRIANGULATED_FACE","a triangulated face"),
    ("OPEN_SHELL","an open shell"),("CLOSED_SHELL","a closed shell in no solid"),
];

fn is_solid(e: &Entity) -> bool { e.is("MANIFOLD_SOLID_BREP") || e.is("BREP_WITH_VOIDS") }

/// A presentation of a product's shape (its annotation, a view of it), which places nothing in it.
fn is_presentation(e: &Entity) -> bool { e.is("DRAUGHTING_MODEL") }

/// A solid placed in its product's space: the entity that moves it last on the way from the
/// product's shape representation (a `MAPPED_ITEM`, or a `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`),
/// and the rigid motion, in millimetres, taking the solid's representation into the product's.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Placed { pub by: u32,pub motion: Rigid }

/// What a file holds beside the solids read, which the reader does not give: the entity, its kind
/// (a complex entity's parts joined by `+`) and what it is. A body or a set of geometry held by a
/// representation (a surface model, a wireframe, a set of curves, tessellated geometry), each
/// item of a `CONSTRUCTIVE_GEOMETRY_REPRESENTATION` (construction geometry: datums, curves,
/// surfaces), and each `DRAUGHTING_MODEL` (annotation, the product's manufacturing information
/// presented), once each, in the file's order.
#[derive(Clone,Debug,PartialEq)]
pub struct Beside { pub entity: u32,pub kind: String,pub what: &'static str }

/// What a file represents: its solids, and what it holds beside them (`Beside`).
#[derive(Clone,Debug,PartialEq)]
pub struct Shapes { pub solids: Vec<Found>,pub beside: Vec<Beside> }

/// The axes `#p`, a representation item in a context `length` millimetres to its unit, as the
/// rigid motion taking the world's axes onto it; an `AXIS2_PLACEMENT_3D` as ISO 10303-42 builds
/// it (its axis `z` where unset, its reference direction `x` made square to the axis, or `y`
/// where unset and the axis lies along `x`), right-handed. Any other item (a cartesian
/// transformation operator) is refused naming it.
fn axes(file: &File,p: u32,length: f64) -> Result<Rigid,Refusal> {
    let e = file.get(p);
    let Some(("AXIS2_PLACEMENT_3D",v)) = e.simple() else {
        return Err(Refusal::unsupported(p,format!("a placement by a {}, which the reader does not read yet \
            (an AXIS2_PLACEMENT_3D is)",e.kinds())))
    };
    let three = |r: Option<&Value>,kind: &str| -> Result<V,Refusal> {
        let r = r.and_then(Value::reference).ok_or_else(|| Refusal::malformed(p,format!("a placement whose {kind} is no reference")))?;
        let (_,w) = file.of(Some(r),&[kind],"a placement's").map_err(|m| Refusal::malformed(p,m))?;
        let c = w.get(1).and_then(Value::list).and_then(|l| l.iter().map(Value::number).collect::<Option<Vec<f64>>>());
        match c.as_deref() {
            Some(&[x,y,z]) if [x,y,z].iter().all(|x| x.is_finite()) => Ok([x,y,z]),
            _ => Err(Refusal::malformed(r,"three finite coordinates expected")),
        }
    };
    let o = three(v.get(1),"CARTESIAN_POINT")?.map(|x| x*length);
    let z = match v.get(2) {
        Some(Value::Unset) => [0.,0.,1.],
        r => normalised(three(r,"DIRECTION")?).ok_or_else(|| Refusal::malformed(p,"a placement's axis of no length"))?,
    };
    let x = match v.get(3) {
        // (the default's own part square to the axis, however small: `geometry::placement`)
        Some(Value::Unset) => {
            let v = if z[1] == 0. && z[2] == 0. { [0.,1.,0.] } else { [1.,0.,0.] };
            normalised(sub(v,scale(z,dot(v,z)))).ok_or_else(|| Refusal::malformed(p,"a placement whose axis leaves its reference direction none"))?
        }
        r => three(r,"DIRECTION")?,
    };
    let x = normalised(sub(x,scale(z,dot(x,z)))).filter(|_| norm(cross(x,z)) > 1e-12*norm(x))
        .ok_or_else(|| Refusal::malformed(p,"a placement whose reference direction lies along its axis"))?;
    let y = cross(z,x);
    Ok(Rigid {r:[[x[0],y[0],z[0]],[x[1],y[1],z[1]],[x[2],y[2],z[2]]],t:o})
}

/// Whether two motions are one (to a part in 1e12, the frames being the file's unit vectors).
fn same(a: &Rigid,b: &Rigid) -> bool {
    let scale = 1.+norm(a.t).max(norm(b.t));
    (0..3).all(|i| (0..3).all(|j| (a.r[i][j]-b.r[i][j]).abs() <= 1e-12) && (a.t[i]-b.t[i]).abs() <= 1e-12*scale)
}

/// Whether `#pd` is a product's definition.
fn is_definition(e: &Entity) -> bool { params(e,&["PRODUCT_DEFINITION","PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS"]).is_some() }

/// The name of the product `#pd` defines, where the chain to it is whole.
fn defined_name(file: &File,pd: u32) -> Option<String> {
    let at = |e: &Entity,kinds: &[&str],k: usize| params(e,kinds)?.get(k)?.reference();
    let pdf = at(file.get(pd),&["PRODUCT_DEFINITION","PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS"],2)?;
    let p = at(file.get(pdf),&["PRODUCT_DEFINITION_FORMATION","PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE"],2)?;
    params(file.get(p),&["PRODUCT"])?.get(1)?.name()
}

/// The product's name a shape definition representation `#sdr` is of, where the chain to it is
/// whole.
fn product_name(file: &File,sdr: u32) -> Option<String> {
    let at = |e: &Entity,kinds: &[&str],k: usize| params(e,kinds)?.get(k)?.reference();
    let pds = at(file.get(sdr),&["SHAPE_DEFINITION_REPRESENTATION","PROPERTY_DEFINITION_REPRESENTATION"],0)?;
    let pd = at(file.get(pds),&["PRODUCT_DEFINITION_SHAPE","PROPERTY_DEFINITION"],2)?;
    defined_name(file,pd)
}

/// A representation reached from a product's shape: the product's definition, the motion taking
/// it into the product's space (or why that motion cannot be read), and the entity that moved
/// it last.
#[derive(Clone)]
struct Reached { product: u32,motion: Result<Rigid,Refusal>,by: Option<u32> }

/// What the file holds beside the solids `solids` (`Beside`).
fn beside(file: &File,solids: &BTreeSet<u32>) -> Vec<Beside> {
    // the shells of a surface model or a solid are said with it
    let mut bodied: BTreeSet<u32> = BTreeSet::new();
    for e in &file.entities {
        if let Some(v) = params(e,&["SHELL_BASED_SURFACE_MODEL"]) { v.iter().filter_map(Value::list).flatten().filter_map(Value::reference).for_each(|s| { bodied.insert(s); }); }
        if solids.contains(&e.id) { let mut add = |s| { bodied.insert(s); }; e.parts.iter().flat_map(|p| &p.1).for_each(|v| v.references(&mut add)); }
    }
    let mut out: BTreeMap<u32,Beside> = BTreeMap::new();
    for e in &file.entities {
        if is_presentation(e) {
            out.entry(e.id).or_insert(Beside {entity:e.id,kind:e.kinds(),what:"annotation (a draughting model)"});
            continue
        }
        let Some((items,_)) = representation(e) else { continue };
        let construction = e.is("CONSTRUCTIVE_GEOMETRY_REPRESENTATION");
        for i in items.iter().filter_map(Value::reference).filter(|i| !solids.contains(i) && !bodied.contains(i)) {
            let item = file.get(i);
            let what = match BODIES.iter().find(|(k,_)| item.is(k)) {
                _ if construction => "construction geometry",
                Some(("SHELL_BASED_SURFACE_MODEL",_)) if params(item,&["SHELL_BASED_SURFACE_MODEL"]).and_then(|v| v.get(1)).and_then(Value::list)
                    .is_some_and(|l| l.iter().filter_map(Value::reference).any(|s| file.get(s).is("CLOSED_SHELL"))) =>
                    "a shell-based surface model closed round a volume",
                Some((_,w)) => w,
                None => continue,
            };
            out.entry(i).or_insert(Beside {entity:i,kind:item.kinds(),what});
        }
    }
    out.into_values().collect()
}

/// Every solid the file represents, in the file's order (see the module's account of which are
/// read and which refused).
pub fn solids(file: &File) -> Result<Vec<Found>,Refusal> { shapes(file).map(|s| s.solids) }

/// Every solid the file represents, in the file's order, each placed in its product's space,
/// and what the file holds beside them (see the module's account).
pub fn shapes(file: &File) -> Result<Shapes,Refusal> {
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
        return Ok(Shapes {solids:Vec::new(),beside:Vec::new()})
    }
    // the representations, relationships, maps and shape definitions, in one pass
    let mut holders: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
    let mut ties: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
    // (relationship, rep_1, rep_2, its transformation)
    let mut moved: Vec<(u32,u32,u32,Option<u32>)> = Vec::new();
    // a map's origin and mapped representation, by the map
    let mut maps: BTreeMap<u32,(Option<u32>,Option<u32>)> = BTreeMap::new();
    // the mapped items a representation holds: (item, map, target), by the representation
    let mut mapping: BTreeMap<u32,Vec<(u32,Option<u32>,Option<u32>)>> = BTreeMap::new();
    let mut defined: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
    // the products' shapes: (product definition, representation), in the file's order
    let mut products: Vec<(u32,u32)> = Vec::new();
    for e in &file.entities {
        if let Some((items,_)) = representation(e) {
            for i in items.iter().filter_map(Value::reference) {
                if solids.contains(&i) { holders.entry(i).or_default().push(e.id); }
                if !is_presentation(e) {
                    if let Some(v) = params(file.get(i),&["MAPPED_ITEM"]) {
                        mapping.entry(e.id).or_default().push((i,v.get(1).and_then(Value::reference),v.get(2).and_then(Value::reference)));
                    }
                }
            }
        }
        if let Some(v) = e.parts.iter().find(|(n,v)| n.ends_with("REPRESENTATION_RELATIONSHIP") && v.len() == 4).map(|(_,v)| v) {
            if let (Some(a),Some(b)) = (v[2].reference(),v[3].reference()) {
                if let Some(t) = e.part("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
                    moved.push((e.id,a,b,t.first().and_then(Value::reference)));
                } else if !e.is("MECHANICAL_DESIGN_AND_DRAUGHTING_RELATIONSHIP") && ![a,b].iter().any(|&r| is_presentation(file.get(r))) {
                    ties.entry(a).or_default().push(b); ties.entry(b).or_default().push(a);
                }
            }
        }
        if let Some(v) = params(e,&["REPRESENTATION_MAP"]) { maps.insert(e.id,(v.first().and_then(Value::reference),v.get(1).and_then(Value::reference))); }
        if let Some(v) = params(e,&["SHAPE_DEFINITION_REPRESENTATION"]) {
            if let Some(r) = v.get(1).and_then(Value::reference) {
                defined.entry(r).or_default().push(e.id);
                let pd = v.first().and_then(Value::reference).and_then(|d| params(file.get(d),&["PRODUCT_DEFINITION_SHAPE"])?.get(2)?.reference());
                if let Some(pd) = pd.filter(|&pd| is_definition(file.get(pd))) { products.push((pd,r)); }
            }
        }
    }
    let mut contexts: BTreeMap<u32,Result<Context,Refusal>> = BTreeMap::new();
    let mut length = |rep: u32| -> Result<f64,Refusal> {
        let at = file.find(rep).and_then(representation).ok_or_else(|| Refusal::malformed(rep,"a representation expected"))?.1;
        contexts.entry(at).or_insert_with(|| self::context(file,at)).clone().map(|c| c.length)
    };
    // each representation reached from a product's shape, with the motion taking it there: through
    // a relationship without a transformation (the same space), into a mapped item's representation
    // (its origin taken onto its target), into a transformed relationship's rep_1 from its rep_2
    // (its first item taken onto its second); never from a presentation
    let mut reached: BTreeMap<u32,Vec<Reached>> = BTreeMap::new();
    let mut budget = 64*(file.entities.len()+64);
    for &(pd,start) in &products {
        let mut stack: Vec<(u32,Reached,usize)> = vec![(start,Reached {product:pd,motion:Ok(Rigid::identity()),by:None},0)];
        while let Some((rep,at,depth)) = stack.pop() {
            let here = reached.entry(rep).or_default();
            if here.iter().any(|h| h.product == pd && match (&h.motion,&at.motion) { (Ok(a),Ok(b)) => same(a,b),(a,b) => a == b }) { continue }
            here.push(at.clone());
            budget = budget.saturating_sub(1);
            if depth > 64 || budget == 0 {
                return Err(Refusal::malformed(at.by.unwrap_or(rep),"representations placed in one another without end"))
            }
            for &t in ties.get(&rep).into_iter().flatten() { stack.push((t,at.clone(),depth+1)); }
            let then = |by: u32,step: Result<Rigid,Refusal>| Reached {product:pd,by:Some(by),motion:match (&at.motion,step) {
                (Ok(m),Ok(s)) => Ok(s.then(m)),(Err(e),_) => Err(e.clone()),(_,Err(e)) => Err(e),
            }};
            for &(item,map,target) in mapping.get(&rep).into_iter().flatten() {
                let Some(&(Some(origin),Some(source))) = map.and_then(|m| maps.get(&m)) else {
                    return Err(Refusal::malformed(item,"a mapped item whose source is no representation map"))
                };
                let Some(target) = target else { return Err(Refusal::malformed(item,"a mapped item without its target")) };
                let step = (|| Ok(axes(file,origin,length(source)?)?.inverse().then(&axes(file,target,length(rep)?)?)))();
                stack.push((source,then(item,step),depth+1));
            }
            for &(rel,one,_,op) in moved.iter().filter(|m| m.2 == rep) {
                let step = (|| {
                    let op = op.ok_or_else(|| Refusal::malformed(rel,"a relationship with a transformation without one"))?;
                    let Some(("ITEM_DEFINED_TRANSFORMATION",v)) = file.get(op).simple() else {
                        return Err(Refusal::unsupported(op,format!("a transformation by a {}, which the reader does not read yet \
                            (an ITEM_DEFINED_TRANSFORMATION is)",file.get(op).kinds())))
                    };
                    let (Some(a),Some(b)) = (v.get(2).and_then(Value::reference),v.get(3).and_then(Value::reference))
                        else { return Err(Refusal::malformed(op,"a transformation without its two items")) };
                    Ok(axes(file,a,length(one)?)?.inverse().then(&axes(file,b,length(rep)?)?))
                })();
                stack.push((one,then(rel,step),depth+1));
            }
        }
    }
    let mut contexts: BTreeMap<u32,Context> = BTreeMap::new();
    let mut out = Vec::new();
    let mut product: Option<u32> = None;
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
        let ways: Vec<&Reached> = reps.iter().flat_map(|r| reached.get(r)).flatten().collect();
        let (name,placed) = if let Some(first) = ways.first() {
            // the solid in its product's shape, placed one way
            if let Some(other) = ways.iter().find(|w| w.product != first.product).map(|w| w.product)
                .or_else(|| product.filter(|&p| p != first.product)) {
                return Err(Refusal::unsupported(other.max(first.product),format!("the solid #{solid} is in the shapes of two products \
                    (#{} and #{}): several products in one file are product structure, which is not read yet",
                    other.min(first.product),other.max(first.product))))
            }
            product = Some(first.product);
            let motion = first.motion.clone()?;
            for w in &ways[1..] {
                if !same(&w.motion.clone()?,&motion) {
                    let said = |b: Option<u32>| b.map_or("the shape itself".to_string(),|b| format!("#{b}"));
                    let by = [w.by,first.by].into_iter().flatten().max().unwrap_or(solid);
                    return Err(Refusal::unsupported(by,format!("the solid #{solid}'s representation is placed twice in its product's shape \
                        (by {} and {}): a shape used more than once is product structure, which is not read yet",said(first.by),said(w.by))))
                }
            }
            let placed = (!same(&motion,&Rigid::identity())).then(|| Placed {by:ways.iter().find_map(|w| w.by).unwrap_or(solid),motion});
            (defined_name(file,first.product).unwrap_or_default(),placed)
        } else {
            // in no product's shape: read where its representation puts it, unless something places it
            let mut tied: Vec<u32> = reps.to_vec();
            let mut seen: BTreeSet<u32> = tied.iter().copied().collect();
            let mut k = 0;
            while k < tied.len() {
                for &t in ties.get(&tied[k]).map(Vec::as_slice).unwrap_or(&[]) { if seen.insert(t) { tied.push(t); } }
                k += 1;
            }
            if let Some(&(r,..)) = moved.iter().find(|m| seen.contains(&m.1)) {
                return Err(Refusal::unsupported(r,format!("a transformation places the solid #{solid}'s representation in a representation \
                    of no product's shape: placed solids outside a product are not read")))
            }
            for (_,items) in &mapping {
                if let Some(&(m,..)) = items.iter().find(|(_,map,_)| map.and_then(|m| maps.get(&m)).and_then(|s| s.1).is_some_and(|s| seen.contains(&s))) {
                    return Err(Refusal::unsupported(m,format!("a mapped item places the solid #{solid}'s representation in a representation \
                        of no product's shape: placed solids outside a product are not read")))
                }
            }
            (tied.iter().flat_map(|r| defined.get(r)).flatten().find_map(|&sdr| product_name(file,sdr)).unwrap_or_default(),None)
        };
        out.push(Found {solid,name,context:context.unwrap(),placed});
    }
    Ok(Shapes {solids:out,beside:beside(file,&solids)})
}
