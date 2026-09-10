//! A body with continuous swept cuts as a mesh, by arrangement of candidate
//! sheets and classification by the declared material field.
//!
//! The static remainder of the body is meshed by the core's own facet evaluator
//! and is the blank. Each sweep's candidate sheet (its contact grid) becomes a
//! thin closed slab, offset either side of the sheet along the contact normals;
//! the tool at both end poses is an exact cap solid. Manifold removes the caps
//! from the blank outright (the tool occupies them), then splits the remainder
//! by every slab. Each cell is judged by one material probe a few slab widths
//! inside its largest triangle, and each slab piece by the field at its
//! mid-surface: strictly material means a hidden wall, kept; otherwise it is the
//! skin of an exposed boundary and is dropped. Material cells and kept walls are
//! united. No trimming or visibility is computed here; an unresolved or mixed
//! cell refuses the build.
use super::manifold::Solid;
use gcs_core::{interval::{Interval,minimum::Options},mesh,model::{Sketch,SolidDef},
    motion::Family,solid::{self,cad,MaterialField,ProbeState}};

fn stage(message: &str) { eprintln!("solventc: {message}"); }
fn sub(a: [f64;3],b: [f64;3]) -> [f64;3] { std::array::from_fn(|k| a[k]-b[k]) }
fn cross(a: [f64;3],b: [f64;3]) -> [f64;3] { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: [f64;3]) -> f64 { (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt() }

/// A candidate sheet in model units: a row-major grid of contact positions with
/// the cutter's outward normal at each. `closed_rows` joins the last row back
/// to the first, a tube.
pub struct SheetGrid { pub points: Vec<[f64;3]>,pub normals: Vec<[f64;3]>,pub rows: usize,pub columns: usize,pub closed_rows: bool }

/// A closed slab of thickness `epsilon` centred on the sheet.
pub fn slab(sheet: &SheetGrid,epsilon: f64) -> Result<Solid,String> {
    let (rows,columns) = (sheet.rows,sheet.columns);
    let n = rows*columns;
    let mut vertices = Vec::with_capacity(2*n);
    for side in [1.,-1.] {
        for (p,normal) in sheet.points.iter().zip(&sheet.normals) {
            vertices.push(std::array::from_fn(|k| p[k]+side*0.5*epsilon*normal[k]));
        }
    }
    let at = |r: usize,c: usize,top: bool| (if top { 0 } else { n }+(r%rows)*columns+c) as u32;
    let row_spans = if sheet.closed_rows { rows } else { rows-1 };
    let mut triangles = Vec::with_capacity(4*row_spans*(columns-1)+4*(rows+columns)*2);
    for r in 0..row_spans { for c in 0..columns-1 {
        let (a,b,d,e) = (at(r,c,true),at(r,c+1,true),at(r+1,c,true),at(r+1,c+1,true));
        triangles.push([a,b,e]); triangles.push([a,e,d]);
        let (a,b,d,e) = (at(r,c,false),at(r,c+1,false),at(r+1,c,false),at(r+1,c+1,false));
        triangles.push([a,e,b]); triangles.push([a,d,e]);
    } }
    // Side walls: for each directed boundary edge p->q of the top surface, the
    // wall quad carries q->p on top and p'->q' below, so every edge pairs. A
    // tube has walls only along its two end columns.
    let mut wall = |p: u32,q: u32| { let (pb,qb) = (p+n as u32,q+n as u32); triangles.push([q,p,pb]); triangles.push([q,pb,qb]); };
    if !sheet.closed_rows {
        for c in 0..columns-1 { wall(at(0,c,true),at(0,c+1,true)); wall(at(rows-1,c+1,true),at(rows-1,c,true)); }
    }
    for r in 0..row_spans { wall(at(r,columns-1,true),at(r+1,columns-1,true)); wall(at(r+1,0,true),at(r,0,true)); }
    let solid = Solid::from_triangles(&vertices,&triangles)?;
    if solid.volume() < 0. {
        // The grid's orientation is the sheet's; wind the other way if the
        // offset side turned out to be inward.
        let flipped: Vec<[u32;3]> = triangles.iter().map(|t| [t[0],t[2],t[1]]).collect();
        return Solid::from_triangles(&vertices,&flipped);
    }
    Ok(solid)
}

/// Evaluate a static solid's Boolean term with Manifold over its primitives.
pub fn solid_of(fixed: &solid::StaticSolid) -> Result<Solid,String> {
    let mut prims: Vec<Option<Solid>> = Vec::with_capacity(fixed.csg.prims.len());
    for prim in &fixed.csg.prims {
        let (vertices,triangles) = solid::primitive_triangles(prim,fixed.origin);
        prims.push(Some(Solid::from_triangles(&vertices,&triangles).map_err(|e| format!("`{}`: {e}",prim.of))?));
    }
    fn evaluate(term: &solid::Term,prims: &[Option<Solid>]) -> Result<Solid,String> {
        Ok(match term {
            solid::Term::Prim(i) => {
                let (vertices,triangles) = prims[*i].as_ref().unwrap().triangles()?;
                Solid::from_triangles(&vertices,&triangles)?
            }
            solid::Term::Union(a,b) => evaluate(a,prims)?.union(&evaluate(b,prims)?)?,
            solid::Term::Diff(a,b) => evaluate(a,prims)?.difference(&evaluate(b,prims)?)?,
            solid::Term::Inter(a,b) => evaluate(a,prims)?.intersection(&evaluate(b,prims)?)?,
            solid::Term::Empty => return Err("an empty operand cannot be meshed".into()),
        })
    }
    evaluate(&fixed.csg.term,&prims)
}

/// A point inside the solid, `depth` behind its largest triangle, with that
/// triangle's outward normal.
pub fn interior_point(solid: &Solid,depth: f64) -> Result<[f64;3],String> {
    let (vertices,triangles) = solid.triangles()?;
    let mut best: Option<(f64,[f64;3],[f64;3])> = None;
    for t in &triangles {
        let [a,b,c] = t.map(|i| vertices[i as usize]);
        let n = cross(sub(b,a),sub(c,a));
        let area = norm(n);
        if area <= 0. { continue; }
        if best.as_ref().is_none_or(|(a,_,_)| area > *a) {
            best = Some((area,std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.),n.map(|v| v/area)));
        }
    }
    let (_,centroid,normal) = best.ok_or("a cell has no triangles")?;
    Ok(std::array::from_fn(|k| centroid[k]-depth*normal[k]))
}

/// Sheets of a sweep whose tool-frame twist is constant: each characteristic
/// carried along the motion over the declared interval, plus a slab's worth of
/// overrun into the caps at both ends.
pub fn constant_twist_sheets(sk: &Sketch,swept: usize,epsilon: f64) -> Result<Vec<SheetGrid>,String> {
    let sweep = solid::SweepContacts::read(sk,swept,1e-10)?;
    let [from,to] = sweep.domain();
    let curves = sweep.characteristics(1e-9)?;
    if curves.is_empty() { return Err(format!("`{}`: the tool has no contact curve under its motion",sk.solids[swept].name)); }
    let motion = sweep.motion();
    let mut sheets = Vec::with_capacity(curves.len());
    for curve in curves {
        let speed = curve.points.iter().map(|p| norm(motion.at(from).map(|m| m.velocity(*p)).unwrap_or([0.;3])))
            .fold(0_f64,f64::max).max(1e-9);
        let overrun = 2.*epsilon/speed;
        let (t0,t1) = (from-overrun,to+overrun);
        let columns = (((t1-t0)*speed/COLUMN_SPACING).ceil() as usize).clamp(2,400);
        let mut points = Vec::with_capacity(curve.points.len()*columns);
        let mut normals = Vec::with_capacity(curve.points.len()*columns);
        for (p,n) in curve.points.iter().zip(&curve.normals) {
            for c in 0..columns {
                let pose = motion.at(t0+(t1-t0)*c as f64/(columns-1) as f64)?;
                points.push(pose.point(*p)); normals.push(pose.vector(*n));
            }
        }
        sheets.push(SheetGrid {points,normals,rows:curve.points.len(),columns,closed_rows:curve.closed});
    }
    Ok(sheets)
}

/// Target node spacing along a sheet's band, model millimetres.
const COLUMN_SPACING: f64 = 0.5;

/// The body as indexed triangles in model units. `sheets` supplies the
/// candidate sheets of a sweep that is not constant-twist, given a containment
/// oracle for the blank in native millimetres.
pub fn construct(sk: &Sketch,body: usize,sheets: &dyn Fn(usize,&dyn Fn(&[[f64;3]]) -> Result<Vec<bool>,String>) -> Result<Vec<SheetGrid>,String>)
    -> Result<(Vec<[f64;3]>,Vec<[u32;3]>),String> {
    let recipe = cad::recipe_static(sk,body)?;
    let scale = sk.units.length.ok_or("CAD construction requires an explicit length unit")?.1;
    let name = sk.solids[body].name.clone();
    let started = std::time::Instant::now();
    let blank = solid::static_solid(sk,body,recipe.sweeps.len())?;
    let blank_mesh = solid_of(&blank)?;
    stage(&format!("`{name}`: blank meshed, {} triangles, {:.6} mm³ ({:?})",blank_mesh.triangle_count(),
        blank_mesh.volume()*scale.powi(3),started.elapsed()));
    // Slab thickness: a micrometre in native millimetres, expressed in model units.
    let epsilon = 1e-3/scale;
    // Chordal facets of a subtracted round face lie outside the true surface by
    // up to the tessellation sagitta, so a cell is judged deeper than that.
    let sagitta = blank.unit*gcs_core::curve::FLATNESS_PX;
    let inside = |points: &[[f64;3]]| -> Result<Vec<bool>,String> { Ok(points.iter().map(|p| blank.contains(p.map(|v| v/scale))).collect()) };
    let mut distinct: Vec<usize> = recipe.sweeps.iter().map(|s| s.swept).collect();
    distinct.sort(); distinct.dedup();
    let (mut slabs,mut caps) = (Vec::new(),Vec::new());
    for swept in distinct {
        let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else { unreachable!() };
        let started = std::time::Instant::now();
        let family = Family::read(sk,*motion as usize)?;
        let grids = if solid::constant_twist(&family,[from.value,to.value],1e-9)? {
            constant_twist_sheets(sk,swept,epsilon)?
        } else { sheets(swept,&inside)? };
        let sheet_solids = grids.iter().map(|g| slab(g,epsilon)).collect::<Result<Vec<_>,_>>()?;
        let tool = solid_of(&solid::static_solid(sk,*source as usize,0)?)?;
        let ends = [family.at(from.value)?,family.at(to.value)?];
        for cut in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            for sheet in &sheet_solids { slabs.push(sheet.placed(&cad::placement_matrix(cut.pose,1.))?); }
            for end in ends {
                let placed = tool.placed(&cad::placement_matrix(end.then(cut.pose),1.))?;
                caps.push(placed);
            }
        }
        stage(&format!("`{}`: {} sheets of {} triangles, {} placements ({:?})",sk.solids[swept].name,sheet_solids.len(),
            sheet_solids.iter().map(|s| s.triangle_count()).sum::<usize>(),
            recipe.sweeps.iter().filter(|c| c.swept == swept).count(),started.elapsed()));
    }
    let started = std::time::Instant::now();
    let cap_union = Solid::batch(&caps.iter().collect::<Vec<_>>(),true)?;
    let body_mesh = blank_mesh.difference(&cap_union)?;
    let removed = blank_mesh.volume()-body_mesh.volume();
    let slab_union = Solid::batch(&slabs.iter().collect::<Vec<_>>(),true)?;
    let (walls,remainder) = body_mesh.split(&slab_union)?;
    let cells = remainder.components()?;
    let wall_pieces = walls.components()?;
    stage(&format!("caps removed {:.6} mm³; {} cells and {} wall pieces ({:?})",removed*scale.powi(3),cells.len(),wall_pieces.len(),started.elapsed()));
    let started = std::time::Instant::now();
    let mut material = MaterialField::read(sk,body,1e-10)?.evaluator(4096);
    let mut kept: Vec<Solid> = Vec::new();
    let (mut removed_cells,mut hidden_walls) = (0,0);
    for cell in cells {
        // A point deep inside the cell: a quarter of its mean thickness, but at
        // least past the tessellation sagitta and the slab. The swept field
        // resolves far faster away from the envelope than beside it.
        let thickness = 2.*cell.volume()/cell.area().max(f64::MIN_POSITIVE);
        let depth = (0.25*thickness).clamp(3.*sagitta+3.*epsilon,0.5/scale);
        let p = interior_point(&cell,depth)?;
        let distance = (depth/3.).max(1.5*epsilon);
        let probe = material.probe(p.map(|x| Interval::point(x).unwrap()),[1.,0.,0.],distance,
            Options {value_tolerance:distance/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))?;
        if std::env::var_os("SOLVENT_TRACE_CELLS").is_some() {
            eprintln!("  cell {:.6} mm³ at {p:?}: {:?}, field {:?}",cell.volume()*scale.powi(3),probe.state,probe.center.value.bounds());
        }
        match probe.state {
            ProbeState::InteriorBall => kept.push(cell),
            ProbeState::ExteriorBall => removed_cells += 1,
            state => return Err(format!("the material at {p:?}, inside a cell of {:.6} mm³, is {state:?}",cell.volume()*scale.powi(3))),
        }
    }
    let cells_time = started.elapsed();
    let hidden_depth = (0.02/scale).max(3.*sagitta);
    for wall in wall_pieces {
        let p = interior_point(&wall,0.5*epsilon)?;
        // A hidden wall is well inside material; an exposed one reads near zero.
        // Only that distinction is needed, so the field is refined coarsely and
        // may stop as soon as a sweep is clear of the band.
        let bounds = material.bounds_outside(p.map(|x| Interval::point(x).unwrap()),
            Interval::new(-hidden_depth,hidden_depth).unwrap(),
            Options {value_tolerance:hidden_depth/2.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))?;
        if bounds.value.bounds()[1] < -hidden_depth { kept.push(wall); hidden_walls += 1; }
    }
    stage(&format!("classified {} material cells, {removed_cells} removed cells ({cells_time:?}), {hidden_walls} hidden walls ({:?})",
        kept.len(),started.elapsed()));
    if kept.is_empty() { return Err("no cell of the blank is material".into()); }
    let started = std::time::Instant::now();
    let part = Solid::batch(&kept.iter().collect::<Vec<_>>(),true)?;
    stage(&format!("united the material: {:.6} mm³, {} triangles ({:?})",part.volume()*scale.powi(3),part.triangle_count(),started.elapsed()));
    // The union is written as it comes: Manifold's own simplification pinches
    // vertices of its own, and a solid that touches itself along an edge is
    // refused by the STL shell check below with the edge named.
    part.triangles()
}

/// Vertices identified by their float32 encoding, and the triangles that survive
/// the identification: STL carries no vertex identity, so a sliver between two
/// coincident vertices is dropped rather than encoded as a degenerate facet.
fn merged(vertices: &[[f64;3]],triangles: &[[u32;3]]) -> (Vec<[f64;3]>,Vec<[u32;3]>) {
    let mut index: std::collections::HashMap<[u32;3],u32> = Default::default();
    let mut merged: Vec<[f64;3]> = Vec::with_capacity(vertices.len());
    let remap: Vec<u32> = vertices.iter().map(|v| *index.entry(v.map(|x| (x as f32).to_bits())).or_insert_with(|| {
        merged.push(*v); (merged.len()-1) as u32 })).collect();
    let kept = triangles.iter().map(|t| t.map(|i| remap[i as usize]))
        .filter(|t| t[0] != t[1] && t[1] != t[2] && t[2] != t[0]).collect();
    (merged,kept)
}

/// Binary STL of the indexed triangles through the core's checked writer, with
/// the encoded shell topology verified. STL carries no vertex identity, so
/// vertices the encoding identifies are merged first and the triangles that
/// collapse under the merge (a sliver between two coincident vertices) dropped;
/// a genuine pinch, three faces on one segment, still refuses.
pub fn stl(vertices: &[[f64;3]],triangles: &[[u32;3]],name: &str) -> Result<Vec<u8>,String> {
    let (vertices,triangles) = merged(vertices,triangles);
    let (vertices,triangles) = (&vertices[..],&triangles[..]);
    let bytes = mesh::indexed_stl(vertices,triangles,name)?;
    if let Err(e) = mesh::stl_shells(&bytes) {
        // Distinct vertices that float32 identifies, and the shortest edges.
        let mut by_key: std::collections::HashMap<[u32;3],Vec<usize>> = Default::default();
        for (i,v) in vertices.iter().enumerate() { by_key.entry(v.map(|x| (x as f32).to_bits())).or_default().push(i); }
        let merged: Vec<_> = by_key.values().filter(|v| v.len() > 1).take(3)
            .map(|v| format!("{:?}",v.iter().map(|&i| vertices[i]).collect::<Vec<_>>())).collect();
        let mut shortest = f64::INFINITY;
        for t in triangles { for k in 0..3 {
            let (a,b) = (vertices[t[k] as usize],vertices[t[(k+1)%3] as usize]);
            shortest = shortest.min(((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt());
        } }
        let mut uses: std::collections::HashMap<(u32,u32),Vec<usize>> = Default::default();
        for (i,t) in triangles.iter().enumerate() { for k in 0..3 {
            let (a,b) = (t[k],t[(k+1)%3]); uses.entry((a.min(b),a.max(b))).or_default().push(i);
        } }
        let mut bad: Vec<String> = uses.iter().filter(|(_,v)| v.len() != 2).take(4).map(|((a,b),v)| format!("edge {:?}-{:?} used by {}",
            vertices[*a as usize],vertices[*b as usize],v.iter().map(|&i| format!("{:?}",triangles[i].map(|j| vertices[j as usize]))).collect::<Vec<_>>().join(", "))).collect();
        bad.sort();
        return Err(format!("mesh STL validation failed: {e}; {} float32-identified vertex groups, e.g. {}; shortest edge {shortest:e}; {}",
            by_key.values().filter(|v| v.len() > 1).count(),merged.join(" | "),bad.join("\n")));
    }
    Ok(bytes)
}
