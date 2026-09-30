//! Meshing a body's material field directly (docs/field-meshing.md): the region and the 1D
//! features every field mesher needs, and `--stl-backend refine`, the core's own Delaunay
//! refinement (`gcs_core::delaunay::refine`). The features are read off the static blank's
//! kernel topology (OCCT) and the field, or with `SOLVENT_FEATURES=field` traced from the field
//! alone by the core (`solid::crease`), as the app does.
use gcs_core::solid::{MaterialField,export::{AtStage,ExportRefusal,Stage}};

fn setting(name: &str,default: f64) -> f64 {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// A body's field and where it lies: the scale to millimetres, and a bounding sphere holding
/// its material with room (outside it is outside the body).
struct Region { field: MaterialField,scale: f64,center: [f64;3],radius: f64,diagonal: f64 }

fn region(sk: &gcs_core::model::Sketch,solid: usize) -> Result<Region,String> {
    let scale = gcs_core::solid::cad::millimetres(sk)?;
    let field = MaterialField::read(sk,solid,gcs_core::solid::cad::AXIS_TOLERANCE)?;
    let support = field.support_bounds().map_err(|e| format!("{e:?}"))?
        .ok_or("the material has no finite support to mesh in")?;
    let (center,diagonal) = gcs_core::space::box_centre_diagonal(&support);
    Ok(Region {field,scale,center,radius:0.5*diagonal*1.05+1e-6,diagonal})
}

/// Mesh `solid` from its field by the core's Delaunay refinement and write the STL at `path`
/// (millimetres), staged through the shell check and the field-agreement gate: a refusal at any
/// stage leaves the old file. Sizes default as the
/// Mesh_3 experiment's do and can be set: `SOLVENT_REFINE_FACET` (facet size, mm),
/// `SOLVENT_REFINE_DISTANCE` (facet distance, mm), `SOLVENT_REFINE_ANGLE` (degrees),
/// `SOLVENT_REFINE_EDGE` (feature spacing, mm).
pub fn export_refine(sk: &gcs_core::model::Sketch,solid: usize,path: &str) -> Result<(),ExportRefusal> {
    let bytes = refine(sk,solid).at(Stage::Refine)?;
    let mut staged = super::output::Staged::new();
    super::output::check_stl(&bytes,"the refined STL fails its shell check").at(Stage::Stl)?;
    staged.write(path,"stl",&bytes).at(Stage::Stl)?;
    super::mark(Stage::Stl);
    super::output::field_agreement(sk,solid,&bytes,None,None)?;
    staged.commit().at(Stage::Written)?;
    super::mark(Stage::Written);
    Ok(())
}

/// The refined mesh of `solid` as STL bytes (millimetres).
fn refine(sk: &gcs_core::model::Sketch,solid: usize) -> Result<Vec<u8>,String> {
    use gcs_core::delaunay::refine::{Criteria,Domain,Progressive,Readings,WithReadings};
    let Region {field,scale,center,radius,diagonal} = region(sk,solid)?;
    let started = std::time::Instant::now();
    let facet = setting("SOLVENT_REFINE_FACET",diagonal*scale/60.)/scale;
    let criteria = Criteria {facet_size:facet,facet_distance:setting("SOLVENT_REFINE_DISTANCE",0.005)/scale,
        facet_angle:setting("SOLVENT_REFINE_ANGLE",25.),edge_size:setting("SOLVENT_REFINE_EDGE",facet*scale)/scale,
        bisection:1e-5*radius,max_points:2_000_000,normal_angle:0.};
    // `SOLVENT_FEATURES=field` finds the features from the field itself, as the app does: a
    // coarse pass without them, and the creases its edges cross (`solid::field_creases`).
    let curves = if std::env::var("SOLVENT_FEATURES").as_deref() == Ok("field") {
        let f = field.clone();
        let (near,within,coarse) = gcs_core::solid::field_first_pass(&field,&criteria,center,radius);
        let mut first = Progressive::new(Box::new(move |p| f.side(p)),near,within,Vec::new(),coarse);
        loop {
            match first.step(usize::MAX) {
                Ok(false) => {}
                Ok(true) => break,
                // the first pass need not close: its edges still cross the creases
                Err(e) => { eprintln!("solventc: refine: the first pass stopped: {e}"); break }
            }
        }
        if let Some(e) = first.done_error() { eprintln!("solventc: refine: the first pass stopped: {e}"); }
        let coarse = first.snapshot();
        eprintln!("solventc: [{:7.1} s] refine: first pass of {} triangles",started.elapsed().as_secs_f64(),coarse.triangles.len());
        let curves = gcs_core::solid::field_creases(&field,&coarse,&criteria,center,radius);
        if let Some(path) = std::env::var_os("SOLVENT_FEATURE_DUMP") {
            let text: String = curves.iter().map(|l| l.iter().map(|q| format!("{} {} {}\n",q[0],q[1],q[2])).collect::<String>()+"\n").collect();
            let _ = std::fs::write(path,text);
        }
        curves
    } else { features(sk,solid,&field,scale,diagonal)? };
    eprintln!("solventc: [{:7.1} s] refine: {} feature curves of {} points",started.elapsed().as_secs_f64(),curves.len(),
        curves.iter().map(Vec::len).sum::<usize>());
    // Time in the field, signs and readings alike, and a line every three seconds.
    let spent = std::cell::Cell::new(std::time::Duration::ZERO);
    let shown = std::cell::Cell::new(0.);
    let timed = |clock: std::time::Instant| {
        spent.set(spent.get()+clock.elapsed());
        let seen = started.elapsed().as_secs_f64();
        if seen > shown.get()+3. {
            shown.set(seen);
            eprintln!("solventc: [{seen:7.1} s] refine: {:.1} s in the field",spent.get().as_secs_f64());
        }
    };
    let side = |p: [f64;3]| { let clock = std::time::Instant::now(); let v = field.side(p); timed(clock); v };
    // Crossings by Newton on the field's value and gradient with `SOLVENT_REFINE_NEWTON=1`. Off by
    // default: a gear's sign queries are cheap (about seven tool evaluations near the boundary),
    // and even warm-started readings leave a gear space 14% dearer than bisecting it
    // (docs/field-meshing.md, "Warm readings").
    let domain: Box<dyn Domain> = if setting("SOLVENT_REFINE_NEWTON",0.) != 0. {
        let mut hints = Vec::new();
        let (field,timed) = (&field,&timed);
        // a reading that continues the last one's crossing continues its contact
        // (`Source::Warm`'s `local`), and so may disagree with `side` there: `Checked`
        Box::new(WithReadings {value:side,readings:Readings::Checked,reading:move |p: [f64;3],warm: bool| {
            use gcs_core::solid::{Query,Source};
            let clock = std::time::Instant::now();
            if !warm { hints.clear(); }
            // a crossing is placed to the bisection tolerance, so its value is needed to a tenth
            // of that, and far from the boundary to a thousandth of itself
            let mut q = Query {accuracy:1e-6*radius,source:Source::Warm {hints:std::mem::take(&mut hints),local:warm},..Query::at(p)};
            let r = field.query(p,&mut q);
            if let Source::Warm {hints:kept,..} = q.source { hints = kept; }
            timed(clock);
            (r.value,r.gradient)
        }})
    } else { Box::new(side) };
    let mut run = Progressive::new(domain,center,radius,curves.clone(),criteria.clone());
    while !run.step(usize::MAX)? {}
    let m = run.finished()?;
    let spent = spent.get();
    eprintln!("solventc: refine: {} triangles in {:?} ({:.1} s in the field), {:?}",m.triangles.len(),started.elapsed(),
        spent.as_secs_f64(),m.report);
    let vertices: Vec<[f64;3]> = m.vertices.iter().map(|v| v.map(|x| x*scale)).collect();
    if std::env::var_os("SOLVENT_REFINE_TRACE").is_some() {
        let f32s = |p: [f64;3]| p.map(|x| x as f32 as f64);
        let flat: Vec<&[u32;3]> = m.triangles.iter().filter(|t| {
            let [a,b,c] = t.map(|i| f32s(vertices[i as usize]));
            gcs_core::mesh::degenerate(a,b,c)
        }).collect();
        eprintln!("solventc: refine: {} triangles flat in float32",flat.len());
        for t in flat.iter().take(5) {
            let [a,b,c] = t.map(|i| vertices[i as usize]);
            let d = gcs_core::space::distance;
            eprintln!("solventc:   {t:?} at {a:?}, edges {:.3e} {:.3e} {:.3e}, flat in f64: {}",d(a,b),d(b,c),d(c,a),
                gcs_core::mesh::degenerate(a,b,c));
        }
    }
    let bytes = gcs_core::mesh::indexed_stl(&vertices,&m.triangles,&sk.solids[solid].name)?;
    Ok(bytes)
}

/// The 1D features of blank-minus-sweeps, in model units, read by the core
/// (`solid::blank_features`) off the static blank's native topology and the field; the
/// `SOLVENT_FEATURE_*` switches say more about them on stderr.
fn features(sk: &gcs_core::model::Sketch,solid: usize,field: &MaterialField,scale: f64,diagonal: f64)
    -> Result<Vec<Vec<[f64;3]>>,String> {
    use super::native::{Session,features::NativeBlank};
    use gcs_core::space::polyline_length;
    let started = std::time::Instant::now();
    let session = Session::new()?;
    let blank = session.construct_recipe(&gcs_core::solid::cad::recipe_static(sk,solid)?.recipe)?;
    let topology = NativeBlank::new(&session,blank)?;
    // `SOLVENT_FEATURE_GRID`: cells a face's contour grid has along each parameter.
    let found = gcs_core::solid::blank_features::features(&topology,field,scale,diagonal,setting("SOLVENT_FEATURE_GRID",64.) as usize)?;
    eprintln!("solventc:   edge features in {:?}",started.elapsed().saturating_sub(found.contour_time));
    eprintln!("solventc:   face contours in {:?} ({} field readings taking {:.1} s)",found.contour_time,
        found.readings.0,found.readings.1);
    for l in &found.lines_mm {
        let length = polyline_length(l);
        if std::env::var_os("SOLVENT_FEATURE_SEAM").is_some() {
            let near: Vec<(usize,[f64;3])> = l.iter().copied().enumerate().filter(|(_,p)| p[1].abs() < 0.15 && p[0] > 3.8)
                .map(|(k,p)| (k,p.map(|x| (x*1e4).round()/1e4))).collect();
            if !near.is_empty() { eprintln!("solventc:   near the seam: {near:?}"); }
        }
        if std::env::var_os("SOLVENT_FEATURE_KINKS").is_some() {
            for w in l.windows(3) {
                use gcs_core::space::{sub,dot,norm};
                let (u,v) = (sub(w[1],w[0]),sub(w[2],w[1]));
                let angle = (dot(u,v)/(norm(u)*norm(v))).clamp(-1.,1.).acos().to_degrees();
                if angle > 30. { eprintln!("solventc:   turn of {angle:.0}° at {:?}",w[1].map(|x| (x*1e4).round()/1e4)); }
            }
        }
        if std::env::var_os("SOLVENT_FEATURE_ENDS").is_some() {
            eprintln!("solventc:   feature of {} points, {:.4} mm, from {:?} to {:?}",l.len(),length,l[0],l.last().unwrap());
        } else {
            eprintln!("solventc:   feature of {} points, {:.4} mm, from {:?} to {:?}",l.len(),length,
                l[0].map(|x| (x*1e4).round()/1e4),l.last().unwrap().map(|x| (x*1e4).round()/1e4));
        }
    }
    if let Some(path) = std::env::var_os("SOLVENT_FEATURE_DUMP") {
        // One line a point, a blank line between curves, in model coordinates.
        let text: String = found.lines.iter().map(|l| l.iter().map(|q| format!("{} {} {}\n",q[0],q[1],q[2]))
            .collect::<String>()+"\n").collect();
        let _ = std::fs::write(path,text);
    }
    Ok(found.lines)
}
