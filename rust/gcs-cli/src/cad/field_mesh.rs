//! Meshing a body's material field directly (docs/field-meshing.md): the region and the 1D
//! features every field mesher needs, and `--stl-backend refine`, the core's own Delaunay
//! refinement (`gcs_core::delaunay::refine`). The features are read off the static blank's
//! kernel topology (OCCT) and the field, or with `SOLVENT_FEATURES=field` traced from the field
//! alone by the core (`solid::crease`), as the app does.
use gcs_core::solid::MaterialField;

pub fn setting(name: &str,default: f64) -> f64 {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// A body's field and where it lies: the scale to millimetres, and a bounding sphere holding
/// its material with room (outside it is outside the body).
pub struct Region { pub field: MaterialField,pub scale: f64,pub center: [f64;3],pub radius: f64,pub diagonal: f64 }

pub fn region(sk: &gcs_core::model::Sketch,solid: usize) -> Result<Region,String> {
    let scale = sk.units.length.ok_or("CAD export requires an explicit model length unit")?.1;
    let field = MaterialField::read(sk,solid,1e-10)?;
    let support = field.support_bounds().map_err(|e| format!("{e:?}"))?
        .ok_or("the material has no finite support to mesh in")?;
    let [lo,hi] = [0,1].map(|k| support.map(|x| x.bounds()[k]));
    let center: [f64;3] = std::array::from_fn(|k| 0.5*(lo[k]+hi[k]));
    let diagonal = (0..3).map(|k| (hi[k]-lo[k]).powi(2)).sum::<f64>().sqrt();
    Ok(Region {field,scale,center,radius:0.5*diagonal*1.05+1e-6,diagonal})
}

/// Mesh `solid` from its field by the core's Delaunay refinement and write the STL at `path`
/// (millimetres), through the shell check and the field-agreement gate. Sizes default as the
/// Mesh_3 experiment's do and can be set: `SOLVENT_REFINE_FACET` (facet size, mm),
/// `SOLVENT_REFINE_DISTANCE` (facet distance, mm), `SOLVENT_REFINE_ANGLE` (degrees),
/// `SOLVENT_REFINE_EDGE` (feature spacing, mm).
pub fn export_refine(sk: &gcs_core::model::Sketch,solid: usize,path: &str) -> Result<(),String> {
    use gcs_core::delaunay::refine::{Criteria,Progressive};
    let Region {field,scale,center,radius,diagonal} = region(sk,solid)?;
    let started = std::time::Instant::now();
    let facet = setting("SOLVENT_REFINE_FACET",diagonal*scale/60.)/scale;
    let criteria = Criteria {facet_size:facet,facet_distance:setting("SOLVENT_REFINE_DISTANCE",0.005)/scale,
        facet_angle:setting("SOLVENT_REFINE_ANGLE",25.),edge_size:setting("SOLVENT_REFINE_EDGE",facet*scale)/scale,
        bisection:1e-5*radius,max_points:2_000_000};
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
    let mut run = Progressive::new(Box::new(side),center,radius,curves.clone(),criteria.clone());
    // Crossings by Newton on the field's value and gradient with `SOLVENT_REFINE_NEWTON=1`. Off by
    // default: a gear's sign queries are cheap (about seven tool evaluations near the boundary),
    // and even warm-started readings leave a gear space 14% dearer than bisecting it
    // (docs/field-meshing.md, "Warm readings").
    if setting("SOLVENT_REFINE_NEWTON",0.) != 0. {
        let mut hints = Vec::new();
        let (field,timed) = (&field,&timed);
        run = run.with_reading(Box::new(move |p: [f64;3],warm: bool| {
            let clock = std::time::Instant::now();
            // a crossing is placed to the bisection tolerance, so its value is needed to a tenth
            // of that, and far from the boundary to a thousandth of itself
            let options = gcs_core::solid::ReadingOptions {accuracy:1e-6*radius,local:warm,..gcs_core::solid::ReadingOptions::at(p)};
            if !warm { hints.clear(); }
            let r = field.reading_warm(p,&options,&mut 0,&mut hints);
            timed(clock);
            (r.value,r.gradient)
        }));
    }
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
            let d = |p: [f64;3],q: [f64;3]| (0..3).map(|k| (p[k]-q[k]).powi(2)).sum::<f64>().sqrt();
            eprintln!("solventc:   {t:?} at {a:?}, edges {:.3e} {:.3e} {:.3e}, flat in f64: {}",d(a,b),d(b,c),d(c,a),
                gcs_core::mesh::degenerate(a,b,c));
        }
    }
    let bytes = gcs_core::mesh::indexed_stl(&vertices,&m.triangles,&sk.solids[solid].name)?;
    gcs_core::mesh::stl_shells(&bytes).map_err(|e| format!("the refined STL fails its shell check: {e}"))?;
    std::fs::write(path,&bytes).map_err(|e| format!("{path}: {e}"))?;
    super::mark("stl");
    super::native::field_agreement(sk,solid,&bytes)?;
    super::mark("written");
    Ok(())
}

/// The 1D features of blank-minus-sweeps, in model units. A point `p` on a blank face (outward
/// normal `n`) is material exactly when `p − εn` is, so the material field read that far inside
/// says whether a sweep took it: along each sharp blank edge (read along the inward bisector)
/// the surviving stretches are features, their ends found by bisection as corners; over each
/// blank face, the zero contour of that reading on the face's parameter grid (marching squares,
/// crossings bisected) is where the swept boundary meets the face. A contour ends where the
/// grid meets a trim or a seam, a cell short of the edge, and there it is joined to the corner
/// it runs to, or to a contour it continues across a seam, so features meet only at endpoints.
pub fn features(sk: &gcs_core::model::Sketch,solid: usize,field: &MaterialField,scale: f64,diagonal: f64)
    -> Result<Vec<Vec<[f64;3]>>,String> {
    use super::native::Session;
    type P = [f64;3];
    let dist = |a: P,b: P| ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt();
    let started = std::time::Instant::now();
    let session = Session::new()?;
    let blank = session.construct_recipe(&gcs_core::solid::cad::recipe_static(sk,solid)?.recipe)?;
    let size = diagonal*scale;
    let eps = 1e-4*size;
    let field_time = std::cell::Cell::new((0usize,0f64));
    let within = |p: P,n: P| -> bool {
        let q: P = std::array::from_fn(|k| (p[k]-eps*n[k])/scale);
        let clock = std::time::Instant::now();
        let value = field.side(q);
        let inside = value < 0.;
        let (c,t) = field_time.get();
        field_time.set((c+1,t+clock.elapsed().as_secs_f64()));
        inside
    };
    let model = |p: P| -> P { p.map(|x| x/scale) };
    let mut lines: Vec<Vec<P>> = Vec::new();
    let mut corners: Vec<P> = Vec::new();
    // Sharp edges.
    for edge in session.boundary(blank)? {
        if edge.seam_or_pole() { continue; }
        let steps = 200;
        let at = |f: f64| -> Result<(P,bool,f64),String> {
            let e = session.boundary_point(&edge,f)?;
            let [a,b] = e.normals;
            let n = [a[0]+b[0],a[1]+b[1],a[2]+b[2]];
            let l = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt().max(1e-12);
            Ok((e.position,within(e.position,n.map(|x| x/l)),e.dihedral))
        };
        let samples: Vec<(P,bool,f64)> = (0..=steps).map(|i| at(i as f64/steps as f64)).collect::<Result<_,_>>()?;
        if samples.iter().all(|s| s.2.abs() < 1e-3) { continue; }
        let mut pieces: Vec<Vec<P>> = Vec::new();
        let mut current: Vec<P> = Vec::new();
        for i in 0..=steps {
            let (p,inside,_) = samples[i];
            if i > 0 && inside != samples[i-1].1 {
                let (mut lo,mut hi) = ((i-1) as f64/steps as f64,i as f64/steps as f64);
                for _ in 0..50 { let m = 0.5*(lo+hi); if at(m)?.1 == samples[i-1].1 { lo = m } else { hi = m } }
                let corner = at(0.5*(lo+hi))?.0;
                corners.push(corner);
                current.push(corner);
                if !current.is_empty() && samples[i-1].1 { pieces.push(std::mem::take(&mut current)); }
                current = vec![corner];
            }
            if inside { current.push(p); }
        }
        if samples[steps].1 && current.len() > 1 { pieces.push(current); }
        // A closed edge whose surviving stretch runs through its start is one piece.
        let closed = dist(samples[0].0,samples[steps].0) < 1e-9*size;
        if closed && pieces.len() > 1 && samples[0].1 && samples[steps].1 {
            let first = pieces.remove(0);
            pieces.last_mut().unwrap().extend(first.into_iter().skip(1));
        }
        lines.extend(pieces.into_iter().filter(|p| p.len() > 1));
    }
    eprintln!("solventc:   edge features in {:?}",started.elapsed());
    let edges_done = std::time::Instant::now();
    // Face contours.
    let mut contours: Vec<Vec<P>> = Vec::new();
    // `SOLVENT_FEATURE_GRID`: cells a face's contour grid has along each parameter.
    let n = setting("SOLVENT_FEATURE_GRID",64.) as usize;
    for face in session.faces(blank)? {
        let tolerance = 1e-9;
        // A direction the face spans a whole period of is sampled at cell centres and wraps, so
        // a contour crosses its seam like any other cell edge; a bounded one runs to its trims.
        let seams = session.face_seams(face)?;
        let axis = |periodic: bool| -> (Vec<f64>,Vec<(usize,usize)>) {
            if periodic { ((0..n).map(|i| (i as f64+0.5)/n as f64).collect(),(0..n).map(|i| (i,(i+1)%n)).collect()) }
            else { ((0..=n).map(|i| i as f64/n as f64).collect(),(0..n).map(|i| (i,i+1)).collect()) }
        };
        let ((us,ucells),(vs,vcells)) = (axis(seams[0]),axis(seams[1]));
        let grid: Vec<Vec<Option<(P,bool)>>> = us.iter().map(|&u| vs.iter().map(|&v| {
            // A point on the face's trim is on another face too, and read just inside this one
            // it is read on that other: the edge features cover the trim.
            session.face_point(face,u,v,tolerance)
                .map(|fp| fp.filter(|fp| !fp.on_trim).map(|fp| (fp.position,within(fp.position,fp.normal))))
        }).collect::<Result<Vec<_>,_>>()).collect::<Result<_,_>>()?;
        // Parameter from one sample to the next, unwrapped across a seam.
        let step = |values: &[f64],a: usize,b: usize| {
            let d = values[b]-values[a];
            if d < -0.5 { d+1. } else if d > 0.5 { d-1. } else { d }
        };
        // The crossing on the grid edge from (i0, j0) to (i1, j1), by bisection on the face.
        let mut crossings: std::collections::BTreeMap<(usize,usize,usize,usize),P> = Default::default();
        let mut crossing = |a: (usize,usize),b: (usize,usize)| -> Result<Option<P>,String> {
            let key = (a.0.min(b.0),a.1.min(b.1),a.0.max(b.0),a.1.max(b.1));
            if let Some(p) = crossings.get(&key) { return Ok(Some(*p)); }
            let (Some(sa),Some(_)) = (grid[a.0][a.1],grid[b.0][b.1]) else { return Ok(None) };
            let (du,dv) = (step(&us,a.0,b.0),step(&vs,a.1,b.1));
            let uv = |t: f64| ((us[a.0]+t*du).rem_euclid(1.),(vs[a.1]+t*dv).rem_euclid(1.));
            let (mut lo,mut hi) = (0.,1.);
            let mut last = None;
            for _ in 0..16 {
                let m = 0.5*(lo+hi);
                let (u,v) = uv(m);
                let Some(fp) = session.face_point(face,u,v,tolerance)? else { return Ok(None) };
                last = Some(fp.position);
                if within(fp.position,fp.normal) == sa.1 { lo = m } else { hi = m }
            }
            if let Some(p) = last { crossings.insert(key,p); }
            Ok(last)
        };
        let mut segments: Vec<(P,P)> = Vec::new();
        for &(i,i1) in &ucells { for &(j,j1) in &vcells {
            let c = [(i,j),(i1,j),(i1,j1),(i,j1)];
            let Some(states) = c.iter().map(|&(a,b)| grid[a][b].map(|g| g.1)).collect::<Option<Vec<bool>>>() else { continue };
            let mut points = Vec::new();
            for k in 0..4 {
                if states[k] != states[(k+1)%4] {
                    if let Some(p) = crossing(c[k],c[(k+1)%4])? { points.push(p); }
                }
            }
            match points.len() {
                2 => segments.push((points[0],points[1])),
                // A saddle: pair the crossings so the material corners are kept apart.
                4 => { if states[0] { segments.push((points[0],points[1])); segments.push((points[2],points[3])); }
                       else { segments.push((points[1],points[2])); segments.push((points[3],points[0])); } }
                _ => {}
            }
        } }
        contours.extend(chain(segments,1e-12*size));
    }
    // Join contours across seams, then to the corners they run to.
    let reach = 4.*size/64.;
    eprintln!("solventc:   face contours in {:?} ({} field readings taking {:.1} s)",edges_done.elapsed(),
        field_time.get().0,field_time.get().1);
    let mut joined = chain_lines(contours,1e-7*size);
    // A contour broken where the grid skips a seam's trim points is closed across the gap,
    // or joined to the contour that carries on beyond it, where no corner lies between.
    let open = |l: &Vec<P>| l.len() > 1 && dist(l[0],*l.last().unwrap()) > 1e-9*size;
    let cornered = |p: P| corners.iter().any(|c| dist(*c,p) < reach);
    // Each end with no corner by it is joined to the nearest such end of another contour within
    // reach, or closes its own contour when its other end is that end: one join at a time, the
    // closest first, until none is left.
    loop {
        let ends: Vec<(usize,bool,P)> = joined.iter().enumerate().filter(|(_,l)| open(l))
            .flat_map(|(k,l)| [(k,false,l[0]),(k,true,*l.last().unwrap())])
            .filter(|&(_,_,p)| !cornered(p)).collect();
        let mut best: Option<(f64,usize,usize)> = None;
        for x in 0..ends.len() { for y in x+1..ends.len() {
            let d = dist(ends[x].2,ends[y].2);
            if d < reach && best.is_none_or(|b| d < b.0) { best = Some((d,x,y)); }
        } }
        let Some((_,x,y)) = best else { break };
        let ((a,a_tail,_),(b,b_tail,_)) = (ends[x],ends[y]);
        if a == b {
            let first = joined[a][0];
            joined[a].push(first);
            continue;
        }
        // Put a's joining end last and b's first, then append b to a.
        let mut first = joined[a].clone();
        if !a_tail { first.reverse(); }
        let mut second = joined[b].clone();
        if b_tail { second.reverse(); }
        first.extend(second);
        let (keep,drop) = (a.min(b),a.max(b));
        joined[keep] = first;
        joined.remove(drop);
    }
    for line in &mut joined {
        if line.len() < 2 || dist(line[0],*line.last().unwrap()) < 1e-9*size { continue; }
        for end in [0,1] {
            let p = if end == 0 { line[0] } else { *line.last().unwrap() };
            if let Some(&c) = corners.iter().filter(|c| dist(**c,p) < reach).min_by(|a,b| dist(**a,p).total_cmp(&dist(**b,p))) {
                if end == 0 { line.insert(0,c) } else { line.push(c) }
            }
        }
    }
    lines.extend(joined);
    // No two consecutive points closer than a millionth of the size: a zero-length segment
    // leaves Mesh_3's protecting balls nowhere to go.
    let lines: Vec<Vec<P>> = lines.into_iter().map(|l| {
        let closed = l.len() > 2 && dist(l[0],*l.last().unwrap()) < 1e-9*size;
        // The ends are kept exactly (a corner shared with another curve must stay bit-identical),
        // so a point near the last one gives way to it rather than the other way round.
        let last = *l.last().unwrap();
        let mut kept: Vec<P> = Vec::new();
        for (k,p) in l.iter().copied().enumerate() {
            let near = kept.last().is_some_and(|q| dist(*q,p) <= 1e-6*size);
            if !near { kept.push(p); }
            else if k+1 == l.len() && kept.len() > 1 { *kept.last_mut().unwrap() = p; }
        }
        if kept.last() != Some(&last) && kept.len() > 1 { *kept.last_mut().unwrap() = last; }
        if closed && kept.len() > 2 { let first = kept[0]; *kept.last_mut().unwrap() = first; }
        kept
    // A closed walk of fewer than three distinct points is a segment walked out and back — two
    // face contours touching within a grid cell — and bounds nothing.
    }).filter(|l| l.len() > 1 && !(l.len() < 4 && l[0] == *l.last().unwrap())).collect();
    for l in &lines {
        let length: f64 = l.windows(2).map(|w| dist(w[0],w[1])).sum();
        if std::env::var_os("SOLVENT_FEATURE_SEAM").is_some() {
            let near: Vec<(usize,[f64;3])> = l.iter().copied().enumerate().filter(|(_,p)| p[1].abs() < 0.15 && p[0] > 3.8)
                .map(|(k,p)| (k,p.map(|x| (x*1e4).round()/1e4))).collect();
            if !near.is_empty() { eprintln!("solventc:   near the seam: {near:?}"); }
        }
        if std::env::var_os("SOLVENT_FEATURE_KINKS").is_some() {
            for w in l.windows(3) {
                let (u,v) = ([0,1,2].map(|k| w[1][k]-w[0][k]),[0,1,2].map(|k| w[2][k]-w[1][k]));
                let (uu,vv) = (u.iter().map(|x| x*x).sum::<f64>().sqrt(),v.iter().map(|x| x*x).sum::<f64>().sqrt());
                let angle = ((u[0]*v[0]+u[1]*v[1]+u[2]*v[2])/(uu*vv)).clamp(-1.,1.).acos().to_degrees();
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
        let text: String = lines.iter().map(|l| l.iter().map(|&p| { let q = model(p); format!("{} {} {}\n",q[0],q[1],q[2]) })
            .collect::<String>()+"\n").collect();
        let _ = std::fs::write(path,text);
    }
    Ok(lines.into_iter().map(|l| l.into_iter().map(model).collect()).collect())
}

/// Segments sharing endpoints (to `tolerance`) chained into polylines, a closed one repeating
/// its first point at its end.
fn chain(segments: Vec<([f64;3],[f64;3])>,tolerance: f64) -> Vec<Vec<[f64;3]>> {
    chain_lines(segments.into_iter().map(|(a,b)| vec![a,b]).collect(),tolerance)
}

fn chain_lines(mut lines: Vec<Vec<[f64;3]>>,tolerance: f64) -> Vec<Vec<[f64;3]>> {
    let same = |a: [f64;3],b: [f64;3]| (0..3).map(|k| (a[k]-b[k]).powi(2)).sum::<f64>().sqrt() <= tolerance;
    let mut out: Vec<Vec<[f64;3]>> = Vec::new();
    while let Some(mut line) = lines.pop() {
        loop {
            let (head,tail) = (line[0],*line.last().unwrap());
            if line.len() > 2 && same(head,tail) { break; }
            let Some(k) = lines.iter().position(|l| same(l[0],tail) || same(*l.last().unwrap(),tail)
                || same(l[0],head) || same(*l.last().unwrap(),head)) else { break };
            let mut other = lines.swap_remove(k);
            if same(other[0],tail) { line.extend(other.into_iter().skip(1)); }
            else if same(*other.last().unwrap(),tail) { other.reverse(); line.extend(other.into_iter().skip(1)); }
            else if same(*other.last().unwrap(),head) { other.pop(); other.extend(line); line = other; }
            else { other.reverse(); other.pop(); other.extend(line); line = other; }
        }
        out.push(line);
    }
    out
}
