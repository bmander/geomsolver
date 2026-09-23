//! A body's boundary meshed from its material field by Delaunay refinement with protected
//! features (CGAL's Mesh_3; `backend/delpsc.cpp`). An experiment behind the `cgal` feature,
//! `--stl-backend cgal`: the field is asked only for signs, and no sheet, fit or kernel
//! Boolean enters. Delaunay refinement needs the domain's sharp curves given as features: here
//! the static blank's sharp edges where material survives, and the curves where the swept
//! boundary meets each blank face, both found from the material field alone (`features`).
use gcs_core::solid::MaterialField;
use std::ffi::{CString,c_char,c_int,c_void};

unsafe extern "C" {
    fn solvent_cgal_mesh(field: extern "C" fn(*mut c_void,*const f64) -> f64,context: *mut c_void,center: *const f64,
        radius: f64,points: *const f64,counts: *const c_int,curves: c_int,sizes: *const f64,scale: f64,
        path: *const c_char,error: *mut c_char,capacity: c_int) -> c_int;
}

/// What the callback reads the field with, and what it has seen.
struct Oracle { field: MaterialField,queries: usize,started: std::time::Instant,shown: f64,spent: std::time::Duration }

/// The side of the boundary a point lies on, as Mesh_3 asks it: the field's plain floating-point
/// side (`MaterialField::side`), since a mesher needs a side and not a proof. The certified
/// probe judges the finished mesh.
extern "C" fn sign(context: *mut c_void,p: *const f64) -> f64 {
    let oracle = unsafe { &mut *(context as *mut Oracle) };
    let p = unsafe { [*p,*p.add(1),*p.add(2)] };
    oracle.queries += 1;
    let seen = oracle.started.elapsed().as_secs_f64();
    if seen > oracle.shown+3. {
        oracle.shown = seen;
        eprintln!("solventc: [{seen:7.1} s] Mesh_3: {} field queries, {:.1} s of them in the field",
            oracle.queries,oracle.spent.as_secs_f64());
    }
    let clock = std::time::Instant::now();
    let value = oracle.field.side(p);
    oracle.spent += clock.elapsed();
    if value.is_finite() { value } else { 1. }
}

fn setting(name: &str,default: f64) -> f64 {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// Mesh `solid` from its field and write the STL at `path` (millimetres). Sizes default to a
/// fraction of the material's support and can be set for the experiment:
/// `SOLVENT_CGAL_FACET` (facet size, mm), `SOLVENT_CGAL_DISTANCE` (facet distance, mm),
/// `SOLVENT_CGAL_ANGLE` (degrees), `SOLVENT_CGAL_EDGE` (feature edge size, mm),
/// `SOLVENT_CGAL_BISECTION` (where a crossing is placed, relative to the bounding sphere).
pub fn export(sk: &gcs_core::model::Sketch,solid: usize,path: &str) -> Result<(),String> {
    let scale = sk.units.length.ok_or("CAD export requires an explicit model length unit")?.1;
    let field = MaterialField::read(sk,solid,1e-10)?;
    let support = field.support_bounds().map_err(|e| format!("{e:?}"))?
        .ok_or("the material has no finite support to mesh in")?;
    let [lo,hi] = [0,1].map(|k| support.map(|x| x.bounds()[k]));
    let center: [f64;3] = std::array::from_fn(|k| 0.5*(lo[k]+hi[k]));
    let diagonal = (0..3).map(|k| (hi[k]-lo[k]).powi(2)).sum::<f64>().sqrt();
    // The bounding sphere must hold the material with room; Mesh_3 treats outside it as void.
    let radius = 0.5*diagonal*1.05+1e-6;
    let facet = setting("SOLVENT_CGAL_FACET",diagonal*scale/60.)/scale;
    let sizes = [facet,setting("SOLVENT_CGAL_DISTANCE",0.005)/scale,setting("SOLVENT_CGAL_ANGLE",25.),
        setting("SOLVENT_CGAL_EDGE",facet*scale)/scale,setting("SOLVENT_CGAL_BISECTION",1e-5)];
    let started = std::time::Instant::now();
    let mut oracle = Oracle {field:field.clone(),queries:0,started,shown:0.,spent:Default::default()};
    let name = CString::new(path).map_err(|e| e.to_string())?;
    let mut error = vec![0 as c_char;1024];
    #[cfg(feature="occt")]
    let lines = features(sk,solid,&field,scale,diagonal)?;
    #[cfg(not(feature="occt"))]
    let lines: Vec<Vec<[f64;3]>> = Vec::new();
    eprintln!("solventc: [{:7.1} s] Mesh_3: {} feature curves of {} points",started.elapsed().as_secs_f64(),lines.len(),
        lines.iter().map(Vec::len).sum::<usize>());
    let points: Vec<f64> = lines.iter().flatten().flatten().copied().collect();
    let counts: Vec<c_int> = lines.iter().map(|l| l.len() as c_int).collect();
    let count = unsafe {
        solvent_cgal_mesh(sign,&mut oracle as *mut Oracle as *mut c_void,center.as_ptr(),radius,points.as_ptr(),
            counts.as_ptr(),counts.len() as c_int,sizes.as_ptr(),scale,name.as_ptr(),error.as_mut_ptr(),error.len() as c_int)
    };
    if count < 0 {
        let message = unsafe { std::ffi::CStr::from_ptr(error.as_ptr()) }.to_string_lossy().into_owned();
        return Err(format!("Mesh_3 failed: {message}"));
    }
    eprintln!("solventc: Mesh_3: {count} triangles from {} field queries ({:.1} s in the field) in {:?}, \
        facet size {:.4} mm, distance {:.4} mm",oracle.queries,oracle.spent.as_secs_f64(),started.elapsed(),sizes[0]*scale,sizes[1]*scale);
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    gcs_core::mesh::stl_shells(&bytes).map_err(|e| format!("the Mesh_3 STL fails its shell check: {e}"))?;
    super::mark("stl");
    #[cfg(feature="occt")]
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
#[cfg(feature="occt")]
fn features(sk: &gcs_core::model::Sketch,solid: usize,field: &MaterialField,scale: f64,diagonal: f64)
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
    for face in session.faces(blank)? {
        let n = 64;
        let tolerance = 1e-9;
        let grid: Vec<Vec<Option<(P,bool)>>> = (0..=n).map(|i| (0..=n).map(|j| {
            // A point on the face's trim is on another face too, and read just inside this one
            // it is read on that other: the edge features cover the trim.
            session.face_point(face,i as f64/n as f64,j as f64/n as f64,tolerance)
                .map(|fp| fp.filter(|fp| !fp.on_trim).map(|fp| (fp.position,within(fp.position,fp.normal))))
        }).collect::<Result<Vec<_>,_>>()).collect::<Result<_,_>>()?;
        // The crossing on the grid edge from (i0, j0) to (i1, j1), by bisection on the face.
        let mut crossings: std::collections::BTreeMap<(usize,usize,usize,usize),P> = Default::default();
        let mut crossing = |a: (usize,usize),b: (usize,usize)| -> Result<Option<P>,String> {
            let key = (a.0.min(b.0),a.1.min(b.1),a.0.max(b.0),a.1.max(b.1));
            if let Some(p) = crossings.get(&key) { return Ok(Some(*p)); }
            let (Some(sa),Some(_)) = (grid[a.0][a.1],grid[b.0][b.1]) else { return Ok(None) };
            let uv = |t: f64| ((a.0 as f64+t*(b.0 as f64-a.0 as f64))/n as f64,(a.1 as f64+t*(b.1 as f64-a.1 as f64))/n as f64);
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
        for i in 0..n { for j in 0..n {
            let c = [(i,j),(i+1,j),(i+1,j+1),(i,j+1)];
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
    loop {
        let mut changed = false;
        'search: for a in 0..joined.len() {
            if !open(&joined[a]) { continue; }
            let (head,tail) = (joined[a][0],*joined[a].last().unwrap());
            if cornered(head) || cornered(tail) { continue; }
            if dist(head,tail) < reach { joined[a].push(head); changed = true; break 'search; }
            for b in 0..joined.len() {
                if b == a || !open(&joined[b]) { continue; }
                let (h,t) = (joined[b][0],*joined[b].last().unwrap());
                if dist(tail,h) < reach && !cornered(h) {
                    let other = joined.remove(b);
                    let a = if b < a { a-1 } else { a };
                    joined[a].extend(other);
                    changed = true; break 'search;
                }
                if dist(tail,t) < reach && !cornered(t) {
                    let mut other = joined.remove(b);
                    other.reverse();
                    let a = if b < a { a-1 } else { a };
                    joined[a].extend(other);
                    changed = true; break 'search;
                }
            }
        }
        if !changed { break; }
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
        let mut kept: Vec<P> = Vec::new();
        for p in l { if kept.last().map_or(true,|q| dist(*q,p) > 1e-6*size) { kept.push(p); } }
        if closed && kept.len() > 2 { let first = kept[0]; *kept.last_mut().unwrap() = first; }
        kept
    }).filter(|l| l.len() > 1).collect();
    for l in &lines {
        let length: f64 = l.windows(2).map(|w| dist(w[0],w[1])).sum();
        eprintln!("solventc:   feature of {} points, {:.4} mm, from {:?} to {:?}",l.len(),length,
            l[0].map(|x| (x*1e4).round()/1e4),l.last().unwrap().map(|x| (x*1e4).round()/1e4));
    }
    Ok(lines.into_iter().map(|l| l.into_iter().map(model).collect()).collect())
}

/// Segments sharing endpoints (to `tolerance`) chained into polylines, a closed one repeating
/// its first point at its end.
#[cfg(feature="occt")]
fn chain(segments: Vec<([f64;3],[f64;3])>,tolerance: f64) -> Vec<Vec<[f64;3]>> {
    chain_lines(segments.into_iter().map(|(a,b)| vec![a,b]).collect(),tolerance)
}

#[cfg(feature="occt")]
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
