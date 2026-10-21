//! The 1D features of blank-minus-sweeps for a field mesher (docs/field-meshing.md), read off
//! a static blank's own topology: its sharp edges and its faces' parameterisations, which a
//! host's kernel supplies through `BlankTopology`, and the material field, which says what a
//! sweep took. A point `p` on a blank face (outward normal `n`) is material exactly when
//! `p − εn` is, so the field read that far inside says whether a sweep took it: along each
//! sharp blank edge (read along the inward bisector) the surviving stretches are features,
//! their ends found by bisection as corners; over each blank face, the zero contour of that
//! reading on the face's parameter grid (marching squares, crossings bisected) is where the
//! swept boundary meets the face. A contour ends where the grid meets a trim or a seam, a cell
//! short of the edge, and there it is joined to the corner it runs to, or to a contour it
//! continues across a seam, so features meet only at endpoints.
//!
//! The field alone can find the same curves (`crease`, as the app does); this reading is the
//! one a host with the blank's kernel topology can afford to trust.
use super::MaterialField;
use crate::space::{add,norm,distance as dist};
use std::{cell::Cell,collections::BTreeMap};
use crate::clock::Instant;

type P = [f64;3];

/// A point on a blank's sharp edge (millimetres): its position, the two faces' outward normals
/// and the signed dihedral there (negative convex, positive concave, zero smooth).
#[derive(Clone,Copy,Debug)]
pub struct EdgePoint { pub position: P,pub normals: [P;2],pub dihedral: f64 }

/// A point of a blank face at parameters in [0, 1]² (millimetres), its outward normal, and
/// whether it lies on the face's trim.
#[derive(Clone,Copy,Debug)]
pub struct FacePoint { pub position: P,pub normal: P,pub on_trim: bool }

/// A static blank as a kernel knows it: its edges that may be features (seams and collapsed
/// poles left out), sampled by fraction, and its faces by normalised parameters.
pub trait BlankTopology {
    fn edges(&self) -> Result<usize,String>;
    fn edge_point(&self,edge: usize,fraction: f64) -> Result<EdgePoint,String>;
    fn faces(&self) -> Result<usize,String>;
    /// Whether the face spans a whole period in u and in v.
    fn face_seams(&self,face: usize) -> Result<[bool;2],String>;
    /// None where (u, v) is off the trimmed face.
    fn face_point(&self,face: usize,u: f64,v: f64,tolerance: f64) -> Result<Option<FacePoint>,String>;
}

/// The features found, in model units, and what they cost.
pub struct BlankFeatures {
    pub lines: Vec<Vec<P>>,
    /// Time over the sharp edges, and over the face contours with their joins.
    pub edge_time: std::time::Duration,
    pub contour_time: std::time::Duration,
    /// Field readings taken, and the seconds they took.
    pub readings: (usize,f64),
    /// The same lines in millimetres, as they were found, for a host's report.
    pub lines_mm: Vec<Vec<P>>,
}

/// The features of the body whose field is `field` and whose static blank is `blank`, the blank
/// `diagonal` model units across at `scale` millimetres a unit; each face's contour grid has
/// `grid` cells along each parameter.
pub fn features(blank: &dyn BlankTopology,field: &MaterialField,scale: f64,diagonal: f64,grid: usize)
    -> Result<BlankFeatures,String> {
    let started = Instant::now();
    let size = diagonal*scale;
    let eps = 1e-4*size;
    let field_time = Cell::new((0usize,0f64));
    let within = |p: P,n: P| -> bool {
        let q: P = std::array::from_fn(|k| (p[k]-eps*n[k])/scale);
        let clock = Instant::now();
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
    for edge in 0..blank.edges()? {
        let steps = 200;
        let at = |f: f64| -> Result<(P,bool,f64),String> {
            let e = blank.edge_point(edge,f)?;
            let [a,b] = e.normals;
            let n = add(a,b);
            let l = norm(n).max(1e-12);
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
    let edge_time = started.elapsed();
    let edges_done = Instant::now();
    // Face contours.
    let mut contours: Vec<Vec<P>> = Vec::new();
    let n = grid;
    for face in 0..blank.faces()? {
        let tolerance = 1e-9;
        // A direction the face spans a whole period of is sampled at cell centres and wraps, so
        // a contour crosses its seam like any other cell edge; a bounded one runs to its trims.
        let seams = blank.face_seams(face)?;
        let axis = |periodic: bool| -> (Vec<f64>,Vec<(usize,usize)>) {
            if periodic { ((0..n).map(|i| (i as f64+0.5)/n as f64).collect(),(0..n).map(|i| (i,(i+1)%n)).collect()) }
            else { ((0..=n).map(|i| i as f64/n as f64).collect(),(0..n).map(|i| (i,i+1)).collect()) }
        };
        let ((us,ucells),(vs,vcells)) = (axis(seams[0]),axis(seams[1]));
        let grid: Vec<Vec<Option<(P,bool)>>> = us.iter().map(|&u| vs.iter().map(|&v| {
            // A point on the face's trim is on another face too, and read just inside this one
            // it is read on that other: the edge features cover the trim.
            blank.face_point(face,u,v,tolerance)
                .map(|fp| fp.filter(|fp| !fp.on_trim).map(|fp| (fp.position,within(fp.position,fp.normal))))
        }).collect::<Result<Vec<_>,_>>()).collect::<Result<_,_>>()?;
        // Parameter from one sample to the next, unwrapped across a seam.
        let step = |values: &[f64],a: usize,b: usize| {
            let d = values[b]-values[a];
            if d < -0.5 { d+1. } else if d > 0.5 { d-1. } else { d }
        };
        // The crossing on the grid edge from (i0, j0) to (i1, j1), by bisection on the face.
        let mut crossings: BTreeMap<(usize,usize,usize,usize),P> = Default::default();
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
                let Some(fp) = blank.face_point(face,u,v,tolerance)? else { return Ok(None) };
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
    let contour_time = edges_done.elapsed();
    let readings = field_time.get();
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
    // leaves a mesher's protecting balls nowhere to go.
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
    Ok(BlankFeatures {lines:lines.iter().map(|l| l.iter().copied().map(model).collect()).collect(),edge_time,contour_time,
        readings,lines_mm:lines})
}

/// Segments sharing endpoints (to `tolerance`) chained into polylines, a closed one repeating
/// its first point at its end.
pub fn chain(segments: Vec<(P,P)>,tolerance: f64) -> Vec<Vec<P>> {
    chain_lines(segments.into_iter().map(|(a,b)| vec![a,b]).collect(),tolerance)
}

/// Polylines sharing endpoints (to `tolerance`) chained into longer ones, each taken until
/// neither of its ends meets another's.
pub fn chain_lines(mut lines: Vec<Vec<P>>,tolerance: f64) -> Vec<Vec<P>> {
    let same = |a: P,b: P| dist(a,b) <= tolerance;
    let mut out: Vec<Vec<P>> = Vec::new();
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
