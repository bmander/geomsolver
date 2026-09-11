//! Every vertex of a seed sheet judged by the field along the sheet's own
//! normal there: kept, moved onto the boundary, or labelled as an inner
//! branch or as off the swept material.
use super::judge::{FieldJudge,JudgeError,Projection};
use crate::solid::SweepPatch;

type V3 = [f64;3];

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Label { Kept, Moved, Inner, Positive, Unresolved }

/// A sheet with its vertices judged: `points` are the vertices as kept or
/// moved (an inner or positive vertex keeps its seed position), `radius` how
/// far the boundary may be from a kept or moved vertex.
#[derive(Clone,Debug)]
pub struct Labelled {
    pub points: Vec<V3>,
    pub labels: Vec<Label>,
    pub radius: Vec<f64>,
    /// How far each moved vertex travelled along its normal, outward positive.
    pub moved_by: Vec<f64>,
    /// Every unresolved vertex: its index, the offset along its normal where
    /// the field gave no sign, and the enclosure there.
    pub unresolved: Vec<(usize,f64,[f64;2])>,
    /// The direction each vertex was judged along.
    pub directions: Vec<V3>,
}

impl Labelled {
    pub fn count(&self,label: Label) -> usize { self.labels.iter().filter(|l| **l == label).count() }
    /// The edges of the sheet whose two ends are labelled differently, as
    /// `(kept_or_moved, other)` vertex pairs: where the sheet leaves the boundary.
    pub fn transitions(&self,patch: &SweepPatch) -> Vec<(u32,u32)> {
        let on = |v: u32| matches!(self.labels[v as usize],Label::Kept | Label::Moved);
        let mut edges: std::collections::BTreeSet<(u32,u32)> = Default::default();
        for t in &patch.triangles { for k in 0..3 {
            let (a,b) = (t[k],t[(k+1)%3]);
            if on(a) && !on(b) { edges.insert((a,b)); } else if on(b) && !on(a) { edges.insert((b,a)); }
        } }
        edges.into_iter().collect()
    }
}

/// Which way each triangle of a sheet faces: `true` where its winding's
/// normal is the outward one. The winding is carried across shared edges
/// (a zipped strip is one orientable surface however its quads were cut)
/// but not across a fold, where the two triangles' normals oppose (a
/// generator through a fixed point of the motion sweeps a bowtie whose
/// halves face opposite ways), and each connected run takes the side its
/// stored normals favour in sum. A triangle's own three stored normals
/// cannot decide it: at a fan point the tracer stores one end of the fan,
/// and a sliver on two such points reads the wrong way by that end alone.
pub fn orientation(sheet: &SweepPatch) -> Vec<bool> {
    let n = sheet.triangles.len();
    let mut owner: std::collections::BTreeMap<(u32,u32),Vec<(usize,bool)>> = Default::default();
    for (i,t) in sheet.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); owner.entry((a.min(b),a.max(b))).or_default().push((i,a < b)); } }
    let normal = |i: usize| -> Option<V3> {
        let [a,b,c] = sheet.triangles[i].map(|v| sheet.points[v as usize]);
        crate::space::stable_normal(a,b,c)
    };
    let mut flip: Vec<Option<bool>> = vec![None;n];
    for start in 0..n {
        if flip[start].is_some() { continue; }
        // one run: flipped relative to the start, carried across edges
        let mut run = vec![start];
        flip[start] = Some(false);
        let mut k = 0;
        while k < run.len() {
            let i = run[k]; k += 1;
            let t = sheet.triangles[i];
            for e in 0..3 {
                let (a,b) = (t[e],t[(e+1)%3]);
                let Some(list) = owner.get(&(a.min(b),a.max(b))) else { continue };
                if list.len() != 2 { continue; }
                for &(j,forward) in list {
                    if j == i || flip[j].is_some() { continue; }
                    // neighbours agree when they walk the shared edge opposite ways
                    let same = forward != (a < b);
                    let (Some(ni),Some(nj)) = (normal(i),normal(j)) else { continue };
                    let d = ni[0]*nj[0]+ni[1]*nj[1]+ni[2]*nj[2];
                    // a fold: carried across, the neighbour would face away
                    if (if same { d } else { -d }) <= 0. { continue; }
                    flip[j] = Some(flip[i].unwrap() != !same);
                    run.push(j);
                }
            }
        }
        // the run faces the way its stored normals say, weighted by area
        let mut vote = 0.;
        for &i in &run {
            let Some(m) = normal(i) else { continue };
            let [a,b,c] = sheet.triangles[i].map(|v| sheet.points[v as usize]);
            let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
            let cr = [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];
            let area = (cr[0]*cr[0]+cr[1]*cr[1]+cr[2]*cr[2]).sqrt();
            let stored: V3 = std::array::from_fn(|k| sheet.triangles[i].iter().map(|&v| sheet.normals[v as usize][k]).sum());
            let sign = if flip[i].unwrap() { -1. } else { 1. };
            vote += sign*area*(m[0]*stored[0]+m[1]*stored[1]+m[2]*stored[2]);
        }
        let outward_is_unflipped = vote >= 0.;
        for &i in &run { let f = flip[i].unwrap(); flip[i] = Some(if outward_is_unflipped { f } else { !f }); }
    }
    flip.into_iter().map(|f| !f.unwrap_or(false)).collect()
}

/// The direction each vertex of each sheet is judged along: the normalised
/// sum of the normals of every triangle, on any sheet, incident on the
/// vertex's position, each weighted by the triangle's angle at the vertex
/// (so a fan of slivers on one face does not outvote the face beside it).
/// On a smooth sheet that is the sheet's normal; where sheets meet at a tool
/// edge or corner it is the bisector, which enters the material where one
/// face's own normal runs along the other face. Positions are identified
/// within `weld`.
pub fn directions(sheets: &[SweepPatch],weld: f64) -> Vec<Vec<V3>> {
    directions_of(&sheets.iter().collect::<Vec<_>>(),weld)
}

fn directions_of(sheets: &[&SweepPatch],weld: f64) -> Vec<Vec<V3>> {
    // every (sheet, vertex) by position, filed by its place in `owners`
    let mut grid = crate::space::Grid::new(weld.max(f64::MIN_POSITIVE)*4.);
    let mut owners: Vec<(usize,usize)> = Vec::new();
    for (s,sheet) in sheets.iter().enumerate() { for (v,p) in sheet.points.iter().enumerate() { grid.insert(*p,owners.len() as u32); owners.push((s,v)); } }
    let grid = grid.pack();
    // the summed triangle normals per (sheet, vertex)
    let mut sums: Vec<Vec<V3>> = sheets.iter().map(|s| vec![[0.;3];s.points.len()]).collect();
    for (s,sheet) in sheets.iter().enumerate() {
        let faces = orientation(sheet);
        for (i,t) in sheet.triangles.iter().enumerate() {
            let [a,b,c] = t.map(|v| sheet.points[v as usize]);
            let Some(mut n) = crate::space::stable_normal(a,b,c) else { continue };
            // the sheet says which side is out; the winding alone does not
            if !faces[i] { n = n.map(|x| -x); }
            for k in 0..3 {
                let (p,q,r) = (sheet.points[t[k] as usize],sheet.points[t[(k+1)%3] as usize],sheet.points[t[(k+2)%3] as usize]);
                let (e1,e2) = ([q[0]-p[0],q[1]-p[1],q[2]-p[2]],[r[0]-p[0],r[1]-p[1],r[2]-p[2]]);
                let (l1,l2) = ((e1[0]*e1[0]+e1[1]*e1[1]+e1[2]*e1[2]).sqrt(),(e2[0]*e2[0]+e2[1]*e2[1]+e2[2]*e2[2]).sqrt());
                let angle = if l1 > 0. && l2 > 0. { ((e1[0]*e2[0]+e1[1]*e2[1]+e1[2]*e2[2])/(l1*l2)).clamp(-1.,1.).acos() } else { 0. };
                for j in 0..3 { sums[s][t[k] as usize][j] += angle*n[j]; }
            }
        }
    }
    // each vertex takes the sum over every vertex within the weld of it
    let mut out: Vec<Vec<V3>> = sheets.iter().map(|s| s.normals.clone()).collect();
    for (s,sheet) in sheets.iter().enumerate() {
        for (v,p) in sheet.points.iter().enumerate() {
            let mut total = [0.;3];
            grid.around(*p,|i| {
                let (s2,v2) = owners[i as usize];
                let q = sheets[s2].points[v2];
                if (0..3).map(|k| (p[k]-q[k]).powi(2)).sum::<f64>().sqrt() <= weld { for k in 0..3 { total[k] += sums[s2][v2][k]; } }
            });
            let len = (total[0]*total[0]+total[1]*total[1]+total[2]*total[2]).sqrt();
            if len > 0. { out[s][v] = total.map(|x| x/len); }
        }
    }
    out
}

/// Judge every vertex of every sheet along the directions of `directions`.
/// `epsilon` is the vertex tolerance, `reach` how far along the direction the
/// boundary is looked for before a vertex is labelled inner or positive.
pub fn label_sheets(judge: &mut FieldJudge,sheets: &[SweepPatch],epsilon: f64,reach: f64) -> Result<Vec<Labelled>,JudgeError> {
    let directions = directions(sheets,1e-9*epsilon.max(1.));
    sheets.iter().zip(&directions).map(|(sheet,m)| label_patch(judge,sheet,m,epsilon,reach)).collect()
}

/// Every vertex of every seed of a construction judged: a cap's and a region's along their own
/// normals
/// (its summed normal across the seeds can run along one face at a tool edge while the vertex
/// lies on another), a traced sheet's along `directions`.
pub fn label_seeds(judge: &mut FieldJudge,seeds: &[super::Seed],epsilon: f64,reach: f64) -> Result<Vec<Labelled>,JudgeError> {
    let patches: Vec<&SweepPatch> = seeds.iter().map(super::Seed::patch).collect();
    let directions = directions_of(&patches,1e-9*epsilon.max(1.));
    seeds.iter().zip(&directions).map(|(seed,m)| {
        let sheet = seed.patch();
        label_patch(judge,sheet,if matches!(seed,super::Seed::Cap(_) | super::Seed::Grazing(_)) { &sheet.normals } else { m },epsilon,reach)
    }).collect()
}

/// Judge every vertex of one sheet along the given directions.
pub fn label_patch(judge: &mut FieldJudge,patch: &SweepPatch,directions: &[V3],epsilon: f64,reach: f64) -> Result<Labelled,JudgeError> {
    let n = patch.points.len();
    let (mut points,mut labels,mut radius,mut moved_by) = (Vec::with_capacity(n),Vec::with_capacity(n),Vec::with_capacity(n),Vec::with_capacity(n));
    let mut unresolved = Vec::new();
    for (i,(p,m)) in patch.points.iter().zip(directions).enumerate() {
        let len = (m[0]*m[0]+m[1]*m[1]+m[2]*m[2]).sqrt();
        if !(len > 0.) { return Err(JudgeError::ReversedNormal {point:*p,direction:*m}); }
        let m = m.map(|x| x/len);
        match judge.project(*p,m,epsilon,reach)? {
            Projection::Kept {radius:r} => { points.push(*p); labels.push(Label::Kept); radius.push(r); moved_by.push(0.); }
            Projection::Moved {point,radius:r,by} => { points.push(point); labels.push(Label::Moved); radius.push(r); moved_by.push(by); }
            Projection::Inner => { points.push(*p); labels.push(Label::Inner); radius.push(f64::INFINITY); moved_by.push(0.); }
            Projection::Positive => { points.push(*p); labels.push(Label::Positive); radius.push(f64::INFINITY); moved_by.push(0.); }
            Projection::Unresolved {radius:r,enclosure} => {
                points.push(*p); labels.push(Label::Unresolved); radius.push(f64::INFINITY); moved_by.push(0.);
                unresolved.push((i,r,enclosure));
            }
        }
    }
    Ok(Labelled {points,labels,radius,moved_by,unresolved,directions:directions.to_vec()})
}
