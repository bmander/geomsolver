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

/// The direction each vertex of each sheet is judged along: the normalised
/// sum of the normals of every triangle, on any sheet, incident on the
/// vertex's position, each weighted by the triangle's angle at the vertex
/// (so a fan of slivers on one face does not outvote the face beside it).
/// On a smooth sheet that is the sheet's normal; where sheets meet at a tool
/// edge or corner it is the bisector, which enters the material where one
/// face's own normal runs along the other face. Positions are identified
/// within `weld`.
pub fn directions(sheets: &[SweepPatch],weld: f64) -> Vec<Vec<V3>> {
    let cell = weld.max(f64::MIN_POSITIVE)*4.;
    let key = |p: V3| p.map(|x| (x/cell).floor() as i64);
    // every (sheet, vertex) by position cell
    let mut cells: std::collections::BTreeMap<[i64;3],Vec<(usize,usize)>> = Default::default();
    for (s,sheet) in sheets.iter().enumerate() { for (v,p) in sheet.points.iter().enumerate() { cells.entry(key(*p)).or_default().push((s,v)); } }
    // the summed triangle normals per (sheet, vertex)
    let mut sums: Vec<Vec<V3>> = sheets.iter().map(|s| vec![[0.;3];s.points.len()]).collect();
    for (s,sheet) in sheets.iter().enumerate() {
        for t in &sheet.triangles {
            let [a,b,c] = t.map(|v| sheet.points[v as usize]);
            let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
            let mut n = [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];
            let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
            if !(len > 0.) { continue; }
            n = n.map(|x| x/len);
            // the sheet's own normals say which side is out; the winding does not
            let stored: V3 = std::array::from_fn(|k| t.iter().map(|&v| sheet.normals[v as usize][k]).sum());
            if n[0]*stored[0]+n[1]*stored[1]+n[2]*stored[2] < 0. { n = n.map(|x| -x); }
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
            let k = key(*p);
            let mut total = [0.;3];
            for x in k[0]-1..=k[0]+1 { for y in k[1]-1..=k[1]+1 { for z in k[2]-1..=k[2]+1 {
                if let Some(list) = cells.get(&[x,y,z]) { for &(s2,v2) in list {
                    let q = sheets[s2].points[v2];
                    if (0..3).map(|k| (p[k]-q[k]).powi(2)).sum::<f64>().sqrt() <= weld { for k in 0..3 { total[k] += sums[s2][v2][k]; } }
                } }
            } } }
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
