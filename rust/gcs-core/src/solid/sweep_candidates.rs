//! Candidate boundary curves of a swept tool whose motion is a one-parameter
//! subgroup (a rotation, translation or screw): the tool-frame twist is then
//! constant, so the contact condition `n·v = 0` does not depend on the
//! parameter, the contact set is one fixed family of curves on the tool's
//! boundary, and the envelope is that set carried along the motion.
//!
//! The curves are traced on each face by the exact fixed-time roots of its
//! station equation (`tool_faces`), trimmed where the tool's own static field
//! hides a portion inside a Boolean operand, joined to the sharp-edge fans
//! contributed by the edges between faces, and chained by endpoint identity.
//! This is candidate generation only: caps, visibility and material remain the
//! arrangement's and the classifier's.
use super::{SweepContacts,ToolFace,ToolEdge,tool_faces::Crease};
use crate::{envelope::{self,Motion},plane};

type V3 = [f64;3];
fn sub(a: V3,b: V3) -> V3 { std::array::from_fn(|k| a[k]-b[k]) }
fn norm(a: V3) -> f64 { plane::dot(a,a).sqrt() }
fn distance(a: V3,b: V3) -> f64 { norm(sub(a,b)) }

/// A polyline of contact positions with outward tool normals, in the tool's
/// own frame. `closed` means the last point joins the first. `joints` are
/// the indices at which pieces from different faces or edges were chained
/// together: the curve may turn a corner there and nowhere else.
#[derive(Clone,Debug)]
pub struct Characteristic {
    pub points: Vec<V3>,
    pub normals: Vec<V3>,
    pub closed: bool,
    pub joints: Vec<usize>,
}

impl Characteristic {
    fn open(points: Vec<V3>,normals: Vec<V3>) -> Self { Self {points,normals,closed:false,joints:Vec::new()} }

    /// The same curve run the other way.
    fn reversed(&self) -> Self {
        let n = self.points.len();
        let mut joints: Vec<usize> = self.joints.iter().map(|&j| n-1-j).collect();
        joints.sort();
        Self {points:self.points.iter().rev().copied().collect(),normals:self.normals.iter().rev().copied().collect(),closed:self.closed,joints}
    }

    /// A closed curve started `k` points on.
    fn rotated(&self,k: usize) -> Self {
        let n = self.points.len();
        let mut joints: Vec<usize> = self.joints.iter().map(|&j| (j+n-k)%n).collect();
        joints.sort();
        let mut c = self.clone();
        c.points.rotate_left(k); c.normals.rotate_left(k); c.joints = joints;
        c
    }

    /// The pieces between joints as index ranges `[start, end]`, a closed
    /// curve's last piece wrapping to its first point.
    fn pieces(&self) -> Vec<(usize,usize)> {
        let n = self.points.len();
        let mut cuts: Vec<usize> = vec![0];
        cuts.extend(self.joints.iter().copied().filter(|&j| j > 0 && j < n));
        cuts.dedup();
        let mut out = Vec::new();
        for w in cuts.windows(2) { out.push((w[0],w[1])); }
        out.push((*cuts.last().unwrap(),if self.closed { n } else { n-1 }));
        out.retain(|(a,b)| b > a);
        out
    }
}

/// An edge of the tool: charted on its two faces, or a Boolean crease traced
/// across them.
#[derive(Clone,Copy)]
enum EdgeRef<'a> { Charted(&'a ToolEdge),Crease(&'a Crease) }

impl EdgeRef<'_> {
    fn faces(self) -> [usize;2] { match self { Self::Charted(e) => e.faces,Self::Crease(c) => c.faces } }
}


/// Each face's outward sign and the faces it is shadowed by.
struct Signs { signs: Vec<f64>, shadows: Vec<Vec<usize>> }

/// A station root in the tool's frame: `(v, position, outward normal)`.
type Root = (f64,V3,V3);

/// A run of station roots at consecutive stations from `first`, with the
/// exact points where it enters and leaves the contact set between stations.
#[derive(Clone,Debug)]
struct Strand { roots: Vec<Root>, head: Option<Root>, tail: Option<Root> }

impl Strand {
    fn into_characteristic(self) -> Characteristic {
        let mut points = Vec::with_capacity(self.roots.len()+2);
        let mut normals = Vec::with_capacity(self.roots.len()+2);
        for (_,p,n) in self.head.iter().chain(&self.roots).chain(&self.tail) { points.push(*p); normals.push(*n); }
        Characteristic::open(points,normals)
    }
}

/// A candidate boundary sheet of a sweep, in the body's frame: a row-major
/// grid of contact positions with the tool's outward normal at each, and the
/// motion parameter each column was taken at. `closed_rows` joins the last row
/// back to the first, a tube.
#[derive(Clone,Debug)]
pub struct SweepSheet {
    pub points: Vec<V3>,
    pub normals: Vec<V3>,
    pub times: Vec<f64>,
    pub rows: usize,
    pub columns: usize,
    pub closed_rows: bool,
}

/// A candidate boundary sheet of a sweep whose twist varies, in the body's
/// frame: the contact curve at each of a sequence of motion parameters, every
/// one with its own traced points, and the triangles zipped between each
/// curve and the next by nearest point. Vertices come column by column, in
/// order along each curve; `column[v]` is the column a vertex belongs to and
/// `times` the parameter of each column.
#[derive(Clone,Debug)]
pub struct SweepPatch {
    pub points: Vec<V3>,
    pub normals: Vec<V3>,
    pub triangles: Vec<[u32;3]>,
    pub column: Vec<u32>,
    pub times: Vec<f64>,
    pub closed: bool,
}

impl SweepPatch {
    /// The vertices of one column, in order along its curve.
    pub fn column_vertices(&self,c: u32) -> Vec<u32> {
        (0..self.points.len() as u32).filter(|&v| self.column[v as usize] == c).collect()
    }
}

/// Whether the motion's tool-frame twist is constant over the domain, sampled
/// at several parameters: the regime in which the contact set is fixed.
pub fn constant_twist(motion: &crate::motion::Family,domain: [f64;2],tolerance: f64) -> Result<bool,String> {
    let twist = |t: f64| -> Result<[V3;4],String> {
        let pose = motion.at(t)?;
        let inverse = pose.inverse();
        let velocity = |p: V3| inverse.vector(pose.velocity(p));
        let v0 = velocity([0.;3]);
        Ok([v0,sub(velocity([1.,0.,0.]),v0),sub(velocity([0.,1.,0.]),v0),sub(velocity([0.,0.,1.]),v0)])
    };
    let reference = twist(domain[0])?;
    for i in 1..=4 {
        let t = domain[0]+(domain[1]-domain[0])*i as f64/4.;
        let sample = twist(t)?;
        for k in 0..4 { if distance(sample[k],reference[k]) > tolerance*(1.+norm(reference[k])) { return Ok(false); } }
    }
    Ok(true)
}

const STATIONS: usize = 96;
const JOIN: f64 = 1e-7;

impl SweepContacts {
    /// Outward normal sign of a face's `du x dv`, decided by the tool's field
    /// just off the surface at a regular point.
    fn outward_sign(&self,face: &ToolFace,scale: f64) -> Result<f64,String> {
        // The face's interior first, then a grid of it: a face of a Boolean
        // operand may be off the tool's boundary where its own interior is.
        let [[u0,u1],[v0,v1]] = face.domain();
        let interior = face.interior().map_err(|e| format!("face interior: {e:?}"))?;
        let grid = (1..24).flat_map(|i| (1..24).map(move |j| (u0+(u1-u0)*i as f64/24.,v0+(v1-v0)*j as f64/24.)));
        for (u,v) in std::iter::once(interior).chain(grid) {
            if !face.contains(u,v) { continue; }
            let Ok(s) = face.at(u,v) else { continue };
            let Ok(c) = envelope::contact(s,Motion::identity()) else { continue };
            let side = |sign: f64| -> Result<Option<bool>,String> {
                let probe: V3 = std::array::from_fn(|k| s.position[k]+sign*c.normal[k]*scale*1e-6);
                let value = self.source.value(probe);
                Ok(if value > scale*1e-9 { Some(false) } else if value < -scale*1e-9 { Some(true) } else { None })
            };
            match (side(1.)?,side(-1.)?) {
                (Some(false),Some(true)) => return Ok(1.),
                (Some(true),Some(false)) => return Ok(-1.),
                _ => continue,
            }
        }
        // A face of an operand nowhere on the tool's boundary contributes no
        // contact; its side is moot.
        Ok(1.)
    }

    /// Whether a point of a face's carrier is off the tool's boundary: inside
    /// the tool, under another operand, or outside it, beyond the operand it
    /// is bounded by.
    fn hidden(&self,p: V3,scale: f64) -> Result<bool,String> {
        Ok(self.source.value(p).abs() > scale*1e-9)
    }

    /// The sheets of a constant-twist sweep: each characteristic carried along
    /// the motion over the declared interval, plus `overrun` (a length) of travel
    /// beyond both ends into the tool's end poses, sampled about `spacing` apart
    /// along the travel and closely enough that no point's path deviates from
    /// the chord between columns by more than `sagitta`. Rows are the
    /// characteristic's points, columns the times.
    pub fn carried_sheets(&self,spacing: f64,sagitta: f64,overrun: f64,tolerance: f64) -> Result<Vec<SweepSheet>,String> {
        let [from,to] = self.domain();
        let curves = self.characteristics(tolerance)?;
        if curves.is_empty() { return Err("the tool has no contact curve under its motion".into()); }
        let motion = self.motion();
        let mut sheets = Vec::with_capacity(curves.len());
        for curve in curves {
            let start = motion.at(from)?;
            let speed = curve.points.iter().map(|p| norm(start.velocity(*p))).fold(0_f64,f64::max).max(1e-9);
            let (t0,t1) = (from-overrun/speed,to+overrun/speed);
            let mut columns = (((t1-t0)*speed/spacing).ceil() as usize).clamp(2,400);
            columns = self.columns_within_sagitta(&curve.points,[t0,t1],columns,sagitta)?;
            let times: Vec<f64> = (0..columns).map(|c| t0+(t1-t0)*c as f64/(columns-1) as f64).collect();
            let poses = times.iter().map(|&t| motion.at(t)).collect::<Result<Vec<_>,_>>()?;
            let mut points = Vec::with_capacity(curve.points.len()*columns);
            let mut normals = Vec::with_capacity(curve.points.len()*columns);
            for (p,n) in curve.points.iter().zip(&curve.normals) {
                for pose in &poses { points.push(pose.point(*p)); normals.push(pose.vector(*n)); }
            }
            sheets.push(SweepSheet {points,normals,times,rows:curve.points.len(),columns,closed_rows:curve.closed});
        }
        Ok(sheets)
    }

    /// The smallest column count, doubling from `columns`, at which no sampled
    /// point's path between consecutive columns leaves the chord between them
    /// by more than `sagitta`.
    fn columns_within_sagitta(&self,points: &[V3],span: [f64;2],columns: usize,sagitta: f64) -> Result<usize,String> {
        let motion = self.motion();
        let sample: Vec<V3> = points.iter().step_by((points.len()/24).max(1)).copied().collect();
        let mut columns = columns.max(2);
        loop {
            let step = (span[1]-span[0])/(columns-1) as f64;
            let mut worst: f64 = 0.;
            for c in 0..columns-1 {
                let (a,m,b) = (span[0]+step*c as f64,span[0]+step*(c as f64+0.5),span[0]+step*(c+1) as f64);
                let (pa,pm,pb) = (motion.at(a)?,motion.at(m)?,motion.at(b)?);
                for p in &sample {
                    let (qa,qm,qb) = (pa.point(*p),pm.point(*p),pb.point(*p));
                    let chord: V3 = std::array::from_fn(|k| 0.5*(qa[k]+qb[k]));
                    worst = worst.max(distance(qm,chord));
                }
            }
            if worst <= sagitta || columns >= 4000 { return Ok(columns); }
            columns *= 2;
        }
    }

    /// The fixed contact curves of a constant-twist motion, chained: the
    /// contact curves of every face, the fans of every edge, trimmed where the
    /// tool's own field hides them, joined by endpoint identity.
    pub fn characteristics(&self,tolerance: f64) -> Result<Vec<Characteristic>,String> {
        let scale = self.scale();
        let signs = self.signs(scale)?;
        self.characteristics_at(self.motion.at(self.roll[0])?,&signs,tolerance,scale)
    }

    /// The contact curves of every face and edge under one instantaneous
    /// motion, trimmed and chained, in the tool's frame.
    /// The untrimmed, unchained pieces at one parameter, each with the face,
    /// edge or crease it came from: what the tracer works from, for inspection.
    pub fn pieces_at(&self,t: f64,tolerance: f64) -> Result<Vec<(String,Characteristic)>,String> {
        let scale = self.scale();
        let signs = self.signs(scale)?;
        let pose = self.motion.at(t)?;
        let mut out = Vec::new();
        for (index,face) in self.faces.iter().enumerate() {
            for c in self.contact_curves(face,signs.signs[index],&signs.shadows[index],pose,tolerance,scale)? { out.push((format!("face {index}"),c)); }
        }
        for edge in &self.edges {
            for c in self.edge_curves(EdgeRef::Charted(edge),&signs.signs,pose,tolerance,scale)? { out.push((format!("edge {:?}",edge.faces),c)); }
        }
        for crease in &self.creases {
            for c in self.edge_curves(EdgeRef::Crease(crease),&signs.signs,pose,tolerance,scale)? { out.push((format!("crease {:?}",crease.faces),c)); }
        }
        Ok(out)
    }

    fn characteristics_at(&self,pose: Motion,signs: &Signs,tolerance: f64,scale: f64) -> Result<Vec<Characteristic>,String> {
        let mut pieces: Vec<Characteristic> = Vec::new();
        let Signs {signs,shadows} = signs;
        for (index,face) in self.faces.iter().enumerate() {
            pieces.extend(self.contact_curves(face,signs[index],&shadows[index],pose,tolerance,scale)?);
        }
        for edge in &self.edges {
            pieces.extend(self.edge_curves(EdgeRef::Charted(edge),signs,pose,tolerance,scale)?);
        }
        for crease in &self.creases {
            pieces.extend(self.edge_curves(EdgeRef::Crease(crease),signs,pose,tolerance,scale)?);
        }
        Ok(chain(self.trimmed(pieces,scale)?,JOIN*scale*10.))
    }

    /// The tool's extent, for tolerances.
    fn scale(&self) -> f64 {
        self.faces.iter().map(|f| f.at(0.5,0.).map(|s| norm(s.position)).unwrap_or(0.)).fold(1_f64,f64::max)
    }

    /// Each face's outward sign, decided once by the tool's field, and the
    /// faces each is shadowed by: earlier faces of other operands on the same
    /// carrier facing the same way, whose extent it leaves to them. Two
    /// operands of the tool's Boolean with a face on one plane or cone (the
    /// base planes of two crowns) would otherwise trace one strand twice.
    fn signs(&self,scale: f64) -> Result<Signs,String> {
        let signs: Vec<f64> = self.faces.iter().map(|face| self.outward_sign(face,scale)).collect::<Result<_,_>>()?;
        let mut shadows: Vec<Vec<usize>> = vec![Vec::new();self.faces.len()];
        for j in 0..self.faces.len() {
            let fj = &self.faces[j];
            let [[u0,u1],[v0,v1]] = fj.domain();
            let samples: Vec<(V3,V3)> = (1..4).flat_map(|a| (1..4).map(move |b| (u0+(u1-u0)*a as f64/4.,v0+(v1-v0)*b as f64/4.)))
                .filter_map(|(u,v)| fj.normal(u,v,signs[j]).ok()).collect();
            if samples.is_empty() { continue; }
            for i in 0..j {
                if self.operands[i] == self.operands[j] { continue; }
                let fi = &self.faces[i];
                let coincident = samples.iter().all(|(p,n)| {
                    fi.implicit(*p).is_some_and(|g| g.abs() < 1e-7*scale)
                        && fi.parameters(*p).and_then(|(u,v)| fi.normal(u,v,signs[i]).ok()).map_or(true,|(_,m)| plane::dot(*n,m) > 0.)
                });
                if coincident { shadows[j].push(i); }
            }
        }
        Ok(Signs {signs,shadows})
    }

    /// Whether a point of a face's carrier is off the tool's boundary or on a
    /// face that shadows this one.
    fn off_face(&self,shadows: &[usize],p: V3,scale: f64) -> Result<bool,String> {
        if self.hidden(p,scale)? { return Ok(true); }
        Ok(shadows.iter().any(|&i| self.faces[i].parameters(p).is_some()))
    }

    /// The roots of the station equations of one face under `motion`, per
    /// station, in the tool's frame: `(v, position, outward normal)`, only
    /// where the face proper is and the tool's field does not hide them. A
    /// station with no varying part or no extent contributes an empty row.
    fn station_rows(&self,face: &ToolFace,sign: f64,shadows: &[usize],motion: Motion,tolerance: f64,scale: f64)
        -> Result<Vec<Vec<Root>>,String> {
        let mut rows = Vec::with_capacity(STATIONS+1);
        for i in 0..=STATIONS {
            let u = i as f64/STATIONS as f64;
            let mut row = Vec::new();
            match face.station(u,motion) {
                Ok(equation) => {
                    let (amplitude,_) = equation.amplitude_and_constant();
                    // A station where the two roots coincide (a fold exactly there)
                    // has no isolated root; its neighbours' refinement reaches the fold.
                    let roots = match equation.roots(tolerance) {
                        Ok(roots) => roots,
                        Err(envelope::Error::Degenerate) => Vec::new(),
                        Err(e) => return Err(format!("station {u}: {e:?}")),
                    };
                    if amplitude > tolerance {
                        for (_,v) in roots {
                            if !face.contains(u,v) { continue; }
                            let s = face.at(u,v).map_err(|e| format!("station {u} root {v}: {e:?}"))?;
                            let contact = envelope::contact(s,motion).map_err(|e| format!("station {u} root {v} contact: {e:?}"))?;
                            if contact.normal_velocity.abs() > tolerance { return Err("a station root did not converge".into()); }
                            let (p,n) = face.normal(u,v,sign).map_err(|e| format!("station {u} root {v} normal: {e:?}"))?;
                            if self.off_face(shadows,p,scale)? { continue; }
                            row.push((v,p,n));
                        }
                    }
                }
                Err(envelope::Error::Degenerate) => {}
                Err(e) => return Err(format!("{e:?}")),
            }
            rows.push(row);
        }
        Ok(rows)
    }

    /// Station rows linked into strands by following each root from one
    /// station to the next: a march that halves its step wherever the root
    /// near the last one is lost, so a root moving fast along its station is
    /// followed and one that vanishes is followed to where it does. A march
    /// that reaches the next station links to the root it lands on; one that
    /// does not ends the strand exactly, at the face's edge or, where the
    /// root folded onto its other branch, at the fold, which both branches
    /// reach and so chain across.
    fn station_strands(&self,face: &ToolFace,sign: f64,shadows: &[usize],motion: Motion,tolerance: f64,scale: f64,rows: Vec<Vec<Root>>)
        -> Result<Vec<Strand>,String> {
        let root_near = |u: f64,near_v: f64| -> Result<Option<Root>,String> {
            let Ok(equation) = face.station(u,motion) else { return Ok(None); };
            let Ok(roots) = equation.roots(tolerance) else { return Ok(None); };
            let Some(&(_,v)) = roots.iter().filter(|(_,v)| face.contains(u,*v))
                .min_by(|a,b| angle_gap(a.1,near_v).total_cmp(&angle_gap(b.1,near_v))) else { return Ok(None); };
            if angle_gap(v,near_v) >= 0.02 { return Ok(None); }
            let (p,n) = face.normal(u,v,sign).map_err(|e| format!("{e:?}"))?;
            if self.off_face(shadows,p,scale)? { return Ok(None); }
            Ok(Some((v,p,n)))
        };
        // The march from a root at `from` toward `to`: the last root reached,
        // its parameter, whether `to` was reached, and the parameter just past
        // the end where the root was lost.
        struct March { root: Root, u: f64, reached: bool, lost: f64 }
        let march = |from: f64,to: f64,root: Root| -> Result<March,String> {
            let (mut u,mut current,mut step,mut lost) = (from,root,to-from,to);
            for _ in 0..200 {
                if step.abs() < 1e-13 { break; }
                let next = if (to-from) > 0. { (u+step).min(to) } else { (u+step).max(to) };
                match root_near(next,current.0)? {
                    Some(r) => { u = next; current = r; if u == to { return Ok(March {root:current,u,reached:true,lost:to}); } }
                    None => { lost = next; step *= 0.5; }
                }
            }
            Ok(March {root:current,u,reached:false,lost})
        };
        // A root lost by folding onto its other branch ends exactly at the fold,
        // where the station's constant reaches its amplitude.
        let fold_end = |m: &March| -> Result<Option<Root>,String> {
            let margin = |u: f64| face.station(u,motion).ok().map(|e| { let (a,c) = e.amplitude_and_constant(); a-c.abs() });
            let (Some(m_lo),Some(m_hi)) = (margin(m.u),margin(m.lost)) else { return Ok(None) };
            let (amplitude,_) = face.station(m.lost,motion).map_err(|e| format!("{e:?}"))?.amplitude_and_constant();
            // lost within the degenerate band about the fold, or past it
            if !(amplitude > tolerance && m_lo > 0. && m_hi <= tolerance) { return Ok(None); }
            let (mut f_lo,mut f_hi) = (m.u,m.lost);
            for _ in 0..50 {
                let mid = 0.5*(f_lo+f_hi);
                match margin(mid) { Some(x) if x > 0. => f_lo = mid,Some(_) => f_hi = mid,None => break }
            }
            let u = 0.5*(f_lo+f_hi);
            let Some(v) = face.station(u,motion).ok().and_then(|e| e.fold(m.root.0)) else { return Ok(None) };
            if !face.contains(u,v) { return Ok(None); }
            let Ok((p,n)) = face.normal(u,v,sign) else { return Ok(None) };
            Ok(if self.off_face(shadows,p,scale)? { None } else { Some((v,p,n)) })
        };
        let end = |m: &March| -> Result<Option<Root>,String> {
            if let Some(r) = fold_end(m)? { return Ok(Some(r)); }
            Ok(Some(m.root))
        };
        let station_u = |i: usize| i as f64/STATIONS as f64;
        let mut done: Vec<Strand> = Vec::new();
        let mut open: Vec<Strand> = Vec::new();
        for (i,row) in rows.into_iter().enumerate() {
            let mut next: Vec<Strand> = Vec::new();
            let mut used = vec![false;row.len()];
            for mut strand in open {
                let last = *strand.roots.last().unwrap();
                let m = march(station_u(i-1),station_u(i),last)?;
                let landed = if m.reached { row.iter().enumerate().filter(|(j,_)| !used[*j])
                    .min_by(|a,b| angle_gap(a.1.0,m.root.0).total_cmp(&angle_gap(b.1.0,m.root.0)))
                    .filter(|(_,r)| angle_gap(r.0,m.root.0) < 1e-6).map(|(j,_)| j) } else { None };
                match landed {
                    Some(j) => { used[j] = true; strand.roots.push(row[j]); next.push(strand); }
                    None => {
                        if let Some(r) = end(&m)? { if distance(r.1,last.1) > scale*1e-12 { strand.tail = Some(r); } }
                        done.push(strand);
                    }
                }
            }
            for (j,&root) in row.iter().enumerate() {
                if used[j] { continue; }
                let mut strand = Strand {roots:vec![root],head:None,tail:None};
                if i > 0 {
                    let m = march(station_u(i),station_u(i-1),root)?;
                    if let Some(r) = end(&m)? { if distance(r.1,root.1) > scale*1e-12 { strand.head = Some(r); } }
                }
                next.push(strand);
            }
            open = next;
        }
        done.extend(open);
        Ok(done)
    }

    /// The contact curves on one face under `motion`: its strands, stationary
    /// stations as whole rings, and poles joined.
    fn contact_curves(&self,face: &ToolFace,sign: f64,shadows: &[usize],motion: Motion,tolerance: f64,scale: f64)
        -> Result<Vec<Characteristic>,String> {
        let mut pieces: Vec<Characteristic> = Vec::new();
        let [_,v_domain] = face.domain();
        let ring = |u: f64| -> Result<Characteristic,String> {
            let mut points = Vec::new(); let mut normals = Vec::new();
            for j in 0..4*STATIONS {
                let v = v_domain[0]+(v_domain[1]-v_domain[0])*j as f64/(4*STATIONS) as f64;
                let (p,n) = face.normal(u,v,sign).map_err(|e| format!("{e:?}"))?;
                points.push(p); normals.push(n);
            }
            Ok(Characteristic {points,normals,closed:true,joints:Vec::new()})
        };
        // With zero amplitude the station's equation is a constant: the whole
        // station is in contact where that constant vanishes. A station with no
        // varying part between varying ones is a ring; a zero shared with a
        // neighbour is a stationary face, covered by the caps and edges; and a
        // sign change of the constant between two such stations is a ring too.
        let constant = |u: f64| -> Result<f64,String> {
            Ok(face.station(u,motion).map_err(|e| format!("{e:?}"))?.amplitude_and_constant().1)
        };
        let equations = (0..=STATIONS).map(|i| match face.station(i as f64/STATIONS as f64,motion) {
            Ok(e) => Ok(Some(e)),
            Err(envelope::Error::Degenerate) => Ok(None),
            Err(e) => Err(format!("{e:?}")),
        }).collect::<Result<Vec<_>,String>>()?;
        let varying = |i: usize| equations.get(i).copied().flatten()
            .is_some_and(|e| e.amplitude_and_constant().0 > tolerance);
        let mut previous_constant: Option<(f64,f64)> = None;
        for i in 0..=STATIONS {
            let u = i as f64/STATIONS as f64;
            let Some(equation) = equations[i] else { previous_constant = None; continue; };
            let (amplitude,c) = equation.amplitude_and_constant();
            if amplitude > tolerance { previous_constant = None; continue; }
            let isolated = (i == 0 || varying(i-1)) && (i == STATIONS || varying(i+1));
            if c.abs() <= tolerance && isolated { pieces.push(ring(u)?); }
            if let Some((u0,c0)) = previous_constant {
                if c.abs() <= tolerance && c0.abs() > tolerance && !isolated { pieces.push(ring(u)?); }
                else if c0.abs() > tolerance && c.abs() > tolerance && (c0 < 0.) != (c < 0.) {
                    let (mut lo,mut hi,mut clo) = (u0,u,c0);
                    for _ in 0..60 {
                        let mid = 0.5*(lo+hi);
                        let cm = constant(mid)?;
                        if (cm < 0.) == (clo < 0.) { lo = mid; clo = cm; } else { hi = mid; }
                    }
                    pieces.push(ring(0.5*(lo+hi))?);
                }
            }
            previous_constant = Some((u,c));
        }
        let rows = self.station_rows(face,sign,shadows,motion,tolerance,scale)?;
        let mut strands: Vec<Characteristic> = self.station_strands(face,sign,shadows,motion,tolerance,scale,rows)?
            .into_iter().map(Strand::into_characteristic).collect();
        // A station on the axis is a pole where every branch meets; a strand
        // reaching the adjacent station is extended to it so the halves chain.
        for (end,neighbour) in [(0.,1./STATIONS as f64),(1.,1.-1./STATIONS as f64)] {
            let Ok(Some((pole,axis))) = face.pole(end,scale) else { continue; };
            // The tangent plane at a pole is perpendicular to the axis.
            let reference = face.normal(neighbour,v_domain[0],sign).map(|(_,n)| n).unwrap_or([0.;3]);
            let n = if plane::dot(axis,reference) < 0. { axis.map(|x| -x) } else { axis };
            for strand in strands.iter_mut().chain(pieces.iter_mut()) {
                let first = strand.points[0]; let last = *strand.points.last().unwrap();
                let near = |p: V3| distance(p,pole) < scale*0.05 && distance(p,pole) > scale*1e-9;
                if near(last) { strand.points.push(pole); strand.normals.push(n); }
                else if near(first) { strand.points.insert(0,pole); strand.normals.insert(0,n); }
            }
        }
        pieces.extend(strands);
        Ok(pieces)
    }

    /// The fan of a sharp edge: the edge points whose outward normal cone
    /// straddles the velocity, each with the contact normal in that cone, as
    /// strands over the edge's samples with exact ends. A smooth or concave
    /// edge contributes nothing.
    fn edge_strands(&self,edge: EdgeRef,signs: &[f64],motion: Motion,tolerance: f64,scale: f64)
        -> Result<Vec<Strand>,String> {
        let [a,b] = edge.faces();
        let (fa,fb) = (&self.faces[a],&self.faces[b]);
        let inverse = motion.inverse();
        // An edge point: its position, both faces' outward normals there and
        // each normal's velocity component. None where the edge is off the
        // tool's boundary or concave, which contributes nothing.
        struct Point { position: V3, normals: [V3;2], speeds: [f64;2] }
        let at = |t: f64| -> Result<Option<Point>,String> {
            let ((ua,va),(ub,vb)) = match edge {
                EdgeRef::Charted(e) => (e.charts[0].at(t,fa),e.charts[1].at(t,fb)),
                EdgeRef::Crease(c) => match c.at(t,fa,fb) { Some(uv) => uv,None => return Ok(None) },
            };
            let sa = fa.at(ua,va).map_err(|e| format!("edge of faces {a} and {b} at {t}: {e:?}"))?;
            let (_,na) = fa.normal(ua,va,signs[a]).map_err(|e| format!("edge normal on face {a} at {t} ({ua},{va}): {e:?}"))?;
            let (_,nb) = fb.normal(ub,vb,signs[b]).map_err(|e| format!("edge normal on face {b} at {t} ({ub},{vb}): {e:?}"))?;
            // Convex when the bisector leaves the tool; a smooth junction is
            // convex enough, its fan being the one point both faces' strands end at.
            let bisector = plane::unit(std::array::from_fn(|k| na[k]+nb[k])).unwrap_or(na);
            let probe: V3 = std::array::from_fn(|k| sa.position[k]+bisector[k]*scale*1e-4);
            let outside = self.source.value(probe) > 0.;
            if !outside || self.hidden(sa.position,scale)? { return Ok(None); }
            let velocity = inverse.vector(motion.velocity(sa.position));
            Ok(Some(Point {position:sa.position,normals:[na,nb],speeds:[plane::dot(na,velocity),plane::dot(nb,velocity)]}))
        };
        // The fan holds a contact where the two normals' velocities differ in
        // strict sign, or one vanishes: the normal between them whose velocity
        // vanishes, or a stationary face's own. An edge both of whose faces
        // are stationary slides along itself and is theirs.
        let class = |x: f64| if x > tolerance { 1 } else if x < -tolerance { -1 } else { 0 };
        let in_fan = |p: &Point| { let (a,b) = (class(p.speeds[0]),class(p.speeds[1])); a*b <= 0 && (a != 0 || b != 0) };
        let fan_normal = |p: &Point| -> V3 {
            let [sa,sb] = p.speeds;
            if sa.abs() <= tolerance { return p.normals[0]; }
            if sb.abs() <= tolerance { return p.normals[1]; }
            let m = sa.abs().max(sb.abs());
            plane::unit(std::array::from_fn(|k| p.normals[0][k]*sb.abs()/m+p.normals[1][k]*sa.abs()/m)).unwrap_or(p.normals[0])
        };
        // The exact parameter between two edge points where face `k`'s normal
        // velocity changes sign, and the point there with that face's normal:
        // where that face's own strand ends, so the two meet exactly.
        let root = |k: usize,mut lo: f64,mut hi: f64,class_lo: i32| -> Result<Option<Root>,String> {
            let mut found: Option<Point> = None;
            let mut t = lo;
            for _ in 0..50 {
                let mid = 0.5*(lo+hi);
                let Some(p) = at(mid)? else { return Ok(None) };
                if class(p.speeds[k]) == class_lo { lo = mid; } else { hi = mid; }
                t = mid; found = Some(p);
            }
            Ok(found.map(|p| (t,p.position,p.normals[k])))
        };
        // Where a fan runs into a part of the edge that contributes nothing,
        // its boundary is bisected on that instead.
        let boundary = |inside: f64,outside: f64| -> Result<Option<Root>,String> {
            let (mut lo,mut hi,mut found) = (inside,outside,None);
            for _ in 0..40 {
                let mid = 0.5*(lo+hi);
                match at(mid)? {
                    Some(p) if in_fan(&p) => { found = Some((mid,p.position,fan_normal(&p))); lo = mid; }
                    _ => hi = mid,
                }
            }
            Ok(found)
        };
        let samples = super::tool_faces::EDGE_SAMPLES;
        let points: Vec<Option<Point>> = (0..=samples).map(|i| at(i as f64/samples as f64)).collect::<Result<_,_>>()?;
        let mut strands = Vec::new();
        let mut current: Option<Strand> = None;
        // A fan of no length (a smooth junction, where both faces' strands
        // already meet) is nothing, and must not stand on their shared point.
        let close = |strand: &mut Option<Strand>,tail: Option<Root>,strands: &mut Vec<Strand>| {
            if let Some(mut s) = strand.take() {
                if let Some(r) = tail { if s.roots.last().map_or(true,|last| distance(last.1,r.1) > scale*1e-12) { s.tail = Some(r); } }
                let points: Vec<V3> = s.head.iter().chain(&s.roots).chain(&s.tail).map(|r| r.1).collect();
                let length: f64 = points.windows(2).map(|w| distance(w[0],w[1])).sum();
                if points.len() >= 2 && length > scale*1e-9 { strands.push(s); }
            }
        };
        for i in 0..=samples {
            let t = i as f64/samples as f64;
            match &points[i] {
                Some(p) if in_fan(p) => {
                    let strand = current.get_or_insert_with(|| Strand {roots:Vec::new(),head:None,tail:None});
                    strand.roots.push((t,p.position,fan_normal(p)));
                }
                Some(_) => close(&mut current,None,&mut strands),
                None => close(&mut current,None,&mut strands),
            }
            if i == samples { break; }
            let t1 = (i+1) as f64/samples as f64;
            // Between this sample and the next, every change of sign of either
            // normal velocity is a point where one face's own strand ends: the
            // fan is cut there, and the pieces chain back by that exact point.
            match (&points[i],&points[i+1]) {
                (Some(p),Some(q)) => {
                    let mut events: Vec<Root> = Vec::new();
                    for k in 0..2 {
                        if class(p.speeds[k]) != class(q.speeds[k]) {
                            if let Some(r) = root(k,t,t1,class(p.speeds[k]))? { events.push(r); }
                        }
                    }
                    events.sort_by(|x,y| x.0.total_cmp(&y.0));
                    for r in events {
                        close(&mut current,Some(r),&mut strands);
                        current = Some(Strand {roots:Vec::new(),head:Some(r),tail:None});
                    }
                    if !in_fan(q) { close(&mut current,None,&mut strands); }
                }
                (Some(p),None) if in_fan(p) => { let r = boundary(t,t1)?; close(&mut current,r,&mut strands); }
                (None,Some(q)) if in_fan(q) => {
                    let r = boundary(t1,t)?;
                    current = Some(Strand {roots:Vec::new(),head:r,tail:None});
                }
                _ => {}
            }
        }
        close(&mut current,None,&mut strands);
        Ok(strands)
    }

    fn edge_curves(&self,edge: EdgeRef,signs: &[f64],motion: Motion,tolerance: f64,scale: f64)
        -> Result<Vec<Characteristic>,String> {
        Ok(self.edge_strands(edge,signs,motion,tolerance,scale)?.into_iter().map(Strand::into_characteristic).collect())
    }

    /// The sheets of a sweep whose twist varies along the motion. At each of a
    /// sequence of motion parameters the contact strands of every face and
    /// edge are traced (cut to `reach`, the part of the body that matters, if
    /// given); each strand is linked to its continuation at the next parameter
    /// by source and position, into strips. Where a strand is born or dies
    /// between two parameters, its source alone is bisected for the parameter
    /// of the event, which then joins the list every source is sampled at, so
    /// the strips of adjacent faces, which end on one shared vertex curve,
    /// hold that curve at the same parameters and meet exactly. Rows are each
    /// strand resampled along its length to within `sagitta` of the traced
    /// points, its exact ends kept; columns are the parameters.
    pub fn characteristic_sheets(&self,spacing: f64,sagitta: f64,overrun: f64,tolerance: f64,
        reach: Option<&dyn Fn(V3) -> bool>) -> Result<Vec<SweepPatch>,String> {
        let scale = self.scale();
        let signs = self.signs(scale)?;
        let [from,to] = self.domain();
        // The fastest material point over the domain sets the parameter step.
        let mut speed: f64 = 1e-9;
        let extents: Vec<V3> = self.faces.iter().flat_map(|f| { let [_,d] = f.domain();
            (0..4).flat_map(move |i| (0..4).filter_map(move |j| f.at(i as f64/3.,d[0]+(d[1]-d[0])*j as f64/3.).ok().map(|s| s.position))) }).collect();
        for t in [from,0.5*(from+to),to] {
            let pose = self.motion.at(t)?;
            for p in &extents { speed = speed.max(norm(pose.velocity(*p))); }
        }
        let (t0,t1) = (from-overrun/speed,to+overrun/speed);
        // The chained contact curves at a parameter, in the tool's frame; where
        // the caller says what region of the body matters, a curve none of
        // whose points (posed) reach it is left out whole. A curve is never cut
        // there: rows are matched between columns by fraction of a curve's
        // length, and a curve cut at a moving boundary changes length between
        // one column and the next, shearing the rows until the sheet folds.
        // Chaining first is what keeps folds and the junctions between faces
        // interior to a curve rather than seams between sheets.
        let curves_at = |t: f64| -> Result<Vec<Characteristic>,String> {
            let pose = self.motion.at(t)?;
            let curves = self.characteristics_at(pose,&signs,tolerance,scale)?;
            let curves: Vec<Characteristic> = match reach {
                Some(reach) => curves.into_iter().filter(|c| c.points.iter().any(|p| reach(pose.point(*p)))).collect(),
                None => curves,
            };
            let length = |c: &Characteristic| c.points.windows(2).map(|w| distance(w[0],w[1])).sum::<f64>();
            Ok(curves.into_iter().filter(|c| c.points.len() >= 2 && length(c) > scale*1e-9).collect())
        };
        // The parameters: as many as the spacing asks at the fastest point and
        // the sagitta of the paths between them allows, over the part of the
        // interval where any curve reaches, found coarsely first and widened by
        // one coarse step each way; a tool that reaches nowhere cuts nothing.
        let (t0,t1) = if reach.is_some() {
            let coarse = 32;
            let reaching: Vec<bool> = (0..=coarse).map(|i| Ok::<_,String>(!curves_at(t0+(t1-t0)*i as f64/coarse as f64)?.is_empty())).collect::<Result<_,_>>()?;
            let Some(first) = reaching.iter().position(|r| *r) else { return Ok(Vec::new()) };
            let last = reaching.iter().rposition(|r| *r).unwrap();
            let step = (t1-t0)/coarse as f64;
            ((t0+step*first as f64-step).max(t0),(t0+step*last as f64+step).min(t1))
        } else { (t0,t1) };
        let count = (((t1-t0)*speed/spacing).ceil() as usize).clamp(8,400);
        let count = self.columns_within_sagitta(&extents,[t0,t1],count+1,sagitta)?-1;
        let mut times: Vec<f64> = (0..=count).map(|c| t0+(t1-t0)*c as f64/count as f64).collect();
        // How far apart two curves are: the farthest any of a coarse sample of
        // either is from the other, so a curve that lost or gained a piece is
        // apart from what it was, and a closed curve is the same however it
        // starts.
        let apart = |a: &Characteristic,b: &Characteristic| -> f64 {
            if a.closed != b.closed { return f64::INFINITY; }
            let to = |from: &Characteristic,onto: &Characteristic| -> f64 {
                let (samples,_) = resample(from,32);
                let n = onto.points.len();
                let segments = if onto.closed { n } else { n-1 };
                samples.iter().map(|p| (0..segments).map(|k| segment_distance(*p,onto.points[k],onto.points[(k+1)%n])).fold(f64::INFINITY,f64::min))
                    .fold(0_f64,f64::max)
            };
            to(a,b).max(to(b,a))
        };
        let continuation = |last: &Characteristic,candidates: &[Characteristic],used: &[bool]| -> Option<usize> {
            candidates.iter().enumerate().filter(|(j,_)| !used[*j]).map(|(j,c)| (j,apart(last,c)))
                .filter(|(_,d)| *d < 2.*spacing).min_by(|a,b| a.1.total_cmp(&b.1)).map(|(j,_)| j)
        };
        // The parameter of every birth and death, found by bisection between
        // the two samples it happened between, joins the list.
        let mut events: Vec<f64> = Vec::new();
        let mut previous: Option<(f64,Vec<Characteristic>)> = None;
        for &t in &times {
            let curves = curves_at(t)?;
            if let Some((t_prev,prev)) = &previous {
                let mut used = vec![false;curves.len()];
                for last in prev {
                    match continuation(last,&curves,&used) {
                        Some(j) => used[j] = true,
                        None => {
                            let (mut lo,mut hi) = (*t_prev,t);
                            for _ in 0..20 {
                                let mid = 0.5*(lo+hi);
                                let mid_c = curves_at(mid)?;
                                if continuation(last,&mid_c,&vec![false;mid_c.len()]).is_some() { lo = mid; } else { hi = mid; }
                            }
                            events.push(lo);
                        }
                    }
                }
                for (j,curve) in curves.iter().enumerate() {
                    if used[j] { continue; }
                    let (mut lo,mut hi) = (*t_prev,t);
                    for _ in 0..20 {
                        let mid = 0.5*(lo+hi);
                        let mid_c = curves_at(mid)?;
                        if continuation(curve,&mid_c,&vec![false;mid_c.len()]).is_some() { hi = mid; } else { lo = mid; }
                    }
                    events.push(hi);
                }
            }
            previous = Some((t,curves));
        }
        times.extend(events);
        times.sort_by(f64::total_cmp);
        times.dedup_by(|a,b| (*a-*b).abs() <= 1e-12*(t1-t0));
        // Strips: every curve tracked across the whole parameter list.
        struct Strip { columns: Vec<(f64,Characteristic)> }
        let mut strips: Vec<Strip> = Vec::new();
        let mut open: Vec<Strip> = Vec::new();
        for &t in &times {
            let curves = curves_at(t)?;
            let mut next: Vec<Strip> = Vec::new();
            let mut used = vec![false;curves.len()];
            for mut strip in open {
                match continuation(&strip.columns.last().unwrap().1,&curves,&used) {
                    Some(j) => { used[j] = true; strip.columns.push((t,curves[j].clone())); next.push(strip); }
                    None => strips.push(strip),
                }
            }
            for (j,curve) in curves.into_iter().enumerate() {
                if !used[j] { next.push(Strip {columns:vec![(t,curve)]}); }
            }
            open = next;
        }
        strips.extend(open);
        // Emit each strip as one sheet: every curve turned the way its
        // predecessor runs (and a closed one started nearest its predecessor's
        // start), carried by its pose, and zipped to the next.
        let mut emitted: Vec<(Vec<f64>,Vec<Characteristic>,bool)> = Vec::new();
        for mut strip in strips {
            // A death and the birth beside it are a pair of parameters a hair
            // apart; a strip living through both keeps one of the pair, its
            // own first and last columns always.
            let mut kept: Vec<(f64,Characteristic)> = Vec::new();
            let last = strip.columns.len()-1;
            for (i,column) in strip.columns.drain(..).enumerate() {
                if i != 0 && i != last { if let Some((t,_)) = kept.last() { if (column.0-*t).abs() <= 1e-7*(t1-t0) { continue; } } }
                kept.push(column);
            }
            strip.columns = kept;
            if strip.columns.len() < 2 { continue; }
            let closed = strip.columns[0].1.closed;
            let mut oriented: Vec<Characteristic> = Vec::with_capacity(strip.columns.len());
            for (_,curve) in &strip.columns {
                let mut c = curve.clone();
                if let Some(prev) = oriented.last() {
                    if closed {
                        let start = prev.points[0];
                        let k = (0..c.points.len()).min_by(|&i,&j| distance(c.points[i],start).total_cmp(&distance(c.points[j],start))).unwrap();
                        c = c.rotated(k);
                    }
                    // Judged with the normals as well as the positions, since a
                    // curve symmetric about its middle reads nearly the same
                    // reversed, and its two arms' normals do not.
                    let (pa,na) = resample(prev,16); let (pb,nb) = resample(&c,16);
                    let gap = |i: usize,j: usize| distance(pa[i],pb[j])+spacing*distance(na[i],nb[j]);
                    let forward = (0..16).map(|i| gap(i,i)).sum::<f64>();
                    let backward = (0..16).map(|i| gap(i,if closed { (16-i)%16 } else { 15-i })).sum::<f64>();
                    if backward < forward {
                        c = c.reversed();
                        if closed { let n = c.points.len(); c = c.rotated(n-1); }
                    }
                }
                oriented.push(c);
            }
            let times: Vec<f64> = strip.columns.iter().map(|(t,_)| *t).collect();
            emitted.push((times,oriented,closed));
        }
        let mut sheets = Vec::new();
        for (times,oriented,closed) in emitted {
            let mut points = Vec::new(); let mut normals = Vec::new(); let mut column = Vec::new();
            let mut triangles: Vec<[u32;3]> = Vec::new();
            let mut first_of: Vec<u32> = Vec::new();
            for (c,(t,curve)) in times.iter().zip(&oriented).enumerate() {
                let pose = self.motion.at(*t)?;
                first_of.push(points.len() as u32);
                for (p,n) in curve.points.iter().zip(&curve.normals) { points.push(pose.point(*p)); normals.push(pose.vector(*n)); column.push(c as u32); }
                if c > 0 {
                    let (a,b) = (first_of[c-1],first_of[c]);
                    for [i,j,k] in zip_pieces(&oriented[c-1],&oriented[c]) {
                        let at = |x: (bool,u32)| if x.0 { b+x.1 } else { a+x.1 };
                        triangles.push([at(i),at(j),at(k)]);
                    }
                }
            }
            sheets.push(SweepPatch {points,normals,triangles,column,times,closed});
        }
        if sheets.is_empty() { return Err("the tool has no contact curve under its motion".into()); }
        Ok(sheets)
    }

    /// Pieces with their hidden portions dropped, split where hidden.
    fn trimmed(&self,pieces: Vec<Characteristic>,scale: f64) -> Result<Vec<Characteristic>,String> {
        let mut trimmed = Vec::new();
        for piece in pieces {
            let hidden: Vec<bool> = piece.points.iter().map(|p| self.hidden(*p,scale)).collect::<Result<_,_>>()?;
            if !hidden.iter().any(|h| *h) { trimmed.push(piece); continue; }
            let mut run: Option<Characteristic> = None;
            for ((p,n),h) in piece.points.iter().zip(&piece.normals).zip(&hidden) {
                if *h {
                    if let Some(r) = run.take() { if r.points.len() > 1 { trimmed.push(r); } }
                } else {
                    run.get_or_insert_with(|| Characteristic::open(vec![],vec![]));
                    let r = run.as_mut().unwrap(); r.points.push(*p); r.normals.push(*n);
                }
            }
            if let Some(r) = run { if r.points.len() > 1 { trimmed.push(r); } }
        }
        Ok(trimmed)
    }
}

/// An open curve resampled to `rows` points by fraction of its length, its
/// ends kept exactly.
fn resample(c: &Characteristic,rows: usize) -> (Vec<V3>,Vec<V3>) {
    let n = c.points.len();
    let segments = if c.closed { n } else { n-1 };
    let mut length = vec![0.;segments+1];
    for i in 1..=segments { length[i] = length[i-1]+distance(c.points[i%n],c.points[i-1]); }
    let total = length[segments];
    let steps = if c.closed { rows } else { rows-1 };
    let mut points = Vec::with_capacity(rows); let mut normals = Vec::with_capacity(rows);
    for r in 0..rows {
        if n == 1 || total == 0. { points.push(c.points[0]); normals.push(c.normals[0]); continue; }
        let target = total*r as f64/steps as f64;
        let i = length.iter().position(|&l| l >= target).unwrap_or(segments).clamp(1,segments);
        let f = ((target-length[i-1])/(length[i]-length[i-1]).max(f64::MIN_POSITIVE)).clamp(0.,1.);
        let (a,b) = ((i-1)%n,i%n);
        points.push(std::array::from_fn(|k| c.points[a][k]+f*(c.points[b][k]-c.points[a][k])));
        let m: V3 = std::array::from_fn(|k| c.normals[a][k]+f*(c.normals[b][k]-c.normals[a][k]));
        normals.push(plane::unit(m).unwrap_or(c.normals[b]));
    }
    (points,normals)
}


fn segment_distance(p: V3,a: V3,b: V3) -> f64 {
    let ab = sub(b,a); let ap = sub(p,a);
    let l = plane::dot(ab,ab);
    let t = if l > 0. { (plane::dot(ap,ab)/l).clamp(0.,1.) } else { 0. };
    distance(p,std::array::from_fn(|k| a[k]+t*ab[k]))
}


/// The triangles between two consecutive curves of a strip: the monotone
/// walk along both, one vertex at a time, whose rungs (the edges from a
/// vertex of one curve to a vertex of the other) are shortest in total, by
/// dynamic programming, so the correspondence follows the shapes and never
/// folds where a curve doubles back on a fillet. Indices are `(on the second
/// curve, index)`. A closed pair is walked from the second's vertex nearest
/// the first's start and wraps.
fn zip(a: &Characteristic,b: &Characteristic) -> Vec<[(bool,u32);3]> {
    zip_polylines(&a.points,&b.points,a.closed && b.closed)
}

/// `zip` over bare polylines: the second's indices are its own, whichever
/// vertex the walk starts from.
pub fn zip_polylines(a: &[V3],b: &[V3],closed: bool) -> Vec<[(bool,u32);3]> {
    let (na,nb) = (a.len(),b.len());
    if na < 2 || nb < 2 { return Vec::new(); }
    let offset = if closed {
        (0..nb).min_by(|&i,&j| distance(b[i],a[0]).total_cmp(&distance(b[j],a[0]))).unwrap()
    } else { 0 };
    // the walks: an open pair over every vertex, a closed pair once round and
    // back to the start
    let (la,lb) = if closed { (na+1,nb+1) } else { (na,nb) };
    let pa = |i: usize| a[i%na];
    let pb = |j: usize| b[(j+offset)%nb];
    // cost[i][j]: the shortest total rung length of a walk reaching (i, j)
    let mut cost = vec![f64::INFINITY;la*lb];
    let mut from = vec![0u8;la*lb]; // 1: from (i-1, j), 2: from (i, j-1)
    let at = |i: usize,j: usize| i*lb+j;
    cost[0] = distance(pa(0),pb(0));
    for i in 0..la { for j in 0..lb {
        if i == 0 && j == 0 { continue; }
        let rung = distance(pa(i),pb(j));
        let (mut best,mut via) = (f64::INFINITY,0);
        if i > 0 && cost[at(i-1,j)]+rung < best { best = cost[at(i-1,j)]+rung; via = 1; }
        if j > 0 && cost[at(i,j-1)]+rung < best { best = cost[at(i,j-1)]+rung; via = 2; }
        cost[at(i,j)] = best; from[at(i,j)] = via;
    } }
    // walk back from the far corner, emitting a triangle per step
    let ai = |i: usize| (false,(i%na) as u32);
    let bj = |j: usize| (true,((j+offset)%nb) as u32);
    let mut out = Vec::new();
    let (mut i,mut j) = (la-1,lb-1);
    while i > 0 || j > 0 {
        match from[at(i,j)] {
            1 => { out.push([ai(i-1),ai(i),bj(j)]); i -= 1; }
            _ => { out.push([ai(i),bj(j),bj(j-1)]); j -= 1; }
        }
    }
    out.reverse();
    out
}

/// The triangles between two consecutive curves of a strip, piece by piece:
/// each piece of one, between its joints, is zipped with the piece of the
/// other that lies alongside it, both walked in order, so a corner at a
/// joint is a rung shared by two bands and never inside one. A piece with
/// no counterpart (one that appeared or vanished between the two
/// parameters) is zipped with the point where it would have been, a fan.
fn zip_pieces(a: &Characteristic,b: &Characteristic) -> Vec<[(bool,u32);3]> {
    if a.joints.is_empty() && b.joints.is_empty() { return zip(a,b); }
    let (pa,pb) = (a.pieces(),b.pieces());
    let (na,nb) = (a.points.len(),b.points.len());
    let poly = |c: &Characteristic,(s,e): (usize,usize)| -> Vec<V3> { (s..=e).map(|i| c.points[i%c.points.len()]).collect() };
    // how far a piece is from another, both ways, by a coarse sample
    let apart = |x: &[V3],y: &[V3]| -> f64 {
        let to = |from: &[V3],onto: &[V3]| (0..8).map(|k| { let p = from[k*(from.len()-1)/7];
            onto.windows(2).map(|w| segment_distance(p,w[0],w[1])).fold(f64::INFINITY,f64::min) }).fold(0_f64,f64::max);
        to(x,y).max(to(y,x))
    };
    // order-preserving alignment of the two piece lists: a match costs its
    // distance, a piece left alone costs its length
    let length = |x: &[V3]| x.windows(2).map(|w| distance(w[0],w[1])).sum::<f64>();
    let (ma,mb) = (pa.len(),pb.len());
    let polys_a: Vec<Vec<V3>> = pa.iter().map(|&r| poly(a,r)).collect();
    let polys_b: Vec<Vec<V3>> = pb.iter().map(|&r| poly(b,r)).collect();
    let mut cost = vec![f64::INFINITY;(ma+1)*(mb+1)];
    let mut from = vec![0u8;(ma+1)*(mb+1)];
    let at = |i: usize,j: usize| i*(mb+1)+j;
    cost[0] = 0.;
    for i in 0..=ma { for j in 0..=mb {
        if i == 0 && j == 0 { continue; }
        let (mut best,mut via) = (f64::INFINITY,0);
        if i > 0 && j > 0 { let c = cost[at(i-1,j-1)]+apart(&polys_a[i-1],&polys_b[j-1]); if c < best { best = c; via = 3; } }
        if i > 0 { let c = cost[at(i-1,j)]+length(&polys_a[i-1]); if c < best { best = c; via = 1; } }
        if j > 0 { let c = cost[at(i,j-1)]+length(&polys_b[j-1]); if c < best { best = c; via = 2; } }
        cost[at(i,j)] = best; from[at(i,j)] = via;
    } }
    let mut steps = Vec::new();
    let (mut i,mut j) = (ma,mb);
    while i > 0 || j > 0 { let v = from[at(i,j)]; steps.push((v,i,j)); match v { 3 => { i -= 1; j -= 1; } 1 => i -= 1,_ => j -= 1 } }
    steps.reverse();
    let mut out = Vec::new();
    let ai = |k: usize| (false,(k%na) as u32);
    let bj = |k: usize| (true,(k%nb) as u32);
    for (v,i,j) in steps {
        match v {
            3 => {
                let (ra,rb) = (pa[i-1],pb[j-1]);
                for t in zip_polylines(&polys_a[i-1],&polys_b[j-1],false) {
                    out.push(t.map(|(on_b,k)| if on_b { bj(rb.0+k as usize) } else { ai(ra.0+k as usize) }));
                }
            }
            // a piece of a with no counterpart fans onto b's point at the
            // joint reached so far (j pieces of b consumed: its start index)
            1 => { let ra = pa[i-1]; let q = if j < mb { pb[j].0 } else { pb[mb-1].1 };
                for k in ra.0..ra.1 { out.push([ai(k),ai(k+1),bj(q)]); } }
            _ => { let rb = pb[j-1]; let p = if i < ma { pa[i].0 } else { pa[ma-1].1 };
                for k in rb.0..rb.1 { out.push([ai(p),bj(k+1),bj(k)]); } }
        }
    }
    out
}

fn angle_gap(a: f64,b: f64) -> f64 { let d = (a-b).rem_euclid(1.); d.min(1.-d) }

/// Chain open pieces whose ends coincide; a piece that meets itself closes.
fn chain(pieces: Vec<Characteristic>,tolerance: f64) -> Vec<Characteristic> {
    // A piece already closed (a stationary ring) is whole; other pieces may
    // end on it without joining it.
    let (mut result,mut pieces): (Vec<_>,Vec<_>) = pieces.into_iter().partition(|p| p.closed);
    // Pieces join only at a vertex exactly two of their ends share: where more
    // meet (a contact line and two fans at a tool vertex) the pieces stay
    // separate, since threading them into one curve would carry a sheet through
    // itself. The ends are counted once, before any joining.
    let ends: Vec<V3> = pieces.iter().flat_map(|p| [p.points[0],*p.points.last().unwrap()]).collect();
    let degree = |q: V3| ends.iter().filter(|e| distance(**e,q) < tolerance).count();
    while let Some(mut current) = pieces.pop() {
        loop {
            let tail = *current.points.last().unwrap();
            let head = current.points[0];
            if current.points.len() > 2 && distance(tail,head) < tolerance && degree(tail) == 2 {
                current.points.pop(); current.normals.pop(); current.closed = true;
                current.joints.retain(|&j| j < current.points.len()); break;
            }
            // `a` then `b`, joined at a's last point, which is a joint
            let append = |a: Characteristic,b: Characteristic| -> Characteristic {
                let n = a.points.len();
                let mut joints = a.joints.clone(); joints.push(n-1);
                joints.extend(b.joints.iter().map(|&j| j+n-1));
                let mut points = a.points; points.extend(b.points.into_iter().skip(1));
                let mut normals = a.normals; normals.extend(b.normals.into_iter().skip(1));
                Characteristic {points,normals,closed:false,joints}
            };
            let mut joined = false;
            for i in 0..pieces.len() {
                let (first,last) = (pieces[i].points[0],*pieces[i].points.last().unwrap());
                if distance(first,tail) < tolerance && degree(tail) == 2 {
                    let piece = pieces.remove(i);
                    current = append(current,piece); joined = true; break;
                }
                if distance(last,tail) < tolerance && degree(tail) == 2 {
                    let piece = pieces.remove(i).reversed();
                    current = append(current,piece); joined = true; break;
                }
                if distance(last,head) < tolerance && degree(head) == 2 {
                    let piece = pieces.remove(i);
                    current = append(piece,current); joined = true; break;
                }
                if distance(first,head) < tolerance && degree(head) == 2 {
                    let piece = pieces.remove(i).reversed();
                    current = append(piece,current); joined = true; break;
                }
            }
            if !joined { break; }
        }
        result.push(current);
    }
    result
}
