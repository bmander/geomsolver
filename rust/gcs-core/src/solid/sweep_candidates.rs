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
use super::{SweepContacts,ToolFace,ToolEdge};
use crate::{envelope::{self,EdgePoint,Motion},interval::Interval,plane};

type V3 = [f64;3];
fn sub(a: V3,b: V3) -> V3 { std::array::from_fn(|k| a[k]-b[k]) }
fn norm(a: V3) -> f64 { plane::dot(a,a).sqrt() }
fn distance(a: V3,b: V3) -> f64 { norm(sub(a,b)) }

/// A polyline of contact positions with outward tool normals, in the tool's
/// own frame. `closed` means the last point joins the first.
#[derive(Clone,Debug)]
pub struct Characteristic {
    pub points: Vec<V3>,
    pub normals: Vec<V3>,
    pub closed: bool,
}

/// A station root in the tool's frame: `(v, position, outward normal)`.
type Root = (f64,V3,V3);

/// A run of station roots at consecutive stations from `first`, with the
/// exact points where it enters and leaves the contact set between stations.
#[derive(Clone,Debug)]
struct Strand { first: usize, roots: Vec<Root>, head: Option<Root>, tail: Option<Root> }

impl Strand {
    fn into_characteristic(self) -> Characteristic {
        let mut points = Vec::with_capacity(self.roots.len()+2);
        let mut normals = Vec::with_capacity(self.roots.len()+2);
        for (_,p,n) in self.head.iter().chain(&self.roots).chain(&self.tail) { points.push(*p); normals.push(*n); }
        Characteristic {points,normals,closed:false}
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
        let (u,v) = face.interior().map_err(|e| format!("{e:?}"))?;
        let s = face.at(u,v).map_err(|e| format!("{e:?}"))?;
        let n = envelope::contact(s,Motion::identity()).map_err(|e| format!("{e:?}"))?.normal;
        let probe: V3 = std::array::from_fn(|k| s.position[k]+n[k]*scale*1e-4);
        let value = self.source.bounds(probe.map(|x| Interval::point(x).unwrap())).map_err(|e| format!("{e:?}"))?;
        Ok(if value.bounds()[0] > 0. { 1. } else if value.bounds()[1] < 0. { -1. } else {
            return Err("a patch's outward side could not be decided from the tool's field".into());
        })
    }

    fn hidden(&self,p: V3,scale: f64) -> Result<bool,String> {
        let value = self.source.bounds(p.map(|x| Interval::point(x).unwrap())).map_err(|e| format!("{e:?}"))?;
        Ok(value.bounds()[1] < -scale*1e-6)
    }

    /// The sheets of a constant-twist sweep: each characteristic carried along
    /// the motion over the declared interval, plus `overrun` (a length) of travel
    /// beyond both ends into the tool's end poses, sampled about `spacing` apart
    /// along the travel. Rows are the characteristic's points, columns the times.
    pub fn carried_sheets(&self,spacing: f64,overrun: f64,tolerance: f64) -> Result<Vec<SweepSheet>,String> {
        let [from,to] = self.domain();
        let curves = self.characteristics(tolerance)?;
        if curves.is_empty() { return Err("the tool has no contact curve under its motion".into()); }
        let motion = self.motion();
        let mut sheets = Vec::with_capacity(curves.len());
        for curve in curves {
            let start = motion.at(from)?;
            let speed = curve.points.iter().map(|p| norm(start.velocity(*p))).fold(0_f64,f64::max).max(1e-9);
            let (t0,t1) = (from-overrun/speed,to+overrun/speed);
            let columns = (((t1-t0)*speed/spacing).ceil() as usize).clamp(2,400);
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

    /// The fixed contact curves of a constant-twist motion, chained: the
    /// contact curves of every face, the fans of every edge, trimmed where the
    /// tool's own field hides them, joined by endpoint identity.
    pub fn characteristics(&self,tolerance: f64) -> Result<Vec<Characteristic>,String> {
        let pose = self.motion.at(self.roll[0])?;
        let scale = self.scale();
        let signs = self.signs(scale)?;
        let mut pieces: Vec<Characteristic> = Vec::new();
        for (index,face) in self.faces.iter().enumerate() {
            pieces.extend(self.contact_curves(face,signs[index],pose,tolerance,scale)?);
        }
        for edge in &self.edges {
            pieces.extend(self.edge_curves(edge,&signs,pose,tolerance,scale)?);
        }
        Ok(chain(self.trimmed(pieces,scale)?,JOIN*scale*10.))
    }

    /// The tool's extent, for tolerances.
    fn scale(&self) -> f64 {
        self.faces.iter().map(|f| f.at(0.5,0.).map(|s| norm(s.position)).unwrap_or(0.)).fold(1_f64,f64::max)
    }

    /// Each face's outward sign, decided once by the tool's field.
    fn signs(&self,scale: f64) -> Result<Vec<f64>,String> {
        self.faces.iter().map(|face| self.outward_sign(face,scale)).collect()
    }

    /// The roots of the station equations of one face under `motion`, per
    /// station, in the tool's frame: `(v, position, outward normal)`, only
    /// where the face proper is and the tool's field does not hide them. A
    /// station with no varying part or no extent contributes an empty row.
    fn station_rows(&self,face: &ToolFace,sign: f64,motion: Motion,tolerance: f64,scale: f64)
        -> Result<Vec<Vec<Root>>,String> {
        let mut rows = Vec::with_capacity(STATIONS+1);
        for i in 0..=STATIONS {
            let u = i as f64/STATIONS as f64;
            let mut row = Vec::new();
            match face.station(u,motion) {
                Ok(equation) => {
                    let (amplitude,_) = equation.amplitude_and_constant();
                    if amplitude > tolerance {
                        for (_,v) in equation.roots(tolerance).map_err(|e| format!("{e:?}"))? {
                            if !face.contains(u,v) { continue; }
                            let s = face.at(u,v).map_err(|e| format!("{e:?}"))?;
                            let contact = envelope::contact(s,motion).map_err(|e| format!("{e:?}"))?;
                            if contact.normal_velocity.abs() > tolerance { return Err("a station root did not converge".into()); }
                            let (p,n) = face.normal(u,v,sign).map_err(|e| format!("{e:?}"))?;
                            if self.hidden(p,scale)? { continue; }
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

    /// Station rows linked into strands by nearest angle between stations. An
    /// unmatched root ends or starts a strand, and the exact end is found
    /// between the two stations by bisection: where the root leaves the face's
    /// domain, or where two branches fold together. Both branches of a fold
    /// refine to the same point, so they chain like any other pieces.
    fn station_strands(&self,face: &ToolFace,sign: f64,motion: Motion,tolerance: f64,scale: f64,rows: Vec<Vec<Root>>)
        -> Result<Vec<Strand>,String> {
        let root_near = |u: f64,near_v: f64| -> Result<Option<Root>,String> {
            let Ok(equation) = face.station(u,motion) else { return Ok(None); };
            let Ok(roots) = equation.roots(tolerance) else { return Ok(None); };
            let Some(&(_,v)) = roots.iter().filter(|(_,v)| face.contains(u,*v))
                .min_by(|a,b| angle_gap(a.1,near_v).total_cmp(&angle_gap(b.1,near_v))) else { return Ok(None); };
            if angle_gap(v,near_v) >= 0.02 { return Ok(None); }
            let (p,n) = face.normal(u,v,sign).map_err(|e| format!("{e:?}"))?;
            if self.hidden(p,scale)? { return Ok(None); }
            Ok(Some((v,p,n)))
        };
        // The last parameter between `have` (a root near `v`) and `lack` (none) at
        // which the root still exists, and the root there.
        let refine = |have: f64,lack: f64,v: f64| -> Result<Option<Root>,String> {
            let (mut lo,mut hi,mut found) = (have,lack,None);
            let mut near_v = v;
            for _ in 0..40 {
                let mid = 0.5*(lo+hi);
                match root_near(mid,near_v)? {
                    Some(r) => { lo = mid; near_v = r.0; found = Some(r); }
                    None => hi = mid,
                }
            }
            Ok(found)
        };
        let station_u = |i: usize| i as f64/STATIONS as f64;
        let mut done: Vec<Strand> = Vec::new();
        let mut open: Vec<Strand> = Vec::new();
        for (i,row) in rows.into_iter().enumerate() {
            let mut next: Vec<Strand> = Vec::new();
            let mut used = vec![false;row.len()];
            for mut strand in open {
                let last_v = strand.roots.last().unwrap().0;
                let nearest = row.iter().enumerate().filter(|(j,_)| !used[*j])
                    .min_by(|a,b| angle_gap(a.1.0,last_v).total_cmp(&angle_gap(b.1.0,last_v)));
                match nearest {
                    Some((j,&root)) if angle_gap(root.0,last_v) < 0.02 => { used[j] = true; strand.roots.push(root); next.push(strand); }
                    _ => {
                        if let Some(r) = refine(station_u(i-1),station_u(i),last_v)? {
                            if distance(r.1,strand.roots.last().unwrap().1) > scale*1e-12 { strand.tail = Some(r); }
                        }
                        done.push(strand)
                    }
                }
            }
            for (j,&root) in row.iter().enumerate() {
                if used[j] { continue; }
                let mut strand = Strand {first:i,roots:vec![root],head:None,tail:None};
                if i > 0 {
                    if let Some(r) = refine(station_u(i),station_u(i-1),root.0)? {
                        if distance(r.1,root.1) > scale*1e-12 { strand.head = Some(r); }
                    }
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
    fn contact_curves(&self,face: &ToolFace,sign: f64,motion: Motion,tolerance: f64,scale: f64)
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
            Ok(Characteristic {points,normals,closed:true})
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
        let rows = self.station_rows(face,sign,motion,tolerance,scale)?;
        let mut strands: Vec<Characteristic> = self.station_strands(face,sign,motion,tolerance,scale,rows)?
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
    fn edge_strands(&self,edge: &ToolEdge,signs: &[f64],motion: Motion,tolerance: f64,scale: f64)
        -> Result<Vec<Strand>,String> {
        let [a,b] = edge.faces;
        let (fa,fb) = (&self.faces[a],&self.faces[b]);
        // The edge point at `t`: position, the contact normal where the velocity
        // is straddled by the two face normals, and whether the edge is sharp and
        // convex there at all.
        enum At { Contact(V3,V3),Outside,Smooth }
        let at = |t: f64| -> Result<At,String> {
            let (ua,va) = edge.charts[0].at(t,fa);
            let (ub,vb) = edge.charts[1].at(t,fb);
            let sa = fa.at(ua,va).map_err(|e| format!("{e:?}"))?;
            let (_,na) = fa.normal(ua,va,signs[a]).map_err(|e| format!("{e:?}"))?;
            let (_,nb) = fb.normal(ub,vb,signs[b]).map_err(|e| format!("{e:?}"))?;
            let angle = plane::dot(na,nb).clamp(-1.,1.).acos();
            if angle < 1e-6 { return Ok(At::Smooth); }
            // Convex when the bisector leaves the tool.
            let bisector = plane::unit(std::array::from_fn(|k| na[k]+nb[k])).unwrap_or(na);
            let probe: V3 = std::array::from_fn(|k| sa.position[k]+bisector[k]*scale*1e-4);
            let outside = self.source.bounds(probe.map(|x| Interval::point(x).unwrap())).map_err(|e| format!("{e:?}"))?.bounds()[0] > 0.;
            if !outside { return Ok(At::Smooth); }
            if self.hidden(sa.position,scale)? { return Ok(At::Outside); }
            let tangent = edge.charts[0].tangent(t,fa,&sa);
            let point = EdgePoint {position:sa.position,tangent,normals:[na,nb],dihedral:-angle};
            match envelope::edge_contact(point,motion,1e-6,tolerance) {
                Ok(Some(contact)) => Ok(At::Contact(sa.position,motion.inverse().vector(contact.normal))),
                Ok(None) | Err(envelope::Error::Degenerate) => Ok(At::Outside),
                Err(e) => Err(format!("{e:?}")),
            }
        };
        // The boundary of a fan lies where one face normal is perpendicular to
        // the velocity, which is where the adjacent face's own strand ends:
        // bisect for it so the two meet exactly.
        let boundary = |inside: f64,outside: f64| -> Result<Option<Root>,String> {
            let (mut lo,mut hi,mut found) = (inside,outside,None);
            for _ in 0..40 {
                let mid = 0.5*(lo+hi);
                match at(mid)? { At::Contact(p,n) => { lo = mid; found = Some((mid,p,n)); } _ => hi = mid }
            }
            Ok(found)
        };
        let mut strands = Vec::new();
        let mut current: Option<Strand> = None;
        let samples = 4*STATIONS;
        let mut last_t: Option<f64> = None;
        for i in 0..=samples {
            let t = i as f64/samples as f64;
            match at(t)? {
                At::Smooth => break,
                At::Contact(p,n) => {
                    let strand = current.get_or_insert_with(|| Strand {first:i,roots:Vec::new(),head:None,tail:None});
                    if strand.roots.is_empty() { if let Some(t0) = last_t {
                        if let Some(r) = boundary(t,t0)? { if distance(r.1,p) > scale*1e-12 { strand.head = Some(r); } }
                    } }
                    strand.roots.push((t,p,n));
                }
                At::Outside => {
                    if let Some(mut strand) = current.take() {
                        if let Some(r) = boundary(last_t.unwrap(),t)? {
                            if distance(r.1,strand.roots.last().unwrap().1) > scale*1e-12 { strand.tail = Some(r); }
                        }
                        strands.push(strand);
                    }
                }
            }
            last_t = Some(t);
        }
        strands.extend(current);
        Ok(strands)
    }

    fn edge_curves(&self,edge: &ToolEdge,signs: &[f64],motion: Motion,tolerance: f64,scale: f64)
        -> Result<Vec<Characteristic>,String> {
        Ok(self.edge_strands(edge,signs,motion,tolerance,scale)?.into_iter().map(Strand::into_characteristic).collect())
    }

    /// The strands of every face and edge at one instant, each tagged by its
    /// source: `(source, strand)` where sources number the faces then the edges.
    fn strands_at(&self,signs: &[f64],motion: Motion,tolerance: f64,scale: f64) -> Result<Vec<(usize,Strand)>,String> {
        let mut out = Vec::new();
        for (index,face) in self.faces.iter().enumerate() {
            let rows = self.station_rows(face,signs[index],motion,tolerance,scale)?;
            out.extend(self.station_strands(face,signs[index],motion,tolerance,scale,rows)?.into_iter().map(|s| (index,s)));
        }
        for (index,edge) in self.edges.iter().enumerate() {
            out.extend(self.edge_strands(edge,signs,motion,tolerance,scale)?.into_iter().map(|s| (self.faces.len()+index,s)));
        }
        Ok(out)
    }

    /// The sheets of a sweep whose twist varies along the motion: the contact
    /// strands of every face and edge at a sequence of motion parameters, each
    /// linked to its continuation at the next parameter by source, station
    /// range and position, into strips. Where a strand appears, disappears or
    /// changes its station range between two parameters, the parameter of that
    /// change is found by bisection and both sides are sampled beside it, so
    /// adjacent strips meet at one column. Rows are the strand's stations with
    /// its exact ends, columns the parameters; a strip's boundary is the
    /// exact locus of an end, a domain limit inside the caps, or the moment a
    /// strand was born or died.
    pub fn characteristic_sheets(&self,spacing: f64,overrun: f64,tolerance: f64) -> Result<Vec<SweepSheet>,String> {
        let scale = self.scale();
        let signs = self.signs(scale)?;
        let [from,to] = self.domain();
        // The fastest material point over the domain sets the parameter step.
        let mut speed: f64 = 1e-9;
        for t in [from,0.5*(from+to),to] {
            let pose = self.motion.at(t)?;
            for face in &self.faces { for i in 0..4 { for j in 0..4 {
                let [_,d] = face.domain();
                if let Ok(s) = face.at(i as f64/3.,d[0]+(d[1]-d[0])*j as f64/3.) { speed = speed.max(norm(pose.velocity(s.position))); }
            } } }
        }
        let (t0,t1) = (from-overrun/speed,to+overrun/speed);
        let count = (((t1-t0)*speed/spacing).ceil() as usize).clamp(8,400);
        let mut times: Vec<f64> = (0..=count).map(|c| t0+(t1-t0)*c as f64/count as f64).collect();
        let strands_at = |t: f64| -> Result<Vec<(usize,Strand)>,String> { self.strands_at(&signs,self.motion.at(t)?,tolerance,scale) };
        let signature = |strands: &[(usize,Strand)]| -> Vec<(usize,usize,usize,bool,bool)> {
            let mut v: Vec<_> = strands.iter().map(|(s,x)| (*s,x.first,x.roots.len(),x.head.is_some(),x.tail.is_some())).collect();
            v.sort(); v
        };
        // Sample, then refine every change of signature to a parameter pair.
        let mut samples: Vec<(f64,Vec<(usize,Strand)>)> = times.iter().map(|&t| strands_at(t).map(|s| (t,s))).collect::<Result<_,_>>()?;
        let mut refined: Vec<(f64,Vec<(usize,Strand)>)> = Vec::new();
        for k in 0..samples.len() {
            if k > 0 && signature(&samples[k-1].1) != signature(&samples[k].1) {
                let (mut lo,mut hi) = (samples[k-1].0,samples[k].0);
                let (mut lo_s,mut hi_s) = (samples[k-1].1.clone(),samples[k].1.clone());
                for _ in 0..30 {
                    let mid = 0.5*(lo+hi);
                    let mid_s = strands_at(mid)?;
                    if signature(&mid_s) == signature(&lo_s) { lo = mid; lo_s = mid_s; } else { hi = mid; hi_s = mid_s; }
                }
                if lo > samples[k-1].0 { refined.push((lo,lo_s)); }
                if hi < samples[k].0 { refined.push((hi,hi_s)); }
            }
            refined.push(std::mem::take(&mut samples[k]));
        }
        times.clear();
        let samples = refined;
        // Link strands across parameters into strips by source, station range
        // overlap and nearest position at a shared station.
        struct Strip { source: usize, columns: Vec<(f64,Strand)> }
        let mut open: Vec<Strip> = Vec::new();
        let mut strips: Vec<Strip> = Vec::new();
        for (t,strands) in samples {
            let mut next: Vec<Strip> = Vec::new();
            let mut used = vec![false;strands.len()];
            for strip in open {
                let (_,last) = strip.columns.last().unwrap();
                let candidate = strands.iter().enumerate().filter(|(j,(s,x))| !used[*j] && *s == strip.source
                    && x.first < last.first+last.roots.len() && last.first < x.first+x.roots.len())
                    .map(|(j,(_,x))| {
                        let i = x.first.max(last.first);
                        (j,distance(x.roots[i-x.first].1,last.roots[i-last.first].1))
                    })
                    .min_by(|a,b| a.1.total_cmp(&b.1));
                match candidate {
                    Some((j,d)) if d < 8.*spacing => {
                        used[j] = true;
                        let mut strip = strip; strip.columns.push((t,strands[j].1.clone())); next.push(strip);
                    }
                    _ => strips.push(strip),
                }
            }
            for (j,(source,strand)) in strands.into_iter().enumerate() {
                if !used[j] { next.push(Strip {source,columns:vec![(t,strand)]}); }
            }
            open = next;
        }
        strips.extend(open);
        // Emit each strip's runs of constant shape as sheets, posed.
        let mut sheets = Vec::new();
        for strip in strips {
            let shape = |s: &Strand| (s.first,s.roots.len(),s.head.is_some(),s.tail.is_some());
            let mut start = 0;
            while start < strip.columns.len() {
                let mut end = start+1;
                while end < strip.columns.len() && shape(&strip.columns[end].1) == shape(&strip.columns[start].1) { end += 1; }
                let run = &strip.columns[start..end];
                if run.len() >= 2 {
                    let rows = run[0].1.roots.len()+usize::from(run[0].1.head.is_some())+usize::from(run[0].1.tail.is_some());
                    let columns = run.len();
                    let mut points = Vec::with_capacity(rows*columns);
                    let mut normals = Vec::with_capacity(rows*columns);
                    let poses = run.iter().map(|(t,_)| self.motion.at(*t)).collect::<Result<Vec<_>,_>>()?;
                    for r in 0..rows {
                        for ((_,strand),pose) in run.iter().zip(&poses) {
                            let (_,p,n) = strand.head.iter().chain(&strand.roots).chain(&strand.tail).nth(r).copied().unwrap();
                            points.push(pose.point(p)); normals.push(pose.vector(n));
                        }
                    }
                    if rows >= 2 {
                        sheets.push(SweepSheet {points,normals,times:run.iter().map(|(t,_)| *t).collect(),rows,columns,closed_rows:false});
                    }
                }
                start = end;
            }
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
                    run.get_or_insert_with(|| Characteristic {points:vec![],normals:vec![],closed:false});
                    let r = run.as_mut().unwrap(); r.points.push(*p); r.normals.push(*n);
                }
            }
            if let Some(r) = run { if r.points.len() > 1 { trimmed.push(r); } }
        }
        Ok(trimmed)
    }
}

fn angle_gap(a: f64,b: f64) -> f64 { let d = (a-b).rem_euclid(1.); d.min(1.-d) }

/// Chain open pieces whose ends coincide; a piece that meets itself closes.
fn chain(pieces: Vec<Characteristic>,tolerance: f64) -> Vec<Characteristic> {
    // A piece already closed (a stationary ring) is whole; other pieces may
    // end on it without joining it.
    let (mut result,mut pieces): (Vec<_>,Vec<_>) = pieces.into_iter().partition(|p| p.closed);
    while let Some(mut current) = pieces.pop() {
        loop {
            let tail = *current.points.last().unwrap();
            let head = current.points[0];
            if current.points.len() > 2 && distance(tail,head) < tolerance {
                current.points.pop(); current.normals.pop(); current.closed = true; break;
            }
            let mut joined = false;
            for i in 0..pieces.len() {
                let (first,last) = (pieces[i].points[0],*pieces[i].points.last().unwrap());
                if distance(first,tail) < tolerance {
                    let piece = pieces.remove(i);
                    current.points.extend(piece.points.into_iter().skip(1)); current.normals.extend(piece.normals.into_iter().skip(1));
                    joined = true; break;
                }
                if distance(last,tail) < tolerance {
                    let piece = pieces.remove(i);
                    current.points.extend(piece.points.into_iter().rev().skip(1)); current.normals.extend(piece.normals.into_iter().rev().skip(1));
                    joined = true; break;
                }
                if distance(last,head) < tolerance {
                    let mut piece = pieces.remove(i);
                    piece.points.extend(current.points.into_iter().skip(1)); piece.normals.extend(current.normals.into_iter().skip(1));
                    current = piece; joined = true; break;
                }
                if distance(first,head) < tolerance {
                    let piece = pieces.remove(i);
                    let mut points: Vec<V3> = piece.points.into_iter().rev().collect();
                    let mut normals: Vec<V3> = piece.normals.into_iter().rev().collect();
                    points.extend(current.points.into_iter().skip(1)); normals.extend(current.normals.into_iter().skip(1));
                    current = Characteristic {points,normals,closed:false}; joined = true; break;
                }
            }
            if !joined { break; }
        }
        result.push(current);
    }
    result
}
