//! Candidate boundary curves of a swept tool whose motion is a one-parameter
//! subgroup (a rotation, translation or screw): the tool-frame twist is then
//! constant, so the contact condition `n·v = 0` does not depend on the
//! parameter, the contact set is one fixed family of curves on the tool's
//! boundary, and the envelope is that set carried along the motion.
//!
//! The curves are traced on each revolved patch by the exact fixed-time roots,
//! trimmed where the tool's own static field hides a portion inside a Boolean
//! operand, joined to the sharp-edge segments contributed by the profile
//! vertices between adjacent patches, and chained by endpoint identity. This
//! is candidate generation only: caps, visibility and material remain the
//! arrangement's and the classifier's.
use super::{SweepContacts,RevolvedSurface};
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
    /// Outward normal sign of a patch's `du x dv`, decided by the tool's field
    /// just off the surface at a regular point.
    fn outward_sign(&self,patch: &RevolvedSurface,scale: f64) -> Result<f64,String> {
        let s = patch.at(0.5,0.25).map_err(|e| format!("{e:?}"))?;
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

    /// The fixed contact curves of a constant-twist motion, chained.
    /// `scale` is the tool's extent, for tolerances.
    pub fn characteristics(&self,tolerance: f64) -> Result<Vec<Characteristic>,String> {
        let pose = self.motion.at(self.roll[0])?;
        let scale = self.patches.iter().map(|p| p.at(0.5,0.).map(|s| norm(s.position)).unwrap_or(0.)).fold(1_f64,f64::max);
        let mut pieces: Vec<Characteristic> = Vec::new();
        let mut signs = Vec::with_capacity(self.patches.len());
        for patch in &self.patches { signs.push(self.outward_sign(patch,scale)?); }
        // Patch characteristics: roots in v at each meridian station, linked by
        // nearest angle between stations, folds joined where a pair of roots
        // appears or disappears.
        for (index,patch) in self.patches.iter().enumerate() {
            let sign = signs[index];
            let mut rows: Vec<Vec<(f64,V3,V3)>> = Vec::with_capacity(STATIONS+1);
            // With zero amplitude the ring's equation is a constant in v: the
            // whole ring is in contact where that constant vanishes. A sign
            // change between stations locates such a stationary ring; a face
            // whose constant vanishes everywhere is stationary and is covered by
            // the end caps and its sharp edges.
            let mut previous_constant: Option<(f64,f64)> = None;
            let ring = |u: f64| -> Result<Characteristic,String> {
                let mut points = Vec::new(); let mut normals = Vec::new();
                for j in 0..4*STATIONS {
                    let s = patch.at(u,j as f64/(4*STATIONS) as f64).map_err(|e| format!("{e:?}"))?;
                    let n = envelope::contact(s,Motion::identity()).map_err(|e| format!("{e:?}"))?.normal;
                    points.push(s.position); normals.push(n.map(|x| x*sign));
                }
                Ok(Characteristic {points,normals,closed:true})
            };
            for i in 0..=STATIONS {
                let u = i as f64/STATIONS as f64;
                let coefficients = match patch.contact_coefficients(u,pose) {
                    Ok(c) => Some(c),
                    Err(envelope::Error::Degenerate) => None,
                    Err(e) => return Err(format!("{e:?}")),
                };
                let mut row = Vec::new();
                if let Some((a,b,c)) = coefficients {
                    if a.hypot(b) <= tolerance {
                        if let Some((u0,c0)) = previous_constant {
                            // An isolated zero at a station is the ring itself; a
                            // zero shared with the previous station is a stationary face.
                            if c.abs() <= tolerance && c0.abs() > tolerance { pieces.push(ring(u)?); }
                            else if c0.abs() > tolerance && c.abs() > tolerance && (c0 < 0.) != (c < 0.) {
                                let (mut lo,mut hi,mut clo) = (u0,u,c0);
                                for _ in 0..60 {
                                    let mid = 0.5*(lo+hi);
                                    let (_,_,cm) = patch.contact_coefficients(mid,pose).map_err(|e| format!("{e:?}"))?;
                                    if (cm < 0.) == (clo < 0.) { lo = mid; clo = cm; } else { hi = mid; }
                                }
                                pieces.push(ring(0.5*(lo+hi))?);
                            }
                        }
                        previous_constant = Some((u,c));
                    } else {
                        previous_constant = None;
                        for root in patch.contacts(u,pose,tolerance).map_err(|e| format!("{e:?}"))? {
                            let s = patch.at(u,root.v).map_err(|e| format!("{e:?}"))?;
                            let n = envelope::contact(s,Motion::identity()).map_err(|e| format!("{e:?}"))?.normal;
                            row.push((root.v,s.position,n.map(|x| x*sign)));
                        }
                    }
                } else { previous_constant = None; }
                rows.push(row);
            }
            // Link station rows into strands by nearest angle; an unmatched root
            // ends or starts a strand. Strands from the two branches that meet at
            // a fold share an end position within the station spacing and are
            // chained below like any other pieces.
            let mut open: Vec<(f64,Characteristic)> = Vec::new();
            for row in rows {
                let mut next: Vec<(f64,Characteristic)> = Vec::new();
                let mut used = vec![false;row.len()];
                for (last_v,mut strand) in open {
                    let nearest = row.iter().enumerate().filter(|(j,_)| !used[*j])
                        .min_by(|a,b| angle_gap(a.1.0,last_v).total_cmp(&angle_gap(b.1.0,last_v)));
                    match nearest {
                        Some((j,&(v,p,n))) if angle_gap(v,last_v) < 0.02 => {
                            used[j] = true; strand.points.push(p); strand.normals.push(n); next.push((v,strand));
                        }
                        _ => pieces.push(strand),
                    }
                }
                for (j,&(v,p,n)) in row.iter().enumerate() {
                    if !used[j] { next.push((v,Characteristic {points:vec![p],normals:vec![n],closed:false})); }
                }
                open = next;
            }
            let mut strands: Vec<Characteristic> = open.into_iter().map(|(_,s)| s).collect();
            // A meridian end on the axis is a pole where every branch meets; a
            // strand reaching the adjacent station is extended to it so the
            // halves chain there.
            for (end,neighbour) in [(0.,1./STATIONS as f64),(1.,1.-1./STATIONS as f64)] {
                let Ok(s) = patch.at(end,0.) else { continue; };
                if norm(s.dv) > scale*1e-9 { continue; }
                let pole = s.position;
                // The tangent plane at a pole is perpendicular to the axis.
                let reference = envelope::contact(patch.at(neighbour,0.).map_err(|e| format!("{e:?}"))?,Motion::identity())
                    .map(|c| c.normal.map(|x| x*sign)).unwrap_or([0.;3]);
                let axis = patch.axis_direction();
                let n = if plane::dot(axis,reference) < 0. { axis.map(|x| -x) } else { axis };
                for strand in strands.iter_mut().chain(pieces.iter_mut()) {
                    let first = strand.points[0]; let last = *strand.points.last().unwrap();
                    let near = |p: V3| distance(p,pole) < scale*0.05 && distance(p,pole) > scale*1e-9;
                    if near(last) { strand.points.push(pole); strand.normals.push(n); }
                    else if near(first) { strand.points.insert(0,pole); strand.normals.insert(0,n); }
                }
            }
            pieces.extend(strands);
        }
        // Sharp profile vertices between adjacent patches contribute the edge
        // points whose normal cone straddles the velocity.
        for profile in &self.loops {
            let n = profile.len();
            for k in 0..n {
                let (Some(a),Some(b)) = (profile[k],profile[(k+1)%n]) else { continue; };
                let (pa,pb) = (&self.patches[a],&self.patches[b]);
                // The shared vertex: whichever ends of the two meridians coincide.
                let ends = |p: &RevolvedSurface| -> Result<[V3;2],String> {
                    Ok([p.at(0.,0.).map_err(|e| format!("{e:?}"))?.position,p.at(1.,0.).map_err(|e| format!("{e:?}"))?.position])
                };
                let (ea,eb) = (ends(pa)?,ends(pb)?);
                let Some((ua,ub)) = [(1.,0.),(1.,1.),(0.,0.),(0.,1.)].into_iter()
                    .find(|&(ua,ub)| distance(ea[ua as usize],eb[ub as usize]) < JOIN*scale) else { continue; };
                let mut points = Vec::new(); let mut normals = Vec::new();
                for i in 0..=4*STATIONS {
                    let v = i as f64/(4*STATIONS) as f64;
                    let sa = pa.at(ua,v).map_err(|e| format!("{e:?}"))?;
                    let sb = pb.at(ub,v).map_err(|e| format!("{e:?}"))?;
                    let na = envelope::contact(sa,Motion::identity()).map_err(|e| format!("{e:?}"))?.normal.map(|x| x*signs[a]);
                    let nb = envelope::contact(sb,Motion::identity()).map_err(|e| format!("{e:?}"))?.normal.map(|x| x*signs[b]);
                    let angle = plane::dot(na,nb).clamp(-1.,1.).acos();
                    if angle < 1e-6 { break; }
                    // Convex when the bisector leaves the tool.
                    let bisector = plane::unit(std::array::from_fn(|k| na[k]+nb[k])).unwrap_or(na);
                    let probe: V3 = std::array::from_fn(|k| sa.position[k]+bisector[k]*scale*1e-4);
                    let outside = self.source.bounds(probe.map(|x| Interval::point(x).unwrap())).map_err(|e| format!("{e:?}"))?.bounds()[0] > 0.;
                    if !outside { break; }
                    let tangent = sa.dv;
                    let edge = EdgePoint {position:sa.position,tangent,normals:[na,nb],dihedral:-angle};
                    match envelope::edge_contact(edge,pose,1e-6,tolerance) {
                        Ok(Some(contact)) => { points.push(sa.position); normals.push(pose.inverse().vector(contact.normal)); }
                        Ok(None) | Err(envelope::Error::Degenerate) => {
                            if !points.is_empty() { pieces.push(Characteristic {points:std::mem::take(&mut points),normals:std::mem::take(&mut normals),closed:false}); }
                        }
                        Err(e) => return Err(format!("{e:?}")),
                    }
                }
                if !points.is_empty() { pieces.push(Characteristic {points,normals,closed:false}); }
            }
        }
        // Trim hidden portions and drop empty pieces.
        let mut trimmed = Vec::new();
        for piece in pieces {
            let hidden: Vec<bool> = piece.points.iter().map(|p| self.hidden(*p,scale)).collect::<Result<_,_>>()?;
            if !hidden.iter().any(|h| *h) { trimmed.push(piece); continue; }
            let mut run: Option<Characteristic> = None;
            for (p,n) in piece.points.iter().zip(&piece.normals) {
                if self.hidden(*p,scale)? {
                    if let Some(r) = run.take() { if r.points.len() > 1 { trimmed.push(r); } }
                } else {
                    run.get_or_insert_with(|| Characteristic {points:vec![],normals:vec![],closed:false});
                    let r = run.as_mut().unwrap(); r.points.push(*p); r.normals.push(*n);
                }
            }
            if let Some(r) = run { if r.points.len() > 1 { trimmed.push(r); } }
        }
        Ok(chain(trimmed,JOIN*scale*10.))
    }
}

fn angle_gap(a: f64,b: f64) -> f64 { let d = (a-b).rem_euclid(1.); d.min(1.-d) }

/// Chain open pieces whose ends coincide; a piece that meets itself closes.
fn chain(mut pieces: Vec<Characteristic>,tolerance: f64) -> Vec<Characteristic> {
    let mut result = Vec::new();
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
