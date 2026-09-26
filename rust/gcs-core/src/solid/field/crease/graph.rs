//! What the traced creases make together: where a trace walks onto another, a crease split at
//! a T, the corners their ends share, and the feature curves they come to.
use super::{Crease,CreaseOptions,CreaseSource,End,P,SYMMETRY_VERTICES,creases_under,seeds};
use crate::space::{distance,closest_on_segment,segment_distance};

/// Run after the ends are welded (`features`), so what is split in is the corner as it will stand.
/// Every crease end lying on another crease's middle — a T, where a crease meets one that runs on
/// through the point (the tip rim, handed on tangentially between two pieces of a cutter's profile,
/// where the crease between those pieces reaches it) — splits that other crease at the end's exact
/// point, which protection then takes as the corner the two share. Ends near another's end are the
/// weld's; one merely near another's end, past the weld, still splits it, beside that end.
fn junctions(creases: &mut Vec<Crease>,within: f64) {
    let mut i = 0;
    while i < creases.len() {
        if !creases[i].closed {
            for which in [0,1] {
                let at = if which == 0 { creases[i].points[0] } else { *creases[i].points.last().unwrap() };
                enum Act { Snap(P),Split(usize) }
                let mut act = None;
                for (k,c) in creases.iter().enumerate() {
                    if k == i { continue; }
                    let ends = [c.points[0],*c.points.last().unwrap()];
                    // already one corner, as the weld or an earlier split left it
                    if ends.contains(&at) { act = None; break; }
                    // near another's end, past the weld: the same corner found two ways, so this
                    // end is moved onto that one — split instead, each would split the other again
                    if let Some(&e) = ends.iter().find(|&&e| !c.closed && distance(e,at) <= 2.*within) { act = Some(Act::Snap(e)); break; }
                    if c.points.windows(2).any(|w| segment_distance(at,w[0],w[1]) < within) { act = Some(Act::Split(k)); break; }
                }
                match act {
                    Some(Act::Snap(e)) => {
                        let n = creases[i].points.len();
                        creases[i].points[if which == 0 { 0 } else { n-1 }] = e;
                    }
                    Some(Act::Split(k)) => split(creases,k,at),
                    None => {}
                }
            }
        }
        i += 1;
    }
}

/// The crease of `existing` that `q` lies within `within` of, and the point of it `q` is to end at:
/// the crease's own end where that is within two of `within`, and otherwise its nearest point.
pub(super) fn meets(existing: &[Crease],q: P,within: f64) -> Option<(usize,P)> {
    for (k,c) in existing.iter().enumerate() {
        let near = c.points.windows(2).map(|w| (w[0],w[1],segment_distance(q,w[0],w[1])))
            .fold(None,|best: Option<(P,P,f64)>,x| if best.is_none_or(|b| x.2 < b.2) { Some(x) } else { best });
        let Some((a,b,d)) = near else { continue };
        if d >= within { continue; }
        let ends = if c.closed { vec![] } else { vec![c.points[0],*c.points.last().unwrap()] };
        if let Some(&e) = ends.iter().find(|&&e| distance(e,q) <= 2.*within) { return Some((k,e)); }
        return Some((k,closest_on_segment(q,a,b)));
    }
    None
}

/// Crease `k` split at `at`, a point on it: `at` put in at its nearest segment, and the crease cut
/// in two there — or, closed, begun and ended there. Nothing is done where `at` is already an end.
pub(super) fn split(creases: &mut Vec<Crease>,k: usize,at: P) {
    let c = &creases[k];
    let n = c.points.len();
    if c.points[0] == at || c.points[n-1] == at { return; }
    let Some(j) = (0..n-1).min_by(|&x,&y| segment_distance(at,c.points[x],c.points[x+1])
        .total_cmp(&segment_distance(at,c.points[y],c.points[y+1]))) else { return };
    let mut points = c.points.clone();
    points.insert(j+1,at);
    if c.closed {
        // a loop begun and ended at the corner: still closed, the corner its one end
        let mut ring: Vec<P> = points[j+1..points.len()-1].to_vec();
        ring.extend_from_slice(&points[..=j]);
        ring.push(at);
        creases[k].points = ring;
    } else {
        let (head,tail) = (points[..=j+1].to_vec(),points[j+1..].to_vec());
        let second = Crease {points:tail,closed:false,operands:c.operands,ends:[End::Met(k),c.ends[1]]};
        creases[k].points = head;
        creases[k].ends[1] = End::Met(creases.len());
        creases.push(second);
    }
}

/// The feature curves of a field, for `delaunay::refine`'s protection, from a mesh made without
/// them: every crease its edges cross, traced, with the ends that meet at one corner made one
/// point, since a corner is a ball the curves share only where their ends are equal. Ends within
/// a fiftieth of a step are one corner: an end found where a crease stops being followed lands
/// within the bisection of the corner another found exactly, and two corners that close could
/// not be protected apart anyway.
pub fn features<S: CreaseSource + ?Sized>(field: &S,vertices: &[P],triangles: &[[u32;3]],options: &CreaseOptions) -> Vec<Vec<P>> {
    // checked at vertices of the surface, where an unlike blank would show
    let samples: Vec<P> = vertices.iter().step_by((vertices.len()/SYMMETRY_VERTICES).max(1)).copied().collect();
    let maps = field.symmetries(&samples);
    let mut found = creases_under(field,&seeds(field,vertices,triangles,options),options,&maps);
    let mut corners: Vec<P> = Vec::new();
    let mut weld = |p: P| -> P {
        match corners.iter().find(|&&c| distance(c,p) <= 0.02*options.step) {
            Some(&c) => c,
            None => { corners.push(p); p }
        }
    };
    for c in found.iter_mut().filter(|c| !c.closed) {
        let last = c.points.len()-1;
        c.points[0] = weld(c.points[0]);
        c.points[last] = weld(c.points[last]);
    }
    // the T-junctions last, at the corners as welded: split any earlier, the weld could move the
    // end away from the point split in, and the two would meet a few microns apart
    junctions(&mut found,0.25*options.step);
    found.into_iter().map(|c| c.points).collect()
}
