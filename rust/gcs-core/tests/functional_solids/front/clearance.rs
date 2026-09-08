//! Conservative separation of an encoded edge from an encoded triangle.
//! Uncertain interval signs retain the collision guard.
use super::*;
type Box3 = [I;3];

fn difference(a:Box3,b:Box3) -> Result<Box3,Error> {
    Ok([a[0].sub(b[0])?,a[1].sub(b[1])?,a[2].sub(b[2])?])
}
fn product(a:Box3,b:Box3) -> Result<Box3,Error> {
    Ok([a[1].mul(b[2])?.sub(a[2].mul(b[1])?)?,
        a[2].mul(b[0])?.sub(a[0].mul(b[2])?)?,
        a[0].mul(b[1])?.sub(a[1].mul(b[0])?)?])
}
fn scalar(a:Box3,b:Box3) -> Result<I,Error> {
    a[0].mul(b[0])?.add(a[1].mul(b[1])?)?.add(a[2].mul(b[2])?)
}

pub(super) fn separated(edge:[V;2],triangle:[V;3]) -> bool {
    separated_except_shared(edge,triangle,[false;2])
}

pub(super) fn separated_except_shared(edge:[V;2],triangle:[V;3],shared:[bool;2]) -> bool {
    if edge.iter().chain(&triangle).flatten().any(|x| !x.is_finite()) { return false; }
    let shared = std::array::from_fn(|k| shared[k] && triangle.contains(&edge[k]));
    enclosure(edge.map(point),triangle.map(point),shared).unwrap_or(false)
}

fn enclosure([p,q]:[Box3;2],triangle:[Box3;3],shared:[bool;2]) -> Result<bool,Error> {
    let [a,b,c] = triangle;
    let n = product(difference(b,a)?,difference(c,a)?)?;
    // Topologically shared vertices have exact zero plane distance. If the
    // other endpoint is strictly off the plane, that vertex is the entire
    // intersection; it must not block an otherwise legal front connection.
    let dp = if shared[0] { I::ZERO } else { scalar(n,difference(p,a)?)? };
    let dq = if shared[1] { I::ZERO } else { scalar(n,difference(q,a)?)? };
    if shared == [true;2] || (shared[0] && !dq.contains(0.)) || (shared[1] && !dp.contains(0.)) {
        return Ok(true);
    }
    if (dp.bounds()[0] > 0. && dq.bounds()[0] > 0.) ||
        (dp.bounds()[1] < 0. && dq.bounds()[1] < 0.) { return Ok(true); }
    let divisor = dp.sub(dq)?;
    if divisor.contains(0.) { return Ok(false); }
    let t = dp.div(divisor)?;
    let [lo,hi] = t.bounds();
    if lo > 1. || hi < 0. { return Ok(true); }
    let t = I::new(lo.max(0.),hi.min(1.))?;
    let d = difference(q,p)?;
    let x = [p[0].add(d[0].mul(t)?)?,p[1].add(d[1].mul(t)?)?,p[2].add(d[2].mul(t)?)?];
    for k in 0..3 {
        let a = triangle[k]; let b = triangle[(k+1)%3];
        if scalar(product(difference(b,a)?,difference(x,a)?)?,n)?.bounds()[1] < 0. {
            return Ok(true);
        }
    }
    Ok(false)
}
