//! Candidate shape policy: retain ordinary quality unless discovered incident
//! branches require an acute corner. This is not a source-geometry certificate.
use super::*;

pub(super) fn acceptable(v:[&Vertex;3]) -> bool {
    let [a,b,c] = v.map(|v| v.p);
    let normal = cross(sub(b,a),sub(c,a)); let area = length(normal);
    if area <= 1e-12 { return false; }
    let longest = length(sub(b,a)).max(length(sub(c,b))).max(length(sub(a,c)));
    if area > 0.12*longest*longest { return true; }
    let normal = mul(normal,1./area);
    let mut forced = false;
    for k in 0..3 {
        let a = v[k]; let b = v[(k+1)%3]; let c = v[(k+2)%3];
        let u = sub(b.p,a.p); let w = sub(c.p,a.p);
        // Every small acute angle needs its own geometric reason. One real
        // corner must not excuse a second sliver caused by unequal edge steps.
        if dot(u,w) > 0. && length(cross(u,w)) <= 0.12*length(u)*length(w) {
            if !intrinsic_corner(a,b,c,normal) { return false; }
            forced = true;
        }
    }
    forced
}

fn intrinsic_corner(a:&Vertex,b:&Vertex,c:&Vertex,face:V) -> bool {
    if a.branches.len() < 3 || !a.branches.iter().any(|&n| dot(n,face) > 1.-1e-8) {
        return false;
    }
    let incident = |end:&Vertex| -> Vec<V> {
        if !end.branches.iter().any(|&n| dot(n,face) > 1.-1e-8) { return vec![]; }
        let edge = unit(sub(end.p,a.p));
        a.branches.iter().copied().filter(|&n|
            length(cross(n,face)) > 1e-5 && dot(n,edge).abs() < 1e-5 &&
            end.branches.iter().any(|&m| dot(n,m) > 1.-1e-8)).collect()
    };
    let left = incident(b); let right = incident(c);
    // Both edges follow distinct discovered crease directions on the same
    // incident face. No supplied model plane or feature curve enters here.
    left.iter().any(|&n| right.iter().any(|&m| length(cross(n,m)) > 1e-5))
}
