//! Local contact candidates contributed by a moving sharp edge.
use super::*;

/// A regular boundary edge point and its incident outward material normals.
/// Dihedral is signed: negative for convex, positive for concave, zero for smooth.
/// The tangent follows either direction along the edge; its magnitude is ignored.
#[derive(Clone,Copy,Debug)]
pub struct EdgePoint {
    pub position: V3,
    pub tangent: V3,
    pub normals: [V3;2],
    pub dihedral: f64,
}

/// A convex edge contributes a regular candidate where its outward normal cone
/// contains a normal perpendicular to the velocity. Smooth and concave edges do
/// not contribute regular sharp-edge faces. On a convex edge, tangent velocity
/// returns an error. Collapsed cones and inconsistent incidence also fail.
///
/// This is a local, interior-time test. Other motion times may cover a candidate;
/// global trimming, endpoint caps and singular cases are separate operations.
/// Angular tolerance bounds incidence/conditioning checks, and velocity tolerance
/// bounds the returned normal-velocity residual in model length per parameter.
pub fn edge_contact(edge: EdgePoint,motion: Motion,angular_tolerance: f64,
    velocity_tolerance: f64) -> Result<Option<Contact>,Error> {
    check_tolerance(angular_tolerance)?; check_tolerance(velocity_tolerance)?;
    if angular_tolerance == 0. || angular_tolerance > 0.1 || velocity_tolerance == 0. {
        return Err(Error::InvalidOptions);
    }
    if !edge.position.iter().chain(&edge.tangent).chain(edge.normals.iter().flatten())
        .all(|v| v.is_finite()) || !edge.dihedral.is_finite() || !motion.is_finite() {
        return Err(Error::NonFinite);
    }
    if edge.dihedral.abs() > std::f64::consts::PI { return Err(Error::InvalidOptions); }
    let tangent = normalized(edge.tangent).ok_or(Error::Degenerate)?;
    let normals = [normalized(edge.normals[0]).ok_or(Error::Degenerate)?,
        normalized(edge.normals[1]).ok_or(Error::Degenerate)?];
    let angle = length(cross(normals[0],normals[1])).atan2(dot(normals[0],normals[1]));
    if normals.iter().any(|n| dot(*n,tangent).abs() > angular_tolerance)
        || (angle-edge.dihedral.abs()).abs() > angular_tolerance {
        return Err(Error::Degenerate);
    }
    if std::f64::consts::PI-angle <= angular_tolerance {
        return Err(Error::Degenerate);
    }
    if angle <= angular_tolerance || edge.dihedral >= 0. { return Ok(None); }
    let tangent = motion.vector(tangent);
    let normals = normals.map(|n| motion.vector(n));
    let velocity = motion.velocity(edge.position);
    let position = motion.point(edge.position);
    if !velocity.iter().chain(&position).all(|v| v.is_finite()) { return Err(Error::NonFinite); }
    let values = normals.map(|n| dot(n,velocity));
    if !values.iter().all(|v| v.is_finite()) { return Err(Error::NonFinite); }
    let [a,b] = values;
    if (a > velocity_tolerance && b > velocity_tolerance)
        || (a < -velocity_tolerance && b < -velocity_tolerance) { return Ok(None); }
    if (a.abs() <= velocity_tolerance && b.abs() <= velocity_tolerance)
        || length(cross(tangent,normalized(velocity).ok_or(Error::Degenerate)?)) <= angular_tolerance {
        return Err(Error::Degenerate);
    }
    let normal = if a.abs() <= velocity_tolerance { normals[0] }
        else if b.abs() <= velocity_tolerance { normals[1] }
        else {
            let scale = a.abs().max(b.abs());
            normalized(add(scaled(normals[0],b.abs()/scale),scaled(normals[1],a.abs()/scale)))
                .ok_or(Error::Degenerate)?
        };
    let normal_velocity = dot(normal,velocity);
    if !normal_velocity.is_finite() { return Err(Error::NonFinite); }
    if normal_velocity.abs() > velocity_tolerance || dot(normal,tangent).abs() > angular_tolerance {
        return Err(Error::NotConverged);
    }
    Ok(Some(Contact {position,normal,velocity,normal_velocity}))
}
