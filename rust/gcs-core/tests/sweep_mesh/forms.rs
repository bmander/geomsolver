//! The closed forms the cases are judged against, written from geometry and
//! calling nothing of the construction: a plane region's swept area by ring
//! quadrature, and the volume each closed case must come to. Sampled
//! membership is a weaker tier and lives with the cases that need it, since
//! it reads the motion through `Family::at`; nothing here reads anything.

/// The area a plane region sweeps turned by `alpha` either way about the
/// origin, by ring quadrature: on each circle about the pivot (out to
/// `r_max`) the region's arcs, grown by the turn and their union measured,
/// at a resolution of `steps` around the circle.
pub(super) fn turned_area(member: &dyn Fn(f64,f64) -> bool,r_max: f64,alpha: f64) -> f64 {
    let (rings,steps) = (1000,3600);
    let reach = (alpha/2./(std::f64::consts::TAU/steps as f64)).round() as usize;
    let mut area = 0.;
    for i in 0..rings {
        let r = r_max*(i as f64+0.5)/rings as f64;
        let on: Vec<bool> = (0..steps).map(|k| { let t = std::f64::consts::TAU*k as f64/steps as f64; member(r*t.cos(),r*t.sin()) }).collect();
        // grown by the turn: covered where some point of the region is
        // within reach, read off a running count round the circle
        let mut prefix = vec![0usize;3*steps+1];
        for k in 0..3*steps { prefix[k+1] = prefix[k]+on[k%steps] as usize; }
        let covered = (0..steps).filter(|&k| prefix[steps+k+reach+1] > prefix[steps+k-reach]).count();
        area += covered as f64/steps as f64*std::f64::consts::TAU*r*r_max/rings as f64;
    }
    area
}

/// The turning prism: the section (3, -0.8), (4.5, 0), (3, 0.8) (in x and z)
/// extruded 3 along y, turned 100° about the vertical through (2.5, 0). At
/// each height its section is a rectangle, swept by ring quadrature.
pub(super) fn turning_prism_volume() -> f64 {
    let alpha = 100_f64.to_radians();
    let layers = 100;
    (0..layers).map(|j| {
        let z = -0.8+1.6*(j as f64+0.5)/layers as f64;
        let x_far = 4.5-1.875*z.abs();
        turned_area(&|x,y| x >= 0.5 && x <= x_far-2.5 && y.abs() <= 1.5,((x_far-2.5).powi(2)+2.25).sqrt(),alpha)*1.6/layers as f64
    }).sum()
}

/// The turned box: the 2 x 3 x 2 box turned 30° about the axis through the
/// centre of its x = 4 face along x, which is a 2 x 3 rectangle turned about
/// its own centre (the corners' arcs cross the faces and the faces turn
/// inner), times its 2 of depth.
pub(super) fn turned_box_volume() -> f64 {
    2.*turned_area(&|y,z| y.abs() <= 1. && z.abs() <= 1.5,(1_f64+2.25).sqrt(),30_f64.to_radians())
}

/// The turned lens: two unit spheres 0.8 apart, turned ±60° about the
/// spindle. Every circle about the spindle meets the lens in one arc: at
/// height z it spans r from 3.8 - rho to 3 + rho (rho the spheres' section
/// radius), so the sweep adds alpha (r_max^2 - r_min^2) / 2 to the lens's
/// own section.
pub(super) fn turned_lens_volume() -> f64 {
    let alpha = 120_f64.to_radians();
    let (h,layers) = (0.6_f64*(2.-0.6)/1_f64,2000);
    let h = h.sqrt(); // the crease's radius: sqrt(1 - 0.4²)
    (0..layers).map(|j| {
        let z = -h+2.*h*(j as f64+0.5)/layers as f64;
        let rho = (1.-z*z).sqrt();
        let (r_min,r_max) = (3.8-rho,3.+rho);
        // the section: two circular segments of the disks of radius rho, cut 0.4 from their centres
        let segment = |d: f64| rho*rho*(d/rho).acos()-d*(rho*rho-d*d).sqrt();
        (2.*segment(0.4)+alpha*(r_max*r_max-r_min*r_min)/2.)*2.*h/layers as f64
    }).sum()
}
