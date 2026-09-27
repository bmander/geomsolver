//! Whole-domain bounds for nominal circular-crown branches and surface regularity.
//! Source-solve error transfer and global interference remain separate.
use super::*;
use gcs_core::interval::{Error,Interval as I};

fn scalar(x: f64) -> I { I::point(x).unwrap() }
fn range(x: [f64;2]) -> I { I::new(x[0],x[1]).unwrap() }

#[derive(Clone,Debug)]
struct Cell {
    u: [f64;2],rho: [f64;2],profile: [[f64;2];6],area_factor: [f64;2],margins: [f64;7],
}

fn profile_bounds(pair: &Pair,patch: &RevolvedSurface,u: [f64;2]) -> Result<[I;6],Error> {
    let [p,d,dd] = patch.generating_profile_jet_bounds(range(u))?;
    let [cx,cy,_] = pair.offset.map(scalar);
    let dx = p[0].sub(cx)?; let dy = p[1].sub(cy)?;
    let r = dx.square()?.add(dy.square()?)?.sqrt()?;
    let h = p[2];
    let dr = dx.mul(d[0])?.add(dy.mul(d[1])?)?.div(r)?;
    let dh = d[2];
    let ddr = d[0].square()?.add(d[1].square()?)?.add(dx.mul(dd[0])?)?
        .add(dy.mul(dd[1])?)?.sub(dr.square()?)?.div(r)?;
    Ok([r,h,dr,dh,ddr,dd[2]])
}

// The generated tangents, pulled back by their common rigid rotation, are
// S_u + V*t_u and S_theta + V*t_theta. On the contact locus V is tangent to S.
// Writing V = alpha*S_u + beta*S_theta gives the oriented area factor
// J = 1 + alpha*t_u + beta*t_theta (the two-by-two determinant lemma).
fn surface_factor(profile: [I;6],center: [I;2],theta: [I;2],cot: I) -> Result<I,Error> {
    let [r,h,dr,dh,ddr,ddh] = profile;
    let [cx,cy] = center; let [ct,st] = theta;
    let d = dr.mul(r)?.add(h.mul(dh)?)?;
    let a = dr.mul(cx)?.add(d.mul(ct)?)?.neg();
    let b = dr.mul(cy)?.add(d.mul(st)?)?.neg();
    let norm2 = a.square()?.add(b.square()?)?;
    let sign = if a.bounds()[0] > 0. { 1. }
        else if a.bounds()[1] < 0. { -1. } else { return Err(Error::DivisionByZero); };
    let g = cx.mul(ct)?.add(cy.mul(st)?)?;
    let k = cy.mul(ct)?.sub(cx.mul(st)?)?;
    let curvature = dr.mul(ddh)?.sub(dh.mul(ddr)?)?;
    let e = dr.mul(dr.square()?.add(dh.square()?)?)?.add(h.mul(curvature)?)?;
    let numerator = e.mul(k.square()?)?.add(d.div(r)?.mul(dr.mul(g)?.add(d)?.square()?)?)?;
    // Simplification of alpha*t_u + beta*t_theta on A*sin(t)+B*cos(t)=0.
    // It avoids separately bounding terms which cancel identically, and remains
    // valid when h' = 0 at a fillet/root endpoint.
    I::ONE.add(cot.mul(scalar(sign))?.mul(h)?.mul(numerator)?
        .div(norm2.mul(norm2.sqrt()?)?)?)
}

fn cell_bounds(pair: &Pair,member: usize,patch: &RevolvedSurface,u: [f64;2],rho: [f64;2])
    -> Result<Cell,Error> {
    let profile = profile_bounds(pair,patch,u)?;
    let [r,h,dr,dh,_,_] = profile;
    let [cx,cy,_] = pair.offset.map(scalar);
    let c2 = cx.square()?.add(cy.square()?)?;
    let c = c2.sqrt()?;
    let q = range(rho).square()?.sub(h.square()?)?.sub(c2)?.sub(r.square()?)?
        .div(scalar(2.).mul(c)?.mul(r)?)?;
    let discriminant = I::ONE.sub(q.square()?)?;
    let root = discriminant.sqrt()?;
    // cos/sin(arg(center)-acos(q)) without inverse trigonometry or branch cuts.
    let ct = cx.mul(q)?.add(cy.mul(root)?)?.div(c)?;
    let st = cy.mul(q)?.sub(cx.mul(root)?)?.div(c)?;
    let other_st = cy.mul(q)?.add(cx.mul(root)?)?.div(c)?;
    let px = cx.add(r.mul(ct)?)?; let py = cy.add(r.mul(st)?)?;
    // An unnormalized meridian normal (dh*cos(theta),dh*sin(theta),-dr).
    // Scaling the normal changes neither the contact equation nor its roll root.
    let a = dr.neg().mul(px)?.sub(h.mul(dh)?.mul(ct)?)?;
    let b = dr.neg().mul(py)?.sub(h.mul(dh)?.mul(st)?)?;
    let ratio = b.neg().div(a)?;
    let roll = pair.domain(patch)[2];
    let tan = |x| -> Result<I,Error> { let (s,c) = scalar(x).sin_cos()?; s.div(c) };
    let [low,high] = ratio.bounds();
    let [al,ah] = a.bounds();
    let cot = scalar(if member == 0 { 1. } else { -1. })
        .mul(scalar(pair.teeth[1-member]))?.div(scalar(pair.teeth[member]))?;
    let [jl,jh] = surface_factor(profile,[cx,cy],[ct,st],cot)?.bounds();
    Ok(Cell {u,rho,profile:profile.map(I::bounds),area_factor:[jl,jh],margins:[
        discriminant.bounds()[0],
        ct.bounds()[0],
        (-st.bounds()[1]).min(other_st.bounds()[0]),
        if al > 0. { al } else { -ah },
        dr.square()?.add(dh.square()?)?.bounds()[0],
        (low-tan(roll[0])?.bounds()[1]).next_down()
            .min((tan(roll[1])?.bounds()[0]-high).next_down()),
        if jl > 0. { jl } else { -jh },
    ]})
}

fn cover(pair: &Pair,member: usize,patch: &RevolvedSurface,rho: [f64;2]) -> Result<Vec<Cell>,String> {
    let domain = pair.domain(patch);
    if domain[0] != [0.,1.] || domain[1][1]-domain[1][0] != 0.5 {
        return Err("the circular-crown proof needs the declared full meridian and source semicircle".into());
    }
    let mut pending = vec![([0.,1.],rho,0)];
    let mut accepted = vec![];
    while let Some((u,rho,depth)) = pending.pop() {
        match cell_bounds(pair,member,patch,u,rho) {
            Ok(cell) if cell.margins.iter().all(|x| x.is_finite() && *x > 0.) => {
                accepted.push(cell);
                continue;
            }
            bound if depth == 20 => {
                return Err(format!("{}: inconclusive cell u={u:?}, rho={rho:?}: {bound:?}",patch.name));
            }
            _ => {}
        }
        // Split the widest normalized dimension, always retaining both children.
        if u[1]-u[0] >= (rho[1]-rho[0])/(0.2*pair.rm) {
            let mid = (u[0]+u[1])/2.;
            pending.push(([mid,u[1]],rho,depth+1));
            pending.push(([u[0],mid],rho,depth+1));
        } else {
            let mid = (rho[0]+rho[1])/2.;
            pending.push((u,[mid,rho[1]],depth+1));
            pending.push((u,[rho[0],mid],depth+1));
        }
    }
    Ok(accepted)
}

#[test]
fn whole_crown_domains_have_transverse_sections_continuous_roll_and_regular_surfaces() {
    let mut report = Vec::new();
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            for member in 0..2 {
                for side in 0..2 {
                    let name = if member == side { "outer" } else { "inner" };
                    for name in [name.to_string(),format!("{name}_round")] {
                        let patch = pair.patch(member,side,&name);
                        let rho = [0.9*pair.rm,1.1*pair.rm];
                        let cells = cover(&pair,member,&patch,rho).unwrap();
                        let area: f64 = cells.iter().map(|c| (c.u[1]-c.u[0])*(c.rho[1]-c.rho[0])).sum();
                        assert!((area-(rho[1]-rho[0])).abs() < module*1e-10);
                        let worst: [f64;7] = std::array::from_fn(|k| cells.iter()
                            .map(|c| c.margins[k]).fold(f64::INFINITY,f64::min));
                        eprintln!("{teeth:?} m={module} {}: {} cells, margins {worst:?}",patch.name,cells.len());
                        report.push(format!("{{\"teeth\":{teeth:?},\"member\":{member},\"module\":{module},\"patch\":\"{}\",\"center\":{:?},\"roll\":{:?},\"domain\":[[0,1],{rho:?}],\"cells\":[{}]}}",
                            patch.name,&pair.offset[..2],pair.domain(&patch)[2],
                            cells.iter().map(|c| format!("{{\"u\":{:?},\"rho\":{:?},\"profile\":{:?},\"area_factor\":{:?},\"margins\":{:?}}}",
                                c.u,c.rho,c.profile,c.area_factor,c.margins))
                                .collect::<Vec<_>>().join(",")));
                    }
                }
            }
        }
    }
    if let Some(path) = std::env::var_os("SOLVENT_BRANCH_OUTPUT") {
        std::fs::write(path,format!("[{}]\n",report.join(",\n"))).unwrap();
    }
}

#[test]
fn a_contact_branch_certificate_is_refused_when_a_sphere_misses_the_crown() {
    let pair = Pair::read([24,48],2.);
    let patch = pair.patch(0,0,"outer");
    assert!(cover(&pair,0,&patch,[0.01*pair.rm,0.02*pair.rm]).is_err());
}

#[test]
fn area_factor_agrees_with_differentiating_the_full_generated_surface_map() {
    let pair = Pair::read([24,48],2.);
    let cross = |a: [f64;3],b: [f64;3]| [a[1]*b[2]-a[2]*b[1],
        a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
    let dot = |a: [f64;3],b: [f64;3]| (0..3).map(|i| a[i]*b[i]).sum::<f64>();
    for member in 0..2 {
        let sign = if member == 0 { 1. } else { -1. };
        for side in 0..2 {
            let name = if member == side { "outer" } else { "inner" };
            for name in [name.to_string(),format!("{name}_round")] {
                let patch = pair.patch(member,side,&name);
                // Independently compose the crown/body rotations and differentiate
                // the resulting positions. No factor or roll-derivative formula is
                // used in these finite differences.
                let at = |u: f64,theta: f64| {
                    let s = patch.at(u,pair.chart_at(&patch,theta)).unwrap();
                    let n = cross(s.du,s.dv);
                    let p = s.position;
                    let a = n[2]*p[0]-p[2]*n[0];
                    let b = n[2]*p[1]-p[2]*n[1];
                    let t = ((-b).atan2(a)+FRAC_PI_2).rem_euclid(PI)-FRAC_PI_2;
                    let motion = rotate(2,t,0.).then(pair.body(member,t).inverse());
                    (p,motion.point(p),motion)
                };
                for u in [0.1,0.5,0.9] {
                    for face in [0.9,1.,1.1] {
                        let rho = face*pair.rm;
                        let v = pair.analytic(member,&patch,u,rho).parameters[1];
                        let theta = pair.azimuth_at(&patch,v);
                        let factor = surface_factor(profile_bounds(&pair,&patch,[u,u]).unwrap(),
                            [scalar(pair.offset[0]),scalar(pair.offset[1])],
                            [scalar(theta.cos()),scalar(theta.sin())],
                            scalar(sign).mul(scalar(pair.teeth[1-member])).unwrap()
                                .div(scalar(pair.teeth[member])).unwrap()).unwrap();
                        let [lo,hi] = factor.bounds();
                        assert!(hi-lo < 1e-8);
                        let expected = (lo+hi)/2.;
                        let difference = |step| {
                            let along_u = [at(u-step,theta),at(u+step,theta)];
                            let along_t = [at(u,theta-step),at(u,theta+step)];
                            let derivative = |p: [[f64;3];2]| std::array::from_fn(|i| (p[1][i]-p[0][i])/(2.*step));
                            let raw = cross(derivative(along_u.map(|p| p.0)),derivative(along_t.map(|p| p.0)));
                            let generated = cross(derivative(along_u.map(|p| p.1)),derivative(along_t.map(|p| p.1)));
                            let pulled_back = at(u,theta).2.inverse().vector(generated);
                            dot(pulled_back,raw)/dot(raw,raw)
                        };
                        let coarse = (difference(1e-4)-expected).abs();
                        let fine = (difference(5e-5)-expected).abs();
                        assert!(fine < 2e-5 && fine <= 0.4*coarse+2e-7,
                            "{}, u={u}, rho={rho}, J={expected}, errors {coarse}, {fine}",patch.name);
                    }
                }
            }
        }
    }
}

#[test]
fn a_regular_contact_branch_can_have_a_singular_generated_surface() {
    // The independently diagnosed 12:36 deep-section cusp (crown.rs). Its
    // contact equation stays regular while the generated oriented area reverses.
    let module = 0.2;
    let rm = module*12f64.hypot(36.)/2.;
    let rc = 0.8*rm;
    let theta = (35f64-90.).to_radians();
    let center = [rm-rc*theta.cos(),-rc*theta.sin()].map(scalar);
    let rho = (0.85*rm).hypot(0.45*module);
    let slope = scalar(-20f64.to_radians().tan());
    let bound = |height: I| {
        let r = scalar(rc).add(height.mul(slope).unwrap()).unwrap();
        let c2 = center[0].square().unwrap().add(center[1].square().unwrap()).unwrap();
        let c = c2.sqrt().unwrap();
        let q = scalar(rho).square().unwrap().sub(height.square().unwrap()).unwrap()
            .sub(c2).unwrap().sub(r.square().unwrap()).unwrap()
            .div(scalar(2.).mul(c).unwrap().mul(r).unwrap()).unwrap();
        let disc = I::ONE.sub(q.square().unwrap()).unwrap();
        assert!(disc.bounds()[0] > 0.);
        let root = disc.sqrt().unwrap();
        let ct = center[0].mul(q).unwrap().add(center[1].mul(root).unwrap()).unwrap().div(c).unwrap();
        let st = center[1].mul(q).unwrap().sub(center[0].mul(root).unwrap()).unwrap().div(c).unwrap();
        surface_factor([r,height,slope,I::ONE,I::ZERO,I::ZERO],center,[ct,st],scalar(3.)).unwrap()
    };
    let before = bound(scalar(0.13));
    let after = bound(scalar(0.14));
    assert!(before.bounds()[0] > 0. && after.bounds()[1] < 0.,"{before:?} {after:?}");
    assert!(bound(range([0.13,0.14])).contains(0.));
}
