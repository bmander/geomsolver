//! Independent closed-form characteristic for these circular crown references.
//! Used to check numerical envelopes and classify material without a triangle mesh.
use super::*;

impl Pair {
    /// Where a crown surface's chart starts about the crown's axis (the azimuth, about the
    /// crown centre, of its first meridian) and which way it turns: the source chart is the
    /// section's own, whichever view it is drawn in and whichever way its axis points.
    fn chart(&self,patch: &RevolvedSurface) -> (f64,f64,f64) {
        let v0 = patch.domain()[1][0];
        let meridian = patch.at(0.5,v0).unwrap();
        let (x,y) = (meridian.position[0]-self.offset[0],meridian.position[1]-self.offset[1]);
        (v0,y.atan2(x),(x*meridian.dv[1]-y*meridian.dv[0]).signum())
    }

    /// The chart's `v` at azimuth `theta` about the crown centre.
    pub(super) fn chart_at(&self,patch: &RevolvedSurface,theta: f64) -> f64 {
        let (v0,start,turn) = self.chart(patch);
        (v0+(turn*(theta-start)/TAU).rem_euclid(1.)).rem_euclid(1.)
    }

    /// The azimuth about the crown centre of the chart's `v`.
    pub(super) fn azimuth_at(&self,patch: &RevolvedSurface,v: f64) -> f64 {
        let (v0,start,turn) = self.chart(patch);
        start+turn*(v-v0)*TAU
    }

    pub(super) fn analytic(&self, member: usize, patch: &RevolvedSurface, u: f64, rho: f64)
        -> envelope::Intersection {
        let meridian = patch.at(u,patch.domain()[1][0]).unwrap();
        let [cx,cy,_] = self.offset;
        let r = (meridian.position[0]-cx).hypot(meridian.position[1]-cy);
        let h = meridian.position[2];
        let c = cx.hypot(cy);
        // A circle (fixed meridian parameter) intersected by a sphere about the apex.
        let cosine = (rho*rho-h*h-c*c-r*r)/(2.*c*r);
        assert!(cosine.abs() <= 1.,"reference circle misses spherical section");
        let theta = cy.atan2(cx)-cosine.acos();
        let v = self.chart_at(patch,theta);
        let s = patch.at(u,v).unwrap();
        let normal = envelope::contact(s,Motion::identity()).unwrap().normal;
        let p = s.position;
        // n · (x_axis × p) = 0 after crown rotation. The three relative angular
        // velocities are collinear with x_axis; no numerical envelope solve enters here.
        let a = normal[2]*p[0]-h*normal[0];
        let b = normal[2]*p[1]-h*normal[1];
        assert!(a.hypot(b) > 1e-10*self.module);
        let t = ((-b).atan2(a)+FRAC_PI_2).rem_euclid(PI)-FRAC_PI_2;
        let contact = envelope::contact(s,self.motion(member,t)).unwrap();
        envelope::Intersection {parameters:[u,v,t],contact,
            residuals:[contact.normal_velocity,0.,0.],iterations:0}
    }
}

#[test]
fn numerical_flanks_and_fillets_agree_with_closed_form_characteristics() {
    let pair = Pair::read([24,48],2.);
    for member in 0..2 {
        for side in 0..2 {
            let outer = member == side;
            let edge = if outer { "outer" } else { "inner" };
            for edge in [edge.to_string(),format!("{edge}_round")] {
                let patch = pair.patch(member,side,&edge);
                for face in [0.9,1.,1.1] {
                    for i in 0..=20 {
                        let u = i as f64/20.;
                        let rho = face*pair.rm;
                        let a = pair.analytic(member,&patch,u,rho);
                        let b = pair.at(member,&patch,u,rho);
                        near(a.parameters,b.parameters,1e-8);
                        near(a.contact.position,b.contact.position,1e-7);
                        near(a.contact.normal,b.contact.normal,1e-8);
                        assert!(a.contact.normal_velocity.abs() < 1e-9);
                    }
                }
            }
        }
    }
}
