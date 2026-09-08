//! NASA CR-191009 (Bibel, Reddy, Kumar, 1994), POINTS.F, printed pp. 25–33;
//! settings pp. 4–6, independent published coordinates pp. 9–11.
//! https://ntrs.nasa.gov/api/citations/19940028394/downloads/19940028394.pdf
//!
//! These are four flank fixtures, not a general design rule for synthesizing machine
//! settings from tooth counts, nor a certification of the assembled reference pair.
use super::*;

#[derive(Clone, Copy)]
struct Flank {
    gear: bool,
    r: f64, q: f64, psi: f64, s: f64, mcw: f64, em: f64, lm: f64,
    deden: f64, mu: f64, addan: f64, cl: f64, rl: f64, fw: f64,
    seed: [f64;3],
    published: [f64;3],
}

fn fixtures() -> [Flank;4] {
    let pinion = Flank { gear: false, r: 2.96562137806, q: 63.9420304635,
        psi: 161.954330248, s: 2.94780202969, mcw: 0.30838512709,
        em: 0.154575896, lm: 0.0384999977874, deden: 1.56666666, mu: 18.4333333,
        addan: 3.8833334, cl: 0.03, rl: 3.191, fw: 1.,
        seed: [9.59703,126.83544,-0.85813],
        published: [0.7948185893337920,0.1518771876505158,2.565573009266192] };
    let gear = Flank { gear: true, r: 3.0325, q: 59.2342023, psi: 158., s: 2.85995004691,
        mcw: 0.9508646, em: 0., lm: 0., deden: 3.8833333333, mu: 71.5666666,
        addan: 1.5666666, cl: 0.0366, rl: 3.191, fw: 1.,
        seed: [8.12602,233.98994,-0.35063],
        published: [2.474433967420987,-0.3395653327372070,0.9856358798469023] };
    [pinion, Flank { r: 3.071306157, q: 53.9259945467, psi: 24.337423854,
        s: 2.80104946, mcw: 0.3220428536, em: -0.17426159493, lm: -0.0518138227,
        seed: [7.42534,124.43689,-11.38663],
        published: [0.7093463178422439,0.3893983454546772,2.565573009266191], ..pinion },
    gear, Flank { r: 2.9675, psi: 22., seed: [7.89156,234.95451,-12.3384],
        published: [2.487328996883535,-0.2265447761173172,0.9856358798469033], ..gear }]
        .map(|mut f| {
            f.q = f.q.to_radians(); f.psi = f.psi.to_radians();
            f.deden = f.deden.to_radians(); f.mu = f.mu.to_radians();
            f.addan = f.addan.to_radians();
            f.seed[1] = f.seed[1].to_radians(); f.seed[2] = f.seed[2].to_radians(); f
        })
}

impl Flank {
    fn surface(self, u: f64, theta: f64) -> SurfacePoint {
        let (s,c) = self.psi.sin_cos();
        let (st,ct) = theta.sin_cos();
        SurfacePoint { position: [self.r*c/s-u*c, u*s*st, u*s*ct],
            du: [-c,s*st,s*ct], dv: [0.,u*s*ct,-u*s*st] }
    }
    fn motion(self, phi: f64) -> Motion {
        let h = if self.gear { -1. } else { 1. };
        // TRANSF's five matrices, applied in their printed column-vector order.
        rotate(0,h*self.q,0.)
            .then(Motion::translation([0.,-h*self.s*self.q.sin(),self.s*self.q.cos()],
                [0.;3]).unwrap())
            .then(rotate(0,-h*phi,-h))
            .then(rotate(1,-self.deden,0.))
            .then(Motion::translation([-self.lm*self.deden.sin(),h*self.em,
                self.lm*self.deden.cos()], [0.;3]).unwrap())
            .then(rotate(1,self.mu,0.))
            .then(rotate(2,-h*phi/self.mcw,-h/self.mcw))
    }
    fn section(self, face: f64, height: f64) -> [f64;2] {
        let g = self.mu-self.deden;
        let root = (self.rl-self.fw/2.)*self.deden.cos();
        let zm = (root+face*self.fw)*g.cos()-self.cl*g.sin();
        let rm = zm*g.tan()+self.cl/g.cos();
        let depth = zm*(self.addan+self.deden).tan()/g.cos()-self.cl;
        [zm-height*depth*g.sin(),rm+height*depth*g.cos()]
    }
    // Independent scalar equation from POINTS.F, not used to produce the geometry.
    fn meshing(self, [u,theta,phi]: [f64;3]) -> f64 {
        let g = self.mu-self.deden;
        let h = if self.gear { -1. } else { 1. };
        let tau = theta-h*self.q+h*phi;
        let (s,c) = self.psi.sin_cos();
        let a = (u-self.r*c*c/s)*g.cos()*tau.sin();
        let b = self.s*(self.mcw-g.sin())*c*theta.sin();
        let c1 = self.s*g.cos()*s*(self.q-phi).sin();
        let d = self.em*(g.cos()*s+g.sin()*c*tau.cos());
        let e = self.lm*g.sin()*c*tau.sin();
        a+b-h*c1+h*d-e
    }
    fn options(self) -> IntersectionOptions {
        IntersectionOptions { bounds: [[self.seed[0]/2.,self.seed[0]*1.5],
            [self.seed[1]-1.,self.seed[1]+1.],[-1.,1.]],
            parameter_scale: [self.rl,1.,1.], residual_tolerance: [1e-11;3],
            max_iterations: 100 }
    }
    fn solve(self, face: f64, height: f64, seed: [f64;3]) -> envelope::Intersection {
        let [z,r] = self.section(face,height);
        envelope::intersect(|u,v| self.surface(u,v), |t| self.motion(t),
            |c| [c.position[2]-z, r-c.position[0].hypot(c.position[1])],
            seed,self.options()).unwrap()
    }
}

#[test]
fn all_four_spiral_bevel_flanks_agree_with_published_coordinates() {
    for f in fixtures() {
        let p = f.solve(0.,0.,f.seed);
        near(p.contact.position,f.published,1e-7);
        assert!(f.meshing(p.parameters).abs() < 1e-10, "{:?}",p.parameters);
    }
}

#[test]
fn four_flank_grids_satisfy_independent_meshing_equation_and_trims() {
    for f in fixtures() {
        let mut start = f.seed;
        for i in 0..8 {
            start = f.solve(i as f64/7.,0.,start).parameters;
            let mut seed = start;
            for j in 0..6 {
                let p = f.solve(i as f64/7.,j as f64/5.,seed);
                assert!(f.meshing(p.parameters).abs() < 1e-10);
                assert!(p.residuals.iter().all(|r| r.abs() < 1e-11));
                seed = p.parameters;
            }
        }
    }
}
