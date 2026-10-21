//! Where the boundary is: the domain's side at a point and at a tetrahedron's orthocentre, and
//! the crossing on a segment between two sides — bisected on the sign, or by safeguarded Newton
//! where the domain gives readings — with a facet's dual edge worked along its own line.
use super::*;
use crate::space::{lerp,orthocentre,norm};

impl Refiner<'_> {
    /// The domain's side at `p`: −1 inside, 1 outside (and everywhere past the bounding ball).
    pub(super) fn side(&mut self,p: P) -> i8 {
        if dist2(p,self.centre) > self.radius*self.radius { return 1; }
        // A rebuild asks again at the orthocentres and bisections it asked before.
        let key = p.map(f64::to_bits);
        if let Some(&s) = self.memo.get(&key) { return s; }
        self.report.queries += 1;
        let s = if self.domain.value(p) < 0. { -1 } else { 1 };
        self.memo.insert(key,s);
        s
    }

    pub(super) fn tet_side(&mut self,t: u32) -> i8 {
        if self.sign.len() <= t as usize { self.sign.resize(t as usize+1,0); }
        if self.sign[t as usize] == 0 {
            let (o,_) = self.tri.orthosphere(t);
            let before = self.report.queries;
            let s = if o.iter().all(|x| x.is_finite()) { self.side(o) } else { 1 };
            self.report.orthocentre_queries += self.report.queries-before;
            self.sign[t as usize] = s;
        }
        self.sign[t as usize]
    }

    /// The boundary crossing between `a` (on side `sa`) and `b` (on the other side).
    pub(super) fn crossing(&mut self,a: P,sa: i8,b: P) -> P {
        let key = [a[0].to_bits(),a[1].to_bits(),a[2].to_bits(),b[0].to_bits(),b[1].to_bits(),b[2].to_bits(),sa as u64];
        if let Some(&c) = self.crossings.get(&key) { return c; }
        self.report.crossings += 1;
        let c = if self.domain.readings() != Readings::Absent { self.newton_crossing(a,sa,b) } else { self.bisected(a,sa,b) };
        self.crossings.insert(key,c);
        c
    }

    fn bisected(&mut self,a: P,sa: i8,b: P) -> P {
        let before = self.report.queries;
        let tol = self.criteria.bisection;
        let (a,b) = crate::roots::bisect(a,b,|a,b| lerp(a,b,0.5),|a,b| dist2(a,b) > tol*tol,|m| self.side(m) == sa);
        self.report.bisection_queries += self.report.queries-before;
        lerp(a,b,0.5)
    }

    /// The crossing on the segment from `a` (on side `sa`) to `b` by Newton on the value along
    /// it, from whichever end reads nearer zero, with the bracket kept: a step that would leave
    /// it, or that does not halve the value, is a bisection instead, and each reading's sign
    /// moves one end. It stops where the Newton step or the bracket is within the bisection
    /// tolerance, so the point is as near the crossing as bisection's would be, in a handful of
    /// readings where bisection takes twenty sign queries. A reading at an end whose sign
    /// disagrees with the one the caller found (a reading is floating point, and a domain may
    /// work out its value and its side differently) falls back to bisection.
    fn newton_crossing(&mut self,a: P,sa: i8,b: P) -> P {
        // An end outside the bounding ball reads outside without asking; the segment is cut at
        // the ball, where the domain reads the same and a reading is worth making.
        let (a,b) = (self.clipped(a,b),self.clipped(b,a));
        let tol = self.criteria.bisection;
        let d = sub(b,a);
        let len = norm(d);
        if !(len > tol) { return lerp(a,b,0.5); }
        // The first reading of a crossing is cold; every later one continues from the last.
        let mut warm = false;
        let mut read = |r: &mut Self,s: f64| -> (f64,f64) {
            let p = lerp(a,b,s);
            if dist2(p,r.centre) > r.radius*r.radius { return (f64::INFINITY,0.); }
            r.report.queries += 1;
            r.report.readings += 1;
            let (v,g) = r.domain.reading(p,warm);
            warm = true;
            (v,dot(g,d))
        };
        // In `s` along the segment: `lo` on `a`'s side, `hi` on the other. Newton starts from `a`,
        // the one end read: `b`'s side is known, and its value would be a second whole reading.
        let (mut lo,mut hi) = (0.,1.);
        let (fa,da) = read(self,0.);
        let side = |v: f64| if v < 0. { -1 } else { 1 };
        if !fa.is_finite() || side(fa) != sa {
            self.report.fallbacks += 1;
            return self.bisected(a,sa,b);
        }
        self.report.newton += 1;
        let (mut s,mut f,mut df) = (0.,fa,da);
        for _ in 0..60 {
            if (hi-lo)*len <= tol { break; }
            let newton = if df != 0. && df.is_finite() { s-f/df } else { f64::NAN };
            let inside = newton > lo.min(hi) && newton < lo.max(hi);
            // Converged: the next Newton step is shorter than the tolerance.
            if inside && (newton-s).abs()*len <= 0.5*tol { s = newton; break; }
            let next = if inside { newton } else { 0.5*(lo+hi) };
            let (fn_,dn) = read(self,next);
            if !fn_.is_finite() { return self.bisected(a,sa,b); }
            // Not halving the value: the next step bisects, whatever Newton says.
            let slow = fn_.abs() > 0.5*f.abs();
            if side(fn_) == sa { lo = next } else { hi = next }
            (s,f,df) = (next,fn_,dn);
            if slow && inside { let (fm,dm) = read(self,0.5*(lo+hi)); let m = 0.5*(lo+hi);
                if !fm.is_finite() { return self.bisected(a,sa,b); }
                if side(fm) == sa { lo = m } else { hi = m }
                (s,f,df) = (m,fm,dm); }
        }
        let s = if s > lo.min(hi) && s < lo.max(hi) { s } else { 0.5*(lo+hi) };
        // The readings after the first may have continued the last one's search rather than
        // made their own (`Readings::Checked`): the bracket they closed is checked by `side`, the
        // query bisection trusts, a tolerance outside each end — within it a reading's sign and
        // `side`'s may both be right about a point that close to the boundary — and a bracket it
        // disowns is bisected from the start. The crossing is then within the tolerance of the
        // point returned, as bisection's is.
        let pad = tol/len;
        let (sl,sh) = ((lo.min(hi)-pad).max(0.),(lo.max(hi)+pad).min(1.));
        let (sl,sh) = if lo <= hi { (sl,sh) } else { (sh,sl) };
        if self.domain.readings() != Readings::Agreeing && ((sl > 0. && self.side(lerp(a,b,sl)) != sa) || (sh < 1. && self.side(lerp(a,b,sh)) == sa)) {
            self.report.fallbacks += 1;
            return self.bisected(a,sa,b);
        }
        lerp(a,b,s)
    }

    /// `p`, or where the segment from `q` to it leaves the bounding ball when `p` is outside it
    /// and `q` is not: a hair inside, so the reading is made.
    fn clipped(&self,p: P,q: P) -> P {
        let r2 = self.radius*self.radius;
        if dist2(p,self.centre) <= r2 || dist2(q,self.centre) > r2 { return p; }
        let (d,w) = (sub(p,q),sub(q,self.centre));
        let (a,b,c) = (dot(d,d),2.*dot(d,w),dot(w,w)-r2);
        let t = (-b+(b*b-4.*a*c).max(0.).sqrt())/(2.*a);
        lerp(q,p,(t*(1.-1e-12)).clamp(0.,1.))
    }

    /// Where the dual edge of facet `face`, from orthocentre `ot` (on side `st`) to `on`,
    /// crosses the boundary. Bisected along the facet's own dual line — the points of equal power
    /// to its three weighted vertices, worked from them — between the orthocentres' projections
    /// onto it: a nearly flat tetrahedron's orthocentre is far off and inaccurate, and bisected
    /// toward it directly the crossing leaves the line, is in conflict with neither tetrahedron,
    /// and inserted leaves the facet standing to be refined again at the same place.
    pub(super) fn dual_crossing(&mut self,face: [u32;3],ot: P,st: i8,on: P) -> P {
        let [a,b,c] = face.map(|v| self.tri.points()[v as usize]);
        let Some((c0,n)) = orthocentre(a.p,b.p,c.p,[a.w,b.w,c.w]) else { return self.crossing(ot,st,on); };
        let nn = dot(n,n);
        let along = |q: P| dot(sub(q,c0),n)/nn;
        let (tt,tn) = (along(ot),along(on));
        let (x,y) = ([0,1,2].map(|k| c0[k]+tt*n[k]),[0,1,2].map(|k| c0[k]+tn*n[k]));
        if !x.iter().chain(&y).all(|z| z.is_finite()) { return self.crossing(ot,st,on); }
        // Where a projection is its orthocentre to within the bisection, the side is known.
        let sn = -st;
        let tol = self.criteria.bisection*self.criteria.bisection;
        let sx = if dist2(x,ot) <= tol { st } else { self.side(x) };
        let sy = if dist2(y,on) <= tol { sn } else { self.side(y) };
        if sx == sy { return self.crossing(ot,st,on); }
        self.crossing(x,sx,y)
    }
}
