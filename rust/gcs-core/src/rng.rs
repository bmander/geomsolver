//! A seeded PRNG.  Determinism is a hard requirement of the program — same sketch plus same edit
//! must give the same result — so every random draw (witness jitter, generic poses for merge
//! decisions, homotopy's gamma trick) comes from a seeded stream here.

pub struct Rng {
    s: u32,
    spare: Option<f64>,
}

impl Rng {
    pub fn new(seed: u32) -> Rng {
        Rng { s: if seed == 0 { 0x9e37_79b9 } else { seed }, spare: None }
    }

    /// mulberry32.  Named `next` because that is what it is; it is not an `Iterator` — a stream of
    /// f64 with no end has nothing to gain from one.
    #[allow(clippy::should_implement_trait)]
    pub fn next(&mut self) -> f64 {
        self.s = self.s.wrapping_add(0x6d2b_79f5);
        let mut t = self.s;
        t = (t ^ (t >> 15)).wrapping_mul(t | 1);
        t ^= t.wrapping_add((t ^ (t >> 7)).wrapping_mul(t | 61));
        ((t ^ (t >> 14)) as f64) / 4_294_967_296.0
    }

    pub fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next()
    }

    /// Standard normal (Box–Muller; the second variate is kept for the next call).
    pub fn normal(&mut self, mu: f64, sigma: f64) -> f64 {
        if let Some(v) = self.spare.take() {
            return mu + sigma * v;
        }
        loop {
            let u = self.next() * 2.0 - 1.0;
            let v = self.next() * 2.0 - 1.0;
            let s = u * u + v * v;
            if s == 0.0 || s >= 1.0 {
                continue;
            }
            let f = (-2.0 * s.ln() / s).sqrt();
            self.spare = Some(v * f);
            return mu + sigma * u * f;
        }
    }

    pub fn int(&mut self, n: usize) -> usize {
        if n == 0 {
            return 0;
        }
        ((self.next() * n as f64).floor() as usize) % n
    }

    /// `k` distinct values from `0..n`, in draw order.
    pub fn sample(&mut self, n: usize, k: usize) -> Vec<usize> {
        let mut pool: Vec<usize> = (0..n).collect();
        let mut out = Vec::new();
        for _ in 0..k {
            if pool.is_empty() {
                break;
            }
            let i = self.int(pool.len());
            out.push(pool.remove(i));
        }
        out
    }

    pub fn choice<T: Copy>(&mut self, a: &[T]) -> T {
        a[self.int(a.len())]
    }
}
