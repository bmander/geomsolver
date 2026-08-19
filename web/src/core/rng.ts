/* A seeded PRNG.  Determinism is a hard requirement of the program — same sketch plus
 * same edit must give the same result — so every random draw (witness jitter, generic
 * poses for merge decisions, homotopy's gamma trick) comes from a seeded stream here,
 * never from Math.random. */

export class Rng {
  private s: number;

  constructor(seed = 0) {
    this.s = (seed >>> 0) || 0x9e3779b9;
  }

  /** mulberry32 */
  next(): number {
    this.s = (this.s + 0x6d2b79f5) >>> 0;
    let t = this.s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  uniform(lo = 0, hi = 1): number {
    return lo + (hi - lo) * this.next();
  }

  /** Standard normal (Box-Muller; the second variate is kept for the next call). */
  normal(mu = 0, sigma = 1): number {
    if (this.spare !== null) {
      const v = this.spare;
      this.spare = null;
      return mu + sigma * v;
    }
    let u = 0, v = 0, s = 0;
    do {
      u = this.next() * 2 - 1;
      v = this.next() * 2 - 1;
      s = u * u + v * v;
    } while (s === 0 || s >= 1);
    const f = Math.sqrt((-2 * Math.log(s)) / s);
    this.spare = v * f;
    return mu + sigma * u * f;
  }

  private spare: number | null = null;

  int(n: number): number {
    return Math.floor(this.next() * n) % n;
  }

  /** `k` distinct values from 0..n-1, in draw order. */
  sample(n: number, k: number): number[] {
    const pool = Array.from({ length: n }, (_, i) => i);
    const out: number[] = [];
    for (let i = 0; i < k && pool.length; i++) out.push(pool.splice(this.int(pool.length), 1)[0]);
    return out;
  }

  choice<T>(a: T[]): T {
    return a[this.int(a.length)];
  }
}
