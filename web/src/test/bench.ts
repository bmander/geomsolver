/* Benchmark: solve, compile, plan replay and drag frame rates in the WebAssembly core.
 *
 *     npm run bench            (node)      or import and call `bench()` from the browser
 *
 * Solve times are for a *compiled* System — what dragging pays per frame — and separately for
 * compile+solve.  Benchmark on a quiet machine. */
import { PlanDrag, PlanSolver } from '../core/decompose.js';
import { diagnose } from '../core/diagnose.js';
import * as examples from '../core/examples.js';
import { Sketch } from '../core/model.js';
import { Drag, METHODS, Method, System, now, solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

type Make = () => Sketch;

/** Milliseconds, from the core's seconds-based timer. */
const ms = (): number => now() * 1000;

function median(v: number[]): number {
  const s = [...v].sort((a, b) => a - b);
  return s.length % 2 ? s[(s.length - 1) / 2] : (s[s.length / 2 - 1] + s[s.length / 2]) / 2;
}

/** (median compiled-solve ms, every solve converged, mean iterations). */
function benchSolve(make: Make, method: Method, reps = 20, sigma = 1.0): [number, boolean, number] {
  const ts: number[] = [];
  let ok = true, its = 0;
  for (let i = 0; i < reps; i++) {
    const sk = make();
    sk.perturb(sigma, i);
    const s = new System(sk);
    const t0 = ms();
    const r = s.solve({ method });
    ts.push(ms() - t0);
    ok &&= r.success;
    its += r.iterations;
    s.dispose();
  }
  return [median(ts), ok, its / reps];
}

function benchCompile(make: Make, reps = 20): number {
  const ts: number[] = [];
  for (let i = 0; i < reps; i++) {
    const sk = make();
    const t0 = ms();
    const s = new System(sk);
    ts.push(ms() - t0);
    s.dispose();
  }
  return median(ts);
}

/** (median frame ms, whether the cached plan drove it). */
function benchDrag(make: Make, plan: boolean, frames = 40): [number, boolean] {
  const sk = make();
  solve(sk);
  const p = sk.points[Math.floor(sk.points.length / 2)];
  const d = plan ? new PlanDrag(sk, p, p.x.value, p.y.value) : new Drag(sk, p, p.x.value, p.y.value);
  const ts: number[] = [];
  for (let i = 0; i < frames; i++) {
    const t0 = ms();
    d.move(p.x.value + 0.3 * Math.cos(i), p.y.value + 0.3 * Math.sin(i));
    ts.push(ms() - t0);
  }
  const driven = d instanceof PlanDrag ? d.usable : false;
  d.end();
  return [median(ts), driven];
}

const CASES: [string, Make][] = [
  ['rect_fillets', () => examples.rectFillets()],
  ['slotted_link', () => examples.slottedLink()],
  ['truss(8)', () => examples.truss(8)],
  ['truss(50)', () => examples.truss(50)],
  ['truss(200)', () => examples.truss(200)],
  ['polygon_chain(12)', () => examples.polygonChain(12)],
];

export interface BenchLine {
  name: string;
  free: number;
  res: number;
  compileMs: number;
  solve: Record<string, number>;
  planMs: number | null;
  dragMs: number;
  planDragMs: number;
  planDriven: boolean;
  diagnoseMs: number;
}

export function bench(): BenchLine[] {
  const out: BenchLine[] = [];
  for (const [name, make] of CASES) {
    const probe = make();
    const s = new System(probe);
    const free = s.nFree, res = s.nRes;
    s.dispose();
    const solveMs: Record<string, number> = {};
    for (const m of METHODS) {
      const [ms, ok] = benchSolve(make, m, name.includes('200') ? 4 : 20);
      solveMs[m] = ok ? ms : NaN;
    }
    let planMs: number | null = null;
    {
      const sk = make();
      const ps = new PlanSolver(sk);
      const ts: number[] = [];
      for (let i = 0; i < 10; i++) {
        sk.perturb(1.0, i);
        const t0 = ms();
        ps.solve();
        ts.push(ms() - t0);
      }
      planMs = median(ts);
      ps.dispose();
    }
    const [dragMs] = benchDrag(make, false, name.includes('200') ? 8 : 40);
    const [planDragMs, planDriven] = benchDrag(make, true, name.includes('200') ? 8 : 40);
    const dsk = make();
    const t0 = ms();
    diagnose(dsk);
    const diagnoseMs = ms() - t0;
    out.push({ name, free, res, compileMs: benchCompile(make, 10), solve: solveMs, planMs, dragMs, planDragMs, planDriven, diagnoseMs });
  }
  return out;
}

export function format(rows: BenchLine[]): string {
  const head = ['case', 'free', 'res', 'compile', ...METHODS, 'plan', 'drag', 'plan-drag', 'diagnose'];
  const body = rows.map((r) => [
    r.name, String(r.free), String(r.res), r.compileMs.toFixed(2),
    ...METHODS.map((m) => (Number.isNaN(r.solve[m]) ? 'FAIL' : r.solve[m].toFixed(2))),
    r.planMs === null ? '-' : r.planMs.toFixed(2),
    r.dragMs.toFixed(2),
    `${r.planDragMs.toFixed(2)}${r.planDriven ? '*' : ''}`,
    r.diagnoseMs.toFixed(1),
  ]);
  const w = head.map((h, i) => Math.max(h.length, ...body.map((b) => b[i].length)));
  const line = (cells: string[]): string => cells.map((c, i) => (i ? c.padStart(w[i]) : c.padEnd(w[i]))).join('  ');
  return [line(head), line(w.map((n) => '-'.repeat(n))), ...body.map(line),
          '', 'milliseconds, median.  * = the cached decomposition plan drove the drag.'].join('\n');
}

if (typeof process !== 'undefined' && process.argv?.[1]?.endsWith('bench.js')) {
  await initCore();
  console.log(format(bench()));
}
