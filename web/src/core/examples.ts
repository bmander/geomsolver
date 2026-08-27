/* Reference sketches used by the tests, the benchmarks and the app's case library.
 *
 * The sketches themselves are built in the core; this is the lookup and the case list. */
import { Sketch } from './model.js';
import { fromSketch } from './program.js';
import { core, lastError, takeJson, takeStr, withStr } from './wasm.js';

/** Build a named example.  Keys are a plain name or `name:arg[:arg]` — `truss:50`, `laman:12:1`. */
export function build(key: string): Sketch {
  const h = withStr(key, (p, n) => core().gcs_example(p, n));
  if (!h) throw new Error(lastError() || `unknown example: ${key}`);
  return new Sketch(h);
}

/** A named example as a *program* — which is what the document is.
 *
 *  A case written as a document has a source somebody wrote, comments and components and all, and
 *  that source is the case.  One built by a function has none, so it is lifted; either way the
 *  caller gets a program and never has to know which kind it was. */
export function source(key: string): string {
  const h = withStr(key, (p, n) => core().gcs_example_source(p, n));
  if (h) return takeStr(h);
  const sk = build(key);
  try {
    return fromSketch(sk);
  } finally {
    sk.dispose();
  }
}

export const rectFillets = (w = 100, h = 60, r = 10): Sketch => build(`rect_fillets:${w}:${h}:${r}`);
export const slottedLink = (): Sketch => build('slotted_link');
export const truss = (bays = 8): Sketch => build(`truss:${bays}`);
export const polygonChain = (n = 12): Sketch => build(`polygon_chain:${n}`);
export const rectFilletsConflict = (): Sketch => build('rect_fillets_conflict');
export const rectFilletsUnder = (): Sketch => build('rect_fillets_under');
export const trussRedundant = (): Sketch => build('truss_redundant');
export const trussConflict = (): Sketch => build('truss_conflict');
export const trussFloating = (bays = 8): Sketch => build(`truss_floating:${bays}`);
export const impossibleTriangle = (): Sketch => build('impossible_triangle');
export const altitudes = (): Sketch => build('altitudes');
/** A belt over two pulleys, each end held on its circle and the line tangent to it: a double
 *  root — rank-deficient at every solution, yet nothing can move. */
export const beltTangency = (): Sketch => build('belt_tangency');
export const parallels = (): Sketch => build('parallels');
/** The graphical proof of the Pythagorean theorem: four a×b right triangles in a square of side
 *  a + b leave a square whose side is claimed to be `c = hypot(a, b)` — judged a theorem. */
export const pythagoras = (a = 30, b = 40): Sketch => build(`pythagoras:${a}:${b}`);
export const k33 = (): Sketch => build('k33');
export const laman = (n = 10, seed = 0): Sketch => build(`laman:${n}:${seed}`);
/** A cubic B-spline with a face held tangent to it and a point riding on it. */
/** The control points are written out in the document, so their number is its own. */
export const splineFollower = (): Sketch => build('spline_follower');
/** `copies` separate staircases of `n` free-length H/V segments: a drag must cost one figure. */
export const zigzag = (n = 32, copies = 1): Sketch => build(`zigzag:${n}:${copies}`);
/** The Peaucellier–Lipkin cell: circling rods whose pen draws an exact straight line, the
 *  straightness one `claim` the diagnosis judges a theorem. */
export const peaucellier = (): Sketch => build('peaucellier');

/** A random Laman graph's edges by Henneberg construction — the property-test generator. */
export function hennebergEdges(n: number, seed = 0): [number, number][] {
  return takeJson<[number, number][]>(core().gcs_henneberg_edges_json(n, seed >>> 0)) ?? [];
}

export const EXAMPLES: Record<string, () => Sketch> = {
  rect_fillets: () => rectFillets(),
  slotted_link: () => slottedLink(),
  truss: () => truss(),
  polygon_chain: () => polygonChain(),
  spline_follower: () => splineFollower(),
};

export interface Case {
  label: string;
  key: string;
  description: string;
}

/** The case library shown in the app: label, key and a one-line description. */
export function cases(): Case[] {
  return takeJson<Case[]>(core().gcs_cases_json()) ?? [];
}
