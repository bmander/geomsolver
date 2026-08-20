/* Reference sketches used by the tests, the benchmarks and the app's case library.
 *
 * The sketches themselves are built in the core; this is the lookup and the case list. */
import { Sketch } from './model.js';
import { core, lastError, takeJson, withStr } from './wasm.js';

/** Build a named example.  Keys are a plain name or `name:arg[:arg]` — `truss:50`, `laman:12:1`. */
export function build(key: string): Sketch {
  const h = withStr(key, (p, n) => core().gcs_example(p, n));
  if (!h) throw new Error(lastError() || `unknown example: ${key}`);
  return new Sketch(h);
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
export const parallels = (): Sketch => build('parallels');
export const k33 = (seed = 3): Sketch => build(`k33:${seed}`);
export const laman = (n = 10, seed = 0): Sketch => build(`laman:${n}:${seed}`);

/** A random Laman graph's edges by Henneberg construction — the property-test generator. */
export function hennebergEdges(n: number, seed = 0): [number, number][] {
  return takeJson<[number, number][]>(core().gcs_henneberg_edges_json(n, seed >>> 0)) ?? [];
}

export const EXAMPLES: Record<string, () => Sketch> = {
  rect_fillets: () => rectFillets(),
  slotted_link: () => slottedLink(),
  truss: () => truss(),
  polygon_chain: () => polygonChain(),
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
