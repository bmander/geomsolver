/* Loading the core and moving numbers across the boundary.
 *
 * The Rust core is compiled to a single self-contained WebAssembly module: no imports, no
 * emscripten runtime, no glue to keep in step — `WebAssembly.instantiate` and the exports below
 * are the whole loader.  This module is the only layer that knows about pointers; nothing under
 * `app/` does.  Heap views are re-read on every access because the module grows its memory on
 * demand, which detaches the old typed arrays.
 */

/** The flat C ABI, as exported by `rust/gcs-ffi`. */
export interface Abi {
  memory: WebAssembly.Memory;

  gcs_malloc(size: number): number;
  gcs_free(ptr: number, size: number): void;
  gcs_str_len(p: number): number;
  gcs_str_ptr(p: number): number;
  gcs_str_free(p: number): void;
  gcs_last_error(): number;

  gcs_registry_json(): number;
  gcs_version(): number;
  gcs_kernel_count(): number;

  gcs_sketch_new(): number;
  gcs_sketch_free(h: number): void;
  gcs_sketch_clone(h: number): number;
  gcs_sketch_from_json(p: number, n: number): number;
  gcs_sketch_to_json(h: number, indent: number): number;
  gcs_sketch_counts(h: number, out: number): void;
  gcs_sketch_point(h: number, x: number, y: number, fixed: number, name: number, nameLen: number): number;
  gcs_sketch_line(h: number, p1: number, p2: number): number;
  gcs_sketch_circle(h: number, center: number, r: number, name: number, nameLen: number): number;
  gcs_sketch_arc(h: number, c: number, s: number, e: number, name: number, nameLen: number): number;
  gcs_sketch_arc_through(h: number, s: number, e: number, tx: number, ty: number, name: number, nameLen: number): number;
  gcs_sketch_spline(h: number, ctrl: number, n: number): number;
  gcs_sketch_spline_knots(h: number, ctrl: number, n: number, knots: number, nk: number): number;
  gcs_sketch_spline_through(h: number, pts: number, n: number, hold: number): number;
  gcs_sketch_rectangle(h: number, a: number, x1: number, y1: number, name: number, nameLen: number, out: number): void;
  gcs_sketch_get_x(h: number, out: number): void;
  gcs_sketch_set_x(h: number, x: number, n: number): number;
  gcs_sketch_perturb(h: number, sigma: number, seed: number): void;
  gcs_sketch_topology_key(h: number): number;
  gcs_sketch_extent(h: number): number;
  gcs_sketch_bounds(h: number, drawn: number, out: number): void;
  gcs_sketch_nearest_point(h: number, x: number, y: number, outDist: number): number;
  gcs_sketch_n_residuals(h: number): number;
  gcs_sketch_set_constraints(h: number, p: number, n: number): void;

  gcs_param_value(h: number, i: number): number;
  gcs_param_set_value(h: number, i: number, v: number): void;
  gcs_param_fixed(h: number, i: number): number;
  gcs_param_set_fixed(h: number, i: number, v: number): void;
  gcs_param_name(h: number, i: number): number;

  gcs_entity_params(h: number, kind: number, idx: number, out: number): number;
  gcs_entity_points(h: number, kind: number, idx: number, out: number): number;
  gcs_entity_radius_param(h: number, kind: number, idx: number): number;
  gcs_entity_construction(h: number, kind: number, idx: number): number;
  gcs_entity_set_construction(h: number, kind: number, idx: number, v: number): void;
  gcs_entity_bounds(h: number, kind: number, idx: number, out: number): void;
  gcs_entity_name(kind: number, idx: number): number;
  gcs_distance_between(h: number, ka: number, ia: number, kb: number, ib: number): number;
  gcs_signed_point_to_line(h: number, x: number, y: number, line: number): number;
  gcs_angle_between(h: number, a: number, b: number): number;
  gcs_on_radius(cx: number, cy: number, tx: number, ty: number, r: number,
                out: number): number;
  gcs_arc_angles(h: number, idx: number, out: number): void;
  gcs_spline_knots(h: number, idx: number, out: number): number;
  gcs_spline_domain(h: number, idx: number, out: number): void;
  gcs_spline_eval(h: number, idx: number, t: number, out: number): void;
  gcs_spline_polyline(h: number, idx: number, unit: number, out: number, cap: number): number;
  gcs_spline_closest(h: number, idx: number, x: number, y: number, out: number): void;
  gcs_spline_insert_control(h: number, idx: number, t: number): number;
  gcs_three_point_arc(ax: number, ay: number, bx: number, by: number, cx: number, cy: number, out: number): number;

  gcs_constraint_add(h: number, p: number, n: number): number;
  gcs_constraint_remove(h: number, id: number): void;
  gcs_constraints_json(h: number): number;
  gcs_constraint_json(h: number, id: number): number;
  gcs_constraint_set_num(h: number, id: number, name: number, nameLen: number,
                         v: number): number;
  gcs_constraint_set_str(h: number, id: number, name: number, nameLen: number,
                         v: number, vLen: number): number;
  gcs_constraint_set_dimension(h: number, id: number, name: number, nameLen: number,
                               text: number, textLen: number): number;
  gcs_exprs_json(h: number): number;
  gcs_constraint_set_target(h: number, id: number, tx: number, ty: number): void;
  gcs_constraint_error(h: number, id: number): number;
  gcs_constraint_params(h: number, id: number, out: number): number;
  gcs_constraint_local_values(h: number, id: number, out: number): number;
  gcs_constraint_eval(h: number, id: number, v: number, r: number, j: number): number;
  gcs_same_constraint(h: number, a: number, aLen: number, b: number,
                      bLen: number): number;
  gcs_constraint_duplicate(h: number, p: number, n: number): number;
  gcs_constraint_stating(h: number, p: number, n: number): number;
  gcs_describe(h: number, id: number): number;
  gcs_callouts_json(h: number, unit: number): number;
  gcs_callout_pick(h: number, unit: number, x: number, y: number, tolPx: number): number;
  gcs_callout_grab(h: number, id: number, unit: number, x: number, y: number,
                   out: number): number;
  gcs_callout_drag(h: number, id: number, x: number, y: number, gu: number, gv: number): number;
  gcs_callout_reset(h: number, id: number): number;
  gcs_fmt_g(v: number, sig: number): number;

  gcs_branches_json(h: number): number;
  gcs_branches_set_json(h: number, p: number, n: number): void;
  gcs_without(h: number, ep: number, en: number, cp: number, cn: number): number;
  gcs_copy(h: number, ep: number, en: number): number;
  gcs_paste(h: number, clip: number, dx: number, dy: number): number;
  gcs_example(name: number, len: number): number;
  gcs_cases_json(): number;

  gcs_system_new(h: number): number;
  gcs_system_free(s: number): void;
  gcs_system_n_res(s: number): number;
  gcs_system_n_free(s: number): number;
  gcs_system_nnz(s: number): number;
  gcs_system_scale(s: number): number;
  gcs_system_hard(s: number, out: number): void;
  gcs_system_z0(s: number, h: number, out: number): void;
  gcs_system_residuals(s: number, z: number, out: number): void;
  gcs_system_jacobian_dense(s: number, z: number, out: number): void;
  gcs_system_csr_structure(s: number, indptr: number, indices: number): void;
  gcs_system_csr_data(s: number, z: number, out: number): void;
  gcs_system_max_hard_residual(s: number, h: number): number;
  gcs_system_constraint_errors(s: number, h: number, ids: number, out: number,
                               cap: number): number;
  gcs_system_n_constraints(s: number): number;
  gcs_system_max_relative_residual(s: number, h: number): number;
  gcs_system_rank(s: number, h: number, rcond: number, hardOnly: number): number;
  gcs_system_update_consts(s: number, h: number, id: number): void;
  gcs_system_refresh_consts(s: number, h: number): void;
  gcs_system_structure_json(s: number): number;
  gcs_system_free_indices(s: number, out: number): void;
  gcs_system_row_of(s: number, id: number): number;
  gcs_system_solve(s: number, h: number, method: number, tol: number, maxIter: number,
                   maxNfev: number, dense: number, writeback: number, out: number): number;
  gcs_solve(h: number, method: number, tol: number, maxIter: number, maxNfev: number,
            dense: number, out: number): number;
  gcs_status_message(status: number): number;

  gcs_min_norm_lstsq(m: number, n: number, nrhs: number, a: number, b: number, rcond: number, x: number): number;
  gcs_rrqr(m: number, n: number, a: number, rcond: number, piv: number): number;
  gcs_svd(m: number, n: number, a: number, u: number, s: number, vt: number): number;
  gcs_rank_nullspace(m: number, n: number, a: number, rcond: number, nOut: number, sOut: number): number;
  gcs_lu_solve(n: number, a: number, b: number): number;

  gcs_check_sketch(h: number, rtol: number, atol: number): number;
  gcs_check_constraint(h: number, id: number, rtol: number, atol: number): number;

  gcs_hopcroft_karp_json(adj: number, len: number, nRight: number): number;
  gcs_dulmage_mendelsohn_json(adj: number, len: number, nCols: number): number;
  gcs_pebble_game_json(n: number, edges: number, len: number): number;
  gcs_bipartite_components_json(adj: number, len: number, nCols: number): number;
  gcs_henneberg_edges_json(n: number, seed: number): number;

  gcs_diagnose_json(h: number, p: number, n: number): number;
  gcs_diagnose_with_json(h: number, s: number, p: number, n: number): number;
  gcs_minimal_conflict_set_json(h: number, p: number, n: number, tol: number): number;
  gcs_violated_json(h: number, tol: number): number;
  gcs_distance_rigidity_json(h: number): number;
  gcs_witness_json(h: number, seed: number): number;
  gcs_make_witness(h: number, seed: number, out: number): void;

  gcs_graph_json(h: number): number;
  gcs_plan_solver_new(h: number, sticky: number): number;
  gcs_plan_solver_free(p: number): void;
  gcs_plan_solver_system(p: number): number;
  gcs_plan_solver_plan_json(p: number): number;
  gcs_plan_solver_graph_json(p: number): number;
  gcs_plan_solver_solve(p: number, h: number, tol: number, fallback: number, method: number): number;
  gcs_plan_solver_flip(p: number, h: number, point: number): number;
  gcs_plan_solver_sticky(p: number, sticky: number): void;
  gcs_plan_solver_execute(p: number, h: number): void;
  gcs_plan_solver_point_element(p: number, point: number): number;
  gcs_ppp_triangles(p: number, out: number): number;
  gcs_plan_steps_placing(p: number, point: number, out: number): number;

  gcs_enumerate_step_json(p: number, h: number, step: number, locate: number, seed: number, maxPaths: number): number;
  gcs_apply_alternative(p: number, h: number, step: number, alt: number, len: number): void;

  gcs_drag_new(h: number, point: number, x: number, y: number, method: number, weight: number,
               guards: number, nGuards: number, maxStepRel: number): number;
  gcs_drag_move(d: number, h: number, x: number, y: number, out: number): number;
  gcs_drag_end(d: number, h: number): void;
  gcs_drag_free(d: number): void;
  gcs_drag_flips(d: number): number;
  gcs_drag_flip_list(d: number, out: number): number;
  gcs_radius_drag_new(h: number, kind: number, idx: number, r: number, method: number): number;
  gcs_radius_drag_move(d: number, h: number, r: number, out: number): number;
  gcs_radius_drag_end(d: number, h: number): void;
  gcs_radius_drag_free(d: number): void;
  gcs_plan_drag_new(h: number, ps: number, point: number, x: number, y: number, guards: number,
                    nGuards: number, maxStepRel: number): number;
  gcs_plan_drag_move(d: number, h: number, x: number, y: number, out: number): number;
  gcs_plan_drag_usable(d: number): number;
  gcs_plan_drag_flips(d: number): number;
  gcs_plan_drag_flip_list(d: number, out: number): number;
  gcs_plan_drag_branches_json(d: number): number;
  gcs_plan_drag_guards(d: number, h: number, out: number): number;
  gcs_plan_drag_end(d: number, h: number): void;
  gcs_plan_drag_free(d: number): void;
}

let abi: Abi | null = null;
const hooks: (() => void)[] = [];

/** Run `h` once the core is loaded (immediately if it already is).  The constraint classes are
 *  generated from the core's registry, so they register here rather than at module load. */
export function onInit(h: () => void): void {
  if (abi) h();
  else hooks.push(h);
}

async function wasmBytes(url?: string): Promise<BufferSource> {
  const target = url ?? new URL('../wasm/gcs.wasm', import.meta.url).href;
  if (typeof fetch === 'function' && !target.startsWith('file:')) {
    const res = await fetch(target);
    if (!res.ok) throw new Error(`could not fetch ${target}: ${res.status}`);
    return await res.arrayBuffer();
  }
  const { readFile } = await import('node:fs/promises');
  const { fileURLToPath } = await import('node:url');
  return await readFile(target.startsWith('file:') ? fileURLToPath(target) : target);
}

/** Load the core once; every later call returns the same instance. */
export async function initCore(opts?: { url?: string; bytes?: BufferSource }): Promise<Abi> {
  if (abi) return abi;
  const bytes = opts?.bytes ?? (await wasmBytes(opts?.url));
  const { instance } = await WebAssembly.instantiate(bytes, {});
  abi = instance.exports as unknown as Abi;
  for (const h of hooks.splice(0)) h();
  return abi;
}

/** The loaded core.  Throws if `initCore` has not resolved yet. */
export function core(): Abi {
  if (!abi) throw new Error('gcs core not initialised — await initCore() first');
  return abi;
}

/* -- heap access ------------------------------------------------------------ */

export const u8 = (): Uint8Array => new Uint8Array(core().memory.buffer);
export const i32 = (): Int32Array => new Int32Array(core().memory.buffer);
export const f64 = (): Float64Array => new Float64Array(core().memory.buffer);

/** A block of the core's heap with a matching view on demand. */
/** A scratch block in the core's heap.
 *
 *  The typed-array getters below build a *view* over the module's memory.  That memory grows on
 *  any core call, and growing detaches every view over the old buffer — so a view must never be
 *  held across a call into the core.  Read it, or copy out of it, first. */
export class Buf {
  readonly ptr: number;
  readonly bytes: number;

  constructor(readonly len: number, readonly width: 4 | 8 | 1 = 8) {
    this.bytes = Math.max(len * width, 8);
    this.ptr = core().gcs_malloc(this.bytes);
  }

  get f64(): Float64Array {
    return f64().subarray(this.ptr >> 3, (this.ptr >> 3) + this.len);
  }

  get i32(): Int32Array {
    return i32().subarray(this.ptr >> 2, (this.ptr >> 2) + this.len);
  }

  get u8(): Uint8Array {
    return u8().subarray(this.ptr, this.ptr + this.len);
  }

  set(src: ArrayLike<number>): this {
    if (this.width === 8) this.f64.set(src);
    else if (this.width === 4) this.i32.set(src);
    else this.u8.set(src);
    return this;
  }

  release(): void {
    core().gcs_free(this.ptr, this.bytes);
  }
}

/** Run `fn` with a scratch buffer, releasing it whatever happens. */
export function withBuf<T>(len: number, width: 4 | 8 | 1, fn: (b: Buf) => T): T {
  const b = new Buf(Math.max(len, 1), width);
  try {
    return fn(b);
  } finally {
    b.release();
  }
}

const encoder = new TextEncoder();
const decoder = new TextDecoder();

/** Copy a string into the core's heap and hand `fn` its (pointer, length). */
export function withStr<T>(s: string, fn: (ptr: number, len: number) => T): T {
  const bytes = encoder.encode(s);
  const b = new Buf(Math.max(bytes.length, 1), 1);
  try {
    u8().set(bytes, b.ptr);
    return fn(b.ptr, bytes.length);
  } finally {
    b.release();
  }
}

export function withJson<T>(v: unknown, fn: (ptr: number, len: number) => T): T {
  return withStr(JSON.stringify(v), fn);
}

/** Consume a length-prefixed string block returned by the core. */
export function takeStr(handle: number): string {
  if (!handle) return '';
  const c = core();
  const n = c.gcs_str_len(handle);
  const p = c.gcs_str_ptr(handle);
  const out = n ? decoder.decode(u8().subarray(p, p + n)) : '';
  c.gcs_str_free(handle);
  return out;
}

export function takeJson<T>(handle: number): T {
  const s = takeStr(handle);
  return (s ? JSON.parse(s) : null) as T;
}

export function lastError(): string {
  return takeStr(core().gcs_last_error());
}
