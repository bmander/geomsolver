/* Ambient declaration for the Emscripten ES6 module built from csrc/. */
export interface GcsModule {
  HEAPF64: Float64Array;
  HEAP32: Int32Array;
  HEAPU32: Uint32Array;
  HEAPU8: Uint8Array;
  _malloc(n: number): number;
  _free(p: number): void;
  _gcs_kernel_count(): number;
  _gcs_kernel_info(kid: number, out: number): void;
  _gcs_min_norm_lstsq(m: number, n: number, nrhs: number, A: number, B: number, rcond: number, X: number): number;
  _gcs_rrqr(m: number, n: number, A: number, rcond: number, piv: number): number;
  _gcs_svd(m: number, n: number, A: number, U: number, S: number, Vt: number): number;
  _gcs_rank_nullspace(m: number, n: number, A: number, rcond: number, N: number, S: number): number;
  _gcs_lu_solve(n: number, A: number, b: number): number;
  _gcs_system_new(nParams: number, x0: number, nFree: number, freeIdx: number, nBlocks: number,
                  kernelId: number, count: number, gidx: number, consts: number, soft: number): number;
  _gcs_system_free(h: number): void;
  _gcs_system_n_res(h: number): number;
  _gcs_system_n_free(h: number): number;
  _gcs_system_nnz(h: number): number;
  _gcs_system_set_x(h: number, x: number): void;
  _gcs_system_get_x(h: number, x: number): void;
  _gcs_system_get_z(h: number, z: number): void;
  _gcs_system_full_x(h: number, z: number, x: number): void;
  _gcs_system_set_consts(h: number, block: number, row: number, c: number): void;
  _gcs_system_set_all_consts(h: number, consts: number): void;
  _gcs_system_residuals(h: number, z: number, r: number): void;
  _gcs_system_jacobian_dense(h: number, z: number, J: number): void;
  _gcs_system_csr_indptr(h: number): number;
  _gcs_system_csr_indices(h: number): number;
  _gcs_system_csr_data(h: number, z: number, data: number): void;
  _gcs_system_hard(h: number): number;
  _gcs_system_max_hard_residual(h: number, z: number): number;
  _gcs_system_constraint_errors(h: number, z: number, out: number): void;
  _gcs_system_rank(h: number, z: number, rcond: number, hardOnly: number): number;
  _gcs_system_solve(h: number, method: number, ftol: number, xtol: number, gtol: number,
                    maxIter: number, maxNfev: number, dense: number, z: number, info: number): number;
}
declare const factory: (opts?: Record<string, unknown>) => Promise<GcsModule>;
export default factory;
