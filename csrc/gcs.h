/* gcs — geometric constraint solver core (C).
 *
 * The flat API the program's Stage 1 calls for: the Python/TypeScript layer owns the
 * object model and compiles a sketch down to an evaluation plan (arrays of kernel id,
 * parameter indices and constants); this library iterates that plan and owns every
 * number that touches the drag loop.
 *
 * Conventions
 *   * every matrix is row-major, every index is 0-based;
 *   * `x` is the full parameter vector, `z` the free sub-vector (x[free]);
 *   * residual rows are grouped by block, in the order the blocks were given.
 */
#ifndef GCS_H
#define GCS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -- kernels ------------------------------------------------------------- */
/* One per constraint type; ids are the registration order shared with the front end. */
enum {
    GCS_K_COINCIDENT = 0,
    GCS_K_DISTANCE,
    GCS_K_MIDPOINT,
    GCS_K_DRAG,
    GCS_K_HORIZONTAL,
    GCS_K_VERTICAL,
    GCS_K_PARALLEL,
    GCS_K_PERPENDICULAR,
    GCS_K_ANGLE,
    GCS_K_EQUAL_LENGTH,
    GCS_K_POINT_ON_LINE,
    GCS_K_POINT_ON_CIRCLE,
    GCS_K_RADIUS,
    GCS_K_EQUAL_RADIUS,
    GCS_K_TANGENT_LINE_CIRCLE,
    GCS_K_TANGENT_CIRCLE_CIRCLE,
    GCS_K_TANGENT_ARC_LINE,
    GCS_N_KERNELS
};

typedef void (*gcs_res_fn)(int n, const double *V, const double *K, double *R);
typedef void (*gcs_jac_fn)(int n, const double *V, const double *K, double *J);

typedef struct {
    const char *name;
    int n_res, n_par, n_const;
    gcs_res_fn res;
    gcs_jac_fn jac;
    const double *const_jac; /* n_res*n_par entries when the Jacobian is instance-independent */
} gcs_kernel;

const gcs_kernel *gcs_kernels(void);
int gcs_kernel_count(void);
/* Metadata for the front end: writes 4 ints (n_res, n_par, n_const, has_const_jac). */
void gcs_kernel_info(int kid, int32_t *out);

/* -- dense linear algebra -------------------------------------------------- */

/* Minimum-norm least-squares solution of A X = B via a complete orthogonal
 * decomposition (rank-revealing QR + RZ) — LAPACK dgelsy's algorithm.
 * A (m*n) and B (m*nrhs) are destroyed.  X is n*nrhs.  Returns the numerical rank. */
int gcs_min_norm_lstsq(int m, int n, int nrhs, double *A, double *B, double rcond, double *X);

/* Rank-revealing QR.  A (m*n) is destroyed; `piv` (n, may be NULL) receives the column
 * pivots — the first `rank` of them index a maximal independent set of columns.
 * The rank convention shared by the whole codebase: |R_ii| > rcond*|R_00|. */
int gcs_rrqr(int m, int n, double *A, double rcond, int32_t *piv);

/* Singular values (descending) and the full right factor of A (m*n), by one-sided Jacobi.
 * `S` holds min(m,n) values, `Vt` is n*n, `U` (may be NULL) is m*min(m,n) (thin). */
int gcs_svd(int m, int n, const double *A, double *U, double *S, double *Vt);

/* Numerical rank and null space from one SVD: `N` (n * n, the first `n - rank` columns
 * used) receives an orthonormal basis of the null space.  Returns the rank. */
int gcs_rank_nullspace(int m, int n, const double *A, double rcond, double *N, double *S);

/* Solve the n*n system A x = b in place (partial-pivoting LU).  Returns 0 on success. */
int gcs_lu_solve(int n, double *A, double *b);

/* -- compiled system ------------------------------------------------------- */

typedef struct gcs_system gcs_system;

/* Build the evaluation plan.  Blocks are groups of constraints sharing a kernel;
 * `gidx` holds each block's (count * n_par) global parameter indices and `consts`
 * its (count * n_const) constants, concatenated in block order.  `soft` is one flag
 * per constraint (also block order): soft rows do not count toward convergence. */
gcs_system *gcs_system_new(int n_params, const double *x0,
                           int n_free, const int32_t *free_idx,
                           int n_blocks, const int32_t *kernel_id, const int32_t *count,
                           const int32_t *gidx, const double *consts, const int32_t *soft);
void gcs_system_free(gcs_system *s);

int gcs_system_n_res(const gcs_system *s);
int gcs_system_n_free(const gcs_system *s);
int gcs_system_nnz(const gcs_system *s);

void gcs_system_set_x(gcs_system *s, const double *x);
void gcs_system_get_x(const gcs_system *s, double *x);
void gcs_system_get_z(const gcs_system *s, double *z);      /* x[free] */
void gcs_system_full_x(const gcs_system *s, const double *z, double *x);
/* Replace one constraint's constants (a moving drag target, an edited dimension). */
void gcs_system_set_consts(gcs_system *s, int block, int row, const double *c);
void gcs_system_set_all_consts(gcs_system *s, const double *consts);

void gcs_system_residuals(gcs_system *s, const double *z, double *r);
void gcs_system_jacobian_dense(gcs_system *s, const double *z, double *J);   /* n_res * n_free */
/* Sparse Jacobian in CSR.  Structure (indptr, indices) is fixed at compile time. */
const int32_t *gcs_system_csr_indptr(const gcs_system *s);
const int32_t *gcs_system_csr_indices(const gcs_system *s);
void gcs_system_csr_data(gcs_system *s, const double *z, double *data);
/* Rows that must be satisfied (0/1 per residual row). */
const uint8_t *gcs_system_hard(const gcs_system *s);
double gcs_system_max_hard_residual(gcs_system *s, const double *z);
/* max |residual| per constraint, in block order. */
void gcs_system_constraint_errors(gcs_system *s, const double *z, double *out);
int gcs_system_rank(gcs_system *s, const double *z, double rcond, int hard_only);

/* -- solvers --------------------------------------------------------------- */

enum { GCS_DOGLEG = 0, GCS_LM = 1 };

typedef struct {
    int status;       /* 0 ftol, 1 xtol, 2 gtol, 3 trust region collapsed, 4 max iterations, -1 failed */
    int nfev, njev, iterations;
    int rank;         /* numerical rank of J at the solution, or -1 (sparse path) */
} gcs_info;

/* Minimise 0.5*||r(z)||^2 from z (updated in place).  `dense` < 0 picks by size. */
int gcs_system_solve(gcs_system *s, int method, double ftol, double xtol, double gtol,
                     int max_iter, int max_nfev, int dense, double *z, gcs_info *info);

#ifdef __cplusplus
}
#endif
#endif /* GCS_H */
