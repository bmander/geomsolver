/* Sparse normal equations for the large-sketch path: J^T J assembled from the fixed
 * CSR structure of the Jacobian, ordered by reverse Cuthill-McKee and factored by an
 * up-looking LDL^T (Davis).  Regularizing the diagonal keeps rank-deficient
 * (under-constrained) systems solvable, which is the normal case while editing.
 */
#ifndef GCS_SPARSE_H
#define GCS_SPARSE_H

#include <stdint.h>

typedef struct {
    int n;                  /* columns of J = rows/cols of A */
    int nnz;                /* entries of A (full symmetric pattern) */
    int32_t *ap, *ai;       /* CSR == CSC of the symmetric A */
    double *ax;
    int n_tri;              /* rank-1 contributions */
    int32_t *ta, *tb, *ts;  /* A[ts[t]] += Jdata[ta[t]] * Jdata[tb[t]] */
    /* upper-triangular CSC view handed to the factorization */
    int32_t *up, *ui;
    double *ux;
    /* factorization workspace */
    int32_t *perm, *iperm, *parent, *lnz, *lp, *li, *flag, *pattern;
    double *d, *lx, *y;
    int lnz_total;
} gcs_ata;

gcs_ata *gcs_ata_new(int n_rows, int n_cols, const int32_t *indptr, const int32_t *indices);
void gcs_ata_free(gcs_ata *a);
/* A <- J^T J from the Jacobian's CSR values. */
void gcs_ata_fill(gcs_ata *a, const double *jdata);
/* Solve (A + diag(damp)) x = b in place; returns 0 on success. */
int gcs_ata_solve(gcs_ata *a, const double *damp, double *b);
/* The diagonal of A, in original (unpermuted) order. */
void gcs_ata_diag(const gcs_ata *a, double *d);

#endif
