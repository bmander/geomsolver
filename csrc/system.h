/* Internal layout of the compiled plan (shared by system.c and newton.c). */
#ifndef GCS_SYSTEM_H
#define GCS_SYSTEM_H

#include <stdint.h>
#include "sparse.h"

typedef struct {
    int kid, count, row0, gidx_off, consts_off, jac_off;
} gcs_block;

struct gcs_system {
    int n_params, n_free, n_blocks, n_res, n_cons;
    int consts_len;
    double *x;
    int32_t *free_idx, *col_of;
    gcs_block *blocks;
    int32_t *gidx;
    double *consts;
    double *V;        /* gather scratch */
    double *jdata;    /* per-block Jacobian entries */
    double *r;        /* residual scratch */
    uint8_t *hard;
    int n_ent, nnz;
    int32_t *ent_src, *ent_slot;
    int32_t *csr_indptr, *csr_indices;
    double *csr_data;
    gcs_ata *ata;     /* J^T J for the sparse path, built on first use */
};

void gcs_system_apply_z(struct gcs_system *s, const double *z);

#endif
