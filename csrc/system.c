/* The compiled evaluation plan: constraints grouped into blocks by kernel, with the
 * Jacobian's sparsity structure (CSR indices and the duplicate-summing scatter map)
 * computed once.  Each evaluation is one kernel call per block plus a scatter — no
 * per-constraint branching, which is the whole point of compiling to a plan.
 */
#include "gcs.h"
#include "sparse.h"
#include "system.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct { int64_t key; int32_t src; } ent;

static int ent_cmp(const void *a, const void *b)
{
    int64_t x = ((const ent *)a)->key, y = ((const ent *)b)->key;
    return x < y ? -1 : (x > y ? 1 : 0);
}

gcs_system *gcs_system_new(int n_params, const double *x0,
                           int n_free, const int32_t *free_idx,
                           int n_blocks, const int32_t *kernel_id, const int32_t *count,
                           const int32_t *gidx, const double *consts, const int32_t *soft)
{
    const gcs_kernel *KS = gcs_kernels();
    gcs_system *s = (gcs_system *)calloc(1, sizeof(gcs_system));
    s->n_params = n_params;
    s->n_free = n_free;
    s->n_blocks = n_blocks;
    s->x = (double *)malloc(sizeof(double) * (size_t)(n_params ? n_params : 1));
    memcpy(s->x, x0, sizeof(double) * (size_t)n_params);
    s->free_idx = (int32_t *)malloc(sizeof(int32_t) * (size_t)(n_free ? n_free : 1));
    memcpy(s->free_idx, free_idx, sizeof(int32_t) * (size_t)n_free);
    s->col_of = (int32_t *)malloc(sizeof(int32_t) * (size_t)(n_params ? n_params : 1));
    for (int i = 0; i < n_params; i++) s->col_of[i] = -1;
    for (int i = 0; i < n_free; i++) s->col_of[free_idx[i]] = i;
    s->blocks = (gcs_block *)calloc((size_t)(n_blocks ? n_blocks : 1), sizeof(gcs_block));

    int row0 = 0, goff = 0, coff = 0, joff = 0, ncons = 0, max_scratch = 1;
    for (int b = 0; b < n_blocks; b++) {
        const gcs_kernel *k = &KS[kernel_id[b]];
        int n = count[b];
        s->blocks[b].kid = kernel_id[b];
        s->blocks[b].count = n;
        s->blocks[b].row0 = row0;
        s->blocks[b].gidx_off = goff;
        s->blocks[b].consts_off = coff;
        s->blocks[b].jac_off = joff;
        row0 += n * k->n_res;
        goff += n * k->n_par;
        coff += n * k->n_const;
        joff += n * k->n_res * k->n_par;
        ncons += n;
        if (n * k->n_par > max_scratch) max_scratch = n * k->n_par;
    }
    s->n_res = row0;
    s->n_cons = ncons;
    s->consts_len = coff;
    s->gidx = (int32_t *)malloc(sizeof(int32_t) * (size_t)(goff ? goff : 1));
    memcpy(s->gidx, gidx, sizeof(int32_t) * (size_t)goff);
    s->consts = (double *)malloc(sizeof(double) * (size_t)(coff ? coff : 1));
    if (coff) memcpy(s->consts, consts, sizeof(double) * (size_t)coff);
    s->V = (double *)malloc(sizeof(double) * (size_t)max_scratch);
    s->jdata = (double *)calloc((size_t)(joff ? joff : 1), sizeof(double));
    s->r = (double *)malloc(sizeof(double) * (size_t)(row0 ? row0 : 1));
    s->hard = (uint8_t *)malloc((size_t)(row0 ? row0 : 1));
    {
        int ci = 0, ri = 0;
        for (int b = 0; b < n_blocks; b++) {
            const gcs_kernel *k = &KS[s->blocks[b].kid];
            for (int i = 0; i < s->blocks[b].count; i++, ci++)
                for (int t = 0; t < k->n_res; t++) s->hard[ri++] = (uint8_t)(soft[ci] == 0);
        }
    }
    /* constant Jacobians are filled once and never recomputed */
    for (int b = 0; b < n_blocks; b++) {
        const gcs_kernel *k = &KS[s->blocks[b].kid];
        if (!k->const_jac) continue;
        size_t sz = (size_t)k->n_res * k->n_par;
        for (int i = 0; i < s->blocks[b].count; i++)
            memcpy(s->jdata + s->blocks[b].jac_off + (size_t)i * sz, k->const_jac, sizeof(double) * sz);
    }

    /* -- Jacobian structure: entry (block, i, res, par) -> (row, col), duplicates merged -- */
    int ncols = n_free > 0 ? n_free : 1;
    ent *es = (ent *)malloc(sizeof(ent) * (size_t)(joff ? joff : 1));
    int ne = 0;
    for (int b = 0; b < n_blocks; b++) {
        const gcs_kernel *k = &KS[s->blocks[b].kid];
        for (int i = 0; i < s->blocks[b].count; i++)
            for (int t = 0; t < k->n_res; t++)
                for (int c = 0; c < k->n_par; c++) {
                    int col = s->col_of[s->gidx[s->blocks[b].gidx_off + (size_t)i * k->n_par + c]];
                    if (col < 0) continue;
                    int row = s->blocks[b].row0 + i * k->n_res + t;
                    es[ne].key = (int64_t)row * ncols + col;
                    es[ne].src = (int32_t)(s->blocks[b].jac_off + ((size_t)i * k->n_res + t) * k->n_par + c);
                    ne++;
                }
    }
    qsort(es, (size_t)ne, sizeof(ent), ent_cmp);
    s->n_ent = ne;
    s->ent_src = (int32_t *)malloc(sizeof(int32_t) * (size_t)(ne ? ne : 1));
    s->ent_slot = (int32_t *)malloc(sizeof(int32_t) * (size_t)(ne ? ne : 1));
    s->csr_indices = (int32_t *)malloc(sizeof(int32_t) * (size_t)(ne ? ne : 1));
    s->csr_indptr = (int32_t *)calloc((size_t)row0 + 1, sizeof(int32_t));
    int nnz = 0;
    for (int e = 0; e < ne; e++) {
        if (e == 0 || es[e].key != es[e - 1].key) {
            s->csr_indices[nnz] = es[e].key % ncols;
            s->csr_indptr[es[e].key / ncols + 1] = nnz + 1;
            nnz++;
        }
        s->ent_src[e] = es[e].src;
        s->ent_slot[e] = nnz - 1;
    }
    for (int i = 1; i <= row0; i++) if (s->csr_indptr[i] < s->csr_indptr[i - 1]) s->csr_indptr[i] = s->csr_indptr[i - 1];
    s->nnz = nnz;
    s->csr_data = (double *)calloc((size_t)(nnz ? nnz : 1), sizeof(double));
    free(es);
    s->ata = NULL;
    return s;
}

void gcs_system_free(gcs_system *s)
{
    if (!s) return;
    free(s->x); free(s->free_idx); free(s->col_of); free(s->blocks);
    free(s->gidx); free(s->consts); free(s->V); free(s->jdata); free(s->r); free(s->hard);
    free(s->ent_src); free(s->ent_slot); free(s->csr_indices);
    free(s->csr_indptr); free(s->csr_data);
    if (s->ata) gcs_ata_free(s->ata);
    free(s);
}

int gcs_system_n_res(const gcs_system *s) { return s->n_res; }
int gcs_system_n_free(const gcs_system *s) { return s->n_free; }
int gcs_system_nnz(const gcs_system *s) { return s->nnz; }
const int32_t *gcs_system_csr_indptr(const gcs_system *s) { return s->csr_indptr; }
const int32_t *gcs_system_csr_indices(const gcs_system *s) { return s->csr_indices; }
const uint8_t *gcs_system_hard(const gcs_system *s) { return s->hard; }

void gcs_system_set_x(gcs_system *s, const double *x) { memcpy(s->x, x, sizeof(double) * (size_t)s->n_params); }
void gcs_system_get_x(const gcs_system *s, double *x) { memcpy(x, s->x, sizeof(double) * (size_t)s->n_params); }
void gcs_system_get_z(const gcs_system *s, double *z)
{
    for (int i = 0; i < s->n_free; i++) z[i] = s->x[s->free_idx[i]];
}
void gcs_system_full_x(const gcs_system *s, const double *z, double *x)
{
    memcpy(x, s->x, sizeof(double) * (size_t)s->n_params);
    for (int i = 0; i < s->n_free; i++) x[s->free_idx[i]] = z[i];
}

void gcs_system_set_consts(gcs_system *s, int block, int row, const double *c)
{
    const gcs_kernel *k = &gcs_kernels()[s->blocks[block].kid];
    if (!k->n_const) return;
    memcpy(s->consts + s->blocks[block].consts_off + (size_t)row * k->n_const, c, sizeof(double) * (size_t)k->n_const);
}

void gcs_system_set_all_consts(gcs_system *s, const double *consts)
{
    if (s->consts_len) memcpy(s->consts, consts, sizeof(double) * (size_t)s->consts_len);
}

/* x[free] <- z, then gather each block's local values and call its kernel. */
void gcs_system_apply_z(gcs_system *s, const double *z)
{
    for (int i = 0; i < s->n_free; i++) s->x[s->free_idx[i]] = z[i];
}

void gcs_system_residuals(gcs_system *s, const double *z, double *r)
{
    const gcs_kernel *KS = gcs_kernels();
    gcs_system_apply_z(s, z);
    for (int b = 0; b < s->n_blocks; b++) {
        const gcs_kernel *k = &KS[s->blocks[b].kid];
        int n = s->blocks[b].count;
        const int32_t *gi = s->gidx + s->blocks[b].gidx_off;
        int len = n * k->n_par;
        for (int t = 0; t < len; t++) s->V[t] = s->x[gi[t]];
        k->res(n, s->V, s->consts + s->blocks[b].consts_off, r + s->blocks[b].row0);
    }
}

static void jac_blocks(gcs_system *s, const double *z)
{
    const gcs_kernel *KS = gcs_kernels();
    gcs_system_apply_z(s, z);
    for (int b = 0; b < s->n_blocks; b++) {
        const gcs_kernel *k = &KS[s->blocks[b].kid];
        if (k->const_jac) continue;
        int n = s->blocks[b].count;
        const int32_t *gi = s->gidx + s->blocks[b].gidx_off;
        int len = n * k->n_par;
        for (int t = 0; t < len; t++) s->V[t] = s->x[gi[t]];
        k->jac(n, s->V, s->consts + s->blocks[b].consts_off, s->jdata + s->blocks[b].jac_off);
    }
}

void gcs_system_csr_data(gcs_system *s, const double *z, double *data)
{
    jac_blocks(s, z);
    memset(data, 0, sizeof(double) * (size_t)(s->nnz ? s->nnz : 1));
    for (int e = 0; e < s->n_ent; e++) data[s->ent_slot[e]] += s->jdata[s->ent_src[e]];
}

void gcs_system_jacobian_dense(gcs_system *s, const double *z, double *J)
{
    if (!s->n_free) return;
    gcs_system_csr_data(s, z, s->csr_data);
    memset(J, 0, sizeof(double) * (size_t)s->n_res * s->n_free);
    for (int r = 0; r < s->n_res; r++)
        for (int p = s->csr_indptr[r]; p < s->csr_indptr[r + 1]; p++)
            J[(size_t)r * s->n_free + s->csr_indices[p]] = s->csr_data[p];
}

double gcs_system_max_hard_residual(gcs_system *s, const double *z)
{
    gcs_system_residuals(s, z, s->r);
    double mx = 0.0;
    for (int i = 0; i < s->n_res; i++) if (s->hard[i]) { double a = fabs(s->r[i]); if (a > mx) mx = a; }
    return mx;
}

void gcs_system_constraint_errors(gcs_system *s, const double *z, double *out)
{
    const gcs_kernel *KS = gcs_kernels();
    gcs_system_residuals(s, z, s->r);
    int ci = 0;
    for (int b = 0; b < s->n_blocks; b++) {
        const gcs_kernel *k = &KS[s->blocks[b].kid];
        for (int i = 0; i < s->blocks[b].count; i++, ci++) {
            double mx = 0.0;
            for (int t = 0; t < k->n_res; t++) {
                double a = fabs(s->r[s->blocks[b].row0 + i * k->n_res + t]);
                if (a > mx) mx = a;
            }
            out[ci] = mx;
        }
    }
}

int gcs_system_rank(gcs_system *s, const double *z, double rcond, int hard_only)
{
    if (!s->n_free || !s->n_res) return 0;
    double *J = (double *)malloc(sizeof(double) * (size_t)s->n_res * s->n_free);
    gcs_system_jacobian_dense(s, z, J);
    int m = s->n_res;
    if (hard_only) {
        int w = 0;
        for (int i = 0; i < s->n_res; i++)
            if (s->hard[i]) {
                if (w != i) memcpy(J + (size_t)w * s->n_free, J + (size_t)i * s->n_free, sizeof(double) * (size_t)s->n_free);
                w++;
            }
        m = w;
    }
    int rank = gcs_rrqr(m, s->n_free, J, rcond, NULL);
    free(J);
    return rank;
}
