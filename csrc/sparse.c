#include "sparse.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* -- pattern construction -------------------------------------------------- */

typedef struct { int32_t r, c, a, b; } trip;

static int trip_cmp(const void *x, const void *y)
{
    const trip *p = (const trip *)x, *q = (const trip *)y;
    if (p->r != q->r) return p->r < q->r ? -1 : 1;
    if (p->c != q->c) return p->c < q->c ? -1 : 1;
    return 0;
}

/* Reverse Cuthill-McKee on the pattern of A: short, and the sketch graphs this solver
 * sees (trusses, chains, rings) are close to banded once relabelled this way. */
static void rcm(int n, const int32_t *ap, const int32_t *ai, int32_t *perm, int32_t *iperm)
{
    int32_t *deg = (int32_t *)malloc(sizeof(int32_t) * (size_t)n);
    uint8_t *seen = (uint8_t *)calloc((size_t)n, 1);
    int32_t *queue = (int32_t *)malloc(sizeof(int32_t) * (size_t)n);
    for (int i = 0; i < n; i++) deg[i] = ap[i + 1] - ap[i];
    int out = 0;
    while (out < n) {
        int start = -1;                       /* lowest-degree unvisited vertex */
        for (int i = 0; i < n; i++) if (!seen[i] && (start < 0 || deg[i] < deg[start])) start = i;
        int head = out, tail = out;
        seen[start] = 1;
        queue[tail++] = start;
        while (head < tail) {
            int v = queue[head++];
            int lo = tail;
            for (int p = ap[v]; p < ap[v + 1]; p++) {
                int w = ai[p];
                if (!seen[w]) { seen[w] = 1; queue[tail++] = w; }
            }
            for (int i = lo; i < tail; i++)   /* neighbours in increasing degree */
                for (int j = i + 1; j < tail; j++)
                    if (deg[queue[j]] < deg[queue[i]]) { int32_t t = queue[i]; queue[i] = queue[j]; queue[j] = t; }
        }
        out = tail;
    }
    for (int i = 0; i < n; i++) perm[i] = queue[n - 1 - i];   /* reversed */
    for (int i = 0; i < n; i++) iperm[perm[i]] = i;
    free(deg); free(seen); free(queue);
}

/* Davis's up-looking LDL^T: symbolic elimination tree and column counts. */
static void ldl_symbolic(int n, const int32_t *ap, const int32_t *ai, int32_t *lp,
                         int32_t *parent, int32_t *lnz, int32_t *flag,
                         const int32_t *perm, const int32_t *iperm)
{
    for (int k = 0; k < n; k++) {
        parent[k] = -1;
        flag[k] = k;
        lnz[k] = 0;
        int kk = perm ? perm[k] : k;
        for (int p = ap[kk]; p < ap[kk + 1]; p++) {
            int i = iperm ? iperm[ai[p]] : ai[p];
            if (i >= k) continue;
            for (; flag[i] != k; i = parent[i]) {
                if (parent[i] == -1) parent[i] = k;
                lnz[i]++;
                flag[i] = k;
            }
        }
    }
    lp[0] = 0;
    for (int k = 0; k < n; k++) lp[k + 1] = lp[k] + lnz[k];
}

static int ldl_numeric(int n, const int32_t *ap, const int32_t *ai, const double *ax,
                       const double *damp, const int32_t *lp, const int32_t *parent,
                       int32_t *lnz, int32_t *li, double *lx, double *d, double *y,
                       int32_t *pattern, int32_t *flag, const int32_t *perm, const int32_t *iperm)
{
    for (int k = 0; k < n; k++) {
        y[k] = 0.0;
        int top = n;
        flag[k] = k;
        lnz[k] = 0;
        int kk = perm ? perm[k] : k;
        for (int p = ap[kk]; p < ap[kk + 1]; p++) {
            int i = iperm ? iperm[ai[p]] : ai[p];
            if (i > k) continue;
            y[i] += ax[p];
            int len = 0;
            for (; flag[i] != k; i = parent[i]) { pattern[len++] = i; flag[i] = k; }
            while (len > 0) pattern[--top] = pattern[--len];
        }
        d[k] = y[k] + (damp ? damp[kk] : 0.0);
        y[k] = 0.0;
        for (; top < n; top++) {
            int i = pattern[top];
            double yi = y[i];
            y[i] = 0.0;
            int p2 = lp[i] + lnz[i];
            for (int p = lp[i]; p < p2; p++) y[li[p]] -= lx[p] * yi;
            double lki = yi / d[i];
            d[k] -= lki * yi;
            li[p2] = k;
            lx[p2] = lki;
            lnz[i]++;
        }
        if (d[k] == 0.0) return k;
    }
    return n;
}

gcs_ata *gcs_ata_new(int n_rows, int n_cols, const int32_t *indptr, const int32_t *indices)
{
    gcs_ata *a = (gcs_ata *)calloc(1, sizeof(gcs_ata));
    a->n = n_cols;
    size_t cap = 0;
    for (int r = 0; r < n_rows; r++) {
        int len = indptr[r + 1] - indptr[r];
        cap += (size_t)len * len;
    }
    trip *tr = (trip *)malloc(sizeof(trip) * (cap ? cap : 1));
    size_t nt = 0;
    for (int r = 0; r < n_rows; r++) {
        for (int p = indptr[r]; p < indptr[r + 1]; p++)
            for (int q = indptr[r]; q < indptr[r + 1]; q++) {
                tr[nt].r = indices[p]; tr[nt].c = indices[q];
                tr[nt].a = p; tr[nt].b = q;
                nt++;
            }
    }
    qsort(tr, nt, sizeof(trip), trip_cmp);
    a->n_tri = (int)nt;
    a->ta = (int32_t *)malloc(sizeof(int32_t) * (nt ? nt : 1));
    a->tb = (int32_t *)malloc(sizeof(int32_t) * (nt ? nt : 1));
    a->ts = (int32_t *)malloc(sizeof(int32_t) * (nt ? nt : 1));
    a->ap = (int32_t *)calloc((size_t)a->n + 1, sizeof(int32_t));
    int32_t *ai = (int32_t *)malloc(sizeof(int32_t) * (nt ? nt : 1));
    int nnz = 0;
    for (size_t t = 0; t < nt; t++) {
        if (t == 0 || tr[t].r != tr[t - 1].r || tr[t].c != tr[t - 1].c) {
            ai[nnz] = tr[t].c;
            a->ap[tr[t].r + 1] = nnz + 1;
            nnz++;
        }
        a->ta[t] = tr[t].a; a->tb[t] = tr[t].b; a->ts[t] = nnz - 1;
    }
    for (int i = 0; i < a->n; i++) if (a->ap[i + 1] < a->ap[i]) a->ap[i + 1] = a->ap[i];
    for (int i = 1; i <= a->n; i++) if (a->ap[i] < a->ap[i - 1]) a->ap[i] = a->ap[i - 1];
    a->nnz = nnz;
    a->ai = (int32_t *)malloc(sizeof(int32_t) * (nnz ? nnz : 1));
    memcpy(a->ai, ai, sizeof(int32_t) * (size_t)nnz);
    a->ax = (double *)calloc((size_t)(nnz ? nnz : 1), sizeof(double));
    free(ai); free(tr);

    /* ordering + symbolic factorization (the pattern never changes) */
    int n = a->n;
    a->perm = (int32_t *)malloc(sizeof(int32_t) * (size_t)(n ? n : 1));
    a->iperm = (int32_t *)malloc(sizeof(int32_t) * (size_t)(n ? n : 1));
    a->parent = (int32_t *)malloc(sizeof(int32_t) * (size_t)(n ? n : 1));
    a->lnz = (int32_t *)malloc(sizeof(int32_t) * (size_t)(n ? n : 1));
    a->flag = (int32_t *)malloc(sizeof(int32_t) * (size_t)(n ? n : 1));
    a->pattern = (int32_t *)malloc(sizeof(int32_t) * (size_t)(n ? n : 1));
    a->lp = (int32_t *)malloc(sizeof(int32_t) * (size_t)(n + 1));
    a->d = (double *)malloc(sizeof(double) * (size_t)(n ? n : 1));
    a->y = (double *)malloc(sizeof(double) * (size_t)(n ? n : 1));
    if (n > 0) {
        rcm(n, a->ap, a->ai, a->perm, a->iperm);
        ldl_symbolic(n, a->ap, a->ai, a->lp, a->parent, a->lnz, a->flag, a->perm, a->iperm);
        a->lnz_total = a->lp[n];
        a->li = (int32_t *)malloc(sizeof(int32_t) * (size_t)(a->lnz_total ? a->lnz_total : 1));
        a->lx = (double *)malloc(sizeof(double) * (size_t)(a->lnz_total ? a->lnz_total : 1));
    } else {
        a->lp[0] = 0;
        a->li = (int32_t *)malloc(sizeof(int32_t));
        a->lx = (double *)malloc(sizeof(double));
    }
    return a;
}

void gcs_ata_free(gcs_ata *a)
{
    if (!a) return;
    free(a->ap); free(a->ai); free(a->ax);
    free(a->ta); free(a->tb); free(a->ts);
    free(a->perm); free(a->iperm); free(a->parent); free(a->lnz);
    free(a->lp); free(a->li); free(a->flag); free(a->pattern);
    free(a->d); free(a->lx); free(a->y);
    free(a);
}

void gcs_ata_fill(gcs_ata *a, const double *jdata)
{
    memset(a->ax, 0, sizeof(double) * (size_t)(a->nnz ? a->nnz : 1));
    for (int t = 0; t < a->n_tri; t++) a->ax[a->ts[t]] += jdata[a->ta[t]] * jdata[a->tb[t]];
}

int gcs_ata_solve(gcs_ata *a, const double *damp, double *b)
{
    int n = a->n;
    if (n == 0) return 0;
    if (ldl_numeric(n, a->ap, a->ai, a->ax, damp, a->lp, a->parent, a->lnz, a->li, a->lx,
                    a->d, a->y, a->pattern, a->flag, a->perm, a->iperm) != n)
        return -1;
    double *x = a->y;
    for (int k = 0; k < n; k++) x[k] = b[a->perm[k]];
    for (int j = 0; j < n; j++)                       /* L y = Pb */
        for (int p = a->lp[j]; p < a->lp[j] + a->lnz[j]; p++) x[a->li[p]] -= a->lx[p] * x[j];
    for (int k = 0; k < n; k++) x[k] /= a->d[k];      /* D */
    for (int j = n - 1; j >= 0; j--)                  /* L^T */
        for (int p = a->lp[j]; p < a->lp[j] + a->lnz[j]; p++) x[j] -= a->lx[p] * x[a->li[p]];
    for (int k = 0; k < n; k++) b[a->perm[k]] = x[k];
    return 0;
}
