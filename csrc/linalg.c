/* Dense linear algebra: rank-revealing QR, the complete orthogonal decomposition behind
 * the minimum-norm least-squares step, a one-sided Jacobi SVD and an LU solve.
 *
 * Everything is row-major.  The rank convention is the codebase's single one:
 * |R_ii| > rcond * |R_00| after pivoted QR, and sigma_i > rcond * sigma_0 after an SVD.
 */
#include "gcs.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define AT(A, ld, i, j) ((A)[(size_t)(i) * (ld) + (j)])

static double dsign(double a, double b) { return b >= 0.0 ? fabs(a) : -fabs(a); }

/* -- Householder ---------------------------------------------------------- */

/* Reflector zeroing x[1..len-1]: returns beta (the new x[0]) and tau, leaving v[1..] in
 * x[1..] (v[0] = 1 implicit).  x is a strided column. */
static double house_gen(double *x, int len, int stride, double *tau)
{
    double alpha = x[0], xnorm = 0.0;
    for (int i = 1; i < len; i++) { double v = x[i * stride]; xnorm += v * v; }
    if (xnorm == 0.0) { *tau = 0.0; return alpha; }
    xnorm = sqrt(xnorm);
    double beta = -dsign(hypot(alpha, xnorm), alpha);
    *tau = (beta - alpha) / beta;
    double s = 1.0 / (alpha - beta);
    for (int i = 1; i < len; i++) x[i * stride] *= s;
    return beta;
}

/* Householder QR with column pivoting.  A (m*n) is overwritten: R in the upper triangle,
 * the reflectors below it.  `piv` receives the column permutation. */
static void qrp(int m, int n, double *A, double *tau, int32_t *piv, int *rank_out, double rcond)
{
    int k = m < n ? m : n;
    double *cn = (double *)malloc(sizeof(double) * (size_t)n * 2);
    double *cn0 = cn + n;
    for (int j = 0; j < n; j++) {
        double s = 0.0;
        for (int i = 0; i < m; i++) { double v = AT(A, n, i, j); s += v * v; }
        cn[j] = cn0[j] = sqrt(s);
        piv[j] = j;
    }
    for (int p = 0; p < k; p++) {
        int best = p;
        for (int j = p + 1; j < n; j++) if (cn[j] > cn[best]) best = j;
        if (best != p) {
            for (int i = 0; i < m; i++) { double t = AT(A, n, i, p); AT(A, n, i, p) = AT(A, n, i, best); AT(A, n, i, best) = t; }
            double t = cn[p]; cn[p] = cn[best]; cn[best] = t;
            t = cn0[p]; cn0[p] = cn0[best]; cn0[best] = t;
            int32_t ti = piv[p]; piv[p] = piv[best]; piv[best] = ti;
        }
        double beta = house_gen(&AT(A, n, p, p), m - p, n, &tau[p]);
        double t = tau[p];
        if (t != 0.0) {
            for (int j = p + 1; j < n; j++) {
                double w = AT(A, n, p, j);
                for (int i = p + 1; i < m; i++) w += AT(A, n, i, p) * AT(A, n, i, j);
                w *= t;
                AT(A, n, p, j) -= w;
                for (int i = p + 1; i < m; i++) AT(A, n, i, j) -= w * AT(A, n, i, p);
            }
        }
        AT(A, n, p, p) = beta;
        /* downdate the trailing column norms, recomputing when cancellation bites */
        for (int j = p + 1; j < n; j++) {
            if (cn[j] == 0.0) continue;
            double r = AT(A, n, p, j) / cn[j];
            double f = 1.0 - r * r;
            f = f < 0.0 ? 0.0 : f;
            double g = cn[j] / (cn0[j] > 0 ? cn0[j] : 1.0);
            if (f * g * g < 1e-8) {
                double s = 0.0;
                for (int i = p + 1; i < m; i++) { double v = AT(A, n, i, j); s += v * v; }
                cn[j] = cn0[j] = sqrt(s);
            } else {
                cn[j] *= sqrt(f);
            }
        }
    }
    int rank = 0;
    if (k > 0) {
        double d0 = fabs(AT(A, n, 0, 0));
        if (d0 > 0.0) for (int i = 0; i < k; i++) if (fabs(AT(A, n, i, i)) > rcond * d0) rank++;
    }
    *rank_out = rank;
    free(cn);
}

/* B (m*nrhs) <- Q^T B, using the reflectors left in A by qrp. */
static void apply_qt(int m, int n, int k, const double *A, const double *tau, double *B, int nrhs)
{
    for (int p = 0; p < k; p++) {
        double t = tau[p];
        if (t == 0.0) continue;
        for (int j = 0; j < nrhs; j++) {
            double w = AT(B, nrhs, p, j);
            for (int i = p + 1; i < m; i++) w += AT(A, n, i, p) * AT(B, nrhs, i, j);
            w *= t;
            AT(B, nrhs, p, j) -= w;
            for (int i = p + 1; i < m; i++) AT(B, nrhs, i, j) -= w * AT(A, n, i, p);
        }
    }
}

int gcs_rrqr(int m, int n, double *A, double rcond, int32_t *piv)
{
    if (m <= 0 || n <= 0) return 0;
    int k = m < n ? m : n;
    double *tau = (double *)malloc(sizeof(double) * (size_t)k);
    int32_t *pv = piv ? piv : (int32_t *)malloc(sizeof(int32_t) * (size_t)n);
    int rank = 0;
    qrp(m, n, A, tau, pv, &rank, rcond);
    free(tau);
    if (!piv) free(pv);
    return rank;
}

/* RZ factorization of the k*n trapezoid in A (k <= n): [R11 R12] Z = [T11 0].
 * Row i keeps its reflector's tail in A[i, k..n-1]; ztau[i] is its scalar. */
static void tzrz(int k, int n, double *A, double *ztau)
{
    int nz = n - k;
    if (nz <= 0) return;
    for (int i = k - 1; i >= 0; i--) {
        /* the reflector acts on ( A[i][i], A[i][k..n-1] ) */
        double alpha = AT(A, n, i, i), xnorm = 0.0;
        for (int j = 0; j < nz; j++) { double v = AT(A, n, i, k + j); xnorm += v * v; }
        if (xnorm == 0.0) { ztau[i] = 0.0; continue; }
        xnorm = sqrt(xnorm);
        double beta = -dsign(hypot(alpha, xnorm), alpha);
        double t = (beta - alpha) / beta;
        double s = 1.0 / (alpha - beta);
        for (int j = 0; j < nz; j++) AT(A, n, i, k + j) *= s;
        ztau[i] = t;
        AT(A, n, i, i) = beta;
        for (int r = 0; r < i; r++) {           /* apply from the right to the rows above */
            double w = AT(A, n, r, i);
            for (int j = 0; j < nz; j++) w += AT(A, n, r, k + j) * AT(A, n, i, k + j);
            w *= t;
            AT(A, n, r, i) -= w;
            for (int j = 0; j < nz; j++) AT(A, n, r, k + j) -= w * AT(A, n, i, k + j);
        }
    }
}

/* y <- Z^T y.  tzrz built [T 0] = R * (H(k-1)...H(0)), so Z^T = H(k-1)...H(0) and the
 * reflectors apply in increasing order. */
static void apply_zt(int k, int n, const double *A, const double *ztau, double *y)
{
    int nz = n - k;
    if (nz <= 0) return;
    for (int i = 0; i < k; i++) {
        double t = ztau[i];
        if (t == 0.0) continue;
        double w = y[i];
        for (int j = 0; j < nz; j++) w += AT(A, n, i, k + j) * y[k + j];
        w *= t;
        y[i] -= w;
        for (int j = 0; j < nz; j++) y[k + j] -= w * AT(A, n, i, k + j);
    }
}

int gcs_min_norm_lstsq(int m, int n, int nrhs, double *A, double *B, double rcond, double *X)
{
    if (n <= 0 || nrhs <= 0) return 0;
    memset(X, 0, sizeof(double) * (size_t)n * nrhs);
    if (m <= 0) return 0;
    int k = m < n ? m : n;
    double *tau = (double *)malloc(sizeof(double) * (size_t)k);
    double *ztau = (double *)calloc((size_t)(k > 0 ? k : 1), sizeof(double));
    int32_t *piv = (int32_t *)malloc(sizeof(int32_t) * (size_t)n);
    int rank = 0;
    qrp(m, n, A, tau, piv, &rank, rcond);
    apply_qt(m, n, k, A, tau, B, nrhs);
    if (rank > 0) {
        tzrz(rank, n, A, ztau);
        double *y = (double *)malloc(sizeof(double) * (size_t)n);
        for (int c = 0; c < nrhs; c++) {
            for (int i = 0; i < n; i++) y[i] = 0.0;
            for (int i = rank - 1; i >= 0; i--) {      /* T11 is upper triangular */
                double s = AT(B, nrhs, i, c);
                for (int j = i + 1; j < rank; j++) s -= AT(A, n, i, j) * y[j];
                y[i] = s / AT(A, n, i, i);
            }
            apply_zt(rank, n, A, ztau, y);
            for (int i = 0; i < n; i++) AT(X, nrhs, piv[i], c) = y[i];
        }
        free(y);
    }
    free(tau); free(ztau); free(piv);
    return rank;
}

/* -- SVD (Golub-Reinsch) ---------------------------------------------------- */

/* Householder bidiagonalization followed by an implicit-shift QR sweep on the bidiagonal —
 * the classic algorithm, chosen over one-sided Jacobi because diagnosis SVDs a Jacobian on
 * every edit and Jacobi's sweep count makes that quadratically too slow at sketch sizes.
 *
 * A (m*n, m >= n) is overwritten with U when `want_u`; `w` receives the singular values in
 * bidiagonalization order and V (n*n) the right factor.  Returns 0 on success. */
static double pythag(double a, double b)
{
    double aa = fabs(a), ab = fabs(b);
    if (aa > ab) { double t = ab / aa; return aa * sqrt(1.0 + t * t); }
    if (ab == 0.0) return 0.0;
    double t = aa / ab;
    return ab * sqrt(1.0 + t * t);
}

static int gr_svd(int m, int n, double *A, double *w, double *V, int want_u)
{
    double *rv1 = (double *)calloc((size_t)n, sizeof(double));
    double g = 0.0, scale = 0.0, anorm = 0.0;
    int l = 0;

    for (int i = 0; i < n; i++) {
        l = i + 1;
        rv1[i] = scale * g;
        g = scale = 0.0;
        double s = 0.0;
        if (i < m) {
            for (int k = i; k < m; k++) scale += fabs(AT(A, n, k, i));
            if (scale != 0.0) {
                for (int k = i; k < m; k++) { AT(A, n, k, i) /= scale; s += AT(A, n, k, i) * AT(A, n, k, i); }
                double f = AT(A, n, i, i);
                g = -dsign(sqrt(s), f);
                double h = f * g - s;
                AT(A, n, i, i) = f - g;
                for (int j = l; j < n; j++) {
                    double ss = 0.0;
                    for (int k = i; k < m; k++) ss += AT(A, n, k, i) * AT(A, n, k, j);
                    double ff = ss / h;
                    for (int k = i; k < m; k++) AT(A, n, k, j) += ff * AT(A, n, k, i);
                }
                for (int k = i; k < m; k++) AT(A, n, k, i) *= scale;
            }
        }
        w[i] = scale * g;
        g = scale = 0.0;
        s = 0.0;
        if (i < m && i != n - 1) {
            for (int k = l; k < n; k++) scale += fabs(AT(A, n, i, k));
            if (scale != 0.0) {
                for (int k = l; k < n; k++) { AT(A, n, i, k) /= scale; s += AT(A, n, i, k) * AT(A, n, i, k); }
                double f = AT(A, n, i, l);
                g = -dsign(sqrt(s), f);
                double h = f * g - s;
                AT(A, n, i, l) = f - g;
                for (int k = l; k < n; k++) rv1[k] = AT(A, n, i, k) / h;
                for (int j = l; j < m; j++) {
                    double ss = 0.0;
                    for (int k = l; k < n; k++) ss += AT(A, n, j, k) * AT(A, n, i, k);
                    for (int k = l; k < n; k++) AT(A, n, j, k) += ss * rv1[k];
                }
                for (int k = l; k < n; k++) AT(A, n, i, k) *= scale;
            }
        }
        double a = fabs(w[i]) + fabs(rv1[i]);
        if (a > anorm) anorm = a;
    }
    /* right-hand transformations */
    for (int i = n - 1; i >= 0; i--) {
        if (i < n - 1) {
            if (g != 0.0) {
                for (int j = l; j < n; j++) AT(V, n, j, i) = (AT(A, n, i, j) / AT(A, n, i, l)) / g;
                for (int j = l; j < n; j++) {
                    double s = 0.0;
                    for (int k = l; k < n; k++) s += AT(A, n, i, k) * AT(V, n, k, j);
                    for (int k = l; k < n; k++) AT(V, n, k, j) += s * AT(V, n, k, i);
                }
            }
            for (int j = l; j < n; j++) AT(V, n, i, j) = AT(V, n, j, i) = 0.0;
        }
        AT(V, n, i, i) = 1.0;
        g = rv1[i];
        l = i;
    }
    /* left-hand transformations */
    if (want_u) {
        for (int i = (m < n ? m : n) - 1; i >= 0; i--) {
            l = i + 1;
            g = w[i];
            for (int j = l; j < n; j++) AT(A, n, i, j) = 0.0;
            if (g != 0.0) {
                g = 1.0 / g;
                for (int j = l; j < n; j++) {
                    double s = 0.0;
                    for (int k = l; k < m; k++) s += AT(A, n, k, i) * AT(A, n, k, j);
                    double f = (s / AT(A, n, i, i)) * g;
                    for (int k = i; k < m; k++) AT(A, n, k, j) += f * AT(A, n, k, i);
                }
                for (int j = i; j < m; j++) AT(A, n, j, i) *= g;
            } else {
                for (int j = i; j < m; j++) AT(A, n, j, i) = 0.0;
            }
            AT(A, n, i, i) += 1.0;
        }
    }
    /* diagonalize the bidiagonal form */
    for (int k = n - 1; k >= 0; k--) {
        for (int its = 0; its < 60; its++) {
            int flag = 1, nm = 0;
            for (l = k; l >= 0; l--) {
                nm = l - 1;
                if (fabs(rv1[l]) + anorm == anorm) { flag = 0; break; }
                if (fabs(w[nm]) + anorm == anorm) break;
            }
            if (flag) {                       /* cancel rv1[l] with Givens rotations */
                double c = 0.0, s = 1.0;
                for (int i = l; i <= k; i++) {
                    double f = s * rv1[i];
                    rv1[i] = c * rv1[i];
                    if (fabs(f) + anorm == anorm) break;
                    g = w[i];
                    double h = pythag(f, g);
                    w[i] = h;
                    h = 1.0 / h;
                    c = g * h;
                    s = -f * h;
                    if (want_u) {
                        for (int j = 0; j < m; j++) {
                            double y = AT(A, n, j, nm), z = AT(A, n, j, i);
                            AT(A, n, j, nm) = y * c + z * s;
                            AT(A, n, j, i) = z * c - y * s;
                        }
                    }
                }
            }
            double z = w[k];
            if (l == k) {                     /* converged */
                if (z < 0.0) {
                    w[k] = -z;
                    for (int j = 0; j < n; j++) AT(V, n, j, k) = -AT(V, n, j, k);
                }
                break;
            }
            if (its == 59) { free(rv1); return -1; }
            double x = w[l];
            nm = k - 1;
            double y = w[nm], h = rv1[k];
            g = rv1[nm];
            double f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = pythag(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + dsign(g, f))) - h)) / x;
            double c = 1.0, s = 1.0;
            for (int j = l; j <= nm; j++) {
                int i = j + 1;
                g = rv1[i];
                y = w[i];
                h = s * g;
                g = c * g;
                z = pythag(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y *= c;
                for (int jj = 0; jj < n; jj++) {
                    double xx = AT(V, n, jj, j), zz = AT(V, n, jj, i);
                    AT(V, n, jj, j) = xx * c + zz * s;
                    AT(V, n, jj, i) = zz * c - xx * s;
                }
                z = pythag(f, h);
                w[j] = z;
                if (z != 0.0) { z = 1.0 / z; c = f * z; s = h * z; }
                f = c * g + s * y;
                x = c * y - s * g;
                if (want_u) {
                    for (int jj = 0; jj < m; jj++) {
                        double yy = AT(A, n, jj, j), zz = AT(A, n, jj, i);
                        AT(A, n, jj, j) = yy * c + zz * s;
                        AT(A, n, jj, i) = zz * c - yy * s;
                    }
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = x;
        }
    }
    free(rv1);
    return 0;
}

int gcs_svd(int m, int n, const double *A, double *U, double *S, double *Vt)
{
    if (m <= 0 || n <= 0) return -1;
    int mn = m < n ? m : n;
    /* A wide matrix is padded with zero rows: the algorithm wants m >= n, and the padding
     * leaves the singular values alone while still producing the full n*n right factor
     * (whose trailing columns are the null space callers ask for). */
    int mm = m > n ? m : n;
    double *W = (double *)calloc((size_t)mm * n, sizeof(double));
    for (int i = 0; i < m; i++) memcpy(W + (size_t)i * n, A + (size_t)i * n, sizeof(double) * (size_t)n);
    double *V = (double *)calloc((size_t)n * n, sizeof(double));
    double *w = (double *)calloc((size_t)n, sizeof(double));
    int info = gr_svd(mm, n, W, w, V, U != NULL);

    int32_t *ord = (int32_t *)malloc(sizeof(int32_t) * (size_t)n);
    for (int j = 0; j < n; j++) ord[j] = j;
    for (int i = 0; i < n - 1; i++) {              /* descending */
        int b = i;
        for (int j = i + 1; j < n; j++) if (w[ord[j]] > w[ord[b]]) b = j;
        int32_t t = ord[i]; ord[i] = ord[b]; ord[b] = t;
    }
    for (int i = 0; i < mn; i++) S[i] = w[ord[i]];
    if (Vt) for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) AT(Vt, n, i, j) = AT(V, n, j, ord[i]);
    if (U) for (int c = 0; c < mn; c++) for (int i = 0; i < m; i++) AT(U, mn, i, c) = AT(W, n, i, ord[c]);
    free(W); free(V); free(w); free(ord);
    return info;
}

int gcs_rank_nullspace(int m, int n, const double *A, double rcond, double *N, double *S)
{
    if (n <= 0) return 0;
    double *sv = S ? S : (double *)malloc(sizeof(double) * (size_t)n);
    double *Vt = (double *)malloc(sizeof(double) * (size_t)n * n);
    int mn = m < n ? m : n;
    if (m <= 0) {
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) AT(N, n, i, j) = (i == j) ? 1.0 : 0.0;
        if (S) for (int i = 0; i < n; i++) S[i] = 0.0;
        free(Vt); if (!S) free(sv);
        return 0;
    }
    gcs_svd(m, n, A, NULL, sv, Vt);
    int rank = 0;
    if (mn > 0 && sv[0] > 0.0) for (int i = 0; i < mn; i++) if (sv[i] > rcond * sv[0]) rank++;
    /* null space = rows rank..n-1 of Vt, delivered as columns of N (n * (n - rank)) */
    int nn = n - rank;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < nn; j++) AT(N, nn > 0 ? nn : 1, i, j) = AT(Vt, n, rank + j, i);
    free(Vt); if (!S) free(sv);
    return rank;
}

int gcs_lu_solve(int n, double *A, double *b)
{
    for (int k = 0; k < n; k++) {
        int p = k;
        for (int i = k + 1; i < n; i++) if (fabs(AT(A, n, i, k)) > fabs(AT(A, n, p, k))) p = i;
        if (AT(A, n, p, k) == 0.0) return -1;
        if (p != k) {
            for (int j = 0; j < n; j++) { double t = AT(A, n, k, j); AT(A, n, k, j) = AT(A, n, p, j); AT(A, n, p, j) = t; }
            double t = b[k]; b[k] = b[p]; b[p] = t;
        }
        double piv = AT(A, n, k, k);
        for (int i = k + 1; i < n; i++) {
            double f = AT(A, n, i, k) / piv;
            if (f == 0.0) continue;
            AT(A, n, i, k) = 0.0;
            for (int j = k + 1; j < n; j++) AT(A, n, i, j) -= f * AT(A, n, k, j);
            b[i] -= f * b[k];
        }
    }
    for (int i = n - 1; i >= 0; i--) {
        double s = b[i];
        for (int j = i + 1; j < n; j++) s -= AT(A, n, i, j) * b[j];
        b[i] = s / AT(A, n, i, i);
    }
    return 0;
}
