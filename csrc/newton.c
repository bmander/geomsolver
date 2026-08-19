/* Powell's DogLeg (default) and Levenberg-Marquardt, both minimising 0.5*||r(z)||^2.
 *
 * The Gauss-Newton step is the *minimum-norm* least-squares solution of J p = -r, so
 * under-constrained sketches (the normal case while editing) move as little as possible.
 * Dense path: the complete orthogonal decomposition in linalg.c, which also reports the
 * rank.  Sparse path: the regularized normal equations (J^T J + eps I) p = -g factored
 * by LDL^T, which keeps rank-deficient systems solvable.
 */
#include "gcs.h"
#include "sparse.h"
#include "system.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define EPS_REL 1e-12

static double vnorm(const double *v, int n)
{
    double s = 0.0;
    for (int i = 0; i < n; i++) s += v[i] * v[i];
    return sqrt(s);
}

static double vdot(const double *a, const double *b, int n)
{
    double s = 0.0;
    for (int i = 0; i < n; i++) s += a[i] * b[i];
    return s;
}

static double absmax(const double *v, int n)
{
    double mx = 0.0;
    for (int i = 0; i < n; i++) { double a = fabs(v[i]); if (a > mx) mx = a; }
    return mx;
}

/* -- the Jacobian, dense or sparse, behind one interface -------------------- */

typedef struct {
    gcs_system *s;
    int dense, m, n;
    double *J;        /* dense m*n */
    double *data;     /* csr values (points at s->csr_data) */
    double *work;     /* n scratch, sparse path only */
    int rank;
} jac_ctx;

static void jac_eval(jac_ctx *c, const double *z)
{
    if (c->dense) gcs_system_jacobian_dense(c->s, z, c->J);
    else gcs_system_csr_data(c->s, z, c->data);
}

static void res_eval(jac_ctx *c, const double *z, double *out)
{
    gcs_system_residuals(c->s, z, out);
}

/* out (n) <- J^T v (m) */
static void jt_mul(const jac_ctx *c, const double *v, double *out)
{
    memset(out, 0, sizeof(double) * (size_t)c->n);
    if (c->dense) {
        for (int i = 0; i < c->m; i++) {
            double vi = v[i];
            if (vi == 0.0) continue;
            const double *row = c->J + (size_t)i * c->n;
            for (int j = 0; j < c->n; j++) out[j] += row[j] * vi;
        }
    } else {
        const int32_t *ip = c->s->csr_indptr, *ii = c->s->csr_indices;
        for (int i = 0; i < c->m; i++) {
            double vi = v[i];
            if (vi == 0.0) continue;
            for (int p = ip[i]; p < ip[i + 1]; p++) out[ii[p]] += c->data[p] * vi;
        }
    }
}

/* out (m) <- J v (n) */
static void j_mul(const jac_ctx *c, const double *v, double *out)
{
    if (c->dense) {
        for (int i = 0; i < c->m; i++) {
            const double *row = c->J + (size_t)i * c->n;
            double s = 0.0;
            for (int j = 0; j < c->n; j++) s += row[j] * v[j];
            out[i] = s;
        }
    } else {
        const int32_t *ip = c->s->csr_indptr, *ii = c->s->csr_indices;
        for (int i = 0; i < c->m; i++) {
            double s = 0.0;
            for (int p = ip[i]; p < ip[i + 1]; p++) s += c->data[p] * v[ii[p]];
            out[i] = s;
        }
    }
}

/* p <- the Gauss-Newton step solving J p ~= -r (minimum norm on the dense path). */
static int gn_step(jac_ctx *c, const double *r, const double *g, double *p)
{
    if (c->dense) {
        double *A = (double *)malloc(sizeof(double) * (size_t)c->m * c->n);
        double *B = (double *)malloc(sizeof(double) * (size_t)(c->m ? c->m : 1));
        memcpy(A, c->J, sizeof(double) * (size_t)c->m * c->n);
        for (int i = 0; i < c->m; i++) B[i] = -r[i];
        c->rank = gcs_min_norm_lstsq(c->m, c->n, 1, A, B, 1e-12, p);
        free(A); free(B);
        return 0;
    }
    if (!c->s->ata) c->s->ata = gcs_ata_new(c->m, c->n, c->s->csr_indptr, c->s->csr_indices);
    gcs_ata *a = c->s->ata;
    gcs_ata_fill(a, c->data);
    gcs_ata_diag(a, c->work);
    double dmax = 0.0;
    for (int i = 0; i < c->n; i++) if (c->work[i] > dmax) dmax = c->work[i];
    double eps = EPS_REL * dmax;
    if (eps <= 0.0) eps = 1e-30;
    for (int i = 0; i < c->n; i++) { c->work[i] = eps; p[i] = -g[i]; }
    c->rank = -1;
    return gcs_ata_solve(a, c->work, p);
}

/* -- DogLeg ----------------------------------------------------------------- */

static int dogleg_core(jac_ctx *c, double *z, double *r, double ftol, double xtol, double gtol, int max_iter, int max_nfev,
                       gcs_info *info)
{
    int m = c->m, n = c->n;
    double *g = (double *)malloc(sizeof(double) * (size_t)n);
    double *p = (double *)malloc(sizeof(double) * (size_t)n);
    double *p_gn = (double *)malloc(sizeof(double) * (size_t)n);
    double *p_sd = (double *)malloc(sizeof(double) * (size_t)n);
    double *z_new = (double *)malloc(sizeof(double) * (size_t)n);
    double *r_new = (double *)malloc(sizeof(double) * (size_t)(m ? m : 1));
    double *tmp = (double *)malloc(sizeof(double) * (size_t)(m ? m : 1));
    int nfev = 1, njev = 0, status = 4, it = 0;
    double delta = INFINITY;
    c->rank = -1;
    for (it = 0; it < max_iter; it++) {
        if (absmax(r, m) < ftol) { status = 0; break; }
        jac_eval(c, z);
        njev++;
        double f = 0.5 * vdot(r, r, m);
        jt_mul(c, r, g);
        if (absmax(g, n) < gtol) { status = 2; break; }
        gn_step(c, r, g, p_gn);
        double gn_norm = vnorm(p_gn, n);
        if (gn_norm <= delta) {
            memcpy(p, p_gn, sizeof(double) * (size_t)n);
        } else {
            j_mul(c, g, tmp);
            double jg = vdot(tmp, tmp, m);
            double alpha = jg > 0.0 ? vdot(g, g, n) / jg : 0.0;
            for (int i = 0; i < n; i++) p_sd[i] = -alpha * g[i];
            double sd_norm = vnorm(p_sd, n);
            if (sd_norm >= delta) {
                double f2 = sd_norm > 0 ? delta / sd_norm : 0.0;
                for (int i = 0; i < n; i++) p[i] = p_sd[i] * f2;
            } else {
                double aa = 0.0, bb = 0.0;
                for (int i = 0; i < n; i++) { double d = p_gn[i] - p_sd[i]; aa += d * d; bb += 2.0 * p_sd[i] * d; }
                double cc = sd_norm * sd_norm - delta * delta;
                double disc = bb * bb - 4.0 * aa * cc;
                double tau = aa > 0.0 ? (-bb + sqrt(disc > 0 ? disc : 0.0)) / (2.0 * aa) : 0.0;
                for (int i = 0; i < n; i++) p[i] = p_sd[i] + tau * (p_gn[i] - p_sd[i]);
            }
        }
        double pnorm = vnorm(p, n);
        if (pnorm < xtol * (1.0 + vnorm(z, n))) { status = 1; break; }
        for (int i = 0; i < n; i++) z_new[i] = z[i] + p[i];
        res_eval(c, z_new, r_new);
        nfev++;
        double f_new = 0.5 * vdot(r_new, r_new, m);
        j_mul(c, p, tmp);
        double lin = 0.0;
        for (int i = 0; i < m; i++) { double v = r[i] + tmp[i]; lin += v * v; }
        double pred = f - 0.5 * lin;
        double rho = pred > 0.0 ? (f - f_new) / pred : (f_new < f ? 1.0 : -1.0);
        if (rho > 0.0) {
            memcpy(z, z_new, sizeof(double) * (size_t)n);
            memcpy(r, r_new, sizeof(double) * (size_t)m);
            if (rho > 0.75) { if (isfinite(delta)) delta = delta > 3 * pnorm ? delta : 3 * pnorm; }
            else if (rho < 0.25) delta = 0.5 * pnorm;
        } else {
            delta = 0.25 * pnorm;
        }
        if (delta < 1e-15 * (1.0 + vnorm(z, n))) { status = 3; it++; break; }
        if (nfev >= max_nfev) { status = 4; it++; break; }
    }
    info->status = status;
    info->nfev = nfev;
    info->njev = njev;
    info->iterations = it;
    info->rank = c->rank;
    free(g); free(p); free(p_gn); free(p_sd); free(z_new); free(r_new); free(tmp);
    return status;
}

/* -- Levenberg-Marquardt (Nielsen damping) ---------------------------------- */

static int lm_core(jac_ctx *c, double *z, double *r, double ftol, double xtol, double gtol, int max_iter, int max_nfev,
                   gcs_info *info)
{
    const double tau0 = 1e-8;
    int m = c->m, n = c->n;
    double *g = (double *)malloc(sizeof(double) * (size_t)n);
    double *p = (double *)malloc(sizeof(double) * (size_t)n);
    double *D = (double *)malloc(sizeof(double) * (size_t)n);
    double *damp = (double *)malloc(sizeof(double) * (size_t)n);
    double *z_new = (double *)malloc(sizeof(double) * (size_t)n);
    double *r_new = (double *)malloc(sizeof(double) * (size_t)(m ? m : 1));
    double *A = c->dense ? (double *)malloc(sizeof(double) * (size_t)n * n) : NULL;
    double *Ad = c->dense ? (double *)malloc(sizeof(double) * (size_t)n * n) : NULL;
    int nfev = 1, njev = 0, status = 4, it = 0;
    double lam = -1.0, nu = 2.0;
    for (it = 0; it < max_iter; it++) {
        if (absmax(r, m) < ftol) { status = 0; break; }
        jac_eval(c, z);
        njev++;
        jt_mul(c, r, g);
        if (absmax(g, n) < gtol) { status = 2; break; }
        if (c->dense) {
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++) {
                    double s = 0.0;
                    for (int k = 0; k < m; k++) s += c->J[(size_t)k * n + i] * c->J[(size_t)k * n + j];
                    A[(size_t)i * n + j] = s;
                }
            for (int i = 0; i < n; i++) D[i] = A[(size_t)i * n + i];
        } else {
            if (!c->s->ata) c->s->ata = gcs_ata_new(m, n, c->s->csr_indptr, c->s->csr_indices);
            gcs_ata_fill(c->s->ata, c->data);
            gcs_ata_diag(c->s->ata, D);
        }
        double dmax = 0.0;
        for (int i = 0; i < n; i++) if (D[i] > dmax) dmax = D[i];
        double floor_ = dmax > 0 ? 1e-8 * dmax : 1e-8;
        for (int i = 0; i < n; i++) if (D[i] < floor_) D[i] = floor_;
        if (lam < 0.0) lam = tau0 * (dmax > 0 ? dmax : 1.0);
        double f = 0.5 * vdot(r, r, m);
        for (;;) {
            for (int i = 0; i < n; i++) damp[i] = lam * D[i];
            int bad = 0;
            if (c->dense) {
                memcpy(Ad, A, sizeof(double) * (size_t)n * n);
                for (int i = 0; i < n; i++) Ad[(size_t)i * n + i] += damp[i];
                for (int i = 0; i < n; i++) p[i] = -g[i];
                bad = gcs_lu_solve(n, Ad, p) != 0;
            } else {
                for (int i = 0; i < n; i++) p[i] = -g[i];
                bad = gcs_ata_solve(c->s->ata, damp, p) != 0;
            }
            if (bad) { lam *= nu; nu *= 2; if (lam > 1e32) { status = 3; goto done; } continue; }
            double pnorm = vnorm(p, n);
            if (pnorm < xtol * (1.0 + vnorm(z, n))) { status = 1; goto done; }
            for (int i = 0; i < n; i++) z_new[i] = z[i] + p[i];
            res_eval(c, z_new, r_new);
            nfev++;
            double f_new = 0.5 * vdot(r_new, r_new, m);
            double pred = 0.0;
            for (int i = 0; i < n; i++) pred += p[i] * (damp[i] * p[i] - g[i]);
            pred *= 0.5;
            double rho = pred > 0.0 ? (f - f_new) / pred : -1.0;
            if (rho > 0.0) {
                memcpy(z, z_new, sizeof(double) * (size_t)n);
                memcpy(r, r_new, sizeof(double) * (size_t)m);
                double t = 1.0 - pow(2.0 * rho - 1.0, 3.0);
                lam *= (t > 1.0 / 3.0 ? t : 1.0 / 3.0);
                nu = 2.0;
                break;
            }
            lam *= nu;
            nu *= 2;
            if (nfev >= max_nfev || lam > 1e32) { status = nfev >= max_nfev ? 4 : 3; it++; goto done; }
        }
    }
done:
    info->status = status;
    info->nfev = nfev;
    info->njev = njev;
    info->iterations = it;
    info->rank = -1;
    free(g); free(p); free(D); free(damp); free(z_new); free(r_new); free(A); free(Ad);
    return status;
}

/* -- entry points ------------------------------------------------------------ */

int gcs_system_solve(gcs_system *s, int method, double ftol, double xtol, double gtol,
                     int max_iter, int max_nfev, int dense, double *z, gcs_info *info)
{
    info->status = 0; info->nfev = 0; info->njev = 0; info->iterations = 0; info->rank = -1;
    if (s->n_free == 0 || s->n_res == 0) { info->nfev = 1; info->status = 0; return 0; }
    if (dense < 0) dense = s->n_free <= 120;
    if (max_nfev <= 0) max_nfev = 4 * max_iter;
    jac_ctx c;
    memset(&c, 0, sizeof(c));
    c.s = s; c.dense = dense; c.m = s->n_res; c.n = s->n_free;
    c.J = dense ? (double *)malloc(sizeof(double) * (size_t)s->n_res * s->n_free) : NULL;
    c.data = s->csr_data;
    c.work = dense ? NULL : (double *)malloc(sizeof(double) * (size_t)s->n_free);
    double *r = (double *)malloc(sizeof(double) * (size_t)s->n_res);
    gcs_system_residuals(s, z, r);
    int st = (method == GCS_LM)
        ? lm_core(&c, z, r, ftol, xtol, gtol, max_iter, max_nfev, info)
        : dogleg_core(&c, z, r, ftol, xtol, gtol, max_iter, max_nfev, info);
    gcs_system_apply_z(s, z);
    free(r); free(c.work); free(c.J);
    return st;
}
