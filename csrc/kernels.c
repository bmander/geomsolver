/* Residual / Jacobian kernels — one per constraint type, evaluated for a whole block
 * of same-typed constraints per call.
 *
 * V is (n * n_par) local parameter values, K is (n * n_const) constants,
 * R is (n * n_res) residuals and J is (n * n_res * n_par).  Column conventions match
 * the `params` tuples the front end builds; see the comment above each kernel.
 * Residual forms follow the program: squared distances (no sqrt), a determinant for
 * parallel, dot/cross for angle, signed distance minus radius for tangency.
 */
#include "gcs.h"

#include <math.h>

/* -- linear kernels: r = J v with a constant J ----------------------------- */

static void lin_res(int n, const double *V, const double *J, int n_res, int n_par, double *R)
{
    for (int i = 0; i < n; i++) {
        const double *v = V + (size_t)i * n_par;
        for (int r = 0; r < n_res; r++) {
            double s = 0.0;
            for (int c = 0; c < n_par; c++) s += J[r * n_par + c] * v[c];
            R[(size_t)i * n_res + r] = s;
        }
    }
}

static void lin_jac(int n, const double *J, int n_res, int n_par, double *out)
{
    size_t sz = (size_t)n_res * n_par;
    for (int i = 0; i < n; i++)
        for (size_t t = 0; t < sz; t++) out[(size_t)i * sz + t] = J[t];
}

#define LINEAR_KERNEL(name, NRES, NPAR, ...)                                              \
    static const double name##_J[(NRES) * (NPAR)] = {__VA_ARGS__};                        \
    static void name##_res(int n, const double *V, const double *K, double *R)            \
    { (void)K; lin_res(n, V, name##_J, NRES, NPAR, R); }                                  \
    static void name##_jac(int n, const double *V, const double *K, double *J)            \
    { (void)V; (void)K; lin_jac(n, name##_J, NRES, NPAR, J); }

/* (px,py,qx,qy) */
LINEAR_KERNEL(coincident, 2, 4, 1, 0, -1, 0, 0, 1, 0, -1)
/* (px,py,ax,ay,bx,by) */
LINEAR_KERNEL(midpoint, 2, 6, 2, 0, -1, 0, -1, 0, 0, 2, 0, -1, 0, -1)
/* (ax,ay,bx,by): ay - by */
LINEAR_KERNEL(horizontal, 1, 4, 0, 1, 0, -1)
/* (ax,ay,bx,by): ax - bx */
LINEAR_KERNEL(vertical, 1, 4, 1, 0, -1, 0)
/* (r1,r2) */
LINEAR_KERNEL(equal_radius, 1, 2, 1, -1)

/* -- point / point --------------------------------------------------------- */

/* (px,py,qx,qy), K = (d): |p-q|^2 - d^2 */
static void distance_res(int n, const double *V, const double *K, double *R)
{
    for (int i = 0; i < n; i++) {
        const double *v = V + 4 * (size_t)i;
        double dx = v[0] - v[2], dy = v[1] - v[3], d = K[i];
        R[i] = dx * dx + dy * dy - d * d;
    }
}

static void distance_jac(int n, const double *V, const double *K, double *J)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 4 * (size_t)i;
        double *j = J + 4 * (size_t)i;
        double dx = 2.0 * (v[0] - v[2]), dy = 2.0 * (v[1] - v[3]);
        j[0] = dx; j[1] = dy; j[2] = -dx; j[3] = -dy;
    }
}

/* (px,py), K = (tx,ty,w): the soft drag target */
static void drag_res(int n, const double *V, const double *K, double *R)
{
    for (int i = 0; i < n; i++) {
        const double *v = V + 2 * (size_t)i;
        const double *k = K + 3 * (size_t)i;
        R[2 * (size_t)i] = k[2] * (v[0] - k[0]);
        R[2 * (size_t)i + 1] = k[2] * (v[1] - k[1]);
    }
}

static void drag_jac(int n, const double *V, const double *K, double *J)
{
    (void)V;
    for (int i = 0; i < n; i++) {
        double *j = J + 4 * (size_t)i;
        double w = K[3 * (size_t)i + 2];
        j[0] = w; j[1] = 0.0; j[2] = 0.0; j[3] = w;
    }
}

/* -- line orientation ------------------------------------------------------ */
/* (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y) */

#define DIRS(v)                                                     \
    double d1x = (v)[2] - (v)[0], d1y = (v)[3] - (v)[1];            \
    double d2x = (v)[6] - (v)[4], d2y = (v)[7] - (v)[5];

static void cross_jac(const double *v, double *j)
{
    DIRS(v)
    j[0] = -d2y; j[1] = d2x; j[2] = d2y; j[3] = -d2x;
    j[4] = d1y;  j[5] = -d1x; j[6] = -d1y; j[7] = d1x;
}

static void dot_jac(const double *v, double *j)
{
    DIRS(v)
    j[0] = -d2x; j[1] = -d2y; j[2] = d2x; j[3] = d2y;
    j[4] = -d1x; j[5] = -d1y; j[6] = d1x; j[7] = d1y;
}

static void parallel_res(int n, const double *V, const double *K, double *R)
{
    (void)K;
    for (int i = 0; i < n; i++) { const double *v = V + 8 * (size_t)i; DIRS(v) R[i] = d1x * d2y - d1y * d2x; }
}

static void parallel_jac(int n, const double *V, const double *K, double *J)
{
    (void)K;
    for (int i = 0; i < n; i++) cross_jac(V + 8 * (size_t)i, J + 8 * (size_t)i);
}

static void perpendicular_res(int n, const double *V, const double *K, double *R)
{
    (void)K;
    for (int i = 0; i < n; i++) { const double *v = V + 8 * (size_t)i; DIRS(v) R[i] = d1x * d2x + d1y * d2y; }
}

static void perpendicular_jac(int n, const double *V, const double *K, double *J)
{
    (void)K;
    for (int i = 0; i < n; i++) dot_jac(V + 8 * (size_t)i, J + 8 * (size_t)i);
}

/* K = (sin theta, cos theta): dot*sin - cross*cos */
static void angle_res(int n, const double *V, const double *K, double *R)
{
    for (int i = 0; i < n; i++) {
        const double *v = V + 8 * (size_t)i;
        DIRS(v)
        R[i] = (d1x * d2x + d1y * d2y) * K[2 * (size_t)i] - (d1x * d2y - d1y * d2x) * K[2 * (size_t)i + 1];
    }
}

static void angle_jac(int n, const double *V, const double *K, double *J)
{
    double jd[8], jc[8];
    for (int i = 0; i < n; i++) {
        const double *v = V + 8 * (size_t)i;
        double *j = J + 8 * (size_t)i;
        double s = K[2 * (size_t)i], c = K[2 * (size_t)i + 1];
        dot_jac(v, jd);
        cross_jac(v, jc);
        for (int t = 0; t < 8; t++) j[t] = jd[t] * s - jc[t] * c;
    }
}

static void equal_length_res(int n, const double *V, const double *K, double *R)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 8 * (size_t)i;
        DIRS(v)
        R[i] = d1x * d1x + d1y * d1y - d2x * d2x - d2y * d2y;
    }
}

static void equal_length_jac(int n, const double *V, const double *K, double *J)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 8 * (size_t)i;
        double *j = J + 8 * (size_t)i;
        DIRS(v)
        j[0] = -2 * d1x; j[1] = -2 * d1y; j[2] = 2 * d1x; j[3] = 2 * d1y;
        j[4] = 2 * d2x;  j[5] = 2 * d2y;  j[6] = -2 * d2x; j[7] = -2 * d2y;
    }
}

/* -- incidence ------------------------------------------------------------- */

/* (px,py,ax,ay,bx,by): (b-a) x (p-a) */
static void point_on_line_res(int n, const double *V, const double *K, double *R)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 6 * (size_t)i;
        double dx = v[4] - v[2], dy = v[5] - v[3], wx = v[0] - v[2], wy = v[1] - v[3];
        R[i] = dx * wy - dy * wx;
    }
}

static void point_on_line_jac(int n, const double *V, const double *K, double *J)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 6 * (size_t)i;
        double *j = J + 6 * (size_t)i;
        double dx = v[4] - v[2], dy = v[5] - v[3], wx = v[0] - v[2], wy = v[1] - v[3];
        j[0] = -dy; j[1] = dx; j[2] = dy - wy; j[3] = wx - dx; j[4] = wy; j[5] = -wx;
    }
}

/* (px,py,cx,cy,r): |p-c|^2 - r^2 */
static void point_on_circle_res(int n, const double *V, const double *K, double *R)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 5 * (size_t)i;
        double ux = v[0] - v[2], uy = v[1] - v[3];
        R[i] = ux * ux + uy * uy - v[4] * v[4];
    }
}

static void point_on_circle_jac(int n, const double *V, const double *K, double *J)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 5 * (size_t)i;
        double *j = J + 5 * (size_t)i;
        double ux = v[0] - v[2], uy = v[1] - v[3];
        j[0] = 2 * ux; j[1] = 2 * uy; j[2] = -2 * ux; j[3] = -2 * uy; j[4] = -2 * v[4];
    }
}

/* -- radii ----------------------------------------------------------------- */

static const double radius_J[1] = {1.0};

static void radius_res(int n, const double *V, const double *K, double *R)
{
    for (int i = 0; i < n; i++) R[i] = V[i] - K[i];
}

static void radius_jac(int n, const double *V, const double *K, double *J)
{
    (void)V; (void)K;
    for (int i = 0; i < n; i++) J[i] = 1.0;
}

/* -- tangency -------------------------------------------------------------- */

/* (ax,ay,bx,by,cx,cy,r), K = (side): cross(b-a, c-a)/|b-a| - side*r */
static void tangent_line_circle_res(int n, const double *V, const double *K, double *R)
{
    for (int i = 0; i < n; i++) {
        const double *v = V + 7 * (size_t)i;
        double dx = v[2] - v[0], dy = v[3] - v[1], wx = v[4] - v[0], wy = v[5] - v[1];
        double L = hypot(dx, dy), C = dx * wy - dy * wx;
        R[i] = C / L - K[i] * v[6];
    }
}

static void tangent_line_circle_jac(int n, const double *V, const double *K, double *J)
{
    for (int i = 0; i < n; i++) {
        const double *v = V + 7 * (size_t)i;
        double *j = J + 7 * (size_t)i;
        double dx = v[2] - v[0], dy = v[3] - v[1], wx = v[4] - v[0], wy = v[5] - v[1];
        double L = hypot(dx, dy), C = dx * wy - dy * wx;
        double dC[7] = {dy - wy, wx - dx, wy, -wx, -dy, dx, 0.0};
        double dL[7] = {-dx / L, -dy / L, dx / L, dy / L, 0.0, 0.0, 0.0};
        double f = C / (L * L);
        for (int t = 0; t < 7; t++) j[t] = dC[t] / L - f * dL[t];
        j[6] = -K[i];
    }
}

/* (c1x,c1y,r1,c2x,c2y,r2), K = (sign): +1 external, -1 internal */
static void tangent_circle_circle_res(int n, const double *V, const double *K, double *R)
{
    for (int i = 0; i < n; i++) {
        const double *v = V + 6 * (size_t)i;
        double ux = v[0] - v[3], uy = v[1] - v[4], Rr = v[2] + K[i] * v[5];
        R[i] = ux * ux + uy * uy - Rr * Rr;
    }
}

static void tangent_circle_circle_jac(int n, const double *V, const double *K, double *J)
{
    for (int i = 0; i < n; i++) {
        const double *v = V + 6 * (size_t)i;
        double *j = J + 6 * (size_t)i;
        double ux = v[0] - v[3], uy = v[1] - v[4], Rr = v[2] + K[i] * v[5];
        j[0] = 2 * ux; j[1] = 2 * uy; j[2] = -2 * Rr;
        j[3] = -2 * ux; j[4] = -2 * uy; j[5] = -2 * Rr * K[i];
    }
}

/* (px,py,cx,cy,ax,ay,bx,by): (p-c).(b-a) */
static void tangent_arc_line_res(int n, const double *V, const double *K, double *R)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 8 * (size_t)i;
        R[i] = (v[0] - v[2]) * (v[6] - v[4]) + (v[1] - v[3]) * (v[7] - v[5]);
    }
}

static void tangent_arc_line_jac(int n, const double *V, const double *K, double *J)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 8 * (size_t)i;
        double *j = J + 8 * (size_t)i;
        double ux = v[0] - v[2], uy = v[1] - v[3], dx = v[6] - v[4], dy = v[7] - v[5];
        j[0] = dx; j[1] = dy; j[2] = -dx; j[3] = -dy;
        j[4] = -ux; j[5] = -uy; j[6] = ux; j[7] = uy;
    }
}

/* -- symmetry -------------------------------------------------------------- */

/* (px,py,qx,qy,ax,ay,bx,by): p and q mirror each other across the line a->b.
 * Two residuals: the midpoint lies on the line (written as p + q - 2a to avoid the halving),
 * and p->q is perpendicular to it. */
static void symmetric_res(int n, const double *V, const double *K, double *R)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 8 * (size_t)i;
        double dx = v[6] - v[4], dy = v[7] - v[5];
        double mx = v[0] + v[2] - 2 * v[4], my = v[1] + v[3] - 2 * v[5];
        R[2 * (size_t)i] = dx * my - dy * mx;
        R[2 * (size_t)i + 1] = (v[2] - v[0]) * dx + (v[3] - v[1]) * dy;
    }
}

static void symmetric_jac(int n, const double *V, const double *K, double *J)
{
    (void)K;
    for (int i = 0; i < n; i++) {
        const double *v = V + 8 * (size_t)i;
        double *j = J + 16 * (size_t)i;
        double dx = v[6] - v[4], dy = v[7] - v[5];
        double mx = v[0] + v[2] - 2 * v[4], my = v[1] + v[3] - 2 * v[5];
        double ex = v[2] - v[0], ey = v[3] - v[1];
        j[0] = -dy; j[1] = dx; j[2] = -dy; j[3] = dx;
        j[4] = 2 * dy - my; j[5] = mx - 2 * dx; j[6] = my; j[7] = -mx;
        j[8] = -dx; j[9] = -dy; j[10] = dx; j[11] = dy;
        j[12] = -ex; j[13] = -ey; j[14] = ex; j[15] = ey;
    }
}

/* -- registry (order == kernel id, shared with the front end) --------------- */

static const gcs_kernel KERNELS[GCS_N_KERNELS] = {
    {"coincident", 2, 4, 0, coincident_res, coincident_jac, coincident_J},
    {"distance", 1, 4, 1, distance_res, distance_jac, NULL},
    {"midpoint", 2, 6, 0, midpoint_res, midpoint_jac, midpoint_J},
    {"drag", 2, 2, 3, drag_res, drag_jac, NULL},
    {"horizontal", 1, 4, 0, horizontal_res, horizontal_jac, horizontal_J},
    {"vertical", 1, 4, 0, vertical_res, vertical_jac, vertical_J},
    {"parallel", 1, 8, 0, parallel_res, parallel_jac, NULL},
    {"perpendicular", 1, 8, 0, perpendicular_res, perpendicular_jac, NULL},
    {"angle", 1, 8, 2, angle_res, angle_jac, NULL},
    {"equal_length", 1, 8, 0, equal_length_res, equal_length_jac, NULL},
    {"point_on_line", 1, 6, 0, point_on_line_res, point_on_line_jac, NULL},
    {"point_on_circle", 1, 5, 0, point_on_circle_res, point_on_circle_jac, NULL},
    {"radius", 1, 1, 1, radius_res, radius_jac, radius_J},
    {"equal_radius", 1, 2, 0, equal_radius_res, equal_radius_jac, equal_radius_J},
    {"tangent_line_circle", 1, 7, 1, tangent_line_circle_res, tangent_line_circle_jac, NULL},
    {"tangent_circle_circle", 1, 6, 1, tangent_circle_circle_res, tangent_circle_circle_jac, NULL},
    {"tangent_arc_line", 1, 8, 0, tangent_arc_line_res, tangent_arc_line_jac, NULL},
    {"symmetric", 2, 8, 0, symmetric_res, symmetric_jac, NULL},
};

const gcs_kernel *gcs_kernels(void) { return KERNELS; }
int gcs_kernel_count(void) { return GCS_N_KERNELS; }

void gcs_kernel_info(int kid, int32_t *out)
{
    const gcs_kernel *k = &KERNELS[kid];
    out[0] = k->n_res; out[1] = k->n_par; out[2] = k->n_const; out[3] = k->const_jac != NULL;
}
