//! A compiled system, solving it, and the linear algebra and checks exposed beside it.

use super::*;

/* -- system ---------------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_system_new(h: *mut Sketch) -> *mut System {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(System::new(sk(h))))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_free(s: *mut System) {
    guard((), move || {
        if !s.is_null() {
            drop(Box::from_raw(s));
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_n_res(s: *mut System) -> i32 {
    guard(-1, move || {
        (*s).n_res as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_n_free(s: *mut System) -> i32 {
    guard(-1, move || {
        (*s).n_free as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_nnz(s: *mut System) -> i32 {
    guard(-1, move || {
        (*s).nnz as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_scale(s: *mut System) -> f64 {
    guard(f64::NAN, move || {
        (*s).scale
    })
}

/// One flag per residual row: rows that must be satisfied.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_hard(s: *mut System, out: *mut u8) {
    guard((), move || {
        write_map(out, &(*s).hard, |&b| b as u8);
    })
}

/// The free values of the current sketch geometry (also refreshes the core's copy of x).
#[no_mangle]
pub unsafe extern "C" fn gcs_system_z0(s: *mut System, h: *mut Sketch, out: *mut f64) {
    guard((), move || {
        let z = (*s).z0(sk(h));
        write(out, &z);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_residuals(s: *mut System, z: *const f64, out: *mut f64) {
    guard((), move || {
        let sys = &mut *s;
        let zz = std::slice::from_raw_parts(z, sys.n_free);
        let r = sys.residuals(zz);
        write(out, &r);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_jacobian_dense(s: *mut System, z: *const f64, out: *mut f64) {
    guard((), move || {
        let sys = &mut *s;
        let zz = std::slice::from_raw_parts(z, sys.n_free);
        let j = sys.jacobian_dense(zz);
        write(out, &j.data);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_csr_structure(
    s: *mut System,
    indptr: *mut i32,
    indices: *mut i32,
) {
    guard((), move || {
        let sys = &*s;
        write(indptr, &sys.csr_indptr);
        write(indices, &sys.csr_indices);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_csr_data(s: *mut System, z: *const f64, out: *mut f64) {
    guard((), move || {
        let sys = &mut *s;
        let zz = std::slice::from_raw_parts(z, sys.n_free);
        let d = sys.compute_csr(zz).to_vec();
        write(out, &d);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_max_hard_residual(
    s: *mut System,
    h: *mut Sketch,
) -> f64 {
    guard(f64::NAN, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        sys.max_hard_residual(&z)
    })
}

/// max |residual| per constraint, in block order; `ids` receives the matching constraint ids.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_constraint_errors(
    s: *mut System,
    h: *mut Sketch,
    ids: *mut i32,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        let e = sys.constraint_errors(&z);
        // one entry per *compiled* constraint, which is not the live sketch's count once the
        // sketch has been edited: never write past what the caller sized
        let n = e.len().min(cap.max(0) as usize);
        if n < e.len() {
            set_error(format!("constraint_errors: {} entries need {} slots", e.len(), e.len()));
        }
        write(out, &e[..n]);
        write_map(ids, &sys.cids[..n], |&c| c as i32);
        n as i32
    })
}

/// max |residual| / that row's units over the hard rows — dimensionless, so one threshold judges
/// every kernel.  This, not `gcs_system_max_hard_residual`, is what "solved" means.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_max_relative_residual(s: *mut System, h: *mut Sketch) -> f64 {
    guard(f64::NAN, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        sys.max_relative_residual(&z)
    })
}

/// How many constraints the plan was compiled from — the size `gcs_system_constraint_errors`
/// needs, which is the live sketch's count only until the sketch is edited.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_n_constraints(s: *mut System) -> i32 {
    guard(-1, move || (*s).n_constraints() as i32)
}

/// Numerical rank of the Jacobian at the sketch's current values; `tol` is absolute and
/// dimensionless (`gcs_rank_tol()` is the diagnosis's own).
#[no_mangle]
pub unsafe extern "C" fn gcs_system_rank(
    s: *mut System,
    h: *mut Sketch,
    tol: f64,
    hard_only: i32,
) -> i32 {
    guard(-1, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        sys.rank(&z, tol, hard_only != 0) as i32
    })
}

/// The tolerance a rank is judged at — `system::RANK_TOL`, so a binding's default is the
/// core's and not a second number.
#[no_mangle]
pub unsafe extern "C" fn gcs_rank_tol() -> f64 {
    gcs_core::system::RANK_TOL
}

/// The hard rows of the Jacobian at the sketch's current values with their units divided out
/// — the matrix every rank and null space in the core is judged on (`System::conditioned`).
/// Row-major, one row per hard row in `gcs_system_structure_json`'s order, `n_free` columns;
/// returns the row count.  A getter, so a test can check a verdict against the matrix the
/// core actually judged.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_conditioned(s: *mut System, h: *mut Sketch, out: *mut f64) -> i32 {
    guard(-1, move || {
        let sys = &mut *s;
        let z = sys.z0(sk(h));
        let c = sys.conditioned(&z);
        write(out, &c.as_mat().data);
        c.rows() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_update_consts(s: *mut System, h: *mut Sketch, id: i32) {
    guard((), move || {
        (*s).update_consts(sk(h), id as u32);
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_system_refresh_consts(s: *mut System, h: *mut Sketch) {
    guard((), move || {
        (*s).refresh_consts(sk(h));
    })
}

/// The structural Jacobian as a bipartite graph, plus row → constraint id.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_structure_json(s: *mut System) -> *mut u8 {
    json_out(move || {
        let (adj, row_c) = (*s).structure();
        json::object([
            (
                "adj",
                Json::Arr(
                    adj.iter()
                        .map(|r| Json::Arr(r.iter().map(|&c| Json::Int(c as i64)).collect()))
                        .collect(),
                ),
            ),
            ("rowC", Json::Arr(row_c.iter().map(|&c| Json::Int(c as i64)).collect())),
            ("nFree", Json::Int((*s).n_free as i64)),
        ])
    })
}

/// Free-parameter index → sketch Param index.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_free_indices(s: *mut System, out: *mut i32) {
    guard((), move || {
        write(out, &(*s).free);
    })
}

/* -- solving --------------------------------------------------------------- */

/// `[success, status, residualNorm, maxResidual, nfev, njev, iterations, rank, settled]` (rank
/// -1 = none): nine numbers, `SolveResult::settled` last (#449).
pub(crate) fn write_result(r: &SolveResult, out: *mut f64) {
    let v = [
        r.success as i32 as f64,
        r.status as f64,
        r.residual_norm,
        r.max_residual,
        r.nfev as f64,
        r.njev as f64,
        r.iterations as f64,
        r.rank.map(|x| x as f64).unwrap_or(-1.0),
        r.settled() as i32 as f64,
    ];
    unsafe {
        for (i, x) in v.iter().enumerate() {
            *out.add(i) = *x;
        }
    }
}

fn opts_from(method: i32, tol: f64, max_iter: i32, max_nfev: i32, dense: i32, writeback: i32) -> SolveOpts {
    SolveOpts {
        method: if method == 1 { Method::Lm } else { Method::DogLeg },
        tol,
        max_nfev,
        writeback: writeback != 0,
        max_iter,
        dense: if dense < 0 { None } else { Some(dense != 0) },
        ..SolveOpts::default()
    }
}

/// Solve on an already-compiled system.  Returns the status message.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_system_solve(
    s: *mut System,
    h: *mut Sketch,
    method: i32,
    tol: f64,
    max_iter: i32,
    max_nfev: i32,
    dense: i32,
    writeback: i32,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = (*s).solve(sk(h), opts_from(method, tol, max_iter, max_nfev, dense, writeback));
        write_result(&r, out);
        out_str(r.message)
    })
}

/// One-shot: compile and solve.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_solve(
    h: *mut Sketch,
    method: i32,
    tol: f64,
    max_iter: i32,
    max_nfev: i32,
    dense: i32,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = gcs_core::solve::solve(sk(h), opts_from(method, tol, max_iter, max_nfev, dense, 1));
        write_result(&r, out);
        out_str(r.message)
    })
}

#[no_mangle]
pub extern "C" fn gcs_status_message(status: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(newton::status_message(status).to_string())
    })
}

/* -- dense linear algebra (exposed so the bindings can be checked) ---------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_min_norm_lstsq(
    m: i32,
    n: i32,
    nrhs: i32,
    a: *const f64,
    b: *const f64,
    rcond: f64,
    x: *mut f64,
) -> i32 {
    guard(-1, move || {
        let (m, n, nrhs) = (m as usize, n as usize, nrhs as usize);
        let am = linalg::Mat::from_vec(m, n, std::slice::from_raw_parts(a, m * n).to_vec());
        let bm = linalg::Mat::from_vec(m, nrhs, std::slice::from_raw_parts(b, m * nrhs).to_vec());
        let (xm, rank) = linalg::min_norm_lstsq(&am, &bm, rcond);
        write(x, &xm.data);
        rank as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_rrqr(
    m: i32,
    n: i32,
    a: *const f64,
    rcond: f64,
    piv: *mut i32,
) -> i32 {
    guard(-1, move || {
        let (m, n) = (m as usize, n as usize);
        let am = linalg::Mat::from_vec(m, n, std::slice::from_raw_parts(a, m * n).to_vec());
        let (rank, p) = linalg::rrqr(&am, rcond);
        if !piv.is_null() {
            write(piv, &p);
        }
        rank as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_svd(
    m: i32,
    n: i32,
    a: *const f64,
    u: *mut f64,
    s: *mut f64,
    vt: *mut f64,
) -> i32 {
    guard(-1, move || {
        let (mm, nn) = (m as usize, n as usize);
        let am = linalg::Mat::from_vec(mm, nn, std::slice::from_raw_parts(a, mm * nn).to_vec());
        let d = linalg::svd(&am, !u.is_null());
        write(s, &d.s);
        write(vt, &d.vt.data);
        if !u.is_null() {
            write(u, &d.u.data);
        }
        if d.converged {
            0
        } else {
            set_error("svd: the QR sweeps did not converge".to_string());
            -1
        }
    })
}

/// Numerical rank and null space from one SVD; `n_out` is written with `n - rank` columns.
#[no_mangle]
pub unsafe extern "C" fn gcs_rank_nullspace(
    m: i32,
    n: i32,
    a: *const f64,
    rcond: f64,
    n_out: *mut f64,
    s_out: *mut f64,
) -> i32 {
    guard(-1, move || {
        let (mm, nn) = (m as usize, n as usize);
        let am = linalg::Mat::from_vec(mm, nn, std::slice::from_raw_parts(a, mm * nn).to_vec());
        let rn = linalg::rank_and_nullspace(&am, rcond);
        write(n_out, &rn.null().data);
        if !s_out.is_null() {
            write(s_out, &rn.s);
        }
        if !rn.converged {
            set_error("rank_nullspace: the SVD did not converge".to_string());
            return -1;
        }
        rn.rank as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_lu_solve(n: i32, a: *mut f64, b: *mut f64) -> i32 {
    guard(-1, move || {
        let n = n as usize;
        let aa = std::slice::from_raw_parts_mut(a, n * n);
        let bb = std::slice::from_raw_parts_mut(b, n);
        linalg::lu_solve(n, aa, bb) as i32 - 1
    })
}

/* -- finite-difference check ----------------------------------------------- */

/// Max |analytic − FD| over the whole sketch Jacobian; negative on mismatch.
#[no_mangle]
pub unsafe extern "C" fn gcs_check_sketch(h: *mut Sketch, rtol: f64, atol: f64) -> f64 {
    guard(f64::NAN, move || {
        match fdcheck::check_sketch(sk(h), rtol, atol) {
            Ok(v) => v,
            Err(e) => {
                set_error(e);
                -1.0
            }
        }
    })
}

/// Max |analytic − FD| for one constraint; negative on mismatch.
#[no_mangle]
pub unsafe extern "C" fn gcs_check_constraint(
    h: *mut Sketch,
    id: i32,
    rtol: f64,
    atol: f64,
) -> f64 {
    guard(f64::NAN, move || {
        let s = sk(h);
        let Some(c) = s.constraint(id as u32) else { return -1.0 };
        match fdcheck::check_constraint(s, c, rtol, atol) {
            Ok(v) => v,
            Err(e) => {
                set_error(e);
                -1.0
            }
        }
    })
}
