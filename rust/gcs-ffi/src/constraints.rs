//! Constraints: stated, read back, evaluated, and described.

use super::*;

/* -- constraints ----------------------------------------------------------- */

/// Add a constraint from its JSON form; returns its document-stable id, or -1 on error.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_add(
    h: *mut Sketch,
    ptr: *const u8,
    len: usize,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let v = as_json(ptr, len);
        match report::constraint_from_json(s, &v) {
            Ok(c) => s.add(c) as i32,
            Err(e) => {
                set_error(e);
                -1
            }
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_remove(h: *mut Sketch, id: i32) {
    guard((), move || {
        sk(h).remove(id as u32);
    })
}

/// Keep exactly these constraint ids, in this order — the one bulk edit the model allows
/// (a filtered constraint list, as diagnosis trials and the UI's delete perform).
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_set_constraints(
    h: *mut Sketch,
    ptr: *const u8,
    len: usize,
) {
    guard((), move || {
        let s = sk(h);
        let v = as_json(ptr, len);
        let ids: Vec<u32> = v.arr().iter().map(|x| x.as_i64() as u32).collect();
        let mut by_id: BTreeMap<u32, Constraint> =
            s.constraints.drain(..).map(|c| (c.id, c)).collect();
        for id in ids {
            if let Some(c) = by_id.remove(&id) {
                s.constraints.push(c);
            }
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraints_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        report::constraints_json(sk(h))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_json(h: *mut Sketch, id: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        match s.constraint(id as u32) {
            Some(c) => out_json(report::constraint_json(s, c)),
            None => out_json(Json::Null),
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_set_num(
    h: *mut Sketch,
    id: i32,
    name: *const u8,
    name_len: usize,
    v: f64,
) -> i32 {
    guard(-1, move || {
        let n = as_str(name, name_len).to_string();
        match sk(h).constraint(id as u32) {
            None => {
                set_error(format!("no constraint {id}"));
                -1
            }
            Some(_) => {
                if sk(h).set_constraint_num(id as u32, &n, v) {
                    0
                } else {
                    set_error(format!("{n} is not an argument a number can set"));
                    -1
                }
            }
        }
    })
}

/// Set a string argument by name (an arc tangency's end).
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_set_str(
    h: *mut Sketch,
    id: i32,
    name: *const u8,
    name_len: usize,
    v: *const u8,
    v_len: usize,
) -> i32 {
    guard(-1, move || {
        let n = as_str(name, name_len).to_string();
        let val = as_str(v, v_len).to_string();
        match sk(h).constraint_mut(id as u32) {
            None => {
                set_error(format!("no constraint {id}"));
                -1
            }
            Some(c) => {
                if c.set_str(&n, &val) {
                    0
                } else {
                    set_error(format!("{n} is not a string argument"));
                    -1
                }
            }
        }
    })
}

/// Write a dimension from text: a bare number is a constant, anything else an expression
/// (`w = 1`, `h = w * 2`), evaluated with the rest of the document's — see `gcs_core::expr`.
/// 0 when it was stored and computed; 1 when it was stored but could not be computed yet (a name
/// nothing defines), with why in `gcs_last_error()`; -1 when it was rejected (it does not parse,
/// or `name` is no dimension) and nothing changed.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_set_dimension(
    h: *mut Sketch,
    id: i32,
    name: *const u8,
    name_len: usize,
    text: *const u8,
    text_len: usize,
) -> i32 {
    guard(-1, move || {
        let n = as_str(name, name_len).to_string();
        let t = as_str(text, text_len).to_string();
        match expr::set_dimension(sk(h), id as u32, &n, &t) {
            Ok(None) => 0,
            Ok(Some(why)) => {
                set_error(why);
                1
            }
            Err(e) => {
                set_error(e);
                -1
            }
        }
    })
}

/// Every dimension expression in the document, evaluated and in evaluation order: each with its
/// constraint id and attribute, its text, the name it defines, its value (degrees for an angle),
/// the names it reads and its error if it could not be computed.
#[no_mangle]
pub unsafe extern "C" fn gcs_exprs_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || report::exprs_json(sk(h)))
}

/// A drag target's (tx, ty) — the one mutation the hot path performs.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_set_target(h: *mut Sketch, id: i32, tx: f64, ty: f64) {
    guard((), move || {
        match sk(h).constraint_mut(id as u32) {
            None => set_error(format!("no constraint {id}")),
            Some(c) => {
                let name = c.type_name();
                if !c.set_target(tx, ty) {
                    set_error(format!("{name} has no drag target"));
                }
            }
        }
    })
}

/// The global Param indices the kernel's columns refer to; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_params(h: *mut Sketch, id: i32, out: *mut i32) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(c) = s.constraint(id as u32) else { return 0 };
        let ps = c.params(s);
        for (i, p) in ps.iter().enumerate() {
            *out.add(i) = *p as i32;
        }
        ps.len() as i32
    })
}

/// A constraint's rows and columns into `out` as `[n_res, n_par]`; returns 2, or 0 for no such
/// constraint.  What a binding sizes its buffers by: a kernel built at its operands' width (a
/// spline's length reads every control point) has no fixed width a registry could publish.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_widths(h: *mut Sketch, id: i32, out: *mut i32) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(c) = s.constraint(id as u32) else { return 0 };
        let k = c.kernel_in(s);
        // the rows it compiles: an energy's or a peg's may be none, whatever its kernel's width
        *out = c.rows_in(s) as i32;
        *out.add(1) = k.n_par as i32;
        2
    })
}

/// The current values of those params; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_local_values(
    h: *mut Sketch,
    id: i32,
    out: *mut f64,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(c) = s.constraint(id as u32) else { return 0 };
        let v = c.local_values(s);
        for (i, x) in v.iter().enumerate() {
            *out.add(i) = *x;
        }
        v.len() as i32
    })
}

/// One row of the kernel at arbitrary local values: `n_res` residuals and the `n_res * n_par`
/// Jacobian.  Returns `n_par`.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_eval(
    h: *mut Sketch,
    id: i32,
    v: *const f64,
    r_out: *mut f64,
    j_out: *mut f64,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(c) = s.constraint(id as u32) else { return 0 };
        let k = c.kernel_in(s);
        let vals = std::slice::from_raw_parts(v, k.n_par);
        // as many rows as it compiles (`gcs_constraint_widths`), which the caller sized for
        let rows = c.rows_in(s).min(k.n_res);
        let r = c.residual(s, vals);
        let j = c.jacobian(s, vals);
        for (i, x) in r.iter().take(rows).enumerate() {
            *r_out.add(i) = *x;
        }
        for (i, x) in j.iter().take(rows * k.n_par).enumerate() {
            *j_out.add(i) = *x;
        }
        k.n_par as i32
    })
}

/// The first residual row of a constraint in a compiled system.
#[no_mangle]
pub unsafe extern "C" fn gcs_system_row_of(s: *mut System, id: i32) -> i32 {
    guard(-1, move || {
        (*s).row_of(id as u32).map_or(-1, |r| r as i32)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_error(h: *mut Sketch, id: i32) -> f64 {
    guard(f64::NAN, move || {
        let s = sk(h);
        s.constraint(id as u32).map(|c| c.error(s)).unwrap_or(0.0)
    })
}

/// Whether two constraint records say exactly the same thing — same type, same entities in the
/// same roles, same values, up to the swap a commutative type allows.  The rule lives in the
/// core; both bindings ask here rather than each keeping a copy of it.
#[no_mangle]
pub unsafe extern "C" fn gcs_same_constraint(
    h: *mut Sketch,
    a_ptr: *const u8,
    a_len: usize,
    b_ptr: *const u8,
    b_len: usize,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let (av, bv) = (as_json(a_ptr, a_len), as_json(b_ptr, b_len));
        let (Ok(a), Ok(b)) = (
            report::constraint_from_json(s, &av),
            report::constraint_from_json(s, &bv),
        ) else {
            return -1;
        };
        gcs_core::constraints::same_constraint(&a, &b) as i32
    })
}

/// The id of a constraint already stating the same *relation* as the one described — the same
/// type on the same entities, whatever number it states — or -1.  What an *edit* of a dimension
/// would land on, for a caller that wants to offer one.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_stating(
    h: *mut Sketch,
    ptr: *const u8,
    len: usize,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let v = as_json(ptr, len);
        let Ok(c) = report::constraint_from_json(s, &v) else { return -1 };
        s.constraints
            .iter()
            .filter(|k| !(k.intrinsic || k.soft))
            .find(|k| gcs_core::constraints::same_relation(k, &c))
            .map(|k| k.id as i32)
            .unwrap_or(-1)
    })
}

/// The id of an existing constraint that says exactly the same thing, or -1.
#[no_mangle]
pub unsafe extern "C" fn gcs_constraint_duplicate(
    h: *mut Sketch,
    ptr: *const u8,
    len: usize,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let v = as_json(ptr, len);
        let Ok(c) = report::constraint_from_json(s, &v) else { return -1 };
        s.constraints
            .iter()
            .find(|k| gcs_core::constraints::same_constraint(k, &c))
            .map(|k| k.id as i32)
            .unwrap_or(-1)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_describe(h: *mut Sketch, id: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        out_str(s.constraint(id as u32).map(io::describe).unwrap_or_default())
    })
}
