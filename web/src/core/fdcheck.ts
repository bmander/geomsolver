/* Finite-difference verification of analytic Jacobians.
 *
 * The check runs in the core, so the browser sees the core's own numbers. */
import { Constraint } from './constraints.js';
import { Sketch } from './model.js';
import { core, lastError } from './wasm.js';

/** Max abs error between a constraint's analytic and FD Jacobian; throws if too large. */
export function checkConstraint(c: Constraint, rtol = 1e-6, atol = 1e-7): number {
  const sk = c.owner;
  const own = c.id < 0;
  if (own) sk.add(c);
  try {
    const err = core().gcs_check_constraint(sk.handle, c.id, rtol, atol);
    if (err < 0) throw new Error(lastError() || `${c.typeName}: Jacobian mismatch`);
    return err;
  } finally {
    if (own) sk.remove(c);
  }
}

/** The assembled Jacobian of a whole sketch against finite differences. */
export function checkSketch(sk: Sketch, rtol = 1e-6, atol = 1e-6): number {
  const err = core().gcs_check_sketch(sk.handle, rtol, atol);
  if (err < 0) throw new Error(lastError() || 'sketch: Jacobian mismatch');
  return err;
}
