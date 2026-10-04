/* Dimension expressions: `3 * 2`, `sin(30) * 4`, `2 * a + 5`.
 *
 * A dimension may be written as an arithmetic expression; the core parses and evaluates them —
 * see `gcs_core::expr` for the language (trigonometry in degrees, like every angle here).  A name
 * an expression reads is an unknown the solver moves (in a Solvent document, one the source
 * declared: `param a: Length`), so the dimensions reading it are tied to each other and the
 * value they share is left open.  This module only reads the report. */
import { Sketch } from './model.js';
import { core, takeJson } from './wasm.js';

export interface ExprItem {
  /** The constraint it is an argument of, and which argument. */
  id: number;
  attr: string;
  text: string;
  /** Its value in the units a person reads (degrees for an angle) — the last one it evaluated
   *  to, when `error` is set. */
  value: number;
  /** The names it reads. */
  deps: string[];
  /** The free names among them — unknowns the solver moves rather than numbers.  At most one:
   *  a dimension can only follow one free variable. */
  free: string[];
  error: string | null;
}

/** Every expression in the sketch, evaluated, in document order. */
export function expressions(sk: Sketch): ExprItem[] {
  return takeJson<ExprItem[]>(core().gcs_exprs_json(sk.handle)) ?? [];
}
