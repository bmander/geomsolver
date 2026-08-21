/* Dimension expressions: `w = 1`, `h = w * 2`, `sin(h * 10)`.
 *
 * A dimension may be written as an arithmetic expression naming values other dimensions
 * define; the core parses them, orders them by what they read and evaluates them — see
 * `gcs_core::expr` for the language (trigonometry in degrees, like every angle here).  This
 * module only reads the report. */
import { Sketch } from './model.js';
import { core, takeJson } from './wasm.js';

export interface ExprItem {
  /** The constraint it is an argument of, and which argument. */
  id: number;
  attr: string;
  text: string;
  /** The name it defines, if any. */
  name: string | null;
  /** Its value in the units a person reads (degrees for an angle) — the last one it evaluated
   *  to, when `error` is set. */
  value: number;
  /** The names it reads. */
  deps: string[];
  error: string | null;
}

/** Every expression in the sketch, evaluated, in evaluation order: each after the names it
 *  reads, earliest in the document first among the ready ones, the ones on a cycle last. */
export function expressions(sk: Sketch): ExprItem[] {
  return takeJson<ExprItem[]>(core().gcs_exprs_json(sk.handle)) ?? [];
}
