/* Stage 4 — the witness configuration method (Michelucci & Foufou 2006).
 *
 * Structural analysis cannot see dependencies that follow from geometric theorems; a witness is a
 * configuration with the sketch's incidence structure but generic dimensions, and the Jacobian
 * there tells the truth.  The analysis runs in the core; this module presents its report. */
import { Constraint } from './constraints.js';
import { Param, Sketch } from './model.js';
import { core, takeJson, withBuf } from './wasm.js';

export interface Dependency {
  constraint: Constraint;        /* a dependent (redundant) equation's constraint */
  impliedBy: Constraint[];       /* constraints whose equations span it */
  theorem: boolean;              /* structural analysis could not see it */
}

/** An infinitesimal motion: velocity per free parameter, scaled to unit max displacement. */
export interface Motion {
  velocity: Float64Array;
  rigid: boolean;                /* a rigid-body motion of the whole sketch */
  /** The Params this motion actually moves — the core's own reading of its velocities, since
   *  which of them count as moving is a fact about the analysis and not about how a caller
   *  chooses to print it. */
  moving: Param[];
}

export interface WitnessReport {
  xWitness: Float64Array;
  usedCurrent: boolean;          /* the sketch itself served as witness */
  numericRank: number;
  dependencies: Dependency[];
  motions: Motion[];             /* null-space basis: rigid modes first, then internal DOFs */
  movable: number[];             /* free-parameter indices taking part in some motion */
  warnings: string[];
  summary: string;
}

export const nDof = (w: WitnessReport): number => w.motions.length;
export const nInternalDof = (w: WitnessReport): number => w.motions.filter((m) => !m.rigid).length;
export const witnessSummary = (w: WitnessReport): string => w.summary;

interface RawDep {
  constraint: number;
  impliedBy: number[];
  theorem: boolean;
}

interface RawReport {
  xWitness: number[];
  usedCurrent: boolean;
  numericRank: number;
  dependencies: RawDep[];
  motions: { velocity: number[]; rigid: boolean; movingParams: number[] }[];
  movable: number[];
  warnings: string[];
  summary: string;
}

export function reportFrom(sk: Sketch, d: RawReport): WitnessReport {
  const con = (i: number): Constraint => sk.constraintById(i)!;
  return {
    xWitness: Float64Array.from(d.xWitness),
    usedCurrent: d.usedCurrent,
    numericRank: d.numericRank,
    dependencies: d.dependencies.map((x) => ({
      constraint: con(x.constraint),
      impliedBy: x.impliedBy.map(con),
      theorem: x.theorem,
    })),
    motions: d.motions.map((m) => ({
      velocity: Float64Array.from(m.velocity),
      rigid: m.rigid,
      moving: m.movingParams.map((i) => sk.paramAt(i)),
    })),
    movable: d.movable,
    warnings: d.warnings,
    summary: d.summary,
  };
}

/** Rank, dependencies and motions of the sketch's constraint system at a witness. */
export function analyze(sk: Sketch, xWitness: ArrayLike<number> | null = null,
                        opts: { seed?: number } = {}): WitnessReport {
  const seed = (opts.seed ?? 0) >>> 0;
  if (xWitness) {
    const x0 = sk.getX();
    sk.setX(xWitness);
    let rep: WitnessReport;
    try {
      rep = reportFrom(sk, takeJson<RawReport>(core().gcs_witness_json(sk.handle, seed)));
    } finally {
      sk.setX(x0);
    }
    rep.xWitness = Float64Array.from(xWitness as Float64Array);
    return rep;
  }
  return reportFrom(sk, takeJson<RawReport>(core().gcs_witness_json(sk.handle, seed)));
}

/** A configuration with the sketch's incidence structure and generic dimensions.  Leaves the
 *  sketch's values and dimensions untouched. */
export function makeWitness(sk: Sketch, seed = 0): Float64Array {
  const n = sk.params.length;
  return withBuf(Math.max(n, 1), 8, (b) => {
    core().gcs_make_witness(sk.handle, seed >>> 0, b.ptr);
    return b.f64.slice(0, n);
  });
}
