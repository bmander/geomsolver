/* Stage 2 — structural constraint diagnosis.
 *
 * Matching / Dulmage–Mendelsohn on the equations-vs-parameters graph, the (2,3) pebble game on the
 * point-distance subgraph, minimal conflict sets, and the structural-vs-numeric rank cross-check.
 * All of it runs in the core; this module presents the report. */
import { Constraint } from './constraints.js';
import { Param, Point, Primitive, Sketch } from './model.js';
import { System } from './system.js';
import { WitnessReport, reportFrom } from './witness.js';
import { core, takeJson, withJson } from './wasm.js';

export type State = 'well' | 'under' | 'over' | 'conflict';

/** Free parameters up to which the automatic numeric cross-check (a dense SVD) runs. */
export const NUMERIC_MAX = 300;

/** A connected component of the constraint graph with its own DOF accounting. */
export interface Component {
  params: Param[];
  constraints: Constraint[];
  structuralRank: number;
  dof: number;
}

export interface Diagnosis {
  nParams: number;                 /* free parameters */
  nEquations: number;              /* hard residual rows */
  structuralRank: number;          /* maximum matching size */
  numericRank: number | null;      /* Jacobian rank at the current configuration */
  numericSkipped: boolean;
  geometricDependency: number;     /* dependencies only the numbers can see */
  over: Constraint[];
  underParams: Param[];            /* what can move at the configuration diagnosed */
  structuralUnderParams: Param[];
  components: Component[];
  entityState: Map<Primitive, State>;
  rigidClusters: Point[][];        /* from the pebble game on the distance graph */
  redundantDistances: Constraint[];
  violated: Constraint[];
  conflicts: Constraint[] | null;  /* minimal conflict set */
  warnings: string[];
  witness: WitnessReport | null;   /* Stage 4 analysis, on demand */
  dof: number;
  structuralDof: number;
  nRedundant: number;
  structuralNRedundant: number;
  status: State;
  summary: string;
}

export function summary(d: Diagnosis): string {
  return d.summary;
}

export interface DiagnoseOptions {
  system?: System | null;
  /** undefined: run the numeric cross-check only below `numericMax`.  true/false: force it. */
  numeric?: boolean;
  conflicts?: boolean | null;
  witness?: boolean;
  tol?: number;
  numericMax?: number;
}

interface RawDiagnosis {
  nParams: number;
  nEquations: number;
  structuralRank: number;
  numericRank: number | null;
  numericSkipped: boolean;
  geometricDependency: number;
  over: number[];
  underParams: number[];
  structuralUnderParams: number[];
  components: { params: number[]; constraints: number[]; structuralRank: number; dof: number }[];
  entityState: [string, number, State][];
  rigidClusters: number[][];
  redundantDistances: number[];
  violated: number[];
  conflicts: number[] | null;
  warnings: string[];
  witness: Parameters<typeof reportFrom>[1] | null;
  dof: number;
  structuralDof: number;
  nRedundant: number;
  structuralNRedundant: number;
  status: State;
  summary: string;
}

function fromRaw(sk: Sketch, d: RawDiagnosis): Diagnosis {
  const con = (i: number): Constraint => sk.constraintById(i)!;
  const prm = (v: number[]): Param[] => v.map((i) => sk.paramAt(i));
  const ents = new Map<string, Primitive>();
  for (const e of sk.primitives()) ents.set(`${e.kind}:${e.index}`, e);
  return {
    nParams: d.nParams,
    nEquations: d.nEquations,
    structuralRank: d.structuralRank,
    numericRank: d.numericRank,
    numericSkipped: d.numericSkipped,
    geometricDependency: d.geometricDependency,
    over: d.over.map(con),
    underParams: prm(d.underParams),
    structuralUnderParams: prm(d.structuralUnderParams),
    components: d.components.map((c) => ({
      params: prm(c.params),
      constraints: c.constraints.map(con),
      structuralRank: c.structuralRank,
      dof: c.dof,
    })),
    entityState: new Map(d.entityState.map(([k, i, s]) => [ents.get(`${k}:${i}`)!, s])),
    rigidClusters: d.rigidClusters.map((c) => c.map((i) => sk.points[i])),
    redundantDistances: d.redundantDistances.map(con),
    violated: d.violated.map(con),
    conflicts: d.conflicts === null ? null : d.conflicts.map(con),
    warnings: d.warnings,
    witness: d.witness ? reportFrom(sk, d.witness) : null,
    dof: d.dof,
    structuralDof: d.structuralDof,
    nRedundant: d.nRedundant,
    structuralNRedundant: d.structuralNRedundant,
    status: d.status,
    summary: d.summary,
  };
}

/** Structural (and optionally numeric) diagnosis of a sketch at its current configuration.
 *
 *  Pass the `System` you just solved with to avoid a recompile.  `conflicts` left null computes
 *  the minimal conflict set only when some constraint is violated.  `numeric` left undefined runs
 *  the Jacobian rank / null-space cross-check only while the system is small enough for a dense
 *  SVD (`numericMax` free parameters). */
export function diagnose(sketch: Sketch, opts: DiagnoseOptions = {}): Diagnosis {
  const payload = {
    numeric: opts.numeric ?? null,
    conflicts: opts.conflicts ?? null,
    witness: opts.witness ?? false,
    tol: opts.tol ?? 1e-6,
    numericMax: opts.numericMax ?? NUMERIC_MAX,
  };
  const sys = opts.system && opts.system.sketch === sketch ? opts.system : null;
  const raw = withJson(payload, (p, n) => takeJson<RawDiagnosis>(
    sys ? core().gcs_diagnose_with_json(sketch.handle, sys.handle, p, n)
        : core().gcs_diagnose_json(sketch.handle, p, n)));
  return fromRaw(sketch, raw);
}

/** Hard constraints whose residual is not (numerically) zero at the current configuration. */
export function violatedConstraints(sys: System, tol = 1e-6): Constraint[] {
  const ids = takeJson<number[]>(core().gcs_violated_json(sys.sketch.handle, tol)) ?? [];
  return ids.map((i) => sys.sketch.constraintById(i)!);
}

/** Minimal infeasible subset among `candidates` (default: all hard constraints).
 *  "Remove one of these." */
export function minimalConflictSet(sketch: Sketch, candidates: Constraint[] | null = null,
                                   tol = 1e-6): Constraint[] {
  const payload = candidates === null ? null : candidates.map((c) => c.id);
  const ids = withJson(payload, (p, n) =>
    takeJson<number[]>(core().gcs_minimal_conflict_set_json(sketch.handle, p, n, tol))) ?? [];
  return ids.map((i) => sketch.constraintById(i)!);
}

/** (2,3) pebble game on the point-distance graph: the rigid clusters and the redundant Distance
 *  constraints. */
export function distanceRigidity(sketch: Sketch):
    { clusters: Point[][]; redundant: Constraint[] } {
  const d = takeJson<{ clusters: number[][]; redundant: number[] }>(
    core().gcs_distance_rigidity_json(sketch.handle));
  return {
    clusters: d.clusters.map((c) => c.map((i) => sketch.points[i])),
    redundant: d.redundant.map((i) => sketch.constraintById(i)!),
  };
}
