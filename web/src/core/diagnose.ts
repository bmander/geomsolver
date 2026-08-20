/* Stage 2 — structural constraint diagnosis.
 *
 * Turns "solver failed" into "these constraints conflict / this entity has 2 DOF":
 *
 *  * bipartite equations-vs-free-parameters graph (`System.structure()`), maximum matching
 *    (Hopcroft-Karp) -> structural rank; Dulmage-Mendelsohn -> over-determined (structurally
 *    redundant equations), under-determined (structurally free parameters) and
 *    well-determined parts; per connected component DOF bookkeeping;
 *  * (2,3) pebble game on the point-distance subgraph -> rigid clusters and redundant
 *    distances (feeds Stage 3);
 *  * minimal conflict set (deletion filter) when the solve is infeasible;
 *  * structural vs numeric rank cross-check: everything above is structural and cannot see
 *    theorem-induced dependencies; when the Jacobian rank is lower than the matching we log
 *    it — that residue is Stage 4's motivation.
 */
import { Constraint, Distance } from './constraints.js';
import { coincidentClasses } from './cgraph.js';
import * as graph from './graph.js';
import { Mat, rankAndNullspace, selectRows, transpose } from './linalg.js';
import { Param, Point, Primitive, Sketch } from './model.js';
import { Method, System } from './system.js';
import { WitnessReport, analyze, movableColumns } from './witness.js';

export type State = 'well' | 'under' | 'over' | 'conflict';
const SEVERITY: Record<State, number> = { well: 0, under: 1, over: 2, conflict: 3 };
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
  /** The numeric cross-check was skipped because the system is past the dense limit. */
  numericSkipped: boolean;
  /** How many dependencies only the numbers can see (0 when the check did not run). */
  geometricDependency: number;
  over: Constraint[];              /* constraints in the over-determined block */
  /** Parameters that can move *at the configuration diagnosed* — the Jacobian null space
   *  when available, else the (generous) structural under-block.  The null space is a
   *  property of the pose, so re-diagnose after moving geometry. */
  underParams: Param[];
  structuralUnderParams: Param[];
  components: Component[];
  entityState: Map<Primitive, State>;
  rigidClusters: Point[][];        /* from the pebble game on the distance graph */
  redundantDistances: Constraint[];
  violated: Constraint[];
  conflicts: Constraint[] | null;  /* minimal conflict set */
  warnings: string[];
  witness: WitnessReport | null;   /* Stage 4 analysis, on demand */
  /** Degrees of freedom left at the current configuration — what can still be dragged.  Uses
   *  the numeric rank when the cross-check ran, because a matching cannot tell that two
   *  equations say the same thing: it counts them both and calls the sketch rigid while the
   *  geometry moves freely. */
  dof: number;
  /** DOF the matching alone sees — it bounds the rank from above, so it bounds DOF below. */
  structuralDof: number;
  nRedundant: number;
  /** Redundant equations the matching alone sees. */
  structuralNRedundant: number;
  status: State;
}

export function summary(d: Diagnosis): string {
  const parts = [
    `${d.nParams} params, ${d.nEquations} equations, structural rank ${d.structuralRank}`,
    `DOF ${d.dof}`,
  ];
  if (d.dof !== d.structuralDof) {
    parts.push(`the matching alone would say DOF ${d.structuralDof} — `
      + `${d.geometricDependency} equation(s) carry no information`);
  }
  if (d.nRedundant) parts.push(`${d.nRedundant} redundant equation(s) among ${d.over.length} constraint(s)`);
  if (d.geometricDependency) {
    parts.push(`numeric rank ${d.numericRank} < structural ${d.structuralRank}: `
      + `${d.geometricDependency} geometric (theorem-type) dependency`);
  }
  if (d.conflicts && d.conflicts.length) {
    parts.push(`CONFLICT — remove one of: ${d.conflicts.map((c) => c.typeName).join(', ')}`);
  } else if (d.violated.length) {
    parts.push(`${d.violated.length} constraint(s) violated`);
  }
  if (d.components.length > 1) {
    parts.push(`${d.components.length} components: DOF ` + d.components.map((c) => c.dof).join(', '));
  }
  if (d.rigidClusters.length) parts.push(`${d.rigidClusters.length} rigid cluster(s) in the distance graph`);
  return parts.join('; ');
}

/** Constraints that could be deleted without losing any information, given `W`, a basis of the
 *  left null space of the Jacobian (one column per dependency).
 *
 *  With W orthonormal, dropping a set of rows R gives
 *      rank(J minus R) = rank(J) - |R| + rank(W[R]),
 *  so a constraint is free to delete exactly when its own rows are *independent* in W.  That
 *  distinction is the whole point: an arc whose endpoints are mirrored about a line through its
 *  centre makes one of `Symmetric`'s two residuals implied by the arc's radius equations, but
 *  `Symmetric` still carries the perpendicularity — its two rows are dependent in W, it is doing
 *  real work, and telling the user to remove it would be wrong.  Only a wholly implied
 *  constraint is worth naming.
 *
 *  Intrinsic constraints are skipped: they come with the primitive and cannot be deleted. */
export function removableConstraints(W: Mat, rowC: Constraint[], rtol = 1e-8): Constraint[] {
  if (W.rows === 0 || W.cols === 0) return [];
  const rows = new Map<Constraint, number[]>();
  rowC.forEach((c, r) => { const v = rows.get(c); if (v) v.push(r); else rows.set(c, [r]); });
  const out: Constraint[] = [];
  for (const [c, rs] of rows) {
    if (c.intrinsic) continue;
    const sub = selectRows(W, rs);
    let peak = 0;
    for (const v of sub.data) peak = Math.max(peak, Math.abs(v));
    if (!(peak > rtol)) continue;
    if (rankAndNullspace(sub, rtol).rank === rs.length) out.push(c);
  }
  return out;
}

const tolAbs = (sketch: Sketch, tol: number): number => tol * Math.max(1, sketch.extent()) ** 2;

/** Hard constraints whose residual is not (numerically) zero at the current configuration. */
export function violatedConstraints(sys: System, tol = 1e-6): Constraint[] {
  const lim = tolAbs(sys.sketch, tol);
  const err = sys.constraintErrors();
  return sys.constraints.filter((c) => !c.soft && (err.get(c) ?? 0) > lim);
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

/** Structural (and optionally numeric) diagnosis of a sketch at its current configuration.
 *
 *  Pass the `System` you just solved with to avoid a recompile.  `conflicts` left null
 *  computes the minimal conflict set only when some constraint is violated.  `witness: true`
 *  adds the Stage-4 analysis (dependent constraints with what implies them, and the motions).
 *
 *  `numeric` left undefined runs the Jacobian rank / null-space cross-check only while the
 *  system is small enough for a dense SVD (`numericMax` free parameters); above that the
 *  diagnosis stays structural — which is what Stage 2 is for — and says so in `warnings`.
 *  Diagnosis runs after every edit, and one dense SVD of a 1000-entity sketch costs more than
 *  every other step put together. */
export function diagnose(sketch: Sketch, opts: DiagnoseOptions = {}): Diagnosis {
  const tol = opts.tol ?? 1e-6;
  const own = !(opts.system && opts.system.sketch === sketch);
  const sys = own ? new System(sketch) : opts.system!;
  try {
    const { adj, rowC } = sys.structure();
    const nCols = sys.nFree;
    const dm = graph.dulmageMendelsohn(adj, nCols);
    const freeParams = Array.from(sys.free, (i) => sketch.params[i]);

    const overSet = new Set<Constraint>();
    const over: Constraint[] = [];
    for (const r of dm.overRows) {
      const c = rowC[r];
      if (!overSet.has(c)) { overSet.add(c); over.push(c); }
    }
    const structuralUnder = dm.underCols.map((j) => freeParams[j]);
    let underParams = structuralUnder;

    // -- components --
    const { compRow, compCol, count: nComp } = graph.bipartiteComponents(adj, nCols);
    const compParams: Param[][] = Array.from({ length: nComp }, () => []);
    const compCs: Set<Constraint>[] = Array.from({ length: nComp }, () => new Set());
    const compRank = new Int32Array(nComp);
    for (let j = 0; j < nCols; j++) {
      compParams[compCol[j]].push(freeParams[j]);
      if (dm.mateCol[j] >= 0) compRank[compCol[j]]++;
    }
    for (let r = 0; r < adj.length; r++) compCs[compRow[r]].add(rowC[r]);
    const components: Component[] = Array.from({ length: nComp }, (_, i) => ({
      params: compParams[i],
      constraints: [...compCs[i]],
      structuralRank: compRank[i],
      dof: compParams[i].length - compRank[i],
    })).sort((a, b) => b.params.length - a.params.length);

    // -- witness analysis (Stage 4), on demand --
    const warnings: string[] = [];
    let wit: WitnessReport | null = null;
    if (opts.witness && nCols && sys.nRes) wit = analyze(sketch, null, { system: sys, overIds: overSet });

    // -- numeric cross-check: rank and the parameters that can actually move --
    let numericRank: number | null = null;
    const numericMax = opts.numericMax ?? NUMERIC_MAX;
    const wantNumeric = opts.numeric ?? nCols <= numericMax;
    const numericSkipped = opts.numeric === undefined && !wantNumeric;
    if (numericSkipped) {
      warnings.push(`numeric cross-check skipped: ${nCols} free parameters is above the dense limit `
        + `(${numericMax}) — the diagnosis below is structural only`);
    }
    if (wantNumeric && nCols && sys.nRes) {
      let movable: number[];
      if (wit && wit.usedCurrent) {
        numericRank = wit.numericRank;             // same J at the same x
        movable = wit.movable;
      } else {
        const dense = sys.jacobianDense(sys.z0());
        const hardRows: number[] = [];
        for (let i = 0; i < sys.nRes; i++) if (sys.hard[i]) hardRows.push(i);
        const { N } = rankAndNullspace(selectRows(dense, hardRows));
        numericRank = nCols - N.cols;
        movable = movableColumns(N);
      }
      // Which parameters can actually move: rows of the null space that are nonzero.  Sharper
      // than the DM under-block (which counts a parameter as free if it could be in some
      // generic assignment); evaluated at the current configuration, so a degenerate placement
      // can still fool it — the witness step generalises this.
      underParams = movable.map((j) => freeParams[j]);
      if (numericRank < dm.rank) {
        // ...and name the constraints worth removing, or the report would say
        // "over-constrained" with nothing to point at.  One extra SVD, only on this path: a
        // healthy sketch never reaches it.
        const dense = sys.jacobianDense(sys.z0());
        const hardRows: number[] = [];
        for (let i = 0; i < sys.nRes; i++) if (sys.hard[i]) hardRows.push(i);
        const { N: W } = rankAndNullspace(transpose(selectRows(dense, hardRows)));
        for (const c of removableConstraints(W, rowC)) {
          if (!overSet.has(c)) { overSet.add(c); over.push(c); }
        }
        warnings.push(`structural rank ${dm.rank} but numeric rank ${numericRank}: `
          + 'a dependency the graph cannot see (theorem-induced or degenerate configuration) — Stage 4');
      }
    }

    // -- violated / conflicts --
    const violated = violatedConstraints(sys, tol);
    let conflictSet: Constraint[] | null = null;
    if (opts.conflicts ?? violated.length > 0) {
      // Candidates = the structurally over-determined block (where a redundancy must live);
      // if the graph sees nothing wrong (e.g. the triangle inequality) fall back to the
      // violated constraints.  Everything else stays fixed, so the result is minimal "among
      // the suspects" — and the filter costs |candidates| solves, not |all|.
      conflictSet = minimalConflictSet(sketch, over.length ? over : violated, tol);
    }

    // -- pebble game on the point-distance graph --
    const { clusters, redundant } = distanceRigidity(sketch);

    // -- entity states --
    const underSet = new Set(underParams);
    const conflictIds = new Set(conflictSet ?? violated);
    const touched = new Map<Primitive, Constraint[]>();
    const touch = (e: Primitive, c: Constraint): void => {
      let l = touched.get(e);
      if (!l) touched.set(e, (l = []));
      l.push(c);
    };
    for (const c of sketch.hardConstraints()) {
      for (const e of c.entities()) {
        touch(e, c);
        for (const ch of e.children) touch(ch, c);
      }
    }
    const ents: Primitive[] = [...sketch.points, ...sketch.lines, ...sketch.circles, ...sketch.arcs];
    const state = new Map<Primitive, State>();
    for (const e of ents) {
      const cs = touched.get(e) ?? [];
      let st: State = 'well';
      if (cs.some((c) => conflictIds.has(c))) st = 'conflict';
      else if (cs.some((c) => overSet.has(c))) st = 'over';
      else if (e.params.some((p) => underSet.has(p))) st = 'under';
      state.set(e, st);
    }
    for (const e of ents) {
      for (const ch of e.children) {
        if (SEVERITY[state.get(ch)!] > SEVERITY[state.get(e)!]) state.set(e, state.get(ch)!);
      }
    }
    if (wit) warnings.push(...wit.warnings);

    // the structural matching is a *generic* upper bound on the rank; where the numeric
    // cross-check ran it is the truth at this configuration, and it is what decides what moves
    const effectiveRank = numericRank === null ? dm.rank : numericRank;
    const dof = nCols - effectiveRank;
    const structuralDof = nCols - dm.rank;
    // beyond the rank = carrying no information; numeric when the cross-check ran, for the
    // same reason as `dof` — a matching counts two equations that say the same thing as two
    const nRedundant = adj.length - effectiveRank;
    const structuralNRedundant = adj.length - dm.rank;
    // `over` only when something is actually worth removing: a rank deficiency alone is not
    // enough, since a dependency can be shared between a user constraint that is still doing
    // work and a primitive's own definition, leaving nothing to delete
    const status: State = (conflictSet && conflictSet.length) || violated.length ? 'conflict'
      : over.length ? 'over' : dof > 0 ? 'under' : 'well';
    return {
      nParams: nCols, nEquations: adj.length, structuralRank: dm.rank, numericRank, numericSkipped,
      geometricDependency: numericRank === null ? 0 : Math.max(0, dm.rank - numericRank),
      over, underParams, structuralUnderParams: structuralUnder, components,
      entityState: state, rigidClusters: clusters, redundantDistances: redundant,
      violated, conflicts: conflictSet, warnings, witness: wit, dof, structuralDof, nRedundant, structuralNRedundant, status,
    };
  } finally {
    if (own) sys.dispose();
  }
}

/** Minimal infeasible subset among `candidates` (default: all hard constraints); the rest
 *  stay in the system throughout.  "Remove one of these."
 *
 *  Grow-then-shrink: add candidates one at a time, each solve warm-started from the previous
 *  feasible configuration, until one breaks feasibility (it is in the conflict); then delete
 *  the earlier ones one at a time, keeping a deletion whenever the rest is still infeasible.
 *  Warm-starting from feasible states is what makes the trials reliable. */
export function minimalConflictSet(sketch: Sketch, candidates: Constraint[] | null = null,
                                   tol = 1e-6, method: Method = 'dogleg', maxIter = 60): Constraint[] {
  const x0 = sketch.getX();
  const hard = sketch.hardConstraints();
  const cands = (candidates ?? hard).filter((c) => !c.soft);
  const candSet = new Set(cands);
  const others = hard.filter((c) => !candSet.has(c));
  const lim = tolAbs(sketch, tol);
  const saved = sketch.constraints;

  const solveWith = (cs: Constraint[], xStart: Float64Array): [boolean, Float64Array] => {
    sketch.setX(xStart);
    sketch.constraints = cs;
    const sys = new System(sketch);
    try {
      const ok = sys.solve({ method, maxIter }).maxResidual <= lim;
      return [ok, sketch.getX()];
    } finally {
      sys.dispose();
      sketch.constraints = saved;
    }
  };

  try {
    let [ok, xBase] = solveWith(others, x0);      // a state satisfying the non-candidates
    if (!ok) xBase = x0;
    const accepted: Constraint[] = [];
    let xFeas = xBase;
    let culprit: Constraint | null = null;
    for (const c of cands) {
      const [good, x] = solveWith([...others, ...accepted, c], xFeas);
      if (good) { accepted.push(c); xFeas = x; }
      else { culprit = c; break; }
    }
    if (!culprit) return [];
    let keep = [...accepted];                     // shrink
    for (const c of accepted) {
      const trial = keep.filter((k) => k !== c);
      const [good] = solveWith([...others, ...trial, culprit], xFeas);
      if (!good) keep = trial;
    }
    return [...keep, culprit];
  } finally {
    sketch.setX(x0);
  }
}

/** (2,3) pebble game on the point-distance graph: vertices are points with Coincident points
 *  merged; edges are Distance constraints.  Returns the rigid clusters (as sets of Points)
 *  and the redundant Distance constraints. */
export function distanceRigidity(sketch: Sketch): { clusters: Point[][]; redundant: Constraint[] } {
  const { of, members } = coincidentClasses(sketch);
  const edgeC = sketch.constraints.filter((c): c is Distance => c instanceof Distance);
  if (!edgeC.length) return { clusters: [], redundant: [] };
  const edges = edgeC.map((c) => [of.get(c.p)!, of.get(c.q)!] as [number, number]);
  const res = graph.pebbleGame(members.length, edges);
  return {
    clusters: res.components.map((comp) => comp.flatMap((v) => members[v])),
    redundant: res.redundant.map((i) => edgeC[i]),
  };
}
