/* The drawing's claims about solids, judged a pose at a time.
 *
 * A claim about solids (`claim a clear(1mm) b`, `claim over turn in (0deg, 360deg) { … }`) reads
 * the solved pose and enters nothing else the diagnosis says, and judging one is seconds of solids
 * evaluated — 37 poses for a claim over a sweep.  So a page's diagnosis leaves them out
 * (`diagnose(…, { solidClaims: false })`) and a worker steps a `ClaimJudge`
 * (`diagnose::ClaimJudge`), saying between poses where it stands, in the core's words. */
import type { Sketch } from './model.js';
import { core, lastError, takeJson } from './wasm.js';

/** A claim about solids as the drawing states it: its statement and its words. */
export interface SolidClaim {
  stmt: number;
  text: string;
}

/** Where the judging stands: `doing` is the core's words for it, which a page shows as they are
 *  (`judging claim a clear(1mm) b (2 of 3), pose 5 of 37`); `fraction` the whole of it done. */
export interface ClaimProgress {
  doing: string;
  fraction: number;
  claim: number;
  claims: number;
  pose: number;
  poses: number;
  stmt: number | null;
}

/** What a claim came to (`report::solid_claim_json`): `line` is the report's own sentence for
 *  it. */
export interface ClaimVerdict {
  stmt: number;
  line: string;
  statement: string;
  verdict: 'holds' | 'sampled-success' | 'refuted' | 'undecided';
  measured: number | null;
  tolerance: number | null;
}

/** The claims about solids this sketch states, judged or not. */
export function solidClaims(sk: Sketch): SolidClaim[] {
  return takeJson<SolidClaim[]>(core().gcs_sketch_solid_claims_json(sk.handle)) ?? [];
}

/** **The claims of one drawing, judged a pose at a time** — the drawing at the pose it stands in
 *  when the judge is made; it judges a copy, so the sketch may change or go meanwhile. */
export class ClaimJudge {
  private constructor(private h: number) {}

  static create(sk: Sketch): ClaimJudge {
    const h = core().gcs_claim_judge_new(sk.handle);
    if (!h) throw new Error(lastError());
    return new ClaimJudge(h);
  }

  /** Judge the next pose: true once every claim is judged. */
  step(): boolean {
    const r = core().gcs_claim_judge_step(this.h);
    if (r < 0) throw new Error(lastError());
    return r === 1;
  }

  progress(): ClaimProgress {
    return takeJson<ClaimProgress>(core().gcs_claim_judge_progress(this.h));
  }

  /** The verdicts so far, in the drawing's order. */
  verdicts(): ClaimVerdict[] {
    return takeJson<ClaimVerdict[]>(core().gcs_claim_judge_verdicts_json(this.h)) ?? [];
  }

  dispose(): void {
    if (this.h) core().gcs_claim_judge_free(this.h);
    this.h = 0;
  }
}
