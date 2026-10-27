/* **The claims about solids are judged in a worker, and the page says how far it has got.**
 *
 * A claim about solids reads the solved pose and enters nothing else the diagnosis says, and
 * judging one is seconds of solids evaluated — a claim over a sweep is 37 poses.  So the page's own
 * diagnosis leaves them out (`diagnose(…, { solidClaims: false })`), and a worker holding its own
 * copy of the core elaborates the same text, takes this sketch's parameter values, and steps a
 * `ClaimJudge` a pose at a time, saying between poses which claim of how many and which pose, in
 * the core's words.  The page is never held for them.
 *
 * A job belongs to one state of the drawing — its text and where it stands: an edit that leaves
 * both as they were leaves the job (or its verdicts) standing, and anything else is a new job,
 * which ends the last at the worker's next pose.  Frames of an old job are ignored here. */
import {
  solidClaims, type ClaimProgress, type ClaimVerdict, type SolidClaim,
} from '../core/claims.js';
import { handed } from '../core/modules.js';
import type { Document } from '../core/program.js';

/** What the page asks of the worker: to judge the claims of one drawing, or to stop. */
export type ClaimJob = {
  kind: 'claims';
  id: number;
  text: string;
  /** The modules the page handed over and the text reaches, as `[use name, text]`. */
  modules: [string, string][];
  /** The page's parameter values, so the worker's drawing is this one and not a fresh solve. */
  x: Float64Array;
  /** The claims as the page's core named them: the worker checks its own drawing names the same. */
  claims: SolidClaim[];
} | { kind: 'cancel'; id: number };

/** What the worker hands back: where the judging stands, the verdicts once all are judged, or why
 *  there are none. */
export interface ClaimFrame {
  id: number;
  progress?: ClaimProgress;
  verdicts?: ClaimVerdict[];
  error?: string;
}

/** The judging as the page tells it: its claims, where it stands, and — once judged — what each
 *  came to; `since` is when the page asked (`performance.now()`), a clock the page keeps
 *  ticking. */
export interface Judging {
  claims: SolidClaim[];
  progress?: ClaimProgress;
  verdicts?: ClaimVerdict[];
  error?: string;
  since: number;
}

/** Why the claims a worker was asked to judge are not the ones its own drawing states, or
 *  nothing: two cores elaborating one text must agree on each, or a verdict would be said of
 *  another claim. */
export function unpairedClaims(asked: SolidClaim[], found: SolidClaim[]): string | undefined {
  const same = (a: SolidClaim, b: SolidClaim): boolean => a.stmt === b.stmt && a.text === b.text;
  if (asked.length !== found.length || asked.some((a, i) => !same(a, found[i]))) {
    return 'the worker\'s drawing does not state the claims the page\'s does';
  }
  return undefined;
}

/** The worker, as this file uses it: a test hands in a stub. */
export interface JudgeWorker {
  postMessage(job: ClaimJob): void;
  onmessage: ((ev: MessageEvent<ClaimFrame>) => void) | null;
  onerror: ((ev: ErrorEvent) => void) | null;
  onmessageerror: ((ev: MessageEvent) => void) | null;
}

const spawnWorker = (): JudgeWorker =>
  new Worker(new URL('./mesh-worker.bundle.js', import.meta.url),
    { type: 'module' }) as unknown as JudgeWorker;

/** Whether two parameter vectors are the same pose, bit for bit. */
export function samePose(a: Float64Array, b: Float64Array): boolean {
  return a.length === b.length && a.every((v, i) => Object.is(v, b[i]));
}

export class ClaimJudging {
  private worker: JudgeWorker | null = null;
  private job = 0;
  /** The drawing the job in hand (or its verdicts) is of. */
  private of: { text: string; x: Float64Array } | null = null;
  private state: Judging | null = null;

  /** `told` hears the judging as it moves, and null once the drawing has no claims to judge; with
   *  no worker (a test's view, node) nothing is judged and nothing told. */
  constructor(private readonly told: (j: Judging | null) => void,
              private readonly spawn: (() => JudgeWorker) | null =
                typeof Worker === 'undefined' ? null : spawnWorker) {}

  /** What the judging stands at, if the drawing has claims. */
  get judging(): Judging | null { return this.state; }

  /** Judge this document's claims about solids at the pose it stands in — unless the job in hand
   *  (or its verdicts) is of this very drawing. */
  start(doc: Document): void {
    if (!this.spawn) return;
    const claims = solidClaims(doc.sketch);
    if (!claims.length) {
      if (this.of) this.cancel();
      this.of = null;
      if (this.state) { this.state = null; this.told(null); }
      return;
    }
    const x = doc.sketch.getX();
    if (this.of && this.of.text === doc.text && samePose(this.of.x, x)) return;
    this.job += 1;
    this.of = { text: doc.text, x };
    this.state = { claims, since: performance.now() };
    if (!this.worker) {
      const w = this.spawn();
      w.onmessage = (ev) => this.receive(ev.data);
      // a worker that dies says nothing of itself: the claims it was judging are unjudged
      w.onerror = (ev) => this.fail(ev.message || 'the worker judging the claims stopped');
      w.onmessageerror =
        () => this.fail('the worker judging the claims sent what could not be read');
      this.worker = w;
    }
    const modules = handed(doc.text);
    this.worker.postMessage({ kind: 'claims', id: this.job, text: doc.text, modules, x, claims });
    this.told(this.state);
  }

  /** End the job in hand: the worker drops it at its next pose. */
  private cancel(): void {
    this.job += 1;
    this.worker?.postMessage({ kind: 'cancel', id: this.job });
  }

  private fail(error: string): void {
    if (!this.state) return;
    this.state = { ...this.state, error };
    this.of = null;                   // the next edit asks again
    this.told(this.state);
  }

  private receive(f: ClaimFrame): void {
    if (f.id !== this.job || !this.state) return;
    if (f.error) return this.fail(f.error);
    const { progress, verdicts } = this.state;
    this.state = { ...this.state, progress: f.progress ?? progress, verdicts: f.verdicts ?? verdicts };
    this.told(this.state);
  }
}
