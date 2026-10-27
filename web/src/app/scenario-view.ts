/* **A load scenario is read in a worker, and the page draws what it says.**
 *
 * What every scenario answers, and how the drawing moves under the one chosen, is one reading
 * (`Document.scenariosPictured`): each body a scenario falls on is meshed and solved once for
 * both.  A solid read in three dimensions takes seconds, so the reading is a worker's — its own
 * instance of `mesh-worker`, holding its own copy of the core — which elaborates the page's text,
 * takes the page's parameter values and posts the answers and the picture back.  The page is
 * never held for them.  The worker reads a level at a time (`Document.scenarioLevels`): its
 * elements four times their size, then twice, then their own, posting each, so a first answer
 * and picture come in a fraction of the time and an edit ends the job before the finer levels.  Where there is no worker (node, a test), the reading is made at once on
 * the page's own thread, so a test sees what the page would.
 *
 * A job belongs to one state of the drawing and one choice — its text, where it stands, and the
 * scenario drawn: an edit that leaves all three as they were leaves the job (or its reading)
 * standing, and anything else is a new job.  Frames of an old job are ignored here.
 *
 * Nothing here is geometry: the picture's arrays are the core's, handed to the box as they came,
 * typed so the worker can transfer them rather than copy. */
import { handed } from '../core/modules.js';
import { samePose } from './claim-judging.js';
import type { Document, Scenario, ScenarioAnswer } from '../core/program.js';
import type { PictureBody, ScenarioPicture } from '../core/scenario.js';

/** One body of a picture as the page holds it: the core's arrays, typed. */
export interface DrawnBody {
  kind: PictureBody['kind'];
  name: string;
  solid: number | null;
  rest: Float32Array;
  moved: Float32Array;
  size: Float32Array;
  triangles: Uint32Array;
  edges: Uint32Array;
}

/** A picture as the page holds it. */
export interface Drawn extends Omit<ScenarioPicture, 'bodies'> {
  bodies: DrawnBody[];
}

/** What the page asks of the worker: to read the scenarios of one drawing and draw one, or to
 *  stop. */
export type ScenarioJob = {
  kind: 'scenario';
  id: number;
  text: string;
  /** The modules the page handed over and the text reaches, as `[use name, text]`. */
  modules: [string, string][];
  /** The page's parameter values, so the worker's drawing is this one and not a fresh solve. */
  x: Float64Array;
  /** Where the page's meshes were made again, if they were (`Sketch.meshedAt`): the worker's
   *  are made there, so a driven row reads on the mesh it was solved on. */
  meshedAt: Float64Array | null;
  /** The scenario to draw, by name — the worker checks its own drawing names it alike — or ''. */
  drawn: string;
} | { kind: 'cancel'; id: number };

/** What the worker hands back: a level of the reading — the last `final` — or why there is
 *  none.  A level before the last says what it is (`levelText`, the core's words). */
export interface ScenarioFrame {
  id: number;
  scenarios?: Scenario[];
  measures?: ScenarioAnswer[];
  picture?: Drawn | null;
  final?: boolean;
  levelText?: string;
  error?: string;
}

/** **The reading as the page tells it**: every scenario's answers, what the drawing's masses and
 *  volumes come to, the picture of the one drawn (null for none, or while it is being read),
 *  whether a job is still out, and why it failed.  While a finer level is being read, the answers
 *  and the picture are a coarser level's, and `levelText` says which (`report::Levels`). */
export interface Reading {
  scenarios: Scenario[] | null;
  measures: ScenarioAnswer[] | null;
  picture: Drawn | null;
  drawn: string;
  reading: boolean;
  levelText?: string;
  error?: string;
}

/** The picture's arrays, typed: what the worker posts, its buffers transferred. */
export function typed(p: ScenarioPicture | null): Drawn | null {
  if (!p) return null;
  return {
    ...p,
    bodies: p.bodies.map((b) => ({
      kind: b.kind, name: b.name, solid: b.solid,
      rest: Float32Array.from(b.rest), moved: Float32Array.from(b.moved),
      size: Float32Array.from(b.size), triangles: Uint32Array.from(b.triangles),
      edges: Uint32Array.from(b.edges),
    })),
  };
}

/** The buffers a picture holds, to hand over rather than copy. */
export function buffers(p: Drawn | null | undefined): ArrayBuffer[] {
  if (!p) return [];
  return p.bodies.flatMap((b) => [b.rest, b.moved, b.size, b.triangles, b.edges]
    .map((a) => a.buffer as ArrayBuffer));
}

/** Scenario `drawn`'s index among the document's (by name, '' for none), −1 for none. */
export function drawnIndex(doc: Document, drawn: string): number {
  return drawn ? doc.scenarioNames().findIndex((s) => s.name === drawn) : -1;
}

/** **Read a document's scenarios and draw scenario `drawn`** (by name, '' for none), where the
 *  document stands, at once and at the readers' own size: what the page does with no worker. */
export function readScenarios(doc: Document, drawn: string)
  : { scenarios: Scenario[]; measures: ScenarioAnswer[]; picture: Drawn | null } {
  const index = drawnIndex(doc, drawn);
  const got = doc.scenariosPictured(index);
  return { scenarios: got.scenarios, measures: got.measures ?? [],
           picture: index >= 0 ? typed(got.picture) : null };
}

/** The worker, as this file uses it: a test hands in a stub. */
export interface ReadWorker {
  postMessage(job: ScenarioJob): void;
  onmessage: ((ev: MessageEvent<ScenarioFrame>) => void) | null;
  onerror: ((ev: ErrorEvent) => void) | null;
  onmessageerror: ((ev: MessageEvent) => void) | null;
}

const spawnWorker = (): ReadWorker =>
  new Worker(new URL('./mesh-worker.bundle.js', import.meta.url),
    { type: 'module' }) as unknown as ReadWorker;

export class ScenarioReading {
  private worker: ReadWorker | null = null;
  private job = 0;
  /** The drawing and the choice the job in hand (or its reading) is of. */
  private of: { text: string; x: Float64Array; meshedAt: Float64Array | null; drawn: string }
    | null = null;
  private state: Reading | null = null;

  /** `told` hears the reading as it moves, and null once the drawing has no scenario; `spawn`
   *  makes the worker, null to read on this thread (node, a test). */
  constructor(private readonly told: (r: Reading | null) => void,
              private readonly spawn: (() => ReadWorker) | null =
                typeof Worker === 'undefined' ? null : spawnWorker) {}

  /** What the reading stands at, if the drawing has a scenario. */
  get reading(): Reading | null { return this.state; }

  /** Read this document's scenarios where it stands, and draw `drawn` — unless the job in hand
   *  (or its reading) is of this very drawing and choice. */
  start(doc: Document, drawn: string): void {
    if (!doc.scenarioNames().length && !doc.measuresStated()) {
      if (this.of) this.cancel();
      this.of = null;
      if (this.state) { this.state = null; this.told(null); }
      return;
    }
    const x = doc.sketch.getX();
    const meshedAt = doc.sketch.meshedAt();
    const sameMeshes = (a: Float64Array | null) =>
      a === null || meshedAt === null ? a === meshedAt : samePose(a, meshedAt);
    if (this.of && this.of.text === doc.text && this.of.drawn === drawn
        && samePose(this.of.x, x) && sameMeshes(this.of.meshedAt)) return;
    this.job += 1;
    this.of = { text: doc.text, x, meshedAt, drawn };
    // what was said of the last drawing stands until this one is read: the answers are kept, so
    // the footer does not blink empty on every edit, and the picture goes, since it is of a pose
    // the drawing has left
    this.state = { scenarios: this.state?.scenarios ?? null,
                   measures: this.state?.measures ?? null, picture: null, drawn, reading: true };
    if (!this.spawn) {
      try {
        const got = readScenarios(doc, drawn);
        this.state = { scenarios: got.scenarios, measures: got.measures, picture: got.picture,
                       drawn, reading: false };
      } catch (e) {
        this.state = { ...this.state, reading: false, error: String(e) };
      }
      this.told(this.state);
      return;
    }
    if (!this.worker) {
      const w = this.spawn();
      w.onmessage = (ev) => this.receive(ev.data);
      // a worker that dies says nothing of itself: the scenarios it was reading are unread
      w.onerror = (ev) => this.fail(ev.message || 'the worker reading the scenarios stopped');
      w.onmessageerror =
        () => this.fail('the worker reading the scenarios sent what could not be read');
      this.worker = w;
    }
    const modules = handed(doc.text);
    this.worker.postMessage(
      { kind: 'scenario', id: this.job, text: doc.text, modules, x, meshedAt, drawn });
    this.told(this.state);
  }

  /** End the job in hand: the worker drops it before it starts. */
  private cancel(): void {
    this.job += 1;
    this.worker?.postMessage({ kind: 'cancel', id: this.job });
  }

  private fail(error: string): void {
    if (!this.state) return;
    this.state = { ...this.state, reading: false, error };
    this.of = null;                   // the next edit asks again
    this.told(this.state);
  }

  /** A level arrived: its answers and its picture stand — a coarser level's still reading, with
   *  what it is — until the next; the last ends the job. */
  private receive(f: ScenarioFrame): void {
    if (f.id !== this.job || !this.state) return;
    if (f.error) return this.fail(f.error);
    const reading = f.final === false;
    this.state = { scenarios: f.scenarios ?? [], measures: f.measures ?? [],
                   picture: f.picture ?? null, drawn: this.state.drawn, reading,
                   ...(reading && f.levelText ? { levelText: f.levelText } : {}) };
    this.told(this.state);
  }
}
