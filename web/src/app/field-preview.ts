/* **Swept solids refine in a worker, and the page draws each surface as it arrives.**
 *
 * A solid with a continuous sweep among its operands is meshed from its material field by
 * Delaunay refinement, which takes seconds — too long for the thread that draws. So the page's
 * sketch is told never to mesh one itself (`deferFields`), and a worker holding its own copy of
 * the core elaborates the same text, takes this sketch's parameter values, and refines each swept
 * object a step at a time, posting the surface as it stands every so often. Worst facet first
 * means the first surface is the rough shape and every later one a finer version of it, like a
 * path-traced viewport; each is handed to the sketch (`supplyField`) and the page redraws.
 *
 * **What to mesh is the core's answer, and this file compares keys.** `fieldJobs` names each
 * swept object with the key of the drawing it is a surface of — equal exactly when the surface
 * is — so a surface finished once is supplied to every later elaboration of that drawing, a job
 * already asking for the same keys stands, and anything else is a new job. A job belongs to one
 * state of the drawing: a new one, or a `cancel`, ends the last at the worker's next step, and
 * frames of an old job are ignored here. */
import { deferFields, fieldJobs, supplyField, type ExactProgress, type FieldJob, type FieldSurface, type MeshProgress } from '../core/field.js';
import type { Sketch } from '../core/model.js';
import { related } from '../core/modules.js';
import type { Document } from '../core/program.js';

/** What the page asks of a worker: to mesh these swept objects of one drawing, to build their
 *  exact surfaces, or to stop. */
export type Job = MeshJob | ExactJob | { kind: 'cancel'; id: number };

/** Build these swept objects' exact surfaces (`ExactBuilder`): the same drawing as a mesh job's. */
export interface ExactJob {
  kind: 'exact';
  id: number;
  text: string;
  modules: [string, string][];
  x: Float64Array;
  solids: FieldJob[];
}

export interface MeshJob {
  kind: 'mesh';
  id: number;
  text: string;
  /** The modules the page handed over and the text reaches, as `[use name, text]`: the library's
   *  own the worker's core has already. */
  modules: [string, string][];
  /** The page's parameter values, so the worker's drawing is this one and not a fresh solve. */
  x: Float64Array;
  /** What to mesh, as the page's core named it: the worker checks its own drawing names the same. */
  solids: FieldJob[];
  /** How much finer than the preview's to refine (`FieldMesher.create`). */
  fineness: number;
}

/** What the worker hands back: a surface as it stands, or why there is none. */
export interface Frame {
  id: number;
  solid: number;
  key: string;
  surface?: FieldSurface;
  error?: string;
  /** Where its meshing stands, and the time since the job began, in milliseconds. */
  progress?: MeshProgress;
  elapsed?: number;
  /** An exact job's: where its build stands, or why it has none. */
  exact?: ExactProgress;
  exactError?: string;
}

/** One swept object's refinement, as the page tells it (`FieldPreview`'s `progressed`): its field's
 *  preview, and its exact surface's build beside it. */
export interface Refining {
  name: string;
  progress?: MeshProgress;
  triangles: number;
  elapsed: number;
  done: boolean;
  error?: string;
  /** When the page asked for it (`performance.now()`): a clock the page keeps ticking, since a
   *  stage of an exact build is seconds with nothing said. */
  since: number;
  exact?: ExactStatus;
}

/** A swept object's name as the page shows it: its last word, a body's own name rather than `body`. */
export function objectName(name: string): string {
  return name.split('.').filter((w) => w !== 'body').pop() ?? name;
}

/** An exact surface's build as the page tells it: its next stage, the stages run and in all, the
 *  last thing a stage said, and whether it is built, or why it will not be. */
export interface ExactStatus {
  doing: string;
  done: number;
  total: number;
  said?: string;
  built: boolean;
  triangles: number;
  error?: string;
}

/** The most workers building exact surfaces at once. */
const EXACT_WORKERS = 4;

/** The least time between two surfaces applied, however cheap the redraw. */
const MIN_GAP_MS = 60;

/** Once the frame now being drawn has been painted: two animation frames, since the first runs
 *  before that paint. Without animation frames (a test's page) or with the page hidden (which
 *  runs none, and paints nothing), at once. */
function afterPaint(then: () => void): void {
  if (typeof requestAnimationFrame === 'undefined' || typeof document === 'undefined' || document.hidden) {
    setTimeout(then, 0);
  }
  else requestAnimationFrame(() => requestAnimationFrame(then));
}

/** The worker, as this file uses it: a test hands in a stub. */
export interface MeshWorker {
  postMessage(job: Job): void;
  onmessage: ((ev: MessageEvent<Frame>) => void) | null;
  onerror: ((ev: ErrorEvent) => void) | null;
  onmessageerror: ((ev: MessageEvent) => void) | null;
}

const spawnWorker = (): MeshWorker =>
  new Worker(new URL('./mesh-worker.bundle.js', import.meta.url), { type: 'module' }) as unknown as MeshWorker;

export class FieldPreview {
  private worker: MeshWorker | null = null;
  private job = 0;
  private sketch: Sketch | null = null;
  /** The keys the job in hand asks for and has not finished. */
  private asked = new Set<string>();
  /** Finished surfaces by key: supplied to any later sketch of the same drawing. */
  private finished = new Map<string, FieldSurface>();
  /** The newest surface of each key not yet applied, and whether a redraw is in hand. */
  private pending = new Map<string, Frame>();
  private busy = false;
  /** How much finer than the preview's the surfaces are refined: view state, like the orbit. */
  private fine = 1;

  /** The workers building exact surfaces (one an object, up to `EXACT_WORKERS`: a gear's members
   *  are built side by side), their job in hand and the keys it asks for. */
  private exactWorkers: MeshWorker[] = [];
  private exactJob = 0;
  private exactAsked = new Set<string>();
  /** Exact surfaces by key: the drawing's own, whatever the fineness, and never meshed again. */
  private exact = new Map<string, FieldSurface>();
  /** Keys whose exact surface will not be built (refused, or failed): their field's is the surface. */
  private noExact = new Set<string>();

  /** Each swept object of the jobs in hand, by key. */
  private refining = new Map<string, Refining>();

  /** `spawn` makes the meshing worker and `spawnExact` the one building exact surfaces; with no
   *  workers (a test's view, node) a sketch meshes a swept solid itself when asked, and a test
   *  that hands in no `spawnExact` sees field meshing alone. */
  constructor(private readonly arrived: (error?: string) => void,
              private readonly progressed: (refining: Refining[]) => void = () => {},
              private readonly spawn: (() => MeshWorker) | null = typeof Worker === 'undefined' ? null : spawnWorker,
              private readonly spawnExact: (() => MeshWorker) | null = typeof Worker === 'undefined' ? null : spawnWorker) {}

  /** Mesh this document's swept objects, but none whose surface is finished or in hand; and build
   *  the exact surface of each that has none, the field's preview drawn till it arrives. */
  start(doc: Document): void {
    if (!this.spawn) return;
    const sk = doc.sketch;
    const jobs = fieldJobs(sk);
    this.sketch = sk;
    if (!jobs.length) {
      // a drawing with nothing to mesh ends the jobs in hand: their frames are another drawing's
      this.cancel();
      this.cancelExact();
      this.finished.clear();
      this.exact.clear();
      this.noExact.clear();
      if (this.refining.size) { this.refining.clear(); this.progressed([]); }
      return;
    }
    deferFields(sk);
    const keys = new Set(jobs.map((j) => j.key));
    for (const k of [...this.finished.keys()]) if (!keys.has(k)) this.finished.delete(k);
    for (const k of [...this.exact.keys()]) if (!keys.has(k)) this.exact.delete(k);
    for (const k of [...this.noExact]) if (!keys.has(k)) this.noExact.delete(k);
    for (const k of [...this.refining.keys()]) if (!keys.has(k)) this.refining.delete(k);
    let supplied = false;
    for (const j of jobs) {
      const s = this.exact.get(j.key) ?? this.finished.get(j.key);
      if (s) { supplyField(sk, j.solid, s); supplied = true; }
    }
    const modules = related(doc.text).filter((f) => f.provided).map((f): [string, string] => [f.name, f.text]);
    this.startExact(doc, jobs.filter((j) => !this.exact.has(j.key) && !this.noExact.has(j.key)), modules);
    const todo = jobs.filter((j) => !this.finished.has(j.key) && !this.exact.has(j.key));
    // the job in hand asks for exactly these: it stands, and its frames are this sketch's too
    if (todo.length && todo.length === this.asked.size && todo.every((j) => this.asked.has(j.key))) {
      if (supplied) this.arrived();
      return;
    }
    if (!todo.length) {
      if (this.asked.size) this.cancel();
      if (supplied) this.arrived();
      return;
    }
    this.job += 1;
    this.asked = new Set(todo.map((j) => j.key));
    this.pending.clear();
    for (const j of todo) {
      const r = this.track(j);
      Object.assign(r, { triangles: 0, elapsed: 0, done: false, error: undefined, progress: undefined });
    }
    this.progressed([...this.refining.values()]);
    if (!this.worker) {
      const w = this.spawn();
      w.onmessage = (ev) => this.receive(ev.data);
      // a worker that dies says nothing of itself: every solid it was meshing has failed
      w.onerror = (ev) => this.failAll(ev.message || 'the meshing worker stopped');
      w.onmessageerror = () => this.failAll('the meshing worker sent what could not be read');
      this.worker = w;
    }
    this.worker.postMessage({ kind: 'mesh', id: this.job, text: doc.text, x: sk.getX(), solids: todo, modules,
      fineness: this.fine });
    if (supplied) this.arrived();
  }

  /** The status of swept object `j`, made where there is none. */
  private track(j: FieldJob): Refining {
    let r = this.refining.get(j.key);
    if (!r) {
      r = { name: j.name, triangles: 0, elapsed: 0, done: false, since: performance.now() };
      this.refining.set(j.key, r);
    }
    return r;
  }

  /** Build the exact surfaces of `todo` (the job in hand stands if it asks for exactly these). */
  private startExact(doc: Document, todo: FieldJob[], modules: [string, string][]): void {
    if (!this.spawnExact) return;
    if (todo.length === this.exactAsked.size && todo.every((j) => this.exactAsked.has(j.key))) return;
    if (!todo.length) { this.cancelExact(); return; }
    this.exactJob += 1;
    this.exactAsked = new Set(todo.map((j) => j.key));
    for (const j of todo) {
      const r = this.track(j);
      r.since = performance.now();
      r.exact = { doing: 'starting', done: 0, total: 1, built: false, triangles: 0 };
    }
    this.progressed([...this.refining.values()]);
    while (this.exactWorkers.length < Math.min(todo.length, EXACT_WORKERS)) {
      const w = this.spawnExact();
      w.onmessage = (ev) => this.receiveExact(ev.data);
      w.onerror = (ev) => this.failExact(ev.message || 'an exact worker stopped');
      w.onmessageerror = () => this.failExact('an exact worker sent what could not be read');
      this.exactWorkers.push(w);
    }
    // the objects dealt round the workers; every one is told the job, so an idle one drops the last
    const x = doc.sketch.getX();
    this.exactWorkers.forEach((w, k) => w.postMessage({ kind: 'exact', id: this.exactJob, text: doc.text, x,
      solids: todo.filter((_, i) => i % this.exactWorkers.length === k), modules }));
  }

  get fineness(): number { return this.fine; }

  /** Refine `fineness` times finer than the preview's. A surface finished at another fineness is
   *  not this one's, so every swept object of `doc` (the view's, now) is meshed again — but one with
   *  its exact surface, which no fineness changes; until its first frame arrives the sketch keeps
   *  drawing the surface it has. */
  setFineness(fineness: number, doc: Document): void {
    if (fineness === this.fine) return;
    this.fine = fineness;
    this.finished.clear();
    if (this.asked.size) this.cancel();
    this.start(doc);
  }

  /** End the job in hand: the worker drops it at its next step, and nothing it sent is applied. */
  private cancel(): void {
    this.job += 1;
    this.asked.clear();
    this.pending.clear();
    this.worker?.postMessage({ kind: 'cancel', id: this.job });
  }

  /** End the exact job in hand, at its next stage. */
  private cancelExact(): void {
    this.exactJob += 1;
    this.exactAsked.clear();
    for (const w of this.exactWorkers) w.postMessage({ kind: 'cancel', id: this.exactJob });
  }

  private failAll(error: string): void {
    for (const r of this.refining.values()) if (!r.done) r.error = error;
    this.progressed([...this.refining.values()]);
    this.asked.clear();               // the next edit starts the job again
  }

  private failExact(error: string): void {
    for (const [key, r] of this.refining) {
      if (r.exact && !r.exact.built) { r.exact.error = error; this.noExact.add(key); }
    }
    this.progressed([...this.refining.values()]);
    this.exactAsked.clear();
  }

  /** **Surfaces are applied no faster than the page can draw them.** Every one redraws the box,
   *  its edges and the sheet, which for a surface of fourteen thousand triangles is a third of a
   *  second — longer than the worker takes to send the next — so applying each as it came kept
   *  the page's thread busy until the refinement ended. The newest surface of each solid waits,
   *  and after a redraw the next is applied no sooner than that redraw took: the page is never
   *  more than half busy with a preview, and the finished surface, the last to arrive, is always
   *  applied. */
  private receive(f: Frame): void {
    if (f.id !== this.job || !this.sketch || !this.asked.has(f.key)) return;
    const r = this.refining.get(f.key);
    if (r) {
      r.progress = f.progress ?? r.progress;
      r.elapsed = f.elapsed ?? r.elapsed;
      r.triangles = f.surface ? f.surface.triangles.length / 3 : r.triangles;
      r.done = !!f.surface && !f.surface.provisional;
      r.error = f.error ?? r.error;
      this.progressed([...this.refining.values()]);
    }
    this.pending.set(f.key, f);
    if (!this.busy) this.apply();
  }

  /** An exact build's frame: its stage said, and once built its surface applied in place of the
   *  field's, whose job then asks for it no longer (and ends, asking for nothing else). */
  private receiveExact(f: Frame): void {
    if (f.id !== this.exactJob || !this.sketch || !this.exactAsked.has(f.key)) return;
    const r = this.refining.get(f.key);
    if (f.exactError) {
      this.exactAsked.delete(f.key);
      this.noExact.add(f.key);
      if (r?.exact) r.exact.error = f.exactError;
      this.progressed([...this.refining.values()]);
      return;
    }
    if (r?.exact && f.exact) {
      Object.assign(r.exact, { doing: f.exact.doing, done: f.exact.done, total: f.exact.total, said: f.exact.said ?? r.exact.said });
    }
    if (f.surface?.exact) {
      this.exactAsked.delete(f.key);
      this.exact.set(f.key, f.surface);
      if (r?.exact) { r.exact.built = true; r.exact.triangles = f.surface.triangles.length / 3; }
      if (this.asked.delete(f.key) && !this.asked.size) this.cancel();
      this.pending.set(f.key, f);
      if (!this.busy) this.apply();
    }
    this.progressed([...this.refining.values()]);
  }

  private apply(): void {
    const sk = this.sketch;
    if (!this.pending.size || !sk) {
      this.busy = false;
      return;
    }
    this.busy = true;
    const frames = [...this.pending.values()];
    this.pending.clear();
    // a frame's own error is the footer's to say (`receive`); only a surface the page refuses
    // is said here, and it does not stop the others, or every later one
    let error: string | undefined;
    for (const f of frames) {
      try {
        if (f.surface) {
          // one key is one solid of one drawing: the index the job named is this sketch's too
          supplyField(sk, f.solid, f.surface);
          // finished: no longer asked for, so a drawing of it asks nothing and cancels nothing
          if (!f.surface.provisional && !f.surface.exact) { this.finished.set(f.key, f.surface); this.asked.delete(f.key); }
        }
      } catch (e) { error = String(e); }
    }
    const start = performance.now();
    try { this.arrived(error); }
    finally { afterPaint(() => setTimeout(() => this.apply(), Math.max(MIN_GAP_MS, performance.now() - start))); }
  }
}
