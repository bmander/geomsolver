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
 * A job belongs to one state of the drawing. An edit starts another and the worker drops the old
 * one at its next step; surfaces from an old job are ignored here. A job whose drawing has not
 * moved is not repeated, and a new elaboration of the same drawing gets the finished surfaces
 * again without meshing them twice. */
import { deferFields, isSwept, supplyField, type FieldSurface, type MeshProgress } from '../core/field.js';
import { objects } from '../core/mesh.js';
import type { Sketch } from '../core/model.js';
import { related } from '../core/modules.js';
import type { Document } from '../core/program.js';

/** What the page asks of the worker: one drawing, and the swept objects to mesh in it. */
export interface Job {
  id: number;
  text: string;
  /** Every module the text reaches, as `[use name, text]`. */
  modules: [string, string][];
  /** The page's parameter values, so the worker's drawing is this one and not a fresh solve. */
  x: Float64Array;
  solids: number[];
}

/** What the worker hands back: a surface as it stands, or why there is none. */
export interface Frame {
  id: number;
  solid: number;
  surface?: FieldSurface;
  error?: string;
  /** Where its meshing stands, and the time since the job began, in milliseconds. */
  progress?: MeshProgress;
  elapsed?: number;
}

/** One swept object's refinement, as the page tells it (`FieldPreview`'s `progressed`). */
export interface Refining {
  name: string;
  progress?: MeshProgress;
  triangles: number;
  elapsed: number;
  done: boolean;
  error?: string;
}

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

export class FieldPreview {
  private worker: Worker | null = null;
  private job = 0;
  private sketch: Sketch | null = null;
  private key = '';
  private finished = new Map<number, FieldSurface>();
  /** The newest surface of each solid not yet applied, and whether a redraw is in hand. */
  private pending = new Map<number, Frame>();
  private busy = false;

  /** Each swept object of the job in hand, by solid index. */
  private refining = new Map<number, Refining>();

  constructor(private readonly arrived: (error?: string) => void,
              private readonly progressed: (refining: Refining[]) => void = () => {}) {}

  /** Mesh this document's swept objects, unless its drawing is the one already meshed. */
  start(doc: Document): void {
    // no workers (a test's view, node): the sketch meshes a swept solid itself when asked
    if (typeof Worker === 'undefined') return;
    const sk = doc.sketch;
    // only objects are meshed here: a swept solid that is no object (a construction's) is never
    // asked for by the page, so it is never deferred either
    const found = objects(sk).filter((o) => isSwept(sk, o.index));
    const solids = found.map((o) => o.index);
    if (!solids.length) {
      // a drawing with nothing to mesh ends the job in hand: its frames are another drawing's
      this.sketch = sk;
      this.cancel();
      if (this.refining.size) { this.refining.clear(); this.progressed([]); }
      return;
    }
    deferFields(sk);
    const x = sk.getX();
    const modules = related(doc.text).map((f): [string, string] => [f.name, f.text]);
    // the drawing is its text, the modules it uses and its parameter values: an edit to any of
    // them is a new job
    const key = [doc.text, ...modules.flat(), Array.from(x).join(',')].join('\u0000');
    if (key === this.key) {
      if (sk === this.sketch) return;                 // nothing moved: the job in flight stands
      if (solids.every((i) => this.finished.has(i))) {
        this.sketch = sk;
        for (const i of solids) supplyField(sk, i, this.finished.get(i)!);
        this.arrived();
        return;
      }
    }
    this.sketch = sk;
    this.key = key;
    this.finished.clear();
    this.pending.clear();
    this.job += 1;
    this.refining = new Map(found.map((o) => [o.index, { name: o.name, triangles: 0, elapsed: 0, done: false }]));
    this.progressed([...this.refining.values()]);
    if (!this.worker) {
      this.worker = new Worker(new URL('./mesh-worker.bundle.js', import.meta.url), { type: 'module' });
      this.worker.onmessage = (ev: MessageEvent<Frame>) => this.receive(ev.data);
      // a worker that dies says nothing of itself: every solid it was meshing has failed
      this.worker.onerror = (ev: ErrorEvent) => this.failAll(ev.message || 'the meshing worker stopped');
      this.worker.onmessageerror = () => this.failAll('the meshing worker sent what could not be read');
    }
    const job: Job = { id: this.job, text: doc.text, x, solids, modules };
    this.worker.postMessage(job);
  }

  /** End the job in hand: the worker drops it at its next step, and nothing it sent is applied. */
  private cancel(): void {
    this.job += 1;
    this.key = '';
    this.finished.clear();
    this.pending.clear();
    this.worker?.postMessage({ id: this.job, text: '', x: new Float64Array(), solids: [], modules: [] } satisfies Job);
  }

  private failAll(error: string): void {
    for (const r of this.refining.values()) if (!r.done) r.error = error;
    this.progressed([...this.refining.values()]);
    this.key = '';                    // the next edit starts the job again
  }

  /** **Surfaces are applied no faster than the page can draw them.** Every one redraws the box,
   *  its edges and the sheet, which for a surface of fourteen thousand triangles is a third of a
   *  second — longer than the worker takes to send the next — so applying each as it came kept
   *  the page's thread busy until the refinement ended. The newest surface of each solid waits,
   *  and after a redraw the next is applied no sooner than that redraw took: the page is never
   *  more than half busy with a preview, and the finished surface, the last to arrive, is always
   *  applied. */
  private receive(f: Frame): void {
    if (f.id !== this.job || !this.sketch) return;
    const r = this.refining.get(f.solid);
    if (r) {
      r.progress = f.progress ?? r.progress;
      r.elapsed = f.elapsed ?? r.elapsed;
      r.triangles = f.surface ? f.surface.triangles.length / 3 : r.triangles;
      r.done = !!f.surface && !f.surface.provisional;
      r.error = f.error ?? r.error;
      this.progressed([...this.refining.values()]);
    }
    this.pending.set(f.solid, f);
    if (!this.busy) this.apply();
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
          supplyField(sk, f.solid, f.surface);
          if (!f.surface.provisional) this.finished.set(f.solid, f.surface);
        }
      } catch (e) { error = String(e); }
    }
    const start = performance.now();
    try { this.arrived(error); }
    finally { afterPaint(() => setTimeout(() => this.apply(), Math.max(MIN_GAP_MS, performance.now() - start))); }
  }
}
