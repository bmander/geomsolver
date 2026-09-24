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
import { deferFields, isSwept, supplyField, type FieldSurface } from '../core/field.js';
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

  constructor(private readonly arrived: (error?: string) => void) {}

  /** Mesh this document's swept objects, unless its drawing is the one already meshed. */
  start(doc: Document): void {
    // no workers (a test's view, node): the sketch meshes a swept solid itself when asked
    if (typeof Worker === 'undefined') return;
    const sk = doc.sketch;
    const solids = objects(sk).map((o) => o.index).filter((i) => isSwept(sk, i));
    if (!solids.length) {
      this.sketch = sk;
      return;
    }
    deferFields(sk);
    const x = sk.getX();
    const key = `${doc.text}\u0000${Array.from(x).join(',')}`;
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
    if (!this.worker) {
      this.worker = new Worker(new URL('./mesh-worker.bundle.js', import.meta.url), { type: 'module' });
      this.worker.onmessage = (ev: MessageEvent<Frame>) => this.receive(ev.data);
    }
    const job: Job = { id: this.job, text: doc.text, x, solids,
      modules: related(doc.text).map((f) => [f.name, f.text]) };
    this.worker.postMessage(job);
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
    let error: string | undefined;
    for (const f of frames) {
      if (f.surface) {
        // one mark a surface, triangles in its detail: the refinement on a performance timeline
        performance.mark('field-surface', { detail: { solid: f.solid, triangles: f.surface.triangles.length / 3,
          provisional: f.surface.provisional } });
        supplyField(sk, f.solid, f.surface);
        if (!f.surface.provisional) this.finished.set(f.solid, f.surface);
      }
      error = f.error ?? error;
    }
    const start = performance.now();
    this.arrived(error);
    afterPaint(() => setTimeout(() => this.apply(), Math.max(MIN_GAP_MS, performance.now() - start)));
  }
}
