/* The worker that meshes swept solids for the page (`field-preview.ts`).
 *
 * It has its own copy of the core: it elaborates the page's text, writes the page's parameter
 * values over the result, and refines every swept object a step at a time, in turn, posting each
 * surface as it stands at most every `FRAME_MS` and when it is finished. Between steps it lets messages in,
 * so a newer job — the page has been edited — ends this one at its next step. */
import { FieldMesher } from '../core/field.js';
import { forget, provide } from '../core/modules.js';
import { Document } from '../core/program.js';
import { initCore } from '../core/wasm.js';
import type { Frame, Job } from './field-preview.js';
import type { MeshProgress } from '../core/field.js';

/** Facets refined between looks at the message queue. */
const BUDGET = 40;
/** The least time between two surfaces posted while refining. */
const FRAME_MS = 120;

const scope = globalThis as unknown as {
  onmessage: ((ev: MessageEvent<Job>) => void) | null;
  postMessage(message: Frame, transfer?: Transferable[]): void;
};
const ready = initCore();
let latest = 0;

scope.onmessage = (ev) => {
  latest = ev.data.id;
  void ready.then(() => run(ev.data));
};

const pause = (): Promise<void> => new Promise((resolve) => setTimeout(resolve, 0));

function post(frame: Frame): void {
  const s = frame.surface;
  scope.postMessage(frame, s ? [s.vertices.buffer, s.triangles.buffer] : []);
}

async function run(job: Job): Promise<void> {
  await pause();                      // a burst of edits: only the last job is started
  if (job.id !== latest) return;
  forget();
  for (const [name, text] of job.modules) provide(name, text);
  let doc: Document;
  try {
    doc = Document.read(job.text);
    doc.sketch.setX(job.x);
  } catch (e) {
    for (const solid of job.solids) post({ id: job.id, solid, error: String(e) });
    return;
  }
  // Every swept object refines in turn, a budget at a time, so each shows its rough shape at
  // once rather than after the ones before it have finished.
  const running: { solid: number; mesher: FieldMesher; shown: number; phase: string }[] = [];
  const started = performance.now();
  const status = (m: FieldMesher): { progress?: MeshProgress; elapsed: number } => {
    let progress: MeshProgress | undefined;
    try { progress = m.progress(); } catch { progress = undefined; }
    return { progress, elapsed: performance.now()-started };
  };
  try {
    for (const solid of job.solids) {
      try {
        running.push({ solid, mesher: FieldMesher.create(doc.sketch, solid), shown: 0, phase: '' });
      } catch (e) {
        post({ id: job.id, solid, error: String(e) });
      }
    }
    while (running.length) {
      for (let k = 0; k < running.length;) {
        if (job.id !== latest) return;
        const r = running[k];
        let done: boolean;
        try {
          done = r.mesher.step(BUDGET);
        } catch (e) {
          post({ id: job.id, solid: r.solid, surface: r.mesher.snapshot(), error: String(e), ...status(r.mesher) });
          r.mesher.dispose();
          running.splice(k, 1);
          continue;
        }
        // a new phase is said at once: the next step may be a long one (the edges traced whole)
        const now = performance.now();
        const s = status(r.mesher);
        const phase = `${s.progress?.phase}/${s.progress?.stage}/${s.progress?.rebuild}`;
        if (done || now - r.shown > FRAME_MS || phase !== r.phase) {
          post({ id: job.id, solid: r.solid, surface: r.mesher.snapshot(), ...s });
          r.shown = now;
          r.phase = phase;
        }
        if (done) {
          r.mesher.dispose();
          running.splice(k, 1);
        } else k++;
        await pause();
      }
    }
  } finally {
    for (const r of running) r.mesher.dispose();
    doc.dispose();
  }
}
