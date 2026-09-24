/* The worker that meshes swept solids for the page (`field-preview.ts`).
 *
 * It has its own copy of the core: it elaborates the page's text, writes the page's parameter
 * values over the result, and refines each swept object a step at a time, posting the surface as
 * it stands at most every `FRAME_MS` and when it is finished. Between steps it lets messages in,
 * so a newer job — the page has been edited — ends this one at its next step. */
import { FieldMesher } from '../core/field.js';
import { forget, provide } from '../core/modules.js';
import { Document } from '../core/program.js';
import { initCore } from '../core/wasm.js';
import type { Frame, Job } from './field-preview.js';

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
  try {
    for (const solid of job.solids) {
      let mesher: FieldMesher;
      try {
        mesher = FieldMesher.create(doc.sketch, solid);
      } catch (e) {
        post({ id: job.id, solid, error: String(e) });
        continue;
      }
      try {
        let shown = 0;
        for (;;) {
          if (job.id !== latest) return;
          let done: boolean;
          try {
            done = mesher.step(BUDGET);
          } catch (e) {
            post({ id: job.id, solid, surface: mesher.snapshot(), error: String(e) });
            break;
          }
          const now = performance.now();
          if (done || now - shown > FRAME_MS) {
            post({ id: job.id, solid, surface: mesher.snapshot() });
            shown = now;
          }
          if (done) break;
          await pause();
        }
      } finally {
        mesher.dispose();
      }
    }
  } finally {
    doc.dispose();
  }
}
