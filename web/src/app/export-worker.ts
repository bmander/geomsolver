/* The worker that makes a solid's exact export for the page (`File ▸ Export solid (STEP)`, and
 * the STL held to a tolerance): the core's own kernel builds the solid — a swept body of the
 * generating class as one sector patterned — writes its STEP parsed back against it, or meshes it
 * within the tolerance and holds it to its material field (`gcs_core::brep::export`, the functions
 * `solventc` calls, so a file is one file wherever it is written). That takes seconds, not
 * milliseconds, so it runs here, off the thread that draws and beside the mesh worker, whose
 * refinement an export must not stop. Like that worker it holds its own copy of the core: it
 * elaborates the page's text and writes the page's parameter values over the result. */
import { install } from '../core/modules.js';
import { Document } from '../core/program.js';
import { initCore } from '../core/wasm.js';
import * as solids from '../core/mesh.js';

/** What the page asks: one object's STEP or STL, of one state of the drawing. */
export interface ExportJob {
  id: number;
  text: string;
  /** The modules the page handed over and the text reaches, as `[use name, text]`. */
  modules: [string, string][];
  /** The page's parameter values, so the worker's drawing is this one and not a fresh solve. */
  x: Float64Array;
  solid: number;
  kind: 'step' | 'stl';
  /** Millimetres the export is held to (0: the gross bars). */
  tolerance: number;
}

/** What the worker hands back: the file, or the export's refusal. */
export type ExportReply = { id: number; bytes: Uint8Array } | { id: number; error: string };

const scope = globalThis as unknown as {
  onmessage: ((ev: MessageEvent<ExportJob>) => void) | null;
  postMessage(message: ExportReply, transfer?: Transferable[]): void;
};
const ready = initCore();

scope.onmessage = (ev) => {
  const job = ev.data;
  ready.then(() => {
    install(job.modules);
    const doc = Document.read(job.text);
    doc.sketch.setX(job.x);
    try {
      const bytes = solids.exact(doc.sketch, job.solid, job.kind, job.tolerance);
      scope.postMessage({ id: job.id, bytes }, [bytes.buffer]);
    } finally { doc.dispose(); }
  }).catch((e: unknown) => scope.postMessage({ id: job.id, error: e instanceof Error ? e.message : String(e) }));
};
