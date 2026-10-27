/* Every object built exactly, elsewhere (#670, #671).
 *
 * The build is the core's (`brep::export::Builder`): one stepped builder for every object, static
 * or swept, which a page's pool of workers drives a stage at a time and hands back as one buffer
 * (`Display::bytes`: its mesh, its faces named, the B-rep's edges).  The page's own sketch builds
 * no object (`supplyObjects`): it names the jobs (`objectJobs`, one a binding, each keyed by what
 * it reads) and is given each build as it arrives (`supplyObject`), or the build's refusal
 * (`refuseObject`).  The buffer is opaque here: the core reads it, and checks it. */
import { Sketch } from './model.js';
import { core, lastError, takeBytes, takeJson, withBuf, withStr } from './wasm.js';

/** One build the pool makes (`Sketch::object_jobs`): the solid built, the name of the first object
 *  it is, its key — equal on every core exactly when its surface is — the objects it stands for (the
 *  placements of one binding: the core places each from the one build), and whether a sweep is cut
 *  in it, which is built first. */
export interface ObjectJob {
  solid: number;
  name: string;
  key: string;
  objects: number[];
  swept: boolean;
}

/** Where an object stands with the pool (`Sketch::object_state`). */
export interface ObjectState {
  solid: number;
  state: 'built' | 'building' | 'rebuilding' | 'refused';
  why?: string;
}

/** Have this sketch draw every object from what it is supplied, never building one itself (or,
 *  `on` false, build its own again). */
export function supplyObjects(sk: Sketch, on = true): void {
  core().gcs_sketch_supply_objects(sk.handle, on ? 1 : 0);
}

/** What the pool has to build for this sketch: the core's answer. */
export function objectJobs(sk: Sketch): ObjectJob[] {
  return takeJson<ObjectJob[]>(core().gcs_sketch_object_jobs(sk.handle)) ?? [];
}

/** Give the solid a job builds its surface: the buffer a builder wrote (`ExactBuilder.display`).
 *  The core holds it current while what the solid reads is what the build read, within a solve's
 *  tolerance — else it is an earlier drawing's, drawn stale till the new one arrives — and checks
 *  the buffer, refusing one that is not a surface. */
export function supplyObject(sk: Sketch, solid: number, display: Uint8Array): void {
  const ok = withBuf(display.length, 1, (b) => {
    b.u8.set(display);
    return core().gcs_sketch_supply_object(sk.handle, solid, b.ptr, display.length);
  });
  if (ok < 0) throw new Error(lastError());
}

/** The build of the solid a job builds refused, in the build's words. */
export function refuseObject(sk: Sketch, solid: number, why: string): void {
  const ok = withStr(why, (p, n) => core().gcs_sketch_refuse_object(sk.handle, solid, p, n));
  if (ok < 0) throw new Error(lastError());
}

/** Where each object stands with the pool. */
export function objectStates(sk: Sketch): ObjectState[] {
  return takeJson<ObjectState[]>(core().gcs_sketch_object_states(sk.handle)) ?? [];
}

/** How many triangles a build's buffer holds, read off its head (`Display::bytes`), or 0 for one
 *  that is too short to say. */
export function displayTriangles(display: Uint8Array): number {
  if (display.byteLength < 24) return 0;
  return new DataView(display.buffer, display.byteOffset, display.byteLength).getFloat64(16, true);
}

/** Where a build stands: what its next stage does, the stages run and in all, and the last thing a
 *  stage said. */
export interface ExactProgress {
  doing: string;
  done: number;
  total: number;
  said: string | null;
}

/** **An object's exact surface, built a stage at a time** (`brep::export::Builder`): a swept body
 *  admitted, its blank, each sweep's sheet, its sector, the cut, the pattern and the mesh — seconds
 *  a stage for a gear — or a static body built from its recipe and meshed; so a worker steps it
 *  and says between stages what it is doing. */
export class ExactBuilder {
  private constructor(private h: number) {}

  /** The build of solid `idx`, or why it has none (the export's refusal, its stage named). */
  static create(sk: Sketch, idx: number): ExactBuilder {
    const h = core().gcs_exact_builder_new(sk.handle, idx);
    if (!h) throw new Error(lastError());
    return new ExactBuilder(h);
  }

  /** Run the next stage: true once built; a refusal throws, with the stage it was met at. */
  step(): boolean {
    const r = core().gcs_exact_builder_step(this.h);
    if (r < 0) throw new Error(lastError());
    return r === 1;
  }

  progress(): ExactProgress {
    return takeJson<ExactProgress>(core().gcs_exact_builder_progress(this.h));
  }

  /** The built surface, as the one buffer `supplyObject` takes. */
  display(): Uint8Array {
    const handle = core().gcs_exact_builder_display(this.h);
    if (!handle) throw new Error(lastError());
    return takeBytes(handle);
  }

  dispose(): void {
    if (this.h) core().gcs_exact_builder_free(this.h);
    this.h = 0;
  }
}
