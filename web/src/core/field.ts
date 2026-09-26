/* A swept solid's surface, meshed from its material field a step at a time.
 *
 * The refinement is the core's (`solid::FieldMesher`); this is the handle a worker drives and the
 * two calls a page makes with what the worker hands back: `deferFields`, so the page's own sketch
 * never meshes a field on the thread that draws, and `supplyField`, which gives it each surface as
 * it arrives.  Numbers cross as flat buffers — three doubles a vertex, three indices a triangle. */
import { Sketch } from './model.js';
import { core, lastError, takeJson, withBuf } from './wasm.js';

/** A surface in world coordinates: `provisional` while the refinement is still going. */
export interface FieldSurface {
  vertices: Float64Array;
  triangles: Uint32Array;
  provisional: boolean;
}

/** One surface for a page's worker to mesh (`Sketch::field_jobs`): a swept object, and the key
 *  of the drawing it is a surface of — equal on every core exactly when the surface is. */
export interface FieldJob {
  solid: number;
  name: string;
  key: string;
}

/** What this sketch's worker has to mesh: the core's answer, which objects are swept and what
 *  each is a surface of. */
export function fieldJobs(sk: Sketch): FieldJob[] {
  return takeJson<FieldJob[]>(core().gcs_sketch_field_jobs(sk.handle)) ?? [];
}

/** Why the solids a worker was asked to mesh are not the ones its own drawing names, or nothing:
 *  two cores elaborating one text must agree on each solid's index, name and key, or the surface
 *  meshed would be supplied to another solid. */
export function unpaired(asked: FieldJob[], found: FieldJob[]): string | undefined {
  for (const a of asked) {
    const f = found.find((j) => j.solid === a.solid);
    if (!f || f.name !== a.name || f.key !== a.key) {
      return `the worker's drawing does not have \`${a.name}\` as the page's has it`;
    }
  }
  return undefined;
}

/** Have this sketch refuse a swept solid it has been given no surface for, rather than mesh it. */
export function deferFields(sk: Sketch, on = true): void {
  core().gcs_sketch_defer_fields(sk.handle, on ? 1 : 0);
}

/** Give swept solid `idx` a surface meshed elsewhere, against the drawing as it stands now. */
export function supplyField(sk: Sketch, idx: number, s: FieldSurface): void {
  const nv = s.vertices.length / 3, nt = s.triangles.length / 3;
  const ok = withBuf(s.vertices.length, 8, (v) => {
    v.set(s.vertices);
    return withBuf(s.triangles.length, 4, (t) => {
      t.set(s.triangles);
      return core().gcs_solid_supply_field(sk.handle, idx, v.ptr, nv, t.ptr, nt, s.provisional ? 1 : 0);
    });
  });
  if (ok < 0) throw new Error(lastError());
}

/** Where a mesher stands (`FieldProgress` in the core): `doing`, the core's words for what it is
 *  doing, which a page shows as they are; its phase, the refinement's stage and rebuilds, the
 *  facets waiting and the worst of them, the work done, the creases traced once they are, and two
 *  estimates — `within` the pass in hand, `fraction` the whole — never a count of work left. */
export interface MeshProgress {
  doing: string;
  phase: 'first pass' | 'tracing edges' | 'final pass';
  stage: 'building' | 'refining' | 'repairing' | 'done' | 'failed';
  rebuild: number;
  queued: number;
  worst: number;
  inserted: number;
  queries: number;
  readings: number;
  curves: number | null;
  within: number;
  fraction: number;
  failed: boolean;
}

/** Delaunay refinement of one swept solid's field, driven by `step` and read by `snapshot`. */
export class FieldMesher {
  private constructor(private h: number) {}

  static create(sk: Sketch, idx: number): FieldMesher {
    const h = core().gcs_field_mesher_new(sk.handle, idx);
    if (!h) throw new Error(lastError() || 'could not start meshing the solid');
    return new FieldMesher(h);
  }

  /** Refine at most about `budget` facets: whether the refinement has finished. */
  step(budget: number): boolean {
    const r = core().gcs_field_mesher_step(this.h, budget);
    if (r < 0) throw new Error(lastError() || 'the refinement failed');
    return r === 1;
  }

  /** Where the meshing stands. */
  progress(): MeshProgress {
    return takeJson<MeshProgress>(core().gcs_field_mesher_progress(this.h));
  }

  /** The surface as it stands. */
  snapshot(): FieldSurface {
    const c = core();
    const [nv, nt, provisional] = withBuf(3, 4, (b) => {
      if (c.gcs_field_mesher_snapshot(this.h, b.ptr) < 0) throw new Error(lastError() || 'the surface could not be read');
      return Array.from(b.i32.slice(0, 3));
    });
    const vertices = withBuf(3 * nv, 8, (b) => {
      if (c.gcs_field_mesher_vertices(this.h, b.ptr, 3 * nv) < 0) throw new Error(lastError());
      return b.f64.slice(0, 3 * nv);
    });
    const triangles = withBuf(3 * nt, 4, (b) => {
      if (c.gcs_field_mesher_triangles(this.h, b.ptr, 3 * nt) < 0) throw new Error(lastError());
      // the same bits read as unsigned: the buffer holds `u32` indices
      return new Uint32Array(b.i32.slice(0, 3 * nt).buffer);
    });
    return { vertices, triangles, provisional: provisional !== 0 };
  }

  dispose(): void {
    if (this.h) core().gcs_field_mesher_free(this.h);
    this.h = 0;
  }
}
