/* A swept solid's surface, meshed from its material field a step at a time.
 *
 * The refinement is the core's (`solid::FieldMesher`); this is the handle a worker drives and the
 * two calls a page makes with what the worker hands back: `deferFields`, so the page's own sketch
 * never meshes a field on the thread that draws, and `supplyField`, which gives it each surface as
 * it arrives.  Numbers cross as flat buffers — three doubles a vertex, three indices a triangle. */
import { Sketch } from './model.js';
import { core, lastError, takeBytes, takeJson, withBuf } from './wasm.js';

/** A surface in world coordinates: `provisional` while the refinement is still going, and `exact`
 *  where it is no field's surface but the exact B-rep's mesh (`ExactBuilder`). */
export interface FieldSurface {
  vertices: Float64Array;
  triangles: Uint32Array;
  provisional: boolean;
  exact?: ExactFaces;
}

/** What an exact surface says of itself: the face each triangle is of, whether each face is
 *  curved (1) or flat (0), and the solid's exact volume. */
export interface ExactFaces {
  of: Uint32Array;
  smooth: Uint8Array;
  volume: number;
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

/** Give swept solid `idx` a surface meshed elsewhere, against the drawing as it stands now: a
 *  field's, or the exact B-rep's. */
export function supplyField(sk: Sketch, idx: number, s: FieldSurface): void {
  const nv = s.vertices.length / 3, nt = s.triangles.length / 3;
  const x = s.exact;
  const ok = withBuf(s.vertices.length, 8, (v) => {
    v.set(s.vertices);
    return withBuf(s.triangles.length, 4, (t) => {
      t.set(s.triangles);
      if (!x) return core().gcs_solid_supply_field(sk.handle, idx, v.ptr, nv, t.ptr, nt, s.provisional ? 1 : 0);
      return withBuf(x.of.length, 4, (o) => {
        o.set(x.of);
        return withBuf(x.smooth.length, 1, (m) => {
          m.set(x.smooth);
          return core().gcs_solid_supply_exact(sk.handle, idx, v.ptr, nv, t.ptr, nt, o.ptr, m.ptr, x.smooth.length, x.volume);
        });
      });
    });
  });
  if (ok < 0) throw new Error(lastError());
}

/** Where an exact build stands: what its next stage does, the stages run and in all, and the last
 *  thing a stage said. */
export interface ExactProgress {
  doing: string;
  done: number;
  total: number;
  said: string | null;
}

/** **A swept solid's exact surface, built a stage at a time** (`brep::export::Builder`): admitted,
 *  its blank, each sweep's sheet, its sector, the cut, the pattern and the mesh — seconds a stage
 *  for a gear, so a worker steps it and says between stages what it is doing. */
export class ExactBuilder {
  private constructor(private h: number) {}

  /** The build of swept solid `idx`, or why it has none. */
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

  /** The built surface. */
  surface(): FieldSurface {
    const handle = core().gcs_exact_builder_surface(this.h);
    if (!handle) throw new Error(lastError());
    const bytes = takeBytes(handle);
    const d = new Float64Array(bytes.buffer, bytes.byteOffset, bytes.byteLength / 8);
    const [nv, nt, nf, volume] = [d[0], d[1], d[2], d[3]];
    let at = 4;
    const take = (n: number): Float64Array => { const s = d.subarray(at, at + n); at += n; return s; };
    const vertices = Float64Array.from(take(3 * nv));
    const triangles = Uint32Array.from(take(3 * nt));
    const of = Uint32Array.from(take(nt));
    const smooth = Uint8Array.from(take(nf));
    return { vertices, triangles, provisional: false, exact: { of, smooth, volume } };
  }

  dispose(): void {
    if (this.h) core().gcs_exact_builder_free(this.h);
    this.h = 0;
  }
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

  /** Mesh swept solid `idx`, `fineness` times finer than the preview's; the core says which
   *  finenesses it takes, and refuses another with the reason. */
  static create(sk: Sketch, idx: number, fineness = 1): FieldMesher {
    const h = core().gcs_field_mesher_new(sk.handle, idx, fineness);
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
