/* Loading the WebAssembly core and moving numbers across the boundary.
 *
 * The C library owns every number in the drag loop; `core/` is the only layer that knows
 * about pointers (this module, `system.ts` and `linalg.ts`), and nothing under `app/` does.
 * Heap views are re-read on every access because the module grows its memory on demand,
 * which detaches the old typed arrays.
 */
import factory, { GcsModule } from '../wasm/gcs.js';

let mod: GcsModule | null = null;

/** Load the core once; every later call returns the same instance. */
export async function initCore(opts?: Record<string, unknown>): Promise<GcsModule> {
  if (!mod) mod = await factory(opts);
  return mod;
}

/** The loaded core.  Throws if `initCore` has not resolved yet. */
export function core(): GcsModule {
  if (!mod) throw new Error('gcs core not initialised — await initCore() first');
  return mod;
}

/** A block of the core's heap with a matching JS view on demand.  `Buf` holds doubles and
 *  `IBuf` 32-bit ints; both exist so callers never compute a byte offset themselves. */
abstract class HeapArray<T extends Float64Array | Int32Array> {
  readonly ptr: number;

  constructor(readonly len: number, bytes: number) {
    this.ptr = core()._malloc(Math.max(len * bytes, 8));
  }

  abstract get view(): T;

  set(src: ArrayLike<number>): this {
    this.view.set(src as T);
    return this;
  }

  copy(): T {
    return this.view.slice() as T;
  }

  release(): void {
    if (this.ptr) core()._free(this.ptr);
  }
}

export class Buf extends HeapArray<Float64Array> {
  constructor(len: number) {
    super(len, 8);
  }
  override get view(): Float64Array {
    return core().HEAPF64.subarray(this.ptr >> 3, (this.ptr >> 3) + this.len);
  }
}

export class IBuf extends HeapArray<Int32Array> {
  constructor(len: number) {
    super(len, 4);
  }
  override get view(): Int32Array {
    return core().HEAP32.subarray(this.ptr >> 2, (this.ptr >> 2) + this.len);
  }
}

export function readI32(ptr: number, len: number): Int32Array {
  return core().HEAP32.slice(ptr >> 2, (ptr >> 2) + len);
}

export function readU8(ptr: number, len: number): Uint8Array {
  return core().HEAPU8.slice(ptr, ptr + len);
}
