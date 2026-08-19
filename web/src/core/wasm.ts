/* Loading the WebAssembly core and moving numbers across the boundary.
 *
 * The C library owns every number in the drag loop; this module is the only place that
 * knows about pointers.  Heap views are re-read on every access because the module grows
 * its memory on demand, which detaches the old typed arrays.
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

export function malloc(bytes: number): number {
  return core()._malloc(Math.max(bytes, 8));
}

export function free(ptr: number): void {
  if (ptr) core()._free(ptr);
}

/** A block of doubles in the core's heap, with a matching JS view on demand. */
export class Buf {
  readonly ptr: number;
  constructor(readonly len: number) {
    this.ptr = malloc(len * 8);
  }
  get view(): Float64Array {
    return core().HEAPF64.subarray(this.ptr >> 3, (this.ptr >> 3) + this.len);
  }
  set(src: ArrayLike<number>): this {
    this.view.set(src as Float64Array);
    return this;
  }
  copy(): Float64Array {
    return new Float64Array(this.view);
  }
  release(): void {
    free(this.ptr);
  }
}

export class IBuf {
  readonly ptr: number;
  constructor(readonly len: number) {
    this.ptr = malloc(len * 4);
  }
  get view(): Int32Array {
    return core().HEAP32.subarray(this.ptr >> 2, (this.ptr >> 2) + this.len);
  }
  set(src: ArrayLike<number>): this {
    this.view.set(src as Int32Array);
    return this;
  }
  copy(): Int32Array {
    return new Int32Array(this.view);
  }
  release(): void {
    free(this.ptr);
  }
}

/** Copy `a` into a fresh heap block; the caller releases it. */
export function toHeapF64(a: ArrayLike<number>): Buf {
  return new Buf(a.length).set(a);
}

export function toHeapI32(a: ArrayLike<number>): IBuf {
  return new IBuf(a.length).set(a);
}

/** Run `fn` with temporary heap blocks, releasing them afterwards even on error. */
export function withBufs<T>(bufs: { release(): void }[], fn: () => T): T {
  try {
    return fn();
  } finally {
    for (const b of bufs) b.release();
  }
}

export function readF64(ptr: number, len: number): Float64Array {
  return new Float64Array(core().HEAPF64.subarray(ptr >> 3, (ptr >> 3) + len));
}

export function readI32(ptr: number, len: number): Int32Array {
  return new Int32Array(core().HEAP32.subarray(ptr >> 2, (ptr >> 2) + len));
}

export function readU8(ptr: number, len: number): Uint8Array {
  return new Uint8Array(core().HEAPU8.subarray(ptr, ptr + len));
}
