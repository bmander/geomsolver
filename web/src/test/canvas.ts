/* A canvas stub, so the app layer can be tested without a browser.
 *
 * `SketchView` needs three things from its canvas: a 2D context to draw into, a size, and pointer
 * events.  The context here swallows everything (nothing is being looked at), and `fire` delivers
 * a synthetic event to the handlers the view registered — which is how the pointer lifecycle
 * (capture, a second finger, a cancelled touch) gets exercised at all. */

/* eslint-disable @typescript-eslint/no-explicit-any */

const context: any = new Proxy(function () { /* every call is a no-op */ } as any, {
  get: (_t, k) => (k === 'measureText' ? () => ({ width: 8 }) : context),
  set: () => true,
  apply: () => context,
});

export interface FakeCanvas {
  fire(type: string, init: Record<string, unknown>): void;
  handlers: Map<string, ((e: any) => void)[]>;
}

/** A canvas the view can be constructed with, plus `fire` for the tests. */
export function fakeCanvas(width = 800, height = 600): HTMLCanvasElement & FakeCanvas {
  const handlers = new Map<string, ((e: any) => void)[]>();
  const captured = new Set<number>();
  const cv: any = {
    clientWidth: width,
    clientHeight: height,
    width,
    height,
    style: {},
    handlers,
    getContext: () => context,
    getBoundingClientRect: () => ({ left: 0, top: 0, width, height }),
    setPointerCapture: (id: number) => captured.add(id),
    releasePointerCapture: (id: number) => captured.delete(id),
    hasPointerCapture: (id: number) => captured.has(id),
    addEventListener: (type: string, fn: (e: any) => void) => {
      const list = handlers.get(type) ?? [];
      list.push(fn);
      handlers.set(type, list);
    },
    removeEventListener: (type: string, fn: (e: any) => void) => {
      handlers.set(type, (handlers.get(type) ?? []).filter((f) => f !== fn));
    },
    fire: (type: string, init: Record<string, unknown>) => {
      const e = { type, preventDefault: () => {}, stopPropagation: () => {}, ...init };
      for (const fn of handlers.get(type) ?? []) fn(e);
    },
  };
  return cv as HTMLCanvasElement & FakeCanvas;
}

/** A pointer event's fields, with the ones the view reads defaulted. */
export function pointer(x: number, y: number, init: Record<string, unknown> = {})
  : Record<string, unknown> {
  return { pointerId: 1, button: 0, buttons: 1, clientX: x, clientY: y, shiftKey: false, ...init };
}
