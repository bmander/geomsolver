/* Parameters that run (#398 L): the declared parameters of a drawing, its march along one, the
 * drawing stood at an instant and sent home, and a point dragged along its track.  Thin proxies
 * over `gcs_parameter*` and `gcs_scrub_*`: every reading, every reach and every refusal is the
 * core's, in its words. */
import type { Point, Sketch } from './model.js';
import { core, lastError, takeJson, takeStr, withStr } from './wasm.js';

/** A parameter the drawing declares (`param t: Angle over (0deg, 360deg)`): its ends in the units
 *  a person reads (degrees for an angle) and as read. */
export interface Parameter {
  name: string;
  dim: string;
  from: number;
  to: number;
  fromText: string;
  toText: string;
}

/** Where the march stopped short of the interval's end, and what is there. */
export interface Singular {
  at: number;
  kind: 'limit' | 'branch';
  text: string;
}

/** How the drawing runs over a parameter: from where and how far it is followed, or why it is
 *  not. */
export type March =
  | { name: string; followed: true; from: number; reached: number; reachedText: string;
      singular: Singular | null; text: string }
  | { name: string; followed: false; why: string; text: string };

export function parameters(sk: Sketch): Parameter[] {
  return takeJson<Parameter[]>(core().gcs_parameters_json(sk.handle)) ?? [];
}

/** Follow the drawing along `name` (remembered by the core against the drawing at home). */
export function march(sk: Sketch, name: string): March {
  return withStr(name, (p, n) => takeJson<March>(core().gcs_parameter_json(sk.handle, p, n)));
}

/** Stand the drawing at `t` of `name`: null, or why it was left at home. */
export function stand(sk: Sketch, name: string, t: number): string | null {
  return withStr(name, (p, n) => takeStr(core().gcs_parameter_stand(sk.handle, p, n, t))) || null;
}

/** Send the drawing home, where it was drawn. */
export function home(sk: Sketch): void {
  core().gcs_parameter_home(sk.handle);
}

/** An instant of `name` as written (`90deg`), or why it is none. */
export function instant(sk: Sketch, name: string,
                        text: string): { at: number } | { error: string } {
  return withStr(name, (np, nn) => withStr(text, (tp, tn) =>
    takeJson<{ at: number } | { error: string }>(core().gcs_parameter_instant(sk.handle, np, nn,
                                                                               tp, tn))));
}

/** A value of `name` as a reader reads it: `129.838deg`. */
export function reading(sk: Sketch, name: string, v: number): string {
  return withStr(name, (p, n) => takeStr(core().gcs_parameter_reading(sk.handle, p, n, v)));
}

/** **A point dragged along the march** (`follow::Scrub`): each move finds the instant where the
 *  point comes nearest the pointer and stands the drawing there.  The pointer is read as a drag
 *  reads it — in the point's own plane, or with `eye` on the eye's picture plane. */
export class Scrub {
  /** Scrubs made and not yet ended — a handle that outlives its gesture shows up here. */
  static live = 0;
  private handle: number;

  private constructor(readonly sketch: Sketch, readonly name: string, handle: number) {
    this.handle = handle;
    Scrub.live += 1;
  }

  /** Take hold of `point`, or why it is not dragged along `name`: the drawing is not followed,
   *  or the point does not move as it runs. */
  static take(sketch: Sketch, name: string, point: Point,
              eye: { az: number; el: number } | null = null): Scrub | { refused: string } {
    const h = withStr(name, (p, n) => core().gcs_scrub_new(
      sketch.handle, p, n, point.index, eye ? 1 : 0, eye?.az ?? 0, eye?.el ?? 0));
    return h ? new Scrub(sketch, name, h) : { refused: lastError() };
  }

  /** The pointer at (`x`, `y`): the instant the drawing now stands at, or null and why. */
  move(x: number, y: number): { at: number } | { refused: string } {
    const t = core().gcs_scrub_move(this.handle, this.sketch.handle, x, y);
    return Number.isNaN(t) ? { refused: lastError() } : { at: t };
  }

  end(): void {
    if (!this.handle) return;
    core().gcs_scrub_free(this.handle);
    this.handle = 0;
    Scrub.live -= 1;
  }
}
