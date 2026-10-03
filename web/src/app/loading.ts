/* **The page says when it has nothing worth touching yet.**
 *
 * Opening a heavy example is seconds in four stages: the core starting, the files arriving, the
 * elaboration and solve (on this thread, so the page is frozen through it), and — for a drawing
 * with swept solids — the worker's first surfaces, before which the workspace is empty.  Through
 * all of them a modal covers the page with what is going on and a bar: a fraction where the
 * worker says one, a sweep where nothing does.  The sweep is a CSS transform, so it keeps moving
 * while a solve holds the thread.
 *
 * The modal waits for a *first* surface of every swept object, not a finished one: the rough
 * shape is something to look at, orbit and edit beside, and an exact build takes minutes. */
import { objectName, type Refining } from './field-preview.js';

const el = document.getElementById('loading') as HTMLElement;
const label = document.getElementById('loading-label') as HTMLElement;
const app = document.getElementById('app') as HTMLElement;

/** Whether an opening is waiting on its swept objects' first surfaces, and how many are left. */
let surfaces = false;
let left = 0;

/** Show the modal saying `what`, with `fraction` done, or a sweep where there is no fraction. */
export function loading(what: string, fraction?: number): void {
  label.textContent = what;
  el.classList.toggle('sweeping', fraction === undefined);
  if (fraction === undefined) el.removeAttribute('aria-valuenow');
  else {
    const done = Math.max(0, Math.min(1, fraction));
    el.style.setProperty('--done', String(done));
    el.setAttribute('aria-valuenow', String(Math.round(done * 100)));
  }
  el.hidden = false;
  app.inert = true;
}

/** The page is worth touching: put the modal away. */
export function loaded(): void {
  surfaces = false;
  left = 0;
  el.hidden = true;
  app.inert = false;
  el.style.removeProperty('--done');
}

/** Once the modal as it stands has been painted, so a stage about to hold the thread is said. */
export function painted(): Promise<void> {
  return new Promise((then) => requestAnimationFrame(() => requestAnimationFrame(() => then())));
}

/** Wait for the swept objects of the drawing about to be opened, as `meshing` hears of them. */
export function awaitSurfaces(): void {
  surfaces = true;
  left = 0;
}

/** Where the swept objects stand (`FieldPreview`'s `progressed`): while an opening waits on them,
 *  the modal follows the slowest still without a surface, and goes once none is. */
export function meshing(list: Refining[]): void {
  if (!surfaces) return;
  const waiting = list.filter((r) => !(r.triangles > 0 || r.exact?.built || r.error || r.progress?.failed));
  left = waiting.length;
  if (!left) { loaded(); return; }
  // the pass in hand's progress where every one waited on has said it; till then (the worker
  // elaborating its own copy of the drawing) a sweep
  const within = waiting.map((r) => r.progress?.within).filter((f): f is number => f !== undefined);
  const doing = waiting.find((r) => r.progress)?.progress?.doing ?? 'preparing';
  loading(`meshing ${waiting.map((r) => objectName(r.name)).join(', ')} — ${doing}`,
    within.length === waiting.length ? Math.min(...within) : undefined);
}

/** The drawing is open: the modal goes, unless swept objects it waits on have no surface yet. */
export function opened(): void {
  if (!surfaces || !left) loaded();
}
