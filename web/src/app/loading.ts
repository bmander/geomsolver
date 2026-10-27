/* **The page says when it has nothing worth touching yet** — and only then.
 *
 * Opening a document is the core starting, the files arriving and the reading — elaboration,
 * solve and diagnosis, on this thread, so the page is frozen through it.  Through those a modal
 * covers the page with what is going on and a bar: a fraction where the work says one, a sweep
 * where nothing does.  The sweep is a CSS transform, so it keeps moving while the thread is held.
 *
 * **Only the reading is modal** (#672): the modal goes once the drawing can be painted, before
 * any object is built.  The objects are built by the pool off the page's thread and arrive one by
 * one, said by the build bar along the workspace's bottom (`build-bar.ts`); the claims about
 * solids are judged in a worker and said in the footer.  Nothing waits for either. */

const el = document.getElementById('loading') as HTMLElement;
const label = document.getElementById('loading-label') as HTMLElement;
const app = document.getElementById('app') as HTMLElement;

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
  if (!el.hidden) performance.mark('gcs:loaded');
  el.hidden = true;
  app.inert = false;
  el.style.removeProperty('--done');
}

/** Once the modal as it stands has been painted, so a stage about to hold the thread is said. */
export function painted(): Promise<void> {
  return new Promise((then) => requestAnimationFrame(() => requestAnimationFrame(() => then())));
}
