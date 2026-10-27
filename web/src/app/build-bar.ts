/* **A build bar along the bottom of the workspace, not a modal** (#672): the page is usable while
 * its objects arrive.
 *
 * The reading holds the page's thread, and only it is modal (`loading.ts`); the builds are the
 * pool's (`object-builds.ts`) and take nothing from the page, so while any is running a thin bar
 * along the workspace's bottom edge says how far they have got — the fraction of the jobs done,
 * each weighted by its builder's stages, the job in hand and what it is doing in the core's words,
 * and the time so far — and goes when nothing is left.  A refusal leaves a mark on it until it is
 * opened.  Clicked, it opens the list: a row a job, its object, its stage, its time, and whether
 * it is done, refused (in the export's words) or waiting; a row clicked selects its object, as the
 * outline's does.  What it says is `barOf`'s, read from the rows alone, so a test reads it too. */
import type { BuildRow } from './object-builds.js';

/** What the bar says of the jobs: whether it shows, how far they have got (`fraction`, each job
 *  weighted by its stages), how many are left and refused of how many, its words, and whether it
 *  is marked by a refusal not yet seen. */
export interface BarState {
  shown: boolean;
  fraction: number;
  left: number;
  refused: number;
  total: number;
  text: string;
  marked: boolean;
}

/** A job's stages in all, before its builder has said: a swept body's are about seven, a static
 *  body's two. */
const GUESS = { swept: 7, static: 2 };

/** An object's name as the bar shows it: its last word, a body's own name rather than `body`. */
export function objectName(name: string): string {
  return name.split('.').filter((w) => w !== 'body').pop() ?? name;
}

/** A clock's words for `ms`: minutes and seconds. */
export function clock(ms: number): string {
  const s = Math.max(0, Math.floor(ms / 1000));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}

/** **What the bar says of `rows`**: building while any job is waiting or building — `building 12
 *  of 35 · wheel_part — cutting the sector by its sheets`, the job named the one building longest —
 *  then, while a refusal is `unseen`, how many were not built; else nothing.  It is marked while a
 *  refusal is unseen. */
export function barOf(rows: BuildRow[], unseen = true): BarState {
  const total = rows.length;
  let weight = 0, done = 0;
  for (const r of rows) {
    const w = r.state === 'waiting' ? (r.swept ? GUESS.swept : GUESS.static) : Math.max(r.total, 1);
    weight += w;
    done += r.state === 'built' || r.state === 'refused' ? w : r.state === 'building' ? Math.min(r.done, w) : 0;
  }
  const left = rows.filter((r) => r.state === 'waiting' || r.state === 'building').length;
  const refused = rows.filter((r) => r.state === 'refused').length;
  const fraction = weight ? done / weight : 1;
  if (left) {
    const building = rows.filter((r) => r.state === 'building')
      .sort((a, b) => (a.since ?? Infinity) - (b.since ?? Infinity))[0];
    const k = Math.min(total - left + 1, total);
    const what = building ? ` · ${objectName(building.name)} — ${building.doing}` : '';
    return { shown: true, fraction, left, refused, total, text: `building ${k} of ${total}${what}`, marked: refused > 0 && unseen };
  }
  if (refused && unseen) {
    return { shown: true, fraction: 1, left, refused, total, marked: true,
             text: `${refused} of ${total} not built — open to see why` };
  }
  return { shown: false, fraction: 1, left, refused, total, text: '', marked: false };
}

/** One row of the list's words: its object, its stage or its refusal, its time. */
export function rowWords(r: BuildRow, now: number): { name: string; state: string; detail: string; time: string } {
  const time = r.ms !== undefined ? clock(r.ms) : r.since !== undefined ? clock(now - r.since) : '';
  switch (r.state) {
    case 'built': return { name: r.name, state: r.stored ? 'kept' : 'done', detail: r.stored ? 'built at an earlier opening' : r.said ?? '', time };
    case 'refused': return { name: r.name, state: 'refused', detail: r.why ?? '', time };
    case 'building': return { name: r.name, state: `${r.done} of ${r.total}`, detail: r.doing, time };
    default: return { name: r.name, state: 'waiting', detail: '', time: '' };
  }
}

/** **The bar as the page draws it**, in `root` (the workspace's bottom edge): `select` is what a
 *  row of its list does when clicked. */
export class BuildBar {
  private readonly head: HTMLButtonElement;
  private readonly fill: HTMLElement;
  private readonly text: HTMLElement;
  private readonly time: HTMLElement;
  private readonly list: HTMLOListElement;
  private rows: BuildRow[] = [];
  /** When the bar last came up (`performance.now()`), for its clock. */
  private since = 0;
  private ticking = 0;
  /** The refusals the list has shown, by key: the mark they left goes once seen. */
  private seen = new Set<string>();

  constructor(private readonly root: HTMLElement, private readonly select: (row: BuildRow) => void) {
    root.hidden = true;
    this.head = document.createElement('button');
    this.head.id = 'build-bar-head';
    this.head.setAttribute('aria-expanded', 'false');
    this.head.title = 'The objects being built, each exactly as the STEP export builds it — click for the list';
    const track = document.createElement('span');
    track.className = 'build-track';
    this.fill = document.createElement('span');
    this.fill.className = 'build-fill';
    track.append(this.fill);
    this.text = document.createElement('span');
    this.text.className = 'build-text';
    this.time = document.createElement('span');
    this.time.className = 'build-time';
    this.head.append(track, this.text, this.time);
    this.list = document.createElement('ol');
    this.list.className = 'build-list';
    this.list.hidden = true;
    root.append(this.list, this.head);
    this.head.addEventListener('click', () => this.toggle());
  }

  /** The jobs as they now stand. */
  update(rows: BuildRow[]): void {
    this.rows = rows;
    this.paint();
  }

  private toggle(): void {
    this.list.hidden = !this.list.hidden;
    this.head.setAttribute('aria-expanded', String(!this.list.hidden));
    if (!this.list.hidden) for (const r of this.rows) if (r.state === 'refused') this.seen.add(r.key);
    this.paint();
  }

  private paint(): void {
    const keys = new Set(this.rows.map((r) => r.key));
    for (const k of [...this.seen]) if (!keys.has(k)) this.seen.delete(k);
    const unseen = this.rows.some((r) => r.state === 'refused' && !this.seen.has(r.key));
    const s = barOf(this.rows, unseen);
    const shown = s.shown || !this.list.hidden && this.rows.length > 0;
    if (shown && this.root.hidden) this.since = performance.now();
    this.root.hidden = !shown;
    this.root.classList.toggle('marked', s.marked);
    this.fill.style.transform = `scaleX(${s.fraction})`;
    this.text.textContent = s.text || `${s.total} built${s.refused ? `, ${s.refused} refused` : ''}`;
    this.head.setAttribute('aria-valuenow', String(Math.round(s.fraction * 100)));
    const now = performance.now();
    this.time.textContent = s.left ? clock(now - this.since) : '';
    if (!this.list.hidden) this.fillList(now);
    // a stage of a gear runs for seconds with nothing said: the clock ticks while any job is left
    if (s.left && !this.ticking) this.ticking = window.setInterval(() => this.paint(), 500);
    if (!s.left && this.ticking) { clearInterval(this.ticking); this.ticking = 0; }
  }

  private fillList(now: number): void {
    this.list.replaceChildren(...this.rows.map((r) => {
      const w = rowWords(r, now);
      const li = document.createElement('li');
      li.className = `build-row ${r.state}`;
      li.title = r.objects.length > 1 ? `${r.objects.length} objects, one build` : '';
      const cells = [['build-name', objectName(w.name)], ['build-state', w.state], ['build-detail', w.detail],
                     ['build-clock', w.time]] as const;
      for (const [cls, text] of cells) {
        const span = document.createElement('span');
        span.className = cls;
        span.textContent = text;
        if (cls === 'build-detail') span.title = text;
        li.append(span);
      }
      li.addEventListener('click', () => this.select(r));
      return li;
    }));
  }
}
