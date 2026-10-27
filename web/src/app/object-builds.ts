/* **Every object is built off the page's thread, exactly, and drawn as it arrives** (#671).
 *
 * An object is its exact B-rep, built as the STEP export builds it (`brep::export::Builder`), or it
 * is unbuilt with the kernel's reason: the page never meshes a field, and its own thread builds
 * no object.  Its sketch is told so (`supplyObjects`) and names the builds (`objectJobs`): one a
 * binding — the placements of one build share it, and the core places each — each keyed by a
 * digest of what it reads, so equal keys are an equal surface on any core.  A pool of workers
 * (`build-work.ts`, at most `MAX_WORKERS`, one core short of the machine's) takes the jobs, the
 * swept ones first, each worker one at a time to the end; what one hands back is supplied to the
 * page's core (`supplyObject`), and picking, sections, bounds and the outline read it.
 *
 * **A build is asked once.**  A job whose key the page holds a build for — this drawing's, an
 * earlier edit's, or the store's from an earlier opening (`surface-store.ts`, #673) — is supplied
 * at once and asked of no worker; a new drawing ends the builds it no longer wants at their next
 * stage, and the ones it still wants carry on.  While its new build is made, an object an edit
 * changed is drawn as its last build was, stale (#672). */
import { objectJobs, refuseObject, supplyObject, supplyObjects, displayTriangles,
  type ObjectJob } from '../core/objects.js';
import type { Sketch } from '../core/model.js';
import { related } from '../core/modules.js';
import type { Document } from '../core/program.js';
import type { BuildAsk, BuildReply, Drawing } from './build-work.js';
import type { SurfaceStore } from './surface-store.js';

/** The most workers building at once: each holds a core and a document. */
export const MAX_WORKERS = 4;

/** How many bytes of builds no longer asked for the page keeps, for an undo. */
const EARLIER_BYTES = 64 * 1024 * 1024;

/** How many workers this machine is given: one core short of its count, at most `MAX_WORKERS`. */
export function poolSize(): number {
  const cores = typeof navigator !== 'undefined' ? navigator.hardwareConcurrency || 2 : 2;
  return Math.max(1, Math.min(cores - 1, MAX_WORKERS));
}

/** A pool worker, as this file uses it: a test hands in one of its own. */
export interface PoolWorker {
  postMessage(ask: BuildAsk): void;
  onmessage: ((ev: MessageEvent<BuildReply>) => void) | null;
  onerror: ((ev: ErrorEvent) => void) | null;
  onmessageerror: ((ev: MessageEvent) => void) | null;
  terminate?(): void;
}

const spawnWorker = (): PoolWorker =>
  new Worker(new URL('./mesh-worker.bundle.js', import.meta.url), { type: 'module' }) as unknown as PoolWorker;

/** **One job as the page tells it** (the build bar, #672): what it builds and stands for, and
 *  where it stands — waiting for a worker, building (its next stage in the core's words, the stages
 *  run and in all, the last thing a stage said, since when), built (and in what time, or taken from
 *  the store), or refused in the export's words. */
export interface BuildRow {
  key: string;
  name: string;
  solid: number;
  objects: number[];
  swept: boolean;
  state: 'waiting' | 'building' | 'built' | 'refused';
  doing: string;
  done: number;
  total: number;
  said?: string;
  /** When its build began (`performance.now()`). */
  since?: number;
  /** How long it took, once built or refused (milliseconds). */
  ms?: number;
  why?: string;
  /** Built at an earlier opening, and taken from the store. */
  stored?: boolean;
  triangles?: number;
}

/** A worker and the build it has in hand. */
interface Slot {
  w: PoolWorker;
  busy: { id: number; key: string } | null;
}

/** Once the frame now being drawn has been painted: two animation frames, since the first runs
 *  before that paint; without animation frames (a test's page) or with the page hidden, at once. */
function afterPaint(then: () => void): void {
  if (typeof requestAnimationFrame === 'undefined' || typeof document === 'undefined' || document.hidden) {
    setTimeout(then, 0);
  }
  else requestAnimationFrame(() => requestAnimationFrame(then));
}

export class ObjectBuilds {
  private slots: Slot[] = [];
  private sketch: Sketch | null = null;
  private drawing: Drawing | null = null;
  /** This drawing's jobs, by key, in the order asked. */
  private jobs = new Map<string, ObjectJob>();
  private rowsBy = new Map<string, BuildRow>();
  /** The keys waiting for a worker, first first. */
  private queue: string[] = [];
  /** Builds and refusals in hand, by key: this drawing's. */
  private built = new Map<string, Uint8Array>();
  private refused = new Map<string, string>();
  /** The last build of each object by name, and its key: offered to the next drawing, which holds
   *  it current where what the object reads is what it read, else draws it stale while the new one
   *  is made. */
  private lastByName = new Map<string, { key: string; bytes: Uint8Array }>();
  private ids = 0;
  /** Which drawing is the latest (`start`): a store's late answer is supplied only to its own. */
  private generation = 0;
  private redrawing = false;
  /** How many workers were made, in all: a test counts them. */
  spawned = 0;

  /** `arrived` redraws, once a frame at most; `progressed` hears every change of a row; `spawn`
   *  makes a pool worker — with none (a test's view, node) the page builds its objects itself, as
   *  the terminal does — and `store` remembers builds across openings. */
  constructor(private readonly arrived: (error?: string) => void,
              private readonly progressed: (rows: BuildRow[]) => void = () => {},
              private readonly spawn: (() => PoolWorker) | null = typeof Worker === 'undefined' ? null : spawnWorker,
              private readonly store: SurfaceStore | null = null,
              private readonly size = poolSize()) {}

  /** Whether the page's objects are built here: with a pool, and so never on its thread. */
  get pooled(): boolean { return !!this.spawn; }

  /** Every job of the drawing in hand, as the page tells it. */
  get rows(): BuildRow[] { return [...this.rowsBy.values()]; }

  /** **Build this document's objects**: its sketch told to build none itself, each job whose build
   *  is in hand (or the store's) supplied at once, the rest asked of the pool. */
  start(doc: Document): void {
    if (!this.spawn) return;
    const sk = doc.sketch;
    supplyObjects(sk);
    this.sketch = sk;
    let jobs = objectJobs(sk);
    this.generation += 1;
    // the outgoing drawing's builds by object, for what this one changed
    for (const j of this.jobs.values()) {
      const b = this.built.get(j.key);
      if (b) this.lastByName.set(j.name, { key: j.key, bytes: b });
    }
    // each object's last build offered to this drawing, where it holds none by key: the core
    // holds one current that a solve settling the drawing again left within its tolerance — its
    // job then keyed by the build — and draws any other stale while the new one is made
    let offered = false;
    for (const j of jobs) {
      const last = this.lastByName.get(j.name);
      if (!last || this.built.has(j.key) || this.refused.has(j.key)) continue;
      offered = this.give(j, last.bytes, false) || offered;
    }
    if (offered) {
      jobs = objectJobs(sk);
      for (const j of jobs) {
        const last = this.lastByName.get(j.name);
        if (last?.key === j.key && !this.built.has(j.key)) this.built.set(j.key, last.bytes);
      }
    }
    this.drawing = { text: doc.text, x: sk.getX(),
      modules: related(doc.text).filter((f) => f.provided).map((f): [string, string] => [f.name, f.text]) };
    this.jobs = new Map(jobs.map((j) => [j.key, j]));
    const names = new Set(jobs.map((j) => j.name));
    // an earlier drawing's builds are kept, the latest first, up to `EARLIER_BYTES`: an undo
    // asks for them again
    let earlier = 0;
    for (const [k, b] of [...this.built].reverse()) {
      if (this.jobs.has(k)) continue;
      earlier += b.byteLength;
      if (earlier > EARLIER_BYTES) this.built.delete(k);
    }
    for (const k of [...this.refused.keys()]) if (!this.jobs.has(k)) this.refused.delete(k);
    for (const n of [...this.lastByName.keys()]) if (!names.has(n)) this.lastByName.delete(n);
    // a build this drawing no longer wants ends at its next stage; one it wants carries on
    for (const s of this.slots) {
      if (s.busy && !this.jobs.has(s.busy.key)) {
        s.w.postMessage({ kind: 'drop', id: s.busy.id });
        s.busy = null;
      }
    }
    const rows = new Map<string, BuildRow>();
    for (const j of jobs) {
      const old = this.rowsBy.get(j.key);
      rows.set(j.key, old ? { ...old, name: j.name, solid: j.solid, objects: j.objects }
        : { key: j.key, name: j.name, solid: j.solid, objects: j.objects, swept: j.swept, state: 'waiting',
            doing: 'waiting', done: 0, total: 1 });
    }
    this.rowsBy = rows;
    let supplied = false;
    const asked: string[] = [];
    for (const j of jobs) {
      const b = this.built.get(j.key);
      const why = this.refused.get(j.key);
      const row = rows.get(j.key)!;
      if (b) {
        supplied = this.give(j, b, true) || supplied;
        if (row.state !== 'built') Object.assign(row, { state: 'built', doing: 'built', triangles: displayTriangles(b) });
      }
      else if (why !== undefined) supplied = this.refuse(j, why) || supplied;
      else if (!this.running(j.key)) asked.push(j.key);
    }
    supplied ||= offered;
    this.queue = asked;
    this.told();
    if (supplied) this.redraw();
    if (!this.store || !asked.length) { this.dispatch(); return; }
    // the store asked first, all at once; what it holds is supplied and asked of no worker
    const generation = this.generation;
    this.store.get(asked).then((hits) => {
      if (generation !== this.generation) return;
      let any = false;
      for (const [key, bytes] of hits) {
        if (!this.queue.includes(key)) continue;
        this.queue = this.queue.filter((k) => k !== key);
        any = this.arrive(key, bytes, { stored: true }) || any;
      }
      if (any) this.redraw();
      this.dispatch();
    }, () => { if (generation === this.generation) this.dispatch(); });
  }

  /** Whether a worker has `key` in hand. */
  private running(key: string): boolean {
    return this.slots.some((s) => s.busy?.key === key);
  }

  /** Each idle worker given the next job waiting, workers made as they are wanted. */
  private dispatch(): void {
    while (this.queue.length && this.drawing) {
      let slot = this.slots.find((s) => !s.busy);
      if (!slot && this.slots.length < this.size) slot = this.make();
      if (!slot) break;
      const key = this.queue.shift()!;
      const job = this.jobs.get(key);
      if (!job) continue;
      const id = ++this.ids;
      slot.busy = { id, key };
      const r = this.rowsBy.get(key);
      if (r) Object.assign(r, { state: 'building', doing: 'starting', done: 0, total: 1, since: performance.now() });
      slot.w.postMessage({ kind: 'build', id, drawing: this.drawing, job });
    }
    this.told();
  }

  private make(): Slot {
    const slot: Slot = { w: this.spawn!(), busy: null };
    this.spawned += 1;
    slot.w.onmessage = (ev) => this.receive(slot, ev.data);
    // a worker that dies says nothing of itself: the build it had is refused, and the worker goes
    const gone = (why: string): void => {
      const busy = slot.busy;
      this.slots = this.slots.filter((s) => s !== slot);
      slot.w.terminate?.();
      if (busy) this.fail(busy.key, why);
      this.dispatch();
    };
    slot.w.onerror = (ev) => gone(ev.message || 'a build worker stopped');
    slot.w.onmessageerror = () => gone('a build worker sent what could not be read');
    this.slots.push(slot);
    return slot;
  }

  /** A worker's word on its build: where it stands, its buffer, or its refusal. */
  private receive(slot: Slot, r: BuildReply): void {
    if (!slot.busy || slot.busy.id !== r.id) return;   // a dropped build's
    const row = this.rowsBy.get(r.key);
    if (row && r.progress) {
      Object.assign(row, { doing: r.progress.doing, done: r.progress.done, total: r.progress.total,
        said: r.progress.said ?? row.said });
    }
    if (r.display) {
      slot.busy = null;
      performance.mark('gcs:built', { detail: { name: row?.name, ms: r.ms } });
      void this.store?.put(r.key, r.display).catch(() => {});
      if (this.arrive(r.key, r.display, { ms: r.ms })) this.redraw();
      this.dispatch();
      return;
    }
    if (r.refused !== undefined) {
      slot.busy = null;
      this.fail(r.key, r.refused, r.ms);
      this.dispatch();
      return;
    }
    this.told();
  }

  /** A build of `key` in hand: kept, and supplied where this drawing asks for it. */
  private arrive(key: string, bytes: Uint8Array, how: { ms?: number; stored?: boolean }): boolean {
    const job = this.jobs.get(key);
    if (!job) return false;
    this.built.delete(key);
    this.built.set(key, bytes);
    const row = this.rowsBy.get(key);
    if (row) {
      Object.assign(row, { state: 'built', doing: 'built', ms: how.ms, stored: how.stored,
        triangles: displayTriangles(bytes) });
      row.done = row.total = Math.max(row.total, 1);
    }
    const given = this.give(job, bytes, true);
    this.told();
    return given;
  }

  /** The build of `key` refused (or its worker lost), in the refusal's words. */
  private fail(key: string, why: string, ms?: number): void {
    const job = this.jobs.get(key);
    if (!job) return;
    this.refused.set(key, why);
    const row = this.rowsBy.get(key);
    if (row) Object.assign(row, { state: 'refused', doing: 'refused', why, ms });
    performance.mark('gcs:unbuilt', { detail: { name: job.name, error: why } });
    if (this.refuse(job, why)) this.redraw();
    this.told();
  }

  /** Supply a build to the page's core, which holds it current or stale by what it was built
   *  from: one built for `job`'s key (`fresh`) said on the User Timing line, and refused as the
   *  build's refusal where the core will not take it. */
  private give(job: ObjectJob, bytes: Uint8Array, fresh: boolean): boolean {
    if (!this.sketch) return false;
    try {
      supplyObject(this.sketch, job.solid, bytes);
      if (fresh) {
        performance.mark('gcs:surface', { detail: { name: job.name, exact: true, provisional: false,
          triangles: displayTriangles(bytes) } });
      }
      return true;
    } catch (e) {
      if (fresh) this.fail(job.key, e instanceof Error ? e.message : String(e));
      return false;
    }
  }

  private refuse(job: ObjectJob, why: string): boolean {
    if (!this.sketch) return false;
    try { refuseObject(this.sketch, job.solid, why); return true; }
    catch { return false; }
  }

  private told(): void { this.progressed(this.rows); }

  /** The page redrawn once the frame in hand is painted: builds arriving together, one redraw. */
  private redraw(): void {
    if (this.redrawing) return;
    this.redrawing = true;
    // (a pool let go redraws nothing: its drawing may be gone)
    afterPaint(() => { this.redrawing = false; if (this.sketch) this.arrived(); });
  }

  /** Whether builds are kept between openings (`surface-store.ts`). */
  get remembers(): boolean { return !!this.store; }

  /** Forget every build kept between openings. */
  async forget(): Promise<void> { await this.store?.clear(); }

  /** Let every worker go, and hear nothing more of them: the page is closing, another pool takes
   *  over, or a test is done — a build arriving late is never supplied to a drawing let go. */
  dispose(): void {
    for (const s of this.slots) {
      s.busy = null;
      s.w.onmessage = null;
      s.w.terminate?.();
    }
    this.slots = [];
    this.queue = [];
    this.sketch = null;
    this.drawing = null;
    this.jobs.clear();
    this.generation += 1;
  }
}
