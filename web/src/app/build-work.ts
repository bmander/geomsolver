/* **A pool worker's half of building every object** (#671): one build at a time, a stage at a time.
 *
 * The page hands a worker one job (`objects.ts`'s `ObjectJob`) with the drawing it is of — its
 * text, the modules it reaches and the page's parameter values — and the worker steps the core's
 * builder (`ExactBuilder`) to the end, saying after each stage what the next does, and hands back
 * the build as one buffer, transferred, or the refusal in the export's words.  Between stages it
 * lets messages in, so a job the page no longer wants (`drop`) ends at its next stage.
 *
 * **A worker keeps its document between jobs**: it holds the last drawing it was handed, and a
 * drawing whose text is the same, or a numeric edit of it (`Document.retext`, as the page takes
 * one), is the same elaboration with the page's values written over it — only a structural edit,
 * or other modules, is read again.  Either way its own core must name the job as the page's did
 * (the same solid, name and key), or the build would be another object's. */
import { install } from '../core/modules.js';
import { ExactBuilder, objectJobs, type ExactProgress, type ObjectJob } from '../core/objects.js';
import { Document } from '../core/program.js';

/** The drawing a job is of, as the page holds it. */
export interface Drawing {
  text: string;
  /** The modules the page handed over and the text reaches, as `[use name, text]`: the library's
   *  own the worker's core has already. */
  modules: [string, string][];
  /** The page's parameter values, so the worker's drawing is the page's and not a fresh solve. */
  x: Float64Array;
}

/** What the page asks of a pool worker: build a job, or stop building one. */
export type BuildAsk =
  | { kind: 'build'; id: number; drawing: Drawing; job: ObjectJob }
  | { kind: 'drop'; id: number };

/** What a pool worker says of build `id` of `key`: where it stands, its buffer once built (with
 *  the milliseconds it took), or the refusal that stopped it. */
export interface BuildReply {
  id: number;
  key: string;
  progress?: ExactProgress;
  display?: Uint8Array;
  refused?: string;
  ms?: number;
}

export class BuildWork {
  /** The last drawing handed over, elaborated, the modules it was read with, the values written
   *  over it, and the jobs its core names at those (asked once a drawing, for every job of it). */
  private held: { modules: string; doc: Document; x: Float64Array; jobs?: ObjectJob[] } | null = null;
  /** How many elaborations the worker has read: a drawing reused reads none. */
  reads = 0;
  private dropped = new Set<number>();
  /** The builds in the order asked, one at a time. */
  private chain: Promise<void> = Promise.resolve();

  /** `post` hands a reply to the page (its buffers transferred), and `pause` lets messages in. */
  constructor(private readonly post: (reply: BuildReply, transfer?: Transferable[]) => void,
              private readonly pause: () => Promise<void>) {}

  receive(ask: BuildAsk): void {
    if (ask.kind === 'drop') { this.dropped.add(ask.id); return; }
    this.chain = this.chain.then(() => this.build(ask)).catch(() => {});
  }

  /** The drawing a job is of, as this worker's core reads it: the one held where the text is
   *  its, or a numeric edit of it, else read again (`fresh`); the page's values written over it. */
  private document(drawing: Drawing): { doc: Document; fresh: boolean } {
    const modules = JSON.stringify(drawing.modules);
    const h = this.held;
    if (h && h.modules === modules && h.doc.live) {
      const same = h.doc.text === drawing.text;
      if (same && sameValues(h.x, drawing.x)) return { doc: h.doc, fresh: false };
      if (same || h.doc.retext(drawing.text)) {
        // (`retext` takes the caller's word that the statements are the same: values that do not
        // fit the drawing held say they are not, and it is read again)
        try {
          h.doc.sketch.setX(drawing.x);
          Object.assign(h, { x: drawing.x, jobs: undefined });
          return { doc: h.doc, fresh: false };
        } catch { /* read again, below */ }
      }
    }
    return { doc: this.read(drawing, modules), fresh: true };
  }

  /** `drawing` read afresh, and held. */
  private read(drawing: Drawing, modules: string): Document {
    this.held?.doc.dispose();
    this.held = null;
    install(drawing.modules);
    const doc = Document.read(drawing.text);
    this.reads += 1;
    doc.sketch.setX(drawing.x);
    this.held = { modules, doc, x: drawing.x };
    return doc;
  }

  /** Why the drawing held does not build `job` as the page's does, or nothing. */
  private unpaired(job: ObjectJob): string | undefined {
    const h = this.held!;
    h.jobs ??= objectJobs(h.doc.sketch);
    const mine = h.jobs.find((j) => j.solid === job.solid);
    if (mine && mine.name === job.name && mine.key === job.key) return undefined;
    return `the worker's drawing does not build \`${job.name}\` as the page's does`;
  }

  private async build(ask: Extract<BuildAsk, { kind: 'build' }>): Promise<void> {
    const { id, job } = ask;
    const key = job.key;
    const refuse = (e: unknown, started: number): void =>
      this.post({ id, key, refused: e instanceof Error ? e.message : String(e), ms: performance.now() - started });
    await this.pause();
    if (this.dropped.delete(id)) return;
    const started = performance.now();
    let builder: ExactBuilder;
    try {
      let { doc, fresh } = this.document(ask.drawing);
      // a document kept across a numeric edit that names the job otherwise is read afresh, once:
      // what `retext` took is the text, and a number only the elaboration writes (a depth) is not
      if (!fresh && this.unpaired(job)) doc = this.read(ask.drawing, JSON.stringify(ask.drawing.modules));
      const why = this.unpaired(job);
      if (why) throw new Error(why);
      builder = ExactBuilder.create(doc.sketch, job.solid);
    } catch (e) {
      refuse(e, started);
      return;
    }
    try {
      this.post({ id, key, progress: builder.progress() });
      for (;;) {
        await this.pause();
        if (this.dropped.delete(id)) return;
        const built = builder.step();
        const progress = builder.progress();
        if (built) {
          const display = builder.display();
          this.post({ id, key, progress, display, ms: performance.now() - started }, [display.buffer]);
          return;
        }
        this.post({ id, key, progress });
      }
    } catch (e) {
      refuse(e, started);
    } finally {
      builder.dispose();
    }
  }
}

/** Whether two lists of parameter values are the same, to the bit. */
function sameValues(a: Float64Array, b: Float64Array): boolean {
  return a.length === b.length && a.every((v, k) => Object.is(v, b[k]));
}
