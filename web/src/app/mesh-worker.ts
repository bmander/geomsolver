/* The page's worker: each instance holds its own copy of the core and does one kind of work.
 *
 * A pool of them builds the page's objects (`build` jobs, `object-builds.ts`): each object's exact
 * B-rep a stage at a time (`build-work.ts`), saying after each stage what the next does — a stage
 * of a gear takes seconds, and the page shows the work going on — and handing the build back once
 * made; the worker keeps its document between jobs.
 *
 * Another judges the drawing's claims about solids (a `claims` job, `claim-judging.ts`): a pose
 * at a time (`ClaimJudge`), saying which claim of how many and which pose, and posting the
 * verdicts once every claim is judged.
 *
 * Another reads the load scenarios (a `scenario` job, `scenario-view.ts`): every scenario's
 * answers and the picture of the one chosen, a level at a time, coarsest first, each posted as it
 * is read — its arrays' buffers handed over rather than copied — and a newer job ending it
 * between levels. */
import { ClaimJudge, solidClaims } from '../core/claims.js';
import { install } from '../core/modules.js';
import { Document, type ScenarioLevels } from '../core/program.js';
import { initCore } from '../core/wasm.js';
import { BuildWork, type BuildAsk, type BuildReply } from './build-work.js';
import { unpairedClaims, type ClaimFrame, type ClaimJob } from './claim-judging.js';
import { buffers, drawnIndex, typed, type ScenarioFrame, type ScenarioJob } from './scenario-view.js';

/** The least time between two progress reports while judging. */
const FRAME_MS = 120;

const scope = globalThis as unknown as {
  onmessage: ((ev: MessageEvent<BuildAsk | ClaimJob | ScenarioJob>) => void) | null;
  postMessage(message: BuildReply | ClaimFrame | ScenarioFrame, transfer?: Transferable[]): void;
};
const ready = initCore();
let latest = 0;

/** Let messages in — a newer job, a cancel — before the next step. By a message to itself and not a
 *  timer: a page out of sight has its timers throttled to one a second or slower, and building
 *  between them crawled while the tab was in the background. */
const pause = (): Promise<void> => new Promise((resolve) => {
  const channel = new MessageChannel();
  channel.port1.onmessage = () => { channel.port1.close(); resolve(); };
  channel.port2.postMessage(0);
});

const builds = new BuildWork((reply, transfer) => scope.postMessage(reply, transfer), pause);

scope.onmessage = (ev) => {
  const job = ev.data;
  if (job.kind === 'build' || job.kind === 'drop') {
    // a core that will not start refuses the build aloud rather than leaving the page waiting
    ready.then(() => builds.receive(job), (e) => {
      if (job.kind === 'build') scope.postMessage({ id: job.id, key: job.job.key, refused: String(e) });
    });
    return;
  }
  latest = job.id;                    // a cancel is only this: the job in hand ends at its next step
  if (job.kind === 'cancel') return;
  if (job.kind === 'claims') {
    ready.then(() => judge(job)).catch((e) => scope.postMessage({ id: job.id, error: String(e) }));
    return;
  }
  ready.then(() => scenarios(job))
    .catch((e) => scope.postMessage({ id: job.id, error: String(e) }));
};

/** **Read the scenarios of the page's drawing, and draw the one chosen**: a level at a time,
 *  coarsest first, each posted as it is read, letting messages in between — a newer job ends this
 *  one before its finer levels.  A burst of edits starts only the last job, and the worker's
 *  drawing must name the scenario chosen as the page's does. */
async function scenarios(job: Extract<ScenarioJob, { kind: 'scenario' }>): Promise<void> {
  await pause();
  if (job.id !== latest) return;
  install(job.modules);
  // the levels hold their own copy of the drawing: the document goes once they have it
  const doc = Document.read(job.text);
  let levels: ScenarioLevels;
  let index: number;
  try {
    // the page's meshes, made where it made them again, before its pose
    if (job.meshedAt) doc.sketch.meshAt(job.meshedAt);
    doc.sketch.setX(job.x);
    index = drawnIndex(doc, job.drawn);
    if (job.drawn && index < 0) {
      throw new Error(`the worker's drawing states no scenario \`${job.drawn}\``);
    }
    levels = doc.scenarioLevels(index);
  } finally {
    doc.dispose();
  }
  try {
    for (let got = levels.next(); got; got = levels.next()) {
      const picture = index >= 0 ? typed(got.picture) : null;
      scope.postMessage({ id: job.id, scenarios: got.scenarios, measures: got.measures, picture,
                          final: got.final, levelText: got.levelText }, buffers(picture));
      if (got.final) return;
      await pause();
      if (job.id !== latest) return;
    }
  } finally {
    levels.dispose();
  }
}

/** Judge the drawing's claims about solids a pose at a time, saying where it stands at most every
 *  `FRAME_MS` and whenever a new claim begins, and posting the verdicts once all are judged. */
async function judge(job: Extract<ClaimJob, { kind: 'claims' }>): Promise<void> {
  await pause();                      // a burst of edits: only the last job is started
  if (job.id !== latest) return;
  install(job.modules);
  let doc: Document;
  try {
    doc = Document.read(job.text);
    doc.sketch.setX(job.x);
    const why = unpairedClaims(job.claims, solidClaims(doc.sketch));
    if (why) throw new Error(why);
  } catch (e) {
    scope.postMessage({ id: job.id, error: String(e) });
    return;
  }
  let judge: ClaimJudge | null = null;
  try {
    judge = ClaimJudge.create(doc.sketch);
    let shown = -Infinity, claim = -1;
    for (;;) {
      const progress = judge.progress();
      const now = performance.now();
      if (now - shown > FRAME_MS || progress.claim !== claim) {
        scope.postMessage({ id: job.id, progress });
        shown = now;
        claim = progress.claim;
      }
      await pause();
      if (job.id !== latest) return;
      if (judge.step()) break;
    }
    scope.postMessage({ id: job.id, progress: judge.progress(), verdicts: judge.verdicts() });
  } catch (e) {
    scope.postMessage({ id: job.id, error: String(e) });
  } finally {
    judge?.dispose();
    doc.dispose();
  }
}
