/* The claims about solids, judged off the page's thread: the core's stepped judge through the
 * binding (`core/claims.ts`), the page's half (`app/claim-judging.ts`) against a stub worker, and
 * the worker's `claims` job itself, driven through its message handler. */
import assert from 'node:assert/strict';
import test from 'node:test';

import {
  ClaimJudging, type ClaimFrame, type ClaimJob, type JudgeWorker, type Judging,
} from '../app/claim-judging.js';
import { ClaimJudge, solidClaims } from '../core/claims.js';
import { diagnose } from '../core/diagnose.js';
import * as modules from '../core/modules.js';
import { Document } from '../core/program.js';
import { System } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

/** A block that slides along `reach`, an unknown the solver answers for, and a wall it must
 *  clear: one claim over the travel (37 poses) and one at the pose. */
const SLIDING = `unit mm
use std
use std.place (horizontal, vertical)
in std.front {
o := point
fix((0, 0)) o
p := point hint((10, 0))
}
param reach: Length
in std.front {
o distance(reach, along: u) p
o horizontal p
q := point hint((14, 0))
p distance(4, along: u) q
p horizontal q
r := point hint((14, 4))
q vertical r
q distance(4, along: v) r
s := point hint((10, 4))
p vertical s
p distance(4, along: v) s
(e0 := line(p, q)) -> (e1 := line(q, r)) -> (e2 := line(r, s)) -> (e3 := line(s, p)) -> close
arm_f := face(e0, e1, e2, e3)
arm := solid(arm_f, depth: 3mm)
w0 := point
w1 := point hint((56, 0))
w2 := point hint((56, 4))
w3 := point hint((50, 4))
(g0 := line(w0, w1)) -> (g1 := line(w1, w2)) -> (g2 := line(w2, w3)) -> \
(g3 := line(w3, w0)) -> close
fix((50, 0)) w0
horizontal g0
vertical g1
w0 distance(6) w1
w1 distance(4) w2
horizontal g2
vertical g3
wall_f := face(g0, g1, g2, g3)
wall := solid(wall_f, depth: 3mm)
claim over reach in (10mm, 40mm) { arm clear(1mm) wall }
claim arm clear(40mm) wall
}
`;

const PLAIN = 'unit mm\no := point\nfix((0, 0)) o\n';

/** The drawing solved, as the page has it once an edit has settled. */
function solved(text: string): Document {
  const doc = Document.read(text);
  const sys = new System(doc.sketch);
  sys.solve({});
  sys.dispose();
  return doc;
}

test('the stepped judge says where it stands and comes to the report\'s verdicts', () => {
  modules.forget();
  const doc = solved(SLIDING);
  const claims = solidClaims(doc.sketch);
  assert.equal(claims.length, 2);
  assert.match(claims[0].text, /^claim over reach/);
  // where each was written is the document's to say, by its statement
  const at = doc.spanOfStatement(claims[1].stmt)!;
  assert.equal(doc.text.slice(at.lo, at.hi), 'claim arm clear(40mm) wall');
  assert.equal(doc.spanOfStatement(1e6), undefined, 'no such statement');

  const judge = ClaimJudge.create(doc.sketch);
  const first = judge.progress();
  assert.equal(first.fraction, 0);
  assert.equal(first.stmt, claims[0].stmt);
  assert.match(first.doing, /^judging claim over reach .* \(1 of 2\), pose 1 of 37$/);
  let steps = 1;
  while (!judge.step()) steps++;
  assert.equal(steps, 38, 'a pose a step: the travel\'s 37 and the pose\'s one');
  assert.deepEqual([judge.progress().fraction, judge.progress().doing], [1, 'judged 2 claims']);
  const verdicts = judge.verdicts();
  assert.deepEqual(verdicts.map((v) => v.stmt), claims.map((c) => c.stmt));
  assert.deepEqual(verdicts.map((v) => v.verdict), ['sampled-success', 'refuted']);
  assert.match(verdicts[1].line, /^claim arm clear\(40mm\) wall — refuted/);
  judge.dispose();

  // the page's diagnosis leaves them to the worker, and says the rest unchanged
  const judged = diagnose(doc.sketch);
  const left = diagnose(doc.sketch, { solidClaims: false });
  assert.deepEqual([left.status, left.summary], [judged.status, judged.summary]);
  doc.dispose();
});

class Stub implements JudgeWorker {
  posted: ClaimJob[] = [];
  onmessage: ((ev: MessageEvent<ClaimFrame>) => void) | null = null;
  onerror: ((ev: ErrorEvent) => void) | null = null;
  onmessageerror: ((ev: MessageEvent) => void) | null = null;
  postMessage(job: ClaimJob): void { this.posted.push(job); }
  send(frame: ClaimFrame): void { this.onmessage!({ data: frame } as MessageEvent<ClaimFrame>); }
  get last(): ClaimJob { return this.posted[this.posted.length - 1]; }
}

test('a drawing\'s claims are one job, asked again only when the drawing or its pose moves', () => {
  modules.forget();
  const stub = new Stub();
  const told: (Judging | null)[] = [];
  const cj = new ClaimJudging((j) => told.push(j), () => stub);
  const doc = solved(SLIDING);
  cj.start(doc);
  assert.equal(stub.posted.length, 1);
  const job = stub.last as Extract<ClaimJob, { kind: 'claims' }>;
  assert.equal(job.kind, 'claims');
  assert.deepEqual(job.claims, solidClaims(doc.sketch));
  assert.deepEqual([...job.x], [...doc.sketch.getX()]);
  assert.deepEqual(told[0]?.claims, job.claims);

  cj.start(doc);
  assert.equal(stub.posted.length, 1, 'the same drawing at the same pose: the job stands');

  // frames of an old job are not this drawing's
  stub.send({ id: job.id - 1, error: 'stale' });
  assert.equal(cj.judging?.error, undefined);
  const progress = { doing: 'judging …', fraction: 0.5, claim: 0, claims: 2, pose: 18, poses: 37,
    stmt: job.claims[0].stmt };
  stub.send({ id: job.id, progress });
  assert.deepEqual(cj.judging?.progress, progress);
  const verdicts = [{ stmt: job.claims[1].stmt, line: 'claim arm clear(40mm) wall — refuted',
    statement: '', verdict: 'refuted' as const, measured: 3, tolerance: 0, nearest: null }];
  stub.send({ id: job.id, verdicts });
  assert.deepEqual(cj.judging?.verdicts, verdicts);
  cj.start(doc);
  assert.equal(stub.posted.length, 1, 'judged, the verdicts stand for the same drawing');

  // a pose moved is another question
  const x = doc.sketch.getX();
  x[0] += 1;
  doc.sketch.setX(x);
  cj.start(doc);
  assert.equal(stub.posted.length, 2);
  assert.equal(cj.judging?.verdicts, undefined, 'a new job has no verdicts yet');

  // a drawing with no claims ends the job in hand and says there is nothing to judge
  const plain = Document.read(PLAIN);
  cj.start(plain);
  assert.equal(stub.last.kind, 'cancel');
  assert.equal(told[told.length - 1], null);
  assert.equal(cj.judging, null);

  // a worker that dies fails the judging aloud
  cj.start(doc);
  stub.onerror!({ message: 'gone' } as ErrorEvent);
  assert.equal((cj.judging as Judging | null)?.error, 'gone');
  plain.dispose();
  doc.dispose();
});

test('the worker judges the page\'s drawing a pose at a time and posts the verdicts', async () => {
  modules.forget();
  const posted: ClaimFrame[] = [];
  const scope = globalThis as unknown as {
    onmessage: ((ev: MessageEvent<ClaimJob>) => void) | null;
    postMessage: (m: ClaimFrame) => void;
  };
  scope.postMessage = (m) => posted.push(m);
  await import('../app/mesh-worker.js');
  const doc = solved(SLIDING);
  const claims = solidClaims(doc.sketch);
  const send = (job: ClaimJob): void =>
    scope.onmessage!({ data: job } as unknown as MessageEvent<ClaimJob>);
  send({ kind: 'claims', id: 7, text: doc.text, modules: [], x: doc.sketch.getX(), claims });
  const over = (f: ClaimFrame): boolean => !!(f.verdicts || f.error);
  for (const end = performance.now() + 20000; !posted.some(over) && performance.now() < end;) {
    await new Promise((resolve) => setTimeout(resolve, 10));
  }
  assert.ok(posted.every((f) => f.id === 7));
  const last = posted[posted.length - 1];
  assert.equal(last.error, undefined, last.error);
  assert.deepEqual(last.verdicts?.map((v) => v.verdict), ['sampled-success', 'refuted']);
  // where it stood was said on the way: the first claim's poses, then the second claim
  assert.ok(posted.some((f) => f.progress?.claim === 0 && f.progress.pose === 0));
  assert.ok(posted.some((f) => f.progress?.claim === 1));

  // a drawing the worker does not read as the page does is refused, not judged
  posted.length = 0;
  send({ kind: 'claims', id: 8, text: doc.text, modules: [], x: doc.sketch.getX(),
    claims: claims.slice(1) });
  for (const end = performance.now() + 5000; !posted.length && performance.now() < end;) {
    await new Promise((resolve) => setTimeout(resolve, 10));
  }
  assert.match(posted[0]?.error ?? '', /does not state the claims/);
  doc.dispose();
});
