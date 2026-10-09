/* The page's half of background meshing (`app/field-preview.ts`), driven against a stub worker:
 * what it asks for is the core's answer, a drawing whose surfaces are in hand or finished asks
 * for nothing again, frames of an old job are ignored, surfaces are applied no faster than the
 * page draws them, and a drawing with nothing to mesh cancels the job in hand. */
import assert from 'node:assert/strict';
import test from 'node:test';

import { readFileSync } from 'node:fs';
import { FieldPreview, type ExactJob, type Frame, type Job, type MeshJob, type MeshWorker, type Refining } from '../app/field-preview.js';
import { ExactBuilder, fieldJobs, type FieldSurface } from '../core/field.js';
import { derived, derivedDetailFree, preview as showSolids } from '../core/derived.js';
import { mesh, provisional } from '../core/mesh.js';
import * as modules from '../core/modules.js';
import { Document } from '../core/program.js';
import { initCore } from '../core/wasm.js';

await initCore();

const TORUS = readFileSync(new URL('../../../rust/examples/swept_torus.sv', import.meta.url), 'utf8');
const PLAIN = 'unit mm\no := point\nfix((0, 0)) o\n';

class Stub implements MeshWorker {
  posted: Job[] = [];
  onmessage: ((ev: MessageEvent<Frame>) => void) | null = null;
  onerror: ((ev: ErrorEvent) => void) | null = null;
  onmessageerror: ((ev: MessageEvent) => void) | null = null;
  postMessage(job: Job): void { this.posted.push(job); }
  send(frame: Frame): void { this.onmessage!({ data: frame } as MessageEvent<Frame>); }
  get last(): Job { return this.posted[this.posted.length - 1]; }
}

/** A surface of one triangle: the page stores what it is handed, and draws that. */
const surface = (provisional: boolean): FieldSurface => ({
  vertices: new Float64Array([0, 0, 0, 1, 0, 0, 0, 1, 0]), triangles: new Uint32Array([0, 1, 2]), provisional,
});

const wait = (ms: number): Promise<void> => new Promise((resolve) => setTimeout(resolve, ms));

/** Wait until `done`, or `ms` have passed. The throttle spaces surfaces by how long the page was
 *  busy after the last, which counts any stall of the thread — V8 swapping in optimised code for
 *  the core is one, a tenth of a second — so a fixed sleep says how fast this machine is. */
async function until(done: () => boolean, ms = 2000): Promise<void> {
  for (const end = performance.now() + ms; !done() && performance.now() < end;) await wait(10);
}

function preview(): { fp: FieldPreview; stub: Stub; arrived: (string | undefined)[] } {
  const stub = new Stub();
  const arrived: (string | undefined)[] = [];
  const fp = new FieldPreview((e) => arrived.push(e), () => {}, () => stub);
  return { fp, stub, arrived };
}

test('a swept drawing is one job of the core\'s naming, and the same drawing asks for nothing again', () => {
  modules.forget();
  const { fp, stub } = preview();
  const doc = Document.read(TORUS);
  fp.start(doc);
  assert.equal(stub.posted.length, 1);
  const job = stub.last as MeshJob;
  assert.equal(job.kind, 'mesh');
  assert.deepEqual(job.solids, fieldJobs(doc.sketch));
  // only what the host handed over travels: `std` is the worker's library too
  assert.deepEqual(job.modules, []);
  fp.start(doc);
  const again = Document.read(TORUS);
  fp.start(again);
  assert.equal(stub.posted.length, 1, 'the job in hand stands for the same drawing');
  doc.dispose();
  again.dispose();
});

test('frames of an old job are ignored, and surfaces are applied no faster than the page draws', async () => {
  const { fp, stub, arrived } = preview();
  const doc = Document.read(TORUS);
  fp.start(doc);
  const job = stub.last as MeshJob;
  const { solid, key } = job.solids[0];
  stub.send({ id: job.id - 1, solid, key, surface: surface(true) });
  assert.equal(arrived.length, 0, 'an old job\'s frame');
  stub.send({ id: job.id, solid, key, surface: surface(true) });
  assert.equal(arrived.length, 1);
  assert.equal(mesh(doc.sketch, solid, 0).positions.length, 9);
  assert.ok(provisional(doc.sketch, solid));
  // the next waits for the redraw the first began
  stub.send({ id: job.id, solid, key, surface: surface(false) });
  assert.equal(arrived.length, 1);
  await until(() => arrived.length > 1);
  assert.equal(arrived.length, 2);
  assert.ok(!provisional(doc.sketch, solid));
  // a new elaboration of the same drawing is given the finished surface, and nothing is asked
  const next = Document.read(TORUS);
  fp.start(next);
  assert.equal(stub.posted.length, 1);
  assert.equal(arrived.length, 3);
  assert.ok(!provisional(next.sketch, solid));
  assert.equal(mesh(next.sketch, solid, 0).positions.length, 9);
  doc.dispose();
  next.dispose();
});

test('an edited drawing is a new job, and one with nothing to mesh cancels the job in hand', async () => {
  const { fp, stub, arrived } = preview();
  const doc = Document.read(TORUS);
  fp.start(doc);
  const first = stub.last as MeshJob;
  const edited = Document.read(TORUS.replace('to: 75deg', 'to: 70deg'));
  fp.start(edited);
  const second = stub.last as MeshJob;
  assert.equal(second.kind, 'mesh');
  assert.ok(second.id > first.id);
  assert.notEqual(second.solids[0].key, first.solids[0].key);
  const plain = Document.read(PLAIN);
  fp.start(plain);
  assert.deepEqual(stub.last, { kind: 'cancel', id: second.id + 1 });
  const { solid, key } = second.solids[0];
  stub.send({ id: second.id, solid, key, surface: surface(false) });
  await wait(20);
  assert.equal(arrived.length, 0, 'a cancelled job\'s frame');
  doc.dispose();
  edited.dispose();
  plain.dispose();
});

test('a new fineness meshes the drawing again, finished surfaces and all, and says how fine', async () => {
  const { fp, stub, arrived } = preview();
  const doc = Document.read(TORUS);
  fp.start(doc);
  const first = stub.last as MeshJob;
  assert.equal(first.fineness, 1);
  const { solid, key } = first.solids[0];
  stub.send({ id: first.id, solid, key, surface: surface(false) });
  await wait(20);
  assert.equal(arrived.length, 1);
  fp.setFineness(1, doc);
  assert.equal(stub.posted.length, 1, 'the fineness in hand changes nothing');
  fp.setFineness(2, doc);
  const second = stub.last as MeshJob;
  assert.equal(second.kind, 'mesh');
  assert.ok(second.id > first.id);
  assert.equal(second.fineness, 2);
  assert.deepEqual(second.solids, first.solids, 'the same drawing, meshed again');
  assert.equal(fp.fineness, 2);
  // a later elaboration of it is not handed the surface finished at the old fineness
  const next = Document.read(TORUS);
  fp.start(next);
  assert.equal(stub.posted.length, 2, 'the job at the new fineness stands');
  assert.equal(arrived.length, 1);
  doc.dispose();
  next.dispose();
});

test('a swept object\'s exact surface is built a stage at a time, and replaces its preview', async () => {
  const doc = Document.read(TORUS);
  const [{ solid, key }] = fieldJobs(doc.sketch);
  // the core's builder, as the exact worker steps it: what each next stage does, then the surface
  const builder = ExactBuilder.create(doc.sketch, solid);
  const said: string[] = [builder.progress().doing];
  while (!builder.step()) said.push(builder.progress().doing);
  const built = builder.surface();
  const total = builder.progress().total;
  builder.dispose();
  assert.equal(said[0], 'admitting its sweeps to the generating class');
  assert.ok(said.includes('cutting the blank by its sheets'), said.join(' / '));
  assert.equal(said.length, total);
  assert.ok(built.exact && built.exact.of.length === built.triangles.length / 3);

  // the page: a field job and an exact job; the exact surface, arriving, is applied in place of
  // the preview and ends the field's job, which asks for nothing else
  const field = new Stub(), exact = new Stub();
  const told: Refining[][] = [];
  const fp = new FieldPreview(() => {}, (r) => told.push(r.map((x) => ({ ...x, exact: x.exact && { ...x.exact } }))),
    () => field, () => exact);
  fp.start(doc);
  assert.equal((field.last as MeshJob).kind, 'mesh');
  const job = exact.last as ExactJob;
  assert.equal(job.kind, 'exact');
  assert.deepEqual(job.solids.map((s) => s.key), [key]);
  exact.send({ id: job.id, solid, key, exact: { doing: 'cutting the blank by its sheets', done: 4, total: 7, said: null } });
  assert.equal(told[told.length - 1][0].exact?.doing, 'cutting the blank by its sheets');
  exact.send({ id: job.id, solid, key, exact: { doing: 'built', done: 7, total: 7, said: null }, surface: built });
  await wait(100);
  assert.ok(told[told.length - 1][0].exact?.built);
  assert.equal(provisional(doc.sketch, solid), false);
  assert.equal(mesh(doc.sketch, solid, 0).positions.length / 9, built.triangles.length / 3);
  assert.equal((field.last as Job).kind, 'cancel', 'the preview asks for nothing the exact surface has given');
  // a later elaboration of the same drawing is given the exact surface, and asks for nothing
  const posted = [field.posted.length, exact.posted.length];
  const again = Document.read(TORUS);
  fp.start(again);
  assert.deepEqual([field.posted.length, exact.posted.length], posted);
  assert.equal(mesh(again.sketch, solid, 0).positions.length / 9, built.triangles.length / 3);
  doc.dispose();
  again.dispose();
});

test('an object the exact build refuses keeps its field\'s surface, and says so', () => {
  const doc = Document.read(TORUS);
  const [{ solid, key }] = fieldJobs(doc.sketch);
  const field = new Stub(), exact = new Stub();
  const told: Refining[][] = [];
  const fp = new FieldPreview(() => {}, (r) => told.push(r.map((x) => ({ ...x, exact: x.exact && { ...x.exact } }))),
    () => field, () => exact);
  fp.start(doc);
  const job = exact.last as ExactJob;
  exact.send({ id: job.id, solid, key, exactError: 'outside the generating-sweep class' });
  assert.equal(told[told.length - 1][0].exact?.error, 'outside the generating-sweep class');
  assert.equal((field.last as Job).kind, 'mesh', 'the preview goes on');
  // and is not asked for again by the same drawing
  const posted = exact.posted.length;
  fp.start(doc);
  assert.equal(exact.posted.length, posted);
  doc.dispose();
});

test('a picture of swept solids alone is the same at every zoom; one of a static solid is not', () => {
  const swept = Document.read(TORUS);
  showSolids(swept.sketch);
  // the torus example's preview projects its swept part, which is one surface whatever the zoom
  assert.ok(derived(swept.sketch, 0.1).length > 0);
  assert.equal(derivedDetailFree(swept.sketch), true);
  const plain = Document.read('unit mm\nuse std\nin std.front {\no := point\nfix((0, 0)) o\n'
    + 'c := circle(center: o) hint(r: 5)\nradius(5mm) c\n}\nbody := solid(face(c), depth: 3mm)\n');
  showSolids(plain.sketch);
  assert.ok(derived(plain.sketch, 0.1).length > 0);
  assert.equal(derivedDetailFree(plain.sketch), false, 'a static solid is cut finer as the zoom asks');
  swept.dispose();
  plain.dispose();
});
