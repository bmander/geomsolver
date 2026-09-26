/* The page's half of background meshing (`app/field-preview.ts`), driven against a stub worker:
 * what it asks for is the core's answer, a drawing whose surfaces are in hand or finished asks
 * for nothing again, frames of an old job are ignored, surfaces are applied no faster than the
 * page draws them, and a drawing with nothing to mesh cancels the job in hand. */
import assert from 'node:assert/strict';
import test from 'node:test';

import { readFileSync } from 'node:fs';
import { FieldPreview, type Frame, type Job, type MeshJob, type MeshWorker } from '../app/field-preview.js';
import { fieldJobs, type FieldSurface } from '../core/field.js';
import { mesh, provisional } from '../core/mesh.js';
import * as modules from '../core/modules.js';
import { Document } from '../core/program.js';
import { initCore } from '../core/wasm.js';

await initCore();

const TORUS = readFileSync(new URL('../../../rust/examples/swept_torus.sv', import.meta.url), 'utf8');
const PLAIN = 'unit mm\npoint o hint(x: 0, y: 0)\nground o\n';

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
  await wait(150);
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
