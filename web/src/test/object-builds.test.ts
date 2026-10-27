/* The page's objects built by a pool of workers (#671), said by the build bar (#672) and kept
 * between openings (#673), driven against workers of the test's own: in-process ones running the
 * worker's own code (`BuildWork`), and stubs that hold their jobs back.  What a pool is asked is
 * the core's answer, one job a binding; the page builds no object on its own thread and meshes no
 * field; a build a drawing still wants is asked once, kept across an edit and an opening, and a
 * worker keeps its document between jobs. */
import assert from 'node:assert/strict';
import test from 'node:test';
import { readFileSync } from 'node:fs';

import { FieldMesher } from '../core/field.js';
import { mesh, objects, stale } from '../core/mesh.js';
import { ExactBuilder, objectJobs, objectStates, supplyObject, supplyObjects } from '../core/objects.js';
import * as modules from '../core/modules.js';
import { Document } from '../core/program.js';
import { initCore } from '../core/wasm.js';
import { barOf } from '../app/build-bar.js';
import { BuildWork, type BuildAsk, type BuildReply } from '../app/build-work.js';
import { ObjectBuilds, type BuildRow, type PoolWorker } from '../app/object-builds.js';
import { MemoryStore } from '../app/surface-store.js';
import { docView } from './views.js';

(globalThis as { requestAnimationFrame?: unknown }).requestAnimationFrame ??=
  (fn: FrameRequestCallback) => { fn(0); return 0; };
(globalThis as { cancelAnimationFrame?: unknown }).cancelAnimationFrame ??= () => {};

await initCore();

const TORUS = readFileSync(new URL('../../../rust/examples/swept_torus.sv', import.meta.url), 'utf8');

/** A block of 40 by 20 by 10, `w` long, on the front plane. */
const block = (w = 40, name = 'block'): string => `in std.front {\n  ${name}_a := point hint((0, 0))\n`
  + `  ${name}_b := point hint((${w}, 0))\n  ${name}_c := point hint((${w}, 20))\n  ${name}_d := point hint((0, 20))\n`
  + `  fix((0, 0)) ${name}_a\n  ${name}_ab := line(${name}_a, ${name}_b)\n  ${name}_bc := line(${name}_b, ${name}_c)\n`
  + `  ${name}_cd := line(${name}_c, ${name}_d)\n  ${name}_da := line(${name}_d, ${name}_a)\n  horizontal ${name}_ab\n`
  + `  horizontal ${name}_cd\n  vertical ${name}_bc\n  vertical ${name}_da\n  length(${w}) ${name}_ab\n`
  + `  length(20) ${name}_bc\n}\n${name} := solid(face(${name}_ab, ${name}_bc, ${name}_cd, ${name}_da), depth: 10)\n`;

const BLOCK = `unit mm\nuse std\n${block()}`;
const TWO = `unit mm\nuse std\n${block(40, 'one')}${block(30, 'two')}`;

/** Two closed instances of one component, standing apart: one binding, placed twice. */
const PAIR = 'unit mm\nuse std\nuse std.place (offset)\ncomponent Plate(w: Length) {\n  in std.front {\n'
  + '    a := point hint((1, 1))\n    b := point hint((w, 1))\n    c := point hint((w, w))\n    e := point hint((1, w))\n'
  + '    a offset(dx: 1mm, dy: 1mm) std.origin\n    b offset(dx: w, dy: 1mm) std.origin\n'
  + '    c offset(dx: w, dy: w) std.origin\n    e offset(dx: 1mm, dy: w) std.origin\n    ab := line(a, b)\n'
  + '    bc := line(b, c)\n    ce := line(c, e)\n    ea := line(e, a)\n'
  + '    body := solid(face(ab, bc, ce, ea), from: 0mm, to: 2mm)\n  }\n}\n'
  + 'in (f := frame) { one := Plate(w: 11mm) }\nfix(dir == (1, 0, 0)) f.u\nfix(dir == (0, 1, 0)) f.v\n'
  + 'fix(origin == (0, 0, 0)) f\n'
  + 'in (g := frame) { two := Plate(w: 11mm) }\nfix(dir == (1, 0, 0)) g.u\nfix(dir == (0, 1, 0)) g.v\n'
  + 'fix(origin == (30, 0, 0)) g\n';

const tick = (ms = 0): Promise<void> => new Promise((resolve) => setTimeout(resolve, ms));

async function until(done: () => boolean, ms = 20000): Promise<void> {
  for (const end = performance.now() + ms; !done() && performance.now() < end;) await tick(5);
}

/** A pool worker running the worker's own code on this thread: what a real one does, in turn. */
class Local implements PoolWorker {
  readonly work = new BuildWork((r) => this.onmessage?.({ data: r } as MessageEvent<BuildReply>), () => tick());
  asked: BuildAsk[] = [];
  onmessage: ((ev: MessageEvent<BuildReply>) => void) | null = null;
  onerror: ((ev: ErrorEvent) => void) | null = null;
  onmessageerror: ((ev: MessageEvent) => void) | null = null;
  postMessage(ask: BuildAsk): void { this.asked.push(ask); this.work.receive(ask); }
}

/** A pool worker that holds every job it is asked until the test answers for it. */
class Held implements PoolWorker {
  asked: BuildAsk[] = [];
  onmessage: ((ev: MessageEvent<BuildReply>) => void) | null = null;
  onerror: ((ev: ErrorEvent) => void) | null = null;
  onmessageerror: ((ev: MessageEvent) => void) | null = null;
  postMessage(ask: BuildAsk): void { this.asked.push(ask); }
  reply(r: BuildReply): void { this.onmessage!({ data: r } as MessageEvent<BuildReply>); }
  get builds(): Extract<BuildAsk, { kind: 'build' }>[] {
    return this.asked.filter((a): a is Extract<BuildAsk, { kind: 'build' }> => a.kind === 'build');
  }
}

/** A pool of `Local` workers, every one made counted. */
function localPool(): { spawn: () => PoolWorker; made: Local[] } {
  const made: Local[] = [];
  return { made, spawn: () => { const w = new Local(); made.push(w); return w; } };
}

/** The build a real worker hands back for solid `solid` of `text`. */
function built(text: string, solid?: number): Uint8Array {
  const doc = Document.read(text);
  try {
    const b = ExactBuilder.create(doc.sketch, solid ?? objectJobs(doc.sketch)[0].solid);
    try { while (!b.step()) { /* a stage at a time */ } return b.display(); }
    finally { b.dispose(); }
  } finally { doc.dispose(); }
}

test('the jobs are the core\'s, one a binding, and a sketch supplied builds none of its own', () => {
  modules.forget();
  const doc = Document.read(PAIR);
  const objs = objects(doc.sketch);
  assert.equal(objs.length, 2);
  supplyObjects(doc.sketch);
  const jobs = objectJobs(doc.sketch);
  assert.equal(jobs.length, 1, 'the two placements of one binding are one build');
  assert.deepEqual(jobs[0].objects, objs.map((o) => o.index));
  assert.match(jobs[0].key, /^[0-9a-f]{16}$/);
  assert.deepEqual(objectStates(doc.sketch).map((s) => s.state), ['building', 'building']);
  assert.throws(() => mesh(doc.sketch, objs[0].index, 0), /still being built/);
  // the one build supplied, both are drawn, each where it stands
  supplyObject(doc.sketch, jobs[0].solid, built(PAIR, jobs[0].solid));
  assert.deepEqual(objectStates(doc.sketch).map((s) => s.state), ['built', 'built']);
  for (const o of objs) assert.ok(mesh(doc.sketch, o.index, 0).positions.length > 0, o.name);
  // a buffer that is no surface is refused by the core, never drawn
  assert.throws(() => supplyObject(doc.sketch, jobs[0].solid, new Uint8Array(16)), /another format|not as long/);
  doc.dispose();
});

test('a worker keeps its document between jobs, and reads again only for a structural edit', async () => {
  const replies: BuildReply[] = [];
  const work = new BuildWork((r) => replies.push(r), () => tick());
  const drawing = (text: string) => { const d = Document.read(text); const x = d.sketch.getX(); const jobs = objectJobs(d.sketch); d.dispose(); return { x, jobs, text }; };
  const ask = (id: number, text: string, k = 0) => {
    const d = drawing(text);
    work.receive({ kind: 'build', id, drawing: { text, modules: [], x: d.x }, job: d.jobs[k] });
  };
  ask(1, TWO, 0);
  ask(2, TWO, 1);
  await until(() => replies.filter((r) => r.display || r.refused).length === 2);
  assert.ok(replies.every((r) => !r.refused), JSON.stringify(replies.find((r) => r.refused)));
  assert.equal(work.reads, 1, 'two jobs of one drawing: read once');
  // a dimension changed is a numeric edit: the same elaboration, the page's values over it
  const edited = TWO.replace('length(30) two_ab', 'length(31) two_ab');
  ask(3, edited, 1);
  await until(() => replies.some((r) => r.id === 3 && (r.display || r.refused)));
  assert.ok(replies.find((r) => r.id === 3)?.display || !replies.find((r) => r.id === 3 && r.refused),
            JSON.stringify(replies.find((r) => r.id === 3 && r.refused)));
  assert.equal(work.reads, 1, 'a numeric edit reads nothing again');
  // a statement added is structural
  ask(4, `${TWO}${block(10, 'three')}`, 2);
  await until(() => replies.some((r) => r.id === 4 && (r.display || r.refused)));
  assert.equal(work.reads, 2);
  // the job dropped ends at its next stage, and says nothing more
  ask(5, TWO, 0);
  work.receive({ kind: 'drop', id: 5 });
  await tick(50);
  assert.ok(!replies.some((r) => r.id === 5 && (r.display || r.refused)), 'a dropped build hands nothing back');
  // a job its drawing does not name as the page's did is refused in words, never built
  const d = drawing(TWO);
  work.receive({ kind: 'build', id: 6, drawing: { text: TWO, modules: [], x: d.x }, job: { ...d.jobs[0], key: '0'.repeat(16) } });
  await until(() => replies.some((r) => r.id === 6));
  assert.match(replies.find((r) => r.id === 6)?.refused ?? '', /does not build `one` as the page's does/);
});

test('an opening builds every object in the pool, none on the page\'s thread, and meshes no field', async () => {
  modules.forget();
  const created: unknown[] = [];
  const create = FieldMesher.create;
  FieldMesher.create = (...args: Parameters<typeof create>) => { created.push(args); return create(...args); };
  try {
    const view = docView(`${TORUS}${block(40, 'beside')}`);
    const held = new Held();
    view.usePool(() => held, null, 1);
    view.readBehind();
    const jobs = view.builds.rows;
    assert.ok(jobs.length >= 2, JSON.stringify(jobs));
    assert.ok(jobs[0].swept, 'the swept one first: it is the long one');
    // held back, nothing is drawn, and the page built nothing: the box cut no object
    view.box3d.sync(view);
    assert.equal(view.box3d.stats.cut, 0, 'no object built on the page\'s thread');
    for (const o of objects(view.sketch)) assert.throws(() => mesh(view.sketch, o.index, 0), /still being built/);
    assert.deepEqual(created, [], 'no field meshed');
    // one worker of one: one job in hand, the rest waiting
    assert.equal(held.builds.length, 1);
    assert.deepEqual(view.builds.rows.map((r) => r.state), ['building', ...jobs.slice(1).map(() => 'waiting')]);
    // the swept one refused (as the kernel says), the static one built: drawn, and no field meshed
    held.reply({ id: held.builds[0].id, key: held.builds[0].job.key, refused: '`part`: outside the generating class (at admission)' });
    await until(() => held.builds.length === 2);
    const second = held.builds[1];
    held.reply({ id: second.id, key: second.job.key, display: built(view.source, second.job.solid), ms: 3 });
    await until(() => view.builds.rows.every((r) => r.state === 'built' || r.state === 'refused'));
    await tick(20);                     // the redraw the arrival asked for
    view.box3d.sync(view);
    assert.equal(view.box3d.stats.cut, 1, 'the static object cut from its supplied build');
    assert.deepEqual(created, [], 'still no field meshed: a refused object is unbuilt, in its words');
    const part = objects(view.sketch).find((o) => o.name === 'part')!;
    assert.throws(() => mesh(view.sketch, part.index, 0), /outside the generating class/);
    view.builds.dispose();
    view.doc.dispose();
  } finally {
    FieldMesher.create = create;
  }
});

test('a placement of a binding is built once across the pool', async () => {
  const view = docView(PAIR);
  const pool = localPool();
  view.usePool(pool.spawn, null, 3);
  view.readBehind();
  await until(() => view.builds.rows.every((r) => r.state === 'built'));
  await tick(20);
  const asked = pool.made.flatMap((w) => w.asked).filter((a) => a.kind === 'build');
  assert.equal(asked.length, 1, 'one build for both');
  assert.equal(pool.made.length, 1, 'and no worker made that nothing asked of');
  view.box3d.sync(view);
  assert.equal(view.box3d.stats.cut, 2, 'both drawn from it');
  view.builds.dispose();
  view.doc.dispose();
});

test('an edit asks only for what it changed, and draws the rest from what is in hand', async () => {
  const view = docView(TWO);
  view.autoSolve = true;
  const pool = localPool();
  const builds = view.usePool(pool.spawn, null, 2);
  view.readBehind();
  await until(() => builds.rows.length === 2 && builds.rows.every((r) => r.state === 'built'));
  const asked = (): number => pool.made.flatMap((w) => w.asked).filter((a) => a.kind === 'build').length;
  assert.equal(asked(), 2);
  // the second block lengthened: its build asked again, the first's drawn from hand; till the new
  // one arrives, the second's last build is drawn, stale
  assert.ok(view.setProgram(view.source.replace('length(30) two_ab', 'length(35) two_ab')), 'edited');
  assert.equal(asked(), 3, 'only the edited block is built again');
  const two = objects(view.sketch).find((o) => o.name === 'two')!;
  const one = objects(view.sketch).find((o) => o.name === 'one')!;
  assert.ok(!stale(view.sketch, one.index), 'the unchanged block drawn from its build in hand');
  assert.ok(stale(view.sketch, two.index), 'the edited one drawn as it was, stale, till its build arrives');
  await until(() => builds.rows.every((r) => r.state === 'built'));
  assert.ok(!stale(view.sketch, two.index));
  // an undo asks for nothing: the build it wants again is in hand
  view.undo();
  assert.equal(asked(), 3);
  builds.dispose();
  view.doc.dispose();
});

test('the build bar says how many are left while the page takes input, and goes when they finish', async () => {
  const view = docView(TWO);
  view.autoSolve = true;
  const held = new Held();
  const told: BuildRow[][] = [];
  view.onBuilds = (rows) => told.push(rows.map((r) => ({ ...r })));
  view.usePool(() => held, null, 1);
  view.readBehind();
  let bar = barOf(view.builds.rows);
  assert.ok(bar.shown);
  assert.equal(bar.left, 2);
  assert.match(bar.text, /^building 1 of 2 · one — starting$/);
  // the page is not held: an edit is taken, drawn and told while the builds are out
  let changed = 0;
  view.onChanged = () => changed++;
  assert.ok(view.setProgram(view.source.replace('length(40) one_ab', 'length(41) one_ab')), 'an edit taken');
  assert.ok(changed > 0, 'and the shell told at once');
  assert.ok(view.outline().length > 0, 'the outline reads');
  // the first job (edited away) was dropped; the edited drawing's are asked afresh
  const drops = held.asked.filter((a) => a.kind === 'drop');
  assert.equal(drops.length, 1);
  bar = barOf(view.builds.rows);
  assert.equal(bar.left, 2);
  // the builds arrive, a stage said on the way
  const first = held.builds[held.builds.length - 1];
  held.reply({ id: first.id, key: first.job.key, progress: { doing: 'meshing the exact surface', done: 1, total: 2, said: null } });
  bar = barOf(view.builds.rows);
  assert.match(bar.text, /meshing the exact surface/);
  assert.ok(bar.fraction > 0 && bar.fraction < 1);
  held.reply({ id: first.id, key: first.job.key, display: built(view.source, first.job.solid), ms: 1 });
  await until(() => held.builds.length === 3);
  bar = barOf(view.builds.rows);
  assert.equal(bar.left, 1);
  assert.match(bar.text, /^building 2 of 2/);
  const last = held.builds[2];
  held.reply({ id: last.id, key: last.job.key, refused: 'the built boundary is invalid (at blank)' });
  bar = barOf(view.builds.rows);
  assert.equal(bar.left, 0);
  assert.ok(bar.shown && bar.marked, 'a refusal leaves a mark till it is seen');
  assert.match(bar.text, /1 of 2 not built/);
  assert.equal(barOf(view.builds.rows, true).shown, false, 'seen, the bar goes');
  assert.ok(told.length > 3, 'every move told');
  view.builds.dispose();
  view.doc.dispose();
});

test('a store keeps builds between openings: the second asks no worker, and draws the same', async () => {
  const store = new MemoryStore();
  const open = async (text: string): Promise<{ asked: number; triangles: number[] }> => {
    const view = docView(text);
    const pool = localPool();
    const builds = view.usePool(pool.spawn, store, 2);
    view.readBehind();
    await until(() => builds.rows.length > 0 && builds.rows.every((r) => r.state === 'built'));
    const asked = pool.made.flatMap((w) => w.asked).filter((a) => a.kind === 'build').length;
    const triangles = objects(view.sketch).map((o) => mesh(view.sketch, o.index, 0).positions.length / 9);
    builds.dispose();
    view.doc.dispose();
    return { asked, triangles };
  };
  const first = await open(TWO);
  assert.equal(first.asked, 2);
  assert.ok((await store.size()) > 0);
  const second = await open(TWO);
  assert.equal(second.asked, 0, 'every build taken from the store');
  assert.deepEqual(second.triangles, first.triangles, 'the same faces');
  // a changed object is built again, the others taken from the store
  const third = await open(TWO.replace(/(two := solid\(.*depth: )10/, '$112'));
  assert.equal(third.asked, 1);
  // bounded: past its limit the least recently used goes
  const small = new MemoryStore(1);
  await small.put('a', new Uint8Array(8));
  assert.equal((await small.get(['a'])).size, 0);
  await store.clear();
  assert.equal(await store.size(), 0);
});
