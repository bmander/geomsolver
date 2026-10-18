/* The timeline (#398 L): a drawing over a parameter scrubbed in the page.  `mechanisms/four_bar.sv`
 * stood at an instant where the core's march puts it (its closed form, and the same instant a
 * `.svd` states); an edit made at home and the drawing stood back after it; the crank's pin
 * dragged along its march; and a march that stops at a singular instant stopping the timeline
 * there.  Nothing shown at an instant is written back.  An edit takes no march before it returns,
 * and a scrub of the V-twin moves its parts in the 3D box rather than cutting them again (#412). */
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

import * as drawings from '../core/drawing.js';
import { Point } from '../core/model.js';
import { Scrub } from '../core/parameters.js';
import { PlanDrag } from '../core/decompose.js';
import { Document } from '../core/program.js';
import { initCore } from '../core/wasm.js';
import { SketchView } from '../app/view.js';
import { canvasOf, fakeCanvas, pointer } from './canvas.js';

(globalThis as { requestAnimationFrame?: unknown }).requestAnimationFrame ??=
  (fn: FrameRequestCallback) => { fn(0); return 0; };
(globalThis as { cancelAnimationFrame?: unknown }).cancelAnimationFrame ??= () => {};

await initCore();
const files: Record<string, string> =
  JSON.parse(await readFile(new URL('../examples/sources.json', import.meta.url), 'utf8'));
const FOUR_BAR = files['mechanisms/four_bar.sv'];
const TRIPLE_ROCKER = files['mechanisms/triple_rocker.sv'];
const VTWIN = files['vtwin/assembly.sv'];

/** The rocker pin of a four-bar (ground `g`, crank `r`, coupler `l`, rocker `k`) at crank angle
 *  `t` degrees, the coupler above the ground: the circles about the crank pin and the far pivot
 *  met. */
function rockerPin(g: number, r: number, l: number, k: number, t: number): [number, number] {
  const [s, c] = [Math.sin((t * Math.PI) / 180), Math.cos((t * Math.PI) / 180)];
  const b = [r * c, r * s];
  const [dx, dy] = [g - b[0], -b[1]];
  const e = Math.hypot(dx, dy);
  const along = (l * l - k * k + e * e) / (2 * e);
  const h = Math.sqrt(l * l - along * along);
  const [ux, uy] = [dx / e, dy / e];
  return [b[0] + along * ux - h * uy, b[1] + along * uy + h * ux];
}

/** A view on `text`, solved and followed, as the page opens it. */
function opened(text: string): SketchView {
  const view = new SketchView(fakeCanvas(), Document.read(text));
  assert.ok(view.doc.ok, JSON.stringify(view.doc.diagnostics));
  view.afterEdit();
  return view;
}

function at(view: SketchView, name: string): [number, number] {
  const p = view.doc.entity(name);
  assert.ok(p instanceof Point, `no point ${name}`);
  return [p.x.value, p.y.value];
}

function near(got: [number, number], want: [number, number], tol: number, what: string): void {
  assert.ok(Math.hypot(got[0] - want[0], got[1] - want[1]) < tol,
            `${what}: (${got}) against (${want})`);
}

test('the four-bar is scrubbed to an instant where its march stands it', (t) => {
  const view = opened(FOUR_BAR);
  t.after(() => view.doc.dispose());
  const [p] = view.parameters();
  assert.deepEqual([p.name, p.dim, p.fromText, p.toText], ['t', 'Angle', '0deg', '360deg']);
  const m = view.marchOf('t');
  assert.ok(m?.followed);
  assert.equal(m.reached, 360);
  assert.equal(m.singular, null);
  assert.equal(m.text, 't: followed from 0deg to 360deg');
  const before = view.source;

  assert.ok(view.showInstant('t', 90));
  assert.deepEqual(view.instant, { name: 't', at: 90 });
  near(at(view, 'c'), rockerPin(40, 10, 35, 25, 90), 1e-8, 'the rocker pin a quarter turn on');
  near(at(view, 'b'), [0, 10], 1e-8, 'the crank pin');
  assert.equal(view.reading('t', 90), '90deg');
  // the same instant a drawing states, which the core stands its model at
  const svg = drawings.render('model m from "four_bar.sv" at t = 90deg\n'
    + 'sheet s {\n  sketch a(m) at (30mm, 40mm)\n  measure distance(m.a, m.c) in a\n}\n',
                              'mechanisms/quarter.svd', files);
  assert.ok(svg.includes('>39.9994</text>'), svg);
  assert.equal(Math.hypot(...at(view, 'c')).toFixed(4), '39.9994');
  // what is shown at an instant is not written back
  view.syncSeeds();
  assert.equal(view.source, before);

  assert.ok(view.showInstant('t', null));
  near(at(view, 'c'), rockerPin(40, 10, 35, 25, 0), 1e-8, 'home');
  assert.equal(view.source, before);
});

test('an edit at an instant is made at home, and the drawing stood back there', (t) => {
  const view = opened(FOUR_BAR);
  t.after(() => view.doc.dispose());
  view.showInstant('t', 90);
  // a longer ground, taken as an edit of the same document
  view.takeDocument(Document.read(view.source.replace('g := 40 ', 'g := 42 ')));
  assert.deepEqual(view.instant, { name: 't', at: 90 });
  near(at(view, 'c'), rockerPin(42, 10, 35, 25, 90), 1e-8, 'stood back a quarter turn on');
  // the seed written is where the rocker pin is drawn, at home, not where it is shown
  const hint = /c := point hint\(\(([-\d.e]+), ([-\d.e]+)\)\)/.exec(view.source);
  assert.ok(hint, view.source);
  near([Number(hint[1]), Number(hint[2])], rockerPin(42, 10, 35, 25, 0), 1e-3, 'the seed');
});

test('dragging the crank pin scrubs the drawing along its march', (t) => {
  const view = opened(FOUR_BAR);
  t.after(() => view.doc.dispose());
  const cv = canvasOf(view);
  const before = view.source;
  const pin = (deg: number, r: number) => view.w2s(r * Math.cos((deg * Math.PI) / 180),
                                                    r * Math.sin((deg * Math.PI) / 180));
  cv.fire('pointerdown', pointer(...pin(0, 10)));
  assert.equal(Scrub.live, 1, 'a press on the pin takes hold of the march');
  assert.equal(PlanDrag.live, 0);
  for (const deg of [30, 60, 120]) {
    cv.fire('pointermove', pointer(...pin(deg, 12)));
    // to the pointer's resolution: a place is read to the digits the zoom justifies
    assert.ok(Math.abs((view.instant?.at ?? NaN) - deg) < 5e-2, `${view.instant?.at} for ${deg}`);
    near(at(view, 'c'), rockerPin(40, 10, 35, 25, view.instant!.at!), 1e-8, `at ${deg}`);
  }
  cv.fire('pointerup', pointer(...pin(120, 12)));
  assert.equal(Scrub.live, 0);
  assert.equal(view.source, before, 'a scrub writes nothing back');
  // shown at an instant, the fixed pivot does not move and nothing else is dragged
  let said = '';
  view.onStatus = (s) => { said = s; };
  cv.fire('pointerdown', pointer(...view.w2s(0, 0)));
  assert.equal(Scrub.live + PlanDrag.live, 0);
  assert.equal(said, 'a: it does not move as `t` runs');
  cv.fire('pointerup', pointer(...view.w2s(0, 0)));
});

test('a timeline stops at the singular instant its march stops at', (t) => {
  const view = opened(TRIPLE_ROCKER);
  t.after(() => view.doc.dispose());
  const m = view.marchOf('t');
  assert.ok(m?.followed);
  assert.equal(m.singular?.kind, 'limit');
  assert.equal(m.singular?.text,
               'singular at t = 129.838deg: no drawing stands past it, the motion turns back');
  assert.ok(view.showInstant('t', 200));
  assert.equal(view.instant?.at, m.reached, 'past the limit, the drawing stands at it');
});

test('an edit takes no march before it returns: the timeline gets its reach once followed', async (t) => {
  const view = opened(FOUR_BAR);
  t.after(() => view.doc.dispose());
  // `opened` is an edit: nothing followed, the timeline the interval alone
  assert.equal(view.followedMarch('t'), undefined);
  assert.deepEqual(view.parameters().map((p) => p.name), ['t']);
  // the page idle, it is followed
  await new Promise((done) => setTimeout(done, 400));
  const m = view.followedMarch('t');
  assert.ok(m?.followed);
  assert.equal(m.reached, 360);
  // another edit at home forgets it, and asking for it follows it then
  view.afterEdit();
  assert.equal(view.followedMarch('t'), undefined);
  const asked = view.marchOf('t');
  assert.ok(asked?.followed && asked.reached === 360);
  assert.deepEqual(view.followedMarch('t'), asked);
  // shown at an instant, an edit stands it back there: that needs the march, so it is taken
  assert.ok(view.showInstant('t', 90));
  view.afterEdit();
  assert.ok(view.followedMarch('t')?.followed);
  assert.deepEqual(view.instant, { name: 't', at: 90 });
  near(at(view, 'c'), rockerPin(40, 10, 35, 25, 90), 1e-8, 'stood back a quarter turn on');
});

test('a scrub of the V-twin moves its parts in the box and cuts none of them again', (t) => {
  assert.ok(VTWIN, 'vtwin/assembly.sv is packed');
  const view = opened(VTWIN);
  t.after(() => view.doc.dispose());
  const box = view.box3d;
  box.sync(view);
  const cut = box.stats.cut;
  assert.ok(cut >= 7, `every object cut once: ${cut}`);
  const moved = box.stats.moved;
  for (const deg of [30, 120, 250]) {
    assert.ok(view.showInstant('t', deg));
    box.sync(view);
  }
  assert.equal(box.stats.cut, cut, 'nothing cut again along the scrub');
  assert.ok(box.stats.moved - moved >= 3 * cut, `every object placed at every step: ${box.stats.moved}`);
});
