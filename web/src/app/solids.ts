/* The solid tools: Extrude and Revolve over the closed region a click is inside (#162 F2, F3),
 * the Booleans over the bodies picked in turn (F4), Fillet and Chamfer over a body's edges and
 * faces, and the datums a button drops or a face is sketched on (F5).  Each writes the source by
 * one edit the core computes — one splice, one undo step — and what it makes is then sized by its
 * arrow (`extent.ts`).  The view keeps the state; this is what a click with one of them does. */
import * as io from '../core/io.js';
import { Axis, Line, Plane } from '../core/model.js';
import type { Primitive } from '../core/model.js';
import type { Edit, Region } from '../core/program.js';
import { offsetHandle, roundSize, solidEdges } from '../core/workspace.js';
import {
  endExtent, extrudeHandleOf, offsetOf, solidIndex, tellExtent, turnHandleOf,
} from './extent.js';
import type { LiveExtent } from './extent.js';
import { bodyOf, combines, partOf, roundsEdges } from './view.js';
import type { Combining, EdgeWord, Rounding, SketchView, SolidPick } from './view.js';

/** What an edge tool has picked, said: `2 edges and 1 face` (`sep` between). */
function picked(r: Rounding, sep: string): string {
  const count = (n: number, thing: string) => (n ? `${n} ${thing}${n === 1 ? '' : 's'}` : '');
  return [count(r.edges.length, 'edge'), count(r.faces.length, 'face')].filter(Boolean).join(sep);
}

/** What a Boolean tool's status line asks for next, by its word: the target, then its tools. */
const COMBINE = {
  union: { target: 'the body to join others to', tools: 'join to it', done: 'joined' },
  cut: { target: 'the body to cut', tools: 'cut it with', done: 'cut' },
  common: { target: 'the body to intersect', tools: 'intersect it with', done: 'intersected' },
} as const;

/** The closed region of the drawing under a canvas point, on the plane being drawn on. */
export function regionAt(v: SketchView, sx: number, sy: number): Region | { refused: string } | null {
  // a region is bounded on a plane, and in space there is none being drawn on
  if (v.openSpace) return null;
  const [x, y] = v.s2wIn(v.toolView, sx, sy);
  return v.doc.regionAt(v.plane, x, y, v.unit);
}

/** **Extrude** the region under a canvas point (#162 F2): the face, the solid and — with a body
 *  selected — the body rule, written as one edit (one undo step): joined, or with `cut` cut
 *  from it, or with `through` cut through all of it.  The new solid is selected and, unless
 *  it goes through, opened for sizing (`liveExtent`). */
export function extrudeAt(v: SketchView, sx: number, sy: number, cut = false, through = false): void {
  const r = regionHere(v, sx, sy, 'extrude');
  const cutting = cut || through;
  const body = r && bodyFor(v, cutting);
  if (!r || body === false) return;
  const e = v.doc.extrude(r.outer, r.holes,
    body ? { word: cutting ? 'cut' : 'union', body, through } : undefined);
  madeSolid(v, e, 'extruded', (solid) => {
    const h = through ? null : extrudeHandleOf(v, solid);
    return h?.way ? { kind: 'extrusion', solid, way: h.way, d: h.thick } : null;
  });
}

/** **Revolve** (#162 F3): the region under a canvas point, then the line or axis to turn it
 *  about — the second click, or one already selected — written as one edit (one undo step):
 *  the face, the solid a whole turn about it and, with a body selected, the body rule (joined,
 *  or with `cut` cut from it).  The new solid is selected and opened for sizing its turn. */
export function revolveAt(v: SketchView, sx: number, sy: number, cut = false): void {
  const held = v.selected.length === 1 ? turnsAbout(v, v.selected[0]) : null;
  const region = v.pendingRevolve ?? regionHere(v, sx, sy, 'revolve');
  if (!region) return;
  const about = v.pendingRevolve ? turnsAbout(v, v.pick(sx, sy)) : held;
  if (!about && !v.pendingRevolve) {
    v.pendingRevolve = region;
    v.onStatus('now click the line or axis to turn it about — Esc to choose another region');
    v.draw();
    return;
  }
  if (!about) {
    v.onStatus('click a line in the region\'s plane, or an axis lying in it, to turn about');
    return;
  }
  const body = bodyFor(v, cut);
  if (body === false) return;
  const e = v.doc.revolve(region.outer, region.holes, about,
    body ? { word: cut ? 'cut' : 'union', body } : undefined);
  v.pendingRevolve = null;
  v.highlight = [];
  madeSolid(v, e, `revolved about ${about}`, (solid) => {
    const h = turnHandleOf(v, solid);
    return h ? { kind: 'turn', solid, d: h.turn } : null;
  });
}

/** **Union, Cut, Intersect** (#162 F4): a click with a Boolean tool down.  The first body
 *  clicked is the target, and each after it a tool — clicked again, let go; a click back on the
 *  target writes the Boolean (`finishCombine`, as Enter does).  **The selection is the
 *  gesture's state**, in the order it was clicked, so bodies selected before the tool was taken
 *  up are where it starts, and the box previews what it would write (`combining`). */
export function combineAt(v: SketchView, sx: number, sy: number): void {
  const pick = v.solidAt(sx, sy);
  if (!pick) {
    tellCombine(v);
    return;
  }
  combinePick(v, bodyOf(pick));
}

/** A body taken by a Boolean tool — clicked, or chosen from what was under a ⌘-click: the
 *  target, a tool, a tool let go, or the target again, which writes it. */
export function combinePick(v: SketchView, pick: SolidPick): void {
  const [target] = v.selectedSolids;
  if (target && pick.index === target.index) {
    finishCombine(v);
    return;
  }
  v.pickSolid(pick, !!target);
  tellCombine(v);
  v.onSelect();
  v.onChanged();
  v.draw();
}

/** The Boolean the selection would write while a Boolean tool is down — the target, the word
 *  and the tools — or null until there is a tool to join. */
export function combining(v: SketchView): Combining | null {
  const word = combines(v.tool);
  const [target, ...tools] = v.selectedSolids;
  return word && target && tools.length ? { target, word, tools } : null;
}

/** Write the Boolean: `tool word target` for every tool, one splice and one undo step, the
 *  target left selected for the next.  Refused, in the core's words, where the language would
 *  refuse it — a body made of itself (E041). */
export function finishCombine(v: SketchView): void {
  const c = combining(v);
  if (!c) {
    tellCombine(v);
    return;
  }
  const tools = c.tools.map((t) => t.name);
  const e = v.doc.addBodyWord(c.word, tools, c.target.name);
  if (!v.apply(e, `${c.target.name}: ${COMBINE[c.word].done} by ${tools.join(', ')}`)) return;
  // the tools are the target's now, and no objects of their own: only the target stays picked
  pickSolidNamed(v, c.target.name);
  v.onSelect();
  v.onChanged();
  v.draw();
}

/** The selection as a Boolean tool takes it: each object picked once, whole, in the order its
 *  first part was picked — a face of a body selected stands for the body. */
export function bodiesPicked(v: SketchView): void {
  const bodies: SolidPick[] = [];
  for (const s of v.selectedSolids) {
    if (!bodies.some((b) => b.index === s.index)) bodies.push(bodyOf(s));
  }
  const picked = v.selectedSolids;
  if (bodies.length < picked.length || picked.some((s) => partOf(s) !== 'body')) {
    v.pickSolids(bodies);
  }
}

/** Say what a Boolean tool wants next. */
export function tellCombine(v: SketchView): void {
  const word = combines(v.tool);
  if (!word) return;
  const [target, ...tools] = v.selectedSolids;
  v.onStatus(!target ? `click ${COMBINE[word].target}`
    : !tools.length ? `${target.name}: now click the bodies to ${COMBINE[word].tools} — Esc to `
                      + 'choose another'
    : `${tools.map((t) => t.name).join(', ')} ${word} ${target.name} — click ${target.name} `
      + 'again, or Enter, to write it');
}

/** **Fillet, Chamfer**: a click with an edge tool down.  An edge the eye sees under the pointer
 *  is picked, else the face of a solid there — whole, its every edge — and a second click on
 *  either lets it go.  **The picks are the selection's**, so what was selected before the tool was
 *  taken up is where it starts.  A pick on another object starts over on that one: the words are
 *  written over one body's faces.  The number is then asked beside the click (`onRound`), and
 *  Enter writes it (`finishRound`). */
export function roundAt(v: SketchView, sx: number, sy: number): void {
  const pick = roundTarget(v, sx, sy);
  if (!pick) {
    v.onStatus(`click an edge of a solid to ${roundsEdges(v.tool)} it, or a face for every `
               + 'edge of it');
    return;
  }
  v.roundClick = [sx, sy];
  roundPick(v, pick);
}

/** What a click with an edge tool down would pick under a canvas point: an edge the eye sees
 *  there, else the face of an object — or nothing. */
export function roundTarget(v: SketchView, sx: number, sy: number): SolidPick | null {
  const edge = v.edgeAt(sx, sy);
  if (edge) return edge;
  const face = v.solidAt(sx, sy);
  return face?.face ? face : null;
}

/** A part of an object taken by an edge tool — clicked, or chosen from what was under a
 *  ⌘-click: toggled in the selection, or on another object, the selection started over on it. */
export function roundPick(v: SketchView, pick: SolidPick): void {
  const other = v.selectedSolids.some((s) => s.index !== pick.index);
  v.pickSolid(pick, !other);
  v.onSelect();
  v.onChanged();
  tellRound(v);                   // last: its number takes the keyboard
  v.draw();
}

/** The selection as an edge tool takes it: the parts of the one object picked last. */
export function onePicked(v: SketchView): void {
  const last = v.selectedSolids.at(-1);
  if (last && v.selectedSolids.some((s) => s.index !== last.index)) {
    v.pickSolids(v.selectedSolids.filter((s) => s.index === last.index));
  }
}

/** What an edge tool would write over: the selection's faces and edges of one object (the one
 *  picked last), a body picked whole standing for its every face — or null for none. */
export function roundingOf(v: SketchView): Rounding | null {
  const last = v.selectedSolids.at(-1);
  if (!last) return null;
  const picks = v.selectedSolids.filter((s) => s.index === last.index);
  const faces: string[] = [];
  const add = (f: string) => {
    if (!faces.includes(f)) faces.push(f);
  };
  for (const p of picks) {
    if (partOf(p) === 'face') add(p.face);
    // a body whole: every face of it, which the core writes as the word over the body
    if (partOf(p) === 'body') for (const e of solidEdges(v.sketch, p.index)) e.faces.forEach(add);
  }
  const edges = picks.flatMap((p) => p.edge ? [p.edge] : []);
  return faces.length || edges.length
    ? { solid: last.index, name: last.name, faces, edges } : null;
}

/** **Write what the edge tool picked** at `size` (the radius, or the flat's leg — what was
 *  typed, else the size it was offered at): the library's word over the faces, one splice and
 *  one undo step, the tool left down for the next.  The core's words where it refuses, and null
 *  where it wrote. */
export function finishRound(v: SketchView, size?: string): string | null {
  const word = roundsEdges(v.tool);
  const r = v.rounding;
  if (!word || !r) {
    tellRound(v);
    return null;
  }
  const text = (size ?? '').trim() || roundOffer(v, word, r);
  const e = v.doc.addRounds(r.solid, word, r.faces, r.edges, text);
  if (e.refused) {
    v.onStatus(e.refused);
    return e.refused;
  }
  v.roundSize[word] = text;
  const done = word === 'round' ? 'rounded' : 'chamfered';
  if (!v.apply(e, `${r.name}: ${picked(r, ' and ')} ${done} at ${text}`)) {
    return null;
  }
  // what was picked is rounded away: the edges are the round's faces' now
  v.selected = [];
  endRound(v);
  v.onChanged();
  v.draw();
  return null;
}

/** The edge tool's number put away: its box closed, where it was asked forgotten.  What it was
 *  asked over stays selected — letting that go is the caller's (Escape, a write). */
export function endRound(v: SketchView): void {
  v.roundClick = null;
  v.onRound(null, null, '');
}

/** The size an edge tool offers on object `r`: the last one written or left, else a tenth of
 *  the object's least extent (the core's). */
function roundOffer(v: SketchView, word: EdgeWord, r: Rounding): string {
  if (v.roundSize[word]) return v.roundSize[word];
  const size = roundSize(v.sketch, r.solid);
  return size === null ? '1' : io.fmt(size, 6);
}

/** Say what an edge tool has picked and wants next, and put its number where it is asked. */
export function tellRound(v: SketchView): void {
  const word = roundsEdges(v.tool);
  if (!word) return;
  const r = v.rounding;
  if (!r) {
    v.onRound(null, null, '');
    v.onStatus(`click edges of a solid to ${word} them — a chain of them, or a face for `
               + 'every edge of it');
    return;
  }
  const size = word === 'round' ? 'the radius' : 'how far the flat runs along each face';
  v.onStatus(`${r.name}: ${picked(r, ', ')} — click more, then type ${size} and Enter `
             + '(Esc lets them go)');
  // beside the click that last picked, or — taken up over a selection — where the pointer is
  const at = v.roundClick ?? v.cursor;
  v.onRound(r, [at[0] + 12, at[1] + 12], roundOffer(v, word, r));
}

/** The name of what a revolution may turn about — a line or an axis, as the source calls it —
 *  or null for anything else. */
export function turnsAbout(v: SketchView, e: Primitive | null | undefined): string | null {
  return e instanceof Line || e instanceof Axis ? v.doc.nameOf(e) ?? null : null;
}

/** The closed region under a canvas point, or null having said why there is none to `verb`. */
function regionHere(v: SketchView, sx: number, sy: number, verb: string): Region | null {
  const r = regionAt(v, sx, sy);
  if (r && !('refused' in r)) return r;
  v.onStatus(r?.refused
    ?? `there is no closed region here to ${verb}: click inside a loop of edges`);
  return null;
}

/** The body a solid tool's new solid joins or cuts: the selected one (undefined for none), or
 *  false having said that a cut wants one. */
function bodyFor(v: SketchView, cut: boolean): string | undefined | false {
  const body = v.selectedSolids[0]?.name;
  if (cut && !body) {
    v.onStatus('select the body to cut first');
    return false;
  }
  return body;
}

/** A solid tool's edit applied (one undo step), the new solid selected and — where `live`
 *  says how — opened for sizing. */
function madeSolid(v: SketchView, e: Edit, what: string,
                   live: (solid: string) => LiveExtent | null): void {
  const solid = e.names[0];
  if (!v.apply(e, `${solid}: ${what}`)) return;
  endExtent(v);                     // the one before is done with: this one is sized now
  pickSolidNamed(v, solid);
  v.liveExtent = live(solid);
  tellExtent(v);
  v.onSelect();
}

/** Select a solid by the name the source gives it. */
function pickSolidNamed(v: SketchView, name: string): void {
  const index = solidIndex(v, name);
  if (index !== undefined) v.pickSolid({ name, index, face: '' });
}

/** **Drop a datum**: a plane or an axis, written at once and constrained to what is selected
 *  by the core's table (`edit::add_datum`): through a point, along a line, square to a plane —
 *  or, with nothing selected, free, set off the plane being drawn on.  It is constrained further
 *  like anything else; the new one is selected, and a plane so becomes the current one. */
export function addDatum(v: SketchView, kind: 'plane' | 'axis', name?: string): boolean {
  // a standard plane chosen before the document had it is brought in first, as a tool's press
  // brings it in, so the datum is set off the plane chosen
  if (!v.ensurePlane()) return false;
  const from = v.namesOf(v.selected);
  if (from.length < v.selected.length) {
    v.onStatus('something selected has no name in the source to constrain to: name it first');
    return false;
  }
  const current = v.plane ? v.doc.nameOf(v.plane) : undefined;
  const e = v.doc.addDatum(kind, from, current, name);
  if (!v.apply(e, `${kind} ${e.names[0]}`)) return false;
  const made = v.doc.entity(e.names[0]);
  if (made) v.selected = [made];
  // a plane set off another is sized at once, as an extrusion is: by its arrow or its number
  endExtent(v);
  const c = made instanceof Plane ? offsetOf(v, e.names[0]) : undefined;
  if (c) {
    const { az, el } = v.orbit;
    const d = offsetHandle(v.sketch, az, el, c.id)?.distance ?? 0;
    v.liveExtent = { kind: 'offset', plane: e.names[0], d };
    tellExtent(v);
  }
  v.onSelect();
  v.onChanged();
  v.draw();
  return true;
}

/** **Sketch on a face** (#162 F5): the plane already lying on it chosen, or a plane written
 *  lying on it — its origin where the click met the face, touching the face there, so it
 *  follows the solid — and chosen.  One edit, one undo step. */
export function sketchOnFace(v: SketchView, pick: SolidPick): boolean {
  if (!pick.face) {
    v.onStatus('click a face of the solid to sketch on it');
    return false;
  }
  const there = v.doc.facePlane(pick.index, pick.face);
  if (there) {
    v.choosePlane(there, false);
    return true;
  }
  const e = v.doc.sketchOnFace(pick.index, pick.face, pick.at);
  if (!v.apply(e, `sketching on ${pick.face}`)) return false;
  v.choosePlane(e.names[0], false);
  v.draw();
  return true;
}
