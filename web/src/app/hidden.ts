/* **Hidden, as view state** (#435): what the model outline has taken out of the drawing — and
 * beside it the kinds the workspace draws (#678) and the parts an assembly is closed on (#679).
 *
 * Hidden is the orbit's kind of state — never written to the source or the `.svd`, never saved,
 * solved or undone — and it is **carried across an edit by path**: the view keeps the rows hidden
 * by the names they are carried by (`Row.path`), and every drawing it is handed is told again
 * which of its entities and solids those rows made (`applyHidden`, from `swap`).  An anonymous row
 * is held by a key good for one elaboration only, as a selection of it is.  A load lets it all go.
 *
 * What hiding a row takes out is what that row *made*, its subtree: a point declared at the root
 * and read by a hidden component's line stays.  The core passes it by everywhere the workspace
 * draws, lights, picks or frames (`Sketch::set_hidden`); the painter here skips it too.
 *
 * **A kind** — objects, planes, axes, sketches, points, fixed marks, dimensions — is taken out
 * everywhere at once (View ▸ Show; `SketchView.kinds`), and the core told so as it is told what is
 * hidden (`Sketch::set_kinds`).  A tool shows the kinds it needs while it is down (`toolKinds`).
 *
 * **A part is closed** (#679): what an instance that makes an object draws to make it — its
 * planes, axes, sketches, points, fixed marks and callouts, at any depth — is out of the way until
 * the instance is *opened* (`SketchView.opened`), when the rest of the assembly is dimmed.  The
 * root's own drawing is drawn; a document with no such instance opens as it always did.
 *
 * A row shown by its eye, or chosen from the outline, is shown whatever closes it or its kind
 * (`SketchView.shownRows`). */
import type { Row } from '../core/outline.js';
import { partOf } from '../core/outline.js';
import { KIND, fromWire } from '../core/model.js';
import type { Part, Primitive, ShownKind } from '../core/model.js';
import type { Ref } from '../core/io.js';
import { boundsOfSeen } from '../core/workspace.js';
import { toggled } from './view.js';
import type { SketchView, Tool } from './view.js';

/** The key a row is hidden by: its path, or for an anonymous row something that names it within
 *  this elaboration — its kind and the first thing it, or the first row under it, made. */
export function rowKey(row: Row): string {
  if (row.path) return row.path;
  const own = row.ents[0] ? row.ents[0].join(':') : row.solid !== undefined ? `solid:${row.solid}`
    : row.children[0] ? rowKey(row.children[0]) : '';
  return `@${row.kind}:${own}:${row.label}`;
}

/** An entity as a row names it, `line:3` — `Primitive.ref` joined, so the two compare. */
const refKey = (r: Ref): string => r.join(':');

/** The proxy an entity a row names stands for, in this elaboration. */
export const entOf = (v: SketchView, [kind, index]: Ref): Primitive | undefined =>
  v.doc.entityOf({ kind: fromWire(kind), index });

/** Whether a key is an anonymous row's, which no edit carries. */
const anonymous = (key: string): boolean => key.startsWith('@');

/** Whether a row can be hidden at all: an object's operand is listed to be found, and has no eye
 *  in this version — nor has anything under one, which `walk` never reaches. */
export function hideable(row: Row): boolean {
  return !row.operand;
}

/** Whether a row, listed under `above`, has an eye: the operand rule, said once for the pane. */
export function hasEye(row: Row, above: readonly Row[]): boolean {
  return hideable(row) && above.every(hideable);
}

/** Each row with the rows above it, outermost first, in the order the tree is read. */
export function* walk(rows: readonly Row[], above: Row[] = []): Generator<[Row, Row[]]> {
  for (const r of rows) {
    yield [r, above];
    if (!r.operand) yield* walk(r.children, [...above, r]);
  }
}

/** Whether a row is an object where it was made: not an operand listed under one. */
const isObject = (r: Row): boolean => r.kind === 'solid' && !r.operand;

/** Whether an instance's row makes an object, at any depth — read once a row (the rows are the
 *  outline's, made again when it changes, so a row is never asked twice for different answers). */
const makes = new WeakMap<Row, boolean>();
function makesObject(r: Row): boolean {
  let m = makes.get(r);
  if (m === undefined) {
    m = r.children.some((c) => !c.operand && (isObject(c) || makesObject(c)));
    makes.set(r, m);
  }
  return m;
}

/** **Whether a row is a part** (#679): an instance that makes an object. */
export function isPart(r: Row): boolean {
  return r.kind === 'instance' && makesObject(r);
}

/** The part a row is inside and closed by, the innermost above it — or null where none is, it is
 *  the opened one, the parts are not closed, or the row is an object (never closed) or was shown
 *  by its eye, or under a row that was. */
export function closingPart(v: SketchView, row: Row, above: readonly Row[]): Row | null {
  if (!v.closeParts || isObject(row) || v.shownRows.has(rowKey(row))) return null;
  let part: Row | null = null;
  for (const a of above) {
    if (v.shownRows.has(rowKey(a))) return null;
    if (isPart(a)) part = a;
  }
  return part && rowKey(part) !== v.opened ? part : null;
}

/** Whether a row is hidden: it or a row above it is, or a closed part holds it (#679). */
export function isHidden(v: SketchView, row: Row,
                         above: readonly Row[] = ancestors(v, row)): boolean {
  return v.hiddenRows.has(rowKey(row)) || above.some((a) => v.hiddenRows.has(rowKey(a)))
    || closingPart(v, row, above) !== null;
}

/** Whether some row under a shown one is hidden — the half eye: a closed part's insides too. */
export function partlyHidden(v: SketchView, row: Row): boolean {
  if (v.closeParts && isPart(row) && rowKey(row) !== v.opened && !v.shownRows.has(rowKey(row))
      && row.children.some((c) => !c.operand && !isObject(c))) {
    return true;
  }
  if (!v.hiddenRows.size) return false;
  for (const [r] of walk(row.children)) if (v.hiddenRows.has(rowKey(r))) return true;
  return false;
}

/** The rows above `row`, outermost first. */
export function ancestors(v: SketchView, row: Row): Row[] {
  return rowAt(v, (r) => r === row)?.above ?? [];
}

/** The first row, in tree order, that `is` takes, with the rows above it. */
function rowAt(v: SketchView, is: (r: Row) => boolean): { row: Row; above: Row[] } | null {
  for (const [row, above] of walk(v.outline())) if (is(row)) return { row, above };
  return null;
}

/** What hidden changed to has to be told: the core and the painter, then the pane and the page. */
function changed(v: SketchView): void {
  applyHidden(v);
  v.onHidden();
  v.draw();
}

/** **The kinds a tool needs drawn to be used** (#678), shown while it is down whatever View ▸
 *  Show says: a drawing tool snaps to the points and figures drawn, Extrude takes a region of
 *  them and Revolve a line or an axis to turn it about, and a Boolean or an edge tool takes
 *  objects. */
export function toolKinds(tool: Tool): number {
  switch (tool) {
    case 'select': return 0;
    case 'extrude': return KIND.sketches;
    case 'revolve': return KIND.sketches | KIND.axes;
    case 'union': case 'cut': case 'common': case 'fillet': case 'chamfer': return KIND.objects;
    default: return KIND.sketches | KIND.points;
  }
}

/** The kinds the workspace draws now: View ▸ Show's, and the tool's. */
export function shownKinds(v: SketchView): number {
  return v.kinds | toolKinds(v.tool);
}

/** Whether a kind is drawn now (`shownKinds`). */
export function shows(v: SketchView, kind: ShownKind): boolean {
  return (shownKinds(v) & KIND[kind]) !== 0;
}

/** **View ▸ Show**: a kind drawn or not (#678).  What was shown whatever its kind is let go with
 *  the change, so a kind turned off is off. */
export function setKind(v: SketchView, kind: ShownKind, on: boolean): void {
  const kinds = on ? v.kinds | KIND[kind] : v.kinds & ~KIND[kind];
  if (kinds === v.kinds) return;
  v.kinds = kinds;
  v.shownRows.clear();
  changed(v);
}

/** **What the closed parts draw** (#679): every entity a row inside a part that is not opened
 *  made, its objects aside — into `into`, which the core is told is hidden.  Whether the drawing
 *  has a part at all: an assembly, shown on its parts. */
function closedPart(v: SketchView, rows: readonly Row[], into: Part): boolean {
  if (!v.closeParts) return false;
  let any = false;
  const down = (rs: readonly Row[], closed: boolean): void => {
    for (const r of rs) {
      if (r.operand) continue;
      any ||= isPart(r);
      if (v.shownRows.has(rowKey(r))) continue;
      if (closed && !isObject(r)) into.ents.push(...r.ents);
      down(r.children, isPart(r) ? rowKey(r) !== v.opened : closed);
    }
  };
  down(rows, false);
  return any;
}

/** **Tell the drawing what is hidden**: the entities and solids every hidden row made and what the
 *  closed parts draw, set whole on the core, the kinds shown beside it; what the core then passes
 *  by is kept here for the painter, and what is selected and now hidden is let go. */
export function applyHidden(v: SketchView): void {
  const kinds = shownKinds(v);
  const rows = v.hiddenRows.size || v.shownRows.size || v.closeParts || v.opened ? v.outline() : [];
  const part: Part = { ents: [], solids: [] };
  if (v.hiddenRows.size) {
    for (const [r] of walk(rows)) {
      if (hideable(r) && v.hiddenRows.has(rowKey(r))) partOf(r, part);
    }
  }
  const onParts = closedPart(v, rows, part);
  v.sketch.setHidden(part);
  const anyway: Part = { ents: [], solids: [] };
  let opened: Row | null = null;
  for (const [r] of walk(rows)) {
    if (v.shownRows.has(rowKey(r))) partOf(r, anyway);
    if (v.opened !== null && rowKey(r) === v.opened && isPart(r)) opened = r;
  }
  v.sketch.setShownKinds(kinds, anyway, onParts);
  v.onParts = onParts;
  // an opened part is one the outline still lists, and the rest of the assembly dims round it
  if (!opened) v.opened = null;
  const inside = opened ? partOf(opened) : null;
  v.openedEnts = inside ? new Set(inside.ents.flatMap((r) => entOf(v, r) ?? [])) : null;
  v.openedSolids = inside ? new Set(inside.solids) : null;
  // what the core passes by, hidden or of a kind not shown
  const unseen = v.sketch.unseen();
  const ents = new Set<Primitive>();
  for (const r of unseen.ents) {
    const p = entOf(v, r);
    if (p) ents.add(p);
  }
  v.hiddenEnts = ents;
  v.hiddenSolids = new Set(unseen.solids);
  v.hiddenEpoch++;
  // **hiding something selected deselects it**
  if (v.selected.some((p) => ents.has(p))) v.selected = v.selected.filter((p) => !ents.has(p));
  if (v.selectedSolids.some((s) => v.hiddenSolids.has(s.index))) {
    v.pickSolids(v.selectedSolids.filter((s) => !v.hiddenSolids.has(s.index)));
  }
  if (v.hovered?.kind === 'entity' && ents.has(v.hovered.ent)) v.hovered = null;
  if (v.hovered?.kind === 'solid' && v.hiddenSolids.has(v.hovered.pick.index)) v.hovered = null;
}

/** A new drawing of the same document (an edit, an undo): hidden rows, rows shown and the part
 *  opened carried by path, the anonymous ones let go. */
export function carryHidden(v: SketchView): void {
  for (const k of [...v.hiddenRows]) if (anonymous(k)) v.hiddenRows.delete(k);
  for (const k of [...v.shownRows]) if (anonymous(k)) v.shownRows.delete(k);
  if (v.opened !== null && anonymous(v.opened)) v.opened = null;
  applyHidden(v);
}

/** Hide a row, or show it.  Showing a row under a hidden one shows that one row: the rows above
 *  it are shown and each one's other children hidden in their stead.  Showing a row a closed part
 *  holds shows it, whatever closes it (#679). */
export function setRowHidden(v: SketchView, row: Row, hide: boolean): void {
  if (!hideable(row)) return;
  if (hide) {
    v.hiddenRows.add(rowKey(row));
    v.shownRows.delete(rowKey(row));
  } else {
    const above = ancestors(v, row);
    const top = above.findIndex((a) => v.hiddenRows.has(rowKey(a)));
    if (top >= 0) {
      const path = [...above.slice(top), row];
      for (let k = 0; k + 1 < path.length; k++) {
        v.hiddenRows.delete(rowKey(path[k]));
        for (const c of path[k].children) {
          if (c !== path[k + 1] && hideable(c)) v.hiddenRows.add(rowKey(c));
        }
      }
    }
    v.hiddenRows.delete(rowKey(row));
    if (closingPart(v, row, above)) v.shownRows.add(rowKey(row));
  }
  changed(v);
}

/** The eye on a row: hidden it shows, shown it hides. */
export function toggleRow(v: SketchView, row: Row): void {
  setRowHidden(v, row, !isHidden(v, row));
}

/** **Bring a row out** where it is not seen: the part holding it opened, the row shown if the
 *  outline hid it, and — still passed by, its kind not shown — shown whatever its kind. */
function bringOut(v: SketchView, row: Row, above: readonly Row[]): void {
  const part = closingPart(v, row, above);
  if (part) v.opened = rowKey(part);
  if (isHidden(v, row, above)) setRowHidden(v, row, false);
  else if (part) changed(v);
  const ent = row.ents[0] && entOf(v, row.ents[0]);
  const solid = row.solid;
  if ((ent && v.hiddenEnts.has(ent)) || (solid !== undefined && v.hiddenSolids.has(solid))) {
    v.shownRows.add(rowKey(row));
    changed(v);
  }
}

/** **Hide what a right-click is on** (`p`, an entity, or object `solid`): the row that made it,
 *  and so all that row made.  The row hidden, or null where none can be — nothing made it that the
 *  tree lists with an eye. */
export function hide(v: SketchView, p: Primitive | number): Row | null {
  const at = hideableRow(v, p);
  if (!at) return null;
  setRowHidden(v, at, true);
  v.onStatus(`hid ${at.label} — ⇧I shows everything`);
  return at;
}

/** The row `hide` would hide, or null. */
export function hideableRow(v: SketchView, p: Primitive | number): Row | null {
  const at = found(v, p);
  return at && hasEye(at.row, at.above) ? at.row : null;
}

/** Show every row that made `p`, or the part holding it, or `p` whatever its kind: what choosing
 *  a plane not drawn, or drawing on one, does. */
export function reveal(v: SketchView, p: Primitive): void {
  if (!v.hiddenEnts.has(p)) return;
  const at = found(v, p);
  if (at) bringOut(v, at.row, at.above);
}

/** **Isolate** (`I`): hide everything but what is selected and the rows that made it.  Refused,
 *  saying why, with nothing selected. */
export function isolate(v: SketchView): boolean {
  const ents = new Set(v.selected.map((p) => refKey(p.ref)));
  const solids = new Set(v.selectedSolids.map((s) => s.index));
  if (!ents.size && !solids.size) {
    v.onStatus('select something to isolate');
    return false;
  }
  const picked = (r: Row): boolean => r.ents.some((e) => ents.has(refKey(e)))
    || (r.solid !== undefined && solids.has(r.solid));
  const holds = (r: Row): boolean => picked(r) || r.children.some((c) => !c.operand && holds(c));
  v.hiddenRows.clear();
  const down = (rows: readonly Row[]): void => {
    for (const r of rows) {
      if (!hideable(r) || picked(r)) continue;
      if (holds(r)) down(r.children);
      else v.hiddenRows.add(rowKey(r));
    }
  };
  down(v.outline());
  changed(v);
  v.onStatus('isolated the selection — ⇧I shows everything');
  return true;
}

/** **Show all** (`⇧I`): nothing the outline hid.  The kinds View ▸ Show turned off, and the parts
 *  closed, stay as they are: they are said where they are set. */
export function showAll(v: SketchView): void {
  v.hiddenRows.clear();
  changed(v);
}

/** **Open a part** (#679): its insides drawn and the rest of the assembly dimmed — or, `null`,
 *  close the one open.  Whether anything changed. */
export function openPart(v: SketchView, part: Row | null): boolean {
  const key = part && isPart(part) ? rowKey(part) : null;
  if (key === v.opened) return false;
  v.opened = key;
  changed(v);
  if (part && key) v.onStatus(`opened ${part.label} — Esc closes it`);
  return true;
}

/** **Close the part open**, if one is: Esc, or a click on nothing.  Whether one was. */
export function closePart(v: SketchView): boolean {
  return openPart(v, null);
}

/** The part a double-click on `p` (an entity, or object `solid`) opens: the innermost part above
 *  the row that made it, not the one already open — or null. */
export function partToOpen(v: SketchView, p: Primitive | number): Row | null {
  const at = found(v, p);
  if (!at || !v.closeParts) return null;
  const parts = [...at.above, at.row].filter(isPart);
  const part = parts.at(-1) ?? null;
  return part && rowKey(part) !== v.opened ? part : null;
}

/** The part open, as its row, or null. */
export function openedRow(v: SketchView): Row | null {
  return v.opened === null ? null : rowAt(v, (r) => rowKey(r) === v.opened)?.row ?? null;
}

/** **Whether the parts are closed** (#679): View ▸ Show ▸ Inside parts, turned off, closes them. */
export function setCloseParts(v: SketchView, on: boolean): void {
  if (v.closeParts === on) return;
  v.closeParts = on;
  v.shownRows.clear();
  changed(v);
}

/** The row that made `p` — an entity — or object `solid`: what a selection made in the
 *  workspace expands the tree to. */
export function rowOf(v: SketchView, p: Primitive | number): Row | null {
  return found(v, p)?.row ?? null;
}

/** `rowOf`'s row, with the rows above it. */
function found(v: SketchView, p: Primitive | number): { row: Row; above: Row[] } | null {
  const key = typeof p === 'number' ? null : refKey(p.ref);
  return rowAt(v, (r) => (key ? r.ents.some((e) => refKey(e) === key) : r.solid === p));
}

/** **Double-click frames a row**: its subtree, as the eye now sees it (`workspace::bounds_of`). */
export function frameRow(v: SketchView, row: Row): boolean {
  const { az, el } = v.orbit;
  const b = boundsOfSeen(v.sketch, v.unit, az, el, partOf(row));
  if (!b) return false;
  v.cam.fitTo(b, v.width, v.height);
  v.draw();
  return true;
}

/** **Click selects a row**: an entity row's entity the drawing's selection, an object row the
 *  solids' — ⇧ toggles it in, as in the workspace.  A row not seen is brought out first, since
 *  what is selected is drawn: a member of a closed part opens the part, a hidden row is shown, and
 *  one of a kind not shown is shown whatever its kind. */
export function selectRow(v: SketchView, row: Row, add = false): void {
  bringOut(v, row, ancestors(v, row));
  if (row.solid !== undefined) {
    const pick = v.solidPick(row.solid);
    if (pick) v.pickSolid(pick, add);
  } else {
    const ent = row.ents[0] && entOf(v, row.ents[0]);
    if (!ent) return;
    v.selected = toggled(v.selected, ent, add);
  }
  v.onSelect();
  v.draw();
}

/** **Hover lights a row's item**, as a row of the ⌘-click list does, or nothing. */
export function hoverRow(v: SketchView, row: Row | null): void {
  if (!row) return v.hoverHit(null);
  if (row.solid !== undefined) {
    const pick = v.solidPick(row.solid);
    return v.hoverHit(pick ? { kind: 'solid', pick, hidden: false } : null);
  }
  const ent = row.ents[0] && entOf(v, row.ents[0]);
  v.hoverHit(ent ? { kind: 'entity', ent } : null);
}

/** **Workspace to tree**: the row of what is selected in the workspace — the first entity, else
 *  the first object — with the rows above it, which the tree opens to show it. */
export function selectionRow(v: SketchView): { row: Row; above: Row[] } | null {
  const first: Primitive | number | undefined = v.selected[0] ?? v.selectedSolids[0]?.index;
  return first === undefined ? null : found(v, first);
}

/** What is selected, as rows name it — read once, then asked of every row (`rowSelected`). */
export function selectedKeys(v: SketchView): Set<string> {
  return new Set([...v.selected.map((p) => refKey(p.ref)),
                  ...v.selectedSolids.map((s) => `solid:${s.index}`)]);
}

/** Whether a row stands for something selected — the rows the tree marks current. */
export function rowSelected(row: Row, selected: ReadonlySet<string>): boolean {
  return row.solid !== undefined ? selected.has(`solid:${row.solid}`)
    : row.ents.some((e) => selected.has(refKey(e)));
}
