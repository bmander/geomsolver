/* **Hidden, as view state** (#435): what the model outline has taken out of the drawing.
 *
 * Hidden is the orbit's kind of state — never written to the source or the `.svd`, never saved,
 * solved or undone — and it is **carried across an edit by path**: the view keeps the rows hidden
 * by the names they are carried by (`Row.path`), and every drawing it is handed is told again
 * which of its entities and solids those rows made (`applyHidden`, from `swap`).  An anonymous row
 * is held by a key good for one elaboration only, as a selection of it is.  A load lets it all go.
 *
 * What hiding a row takes out is what that row *made*, its subtree: a point declared at the root
 * and read by a hidden component's line stays.  The core passes it by everywhere the workspace
 * draws, lights, picks or frames (`Sketch::set_hidden`); the painter here skips it too. */
import type { Row } from '../core/outline.js';
import { partOf } from '../core/outline.js';
import { fromWire, toWire } from '../core/model.js';
import type { Part, Primitive } from '../core/model.js';
import { boundsOfSeen } from '../core/workspace.js';
import type { SketchView } from './view.js';

/** The key a row is hidden by: its path, or for an anonymous row something that names it within
 *  this elaboration — its kind and the first thing it made. */
export function rowKey(row: Row): string {
  if (row.path) return row.path;
  const p = partOf(row);
  const first = p.ents[0] ? p.ents[0].join(':') : p.solids.length ? `solid:${p.solids[0]}` : '';
  return `@${row.kind}:${first}:${row.label}`;
}

/** An entity as a row names it: the core's kind word and its index, `line:3`. */
const wireOf = (p: Primitive): string => `${toWire(p.kind)}:${p.index}`;

/** Whether a key is an anonymous row's, which no edit carries. */
const anonymous = (key: string): boolean => key.startsWith('@');

/** Whether a row can be hidden at all: an object's operand is listed to be found, and has no eye
 *  in this version — nor has anything under one. */
export function hideable(row: Row): boolean {
  return !row.operand;
}

/** Each row with the rows above it, outermost first, in the order the tree is read. */
export function* walk(rows: readonly Row[], above: Row[] = []): Generator<[Row, Row[]]> {
  for (const r of rows) {
    yield [r, above];
    if (!r.operand) yield* walk(r.children, [...above, r]);
  }
}

/** Whether a row is hidden: it or a row above it is. */
export function isHidden(v: SketchView, row: Row,
                         above: readonly Row[] = ancestors(v, row)): boolean {
  return v.hiddenRows.has(rowKey(row)) || above.some((a) => v.hiddenRows.has(rowKey(a)));
}

/** Whether some row under a shown one is hidden — the half eye. */
export function partlyHidden(v: SketchView, row: Row): boolean {
  for (const [r] of walk(row.children)) if (v.hiddenRows.has(rowKey(r))) return true;
  return false;
}

/** The rows above `row`, outermost first. */
export function ancestors(v: SketchView, row: Row): Row[] {
  for (const [r, above] of walk(v.outline())) if (r === row) return above;
  return [];
}

/** **Tell the drawing what is hidden**: the entities and solids every hidden row made, set whole
 *  on the core and kept here for the painter; what is selected and now hidden is let go. */
export function applyHidden(v: SketchView): void {
  const part: Part = { ents: [], solids: [] };
  if (v.hiddenRows.size) {
    for (const [r] of walk(v.outline())) {
      if (hideable(r) && v.hiddenRows.has(rowKey(r))) partOf(r, part);
    }
  }
  v.sketch.setHidden(part);
  const ents = new Set<Primitive>();
  for (const [kind, index] of part.ents) {
    const p = v.doc.entityOf({ kind: fromWire(kind), index });
    if (p) ents.add(p);
  }
  v.hiddenEnts = ents;
  v.hiddenSolids = new Set(part.solids);
  v.hiddenKey = JSON.stringify(part);
  // **hiding something selected deselects it**
  if (v.selected.some((p) => ents.has(p))) v.selected = v.selected.filter((p) => !ents.has(p));
  if (v.selectedSolids.some((s) => v.hiddenSolids.has(s.index))) {
    v.pickSolids(v.selectedSolids.filter((s) => !v.hiddenSolids.has(s.index)));
  }
  if (v.hovered?.kind === 'entity' && ents.has(v.hovered.ent)) v.hovered = null;
  if (v.hovered?.kind === 'solid' && v.hiddenSolids.has(v.hovered.pick.index)) v.hovered = null;
}

/** A new drawing of the same document (an edit, an undo): hidden rows carried by path, the
 *  anonymous ones let go. */
export function carryHidden(v: SketchView): void {
  for (const k of [...v.hiddenRows]) if (anonymous(k)) v.hiddenRows.delete(k);
  applyHidden(v);
}

/** Hide a row, or show it.  Showing a row under a hidden one shows that one row: the rows above
 *  it are shown and each one's other children hidden in their stead. */
export function setRowHidden(v: SketchView, row: Row, hide: boolean): void {
  if (!hideable(row)) return;
  if (hide) {
    v.hiddenRows.add(rowKey(row));
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
  }
  applyHidden(v);
  v.onHidden();
  v.draw();
}

/** The eye on a row: hidden it shows, shown it hides. */
export function toggleRow(v: SketchView, row: Row): void {
  setRowHidden(v, row, !isHidden(v, row));
}

/** Show every row that made `p`: what choosing a hidden plane, or drawing on one, does. */
export function reveal(v: SketchView, p: Primitive): void {
  if (!v.hiddenEnts.has(p)) return;
  const at = rowOf(v, p);
  if (at) setRowHidden(v, at, false);
}

/** **Isolate** (`I`): hide everything but what is selected and the rows that made it.  Refused,
 *  saying why, with nothing selected. */
export function isolate(v: SketchView): boolean {
  const ents = new Set(v.selected.map(wireOf));
  const solids = new Set(v.selectedSolids.map((s) => s.index));
  if (!ents.size && !solids.size) {
    v.onStatus('select something to isolate');
    return false;
  }
  const picked = (r: Row): boolean => r.ents.some((e) => ents.has(e.join(':')))
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
  applyHidden(v);
  v.onHidden();
  v.onStatus('isolated the selection — ⇧I shows everything');
  v.draw();
  return true;
}

/** **Show all** (`⇧I`): nothing hidden. */
export function showAll(v: SketchView): void {
  v.hiddenRows.clear();
  applyHidden(v);
  v.onHidden();
  v.draw();
}

/** The deepest row that made `p` — an entity — or object `solid`: what a selection made in the
 *  workspace expands the tree to. */
export function rowOf(v: SketchView, p: Primitive | number): Row | null {
  const wire = typeof p === 'number' ? null : wireOf(p);
  for (const [r] of walk(v.outline())) {
    if (wire ? r.ents.some((e) => e.join(':') === wire) : r.solid === p) return r;
  }
  return null;
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

/** **Click selects a row**: an entity row's entity joins the drawing's selection, an object row
 *  the solids' — ⇧ adds.  A hidden row is shown first, since what is selected is drawn. */
export function selectRow(v: SketchView, row: Row, add = false): void {
  if (isHidden(v, row)) setRowHidden(v, row, false);
  if (row.solid !== undefined) {
    const pick = v.solidPick(row.solid);
    if (pick) v.pickSolid(pick, add);
  } else {
    const ents = row.ents.flatMap(([kind, index]) => {
      const p = v.doc.entityOf({ kind: fromWire(kind), index });
      return p ? [p] : [];
    });
    if (!ents.length) return;
    v.selected = add ? [...v.selected.filter((p) => !ents.includes(p)), ...ents] : ents;
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
  const [kind, index] = row.ents[0] ?? ['', -1];
  const ent = v.doc.entityOf({ kind: fromWire(kind), index });
  v.hoverHit(ent ? { kind: 'entity', ent } : null);
}

/** **Workspace to tree**: the row of what is selected in the workspace — the first entity, else
 *  the first object — with the rows above it, which the tree opens to show it. */
export function selectionRow(v: SketchView): { row: Row; above: Row[] } | null {
  const first: Primitive | number | undefined = v.selected[0] ?? v.selectedSolids[0]?.index;
  if (first === undefined) return null;
  const row = rowOf(v, first);
  return row ? { row, above: ancestors(v, row) } : null;
}

/** Whether a row stands for something selected — the rows the tree marks current. */
export function rowSelected(v: SketchView, row: Row): boolean {
  if (row.solid !== undefined) return v.selectedSolids.some((s) => s.index === row.solid);
  return row.ents.length > 0 && v.selected.some((p) => {
    const w = wireOf(p);
    return row.ents.some((e) => e.join(':') === w);
  });
}
