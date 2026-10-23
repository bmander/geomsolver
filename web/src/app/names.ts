/* **The drawing and the text are one thing, read two ways**: what is selected in the workspace is
 * lit wherever the text names it, and a name clicked in the text selects what it names there.
 *
 * Both directions are one join between two of the core's answers, made here and nowhere else:
 * the outline's rows (`Document.outline`) — each thing the document made, the path it is called
 * by and the statement it was written in — and the text's names (`core/program.ts::symbols`) —
 * every word that names something and the declaration it names.  A row is the symbol given by
 * its statement under the last part of its path (`mid` of `t0.mid`, given by `mid := point` in
 * `Rung`'s body); a module's datum, written by no statement of this text, is the member read by
 * its whole path (`std.front`).  The places that symbol is named are the row's **where they are
 * written as a part of its path**: `t0.mid` and the bare `mid` inside `Rung` name `t0`'s, and
 * `t1.mid` does not.  Read the other way, a name stands for every row it is so a part of — the
 * `mid` `Rung` declares is each instance's.
 *
 * The join reads the text's dots and nothing else of Solvent: which word is a name, which
 * declaration it names and what each row is are the core's.  It belongs there too, once the
 * index says what each member is read through and a row where its name is given. */
import type { Mark } from './editor.js';
import { markOf } from './occurrences.js';
import { partition } from '../core/program.js';
import type { Occurrence, Symbols } from '../core/program.js';
import type { Row } from '../core/outline.js';
import { partOf } from '../core/outline.js';
import type { Primitive } from '../core/model.js';
import { entOf, isHidden, selectedKeys, setRowHidden, walk } from './hidden.js';
import { samePick } from './view.js';
import type { SketchView, SolidPick } from './view.js';

/** Whether `part` is the end of `path` by whole names: `mid` and `t0.mid` of `t0.mid`, not `0.mid`
 *  nor `t1.mid`. */
function ends(path: string, part: string): boolean {
  return path === part || path.endsWith(`.${part}`);
}

/** A symbol's whole path, through what each member is read of: `std.front`. */
function pathName(index: Symbols, k: number): string {
  const s = index.symbols[k];
  return s.parent < 0 ? s.name : `${pathName(index, s.parent)}.${s.name}`;
}

/** The key a selection holds a row's entity or body by (`hidden.selectedKeys`). */
function keysOf(row: Row): string[] {
  return row.solid !== undefined ? [`solid:${row.solid}`] : row.ents.map((e) => e.join(':'));
}

/** The outline and the names of one text, joined (module header) — built once per text and
 *  outline, then asked of each selection and each caret. */
export class Names {
  /** The symbol each named row is. */
  private symbol = new Map<Row, number>();
  /** The rows each symbol is. */
  private rows = new Map<number, Row[]>();
  /** Where each of those symbols is named, in text order. */
  private places = new Map<number, Occurrence[]>();
  /** The named rows by the keys a selection holds them by. */
  private byKey = new Map<string, Row[]>();

  constructor(readonly text: string, readonly index: Symbols, readonly outline: readonly Row[]) {
    const defs = index.occurrences.filter((o) => o.def);
    for (const [row] of walk(outline)) {
      // an anonymous row is called by nothing the text could name
      if (!row.path) continue;
      const s = this.symbolOf(row, defs);
      if (s === null) continue;
      this.symbol.set(row, s);
      push(this.rows, s, row);
      for (const key of keysOf(row)) push(this.byKey, key, row);
    }
    for (const o of index.occurrences) if (this.rows.has(o.symbol)) push(this.places, o.symbol, o);
  }

  /** The symbol a row is (module header), or null where the text names it nowhere. */
  private symbolOf(row: Row, defs: readonly Occurrence[]): number | null {
    const last = row.path.slice(row.path.lastIndexOf('.') + 1);
    const { text, index } = this;
    if (!row.span) {
      // a module's: written nowhere here, read by its path — `std.front` is `std`'s member `front`
      const k = index.symbols.findIndex((s, k) =>
        s.name === last && pathName(index, k) === row.path);
      return k < 0 ? null : k;
    }
    const { lo, hi } = row.span;
    for (let k = partition(defs.length, (k) => defs[k].lo < lo); defs[k]?.lo < hi; k += 1) {
      const o = defs[k];
      if (o.hi <= hi && text.slice(o.lo, o.hi) === last) return o.symbol;
    }
    return null;
  }

  /** The path an occurrence is written as: the names before it joined by their dots, `t0.mid` for
   *  the `mid` of `t0.mid` — null where it is read off something with no name (`Rung(…).mid`). */
  private pathOf(o: Occurrence): string | null {
    const { text, index } = this;
    let path = text.slice(o.lo, o.hi);
    for (let at = o; text[at.lo - 1] === '.';) {
      const before = index.at(at.lo - 1);
      if (!before || before.hi !== at.lo - 1) return null;
      path = `${text.slice(before.lo, before.hi)}.${path}`;
      at = before;
    }
    return path;
  }

  /** The rows the name at `o` stands for: every one whose symbol it is, written as a part of the
   *  row's path. */
  rowsAt(o: Occurrence): Row[] {
    const path = this.pathOf(o);
    if (path === null) return [];
    return (this.rows.get(o.symbol) ?? []).filter((r) => ends(r.path, path));
  }

  /** Every place a row held by one of `keys` is named, marked as the name under the caret is. */
  marksOf(keys: Iterable<string>): Mark[] {
    const lit = new Map<number, Mark>();
    for (const key of keys) {
      for (const row of this.byKey.get(key) ?? []) {
        for (const o of this.places.get(this.symbol.get(row)!) ?? []) {
          const path = this.pathOf(o);
          if (path !== null && ends(row.path, path)) lit.set(o.lo, markOf(o));
        }
      }
    }
    return [...lit.values()].sort((a, b) => a.lo - b.lo);
  }
}

function push<K, V>(m: Map<K, V[]>, k: K, v: V): void {
  const at = m.get(k);
  if (at) at.push(v);
  else m.set(k, [v]);
}

/** **Workspace to text**: every place the text names what is selected — an entity, a body. */
export function selectionMarks(v: SketchView, names: Names): Mark[] {
  if (!v.selected.length && !v.selectedSolids.length) return [];
  return names.marksOf(selectedKeys(v));
}

/** What selecting rows selects: an entity row's entities, an instance's every member, a solid
 *  row's body — entities where any are named, else bodies, as the workspace holds one kind of
 *  selection at a time — and the rows that stand for them.  A module's datums are no one thing. */
function picked(v: SketchView, rows: readonly Row[]):
    { rows: Row[]; ents: Primitive[]; solids: SolidPick[] } {
  const kept = rows.filter((r) => r.kind !== 'datums');
  const ents = new Set<Primitive>(), solids: SolidPick[] = [];
  for (const r of kept) {
    const part = r.kind === 'instance' ? partOf(r)
      : { ents: r.ents, solids: r.solid === undefined ? [] : [r.solid] };
    for (const ref of part.ents) {
      const p = entOf(v, ref);
      if (p) ents.add(p);
    }
    for (const s of part.solids) {
      const pick = v.solidPick(s);
      if (pick && !solids.some((x) => samePick(x, pick))) solids.push(pick);
    }
  }
  return { rows: kept, ents: [...ents], solids };
}

/** **Text to workspace**: select what `rows` stand for, showing a hidden one first, since what is
 *  selected is drawn — as a click on its row in the outline does.  Whether anything changed:
 *  rows that stand for nothing to select, or for what is selected already, leave it alone. */
export function selectRows(v: SketchView, rows: readonly Row[]): boolean {
  const { rows: kept, ents, solids } = picked(v, rows);
  if (!ents.length && !solids.length) return false;
  const unchanged = ents.length
    ? !v.selectedSolids.length && v.selected.length === ents.length
      && ents.every((p) => v.selected.includes(p))
    : !v.selected.length && v.selectedSolids.length === solids.length
      && solids.every((p) => v.selectedSolids.some((x) => samePick(x, p)));
  if (unchanged) return false;
  for (const r of kept) if (isHidden(v, r)) setRowHidden(v, r, false);
  if (ents.length) v.selected = ents;
  else v.pickSolids(solids);
  return true;
}
