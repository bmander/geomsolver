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
 * declaration it names and what each row is are the core's. */
import type { Mark } from './editor.js';
import type { Occurrence, Symbols } from '../core/program.js';
import type { Row } from '../core/outline.js';
import { partOf } from '../core/outline.js';
import { fromWire } from '../core/model.js';
import type { Primitive } from '../core/model.js';
import { isHidden, rowSelected, selectedKeys, setRowHidden, walk } from './hidden.js';
import { samePick } from './view.js';
import type { SketchView, SolidPick } from './view.js';

/** Whether `part` is the end of `path` by whole names: `mid` and `t0.mid` of `t0.mid`, not `0.mid`
 *  nor `t1.mid`. */
function ends(path: string, part: string): boolean {
  return path === part || path.endsWith(`.${part}`);
}

/** The first of `os` (in text order) starting at or after `at`. */
function from(os: readonly Occurrence[], at: number): number {
  let lo = 0, hi = os.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (os[mid].lo < at) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}

/** The outline and the names of one text, joined (module header). */
export class Names {
  /** The rows each symbol is, by symbol. */
  private rows = new Map<number, Row[]>();
  /** The path each occurrence is written as, by its start, once asked. */
  private paths = new Map<number, string | null>();

  constructor(readonly text: string, readonly index: Symbols, readonly outline: readonly Row[]) {
    const defs = index.occurrences.filter((o) => o.def);
    for (const [row] of walk(outline)) {
      // an anonymous row is called by nothing the text could name
      if (!row.path) continue;
      const s = this.symbolOf(row, defs);
      if (s === null) continue;
      const of = this.rows.get(s);
      if (of) of.push(row);
      else this.rows.set(s, [row]);
    }
  }

  /** The symbol a row is (module header), or null where the text names it nowhere. */
  private symbolOf(row: Row, defs: readonly Occurrence[]): number | null {
    const last = row.path.slice(row.path.lastIndexOf('.') + 1);
    const { text, index } = this;
    if (row.span) {
      for (let k = from(defs, row.span.lo); k < defs.length && defs[k].lo < row.span.hi; k += 1) {
        const o = defs[k];
        if (o.hi <= row.span.hi && text.slice(o.lo, o.hi) === last) return o.symbol;
      }
      return null;
    }
    // a module's: written nowhere here, read by its path — `std.front` is `std`'s member `front`
    const named = (k: number): string => {
      const s = index.symbols[k];
      return s.parent < 0 ? s.name : `${named(s.parent)}.${s.name}`;
    };
    const k = index.symbols.findIndex((s, k) => s.name === last && named(k) === row.path);
    return k < 0 ? null : k;
  }

  /** The path an occurrence is written as: the names before it joined by their dots, `t0.mid` for
   *  the `mid` of `t0.mid` — null where it is read off something with no name (`Rung(…).mid`). */
  pathOf(o: Occurrence): string | null {
    const known = this.paths.get(o.lo);
    if (known !== undefined) return known;
    const { text, index } = this;
    let path: string | null = text.slice(o.lo, o.hi);
    for (let at = o; text[at.lo - 1] === '.';) {
      const before = index.at(at.lo - 1);
      if (!before || before.hi !== at.lo - 1) { path = null; break; }
      path = `${text.slice(before.lo, before.hi)}.${path}`;
      at = before;
    }
    this.paths.set(o.lo, path);
    return path;
  }

  /** The rows the name at `o` stands for: every one whose symbol it is, written as a part of the
   *  row's path. */
  rowsAt(o: Occurrence): Row[] {
    const path = this.pathOf(o);
    if (path === null) return [];
    return (this.rows.get(o.symbol) ?? []).filter((r) => ends(r.path, path));
  }

  /** Every place a row is named — `occurrence-def` where it is given, `occurrence` where it is
   *  read — as the name under the caret is lit (`occurrences.ts`). */
  marksOf(rows: readonly Row[]): Mark[] {
    const lit = new Map<number, Mark>();
    for (const [s, of] of this.rows) {
      const these = of.filter((r) => rows.includes(r));
      if (!these.length) continue;
      for (const o of this.index.of(s)) {
        const path = this.pathOf(o);
        if (path !== null && these.some((r) => ends(r.path, path))) {
          lit.set(o.lo, { lo: o.lo, hi: o.hi, cls: o.def ? 'occurrence-def' : 'occurrence' });
        }
      }
    }
    return [...lit.values()].sort((a, b) => a.lo - b.lo);
  }

  /** Every row that is named here at all. */
  get named(): Row[] {
    return [...this.rows.values()].flat();
  }
}

/** **Workspace to text**: every place the text names what is selected — an entity, a body. */
export function selectionMarks(v: SketchView, names: Names): Mark[] {
  if (!v.selected.length && !v.selectedSolids.length) return [];
  const keys = selectedKeys(v);
  return names.marksOf(names.named.filter((r) => rowSelected(r, keys)));
}

/** What selecting rows selects: an entity row's entities, an instance's every member, a solid
 *  row's body — entities where any are named, else bodies, as the workspace holds one kind of
 *  selection at a time.  A module's datums are no one thing to select. */
function picked(v: SketchView, rows: readonly Row[]): { ents: Primitive[]; solids: SolidPick[] } {
  const ents: Primitive[] = [], solids: SolidPick[] = [];
  for (const r of rows) {
    if (r.kind === 'datums') continue;
    const part = r.kind === 'instance' ? partOf(r)
      : { ents: r.ents, solids: r.solid === undefined ? [] : [r.solid] };
    for (const [kind, index] of part.ents) {
      const p = v.doc.entityOf({ kind: fromWire(kind), index });
      if (p && !ents.includes(p)) ents.push(p);
    }
    for (const s of part.solids) {
      const pick = v.solidPick(s);
      if (pick && !solids.some((x) => samePick(x, pick))) solids.push(pick);
    }
  }
  return { ents, solids };
}

/** **Text to workspace**: select what `rows` stand for, showing a hidden one first, since what is
 *  selected is drawn — as a click on its row in the outline does.  Whether anything changed:
 *  rows that stand for nothing to select, or for what is selected already, leave it alone. */
export function selectRows(v: SketchView, rows: readonly Row[]): boolean {
  const { ents, solids } = picked(v, rows);
  const same = <T>(a: readonly T[], b: readonly T[], eq: (x: T, y: T) => boolean) =>
    a.length === b.length && a.every((x) => b.some((y) => eq(x, y)));
  if (ents.length) {
    if (same(v.selected, ents, (a, b) => a === b) && !v.selectedSolids.length) return false;
  } else if (solids.length) {
    if (same(v.selectedSolids, solids, samePick) && !v.selected.length) return false;
  } else return false;
  for (const r of rows) if (r.kind !== 'datums' && isHidden(v, r)) setRowHidden(v, r, false);
  if (ents.length) v.selected = ents;
  else v.pickSolids(solids);
  return true;
}
