/* A dimension's number, edited where it is drawn.  One input, moved to whichever callout is
 * being written and filled in by the view, so the figure, where it sits, which of the three it
 * is and what it says are one gesture — and every part of it is on the drawing while it
 * happens, rather than behind a box in the middle of the screen. */
import * as io from '../core/io.js';
import { Constraint } from '../core/constraints.js';
import { editDimension } from './commands.js';
import { invalidateRows } from './lists.js';
import { view } from './shell.js';
import { toast } from './ui.js';
import type { LiveDim, LiveExtent } from './view.js';

/** A dimension's number as a person reads and writes it: what it was written as if it was
 *  written (`w` for a `param`, which is also what its callout says), else what the drawing says
 *  it is — the same rounding, so the editor that sits on a callout shows the callout's own
 *  number rather than a longer one that means the same.  In
 *  degrees for an angle, which is the unit the core takes text in either way.
 *
 *  Nothing is written back unless somebody types, so the rounding here costs no precision: a
 *  dimension nobody edits keeps the measurement it was stated at, to the last figure. */
function dimensionField(c: Constraint): { attr: string; text: string } | null {
  const [d] = c.dimensions();
  if (!d) return null;
  const [attr, kind] = d;
  const v = (c as unknown as Record<string, number>)[attr];
  const reading = io.fmt(kind === 'angle' ? (v * 180) / Math.PI : v, 4);
  return { attr, text: c.written ?? c.expr(attr) ?? reading };
}

/** Open a dimension's number for editing, on the drawing where it is drawn. */
export function editValue(c: Constraint): void {
  if (!dimensionField(c)) return toast(`${c.typeName} has no editable dimension`);
  editDimension(c);            // already stated: nothing to add and nothing to place
}

/** Write `text` on every constraint being edited; false if the core would not have it, which
 *  leaves them all as they were.  A text the core takes but cannot yet compute *is* taken —
 *  the row says so until it can — since that is a document half-written, not a mistake. */
function setDimension(cs: Constraint[], text: string): boolean {
  try {
    let why: string | null = null;
    for (const c of cs) {
      const f = dimensionField(c);
      if (f) why = c.setDimension(f.attr, text) ?? why;
    }
    if (why) toast(`stored, but ${why}`);
    return true;
  } catch (err) {
    toast(`not a dimension: ${(err as Error).message}`);
    return false;
  }
}

/** **The one number editor**, on the canvas where its number is drawn: Enter accepts (and keeps
 *  it open on a refusal), Esc takes it back, clicking away accepts — the way a cell in a sheet
 *  does.  A dimension's and an extrusion's are each one of these; `finish` is the owner's. */
class NumberBox {
  private el: HTMLInputElement | null = null;
  /** Whether anybody has typed in it: until they have, it shows what it is told, which changes
   *  under it as the thing it edits is moved. */
  typed = false;

  constructor(private readonly title: string,
              private readonly finish: (commit: boolean, keepOpen?: boolean) => void) {}

  get open(): boolean { return this.el !== null; }

  /** What was typed, or nothing where nobody has typed. */
  get text(): string { return this.typed && this.el ? this.el.value.trim() : ''; }

  /** Show `text` — opening the box first — unless somebody has typed over it.  Only when the
   *  number itself moved: re-selecting the text every frame would fight a caret put in by hand. */
  show(text: string): HTMLInputElement {
    const box = this.el ?? this.make();
    if (!this.typed && box.value !== text) {
      box.value = text;
      sizeBox(box);
      box.select();
    }
    return box;
  }

  /** Put it at a canvas point, `dx` to the right of it. */
  place(at: [number, number] | null, dx = 0): void {
    if (!this.el || !at) return;
    this.el.style.left = `${at[0] + dx}px`;
    this.el.style.top = `${at[1]}px`;
  }

  close(): void {
    const box = this.el;
    this.el = null;                   // first, so the blur that removing it may fire does nothing
    box?.remove();
  }

  private make(): HTMLInputElement {
    const box = document.createElement('input');
    box.type = 'text';
    box.className = 'dim';
    box.spellcheck = false;
    box.title = this.title;
    this.typed = false;
    box.addEventListener('input', () => { this.typed = true; sizeBox(box); });
    box.addEventListener('keydown', (e) => {
      e.stopPropagation();
      if (e.key === 'Enter') { e.preventDefault(); this.finish(true, true); }
      if (e.key === 'Escape') { e.preventDefault(); this.finish(false); }
    });
    // the click that *plants* a callout is not a click away: the canvas refuses the focus for
    // exactly that reason
    box.addEventListener('blur', () => this.finish(true));
    (document.getElementById('canvas-wrap') as HTMLElement).append(box);
    this.el = box;
    box.focus();
    return box;
  }
}

function sizeBox(box: HTMLInputElement): void {
  box.style.width = `${Math.max(4, box.value.length + 1)}ch`;
}

const dimBox = new NumberBox('a number, or an expression over the document\'s numbers — `w / 2` '
  + 'reads `w := 80`,\nand a declared unknown (`param a: Length`) ties the dimensions that read '
  + 'it.\nEnter to accept, Esc to take it back', (commit, keepOpen) => finishDim(commit, keepOpen));

/** Done typing.  Accepted, the text goes on every constraint the dimension was opened on;
 *  refused, the whole thing comes back out.  A text the core will not have keeps the editor
 *  open when there is somebody still typing in it to correct it. */
function finishDim(commit: boolean, keepOpen = false): void {
  const live = view.liveDim;
  if (!live || !dimBox.open) return;
  // an untouched number is not written: the dimension keeps the measurement it was stated at,
  // which is exact, rather than the rounded one the editor showed
  const text = dimBox.text;
  // one the source already states is rewritten there, where its names mean what the document
  // says, and ends itself; one being stated now reaches the source when it is accepted
  if (commit && text && !live.fresh) {
    const why = view.rewriteDimension(text);
    if (why === null) {
      dimBox.close();
      invalidateRows();
      return;
    }
    toast(why);
    if (keepOpen) return;
    commit = false;
  } else if (commit && text && !setDimension(live.targets, text)) {
    if (keepOpen) return;
    commit = false;
  }
  dimBox.close();
  invalidateRows();                 // the row text states the number, so it has to rebuild
  view.endDimension(commit);
}

/** The view is writing a dimension, and this is where its number is on screen — or nulls
 *  when there is no longer one being written.  Wired onto the view in `main`. */
export function onDimension(live: LiveDim | null, at: [number, number] | null): void {
  if (!live) return dimBox.close();
  const box = dimBox.show(dimensionField(live.targets[0])?.text ?? '');
  // while it is being carried the number sits under the pointer, so a click meant to plant it
  // would land on the editor and plant nothing: the box lets the pointer through until the
  // placement is settled.  It keeps the focus throughout — carrying it and typing in it are
  // two different things, and the click that ends the first must not end the second.
  box.classList.toggle('carried', live.placing);
  dimBox.place(at);
}

const extentBox = new NumberBox('how thick: a number, or an expression over the document\'s '
  + 'numbers.\nDrag the arrow to size it — past its face for the other side, with ⇧ both ways.'
  + '\nEnter to accept, Esc to stop sizing', (commit, keepOpen) => finishExtent(commit, keepOpen));

/** Done typing an extrusion's thickness: accepted, it is written over the solid (`commitExtent`)
 *  and the box stays on the arrow for more; taken back, the sizing ends where it was last
 *  written.  A thickness the core will not have keeps the box open on Enter. */
function finishExtent(commit: boolean, keepOpen = false): void {
  if (!extentBox.open) return;
  const text = extentBox.text;
  if (commit && text) {
    const why = view.commitExtent(text);
    if (why) toast(why);
    else extentBox.typed = false;
    if (keepOpen) return;
  }
  extentBox.close();
  if (!commit) view.endExtent();
}

/** The view is sizing an extrusion, and this is where its number is on screen — or nulls when
 *  it is done.  Wired onto the view in `main`. */
export function onExtent(live: LiveExtent | null, at: [number, number] | null): void {
  if (!live) return extentBox.close();
  extentBox.show(io.fmt(live.d, 6));
  extentBox.place(at, 12);
}
