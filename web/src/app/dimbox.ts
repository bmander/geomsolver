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
import type { LiveDim } from './view.js';

/** A dimension's number as a person reads and writes it: what it was written as if it was
 *  written, else what the drawing says it is — the same rounding, so the editor that sits on a
 *  callout shows the callout's own number rather than a longer one that means the same.  In
 *  degrees for an angle, which is the unit the core takes text in either way.
 *
 *  Nothing is written back unless somebody types, so the rounding here costs no precision: a
 *  dimension nobody edits keeps the measurement it was stated at, to the last figure. */
function dimensionField(c: Constraint): { attr: string; text: string } | null {
  const [d] = c.dimensions();
  if (!d) return null;
  const [attr, kind] = d;
  const v = (c as unknown as Record<string, number>)[attr];
  return { attr, text: c.expr(attr) ?? io.fmt(kind === 'angle' ? (v * 180) / Math.PI : v, 4) };
}

/** Open a dimension's number for editing, on the drawing where it is drawn. */
export function editValue(c: Constraint): void {
  if (!dimensionField(c)) return toast(`${c.typeName} has no editable dimension`);
  editDimension(c);            // already stated: nothing to add and nothing to place
}

/** Write `text` on every constraint being edited; false if the core would not have it, which
 *  leaves them all as they were.  A text that reads a name nothing defines *is* taken — the
 *  row says so until the name appears — since that is a document half-written, not a mistake. */
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

let dimBox: HTMLInputElement | null = null;
/** Whether anybody has typed in it: until they have, it keeps showing what the dimension
 *  measures, which changes under it as the callout is moved from one kind to another. */
let dimTyped = false;

function sizeDimBox(box: HTMLInputElement): void {
  box.style.width = `${Math.max(4, box.value.length + 1)}ch`;
}

function openDimBox(): HTMLInputElement {
  const box = document.createElement('input');
  box.type = 'text';
  box.className = 'dim';
  box.spellcheck = false;
  box.title = 'a number, or an expression — `w = 80` names one, `w / 2` reads it, and a name\nnothing defines ties the dimensions that read it and leaves what they are worth open.\nEnter to accept, Esc to take it back';
  dimTyped = false;
  box.addEventListener('input', () => { dimTyped = true; sizeDimBox(box); });
  box.addEventListener('keydown', (e) => {
    e.stopPropagation();
    if (e.key === 'Enter') { e.preventDefault(); finishDim(true, true); }
    if (e.key === 'Escape') { e.preventDefault(); finishDim(false); }
  });
  // clicking away accepts it, the way a cell in a sheet does.  The click that *plants* the
  // callout is not a click away: the canvas refuses the focus for exactly that reason
  box.addEventListener('blur', () => finishDim(true));
  (document.getElementById('canvas-wrap') as HTMLElement).append(box);
  dimBox = box;
  box.focus();
  return box;
}

function closeDimBox(): void {
  const box = dimBox;
  dimBox = null;                    // first, so the blur that removing it may fire does nothing
  box?.remove();
}

/** Done typing.  Accepted, the text goes on every constraint the dimension was opened on;
 *  refused, the whole thing comes back out.  A text the core will not have keeps the editor
 *  open when there is somebody still typing in it to correct it. */
function finishDim(commit: boolean, keepOpen = false): void {
  const live = view.liveDim;
  if (!live || !dimBox) return;
  // an untouched number is not written: the dimension keeps the measurement it was stated at,
  // which is exact, rather than the rounded one the editor showed
  const text = dimTyped ? dimBox.value.trim() : '';
  if (commit && text && !setDimension(live.targets, text)) {
    if (keepOpen) return;
    commit = false;
  }
  closeDimBox();
  invalidateRows();                 // the row text states the number, so it has to rebuild
  view.endDimension(commit);
}

/** The view is writing a dimension, and this is where its number is on screen — or nulls
 *  when there is no longer one being written.  Wired onto the view in `main`. */
export function onDimension(live: LiveDim | null, at: [number, number] | null): void {
  if (!live) return closeDimBox();
  const box = dimBox ?? openDimBox();
  // while it is being carried the number sits under the pointer, so a click meant to plant it
  // would land on the editor and plant nothing: the box lets the pointer through until the
  // placement is settled.  It keeps the focus throughout — carrying it and typing in it are
  // two different things, and the click that ends the first must not end the second.
  box.classList.toggle('carried', live.placing);
  if (!dimTyped) {
    const text = dimensionField(live.targets[0])?.text ?? '';
    if (box.value !== text) {         // only when the number itself moved: re-selecting the
      box.value = text;               // text every frame would fight a caret put in by hand
      sizeDimBox(box);
      box.select();
    }
  }
  if (at) {
    box.style.left = `${at[0]}px`;
    box.style.top = `${at[1]}px`;
  }
}
