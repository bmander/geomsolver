/* The program the drawing came from, shown beside it and edited back into it.
 *
 * The core owns the language entirely — printing, reading, elaborating, and where every statement
 * sits.  This module is the panel: it puts the text on the page, keeps out of the way while
 * somebody is typing in it, and hands what they typed back.
 *
 * Two rules, both borrowed from modules that learned them the hard way.  `lists.ts` rebuilds its
 * rows only when the contents actually changed, so a caret and a scroll position survive an edit
 * made elsewhere; `dimbox.ts` will not overwrite a box somebody is in.  A panel that reprinted on
 * every solve would take the line out from under the cursor. */
import * as io from '../core/io.js';
import type { Diagnostic } from '../core/io.js';
import { pdiags, ppanel, ppanelState, ptext, view } from './shell.js';
import { toast } from './ui.js';

/** The text the panel last put there.  An identical reprint is one string compare and no write,
 *  which is what makes refreshing on every change affordable. */
let shown = '';
/** Somebody is editing: nothing overwrites the box until they are done with it. */
let typed = false;
let lastMap: io.SourceMap = { entities: [], constraints: [] };

export function programPanelOpen(): boolean {
  return !ppanel.hidden;
}

/** Show the program, or stop showing it.  Off by default: it is a second way of looking at the
 *  drawing, and the drawing is the first. */
export function toggleProgramPanel(): void {
  ppanel.hidden = !ppanel.hidden;
  if (!ppanel.hidden) {
    shown = '';                 // it has been away; whatever it held is stale
    refreshProgram();
  }
  view.resize();                // the canvas is a different width now
}

/** Re-print, unless the panel is being typed in or already says this. */
export function refreshProgram(): void {
  if (ppanel.hidden) return;
  if (typed) {
    ppanel.classList.add('dirty');
    ppanelState.textContent = ' — edited, ⌘↵ to apply';
    return;
  }
  const text = io.toProgram(view.sketch);
  if (text === shown) return;
  const { selectionStart, selectionEnd, scrollTop } = ptext;
  shown = text;
  ptext.value = text;
  ptext.setSelectionRange(selectionStart, selectionEnd);
  ptext.scrollTop = scrollTop;
  ppanel.classList.remove('dirty');
  ppanelState.textContent = '';
  showDiags([]);
}

/** Apply what is in the box.  One undo entry, one solve, one diagnosis — like every other edit.
 *
 *  A program that will not read leaves the drawing exactly as it was and says why: half a
 *  statement is not an instruction to delete anything. */
export function applyProgram(): boolean {
  const text = ptext.value;
  let e: io.Elaboration;
  try {
    e = io.fromProgram(text);
  } catch (err) {
    toast(`the program could not be read: ${(err as Error).message}`);
    return false;
  }
  showDiags(e.diagnostics);
  if (!e.ok || !e.sketch) {
    e.sketch?.dispose();
    const first = e.diagnostics.find((d) => d.severity === 'error');
    toast(first ? `line ${first.line}: ${first.message}` : 'the program has an error');
    return false;
  }
  lastMap = e.map;
  typed = false;
  shown = text;
  ppanel.classList.remove('dirty');
  ppanelState.textContent = '';
  view.pushUndo();
  view.setSketch(e.sketch, false);
  toast('program applied');
  return true;
}

/** Put back what the drawing says, throwing away what was typed. */
export function revertProgram(): void {
  typed = false;
  shown = '';
  ppanel.classList.remove('dirty');
  refreshProgram();
}

function showDiags(ds: Diagnostic[]): void {
  pdiags.replaceChildren();
  for (const d of ds) {
    const li = document.createElement('li');
    li.className = d.severity === 'error' ? 'error' : 'warn';
    li.textContent = `${d.code}  ${d.line}:${d.col}  ${d.message}`;
    li.addEventListener('click', () => {
      ptext.focus();
      ptext.setSelectionRange(d.lo, Math.max(d.hi, d.lo + 1));
    });
    pdiags.append(li);
  }
}

/** Light the statement that made what is picked.  `highlight`, never `selected`: setting the
 *  selection would fire `onSelect`, which is a press on the canvas and shuts the constraints
 *  window. */
export function showStatementFor(): void {
  if (ppanel.hidden || typed || document.activeElement === ptext) return;
  const first = view.selected[0];
  if (!first) return;
  const found = lastMap.entities.find(
    (x) => x.kind === first.ref[0] && x.index === first.ref[1],
  );
  if (!found) return;
  ptext.setSelectionRange(found.lo, found.hi);
  // scroll it into view without stealing the focus from the canvas
  const line = shown.slice(0, found.lo).split('\n').length - 1;
  ptext.scrollTop = Math.max(0, (line - 4) * 12 * 1.45);
}

/** Wire the box up.  Called once, from `main`, so this module is reached the way `lists` is and
 *  the view never has to import the shell. */
export function bindProgramPanel(): void {
  ptext.addEventListener('input', () => {
    typed = true;
    ppanel.classList.add('dirty');
    ppanelState.textContent = ' — edited, ⌘↵ to apply';
  });
  ptext.addEventListener('keydown', (e) => {
    // every accelerator in `main` is already yielded inside a TEXTAREA, but a handler that did
    // not stop here would still reach the window listeners below it
    e.stopPropagation();
    if (e.key === 'Enter' && (e.metaKey || e.ctrlKey)) {
      e.preventDefault();
      applyProgram();
    } else if (e.key === 'Escape') {
      e.preventDefault();
      // put the text back *before* blurring, so the blur handler finds nothing to apply
      revertProgram();
      ptext.blur();
    }
  });
  ptext.addEventListener('blur', () => {
    if (typed) applyProgram();
  });
  // a click in the text says which statement, and the drawing lights what it made
  ptext.addEventListener('click', () => {
    const off = ptext.selectionStart;
    const ent = lastMap.entities.find((x) => off >= x.lo && off < x.hi);
    if (!ent) return;
    const of: Record<string, () => { ref: io.Ref }[]> = {
      point: () => view.sketch.points,
      line: () => view.sketch.lines,
      circle: () => view.sketch.circles,
      arc: () => view.sketch.arcs,
      spline: () => view.sketch.splines,
      ellipse: () => view.sketch.ellipses,
    };
    const e = of[ent.kind]?.()[ent.index];
    if (e) {
      view.highlight = [e as never];
      view.draw();
    }
  });
}
