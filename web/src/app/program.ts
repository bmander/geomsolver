/* The document, shown beside the drawing it makes.
 *
 * **This is not a view of the sketch — it is the source, and the drawing is what it came to.**  So
 * the panel shows `view.source` and never re-prints: the comments somebody wrote, the components
 * they factored out and the blank lines they left are the document as much as the numbers are, and
 * a re-print would put a lift of the drawing there instead, losing all of it on the first drag.
 *
 * The core owns the language entirely — reading, elaborating, editing and where every statement
 * sits.  This module is the panel: it puts the text on the page, keeps out of the way while
 * somebody is typing in it, and hands what they typed back.
 *
 * Two rules, both borrowed from modules that learned them the hard way.  `lists.ts` rebuilds its
 * rows only when the contents actually changed, so a caret and a scroll position survive an edit
 * made elsewhere; `dimbox.ts` will not overwrite a box somebody is in.  A panel that reprinted on
 * every solve would take the line out from under the cursor. */
import type { Diagnostic, SourceMap } from '../core/program.js';
import { pdiags, ppanel, ppanelState, ptext, view } from './shell.js';
import { toast } from './ui.js';

/** The text the panel last put there.  An identical reprint is one string compare and no write,
 *  which is what makes refreshing on every change affordable. */
let shown = '';
/** Somebody is editing: nothing overwrites the box until they are done with it. */
let typed = false;


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
  const text = view.source;
  if (text === shown) return;
  const { selectionStart, selectionEnd, scrollTop } = ptext;
  shown = text;
  ptext.value = text;
  ptext.setSelectionRange(selectionStart, selectionEnd);
  ptext.scrollTop = scrollTop;
  ppanel.classList.remove('dirty');
  ppanelState.textContent = '';
  showDiags(view.doc.diagnostics);
}

/** Apply what is in the box.  One undo entry, one solve, one diagnosis — like every other edit.
 *
 *  A program that will not read leaves the drawing exactly as it was and says why: half a
 *  statement is not an instruction to delete anything. */
export function applyProgram(): boolean {
  const text = ptext.value;
  const undo = view.source;
  if (!view.setProgram(text, false)) return false;
  showDiags(view.doc.diagnostics);
  if (!view.doc.ok) {
    // it drew what it could, and the errors are in the gutter beside the lines that caused them
    const first = view.doc.diagnostics.find((d) => d.severity === 'error');
    toast(first ? `line ${first.line}: ${first.message}` : 'the program has an error');
  } else {
    toast('program applied');
  }
  view.pushUndo(undo);
  typed = false;
  shown = text;
  ppanel.classList.remove('dirty');
  ppanelState.textContent = '';
  return view.doc.ok;
}

/** Put back what the drawing says, throwing away what was typed. */
export function revertProgram(): void {
  typed = false;
  shown = '';
  ppanel.classList.remove('dirty');
  refreshProgram();
}

/** Where every part of the drawing was written, in *this* elaboration.  Asked each time rather
 *  than remembered: a map belongs to the elaboration that made it, and this panel outlives many. */
function map(): SourceMap {
  return view.doc.map;
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
  const found = map().entities.find(
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
    // the innermost statement containing the caret: a statement inside a block is inside its
    // block's span, and the one that made something is the one a click there means
    let best: { name: string; lo: number; hi: number } | null = null;
    for (const x of map().entities) {
      if (off < x.lo || off >= x.hi) continue;
      if (!best || x.hi - x.lo < best.hi - best.lo) best = x;
    }
    const e = best && view.doc.entity(best.name);
    if (e) {
      view.highlight = [e];
      view.draw();
    }
  });
}
