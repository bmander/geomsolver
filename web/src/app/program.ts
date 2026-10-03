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
import type { Mark } from './editor.js';
import type { Diagnostic, SourceMap, SourceSpan } from '../core/program.js';
import * as modules from '../core/modules.js';
import type { DrawingBundle } from '../core/drawing.js';
import { drawingActive, pickDrawingFolder, renderDrawing, showDrawing } from './drawing.js';
import { currentConstraint, pdiags, ped, ppanel, ppanelState, psplit, view } from './shell.js';
import { download, toast } from './ui.js';
import { awaitSurfaces, loading, opened, painted } from './loading.js';
import { ProjectExplorer, projectFile, sourceFile } from './project-explorer.js';

/** The box somebody types in.  `app/editor.ts` owns the two layers and the colouring; this module
 *  owns what the text *means* — which document it came from and what applying it does. */
const ptext = ped.box;

/** The text the panel last put there.  An identical reprint is one string compare and no write,
 *  which is what makes refreshing on every change affordable. */
let shown = '';
/** Somebody is editing: nothing overwrites the box until they are done with it. */
let typed = false;

const filePicker = document.getElementById('program-file') as HTMLSelectElement;
const fileBar = document.getElementById('program-files') as HTMLElement;
const programName = document.getElementById('program-name')!;
const explorer = new ProjectExplorer(document.getElementById('project-pane')!,
  document.getElementById('project-tree')!, document.getElementById('project-title')!,
  (p) => void chooseFile(p));
let directory: string | undefined;
let onProjectFile: ((path: string) => void) | undefined;
let activeFile = '';
/** The project's `.sv` the canvas holds, which a text file shown in the panel leaves in place. */
let modelFile = '';
let filesFor: string | null = null;
let files: modules.SourceFile[] = [];
let project: DrawingBundle | null = null;
let switching = false;
const positions = new Map<string, { start: number; end: number; top: number; left: number }>();

/** The panel shows a project's text file (a README): kept and saved, never elaborated. */
function showingText(): boolean {
  return project !== null && !sourceFile(activeFile);
}

/** What the canvas came to belongs to the model it holds, whichever file the panel shows. */
function keepModel(): void {
  if (project && modelFile && !drawingActive() && !typed) project.files[modelFile] = view.source;
}

function refreshFiles(): void {
  if (project) return;
  if (filesFor === view.source) return;
  filesFor = view.source;
  files = modules.related(view.source);
  if (!files.some((f) => f.name === activeFile)) activeFile = '';
  filePicker.replaceChildren(new Option('Main drawing', ''),
    ...files.map((f) => new Option(f.path, f.name)));
  filePicker.value = activeFile;
  fileBar.hidden = files.length === 0;
}

/** Select a project file, or browse an import of a standalone model. */
function selectFile(name: string): void {
  if (name === activeFile) return;
  if (typed && !applyProgram() && !project) {
    filePicker.value = activeFile;
    return;
  }
  positions.set(activeFile, { start: ptext.selectionStart, end: ptext.selectionEnd,
    top: ptext.scrollTop, left: ptext.scrollLeft });
  if (project) {
    keepModel();
    if (!activateProjectFile(name)) { filePicker.value = activeFile; return; }
  }
  activeFile = name;
  filePicker.value = name;
  explorer.select(name);
  if (project) onProjectFile?.(name);
  shown = '';
  refreshProgram();
  const pos = positions.get(name);
  ptext.setSelectionRange(pos?.start ?? 0, pos?.end ?? 0);
  ptext.scrollTop = pos?.top ?? 0;
  ptext.scrollLeft = pos?.left ?? 0;
  ptext.dispatchEvent(new Event('scroll'));
}

/** A file a person picks.  A project's source file is elaborated and solved (or its paper
 *  rendered) on this thread, so the loading modal says so first and, as an opening does, stays
 *  until the swept objects of the model it shows have a surface. */
async function chooseFile(name: string): Promise<void> {
  if (name === activeFile || !project || !sourceFile(name)) { selectFile(name); return; }
  loading(`${name.endsWith('.svd') ? 'rendering' : 'solving'} ${name}…`);
  await painted();
  awaitSurfaces();
  try { selectFile(name); }
  finally { opened(); }
}

/** Close whatever project the panel holds, as a new document replaces it; false where a project's
 *  own file is being shown, which is no new document. */
export function resetProgramFiles(): boolean {
  if (switching) return false;
  project = null;
  directory = undefined; onProjectFile = undefined;
  explorer.show([]); programName.hidden = true;
  showDrawing(null);
  activeFile = ''; modelFile = '';
  ped.plain = false;
  filesFor = null;
  positions.clear();
  revertProgram();
  return true;
}

/** Both source kinds live in one project; the selected extension chooses the left-hand view.
 *  A text file is shown in the panel beside whichever source the left-hand view last held, so a
 *  project opened on its README draws its entry (`options.entry`) there. */
export function openProject(bundle: DrawingBundle, options: {
  directory?: string; entry?: string; onSelect?: (path: string) => void;
} = {}): boolean {
  const names = Object.keys(bundle.files).filter(projectFile).sort();
  const sources = names.filter(sourceFile);
  const source = (p?: string) => p !== undefined && sources.includes(p) ? p : undefined;
  const main = source(bundle.source) ?? source(options.entry)
    ?? sources.find((p) => p.endsWith('.svd')) ?? sources[0];
  if (!main) { toast('This project has no .sv or .svd files'); return false; }
  view.pauseEditing();
  project = { source: main, files: { ...bundle.files } };
  directory = options.directory; onProjectFile = options.onSelect;
  positions.clear();
  activeFile = main; modelFile = '';
  filePicker.replaceChildren(new Option(main, main),
    ...names.filter((p) => p !== main).map((p) => new Option(p, p)));
  filePicker.value = main;
  fileBar.hidden = directory !== undefined;
  explorer.show(names, directory); explorer.select(main);
  activateProjectFile(main);
  revertProgram();
  if (bundle.source !== main && names.includes(bundle.source)) selectFile(bundle.source);
  return true;
}

/** Open a folder the person picks as the project; whether one was opened. */
export async function openDrawing(): Promise<boolean> {
  try {
    const bundle = await pickDrawingFolder();
    if (bundle) return openProject(bundle, { directory: '' });
  } catch (e) { toast(`Could not read drawing folder: ${(e as Error).message}`); }
  return false;
}

/** Save the selected source, including a draft that has not compiled yet. */
export function saveProjectFile(): boolean {
  if (!project) return false;
  download(activeFile.split('/').pop()!, typed ? ptext.value : project.files[activeFile]);
  return true;
}
function activateProjectFile(name: string): boolean {
  if (!project || !(name in project.files)) return false;
  switching = true;
  try {
    if (!sourceFile(name)) {
      // the left-hand view stays on what it was showing
    } else if (name.endsWith('.svd')) {
      view.pauseEditing();
      showDrawing({ source: name, files: project.files });
    } else {
      modules.provideProject(name, project.files);
      if (!view.openProjectFile(project.files[name])) return false;
      modelFile = name;
      showDrawing(null);
    }
    typed = false;
    return true;
  } finally { switching = false; }
}


export function programPanelOpen(): boolean {
  return !ppanel.hidden;
}

/** Show the program, or stop showing it.
 *
 *  **On by default.**  The source is not a remark about the drawing — it is what the drawing
 *  *is*, so the page opens on both, and closing it is the deliberate act.  The partition goes
 *  wherever the panel does: a handle for something that is not there resizes nothing. */
export function toggleProgramPanel(): void {
  ppanel.hidden = !ppanel.hidden;
  psplit.hidden = ppanel.hidden;
  if (!ppanel.hidden) {
    shown = '';                 // it has been away; whatever it held is stale
    refreshProgram();
  }
  view.resize();                // the canvas is a different width now
}

/** The partition between the drawing and the source.
 *
 *  The panel is on the right, so its width is the distance from the pointer to the row's right
 *  edge — measured against that edge rather than accumulated from where the drag began, so a
 *  pointer that runs past a limit and comes back picks the edge up where it left it instead of
 *  an offset away.  The canvas is watched by a `ResizeObserver` (`main.ts`), so nothing here
 *  has to tell the view it got narrower. */
function bindPartition(): void {
  const to = (clientX: number): void => {
    const row = ppanel.parentElement;
    if (row) setPanelWidth(row.getBoundingClientRect().right - clientX);
  };
  psplit.addEventListener('pointerdown', (e) => {
    e.preventDefault();                        // a drag on a separator is not a text selection
    psplit.setPointerCapture(e.pointerId);
    psplit.classList.add('dragging');
  });
  psplit.addEventListener('pointermove', (e) => {
    if (psplit.hasPointerCapture(e.pointerId)) to(e.clientX);
  });
  const done = (e: PointerEvent): void => {
    if (!psplit.hasPointerCapture(e.pointerId)) return;
    psplit.releasePointerCapture(e.pointerId);
    psplit.classList.remove('dragging');
  };
  psplit.addEventListener('pointerup', done);
  psplit.addEventListener('pointercancel', done);
  // and by keyboard, since it is focusable: a separator nobody can reach with the keyboard is a
  // control only half the people using it have
  psplit.addEventListener('keydown', (e) => {
    const step = e.key === 'ArrowLeft' ? 1 : e.key === 'ArrowRight' ? -1 : 0;
    if (!step) return;
    e.preventDefault();
    setPanelWidth(ppanel.getBoundingClientRect().width + step * (e.shiftKey ? 40 : 8));
  });
}

/** How wide the panel may be, as the stylesheet says.  Read rather than restated: the drag and
 *  the layout would otherwise be two rules about one limit, and the first edit to either would
 *  make them disagree. */
function widthBounds(room: number): [number, number] {
  const s = getComputedStyle(ppanel);
  // `width` comes back resolved to pixels; `min-width`/`max-width` do **not** — a percentage is
  // handed back as it was written, so `parseFloat('60%')` is sixty *pixels* unless it is asked
  // what of.  Left unasked, the panel clamped itself to a 60px maximum and the handle spent the
  // whole drag pinned against a limit the layout was quietly correcting.
  const px = (v: string, fallback: number): number => {
    const n = parseFloat(v);
    if (!Number.isFinite(n)) return fallback;
    return v.trimEnd().endsWith('%') ? (n / 100) * room : n;
  };
  return [px(s.minWidth, 240), px(s.maxWidth, room * 0.6)];
}

/** Put the partition somewhere, in whatever units keep it there.
 *
 *  A **percentage** of the row, not pixels: the panel opens at 30% of the window, and a width
 *  frozen in pixels the moment somebody nudged the handle would stop tracking a window that is
 *  then resized — the drawing and the source would drift apart on a laptop being plugged into a
 *  monitor.  Clamped here to what the stylesheet allows, so the handle never runs on past a
 *  limit the layout is quietly enforcing and leave the pointer somewhere the edge is not. */
function setPanelWidth(px: number): void {
  const room = ppanel.parentElement?.clientWidth ?? window.innerWidth;
  if (room <= 0) return;
  const [lo, hi] = widthBounds(room);
  const w = Math.min(Math.max(px, lo), Math.min(hi, room));
  ppanel.style.width = `${((w / room) * 100).toFixed(3)}%`;
}

/** Re-print, unless the panel is being typed in or already says this. */
export function refreshProgram(): void {
  if (switching) return;
  keepModel();
  if (ppanel.hidden) return;
  refreshFiles();
  programName.hidden = directory === undefined;
  programName.textContent = activeFile;
  const imported = !project && activeFile !== '';
  ptext.readOnly = imported;
  ptext.setAttribute('aria-label', project ? activeFile
    : activeFile ? modules.pathOf(activeFile) : 'Main drawing source');
  markStatement();              // the pick may have moved even where the text has not
  if (typed) {
    ppanel.classList.add('dirty');
    ppanelState.textContent = ' — edited, ⌘↵ to apply';
    return;
  }
  const text = project ? project.files[activeFile]
    : files.find((f) => f.name === activeFile)?.text ?? view.source;
  if (text === shown) return;
  shown = text;
  const quiet = imported || drawingActive() || showingText();
  ped.plain = showingText();
  ped.setText(text, quiet ? [] : marks());
  ppanel.classList.remove('dirty');
  ppanelState.textContent = imported ? ' — imported file, read only' : '';
  showDiags(quiet ? [] : view.doc.diagnostics);
}

/** Apply what is in the box.  One undo entry, one solve, one diagnosis — like every other edit.
 *
 *  A program that will not read leaves the drawing exactly as it was and says why: half a
 *  statement is not an instruction to delete anything. */
export function applyProgram(): boolean {
  if (!project && activeFile) return false;
  // a text file is kept as it is typed, and applying it re-renders whatever the left side shows
  if (showingText()) return drawingActive() ? renderDrawing() : true;
  if (drawingActive() && !typed) return renderDrawing();
  const text = ptext.value;
  if (project) project.files[activeFile] = text;
  if (drawingActive()) {
    typed = false;
    shown = '';
    const ok = renderDrawing();
    refreshProgram();
    return ok;
  }
  const undo = view.source;
  if (project) modules.provideProject(activeFile, project.files);
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
  // The solve and the diagnosis happened inside `setProgram`, back when `typed` was still true —
  // so the refresh it triggered took the early exit above and the marks still describe the
  // document as it was before this apply.  Nothing will come back for them, either: `shown` now
  // equals the text, so the next `refreshProgram` returns at its own guard.  This is the moment
  // the panel stops being a draft, and so the moment it has to catch up with what was judged.
  markStatement();
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

/** Where the thing being looked at was written down — the focused constraint, or else the first
 *  selected element.  Null when neither is, and when what is picked came out of a component and
 *  so has no statement of its own in this document to point at.
 *
 *  The *policy* is here and the *lookup* is the document's: which of the two picks answers is a
 *  question about this panel, and where a thing was written is a question about the source. */
function litSpan(): SourceSpan | null {
  const c = currentConstraint;
  const first = view.selected[0];
  const doc = view.doc;
  const at = c ? doc.spanOfConstraint(c.id) : first ? doc.spanOf(first) : undefined;
  return at ?? null;
}

/** Every stretch of the source that is marked: the statement whatever is picked was written as,
 *  and each `claim` (§9.7) tinted by how the diagnosis judged it.
 *
 *  A claim is judged where it is *written*, because that is what it is — a sentence in the
 *  document, not a state of the drawing.  It is also the quietest place to say so, which is the
 *  point: this app is for sketching, and a claim is a remark somebody left in the margin.  A
 *  wash of colour behind the statement is as much attention as proving should ever ask for, and
 *  it costs the drawing none at all.
 *
 *  The three verdicts are the classical trichotomy, and the words are chosen to be exact:
 *  *proved*, the document entails it; *refuted*, this drawing is a counterexample; *independent*,
 *  true here but not implied — stating it would have cost a freedom. */
function marks(): Mark[] {
  const out: Mark[] = [];
  const lit = litSpan();
  if (lit) out.push({ lo: lit.lo, hi: lit.hi, cls: 'lit' });
  const d = view.diagnosis;
  if (!d) return out;
  const doc = view.doc;
  for (const [cs, cls] of [
    [d.claimsTheorem, 'claim-proved'],
    [d.claimsViolated, 'claim-refuted'],
    [d.claimsConsuming, 'claim-independent'],
  ] as const) {
    for (const c of cs) {
      const at = doc.spanOfConstraint(c.id);
      if (at) out.push({ lo: at.lo, hi: at.hi, cls });
    }
  }
  return out;
}

/** Mark the statement whatever is picked was written as, and the claims the diagnosis has judged.
 *
 *  Cheap enough to call whenever anything might have moved — `setMarks` returns without painting
 *  when the range is the one it already has — **except during a gesture**, where it is not:
 *  a rubber band fires `onSelect` every frame, the leading edge sweeping past an element changes
 *  which one is first, and repainting the whole copy mid-sweep would spend a millisecond a frame
 *  marking something nobody can read yet.  The gesture ends in `onChanged`, which comes back
 *  through here. */
export function markStatement(): void {
  if (switching || drawingActive() || showingText() || ppanel.hidden || (!project && activeFile)
    || typed || view.gesture) return;
  ped.setMarks(marks());
}

/** Mark it *and* bring it on screen — what a deliberate pick does, as against the drawing moving
 *  under one.  Called from both funnels: `view.onSelect` for the drawing, `hooks.focusChanged`
 *  for a constraint, so picking either way says the same thing here. */
export function showStatementFor(): void {
  if (switching || drawingActive() || showingText()) return;
  markStatement();
  const where = litSpan();
  // the scroll is only for a box nobody is in: moving it under somebody who is typing would take
  // the line out from under their caret
  if (!where || ppanel.hidden || typed || view.gesture) return;
  if (!project && activeFile) selectFile('');
  markStatement();
  if (document.activeElement !== ptext) ped.scrollToLine(lineAt(where.lo));
}

/** Which line an offset falls on.  Counted rather than sliced: `slice(0, off).split` builds the
 *  whole prefix and an array of every line in it, on every pick. */
function lineAt(off: number): number {
  let line = 0;
  for (let i = 0; i < off && i < shown.length; i += 1) if (shown[i] === '\n') line += 1;
  return line;
}

/** Wire the box up.  Called once, from `main`, so this module is reached the way `lists` is and
 *  the view never has to import the shell. */
export function bindProgramPanel(): void {
  bindPartition();
  document.getElementById('drawing-render')!.addEventListener('click', () => applyProgram());
  filePicker.addEventListener('change', () => void chooseFile(filePicker.value));
  ptext.addEventListener('input', () => {
    if (!project && activeFile) return;
    if (showingText()) {
      project!.files[activeFile] = shown = ptext.value;
      ped.repaint();
      return;
    }
    typed = true;
    // colour what was just typed, not what the drawing came from: half a statement is still the
    // program somebody is looking at, and the core colours it as far as it goes
    ped.repaint();
    ppanel.classList.add('dirty');
    ppanelState.textContent = ' — edited, ⌘↵ to apply';
  });
  ptext.addEventListener('keydown', (e) => {
    // every accelerator in `main` is already yielded inside a TEXTAREA, but a handler that did
    // not stop here would still reach the window listeners below it
    e.stopPropagation();
    if (!project && activeFile) return;
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
    if (drawingActive() || showingText() || (!project && activeFile)) return;
    const off = ptext.selectionStart;
    // the innermost statement containing the caret: a statement inside a block is inside its
    // block's span, and the one that made something is the one a click there means
    // resolved by kind and index rather than by name, so a statement that declares an
    // **anonymous** element lights it like any other — it has no name to be asked by
    let best: { kind: string; index: number; lo: number; hi: number } | null = null;
    for (const x of map().entities) {
      if (off < x.lo || off >= x.hi) continue;
      if (!best || x.hi - x.lo < best.hi - best.lo) best = x;
    }
    const e = best && view.doc.entityOf(best);
    if (e) {
      view.highlight = [e];
      view.draw();
    }
  });
}
