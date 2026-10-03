/* What the whole front end holds in common: the page's elements, the one SketchView on it, and
 * which constraint has the keyboard focus.  The core is started here, before anything reads it —
 * a module that imports this one is guaranteed a solver and a sketch. */
import { Constraint } from '../core/constraints.js';
import { expand, type Primitive } from '../core/model.js';
import { initCore } from '../core/wasm.js';
import { Document, highlight } from '../core/program.js';
import { CodeEditor } from './editor.js';
import { SketchView } from './view.js';

export const canvas = document.getElementById('canvas') as HTMLCanvasElement;
/** The workspace's three.js canvas: under the sketch's and deaf to the pointer.  It is a *second
 *  surface* and not a second app — every gesture runs against the one above it. */
export const boxCanvas = document.getElementById('box') as HTMLCanvasElement;
/** The plane the next thing is drawn on, chosen in the viewport's upper right. */
export const planeSelect = document.getElementById('plane-select') as HTMLSelectElement;
export const menubar = document.getElementById('menubar') as HTMLElement;
export const aboutBadge = document.getElementById('about') as HTMLButtonElement;
export const aboutDag = document.getElementById('about-dag') as HTMLTemplateElement;
export const barTools = document.getElementById('bar-tools') as HTMLElement;
export const barConstraints = document.getElementById('bar-constraints') as HTMLElement;
export const cpanel = document.getElementById('cpanel') as HTMLElement;
export const cpanelTitle = document.getElementById('cpanel-title') as HTMLElement;
export const clist = document.getElementById('clist') as HTMLElement;
export const banner = document.getElementById('banner') as HTMLElement;
export const bannerText = document.getElementById('banner-text') as HTMLElement;
export const bannerSelect = document.getElementById('banner-select') as HTMLButtonElement;
export const measureEl = document.getElementById('measure') as HTMLElement;
export const ppanel = document.getElementById('ppanel') as HTMLElement;
export const psplit = document.getElementById('psplit') as HTMLElement;
export const ppanelState = document.getElementById('ppanel-state') as HTMLElement;
/** The program panel's code box — the two layers, built into `#pcode` by `app/editor.ts`.  It is
 *  handed the core's colouring and knows nothing else about Solvent. */
export const ped = new CodeEditor(document.getElementById('pcode') as HTMLElement, highlight);
export const pdiags = document.getElementById('pdiags') as HTMLElement;
export const componentEl = document.getElementById('component') as HTMLElement;
export const footerEl = document.querySelector('footer') as HTMLElement;

// the loading modal stays up through the first example's opening (`openExample`), which puts it away
await initCore();

/** Example routes open an authored drawing after the shell is wired. The model editor
 *  starts empty; opening an example never loads its assembly as the active document. */
const url = new URL(location.href);
export const initialExample = url.searchParams.get('example')
  ?? /\/example\/([^/]+)\/?$/.exec(url.pathname)?.[1] ?? 'rect_fillets';
export const view = new SketchView(canvas, Document.read(''), boxCanvas);
export let currentConstraint: Constraint | null = null;

/** What the shell tells the rest of the page when the focus moves.  Assigned by `main`, the way
 *  the view's own handlers are: `program` reads the shell, so the shell must not read it back. */
export const hooks: { focusChanged: () => void } = { focusChanged: () => {} };

/** Move the keyboard focus onto a constraint row, or off it with null.  Delete acts on
 *  whichever of the two selections holds the focus, so exactly one of `currentConstraint` and
 *  `view.selected` is ever populated — that is the whole reason deleting a constraint stopped
 *  taking the geometry with it, so every path that sets either one comes through here. */
export function focusConstraint(c: Constraint | null, highlight?: Primitive[]): void {
  currentConstraint = c;
  view.litConstraint = c;             // so its callout on the drawing says so too
  view.highlight = highlight ?? (c ? expand(c.entities()) : []);
  if (c) {
    view.selected = [];
    view.dropImage();       // focusing a constraint is selecting something, so the picture lets go
  }
  hooks.focusChanged();               // last: everything it reads is settled by now
}

/** The focused constraint is gone from the document.  Drop the focus without touching what is
 *  lit: the drawing is already rid of it, and `focusConstraint` would be saying something
 *  about a constraint that no longer exists. */
export function clearFocus(): void {
  currentConstraint = null;
  hooks.focusChanged();               // the other writer of the focus, and it says so too
}
