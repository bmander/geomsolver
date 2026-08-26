/* What the whole front end holds in common: the page's elements, the one SketchView on it, and
 * which constraint has the keyboard focus.  The core is started here, before anything reads it —
 * a module that imports this one is guaranteed a solver and a sketch. */
import * as examples from '../core/examples.js';
import { Constraint } from '../core/constraints.js';
import { Primitive, Sketch, expand } from '../core/model.js';
import { initCore } from '../core/wasm.js';
import { Document, fromSketch, highlight } from '../core/program.js';
import { CodeEditor } from './editor.js';
import { SketchView } from './view.js';
import { toast } from './ui.js';

export const canvas = document.getElementById('canvas') as HTMLCanvasElement;
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

await initCore();
(document.getElementById('loading') as HTMLElement).remove();

/** The document the page opens on: the case named in the URL — `?example=pythagoras`, or an
 *  `…/example/<slug>` path the server handed to the page — else the default.  The slug is a case
 *  key, arguments and all (`truss:50`); one nothing answers to is said and the default shown.
 *
 *  A *program*, not a sketch, because that is what a document is: a case written as one comes
 *  with the source somebody wrote — the gear's curve family, its components, the reasons in the
 *  comments — and lifting the drawing instead would open a hundred and twenty point declarations
 *  about the same shape.  One built by a function has no source and is lifted, which is the only
 *  place `examples.source` differs from `examples.build`. */
function initialProgram(): string {
  const url = new URL(location.href);
  const slug = url.searchParams.get('example') ?? /\/example\/([^/]+)\/?$/.exec(url.pathname)?.[1];
  if (slug) {
    try {
      return examples.source(decodeURIComponent(slug));
    } catch (err) {
      setTimeout(() => toast(`no example “${slug}”: ${(err as Error).message}`, 12000), 0);
    }
  }
  return examples.source('rect_fillets:100:60:10');
}

export const view = new SketchView(canvas, Document.read(initialProgram()));
export let currentConstraint: Constraint | null = null;

/** Move the keyboard focus onto a constraint row, or off it with null.  Delete acts on
 *  whichever of the two selections holds the focus, so exactly one of `currentConstraint` and
 *  `view.selected` is ever populated — that is the whole reason deleting a constraint stopped
 *  taking the geometry with it, so every path that sets either one comes through here. */
export function focusConstraint(c: Constraint | null, highlight?: Primitive[]): void {
  currentConstraint = c;
  view.litConstraint = c;             // so its callout on the drawing says so too
  view.highlight = highlight ?? (c ? expand(c.entities()) : []);
  if (c) view.selected = [];
}

/** The focused constraint is gone from the document.  Drop the focus without touching what is
 *  lit: the drawing is already rid of it, and `focusConstraint` would be saying something
 *  about a constraint that no longer exists. */
export function clearFocus(): void {
  currentConstraint = null;
}
