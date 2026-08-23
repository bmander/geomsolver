/* What the whole front end holds in common: the page's elements, the one SketchView on it, and
 * which constraint has the keyboard focus.  The core is started here, before anything reads it —
 * a module that imports this one is guaranteed a solver and a sketch. */
import * as examples from '../core/examples.js';
import { Constraint } from '../core/constraints.js';
import { Primitive, Sketch, expand } from '../core/model.js';
import { initCore } from '../core/wasm.js';
import { SketchView } from './view.js';
import { toast } from './ui.js';

export const canvas = document.getElementById('canvas') as HTMLCanvasElement;
export const menubar = document.getElementById('menubar') as HTMLElement;
export const aboutBadge = document.getElementById('about') as HTMLButtonElement;
export const aboutDag = document.getElementById('about-dag') as HTMLTemplateElement;
export const barTools = document.getElementById('bar-tools') as HTMLElement;
export const barConstraints = document.getElementById('bar-constraints') as HTMLElement;
export const clist = document.getElementById('clist') as HTMLElement;
export const elist = document.getElementById('elist') as HTMLElement;
export const banner = document.getElementById('banner') as HTMLElement;
export const bannerText = document.getElementById('banner-text') as HTMLElement;
export const bannerSelect = document.getElementById('banner-select') as HTMLButtonElement;
export const measureEl = document.getElementById('measure') as HTMLElement;
export const footerEl = document.querySelector('footer') as HTMLElement;

await initCore();
(document.getElementById('loading') as HTMLElement).remove();

/** The sketch the page opens on: the case named in the URL — `?example=pythagoras`, or an
 *  `…/example/<slug>` path the server handed to the page — else the default.  The slug is a case
 *  key, arguments and all (`truss:50`); one nothing answers to is said and the default shown. */
function initialSketch(): Sketch {
  const url = new URL(location.href);
  const slug = url.searchParams.get('example') ?? /\/example\/([^/]+)\/?$/.exec(url.pathname)?.[1];
  if (slug) {
    try {
      return examples.build(decodeURIComponent(slug));
    } catch (err) {
      setTimeout(() => toast(`no example “${slug}”: ${(err as Error).message}`, 12000), 0);
    }
  }
  return examples.rectFillets();
}

export const view = new SketchView(canvas, initialSketch());
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
