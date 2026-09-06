/** The paper preview in the main workspace. Source editing belongs to the project panel. */
import * as drawing from '../core/drawing.js';
import { download } from './ui.js';
import { DrawingViewport } from './drawing-viewport.js';

export async function pickDrawingFolder(): Promise<drawing.DrawingBundle | null> {
  const picked = await new Promise<File[]>((resolve) => {
    const input = document.createElement('input');
    input.type = 'file'; input.multiple = true; input.webkitdirectory = true;
    input.addEventListener('change', () => resolve(Array.from(input.files ?? [])));
    input.addEventListener('cancel', () => resolve([]));
    input.click();
  });
  if (!picked.length) return null;
  const files: Record<string, string> = {};
  await Promise.all(picked.filter((f) => /\.svd?$/.test(f.name)).map(async (f) => {
    const path = (f.webkitRelativePath || f.name).replace(/^[^/]+\//, '');
    files[path] = await f.text();
  }));
  return { source: '', files };
}

const pane = document.getElementById('drawing-view') as HTMLElement;
const sheet = document.getElementById('drawing-sheet') as HTMLSelectElement;
const image = document.getElementById('drawing-image') as HTMLImageElement;
const message = document.getElementById('drawing-status') as HTMLElement;
const viewport = new DrawingViewport(document.getElementById('drawing-viewport')!, image,
  document.getElementById('drawing-zoom')!);
let bundle: drawing.DrawingBundle | null = null;
let projectFiles: Record<string, string> | null = null;
let svg = '', url = '';
const sheets = new Map<string, string>();

export function drawingActive(): boolean { return bundle !== null; }

/** Paper is a workspace decoration, sized by the SVG's actual sheet bounds. Keep it out of
 *  the authored SVG so exports retain exactly what the drawing specifies. */
function paperPreview(svg: string): string {
  const doc = new DOMParser().parseFromString(svg, 'image/svg+xml');
  const paper = doc.createElementNS('http://www.w3.org/2000/svg', 'rect');
  for (const [name, value] of Object.entries({
    width: '100%', height: '100%', fill: '#ffffff', stroke: '#a1a1aa',
    'stroke-width': '2', 'vector-effect': 'non-scaling-stroke', 'data-preview-paper': '',
  })) paper.setAttribute(name, value);
  doc.documentElement.prepend(paper);
  return new XMLSerializer().serializeToString(doc);
}

export function showDrawing(next: drawing.DrawingBundle | null): void {
  if (next && next.files !== projectFiles) {
    sheets.clear();
    viewport.reset();
    projectFiles = next.files;
  }
  bundle = next;
  pane.hidden = next === null;
  document.getElementById('app')!.classList.toggle('drawing-active', next !== null);
  for (const id of ['bar-tools', 'bar-constraints']) {
    document.getElementById(id)!.inert = next !== null;
  }
  if (next) renderDrawing();
  else {
    viewport.stopPan();
    if (url) URL.revokeObjectURL(url);
    url = ''; svg = ''; image.removeAttribute('src');
  }
}

export function renderDrawing(): boolean {
  if (!bundle) return false;
  const { source, files } = bundle;
  try {
    const info = drawing.info(files[source]);
    const selected = sheets.get(source);
    sheet.replaceChildren(...info.sheets.map((s) => new Option(s, s)));
    if (selected && info.sheets.includes(selected)) sheet.value = selected;
    svg = drawing.render(files[source], source, files, sheet.value || undefined);
    viewport.show(`${source}\0${sheet.value}`);
    if (url) URL.revokeObjectURL(url);
    url = URL.createObjectURL(new Blob([paperPreview(svg)], { type: 'image/svg+xml' }));
    image.src = url; message.textContent = '';
    return true;
  } catch (e) {
    svg = ''; image.removeAttribute('src'); message.textContent = (e as Error).message;
    return false;
  }
}

export function exportDrawingSvg(): void {
  if (renderDrawing()) download(`${sheet.value}.svg`, svg);
}

sheet.addEventListener('change', () => {
  if (bundle) sheets.set(bundle.source, sheet.value);
  renderDrawing();
});
document.getElementById('drawing-export')!.addEventListener('click', exportDrawingSvg);
document.getElementById('drawing-fit')!.addEventListener('click', () => viewport.fit());
document.getElementById('drawing-zoom-in')!.addEventListener('click', () => viewport.zoom(1.25));
document.getElementById('drawing-zoom-out')!.addEventListener('click', () => viewport.zoom(1 / 1.25));
