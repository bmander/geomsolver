/* **The section bar**: while the workspace is sectioned (`SketchView.section`), a strip in the
 * viewport's upper left saying by which plane, how far along its normal the cut is moved, and
 * which side it keeps.  The plane is chosen from the planes a sketch can be drawn on; the slider
 * runs over how far the objects reach along that plane's normal (the core's `reachAlong`), so the
 * cut sweeps through all of them and no farther; Flip keeps the other side, and × stops
 * sectioning.  All of it is view state: nothing here is written to the document. */
import { fmt } from '../core/io.js';
import { reachAlong } from '../core/section.js';
import type { SketchView } from './view.js';

export class SectionBar {
  private readonly plane = document.createElement('select');
  private readonly slider = document.createElement('input');
  private readonly readout = document.createElement('input');
  private readonly flip = document.createElement('button');
  /** The planes the chooser was filled with, so it is filled again only when they change. */
  private planes = '';

  constructor(readonly el: HTMLElement, readonly view: SketchView) {
    const name = document.createElement('span');
    name.className = 'sb-name';
    name.textContent = 'Section';
    this.plane.setAttribute('aria-label', 'The plane the workspace is cut by');
    this.slider.type = 'range';
    this.slider.step = 'any';
    this.slider.setAttribute('aria-label', 'How far the cut is moved along its plane’s normal');
    this.readout.className = 'sb-at';
    this.readout.setAttribute('aria-label', 'The cut’s offset, in the drawing’s unit');
    this.flip.textContent = 'Flip';
    this.flip.title = 'Keep the other side of the plane';
    const close = document.createElement('button');
    close.textContent = '×';
    close.title = 'Stop sectioning (⇧X)';
    close.setAttribute('aria-label', 'Stop sectioning');
    el.replaceChildren(name, this.plane, this.slider, this.readout, this.flip, close);
    const move = (offset: number) => {
      const s = this.view.section;
      if (s && Number.isFinite(offset)) this.view.setSection({ ...s, offset });
    };
    this.plane.addEventListener('change', () => {
      const s = this.view.section;
      if (s) this.view.setSection({ ...s, plane: this.plane.value, offset: 0 });
    });
    this.slider.addEventListener('input', () => move(Number(this.slider.value)));
    this.readout.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') { this.readout.blur(); this.refresh(); return; }
      if (e.key === 'Enter') move(Number(this.readout.value));
    });
    this.readout.addEventListener('blur', () => this.refresh());
    this.flip.addEventListener('click', () => {
      const s = this.view.section;
      if (s) this.view.setSection({ ...s, flip: !s.flip });
    });
    close.addEventListener('click', () => this.view.setSection(null));
  }

  /** Bring the strip up to the view's section: shown only while there is one. */
  refresh(): void {
    const s = this.view.section;
    this.el.hidden = !s;
    if (!s) return;
    const choices = this.view.planeChoices();
    if (!choices.includes(s.plane)) choices.push(s.plane);
    const key = choices.join('\n');
    if (key !== this.planes) {
      this.planes = key;
      this.plane.replaceChildren(...choices.map((n) => new Option(n, n)));
    }
    this.plane.value = s.plane;
    // the slider runs over the objects, the cut where it stands kept within it
    const frame = this.view.sectionFrame();
    const reach = frame && reachAlong(this.view.sketch, frame);
    const [lo, hi] = reach ?? [Math.min(s.offset, 0), Math.max(s.offset, 0)];
    this.slider.min = String(Math.min(lo, s.offset));
    this.slider.max = String(Math.max(hi, s.offset));
    this.slider.disabled = !reach;
    if (document.activeElement !== this.slider) this.slider.value = String(s.offset);
    if (document.activeElement !== this.readout) this.readout.value = fmt(s.offset);
    this.flip.setAttribute('aria-pressed', String(s.flip));
  }
}
