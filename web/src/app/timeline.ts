/* **The timeline** (#398 L): a row per parameter the drawing declares, its interval a slider.
 *
 * Scrubbing it shows the drawing at that instant (`SketchView.showInstant`), on the branch the
 * core's march carries; past where the march stopped the track is hatched, and the singular
 * instant it stopped at is marked with what the core says it is.  The readout takes an instant as
 * written (`90deg`), read by the core; Run steps the drawing along the interval, and Home puts it
 * back where it is drawn.  All of it is presentation: the instant is view state, and an edit is
 * made at home.  Nothing here reads a number the core did not hand it. */
import { instant } from '../core/parameters.js';
import type { Parameter } from '../core/parameters.js';
import type { SketchView } from './view.js';

/** How long Run takes over the whole of an interval. */
const RUN_MS = 4000;

interface Row {
  p: Parameter;
  el: HTMLElement;
  slider: HTMLInputElement;
  reach: HTMLElement;
  mark: HTMLElement;
  readout: HTMLInputElement;
  play: HTMLButtonElement;
}

export class Timeline {
  private rows: Row[] = [];
  /** What the rows were built for: each parameter's name and interval. */
  private built = '';
  /** The parameter Run is stepping, the frame it asked for and when it last stepped. */
  private running: { name: string; frame: number; last: number } | null = null;

  constructor(readonly el: HTMLElement, readonly view: SketchView) {}

  /** Bring the rows up to the view's parameters, their marches and the instant shown. */
  refresh(): void {
    const ps = this.view.parameters();
    const key = JSON.stringify(ps.map((p) => [p.name, p.from, p.to]));
    if (key !== this.built) {
      this.stop();
      this.built = key;
      this.el.replaceChildren();
      this.rows = ps.map((p) => this.row(p));
    }
    this.el.hidden = this.rows.length === 0;
    this.updateAll();
  }

  private updateAll(): void {
    for (const r of this.rows) this.update(r);
  }

  private row(p: Parameter): Row {
    const el = document.createElement('div');
    el.className = 'tl-row';
    const name = document.createElement('span');
    name.className = 'tl-name';
    name.textContent = p.name;
    const track = document.createElement('div');
    track.className = 'tl-track';
    const reach = document.createElement('div');
    reach.className = 'tl-reach';
    const mark = document.createElement('div');
    mark.className = 'tl-singular';
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(p.from);
    slider.max = String(p.to);
    slider.step = 'any';
    slider.setAttribute('aria-label', `${p.name}: the instant shown`);
    track.append(reach, mark, slider);
    const readout = document.createElement('input');
    readout.className = 'tl-at';
    readout.setAttribute('aria-label', `${p.name}: an instant, as written`);
    const play = document.createElement('button');
    play.className = 'tl-play';
    const home = document.createElement('button');
    home.className = 'tl-home';
    home.textContent = 'Home';
    home.title = 'Back to where the drawing is drawn: the interval’s start';
    el.append(name, track, readout, play, home);
    this.el.append(el);
    const r: Row = { p, el, slider, reach, mark, readout, play };
    slider.addEventListener('input', () => {
      this.stop();
      this.view.showInstant(p.name, Number(slider.value));
    });
    readout.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') { readout.blur(); this.update(r); return; }
      if (e.key !== 'Enter') return;
      const got = instant(this.view.sketch, p.name, readout.value);
      if ('error' in got) this.view.onStatus(got.error);
      else { this.stop(); this.view.showInstant(p.name, got.at); }
    });
    readout.addEventListener('blur', () => this.update(r));
    play.addEventListener('click', () => this.running?.name === p.name ? this.stop()
                                                                       : this.run(p.name));
    home.addEventListener('click', () => { this.stop(); this.view.showInstant(p.name, null); });
    return r;
  }

  private update(r: Row): void {
    const { p } = r;
    const m = this.view.marchOf(p.name);
    const span = p.to - p.from;
    const frac = (v: number) => `${(100 * (v - p.from)) / span}%`;
    const shown = this.view.instant?.name === p.name ? this.view.instant.at : null;
    const at = shown ?? p.from;
    r.el.title = m?.text ?? '';
    r.el.classList.toggle('tl-off', !m?.followed);
    r.slider.disabled = !m?.followed;
    if (document.activeElement !== r.slider) r.slider.value = String(at);
    if (document.activeElement !== r.readout) {
      r.readout.value = this.view.reading(p.name, at);
    }
    r.el.classList.toggle('tl-shown', shown !== null);
    const reached = m?.followed ? m.reached : p.from;
    r.reach.style.width = frac(reached);
    const singular = m?.followed ? m.singular : null;
    r.mark.hidden = !singular;
    if (singular) {
      r.mark.style.left = frac(singular.at);
      r.mark.title = singular.text;
    }
    const running = this.running?.name === p.name;
    r.play.textContent = running ? 'Pause' : 'Run';
    r.play.title = running ? 'Stop the drawing here'
      : `Run the drawing along ${p.name}, from ${p.fromText} to ${m?.followed ? m.reachedText
                                                                                 : p.toText}`;
    r.play.disabled = !m?.followed;
  }

  /** Run the drawing along `name`, round again from the start once it reaches the end. */
  private run(name: string): void {
    this.stop();
    const tick = (now: number) => {
      const run = this.running;
      const p = this.rows.find((r) => r.p.name === name)?.p;
      const m = this.view.marchOf(name);
      if (!run || !p || !m?.followed) return this.stop();
      const step = (Math.max(0, now - run.last) / RUN_MS) * (p.to - p.from);
      run.last = now;
      const from = this.view.instant?.name === name ? this.view.instant.at ?? p.from : p.from;
      const next = from + step > m.reached ? p.from : from + step;
      if (!this.view.showInstant(name, next)) return this.stop();
      run.frame = requestAnimationFrame(tick);
    };
    this.running = { name, frame: requestAnimationFrame(tick), last: performance.now() };
    this.updateAll();
  }

  /** Stop Run, wherever it has got to. */
  stop(): void {
    if (!this.running) return;
    cancelAnimationFrame(this.running.frame);
    this.running = null;
    this.updateAll();
  }
}
