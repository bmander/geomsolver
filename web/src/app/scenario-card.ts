/* **The scenario chooser and its card** (#510): beside the plane chooser, where the document
 * states a load scenario, a chooser of them; once one is chosen, a card beneath it with the
 * exaggeration the picture is drawn at in its head, the colour ramp the moves are read by, what
 * the reading says, and a row for each line of the scenario — the line as written, and what the
 * core answered of a requirement.  A row puts the program editor's caret on its line.
 *
 * The page's own: every word is the core's (a mark's `text`, an answer's sentence) or the
 * document's (the line's own text, by its span), and choosing writes nothing. */
import type { ScenarioMark } from '../core/scenario.js';
import { goToSource } from './program.js';
import { RAMP, bare } from './scenario-box.js';
import type { SketchView } from './view.js';

const pick = document.getElementById('scenario-pick') as HTMLLabelElement;
const select = document.getElementById('scenario-select') as HTMLSelectElement;
const card = document.getElementById('scenario-card') as HTMLElement;
const slider = document.getElementById('scenario-scale') as HTMLInputElement;
const scaleText = document.getElementById('scenario-scale-text') as HTMLOutputElement;
const animate = document.getElementById('scenario-animate') as HTMLInputElement;
const meshed = document.getElementById('scenario-mesh') as HTMLInputElement;
const legend = document.getElementById('scenario-legend') as HTMLElement;
const legendHi = document.getElementById('scenario-legend-hi') as HTMLElement;
const rampEl = document.getElementById('scenario-ramp') as HTMLElement;
const status = document.getElementById('scenario-status') as HTMLElement;
const lines = document.getElementById('scenario-lines') as HTMLUListElement;

/** The slider's steps, end to end. */
const STEPS = 1000;

/** What a mark is drawn as, in a row: a glyph as the box draws it, near enough. */
const GLYPH: Record<ScenarioMark['kind'], string> = {
  fixed: '▣', pinned: '▲', load: '→', pressure: '⇉', accelerate: '⇣', requirement: '◆',
  plane: '▭',
};

/** **The slider's reach**: from true scale at its left end to ten times the picture's own factor
 *  (and at least ten) at its right, by equal ratios, so a factor of thousands and one near one
 *  are both in hand. */
function reach(view: SketchView): number {
  return Math.max(10, 10 * (view.picture?.scale ?? 1));
}

function factorAt(view: SketchView, t: number): number {
  return Math.pow(reach(view), t / STEPS);
}

function stepOf(view: SketchView, factor: number): number {
  return Math.round(STEPS * Math.log(Math.max(1, factor)) / Math.log(reach(view)));
}

/** A factor as the card says it: `×200`, `×1 (true scale)`. */
function factorText(f: number): string {
  const r = f >= 100 ? Math.round(f)
    : f >= 10 ? Math.round(f * 10) / 10 : Math.round(f * 100) / 100;
  return r === 1 ? '×1 (true scale)' : `×${r.toLocaleString('en-US')}`;
}

/** Wire the chooser and the card to the view. */
export function bindScenarioCard(view: SketchView): void {
  rampEl.style.background =
    `linear-gradient(to right, ${RAMP.map(([t, c]) => `${c} ${t * 100}%`).join(', ')})`;
  select.addEventListener('change', () => {
    view.chooseScenario(select.value);
    select.blur();                  // the keys are the drawing's again: a tool letter, Escape
  });
  slider.addEventListener('input', () => {
    view.scenarioScale = factorAt(view, Number(slider.value));
    scaleText.value = factorText(view.scenarioFactor);
    view.draw();
  });
  animate.addEventListener('change', () => view.setScenarioAnimate(animate.checked));
  meshed.addEventListener('change', () => {
    view.scenarioMesh = meshed.checked;
    view.draw();
  });
}

/** **Bring the chooser and the card up to the view**: the scenarios the document states, the one
 *  chosen, and — chosen — its card. */
export function refreshScenarioCard(view: SketchView): void {
  const names = view.scenarioNames();
  pick.hidden = names.length === 0;
  const options = ['', ...names.map((n) => n.name)];
  const shown = [...select.options].map((o) => o.value);
  if (shown.length !== options.length || options.some((n, i) => shown[i] !== n)) {
    select.replaceChildren(new Option('None', ''), ...names.map((n) =>
      new Option(n.extends ? `${n.name} : ${n.extends}` : n.name, n.name)));
  }
  select.value = view.scenario;
  const fails = view.scenarios.some((s) =>
    s.words.length || s.answers.some((a) => a.holds !== true));
  pick.classList.toggle('failed', fails);
  card.hidden = !view.scenario;
  if (!view.scenario) return;
  // the head: the exaggeration, at the card's factor or the picture's own
  const p = view.picture;
  const moves = !!p && p.largest > 0;
  slider.disabled = !moves;
  slider.value = String(stepOf(view, view.scenarioFactor));
  scaleText.value = moves ? factorText(view.scenarioFactor) : '';
  animate.checked = view.scenarioAnimate;
  meshed.checked = view.scenarioMesh;
  legend.hidden = !moves;
  legendHi.textContent = p ? p.largestText : '';
  legend.title =
    p ? `the largest move, ${p.largestText}, drawn ${factorText(view.scenarioFactor)}` : '';
  // what the reading says: still going, failed, a body not drawn, the scenario's own words
  const r = view.scenarioReading;
  // the rows show the reading as it stands: a coarser level's answers while a finer is read
  const sc = (r?.scenarios ?? view.scenarios).find((s) => s.name === view.scenario);
  const said: string[] = [];
  if (r?.error) said.push(r.error);
  else if (r?.reading) said.push(r.levelText ? `${r.levelText}…` : 'reading the scenario…');
  else if (p && !p.bodies.length && !p.words.length) said.push('nothing is read under it');
  else if (p && !moves) said.push('nothing is loaded, so nothing moves');
  const words = [...(p?.words ?? []), ...(sc?.words ?? [])].map(bare);
  status.textContent = [...said, ...new Set(words)].join('\n');
  status.className = r?.error || words.length ? 'failed' : '';
  // a row for each line, in the order written
  const text = view.doc.text;
  lines.replaceChildren(...view.scenarioMarks.map((m) => {
    const li = document.createElement('li');
    const glyph = document.createElement('span');
    glyph.className = 'glyph';
    glyph.textContent = GLYPH[m.kind];
    const body = document.createElement('span');
    const code = document.createElement('code');
    code.textContent = m.span ? text.slice(m.span[0], m.span[1]).trim() : m.text;
    body.append(code);
    let note = m.kind === 'load' || m.kind === 'pressure' || m.kind === 'accelerate' ? m.text : '';
    if (m.kind === 'requirement') {
      const a = sc?.answers.find((x) => x.index === m.index);
      // no answer at a coarser level is no verdict: the finer may have one
      if (a && !(r?.reading && a.holds === null)) {
        note = bare(a.text);
        li.classList.add(a.holds === true ? 'kept' : 'refuted');
        if (r?.reading) li.classList.add('provisional');
      } else if (r?.reading) {
        note = 'reading…';
      }
    }
    if (note) {
      const n = document.createElement('span');
      n.className = 'said';
      n.textContent = note;
      body.append(n);
    }
    li.append(glyph, body);
    li.title = m.span ? 'Go to the line it is written on' : '';
    const span = m.span;
    if (span) li.addEventListener('click', () => goToSource({ lo: span[0], hi: span[1] }));
    return li;
  }));
}
