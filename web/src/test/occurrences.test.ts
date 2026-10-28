/* The name under the caret, lit wherever else the text names it (`app/occurrences.ts`), over the
 * core's name index (`core/program.ts::symbols`): the index crossed onto the string the browser
 * holds, and what a caret or a selection means, lights, steps to and goes to the definition of. */
import assert from 'node:assert/strict';
import test from 'node:test';

import { symbols } from '../core/program.js';
import { definitionOf, meant, occurrenceMarks, stepTo } from '../app/occurrences.js';
import { initCore } from '../core/wasm.js';

await initCore();

/** `text` with every mark written into it: `«given»`, `‹read›`. */
function marked(text: string, lo: number, hi = lo): string {
  let out = '', end = 0;
  for (const m of occurrenceMarks(symbols(text), lo, hi)) {
    const [open, close] = m.cls === 'occurrence-def' ? ['«', '»'] : ['‹', '›'];
    out += text.slice(end, m.lo) + open + text.slice(m.lo, m.hi) + close;
    end = m.hi;
  }
  return out + text.slice(end);
}

const RUNG = 'component Rung(a: point, b: point, len: Length) {\n'
  + '  mid := point\n  a distance(len) mid\n}\n'
  + 'p := point\nq := point\nt0 := Rung(p, q, len: 50)\nt0.mid distance(3) q\n';

test('the caret in a name lights every place it is named, given and read', () => {
  const at = RUNG.indexOf('len)');
  assert.equal(marked(RUNG, at + 1),
    'component Rung(a: point, b: point, «len»: Length) {\n'
    + '  mid := point\n  a distance(‹len›) mid\n}\n'
    + 'p := point\nq := point\nt0 := Rung(p, q, ‹len›: 50)\nt0.mid distance(3) q\n');
  // a member read through the instance is the one the component declares
  assert.equal(marked(RUNG, RUNG.indexOf('t0.mid') + 4),
    'component Rung(a: point, b: point, len: Length) {\n'
    + '  «mid» := point\n  a distance(len) ‹mid›\n}\n'
    + 'p := point\nq := point\nt0 := Rung(p, q, len: 50)\nt0.‹mid› distance(3) q\n');
});

test('the caret just past a name means it; a space, a keyword or more than it, none', () => {
  const index = symbols(RUNG);
  const end = RUNG.indexOf(' := Rung');
  assert.equal(meant(index, end, end)?.lo, RUNG.indexOf('t0 := '), 't0| is t0');
  assert.equal(meant(index, end + 1, end + 1), null, 'past the space is not');
  assert.equal(meant(index, 2, 2), null, '`component` names nothing');
  const t0 = RUNG.indexOf('t0 := ');
  assert.equal(meant(index, t0, t0 + 2)?.lo, t0, 'the name selected whole is the name');
  assert.equal(meant(index, t0, t0 + 4), null, 'more than the name is no name');
  assert.deepEqual(occurrenceMarks(index, end + 1, end + 1), []);
});

test('offsets cross onto the string past a character wider in UTF-8', () => {
  // the em dash is three bytes and one string index: every offset after it is the string's
  const text = '// a pin — and its ring\npin := point\nring := circle(center: pin)\n';
  const at = text.lastIndexOf('pin');
  assert.equal(marked(text, at),
    '// a pin — and its ring\n«pin» := point\nring := circle(center: ‹pin›)\n');
  for (const o of symbols(text).occurrences) {
    assert.equal(text.slice(o.lo, o.hi), symbols(text).symbols[o.symbol].name);
  }
});

test('F7 steps to the next place the name is named and ⇧F7 to the last, round the ends', () => {
  const text = 'a := point\nb := point\nl := line(a, b)\na distance(5) b\n';
  const index = symbols(text);
  const all = [...text.matchAll(/\ba\b/g)].map((m) => m.index!);
  assert.equal(all.length, 3);
  assert.equal(stepTo(index, all[0], all[0], 1)?.lo, all[1]);
  assert.equal(stepTo(index, all[1] + 1, all[1] + 1, 1)?.lo, all[2], 'from the end of one too');
  assert.equal(stepTo(index, all[2], all[2], 1)?.lo, all[0], 'round the end');
  assert.equal(stepTo(index, all[0], all[0], -1)?.lo, all[2], 'and back round the start');
  assert.equal(stepTo(index, text.indexOf('point'), text.indexOf('point'), 1), null);
});

test('Go to definition puts the caret where the name under it is given', () => {
  const index = symbols(RUNG);
  // a formal read in the body and filled by label at the call, both to the formal
  const formal = RUNG.indexOf('len: Length');
  assert.equal(definitionOf(index, RUNG.indexOf('len)') + 1, RUNG.indexOf('len)') + 1)?.lo,
    formal);
  const label = RUNG.indexOf('len: 50');
  assert.equal(definitionOf(index, label, label)?.lo, formal, 'a call\'s label is its formal');
  // a member read through the instance, to the declaration in the component
  const t0mid = RUNG.indexOf('t0.mid') + 3;
  assert.equal(definitionOf(index, t0mid, t0mid + 3)?.lo, RUNG.indexOf('mid := point'));
  // the instance, to where it is bound; standing there, it stays
  const t0 = RUNG.indexOf('t0 := ');
  assert.equal(definitionOf(index, t0mid - 2, t0mid - 2)?.lo, t0);
  assert.equal(definitionOf(index, t0, t0)?.lo, t0, 'given once: on it is where it is given');
  // a component called, to its header
  const call = RUNG.indexOf('Rung(p');
  assert.equal(definitionOf(index, call, call)?.lo, RUNG.indexOf('Rung(a'));
});

test('Go to definition has nowhere to go from what the text does not give', () => {
  const text = 'l := line\np := point\nl.p1 distance(3) p\nq distance(1) p\n';
  const index = symbols(text);
  const p1 = text.indexOf('p1');
  assert.equal(definitionOf(index, p1, p1), null, 'a member of a line nothing declares');
  assert.equal(definitionOf(index, text.indexOf('q '), text.indexOf('q ')), null, 'unknown');
  assert.equal(definitionOf(index, text.indexOf('line'), text.indexOf('line')), null, 'a type');
  assert.equal(definitionOf(index, text.indexOf(' := '), text.indexOf(' := ') + 1), null);
});
