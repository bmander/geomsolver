/* Loading the core: a module compiled as it arrives, and the page told how much of it has.
 *
 * The module here is an empty one with a custom section for bulk, sent in chunks the way a
 * network delivers it — what is under test is the reading, not the core. */
import assert from 'node:assert/strict';
import test from 'node:test';

import { fromResponse } from '../core/wasm.js';

/** An empty module carrying `n` bytes of a custom section named `x`. */
function module(n: number): Uint8Array {
  const leb: number[] = [];
  for (let v = n + 2; ; v >>>= 7) {
    if (v < 0x80) { leb.push(v); break; }
    leb.push((v & 0x7f) | 0x80);
  }
  return new Uint8Array([0, 97, 115, 109, 1, 0, 0, 0, 0, ...leb, 1, 120, ...new Uint8Array(n)]);
}

/** A response delivering `bytes` in `chunks` pieces, with `headers`. */
function sent(bytes: Uint8Array, chunks: number, headers: Record<string, string>): Response {
  const step = Math.ceil(bytes.length / chunks);
  let at = 0;
  const body = new ReadableStream<Uint8Array>({
    pull(c) {
      if (at >= bytes.length) { c.close(); return; }
      c.enqueue(bytes.slice(at, at += step));
    },
  });
  return new Response(body, { headers });
}

type Heard = [string, number | undefined][];

test('a module compiled as it arrives says how much has, then that it is starting', async () => {
  const bytes = module(40_000);
  const heard: Heard = [];
  const instance = await fromResponse(
    sent(bytes, 8, { 'content-type': 'application/wasm' }),
    (what, done) => heard.push([what, done]), bytes.length);
  assert.ok(instance instanceof WebAssembly.Instance);
  const fractions = heard.map(([, f]) => f).filter((f): f is number => f !== undefined);
  assert.equal(fractions[0], 0, 'the bar starts empty');
  assert.ok(fractions.every((f, i) => i === 0 || f >= fractions[i - 1]), 'and only fills');
  assert.equal(fractions.at(-1), 1, 'to full once every byte is in');
  assert.ok(heard.some(([w]) => /^downloading the solver — 0\.0\d? of 0\.0\d? MB$/.test(w)), 'in MB');
  assert.deepEqual(heard.at(-1), ['starting the solver…', undefined]);
});

test('a compressed response\'s length is not the module\'s, so it is not taken for one', async () => {
  const bytes = module(20_000);
  const heard: Heard = [];
  await fromResponse(
    sent(bytes, 4, { 'content-type': 'application/wasm', 'content-encoding': 'gzip',
                     'content-length': '3000' }),
    (what, done) => heard.push([what, done]));
  assert.ok(heard.every(([, f]) => f === undefined), 'a sweep, not a fraction past one');
  // one with nothing compressed says its own size
  const plain: Heard = [];
  await fromResponse(
    sent(bytes, 4, { 'content-type': 'application/wasm', 'content-length': String(bytes.length) }),
    (what, done) => plain.push([what, done]));
  assert.equal(plain.filter(([, f]) => f !== undefined).at(-1)?.[1], 1);
});

test('a host that does not say application/wasm still gets the module compiled', async () => {
  const bytes = module(1_000);
  const instance = await fromResponse(sent(bytes, 2, { 'content-type': 'application/octet-stream' }));
  assert.ok(instance instanceof WebAssembly.Instance);
});
