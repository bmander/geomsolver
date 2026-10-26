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

type Heard = [number, number][];

test('a module compiled as it arrives says how much has, the last word every byte', async () => {
  const bytes = module(40_000);
  const heard: Heard = [];
  const instance = await fromResponse(
    sent(bytes, 8, { 'content-type': 'application/wasm' }),
    (got, total) => heard.push([got, total]), bytes.length);
  assert.ok(instance instanceof WebAssembly.Instance);
  assert.ok(heard.every(([, total]) => total === bytes.length), 'of the module\'s own size');
  assert.deepEqual(heard[0], [0, bytes.length], 'the bar starts empty');
  assert.ok(heard.every(([got], i) => i === 0 || got >= heard[i - 1][0]), 'and only fills');
  assert.ok(heard.length > 3, 'a chunk at a time');
  assert.deepEqual(heard.at(-1), [bytes.length, bytes.length], 'to full once every byte is in');
});

test('a size the page was not told is not guessed, and nothing is counted', async () => {
  const bytes = module(20_000);
  const heard: Heard = [];
  // a compressing host's Content-Length is the compressed size, not the module's
  await fromResponse(
    sent(bytes, 4, { 'content-type': 'application/wasm', 'content-encoding': 'gzip',
                     'content-length': '3000' }),
    (got, total) => heard.push([got, total]));
  assert.deepEqual(heard, []);
});

test('a host that does not say application/wasm still gets the module compiled', async () => {
  const bytes = module(1_000);
  const instance = await fromResponse(sent(bytes, 2, { 'content-type': 'application/octet-stream' }));
  assert.ok(instance instanceof WebAssembly.Instance);
});
