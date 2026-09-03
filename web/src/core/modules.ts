/* Modules a document `use`s, handed to the core by the host.
 *
 * The core takes text and has no filesystem: in the terminal `solventc` reads `engine.parts` as
 * `engine/parts.sv` beside the document and comes to the library compiled in second.  A browser
 * has no beside — unless its page has a server to ask.  `link` is that: it asks the core which
 * modules a text uses, fetches each through the function it is given, hands the text over, and
 * follows the fetched texts' own `use`s the same way.  What the host has not got is left to the
 * library, so a document of the compiled-in kind links as it always did.  No resolution happens
 * here: the core links, and the same way it links for the CLI. */
import { core, takeJson, withStr } from './wasm.js';

/** The module names a text `use`s directly, as written (`engine.parts`). */
export function uses(text: string): string[] {
  return withStr(text, (p, n) => takeJson<string[]>(core().gcs_program_uses(p, n))) ?? [];
}

/** Hand the core a module's text under its `use` name; it outranks the library's copy. */
export function provide(name: string, text: string): void {
  withStr(name, (np, nn) => withStr(text, (tp, tn) => core().gcs_module_set(np, nn, tp, tn)));
}

/** Forget everything handed over: the next program read links against the library alone. */
export function forget(): void {
  core().gcs_module_forget();
}

/** A `use` name as the path beside the document: `engine.parts` is `engine/parts.sv`. */
export function pathOf(name: string): string {
  return name.split('.').join('/') + '.sv';
}

/** Fetch and hand over every module `text` reaches, through `fetchText(path)` — `null` for one
 *  the host has not got, which is then the library's.  Each name is asked for once, however many
 *  texts use it.  Returns the names handed over. */
export async function link(
  text: string,
  fetchText: (path: string) => Promise<string | null>,
): Promise<string[]> {
  const got: string[] = [];
  const asked = new Set<string>();
  const queue = uses(text);
  while (queue.length) {
    const name = queue.shift()!;
    if (asked.has(name)) continue;
    asked.add(name);
    const t = await fetchText(pathOf(name));
    if (t === null) continue;
    provide(name, t);
    got.push(name);
    queue.push(...uses(t));
  }
  return got;
}
