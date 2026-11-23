/* Modules a document `use`s, handed to the core by the host.
 *
 * The core takes text and has no filesystem: in the terminal `solventc` reads `engine.parts` as
 * `engine/parts.sv` (or `engine/parts.step`, `engine/parts.stp`: a file that gives a body, #993
 * stage 2) beside the document and comes to the library compiled in second.  Which paths, in
 * which order, is the core's answer (`searchPaths`), and so is whether a text is an exchange file
 * (`isStep`): the binding carries the texts and decides neither.  A browser has no beside —
 * unless its page has a server to ask.  `link` is that: it asks the core which
 * modules a text uses, fetches each through the function it is given, hands the text over, and
 * follows the fetched texts' own `use`s the same way.  What the host has not got is left to the
 * library, so a document of the compiled-in kind links as it always did.  No resolution happens
 * here: the core links, and the same way it links for the CLI. */
import { core, takeJson, takeStr, withStr } from './wasm.js';

/** The effective source, including modules supplied by the host and shared libraries. */
export function source(name: string): string | null {
  const p = withStr(name, (np, nn) => core().gcs_module_source(np, nn));
  return p ? takeStr(p) : null;
}

/** The digest of the text a `use` of `name` reads (`gcs_module_digest`, the core's
 *  `modules::digest`), or null for a module there is none of: what names a text of megabytes
 *  without copying it out of the core. */
export function digest(name: string): string | null {
  const p = withStr(name, (np, nn) => core().gcs_module_digest(np, nn));
  return p ? takeStr(p) : null;
}

/** A module a text reaches: `provided` when the host handed it over, and not the library's;
 *  `digest` names its text. */
export interface SourceFile { name: string; path: string; text: string; provided: boolean; digest: string }

/** The texts `related` read last, by digest: a text the core holds unchanged is not copied out of
 *  it again (a given module's file may be megabytes, and the page asks on every edit). */
let lastTexts = new Map<string, string>();

/** All imports reachable from a document, once each, in dependency traversal order. */
export function related(text: string): SourceFile[] {
  const files: SourceFile[] = [];
  const seen = new Set<string>();
  const texts = new Map<string, string>();
  const queue = uses(text);
  for (let i = 0; i < queue.length; i++) {
    const name = queue[i];
    if (seen.has(name)) continue;
    seen.add(name);
    const d = digest(name);
    const text = d === null ? null : lastTexts.get(d) ?? texts.get(d) ?? source(name);
    if (d === null || text === null) continue;
    texts.set(d, text);
    files.push({ name, path: pathOf(name, text), text, provided: provided(name), digest: d });
    queue.push(...uses(text));
  }
  lastTexts = texts;
  return files;
}

/** The module names a text `use`s directly, as written (`engine.parts`).  An exchange file uses
 *  none, and is not handed to the core to say so: it may be megabytes, and it is not Solvent. */
export function uses(text: string): string[] {
  if (isStep(text)) return [];
  return withStr(text, (p, n) => takeJson<string[]>(core().gcs_program_uses(p, n))) ?? [];
}

/** How much of a text the core is shown to tell an exchange file by: `modules::is_step` reads
 *  the first token only, after a byte-order mark and white space. */
const HEAD = 1024;

/** Whether a module's text is an ISO 10303-21 exchange file, which gives a body, rather than
 *  Solvent (#993 stage 2): the core's test, asked of the text's head alone. */
export function isStep(text: string): boolean {
  return withStr(text.slice(0, HEAD), (p, n) => core().gcs_text_is_step(p, n)) === 1;
}

/** Hand the core a module's text under its `use` name; it outranks the library's copy.  `path` is
 *  where the host found it (`parts/bracket.step`), or none for a text handed on (a worker given
 *  the page's): a file found by an exchange file's name is one to the core whatever its text, so
 *  one that is none is refused by the kernel's reader at the `use` (`modules::decode`). */
export function provide(name: string, text: string, path = ''): void {
  installed = null;
  withStr(name, (np, nn) => withStr(path, (pp, pn) => withStr(text, (tp, tn) =>
    core().gcs_module_set(np, nn, pp, pn, tp, tn))));
}

/** Whether the host handed module `name` over, rather than a `use` of it reading the library. */
export function provided(name: string): boolean {
  return withStr(name, (np, nn) => core().gcs_module_provided(np, nn)) === 1;
}

/** A module handed to a worker: its `use` name, its digest, and its text — left out where the
 *  worker has it already (`sending`), so a file of megabytes crosses to a worker once. */
export type Handed = [name: string, digest: string, text?: string];

/** The modules handed over that `text` reaches, as `[use name, digest, text]`: what a worker is
 *  given to read the page's drawing as the page reads it (`install`); the library's own it has
 *  already. */
export function handed(text: string): Handed[] {
  return related(text).filter((f) => f.provided).map((f): Handed => [f.name, f.digest, f.text]);
}

/** `list` as one worker is sent it: a text it was sent last time, by its digest, left out.  `sent`
 *  is that worker's — the digests of the last list it was sent, which `install` keeps the texts
 *  of — and becomes this list's. */
export function sending(sent: Set<string>, list: Handed[]): Handed[] {
  const out = list.map(([name, d, text]): Handed => (sent.has(d) ? [name, d] : [name, d, text]));
  sent.clear();
  for (const [, d] of list) sent.add(d);
  return out;
}

/** What tells two lists of modules apart: their names and digests, never their texts. */
export function modulesKey(list: Handed[]): string {
  return list.map(([name, d]) => `${name} ${d}`).join('\n');
}

/** The texts a worker was sent, by digest: those of the last list it received, as `sending` keeps
 *  the page's account of it. */
let held = new Map<string, string>();
/** The list the worker's core has now, by `modulesKey`. */
let installed: string | null = null;

/** `list` as a worker receives it, its texts left out (`sending`) put back from the last list it
 *  received.  Called as each message arrives, in the order the page sent them, so the worker
 *  holds what the page counts it as holding, whatever it does with the job after. */
export function received(list: Handed[]): Handed[] {
  const texts = new Map<string, string>();
  const out = list.map(([name, d, text]): Handed => {
    const t = text ?? held.get(d);
    if (t === undefined) throw new Error(`module \`${name}\` was named without its text`);
    texts.set(d, t);
    return [name, d, t];
  });
  held = texts;
  return out;
}

/** Replace everything handed over with `list` (`received`, its texts in it): a worker given the
 *  page's modules.  A list the core has already is not handed over again. */
export function install(list: Handed[]): void {
  const key = modulesKey(list);
  if (key === installed) return;
  forget();
  for (const [name, d, text] of list) {
    if (text === undefined) throw new Error(`module \`${name}\` (${d}) was named without its text`);
    provide(name, text);
  }
  installed = key;
}

/** Forget everything handed over: the next program read links against the library alone. */
export function forget(): void {
  core().gcs_module_forget();
  installed = null;
}

/** A `use` name as the path beside the document: `engine.parts` is `engine/parts.sv`, or, for
 *  a module whose `text` is an exchange file, `engine/parts.step`. */
export function pathOf(name: string, text?: string): string {
  return name.split('.').join('/') + (text !== undefined && isStep(text) ? '.step' : '.sv');
}

/** Project paths a `use` of `name` in the model at `root` reads, the first the host has winning:
 *  `name.sv`, `name.step`, `name.stp` beside the model, then in each of its ancestors, nearest
 *  first (`modules::search_paths`, the CLI's order). */
export function searchPaths(name: string, root: string): string[] {
  return withStr(name, (np, nn) => withStr(root, (sp, sn) =>
    takeJson<string[]>(core().gcs_module_search_paths(np, nn, sp, sn)))) ?? [];
}

/** Provide current project texts when opening a model or an individual component. */
export function provideProject(root: string, files: Record<string, string>): void {
  forget();
  const found: [string, string, string][] = [];
  const seen = new Set<string>(), queue = uses(files[root] ?? '');
  for (let i = 0; i < queue.length; i++) {
    const name = queue[i];
    if (seen.has(name)) continue;
    seen.add(name);
    const path = searchPaths(name, root).find((candidate) => candidate in files);
    if (path !== undefined) found.push([name, files[path], path]);
    // what a `use` of it will read: the project's copy, or (nothing handed over yet) the library's
    const text = path !== undefined ? files[path] : source(name);
    if (text !== null) queue.push(...uses(text));
  }
  for (const [name, text, path] of found) provide(name, text, path);
}

/** Fetch and hand over every module `text` reaches, through `fetchText(path)` — `null` for one
 *  the host has not got, which is then the library's.  Each name is asked for once, however many
 *  texts use it, at each of its paths beside the document (`searchPaths`) until one answers; an
 *  exchange file is handed over as it came.  Returns the names handed over. */
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
    let t: string | null = null, at = '';
    for (const path of searchPaths(name, '')) if ((t = await fetchText((at = path))) !== null) break;
    if (t === null) continue;
    provide(name, t, at);
    got.push(name);
    queue.push(...uses(t));
  }
  return got;
}
