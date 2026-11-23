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
 *  `digest` names its text; `at` is the path the host handed it over with (`foundAt`, '' where it
 *  gave none or the text is the library's) and `path` the one it is shown by. */
export interface SourceFile {
  name: string; path: string; at: string; text: string; provided: boolean; digest: string;
}

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
    const at = foundAt(name) ?? '';
    files.push({ name, path: at || pathOf(name, text), at, text, provided: provided(name), digest: d });
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
 *  where the host found it (`parts/bracket.step`), or none for a text told by its content alone: a
 *  file found by an exchange file's name is one to the core whatever its text, so one that is none
 *  is refused by the kernel's reader at the `use` (`modules::Found::at`). */
export function provide(name: string, text: string, path = ''): void {
  installed = null;
  withStr(name, (np, nn) => withStr(path, (pp, pn) => withStr(text, (tp, tn) =>
    core().gcs_module_set(np, nn, pp, pn, tp, tn))));
}

/** Whether the host handed module `name` over, rather than a `use` of it reading the library. */
export function provided(name: string): boolean {
  return withStr(name, (np, nn) => core().gcs_module_provided(np, nn)) === 1;
}

/** The path the host handed module `name` over with (`gcs_module_path`; '' where it gave none),
 *  or null where it handed none over. */
export function foundAt(name: string): string | null {
  const p = withStr(name, (np, nn) => core().gcs_module_path(np, nn));
  return p ? takeStr(p) : null;
}

/** A module handed to a worker: its `use` name, its digest, the path the page was handed it with
 *  (a file found as `NAME.step` is one whatever it holds), and its text — left out where the
 *  worker has it already (`ModuleOutbox`), so a file of megabytes crosses to a worker once. */
export type Handed = [name: string, digest: string, path: string, text?: string];

/** The modules handed over that `text` reaches, as `[use name, digest, path, text]`: what a worker
 *  is given to read the page's drawing as the page reads it (`install`); the library's own it has
 *  already. */
export function handed(text: string): Handed[] {
  return related(text).filter((f) => f.provided).map((f): Handed => [f.name, f.digest, f.at, f.text]);
}

/** **The page's account of one worker's modules**, made with the worker: the digests of the last
 *  list it was sent, whose texts the worker keeps (`received`).  A worker made afresh is given a
 *  fresh one, so every text is sent it again. */
export class ModuleOutbox {
  private sent = new Set<string>();

  /** `list` as this worker is sent it: a text it was sent last time, by its digest, left out. */
  send(list: Handed[]): Handed[] {
    const out = list.map(([name, d, path, text]): Handed =>
      (this.sent.has(d) ? [name, d, path] : [name, d, path, text]));
    this.sent = new Set(list.map(([, d]) => d));
    return out;
  }
}

/** What tells two lists of modules apart: their names, digests and paths, never their texts. */
export function modulesKey(list: Handed[]): string {
  return list.map(([name, d, path]) => `${name} ${d} ${path}`).join('\n');
}

/** The texts a worker was sent, by digest: those of the last list it received, as the page's
 *  `ModuleOutbox` keeps its account of it. */
let held = new Map<string, string>();
/** The list the worker's core has now, by `modulesKey`. */
let installed: string | null = null;

/** `list` as a worker receives it, its texts left out (`ModuleOutbox`) put back from the last
 *  list it received.  Called as each message arrives, in the order the page sent them, so the worker
 *  holds what the page counts it as holding, whatever it does with the job after. */
export function received(list: Handed[]): Handed[] {
  const texts = new Map<string, string>();
  const out = list.map(([name, d, path, text]): Handed => {
    const t = text ?? held.get(d);
    if (t === undefined) throw new Error(`module \`${name}\` was named without its text`);
    texts.set(d, t);
    return [name, d, path, t];
  });
  held = texts;
  return out;
}

/** A job's modules taken as a worker receives it (`received`, put back in place), or why not
 *  handed to `refuse`, the job's own refusal: false then, and the job is not done. */
export function receive(job: { modules: Handed[] }, refuse: (why: string) => void): boolean {
  try {
    job.modules = received(job.modules);
    return true;
  } catch (e) {
    refuse(String(e));
    return false;
  }
}

/** Replace everything handed over with `list` (`received`, its texts in it): a worker given the
 *  page's modules, each with the path the page was handed it with.  A list the core has already
 *  is not handed over again. */
export function install(list: Handed[]): void {
  const key = modulesKey(list);
  if (key === installed) return;
  forget();
  for (const [name, , path, text] of list) provide(name, text!, path);
  installed = key;
}

/** Forget everything handed over: the next program read links against the library alone. */
export function forget(): void {
  core().gcs_module_forget();
  installed = null;
}

/** A `use` name as the path beside the document: `engine.parts` is `engine/parts.sv`, or, for
 *  a module whose `text` is an exchange file, `engine/parts.step` — the core's first candidates
 *  of each kind (`searchPaths`). */
export function pathOf(name: string, text?: string): string {
  return searchPaths(name, '')[text !== undefined && isStep(text) ? 1 : 0];
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
