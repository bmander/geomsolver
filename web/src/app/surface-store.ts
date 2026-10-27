/* **Built objects remembered across openings** (#673): a store of what a pool worker hands back
 * for a job — the build's buffer (`Display::bytes`: its mesh, faces, edges and volume) — keyed by
 * the core's build identity and the job's key.
 *
 * A job's key is a digest of everything its solid reads, equal on every core exactly when the
 * surface is (`Sketch::object_jobs`), so a build kept is the build a worker would make — by the
 * same core.  So the key is prefixed by the core's identity (`coreIdentity`, the module's digest
 * written in at bundling): a new core never reads an old core's surfaces.  The store is bounded,
 * the least recently used going past its limit, and can be cleared.  It is view state: nothing in
 * it is written into the document, its drawing or an export. */

/** Where builds are kept between openings. */
export interface SurfaceStore {
  /** The builds held for these job keys (a key held none is absent), each marked used now. */
  get(keys: string[]): Promise<Map<string, Uint8Array>>;
  /** Keep the build of a job key, the least recently used going past the store's limit. */
  put(key: string, display: Uint8Array): Promise<void>;
  /** Forget every build. */
  clear(): Promise<void>;
  /** How many bytes are held. */
  size(): Promise<number>;
}

/** The store's limit with none given: the builds of a few large assemblies. */
export const STORE_LIMIT = 200 * 1024 * 1024;

/** **A store in memory**, bounded and least-recently-used as the persistent one is: what a test
 *  opens a document twice with, and a page with no IndexedDB. */
export class MemoryStore implements SurfaceStore {
  private held = new Map<string, Uint8Array>();
  constructor(private readonly limit = STORE_LIMIT) {}

  async get(keys: string[]): Promise<Map<string, Uint8Array>> {
    const out = new Map<string, Uint8Array>();
    for (const k of keys) {
      const b = this.held.get(k);
      if (!b) continue;
      // a Map keeps the order put: used again, put last
      this.held.delete(k);
      this.held.set(k, b);
      out.set(k, b);
    }
    return out;
  }

  async put(key: string, display: Uint8Array): Promise<void> {
    this.held.delete(key);
    this.held.set(key, display);
    let total = await this.size();
    for (const [k, b] of this.held) {
      if (total <= this.limit) break;
      this.held.delete(k);
      total -= b.byteLength;
    }
  }

  async clear(): Promise<void> { this.held.clear(); }

  async size(): Promise<number> {
    let n = 0;
    for (const b of this.held.values()) n += b.byteLength;
    return n;
  }
}

const DB = 'gcs-built-objects';
/** The builds' bytes, by key; and beside them what the bound reads — each build's size and when it
 *  was last used (`Date.now()`) — so neither a hit nor the bound reads or writes bytes it does not
 *  hand back. */
const BYTES = 'bytes';
const META = 'meta';

interface Meta {
  key: string;
  size: number;
  used: number;
}

const done = <T>(r: IDBRequest<T>): Promise<T> => new Promise((resolve, reject) => {
  r.onsuccess = () => resolve(r.result);
  r.onerror = () => reject(r.error);
});

const finished = (t: IDBTransaction): Promise<void> => new Promise((resolve, reject) => {
  t.oncomplete = () => resolve();
  t.onerror = () => reject(t.error);
  t.onabort = () => reject(t.error);
});

/** **The page's store, in IndexedDB**: each build under `identity/key`, least recently used out
 *  past `limit` bytes. */
export class IdbStore implements SurfaceStore {
  private db: Promise<IDBDatabase>;

  constructor(private readonly identity: string, private readonly limit = STORE_LIMIT,
              factory: IDBFactory = indexedDB) {
    this.db = new Promise((resolve, reject) => {
      const open = factory.open(DB, 2);
      open.onupgradeneeded = () => {
        const db = open.result;
        // (a first version kept bytes and their use together: its builds are let go)
        for (const name of [...db.objectStoreNames]) db.deleteObjectStore(name);
        db.createObjectStore(BYTES);
        db.createObjectStore(META, { keyPath: 'key' }).createIndex('used', 'used');
      };
      open.onsuccess = () => resolve(open.result);
      open.onerror = () => reject(open.error);
    });
  }

  private named(key: string): string { return `${this.identity}/${key}`; }

  async get(keys: string[]): Promise<Map<string, Uint8Array>> {
    const out = new Map<string, Uint8Array>();
    if (!keys.length) return out;
    const db = await this.db;
    const t = db.transaction([BYTES, META], 'readwrite');
    const [bytes, meta] = [t.objectStore(BYTES), t.objectStore(META)];
    const now = Date.now();
    await Promise.all(keys.map(async (k) => {
      const kept = await done(bytes.get(this.named(k))) as Uint8Array | undefined;
      if (!kept) return;
      out.set(k, kept);
      meta.put({ key: this.named(k), size: kept.byteLength, used: now } satisfies Meta);
    }));
    await finished(t);
    return out;
  }

  async put(key: string, display: Uint8Array): Promise<void> {
    const db = await this.db;
    const t = db.transaction([BYTES, META], 'readwrite');
    const [bytes, meta] = [t.objectStore(BYTES), t.objectStore(META)];
    bytes.put(display, this.named(key));
    meta.put({ key: this.named(key), size: display.byteLength, used: Date.now() } satisfies Meta);
    // the least recently used out, past the limit: every build's size, oldest first
    const all = await done(meta.index('used').getAll()) as Meta[];
    let total = all.reduce((n, m) => n + m.size, 0);
    for (const m of all) {
      if (total <= this.limit) break;
      bytes.delete(m.key);
      meta.delete(m.key);
      total -= m.size;
    }
    await finished(t);
  }

  async clear(): Promise<void> {
    const db = await this.db;
    const t = db.transaction([BYTES, META], 'readwrite');
    t.objectStore(BYTES).clear();
    t.objectStore(META).clear();
    await finished(t);
  }

  async size(): Promise<number> {
    const db = await this.db;
    const all = await done(db.transaction(META, 'readonly').objectStore(META).getAll()) as Meta[];
    return all.reduce((n, m) => n + m.size, 0);
  }
}

/** The page's store: IndexedDB under the core's identity where the page has both, else none. */
export function pageStore(identity: string | undefined): SurfaceStore | null {
  if (!identity || typeof indexedDB === 'undefined') return null;
  try { return new IdbStore(identity); } catch { return null; }
}
