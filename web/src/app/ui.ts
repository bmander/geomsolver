/* Small DOM helpers: the modal used for reports, numeric prompts and root selection, plus
 * the status line.  Kept deliberately thin — the app's substance is in core/. */

const modal = document.getElementById('modal') as HTMLDialogElement;
const modalTitle = document.getElementById('modal-title') as HTMLElement;
const modalBody = document.getElementById('modal-body') as HTMLElement;
const modalActions = document.getElementById('modal-actions') as HTMLElement;
const toastEl = document.getElementById('toast') as HTMLElement;
const statsEl = document.getElementById('stats') as HTMLElement;

let toastTimer = 0;

/** Transient message in the status bar. */
export function toast(msg: string, ms = 6000): void {
  toastEl.textContent = msg ? `${msg}   |   ` : '';
  clearTimeout(toastTimer);
  if (msg) toastTimer = window.setTimeout(() => { toastEl.textContent = ''; }, ms);
}

export function stats(msg: string): void {
  statsEl.textContent = msg;
}

const claimsEl = document.getElementById('claims') as HTMLElement;

/** The claims about solids' line in the footer, and how it reads: going, failed (one refuted, or
 *  the judging failed) or finished. */
export function judgingLine(msg: string, state: 'going' | 'failed' | 'finished' = 'going'): void {
  claimsEl.textContent = msg;
  claimsEl.className = state === 'going' ? '' : state;
}

const scenariosEl = document.getElementById('scenarios') as HTMLElement;

/** The load scenarios' line in the footer: failed (a requirement that does not hold, or has no
 *  answer) or finished, with every sentence the core said of them as its tooltip; empty, none. */
export function scenarioLine(msg: string, state: '' | 'failed' | 'finished' = '', said = ''): void {
  scenariosEl.textContent = msg;
  scenariosEl.className = state;
  scenariosEl.title = said;
}

function open<T>(title: string, build: (resolve: (v: T) => void) => void): Promise<T> {
  modalTitle.textContent = title;
  modalBody.replaceChildren();
  modalActions.replaceChildren();
  return new Promise<T>((resolve) => {
    let done = false;
    const finish = (v: T): void => {
      if (done) return;
      done = true;
      modal.close();
      resolve(v);
    };
    modal.addEventListener('close', () => finish(undefined as T), { once: true });
    build(finish);
    modal.showModal();
  });
}

export function button(label: string, onClick: () => void, primary = false): HTMLButtonElement {
  const b = document.createElement('button');
  b.textContent = label;
  if (primary) b.style.borderColor = 'var(--accent)';
  b.addEventListener('click', onClick);
  return b;
}

/** A modal whose only action is Close: the caller fills the body.  Nothing in one is
 *  confirmed — a report has nothing to confirm, and a sheet of switches has already taken
 *  effect by the time you read them. */
export function showSheet(title: string, build: (body: HTMLElement) => void): Promise<void> {
  return open<void>(title, (resolve) => {
    build(modalBody);
    modalActions.append(button('Close', () => resolve(), true));
  });
}

/** Scrollable read-only report. */
export function showReport(title: string, text: string): Promise<void> {
  return showSheet(title, (body) => {
    const pre = document.createElement('pre');
    pre.textContent = text;
    body.append(pre);
  });
}

/** Numeric prompt; resolves null on cancel. */
export function askNumber(title: string, label: string, value: number, step = 'any'): Promise<number | null> {
  return open<number | null>(title, (resolve) => {
    const l = document.createElement('label');
    l.textContent = label;
    l.style.display = 'block';
    l.style.marginBottom = '6px';
    const input = document.createElement('input');
    input.type = 'number';
    input.step = step;
    input.value = String(Number(value.toPrecision(10)));
    const submit = (): void => {
      const v = Number(input.value);
      resolve(Number.isFinite(v) ? v : null);
    };
    input.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); submit(); } });
    modalBody.append(l, input);
    modalActions.append(button('Cancel', () => resolve(null)), button('OK', submit, true));
    setTimeout(() => { input.focus(); input.select(); }, 0);
  });
}

/** A form: the caller fills the body with fields (`addText`, `addNumber`, `addSelect`) whose
 *  callbacks hold the answers, and this resolves true on OK and false on Cancel.  Enter in any
 *  field is OK.  Nothing is validated here — what a field's text means is the caller's, and
 *  usually the core's, to say. */
export function askFields(title: string, build: (body: HTMLElement) => void): Promise<boolean> {
  return open<boolean>(title, (resolve) => {
    const box = document.createElement('div');
    box.className = 'fields';
    build(box);
    box.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && (e.target as HTMLElement).tagName === 'INPUT') {
        e.preventDefault();
        resolve(true);
      }
    });
    modalBody.append(box);
    modalActions.append(button('Cancel', () => resolve(false)),
                        button('OK', () => resolve(true), true));
    setTimeout(() => box.querySelector('input')?.focus(), 0);
  });
}

/** Pick one of a list; resolves null on cancel. */
export function askChoice(title: string, label: string,
                          options: (string | { title: string; description: string })[],
                          searchable = false): Promise<number | null> {
  return open<number | null>(title, (resolve) => {
    const p = document.createElement('p');
    p.textContent = label;
    const box = document.createElement('div');
    box.className = 'choices';
    const rows = options.map((o, i) => {
      const row = button(typeof o === 'string' ? o : '', () => resolve(i));
      if (typeof o !== 'string') {
        row.className = 'described-choice';
        const title = document.createElement('strong');
        title.textContent = o.title;
        const description = document.createElement('span');
        description.textContent = o.description;
        row.append(title, description);
      }
      box.append(row);
      return row;
    });
    modalBody.append(p);
    if (searchable) {
      const search = document.createElement('input');
      search.type = 'search';
      search.placeholder = 'Search examples';
      search.setAttribute('aria-label', 'Search examples');
      const empty = document.createElement('p');
      empty.textContent = 'No examples match your search.';
      empty.hidden = true;
      search.addEventListener('input', () => {
        const query = search.value.trim().toLocaleLowerCase();
        for (const row of rows) row.hidden = !row.textContent?.toLocaleLowerCase().includes(query);
        empty.hidden = rows.some((r) => !r.hidden);
      });
      // the bar stays put above the list as it scrolls, so the search is in reach however far down
      const bar = document.createElement('div');
      bar.className = 'search-bar';
      bar.append(search);
      modalBody.append(bar, empty);
    }
    modalBody.append(box);
    modalActions.append(button('Cancel', () => resolve(null)));
  });
}

export interface ToolbarButton {
  label: string;
  onClick: () => void;
  key?: string;
  toggle?: boolean;
  title?: string;
}

export function addButton(bar: HTMLElement, spec: ToolbarButton): HTMLButtonElement {
  const b = document.createElement('button');
  b.textContent = spec.label;
  if (spec.key) {
    const k = document.createElement('kbd');
    k.textContent = spec.key.toUpperCase();
    b.append(k);
  }
  if (spec.title) b.title = spec.title;
  if (spec.toggle) b.setAttribute('aria-pressed', 'false');
  b.addEventListener('click', spec.onClick);
  bar.append(b);
  return b;
}

/** An outbound link on a bar.  Its own tab, because following it leaves the sketch behind. */
export function addLink(bar: HTMLElement, label: string, href: string): HTMLAnchorElement {
  const a = document.createElement('a');
  a.textContent = label;
  a.href = href;
  a.target = '_blank';
  a.rel = 'noopener';
  bar.append(a);
  return a;
}

export function addSeparator(bar: HTMLElement): void {
  const s = document.createElement('span');
  s.className = 'sep';
  bar.append(s);
}

/** The wrapper a labelled control sits in, so the text is part of the control's hit area and
 *  the tooltip covers both.  Callers append in the order the control reads: a checkbox before
 *  its label, a select after it. */
function field(bar: HTMLElement, title?: string): HTMLLabelElement {
  const l = document.createElement('label');
  l.className = 'check';
  if (title) l.title = title;
  bar.append(l);
  return l;
}

export function addCheckbox(bar: HTMLElement, label: string, checked: boolean,
                            onChange: (v: boolean) => void, title?: string): HTMLInputElement {
  const c = document.createElement('input');
  c.type = 'checkbox';
  c.checked = checked;
  c.addEventListener('change', () => onChange(c.checked));
  field(bar, title).append(c, document.createTextNode(label));
  return c;
}

/** A text field.  `onChange` hears every keystroke, so the caller's variable is the field's
 *  current text by the time the form is accepted. */
export function addText(bar: HTMLElement, label: string, value: string,
                        onChange: (v: string) => void, title?: string): HTMLInputElement {
  const t = document.createElement('input');
  t.type = 'text';
  t.value = value;
  t.addEventListener('input', () => onChange(t.value));
  field(bar, title).append(document.createTextNode(label), t);
  return t;
}

/** A number field, handed on as the *text* typed: a form's numbers are spliced into the
 *  source as written, and `-90` and `-90.0` are two spellings the document keeps apart. */
export function addNumber(bar: HTMLElement, label: string, value: number,
                          onChange: (v: string) => void, title?: string): HTMLInputElement {
  const t = document.createElement('input');
  t.type = 'number';
  t.step = 'any';
  t.value = String(value);
  t.addEventListener('input', () => onChange(t.value));
  field(bar, title).append(document.createTextNode(label), t);
  return t;
}

export function addSelect(bar: HTMLElement, label: string, options: string[], value: string,
                          onChange: (v: string) => void, title?: string): HTMLSelectElement {
  const s = document.createElement('select');
  for (const o of options) {
    const opt = document.createElement('option');
    opt.value = o;
    opt.textContent = o;
    s.append(opt);
  }
  s.value = value;
  s.addEventListener('change', () => onChange(s.value));
  field(bar, title).append(document.createTextNode(label), s);
  return s;
}

/* -- commands ----------------------------------------------------------------------- */

/** A command of the app as `main` tables it, the Words menu's App branch lists it and the
 *  keyboard reads its key — described as a button is, `toggle` being no use to it. */
export type MenuItem = Omit<ToolbarButton, 'toggle'> & {
  /** A setting's: whether it is on, asked each time the menu lists it. */
  checked?: () => boolean;
};

/** What a downloaded file is, by what it is called.  The name already states the format, so a
 *  caller that had to state it again would be writing one fact twice — and a Blob given the
 *  wrong type fails silently, which is the way to get it wrong that nobody notices.  A format
 *  the table does not know is offered as plain text: everything this app writes is text. */
const MIME: Record<string, string> = {
  json: 'application/json',
  svg: 'image/svg+xml',
  sv: 'text/plain',
  glb: 'model/gltf-binary',
  stl: 'model/stl',
};

/** Download a string as a file — the document as JSON, the source as `.sv`, the drawing
 *  as SVG. */
export function download(name: string, data: string | Uint8Array): void {
  // bytes as readily as text: a `.glb` and a `.stl` are binary, and a Blob takes either
  const part: BlobPart = typeof data === 'string' ? data : new Uint8Array(data);
  const blob = new Blob([part], { type: MIME[name.split('.').pop() ?? ''] ?? 'text/plain' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

/** Ask the browser for a local file (null if the user cancels).  The dialog is the same
 *  whatever is wanted off the file afterwards, so it is asked for in one place — what a caller
 *  does with the `File` is the only part that differs. */
function pickFile(accept: string): Promise<File | null> {
  return new Promise((resolve) => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = accept;
    input.addEventListener('change', () => resolve(input.files?.[0] ?? null));
    input.addEventListener('cancel', () => resolve(null));
    input.click();
  });
}

/** Ask for a local file and return its text (null if the user cancels). */
export function openFile(accept = '.json'): Promise<string | null> {
  return pickFile(accept).then((f) => (f ? f.text().catch(() => null) : null));
}

/** Pick an image file and decode it, as the element the canvas will draw and the URL that
 *  element is holding — the caller lets the URL go when it lets the picture go.
 *
 *  An object URL rather than a data URL: the picture is a photograph, and reading a ten-megabyte
 *  one into a base64 string to hand straight back to the same browser is work nobody asked for. */
export async function openImage():
    Promise<{ image: HTMLImageElement; name: string; url: string } | null> {
  const f = await pickFile('image/*');
  if (!f) return null;
  const url = URL.createObjectURL(f);
  const image = new Image();
  image.src = url;
  try {
    // `decode()` and not `load`: `load` only promises the bytes and the intrinsic size, and
    // leaves the decode to the first `drawImage` — which happens inside a frame callback, so a
    // twelve-megapixel photograph would spend it stalling the canvas.  Decoded here, the frame
    // that first draws the picture costs what every later one does.
    await image.decode();
    return { image, name: f.name, url };
  } catch {
    URL.revokeObjectURL(url);              // nothing will ever draw it, so nothing will free it
    return null;
  }
}

/** Make a floating window draggable by a handle inside it, kept inside its offset parent.
 *  Where it sits is the user's, so it is written into `left`/`top` once and the CSS corner it
 *  started in is dropped — nothing re-places it afterwards. */
export function dragWindow(win: HTMLElement, handle: HTMLElement): void {
  handle.addEventListener('pointerdown', (e) => {
    if (e.button !== 0) return;
    e.preventDefault();
    const box = win.getBoundingClientRect();
    const par = (win.offsetParent ?? document.body).getBoundingClientRect();
    const dx = e.clientX - box.left;
    const dy = e.clientY - box.top;
    const move = (m: PointerEvent): void => {
      const x = Math.max(0, Math.min(par.width - box.width, m.clientX - par.left - dx));
      const y = Math.max(0, Math.min(par.height - box.height, m.clientY - par.top - dy));
      win.style.left = `${x}px`;
      win.style.top = `${y}px`;
      win.style.right = 'auto';
    };
    handle.setPointerCapture(e.pointerId);
    handle.addEventListener('pointermove', move);
    handle.addEventListener('pointerup', () => handle.removeEventListener('pointermove', move),
                            { once: true });
  });
}
