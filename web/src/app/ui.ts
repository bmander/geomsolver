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

/** Pick one of a list; resolves null on cancel. */
export function askChoice(title: string, label: string, options: string[]): Promise<number | null> {
  return open<number | null>(title, (resolve) => {
    const p = document.createElement('p');
    p.textContent = label;
    const box = document.createElement('div');
    box.className = 'choices';
    options.forEach((o, i) => box.append(button(o, () => resolve(i))));
    modalBody.append(p, box);
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

/* -- menu bar ------------------------------------------------------------------- */

/** A menu item is a button that happens to live in a list, so it is described the same way —
 *  `toggle` is the one thing a menu has no use for. */
export type MenuItem = Omit<ToolbarButton, 'toggle'>;

/** The one menu that is down, if any: its top button, whose `aria-expanded` is what the CSS
 *  drops the list on.  One variable, so nothing can disagree about what is open. */
let openMenu: HTMLButtonElement | null = null;

/** Shut the open menu; true if there was one, which is how Escape knows whether it has
 *  already done something before it goes on to back out of the tool. */
export function closeMenus(): boolean {
  if (!openMenu) return false;
  openMenu.setAttribute('aria-expanded', 'false');
  openMenu = null;
  return true;
}

/** One menu on the bar: a name, and the items under it with `null` for a dividing rule.
 *  Once one menu is open the others take over on hover, the way a desktop menu bar behaves. */
export function addMenu(bar: HTMLElement, label: string, items: (MenuItem | null)[]): void {
  const wrap = document.createElement('div');
  wrap.className = 'menu';
  const top = document.createElement('button');
  top.textContent = label;
  top.setAttribute('aria-expanded', 'false');
  const drop = (): void => {
    closeMenus();
    top.setAttribute('aria-expanded', 'true');
    openMenu = top;
  };
  // pointerdown rather than click, so the menu is up under the finger that opened it, and
  // default-prevented so the focus stays wherever it was and the key handler keeps working
  top.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (openMenu === top) closeMenus();
    else drop();
  });
  top.addEventListener('pointerenter', () => { if (openMenu && openMenu !== top) drop(); });
  const list = document.createElement('ul');
  for (const item of items) {
    const li = document.createElement('li');
    if (!item) li.className = 'sep';
    else addButton(li, { ...item, onClick: () => { closeMenus(); item.onClick(); } });
    list.append(li);
  }
  wrap.append(top, list);
  bar.append(wrap);
}

/* A press anywhere else puts the menu away — including on the canvas, where the same press
 * goes on to do whatever it was going to do. */
document.addEventListener('pointerdown', (e) => {
  if (openMenu && !(e.target as HTMLElement).closest('.menu')) closeMenus();
});

/** Download a string as a file (used by Save). */
export function download(name: string, text: string): void {
  const blob = new Blob([text], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

/** Ask for a local file and return its text (null if the user cancels). */
export function openFile(accept = '.json'): Promise<string | null> {
  return new Promise((resolve) => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = accept;
    input.addEventListener('change', () => {
      const f = input.files?.[0];
      if (!f) return resolve(null);
      f.text().then(resolve, () => resolve(null));
    });
    input.addEventListener('cancel', () => resolve(null));
    input.click();
  });
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
