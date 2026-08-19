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

/** Scrollable read-only report. */
export function showReport(title: string, text: string): Promise<void> {
  return open<void>(title, (resolve) => {
    const pre = document.createElement('pre');
    pre.textContent = text;
    modalBody.append(pre);
    modalActions.append(button('Close', () => resolve(), true));
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
    p.style.margin = '0 0 8px';
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

export function addSeparator(bar: HTMLElement): void {
  const s = document.createElement('span');
  s.className = 'sep';
  bar.append(s);
}

export function addCheckbox(bar: HTMLElement, label: string, checked: boolean,
                            onChange: (v: boolean) => void): HTMLInputElement {
  const l = document.createElement('label');
  l.className = 'check';
  const c = document.createElement('input');
  c.type = 'checkbox';
  c.checked = checked;
  c.addEventListener('change', () => onChange(c.checked));
  l.append(c, document.createTextNode(label));
  bar.append(l);
  return c;
}

export function addSelect(bar: HTMLElement, options: { value: string; label: string; title?: string }[],
                          onChange: (v: string) => void, width?: string): HTMLSelectElement {
  const s = document.createElement('select');
  for (const o of options) {
    const opt = document.createElement('option');
    opt.value = o.value;
    opt.textContent = o.label;
    if (o.title) opt.title = o.title;
    s.append(opt);
  }
  if (width) s.style.minWidth = width;
  s.addEventListener('change', () => onChange(s.value));
  bar.append(s);
  return s;
}

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
