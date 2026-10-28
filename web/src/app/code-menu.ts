/* **The program editor's right-click menu**, as every code editor has one (VS Code's, JetBrains'):
 * what can be done at the caret, the code's own verbs first and the clipboard's after.
 *
 * Conventions taken from those editors, since a person arrives with them in their hands:
 *   * a right-click outside the selection puts the caret where it was pressed first, and one
 *     inside it keeps the selection, so the menu acts on what was right-clicked;
 *   * a verb with nothing to act on here is shown greyed rather than left out, so the menu keeps
 *     its shape and says what it could do;
 *   * ⇧F10 and the menu key open it at the caret; the arrows walk it, Enter takes a row, Escape
 *     closes it and any other key closes it and is the text's;
 *   * ⇧ held on the right-click lets the browser's own menu through (a spelling, an inspect).
 *
 * And one of this app's: **the menu never takes the focus from the text.**  A blur applies a
 * draft (`program.ts`), whose reprint moves every offset the menu was opened on, so the menu's
 * keys are read off the box while it is open and a press on it keeps the focus where it was — as
 * the compile button's does.
 *
 * The menu knows nothing of Solvent: it is handed the rows to offer when it opens
 * (`CodeMenu.items`), and the editor knows nothing of the menu. */
import type { CodeEditor } from './editor.js';

/** One row of the menu: its words, a key that does the same (`note`), and what taking it does —
 *  null where there is nothing to do here, which shows it greyed. */
export interface CodeMenuRow {
  label: string;
  note?: string;
  run: (() => void) | null;
}

/** The clipboard's rows, as a text box's own menu has them: Cut and Paste only where the text can
 *  be changed, Cut and Copy only over a selection.  Each goes through the box as typing would
 *  (`execCommand`), so an undo takes it back and the page hears it as an edit. */
export function clipboardItems(ed: CodeEditor): CodeMenuRow[] {
  const box = ed.box;
  const { lo, hi } = ed.caret;
  const some = lo < hi;
  const writable = !box.readOnly && !box.disabled;
  // the key the platform's shortcuts are held with, as a menu writes it
  const mod = /Mac|iPhone|iPad/.test(navigator.platform) ? '⌘' : 'Ctrl+';
  const paste = writable && navigator.clipboard?.readText
    ? () => {
      void navigator.clipboard.readText()
        .then((t) => { document.execCommand('insertText', false, t); }, () => {});
    }
    : null;
  return [
    { label: 'Cut', note: `${mod}X`,
      run: some && writable ? () => document.execCommand('cut') : null },
    { label: 'Copy', note: `${mod}C`, run: some ? () => document.execCommand('copy') : null },
    { label: 'Paste', note: `${mod}V`, run: paste },
    { label: 'Select all', note: `${mod}A`, run: () => box.select() },
  ];
}

/** The menu on one editor (module header). */
export class CodeMenu {
  private open: { el: HTMLElement; away: (e: Event) => void } | null = null;
  /** The rows of the menu open, and the one the arrows or the pointer are on. */
  private rows: { el: HTMLButtonElement; run: (() => void) | null }[] = [];
  private active = -1;

  /** `items` is asked for the rows each time the menu opens, the caret already where the
   *  right-click put it: groups of rows, a line drawn between each. */
  constructor(private readonly ed: CodeEditor, private readonly items: () => CodeMenuRow[][]) {
    const box = ed.box;
    box.addEventListener('contextmenu', (e) => {
      if (e.shiftKey) return;
      e.preventDefault();
      // a menu key reports no point of its own, and the menu goes to the caret; a mouse's is its
      // right button, or a Mac's ⌃-click
      const pointed = (e.button === 2 || e.ctrlKey) && (e.clientX !== 0 || e.clientY !== 0);
      if (!pointed) {
        this.atCaret();
        return;
      }
      box.focus();
      const at = ed.offsetAt(e.clientX, e.clientY);
      const { lo, hi } = ed.caret;
      if (lo === hi || at < lo || at > hi) ed.select(at);
      this.show([e.clientX, e.clientY]);
    });
    // the menu's keys, ahead of the box's own (`program.ts`'s Escape reverts a draft): a
    // listener in the capture phase runs first at the box
    box.addEventListener('keydown', (e) => {
      if (!this.open) {
        if (e.key === 'F10' && e.shiftKey && !e.metaKey && !e.ctrlKey && !e.altKey) {
          e.preventDefault();
          this.atCaret();
        }
        return;
      }
      if (this.key(e.key)) {
        e.preventDefault();
        e.stopImmediatePropagation();
      } else {
        this.close();
      }
    }, true);
    box.addEventListener('blur', () => this.close());
  }

  /** Open the menu under the caret. */
  atCaret(): void {
    this.show(this.ed.pointAt(this.ed.caret.hi));
  }

  /** Open the menu at a point on the page (client pixels), kept inside the window. */
  show(at: [number, number]): void {
    this.close();
    const el = document.createElement('div');
    el.className = 'chooser code-menu';
    el.id = 'code-menu';
    el.setAttribute('role', 'menu');
    for (const group of this.items()) {
      if (this.rows.length && group.length) el.append(document.createElement('hr'));
      for (const item of group) {
        const k = this.rows.length;
        const row = document.createElement('button');
        row.type = 'button';
        row.tabIndex = -1;
        row.id = `code-menu-${k}`;
        row.setAttribute('role', 'menuitem');
        const verb = document.createElement('span');
        verb.className = 'verb';
        verb.textContent = item.label;
        row.append(verb);
        if (item.note) {
          const note = document.createElement('span');
          note.className = 'note';
          note.textContent = item.note;
          row.append(note);
        }
        if (!item.run) row.setAttribute('aria-disabled', 'true');
        row.addEventListener('pointerenter', () => this.light(k));
        row.addEventListener('click', () => this.take(k));
        el.append(row);
        this.rows.push({ el: row, run: item.run });
      }
    }
    if (!this.rows.length) return;
    // a press on the menu keeps the focus in the text (module header)
    el.addEventListener('pointerdown', (e) => e.preventDefault());
    el.addEventListener('contextmenu', (e) => e.preventDefault());
    el.addEventListener('pointerleave', () => this.light(-1));
    document.body.append(el);
    const x = Math.max(4, Math.min(at[0] + 2, window.innerWidth - el.offsetWidth - 4));
    const y = Math.max(4, Math.min(at[1] + 2, window.innerHeight - el.offsetHeight - 4));
    el.style.left = `${x}px`;
    el.style.top = `${y}px`;
    // a press anywhere else, or the wheel, is done with it
    const away = (e: Event) => {
      if (!el.contains(e.target as Node)) this.close();
    };
    document.addEventListener('pointerdown', away, true);
    document.addEventListener('wheel', away, true);
    this.open = { el, away };
    // the text keeps the focus, and says which row the arrows are on to whoever reads it aloud
    this.ed.box.setAttribute('aria-controls', el.id);
    this.light(-1);
  }

  /** Close the menu, if it is open. */
  close(): void {
    if (!this.open) return;
    const { el, away } = this.open;
    this.open = null;
    this.rows = [];
    document.removeEventListener('pointerdown', away, true);
    document.removeEventListener('wheel', away, true);
    el.remove();
    this.ed.box.removeAttribute('aria-controls');
    this.ed.box.removeAttribute('aria-activedescendant');
  }

  /** A key pressed in the text while the menu is open: whether it was the menu's. */
  private key(key: string): boolean {
    // the arrows walk the rows that can be taken, round the ends
    const live = this.rows.flatMap((r, k) => (r.run ? [k] : []));
    if (key === 'Escape' || key === 'Tab') {
      this.close();
    } else if (key === 'ArrowDown' || key === 'ArrowUp') {
      const step = key === 'ArrowDown' ? 1 : -1;
      const i = live.indexOf(this.active);
      const j = i < 0 ? (step > 0 ? 0 : live.length - 1) : (i + step + live.length) % live.length;
      if (live.length) this.light(live[j]);
    } else if (key === 'Home' || key === 'End') {
      if (live.length) this.light(live[key === 'Home' ? 0 : live.length - 1]);
    } else if ((key === 'Enter' || key === ' ') && this.active >= 0) {
      this.take(this.active);
    } else if (key !== 'Shift' && key !== 'Control' && key !== 'Alt' && key !== 'Meta') {
      return false;
    }
    return true;
  }

  /** Light row `k` as the one Enter takes, or none at -1. */
  private light(k: number): void {
    this.active = k;
    this.rows.forEach((r, i) => r.el.classList.toggle('active', i === k));
    if (k >= 0) this.ed.box.setAttribute('aria-activedescendant', this.rows[k].el.id);
    else this.ed.box.removeAttribute('aria-activedescendant');
  }

  /** Take row `k`: close the menu and do what it does, where it does something. */
  private take(k: number): void {
    const run = this.rows[k]?.run;
    if (!run) return;
    this.close();
    run();
  }
}
