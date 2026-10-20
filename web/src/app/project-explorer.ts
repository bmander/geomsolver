/** A file the tree shows: Solvent sources, and the plain text beside them (a README). */
export const projectFile = (path: string): boolean => /\.(?:svd?|md|txt)$/.test(path);
/** A file the core reads.  Any other project file is text to show, never to elaborate. */
export const sourceFile = (path: string): boolean => /\.svd?$/.test(path);

/** Fold a pane of the program panel to its head and open it again by its head's toggle — the
 *  project's files and the model outline alike.  `noun` is what the toggle's title calls it;
 *  `opened` is told each time it is opened. */
export function bindPaneToggle(pane: HTMLElement, tree: HTMLElement, toggle: HTMLButtonElement,
  noun: string, opened: () => void = () => {}): void {
  toggle.addEventListener('click', () => {
    const collapsed = toggle.getAttribute('aria-expanded') === 'true';
    toggle.setAttribute('aria-expanded', String(!collapsed));
    toggle.title = `${collapsed ? 'Expand' : 'Collapse'} ${noun}`;
    pane.classList.toggle('collapsed', collapsed);
    tree.hidden = collapsed;
    if (!collapsed) opened();
  });
  // Enter and Space belong to the native button, not the model editor's accelerators.
  toggle.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' || e.key === ' ') e.stopPropagation();
  });
}

/** A directory pane with native, keyboard-accessible folder toggles and file buttons. */
export class ProjectExplorer {
  private readonly buttons = new Map<string, HTMLButtonElement>();
  constructor(private readonly pane: HTMLElement, private readonly tree: HTMLElement,
    private readonly title: HTMLElement, private readonly onSelect: (path: string) => void) {
    bindPaneToggle(pane, tree, pane.querySelector<HTMLButtonElement>('#project-toggle')!,
      'file explorer');
  }

  show(paths: string[], root?: string): void {
    this.pane.hidden = root === undefined;
    this.tree.replaceChildren(); this.buttons.clear();
    if (root === undefined) return;
    this.title.textContent = root || 'Project';
    const folders = new Map<string, HTMLElement>([['', this.tree]]);
    const folder = (path: string): HTMLElement => {
      const found = folders.get(path);
      if (found) return found;
      const slash = path.lastIndexOf('/');
      const parent = folder(slash < 0 ? '' : path.slice(0, slash));
      const details = document.createElement('details');
      const summary = document.createElement('summary');
      summary.textContent = path.slice(slash + 1);
      summary.append(this.pane.querySelector('.project-disclosure')!.cloneNode(true));
      const children = document.createElement('div'); children.className = 'project-folder';
      details.append(summary, children); parent.append(details);
      folders.set(path, children);
      return children;
    };
    for (const path of [...paths].sort()) {
      const relative = root && path.startsWith(`${root}/`) ? path.slice(root.length + 1) : path;
      const slash = relative.lastIndexOf('/');
      const button = document.createElement('button');
      button.type = 'button'; button.textContent = relative.slice(slash + 1);
      button.title = path; button.dataset.path = path;
      button.className = `project-file${path.endsWith('.svd') ? ' drawing-file'
        : sourceFile(path) ? '' : ' text-file'}`;
      button.addEventListener('click', () => this.onSelect(path));
      folder(slash < 0 ? '' : relative.slice(0, slash)).append(button);
      this.buttons.set(path, button);
    }
  }

  select(path: string): void {
    for (const [name, button] of this.buttons) {
      if (name === path) button.setAttribute('aria-current', 'true');
      else button.removeAttribute('aria-current');
    }
    const selected = this.buttons.get(path);
    for (let parent = selected?.parentElement; parent && parent !== this.tree;
      parent = parent.parentElement) {
      if (parent instanceof HTMLDetailsElement) parent.open = true;
    }
    if (!this.tree.hidden) selected?.scrollIntoView({ block: 'nearest' });
  }
}
