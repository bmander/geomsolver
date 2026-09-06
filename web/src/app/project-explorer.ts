/** A directory pane with native, keyboard-accessible folder toggles and file buttons. */
export class ProjectExplorer {
  private readonly buttons = new Map<string, HTMLButtonElement>();
  constructor(private readonly pane: HTMLElement, private readonly tree: HTMLElement,
    private readonly title: HTMLElement, private readonly onSelect: (path: string) => void) {
    const toggle = pane.querySelector<HTMLButtonElement>('#project-toggle')!;
    toggle.addEventListener('click', () => {
      const collapsed = toggle.getAttribute('aria-expanded') === 'true';
      toggle.setAttribute('aria-expanded', String(!collapsed));
      toggle.title = `${collapsed ? 'Expand' : 'Collapse'} file explorer`;
      pane.classList.toggle('collapsed', collapsed);
      tree.hidden = collapsed;
    });
    // Enter and Space belong to the native button, not the model editor's accelerators.
    toggle.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' || e.key === ' ') e.stopPropagation();
    });
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
      button.className = path.endsWith('.svd') ? 'project-file drawing-file' : 'project-file';
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
