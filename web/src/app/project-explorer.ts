/** A file the tree shows: Solvent sources, and the plain text beside them (a README). */
export const projectFile = (path: string): boolean => /\.(?:svd?|md|txt)$/.test(path);
/** A file the core reads.  Any other project file is text to show, never to elaborate. */
export const sourceFile = (path: string): boolean => /\.svd?$/.test(path);

/** The chevron a folder or an outline row opens by, pointing at what it opens to. */
export function disclosure(): SVGSVGElement {
  const ns = 'http://www.w3.org/2000/svg';
  const svg = document.createElementNS(ns, 'svg');
  svg.setAttribute('class', 'project-disclosure');
  svg.setAttribute('aria-hidden', 'true');
  svg.setAttribute('viewBox', '0 0 24 24');
  const path = document.createElementNS(ns, 'path');
  path.setAttribute('d', 'm9 5 7 7-7 7');
  svg.append(path);
  return svg;
}

/** A directory pane with native, keyboard-accessible folder toggles and file buttons. */
export class ProjectExplorer {
  private readonly buttons = new Map<string, HTMLButtonElement>();
  constructor(private readonly tree: HTMLElement, private readonly title: HTMLElement,
    private readonly onSelect: (path: string) => void) {}

  /** The files of the project rooted at `root`, or none where no folder is open. */
  show(paths: string[], root?: string): void {
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
      summary.append(disclosure());
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
