/** Browser entry points are files or directories; solver cases also retain their individual keys. */
import { cases, type Case } from '../core/examples.js';

export type ExampleTarget = { kind: 'file'; path: string }
  | { kind: 'directory'; path: string; entry: string };
export interface Example extends Case { target: ExampleTarget }
const parts = ['cylinder', 'plate', 'piston', 'disc', 'flywheel', 'throttle'];

/** Projects the core's case library does not hold, read like the V-twin from the demo server or
 *  the packaged sources: their modules are theirs, not the library's. */
const projects: Example[] = [{
  label: 'Hypoid gear pair · swept solids', key: 'spiral_bevel',
  description: 'A 24-tooth pinion and a 48-tooth gear, each cut by one crown tooth rolled through '
    + 'its blank at every index. Edit the tooth counts, module, offset or spiral angle in '
    + 'configuration.sv; the solids refine in the background — open the glass box (⌘B) to watch.',
  target: { kind: 'directory', path: 'spiral_bevel', entry: 'gears.sv' },
}];

export function exampleCases(): Example[] {
  return cases().filter((c) => !parts.some((p) => c.key === `vtwin_${p}`)).map((c): Example =>
    c.key === 'vtwin' ? { ...c, label: 'V-twin air engine',
      description: 'Assembly and six part drawings, with their editable models and shared components.',
      target: { kind: 'directory', path: 'vtwin', entry: 'assembly.svd' } }
    : { ...c, target: { kind: 'file', path: `${c.key}.svd` } }).concat(projects);
}

/** Keep old part links useful, and accept explicit source paths as file examples. */
export function exampleTarget(key: string): { key: string; target: ExampleTarget; file?: string } {
  const part = parts.find((p) => key === `vtwin_${p}`);
  if (part) return { ...exampleTarget('vtwin'), file: `vtwin/${part}.svd` };
  const stem = key.split(':')[0];
  const known = exampleCases().find((c) => c.key === stem);
  if (known) return { key, target: known.target };
  if (/^(?:[\w-]+\/)*[\w-]+\.svd?$/.test(key)) {
    return { key, target: { kind: 'file', path: key } };
  }
  throw new Error(`unknown example: ${key}`);
}
