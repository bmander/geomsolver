/* **The model outline** (#435): every instance the document makes and its members, as a tree —
 * the core's (`overview::outline`), read through `Document.outline`, which brings its spans onto
 * the string as every report's are.  Nothing here walks the source: a row says what it is, what
 * it is carried by and what it made. */
import type { Part } from './model.js';
import type { SourceSpan } from './program.js';

/** What a row stands for: an instance, a block and its copies, an entity a statement made, a
 *  solid (an object, or an operand listed under one), or what a module's datums made. */
export type RowKind = 'instance' | 'block' | 'copy' | 'entity' | 'solid' | 'datums';

/** One row of the outline, as `gcs_elab_outline_json` hands it over. */
export interface Row {
  label: string;
  kind: RowKind;
  /** The component an instance is of. */
  component?: string;
  /** The name the row is carried across an edit by — empty for an anonymous row, which is not. */
  path: string;
  /** Where its statement is in the document; none for a module's statement. */
  span?: SourceSpan;
  /** The entities the row itself stands for, by the core's kind word and index. */
  ents: [string, number][];
  /** A solid row's solid. */
  solid?: number;
  /** A feature or cutter listed under its object: there to be found, not to be hidden. */
  operand?: boolean;
  children: Row[];
}

/** Every entity and solid a row and the rows under it stand for: what hiding it takes out. */
export function partOf(row: Row, into: Part = { ents: [], solids: [] }): Part {
  into.ents.push(...row.ents);
  if (row.solid !== undefined) into.solids.push(row.solid);
  for (const c of row.children) partOf(c, into);
  return into;
}
