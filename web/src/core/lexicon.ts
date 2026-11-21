/* The lexicon: every word a document can be written with, as the core reads it
 * (`lexicon.rs`).  Types only — what each word takes is the core's, and `Document.lexicon`,
 * `wordText` and `writeWord` are where it is asked. */

/** How a slot is written in its statement. */
export type Role =
  | 'name' | 'left' | 'right' | 'operand' | 'arg' | 'value' | 'option' | 'pin' | 'seed';

/** One hole of a word. */
export interface Slot {
  key: string;
  role: Role;
  /** The kinds a click may fill it with (`point`, `plane`, `face`, `solid`); `any` is anything
   *  drawn.  Empty for a slot only typed into. */
  kinds: string[];
  /** What is typed: `Length`, `Angle`, `Scalar`, `Int`, `Bool`, `word`, `expr`, `vector`,
   *  `place`, `name`, or an interface's name.  `null` for a slot only clicked. */
  type: string | null;
  /** The words it takes, where it takes words. */
  words: string[];
  optional: boolean;
  /** Several, in order: a spline's control points, a face's edges. */
  many: boolean;
  /** A point slot a click on nothing fills with a new point. */
  mints: boolean;
  /** What a formal is given when left out. */
  default: string | null;
}

/** One word. */
export interface Word {
  /** Unique: `line`, `distance`, `std.place.right_of`, `std.solids.Prism`, a document's `Lug`. */
  id: string;
  word: string;
  /** `''` for the language's own, `'.'` for the document's, else the module. */
  module: string;
  /** What it is for: `Geometry`, `Relations`, `Dimensions`, `Placement`, … */
  group: string;
  form: 'declare' | 'infix' | 'prefix' | 'call' | 'instance' | 'param' | 'value';
  doc: string;
  makes: string | null;
  slots: Slot[];
  /** A built-in infix word's operand kinds, pair by pair, each with the selectors it is settled
   *  by only when they are given (`tangent` between two lines needs `at`). */
  pairs: [string, string, string[]][];
}

/** A word and what fills its slots: each slot's text by its key. */
export interface WordRequest {
  id: string;
  args: Record<string, string>;
  plane?: string;
}
