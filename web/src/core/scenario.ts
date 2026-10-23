/* **A load scenario seen in the workspace** (#510): where its lines fall on the drawing, and how
 * the drawing moves under it — the core's `report::scenario_marks_json` and
 * `report::scenarios_pictured_json`, typed.  No geometry here: every place, direction and move
 * is the core's, and `Document.scenarioMarks` / `Document.scenariosPictured` are the calls. */
import type { Scenario, ScenarioAnswer } from './program.js';

/** A point or a direction in space, in the document's units. */
export type Vec3 = [number, number, number];

/** What a mark is: a support (`fixed`, `pinned`), a load (`load`, `pressure`, `accelerate`), a
 *  requirement, or a solid read in its plane. */
export type MarkKind =
  'fixed' | 'pinned' | 'load' | 'pressure' | 'accelerate' | 'requirement' | 'plane';

/** **One line of a scenario, drawn**: what it is and which line (its index among the scenario's
 *  supports, loads, requirements or readings, by kind); where it is drawn (`at`); at each place
 *  the direction it acts along — a force's, a pressure's into its face, the one direction a
 *  support holds — empty where it acts along none; a face's outward normal there, empty off a
 *  face; the faces it tints, nine numbers a triangle; its size among the scenario's loads of its
 *  kind (0 to 1); its words (`600N`, `pinned along z`) and where they go; and where the line
 *  it is written on is in the document's text (string offsets), null where the document does
 *  not hold it. */
export interface ScenarioMark {
  kind: MarkKind;
  index: number;
  at: Vec3[];
  dir: Vec3[];
  normal: Vec3[];
  tint: number[];
  size: number;
  text: string;
  label: Vec3;
  /** A requirement read at a place — a deflection — marked there; one of a whole body (its mass,
   *  its safety) has no place of its own, and is the card's to say. */
  probe: boolean;
  span: [number, number] | null;
}

/** **One body of a scenario's picture**: a frame of beams (a tube along each element, sampled on
 *  its cubic), a region read in its plane, or a solid read in three dimensions — the mesh that
 *  was solved, never the display's.  Flat arrays: three numbers a vertex at rest and three for
 *  its move, its move as a fraction of the picture's largest, three vertex indices a triangle
 *  and two an edge of the mesh. */
export interface PictureBody {
  kind: 'frame' | 'region' | 'solid';
  name: string;
  /** The solid it is, a region's or a solid's: what the box ghosts beneath it. */
  solid: number | null;
  rest: number[];
  moved: number[];
  size: number[];
  triangles: number[];
  edges: number[];
}

/** **How the drawing moves under a scenario**: its bodies; why any it falls on is not drawn, in
 *  the core's words; the largest move, as a reader reads it, and where it stands; the extent of
 *  what is drawn; and the factor to draw the moves at — the largest a tenth of the extent,
 *  rounded down to 1, 2 or 5 of a decade, never below true scale. */
export interface ScenarioPicture {
  bodies: PictureBody[];
  words: string[];
  largest: number;
  largestText: string;
  largestAt: Vec3;
  extent: number;
  scale: number;
}

/** Every scenario's answers, and one's picture (null where none was asked for); and what each
 *  mass and volume the drawing states comes to (#510), read beside them. */
export interface Pictured {
  scenarios: Scenario[];
  picture: ScenarioPicture | null;
  measures: ScenarioAnswer[];
}

/** **One level of a progressive reading** (`report::Levels`): `Pictured` read with its elements
 *  `level` times the readers' size, its errors against the level before; `final` at the last,
 *  which is `Document.scenariosPictured`'s reading; `levelText` what it is, in the core's words
 *  (empty at the last). */
export interface Level extends Pictured {
  level: number;
  final: boolean;
  levelText: string;
}
