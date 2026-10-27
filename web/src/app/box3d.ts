/* **The workspace's depth, rendered by three.js.**
 *
 * Every sketch is stroked on the 2D canvas over this one, each through its own plane's camera
 * (`paint.ts`), because that is where selection, state colours, callouts and the tools' previews
 * live.  What is left for a renderer with a depth buffer is what a flat canvas cannot order: the
 * solids' meshes, the planes' panes and the axes, which stand in space rather than on a plane.
 *
 * **The core owns every rule about what is here** — `overview::scene3d` says which panes exist and
 * how far each reaches, `mesh::grouped` what an object's faces are and which of them is round —
 * and this file turns that into three.js objects.  It computes no geometry: there is no place here
 * where a coordinate is worked out rather than read.  This canvas sits *under* the 2D one and
 * takes no pointer events; three.js's camera is set from `view.orbit` and the eye's camera
 * (`view.cam`), which is what the core projects every plane for, so the two agree about where a
 * thing is on screen.
 *
 * **An object is drawn as a CAD viewer draws a part** (#674–#676): one indexed geometry per shape
 * (`core/mesh.ts`'s `drawn`), shared by every object drawn from it — the solids of one binding
 * are one geometry — and one draw call for its faces and one for its edges.  Its faces are its
 * part's colour (`Document.partColours`: the drawing's `fill`, else the palette's), lit by a
 * hemisphere, a key and a fill light and a soft room; its B-rep's edges are inked as fat lines,
 * the sharp ones dark and the tangent ones lighter or not at all (View ▸ Tangent edges); and a
 * screen-space pass over the objects' normals and depths inks where a curved face turns away.
 * What is selected or under the pointer is a tint over the part's colour, never a colour of its
 * own: an overlay drawing the object's geometry again, its faces lit by a texture indexed by
 * each vertex's face.
 *
 * **A section** (`v.section`, `core/section.ts`) is a clipping plane on every object's material,
 * the one the core says it cuts by, and each object's cut face and outline from the core, hung in
 * the object's own group so its matrix places them as it places the object.  The face is hatched
 * in the picture's own pixels, as a draughtsman marks a cut.
 */
import * as THREE from 'three';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { LineSegments2 } from 'three/addons/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/addons/lines/LineSegmentsGeometry.js';

import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

import { drawn, faces, placement, preview, shownObjects, stale } from '../core/mesh.js';
import type { Drawn, Face, Mesh, Placement } from '../core/mesh.js';
import { overview3 } from '../core/overview.js';
import { clipPlane, sectioned } from '../core/section.js';
import type { Sectioned } from '../core/section.js';
import { eyeFrame, placeOf, solidEdges } from '../core/workspace.js';
import type { SolidEdge } from '../core/workspace.js';
import type { Item3 } from '../core/overview.js';
import { COL, DIMMED, chromeOf, inkOf } from './paint.js';
import { dropLabel, scaleLabels, textLabel } from './label.js';
import type { Label } from './label.js';
import { ScenarioScene } from './scenario-box.js';
import { partOf, samePick } from './view.js';
import type { Combining, SketchView } from './view.js';

/** The box's own ink.  Chrome, as the 2D painter's palette is: the core says how squarely a face
 *  meets the light and what kind of thing each polyline is, and what colour that is belongs
 *  here. */
const INK = {
  bg: 0xfafafa,
  pane: 0x7fb37f,
  axis: 0x5f8f5f,
  drawn: 0x1f77b4,
  /** An object's edges: the sharp ones, and where one face runs tangent into the next. */
  edge: 0x2b2f35,
  tangent: 0x8f959d,
  /** An object no part colour reaches. */
  solid: 0xb0b3b8,
  /** A Boolean's tools, ghosted in the colour of what they do to the body (#162 F4). */
  union: 0x2ca02c,
  cut: 0xd62728,
  common: 0x1f77b4,
  /** A section's cut face, and its hatching. */
  section: 0x9aa1aa,
  hatch: 0x5d636b,
};

/** A cut face's hatching, in device pixels: a line every `pitch`, `ink` of it dark. */
const HATCH = { pitch: 9, ink: 2 };

/** How much of a Boolean's tool shows through as a ghost. */
const GHOST = 0.4;

/** A pane's frame, in screen pixels: the plane being drawn on is bold.  WebGL draws every
 *  `LineBasicMaterial` one pixel wide whatever it is asked, so a frame is a fat line. */
const FRAME_PX = { plain: 1, current: 2.5 };

/** How an assembly shown on its parts draws its datums (#679): quietly — a pane's wash, its
 *  frame and the axes — so the parts are what is seen. */
const QUIET = { pane: 0.03, frame: 0.2, axis: 0.3 };

/** A pane's name, in CSS pixels: its type's height, and the room round it (#679). */
const LABEL_PX = { size: 11, pad: 3 };

/** An object's edges in pixels: as inked, and selected or under the pointer. */
const EDGE_PX = { ink: 1.25, picked: 3.5, hover: 2.5 };

/** How strongly a face is tinted in the ink of what is said of it: selected, its body selected,
 *  or under the pointer (itself, or its body). */
const GLOW = { face: 0.5, body: 0.28, hover: 0.35, hoverBody: 0.2 };

/** How a part's surface takes the light, the same for every part and a Boolean's preview. */
const SURFACE = { metalness: 0.05, roughness: 0.55 };

/** How much of the soft room an object's faces take: a gradient over a curved face, never
 *  enough to wash a part's colour out. */
const ROOM = 0.35;

/** The layer the objects' faces are on as well as the scene's, so the silhouette pass draws
 *  them and nothing else. */
const FACES = 1;

/** A step in depth the silhouette pass inks, in the picture's pixels: more than this many of
 *  them at the eye's scale, across one. */
const STEP_PX = 2;

/** The kinds that stand in space and on no one plane, so are this renderer's to draw. */
const IN_SPACE = new Set(['axis', 'traced']);

/** What the scene was built from, so a repaint that changes nothing rebuilds nothing.  Between
 *  edits mostly the camera moves — and rebuilding a mesh on every pointer move is the one way to
 *  make a depth buffer slower than the painter it replaced.  The zoom is not in it either: the box
 *  is cut to the object rather than to the screen.  **The pose is**: a drag moves numbers inside
 *  the same sketch, and a plane held to a dragged point moves with it, so the datums follow the
 *  pose every frame.  An object follows it too, by its matrix alone while it is the same shape
 *  (#412); one whose shape a gesture changes is cut again once the gesture is let go. */
interface Built {
  sketch: unknown;
  solid: boolean;
  /** The pose the datums were built at, and the one every object was placed at (`NaN` while one
   *  waits on a gesture to end before it is cut again). */
  datums: number;
  objects: number;
  /** The sketch's `shapeEpoch` the objects were built at: a solid re-shaped in place (an
   *  extrusion being sized) is re-cut at once, gesture or no. */
  shape: number;
  /** What the outline hid, as of `SketchView.hiddenEpoch`: a pane, an axis or an object hidden
   *  or shown is taken out or put back, nothing cut again. */
  hidden: number;
  /** The section, as of `SketchView.sectionEpoch`: the caps are cut again, the objects not. */
  section: number;
}

/** A geometry the box holds (`Placement.geometry`), shared by every object drawn from it: its
 *  faces indexed, each vertex with the face it is of, and its edges, sharp then tangent, as one
 *  fat line set — so the tangent ones are left out by drawing fewer of them. */
interface Shared {
  faces: THREE.BufferGeometry;
  edges: LineSegmentsGeometry | null;
  sharp: number;
  tangent: number;
  /** How many objects are drawn from it: let go at none. */
  users: number;
}

/** One object in the box: the evaluation its geometry was built from (`Placement.shape`), the
 *  geometry it is drawn from (`Placement.geometry`), its faces by name — its own, a peer of its
 *  binding naming them by its own instance — and the group its mesh and edges hang in, placed by
 *  the object's matrix. */
interface Placed {
  shape: number;
  geometry: number;
  faces: Face[];
  group: THREE.Group;
  /** Its faces, one draw call; none where the surfaces are not shown (a wireframe). */
  mesh: THREE.Mesh | null;
  edges: LineSegments2 | null;
  /** What of it is selected or under the pointer, tinted over its colour (`buildTints`): its
   *  geometry drawn again, in its group, so it moves as the object does. */
  tint: THREE.Mesh | null;
  /** Drawn from a build of an earlier drawing while its new one is made (#672): a ghost. */
  stale: boolean;
}

/** What the renderer drew in its last frame, and the triangles each object holds: what
 *  `tools/open-bench.mjs` reads once an opening has settled. */
export interface DrawCounts {
  calls: number;
  triangles: number;
  lines: number;
  geometries: number;
  textures: number;
  programs: number;
  objects: { index: number; meshes: number; lines: number; triangles: number }[];
  /** Geometries the box holds for its objects, which the objects of one binding share. */
  shared: number;
  /** Whether the silhouette pass ran. */
  silhouettes: boolean;
}

/** What the box has done since it was made — read by the tests, which have no WebGL to look at:
 *  how many objects it cut (built geometry for) and how many it only moved. */
export interface BoxStats {
  cut: number;
  moved: number;
  /** How many geometries it built: an object of a binding already held is cut from its peer's. */
  built: number;
  /** How many objects the last section was hung on: each a cut face from the core. */
  capped: number;
}

/** A fingerprint of where everything is: FNV-1a over the parameters' bits.  A drag frame changes
 *  it; a camera move does not. */
function poseOf(x: Float64Array): number {
  const w = new Uint32Array(x.buffer, x.byteOffset, x.length * 2);
  let h = 0x811c9dc5;
  for (let i = 0; i < w.length; i++) h = Math.imul(h ^ w[i], 0x01000193);
  return h >>> 0;
}

export class Box3D {
  private renderer: THREE.WebGLRenderer | null = null;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -1e4, 1e4);
  /** The panes, the axes and what stands in space — rebuilt as the pose moves. */
  private readonly datums = new THREE.Group();
  /** Each pane's name while an assembly is shown on its parts (#679), by its plane's name: kept
   *  across the datums' rebuilds, which a drag makes every frame, and only moved by them. */
  private readonly names = new THREE.Group();
  private readonly labels = new Map<string, Label>();
  /** The objects' meshes and edges, a group each (`placed`), moved by their matrices. */
  private readonly objects = new THREE.Group();
  /** Each object as the box holds it, by its index among the sketch's solids. */
  private readonly placed = new Map<number, Placed>();
  /** The geometries the objects are drawn from, by `Placement.geometry`. */
  private readonly shared = new Map<number, Shared>();
  /** **One material for each colour and each way of drawing it** — solid, a ghost, or dimmed
   *  round a part open (#679) — shared by every object of that colour, not one a face (#674). */
  private readonly materials = new Map<string, THREE.MeshStandardMaterial>();
  /** The objects' edges' one material: the ink is each segment's own (sharp, tangent) — and its
   *  twin, faint, for an object dimmed round a part open (#679). */
  private readonly edgeInk: LineMaterial;
  private readonly edgeDim: LineMaterial;
  readonly stats: BoxStats = { cut: 0, moved: 0, built: 0, capped: 0 };
  /** The objects cut since the last frame was rendered, marked (`gcs:drawn`) once it is. */
  private readonly drawnNow: { index: number; name: string; triangles: number }[] = [];
  private built: Built | null = null;
  /** The panes' outlines and the views' geometry, kept by what they are *of* — because the two
   *  things the app says about them change with the pointer and not with the drawing: which pane
   *  the cursor is on, and what is selected.  A material is written per frame; a rebuild is not,
   *  which is what keeps a mesh off the hover path. */
  private readonly frames: { it: Item3; line: THREE.LineSegments }[] = [];
  /** Each pane's frame, apart from `frames` because it is a fat line with a width to write. */
  private readonly panes: { it: Item3; line: LineSegments2 }[] = [];
  /** **What a Boolean being picked would make of its target** (#162 F4), shown in the target's
   *  place before anything is written: the core's preview of the body rule, rebuilt when the
   *  bodies picked change or the objects are re-cut, and nothing while no Boolean is pending. */
  private readonly previewed = new THREE.Group();
  /** The Boolean the preview is of — word, target and tools — and whether the core gave one: a
   *  refused preview (a body with a swept operand, never previewed) leaves the target standing
   *  and the tools as ghosts, which say as much. */
  private preview: { key: string | null; shown: boolean } = { key: null, shown: false };
  /** **The edges selected**, and the one under the pointer, lit over everything: rebuilt when what
   *  is picked or hovered changes, or the objects are re-cut. */
  private readonly litEdges = new THREE.Group();
  private litKey: string | null = null;
  /** **The faces selected**, and what is under the pointer, tinted over their part's colour
   *  (`Placed.tint`): rebuilt when what is picked or hovered changes, or an object is re-cut —
   *  what each face's tint is, by object, and what that was worked out from. */
  private tinted = new Map<number, (readonly [string, number] | null)[]>();
  private tintKey: string | null = null;
  /** Each object's edges, as the core gave them, kept until the objects are re-cut. */
  private readonly edgesOf = new Map<number, SolidEdge[]>();
  /** **The load scenario drawn** (#510): its marks and the drawing deformed under it. */
  readonly scenario = new ScenarioScene();
  /** **The section**: the plane every object's material is clipped by — the one array all of
   *  them share, holding it while the workspace is sectioned and empty while it is not — and the
   *  faces the cut leaves, a group an object, placed as the object is. */
  private readonly clip = new THREE.Plane();
  private readonly clipping: THREE.Plane[] = [];
  private readonly caps = new THREE.Group();
  /** **The silhouettes** (#675): the objects' view normals and depths drawn into a target of
   *  their own, then a pass over the picture inking where the depth steps — a curved face
   *  turning away, or one part standing in front of another.  Made with the renderer, and
   *  nothing where it cannot keep a float target. */
  private outline: { target: THREE.WebGLRenderTarget; depth: THREE.ShaderMaterial;
                     ink: THREE.ShaderMaterial; quad: THREE.Mesh } | null = null;
  private silhouetted = false;
  private readonly bufferSize = new THREE.Vector2();

  constructor(readonly canvas: HTMLCanvasElement | null) {
    this.scene.background = new THREE.Color(INK.bg);
    this.scene.add(this.datums, this.names, this.objects, this.previewed, this.litEdges,
                   this.caps, this.scenario.group);
    // **light that shows the shape** (#676): a sky over a ground, so every face is lit and an
    // upward one lighter; a key over the eye's left shoulder and a fill from the right, both
    // following the eye, so the object is read for its shape from wherever it is seen; and a
    // soft room (`start`), which gives a curved face its gradient
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8c92, 0.7));
    const light = (intensity: number, x: number, y: number, z: number): void => {
      const l = new THREE.DirectionalLight(0xffffff, intensity);
      l.position.set(x, y, z);
      this.camera.add(l, l.target);
    };
    light(1.1, -0.45, 0.55, 1);
    light(0.3, 0.8, -0.25, 0.6);
    this.scene.add(this.camera);
    this.edgeInk = new LineMaterial({
      color: 0xffffff, vertexColors: true, linewidth: EDGE_PX.ink, clippingPlanes: this.clipping,
    });
    this.edgeDim = new LineMaterial({
      color: 0xffffff, vertexColors: true, linewidth: EDGE_PX.ink, clippingPlanes: this.clipping,
      transparent: true, opacity: DIMMED,
    });
  }

  /** Draw the box for this view.  Rebuilds the scene only when the drawing or the zoom it was
   *  refined at has changed; otherwise it is a camera move and a draw call. */
  paint(v: SketchView): void {
    const gl = this.start();
    if (!gl) return;
    this.sync(v);
    const c = v.showSolid ? v.combining : null;
    const key = c && `${c.word} ${c.target.index} ${c.tools.map((t) => t.index).join(' ')}`;
    if (key !== this.preview.key) {
      this.buildPreview(v, c, key);
      this.paintParts(v);             // the target hidden where its preview now stands
    }
    const picked = v.selectedSolids.filter((s) => s.edge).map((s) => [s.index, s.edge]);
    const h = v.hovered?.kind === 'solid' && v.hovered.pick.edge ? v.hovered.pick : null;
    const lit = picked.length || h ? JSON.stringify([picked, h && [h.index, h.edge]]) : null;
    if (lit !== this.litKey) this.buildLit(v, lit);
    this.scenario.paint(v);
    this.aim(v);
    this.chrome(v);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (gl.getPixelRatio() !== dpr) gl.setPixelRatio(dpr);
    gl.setSize(v.width, v.height, false);
    this.silhouettes(gl, v, dpr);
    gl.render(this.scene, this.camera);
    for (const d of this.drawnNow) performance.mark('gcs:drawn', { detail: d });
    this.drawnNow.length = 0;
  }

  /** What the renderer drew in its last frame (`renderer.info`), and each object's triangles;
   *  null with no renderer. */
  counts(): DrawCounts | null {
    const gl = this.renderer;
    if (!gl) return null;
    const objects = [...this.placed].map(([index, held]) => ({
      index,
      meshes: held.mesh ? 1 : 0,
      lines: held.edges ? 1 : 0,
      triangles: (this.shared.get(held.geometry)?.faces.index?.count ?? 0) / 3,
    }));
    const { render, memory } = gl.info;
    return { calls: render.calls, triangles: render.triangles, lines: render.lines,
             geometries: memory.geometries, textures: memory.textures,
             programs: gl.info.programs?.length ?? 0, objects, shared: this.shared.size,
             silhouettes: this.silhouetted };
  }

  /** **Each object's colour as the box draws it**, `#rrggbb` by its index: its part's, or a
   *  Boolean's ghost's — what a test reads, having no WebGL to look at. */
  colours(): Map<number, string> {
    const out = new Map<number, string>();
    for (const [index, held] of this.placed) {
      const m = held.mesh?.material as THREE.MeshStandardMaterial | undefined;
      if (m) out.set(index, `#${m.color.getHexString()}`);
    }
    return out;
  }

  /** **What the box tints**, by object: each face's ink and strength, none for a face left
   *  alone — what a test reads. */
  tints(): Map<number, (readonly [string, number] | null)[]> {
    return this.tinted;
  }

  /** **The scene brought up to the drawing**, nothing drawn: what `paint` does before it renders,
   *  and what a test without WebGL asks to see what the box would hold. */
  sync(v: SketchView): void {
    // **The zoom is the camera's business and not the scene's**, which is why nothing here is a
    // function of `v.unit`: the scene and the mesh are both asked for at the *object's* own scale
    // (`0`), so a wheel tick moves the camera and rebuilds nothing.  Asked at the sheet's zoom
    // this re-evaluated the whole term on every tick — and finer without bound as you zoomed in,
    // since a screen pixel is a smaller world length the closer you get
    const pose = poseOf(v.sketch.getX());
    const fresh = !this.built || this.built.sketch !== v.sketch || this.built.solid !== v.showSolid;
    const shape = v.sketch.shapeEpoch;
    const shown = fresh || this.built!.hidden !== v.hiddenEpoch;
    const datums = shown || this.built!.datums !== pose;
    const objects = shown || this.built!.shape !== shape || this.built!.objects !== pose;
    const cut = fresh || this.built!.section !== v.sectionEpoch;
    if (datums || objects || cut) this.rebuild(v, { fresh, pose, shape, datums, objects, cut });
    this.paintParts(v);
  }

  /** What `sync` found changed, built again: the datums as the pose moved, the objects placed
   *  or cut, the section's caps. */
  private rebuild(v: SketchView, at: { fresh: boolean; pose: number; shape: number;
                                       datums: boolean; objects: boolean; cut: boolean }): void {
    const { fresh, pose, shape, datums, objects, cut } = at;
    // the section's plane stands where the pose puts it: told the core before anything is cut
    if (datums || cut) {
      v.applySection();
      this.clipTo(v);
    }
    if (datums) this.buildDatums(v, overview3(v.sketch, 0, 'datums'));
    let placedAt = this.built?.objects ?? NaN;
    if (objects) {
      // an object is cut again under a gesture only where its shape was changed in place, which
      // is what that gesture is for: one a drag re-shapes waits for the drag to be let go
      const settled = fresh || this.built!.shape !== shape || (!v.gesture && !v.anim);
      if (fresh) this.dropObjects();
      placedAt = this.placeObjects(v, settled) ? pose : NaN;
      this.preview.key = null;          // a preview is of the objects as they stood
      this.forgetLit();                 // and so are the edges lit on them
    }
    // the faces a section leaves, cut where every object now stands; a gesture that left one
    // waiting leaves the caps as they were too
    if ((objects || cut) && placedAt === pose) this.buildCaps(v);
    this.built = {
      sketch: v.sketch, solid: v.showSolid,
      datums: datums ? pose : this.built!.datums,
      objects: placedAt,
      shape,
      hidden: v.hiddenEpoch,
      section: v.sectionEpoch,
    };
  }

  /** Every object in its part's colour (`SketchView.partColours`), a Boolean's tools as ghosts in
   *  the ink of what they do, a solid the scenario deforms and one drawn from an earlier drawing's
   *  build while its new one is made (#672) as a ghost of its own colour.  A
   *  material is chosen, never made, per frame: one a colour is kept. */
  private paintParts(v: SketchView): void {
    const c = v.showSolid ? v.combining : null;
    const colours = v.partColours();
    // a solid the scenario drawn deforms shows through its deformation, at rest
    const deformed = this.scenario.ghosted(v);
    for (const [index, held] of this.placed) {
      if (!held.mesh) continue;
      const tool = c?.tools.some((t) => t.index === index);
      held.mesh.visible = !(c && this.preview.shown && c.target.index === index);
      const colour = tool ? INK[c!.word] : colours.get(index) ?? INK.solid;
      // a part open, the rest of the assembly is dimmed round it (#679)
      const m = this.material(colour,
        tool || deformed.has(index) || held.stale ? GHOST : dimmed(v, index) ? DIMMED : 1);
      if (held.mesh.material !== m) held.mesh.material = m;
    }
    // and an object dimmed is dimmed in its edges too
    for (const [index, held] of this.placed) {
      const ink = dimmed(v, index) ? this.edgeDim : this.edgeInk;
      if (held.edges && held.edges.material !== ink) held.edges.material = ink;
    }
    for (const s of this.shared.values()) {
      if (s.edges) s.edges.instanceCount = s.sharp + (v.showTangentEdges ? s.tangent : 0);
    }
    // what is selected or under the pointer, tinted over it — not over a ghost, nor over a
    // target the preview stands in for
    this.buildTints(v);
    for (const held of this.placed.values()) {
      if (held.tint && held.mesh) {
        held.tint.visible = held.mesh.visible && !(held.mesh.material as THREE.Material).transparent;
      }
    }
  }

  /** The material of `colour` at `alpha`: solid at one, and below it see-through and hiding
   *  nothing behind it — a ghost, or dimmed round a part open (#679).  Its faces stand a hair
   *  behind their depth, so an edge drawn on them wins its depth test against them without
   *  fighting, and a nearer face still hides it. */
  private material(colour: string | number, alpha: number): THREE.MeshStandardMaterial {
    const key = `${colour} ${alpha}`;
    const ghosted = alpha < 1;
    let m = this.materials.get(key);
    if (!m) {
      m = new THREE.MeshStandardMaterial({
        color: colour,
        ...SURFACE,
        clippingPlanes: this.clipping,
        polygonOffset: true,
        polygonOffsetFactor: 1,
        polygonOffsetUnits: 1,
        transparent: ghosted,
        opacity: alpha,
        depthWrite: !ghosted,
      });
      this.materials.set(key, m);
    }
    return m;
  }

  /** The plane the core cuts by, as every object's material clips: written into the one shared
   *  plane, and the array holding it filled or emptied. */
  private clipTo(v: SketchView): void {
    const p = clipPlane(v.sketch);
    this.clipping.length = 0;
    if (!p) return;
    this.clip.normal.set(p[0], p[1], p[2]);
    this.clip.constant = p[3];
    this.clipping.push(this.clip);
  }

  /** **The faces the section leaves**: each object shown cut by the core, its face hatched and
   *  its outline inked, in a group its matrix places.  Nothing while the workspace is not
   *  sectioned. */
  private buildCaps(v: SketchView): void {
    release(this.caps);
    this.stats.capped = 0;
    if (!this.clipping.length) return;
    for (const [index, held] of this.placed) {
      let cut: Sectioned | null;
      try {
        cut = sectioned(v.sketch, index, 0);
      } catch {
        continue;
      }
      if (!cut || (!cut.face.length && !cut.outline.length)) continue;
      const group = new THREE.Group();
      group.matrixAutoUpdate = false;
      group.matrix.copy(held.group.matrix);
      if (cut.face.length && v.showSolid) group.add(new THREE.Mesh(at(cut.face), hatched()));
      if (cut.outline.length) {
        group.add(new THREE.LineSegments(at(cut.outline),
                                         new THREE.LineBasicMaterial({ color: INK.edge })));
      }
      this.caps.add(group);
      this.stats.capped++;
    }
  }

  /** The drawing is the same object but what it shows changed (a swept solid's surface arrived):
   *  the next paint rebuilds the scene. */
  invalidate(): void {
    this.built = null;
  }

  /** Let the box go — the canvas keeps its context otherwise, and a browser only allows so many. */
  clear(): void {
    this.dispose();
    this.scenario.clear();
    this.built = null;
    this.renderer?.clear();
  }

  private start(): THREE.WebGLRenderer | null {
    if (this.renderer) return this.renderer;
    if (!this.canvas) return null;   // no box on this page: a test's view, which draws nothing
    try {
      // **the faster GPU where there are two**: Chrome 155 on an Intel UHD 630 (a 2018 MacBook
      // Pro, ANGLE over Metal) puts stale frames on screen, so a solid jumps between the poses of
      // past frames as the eye pans or turns; on the same machine's AMD it does not.  A browser
      // with one GPU ignores the preference
      this.renderer = new THREE.WebGLRenderer({
        canvas: this.canvas,
        antialias: true,
        powerPreference: 'high-performance',
      });
      // a section clips each object's own materials, never the panes or the axes
      this.renderer.localClippingEnabled = true;
      // the soft room every object reflects a little of: three's own, built here, no network
      const pmrem = new THREE.PMREMGenerator(this.renderer);
      this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
      this.scene.environmentIntensity = ROOM;
      pmrem.dispose();
      this.outline = silhouette(this.renderer);
      if (this.outline) this.scene.add(this.outline.quad);
      return this.renderer;
    } catch {
      // no WebGL: the box simply does not draw, and the status line has already said what the
      // box is.  Better than a dialog about a thing the drawing does not depend on
      return null;
    }
  }

  /** **The silhouette pass** (#675): the objects' faces alone (`FACES`) drawn as their view
   *  normals and depths into the outline's target, which the quad then reads in the main pass —
   *  one more draw of the faces and one of the screen, however many objects there are, and
   *  nothing asked of the core per view.  Skipped, the quad hidden, with no surfaces shown. */
  private silhouettes(gl: THREE.WebGLRenderer, v: SketchView, dpr: number): void {
    const o = this.outline;
    this.silhouetted = false;
    if (o && v.showSolid) for (const p of this.placed.values()) if (p.mesh) this.silhouetted = true;
    if (!o) return;
    o.quad.visible = this.silhouetted;
    if (!this.silhouetted) return;
    const size = gl.getDrawingBufferSize(this.bufferSize);
    if (o.target.width !== size.x || o.target.height !== size.y) o.target.setSize(size.x, size.y);
    const step = Math.max(1, Math.round(dpr));
    o.ink.uniforms.texel.value.set(step / size.x, step / size.y);
    // a step in depth worth inking, at the eye's scale: `v.unit` is a CSS pixel's world length
    o.ink.uniforms.jump.value = STEP_PX * v.unit * step / dpr;
    const background = this.scene.background;
    this.scene.background = null;
    this.scene.overrideMaterial = o.depth;
    this.camera.layers.set(FACES);
    gl.setRenderTarget(o.target);
    gl.setClearColor(0x000000, 0);
    gl.clear();
    gl.render(this.scene, this.camera);
    gl.setRenderTarget(null);
    this.camera.layers.set(0);
    this.scene.overrideMaterial = null;
    this.scene.background = background;
  }

  /** **The camera three.js draws with is the camera the picking already assumes.**
   *
   *  `space::eye` gives the same right/up basis the core flattens with, and `camera.ts` maps
   *  that plane to the screen.  Setting three's orthographic camera from the same two vectors and
   *  the same scale is what keeps a click landing on the thing under the cursor. */
  private aim(v: SketchView): void {
    const frame = eyeFrame(v.orbit.az, v.orbit.el);
    const eye = new THREE.Vector3(...frame.toward);
    const up = new THREE.Vector3(...frame.up);
    // the eye length of half the canvas, which is what `camera.ts` reads its scale as
    const [cx, cy] = v.cam.s2w(v.width / 2, v.height / 2);
    const centre = new THREE.Vector3(-cx, -cy, 0);
    const halfW = (v.width / 2) * v.unit;
    const halfH = (v.height / 2) * v.unit;
    const c = this.camera;
    c.left = -halfW;
    c.right = halfW;
    c.top = halfH;
    c.bottom = -halfH;
    c.near = -1e5;
    c.far = 1e5;
    c.up.copy(up);
    c.position.copy(eye);
    c.lookAt(0, 0, 0);
    c.updateProjectionMatrix();
    // the pan: the core projects about the world origin and `camera.ts` then translates, so the
    // same translation is applied here in the camera's own screen plane
    const right = new THREE.Vector3(...frame.right);
    c.position.addScaledVector(right, -centre.x);
    c.position.addScaledVector(up, -centre.y);
    c.updateMatrixWorld();
  }

  /** The datums, from what the core says is in the box: the panes, their axes, and what is drawn
   *  in space. */
  private buildDatums(v: SketchView, items: Item3[]): void {
    this.frames.length = 0;
    this.panes.length = 0;
    release(this.datums);
    // an assembly shown on its parts draws its datums quietly, each pane labelled (#679)
    const quiet = v.onParts;
    const named = new Map<string, [number, number, number]>();
    // the panes, translucent and double-sided: geometry on the far side of one must read through
    for (const it of items.filter((i) => i.part === 'face')) {
      const g = fanned(it.pts);
      if (!g) continue;
      this.datums.add(new THREE.Mesh(g, new THREE.MeshBasicMaterial({
        color: INK.pane,
        transparent: true,
        opacity: quiet ? QUIET.pane : 0.07,
        side: THREE.DoubleSide,
        depthWrite: false,
      })));
      // its frame, separately, because the frame is what is picked and what bolds under the
      // cursor — the rule everything on this canvas is picked by, and the wash is never it
      this.panes.push({ it, line: this.frame(it) });
      const pl = quiet && this.renderer ? v.planeOf(it) : null;
      const name = pl && v.doc.nameOf(pl);
      if (name) named.set(name, it.pts[3] ?? it.pts[0]);
    }
    this.nameAt(named);
    this.lines(this.datums, items.filter((i) => i.part === 'axis'), INK.axis,
               quiet ? QUIET.axis : 0.55);
    // of what is drawn, only what stands in space: a sketch on a plane is stroked on the canvas
    // above, through its plane's camera, with the selection and the state colours it carries
    for (const it of items.filter((i) => i.part === 'drawn' && IN_SPACE.has(i.kind ?? ''))) {
      this.frames.push({ it, line: this.lines(this.datums, [it], INK.drawn, 1)! });
    }
  }

  /** Each pane named at its corner (#679): a label already made moved there, one no longer
   *  wanted let go — its text drawn and uploaded once, not each frame a drag remakes the datums. */
  private nameAt(named: Map<string, [number, number, number]>): void {
    for (const [name, label] of this.labels) {
      if (named.has(name)) continue;
      dropLabel(label);
      this.labels.delete(name);
    }
    for (const [name, at] of named) {
      let label = this.labels.get(name) ?? null;
      if (!label) {
        label = textLabel(name, at, { font: LABEL_PX.size, pad: LABEL_PX.pad, ink: '#5f8f5f' });
        if (!label) continue;
        // drawn from its lower left, at the corner
        label.sprite.center.set(0, 0);
        label.sprite.renderOrder = 2;
        this.labels.set(name, label);
        this.names.add(label.sprite);
      }
      label.sprite.position.set(...at);
    }
  }

  /** **The objects, each where it stands** (#412).  An object the box holds whose shape the core
   *  still has — moved rigidly or not at all — keeps its geometry and takes its new matrix; any
   *  other is cut: drawn from the geometry it shares with its binding's objects where the box
   *  holds that already, else from a geometry built for it (#674).  Its edges stand whether or
   *  not its surfaces are drawn (with them the corners of a shaded part, without them the
   *  wireframe).  Unsettled (a gesture), an object the core would have to build is left as it
   *  stood.  Whether every object is where the drawing now puts it. */
  private placeObjects(v: SketchView, settled: boolean): boolean {
    let current = true;
    // what the outline hid is not drawn (#435)
    const now = shownObjects(v.sketch);
    const keep = new Set(now.map((o) => o.index));
    for (const index of [...this.placed.keys()]) if (!keep.has(index)) this.unplace(index);
    for (const o of now) {
      // **cut to the object and not to the zoom** — `0` asks the core for `solid::mesh_unit`,
      // a sagitta a fixed fraction of the solid's own diagonal.  Handed `v.unit` this re-cut the
      // whole boundary on every wheel tick, and finer without bound as you zoomed in: one tick
      // on the V-twin cylinder locked the tab for the better part of a minute.  An object that
      // does not evaluate (a revolution whose axis crosses its profile) is left out: thrown here,
      // it stopped every paint before the drawing, and the page with it
      let at: Placement | null;
      try {
        at = placement(v.sketch, o.index, 0, !settled);
      } catch {
        this.unplace(o.index);
        continue;
      }
      if (!at) {
        current = false;
        continue;
      }
      let held = this.placed.get(o.index);
      if (held?.shape === at.shape) {
        this.stats.moved++;
      } else {
        this.unplace(o.index);
        try {
          held = this.cut(v, o.index, o.name, at);
        } catch {
          continue;
        }
        this.placed.set(o.index, held);
        this.objects.add(held.group);
        this.stats.cut++;
      }
      held.group.matrix.fromArray(at.matrix);
      held.group.matrixWorldNeedsUpdate = true;
    }
    return current;
  }

  /** One object, in its own coordinates, in a group its matrix places: its faces as one mesh and
   *  its edges as one fat line set, both drawn from the geometry its binding's objects share. */
  private cut(v: SketchView, solid: number, name: string, at: Placement): Placed {
    let s = this.shared.get(at.geometry);
    let named: Face[];
    if (s) {
      named = faces(v.sketch, solid, 0);
    } else {
      const d = drawn(v.sketch, solid, 0);
      named = d.faces;
      s = share(d);
      this.shared.set(at.geometry, s);
      this.stats.built++;
    }
    s.users++;
    const group = new THREE.Group();
    group.matrixAutoUpdate = false;
    const triangles = (s.faces.index?.count ?? 0) / 3;
    if (this.renderer) this.drawnNow.push({ index: solid, name, triangles });
    const held: Placed = { shape: at.shape, geometry: at.geometry, faces: named, group,
                           mesh: null, edges: null, tint: null, stale: stale(v.sketch, solid) };
    if (s.edges) {
      held.edges = new LineSegments2(s.edges, this.edgeInk);
      group.add(held.edges);
    }
    if (!v.showSolid) return held;
    // **the object itself, one mesh** — its faces named by the core's face table, so a pick and
    // a tint read them by range and the scene needs no mesh a face
    held.mesh = new THREE.Mesh(s.faces, this.material(INK.solid, 1));
    held.mesh.layers.enable(FACES);
    group.add(held.mesh);
    return held;
  }

  /** Let one object go, and its geometry with the last object drawn from it. */
  private unplace(solid: number): void {
    const held = this.placed.get(solid);
    if (!held) return;
    this.objects.remove(held.group);
    this.placed.delete(solid);
    if (held.tint) {
      untint(held.tint);
      this.tintKey = null;            // what it was tinted is worked out again
    }
    const s = this.shared.get(held.geometry);
    if (s && --s.users === 0) {
      s.faces.dispose();
      s.edges?.dispose();
      this.shared.delete(held.geometry);
    }
  }

  /** Let every object's geometry go: a different drawing, or its surfaces shown or hidden. */
  private dropObjects(): void {
    for (const index of [...this.placed.keys()]) this.unplace(index);
  }

  /** The result of the Boolean being picked, from the core, standing where its target does. */
  private buildPreview(v: SketchView, c: Combining | null, key: string | null): void {
    release(this.previewed);
    this.preview = { key, shown: false };
    if (!c) return;
    let m: Mesh;
    try {
      m = preview(v.sketch, c.target.index, c.word, c.tools.map((t) => t.index));
    } catch {
      return;
    }
    // lit as its target is, being what the selected body will be
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(Float32Array.from(m.positions), 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute(Float32Array.from(m.normals), 3));
    const colour = v.partColours().get(c.target.index) ?? INK.solid;
    this.previewed.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({
      color: colour, ...SURFACE,
      emissive: new THREE.Color(COL.sel), emissiveIntensity: 0.18, clippingPlanes: this.clipping,
    })));
    this.preview.shown = true;
  }

  /** The edges selected, and the one under the pointer, as fat lines drawn over the material:
   *  the core's stretches of each, read once per change. */
  private buildLit(v: SketchView, key: string | null): void {
    release(this.litEdges);
    this.litKey = key;
    const draw = (solid: number, pairs: [string, string][], color: string, px: number) => {
      if (!this.edgesOf.has(solid)) this.edgesOf.set(solid, solidEdges(v.sketch, solid));
      const pts: number[] = [];
      for (const e of this.edgesOf.get(solid)!) {
        if (!pairs.some(([a, b]) => e.faces[0] === a && e.faces[1] === b)) continue;
        for (const seg of e.segs) pts.push(...seg);
      }
      if (!pts.length) return;
      const g = new LineSegmentsGeometry();
      g.setPositions(pts);
      const m = new LineMaterial({ color, linewidth: px, depthTest: false, transparent: true,
                                   clippingPlanes: this.clipping });
      m.resolution.set(v.width, v.height);
      const line = new LineSegments2(g, m);
      line.renderOrder = 10;
      this.litEdges.add(line);
    };
    const picked = v.selectedSolids.filter((s) => s.edge);
    for (const solid of new Set(picked.map((s) => s.index))) {
      draw(solid, picked.filter((s) => s.index === solid).map((s) => s.edge!), COL.sel,
           EDGE_PX.picked);
    }
    // under the pointer, unless it is selected: what is held keeps the selection's ink
    const h = v.hovered?.kind === 'solid' ? v.hovered.pick : null;
    if (h?.edge && !picked.some((s) => samePick(s, h))) {
      draw(h.index, [h.edge], COL.highlight, EDGE_PX.hover);
    }
  }

  /** The lit edges let go, with the edges they were read from: the objects were re-cut or
   *  moved. */
  private forgetLit(): void {
    release(this.litEdges);
    this.litKey = null;
    this.edgesOf.clear();
  }

  /** **What is selected and under the pointer, tinted** over each object's own colour (#676): a
   *  face selected in the selection's ink, a body selected all over a little less, and what is
   *  under the pointer in the highlight's.  Each object with something to tint is drawn once
   *  more from its own geometry, every face its tint from a texture its vertices index
   *  (`Drawn.face`); rebuilt only when the tints change. */
  private buildTints(v: SketchView): void {
    // what is tinted, as small as it is said: the faces and bodies picked, and the hover
    const h = v.hovered?.kind === 'solid' ? v.hovered.pick : null;
    const asked = [...v.selectedSolids, ...(h ? [h] : [])]
      .map((s) => [s.index, s.face, partOf(s) === 'body'] as const);
    const key = asked.length ? JSON.stringify([asked, !!h]) : null;
    if (key === this.tintKey) return;
    this.tintKey = key;
    const tints = new Map<number, (readonly [string, number] | null)[]>();
    const tint = (solid: number, face: string | null, ink: string, glow: number): void => {
      const held = this.placed.get(solid);
      if (!held?.mesh) return;
      let t = tints.get(solid);
      if (!t) tints.set(solid, t = held.faces.map(() => null));
      held.faces.forEach((f, k) => {
        if (face !== null && f.path !== face) return;
        if (!t![k] || t![k]![1] < glow) t![k] = [ink, glow];
      });
    };
    for (const s of v.selectedSolids) {
      if (s.face) tint(s.index, s.face, COL.sel, GLOW.face);
      else if (partOf(s) === 'body') tint(s.index, null, COL.sel, GLOW.body);
    }
    if (h?.face) tint(h.index, h.face, COL.highlight, GLOW.hover);
    else if (h && partOf(h) === 'body') tint(h.index, null, COL.highlight, GLOW.hoverBody);
    this.tinted = tints;
    for (const [solid, held] of this.placed) {
      if (held.tint) {
        held.group.remove(held.tint);
        untint(held.tint);
        held.tint = null;
      }
      const faces = tints.get(solid);
      if (!faces || !held.mesh) continue;
      held.tint = new THREE.Mesh(held.mesh.geometry, tinted(faces, this.clipping));
      held.tint.renderOrder = 4;
      held.group.add(held.tint);
    }
  }

  /** **What the app says about the scene, written per frame.**  The selection and the plane being
   *  drawn on change without the drawing changing, so they are a material write and never a
   *  rebuild: the ink rule itself is `paint.ts`'s, so a thing picked is lit the same colour here
   *  as on the canvas above. */
  private chrome(v: SketchView): void {
    const sel = new Set(v.selected);
    const hl = new Set(v.lit);
    // std's datums are the document's frame of reference, held outright: they keep their quiet
    // ink, so only what the drawing adds is coloured by its state
    const ink = (it: Item3, base: string | number): string | number => {
      const ent = v.entityOf(it);
      if (!ent) return base;
      const own = !v.doc.nameOf(ent)?.startsWith('std.');
      return (own ? inkOf(v, sel, hl, ent) : chromeOf(sel, hl, ent)?.[0]) ?? base;
    };
    for (const { it, line } of this.frames) {
      (line.material as THREE.LineBasicMaterial).color.set(ink(it, INK.drawn));
    }
    // a pane is a place's (`std.up` has none of its own: it stands where `std.front` does), so
    // the one that bolds is the place of the plane being drawn on
    const ws = v.workspace();
    const here = v.plane ? placeOf(ws, v.plane.index) : null;
    for (const { it, line } of this.panes) {
      const m = line.material;
      m.color.set(ink(it, INK.pane));
      // the plane being drawn on: its frame bolds, so where the next thing goes is in sight
      const pl = v.planeOf(it);
      const current = here !== null && pl !== null && placeOf(ws, pl.index) === here;
      m.opacity = current ? 0.8 : v.onParts ? QUIET.frame : 0.35;
      m.linewidth = current ? FRAME_PX.current : FRAME_PX.plain;
      m.resolution.set(v.width, v.height);
    }
    // (each object's colour and tint is `sync`'s, which has just run)
    this.edgeInk.resolution.set(v.width, v.height);
    this.edgeDim.resolution.set(v.width, v.height);
    // a pane's name keeps its size as the eye zooms (#679)
    scaleLabels(this.labels.values(), v.unit);
    for (const line of this.litEdges.children as LineSegments2[]) {
      line.material.resolution.set(v.width, v.height);
    }
  }

  /** A pane's frame as a closed fat line; `chrome` writes its colour, opacity and width. */
  private frame(it: Item3): LineSegments2 {
    const pts: number[] = [];
    const n = it.pts.length;
    for (let i = 0; i + 1 < n; i++) pts.push(...it.pts[i], ...it.pts[i + 1]);
    // the core's face comes back to its first corner; closed here should one not
    const [a, b] = [it.pts[0], it.pts[n - 1]];
    if (n > 2 && a.some((x, k) => x !== b[k])) pts.push(...b, ...a);
    const g = new LineSegmentsGeometry();
    g.setPositions(pts);
    const line = new LineSegments2(g, new LineMaterial({
      color: INK.pane,
      linewidth: FRAME_PX.plain,
      transparent: true,
      opacity: 0.35,
    }));
    this.datums.add(line);
    return line;
  }

  private lines(into: THREE.Group, items: Item3[], color: number,
                opacity: number): THREE.LineSegments | null {
    const pts: number[] = [];
    for (const it of items) {
      const n = it.pts.length;
      for (let i = 0; i + 1 < n; i++) pts.push(...it.pts[i], ...it.pts[i + 1]);
    }
    if (pts.length === 0) return null;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
    const line = new THREE.LineSegments(g, new THREE.LineBasicMaterial({
      color,
      transparent: true,   // the chrome pass writes an opacity, so every line carries one
      opacity,
    }));
    into.add(line);
    return line;
  }

  /** Free every buffer and material the last scene made. */
  private dispose(): void {
    this.frames.length = 0;
    this.panes.length = 0;
    release(this.datums);
    this.nameAt(new Map());
    this.dropObjects();
    for (const m of this.materials.values()) m.dispose();
    this.materials.clear();
    release(this.previewed);
    this.preview = { key: null, shown: false };
    this.forgetLit();
    release(this.caps);
  }
}

/** A geometry the box holds, from the core's `Drawn`: positions, normals and each vertex's face
 *  over one index — the normals the core's, averaged across a round face and its own across a
 *  flat one, so a bore's wall shades round with no smoothing pass here — and its edges as one fat
 *  line set, sharp first and inked dark, tangent after and lighter. */
function share(d: Drawn): Shared {
  const faces = new THREE.BufferGeometry();
  faces.setAttribute('position', new THREE.BufferAttribute(d.positions, 3));
  faces.setAttribute('normal', new THREE.BufferAttribute(d.normals, 3));
  faces.setAttribute('face', new THREE.BufferAttribute(d.face, 1));
  faces.setIndex(new THREE.BufferAttribute(d.index, 1));
  faces.computeBoundingSphere();
  const sharp = d.sharp.length / 6, tangent = d.tangent.length / 6;
  let edges: LineSegmentsGeometry | null = null;
  if (sharp + tangent) {
    edges = new LineSegmentsGeometry();
    const pts = new Float32Array(6 * (sharp + tangent));
    pts.set(d.sharp);
    pts.set(d.tangent, d.sharp.length);
    edges.setPositions(pts);
    const ink = new Float32Array(pts.length);
    const [dark, light] = [new THREE.Color(INK.edge), new THREE.Color(INK.tangent)];
    for (let k = 0; k < 2 * (sharp + tangent); k++) {
      const c = k < 2 * sharp ? dark : light;
      ink[3 * k] = c.r;
      ink[3 * k + 1] = c.g;
      ink[3 * k + 2] = c.b;
    }
    edges.setColors(ink);
  }
  return { faces, edges, sharp, tangent, users: 0 };
}

/** **The tint over one object's faces**: a texture of a texel a face — its ink and how strongly,
 *  none for a face left alone — read at each vertex by the face it is of, so every face of the
 *  object takes its own from one draw.  A hair toward the eye, so it wins against the faces it
 *  lies on, and clipped as they are. */
function tinted(faces: (readonly [string, number] | null)[],
                clippingPlanes: THREE.Plane[]): THREE.ShaderMaterial {
  const w = Math.max(1, Math.min(faces.length, 1024));
  const h = Math.max(1, Math.ceil(faces.length / w));
  const texels = new Uint8Array(4 * w * h);
  const c = new THREE.Color();
  faces.forEach((t, k) => {
    if (!t) return;
    c.set(t[0]);
    texels.set([c.r * 255, c.g * 255, c.b * 255, t[1] * 255].map(Math.round), 4 * k);
  });
  const lit = new THREE.DataTexture(texels, w, h);
  lit.needsUpdate = true;
  return new THREE.ShaderMaterial({
    uniforms: { lit: { value: lit }, size: { value: new THREE.Vector2(w, h) } },
    vertexShader: `attribute float face;
    uniform sampler2D lit;
    uniform vec2 size;
    varying vec4 vTint;
    #include <common>
    #include <clipping_planes_pars_vertex>
    void main() {
      vec2 at = vec2(mod(face, size.x) + 0.5, floor(face / size.x) + 0.5) / size;
      vTint = texture2D(lit, at);
      vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
      gl_Position = projectionMatrix * mvPosition;
      #include <clipping_planes_vertex>
    }`,
    fragmentShader: `varying vec4 vTint;
    #include <clipping_planes_pars_fragment>
    void main() {
      #include <clipping_planes_fragment>
      if (vTint.a <= 0.0) discard;
      gl_FragColor = vTint;
      #include <colorspace_fragment>
    }`,
    transparent: true,
    depthWrite: false,
    depthFunc: THREE.LessEqualDepth,
    polygonOffset: true,
    polygonOffsetFactor: -1,
    polygonOffsetUnits: -1,
    clipping: true,
    clippingPlanes,
  });
}

/** A tint let go: its material and texture — never the geometry, which is the object's. */
function untint(t: THREE.Mesh): void {
  const m = t.material as THREE.ShaderMaterial;
  (m.uniforms.lit.value as THREE.Texture).dispose();
  m.dispose();
}

/** **The silhouette pass's parts** (#675), or null where the renderer cannot draw into a float
 *  target: the target the objects' view normals and depths are drawn into, the material they are
 *  drawn with, and the quad over the picture whose material inks — on the near side, one pixel
 *  in — wherever a covered pixel has the ground beside it or its depth steps by more than
 *  `jump` across it: a curved face's outline, or a part's against another behind it. */
function silhouette(gl: THREE.WebGLRenderer): Box3D['outline'] {
  if (!gl.capabilities.isWebGL2 && !gl.extensions.has('OES_texture_float')) return null;
  if (!gl.extensions.has('EXT_color_buffer_float')) return null;
  const target = new THREE.WebGLRenderTarget(1, 1, {
    type: THREE.FloatType, format: THREE.RGBAFormat, depthBuffer: true,
    minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter,
  });
  const depth = new THREE.ShaderMaterial({
    vertexShader: `varying vec3 vNormal;
    varying float vDepth;
    #include <common>
    #include <clipping_planes_pars_vertex>
    void main() {
      vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
      vNormal = normalize(normalMatrix * normal);
      vDepth = -mvPosition.z;
      gl_Position = projectionMatrix * mvPosition;
      #include <clipping_planes_vertex>
    }`,
    fragmentShader: `varying vec3 vNormal;
    varying float vDepth;
    #include <clipping_planes_pars_fragment>
    void main() {
      #include <clipping_planes_fragment>
      gl_FragColor = vec4(normalize(vNormal), vDepth);
    }`,
    clipping: true,
  });
  const ink = new THREE.ShaderMaterial({
    uniforms: {
      map: { value: target.texture },
      texel: { value: new THREE.Vector2(1, 1) },
      jump: { value: 1 },
      ink: { value: new THREE.Color(INK.edge) },
    },
    vertexShader: `varying vec2 vUv;
    void main() {
      vUv = uv;
      gl_Position = vec4(position.xy, 0.0, 1.0);
    }`,
    fragmentShader: `uniform sampler2D map;
    uniform vec2 texel;
    uniform float jump;
    uniform vec3 ink;
    varying vec2 vUv;
    bool covered(vec4 s) { return dot(s.xyz, s.xyz) > 0.25; }
    void main() {
      vec4 c = texture2D(map, vUv);
      if (!covered(c)) discard;
      vec4 l = texture2D(map, vUv - vec2(texel.x, 0.0));
      vec4 r = texture2D(map, vUv + vec2(texel.x, 0.0));
      vec4 d = texture2D(map, vUv - vec2(0.0, texel.y));
      vec4 u = texture2D(map, vUv + vec2(0.0, texel.y));
      bool edge = !covered(l) || !covered(r) || !covered(d) || !covered(u)
        || l.w + r.w - 2.0 * c.w > jump || d.w + u.w - 2.0 * c.w > jump;
      if (!edge) discard;
      gl_FragColor = vec4(ink, 1.0);
      #include <colorspace_fragment>
    }`,
    transparent: true,
    depthTest: false,
    depthWrite: false,
  });
  const quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), ink);
  quad.frustumCulled = false;
  quad.renderOrder = 5;
  return { target, depth, ink, quad };
}

/** Empty a group and free what it held, groups within it too.  A `Group` emptied is not a
 *  `Group` released: WebGL resources outlive the tree that named them.  Never an object's: what
 *  objects share is the box's own maps' to let go (`unplace`, `dispose`). */
function release(g: THREE.Group): void {
  for (const o of [...g.children]) {
    g.remove(o);
    if (o instanceof THREE.Group) {
      release(o);
      continue;
    }
    const any = o as THREE.Mesh;
    any.geometry?.dispose?.();
    const m = any.material;
    if (Array.isArray(m)) m.forEach((x) => x.dispose());
    else m?.dispose?.();
  }
}

/** A geometry of nothing but the core's points, three doubles each. */
function at(points: Float64Array): THREE.BufferGeometry {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(Float32Array.from(points), 3));
  return g;
}

/** **A cut face's material**: its ink, crossed by hatching laid in the picture's own pixels —
 *  a draughtsman's section lines, the same pitch however near the eye is — drawn a hair toward
 *  the eye, so a face of the object lying in the plane does not show through it.  Both sides, as
 *  a mirrored object's face turns the other way. */
function hatched(): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    uniforms: {
      ink: { value: new THREE.Color(INK.section) },
      hatch: { value: new THREE.Color(INK.hatch) },
      pitch: { value: HATCH.pitch },
      width: { value: HATCH.ink },
    },
    vertexShader: `void main() {
      gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    }`,
    fragmentShader: `uniform vec3 ink;
    uniform vec3 hatch;
    uniform float pitch;
    uniform float width;
    void main() {
      float k = mod(gl_FragCoord.x + gl_FragCoord.y, pitch);
      gl_FragColor = vec4(k < width ? hatch : ink, 1.0);
      #include <colorspace_fragment>
    }`,
    side: THREE.DoubleSide,
    polygonOffset: true,
    polygonOffsetFactor: -1,
    polygonOffsetUnits: -1,
  });
}

/** Whether object `solid` is dimmed: a part is open (#679) that did not make it. */
function dimmed(v: SketchView, solid: number): boolean {
  return !!v.inside && !v.inside.solids.has(solid);
}

/** A convex polygon as a triangle fan, which is what a pane is.  The one place this file makes a
 *  triangle, and it makes it out of points the core placed. */
function fanned(pts: [number, number, number][]): THREE.BufferGeometry | null {
  if (pts.length < 3) return null;
  const out: number[] = [];
  for (let i = 1; i + 1 < pts.length; i++) out.push(...pts[0], ...pts[i], ...pts[i + 1]);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(out, 3));
  g.computeVertexNormals();
  return g;
}
