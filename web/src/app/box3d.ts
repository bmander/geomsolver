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
 */
import * as THREE from 'three';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { LineSegments2 } from 'three/addons/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/addons/lines/LineSegmentsGeometry.js';

import { objects, own, placement, preview } from '../core/mesh.js';
import type { Face, Mesh, Placement } from '../core/mesh.js';
import { overview3 } from '../core/overview.js';
import { eyeFrame, placeOf, solidEdges } from '../core/workspace.js';
import type { SolidEdge } from '../core/workspace.js';
import type { Item3 } from '../core/overview.js';
import { COL, chromeOf, inkOf } from './paint.js';
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
  edge: 0x3a3f45,
  solid: 0xb0b3b8,
  /** A Boolean's tools, ghosted in the colour of what they do to the body (#162 F4). */
  union: 0x2ca02c,
  cut: 0xd62728,
  common: 0x1f77b4,
};

/** How much of a Boolean's tool shows through as a ghost. */
const GHOST = 0.4;

/** A pane's frame, in screen pixels: the plane being drawn on is bold.  WebGL draws every
 *  `LineBasicMaterial` one pixel wide whatever it is asked, so a frame is a fat line. */
const FRAME_PX = { plain: 1, current: 2.5 };

/** An object's edge selected, or under the pointer, in pixels. */
const EDGE_PX = { picked: 3.5, hover: 2.5 };

/** How strongly a face glows in the ink of what is said of it: selected, its body selected, or
 *  under the pointer (itself, or its body). */
const GLOW = { face: 0.55, body: 0.3, hover: 0.35, hoverBody: 0.22 };

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
}

/** One object in the box: the evaluation its geometry was built from (`Placement.shape`), and
 *  the group that geometry hangs in, placed by the object's matrix. */
interface Placed {
  shape: number;
  group: THREE.Group;
}

/** What the box has done since it was made — read by the tests, which have no WebGL to look at:
 *  how many objects it cut (built geometry for) and how many it only moved. */
export interface BoxStats {
  cut: number;
  moved: number;
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
  /** The objects' meshes and creases, a group each (`placed`), moved by their matrices. */
  private readonly objects = new THREE.Group();
  /** Each object as the box holds it, by its index among the sketch's solids. */
  private readonly placed = new Map<number, Placed>();
  readonly stats: BoxStats = { cut: 0, moved: 0 };
  private built: Built | null = null;
  /** The panes' outlines and the views' geometry, kept by what they are *of* — because the two
   *  things the app says about them change with the pointer and not with the drawing: which pane
   *  the cursor is on, and what is selected.  A material is written per frame; a rebuild is not,
   *  which is what keeps a mesh off the hover path. */
  private readonly frames: { it: Item3; line: THREE.LineSegments }[] = [];
  /** Each pane's frame, apart from `frames` because it is a fat line with a width to write. */
  private readonly panes: { it: Item3; line: LineSegments2 }[] = [];
  /** Each object face's mesh, by the solid it is a face of and its path, for `chrome` to light. */
  private readonly faces: { solid: number; path: string; mesh: THREE.Mesh }[] = [];
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
  /** Each object's edges, as the core gave them, kept until the objects are re-cut. */
  private readonly edgesOf = new Map<number, SolidEdge[]>();

  constructor(readonly canvas: HTMLCanvasElement | null) {
    this.scene.background = new THREE.Color(INK.bg);
    this.scene.add(this.datums, this.objects, this.previewed, this.litEdges);
    // a headlight and a soft fill: the object is read for its shape, so the light follows the
    // eye rather than standing somewhere in the world and leaving half of it dark
    this.scene.add(new THREE.AmbientLight(0xffffff, 1.6));
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(-0.45, 0.35, 1);
    this.camera.add(key);
    this.scene.add(this.camera);
  }

  /** Draw the box for this view.  Rebuilds the scene only when the drawing or the zoom it was
   *  refined at has changed; otherwise it is a camera move and a draw call. */
  paint(v: SketchView): void {
    const gl = this.start();
    if (!gl) return;
    this.sync(v);
    const c = v.showSolid ? v.combining : null;
    const key = c && `${c.word} ${c.target.index} ${c.tools.map((t) => t.index).join(' ')}`;
    if (key !== this.preview.key) this.buildPreview(v, c, key);
    const picked = v.selectedSolids.filter((s) => s.edge).map((s) => [s.index, s.edge]);
    const h = v.hovered?.kind === 'solid' && v.hovered.pick.edge ? v.hovered.pick : null;
    const lit = picked.length || h ? JSON.stringify([picked, h && [h.index, h.edge]]) : null;
    if (lit !== this.litKey) this.buildLit(v, lit);
    this.aim(v);
    this.chrome(v, c);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (gl.getPixelRatio() !== dpr) gl.setPixelRatio(dpr);
    gl.setSize(v.width, v.height, false);
    gl.render(this.scene, this.camera);
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
    const datums = fresh || this.built!.datums !== pose;
    const objects = fresh || this.built!.shape !== shape || this.built!.objects !== pose;
    if (!datums && !objects) return;
    if (datums) this.buildDatums(overview3(v.sketch, 0, 'datums'));
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
    this.built = {
      sketch: v.sketch, solid: v.showSolid,
      datums: datums ? pose : this.built!.datums,
      objects: placedAt,
      shape,
    };
  }

  /** The drawing is the same object but what it shows changed (a swept solid's surface arrived):
   *  the next paint rebuilds the scene. */
  invalidate(): void {
    this.built = null;
  }

  /** Let the box go — the canvas keeps its context otherwise, and a browser only allows so many. */
  clear(): void {
    this.dispose();
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
      return this.renderer;
    } catch {
      // no WebGL: the box simply does not draw, and the status line has already said what the
      // box is.  Better than a dialog about a thing the drawing does not depend on
      return null;
    }
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
  private buildDatums(items: Item3[]): void {
    this.frames.length = 0;
    this.panes.length = 0;
    release(this.datums);
    // the panes, translucent and double-sided: geometry on the far side of one must read through
    for (const it of items.filter((i) => i.part === 'face')) {
      const g = fanned(it.pts);
      if (!g) continue;
      this.datums.add(new THREE.Mesh(g, new THREE.MeshBasicMaterial({
        color: INK.pane,
        transparent: true,
        opacity: 0.07,
        side: THREE.DoubleSide,
        depthWrite: false,
      })));
      // its frame, separately, because the frame is what is picked and what bolds under the
      // cursor — the rule everything on this canvas is picked by, and the wash is never it
      this.panes.push({ it, line: this.frame(it) });
    }
    this.lines(this.datums, items.filter((i) => i.part === 'axis'), INK.axis, 0.55);
    // of what is drawn, only what stands in space: a sketch on a plane is stroked on the canvas
    // above, through its plane's camera, with the selection and the state colours it carries
    for (const it of items.filter((i) => i.part === 'drawn' && IN_SPACE.has(i.kind ?? ''))) {
      this.frames.push({ it, line: this.lines(this.datums, [it], INK.drawn, 1)! });
    }
  }

  /** **The objects, each where it stands** (#412).  An object the box holds whose shape the core
   *  still has — moved rigidly or not at all — keeps its geometry and takes its new matrix; any
   *  other is cut: its creases, which stand whether or not its surfaces are drawn (with them the
   *  corners of a shaded part, without them the wireframe), and, shown, its surfaces, all in its
   *  own coordinates.  Unsettled (a gesture), an object the core would have to build is left as
   *  it stood.  Whether every object is where the drawing now puts it. */
  private placeObjects(v: SketchView, settled: boolean): boolean {
    let current = true;
    const now = objects(v.sketch);
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
        held = { shape: at.shape, group: this.cut(v, o.index) };
        this.placed.set(o.index, held);
        this.objects.add(held.group);
        this.stats.cut++;
      }
      held.group.matrix.fromArray(at.matrix);
      held.group.matrixWorldNeedsUpdate = true;
    }
    return current;
  }

  /** One object's geometry, in its own coordinates, in a group its matrix places. */
  private cut(v: SketchView, solid: number): THREE.Group {
    const group = new THREE.Group();
    group.matrixAutoUpdate = false;
    const m = own(v.sketch, solid, 0);
    if (m.creases.length) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(Float32Array.from(m.creases), 3));
      group.add(new THREE.LineSegments(g, new THREE.LineBasicMaterial({
        color: INK.edge, transparent: true, opacity: 1,
      })));
    }
    if (!v.showSolid) return group;
    // **the object itself, as a mesh** — every face of it a named child, so a later selection has
    // something to name
    for (const f of m.faces) {
      const face = faceMesh(m, f);
      group.add(face);
      this.faces.push({ solid, path: f.path, mesh: face });
    }
    return group;
  }

  /** Let one object's geometry go. */
  private unplace(solid: number): void {
    const held = this.placed.get(solid);
    if (!held) return;
    this.objects.remove(held.group);
    for (let i = this.faces.length - 1; i >= 0; i--) {
      if (this.faces[i].solid === solid) this.faces.splice(i, 1);
    }
    release(held.group);
    this.placed.delete(solid);
  }

  /** Let every object's geometry go: a different drawing, or its surfaces shown or hidden. */
  private dropObjects(): void {
    this.faces.length = 0;
    this.placed.clear();
    release(this.objects);
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
    for (const f of m.faces) {
      const face = faceMesh(m, f);
      // lit as its target is, being what the selected body will be
      const mat = face.material as THREE.MeshStandardMaterial;
      mat.emissive.set(COL.sel);
      mat.emissiveIntensity = 0.18;
      this.previewed.add(face);
    }
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
      const m = new LineMaterial({ color, linewidth: px, depthTest: false, transparent: true });
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

  /** The lit edges let go, with the edges they were read from: the objects were re-cut. */
  private forgetLit(): void {
    release(this.litEdges);
    this.litKey = null;
    this.edgesOf.clear();
  }

  /** **What the app says about the scene, written per frame.**  The selection and the plane being
   *  drawn on change without the drawing changing, so they are a material write and never a
   *  rebuild: the ink rule itself is `paint.ts`'s, so a thing picked is lit the same colour here
   *  as on the canvas above. */
  private chrome(v: SketchView, c: Combining | null): void {
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
      m.opacity = current ? 0.8 : 0.35;
      m.linewidth = current ? FRAME_PX.current : FRAME_PX.plain;
      m.resolution.set(v.width, v.height);
    }
    // a face selected glows in the selection's ink, a body selected all over a little less; what
    // is under the pointer glows in the highlight's.  A Boolean being picked shows its result in
    // its target's place, and its tools as ghosts in the colour of what they do
    // the selection by object, once a frame: the faces picked, and whether the body whole is
    const picked = new Map<number, { faces: Set<string>; body: boolean }>();
    for (const s of v.selectedSolids) {
      const o = picked.get(s.index) ?? { faces: new Set<string>(), body: false };
      if (s.face) o.faces.add(s.face);
      o.body ||= partOf(s) === 'body';
      picked.set(s.index, o);
    }
    const h = v.hovered?.kind === 'solid' ? v.hovered.pick : null;
    for (const { solid, path, mesh: face } of this.faces) {
      const m = face.material as THREE.MeshStandardMaterial;
      const tool = c?.tools.some((t) => t.index === solid);
      face.visible = !(c && this.preview.shown && c.target.index === solid);
      ghost(m, !!tool);
      m.color.set(tool ? INK[c!.word] : INK.solid);
      const mine = picked.get(solid);
      const [ink, glow]: [string | number, number] = tool ? [INK[c!.word], 0.35]
        : mine?.faces.has(path) ? [COL.sel, GLOW.face]
        : h?.index === solid && h.face === path ? [COL.highlight, GLOW.hover]
        : mine?.body ? [COL.sel, GLOW.body]
        : h?.index === solid && partOf(h) === 'body' ? [COL.highlight, GLOW.hoverBody]
        : [0x000000, 0];
      m.emissive.set(ink);
      m.emissiveIntensity = glow;
    }
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
    this.dropObjects();
    release(this.previewed);
    this.preview = { key: null, shown: false };
    this.forgetLit();
  }
}

/** Empty a group and free what it held, groups within it too.  A `Group` emptied is not a
 *  `Group` released: WebGL resources outlive the tree that named them. */
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

/** One face of an object's mesh as a three.js mesh, named by its path.  The normals are the
 *  core's: averaged across a round face and flat across a corner, so a bore's wall shades round
 *  with no smoothing pass here. */
function faceMesh(m: Mesh, f: Face): THREE.Mesh {
  const from = f.start * 9;
  const to = from + f.count * 9;
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(
    Float32Array.from(m.positions.subarray(from, to)), 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(
    Float32Array.from(m.normals.subarray(from, to)), 3));
  const face = new THREE.Mesh(g, new THREE.MeshStandardMaterial({
    color: INK.solid,
    metalness: 0.1,
    roughness: 0.65,
    flatShading: !f.smooth,
  }));
  face.name = f.path;
  face.userData.face = f.path;
  return face;
}

/** A face drawn as a ghost — see-through, and hiding nothing behind it — or solid again.  A
 *  material compiles anew when it changes whether it is transparent, so only then is it told. */
function ghost(m: THREE.MeshStandardMaterial, on: boolean): void {
  if (m.transparent === on) return;
  m.transparent = on;
  m.opacity = on ? GHOST : 1;
  m.depthWrite = !on;
  m.needsUpdate = true;
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
