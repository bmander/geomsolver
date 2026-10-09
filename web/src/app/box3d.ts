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

import { mesh, objects } from '../core/mesh.js';
import { overview3 } from '../core/overview.js';
import type { Item3 } from '../core/overview.js';
import { COL, chromeOf } from './paint.js';
import type { SketchView } from './view.js';

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
};

/** A pane's frame, in screen pixels: the plane being drawn on is bold.  WebGL draws every
 *  `LineBasicMaterial` one pixel wide whatever it is asked, so a frame is a fat line. */
const FRAME_PX = { plain: 1, current: 2.5 };

/** The kinds that stand in space and on no one plane, so are this renderer's to draw. */
const IN_SPACE = new Set(['axis']);

/** What the scene was built from, so a repaint that changes nothing rebuilds nothing.  The box is
 *  read-only, so between edits only the camera moves — and rebuilding a mesh on every pointer
 *  move is the one way to make a depth buffer slower than the painter it replaced.  The zoom is
 *  not in it either: the box is cut to the object rather than to the screen. */
interface Built {
  sketch: unknown;
  solid: boolean;
}

export class Box3D {
  private renderer: THREE.WebGLRenderer | null = null;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -1e4, 1e4);
  private readonly content = new THREE.Group();
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

  constructor(readonly canvas: HTMLCanvasElement | null) {
    this.scene.background = new THREE.Color(INK.bg);
    this.scene.add(this.content);
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
    // **The zoom is the camera's business and not the scene's**, which is why nothing here is a
    // function of `v.unit`: the scene and the mesh are both asked for at the *object's* own scale
    // (`0`), so a wheel tick moves the camera and rebuilds nothing.  Asked at the sheet's zoom
    // this re-evaluated the whole term on every tick — and finer without bound as you zoomed in,
    // since a screen pixel is a smaller world length the closer you get
    if (!this.built || this.built.sketch !== v.sketch || this.built.solid !== v.showSolid) {
      this.build(v);
      this.built = { sketch: v.sketch, solid: v.showSolid };
    }
    this.aim(v);
    this.chrome(v);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (gl.getPixelRatio() !== dpr) gl.setPixelRatio(dpr);
    gl.setSize(v.width, v.height, false);
    gl.render(this.scene, this.camera);
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
      this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true });
      return this.renderer;
    } catch {
      // no WebGL: the box simply does not draw, and the status line has already said what the
      // box is.  Better than a dialog about a thing the drawing does not depend on
      return null;
    }
  }

  /** **The camera three.js draws with is the camera the picking already assumes.**
   *
   *  `overview::eye` gives the same right/up basis the core flattens with, and `camera.ts` maps
   *  that plane to the screen.  Setting three's orthographic camera from the same two vectors and
   *  the same scale is what keeps a click landing on the thing under the cursor. */
  private aim(v: SketchView): void {
    const { az, el } = v.orbit;
    const ce = Math.cos(el);
    const eye = new THREE.Vector3(Math.cos(az) * ce, Math.sin(az) * ce, Math.sin(el));
    const up = new THREE.Vector3(-Math.cos(az) * Math.sin(el), -Math.sin(az) * Math.sin(el), ce);
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
    const right = new THREE.Vector3(-Math.sin(az), Math.cos(az), 0);
    c.position.addScaledVector(right, -centre.x);
    c.position.addScaledVector(up, -centre.y);
    c.updateMatrixWorld();
  }

  /** The scene, from what the core says is in the box. */
  private build(v: SketchView): void {
    this.dispose();
    const items = overview3(v.sketch, 0);
    // the panes, translucent and double-sided: geometry on the far side of one must read through
    for (const it of items.filter((i) => i.part === 'face')) {
      const g = fanned(it.pts);
      if (!g) continue;
      this.content.add(new THREE.Mesh(g, new THREE.MeshBasicMaterial({
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
    this.lines(items.filter((i) => i.part === 'axis'), INK.axis, 0.55);
    // of what is drawn, only what stands in space: a sketch on a plane is stroked on the canvas
    // above, through its plane's camera, with the selection and the state colours it carries
    for (const it of items.filter((i) => i.part === 'drawn' && IN_SPACE.has(i.kind ?? ''))) {
      this.frames.push({ it, line: this.lines([it], INK.drawn, 1)! });
    }
    // the object's creases, which stand whether or not its surfaces are drawn: with the
    // surfaces they are the corners of a shaded part, and without them they are the wireframe
    this.lines(items.filter((i) => i.part === 'solid'), INK.edge, 1);
    if (!v.showSolid) return;
    // **the object itself, as a mesh** — every face of it a named child, so a later selection has
    // something to name.  The normals are the core's: averaged across a round face and flat
    // across a corner, so a bore's wall shades round with no smoothing pass here
    for (const o of objects(v.sketch)) {
      // **cut to the object and not to the zoom** — `0` asks the core for `solid::mesh_unit`,
      // a sagitta a fixed fraction of the solid's own diagonal.  Handed `v.unit` this re-cut the
      // whole boundary on every wheel tick, and finer without bound as you zoomed in: one tick
      // on the V-twin cylinder locked the tab for the better part of a minute
      const m = mesh(v.sketch, o.index, 0);
      for (const f of m.faces) {
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
        this.content.add(face);
        this.faces.push({ solid: o.index, path: f.path, mesh: face });
      }
    }
  }

  /** **What the app says about the scene, written per frame.**  The selection and the plane being
   *  drawn on change without the drawing changing, so they are a material write and never a
   *  rebuild: the ink rule itself is `paint.ts`'s, so a thing picked is lit the same colour here
   *  as on the canvas above. */
  private chrome(v: SketchView): void {
    const sel = new Set(v.selected);
    const hl = new Set(v.highlight);
    for (const { it, line } of this.frames) {
      const m = line.material as THREE.LineBasicMaterial;
      const ent = v.entityOf(it);
      const chrome = ent ? chromeOf(sel, hl, ent) : null;
      m.color.set(chrome ? chrome[0] : INK.drawn);
    }
    for (const { it, line } of this.panes) {
      const m = line.material;
      const ent = v.entityOf(it);
      const chrome = ent ? chromeOf(sel, hl, ent) : null;
      m.color.set(chrome ? chrome[0] : INK.pane);
      // the plane being drawn on: its frame bolds, so where the next thing goes is in sight
      const current = v.planeOf(it) === v.plane;
      m.opacity = current ? 0.8 : 0.35;
      m.linewidth = current ? FRAME_PX.current : FRAME_PX.plain;
      m.resolution.set(v.width, v.height);
    }
    // a solid picked: the face the click landed on in the selection's ink, the rest of it washed
    const picked = v.selectedSolids;
    for (const { solid, path, mesh: face } of this.faces) {
      const m = face.material as THREE.MeshStandardMaterial;
      const pick = picked.find((s) => s.index === solid);
      m.emissive.set(pick ? COL.sel : 0x000000);
      m.emissiveIntensity = !pick ? 0 : pick.face === path ? 0.55 : 0.18;
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
    this.content.add(line);
    return line;
  }

  private lines(items: Item3[], color: number, opacity: number): THREE.LineSegments | null {
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
    this.content.add(line);
    return line;
  }

  /** Free every buffer and material the last scene made.  A `Group` emptied is not a `Group`
   *  released: WebGL resources outlive the tree that named them. */
  private dispose(): void {
    this.frames.length = 0;
    this.panes.length = 0;
    this.faces.length = 0;
    for (const o of [...this.content.children]) {
      this.content.remove(o);
      const any = o as THREE.Mesh;
      any.geometry?.dispose?.();
      const m = any.material;
      if (Array.isArray(m)) m.forEach((x) => x.dispose());
      else m?.dispose?.();
    }
  }
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
