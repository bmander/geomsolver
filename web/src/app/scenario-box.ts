/* **A load scenario in the box** (#510): its supports and loads as marks, and the drawing
 * deformed under it, coloured by how far it moves.
 *
 * Everything placed here is the core's: a mark's anchors, its directions and its face's normals
 * (`ScenarioMark`), a body's places at rest and their moves (`Drawn`).  What a mark *looks like*
 * — a cone for a pin, a plate for a fixing, an arrow for a load — and how large on screen is
 * chrome, as the colour ramp is: each glyph is made in pixels about its anchor and scaled by the
 * world length of one pixel every frame, so it keeps its size as the eye zooms, as a callout
 * does.  **The deformation is a morph target**: three.js draws `rest + s·moved` on the GPU, so
 * the factor the card's slider sets — or the swing, animated — is a number written per frame,
 * and no coordinate is worked out on this side. */
import * as THREE from 'three';

import type { ScenarioAnswer } from '../core/program.js';
import type { ScenarioMark, Vec3 } from '../core/scenario.js';
import type { Drawn } from './scenario-view.js';
import type { SketchView } from './view.js';

/** **The colour ramp** a move is read by, from none to the largest: stops on 0 to 1.  The card's
 *  legend draws the same stops. */
export const RAMP: [number, string][] = [
  [0, '#30408f'], [0.25, '#1f8fcf'], [0.5, '#3dbb74'], [0.75, '#f0bf2a'], [1, '#d4351f'],
];

/** The marks' own ink. */
const INK = {
  support: 0x2f5d8a,
  load: 0xc0392b,
  pressure: 0xd35400,
  accelerate: 0x7d3fbf,
  requirement: 0x5f6368,
  kept: 0x1e6b2e,
  refuted: 0xb3001b,
  mesh: 0x1b1b1f,
};

/** Each glyph's size, in screen pixels. */
const PX = {
  arrow: 46, head: 11, headR: 4.5, shaft: 1.3,
  cube: 9, plate: 16, plateT: 3,
  cone: 13, coneR: 6.5, roller: 15,
  probe: 5,
  font: 12, pad: 3,
};

/** How long a swing takes, there and back, in milliseconds. */
const SWING_MS = 1600;

const UP = new THREE.Vector3(0, 1, 0);

/** The colour of a move `t` of the largest, on the ramp. */
export function ramp(t: number, out = new THREE.Color()): THREE.Color {
  const x = Math.min(1, Math.max(0, t));
  for (let k = 1; k < RAMP.length; k++) {
    const [b, cb] = RAMP[k];
    if (x <= b || k === RAMP.length - 1) {
      const [a, ca] = RAMP[k - 1];
      return out.set(ca).lerp(new THREE.Color(cb), b > a ? (x - a) / (b - a) : 0);
    }
  }
  return out.set(RAMP[0][1]);
}

/** What the scenario drawn answered of its requirement `index`, once it is read. */
function answerOf(v: SketchView, index: number): ScenarioAnswer | undefined {
  return v.scenarios.find((s) => s.name === v.scenario)?.answers.find((a) => a.index === index);
}

/** A sentence the core said of a scenario, without the scenario's name at its head — what the
 *  card and a label say, the scenario being the one chosen. */
export function bare(text: string): string {
  return text.replace(/^scenario `[^`]*`:? /, '');
}

/** A requirement's label: its sentence without the scenario's name, to its value. */
function said(v: SketchView, m: ScenarioMark): string {
  const a = answerOf(v, m.index);
  if (!a) return m.text;
  const text = bare(a.text);
  const cut = text.indexOf(': ', text.indexOf(' is '));
  return cut > 0 ? text.slice(0, cut) : text;
}

export class ScenarioScene {
  /** What the box adds to its scene: the marks, then the bodies, then the labels over both. */
  readonly group = new THREE.Group();
  private readonly marks = new THREE.Group();
  private readonly bodies = new THREE.Group();
  /** Each glyph, made in pixels about its anchor: scaled by the pixel's world length a frame. */
  private readonly glyphs: THREE.Object3D[] = [];
  /** Each label, its size in pixels: a sprite scaled the same way. */
  private readonly labels: { sprite: THREE.Sprite; w: number; h: number }[] = [];
  /** Each requirement's probe, by its index, for its verdict's ink. */
  private readonly probes: { index: number; mat: THREE.MeshBasicMaterial }[] = [];
  /** The deformed bodies and their meshes' edges, for the factor written a frame. */
  private readonly morphed: (THREE.Mesh | THREE.LineSegments)[] = [];
  private readonly edges: THREE.LineSegments[] = [];
  /** What the scene was built from. */
  private built: { epoch: number; said: string } | null = null;

  constructor() {
    this.group.add(this.marks, this.bodies);
  }

  /** The solids the picture draws deformed, which the box ghosts: the deformation is what is
   *  looked at, and the solid at rest shows through it. */
  ghosted(v: SketchView): Set<number> {
    const out = new Set<number>();
    if (!this.shown(v) || !v.picture) return out;
    for (const b of v.picture.bodies) if (b.solid !== null) out.add(b.solid);
    return out;
  }

  /** Whether the scenario is drawn now: one is chosen, and no gesture is moving the drawing it
   *  was read of. */
  private shown(v: SketchView): boolean {
    return !!v.scenario && !v.gesture;
  }

  /** **Brought up to the view and drawn for this frame**: rebuilt only when the marks, the
   *  picture or the answers change; per frame, every glyph's and label's scale and the factor. */
  paint(v: SketchView): void {
    const key = JSON.stringify(v.scenarios.find((s) => s.name === v.scenario)?.answers ?? []);
    if (!this.built || this.built.epoch !== v.scenarioEpoch || this.built.said !== key) {
      this.build(v);
      this.built = { epoch: v.scenarioEpoch, said: key };
    }
    this.group.visible = this.shown(v);
    if (!this.group.visible) return;
    const unit = v.unit;
    for (const g of this.glyphs) g.scale.setScalar(unit);
    for (const { sprite, w, h } of this.labels) sprite.scale.set(w * unit, h * unit, 1);
    for (const p of this.probes) {
      const held = answerOf(v, p.index)?.holds ?? null;
      p.mat.color.set(held === true ? INK.kept : held === false ? INK.refuted : INK.requirement);
    }
    let s = v.scenarioFactor;
    if (v.scenarioAnimate) s *= 0.5 - 0.5 * Math.cos(2 * Math.PI * performance.now() / SWING_MS);
    for (const m of this.morphed) if (m.morphTargetInfluences) m.morphTargetInfluences[0] = s;
    for (const e of this.edges) e.visible = v.scenarioMesh;
  }

  /** Throw the scene away: a document let go. */
  clear(): void {
    this.built = null;
    this.release();
  }

  private release(): void {
    for (const g of [this.marks, this.bodies]) {
      g.traverse((o) => {
        const m = o as THREE.Mesh;
        m.geometry?.dispose();
        const mat = m.material as THREE.Material | THREE.Material[] | undefined;
        for (const x of Array.isArray(mat) ? mat : mat ? [mat] : []) {
          (x as THREE.SpriteMaterial).map?.dispose();
          x.dispose();
        }
      });
      g.clear();
    }
    this.glyphs.length = 0;
    this.labels.length = 0;
    this.probes.length = 0;
    this.morphed.length = 0;
    this.edges.length = 0;
  }

  private build(v: SketchView): void {
    this.release();
    for (const m of v.scenarioMarks) this.mark(v, m);
    if (v.picture) this.picture(v.picture);
  }

  /** One mark: a glyph at each anchor, its faces tinted, its words at its label's place. */
  private mark(v: SketchView, m: ScenarioMark): void {
    // a reading in the plane, and a requirement of a whole body, have no place: the card's
    if (m.kind === 'plane' || (m.kind === 'requirement' && !m.probe)) return;
    const ink = m.kind === 'fixed' || m.kind === 'pinned' ? INK.support
      : m.kind === 'load' ? INK.load : m.kind === 'pressure' ? INK.pressure
      : m.kind === 'accelerate' ? INK.accelerate : INK.requirement;
    if (m.tint.length) this.tint(m.tint, ink);
    m.at.forEach((at, i) => {
      const dir = m.dir[i] as Vec3 | undefined;
      const normal = m.normal[i] as Vec3 | undefined;
      switch (m.kind) {
        case 'fixed': this.place(fixed(ink, !!normal), at, normal ?? null); break;
        case 'pinned': this.place(pinned(ink, !!dir && !normal), at,
          normal ? neg(normal) : dir ?? [0, 0, 1]); break;
        case 'load': case 'pressure': case 'accelerate':
          if (dir && (dir[0] || dir[1] || dir[2])) {
            this.place(arrow(ink, 0.45 + 0.55 * m.size, m.kind === 'accelerate'), at, dir);
          }
          break;
        case 'requirement': {
          const mat = new THREE.MeshBasicMaterial({ color: INK.requirement, depthTest: false });
          const probe = new THREE.Mesh(new THREE.OctahedronGeometry(PX.probe), mat);
          probe.renderOrder = 3;
          this.probes.push({ index: m.index, mat });
          this.place(probe, at, null);
          break;
        }
        default: break;
      }
    });
    const text = m.kind === 'requirement' ? said(v, m) : m.text;
    this.label(text, m.label, ink);
  }

  /** A glyph at `at`, its +y turned onto `dir` (none: as it was made). */
  private place(o: THREE.Object3D, at: Vec3, dir: Vec3 | null): void {
    o.position.set(...at);
    if (dir) o.quaternion.setFromUnitVectors(UP, new THREE.Vector3(...dir).normalize());
    this.glyphs.push(o);
    this.marks.add(o);
  }

  /** Triangles a mark is on, washed in its ink over the face beneath. */
  private tint(tris: number[], ink: number): void {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(tris, 3));
    const wash = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
      color: ink, transparent: true, opacity: 0.28, side: THREE.DoubleSide, depthWrite: false,
      polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
    }));
    this.marks.add(wash);
  }

  /** Words at `at`, in a box beside it, always over the drawing. */
  private label(text: string, at: Vec3, ink: number): void {
    if (typeof document === 'undefined' || !text) return;   // node: nothing is drawn
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const dpr = 2;
    const font = `${PX.font * dpr}px ui-sans-serif, system-ui, sans-serif`;
    ctx.font = font;
    const w = Math.ceil(ctx.measureText(text).width / dpr) + 2 * PX.pad;
    const h = PX.font + 2 * PX.pad;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    ctx.font = font;
    ctx.fillStyle = 'rgba(255,255,255,0.88)';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = `#${new THREE.Color(ink).getHexString()}`;
    ctx.textBaseline = 'middle';
    ctx.fillText(text, PX.pad * dpr, (h / 2) * dpr);
    const map = new THREE.CanvasTexture(canvas);
    map.colorSpace = THREE.SRGBColorSpace;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map, depthTest: false }));
    sprite.center.set(-0.12, -0.35);       // beside the place, up and to the right
    sprite.position.set(...at);
    sprite.renderOrder = 4;
    this.labels.push({ sprite, w, h });
    this.marks.add(sprite);
  }

  /** The bodies deformed, a morph target each, coloured by their moves; the mesh's edges. */
  private picture(p: Drawn): void {
    const colour = new THREE.Color();
    for (const b of p.bodies) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(b.rest, 3));
      g.morphAttributes.position = [new THREE.BufferAttribute(b.moved, 3)];
      g.morphTargetsRelative = true;
      const colours = new Float32Array(b.size.length * 3);
      b.size.forEach((t, i) => ramp(t, colour).toArray(colours, 3 * i));
      g.setAttribute('color', new THREE.BufferAttribute(colours, 3));
      g.setIndex(new THREE.BufferAttribute(b.triangles, 1));
      const mesh = new THREE.Mesh(g, new THREE.MeshStandardMaterial({
        vertexColors: true, flatShading: true, side: THREE.DoubleSide,
        metalness: 0, roughness: 0.75,
        // drawn over the ghost of the solid at rest where the two coincide, not fighting it
        polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1,
      }));
      mesh.frustumCulled = false;            // a move can carry it past its bounds at rest
      mesh.name = b.name;
      this.bodies.add(mesh);
      this.morphed.push(mesh);
      if (!b.edges.length) continue;
      const eg = new THREE.BufferGeometry();
      eg.setAttribute('position', g.getAttribute('position'));
      eg.morphAttributes.position = g.morphAttributes.position;
      eg.morphTargetsRelative = true;
      eg.setIndex(new THREE.BufferAttribute(b.edges, 1));
      const lines = new THREE.LineSegments(eg, new THREE.LineBasicMaterial({
        color: INK.mesh, transparent: true, opacity: 0.45,
      }));
      lines.frustumCulled = false;
      this.bodies.add(lines);
      this.morphed.push(lines);
      this.edges.push(lines);
    }
  }
}

function neg(v: Vec3): Vec3 {
  return [-v[0], -v[1], -v[2]];
}

/** A load's arrow, made along +y in pixels, its point at the origin: `scale` of its full
 *  length; an acceleration's hollow, its shaft dashed by its ink alone. */
function arrow(ink: number, scale: number, body: boolean): THREE.Object3D {
  const o = new THREE.Group();
  const mat =
    new THREE.MeshBasicMaterial({ color: ink, transparent: body, opacity: body ? 0.7 : 1 });
  const length = PX.arrow * scale;
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(PX.shaft, PX.shaft, Math.max(1, length - PX.head), 6), mat);
  shaft.position.y = -(PX.head + (length - PX.head) / 2);
  const head = new THREE.Mesh(new THREE.ConeGeometry(PX.headR, PX.head, 12), mat);
  head.position.y = -PX.head / 2;
  o.add(shaft, head);
  return o;
}

/** A fixing: a plate on a face (+y out of it), or a block about a point or along a line. */
function fixed(ink: number, onFace: boolean): THREE.Object3D {
  const mat = new THREE.MeshStandardMaterial({ color: ink, metalness: 0, roughness: 0.6 });
  if (!onFace) return new THREE.Mesh(new THREE.BoxGeometry(PX.cube, PX.cube, PX.cube), mat);
  const plate = new THREE.Mesh(new THREE.BoxGeometry(PX.plate, PX.plateT, PX.plate), mat);
  plate.position.y = PX.plateT / 2;
  const o = new THREE.Group();
  o.add(plate);
  return o;
}

/** A pin: a cone whose point is the place, made along +y; one held along a single direction
 *  stands on a roller. */
function pinned(ink: number, roller: boolean): THREE.Object3D {
  const mat = new THREE.MeshStandardMaterial({ color: ink, metalness: 0, roughness: 0.6 });
  const o = new THREE.Group();
  const cone = new THREE.Mesh(new THREE.ConeGeometry(PX.coneR, PX.cone, 16), mat);
  cone.position.y = -PX.cone / 2;
  o.add(cone);
  if (roller) {
    const r = new THREE.Mesh(new THREE.CylinderGeometry(PX.roller / 2, PX.roller / 2, 2, 20), mat);
    r.position.y = -PX.cone - 2;
    o.add(r);
  }
  return o;
}
