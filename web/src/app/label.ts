/* **Words hung in the box**: a line of text drawn once into a canvas of its own and shown as a
 * sprite, always over the drawing and the same size in pixels however near the eye is — a
 * scenario's readings (`scenario-box.ts`) and an assembly's plane names (`box3d.ts`, #679). */
import * as THREE from 'three';

/** A label: its sprite, and its size in CSS pixels, which `scaleLabels` turns into the eye's
 *  lengths each frame. */
export interface Label {
  sprite: THREE.Sprite;
  w: number;
  h: number;
}

/** How a label is drawn: its type's height and the room round it, in CSS pixels, its ink, and
 *  the box behind it where it has one. */
export interface LabelStyle {
  font: number;
  pad: number;
  ink: string;
  back?: string;
}

/** The pixels a label's canvas holds for each CSS pixel: sharp on a dense screen. */
const DPR = 2;

/** `text` as a label standing at `at`, or null where there is no page to draw it on (node). */
export function textLabel(text: string, at: [number, number, number],
                          style: LabelStyle): Label | null {
  if (typeof document === 'undefined' || !text) return null;
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  if (!ctx) return null;
  const font = `${style.font * DPR}px ui-sans-serif, system-ui, sans-serif`;
  ctx.font = font;
  const w = Math.ceil(ctx.measureText(text).width / DPR) + 2 * style.pad;
  const h = style.font + 2 * style.pad;
  canvas.width = w * DPR;
  canvas.height = h * DPR;
  ctx.font = font;
  if (style.back) {
    ctx.fillStyle = style.back;
    ctx.fillRect(0, 0, canvas.width, canvas.height);
  }
  ctx.fillStyle = style.ink;
  ctx.textBaseline = 'middle';
  ctx.fillText(text, style.pad * DPR, (h / 2) * DPR);
  const map = new THREE.CanvasTexture(canvas);
  map.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map, depthTest: false }));
  sprite.position.set(...at);
  return { sprite, w, h };
}

/** Each label its size in pixels, `unit` the eye length of one: what keeps it the same size as
 *  the eye zooms. */
export function scaleLabels(labels: Iterable<Label>, unit: number): void {
  for (const { sprite, w, h } of labels) sprite.scale.set(w * unit, h * unit, 1);
}

/** Let a label go: its texture as well as its material, which three.js does not free with it. */
export function dropLabel({ sprite }: Label): void {
  sprite.removeFromParent();
  sprite.material.map?.dispose();
  sprite.material.dispose();
}
