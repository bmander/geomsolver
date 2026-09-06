/** Navigation over an already-rendered sheet. Camera changes only transform the SVG image. */
import { Camera } from './camera.js';

export class DrawingViewport {
  private readonly cameras = new Map<string, Camera>();
  private camera = new Camera();
  private key = '';
  private pan: { id: number; x: number; y: number } | null = null;

  constructor(private readonly host: HTMLElement, private readonly image: HTMLImageElement,
    private readonly zoomLabel: HTMLElement) {
    this.fit();
    host.addEventListener('wheel', (e) => {
      e.preventDefault();
      const rect = host.getBoundingClientRect();
      const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? rect.height : 1;
      this.zoom(1.0015 ** (-e.deltaY * unit),
        e.clientX - rect.left - rect.width / 2, e.clientY - rect.top - rect.height / 2);
    }, { passive: false });
    host.addEventListener('pointerdown', (e) => {
      if (this.pan || e.button > 2) return;
      e.preventDefault();
      host.focus({ preventScroll: true });
      host.setPointerCapture(e.pointerId);
      this.pan = { id: e.pointerId, x: e.clientX, y: e.clientY };
      host.classList.add('panning');
    });
    host.addEventListener('pointermove', (e) => {
      if (!this.pan || e.pointerId !== this.pan.id) return;
      this.camera.panBy(e.clientX - this.pan.x, e.clientY - this.pan.y);
      this.pan.x = e.clientX; this.pan.y = e.clientY;
      this.paint();
    });
    for (const event of ['pointerup', 'pointercancel', 'lostpointercapture'] as const) {
      host.addEventListener(event, (e) => {
        if (this.pan?.id === e.pointerId) this.stopPan();
      });
    }
    host.addEventListener('contextmenu', (e) => e.preventDefault());
    host.addEventListener('keydown', (e) => {
      if (e.metaKey || e.ctrlKey || e.altKey) return;
      const step = e.shiftKey ? 120 : 40;
      switch (e.key) {
        case '+': case '=': this.zoom(1.25); break;
        case '-': this.zoom(1 / 1.25); break;
        case '0': case 'Home': this.fit(); break;
        case 'ArrowLeft': this.camera.panBy(-step, 0); break;
        case 'ArrowRight': this.camera.panBy(step, 0); break;
        case 'ArrowUp': this.camera.panBy(0, -step); break;
        case 'ArrowDown': this.camera.panBy(0, step); break;
        default: return;
      }
      e.preventDefault(); e.stopPropagation(); this.paint();
    });
  }

  /** Each file and sheet keeps its camera through source edits and project-file switches. */
  show(key: string): void {
    if (key !== this.key) this.stopPan();
    this.key = key;
    const saved = this.cameras.get(key);
    if (saved) this.camera = saved;
    else {
      this.camera = new Camera();
      this.fit();
      this.cameras.set(key, this.camera);
    }
    this.paint();
  }

  reset(): void {
    this.stopPan(); this.cameras.clear(); this.key = '';
    this.camera = new Camera(); this.fit();
  }

  stopPan(): void {
    const pan = this.pan;
    this.pan = null;
    if (pan && this.host.hasPointerCapture(pan.id)) this.host.releasePointerCapture(pan.id);
    this.host.classList.remove('panning');
  }

  fit(): void {
    this.camera.scale = 1; this.camera.originX = 0; this.camera.originY = 0;
    this.paint();
  }

  zoom(factor: number, x = 0, y = 0): void {
    const scale = Math.max(.1, Math.min(64, this.camera.scale * factor));
    this.camera.zoomAt(x, y, scale / this.camera.scale);
    this.paint();
  }

  private paint(): void {
    const c = this.camera;
    this.image.style.transform = `translate(${c.originX}px, ${c.originY}px) scale(${c.scale})`;
    this.zoomLabel.textContent = `${Math.round(c.scale * 100)}%`;
  }
}
