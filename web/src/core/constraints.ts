/* Constraint types — generated from the core's own registry.
 *
 * The core declares every type's `spec` (its constructor arguments as (attribute, kind) pairs);
 * this module turns that declaration into classes, so adding a constraint type in Rust makes it
 * appear here with its attributes, its JSON form and its value editing, with nothing to change.
 *
 * A constraint can be built before it belongs to a sketch: until `Sketch.add` binds it, its
 * arguments live locally; afterwards every read and write goes through the core.
 */
import {
  ConstraintRecord, Entity, Kind, Param, Primitive, Sketch, registerConstraintFactory,
} from './model.js';
import { core, lastError, onInit, takeJson, takeStr, withBuf, withJson, withStr } from './wasm.js';

export type SpecKind =
  | 'point' | 'line' | 'circle' | 'arc' | 'circle_or_arc'
  | 'length' | 'angle' | 'float' | 'int' | 'str' | 'bool';

export const ENTITY_KINDS: ReadonlySet<string> =
  new Set(['point', 'line', 'circle', 'arc', 'circle_or_arc']);
export const DIMENSION_KINDS: ReadonlySet<string> = new Set(['length', 'angle']);

export type Spec = readonly (readonly [string, SpecKind])[];
export type Entity_ = Primitive;

interface TypeEntry {
  name: string;
  spec: [string, SpecKind][];
  defaults: unknown[];
  soft: boolean;
  commutative: boolean;
  kernel: number;
}

interface KernelEntry {
  name: string;
  nRes: number;
  nPar: number;
  nConst: number;
}

interface Registry {
  types: TypeEntry[];
  kernels: KernelEntry[];
}

let registry: Registry | null = null;

/** The core's constraint-type and kernel registry.  Read once, after `initCore`. */
export function REGISTRY(): Registry {
  if (!registry) registry = takeJson<Registry>(core().gcs_registry_json());
  return registry;
}

const MAX_PAR = 16;   // the widest kernel takes 8; a little headroom costs nothing

export abstract class Constraint {
  static readonly spec: Spec = [];
  static readonly defaults: readonly unknown[] = [];
  static readonly commutative: boolean = false;
  static readonly softByDefault: boolean = false;
  static readonly kernelId: number = -1;

  /* Spec-declared arguments are also exposed as properties (`c.d`, `c.side`, `c.external`),
   * defined by the generated subclass — hence the index signature. */
  [key: string]: unknown;

  args: unknown[] = [];
  soft = false;
  intrinsic = false;
  /** Document-stable identity, -1 until a sketch adopts it. */
  id = -1;
  sketch: Sketch | null = null;

  get spec(): Spec {
    return (this.constructor as typeof Constraint).spec;
  }

  get commutative(): boolean {
    return (this.constructor as typeof Constraint).commutative;
  }

  get kernelId(): number {
    return (this.constructor as typeof Constraint).kernelId;
  }

  get typeName(): string {
    return this.constructor.name;
  }

  get nResiduals(): number {
    return REGISTRY().kernels[this.kernelId].nRes;
  }

  /** Entities this constraint references directly, in spec order. */
  entities(): Primitive[] {
    return this.spec.flatMap(([, k], i) =>
      (ENTITY_KINDS.has(k) ? [this.args[i] as Primitive] : []));
  }

  /** The (attribute, kind) pairs of this constraint's dimension values. */
  dimensions(): (readonly [string, SpecKind])[] {
    return this.spec.filter(([, k]) => DIMENSION_KINDS.has(k));
  }

  /** The sketch this constraint belongs to, or the one its entities come from. */
  get owner(): Sketch {
    if (this.sketch) return this.sketch;
    for (const e of this.entities()) if (e) return e.sketch;
    throw new Error(`${this.typeName} references no sketch`);
  }

  toRecord(): { type: string; args: unknown[]; soft: boolean; intrinsic: boolean } {
    const args = this.spec.map(([, kind], i) => {
      const v = this.args[i];
      return ENTITY_KINDS.has(kind) && v instanceof Entity ? (v as Primitive).ref : v;
    });
    return { type: this.typeName, args, soft: this.soft, intrinsic: this.intrinsic };
  }

  bind(sk: Sketch): void {
    if (this.id >= 0 && this.sketch === sk) return;
    const id = withJson(this.toRecord(), (p, n) => core().gcs_constraint_add(sk.handle, p, n));
    if (id < 0) throw new Error(lastError() || 'constraint rejected');
    this.sketch = sk;
    this.id = id;
    sk.byId.set(id, this);
    // the core may have filled in a value we left to the geometry (a tangency's side)
    const rec = takeJson<ConstraintRecord>(core().gcs_constraint_json(sk.handle, id));
    if (rec) this.absorb(sk, rec);
  }

  unbind(): void {
    this.id = -1;
  }

  absorb(sk: Sketch, rec: ConstraintRecord): void {
    this.sketch = sk;
    this.id = rec.id;
    this.soft = rec.soft;
    this.intrinsic = rec.intrinsic;
    this.args = this.spec.map(([, kind], i) => fromJson(sk, rec.args[i], kind));
  }

  setValue(name: string, v: unknown): void {
    const i = this.spec.findIndex(([n]) => n === name);
    if (i < 0) return;
    const kind = this.spec[i][1];
    this.args[i] = v;
    // a string argument is not a number: `Number('start')` is NaN, and sending that replaced the
    // string in the core with NaN while the proxy went on showing 'start'
    if (this.id >= 0 && this.sketch && !ENTITY_KINDS.has(kind)) {
      withStr(name, (p, n) => (kind === 'str'
        ? withStr(String(v), (vp, vn) =>
          core().gcs_constraint_set_str(this.sketch!.handle, this.id, p, n, vp, vn))
        : core().gcs_constraint_set_num(this.sketch!.handle, this.id, p, n, Number(v))));
    }
  }

  /** The values of the params the kernel's columns refer to. */
  localValues(): Float64Array {
    return this.evaluated((sk, id) => withBuf(MAX_PAR, 8, (b) => {
      const n = core().gcs_constraint_local_values(sk.handle, id, b.ptr);
      return b.f64.slice(0, n);
    }));
  }

  /** The global Param indices the kernel's columns refer to. */
  paramIndices(): number[] {
    return this.evaluated((sk, id) => withBuf(MAX_PAR, 4, (b) => {
      const n = core().gcs_constraint_params(sk.handle, id, b.ptr);
      return [...b.i32.subarray(0, n)];
    }));
  }

  residual(v: ArrayLike<number>): Float64Array {
    return this.eval(v).r;
  }

  /** nResiduals x nParams, row-major, as the kernel computes it. */
  jacobian(v: ArrayLike<number>): { rows: number; cols: number; data: Float64Array } {
    const { r, j, nPar } = this.eval(v);
    return { rows: r.length, cols: nPar, data: j };
  }

  private eval(v: ArrayLike<number>): { r: Float64Array; j: Float64Array; nPar: number } {
    const nRes = this.nResiduals;
    return this.evaluated((sk, id) => withBuf(v.length, 8, (vb) => withBuf(nRes, 8, (rb) =>
      withBuf(nRes * MAX_PAR, 8, (jb) => {
        vb.set(v);
        const nPar = core().gcs_constraint_eval(sk.handle, id, vb.ptr, rb.ptr, jb.ptr);
        return { r: rb.f64.slice(), j: jb.f64.slice(0, nRes * nPar), nPar };
      }))));
  }

  /** Current residual norm (convenience for reporting). */
  error(): number {
    return this.evaluated((sk, id) => core().gcs_constraint_error(sk.handle, id));
  }

  describe(): string {
    return this.evaluated((sk, id) => takeStr(core().gcs_describe(sk.handle, id)));
  }

  /** Run against the core.  A constraint the user has not added yet is placed in its entities'
   *  sketch for the call and taken out again. */
  private evaluated<T>(fn: (sk: Sketch, id: number) => T): T {
    if (this.id >= 0 && this.sketch) return fn(this.sketch, this.id);
    const sk = this.owner;
    const temp = new (this.constructor as new (...a: never[]) => Constraint)();
    temp.args = this.args.slice();
    temp.soft = this.soft;
    temp.intrinsic = this.intrinsic;
    temp.bind(sk);
    try {
      this.args = temp.args.slice();   // a value the core filled in belongs to the original too
      return fn(sk, temp.id);
    } finally {
      sk.remove(temp);
    }
  }
}

function fromJson(sk: Sketch, v: unknown, kind: SpecKind): unknown {
  if (ENTITY_KINDS.has(kind) && Array.isArray(v)) {
    const [k, i] = v as [Kind, number];
    return sk.entities(k)[i];
  }
  return v;
}

/** True when two constraints say exactly the same thing: same type, the same entities in the same
 *  roles, the same values.  `commutative` types also match with their first two entities swapped.
 *
 *  A duplicate adds equations without adding rank, and a structural matching cannot see that, so
 *  it stays invisible until some unrelated edit tips the block into a spurious over-constrained
 *  report — which is why it is worth keeping out. */
export function sameConstraint(a: Constraint, b: Constraint): boolean {
  if (a.constructor !== b.constructor) return false;
  const match = (swap: boolean): boolean => {
    const order = a.spec.map((_, i) => i);
    if (swap) {
      const ents = a.spec.flatMap(([, k], i) => (ENTITY_KINDS.has(k) ? [i] : []));
      if (ents.length < 2) return false;
      [order[ents[0]], order[ents[1]]] = [order[ents[1]], order[ents[0]]];
    }
    return a.spec.every(([, kind], i) => (ENTITY_KINDS.has(kind)
      ? a.args[i] === b.args[order[i]]
      : Object.is(a.args[i], b.args[order[i]])));
  };
  return match(false) || (a.commutative && match(true));
}

/* -- generated types --------------------------------------------------------- */

export type ConstraintCtor = (new (...args: any[]) => Constraint) & {
  spec: Spec;
  defaults: readonly unknown[];
  commutative: boolean;
  softByDefault: boolean;
  kernelId: number;
};

export const CONSTRAINT_TYPES: Record<string, ConstraintCtor> = {};

function make(entry: TypeEntry): ConstraintCtor {
  const spec = entry.spec.map(([n, k]) => [n, k] as const);
  const cls = class extends Constraint {
    constructor(...args: unknown[]) {
      super();
      // an omitted value takes the core's own default (a drag weight of 1, an external tangency,
      // a tangency side read off the geometry)
      this.args = spec.map((_, i) => (args[i] === undefined ? entry.defaults[i] : args[i]));
      this.soft = entry.soft;
    }
  };
  Object.defineProperty(cls, 'name', { value: entry.name });
  Object.defineProperties(cls, {
    spec: { value: spec },
    defaults: { value: entry.defaults },
    commutative: { value: entry.commutative },
    softByDefault: { value: entry.soft },
    kernelId: { value: entry.kernel },
  });
  spec.forEach(([attr], i) => {
    Object.defineProperty(cls.prototype, attr, {
      get(this: Constraint) {
        return this.args[i];
      },
      set(this: Constraint, v: unknown) {
        this.setValue(attr, v);
      },
    });
  });
  return cls as unknown as ConstraintCtor;
}

/** Build the classes from the core's registry.  Idempotent; run by `initCore`. */
export function initTypes(): Record<string, ConstraintCtor> {
  if (Object.keys(CONSTRAINT_TYPES).length) return CONSTRAINT_TYPES;
  for (const entry of REGISTRY().types) CONSTRAINT_TYPES[entry.name] = make(entry);
  registerConstraintFactory((sk, rec) => {
    const Cls = CONSTRAINT_TYPES[rec.type];
    const c = Object.create(Cls.prototype) as Constraint;
    c.args = [];
    c.soft = false;
    c.intrinsic = false;
    c.id = -1;
    c.sketch = sk;
    c.absorb(sk, rec);
    return c;
  });
  ({
    Coincident, Distance, Midpoint, DragTarget, Horizontal, Vertical, Parallel, Perpendicular,
    Angle, ParallelDistance, EqualLength, PointOnLine, PointLineDistance, PointOnCircle, Radius,
    EqualRadius, AnnularDistance, TangentLineCircle, TangentCircleCircle, TangentArcLine,
    Symmetric,
  } = CONSTRAINT_TYPES);
  return CONSTRAINT_TYPES;
}

/* The types themselves.  They are live bindings, filled in by `initTypes` once the core is up —
 * the registry is the core's, so there is nothing to declare twice. */
export let Coincident: ConstraintCtor;
export let Distance: ConstraintCtor;
export let Midpoint: ConstraintCtor;
export let DragTarget: ConstraintCtor;
export let Horizontal: ConstraintCtor;
export let Vertical: ConstraintCtor;
export let Parallel: ConstraintCtor;
export let Perpendicular: ConstraintCtor;
export let Angle: ConstraintCtor;
export let ParallelDistance: ConstraintCtor;
export let EqualLength: ConstraintCtor;
export let PointOnLine: ConstraintCtor;
export let PointLineDistance: ConstraintCtor;
export let PointOnCircle: ConstraintCtor;
export let Radius: ConstraintCtor;
export let EqualRadius: ConstraintCtor;
export let AnnularDistance: ConstraintCtor;
export let TangentLineCircle: ConstraintCtor;
export let TangentCircleCircle: ConstraintCtor;
export let TangentArcLine: ConstraintCtor;
export let Symmetric: ConstraintCtor;

onInit(() => {
  initTypes();
});

/** Type test by name.  `instanceof` on a generated class narrows both branches to `Constraint`,
 *  which makes a chain of checks collapse to `never`; asking the constraint what it is keeps the
 *  chain readable and needs no per-type declaration. */
export function isType(c: Constraint | null | undefined, name: string): boolean {
  return !!c && c.typeName === name;
}

/** A constraint type by name — the generic path the toolbar applier and I/O use. */
export function type(name: string): ConstraintCtor {
  const t = initTypes()[name];
  if (!t) throw new Error(`unknown constraint type: ${name}`);
  return t;
}

export function build(name: string, args: unknown[]): Constraint {
  const Cls = type(name) as unknown as new (...a: unknown[]) => Constraint;
  return new Cls(...args);
}

/** DragTarget's mutable target — the one write the hot path performs. */
export function setTarget(c: Constraint, tx: number, ty: number): void {
  c.args[1] = tx;
  c.args[2] = ty;
  if (c.id >= 0 && c.sketch) core().gcs_constraint_set_target(c.sketch.handle, c.id, tx, ty);
}

export type { Param, Primitive };
