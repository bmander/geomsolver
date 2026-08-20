/* Kernel metadata, mirroring the registry in csrc/kernels.c.  The ids ARE the registration
 * order and are the contract between this module and the C core — adding a constraint type
 * means adding a kernel there and a row here, in the same position. */

export const enum K {
  Coincident = 0,
  Distance,
  Midpoint,
  Drag,
  Horizontal,
  Vertical,
  Parallel,
  Perpendicular,
  Angle,
  EqualLength,
  PointOnLine,
  PointOnCircle,
  Radius,
  EqualRadius,
  TangentLineCircle,
  TangentCircleCircle,
  TangentArcLine,
  Symmetric,
  ParallelDistance,
  PointLineDistance,
  AnnularDistance,
}

export interface KernelInfo {
  name: string;
  nRes: number;
  nPar: number;
  nConst: number;
}

export const KERNELS: KernelInfo[] = [
  { name: 'coincident', nRes: 2, nPar: 4, nConst: 0 },
  { name: 'distance', nRes: 1, nPar: 4, nConst: 1 },
  { name: 'midpoint', nRes: 2, nPar: 6, nConst: 0 },
  { name: 'drag', nRes: 2, nPar: 2, nConst: 3 },
  { name: 'horizontal', nRes: 1, nPar: 4, nConst: 0 },
  { name: 'vertical', nRes: 1, nPar: 4, nConst: 0 },
  { name: 'parallel', nRes: 1, nPar: 8, nConst: 0 },
  { name: 'perpendicular', nRes: 1, nPar: 8, nConst: 0 },
  { name: 'angle', nRes: 1, nPar: 8, nConst: 2 },
  { name: 'equal_length', nRes: 1, nPar: 8, nConst: 0 },
  { name: 'point_on_line', nRes: 1, nPar: 6, nConst: 0 },
  { name: 'point_on_circle', nRes: 1, nPar: 5, nConst: 0 },
  { name: 'radius', nRes: 1, nPar: 1, nConst: 1 },
  { name: 'equal_radius', nRes: 1, nPar: 2, nConst: 0 },
  { name: 'tangent_line_circle', nRes: 1, nPar: 7, nConst: 1 },
  { name: 'tangent_circle_circle', nRes: 1, nPar: 6, nConst: 1 },
  { name: 'tangent_arc_line', nRes: 1, nPar: 8, nConst: 0 },
  { name: 'symmetric', nRes: 2, nPar: 8, nConst: 0 },
  { name: 'parallel_distance', nRes: 1, nPar: 8, nConst: 1 },
  { name: 'point_line_distance', nRes: 1, nPar: 6, nConst: 1 },
  { name: 'annular_distance', nRes: 1, nPar: 2, nConst: 1 },
];
