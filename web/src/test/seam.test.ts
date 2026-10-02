import assert from 'node:assert/strict';
import test from 'node:test';
import { Document } from '../core/program.js';
import { boundarySeamDomain, boundarySeamSample, seamDomain, seamSample } from '../core/seam.js';
import { boundaryVertexDomain, boundaryVertexPosition, junctionVertexDomain, junctionVertexPosition } from '../core/vertex.js';
import { edgeSample } from '../core/edge.js';
import { solve } from '../core/system.js';
import { initCore } from '../core/wasm.js';

await initCore();

test('spatial seams, vertices and finite edges retain their defining geometry through the ABI', () => {
  const doc = Document.read(`unit mm
o := point
q := point
x := point
a := point
b := point
m := point
c := point
d := point
fix(x == 0, y == 0) o
fix(x == 0, y == 2) q
fix(x == 1, y == 0) x
fix(x == 2, y == 0) a
fix(x == 3, y == 0) b
fix(x == 3, y == 1) m
fix(x == 3, y == 2) c
fix(x == 2, y == 2) d
axis := line(o,q)
spin_axis := line(o,x)
profile := (bottom := line(a,b)) -> (low := line(b,m)) -> (high := line(m,c)) ->
          (top := line(c,d)) -> (inner := line(d,a)) -> close
body := solid(profile,about: axis)
first_surface := surface(body,low,from: 0deg,to: 90deg)
second_surface := surface(body,high,from: 0deg,to: 90deg)
roll := motion(about: spin_axis)
first_envelope := envelope(first_surface,under: roll,from: -20deg,to: 20deg)
second_envelope := envelope(second_surface,under: roll,from: -20deg,to: 20deg)
shared := seam(first_envelope,second_envelope)
south := point
north := point
fix(x == 0, y == -sqrt(9.25)) south
fix(x == 0, y == sqrt(9.25)) north
rim := arc(center: o,start: south,end: north)
radius(sqrt(9.25) * 1mm) rim
diameter := line(north,south)
ball := solid(face(rim,diameter),about: diameter)
boundary := surface(ball,rim)
sphere_edge := seam(first_envelope,boundary)
component Sphere(origin: point,size: Length) {
  private south := point hint(x: origin.x,y: origin.y-size)
  private north := point hint(x: origin.x,y: origin.y+size)
  south vertical origin
  north vertical origin
  private rim := arc(center: origin,start: south,end: north)
  radius(size) rim
  private diameter := line(north,south)
  private carrier := solid(face(rim,diameter),about: diameter)
  wall := surface(carrier,rim)
}
shifted := point
fix(x == 0, y == 1) shifted
offset := Sphere(shifted,size: sqrt(10.25-cos(0.1rad))*1mm)
join_cut := Sphere(shifted,size: sqrt(11-2*cos(0.1rad))*1mm)
offset_edge := seam(first_envelope,offset.wall)
join_edge := seam(second_envelope,join_cut.wall)
corner := vertex(sphere_edge,offset_edge)
junction := vertex(shared,join_edge)
end_offset := Sphere(shifted,size: sqrt(10.25-cos(0.2rad))*1mm)
end_join_cut := Sphere(shifted,size: sqrt(11-2*cos(0.2rad))*1mm)
end_edge := seam(first_envelope,end_offset.wall)
end_join_edge := seam(second_envelope,end_join_cut.wall)
finish := vertex(sphere_edge,end_edge)
finish_join := vertex(shared,end_join_edge)
bounded := edge(sphere_edge,from: corner,to: finish,along: axis)
joined := edge(shared,from: junction,to: finish_join,along: axis)
`);
  try {
    assert.ok(doc.ok, JSON.stringify(doc.diagnostics));
    assert.ok(solve(doc.sketch).success);
    assert.deepEqual(doc.seams(), ['shared', 'sphere_edge', 'offset_edge', 'join_edge', 'end_edge', 'end_join_edge']
      .map((name, index) => ({ name, index })));
    const tolerance = { position: 1e-9, normal: 1e-9, axis: 1e-10, normalVelocity: 1e-9, trim: 1e-9 };
    assert.deepEqual(seamDomain(doc.sketch, 0, tolerance).slice(0, 2), [[1, 1], [0, .25]]);
    const p = seamSample(doc.sketch, 0, 1, 0, .1, tolerance);
    [3, -Math.sin(.1), Math.cos(.1)].forEach((v, i) => assert.ok(Math.abs(v-p.position[i]) < 1e-9));
    assert.throws(() => seamSample(doc.sketch, 0, 1, .1, .1, tolerance), /OutsideDomain/);
    assert.throws(() => seamSample(doc.sketch, 0, 0, 0, .1, tolerance), /OutsideDomain/);
    assert.throws(() => seamDomain(doc.sketch, 99, tolerance), /no such seam/);
    const bt = { axis: 1e-10, normalVelocity: 1e-9, incidence: 1e-9, trim: 1e-9 };
    assert.deepEqual(boundarySeamDomain(doc.sketch, 1, bt.axis).slice(0, 2), [[0, 1], [0, .25]]);
    const q = boundarySeamSample(doc.sketch, 1, .5, 0, .1, bt);
    [3, -.5*Math.sin(.1), .5*Math.cos(.1)].forEach((v, i) => assert.ok(Math.abs(v-q.position[i]) < 1e-9));
    assert.throws(() => boundarySeamSample(doc.sketch, 1, .6, 0, .1, bt), /OutsideDomain/);
    assert.throws(() => boundarySeamSample(doc.sketch, 1, .5, .1, .1, bt), /OutsideDomain/);
    assert.throws(() => boundarySeamDomain(doc.sketch, 0, bt.axis), /boundary seam needs/);
    assert.deepEqual(doc.vertices(), ['corner', 'junction', 'finish', 'finish_join']
      .map((name, index) => ({ name, index })));
    assert.deepEqual(boundaryVertexDomain(doc.sketch, 0, bt.axis).slice(0, 2), [[0, 1], [0, .25]]);
    assert.deepEqual(junctionVertexDomain(doc.sketch, 1, tolerance).slice(0, 2), [[1, 1], [0, .25]]);
    const corner = boundaryVertexPosition(doc.sketch, 0, .5, 0, .1, bt);
    const junction = junctionVertexPosition(doc.sketch, 1, 1, 0, .1, { ...tolerance, incidence: 1e-9 });
    q.position.forEach((v, i) => assert.ok(Math.abs(v-corner[i]) < 1e-9));
    p.position.forEach((v, i) => assert.ok(Math.abs(v-junction[i]) < 1e-9));
    assert.throws(() => boundaryVertexPosition(doc.sketch, 0, .5, 0, .2, bt), /OutsideDomain/);
    assert.throws(() => junctionVertexPosition(doc.sketch, 1, 0, 0, .1,
      { ...tolerance, incidence: 1e-9 }), /OutsideDomain/);
    assert.throws(() => boundaryVertexDomain(doc.sketch, 1, bt.axis), /generating junction/);
    assert.throws(() => junctionVertexDomain(doc.sketch, 0, tolerance), /two boundary seams/);
    assert.throws(() => boundaryVertexDomain(doc.sketch, -1, bt.axis), /no such vertex/);
    assert.throws(() => junctionVertexDomain(doc.sketch, 99, tolerance), /no such vertex/);
    assert.deepEqual(doc.edges(), [{ name: 'bounded', index: 0 }, { name: 'joined', index: 1 }]);
    const et = { ...tolerance, incidence: 1e-9 };
    for (const [index, u] of [[0, .5], [1, 1]]) {
      const endpoints: [[number, number, number], [number, number, number]] = [[u, 0, .1], [u, 0, .2]];
      const mid = edgeSample(doc.sketch, index, endpoints, .5, et);
      const angle = Math.acos((Math.cos(.1)+Math.cos(.2))/2);
      [3, -u*Math.sin(angle), u*Math.cos(angle)].forEach((v, i) =>
        assert.ok(Math.abs(v-mid.position[i]) < 1e-8));
      [u, 0, angle].forEach((v, i) => assert.ok(Math.abs(v-mid.parameters[i]) < 1e-8));
      assert.throws(() => edgeSample(doc.sketch, index, endpoints, 1.01, et), /OutsideDomain/);
      assert.throws(() => edgeSample(doc.sketch, index, endpoints, .5, et, 0), /InvalidOptions/);
      assert.throws(() => edgeSample(doc.sketch, index, [[u, 0, .3], endpoints[1]], .5, et), /invalid edge endpoint/);
    }
    assert.throws(() => edgeSample(doc.sketch, -1, [[.5, 0, .1], [.5, 0, .2]], .5, et), /no such edge/);
  } finally { doc.dispose(); }
});
