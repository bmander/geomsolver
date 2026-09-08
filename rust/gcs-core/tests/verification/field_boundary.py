"""Independent rational audit of the sphere boundary-extraction certificate.

No core interval, meshing, topology, or distance code is imported. Exact rational
inequalities check support, partition, field bounds, and both distance directions.
This verifies the explicit sphere fixture, not arbitrary gear fields/source error.
"""
import copy
import json
import math
import sys
from collections import defaultdict
from fractions import Fraction as F


def scalar(x):
    assert isinstance(x, (int, float)) and not isinstance(x, bool)
    assert math.isfinite(x)
    return F(x)


def point(p):
    assert len(p) == 3
    return tuple(map(scalar, p))


def interval(b):
    assert len(b) == 2
    a, b = map(scalar, b)
    assert a <= b
    return a, b


def box(b):
    assert len(b) == 3
    return tuple(map(interval, b))


def contains(b, p):
    return all(lo <= x <= hi for (lo, hi), x in zip(b, p))


def farthest_squared(b, p):
    return sum(max(abs(lo-x), abs(hi-x))**2 for (lo, hi), x in zip(b, p))


def norm_squared(p):
    return sum(x*x for x in p)


def sphere_enclosure(p, bounds, center, radius):
    q = tuple(x-y for x, y in zip(p, center))
    d2 = norm_squared(q)
    lo, hi = bounds
    # lo <= sqrt(d2)-radius <= hi, without any floating-point square root.
    assert hi+radius >= 0 and (hi+radius)**2 >= d2
    assert lo+radius <= 0 or (lo+radius)**2 <= d2


def topology(vertices, triangles):
    edges = defaultdict(list)
    links = [defaultdict(list) for _ in vertices]
    for i, t in enumerate(triangles):
        assert len(t) == 3 and len(set(t)) == 3
        assert all(isinstance(v, int) and 0 <= v < len(vertices) for v in t)
        for k in range(3):
            a, b, c = t[k], t[(k+1) % 3], t[(k+2) % 3]
            edges[min(a, b), max(a, b)].append((i, a < b))
            links[a][b].append(c)
            links[a][c].append(b)
    adjacency = defaultdict(list)
    for uses in edges.values():
        assert len(uses) == 2 and uses[0][1] != uses[1][1]
        a, b = uses[0][0], uses[1][0]
        adjacency[a].append(b)
        adjacency[b].append(a)

    def connected(graph):
        assert graph
        seen = set()
        pending = [next(iter(graph))]
        while pending:
            v = pending.pop()
            if v not in seen:
                seen.add(v)
                pending.extend(graph[v])
        assert seen == set(graph)

    connected(adjacency)
    for link in links:
        assert all(len(around) == 2 for around in link.values())
        connected(link)
    euler = len(vertices)-len(edges)+len(triangles)
    assert euler == 2  # The independent fixture is one sphere.


def verify(data):
    assert data['schema'] == 1 and set(data['source']) == {'sphere'}
    source = data['source']['sphere']
    center, radius = point(source['center']), scalar(source['radius'])
    assert radius > 0
    domain, n = box(data['domain']), data['divisions']
    assert isinstance(n, int) and 0 < n <= 2**24 and n & (n-1) == 0
    assert all(lo <= x-radius and hi >= x+radius for (lo, hi), x in zip(domain, center))
    error = scalar(data['spatial_error'])
    assert error > 0
    vertices = list(map(point, data['vertices']))
    triangles = data['triangles']
    assert vertices and triangles
    topology(vertices, triangles)
    cells = data['cells']
    tree, planes, boxes = {}, [dict() for _ in range(3)], []
    unknown = 0
    for i, c in enumerate(cells):
        start, step = c['start'], c['step']
        assert isinstance(step, int) and 0 < step <= n and step & (step-1) == 0
        assert len(start) == 3 and all(isinstance(v, int) and 0 <= v <= n-step and v % step == 0 for v in start)
        node, span = tree, n
        while span > step:
            assert 'leaf' not in node
            span //= 2
            octant = sum((bool(v & span) << k) for k, v in enumerate(start))
            node = node.setdefault(octant, {})
        assert not node  # Refuse duplicates and an ancestor of an existing cell.
        node['leaf'] = i
        b = box(c['bounds'])
        boxes.append(b)
        for k, (lo, hi) in enumerate(b):
            assert domain[k][0] <= lo < hi <= domain[k][1]
            for index, value in [(start[k], lo), (start[k]+step, hi)]:
                assert planes[k].setdefault(index, value) == value
        p = point(c['center'])
        value = interval(c['value'])
        cv = interval(c['center_value'])
        r = scalar(c['radius'])
        assert r >= 0 and farthest_squared(b, p) <= r*r
        sphere_enclosure(p, cv, center, radius)
        assert value[0] <= cv[0]-r and value[1] >= cv[1]+r
        witness = c['mesh_vertex']
        if value[0] <= 0 <= value[1]:
            unknown += 1
            assert isinstance(witness, int) and 0 <= witness < len(vertices)
            assert farthest_squared(b, vertices[witness]) <= error*error
        else:
            assert witness is None

    def closed(node):
        if 'leaf' in node:
            assert set(node) == {'leaf'}
        else:
            assert set(node) == set(range(8)), 'missing spatial cell'
            for child in node.values():
                closed(child)
    closed(tree)
    for k, axis in enumerate(planes):
        assert axis[0] == domain[k][0] and axis[n] == domain[k][1]
        values = [axis[i] for i in sorted(axis)]
        assert all(a < b for a, b in zip(values, values[1:]))

    covered = set()
    for c in data['crossings']:
        b = boxes[c['cell']]
        assert sum((hi-lo)**2 for lo, hi in b) <= error*error
        for name, inside in [('inside', True), ('outside', False)]:
            p, value = point(c[name]['point']), interval(c[name]['value'])
            assert contains(b, p)
            sphere_enclosure(p, value, center, radius)
            assert value[1] < 0 if inside else value[0] > 0
        a, z = c['triangles']
        assert isinstance(a, int) and isinstance(z, int) and 0 <= a < z <= len(triangles)
        for i in range(a, z):
            assert i not in covered
            covered.add(i)
            assert all(contains(b, vertices[v]) for v in triangles[i])
    assert covered == set(range(len(triangles)))
    return {'vertices': len(vertices), 'triangles': len(triangles),
            'partition_cells': len(cells), 'possible_boundary_cells': unknown,
            'crossing_cells': len(data['crossings']), 'spatial_error_bound': float(error),
            'arithmetic': 'exact rational inequalities',
            'scope': 'two-sided distance to the explicit sphere boundary and closed sphere topology; embedding and quantized export not certified'}


def negative_controls(data):
    changes = {
        'missing_cell': lambda d: d['cells'].pop(),
        'false_radius': lambda d: d['cells'][0].__setitem__('radius', 0),
        'false_center_bound': lambda d: d['cells'][0].__setitem__('center_value', [-999, -998]),
        'missing_crossing': lambda d: d['crossings'].pop(),
        'false_inside': lambda d: d['crossings'][0].__setitem__('inside', d['crossings'][0]['outside']),
        'moved_vertex': lambda d: d['vertices'].__setitem__(0, [1000, 1000, 1000]),
        'false_distance': lambda d: d.__setitem__('spatial_error', 1e-15),
        'cropped_support': lambda d: d['domain'][0].__setitem__(1, 0),
    }
    for name, change in changes.items():
        broken = copy.deepcopy(data)
        change(broken)
        try:
            verify(broken)
        except (AssertionError, KeyError, IndexError):
            continue
        raise AssertionError(f'accepted corrupted evidence: {name}')
    return list(changes)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit('usage: python3 field_boundary.py CERTIFICATE.json')
    with open(sys.argv[1]) as f:
        data = json.load(f)
    result = verify(data)
    result['rejected_controls'] = negative_controls(data)
    print(json.dumps(result, indent=2))
