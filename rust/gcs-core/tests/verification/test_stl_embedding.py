"""Geometric negative controls for the independent encoded-mesh audit."""
import itertools
import random
import struct
import unittest
from fractions import Fraction

import stl_embedding as audit
import stl_topology


def encode(points, triangles):
    out = bytearray(80)+struct.pack('<I', len(triangles))
    for t in triangles:
        out += bytes(12)+struct.pack('<9f', *(x for i in t for x in points[i]))+bytes(2)
    return out


TETRA = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)]
POINTS = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]


class ExactEmbeddingTests(unittest.TestCase):
    def test_contacts_under_permutations_and_affine_changes(self):
        a = ((0, 0, 0), (4, 0, 0), (0, 4, 0))
        cases = [
            ('parallel gap', ((0, 0, 1), (4, 0, 1), (0, 4, 1)), False),
            ('transverse crossing', ((1, 1, -1), (1, 1, 1), (3, 1, 1)), True),
            ('interior point touch', ((1, 1, 0), (1, 1, 1), (2, 1, 1)), True),
            ('partial edge overlap', ((1, 0, 0), (3, 0, 0), (2, -2, 0)), True),
            ('vertex on other edge', ((2, 0, 0), (2, -2, 1), (2, -2, -1)), True),
            ('coplanar shared edge', ((0, 0, 0), (4, 0, 0), (0, -4, 0)), False),
            ('overlap along shared edge', ((0, 0, 0), (4, 0, 0), (4, 4, 0)), True),
            ('distinct planes shared edge', ((0, 0, 0), (4, 0, 0), (0, 2, 2)), False),
            ('coplanar shared vertex', ((0, 0, 0), (-1, 0, 0), (0, -1, 0)), False),
            ('coplanar beyond shared vertex', ((0, 0, 0), (2, 0, 0), (0, 2, 0)), True),
            ('distinct planes shared vertex', ((0, 0, 0), (0, 0, 1), (-1, 0, 1)), False),
            ('crossing beyond shared vertex', ((0, 0, 0), (2, 2, 1), (2, 2, -1)), True),
            ('coplanar disjoint', ((3, 3, 0), (5, 3, 0), (3, 5, 0)), False),
            ('coplanar point touch', ((2, 2, 0), (3, 2, 0), (2, 3, 0)), True),
            ('disjoint plane sections', ((3, 3, -1), (3, 3, 1), (4, 3, 1)), False),
            ('duplicate face', a, True),
        ]
        transforms = [lambda p: p, lambda p: (p[2], -p[0], p[1]),
                      lambda p: (p[0]+2*p[1]+3*p[2], p[1]+2*p[2], p[2]),
                      lambda p: tuple((1 << 90)*x+(1 << 150) for x in p)]
        for name, b, expected in cases:
            for transform in transforms:
                aa, bb = tuple(map(transform, a)), tuple(map(transform, b))
                for x, y in itertools.product(itertools.permutations(aa), itertools.permutations(bb)):
                    self.assertEqual(audit.improper_intersection(x, y), expected, name)
                    self.assertEqual(audit.improper_intersection(y, x), expected, name)

    def test_closed_tetrahedra_at_extreme_encoded_scales(self):
        for power in [-149, 0, 120]:
            scale = 2.**power
            data = encode([tuple(x*scale for x in p) for p in POINTS], TETRA)
            result = audit.check(data)
            self.assertTrue(result['embedded'])
            self.assertEqual(result['genus'], 0)
            self.assertEqual(Fraction(result['signed_volume_exact']), Fraction(scale)**3/6)
            self.assertEqual(result['all_triangle_pairs'], 6)
            self.assertEqual(result['exact_pair_tests'], 6)

    def test_closed_topology_does_not_accept_coplanar_overlap(self):
        data = encode(POINTS[:3]+[(1, 1, 0)], TETRA)
        self.assertEqual(stl_topology.check(data)['genus'], 0)
        with self.assertRaisesRegex(ValueError, 'intersect beyond'):
            audit.check(data)

    def test_closed_topology_does_not_accept_transverse_overlap(self):
        # An octahedral connectivity with one pole pulled through an opposite
        # face. Every triangle and vertex fan remains combinatorially valid.
        points = [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0),
                  (1.5, 0, -0.5), (0, 0, -1)]
        triangles = [(i, (i+1) % 4, 4) for i in range(4)]
        triangles += [((i+1) % 4, i, 5) for i in range(4)]
        data = encode(points, triangles)
        self.assertEqual(stl_topology.check(data)['genus'], 0)
        with self.assertRaisesRegex(ValueError, 'intersect beyond'):
            audit.check(data)

    def test_inward_orientation_is_refused(self):
        with self.assertRaisesRegex(ValueError, 'positive exact signed volume'):
            audit.check(encode(POINTS, [tuple(reversed(t)) for t in TETRA]))

    def test_float32_collapse_is_refused(self):
        points = [tuple(1+x*2.**-30 for x in p) for p in POINTS]
        with self.assertRaisesRegex(ValueError, 'exactly degenerate'):
            audit.check(encode(points, TETRA))

    def test_float32_rounding_can_introduce_an_intersection_without_collapse(self):
        # Reduced from an unaccepted thin-tetrahedron front. The two faces
        # have no improper intersection in binary64, but do after STL encoding.
        points = [
            (-0.008599389564634016, -0.014894579640023666, 0.9267073623577332),
            (0.015019684450535295, 6.938893903907228e-18, 0.999343851648823),
            (-0.00731342585889376, -0.012667225164992052, 1.0124382760737483),
            (-0.00657828969376734, 0.011393931976511729, 1.0614473537488445),
            (-0.006131576236209524, -0.010620201571596793, 1.0912282509193658),
        ]
        faces = [(0, 1, 2), (3, 0, 4)]
        ratios = {x: x.as_integer_ratio() for p in points for x in p}
        scale = max(d for _, d in ratios.values())
        integer = {x: n*(scale//d) for x, (n, d) in ratios.items()}
        source = [tuple(tuple(integer[x] for x in points[i]) for i in t) for t in faces]
        self.assertFalse(audit.improper_intersection(*source))
        encoded, _ = audit.read_triangles(encode(points, faces))
        self.assertTrue(audit.improper_intersection(*encoded))

    def test_malformed_and_nonfinite_inputs_are_refused(self):
        data = encode(POINTS, TETRA)
        for bad in [data[:80], data[:-1], data+b'\0',
                    encode([(float('inf'), 0, 0)]+POINTS[1:], TETRA),
                    encode([(float('nan'), 0, 0)]+POINTS[1:], TETRA)]:
            with self.assertRaises(ValueError):
                audit.check(bad)

    def test_broad_phase_retains_all_closed_box_contacts(self):
        rng = random.Random(4824)
        triangles = [tuple(tuple(rng.randrange(-8, 9) for _ in range(3))
                           for _ in range(3)) for _ in range(50)]
        expected = set()
        for i, j in itertools.combinations(range(len(triangles)), 2):
            if all(max(min(p[k] for p in triangles[i]), min(p[k] for p in triangles[j]))
                   <= min(max(p[k] for p in triangles[i]), max(p[k] for p in triangles[j]))
                   for k in range(3)):
                expected.add((i, j))
        actual = [tuple(sorted(pair)) for pair in audit.candidate_pairs(triangles)]
        self.assertEqual(len(actual), len(set(actual)))
        self.assertEqual(set(actual), expected)


if __name__ == '__main__':
    unittest.main()
