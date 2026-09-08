"""Independent exact geometric controls for whole-patch support bounds."""
import copy
from fractions import Fraction as F
from math import comb, sqrt
import unittest

from bernstein import insert, patches, refine, support_bound


def surface(grid, degrees):
    return dict(degrees=degrees, poles=[[[v/p[3] for v in p[:3]] for p in row] for row in grid],
                weights=[[p[3] for p in row] for row in grid],
                knots=[[F(0)]*(d+1)+[F(1)]*(d+1) for d in degrees])


def sphere():
    # Rational quarter circle: (1-t², 2t)/(1+t²), in degree-two Bernstein form.
    circle = [(F(1), F(0), F(1)), (F(1), F(1), F(1)), (F(0), F(2), F(2))]
    return [[(x*u, y*u, w*v, w*z) for u, v, z in circle] for x, y, w in circle]


class Bounds(unittest.TestCase):
    def test_exact_rational_sphere_and_cone(self):
        ball = next(patches(surface(sphere(), [2, 2])))["poles"]
        self.assertEqual(support_bound(ball, dict(kind="sphere", radius_mm=1)), 0)
        circle = [(F(1), F(0), F(1)), (F(1), F(1), F(1)), (F(0), F(2), F(2))]
        cone = [[(r*x, r*y, r*w, w) for r in (1, 2)] for x, y, w in circle]
        target = dict(kind="cone", meridian=[[1, 0, 1], [2, 0, 2]])
        self.assertEqual(support_bound(cone, target), 0)
        other_nappe = [[(x, y, -z, w) for x, y, z, w in row] for row in cone]
        with self.assertRaisesRegex(ValueError, "positive cone branch"):
            support_bound(other_nappe, target)

    def test_interior_bulge_cannot_hide_behind_exact_corner_samples(self):
        grid = copy.deepcopy(sphere())
        x, y, z, w = grid[1][1]
        grid[1][1] = (x+F(1, 10), y, z, w)
        middle = [sum(grid[i][j][k]*F(comb(2, i)*comb(2, j), 16)
                      for i in range(3) for j in range(3)) for k in range(4)]
        miss = abs(sqrt(sum(float(v/middle[3])**2 for v in middle[:3]))-1)
        self.assertGreater(miss, .001)
        bound = support_bound(grid, dict(kind="sphere", radius_mm=1))
        self.assertGreaterEqual(float(bound), miss)

    def test_exact_knot_insertion_preserves_known_circle_parameter(self):
        curve = [(F(1), F(0), F(1)), (F(1), F(1), F(1)), (F(0), F(2), F(2))]
        points, knots = insert(curve, [F(0)]*3+[F(1)]*3, 2, F(1, 2))
        points, knots = refine(points, knots, 2)
        self.assertEqual(knots, [F(0)]*3+[F(1, 2)]*2+[F(1)]*3)
        for span in range(2):
            for j in range(9):
                s = F(j, 8)
                t = (span+s)/2
                p = [sum(points[2*span+i][k]*comb(2, i)*s**i*(1-s)**(2-i)
                         for i in range(3)) for k in range(3)]
                self.assertEqual(p[0]/p[2], (1-t*t)/(1+t*t))
                self.assertEqual(p[1]/p[2], 2*t/(1+t*t))

    def test_refuse_invalid_basis_and_nonpositive_weights(self):
        data = surface(sphere(), [2, 2])
        data["weights"][1][1] = 0
        with self.assertRaises(ValueError):
            list(patches(data))
        data = surface(sphere(), [2, 2])
        data["knots"][0][0] = F(-1)
        with self.assertRaisesRegex(ValueError, "clamped"):
            list(patches(data))
        data = surface(sphere(), [2, 2])
        data["weights"][0].pop()
        with self.assertRaisesRegex(ValueError, "dimensions"):
            list(patches(data))

    def test_tensor_refinement_covers_nonuniform_parameter_rectangle(self):
        grid = sphere()
        knots = [F(0)]*3+[F(1)]*3
        columns = [insert([row[j] for row in grid], knots, 2, F(1, 3))[0] for j in range(3)]
        grid = [insert(list(row), knots, 2, F(2, 5))[0] for row in zip(*columns)]
        data = surface(grid, [2, 2])
        data["knots"] = [knots[:3]+[F(1, 3)]+knots[3:], knots[:3]+[F(2, 5)]+knots[3:]]
        pieces = list(patches(data))
        self.assertEqual(len(pieces), 4)
        self.assertEqual(sum((d[0][1]-d[0][0])*(d[1][1]-d[1][0])
                             for d in (p["domain"] for p in pieces)), 1)
        for patch in pieces:
            u, v = [sum(axis)/2 for axis in patch["domain"]]
            coefficients = patch["poles"]
            middle = [sum(coefficients[i][j][k]*F(comb(2, i)*comb(2, j), 16)
                          for i in range(3) for j in range(3)) for k in range(4)]
            self.assertEqual([x/middle[3] for x in middle[:3]],
                [(1-u*u)*(1-v*v)/((1+u*u)*(1+v*v)), 2*u*(1-v*v)/((1+u*u)*(1+v*v)), 2*v/(1+v*v)])
            self.assertEqual(support_bound(coefficients, dict(kind="sphere", radius_mm=1)), 0)


if __name__ == "__main__":
    unittest.main()
