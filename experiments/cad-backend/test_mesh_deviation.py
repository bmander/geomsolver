"""Interior sag, frame rounding and discontinuities must not evade the bound."""
from fractions import Fraction as F
import unittest

from mesh_deviation import surface, triangle_bound, interpolation_bound


FRAME = dict(origin=[0, 0, 0], basis=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])


class MeshDeviationTests(unittest.TestCase):
    def test_plane_includes_encoded_vertex_displacement(self):
        plane = surface(dict(kind="plane", **FRAME))
        nodes = [[0, 0, 0, 0, F(1, 100)], [1, 0, 1, 0, 0], [0, 1, 0, 1, 0]]
        result = triangle_bound(plane, nodes, F(1, 50))
        self.assertTrue(result["passes"])
        self.assertGreaterEqual(result["bound"], F(1, 100))
        self.assertLess(result["bound"], F(11, 1000))
        self.assertFalse(triangle_bound(plane, nodes, F(1, 1000))["passes"])

    def bulge(self):
        return dict(kind="polynomial", degrees=[2, 2], knots=[[0, 0, 0, 1, 1, 1]]*2,
            weights=[[1]*3 for _ in range(3)],
            poles=[[[F(i, 2), F(j, 2), [0, 2, 0][i]] for j in range(3)] for i in range(3)])

    def test_interior_bulge_with_three_exact_corners_is_detected(self):
        curved = surface(self.bulge())
        nodes = [[0, 0, 0, 0, 0], [1, 0, 1, 0, 0], [0, 1, 0, 1, 0]]
        self.assertEqual(curved.value((F(1, 2), F(0)))[2], (F(1), F(1)))
        self.assertFalse(triangle_bound(curved, nodes, F(1, 10))["passes"])
        result = triangle_bound(curved, nodes, F(2))
        self.assertGreaterEqual(result["bound"], 1)

    def test_sphere_uses_encoded_frame_norm(self):
        sphere = surface(dict(kind="sphere", radius=5, **FRAME))
        hessian = sphere.hessian([(F(0), F(1, 10)), (F(0), F(1, 10))])
        self.assertGreaterEqual(hessian[0], 5)
        uv = [(F(0), F(0)), (F(1, 10), F(0)), (F(0), F(1, 10))]
        bound = interpolation_bound(uv, hessian)
        self.assertGreaterEqual(bound, F(1, 80))
        self.assertLess(bound, F(1, 50))

    def test_rational_and_non_c1_splines_are_refused(self):
        record = self.bulge()
        record["weights"][1][1] = F(1, 2)
        with self.assertRaisesRegex(ValueError, "polynomial"):
            surface(record)
        record = self.bulge()
        record["knots"][0] = [0, 0, 0, F(1, 2), F(1, 2), 1, 1, 1]
        record["poles"].extend(record["poles"][:2])
        record["weights"].extend([[1]*3 for _ in range(2)])
        with self.assertRaisesRegex(ValueError, "C1"):
            surface(record)

    def test_sphere_pole_and_seam_do_not_require_linear_uv_interpolation(self):
        sphere = surface(dict(kind="sphere", radius=5, **FRAME))
        nodes = [[3, -1.48, F(-435, 1000), 0, F(-4981, 1000)],
                 [0, -1.57, 0, 0, -5], [6.28, -1.57, 0, 0, -5]]
        result = triangle_bound(sphere, nodes, F(1, 50))
        self.assertTrue(result["passes"])
        # A chord passing through the sphere's centre is far from its surface
        # despite having all vertices on the sphere.
        nodes = [[0, 0, 5, 0, 0], [3, 0, -5, 0, 0], [0, 1, 0, 0, 5]]
        self.assertFalse(triangle_bound(sphere, nodes, F(1, 50))["passes"])


if __name__ == "__main__":
    unittest.main()
