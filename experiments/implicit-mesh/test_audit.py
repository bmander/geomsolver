"""Checks for the experiment's independent geometric sampling helpers."""
import math
import unittest

import audit
import inspect_components


class AuditGeometryTests(unittest.TestCase):
    def test_component_diagnostic_distinguishes_rounding_from_native_contact(self):
        vertices = [(1, 0, 0), (1, 1, 0), (1, 0, 1),
                    (1+2**-30, 0, 0), (2, 0, 0), (2, 1, 0)]
        source = dict(vertices=vertices, triangles=[(0, 1, 2), (3, 4, 5)])
        report = inspect_components.inspect(source)
        self.assertEqual(len(report["indexed_components"]), 2)
        self.assertEqual(report["coincident_vertices"]["native64"], [])
        self.assertEqual(report["coincident_vertices"]["stl32"][0]["components"], [0, 1])
        vertices[3] = vertices[0]
        report = inspect_components.inspect(source)
        self.assertEqual(report["coincident_vertices"]["native64"][0]["components"], [0, 1])

    def test_finite_triangle_distance(self):
        triangle = ((0, 0, 0), (1, 0, 0), (0, 1, 0))
        # Interior projection, outside the face, nearest vertex, and on-face.
        for point, expected in [((.2, .2, 1), 1), ((1, 1, 0), math.sqrt(.5)),
                                ((2, 0, 0), 1), ((.25, .25, 0), 0)]:
            self.assertAlmostEqual(audit.triangle_distance(point, triangle), expected)

    def test_reference_landmarks_lie_on_finite_boundaries(self):
        for name in ["sphere", "cube", "rotated_cube", "torus", "spiky_tetrahedron",
                     "rotated_tetrahedron", "thin_plate", "disconnected"]:
            distance, points = audit.reference(name)
            self.assertLess(max(map(distance, points)), 1e-12, name)

    def test_tetrahedron_reference_does_not_extend_planes(self):
        distance, points = audit.reference("spiky_tetrahedron")
        # This point is in the base plane, but far outside its finite triangle.
        self.assertGreater(distance((1, 0, -.5)), .9)
        self.assertAlmostEqual(distance((0, 0, 1.6)), .1)

    def test_components_include_vertex_contacts(self):
        triangles = [((0, 0, 0), (1, 0, 0), (0, 1, 0)),
                     ((0, 0, 0), (0, 0, 1), (1, 0, 0)),
                     ((9, 9, 9), (10, 9, 9), (9, 10, 9))]
        self.assertEqual(audit.components(triangles), [[0, 1], [2]])


if __name__ == "__main__":
    unittest.main()
