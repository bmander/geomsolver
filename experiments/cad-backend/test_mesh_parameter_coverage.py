"""Coverage needs oriented edge cancellation, not just area or corner agreement."""
from copy import deepcopy
import unittest

from mesh_parameter_coverage import cover


def mesh():
    return dict(nodes=[[0, 0], [1, 0], [1, 1], [0, 1]], triangles=[[0, 0, 1, 2], [1, 0, 2, 3]])


class ParameterCoverageTests(unittest.TestCase):
    def test_square_and_reversed_orientation(self):
        face = mesh()
        self.assertEqual(cover(face)["parameter_coverage"], "complete_once")
        face["triangles"] = [[i, a, c, b] for i, a, b, c in face["triangles"]]
        self.assertEqual(cover(face)["parameter_area_exact"], "1")

    def test_equal_area_with_overlap_and_hole_is_refused(self):
        face = mesh()
        face["triangles"][1] = [1, 0, 1, 2]
        with self.assertRaisesRegex(ValueError, "overlapping"):
            cover(face)

    def test_missing_reversed_degenerate_and_outside_triangles_are_refused(self):
        cases = []
        f = mesh(); f["triangles"].pop(); cases.append(f)
        f = mesh(); f["triangles"][1] = [1, 0, 3, 2]; cases.append(f)
        f = mesh(); f["triangles"][1] = [1, 0, 2, 2]; cases.append(f)
        f = mesh(); f["nodes"][1][0] = 1+1e-12; cases.append(f)
        for face in cases:
            with self.subTest(face=face), self.assertRaises(ValueError):
                cover(face)

    def test_coordinate_identity_handles_duplicate_node_records(self):
        face = mesh()
        face["nodes"].extend(deepcopy(face["nodes"][:3]))
        face["triangles"][1] = [1, 4, 6, 3]
        self.assertEqual(cover(face)["parameter_coverage"], "complete_once")
