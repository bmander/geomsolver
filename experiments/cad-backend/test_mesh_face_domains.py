"""Trim transfer must not accept a support, crop or partial inventory as a finite face."""
from copy import deepcopy
import hashlib
import json
import unittest

from check_mesh_face_domains import verify, reference_identity
from test_face_domains import square


def fixture():
    surface = dict(kind="polynomial", degrees=[1, 1], knots=[[0, 0, 1, 1]]*2,
                   weights=[[1, 1], [1, 1]], poles=[[[0, 0, 0], [0, 1, 0]], [[1, 0, 0], [1, 1, 0]]])
    faces = [dict(face_index=0, surface=surface,
                  nodes=[[0, 0, 0, 0, 0], [1, 0, 1, 0, 0], [0, 1, 0, 1, 0]], triangles=[[0, 0, 1, 2]]),
             dict(face_index=1, surface=dict(kind="sphere", radius=1))]
    trims = []
    for f in faces:
        s = f["surface"]
        trims.append(dict(face_index=f["face_index"], kind=s["kind"],
            surface_sha256=hashlib.sha256(json.dumps(s, sort_keys=True, separators=(",", ":")).encode()).hexdigest()))
    trims[0]["wires"] = square()
    return dict(faces=faces), dict(faces=trims)


class MeshFaceDomains(unittest.TestCase):
    def test_membership_does_not_claim_reverse_or_analytical_coverage(self):
        parameters, trims = fixture()
        # The single UV triangle covers only half the square. Forward membership
        # is valid; this checker deliberately does not establish reverse coverage.
        result = verify(parameters, trims)
        self.assertEqual(result["rectangular_face_triangles"], 1)
        self.assertEqual(result["unverified_face_indices"], [1])

    def test_outside_vertex_hole_and_incomplete_wire_are_refused(self):
        parameters, trims = fixture()
        bad = deepcopy(parameters)
        bad["faces"][0]["nodes"][0][0] = -1e-16
        with self.assertRaisesRegex(ValueError, "leaves"):
            verify(bad, trims)
        bad = deepcopy(trims)
        bad["faces"][0]["wires"].append(square()[0])
        with self.assertRaises(ValueError):
            verify(parameters, bad)
        bad = deepcopy(trims)
        bad["faces"][0]["wires"][0].pop()
        with self.assertRaises(ValueError):
            verify(parameters, bad)

    def test_changed_support_and_missing_face_are_refused(self):
        parameters, trims = fixture()
        bad = deepcopy(parameters)
        bad["faces"][0]["surface"]["poles"][0][0][2] = .1
        with self.assertRaisesRegex(ValueError, "disagree"):
            verify(bad, trims)
        trims["faces"].pop()
        with self.assertRaisesRegex(ValueError, "inventory"):
            verify(parameters, trims)

    def test_reference_transfer_requires_identical_coefficients_and_complete_inventory(self):
        parameters, _ = fixture()
        s = {k:v for k,v in parameters["faces"][0]["surface"].items() if k != "kind"}
        reference = dict(surfaces=[dict(member=0, face_index=0, surface=deepcopy(s), wires=square())])
        self.assertEqual(reference_identity(parameters, reference, 0)["identical_spline_faces"], 1)
        reference["surfaces"][0]["surface"]["poles"][0][0][2] = 1e-15
        with self.assertRaisesRegex(ValueError, "coefficients"):
            reference_identity(parameters, reference, 0)
        reference["surfaces"] = []
        with self.assertRaisesRegex(ValueError, "inventory"):
            reference_identity(parameters, reference, 0)


if __name__ == "__main__":
    unittest.main()
