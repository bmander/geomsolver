"""A coefficient fixture tests mesh binding without trusting a CAD reader."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest

from check_mesh_deviation import run


class MeshBindingTests(unittest.TestCase):
    def test_changed_omitted_and_misassociated_triangles_are_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            step, stl, source, output = [root/name for name in ("source.step", "mesh.stl", "input.json", "report.json")]
            step.write_bytes(b"coefficient fixture provenance; not a STEP reader test")
            stl.write_bytes(bytes(80)+struct.pack("<I12fH", 1, *([0]*3+[0, 0, 0, 1, 0, 0, 0, 1, 0]), 0))
            face = dict(face_index=0, surface=dict(kind="plane", origin=[0, 0, 0],
                        basis=[[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                        nodes=[[0, 0, 0, 0, 0], [1, 0, 1, 0, 0], [0, 1, 0, 1, 0]],
                        triangles=[[0, 0, 1, 2]])
            data = dict(source_file=str(step), stl_file=str(stl),
                        source_sha256=hashlib.sha256(step.read_bytes()).hexdigest(),
                        stl_sha256=hashlib.sha256(stl.read_bytes()).hexdigest(), faces=[face])

            def check():
                source.write_text(json.dumps(data))
                with contextlib.redirect_stdout(io.StringIO()):
                    return run(source, output)

            self.assertTrue(check()["passes_surface_distance_bound"])
            face["nodes"][1][0] = 2
            refused = check()
            self.assertFalse(refused["passes_surface_distance_bound"])
            self.assertIsNone(refused["maximum_bound_mm"])
            self.assertIsNone(refused["faces"][0]["bound_mm"])
            face["nodes"][1][0] = 1
            face["triangles"] = [[0, 0, 2, 1]]
            with self.assertRaisesRegex(ValueError, "does not match"):
                check()
            face["triangles"] = [[0, 0, 1, 2]]*2
            with self.assertRaisesRegex(ValueError, "duplicate triangle"):
                check()
            data["faces"] = []
            with self.assertRaisesRegex(ValueError, "omitted"):
                check()
            data["faces"] = [face]
            stl.write_bytes(stl.read_bytes()[:-1]+b"x")
            with self.assertRaisesRegex(ValueError, "changed input"):
                check()

    def test_wrong_parameter_witness_cannot_pass_on_mesh_binding_alone(self):
        # Exact surface evaluation, not the native parameter proposal, controls acceptance.
        from fractions import Fraction as F
        from mesh_deviation import surface, triangle_bound
        plane = surface(dict(kind="plane", origin=[0, 0, 0],
                             basis=[[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        nodes = [[0, 0, 0, 0, 0], [2, 0, 1, 0, 0], [0, 1, 0, 1, 0]]
        self.assertFalse(triangle_bound(plane, nodes, F(1, 50))["passes"])


if __name__ == "__main__":
    unittest.main()
