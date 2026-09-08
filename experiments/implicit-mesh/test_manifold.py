"""Run using the isolated manifold-requirements environment."""
import unittest
import json
from pathlib import Path
import tempfile

import audit
import manifold_fixtures as fixtures

try:
    import manifold3d
    from manifold_bench import encode
except ImportError:
    manifold3d = None


class FieldTests(unittest.TestCase):
    def test_source_boundary_and_side_conventions(self):
        for name in fixtures.NAMES:
            field, extent = fixtures.build(name)
            self.assertGreater(field((extent, extent, extent)), 0, name)
            if name == "zero_only":
                self.assertGreater(field((0, 0, 0)), 0)
                self.assertEqual(field((1, 0, 0)), 0)
                continue
            _, points = audit.reference(name)
            self.assertLess(max(abs(field(p)) for p in points), 1e-12, name)
            inside = (2, 0, 0) if name == "torus" else (-.4, 0, 0) if name == "disconnected" else (0, 0, 0)
            self.assertLess(field(inside), 0, name)


@unittest.skipUnless(manifold3d, "requires the isolated Manifold environment")
class BindingTests(unittest.TestCase):
    def test_unresolved_callback_propagates(self):
        def unresolved(x, y, z):
            raise ValueError("unresolved-field-marker")
        with self.assertRaisesRegex(ValueError, "unresolved-field-marker"):
            manifold3d.Manifold.level_set(unresolved, (-1, -1, -1, 1, 1, 1), .5)

    def test_stl_encoder_preserves_closed_tetrahedron(self):
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
        triangles = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)]
        report = audit.stl_embedding.check(encode(vertices, triangles))
        self.assertEqual(report["triangles"], 4)
        self.assertTrue(report["embedded"])
        self.assertAlmostEqual(report["signed_volume"], 1/6)

    def test_zero_area_cleanup_preserves_source_and_closed_surface(self):
        from postprocess_libfive import run
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
        triangles = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3), (0, 0, 1)]
        encoded = encode(vertices, triangles)
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/"source.stl", Path(directory)/"output.stl"
            source.write_bytes(encoded)
            source.with_suffix(".json").write_text(json.dumps(dict(extraction_seconds=0)))
            run(source, output, None)
            self.assertEqual(source.read_bytes(), encoded)
            report = audit.stl_embedding.check(output.read_bytes())
            self.assertEqual(report["triangles"], 4)
            self.assertEqual(report["signed_volume_exact"], "1/6")
            metadata = json.loads(output.with_suffix(".json").read_text())
            self.assertEqual(metadata["postprocessing"]["removed_zero_area_faces"], [4])


if __name__ == "__main__":
    unittest.main()
