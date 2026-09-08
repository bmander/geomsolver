"""Producer connectivity must survive interchange, even at coincident positions."""
import json
from pathlib import Path
import tempfile
import unittest

import run_indexed_cleanup


class IndexedCleanupTests(unittest.TestCase):
    def test_interchange_preserves_distinct_indices_and_binary64_values(self):
        source = dict(vertices=[[0., 0., 0.], [0., 0., 0.], [1+2**-40, 0., 0.]],
                      triangles=[[0, 1, 2]])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"input.off"
            run_indexed_cleanup.write_indexed(source, path)
            lines = path.read_text().splitlines()
            self.assertEqual(lines[:2], ["OFF", "3 1 0"])
            self.assertEqual([list(map(float, line.split())) for line in lines[2:5]],
                             source["vertices"])
            self.assertEqual(lines[5], "3 0 1 2")

    def test_direct_runner_cannot_overwrite_source_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"input.json"
            raw = json.dumps(dict(vertices=[], triangles=[]))
            source.write_text(raw)
            with self.assertRaisesRegex(ValueError, "preserve the source"):
                run_indexed_cleanup.run(Path("unused"), source, source.with_suffix(".stl"))
            self.assertEqual(source.read_text(), raw)


if __name__ == "__main__":
    unittest.main()
