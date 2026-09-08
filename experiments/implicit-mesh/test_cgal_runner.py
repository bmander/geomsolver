import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import run_cgal


class RepairRunnerTests(unittest.TestCase):
    def test_native_failure_is_preserved_without_a_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/"input.stl", Path(directory)/"output.stl"
            data = bytes(80)+struct.pack("<I", 1)
            data += bytes(12)+struct.pack("<9fH", 0, 0, 0, 1, 0, 0, 0, 1, 0, 0)
            source.write_bytes(data)
            source.with_suffix(".json").write_text(json.dumps(dict(extraction_seconds=0)))
            failure = subprocess.CompletedProcess([], 2, "", "assertion-marker")
            with patch("run_cgal.subprocess.run", return_value=failure):
                with self.assertRaisesRegex(RuntimeError, "assertion-marker"):
                    run_cgal.run(Path("unused"), source, output)
            saved = json.loads(Path(str(output)+".failure.json").read_text())
            self.assertEqual(saved["returncode"], 2)
            self.assertEqual(saved["stderr"], "assertion-marker")
            self.assertFalse(output.exists())
            self.assertEqual(source.read_bytes(), data)

    def test_empty_input_does_not_invoke_native_repair(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/"input.stl", Path(directory)/"output.stl"
            source.write_bytes(bytes(84))
            source.with_suffix(".json").write_text(json.dumps(dict(extraction_seconds=0)))
            with patch("run_cgal.subprocess.run") as native:
                self.assertTrue(run_cgal.run(Path("unused"), source, output))
                native.assert_not_called()
            self.assertEqual(output.read_bytes(), bytes(84))
            saved = json.loads(output.with_suffix(".json").read_text())
            self.assertTrue(saved["cgal_repair"]["empty_input"])
            self.assertEqual(saved["triangles"], [])


if __name__ == "__main__":
    unittest.main()
