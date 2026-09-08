"""Reject evidence detached from the CAD probes before the costly rational audit."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import check_material


class MaterialBindingTests(unittest.TestCase):
    def fixture(self, root):
        probes, evidence = root/"probes.txt", root/"evidence.json"
        probes.write_text("0 face0 0 0 0 0\n1 face0 0 0 0 0\n")
        probes.with_suffix(".json").write_text(json.dumps(dict(
            probe_sha256=hashlib.sha256(probes.read_bytes()).hexdigest(), provenance=[])))
        data = dict(cad_failures=[], cases=[dict(definition=dict(member=name, teeth=teeth),
            points=[dict(label="face0", expected_inside=False, position_mm=[0,0,0])])
            for name, teeth in [("pinion",24),("gear",48)]])
        evidence.write_text(json.dumps(data))
        return probes, evidence, data

    def test_matching_records_reach_independent_audit(self):
        with tempfile.TemporaryDirectory() as directory:
            probes, evidence, data = self.fixture(Path(directory))
            with patch.object(check_material.member_material, "check", return_value={}) as audit:
                report = check_material.check(probes, evidence)
                audit.assert_called_once_with(data, progress=True)
                self.assertEqual(report["cad_probe_count"], 2)

    def test_changed_expectation_omission_frame_and_native_failure_are_refused(self):
        for change in ("expectation", "omission", "frame", "native_failure", "hash"):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                probes, evidence, data = self.fixture(Path(directory))
                row = data["cases"][0]["points"][0]
                if change == "expectation": row["expected_inside"] = True
                elif change == "omission": data["cases"][0]["points"] = []
                elif change == "frame": row["position_mm"][0] = .1
                elif change == "native_failure": data["cad_failures"] = ["unresolved"]
                else: probes.write_text(probes.read_text()+"0 extra 0 0 0 0\n")
                evidence.write_text(json.dumps(data))
                with patch.object(check_material.member_material, "check") as audit:
                    with self.assertRaises(ValueError): check_material.check(probes, evidence)
                    audit.assert_not_called()


if __name__ == "__main__":
    unittest.main()
