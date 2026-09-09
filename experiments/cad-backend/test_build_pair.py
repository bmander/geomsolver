"""Successful CAD operations alone must not pass the pair workflow."""
import tempfile
from pathlib import Path
import unittest

from build_pair import Workflow, check_assembly, check_contacts


class BuildPairTests(unittest.TestCase):
    def assembly(self, volume=0., solids=0):
        return dict(gear_phase_offset_rad=0., samples=[dict(tooth_fraction=0.,
            done=True, valid=True, volume_mm3=volume, solids=solids)])

    def test_completed_boolean_with_overlap_is_not_a_pass(self):
        with self.assertRaisesRegex(ValueError, "overlap"):
            check_assembly(self.assembly(.1, 1), [0.], 0.)

    def test_missing_phase_and_nonfinite_volume_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "coverage"):
            check_assembly(self.assembly(), [0., .5], 0.)
        with self.assertRaisesRegex(ValueError, "volume"):
            check_assembly(self.assembly(float("nan")), [0.], 0.)

    def test_negative_control_requires_detected_volume(self):
        with self.assertRaisesRegex(ValueError, "did not detect"):
            check_assembly(self.assembly(), [0.], 0., expect_overlap=True)
        check_assembly(self.assembly(.1, 1), [0.], 0., expect_overlap=True)

    def test_crashed_contact_control_is_not_a_detected_error(self):
        result = dict(samples=[], gear_phase_offset_rad=.001, passes_sampled_checks=False,
                      failures=[dict(reason="contact misses CAD boundary")])
        with self.assertRaisesRegex(ValueError, "incomplete"):
            check_contacts(result, .001, negative=True)
        result["samples"] = [{}]*250
        check_contacts(result, .001, negative=True)
        result["failures"] = [dict(reason="some unrelated error")]
        with self.assertRaisesRegex(ValueError, "did not detect"):
            check_contacts(result, .001, negative=True)

    def test_existing_evidence_directory_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            evidence = path/"report.json"
            evidence.write_text("previous evidence")
            with self.assertRaises(FileExistsError):
                Workflow(path, {})
            self.assertEqual(evidence.read_text(), "previous evidence")


if __name__ == "__main__":
    unittest.main()
