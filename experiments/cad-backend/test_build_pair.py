"""Successful CAD operations alone must not pass the pair workflow."""
import tempfile
from pathlib import Path
import unittest

from build_pair import Workflow, check_assembly, check_contacts, check_engagement


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

    def test_engagement_report_cannot_hide_phase_gaps_or_wrong_parameters(self):
        def level(n, faces, phases, offset=0.):
            value = -.01 if offset else 0.
            return dict(profile_subdivisions=n, face_intervals=faces, phase_intervals=phases,
                gear_phase_offset_rad=offset, minimum_signed_polar_clearance_mm=value,
                boundary_queries=12*n*72*(faces+1)*(phases+1),
                phases=[dict(tooth_fraction=i/max(phases, 1), minimum_signed_polar_clearance_mm=value)
                        for i in range(phases+1)])
        result = dict(teeth=[24, 48], module_mm=2., passes_sampled_checks=True,
            levels=[level(8, 4, 16), level(16, 8, 32), level(32, 8, 64)],
            wrong_phase=level(16, 4, 0, .001))
        check_engagement(result, [24, 48], 2.)
        with self.assertRaisesRegex(ValueError, "mismatch"):
            check_engagement(result, [32, 32], 1.)
        result["levels"][-1]["phases"].pop(32)
        with self.assertRaisesRegex(ValueError, "coverage"):
            check_engagement(result, [24, 48], 2.)


if __name__ == "__main__":
    unittest.main()
