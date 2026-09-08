"""Support-certificate input binding and honest partial-coverage reporting."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from check_supports import run
from test_bernstein import sphere, surface


class Binding(unittest.TestCase):
    def test_reference_mutations_and_missing_coverage_are_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            members = [dict(member=m, teeth=24*(m+1), blank_meridian=[]) for m in (0, 1)]
            source = dict(members=members)
            nominal = dict(members=[dict(**m, cones_tip_root_back=[]) for m in members], sphere_radii_mm=[1, 1])
            data = dict(nominal_supports=nominal, inputs=[], surfaces=[])
            for name, value in (("source", source), ("supports", nominal)):
                path = root/f"{name}.json"
                path.write_text(json.dumps(value))
                data[f"{name}_file"] = str(path)
                data[f"{name}_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
            for member in (0, 1):
                step = root/f"{member}.step"
                # This is a coefficient-audit fixture, not a STEP-reader test.
                step.write_bytes(b"coefficient fixture provenance")
                data["inputs"].append(dict(member=member, step_file=str(step),
                    step_sha256=hashlib.sha256(step.read_bytes()).hexdigest()))
                for role in ("tip", "root", "toe_round", "toe_flank", "heel_round", "heel_flank"):
                    s = surface(sphere(), [2, 2])
                    s = json.loads(json.dumps(s, default=float))
                    data["surfaces"].append(dict(member=member, role=role, surface=s,
                        support=dict(kind="sphere", radius_mm=1)))
            path, output = root/"coefficients.json", root/"result.json"
            path.write_text(json.dumps(data))
            result = run(path, output, {"0_toe_round"})
            self.assertTrue(result["passes_selected_bounds"])
            self.assertFalse(result["complete_closure_coverage"])
            variants = []
            changed = copy.deepcopy(data)
            changed["nominal_supports"]["sphere_radii_mm"][0] = 2
            variants.append(changed)
            changed = copy.deepcopy(data)
            changed["surfaces"][2]["support"]["radius_mm"] = 2
            variants.append(changed)
            changed = copy.deepcopy(data)
            changed["surfaces"].pop()
            variants.append(changed)
            changed = copy.deepcopy(data)
            changed["inputs"][1]["member"] = 0
            variants.append(changed)
            for index, changed in enumerate(variants):
                with self.subTest(changed=index):
                    path.write_text(json.dumps(changed))
                    with self.assertRaises(ValueError):
                        run(path, output, {"0_toe_round"})
            changed = copy.deepcopy(data)
            changed["surfaces"][3]["surface"]["weights"][0][0] = 0
            path.write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                run(path, output, {"0_toe_round", "0_toe_flank"})
            partial = json.loads(output.read_text())
            self.assertFalse(partial["complete_closure_coverage"])
            self.assertNotIn("passes_selected_bounds", partial)
            path.write_text(json.dumps(data))
            Path(data["source_file"]).write_text("{}")
            with self.assertRaisesRegex(ValueError, "source changed"):
                run(path, output, {"0_toe_round"})


if __name__ == "__main__":
    unittest.main()
