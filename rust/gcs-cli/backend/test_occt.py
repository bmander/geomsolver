"""Optional native integration tests: real Solvent source -> CLI -> STEP -> OCCT."""
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader

def volume(shape):
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()

RUST = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("SOLVENTC", RUST/"target/debug/solventc"))


class StepTests(unittest.TestCase):
    def export(self, source, expected_volume, expected_bounds=None):
        with tempfile.TemporaryDirectory() as directory:
            model, step = Path(directory)/"model.sv", Path(directory)/"result.step"
            model.write_text(source)
            result = subprocess.run([str(CLI), str(model), "--step", str(step),
                                     "--no-diagnose", "--json"], text=True, capture_output=True,
                env=dict(os.environ, SOLVENT_CAD_PYTHON="/no-python-subprocess-allowed"))
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            # Kernel progress must not corrupt the CLI's structured output.
            import json
            json.loads(result.stdout)
            reader = STEPControl_Reader()
            self.assertEqual(reader.ReadFile(str(step)), IFSelect_RetDone)
            self.assertTrue(reader.TransferRoots())
            shape = reader.OneShape()
            self.assertTrue(BRepCheck_Analyzer(shape).IsValid())
            self.assertAlmostEqual(volume(shape), expected_volume, delta=expected_volume*1e-7)
            if expected_bounds:
                box = Bnd_Box()
                BRepBndLib.AddOptimal_s(shape, box, False, False)
                for actual, expected in zip(box.Get(), expected_bounds):
                    self.assertAlmostEqual(actual, expected, delta=1e-6)

    def test_inches_are_exported_in_millimetres(self):
        self.export("unit in\npoint o hint(x: 0,y: 0)\nground o\ncircle c(center: o)\n"
                    "radius(1) c\nsolid body(face(c), depth: 2)\n",
                    math.pi*25.4**2*50.8, [-25.4, 0, -25.4, 25.4, 50.8, 25.4])

    def test_tilted_profile_and_hole_use_the_same_spatial_frame(self):
        self.export("unit mm\nuse std\npoint q hint(x: 1,y: 0)\nground q\n"
            "plane p(origin: std.origin, toward: q, u: (0,1,0), v: (0,0,1))\n"
            "in p {\ncircle a(center: std.origin)\ncircle b(center: std.origin)\n"
            "radius(2) a\nradius(1) b\nsolid body(face(a, holes: b), depth: 3)\n}\n",
            math.pi*9, [-3, -2, -2, 0, 2, 2])

    def test_circular_revolution_and_clockwise_partial_revolution(self):
        source = ("unit mm\npoint a hint(x: 0,y: 0)\nground a\n"
            "point b hint(x: 0,y: 1)\nground b\nline axis(a,b)\n"
            "point c hint(x: 10,y: 2)\nground c\ncircle ring(center: c)\n"
            "radius(2) ring\nsolid body(face(ring), about: axis%s)\n")
        self.export(source % "", 80*math.pi**2, [-12, -12, 0, 12, 12, 4])
        self.export(source % ", sweep: 90deg, sense: cw", 20*math.pi**2)

    def test_component_bolt_pattern_through_additive_body(self):
        # Run the actual example in place so its project modules resolve normally.
        with tempfile.TemporaryDirectory() as directory:
            step = Path(directory)/"flange.step"
            result = subprocess.run([str(CLI), str(RUST/"examples/solid_flange.sv"),
                "--step", str(step), "--solid", "body", "--no-diagnose"],
                text=True, capture_output=True, env=dict(os.environ, SOLVENT_CAD_PYTHON="/no-python-subprocess-allowed"))
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            reader = STEPControl_Reader()
            self.assertEqual(reader.ReadFile(str(step)), IFSelect_RetDone)
            reader.TransferRoots()
            expected = math.pi*((32**2-8**2)*6+(15**2-8**2)*10-6*3**2*6)
            self.assertAlmostEqual(volume(reader.OneShape()), expected, delta=expected*1e-7)

    def test_motion_placement_uses_offset_world_axes_and_model_units(self):
        self.export("unit in\npoint a hint(x: 1,y: 0)\nground a\n"
            "point b hint(x: 1,y: 1)\nground b\nline axis(a,b)\n"
            "point c hint(x: 2,y: 0)\nground c\ncircle ring(center: c)\n"
            "radius(0.25) ring\nsolid stock(face(ring), depth: 0.5)\n"
            "motion turn(about: axis)\nsolid moved(stock, under: turn, at: 90deg)\n",
            math.pi*(0.25*25.4)**2*(0.5*25.4),
            [0.5*25.4,0.75*25.4,-0.25*25.4,25.4,1.25*25.4,0.25*25.4])

    def test_indexed_motion_component_reuses_one_through_cutter(self):
        self.export((RUST/"examples/solid_indexed_pattern.sv").read_text(),
                    math.pi*(400-6*4)*5, [-20,-20,-5,20,20,0])

    def test_unsupported_operation_preserves_existing_export(self):
        with tempfile.TemporaryDirectory() as directory:
            step = Path(directory)/"existing.step"
            step.write_text("existing export")
            result = subprocess.run([str(CLI), str(RUST/"examples/solid_elbow.sv"),
                "--step", str(step), "--no-diagnose"], text=True, capture_output=True,
                env=dict(os.environ, SOLVENT_CAD_PYTHON="/no-python-subprocess-allowed"))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("along-guide lofts", result.stderr)
            self.assertEqual(step.read_text(), "existing export")

    def test_unitless_and_unsolved_models_do_not_replace_an_export(self):
        source = ("point o hint(x: 0,y: 0)\nground o\ncircle c(center: o)\n"
                  "radius(2) c\nsolid body(face(c), depth: 3)\n")
        for model_source, diagnostic in [
            (source, "explicit model length unit"),
            ("unit mm\n"+source+"radius(3) c\n", "requires a solved model"),
        ]:
            with tempfile.TemporaryDirectory() as directory:
                model, step = Path(directory)/"model.sv", Path(directory)/"result.step"
                model.write_text(model_source)
                step.write_text("existing export")
                result = subprocess.run([str(CLI), str(model), "--step", str(step),
                    "--allow-unsolved", "--no-diagnose"], text=True, capture_output=True,
                    env=dict(os.environ, SOLVENT_CAD_PYTHON="/no-python-subprocess-allowed"))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(diagnostic, result.stderr)
                self.assertEqual(step.read_text(), "existing export")

    def test_native_exception_becomes_a_diagnostic_without_replacing_the_export(self):
        with tempfile.TemporaryDirectory() as directory:
            model, step = Path(directory)/"model.sv", Path(directory)/"result.step"
            model.write_text("unit mm\npoint o hint(x: 0,y: 0)\nground o\n"
                "circle c(center: o)\nradius(2) c\nsolid stock(face(c), depth: 3)\n"
                "solid body(stock)\nstock cut body\n")
            step.write_text("existing export")
            result = subprocess.run([str(CLI), str(model), "--step", str(step),
                "--solid", "body", "--no-diagnose"], text=True, capture_output=True)
            self.assertEqual(result.returncode, 1, result.stdout+result.stderr)
            self.assertIn("operation produced no solid", result.stderr)
            self.assertEqual(step.read_text(), "existing export")


if __name__ == "__main__":
    unittest.main()
