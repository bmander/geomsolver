"""Optional native integration tests: real Solvent source -> CLI -> STEP -> OCCT."""
import math
import struct
from collections import Counter
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

def check_stl(test, path, expected_volume):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    test.assertGreater(count, 0)
    test.assertEqual(len(data), 84+50*count)
    edges, signed_volume = Counter(), 0.0
    for offset in range(84, len(data), 50):
        a, b, c = [struct.unpack_from("<fff", data, offset+12+12*j) for j in range(3)]
        test.assertTrue(all(math.isfinite(x) for p in (a,b,c) for x in p))
        cross = lambda a,b: (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
        normal = cross(tuple(b[i]-a[i] for i in range(3)), tuple(c[i]-a[i] for i in range(3)))
        test.assertGreater(sum(x*x for x in normal), 0)
        signed_volume += sum(a[i]*cross(b,c)[i] for i in range(3))/6
        for p,q in [(a,b),(b,c),(c,a)]:
            edges[p,q] += 1
    test.assertTrue(all(n == 1 and edges[q,p] == 1 for (p,q),n in edges.items()))
    test.assertAlmostEqual(signed_volume, expected_volume, delta=expected_volume*.005)
    return count

RUST = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("SOLVENTC", RUST/"target/debug/solventc"))


class StepTests(unittest.TestCase):
    def export(self, source, expected_volume, expected_bounds=None, stl=False):
        with tempfile.TemporaryDirectory() as directory:
            model, step = Path(directory)/"model.sv", Path(directory)/"result.step"
            model.write_text(source)
            result = subprocess.run([str(CLI), str(model), "--step", str(step),
                                     "--no-diagnose", "--json"] + (["--stl", str(Path(directory)/"result.stl")] if stl else []), text=True, capture_output=True,
                env=dict(os.environ, SOLVENT_CAD_PYTHON="/no-python-subprocess-allowed"))
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            if stl:
                check_stl(self, Path(directory)/"result.stl", expected_volume)
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
        self.export("unit in\npoint o hint((0, 0))\nground o\ncircle c(center: o)\n"
                    "radius(1) c\nsolid body(face(c), depth: 2)\n",
                    math.pi*25.4**2*50.8, [-25.4, 0, -25.4, 25.4, 50.8, 25.4])

    def test_tilted_profile_and_hole_use_the_same_spatial_frame(self):
        self.export("unit mm\nuse std\npoint q hint((1, 0))\nground q\n"
            "plane p(origin: std.origin, toward: q, u: (0,1,0), v: (0,0,1))\n"
            "in p {\ncircle a(center: std.origin)\ncircle b(center: std.origin)\n"
            "radius(2) a\nradius(1) b\nsolid body(face(a, holes: b), depth: 3)\n}\n",
            math.pi*9, [-3, -2, -2, 0, 2, 2], stl=True)

    def test_circular_revolution_and_clockwise_partial_revolution(self):
        source = ("unit mm\npoint a hint((0, 0))\nground a\n"
            "point b hint((0, 1))\nground b\nline axis(a,b)\n"
            "point c hint((10, 2))\nground c\ncircle ring(center: c)\n"
            "radius(2) ring\nsolid body(face(ring), about: axis%s)\n")
        self.export(source % "", 80*math.pi**2, [-12, -12, 0, 12, 12, 4], stl=True)
        self.export(source % ", sweep: 90deg, sense: cw", 20*math.pi**2, stl=True)

    def test_component_bolt_pattern_through_additive_body(self):
        # Run the actual example in place so its project modules resolve normally.
        with tempfile.TemporaryDirectory() as directory:
            step = Path(directory)/"flange.step"
            result = subprocess.run([str(CLI), str(RUST/"examples/solid_flange.sv"),
                "--step", str(step), "--solid", "flange", "--no-diagnose"],
                text=True, capture_output=True, env=dict(os.environ, SOLVENT_CAD_PYTHON="/no-python-subprocess-allowed"))
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            reader = STEPControl_Reader()
            self.assertEqual(reader.ReadFile(str(step)), IFSelect_RetDone)
            reader.TransferRoots()
            expected = math.pi*((32**2-8**2)*6+(15**2-8**2)*10-6*3**2*6)
            self.assertAlmostEqual(volume(reader.OneShape()), expected, delta=expected*1e-7)

    def test_motion_placement_uses_offset_world_axes_and_model_units(self):
        self.export("unit in\npoint a hint((1, 0))\nground a\n"
            "point b hint((1, 1))\nground b\nline axis(a,b)\n"
            "point c hint((2, 0))\nground c\ncircle ring(center: c)\n"
            "radius(0.25) ring\nsolid stock(face(ring), depth: 0.5)\n"
            "motion turn(about: axis)\nsolid moved(stock, under: turn, at: 90deg)\n",
            math.pi*(0.25*25.4)**2*(0.5*25.4),
            [0.5*25.4,0.75*25.4,-0.25*25.4,25.4,1.25*25.4,0.25*25.4])

    def test_placement_binds_motion_after_dependency_ordering(self):
        self.export("unit in\npoint a hint((1, 0))\nground a\n"
            "point b hint((1, 1))\nground b\nline axis(a,b)\n"
            "point c hint((2, 0))\nground c\ncircle ring(center: c)\n"
            "radius(0.25) ring\nsolid stock(face(ring), depth: 0.5)\n"
            "motion z_turn(about: axis,phase: 90deg)\n"
            "motion rest(about: axis)\nmotion a_relative(z_turn,relative_to: rest)\n"
            "component Copy(body: solid,move: motion) {\n"
            "solid result(body,under: move,at: 0deg)\n}\ncopy: Copy(stock,a_relative)\n",
            math.pi*(0.25*25.4)**2*(0.5*25.4),
            [0.5*25.4,0.75*25.4,-0.25*25.4,25.4,1.25*25.4,0.25*25.4], stl=True)

    def test_indexed_motion_component_reuses_one_through_cutter(self):
        self.export((RUST/"examples/solid_indexed_pattern.sv").read_text(),
                    math.pi*(400-6*4)*5, [-20,-20,-5,20,20,0], stl=True)

    def test_native_stl_alone_uses_millimetres_and_checks_multiple_shells(self):
        with tempfile.TemporaryDirectory() as directory:
            model, stl = Path(directory)/"model.sv", Path(directory)/"result.stl"
            model.write_text("unit in\npoint a hint((0, 0))\nground a\n"
                "point b hint((4, 0))\nground b\ncircle ca(center: a)\n"
                "radius(1) ca\ncircle cb(center: b)\nradius(1) cb\n"
                "solid first(face(ca), depth: 2)\nsolid second(face(cb), depth: 2)\n"
                "solid body(first)\nsecond on body\n")
            result = subprocess.run([str(CLI), str(model), "--stl", str(stl), "--no-diagnose"],
                text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            check_stl(self, stl, 4*math.pi*25.4**3)

    def test_failed_stl_encoding_preserves_both_requested_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            model, step, stl = [Path(directory)/n for n in ("model.sv", "result.step", "result.stl")]
            model.write_text("unit mm\npoint c hint((1000000, 1000000))\nground c\n"
                "circle profile(center: c)\nradius(0.01) profile\nsolid body(face(profile), depth: 0.01)\n")
            step.write_text("old step")
            stl.write_text("old stl")
            result = subprocess.run([str(CLI), str(model), "--step", str(step), "--stl", str(stl),
                "--no-diagnose"], text=True, capture_output=True)
            self.assertEqual(result.returncode, 1, result.stdout+result.stderr)
            self.assertIn("float32 STL validation failed", result.stderr)
            self.assertEqual(step.read_text(), "old step")
            self.assertEqual(stl.read_text(), "old stl")
            self.assertFalse(list(Path(directory).glob(".solvent-cad-*")))

    def test_native_stl_preserves_an_enclosed_cavity(self):
        self.export("unit mm\npoint o hint((0, 0))\nground o\n"
            "circle rim(center: o)\nradius(3) rim\ncircle hole(center: o)\nradius(1) hole\n"
            "solid stock(face(rim), depth: 6)\nsolid tool(face(hole), from: -4, to: -2)\n"
            "solid body(stock)\ntool cut body\n", 52*math.pi, stl=True)

    def test_step_and_stl_cannot_share_a_destination(self):
        with tempfile.TemporaryDirectory() as directory:
            model, output = Path(directory)/"model.sv", Path(directory)/"result"
            model.write_text("unit mm\npoint c hint((0, 0))\nground c\n"
                "circle profile(center: c)\nradius(2) profile\nsolid body(face(profile), depth: 3)\n")
            output.write_text("old output")
            result = subprocess.run([str(CLI), str(model), "--step", str(output), "--stl", str(output),
                "--no-diagnose"], text=True, capture_output=True)
            self.assertEqual(result.returncode, 1, result.stdout+result.stderr)
            self.assertIn("distinct output paths", result.stderr)
            self.assertEqual(output.read_text(), "old output")

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

    def test_continuous_sweep_refusal_preserves_both_exports(self):
        with tempfile.TemporaryDirectory() as directory:
            step, stl = Path(directory)/"sweep.step", Path(directory)/"sweep.stl"
            step.write_text("old step")
            stl.write_text("old stl")
            result = subprocess.run([str(CLI), str(RUST/"examples/solid_generating_sweep.sv"),
                "--solid", "removal.body", "--step", str(step), "--stl", str(stl), "--no-diagnose"],
                text=True, capture_output=True)
            self.assertEqual(result.returncode, 1, result.stdout+result.stderr)
            self.assertIn("native CAD boundary construction for continuous motion sweeps", result.stderr)
            self.assertEqual(step.read_text(), "old step")
            self.assertEqual(stl.read_text(), "old stl")

    def test_unitless_and_unsolved_models_do_not_replace_an_export(self):
        source = ("point o hint((0, 0))\nground o\ncircle c(center: o)\n"
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
            model.write_text("unit mm\npoint o hint((0, 0))\nground o\n"
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
