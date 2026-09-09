"""Build a configured inspection pair and run its existing export/engagement checks.

Run with the Python interpreter containing the pinned CAD experiment dependencies.
The workflow succeeds only if all listed checks and negative controls complete;
success is explicitly separate from production or continuous-geometry acceptance.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FRACTIONS = [0., .25, .5, .75, 1.]
PENDING = [
    "Global material coverage and interference between sampled assembly phases",
    "End-to-end exported geometry accuracy, including fitting and tessellation",
    "Supported parameter domain and explicit tooth thickness/backlash design",
    "Ordinary Solvent solid outputs through the public application/export interface",
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def check_assembly(result, fractions, offset, expect_overlap=False):
    rows = result["samples"]
    if ([r["tooth_fraction"] for r in rows] != fractions
            or result["gear_phase_offset_rad"] != offset):
        raise ValueError("assembly check has incomplete or incorrect phase coverage")
    if not rows or not all(r["done"] and r["valid"] for r in rows):
        raise ValueError("assembly intersection operation failed")
    if not all(math.isfinite(r["volume_mm3"]) and r["volume_mm3"] >= 0 for r in rows):
        raise ValueError("invalid assembly intersection volume")
    overlaps = [r["solids"] > 0 or r["volume_mm3"] > 0 for r in rows]
    if expect_overlap:
        if not any(r["solids"] > 0 and r["volume_mm3"] > 0 for r in rows):
            raise ValueError("wrong-phase control did not detect volumetric overlap")
    elif any(overlaps):
        raise ValueError("nominal assembly has sampled volumetric overlap")


def check_contacts(result, offset, negative=False):
    if len(result["samples"]) != 250 or result["gear_phase_offset_rad"] != offset:
        raise ValueError("contact check incomplete or at the wrong phase offset")
    if negative:
        if (result["passes_sampled_checks"] or not any(
                r["reason"] == "contact misses CAD boundary" for r in result["failures"])):
            raise ValueError("wrong-phase control did not detect a missed contact")
    elif not result["passes_sampled_checks"] or result["failures"]:
        raise ValueError("nominal CAD contact check failed")


class Workflow:
    def __init__(self, output, configuration):
        # Never mix a new run with a previous run's successful evidence.
        output.mkdir(parents=True, exist_ok=False)
        self.output = output
        self.report = dict(configuration=configuration, stages=[], status="running",
                           acceptance="incomplete", pending=PENDING)
        self.env = dict(os.environ)
        self.env.update(SOLVENT_CAD_PINION_TEETH=str(configuration["teeth"][0]),
                        SOLVENT_CAD_GEAR_TEETH=str(configuration["teeth"][1]),
                        SOLVENT_CAD_MODULE_MM=str(configuration["mean_module_mm"]))
        self.save()

    def save(self):
        path = self.output/"report.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.report, indent=2, allow_nan=False)+"\n")
        temporary.replace(path)

    def stage(self, name, command, expected=0):
        log = self.output/f"{name}.log"
        row = dict(name=name, command=list(map(str, command)), log=log.name, status="running")
        self.report["stages"].append(row)
        self.save()
        print(f"Starting {name}; log: {log}", flush=True)
        started = time.perf_counter()
        with log.open("w") as stream:
            result = subprocess.run(row["command"], cwd=ROOT, env=self.env,
                                    stdout=stream, stderr=subprocess.STDOUT)
        row.update(seconds=time.perf_counter()-started, returncode=result.returncode,
                   status="completed" if result.returncode == expected else "failed")
        self.save()
        if result.returncode != expected:
            raise RuntimeError(f"{name} exited {result.returncode}; see {log}")
        print(f"Completed {name} in {row['seconds']:.2f}s", flush=True)
        return log

    def python(self, name, script, *args, expected=0):
        return self.stage(name, [sys.executable, HERE/script, *args], expected)

    def export(self, name, test, variable, path):
        self.env[variable] = str(path)
        self.stage(name, ["cargo", "test", "--manifest-path", ROOT/"rust/Cargo.toml",
                         test, "--", "--ignored", "--nocapture"])
        if not path.is_file():
            raise ValueError(f"{test} did not write its required artifact")

    def run(self):
        out, config = self.output, self.report["configuration"]
        try:
            self.stage("revision", ["git", "rev-parse", "HEAD"])
            self.stage("working-tree", ["git", "status", "--porcelain"])
            self.stage("working-diff", ["git", "diff", "HEAD", "--binary"])
            self.stage("dependencies", [sys.executable, "-m", "pip", "freeze"])
            self.report["revision"] = (out/"revision.log").read_text().strip()
            self.report["source_files"] = {
                str(p.relative_to(ROOT)): digest(p)
                for directory in [ROOT/"rust/examples/spiral_bevel", HERE]
                for p in sorted(directory.rglob("*")) if p.suffix in (".sv", ".py")}
            self.report["python"] = sys.version
            source, contacts = out/"sections.json", out/"contacts.json"
            self.export("sections", "export_tooth_space_sections_for_cad_backend",
                        "SOLVENT_CAD_SECTIONS_OUTPUT", source)
            data = read(source)
            if ([m["teeth"] for m in data["members"]] != config["teeth"]
                    or data["module_mm"] != config["mean_module_mm"]):
                raise ValueError("exported source does not match requested parameters")
            self.export("contact-reference", "export_contact_windows_for_cad_backend",
                        "SOLVENT_CAD_CONTACTS_OUTPUT", contacts)
            spaces = out/"spaces"
            self.python("tooth-spaces", "tooth_space.py", source, spaces, "--subdivisions", 16)
            reports = []
            for member, name in enumerate(("pinion", "gear")):
                directory = out/name
                self.python(f"{name}-cuts", "indexed_cuts.py", source, spaces, directory,
                            "--full", "--member", member)
                reports.append(directory/"report.json")
                step = Path(read(reports[-1])["results"][0]["step_file"])
                stl = directory/f"{name}.stl"
                self.python(f"{name}-stl", "export_stl.py", step, stl,
                            "--deflection", config["stl_deflection_mm"])
                self.stage(f"{name}-embedding", [sys.executable,
                    ROOT/"rust/gcs-core/tests/verification/stl_embedding.py", stl])
            assembly = out/"assembly.json"
            self.python("assembly", "assembly.py", source, *reports, assembly,
                        "--fractions", *FRACTIONS)
            check_assembly(read(assembly), FRACTIONS, 0.)
            checked = out/"checked-contacts.json"
            self.python("contacts", "check_contacts.py", source, *reports, contacts, checked)
            check_contacts(read(checked), 0.)
            wrong_assembly, wrong_contacts = out/"wrong-assembly.json", out/"wrong-contacts.json"
            self.python("wrong-assembly", "assembly.py", source, *reports, wrong_assembly,
                        "--fractions", 0, "--gear-offset", .001)
            check_assembly(read(wrong_assembly), [0.], .001, expect_overlap=True)
            self.python("wrong-contacts", "check_contacts.py", source, *reports, contacts,
                        wrong_contacts, "--gear-offset", .001, expected=1)
            check_contacts(read(wrong_contacts), .001, negative=True)
            self.report.update(status="completed", checks=dict(
                native_and_step="passed", encoded_stl_embedding="passed",
                sampled_assembly="passed", sampled_contact="passed", wrong_phase_controls="passed"))
        except BaseException as error:
            self.report.update(status="failed", error=f"{type(error).__name__}: {error}")
            raise
        finally:
            self.report["artifacts"] = {str(p.relative_to(out)): digest(p)
                for p in sorted(out.rglob("*")) if p.is_file() and p != out/"report.json"}
            self.save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="new output directory (must not exist)")
    parser.add_argument("--teeth", type=int, nargs=2, default=[24, 48], metavar=("PINION", "GEAR"))
    parser.add_argument("--module-mm", type=float, default=2.)
    parser.add_argument("--stl-deflection-mm", type=float, default=.01)
    args = parser.parse_args()
    if (min(args.teeth) <= 0 or not all(math.isfinite(v) and v > 0
            for v in (args.module_mm, args.stl_deflection_mm))):
        parser.error("tooth counts, module and deflection must be positive and finite")
    config = dict(teeth=args.teeth, mean_module_mm=args.module_mm,
                  geometry_accuracy_target_mm=.0254,
                  stl_deflection_mm=args.stl_deflection_mm, shaft_angle_deg=90,
                  spiral_angle_deg=35, pressure_angle_deg=20, nominal_backlash_mm=0,
                  generating_system="documented complementary common-crown references",
                  design_document="docs/spiral-bevel-design.md")
    Workflow(args.output.resolve(), config).run()
    print(f"Inspection workflow completed. Acceptance remains incomplete: {args.output/'report.json'}")


if __name__ == "__main__":
    main()
