"""Whole-patch Taylor correspondence bounds for the generated CAD fillets."""
import argparse
from fractions import Fraction as F
import hashlib
import json
from pathlib import Path
import time

from bernstein import bounds, coordinate, derivative, midpoint, patches, split_patch
from check_supports import upper
from crown_fillet import check_samples, evaluate
from interval_jet import Jet, f, point


def magnitude(value):
    return max(abs(x) for x in value)


class CorrespondenceError(ValueError):
    def __init__(self, lower):
        self.lower = lower
        super().__init__("center correspondence exceeds target")


def error_bound(data, reference, patch):
    domain = patch["domain"]
    spans = [b-a for a, b in domain]
    h = [s/2 for s in spans]
    center = evaluate(data, reference, *(Jet.variable(sum(d)/2, i) for i, d in enumerate(domain)))
    box = evaluate(data, reference, *(Jet.variable(d, i, order=3) for i, d in enumerate(domain)))
    if any(p[3] != 1 for row in patch["poles"] for p in row):
        raise ValueError("Taylor checker currently requires polynomial CAD surfaces")
    errors, center_square = [], F(0)
    for axis in range(3):
        net = coordinate(patch["poles"], axis)
        ds = [derivative(net, i, spans[i]) for i in range(2)]
        hs = [derivative(ds[i], j, spans[j]) for i, j in ((0, 0), (0, 1), (1, 1))]
        ts = [derivative(hs[i], j, spans[j]) for i, j in ((0, 0), (0, 1), (1, 1), (2, 1))]
        difference = f.sub(point(midpoint(net)), center[axis].v)
        center_square += f.square(difference)[0]
        e = magnitude(difference)
        e += sum(magnitude(f.sub(point(midpoint(ds[i])), center[axis].d[i]))*h[i] for i in range(2))
        e += sum(magnitude(f.sub(point(midpoint(hs[k])), center[axis].h[k]))*h[i]*h[j]*F(1 if i == j else 2, 2)
                 for k, (i, j) in enumerate(((0, 0), (0, 1), (1, 1))))
        e += sum(magnitude(f.sub(bounds(ts[k]), box[axis].t[k]))*h[0]**(3-k)*h[1]**k*F(1 if k in (0, 3) else 3, 6)
                 for k in range(4))
        errors.append(e)
    lower = f.arithmetic.sqrt(point(center_square))[0]
    if lower > F("0.001"):
        raise CorrespondenceError(lower)
    return f.arithmetic.sqrt(point(sum(e*e for e in errors)))[1]


def run(path, output, member=0, side=0, patch_limit=None, max_cells=128):
    if max_cells <= 0 or (patch_limit is not None and patch_limit <= 0):
        raise ValueError("expected positive work and patch limits")
    data = json.loads(path.read_text())
    expected = {(m, s) for m in (0, 1) for s in (0, 1)}
    for records in (data["surfaces"], data["references"]["fillets"]):
        if len(records) != 4 or {(r["member"], r["side"]) for r in records} != expected:
            raise ValueError("incomplete or duplicate fillet definitions")
    for name in ("source", "references"):
        raw = Path(data[f"{name}_file"]).read_bytes()
        if hashlib.sha256(raw).hexdigest() != data[f"{name}_sha256"]:
            raise ValueError("changed reference input")
        if name == "references" and json.loads(raw) != data["references"]:
            raise ValueError("changed embedded reference")
        if name == "source":
            source = json.loads(raw)
    for item in data["inputs"]:
        if hashlib.sha256(Path(item["step_file"]).read_bytes()).hexdigest() != item["step_sha256"]:
            raise ValueError("changed STEP input")
    record, = [r for r in data["surfaces"] if (r["member"], r["side"]) == (member, side)]
    reference, = [r for r in data["references"]["fillets"] if (r["member"], r["side"]) == (member, side)]
    comparison = check_samples(data["references"], source, reference)
    if any(w != 1 for row in record["surface"]["weights"] for w in row):
        raise ValueError("Taylor checker currently requires polynomial CAD surfaces")
    pieces = list(patches(record["surface"]))
    if any([pieces[0]["domain"][i][0], pieces[-1]["domain"][i][1]] != [0, 1] for i in range(2)):
        raise ValueError("unexpected CAD parameter range")
    selected = pieces[:patch_limit] if patch_limit else pieces
    pending = list(reversed(list(enumerate(selected))))
    target = F("0.001")
    result = dict(evidence_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  member=member, side=side, target_mm=.001, complete_surface=False, status="running",
                  initial_patches=len(pending), total_surface_patches=len(pieces),
                  source_sample_comparison_bound_mm=upper(comparison),
                  cells_tested=0, accepted=[], unresolved=[], refinement_refusals={}, violations=[],
                  scope="Taylor correspondence to ideal common-crown reference with enclosed solved coefficients; source solve, trim coverage and global material separate")
    started = time.perf_counter()
    while pending and result["cells_tested"] < max_cells:
        original, patch = pending.pop()
        result["cells_tested"] += 1
        try:
            bound = error_bound(data["references"], reference, patch)
        except CorrespondenceError as error:
            result["violations"].append(dict(original_patch=original,
                domain=[[str(x) for x in d] for d in patch["domain"]],
                center_error_lower_bound_mm=-upper(-error.lower)))
            break
        except (AssertionError, ValueError, ZeroDivisionError) as error:
            bound = None
            reason = str(error) or type(error).__name__
            result["refinement_refusals"][reason] = result["refinement_refusals"].get(reason, 0)+1
        if bound is not None and bound <= target:
            result["accepted"].append(dict(original_patch=original, domain=[[str(x) for x in d] for d in patch["domain"]],
                                           bound_mm=upper(bound)))
        else:
            axis = max(range(2), key=lambda i: patch["domain"][i][1]-patch["domain"][i][0])
            pending.extend((original, child) for child in reversed(split_patch(patch, axis)))
        if result["cells_tested"] % 128 == 0:
            print(f"cells {result['cells_tested']}, accepted {len(result['accepted'])}, pending {len(pending)}", flush=True)
            output.write_text(json.dumps(result, indent=2)+"\n")
    success = not pending and not result["violations"]
    if success:
        def exact_floats(domain):
            converted = [[float(F(x)) for x in row] for row in domain]
            if any(F(a) != F(b) for left, right in zip(domain, converted) for a, b in zip(left, right)):
                raise ValueError("parameter partition needs an exact-rational cover adapter")
            return converted
        for i, original in enumerate(selected):
            cells = []
            for cell in result["accepted"]:
                if cell["original_patch"] == i:
                    u, v = exact_floats(cell["domain"])
                    cells.append(dict(u=u, rho=v))
            # Reuse the independent exact slab-cover audit, after checking that
            # its binary64 input adapter preserves every parameter endpoint.
            f.arithmetic.cover(exact_floats(original["domain"]), cells)
    result.update(seconds=time.perf_counter()-started, complete_surface=success and patch_limit is None,
                  status="verified" if success else "correspondence_failure" if result["violations"] else "budget_exhausted",
                  passes_selected_patches=success, unresolved=[dict(original_patch=i,
                      domain=[[str(x) for x in d] for d in p["domain"]]) for i, p in pending])
    output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k not in ("accepted", "unresolved")}), flush=True)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--member", type=int, choices=(0, 1), default=0)
    parser.add_argument("--side", type=int, choices=(0, 1), default=0)
    parser.add_argument("--patches", type=int)
    parser.add_argument("--max-cells", type=int, default=128)
    args = parser.parse_args()
    result = run(args.input, args.output, args.member, args.side, args.patches, args.max_cells)
    raise SystemExit(0 if result["passes_selected_patches"] else 1)
