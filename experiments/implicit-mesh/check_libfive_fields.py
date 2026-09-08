"""Cross-check native expression fixtures against separately evaluated Python fields."""
import random
import subprocess
import sys

import audit
import manifold_fixtures as fixtures

if len(sys.argv) != 2:
    raise SystemExit("usage: python3 check_libfive_fields.py PATH_TO_BENCHMARK")
rng = random.Random(97343)
requests, expected = [], []
for name in fixtures.NAMES:
    field, extent = fixtures.build(name)
    points = [tuple(rng.uniform(-extent, extent) for _ in range(3)) for _ in range(32)]
    if name != "zero_only":
        points += audit.reference(name)[1]
    for point in points:
        requests.append(name+" "+" ".join(map(str, point)))
        expected.append(field(point))
result = subprocess.run([sys.argv[1], "--sample"], input="\n".join(requests)+"\n",
                        text=True, capture_output=True, check=True)
actual = list(map(float, result.stdout.splitlines()))
assert len(actual) == len(expected), result.stdout
error = max(abs(a-b) for a, b in zip(actual, expected))
assert error < 2e-6, error  # Native evaluator uses binary32 input and arithmetic.
print(f"{len(actual)} field samples agree within {error:.3g} model units")
