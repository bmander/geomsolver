"""Independent common-crown fillet parameterization, including interval derivatives."""
from fractions import Fraction as F
from interval_jet import Jet, f, point
from crown_geometry import generated, radius


def check_samples(data, source, record, evaluator=None, grid_offset=0):
    evaluator = evaluator or evaluate
    if (data["teeth"] != [m["teeth"] for m in source["members"]]
            or data["module_mm"] != source["module_mm"]
            or data["rho_range"] != [.9*source["mean_distance"], 1.1*source["mean_distance"]]):
        raise ValueError("nominal reference disagrees with the CAD source parameters")
    member = source["members"][record["member"]]
    level = next(l for l in member["levels"] if l["subdivisions"] == 32)
    maximum = F(0)
    # The source grids precede the gear-side indexing used to form its tooth space.
    raw = dict(record, index_angle=0.)
    for i in (0, 16, 32):
        for j in (0, 16, 32):
            expected = level["sides"][record["side"]][i][j+grid_offset]
            found = evaluator(data, raw, Jet(F(i, 32)), Jet(F(j, 32)))
            difference = [f.sub(value.v, point(x)) for value, x in zip(found, expected)]
            bound = f.arithmetic.sqrt(point(sum(max(abs(x) for x in d)**2 for d in difference)))[1]
            maximum = max(maximum, bound)
    if maximum > F("0.0000001"):
        raise ValueError("independent reference misses the original source samples")
    return maximum


def evaluate(data, record, u, v):
    rho = radius(data, u)
    parameter = record["u_range"][0]+v*(record["u_range"][1]-record["u_range"][0])
    sine, cosine = (parameter*record["sweep"]).sin_cos()
    a, b, center = [[Jet(c) for c in record[key]] for key in ("a", "b", "center")]
    profile = [c+x*cosine+y*sine for c, x, y in zip(center, a, b)]
    derivative = [(-x*sine+y*cosine)*record["sweep"] for x, y in zip(a, b)]
    return generated(data, record, profile, derivative, rho)
