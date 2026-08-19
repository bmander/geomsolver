from gcs import examples, io, solve
from gcs import constraints as C


def test_roundtrip_all_examples() -> None:
    for name, make in examples.EXAMPLES.items():
        sk = make()
        s = io.dumps(sk)
        sk2 = io.loads(s)
        assert io.dumps(sk2) == s, name
        assert len(sk2.constraints) == len(sk.constraints)
        assert solve(sk2).success


def test_without_removes_dependents() -> None:
    sk = examples.rect_fillets()
    n_arc_c = sum(isinstance(c, C.TangentArcLine) for c in sk.constraints)
    sk2 = io.without(sk, entities=[sk.arcs[0].center])
    assert len(sk2.arcs) == 3
    assert sum(isinstance(c, C.TangentArcLine) for c in sk2.constraints) == n_arc_c - 2
    io.dumps(sk2)  # every kept constraint references only live entities


def test_without_line_keeps_points() -> None:
    sk = examples.truss(3)
    n_pts = len(sk.points)
    sk2 = io.without(sk, entities=[sk.lines[0]])
    assert len(sk2.points) == n_pts
    assert len(sk2.lines) == len(sk.lines) - 1


def test_every_constraint_type_has_spec_and_roundtrips() -> None:
    """Every concrete Constraint subclass declares a spec that reconstructs it."""
    from tests.test_jacobians import all_constraints

    seen = set()
    for c in all_constraints(0):
        seen.add(type(c))
        assert c.spec, type(c)
        c2 = type(c)(*c.args())
        assert c2.args() == c.args()
    assert seen >= {t for t in io.BY_NAME.values() if not t.__name__.startswith("_") and t is not C.Constraint} - {C._TwoLine}


def test_describe() -> None:
    sk = examples.rect_fillets()
    assert io.describe(sk.constraints[-1], sk) == "Distance(P6, P7, 40)"
