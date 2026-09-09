"""Analytical derivatives and a different high-order transcendental reference."""
from fractions import Fraction as F
import unittest
from unittest.mock import patch

from bernstein import split_patch
from check_generated import CorrespondenceError, error_bound
from interval_jet import Jet, atan_point, sin_cos, f
from interval_arithmetic import reference_trig


class Jets(unittest.TestCase):
    def encloses(self, actual, expected):
        if not isinstance(expected, tuple):
            expected = (expected, expected)
        self.assertLessEqual(actual[0], expected[0])
        self.assertGreaterEqual(actual[1], expected[1])

    def test_reciprocal_and_sqrt_derivatives(self):
        x = Jet.variable(4, 0)
        for result, expected in ((x.reciprocal(), (F(1, 4), -F(1, 16), F(1, 32))),
                                 (x.sqrt(), (2, F(1, 4), -F(1, 32)))):
            for actual, wanted in zip((result.v, result.d[0], result.h[0]), expected):
                self.encloses(actual, wanted)

    def test_trig_and_mixed_chain_rule_against_longer_taylor_reference(self):
        for x in (F(-1), F(0), F(1, 2), F(2)):
            sine, cosine = sin_cos((x, x))
            self.encloses(sine, reference_trig(x, False))
            self.encloses(cosine, reference_trig(x, True))
        x, y = Jet.variable(1, 0), Jet.variable(2, 1)
        value, _ = (x*y).sin_cos()
        s, c = reference_trig(F(2), False), reference_trig(F(2), True)
        self.encloses(value.h[0], f.arithmetic.mul((-F(4), -F(4)), s))
        self.encloses(value.h[1], f.arithmetic.sub(c, f.arithmetic.mul((F(2), F(2)), s)))
        self.encloses(value.h[2], f.arithmetic.neg(s))

    def test_atan_against_machin_identity_and_exact_derivatives(self):
        def series(x):
            s = sum((-1)**k*x**(2*k+1)/F(2*k+1) for k in range(81))
            e = x**163/163
            return s-e, s+e
        a, b = series(F(1, 5)), series(F(1, 239))
        quarter_pi = (4*a[0]-b[1], 4*a[1]-b[0])
        self.encloses(atan_point(F(1)), quarter_pi)
        result = Jet.variable(1, 0).atan()
        self.encloses(result.d[0], F(1, 2))
        self.encloses(result.h[0], -F(1, 2))
        with self.assertRaises(ValueError):
            atan_point(F(3))
        with self.assertRaises(ValueError):
            sin_cos((F(5), F(5)))

    def test_third_derivatives_and_mixed_leibniz_terms(self):
        x = Jet.variable(4, 0, order=3)
        self.encloses(x.sqrt().t[0], F(3, 256))
        self.encloses(x.reciprocal().t[0], -F(3, 128))
        self.encloses(Jet.variable(1, 0, order=3).atan().t[0], F(1, 2))
        x, y = Jet.variable(1, 0, order=3), Jet.variable(2, 1, order=3)
        product = x*x*y*y
        self.encloses(product.t[1], 8)
        self.encloses(product.t[2], 4)
        sine, _ = (x*y).sin_cos()
        s, c = reference_trig(F(2), False), reference_trig(F(2), True)
        for actual, cs, cc in zip(sine.t, (0, -4, -2, 0), (-8, -4, -2, -1)):
            self.encloses(actual, f.arithmetic.add(f.arithmetic.mul((F(cs), F(cs)), s),
                                                  f.arithmetic.mul((F(cc), F(cc)), c)))
        with self.assertRaises(ValueError):
            Jet((2, 1))

    def test_taylor_bound_and_subdivision_on_exact_paraboloid(self):
        fixture = dict(domain=[[F(0), F(1)], [F(0), F(1)]], poles=[
            [(F(i, 2), F(j, 2), F(int(i == 2)+int(j == 2)), F(1)) for j in range(3)]
            for i in range(3)])
        def reference(data, record, u, v):
            return [u, v, u*u+v*v]
        with patch("check_generated.evaluate", reference):
            self.assertLess(error_bound({}, {}, fixture), F(1, 10**40))
            children = [q for p in split_patch(fixture, 0) for q in split_patch(p, 1)]
            self.assertEqual(sum((p["domain"][0][1]-p["domain"][0][0])*
                                 (p["domain"][1][1]-p["domain"][1][0]) for p in children), 1)
            for child in children:
                self.assertLess(error_bound({}, {}, child), F(1, 10**40))
            x, y, z, w = fixture["poles"][1][1]
            fixture["poles"][1][1] = x, y, z+F(1, 10), w
            with self.assertRaises(CorrespondenceError) as caught:
                error_bound({}, {}, fixture)
            self.assertGreater(caught.exception.lower, F("0.001"))


if __name__ == "__main__":
    unittest.main()
