"""Independent controls for the moving tip root and its implicit derivatives."""
from fractions import Fraction as F
import json
import unittest
from unittest.mock import patch

from crown_flank import tip_function, tip_height
from interval_jet import Jet


class TipRoot(unittest.TestCase):
    def setUp(self):
        tip_function.cache_clear()

    def encloses(self, actual, expected):
        self.assertLessEqual(actual[0], expected)
        self.assertGreaterEqual(actual[1], expected)

    def test_moving_root_with_exact_derivatives(self):
        # T(q) = sqrt(1+q)/2; exercise the mixed third-order formula too
        # by multiplying the equation by a positive, nonconstant factor.
        def equation(data, record, q, t):
            return (t*t-(1+q)/4)*(1+q*t)
        with patch('crown_flank.residual', equation):
            value, d, h, third = tip_function('[{},{}]', (F(0), F(0)), 3)
            for actual, expected in zip((value, d, h, third), (F(1,2), F(1,4), -F(1,8), F(3,16))):
                self.encloses(actual, expected)
                self.assertLess(actual[1]-actual[0], F(1,10**15))
            # A whole q interval must retain the roots at both endpoints.
            value, *_ = tip_function('[{},{}]', (F(0), F(9,16)), 3)
            self.encloses(value, F(1,2))
            self.encloses(value, F(5,8))

    def test_cache_binds_coefficients_and_domain(self):
        def equation(data, record, q, t):
            return t-record['offset']-q/8
        with patch('crown_flank.residual', equation):
            for offset, q in ((.25,F(0)), (.5,F(0)), (.25,F(1))):
                encoded = json.dumps([{}, dict(offset=offset)])
                value, *_ = tip_function(encoded, (q,q), 0)
                self.encloses(value, F(offset)+q/8)
                self.assertLess(value[1]-value[0], F(1,10**15))

    def test_missing_and_nonuniform_roots_are_refused(self):
        for equation, domain in ((lambda d,r,q,t: t*t+1, (F(0),F(0))),
                                 (lambda d,r,q,t: t-q, (-F(1,4),F(3,4)))):
            tip_function.cache_clear()
            with patch('crown_flank.residual', equation), self.assertRaises(ValueError):
                tip_function('[{},{}]', domain, 3)

    def test_exact_cone_sphere_branch_and_derivatives(self):
        # Cone radius = z. At rho = sqrt(2)*z use rational rho=2,
        # so z = sqrt(2), z' = 1/sqrt(2), z'' = z''' = 0.
        record = dict(tip_meridian=[[1,0,1], [2,0,2]])
        result = tip_height(record, Jet.variable(2,0,3))
        self.assertLessEqual(result.v[0]**2, 2)
        self.assertGreaterEqual(result.v[1]**2, 2)
        self.assertLessEqual(result.d[0][0]**2, F(1,2))
        self.assertGreaterEqual(result.d[0][1]**2, F(1,2))
        self.encloses(result.h[0], 0)
        self.encloses(result.t[0], 0)
        with self.assertRaises(ValueError):
            tip_height(dict(tip_meridian=[[2,0,2],[1,0,1]]), Jet(2))


if __name__ == '__main__':
    unittest.main()
