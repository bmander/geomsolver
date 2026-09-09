"""Exact curves, independent trig series and known edge-correspondence failures."""
from copy import deepcopy
from fractions import Fraction as F
from math import factorial
import unittest

from curve_jets import Curve,correspondence_bound,elementary_bound,periodic_trig
from curve_taylor import Taylor
from edge_geometry import iso_curve,same_basis_bound
from interval_arithmetic import reference_trig
from interval_jet import f


class EdgeGeometry(unittest.TestCase):
    def encloses(self,interval,value):
        self.assertLessEqual(interval[0],value);self.assertGreaterEqual(interval[1],value)

    def test_taylor_product_and_sine_composition_through_sixth_order(self):
        x=Taylor.variable(F(1,2),4)
        result=x*x*x*x
        for actual,wanted in zip(result.c,(F(1,16),F(1,2),F(3,2),F(2),F(1))):self.encloses(actual,wanted)
        x=Taylor.variable(0,6)
        sine,cosine=(x*x).sin_cos()
        for i in range(7):
            self.encloses(sine.c[i],1 if i==2 else -F(1,6) if i==6 else 0)
            self.encloses(cosine.c[i],1 if i==0 else -F(1,2) if i==4 else 0)
        with self.assertRaises(ValueError):Taylor.variable(1,2)+Taylor.variable(1,3)

    def test_period_reduction_against_independent_long_series(self):
        sine,cosine=periodic_trig(Taylor.variable(6,4))
        s,c=reference_trig(F(6),False),reference_trig(F(6),True)
        for k,expected in enumerate((s,c,f.neg(s),f.neg(c),s)):
            for bound in expected:
                self.assertLessEqual(sine.c[k][0],bound/factorial(k))
                self.assertGreaterEqual(sine.c[k][1],bound/factorial(k))

    def test_polynomial_derivatives_and_end_extension(self):
        record=dict(kind='bspline',degree=3,knots=[0]*4+[2]*4,weights=[1]*4,
                    poles=[[F(2*i,3),8 if i==3 else 0,0] for i in range(4)],interval=[0,2])
        curve=Curve(record)
        for t in (F(1),F(3)):
            x,y,z=curve.evaluate(Taylor.variable(t,4))
            for actual,wanted in zip(x.c,(t,1,0,0,0)):self.encloses(actual,wanted)
            for actual,wanted in zip(y.c,(t**3,3*t*t,3*t,1,0)):self.encloses(actual,wanted)

    def test_reverse_affine_iso_parameter_and_distortion(self):
        surface=dict(degrees=[1,1],knots=[[0,0,1,1]]*2,weights=[[1,1],[1,1]],
                     poles=[[[0,0,0],[0,1,1]],[[1,0,1],[1,1,2]]])
        parameter=dict(kind='line',location=[2,1],direction=[-1,0],interval=[1,2])
        actual=iso_curve(surface,parameter)
        expected=dict(kind='bspline',degree=1,knots=[1,1,2,2],weights=[1,1],poles=[[1,1,2],[0,1,1]],interval=[1,2])
        self.assertLess(same_basis_bound(expected,actual),F(1,10**40))
        altered=deepcopy(expected);altered['poles'][0][2]+=1
        self.assertGreaterEqual(same_basis_bound(altered,actual),1)

    def test_circle_factorization_and_correspondence_failure(self):
        sphere=dict(kind='sphere',origin=[0,0,0],radius=2,basis=[[1,0,0],[0,1,0],[0,0,1]])
        parameter=dict(kind='line',location=[0,0],direction=[0,1],interval=[0,6])
        circle=dict(kind='circle',origin=[0,0,0],radius=2,basis=[[1,0,0],[0,0,1]],interval=[0,6])
        self.assertLess(elementary_bound(circle,parameter,sphere),F(1,10**40))
        circle['origin'][1]=.01
        self.assertGreaterEqual(elementary_bound(circle,parameter,sphere),F(.01))
        lower,upper=correspondence_bound(Curve(circle),Curve(parameter),sphere,(F(0),F(1,10)))
        self.assertGreater(lower,F('.009'))
        self.assertGreaterEqual(upper,lower)


if __name__=='__main__':unittest.main()
