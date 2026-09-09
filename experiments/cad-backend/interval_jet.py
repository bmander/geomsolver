"""Two-variable jets through third order with outward rational interval arithmetic."""
from fractions import Fraction as F
from functools import lru_cache
from math import factorial
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"rust/gcs-core/tests/verification"))
import crown_branches as arithmetic


class Arithmetic:
    """Outward dyadic rounding for this spatial-error audit, separate from sweep proofs."""
    arithmetic = arithmetic
    scale = 1 << 80

    @classmethod
    def rounded(cls, value):
        def floor(x):
            return x.numerator*cls.scale//x.denominator
        return F(floor(value[0]), cls.scale), F(-floor(-value[1]), cls.scale)

    @classmethod
    def add(cls, a, b): return cls.rounded(arithmetic.add(a, b))
    @classmethod
    def sub(cls, a, b): return cls.rounded(arithmetic.sub(a, b))
    @classmethod
    def mul(cls, a, b): return cls.rounded(arithmetic.mul(a, b))
    @classmethod
    def div(cls, a, b): return cls.rounded(arithmetic.div(a, b))
    @classmethod
    def square(cls, a): return cls.rounded(arithmetic.square(a))
    @staticmethod
    def neg(a): return arithmetic.neg(a)


f = Arithmetic


def point(x):
    x = F(x)
    return x, x


ZERO, ONE = point(0), point(1)
THIRD_TERMS = (((0, 0, 3),), ((0, 1, 1), (1, 0, 2)),
               ((2, 0, 1), (1, 1, 2)), ((2, 1, 3),))


@lru_cache(2048)
def trig_point(x):
    if abs(x) > 4:
        raise ValueError("trigonometric proof range exceeded")
    xx = point(-x*x)
    outputs = []
    for odd in (0, 1):
        value = point(F(1, factorial(24+odd)))
        for k in range(11, -1, -1):
            value = f.add(point(F(1, factorial(2*k+odd))), f.mul(xx, value))
        if odd:
            value = f.mul(point(x), value)
        remainder = abs(x)**(25+odd)/factorial(25+odd)
        outputs.append(f.add(value, (-remainder, remainder)))
    return outputs[1], outputs[0]


def sin_cos(value):
    lo, hi = value
    mid, h = (lo+hi)/2, (hi-lo)/2
    if max(abs(lo), abs(hi)) > 4:
        raise ValueError("trigonometric proof range exceeded")
    sine, cosine = trig_point(mid)
    # cos(delta) >= 1-delta²/2 and |sin(delta)| <= |delta| for real delta.
    ch = (max(F(-1), 1-h*h/2), F(1))
    sh = (-min(F(1), h), min(F(1), h))
    return f.add(f.mul(sine, ch), f.mul(cosine, sh)), f.sub(f.mul(cosine, ch), f.mul(sine, sh))


@lru_cache(2048)
def atan_point(x):
    if abs(x) > 2:
        raise ValueError("arctangent proof range exceeded")
    # atan(x)=2 atan(x/(1+sqrt(1+x²))); the reduced argument has |q| < .62.
    q = f.div(point(x), f.add(ONE, f.arithmetic.sqrt(point(1+x*x))))
    q2 = f.neg(f.square(q))
    value = point(F(1, 41))
    for k in range(19, -1, -1):
        value = f.add(point(F(1, 2*k+1)), f.mul(q2, value))
    bound = max(abs(v) for v in q)**43/43
    return f.mul(point(2), f.add(f.mul(q, value), (-bound, bound)))


def atan(value):
    lo, hi = value
    if max(abs(lo), abs(hi)) > 2:
        raise ValueError("arctangent proof range exceeded")
    mid, h = (lo+hi)/2, (hi-lo)/2
    radius = h/(1+f.square(value)[0])
    return f.add(atan_point(mid), (-radius, radius))


class Jet:
    def __init__(self, value, d=(ZERO, ZERO), h=(ZERO, ZERO, ZERO), t=None):
        self.v = tuple(map(F, value)) if isinstance(value, (tuple, list)) else point(value)
        if len(self.v) != 2 or self.v[0] > self.v[1]:
            raise ValueError("invalid interval endpoints")
        self.d, self.h = d, h  # h stores uu, uv, vv.
        self.t = t  # Optional uuu, uuv, uvv, vvv.

    def constant(self):
        return self.d == (ZERO, ZERO) and self.h == (ZERO, ZERO, ZERO) and (self.t is None or self.t == (ZERO,)*4)

    @staticmethod
    def variable(value, axis, order=2):
        if order not in (2, 3):
            raise ValueError("unsupported jet order")
        return Jet(value, tuple(ONE if i == axis else ZERO for i in range(2)), t=(ZERO,)*4 if order == 3 else None)

    def third(self):
        if self.t is None and not self.constant():
            raise ValueError("cannot promote unknown third derivatives")
        return self.t or (ZERO,)*4

    def __add__(self, other):
        other = other if isinstance(other, Jet) else Jet(other)
        if other.constant():
            return Jet(f.add(self.v, other.v), self.d, self.h, self.t)
        if self.constant():
            return other+self
        t = tuple(f.add(a, b) for a, b in zip(self.third(), other.third())) if self.t is not None or other.t is not None else None
        return Jet(f.add(self.v, other.v), tuple(f.add(a, b) for a, b in zip(self.d, other.d)),
                   tuple(f.add(a, b) for a, b in zip(self.h, other.h)), t)

    __radd__ = __add__

    def __neg__(self):
        return Jet(f.neg(self.v), tuple(map(f.neg, self.d)), tuple(map(f.neg, self.h)),
                   tuple(map(f.neg, self.t)) if self.t is not None else None)

    def __sub__(self, other):
        return self + -(other if isinstance(other, Jet) else Jet(other))

    def __rsub__(self, other):
        return -self+other

    def __mul__(self, other):
        b = other if isinstance(other, Jet) else Jet(other)
        if b.constant():
            if self.constant():
                return Jet(f.mul(self.v, b.v))
            return Jet(f.mul(self.v, b.v), tuple(f.mul(x, b.v) for x in self.d),
                       tuple(f.mul(x, b.v) for x in self.h),
                       tuple(f.mul(x, b.v) for x in self.t) if self.t is not None else None)
        if self.constant():
            return b*self
        d = tuple(f.add(f.mul(x, b.v), f.mul(self.v, y)) for x, y in zip(self.d, b.d))
        h = tuple(f.add(f.add(f.mul(self.h[k], b.v), f.mul(self.v, b.h[k])),
                        f.add(f.mul(self.d[i], b.d[j]), f.mul(self.d[j], b.d[i])))
                  for k, (i, j) in enumerate(((0, 0), (0, 1), (1, 1))))
        t = None
        if self.t is not None or b.t is not None:
            at, bt = self.third(), b.third()
            # The middle terms in the multivariate Leibniz rule.
            values = []
            for k, pairs in enumerate(THIRD_TERMS):
                value = f.add(f.mul(at[k], b.v), f.mul(self.v, bt[k]))
                for hi, di, weight in pairs:
                    value = f.add(value, f.mul(point(weight), f.add(
                        f.mul(self.h[hi], b.d[di]), f.mul(self.d[di], b.h[hi]))))
                values.append(value)
            t = tuple(values)
        return Jet(f.mul(self.v, b.v), d, h, t)

    __rmul__ = __mul__

    def compose(self, value, first, second, third=None):
        if self.constant():
            return Jet(value)
        t = None
        if self.t is not None:
            if third is None:
                raise ValueError("missing third derivative of composition")
            values = []
            for k, pairs in enumerate(THIRD_TERMS):
                mixed = ZERO
                for hi, di, weight in pairs:
                    mixed = f.add(mixed, f.mul(point(weight), f.mul(self.h[hi], self.d[di])))
                cube = ONE
                for i in (0,)*(3-k)+(1,)*k:
                    cube = f.mul(cube, self.d[i])
                values.append(f.add(f.mul(first, self.t[k]), f.add(f.mul(second, mixed), f.mul(third, cube))))
            t = tuple(values)
        return Jet(value, tuple(f.mul(first, x) for x in self.d),
                   tuple(f.add(f.mul(first, self.h[k]), f.mul(second,
                       f.square(self.d[i]) if i == j else f.mul(self.d[i], self.d[j])))
                       for k, (i, j) in enumerate(((0, 0), (0, 1), (1, 1)))), t)

    def reciprocal(self):
        inverse = f.div(ONE, self.v)
        if self.constant():
            return Jet(inverse)
        return self.compose(inverse, f.neg(f.square(inverse)), f.mul(point(2), f.mul(f.square(inverse), inverse)),
                            f.mul(point(-6), f.square(f.square(inverse))) if self.t is not None else None)

    def __truediv__(self, other):
        return self*(other if isinstance(other, Jet) else Jet(other)).reciprocal()

    def sqrt(self):
        root = f.arithmetic.sqrt(self.v)
        if self.constant():
            return Jet(root)
        return self.compose(root, f.div(ONE, f.mul(point(2), root)),
                            f.neg(f.div(ONE, f.mul(point(4), f.mul(self.v, root)))),
                            f.div(point(3), f.mul(point(8), f.mul(f.square(self.v), root))) if self.t is not None else None)

    def sin_cos(self):
        sine, cosine = sin_cos(self.v)
        return self.compose(sine, cosine, f.neg(sine), f.neg(cosine)), self.compose(cosine, f.neg(sine), f.neg(cosine), sine)

    def atan(self):
        first = f.div(ONE, f.add(ONE, f.square(self.v)))
        third = f.mul(f.sub(f.mul(point(6), f.square(self.v)), point(2)),
                      f.mul(f.square(first), first)) if self.t is not None else None
        return self.compose(atan(self.v), first, f.mul(point(-2), f.mul(self.v, f.square(first))), third)
