"""Short univariate Taylor jets using the shared outward interval arithmetic."""
from fractions import Fraction as F

from interval_jet import ZERO, ONE, f, point, sin_cos


class Taylor:
    def __init__(self,value,coefficients=()):
        self.c = (tuple(map(F,value)) if isinstance(value,(tuple,list)) else point(value),)+tuple(coefficients)
        if any(len(v)!=2 or v[0]>v[1] for v in self.c):
            raise ValueError('invalid Taylor coefficient interval')

    @property
    def v(self): return self.c[0]

    @staticmethod
    def variable(value,order=3):
        if type(order) is not int or order<1:
            raise ValueError('invalid Taylor order')
        return Taylor(value,(ONE,)+(ZERO,)*(order-1))

    def __add__(self,other):
        other = other if isinstance(other,Taylor) else Taylor(other)
        if len(self.c)>1 and len(other.c)>1 and len(self.c)!=len(other.c):
            raise ValueError('cannot invent higher Taylor coefficients')
        n = max(len(self.c),len(other.c))
        a,b = self.c+(ZERO,)*(n-len(self.c)),other.c+(ZERO,)*(n-len(other.c))
        c = [f.add(x,y) for x,y in zip(a,b)]
        return Taylor(c[0],c[1:])

    __radd__ = __add__

    def __neg__(self):
        c = [f.neg(x) for x in self.c]
        return Taylor(c[0],c[1:])

    def __sub__(self,other):
        return self+-(other if isinstance(other,Taylor) else Taylor(other))

    def __rsub__(self,other): return -self+other

    def __mul__(self,other):
        other = other if isinstance(other,Taylor) else Taylor(other)
        if len(self.c)>1 and len(other.c)>1 and len(self.c)!=len(other.c):
            raise ValueError('cannot invent higher Taylor coefficients')
        n = max(len(self.c),len(other.c))
        c = [ZERO]*n
        for i,a in enumerate(self.c):
            if a == ZERO: continue
            for j,b in enumerate(other.c[:n-i]):
                if b != ZERO:c[i+j] = f.add(c[i+j],f.mul(a,b))
        return Taylor(c[0],c[1:])

    __rmul__ = __mul__

    def __truediv__(self,constant):
        return self*Taylor(f.div(ONE,point(constant)))

    def sin_cos(self):
        s,c = sin_cos(self.v)
        sine,cosine = [s],[c]
        # s'=c*x', c'=-s*x'; coefficients store derivative / factorial.
        for k in range(1,len(self.c)):
            a,b = ZERO,ZERO
            for j in range(1,k+1):
                term = f.mul(point(j),self.c[j])
                a = f.add(a,f.mul(term,cosine[k-j]))
                b = f.sub(b,f.mul(term,sine[k-j]))
            sine.append(f.div(a,point(k)));cosine.append(f.div(b,point(k)))
        return Taylor(sine[0],sine[1:]),Taylor(cosine[0],cosine[1:])
