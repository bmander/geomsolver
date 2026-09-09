"""Polynomial curve pieces and analytical surface composition with interval jets."""
from fractions import Fraction as F
from math import comb

from bernstein import refine
from interval_jet import ZERO, ONE, atan_point, f, point
from curve_taylor import Taylor as Jet


PI = f.mul(point(4),atan_point(F(1)))


def periodic_trig(value):
    # An approximate integer only selects a reduction; its multiple of pi is
    # enclosed exactly, with the corresponding parity sign restored afterward.
    turns = round(float(sum(value.v)/2)/float(sum(PI)/2))
    sine,cosine = (value-Jet(f.mul(point(turns),PI))).sin_cos()
    return (sine,cosine) if turns%2 == 0 else (-sine,-cosine)


class Curve:
    def __init__(self, record):
        self.record = record
        self.breaks = []
        self.pieces = []
        if record['kind'] == 'bspline':
            if len(record['weights']) != len(record['poles']) or any(w != 1 for w in record['weights']):
                raise ValueError('expected a polynomial edge curve')
            degree = record['degree']
            poles,knots = refine([tuple(F(x) for x in p) for p in record['poles']],
                                 [F(k) for k in record['knots']],degree)
            self.breaks = sorted(set(knots))
            for i,(a,b) in enumerate(zip(self.breaks,self.breaks[1:])):
                differences = poles[i*degree:i*degree+degree+1]
                power = []
                for k in range(degree+1):
                    power.append(tuple(comb(degree,k)*x for x in differences[0]))
                    differences = [tuple(y-x for x,y in zip(p,q)) for p,q in zip(differences,differences[1:])]
                self.pieces.append((a,b,power))
        elif record['kind'] not in ('line','circle'):
            raise ValueError('unsupported edge curve')

    def evaluate(self, t):
        record = self.record
        if record['kind'] == 'line':
            return [Jet(a)+b*t for a,b in zip(record['location'],record['direction'])]
        if record['kind'] == 'circle':
            sine,cosine = periodic_trig(t)
            return [Jet(o)+record['radius']*(x*cosine+y*sine)
                    for o,x,y in zip(record['origin'],*record['basis'])]
        mid = sum(t.v)/2
        index = max(0,min(len(self.pieces)-1,sum(k <= mid for k in self.breaks)-1))
        a,b,power = self.pieces[index]
        if (index > 0 and t.v[0] < a) or (index < len(self.pieces)-1 and t.v[1] > b):
            raise ValueError('curve cell crosses an interior knot')
        if t.c[1:] and t.c[1:] != (ONE,)+(ZERO,)*(len(t.c)-2):
            raise ValueError('curve evaluator requires its original scalar parameter')
        q = f.div(f.sub(t.v,point(a)),point(b-a))
        coordinates = [[] for _ in power[0]]
        for order in range(len(t.c)):
            for axis,values in enumerate(coordinates):
                value = ZERO
                for i in reversed(range(order,len(power))):
                    coefficient = power[i][axis]*comb(i,order)/(b-a)**order
                    value = f.add(f.mul(value,q),point(coefficient))
                values.append(value)
        return [Jet(c[0],c[1:]) for c in coordinates]


def analytical_surface(record, uv):
    u,v = uv
    sine,cosine = periodic_trig(u)
    radial = [x*cosine+y*sine for x,y in zip(*record['basis'][:2])]
    if record['kind'] == 'sphere':
        sv,cv = periodic_trig(v)
        return [Jet(o)+record['radius']*(r*cv+z*sv) for o,r,z in zip(record['origin'],radial,record['basis'][2])]
    if record['kind'] == 'cone':
        sa,ca = Jet(record['angle']).sin_cos()
        return [Jet(o)+(record['radius']+v*sa)*r+v*ca*z for o,r,z in zip(record['origin'],radial,record['basis'][2])]
    raise ValueError('unsupported analytical edge face')


def correspondence_bound(curve, parameter, surface, domain, order=3):
    h = (domain[1]-domain[0])/2
    def difference(t):
        a = curve.evaluate(t)
        b = analytical_surface(surface,parameter.evaluate(t))
        return [x-y for x,y in zip(a,b)]
    center = difference(Jet.variable(sum(domain)/2,order))
    box = difference(Jet.variable(domain,order+1))
    errors = [sum(max(map(abs,c.c[k]))*h**k for k in range(order+1))
              +max(map(abs,b.c[order+1]))*h**(order+1) for c,b in zip(center,box)]
    upper = f.arithmetic.sqrt(point(sum(e*e for e in errors)))[1]
    lower = f.arithmetic.sqrt(point(sum(f.square(c.v)[0] for c in center)))[0]
    return lower,upper


def norm_bound(values):
    return f.arithmetic.sqrt(point(sum(max(map(abs,x.v))**2 for x in values)))[1]


def elementary_bound(curve, parameter, surface):
    """Factor shared line/circle functions before interval subtraction."""
    if parameter['kind'] != 'line':
        return None
    origin,direction = parameter['location'],parameter['direction']
    if curve['kind']=='line' and surface['kind']=='cone' and direction[0]==0:
        t = Jet.variable(0,1)
        a = Curve(curve).evaluate(t)
        b = analytical_surface(surface,Curve(parameter).evaluate(t))
        difference = [x-y for x,y in zip(a,b)]
        extent = max(abs(F(x)) for x in curve['interval'])
        return norm_bound([Jet(x.c[0]) for x in difference])+extent*norm_bound([Jet(x.c[1]) for x in difference])
    varying = [i for i,x in enumerate(direction) if x!=0]
    if curve['kind']!='circle' or len(varying)!=1 or direction[varying[0]] not in (-1,1):
        return None
    axis, = varying
    x,y,z = surface['basis']
    offset = [Jet(v) for v in surface['origin']]
    if surface['kind']=='sphere' and axis==1:
        sine,cosine = periodic_trig(Jet(origin[0]))
        cosine_axis = [surface['radius']*(a*cosine+b*sine) for a,b in zip(x,y)]
        sine_axis = [Jet(surface['radius'])*a for a in z]
    else:
        if axis!=0:return None
        if surface['kind']=='sphere':
            sine,cosine = periodic_trig(Jet(origin[1]))
            radius = surface['radius']*cosine
            height = surface['radius']*sine
        else:
            sine,cosine = Jet(surface['angle']).sin_cos()
            radius = surface['radius']+origin[1]*sine
            height = origin[1]*cosine
        offset = [o+height*a for o,a in zip(offset,z)]
        cosine_axis,sine_axis = [[radius*a for a in d] for d in (x,y)]
    sine,cosine = periodic_trig(Jet(origin[axis]))
    c = [a*cosine+b*sine for a,b in zip(cosine_axis,sine_axis)]
    s = [direction[axis]*(-a*sine+b*cosine) for a,b in zip(cosine_axis,sine_axis)]
    return (norm_bound([Jet(a)-b for a,b in zip(curve['origin'],offset)])
            +norm_bound([Jet(curve['radius'])*a-b for a,b in zip(curve['basis'][0],c)])
            +norm_bound([Jet(curve['radius'])*a-b for a,b in zip(curve['basis'][1],s)]))
