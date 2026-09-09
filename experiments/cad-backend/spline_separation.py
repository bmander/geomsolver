"""Exact separating support planes and complete tensor-spline subdivision."""
from fractions import Fraction as F
from math import gcd

from bernstein import insert
from spline_embedding import polynomial_surface, dot


WORLD = [(1,0,0),(0,1,0),(0,0,1)]


def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])


def direction(v):
    magnitude = max(map(abs,v))
    if magnitude==0:return None
    # Quantization only selects a plane direction. Actual support bounds use
    # exact dot products against every control coefficient, not these secants.
    n = tuple(round(F(x)*65536/magnitude) for x in v)
    divisor = gcd(*n)
    sign = 1 if next(x for x in n if x)>0 else -1
    return tuple(sign*x//divisor for x in n)


class Node:
    def __init__(self,degrees,knots,grid):
        self.degrees,self.knots,self.grid = degrees,knots,grid
        self.cache = {};self.children = None;self.directions = None

    @classmethod
    def surface(cls,surface):return cls(*polynomial_surface(surface))

    def interval(self,axis):
        if axis not in self.cache:
            values = [dot(axis,p) for row in self.grid for p in row]
            self.cache[axis] = min(values),max(values)
        return self.cache[axis]

    def axes(self):
        if self.directions is not None:return self.directions
        grid = self.grid
        a = tuple(y-x for x,y in zip(grid[0][len(grid[0])//2],grid[-1][len(grid[0])//2]))
        b = tuple(y-x for x,y in zip(grid[len(grid)//2][0],grid[len(grid)//2][-1]))
        c = cross(a,b)
        self.directions = [n for v in (c,cross(b,c),cross(c,a)) if (n:=direction(v)) is not None]
        return self.directions

    def extent(self):return max(self.interval(n)[1]-self.interval(n)[0] for n in WORLD)

    def split(self):
        if self.children is not None:return self.children
        extents = []
        for axis in (0,1):
            lines = list(zip(*self.grid)) if axis==0 else self.grid
            extents.append(max(sum(abs(y-x) for x,y in zip(line[0],line[-1])) for line in lines))
        axis = int(extents[1]>extents[0])
        degree,knots = self.degrees[axis],self.knots[axis]
        t = (knots[0]+knots[-1])/2
        lines = list(zip(*self.grid)) if axis==0 else self.grid
        halves = []
        for line in lines:
            poles,k = list(line),knots
            while k.count(t)<degree:poles,k = insert(poles,k,degree,t)
            boundary = max(i for i,x in enumerate(k) if x==t)-degree
            halves.append((poles[:boundary+1],poles[boundary:]))
        left_knots = k[:k.index(t)+degree]+[t]
        right_knots = [t]+k[k.index(t):]
        result = []
        for side,ks in enumerate((left_knots,right_knots)):
            grid = [h[side] for h in halves]
            if axis==0:grid = [list(row) for row in zip(*grid)]
            pair = list(self.knots);pair[axis] = ks
            result.append(Node(self.degrees,pair,grid))
        self.children = result
        return result


def plane_gap(a,b,axes):
    for axis in axes:
        x,y = a.interval(axis),b.interval(axis)
        gap = max(y[0]-x[1],x[0]-y[1])
        if gap>0:return gap/sum(map(abs,axis))
    return None


def separate(a,b,max_cells=1024):
    if max_cells<1:raise ValueError('positive surface-pair budget required')
    axes = list(dict.fromkeys(WORLD+a.axes()+b.axes()))
    pending = [(a,b)];tested=accepted=0;minimum=None
    while pending and tested<max_cells:
        p,q = pending.pop();tested+=1
        gap = plane_gap(p,q,axes)
        if gap is not None:
            accepted+=1;minimum=gap if minimum is None else min(minimum,gap)
            continue
        if p.extent()>=q.extent():pending.extend((child,q) for child in p.split())
        else:pending.extend((p,child) for child in q.split())
    return dict(verified=not pending,tested_cells=tested,accepted_cells=accepted,
                unresolved_cells=len(pending),distance_lower=minimum if not pending else None)
