"""Independent positive lower bounds over a complete roll interval.

Only explicit one-Lipschitz fields supported by functional_sweep are used.
No supplied cell values, speed estimates or primary minimizer state are trusted.
"""
import functional_sweep as f
import math


def speed_bound(p,motion):
    # inverse(M) = inverse(source) observer. For a rotation of rate w about o,
    # radius <= r+2|o| and speed <= s+|w|(r+|o|). Compose the two inequalities.
    # p already encloses the fixed inverse-indexed point.
    r = f.norm(p)
    source,observer = motion['source'],motion['observer']
    a,b = f.norm(f.vec(source['origin'])),f.norm(f.vec(observer['origin']))
    ws,wo = f.point(abs(source['ratio'])),f.point(abs(observer['ratio']))
    return f.add(f.mul(wo,f.add(r,b)),f.mul(ws,f.add(f.add(r,f.mul(f.point(2),b)),a)))


def intervals(domain,cover):
    assert cover and cover['intervals'], 'missing positive cover'
    lower = f.point(cover['lower'])[0]
    assert lower > 0, 'nonpositive cover margin'
    cells = sorted((f.interval(c),c) for c in cover['intervals'])
    end = domain[0]
    for (lo,hi),_ in cells:
        assert domain[0] <= lo < hi <= domain[1], 'cell outside roll domain'
        assert lo <= end, 'uncovered roll interval'
        end = max(end,hi)
    assert end == domain[1], 'uncovered roll endpoint'
    return lower,cells


def check(p,motion,generator,domain,cover):
    lower,cells = intervals(domain,cover)
    speed = speed_bound(p,motion)
    for (lo,hi),raw in cells:
        # Match the exported binary64 cell's center choice, but bound its true
        # distance to both endpoints with exact rationals. There is no assumed
        # allowance for libm or floating-point field evaluation.
        mid = raw[0]*0.5+raw[1]*0.5
        center = f.point(mid)[0]
        assert lo <= center <= hi
        h = max(center-lo,hi-center)
        q = f.rotation(p,motion['observer'],mid)
        q = f.rotation(q,motion['source'],mid,inverse=True)
        value = f.field(q,generator)
        bound = f.sub(value,f.mul(speed,(h,h)))
        assert bound[0] >= lower, ('unproved positive lower bound',raw,float(bound[0]),float(lower))
    return len(cells),lower


def self_checks():
    turn = {'origin':[0,0,0],'axis':[0,0,1],'ratio':1,'phase':0}
    motion = {'source':turn,'observer':dict(turn,ratio=2)}
    sphere = {'origin':[0,0,0],'axis':[0,0,1],
        'profile':{'disk':{'center':[0,0],'radius':1}}}
    p = f.vec([3,0,0])
    domain = f.interval([-0.05,0.05])
    # This sphere's field is constantly 2 on the orbit. The deliberately loose
    # speed bound 9 still proves at least 1.5 over the entire interval.
    assert check(p,motion,sphere,domain,{'lower':1.5,'intervals':[[-0.05,0.05]]}) == (1,f.F(3,2))
    offset = {'source':dict(turn,origin=[2,0,0],ratio=2),
        'observer':dict(turn,origin=[0,3,0],ratio=3)}
    speed = speed_bound(f.vec([3,4,0]),offset)
    assert speed[0] <= 50 <= speed[1] and speed[1]-speed[0] < f.F(1,10**45)
    for cover in [
        {'lower':1,'intervals':[[0,0.5],[math.nextafter(0.5,1),1]]},
        {'lower':1,'intervals':[[0,0.9]]},
        {'lower':1,'intervals':[[-0.1,1]]},
    ]:
        try: intervals(f.interval([0,1]),cover)
        except AssertionError: pass
        else: raise AssertionError('invalid coverage accepted')
    try: check(p,motion,sphere,domain,{'lower':2.1,'intervals':[[-0.05,0.05]]})
    except AssertionError: pass
    else: raise AssertionError('false orbit lower bound accepted')
