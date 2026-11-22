# A slab under face #27487 of a STEP file: the face, the same face moved by -h d (turned), and
# six walls, each its top edge's curve extruded along -h d.
import re, sys, math
src, out = sys.argv[1], sys.argv[2]
h = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
text = open(src).read()
data = text[text.index('DATA;')+5:text.index('ENDSEC;', text.index('DATA;'))]
ents = {}
for m in re.finditer(r'#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|$)', data, re.S):
    ents[int(m.group(1))] = ' '.join(m.group(2).split())
def refs(s): return [int(x) for x in re.findall(r'#(\d+)', s)]
def closure(root):
    seen, st = set(), [root]
    while st:
        x = st.pop()
        if x in seen: continue
        seen.add(x); st.extend(refs(ents[x]))
    return seen
FACE = 27487
keep = closure(FACE)
nums = lambda s: [float(x) for x in re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', s)]
def point(i): return nums(ents[i][ents[i].index('('):])[-3:]
# the direction: the net's poles' corners
surf = refs(ents[FACE])[-1]
net_rows = re.findall(r'\(([#\d,\s]+)\)', ents[surf])
grid = [refs(r) for r in net_rows if len(refs(r)) > 2]
P = [[point(i) for i in row] for row in grid]
def sub(a,b): return [a[k]-b[k] for k in range(3)]
def cross(a,b): return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
mi, mj = len(P)//2, len(P[0])//2
n = cross(sub(P[-1][mj],P[0][mj]), sub(P[mi][-1],P[mi][0]))
l = math.sqrt(sum(x*x for x in n)); d = [x/l for x in n]
t = [-h*x for x in d]
nxt = [max(ents)+1]
newents = {}
def new(s):
    i = nxt[0]; nxt[0] += 1; newents[i] = s; return i
# the moved copy of an entity's closure
moved = {}
def move(i):
    if i in moved: return moved[i]
    s = ents[i]
    if s.startswith('CARTESIAN_POINT'):
        p = point(i); j = new("CARTESIAN_POINT('',(%r,%r,%r))" % tuple(p[k]+t[k] for k in range(3)))
    else:
        j = new(None)
        s2 = re.sub(r'#(\d+)', lambda m: '#%d' % move(int(m.group(1))), s)
        newents[j] = s2
    moved[i] = j; return j
# the top face's loop uses
face = ents[FACE]
bound = refs(face)[0]; loop = refs(ents[bound])[0]
uses = []
for oe in refs(ents[loop]):
    e = refs(ents[oe])[-1]; o = ents[oe].strip().endswith('.T.)')
    uses.append((e, o))
bottom_face = move(FACE)
# turned: its sense and its bound's orientation flipped
flip = lambda s: s[:-4] + ('.F.)' if s.endswith('.T.)') else '.T.)')
newents[bottom_face] = flip(newents[bottom_face])
bb = refs(newents[bottom_face])[0]; newents[bb] = flip(newents[bb])
vec = new("VECTOR('',#%d,%r)" % (new("DIRECTION('',(%r,%r,%r))" % tuple(-x for x in d)), h))
def vertical(v):
    p = point(refs(ents[v])[0])
    line = new("LINE('',#%d,#%d)" % (new("CARTESIAN_POINT('',(%r,%r,%r))" % tuple(p)), vec))
    return new("EDGE_CURVE('',#%d,#%d,#%d,.T.)" % (v, move(v), line))
verticals = {}
def vert(v):
    if v not in verticals: verticals[v] = vertical(v)
    return verticals[v]
walls = []
for e, o in uses:
    a, b, curve, same = refs(ents[e])[0], refs(ents[e])[1], refs(ents[e])[2], ents[e].strip().endswith('.T.)')
    P0, Q0 = (a, b) if o else (b, a)   # the top's traversal P0 -> Q0
    wall = new("SURFACE_OF_LINEAR_EXTRUSION('',#%d,#%d)" % (curve, vec))
    oes = [new("ORIENTED_EDGE('',*,*,#%d,%s)" % (e, '.F.' if o else '.T.')),
           new("ORIENTED_EDGE('',*,*,#%d,.T.)" % vert(P0)),
           new("ORIENTED_EDGE('',*,*,#%d,%s)" % (move(e), '.T.' if o else '.F.')),
           new("ORIENTED_EDGE('',*,*,#%d,.F.)" % vert(Q0))]
    lp = new("EDGE_LOOP('',(%s))" % ','.join('#%d' % x for x in oes))
    fb = new("FACE_OUTER_BOUND('',#%d,.T.)" % lp)
    forward = (not o) == same
    walls.append(new("ADVANCED_FACE('',(#%d),#%d,%s)" % (fb, wall, '.T.' if forward else '.F.')))
shell = new("CLOSED_SHELL('',(%s))" % ','.join('#%d' % x for x in [FACE, bottom_face] + walls))
solid = new("MANIFOLD_SOLID_BREP('',#%d)" % shell)
body = []
for i in sorted(keep): body.append('#%d = %s;' % (i, ents[i]))
for i in sorted(newents): body.append('#%d = %s;' % (i, newents[i]))
head = ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n"
  "#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n#2 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n"
  "#3 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#1,'distance_accuracy_value','');\n"
  "#4 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#3)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2)) REPRESENTATION_CONTEXT('','') );\n")
tail = "#5 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#%d),#4);\nENDSEC;\nEND-ISO-10303-21;\n" % solid
open(out, 'w').write(head + '\n'.join(body) + '\n' + tail)
print('d', d, 'entities', len(body))
