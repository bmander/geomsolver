//! **The curves and surfaces a STEP file may hold, read as the kernel holds the same points**
//! (`brep::step_read::geometry`; #993 stage 1): each kind the standard allows read to the file's
//! own set of points exactly, in a form the kernel has — or refused naming the entity, never read
//! wrong. Every file here is written by hand, a solid whose volume and area are closed forms of
//! its numbers (or a one-dimensional integral of them, summed to the last bits), and read to them.
use gcs_core::brep::props::{area, volume};
use gcs_core::brep::step_read::{read, Refusal, Why};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

type V = [f64; 3];

fn close(what: &str, a: f64, b: f64, rel: f64) {
    assert!((a - b).abs() <= rel * b.abs(), "{what}: {a} read, {b} the closed form ({:e})", (a - b) / b.abs());
}

/// A real as a STEP file writes it.
fn real(x: f64) -> String { format!("{x:.17E}") }

fn reals(x: &[f64]) -> String { x.iter().map(|&x| real(x)).collect::<Vec<_>>().join(",") }

fn add(a: V, b: V) -> V { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
fn sub(a: V, b: V) -> V { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
fn scale(a: V, s: f64) -> V { [a[0] * s, a[1] * s, a[2] * s] }
fn dot(a: V, b: V) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
fn cross(a: V, b: V) -> V { [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]] }
fn norm(a: V) -> f64 { dot(a, a).sqrt() }
fn unit(a: V) -> V { scale(a, 1. / norm(a)) }
/// `p` turned by `u` about the `z` axis.
fn turn(p: V, u: f64) -> V { [p[0] * u.cos() - p[1] * u.sin(), p[0] * u.sin() + p[1] * u.cos(), p[2]] }

/// A file's entities, numbered from 100 as they are added; the solid `#51`, its shell `#50`.
struct Ents { next: u32, text: String }

impl Ents {
    fn new() -> Ents { Ents { next: 100, text: String::new() } }
    fn add(&mut self, s: impl AsRef<str>) -> u32 {
        let n = self.next;
        self.next += 1;
        self.text += &format!("#{n} = {};\n", s.as_ref());
        n
    }
    fn point(&mut self, p: V) -> u32 { self.add(format!("CARTESIAN_POINT('',({}))", reals(&p))) }
    fn flat_point(&mut self, p: [f64; 2]) -> u32 { self.add(format!("CARTESIAN_POINT('',({}))", reals(&p))) }
    fn dir(&mut self, d: V) -> u32 { self.add(format!("DIRECTION('',({}))", reals(&d))) }
    fn place(&mut self, o: V, z: V, x: V) -> u32 {
        let (o, z, x) = (self.point(o), self.dir(z), self.dir(x));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})"))
    }
    fn vertex(&mut self, p: V) -> u32 { let p = self.point(p); self.add(format!("VERTEX_POINT('',#{p})")) }
    fn line(&mut self, p: V, d: V, m: f64) -> u32 {
        let (p, d) = (self.point(p), self.dir(d));
        let v = self.add(format!("VECTOR('',#{d},{})", real(m)));
        self.add(format!("LINE('',#{p},#{v})"))
    }
    fn plane(&mut self, o: V, z: V, x: V) -> u32 { let a = self.place(o, z, x); self.add(format!("PLANE('',#{a})")) }
    fn circle(&mut self, o: V, z: V, x: V, r: f64) -> u32 { let a = self.place(o, z, x); self.add(format!("CIRCLE('',#{a},{})", real(r))) }
    fn edge(&mut self, a: u32, b: u32, curve: u32, sense: bool) -> u32 { self.add(format!("EDGE_CURVE('',#{a},#{b},#{curve},{})", logical(sense))) }
    /// A face of a loop of `(edge, along)`, its bound `turned` or not.
    fn face(&mut self, uses: &[(u32, bool)], turned: bool, surface: u32, sense: bool) -> u32 {
        let uses: Vec<String> = uses.iter().map(|&(e, along)| format!("#{}", self.add(format!("ORIENTED_EDGE('',*,*,#{e},{})", logical(along))))).collect();
        let l = self.add(format!("EDGE_LOOP('',({}))", uses.join(",")));
        let b = self.add(format!("FACE_BOUND('',#{l},{})", logical(!turned)));
        self.add(format!("ADVANCED_FACE('',(#{b}),#{surface},{})", logical(sense)))
    }
    /// The file of the solid bounded by `faces`.
    fn file(self, faces: &[u32]) -> String {
        let faces: Vec<String> = faces.iter().map(|f| format!("#{f}")).collect();
        format!("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('','',(''),(''),'','','');\n\
            FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n\
            #1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
            #2 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n\
            #3 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#1,'distance_accuracy_value','');\n\
            #4 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#3)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2)) REPRESENTATION_CONTEXT('','') );\n\
            #10 = CARTESIAN_POINT('',(0.,0.,0.));\n#11 = DIRECTION('',(0.,0.,1.));\n#12 = DIRECTION('',(1.,0.,0.));\n\
            #13 = AXIS2_PLACEMENT_3D('',#10,#11,#12);\n{}#50 = CLOSED_SHELL('',({}));\n#51 = MANIFOLD_SOLID_BREP('',#50);\n\
            #52 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#13,#51),#4);\nENDSEC;\nEND-ISO-10303-21;\n", self.text, faces.join(","))
    }
}

fn logical(b: bool) -> &'static str { if b { ".T." } else { ".F." } }

/// The solid a file reads to: its volume and area.
fn measures(text: &str) -> Result<(f64, f64), Refusal> {
    let given = read(text)?;
    assert_eq!(given.len(), 1);
    let b = &given[0].brep;
    Ok((volume(b), b.faces.iter().map(|f| area(b, f)).sum()))
}

/// A file read to its closed forms, to `rel`.
fn reads(what: &str, text: &str, v: f64, a: f64, rel: f64) {
    let (rv, ra) = measures(text).unwrap_or_else(|r| panic!("{what}: refused {r:?}"));
    close(&format!("{what}: volume"), rv, v, rel);
    close(&format!("{what}: area"), ra, a, rel);
}

/// A file refused, why and naming which entity.
fn refused(what: &str, text: &str, why: Why, entity: u32) {
    let r = measures(text).map(|_| ()).expect_err(what);
    assert_eq!((r.why, r.entity), (why, Some(entity)), "{what}: {r}");
}

/// `∫ f` over `[a, b]` by Simpson's rule on `n` (even) panels: a smooth integrand's to its last bits.
fn integral(f: &dyn Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let h = (b - a) / n as f64;
    let mut s = f(a) + f(b);
    for k in 1..n { s += f(a + k as f64 * h) * if k % 2 == 1 { 4. } else { 2. }; }
    s * h / 3.
}

/// The edges and faces of the box `[0, a] × [0, b] × [0, c]`: the curve of edge `k` from its
/// first corner to its last as `curve` makes it (its entity and whether it runs along the edge),
/// a line where it makes none; the surface of face `k` as `surface` makes it (its entity and
/// whether its normal points out), a plane where it makes none. Edges `0` to `3` run along `x`
/// (edge 0 from the origin to `(a, 0, 0)`), faces `2d + s` are square to axis `d`, at its `s` end
/// (face 5 the top, `z = c`).
fn boxed(size: V, curve: &dyn Fn(&mut Ents, usize, V, V) -> Option<(u32, bool)>,
    surface: &dyn Fn(&mut Ents, usize, V, V) -> Option<(u32, bool)>) -> String {
    let mut e = Ents::new();
    let corner = |i: usize| -> V { std::array::from_fn(|d| if i >> d & 1 == 1 { size[d] } else { 0. }) };
    let vertices: Vec<u32> = (0..8).map(|i| e.vertex(corner(i))).collect();
    let mut edges = Vec::new();
    for d in 0..3 {
        for c in (0..8).filter(|c| c >> d & 1 == 0) {
            let (a, b) = (corner(c), corner(c | 1 << d));
            let (g, sense) = curve(&mut e, edges.len(), a, b).unwrap_or_else(|| (e.line(a, unit(sub(b, a)), 1.), true));
            edges.push((c, c | 1 << d, e.edge(vertices[c], vertices[c | 1 << d], g, sense)));
        }
    }
    let mut faces = Vec::new();
    for d in 0..3 {
        for s in 0..2 {
            let (d1, d2) = ((d + 1) % 3, (d + 2) % 3);
            let mut ring: Vec<usize> = [(0, 0), (1, 0), (1, 1), (0, 1)].iter().map(|&(i, j)| s << d | i << d1 | j << d2).collect();
            if s == 0 { ring.reverse(); }
            let uses: Vec<(u32, bool)> = (0..4).map(|k| {
                let (p, q) = (ring[k], ring[(k + 1) % 4]);
                let &(from, _, id) = edges.iter().find(|&&(a, b, _)| (a, b) == (p, q) || (a, b) == (q, p)).unwrap();
                (id, from == p)
            }).collect();
            let mut n = [0.; 3];
            n[d] = if s == 1 { 1. } else { -1. };
            let mut o = scale(size, 0.5);
            o[d] = if s == 1 { size[d] } else { 0. };
            let mut x = [0.; 3];
            x[d1] = 1.;
            let (g, sense) = surface(&mut e, 2 * d + s, o, n).unwrap_or_else(|| (e.plane(o, n, x), true));
            faces.push(e.face(&uses, false, g, sense));
        }
    }
    e.file(&faces)
}

const SIZE: V = [3., 4., 5.];

fn box_volume() -> f64 { SIZE[0] * SIZE[1] * SIZE[2] }
fn box_area() -> f64 { 2. * (SIZE[0] * SIZE[1] + SIZE[1] * SIZE[2] + SIZE[2] * SIZE[0]) }

/// The box with its edge 0's curve made by `curve` (its entity, whether along the edge).
fn box_edge(curve: &dyn Fn(&mut Ents, V, V) -> (u32, bool)) -> String {
    boxed(SIZE, &|e, k, a, b| (k == 0).then(|| curve(e, a, b)), &|_, _, _, _| None)
}

/// The box with its top's surface made by `surface`.
fn box_top(surface: &dyn Fn(&mut Ents, V, V) -> (u32, bool)) -> String {
    boxed(SIZE, &|_, _, _, _| None, &|e, k, o, n| (k == 5).then(|| surface(e, o, n)))
}

/// Points at `xi` along the segment from `a` to `b`, `xi` its fractions.
fn along(e: &mut Ents, a: V, b: V, xi: &[f64]) -> String {
    xi.iter().map(|&x| format!("#{}", e.point(add(a, scale(sub(b, a), x))))).collect::<Vec<_>>().join(",")
}

/// **A B-spline is read whatever says its knots**: written (clamped or not), implied by its kind
/// — uniform (`−p, …, n`), quasi-uniform, piecewise Bézier — or the complex entity of either with
/// its weights; one whose knots are not clamped clamped over its domain by inserting them, which is
/// exact. Each a straight edge of a box as a spline whose poles are its Greville points (a spline
/// reproduces a line from them), the box read to its volume and area; its `closed_curve` the
/// writer's word, read as nothing (`.T.` on an open one). A bare `B_SPLINE_CURVE` (an abstract
/// supertype) and a Bézier spline of no whole number of pieces refused as malformed.
#[test]
fn a_b_spline_reads_whatever_says_its_knots() {
    // degree 2, five poles: uniform knots −2…5 (domain [0, 3], Greville i − ½), quasi-uniform
    // 0,0,0,1,2,3,3,3, Bézier 0,0,0,1,1,2,2,2
    let uniform = [-0.5 / 3., 0.5 / 3., 1.5 / 3., 2.5 / 3., 3.5 / 3.];
    let quasi = [0., 0.5 / 3., 1.5 / 3., 2.5 / 3., 1.];
    let bezier = [0., 0.25, 0.5, 0.75, 1.];
    let cases: Vec<(&str, Box<dyn Fn(&mut Ents, V, V) -> (u32, bool)>)> = vec![
        ("unclamped knots, said closed", Box::new(move |e, a, b| { let p = along(e, a, b, &uniform);
            (e.add(format!("B_SPLINE_CURVE_WITH_KNOTS('',2,({p}),.POLYLINE_FORM.,.T.,.F.,(1,1,1,1,1,1,1,1),(-2.,-1.,0.,1.,2.,3.,4.,5.),.UNIFORM_KNOTS.)")), true) })),
        ("a uniform curve", Box::new(move |e, a, b| { let p = along(e, a, b, &uniform);
            (e.add(format!("UNIFORM_CURVE('',2,({p}),.UNSPECIFIED.,.F.,.F.)")), true) })),
        ("a quasi-uniform curve", Box::new(move |e, a, b| { let p = along(e, a, b, &quasi);
            (e.add(format!("QUASI_UNIFORM_CURVE('',2,({p}),.UNSPECIFIED.,.F.,.F.)")), true) })),
        ("a Bézier curve", Box::new(move |e, a, b| { let p = along(e, a, b, &bezier);
            (e.add(format!("BEZIER_CURVE('',2,({p}),.UNSPECIFIED.,.F.,.F.)")), true) })),
        ("a rational quasi-uniform curve", Box::new(move |e, a, b| { let p = along(e, a, b, &quasi);
            (e.add(format!("( BOUNDED_CURVE() B_SPLINE_CURVE(2,({p}),.UNSPECIFIED.,.F.,.F.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() \
                QUASI_UNIFORM_CURVE() RATIONAL_B_SPLINE_CURVE((1.,2.,3.,2.,1.)) REPRESENTATION_ITEM('') )")), true) })),
        ("a rational curve of unclamped knots", Box::new(move |e, a, b| { let p = along(e, a, b, &uniform);
            (e.add(format!("( BOUNDED_CURVE() B_SPLINE_CURVE(2,({p}),.UNSPECIFIED.,.T.,.F.) B_SPLINE_CURVE_WITH_KNOTS((1,1,1,1,1,1,1,1),\
                (-2.,-1.,0.,1.,2.,3.,4.,5.),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((2.,2.,2.,2.,2.)) \
                REPRESENTATION_ITEM('') )")), true) })),
        ("a polyline", Box::new(|e, a, b| { let p = along(e, a, b, &[0., 0.3, 1.]); (e.add(format!("POLYLINE('',({p}))")), true) })),
    ];
    for (what, curve) in &cases { reads(what, &box_edge(&**curve), box_volume(), box_area(), 1e-12); }
    // the same as surfaces: the box's top a sheet of its kind, its poles the Greville points of a
    // plane past the top, `u` along `x` and `v` along `y` (its normal up)
    let grid = |e: &mut Ents, xi: &[f64]| -> String {
        xi.iter().map(|&s| {
            let row: Vec<String> = xi.iter().map(|&t| format!("#{}", e.point([-1. + 5. * s, -1. + 6. * t, SIZE[2]]))).collect();
            format!("({})", row.join(","))
        }).collect::<Vec<_>>().join(",")
    };
    let sheets: Vec<(&str, Box<dyn Fn(&mut Ents, V, V) -> (u32, bool)>)> = vec![
        ("unclamped knots, said closed both ways", Box::new(move |e, _, _| { let p = grid(e, &uniform);
            (e.add(format!("B_SPLINE_SURFACE_WITH_KNOTS('',2,2,({p}),.PLANE_SURF.,.T.,.T.,.F.,(1,1,1,1,1,1,1,1),(1,1,1,1,1,1,1,1),\
                (-2.,-1.,0.,1.,2.,3.,4.,5.),(-2.,-1.,0.,1.,2.,3.,4.,5.),.UNIFORM_KNOTS.)")), true) })),
        ("a uniform surface", Box::new(move |e, _, _| { let p = grid(e, &uniform);
            (e.add(format!("UNIFORM_SURFACE('',2,2,({p}),.UNSPECIFIED.,.F.,.F.,.F.)")), true) })),
        ("a quasi-uniform surface", Box::new(move |e, _, _| { let p = grid(e, &quasi);
            (e.add(format!("QUASI_UNIFORM_SURFACE('',2,2,({p}),.UNSPECIFIED.,.F.,.F.,.F.)")), true) })),
        ("a Bézier surface", Box::new(move |e, _, _| { let p = grid(e, &bezier);
            (e.add(format!("BEZIER_SURFACE('',2,2,({p}),.UNSPECIFIED.,.F.,.F.,.F.)")), true) })),
        ("a rational uniform surface", Box::new(move |e, _, _| { let p = grid(e, &uniform);
            let w = format!("({})", vec!["(3.,3.,3.,3.,3.)"; 5].join(","));
            (e.add(format!("( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,2,({p}),.UNSPECIFIED.,.F.,.F.,.F.) GEOMETRIC_REPRESENTATION_ITEM() \
                RATIONAL_B_SPLINE_SURFACE({w}) REPRESENTATION_ITEM('') SURFACE() UNIFORM_SURFACE() )")), true) })),
    ];
    for (what, sheet) in &sheets { reads(what, &box_top(&**sheet), box_volume(), box_area(), 1e-12); }
    // a B-spline of no kind of knots, and a Bézier one of no whole number of pieces
    let bare = box_edge(&|e, a, b| { let p = along(e, a, b, &quasi); (e.add(format!("B_SPLINE_CURVE('',2,({p}),.UNSPECIFIED.,.F.,.F.)")), true) });
    refused("a bare B_SPLINE_CURVE", &bare, Why::Malformed, first(&bare, "B_SPLINE_CURVE"));
    let torn = box_edge(&|e, a, b| { let p = along(e, a, b, &[0., 0.3, 0.6, 1.]); (e.add(format!("BEZIER_CURVE('',2,({p}),.UNSPECIFIED.,.F.,.F.)")), true) });
    refused("a Bézier curve of a piece and a half", &torn, Why::Malformed, first(&torn, "BEZIER_CURVE"));
}

/// The box's top a plane wrapped by `f` (given the plane's entity, the wrapping entity's text).
fn wrapped(f: impl Fn(&mut Ents, u32) -> String) -> impl Fn(&mut Ents, V, V) -> (u32, bool) {
    move |e, o, n| {
        let p = e.plane(o, n, [1., 0., 0.]);
        let s = f(e, p);
        (e.add(s), true)
    }
}

/// The number of the first entity of `kind` in a file.
fn first(text: &str, kind: &str) -> u32 {
    let line = text.lines().find(|l| l.split(" = ").nth(1).is_some_and(|r| r.starts_with(&format!("{kind}(")))).unwrap();
    line[1..line.find(' ').unwrap()].parse().unwrap()
}

/// **A trimmed curve, a composite curve of one segment and a replica that moves nothing are the
/// curves they wrap**: a trimmed curve by its parameters, its points or both, one running against
/// its basis (`sense_agreement` false) read the other way round — under an edge that runs against
/// it in turn; a composite's one segment the same way. A composite of two segments, a replica
/// moved, an offset curve and a line of no length are refused naming them.
#[test]
fn a_curve_wrapped_is_the_curve_it_wraps() {
    let l = SIZE[0];
    let trimmed = |trims: &'static str, sense: bool, edge: bool| move |e: &mut Ents, a: V, b: V| -> (u32, bool) {
        // the basis running from `b` to `a` where the trimmed curve runs against it
        let (p, q) = if sense { (a, b) } else { (b, a) };
        let line = e.line(p, unit(sub(q, p)), 1.);
        let (pa, pb) = (e.point(a), e.point(b));
        let (t0, t1) = if sense { (0., l) } else { (l, 0.) };
        let t = trims.replace("P0", &format!("#{}", if sense { pa } else { pb })).replace("P1", &format!("#{}", if sense { pb } else { pa }))
            .replace("T0", &real(t0)).replace("T1", &real(t1));
        let c = e.add(format!("TRIMMED_CURVE('',#{line},{t},{},.UNSPECIFIED.)", logical(sense)));
        (c, edge)
    };
    reads("trimmed by its parameters", &box_edge(&trimmed("(PARAMETER_VALUE(T0)),(PARAMETER_VALUE(T1))", true, true)), box_volume(), box_area(), 1e-12);
    reads("trimmed by its points", &box_edge(&trimmed("(P0),(P1)", true, true)), box_volume(), box_area(), 1e-12);
    reads("trimmed by both", &box_edge(&trimmed("(P0,PARAMETER_VALUE(T0)),(PARAMETER_VALUE(T1),P1)", true, true)), box_volume(), box_area(), 1e-12);
    // (its basis from b to a, the trimmed curve against it from a to b: the edge along it)
    reads("trimmed against its basis", &box_edge(&trimmed("(PARAMETER_VALUE(T0)),(PARAMETER_VALUE(T1))", false, true)), box_volume(), box_area(), 1e-12);
    let segment = |same: bool| move |e: &mut Ents, a: V, b: V| -> (u32, bool) {
        let (p, q) = if same { (a, b) } else { (b, a) };
        let line = e.line(p, unit(sub(q, p)), 2.);
        let s = e.add(format!("COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,{},#{line})", logical(same)));
        (e.add(format!("COMPOSITE_CURVE('',(#{s}),.F.)")), true)
    };
    reads("a composite of one segment", &box_edge(&segment(true)), box_volume(), box_area(), 1e-12);
    reads("a composite of one segment against it", &box_edge(&segment(false)), box_volume(), box_area(), 1e-12);
    let replica = |origin: V| move |e: &mut Ents, a: V, b: V| -> (u32, bool) {
        let line = e.line(a, unit(sub(b, a)), 1.);
        let o = e.point(origin);
        let op = e.add(format!("CARTESIAN_TRANSFORMATION_OPERATOR_3D('',$,$,#{o},$,$)"));
        (e.add(format!("CURVE_REPLICA('',#{line},#{op})")), true)
    };
    reads("a replica moving nothing", &box_edge(&replica([0.; 3])), box_volume(), box_area(), 1e-12);
    let moved = box_edge(&replica([0., 0., 1.]));
    refused("a replica moved", &moved, Why::Unsupported, first(&moved, "CURVE_REPLICA"));
    let two = box_edge(&|e, a, b| {
        let m = scale(add(a, b), 0.5);
        let (l0, l1) = (e.line(a, unit(sub(b, a)), 1.), e.line(m, unit(sub(b, a)), 1.));
        let (s0, s1) = (e.add(format!("COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,.T.,#{l0})")), e.add(format!("COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,.T.,#{l1})")));
        (e.add(format!("COMPOSITE_CURVE('',(#{s0},#{s1}),.F.)")), true)
    });
    refused("a composite of two segments", &two, Why::Unsupported, first(&two, "COMPOSITE_CURVE"));
    let offset = box_edge(&|e, a, b| {
        let line = e.line(add(a, [0., 0., -1.]), unit(sub(b, a)), 1.);
        let d = e.dir([0., 1., 0.]);
        (e.add(format!("OFFSET_CURVE_3D('',#{line},1.,.F.,#{d})")), true)
    });
    refused("an offset curve", &offset, Why::Unsupported, first(&offset, "OFFSET_CURVE_3D"));
    // a line whose vector has no length is a point, along which no edge runs
    let point = box_edge(&|e, a, b| (e.line(a, unit(sub(b, a)), 0.), true));
    refused("a line of no length", &point, Why::Malformed, first(&point, "LINE"));
}

/// **A face's surface trimmed, bounded, replicated or offset is the surface it is**: a
/// rectangular trimmed surface's and a curve-bounded surface's basis (the face's bounds trim it),
/// a replica moving nothing its parent, an offset of a plane the plane it is, of a spline sheet
/// the kernel's offset of the sheet, an offset of an offset the two together. Refused: a trimmed
/// surface against its basis in one parameter (its normal turned), a replica moved, an offset
/// its writer says intersects itself.
#[test]
fn a_surface_wrapped_is_the_surface_it_is() {
    reads("a rectangular trimmed plane", &box_top(&wrapped(|_, p| format!("RECTANGULAR_TRIMMED_SURFACE('',#{p},0.,3.,0.,4.,.T.,.T.)"))),
        box_volume(), box_area(), 1e-12);
    reads("a rectangular trimmed plane against it both ways", &box_top(&wrapped(|_, p| format!("RECTANGULAR_TRIMMED_SURFACE('',#{p},3.,0.,4.,0.,.F.,.F.)"))),
        box_volume(), box_area(), 1e-12);
    reads("a curve-bounded plane", &box_top(&wrapped(|_, p| format!("CURVE_BOUNDED_SURFACE('',#{p},(),.F.)"))), box_volume(), box_area(), 1e-12);
    reads("a plane's replica moving nothing", &box_top(&wrapped(|e, p| {
        let (o, x) = (e.point([0.; 3]), e.dir([1., 0., 0.]));
        let op = e.add(format!("CARTESIAN_TRANSFORMATION_OPERATOR_3D('','',#{x},$,#{o},1.,$)"));
        format!("SURFACE_REPLICA('',#{p},#{op})")
    })), box_volume(), box_area(), 1e-12);
    let turned = box_top(&wrapped(|_, p| format!("RECTANGULAR_TRIMMED_SURFACE('',#{p},0.,3.,4.,0.,.T.,.F.)")));
    refused("a trimmed plane against it in v alone", &turned, Why::Unsupported, first(&turned, "RECTANGULAR_TRIMMED_SURFACE"));
    let moved = box_top(&wrapped(|e, p| {
        let o = e.point([0., 0., 1.]);
        let op = e.add(format!("CARTESIAN_TRANSFORMATION_OPERATOR_3D('',$,$,#{o},$,$)"));
        format!("SURFACE_REPLICA('',#{p},#{op})")
    }));
    refused("a plane's replica moved", &moved, Why::Unsupported, first(&moved, "SURFACE_REPLICA"));
    // offsets of the top: a plane below it by one, along its normal; a plane above it facing down,
    // by minus one (along its own normal, up); a bilinear sheet below it, the kernel's offset; an
    // offset of an offset
    let offset = |below: f64, up: bool, flag: &'static str| move |e: &mut Ents, o: V, _: V| -> (u32, bool) {
        let p = if up { e.plane(sub(o, [0., 0., below]), [0., 0., 1.], [1., 0., 0.]) } else { e.plane(add(o, [0., 0., below]), [0., 0., -1.], [1., 0., 0.]) };
        (e.add(format!("OFFSET_SURFACE('',#{p},{},{flag})", real(below))), up)
    };
    reads("a plane's offset", &box_top(&offset(1., true, ".F.")), box_volume(), box_area(), 1e-12);
    reads("a plane facing down, offset against it", &box_top(&offset(1., false, ".U.")), box_volume(), box_area(), 1e-12);
    reads("an offset of an offset", &box_top(&|e, o, _| {
        let p = e.plane(sub(o, [0., 0., 3.]), [0., 0., 1.], [1., 0., 0.]);
        let q = e.add(format!("OFFSET_SURFACE('',#{p},2.,.F.)"));
        (e.add(format!("OFFSET_SURFACE('',#{q},1.,.F.)")), true)
    }), box_volume(), box_area(), 1e-12);
    reads("a spline sheet's offset", &box_top(&|e, _, _| {
        let rows: Vec<String> = [-1., 4.].iter().map(|&x| {
            let (a, b) = (e.point([x, -1., SIZE[2] - 2.]), e.point([x, 5., SIZE[2] - 2.]));
            format!("(#{a},#{b})")
        }).collect();
        let net = e.add(format!("B_SPLINE_SURFACE_WITH_KNOTS('',1,1,({}),.UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.)", rows.join(",")));
        (e.add(format!("OFFSET_SURFACE('',#{net},2.,.F.)")), true)
    }), box_volume(), box_area(), 1e-9);
    let crossing = box_top(&offset(1., true, ".T."));
    refused("an offset said to intersect itself", &crossing, Why::Unsupported, first(&crossing, "OFFSET_SURFACE"));
}

/// **A solid turned about `z` between two planes**: its side `side` makes (its surface and whether
/// its normal points out), meeting the plane `z = vb[2]` in the circle through `vb` (a pole where
/// `vb` is on the axis) and `z = vt[2]` in the circle through `vt`, its seam from `vb` to `vt` the
/// curve `seam` makes given the side's entity (the edge's geometry: a curve, or a seam curve with
/// its curves in the side's parameters).
fn drum(side: &dyn Fn(&mut Ents) -> (u32, bool), seam: &dyn Fn(&mut Ents, u32) -> u32, vb: V, vt: V) -> String {
    let mut e = Ents::new();
    let z: V = [0., 0., 1.];
    let (s, out) = side(&mut e);
    let pole = vb[0].hypot(vb[1]) == 0.;
    let (b, t) = (e.vertex(vb), e.vertex(vt));
    let g = seam(&mut e, s);
    let es = e.edge(b, t, g, true);
    let ring = |e: &mut Ents, v: V, at: u32| -> u32 {
        let c = e.circle([0., 0., v[2]], z, unit([v[0], v[1], 0.]), v[0].hypot(v[1]));
        e.edge(at, at, c, true)
    };
    let et = ring(&mut e, vt, t);
    let top = e.plane([0., 0., vt[2]], z, [1., 0., 0.]);
    let mut faces = vec![e.face(&[(et, true)], false, top, true)];
    if pole {
        faces.push(e.face(&[(es, true), (et, false), (es, false)], false, s, out));
    } else {
        let eb = ring(&mut e, vb, b);
        let bottom = e.plane([0., 0., vb[2]], z, [1., 0., 0.]);
        faces.push(e.face(&[(eb, false)], false, bottom, false));
        faces.push(e.face(&[(eb, true), (es, true), (et, false), (es, false)], false, s, out));
    }
    e.file(&faces)
}

/// A line from `a` to `b`, as an edge's curve.
fn segment(e: &mut Ents, a: V, b: V) -> u32 { e.line(a, unit(sub(b, a)), 1.) }

/// The volume and area of the solid turned about `z` whose side's meridian is `ρ(z)` from `z0` to
/// `z1` (each disc at an end where `ρ` is not zero there): `π ∫ ρ²` and `2π ∫ ρ √(1 + ρ′²)` with
/// the discs' `π ρ²`.
fn turned_measures(rho: &dyn Fn(f64) -> f64, slope: &dyn Fn(f64) -> f64, z0: f64, z1: f64) -> (f64, f64) {
    let v = PI * integral(&|z| rho(z) * rho(z), z0, z1, 4000);
    let a = TAU * integral(&|z| rho(z) * (1. + slope(z) * slope(z)).sqrt(), z0, z1, 4000) + PI * (rho(z0).powi(2) + rho(z1).powi(2));
    (v, a)
}

/// **An offset of an elementary surface is the surface of its kind it is, and a cone of negative
/// radius the cone its formula says**: a cylinder, a cone and a sphere each grown or shrunk along
/// its normal (a cone's radius at its origin `r + d / cos α`, its frame kept), and a frustum on a
/// `CONICAL_SURFACE` whose radius at its origin is negative — its apex beyond its origin, which the
/// standard asks not to be but whose points and normal the formula gives as for any other —
/// each read to its closed form. A cone of no half-angle or of a right angle is refused, malformed.
#[test]
fn an_offset_of_an_elementary_surface_is_one_of_its_kind() {
    let (z, x): (V, V) = ([0., 0., 1.], [1., 0., 0.]);
    // a cylinder of radius 2 from z = 1 to 3
    let cylinder = |e: &mut Ents, r: f64, d: f64| -> (u32, bool) {
        let a = e.place([0.; 3], z, x);
        let c = e.add(format!("CYLINDRICAL_SURFACE('',#{a},{})", real(r)));
        (e.add(format!("OFFSET_SURFACE('',#{c},{},.F.)", real(d))), true)
    };
    let (v, a) = (PI * 4. * 2., TAU * 2. * 2. + 2. * PI * 4.);
    reads("a cylinder grown", &drum(&|e| cylinder(e, 1.5, 0.5), &|e, _| segment(e, [2., 0., 1.], [2., 0., 3.]), [2., 0., 1.], [2., 0., 3.]), v, a, 1e-12);
    reads("a cylinder shrunk", &drum(&|e| cylinder(e, 2.5, -0.5), &|e, _| segment(e, [2., 0., 1.], [2., 0., 3.]), [2., 0., 1.], [2., 0., 3.]), v, a, 1e-12);
    // the frustum ρ = 1 + z from z = 1 to 3: its cone's radius at the origin 1, half-angle π/4
    let (v, a) = turned_measures(&|z| 1. + z, &|_| 1., 1., 3.);
    let frustum = |side: &dyn Fn(&mut Ents) -> (u32, bool)| drum(side, &|e, _| segment(e, [2., 0., 1.], [4., 0., 3.]), [2., 0., 1.], [4., 0., 3.]);
    reads("a cone grown", &frustum(&|e| {
        let p = e.place([0.; 3], z, x);
        let c = e.add(format!("CONICAL_SURFACE('',#{p},{},{})", real(1. - 0.5 / FRAC_PI_4.cos()), real(FRAC_PI_4)));
        (e.add(format!("OFFSET_SURFACE('',#{c},0.5,.F.)")), true)
    }), v, a, 1e-12);
    reads("a cone of negative radius", &frustum(&|e| {
        // its origin at z = −2, where the cone's radius is 1 − 2 = −1
        let p = e.place([0., 0., -2.], z, x);
        (e.add(format!("CONICAL_SURFACE('',#{p},-1.,{})", real(FRAC_PI_4))), true)
    }), v, a, 1e-12);
    // a cone of no half-angle (a cylinder) or a right angle's (a plane) is no cone: malformed
    for half in [0., FRAC_PI_2] {
        let flat = frustum(&|e| { let p = e.place([0.; 3], z, x); (e.add(format!("CONICAL_SURFACE('',#{p},1.,{})", real(half))), true) });
        refused(&format!("a cone of half-angle {half}"), &flat, Why::Malformed, first(&flat, "CONICAL_SURFACE"));
    }
    // a ball's cap below z = 1, of radius 3: a sphere of radius 2 grown by one, its seam the
    // meridian from the south pole
    let (r, h): (f64, f64) = (3., 1.);
    let (v, a) = (PI * (r * r * (h + r) - (h.powi(3) + r.powi(3)) / 3.), TAU * r * (r + h) + PI * (r * r - h * h));
    let top = [(r * r - h * h).sqrt(), 0., h];
    let cap = drum(&|e| {
        let p = e.place([0.; 3], z, x);
        let s = e.add(format!("SPHERICAL_SURFACE('',#{p},2.)"));
        (e.add(format!("OFFSET_SURFACE('',#{s},1.,.F.)")), true)
    }, &|e, _| e.circle([0.; 3], [0., -1., 0.], x, r), [0., 0., -r], top);
    reads("a sphere grown", &cap, v, a, 1e-12);
}

/// **A line turned about an axis it does not lie in a plane with** sweeps a hyperboloid of one
/// sheet: read as the revolution of the hyperbola its points sweep in a half-plane of the axis,
/// bounded past its faces. **One meeting the axis** sweeps a double cone: read as the half of the
/// line its faces lie on, turned. Each a solid between two planes, its seam the line itself, read
/// to its closed form.
#[test]
fn a_line_turned_off_its_axis_plane_sweeps_its_hyperboloid() {
    // the line (a, s t, c t): ρ² = a² + s² t², z = c t
    let (a, s, c) = (2., 0.6, 0.8);
    let h = 3.;
    let at = |z: f64| -> V { [a, s * z / c, z] };
    let (v, ar) = turned_measures(&|z| (a * a + (s * z / c).powi(2)).sqrt(), &|z| (s / c).powi(2) * z / (a * a + (s * z / c).powi(2)).sqrt(), -h, h);
    close("the hyperboloid's volume in closed form", v, PI * (2. * h * a * a + (s / c).powi(2) * 2. * h.powi(3) / 3.), 1e-12);
    let side = |e: &mut Ents| -> (u32, bool) {
        let l = e.line([a, -1., -c / s], [0., s, c], 1.);
        let axis = { let (o, d) = (e.point([0.; 3]), e.dir([0., 0., 1.])); e.add(format!("AXIS1_PLACEMENT('',#{o},#{d})")) };
        (e.add(format!("SURFACE_OF_REVOLUTION('',#{l},#{axis})")), true)
    };
    reads("a hyperboloid of one sheet", &drum(&side, &|e, _| segment(e, at(-h), at(h)), at(-h), at(h)), v, ar, 1e-11);
    // the frustum ρ = z from z = 1 to 3, its line given below the apex, on the other nappe
    let (v, ar) = turned_measures(&|z| z, &|_| 1., 1., 3.);
    let nappe = |e: &mut Ents| -> (u32, bool) {
        let l = e.line([-1., 0., -1.], [1., 0., 1.], 1.);
        let axis = { let (o, d) = (e.point([0.; 3]), e.dir([0., 0., 1.])); e.add(format!("AXIS1_PLACEMENT('',#{o},#{d})")) };
        (e.add(format!("SURFACE_OF_REVOLUTION('',#{l},#{axis})")), true)
    };
    reads("a cone turned from a line across its apex", &drum(&nappe, &|e, _| segment(e, [1., 0., 1.], [3., 0., 3.]), [1., 0., 1.], [3., 0., 3.]), v, ar, 1e-12);
}

/// **A parabola's and a hyperbola's arcs are exact rational quadratics between an edge's
/// vertices**, and **a conic without end turned about an axis in its plane** is the revolution of
/// its arc over its faces. A paraboloid `z = ρ² / 4F` whose seam is its parabola: the parabola
/// crosses the axis at its vertex, its halves turning to the same points facing opposite ways, the
/// half its faces lie on said by its curve in their parameters — and refused where none says it;
/// a hyperboloid of one sheet whose seam is its hyperbola (one half, no curve needed): each read
/// to its closed form.
#[test]
fn a_conic_without_end_is_read_as_its_arc() {
    let (z, x): (V, V) = ([0., 0., 1.], [1., 0., 0.]);
    let f = 1.;
    let h = 4.;
    // the parabola F (u² z + 2u x) in the plane y = 0: ρ = 2Fu, z = Fu²
    let parabola = |e: &mut Ents| { let p = e.place([0.; 3], [0., 1., 0.], z); e.add(format!("PARABOLA('',#{p},{})", real(f))) };
    let axis = |e: &mut Ents| { let (o, d) = (e.point([0.; 3]), e.dir(z)); e.add(format!("AXIS1_PLACEMENT('',#{o},#{d})")) };
    let side = |e: &mut Ents| -> (u32, bool) { let (c, a) = (parabola(e), axis(e)); (e.add(format!("SURFACE_OF_REVOLUTION('',#{c},#{a})")), true) };
    let rim = [2. * (f * h).sqrt(), 0., h];
    let v = TAU * f * h * h;
    let a = 8. * PI * f * f / 3. * ((1. + h / f).powf(1.5) - 1.) + 4. * PI * f * h;
    let with_pcurves = drum(&side, &|e, s| {
        let c = parabola(e);
        let pcurve = |e: &mut Ents, u: f64| {
            let (p, d) = (e.flat_point([u, 0.]), e.add("DIRECTION('',(0.,1.))"));
            let v = e.add(format!("VECTOR('',#{d},1.)"));
            let l = e.add(format!("LINE('',#{p},#{v})"));
            let ctx = e.add("( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D SPACE','') )");
            let r = e.add(format!("DEFINITIONAL_REPRESENTATION('',(#{l}),#{ctx})"));
            e.add(format!("PCURVE('',#{s},#{r})"))
        };
        let (p0, p1) = (pcurve(e, 0.), pcurve(e, TAU));
        e.add(format!("SEAM_CURVE('',#{c},(#{p0},#{p1}),.PCURVE_S1.)"))
    }, [0.; 3], rim);
    reads("a paraboloid", &with_pcurves, v, a, 1e-12);
    let without = drum(&side, &|e, _| parabola(e), [0.; 3], rim);
    refused("a paraboloid no curve says the half of", &without, Why::Unsupported, first(&without, "SURFACE_OF_REVOLUTION"));
    // the hyperbola a cosh u x + b sinh u z, turned about z: ρ = a cosh u, z = b sinh u
    let (ha, hb, u1): (f64, f64, f64) = (2., 3., 0.9);
    let hyperbola = |e: &mut Ents| { let p = e.place([0.; 3], [0., -1., 0.], x); e.add(format!("HYPERBOLA('',#{p},{},{})", real(ha), real(hb))) };
    let side = |e: &mut Ents| -> (u32, bool) { let (c, a) = (hyperbola(e), axis(e)); (e.add(format!("SURFACE_OF_REVOLUTION('',#{c},#{a})")), true) };
    let zt = hb * u1.sinh();
    let rho = |z: f64| ha * (1. + z * z / (hb * hb)).sqrt();
    let (v, a) = turned_measures(&rho, &|z| ha * z / (hb * hb) / (1. + z * z / (hb * hb)).sqrt(), -zt, zt);
    close("the hyperboloid's volume in closed form", v, PI * ha * ha * (2. * zt + 2. * zt.powi(3) / (3. * hb * hb)), 1e-12);
    reads("a hyperboloid of its hyperbola", &drum(&side, &|e, _| hyperbola(e), [rho(-zt), 0., -zt], [rho(zt), 0., zt]), v, a, 1e-11);
}

/// **A curve turned off its axis's half-plane is the rational net of its revolution, its seam
/// where no face crosses it** (CADGenBench 249, read 217 mm off before: a net has no period, and
/// its face crossed where the file's turn starts). A circle beside the axis, not in a plane of it,
/// turned from `u1 < 0` to `u2 > 0` — across where the file's turn starts — sweeps a solid bounded
/// by the side it turns and the two discs it starts and ends at. Its volume is the disc's sweep,
/// `(u2 − u1) π r² c·(n × z)` (the speed of a point of the disc along its normal is linear in it),
/// its side's area `(u2 − u1) ∫ |(z × C) × C′| dt`, read to them; and the same turned from `0.3`
/// to `1.8`, on one side of it.
#[test]
fn a_curve_turned_off_its_axis_plane_is_seated_where_no_face_crosses() {
    let z: V = [0., 0., 1.];
    let c: V = [10., 2., 1.];
    let r = 3.;
    let n = unit([0.3, 1., 0.2]);
    let ax = unit(cross([0., 0., 1.], n));
    let ay = cross(n, ax);
    let at = |t: f64| add(c, add(scale(ax, r * t.cos()), scale(ay, r * t.sin())));
    let side_area = integral(&|t| {
        let p = at(t);
        let d = add(scale(ax, -r * t.sin()), scale(ay, r * t.cos()));
        norm(cross(cross(z, p), d))
    }, 0., TAU, 2000);
    for (u1, u2) in [(-0.6, 0.9), (0.3, 1.8)] {
        let mut e = Ents::new();
        let du = u2 - u1;
        let (v1, v2) = (e.vertex(turn(at(0.), u1)), e.vertex(turn(at(0.), u2)));
        let ring = |e: &mut Ents, u: f64| e.circle(turn(c, u), turn(n, u), turn(ax, u), r);
        let (c1, c2) = (ring(&mut e, u1), ring(&mut e, u2));
        let (e1, e2) = (e.edge(v1, v1, c1, true), e.edge(v2, v2, c2, true));
        let start = at(0.);
        let lat = e.circle([0., 0., start[2]], z, unit([turn(start, u1)[0], turn(start, u1)[1], 0.]), start[0].hypot(start[1]));
        let e3 = e.edge(v1, v2, lat, true);
        let curve = e.circle(c, n, ax, r);
        let axis = { let (o, d) = (e.point([0.; 3]), e.dir(z)); e.add(format!("AXIS1_PLACEMENT('',#{o},#{d})")) };
        let side = e.add(format!("SURFACE_OF_REVOLUTION('',#{curve},#{axis})"));
        // (its normal `S_u × S_v` inward: the disc moves along +n as it turns)
        let lateral = e.face(&[(e3, true), (e2, true), (e3, false), (e1, false)], true, side, false);
        let p2 = e.plane(turn(c, u2), turn(n, u2), turn(ax, u2));
        let p1 = e.plane(turn(c, u1), turn(n, u1), turn(ax, u1));
        let cap2 = e.face(&[(e2, true)], false, p2, true);
        let cap1 = e.face(&[(e1, false)], false, p1, false);
        let text = e.file(&[lateral, cap1, cap2]);
        let v = du * PI * r * r * dot(c, cross(n, z));
        reads(&format!("a circle turned from {u1} to {u2}"), &text, v, du * side_area + 2. * PI * r * r, 1e-9);
    }
}

/// **A degenerate torus is the spindle torus that holds it**: the lemon (`select_outer` false) of
/// major radius 3 and minor 5, its face the kernel's torus's between its poles, read to the lemon's
/// closed form — the same as the kernel writes it, a toroidal surface of major radius −3 (the same
/// points, its `v` turned, so the face's sense with it). One whose major radius is not under its
/// minor is refused, malformed.
#[test]
fn a_degenerate_torus_is_its_spindle_torus() {
    let lemon = |surface: &str, sense: bool| -> String {
        let mut e = Ents::new();
        let (b, t) = (e.vertex([0., 0., -4.]), e.vertex([0., 0., 4.]));
        let a = e.place([0.; 3], [0., 0., 1.], [1., 0., 0.]);
        let s = e.add(surface.replace('@', &format!("#{a}")));
        let c = e.circle([-3., 0., 0.], [0., -1., 0.], [1., 0., 0.], 5.);
        let m = e.edge(b, t, c, true);
        let f = e.face(&[(m, true), (m, false)], false, s, sense);
        e.file(&[f])
    };
    let rho = |z: f64| (25. - z * z).sqrt() - 3.;
    let (v, a) = turned_measures(&rho, &|z| -z / (25. - z * z).sqrt(), -4., 4.);
    reads("the kernel's lemon", &lemon("TOROIDAL_SURFACE('',@,-3.,5.)", true), v, a, 1e-11);
    reads("a degenerate torus's lemon", &lemon("DEGENERATE_TOROIDAL_SURFACE('',@,3.,5.,.F.)", false), v, a, 1e-11);
    let wrong = lemon("DEGENERATE_TOROIDAL_SURFACE('',@,5.,3.,.F.)", false);
    refused("a degenerate torus of a ring", &wrong, Why::Malformed, first(&wrong, "DEGENERATE_TOROIDAL_SURFACE"));
}

/// **A placement's reference direction left unset is the standard's** (ISO 10303-42,
/// `first_proj_axis`: `[1, 0, 0]`, or `[0, 1, 0]` about an axis along `±x`): a cylinder the kernel
/// writes along `−x`, every placement about `±x` written again with no reference direction — as
/// CATIA V5 writes `AXIS2_PLACEMENT_3D('…',#p,#z,$)` — reads to the same cylinder (refused before,
/// "a placement whose reference direction lies along its axis").
#[test]
fn a_placement_without_a_reference_direction_takes_the_standards() {
    use gcs_core::brep::build::{disc, prism};
    use gcs_core::brep::step::{write, Fit};
    let b = prism(&disc([15., 0., 0.], [-1., 0., 0.], 5.), 0., 30.).unwrap();
    let text = write(&b, "pin", 1e-7, Fit::Exact).unwrap();
    let line = |n: &str| text.lines().find(|l| l.starts_with(&format!("{n} = "))).unwrap_or_else(|| panic!("no {n}"));
    let (mut out, mut unset) = (String::new(), 0);
    for l in text.lines() {
        let Some(at) = l.find("AXIS2_PLACEMENT_3D(") else { out += l; out.push('\n'); continue };
        let refs: Vec<&str> = l[at..].trim_end_matches(");").split(',').skip(1).collect();
        let axis: Vec<f64> = line(refs[1]).split("'',(").nth(1).unwrap().trim_end_matches("));").split(',').map(|x| x.parse().unwrap()).collect();
        if axis[0].abs() == 1. && axis[1] == 0. && axis[2] == 0. {
            out += &l.replace(&format!(",{});", refs[2]), ",$);");
            unset += 1;
        } else { out += l; }
        out.push('\n');
    }
    assert!(unset >= 3, "the cylinder's and its caps' placements stand about ±x ({unset} found)");
    // and with each such axis a rounding off `±x`, as CATIA writes `(1.,-2.1862853408E-015,0.)`:
    // the default `[1, 0, 0]` a hair off the axis, its part square to it still a direction
    let hair = out.replace("(-1.0,0.0,0.0))", "(-1.,2.1862853408E-015,0.))");
    assert!(hair != out, "an axis written along -x");
    for (what, text) in [("along ±x", &out), ("a rounding off ±x", &hair)] {
        let (v, a) = measures(text).unwrap_or_else(|r| panic!("{what}: refused: {r}"));
        close(&format!("volume, {what}"), v, PI * 25. * 30., 1e-12);
        close(&format!("area, {what}"), a, 2. * PI * 5. * 30. + 2. * PI * 25., 1e-12);
    }
}

/// A rational B-spline curve of degree 2 through `poles` weighed `weights`, its knots `knots`
/// with multiplicities `mults`: the complex entity a writer writes.
fn rational_curve(e: &mut Ents, poles: &[V], weights: &[f64], mults: &[usize], knots: &[f64]) -> u32 {
    let p: Vec<String> = poles.iter().map(|&q| format!("#{}", e.point(q))).collect();
    let m: Vec<String> = mults.iter().map(|m| m.to_string()).collect();
    e.add(format!("( BOUNDED_CURVE() B_SPLINE_CURVE(2,({}),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS(({}),({}),.UNSPECIFIED.) \
        CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM('') )",
        p.join(","), m.join(","), reals(knots), reals(weights)))
}

/// The half ellipse `(a cos v, 0, c sin v)`, `v` from `−π/2` to `π/2`, as two rational quadratics:
/// its poles and weights.
fn half_ellipse(a: f64, c: f64) -> (Vec<V>, Vec<f64>) {
    let w = FRAC_PI_4.cos();
    (vec![[0., 0., -c], [a, 0., -c], [a, 0., 0.], [a, 0., c], [0., 0., c]], vec![1., w, 1., w, 1.])
}

/// The volume and area of the spheroid of equatorial radius `a` and polar `c` (`c > a`).
fn spheroid(a: f64, c: f64) -> (f64, f64) {
    let e = (1. - a * a / (c * c)).sqrt();
    (4. / 3. * PI * a * a * c, TAU * a * a * (1. + c / (a * e) * e.asin()))
}

/// **A face bounded by vertices alone is the whole of a surface that closes** (ISO 10303-42 lets a
/// face's bounds be `VERTEX_LOOP`s; Open CASCADE writes an ellipsoid, a revolved spheroid and a
/// torus of revolution so): a spheroid as the rational net of its half ellipse turned (closed
/// round `u`, its rows at the ends of `v` its poles), bounded by its south pole alone; the same
/// spheroid as a revolution of that half ellipse, bounded by its north pole; a ring torus as a
/// revolution of a circle, bounded by a vertex on its outer equator — each read to its closed form.
/// A circle turned across its axis, whose whole is the apple and the lemon of a spindle torus
/// touching at its poles, is refused naming the face (no face of a manifold solid); a vertex on the
/// net off both its poles and its seam is refused naming its bound.
#[test]
fn a_face_bounded_by_vertices_alone_is_the_whole_of_a_surface_that_closes() {
    let (a, c) = (3., 5.);
    let (meridian, mw) = half_ellipse(a, c);
    let lone = |e: &mut Ents, p: V, surface: u32| -> u32 {
        let v = e.vertex(p);
        let l = e.add(format!("VERTEX_LOOP('',#{v})"));
        let b = e.add(format!("FACE_BOUND('',#{l},.T.)"));
        e.add(format!("ADVANCED_FACE('',(#{b}),#{surface},.T.)"))
    };
    // the net: a whole turn's rational circle (four quarters) times the half ellipse
    let w = FRAC_PI_4.cos();
    let round: [(f64, f64, f64); 9] = [(1., 0., 1.), (1., 1., w), (0., 1., 1.), (-1., 1., w), (-1., 0., 1.), (-1., -1., w), (0., -1., 1.), (1., -1., w), (1., 0., 1.)];
    let net = |at: V| -> String {
        let mut e = Ents::new();
        let rows: Vec<String> = round.iter().map(|&(x, y, _)| {
            let r: Vec<String> = meridian.iter().map(|q| format!("#{}", e.point([q[0] * x, q[0] * y, q[2]]))).collect();
            format!("({})", r.join(","))
        }).collect();
        let weights: Vec<String> = round.iter().map(|&(_, _, wu)| format!("({})", reals(&mw.iter().map(|wv| wu * wv).collect::<Vec<_>>()))).collect();
        let s = e.add(format!("( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,2,({}),.UNSPECIFIED.,.T.,.F.,.F.) \
            B_SPLINE_SURFACE_WITH_KNOTS((3,2,2,2,3),(3,2,3),(0.,0.25,0.5,0.75,1.),(0.,0.5,1.),.UNSPECIFIED.) \
            GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({})) REPRESENTATION_ITEM('') SURFACE() )",
            rows.join(","), weights.join(",")));
        let f = lone(&mut e, at, s);
        e.file(&[f])
    };
    let (v, ar) = spheroid(a, c);
    reads("a spheroid's net bounded by its south pole", &net([0., 0., -c]), v, ar, 1e-9);
    let off = net([-a, 0., 0.]);
    refused("a vertex off the net's poles and seam", &off, Why::Unsupported, first(&off, "VERTEX_LOOP"));
    let revolution = |curve: &dyn Fn(&mut Ents) -> u32, at: V| -> String {
        let mut e = Ents::new();
        let k = curve(&mut e);
        let (o, z) = (e.point([0.; 3]), e.dir([0., 0., 1.]));
        let axis = e.add(format!("AXIS1_PLACEMENT('',#{o},#{z})"));
        let s = e.add(format!("SURFACE_OF_REVOLUTION('',#{k},#{axis})"));
        let f = lone(&mut e, at, s);
        e.file(&[f])
    };
    let spheroid_turned = revolution(&|e| rational_curve(e, &meridian, &mw, &[3, 2, 3], &[0., 0.5, 1.]), [0., 0., c]);
    reads("a spheroid turned, bounded by its north pole", &spheroid_turned, v, ar, 1e-9);
    let (big, r) = (10., 2.);
    let ring = revolution(&|e| e.circle([big, 0., 0.], [0., -1., 0.], [1., 0., 0.], r), [big + r, 0., 0.]);
    reads("a ring torus turned, bounded by a vertex on its outer equator", &ring, TAU * PI * big * r * r, TAU * TAU * big * r, 1e-9);
    let apple = revolution(&|e| e.circle([-3., 0., 0.], [0., -1., 0.], [0.6, 0., -0.8], 5.), [0., 0., 4.]);
    refused("a circle turned across its axis, whole", &apple, Why::Unsupported, first(&apple, "ADVANCED_FACE"));
}

/// **An edge from a vertex to itself on a line of no length is that point** (brepkit and vcad
/// write a torus's seams so): a torus whose loop is two such edges each way round is the whole
/// torus, read to its closed form with both edges the file's, each the kernel's degenerate edge
/// there. A line of no length between two vertices, or standing off its one, is refused, malformed,
/// naming the line.
#[test]
fn an_edge_that_is_a_point_reads_as_one() {
    // (`apart`: one edge from the vertex to a second at `(8, 0, 0)`, the loop out along it and back)
    let torus = |apart: bool, at: V| -> String {
        let mut e = Ents::new();
        let v = e.vertex([12., 0., 0.]);
        let s = e.place([0.; 3], [0., 0., 1.], [1., 0., 0.]);
        let s = e.add(format!("TOROIDAL_SURFACE('',#{s},10.,2.)"));
        let f = if apart {
            let w = e.vertex([8., 0., 0.]);
            let l = e.line(at, [1., 0., 0.], 0.);
            let x = e.edge(v, w, l, true);
            e.face(&[(x, true), (x, false)], false, s, true)
        } else {
            let l1 = e.line(at, [1., 0., 0.], 0.);
            let e1 = e.edge(v, v, l1, true);
            let l2 = e.line(at, [0., 1., 0.], 0.);
            let e2 = e.edge(v, v, l2, true);
            e.face(&[(e1, true), (e2, true), (e1, false), (e2, false)], false, s, true)
        };
        e.file(&[f])
    };
    let whole = torus(false, [12., 0., 0.]);
    reads("a torus bounded by two edges that are its vertex", &whole, TAU * PI * 10. * 4., TAU * TAU * 20., 1e-9);
    let given = read(&whole).unwrap();
    let mut named: Vec<u32> = given[0].edges.iter().copied().filter(|&x| x != 0).collect();
    named.sort_unstable();
    named.dedup();
    assert_eq!(named.len(), 2, "both edges the file's: {:?}", given[0].edges);
    let apart = torus(true, [12., 0., 0.]);
    refused("a line of no length between two vertices", &apart, Why::Malformed, first(&apart, "LINE"));
    let off = torus(false, [12., 0., 1.]);
    refused("a line of no length off its vertex", &off, Why::Malformed, first(&off, "LINE"));
}

/// **A closed conic turned across its axis is the revolution of its arc on the side its faces lie**
/// (springrep's lemon, a ball swung across its own centre line): the circle of radius 5 about
/// `(−3, 0, 0)` in the plane `y = 0`, turned about `z`, crosses the axis at `z = ±4`; a face bounded
/// by its seam along the arc from the south crossing to the north on the axis's `+x` side is the
/// lemon, `π ∫ ρ²` and `2π ∫ ρ √(1 + ρ′²)`, one along the other arc the apple, the lemon's and the
/// whole circle's by Pappus — each read to its closed form (the turned net of the whole circle read
/// the lemon's seam 8 mm off it).
#[test]
fn a_conic_turned_across_its_axis_is_its_arc_on_its_faces_side() {
    let file = |lemon: bool| -> String {
        let mut e = Ents::new();
        let (s, n) = (e.vertex([0., 0., -4.]), e.vertex([0., 0., 4.]));
        let k = e.circle([-3., 0., 0.], [0., -1., 0.], [0.6, 0., -0.8], 5.);
        let (o, z) = (e.point([0.; 3]), e.dir([0., 0., 1.]));
        let axis = e.add(format!("AXIS1_PLACEMENT('',#{o},#{z})"));
        let surface = e.add(format!("SURFACE_OF_REVOLUTION('',#{k},#{axis})"));
        let c = e.circle([-3., 0., 0.], [0., -1., 0.], [0.6, 0., -0.8], 5.);
        let m = if lemon { e.edge(s, n, c, true) } else { e.edge(n, s, c, true) };
        let f = e.face(&[(m, false), (m, true)], false, surface, lemon);
        e.file(&[f])
    };
    let root = |z: f64| (25. - z * z).sqrt();
    let (v, a) = turned_measures(&|z| root(z) - 3., &|z| -z / root(z), -4., 4.);
    reads("the lemon", &file(true), v, a, 1e-9);
    // (Pappus's theorem over the whole circle, its radius from the axis signed: the apple less the
    // lemon, `2π² R r²` and `4π² R r`)
    reads("the apple", &file(false), TAU * PI * 3. * 25. + v, TAU * TAU * 15. + a, 1e-9);
}

/// **A closed edge's vertex may stand anywhere on its closed curve** (ISO 10303-42; springrep's
/// offset hole): a cylinder whose two circles are closed rational B-splines starting at `+x`, each
/// bounded by a vertex at `+y`, its seam the line between them, reads to its closed form — each
/// spline started again at its vertex, the same points.
#[test]
fn a_closed_edge_on_a_spline_runs_round_from_its_vertex() {
    let (r, h) = (4., 7.);
    let w = FRAC_PI_4.cos();
    let ring = |z: f64| -> Vec<V> { [(1., 0.), (1., 1.), (0., 1.), (-1., 1.), (-1., 0.), (-1., -1.), (0., -1.), (1., -1.), (1., 0.)].iter().map(|&(x, y)| [r * x, r * y, z]).collect() };
    let weights = [1., w, 1., w, 1., w, 1., w, 1.];
    let mut e = Ents::new();
    let (b, t) = (e.vertex([0., r, 0.]), e.vertex([0., r, h]));
    let cb = rational_curve(&mut e, &ring(0.), &weights, &[3, 2, 2, 2, 3], &[0., 0.25, 0.5, 0.75, 1.]);
    let ct = rational_curve(&mut e, &ring(h), &weights, &[3, 2, 2, 2, 3], &[0., 0.25, 0.5, 0.75, 1.]);
    let (eb, et) = (e.edge(b, b, cb, true), e.edge(t, t, ct, true));
    let seam = segment(&mut e, [0., r, 0.], [0., r, h]);
    let es = e.edge(b, t, seam, true);
    let side = e.place([0.; 3], [0., 0., 1.], [1., 0., 0.]);
    let side = e.add(format!("CYLINDRICAL_SURFACE('',#{side},{})", real(r)));
    let bottom = e.plane([0.; 3], [0., 0., 1.], [1., 0., 0.]);
    let top = e.plane([0., 0., h], [0., 0., 1.], [1., 0., 0.]);
    let faces = [e.face(&[(eb, false)], false, bottom, false), e.face(&[(et, true)], false, top, true),
        e.face(&[(eb, true), (es, true), (et, false), (es, false)], false, side, true)];
    let text = e.file(&faces);
    reads("a cylinder whose circles start off their vertices", &text, PI * r * r * h, TAU * r * r + TAU * r * h, 1e-9);
}

/// A cylinder of radius `side` between `z = 0` and `z = h`, its seam at `+x`, whose top edge's
/// geometry is a `SURFACE_CURVE` of the circle of radius `r` in space and two curves in its faces'
/// parameters — the top plane's the circle of radius `r + δ` (run at the edge's own angle), the
/// side's the line `v = h` (its image the circle of radius `side`) — with the master `master`
/// names; `on` says, by the surface's place (0 the top, 1 the side, 2 the bottom), which surface
/// the top plane's curve says it lies on. Its area.
fn tolerant_drum(r: f64, side: f64, h: f64, delta: f64, master: &str, on: usize) -> Result<f64, Refusal> {
    let mut e = Ents::new();
    let z: V = [0., 0., 1.];
    let (b, t) = (e.vertex([side, 0., 0.]), e.vertex([side, 0., h]));
    let wall = e.place([0.; 3], z, [1., 0., 0.]);
    let wall = e.add(format!("CYLINDRICAL_SURFACE('',#{wall},{})", real(side)));
    let bottom = e.plane([0.; 3], z, [1., 0., 0.]);
    let top = e.plane([0., 0., h], z, [1., 0., 0.]);
    let cb = e.circle([0.; 3], z, [1., 0., 0.], side);
    let eb = e.edge(b, b, cb, true);
    let ct = e.circle([0., 0., h], z, [1., 0., 0.], r);
    // (the top's curve: the circle of radius r + δ about the plane's origin, from its x axis)
    let (o, x) = (e.flat_point([0., 0.]), e.add(format!("DIRECTION('',({}))", reals(&[1., 0.]))));
    let a2 = e.add(format!("AXIS2_PLACEMENT_2D('',#{o},#{x})"));
    let wide = e.add(format!("CIRCLE('',#{a2},{})", real(r + delta)));
    let surfaces = [top, wall, bottom];
    let ctx = e.add("( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('','') )");
    let d1 = e.add(format!("DEFINITIONAL_REPRESENTATION('',(#{wide}),#{ctx})"));
    let p1 = e.add(format!("PCURVE('',#{},#{d1})", surfaces[on]));
    // (the side's: the line v = h, its parameter the angle)
    let (q, dq) = (e.flat_point([0., h]), e.add(format!("DIRECTION('',({}))", reals(&[1., 0.]))));
    let vq = e.add(format!("VECTOR('',#{dq},1.)"));
    let l = e.add(format!("LINE('',#{q},#{vq})"));
    let d2 = e.add(format!("DEFINITIONAL_REPRESENTATION('',(#{l}),#{ctx})"));
    let p2 = e.add(format!("PCURVE('',#{wall},#{d2})"));
    let sc = e.add(format!("SURFACE_CURVE('',#{ct},(#{p1},#{p2}),.{master}.)"));
    let et = e.edge(t, t, sc, true);
    let seam = segment(&mut e, [side, 0., 0.], [side, 0., h]);
    let es = e.edge(b, t, seam, true);
    let faces = [e.face(&[(eb, false)], false, bottom, false), e.face(&[(et, true)], false, top, true),
        e.face(&[(eb, true), (es, true), (et, false), (es, false)], false, wall, true)];
    measures(&e.file(&faces)).map(|(_, a)| a)
}

/// **The master is the edge** (ISO 10303-42 `surface_curve.master_representation`: the
/// representation preferred where a tolerant file's 3D curve and pcurves differ). A cylinder of
/// radius `r + δ` whose top edge is, in space, the circle of radius `r` and, in its faces'
/// parameters, the circle of radius `r + δ` in the top plane's and the line `v = h` in the side's
/// (the circle of radius `r + δ` too): the 3D curve stands `δ` off both pcurves, which agree. Under
/// `.PCURVE_S1.` (the plane's curve the master, the side's following it) and `.PCURVE_S2.` (the
/// side's line the master, the plane's following it) the top is the disc the pcurves bound,
/// `π (r + δ)²`, the edge's tolerance `δ`, measured; under `.CURVE_3D.` neither pcurve is the
/// edge's within the file's uncertainty and each face is bounded by the circle's foot: the top
/// `π r²`. The side and the bottom are the same under all three.
#[test]
fn an_edge_is_what_its_master_says() {
    let (r, h, delta) = (10., 20., 0.01);
    let (wide, narrow) = (PI * (r + delta) * (r + delta), PI * r * r);
    let rest = wide + TAU * (r + delta) * h;
    for (master, top) in [("PCURVE_S1", wide), ("PCURVE_S2", wide), ("CURVE_3D", narrow)] {
        let a = tolerant_drum(r, r + delta, h, delta, master, 0).unwrap_or_else(|e| panic!("{master}: refused {e:?}"));
        close(&format!("a top whose master is {master}: area"), a, rest + top, 1e-9);
    }
}

/// **A master its edge's other representations stand apart from is set aside**: the cylinder of
/// radius `r`, its top edge the circle of radius `r` in space and the side's line `v = h` (exact),
/// the top plane's curve the circle of radius `r + δ` and the master `.PCURVE_S1.`. The side's
/// pcurve follows the 3D curve, not the master: read by the master, the top would be bounded by
/// a circle the side does not meet, the solid open by a ring of width `δ`, and its tolerance `δ`
/// measured only because that one reading made it. The edge is read by its 3D curve, the top
/// `π r²`. (CADGenBench's `.PCURVE_S1.` stands so, 201's edge #6220 6.7e-2 mm off a 3D curve and
/// a second pcurve 6.4e-4 apart: read by it, face #6071 read a hundredth under OCCT and the
/// measurer, both of which follow the other two.)
#[test]
fn a_master_its_edges_other_curves_stand_apart_from_is_set_aside() {
    let (r, h, delta) = (10., 20., 0.01);
    let a = tolerant_drum(r, r, h, delta, "PCURVE_S1", 0).unwrap_or_else(|e| panic!("refused {e:?}"));
    close("a master standing apart: area", a, TAU * r * r + TAU * r * h, 1e-9);
}

/// **A master that is not the edge is no master**: the top edge's master `.PCURVE_S1.` names the
/// circle of radius `r + δ` in a plane's parameters but says it lies on another surface than the
/// top's — the bottom plane, which no face using the edge lies on (its image 20 mm under the edge),
/// or the side, the edge's other face (its image wound round the cylinder, nowhere near the edge's
/// vertex). Either is plainly another curve than the edge, farther from its 3D curve and its
/// vertex than `sound::LOOSE` of the solid's size, past which a reading says nothing: trusting it
/// would read a solid the file does not describe. It is set aside and the edge read by its 3D
/// curve, as where its master is that — the curve that runs between the edge's vertices and lies
/// on both its faces — the top `π r²`, the side bounded by its own exact line.
#[test]
fn a_master_pcurve_on_another_surface_is_set_aside() {
    let (r, h, delta) = (10., 20., 0.01);
    for on in [2, 1] {
        let a = tolerant_drum(r, r, h, delta, "PCURVE_S1", on).unwrap_or_else(|e| panic!("on {on}: refused {e:?}"));
        close(&format!("a master pcurve on surface {on}: area"), a, TAU * r * r + TAU * r * h, 1e-9);
    }
}
