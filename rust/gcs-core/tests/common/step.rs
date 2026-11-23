//! A STEP file written by hand, entity by entity (`Ents`): what the reader's tests of the kinds a
//! file may hold (`step_kinds`) and the writer's of the solids it reads (`step_rewrite`) build
//! their files with.

type V = [f64; 3];

/// A real as a STEP file writes it.
pub fn real(x: f64) -> String { format!("{x:.17E}") }

pub fn reals(x: &[f64]) -> String { x.iter().map(|&x| real(x)).collect::<Vec<_>>().join(",") }

pub fn logical(b: bool) -> &'static str { if b { ".T." } else { ".F." } }

/// A file's entities, numbered from 100 as they are added; the solid `#51`, its shell `#50`.
pub struct Ents { next: u32, text: String }

impl Ents {
    pub fn new() -> Ents { Ents { next: 100, text: String::new() } }
    pub fn add(&mut self, s: impl AsRef<str>) -> u32 {
        let n = self.next;
        self.next += 1;
        self.text += &format!("#{n} = {};\n", s.as_ref());
        n
    }
    pub fn point(&mut self, p: V) -> u32 { self.add(format!("CARTESIAN_POINT('',({}))", reals(&p))) }
    pub fn flat_point(&mut self, p: [f64; 2]) -> u32 { self.add(format!("CARTESIAN_POINT('',({}))", reals(&p))) }
    pub fn dir(&mut self, d: V) -> u32 { self.add(format!("DIRECTION('',({}))", reals(&d))) }
    pub fn place(&mut self, o: V, z: V, x: V) -> u32 {
        let (o, z, x) = (self.point(o), self.dir(z), self.dir(x));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})"))
    }
    pub fn vertex(&mut self, p: V) -> u32 { let p = self.point(p); self.add(format!("VERTEX_POINT('',#{p})")) }
    /// The line through `p` along `d`, its vector of length `m`.
    pub fn line(&mut self, p: V, d: V, m: f64) -> u32 {
        let (p, d) = (self.point(p), self.dir(d));
        let v = self.add(format!("VECTOR('',#{d},{})", real(m)));
        self.add(format!("LINE('',#{p},#{v})"))
    }
    pub fn plane(&mut self, o: V, z: V, x: V) -> u32 { let a = self.place(o, z, x); self.add(format!("PLANE('',#{a})")) }
    pub fn circle(&mut self, o: V, z: V, x: V, r: f64) -> u32 { let a = self.place(o, z, x); self.add(format!("CIRCLE('',#{a},{})", real(r))) }
    pub fn ellipse(&mut self, o: V, z: V, x: V, a: f64, b: f64) -> u32 {
        let p = self.place(o, z, x);
        self.add(format!("ELLIPSE('',#{p},{},{})", real(a), real(b)))
    }
    /// The extrusion of `curve` along `d`.
    pub fn extrusion(&mut self, curve: u32, d: V) -> u32 {
        let d = self.dir(d);
        let v = self.add(format!("VECTOR('',#{d},1.)"));
        self.add(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{curve},#{v})"))
    }
    /// The revolution of `curve` about the `z` axis.
    pub fn revolution(&mut self, curve: u32) -> u32 {
        let (o, z) = (self.point([0.; 3]), self.dir([0., 0., 1.]));
        let a = self.add(format!("AXIS1_PLACEMENT('',#{o},#{z})"));
        self.add(format!("SURFACE_OF_REVOLUTION('',#{curve},#{a})"))
    }
    pub fn edge(&mut self, a: u32, b: u32, curve: u32, sense: bool) -> u32 { self.add(format!("EDGE_CURVE('',#{a},#{b},#{curve},{})", logical(sense))) }
    /// A face of a loop of `(edge, along)`, its `FACE_BOUND` `turned` or not.
    pub fn face(&mut self, uses: &[(u32, bool)], turned: bool, surface: u32, sense: bool) -> u32 {
        let l = self.edge_loop(uses);
        let b = self.add(format!("FACE_BOUND('',#{l},{})", logical(!turned)));
        self.add(format!("ADVANCED_FACE('',(#{b}),#{surface},{})", logical(sense)))
    }
    /// A face of a loop of `(edge, along)` written as its `FACE_OUTER_BOUND`.
    pub fn outer_face(&mut self, uses: &[(u32, bool)], surface: u32, sense: bool) -> u32 {
        let l = self.edge_loop(uses);
        let b = self.add(format!("FACE_OUTER_BOUND('',#{l},.T.)"));
        self.add(format!("ADVANCED_FACE('',(#{b}),#{surface},{})", logical(sense)))
    }
    fn edge_loop(&mut self, uses: &[(u32, bool)]) -> u32 {
        let uses: Vec<String> = uses.iter().map(|&(e, along)| format!("#{}", self.add(format!("ORIENTED_EDGE('',*,*,#{e},{})", logical(along))))).collect();
        self.add(format!("EDGE_LOOP('',({}))", uses.join(",")))
    }
    /// The file of the solid bounded by `faces`.
    pub fn file(self, faces: &[u32]) -> String {
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
