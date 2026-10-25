//! **Developable panels in the model** (#645): the rails a panel is built between, as the drawing
//! names them, and a rail of it laid flat as a curve of the drawing.

use super::*;

/// A rail of a panel as the drawing names it: one path (a line, an arc, a circle, a spline, a
/// curve, a curve traced in space) or a named chain's links in order — closed where it comes
/// round — run the other way where `reversed` (`sense: opposite`).
#[derive(Clone, Debug, PartialEq)]
pub struct Rail {
    pub links: Vec<EntRef>,
    pub chain: bool,
    pub closed: bool,
    pub reversed: bool,
}

impl Rail {
    /// The rail read as a path, as the drawing stands.
    pub fn path(&self, sk: &Sketch) -> Result<crate::path::Path, String> {
        let p = match (self.chain, self.links.as_slice()) {
            (false, [g]) => crate::path::Path::read(sk, *g)?,
            (_, links) => crate::path::Path::chain(sk, links, self.closed)?,
        };
        Ok(if self.reversed { p.backwards() } else { p })
    }

    /// The same rail over the entities `map` renumbers its links to, or `None` where one went.
    pub fn remapped(&self, map: &dyn Fn(EntRef) -> Option<EntRef>) -> Option<Rail> {
        let links = self.links.iter().map(|&e| map(e)).collect::<Option<Vec<_>>>()?;
        Some(Rail { links, ..self.clone() })
    }
}

/// **A panel's rail laid flat** (`flat.a`, `flat.b`): the image of rail `which` (0 `a`, 1 `b`) of
/// the panel between `rails`, its layer `d` along the panel's normal, laid from the home ruling
/// through `s` on `a`; the curve's two arguments are the home ruling's ends where it is laid, its
/// columns.  `n_const` is its contacts' constants' width, fixed when it is built.
#[derive(Clone, Debug)]
pub struct Developed {
    pub rails: [Rail; 2],
    pub s: u32,
    pub d: f64,
    pub which: u8,
    pub n_const: usize,
}

impl Developed {
    /// Its contacts' constants as the drawing stands (`developable::consts`), or why the rails
    /// cannot be read.
    pub fn consts(&self, sk: &Sketch) -> Result<Vec<f64>, String> {
        let (a, b) = (self.rails[0].path(sk)?, self.rails[1].path(sk)?);
        Ok(crate::developable::consts(&a, &b, sk.world_point(self.s as usize), self.d, self.which))
    }
}

/// A laid rail's definition: over its two home ends, run in `t` from 0 to 1.
pub fn developed_def(name: String, dev: Developed) -> CurveDef {
    CurveDef {
        name,
        component: String::new(),
        port: String::new(),
        formals: vec![("f".into(), EntKind::Point), ("g".into(), EntKind::Point)],
        columns: Vec::new(),
        values: Vec::new(),
        param: "t".into(),
        turns: false,
        vars: ["t", "f.x", "f.y", "g.x", "g.y"].map(String::from).to_vec(),
        body: CurveBody::Developed(dev),
        pose_of: Vec::new(),
        home: false,
    }
}

impl Sketch {
    /// A laid rail's contacts' constants, read as the drawing now stands — `None` for another
    /// curve.
    pub fn developed_consts(&self, i: usize) -> Option<Vec<f64>> {
        let d = &self.curve_defs[self.curves.get(i)?.def as usize];
        let CurveBody::Developed(dev) = &d.body else { return None };
        let k = dev.consts(self).ok()?;
        (k.len() == dev.n_const).then_some(k)
    }

    /// The development curve `i` lays, built from the drawing as it stands — `None` for another
    /// curve, and the refusal where the panel cannot be built.
    pub fn development_of(&self, i: usize)
        -> Option<std::rc::Rc<Result<crate::developable::Development, crate::developable::Refusal>>> {
        crate::developable::developed(&self.developed_consts(i)?)
    }
}
