//! How a recipe's numbers are written (`solid::cad`): in millimetres, about the world's origin or
//! about another. A kernel building a solid far from the world's origin builds it about its own
//! instead (`solid::cad::recipe_about`), where a double still carries the solid's sizes (a part at
//! 1e9 mm is good to 1e-7 mm there). Every writer of a node writes its points through one `Writer`,
//! so a node is written about the origin asked for as it is written at all.
use crate::brep::geom::V;
use crate::json::Json;

/// `scale` millimetres a model unit, and — where a recipe is written about another origin than the
/// world's — that origin (millimetres): every point the recipe carries less `about`, every direction
/// as it was, and a point that only places a plane or a line (a profile's origin, a revolution's)
/// put where its plane or line comes nearest the new origin: moved within it, it says the same, and
/// the frames built about it keep parameters of the solid's own size.
#[derive(Clone,Copy,Debug)]
pub struct Writer { pub scale: f64,pub about: Option<V> }

impl Writer {
    /// About the world's origin.
    pub fn world(scale: f64) -> Writer { Writer {scale,about:None} }
    /// About `about` (millimetres).
    pub fn about(scale: f64,about: V) -> Writer { Writer {scale,about:Some(about)} }
    /// Three numbers, as they stand: a direction.
    pub fn triple(v: V) -> Json { Json::Arr(v.into_iter().map(Json::from).collect()) }
    /// A point already in millimetres, about the origin written about.
    pub fn moved(&self,p: V) -> V {
        match self.about { Some(by) => std::array::from_fn(|k| p[k]-by[k]),None => p }
    }
    /// A point already in millimetres, written about the origin written about.
    pub fn at(&self,p: V) -> Json { Writer::triple(self.moved(p)) }
    /// A point of the model (its units), written.
    pub fn point(&self,p: V) -> Json { self.at(p.map(|x| x*self.scale)) }
    /// A length of the model (its units), written.
    pub fn length(&self,x: f64) -> Json { (x*self.scale).into() }
    /// The point (millimetres) placing the plane through `o` square to `normal`, written.
    pub fn plane_origin(&self,o: V,normal: V) -> Json { self.nearest(o,normal,true) }
    /// The point (millimetres) placing the line through `o` along `along`, written.
    pub fn line_origin(&self,o: V,along: V) -> Json { self.nearest(o,along,false) }
    /// `o` put where the plane through it square to `d` (or the line through it along `d`) comes
    /// nearest the origin written about; `o` itself, unmoved, where `d` is nothing.
    fn nearest(&self,o: V,d: V,plane: bool) -> Json {
        let Some(by) = self.about else { return Writer::triple(o) };
        let moved: V = std::array::from_fn(|k| o[k]-by[k]);
        let dd: f64 = d.iter().map(|x| x*x).sum();
        if !(dd > 0.) { return Writer::triple(o) }
        let t = (0..3).map(|k| moved[k]*d[k]).sum::<f64>()/dd;
        Writer::triple(std::array::from_fn(|k| if plane { t*d[k] } else { moved[k]-t*d[k] }))
    }
    /// A row-major 3x4 placement (its translation in millimetres), written: about the origin written
    /// about, `t + R by − by`.
    pub fn matrix(&self,m: [f64;12]) -> Json {
        let mut moved = m;
        if let Some(by) = self.about {
            for r in 0..3 { moved[4*r+3] = m[4*r+3]+(0..3).map(|c| m[4*r+c]*by[c]).sum::<f64>()-by[r]; }
        }
        Json::Arr(moved.into_iter().map(Json::from).collect())
    }
}
