//! What a material field reads at a point: its value, which way it rises, and which operand
//! decides it — for a mesher that wants more than a sign. In plain floating point, like `side`:
//! a reading, never an interval claim.
//!
//! The field is one-Lipschitz, so |value| is a lower bound on the distance to the boundary, and
//! it is smooth wherever one operand decides it: there the gradient is that operand's own — a
//! static leaf's by forward differences, a sweep's the tool's at the roll time it is least at,
//! turned into the world by that pose. Where two operands, or two contact times of one sweep,
//! read within `tie` of each other the point is at or beside a crease: the reading says so, and
//! its gradient is one side's.

/// A piece number standing for a leaf's whole boundary: its own value, trimmed, rather than one
/// piece's untrimmed carrier.
pub const WHOLE: usize = usize::MAX;

/// A reading at a point.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Reading {
    pub value: f64,
    pub gradient: [f64;3],
    /// The leaf deciding the value, numbered depth-first over the whole field, a sweep's source
    /// leaves among them.
    pub leaf: usize,
    /// The piece of that leaf's boundary deciding it (`PlanarField::piece_count`): a face, where
    /// the leaf is the whole solid.
    pub piece: usize,
    /// The roll time a sweep decides the value at.
    pub time: Option<f64>,
    /// Another operand or another contact time reads within the tie: a crease.
    pub ambiguous: bool,
}

/// How closely a reading is made.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct ReadingOptions {
    /// A sweep's minimum is found to this, or to `relative` of its own size where that is coarser.
    pub accuracy: f64,
    pub relative: f64,
    /// Two operands or contact times this close are a crease.
    pub tie: f64,
    /// The finite-difference step for a static leaf's gradient.
    pub step: f64,
    /// A warm reading continues each sweep's contact from its hint without searching the whole
    /// roll (`SweptField::minimum_hinted`): a continuation, which a caller must check otherwise.
    pub local: bool,
    /// Given, a sweep is read from its adaptive distance field refined to this
    /// (`SweptField::cached`) instead of searched: a mesher's reading, off the exact field by about
    /// the resolution's tolerance. `None` reads exactly.
    pub cached: Option<super::Resolution>,
}

impl ReadingOptions {
    /// Scaled to the point's size: a sweep's minimum to a ten-billionth of it (as `side` stops) or
    /// a thousandth of its own value, a tie at a millionth, a difference step at a ten-millionth.
    pub fn at(p: [f64;3]) -> Self {
        let size = 1.+(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]).sqrt();
        Self {accuracy:1e-10*size,relative:1e-3,tie:1e-6*size,step:1e-7*size,local:false,cached:None}
    }
}

impl Reading {
    /// An operand left unread because a bound settled it: `value` at least, numbered `leaf`, with
    /// no gradient worth reading.
    pub(super) fn bound(value: f64,leaf: usize,time: Option<f64>) -> Self {
        Self {value,gradient:[0.;3],leaf,piece:0,time,ambiguous:false}
    }
    pub(super) fn negated(self) -> Self {
        Self {value:-self.value,gradient:self.gradient.map(|g| -g),..self}
    }
}

/// The lower of two readings, a crease where they tie.
pub(super) fn lower(a: Reading,b: Reading,tie: f64) -> Reading {
    let near = (a.value-b.value).abs() <= tie;
    let mut r = if a.value <= b.value { a } else { b };
    r.ambiguous |= near;
    r
}

/// The higher of two readings, a crease where they tie.
pub(super) fn higher(a: Reading,b: Reading,tie: f64) -> Reading {
    lower(a.negated(),b.negated(),tie).negated()
}

/// A leaf's reading: its value, and its gradient by forward differences.
pub(super) fn leaf(value: impl Fn([f64;3]) -> f64,p: [f64;3],step: f64,index: usize,piece: usize) -> Reading {
    // forward differences: the step is a ten-millionth of the point's size, so the curvature's
    // share of the error is as small as central differences' roundoff, at half their evaluations
    let v = value(p);
    let gradient = std::array::from_fn(|k| {
        let mut up = p;
        up[k] += step;
        (value(up)-v)/step
    });
    Reading {value:v,gradient,leaf:index,piece,time:None,ambiguous:false}
}
