//! What a material field reads at a point: its value, which way it rises, and which operand
//! decides it — for a mesher that wants more than a sign. In plain floating point, like `side`:
//! a reading, never an interval claim.
//!
//! The field is one-Lipschitz, so |value| is a lower bound on the distance to the boundary, and
//! it is smooth wherever one operand decides it: there the gradient is that operand's own — a
//! static leaf's by central differences, a sweep's the tool's at the roll time it is least at,
//! turned into the world by that pose. Where two operands, or two contact times of one sweep,
//! read within `tie` of each other the point is at or beside a crease: the reading says so, and
//! its gradient is one side's.

/// A reading at a point.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Reading {
    pub value: f64,
    pub gradient: [f64;3],
    /// The leaf deciding the value, numbered depth-first over the whole field, a sweep's source
    /// leaves among them.
    pub leaf: usize,
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
    /// The central-difference step for a static leaf's gradient.
    pub step: f64,
    /// A warm reading continues each sweep's contact from its hint without searching the whole
    /// roll (`SweptField::minimum_hinted`): a continuation, which a caller must check otherwise.
    pub local: bool,
}

impl ReadingOptions {
    /// Scaled to the point's size: a sweep's minimum to a ten-billionth of it (as `side` stops) or
    /// a thousandth of its own value, a tie at a millionth, a difference step at a ten-millionth.
    pub fn at(p: [f64;3]) -> Self {
        let size = 1.+(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]).sqrt();
        Self {accuracy:1e-10*size,relative:1e-3,tie:1e-6*size,step:1e-7*size,local:false}
    }
}

impl Reading {
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

/// A leaf's reading: its value, and its gradient by central differences.
pub(super) fn leaf(value: impl Fn([f64;3]) -> f64,p: [f64;3],step: f64,index: usize) -> Reading {
    let gradient = std::array::from_fn(|k| {
        let (mut up,mut down) = (p,p);
        up[k] += step;
        down[k] -= step;
        (value(up)-value(down))/(2.*step)
    });
    Reading {value:value(p),gradient,leaf:index,time:None,ambiguous:false}
}
