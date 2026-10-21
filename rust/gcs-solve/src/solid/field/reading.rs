//! What a material field reads at a point: its value, which way it rises, and which operand
//! decides it — for a mesher that wants more than a sign — and the query that asks for one
//! (`Query`). In plain floating point, like a sign: a reading, never an interval claim.
//!
//! The field is one-Lipschitz, so |value| is a lower bound on the distance to the boundary, and
//! it is smooth wherever one operand decides it: there the gradient is that operand's own — a
//! static leaf's by forward differences, a sweep's the tool's at the roll time it is least at,
//! turned into the world by that pose. Where two operands, or two contact times of one sweep,
//! read within `tie` of each other the point is at or beside a crease: the reading says so, and
//! its gradient is one side's.

/// Which operand decides a reading: a piece of a leaf's boundary (a face of a prism, a turned edge
/// of a revolution), and for a leaf of a sweep the roll time of its contact — or a leaf whole,
/// its own trimmed value rather than one piece's untrimmed carrier. Read off a reading and handed
/// back to `MaterialField::operand`; what the numbers are is the field's business.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct OperandId {
    /// The leaf, numbered depth-first over the whole field, a sweep's source leaves among them.
    leaf: usize,
    /// The piece of that leaf's boundary (`PlanarField::piece_count`), or none for the leaf whole.
    piece: Option<usize>,
    /// The roll time a sweep decides the value at.
    time: Option<f64>,
}

impl OperandId {
    /// Piece `piece` of leaf `leaf`, at no roll time: for a source of readings other than a
    /// material field (`crease::CreaseSource`), which numbers its own.
    pub fn new(leaf: usize,piece: usize) -> Self { Self {leaf,piece:Some(piece),time:None} }
    /// The leaf, as the field numbers them.
    pub fn leaf(&self) -> usize { self.leaf }
    /// The piece of the leaf's boundary, or none for the leaf whole.
    pub fn piece(&self) -> Option<usize> { self.piece }
    /// The roll time of a sweep's contact.
    pub fn time(&self) -> Option<f64> { self.time }
    /// The leaf this operand is a piece of, whole: what the field reads of it, trimmed.
    pub fn whole(self) -> Self { Self {piece:None,..self} }
    /// The same operand at another contact time (a sweep's contact followed to a new point).
    pub fn at_time(self,time: Option<f64>) -> Self { Self {time,..self} }
    /// Both are of one leaf.
    pub fn same_leaf(&self,other: &Self) -> bool { self.leaf == other.leaf }
    /// Two operands are different: other pieces, or one sweep's contacts `gap` apart.
    pub fn differs(self,other: Self,gap: f64) -> bool {
        self.leaf != other.leaf || self.piece != other.piece || match (self.time,other.time) {
            (Some(s),Some(t)) => (s-t).abs() > gap,
            _ => false,
        }
    }
}

/// A reading at a point.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Reading {
    pub value: f64,
    pub gradient: [f64;3],
    /// The operand deciding the value, where the reading was made from one: none for a sign, a
    /// bound that settled an operand unread, or a value interpolated from a sweep's adaptive
    /// distance field, none of which knows what decides the field there.
    pub operand: Option<OperandId>,
    /// Another operand or another contact time reads within the tie: a crease.
    pub ambiguous: bool,
}

/// What a point query asks of a material field (`MaterialField::query`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Want {
    /// A number with the field's sign, for a mesher that asks only which side a point is on: its
    /// magnitude only an upper bound on the field, and no gradient or operand.
    Sign,
    /// The value, its gradient and the deciding operand.
    Reading,
}

/// Where a sweep's value comes from.
#[derive(Clone,Debug,PartialEq)]
pub enum Source {
    /// A search over the roll.
    Exact,
    /// The sweep's adaptive distance field refined to this (`SweptField::cached`): a mesher's
    /// reading, off the exact field by about the resolution's tolerance, and naming no operand.
    Cached(super::Resolution),
    /// A search warm-started from a nearby point's: `hints[k]` is the contact time of the sweep
    /// numbered `k` there, and is left holding this point's; an empty vector is a cold reading,
    /// and a hint from far away costs time, never the answer. `local` continues each contact from
    /// its hint without searching the whole roll (`SweptField::minimum_hinted`): a continuation,
    /// which a caller must check otherwise.
    Warm {hints: Vec<Option<f64>>,local: bool},
}

/// A point query of a material field, and how closely it is made.
#[derive(Clone,Debug,PartialEq)]
pub struct Query {
    pub want: Want,
    /// A sweep's minimum is found to this, or to `relative` of its own size where that is coarser.
    pub accuracy: f64,
    pub relative: f64,
    /// Two operands or contact times this close are a crease.
    pub tie: f64,
    /// The finite-difference step for a static leaf's gradient.
    pub step: f64,
    pub source: Source,
}

impl Query {
    /// A reading scaled to the point's size: a sweep's minimum to a ten-billionth of it (as a sign
    /// stops) or a thousandth of its own value, a tie at a millionth, a difference step at a
    /// ten-millionth, every sweep searched.
    pub fn at(p: [f64;3]) -> Self {
        let size = 1.+crate::space::norm(p);
        Self {want:Want::Reading,accuracy:1e-10*size,relative:1e-3,tie:1e-6*size,step:1e-7*size,source:Source::Exact}
    }
    /// The field's sign, every sweep searched. A sign reads none of the tolerances.
    pub fn sign() -> Self { Self {want:Want::Sign,..Self::at([0.;3])} }
}

impl Reading {
    /// A value alone: an operand left unread because a bound settled it (`value` at least, with
    /// no gradient worth reading), or a sign.
    pub(super) fn bound(value: f64) -> Self { Self {value,gradient:[0.;3],operand:None,ambiguous:false} }
    /// The roll time of the deciding operand's contact, where a sweep decides.
    pub fn time(&self) -> Option<f64> { self.operand.and_then(|o| o.time) }
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
    Reading {value:v,gradient,operand:Some(OperandId::new(index,piece)),ambiguous:false}
}
