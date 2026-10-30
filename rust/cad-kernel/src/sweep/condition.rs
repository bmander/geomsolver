//! The rows of the swept classes (docs/generating-sweeps.md in the solver): what a sweep is asked
//! before a boundary is built for it, each with the code and the words a refusal names it by.
/// The rows of the class, in the order they are asked.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Condition { Tool, Corner, Motion, Stationary, Reach, Clearance, Single, Fold, Crossing, Sections, Regular, Once, Edgewise, Lead,
    Plane, Prisms, Period, Envelope, Wall, Members, Slabs, Closes, Kept, Curvature, Return }

impl Condition {
    pub fn code(self) -> &'static str {
        match self { Condition::Tool => "T1", Condition::Corner => "T2", Condition::Motion => "M1",
            Condition::Stationary => "M2", Condition::Reach => "E0", Condition::Clearance => "E1", Condition::Single => "E2",
            Condition::Fold => "E3", Condition::Crossing => "E4", Condition::Sections => "T3", Condition::Regular => "S1", Condition::Once => "S2",
            Condition::Edgewise => "S3", Condition::Lead => "S4", Condition::Plane => "P1", Condition::Prisms => "P2",
            Condition::Period => "P3", Condition::Envelope => "P4", Condition::Wall => "P5", Condition::Members => "Q1",
            Condition::Slabs => "Q2", Condition::Closes => "Q3", Condition::Kept => "Q4", Condition::Curvature => "B1",
            Condition::Return => "B2" }
    }
    /// What the row asks, in the words the scope document uses.
    pub fn rule(self) -> &'static str {
        match self {
            Condition::Tool => "the tool is a full revolution, or a prism (drafted or not), of lines and arcs, or an intersection \
                of such, its caps clear of the blank unless it is an undrafted prism standing alone, or a union of tools each in \
                the class",
            Condition::Corner => "every profile corner the sweep carries into the blank is convex or tangent",
            Condition::Motion => "the motion is rotations about fixed axes at constant ratios, or one screw, turn or slide (a joint driven by a law alone read as its joint over the values the law reaches)",
            Condition::Stationary => "the contact condition depends on the motion",
            Condition::Reach => "the sweep's contacts reach the blank",
            Condition::Clearance => "the tool is clear of the blank at both ends of the roll",
            Condition::Single => "each tool point touches the blank at most once in the roll",
            Condition::Fold => "the generated surface does not fold",
            Condition::Crossing => "the generated surface does not cross itself",
            Condition::Sections => "a prism whose cap reaches the blank is sectioned through its extrusion in a band clear of its \
                profile's corners and its ends",
            Condition::Regular => "the screw's characteristic is a regular curve through the blank",
            Condition::Once => "the characteristic reaches the blank in one stretch, one point a ring",
            Condition::Edgewise => "the characteristic never runs along the screw's path",
            Condition::Lead => "the sweep does not cross itself within a lead, or a turn",
            Condition::Plane => "the motion turns a plane in itself about axes square to it",
            Condition::Prisms => "the tool is a stock cut by one pocket, prisms square to that plane through the blank, the \
                pocket's profile one closed curve",
            Condition::Period => "the roll is a whole period of the motion",
            Condition::Envelope => "the pocket's inner envelope is one regular loop that neither folds nor crosses itself",
            Condition::Wall => "the stock's outer wall never reaches the blank",
            Condition::Members => "the tool is prisms square to the plane the motion keeps, of lines and arcs, joined by union and \
                common",
            Condition::Slabs => "every member's caps lie outside the blank, or on a plane the blank is split at",
            Condition::Closes => "the roll is a whole period of the motion, or the tool is clear of the blank at both its ends",
            Condition::Kept => "the swept region's boundary is regular: its pieces meet at corners or leave the blank's shadow, with no \
                fold and no crossing",
            Condition::Curvature => "the ball's centre's path never stops and turns nowhere tighter than its radius",
            Condition::Return => "the ball's centre's path comes back within its diameter of itself nowhere",
        }
    }
}
