//! Language diagnostic codes and source locations.

use crate::syntax::{line_col, Span, StmtId};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Note,
    Warning,
    Error,
}

/// A spec §16 code, plus the ones this implementation adds.
///
/// A *code* is what a front end can act on; a message is for a reader and may be reworded.  The
/// `E1xx` block is ours: the spec numbers the errors a language has, and these are the ones a
/// language over *this* model has as well.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Code {
    /// redeclaration within a body
    E001,
    /// a block's index or edge binder over a name already in scope (§12)
    E002,
    /// a component that instantiates itself, directly or through others (§8)
    E003,
    /// an argument a formal list would silently take for another: a positional one after a
    /// labelled one, or a number given by position (§4.1)
    E004,
    /// a `ring`'s index read in its body (§11, §12.3): every copy is the representative turned,
    /// so there is nothing an index could vary
    E015,
    /// `next` or `prev` where no `cycle` closes the copies (§12.1)
    E020,
    /// a `ring` reading what its turn would move: an entity outside it other than its centre, a
    /// circle about it, or a plane a point is drawn in — or a copy reached by index (§12.5)
    E021,
    /// a `ring` inside a `ring` (§12.6)
    E022,
    /// what a `ring` cannot turn (§12.3): a plane, an axis or a motion declared in it, a point
    /// outside its centre's view, a centre that is neither a point nor an axis held in its
    /// direction, a hold or a curve contact on a turned copy
    E023,
    /// two gauges that disagree (§13, §9.6): a number held at two values, or three points
    /// oriented both ways — applied as read, the later would silently win
    E031,
    /// type mismatch within an alias class
    E040,
    /// a cyclic definitional dependency: a value defined in terms of itself, a group nested in
    /// itself, a body made of itself (§11)
    E041,
    /// a point given two planes (§6.7)
    E060,
    /// a `project` the model refuses: a point on no plane, both on one, or parallel planes
    /// (§6.7) — the core's own words, given a span
    E061,
    /// a word across views that has no meaning in space (§9.2): `horizontal`, a run or a rise, a
    /// tangency between drawn figures, a curve's contacts; or a relation across views naming a
    /// point of a 2D sketch, which has no place in space
    E062,
    /// a relation in space that came out degenerate at the solve (§6.7): two planes a `project`
    /// relates that came out parallel — they share no fold line — or two lines whose skew
    /// distance (or a cylinder's tangency) is stated that came out parallel
    E065,
    /// a plane whose axes cannot pass through its origin (§6.7): held axes that do not meet, or
    /// a plane held where its held axes are not
    E067,
    /// a `use` nothing resolves (§14.4)
    E070,
    /// a component defined twice, across the document and its modules (§14.4)
    E071,
    /// a face that is not a loop on one plane (§6.8)
    E080,
    /// a revolution's axis: not a line, or not in the face's own plane (§6.9)
    E081,
    /// a face of a body that the body no longer has (§6.9)
    E082,
    /// a section whose cutting plane is not parallel to the view it is drawn in (§6.11)
    E084,
    /// syntax
    E100,
    /// no such name
    E101,
    /// not a constraint type
    E102,
    /// not a shape the model can build
    E103,
    /// longer than the model will hold
    E104,
    /// a `fix` of a number the entity does not have
    E105,
    /// not yet: a construct the language has and elaboration does not
    E106,
    /// a measurement of the solved drawing (`length(l)`) read where the number is needed before
    /// the solve: a `param`, a seed, a constraint's number, a solid's extent or placement angle.
    /// Only a motion's `ratio:`, `phase:` and `advance:` are read after it (§6.14)
    E107,
    /// an expression that would not compute — the last number stands
    W110,
    /// a declaration over a built-in name (§3.3, §5): the built-in is what an expression reads
    W112,
    /// two planes lying on one another, each with geometry drawn in it: one plane in space
    /// (§6.7, `docs/planes-plan.md`)
    W113,
    /// a free curve whose length nothing holds and no length makes its energy stationary in: it
    /// has no shape to settle on (§9.10)
    W114,
}

impl Code {
    pub fn as_str(self) -> &'static str {
        match self {
            Code::E001 => "E001",
            Code::E002 => "E002",
            Code::E003 => "E003",
            Code::E004 => "E004",
            Code::E015 => "E015",
            Code::E020 => "E020",
            Code::E021 => "E021",
            Code::E022 => "E022",
            Code::E023 => "E023",
            Code::E031 => "E031",
            Code::E040 => "E040",
            Code::E041 => "E041",
            Code::E060 => "E060",
            Code::E061 => "E061",
            Code::E062 => "E062",
            Code::E065 => "E065",
            Code::E067 => "E067",
            Code::E070 => "E070",
            Code::E071 => "E071",
            Code::E080 => "E080",
            Code::E081 => "E081",
            Code::E082 => "E082",
            Code::E084 => "E084",
            Code::E100 => "E100",
            Code::E101 => "E101",
            Code::E102 => "E102",
            Code::E103 => "E103",
            Code::E104 => "E104",
            Code::E105 => "E105",
            Code::E106 => "E106",
            Code::E107 => "E107",
            Code::W110 => "W110",
            Code::W112 => "W112",
            Code::W113 => "W113",
            Code::W114 => "W114",
        }
    }

    pub fn severity(self) -> Severity {
        match self {
            Code::W110 | Code::W112 | Code::W113 | Code::W114 => Severity::Warning,
            _ => Severity::Error,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Diag {
    pub code: Code,
    pub span: Span,
    pub stmt: Option<StmtId>,
    pub message: String,
}

impl Diag {
    pub fn severity(&self) -> Severity {
        self.code.severity()
    }

    /// 1-based line and column, against the program the span indexes.
    pub fn at(&self, text: &str) -> (u32, u32) {
        line_col(text, self.span.lo)
    }
}
