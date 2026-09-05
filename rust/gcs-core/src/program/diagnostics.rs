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
    /// an argument a formal list would silently take for another: a positional one after a
    /// labelled one, or a number given by position (§4.1)
    E004,
    /// type mismatch within an alias class
    E040,
    /// a cyclic definitional dependency: a plane folded from itself (§6.7)
    E041,
    /// a point given two planes (§6.7)
    E060,
    /// a `project` the model refuses: a point on no plane, both on one, or parallel planes
    /// (§6.7) — the core's own words, given a span
    E061,
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
    /// a stack that contradicts itself, or a placed plane placed twice or never (§6.10)
    E083,
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
    /// `ground`/`fix` on something the document cannot express
    E105,
    /// not yet: a construct the language has and elaboration does not
    E106,
    /// an expression that would not compute — the last number stands
    W110,
    /// a free variable: which dimensions it ties together
    W111,
    /// a declaration over a built-in name (§3.3, §5): the built-in is what an expression reads
    W112,
}

impl Code {
    pub fn as_str(self) -> &'static str {
        match self {
            Code::E001 => "E001",
            Code::E004 => "E004",
            Code::E040 => "E040",
            Code::E041 => "E041",
            Code::E060 => "E060",
            Code::E061 => "E061",
            Code::E070 => "E070",
            Code::E071 => "E071",
            Code::E080 => "E080",
            Code::E081 => "E081",
            Code::E082 => "E082",
            Code::E083 => "E083",
            Code::E084 => "E084",
            Code::E100 => "E100",
            Code::E101 => "E101",
            Code::E102 => "E102",
            Code::E103 => "E103",
            Code::E104 => "E104",
            Code::E105 => "E105",
            Code::E106 => "E106",
            Code::W110 => "W110",
            Code::W111 => "W111",
            Code::W112 => "W112",
        }
    }

    pub fn severity(self) -> Severity {
        match self {
            Code::W110 | Code::W111 | Code::W112 => Severity::Warning,
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
