//! A motion's contact chart, the kernel's (`brep::swept::generating`, #721 F2c), said in the
//! drawing's words where the kernel refuses it.
use super::{Family,Step};
use crate::brep::swept::generating::Form;
use crate::envelope::SurfacePoint;

pub use crate::brep::swept::generating::NormalVelocity;

impl Family {
    /// The contact chart of `surface`'s point under this motion (`generating::Form`): a sinusoid in
    /// the roll under a turn or two relative turns, a line under a slide seen from a turn. Nested
    /// relative motions can contain additional harmonics and are refused, as are a drive's and a
    /// path's, by what the drawing says the motion is.
    pub fn normal_velocity(&self,surface: SurfacePoint) -> Result<NormalVelocity,String> {
        Form::of(self.path()?).map_err(|_| self.uncharted())?.normal_velocity(surface).map_err(|e| e.to_string())
    }

    /// Why this motion has no contact chart (the kernel's `Uncharted`), as the drawing says it: what
    /// its root step is, then whether a step is driven by a law, then which step is no turn.
    pub(crate) fn uncharted(&self) -> String {
        let (source,observer) = match *self.root() {
            Step::Rotation {..} => (self.root(),None),
            Step::Relative {source,observer} => (&self.steps[source],Some(&self.steps[observer])),
            Step::Translation {..} => return "temporal contacts require a rotation or two relative rotations".into(),
            Step::Guided {..} => return "temporal contacts are read of constant rates, and a \
                frame along a path turns as its path does (#398)".into(),
            Step::Followed {..} => return "temporal contacts are read of constant rates, and a \
                frame the drawing moves turns as the drawing does (#398)".into(),
        };
        if self.steps.iter().any(Step::is_driven) {
            return "temporal contacts are read of constant rates, and a drive's law runs \
                at its own (#391): a joint driven alone is swept as the joint over the \
                values its law reaches (#684 L), and one seen from another motion is no \
                constant twist".into()
        }
        if matches!(source,Step::Translation {..}) && observer.is_some_and(|o| !matches!(o,Step::Rotation {advance:0.,..})) {
            return "a translation's temporal contacts require it be seen from a rotation without advance".into()
        }
        "temporal contacts require a rotation or two relative rotations without advance".into()
    }
}
