//! A file's geometry read into the kernel's: a curve in space (`geom::Curve`), a curve in a
//! face's parameters, and a surface (`geom::Surface`), each scaled from the file's units
//! (`context::Context`) and placed as the kernel places it. What the kernel has no curve or
//! surface for is refused naming the entity (`Refusal::unsupported`).
