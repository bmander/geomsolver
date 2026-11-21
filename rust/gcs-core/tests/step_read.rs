//! **The kernel reads STEP** (`brep::step_read`; #993 stage 1): every file the kernel writes
//! reads back to the same solid, each face on its own surface and each edge on its own curve.
