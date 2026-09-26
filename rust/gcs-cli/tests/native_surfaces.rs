//! Bodies with continuous swept cuts, built by the same native construction solventc uses.
use std::{ffi::c_int,path::Path};

use fixtures::gear::read;
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
#[path="native_surfaces/gear_cells.rs"]
mod gear_cells;

struct Cad(native::Session);
impl Cad {
    fn new() -> Self { Self(native::Session::new().unwrap()) }
}
