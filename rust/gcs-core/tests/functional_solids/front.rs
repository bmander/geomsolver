//! General-field advancing-front workbench. No analytic surface charts or
//! geometry-specific boundary pieces. Sampling/shape controls are experimental;
//! this is not yet a whole-boundary accuracy or component-coverage certificate.
use super::*;
use gcs_core::{interval::minimum::Options,solid::{MaterialField,MaterialEvaluator}};
use std::collections::{BTreeMap,BTreeSet,VecDeque};

mod geometry;
use geometry::*;
mod field;
use field::Surface;
mod mesh;
use mesh::Front;
mod features;
mod quality;
mod clearance;
mod cases;
