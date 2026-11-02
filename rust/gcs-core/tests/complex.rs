//! **The cell complex** (#735 A1, `brep::complex`): faces each parting two cells, edges each
//! bounding any number of faces in order round them. A touch along an edge or at a point, a void,
//! and faces tangent along an edge are states it holds, each held to arithmetic: its cells'
//! volumes, its check, and the solid a selection of its cells is.
use crate::brep::block;
use crate::common::close;
use gcs_core::brep::boolean::{arrangement,combined,Op};
use gcs_core::brep::build::{disc,prism};
use gcs_core::brep::complex::{Complex,Touch};
use gcs_core::brep::props::volume;
use std::f64::consts::PI;

/// The bounded cells' volumes, smallest first.
pub(crate) fn volumes(c: &Complex) -> Vec<f64> {
    let mut v: Vec<f64> = c.cells().unwrap()[1..].iter().map(|k| k.volume).collect();
    v.sort_by(f64::total_cmp);
    v
}

/// The bounded cell of volume `v`.
fn cell_of(c: &Complex,v: f64) -> usize {
    (1..c.cells().unwrap().len()).find(|&k| (c.cells().unwrap()[k].volume-v).abs() <= 1e-9*v.max(1.)).unwrap()
}

/// **Two blocks meeting along an edge are two cells round one edge of four faces**: the complex
/// their arrangement is holds it (no B-rep check could); each block's cell its volume, and the two
/// selected together a valid boundary of both, saying it touches itself along that edge.
#[test]
fn two_blocks_meeting_along_an_edge_are_two_cells_round_one_edge() {
    let (a,b) = (block([0.;3],[1.,1.,1.]),block([1.,1.,0.],[3.,2.,1.]));
    let c = arrangement(&[&a,&b],1e-9).unwrap();
    c.check(1e-9).unwrap();
    assert_eq!(c.cells().unwrap().len(),3);
    let v = volumes(&c);
    close(v[0],1.);
    close(v[1],2.);
    let four: Vec<_> = c.radial.iter().filter(|r| r.len() == 4).collect();
    assert_eq!(four.len(),1,"one edge of four faces");
    // round it, the wedges are the outside, a block, the outside and the other block
    let cells: Vec<usize> = four[0].iter().map(|u| c.cell_of_shell(u.ahead).unwrap()).collect();
    assert_eq!(cells.iter().filter(|&&k| k == 0).count(),2,"{cells:?}");
    let both = c.select(&|k| k != 0).unwrap();
    both.brep.check(1e-9).unwrap();
    close(volume(&both.brep),3.);
    assert!(matches!(both.touches[..],[Touch::Along {faces:4,..}]),"{:?}",both.touches);
    // one cell alone touches nothing
    let one = c.select(&|k| k == cell_of(&c,1.)).unwrap();
    one.brep.check(1e-9).unwrap();
    close(volume(&one.brep),1.);
    assert!(one.touches.is_empty());
}

/// **Two blocks meeting at a corner are two cells pinched at a point**, the union saying so.
#[test]
fn two_blocks_meeting_at_a_corner_are_pinched_there() {
    let (a,b) = (block([0.;3],[1.,1.,1.]),block([1.,1.,1.],[2.,2.,2.]));
    let c = arrangement(&[&a,&b],1e-9).unwrap();
    c.check(1e-9).unwrap();
    assert_eq!(volumes(&c).len(),2);
    let both = c.select(&|k| k != 0).unwrap();
    both.brep.check(1e-9).unwrap();
    close(volume(&both.brep),2.);
    assert!(matches!(both.touches[..],[Touch::At(p)] if (p[0]-1.).abs()+(p[1]-1.).abs()+(p[2]-1.).abs() < 1e-9),"{:?}",both.touches);
}

/// **A void is a shell of the cell holding it**: a block with a block cut from its middle is three
/// cells — the outside, the material with its void, and the void — and selections of them are the
/// solid, the void's block and the whole block.
#[test]
fn a_void_is_a_shell_of_the_cell_holding_it() {
    let (outer,inner) = (block([0.;3],[4.,4.,4.]),block([1.;3],[2.,3.,2.]));
    let hollow = combined(&outer,&inner,Op::Cut,0.).unwrap();
    let c = Complex::of(&hollow,1e-9).unwrap();
    c.check(1e-9).unwrap();
    let v = volumes(&c);
    assert_eq!(v.len(),2);
    close(v[0],2.);
    close(v[1],62.);
    let (material,void) = (cell_of(&c,62.),cell_of(&c,2.));
    assert_eq!(c.cells().unwrap()[material].shells.len(),2,"the material's outer shell and its void");
    assert_eq!(c.cell_at([0.5,0.5,0.5]),Some(material));
    assert_eq!(c.cell_at([1.5,2.,1.5]),Some(void));
    assert_eq!(c.cell_at([5.,0.,0.]),Some(0));
    let solid = c.select(&|k| k == material).unwrap();
    solid.brep.check(1e-9).unwrap();
    close(volume(&solid.brep),62.);
    let whole = c.select(&|k| k != 0).unwrap();
    whole.brep.check(1e-9).unwrap();
    close(volume(&whole.brep),64.);
    assert_eq!(whole.brep.faces.len(),6);
    close(volume(&c.select(&|k| k == void).unwrap().brep),2.);
}

/// **Faces tangent along an edge are ordered round it by how they part a step in**: a rod lying on
/// a block's top, tangent to it along a line, splits the top along that line in the arrangement
/// (a touch represented, not dropped), and round that line the top's two pieces and the rod's face
/// both ways are tangent pairs, ordered by their curvatures' difference a step in — so the rod's
/// cell, the block's and the outside are each one cell, and the two united say they touch there.
#[test]
fn faces_tangent_along_an_edge_are_ordered_by_their_curvature() {
    let block_ = block([-2.,-2.,-1.],[2.,2.,0.]);
    // a rod of radius 1 along x on the block's top, touching it along the line y = 0, z = 0
    let rod = prism(&disc([-1.,0.,1.],[1.,0.,0.],1.),0.,2.).unwrap();
    let c = arrangement(&[&block_,&rod],1e-9).unwrap();
    c.check(1e-9).unwrap();
    let v = volumes(&c);
    assert_eq!(v.len(),2,"{v:?}");
    close(v[0],2.*PI);
    close(v[1],16.);
    let four: Vec<_> = c.radial.iter().filter(|r| r.len() > 2).collect();
    assert_eq!(four.len(),1,"the line they touch along");
    assert_eq!(four[0].len(),4);
    let both = c.select(&|k| k != 0).unwrap();
    both.brep.check(1e-9).unwrap();
    close(volume(&both.brep),16.+2.*PI);
    assert!(matches!(both.touches[..],[Touch::Along {faces:4,..}]),"{:?}",both.touches);
}

/// **Two surfaces of revolution touching along a circle are cells parted by it** (#735's touch
/// cases: what a manifold-only B-rep cannot hold): a ball in a cylinder of its own radius touches it
/// round its equator, laid as an edge of both, so the cylinder less the ball is two cusp-shaped cells
/// of `4π/3` each, beside the ball's — and selected together they are a valid boundary saying it
/// touches itself along that circle.
#[test]
fn a_ball_in_a_cylinder_of_its_radius_touches_it_along_a_circle() {
    use gcs_core::brep::build::sphere;
    let cylinder = prism(&disc([0.,0.,-2.],[0.,0.,1.],1.),0.,4.).unwrap();
    let ball = sphere([0.;3],[0.,0.,1.],1.).unwrap();
    let c = arrangement(&[&cylinder,&ball],1e-9).unwrap();
    c.check(1e-9).unwrap();
    let v = volumes(&c);
    assert_eq!(v.len(),3,"{v:?}");
    for x in v { close(x,4.*PI/3.); }
    let rest = gcs_core::brep::boolean::selected(&[&cylinder,&ball],&|m| m[0] && !m[1],1e-9).unwrap();
    rest.brep.check(1e-9).unwrap();
    close(volume(&rest.brep),4.*PI-4.*PI/3.);
    assert!(matches!(rest.touches[..],[Touch::Along {faces:4,..}]),"{:?}",rest.touches);
}

/// **A plane tangent to a cone along a generator touches it along that line**: a block whose top
/// holds the cone's lowest generator, the touch laid as an edge of four faces, the two cells their
/// volumes, and the two united a valid boundary saying it touches itself along the generator.
#[test]
fn a_block_tangent_to_a_cone_along_a_generator_touches_it_along_that_line() {
    use gcs_core::brep::build::{cone,square};
    let frustum = cone([0.,0.,0.],[1.,0.,0.],0.5,1.,2.).unwrap();
    let l = (0.25f64+4.).sqrt();
    let top = prism(&square([1.,0.,-0.75],[0.5/l,0.,2./l],3.),-2.,0.).unwrap();
    let c = arrangement(&[&top,&frustum],1e-9).unwrap();
    c.check(1e-9).unwrap();
    let v = volumes(&c);
    close(v[0],PI*2./3.*(0.25+0.5+1.));
    close(v[1],72.);
    assert_eq!(c.radial.iter().filter(|r| r.len() > 2).count(),1);
    let both = c.select(&|k| k != 0).unwrap();
    both.brep.check(1e-9).unwrap();
    close(volume(&both.brep),72.+PI*2./3.*(0.25+0.5+1.));
    assert!(matches!(both.touches[..],[Touch::Along {faces:4,..}]),"{:?}",both.touches);
}
