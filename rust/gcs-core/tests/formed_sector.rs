//! A static body cut by copies turned about its blank's axis (`brep::formed`): built as one sector
//! patterned, it is the body cut copy by copy — the same volume, every face of the copies there —
//! and where a premise fails (two copies, too few for a sector) it is built copy by copy.
use gcs_core::brep::{props,recipe,topo::Brep};
use gcs_core::json::Json;
use std::collections::BTreeMap;

/// A ring about the spindle cut by balls turned to `count` places about it.
fn ring(count: usize) -> String {
    format!("{}construction removal := solid(tool)\n{}",fixtures::tools::sphere(1.5),fixtures::tools::indexed_ring(count,2.5,3.5,1.,2.))
}

/// `part`'s recipe built node by node, every body cut copy by copy.
fn copy_by_copy(recipe: &Json) -> Brep {
    let mut built = BTreeMap::new();
    for n in recipe.get("nodes").unwrap().arr() { built.insert(n.get("id").unwrap().as_i64(),recipe::node(n,&built).unwrap()); }
    built.remove(&recipe.get("root").unwrap().as_i64()).unwrap()
}

#[test]
fn a_ring_cut_by_turned_copies_is_one_sector_patterned() {
    let e = fixtures::read(&ring(6));
    let recipe = gcs_core::solid::cad::recipe(&e.sketch,fixtures::solid(&e,"part")).unwrap();
    let nodes = recipe::by_id(&recipe).unwrap();
    let root = nodes[&recipe.get("root").unwrap().as_i64()];
    let mut built = BTreeMap::new();
    for n in recipe.get("nodes").unwrap().arr() {
        let id = n.get("id").unwrap().as_i64();
        if n.get("id") == root.get("id") { break }
        built.insert(id,recipe::node(n,&built).unwrap());
    }
    let sector = gcs_core::brep::formed::body(root,&nodes,&built).expect("the ring is built as one sector");
    let whole = copy_by_copy(&recipe);
    sector.check(1e-6).unwrap();
    let (a,b) = (props::volume(&sector),props::volume(&whole));
    assert!((a-b).abs() <= 1e-7*b,"{a} against {b}");
    // every face the copies leave, under its copy's name
    let cut = |s: &Brep| s.faces.iter().filter(|f| f.name.contains("indexed")).count();
    assert_eq!(cut(&sector),cut(&whole));
    // and `build` takes that path
    assert!((props::volume(&recipe::build(&recipe).unwrap())-b).abs() <= 1e-7*b);
}

#[test]
fn two_copies_are_cut_one_by_one() {
    let e = fixtures::read(&ring(2));
    let recipe = gcs_core::solid::cad::recipe(&e.sketch,fixtures::solid(&e,"part")).unwrap();
    let nodes = recipe::by_id(&recipe).unwrap();
    let root = nodes[&recipe.get("root").unwrap().as_i64()];
    assert!(gcs_core::brep::formed::body(root,&nodes,&BTreeMap::new()).is_none());
    let (a,b) = (props::volume(&recipe::build(&recipe).unwrap()),props::volume(&copy_by_copy(&recipe)));
    assert!((a-b).abs() <= 1e-9*b,"{a} against {b}");
}
