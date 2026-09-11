//! Coverage of one triangle by a sheet of tiles on the same surface: what
//! the overlap clip asks of every triangle of a later source.
use gcs_core::solid::swept_boundary::{covered_by as covered_by_sheet,uncovered as uncovered_by_sheet};
use std::f64::consts::PI;

type V3 = [f64;3];

/// The boundary edges of a set of tiles, each with its tile.
fn outline(tiles: &[[V3;3]]) -> Vec<([V3;3],V3,V3)> {
    let key = |p: V3| p.map(|x| (x*1e9).round() as i64);
    let mut uses: std::collections::BTreeMap<([i64;3],[i64;3]),(usize,usize,usize)> = Default::default();
    for (i,t) in tiles.iter().enumerate() { for k in 0..3 {
        let (a,b) = (key(t[k]),key(t[(k+1)%3]));
        let e = uses.entry((a.min(b),a.max(b))).or_insert((0,i,k)); e.0 += 1;
    } }
    uses.values().filter(|(n,_,_)| *n == 1).map(|&(_,i,k)| (tiles[i],tiles[i][k],tiles[i][(k+1)%3])).collect()
}

fn covered_by(t: [V3;3],tiles: impl Iterator<Item = [V3;3]>,tolerance: f64) -> bool {
    let tiles: Vec<[V3;3]> = tiles.collect();
    covered_by_sheet(t,&tiles,&outline(&tiles),tolerance)
}

fn uncovered(t: [V3;3],tiles: impl Iterator<Item = [V3;3]>,tolerance: f64) -> Option<Vec<[V3;3]>> {
    let tiles: Vec<[V3;3]> = tiles.collect();
    uncovered_by_sheet(t,&tiles,&outline(&tiles),tolerance)
}

#[test]
fn a_triangle_inside_one_tile_is_covered() {
    let t = [[0.2,0.2,0.],[0.4,0.2,0.],[0.3,0.4,0.]];
    let tile = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]];
    assert!(covered_by(t,[tile].into_iter(),0.04));
}

#[test]
fn a_triangle_across_two_tiles_sharing_an_edge_is_covered() {
    let t = [[0.3,0.3,0.],[0.7,0.3,0.],[0.5,0.6,0.]];
    let tiles = [[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],[[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]]];
    assert!(covered_by(t,tiles.into_iter(),0.04));
}

#[test]
fn a_triangle_half_outside_the_tiles_is_not_covered() {
    let t = [[0.5,0.5,0.],[1.5,0.5,0.],[1.,0.8,0.]];
    let tiles = [[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],[[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]]];
    assert!(!covered_by(t,tiles.into_iter(),0.04));
}

#[test]
fn a_tile_that_only_touches_an_edge_takes_nothing() {
    let t = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]];
    let tiles = [[[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]]];
    assert!(!covered_by(t,tiles.into_iter(),0.04));
}

#[test]
fn a_tile_facing_the_other_way_or_off_the_surface_does_not_count() {
    let t = [[0.2,0.2,0.],[0.4,0.2,0.],[0.3,0.4,0.]];
    let reversed = [[0.,0.,0.],[0.,1.,0.],[1.,0.,0.]];
    assert!(!covered_by(t,[reversed].into_iter(),0.04));
    let lifted = [[0.,0.,0.1],[1.,0.,0.1],[0.,1.,0.1]];
    assert!(!covered_by(t,[lifted].into_iter(),0.04));
}

/// A sliver of a cylinder wall between a coarse and a fine polygon's
/// vertices, against the wall as a coarser sweep tessellates it: the
/// plunged cylinder's cap wall against its rim's sweep.
#[test]
fn a_wall_sliver_is_covered_by_the_coarser_wall_tiles() {
    let at = |angle: f64,z: f64| -> V3 { [3.+angle.cos(),angle.sin(),z] };
    let coarse = 2.*PI/17.;
    let fine = 2.*PI/64.;
    // the sliver: a 17-gon vertex at the top, the same at the bottom, the
    // next 64-gon vertex at the bottom (wound outward)
    let t = [at(-2.*coarse,-5.),at(-2.*coarse,-7.),at(-2.*coarse+fine,-7.)];
    let n = gcs_core::solid::swept_boundary::certify::triangle_normal(t[0],t[1],t[2]).unwrap();
    let outward = at(-2.*coarse+fine/2.,-6.);
    assert!(n[0]*(outward[0]-3.)+n[1]*outward[1] > 0.,"the sliver must face outward");
    // ribbon tiles: rows of half a unit between z = -7 and -1, 17 around
    let mut tiles = Vec::new();
    for row in 0..12 {
        let (z0,z1) = (-7.+0.5*row as f64,-6.5+0.5*row as f64);
        for k in 0..17 {
            let (a0,a1) = (k as f64*coarse,(k+1) as f64*coarse);
            tiles.push([at(a0,z0),at(a1,z0),at(a1,z1)]);
            tiles.push([at(a0,z0),at(a1,z1),at(a0,z1)]);
        }
    }
    assert!(covered_by(t,tiles.into_iter(),0.04));
}

#[test]
fn a_triangle_half_outside_the_tiles_keeps_its_outer_half() {
    let t = [[0.5,0.5,0.],[1.5,0.5,0.],[1.,0.8,0.]];
    let tiles = [[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],[[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]]];
    let left = uncovered(t,tiles.into_iter(),0.04).expect("touched");
    let area: f64 = left.iter().map(|[a,b,c]| ((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])).abs()/2.).sum();
    // the whole triangle is 0.15; its part at x > 1 is a triangle of base 0.5 and height 0.3... cut at x = 1
    assert!((area-0.075).abs() < 1e-9,"area left {area}");
    assert!(left.iter().all(|tri| tri.iter().all(|p| p[0] >= 1.-1e-9)));
}
