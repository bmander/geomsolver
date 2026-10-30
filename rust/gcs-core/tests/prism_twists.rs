//! **Prisms under a constant twist** (#684 M): a prism tool carried by a turn, a screw or a slide.
//! Each moves every ruling of the prism — a line along it from one cap to the other — as one
//! affine field, and a ruling's normal is one, so the contact `n·v` is linear along it: one point
//! of contact a ruling, fixed on the tool, or under a slide the whole ruling at each of the
//! silhouette's points. The characteristic is walked over the sides and round the convex corners'
//! fans where they meet, and carried as a revolution's is (`solid::constant_twist`); across a cap,
//! where the contact is a line, from a side round the rim's fan (#684 AE), and across the crease of
//! a prism drafted both ways from its plane (#684 AG). Each case's volume is computed apart from
//! the core — in closed form, or from the bar's half-spaces (`Bar`) — and each exact solid's STL
//! agrees with the field; the square thread (`rust/examples/solid_sweeps/square_thread.sv`) is
//! `solid_sweeps_examples::the_square_tool_leaves_a_groove_widening_outward`, and the flat end
//! and the creased bar turned (`flat_end.sv`, `creased.sv`) are held there too.
use gcs_core::solid::admission::{self,Class,Condition,Options};
use std::f64::consts::{PI,TAU};

use crate::solid_sweeps_examples::class;
use crate::turns_and_slides::{admit,exported,exported_at,near,refused};
use gcs_core::space::cross;

/// A bar turned about z across `turn.sv`'s block, a ring sector (radii 8 and 12, heights ±1, 60°
/// round): its section the quadrilateral `corners`, drawn in the plane through x leaning `lean`
/// from upright across the turn (`du` along x, `dv` across the turn, falling as it leans), and the
/// bar standing `half` either side of that plane along its length: drawn in the plane through
/// x set `half` back along the length, and extruded from it twice that, drafted `draft` degrees.
fn turned_bar(lean: f64,corners: [[f64;2];4],half: f64,draft: f64) -> String {
    let [c0,c1,c2,c3] = corners.map(|[u,v]| format!("du: {u}mm, dv: {v}mm"));
    let drafted = if draft == 0. { String::new() } else { format!(", draft: {draft}deg") };
    format!("unit mm
use std
use std.place (coords)
use std.motion (turns)

private across := axis
fix(dir == (1, 0, 0)) across
private along := axis
fix(dir == (0, cos({lean}deg), -sin({lean}deg))) along
holder := plane(u: across, v: along)
fix(origin == (0, -{half} * sin({lean}deg), -{half} * cos({lean}deg))) holder
in holder {{
  private c0 := point
  private c1 := point
  private c2 := point
  private c3 := point
  c0 coords({c0}) holder
  c1 coords({c1}) holder
  c2 coords({c2}) holder
  c3 coords({c3}) holder
  private lower := line(c0, c1)
  private upper := line(c1, c2)
  private outer := line(c2, c3)
  private under := line(c3, c0)
}}
construction bar := solid(face(lower, upper, outer, under), from: 0mm, to: 2 * {half}mm{drafted})

in std.front {{
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  q0 coords(du: 8mm, dv: -1mm) std.front
  q1 coords(du: 12mm, dv: -1mm) std.front
  q2 coords(du: 12mm, dv: 1mm) std.front
  q3 coords(du: 8mm, dv: 1mm) std.front
  private wall_in := line(q3, q0)
  private underside := line(q0, q1)
  private wall_out := line(q1, q2)
  private topside := line(q2, q3)
}}
ring := solid(face(wall_in, underside, wall_out, topside), about: ax)
in std.top {{
  private w0 := point
  private w1 := point
  private w2 := point
  fix((0, 0)) w0
  w1 coords(du: 24mm, dv: 0mm) std.top
  w2 coords(du: 24mm * cos(60deg), dv: 24mm * sin(60deg)) std.top
  private wa := line(w0, w1)
  private wb := line(w1, w2)
  private wc := line(w2, w0)
}}
construction wedge := solid(face(wa, wb, wc), from: -2mm, to: 2mm)
wedge common ring
part := solid(ring)

swing := frame
swing turns(1) ax
construction groove := sweep(bar over t in (-140deg, 140deg), under: swing)
groove cut part
")
}

/// A bar of square section stood on a corner `inner` from the axis, standing ±4 along its length.
fn diamond(inner: f64) -> [[f64;2];4] { [[inner,0.],[inner+1.5,1.5],[inner+3.,0.],[inner+1.5,-1.5]] }

/// A turn of the leaning bar: within the block the characteristic is the ruling through its inner
/// corner, round the corner's fan, a line leaning across the turn, so the groove's wall is that
/// line turned, a hyperboloid of one sheet. Its point at height z stands `z tan(lean)` along the
/// turn from the plane through x, so `√(inner² + z² tan² lean)` from the axis (the bar's sides run
/// outward from that corner, so nothing of it at that height stands nearer), and the bar reaches
/// past the block's outer radius: the block loses everything outside that radius, 60° round.
#[test]
fn a_leaning_bar_turned_leaves_a_hyperboloid() {
    let (lean,inner) = (20_f64,10_f64);
    let e = fixtures::read(&turned_bar(lean,diamond(inner),4.,0.));
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn(),"read as a turn");
    assert_eq!(found.fans(),["the corner of `bar.under` and `bar.lower`"]);
    let t2 = lean.to_radians().tan().powi(2);
    let removed = PI/6.*(2.*(144.-inner*inner)-2./3.*t2);
    near(exported(&e,"part"),PI/3.*(144.-64.)-removed,1e-5);
}

/// The leaning bar drafted 2° (#684 G): each side is a plane through its edge's sections at the
/// bar's two ends, and the section shrinks along the bar, its inner corner moving `h tan 2° / sin 45°`
/// outward along its bisector at `h` along the bar. That corner is still a line, the characteristic
/// within the block, and the groove's wall is it turned: at `s = h − 4` it stands at
/// `(10 + √2 tan 2° (s + 4), s sin 20°, s cos 20°)`, so its square distance from the axis is
/// quadratic in the height, and Simpson's rule integrates the block's loss exactly.
#[test]
fn a_drafted_bar_turned_leaves_its_corners_line_turned() {
    let e = fixtures::read(&turned_bar(20.,diamond(10.),4.,2.));
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    // a drafted prism's sides are read loop by loop as its sections walk them
    assert_eq!(found.fans(),["the corner of `bar.lower` and `bar.under`"]);
    let (sl,cl) = 20f64.to_radians().sin_cos();
    let shift = 2f64.sqrt()*2f64.to_radians().tan();
    let r2 = |z: f64| { let s = z/cl; (10.+shift*(s+4.)).powi(2)+(s*sl).powi(2) };
    let removed = PI/6.*crate::brep::simpson(-1.,1.,2,|z| 144.-r2(z));
    near(exported(&e,"part"),PI/3.*(144.-64.)-removed,1e-5);
}

/// Upright, the bar's length is the turn's axis, and the turn keeps the plane of its section: the
/// planar subtraction class reads its swept region whole, as it did before the constant twist read
/// prisms.
#[test]
fn an_upright_bar_turned_is_still_the_planar_subtraction_classs() {
    let e = fixtures::read(&turned_bar(0.,diamond(10.),4.,0.));
    assert!(matches!(class(&e,"part"),Class::Subtraction(_)),"read by the planar subtraction class");
}

/// A bar of square section lying along x, sharp corner down (`diamond`) or flat side down, slid
/// along `(0.3, 1, drop)` across the top of a 10 mm cube: from y = −4 to 14, its lowest point
/// starting 9.8 high, so `drop` −0.2 carries it from 9 over the cube's near face to 7 over its
/// far one. Its length stands from x = −8 to 18, and the slide carries it 5.4 along x.
fn slid_bar(diamond: bool,drop: f64) -> String {
    let section = if diamond { "c0 coords(du: -4mm, dv: 9.8mm) std.side
  c1 coords(du: -2mm, dv: 11.8mm) std.side
  c2 coords(du: -4mm, dv: 13.8mm) std.side
  c3 coords(du: -6mm, dv: 11.8mm) std.side" } else { "c0 coords(du: -2mm, dv: 9.8mm) std.side
  c1 coords(du: -2mm, dv: 12.8mm) std.side
  c2 coords(du: -6mm, dv: 12.8mm) std.side
  c3 coords(du: -6mm, dv: 9.8mm) std.side" };
    let length = (0.09+1.+drop*drop).sqrt();
    format!("unit mm
use std
use std.place (coords)
use std.motion (slides)

in std.top {{
  private k0 := point
  private k1 := point
  private k2 := point
  private k3 := point
  fix((0, 0)) k0
  k1 coords(du: 10mm, dv: 0mm) std.top
  k2 coords(du: 10mm, dv: 10mm) std.top
  k3 coords(du: 0mm, dv: 10mm) std.top
}}
part := solid(face(k0, k1, k2, k3, -> close), from: 0mm, to: 10mm)

in std.side {{
  private c0 := point
  private c1 := point
  private c2 := point
  private c3 := point
  {section}
  private lower := line(c0, c1)
  private upper := line(c1, c2)
  private outer := line(c2, c3)
  private under := line(c3, c0)
}}
construction bar := solid(face(lower, upper, outer, under), from: -8mm, to: 18mm)

private way := axis
fix(dir == ({}, {}, {})) way
feed := frame
feed slides({}mm) way
construction groove := sweep(bar over t in (0deg, 360deg), under: feed)
groove cut part
",0.3/length,1./length,drop/length,18.*length)
}

/// A slide of the bar stood on its corner: the slide moves every ruling's points alike, so the
/// rulings in contact are whole, its silhouette, where the fan of its lowest corner turns square to
/// the slide. That ruling slid is a plane falling 0.2 along y, and the cube loses what lies above
/// it, from 9 high at y = 0 to 7 at y = 10: 10 × 10 × 2 on average. The slide carries the bar
/// along its length too, so it keeps no plane of its section.
#[test]
fn a_bar_slid_aslant_leaves_a_plane() {
    let e = fixtures::read(&slid_bar(true,-0.2));
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert!(found.screw.is_slide(),"read as a slide");
    near(exported(&e,"part"),1000.-200.,1e-6);
}

// -- refusals --------------------------------------------------------------------------------

/// Lying along the turn, a bar of oblong section's underside (`lower`) is square to the turn's axis: the turn
/// keeps it in its own plane, and every point of it is on the boundary, which the class does not
/// sweep (S1, box O of #684). Its ends stand clear of the block.
#[test]
fn a_side_square_to_the_turn_is_refused() {
    let r = refused(admit(&turned_bar(90.,[[10.,0.],[13.,0.],[13.,-3.],[10.,-3.]],9.,0.)));
    assert_eq!(r.condition,Condition::Regular);
    assert!(r.message.contains("is in contact all over: the motion keeps it in its own plane"),"{}",r.message);
}

/// Flat side down and slid level, the bar's underside runs along the slide: every point of it moves
/// along it, the face kept in its own plane (S1).
#[test]
fn a_side_along_the_slide_is_refused() {
    let r = refused(admit(&slid_bar(false,0.)));
    assert_eq!(r.condition,Condition::Regular);
    assert!(r.message.contains("`bar.under` moves along itself"),"{}",r.message);
}

/// The leaning bar of square section drafted both ways from its parting plane (#684 P, AG): its
/// characteristic is its inner corner's line, which reaches the plane partway round the corner's
/// fan, at the vertex where the corner meets the crease. Every normal between the four faces
/// meeting there is the tool's, and the sweep has a sharp edge along that point's path, which the
/// class does not build (S1). (Its flat-sided neighbour crosses the crease mid-side, below.)
#[test]
fn a_corner_meeting_its_crease_turned_is_refused() {
    let text = turned_bar(20.,diamond(10.),4.,2.).replace("from: 0mm, to: 2 * 4mm","from: -4mm, to: 4mm")
        .replace("fix(origin == (0, -4 * sin(20deg), -4 * cos(20deg))) holder","fix(origin == (0, 0, 0)) holder");
    let r = refused(admit(&text));
    assert_eq!(r.condition,Condition::Regular);
    assert!(r.message.contains("the corner of `bar.lower` and `bar.under` runs into a vertex of the prism"),"{}",r.message);
}

/// The square thread's bar made too short to stand past the rod either side: its caps pass through
/// the rod, and the corners where its tip meets its flanks, whose roots run out along their rulings
/// as their fans turn toward the flanks', reach a cap at a vertex of the prism, where the sweep has
/// a sharp edge along that point's path (S1). A cap is crossed only over a rim, mid-side (#684 AE).
#[test]
fn a_corner_reaching_a_cap_under_a_screw_is_refused() {
    let text = crate::solid_sweeps_examples::text("square_thread.sv").replace("reach := 8mm","reach := 2.5mm");
    let e = fixtures::read(&text);
    let r = refused(admission::admit_body(&e.sketch,fixtures::solid(&e,"rod"),&Options::default()));
    assert_eq!(r.condition,Condition::Regular);
    assert!(r.message.contains("the corner of `bar.upper` and `bar.nose` runs into a vertex of the prism"),"{}",r.message);
}

/// The bar slid aslant made short enough for its caps to stand in the cube: under a slide a cap is
/// in contact all over or nowhere, so the silhouette runs over a cap only by turning at a vertex of
/// the prism, a sharp edge of the sweep (S1).
#[test]
fn a_cap_in_the_blank_under_a_slide_is_refused() {
    let r = refused(admit(&slid_bar(true,-0.2).replace("from: -8mm, to: 18mm","from: 2mm, to: 8mm")));
    assert_eq!(r.condition,Condition::Regular);
    assert!(r.message.contains("runs over a cap of the prism, which the class does not walk under a slide"),"{}",r.message);
}

// -- caps and creases (#684 AE, AG) -------------------------------------------------------------

type V = [f64;3];

/// A bar turned across `turned_bar`'s block, or screwed `lead` a turn along its axis where that is
/// not nought: its profile the polygon `corners` (`du`, `dv`) in the plane through `origin` along
/// the unit `across` and `along`, its sides named `names` from each corner to the next, extruded
/// along `across × along` from `from` to `to`, drafted `draft` degrees (both ways from the plane
/// where it runs both ways).
pub(crate) struct Bar { origin: V,across: V,along: V,corners: Vec<[f64;2]>,names: Vec<&'static str>,from: f64,to: f64,draft: f64,
    lead: f64 }

impl Bar {
    /// The drawing: the block turned past by the bar over (−140°, 140°) about z, as `turned_bar`, or
    /// screwed over (−280°, 280°): the sheet's columns stand at the roll's ends delayed by each
    /// point's climb to the section, so a screw's roll reaches further. A lead longer than the bar
    /// is high keeps every other turn of it clear of the block.
    fn text(&self) -> String {
        let [o,a,b] = [self.origin,self.across,self.along];
        let n = self.corners.len();
        let points: String = self.corners.iter().enumerate().map(|(k,[u,v])| format!("  private c{k} := point\n  \
            c{k} coords(du: {u}mm, dv: {v}mm) holder\n")).collect();
        let lines: String = (0..n).map(|k| format!("  private {} := line(c{k}, c{})\n",self.names[k],(k+1)%n)).collect();
        let drafted = if self.draft == 0. { String::new() } else { format!(", draft: {}deg",self.draft) };
        let (motion,roll) = if self.lead == 0. { ("turns(1)".to_string(),140) } else { (format!("screws({}mm)",self.lead),280) };
        format!("unit mm
use std
use std.place (coords)
use std.motion (turns, screws)

private across := axis
fix(dir == ({}, {}, {})) across
private along := axis
fix(dir == ({}, {}, {})) along
holder := plane(u: across, v: along)
fix(origin == ({}, {}, {})) holder
in holder {{
{points}{lines}}}
construction bar := solid(face({}), from: {}mm, to: {}mm{drafted})
{BLOCK}
swing := frame
swing {motion} ax
construction groove := sweep(bar over t in (-{roll}deg, {roll}deg), under: swing)
groove cut part
",a[0],a[1],a[2],b[0],b[1],b[2],o[0],o[1],o[2],self.names.join(", "),self.from,self.to)
    }

    /// The bar as half-spaces `n·p ≤ c` in space: each side's, moved `|h| tan draft` into the
    /// material at `h` along the extrusion (one each side of the plane where it runs both ways),
    /// and its two caps. The bar is convex, so it is their intersection.
    fn halves(&self) -> Vec<(V,f64)> {
        assert!(self.draft == 0. || (self.from < 0. && self.to > 0.),"drafted, the bar runs both ways from its plane");
        let e = cross(self.across,self.along);
        let (o,n) = (self.origin,self.corners.len());
        let area: f64 = (0..n).map(|k| { let ([x0,y0],[x1,y1]) = (self.corners[k],self.corners[(k+1)%n]); x0*y1-x1*y0 }).sum();
        let lift = |x: f64,y: f64,h: f64| -> V { std::array::from_fn(|i| x*self.across[i]+y*self.along[i]+h*e[i]) };
        let dot = |p: V,q: V| p[0]*q[0]+p[1]*q[1]+p[2]*q[2];
        let tan = self.draft.to_radians().tan();
        let mut out = Vec::new();
        for k in 0..n {
            let ([x0,y0],[x1,y1]) = (self.corners[k],self.corners[(k+1)%n]);
            // outward in the profile's plane, for either way round
            let (mut nx,mut ny) = (y1-y0,x0-x1);
            if area < 0. { (nx,ny) = (-nx,-ny); }
            let l = nx.hypot(ny);
            let (nx,ny) = (nx/l,ny/l);
            let c = nx*x0+ny*y0;
            // `nx du + ny dv ± h tan ≤ c`, each way from the plane the draft parts at
            for sign in if self.draft == 0. { vec![0.] } else { vec![1.,-1.] } {
                let normal = lift(nx,ny,sign*tan);
                out.push((normal,c+dot(normal,o)));
            }
        }
        out.push((e.map(|x| -x),-self.from-dot(e,o)));
        out.push((e,self.to+dot(e,o)));
        out
    }

    /// What the turn removes from the block, read apart from the code under test: the block's
    /// points at radius r and height z within its 60° are removed where some point of the bar
    /// stands at r and z, since the turn carries the bar right across them. The bar's points at
    /// height z are a convex polygon (`halves` cut at z, a square clipped by each), so those at z
    /// stand from its least distance from the axis to its greatest, and the block loses
    /// `π/3 ∫ (r₁² − r₀²)/2 dz` over that span within the ring's, by Simpson's rule on 20 000 steps.
    pub(crate) fn removed(&self) -> f64 {
        PI/3.*crate::brep::simpson(-1.,1.,20_000,|z| self.span(z).map_or(0.,|[r0,r1]| {
            let (r0,r1) = (r0.max(8.),r1.min(12.));
            if r1 > r0 { (r1*r1-r0*r0)/2. } else { 0. }
        }))
    }

    /// What the screw removes from the block, read apart from the code under test. It carries a
    /// bar point at angle θ' and height z' about the axis to θ' + t and z' + kt (k its lead over a
    /// turn), so a block point at angle θ and height z is the bar's at time t where the bar has a
    /// point at its radius r at θ − t and z − kt: where `z − kθ`, which the screw keeps, is one of
    /// the bar's points' at r (the roll carries the bar right across the block, and a lead's climb
    /// carries it clear of the block's heights on the turns before and after). Those are the
    /// span from the least to the greatest of `z' − kθ'` over the bar's points at r, each angle's
    /// heights where the bar meets the line up through r and θ' (`halves` cut along it), read at
    /// 2001 angles and refined about the best by four grids of 64 each. The block at r and θ loses
    /// that span raised kθ within its heights, a length linear in θ between where its ends cross
    /// the block's, summed exactly over its 60°, and the radii by Simpson's rule on 4000 steps.
    fn screwed_removed(&self) -> f64 {
        let halves = self.halves();
        let k = self.lead/TAU;
        // the heights at which the bar meets the line up through radius r at angle a
        let heights = |r: f64,a: f64| -> Option<[f64;2]> {
            let (x,y) = (r*a.cos(),r*a.sin());
            let [mut lo,mut hi] = [f64::NEG_INFINITY,f64::INFINITY];
            for &(n,c) in &halves {
                let c = c-n[0]*x-n[1]*y;
                if n[2] > 0. { hi = hi.min(c/n[2]) } else if n[2] < 0. { lo = lo.max(c/n[2]) } else if c < 0. { return None }
            }
            (lo <= hi).then_some([lo,hi])
        };
        // the greatest of `f` over the angles, read on a grid and refined about the best
        let greatest = |f: &dyn Fn(f64) -> f64| -> f64 {
            let mut step = 0.6/1000.;
            let better = |b: (f64,f64),a: f64| { let v = f(a); if v > b.0 { (v,a) } else { b } };
            let mut best = (-1000..=1000).map(|i| i as f64*step).fold((f64::NEG_INFINITY,0.),better);
            for _ in 0..4 {
                let at = best.1;
                let fine = step/32.;
                best = (-32..=32).map(|i| at+i as f64*fine).fold(best,better);
                step = fine;
            }
            best.0
        };
        let span = |r: f64| -> Option<[f64;2]> {
            let top = greatest(&|a| heights(r,a).map_or(f64::NEG_INFINITY,|h| h[1]-k*a));
            let bottom = -greatest(&|a| heights(r,a).map_or(f64::NEG_INFINITY,|h| k*a-h[0]));
            top.is_finite().then_some([bottom,top])
        };
        // the length of [−1, 1] the span raised kθ covers, summed over θ in [0, π/3]: linear in θ
        // between where the span's ends cross ±1
        let across = |[lo,hi]: [f64;2]| -> f64 {
            let covered = |a: f64| ((hi+k*a).min(1.)-(lo+k*a).max(-1.)).max(0.);
            let mut breaks = vec![0.,PI/3.];
            if k != 0. { for e in [lo,hi] { for b in [-1.,1.] { let a = (b-e)/k; if a > 0. && a < PI/3. { breaks.push(a) } } } }
            breaks.sort_by(f64::total_cmp);
            breaks.windows(2).map(|w| 0.5*(covered(w[0])+covered(w[1]))*(w[1]-w[0])).sum()
        };
        crate::brep::simpson(8.,12.,4000,|r| r*span(r).map_or(0.,across))
    }

    /// The least and greatest distances from the axis of the bar's points at height `z`.
    fn span(&self,z: f64) -> Option<[f64;2]> {
        let halves = self.halves();
        let span = |z: f64| -> Option<[f64;2]> {
            let mut polygon: Vec<[f64;2]> = vec![[-50.,-50.],[50.,-50.],[50.,50.],[-50.,50.]];
            for &(n,c) in &halves {
                let c = c-n[2]*z;
                let side = |p: [f64;2]| n[0]*p[0]+n[1]*p[1]-c;
                let mut kept = Vec::new();
                for k in 0..polygon.len() {
                    let (p,q) = (polygon[k],polygon[(k+1)%polygon.len()]);
                    let (sp,sq) = (side(p),side(q));
                    if sp <= 0. { kept.push(p); }
                    if sp*sq < 0. { let t = sp/(sp-sq); kept.push([p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1])]); }
                }
                polygon = kept;
                if polygon.len() < 3 { return None }
            }
            let far = polygon.iter().map(|p| p[0].hypot(p[1])).fold(0_f64,f64::max);
            let m = polygon.len();
            let inside = (0..m).all(|k| { let (p,q) = (polygon[k],polygon[(k+1)%m]); (q[0]-p[0])*(-p[1])-(q[1]-p[1])*(-p[0]) >= 0. })
                || (0..m).all(|k| { let (p,q) = (polygon[k],polygon[(k+1)%m]); (q[0]-p[0])*(-p[1])-(q[1]-p[1])*(-p[0]) <= 0. });
            let near = if inside { 0. } else { (0..m).map(|k| {
                let (p,q) = (polygon[k],polygon[(k+1)%m]);
                let d = [q[0]-p[0],q[1]-p[1]];
                let t = (-(p[0]*d[0]+p[1]*d[1])/(d[0]*d[0]+d[1]*d[1])).clamp(0.,1.);
                (p[0]+t*d[0]).hypot(p[1]+t*d[1])
            }).fold(f64::INFINITY,f64::min) };
            Some([near,far])
        };
        span(z)
    }
}

/// `turned_bar`'s block, the ring sector it cuts: radii 8 and 12, heights ±1, 60° round.
const BLOCK: &str = "in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  q0 coords(du: 8mm, dv: -1mm) std.front
  q1 coords(du: 12mm, dv: -1mm) std.front
  q2 coords(du: 12mm, dv: 1mm) std.front
  q3 coords(du: 8mm, dv: 1mm) std.front
  private wall_in := line(q3, q0)
  private underside := line(q0, q1)
  private wall_out := line(q1, q2)
  private topside := line(q2, q3)
}
ring := solid(face(wall_in, underside, wall_out, topside), about: ax)
in std.top {
  private w0 := point
  private w1 := point
  private w2 := point
  fix((0, 0)) w0
  w1 coords(du: 24mm, dv: 0mm) std.top
  w2 coords(du: 24mm * cos(60deg), dv: 24mm * sin(60deg)) std.top
  private wa := line(w0, w1)
  private wb := line(w1, w2)
  private wc := line(w2, w0)
}
construction wedge := solid(face(wa, wb, wc), from: -2mm, to: 2mm)
wedge common ring
part := solid(ring)";

/// The rectangle `[0, w] × [−h, h]` turned `beta` degrees about the origin of its plane.
fn turned_rectangle(w: f64,h: f64,beta: f64) -> Vec<[f64;2]> {
    let (s,c) = beta.to_radians().sin_cos();
    [[0.,-h],[w,-h],[w,h],[0.,h]].iter().map(|&[u,v]| [c*u-s*v,s*u+c*v]).collect()
}

/// A bar standing up and outward from its flat end, `gamma` degrees from upright, the end facing
/// down toward the axis: drawn in the plane through (10, 0, 0) along `(cos γ, 0, −sin γ)` and y,
/// its profile `turned_rectangle(3, 2, beta)`, and extruded 3 up and out from it. Turned, 20° and
/// 5°, it is `solid_sweeps/flat_end.sv` (`solid_sweeps_examples::the_flat_end_cuts_a_floor`).
pub(crate) fn flat_end(gamma: f64,beta: f64) -> Bar {
    let (s,c) = gamma.to_radians().sin_cos();
    Bar {origin:[10.,0.,0.],across:[c,0.,-s],along:[0.,1.,0.],corners:turned_rectangle(3.,2.,beta),
        names:vec!["lower","outer","upper","inner"],from:0.,to:3.,draft:0.,lead:0.}
}

/// A bar leaning `lean` across the turn as `turned_bar`'s does, its section the rectangle 10 to 13
/// from the axis and ±2 across, standing ±4 either side of the plane through x its section is
/// drawn in, drafted `draft` degrees both ways from that plane. Turned, 20° and 2°, it is
/// `solid_sweeps/creased.sv` (`solid_sweeps_examples::the_creased_bar_crosses_its_crease`).
pub(crate) fn parted(lean: f64,draft: f64) -> Bar {
    let (s,c) = lean.to_radians().sin_cos();
    Bar {origin:[0.;3],across:[1.,0.,0.],along:[0.,c,-s],corners:vec![[10.,-2.],[13.,-2.],[13.,2.],[10.,2.]],
        names:vec!["lower","outer","upper","inner"],from:-4.,to:4.,draft,lead:0.}
}

/// The flat end's bar shared with a block (a `common`) is no prism standing alone: its cap in the
/// block is refused at T1, under a turn as under a relative rotation (#684 F).
#[test]
fn a_cap_of_a_tool_no_lone_prism_turned_is_refused() {
    let text = flat_end(20.,5.).text().replace("construction groove := sweep(bar","in std.top {\n  private k0 := point\n  \
        private k1 := point\n  private k2 := point\n  private k3 := point\n  fix((0, -20)) k0\n  fix((30, -20)) k1\n  \
        fix((30, 20)) k2\n  fix((0, 20)) k3\n}\nconstruction keep := solid(face(k0, k1, k2, k3, -> close), from: -20mm, to: 20mm)\n\
        construction tool := solid(bar)\nkeep common tool\nconstruction groove := sweep(tool");
    let r = refused(admit(&text));
    assert_eq!(r.condition,Condition::Tool);
    assert!(r.message.contains("standing alone"),"{}",r.message);
}

/// The flat end leaning 45° and screwed 8 mm a turn across the block (#684 AE): the screw's advance
/// moves the end's contact line off the plane through the axis, `k cot 45°` across the end, and the
/// inner side's with it, and the floor and wall are those lines carried along the helix, joined
/// round the rim. The block loses what `z − kθ`, which the screw keeps, says the bar covers at
/// each radius (`Bar::screwed_removed`), within 2e-5 at 0.002 mm (1.7e-5 at 0.01). Leaning 40°
/// and screwed 6 mm a turn, the same bar is admitted, but the screw's sheet across the rim's fan
/// misses its normals' bars and the export refuses it, saying so; and at 0.001 mm this one's STL
/// leaves triangles facing against the fan's narrow face.
#[test]
fn a_bars_flat_end_screwed_cuts_a_floor() {
    let bar = Bar {lead:8.,..flat_end(45.,5.)};
    let e = fixtures::read(&bar.text());
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a screw");
    assert_eq!(found.fans(),["the rim where `bar.inner` meets `bar.start`"]);
    near(exported_at(&e,"part",0.002),PI/3.*(144.-64.)-bar.screwed_removed(),2e-5);
}

/// The bar drafted 2° both ways from its plane and screwed 6 mm a turn across the block (#684 AG):
/// its inner side faces the axis, and each half of it leans 2° from it, so each touches along its
/// own line crossing its rulings, and the crease where the halves meet carries the contact from
/// the one to the other round its fan; the three carried along the helix are the groove's wall.
/// The block loses what `z − kθ` says the bar covers at each radius (`Bar::screwed_removed`).
#[test]
fn a_bar_drafted_both_ways_screwed_crosses_its_crease() {
    let bar = Bar {lead:6.,..parted(20.,2.)};
    let e = fixtures::read(&bar.text());
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a screw");
    assert_eq!(found.fans(),["the crease across `bar.inner`"]);
    near(exported_at(&e,"part",0.001),PI/3.*(144.-64.)-bar.screwed_removed(),2e-5);
}


