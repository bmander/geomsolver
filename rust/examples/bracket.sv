// An L-bracket drawn the way a draughtsman draws a part: three views on one sheet, and an
// auxiliary view for the one face none of the three shows true-size (§6.7).
//
// Every view is a `plane` — a datum on the page with an attitude in space — and each view's
// geometry is written in one `in … { … }` block: the membership clause written once, over the
// section that draws that view.  `project` then says two points are two images of one corner
// of the part: their coordinates along the fold line the views share agree.  That is the whole
// of descriptive geometry, and it is one equation per pair.  Nothing three-dimensional is ever
// solved for; the sheet stays a sheet.
//
// The part, in the space behind the front view: a 60 × 30 footprint, 15 thick at the base, with
// a 15-wide upright rising to 40 whose outer face is cut on an incline from 30 to 40.  The views
// look at it from the front (the page itself), from above (`top`, folded up from the front's
// x-axis), from the right (`right`, folded from the front's z-axis and laid flat beside it) and
// square-on to the incline (`aux`, folded from the front along the incline's own bearing).

width := 60
depth := 30
base := 15
leg := 15
rise := 40
toe := 30
tilt := atan((rise - toe) / leg)   // the incline's bearing in the front view

// the four datums.  Each view's origin is the part's corner A as that view sees it, so the
// origins are all images of one point and no projection between them needs stating; the
// `toward` points set which way each view is turned on the page.  Third-angle layout: the top
// view above the front, the right view beside it turned so its z is up and its depth grows to
// the right, the auxiliary across the incline's fold line.
Af := point hint(x: 0, y: 0) in front
qf := point hint(x: 40, y: 0)
front := plane(origin: Af, toward: qf)
At := point hint(x: 0, y: 90) in top
qt := point hint(x: 40, y: 90)
top := plane(origin: At, toward: qt, from: front, fold: 0deg)
Ar := point hint(x: 150, y: 0) in right
qr := point hint(x: 150, y: -40)
right := plane(origin: Ar, toward: qr, from: front, fold: -90deg)
oa := point hint(x: -70, y: 70)
qa := point hint(x: -36.7, y: 92.2)
aux := plane(origin: oa, toward: qa, from: front, fold: tilt)
ground Af
ground qf
ground At
ground qt
ground Ar
ground qr
ground oa
ground qa

// the front view: the profile, and every dimension the part is made to
in front {
  Bf := point hint(x: 60, y: 0)
  Cf := point hint(x: 60, y: 15)
  Df := point hint(x: 15, y: 15)
  Ef := point hint(x: 15, y: 40)
  Ff := point hint(x: 0, y: 30)
  horizontal (ab := line(Af, Bf)) -> vertical (bc := line(Bf, Cf)) -> horizontal (cd := line(Cf, Df)) ->
    vertical (de := line(Df, Ef)) -> (ef := line(Ef, Ff)) -> vertical (fa := line(Ff, Af))
  Af distance(width) Bf
  Bf distance(base) Cf
  Af distance(leg, along: x) Df
  Af distance(rise, along: y) Ef
  Af distance(toe, along: y) Ff
}

// the top view: the footprint, the upright's ridge, and the one dimension the front cannot
// show — the depth.  F sits under A from above, so its image here is stated as coincident
// with A's, which is what the auxiliary view will project from.
in top {
  Bt := point
  A2t := point
  B2t := point
  Et := point
  E2t := point
  horizontal (t1 := line(At, Bt)) -> vertical (t2 := line(Bt, B2t)) -> horizontal (t3 := line(B2t, A2t)) ->
    vertical (t4 := line(A2t, At))
  ridge_t := vertical line(Et, E2t)
  At horizontal Et
  A2t horizontal E2t
  At distance(depth, along: y) A2t
  Bf project Bt
  Ef project Et
  Ft := point
  F2t := point
  Ft coincident At
  F2t coincident A2t
}

// the right view: the L's silhouette, the step, and the incline's lower edge hidden behind the
// upright.  Heights come across from the front, the depth up from the top.
in right {
  A2r := point
  Er := point
  E2r := point
  Cr := point
  C2r := point
  Fr := point
  F2r := point
  horizontal (r1 := line(Ar, A2r)) -> vertical (r2 := line(A2r, E2r)) -> horizontal (r3 := line(E2r, Er)) ->
    vertical (r4 := line(Er, Ar))
  step_r := horizontal line(Cr, C2r)
  toe_r := horizontal line(Fr, F2r)
  Ar vertical Cr
  A2r vertical C2r
  Ar vertical Fr
  A2r vertical F2r
  A2t project A2r
  Ef project Er
  Cf project Cr
  Ff project Fr
}

// the auxiliary view: the inclined face true-size.  Its four corners are placed by projection
// alone — along the incline from the front, and in depth from the top — so the view is wholly
// derived and the face comes out the rectangle it is.
in aux {
  Fa := point
  Ea := point
  F2a := point
  E2a := point
  (a1 := line(Fa, Ea)) -> (a2 := line(Ea, E2a)) -> (a3 := line(E2a, F2a)) -> (a4 := line(F2a, Fa))
  Ff project Fa
  Ef project Ea
  Ff project F2a
  Ef project E2a
  Ft project Fa
  Et project Ea
  F2t project F2a
  E2t project E2a
}

// `solventc` on this document: 58 params, 58 equations, structural rank 58, DOF 0 — every
// view placed by the front's six dimensions and the top's one, and the auxiliary face coming
// out hypot(15, 10) by 30, which no principal view can show.
