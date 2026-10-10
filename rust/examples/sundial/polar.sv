// A polar sundial, the one that works on the equator. A horizontal dial's style rises at the
// latitude, so on the equator it lies on the plate and every hour line folds onto noon. Lift the
// style instead: here it runs due north, level, on a fin `height` above the plate, pointing at
// the celestial pole on the horizon.
//
// The hour lines are found as the horizontal dial's are: `sun.EquinoxSun` sets the sun round the
// style an hour at a time, and each hour line is where the plane through the style and that
// hour's sun meets the plate: its ends are on the plate's edges, each in the plane — square to
// its normal (`sky.normal[k]`) as seen from the style. Now the planes all hold a line parallel to
// the plate, so they meet it in lines parallel to noon, each `height * tan(h)` east or west of it
// for the hour `h` from noon, which nothing here states. Three o'clock is `height` out, and six
// o'clock never comes — its plane is level with the plate — so the plate reads seven to five.
//
// Elsewhere a polar dial is this one tilted to the latitude, its plate and style still parallel
// to the earth's axis, and its hour lines do not change. Edit `height` and they spread or close
// up together; open the glass box (⌘B) to see the sun's ring standing square to the style.
unit mm
use std (horizontal, vertical)
use sun
height := 25mm    // the style above the plate
length := 120mm   // the plate, south to north
width := 220mm    // and west to east

// the plate, its noon line running north from O, the middle of its south edge
O := point in std.top, std.side
N := point in std.top, std.side hint((0, 120))
fix(y == 0) O
in std.top {
  C := point hint((0, 60))
  plate := std.CenteredRectangle(C, w: width, h: length)
  O midpoint plate.ab
  N coincident plate.cd
}

// the gnomon, a fin on the noon line in the side plane: the style along its top, `height` up,
// and the ray from its south end toward the noon sun, straight up
in std.side {
  A := point hint((0, 25))
  B := point hint((120, 25))
  noon := line(O, N)
  north_post := line(N, B)
  style := line(A, B)
  south_post := line(O, A)
  O vertical A
  N vertical B
  A horizontal B
  distance(height) south_post
  S := point hint((0, 50))
  construction noon_sun := line(A, S)
  noon_sun perpendicular style
  noon_sun equal south_post
}
// and due east from it, level with the style
E := point hint((25, 0, 25))
construction east := line(A, E)
east parallel std.x
east equal south_post
sky := sun.EquinoxSun(A, style, east, noon_sun, r: 40mm)

// the hour lines, seven o'clock to five: each where the shadow plane meets the plate, from its
// south edge to its north edge, each end seen from the style square to the plane's normal
repeat 11 as i {
  s := point in std.top hint((0, 0))
  n := point in std.top hint((0, 120))
  s coincident plate.ab
  n coincident plate.cd
  hour := line(s, n)
  construction drop_s := line(A, s)
  construction drop_n := line(A, n)
  drop_s perpendicular sky.normal[7 + i]
  drop_n perpendicular sky.normal[7 + i]
}

// the plate and the fin on it
base := solid(plate.loop, depth: 6mm)
fin := solid(face(noon, north_post, style, south_post), from: -1.5mm, to: 1.5mm)
