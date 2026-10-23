// A pursuit curve: a dog runs straight at a hare, always, and twice as fast. The hare runs along a
// line from `o` at unit speed; the dog starts `a` away, square to the hare's line.
//
// The hare is a point of the body placed by rows, `t` along its line at time `t`. The dog is placed
// by rates alone (#339 D2): it moves along the line to the hare, at twice the hare's speed.
//
//   rate(p over t) parallel chase
//   rate(p over t) distance(2, along: chase)
//
// A rate stands where a direction is read, and here the direction is the line between two points
// of the body that both move. Where the dog catches the hare that line has no direction, the rates
// cannot be had, and the curve stops there: `solventc` says it is refused at t = 20, the capture,
// though its interval runs on to 25.
//
// Closed form, the speeds' ratio n = 1/2: x = (a/2)[(y/a)^(1+n)/(1+n) − (y/a)^(1−n)/(1−n)]
// + an/(1 − n²); the dog catches the hare at x = an/(1 − n²) = 20, at t = 20.
unit mm
use std

a := 30mm

component Pursuit(o: point, away: Length, t: Scalar) {
  hare := point hint((0, 0))
  dog := point hint((0, 30))
  hare level(up) o
  o distance(1mm * t, along: right) hare     // the hare runs along its line at unit speed
  chase := line(dog, hare)
  rate(dog over t) parallel chase            // the dog runs straight at it...
  rate(dog over t) distance(2, along: chase) // ...twice as fast
  initially {
    dog level(right) o                       // the home: the dog starts square to the line...
    o distance(away, along: up) dog          // ...`away` from it
  }
}

in std.front {
  o := point
  fix((0mm, 0mm)) o
  run := Pursuit(o, away: a, t: 0)
}
dog := trace(run.dog over t in (0, 25))
