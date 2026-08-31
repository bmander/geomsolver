// A square, stated the way a draughtsman says it: four equal sides, each meeting the next at a
// right angle — one line and one corner, four times round.
//
// The body ends mid-joint (§6.6): the `->` standing at the body's `}` threads the chain onto
// the *next copy's* first link, and `cycle` wraps, so the last side welds back to the first.
// No `close`, no named corners, no written points — each weld is a shared point the chain
// mints.  `perpendicular` and `equal` are stated at all four corners; round a closed loop one
// of each is a theorem, which the diagnosis notes as implied and never paints.

cycle 4 {
  line s -> perpendicular equal
}

// the loop states everything but a size and a pose: one dimension scales it, and a grounded
// corner leaves a single freedom — drag any side and the square swings about that corner
s[0].p1 distance(50) s[0].p2
ground s[0].p1
