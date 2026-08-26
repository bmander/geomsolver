// A closed chain of equal-length links — and a redundancy no structural count can see.
//
// Every link is its own line with its own two ends, joined to the next by a coincidence rather
// than by sharing a point, and every link is told it is as long as the one after it.  Round a
// closed ring that last statement is one too many: `e0 = e1`, `e1 = e2`, … `e[n-1] = e0` is `n`
// equations saying what `n - 1` of them already said, so the matching counts a rank the Jacobian
// does not have.  Only the numeric cross-check finds it, which is what the case is for.
//
// Nothing sizes the ring, so it is under-constrained on purpose: the links may all grow together.

param n = 12
param radius = 50

cycle n as i {
  // the link from bearing i to bearing i+1 round the ring, each end its own point
  point a at (radius * cos(tau * i / n), radius * sin(tau * i / n))
  point b at (radius * cos(tau * (i + 1) / n), radius * sin(tau * (i + 1) / n))
  line  e(a, b)

  coincident(b, next.a)
  equal_length(e, next.e)
}

// the ring floats otherwise; one end of the first link is enough to pin it
ground(a[0])
