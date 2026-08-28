// A closed ring of equal-length links, and a redundancy that only arithmetic finds.
//
// Each link is its own segment, joined end to end all the way round, and each is told it is the
// same length as the next one.  Round a closed ring that last statement is one too many: if each
// link equals the one after it the whole way round, then "and the last equals the first" was
// already true before it was said.
//
// Counting does not notice.  There are as many statements as links and each looks like it says
// something.  Only working through the actual numbers shows that one of them adds nothing, and
// that is what this case is here to be checked against.
//
// Nothing says how big the ring is, so it is under-determined on purpose: all the links can grow
// together, and dragging one shows it.

param n = 12
param radius = 50

cycle n as i {
  // the link from bearing i to bearing i+1 round the ring, each end its own point
  point a hint(x: radius * cos(tau * i / n), y: radius * sin(tau * i / n))
  point b hint(x: radius * cos(tau * (i + 1) / n), y: radius * sin(tau * (i + 1) / n))
  line  e(a, b)

  coincident(b, next.a)
  equal_length(e, next.e)
}

// the ring floats otherwise; one end of the first link is enough to pin it
ground(a[0])
