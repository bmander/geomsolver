(** * Layer 2 — [util/expansion.rs], [util/predicates.rs]: each predicate is the sign of its
    determinant.

    The four filtered predicates ([orient], [power], [orient2], [in_circle]) are claimed exact:
    the sign each returns is the sign of a determinant of real numbers.  That is true only of
    inputs whose products neither overflow nor underflow, which the Rust assumes and does not
    check ([expansion.rs]: "Assumes round-to-nearest-even binary64 without overflow or
    underflow"; [gaps/] shows [orient] answering "coplanar" for a tetrahedron of side 1e-108).
    So each statement carries the range it is claimed over ([tame]), wide enough for any drawing
    and well inside what the arithmetic needs; the proofs are to say how much wider it may be.

    The error-free transformations are transcribed and their theorems stated of the
    transcription.  The predicates are stated as properties any function standing for them must
    have, their transcription (the filter, and the expansions it falls back to) left to the
    translator; the invariant the expansions keep is the proof's to choose, and is not stated. *)

From Coq Require Import ZArith Reals Bool.
From Flocq Require Import Core BinarySingleNaN.
From Kernel Require Import Float.
Open Scope R_scope.

(** ** The error-free transformations, transcribed *)

Definition two_sum (a b : f64) : f64 * f64 :=
  let x := fadd a b in
  let bv := fsub x a in
  let av := fsub x bv in
  (x, fadd (fsub a av) (fsub b bv)).

Definition fast_two_sum (a b : f64) : f64 * f64 :=
  let x := fadd a b in
  (x, fsub b (fsub x a)).

Definition two_diff (a b : f64) : f64 * f64 :=
  let x := fsub a b in
  let bv := fsub a x in
  let av := fadd x bv in
  (x, fadd (fsub a av) (fsub bv b)).

(** [SPLITTER], 2^27 + 1. *)
Definition splitter : f64 := of_Z 134217729 0.

Definition split (a : f64) : f64 * f64 :=
  let c := fmul splitter a in
  let h := fsub c (fsub c a) in
  (h, fsub a h).

Definition two_product (a b : f64) : f64 * f64 :=
  let x := fmul a b in
  let '(ahi, alo) := split a in
  let '(bhi, blo) := split b in
  let err := fsub (fsub (fsub x (fmul ahi bhi)) (fmul alo bhi)) (fmul ahi blo) in
  (x, fsub (fmul alo blo) err).

(** A float no larger than [2^e] in magnitude. *)
Definition within_pow (e : Z) (x : f64) : Prop :=
  finite x = true /\ Rabs (real x) <= bpow radix2 e.

(** ** Their theorems: the pair's sum is the exact result, and its first the rounded one *)

Theorem two_sum_exact : forall a b, within_pow 1022 a -> within_pow 1022 b ->
  let '(x, y) := two_sum a b in
  finite x = true /\ finite y = true /\
  real x = rnd (real a + real b) /\ real x + real y = real a + real b.
Admitted.

(** The quick form needs the larger operand first. *)
Theorem fast_two_sum_exact : forall a b, within_pow 1022 a -> within_pow 1022 b ->
  Rabs (real b) <= Rabs (real a) ->
  let '(x, y) := fast_two_sum a b in
  finite x = true /\ finite y = true /\
  real x = rnd (real a + real b) /\ real x + real y = real a + real b.
Admitted.

Theorem two_diff_exact : forall a b, within_pow 1022 a -> within_pow 1022 b ->
  let '(x, y) := two_diff a b in
  finite x = true /\ finite y = true /\
  real x = rnd (real a - real b) /\ real x + real y = real a - real b.
Admitted.

(** A product's error is a float only where it does not underflow: the operands' last places,
    [2^ea] and [2^eb], must multiply to no less than the least subnormal. *)
Theorem two_product_exact : forall a b (ma mb ea eb : Z),
  within_pow 500 a -> within_pow 500 b ->
  real a = IZR ma * bpow radix2 ea -> (Z.abs ma < 2 ^ 53)%Z ->
  real b = IZR mb * bpow radix2 eb -> (Z.abs mb < 2 ^ 53)%Z ->
  (-1074 <= ea + eb)%Z ->
  let '(x, y) := two_product a b in
  finite x = true /\ finite y = true /\
  real x = rnd (real a * real b) /\ real x + real y = real a * real b.
Admitted.

(** ** The determinants, over the reals *)

Definition R3 : Type := (R * R * R)%type.

Definition minus3 (p q : R3) : R3 :=
  let '(px, py, pz) := p in let '(qx, qy, qz) := q in (px - qx, py - qy, pz - qz).

Definition norm2 (p : R3) : R := let '(x, y, z) := p in x * x + y * y + z * z.

(** The determinant with rows [r0], [r1], [r2]. *)
Definition det3 (r0 r1 r2 : R3) : R :=
  let '(a, b, c) := r0 in let '(d, e, f) := r1 in let '(g, h, i) := r2 in
  a * (e * i - f * h) + b * (f * g - d * i) + c * (d * h - e * g).

(** The sign of a real, as the predicates return it. *)
Definition sgn (r : R) : Z :=
  match Rcompare r 0 with Lt => (-1)%Z | Eq => 0%Z | Gt => 1%Z end.

Definition P3 : Type := (f64 * f64 * f64)%type.
Definition P2 : Type := (f64 * f64)%type.

Definition real3 (p : P3) : R3 := let '(x, y, z) := p in (real x, real y, real z).

(** [orient]'s determinant: rows [b − a], [c − a], [d − a]. *)
Definition orient_det (a b c d : P3) : R :=
  det3 (minus3 (real3 b) (real3 a)) (minus3 (real3 c) (real3 a)) (minus3 (real3 d) (real3 a)).

(** A weighted point, and [power]'s determinant: rows [(p − e, |p − e|² − (w − w_e))] for
    [a], [b], [c], [d], expanded along the lifted column as [power_exact] expands it. *)
Definition W3 : Type := (P3 * f64)%type.

Definition power_det (a b c d e : W3) : R :=
  let row (q : W3) := minus3 (real3 (fst q)) (real3 (fst e)) in
  let lift (q : W3) := norm2 (row q) - (real (snd q) - real (snd e)) in
  - lift a * det3 (row b) (row c) (row d) + lift b * det3 (row a) (row c) (row d)
  - lift c * det3 (row a) (row b) (row d) + lift d * det3 (row a) (row b) (row c).

(** [orient2]'s: [(b − a) × (c − a)]. *)
Definition orient2_det (a b c : P2) : R :=
  let '(ax, ay) := a in let '(bx, by_) := b in let '(cx, cy) := c in
  (real bx - real ax) * (real cy - real ay) - (real by_ - real ay) * (real cx - real ax).

(** [in_circle]'s: [a], [b], [c] read from [d], each row lifted by its square. *)
Definition in_circle_det (a b c d : P2) : R :=
  let rel (p : P2) := (real (fst p) - real (fst d), real (snd p) - real (snd d)) in
  let '(ax, ay) := rel a in let '(bx, by_) := rel b in let '(cx, cy) := rel c in
  (ax * ax + ay * ay) * (bx * cy - cx * by_)
  + (bx * bx + by_ * by_) * (cx * ay - ax * cy)
  + (cx * cx + cy * cy) * (ax * by_ - bx * ay).

(** ** The range the predicates are claimed over

    A float is [tame g] where it is a whole multiple of [2^-g] no larger than [2^g]: nought, or
    between about 1e-45 and 1e45 in magnitude at [g = 150].  A determinant of degree 5 in such
    coordinates is a multiple of [2^-750] below [2^1024], so no product in it underflows or
    overflows.  A weight is an area, so it is held to twice the exponent. *)
Definition tame (g : Z) (x : f64) : Prop :=
  finite x = true /\ Rabs (real x) <= bpow radix2 g /\
  exists k : Z, real x = IZR k * bpow radix2 (- g).

Definition tame3 (p : P3) : Prop :=
  let '(x, y, z) := p in tame 150 x /\ tame 150 y /\ tame 150 z.
Definition tame2 (p : P2) : Prop := tame 150 (fst p) /\ tame 150 (snd p).
Definition tameW (q : W3) : Prop := tame3 (fst q) /\ tame 300 (snd q).

(** ** The predicates: the properties, awaiting their transcription *)

(** [predicates::orient] and [orient_exact]: positive where [d] is on the side of the plane
    [abc] that [(b − a) × (c − a)] points to. *)
Definition orient_exact_sign (orient : P3 -> P3 -> P3 -> P3 -> Z) : Prop :=
  forall a b c d, tame3 a -> tame3 b -> tame3 c -> tame3 d ->
    orient a b c d = sgn (orient_det a b c d).

(** [predicates::power] and [power_exact]: the lifted determinant's sign, reversed. *)
Definition power_exact_sign (power : W3 -> W3 -> W3 -> W3 -> W3 -> Z) : Prop :=
  forall a b c d e, tameW a -> tameW b -> tameW c -> tameW d -> tameW e ->
    power a b c d e = (- sgn (power_det a b c d e))%Z.

(** [predicates::orient2]: positive where [a], [b], [c] turn counter-clockwise. *)
Definition orient2_exact_sign (orient2 : P2 -> P2 -> P2 -> Z) : Prop :=
  forall a b c, tame2 a -> tame2 b -> tame2 c ->
    orient2 a b c = sgn (orient2_det a b c).

(** [predicates::in_circle]: positive where [d] is inside the circle through the
    counter-clockwise [a], [b], [c]. *)
Definition in_circle_exact_sign (in_circle : P2 -> P2 -> P2 -> P2 -> Z) : Prop :=
  forall a b c d, tame2 a -> tame2 b -> tame2 c -> tame2 d ->
    in_circle a b c d = sgn (in_circle_det a b c d).
