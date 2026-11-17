(** * Layer 1 — [util/interval.rs]: every operation encloses.

    The claim of the module's first line, "arithmetic encloses real operations on binary64
    endpoints; invalid domains and overflow fail closed", made exact: an operation that answers
    [Ok c] answers an interval [c] holding the real result for every real its operands hold, and
    an operation that cannot answers [Err].

    The arithmetic ([new] to [sqrt], and [Iv]'s readings) is transcribed below from the Rust and
    its theorems are stated of the transcription.  The trigonometry is stated as a property any
    function standing for [sin_cos] must have, its transcription left to the translator. *)

From Coq Require Import ZArith Reals Bool List Lra.
From Flocq Require Import Core BinarySingleNaN.
From Kernel Require Import Float.
Open Scope R_scope.

(** ** The Rust, transcribed *)

Inductive error : Type := InvalidBounds | Overflow | DivisionByZero | OutsideDomain.

Inductive result (A : Type) : Type :=
  | Ok (a : A)
  | Err (e : error).
Arguments Ok {A} a.
Arguments Err {A} e.

Definition bind {A B : Type} (r : result A) (f : A -> result B) : result B :=
  match r with Ok a => f a | Err e => Err e end.

(** [struct Interval { lo, hi }].  Its fields are private in Rust, and [valid] is what every
    constructor keeps of them. *)
Record interval : Type := mk { lo : f64; hi : f64 }.

Definition valid (a : interval) : Prop :=
  finite (lo a) = true /\ finite (hi a) = true /\ real (lo a) <= real (hi a).

(** The reals an interval holds. *)
Definition holds (a : interval) (x : R) : Prop := real (lo a) <= x <= real (hi a).

(** [Interval::new]. *)
Definition new (l h : f64) : result interval :=
  if negb (finite l) || negb (finite h) || flt h l then Err InvalidBounds else Ok (mk l h).

(** [Interval::point]. *)
Definition point (x : f64) : result interval := new x x.

(** [Interval::contains]. *)
Definition contains (a : interval) (x : f64) : bool := fle (lo a) x && fle x (hi a).

(** [Interval::neg]. *)
Definition ineg (a : interval) : interval := mk (fneg (hi a)) (fneg (lo a)).

(** [Interval::outward]: each end moved one float outward. *)
Definition outward (l h : f64) : result interval :=
  let l := next_down l in
  let h := next_up h in
  if negb (finite l) || negb (finite h) then Err Overflow else new l h.

Definition iadd (a b : interval) : result interval :=
  outward (fadd (lo a) (lo b)) (fadd (hi a) (hi b)).

Definition isub (a b : interval) : result interval :=
  outward (fsub (lo a) (hi b)) (fsub (hi a) (lo b)).

(** [p.into_iter().fold(f64::INFINITY, f64::min)] and its mirror, over the four products. *)
Definition least4 (p q r s : f64) : f64 := fmin (fmin (fmin (fmin infinity p) q) r) s.
Definition most4 (p q r s : f64) : f64 := fmax (fmax (fmax (fmax neg_infinity p) q) r) s.

Definition imul (a b : interval) : result interval :=
  let p := fmul (lo a) (lo b) in
  let q := fmul (lo a) (hi b) in
  let r := fmul (hi a) (lo b) in
  let s := fmul (hi a) (hi b) in
  outward (least4 p q r s) (most4 p q r s).

Definition idiv (a b : interval) : result interval :=
  if contains b zero then Err DivisionByZero else
  let p := fdiv (lo a) (lo b) in
  let q := fdiv (lo a) (hi b) in
  let r := fdiv (hi a) (lo b) in
  let s := fdiv (hi a) (hi b) in
  outward (least4 p q r s) (most4 p q r s).

Definition isquare (a : interval) : result interval :=
  let p := fmul (lo a) (lo a) in
  let q := fmul (hi a) (hi a) in
  bind (outward (if contains a zero then zero else fmin p q) (fmax p q))
       (fun c => Ok (mk (fmax (lo c) zero) (hi c))).

Definition isqrt (a : interval) : result interval :=
  if flt (lo a) zero then Err OutsideDomain else
  bind (outward (fsqrt (lo a)) (fsqrt (hi a)))
       (fun c => Ok (mk (fmax (lo c) zero) (hi c))).

(** ** What it is to enclose *)

Definition encloses1 (op : interval -> result interval) (f : R -> R) : Prop :=
  forall a c, valid a -> op a = Ok c ->
    valid c /\ forall x, holds a x -> holds c (f x).

Definition encloses2 (op : interval -> interval -> result interval) (f : R -> R -> R) : Prop :=
  forall a b c, valid a -> valid b -> op a b = Ok c ->
    valid c /\ forall x y, holds a x -> holds b y -> holds c (f x y).

(** ** The theorems *)

(** The lemma the layer stands on: a real lies between the floats either side of the float it
    rounds to.  [outward] is this, read at each end. *)
Lemma finite_is_no_infinity : forall (x : f64) (s : bool),
  finite x = true -> B2SF x <> SpecFloat.S754_infinity s.
Proof.
  intros x s H E. unfold finite in H. rewrite <- is_finite_SF_B2SF in H.
  rewrite E in H. discriminate.
Qed.

Lemma rounded_is_bracketed : forall (x : f64) (r : R),
  finite x = true -> real x = rnd r ->
  finite (next_down x) = true -> finite (next_up x) = true ->
  real (next_down x) <= r <= real (next_up x).
Proof.
  intros x r Fx Hx Fd Fu. unfold real, rnd, next_down, next_up in *. split.
  - pose proof (Bpred_correct prec emax _ _ x Fx) as H.
    destruct (Rlt_bool _ _).
    + destruct H as [H _]. rewrite H, Hx. apply pred_round_le_id; typeclasses eauto.
    + exfalso. exact (finite_is_no_infinity _ _ Fd H).
  - pose proof (Bsucc_correct prec emax _ _ x Fx) as H.
    destruct (Rlt_bool _ _).
    + destruct H as [H _]. rewrite H, Hx. apply succ_round_ge_id; typeclasses eauto.
    + exfalso. exact (finite_is_no_infinity _ _ Fu H).
Qed.

Theorem new_valid : forall l h c, new l h = Ok c -> valid c /\ lo c = l /\ hi c = h.
Proof.
  intros l h c H. unfold new in H.
  destruct (finite l) eqn:Fl; destruct (finite h) eqn:Fh; simpl in H; try discriminate.
  destruct (flt h l) eqn:L; try discriminate.
  inversion H; subst; simpl. repeat split; auto.
  unfold flt in L. rewrite Bltb_correct in L by assumption.
  destruct (Rlt_bool_spec (B2R h) (B2R l)); [discriminate | assumption].
Qed.

Theorem ineg_encloses : forall a, valid a ->
  valid (ineg a) /\ forall x, holds a x -> holds (ineg a) (- x).
Proof.
  intros a [Fl [Fh Le]]. unfold valid, holds, ineg, finite, real, fneg in *; simpl.
  rewrite !is_finite_Bopp, !B2R_Bopp. repeat split; auto; lra.
Qed.

Theorem iadd_encloses : encloses2 iadd Rplus.
Admitted.

Theorem isub_encloses : encloses2 isub Rminus.
Admitted.

Theorem imul_encloses : encloses2 imul Rmult.
Admitted.

(** A divisor that answers holds no nought, so the quotient is the real one. *)
Theorem idiv_encloses : encloses2 idiv Rdiv.
Admitted.

Theorem isquare_encloses : encloses1 isquare (fun x => x * x).
Admitted.

(** A square's enclosure is never below nought, which [sqrt] of it relies on. *)
Theorem isquare_nonneg : forall a c, valid a -> isquare a = Ok c -> 0 <= real (lo c).
Admitted.

(** A radicand that answers holds nothing below nought, so the root is the real one. *)
Theorem isqrt_encloses : encloses1 isqrt sqrt.
Admitted.

(** ** Trigonometry: the property, awaiting its transcription

    [Interval::sin_cos] (inputs within 8 of nought), [Interval::quarter] (within 0.8) and
    [Interval::sin_cos_periodic] (anywhere) each answer a pair that must hold the sine and the
    cosine of every real the input holds.  The reductions by quarter turns and whole turns, the
    Taylor remainders and the table of [1/n!] are all inside this one property. *)
Definition encloses_sin_cos (sin_cos : interval -> result (interval * interval)) : Prop :=
  forall a s c, valid a -> sin_cos a = Ok (s, c) ->
    valid s /\ valid c /\ forall x, holds a x -> holds s (sin x) /\ holds c (cos x).

(** ** [Iv]: an interval, or every number *)

Definition iv : Type := option interval.

Definition iv_valid (a : iv) : Prop := match a with Some i => valid i | None => True end.

(** The reals it holds: every one, where an operation left its domain. *)
Definition iv_holds (a : iv) (x : R) : Prop :=
  match a with Some i => holds i x | None => True end.

Definition ok {A : Type} (r : result A) : option A :=
  match r with Ok a => Some a | Err _ => None end.

Definition lift2 (op : interval -> interval -> result interval) (a b : iv) : iv :=
  match a, b with Some x, Some y => ok (op x y) | _, _ => None end.

Definition iv_add : iv -> iv -> iv := lift2 iadd.
Definition iv_sub : iv -> iv -> iv := lift2 isub.
Definition iv_mul : iv -> iv -> iv := lift2 imul.
Definition iv_div : iv -> iv -> iv := lift2 idiv.
Definition iv_neg (a : iv) : iv := option_map ineg a.
Definition iv_sqrt (a : iv) : iv := match a with Some x => ok (isqrt x) | None => None end.

(** [Iv::new] (either way round) and [Iv::point]. *)
Definition iv_new (l h : f64) : iv := ok (new (fmin l h) (fmax l h)).
Definition iv_point (x : f64) : iv := ok (point x).

(** [Iv::lo], [Iv::hi], [Iv::mag], [Iv::has_zero]. *)
Definition iv_lo (a : iv) : f64 := match a with Some i => lo i | None => neg_infinity end.
Definition iv_hi (a : iv) : f64 := match a with Some i => hi i | None => infinity end.
Definition iv_mag (a : iv) : f64 :=
  match a with Some i => fmax (fabs (lo i)) (fabs (hi i)) | None => infinity end.
Definition iv_has_zero (a : iv) : bool := fle (iv_lo a) zero && fle zero (iv_hi a).

(** [Iv::hull] and [Iv::meet]. *)
Definition iv_hull (a b : iv) : iv :=
  match a, b with
  | Some x, Some y => ok (new (fmin (lo x) (lo y)) (fmax (hi x) (hi y)))
  | _, _ => None
  end.
Definition iv_meet (a b : iv) : iv :=
  match a, b with
  | None, _ => b
  | _, None => a
  | Some x, Some y =>
      match new (fmax (lo x) (lo y)) (fmin (hi x) (hi y)) with
      | Ok c => Some c
      | Err _ => Some x
      end
  end.

Definition iv_encloses2 (op : iv -> iv -> iv) (f : R -> R -> R) : Prop :=
  forall a b, iv_valid a -> iv_valid b ->
    iv_valid (op a b) /\ forall x y, iv_holds a x -> iv_holds b y -> iv_holds (op a b) (f x y).

Theorem iv_add_encloses : iv_encloses2 iv_add Rplus.
Admitted.

Theorem iv_sub_encloses : iv_encloses2 iv_sub Rminus.
Admitted.

Theorem iv_mul_encloses : iv_encloses2 iv_mul Rmult.
Admitted.

(** Division by an interval holding nought answers every number, so nothing is claimed of a
    quotient that is not a real one. *)
Theorem iv_div_encloses : forall a b, iv_valid a -> iv_valid b ->
  iv_valid (iv_div a b) /\
  forall x y, iv_holds a x -> iv_holds b y -> y <> 0 -> iv_holds (iv_div a b) (x / y).
Admitted.

Theorem iv_new_holds : forall l h x, finite l = true -> finite h = true ->
  (real l <= x <= real h \/ real h <= x <= real l) ->
  iv_valid (iv_new l h) /\ iv_holds (iv_new l h) x.
Admitted.

Theorem iv_point_holds : forall x, finite x = true ->
  iv_valid (iv_point x) /\ iv_holds (iv_point x) (real x).
Admitted.

(** What every test reads an enclosure by: one that holds nought says so. *)
Theorem iv_has_zero_sound : forall a, iv_valid a -> iv_holds a 0 -> iv_has_zero a = true.
Admitted.

(** Nothing an enclosure holds is larger than its [mag]. *)
Theorem iv_mag_bounds : forall a i x, a = Some i -> valid i -> holds i x ->
  Rabs x <= real (iv_mag a).
Admitted.

Theorem iv_hull_holds : forall a b x, iv_valid a -> iv_valid b ->
  iv_holds a x \/ iv_holds b x -> iv_valid (iv_hull a b) /\ iv_holds (iv_hull a b) x.
Admitted.

(** Two enclosures of one number meet in an enclosure of it. *)
Theorem iv_meet_holds : forall a b x, iv_valid a -> iv_valid b ->
  iv_holds a x -> iv_holds b x -> iv_valid (iv_meet a b) /\ iv_holds (iv_meet a b) x.
Admitted.
