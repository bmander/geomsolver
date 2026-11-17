(** * Layer 1 — [util/interval.rs]: every operation encloses.

    The claim of the module's first line, "arithmetic encloses real operations on binary64
    endpoints; invalid domains and overflow fail closed", made exact: an operation that answers
    [Ok c] answers an interval [c] holding the real result for every real its operands hold, and
    an operation that cannot answers [Err].

    The arithmetic ([new] to [sqrt], and [Iv]'s readings) is transcribed below from the Rust and
    its theorems are proved of the transcription, overflow to an infinity included, from what
    [Rounding.v] proves of the floats.  The trigonometry is stated as a property any function
    standing for [sin_cos] must have, its transcription left to the translator, and not proved. *)

From Coq Require Import ZArith Reals Bool List Lra Psatz.
From Flocq Require Import Core BinarySingleNaN.
From Kernel Require Import Float Rounding.
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

(** [new] answers the interval of two finite floats the right way round. *)
Lemma new_ok : forall l h, finite l = true -> finite h = true -> real l <= real h ->
  new l h = Ok (mk l h).
Proof.
  intros l h Fl Fh L. unfold new. rewrite Fl, Fh. simpl.
  rewrite flt_real by assumption. rewrite Rlt_bool_false by exact L. reflexivity.
Qed.

Lemma outward_spec : forall l h c, outward l h = Ok c ->
  valid c /\ lo c = next_down l /\ hi c = next_up h.
Proof.
  intros l h c H. unfold outward in H. cbv zeta in H.
  destruct (negb (finite (next_down l)) || negb (finite (next_up h))); [discriminate|].
  apply new_valid in H. exact H.
Qed.

(** What [outward] is for: it holds every real whose rounding its ends' keys bound. *)
Lemma outward_encloses : forall l h c t, outward l h = Ok c ->
  is_nan l = false -> is_nan h = false -> key l <= cr t -> cr t <= key h -> holds c t.
Proof.
  intros l h c t H Nl Nh Kl Kh.
  destruct (outward_spec _ _ _ H) as [[Fl [Fh _]] [El Eh]].
  unfold holds. rewrite El, Eh in *. split; [apply down_le_key | apply up_ge_key]; assumption.
Qed.

Theorem iadd_encloses : encloses2 iadd Rplus.
Proof.
  intros a b c Va Vb H. unfold iadd in H. split; [exact (proj1 (outward_spec _ _ _ H))|].
  intros x y Hx Hy. destruct Va as [Fal [Fah _]], Vb as [Fbl [Fbh _]].
  destruct (fadd_key (lo a) (lo b) Fal Fbl) as [N1 K1].
  destruct (fadd_key (hi a) (hi b) Fah Fbh) as [N2 K2].
  apply (outward_encloses _ _ _ _ H N1 N2); [rewrite K1 | rewrite K2];
    apply cr_le; unfold holds in *; lra.
Qed.

Theorem isub_encloses : encloses2 isub Rminus.
Proof.
  intros a b c Va Vb H. unfold isub in H. split; [exact (proj1 (outward_spec _ _ _ H))|].
  intros x y Hx Hy. destruct Va as [Fal [Fah _]], Vb as [Fbl [Fbh _]].
  destruct (fsub_key (lo a) (hi b) Fal Fbh) as [N1 K1].
  destruct (fsub_key (hi a) (lo b) Fah Fbl) as [N2 K2].
  apply (outward_encloses _ _ _ _ H N1 N2); [rewrite K1 | rewrite K2];
    apply cr_le; unfold holds in *; lra.
Qed.

Lemma least4_key : forall p q r s : f64,
  is_nan p = false -> is_nan q = false -> is_nan r = false -> is_nan s = false ->
  is_nan (least4 p q r s) = false /\
  key (least4 p q r s) = Rmin (Rmin (Rmin (Rmin top (key p)) (key q)) (key r)) (key s).
Proof.
  intros p q r s Np Nq Nr Ns. unfold least4.
  destruct (fmin_key infinity p eq_refl Np) as [N1 [K1 _]].
  destruct (fmin_key _ q N1 Nq) as [N2 [K2 _]].
  destruct (fmin_key _ r N2 Nr) as [N3 [K3 _]].
  destruct (fmin_key _ s N3 Ns) as [N4 [K4 _]].
  split; [exact N4|]. rewrite K4, K3, K2, K1. reflexivity.
Qed.

Lemma most4_key : forall p q r s : f64,
  is_nan p = false -> is_nan q = false -> is_nan r = false -> is_nan s = false ->
  is_nan (most4 p q r s) = false /\
  key (most4 p q r s) = Rmax (Rmax (Rmax (Rmax (- top) (key p)) (key q)) (key r)) (key s).
Proof.
  intros p q r s Np Nq Nr Ns. unfold most4.
  destruct (fmax_key neg_infinity p eq_refl Np) as [N1 [K1 _]].
  destruct (fmax_key _ q N1 Nq) as [N2 [K2 _]].
  destruct (fmax_key _ r N2 Nr) as [N3 [K3 _]].
  destruct (fmax_key _ s N3 Ns) as [N4 [K4 _]].
  split; [exact N4|]. rewrite K4, K3, K2, K1. reflexivity.
Qed.

(** A product over a box of two reals lies between its values at the corners. *)
Lemma mul_corners : forall la ha lb hb x y, la <= x <= ha -> lb <= y <= hb ->
  (la * lb <= x * y \/ la * hb <= x * y \/ ha * lb <= x * y \/ ha * hb <= x * y) /\
  (x * y <= la * lb \/ x * y <= la * hb \/ x * y <= ha * lb \/ x * y <= ha * hb).
Proof.
  intros la ha lb hb x y [Lx Hx] [Ly Hy]. split.
  - destruct (Rle_dec 0 y); [destruct (Rle_dec 0 la) | destruct (Rle_dec 0 ha)].
    + left; nra.
    + right; left; nra.
    + right; right; left; nra.
    + right; right; right; nra.
  - destruct (Rle_dec 0 y); [destruct (Rle_dec 0 ha) | destruct (Rle_dec 0 la)].
    + right; right; right; nra.
    + right; right; left; nra.
    + right; left; nra.
    + left; nra.
Qed.

(** What the four products' least and greatest bound, read by their keys. *)
Lemma corners_enclose : forall (kp kq kr ks v : R),
  (kp <= v \/ kq <= v \/ kr <= v \/ ks <= v) ->
  (v <= kp \/ v <= kq \/ v <= kr \/ v <= ks) ->
  Rmin (Rmin (Rmin (Rmin top kp) kq) kr) ks <= v /\
  v <= Rmax (Rmax (Rmax (Rmax (- top) kp) kq) kr) ks.
Proof.
  intros kp kq kr ks v L U. split.
  - unfold Rmin; repeat destruct Rle_dec; destruct L as [L|[L|[L|L]]]; lra.
  - unfold Rmax; repeat destruct Rle_dec; destruct U as [U|[U|[U|U]]]; lra.
Qed.

Theorem imul_encloses : encloses2 imul Rmult.
Proof.
  intros a b c Va Vb H. unfold imul in H. cbv zeta in H.
  split; [exact (proj1 (outward_spec _ _ _ H))|].
  intros x y Hx Hy. destruct Va as [Fal [Fah _]], Vb as [Fbl [Fbh _]].
  destruct (fmul_key (lo a) (lo b) Fal Fbl) as [Np Kp].
  destruct (fmul_key (lo a) (hi b) Fal Fbh) as [Nq Kq].
  destruct (fmul_key (hi a) (lo b) Fah Fbl) as [Nr Kr].
  destruct (fmul_key (hi a) (hi b) Fah Fbh) as [Ns Ks].
  destruct (least4_key _ _ _ _ Np Nq Nr Ns) as [NL KL].
  destruct (most4_key _ _ _ _ Np Nq Nr Ns) as [NM KM].
  destruct (mul_corners _ _ _ _ x y Hx Hy) as [Lo Hi].
  destruct (corners_enclose (cr (real (lo a) * real (lo b))) (cr (real (lo a) * real (hi b)))
              (cr (real (hi a) * real (lo b))) (cr (real (hi a) * real (hi b))) (cr (x * y)))
    as [B1 B2].
  - destruct Lo as [C|[C|[C|C]]]; apply cr_le in C; auto.
  - destruct Hi as [C|[C|[C|C]]]; apply cr_le in C; auto.
  - apply (outward_encloses _ _ _ _ H NL NM); [rewrite KL | rewrite KM];
      rewrite Kp, Kq, Kr, Ks; assumption.
Qed.

(** A divisor that holds no nought is all above it or all below. *)
Lemma not_contains_zero : forall b, valid b -> contains b zero = false ->
  0 < real (lo b) \/ real (hi b) < 0.
Proof.
  intros b [Fl [Fh _]] C. unfold contains in C.
  rewrite !fle_real in C by first [assumption | reflexivity]. rewrite real_zero in C.
  destruct (Rle_bool_spec (real (lo b)) 0); destruct (Rle_bool_spec 0 (real (hi b)));
    simpl in C; try discriminate; lra.
Qed.

(** Over such a divisor the reciprocal runs between its ends' reciprocals, the other way. *)
Lemma inv_between : forall l h y, 0 < l \/ h < 0 -> l <= y <= h -> / h <= / y <= / l.
Proof.
  intros l h y S [L H].
  assert (0 < l * y /\ 0 < y * h) as [P Q] by (destruct S; split; nra).
  assert (l <> 0 /\ y <> 0 /\ h <> 0) as [Nl [Ny Nh]] by (destruct S; repeat split; lra).
  split.
  - assert (0 < / (y * h)) by (apply Rinv_0_lt_compat; exact Q).
    replace (/ h) with (y * / (y * h)) by (field; repeat split; assumption).
    replace (/ y) with (h * / (y * h)) by (field; repeat split; assumption). nra.
  - assert (0 < / (l * y)) by (apply Rinv_0_lt_compat; exact P).
    replace (/ y) with (l * / (l * y)) by (field; repeat split; assumption).
    replace (/ l) with (y * / (l * y)) by (field; repeat split; assumption). nra.
Qed.

(** A divisor that answers holds no nought, so the quotient is the real one. *)
Theorem idiv_encloses : encloses2 idiv Rdiv.
Proof.
  intros a b c Va Vb H. unfold idiv in H.
  destruct (contains b zero) eqn:Cz; [discriminate|]. cbv zeta in H.
  split; [exact (proj1 (outward_spec _ _ _ H))|].
  pose proof (not_contains_zero b Vb Cz) as S.
  intros x y Hx Hy. destruct Va as [Fal [Fah _]], Vb as [Fbl [Fbh Lb]].
  assert (real (lo b) <> 0 /\ real (hi b) <> 0) as [Nlb Nhb] by (destruct S; split; lra).
  destruct (fdiv_key (lo a) (lo b) Fal Fbl Nlb) as [Np Kp].
  destruct (fdiv_key (lo a) (hi b) Fal Fbh Nhb) as [Nq Kq].
  destruct (fdiv_key (hi a) (lo b) Fah Fbl Nlb) as [Nr Kr].
  destruct (fdiv_key (hi a) (hi b) Fah Fbh Nhb) as [Ns Ks].
  destruct (least4_key _ _ _ _ Np Nq Nr Ns) as [NL KL].
  destruct (most4_key _ _ _ _ Np Nq Nr Ns) as [NM KM].
  pose proof (inv_between _ _ y S Hy) as Hi.
  destruct (mul_corners _ _ _ _ x (/ y) Hx Hi) as [Lo Up].
  destruct (corners_enclose (cr (real (lo a) / real (lo b))) (cr (real (lo a) / real (hi b)))
              (cr (real (hi a) / real (lo b))) (cr (real (hi a) / real (hi b))) (cr (x / y)))
    as [B1 B2].
  - unfold Rdiv. destruct Lo as [C|[C|[C|C]]]; apply cr_le in C; auto.
  - unfold Rdiv. destruct Up as [C|[C|[C|C]]]; apply cr_le in C; auto.
  - apply (outward_encloses _ _ _ _ H NL NM); [rewrite KL | rewrite KM];
      rewrite Kp, Kq, Kr, Ks; assumption.
Qed.

(** An interval's lower end raised to nought, where nothing it must hold is below nought. *)
Lemma raised_to_nought : forall c0, valid c0 -> 0 <= real (hi c0) ->
  valid (mk (fmax (lo c0) zero) (hi c0)) /\
  0 <= real (fmax (lo c0) zero) /\
  forall t, holds c0 t -> 0 <= t -> holds (mk (fmax (lo c0) zero) (hi c0)) t.
Proof.
  intros c0 [Fl [Fh L]] P. destruct (fmax_finite (lo c0) zero Fl eq_refl) as [F E].
  rewrite real_zero in E. unfold valid, holds; simpl. rewrite E.
  repeat split; auto.
  - unfold Rmax; destruct Rle_dec; lra.
  - apply Rmax_r.
  - unfold Rmax; destruct Rle_dec; destruct H; lra.
  - destruct H; lra.
Qed.

(** [square], before its lower end is raised: what [outward] answers holds every square. *)
Lemma isquare_outward : forall a c0, valid a ->
  outward (if contains a zero then zero else fmin (fmul (lo a) (lo a)) (fmul (hi a) (hi a)))
          (fmax (fmul (lo a) (lo a)) (fmul (hi a) (hi a))) = Ok c0 ->
  valid c0 /\ forall x, holds a x -> holds c0 (x * x).
Proof.
  intros a c0 Va O. split; [exact (proj1 (outward_spec _ _ _ O))|].
  intros x Hx. pose proof Va as [Fl [Fh L]].
  destruct (fmul_key (lo a) (lo a) Fl Fl) as [Np Kp].
  destruct (fmul_key (hi a) (hi a) Fh Fh) as [Nq Kq].
  destruct (fmin_key _ _ Np Nq) as [Nmin [Kmin _]].
  destruct (fmax_key _ _ Np Nq) as [Nmax [Kmax _]].
  unfold holds in Hx.
  assert (cr (x * x) <= key (fmax (fmul (lo a) (lo a)) (fmul (hi a) (hi a)))) as Up.
  { rewrite Kmax, Kp, Kq. destruct (Rle_dec 0 x).
    - apply Rle_trans with (cr (real (hi a) * real (hi a))); [apply cr_le; nra | apply Rmax_r].
    - apply Rle_trans with (cr (real (lo a) * real (lo a))); [apply cr_le; nra | apply Rmax_l]. }
  destruct (contains a zero) eqn:Cz.
  - apply (outward_encloses _ _ _ _ O eq_refl Nmax); [|exact Up].
    change (key zero) with 0. rewrite <- cr_0. apply cr_le. nra.
  - apply (outward_encloses _ _ _ _ O Nmin Nmax); [|exact Up].
    rewrite Kmin, Kp, Kq. destruct (not_contains_zero a Va Cz).
    + apply Rle_trans with (cr (real (lo a) * real (lo a))); [apply Rmin_l | apply cr_le; nra].
    + apply Rle_trans with (cr (real (hi a) * real (hi a))); [apply Rmin_r | apply cr_le; nra].
Qed.

Theorem isquare_encloses : encloses1 isquare (fun x => x * x).
Proof.
  intros a c Va H. unfold isquare in H. cbv zeta in H. unfold bind in H.
  destruct (outward _ _) as [c0|] eqn:O; [|discriminate]. inversion H; subst c; clear H.
  destruct (isquare_outward a c0 Va O) as [V0 E0].
  assert (0 <= real (hi c0)) as P.
  { destruct Va as [_ [_ L]]. assert (holds a (real (lo a))) as Hl by (unfold holds; lra).
    destruct (E0 _ Hl) as [_ U]. nra. }
  destruct (raised_to_nought c0 V0 P) as [V [_ E]]. split; [exact V|].
  intros x Hx. apply E; [apply E0; exact Hx | nra].
Qed.

(** A square's enclosure is never below nought, which [sqrt] of it relies on. *)
Theorem isquare_nonneg : forall a c, valid a -> isquare a = Ok c -> 0 <= real (lo c).
Proof.
  intros a c Va H. unfold isquare in H. cbv zeta in H. unfold bind in H.
  destruct (outward _ _) as [c0|] eqn:O; [|discriminate]. inversion H; subst c; clear H.
  destruct (isquare_outward a c0 Va O) as [V0 E0].
  assert (0 <= real (hi c0)) as P.
  { destruct Va as [_ [_ L]]. assert (holds a (real (lo a))) as Hl by (unfold holds; lra).
    destruct (E0 _ Hl) as [_ U]. nra. }
  destruct (raised_to_nought c0 V0 P) as [_ [N _]]. exact N.
Qed.

(** A radicand that answers holds nothing below nought, so the root is the real one. *)
Theorem isqrt_encloses : encloses1 isqrt sqrt.
Proof.
  intros a c Va H. unfold isqrt in H. pose proof Va as [Fl [Fh L]].
  destruct (flt (lo a) zero) eqn:Neg; [discriminate|].
  rewrite flt_real in Neg by first [assumption | reflexivity]. rewrite real_zero in Neg.
  assert (0 <= real (lo a)) as P0 by (destruct (Rlt_bool_spec (real (lo a)) 0); [discriminate | assumption]).
  unfold bind in H. destruct (outward _ _) as [c0|] eqn:O; [|discriminate].
  inversion H; subst c; clear H.
  assert (0 <= real (hi a)) as P1 by lra.
  destruct (fsqrt_key (lo a) Fl P0) as [N1 K1]. destruct (fsqrt_key (hi a) Fh P1) as [N2 K2].
  assert (forall x, holds a x -> holds c0 (sqrt x)) as E0.
  { intros x [Lx Hx]. apply (outward_encloses _ _ _ _ O N1 N2); [rewrite K1 | rewrite K2];
      apply cr_le, sqrt_le_1_alt; assumption. }
  assert (0 <= real (hi c0)) as P.
  { assert (holds a (real (lo a))) as Hl by (unfold holds; lra).
    destruct (E0 _ Hl) as [_ U]. pose proof (sqrt_pos (real (lo a))). lra. }
  destruct (raised_to_nought c0 (proj1 (outward_spec _ _ _ O)) P) as [V [_ E]].
  split; [exact V|]. intros x Hx. apply E; [apply E0; exact Hx | apply sqrt_pos].
Qed.

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

Lemma lift2_encloses : forall op f, encloses2 op f -> iv_encloses2 (lift2 op) f.
Proof.
  intros op f E a b Va Vb. destruct a as [i|], b as [j|]; simpl; auto.
  destruct (op i j) as [c|] eqn:O; simpl; [|auto].
  destruct (E i j c Va Vb O) as [V H]. split; [exact V | exact H].
Qed.

Theorem iv_add_encloses : iv_encloses2 iv_add Rplus.
Proof. exact (lift2_encloses _ _ iadd_encloses). Qed.

Theorem iv_sub_encloses : iv_encloses2 iv_sub Rminus.
Proof. exact (lift2_encloses _ _ isub_encloses). Qed.

Theorem iv_mul_encloses : iv_encloses2 iv_mul Rmult.
Proof. exact (lift2_encloses _ _ imul_encloses). Qed.

(** Division by an interval holding nought answers every number, so nothing is claimed of a
    quotient that is not a real one. *)
Theorem iv_div_encloses : forall a b, iv_valid a -> iv_valid b ->
  iv_valid (iv_div a b) /\
  forall x y, iv_holds a x -> iv_holds b y -> y <> 0 -> iv_holds (iv_div a b) (x / y).
Proof.
  intros a b Va Vb. destruct (lift2_encloses _ _ idiv_encloses a b Va Vb) as [V H].
  split; [exact V|]. intros x y Hx Hy _. apply H; assumption.
Qed.

Theorem iv_new_holds : forall l h x, finite l = true -> finite h = true ->
  (real l <= x <= real h \/ real h <= x <= real l) ->
  iv_valid (iv_new l h) /\ iv_holds (iv_new l h) x.
Proof.
  intros l h x Fl Fh B. unfold iv_new.
  destruct (fmin_finite l h Fl Fh) as [F1 E1]. destruct (fmax_finite l h Fl Fh) as [F2 E2].
  assert (real (fmin l h) <= real (fmax l h)) as L
    by (rewrite E1, E2; apply Rle_trans with (real l); [apply Rmin_l | apply Rmax_l]).
  rewrite (new_ok _ _ F1 F2 L). simpl. unfold valid, holds; simpl. rewrite E1, E2 in *.
  repeat split; auto; unfold Rmin, Rmax; repeat destruct Rle_dec; destruct B as [[? ?]|[? ?]]; lra.
Qed.

Theorem iv_point_holds : forall x, finite x = true ->
  iv_valid (iv_point x) /\ iv_holds (iv_point x) (real x).
Proof.
  intros x F. unfold iv_point, point. rewrite (new_ok _ _ F F (Rle_refl _)). simpl.
  unfold valid, holds; simpl. repeat split; auto; lra.
Qed.

(** What every test reads an enclosure by: one that holds nought says so. *)
Theorem iv_has_zero_sound : forall a, iv_valid a -> iv_holds a 0 -> iv_has_zero a = true.
Proof.
  intros a Va H. unfold iv_has_zero. destruct a as [i|]; simpl in *; [|reflexivity].
  destruct Va as [Fl [Fh _]]. destruct H as [Hl Hh].
  rewrite !fle_real by first [assumption | reflexivity]. rewrite real_zero.
  rewrite !Rle_bool_true by assumption. reflexivity.
Qed.

(** Nothing an enclosure holds is larger than its [mag]. *)
Theorem iv_mag_bounds : forall a i x, a = Some i -> valid i -> holds i x ->
  Rabs x <= real (iv_mag a).
Proof.
  intros a i x -> [Fl [Fh L]] [Hl Hh]. simpl.
  assert (finite (fabs (lo i)) = true) as F1 by (unfold finite, fabs; rewrite is_finite_Babs; exact Fl).
  assert (finite (fabs (hi i)) = true) as F2 by (unfold finite, fabs; rewrite is_finite_Babs; exact Fh).
  destruct (fmax_finite _ _ F1 F2) as [_ E]. rewrite E. unfold real, fabs. rewrite !B2R_Babs.
  fold (real (lo i)) (real (hi i)).
  unfold Rmax, Rabs; repeat destruct Rle_dec; repeat destruct Rcase_abs; lra.
Qed.

Theorem iv_hull_holds : forall a b x, iv_valid a -> iv_valid b ->
  iv_holds a x \/ iv_holds b x -> iv_valid (iv_hull a b) /\ iv_holds (iv_hull a b) x.
Proof.
  intros a b x Va Vb H. destruct a as [i|], b as [j|]; simpl; auto.
  destruct Va as [Fil [Fih Li]], Vb as [Fjl [Fjh Lj]].
  destruct (fmin_finite (lo i) (lo j) Fil Fjl) as [F1 E1].
  destruct (fmax_finite (hi i) (hi j) Fih Fjh) as [F2 E2].
  assert (real (fmin (lo i) (lo j)) <= real (fmax (hi i) (hi j))) as L
    by (rewrite E1, E2; unfold Rmin, Rmax; repeat destruct Rle_dec; lra).
  rewrite (new_ok _ _ F1 F2 L). simpl in *. unfold valid, holds in *; simpl. rewrite E1, E2 in *.
  repeat split; auto; unfold Rmin, Rmax; repeat destruct Rle_dec; destruct H as [[? ?]|[? ?]]; lra.
Qed.

(** Two enclosures of one number meet in an enclosure of it. *)
Theorem iv_meet_holds : forall a b x, iv_valid a -> iv_valid b ->
  iv_holds a x -> iv_holds b x -> iv_valid (iv_meet a b) /\ iv_holds (iv_meet a b) x.
Proof.
  intros a b x Va Vb Ha Hb. destruct a as [i|], b as [j|]; simpl in *; auto.
  destruct (new (fmax (lo i) (lo j)) (fmin (hi i) (hi j))) as [c|] eqn:N; simpl; [|split; assumption].
  destruct (new_valid _ _ _ N) as [V [El Eh]]. split; [exact V|].
  destruct Va as [Fil [Fih _]], Vb as [Fjl [Fjh _]].
  destruct (fmax_finite (lo i) (lo j) Fil Fjl) as [_ E1].
  destruct (fmin_finite (hi i) (hi j) Fih Fjh) as [_ E2].
  unfold holds in *. rewrite El, Eh, E1, E2.
  unfold Rmin, Rmax; repeat destruct Rle_dec; lra.
Qed.
