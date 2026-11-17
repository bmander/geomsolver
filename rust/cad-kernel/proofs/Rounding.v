(** * What the interval proofs need of the floats.

    An operation on finite floats answers the rounded exact result, or an infinity where that
    overflows.  [key] reads any float but a NaN as a real, an infinity as the power of two no
    finite float reaches, so that the order of floats is the order of their keys and a rounded
    operation's key is its exact result rounded and clamped ([cr]), which is monotone.  Moving a
    float one outward then brackets every real whose [cr] it bounds ([down_le_key],
    [up_ge_key]): the whole of why [Interval::outward] is sound, overflow included. *)

From Coq Require Import ZArith Reals Bool Lra Psatz.
From Flocq Require Import Core BinarySingleNaN.
From Kernel Require Import Float.
Open Scope R_scope.

(** The power of two past every finite float. *)
Definition top : R := bpow radix2 emax.

Lemma top_pos : 0 < top.
Proof. apply bpow_gt_0. Qed.

Definition clamp (v : R) : R := Rmax (- top) (Rmin top v).

(** An exact result, rounded and clamped: what a rounded operation's key is. *)
Definition cr (t : R) : R := clamp (rnd t).

Definition key (z : f64) : R :=
  match z with
  | B754_infinity false => top
  | B754_infinity true => - top
  | _ => real z
  end.

Lemma rnd_le : forall a b, a <= b -> rnd a <= rnd b.
Proof. intros a b H. apply round_le; [typeclasses eauto | typeclasses eauto | exact H]. Qed.

Lemma rnd_0 : rnd 0 = 0.
Proof. apply round_0. typeclasses eauto. Qed.

Lemma rnd_real : forall x : f64, rnd (real x) = real x.
Proof. intros x. apply round_generic; [typeclasses eauto | apply generic_format_B2R]. Qed.

Lemma clamp_le : forall a b, a <= b -> clamp a <= clamp b.
Proof.
  intros a b H. pose proof top_pos. unfold clamp, Rmax, Rmin.
  repeat destruct Rle_dec; lra.
Qed.

Lemma cr_le : forall a b, a <= b -> cr a <= cr b.
Proof. intros a b H. apply clamp_le, rnd_le, H. Qed.

Lemma clamp_id : forall v, - top < v < top -> clamp v = v.
Proof. intros v H. unfold clamp, Rmax, Rmin. repeat destruct Rle_dec; lra. Qed.

Lemma real_lt_top : forall x : f64, - top < real x < top.
Proof.
  intros x. pose proof (abs_B2R_lt_emax prec emax x) as H.
  apply Rabs_def2 in H. unfold real, top. lra.
Qed.

Lemma cr_0 : cr 0 = 0.
Proof. unfold cr. rewrite rnd_0. apply clamp_id. pose proof top_pos. lra. Qed.

Lemma finite_not_nan : forall z : f64, finite z = true -> is_nan z = false.
Proof. intros z. destruct z; simpl; auto; discriminate. Qed.

Lemma key_finite : forall z, finite z = true -> key z = real z.
Proof. intros z. destruct z as [s|s| |s m e H]; simpl; auto; discriminate. Qed.

Lemma key_bounds : forall z, - top <= key z <= top.
Proof.
  intros z. pose proof top_pos as T. pose proof (real_lt_top z) as L.
  destruct z as [s|[|]| |s m e Hb]; cbv [key] in *; lra.
Qed.

(** A finite float that is a rounding has that rounding's key. *)
Lemma key_rounded : forall (z : f64) (r : R),
  finite z = true -> real z = rnd r -> key z = cr r.
Proof.
  intros z r F E. rewrite key_finite by exact F. unfold cr. rewrite <- E.
  symmetry. apply clamp_id, real_lt_top.
Qed.

(** ** The order of floats is the order of their keys *)

Lemma flt_key : forall a b : f64, is_nan a = false -> is_nan b = false ->
  flt a b = Rlt_bool (key a) (key b).
Proof.
  intros a b Na Nb. pose proof top_pos as T.
  pose proof (real_lt_top a) as La. pose proof (real_lt_top b) as Lb.
  destruct (finite a) eqn:Fa; destruct (finite b) eqn:Fb.
  - rewrite !key_finite by assumption. apply Bltb_correct; assumption.
  - rewrite (key_finite a Fa). set (ra := real a) in *. clearbody ra.
    destruct b as [sb|[|]| |sb mb eb Hb]; try discriminate; cbv [key];
      destruct a as [sa|sa| |sa ma ea Ha]; try discriminate; unfold flt, Bltb; simpl;
      symmetry; first [apply Rlt_bool_false; lra | apply Rlt_bool_true; lra].
  - rewrite (key_finite b Fb). set (rb := real b) in *. clearbody rb.
    destruct a as [sa|[|]| |sa ma ea Ha]; try discriminate; cbv [key];
      destruct b as [sb|sb| |sb mb eb Hb]; try discriminate; unfold flt, Bltb; simpl;
      symmetry; first [apply Rlt_bool_false; lra | apply Rlt_bool_true; lra].
  - destruct a as [sa|[|]| |sa ma ea Ha]; try discriminate;
      destruct b as [sb|[|]| |sb mb eb Hb]; try discriminate; cbv [key]; unfold flt, Bltb; simpl;
      symmetry; first [apply Rlt_bool_false; lra | apply Rlt_bool_true; lra].
Qed.

Lemma fmin_key : forall a b : f64, is_nan a = false -> is_nan b = false ->
  is_nan (fmin a b) = false /\ key (fmin a b) = Rmin (key a) (key b) /\ (fmin a b = a \/ fmin a b = b).
Proof.
  intros a b Na Nb. unfold fmin. rewrite Na, Nb, (flt_key b a Nb Na).
  destruct (Rlt_bool_spec (key b) (key a)) as [L|L]; unfold Rmin; destruct Rle_dec; auto;
    repeat split; auto; lra.
Qed.

Lemma fmax_key : forall a b : f64, is_nan a = false -> is_nan b = false ->
  is_nan (fmax a b) = false /\ key (fmax a b) = Rmax (key a) (key b) /\ (fmax a b = a \/ fmax a b = b).
Proof.
  intros a b Na Nb. unfold fmax. rewrite Na, Nb, (flt_key a b Na Nb).
  destruct (Rlt_bool_spec (key a) (key b)) as [L|L]; unfold Rmax; destruct Rle_dec; auto;
    repeat split; auto; lra.
Qed.

(** ** A rounded operation's key is its exact result, rounded and clamped *)

Lemma sign_real : forall x : f64,
  (Bsign x = false -> 0 <= real x) /\ (Bsign x = true -> real x <= 0).
Proof.
  intros x. unfold real. destruct x as [s|s| |s m e Hb]; simpl; try (split; intros; lra).
  destruct s; split; intro E; try discriminate.
  - apply F2R_le_0. simpl. lia.
  - apply F2R_ge_0. simpl. lia.
Qed.

(** Where the rounding overflows, the float is the infinity of the result's sign. *)
Lemma overflow_cr : forall (z : f64) (s : bool) (r : R),
  B2SF z = binary_overflow prec emax mode_NE s ->
  top <= Rabs (rnd r) -> (s = false -> 0 <= r) -> (s = true -> r <= 0) ->
  is_nan z = false /\ key z = cr r.
Proof.
  intros z s r E A P Q. pose proof top_pos as T.
  assert (z = B754_infinity s) as -> by (apply B2SF_inj; rewrite E; reflexivity).
  split; [reflexivity|]. unfold cr, clamp, Rmax, Rmin.
  destruct s; cbv [key].
  - assert (rnd r <= 0) by (rewrite <- rnd_0; apply rnd_le; auto).
    unfold Rabs in A; destruct Rcase_abs in A; repeat destruct Rle_dec; lra.
  - assert (0 <= rnd r) by (rewrite <- rnd_0; apply rnd_le; auto).
    unfold Rabs in A; destruct Rcase_abs in A; repeat destruct Rle_dec; lra.
Qed.

Lemma fadd_key : forall x y : f64, finite x = true -> finite y = true ->
  is_nan (fadd x y) = false /\ key (fadd x y) = cr (real x + real y).
Proof.
  intros x y Fx Fy. pose proof (Bplus_correct prec emax _ _ mode_NE x y Fx Fy) as H.
  change (bpow radix2 emax) with top in H.
  change (round radix2 (SpecFloat.fexp prec emax) (round_mode mode_NE) (B2R x + B2R y))
    with (rnd (real x + real y)) in H.
  destruct (Rlt_bool_spec (Rabs (rnd (real x + real y))) top) as [L|L].
  - destruct H as [E [F _]]. split; [apply finite_not_nan, F | apply key_rounded; assumption].
  - destruct H as [E S]. apply (overflow_cr _ (Bsign x)); auto; intro Sx;
      pose proof (sign_real x) as [Px Nx]; pose proof (sign_real y) as [Py Ny];
      rewrite <- S in Py, Ny; specialize (Px) ; specialize (Py); specialize (Nx); specialize (Ny);
      [specialize (Px Sx); specialize (Py Sx) | specialize (Nx Sx); specialize (Ny Sx)]; lra.
Qed.

Lemma fsub_key : forall x y : f64, finite x = true -> finite y = true ->
  is_nan (fsub x y) = false /\ key (fsub x y) = cr (real x - real y).
Proof.
  intros x y Fx Fy. pose proof (Bminus_correct prec emax _ _ mode_NE x y Fx Fy) as H.
  change (bpow radix2 emax) with top in H.
  change (round radix2 (SpecFloat.fexp prec emax) (round_mode mode_NE) (B2R x - B2R y))
    with (rnd (real x - real y)) in H.
  destruct (Rlt_bool_spec (Rabs (rnd (real x - real y))) top) as [L|L].
  - destruct H as [E [F _]]. split; [apply finite_not_nan, F | apply key_rounded; assumption].
  - destruct H as [E S]. apply (overflow_cr _ (Bsign x)); auto; intro Sx;
      pose proof (sign_real x) as [Px Nx]; pose proof (sign_real y) as [Py Ny];
      rewrite Sx in S; destruct (Bsign y); try discriminate;
      [specialize (Px Sx); specialize (Ny eq_refl) | specialize (Nx Sx); specialize (Py eq_refl)]; lra.
Qed.

Lemma fmul_key : forall x y : f64, finite x = true -> finite y = true ->
  is_nan (fmul x y) = false /\ key (fmul x y) = cr (real x * real y).
Proof.
  intros x y Fx Fy. pose proof (Bmult_correct prec emax _ _ mode_NE x y) as H.
  change (bpow radix2 emax) with top in H.
  change (round radix2 (SpecFloat.fexp prec emax) (round_mode mode_NE) (B2R x * B2R y))
    with (rnd (real x * real y)) in H.
  destruct (Rlt_bool_spec (Rabs (rnd (real x * real y))) top) as [L|L].
  - destruct H as [E [F _]]. unfold finite in Fx, Fy. rewrite Fx, Fy in F.
    split; [apply finite_not_nan, F | apply key_rounded; assumption].
  - pose proof (sign_real x) as [Px Nx]; pose proof (sign_real y) as [Py Ny].
    apply (overflow_cr _ (xorb (Bsign x) (Bsign y))); auto; intro S;
      destruct (Bsign x), (Bsign y); try discriminate;
      repeat match goal with K : ?a = ?a -> _ |- _ => specialize (K eq_refl) end; nra.
Qed.

Lemma fdiv_key : forall x y : f64, finite x = true -> finite y = true -> real y <> 0 ->
  is_nan (fdiv x y) = false /\ key (fdiv x y) = cr (real x / real y).
Proof.
  intros x y Fx Fy Ny0. pose proof (Bdiv_correct prec emax _ _ mode_NE x y Ny0) as H.
  change (bpow radix2 emax) with top in H.
  change (round radix2 (SpecFloat.fexp prec emax) (round_mode mode_NE) (B2R x / B2R y))
    with (rnd (real x / real y)) in H.
  destruct (Rlt_bool_spec (Rabs (rnd (real x / real y))) top) as [L|L].
  - destruct H as [E [F _]]. unfold finite in Fx. rewrite Fx in F.
    split; [apply finite_not_nan, F | apply key_rounded; assumption].
  - pose proof (sign_real x) as [Px Nx]; pose proof (sign_real y) as [Py Ny].
    apply (overflow_cr _ (xorb (Bsign x) (Bsign y))); auto; intro S;
      destruct (Bsign x), (Bsign y); try discriminate;
      repeat match goal with K : ?a = ?a -> _ |- _ => specialize (K eq_refl) end;
      unfold Rdiv;
      first [ assert (0 < / real y) by (apply Rinv_0_lt_compat; lra)
            | assert (/ real y < 0) by (apply Rinv_lt_0_compat; lra) ]; nra.
Qed.

Lemma fsqrt_key : forall x : f64, finite x = true -> 0 <= real x ->
  is_nan (fsqrt x) = false /\ key (fsqrt x) = cr (sqrt (real x)).
Proof.
  intros x Fx P. pose proof (Bsqrt_correct prec emax _ _ mode_NE x) as [E [F _]].
  assert (finite (fsqrt x) = true) as Fz.
  { unfold finite, fsqrt. rewrite F. unfold real in P.
    destruct x as [s|s| |s m e Hb]; try discriminate; auto.
    destruct s; auto. exfalso. simpl in P.
    assert (F2R (Float radix2 (Z.neg m) e) < 0) by (apply F2R_lt_0; simpl; lia). lra. }
  split; [apply finite_not_nan, Fz | apply key_rounded; assumption].
Qed.

(** ** One float outward brackets every real the key bounds *)

Lemma clamp_ge_real : forall (z : f64) (v : R), real z <= clamp v -> real z <= v.
Proof.
  intros z v H. pose proof (real_lt_top z). unfold clamp, Rmax, Rmin in H.
  repeat destruct Rle_dec; lra.
Qed.

Lemma clamp_le_real : forall (z : f64) (v : R), clamp v <= real z -> v <= real z.
Proof.
  intros z v H. pose proof (real_lt_top z). unfold clamp, Rmax, Rmin in H.
  repeat destruct Rle_dec; lra.
Qed.

Lemma down_le_key : forall (z : f64) (t : R),
  is_nan z = false -> finite (next_down z) = true -> key z <= cr t ->
  real (next_down z) <= t.
Proof.
  intros z t N F K. pose proof top_pos as T.
  destruct (finite z) eqn:Fz.
  - rewrite key_finite in K by exact Fz. apply clamp_ge_real in K.
    pose proof (Bpred_correct prec emax _ _ z Fz) as H.
    destruct (Rlt_bool _ _).
    + destruct H as [H _]. unfold real, next_down. rewrite H.
      apply Rle_trans with (pred radix2 (SpecFloat.fexp prec emax) (rnd t)).
      * apply pred_le; [typeclasses eauto | apply generic_format_B2R
                        | apply generic_format_round; typeclasses eauto | exact K].
      * apply pred_round_le_id; typeclasses eauto.
    + exfalso. unfold finite, next_down in F. rewrite <- is_finite_SF_B2SF in F.
      rewrite H in F. discriminate.
  - (* an infinity: below the lower one stands no finite float, and what stands below the
       upper one is below every real that rounds past the largest *)
    destruct z as [s|[|]| |s m e Hb]; try discriminate.
    cbv [key] in K. set (m := next_down (B754_infinity false)) in *.
    pose proof (real_lt_top m) as Lm.
    destruct (Rle_dec (real m) t) as [|G]; auto. exfalso.
    assert (rnd t <= real m) by (rewrite <- (rnd_real m); apply rnd_le; lra).
    unfold cr, clamp, Rmax, Rmin in K; repeat destruct Rle_dec; lra.
Qed.

Lemma up_ge_key : forall (z : f64) (t : R),
  is_nan z = false -> finite (next_up z) = true -> cr t <= key z ->
  t <= real (next_up z).
Proof.
  intros z t N F K. pose proof top_pos as T.
  destruct (finite z) eqn:Fz.
  - rewrite key_finite in K by exact Fz. apply clamp_le_real in K.
    pose proof (Bsucc_correct prec emax _ _ z Fz) as H.
    destruct (Rlt_bool _ _).
    + destruct H as [H _]. unfold real, next_up. rewrite H.
      apply Rle_trans with (succ radix2 (SpecFloat.fexp prec emax) (rnd t)).
      * apply succ_round_ge_id; typeclasses eauto.
      * apply succ_le; [typeclasses eauto | apply generic_format_round; typeclasses eauto
                        | apply generic_format_B2R | exact K].
    + exfalso. unfold finite, next_up in F. rewrite <- is_finite_SF_B2SF in F.
      rewrite H in F. discriminate.
  - destruct z as [s|[|]| |s m e Hb]; try discriminate.
    cbv [key] in K. set (m := next_up (B754_infinity true)) in *.
    pose proof (real_lt_top m) as Lm.
    destruct (Rle_dec t (real m)) as [|G]; auto. exfalso.
    assert (real m <= rnd t) by (rewrite <- (rnd_real m); apply rnd_le; lra).
    unfold cr, clamp, Rmax, Rmin in K; repeat destruct Rle_dec; lra.
Qed.

(** ** Finite floats: the least and the greater of two, comparisons, magnitudes *)

Lemma fmin_finite : forall a b : f64, finite a = true -> finite b = true ->
  finite (fmin a b) = true /\ real (fmin a b) = Rmin (real a) (real b).
Proof.
  intros a b Fa Fb.
  destruct (fmin_key a b (finite_not_nan a Fa) (finite_not_nan b Fb)) as [_ [K [E|E]]];
    rewrite !key_finite in K; rewrite ?E; auto; rewrite E in K; auto.
Qed.

Lemma fmax_finite : forall a b : f64, finite a = true -> finite b = true ->
  finite (fmax a b) = true /\ real (fmax a b) = Rmax (real a) (real b).
Proof.
  intros a b Fa Fb.
  destruct (fmax_key a b (finite_not_nan a Fa) (finite_not_nan b Fb)) as [_ [K [E|E]]];
    rewrite !key_finite in K; rewrite ?E; auto; rewrite E in K; auto.
Qed.

Lemma real_zero : real zero = 0.
Proof. reflexivity. Qed.

Lemma fle_real : forall a b : f64, finite a = true -> finite b = true ->
  fle a b = Rle_bool (real a) (real b).
Proof. intros a b Fa Fb. apply Bleb_correct; assumption. Qed.

Lemma flt_real : forall a b : f64, finite a = true -> finite b = true ->
  flt a b = Rlt_bool (real a) (real b).
Proof. intros a b Fa Fb. apply Bltb_correct; assumption. Qed.
