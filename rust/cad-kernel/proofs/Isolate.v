(** * Layer 3 — [util/isolate.rs]: what a certified search may claim.

    The module's claim is "no root is missed … and every root returned proved … or the search
    refused".  Made exact, it is three kinds of statement:

    - what a caller owes the search: enclosures that enclose ([encloses_function],
      [encloses_system]), the only hypotheses anything below rests on;
    - what a test proves of one stretch or one box ([slope_read_sound], [krawczyk_read_proves]);
    - what a whole search proves ([solve_empty_means_none], [solve_crossings_are_roots],
      [solve_misses_none], [roots_empty_means_none]).

    Two of the last are **not what the Rust does today**, each shown by a run of [gaps/]; they
    are stated as the module's documentation gives them, so the difference is on the page:
    [solve_misses_none] (a box is dropped whole for a run that covers part of it) and
    [roots_empty_means_none] (in one unknown a stretch nothing proved is dropped, where in
    several the search refuses).

    A function's slope is asked for in the mean-value form, between any two points of a
    stretch, and not as a derivative: it is what the tests use, and it holds across a kink.

    [Slope]'s test is transcribed and its theorem stated of the transcription.  The Krawczyk
    test and the searches are stated as properties any function standing for them must have. *)

From Coq Require Import ZArith Reals Bool List.
From Flocq Require Import Core BinarySingleNaN.
From Kernel Require Import Float Interval.
Open Scope R_scope.

(** ** One unknown *)

(** [trait Function]: [at] (value and rate in floats), [value], [slope], [within]. *)
Record function1 : Type := {
  f_at : f64 -> f64 * f64;
  f_value : f64 -> iv;
  f_slope : f64 -> f64 -> iv;
  f_within : f64 -> f64 -> bool
}.

(** What a [Function] owes: over [[t0, t1]], [value] holds the real function [G] at each float,
    and [slope] over a stretch holds the slope of [G] between any two points of it. *)
Definition encloses_function (g : function1) (G : R -> R) (t0 t1 : R) : Prop :=
  (forall t, finite t = true -> t0 <= real t <= t1 ->
     iv_valid (f_value g t) /\ iv_holds (f_value g t) (G (real t))) /\
  (forall a b, finite a = true -> finite b = true ->
     t0 <= real a -> real a <= real b -> real b <= t1 ->
     iv_valid (f_slope g a b) /\
     forall x y, real a <= x -> x < y -> y <= real b ->
       exists s, iv_holds (f_slope g a b) s /\ G y - G x = s * (y - x)).

(** [enum Stretch]. *)
Inductive stretch : Type :=
  | S_off
  | S_monotone (s : iv)
  | S_unsure.

Definition half : f64 := of_Z 1 (-1).

(** [Slope::read], transcribed. *)
Definition slope_read (g : function1) (a b : f64) : stretch * f64 :=
  let m := fmul half (fadd a b) in
  let s := f_slope g a b in
  let reach := iv_mul s (iv_sub (iv_new a b) (iv_point m)) in
  if negb (iv_has_zero s) then (S_monotone s, iv_mag reach)
  else
    let around := iv_add (f_value g m) reach in
    (if negb (iv_has_zero around) then S_off else S_unsure, iv_mag reach).

(** A stretch read off holds no root; one read monotone runs strictly one way. *)
Theorem slope_read_sound : forall g G t0 t1 a b,
  encloses_function g G t0 t1 ->
  finite a = true -> finite b = true -> t0 <= real a -> real a <= real b -> real b <= t1 ->
  match fst (slope_read g a b) with
  | S_off => forall x, real a <= x <= real b -> G x <> 0
  | S_monotone _ =>
      (forall x y, real a <= x -> x < y -> y <= real b -> G x < G y) \/
      (forall x y, real a <= x -> x < y -> y <= real b -> G y < G x)
  | S_unsure => True
  end.
Admitted.

(** [struct Root], and what a search answers: its roots, or that it gave up. *)
Record root1 : Type := { r_at : f64; r_touch : bool }.

Inductive outcome (A : Type) : Type :=
  | Found (l : list A)
  | GaveUp.
Arguments Found {A} l.
Arguments GaveUp {A}.

(** [isolate::roots]: the function, the stretch's ends, the width, [near]. *)
Definition roots_fn : Type := function1 -> f64 -> f64 -> f64 -> (f64 -> bool) -> outcome root1.

(** **Not what the Rust does today.**  A search that answers no root has proved there is none.
    [roots_at] drops a stretch too narrow to halve that its test left unsure and [near] does not
    accept, and answers [Ok] (a crossing whose rate is enclosed by [Iv::ANY] is lost: [gaps/]). *)
Definition roots_empty_means_none (roots : roots_fn) : Prop :=
  forall g G t0 t1 width near,
    finite t0 = true -> finite t1 = true ->
    encloses_function g G (real t0) (real t1) ->
    roots g t0 t1 width near = Found nil ->
    forall x, real t0 <= x <= real t1 -> G x <> 0.

(** ** Several unknowns *)

Section Several.

(** The number of unknowns, and of equations. *)
Variable N : nat.

Definition point : Type := nat -> f64.
Definition box : Type := nat -> f64 * f64.
Definition rvec : Type := nat -> R.
Definition rbox : Type := nat -> R * R.

Definition at_real (p : point) : rvec := fun k => real (p k).
Definition box_real (b : box) : rbox := fun k => (real (fst (b k)), real (snd (b k))).

(** A box of finite floats, each side the right way round. *)
Definition box_ok (b : box) : Prop :=
  forall k, (k < N)%nat ->
    finite (fst (b k)) = true /\ finite (snd (b k)) = true /\ real (fst (b k)) <= real (snd (b k)).

Definition point_ok (p : point) : Prop := forall k, (k < N)%nat -> finite (p k) = true.

(** [x] is in [u]; [u] is inside [v]. *)
Definition within (u : rbox) (x : rvec) : Prop :=
  forall k, (k < N)%nat -> fst (u k) <= x k <= snd (u k).
Definition inside (u v : rbox) : Prop :=
  forall k, (k < N)%nat -> fst (v k) <= fst (u k) /\ snd (u k) <= snd (v k).

Definition is_root (F : rvec -> rvec) (x : rvec) : Prop :=
  forall i, (i < N)%nat -> F x i = 0.

Definition same (x y : rvec) : Prop := forall k, (k < N)%nat -> x k = y k.

(** [u] holds exactly one root of [F]. *)
Definition one_root (F : rvec -> rvec) (u : rbox) : Prop :=
  (exists x, within u x /\ is_root F x) /\
  forall x y, within u x -> within u y -> is_root F x -> is_root F y -> same x y.

Fixpoint sum_upto (n : nat) (f : nat -> R) : R :=
  match n with O => 0 | S k => sum_upto k f + f k end.

(** [trait System]: [at], [value], [over] (values and Jacobian over a box), [frame], [run]. *)
Record system : Type := {
  sys_at : point -> (nat -> f64) * (nat -> nat -> f64);
  sys_value : point -> nat -> iv;
  sys_over : box -> (nat -> iv) * (nat -> nat -> iv);
  sys_frame : point -> option (nat -> nat -> f64);
  sys_run : point -> option (f64 * f64)
}.

(** What a [System] owes, over the domain [dom]: [value] holds [F] at each point; [over] a box
    holds [F] across it, and between any two points of it a matrix of slopes its Jacobian's
    enclosure holds.  Nothing is owed of [at], [frame] or [run]. *)
Definition encloses_system (s : system) (F : rvec -> rvec) (dom : box) : Prop :=
  (forall p, point_ok p -> within (box_real dom) (at_real p) ->
     forall i, (i < N)%nat ->
       iv_valid (sys_value s p i) /\ iv_holds (sys_value s p i) (F (at_real p) i)) /\
  (forall b, box_ok b -> inside (box_real b) (box_real dom) ->
     (forall i k, (i < N)%nat -> (k < N)%nat ->
        iv_valid (fst (sys_over s b) i) /\ iv_valid (snd (sys_over s b) i k)) /\
     (forall x, within (box_real b) x ->
        forall i, (i < N)%nat -> iv_holds (fst (sys_over s b) i) (F x i)) /\
     (forall x y, within (box_real b) x -> within (box_real b) y ->
        exists A : nat -> nat -> R,
          (forall i k, (i < N)%nat -> (k < N)%nat -> iv_holds (snd (sys_over s b) i k) (A i k)) /\
          forall i, (i < N)%nat ->
            F y i - F x i = sum_upto N (fun k => A i k * (y k - x k)))).

(** [enum Verdict]. *)
Inductive verdict : Type :=
  | V_off
  | V_one (inner unique : box)
  | V_unsure.

(** [Krawczyk::read]: a box read off holds no root; one read as one holds exactly one, and
    that one inside the box the verdict names. *)
Definition krawczyk_read_proves (read : system -> box -> verdict) : Prop :=
  forall s F dom b,
    encloses_system s F dom -> box_ok b -> inside (box_real b) (box_real dom) ->
    match read s b with
    | V_off => forall x, within (box_real b) x -> ~ is_root F x
    | V_one inner unique =>
        box_ok inner /\ inside (box_real inner) (box_real b) /\
        unique = b /\ one_root F (box_real b) /\
        forall x, within (box_real b) x -> is_root F x -> within (box_real inner) x
    | V_unsure => True
    end.

(** [struct Point]. *)
Record found : Type := { p_at : point; p_touch : bool }.

(** [isolate::solve]: the system, the domain, the width, [near]. *)
Definition solve_fn : Type := system -> box -> f64 -> (point -> bool) -> outcome found.

(** A search that answers no root has proved there is none. *)
Definition solve_empty_means_none (solve : solve_fn) : Prop :=
  forall s F dom width near,
    encloses_system s F dom -> box_ok dom ->
    solve s dom width near = Found nil ->
    forall x, within (box_real dom) x -> ~ is_root F x.

(** Every point returned as a crossing is accepted by [near] and lies in a box, inside the
    domain, that holds exactly one root.  How near the root it lies is not claimed: it is polished
    in floats. *)
Definition solve_crossings_are_roots (solve : solve_fn) : Prop :=
  forall s F dom width near ps,
    encloses_system s F dom -> box_ok dom ->
    solve s dom width near = Found ps ->
    forall p, In p ps -> p_touch p = false ->
      near (p_at p) = true /\
      exists u, inside u (box_real dom) /\ within u (at_real (p_at p)) /\ one_root F u.

(** How a root is accounted for by what a search returns: a crossing certified in a box the
    root shares; a touch within the search's resolution of it; or a run the system proved
    through a point, whose stretch of the first unknown holds the root's. *)
Definition certified (F : rvec -> rvec) (dom : box) (p : found) (x : rvec) : Prop :=
  p_touch p = false /\
  exists u, inside u (box_real dom) /\ within u x /\ within u (at_real (p_at p)) /\ one_root F u.

Definition resolved (width : f64) (p : found) (x : rvec) : Prop :=
  p_touch p = true /\
  forall k, (k < N)%nat -> Rabs (real (p_at p k) - x k) <= 3 * real width.

Definition on_a_run (s : system) (x : rvec) : Prop :=
  exists m a b, sys_run s m = Some (a, b) /\ real a <= x 0%nat <= real b.

(** **Not what the Rust does today.**  No root is missed: each is certified, resolved, or on a
    run.  [solve_by] drops the box it is reading when a run is proved through its middle, though
    the box may reach past the run, and a crossing in the rest of it is lost ([gaps/]).  Two
    more things stand between the code and this statement, neither shown to lose a root: a box
    200 halvings deep is read as narrow whatever its width, and two crossings are merged into
    one where the floats they were polished to stand within two widths. *)
Definition solve_misses_none (solve : solve_fn) : Prop :=
  forall s F dom width near ps,
    encloses_system s F dom -> box_ok dom -> finite width = true ->
    solve s dom width near = Found ps ->
    forall x, within (box_real dom) x -> is_root F x ->
      (exists p, In p ps /\ (certified F dom p x \/ resolved width p x)) \/ on_a_run s x.

End Several.
