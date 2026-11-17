(** * The model of Rust's [f64] every statement here is written over.

    IEEE-754 binary64 as Flocq formalises it ([BinarySingleNaN]: one NaN, no payloads, which is
    all the kernel's arithmetic can tell apart), each operation rounded to nearest, ties to even,
    which is the only rounding Rust's [f64] has.  [real] reads a float as the real number it is;
    Flocq reads an infinity and a NaN as nought, so a statement about [real x] says nothing until
    [finite x] is known, and every statement below says which floats it takes to be finite.

    What is trusted here, and not proved: that [rustc] and the hardware compute these operations
    as IEEE-754 gives them, on the native targets and on wasm32 alike (no fused multiply-add
    contracted in, no extended precision), and that the definitions below read the Rust they
    name correctly.  They are transcribed by hand; a translator's output is to replace them. *)

From Coq Require Import ZArith Reals Bool.
From Flocq Require Import Core BinarySingleNaN.

Definition prec : Z := 53.
Definition emax : Z := 1024.

Global Instance prec_gt_0 : Prec_gt_0 prec.
Proof. reflexivity. Qed.

Global Instance prec_lt_emax : Prec_lt_emax prec emax.
Proof. reflexivity. Qed.

Definition f64 : Type := binary_float prec emax.

(** The real number a finite float is. *)
Definition real (x : f64) : R := B2R x.

(** [f64::is_finite]. *)
Definition finite (x : f64) : bool := is_finite x.

(** The rounding the arithmetic below performs, as a function of the reals. *)
Definition rnd (r : R) : R := round radix2 (SpecFloat.fexp prec emax) ZnearestE r.

Definition zero : f64 := B754_zero false.
Definition infinity : f64 := B754_infinity false.
Definition neg_infinity : f64 := B754_infinity true.

(** The float an integer times a power of two rounds to: how a literal is written here
    ([of_Z 1 (-1)] is [0.5]). *)
Definition of_Z (m e : Z) : f64 := binary_normalize prec emax _ _ mode_NE m e false.

(** [a + b], [a - b], [a * b], [a / b], [a.sqrt()], [-a], [a.abs()]. *)
Definition fadd : f64 -> f64 -> f64 := Bplus mode_NE.
Definition fsub : f64 -> f64 -> f64 := Bminus mode_NE.
Definition fmul : f64 -> f64 -> f64 := Bmult mode_NE.
Definition fdiv : f64 -> f64 -> f64 := Bdiv mode_NE.
Definition fsqrt : f64 -> f64 := Bsqrt mode_NE.
Definition fneg : f64 -> f64 := Bopp.
Definition fabs : f64 -> f64 := Babs.

(** [f64::next_up] and [f64::next_down]: the neighbouring float, an infinity past the largest. *)
Definition next_up : f64 -> f64 := Bsucc.
Definition next_down : f64 -> f64 := Bpred.

(** [a < b] and [a <= b]: false where either is a NaN, as Rust's are. *)
Definition flt : f64 -> f64 -> bool := Bltb.
Definition fle : f64 -> f64 -> bool := Bleb.

(** [f64::min] and [f64::max]: the other operand where one is a NaN.  Which zero comes back of
    [-0.0] and [0.0] is left open by Rust and immaterial to [real]. *)
Definition fmin (a b : f64) : f64 :=
  if is_nan a then b else if is_nan b then a else if flt b a then b else a.
Definition fmax (a b : f64) : f64 :=
  if is_nan a then b else if is_nan b then a else if flt a b then b else a.
