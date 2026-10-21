# Mechanisms over a parameter

Linkages driven by a number that runs. Each document declares the crank's angle as a parameter,
`param t: Angle over (0deg, 360deg)`, and one relation reads it (`ground angle(t) crank`, or a joint's law,
`fix(value == …) follower`). Every
relation of the drawing then holds for every value of `t`, so the drawing is a function of `t`:
it is drawn at the interval's start and **followed** from there by continuation, carrying the one
assembly mode its seeds pick, to the end or to a **singular instant**, where the march stops and
says why. `trace(m over t)` is a point of the drawing as `t` runs. The diagnosis counts the
freedom each parameter drives (`1 DOF over t`: the crank), and `solventc` says how far the drawing
was followed. Issue #391, stage 4; [docs/contract/parameters.md](../../../docs/contract/parameters.md)
holds the rules.

Read them in this order:

| File | What it shows |
|---|---|
| `four_bar.sv` | **A whole turn.** A crank-rocker four-bar (Grashof's condition holds, so the crank turns round) and its coupler curve, the coupler's midpoint traced over `t` and closed on itself. |
| `slider_crank.sv` | **Through the dead centres.** A piston's mechanism: the slider stands still twice a turn, the drawing does not, and the march runs straight through. The slider's path is the stroke, `2r` long; the rod's midpoint draws an oval. |
| `triple_rocker.sv` | **Where the motion turns back.** The same four rods in another proportion: no rod turns round, and the march stops at 129.838°, where the coupler and the rocker stand in line, saying no drawing stands past it. |
| `change_point.sv` | **Where two branches cross.** A four-bar on the boundary of Grashof's condition folds flat at 180°, where its two assembly modes meet; the march stops there and says so. A rocker a ten-thousandth longer is carried through the near-fold round the turn. |
| `cam_follower.sv` | **A cam and its follower.** The cam is cut from the follower's law, and the roller is drawn in the follower's frame (`riding := c in follower`) with a pin on the cam (`pin := p in cam`). Followed round the turn, the roller turned back by the cam is a roller's radius from the profile at every instant. |

In the app each opens with a timeline under the drawing: scrub `t` and the drawing stands at that
instant, the singular instant marked where the march stopped; drag the crank's pin and the linkage
turns with it. A drawing shows one at an instant with `model m from "four_bar.sv" at t = 90deg`.
The instant is presentation, never written into the model.

What is not here yet: these traces are drawn but nothing is held to them (a relation naming a
followed trace is E061), so the series in `../generation/` that solves *through* a traced path —
the Jansen stride standing on the ground, the dwell circle fitted by curvature — still traces a
component's formal (`trace(leg.toe over theta in (0, 360))`).

A cam driven by a law of `t` is followed and swept in space (`tests/drives.rs`), and a profile in
the plane is generated under one: `cam_follower.sv` here, and `../generation/heart_cam.sv` and the
cams after it, each cut from a single law.
