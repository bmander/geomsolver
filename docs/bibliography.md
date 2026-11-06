# Bibliography

The methods the core implements, the closed forms its tests hold it to, and the prior art for
what it works out for itself.  Each entry names where it is used.  Surveys of work read and
compared but not implemented stay with the design notes that made them (the last section).

## Constraint solving and diagnosis

- Owen, *Algebraic solution for geometry from dimensional constraints*, SMA 1991.
- Bouma, Fudos, Hoffmann, Cai, Paige, *Geometric constraint solver*, CAD 27(6), 1995.
- Fudos & Hoffmann, *A graph-constructive approach to solving systems of geometric constraints*,
  ACM TOG 16(2), 1997.  `decompose.rs`'s merge rule.
- Hoffmann, Lomonosov, Sitharam, *Decomposition plans for geometric constraint systems*, I & II,
  J. Symbolic Computation 31, 2001.
- Durand & Hoffmann, *A systematic framework for solving geometric constraints analytically*,
  J. Symbolic Computation 30, 2000.
- Michelucci & Foufou, *Geometric constraint solving: the witness configuration method*, CAD
  38(4), 2006.  `witness.rs`; `diagnose::witnessed`, a `claim { }` block's rank test.
- Sitharam, Arbree, Zhou, Kohareswaran, *Solution space navigation for geometric constraint
  systems*, ACM TOG 25(2), 2006.
- Zou et al., *A review on geometric constraint solving*, arXiv:2202.13795, 2022.
- Hopcroft & Karp, *An n^5/2 algorithm for maximum matchings in bipartite graphs*, SIAM J.
  Computing 2(4), 1973.  `graph.rs`'s matching.
- Dulmage & Mendelsohn, *Coverings of bipartite graphs*, Canadian J. Mathematics 10, 1958.
  `graph::dulmage_mendelsohn`: over-, well- and under-constrained parts.
- Pothen & Fan, *Computing the block triangular form of a sparse matrix*, ACM TOMS 16(4), 1990.
  `System::block_order`, the block-triangular solve.
- Tarjan, *Depth-first search and linear graph algorithms*, SIAM J. Computing 1(2), 1972.  The
  strongly connected blocks of the block order (`graph::blocks`).
- Laman, *On graphs and rigidity of plane skeletal structures*, J. Engineering Mathematics 4,
  1970.  The count the pebble game decides; property tests.
- Henneberg, *Die graphische Statik der starren Systeme*, Teubner, 1911.  Frameworks built by his
  moves, whose answer is known by construction (`fixtures.rs`, docs/testing.md).
- Jacobs & Hendrickson, *An algorithm for two-dimensional rigidity percolation: the pebble game*,
  J. Comput. Phys. 137, 1997.  `graph::pebble_game`.

## Numerical linear algebra and optimisation

- Powell, *A hybrid method for nonlinear equations*, in Rabinowitz (ed.), *Numerical Methods for
  Nonlinear Algebraic Equations*, Gordon and Breach, 1970.  `newton::dogleg`.
- Nocedal & Wright, *Numerical Optimization*, 2nd ed., Springer, 2006 (ch. 4 and 10).  The trust
  region and Levenberg–Marquardt (`newton.rs`).
- Golub & Reinsch, *Singular value decomposition and least squares solutions*, Numerische
  Mathematik 14, 1970.  `linalg.rs`'s SVD.
- Golub & Van Loan, *Matrix Computations*, 4th ed., Johns Hopkins, 2013.  Householder QR with
  column pivoting and the complete orthogonal decomposition (`linalg.rs`).
- Davis, *Direct Methods for Sparse Linear Systems*, SIAM, 2006.  `sparse.rs`'s up-looking
  LDLᵀ; `cs_amd`.
- Amestoy, Davis & Duff, *An approximate minimum degree ordering algorithm*, SIAM J. Matrix
  Analysis and Applications 17(4), 1996.  `amd.rs`.
- Liu, *The role of elimination trees in sparse factorization*, SIAM J. Matrix Analysis and
  Applications 11(1), 1990.  `cholesky.rs`'s elimination tree.
- Cuthill & McKee, *Reducing the bandwidth of sparse symmetric matrices*, Proc. 24th ACM National
  Conference, 1969.  Reverse Cuthill–McKee in `sparse.rs`.
- Brent, *Algorithms for Minimization without Derivatives*, Prentice-Hall, 1973.  `roots.rs`'s
  minimum; the sweeps' search for a ring or ruling in contact all round.

## Intervals, certified roots and exact predicates

- Moore, *Interval Analysis*, Prentice-Hall, 1966.  The interval arithmetic under the certified
  readings (`util::interval`, docs/interval-geometry.md).
- Brönnimann, Melquiond & Pion, *The design of the Boost interval arithmetic library*,
  Theoretical Computer Science 351(1), 2006.  Outward rounding of the elementary functions.
- Krawczyk, *Newton-Algorithmen zur Bestimmung von Nullstellen mit Fehlerschranken*, Computing 4,
  1969.  `isolate.rs`'s preconditioned exclusion.
- Kantorovich & Akilov, *Functional Analysis*, 2nd ed., Pergamon, 1982.  Kantorovich's theorem,
  by which `isolate.rs` certifies each root (docs/contract/brep.md).
- Shewchuk, *Adaptive precision floating-point arithmetic and fast robust geometric predicates*,
  Discrete & Computational Geometry 18(3), 1997.  `delaunay::predicates`, the exact expansions
  (docs/field-meshing.md).

## Derivatives, rates and extremals

- Griewank & Walther, *Evaluating Derivatives*, 2nd ed., SIAM, 2008.
- Griewank, Utke & Walther, *Evaluating higher derivative tensors by forward propagation of
  univariate Taylor series*, Mathematics of Computation 69(231), 2000.  Mixed partials from
  univariate jets (`rates.rs`).
- Wagner, Walther & Schaefer, *On the efficient computation of high-order derivatives for
  implicitly defined functions*, Computer Physics Communications 181(4), 2010.  `taylor.rs`;
  `locus::higher_orders` (docs/motion-generation.md).
- Fike & Alonso, *The development of hyper-dual numbers for exact second-derivative
  calculations*, AIAA 2011-886, 2011.
- Haug, *Computer Aided Kinematics and Dynamics of Mechanical Systems*, Allyn and Bacon, 1989; and
  Nikravesh, *Computer-Aided Analysis of Mechanical Systems*, Prentice Hall, 1988.  Acceleration
  analysis as a constrained system's second derivative.
- Pontryagin, Boltyanskii, Gamkrelidze & Mishchenko, *The Mathematical Theory of Optimal
  Processes*, Interscience, 1962.  `extremal.rs`: `H_θ = 0`, `H_θθ > 0`, the costate's jumps.

## Curves, surfaces and rigid motion

- Piegl & Tiller, *The NURBS Book*, 2nd ed., Springer, 1997.  `nurbs.rs` (A2.2, A2.3).
- Cox, *The numerical evaluation of B-splines*, IMA J. Applied Mathematics 10, 1972; and de Boor,
  *On calculating with B-splines*, J. Approximation Theory 6, 1972.  The recurrence in `nurbs.rs`.
- Wang, Jüttler, Zheng & Liu, *Computation of rotation minimizing frames*, ACM TOG 27(1), 2008.
  Double reflection: `trajectory.rs`, `motion/guide.rs`, `std.stations`.
- Mozzi, *Discorso matematico sopra il rotamento momentaneo dei corpi*, Naples, 1763; and
  Chasles, *Note sur les propriétés générales du système de deux corps semblables entr'eux*,
  Bulletin de Férussac 14, 1830.  Every instant of a rigid motion is a screw (`trajectory::Twist`).
- Eberly, *Distance from a point to an ellipse, an ellipsoid, or a hyperellipsoid*, Geometric
  Tools.  `affine.rs`'s `ellipsoid_foot` and `ellipse_foot`.
- Eberly, *Triangulation by ear clipping*, Geometric Tools.  Holes bridged into a cut's face
  (docs/contract/solids.md).
- Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005.  `space.rs`'s segment and
  triangle distances.
- Möller & Trumbore, *Fast, minimum storage ray–triangle intersection*, J. Graphics Tools 2(1),
  1997.  The workspace's picking.
- Barnhill & Kersey, *A marching method for parametric surface/surface intersection*, CAGD 7,
  1990; and Patrikalakis & Maekawa, *Shape Interrogation for Computer Aided Design and
  Manufacturing*, Springer, 2002.  The traced meeting (`meeting.rs`).

## Solid modelling

- Requicha & Voelcker, *Boolean operations in solid modeling: boundary evaluation and merging
  algorithms*, Proc. IEEE 73(1), 1985.  Classification against `Csg` (`csg.rs`, `solid.rs`).
- Weiler, *Topological structures for geometric modeling*, PhD thesis, Rensselaer Polytechnic
  Institute, 1986; and Rossignac & O'Connor, *SGC: a dimension-independent model for pointsets
  with internal structures and incomplete boundaries*, in Wozny, Turner & Preiss (eds.),
  *Geometric Modeling for Product Engineering*, North-Holland, 1989.  `complex.rs`, the one
  arrangement.
- Jackson, *Boundary representation modelling with local tolerances*, Proc. 3rd ACM Symposium on
  Solid Modeling and Applications, 1995.  `tolerant.rs`.
- Steiner, *Über parallele Flächen*, Monatsbericht der Akademie der Wissenschaften zu Berlin,
  1840.  The volume of a body grown, its edges' and corners' terms (docs/contract/sets.md).

## Swept solids

- Abdel-Malek & Yeh, *Geometric representation of the swept volume using Jacobian
  rank-deficiency conditions*, CAD 29(6), 1997.
- Blackmore, Leu & Wang, *The sweep-envelope differential equation algorithm and its application
  to NC machining verification*, CAD 29(9), 1997.  What a sweep's boundary is made of, and
  membership deciding which candidate bounds it (docs/swept-solid-design.md §1, §4).
- Adsul, Machchhar & Sohoni, *A computational framework for boundary representation of solid
  sweeps*, arXiv:1404.0119, 2014; and *Incorporating sharp features in the general solid sweep
  framework*, Computer Graphics Forum 35(5), 2016 (arXiv:1405.7457).  The contact set read face
  by face; a sharp edge's fan (docs/spiral-bevel-roadmap.md).
- Machchhar, Plakhotnik & Elber, *Precise algebraic-based swept volumes for arbitrary free-form
  shaped tools towards multi-axis CNC machining verification*, CAD 90, 2017.
- Sellán, Aigerman & Jacobson, *Swept volumes via spacetime numerical continuation*, ACM TOG
  40(4), 2021.  The reference the swept field and its gates were set against.
- Jacobson, Kavan & Sorkine-Hornung, *Robust inside-outside segmentation using generalized winding
  numbers*, ACM TOG 32(4), 2013.  `swept::member`'s tool, a mesh that only nearly closes.
- Weyl, *On the volume of tubes*, American J. Mathematics 61(2), 1939.  A ball screwed, held to
  its tube's volume (`swept_closed.rs`, `swept_loci.rs`).

## Gears

- Litvin & Fuentes, *Gear Geometry and Applied Theory*, 2nd ed., Cambridge University Press,
  2004.  The equation of meshing; singularities of a generated surface (the admission's E3).
- Chang, Huston & Coy, *Computer-aided design of bevel gear tooth surfaces*, NASA TM-101449,
  1989.  Crown generation (`tests/envelope/crown.rs`, docs/spiral-bevel-design.md).
- Bibel, Reddy & Kumar, *Manual for automatic generation of finite element models of spiral bevel
  gears in mesh*, NASA CR-191009, 1994.  POINTS.F, the independent numerical reference
  (`tests/envelope/nasa.rs`).
- Teixeira Alves et al., *Static and dynamic models for spiral bevel gears*, Mechanics & Industry
  13(5), 2012.  The conical and toroidal generating surfaces of the root's fillet.
- Phillips, *General Spatial Involute Gearing*, Springer, 2003.  `tests/phillips.rs`,
  `spiral_bevel/phillips.sv`.

## Meshing

- Bowyer, *Computing Dirichlet tessellations*, and Watson, *Computing the n-dimensional Delaunay
  tessellation with application to Voronoi polytopes*, both The Computer Journal 24(2), 1981.
- Lawson, *Software for C¹ surface interpolation*, in Rice (ed.), *Mathematical Software III*,
  Academic Press, 1977.  `mesh.rs`'s flips.
- Ruppert, *A Delaunay refinement algorithm for quality 2-dimensional mesh generation*,
  J. Algorithms 18(3), 1995.  `delaunay/plane.rs`.
- Rivara, *Algorithms for refining triangular grids suitable for adaptive and multigrid
  techniques*, Int. J. Numerical Methods in Engineering 20(4), 1984.  Longest-edge refinement
  (docs/implicit-meshing-methods.md).
- Li & Teng, *Generating well-shaped Delaunay meshes in 3D*, Proc. 12th ACM-SIAM SODA, 2001.  A
  sliver's picking region (`delaunay/refine/cells.rs`).
- Cheng, Dey & Ramos, *Delaunay refinement for piecewise smooth complexes*, Discrete &
  Computational Geometry 43(1), 2010; and Dey & Levine, *Delaunay meshing of piecewise smooth
  complexes without expensive predicates*, Algorithms 2(4), 2009.  `delaunay/refine`,
  protected features (docs/field-meshing.md).
- Johnen, Remacle & Geuzaine, *Geometrical validity of curvilinear finite elements*, J. Comput.
  Phys. 233, 2013.  A TET10's Jacobian bounded by its Bernstein coefficients
  (`delaunay/tetra.rs`).
- Ju, Losasso, Schaefer & Warren, *Dual contouring of Hermite data*, ACM TOG 21(3), 2002.

## Structural analysis

- Ericsson & Ruhe, *The spectral transformation Lanczos method for the numerical solution of large
  sparse generalized symmetric eigenvalue problems*, Mathematics of Computation 35(152), 1980.
  Shift-invert (`eigen.rs`).
- Wu & Simon, *Thick-restart Lanczos method for large symmetric eigenvalue problems*, SIAM J.
  Matrix Analysis and Applications 22(2), 2000.  `eigen.rs`.
- Sylvester, *A demonstration of the theorem that every homogeneous quadratic polynomial is
  reducible by real orthogonal substitutions to the form of a sum of positive and negative
  squares*, Philosophical Magazine 4, 1852.  The inertia count that no pair is missed.
- Grimes, Lewis & Simon, *A shifted block Lanczos algorithm for solving sparse symmetric
  generalized eigenproblems*, SIAM J. Matrix Analysis and Applications 15(1), 1994.  The same
  count as a Sturm-sequence check, in production.
- Williams, *Stress singularities resulting from various boundary conditions in angular corners
  of plates in extension*, J. Applied Mechanics 19, 1952.  `diagnose::analysis::williams`.
- Kirsch, *Die Theorie der Elastizität und die Bedürfnisse der Festigkeitslehre*, Zeitschrift des
  VDI 42, 1898; Lamé, *Leçons sur la théorie mathématique de l'élasticité des corps solides*,
  Bachelier, 1852.  Closed forms of the verification corpus (`tests/analysis_corpus`).
- Timoshenko, *On the correction for shear of the differential equation for transverse
  vibrations of prismatic bars*, Philosophical Magazine 41, 1921; and Timoshenko &
  Woinowsky-Krieger, *Theory of Plates and Shells*, 2nd ed., McGraw-Hill, 1959.  The corpus's
  beam with shear and its clamped plate.
- NAFEMS, *The Standard NAFEMS Benchmarks*, TNSB Rev. 3, 1990.  LE1, LE10, FV32 and the rest of
  the corpus's benchmarks.
- Dhondt, *The Finite Element Method for Three-Dimensional Thermomechanical Applications*, Wiley,
  2004.  CalculiX, the oracle (`make test-ccx`).

## Prior art for what the core works out for itself

The core derives these results in place, often in closed forms of its own.  They are already in
the literature, and are listed so that a claim of originality is checked against them first.

- Pottmann & Wallner, *Computational Line Geometry*, Springer, 2001.  A surface's contact under a
  helical motion, fixed on the tool, its characteristic carried: the constant-twist class
  (`swept::constant_twist`, `contact.rs`).
- Kim, Varadhan, Lin & Manocha, *Fast swept volume approximation of complex polyhedral models*,
  CAD 36(11), 2004.  The candidates' arrangement, kept by membership (`swept::closed`).
- Tilove, *Set membership classification: a unified approach to geometric intersection problems*,
  IEEE Trans. Computers C-29(10), 1980.  Membership as the decision (`swept::member`, `csg.rs`).
- Ceccarelli, *A formulation for the workspace boundary of general N-revolute manipulators*,
  Mechanism and Machine Theory 31(5), 1996; and Pottmann, Shi, Skopenkov & Wallner, *Darboux
  cyclides and webs from circles*, CAGD 29(1), 2012.  A circle turned about any axis is affine in
  `(ρ² + z², z)`, so its surface is a Darboux cyclide; a plane holding one of its circles meets it
  in another (`turned::Rim`, `ssi::plane_holding_rim`).
- Koenderink, *What does the occluding contour tell us about solid shape?*, Perception 13, 1984.
  An apparent contour cusps where the view runs along an asymptotic direction: a torus slid at a
  lean folds at `tan² β = (R − r)/r` (`slid::Slid`).
- Tang, Kim & Manocha, *C2A: controlled conservative advancement for continuous collision
  detection of polygonal models*, ICRA 2009.  Motion bounds composed along a chain of rigid
  motions (`motion.rs`, `Trajectory::inverse_point_speed_bound_over`).
- Grandine & Klein, *A new approach to the surface intersection problem*, CAGD 14(2), 1997.
  Every branch of a meeting seeded at the box's edge or at a critical point (`meeting.rs`).
- Danskin, *The theory of max-min, with applications*, SIAM J. Applied Mathematics 14(4), 1966.
  The gradient of a least value at its minimiser (`envelope/regular.rs`).
- Connelly & Whiteley, *Second-order rigidity and prestress stability for tensegrity
  frameworks*, SIAM J. Discrete Mathematics 9(3), 1996.  A freedom blocked at second order
  (`witness.rs`).
- Aumann, *Interpolation with developable Bézier patches*, CAGD 8(5), 1991; and Chu & Séquin,
  *Developable Bézier patches: properties and design*, CAD 34(7), 2002.  A ruling between two
  rails where `det(a′, b′, B − A) = 0` (`developable.rs`).
- Rossignac & Requicha, *Constant-radius blending in solid modelling*, Computers in Mechanical
  Engineering 3, 1984; and *Offsetting operations in solid modelling*, CAGD 3(2), 1986.  A round
  as a rolling ball's set, a body grown as faces swept, edges' tubes and corners' balls
  (`std.rounds`, `offset.rs`).

## Oracles and formats

- Open CASCADE Technology (OCCT), the native export's kernel and the clean-room kernel's oracle
  (docs/contract/brep.md).
- CalculiX (Dhondt, above), the analysis oracle.
- `nalgebra`, the linear algebra tests' one reference implementation (a dev-dependency).
- ISO 10303-21 and ISO 10303-214 (STEP, AP214): `step.rs`.
- Khronos, *glTF 2.0 Specification*: `gltf.rs`.

## Surveys kept with their notes

- docs/implicit-meshing-methods.md and docs/implicit-library-review.md: implicit meshing (marching
  triangles, dual and manifold dual contouring, Plantinga–Vegter, libfive and Fidget, CGAL's
  Mesh_3, Manifold), read against the swept field.
- docs/motion-generation.md, *Prior art*: curvature synthesis, envelopes and conjugates, dynamic
  geometry (GeoGebra, Geometry Expressions).
