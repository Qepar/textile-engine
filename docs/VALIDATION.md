# Correctness campaign — 2026-10-13

Base commit: `40ba2757641dd1d4536315859d1671c9ae736013`.

Before correction, bounded probes reproduced broken imports, unsupported array attributes in anisotropic/wet constraints, an extra dt factor in gravity, erased free-motion velocity, collision moving fixed points, energy-injecting friction, non-rest initial bending, skipped ground in the grid path, unweighted distance projections and nonfinite default simulation.

Local validation: 20 physics tests passed on Python 3.12/NumPy 2.3.5. The bounded default-cloth test covers a 4×4 grid for 120 steps; it does not prove stability for arbitrary parameters or scenes. Renderer code paths were repaired statically; no interactive visual test is claimed. The new CI matrix is configured for Linux/Windows and Python 3.10/3.12; CI results must be checked after publication.

An independently scoped distance-XPBD reference previously passed 9 analytic tests and a 36-vertex Blender sheet bridge. That separate artifact does not imply this PBD baseline implements XPBD or full textile material physics.

The optional C++ kernel compiled with a Linux C++17 compiler. Position differences from the sequential NumPy distance oracle were at most about 1.1e-16 on the three tested grids (100, 400, 2500 vertices). Invalid input is rejected, with rollback on arithmetic overflow. Timings use three warmed batches >=0.15 seconds and report median per sweep. The recorded kernel experiment is not a speed guarantee on Windows/AMD, a full-frame benchmark or a superiority comparison with external engines. No native Windows build or HIP kernel has run in this campaign.

Main remaining limitations: PBD stiffness depends on timestep/iterations; no calibrated dihedral bending, triangle CCD, body contacts, cloth self-collision, seams or production garment test. The previously advertised particle-grid collision path is explicitly disabled. The next priority is a contact contract and independent collision cases, then a calibrated compliance/material formulation.
