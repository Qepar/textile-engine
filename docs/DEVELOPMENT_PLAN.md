# Measured research and engineering program

Goal: compete on numerical correctness, cloth realism, robust contact and throughput. No overall superiority claim is supported by the current tests. Language selection follows measured bottlenecks, not a Python-only constraint.

## Priority and gates

1. **Correctness baseline**: analytic motion/force tests, mass-aware projection, pin conservation, dissipative contact/friction, finite-state rejection. Preserve a readable NumPy oracle.
2. **Material formulation**: compare PBD, XPBD and AVBD on identical rest geometry, mass, material and timestep budgets. Add tensile, shear and dihedral-bending calibration; report error versus a trusted numerical/material reference, not just visual appeal. Wetness needs explicit mass and material laws.
3. **Contact**: triangle surfaces, thickness, broad phase and narrow phase, adjacency handling, friction and continuous motion. Evaluate IPC Toolkit and Offset Geometric Contact on same collision cases. A particle grid or zero BVH candidates is not a penetration-free guarantee.
4. **Garments**: patterns, seams, grain directions and multilayer contacts; retain exact source indices and unit/space contracts through Blender evaluation.
5. **Native CPU**: contiguous data, optional C++ kernels, deterministic graph scheduling and profiling. Qualify every backend against the oracle before integration.
6. **AMD GPU**: prototype HIP only after verifying the actual Windows SDK/compiler/device stack and numerical parity. RX 7900 XTX appears in AMD's supported Windows HIP SDK table, but this does not validate our kernels or all ROCm libraries. Keep CPU fallback. Compare transfer, synchronization, memory and total frame cost.

## Comparative benchmark specification

| Dimension | Measurement | Acceptance basis |
|---|---|---|
| Dynamics | free motion, free fall, force/mass response | analytic error and refinement |
| Material | tension, shear, bending | calibrated reference curves and units |
| Stability | timestep/iteration sweep, duration, finite state | explicit error/strain bounds |
| Contact | minimum separation, crossings, contact residual | controlled grazing, coplanar and fast-motion cases |
| Fidelity | folds, seams, rest shape, multilayers | matched inputs and independent reference |
| Performance | median/tail step time, peak memory | same geometry, tolerances and machine |
| Backend parity | position/velocity/contact difference | bounded float64/float32 error |
| Pipeline | source indices, pins, modifiers, spaces | exact identity and reproducible artifacts |

Populate baseline comparisons before setting performance targets. Start at small controlled scenes; scale after gates pass. GPU occupancy alone is not progress.

## Primary sources consulted 2026-10-13

- Macklin, Müller, Chentanez, XPBD (2016): https://matthias-research.github.io/pages/publications/XPBD.pdf
- Utah Graphics Lab, AVBD (SIGGRAPH 2025): https://graphics.cs.utah.edu/research/projects/avbd/
- Utah Graphics Lab, Offset Geometric Contact (SIGGRAPH 2025): https://graphics.cs.utah.edu/research/projects/ogc/
- IPC Toolkit: https://github.com/ipc-sim/ipc-toolkit
- AMD Windows HIP requirements: https://rocm.docs.amd.com/projects/install-on-windows/en/develop/reference/system-requirements.html

These are research candidates, not components already integrated. No CUDA dependency is introduced by this correctness campaign. Existing character canon and global native conventions are outside this repository's test scope.
