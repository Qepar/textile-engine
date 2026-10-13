# Qepar Textile Engine

Experimental textile dynamics with a CPU correctness baseline. This project does not yet qualify realistic garments, cloth/body contact or continuous self-collision.

## Run and test

Python 3.10+ and NumPy:

```sh
python -m pip install -e .
python -m unittest discover -s tests -v
```

The interactive renderer is optional: install `.[render]` and an operating-system GLUT runtime, then run `python main.py`. Interactive rendering has not been validated in the headless physics tests.

## Contracts and supported scope

- `stiffness` is a dimensionless PBD projection fraction in **[0, 1]**. Previous defaults of 10/20 were invalid projection factors and have been replaced with 0.8.
- Gravity is acceleration; external forces are forces and use each particle's inverse mass.
- Fixed particles retain their position. Ground contact is discrete and acts on particle centres, without cloth thickness or continuous collision detection.
- Distance projections use inverse-mass weighting. Three-point angle bending stores the initial rest angle; it is not a calibrated shell/dihedral textile model.
- Directional and wet distance constraints are experimental approximations, with explicit array inputs. Combined anisotropic/wet construction is rejected pending a material contract.
- `use_spatial_grid=True` is rejected explicitly: the former path omitted ground contact and did not implement qualified cloth self-collision. `SpatialGrid` remains available as a tested building block, including negative coordinates.
- Internal particle friction is disabled by default. Its optional dissipative reference implementation is quadratic and unsuitable for large production meshes.
- No garment pattern, seams, calibrated material parameters, triangle body collision, CCD, or full self-collision guarantee is provided.

## Native experiment

An optional C++17 distance-only projection kernel preserves the sequential reference algorithm. It is **not** automatically selected by `PBDSolver`.

Linux example:

```sh
c++ -O3 -std=c++17 -shared -fPIC native/distance_sweep.cpp -o distance_sweep.so
python benchmarks/native_distance.py ./distance_sweep.so native-benchmark.json
```

Windows build candidate (not yet validated): use a Visual Studio developer shell and `cl /O2 /std:c++17 /LD native\distance_sweep.cpp /Fe:distance_sweep.dll`, then supply the DLL path to the benchmark. Native builds, dimensions, finite state and parity must be qualified on the target machine before adoption.

The benchmark checks positional parity after five sweeps, fixed-point preservation, invalid topology rejection and overflow rollback. Three warmed timing batches of at least 0.15 s compare distance projections only. Timings exclude construction and compilation, use a converging state, and do not represent the full simulator or GPU performance.

See [development plan](docs/DEVELOPMENT_PLAN.md) and [validation](docs/VALIDATION.md). MIT license retained from the initial repository.
