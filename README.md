# Textile Engine

A **custom 3D cloth simulation engine** written in **pure Python**.
It implements **Position-Based Dynamics (PBD)** for stable and realistic fabric deformation, with **no dependencies on external physics engines** like PyBullet or Blender.

## ✨ Features
- **100% Original Implementation**: All physics, constraints, and collision detection are implemented from scratch
- **Position-Based Dynamics**: Stable simulation that doesn't suffer from numerical instability
- **Custom Constraints**:
  - Distance constraints (structural springs)
  - Bend constraints (for realistic folding)
  - Shear constraints (for diagonal stiffness)
  - Anisotropic constraints (direction-dependent stiffness)
  - Wet fabric simulation (reduced stiffness when damp)
- **Collision Detection**:
  - Ground plane collisions
  - Sphere collisions
  - Spatial grid optimization for large meshes
  - Friction modeling
- **Interactive 3D Rendering**: Real-time visualization using PyOpenGL
- **Extensible Architecture**: Easy to add new constraints, forces, or behaviors

## 📦 Installation
```bash
pip install numpy PyOpenGL pygame
