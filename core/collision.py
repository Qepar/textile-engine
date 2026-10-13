"""Discrete particle-centre ground contacts. No triangle/body CCD guarantee."""
import numpy as np

def detect_collisions(cloth,ground_height=-1.):
    for p in cloth.particles:
        p.collision_normal=None;p.collision_depth=0.
        if not p.fixed and p.position[2]<ground_height:
            p.collision_normal=np.array([0.,0.,1.]);p.collision_depth=ground_height-p.position[2]

def resolve_collisions(cloth,ground_height=-1.,friction=.2):
    for p in cloth.particles:
        if p.fixed or p.collision_normal is None:continue
        p.position+=p.collision_normal*p.collision_depth
        vn=float(np.dot(p.velocity,p.collision_normal))
        tangent=p.velocity-vn*p.collision_normal
        p.velocity=max(vn,0.)*p.collision_normal+max(0.,1.-friction)*tangent
        p.collision_normal=None;p.collision_depth=0.
