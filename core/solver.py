"""Correctness baseline: sequential mass-aware PBD, CPU NumPy.
Particle proximity/friction is experimental and is not cloth self-collision.
"""
import numpy as np
from .collision import detect_collisions,resolve_collisions

class SpatialGrid:
    def __init__(self,cell_size=.5):
        if not np.isfinite(cell_size) or cell_size<=0:raise ValueError('invalid cell size')
        self.cell_size=float(cell_size);self.grid={}
    def clear(self):self.grid.clear()
    def _get_cell(self,position):return tuple(np.floor(np.asarray(position)/self.cell_size).astype(int))
    def add_particle(self,particle,index):self.grid.setdefault(self._get_cell(particle.position),[]).append(index)
    def get_neighbors(self,particle,index):
        cell=self._get_cell(particle.position);result=[]
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                for dz in (-1,0,1):
                    result.extend(i for i in self.grid.get((cell[0]+dx,cell[1]+dy,cell[2]+dz),[]) if i!=index)
        return result

def apply_internal_friction(cloth,friction_coeff=.1,radius=.15):
    if friction_coeff<0 or radius<=0:raise ValueError('invalid friction parameters')
    for i,p1 in enumerate(cloth.particles):
        for p2 in cloth.particles[i+1:]:
            if np.linalg.norm(p1.position-p2.position)<radius:
                force=friction_coeff*(p2.velocity-p1.velocity)
                p1.apply_force(force);p2.apply_force(-force)

class PBDSolver:
    def __init__(self,cloth,ground_height=-1.,friction=.2,iterations=5,use_spatial_grid=False,internal_friction=0.):
        if not isinstance(iterations,int) or iterations<1 or not 0<=friction<=1 or not np.isfinite(ground_height):raise ValueError('invalid solver parameters')
        if use_spatial_grid:raise NotImplementedError('particle-grid self-collision is not qualified; ground contacts do not require a grid')
        if not np.isfinite(internal_friction) or internal_friction<0:raise ValueError('invalid internal friction')
        self.cloth=cloth;self.ground_height=ground_height;self.friction=friction;self.iterations=iterations;self.internal_friction=internal_friction
    def step(self,dt,gravity=(0,0,-9.81)):
        g=np.asarray(gravity,dtype=float)
        if not np.isfinite(dt) or dt<=0 or g.shape!=(3,) or not np.isfinite(g).all():raise ValueError('invalid step')
        ps=self.cloth.particles
        if not ps:raise ValueError('empty cloth')
        if any(not np.isfinite(p.position).all() or not np.isfinite(p.velocity).all() or not np.isfinite(p.force).all() for p in ps):raise ValueError('nonfinite state')
        if self.internal_friction:apply_internal_friction(self.cloth,self.internal_friction)
        old=np.array([p.position.copy() for p in ps])
        weights=np.array([0. if p.fixed else p.inv_mass for p in ps]);dampness=np.array([p.dampness for p in ps])
        for p in ps:
            p.prev_position=p.position.copy()
            if not p.fixed:
                p.velocity+=(g+p.force*p.inv_mass)*dt
                p.position+=p.velocity*dt
            p.reset_force()
        positions=np.array([p.position for p in ps])
        for _ in range(self.iterations):
            for c in self.cloth.constraints:
                corrections=c.solve(positions,dt,inverse_masses=weights,dampness=dampness)
                for k,idx in enumerate(c.particles):positions[idx]+=corrections[k]
            movable=weights>0
            positions[movable,2]=np.maximum(positions[movable,2],self.ground_height)
        for i,p in enumerate(ps):
            if p.fixed:p.position=old[i];p.velocity[:]=0
            else:p.position=positions[i];p.velocity=(p.position-old[i])/dt
        detect_collisions(self.cloth,self.ground_height)
        # Ground projection already placed particles on the plane; apply a nonpenetrating velocity response there.
        for p in ps:
            if not p.fixed and p.position[2]<=self.ground_height+1e-12:
                p.velocity[2]=max(0.,p.velocity[2]);p.velocity[:2]*=1-self.friction
        if any(not np.isfinite(p.position).all() or not np.isfinite(p.velocity).all() for p in ps):raise FloatingPointError('nonfinite result')
