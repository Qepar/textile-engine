"""Experimental PBD projections with explicit array and inverse-mass inputs.
Stiffness is a dimensionless projection fraction in [0, 1], not a spring constant.
Three-point angle bending is not a calibrated shell/dihedral material model.
"""
import numpy as np

def fraction(value):
    value=float(value)
    if not np.isfinite(value) or not 0<=value<=1:raise ValueError('stiffness must be in [0, 1]')
    return value

class Constraint:
    def __init__(self,particles,stiffness=1.):
        self.particles=list(particles);self.stiffness=fraction(stiffness)
    def weights(self,inverse_masses):
        return np.ones(len(self.particles)) if inverse_masses is None else np.asarray(inverse_masses)[self.particles]
    def solve(self,positions,dt,inverse_masses=None,dampness=None):raise NotImplementedError

class DistanceConstraint(Constraint):
    def __init__(self,p1_idx,p2_idx,rest_length,stiffness=1.):
        super().__init__([p1_idx,p2_idx],stiffness);self.rest_length=float(rest_length)
        if not np.isfinite(self.rest_length) or self.rest_length<=0:raise ValueError('rest length must be positive')
    def effective_stiffness(self,delta,dampness):return self.stiffness
    def solve(self,positions,dt,inverse_masses=None,dampness=None):
        a,b=np.asarray(positions)[self.particles];delta=b-a;length=np.linalg.norm(delta)
        weights=self.weights(inverse_masses);total=weights.sum()
        if length<=1e-15 or total<=0:return np.zeros((2,3))
        correction=self.effective_stiffness(delta,dampness)*(length-self.rest_length)*delta/length/total
        return np.array([weights[0]*correction,-weights[1]*correction])

class BendConstraint(Constraint):
    def __init__(self,p1_idx,p2_idx,p3_idx,stiffness=.5,rest_angle=np.pi):
        super().__init__([p1_idx,p2_idx,p3_idx],stiffness);self.rest_angle=float(rest_angle)
        if not np.isfinite(self.rest_angle) or not 0<=self.rest_angle<=np.pi:raise ValueError('invalid rest angle')
    def set_rest_angle(self,positions):
        a,b,c=np.asarray(positions)[self.particles];u=a-b;v=c-b
        self.rest_angle=float(np.arccos(np.clip(np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v)),-1,1)))
    def solve(self,positions,dt,inverse_masses=None,dampness=None):
        a,b,c=np.asarray(positions)[self.particles];u=a-b;v=c-b
        l1=np.linalg.norm(u);l2=np.linalg.norm(v)
        if min(l1,l2)<=1e-15:return np.zeros((3,3))
        cosine=np.clip(np.dot(u,v)/(l1*l2),-1,1);sine=np.sqrt(max(0,1-cosine*cosine))
        if sine<=1e-10:return np.zeros((3,3))
        ga=-(v/(l1*l2)-cosine*u/l1**2)/sine
        gc=-(u/(l1*l2)-cosine*v/l2**2)/sine
        grads=np.array([ga,-ga-gc,gc]);weights=self.weights(inverse_masses)
        denom=np.sum(weights*np.sum(grads*grads,axis=1))
        if denom<=1e-15:return np.zeros((3,3))
        dl=-self.stiffness*(np.arccos(cosine)-self.rest_angle)/denom
        return weights[:,None]*dl*grads

class ShearConstraint(Constraint):
    def __init__(self,p1_idx,p2_idx,p3_idx,p4_idx,stiffness=.8):
        super().__init__([p1_idx,p2_idx,p3_idx,p4_idx],stiffness);self.rest_length=0.
    def set_rest_length(self,positions):
        a,b,c,d=np.asarray(positions)[self.particles];self.rest_length=np.linalg.norm(c-a-d+b)
    def solve(self,positions,dt,inverse_masses=None,dampness=None):
        a,b,c,d=np.asarray(positions)[self.particles];delta=c-a-d+b;length=np.linalg.norm(delta)
        weights=self.weights(inverse_masses)
        if length<=1e-15 or weights.sum()<=0:return np.zeros((4,3))
        grads=np.array([-delta,delta,delta,-delta])/length
        dl=-self.stiffness*(length-self.rest_length)/weights.sum()
        return weights[:,None]*dl*grads

class AnisotropicDistanceConstraint(DistanceConstraint):
    def __init__(self,p1_idx,p2_idx,rest_length,stiffness_x=1.,stiffness_y=.5,direction=(1,0,0)):
        super().__init__(p1_idx,p2_idx,rest_length)
        self.stiffness_x=fraction(stiffness_x);self.stiffness_y=fraction(stiffness_y)
        direction=np.asarray(direction,dtype=float);n=np.linalg.norm(direction)
        if direction.shape!=(3,) or not np.isfinite(direction).all() or n<=0:raise ValueError('invalid direction')
        self.direction=direction/n
    def effective_stiffness(self,delta,dampness):
        cosine=np.dot(delta,self.direction)/np.linalg.norm(delta)
        return self.stiffness_x*cosine**2+self.stiffness_y*(1-cosine**2)

class WetDistanceConstraint(DistanceConstraint):
    def effective_stiffness(self,delta,dampness):
        if dampness is None:raise ValueError('dampness array required')
        values=np.asarray(dampness)[self.particles]
        if not np.isfinite(values).all() or np.any((values<0)|(values>1)):raise ValueError('invalid dampness')
        return self.stiffness*(1-.5*values.mean())
