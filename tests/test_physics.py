import unittest
from types import SimpleNamespace
import numpy as np
from core import Particle,Cloth,PBDSolver,DistanceConstraint,AnisotropicDistanceConstraint,WetDistanceConstraint,BendConstraint,SpatialGrid
from core.solver import apply_internal_friction
from core.collision import detect_collisions,resolve_collisions
class Physics(unittest.TestCase):
 def tiny(self,particles,constraints=None):return SimpleNamespace(particles=particles,constraints=constraints or [])
 def test_freefall(self):
  p=Particle([0,0,0]);PBDSolver(self.tiny([p]),ground_height=-100).step(.1)
  np.testing.assert_allclose(p.position,[0,0,-.0981]);np.testing.assert_allclose(p.velocity,[0,0,-.981])
 def test_free_motion(self):
  p=Particle([0,0,0],velocity=[1,0,0]);s=PBDSolver(self.tiny([p]))
  for _ in range(10):s.step(.1,gravity=[0,0,0])
  np.testing.assert_allclose(p.position,[1,0,0]);np.testing.assert_allclose(p.velocity,[1,0,0])
 def test_gravity_independent_mass(self):
  ps=[Particle([0,0,0],mass=m) for m in (1,3)];PBDSolver(self.tiny(ps),ground_height=-100).step(.1)
  np.testing.assert_allclose(ps[0].position,ps[1].position)
 def test_force_inverse_mass(self):
  ps=[Particle([0,0,0],mass=m) for m in (1,2)]
  for p in ps:p.apply_force(np.array([1,0,0]))
  PBDSolver(self.tiny(ps)).step(.1,gravity=(0,0,0));self.assertAlmostEqual(ps[0].velocity[0],2*ps[1].velocity[0])
 def test_pinned_ground(self):
  p=Particle([0,0,-2],fixed=True);c=self.tiny([p]);detect_collisions(c);resolve_collisions(c)
  PBDSolver(c).step(.1);np.testing.assert_array_equal(p.position,[0,0,-2])
 def test_ground_velocity_dissipates(self):
  p=Particle([0,0,-2],velocity=[2,0,-1]);c=self.tiny([p]);detect_collisions(c);resolve_collisions(c)
  self.assertGreaterEqual(p.position[2],-1);self.assertGreaterEqual(p.velocity[2],0);self.assertLessEqual(np.linalg.norm(p.velocity),np.sqrt(5))
 def test_mass_projection(self):
  corr=DistanceConstraint(0,1,1).solve(np.array([[0.,0,0],[2,0,0]]),.1,inverse_masses=[1,1/3])
  np.testing.assert_allclose(corr[:,0],[.75,-.25])
 def test_pinned_constraint_full_correction(self):
  ps=[Particle([0,0,0],fixed=True),Particle([2,0,0])];s=PBDSolver(self.tiny(ps,[DistanceConstraint(0,1,1)]),iterations=1)
  s.step(.1,gravity=(0,0,0));np.testing.assert_allclose(ps[1].position,[1,0,0])
 def test_friction_dissipates(self):
  a=Particle([0,0,0],velocity=[-1,0,0]);b=Particle([.1,0,0],velocity=[1,0,0]);apply_internal_friction(self.tiny([a,b]))
  self.assertLessEqual(np.dot(a.force,a.velocity)+np.dot(b.force,b.velocity),0)
 def test_rest_sheet_bending(self):
  c=Cloth(num_width=3,num_height=3);pos=np.array([p.position for p in c.particles])
  for k in c.constraints:np.testing.assert_allclose(k.solve(pos,.1,inverse_masses=np.ones(9),dampness=np.zeros(9)),0,atol=1e-14)
 def test_anisotropy_array(self):
  cor=AnisotropicDistanceConstraint(0,1,1).solve(np.array([[0.,0,0],[2,0,0]]),.1,inverse_masses=[1,1]);self.assertTrue(np.isfinite(cor).all())
 def test_wet_array(self):
  k=WetDistanceConstraint(0,1,1);pos=np.array([[0.,0,0],[2,0,0]])
  dry=k.solve(pos,.1,dampness=[0,0]);wet=k.solve(pos,.1,dampness=[1,1]);np.testing.assert_allclose(wet,.5*dry)
 def test_material_options(self):
  for kw in ({'use_anisotropic':True},{'use_wet_simulation':True}):
   c=Cloth(num_width=3,num_height=3,**kw);PBDSolver(c).step(.01);self.assertTrue(np.isfinite([p.position for p in c.particles]).all())
 def test_default_bounded_stability(self):
  c=Cloth(num_width=4,num_height=4);s=PBDSolver(c);anchors=[p.position.copy() for p in c.particles if p.fixed]
  for _ in range(120):s.step(1/60)
  pos=np.array([p.position for p in c.particles]);self.assertTrue(np.isfinite(pos).all());self.assertLess(abs(pos).max(),10)
  np.testing.assert_array_equal([p.position for p in c.particles if p.fixed],anchors)
 def test_invalid_stiffness(self):
  for value in (-1,10,float('nan')):
   with self.assertRaises(ValueError):Cloth(stiffness=value)
 def test_invalid_dt(self):
  s=PBDSolver(self.tiny([Particle([0,0,0])]))
  for dt in (0,-1,float('nan'),float('inf')):
   with self.assertRaises(ValueError):s.step(dt)
 def test_grid_negative_coordinates(self):self.assertEqual(SpatialGrid(.2)._get_cell([-.01,0,0]),(-1,0,0))
 def test_grid_unsupported_explicit(self):
  with self.assertRaises(NotImplementedError):PBDSolver(Cloth(),use_spatial_grid=True)
 def test_zero_mass_rejected(self):
  with self.assertRaises(ValueError):Particle([0,0,0],mass=0)
 def test_bend_error_reduced(self):
  pos=np.array([[1.,0,0],[0,0,0],[.5,1,0]]);k=BendConstraint(0,1,2,stiffness=.2,rest_angle=np.pi/2)
  def error(p):return abs(np.arccos(np.dot(p[0],p[2])/np.linalg.norm(p[0])/np.linalg.norm(p[2]))-np.pi/2)
  self.assertLess(error(pos+k.solve(pos,.1)),error(pos))
if __name__=='__main__':unittest.main()
