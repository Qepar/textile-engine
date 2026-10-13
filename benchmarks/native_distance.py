"""Parity-gated kernel benchmark. No end-to-end or GPU performance claim."""
import sys,json,time,platform,statistics
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.constraints import DistanceConstraint
from native.distance_backend import NativeDistance
backend=NativeDistance(Path(sys.argv[1]).resolve());results=[]
def timed(function):
 samples=[];calls=[]
 for _ in range(3):
  count=0;start=time.perf_counter()
  while time.perf_counter()-start<.15:
   function();count+=1
  elapsed=time.perf_counter()-start
  samples.append(elapsed/count);calls.append(count)
 return statistics.median(samples),samples,calls

for n in (10,20,50):
 positions=np.array([[j/n,i/n,0.] for i in range(n) for j in range(n)],dtype=np.float64)
 edges=np.array([(i*n+j,i*n+j+1) for i in range(n) for j in range(n-1)]+[(i*n+j,(i+1)*n+j) for i in range(n-1) for j in range(n)],dtype=np.int64)
 rest=np.linalg.norm(positions[edges[:,1]]-positions[edges[:,0]],axis=1);weights=np.ones(n*n);weights[0]=0
 positions*=1.1
 constraints=[DistanceConstraint(int(a),int(b),float(r),.8) for (a,b),r in zip(edges,rest)]
 def python_sweep(x):
  for c in constraints:
   corrections=c.solve(x,1/60,inverse_masses=weights)
   for k,i in enumerate(c.particles):x[i]+=corrections[k]
 a=positions.copy();b=positions.copy()
 start=time.perf_counter()
 for _ in range(5):python_sweep(a)
 python_seconds=time.perf_counter()-start
 start=time.perf_counter()
 for _ in range(5):backend.sweep(b,edges,rest,weights,.8)
 native_seconds=time.perf_counter()-start
 error=float(np.max(np.abs(a-b)));assert error<1e-12 and np.array_equal(a[0],positions[0])
 python_seconds,python_samples,python_calls=timed(lambda:python_sweep(a))
 native_seconds,native_samples,native_calls=timed(lambda:backend.sweep(b,edges,rest,weights,.8))
 results.append({'python_samples_per_sweep':python_samples,'native_samples_per_sweep':native_samples,'python_timing_calls':python_calls,'native_timing_calls':native_calls,'grid':n,'vertices':n*n,'edges':len(edges),'parity_sweeps':5,'python_median_seconds_per_sweep':python_seconds,'native_median_seconds_per_sweep':native_seconds,'kernel_speedup':python_seconds/native_seconds,'max_absolute_position_error':error})
# Invalid topology must be rejected before mutation.
x=np.zeros((2,3));before=x.copy()
try:backend.sweep(x,np.array([[0,2]]),[1.],[1.,1.],.8)
except ValueError:pass
else:raise AssertionError('invalid index accepted')
assert np.array_equal(x,before)
x=np.array([[1e308,0,0],[-1e308,0,0]])
before=x.copy()
try:backend.sweep(x,np.array([[0,1]]),[1.],[1.,1.],.8)
except ValueError:pass
else:raise AssertionError('overflow accepted')
assert np.array_equal(x,before)
report={'status':'PASS' ,'scope':'DISTANCE_KERNEL_ONLY_NOT_FULL_SIMULATOR','environment':platform.platform(),'numpy':np.__version__,'results':results,'invalid_input_no_mutation':True,'overflow_rollback':True,'timing_protocol':'3 repeated batches >=0.15 seconds; median per sweep; warmed and converging state; timings exclude construction and compilation'}
Path(sys.argv[2]).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
