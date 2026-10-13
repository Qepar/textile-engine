import ctypes
import numpy as np
class NativeDistance:
    def __init__(self,library_path):
        self.library=ctypes.CDLL(str(library_path));self.function=self.library.textile_distance_sweep
        self.function.argtypes=[ctypes.c_int64,ctypes.POINTER(ctypes.c_double),ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.c_double]
        self.function.restype=ctypes.c_int
    def sweep(self,positions,edges,rest,inverse_masses,stiffness):
        if not isinstance(positions,np.ndarray) or positions.dtype!=np.float64 or not positions.flags.c_contiguous or not positions.flags.writeable or positions.ndim!=2 or positions.shape[1]!=3:raise ValueError('positions require writable contiguous float64 N x 3')
        raw_edges=np.asarray(edges)
        if raw_edges.dtype.kind not in 'iu':raise ValueError('edge indices must be integers')
        edges=np.ascontiguousarray(raw_edges,dtype=np.int64);rest=np.ascontiguousarray(rest,dtype=np.float64);weights=np.ascontiguousarray(inverse_masses,dtype=np.float64)
        if edges.ndim!=2 or edges.shape[1]!=2 or rest.shape!=(len(edges),) or weights.shape!=(len(positions),):raise ValueError('invalid array dimensions')
        before=positions.copy()
        status=self.function(len(positions),positions.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),len(edges),edges.ctypes.data_as(ctypes.POINTER(ctypes.c_int64)),rest.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),weights.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),stiffness)
        if status:
            positions[:]=before
            raise ValueError('native distance input rejected: '+str(status))
