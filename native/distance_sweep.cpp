// Optional distance-only Gauss-Seidel kernel. No collision or material model.
#include <cmath>
#include <cstdint>
#if defined(_WIN32)
#define API __declspec(dllexport)
#else
#define API
#endif
extern "C" API int textile_distance_sweep(std::int64_t n, double* x,
    std::int64_t m, const std::int64_t* edges, const double* rest,
    const double* weights, double stiffness) {
    if(n<1 || m<0 || !x || !weights || !std::isfinite(stiffness) || stiffness<0 || stiffness>1) return 1;
    if(m && (!edges || !rest)) return 1;
    for(std::int64_t i=0;i<n;++i) {
        if(!std::isfinite(weights[i]) || weights[i]<0) return 2;
        for(int k=0;k<3;++k) if(!std::isfinite(x[3*i+k])) return 2;
    }
    for(std::int64_t e=0;e<m;++e)
        if(edges[2*e]<0 || edges[2*e]>=n || edges[2*e+1]<0 || edges[2*e+1]>=n || !std::isfinite(rest[e]) || rest[e]<=0) return 3;
    for(std::int64_t e=0;e<m;++e) {
        auto a=edges[2*e], b=edges[2*e+1]; double delta[3];
        for(int k=0;k<3;++k) delta[k]=x[3*b+k]-x[3*a+k];
        double length=std::hypot(delta[0],delta[1],delta[2]);
        if(!std::isfinite(length)) return 4;
        double sum=weights[a]+weights[b]; if(length<=1e-15 || sum<=0) continue;
        double factor=stiffness*(length-rest[e])/length/sum;
        double next_a[3], next_b[3];
        for(int k=0;k<3;++k) {double d=factor*delta[k];next_a[k]=x[3*a+k]+weights[a]*d;next_b[k]=x[3*b+k]-weights[b]*d;if(!std::isfinite(next_a[k]) || !std::isfinite(next_b[k])) return 4;}
        for(int k=0;k<3;++k) {x[3*a+k]=next_a[k];x[3*b+k]=next_b[k];}
    }
    return 0;
}
