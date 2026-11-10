#ifndef AUBO_SORTING_CORE_OCCLUSION_GEOMETRY_HPP
#define AUBO_SORTING_CORE_OCCLUSION_GEOMETRY_HPP
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
namespace aubo_sorting_core {
using Point3 = std::array<double,3>;
inline Point3 subtract3(const Point3& a, const Point3& b) {
  return {{a[0]-b[0],a[1]-b[1],a[2]-b[2]}};
}
inline double dot3(const Point3& a,const Point3& b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline double pointSegmentDistance2(const Point3& p,const Point3& a,const Point3& b) {
  const auto u=subtract3(b,a), w=subtract3(p,a);
  const double length=dot3(u,u), t=length>1e-16 ? std::max(0.,std::min(1.,dot3(w,u)/length)) : 0;
  const Point3 delta{{w[0]-t*u[0],w[1]-t*u[1],w[2]-t*u[2]}};
  return dot3(delta,delta);
}
inline double segmentDistance2(const Point3& p,const Point3& q,const Point3& a,const Point3& b) {
  double result=std::min({pointSegmentDistance2(p,a,b),pointSegmentDistance2(q,a,b),
                          pointSegmentDistance2(a,p,q),pointSegmentDistance2(b,p,q)});
  const auto u=subtract3(q,p), v=subtract3(b,a), w=subtract3(p,a);
  const double aa=dot3(u,u), bb=dot3(u,v), cc=dot3(v,v), dd=dot3(u,w), ee=dot3(v,w);
  const double determinant=aa*cc-bb*bb;
  if (determinant>1e-16) {
    const double s=(bb*ee-cc*dd)/determinant, t=(aa*ee-bb*dd)/determinant;
    if (s>=0 && s<=1 && t>=0 && t<=1) {
      const Point3 delta{{w[0]+s*u[0]-t*v[0],w[1]+s*u[1]-t*v[1],w[2]+s*u[2]-t*v[2]}};
      result=std::min(result,dot3(delta,delta));
    }
  }
  return result;
}
// Robot link origins form conservative capsules. Ignore the first 8 cm of the
// ray to avoid treating an eye-in-hand camera's own mounting as an occluder.
inline bool robotOccludesTarget(const Point3& camera,const Point3& target,
                                const std::vector<Point3>& links,double radius) {
  if (!std::isfinite(radius) || radius<=0 || links.empty()) return false;
  for (const auto& p: links) for (double v:p) if (!std::isfinite(v)) return false;
  for (double v:camera) if (!std::isfinite(v)) return false;
  for (double v:target) if (!std::isfinite(v)) return false;
  const auto ray=subtract3(target,camera); const double length=std::sqrt(dot3(ray,ray));
  if (length<=.08) return false;
  const Point3 start{{camera[0]+.08*ray[0]/length,camera[1]+.08*ray[1]/length,camera[2]+.08*ray[2]/length}};
  if (pointSegmentDistance2(links[0],start,target)<=radius*radius) return true;
  for (std::size_t i=1;i<links.size();++i)
    if (segmentDistance2(start,target,links[i-1],links[i])<=radius*radius) return true;
  return false;
}
// A partially hidden outline can change its class/center before the center ray
// is hidden. Test the frozen footprint boundary as well as the center ray.
inline bool robotOccludesRegion(const Point3& camera,const Point3& center,
                                const std::vector<Point3>& boundary,
                                const std::vector<Point3>& links,double radius) {
  if (robotOccludesTarget(camera,center,links,radius)) return true;
  for (std::size_t i=0;i<boundary.size();++i) {
    const auto& a=boundary[i];const auto& b=boundary[(i+1)%boundary.size()];
    const Point3 midpoint{{(a[0]+b[0])/2,(a[1]+b[1])/2,(a[2]+b[2])/2}};
    if (robotOccludesTarget(camera,a,links,radius) ||
        robotOccludesTarget(camera,midpoint,links,radius)) return true;
  }
  return false;
}
}
#endif
