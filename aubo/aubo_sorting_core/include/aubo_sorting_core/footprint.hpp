#ifndef AUBO_SORTING_CORE_FOOTPRINT_HPP
#define AUBO_SORTING_CORE_FOOTPRINT_HPP
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

namespace aubo_sorting_core {
using Point2 = std::array<double, 2>;
using Footprint = std::vector<Point2>;
inline double cross2(const Point2& a, const Point2& b, const Point2& p) {
  return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]);
}
inline double signedArea(const Footprint& polygon) {
  double area = 0;
  for (std::size_t i=0; i<polygon.size(); ++i) {
    const auto& a=polygon[i]; const auto& b=polygon[(i+1)%polygon.size()];
    if (!std::isfinite(a[0]) || !std::isfinite(a[1])) return 0;
    area += a[0]*b[1]-b[0]*a[1];
  }
  return area*.5;
}
// Convex footprint intersection divided by the smaller footprint's area.
inline double footprintOverlap(const Footprint& a, const Footprint& b) {
  const double aa=std::abs(signedArea(a)), ab=std::abs(signedArea(b));
  if (a.size()<3 || b.size()<3 || aa<1e-10 || ab<1e-10) return 0;
  Footprint output=a;
  const double direction=signedArea(b)>0 ? 1 : -1;
  for (std::size_t i=0; i<b.size() && !output.empty(); ++i) {
    const auto& start=b[i]; const auto& end=b[(i+1)%b.size()];
    const Footprint input=output; output.clear();
    Point2 previous=input.back();
    double old_side=direction*cross2(start,end,previous);
    for (const auto& current: input) {
      const double side=direction*cross2(start,end,current);
      if ((side>=0)!=(old_side>=0)) {
        const double t=old_side/(old_side-side);
        output.push_back({{previous[0]+t*(current[0]-previous[0]),
                           previous[1]+t*(current[1]-previous[1])}});
      }
      if (side>=0) output.push_back(current);
      previous=current; old_side=side;
    }
  }
  return std::min(1., std::max(0., std::abs(signedArea(output))/std::min(aa,ab)));
}
}
#endif
