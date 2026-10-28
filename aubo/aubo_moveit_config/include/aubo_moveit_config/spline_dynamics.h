#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace aubo_moveit_config {
namespace spline_dynamics {
using Polynomial =
    std::vector<double>; // Increasing powers of normalized time u in [0,1].

inline double evaluate(const Polynomial &polynomial, double u) {
  double value = 0.0;
  for (auto coefficient = polynomial.rbegin(); coefficient != polynomial.rend();
       ++coefficient)
    value = value * u + *coefficient;
  return value;
}

inline Polynomial derivative(const Polynomial &polynomial) {
  Polynomial result;
  for (std::size_t i = 1; i < polynomial.size(); ++i)
    result.push_back(i * polynomial[i]);
  return result;
}

// Recursively partition at derivative roots, then bisect monotone intervals.
// Degree is at most four here. Including tangent roots is important: sign
// changes alone miss repeated roots and can underestimate a spline extremum.
inline std::vector<double> roots(const Polynomial &polynomial) {
  if (polynomial.size() <= 1)
    return {};
  std::vector<double> cuts{0.0};
  const auto critical = roots(derivative(polynomial));
  cuts.insert(cuts.end(), critical.begin(), critical.end());
  cuts.push_back(1.0);
  std::sort(cuts.begin(), cuts.end());
  cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
  double scale = 0.0;
  for (double coefficient : polynomial)
    scale += std::fabs(coefficient);
  if (scale == 0.0)
    return {};
  const double tolerance = scale * 1e-12;
  std::vector<double> result;
  for (double cut : cuts)
    if (std::fabs(evaluate(polynomial, cut)) <= tolerance)
      result.push_back(cut);
  for (std::size_t i = 1; i < cuts.size(); ++i) {
    double left = cuts[i - 1], right = cuts[i];
    double left_value = evaluate(polynomial, left);
    const double right_value = evaluate(polynomial, right);
    if ((left_value < 0.0 && right_value > 0.0) ||
        (left_value > 0.0 && right_value < 0.0)) {
      for (int iteration = 0; iteration < 60; ++iteration) {
        const double middle = (left + right) * 0.5;
        const double value = evaluate(polynomial, middle);
        if ((left_value < 0.0 && value < 0.0) ||
            (left_value > 0.0 && value > 0.0)) {
          left = middle;
          left_value = value;
        } else
          right = middle;
      }
      result.push_back((left + right) * 0.5);
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

inline double maximumMagnitude(const Polynomial &polynomial) {
  double peak = std::max(std::fabs(evaluate(polynomial, 0.0)),
                         std::fabs(evaluate(polynomial, 1.0)));
  for (double u : roots(derivative(polynomial)))
    peak = std::max(peak, std::fabs(evaluate(polynomial, u)));
  return peak;
}

// Same position/velocity/acceleration boundary conditions as ros_control's
// QuinticSplineSegment. Normalize time to avoid ill-conditioned short segments.
inline Polynomial quintic(double p0, double v0, double a0, double p1, double v1,
                          double a1, double duration) {
  for (double value : {p0, v0, a0, p1, v1, a1, duration})
    if (!std::isfinite(value))
      throw std::invalid_argument("nonfinite spline boundary");
  if (duration <= 0.0)
    throw std::invalid_argument("spline duration must be positive");
  const double d = p1 - p0;
  const double x = v0 * duration, y = v1 * duration;
  const double b = a0 * duration * duration, c = a1 * duration * duration;
  Polynomial polynomial{p0,
                        x,
                        b * 0.5,
                        10 * d - 6 * x - 4 * y - 1.5 * b + 0.5 * c,
                        -15 * d + 8 * x + 7 * y + 1.5 * b - c,
                        6 * d - 3 * x - 3 * y - 0.5 * b + 0.5 * c};
  for (double coefficient : polynomial)
    if (!std::isfinite(coefficient))
      throw std::invalid_argument("spline coefficient overflow");
  return polynomial;
}

inline std::array<double, 3> peaks(Polynomial position, double duration) {
  std::array<double, 3> result{};
  double divisor = 1.0;
  for (std::size_t order = 0; order < result.size(); ++order) {
    position = derivative(position);
    divisor *= duration;
    result[order] = maximumMagnitude(position) / divisor;
    if (!std::isfinite(result[order]))
      throw std::invalid_argument("spline derivative overflow");
  }
  return result;
}

inline double requiredScale(const std::array<double, 3> &peak,
                            const std::array<double, 3> &limit) {
  double scale = 1.0;
  for (std::size_t order = 0; order < peak.size(); ++order) {
    if (!std::isfinite(limit[order]) || limit[order] <= 0.0 ||
        !std::isfinite(peak[order]))
      throw std::invalid_argument("invalid spline dynamics limit");
    scale = std::max(scale,
                     std::pow(peak[order] / limit[order], 1.0 / (order + 1)));
  }
  return scale;
}
} // namespace spline_dynamics
} // namespace aubo_moveit_config
