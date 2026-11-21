// SPDX-License-Identifier: AGPL-3.0-only
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace gc_yolo {

struct Box {
  float x;
  float y;
  float width;
  float height;
  float angle;
  float score;
  int class_id;
};

struct Covariance {
  float xx;
  float yy;
  float xy;
};

inline Covariance covariance(const Box &box) {
  const float w = box.width * box.width / 12.0f;
  const float h = box.height * box.height / 12.0f;
  const float c = std::cos(box.angle);
  const float s = std::sin(box.angle);
  return {w * (c * c) + h * (s * s), w * (s * s) + h * (c * c), (w - h) * c * s};
}

inline float probabilistic_iou(const Box &first, const Covariance &a, const Box &second, const Covariance &b) {
  constexpr float eps = 1e-7f;
  const float xx = a.xx + b.xx;
  const float yy = a.yy + b.yy;
  const float xy = a.xy + b.xy;
  const float determinant = xx * yy - xy * xy;
  const float dx = first.x - second.x;
  const float dy = first.y - second.y;
  const float t1 = (xx * (dy * dy) + yy * (dx * dx)) / (determinant + eps) * 0.25f;
  const float t2 = xy * (-dx) * dy / (determinant + eps) * 0.5f;
  const float det_a = std::max(0.0f, a.xx * a.yy - a.xy * a.xy);
  const float det_b = std::max(0.0f, b.xx * b.yy - b.xy * b.xy);
  const float t3 = std::log(determinant / (4.0f * std::sqrt(det_a * det_b) + eps) + eps) * 0.5f;
  const float distance = std::clamp(t1 + t2 + t3, eps, 100.0f);
  return 1.0f - std::sqrt(1.0f - std::exp(-distance) + eps);
}

inline std::vector<Box> decode_obb(const float *output, int anchors, int classes, float conf, float iou,
                                   size_t max_detections) {
  std::vector<Box> candidates;
  candidates.reserve(anchors);
  for (int anchor = 0; anchor < anchors; ++anchor) {
    const auto channel = [=](int index) { return output[static_cast<size_t>(index) * anchors + anchor]; };
    int best_class = 0;
    float score = channel(4);
    for (int id = 1; id < classes; ++id) {
      if (channel(4 + id) > score) {
        score = channel(4 + id);
        best_class = id;
      }
    }
    if (std::isfinite(score) && score > conf) {
      Box box{channel(0), channel(1), channel(2), channel(3), channel(4 + classes), score, best_class};
      if (std::isfinite(box.x) && std::isfinite(box.y) && std::isfinite(box.width) && std::isfinite(box.height) &&
          std::isfinite(box.angle) && box.width > 0 && box.height > 0) {
        candidates.push_back(box);
      }
    }
  }
  std::stable_sort(candidates.begin(), candidates.end(), [](const Box &a, const Box &b) { return a.score > b.score; });
  std::vector<Covariance> covariances;
  covariances.reserve(candidates.size());
  for (const auto &box : candidates) {
    covariances.push_back(covariance(box));
  }

  std::vector<Box> detections;
  for (size_t i = 0; i < candidates.size() && detections.size() < max_detections; ++i) {
    const auto &box = candidates[i];
    // Ultralytics Fast NMS compares against all higher-scoring boxes, including suppressed ones.
    bool suppressed = false;
    for (size_t j = 0; j < i; ++j) {
      if (box.class_id == candidates[j].class_id &&
          probabilistic_iou(box, covariances[i], candidates[j], covariances[j]) >= iou) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      detections.push_back(box);
    }
  }
  return detections;
}

inline void to_original_image(Box &box, double scale, int left, int top, int width, int height) {
  constexpr float half_pi = 1.5707963267948966f;
  constexpr float pi = 2 * half_pi;
  const float angle_mod_pi = std::fmod(box.angle, pi);
  if ((angle_mod_pi < 0 ? angle_mod_pi + pi : angle_mod_pi) >= half_pi) {
    std::swap(box.width, box.height);
  }
  box.angle = std::fmod(box.angle, half_pi);
  if (box.angle < 0) {
    box.angle += half_pi;
  }
  box.x = static_cast<float>(std::clamp((box.x - left) / scale, 0.0, static_cast<double>(width)));
  box.y = static_cast<float>(std::clamp((box.y - top) / scale, 0.0, static_cast<double>(height)));
  box.width = static_cast<float>(std::clamp(box.width / scale, 0.0, static_cast<double>(width)));
  box.height = static_cast<float>(std::clamp(box.height / scale, 0.0, static_cast<double>(height)));
}

}  // namespace gc_yolo
