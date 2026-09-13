#ifndef AUBO_SORTING_CORE_INSTANCE_QUEUE_HPP
#define AUBO_SORTING_CORE_INSTANCE_QUEUE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace aubo_sorting_core
{
// ROS-independent tracker. Its owner serializes access; no lock is held during motion.
template<class Payload> class InstanceQueue
{
public:
  enum class Status { CANDIDATE, READY, RESERVED, DONE, REVIEW };
  struct Sample
  {
    std::string color;
    double x{0}, y{0}, z{0};
    bool eligible{true};
    Payload payload;
  };
  struct Track
  {
    std::uint64_t id{0};
    Sample sample;
    Status status{Status::CANDIDATE};
    unsigned observations{0};
    double last_seen{0}, changed_at{0};
    bool disturbed{false};
    double disturbance_distance{0};
    std::string disturbance_color;
  };
  double match_distance{0.04}, stable_distance{0.015}, reserved_distance{0.035}, duplicate_distance{0.008};
  double max_age{3.0}, confirmation_gap{1.0}, retention{30.0}, done_hold{2.0};
  unsigned min_observations{5};
  std::size_t capacity{200};

  static const char* statusName(Status status)
  {
    switch (status) {
      case Status::CANDIDATE: return "candidate";
      case Status::READY: return "ready";
      case Status::RESERVED: return "reserved";
      case Status::DONE: return "done";
      case Status::REVIEW: return "review";
    }
    return "review";
  }
  const std::map<std::uint64_t, Track>& tracks() const { return tracks_; }
  void clear() { tracks_.clear(); } // Never recycle IDs across workspaces.
  void invalidate()
  {
    for (auto& item : tracks_)
      if (item.second.status != Status::DONE && item.second.status != Status::RESERVED &&
          item.second.status != Status::REVIEW) {
        item.second.status = Status::CANDIDATE;
        item.second.observations = 0;
      }
  }
  void update(const std::vector<Sample>& input, double now)
  {
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (it->second.status != Status::RESERVED && now - it->second.last_seen > retention)
        it = tracks_.erase(it);
      else ++it;
    }
    std::vector<Sample> samples;
    for (const auto& sample : input) {
      if (!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.z)) continue;
      bool duplicate = false;
      for (const auto& kept : samples)
        duplicate = duplicate || (sample.color == kept.color && distance(sample, kept) < duplicate_distance);
      if (!duplicate) samples.push_back(sample);
    }
    struct Edge { double distance; std::uint64_t id; std::size_t sample; };
    std::vector<Edge> edges;
    std::set<std::size_t> used_samples;
    // Reserved objects also participate in one-to-one matching. A radius-wide
    // suppression would wrongly consume a distinct neighbouring object.
    for (std::size_t i = 0; i < samples.size(); ++i)
      for (auto& item : tracks_) {
        auto& track = item.second;
        const double d = distance(track.sample, samples[i]);
        if ((track.sample.color == samples[i].color || track.status == Status::RESERVED) && d <= match_distance)
          edges.push_back({d, item.first, i});
      }
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
      if (a.distance != b.distance) return a.distance < b.distance;
      if (a.id != b.id) return a.id < b.id;
      return a.sample < b.sample;
    });
    std::set<std::uint64_t> used_tracks;
    for (const auto& edge : edges) {
      if (used_samples.count(edge.sample) || used_tracks.count(edge.id)) continue;
      used_samples.insert(edge.sample);
      used_tracks.insert(edge.id);
      auto& track = tracks_.at(edge.id);
      if (track.status == Status::RESERVED) {
        // 机械臂移动时手眼视角会让轮廓中心轻微偏移；预留目标采用独立的执行容差。
        if (edge.distance > reserved_distance || samples[edge.sample].color != track.sample.color) {
          track.disturbed = true;
          track.disturbance_distance = edge.distance;
          track.disturbance_color = samples[edge.sample].color;
        } else track.last_seen = now;
        continue;
      }
      if (track.status == Status::DONE && now - track.changed_at < done_hold) continue;
      const Sample& sample = samples[edge.sample];
      const bool stable = edge.distance <= stable_distance && now - track.last_seen <= confirmation_gap &&
                          track.status != Status::DONE && track.status != Status::REVIEW &&
                          track.sample.eligible && sample.eligible;
      track.observations = sample.eligible ? (stable ? track.observations + 1 : 1) : 0;
      Sample filtered = sample;
      if (stable) { // Bounded-history smoothing; payload retains current grasp geometry.
        filtered.x = 0.5 * (sample.x + track.sample.x);
        filtered.y = 0.5 * (sample.y + track.sample.y);
        filtered.z = 0.5 * (sample.z + track.sample.z);
      }
      track.sample = filtered;
      track.last_seen = now;
      track.status = sample.eligible && track.observations >= min_observations ? Status::READY : Status::CANDIDATE;
    }
    // 超出关联半径的同色新观测也可能是预留目标的大幅位移，不能让旧坐标继续有效。
    for (auto& item : tracks_) {
      auto& track = item.second;
      if (track.status != Status::RESERVED || used_tracks.count(item.first)) continue;
      for (std::size_t i = 0; i < samples.size(); ++i) {
        if (used_samples.count(i) || samples[i].color != track.sample.color) continue;
        track.disturbed = true;
        track.disturbance_distance = distance(track.sample, samples[i]);
        track.disturbance_color = samples[i].color;
        break;
      }
    }
    for (std::size_t i = 0; i < samples.size() && tracks_.size() < capacity; ++i) {
      if (used_samples.count(i)) continue;
      // A newly appearing same-class object could be an unmatched moved object.
      // Reconfirm unmatched old tracks instead of dispatching their old coordinates.
      for (auto& item : tracks_)
        if (!used_tracks.count(item.first) && item.second.sample.color == samples[i].color &&
            item.second.status == Status::READY) {
          item.second.status = Status::CANDIDATE;
          item.second.observations = 0;
        }
      Track track;
      track.id = next_id_++;
      track.sample = samples[i];
      track.observations = 1;
      track.last_seen = now;
      tracks_[track.id] = track;
    }
  }
  bool reserve(const std::vector<std::string>& colors, double now, Track& result)
  {
    for (const auto& color : colors)
      for (auto& item : tracks_) {
        auto& track = item.second;
        if (track.sample.color != color || track.status != Status::READY || now - track.last_seen > max_age) continue;
        track.status = Status::RESERVED;
        track.disturbed = false;
        track.disturbance_distance = 0;
        track.disturbance_color.clear();
        track.changed_at = now;
        result = track; // Immutable execution snapshot, never a pointer into the cache.
        return true;
      }
    return false;
  }
  void finish(std::uint64_t id, bool success, double now)
  {
    auto found = tracks_.find(id);
    if (found == tracks_.end()) return;
    found->second.status = success ? Status::DONE : Status::REVIEW;
    found->second.changed_at = found->second.last_seen = now;
    found->second.observations = 0;
    if (!success) invalidate(); // A failed motion may have disturbed neighbours.
  }
  bool executionValid(std::uint64_t id, double now, double reserved_max_age) const
  {
    const auto found = tracks_.find(id);
    return found != tracks_.end() && found->second.status == Status::RESERVED &&
        !found->second.disturbed && now - found->second.last_seen <= reserved_max_age;
  }
  bool executionValid(std::uint64_t id, double now) const { return executionValid(id, now, max_age); }
private:
  static double distance(const Sample& a, const Sample& b) { return std::hypot(a.x-b.x, a.y-b.y); }
  std::map<std::uint64_t, Track> tracks_;
  std::uint64_t next_id_{1};
};
} // namespace aubo_sorting_core
#endif
