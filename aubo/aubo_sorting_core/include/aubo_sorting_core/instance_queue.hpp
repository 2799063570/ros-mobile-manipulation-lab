#ifndef AUBO_SORTING_CORE_INSTANCE_QUEUE_HPP
#define AUBO_SORTING_CORE_INSTANCE_QUEUE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <aubo_sorting_core/footprint.hpp>

namespace aubo_sorting_core
{
// 与 ROS 无关的目标实例缓存。调用方负责串行访问；机械臂运动期间不持有队列锁。
template<class Payload> class InstanceQueue
{
public:
  // 候选：等待多帧确认；就绪：允许分配；预留：正在执行抓取；
  // 完成：短暂保留以屏蔽残留画面；复核：执行失败，不能直接再次分配。
  enum class Status { CANDIDATE, READY, RESERVED, DONE, REVIEW };
  // 单帧观测。eligible 表示几何条件允许抓取；payload 保存业务层原始检测结果。
  struct Sample
  {
    std::string category;
    double x{0}, y{0}, z{0};
    double confidence{0};
    Footprint footprint;
    bool eligible{true};
    Payload payload;
  };
  // 同一物体跨帧形成的轨迹；id 在 clear() 后也不会重新使用。
  struct Track
  {
    std::uint64_t id{0};
    Sample sample;
    Status status{Status::CANDIDATE};
    unsigned observations{0};
    double last_seen{0}, changed_at{0}, sampled_at{0};
    bool disturbed{false};
    double disturbance_distance{0};
    std::string disturbance_category;
  };
  // 距离单位为米：关联半径、稳定确认半径、预留目标容差、同帧去重半径。
  double match_distance{0.04}, stable_distance{0.015}, reserved_distance{0.035}, duplicate_distance{0.008};
  double merge_distance{0.02}, merge_overlap{0.5};
  // 时间单位为秒：可分配观测时效、连续确认间隔、轨迹保留期、完成后屏蔽期。
  double max_age{3.0}, confirmation_gap{1.0}, retention{30.0}, done_hold{2.0};
  // 达到确认帧数才可分配；容量限制防止轨迹无限增长。
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
  std::uint64_t mergedDetectionsTotal() const { return merged_detections_total_; }
  void clear() { tracks_.clear(); } // 清空工作区轨迹，但不回收已分配的 ID。
  // 感知中断或抓取失败后，要求可用目标重新积累稳定观测。
  void invalidate()
  {
    for (auto& item : tracks_)
      if (item.second.status != Status::DONE && item.second.status != Status::RESERVED &&
          item.second.status != Status::REVIEW) {
        item.second.status = Status::CANDIDATE;
        item.second.observations = 0;
      }
  }
  // 处理一帧观测：清理过期轨迹、去重、一对一关联、更新确认状态并加入新目标。
  // now 与其他时间参数必须使用同一种时钟。
  void update(const std::vector<Sample>& input, double now)
  {
    // 预留中的目标即使暂时被遮挡，也由执行流程决定何时结束。
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (it->second.status != Status::RESERVED && now - it->second.last_seen > retention)
        it = tracks_.erase(it);
      else ++it;
    }
    // Class-agnostic confidence NMS in world geometry. Preserve the earliest
    // input position of each group so confidence sorting does not reorder FIFO.
    std::vector<std::size_t> order;
    for (std::size_t i=0;i<input.size();++i)
      if (std::isfinite(input[i].x) && std::isfinite(input[i].y) && std::isfinite(input[i].z) &&
          std::isfinite(input[i].confidence)) order.push_back(i);
    std::stable_sort(order.begin(),order.end(),[&input](std::size_t a,std::size_t b) {
      return input[a].confidence>input[b].confidence;
    });
    std::vector<std::pair<std::size_t,Sample>> kept;
    std::vector<std::vector<std::size_t>> duplicate_groups;
    for (const auto index:order) {
      auto found=std::find_if(kept.begin(),kept.end(),[this,&input,index](const std::pair<std::size_t,Sample>& item) {
        return duplicateSamples(input[index],item.second);
      });
      if (found==kept.end()) { kept.emplace_back(index,input[index]); duplicate_groups.push_back({index}); }
      else {
        duplicate_groups[static_cast<std::size_t>(found-kept.begin())].push_back(index);
        found->first=std::min(found->first,index); ++merged_detections_total_;
      }
    }
    mergeObservedTracks(input,duplicate_groups,now);
    std::sort(kept.begin(),kept.end(),[](const std::pair<std::size_t,Sample>& a,const std::pair<std::size_t,Sample>& b) {
      return a.first<b.first;
    });
    std::vector<Sample> samples;
    for (const auto& item:kept) samples.push_back(item.second);
    struct Edge { double distance; std::uint64_t id; std::size_t sample; };
    std::vector<Edge> edges;
    std::set<std::size_t> used_samples;
    // 预留目标也参加一对一匹配，避免按半径屏蔽时误吞掉附近的另一物体。
    // 按距离由近到远选边，每个观测和轨迹在本帧最多匹配一次。
    for (std::size_t i = 0; i < samples.size(); ++i)
      for (auto& item : tracks_) {
        auto& track = item.second;
        const double d = distance(track.sample, samples[i]);
        if ((track.sample.category == samples[i].category || track.status == Status::RESERVED ||
             overlappingDuplicate(track.sample,samples[i])) && d <= match_distance)
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
        if (edge.distance > reserved_distance || (samples[edge.sample].category != track.sample.category &&
                                                 !overlappingDuplicate(track.sample,samples[edge.sample]))) {
          track.disturbed = true;
          track.disturbance_distance = edge.distance;
          track.disturbance_category = samples[edge.sample].category;
        } else track.last_seen = now;
        continue;
      }
      // 已完成目标在屏蔽期内吸收原位置的残留检测，不重新成为可抓取目标。
      if (track.status == Status::DONE && now - track.changed_at < done_hold) continue;
      const Sample& sample = samples[edge.sample];
      const bool different_class=sample.category!=track.sample.category;
      const bool retain_best=different_class && sample.confidence<=track.sample.confidence &&
          edge.distance<=stable_distance && now-track.sampled_at<=max_age;
      const bool stable = edge.distance <= stable_distance && now - track.last_seen <= confirmation_gap &&
                          track.status != Status::DONE && track.status != Status::REVIEW &&
                          track.sample.eligible && sample.eligible && (!different_class || retain_best);
      track.observations = sample.eligible ? (stable ? track.observations + 1 : 1) : 0;
      Sample filtered = retain_best ? track.sample : sample;
      if (stable && !retain_best) { // 坐标与上次轨迹坐标平滑；payload 保留当前帧的抓取几何信息。
        filtered.x = 0.5 * (sample.x + track.sample.x);
        filtered.y = 0.5 * (sample.y + track.sample.y);
        filtered.z = 0.5 * (sample.z + track.sample.z);
      }
      track.sample = filtered;
      if (!retain_best) track.sampled_at=now;
      track.last_seen = now;
      track.status = sample.eligible && track.observations >= min_observations ? Status::READY : Status::CANDIDATE;
    }
    // One new observation must not validate a second historical same-class
    // record that lost association to a closer cross-class record.
    for (auto& item:tracks_) {
      auto& track=item.second;
      if (used_tracks.count(item.first) || track.status!=Status::READY) continue;
      for (std::size_t i=0;i<samples.size();++i)
        if (used_samples.count(i) && samples[i].category==track.sample.category &&
            distance(samples[i],track.sample)<=match_distance &&
            footprintOverlap(samples[i].footprint,track.sample.footprint)>=merge_overlap) {
          track.status=Status::CANDIDATE; track.observations=0; break;
        }
    }
    // 超出关联半径的同类新观测也可能是预留目标的大幅位移，不能让旧坐标继续有效。
    for (auto& item : tracks_) {
      auto& track = item.second;
      if (track.status != Status::RESERVED || used_tracks.count(item.first)) continue;
      for (std::size_t i = 0; i < samples.size(); ++i) {
        if (used_samples.count(i) || samples[i].category != track.sample.category) continue;
        track.disturbed = true;
        track.disturbance_distance = distance(track.sample, samples[i]);
        track.disturbance_category = samples[i].category;
        break;
      }
    }
    for (std::size_t i = 0; i < samples.size() && tracks_.size() < capacity; ++i) {
      if (used_samples.count(i)) continue;
      // 未匹配的新观测可能是旧同类目标移动后的结果；旧目标须重新确认，
      // 避免继续按旧坐标执行抓取。
      for (auto& item : tracks_)
        if (!used_tracks.count(item.first) && item.second.sample.category == samples[i].category &&
            item.second.status == Status::READY) {
          item.second.status = Status::CANDIDATE;
          item.second.observations = 0;
        }
      Track track;
      track.id = next_id_++;
      track.sample = samples[i];
      track.observations = 1;
      track.last_seen = now;
      track.sampled_at = now;
      tracks_[track.id] = track;
    }
    mergeTracks(now);
  }
  // categories 只作为允许类别列表；按入队 ID 领取最早的近期就绪目标。
  bool reserve(const std::vector<std::string>& categories, double now, Track& result)
  {
    for (auto& item : tracks_)
    {
      auto& track = item.second;
      if (track.status != Status::READY || now - track.last_seen > max_age || now-track.sampled_at>max_age ||
          std::find(categories.begin(), categories.end(), track.sample.category) == categories.end()) continue;
      track.status = Status::RESERVED;
      track.disturbed = false;
      track.disturbance_distance = 0;
      track.disturbance_category.clear();
      track.changed_at = now;
      result = track; // 返回独立的执行快照，后续帧不会改变本次抓取坐标。
      return true;
    }
    return false;
  }
  // 成功后进入残留画面屏蔽期；失败后隔离该目标并使其他可用目标重新确认。
  void finish(std::uint64_t id, bool success, double now)
  {
    auto found = tracks_.find(id);
    if (found == tracks_.end()) return;
    found->second.status = success ? Status::DONE : Status::REVIEW;
    found->second.changed_at = found->second.last_seen = now;
    found->second.observations = 0;
    if (!success) invalidate(); // 失败的运动也可能碰动邻近目标。
  }
  // 抓取下降前检查：目标仍被预留、未发现位移，且最近观测未超时。
  bool executionValid(std::uint64_t id, double now, double reserved_max_age) const
  {
    const auto found = tracks_.find(id);
    return found != tracks_.end() && found->second.status == Status::RESERVED &&
        !found->second.disturbed && now - found->second.last_seen <= reserved_max_age;
  }
  bool executionValid(std::uint64_t id, double now) const { return executionValid(id, now, max_age); }
private:
  int protectionRank(const Track& track,double now) const {
    return track.status==Status::RESERVED ? 3 :
        (track.status==Status::DONE && now-track.changed_at<done_hold ? 2 : (track.status==Status::REVIEW ? 1 : 0));
  }
  void mergeObservedTracks(const std::vector<Sample>& input,
                           const std::vector<std::vector<std::size_t>>& groups,double now) {
    for (const auto& group:groups) {
      if (group.size()<2) continue;
      std::set<std::uint64_t> ids;
      for (auto index:group) {
        double closest=match_distance;
        std::uint64_t nearest=0;
        for (const auto& item:tracks_) {
          const auto& track=item.second;
          const double d=positionDistance(input[index],track.sample);
          if (track.sample.category==input[index].category && now-track.last_seen<=max_age &&
              d<=closest && (d<closest || nearest==0)) { closest=d; nearest=item.first; }
        }
        if (nearest) ids.insert(nearest);
      }
      if (ids.size()<2) continue;
      auto keep=tracks_.find(*ids.begin());
      unsigned reserved=0;
      for (auto id:ids) {
        auto it=tracks_.find(id);
        reserved+=it->second.status==Status::RESERVED;
        if (protectionRank(it->second,now)>protectionRank(keep->second,now)) keep=it;
      }
      if (reserved>1) continue;
      for (auto id:ids) if (id!=keep->first) {
        const auto& removed=tracks_.at(id);
        if (keep->second.status==Status::RESERVED && (removed.disturbed || protectionRank(removed,now)>0)) {
          keep->second.disturbed=true;
          keep->second.disturbance_category=removed.sample.category;
          keep->second.disturbance_distance=distance(keep->second.sample,removed.sample);
        }
        tracks_.erase(id); ++merged_detections_total_;
      }
    }
  }
  bool overlappingDuplicate(const Sample& a,const Sample& b) const {
    return positionDistance(a,b)<=merge_distance && footprintOverlap(a.footprint,b.footprint)>=merge_overlap;
  }
  bool duplicateSamples(const Sample& a,const Sample& b) const {
    return (a.category==b.category && (a.footprint.empty() || b.footprint.empty()) &&
            distance(a,b)<duplicate_distance) || overlappingDuplicate(a,b);
  }
  void mergeTracks(double now) {
    bool merged=true;
    while (merged) {
      merged=false;
      for (auto a=tracks_.begin();a!=tracks_.end() && !merged;++a) {
        auto b=a; ++b;
        for (;b!=tracks_.end();++b) {
          if (now-a->second.last_seen>max_age || now-b->second.last_seen>max_age ||
              !duplicateSamples(a->second.sample,b->second.sample) ||
              (a->second.status==Status::RESERVED && b->second.status==Status::RESERVED)) continue;
          auto keep=protectionRank(b->second,now)>protectionRank(a->second,now) ? b:a;
          auto remove=keep==a ? b:a;
          auto& track=keep->second;
          const auto& other=remove->second;
          if (track.status==Status::RESERVED && (other.disturbed || protectionRank(other,now)>0)) {
            track.disturbed=true; track.disturbance_category=other.sample.category;
            track.disturbance_distance=distance(track.sample,other.sample);
          }
          if (protectionRank(track,now)==0) {
            const bool replace=other.sample.confidence>track.sample.confidence;
            const bool category_change=replace && other.sample.category!=track.sample.category;
            if (replace) { track.sample=other.sample; track.sampled_at=other.sampled_at; }
            track.observations=category_change ? 1:std::min(track.observations,other.observations);
            track.status=track.sample.eligible && track.observations>=min_observations ? Status::READY:Status::CANDIDATE;
            track.last_seen=std::max(track.last_seen,other.last_seen);
          }
          tracks_.erase(remove);
          ++merged_detections_total_;
          merged=true;
          break;
        }
        if (merged) break; // An erased 'a' iterator must not be incremented.
      }
    }
  }
  static double distance(const Sample& a, const Sample& b) { return std::hypot(a.x-b.x, a.y-b.y); }
  static double positionDistance(const Sample& a,const Sample& b) {
    const double xy=distance(a,b), dz=a.z-b.z;
    return std::sqrt(xy*xy+dz*dz);
  }
  std::map<std::uint64_t, Track> tracks_;
  std::uint64_t next_id_{1};
  std::uint64_t merged_detections_total_{0};
};
} // namespace aubo_sorting_core
#endif
