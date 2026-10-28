#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace explore {
// Map-space frontier identities. No robot heading or local sensor horizon.
struct RegionPoint { double x, y; };
struct GlobalRegion {
  std::uint64_t id, parent;
  std::vector<RegionPoint> evidence;
  bool covered{false};
  unsigned confirmations{0};
};
class GlobalRegions {
 public:
  std::vector<GlobalRegion> regions;
  std::uint64_t active{0};
  template<class IsKnown>
  std::vector<std::uint64_t> update(
      const std::vector<std::vector<RegionPoint>>& observations,
      std::uint64_t revision, IsKnown is_known) {
    const auto old_size = regions.size();
    const auto previous = regions;
    std::vector<bool> matched(old_size, false);
    std::vector<std::uint64_t> ids;
    for (const auto& points : observations) {
      double best = 0.60;
      size_t selected = old_size;
      for (size_t i = 0; i < old_size; ++i) {
        if (matched[i]) continue;
        for (const auto& a : previous[i].evidence) {
          for (const auto& b : points) {
            const double d = std::hypot(a.x-b.x,a.y-b.y);
            if (d < best) { best = d; selected = i; }
          }
        }
      }
      if (selected < old_size && !matched[selected]) {
        matched[selected] = true;
        auto& r = regions[selected];
        r.evidence = points;
        r.covered = false;
        r.confirmations = 0;
        ids.push_back(r.id);
      } else {
        // A frontier split supplies a parent relation, rather than an angle
        // measured at a moving robot. Other observations are independent roots.
        std::uint64_t parent = 0;
        double parent_distance = 0.60;
        for (size_t i=0;i<old_size;++i) {
          for(const auto& a:previous[i].evidence)for(const auto& b:points){
            double d=std::hypot(a.x-b.x,a.y-b.y);
            if(d<parent_distance){parent_distance=d;parent=previous[i].id;}
          }
        }
        regions.push_back(GlobalRegion{++next_id_, parent, points, false, 0});
        ids.push_back(next_id_);
      }
    }
    if (revision != last_revision_) {
      for (size_t i = 0; i < old_size; ++i) {
        auto& r = regions[i];
        if (matched[i] || r.covered || r.evidence.empty()) continue;
        const bool known = std::all_of(r.evidence.begin(), r.evidence.end(), is_known);
        r.confirmations = known ? r.confirmations + 1 : 0;
        if (r.confirmations >= 3) r.covered = true;
      }
    }
    last_revision_ = revision;
    return ids;
  }
  // Retain the active global region across short execution goals. A child is
  // preferred when an observed split exists. Costs are known-space route costs.
  std::uint64_t choose(const std::vector<std::pair<std::uint64_t,double>>& costs) {
    auto eligible = [&](std::uint64_t id) {
      for (const auto& c : costs) if (c.first == id && std::isfinite(c.second)) return true;
      return false;
    };
    auto parentOf = [&](std::uint64_t id) {
      for (const auto& r : regions) if (r.id == id) return r.parent;
      return std::uint64_t{0};
    };
    auto cheapest = [&](std::uint64_t parent, bool restrict_parent) {
      double best = std::numeric_limits<double>::infinity();
      std::uint64_t id = 0;
      for (const auto& c : costs) {
        if (restrict_parent && parentOf(c.first) != parent) continue;
        if (c.second < best) {best=c.second;id=c.first;}
      }
      return id;
    };
    if (active) {
      auto child = cheapest(active, true);
      if (child) return active = child;
      if (eligible(active)) return active;
      auto ancestor = parentOf(active);
      for (size_t guard=0; ancestor && guard<regions.size(); ++guard) {
        auto sibling = cheapest(ancestor, true);
        if (sibling) return active = sibling;
        ancestor = parentOf(ancestor);
      }
    }
    auto next = cheapest(0, false);
    if (next) active = next;
    return next;  // Zero means pending/blocked, never automatic completion.
  }
 private:
  std::uint64_t next_id_{0}, last_revision_{0};
};
}  // namespace explore
