#pragma once

#include <algorithm>
#include <vector>

#include "geom.h"

namespace collision_detector {

struct CollectionResult {
  bool IsCollected(double collect_radius) const;

  // квадрат расстояния до точки
  double sq_distance;

  // доля пройденного отрезка
  double proj_ratio;
};

CollectionResult TryCollectPoint(geom::Point2D a, geom::Point2D b,
                                 geom::Point2D c);

struct Item {
  geom::Point2D position;
  double width;
};

struct Gatherer {
  geom::Point2D start_pos;
  geom::Point2D end_pos;
  double width;
};

class ItemGathererProvider {
 protected:
  ~ItemGathererProvider() = default;

 public:
  virtual size_t ItemsCount() const = 0;
  virtual Item GetItem(size_t idx) const = 0;
  virtual size_t GatherersCount() const = 0;
  virtual Gatherer GetGatherer(size_t idx) const = 0;
};

struct GatheringEvent {
  size_t item_id;
  size_t gatherer_id;
  double sq_distance;
  double time;
};

std::vector<GatheringEvent> FindGatherEvents(
    const ItemGathererProvider& provider);

class Provider : public collision_detector::ItemGathererProvider {
 public:
  Provider(const std::vector<collision_detector::Item>& items,
           const std::vector<collision_detector::Gatherer>& gatherers)
      : items(items), gatherers(gatherers) {}

  size_t ItemsCount() const override;
  collision_detector::Item GetItem(size_t idx) const override;
  size_t GatherersCount() const override;
  collision_detector::Gatherer GetGatherer(size_t idx) const override;

 private:
  std::vector<collision_detector::Item> items;
  std::vector<collision_detector::Gatherer> gatherers;
};

}  // namespace collision_detector