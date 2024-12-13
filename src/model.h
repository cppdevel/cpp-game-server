#pragma once

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/function.hpp>
#include <boost/signals2/signal.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "collision_detector.h"
#include "database.h"
#include "loot_generator.h"
#include "tagged.h"

namespace net = boost::asio;

namespace model {

constexpr uint64_t MS_IN_SECOND = 1000;

const double DOG_WIDTH = 0.6;
const double ITEM_WIDTH = 0.0;
const double OFFICE_WIDTH = 0.5;
const double ROAD_WIDTH = 0.8;
const double HALF_ROAD_WIDTH = 0.4;

using Dimension = int;
using Coord = Dimension;

struct Point {
  Coord x, y;

  bool operator==(const Point& other) const {
    return (x == other.x && y == other.y);
  }
};

struct Size {
  Dimension width, height;
};

struct Rectangle {
  Point position;
  Size size;
};

struct Offset {
  Dimension dx, dy;
};

enum Direction { NORTH, SOUTH, WEST, EAST };

std::string GetDirectionForJson(Direction direction);

enum RoadType { HORIZONTAL, VERTICAL };

struct Boundary {
  geom::Point2D lower_west{0, 0};
  geom::Point2D lower_east{0, 0};
  geom::Point2D upper_west{0, 0};
  geom::Point2D upper_east{0, 0};
};

class Road {
  struct HorizontalTag {
    HorizontalTag() = default;
  };

  struct VerticalTag {
    VerticalTag() = default;
  };

 public:
  constexpr static HorizontalTag HORIZONTAL{};
  constexpr static VerticalTag VERTICAL{};

  Road(HorizontalTag, Point start, Coord end_x) noexcept
      : start_{start}, end_{end_x, start.y}, type_{RoadType::HORIZONTAL} {
    CalculateHorizonalBoundary();
  }

  Road(VerticalTag, Point start, Coord end_y) noexcept
      : start_{start}, end_{start.x, end_y}, type_{RoadType::VERTICAL} {
    CalculateVerticalBoundary();
  }

  bool IsHorizontal() const noexcept { return start_.y == end_.y; }

  bool IsVertical() const noexcept { return start_.x == end_.x; }

  Point GetStart() const noexcept { return start_; }

  Point GetEnd() const noexcept { return end_; }

  const Boundary& GetBoundary() const noexcept { return boundary_; }

 private:
  void CalculateHorizonalBoundary();
  void CalculateVerticalBoundary();

  Point start_;
  Point end_;
  RoadType type_;
  Boundary boundary_;
};

class Building {
 public:
  explicit Building(Rectangle bounds) noexcept : bounds_{bounds} {}

  const Rectangle& GetBounds() const noexcept { return bounds_; }

 private:
  Rectangle bounds_;
};

class Office {
 public:
  using Id = util::Tagged<std::string, Office>;

  Office(Id id, Point position, Offset offset) noexcept
      : id_{std::move(id)}, position_{position}, offset_{offset} {}

  const Id& GetId() const noexcept { return id_; }

  Point GetPosition() const noexcept { return position_; }

  Offset GetOffset() const noexcept { return offset_; }

 private:
  Id id_;
  Point position_;
  Offset offset_;
};

struct Loot {
  explicit Loot(unsigned type_count) : id(loot_id_count++), type(type_count) {}

  explicit Loot(unsigned type_count, geom::Point2D new_pos)
      : id(loot_id_count++), type(type_count), pos(new_pos) {}

  explicit Loot(unsigned loot_id, unsigned loot_type, geom::Point2D position)
      : id(loot_id), type(loot_type), pos(position) {}

  void SetLootPosition(const geom::Point2D& new_pos);

  static void SetIdCount(unsigned id_count) { loot_id_count = id_count; }

  unsigned id = 0, type = 0;
  geom::Point2D pos;
  static unsigned loot_id_count;
};

struct CollisionObject {
  bool is_base = false;
  bool is_collected = false;
  collision_detector::Item item;
  std::shared_ptr<Loot> loot_ptr = nullptr;
};

using Loots = std::vector<std::shared_ptr<Loot>>;

class Map {
 public:
  using Id = util::Tagged<std::string, Map>;
  using Roads = std::vector<Road>;
  using Buildings = std::vector<Building>;
  using Offices = std::vector<Office>;
  using LootTypeToValue = std::unordered_map<int, int>;

  Map(Id id, std::string name) noexcept
      : id_(std::move(id)), name_(std::move(name)) {}

  const Id& GetId() const noexcept { return id_; }

  const std::string& GetName() const noexcept { return name_; }

  const Buildings& GetBuildings() const noexcept { return buildings_; }

  const Roads& GetRoads() const noexcept { return roads_; }

  bool IsRoadsEmpty() const noexcept { return roads_.empty(); }

  std::set<std::shared_ptr<Road>> GetRoadsByCoords(Point pos) const;

  const Offices& GetOffices() const noexcept { return offices_; }

  double GetDogSpeed() const noexcept { return dog_speed_; }

  void AddRoad(const Road& road);

  void AddBuilding(const Building& building) {
    buildings_.emplace_back(building);
  }

  void AddOffice(Office office);

  void SetDogSpeed(double dog_speed) { dog_speed_ = dog_speed; }

  void SetLootCount(unsigned int loot_count) { loot_count_ = loot_count; }

  unsigned GetLootCount() const noexcept { return loot_count_; }

  void AddLoot(Loot map_loot);

  std::vector<Loot> GetLoots() const noexcept { return map_loots_; }

  void SetBagCapacity(int bag_capacity) { bag_capacity_ = bag_capacity; }

  int GetBagCapacity() const noexcept { return bag_capacity_; }

  void AddLootValue(int type, int value) {
    loot_type_to_value_.insert({type, value});
  }

  const int GetLootValue(int type) const noexcept;
  unsigned GenerateRandomType(unsigned loot_count) const;
  geom::Point2D CalculateCoords(bool random) const;

  void SetDogRetirementTime(double dog_retirement_time) {
    dog_retirement_time_ms_ =
        static_cast<uint64_t>(dog_retirement_time * MS_IN_SECOND);
  }

  const uint64_t GetDogRetirementTime() const noexcept {
    return dog_retirement_time_ms_;
  }

 private:
  struct PointHash {
    std::size_t operator()(const Point& p) const {
      return std::hash<Coord>()(p.x) ^ (std::hash<Coord>()(p.y) << 1);
    }
  };

  using OfficeIdToIndex =
      std::unordered_map<Office::Id, size_t, util::TaggedHasher<Office::Id>>;
  using PointToRoadPtr =
      std::unordered_map<Point, std::set<std::shared_ptr<Road>>, PointHash>;

  Id id_;
  std::string name_;
  Roads roads_;
  PointToRoadPtr point_to_road_ptr_;
  Buildings buildings_;
  std::vector<Loot> map_loots_;
  int bag_capacity_ = 0;

  OfficeIdToIndex warehouse_id_to_index_;
  Offices offices_;

  double dog_speed_ = 0.0;
  unsigned loot_count_ = 0;
  LootTypeToValue loot_type_to_value_;
  uint64_t dog_retirement_time_ms_ = 0;
};

class Dog : public std::enable_shared_from_this<Dog> {
 public:
  using Id = util::Tagged<int, Dog>;
  using DogInactiveSignal =
      boost::signals2::signal<void(const std::shared_ptr<Dog>&)>;

  explicit Dog(std::string name)
      : id_(dog_id_count_++), name_(std::move(name)) {}

  const Id& GetId() const noexcept { return id_; }

  std::string DogIdToString() const { return std::to_string(id_.GetValue()); }

  const std::string& GetName() const noexcept { return name_; }

  const geom::Point2D& GetCurCoordinates() const noexcept {
    return cur_coordinates_;
  }

  const geom::Point2D& GetPrevCoordinates() const noexcept {
    return prev_coordinates_;
  }

  void SetCoordinates(const geom::Point2D pos);
  void SetCoordinates(const Map& map, bool random);

  const geom::Vec2D& GetSpeed() const noexcept { return speed_; }

  void SetSpeed(const geom::Vec2D& speed) { speed_ = speed; }

  void StopDog() { SetSpeed({0.0, 0.0}); }

  bool IsDogStopped() const noexcept {
    if (speed_.x == 0.0 && speed_.y == 0.0) {
      return true;
    }
    return false;
  }

  Direction GetDirection() const noexcept { return direction_; }

  void AddBagItem(std::shared_ptr<Loot> item);
  std::shared_ptr<Loot> GetBagItem(int item_id) const noexcept;

  Loots GetBag() const noexcept { return bag_; }

  size_t GetBagSize() const;
  void ClearBag();
  void IncreaseScore(std::shared_ptr<Map> map);

  int GetScore() const noexcept { return score_; }

  void SetMotionSettings(std::pair<double, double> dog_speed,
                         double speed_on_map,
                         const model::Direction& direction);
  std::optional<geom::Point2D> CheckIntersect(const Direction& direction,
                                              const Boundary& road_boundary,
                                              const geom::Point2D& dog_cur_pos,
                                              const geom::Point2D dog_end_pos);
  void SetTick(uint64_t time_delta, const Map& map);

  explicit Dog(Id id, std::string name) : id_(id), name_(name) {}

  void SetDirection(const Direction direction) { direction_ = direction; }

  void AddScore(unsigned score) noexcept { score_ += score; }

  void SetCurCoordinates(geom::Point2D cur_coords) noexcept {
    cur_coordinates_ = cur_coords;
  }

  void SetPrevCoordinates(geom::Point2D prev_coords) noexcept {
    prev_coordinates_ = prev_coords;
  }

  static unsigned GetIdCount() noexcept { return dog_id_count_; }

  static void SetIdCount(unsigned id_count) { dog_id_count_ = id_count; }

  uint64_t GetInactiveTime() const noexcept { return inactive_time_ms_; }

  void UpdateInactiveTime(uint64_t inactive_time_ms) {
    inactive_time_ms_ += inactive_time_ms;
  }

  void ResetInactiveTime() { inactive_time_ms_ = 0; }

  uint64_t GetActiveTime() const noexcept { return active_time_ms_; }

  void UpdateActiveTime(uint64_t active_time_ms) {
    active_time_ms_ += active_time_ms;
  }

  void SetActivity(bool is_active) {
    is_active_ = is_active;
    if (is_active_) {
      ResetInactiveTime();
    }
  }

  void PushInactiveSignal() { on_inactive_signal_(shared_from_this()); }

  DogInactiveSignal& OnInactive() { return on_inactive_signal_; }

 private:
  geom::Point2D CalculateNewCoordinates(uint64_t time_delta);

  static unsigned dog_id_count_;
  Id id_;
  std::string name_;
  geom::Point2D cur_coordinates_{0.0, 0.0};
  geom::Point2D prev_coordinates_{0.0, 0.0};
  geom::Vec2D speed_{0.0, 0.0};
  Direction direction_{Direction::NORTH};
  Loots bag_;
  unsigned score_ = 0;
  uint64_t inactive_time_ms_ = 0;
  uint64_t active_time_ms_ = 0;
  bool is_active_ = true;
  DogInactiveSignal on_inactive_signal_;
};

class GameSession : public collision_detector::ItemGathererProvider {
 public:
  using Id = util::Tagged<unsigned, GameSession>;
  using Dogs = std::vector<std::shared_ptr<Dog>>;

  explicit GameSession(std::shared_ptr<Map> map_sptr,
                       std::shared_ptr<loot_gen::LootGenerator> loot_gen)
      : session_id_(session_id_count_++),
        map_ptr_(map_sptr),
        loot_gen_(loot_gen) {
    AddBasesToCollisionObj();
  }

  explicit GameSession(unsigned id, std::shared_ptr<Map> map_sptr,
                       std::shared_ptr<loot_gen::LootGenerator> loot_gen)
      : session_id_(id), map_ptr_(map_sptr), loot_gen_(loot_gen) {
    AddBasesToCollisionObj();
  }

  const Id GetSessionId() const noexcept { return session_id_; }

  void AddDogAndGeneratePos(std::shared_ptr<Dog> dog_sptr, bool random);
  void AddDog(std::shared_ptr<Dog> dog_sptr);
  void RemoveInactiveDogs();

  const Dogs& GetDogs() const noexcept { return dogs_; }

  std::shared_ptr<Dog> FindDogById(int dog_id);

  std::shared_ptr<Dog> GetLastAddedDog() const { return dogs_.back(); }

  const Map& GetMap() const { return *map_ptr_; }

  const Map::Id& GetMapId() const noexcept { return map_ptr_->GetId(); }

  void AddLoot(Loot loot_sptr);

  const Loots GetLoots() const noexcept { return loots_; }

  std::shared_ptr<Dog> FindDogByIdx(size_t idx) { return dogs_.at(idx); }

  CollisionObject& FindCollisionObjByIdx(size_t idx) {
    return collision_objects_.at(idx);
  }

  void SetDogsTick(uint64_t time_delta, const Map& map,
                   std::shared_ptr<database::Database> db);
  void SetLootsTick(uint64_t time_delta, const Map& map);
  void FindCollision();
  void SetTick(uint64_t time_delta, std::shared_ptr<database::Database> db);

  size_t ItemsCount() const override;
  collision_detector::Item GetItem(size_t idx) const override;
  size_t GatherersCount() const override;
  collision_detector::Gatherer GetGatherer(size_t idx) const override;

  void AddLootToCollisionObj(geom::Point2D pos,
                             std::shared_ptr<Loot> loot_sptr);

  static unsigned GetIdCount() noexcept { return session_id_count_; }

  static void SetIdCount(unsigned id_count) { session_id_count_ = id_count; }

 private:
  bool IsBagFull(const std::shared_ptr<Dog> dog) const;
  void AddBasesToCollisionObj();

  static unsigned session_id_count_;
  Id session_id_;
  std::shared_ptr<Map> map_ptr_;
  Loots loots_;
  std::vector<CollisionObject> collision_objects_;
  Dogs dogs_;
  Dogs inactive_dogs_;
  std::shared_ptr<loot_gen::LootGenerator> loot_gen_;
};

double CustomRandomGenerator();

class Game {
 public:
  using Maps = std::vector<std::shared_ptr<Map>>;
  using Sessions = std::vector<std::shared_ptr<GameSession>>;

  void AddMap(Map map);

  Maps GetMaps() const noexcept { return maps_; }

  std::shared_ptr<Map> FindMap(const Map::Id& id) const noexcept;
  bool CheckMapAvailability(const Map::Id& id) const noexcept;

  void AddGameSession(std::shared_ptr<GameSession> game_session);
  std::shared_ptr<GameSession> CreateGameSession(const Map::Id& map_id);

  const Sessions& GetGameSessions() const noexcept { return game_sessions_; }

  std::shared_ptr<GameSession> FindGameSessionById(
      const GameSession::Id& id) noexcept;
  std::shared_ptr<GameSession> FindGameSessionByMapId(
      const Map::Id& id) noexcept;
  std::shared_ptr<GameSession> ConnectToGameSession(
      const Map::Id& map_id, std::shared_ptr<Dog> dog_ptr, bool random);

  void SetServerTick(uint64_t time_delta);

  void SetLootGen(std::chrono::milliseconds period, double probability);

  std::shared_ptr<loot_gen::LootGenerator> GetLootGen() const noexcept {
    return loot_generator_;
  }

  unsigned GenerateLoot(std::chrono::milliseconds time_delta,
                        unsigned loot_count, unsigned looter_count);

  void SetDB(std::shared_ptr<database::Database> db) { db_ = db; }

  std::shared_ptr<database::Database> GetDB() const noexcept { return db_; }

 private:
  using MapIdHasher = util::TaggedHasher<Map::Id>;
  using MapIdToIndex = std::unordered_map<Map::Id, size_t, MapIdHasher>;
  Maps maps_;
  MapIdToIndex map_id_to_index_;

  using GameSessionMapIdToIndex =
      std::unordered_map<Map::Id, size_t, MapIdHasher>;
  GameSessionMapIdToIndex game_session_map_id_to_index_;

  using GameSessionIdHasher = util::TaggedHasher<GameSession::Id>;
  using GameSessionIdToIndex =
      std::unordered_map<GameSession::Id, size_t, GameSessionIdHasher>;
  GameSessionIdToIndex game_session_id_to_index_;
  Sessions game_sessions_;

  std::shared_ptr<loot_gen::LootGenerator> loot_generator_ = nullptr;

  std::shared_ptr<database::Database> db_ = nullptr;
};

}  // namespace model