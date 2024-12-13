#include "model.h"

#include <algorithm>
#include <ios>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace model {

using namespace std::literals;

unsigned Loot::loot_id_count = 0;
unsigned Dog::dog_id_count_ = 0;
unsigned GameSession::session_id_count_ = 0;

std::string GetDirectionForJson(Direction direction) {
  switch (direction) {
    case model::Direction::NORTH:
      return "U";
    case model::Direction::SOUTH:
      return "D";
    case model::Direction::WEST:
      return "L";
    case model::Direction::EAST:
      return "R";
    default:
      return "";
  }
}

void Road::CalculateHorizonalBoundary() {
  if (start_.x < end_.x) {
    boundary_.lower_west = {start_.x - HALF_ROAD_WIDTH,
                            start_.y - HALF_ROAD_WIDTH};
    boundary_.lower_east = {end_.x + HALF_ROAD_WIDTH, end_.y - HALF_ROAD_WIDTH};
    boundary_.upper_west = {start_.x - HALF_ROAD_WIDTH,
                            start_.y + HALF_ROAD_WIDTH};
    boundary_.upper_east = {end_.x + HALF_ROAD_WIDTH, end_.y + HALF_ROAD_WIDTH};
  } else if (start_.x > end_.x) {
    boundary_.lower_west = {end_.x - HALF_ROAD_WIDTH, end_.y - HALF_ROAD_WIDTH};
    boundary_.lower_east = {start_.x + HALF_ROAD_WIDTH,
                            start_.y - HALF_ROAD_WIDTH};
    boundary_.upper_west = {end_.x - HALF_ROAD_WIDTH, end_.y + HALF_ROAD_WIDTH};
    boundary_.upper_east = {start_.x + HALF_ROAD_WIDTH,
                            start_.y + HALF_ROAD_WIDTH};
  }
}

void Road::CalculateVerticalBoundary() {
  if (start_.y < end_.y) {
    boundary_.lower_west = {start_.x - HALF_ROAD_WIDTH,
                            start_.y - HALF_ROAD_WIDTH};
    boundary_.lower_east = {start_.x + HALF_ROAD_WIDTH,
                            start_.y - HALF_ROAD_WIDTH};
    boundary_.upper_west = {end_.x - HALF_ROAD_WIDTH, end_.y + HALF_ROAD_WIDTH};
    boundary_.upper_east = {end_.x + HALF_ROAD_WIDTH, end_.y + HALF_ROAD_WIDTH};
  } else if (start_.y > end_.y) {
    boundary_.lower_west = {end_.x - HALF_ROAD_WIDTH, end_.y - HALF_ROAD_WIDTH};
    boundary_.lower_east = {end_.x + HALF_ROAD_WIDTH, end_.y - HALF_ROAD_WIDTH};
    boundary_.upper_west = {start_.x - HALF_ROAD_WIDTH,
                            start_.y + HALF_ROAD_WIDTH};
    boundary_.upper_east = {start_.x + HALF_ROAD_WIDTH,
                            start_.y + HALF_ROAD_WIDTH};
  }
}

void Loot::SetLootPosition(const geom::Point2D& new_pos) { pos = new_pos; }

std::set<std::shared_ptr<Road>> Map::GetRoadsByCoords(Point point) const {
  if (point_to_road_ptr_.contains(point)) {
    return point_to_road_ptr_.at(point);
  }
  return {};
}

void Map::AddRoad(const Road& road) {
  roads_.emplace_back(road);
  std::shared_ptr<Road> road_ptr = std::make_shared<Road>(roads_.back());
  if (road.IsHorizontal()) {
    for (Coord x = std::min(road_ptr->GetStart().x, road_ptr->GetEnd().x);
         x <= std::max(road_ptr->GetStart().x, road_ptr->GetEnd().x); x++) {
      point_to_road_ptr_[{x, road_ptr->GetStart().y}].emplace(road_ptr);
    }
  }
  if (road.IsVertical()) {
    for (Coord y = std::min(road_ptr->GetStart().y, road_ptr->GetEnd().y);
         y <= std::max(road_ptr->GetStart().y, road_ptr->GetEnd().y); y++) {
      point_to_road_ptr_[{road_ptr->GetStart().x, y}].emplace(road_ptr);
    }
  }
}

void Map::AddOffice(Office office) {
  if (warehouse_id_to_index_.contains(office.GetId())) {
    throw std::invalid_argument("Duplicate warehouse");
  }
  const size_t index = offices_.size();
  Office& o = offices_.emplace_back(std::move(office));
  try {
    warehouse_id_to_index_.emplace(o.GetId(), index);
  } catch (const std::exception& e) {
    offices_.pop_back();
    std::cerr << "Exception occurred: " << e.what() << std::endl;
    throw;
  }
}

void Map::AddLoot(Loot map_loot) { map_loots_.push_back(std::move(map_loot)); }

const int Map::GetLootValue(int type) const noexcept {
  auto it = loot_type_to_value_.find(type);
  return it != loot_type_to_value_.end() ? it->second : 0;
}

unsigned Map::GenerateRandomType(unsigned loot_count) const {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<unsigned> dis(0, loot_count - 1);
  return dis(gen);
}

geom::Point2D Map::CalculateCoords(bool random) const {
  const auto& roads = GetRoads();
  if (random) {
    std::random_device rand_dev;
    std::mt19937 gen(rand_dev());
    std::uniform_int_distribution<> dis(0, static_cast<int>(roads.size() - 1));
    const auto& road = roads[dis(gen)];
    int min_x = std::min(road.GetStart().x, road.GetEnd().x);
    int max_x = std::max(road.GetStart().x, road.GetEnd().x);
    std::uniform_real_distribution<double> coord_dis(min_x, max_x);
    return geom::Point2D{coord_dis(gen),
                         static_cast<double>(road.GetStart().y)};
  }
  const auto& first_road = roads.front();
  return geom::Point2D({static_cast<double>(first_road.GetStart().x),
                        static_cast<double>(first_road.GetStart().y)});
}

void Dog::SetCoordinates(const geom::Point2D pos) {
  prev_coordinates_ = cur_coordinates_;
  cur_coordinates_ = pos;
}

void Dog::SetCoordinates(const Map& map, bool random) {
  if (map.IsRoadsEmpty()) {
    SetCoordinates({0.0, 0.0});
    return;
  }
  SetCoordinates(map.CalculateCoords(random));
}

void Dog::AddBagItem(std::shared_ptr<Loot> item) { bag_.push_back(item); }

std::shared_ptr<Loot> Dog::GetBagItem(int item_id) const noexcept {
  auto it = std::find_if(bag_.begin(), bag_.end(),
                         [item_id](const std::shared_ptr<Loot>& item) {
                           return item->id == item_id;
                         });
  return (it != bag_.end()) ? *it : nullptr;
}

size_t Dog::GetBagSize() const { return GetBag().size(); }

void Dog::ClearBag() { bag_.clear(); }

void Dog::IncreaseScore(std::shared_ptr<Map> map) {
  for (const auto& item : bag_) {
    int value = map->GetLootValue(item->type);
    score_ += value;
  }
}

void Dog::SetMotionSettings(std::pair<double, double> dog_speed,
                            double speed_on_map, const Direction& direction) {
  speed_.x = speed_on_map * dog_speed.first;
  speed_.y = speed_on_map * dog_speed.second;
  direction_ = direction;
}

std::optional<geom::Point2D> Dog::CheckIntersect(
    const Direction& direction, const Boundary& road_boundary,
    const geom::Point2D& dog_cur_pos, const geom::Point2D dog_end_pos) {
  if (direction == NORTH && road_boundary.lower_west.y > dog_end_pos.y) {
    return geom::Point2D{dog_cur_pos.x, road_boundary.lower_west.y};
  }
  if (direction == SOUTH && road_boundary.upper_west.y < dog_end_pos.y) {
    return geom::Point2D{dog_cur_pos.x, road_boundary.upper_west.y};
  }
  if (direction == WEST && road_boundary.lower_west.x > dog_end_pos.x) {
    return geom::Point2D{road_boundary.lower_west.x, dog_cur_pos.y};
  }
  if (direction == EAST && road_boundary.lower_east.x < dog_end_pos.x) {
    return geom::Point2D{road_boundary.lower_east.x, dog_cur_pos.y};
  }
  return std::nullopt;
}

void Dog::SetTick(uint64_t time_delta, const Map& map) {
  UpdateActiveTime(time_delta);
  const geom::Point2D dog_cur_pos{GetCurCoordinates()};
  const geom::Vec2D dog_start_speed{GetSpeed()};
  const Direction dog_direction = GetDirection();
  Point dog_cur_point = {static_cast<Coord>(std::round(dog_cur_pos.x)),
                         static_cast<Coord>(std::round(dog_cur_pos.y))};
  std::set<std::shared_ptr<Road>> roads_on_point =
      map.GetRoadsByCoords(dog_cur_point);
  geom::Point2D dog_end_pos{CalculateNewCoordinates(time_delta)};
  std::set<geom::Point2D, geom::Point2DComparator> intersection_set;
  for (const auto& road : roads_on_point) {
    const auto& road_boundary = road->GetBoundary();
    std::optional<geom::Point2D> intersection_opt =
        CheckIntersect(dog_direction, road_boundary, dog_cur_pos, dog_end_pos);
    if (intersection_opt.has_value()) {
      intersection_set.insert(intersection_opt.value());
      SetCoordinates(*intersection_set.rbegin());
      StopDog();
      continue;
    } else {
      SetCoordinates(dog_end_pos);
      SetSpeed(dog_start_speed);
      break;
    }
  }
}

geom::Point2D Dog::CalculateNewCoordinates(uint64_t time_delta) {
  double time = static_cast<double>(time_delta / MS_IN_SECOND);
  double new_x = cur_coordinates_.x + speed_.x * time;
  double new_y = cur_coordinates_.y + speed_.y * time;
  return geom::Point2D(new_x, new_y);
}

void GameSession::AddDogAndGeneratePos(std::shared_ptr<Dog> dog, bool random) {
  dog->SetCoordinates(GetMap(), random);
  dogs_.emplace_back(dog);
}

void GameSession::AddDog(std::shared_ptr<Dog> dog) {
  dogs_.push_back(std::move(dog));
}

void GameSession::RemoveInactiveDogs() {
  for (auto& dog : inactive_dogs_) {
    dog->PushInactiveSignal();
    dogs_.erase(std::remove(dogs_.begin(), dogs_.end(), dog), dogs_.end());
  }
  inactive_dogs_.clear();
}

std::shared_ptr<Dog> GameSession::FindDogById(int dog_id) {
  auto it = std::find_if(dogs_.begin(), dogs_.end(),
                         [dog_id](const std::shared_ptr<Dog>& dog) {
                           return dog->GetId().GetValue() == dog_id;
                         });
  return (it != dogs_.end()) ? *it : nullptr;
}

void GameSession::AddLoot(Loot loot) {
  loots_.push_back(std::make_shared<Loot>(loot));
}

bool GameSession::IsBagFull(const std::shared_ptr<Dog> dog) const {
  return dog->GetBagSize() >= map_ptr_->GetBagCapacity();
}

void GameSession::SetDogsTick(uint64_t time_delta, const Map& map,
                              std::shared_ptr<database::Database> db) {
  for (auto& dog_ptr : dogs_) {
    dog_ptr->SetTick(time_delta, map);
    if (dog_ptr->IsDogStopped()) {
      dog_ptr->UpdateInactiveTime(time_delta);
      if (map.GetDogRetirementTime() <= dog_ptr->GetInactiveTime()) {
        db->AddPlayerRecord(dog_ptr->GetName(), dog_ptr->GetScore(),
                            (dog_ptr->GetActiveTime()));
        inactive_dogs_.emplace_back(dog_ptr);
      }
    }
  }
}

void GameSession::SetLootsTick(uint64_t time_delta, const Map& map) {
  std::chrono::milliseconds time_delta_ms(time_delta);
  unsigned dogs_count = static_cast<unsigned>(GetDogs().size());
  unsigned loots_count = static_cast<unsigned>(GetLoots().size());
  unsigned gen_count =
      loot_gen_->Generate(time_delta_ms, loots_count, dogs_count);
  for (unsigned i = 0; i < gen_count; ++i) {
    unsigned random_loot_type = map.GenerateRandomType(map.GetLootCount());
    geom::Point2D random_loots_pos = map.CalculateCoords(true);
    Loot loot{random_loot_type, random_loots_pos};
    AddLoot(loot);
    AddLootToCollisionObj(random_loots_pos, GetLoots().back());
  }
}

void GameSession::FindCollision() {
  std::vector<collision_detector::GatheringEvent> events =
      collision_detector::FindGatherEvents(*this);
  if (events.empty()) {
    return;
  }
  for (const auto& event : events) {
    auto dog = FindDogByIdx(event.gatherer_id);
    auto& coll_obj = FindCollisionObjByIdx(event.item_id);
    if (coll_obj.is_base) {
      dog->IncreaseScore(map_ptr_);
      dog->ClearBag();
      continue;
    }
    if (!IsBagFull(dog)) {
      if (!coll_obj.is_collected) {
        dog->AddBagItem(coll_obj.loot_ptr);
        coll_obj.is_collected = true;
        continue;
      }
    } else {
      continue;
    }
  }
}

void GameSession::SetTick(uint64_t time_delta,
                          std::shared_ptr<database::Database> db) {
  const Map& map = GetMap();
  SetDogsTick(time_delta, map, db);
  SetLootsTick(time_delta, map);
  RemoveInactiveDogs();
  FindCollision();
}

size_t GameSession::ItemsCount() const { return collision_objects_.size(); }

collision_detector::Item GameSession::GetItem(size_t idx) const {
  return collision_objects_[idx].item;
}

size_t GameSession::GatherersCount() const { return dogs_.size(); }

collision_detector::Gatherer GameSession::GetGatherer(size_t idx) const {
  const std::shared_ptr<Dog> dog = dogs_[idx];
  geom::Point2D start_pos{dog->GetPrevCoordinates().x,
                          dog->GetPrevCoordinates().y};
  geom::Point2D end_pos{dog->GetCurCoordinates().x, dog->GetCurCoordinates().y};
  double width = DOG_WIDTH;
  return {start_pos, end_pos, width};
}

void GameSession::AddBasesToCollisionObj() {
  const Map::Offices& offices_on_map = map_ptr_->GetOffices();
  for (const auto& office : offices_on_map) {
    geom::Point2D position{static_cast<double>(office.GetPosition().x),
                           static_cast<double>(office.GetPosition().y)};
    double width = OFFICE_WIDTH;
    CollisionObject collision_obj{true, false,
                                  collision_detector::Item{position, width}};
    collision_objects_.push_back(std::move(collision_obj));
  }
}

void GameSession::AddLootToCollisionObj(geom::Point2D pos,
                                        std::shared_ptr<Loot> loot_sptr) {
  geom::Point2D position{pos.x, pos.y};
  double width = ITEM_WIDTH;
  CollisionObject collision_obj{
      false, false, collision_detector::Item{position, width}, loot_sptr};
  collision_objects_.push_back(std::move(collision_obj));
}

double CustomRandomGenerator() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<> dis(0.0, 1.0);
  return dis(gen);
}

void Game::AddMap(Map map) {
  const size_t index = maps_.size();
  if (auto [it, inserted] = map_id_to_index_.emplace(map.GetId(), index);
      !inserted) {
    throw std::invalid_argument("Map with id "s + *map.GetId() +
                                " already exists"s);
  } else {
    try {
      maps_.emplace_back(std::move(std::make_shared<Map>(map)));
    } catch (const std::exception& e) {
      map_id_to_index_.erase(it);
      std::cerr << "Exception caught: " << e.what() << std::endl;
      throw;
    }
  }
}

std::shared_ptr<Map> Game::FindMap(const Map::Id& id) const noexcept {
  if (auto it = map_id_to_index_.find(id); it != map_id_to_index_.end()) {
    return maps_.at(it->second);
  }
  return nullptr;
}

bool Game::CheckMapAvailability(const Map::Id& id) const noexcept {
  return map_id_to_index_.find(id) != map_id_to_index_.end();
}

void Game::AddGameSession(std::shared_ptr<GameSession> game_session) {
  const size_t index = game_sessions_.size();
  const model::Map::Id& map_id = game_session->GetMapId();
  if (auto [it, inserted] = game_session_id_to_index_.emplace(
          game_session->GetSessionId(), index);
      !inserted) {
    throw std::invalid_argument(
        "GameSession with id "s +
        std::to_string(game_session->GetSessionId().GetValue()) +
        " already exists"s);
  } else {
    try {
      game_sessions_.emplace_back(game_session);
      game_session_map_id_to_index_.emplace(map_id, index);
    } catch (const std::exception& e) {
      game_session_id_to_index_.erase(it);
      std::cerr << "Exception caught: " << e.what() << std::endl;
      throw;
    }
  }
}

std::shared_ptr<GameSession> Game::CreateGameSession(const Map::Id& map_id) {
  std::shared_ptr<GameSession> game_session =
      std::make_shared<GameSession>(FindMap(map_id), loot_generator_);
  AddGameSession(std::move(game_session));
  return FindGameSessionByMapId(map_id);
}

std::shared_ptr<GameSession> Game::FindGameSessionById(
    const GameSession::Id& id) noexcept {
  if (auto it = game_session_id_to_index_.find(id);
      it != game_session_id_to_index_.end()) {
    return game_sessions_.at(it->second);
  }
  return nullptr;
}

std::shared_ptr<GameSession> Game::FindGameSessionByMapId(
    const Map::Id& map_id) noexcept {
  if (auto it = game_session_map_id_to_index_.find(map_id);
      it != game_session_map_id_to_index_.end()) {
    return game_sessions_.at(it->second);
  }
  return nullptr;
}

std::shared_ptr<GameSession> Game::ConnectToGameSession(
    const Map::Id& map_id, std::shared_ptr<Dog> dog_ptr, bool random) {
  auto game_session = FindGameSessionByMapId(map_id);
  if (!game_session) {
    game_session = CreateGameSession(map_id);
  }
  game_session->AddDogAndGeneratePos(dog_ptr, random);
  return game_session;
}

void Game::SetServerTick(uint64_t time_delta) {
  for (const auto& session : game_sessions_) {
    session->SetTick(time_delta, db_);
  }
}

void Game::SetLootGen(std::chrono::milliseconds period, double probability) {
  loot_generator_ =
      std::make_shared<loot_gen::LootGenerator>(period, probability);
}

unsigned Game::GenerateLoot(std::chrono::milliseconds time_delta,
                            unsigned loot_count, unsigned looter_count) {
  return loot_generator_->Generate(time_delta, loot_count, looter_count);
}

}  // namespace model