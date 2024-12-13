#include "application.h"

#include <algorithm>

namespace app {

using namespace constant_literals;

std::string BuildError(const std::string& code, const std::string& message) {
  json::object response;
  response[code_key] = code;
  response[message_key] = message;
  return json::serialize(response);
}

Player::Player(Token token, std::shared_ptr<model::Dog> dog,
               std::shared_ptr<model::GameSession> session) noexcept
    : token_(std::move(token)), dog_(dog), session_(session) {}

const Player::Token& Player::GetToken() const noexcept { return token_; }

const model::GameSession& Player::GetSession() const noexcept {
  return *session_;
}

std::shared_ptr<model::Dog> Player::GetDog() const noexcept { return dog_; }

model::Dog& Player::GetDogNonConstant() { return *dog_; }

model::Dog::Id Player::GetId() { return dog_->GetId(); }

std::shared_ptr<Player> Players::AddPlayer(
    std::shared_ptr<model::Dog> dog,
    std::shared_ptr<model::GameSession> session) {
  Player::Token token = GenerateToken();
  std::shared_ptr<Player> player =
      std::make_shared<Player>(token, dog, session);
  players_.emplace_back(player);
  try {
    token_to_player_.emplace(token, player);
    player->GetDog()->OnInactive().connect(
        [this, player](const std::shared_ptr<model::Dog>& inactive_dog) {
          RemovePlayer(player);
        });
    return player;
  } catch (const std::exception& e) {
    players_.pop_back();
    std::cerr << "Exception caught: " << e.what() << std::endl;
    throw;
  }
  return nullptr;
}

void Players::RemovePlayer(const std::shared_ptr<Player> player_sptr) {
  auto it = std::remove(players_.begin(), players_.end(), player_sptr);
  if (it != players_.end()) {
    players_.erase(it, players_.end());
  }
  auto token_it = token_to_player_.find(player_sptr->GetToken());
  if (token_it != token_to_player_.end()) {
    token_to_player_.erase(token_it);
  }
}

std::shared_ptr<Player> Players::FindByDogIdAndMapId(
    const model::Dog::Id& dog_id, const model::Map::Id& map_id) {
  auto it =
      std::find_if(players_.begin(), players_.end(), [&](const auto& player) {
        return player->GetDog()->GetId() == dog_id &&
               player->GetSession().GetMapId() == map_id;
      });
  return it != players_.end() ? *it : nullptr;
}

std::shared_ptr<Player> Players::FindPlayerByToken(const Player::Token& token) {
  auto it = token_to_player_.find(token);
  return it != token_to_player_.end() ? it->second : nullptr;
}

std::vector<std::shared_ptr<Player>> Players::GetPlayers() const noexcept {
  return players_;
}

void Players::AddRepresPlayer(std::shared_ptr<app::Player> player_sptr) {
  players_.emplace_back(player_sptr);
  token_to_player_.emplace(player_sptr->GetToken(), player_sptr);
}

Player::Token Players::GenerateToken() {
  std::stringstream stream;
  if (!stream) {
    throw std::runtime_error("Failed to create std::stringstream");
  }
  stream << std::hex << std::setw(16) << std::setfill('0') << generator1_();
  stream << std::hex << std::setw(16) << std::setfill('0') << generator2_();
  return Player::Token(stream.str());
}

GetMapUseCase::GetMapUseCase(model::Game& game) : game_{game} {}

json::object GetMapUseCase::GetSerializedMap(
    std::shared_ptr<model::Map> map_ptr) {
  return BuildTheMap(*map_ptr);
}

json::array GetMapUseCase::LoadRoads(const model::Map& map) {
  json::array roads;
  for (const auto& road : map.GetRoads()) {
    json::object road_object;
    road_object[x0_key] = road.GetStart().x;
    road_object[y0_key] = road.GetStart().y;
    if (road.IsHorizontal()) {
      road_object[x1_key] = road.GetEnd().x;
    } else if (road.IsVertical()) {
      road_object[y1_key] = road.GetEnd().y;
    }
    roads.emplace_back(std::move(road_object));
  }
  return roads;
}

json::array GetMapUseCase::LoadBuildings(const model::Map& map) {
  json::array buildings;
  for (const auto& building : map.GetBuildings()) {
    json::object building_object;
    building_object[x_key] = building.GetBounds().position.x;
    building_object[y_key] = building.GetBounds().position.y;
    building_object[w_key] = building.GetBounds().size.width;
    building_object[h_key] = building.GetBounds().size.height;
    buildings.emplace_back(std::move(building_object));
  }
  return buildings;
}

json::array GetMapUseCase::LoadOffices(const model::Map& map) {
  json::array offices;
  for (const auto& office : map.GetOffices()) {
    json::object office_object;
    office_object[id_key] = *office.GetId();
    office_object[x_key] = office.GetPosition().x;
    office_object[y_key] = office.GetPosition().y;
    office_object[offset_x_key] = office.GetOffset().dx;
    office_object[offset_y_key] = office.GetOffset().dy;
    offices.emplace_back(std::move(office_object));
  }
  return offices;
}

json::object GetMapUseCase::BuildTheMap(const model::Map& map) {
  json::object response;
  response[id_key] = *map.GetId();
  response[name_key] = map.GetName();
  response[roads_key] = LoadRoads(map);
  response[buildings_key] = LoadBuildings(map);
  response[offices_key] = LoadOffices(map);
  return response;
  // return json::serialize(response);
}

JoinGameError::JoinGameError(JoinGameErrorReason reason) : reason_{reason} {}

const JoinGameErrorReason JoinGameError::GetReason() const noexcept {
  return reason_;
}

JoinGameResult::JoinGameResult(const Player::Token& token,
                               const model::Dog::Id& dog_id)
    : token_{token}, player_id_{dog_id} {}

const Player::Token& JoinGameResult::GetToken() const noexcept {
  return token_;
}

JoinGameUseCase::JoinGameUseCase(model::Game& game, Players& players)
    : game_{game}, players_{players} {}

const model::Dog::Id& JoinGameResult::GetPlayerId() const noexcept {
  return player_id_;
}

JoinGameResult JoinGameUseCase::Join(const model::Map::Id& map_id,
                                     std::string name, bool random) {
  if (name.empty()) {
    throw JoinGameError{JoinGameErrorReason::InvalidName};
  }
  if (!game_.CheckMapAvailability(model::Map::Id(map_id))) {
    throw JoinGameError{JoinGameErrorReason::InvalidMap};
  }
  std::shared_ptr<model::GameSession> session = game_.ConnectToGameSession(
      model::Map::Id(map_id),
      std::make_shared<model::Dog>(model::Dog(name.data())), random);
  std::shared_ptr<Player> new_player_ptr =
      players_.AddPlayer(session->GetLastAddedDog(), session);
  const Player::Token& new_player_token = new_player_ptr->GetToken();
  const model::Dog::Id& player_id =
      players_.FindPlayerByToken(new_player_token)->GetDog()->GetId();
  return JoinGameResult(new_player_token, player_id);
}

ListMapsUseCase::ListMapsUseCase(model::Game& game) : game_{game} {}

std::string ListMapsUseCase::GetSerializedMaps() {
  return BuildMaps(game_.GetMaps());
}

std::string ListMapsUseCase::BuildMaps(
    std::vector<std::shared_ptr<model::Map>> maps) {
  json::array response;
  for (const auto& map : maps) {
    json::object map_object;
    map_object[id_key] = *map->GetId();
    map_object[name_key] = map->GetName();
    response.push_back(std::move(map_object));
  }
  return json::serialize(response);
}

ListPlayersUseCase::ListPlayersUseCase(model::Game& game, Players& players)
    : game_{game}, players_{players} {}

std::string ListPlayersUseCase::GetSerializedSessionPlayers(
    const std::shared_ptr<Player> player_ptr) {
  const model::GameSession& session = player_ptr->GetSession();
  const model::GameSession::Dogs& dogs = session.GetDogs();
  json::object response;
  for (const auto& dog : dogs) {
    json::object player_object;
    player_object[name_key] = dog->GetName();
    response[dog->DogIdToString()] = player_object;
  }
  return json::serialize(response);
}

GameState::GameState(model::Game& game, Players& players)
    : game_{game}, players_{players} {}

json::array GameState::BuildBagArr(const std::shared_ptr<model::Dog>& dog) {
  json::array bag_array;
  model::Loots bag = dog->GetBag();
  for (const auto& item : bag) {
    json::object item_object;
    item_object[id_key] = item->id;
    item_object[type_key] = item->type;
    bag_array.push_back(item_object);
  }
  return bag_array;
}

std::string GameState::GetSerializedGameState(
    const std::shared_ptr<Player> player_ptr) {
  const model::GameSession& session = player_ptr->GetSession();
  const model::GameSession::Dogs& dogs = session.GetDogs();
  const std::vector<std::shared_ptr<model::Loot>> loots = session.GetLoots();
  json::object state_object;
  json::object players_object;
  for (const auto& dog : dogs) {
    json::object dog_object;
    dog_object[pos_key] =
        json::array{dog->GetCurCoordinates().x, dog->GetCurCoordinates().y};
    dog_object[speed_key] = json::array{dog->GetSpeed().x, dog->GetSpeed().y};
    dog_object[dir_key] = model::GetDirectionForJson(dog->GetDirection());
    dog_object[bag_key] = BuildBagArr(dog);
    dog_object[score_key] = dog->GetScore();
    players_object[dog->DogIdToString()] = std::move(dog_object);
  }
  json::object lost_objects;
  for (const auto& loot : loots) {
    json::object loot_obj;
    loot_obj[type_key] = loot->type;
    json::array loot_pos;
    loot_pos.push_back(loot->pos.x);
    loot_pos.push_back(loot->pos.y);
    loot_obj[pos_key] = loot_pos;
    lost_objects[std::to_string(loot->id)] = std::move(loot_obj);
  }
  state_object[players_key] = std::move(players_object);
  state_object[lost_objects_key] = std::move(lost_objects);
  return json::serialize(state_object);
}

void PlayerAction::ControlPlayerAction(const std::shared_ptr<Player> player_ptr,
                                       std::pair<double, double> dog_speed,
                                       const model::Direction& direction,
                                       bool is_active) {
  const model::GameSession& session = player_ptr->GetSession();
  const model::Map& map = session.GetMap();
  model::Dog& dog = player_ptr->GetDogNonConstant();
  double speed_on_map = map.GetDogSpeed();
  dog.SetMotionSettings(dog_speed, speed_on_map, direction);
  dog.SetActivity(is_active);
}

TickUseCase::TickUseCase(model::Game& game) : game_(game) {}

void TickUseCase::Tick(uint64_t time_delta) { game_.SetServerTick(time_delta); }

PlayersRecords::PlayersRecords(model::Game& game) : game_(game) {}

std::string PlayersRecords::GetSerializedRecords(size_t start,
                                                 size_t max_items) {
  const auto db_sptr = game_.GetDB();
  const auto players_records = db_sptr->GetTopPlayers(start, max_items);
  json::array records;
  for (const auto& record : players_records) {
    json::object record_obj;
    record_obj[name_key] = std::get<0>(record);
    record_obj[score_key] = std::get<1>(record);
    record_obj[play_time_key] = static_cast<double>(std::get<2>(record)) /
                             static_cast<double>(model::MS_IN_SECOND);
    records.push_back(record_obj);
  }
  return json::serialize(records);
}

Application::Application(model::Game& game, Players& players,
                         ApplicationListener& listener)
    : game_{game},
      players_{players},
      join_game_use_case_{game, players},
      get_map_use_case_{game},
      list_maps_use_case_{game},
      list_players_use_case_{game, players},
      game_state_{game, players},
      tick_use_case_{game},
      listener_{listener},
      players_records_{game} {}

std::string Application::ListMaps() {
  return list_maps_use_case_.GetSerializedMaps();
}

std::optional<json::object> Application::FindMap(const model::Map::Id& map_id) {
  auto map_ptr = game_.FindMap(map_id);
  if (!map_ptr) {
    return std::nullopt;
  }
  return get_map_use_case_.GetSerializedMap(map_ptr);
}

std::string Application::JoinGame(const model::Map::Id& map_id,
                                  const std::string& name,
                                  bool random_dog_coord) {
  try {
    JoinGameResult join_game_result =
        join_game_use_case_.Join(map_id, name, random_dog_coord);
    json::object response;
    response[auth_token_key] = join_game_result.GetToken().GetValue();
    response[player_id_key] = join_game_result.GetPlayerId().GetValue();
    return json::serialize(response);
  } catch (const JoinGameError& error) {
    if (error.GetReason() == JoinGameErrorReason::InvalidName) {
      return BuildError("invalidArgument", "Invalid name");
    } else if (error.GetReason() == JoinGameErrorReason::InvalidMap) {
      return BuildError("mapNotFound", "Map not found");
    }
  }
  return BuildError("badRequest", "Bad request");
}

std::string Application::ListPlayers(const std::shared_ptr<Player> player_ptr) {
  return list_players_use_case_.GetSerializedSessionPlayers(player_ptr);
}

std::string Application::ListGameState(
    const std::shared_ptr<Player> player_ptr) {
  return game_state_.GetSerializedGameState(player_ptr);
}

std::string Application::GetPlayersRecords(size_t start, size_t max_items) {
  return players_records_.GetSerializedRecords(start, max_items);
}

void Application::ModifyPlayer(const std::shared_ptr<Player> player_ptr,
                               std::pair<double, double> dog_speed,
                               const model::Direction& direction,
                               bool is_active) {
  player_action_.ControlPlayerAction(player_ptr, dog_speed, direction,
                                     is_active);
}

void Application::Tick(uint64_t time_delta) {
  tick_use_case_.Tick(time_delta);
  std::chrono::milliseconds time_delta_ms(time_delta);
  listener_.OnTick(time_delta_ms);
}

}  // namespace app