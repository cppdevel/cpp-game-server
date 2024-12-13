#pragma once

#define BOOST_BEAST_USE_STD_STRING_VIEW

#include <boost/json.hpp>
#include <chrono>
#include <iomanip>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "model.h"
#include "tagged.h"

namespace app {

using namespace std::literals;
namespace json = boost::json;

namespace constant_literals {
constexpr const char* x_key = "x";
constexpr const char* y_key = "y";
constexpr const char* code_key = "code";
constexpr const char* message_key = "message";
constexpr const char* x0_key = "x0";
constexpr const char* y0_key = "y0";
constexpr const char* x1_key = "x1";
constexpr const char* y1_key = "y1";
constexpr const char* w_key = "w";
constexpr const char* h_key = "h";
constexpr const char* id_key = "id";
constexpr const char* offset_x_key = "offsetX";
constexpr const char* offset_y_key = "offsetY";
constexpr const char* name_key = "name";
constexpr const char* roads_key = "roads";
constexpr const char* buildings_key = "buildings";
constexpr const char* offices_key = "offices";
constexpr const char* type_key = "type";
constexpr const char* pos_key = "pos";
constexpr const char* speed_key = "speed";
constexpr const char* dir_key = "dir";
constexpr const char* bag_key = "bag";
constexpr const char* score_key = "score";
constexpr const char* players_key = "players";
constexpr const char* lost_objects_key = "lostObjects";
constexpr const char* play_time_key = "playTime";
constexpr const char* auth_token_key = "authToken";
constexpr const char* player_id_key = "playerId";
}  // namespace constant_literals

std::string BuildError(const std::string& code, const std::string& message);

namespace detail {
struct TokenTag {};
}  // namespace detail

class Player {
 public:
  using Token = util::Tagged<std::string, detail::TokenTag>;

  explicit Player(Token token, std::shared_ptr<model::Dog> dog,
                  std::shared_ptr<model::GameSession> session) noexcept;
  const Token& GetToken() const noexcept;
  const model::GameSession& GetSession() const noexcept;
  std::shared_ptr<model::Dog> GetDog() const noexcept;
  model::Dog& GetDogNonConstant();
  model::Dog::Id GetId();

 private:
  Token token_;
  std::shared_ptr<model::GameSession> session_;
  std::shared_ptr<model::Dog> dog_;
};

class Players {
 public:
  std::shared_ptr<Player> AddPlayer(
      std::shared_ptr<model::Dog> dog,
      std::shared_ptr<model::GameSession> session);
  void RemovePlayer(const std::shared_ptr<Player> player_sptr);
  std::shared_ptr<Player> FindByDogIdAndMapId(const model::Dog::Id& dog_id,
                                              const model::Map::Id& map_id);
  std::shared_ptr<Player> FindPlayerByToken(const Player::Token& token);
  std::vector<std::shared_ptr<Player>> GetPlayers() const noexcept;
  void AddRepresPlayer(std::shared_ptr<app::Player> player_sptr);

 private:
  Player::Token GenerateToken();

  std::random_device random_device_;
  std::mt19937_64 generator1_{[this] {
    std::uniform_int_distribution<std::mt19937_64::result_type> dist;
    return dist(random_device_);
  }()};
  std::mt19937_64 generator2_{[this] {
    std::uniform_int_distribution<std::mt19937_64::result_type> dist;
    return dist(random_device_);
  }()};

  std::vector<std::shared_ptr<Player>> players_;
  std::unordered_map<Player::Token, std::shared_ptr<Player>,
                     util::TaggedHasher<Player::Token>>
      token_to_player_;
};

class GetMapUseCase {
 public:
  explicit GetMapUseCase(model::Game& game);
  json::object GetSerializedMap(std::shared_ptr<model::Map> map_ptr);

 private:
  static json::array LoadRoads(const model::Map& map);
  static json::array LoadBuildings(const model::Map& map);
  static json::array LoadOffices(const model::Map& map);
  static json::object BuildTheMap(const model::Map& map);
  model::Game& game_;
};

enum class JoinGameErrorReason { InvalidName, InvalidMap };

class JoinGameError {
 public:
  explicit JoinGameError(JoinGameErrorReason reason);
  const JoinGameErrorReason GetReason() const noexcept;

 private:
  JoinGameErrorReason reason_;
};

class JoinGameResult {
 public:
  explicit JoinGameResult(const Player::Token& token,
                          const model::Dog::Id& dog_id);
  const Player::Token& GetToken() const noexcept;
  const model::Dog::Id& GetPlayerId() const noexcept;

 private:
  const Player::Token& token_;
  const model::Dog::Id& player_id_;
};

class JoinGameUseCase {
 public:
  explicit JoinGameUseCase(model::Game& game, Players& players);
  JoinGameResult Join(const model::Map::Id& map_id, std::string name,
                      bool random);

 private:
  model::Game& game_;
  Players& players_;
};

class ListMapsUseCase {
 public:
  explicit ListMapsUseCase(model::Game& game);
  std::string GetSerializedMaps();

 private:
  static std::string BuildMaps(std::vector<std::shared_ptr<model::Map>> maps);
  model::Game& game_;
};

class ListPlayersUseCase {
 public:
  explicit ListPlayersUseCase(model::Game& game, Players& players);
  std::string GetSerializedSessionPlayers(
      const std::shared_ptr<Player> player_ptr);

 private:
  model::Game& game_;
  Players& players_;
};

class GameState {
 public:
  explicit GameState(model::Game& game, Players& players);
  json::array BuildBagArr(const std::shared_ptr<model::Dog>& dog);
  std::string GetSerializedGameState(const std::shared_ptr<Player> player_ptr);

 private:
  model::Game& game_;
  Players& players_;
};

class PlayerAction {
 public:
  void ControlPlayerAction(const std::shared_ptr<Player> player_ptr,
                           std::pair<double, double> dog_speed,
                           const model::Direction& direction, bool is_active);
};

class TickUseCase {
 public:
  explicit TickUseCase(model::Game& game);
  void Tick(uint64_t time_delta);

 private:
  model::Game& game_;
};

class ApplicationListener {
 public:
  virtual void OnTick(std::chrono::milliseconds time_delta) = 0;
};

class PlayersRecords {
 public:
  explicit PlayersRecords(model::Game& game);
  std::string GetSerializedRecords(size_t start, size_t max_items);

 private:
  model::Game& game_;
};

class Application {
 public:
  explicit Application(model::Game& game, Players& players,
                       ApplicationListener& listener);
  std::string ListMaps();
  std::optional<json::object> FindMap(const model::Map::Id& map_id);
  std::string JoinGame(const model::Map::Id& map_id, const std::string& name,
                       bool random_dog_coord);
  std::string ListPlayers(const std::shared_ptr<Player> player_ptr);
  std::string ListGameState(const std::shared_ptr<Player> player_ptr);
  std::string GetPlayersRecords(size_t start, size_t max_items);
  void ModifyPlayer(const std::shared_ptr<Player> player_ptr,
                    std::pair<double, double> dog_speed,
                    const model::Direction& direction, bool is_active);
  void Tick(uint64_t time_delta);

 private:
  model::Game& game_;
  Players& players_;
  JoinGameUseCase join_game_use_case_;
  GetMapUseCase get_map_use_case_;
  ListMapsUseCase list_maps_use_case_;
  ListPlayersUseCase list_players_use_case_;
  GameState game_state_;
  PlayerAction player_action_;
  TickUseCase tick_use_case_;
  ApplicationListener& listener_;
  PlayersRecords players_records_;
};

}  // namespace app