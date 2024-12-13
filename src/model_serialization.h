#include <boost/serialization/vector.hpp>

#include "application.h"
#include "model.h"

namespace geom {

template <typename Archive>
void serialize(Archive& ar, Point2D& point,
               [[maybe_unused]] const unsigned version) {
  ar & point.x;
  ar & point.y;
}

template <typename Archive>
void serialize(Archive& ar, Vec2D& vec,
               [[maybe_unused]] const unsigned version) {
  ar & vec.x;
  ar & vec.y;
}

}  // namespace geom

namespace model {

template <typename Archive>
void serialize(Archive& ar, Loot& obj,
               [[maybe_unused]] const unsigned version) {
  ar&(obj.id);
  ar&(obj.type);
  ar&(obj.pos);
}

}  // namespace model

namespace serialization {

class LootRepr {
 public:
  LootRepr() = default;
  explicit LootRepr(model::Loot loot);
  [[nodiscard]] model::Loot Restore() const;

  template <typename Archive>
  void serialize(Archive& ar, [[maybe_unused]] const unsigned version) {
    ar & id_;
    ar & type_;
    ar & pos_;
    ar & id_count_;
  }

 private:
  unsigned id_ = 0, type_ = 0;

 public:
  geom::Point2D pos_;
  unsigned id_count_;
};

class DogRepr {
 public:
  DogRepr() = default;
  explicit DogRepr(model::Dog& dog);
  [[nodiscard]] model::Dog Restore() const;

  template <typename Archive>
  void serialize(Archive& ar, [[maybe_unused]] const unsigned version) {
    ar & id_;
    ar & name_;
    ar & cur_pos_;
    ar & prev_pos_;
    ar & speed_;
    ar & direction_;
    ar & score_;
    ar & id_count_;
    ar & bag_content_;
  }

 private:
  int id_;
  std::string name_;
  geom::Point2D cur_pos_;
  geom::Point2D prev_pos_;
  geom::Vec2D speed_;
  model::Direction direction_ = model::Direction::NORTH;
  unsigned score_ = 0;
  unsigned id_count_;
  std::vector<LootRepr> bag_content_;
};

class PlayerRepr {
 public:
  PlayerRepr() = default;
  explicit PlayerRepr(app::Player player);
  [[nodiscard]] app::Player Restore(
      std::shared_ptr<model::GameSession> session) const;

  template <typename Archive>
  void serialize(Archive& ar, [[maybe_unused]] const unsigned version) {
    ar & token_;
    ar & dog_id_;
    ar & session_id_;
  }

 private:
  std::string token_;
  unsigned dog_id_;

 public:
  unsigned session_id_;
};

class GameSessionRepr {
 public:
  GameSessionRepr() = default;
  explicit GameSessionRepr(model::GameSession session);
  [[nodiscard]] model::GameSession Restore(const model::Game& game) const;

  template <typename Archive>
  void serialize(Archive& ar, [[maybe_unused]] const unsigned version) {
    ar & session_id_;
    ar & map_id_;
    ar & loots_repr_;
    ar & dogs_repr_;
    ar & id_count_;
  }

 private:
  unsigned session_id_;
  std::string map_id_;
  std::vector<LootRepr> loots_repr_;
  std::vector<DogRepr> dogs_repr_;
  unsigned id_count_;
};

class GameRepr {
 public:
  GameRepr() = default;
  explicit GameRepr(const model::Game& game, const app::Players& players);
  void Restore(model::Game& new_game, app::Players& new_players) const;

  template <typename Archive>
  void serialize(Archive& ar, [[maybe_unused]] const unsigned version) {
    ar & game_sessions_repr_;
    ar & players_repr_;
  }

 private:
  std::vector<GameSessionRepr> game_sessions_repr_;
  std::vector<PlayerRepr> players_repr_;
};

}  // namespace serialization