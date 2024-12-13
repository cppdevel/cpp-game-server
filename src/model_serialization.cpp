#include "model_serialization.h"

namespace serialization {

LootRepr::LootRepr(model::Loot loot)
    : id_(loot.id),
      type_(loot.type),
      pos_(loot.pos),
      id_count_(loot.loot_id_count) {}

[[nodiscard]] model::Loot LootRepr::Restore() const {
  model::Loot loot{id_, type_, pos_};
  loot.SetIdCount(id_count_);
  return loot;
}

DogRepr::DogRepr(model::Dog& dog)
    : id_(dog.GetId().GetValue()),
      name_(dog.GetName()),
      cur_pos_(dog.GetCurCoordinates()),
      prev_pos_(dog.GetPrevCoordinates()),
      speed_(dog.GetSpeed()),
      direction_(dog.GetDirection()),
      score_(dog.GetScore()),
      id_count_(dog.GetIdCount()) {
  for (const auto& item : dog.GetBag()) {
    bag_content_.push_back(LootRepr(*item));
  }
}

[[nodiscard]] model::Dog DogRepr::Restore() const {
  model::Dog dog{model::Dog::Id(id_), name_};
  dog.SetCurCoordinates(cur_pos_);
  dog.SetPrevCoordinates(prev_pos_);
  dog.SetSpeed(speed_);
  dog.SetDirection(direction_);
  dog.AddScore(score_);
  dog.SetIdCount(id_count_);
  for (const auto& item : bag_content_) {
    dog.AddBagItem(std::make_shared<model::Loot>(item.Restore()));
  }
  return dog;
}

PlayerRepr::PlayerRepr(app::Player player)
    : token_(player.GetToken().GetValue()),
      dog_id_(player.GetId().GetValue()),
      session_id_(player.GetSession().GetSessionId().GetValue()) {}

[[nodiscard]] app::Player PlayerRepr::Restore(
    std::shared_ptr<model::GameSession> session) const {
  return app::Player(app::Player::Token{token_}, session->FindDogById(dog_id_),
                     session);
}

GameSessionRepr::GameSessionRepr(model::GameSession session)
    : session_id_(session.GetSessionId().GetValue()),
      map_id_(session.GetMapId().GetValue()),
      id_count_(session.GetIdCount()) {
  for (const auto& loot : session.GetLoots()) {
    loots_repr_.push_back(LootRepr(*loot));
  }
  for (const auto& dog : session.GetDogs()) {
    dogs_repr_.push_back(DogRepr(*dog));
  }
}

[[nodiscard]] model::GameSession GameSessionRepr::Restore(
    const model::Game& game) const {
  auto map_sptr = game.FindMap(model::Map::Id(map_id_));
  model::GameSession game_session(session_id_, map_sptr, game.GetLootGen());
  game_session.SetIdCount(id_count_);
  for (const auto& loot_repr : loots_repr_) {
    game_session.AddLoot(loot_repr.Restore());
    game_session.AddLootToCollisionObj(loot_repr.pos_,
                                       game_session.GetLoots().back());
  }
  for (const auto& dog_repr : dogs_repr_) {
    game_session.AddDog(std::make_shared<model::Dog>(dog_repr.Restore()));
  }
  return game_session;
}

GameRepr::GameRepr(const model::Game& game, const app::Players& players) {
  for (const auto& session : game.GetGameSessions()) {
    game_sessions_repr_.push_back(GameSessionRepr(*session));
  }
  for (const auto& player : players.GetPlayers()) {
    players_repr_.push_back(PlayerRepr(*player));
  }
}

void GameRepr::Restore(model::Game& new_game, app::Players& new_players) const {
  for (const auto& session_repr : game_sessions_repr_) {
    std::shared_ptr<model::GameSession> session =
        std::make_shared<model::GameSession>(session_repr.Restore(new_game));
    new_game.AddGameSession(std::move(session));
  }
  for (const auto& player_repr : players_repr_) {
    auto session = new_game.FindGameSessionById(
        model::GameSession::Id(player_repr.session_id_));
    auto player = player_repr.Restore(session);
    new_players.AddRepresPlayer(std::make_shared<app::Player>(player));
  }
}

}  // namespace serialization