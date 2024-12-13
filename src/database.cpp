#include "database.h"

#include "tagged_uuid.h"

namespace database {

AppConfig GetConfigFromEnv() {
  AppConfig config;
  if (const auto* url = std::getenv(DB_URL_ENV_NAME)) {
    config.db_url = url;
  } else {
    throw std::runtime_error(DB_URL_ENV_NAME +
                             " environment variable not found"s);
  }
  return config;
}

Database::Database(const unsigned num_threads) {
  auto factory = []() {
    return std::make_shared<pqxx::connection>(GetConfigFromEnv().db_url);
  };
  connection_pool_ = std::make_shared<ConnectionPool>(num_threads, factory);
  CreateTable();
}

void Database::CreateTable() {
  auto conn = connection_pool_->GetConnection();
  pqxx::work work(*conn);
  work.exec(R"(CREATE TABLE IF NOT EXISTS retired_players (
    		id UUID PRIMARY KEY,
    		name varchar(100) NOT NULL,
			score integer NOT NULL,
			play_time_ms integer NOT NULL );
		)"_zv);
  work.exec(
      R"(CREATE INDEX IF NOT EXISTS scores_rating_idx ON retired_players (score DESC, play_time_ms, name);)"_zv);
  work.commit();
}

void Database::AddPlayerRecord(const std::string& name, const uint64_t score,
                               const uint64_t play_time_ms) {
  auto conn = connection_pool_->GetConnection();
  pqxx::work work(*conn);
  work.exec_params(
      R"(INSERT INTO retired_players (id, name, score, play_time_ms) VALUES ($1, $2, $3, $4))"_zv,
      util::detail::UUIDToString(util::detail::NewUUID()), name, score,
      play_time_ms);
  work.commit();
}

std::vector<std::tuple<std::string, size_t, size_t>> Database::GetTopPlayers(
    size_t offset, size_t limit) {
  std::vector<std::tuple<std::string, size_t, size_t>> players_records;
  auto conn = connection_pool_->GetConnection();
  pqxx::read_transaction read_transaction(*conn);
  auto query_text =
      "SELECT name, score, play_time_ms FROM retired_players ORDER BY score "
      "DESC, play_time_ms ASC, name ASC LIMIT " +
      read_transaction.quote(limit) + " OFFSET " +
      read_transaction.quote(offset) + ";";
  for (auto [name, score, play_time_ms] :
       read_transaction.query<std::string, size_t, size_t>(query_text)) {
    players_records.emplace_back(name, score, play_time_ms);
  }
  return players_records;
}

}  // namespace database