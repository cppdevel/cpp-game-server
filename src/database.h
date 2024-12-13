#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <pqxx/connection>
#include <pqxx/pqxx>
#include <pqxx/result>
#include <pqxx/transaction>
#include <stdexcept>
#include <vector>

#include "pqxx/zview.hxx"

namespace database {

using namespace std::literals;
using pqxx::operator"" _zv;

const size_t DEFAULT_OFFSET = 0;
const size_t DEFAULT_LIMIT = 100;

constexpr const char DB_URL_ENV_NAME[]{"GAME_DB_URL"};

struct AppConfig {
  std::string db_url;
};

AppConfig GetConfigFromEnv();

class ConnectionPool {
  using PoolType = ConnectionPool;
  using ConnectionPtr = std::shared_ptr<pqxx::connection>;

 public:
  class ConnectionWrapper {
   public:
    ConnectionWrapper(std::shared_ptr<pqxx::connection>&& conn,
                      PoolType& pool) noexcept
        : conn_{std::move(conn)}, pool_{&pool} {}

    ConnectionWrapper(const ConnectionWrapper&) = delete;
    ConnectionWrapper& operator=(const ConnectionWrapper&) = delete;

    ConnectionWrapper(ConnectionWrapper&&) = default;
    ConnectionWrapper& operator=(ConnectionWrapper&&) = default;

    pqxx::connection& operator*() const& noexcept { return *conn_; }
    pqxx::connection& operator*() const&& = delete;

    pqxx::connection* operator->() const& noexcept { return conn_.get(); }

    ~ConnectionWrapper() {
      if (conn_) {
        pool_->ReturnConnection(std::move(conn_));
      }
    }

   private:
    std::shared_ptr<pqxx::connection> conn_;
    PoolType* pool_;
  };

  // ConnectionFactory is a functional object returning
  // std::shared_ptr<pqxx::connection>
  template <typename ConnectionFactory>
  ConnectionPool(size_t capacity, ConnectionFactory&& connection_factory) {
    pool_.reserve(capacity);
    for (size_t i = 0; i < capacity; ++i) {
      pool_.emplace_back(connection_factory());
    }
  }

  ConnectionWrapper GetConnection() {
    std::unique_lock lock{mutex_};
    cond_var_.wait(lock, [this] { return used_connections_ < pool_.size(); });
    return {std::move(pool_[used_connections_++]), *this};
  }

 private:
  void ReturnConnection(ConnectionPtr&& conn) {
    {
      std::lock_guard lock{mutex_};
      pool_[--used_connections_] = std::move(conn);
    }
    cond_var_.notify_one();
  }

  std::mutex mutex_;
  std::condition_variable cond_var_;
  std::vector<ConnectionPtr> pool_;
  size_t used_connections_ = 0;
};

class Database {
 public:
  explicit Database(const unsigned num_threads);
  void AddPlayerRecord(const std::string& name, const uint64_t score,
                       const uint64_t play_time_ms);
  std::vector<std::tuple<std::string, size_t, size_t>> GetTopPlayers(
      size_t start, size_t limit);

 private:
  void CreateTable();
  std::shared_ptr<ConnectionPool> connection_pool_ = nullptr;
};

}  // namespace database