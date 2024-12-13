#pragma once

#include <chrono>
#include <filesystem>

#include "application.h"
#include "model.h"
#include "model_serialization.h"

namespace infrastructure {

class SerializingListener : public app::ApplicationListener {
 public:
  explicit SerializingListener(model::Game& game, app::Players& players,
                               std::filesystem::path file_path,
                               uint64_t save_period)
      : game_(game),
        players_(players),
        file_path_(file_path),
        save_period_milliseconds_(save_period),
        time_since_save_milliseconds_(0) {}

  void OnTick(std::chrono::milliseconds time_delta) override;
  void SaveState();
  void RestoreState();

 private:
  model::Game& game_;
  app::Players& players_;
  std::filesystem::path file_path_;
  std::chrono::milliseconds save_period_milliseconds_;
  std::chrono::milliseconds time_since_save_milliseconds_;
};

}  // namespace infrastructure