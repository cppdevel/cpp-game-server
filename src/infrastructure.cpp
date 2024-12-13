#include "infrastructure.h"

#include <fstream>

#include "boost/archive/text_iarchive.hpp"
#include "boost/archive/text_oarchive.hpp"

namespace infrastructure {

void SerializingListener::OnTick(std::chrono::milliseconds time_delta) {
  time_since_save_milliseconds_ += time_delta;
  if (time_since_save_milliseconds_ >= save_period_milliseconds_) {
    SaveState();
    time_since_save_milliseconds_ = std::chrono::milliseconds::zero();
  }
}

void SerializingListener::SaveState() {
  try {
    if (file_path_.empty()) {
      return;
    }
    std::filesystem::path temp_file = file_path_.string() + ".temporary";
    std::ofstream ofs;
    ofs.open(temp_file, std::ios_base::out);
    boost::archive::text_oarchive oa(ofs);
    serialization::GameRepr game_repr(game_, players_);
    oa << game_repr;
    ofs.close();
    std::filesystem::rename(temp_file, file_path_);
  } catch (const std::exception& e) {
    std::cerr << "Error saving game state: " << e.what() << std::endl;
  }
}

void SerializingListener::RestoreState() {
  if (file_path_.empty()) {
    return;
  }
  try {
    std::ifstream ifs;
    ifs.open(file_path_);
    if (!ifs) {
      throw std::runtime_error("Failed to ipen state file for reading.");
    }
    boost::archive::text_iarchive ia(ifs);
    serialization::GameRepr game_repr;
    ia >> game_repr;
    ifs.close();
    game_repr.Restore(game_, players_);
  } catch (const std::exception& e) {
    std::cerr << "Error restoring game state: " << e.what() << std::endl;
  }
}

}  //  namespace infrastructure