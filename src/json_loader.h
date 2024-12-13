#pragma once

#include <boost/json.hpp>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include "extra_data.h"
#include "loot_generator.h"
#include "model.h"

namespace json = boost::json;
namespace fs = std::filesystem;

namespace json_loader {

// auxiliary functions for LoadGame
void ProcessLGC(const json::object& parsed_json_obj, double lgc_period,
                double lgc_probability);
void ProcessLoots(model::Map& map, extra_data::Loots& loots,
                  json::array& loot_types_arr);
void ProcessRoads(const json::value& map_from_array, model::Map& map);
void ProcessBuildings(const json::value& map_from_array, model::Map& map);
void ProcessOffices(const json::value& map_from_array, model::Map& map);
void ProcessMaps(const json::object& parsed_json_obj, model::Game& game,
                 double dog_speed, int bag_capacity, double dog_retirement_time,
                 extra_data::Loots& loots);
void ProcessGame(const json::object& parsed_json_obj, model::Game& game,
                 extra_data::Loots& loots);

std::pair<model::Game, extra_data::Loots> LoadGame(const fs::path& json_path);

}  // namespace json_loader