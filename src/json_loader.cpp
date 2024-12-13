#include "json_loader.h"

#include <boost/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

using namespace std::literals;

namespace json_loader {

void ProcessLGC(const json::object& parsed_json_obj, double lgc_period,
                double lgc_probability) {
  if (parsed_json_obj.if_contains("lootGeneratorConfig")) {
    json::object lgc_obj =
        parsed_json_obj.at("lootGeneratorConfig").as_object();
    if (lgc_obj.if_contains("period")) {
      lgc_period = lgc_obj.at("period").as_double();
    }
    if (lgc_obj.if_contains("probability")) {
      lgc_probability = lgc_obj.at("probability").as_double();
    }
  }
}

void ProcessLoots(model::Map& map, extra_data::Loots& loots,
                  json::array& loot_types_arr) {
  if (!loot_types_arr.empty()) {
    map.SetLootCount(static_cast<unsigned>(loot_types_arr.capacity()));
    extra_data::Loots::LootTypeList loot_list;
    unsigned loot_type_count = 0;
    for (const auto& loot_item : loot_types_arr) {
      if (loot_item.as_object().if_contains("value")) {
        int value = json::value_to<int>(loot_item.at("value"));
        map.AddLootValue(loot_type_count, value);
      }
      loot_list.push_back(loot_item.as_object());
      map.AddLoot(model::Loot(loot_type_count));
      ++loot_type_count;
    }
    loots.AddLoot(map.GetId().GetValue(), loot_list);
  }
}

void ProcessRoads(const json::value& map_from_array, model::Map& map) {
  auto& roads = map_from_array.at("roads").as_array();
  for (const auto& road_from_array : roads) {
    if (road_from_array.as_object().contains("x1")) {
      model::Road road{
          model::Road::HORIZONTAL,
          model::Point{json::value_to<int>(road_from_array.at("x0")),
                       json::value_to<int>(road_from_array.at("y0"))},
          json::value_to<int>(road_from_array.at("x1"))};
      map.AddRoad(std::move(road));
    } else {
      model::Road road{
          model::Road::VERTICAL,
          model::Point{json::value_to<int>(road_from_array.at("x0")),
                       json::value_to<int>(road_from_array.at("y0"))},
          json::value_to<int>(road_from_array.at("y1"))};
      map.AddRoad(std::move(road));
    }
  }
}

void ProcessBuildings(const json::value& map_from_array, model::Map& map) {
  auto& buildings = map_from_array.at("buildings").as_array();
  for (const auto& building : buildings) {
    model::Rectangle rectangle{
        model::Point{json::value_to<int>(building.at("x")),
                     json::value_to<int>(building.at("y"))},
        model::Size{json::value_to<int>(building.at("w")),
                    json::value_to<int>(building.at("h"))}};
    map.AddBuilding(model::Building(std::move(rectangle)));
  }
}

void ProcessOffices(const json::value& map_from_array, model::Map& map) {
  auto& offices = map_from_array.at("offices").as_array();
  for (const auto& office : offices) {
    model::Office::Id id{json::value_to<std::string>(office.at("id"))};
    model::Point coordinates{json::value_to<int>(office.at("x")),
                             json::value_to<int>(office.at("y"))};
    model::Offset offset{json::value_to<int>(office.at("offsetX")),
                         json::value_to<int>(office.at("offsetY"))};
    map.AddOffice(model::Office((id), coordinates, offset));
  }
}

void ProcessMaps(const json::object& parsed_json_obj, model::Game& game,
                 double dog_speed, int bag_capacity, double dog_retirement_time,
                 extra_data::Loots& loots) {
  auto& maps = parsed_json_obj.at("maps").as_array();
  extra_data::Loots::MapLootData map_loot_data;
  for (const auto& map_from_array : maps) {
    const auto map_id = json::value_to<std::string>(map_from_array.at("id"));
    const auto map_name =
        json::value_to<std::string>(map_from_array.at("name"));
    model::Map map = model::Map(model::Map::Id(map_id), map_name);

    if (map_from_array.as_object().if_contains("dogSpeed")) {
      double dog_speed_on_map =
          json::value_to<double>(map_from_array.at("dogSpeed"));
      map.SetDogSpeed(std::move(dog_speed_on_map));
    } else {
      map.SetDogSpeed(dog_speed);
    }

    if (map_from_array.as_object().if_contains("bagCapacity")) {
      int bag_capacity_on_map =
          json::value_to<int>(map_from_array.at("bagCapacity"));
      map.SetBagCapacity(std::move(bag_capacity_on_map));
    } else {
      map.SetBagCapacity(bag_capacity);
    }

    json::array loot_types_arr;
    if (map_from_array.as_object().if_contains("lootTypes")) {
      loot_types_arr = map_from_array.at("lootTypes").as_array();
    }

    ProcessLoots(map, loots, loot_types_arr);
    ProcessRoads(map_from_array, map);
    ProcessBuildings(map_from_array, map);
    ProcessOffices(map_from_array, map);
    map.SetDogRetirementTime(dog_retirement_time);
    game.AddMap(map);
  }
}

void ProcessGame(const json::object& parsed_json_obj, model::Game& game,
                 extra_data::Loots& loots) {
  constexpr const double DEFAULT_DOG_SPEED = 1.0;
  double dog_speed = DEFAULT_DOG_SPEED;
  if (parsed_json_obj.if_contains("defaultDogSpeed")) {
    dog_speed = json::value_to<double>(parsed_json_obj.at("defaultDogSpeed"));
  }

  constexpr const int DEFAULT_BAG_CAPACITY = 3;
  int bag_capacity = DEFAULT_BAG_CAPACITY;
  if (parsed_json_obj.if_contains("defaultBagCapacity")) {
    bag_capacity =
        json::value_to<int>(parsed_json_obj.at("defaultBagCapacity"));
  }

  constexpr const double DEFAULT_DOG_RETIREMENT_TIME_MS = 60000.0;
  double dog_retirement_time = DEFAULT_DOG_RETIREMENT_TIME_MS;
  if (parsed_json_obj.if_contains("dogRetirementTime")) {
    dog_retirement_time =
        json::value_to<double>(parsed_json_obj.at("dogRetirementTime"));
  }

  double lgc_period = 1.0;
  double lgc_probability = 0.5;
  ProcessLGC(parsed_json_obj, lgc_period, lgc_probability);
  std::chrono::milliseconds lgc_period_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(lgc_period *
                                                            1000ms);
  game.SetLootGen(lgc_period_ms, lgc_probability);
  ProcessMaps(parsed_json_obj, game, dog_speed, bag_capacity,
              dog_retirement_time, loots);
}

std::pair<model::Game, extra_data::Loots> LoadGame(const fs::path& json_path) {
  fs::path json_path_content = json_path.string();
  std::string json_string;
  std::ifstream json_istream(json_path_content);
  if (json_istream.is_open()) {
    std::stringstream ss;
    ss << json_istream.rdbuf();
    json_string = ss.str();
  } else {
    throw std::runtime_error("No file found");
  }
  auto parsed_json_obj = json::parse(json_string).as_object();
  model::Game game;
  extra_data::Loots loots;
  ProcessGame(parsed_json_obj, game, loots);
  return {game, loots};
}

}  // namespace json_loader