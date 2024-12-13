#pragma once

#include <boost/json.hpp>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "loot_generator.h"
#include "model.h"

namespace extra_data {

namespace json = boost::json;

class Loots {
 public:
  using LootType = json::object;
  using LootTypeList = std::vector<LootType>;
  using MapLootData = std::map<std::string, LootTypeList>;

  void AddLoot(const std::string& map_name, const LootTypeList loots_types);
  std::optional<LootTypeList> GetMapLoot(const std::string map_id) const;
  json::array BuildMapLoots(const std::string& map_id) const;
  std::string GetMapLootResponce(const std::string& map_id,
                                 json::object& responce_to_handler) const;

 private:
  MapLootData map_id_to_loots_;
};

}  // namespace extra_data