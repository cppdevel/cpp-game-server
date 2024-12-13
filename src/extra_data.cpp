#include "extra_data.h"

namespace extra_data {

void Loots::AddLoot(const std::string& map_name,
                    const Loots::LootTypeList loots_types) {
  map_id_to_loots_[map_name] = loots_types;
}

std::optional<Loots::LootTypeList> Loots::GetMapLoot(
    const std::string map_id) const {
  auto it = map_id_to_loots_.find(map_id);
  if (it != map_id_to_loots_.end()) {
    return it->second;
  }
  return std::nullopt;
}

json::array Loots::BuildMapLoots(const std::string& map_id) const {
  std::optional<LootTypeList> loots = GetMapLoot(map_id);
  if (!loots.has_value()) {
    return {};
  }
  json::array loots_array;
  for (const auto& loot : loots.value()) {
    loots_array.emplace_back(loot);
  }
  return loots_array;
}

std::string Loots::GetMapLootResponce(const std::string& map_id,
                                      json::object& responce_to_handler) const {
  responce_to_handler["lootTypes"] = BuildMapLoots(map_id);
  return json::serialize(responce_to_handler);
}

}  // namespace extra_data