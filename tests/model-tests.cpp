#include <cmath>
#include <catch2/catch_test_macros.hpp>

#include "../src/model.h"

using namespace std::literals;

using namespace model;
using namespace std::literals;

SCENARIO("Loot object creation and manipulation") {
    GIVEN("A loot object with a specific type") {
        Loot loot(5);

        THEN("Loot should have the correct type") {
            REQUIRE(loot.type == 5);
        }

        THEN("Loot should have a unique ID") {
            Loot loot2(3);
            REQUIRE(loot.id != loot2.id);
        }

        THEN("Loot position should be default initialized") {
            REQUIRE(loot.pos.x == 0.0);
            REQUIRE(loot.pos.y == 0.0);
        }
    }
}

SCENARIO("Map object creation and adding loot") {
    GIVEN("A map with a loot count of 10") {
        Map::Id map_id{ "map1" };
        Map map(map_id, "Test Map");
        map.SetLootCount(10);

        THEN("The map should have the correct loot count") {
            REQUIRE(map.GetLootCount() == 10);
        }

        WHEN("Adding a loot object to the map") {
            Loot loot(2);
            map.AddLoot(loot);

            THEN("The map's loot count should increase by 1") {
                REQUIRE(map.GetLoots().size() == 1);
            }
        }
    }
}