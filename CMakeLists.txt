cmake_minimum_required(VERSION 3.11)

project(game_server CXX)
set(CMAKE_CXX_STANDARD 20)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

find_package(Boost 1.78.0 REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

include(CTest)
include(${CONAN_BUILD_DIRS_CATCH2}/Catch.cmake)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(GameServerStaticLib STATIC 
        src/boost_json.cpp
        src/collision_detector.h
        src/collision_detector.cpp
	src/database.h
	src/database.cpp
        src/extra_data.h
        src/extra_data.cpp
        src/geom.h
        src/loot_generator.h
        src/loot_generator.cpp
        src/model.h
        src/model.cpp
        src/model_serialization.h
	src/model_serialization.cpp
        src/sdk.h
        src/tagged.h
        src/tagged_uuid.h
        src/tagged_uuid.cpp
)

target_include_directories(GameServerStaticLib PUBLIC ${ZLIB_INCLUDES} CONAN_PKG::libpq CONAN_PKG::libpqxx)
target_link_libraries(GameServerStaticLib PUBLIC ${BOOST_LIB} ${ZLIB_LIB} CONAN_PKG::libpq CONAN_PKG::libpqxx)

add_executable(game_server
        src/main.cpp
        src/application.h
        src/application.cpp
        src/command_line.h
        src/command_line.cpp
        src/infrastructure.h
        src/infrastructure.cpp
        src/http_server.h
        src/http_server.cpp
        src/json_loader.h
        src/json_loader.cpp
        src/request_handler.cpp
        src/request_handler.h
        src/ticker.h
        src/ticker.cpp
)

target_include_directories(game_server PUBLIC CONAN_PKG::boost)
target_link_libraries(game_server CONAN_PKG::boost) 

add_executable(game_server_tests
        tests/loot_generator_tests.cpp
)

target_include_directories(game_server_tests PUBLIC CONAN_PKG::boost)
target_link_libraries(game_server_tests CONAN_PKG::catch2 CONAN_PKG::boost Threads::Threads) 

# для сборки под windows 10 и выше, чтобы избежать ошибок линковщика из-за boost::log
if(WIN32)
  target_compile_definitions(game_server PRIVATE _WIN32_WINNT=0x0A00)
endif()

target_link_libraries(game_server GameServerStaticLib)
target_link_libraries(game_server_tests ${CATCH2_LIB} GameServerStaticLib)

catch_discover_tests(game_server_tests)
