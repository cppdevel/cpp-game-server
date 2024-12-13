#pragma once

#include <boost/program_options.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace command_line_parser {

using namespace std::literals;

struct Args {
  unsigned int tick_period = 0;
  std::string config_file;
  std::string www_root;
  bool randomize_spawn_points = false;
  std::string state_file;
  uint64_t save_period = 0;
};

[[nodiscard]] std::optional<Args> ParseCommandLine(int argc,
                                                   const char* const argv[]);
std::string CommandLineToLower(std::string line);

}  // namespace command_line_parser