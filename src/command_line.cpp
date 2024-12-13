#include "command_line.h"

namespace command_line_parser {

std::optional<Args> ParseCommandLine(int argc, const char* const argv[]) {
  namespace po = boost::program_options;
  po::options_description desc{"All options"s};
  Args args;
  desc.add_options()("help,h", "produce help message")(
      "tick-period,t",
      po::value<unsigned int>(&args.tick_period)->value_name("milliseconds"s),
      "set tick period")("config-file,c",
                         po::value(&args.config_file)->value_name("files"s),
                         "set config file path")(
      "www-root,w", po::value(&args.www_root)->value_name("dir"s),
      "set static files root")("state-file,s",
                               po::value(&args.state_file)->value_name("file"s),
                               "set state file path")(
      "save-state-period,p", po::value(&args.save_period)->value_name("ms"s),
      "set state file save period");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.contains("help"s)) {
    std::cout << desc;
    return std::nullopt;
  }
  if (!vm.contains("config-file"s) || !vm.contains("www-root"s)) {
    throw std::runtime_error(
        "Missing necessary arguments: check config-file and www-root"s);
  }
  if (vm.contains("randomize-spawn-points")) {
    args.randomize_spawn_points = true;
  }
  return args;
}

std::string CommandLineToLower(std::string line) {
  std::transform(line.begin(), line.end(), line.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return line;
}

}  // namespace command_line_parser