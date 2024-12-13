#define BOOST_USE_WINAPI_VERSION BOOST_WINAPI_VERSION_WIN7

#include <boost/asio/io_context.hpp>
#include <boost/asio/signal_set.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

#include "application.h"
#include "command_line.h"
#include "database.h"
#include "infrastructure.h"
#include "json_loader.h"
#include "request_handler.h"
#include "sdk.h"
#include "ticker.h"

using namespace std::literals;

namespace fs = std::filesystem;
namespace json = boost::json;
namespace net = boost::asio;
namespace sys = boost::system;

namespace {

// Запускает функцию fn на num_threads потоках, включая текущий
template <typename Fn>
void RunWorkers(unsigned num_threads, const Fn& fn) {
  num_threads = std::max(1u, num_threads);
  std::vector<std::jthread> workers;
  workers.reserve(num_threads - 1);
  // Запускаем num_threads-1 рабочих потоков, выполняющих функцию fn
  while (--num_threads) {
    workers.emplace_back(fn);
  }
  fn();
}

}  // namespace

BOOST_LOG_ATTRIBUTE_KEYWORD(additional_data, "AdditionalData", json::value)

int main(int argc, const char* argv[]) {
  command_line_parser::Args command_line_args;
  try {
    if (auto args = command_line_parser::ParseCommandLine(argc, argv)) {
      command_line_args = args.value();
    } else {
      return EXIT_FAILURE;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed parsing command line arguments: " << e.what()
              << std::endl;
    return EXIT_FAILURE;
  }
  try {
    const unsigned num_threads = std::thread::hardware_concurrency();

    fs::path config =
        fs::weakly_canonical(fs::path(command_line_parser::CommandLineToLower(
            command_line_args.config_file)));
    auto loading_game_res = json_loader::LoadGame(config);

    model::Game game = loading_game_res.first;
    game.SetDB(
        std::make_shared<database::Database>(database::Database(num_threads)));

    extra_data::Loots extra_data = loading_game_res.second;
    app::Players players;

    infrastructure::SerializingListener listener(game, players,
                                                 command_line_args.state_file,
                                                 command_line_args.save_period);
    app::Application application(game, players, listener);

    if (fs::exists(command_line_args.state_file) &&
        !command_line_args.state_file.empty()) {
      listener.RestoreState();
    }

    net::io_context ioc{static_cast<int>(num_threads)};

    // 3. Добавляем асинхронный обработчик сигналов SIGINT и SIGTERM
    // Подписываемся на сигналы и при их получении завершаем работу сервера
    net::signal_set signals(ioc, SIGINT, SIGTERM);
    signals.async_wait([&ioc, &listener](const sys::error_code& ec,
                                         [[maybe_unused]] int signal_number) {
      if (!ec) {
        ioc.stop();
        listener.SaveState();
      }
    });

    fs::path static_files_root = fs::weakly_canonical(fs::path(
        command_line_parser::CommandLineToLower(command_line_args.www_root)));

    // strand для выполнения запросов к API
    auto api_strand = net::make_strand(ioc);

    bool tick_param = false;
    if (command_line_args.tick_period > 0) {
      std::chrono::milliseconds time_milliseconds(
          command_line_args.tick_period);
      auto ticker = std::make_shared<ticker::Ticker>(
          api_strand, time_milliseconds,
          [&game, &listener](std::chrono::milliseconds time_delta) {
            game.SetServerTick(static_cast<uint64_t>(time_delta.count()));
            listener.SaveState();
          });
      tick_param = true;
      ticker->Start();
    }

    // Создаём обработчик запросов в куче, управляемый shared_ptr
    auto handler = std::make_shared<http_handler::RequestHandler>(
        static_files_root, api_strand, game, players,
        command_line_args.randomize_spawn_points, tick_param, extra_data,
        application);

    // Оборачиваем его в логирующий декоратор
    http_handler::server_logging::LoggingRequestHandler logging_handler{
        [handler](auto&& endpoint, auto&& req, auto&& send) {
          // Обрабатываем запрос
          (*handler)(std::forward<decltype(endpoint)>(endpoint),
                     std::forward<decltype(req)>(req),
                     std::forward<decltype(send)>(send));
        }};

    const auto address = net::ip::make_address("0.0.0.0");
    constexpr net::ip::port_type port = 8080;

    // Запускаем обработку запросов
    http_server::ServeHttp(ioc, {address, port}, logging_handler);

    json::value custom_data{{"port", port}, {"address", address.to_string()}};
    BOOST_LOG_TRIVIAL(info)
        << boost::log::add_value(additional_data, custom_data)
        << "server started"sv;

    RunWorkers(std::max(1u, num_threads), [&ioc] { ioc.run(); });
    listener.SaveState();
  } catch (const std::exception& ex) {
    json::value custom_data{{"code"s, EXIT_FAILURE}, {"exception"s, ex.what()}};
    BOOST_LOG_TRIVIAL(fatal)
        << boost::log::add_value(additional_data, custom_data)
        << "server exited"sv;
    return EXIT_FAILURE;
  }
  json::value custom_data{{"code"s, 0}};
  BOOST_LOG_TRIVIAL(info) << boost::log::add_value(additional_data, custom_data)
                          << "server exited"sv;
}