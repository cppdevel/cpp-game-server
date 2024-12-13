#pragma once

#define BOOST_USE_WINAPI_VERSION BOOST_WINAPI_VERSION_WIN7
#define BOOST_BEAST_USE_STD_STRING_VIEW

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core/file_base.hpp>
#include <boost/beast/core/string.hpp>
#include <boost/beast/http.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/json.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

#include "application.h"
#include "extra_data.h"
#include "http_server.h"
#include "model.h"

using namespace boost::posix_time;
using namespace std::literals;

namespace beast = boost::beast;
namespace expr = boost::log::expressions;
namespace fs = std::filesystem;
namespace http = beast::http;
namespace json = boost::json;
namespace net = boost::asio;
namespace sys = boost::system;

using tcp = net::ip::tcp;

namespace http_handler {

using namespace std::literals;

using EmptyResponse = http::response<http::empty_body>;
using FileResponse = http::response<http::file_body>;
using StringRequest = http::request<http::string_body>;
using StringResponse = http::response<http::string_body>;

std::string DecodeURL(const fs::path& encoded_url);

struct ContentType {
  ContentType() = delete;

  constexpr static std::string_view TEXT_HTML = "text/html"sv;
  constexpr static std::string_view TEXT_CSS = "text/css"sv;
  constexpr static std::string_view TEXT_PLAIN = "text/plain"sv;
  constexpr static std::string_view TEXT_JS = "text/javascript"sv;
  constexpr static std::string_view APP_JSON = "application/json"sv;
  constexpr static std::string_view APP_XML = "application/xml"sv;
  constexpr static std::string_view IMG_PNG = "image/png"sv;
  constexpr static std::string_view IMG_JPEG = "image/jpeg"sv;
  constexpr static std::string_view IMG_GIF = "image/gif"sv;
  constexpr static std::string_view IMG_BMP = "image/bmp"sv;
  constexpr static std::string_view IMG_VMI = "image/vnd.microsoft.icon"sv;
  constexpr static std::string_view IMG_TIFF = "image/tiff"sv;
  constexpr static std::string_view IMG_SVG_XML = "image/svg+xml"sv;
  constexpr static std::string_view AUDIO_MPEG = "audio/mpeg"sv;
  constexpr static std::string_view APP_OS = "application/octet-stream"sv;
};

BOOST_LOG_ATTRIBUTE_KEYWORD(additional_data, "AdditionalData", json::value)
BOOST_LOG_ATTRIBUTE_KEYWORD(timestamp, "TimeStamp", boost::posix_time::ptime)

namespace server_logging {

template <class SomeRequestHandler>
class LoggingRequestHandler {
 public:
  LoggingRequestHandler(SomeRequestHandler some_rh) : some_rh_{some_rh} {
    InitLogWithFormatter();
  }

  template <typename Body, typename Allocator, typename Send>
  void operator()(tcp::endpoint address,
                  http::request<Body, http::basic_fields<Allocator>>&& req,
                  Send&& send) {
    LogRequest(std::forward<decltype(req)>(req), address);
    auto start_time = boost::posix_time::second_clock::local_time();
    auto sending_rq = [&send, this, start_time](auto&& result) {
      this->LogResponse(result, start_time);
      send(std::forward<decltype(result)>(result));
    };
    some_rh_(address, std::forward<decltype(req)>(req), std::move(sending_rq));
  }

 private:
  static void Formatter(boost::log::record_view const& rec,
                        boost::log::formatting_ostream& strm) {
    auto ts = *rec[timestamp];
    json::value log{{"timestamp"s, to_iso_extended_string(ts)},
                    {"data"s, *rec[additional_data]},
                    {"message"s, *rec[expr::smessage]}};
    strm << log;
  }

  static void InitLogWithFormatter() {
    boost::log::add_common_attributes();
    auto console_log = boost::log::add_console_log(
        std::cout, boost::log::keywords::auto_flush = true);
    console_log->set_formatter(&Formatter);
  }

  template <typename Response>
  static std::string GetContentTypeFromResponse(const Response& response) {
    auto it = std::find_if(response.base().begin(), response.base().end(),
                           [](const auto& header) {
                             return header.name_string() == "Content-Type";
                           });
    if (it != response.base().end()) {
      return std::string{it->value()};
    }
    return "null";
  }

  template <typename Request>
  static void LogRequest(const Request& request, const tcp::endpoint& address) {
    json::value custom_data{{"ip"s, address.address().to_string()},
                            {"URI"s, request.target()},
                            {"method"s, request.method_string()}};
    BOOST_LOG_TRIVIAL(info)
        << boost::log::add_value(additional_data, custom_data)
        << "request received"sv;
  }

  template <typename Body>
  static void LogResponse(const http::response<Body>& response,
                          boost::posix_time::ptime start) {
    auto response_time = start - boost::posix_time::second_clock::local_time();
    std::string content_type = GetContentTypeFromResponse(response);
    json::value custom_data{
        {"response_time"s, response_time.total_milliseconds()},
        {"code"s, response.result_int()},
        {"content_type"s, content_type}};
    BOOST_LOG_TRIVIAL(info)
        << boost::log::add_value(additional_data, custom_data)
        << "response sent"sv;
  }

  SomeRequestHandler& some_rh_;
};

}  // namespace server_logging

StringResponse MakeStringResponse(http::status status, std::string body,
                                  unsigned http_version, bool keep_alive,
                                  http::verb method,
                                  std::string_view content_type,
                                  std::string_view cache_control = ""sv,
                                  std::string_view allow = ""sv);
FileResponse MakeFileResponse(http::status status,
                              http::file_body::value_type file,
                              unsigned http_version, bool keep_alive,
                              http::verb method, std::string_view content_type);
EmptyResponse MakeEmptyResponse(http::status status, unsigned http_version,
                                bool keep_alive, http::verb method,
                                std::string_view content_type);

class ApiHandler {
 public:
  explicit ApiHandler(model::Game& game, app::Players& players,
                      app::Application& application, bool random_dog_coord,
                      bool tick_param, extra_data::Loots extra_data)
      : game_(game),
        players_(players),
        application_(application),
        random_dog_coord_(random_dog_coord),
        tick_param_(tick_param),
        extra_data_(extra_data) {}

  StringResponse HandleGetPlayersRequest(const StringRequest& req);
  StringResponse HandleJoinRequest(const StringRequest& req);
  StringResponse HandleGetMapsReq(const StringRequest& req);
  StringResponse HandleGetMapReq(const model::Map::Id& map_id,
                                 const StringRequest& req);
  StringResponse HandleGetStateReq(const StringRequest& req);
  StringResponse HandlePlayerActionReq(const StringRequest& req);
  StringResponse HandleTickReq(const StringRequest& req);
  StringResponse HandleRecordsReq(const StringRequest& req);

 private:
  std::shared_ptr<app::Player> Authorize(const StringRequest& req,
                                         StringResponse& string_response);
  std::unordered_map<std::string, std::string> ParseQueryParams(
      const StringRequest& req);
  std::pair<size_t, size_t> GetDBQueryParams(const StringRequest& req);

  template <typename Fn>
  StringResponse ExecuteAuthorized(const StringRequest& req, Fn&& action) {
    StringResponse string_response;
    auto player_ptr = Authorize(req, string_response);
    if (!player_ptr) {
      return string_response;
    }
    return action(player_ptr);
  }

  model::Game& game_;
  app::Players& players_;
  app::Application& application_;
  bool random_dog_coord_ = false;
  bool tick_param_ = false;
  extra_data::Loots extra_data_;
};

class RequestHandler : public std::enable_shared_from_this<RequestHandler> {
 public:
  using Strand = net::strand<net::io_context::executor_type>;

  explicit RequestHandler(fs::path root, Strand api_strand, model::Game& game,
                          app::Players& players, bool random_dog_coord,
                          bool tick_param, extra_data::Loots extra_data,
                          app::Application& application)
      : root_(std::move(root)),
        api_strand_(api_strand),
        game_(game),
        players_(players),
        api_handler_(game, players, application, random_dog_coord, tick_param,
                     extra_data) {}

  RequestHandler(const RequestHandler&) = delete;
  RequestHandler& operator=(const RequestHandler&) = delete;

  template <typename Body, typename Allocator, typename Send>
  void operator()(tcp::endpoint,
                  http::request<Body, http::basic_fields<Allocator>>&& req,
                  Send&& send) {
    auto version = req.version();
    auto keep_alive = req.keep_alive();

    try {
      if (IsApiRequest(req)) {
        auto handle = [self = shared_from_this(), send,
                       req = std::forward<decltype(req)>(req), version,
                       keep_alive] {
          try {
            return send(self->HandleApiRequest(req));
          } catch (...) {
            send(self->ReportServerError(req));
          }
        };
        return net::dispatch(api_strand_, handle);
      }
      // Возвращаем результат обработки запроса к файлу
      return std::visit(
          [&send](auto&& result) {
            send(std::forward<decltype(result)>(result));
          },
          HandleFileRequest(req));
    } catch (...) {
      send(ReportServerError(req));
    }
  }

 private:
  using FileRequestResult =
      std::variant<EmptyResponse, StringResponse, FileResponse>;

  FileRequestResult HandleFileRequest(const StringRequest& req);
  StringResponse HandleApiRequest(const StringRequest& req);
  StringResponse ReportServerError(const StringRequest& req);

  std::vector<std::string> ParseTarget(std::string_view target);
  std::string_view DefineMIMEType(const fs::path& path);
  bool IsSubPath(fs::path path, fs::path base);
  bool IsApiRequest(const StringRequest& req);

  fs::path GetAbsolutePath(std::string_view target);
  bool OpenFile(const fs::path& abs_path, http::file_body::value_type& file);

  fs::path root_;
  Strand api_strand_;
  model::Game& game_;
  app::Players& players_;
  ApiHandler api_handler_;
};

}  // namespace http_handler