#include "request_handler.h"

#include <sstream>
#include <stdexcept>

namespace http_handler {

std::string DecodeURL(const fs::path& encoded_url) {
  std::string encoded_url_str = encoded_url.string();
  std::string decoded_url;
  for (auto it_begin = encoded_url_str.begin(), it_end = encoded_url_str.end();
       it_begin < it_end; ++it_begin) {
    auto ch = (*it_begin);
    switch (ch) {
      case '%':
        if (it_begin[1] && it_begin[2]) {
          const char hs[]{it_begin[1], it_begin[2]};
          decoded_url += static_cast<char>(std::strtol(hs, nullptr, 16));
          it_begin += 2;
        }
        break;
      case '+':
        decoded_url += ' ';
        break;
      default:
        decoded_url += ch;
    }
  }
  return decoded_url;
}

StringResponse MakeStringResponse(http::status status, std::string body,
                                  unsigned http_version, bool keep_alive,
                                  http::verb method,
                                  std::string_view content_type,
                                  std::string_view cache_control,
                                  std::string_view allow) {
  StringResponse response(status, http_version);
  response.set(http::field::content_type, content_type);
  if (!cache_control.empty()) {
    response.set(http::field::cache_control, cache_control);
  }
  if (!allow.empty()) {
    response.set(http::field::allow, allow);
  }
  response.body() = body;
  response.content_length(body.size());
  response.keep_alive(keep_alive);
  return response;
}

FileResponse MakeFileResponse(http::status status,
                              http::file_body::value_type file,
                              unsigned http_version, bool keep_alive,
                              http::verb method,
                              std::string_view content_type) {
  FileResponse response(status, http_version);
  response.set(http::field::content_type, content_type);
  response.body() = std::move(file);
  response.keep_alive(keep_alive);
  response.prepare_payload();
  return response;
}

EmptyResponse MakeEmptyResponse(http::status status, unsigned http_version,
                                bool keep_alive, http::verb method,
                                std::string_view content_type) {
  EmptyResponse response(status, http_version);
  response.set(http::field::content_type, content_type);
  response.keep_alive(keep_alive);
  return response;
}

std::shared_ptr<app::Player> ApiHandler::Authorize(
    const StringRequest& req, StringResponse& string_response) {
  std::string_view auth_token = req[http::field::authorization];
  if (auth_token.empty() || !auth_token.starts_with("Bearer ") ||
      auth_token.size() != 39) {
    string_response = MakeStringResponse(
        http::status::unauthorized,
        app::BuildError("invalidToken", "Authorization header is missing"),
        req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON,
        "no-cache"sv);
    return nullptr;
  }
  std::string token = std::string(auth_token.substr(7));
  auto player_ptr = players_.FindPlayerByToken(app::Player::Token(token));
  if (!player_ptr) {
    string_response = MakeStringResponse(
        http::status::unauthorized,
        app::BuildError("unknownToken", "Player token has not been found"),
        req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON,
        "no-cache"sv);
    return nullptr;
  }
  return player_ptr;
}

std::unordered_map<std::string, std::string> ApiHandler::ParseQueryParams(
    const StringRequest& req) {
  std::unordered_map<std::string, std::string> query_params;
  auto url = req.target();
  auto query_start = url.find('?');
  if (query_start == std::string::npos) {
    return query_params;
  }
  std::string query_string{url.substr(query_start + 1)};
  std::istringstream query_stream(query_string);
  std::string param;
  while (std::getline(query_stream, param, '&')) {
    auto eq_pos = param.find('=');
    if (eq_pos != std::string::npos) {
      auto key = param.substr(0, eq_pos);
      auto value = param.substr(eq_pos + 1);
      query_params[key] = value;
    } else {
      query_params[param] = "";
    }
  }
  return query_params;
}

std::pair<size_t, size_t> ApiHandler::GetDBQueryParams(
    const StringRequest& req) {
  size_t start = database::DEFAULT_OFFSET;
  size_t limit = database::DEFAULT_LIMIT;
  auto query_params = ParseQueryParams(req);
  if (query_params.find("start") != query_params.end()) {
    start = static_cast<size_t>(std::stoull(query_params.at("start")));
  }
  if (query_params.find("maxItems") != query_params.end()) {
    limit = static_cast<size_t>(std::stoull(query_params.at("maxItems")));
  }
  return {start, limit};
}

StringResponse ApiHandler::HandleGetPlayersRequest(const StringRequest& req) {
  if (req.method() != http::verb::get && req.method() != http::verb::head) {
    return MakeStringResponse(
        http::status::method_not_allowed,
        app::BuildError("invalidMethod", "Invalid method"), req.version(),
        req.keep_alive(), req.method(), ContentType::APP_JSON, "no-cache"sv,
        "GET, HEAD"sv);
  }
  return ExecuteAuthorized(
      req, [this, &req](const std::shared_ptr<app::Player> player_ptr) {
        auto response = application_.ListPlayers(player_ptr);
        return MakeStringResponse(http::status::ok, response, req.version(),
                                  req.keep_alive(), req.method(),
                                  ContentType::APP_JSON, "no-cache"sv);
      });
}

StringResponse ApiHandler::HandleJoinRequest(const StringRequest& req) {
  if (req.method() != http::verb::post) {
    return MakeStringResponse(
        http::status::method_not_allowed,
        app::BuildError("invalidMethod", "Only POST method is expected"),
        req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON,
        "no-cache"sv, "POST"sv);
  }
  std::string name;
  std::string map_id;
  try {
    boost::json::value value = json::parse(req.body());
    name = json::value_to<std::string>(value.at("userName"));
    map_id = json::value_to<std::string>(value.at("mapId"));
  } catch (const std::exception& e) {
    return MakeStringResponse(http::status::bad_request,
                              app::BuildError("invalidArgument", e.what()),
                              req.version(), req.keep_alive(), req.method(),
                              ContentType::APP_JSON, "no-cache"sv);
  }
  auto response =
      application_.JoinGame(model::Map::Id{map_id}, name, random_dog_coord_);
  return MakeStringResponse(http::status::ok, response, req.version(),
                            req.keep_alive(), req.method(),
                            ContentType::APP_JSON, "no-cache"sv);
}

StringResponse ApiHandler::HandleGetMapsReq(const StringRequest& req) {
  auto response = application_.ListMaps();
  return MakeStringResponse(http::status::ok, response, req.version(),
                            req.keep_alive(), req.method(),
                            ContentType::APP_JSON, "no-cache"sv);
}

StringResponse ApiHandler::HandleGetMapReq(const model::Map::Id& map_id,
                                           const StringRequest& req) {
  if (req.method() != http::verb::get && req.method() != http::verb::head) {
    return MakeStringResponse(
        http::status::method_not_allowed,
        app::BuildError("invalidMethod", "Invalid method"), req.version(),
        req.keep_alive(), req.method(), ContentType::APP_JSON, "no-cache"sv,
        "GET, HEAD"sv);
  }
  std::optional<json::object> responce_obj = application_.FindMap(map_id);
  if (!responce_obj.has_value()) {
    return MakeStringResponse(http::status::not_found,
                              app::BuildError("mapNotFound", "Map not found"),
                              req.version(), req.keep_alive(), req.method(),
                              ContentType::APP_JSON, "no-cache"sv);
  }
  std::string responce_str =
      extra_data_.GetMapLootResponce(map_id.GetValue(), responce_obj.value());
  return MakeStringResponse(http::status::ok, responce_str, req.version(),
                            req.keep_alive(), req.method(),
                            ContentType::APP_JSON, "no-cache"sv);
}

StringResponse ApiHandler::HandleGetStateReq(const StringRequest& req) {
  if (req.method() != http::verb::get && req.method() != http::verb::head) {
    return MakeStringResponse(
        http::status::method_not_allowed,
        app::BuildError("invalidMethod", "Invalid method"), req.version(),
        req.keep_alive(), req.method(), ContentType::APP_JSON, "no-cache"sv,
        "GET, HEAD"sv);
  }
  return ExecuteAuthorized(
      req, [this, &req](const std::shared_ptr<app::Player> player_ptr) {
        auto response = application_.ListGameState(player_ptr);
        return MakeStringResponse(http::status::ok, response, req.version(),
                                  req.keep_alive(), req.method(),
                                  ContentType::APP_JSON, "no-cache"sv);
      });
}

StringResponse ApiHandler::HandlePlayerActionReq(const StringRequest& req) {
  if (req.method() != http::verb::post) {
    return MakeStringResponse(
        http::status::method_not_allowed,
        app::BuildError("invalidMethod", "Only POST method is expected"),
        req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON,
        "no-cache"sv, "POST"sv);
  }
  if (!req.has_content_length() ||
      req.base().at(http::field::content_type) != "application/json") {
    return MakeStringResponse(
        http::status::bad_request,
        app::BuildError("invalidArgument", "Invalid content type"),
        req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON,
        "no-cache"sv, "POST"sv);
  }
  return ExecuteAuthorized(
      req, [this, &req](const std::shared_ptr<app::Player> player_ptr) {
        std::pair<double, double> dog_speed;
        model::Direction direction = model::Direction::NORTH;
        std::string move;
        bool is_active = true;
        try {
          boost::json::value value = json::parse(req.body());
          move = json::value_to<std::string>(value.at("move"));
        } catch (const std::exception& e) {
          return MakeStringResponse(
              http::status::bad_request,
              app::BuildError("invalidArgument", e.what()), req.version(),
              req.keep_alive(), req.method(), ContentType::APP_JSON,
              "no-cache"sv);
        }
        if (move == "L") {
          dog_speed = {-1.0, 0.0};
          direction = model::Direction::WEST;
        } else if (move == "R") {
          dog_speed = {1.0, 0.0};
          direction = model::Direction::EAST;
        } else if (move == "U") {
          dog_speed = {0.0, -1.0};
          direction = model::Direction::NORTH;
        } else if (move == "D") {
          dog_speed = {0.0, 1.0};
          direction = model::Direction::SOUTH;
        } else if (move == "") {
          dog_speed = {0.0, 0.0};
          is_active = false;
        }
        application_.ModifyPlayer(player_ptr, dog_speed, direction, is_active);
        return MakeStringResponse(http::status::ok, R"({})", req.version(),
                                  req.keep_alive(), req.method(),
                                  ContentType::APP_JSON, "no-cache"sv);
      });
}

StringResponse ApiHandler::HandleTickReq(const StringRequest& req) {
  if (tick_param_) {
    return MakeStringResponse(
        http::status::bad_request,
        app::BuildError("invalidArgument", "Invalid endpoint"), req.version(),
        req.keep_alive(), req.method(), ContentType::APP_JSON, "no-cache"sv);
  }
  if (req.method() != http::verb::post) {
    return MakeStringResponse(
        http::status::method_not_allowed,
        app::BuildError("invalidMethod", "Only POST method is expected"),
        req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON,
        "no-cache"sv, "POST"sv);
  }
  uint64_t time_delta = 0;
  try {
    boost::json::value value = json::parse(req.body());
    time_delta = json::value_to<uint64_t>(value.at("timeDelta"));
  } catch (const std::exception& e) {
    return MakeStringResponse(http::status::bad_request,
                              app::BuildError("invalidArgument", e.what()),
                              req.version(), req.keep_alive(), req.method(),
                              ContentType::APP_JSON, "no-cache"sv);
  }
  if (time_delta == 0) {
    return MakeStringResponse(
        http::status::bad_request,
        app::BuildError("invalidArgument", "Invalid time delta: null"),
        req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON,
        "no-cache"sv);
  }
  application_.Tick(time_delta);
  return MakeStringResponse(http::status::ok, R"({})", req.version(),
                            req.keep_alive(), req.method(),
                            ContentType::APP_JSON, "no-cache"sv);
}

StringResponse ApiHandler::HandleRecordsReq(const StringRequest& req) {
  if (req.method() != http::verb::get && req.method() != http::verb::head) {
    return MakeStringResponse(
        http::status::method_not_allowed,
        app::BuildError("invalidMethod", "Invalid method"), req.version(),
        req.keep_alive(), req.method(), ContentType::APP_JSON, "no-cache"sv,
        "GET, HEAD"sv);
  }
  auto db_query_params = GetDBQueryParams(req);
  if (db_query_params.second > database::DEFAULT_LIMIT) {
    return (MakeStringResponse(
        http::status::bad_request,
        app::BuildError("Invalid Argument", "Limit error"), req.version(),
        req.keep_alive(), req.method(), ContentType::APP_JSON, "no-cache"sv));
  }
  auto response = application_.GetPlayersRecords(db_query_params.first,
                                                 db_query_params.second);
  return MakeStringResponse(http::status::ok, response, req.version(),
                            req.keep_alive(), req.method(),
                            ContentType::APP_JSON, "no-cache"sv);
}

RequestHandler::FileRequestResult RequestHandler::HandleFileRequest(
    const StringRequest& req) {
  fs::path abs_path = GetAbsolutePath(req.target());
  if (!IsSubPath(abs_path, root_)) {
    return MakeStringResponse(
        http::status::bad_request,
        app::BuildError("badRequest",
                        "Absolute path isn't a subpath of the root directory"),
        req.version(), req.keep_alive(), req.method(), ContentType::TEXT_PLAIN);
  }
  http::file_body::value_type file;
  if (!OpenFile(abs_path, file)) {
    return MakeStringResponse(http::status::not_found,
                              app::BuildError("notFound", "Source not found"),
                              req.version(), req.keep_alive(), req.method(),
                              ContentType::TEXT_PLAIN);
  }
  std::string_view content_type = DefineMIMEType(abs_path);
  if (req.method() == http::verb::get) {
    return MakeFileResponse(http::status::ok, std::move(file), req.version(),
                            req.keep_alive(), req.method(), content_type);
  } else if (req.method() == http::verb::head) {
    return MakeEmptyResponse(http::status::ok, req.version(), req.keep_alive(),
                             req.method(), content_type);
  }
  return MakeStringResponse(
      http::status::bad_request, app::BuildError("badRequest", "Bad request"),
      req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON);
}

StringResponse RequestHandler::HandleApiRequest(const StringRequest& req) {
  auto parsed_target = ParseTarget(req.target());
  if (parsed_target[2] == "v1" && parsed_target[3] == "maps") {
    if (parsed_target.size() == 4) {
      return api_handler_.HandleGetMapsReq(req);
    } else if (parsed_target.size() == 5) {
      return api_handler_.HandleGetMapReq(
          model::Map::Id({parsed_target[4].begin(), parsed_target[4].end()}),
          req);
    }
    return MakeStringResponse(
        http::status::bad_request, app::BuildError("badRequest", "Bad request"),
        req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON);
  } else if (parsed_target[2] == "v1" && parsed_target[3] == "game") {
    if (parsed_target[4] == "join") {
      return api_handler_.HandleJoinRequest(req);
    } else if (parsed_target[4] == "players") {
      return api_handler_.HandleGetPlayersRequest(req);
    } else if (parsed_target[4] == "state") {
      return api_handler_.HandleGetStateReq(req);
    } else if (parsed_target[4] == "player") {
      if (parsed_target[5] == "action") {
        return api_handler_.HandlePlayerActionReq(req);
      }
    } else if (parsed_target[4] == "tick") {
      return api_handler_.HandleTickReq(req);
    } else if (parsed_target[4].starts_with("records")) {
      return api_handler_.HandleRecordsReq(req);
    }
  }
  return MakeStringResponse(
      http::status::bad_request, app::BuildError("badRequest", "Bad request"),
      req.version(), req.keep_alive(), req.method(), ContentType::APP_JSON);
}

StringResponse RequestHandler::ReportServerError(const StringRequest& req) {
  return MakeStringResponse(
      http::status::bad_request, app::BuildError("badRequest", "Bad request"),
      req.version(), req.keep_alive(), req.method(), ContentType::TEXT_HTML);
}

std::vector<std::string> RequestHandler::ParseTarget(std::string_view target) {
  std::vector<std::string> parsed_target;
  boost::split(parsed_target, target, boost::is_any_of("/"));
  return parsed_target;
}

std::string_view RequestHandler::DefineMIMEType(const fs::path& path) {
  using beast::iequals;
  std::string file_extension = path.extension().string();
  std::transform(file_extension.begin(), file_extension.end(),
                 file_extension.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (iequals(file_extension, ".htm") || iequals(file_extension, ".html")) {
    return ContentType::TEXT_HTML;
  }
  if (iequals(file_extension, ".css")) {
    return ContentType::TEXT_CSS;
  }
  if (iequals(file_extension, ".txt")) {
    return ContentType::TEXT_PLAIN;
  }
  if (iequals(file_extension, ".js")) {
    return ContentType::TEXT_JS;
  }
  if (iequals(file_extension, ".json")) {
    return ContentType::APP_JSON;
  }
  if (iequals(file_extension, ".xml")) {
    return ContentType::APP_XML;
  }
  if (iequals(file_extension, ".png")) {
    return ContentType::IMG_PNG;
  }
  if (iequals(file_extension, ".jpg") || iequals(file_extension, ".jpe") ||
      iequals(file_extension, ".jpeg")) {
    return ContentType::IMG_JPEG;
  }
  if (iequals(file_extension, ".gif")) {
    return ContentType::IMG_GIF;
  }
  if (iequals(file_extension, ".bmp")) {
    return ContentType::IMG_BMP;
  }
  if (iequals(file_extension, ".ico")) {
    return ContentType::IMG_VMI;
  }
  if (iequals(file_extension, ".tiff") || iequals(file_extension, ".tif")) {
    return ContentType::IMG_TIFF;
  }
  if (iequals(file_extension, ".svg") || iequals(file_extension, ".svgz")) {
    return ContentType::IMG_SVG_XML;
  }
  return ContentType::APP_OS;
}

bool RequestHandler::IsSubPath(fs::path path, fs::path base) {
  path = fs::weakly_canonical(path);
  base = fs::weakly_canonical(base);
  for (auto b = base.begin(), p = path.begin(); b != base.end(); ++b, ++p) {
    if (p == path.end() || *p != *b) {
      return false;
    }
  }
  return true;
}

bool RequestHandler::IsApiRequest(const StringRequest& req) {
  return req.target().starts_with("/api");
}

fs::path RequestHandler::GetAbsolutePath(std::string_view target) {
  fs::path req_path = fs::path(target);
  fs::path file_path = fs::path(root_.string() + "/" + req_path.string());
  if (fs::is_directory(file_path)) {
    file_path = file_path / "index.html";
  }
  return file_path;
}

bool RequestHandler::OpenFile(const fs::path& abs_path,
                              http::file_body::value_type& file) {
  sys::error_code ec;
  file.open(abs_path.string().c_str(), beast::file_mode::read, ec);
  if (ec) {
    // throw std::runtime_error("Failed to open file: " + abs_path.string());
    return false;
  }
  return true;
}

}  // namespace http_handler
