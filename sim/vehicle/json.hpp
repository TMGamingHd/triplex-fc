// SPDX-License-Identifier: MIT
// A small, strict JSON reader for the vehicle description files (host only, no exceptions: the simulator's tests build with -fno-exceptions). Standard JSON, plus `//` comments to the end
// of a line (a vehicle file explains its numbers). No trailing commas, no NaN or Infinity, depth limited; every error names the line and the column.
#pragma once
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace sim {

struct Json {
  enum class Type { Null, Bool, Number, String, Array, Object };
  Type type = Type::Null;
  bool boolean = false;
  double number = 0.0;
  std::string text;
  std::vector<Json> items;                           // an array's elements
  std::vector<std::pair<std::string, Json>> fields;  // an object's members, in file order
  int line = 1;
  int col = 1;

  [[nodiscard]] const Json* find(const std::string& key) const {
    for (const std::pair<std::string, Json>& f : fields) {
      if (f.first == key) {
        return &f.second;
      }
    }
    return nullptr;
  }
};

namespace detail {

class JsonParser {
 public:
  explicit JsonParser(const std::string& s) : s_(s) {}

  bool parse(Json& out, std::string& error) {
    skip();
    if (!value(out, 0)) {
      error = error_;
      return false;
    }
    skip();
    if (i_ != s_.size()) {
      fail("unexpected text after the end of the value");
      error = error_;
      return false;
    }
    return true;
  }

 private:
  static constexpr int kMaxDepth = 64;

  void fail(const std::string& what) {
    if (error_.empty()) {
      error_ = "line " + std::to_string(line_) + ", column " + std::to_string(col_) + ": " + what;
    }
  }

  [[nodiscard]] bool at_end() const { return i_ >= s_.size(); }
  [[nodiscard]] char peek() const { return at_end() ? '\0' : s_[i_]; }
  char next() {
    const char c = s_[i_++];
    if (c == '\n') {
      ++line_;
      col_ = 1;
    } else {
      ++col_;
    }
    return c;
  }

  void skip() {
    while (!at_end()) {
      const char c = peek();
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
        next();
      } else if (c == '/' && i_ + 1 < s_.size() && s_[i_ + 1] == '/') {
        while (!at_end() && peek() != '\n') {
          next();
        }
      } else {
        break;
      }
    }
  }

  bool literal(const char* word) {
    for (const char* p = word; *p != '\0'; ++p) {
      if (at_end() || peek() != *p) {
        return false;
      }
      next();
    }
    return true;
  }

  bool value(Json& out, int depth) {
    if (depth > kMaxDepth) {
      fail("nested too deeply");
      return false;
    }
    out.line = line_;
    out.col = col_;
    const char c = peek();
    if (c == '{') {
      return object(out, depth);
    }
    if (c == '[') {
      return array(out, depth);
    }
    if (c == '"') {
      out.type = Json::Type::String;
      return string(out.text);
    }
    if (c == 't' || c == 'f' || c == 'n') {
      const bool is_true = c == 't';
      const bool is_null = c == 'n';
      if (!literal(is_true ? "true" : (is_null ? "null" : "false"))) {
        fail("expected true, false or null");
        return false;
      }
      out.type = is_null ? Json::Type::Null : Json::Type::Bool;
      out.boolean = is_true;
      return true;
    }
    if (c == '-' || (c >= '0' && c <= '9')) {
      return number(out);
    }
    fail(at_end() ? "unexpected end of the file" : std::string("unexpected character '") + c + "'");
    return false;
  }

  bool number(Json& out) {
    const std::size_t start = i_;
    if (peek() == '-') {
      next();
    }
    if (peek() == '0') {
      next();
    } else if (peek() >= '1' && peek() <= '9') {
      while (peek() >= '0' && peek() <= '9') {
        next();
      }
    } else {
      fail("a number needs a digit");
      return false;
    }
    if (peek() == '.') {
      next();
      if (!(peek() >= '0' && peek() <= '9')) {
        fail("a digit must follow the decimal point");
        return false;
      }
      while (peek() >= '0' && peek() <= '9') {
        next();
      }
    }
    if (peek() == 'e' || peek() == 'E') {
      next();
      if (peek() == '+' || peek() == '-') {
        next();
      }
      if (!(peek() >= '0' && peek() <= '9')) {
        fail("a digit must follow the exponent");
        return false;
      }
      while (peek() >= '0' && peek() <= '9') {
        next();
      }
    }
    out.type = Json::Type::Number;
    out.number = std::strtod(s_.substr(start, i_ - start).c_str(), nullptr);
    if (!std::isfinite(out.number)) {
      fail("the number is too large");
      return false;
    }
    return true;
  }

  bool string(std::string& out) {
    next();  // the opening quote
    out.clear();
    while (true) {
      if (at_end()) {
        fail("the string is not closed");
        return false;
      }
      const char c = next();
      if (c == '"') {
        return true;
      }
      if (c == '\n') {
        fail("a string cannot run over a line");
        return false;
      }
      if (c != '\\') {
        out.push_back(c);
        continue;
      }
      if (at_end()) {
        fail("the string is not closed");
        return false;
      }
      const char e = next();
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          unsigned code = 0;
          for (int k = 0; k < 4; ++k) {
            if (at_end() || !std::isxdigit(static_cast<unsigned char>(peek()))) {
              fail("\\u needs four hexadecimal digits");
              return false;
            }
            const char h = next();
            code = (code * 16U) + static_cast<unsigned>(h <= '9' ? h - '0' : (h | 0x20) - 'a' + 10);
          }
          out.push_back(code < 0x80U ? static_cast<char>(code) : '?');  // names and descriptions are ASCII
          break;
        }
        default:
          fail(std::string("unknown escape \\") + e);
          return false;
      }
    }
  }

  bool array(Json& out, int depth) {
    out.type = Json::Type::Array;
    next();
    skip();
    if (peek() == ']') {
      next();
      return true;
    }
    while (true) {
      skip();
      Json item;
      if (!value(item, depth + 1)) {
        return false;
      }
      out.items.push_back(std::move(item));
      skip();
      if (peek() == ',') {
        next();
        continue;
      }
      if (peek() == ']') {
        next();
        return true;
      }
      fail(at_end() ? "unexpected end of the file: a [ is not closed" : "expected ',' or ']'");
      return false;
    }
  }

  bool object(Json& out, int depth) {
    out.type = Json::Type::Object;
    next();
    skip();
    if (peek() == '}') {
      next();
      return true;
    }
    while (true) {
      skip();
      if (peek() != '"') {
        fail("expected a quoted name");
        return false;
      }
      std::string key;
      if (!string(key)) {
        return false;
      }
      if (out.find(key) != nullptr) {
        fail("the name \"" + key + "\" appears twice in this object");
        return false;
      }
      skip();
      if (peek() != ':') {
        fail("expected ':' after the name");
        return false;
      }
      next();
      skip();
      Json v;
      if (!value(v, depth + 1)) {
        return false;
      }
      out.fields.emplace_back(std::move(key), std::move(v));
      skip();
      if (peek() == ',') {
        next();
        continue;
      }
      if (peek() == '}') {
        next();
        return true;
      }
      fail(at_end() ? "unexpected end of the file: a { is not closed" : "expected ',' or '}'");
      return false;
    }
  }

  const std::string& s_;
  std::size_t i_ = 0;
  int line_ = 1;
  int col_ = 1;
  std::string error_;
};

}  // namespace detail

// Parse `text` into `out`; on failure return false with the reason (and where) in `error`.
inline bool parse_json(const std::string& text, Json& out, std::string& error) {
  out = Json{};  // whatever it held before is gone
  detail::JsonParser p(text);
  return p.parse(out, error);
}

}  // namespace sim
