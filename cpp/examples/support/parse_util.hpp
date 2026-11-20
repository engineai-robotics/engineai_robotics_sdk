#pragma once

#include <charconv>
#include <cmath>
#include <string>
#include <system_error>

namespace robot_session_examples {

// The whole token must be a base-10 integer. "12abc" and "abc" are rejected.
inline bool ParseStrictInt(const std::string& text, int* out) {
  if (text.empty() || out == nullptr) {
    return false;
  }
  int value = 0;
  const char* first = text.data();
  const char* last = first + text.size();
  const auto parsed = std::from_chars(first, last, value);
  if (parsed.ec != std::errc{} || parsed.ptr != last) {
    return false;
  }
  *out = value;
  return true;
}

// The whole token must be a finite decimal. "nan", "inf", and "1abc" are rejected.
inline bool ParseFiniteDouble(const std::string& text, double* out) {
  if (text.empty() || out == nullptr) {
    return false;
  }
  double value = 0.0;
  const char* first = text.data();
  const char* last = first + text.size();
  const auto parsed = std::from_chars(first, last, value);
  if (parsed.ec != std::errc{} || parsed.ptr != last || !std::isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

}  // namespace robot_session_examples
