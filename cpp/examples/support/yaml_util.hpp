#pragma once

#include <cctype>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "parse_util.hpp"

namespace robot_session_examples {

inline bool FileExists(const std::string& path) {
  std::ifstream in(path);
  return static_cast<bool>(in);
}

inline bool ParseYamlBool(const YAML::Node& node, bool* out) {
  if (!node || !node.IsScalar() || out == nullptr) {
    return false;
  }
  std::string text = node.Scalar();
  for (char& ch : text) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (text == "true") {
    *out = true;
    return true;
  }
  if (text == "false") {
    *out = false;
    return true;
  }
  return false;
}

inline void FlattenDoubles(const YAML::Node& node, std::vector<double>* out) {
  if (!node || node.IsNull()) {
    return;
  }
  if (node.IsScalar()) {
    double value = 0.0;
    if (!ParseFiniteDouble(node.Scalar(), &value)) {
      throw std::runtime_error("expected a finite number, got '" + node.Scalar() + "'");
    }
    out->push_back(value);
    return;
  }
  if (!node.IsSequence()) {
    throw std::runtime_error("expected YAML sequence or scalar of doubles");
  }
  for (const auto& child : node) {
    FlattenDoubles(child, out);
  }
}

inline void FlattenInts(const YAML::Node& node, std::vector<int32_t>* out) {
  if (!node || node.IsNull()) {
    return;
  }
  if (node.IsScalar()) {
    int value = 0;
    if (!ParseStrictInt(node.Scalar(), &value)) {
      throw std::runtime_error("expected an integer, got '" + node.Scalar() + "'");
    }
    out->push_back(static_cast<int32_t>(value));
    return;
  }
  if (!node.IsSequence()) {
    throw std::runtime_error("expected YAML sequence or scalar of ints");
  }
  for (const auto& child : node) {
    FlattenInts(child, out);
  }
}

}  // namespace robot_session_examples
