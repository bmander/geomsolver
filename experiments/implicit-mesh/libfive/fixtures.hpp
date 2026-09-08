#pragma once
#include <string>
#include <utility>
#include <vector>
#include <libfive/tree/tree.hpp>

extern const std::vector<std::string> names;
std::pair<libfive::Tree, double> fixture(const std::string& name);
