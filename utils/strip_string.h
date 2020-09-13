#pragma once

#include <string>
#include <vector>

std::string strip_string(const std::string& str, const std::vector<char> remove_chars = {' '});
