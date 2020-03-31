#pragma once

#include "common/print.h"

namespace mc {

void check(bool condition,
           const std::string fmt = "",
           const fmt::format_args& args = fmt::format_args(),
           const std::experimental::source_location& location =
               std::experimental::source_location::current());

void weak_check(bool condition,
                const std::string fmt = "",
                const fmt::format_args& args = fmt::format_args(),
                const std::experimental::source_location& location =
                    std::experimental::source_location::current());

}  // namespace mc
