#pragma once

#include <fmt/color.h>
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ostream.h>

#ifdef __clang_analyzer__
#define __builtin_FILE() __FILE__
#define __builtin_FUNCTION() __FUNCTION__
#define __builtin_LINE() __LINE__
#endif
#include <experimental/source_location>

namespace mc {

enum class LoggingLevel
{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

inline constexpr bool operator>=(LoggingLevel a, LoggingLevel b)
{
    return static_cast<uint8_t>(a) >= static_cast<uint8_t>(b);
}

#ifndef LOG_LEVEL
#define LOG_LEVEL LoggingLevel::DEBUG
#endif

constexpr LoggingLevel LOGGING_LEVEL = static_cast<LoggingLevel>(LOG_LEVEL);

template <typename Context = fmt::format_context, typename... Args>
inline fmt::format_arg_store<Context, Args...> fmt(const Args&... args)
{
    return {args...};
}

//clang-format off
#define watch(x) dbg(#x "= {}", fmt(x))
//clang-format on

void base_print(const std::string fmt,
                const fmt::format_args& args,
                const std::experimental::source_location& location =
                    std::experimental::source_location::current());

void dbg(const std::string fmt,
         const fmt::format_args& args = fmt::format_args(),
         const std::experimental::source_location& location =
             std::experimental::source_location::current());

void info(const std::string fmt,
          const fmt::format_args& args = fmt::format_args(),
          const std::experimental::source_location& location =
              std::experimental::source_location::current());

void warn(const std::string fmt,
          const fmt::format_args& args = fmt::format_args(),
          const std::experimental::source_location& location =
              std::experimental::source_location::current());

void error(const std::string fmt,
           const fmt::format_args& args = fmt::format_args(),
           const std::experimental::source_location& location =
               std::experimental::source_location::current());

void fatal(const std::string fmt,
           const fmt::format_args& args = fmt::format_args(),
           const std::experimental::source_location& location =
               std::experimental::source_location::current());

void dbg(bool condition,
         const std::string fmt,
         const fmt::format_args& args = fmt::format_args(),
         const std::experimental::source_location& location =
             std::experimental::source_location::current());

void info(bool condition,
          const std::string fmt,
          const fmt::format_args& args = fmt::format_args(),
          const std::experimental::source_location& location =
              std::experimental::source_location::current());

void warn(bool condition,
          const std::string fmt,
          const fmt::format_args& args = fmt::format_args(),
          const std::experimental::source_location& location =
              std::experimental::source_location::current());

void error(bool condition,
           const std::string fmt,
           const fmt::format_args& args = fmt::format_args(),
           const std::experimental::source_location& location =
               std::experimental::source_location::current());

void fatal(bool condition,
           const std::string fmt,
           const fmt::format_args& args = fmt::format_args(),
           const std::experimental::source_location& location =
               std::experimental::source_location::current());

}  // namespace mc
