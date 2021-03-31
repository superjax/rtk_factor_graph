#include "common/print.h"

namespace mc {

void base_print(const std::string fmt,
                const fmt::format_args& args,
                const std::experimental::source_location& location)
{
    fmt::print("{}:{} ", location.file_name(), location.line());
    fmt::vprint(fmt + "\n", fmt::format_args(args));
}

void dbg(const std::string fmt,
         const fmt::format_args& args,
         const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::DEBUG)
    {
        fmt::print(fg(fmt::color::pale_green) | fmt::emphasis::bold, "{}:{} ", location.file_name(),
                   location.line());
        fmt::vprint(fmt + "\n", args);
    }
}

void info(const std::string fmt,
          const fmt::format_args& args,
          const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::INFO)
    {
        fmt::print("{}:{} ", location.file_name(), location.line());
        fmt::vprint(fmt + "\n", fmt::format_args(args));
    }
}

void warn(const std::string fmt,
          const fmt::format_args& args,
          const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::WARN)
    {
        fmt::print(fg(fmt::color::orange) | fmt::emphasis::bold, "{}:{} ", location.file_name(),
                   location.line());
        fmt::vprint(fmt + "\n", fmt::format_args(args));
    }
}

void error(const std::string fmt,
           const fmt::format_args& args,
           const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::WARN)
    {
        fmt::print(stderr, fg(fmt::color::crimson) | fmt::emphasis::bold, "{}:{} ",
                   location.file_name(), location.line());
        fmt::vprint(stderr, fmt + "\n", fmt::format_args(args));
    }
}

void fatal(const std::string fmt,
           const fmt::format_args& args,
           const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::WARN)
    {
        fmt::print(stderr, bg(fmt::color::crimson) | fmt::emphasis::bold, "{}:{} ",
                   location.file_name(), location.line());
        fmt::vprint(stderr, bg(fmt::color::crimson) | fmt::emphasis::bold, fmt,
                    fmt::format_args(args));
        fmt::print(stderr, "\n");
    }
}

void dbg(bool condition,
         const std::string fmt,
         const fmt::format_args& args,
         const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::DEBUG)
    {
        if (condition)
        {
            dbg(fmt, args, location);
        }
    }
}

void info(bool condition,
          const std::string fmt,
          const fmt::format_args& args,
          const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::INFO)
    {
        if (condition)
        {
            info(fmt, args, location);
        }
    }
}

void warn(bool condition,
          const std::string fmt,
          const fmt::format_args& args,
          const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::WARN)
    {
        if (condition)
        {
            warn(fmt, args, location);
        }
    }
}

void error(bool condition,
           const std::string fmt,
           const fmt::format_args& args,
           const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::WARN)
    {
        if (condition)
        {
            error(fmt, args, location);
        }
    }
}

void fatal(bool condition,
           const std::string fmt,
           const fmt::format_args& args,
           const std::experimental::source_location& location)
{
    if constexpr (LOGGING_LEVEL <= LoggingLevel::WARN)
    {
        if (condition)
        {
            fatal(fmt, args, location);
        }
    }
}

}  // namespace mc
