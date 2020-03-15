#include "common/check.h"

namespace mc {

void check(bool condition,
           const std::string fmt,
           const fmt::format_args& args,
           const std::experimental::source_location& location)
{
#ifndef DISABLE_CHECK
    if (!condition)
    {
        fatal(fmt, args, location);
        exit(-1);
    }
#endif
}

void weak_check(bool condition,
                const std::string fmt,
                const fmt::format_args& args,
                const std::experimental::source_location& location)
{
#ifndef DISABLE_CHECK
    if (!condition)
    {
        fatal(fmt, args, location);
    }
#endif
}

}  // namespace mc
