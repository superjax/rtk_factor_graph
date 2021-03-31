#include "common/check.h"

#include <unistd.h>

#ifdef FANCY_BACKTRACE
#define BACKWARD_HAS_DW 1
#endif
#include "third_party/backward/backward.h"

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
        third_party::backward::StackTrace st;
        st.load_here(32);
        third_party::backward::Printer p;
        p.print(st);
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
