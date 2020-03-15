#include <gtest/gtest.h>

#include "common/print.h"

namespace mc {

TEST(Print, BasePrint)
{
    base_print("hi there I'm {}, you're {}", fmt("Bob", "Jill"));
}

TEST(Print, DebugPrint)
{
    dbg("This is a {} message", fmt("debug"));
}

TEST(Print, InfoPrint)
{
    info("This is an {} message", fmt("info"));
}

TEST(Print, WarnPrint)
{
    warn("This is a {} message", fmt("warn"));
}

TEST(Print, ErrorPrint)
{
    error("This is an {} message", fmt("error"));
}

TEST(Print, FatalPrint)
{
    fatal("This is a {} message", fmt("fatal"));
}

}  // namespace mc
