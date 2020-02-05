#pragma once

#include "common/print.h"

#ifndef DISABLE_ASSERT
#define ASSERT(condition, fmt, ...)    \
    {                                  \
        if (!(condition))              \
        {                              \
            fatal(fmt, ##__VA_ARGS__); \
            exit(-1);                  \
        }                              \
    }

#define WEAK_ASSERT(condition, fmt, ...) \
    {                                    \
        if (!(condition))                \
        {                                \
            fatal(fmt, ##__VA_ARGS__);   \
        }                                \
    }

#else
#define ASSERT(...)
#define WEAK_ASSERT(...)
#endif
