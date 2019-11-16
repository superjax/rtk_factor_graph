#include <cstdio>
#include <fstream>
#include <iostream>

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

#if defined(LOGGING_LEVEL_DEBUG)
#define LOGGING_LEVEL LoggingLevel::DEBUG;
#elif defined(LOGGING_LEVEL_INFO)
#define LOGGING_LEVEL LoggingLevel::INFO;
#elif defined(LOGGING_LEVEL_WARN)
#define LOGGING_LEVEL LoggingLevel::WARN;
#elif defined(LOGGING_LEVEL_ERROR)
#define LOGGING_LEVEL LoggingLevel::ERROR;
#elif defined(LOGGING_LEVEL_FATAL)
#define LOGGING_LEVEL LoggingLevel::FATAL;
#else
#define LOGGING_LEVEL LoggingLevel::DEBUG
#endif

std::fstream nullstream("/dev/null", std::ofstream::out | std::ofstream::app);

#define print(level, fmt, ...)                                        \
    if ((level) >= LOGGING_LEVEL)                                     \
    {                                                                 \
        printf("%s:%d " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    }

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

#define printRed(fmt, ...) printf(ANSI_COLOR_RED fmt ANSI_COLOR_RESET, ##__VA_ARGS__)
#define printGreen(fmt, ...) printf(ANSI_COLOR_GREEN fmt ANSI_COLOR_RESET, ##__VA_ARGS__)
#define printYellow(fmt, ...) printf(ANSI_COLOR_YELLOW fmt ANSI_COLOR_RESET, ##__VA_ARGS__)
#define printBlue(fmt, ...) printf(ANSI_COLOR_BLUE fmt ANSI_COLOR_RESET, ##__VA_ARGS__)
#define printMagenta(fmt, ...) printf(ANSI_COLOR_MAGENTA fmt ANSI_COLOR_RESET, ##__VA_ARGS__)
#define printCyan(fmt, ...) printf(ANSI_COLOR_CYAN fmt ANSI_COLOR_RESET, ##__VA_ARGS__)

#define dbg(fmt, ...)                                                    \
    if constexpr (LoggingLevel::DEBUG >= LOGGING_LEVEL)                  \
    {                                                                    \
        printCyan("%s:%d " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    }

#define info(fmt, ...)                                                \
    if constexpr (LoggingLevel::INFO >= LOGGING_LEVEL)                \
    {                                                                 \
        printf("%s:%d " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    }

#define warn(fmt, ...)                                                     \
    if constexpr (LoggingLevel::WARN >= LOGGING_LEVEL)                     \
    {                                                                      \
        printYellow("%s:%d " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    }

#define err(fmt, ...)                                                   \
    if constexpr (LoggingLevel::ERROR >= LOGGING_LEVEL)                 \
    {                                                                   \
        printRed("%s:%d " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    }

#define fatal(fmt, ...)                                                     \
    if constexpr (LoggingLevel::FATAL >= LOGGING_LEVEL)                     \
    {                                                                       \
        printMagenta("%s:%d " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    }

class RedStream : public std::ostream
{
    RedStream() { std::cout << ANSI_COLOR_RED; }
    ~RedStream() { std::cout << ANSI_COLOR_RESET; }

    template <typename T>
    RedStream& operator<<(const T& s)
    {
        std::cout << s;
        return *this;
    }
}

std::ostream&
LOG(const LoggingLevel& level)
{
    if (level >= LOGGING_LEVEL)
    {
        return std::cout;
    }
    else
    {
        return nullstream;
    }
}
