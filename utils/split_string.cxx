#include <algorithm>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace mc {
namespace utils {

std::vector<std::string> split_string(const std::string& str, const char delim = ' ')
{
    std::vector<std::string> out;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim))
    {
        out.push_back(token);
    }
    return out;
}

}  // namespace utils
}  // namespace mc
