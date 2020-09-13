#include "utils/strip_string.h"

std::string strip_string(const std::string& str, const std::vector<char> remove_chars)
{
    std::string out;
    for (const auto& character : str)
    {
        bool add = true;
        for (const auto& bad_character : remove_chars)
        {
            if (bad_character == character)
            {
                add = false;
                break;
            }
        }
        if (add)
        {
            out.push_back(character);
        }
    }
    return out;
}
