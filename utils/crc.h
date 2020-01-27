#pragma once

#include <cstdint>

namespace mc {
namespace utils {

// Calculate Qualcomm 24 - bit Cyclical Redundancy Check(CRC - 24Q).
// With the polynomial
// x²⁴ + x²³ + x¹⁸ + x¹⁷ + x¹⁴ + x¹¹ + x¹⁰ + x⁷ + x⁶ + x⁵ + x⁴ + x³ + x + 1
uint32_t crc24(const unsigned char *buf, int len);

// Check the Hamming code of a Glonass message
// @returns true if hamming code check passes
bool gloTest(const uint8_t *buf);

}  // namespace utils
}  // namespace mc
