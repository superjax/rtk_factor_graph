#pragma once

#include "common/defs.h"
#include "common/utctime.h"

namespace mc {

class QuantizedTime : public UTCTime
{
 public:
    bool operator>(const UTCTime& other) const;
    bool operator>=(const UTCTime& other) const;
    bool operator<(const UTCTime& other) const;
    bool operator<=(const UTCTime& other) const;
    bool operator==(const UTCTime& other) const;
    bool operator!=(const UTCTime& other) const;

    QuantizedTime& operator=(const UTCTime& other);

    static constexpr double resolution_half = TIME_QUANTIZATION / 2.0;
};

bool operator>(const UTCTime& other, const QuantizedTime& q);
bool operator>=(const UTCTime& other, const QuantizedTime& q);
bool operator<(const UTCTime& other, const QuantizedTime& q);
bool operator<=(const UTCTime& other, const QuantizedTime& q);
bool operator==(const UTCTime& other, const QuantizedTime& q);
bool operator!=(const UTCTime& other, const QuantizedTime& q);

}  // namespace mc
