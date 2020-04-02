#pragma once

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

    double resolution_half = 0;
};

bool operator>(const UTCTime& other, const QuantizedTime& q);
bool operator>=(const UTCTime& other, const QuantizedTime& q);
bool operator<(const UTCTime& other, const QuantizedTime& q);
bool operator<=(const UTCTime& other, const QuantizedTime& q);
bool operator==(const UTCTime& other, const QuantizedTime& q);
bool operator!=(const UTCTime& other, const QuantizedTime& q);

}  // namespace mc
