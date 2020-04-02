#include "common/quantized_time.h"

namespace mc {

bool QuantizedTime::operator>(const UTCTime& other) const
{
    return ((*this) - other).toSec() > resolution_half;
}

bool QuantizedTime::operator>=(const UTCTime& other) const
{
    return (other - (*this)).toSec() <= resolution_half;
}

bool QuantizedTime::operator<(const UTCTime& other) const
{
    return (other - (*this)).toSec() > resolution_half;
}

bool QuantizedTime::operator<=(const UTCTime& other) const
{
    return ((*this) - other).toSec() <= resolution_half;
}

bool QuantizedTime::operator==(const UTCTime& other) const
{
    return std::abs(((*this) - other).toSec()) <= resolution_half;
}

bool QuantizedTime::operator!=(const UTCTime& other) const
{
    return std::abs(((*this) - other).toSec()) > resolution_half;
}

bool operator>(const UTCTime& t, const QuantizedTime& q)
{
    return (t - q).toSec() > q.resolution_half;
}

bool operator>=(const UTCTime& t, const QuantizedTime& q)
{
    return (q - t).toSec() <= q.resolution_half;
}

bool operator<(const UTCTime& t, const QuantizedTime& q)
{
    return (q - t).toSec() > q.resolution_half;
}

bool operator<=(const UTCTime& t, const QuantizedTime& q)
{
    return (t - q).toSec() <= q.resolution_half;
}

bool operator==(const UTCTime& t, const QuantizedTime& q)
{
    return std::abs((t - q).toSec()) <= q.resolution_half;
}

bool operator!=(const UTCTime& t, const QuantizedTime& q)
{
    return std::abs((t - q).toSec()) > q.resolution_half;
}

}  // namespace mc
