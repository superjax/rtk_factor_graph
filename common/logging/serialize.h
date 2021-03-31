#pragma once

#include <istream>
#include <ostream>

#include "utils/has_data.h"

namespace mc {

// Forward declaration for special serialize types
class UTCTime;
class QuantizedTime;
namespace ephemeris {
class GPSEphemeris;
class GlonassEphemeris;
class GalileoEphemeris;
}  // namespace ephemeris
namespace meas {
class ImuSample;
class GnssObservation;
}  // namespace meas
namespace math {
template <typename T>
class Quat;
template <typename T>
class DQuat;
template <typename T>
class Jet;
template <typename T>
class TwoJet;
}  // namespace math

namespace logging {

void serialize(std::ostream& file, const UTCTime& t);
void serialize(std::ostream& file, const QuantizedTime& t);
void serialize(std::ostream& file, const ephemeris::GPSEphemeris& eph);
void serialize(std::ostream& file, const ephemeris::GlonassEphemeris& eph);
void serialize(std::ostream& file, const ephemeris::GalileoEphemeris& eph);
void serialize(std::ostream& file, const meas::ImuSample& imu);
void serialize(std::ostream& file, const meas::GnssObservation& obs);
void serialize(std::ostream& file, const math::Quat<double>& q);
void serialize(std::ostream& file, const math::DQuat<double>& dq);
void serialize(std::ostream& file, const math::Jet<double>& x);
void serialize(std::ostream& file, const math::TwoJet<double>& x);

template <typename T>
void serialize(std::ostream& file, const T& data)
{
    if constexpr (utils::has_data<T>::value)
    {
        file.write((const char*)data.data(), sizeof(decltype(*data.data())) * data.size());
    }
    else
    {
        file.write((const char*)&data, sizeof(T));
    }
}

template <typename... T>
void log(std::ostream& file, const T&... data)
{
    int dummy[sizeof...(data)] = {(serialize(file, data), 1)...};
    (void)dummy;
}

void deserialize(std::istream& file, UTCTime& t);
void deserialize(std::istream& file, ephemeris::GPSEphemeris& eph);
void deserialize(std::istream& file, ephemeris::GlonassEphemeris& eph);
void deserialize(std::istream& file, ephemeris::GalileoEphemeris& eph);
void deserialize(std::istream& file, meas::GnssObservation& obs);
void deserialize(std::istream& file, meas::ImuSample& imu);
void deserialize(std::istream& file, math::Quat<double>& q);
void deserialize(std::istream& file, math::DQuat<double>& dq);
void deserialize(std::istream& file, math::Jet<double>& x);
void deserialize(std::istream& file, math::TwoJet<double>& x);

template <typename T>
void deserialize(std::istream& file, T& x)
{
    if constexpr (utils::has_data<T>::value)
    {
        file.read(reinterpret_cast<char*>(x.data()), sizeof(decltype(*x.data())) * x.size());
    }
    else
    {
        file.read(reinterpret_cast<char*>(&x), sizeof(T));
    }
}

template <typename... T>
void read(std::istream& file, T&... data)
{
    int dummy[sizeof...(data)] = {(deserialize(file, data), 1)...};
    (void)dummy;
}



}  // namespace logging
}  // namespace mc
