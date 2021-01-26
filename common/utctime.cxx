#include "common/utctime.h"

#include "common/quantized_time.h"

namespace mc {

constexpr int64_t UTCTime::E9;
constexpr int64_t UTCTime::E6;
constexpr int UTCTime::SEC_IN_WEEK;
constexpr int UTCTime::SEC_IN_DAY;
constexpr int64_t UTCTime::LEAP_SECONDS;
constexpr int UTCTime::GPS_BEIDOU_OFFSET;
constexpr int UTCTime::GPS_WEEK_ROLLOVER;
constexpr int64_t UTCTime::GPS_UTC_OFFSET;
constexpr int64_t UTCTime::GLO_UTC_OFFSET;
constexpr int UTCTime::BD_UTC_OFFSET;

UTCTime::UTCTime()
{
    sec = 0;
    nsec = 0;
}

bool UTCTime::operator>(const UTCTime& other) const
{
    return (sec > other.sec) ? true
                             : (sec < other.sec) ? false : (nsec > other.nsec) ? true : false;
}
bool UTCTime::operator>=(const UTCTime& other) const
{
    return (sec >= other.sec) ? true
                              : (sec < other.sec) ? false : (nsec >= other.nsec) ? true : false;
}
bool UTCTime::operator<(const UTCTime& other) const
{
    return (sec < other.sec) ? true
                             : (sec > other.sec) ? false : (nsec < other.nsec) ? true : false;
}
bool UTCTime::operator<=(const UTCTime& other) const
{
    return (sec <= other.sec) ? true
                              : (sec > other.sec) ? false : (nsec <= other.nsec) ? true : false;
}

bool UTCTime::operator==(const UTCTime& other) const
{
    return (sec == other.sec && nsec == other.nsec);
}

UTCTime UTCTime::operator-(const UTCTime& other) const
{
    UTCTime out;
    out.sec = sec - other.sec;
    out.nsec = nsec - other.nsec;
    out.wrapNsec();
    return out;
}

UTCTime& UTCTime::operator-=(const UTCTime& other)
{
    sec = sec - other.sec;
    nsec = nsec - other.nsec;
    wrapNsec();
    return (*this);
}

UTCTime UTCTime::operator+(const UTCTime& other) const
{
    UTCTime out;
    out.sec = sec + other.sec;
    out.nsec = nsec + other.nsec;
    out.wrapNsec();
    return out;
}

UTCTime& UTCTime::operator+=(const UTCTime& other)
{
    sec = sec + other.sec;
    nsec = nsec + other.nsec;
    wrapNsec();
    return (*this);
}

double UTCTime::toSec() const
{
    return (double)sec + (double)nsec * 1e-9;
}

UTCTime UTCTime::operator+(double sec_) const
{
    UTCTime out;
    out.nsec = nsec + (int64_t)(sec_ * E9) % E9;
    out.sec = sec + (int64_t)sec_;
    out.wrapNsec();
    return out;
}
UTCTime& UTCTime::operator+=(double sec_)
{
    nsec = nsec + (int64_t)(sec_ * E9) % E9;
    sec = sec + (int64_t)sec_;
    wrapNsec();
    return *this;
}
UTCTime UTCTime::operator+(int sec_) const
{
    UTCTime out;
    out.sec = sec + sec_;
    out.nsec = nsec;
    return out;
}
UTCTime& UTCTime::operator+=(int sec_)
{
    sec = sec + sec_;
    return *this;
}

UTCTime UTCTime::operator-(double sec_) const
{
    UTCTime out;
    out.nsec = nsec - (int64_t)(sec_ * E9) % E9;
    out.sec = sec - (int64_t)sec_;
    out.wrapNsec();
    return out;
}
UTCTime& UTCTime::operator-=(double sec_)
{
    nsec = nsec - (int64_t)(sec_ * E9) % E9;
    sec = sec - (int64_t)sec_;
    wrapNsec();
    return *this;
}
UTCTime UTCTime::operator-(int sec_) const
{
    UTCTime out;
    out.sec = sec - sec_;
    out.nsec = nsec;
    return out;
}
UTCTime& UTCTime::operator-=(int sec_)
{
    sec = sec - sec_;
    return *this;
}

UTCTime UTCTime::Random()
{
    UTCTime out;
    out.sec = rand();
    out.nsec = rand();
    return out;
}

int UTCTime::week() const
{
    return std::floor(sec / SEC_IN_WEEK);
}

int UTCTime::GpsWeek() const
{
    int64_t gps_sec = sec - GPS_UTC_OFFSET;
    return std::floor(gps_sec / SEC_IN_WEEK);
}

// int UTCTime::BeidouWeek()
//{
//    int64_t gps_sec = sec - GPS_UTC_OFFSET + GPS_BEIDOU_OFFSET;
//    return std::floor(gps_sec/SEC_IN_WEEK);
//}

int UTCTime::GlonassWeek() const
{
    int64_t glonass_sec = sec - GLO_UTC_OFFSET;
    return std::floor(glonass_sec / SEC_IN_WEEK);
}

int UTCTime::GlonassDayOfWeek() const
{
    int64_t glonass_sec = sec - GLO_UTC_OFFSET;
    int32_t sec_of_week = glonass_sec % SEC_IN_WEEK;
    int32_t day_of_week = std::floor(sec_of_week / SEC_IN_DAY);
    return day_of_week;
}

UTCTime UTCTime::now()
{
    timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    UTCTime out;
    out.sec = start.tv_sec;
    out.nsec = start.tv_nsec;
    return out;
}

UTCTime UTCTime::fromGPS(int week, int tow_ms)
{
    UTCTime out;
    out.sec = week * SEC_IN_WEEK + tow_ms / 1000 + GPS_UTC_OFFSET;
    out.nsec = (tow_ms % 1000) * E9;
    out.wrapNsec();
    return out;
}

UTCTime UTCTime::fromGlonassTimeOfDay(const UTCTime& ref, int tod_ms)
{
    uint32_t glonass_week = ref.GlonassWeek();
    uint32_t glonass_day = ref.GlonassDayOfWeek();

    int glonass_tow_ms = (glonass_day * SEC_IN_DAY) * 1000 + tod_ms;

    UTCTime out = UTCTime::fromGlonass(glonass_week, glonass_tow_ms);
    return out;
}

UTCTime UTCTime::fromGalileo(int week, int tow_ms)
{
    // Galileo and GPS use the same time system
    return fromGPS(week, tow_ms);
}

// UTCTime UTCTime::fromBeidou(int week, int tow_ms)
//{

//    int64_t new_tow = tow_ms - GPS_BEIDOU_OFFSET*1000;
//    if (new_tow < 0)
//    {
//        week -= SEC_IN_WEEK;
//        tow_ms += SEC_IN_WEEK;
//    }
//    return fromGPS(week, tow_ms);
//}

UTCTime UTCTime::fromGlonass(int week, int tow_ms)
{
    UTCTime out;
    out.sec = int64_t(week * SEC_IN_WEEK) + tow_ms / 1000 + GLO_UTC_OFFSET;
    out.nsec = (tow_ms % 1000) * E9;
    out.wrapNsec();
    return out;
}

void UTCTime::wrapNsec()
{
    if (nsec < 0)
    {
        sec -= 1;
        nsec += E9;
    }
    else if (nsec > E9)
    {
        sec += 1;
        nsec -= E9;
    }
}

UTCTime UTCTime::fromCalendar(int year, int month, int day, int hour, int minute, double sec)
{
    tm gtime;
    gtime.tm_year = year - 1900;
    gtime.tm_mon = month - 1;
    gtime.tm_mday = day;
    gtime.tm_hour = hour;
    gtime.tm_min = minute;
    gtime.tm_sec = std::floor(sec);

    time_t epoch_time = timegm(&gtime);
    int64_t nsec = std::round(1e9 * (sec - std::floor(sec)));
    return UTCTime((int64_t)epoch_time, nsec);
}
std::string UTCTime::str() const
{
    time_t time(sec);
    tm* date = gmtime(&time);

    std::stringstream ss;
    ss << 1900 + date->tm_year << "/" << 1 + date->tm_mon << "/" << date->tm_mday << " "
       << date->tm_hour << ":" << date->tm_min << ":" << date->tm_sec << "." << nsec / 1'000'000;

    return ss.str();
}

double UTCTime::GpsTow() const
{
    return ((*this) - UTCTime::fromGPS(GpsWeek(), 0)).toSec();
}

QuantizedTime UTCTime::quantized(double resolution) const
{
    QuantizedTime out;
    out.sec = sec;
    out.nsec = nsec;
    out.resolution_half = resolution / 2.0;
    return out;
}

std::ostream& operator<<(std::ostream& os, const UTCTime& t)
{
    time_t time(t.sec);
    tm* date = gmtime(&time);

    os << 1900 + date->tm_year << "/" << 1 + date->tm_mon << "/" << date->tm_mday << " "
       << date->tm_hour << ":" << date->tm_min << ":" << date->tm_sec << "." << t.nsec / 1'000'000;

    return os;
}

}  // namespace mc
