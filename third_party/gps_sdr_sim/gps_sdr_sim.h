#pragma once

namespace third_party {
namespace gps_sdr_sim {

/*! \brief Structure representing GPS time */
typedef struct
{
    int week;   /*!< GPS week number (since January 1980) */
    double sec; /*!< second inside the GPS \a week */
} gpstime_t;

/*! \brief Structure repreenting UTC time */
typedef struct
{
    int y;      /*!< Calendar year */
    int m;      /*!< Calendar month */
    int d;      /*!< Calendar day */
    int hh;     /*!< Calendar hour */
    int mm;     /*!< Calendar minutes */
    double sec; /*!< Calendar seconds */
} datetime_t;

typedef struct
{
    gpstime_t g;
    double range;  // pseudorange
    double rate;
    double d;  // geometric distance
    double azel[2];
    double iono_delay;
} range_t;

typedef struct
{
    int vflg; /*!< Valid Flag */
    datetime_t t;
    gpstime_t toc; /*!< Time of Clock */
    gpstime_t toe; /*!< Time of Ephemeris */
    int iodc;      /*!< Issue of Data, Clock */
    int iode;      /*!< Isuse of Data, Ephemeris */
    double deltan; /*!< Delta-N (radians/sec) */
    double cuc;    /*!< Cuc (radians) */
    double cus;    /*!< Cus (radians) */
    double cic;    /*!< Correction to inclination cos (radians) */
    double cis;    /*!< Correction to inclination sin (radians) */
    double crc;    /*!< Correction to radius cos (meters) */
    double crs;    /*!< Correction to radius sin (meters) */
    double ecc;    /*!< e Eccentricity */
    double sqrta;  /*!< sqrt(A) (sqrt(m)) */
    double m0;     /*!< Mean anamoly (radians) */
    double omg0;   /*!< Longitude of the ascending node (radians) */
    double inc0;   /*!< Inclination (radians) */
    double aop;
    double omgdot; /*!< Omega dot (radians/s) */
    double idot;   /*!< IDOT (radians/s) */
    double af0;    /*!< Clock offset (seconds) */
    double af1;    /*!< rate (sec/sec) */
    double af2;    /*!< acceleration (sec/sec^2) */
    double tgd;    /*!< Group delay L2 bias */
    int svhlth;
    int codeL2;
    // Working variables follow
    double n;       /*!< Mean motion (Average angular velocity) */
    double sq1e2;   /*!< sqrt(1-e^2) */
    double A;       /*!< Semi-major axis */
    double omgkdot; /*!< OmegaDot-OmegaEdot */
} ephem_t;

typedef struct
{
    int enable;
    int vflg;
    double alpha0, alpha1, alpha2, alpha3;
    double beta0, beta1, beta2, beta3;
    double A0, A1;
    int dtls, tot, wnt;
    int dtlsf, dn, wnlsf;
} ionoutc_t;

/*! \brief Compute range between a satellite and the receiver
 *  \param[out] rho The computed range
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at time of receiving the signal
 *  \param[in] xyz position of the receiver
 */
void computeRange(range_t *rho,
                  //   const ephem_t &eph,
                  double pos[3],
                  double vel[3],
                  double clk[2],
                  const ionoutc_t *ionoutc,
                  const gpstime_t &g,
                  const double xyz[]);

void gps2date(const gpstime_t *g, datetime_t *t);

}  // namespace gps_sdr_sim
}  // namespace third_party
