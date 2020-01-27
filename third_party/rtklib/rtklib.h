#pragma once

#include <time.h>

namespace third_party {
namespace rtklib {

#ifdef WIN32
typedef struct
{                /* time struct */
    time_t time; /* time (s) expressed by standard time_t */
    double sec;  /* fraction of second under 1 s */
} gtime_t;
#else
typedef struct
{                                           /* time struct */
    time_t time;                            /* time (s) expressed by standard time_t */
    __attribute__((aligned(8))) double sec; /* fraction of second under 1 s */
} gtime_t;
#endif /*WIN32*/

typedef struct
{                          /* GPS/QZS/GAL broadcast ephemeris type */
    int sat;               /* satellite number */
    int iode, iodc;        /* IODE,IODC */
    int sva;               /* SV accuracy (URA index) */
    int svh;               /* SV health (0:ok) */
    int week;              /* GPS/QZS: gps week, GAL: galileo week */
    int code;              /* GPS/QZS: code on L2 */
                           /* GAL: data source defined as rinex 3.03 */
                           /* BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
    int flag;              /* GPS/QZS: L2 P data flag */
                           /* BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO) */
    gtime_t toe, toc, ttr; /* Toe,Toc,T_trans */
                           /* SV orbit parameters */
    double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
    double crc, crs, cuc, cus, cic, cis;
    double toes;       /* Toe (s) in week */
    double fit;        /* fit interval (h) */
    double f0, f1, f2; /* SV clock parameters (af0,af1,af2) */
    double tgd[4];     /* group delay parameters */
                       /* GPS/QZS:tgd[0]=TGD */
                       /* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
                       /* CMP    :tgd[0]=BGD1,tgd[1]=BGD2 */
    double Adot, ndot; /* Adot,ndot for CNAV */
} eph_t;

typedef struct
{                      /* GLONASS broadcast ephemeris type */
    int sat;           /* satellite number */
    int iode;          /* IODE (0-6 bit of tb field) */
    int frq;           /* satellite frequency number */
    int svh, sva, age; /* satellite health, accuracy, age of operation */
    gtime_t toe;       /* epoch of epherides (gpst) */
    gtime_t tof;       /* message frame time (gpst) */
    double pos[3];     /* satellite position (ecef) (m) */
    double vel[3];     /* satellite velocity (ecef) (m/s) */
    double acc[3];     /* satellite acceleration (ecef) (m/s^2) */
    double taun, gamn; /* SV clock bias (s)/relative freq bias */
    double dtaun;      /* delay between L1 and L2 (s) */
} geph_t;

/* decode Galileo I/NAV ephemeris ----------------------------------------------
 * decode Galileo I/NAV (ref [5] 4.3)
 * args   : unsigned char *buff I Galileo I/NAV subframe bits
 *                                  buff[ 0-15]: I/NAV word type 0 (128 bit)
 *                                  buff[16-31]: I/NAV word type 1
 *                                  buff[32-47]: I/NAV word type 2
 *                                  buff[48-63]: I/NAV word type 3
 *                                  buff[64-79]: I/NAV word type 4
 *                                  buff[80-95]: I/NAV word type 5
 *          eph_t    *eph    IO  ephemeris structure
 * return : status (1:ok,0:error)
 *-----------------------------------------------------------------------------*/
extern int decode_gal_inav(const unsigned char *buff, eph_t *eph);

/* broadcast ephemeris to satellite position and clock bias --------------------
 * compute satellite position and clock bias with broadcast ephemeris (gps,
 * galileo, qzss)
 * args   : gtime_t time     I   time (gpst)
 *          eph_t *eph       I   broadcast ephemeris
 *          double *rs       O   satellite position (ecef) {x,y,z} (m)
 *          double *dts      O   satellite clock bias (s)
 *          double *var      O   satellite position and clock variance (m^2)
 * return : none
 * notes  : see ref [1],[7],[8]
 *          satellite clock includes relativity correction without code bias
 *          (tgd or bgd)
 *-----------------------------------------------------------------------------*/
extern void eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts, double *var);

/* glonass ephemeris to satellite position and clock bias ----------------------
 * compute satellite position and clock bias with glonass ephemeris
 * args   : gtime_t time     I   time (gpst)
 *          geph_t *geph     I   glonass ephemeris
 *          double *rs       O   satellite position {x,y,z} (ecef) (m)
 *          double *dts      O   satellite clock bias (s)
 *          double *var      O   satellite position and clock variance (m^2)
 * return : none
 * notes  : see ref [2]
 *-----------------------------------------------------------------------------*/
extern void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts, double *var);

}  // namespace rtklib
}  // namespace third_party
