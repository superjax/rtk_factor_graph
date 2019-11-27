// clang-format off

#include "rtklib.h"

#include <cstdio>
#include <cmath>

namespace rtklib
{

#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */
#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606881E-20 /* 2^-66 for BeiDou ephemeris */

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#ifdef ENAGLO
#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   27                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
#else
#define MINPRNGLO   0
#define MAXPRNGLO   0
#define NSATGLO     0
#define NSYSGLO     0
#endif
#ifdef ENAGAL
#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL   36                  /* max satellite PRN number of Galileo */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define NSYSGAL     1
#else
#define MINPRNGAL   0
#define MAXPRNGAL   0
#define NSATGAL     0
#define NSYSGAL     0
#endif
#ifdef ENAQZS
#define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
#define MAXPRNQZS   202                 /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS SAIF */
#define MAXPRNQZS_S 191                 /* max satellite PRN number of QZSS SAIF */
#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
#define NSYSQZS     1
#else
#define MINPRNQZS   0
#define MAXPRNQZS   0
#define MINPRNQZS_S 0
#define MAXPRNQZS_S 0
#define NSATQZS     0
#define NSYSQZS     0
#endif
#ifdef ENACMP
#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP   37                  /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP     1
#else
#define MINPRNCMP   0
#define MAXPRNCMP   0
#define NSATCMP     0
#define NSYSCMP     0
#endif
#ifdef ENAIRN
#define MINPRNIRN   1                   /* min satellite sat number of IRNSS */
#define MAXPRNIRN   7                   /* max satellite sat number of IRNSS */
#define NSATIRN     (MAXPRNIRN-MINPRNIRN+1) /* number of IRNSS satellites */
#define NSYSIRN     1
#else
#define MINPRNIRN   0
#define MAXPRNIRN   0
#define NSATIRN     0
#define NSYSIRN     0
#endif
#ifdef ENALEO
#define MINPRNLEO   1                   /* min satellite sat number of LEO */
#define MAXPRNLEO   10                  /* max satellite sat number of LEO */
#define NSATLEO     (MAXPRNLEO-MINPRNLEO+1) /* number of LEO satellites */
#define NSYSLEO     1
#else
#define MINPRNLEO   0
#define MAXPRNLEO   0
#define NSATLEO     0
#define NSYSLEO     0
#endif
#define NSYS        (NSYSGPS+NSYSGLO+NSYSGAL+NSYSQZS+NSYSCMP+NSYSIRN+NSYSLEO) /* number of systems */

#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   142                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */

#define MAXSAT      (NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+NSATSBS+NSATLEO)
                                        /* max satellite number (1 to MAXSAT) */
#define MAXSTA      255

#define trace(level, fmt, ...) printf(fmt, ##__VA_ARGS__);

static const double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */
static const double gst0 []={1999,8,22,0,0,0}; /* galileo system time reference */
static const double bdt0 []={2006,1, 1,0,0,0}; /* beidou time reference */

extern unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    return bits;
}
extern int getbits(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=getbitu(buff,pos,len);
    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int)bits;
    return (int)(bits|(~0u<<len)); /* extend sign */
}
extern double timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}

extern gtime_t epoch2time(const double *ep)
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    gtime_t time={};
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];

    if (year<1970||2099<year||mon<1||12<mon) return time;

    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}

extern gtime_t gst2time(int week, double sec)
{
    gtime_t t=epoch2time(gst0);

    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=(time_t)86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}


extern int satno(int sys, int prn)
{
    if (prn<=0) return 0;
    switch (sys) {
        case SYS_GPS:
            if (prn<MINPRNGPS||MAXPRNGPS<prn) return 0;
            return prn-MINPRNGPS+1;
        case SYS_GLO:
            if (prn<MINPRNGLO||MAXPRNGLO<prn) return 0;
            return NSATGPS+prn-MINPRNGLO+1;
        case SYS_GAL:
            if (prn<MINPRNGAL||MAXPRNGAL<prn) return 0;
            return NSATGPS+NSATGLO+prn-MINPRNGAL+1;
        case SYS_QZS:
            if (prn<MINPRNQZS||MAXPRNQZS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+prn-MINPRNQZS+1;
        case SYS_CMP:
            if (prn<MINPRNCMP||MAXPRNCMP<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+prn-MINPRNCMP+1;
        case SYS_IRN:
            if (prn<MINPRNIRN||MAXPRNIRN<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+prn-MINPRNIRN+1;
        case SYS_LEO:
            if (prn<MINPRNLEO||MAXPRNLEO<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+
                   prn-MINPRNLEO+1;
        case SYS_SBS:
            if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+NSATLEO+
                   prn-MINPRNSBS+1;
        default:
            return 0;
    }
    return 0;
}


extern int decode_gal_inav(const unsigned char *buff, eph_t *eph)
{
    double tow,toc,tt,sqrtA;
    int i,time_f,week,e5b_hs,e1b_hs,e5b_dvs,e1b_dvs,type[6],iod_nav[4];

    i=0; /* word type 0 */
    type[0]    =getbitu(buff,i, 6);              i+= 6;
    time_f     =getbitu(buff,i, 2);              i+= 2+88;
    week       =getbitu(buff,i,12);              i+=12; /* gst-week */
    tow        =getbitu(buff,i,20);

    i=128; /* word type 1 */
    type[1]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[0] =getbitu(buff,i,10);              i+=10;
    eph->toes  =getbitu(buff,i,14)*60.0;         i+=14;
    eph->M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->e     =getbitu(buff,i,32)*P2_33;        i+=32;
    sqrtA      =getbitu(buff,i,32)*P2_19;

    i=128*2; /* word type 2 */
    type[2]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[1] =getbitu(buff,i,10);              i+=10;
    eph->OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->idot  =getbits(buff,i,14)*P2_43*SC2RAD;

    i=128*3; /* word type 3 */
    type[3]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[2] =getbitu(buff,i,10);              i+=10;
    eph->OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
    eph->deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
    eph->cuc   =getbits(buff,i,16)*P2_29;        i+=16;
    eph->cus   =getbits(buff,i,16)*P2_29;        i+=16;
    eph->crc   =getbits(buff,i,16)*P2_5;         i+=16;
    eph->crs   =getbits(buff,i,16)*P2_5;         i+=16;
    eph->sva   =getbitu(buff,i, 8);

    i=128*4; /* word type 4 */
    type[4]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[3] =getbitu(buff,i,10);              i+=10;
    /*svid       =getbitu(buff,i, 6);*/          i+= 6;
    eph->cic   =getbits(buff,i,16)*P2_29;        i+=16;
    eph->cis   =getbits(buff,i,16)*P2_29;        i+=16;
    toc        =getbitu(buff,i,14)*60.0;         i+=14;
    eph->f0    =getbits(buff,i,31)*P2_34;        i+=31;
    eph->f1    =getbits(buff,i,21)*P2_46;        i+=21;
    eph->f2    =getbits(buff,i, 6)*P2_59;

    i=128*5; /* word type 5 */
    type[5]    =getbitu(buff,i, 6);              i+= 6+41;
    eph->tgd[0]=getbits(buff,i,10)*P2_32;        i+=10; /* BGD E5a/E1 */
    eph->tgd[1]=getbits(buff,i,10)*P2_32;        i+=10; /* BGD E5b/E1 */
    e5b_hs     =getbitu(buff,i, 2);              i+= 2;
    e1b_hs     =getbitu(buff,i, 2);              i+= 2;
    e5b_dvs    =getbitu(buff,i, 1);              i+= 1;
    e1b_dvs    =getbitu(buff,i, 1);

    /* test word types */
    if (type[0]!=0||type[1]!=1||type[2]!=2||type[3]!=3||type[4]!=4||type[5]!=5) {
        trace(3,"decode_gal_inav error: type=%d %d %d %d %d %d\n",type[0],
              type[1],type[2],type[3],type[4],type[5]);
        return 0;
    }
    /* test word type 0 time field */
    if (time_f!=2) {
        trace(3,"decode_gal_inav error: word0-time=%d\n",time_f);
        return 0;
    }
    /* test consistency of iod_nav */
    if (iod_nav[0]!=iod_nav[1]||iod_nav[0]!=iod_nav[2]||iod_nav[0]!=iod_nav[3]) {
        trace(3,"decode_gal_inav error: ionav=%d %d %d %d\n",iod_nav[0],
              iod_nav[1],iod_nav[2],iod_nav[3]);
        return 0;
    }
    // RTKLIB has a different numbering scheme for satellites than we do.  This will make
    // RTKLIB angry
    // if (!(eph->sat=satno(SYS_GAL,svid))) {
    //     trace(2,"decode_gal_inav svid error: svid=%d\n",svid);
    //     return 0;
    // }
    eph->A=sqrtA*sqrtA;
    eph->iode=eph->iodc=iod_nav[0];
    eph->svh=(e5b_hs<<7)|(e5b_dvs<<6)|(e1b_hs<<1)|e1b_dvs;
    eph->ttr=gst2time(week,tow);
    tt=timediff(gst2time(week,eph->toes),eph->ttr); /* week complient to toe */
    if      (tt> 302400.0) week--;
    else if (tt<-302400.0) week++;
    eph->toe=gst2time(week,eph->toes);
    eph->toc=gst2time(week,toc);
    eph->week=week+1024; /* gal-week = gst-week + 1024 */
    eph->code =(1<<0)|(1<<9); /* data source = i/nav e1b, af0-2,toc,sisa for e5b-e1 */
    return 1;
}

// clang-format on
}  // namespace rtklib
