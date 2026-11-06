#ifndef __UBLOX_H
#define	__UBLOX_H

#include <stdint.h>

#define UBLOX_SYNC1	    0xB5
#define UBLOX_SYNC2	    0x62

#define UBLOX_NAV_CLASS	    0x01
#define UBLOX_RXM_CLASS	    0x02
#define UBLOX_ACK_CLASS     0x05
#define UBLOX_CFG_CLASS	    0x06
#define UBLOX_MON_CLASS	    0x0a
#define UBLOX_AID_CLASS	    0x0b
#define UBLOX_TIM_CLASS	    0x0d

#define UBLOX_NAV_POSLLH    0x02
#define UBLOX_NAV_DOP	    0x04
#define UBLOX_NAV_SOL       0x06
#define UBLOX_NAV_PVT       0x07
#define UBLOX_NAV_VALNED    0x12
#define UBLOX_NAV_TIMEUTC   0x21
#define UBLOX_NAV_SBAS	    0x32
#define UBLOX_NAV_SVINFO    0x30

#define UBLOX_ACK_NAK       0x00
#define UBLOX_ACK_ACK       0x01

#define UBLOX_AID_REQ	    0x00

#define UBLOX_RXM_RAW	    0x10
#define UBLOX_RXM_SFRB	    0x11

#define UBLOX_MON_VER	    0x04
#define UBLOX_MON_HW	    0x09

#define UBLOX_TIM_TP	    0x01

#define UBLOX_CFG_PRT       0x00
#define UBLOX_CFG_MSG	    0x01
#define UBLOX_CFG_TP	    0x07
#define UBLOX_CFG_RTATE	    0x08
#define UBLOX_CFG_CFG	    0x09
#define UBLOX_CFG_SBAS	    0x16
#define UBLOX_CFG_NAV5	    0x24

#define UBLOX_MAX_PAYLOAD   256

typedef struct {
    float time;		 // Time
    double latitude;	 // Latitude
    double longitude;    // Longitude
    float altitude;	 // Altitude
    float velN;		 // North velocity
    float velE;		 // East velocity
    float velD;		 // Down velocity
    float speed;         // Ground speed
    float heading;       // Heading
    float hAcc;		 // Horizontal position accuracy
    float vAcc;          // Vertical position accuracy
    float sAcc;          // Speed accuracy
    float cAcc;          // Heading accuracy
    uint8_t fixed;       // Fix status
    uint8_t numSV;	 // Number of satellites
} GPS_t;

enum
{
    UBLOX_WAIT_SYNC1 = 0,
    UBLOX_WAIT_SYNC2,
    UBLOX_WAIT_CLASS,
    UBLOX_WAIT_ID,
    UBLOX_WAIT_LEN1,
    UBLOX_WAIT_LEN2,
    UBLOX_PAYLOAD,
    UBLOX_CHECK1,
    UBLOX_CHECK2
};

enum
{
    PORTABLE = 0,
    STATIONARY = 2,
    PEDESTRIAN = 3,
    AUTOMOTIVE = 4,
    SEA = 5,
    AIRBORNE_1G = 6,
    AIRBORNE_2G = 7,
    AIRBORNE_4G = 8
};

#pragma pack (1)
typedef struct {
    uint32_t   iTOW;	    //GPS Millisecond Time of Week	ms
    uint16_t   year;        //Year, range 1999..2099 (UTC)
    uint8_t    month;		//Month, range 1..12 (UTC)
    uint8_t    day;		    //Day of Month, range 1..31 (UTC)
    uint8_t    hour;		//Hour of Day, range 0..23 (UTC)
    uint8_t    min;		    //Minute of Hour, range 0..59 (UTC)
    uint8_t    sec;		    //Seconds of Minute, range 0..59 (UTC)
    uint8_t    valid;       //Validity Flags
    uint32_t   tAcc;        //Time Accuracy Estimate
    int32_t    nano;        //Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t    gpsFix;      //GPSfix Type, range 0..5 0: no fix 1: dead reckoning only 2: 2D-fix 3: 3D-fix 4: GNSS + dead reckoning combined 5: time only fix
    uint8_t    flags;       //Fix status flags
    uint8_t    flags2;      //Additional flags
    uint8_t    numSV;       //Number of SVs used in Nav Solution
    int32_t	   lon;         //Longitude
    int32_t	   lat;         //Latitude
    int32_t	   height;      //Height above Ellipsoid
    int32_t	   hMSL;        //Height above mean sea level
    uint32_t   hAcc;        //Horizontal Accuracy Estimate
    uint32_t   vAcc;        //Vertical Accuracy Estimate
    int32_t    velN;        //NED north velocity	mm/s
    int32_t    velE;        //NED east velocity		mm/s
    int32_t    velD;        //NED down velocity		mm/s
    int32_t    gSpeed;		//Ground Speed (2-D)	mm/s
    int32_t    heading;		// Heading of motion (2-D), 1e-5 deg
    uint32_t   sAcc;		//Speed Accuracy Estimate	mm/s
    uint32_t   cAcc;		// Heading accuracy estimate, 1e-5 deg
    uint16_t   pDOP;        //Position DOP
    uint8_t    res1;        //reserved
    int32_t    headVeh;     //Heading of vehicle (2-D)
    uint8_t    res2;        //reserved
} UBLOX_PVT_t;

typedef struct {
    union
    {
        UBLOX_PVT_t     pvt;
        char other[UBLOX_MAX_PAYLOAD];
    } payload;

    uint8_t  state;
    uint16_t count;
    uint8_t  class;
    uint8_t  id;
    uint16_t length;
    uint16_t checksumErrors;

    uint8_t  ubloxRxCK_A;
    uint8_t  ubloxRxCK_B;

    uint8_t  ubloxTxCK_A;
    uint8_t  ubloxTxCK_B;
} UBLOX_RAW_t;

typedef struct {
    uint16_t   year;
    uint8_t    month;
    uint8_t    day;
    uint8_t    hour;
    uint8_t    min;
    uint8_t    sec;
} UTC_TIME_t;

#pragma pack ()

void ublox_init(void);

#endif

