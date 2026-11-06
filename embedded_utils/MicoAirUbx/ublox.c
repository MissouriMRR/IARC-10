#include "ublox.h"

// Default GPS baud rate
#define GPS_DEFAULT_BAUDRATE 115200

GPS_t gps_data;
UTC_TIME_t time;

static void ublox_parse_char(uint8_t data);
static void ublox_enable_message(uint8_t class, uint8_t id, uint8_t rate);
static void ublox_cfg_rate(uint16_t rate);
static void ublox_cfg_port(uint32_t baudrate);
static void ublox_cfg_nav5(void);

// Send data over the serial port; implement this function for the target platform
void ublox_send_message(uint8_t *data, uint8_t length)
{
    // Implement serial transmission for the target platform
}

// u-blox data decoder; call it from the serial interrupt or serial-data handler and pass in each received byte
void ublox_decode(uint8_t data)
{
    ublox_parse_char(data);
}

// Millisecond delay; implement this function for the target platform
void delay_ms(uint32_t ms)
{
}

/**********************************************************************************************************
* Function: ublox_init
* Description: Initialize the u-blox configuration
* Parameters: None
* Return value: None
**********************************************************************************************************/
void ublox_init(void)
{
    // Implement serial-port initialization for the target platform; the MG90X default baud rate is 115200
    // usart_init(GPS_DEFAULT_BAUDRATE);
    
    // Wait 300 ms after powering the GPS module before configuring it
    delay_ms(300);
    
    // Configure the u-blox UART
    // Baud rate: 115200
    // Input protocols: NMEA + UBX + RTCM
    // Output protocol: UBX
    ublox_cfg_port(GPS_DEFAULT_BAUDRATE);
    delay_ms(5);
    
    // Configure the output rate
    // The input parameter is the interval in ms
    // At output rates >=10 Hz, the u-blox M9140 (MG-902/903) limits navigation to 16 satellites
    // (the 16 satellites with the best signal quality among those visible)
    // To use the maximum number of satellites for navigation (32 on M9), set the output rate to no more than 8 Hz
    ublox_cfg_rate(125); // 125 ms output interval
    delay_ms(5);
    
    // Configure navigation parameters
    // AIRBORNE_2G mode (recommended for drones)
    ublox_cfg_nav5();
    delay_ms(5);
    
    // Enable PVT message output
    // The PVT message contains latitude, longitude, time, velocity, and the other data required for navigation,
    // so outputting and parsing this message alone is sufficient
    ublox_enable_message(UBLOX_NAV_CLASS, UBLOX_NAV_PVT, 1);
}

/**********************************************************************************************************
* Function: ublox_payload_decode
* Description: Decode the u-blox payload
* Parameters: Input data
* Return value: None
**********************************************************************************************************/
static void ublox_payload_decode(UBLOX_RAW_t raw_data)
{
    if(raw_data.class == UBLOX_NAV_CLASS)
    {
        switch(raw_data.id)
        {
        case UBLOX_NAV_PVT:
            gps_data.time      = (float)raw_data.payload.pvt.iTOW / 1000;  
            gps_data.latitude  = (double)raw_data.payload.pvt.lat * (double)1e-7; // Unit: deg
            gps_data.longitude = (double)raw_data.payload.pvt.lon * (double)1e-7; // Unit: deg  
            gps_data.altitude  = (float)raw_data.payload.pvt.hMSL * 0.001f;       // Unit: m
            gps_data.hAcc      = (float)raw_data.payload.pvt.hAcc * 0.001f;       // Unit: m
            gps_data.vAcc      = (float)raw_data.payload.pvt.vAcc * 0.001f;       // Unit: m
            gps_data.velN      = (float)raw_data.payload.pvt.velN * 0.001f;       // Unit: m/s
            gps_data.velE      = (float)raw_data.payload.pvt.velE * 0.001f;       // Unit: m/s
            gps_data.velD      = (float)raw_data.payload.pvt.velD * 0.001f;       // Unit: m/s
            gps_data.speed     = (float)raw_data.payload.pvt.gSpeed * 0.001f;     // Unit: m/s
            gps_data.heading   = (float)raw_data.payload.pvt.heading * 1e-5f;     // Unit: deg
            gps_data.sAcc      = (float)raw_data.payload.pvt.sAcc * 0.001f;       // Unit: m/s
            gps_data.cAcc      = (float)raw_data.payload.pvt.cAcc * 1e-5f;        // Unit: deg
            gps_data.numSV     = raw_data.payload.pvt.numSV;
            gps_data.fixed     = raw_data.payload.pvt.gpsFix;
            time.year          = raw_data.payload.pvt.year;
            time.month         = raw_data.payload.pvt.month;
            time.day           = raw_data.payload.pvt.day;
            time.hour          = raw_data.payload.pvt.hour;
            time.min           = raw_data.payload.pvt.min;
            time.sec           = raw_data.payload.pvt.sec;
            break;

        default:
            break;
        }
    }
    else if(raw_data.class == UBLOX_ACK_CLASS)
    {
        if(raw_data.id == UBLOX_ACK_ACK)
        {
        }
    }
}

/**********************************************************************************************************
* Function: ublox_parse_char
* Description: Parse a u-blox protocol byte stream
* Parameters: Input data
* Return value: None
**********************************************************************************************************/
static void ublox_parse_char(uint8_t data)
{
    static UBLOX_RAW_t ublox_raw;

    switch (ublox_raw.state)
    {
    /* Frame header 1 */
    case UBLOX_WAIT_SYNC1:
        if (data == UBLOX_SYNC1)
            ublox_raw.state = UBLOX_WAIT_SYNC2;
        break;

    /* Frame header 2 */
    case UBLOX_WAIT_SYNC2:
        if (data == UBLOX_SYNC2)
            ublox_raw.state = UBLOX_WAIT_CLASS;
        else
            ublox_raw.state = UBLOX_WAIT_SYNC1;
        break;

    /* Message class */
    case UBLOX_WAIT_CLASS:
        ublox_raw.class = data;
        // Initialize the checksum
        ublox_raw.ubloxRxCK_A = 0;
        ublox_raw.ubloxRxCK_B = 0;
        // Calculate the checksum
        ublox_raw.ubloxRxCK_A += data;
        ublox_raw.ubloxRxCK_B += ublox_raw.ubloxRxCK_A;
        ublox_raw.state = UBLOX_WAIT_ID;
        break;

    /* Message ID */
    case UBLOX_WAIT_ID:
        ublox_raw.id = data;
        // Calculate the checksum
        ublox_raw.ubloxRxCK_A += data;
        ublox_raw.ubloxRxCK_B += ublox_raw.ubloxRxCK_A;
        ublox_raw.state = UBLOX_WAIT_LEN1;
        break;

    /* Low 8 bits of the message length */
    case UBLOX_WAIT_LEN1:
        ublox_raw.length = data;
        // Calculate the checksum
        ublox_raw.ubloxRxCK_A += data;
        ublox_raw.ubloxRxCK_B += ublox_raw.ubloxRxCK_A;
        ublox_raw.state = UBLOX_WAIT_LEN2;
        break;

    /* High 8 bits of the message length */
    case UBLOX_WAIT_LEN2:
        ublox_raw.length += (data << 8);
        // Calculate the checksum
        ublox_raw.ubloxRxCK_A += data;
        ublox_raw.ubloxRxCK_B += ublox_raw.ubloxRxCK_A;
        if (ublox_raw.length >= (UBLOX_MAX_PAYLOAD-1))
        {
            ublox_raw.length = 0;
            ublox_raw.state = UBLOX_WAIT_SYNC1;
        }
        else if (ublox_raw.length > 0)
        {
            ublox_raw.count = 0;
            ublox_raw.state = UBLOX_PAYLOAD;
        }
        else
        {
            ublox_raw.state = UBLOX_CHECK1;
        }
        break;

    /* Message payload */
    case UBLOX_PAYLOAD:
        *((char *)(&ublox_raw.payload) + ublox_raw.count) = data;
        if (++ublox_raw.count == ublox_raw.length)
        {
            ublox_raw.state = UBLOX_CHECK1;
        }
        // Calculate the checksum
        ublox_raw.ubloxRxCK_A += data;
        ublox_raw.ubloxRxCK_B += ublox_raw.ubloxRxCK_A;
        break;

    /* Compare the CKA checksum byte */
    case UBLOX_CHECK1:
        if (data == ublox_raw.ubloxRxCK_A)
        {
            ublox_raw.state = UBLOX_CHECK2;
        }
        else
        {
            ublox_raw.state = UBLOX_WAIT_SYNC1;
            ublox_raw.checksumErrors++;
        }
        break;

    /* Compare the CKB checksum byte */
    case UBLOX_CHECK2:
        ublox_raw.state = UBLOX_WAIT_SYNC1;
        if (data == ublox_raw.ubloxRxCK_B)
        {
            // Reception is complete; decode the payload
            ublox_payload_decode(ublox_raw);
        }
        else
        {
            ublox_raw.checksumErrors++;
        }
        break;

    default:
        break;
    }
}

/**********************************************************************************************************
* Function: ublox_enable_message
* Description: Enable u-blox message output
* Parameters: Message class, message ID, and output rate
*             (0 disables output; 1 outputs once per cycle, the fastest setting; larger values reduce the output rate)
* Return value: None
**********************************************************************************************************/
static void ublox_enable_message(uint8_t class, uint8_t id, uint8_t rate)
{
    uint8_t data[11];
    uint8_t msg_len = 11;
    uint8_t ck_a = 0, ck_b = 0;

    data[0] = UBLOX_SYNC1;     // Frame header 1
    data[1] = UBLOX_SYNC2;     // Frame header 2
    data[2] = UBLOX_CFG_CLASS; // Message class
    data[3] = UBLOX_CFG_MSG;   // Message ID
    data[4] = 0x03;            // Low 8 bits of the payload length
    data[5] = 0x00;            // High 8 bits of the payload length

    data[6] = class;           // Set the message class
    data[7] = id;              // Set the message ID
    data[8] = rate;            // Set the message output rate

    for(uint8_t i=2; i<msg_len-2; i++)
    {
        ck_a += data[i];
        ck_b += ck_a;
    }

    data[9] = ck_a;
    data[10] = ck_b;

    ublox_send_message(data, msg_len);
}

/**********************************************************************************************************
* Function: ublox_cfg_rate
* Description: Set the u-blox output rate
* Parameters: Output interval (unit: ms)
* Return value: None
**********************************************************************************************************/
static void ublox_cfg_rate(uint16_t rate)
{
    uint8_t data[14];
    uint8_t msg_len = 14;
    uint8_t ck_a = 0, ck_b = 0;

    data[0] = UBLOX_SYNC1;             // Frame header 1
    data[1] = UBLOX_SYNC2;             // Frame header 2
    data[2] = UBLOX_CFG_CLASS;         // Message class
    data[3] = UBLOX_CFG_RTATE;         // Message ID
    data[4] = 0x06;                    // Low 8 bits of the payload length
    data[5] = 0x00;                    // High 8 bits of the payload length

    data[6] = (uint8_t)rate;           // Low eight bits of the output interval
    data[7] = (uint8_t)(rate >> 8);    // High eight bits of the output interval
    data[8] = 0x01;                    // Low eight bits of the navigation-cycle count
    data[9] = 0x00;                    // High eight bits of the navigation-cycle count
    data[10] = 0x01;                    //timeRef 0:UTC, 1:GPS time
    data[11] = 0x00;                    // High eight bits

    for(uint8_t i=2; i<msg_len-2; i++)
    {
        ck_a += data[i];
        ck_b += ck_a;
    }

    data[12] = ck_a;
    data[13] = ck_b;

    ublox_send_message(data, msg_len);
}

/**********************************************************************************************************
* Function: ublox_cfg_port
* Description: Configure u-blox output
* Parameters: Baud rate
* Return value: None
**********************************************************************************************************/
static void ublox_cfg_port(uint32_t baudrate)
{
    uint8_t data[28];
    uint8_t msg_len = 28;
    uint8_t ck_a = 0, ck_b = 0;

    data[0] = UBLOX_SYNC1;               // Frame header 1
    data[1] = UBLOX_SYNC2;               // Frame header 2
    data[2] = UBLOX_CFG_CLASS;           // Message class
    data[3] = UBLOX_CFG_PRT;             // Message ID
    data[4] = 0x14;                      // Low 8 bits of the payload length
    data[5] = 0x00;                      // High 8 bits of the payload length

    data[6] = 0x01;                      // Port number, 1: UART1
    data[7] = 0x00;                      // Reserved
    data[8] = 0x00;                      // High eight bits of TX Ready
    data[9] = 0x00;                      // Low eight bits of TX Ready
    data[10] = (uint8_t)(0x08D0);         // Serial-port configuration
    data[11] = (uint8_t)(0x08D0 >> 8);    // 8 data bits
    data[12] = (uint8_t)(0x08D0 >> 16);   // 1 stop bit
    data[13] = (uint8_t)(0x08D0 >> 24);   // No parity
    data[14] = (uint8_t)(baudrate);       // Serial baud rate
    data[15] = (uint8_t)(baudrate >> 8);  //
    data[16] = (uint8_t)(baudrate >> 16); //
    data[17] = (uint8_t)(baudrate >> 24); //
    data[18] = 0x07;                      // Input-protocol configuration
    data[19] = 0x00;                      //ubx+nmea+rtcm
    data[20] = 0x01;                      // Output-protocol configuration
    data[21] = 0x00;                      //ubx
    data[22] = 0x00;                      // Low eight bits of flags
    data[23] = 0x00;                      // High eight bits of flags
    data[24] = 0x00;                      //reserved
    data[25] = 0x00;                      //reserved

    for(uint8_t i=2; i<msg_len-2; i++)
    {
        ck_a += data[i];
        ck_b += ck_a;
    }

    data[26] = ck_a;
    data[27] = ck_b;

    ublox_send_message(data, msg_len);
}

/**********************************************************************************************************
* Function: ublox_cfg_nav5
* Description: Configure navigation parameters
* Parameters: None
* Return value: None
**********************************************************************************************************/
static void ublox_cfg_nav5(void)
{
    uint8_t data[44];
    uint8_t msg_len = 44;
    uint8_t ck_a = 0, ck_b = 0;

    data[0] = UBLOX_SYNC1;               // Frame header 1
    data[1] = UBLOX_SYNC2;               // Frame header 2
    data[2] = UBLOX_CFG_CLASS;           // Message class
    data[3] = UBLOX_CFG_NAV5;            // Message ID
    data[4] = 0x24;                      // Low 8 bits of the payload length
    data[5] = 0x00;                      // High 8 bits of the payload length

    data[6] = 0xFF;                      // Mask
    data[7] = 0xFF;                      // Mask
    data[8] = AIRBORNE_2G;               // Dynamic platform model; AIRBORNE_2G is recommended for drones
    data[9] = 0x03;                      // position fixing mode 3=auto 2D/3D
    data[10] = (uint8_t)(0x00);          // Fixed altitude(mean sea level) for 2D fix mode
    data[11] = (uint8_t)(0x00 >> 8);   
    data[12] = (uint8_t)(0x00 >> 16);  
    data[13] = (uint8_t)(0x00 >> 24);  
    data[14] = (uint8_t)(10000);         // Fixed altitude variance for 2D mode
    data[15] = (uint8_t)(10000 >> 8);  
    data[16] = (uint8_t)(10000 >> 16); 
    data[17] = (uint8_t)(10000 >> 24);
    data[18] = 0x0A;                     // Minimum satellite elevation: 10°
    data[19] = 0x00;                     // drLimit
    data[20] = (uint8_t)(250);           // pDop
    data[21] = (uint8_t)(250 >> 8);     
    data[22] = (uint8_t)(250);           // tDop
    data[23] = (uint8_t)(250 >> 8);     
    data[24] = (uint8_t)(100);           // pAcc Mask
    data[25] = (uint8_t)(100 >> 8);     
    data[26] = (uint8_t)(350);           // tAcc Mask
    data[27] = (uint8_t)(350 >> 8);     
    data[28] = 0;                        // Static hold threshold
    data[29] = 60;                       // DGNSS timeout
    data[30] = 0;                        // cn0ThreshNumSVs
    data[31] = 25;                       // Minimum C/N0 required for a satellite to participate in navigation
    data[32] = 0;                        // pAccAdr
    data[33] = 0;                                 
    data[34] = 0;                        // staticHoldMaxDist     
    data[35] = 0;
    data[36] = 0;                        // utcStandard
    data[37] = 0;                        // reserved
    data[38] = 0;    
    data[39] = 0;    
    data[40] = 0;    
    data[41] = 0;    

    for(uint8_t i=2; i<msg_len-2; i++)
    {
        ck_a += data[i];
        ck_b += ck_a;
    }

    data[42] = ck_a;
    data[43] = ck_b;

    ublox_send_message(data, msg_len);
}



