/**
 * \file
 * \brief Combined application for WEICLEDFeatherWing and Sensor2BLE functionality.
 *
 * \copyright (c) 2024 Würth Elektronik eiSos GmbH & Co. KG
 * \copyright (c) 2022 Würth Elektronik eiSos GmbH & Co. KG
 *
 * \page License
 *
 * THE SOFTWARE INCLUDING THE SOURCE CODE IS PROVIDED "AS IS". YOU ACKNOWLEDGE
 * THAT WÜRTH ELEKTRONIK EISOS MAKES NO REPRESENTATIONS AND WARRANTIES OF ANY
 * KIND RELATED TO, BUT NOT LIMITED TO THE NON-INFRINGEMENT OF THIRD PARTIES'
 * INTELLECTUAL PROPERTY RIGHTS OR THE MERCHANTABILITY OR FITNESS FOR YOUR
 * INTENDED PURPOSE OR USAGE. WÜRTH ELEKTRONIK EISOS DOES NOT WARRANT OR
 * REPRESENT THAT ANY LICENSE, EITHER EXPRESS OR IMPLIED, IS GRANTED UNDER ANY
 * PATENT RIGHT, COPYRIGHT, MASK WORK RIGHT, OR OTHER INTELLECTUAL PROPERTY
 * RIGHT RELATING TO ANY COMBINATION, MACHINE, OR PROCESS IN WHICH THE PRODUCT
 * IS USED. INFORMATION PUBLISHED BY WÜRTH ELEKTRONIK EISOS REGARDING
 * THIRD-PARTY PRODUCTS OR SERVICES DOES NOT CONSTITUTE A LICENSE FROM WÜRTH
 * ELEKTRONIK EISOS TO USE SUCH PRODUCTS OR SERVICES OR A WARRANTY OR
 * ENDORSEMENT THEREOF
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS PACKAGE
 */

#define PROTEUSIIIFEATHERWING true
#define SENSORFEATHERWING true
#define SETEBOS_MODE_PIN 14

/**         Includes         */
#include "global.h"
#include "debug.h"
#include "ICLED.h"
#include "ICLED_demos.h"
#include "ProteusIII.h"
#include "sensorBoard.h"

/* Common Definitions */
#define HIDS_PART_NUMBER 2525020210001
#define PAYLOAD_BUFFER_SIZE 64
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

/* Sensor Data Variables */
static float PADS_pressure, PADS_temp;
static float ITDS_accelX, ITDS_accelY, ITDS_accelZ, ITDS_temp;
static bool ITDS_doubleTapEvent, ITDS_freeFallEvent;
static float TIDS_temp;
static float HIDS_humidity, HIDS_temp;
static volatile bool check_orientation = false;

/* Test Modes */
typedef enum
{
    TEST1,
    TEST2,
    TEST3,
    TEST4,
    TEST5,
    TEST6,
    TEST7,
    TEST8,
#if PROTEUSIIIFEATHERWING == true
    TEST9,
#endif
#if SENSORFEATHERWING == true
    TEST10,
    TEST11,
#endif
    TEST_Total_Count
} TestMode;

/* Sensor2BLE State Machine */
typedef enum
{
    Sensor2BLE_SM_Idle = 0x0,
    Sensor2BLE_SM_Send_Sensor_Interval = 0x1,
    Sensor2BLE_SM_Send_Sensor_Data = 0x2
} Sensor2BLE_SM_t;

/* Global Variables */
static volatile TestMode current_mode = TEST1;
static volatile Sensor2BLE_SM_t currentstate = Sensor2BLE_SM_Idle;
static volatile bool initial_test_run = true;
static bool running_loop = true;
static bool orientation_changed = false;
static unsigned long prog_debounce_time_elapsed = 0;
static unsigned long eventattrdelay = 3;
static unsigned long eventattrlastupdate = 0;

static uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];
static uint8_t payload[54] = {0};
static int payloadLength = 0;

static ProteusIII_Pins_t ProteusIII_pins;

typedef struct
{
    unsigned long sensorperiod;
    unsigned long lastupdate;
} sensorperiod;

sensorperiod padssensorperiod, itdssensorperiod, hidssensorperiod, tidssensorperiod;

/* Function Prototypes */
void startTimer(int frequencyHz);
void TC3_Handler();
static void PROG_ISR_handler();
void uint32touint8array(uint8_t *result, uint32_t value);
uint32_t int8arraytouint32(uint8_t *value);
bool namechanged(uint8_t *newname, uint8_t *oldname);

/* Proteus-III Callbacks */
static void RxCallback(uint8_t *payload, uint16_t payloadLength, uint8_t *btMac, int8_t rssi);
static void ConnectCallback(bool success, uint8_t *btMac);
static void DisconnectCallback(ProteusIII_DisconnectReason_t reason);
static void ChannelOpenCallback(uint8_t *btMac, uint16_t maxPayload);

void setup()
{
    // Using the USB serial port for debug messages
#ifdef WE_DEBUG
    WE_Debug_Init();
#endif

    /* Set the mode pin on Setebos to Proteus-III mode */
    SetPinMode(SETEBOS_MODE_PIN, OUTPUT);
    WritePin(SETEBOS_MODE_PIN, LOW);

#if PROTEUSIIIFEATHERWING == true
    ProteusIII_pins.ProteusIII_Pin_SleepWakeUp.pin = 9;
    ProteusIII_pins.ProteusIII_Pin_Mode.pin = 17;

    ProteusIII_CallbackConfig_t callbackConfig = {0};
    callbackConfig.rxCb = RxCallback;
    callbackConfig.connectCb = ConnectCallback;
    callbackConfig.disconnectCb = DisconnectCallback;
    callbackConfig.channelOpenCb = ChannelOpenCallback;

    if (!ProteusIII_Init(&ProteusIII_pins, PROTEUSIII_DEFAULT_BAUDRATE,
                         WE_FlowControl_NoFlowControl,
                         ProteusIII_OperationMode_CommandMode,
                         callbackConfig))
    {
        WE_DEBUG_PRINT("Proteus init failed \r\n");
    }

    uint8_t fwversion[3] = {};
    if (ProteusIII_GetFWVersion(fwversion))
    {
        WE_DEBUG_PRINT("read version %d.%d.%d \n", fwversion[2],
                       fwversion[1], fwversion[0]);
        // check for minimum firmware version 1.4.0
        if ((fwversion[2] < 1) ||
            ((fwversion[2] == 1) && fwversion[1] < 4))
        {
            WE_DEBUG_PRINT("unsupported FW version \n");
            exit(0);
        }
    }

    uint8_t readcfg[2] = {0};
    if (ProteusIII_GetCFGFlags((uint16_t *)readcfg))
    {
        uint16_t writecfg = 0x1000;
        if ((readcfg[0] != 0x10) && ProteusIII_SetCFGFlags(writecfg))
        {
            WE_DEBUG_PRINT("flags set \n");
        }
    }

    ProteusIII_ConnectionTiming_t timing;
    if (ProteusIII_GetConnectionTiming(&timing) && timing != ProteusIII_ConnectionTiming_12)
    {
        if (ProteusIII_SetConnectionTiming(ProteusIII_ConnectionTiming_12))
        {
            WE_DEBUG_PRINT("timing set \n");
        }
    }

    uint8_t oldName[32];
    uint16_t oldnameLength = sizeof(oldName);
    uint8_t btmacaddress[6] = {};
    if (ProteusIII_GetBTMAC(btmacaddress))
    {
        char newName[32];
        sprintf(newName, "W-%02x%02x%02x", btmacaddress[2], btmacaddress[1], btmacaddress[0]);

        if (ProteusIII_GetDeviceName(oldName, &oldnameLength))
        {
            if (memcmp(newName, oldName, strlen(newName)) != 0)
            {
                if (ProteusIII_SetAdvertisingFlags(ProteusIII_AdvertisingFlags_DeviceNameAndUuid))
                {
                    if (ProteusIII_SetDeviceName((uint8_t *)newName, strlen(newName)))
                    {
                        WE_DEBUG_PRINT("new name set\n");
                    }
                }
            }
        }
    }

    uint8_t bname[] = {'F', 'W', 'I', 'N', 'G'};
    if (ProteusIII_SetBeacon(bname, 5))
    {
        WE_DEBUG_PRINT("beacon data set\n");
    }
#endif

#if SENSORFEATHERWING == true
    if (!sensorBoard_Init())
    {
        WE_DEBUG_PRINT("I2C init failed \r\n");
    }

    // Initialize the sensors in default mode
    if (!PADS_2511020213301_simpleInit())
    {
        WE_DEBUG_PRINT("PADS init failed \r\n");
    }

    if (!ITDS_2533020201601_simpleInit())
    {
        WE_DEBUG_PRINT("ITDS init failed \r\n");
    }

    if (!TIDS_2521020222501_simpleInit())
    {
        WE_DEBUG_PRINT("TIDS init failed \r\n");
    }

#if HIDS_PART_NUMBER == 2525020210001
    if (!HIDS_2525020210001_simpleInit())
    {
        WE_DEBUG_PRINT("HIDS init failed \r\n");
    }
#elif HIDS_PART_NUMBER == 2525020210002
    if (!HIDS_2525020210002_simpleInit())
    {
        WE_DEBUG_PRINT("HIDS init failed \r\n");
    }
#endif

    startTimer(5);
    padssensorperiod = {5000, 0};
    itdssensorperiod = {30, 0};
    tidssensorperiod = {5000, 0};
    hidssensorperiod = {5000, 0};
#endif

    if (!ICLED_Init(RGB, Landscape))
    {
        WE_DEBUG_PRINT("ICLED init failed \r\n");
    }

    pinMode(ICLED_PROG_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ICLED_PROG_PIN), PROG_ISR_handler, FALLING);
}

void loop()
{
    // Handle Sensor2BLE state machine
    switch (currentstate)
    {
    default:
    case Sensor2BLE_SM_Idle:
        break;
    case Sensor2BLE_SM_Send_Sensor_Interval:
    {
        payload[0] = 7;
        payload[1] = 1;
        payload[2] = 0;
        uint32touint8array(&payload[3], padssensorperiod.sensorperiod);
        payload[7] = 7;
        payload[8] = 1;
        payload[9] = 1;
        uint32touint8array(&payload[10], itdssensorperiod.sensorperiod);
        payload[14] = 7;
        payload[15] = 1;
        payload[16] = 2;
        uint32touint8array(&payload[17], tidssensorperiod.sensorperiod);
        payload[21] = 7;
        payload[22] = 1;
        payload[23] = 3;
        uint32touint8array(&payload[24], hidssensorperiod.sensorperiod);
        payloadLength = 28;
        ProteusIII_Transmit(payload, payloadLength);
        WE_Delay(50);
        currentstate = Sensor2BLE_SM_Send_Sensor_Data;
        break;
    }
    case Sensor2BLE_SM_Send_Sensor_Data:
    {
        unsigned long currtime = millis();
        payloadLength = 0;

        if (((currtime - padssensorperiod.lastupdate) >= padssensorperiod.sensorperiod) &&
            PADS_2511020213301_readSensorData(&PADS_pressure, &PADS_temp))
        {
            WE_DEBUG_PRINT("WSEN_PADS: Atm. Pres: %f kPa Temp: %f °C\r\n", PADS_pressure, PADS_temp);
            payload[0] = 11;
            payload[1] = 0;
            payload[2] = 0;
            uint32touint8array(&payload[3], PADS_pressure * 100);
            uint32touint8array(&payload[7], PADS_temp * 100);
            payloadLength += 11;
            padssensorperiod.lastupdate = currtime;
        }

        if (((currtime - itdssensorperiod.lastupdate) >= itdssensorperiod.sensorperiod) &&
            ITDS_2533020201601_readSensorData(&ITDS_accelX, &ITDS_accelY, &ITDS_accelZ, &ITDS_temp))
        {
            payload[payloadLength] = 19;
            payload[payloadLength + 1] = 0;
            payload[payloadLength + 2] = 1;
            uint32touint8array(&payload[payloadLength + 3], ITDS_accelX * 1000);
            uint32touint8array(&payload[payloadLength + 7], ITDS_accelY * 1000);
            uint32touint8array(&payload[payloadLength + 11], ITDS_accelZ * 1000);
            uint32touint8array(&payload[payloadLength + 15], ITDS_temp * 100);
            payloadLength += 19;
            itdssensorperiod.lastupdate = currtime;
        }

        if ((currtime - eventattrlastupdate) >= eventattrdelay)
        {
            if (ITDS_2533020201601_readDoubleTapEvent(&ITDS_doubleTapEvent) && ITDS_doubleTapEvent)
            {
                WE_DEBUG_PRINT("Double Tap Detected\r\n");
                payload[payloadLength] = 3;
                payload[payloadLength + 1] = 2;
                payload[payloadLength + 2] = 1;
                payloadLength += 3;
            }
            if (ITDS_2533020201601_readFreeFallEvent(&ITDS_freeFallEvent) && ITDS_freeFallEvent)
            {
                WE_DEBUG_PRINT("Free Fall Detected\r\n");
                payload[payloadLength] = 3;
                payload[payloadLength + 1] = 3;
                payload[payloadLength + 2] = 1;
                payloadLength += 3;
            }
            eventattrlastupdate = currtime;
        }

        if (((currtime - tidssensorperiod.lastupdate) >= tidssensorperiod.sensorperiod) &&
            TIDS_2521020222501_readSensorData(&TIDS_temp))
        {
            WE_DEBUG_PRINT("WSEN_TIDS(Temperature): %f °C\r\n", TIDS_temp);
            payload[payloadLength] = 7;
            payload[payloadLength + 1] = 0;
            payload[payloadLength + 2] = 2;
            uint32touint8array(&payload[payloadLength + 3], TIDS_temp * 100);
            payloadLength += 7;
            tidssensorperiod.lastupdate = currtime;
        }

        if (((currtime - hidssensorperiod.lastupdate) >= hidssensorperiod.sensorperiod) &&
#if HIDS_PART_NUMBER == 2525020210001
            HIDS_2525020210001_readSensorData(&HIDS_humidity, &HIDS_temp)
#elif HIDS_PART_NUMBER == 2525020210002
            HIDS_2525020210002_readSensorData(&HIDS_humidity, &HIDS_temp)
#else
            false
#endif
        )
        {
            WE_DEBUG_PRINT("WSEN_HIDS: RH: %f %% Temp: %f °C\r\n", HIDS_humidity, HIDS_temp);
            payload[payloadLength] = 11;
            payload[payloadLength + 1] = 0;
            payload[payloadLength + 2] = 3;
            uint32touint8array(&payload[payloadLength + 3], HIDS_humidity * 100);
            uint32touint8array(&payload[payloadLength + 7], HIDS_temp * 100);
            payloadLength += 11;
            hidssensorperiod.lastupdate = currtime;
        }

        if (payloadLength != 0)
        {
            ProteusIII_Transmit(payload, payloadLength);
        }
        break;
    }
    }

    // Handle ICLED orientation changes
#if SENSORFEATHERWING == true
    ICLED_Orientation current_orientation = ICLED_get_orientation();
    if (check_orientation)
    {
        if (!TIDS_2521020222501_readSensorData(&TIDS_temp))
        {
            WE_DEBUG_PRINT("error reading TIDS data \r\n");
        }
#if HIDS_PART_NUMBER == 2525020210001
        if (!HIDS_2525020210001_readSensorData(&HIDS_humidity, NULL))
        {
            WE_DEBUG_PRINT("error reading TIDS data \r\n");
        }
#elif HIDS_PART_NUMBER == 2525020210002
        if (!HIDS_2525020210002_readSensorData(&HIDS_humidity, NULL))
        {
            WE_DEBUG_PRINT("error reading TIDS data \r\n");
        }
#endif
        if (ITDS_2533020201601_readSensorData(NULL, &ITDS_accelY, NULL, NULL))
        {
            ICLED_set_orientation(ITDS_accelY > 0.05 ? Landscape_UpsideDown : Landscape);
        }
        check_orientation = false;
    }
    orientation_changed = (current_orientation != ICLED_get_orientation());
#endif

    // Handle ICLED test modes
    ICLED_set_color_system(RGB);

    switch (current_mode)
    {
    case TEST1:
    {
        running_loop = true;
        ICLED_demo_start_show((bool *)&initial_test_run, &running_loop, (uint8_t *)&current_mode);
        break;
    }
    case TEST2:
    {
        if (!initial_test_run && !orientation_changed)
        {
            break;
        }
        ICLED_clear(false);
        initial_test_run = false;
        ICLED_set_pixel(0, 0, 0, 50, 128);
        ICLED_set_screen_pixel(3, 6, 0, 50, 0, 128);
        ICLED_set_pixel(104, 50, 0, 0, 255);
        break;
    }
    case TEST3:
    {
        ICLED_clear(false);
        running_loop = true;
        ICLED_demo_send_alphabet(128, 128, 128, 10, 80, &running_loop);
        break;
    }
    case TEST4:
    {
        ICLED_clear(false);
        running_loop = true;
        uint16_t place = 12;
        char string[] = "Hello World!";
        ICLED_set_string(string, &place, 128, 128, 128, 10, false);
        static uint16_t hello_world_current_column = place;
        ICLED_start_conditional_loop(0, place, 100, &hello_world_current_column, &running_loop);
        break;
    }
    case TEST5:
    {
        ICLED_clear(false);
        running_loop = true;
        ICLED_demo_show_rainbow(8, 35, &running_loop);
        break;
    }
    case TEST6:
    {
        char price[] = "42,69";
        ICLED_clear(false);
        running_loop = true;
        ICLED_demo_show_price(price, 0, 128, 0, 10, 80, &running_loop);
        break;
    }
    case TEST7:
    {
        uint16_t place = 0;
        ICLED_clear(false);
        running_loop = true;
        ICLED_set_emoji(Emoji_Smile_Face, &place, 128, 128, 128, 10, false);
        ICLED_set_emoji(Emoji_Neutral_Face, &place, 128, 128, 128, 10, false);
        ICLED_set_emoji(Emoji_Frown_Face, &place, 128, 128, 128, 10, false);
        ICLED_set_emoji(Emoji_Heart, &place, 128, 0, 0, 15, false);
        ICLED_set_emoji(Emoji_Smile_Face, &place, 128, 128, 128, 10, false);
        ICLED_set_emoji(Emoji_Neutral_Face, &place, 128, 128, 128, 10, false);
        ICLED_set_emoji(Emoji_Frown_Face, &place, 128, 128, 128, 10, false);
        ICLED_set_emoji(Emoji_Heart, &place, 128, 0, 0, 15, false);
        ICLED_set_emoji(Emoji_Smile_Face, &place, 128, 128, 128, 10, false);
        ICLED_set_emoji(Emoji_Neutral_Face, &place, 128, 128, 128, 10, false);
        ICLED_set_emoji(Emoji_Frown_Face, &place, 128, 128, 128, 10, false);
        ICLED_set_emoji(Emoji_Heart, &place, 128, 0, 0, 15, false);
        static uint16_t emojis_current_column = 0;
        ICLED_start_conditional_loop(0, place, 100, &emojis_current_column, &running_loop);
        break;
    }
    case TEST8:
    {
        static uint8_t col = 0;
        static uint8_t frameDelay = 0;

        if (orientation_changed)
        {
            col = 0;
            frameDelay = 0;
            orientation_changed = false;
            initial_test_run = true;
        }

        if (!initial_test_run)
        {
            break; // stop running after one full sweep
        }

        if (++frameDelay < 10)
        {
            break; // simple delay between frames (adjust for speed)
        }
        frameDelay = 0;

        ICLED_clear(false);

        ICLED_set_color_system(RGB);
        for (uint8_t row = 1; row < ICLED_ROWS; row++)
        {
            ICLED_set_screen_pixel(row, col, col * 2, 0, 255 - col * 2, 30);
        }

        ICLED_set_color_system(HSV);
        for (uint8_t row = 1; row < ICLED_ROWS; row++)
        {
            uint16_t hue = (col * 360) / ICLED_COLUMNS;
            ICLED_set_screen_pixel(row, col, hue, 100, 80, 20);
        }

        col++;
        if (col >= ICLED_COLUMNS)
        {
            initial_test_run = false; // stop after finishing last column
        }

        break;
    }
#if PROTEUSIIIFEATHERWING == true
    case TEST9:
    {
        if (initial_test_run)
        {
            strcpy((char *)payload_buffer, "Send BLE Data");
        }
        initial_test_run = false;
        ICLED_clear(false);
        running_loop = true;
        ICLED_demo_show_ble_data((char *)payload_buffer, 0, 0, 255, 25, 80, &running_loop);
        break;
    }
#endif
#if SENSORFEATHERWING == true
    case TEST10:
    {
        ICLED_clear(false);
        running_loop = true;
        ICLED_demo_show_temp_sensor_data(TIDS_temp, 25, 80, &running_loop);
        break;
    }
    case TEST11:
    {
        ICLED_clear(false);
        running_loop = true;
        ICLED_demo_show_hum_sensor_data(HIDS_humidity, 25, 80, &running_loop);
        break;
    }
#endif
    case TEST_Total_Count:
    {
        WE_DEBUG_PRINT("Invalid Test \r\n");
        exit(0);
    }
    }
}

/* Helper Functions */
void uint32touint8array(uint8_t *result, uint32_t value)
{
    result[0] = (value & 0x000000ff);
    result[1] = (value & 0x0000ff00) >> 8;
    result[2] = (value & 0x00ff0000) >> 16;
    result[3] = (value & 0xff000000) >> 24;
}

uint32_t int8arraytouint32(uint8_t *value)
{
    return value[0] | (value[1] << 8) | (value[2] << 16) | (value[3] << 24);
}

bool namechanged(uint8_t *newname, uint8_t *oldname)
{
    for (int i = 0; i < 8; i++)
    {
        if (oldname[i] != newname[i])
        {
            return true;
        }
    }
    return false;
}

/* Proteus-III Callback Implementations */
static void RxCallback(uint8_t *payload, uint16_t payloadLength, uint8_t *btMac, int8_t rssi)
{
    if (current_mode == TEST9)
    {
        if (payloadLength > (PAYLOAD_BUFFER_SIZE - 1))
        {
            return;
        }
        memcpy(payload_buffer, payload, payloadLength);
        payload_buffer[payloadLength] = '\0';
        running_loop = false;
    }
    else
    {
        int byteidx = 7;
        while (byteidx < payloadLength)
        {
            int packetlength = payload[byteidx];
            int packetfunction = payload[byteidx + 1];
            int packetsensortype = payload[byteidx + 2];
            switch (packetfunction)
            {
            case 1:
                switch (packetsensortype)
                {
                case 0:
                    padssensorperiod.sensorperiod = int8arraytouint32(&payload[byteidx + 3]);
                    break;
                case 1:
                    itdssensorperiod.sensorperiod = int8arraytouint32(&payload[byteidx + 3]);
                    break;
                case 2:
                    tidssensorperiod.sensorperiod = int8arraytouint32(&payload[byteidx + 3]);
                    break;
                case 3:
                    hidssensorperiod.sensorperiod = int8arraytouint32(&payload[byteidx + 3]);
                    break;
                }
                break;
            }
            byteidx += packetlength;
        }
    }
}

static void ConnectCallback(bool success, uint8_t *btMac)
{
    WE_DEBUG_PRINT("device connected \n");
    currentstate = Sensor2BLE_SM_Idle;
}

static void DisconnectCallback(ProteusIII_DisconnectReason_t reason)
{
    WE_DEBUG_PRINT("device disconnected \n");
    currentstate = Sensor2BLE_SM_Idle;
}

static void ChannelOpenCallback(uint8_t *btMac, uint16_t maxPayload)
{
    WE_DEBUG_PRINT("device started notifications \n");
    currentstate = Sensor2BLE_SM_Send_Sensor_Interval;
}

/* Timer Functions */
void startTimer(int frequencyHz)
{
    REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);
    while (GCLK->STATUS.bit.SYNCBUSY == 1)
        ;

    TcCount16 *TC = (TcCount16 *)TC3;

    TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (TC->STATUS.bit.SYNCBUSY == 1)
        ;

    // Use the 16-bit timer
    TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    while (TC->STATUS.bit.SYNCBUSY == 1)
        ;

    // Use match mode so that the timer counter resets when the count matches the compare register
    TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    while (TC->STATUS.bit.SYNCBUSY == 1)
        ;

    // Set prescaler to 1024
    TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
    while (TC->STATUS.bit.SYNCBUSY == 1)
        ;

    int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
    // Make sure the count is in a proportional position to where it was
    // to prevent any jitter or disconnect when changing the compare value.
    TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
    TC->CC[0].reg = compareValue;
    while (TC->STATUS.bit.SYNCBUSY == 1)
        ;
    // Enable the compare interrupt
    TC->INTENSET.reg = 0;
    TC->INTENSET.bit.MC0 = 1;

    NVIC_EnableIRQ(TC3_IRQn);

    TC->CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC->STATUS.bit.SYNCBUSY == 1)
        ;
}

void TC3_Handler()
{
    TcCount16 *TC = (TcCount16 *)TC3;
    if (TC->INTFLAG.bit.MC0 == 1)
    {
        TC->INTFLAG.bit.MC0 = 1;
        check_orientation = true;
        running_loop = false;
    }
}

/* Interrupt Handler */
static void PROG_ISR_handler()
{
    if ((millis() - prog_debounce_time_elapsed) < 300)
    {
        return;
    }
    prog_debounce_time_elapsed = millis();
    current_mode = (TestMode)((current_mode + 1) % TEST_Total_Count);
    running_loop = false;
    initial_test_run = true;
}