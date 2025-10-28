#ifndef CRSF_H
#define CRSF_H

#include <Arduino.h>
#include <crsf_protocol.h>


// Basic setup
#define CRSF_MAX_CHANNEL        16
#define CRSF_FRAME_SIZE_MAX     64
// Device address & type
#define RADIO_ADDRESS           0xEA
#define TYPE_CHANNELS           0x16

// Define AUX channel input limite
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811
#define CRSF_DIGITAL_CHANNEL_MID 992

// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100
#define SERIAL_BAUDRATE                 416666
#define CRSF_TIME_BETWEEN_FRAMES_US     4000
#define CRSF_PAYLOAD_OFFSET             offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE            128
#define CRSF_MSP_TX_BUF_SIZE            128
#define CRSF_PAYLOAD_SIZE_MAX           60
#define CRSF_PACKET_LENGTH              22
#define CRSF_PACKET_SIZE                26
#define CRSF_FRAME_LENGTH               24
#define CRSF_CMD_PACKET_SIZE            8

// ELRS command
#define ELRS_ADDRESS                    0xEE
#define ELRS_PKT_RATE_COMMAND           0x01
#define ELRS_TLM_RATIO_COMMAND          0x02
#define ELRS_SWITCH_MODE_COMMAND        0x03
#define ELRS_MODEL_MATCH_COMMAND        0x04
#define ELRS_POWER_COMMAND              0x06
#define ELRS_DYNAMIC_POWER_COMMAND      0x07
#define ELRS_BLE_JOYSTIC_COMMAND        0x11
#define ELRS_WIFI_COMMAND               0x0F
#define ELRS_BIND_COMMAND               0x11
#define ELRS_START_COMMAND              0x04
#define TYPE_SETTINGS_WRITE             0x2D
#define ADDR_RADIO                      0xEA


class CRSF {
public:
    void begin(void);
    void update();
    void crsfPrepareDataPacket(uint8_t packet[], int16_t channels[]);
    void crsfPrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value);
    void CrsfWritePacket(uint8_t packet[], uint8_t packetLength);

private:
    uint8_t _rxBuf[CRSF_MAX_PACKET_LEN+3];
    uint8_t _rxBufPos;
    int _channels[CRSF_NUM_CHANNELS];
    uint32_t _lastReceive;
    uint32_t _lastChannelsPacket;
    bool _linkIsUp;

    uint8_t _device_address;
    char _device_name[CRSF_MAX_NAME_LEN];
    deviceInformationPacket_t _deviceInfo;

    crsf_channels_t _channelsPacked;
    crsf_sensor_battery_t _batterySensor;
    crsfLinkStatistics_t _linkStatistics;
    crsf_sensor_gps_t _gpsSensor;
    crsf_sensor_vario_t _varioSensor;
    crsf_sensor_baro_altitude_t _baroAltitudeSensor;
    crsf_sensor_attitude_t _attitudeSensor;

    void handleSerialIn();
    void handleByteReceived();
    void shiftRxBuffer(uint8_t cnt);
    void processPacketIn(uint8_t len);

    // Packet RX Handlers
    void packetChannelsPacked(const crsf_header_t *p);
    void packetBatterySensor(const crsf_header_t *p);
    void packetLinkStatistics(const crsf_header_t *p);
    void packetGps(const crsf_header_t *p);
    void packetVario(const crsf_header_t *p);
    void packetBaroAltitude(const crsf_header_t *p);
    void packetAttitude(const crsf_header_t *p);
    void packetDeviceInfo(const crsf_ext_header_t *p);
    void packetParameterSettingsEntry(const crsf_ext_header_t *p);
    void packetRadioId(const crsf_ext_header_t *p);
};


// CRC8 (poly 0xD5)
uint8_t crc8_dvb_s2(const uint8_t *data, size_t len);

void setCRSF_RX();
void setCRSF_TX();

#endif
