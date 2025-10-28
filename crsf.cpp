#include "crsf.h"
#include <rom/gpio.h>


// CRSF-Konfiguration
#define CRSF_RX_PIN 17
#define CRSF_TX_PIN 17
#define GPIO_PIN_RCSIGNAL_UART_INV false

HardwareSerial CRSFSerial(1);  // UART1

unsigned char crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

// Calculate checksum
uint8_t crc8(const uint8_t * ptr, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i=0; i<len; i++)
      crc = crc8tab[crc ^ *ptr++];
  return crc;
}

// Halfduplex Mode
void setCRSF_RX() {
  portDISABLE_INTERRUPTS();

  gpio_set_direction((gpio_num_t)CRSF_RX_PIN, GPIO_MODE_INPUT);

#if GPIO_PIN_RCSIGNAL_UART_INV
  gpio_matrix_in(CRSF_RX_PIN, U1RXD_IN_IDX, true);
  gpio_pulldown_en((gpio_num_t)CRSF_RX_PIN);
  gpio_pullup_dis((gpio_num_t)CRSF_RX_PIN);
#else
  gpio_matrix_in(CRSF_RX_PIN, U1RXD_IN_IDX, false);
  gpio_pullup_en((gpio_num_t)CRSF_RX_PIN);
  gpio_pulldown_dis((gpio_num_t)CRSF_RX_PIN);
#endif

  portENABLE_INTERRUPTS();
}

void setCRSF_TX() {
  portDISABLE_INTERRUPTS();

  gpio_set_pull_mode((gpio_num_t)CRSF_TX_PIN, GPIO_FLOATING);
  gpio_set_direction((gpio_num_t)CRSF_TX_PIN, GPIO_MODE_OUTPUT);

#if GPIO_PIN_RCSIGNAL_UART_INV
  gpio_set_level((gpio_num_t)CRSF_TX_PIN, 0);
  gpio_matrix_in(GPIO_FUNC_IN_LOW, U1RXD_IN_IDX, false);  // RX detach
  gpio_matrix_out(CRSF_TX_PIN, U1TXD_OUT_IDX, true, false);
#else
  gpio_set_level((gpio_num_t)CRSF_TX_PIN, 1);
  gpio_matrix_in(GPIO_FUNC_IN_HIGH, U1RXD_IN_IDX, false); // RX detach
  gpio_matrix_out(CRSF_TX_PIN, U1TXD_OUT_IDX, false, false);
#endif

  portENABLE_INTERRUPTS();
}


void CRSF::begin() {
  CRSFSerial.begin(416666, SERIAL_8N1, CRSF_RX_PIN , CRSF_TX_PIN);
  delay(1000); // Longer delay for module initialization
  Serial.println("CRSF Serial1 started on GPIO 17");
}

// Call from main loop to update
void CRSF::update()
{
	handleSerialIn();
  //Serial.println("CRSF_update");
}

void CRSF::handleSerialIn()
{
	while (CRSFSerial.available())
	{
		uint8_t b = CRSFSerial.read();
		//_lastReceive = millis();
    //Serial.print("Empfangen: 0x");
    //Serial.println(b, HEX);

		_rxBuf[_rxBufPos++] = b;
		handleByteReceived();

		if (_rxBufPos == (sizeof(_rxBuf)/sizeof(_rxBuf[0])))
		{
			// Packet buffer filled and no valid packet found, dump the whole thing
			_rxBufPos = 0;
		}
	}
	//checkPacketTimeout();
	//checkLinkDown();
} 

void CRSF::handleByteReceived()
{
	bool reprocess;
	do
	{
		reprocess = false;
		if (_rxBufPos > 1)
		{
			uint8_t len = _rxBuf[1];
			// Sanity check the declared length, can't be shorter than Type, X, CRC
			if (len < 3 || len > CRSF_MAX_PACKET_LEN)
			{
				shiftRxBuffer(1);
				reprocess = true;
			}

			else if (_rxBufPos >= (len + 2))
			{
				uint8_t inCrc = _rxBuf[2 + len - 1];
				uint8_t crc = crc8(&_rxBuf[2], len - 1);
				if (crc == inCrc)
				{
					processPacketIn(len);
					shiftRxBuffer(len + 2);
					reprocess = true;
				}
				else
				{
Serial.printf("CRC error...\r\n");
					shiftRxBuffer(1);
					reprocess = true;
				}
			}  // if complete packet
		} // if pos > 1
	} while (reprocess);
}

void CRSF::processPacketIn(uint8_t len)
{
	const crsf_header_t *hdr = (crsf_header_t *)_rxBuf;
	const crsf_ext_header_t *extHdr = (crsf_ext_header_t *)_rxBuf;

	//if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
	{
		switch (hdr->type)
		{
		case CRSF_FRAMETYPE_GPS: // 0x02
			packetGps(hdr);
      //Serial.println("CRSF_FRAMETYPE_GPS");
			break;
		case CRSF_FRAMETYPE_BATTERY_SENSOR:
			packetBatterySensor(hdr);
      //Serial.println("CRSF_FRAMETYPE_BATTERY_SENSOR");
			break;
		case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: // 0x16
			packetChannelsPacked(hdr);
      //Serial.println("CRSF_FRAMETYPE_RC_CHANNELS_PACKED");
			break;
		case CRSF_FRAMETYPE_LINK_STATISTICS: // 0x14
			packetLinkStatistics(hdr);
      //Serial.println("CRSF_FRAMETYPE_LINK_STATISTICS");
			break;
		case CRSF_FRAMETYPE_BARO_ALTITUDE: // 0x09
			packetBaroAltitude(hdr);
      //Serial.println("CRSF_FRAMETYPE_BARO_ALTITUDE");
			break;
		case CRSF_FRAMETYPE_VARIO: // 0x07
			packetVario(hdr);
      //Serial.println("CRSF_FRAMETYPE_VARIO");
			break;
// Used by extended header frames (type in range 0x28 to 0x96)    
		case CRSF_FRAMETYPE_DEVICE_INFO: // 0x29
			packetDeviceInfo(extHdr); 
      //Serial.println("CRSF_FRAMETYPE_DEVICE_INFO");
			break;
		case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
#if 0
			Serial.printf("Device Address 0x%02x, Frame Type 0x%02x\r\n", hdr->device_addr, hdr->type);
			for(int i=0;i<hdr->frame_size;i++) {
				Serial.printf("0x%02x ", _rxBuf[i+2]);
			}
			Serial.printf("\r\n");
#endif
			//packetParameterSettingsEntry(extHdr);
			break;
		case CRSF_FRAMETYPE_RADIO_ID: // 0x3A
			//packetRadioId(extHdr);
			break;
		default:
			Serial.printf("Device Address 0x%02x, Frame Type 0x%02x\r\n", hdr->device_addr, hdr->type);
			for(int i=0;i<hdr->frame_size;i++) {
				Serial.printf("0x%02x ", _rxBuf[i+2]);
			}
			Serial.printf("\r\n");
			break;
		}
	}
}

void CRSF::crsfPrepareDataPacket(uint8_t packet[], int16_t channels[]) {

    // const uint8_t crc = crsf_crc8(&packet[2], CRSF_PACKET_SIZE-3);
    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
    */

    // packet[0] = UART_SYNC; //Header
    packet[0] = ELRS_ADDRESS; // Header
    packet[1] = 24;           // length of type (24) + payload + crc
    packet[2] = TYPE_CHANNELS;
    packet[3] = (uint8_t)(channels[0] & 0x07FF);
    packet[4] = (uint8_t)((channels[0] & 0x07FF) >> 8 | (channels[1] & 0x07FF) << 3);
    packet[5] = (uint8_t)((channels[1] & 0x07FF) >> 5 | (channels[2] & 0x07FF) << 6);
    packet[6] = (uint8_t)((channels[2] & 0x07FF) >> 2);
    packet[7] = (uint8_t)((channels[2] & 0x07FF) >> 10 | (channels[3] & 0x07FF) << 1);
    packet[8] = (uint8_t)((channels[3] & 0x07FF) >> 7 | (channels[4] & 0x07FF) << 4);
    packet[9] = (uint8_t)((channels[4] & 0x07FF) >> 4 | (channels[5] & 0x07FF) << 7);
    packet[10] = (uint8_t)((channels[5] & 0x07FF) >> 1);
    packet[11] = (uint8_t)((channels[5] & 0x07FF) >> 9 | (channels[6] & 0x07FF) << 2);
    packet[12] = (uint8_t)((channels[6] & 0x07FF) >> 6 | (channels[7] & 0x07FF) << 5);
    packet[13] = (uint8_t)((channels[7] & 0x07FF) >> 3);
    packet[14] = (uint8_t)((channels[8] & 0x07FF));
    packet[15] = (uint8_t)((channels[8] & 0x07FF) >> 8 | (channels[9] & 0x07FF) << 3);
    packet[16] = (uint8_t)((channels[9] & 0x07FF) >> 5 | (channels[10] & 0x07FF) << 6);
    packet[17] = (uint8_t)((channels[10] & 0x07FF) >> 2);
    packet[18] = (uint8_t)((channels[10] & 0x07FF) >> 10 | (channels[11] & 0x07FF) << 1);
    packet[19] = (uint8_t)((channels[11] & 0x07FF) >> 7 | (channels[12] & 0x07FF) << 4);
    packet[20] = (uint8_t)((channels[12] & 0x07FF) >> 4 | (channels[13] & 0x07FF) << 7);
    packet[21] = (uint8_t)((channels[13] & 0x07FF) >> 1);
    packet[22] = (uint8_t)((channels[13] & 0x07FF) >> 9 | (channels[14] & 0x07FF) << 2);
    packet[23] = (uint8_t)((channels[14] & 0x07FF) >> 6 | (channels[15] & 0x07FF) << 5);
    packet[24] = (uint8_t)((channels[15] & 0x07FF) >> 3);

    packet[25] = crc8(&packet[2], packet[1] - 1); // CRC
}

// Shift the bytes in the RxBuf down by cnt bytes
void CRSF::shiftRxBuffer(uint8_t cnt)
{
	// If removing the whole thing, just set pos to 0
	if (cnt >= _rxBufPos)
	{
		_rxBufPos = 0;
		return;
	}

	// Otherwise do the slow shift down
	uint8_t *src = &_rxBuf[cnt];
	uint8_t *dst = &_rxBuf[0];
	_rxBufPos -= cnt;
	uint8_t left = _rxBufPos;
	while (left--)
		*dst++ = *src++;
}

void CRSF::packetChannelsPacked(const crsf_header_t *p)
{
	crsf_channels_t *ch = (crsf_channels_t *)&p->data;
	_channels[0] = ch->ch0;
	_channels[1] = ch->ch1;
	_channels[2] = ch->ch2;
	_channels[3] = ch->ch3;
	_channels[4] = ch->ch4;
	_channels[5] = ch->ch5;
	_channels[6] = ch->ch6;
	_channels[7] = ch->ch7;
	_channels[8] = ch->ch8;
	_channels[9] = ch->ch9;
	_channels[10] = ch->ch10;
	_channels[11] = ch->ch11;
	_channels[12] = ch->ch12;
	_channels[13] = ch->ch13;
	_channels[14] = ch->ch14;
	_channels[15] = ch->ch15;

	for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
		_channels[i] = map(_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);

	_linkIsUp = true;
	_lastChannelsPacket = millis();

	memcpy(&_channelsPacked, ch, sizeof(_channelsPacked));
}

void CRSF::packetBatterySensor(const crsf_header_t *p)
{
	const crsf_sensor_battery_t *bat = (crsf_sensor_battery_t *)p->data;
	memcpy(&_batterySensor, bat, sizeof(_batterySensor));
	
	Serial.printf("[ Battery Sensor ]\r\n");
	Serial.printf("\tVoltage : %.1f V\r\n", be16toh(bat->voltage) / 10.0f);
	Serial.printf("\tCurrent : %.1f A\r\n", be16toh(bat->current) / 10.0f);
	uint32_t c = bat->capacity;
	Serial.printf("\tcapacity : %u mAh\r\n", be32toh(c));
	Serial.printf("\tremaining : %u %\r\n", bat->remaining);
}

/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */

void CRSF::packetLinkStatistics(const crsf_header_t *p)
{
	const crsfLinkStatistics_t *link = (crsfLinkStatistics_t *)p->data;
	memcpy(&_linkStatistics, link, sizeof(_linkStatistics));
	
	Serial.printf("[ Link Statistics ]\r\n");
	Serial.printf("\tUp Link ->\r\n");
	Serial.printf("\tRSSI 1 : %u\r\n", link->uplink_RSSI_1);
	Serial.printf("\tRSSI 2 : %u\r\n", link->uplink_RSSI_2);
	Serial.printf("\tLink Quality : %u\r\n", link->uplink_Link_quality);
	Serial.printf("\tSNR : %d\r\n", link->uplink_SNR);
	Serial.printf("\tActive Antenna : %u\r\n", link->active_antenna);
	Serial.printf("\tRF Mode : %u\r\n", link->rf_Mode);
	Serial.printf("\tTX Power : %u\r\n", link->uplink_TX_Power);
	Serial.printf("\tDown Link ->\r\n");
	Serial.printf("\tRSSI : %u\r\n", link->downlink_RSSI);
	Serial.printf("\tLink Quality : %u\r\n", link->downlink_Link_quality);
	Serial.printf("\tSNR : %d\r\n", link->downlink_SNR);
}

void CRSF::packetGps(const crsf_header_t *p)
{
	const crsf_sensor_gps_t *gps = (crsf_sensor_gps_t *)p->data;
	_gpsSensor.latitude = be32toh(gps->latitude);
	_gpsSensor.longitude = be32toh(gps->longitude);
	_gpsSensor.groundspeed = be16toh(gps->groundspeed);
	_gpsSensor.heading = be16toh(gps->heading);
	_gpsSensor.altitude = be16toh(gps->altitude);
	_gpsSensor.satellites = gps->satellites;
}

void CRSF::packetVario(const crsf_header_t *p)
{
	const crsf_sensor_vario_t *vario = (crsf_sensor_vario_t *)p->data;
	_varioSensor.verticalspd = be16toh(vario->verticalspd);
}

void CRSF::packetBaroAltitude(const crsf_header_t *p)
{
	const crsf_sensor_baro_altitude_t *baroAltitude = (crsf_sensor_baro_altitude_t *)p->data;
	_baroAltitudeSensor.altitude = be16toh(baroAltitude->altitude);
	_baroAltitudeSensor.verticalspd = be16toh(baroAltitude->verticalspd);
}

void CRSF::packetAttitude(const crsf_header_t *p)
{
	const crsf_sensor_attitude_t *attitude = (crsf_sensor_attitude_t *)p->data;
	_attitudeSensor.pitch = be16toh(attitude->pitch);
	_attitudeSensor.roll = be16toh(attitude->roll);
	_attitudeSensor.yaw = be16toh(attitude->yaw);
}

void CRSF::packetDeviceInfo(const crsf_ext_header_t *p)
{
	const uint8_t *data = (const uint8_t *)p->data;
	_device_address = p->orig_addr;
	
	strlcpy(_device_name, (const char *)&p->data[0], CRSF_MAX_NAME_LEN);
	int i = strlen((const char *)_device_name) + 1; // name + '\0'
	
	deviceInformationPacket_t *deviceInfo = (deviceInformationPacket_t *)&p->data[i];
	
	_deviceInfo.serialNo = be32toh(deviceInfo->serialNo); // ['E', 'L', 'R', 'S'], seen [0x00, 0x0a, 0xe7, 0xc6] // "Serial 177-714694" (value is 714694)
	_deviceInfo.hardwareVer = be32toh(deviceInfo->hardwareVer); // unused currently by us, seen [ 0x00, 0x0b, 0x10, 0x01 ] // "Hardware: V 1.01" / "Bootloader: V 3.06"
	_deviceInfo.softwareVer = be32toh(deviceInfo->softwareVer); // seen [ 0x00, 0x00, 0x05, 0x0f ] // "Firmware: V 5.15"
	_deviceInfo.fieldCnt = deviceInfo->fieldCnt;
	_deviceInfo.parameterVersion = deviceInfo->parameterVersion;
	
	Serial.printf("[ Device Information ]\r\n");
	Serial.printf("\tDevice Address 0x%02x\r\n", _device_address);
	Serial.printf("\tDevice Name %s\r\n", _device_name);
	Serial.printf("\tSerial No 0x%08x\r\n", _deviceInfo.serialNo);
	Serial.printf("\tSerial No %c%c%c%c\r\n", _deviceInfo.serialNo >> 24, (_deviceInfo.serialNo >> 16) & 0xff, (_deviceInfo.serialNo >> 8) & 0xff, _deviceInfo.serialNo & 0xff);
	Serial.printf("\tHardware Version 0x%08x\r\n", _deviceInfo.hardwareVer);
	Serial.printf("\tFoftware Version 0x%08x\r\n", _deviceInfo.softwareVer);
	Serial.printf("\tField Count %d\r\n", _deviceInfo.fieldCnt);
	Serial.printf("\tParameter Version %d\r\n", _deviceInfo.parameterVersion);
}

void CRSF::CrsfWritePacket(uint8_t packet[], uint8_t packetLength) {
  setCRSF_TX(); // Send
  CRSFSerial.write(packet, packetLength);
  CRSFSerial.flush();
  setCRSF_RX();            // Zurück auf Empfang
 
  // Debug: print first few bytes of packet
  static int packetCount = 0;
  if (packetCount++ % 100 == 0) {
    Serial.print("CRSF Packet: ");
    for(int i = 0; i < 26; i++) {
      Serial.print(packet[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// Stub functions (not needed for basic test)
void CRSF::crsfPrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value) {
    packetCmd[0] = ELRS_ADDRESS;
    packetCmd[1] = 6; // length of Command (4) + payload + crc
    packetCmd[2] = TYPE_SETTINGS_WRITE;
    packetCmd[3] = ELRS_ADDRESS;
    packetCmd[4] = ADDR_RADIO;
    packetCmd[5] = command;
    packetCmd[6] = value;
    packetCmd[7] = crc8(&packetCmd[2], packetCmd[1] - 1); // CRC
}
