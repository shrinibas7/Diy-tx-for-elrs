#include "crsf.h"

#define CRSF_TX_PIN 17

void CRSF::begin() {
  Serial1.begin(416666, SERIAL_8N1, -1, CRSF_TX_PIN);
  delay(1000); // Longer delay for module initialization
  Serial.println("CRSF Serial1 started on GPIO 17");
}

void CRSF::crsfPrepareDataPacket(uint8_t packet[], int16_t channels[]) {
  // Ultra simple packet - just send basic channel data
  packet[0] = 0xEE; // ELRS address
  packet[1] = 24;   // Length
  packet[2] = 0x16; // RC channels type
  
  // Simple channel data (all channels at midpoint)
  for(int i = 3; i < 25; i++) {
    packet[i] = 0x00;
  }
  
  // Set some basic channel values
  uint16_t ch1 = 992; // Midpoint
  packet[3] = ch1 & 0xFF;
  packet[4] = (ch1 >> 8) & 0xFF;
  
  packet[25] = 0x00; // Simple CRC (we'll fix this later)
}

void CRSF::CrsfWritePacket(uint8_t packet[], uint8_t packetLength) {
  Serial1.write(packet, packetLength);
  Serial1.flush();
  
  // Debug: print first few bytes of packet
  static int packetCount = 0;
  if (packetCount++ % 100 == 0) {
    Serial.print("CRSF Packet: ");
    for(int i = 0; i < 6; i++) {
      Serial.print(packet[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// Stub functions (not needed for basic test)
void CRSF::crsfPrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value) {}