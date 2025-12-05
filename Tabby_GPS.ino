#include <Arduino.h>
#include <cctype>
#include <TinyGPSPlus.h>
#include "driver/twai.h"
#include <stdio.h>

// CAN Setup
#define CAN_TX_GPIO     (gpio_num_t)23
#define CAN_RX_GPIO     (gpio_num_t)22
#define CANBUS_SPEED    500000   // 500kbps
#define CAN_QUEUE_LENGTH 32

// GPS Serial Setup
#define RXD2 16
#define TXD2 17
#define PPS_PIN 4

#define GPS_BAUD 9600

bool location_data_ready = false;
bool gps_started = false;

// Initialise dependancies
TinyGPSPlus gps;

// Time between new data send steps
#define STEP_INTERVAL 500

// Structures
typedef struct struct_gps {
  float gps_lat;
  float gps_lng;
} struct_gps;

struct_gps GPSData;
HardwareSerial gpsSerial(2);

// initialise CANBus
void canbus_init(void) {
  // Configure TWAI (CAN)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();  // Accept all IDs
 
    // Install and start TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI driver installed.");
    } else {
        Serial.println("Failed to install TWAI driver.");
        while (1);
    }

    if (twai_start() == ESP_OK) {
        Serial.println("TWAI driver started.");
    } else {
        Serial.println("Failed to start TWAI driver.");
        while (1);
    }
}

void send_location_data() {
  Serial.print("Sending location data: Lat=");
  Serial.print(GPSData.gps_lat, 6);
  Serial.print(", Lng=");
  Serial.println(GPSData.gps_lng, 6);

  // Prepare CAN message
  twai_message_t message = {};
  message.identifier = 0x430; // Example CAN ID, adjust as needed
  message.data_length_code = 8; // 4 bytes for lat, 4 bytes for lng
  message.flags = 0;

  // Pack floats into CAN data (little-endian)
  float lat = GPSData.gps_lat;
  float lng = GPSData.gps_lng;
  memcpy(&message.data[0], &lat, 4);
  memcpy(&message.data[4], &lng, 4);

  esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(10));
  if (err == ESP_OK) {
    Serial.println("CAN message sent successfully.");
  } else {
    Serial.printf("Failed to transmit TWAI message ID 0x%03X: %d\n", message.identifier, err);
  }
}

void check_gps() {
    gps_started = true;

    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    // Update location if valid
    if (gps.location.isValid() && gps.location.lat() != 0.0f && gps.location.lng() != 0.0f) {
      Serial.println("Latest location known");
      GPSData.gps_lat = gps.location.lat();
      GPSData.gps_lng = gps.location.lng();
      location_data_ready = true;
    } else {
      Serial.println("Latest location not known or invalid");
    }
}

void setup() {
  Serial.begin(115200);
  canbus_init();

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  pinMode(PPS_PIN, INPUT_PULLUP);
}

void loop() {
  static unsigned long lastSendTime = 0;
  unsigned long currentMillis = millis();

  // Always read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Update location if valid
  if (gps.location.isValid() && gps.location.lat() != 0.0f && gps.location.lng() != 0.0f) {
    GPSData.gps_lat = gps.location.lat();
    GPSData.gps_lng = gps.location.lng();
    location_data_ready = true;
  } else {
    location_data_ready = false;
  }

  // Send every 500ms
  if (location_data_ready && (currentMillis - lastSendTime >= STEP_INTERVAL)) {
    send_location_data();
    lastSendTime = currentMillis;
  }
}