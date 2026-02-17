
/*
 * Combined Room Sensors Sketch (Zigbee End Device)
 * - Cozir CO2 Sensor (SoftwareSerial on 20/19)
 * - LD2410 Radar Sensor (Hardware Serial)
 * - BH1750 Light Sensor (I2C on 22/21)
 * - NeoPixel Status LED (Pin 8)
 * - Zigbee Endpoints: CO2, Temp+Hum, Light, Occupancy
 *
 * NOTE: Select "Zigbee ED (End Device)" in Tools -> Zigbee Mode
 * NOTE: Select a Partition Scheme with Zigbee (e.g., "Zigbee 4MB with spiffs")
 */

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Arduino.h"
#include "Zigbee.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// --- Libraries ---
#include "cozir.h"
#include <ld2410.h>
#include <BH1750.h>

// --- FILTERING CLASS ---
template <typename T, int SIZE>
class MovingQuantile {
  T buffer[SIZE];
  uint8_t idx = 0;
  uint8_t count = 0;
  float quantile;

public:
  MovingQuantile(float q = 0.5) : quantile(q) {}

  void add(T value) {
    buffer[idx] = value;
    idx = (idx + 1) % SIZE;
    if (count < SIZE) count++;
  }

  T get() {
    if (count == 0) return 0;
    T sorted[SIZE];
    for (uint8_t i = 0; i < count; i++) sorted[i] = buffer[i];
    // Simple Bubble Sort
    for (uint8_t i = 0; i < count - 1; i++) {
      for (uint8_t j = 0; j < count - i - 1; j++) {
        if (sorted[j] > sorted[j + 1]) {
          T temp = sorted[j];
          sorted[j] = sorted[j + 1];
          sorted[j + 1] = temp;
        }
      }
    }
    int qIdx = (int)(quantile * (count - 1));
    return sorted[qIdx];
  }
};

// --- ZIGBEE CONFIG ---
#define EP_CO2 10
#define EP_TEMP_HUM 11  // Combined endpoint for Temp & Humidity
#define EP_LIGHT 13
#define EP_OCCUPANCY 14

uint8_t button = BOOT_PIN; 

// Create Zigbee Sensor Objects
ZigbeeCarbonDioxideSensor zbCo2(EP_CO2);
ZigbeeTempSensor          zbTempHum(EP_TEMP_HUM); // Handles both Temp and Hum
ZigbeeIlluminanceSensor   zbLight(EP_LIGHT);
ZigbeeOccupancySensor     zbOcc(EP_OCCUPANCY);

// --- PIN DEFINITIONS & OBJECTS ---

// 1. Cozir CO2 (SoftwareSerial)
SoftwareSerial sws(20, 19); //RX TX
COZIR czr(&sws);
C0ZIRParser czrp;

// 2. BH1750 Light Sensor (I2C)
BH1750 lightMeter(0x5C);

// 3. NeoPixel (Pin 8)
#define NEOPIXEL_PIN 8
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// 4. LD2410 Radar (Hardware Serial)
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 7
#define RADAR_TX_PIN 5 
ld2410 radar;

// --- TIMERS ---
uint32_t lastRadarReport = 0;
uint32_t lastCozirReport = 0;
uint32_t lastLightReport = 0;

// --- FILTERING ---
MovingQuantile<uint32_t, 5> co2Filter(0.5);
MovingQuantile<float, 5> tempFilter(0.5);
MovingQuantile<float, 5> humFilter(0.5);
MovingQuantile<float, 5> lightFilter(0.5);

const uint32_t RADAR_REPORT_INTERVAL = 1000;
const uint32_t COZIR_REPORT_INTERVAL = 5000;
const uint32_t LIGHT_REPORT_INTERVAL = 20000;

void setup()
{
  // 1. Monitor Serial
  MONITOR_SERIAL.begin(115200);
  delay(2000);
  MONITOR_SERIAL.println(F("\n--- Combined Room Sensors (Zigbee ED) Init ---"));

  // 2. Button Init
  pinMode(button, INPUT_PULLUP);

  // 3. NeoPixel Init
  MONITOR_SERIAL.println(F("[NeoPixel] Init..."));
  pixels.begin();
  pixels.show(); 

  // 4. Cozir Setup
  MONITOR_SERIAL.println(F("[Cozir] Init..."));
  sws.begin(9600);
  czr.init();
  czrp.init();
  czr.setOperatingMode(CZR_STREAMING);
  czr.setOutputFields(CZR_DEFAULT);

  // 5. BH1750 Setup
  MONITOR_SERIAL.println(F("[BH1750] Init..."));
  Wire.begin(22, 21); 
  if (lightMeter.begin()) {
    MONITOR_SERIAL.println(F("[BH1750] OK"));
  } else {
    MONITOR_SERIAL.println(F("[BH1750] Error initialising"));
  }

  // 6. Radar Setup
  MONITOR_SERIAL.print(F("[Radar] Connecting to RX:"));
  MONITOR_SERIAL.print(RADAR_RX_PIN);
  MONITOR_SERIAL.print(F(" TX:"));
  MONITOR_SERIAL.println(RADAR_TX_PIN);

  #if defined(ESP32)
    RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  #elif defined(__AVR_ATmega32U4__)
    RADAR_SERIAL.begin(256000);
  #endif

  if(radar.begin(RADAR_SERIAL))
  {
    MONITOR_SERIAL.println(F("[Radar] OK"));
  }
  else
  {
    MONITOR_SERIAL.println(F("[Radar] Not connected"));
  }

  // 7. Zigbee Setup
  MONITOR_SERIAL.println(F("[Zigbee] Init..."));
  
  // Set Info
  zbCo2.setManufacturerAndModel("Espressif", "MultiSensor_CO2");
  
  // Configure Combined Temp/Hum Sensor
  zbTempHum.setManufacturerAndModel("Espressif", "MultiSensor_TempHum");
  // Enable Humidity on this sensor (Min:0%, Max:100%, Tolerance:1%)
  zbTempHum.addHumiditySensor(0, 100, 1.0); 

  zbLight.setManufacturerAndModel("Espressif", "MultiSensor_Light");
  zbOcc.setManufacturerAndModel("Espressif", "MultiSensor_Occ");

  // Add Endpoints
  Zigbee.addEndpoint(&zbCo2);
  Zigbee.addEndpoint(&zbTempHum);
  Zigbee.addEndpoint(&zbLight);
  Zigbee.addEndpoint(&zbOcc);

  if (!Zigbee.begin()) {
    MONITOR_SERIAL.println(F("[Zigbee] Failed to start! Rebooting..."));
    ESP.restart();
  } else {
    MONITOR_SERIAL.println(F("[Zigbee] Started successfully!"));
  }
  
  MONITOR_SERIAL.println(F("[Zigbee] Connecting to network..."));
  while (!Zigbee.connected()) {
    MONITOR_SERIAL.print(".");
    delay(100);
  }
  MONITOR_SERIAL.println();
  
  // Configure Reporting
  zbCo2.setReporting(1, 30, 10);     
  
  // Temp Reporting (Min, Max, Delta)
  zbTempHum.setReporting(1, 30, 0.5);   
  // Humidity Reporting (Min, Max, Delta) - Separate function
  zbTempHum.setHumidityReporting(1, 30, 1.0);
  
  zbLight.setReporting(1, 30, 50);   
  
  // NOTE: zbOcc (Binary Sensor) usually reports on state change automatically. 
  // Removed setReporting() as it is not supported by the Occupancy class.

  MONITOR_SERIAL.println(F("--- Setup Complete ---\n"));
}

void loop()
{
  // --- READ RADAR (Must be called frequently) ---
  radar.read();

  uint32_t currentMillis = millis();

  // --- REPORT RADAR & UPDATE NEOPIXEL ---
  if(radar.isConnected() && (currentMillis - lastRadarReport > RADAR_REPORT_INTERVAL))
  {
    lastRadarReport = currentMillis;
    bool presence = radar.presenceDetected();
    
    // Update Zigbee
    zbOcc.setOccupancy(presence);
    
    if(presence)
    {
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red
      
      // if(radar.stationaryTargetDetected())
      // {
      //   MONITOR_SERIAL.print(F("[Radar] Stat Dist: "));
      //   MONITOR_SERIAL.print(radar.stationaryTargetDistance());
      //   MONITOR_SERIAL.print(F("cm Energy: "));
      //   MONITOR_SERIAL.print(radar.stationaryTargetEnergy());
      //   MONITOR_SERIAL.print(' ');
      // }
      // if(radar.movingTargetDetected())
      // {
      //   MONITOR_SERIAL.print(F("[Radar] Mov Dist: "));
      //   MONITOR_SERIAL.print(radar.movingTargetDistance());
      //   MONITOR_SERIAL.print(F("cm Energy: "));
      //   MONITOR_SERIAL.print(radar.movingTargetEnergy());
      // }
      // MONITOR_SERIAL.println();
    }
    else
    {
      pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green
      MONITOR_SERIAL.println(F("[Radar] No target"));
    }
    pixels.show();
  }

  // --- READ COZIR STREAM ---


  // --- REPORT COZIR (Serial Monitor) ---
  if (currentMillis - lastCozirReport > COZIR_REPORT_INTERVAL) {
    lastCozirReport = currentMillis;

    while (sws.available()) {
      int field = czrp.nextChar(sws.read());
      if (field != 0) {
        // Filter CO2 (Max 4000)
        if (field == 'Z') {
          uint32_t co2 = czrp.CO2();
          if (co2 <= 4000) {
            co2Filter.add(co2);
            zbCo2.setCarbonDioxide(co2Filter.get());
          }
        }
        // Filter Temp (0-100 C)
        else if (field == 'T') {
          float t = czrp.celsius();
          if (t >= 0 && t <= 100) {
            tempFilter.add(t);
            zbTempHum.setTemperature(tempFilter.get());
          }
        }
        // Filter Humidity
        else if (field == 'H') {
          float h = czrp.humidity();
          if (h >= 0 && h <= 100) {
            humFilter.add(h);
            zbTempHum.setHumidity(humFilter.get());
          }
        }
      }
    }
    uint32_t c = czrp.CO2(); 
    float t = czrp.celsius();
    float h = czrp.humidity();

    MONITOR_SERIAL.print(F("[Cozir] Temp: "));
    MONITOR_SERIAL.print(t);
    MONITOR_SERIAL.print(F(" C\tHum: "));
    MONITOR_SERIAL.print(h);
    MONITOR_SERIAL.print(F("%\tCO2: "));
    MONITOR_SERIAL.println(c);
  }

  // --- REPORT LIGHT ---
  if (currentMillis - lastLightReport > LIGHT_REPORT_INTERVAL) {
    lastLightReport = currentMillis;
      
      float lux = lightMeter.readLightLevel();
      lightFilter.add(lux);
      float filteredLux = lightFilter.get();
      
      // 1. Calculate the Zigbee "MeasuredValue"
      // Formula: 10000 * log10(Lux) + 1
      uint16_t zigbeeRaw;
      
      if (filteredLux <= 0.0001) { 
        zigbeeRaw = 0; // Darkness
      } else if (filteredLux > 1000000) {
        zigbeeRaw = 0xfffe; // Maximum defined value
      } else {
        zigbeeRaw = (uint16_t)(10000 * log10(filteredLux) + 1);
      }

      // 2. Log both for debugging
      MONITOR_SERIAL.print(F("[Light] Real Lux: "));
      MONITOR_SERIAL.println(lux);
      // MONITOR_SERIAL.print(F(" | Zigbee Raw: "));
      // MONITOR_SERIAL.println(zigbeeRaw);
      
      // 3. Send the RAW value to Zigbee
      zbLight.setIlluminance(zigbeeRaw);
  }

  // --- BUTTON (Factory Reset) ---
  if (digitalRead(button) == LOW) { 
    delay(100); // Debounce
    uint32_t startTime = millis();
    while (digitalRead(button) == LOW) {
      delay(50);
      if ((millis() - startTime) > 3000) {
        MONITOR_SERIAL.println(F("Resetting Zigbee to factory and rebooting..."));
        pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Blue for reset
        pixels.show();
        delay(1000);
        Zigbee.factoryReset();
      }
    }
    // Manually trigger report on button click for testing
    zbCo2.report();
    zbTempHum.report(); // Reports both Temp & Hum
    zbLight.report();
    zbOcc.report();
  }
}