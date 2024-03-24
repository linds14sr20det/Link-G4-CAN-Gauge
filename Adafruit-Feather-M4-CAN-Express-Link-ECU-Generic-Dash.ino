//
// This script will attempt to parse Generic Dash format CAN messages from a Link G4 / G4+ / G4X ECU
// Information on these can be found in the "Device Specific CAN Information" section of the Link help file
// Or here if you are being lazy; https://www.akao.co.uk/manuals/Link%20G4+%20Manual/device_specific_can_informatio.htm#genericdash
//
// This code was written for use with the Adafruit Feather M4 CAN Express, more information here;
// https://learn.adafruit.com/adafruit-feather-m4-can-express
//
// Written by JBG 20211121
//

//NOTE: White wire is Can Hi, Blue wire is Can Lo

// Local header file with configuration and constants for each value
#include "Adafruit-Feather-M4-CAN-Express-Link-ECU-Generic-Dash.h"

#include <CAN.h>               // CAN Adafruit Fork by adafruit v1.2.1
#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"
#include <Adafruit_SPIFlash.h>    // SPI / QSPI flash library
#include <Adafruit_ImageReader.h> // Image-reading functions

#define maximumPacketSize 64
int nextCANByte;
unsigned char CANFrameBuffer[maximumPacketSize];
volatile unsigned char GenericDash[GenericDashFrames][GenericDashBytes];

unsigned long currentMillis = 0;
unsigned long updateDisplayMillis = 0;
unsigned long updateDisplayMilliRate = SERIAL_UPDATE_MILLISECONDS; // Milliseconds between serial updates

signed int RPM = 0;
float airFuelRatio = 0; 
signed int knockCount = 0; 
float boostPressure = 0; 
signed int coolantTemperature = 0; 
signed int oilTemperature = 0; 
float oilPressure = 0;
signed int sportMode = 0;
signed int launchControl = 0;

#ifdef ESP8266
   #define TFT_CS   0
   #define TFT_DC   15
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
   #define TFT_CS   15
   #define TFT_DC   33
#elif defined(TEENSYDUINO)
   #define TFT_DC   10
   #define TFT_CS   4
#elif defined(ARDUINO_STM32_FEATHER)
   #define TFT_DC   PB4
   #define TFT_CS   PA15
#elif defined(ARDUINO_NRF52832_FEATHER)  /* BSP 0.6.5 and higher! */
   #define TFT_DC   11
   #define TFT_CS   31
#elif defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
   #define TFT_DC   P5_4
   #define TFT_CS   P5_3
#else
    // Anything else, defaults!
   #define TFT_CS   9
   #define TFT_DC   10
#endif

#define TFT_RST -1


// SPI or QSPI flash filesystem (i.e. CIRCUITPY drive)
  #if defined(__SAMD51__) || defined(NRF52840_XXAA)
    Adafruit_FlashTransport_QSPI flashTransport(PIN_QSPI_SCK, PIN_QSPI_CS,
      PIN_QSPI_IO0, PIN_QSPI_IO1, PIN_QSPI_IO2, PIN_QSPI_IO3);
  #else
    #if (SPI_INTERFACES_COUNT == 1)
      Adafruit_FlashTransport_SPI flashTransport(SS, &SPI);
    #else
      Adafruit_FlashTransport_SPI flashTransport(SS1, &SPI1);
    #endif
  #endif
  Adafruit_SPIFlash    flash(&flashTransport);
  FatFileSystem        filesys;
  Adafruit_ImageReader reader(filesys); // Image-reader, pass in flash filesys

// Use hardware SPI and the above for CS/DC
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  // Init hardware
  Serial.begin(115200);

  Serial.println("3.5\" HX8357D FeatherWing Test!"); 

  tft.begin();

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(HX8357_RDPOWMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDCOLMOD);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDDIM);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDDSDR);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); 
  
  Serial.println(F("Benchmark                Time (microseconds)"));

  tft.setRotation(1);
  tft.fillScreen(HX8357_BLACK);

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // Turn off CAN IC standby mode
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // Turn on CAN IC voltage booster

  
  // start the CAN bus at 500 kbps
  if (!CAN.begin(CAN_BAUD)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // Filter out only what messages we are interested in
  CAN.filter(ECU_HEADER, 0x7FF);

  // Add callback function for when a message is received
  CAN.onReceive(CANReceiveCallback);

  // Print a message to say we are now running
  Serial.println("Generic Dash CAN receiver started.");

  if(!flash.begin()) {
    Serial.println(F("flash begin() failed"));
    for(;;);
  }
  if(!filesys.begin(&flash)) {
    Serial.println(F("filesys begin() failed"));
    for(;;);
  }

  ImageReturnCode stat;
  stat = reader.drawBMP("/sd/nissan_logo.bmp", tft, 155, 93);
  reader.printStatus(stat);   // How'd we do?
  delay(1000);
}

void CANReceiveCallback(int packetSize) {
  // We have received a CAN packet, process it below

  switch (CAN.packetId()) {
    case ECU_HEADER:
      for (int i = 0; i < packetSize; i++) {
        if (CAN.peek() == -1) { break; }
        CANFrameBuffer[i] = (char)CAN.read();
      }
      for (int i = 0; i < sizeof(GenericDash[0]); i++) {
        GenericDash[CANFrameBuffer[0]][i] = CANFrameBuffer[i];
      }
      break;
    default:
      Serial.print("Received unknown ");
      if (CAN.packetExtended()) { Serial.print("extended "); } // Extended packet (29-bit)
      if (CAN.packetRtr()) { Serial.print("RTR "); } // Remote transmission request, packet contains no data
      Serial.print("packet with id 0x");
      Serial.print(CAN.packetId(), HEX);
  
      if (CAN.packetRtr()) {
        Serial.print(" and requested length ");
        Serial.println(CAN.packetDlc());
      } else {
        Serial.print(" and length ");
        Serial.print(packetSize);
        Serial.print(": ");
        for (int i = 0; i < packetSize; i++) {
          if (CAN.peek() == -1) { break; }
          nextCANByte = CAN.read();
          for (int i = 0; i < 8; i++) {
            Serial.print((bool)bitRead(nextCANByte, i));
          }
        }
        Serial.println();
      }
      break;
  }
}

// unsigned int getBetweenColourByPercent(float value = 0.5 /* 0-1 */, unsigned int highColor = 0x0000FF, unsigned int lowColor = 0xFF0000) {
//     unsigned int r = highColor >> 16;
//     unsigned int g = highColor >> 8 & 0xFF;
//     unsigned int b = highColor & 0xFF;

//     r += ((lowColor >> 16) - r) * value;
//     g += ((lowColor >> 8 & 0xFF) - g) * value;
//     b += ((lowColor & 0xFF) - b) * value;

//     return (r << 16 | g << 8 | b);
// }

unsigned long printRPM() {
  unsigned long start;

  char sensorValue[5];
  snprintf(sensorValue, 5, "%-4d", RPM); // Left-justified message
  
  tft.setCursor(10, 60);
  tft.setTextColor(HX8357_WHITE, HX8357_BLACK);  tft.setTextSize(4);
  tft.print(sensorValue);
  tft.setCursor(10, 92);
  tft.setTextColor(HX8357_CYAN, HX8357_BLACK);  tft.setTextSize(2);
  tft.println("RPM");

  return micros() - start;
}

unsigned long printAFR() {
  unsigned long start;
  
  char sensorValue[5];
  snprintf(sensorValue, 5, "%-.1f", airFuelRatio); // Left-justified message

  int backgroundColor = HX8357_BLACK;
  // backgroundColor = getBetweenColourByPercent(0.5);

  // Serial.print("Background Color: ");
  // Serial.println(backgroundColor);
  
  
  tft.setCursor(10, 160);
  tft.setTextColor(HX8357_WHITE, backgroundColor);  tft.setTextSize(4);
  tft.print(sensorValue);
  tft.setCursor(10, 192);
  tft.setTextColor(HX8357_CYAN, HX8357_BLACK);  tft.setTextSize(2);
  tft.println("AFR");

  return micros() - start;
}

unsigned long printKnockCount() {
  unsigned long start;
  
  char sensorValue[5];
  snprintf(sensorValue, 5, "%-4d", knockCount); // Left-justified message

  int backgroundColor = HX8357_BLACK;
  if (knockCount > 0) {
    backgroundColor = HX8357_RED;
  }
  
  tft.setCursor(10, 160);
  tft.setTextColor(HX8357_WHITE, backgroundColor);  tft.setTextSize(4);
  tft.print(sensorValue);
  tft.setCursor(10, 192);
  tft.setTextColor(HX8357_CYAN, HX8357_BLACK);  tft.setTextSize(2);
  tft.println("KNOCK");

  return micros() - start;
}

unsigned long printBoost() {
  unsigned long start;

  char sensorValue[5];
  if (boostPressure < -10) {
    snprintf(sensorValue, 5, "%-4.0f", boostPressure);
  } else if (boostPressure < 0) {
   snprintf(sensorValue, 5, "%-2.1f", boostPressure);
  } else { 
    snprintf(sensorValue, 5, "%-4.1f", boostPressure);
  }

  int backgroundColor = HX8357_BLACK;
  if (boostPressure > 22) {
    backgroundColor = HX8357_RED;
  }
  
  tft.setCursor(10, 260);
  tft.setTextColor(HX8357_WHITE, backgroundColor);  tft.setTextSize(4);
  tft.print(sensorValue);
  tft.setCursor(10, 292);
  tft.setTextColor(HX8357_CYAN, HX8357_BLACK);  tft.setTextSize(2);
  tft.println("BOOST");

  return micros() - start;
}

unsigned long printECT() {
  unsigned long start;
  char sensorValue[5];
  snprintf(sensorValue, 5, "%4d", coolantTemperature); // Left-justified message

  int backgroundColor = HX8357_BLACK;
  if (coolantTemperature >= 100) {
    backgroundColor = HX8357_RED;
  }
  
  tft.setCursor(380, 60);
  tft.setTextColor(HX8357_WHITE, backgroundColor);  tft.setTextSize(4);
  tft.print(sensorValue);
  tft.setCursor(380, 92);
  tft.setTextColor(HX8357_CYAN, HX8357_BLACK);  tft.setTextSize(2);
  tft.println("ECT");

  return micros() - start;
}

unsigned long printOilTemp() {
  unsigned long start;
  char sensorValue[5];
  snprintf(sensorValue, 5, "%4d", oilTemperature); // Left-justified message
  
  int backgroundColor = HX8357_BLACK;
  if (oilTemperature > 120) {
    backgroundColor = HX8357_RED;
  }

  tft.setCursor(380, 160);
  tft.setTextColor(HX8357_WHITE, backgroundColor);  tft.setTextSize(4);
  tft.print(sensorValue);
  tft.setCursor(380, 192);
  tft.setTextColor(HX8357_CYAN, HX8357_BLACK);  tft.setTextSize(2);
  tft.println("Oil Temp");

  return micros() - start;
}

unsigned long printOilPressure() {
  unsigned long start;
  char sensorValue[5];

  if (oilPressure > 100) {
      snprintf(sensorValue, 5, "%4.0f", oilPressure);
  } else {
      snprintf(sensorValue, 5, "%4.1f", oilPressure);
  }


  int backgroundColor = HX8357_BLACK;
  if (oilPressure < 20) {
    backgroundColor = HX8357_RED;
  }

  tft.setCursor(380, 260);
  tft.setTextColor(HX8357_WHITE, backgroundColor);  tft.setTextSize(4);
  tft.print(sensorValue);
  tft.setCursor(380, 292);
  tft.setTextColor(HX8357_CYAN, HX8357_BLACK);  tft.setTextSize(2);
  tft.println("Oil Pres");

  return micros() - start;
}

unsigned long printLaunchControl() {
  unsigned long start;

  if(launchControl == 1) {
    tft.fillCircle(300, 290, 20, HX8357_GREEN);
  } else {
    tft.fillCircle(300, 290, 20, HX8357_BLACK);
  }

  return micros() - start;
}

unsigned long printSportMode() {
  unsigned long start;
 
  if(sportMode == 1) {
    tft.fillCircle(180, 290, 20, HX8357_RED);
  } else {
    tft.fillCircle(180, 290, 20, HX8357_BLACK);
  }

  return micros() - start;
}

unsigned long printShiftLight() {
  unsigned long start;

  Serial.println(micros());
  Serial.println("");
  if ((micros()/100000)%2) {
    if(RPM > 7400) {
      tft.fillCircle(60, 20, 20, HX8357_RED);
      tft.fillCircle(120, 20, 20, HX8357_YELLOW);
      tft.fillCircle(180, 20, 20, HX8357_GREEN);
      tft.fillCircle(240, 20, 20, HX8357_GREEN);
      tft.fillCircle(300, 20, 20, HX8357_GREEN);
      tft.fillCircle(360, 20, 20, HX8357_YELLOW);
      tft.fillCircle(420, 20, 20, HX8357_RED);
    } else if (RPM > 7100) {
      tft.fillCircle(120, 20, 20, HX8357_YELLOW);
      tft.fillCircle(180, 20, 20, HX8357_GREEN);
      tft.fillCircle(240, 20, 20, HX8357_GREEN);
      tft.fillCircle(300, 20, 20, HX8357_GREEN);
      tft.fillCircle(360, 20, 20, HX8357_YELLOW);
    } else if (RPM > 6800) {
      tft.fillCircle(180, 20, 20, HX8357_GREEN);
      tft.fillCircle(240, 20, 20, HX8357_GREEN);
      tft.fillCircle(300, 20, 20, HX8357_GREEN);
    }
  } else {
    tft.fillCircle(60, 20, 20, HX8357_BLACK);
    tft.fillCircle(120, 20, 20, HX8357_BLACK);
    tft.fillCircle(180, 20, 20, HX8357_BLACK);
    tft.fillCircle(240, 20, 20, HX8357_BLACK);
    tft.fillCircle(300, 20, 20, HX8357_BLACK);
    tft.fillCircle(360, 20, 20, HX8357_BLACK);
    tft.fillCircle(420, 20, 20, HX8357_BLACK);
  }
  return micros() - start;
}

void loop() {
  // // Update our timer
  currentMillis = millis();

  // // Call this every updateDisplayMilliRate milliseconds
  if (currentMillis - updateDisplayMillis > updateDisplayMilliRate) {
    updateDisplayMillis = currentMillis;
    
    RPM = (signed int)getGenericDashValue(GenericDash, ECU_ENGINE_SPEED_RPM);
    //RPM = 7500;
    Serial.print("RPM: "); Serial.print(RPM); Serial.println(" RPM");
    printRPM();
    

    airFuelRatio = ((float)getGenericDashValue(GenericDash, ECU_LAMBDA_1_LAMBDA))*14.7;
    // airFuelRatio = 14.7;
    Serial.print("AFR: "); Serial.println(airFuelRatio);
    // printAFR();


    knockCount = ((signed int)getGenericDashValue(GenericDash, ECU_KNOCK_COUNT_GLOBAL));
    // knockCount = 10.7;
    Serial.print("Knock level: "); Serial.println(knockCount);
    printKnockCount();


    boostPressure = ((signed int)getGenericDashValue(GenericDash, ECU_MGP_KPA)) * 0.145038;
    // boostPressure += 1; // 137 * -0.145038;
    Serial.print("Boost: "); Serial.print(boostPressure); Serial.println(" PSI");
    printBoost();
    

    coolantTemperature = (signed int)getGenericDashValue(GenericDash, ECU_ENGINE_COOLANT_TEMPERATURE_DEGREES_C);
    // coolantTemperature = 100;
    Serial.print("ECT: "); Serial.print(coolantTemperature); Serial.println(" °C");
    printECT();


    oilTemperature = (signed int)getGenericDashValue(GenericDash, ECU_OIL_TEMPERATURE_DEGREES_C);
    // oilTemperature = 121;
    Serial.print("Oil Temp: "); Serial.print(oilTemperature); Serial.println(" °C");
    printOilTemp();

    
    oilPressure = ((signed int)getGenericDashValue(GenericDash, ECU_OIL_PRESSURE_KPA)) * 0.145038;
    // oilPressure = 137 * 0.145038;
    Serial.print("Oil Pressure: "); Serial.print(oilPressure); Serial.println(" PSI");
    printOilPressure();


    launchControl = ((signed int)getGenericDashValue(GenericDash, ECU_LAUNCH_CONTROL_STATUS));
    // launchControl = 1;
    Serial.print("Launch Control: "); Serial.println(launchControl);
    printLaunchControl();


    sportMode = ((signed int)getGenericDashValue(GenericDash, ECU_SPORT_MODE));
    // sportMode = 1;
    Serial.print("Sport Mode: "); Serial.println(sportMode);
    Serial.println("");
    printSportMode();

    printShiftLight();
  }
}