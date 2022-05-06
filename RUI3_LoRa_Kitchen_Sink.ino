#include "rak1901.h"
#include "rak1902.h"
#include "rak1903.h"
#include "Seeed_BME280.h" // http://librarymanager/All#Seeed_BME280
// #include "unishox2.h"
#include <ClosedCube_BME680.h> // http://librarymanager/All#ClosedCube_BME680
#include <ss_oled.h> // http://librarymanager/All#ss_oled By Larry Bank
/*
   PATCHES TO BE DONE IN ORDER TO BE ABLE TO COMPILE!
  --> ss_oled.h
  #include <BitBang_I2C.h>
  #ifdef __RUI_TOP_H__
  #include "/Users/XXXXXX/Library/Arduino15/packages/rakwireless/hardware/nrf52/1.0.1/cores/nRF5/avr/pgmspace.h"
  #endif

  --> BitBang_I2C.cpp
  void I2CInit(BBI2C *pI2C, uint32_t iClock) {
  [...]
  #if defined(TEENSYDUINO) || defined(ARDUINO_ARCH_RP2040) || defined(__AVR__) || defined(NRF52)
  || defined (ARDUINO_ARCH_NRF52840) || defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_SAM) || defined(__RUI_TOP_H__)

  NOTE: These patches are not needed any longer – Leaving this warning just in case...
*/
#include <BBQ10Keyboard.h> // http://librarymanager/All#BBQ10Keyboard
#include <HTU21D.h> // http://librarymanager/All#HTU21D Library by Daniel Wiese
#include <CayenneLPP.h> // http://librarymanager/All#CayenneLPP
#include <DS3231M.h> // http://librarymanager/All#DS3231M
#include <Melopero_RV3028.h> // http://librarymanager/All#Melopero_RV3028
#include <BH1750.h> // https://github.com/claws/BH1750 or http://librarymanager/All#BH1750 by CLaws

BBQ10Keyboard keyboard;
// Stuff for the beyboard
// See https://github.com/solderparty/bbq10kbd_i2c_sw
bool SYM = false; // Is the SYM key being held down
#define _SYM_KEY 19 // Key code
#define CFG_REPORT_MODS 0b01000000
// https://github.com/solderparty/bbq10kbd_i2c_sw#the-configuration-register-reg_cfg--0x02
#define _REG_CFG 0x02
#define _REG_KEY 0x04
#define bbqLimit 16
char bbqBuff[bbqLimit + 1] = {0};
uint8_t bbqIndex = 0;
uint32_t bbqDELAY = 6000;

BH1750 bh1750;
Melopero_RV3028 rak12002;
DS3231M_Class DS3231M; // /< Create an instance of the DS3231M class
CayenneLPP lpp(51);
//Create an instance of the HTU21D object
HTU21D myHTU21D;
/** Temperature & Humidity sensor **/
rak1901 th_sensor;
/** Air Pressure sensor **/
rak1902 p_sensor;
rak1903 rak1903_lux;
ClosedCube_BME680 bme680;
BME280 bme280;

int rc;
SSOLED oled;
// Stuff for the OLED
#define SDA_PIN WB_I2C1_SDA
#define SCL_PIN WB_I2C1_SCL
#define RESET_PIN -1
#define FLIPPED 0
#define INVERTED 0
// Use bit banging to get higher speed output
#define HARDWARE_I2C 1
#define WIDTH 128
#define HEIGHT 64
static uint8_t ucBuffer[1024];
double oledLastOn, OLEDdelay = 30000;

bool hasOLED = true; // Doesn't mean you have an OLED – Check and verify!
// It means that IF an OLED is detected, it will be used.
// OTOH if you set it to false, even if it is detected, it won't be used.
bool hasTH = false, hasPA = false, has1903 = false, hasBME680 = false, hasHTU21D = false, hasDS3231M = false;
bool hasRAK12002 = false, hasBH1750 = false, hasBBQ10 = false, oledON, hasBME280 = false;;
bool needSerial1 = true; // Set to false if you want to disable Serial1
float temp, humid, HPa, Lux, MSL = 1013.5, alt;
long startTime;
// LoRa SETUP
// The LoRa chip come pre-wired: all you need to do is define the parameters:
// frequency, SF, BW, CR, Preamble Length and TX power
double myFreq = 868125000;
uint16_t counter = 0, sf = 12, bw = 125, cr = 0, preamble = 8, txPower = 22;
uint32_t myBWs[10] = {125, 125, 125, 125, 125, 125, 125, 125, 250, 500};
/*
  There's a bug presently in the API, whereas BW values below 7/125 KHz are not recognized.
  Temporary fix until the engineers enables all 10 values,
  SignalBandwidth   0     1     2      3      4      5      6    7   8    9
  BW_L [kHz]      7.810 10.420 15.630 20.830 31.250 41.67 62.50 125 250  500
*/
char msg[128]; // general-use buffer

#include "Utilities.h"
#include "Commands.h"

/*
  typedef struct rui_lora_p2p_revc {
    // Pointer to the received data stream
    uint8_t *Buffer;
    // Size of the received data stream
    uint8_t BufferSize;
    // Rssi of the received packet
    int16_t Rssi;
    // Snr of the received packet
    int8_t Snr;
  } rui_lora_p2p_recv_t;
*/
void recv_cb(rui_lora_p2p_recv_t data) {
  // RX callback
  api.lorawan.precv(0); // We're busy lah, do not disturb! :-)
  uint16_t ln = data.BufferSize;
  hexDump(data.Buffer, ln);
  if (ln == 0) {
    // This should not happen. But, you know...
    // will not != should not
    Serial.println("Empty buffer.");
    Serial.printf("Reset Rx mode %s\n", api.lorawan.precv(65534) ? "Success" : "Fail");
    return;
  }
  sprintf(msg, "Incoming message, length: %d, RSSI: %d, SNR: %d\n", ln, data.Rssi, data.Snr);
  Serial.print(msg);
#ifdef __RAKBLE_H__
  // Serial.println("Sending to BLE");
  // Adafruit's Bluefruit connect is being difficult, and seems to require \n to fully receive a message..
  sprintf(msg, "Length: %d, RSSI %d, SNR %d\n", ln, data.Rssi, data.Snr);
  sendToBle(msg);
#endif
  bool isLPP = (data.Buffer[0] == 'l' && data.Buffer[1] == 'p' && data.Buffer[2] == 'p');
  if (isLPP) {
    //    // DynamicJsonDocument jsonBuffer(512);
    //    StaticJsonDocument<1024> jsonBuffer;
    //    JsonArray root = jsonBuffer.to<JsonArray>();
    //    ln -= 3;
    //    uint8_t buffer[ln], count;
    //    memcpy(buffer, data.Buffer + 3, ln);
    //    count = lpp.decode(buffer, ln, root);
    decodeLPP((char*)(data.Buffer + 3), ln - 3);
    //    serializeJsonPretty(root, msg, measureJsonPretty(root));
    //    Serial.println(msg);
#ifdef __RAKBLE_H__
    // Serial.println("Sending to BLE");
    // Adafruit's Bluefruit connect is being difficult, and seems to require \n to fully receive a message..
    sprintf(msg, "[%d] RSSI %d SNR %d\n", ln, data.Rssi, data.Snr);
    sendToBle(msg);
    sprintf(msg, "%s\n", (char*)data.Buffer);
    sendToBle(msg);
#endif
  } else {
    sprintf(msg, "{\"RSSI\":%d,\"SNR\":%d,\"msg\":\"%s\"}\n", data.Rssi, data.Snr, data.Buffer);
    if (needSerial1) Serial1.print(msg);
#ifdef __RAKBLE_H__
    // Serial.println("Sending to BLE");
    // Adafruit's Bluefruit connect is being difficult, and seems to require \n to fully receive a message..
    sprintf(msg, "%s\n", (char*)data.Buffer);
    sendToBle(msg);
#endif
  }
  Serial.printf("Reset Rx mode %s\n", api.lorawan.precv(65534) ? "Success" : "Fail");
  if (pongBack) {
    Serial.print("We need to pong back: delay before pongback...\n  ");
    uint8_t x = 3;
    while (x > 0) {
      Serial.printf("%d, ", x--);
      delay(500);
    } // Just for show
    Serial.println("0!");
    sprintf(msg, "rcvd at %d %d", data.Rssi, data.Snr);
    sendMsg(msg);
  }
}

void send_cb(void) {
  // TX callback
  Serial.println("Tx done!");
  Serial.printf("reset Rx mode[65534] %s\n", api.lorawan.precv(65534) ? "Success" : "Fail");
  // set the LoRa module to indefinite listening mode:
  // no timeout + no limit to the number of packets
  // NB: 65535 = wait for ONE packet, no timeout
}

void sendMsg(char* msgToSend) {
  uint8_t ln = strlen(msgToSend);
  api.lorawan.precv(0);
  // turn off reception – a little hackish, but without that send might fail.
  // memset(msg, 0, ln + 20);
  Serial.printf("Sending `%s`: ", msgToSend);
  bool rslt = api.lorawan.psend(ln, (uint8_t*)msgToSend);
  // when done it will call void send_cb(void);
  Serial.printf("%s\n", rslt ? "Success" : "Fail");
#ifdef __RAKBLE_H__
  sprintf(msg, "Sending `%s` via P2P: %s\n", msgToSend, rslt ? "Success" : "Fail");
  sendToBle(msg);
#endif
  if (hasOLED) {
    sprintf(msg, "Sent %s via P2P:", rslt ? "[o]" : "[x]");
    displayScroll(msg);
    displayScroll(msgToSend);
  }
}

void sendLPP(char * param) {
  lpp.reset();
  uint8_t channel = 1;
  if (hasDS3231M) {
    if (hasOLED) displayScroll(" * ds3231m");
    DateTime now = DS3231M.now();
    uint32_t ut = now.unixtime();
    lpp.addUnixTime(channel++, ut);
    Serial.printf("Adding Unix Time %d\n", ut);
  } else if (hasRAK12002) {
    if (hasOLED) displayScroll(" * rak12002");
    uint32_t ut = rak12002.getUnixTime();
    lpp.addUnixTime(channel++, ut);
    Serial.printf("Adding Unix Time %d\n", ut);
  }
  if (hasTH) {
    if (hasOLED) displayScroll(" * rak1901");
    th_sensor.update();
    temp = th_sensor.temperature();
    humid = th_sensor.humidity();
    lpp.addTemperature(channel++, temp);
    Serial.printf("Adding Temperature %.2f\n", temp);
    lpp.addRelativeHumidity(channel++, humid);
    Serial.printf("Adding Humidity %.2f\n", humid);
  }
  if (hasPA) {
    if (hasOLED) displayScroll(" * rak1902");
    HPa = p_sensor.pressure(MILLIBAR);
    lpp.addBarometricPressure(channel++, HPa);
    Serial.printf("Adding HPa %.2f\n", HPa);
  }
  if (hasBME680) {
    if (hasOLED) displayScroll(" * rak1906");
    ClosedCube_BME680_Status status = bme680.readStatus();
    //  if (status.newDataFlag) {
    temp = bme680.readTemperature();
    HPa = bme680.readPressure();
    humid = bme680.readHumidity();
  } else if (hasBME280) {
    if (hasOLED) displayScroll(" * bme280");
    temp = bme280.getTemperature();
    HPa = bme280.getPressure() / 100.0;
    humid = bme280.getHumidity();
  }
  if (hasBME680 || hasBME280) {
    lpp.addTemperature(channel++, temp);
    Serial.printf("Adding Temperature %.2f\n", temp);
    lpp.addRelativeHumidity(channel++, humid);
    Serial.printf("Adding Humidity %.2f\n", humid);
    lpp.addBarometricPressure(channel++, HPa);
    Serial.printf("Adding HPa %.2f\n", HPa);
    alt = calcAlt(HPa);
    lpp.addAltitude(channel++, alt);
    Serial.printf("Adding Altitude %.2f\n", alt);
  }
  if (hasHTU21D) {
    if (hasOLED) displayScroll(" * HTU21D");
    myHTU21D.measure();
    temp = myHTU21D.getTemperature();
    humid = myHTU21D.getHumidity();
    lpp.addTemperature(channel++, temp);
    Serial.printf("Adding Temperature %.2f\n", temp);
    lpp.addRelativeHumidity(channel++, humid);
    Serial.printf("Adding Humidity %.2f\n", humid);
  }
  if (has1903) {
    if (hasOLED) displayScroll(" * rak1903");
    rak1903_lux.update();
    Lux = rak1903_lux.lux();
    lpp.addLuminosity(channel++, Lux);
    Serial.printf("Adding Luminosity %.2f\n", Lux);
  }
  uint8_t ln = lpp.getSize();
  if (ln == 0) {
    sprintf(msg, "Nothing to send!");
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) {
      displayScroll(msg);
    }
    return;
  }
  api.lorawan.precv(0);
  // turn off reception – a little hackish, but without that send might fail.
  uint8_t lBuff[ln + 3];
  memcpy(lBuff + 3, lpp.getBuffer(), ln);
  lBuff[0] = 'l';
  lBuff[1] = 'p';
  lBuff[2] = 'p';
  bool rslt = api.lorawan.psend(ln + 3, lBuff);
  sprintf(msg, "Sending LPP payload [%d]: %s\n", (ln + 3), rslt ? "Success" : "Fail");
  Serial.print(msg);
  hexDump(lBuff, ln + 3);
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
  if (hasOLED) {
    sprintf(msg, "LPP %d b: %s", ln, rslt ? "[o]" : "[x]");
    displayScroll(msg);
  }
}

void sendPing(char* param) {
  sprintf(msg, "PING #0x%04x", counter++);
  if (needSerial1) Serial1.println(msg);
  sendMsg(msg);
}

void sendTH() {
  th_sensor.update();
  temp = th_sensor.temperature();
  humid = th_sensor.humidity();
  sprintf(msg, "%.2f C %.2f%%", temp, humid);
  sendMsg(msg);
}

void sendHTU21D() {
  myHTU21D.measure();
  temp = myHTU21D.getTemperature();
  humid = myHTU21D.getHumidity();
  sprintf(msg, "%.2f C %.2f%%", temp, humid);
  sendMsg(msg);
}

void sendPA() {
  HPa = p_sensor.pressure(MILLIBAR);
  sprintf(msg, "%.2f HPa", HPa);
  sendMsg(msg);
}

void sendLux() {
  // Update lux value then send.
  if (has1903 && hasBH1750) {
    rak1903_lux.update();
    Lux = rak1903_lux.lux();
    if (bh1750.measurementReady(true)) {
      float Lux0 = bh1750.readLightLevel();
      sprintf(msg, "lux: %.2f %.2f", Lux0, Lux);
    } else {
      sprintf(msg, "lux: NaN %.2f", Lux);
    }
    sendMsg(msg);
    return;
  } else {
    if (has1903) {
      rak1903_lux.update();
      Lux = rak1903_lux.lux();
      sprintf(msg, "rak1903: %.2f", Lux);
      sendMsg(msg);
    }
    if (hasBH1750) {
      if (bh1750.measurementReady(true)) {
        Lux = bh1750.readLightLevel();
        sprintf(msg, "bh1750: %.2f", Lux);
      } else {
        sprintf(msg, "bh1750 not ready");
      }
      sendMsg(msg);
    }
  }
}

void sendBME280(bool showPA = true) {
  temp = bme280.getTemperature();
  HPa = bme280.getPressure() / 100.0;
  humid = bme280.getHumidity();
  if (showPA) sprintf(msg, "%.2f C %.2f%% %.2f HPa", temp, humid, HPa);
  else sprintf(msg, "%.2f C %.2f%%", temp, humid);
  if (hasOLED && showPA) {
    // if we show all 3 data points: do it on two lines
    hasOLED = false;
    // we will display on 2 lines, separately
    sendMsg(msg);
    sprintf(msg, "%.2fC %.2f%%", temp, humid);
    displayScroll(msg);
    sprintf(msg, "%.2f HPa", HPa);
    displayScroll(msg);
    hasOLED = true;
  } else {
    sendMsg(msg);
  }
}

void sendBME680(bool showPA = true) {
  ClosedCube_BME680_Status status = bme680.readStatus();
  //  if (status.newDataFlag) {
  temp = bme680.readTemperature();
  HPa = bme680.readPressure();
  humid = bme680.readHumidity();
  if (showPA) sprintf(msg, "%.2f C %.2f%% %.2f HPa", temp, humid, HPa);
  else sprintf(msg, "%.2f C %.2f%%", temp, humid);
  if (hasOLED && showPA) {
    // if we show all 3 data points: do it on two lines
    hasOLED = false;
    // we will display on 2 lines, separately
    sendMsg(msg);
    sprintf(msg, "%.2fC %.2f%%", temp, humid);
    displayScroll(msg);
    sprintf(msg, "%.2f HPa", HPa);
    displayScroll(msg);
    hasOLED = true;
  } else {
    sendMsg(msg);
  }
  bme680.setForcedMode();
  //  } else Serial.println("BME data not ready.");
}

float calcAlt(float pressure) {
  float A = pressure / MSL;
  float B = 1 / 5.25588;
  float C = pow(A, B);
  C = 1.0 - C;
  C = C / 0.0000225577;
  return C;
}

void displayTime() {
  if (hasRAK12002) {
    sprintf(msg, "rak12002 date: %04d/%02d/%02d\n", rak12002.getYear(), rak12002.getMonth(), rak12002.getDate());
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    sprintf(msg, "rak12002 time: %02d:%02d:%02d\n", rak12002.getHour(), rak12002.getMinute(), rak12002.getSecond());
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    uint32_t ut = rak12002.getUnixTime();
    sprintf(msg, "Unix Time: %d = 0x%x\n", ut, ut);
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) {
      sprintf(msg, "%04d/%02d/%02d", rak12002.getYear(), rak12002.getMonth(), rak12002.getDate());
      displayScroll(msg);
      sprintf(msg, "%02d:%02d:%02d", rak12002.getHour(), rak12002.getMinute(), rak12002.getSecond());
      displayScroll(msg);
    }
    return;
  }

  if (hasDS3231M) {
    DateTime now = DS3231M.now(); // get the current time from device
    uint32_t ut = now.unixtime();
    // Output if seconds have changed
    // Use sprintf() to pretty print the date/time with leading zeros
    sprintf(msg, "%04d/%02d/%02d %02d:%02d:%02d. Unix Time: %d = 0x%x\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) {
      sprintf(msg, "%04d/%02d/%02d", now.year(), now.month(), now.day());
      displayScroll(msg);
      sprintf(msg, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      displayScroll(msg);
    }
    return;
  }
}

void setup() {
  Serial.begin(115200, RAK_CUSTOM_MODE);
  if (needSerial1) Serial1.begin(115200, RAK_CUSTOM_MODE);
  // RAK_CUSTOM_MODE disables AT firmware parsing
  time_t timeout = millis();
  while (!Serial) {
    // on nRF52840, Serial is not available right away.
    // make the MCU wait a little
    if ((millis() - timeout) < 5000) {
      delay(100);
    } else {
      break;
    }
  }
  uint8_t x = 5;
  while (x > 0) {
    Serial.printf(" % d, ", x--);
    delay(500);
  } // Just for show
  Serial.println("0!");
  Serial.println("RAKwireless LoRa P2P Kitchen Sink");
  Serial.println("------------------------------------------------------");
  if (needSerial1) Serial1.println("Kitchen Sink");
  cmdCount = sizeof(cmds) / sizeof(myCommand);
  codeCount = sizeof(lppCodes) / sizeof(myCodes);
  Serial.printf("codeCount = % d\n", codeCount);
  for (uint8_t i = 0; i < codeCount; i++) Serial.printf(" . % s: % d\n", lppCodes[i].name, lppCodes[i].code);
  handleHelp(" / help");
  Wire.begin();
  Wire.setClock(100000);
  i2cScan("");
  // Test for OLED
  byte error = myBus[0x3c];
  if (error != 0) {
    Serial.println("OLED present");
    if (hasOLED) {
      // the user wants OLED display
      uint8_t uc[8];
      rc = oledInit(&oled, OLED_128x64, 0x3c, FLIPPED, INVERTED, HARDWARE_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L);
      if (rc != OLED_NOT_FOUND) {
        switchOLED(true);
        oledSetBackBuffer(&oled, ucBuffer);
        oledSetTextWrap(&oled, 1);
        oledFill(&oled, 0, 1);
        oledWriteString(&oled, 0, -1, -1, "LoRa p2p", FONT_16x16, 0, 1);
        oledLastOn = millis();
        oledON = true;
        /*
          delay(500);
          uint8_t backup0[1024];
          uint8_t backup1[1024];
          memcpy(backup0, ucBuffer, 1024);
          oledFill(&oled, 0, 1);
          oledWriteString(&oled, 0, -1, -1, "TEST!", FONT_16x16, 0, 1);
          memcpy(backup1, ucBuffer, 1024);
          delay(500);
          oledDumpBuffer(&oled, backup0);
          delay(500);
          oledDumpBuffer(&oled, backup1);
          delay(500);
          oledDumpBuffer(&oled, backup0);
          delay(500);
          oledDumpBuffer(&oled, backup1);
          delay(500);
          oledDumpBuffer(&oled, backup0);
          delay(500);
        */
      } else {
        Serial.println("But you specified you didn't want OLED display!");
      }
    } else hasOLED = false;
    // Even if the user wanted it – since it ain't there, we set it to false.
  }
  // Test for BBQ10 kbd
  error = myBus[0x1f];
  if (error != 0) {
    Serial.println("BBQ10 Keyboard present!");
    hasBBQ10 = true;
    if (hasOLED) displayScroll("* BBQ10 kbd");
    keyboard.begin();
    keyboard.setBacklight(0.5f);
    keyboard.setBacklight2(1.0f);
    uint8_t cfg = keyboard.readRegister8(_REG_CFG);
    Serial.printf(" . cfg: %08x\n", cfg);
    // Report SYM, ALT, KEYCAPS etc
    cfg |= CFG_REPORT_MODS;
    Serial.printf(" . cfg: %08x\n", cfg);
    keyboard.writeRegister(_REG_CFG, cfg);
  }

  // Test for rtc
  error = myBus[0x52];
  if (error != 0) {
    Serial.println("RAK12002 RTC present!");
    hasRAK12002 = true;
    rak12002.initI2C(); // First initialize and create the rtc device
    rak12002.writeToRegister(0x35, 0x00);
    rak12002.writeToRegister(0x37, 0xB4); //Direct Switching Mode (DSM): when VDD < VBACKUP, switchover occurs from VDD to VBACKUP
    rak12002.set24HourMode(); // Set the device to use the 24-hour format (default) instead of the 12-hour format
    if (hasOLED) displayScroll("* RAK12002 RTC");
  }

  if (!hasRAK12002) {
    error = myBus[0x68];
    if (error != 0) {
      Serial.println("DS3231M RTC present!");
      hasDS3231M = DS3231M.begin();
      Serial.printf("DS3231M init %s\n", hasDS3231M ? "success" : "fail");
      if (hasDS3231M && hasOLED) displayScroll("* DS3231M RTC");
    }
  }

  // Test for rak1901
  error = myBus[0x70];
  if (error != 0) {
    Serial.println("Temperature & Humidity Sensor present!");
    hasTH = th_sensor.init();
    Serial.printf("RAK1901 init %s\n", hasTH ? "success" : "fail");
    th_sensor.update();
    temp = th_sensor.temperature();
    humid = th_sensor.humidity();
    if (hasOLED) displayScroll("* rak1901");
  }

  // Test for rak1902
  error = myBus[0x5C];
  if (error != 0) {
    Serial.println("Pressure Sensor present!");
    hasPA = p_sensor.init();
    Serial.printf("RAK1902 init %s\n", hasPA ? "success" : "fail");
    HPa = p_sensor.pressure(MILLIBAR);
    if (hasOLED) displayScroll("* rak1902");
  }

  // Test for rak1903
  error = myBus[0x44];
  if (error != 0) {
    Serial.println("RAK1903 Light Sensor present!");
    has1903 = rak1903_lux.init();
    Serial.printf("RAK1903 init %s\n", has1903 ? "success" : "fail");
    Lux = rak1903_lux.lux();
    if (hasOLED) displayScroll("* rak1903");
  }

  // Test for BH1750
  error = myBus[0x23];
  if (error != 0) {
    Serial.println("hasBH1750 Light Sensor present!");
    hasBH1750 = true;
    bh1750.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
    if (bh1750.measurementReady(true)) {
      Lux = bh1750.readLightLevel();
      Serial.printf("BH1750 init: %d\n", Lux);
      if (hasOLED) displayScroll("* bh1750");
    }
  }

  // Test for rak1906
  error = myBus[0x76];
  hasBME280 = false;
  if (error != 0) {
    Serial.println("BME device present!");
    bme680.init(0x76); // I2C address: 0x76 or 0x77
    bme680.reset();
    Serial.print("Chip ID=0x");
    uint8_t id = bme680.getChipID();
    Serial.println(id, HEX);
    hasBME680 = (id == 0x61);
    if (hasBME680) {
      Serial.println("RAK1906 init success!");
      if (hasOLED) displayScroll("* rak1906");
      bme680.setOversampling(BME680_OVERSAMPLING_X1, BME680_OVERSAMPLING_X2, BME680_OVERSAMPLING_X16);
      bme680.setIIRFilter(BME680_FILTER_3);
      bme680.setForcedMode();
    } else if (id == 0x60) {
      hasBME280 = bme280.init();
      if (!hasBME280) {
        Serial.println("* bme280 startup error!");
        if (hasOLED) displayScroll("* bme280 error!");
      } else {
        Serial.println("* bme280");
        if (hasOLED) displayScroll("* bme280");
      }
    } else {
      Serial.println("BMEx80 init fail!");
      if (hasOLED) displayScroll("BMEx80 init fail!");
    }
  }

  // Test for HTU21D
  error = myBus[0x40];
  if (error != 0) {
    Serial.println("HTU21D Temperature/Humidity present!");
    if (hasOLED) displayScroll("* HTU21D");
    myHTU21D.begin();
    hasHTU21D = true;
  }

  Serial.println("P2P Start");
  if (hasOLED) displayScroll("* P2P Start");
  char HardwareID[16]; // nrf52840
  strcpy(HardwareID, api.system.chipId.get().c_str());
  Serial.printf("Hardware ID: %s\n", HardwareID);
  if (strcmp(HardwareID, "nrf52840") == 0) {
    Serial.println("BLE compatible!");
    if (hasOLED) displayScroll("* BLE available");
  }
  sprintf(msg, "* Model ID: %s", api.system.modelId.get().c_str());
  Serial.println(msg);
  if (hasOLED) {
    sprintf(msg, "* MCU: %s", api.system.modelId.get().c_str());
    displayScroll(msg);
  }
  Serial.printf("* RUI API Version: %s\n", api.system.apiVersion.get().c_str());
  Serial.printf("* Firmware Version: %s\n", api.system.firmwareVersion.get().c_str());
  Serial.printf("* AT version: %s\n", api.system.cliVersion.get().c_str());

  // LoRa setup – everything else has been done for you. No need to fiddle with pins, etc
  Serial.printf("Set work mode to P2P: %s\n", api.lorawan.nwm.set(0) ? "Success" : "Fail");
  Serial.printf("Set P2P frequency to %3.3f: %s\n", (myFreq / 1e6), api.lorawan.pfreq.set(myFreq) ? "Success" : "Fail");
  Serial.printf("Set P2P spreading factor to %d: %s\n", sf, api.lorawan.psf.set(sf) ? "Success" : "Fail");
  Serial.printf("Set P2P bandwidth to %d: %s\n", bw, api.lorawan.pbw.set(bw) ? "Success" : "Fail");
  Serial.printf("Set P2P coding rate to 4/%d: %s\n", (cr + 5), api.lorawan.pcr.set(cr) ? "Success" : "Fail");
  Serial.printf("Set P2P preamble length to %d: %s\n", preamble, api.lorawan.ppl.set(preamble) ? "Success" : "Fail");
  Serial.printf("Set P2P TX power to %d: %s\n", txPower, api.lorawan.ptp.set(txPower) ? "Success" : "Fail");
  // SX126xWriteRegister(0x08e7, OCP_value);
  // Serial.printf("Set OCP to 0x%2x [%d]\n", OCP_value, (OCP_value * 2.5));
  // LoRa callbacks
  api.lorawan.registerPRecvCallback(recv_cb);
  api.lorawan.registerPSendCallback(send_cb);

  // api.system.restoreDefault();
  // This causes various issues. Including a reboot. Let's stay away from that.

#ifdef __RAKBLE_H__
  Serial6.begin(115200, RAK_CUSTOM_MODE);
  // If you want to read and write data through BLE API operations,
  // you need to set BLE Serial (Serial6) to Custom Mode

  uint8_t pairing_pin[] = "004631";
  Serial.print("Setting pairing PIN to: ");
  Serial.println((char *)pairing_pin);
  api.ble.uart.setPIN(pairing_pin, 6); //pairing_pin = 6-digit (digit 0..9 only)
  // Set Permission to access BLE Uart is to require man-in-the-middle protection
  // This will cause apps to perform pairing with static PIN we set above
  // now support SET_ENC_WITH_MITM and SET_ENC_NO_MITM
  api.ble.uart.setPermission(RAK_SET_ENC_WITH_MITM);

  char ble_name[32];
  sprintf(ble_name, "RAK_%s", api.ble.mac.get()); // You have to be French to understand this joke
  Serial.print("Setting Broadcast Name to: ");
  Serial.println(ble_name);
  api.ble.settings.broadcastName.set(ble_name, strlen(ble_name));
  api.ble.uart.start();
  api.ble.advertise.start(0);
#endif
  // This version doesn't have an automatic Tx functionality:
  // YOU are in charge of sending, either via Serial or BLE.
  Serial.printf("SetRx mode %s\n", api.lorawan.precv(65534) ? "Success" : "Fail");
  startTime = millis();
  if (hasOLED) oledLastOn = millis();
  if (autoPing) lastPing = millis();
  char myLPP[] = {
    0x01, 0x85, 0x62, 0x72, 0x9e, 0x2d, 0x02, 0x67, 0x00, 0xef,
    0x03, 0x68, 0x6e, 0x04, 0x73, 0x26, 0xed, 0x05, 0x67, 0x00,
    0xf5, 0x06, 0x68, 0x74, 0x07, 0x73, 0x26, 0xdf, 0x08, 0x65,
    0x00, 0x05
  };
  decodeLPP(myLPP, 32);
}

void loop() {
  if (hasBBQ10) {
    // If you don't have an OLED it's gonna be fun...
    // Watch the Serial Monitor!
    const int keyCount = keyboard.keyCount();
    if (keyCount == 0 && isBBQing) {
      uint32_t t0 = millis();
      if (t0 - lastKbdInput > bbqDELAY) {
        Serial.println("bbq10 timeout, reset screen.");
        restoreScreen();
      }
    } else if (keyCount > 0) {
      lastKbdInput = millis();
      if (!isBBQing && hasOLED) restoreBBQinput();
      if (hasOLED) oledLastOn = millis();
      const BBQ10Keyboard::KeyEvent key = keyboard.keyEvent();
      if (key.state == BBQ10Keyboard::StateLongPress) {
        if (key.key == _SYM_KEY) SYM = true;
        return;
      } else if (key.state == BBQ10Keyboard::StateRelease) {
        if (key.key == _SYM_KEY) {
          SYM = false;
          return;
        } else if (key.key > 31) {
          if (bbqIndex < bbqLimit) bbqBuff[bbqIndex++] = key.key;
        } else if (key.key == 8) {
          if (SYM) {
            // SYM+Backspace = erase everything
            // Serial.println(" > Full erase!");
            memset(bbqBuff, 0, bbqLimit);
            bbqIndex = 0;
          } else {
            // Erase one char, if any left
            bbqBuff[bbqIndex] = 0; // Not really necessary but belt and suspenders...
            if (bbqIndex > 0) bbqIndex -= 1;
            bbqBuff[bbqIndex] = 0;
          }
          if (hasOLED) {
            // easiest way to redraw after erase is to draw off-screen. then dump the buffer
            oledFill(&oled, 0, 0);
            oledWriteString(&oled, 0, 10, 0, "BBQ10", FONT_16x16, 0, 0);
            oledWriteString(&oled, 0, 0, 2, ">", FONT_8x8, 0, 0);
            if (bbqIndex > 0) oledWriteString(&oled, 0, 8, 2, bbqBuff, FONT_8x8, 0, 0);
            oledDumpBuffer(&oled, ucBuffer);
          }
        } else if (key.key == 10) {
          // enter = let's go!
          restoreScreen(); // back to what it was
          handleCommands(bbqBuff);
          memset(bbqBuff, 0, bbqLimit);
          bbqIndex = 0;
          return;
        }
        if (key.key > 31) {
          if (hasOLED) {
            for (uint8_t x = 0; x < 128; x++) ucBuffer[x + 256] = 0;
            oledWriteString(&oled, 0, 0, 2, ">", FONT_8x8, 0, 0);
            oledWriteString(&oled, 0, 8, 2, bbqBuff, FONT_8x8, 0, 0);
            oledDumpBuffer(&oled, ucBuffer);
          }
          Serial.printf("BBQ10 Buffer [%d]: %s\n", bbqIndex, bbqBuff);
        }
      }
    }
  }
  if (autoPing) {
    uint32_t t0 = millis();
    if (t0 - lastPing > apPeriod) {
      Serial.printf("autoping at %d / %d\n", (t0 - lastPing), apPeriod);
#ifdef __RAKBLE_H__
      sendToBle("autoping");
#endif
      // if (hasOLED) displayScroll("autoping");
      sendPing("");
      lastPing = millis();
    }
  }
#ifdef __RAKBLE_H__
  if (api.ble.uart.available()) {
    if (hasOLED && !oledON) {
      oledON = true;
      switchOLED(true);
      oledLastOn = millis();
    }
    // store the incoming string into a buffer
    Serial.println("\nBLE in:");
    // if (hasOLED) displayScroll("BLE in:");
    char str1[256];
    uint8_t ix = 0;
    // with a 256-byte buffer and a uint8_t index
    // you won't get a buffer overrun :-)
    while (api.ble.uart.available()) {
      char c = api.ble.uart.read();
      if (c > 31) str1[ix++] = c;
      // strip \n and the like
      // this is ok because we expect text. You might want to adjust if you are accepting binary data
    }
    str1[ix] = 0;
    Serial.println(str1);
    if (hasOLED) displayScroll(str1);
    handleCommands(str1);
    // pass the string to the command-handling fn
  }
#endif
  if (Serial.available()) {
    if (hasOLED && !oledON) {
      oledON = true;
      switchOLED(true);
      oledLastOn = millis();
    }
    //Serial.println("\nIncoming:");
    // if (hasOLED) displayScroll("Serial in:");
    char str1[256];
    str1[0] = '>';
    str1[1] = ' ';
    uint8_t ix = 2;
    while (Serial.available()) {
      char c = Serial.read();
      if (c > 31) str1[ix++] = c;
    }
    str1[ix] = 0;
    Serial.println(str1);
    if (hasOLED) displayScroll(str1);
    handleCommands(str1 + 2);
  }

  if (needSerial1) {
    if (Serial1.available()) {
      if (hasOLED && !oledON) {
        oledON = true;
        switchOLED(true);
        oledLastOn = millis();
      }
      char str1[256];
      str1[0] = '>';
      str1[1] = ' ';
      uint8_t ix = 2;
      while (Serial1.available()) {
        char c = Serial1.read();
        delay(10);
        if (c > 31) str1[ix++] = c;
      }
      str1[ix] = 0;
      Serial.println("From Serial1");
      Serial.println(str1);
      if (hasOLED) displayScroll(str1);
      handleCommands(str1 + 2);
    }
  }
  if (hasOLED && oledON) {
    double t0 = millis();
    if (t0 - oledLastOn > OLEDdelay) switchOLED(false);
  }
}
