#include "rak1901.h"
#include "rak1902.h"
#include "rak1903.h"
// #include "unishox2.h"
#include "ClosedCube_BME680.h"
#include <ss_oled.h>
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
*/
#include "HTU21D.h"
#include <CayenneLPP.h>
#include <DS3231M.h> // Include the DS3231M RTC library
#include "Melopero_RV3028.h" //http://librarymanager/All#Melopero_RV3028
#include <BH1750.h> // https://github.com/claws/BH1750

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
#define SDA_PIN WB_I2C1_SDA
#define SCL_PIN WB_I2C1_SCL

#define RESET_PIN -1
#define FLIPPED 0
#define INVERTED 0
// Use bit banging to get higher speed output
#define HARDWARE_I2C 1
#define WIDTH 128
#define HEIGHT 64
int rc;
SSOLED oled;

static uint8_t ucBuffer[1024];
bool hasOLED = true, hasTH = false, hasPA = false, has1903 = false, hasBME680 = false, hasHTU21D = false, hasDS3231M = false, hasRAK12002 = false, hasBH1750 = false;
float temp, humid, HPa, Lux;
long startTime;
// LoRa SETUP
// The LoRa chip come pre-wired: all you need to do is define the parameters:
// frequency, SF, BW, CR, Preamble Length and TX power
double myFreq = 868000000;
float MSL = 1013.5;
uint16_t counter = 0, sf = 12, bw = 125, cr = 0, preamble = 8, txPower = 22;
char msg[128];
uint32_t myBWs[10] = {125, 125, 125, 125, 125, 125, 125, 125, 250, 500};
/*
  There's a bug presently in the API, whereas BW values below 7/125 KHz are not recognized.
  Temporary fix until the engineers enables all 10 values,
  SignalBandwidth   0     1     2      3      4      5      6    7   8    9
  BW_L [kHz]      7.810 10.420 15.630 20.830 31.250 41.67 62.50 125 250  500
*/

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
  if (data.BufferSize == 0) {
    // This should not happen. But, you know...
    // will not != should not
    // Serial.println("Empty buffer.");
    return;
  }
  sprintf(msg, "Incoming message, length: %d, RSSI: %d, SNR: %d\n", data.BufferSize, data.Rssi, data.Snr);
  Serial.print(msg);
  // Adafruit's Bluefruit connect is being difficult, and seems to require \n to fully receive a message..
  sprintf(msg, "[%d] RSSI %d SNR %d\n", data.BufferSize, data.Rssi, data.Snr);
  if (hasOLED) {
    sprintf(msg, "LoRa msg: %d", data.BufferSize);
    displayScroll(msg);
    sprintf(msg, "RSSI: %d", data.Rssi);
    displayScroll(msg);
    sprintf(msg, "SNR: %d", data.Snr);
    displayScroll(msg);
    if (data.BufferSize < 17) displayScroll((char*)data.Buffer);
    else {
      memset(msg, 0, 128);
      memcpy(msg, (char*)data.Buffer, data.BufferSize);
      for (uint8_t i = 0; i < data.BufferSize; i += 16) {
        char c = msg[i + 16];
        msg[i + 16] = 0;
        displayScroll(msg + i);
        msg[i + 16] = c;
      }
    }
  }
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
  hexDump(data.Buffer, data.BufferSize);
  Serial.println("Sending to BLE");
  sprintf(msg, "%s\n", (char*)data.Buffer);
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
}

void send_cb(void) {
  // TX callback
  Serial.printf("Set infinite Rx mode %s\n", api.lorawan.precv(65534) ? "Success" : "Fail");
  // set the LoRa module to indefinite listening mode:
  // no timeout + no limit to the number of packets
  // NB: 65535 = wait for ONE packet, no timeout
}

void sendLPP() {
  lpp.reset();
  uint8_t channel = 1;
  if (hasTH) {
    if (hasOLED) displayScroll(" * rak1901");
    th_sensor.update();
    temp = th_sensor.temperature();
    humid = th_sensor.humidity();
    lpp.addTemperature(channel++, temp);
    lpp.addRelativeHumidity(channel++, humid);
  }
  if (hasPA) {
    if (hasOLED) displayScroll(" * rak1902");
    HPa = p_sensor.pressure(MILLIBAR);
    lpp.addBarometricPressure(channel++, HPa);
  }
  if (hasBME680) {
    if (hasOLED) displayScroll(" * rak1906");
    ClosedCube_BME680_Status status = bme680.readStatus();
    //  if (status.newDataFlag) {
    temp = bme680.readTemperature();
    HPa = bme680.readPressure();
    humid = bme680.readHumidity();
    lpp.addTemperature(channel++, temp);
    lpp.addRelativeHumidity(channel++, humid);
    lpp.addBarometricPressure(channel++, HPa);
  }
  if (hasHTU21D) {
    if (hasOLED) displayScroll(" * HTU21D");
    myHTU21D.measure();
    temp = myHTU21D.getTemperature();
    humid = myHTU21D.getHumidity();
    lpp.addTemperature(channel++, temp);
    lpp.addRelativeHumidity(channel++, humid);
  }
  if (has1903) {
    if (hasOLED) displayScroll(" * rak1903");
    rak1903_lux.update();
    lpp.addLuminosity(channel++, rak1903_lux.lux());
  }
  uint8_t ln = lpp.getSize();
  api.lorawan.precv(0);
  // turn off reception – a little hackish, but without that send might fail.
  uint8_t lBuff[48];
  memcpy(lBuff, lpp.getBuffer(), ln);
  bool rslt = api.lorawan.psend(ln, lBuff);
  sprintf(msg, "Sending LPP payload: %s\n", rslt ? "Success" : "Fail");
  Serial.print(msg);
  hexDump(lBuff, ln);
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
  if (hasOLED) {
    sprintf(msg, "LPP %d b: %s", ln, rslt ? "[o]" : "[x]");
    displayScroll(msg);
  }
}

void sendPing() {
  sprintf(msg, "PING #0x%04x", counter++);
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
    float Lux0 = bh1750.readLightLevel();
    sprintf(msg, "lux: %.2f %.2f", Lux0, Lux);
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
      Lux = bh1750.readLightLevel();
      sprintf(msg, "bh1750: %.2f", Lux);
      sendMsg(msg);
    }
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

void sendMsg(char* msgToSend) {
  uint8_t ln = strlen(msgToSend);
  api.lorawan.precv(0);
  // turn off reception – a little hackish, but without that send might fail.
  // memset(msg, 0, ln + 20);
  char buff[128];
  sprintf(buff, "Sending `%s`: %s\n", msgToSend, api.lorawan.psend(ln, (uint8_t*)msgToSend) ? "Success" : "Fail");
  Serial.print(buff);
#ifdef __RAKBLE_H__
  sendToBle(buff);
#endif
  if (hasOLED) {
    displayScroll("Sending P2P:");
    displayScroll(msgToSend);
  }
  /*
    // Results are not good...
    char test0[64], test1[64];
    Serial.println("Unishox2 compression:");
    int cLen = unishox2_compress(msg, ln, test0, USX_PSET_FAVOR_ALPHA);
    Serial.printf("Compressed: %d vs %d\n", ln, cLen);
    hexDump((uint8_t*)test0, cLen);
    int dLen = unishox2_decompress(test0, cLen, test1, USX_PSET_FAVOR_ALPHA);
    Serial.printf("Decompressed: %d vs %d\n", cLen, dLen);
    hexDump((uint8_t*)test1, dLen);
  */
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
    sprintf(msg, "rak12002 date: %04d/%02d/%02d", rak12002.getYear(), rak12002.getMonth(), rak12002.getDate());
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    sprintf(msg, "rak12002 time: %02d:%02d:%02d UTC", rak12002.getHour(), rak12002.getMinute(), rak12002.getSecond());
    Serial.println(msg);
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
    // Output if seconds have changed
    // Use sprintf() to pretty print the date/time with leading zeros
    sprintf(msg, "%04d/%02d/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    Serial.println(msg);
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

void i2cScan() {
  byte error, addr;
  uint8_t nDevices, ix = 0;
  Serial.println("\nI2C scan in progress...");
  nDevices = 0;
  Serial.print("   |   .0   .1   .2   .3   .4   .5   .6   .7   .8   .9   .A   .B   .C   .D   .E   .F\n");
  Serial.print("-------------------------------------------------------------------------------------\n0. |   .  ");
  char memo[64];
  char buff[32];
  /*
    if (hasOLED) {
    oledFill(&oled, 0, 1);
    oledSetContrast(&oled, 127);
    oledWriteString(&oled, 0, -1, -1, (char *)"LoRa p2p", FONT_16x16, 0, 1);
    oledWriteString(&oled, 0, 0, 2, (char *)"Scanning", FONT_8x8, 0, 1);
    }
    posY = 3;
  */
  int posX = 0;
  displayScroll("Scanning");
  memset(msg, 0, 128);
  uint8_t px = 0;
  for (addr = 1; addr < 128; addr++) {
    Wire.beginTransmission(addr);
    error = Wire.endTransmission();
    if (error == 0) {
      sprintf(msg + px, "0x%2x ", addr);
      Serial.print(msg + px);
      // msg[ix++] = addr;
      // I am not doing anything with the IDs for now.
      if (nDevices > 0 && nDevices % 3 == 0) {
        posY += 1;
        posX = 0;
        if (posY == 8) {
          posY = 7;
          for (uint8_t i = 0; i < 8; i++) {
            oledScrollBuffer(&oled, 0, 127, 2, 7, 1);
            oledDumpBuffer(&oled, NULL);
          }
        }
      }
      nDevices++;
      if (hasOLED) {
        oledWriteString(&oled, 0, posX, posY, msg + px, FONT_8x8, 0, 1);
        posX += 40;
      }
      px += 5;
    } else {
      Serial.print("  .  ");
    }
    if (addr > 0 && (addr + 1) % 16 == 0 && addr < 127) {
      Serial.write('\n');
      Serial.print(addr / 16 + 1);
      Serial.print(". | ");
    }
  }
  msg[px] = 0;
  Serial.println("\n-------------------------------------------------------------------------------------");
  Serial.println("I2C devices found: " + String(nDevices));
#ifdef __RAKBLE_H__
  sprintf(buff, "%d devices\n", nDevices);
  sendToBle(buff);
  sendToBle(msg);
#endif
  /*
    for (uint8_t i = 0; i < 8; i++) {
    oledScrollBuffer(&oled, 0, 127, 2, 7, 1);
    oledDumpBuffer(&oled, NULL);
    }
    oledWriteString(&oled, 0, 0, 7, buff, FONT_8x8, 0, 1);
    posY = 7;
  */
  displayScroll(buff);
}

void setup() {
  Serial.begin(115200, RAK_CUSTOM_MODE);
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
    Serial.printf("%d, ", x--);
    delay(500);
  } // Just for show
  Serial.println("0!");
  Serial.println("RAKwireless LoRa P2P Kitchen Sink");
  Serial.println("------------------------------------------------------");
  Wire.begin();
  Wire.setClock(400000);
  // Test for OLED
  Wire.beginTransmission(0x3c);
  delay(100);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("OLED present");
    if (hasOLED) {
      // the user wants OLED display
      uint8_t uc[8];
      rc = oledInit(&oled, OLED_128x64, 0x3c, FLIPPED, INVERTED, HARDWARE_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L);
      if (rc != OLED_NOT_FOUND) {
        oledSetBackBuffer(&oled, ucBuffer);
        oledSetTextWrap(&oled, 1);
        oledFill(&oled, 0, 1);
        oledSetContrast(&oled, 127);
        oledWriteString(&oled, 0, -1, -1, "LoRa p2p", FONT_16x16, 0, 1);
      } else hasOLED = false;
    } else {
      Serial.println("But you specified you didn't want OLED display!");
    }
  } else hasOLED = false;
  // Even if the user wanted it – since it ain't there, we set it to false.

  // Test for rtc
  Wire.beginTransmission(0x52);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("RAK12002 RTC present!");
    hasRAK12002 = true;
    rak12002.initI2C(); // First initialize and create the rtc device
    rak12002.writeToRegister(0x35, 0x00);
    rak12002.writeToRegister(0x37, 0xB4); //Direct Switching Mode (DSM): when VDD < VBACKUP, switchover occurs from VDD to VBACKUP
    rak12002.set24HourMode(); // Set the device to use the 24-hour format (default) instead of the 12-hour format
    if (hasOLED) displayScroll("* RAK12002 RTC");
  }

  if (!hasRAK12002) {
    Wire.beginTransmission(0x68);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.println("DS3231M RTC present!");
      hasDS3231M = DS3231M.begin();
      Serial.printf("DS3231M init %s\n", hasDS3231M ? "success" : "fail");
      if (hasDS3231M && hasOLED) displayScroll("* DS3231M RTC");
    }
  }

  // Test for rak1901
  Wire.beginTransmission(0x70);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("Temperature & Humidity Sensor present!");
    hasTH = th_sensor.init();
    Serial.printf("RAK1901 init %s\n", hasTH ? "success" : "fail");
    th_sensor.update();
    temp = th_sensor.temperature();
    humid = th_sensor.humidity();
    if (hasOLED) displayScroll("* rak1901");
  }

  // Test for rak1902
  Wire.beginTransmission(0x5C);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("Pressure Sensor present!");
    hasPA = p_sensor.init();
    Serial.printf("RAK1902 init %s\n", hasPA ? "success" : "fail");
    HPa = p_sensor.pressure(MILLIBAR);
    if (hasOLED) displayScroll("* rak1902");
  }

  // Test for rak1903
  Wire.beginTransmission(0x44);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("RAK1903 Light Sensor present!");
    has1903 = rak1903_lux.init();
    Serial.printf("RAK1903 init %s\n", has1903 ? "success" : "fail");
    Lux = rak1903_lux.lux();
    if (hasOLED) displayScroll("* rak1903");
  }

  // Test for BH1750
  Wire.beginTransmission(0x23);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("hasBH1750 Light Sensor present!");
    hasBH1750 = true;
    bh1750.begin();
    Lux = bh1750.readLightLevel();
    Serial.printf("BH1750 init: %d\n", Lux);
    if (hasOLED) displayScroll("* bh1750");
  }

  // Test for rak1906
  Wire.beginTransmission(0x76);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("RAK1906 Light Sensor present!");
    if (hasOLED) displayScroll("* rak1906");
    bme680.init(0x76); // I2C address: 0x76 or 0x77
    bme680.reset();
    Serial.print("Chip ID=0x");
    uint8_t id = bme680.getChipID();
    Serial.println(id, HEX);
    hasBME680 = (id != 0xFF);
    if (hasBME680) {
      Serial.println("RAK1906 init success!");
      bme680.setOversampling(BME680_OVERSAMPLING_X1, BME680_OVERSAMPLING_X2, BME680_OVERSAMPLING_X16);
      bme680.setIIRFilter(BME680_FILTER_3);
      bme680.setForcedMode();
    } else Serial.println("RAK1906 init fail!");
  }

  // Test for HTU21D
  Wire.beginTransmission(0x40);
  error = Wire.endTransmission();
  if (error == 0) {
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
  Serial.printf("Set infinite Rx mode %s\n", api.lorawan.precv(65534) ? "Success" : "Fail");
  startTime = millis();
}

void loop() {
#ifdef __RAKBLE_H__
  if (api.ble.uart.available()) {
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
    Serial.println("\nIncoming:");
    // if (hasOLED) displayScroll("Serial in:");
    char str1[256];
    uint8_t ix = 0;
    while (Serial.available()) {
      char c = Serial.read();
      if (c > 31) str1[ix++] = c;
    }
    str1[ix] = 0;
    Serial.println(str1);
    if (hasOLED) displayScroll(str1);
    handleCommands(str1);
  }
}
