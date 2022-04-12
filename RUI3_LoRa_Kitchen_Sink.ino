#include "rak1901.h"
#include "rak1902.h"
#include "rak1903.h"
// #include "unishox2.h"
#include "ClosedCube_BME680.h"
#include <ss_oled.h>
/*
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

DS3231M_Class DS3231M; // /< Create an instance of the DS3231M class
CayenneLPP lpp(51);
//Create an instance of the HTU21D object
HTU21D myHTU21D;
/** Temperature & Humidity sensor **/
rak1901 th_sensor;
/** Air Pressure sensor **/
rak1902 p_sensor;
rak1903 lux_sensor;
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
bool hasOLED = true, hasTH = false, hasPA = false, hasLux = false, hasBME680 = false, hasHTU21D = false, hasDS3231M = false;
float temp, humid, HPa, Lux;
long startTime;
// LoRa SETUP
// The LoRa chip come pre-wired: all you need to do is define the parameters:
// frequency, SF, BW, CR, Preamble Length and TX power
double myFreq = 868000000;
float MSL = 1013.5;
uint16_t counter = 0, sf = 12, bw = 125, cr = 0, preamble = 8, txPower = 22;
char msg[128];

void hexDump(uint8_t* buf, uint16_t len) {
  // Something similar to the Unix/Linux hexdump -C command
  // Pretty-prints the contents of a buffer, 16 bytes a row
  char alphabet[17] = "0123456789abcdef";
  uint16_t i, index;
  Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
  Serial.print(F("   |.0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .a .b .c .d .e .f | |      ASCII     |\n"));
  for (i = 0; i < len; i += 16) {
    if (i % 128 == 0) Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
    char s[] = "|                                                | |                |\n";
    uint8_t ix = 1, iy = 52, j;
    for (j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        s[ix++] = alphabet[(c >> 4) & 0x0F];
        s[ix++] = alphabet[c & 0x0F];
        ix++;
        if (c > 31 && c < 128) s[iy++] = c;
        else s[iy++] = '.';
      }
    }
    index = i / 16;
    if (i < 256) Serial.write(' ');
    Serial.print(index, HEX); Serial.write('.');
    Serial.print(s);
  }
  Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
}

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
    displayScroll((char*)data.Buffer);
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

#ifdef __RAKBLE_H__
void sendToBle(char *msgToSend) {
  Serial.println(F("Sending to BLE..."));
  api.ble.uart.write((uint8_t*)msgToSend, strlen(msgToSend));
}
#endif


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
  if (hasLux) {
    if (hasOLED) displayScroll(" * rak1903");
    lux_sensor.update();
    lpp.addLuminosity(channel++, lux_sensor.lux());
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
  if (lux_sensor.update()) {
    Lux = lux_sensor.lux();
    sprintf(msg, "Lux: %.2f", Lux);
    sendMsg(msg);
  } else Serial.println("Couldn't update lux sensor!");
}

void sendBME680() {
  ClosedCube_BME680_Status status = bme680.readStatus();
  //  if (status.newDataFlag) {
  temp = bme680.readTemperature();
  HPa = bme680.readPressure();
  humid = bme680.readHumidity();
  sprintf(msg, "%.2f C %.2f%% %.2f HPa", temp, humid, HPa);
  if (hasOLED) {
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
  sprintf(msg, "Sending `%s`: %s\n", msgToSend, api.lorawan.psend(ln, (uint8_t*)msgToSend) ? "Success" : "Fail");
  Serial.print(msg);
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
  if (hasOLED) {
    displayScroll("Sending P2P:");
    displayScroll(msgToSend);
  }
  /*
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
}

void handleCommands(char *cmd) {
  if (cmd[0] != '/') return;
  // If the string doesn't start with / – it's not a command

  if (hasDS3231M) {
    if (cmd[1] == 's' &cmd[2] == 'e' &cmd[3] == 't' &cmd[4] == ' ') {
      unsigned int tokens, year, month, day, hour, minute, second;
      // Variables to hold parsed date/time
      // Use sscanf() to parse the date/time into component variables
      tokens = sscanf(cmd, "%*s %u-%u-%u %u:%u:%u;", &year, &month, &day, &hour, &minute, &second);
      if (tokens != 6) {
        // Check to see if it was parsed correctly
        Serial.print(F("Unable to parse date/time\n"));
      } else {
        DS3231M.adjust(DateTime(year, month, day, hour, minute, second));
        // Adjust the RTC date/time
        Serial.print(F("Date / Time set."));
#ifdef __RAKBLE_H__
        sendToBle("Date / Time set.");
#endif
      }
      displayTime();
      return;
    }
    if (strcmp(cmd, "/rtc") == 0) {
      displayTime();
    }
  }

  if (strcmp(cmd, "/ping") == 0) {
    sendPing();
    return;
  }

  if (cmd[1] == '>' && cmd[2] == ' ') {
    sendMsg(cmd + 3);
    return;
  }

  if (strcmp(cmd, "/alt") == 0) {
    float pressure = 0.0;
    if (hasPA) {
      float alt = calcAlt(HPa);
      sprintf(msg, "1902: %.2f m", alt);
      Serial.println(msg);
      if (hasOLED) displayScroll(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
    }
    if (hasBME680) {
      ClosedCube_BME680_Status status = bme680.readStatus();
      float alt = calcAlt(bme680.readPressure());
      sprintf(msg, "bme: %.2f m", alt);
      Serial.println(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) displayScroll(msg);
    }
    return;
  }

  if (strlen(cmd) > 7) {
    // /msl 996.2
    if (cmd[1] == 'm' && cmd[2] == 's' && cmd[3] == 'l' && cmd[4] == ' ') {
      float x = atof(cmd + 5);
      if (x > 900.0 && x < 1100.0) {
        MSL = x;
        Serial.printf("MSL set to: %.2f HPa", MSL);
#ifdef __RAKBLE_H__
        sendToBle(msg);
#endif
        if (hasOLED) {
          sprintf(msg, "New MSL: %.2f HPa", MSL);
          displayScroll(msg);
        }
      } else {
        Serial.printf("Incorrect MSL: %.2f", x);
      }
      return;
    }
  }

#ifdef __RAKBLE_H__
  if (strcmp(cmd, "/whoami") == 0) {
    sprintf(msg, "Broadcast name: %s\n", api.ble.settings.broadcastName.get());
    Serial.println(msg);
    if (hasOLED) displayScroll(msg);
    sendToBle(api.ble.settings.broadcastName.get());
    return;
  }
#endif

  if (strcmp(cmd, "/i2c") == 0) {
    i2cScan();
    return;
  }

  if (strcmp(cmd, "/lpp") == 0) {
    sendLPP();
    return;
  }

  if (strcmp(cmd, "/th") == 0) {
    if (hasTH) sendTH();
    else Serial.println("No RAK1901 module installed!");
    return;
  }

  if (strcmp(cmd, "/htu") == 0) {
    if (hasHTU21D) sendHTU21D();
    else Serial.println("No HTU21D module installed!");
    return;
  }

  if (strcmp(cmd, "/pa") == 0) {
    if (hasPA) sendPA();
    else Serial.println("No RAK1902 module installed!");
    return;
  }

  if (strcmp(cmd, "/bme") == 0) {
    if (hasBME680) sendBME680();
    else Serial.println("No RAK1906 module installed!");
    return;
  }

  if (strcmp(cmd, "/lux") == 0) {
    if (hasLux) sendLux();
    else Serial.println("No RAK1903 module installed!");
    return;
  }
}

int posY = 1;
void displayScroll(char *msgToSend) {
  posY += 1;
  if (posY == 8) {
    posY = 7;
    for (uint8_t i = 0; i < 8; i++) {
      oledScrollBuffer(&oled, 0, 127, 2, 7, 1);
      oledDumpBuffer(&oled, NULL);
    }
  }
  oledWriteString(&oled, 0, 0, posY, msgToSend, FONT_8x8, 0, 1);
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
    // Wire.beginTransmission(addr);
    // error = Wire.endTransmission();
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
      Serial.print("  . ");
    }
    if (addr > 0 && (addr + 1) % 16 == 0 && addr < 127) {
      Serial.write('\n');
      Serial.print(addr / 16 + 1);
      Serial.print(". | ");
    }
  }
  Serial.println("\n-------------------------------------------------------------------------------------");
  Serial.println("I2C devices found: " + String(nDevices));
#ifdef __RAKBLE_H__
  sprintf(buff, "%d devices", nDevices);
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
  Wire.beginTransmission(0x68);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("DS3231M RTC present!");
    hasDS3231M = DS3231M.begin();
    Serial.printf("DS3231M init %s\n", hasDS3231M ? "success" : "fail");
    if (hasDS3231M && hasOLED) displayScroll("* DS3231M RTCC");
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
    hasLux = lux_sensor.init();
    Serial.printf("RAK1903 init %s\n", hasLux ? "success" : "fail");
    Lux = lux_sensor.lux();
    if (hasOLED) displayScroll("* rak1903");
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
  Serial.printf("Set P2P code rate to 4/%d: %s\n", (cr + 5), api.lorawan.pcr.set(0) ? "Success" : "Fail");
  Serial.printf("Set P2P preamble length to %d: %s\n", preamble, api.lorawan.ppl.set(8) ? "Success" : "Fail");
  Serial.printf("Set P2P TX power to %d: %s\n", txPower, api.lorawan.ptp.set(22) ? "Success" : "Fail");

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

  char ble_name[] = "3615_My_RAK"; // You have to be French to understand this joke
  Serial.print("Setting Broadcast Name to: ");
  Serial.println(ble_name);
  api.ble.settings.broadcastName.set(ble_name, strlen(ble_name));
  api.ble.uart.start();
  api.ble.advertise.start(0);
#endif
  // This version doesn't have an automatic Tx functionality:
  // YOU are in charge of sending, either via Serial or BLE.
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
