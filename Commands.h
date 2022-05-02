#include <stdio.h>
#include <string.h>

void i2cScan(char*);
void displayTime();
void sendLux();
void sendBME680(bool);
void sendPA();
void sendHTU21D();
void sendTH();
void sendLPP(char*);
void sendMsg(char*);
void sendPing(char*);
float calcAlt(float);
void handleHelp(char *);
void whoami(char *);
void handleP2P(char *);
void handleFreq(char *);
void handleBW(char *);
void handleSF(char *);
void handleCR(char *);
void handleTX(char *);
void handleLux(char *);
void handleTH(char *);
void handlePA(char *);
void handleBME(char *);
void handleALT(char *);
void handleMSL(char *);
void handleRTC(char *);
void handleAutoPing(char *);
void handleAES(char *);
void handleOCP(char *);
void handlePongBack(char *);
void handleSerial1(char *);
void handleSendMsg(char *);

bool needAES = false, needJSON = false, pongBack = false, autoPing = true;
uint32_t apPeriod = 30000, lastPing;
char pwd[32];
uint8_t OCP_value = 0x38;

int cmdCount = 0;
struct myCommand {
  void (*ptr)(char *); // Function pointer
  char name[12];
  char help[48];
};

myCommand cmds[] = {
  {handleHelp, "help", "Shows this help."},
  {i2cScan, "i2c", "Scans the I2C bus."},
#ifdef __RAKBLE_H__
  {whoami, "whoami", "Gets the BLE broadcast name."},
#endif
  {sendPing, "ping", "Sends a ping."},
  {handleP2P, "p2p", "Shows the P2P settings."},
  {handleFreq, "fq", "Gets/sets the working frequency."},
  {handleBW, "bw", "Gets/sets the working bandwidth."},
  {handleSF, "sf", "Gets/sets the working spreading factor."},
  {handleCR, "cr", "Gets/sets the working coding rate."},
  {handleTX, "tx", "Gets/sets the working TX power."},
  {handleLux, "lux", "Gets the ambient light."},
  {handleTH, "th", "Gets the temperature and humidity."},
  {handlePA, "pa", "Gets the atmospheric pressure."},
  {handleBME, "bme", "Gets data (T/H/Pa) from a BME680."},
  {handleALT, "alt", "Computes altitude."},
  {handleMSL, "msl", "Gets/sets the MSL pressure."},
  {sendLPP, "lpp", "Sends a Cayenne packet."},
  {handleRTC, "rtc", "Gets/sets datetime."},
  {handleAES, "aes", "AES-related commands."},
  {handleOCP, "ocp", "Gets/sets OCP value."},
  {handleAutoPing, "ap", "autoping 0/x seconds."},
  {handlePongBack, "pb", "Gets/sets pong back."},
  {handleSerial1, "s1", "Enables/disables Serial1."},
  {handleSendMsg, "send", "Sends a custom P2P packet."},
};

void handleSendMsg(char *param) {
  memset(msg, 0, 128);
  uint8_t i = sscanf(param, "/send %s", msg);
  if (i > -1) {
    sendMsg(msg);
  }
}

void handleOCP(char *param) {
  uint16_t value;
  int i = sscanf(param, "/ocp %d", &value);
  if (i == 1) {
    // 60 to 140
    if (value < 60) value = 60;
    else if (value > 140) value = 140;
    value *= 10;
    OCP_value = value / 25;
    // SX126xWriteRegister(0x08e7, OCP_value);
    sprintf(msg, "OCP = 0x%2x [%d]", OCP_value, (value / 10));
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
  } else {
    sprintf(msg, "OCP = 0x%2x [%d]", OCP_value, (OCP_value * 2.5));
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
  }
}

void handlePongBack(char *param) {
  uint8_t value;
  int i = sscanf(param, "/pb %d", &value);
  if (i == 1) {
    // 0 or 1..n OFF or ON
    sprintf(msg, "pb %s [%d]", (value == 0) ? "off" : "on", value);
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
    pongBack = (value > 0);
  } else {
    sprintf(msg, "pong %s", pongBack ? "on" : "off");
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
  }
}

void handleSerial1(char *param) {
  uint8_t value;
  int i = sscanf(param, "/s1 %d", &value);
  if (i == 1) {
    // 0 or 1..n OFF or ON
    sprintf(msg, "s1 %s [%d]", (value == 0) ? "off" : "on", value);
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
    needSerial1 = (value > 0);
    if (needSerial1) Serial1.begin(115200, RAK_CUSTOM_MODE);
    else Serial1.end();
  } else {
    sprintf(msg, "Serial1 %s", needSerial1 ? "on" : "off");
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
  }
}

void handleAutoPing(char *param) {
  uint8_t value;
  int i = sscanf(param, "/ap %d", &value);
  if (i == 1) {
    // 0 OFF or xx ON
    sprintf(msg, "ap %s [%d]", (value == 0) ? "off" : "on", value);
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
    autoPing = (value > 0);
    apPeriod = 1000 * value;
    lastPing = millis();
  } else {
    sprintf(msg, "ap %s [%d]", autoPing ? "on" : "off", (apPeriod / 1000));
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
  }
}

void handleAES(char *param) {
  int i = sscanf(param, "/aes off");
  if (i == 0) {
    // OFF
    sprintf(msg, "aes OFF");
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
    needAES = false;
    return;
  }
  i = sscanf(param, "/aes %s", msg);
  if (i > -1) {
    // ON with pwd
    i = strlen(msg);
    if (i != 16) {
      // AES128 pwd len = 16 bytes
      // Later we will accept hex string 010203 etc of 32 bytes
      sprintf(msg, "wrong aes pwd length!");
      Serial.println(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) displayScroll(msg);
      needAES = false;
      return;
    }
    memcpy(pwd, msg, 16);
    sprintf(msg, "aes ON!");
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) displayScroll(msg);
    needAES = true;
    return;
  }
}

void handleHelp(char *param) {
  Serial.printf("Available commands: %d\n", cmdCount);
  for (int i = 0; i < cmdCount; i++) {
    sprintf(msg, " . /%s: %s", cmds[i].name, cmds[i].help);
    Serial.println(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
  }
}

#ifdef __RAKBLE_H__
void whoami(char* param) {
  sprintf(msg, "Broadcast name: %s", api.ble.settings.broadcastName.get());
  Serial.println(msg);
  sendToBle(api.ble.settings.broadcastName.get());
  if (hasOLED) {
    displayScroll("Broadcast name:");
    displayScroll( api.ble.settings.broadcastName.get());
  }
  return;
}
#endif

void handleP2P(char *param) {
  float f0 = myFreq / 1e6, f1 = api.lorawan.pfreq.get() / 1e6;
  // check stored value vs real value
  sprintf(msg, "P2P frequency: %.3f/%.3f MHz\n", f0, f1);
  Serial.print(msg);
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
  if (hasOLED) {
    sprintf(msg, "Fq: %.3f MHz\n", f1);
    displayScroll(msg);
  }
  sprintf(msg, "P2P SF: %d\n", sf);
  Serial.print(msg);
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
  if (hasOLED) {
    displayScroll(msg);
  }
  sprintf(msg, "P2P bandwidth: %d KHz\n", bw);
  Serial.print(msg);
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
  if (hasOLED) {
    sprintf(msg, "BW: %d KHz", bw);
    displayScroll(msg);
  }
  sprintf(msg, "P2P C/R: 4/%d\n", (cr + 5));
  Serial.print(msg);
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
  if (hasOLED) {
    displayScroll(msg);
  }
  sprintf(msg, "P2P TX power: %d\n", txPower);
  Serial.print(msg);
#ifdef __RAKBLE_H__
  sendToBle(msg);
#endif
  if (hasOLED) {
    sprintf(msg, "TX power: %d", txPower);
    displayScroll(msg);
  }
}

void handleFreq(char *param) {
  float value;
  int i = sscanf(param, "/fq %f", &value);
  if (i == -1) {
    // no parameters
    sprintf(msg, "P2P frequency: %.3f MHz\n", (myFreq / 1e6));
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) {
      sprintf(msg, "Fq: %.3f MHz\n", (myFreq / 1e6));
      displayScroll(msg);
    }
    return;
  } else {
    // fq xxx.xxx set frequency
    value = atof(param + 4);
    // for some reason sscanf returns 0.000 as value...
    if (value < 150.0 || value > 960.0) {
      // sx1262 freq range 150MHz to 960MHz
      // Your chip might not support all...
      sprintf(msg, "Invalid frequency value: %.3f\n", value);
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      return;
    }
    myFreq = value * 1e6;
    api.lorawan.precv(0);
    // turn off reception
    sprintf(msg, "Set P2P frequency to %3.3f: %s MHz\n", (myFreq / 1e6), api.lorawan.pfreq.set(myFreq) ? "Success" : "Fail");
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    api.lorawan.precv(65534);
    if (hasOLED) {
      sprintf(msg, "New freq: %.3f", myFreq);
      displayScroll(msg);
    }
    return;
  }
}

void handleBW(char*param) {
  int value;
  int i = sscanf(param, "/%*s %d", &value);
  if (i == -1) {
    // no parameters
    sprintf(msg, "P2P bandwidth: %d KHz\n", bw);
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) {
      sprintf(msg, "BW: %d KHz", bw);
      displayScroll(msg);
    }
    return;
  } else {
    // bw xxxx set BW
    if (value > 9) {
      sprintf(msg, "Invalid BW value: %d\n", value);
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      return;
    }
    bw = myBWs[value];
    api.lorawan.precv(0);
    // turn off reception
    sprintf(msg, "Set P2P bandwidth to %d/%d: %s\n", value, bw, api.lorawan.pbw.set(bw) ? "Success" : "Fail");
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    api.lorawan.precv(65534);
    if (hasOLED) {
      sprintf(msg, "New BW: %d", bw);
      displayScroll(msg);
    }
    return;
  }
}

void handleSF(char*param) {
  int value;
  int i = sscanf(param, "/%*s %d", &value);
  if (i == -1) {
    // no parameters
    sprintf(msg, "P2P SF: %d\n", sf);
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) {
      sprintf(msg, "SF: %d", sf);
      displayScroll(msg);
    }
    return;
  } else {
    // sf xxxx set SF
    if (value < 5 || value > 12) {
      sprintf(msg, "Invalid SF value: %d\n", value);
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      return;
    }
    sf = value;
    api.lorawan.precv(0);
    // turn off reception
    sprintf(msg, "Set P2P spreading factor to %d: %s\n", sf, api.lorawan.psf.set(sf) ? "Success" : "Fail");
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    api.lorawan.precv(65534);
    if (hasOLED) {
      sprintf(msg, "SF set to %d", sf);
      displayScroll(msg);
    }
    return;
  }
}

void handleCR(char*param) {
  int value;
  int i = sscanf(param, "/%*s %d", &value);
  if (i == -1) {
    // no parameters
    sprintf(msg, "P2P CR: 4/%d\n", (cr + 5));
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) {
      sprintf(msg, "CR: 4/%d", (cr + 5));
      displayScroll(msg);
    }
    return;
  } else {
    // sf xxxx set SF
    if (value < 5 || value > 8) {
      sprintf(msg, "Invalid CR value: %d\n", value);
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      return;
    }
    cr = value - 5;
    api.lorawan.precv(0);
    // turn off reception
    sprintf(msg, "Set P2P coding rate to %d: %s\n", cr, api.lorawan.pcr.set(cr) ? "Success" : "Fail");
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    api.lorawan.precv(65534);
    if (hasOLED) {
      sprintf(msg, "CR set to 4/%d", (cr + 5));
      displayScroll(msg);
    }
    return;
  }
}

void handleTX(char*param) {
  int value;
  int i = sscanf(param, "/%*s %d", &value);
  if (i == -1) {
    // no parameters
    sprintf(msg, "P2P TX power: %d\n", txPower);
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) {
      sprintf(msg, "Tx pwr: %d", txPower);
      displayScroll(msg);
    }
    return;
  } else {
    // sf xxxx set SF
    if (value < 5 || value > 22) {
      sprintf(msg, "Invalid TX power value: %d\n", value);
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      return;
    }
    txPower = value;
    api.lorawan.precv(0);
    // turn off reception
    sprintf(msg, "Set P2P Tx power to %d: %s\n", cr, api.lorawan.ptp.set(txPower) ? "Success" : "Fail");
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    api.lorawan.precv(65534);
    if (hasOLED) {
      sprintf(msg, "Tx pwr set to %d", txPower);
      displayScroll(msg);
    }
    return;
  }
}

void handleTH(char *param) {
  bool hasSomething = false;
  if (hasTH) {
    sendTH();
    hasSomething = true;
  }
  if (hasHTU21D) {
    sendHTU21D();
    hasSomething = true;
  }
  if (hasBME680) {
    sendBME680(false);
    hasSomething = true;
  }
  if (!hasSomething) {
    sprintf(msg, "No Temp/Humidity module installed!");
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
  }
}

void handlePA(char *param) {
  if (hasPA) sendPA();
  else Serial.println("No RAK1902 module installed!");
}

void handleBME(char *param) {
  if (hasBME680) sendBME680(true);
  else Serial.println("No RAK1906 module installed!");
}

void handleALT(char *param) {
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
}

void handleMSL(char *param) {
  float value;
  int i = sscanf(param, "/msl %f", &value);
  if (i == -1) {
    // no parameters
    sprintf(msg, "Current MSL: %.3f HPa\n", MSL);
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    if (hasOLED) {
      sprintf(msg, "MSL: %.3f HPa\n", MSL);
      displayScroll(msg);
    }
    return;
  } else {
    // msl xxx.xxx set MSL
    value = atof(param + 5);
    // for some reason sscanf returns 0.000 as value...
    if (value > 900.0 && value < 1100.0) {
      MSL = value;
      sprintf(msg, "MSL set to: %.2f HPa\n", MSL);
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) {
        sprintf(msg, "New MSL: %.2f HPa", MSL);
        displayScroll(msg);
      }
    } else {
      Serial.printf("Incorrect MSL: %.2f", value);
    }
    return;
  }
}

void handleLux(char *param) {
  if (has1903 || hasBH1750) sendLux();
  else Serial.println("No light sensor installed!");
}

void handleRTC(char* param) {
  if (!hasDS3231M && !hasRAK12002) {
    sprintf(msg, "No RTC module installed!");
    Serial.print(msg);
#ifdef __RAKBLE_H__
    sendToBle(msg);
#endif
    return;
  }
  // Variables to hold parsed date/time
  // Use sscanf() to parse the date/time into component variables
  if (hasDS3231M) {
    unsigned int tokens, year, month, day, hour, minute, second;
    tokens = sscanf(param, "%*s %u-%u-%u %u:%u:%u;", &year, &month, &day, &hour, &minute, &second);
    if (tokens != 6) {
      displayTime();
      return;
    }
    DS3231M.adjust(DateTime(year, month, day, hour, minute, second));
    // Adjust the RTC date/time
#ifdef __RAKBLE_H__
    sendToBle("Date / Time set.");
#endif
    displayTime();
    return;
  } else {
    // hasRAK12002
    unsigned int tokens, year, month, dayOfWeek, day, hour, minute, second;
    tokens = sscanf(param, "%*s %u/%u/%u %u %u:%u:%u;", &year, &month, &day, &dayOfWeek, &hour, &minute, &second);
    //Serial.printf("tokens: %d\n", tokens);
    if (tokens != 7) {
      // Check to see if it was parsed correctly
      displayTime();
      return;
    } else {
      rak12002.setTime(year, month, dayOfWeek, day, hour, minute, second);
      // Adjust the RTC date/time
#ifdef __RAKBLE_H__
      sendToBle("Date / Time set.");
#endif
      Serial.println(F("Date / Time set to:"));
      displayTime();
    }
  }
}

/*
  if (cmd[1] == '>' && cmd[2] == ' ') {
    sendMsg(cmd + 3);
    return;
  }
*/

void evalCmd(char *str, char *fullString) {
  char strq[12];
  for (int i = 0; i < cmdCount; i++) {
    sprintf(strq, "%s?", cmds[i].name);
    if (strcmp(str, cmds[i].name) == 0 || strcmp(strq, str) == 0) {
      cmds[i].ptr(fullString);
      return;
    }
  }
}

void handleCommands(char *str1) {
  char kwd[32];
  int i = sscanf(str1, "/%s", kwd);
  if (i > 0) evalCmd(kwd, str1);
  else handleHelp("");
  oledLastOn = millis();
}
