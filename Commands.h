void i2cScan();
void displayTime();
void sendLux();
void sendBME680(bool);
void sendPA();
void sendHTU21D();
void sendTH();
void sendLPP();
void sendMsg(char*);
void sendPing();
float calcAlt(float);

void handleCommands(char *cmd) {
  if (cmd[0] != '/') return;
  // If the string doesn't start with / – it's not a command

  // /p2p command: display main P2P settings
  if (cmd[1] == 'p' && cmd[2] == '2' && cmd[3] == 'p') {
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
    return;
  }

  // /sf command
  if (cmd[1] == 's' && cmd[2] == 'f') {
    if (cmd[3] == '?' || cmd[3] == 0) {
      sprintf(msg, "P2P SF: %d\n", sf);
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) {
        displayScroll(msg);
      }
      return;
    } else if (cmd[3] == ' ') {
      // sf xxxx set SF
      uint16_t tmp = atoi(cmd + 4);
      if (tmp < 5 || tmp > 12) {
        sprintf(msg, "Invalid SF value: %d\n", tmp);
        Serial.print(msg);
#ifdef __RAKBLE_H__
        sendToBle(msg);
#endif
        return;
      }
      sf = tmp;
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
    } else {
      sprintf(msg, "Unknown command!");
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) {
        displayScroll(msg);
      }
      return;
    }
  }

  // /bw command
  if (cmd[1] == 'b' && cmd[2] == 'w') {
    if (cmd[3] == '?' || cmd[3] == 0) {
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
    } else if (cmd[3] == ' ') {
      // bw xxxx set BW
      uint16_t tmp = atoi(cmd + 4);
      if (tmp > 9) {
        sprintf(msg, "Invalid BW value: %d\n", tmp);
        Serial.print(msg);
#ifdef __RAKBLE_H__
        sendToBle(msg);
#endif
        return;
      }
      bw = myBWs[tmp];
      api.lorawan.precv(0);
      // turn off reception
      sprintf(msg, "Set P2P bandwidth to %d/%d: %s\n", tmp, bw, api.lorawan.pbw.set(bw) ? "Success" : "Fail");
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
    } else {
      sprintf(msg, "Unknown command!");
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) {
        displayScroll(msg);
      }
      return;
    }
  }

  // /sf command
  if (cmd[1] == 'c' && cmd[2] == 'r') {
    if (cmd[3] == '?' || cmd[3] == 0) {
      sprintf(msg, "P2P C/R: 4/%d\n", (cr + 5));
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) {
        displayScroll(msg);
      }
      return;
    } else if (cmd[3] == ' ') {
      // cr xxxx set CR
      uint16_t tmp = atoi(cmd + 4);
      if (tmp < 5 || tmp > 8) {
        sprintf(msg, "Invalid C/R value: %d\n", tmp);
        Serial.print(msg);
#ifdef __RAKBLE_H__
        sendToBle(msg);
#endif
        return;
      }
      cr = tmp - 5;
      api.lorawan.precv(0);
      // turn off reception
      sprintf(msg, "Set P2P coding rate to 4/%d: %s\n", (cr + 5), api.lorawan.pcr.set(cr) ? "Success" : "Fail");
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      api.lorawan.precv(65534);
      if (hasOLED) {
        sprintf(msg, "C/R set to %d", sf);
        displayScroll(msg);
      }
      return;
    } else {
      sprintf(msg, "Unknown command!");
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) {
        displayScroll(msg);
      }
      return;
    }
  }

  // /fq command
  if (cmd[1] == 'f' && cmd[2] == 'q') {
    if (cmd[3] == '?' || cmd[3] == 0) {
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
    } else if (cmd[3] == ' ') {
      // fq xxxx set frequency
      float tmp = atof(cmd + 4);
      if (tmp < 150.0 || tmp > 960.0) {
        // sx1262 freq range 150MHz to 960MHz
        // Your chip might not support all...
        sprintf(msg, "Invalid frequency value: %d\n", tmp);
        Serial.print(msg);
#ifdef __RAKBLE_H__
        sendToBle(msg);
#endif
        return;
      }
      myFreq = tmp * 1e6;
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

  // /tx command
  if (cmd[1] == 't' && cmd[2] == 'x') {
    if (cmd[3] == '?' || cmd[3] == 0) {
      sprintf(msg, "P2P TX power: %d\n", txPower);
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) {
        sprintf(msg, "TX power: %d", txPower);
        displayScroll(msg);
      }
      return;
    } else if (cmd[3] == ' ') {
      // tx xxxx set tx power
      uint16_t tmp = atoi(cmd + 4);
      if (tmp < 5 || tmp > 22) {
        sprintf(msg, "Invalid TX power value: %d\n", tmp);
        Serial.print(msg);
#ifdef __RAKBLE_H__
        sendToBle(msg);
#endif
        return;
      }
      txPower = tmp;
      api.lorawan.precv(0);
      // turn off reception
      sprintf(msg, "Set P2P TX power to %d: %s\n", txPower, api.lorawan.ptp.set(22) ? "Success" : "Fail");
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      api.lorawan.precv(65534);
      if (hasOLED) {
        sprintf(msg, "New TX pwr: %d", txPower);
        displayScroll(msg);
      }
      return;
    } else {
      sprintf(msg, "Unknown command!");
      Serial.print(msg);
#ifdef __RAKBLE_H__
      sendToBle(msg);
#endif
      if (hasOLED) {
        displayScroll(msg);
      }
      return;
    }
  }

  // RTC related commands
  // Only parsed if there's an RTC installed!
  if (hasDS3231M || hasRAK12002) {
    if (strcmp(cmd, "/rtc") == 0) {
      displayTime();
      return;
    }
    if (cmd[1] == 's' && cmd[2] == 'e' && cmd[3] == 't' && cmd[4] == ' ') {
      // Variables to hold parsed date/time
      // Use sscanf() to parse the date/time into component variables
      if (hasDS3231M) {
        unsigned int tokens, year, month, day, hour, minute, second;
        tokens = sscanf(cmd, "%*s %u-%u-%u %u:%u:%u;", &year, &month, &day, &hour, &minute, &second);
        if (tokens != 6) {
          // Check to see if it was parsed correctly
          Serial.print(F("Unable to parse date/time\n"));
        } else {
          DS3231M.adjust(DateTime(year, month, day, hour, minute, second));
          // Adjust the RTC date/time
#ifdef __RAKBLE_H__
          sendToBle("Date / Time set.");
#endif
          displayTime();
        }
      } else {
        unsigned int tokens, year, month, dayOfWeek, day, hour, minute, second;
        tokens = sscanf(cmd, "%*s %u-%u-%u %u %u:%u:%u;", &year, &month, &dayOfWeek, &day, &hour, &minute, &second);
        if (tokens != 7) {
          // Check to see if it was parsed correctly
          Serial.print(F("Unable to parse date/time\n"));
        } else {
          rak12002.setTime(year, month, dayOfWeek, day, hour, minute, second);
          // Adjust the RTC date/time
#ifdef __RAKBLE_H__
          sendToBle("Date / Time set.");
#endif
          displayTime();
        }
      }
      return;
    }
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

  if (strcmp(cmd, "/ping") == 0) {
    sendPing();
    return;
  }

  if (cmd[1] == '>' && cmd[2] == ' ') {
    sendMsg(cmd + 3);
    return;
  }

  if (strcmp(cmd, "/lpp") == 0) {
    // Sends all the available data points as a Cayenne LPP packet
    sendLPP();
    return;
  }

  // the /th command works for any sensor that has T and H
  if (strcmp(cmd, "/th") == 0) {
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
    return;
  }

  if (strcmp(cmd, "/pa") == 0) {
    if (hasPA) sendPA();
    else Serial.println("No RAK1902 module installed!");
    return;
  }

  if (strcmp(cmd, "/bme") == 0) {
    if (hasBME680) sendBME680(true);
    else Serial.println("No RAK1906 module installed!");
    return;
  }

  if (strcmp(cmd, "/lux") == 0) {
    if (has1903 || hasBH1750) sendLux();
    else Serial.println("No light sensor installed!");
    return;
  }
}
