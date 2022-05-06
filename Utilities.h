void switchOLED(bool onOff) {
  oledON = onOff;
  oledSetContrast(&oled, onOff ? 127 : 0);
  oledPower(&oled, onOff ? 1 : 0);
}

#ifdef __RAKBLE_H__
void sendToBle(char *msgToSend) {
  api.ble.uart.write((uint8_t*)msgToSend, strlen(msgToSend));
}
#endif

int posY = 1; // First 2 lines are reserved for the title.
// Since displayScroll starts with incrementing posY, we start with posY = 1
void displayScroll(char *msgToSend) {
  posY += 1;
  if (posY == 8) {
    posY = 7; // keep it at 7, the last line
    for (uint8_t i = 0; i < 8; i++) {
      // and scroll, one pixel line – not text line – at a time.
      oledScrollBuffer(&oled, 0, 127, 2, 7, 1);
      oledDumpBuffer(&oled, NULL);
    }
  }
  // then display text
  oledWriteString(&oled, 0, 0, posY, msgToSend, FONT_8x8, 0, 1);
}

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
    // pre-formated line. We will replace the spaces with text when appropriate.
    uint8_t ix = 1, iy = 52, j;
    for (j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        // fastest way to convert a byte to its 2-digit hex equivalent
        s[ix++] = alphabet[(c >> 4) & 0x0F];
        s[ix++] = alphabet[c & 0x0F];
        ix++;
        if (c > 31 && c < 128) s[iy++] = c;
        else s[iy++] = '.'; // display ASCII code 0x20-0x7F or a dot.
      }
    }
    index = i / 16;
    // display line number then the text
    if (i < 256) Serial.write(' ');
    Serial.print(index, HEX); Serial.write('.');
    Serial.print(s);
  }
  Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
}

uint8_t myBus[128];
void i2cScan(char* param) {
  byte error, addr;
  memset(myBus, 0, 128);
  uint8_t nDevices, ix = 0;
  Serial.println("\nI2C scan in progress...");
  nDevices = 0;
  Serial.print("   |   .0   .1   .2   .3   .4   .5   .6   .7   .8   .9   .A   .B   .C   .D   .E   .F\n");
  Serial.print("-------------------------------------------------------------------------------------\n0. |   .  ");
  char memo[64];
  char buff[32];
  int posX = 0;
  displayScroll("Scanning");
  memset(msg, 0, 128);
  uint8_t px = 0;
  for (addr = 1; addr < 128; addr++) {
    Wire.beginTransmission(addr);
    error = Wire.endTransmission();
    if (error == 0) {
      sprintf(msg + px, "0x%2x      ", addr);
      // more spaces than required to be sure to erase "Scanning"
      msg[px + 5] = 0;
      Serial.print(msg + px);
      msg[px + 5] = ' ';
      // save the addr in the table
      // makes it easier to detect available devices during setup()
      myBus[addr] = addr;
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
  sprintf(buff, "devices: %d     ", nDevices);
  displayScroll(buff);
}

uint8_t backup0[1024];
uint32_t lastKbdInput;
bool isBBQing = false;

void restoreBBQinput() {
  if (hasOLED) {
    switchOLED(true);
    memcpy(backup0, ucBuffer, 1024);
    // save the current screen
    oledFill(&oled, 0, 1); // erase everything
    oledWriteString(&oled, 0, 10, 0, "BBQ10", FONT_16x16, 0, 1);
    oledWriteString(&oled, 0, 0, 2, ">", FONT_8x8, 0, 1);
    oledWriteString(&oled, 0, -1, 2, bbqBuff, FONT_8x8, 0, 1);
    // -1 means current position
  }
  lastKbdInput = millis();
  isBBQing = true;
}

void restoreScreen() {
  if (hasOLED) {
    switchOLED(true); // in case if went off in between
    memcpy(ucBuffer, backup0, 1024); // restore screen
    oledDumpBuffer(&oled, ucBuffer);
  }
  isBBQing = false;
}

struct myCodes {
  uint8_t code;
  uint8_t size;
  uint16_t mult;
  char *name;
};
uint8_t codeCount = 0;
myCodes lppCodes[] = {
  {LPP_UNIXTIME, LPP_UNIXTIME_SIZE, LPP_UNIXTIME_MULT, "UnixTime"},
  {LPP_TEMPERATURE, LPP_TEMPERATURE_SIZE, LPP_TEMPERATURE_MULT, "Temperature"},
  {LPP_RELATIVE_HUMIDITY, LPP_RELATIVE_HUMIDITY_SIZE, LPP_RELATIVE_HUMIDITY_MULT, "Humidity"},
  {LPP_BAROMETRIC_PRESSURE, LPP_BAROMETRIC_PRESSURE_SIZE, LPP_BAROMETRIC_PRESSURE_MULT, "Pressure"},
  {LPP_ALTITUDE, LPP_ALTITUDE_SIZE, LPP_ALTITUDE_MULT, "Altitude"},
  {LPP_LUMINOSITY, LPP_LUMINOSITY_SIZE, LPP_LUMINOSITY_MULT, "Luminosity"},
};

void decodeLPP(char *buff, uint8_t len) {
  //  hexDump((uint8_t*)buff, len);
  uint8_t myIndex = 0;
  while (myIndex < len) {
    uint8_t channelIndex = buff[myIndex++];
    uint8_t codeIndex = buff[myIndex++];
    // Serial.printf("\nHandling Channel %d, Code %d\n", channelIndex, codeIndex);
    int foundCode = -1;
    for (uint8_t i = 0; i < codeCount; i++) {
      // Serial.printf("%d: Code %d vs %d\n", i, codeIndex, lppCodes[i].code);
      if (lppCodes[i].code == codeIndex) {
        foundCode = i;
        // Serial.printf("foundCode = %d at %d\n", foundCode, i);
        i = codeCount;
      }
    }
    if (foundCode == -1) {
      Serial.printf("Code %d unknown! Aborting...\n\n", codeIndex);
      return;
    }
    myCodes cd = lppCodes[foundCode];
    if (cd.size == 1) {
      // Serial.printf("Size 1, cd.mult = %d\n", cd.mult);
      // hexDump((uint8_t*)(buff + myIndex - 2), 3);
      uint8_t n = buff[myIndex++];
      if (cd.mult == 1) Serial.printf("%s: %d\n", cd.name, n);
      else Serial.printf("%s: %.2f\n", cd.name, (float)(n / cd.mult));
    } else if (cd.size == 2) {
      // Serial.printf("Size 2, cd.mult = %d\n", cd.mult);
      // hexDump((uint8_t*)(buff + myIndex - 2), 4);
      double nn = (buff[myIndex] << 8) | buff[myIndex + 1];
      myIndex += 2;
      if (cd.mult == 1) Serial.printf("%s: %d\n", cd.name, nn);
      else {
        float ff = nn / cd.mult;
        Serial.printf("%s: %.2f\n", cd.name, ff);
      }
    } else if (cd.size == 4) {
      // Serial.printf("Size 4, cd.mult = %d\n", cd.mult);
      // hexDump((uint8_t*)(buff + myIndex - 2), 6);
      uint32_t nnnn = 0;
      for (uint8_t i = 0; i < 4; i++) nnnn = (nnnn << 8) + buff[myIndex + i];
      myIndex += 4;
      if (cd.mult == 1) {
        //if (hasDS3231M) {
        if (cd.code == LPP_UNIXTIME) {
          DateTime now = DateTime(nnnn);
          Serial.printf("Unix Time [%d]: %04d/%02d/%02d %02d:%02d:%02d\n", nnnn, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
        }
        //}
        else Serial.printf(" . %s: %d\n", cd.name, nnnn);
      } else {
        float ff = nnnn / cd.mult;
        Serial.printf(" * %s: %.2f\n", cd.name, ff);
      }
    } else {
      Serial.printf("Unknown length: %d\n", cd.size);
      return;
    }
    //Serial.printf("myIndex = %d\n", myIndex);
  }
}
