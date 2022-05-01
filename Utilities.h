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
  switchOLED(true);
  memcpy(backup0, ucBuffer, 1024);
  oledFill(&oled, 0, 1);
  oledWriteString(&oled, 0, 10, 0, "BBQ10", FONT_16x16, 0, 1);
  oledWriteString(&oled, 0, 0, 2, ">", FONT_8x8, 0, 1);
  oledWriteString(&oled, 0, -1, 2, bbqBuff, FONT_8x8, 0, 1);
  lastKbdInput = millis();
  isBBQing = true;
}

void restoreScreen() {
  switchOLED(true);
  oledDumpBuffer(&oled, backup0);
  isBBQing = false;
}
