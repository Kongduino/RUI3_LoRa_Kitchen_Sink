#ifdef __RAKBLE_H__
void sendToBle(char *msgToSend) {
  api.ble.uart.write((uint8_t*)msgToSend, strlen(msgToSend));
}
#endif

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
