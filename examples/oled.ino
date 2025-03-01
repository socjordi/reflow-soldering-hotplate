#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 2, /* data=*/ 3);

void setup(void) {
    u8g2.begin();
}

void loop(void) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_logisoso16_tr);
    u8g2.drawStr(5,15,"Hello World");
    u8g2.sendBuffer();
    delay(1000);  
}
