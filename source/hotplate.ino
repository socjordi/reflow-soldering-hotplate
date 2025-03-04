#include <U8g2lib.h>
#include <Wire.h>
#include <MAX6675.h>
#include <PID_v1.h>

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 2, /* data=*/ 3);

const int temp_dataPin = 5;
const int temp_clockPin = 4;
const int temp_selectPin = 7;
MAX6675 thermoCouple(temp_selectPin, temp_dataPin, temp_clockPin);

#define SSR_PIN 10

double currentTemp, targetTemp, output;
double Kp = 30, Ki = 0.2, Kd = 10; // PID tuning parameters
PID myPID(&currentTemp, &output, &targetTemp, Kp, Ki, Kd, DIRECT);

const int button_Select_Pin = 9;
const int button_Start_Pin = 8;

struct ReflowStage {
    float temperature;
    int duration;  // seconds
};
ReflowStage profile[] = {
    {100, 80},  // Preheat
    {120, 90},  // Soak
    {160, 70},  // Reflow
    {20, 360}   // Cooling
};
const int numStages = sizeof(profile) / sizeof(profile[0]);
int currentStage = -1;

unsigned long startTime;
unsigned long lastPWMTime;
const int pwmPeriod = 1000; // 1-second PWM period
bool SSRState = false;

char msg[64];

void setup(void) {

  Serial.begin(115200);
  
  pinMode(button_Select_Pin, INPUT);
  pinMode(button_Start_Pin, INPUT);

  u8g2.begin();
  //u8g2.setFont(u8g2_font_logisoso16_tr);
  u8g2.setFont(u8g2_font_t0_12_tf);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  startTime = millis();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
}

void loop(void) {

  unsigned long elapsedTime = (millis() - startTime) / 1000;

  u8g2.clearBuffer();

  int status = thermoCouple.read();
  currentTemp = thermoCouple.getTemperature();
  //thermoCouple.setOffset(-10.0);
  if (status == 0) {
    sprintf(msg, "Temp: %d°C", int(currentTemp));
    u8g2.drawUTF8(5, 10, msg);
    sprintf(msg, "%d %d", currentStage, elapsedTime);
    u8g2.drawStr(5, 20, msg);
  }
  else if (status == 4) {
    u8g2.drawStr(5, 15, "Check thermopar");
  }

  else if (status == 129) {
    u8g2.drawStr(5, 15, "No comms");
  }

  /*
  int buttonState = digitalRead(button_Select_Pin);
  if (buttonState == LOW) {
    u8g2.drawStr(5, 15, "SELECT");
  }
  */

  int buttonState = digitalRead(button_Start_Pin);
  if (buttonState == LOW) {
    u8g2.drawStr(5, 15, "START");
    startTime = millis();
    currentStage = 0;
  }

  u8g2.sendBuffer();

  if (currentStage >= 0) {

    if (currentStage < numStages) {
      if (elapsedTime > profile[currentStage].duration) {
          currentStage++;
          startTime = millis();
      }
      targetTemp = profile[currentStage].temperature;
    }

    myPID.Compute();

    unsigned long now = millis();
    if (now - lastPWMTime >= pwmPeriod) {
      lastPWMTime = now;
      if (output > 50) {
          digitalWrite(SSR_PIN, HIGH);  // More than 50% duty cycle → ON
      } else {
          digitalWrite(SSR_PIN, LOW);   // Less than 50% duty cycle → OFF
      }
    }

    Serial.print("Elapsed: "); Serial.print(elapsedTime);
    Serial.print(" | Stage: "); Serial.print(currentStage);
    Serial.print(" | Temp: "); Serial.print(currentTemp);
    Serial.print(" | Target: "); Serial.print(targetTemp);
    Serial.print(" | Output: "); Serial.print(output);
    Serial.print(" | SSR: "); Serial.println(digitalRead(SSR_PIN));
  }

  delay(500);
}
