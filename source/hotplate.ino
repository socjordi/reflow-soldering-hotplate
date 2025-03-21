#include "USB.h"
#include <WiFi.h>
#include <WebServer.h>
//#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <MAX6675.h>
#include <PID_v1.h>

const char* ssid = "SIBUR";  // o SIBUR_5G
const char* password = "tsupufeuf1969";
const char* jsonFileName = "/config.json";
const char* serverUrl = "http://192.168.1.13/config.json";

IPAddress staticIP(192, 168, 1, 13);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(1, 1, 1, 1);

WebServer server(80);


U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 2, /* data=*/ 3);

const int temp_dataPin = 5;
const int temp_clockPin = 4;
const int temp_selectPin = 7;
MAX6675 thermoCouple(temp_selectPin, temp_dataPin, temp_clockPin);

#define SSR_PIN 10

#define FAN_PIN 21

double currentTemp, targetTemp, output;
double Kp = 30, Ki = 0.2, Kd = 10; // PID tuning parameters
PID myPID(&currentTemp, &output, &targetTemp, Kp, Ki, Kd, DIRECT);

const int button_Select_Pin = 9;
const int button_Start_Pin = 8;

int currentProfile = 0;
int currentStage = -1;

int elapsedTime;

unsigned long totalstartTime;
unsigned long startTime;
unsigned long lastPWMTime;
const int pwmPeriod = 1000; // 1-second PWM period
bool SSRState = false;

char msg[64];

struct Profile {
  char name[32];
  struct {
    float temperature;
    int duration;  // seconds
  } stages[4];
};

int numProfiles = 0;
Profile profiles[5];

struct LogEntry {
  int elapsed;
  char stage;
  int current_temp;
  int target_temp;
  float output;
  char ssr;
};

LogEntry logs[360];
int numLogs = 0;

void parseJsonArray(String jsonString) {

  // Allocate memory for JSON parsing (adjust size if needed)
  StaticJsonDocument<512> doc;
  
  // Deserialize JSON
  DeserializationError error = deserializeJson(doc, jsonString);
  if (error) {
      Serial.print("JSON Parsing failed: ");
      Serial.println(error.f_str());
      return;
  }

  // Access JSON array
  JsonArray a_profiles = doc.as<JsonArray>();

  numProfiles = 0;

  // Iterate over array
  for (JsonArray profile : a_profiles) {

    sprintf(msg, "Profile #%d", numProfiles);
    Serial.println(msg);

    strncpy(profiles[numProfiles].name, profile[0].as<const char*>(), sizeof(profiles[numProfiles].name) - 1);
    profiles[numProfiles].name[sizeof(profiles[numProfiles].name) - 1] = '\0';

    sprintf(msg, "Name: %s", profiles[numProfiles].name);
    Serial.println(msg);

    for (int i=0; (i<4); i++) {

      JsonArray stage = profile[i + 1]; 

      profiles[numProfiles].stages[i].temperature = stage[0];
      profiles[numProfiles].stages[i].duration = stage[1];

      sprintf(msg, "%i %f %i",
        i,
        profiles[numProfiles].stages[i].temperature,
        profiles[numProfiles].stages[i].duration
        );
      Serial.println(msg);
    }

    numProfiles++;
  }
}

String generateCSV() {
  String csv = "Elapsed,Stage,Current Temp,Target Temp,Output,SSR\n"; // CSV Header
  for (int i = 0; i < numLogs; i++) {
    csv += String(logs[i].elapsed) + "," +
            String(logs[i].stage) + "," +
            String(logs[i].current_temp) + "," +
            String(logs[i].target_temp) + "," +
            String(logs[i].output, 2) + "," +
            String(logs[i].ssr) + "\n";
  }
  return csv;
}

void handleLogsGet() {
  server.sendHeader("Content-Disposition", "attachment; filename=logs.csv");
  server.send(200, "text/csv", generateCSV());
}

void handleProfilesGet() {
  String jsonString = readFile(jsonFileName);
  if (jsonString.length() > 0) {
    server.send(200, "application/json", jsonString);
  } else {
    server.send(404, "text/plain", "Profiles not found");
  }
}

void handleProfilesPut() {

  if (server.hasArg("plain") == false) {
    server.send(400, "text/plain", "Missing JSON payload");
    return;
  }

  String jsonString = server.arg("plain");

  StaticJsonDocument<512> doc; // Adjust size as needed
  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  writeFile(jsonFileName, jsonString.c_str());
  server.send(200, "text/plain", "Profiles updated successfully");
}

void writeFile(const char* path, const char* message) {
  Serial.printf("Writing file: %s\r\n", path);

  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

String readFile(const char* path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = SPIFFS.open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    Serial.println("Failed to open file for reading");
    return String();
  }

  String fileContent;
  while (file.available()) {
    fileContent += String((char)file.read());
  }
  file.close();

  return fileContent;
}

void setup(void) {

  Serial.begin(115200);
  
  pinMode(button_Select_Pin, INPUT_PULLUP);
  pinMode(button_Start_Pin, INPUT_PULLUP);

  u8g2.begin();
  //u8g2.setFont(u8g2_font_logisoso16_tr);
  u8g2.setFont(u8g2_font_t0_12_tf);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, HIGH);
  delay(5000);
  digitalWrite(FAN_PIN, LOW);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed, formatting...");
    SPIFFS.format();
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS Mount Failed again!");
      return;
    }
  }

  WiFi.config(staticIP, gateway, subnet, dns);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.print("Static IP: ");
  Serial.println(WiFi.localIP());
  u8g2.clearBuffer();

  IPAddress localIP = WiFi.localIP();
  snprintf(msg, sizeof(msg), "%u.%u.%u.%u", localIP[0], localIP[1], localIP[2], localIP[3]);
  u8g2.drawUTF8(5, 20, msg);

  server.on("/profiles", HTTP_GET, handleProfilesGet);
  server.on("/profiles", HTTP_PUT, handleProfilesPut);
  server.on("/logs", HTTP_GET, handleLogsGet);
  server.begin();
  Serial.println("HTTP server started");

  String jsonString = readFile(jsonFileName);
  parseJsonArray(jsonString);

  Serial.print("Free heap memory: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
}

void loop(void) {

  int buttonState;

  server.handleClient();

  u8g2.clearBuffer();

  int status = thermoCouple.read();
  currentTemp = thermoCouple.getTemperature();
  //thermoCouple.setOffset(-10.0);
  if (status == 0) {
    sprintf(msg, "Temp: %d°C", int(currentTemp));
    u8g2.drawUTF8(5, 10, msg);
    if (currentStage > 0) {
      sprintf(msg, "%d %d", currentStage, elapsedTime);
      u8g2.drawStr(5, 20, msg);
    }
  }
  else if (status == 4) {
    u8g2.drawStr(5, 15, "Check thermopar");
  }
  else if (status == 129) {
    u8g2.drawStr(5, 15, "No comms");
  }

  buttonState = digitalRead(button_Select_Pin);
  if (buttonState == LOW) {
    u8g2.drawStr(5, 15, "SELECT");
  }

  buttonState = digitalRead(button_Start_Pin);
  if (buttonState == LOW) {
    u8g2.drawStr(5, 15, "START");
    totalstartTime = millis();
    startTime = millis();
    currentStage = 0;
  }

  u8g2.sendBuffer();

  Profile profile = profiles[currentProfile];

  if (currentStage >= 0) {

    if (currentStage < 4) {
      if (elapsedTime > profile.stages[currentStage].duration) {
          currentStage++;
          startTime = millis();
      }
      targetTemp = profile.stages[currentStage].temperature;
    }

    myPID.Compute();

    unsigned long now = millis();
    int ssr;
    if (now - lastPWMTime >= pwmPeriod) {
      lastPWMTime = now;
      if (output > 50) {
          ssr = HIGH;     // More than 50% duty cycle → ON
          //digitalWrite(FAN_PIN, LOW);
      } else {
          ssr = LOW;      // Less than 50% duty cycle → OFF
          //digitalWrite(FAN_PIN, HIGH);
      }
      digitalWrite(SSR_PIN, ssr);
     }

    if (currentStage == 3) digitalWrite(FAN_PIN, HIGH);

    unsigned long totalTime = (millis() - totalstartTime) / 1000;

    Serial.print("Total: "); Serial.print(totalTime);
    Serial.print(" | Elapsed: "); Serial.print(elapsedTime);
    Serial.print(" | Stage: "); Serial.print(currentStage);
    Serial.print(" | Temp: "); Serial.print(currentTemp);
    Serial.print(" | Target: "); Serial.print(targetTemp);
    Serial.print(" | Output: "); Serial.print(output);
    Serial.print(" | SSR: "); Serial.println(ssr);

    logs[numLogs].elapsed = totalTime;
    logs[numLogs].stage = currentStage;
    logs[numLogs].current_temp = currentTemp;
    logs[numLogs].target_temp = targetTemp;
    logs[numLogs].output = output;
    logs[numLogs].ssr = ssr;

    numLogs++;
  }

  delay(500);
}
