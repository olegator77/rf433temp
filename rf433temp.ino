#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "Arduino.h"

const char* ssid = "gp_home";
const char* password = "12345ABCDE";

ESP8266WebServer server(80);

typedef struct {
  byte ID : 4;
  byte rollingID : 8;
  bool battery : 1;
  bool button : 1;
  byte channel : 2;
  float temp;
  float humidity;
} PrologueData;

class PrologueDecoder {
public:
  PrologueDecoder();
  bool pulse(word width, bool high);
  PrologueData getData();

private:
  void decodeRawData();
  void reset();
  byte state;
  byte rawData[5], i;
  PrologueData data;
};

/*
  The prologue protocol consists of 9 nibbles:
    0           1           2           3           4
  7654 3210   7654 3210   7654 3210   7654 3210   7654
  IIII RRRR   RRRR bBCC   TTTT TTTT   TTTT HHHH   HHHH
  I - ID
  R - Rolling ID
  b - battery
  B - Button
  C - channel
  T - Temp
  H - Humidity
*/

#define PROTOCOL_SIZE 36
#define setBit(byteArray, index) ((byteArray)[(index) / 8] |= (1 << (7 - (index) % 8)))

enum { IDLE, SYNCED };
const char SYNC_KEY[] = {1, 0, 0, 1, 1, 0, 0, 1};

PrologueDecoder::PrologueDecoder() { reset(); }

bool PrologueDecoder::pulse(word width, bool high) {
  if (high) {
    if (width < 330 && width > 530) {
      state = IDLE;
    }
    return false;
  }

  switch (state) {
    case IDLE:

      if (width > 7500 && width < 10000) {
        reset();
        state = SYNCED;
      }
      break;

    case SYNCED:

      // Zero bit
      if (width > 1500 && width < 2500) {
        Serial.print(0);
        i++;
      }
      // One bit
      else if (width > 3500 && width < 4500) {
        Serial.print(1);
        setBit(rawData, i);
        i++;
      }
      // Corrupt
      else {
        state = IDLE;
      }

      if (i == PROTOCOL_SIZE) {
        decodeRawData();

        return true;
      }
      break;
  }

  return false;
}

PrologueData PrologueDecoder::getData() { return data; }

void PrologueDecoder::decodeRawData() {
  data.ID = (rawData[0] & 0xF0) >> 4;
  data.rollingID = ((rawData[0] & 0x0F) << 4) | ((rawData[1] & 0xF0) >> 4);
  data.battery = (rawData[1] & 0x08);
  data.button = (rawData[1] & 0x04);
  data.channel = (rawData[1] & 0x03) + 1;
  data.temp = (((int16_t)(((uint16_t)rawData[2] << 8) | (rawData[3] & 0xF0))) / 16) * 0.1;
  data.humidity = (((int16_t)(((uint16_t)(rawData[3] & 0xF) << 8) | (rawData[4] & 0xF0))) / 16);
}

void PrologueDecoder::reset() {
  memset(rawData, 0, sizeof(rawData));
  state = IDLE;
  i = 0;
}

PrologueDecoder dec;

#define ledRF 2
#define ledGreen 12
#define ledBlue 13
#define ledRed 15

long lastTs = 0;
bool lastState = false;
long lastDataMs = 0;

int getPulse(int pin, bool& state) {
  int lastEdge = micros();
  long ts = micros();
  for (;;) {
    state = digitalRead(pin);
    ts = micros();
    if (state != lastState) {
      if (ts - lastEdge > 30) {
        break;
      }
      lastEdge = ts;
    }
    if (ts - lastTs > 20000) {
      break;
    }
  }
  int ret = ts - lastTs;
  lastTs = ts;
  lastState = state;
  return ret;
}

void handleTemp() {
  digitalWrite (ledBlue,HIGH);
  
  auto data = dec.getData();
  auto str =
    String("{\"temperature\":") + data.temp + ",\"humidity\":" + data.humidity + ",\"elapsed\":" + (millis() - lastDataMs) / 1000 + "}";
  server.send(200, "application/json", str);
  digitalWrite (ledBlue,LOW);

}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("");
  // 12 green, 13 blue, 15 red
  pinMode (ledGreen,OUTPUT);
  pinMode (ledBlue,OUTPUT);
  pinMode (ledRed,OUTPUT);
  analogWrite (ledGreen,20);
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(ledRF, INPUT);
  server.on("/temp.json", handleTemp);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  bool state = false;
  int width = getPulse(ledRF,state);
  if (dec.pulse(width, !state)) {
    digitalWrite (ledRed,HIGH);
    auto data = dec.getData();
    Serial.println(String("channel=") + data.channel + " T=" + data.temp + " H=" + data.humidity + " Battery=" + data.battery);
    lastDataMs = millis();
    delay (100);
    digitalWrite (ledRed,LOW);

  }
  server.handleClient();
//  Serial.println (analogRead (0));
}

