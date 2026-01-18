#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <WiFiClientSecure.h>

// WiFi 
const char* ssid = "redmagic";
const char* password = "12345678";

// MQTT 
const char* mqtt_server = "a2036f82afb849f78974fc64b55cb465.s1.eu.hivemq.cloud";
const int   mqtt_port   = 8883;

const char* mqtt_user = "group4";
const char* mqtt_pass = "Group123456";

const char* TOPIC_SENSORS = "group4/iot/sensors";
const char* TOPIC_LIGHT   = "group4/iot/light";

StaticJsonDocument<256> doc;

WiFiClientSecure net;
PubSubClient client(net);

// Relay 
const int RELAY_PIN = 25;
int RELAY_state = 0;

// Servo 
const int SERVO_PIN    = 13;
const int OPEN_ANGLE   = 90;
const int CLOSE_ANGLE  = 0;
const unsigned long closeDelayMs = 500;

Servo gate;
bool gateOpen = false;
unsigned long clearSince = 0;

void servoInit() {
  gate.setPeriodHertz(50);
  gate.attach(SERVO_PIN, 500, 2400);
  gate.write(CLOSE_ANGLE);
  gateOpen = false;
  clearSince = 0;
}
void openGate()  { gate.write(OPEN_ANGLE);  gateOpen = true;  }
void closeGate() { gate.write(CLOSE_ANGLE); gateOpen = false; }


void updateGateAuto(bool carPresent) {
  if (carPresent) {
    if (!gateOpen) openGate();
    clearSince = 0;
  } else if (gateOpen) {
    if (clearSince == 0) clearSince = millis();
    else if (millis() - clearSince >= closeDelayMs) {
      closeGate();
      clearSince = 0;
    }
  }
}

//  Ultrasonic/Servo params 
const int THRESH_CM = 10;
const unsigned long PULSE_TO = 30000UL;
const unsigned long XTLK_DELAY_MS = 40;

// Ultrasonic 
const int trigA = 27, echoA = 34;
const int trigB = 26, echoB = 39;

// IR Sensors 
const int ir1Pin = 32;
const int ir2Pin = 33;

//  Flame Sensors
const int FLAME_DO_PIN = 4;
int  flameState     = 0;
bool flameDetected  = false;
unsigned long lastChange = 0;
const unsigned long debounceMs = 50;

// Buzzer
const int BUZZER_PIN = 5;

// RGB 
const int IR1_R = 21, IR1_G = 22;
const int IR2_R = 19, IR2_G = 18;

void rgbInit() {
  pinMode(IR1_R, OUTPUT);
  pinMode(IR1_G, OUTPUT);
  pinMode(IR2_R, OUTPUT);
  pinMode(IR2_G, OUTPUT);
  digitalWrite(IR1_R, LOW);
  digitalWrite(IR1_G, LOW);
  digitalWrite(IR2_R, LOW);
  digitalWrite(IR2_G, LOW);
}

inline void setRG(int pinR, int pinG, bool redOn, bool greenOn) {
  digitalWrite(pinR, redOn   ? HIGH : LOW);
  digitalWrite(pinG, greenOn ? HIGH : LOW);
}

// MQTT timing
unsigned long lastPublish=0;
const unsigned long publishInterval=1000;

// RELAY
void MqttMessage(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if (msg == "1") {
    digitalWrite(RELAY_PIN, HIGH);
    RELAY_state = 1;
  } else if (msg == "0") {
    digitalWrite(RELAY_PIN, LOW);
    RELAY_state = 0;
  }
}

// WiFi 
void setup_wifi(){
  WiFi.begin(ssid, password);
  while(WiFi.status()!=WL_CONNECTED){ delay(500); Serial.print("."); }
  Serial.println("WiFi connected");
}

// MQTT Reconnect 
void reconnect(){
  while(!client.connected()){
    if (client.connect("ESP32GateJSON", mqtt_user, mqtt_pass)) {
      client.subscribe(TOPIC_LIGHT);
      client.setCallback(MqttMessage);
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT failed, rc="); Serial.println(client.state());
      delay(2000);
    }
  }
}

// Ultrasonic Function 
long read_ultrasonic_cm(int trigPin, int echoPin){
  digitalWrite(trigPin,LOW); delayMicroseconds(2);
  digitalWrite(trigPin,HIGH); delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  unsigned long dur = pulseIn(echoPin, HIGH, PULSE_TO);
  if (dur == 0) return -1;
  return (long)(dur * 0.034f / 2.0f);
}

inline bool isNear(long cm) { return (cm >= 0) && (cm <= THRESH_CM); }

// log
int last_entry = -1, last_exit = -1;
int last_ir1   = -1, last_ir2  = -1;
int last_flame = -1;
int last_light = -1;


volatile int entry_count=0, exit_count=0;

void maybeLog(int entryVal, int exitVal, int ir1Val, int ir2Val, int flameVal, int relayVal) {
  if (entryVal != last_entry || exitVal != last_exit ||
      ir1Val   != last_ir1   || ir2Val  != last_ir2  ||
      flameVal != last_flame || relayVal != last_light) {

    Serial.printf("entry=%d  exit=%d  IR1=%d  IR2=%d  flame=%d  light=%d\n",
                  entryVal, exitVal, ir1Val, ir2Val, flameVal, relayVal);

    last_entry = entryVal;  last_exit = exitVal;
    last_ir1   = ir1Val;    last_ir2  = ir2Val;
    last_flame = flameVal;
    last_light = relayVal;
  }
}

enum PassState { IDLE, SAW_A, SAW_B, WAIT_CLEAR };
PassState passState = IDLE;
unsigned long stateSince = 0;
const unsigned long PASS_WINDOW_MS = 3000;

void resetToIdle() { passState = IDLE; stateSince = millis(); }

void setup() {
  Serial.begin(115200);

  pinMode(trigA,OUTPUT); pinMode(echoA,INPUT);
  pinMode(trigB,OUTPUT); pinMode(echoB,INPUT);

  pinMode(ir1Pin, INPUT_PULLUP);
  pinMode(ir2Pin, INPUT_PULLUP);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(FLAME_DO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  servoInit();
  rgbInit();

  setup_wifi();
  net.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(MqttMessage);

  resetToIdle();
}

void loop() {
  if(!client.connected()) reconnect();
  client.loop();

  // Ultrasonic (เว้นจังหวะกันครอสทอล์ค)
  long dA = read_ultrasonic_cm(trigA, echoA);
  delay(XTLK_DELAY_MS);
  long dB = read_ultrasonic_cm(trigB, echoB);

  bool nearA = isNear(dA);
  bool nearB = isNear(dB);

  // เปิด/ปิดประตูอัตโนมัติจากการพบรถที่ใดก็ได้ (รวม IR)
  int ir1Raw = digitalRead(ir1Pin);
  int ir2Raw = digitalRead(ir2Pin);
  int ir1 = (ir1Raw==LOW)?1:0;
  int ir2 = (ir2Raw==LOW)?1:0;

  bool carPresent = (nearA || nearB);
  updateGateAuto(carPresent);

  // State machine สำหรับการนับ
  unsigned long nowMs = millis();
  if (passState == IDLE) {
    if (nearA) { passState = SAW_A; stateSince = nowMs; }
    else if (nearB) { passState = SAW_B; stateSince = nowMs; }
  }
  else if (passState == SAW_A) {
    if (nearB) {
      entry_count++;
      passState = WAIT_CLEAR;
    } else if (nowMs - stateSince > PASS_WINDOW_MS) {
      resetToIdle();
    }
  }
  else if (passState == SAW_B) {
    if (nearA) { 
      exit_count++;
      passState = WAIT_CLEAR;
    } else if (nowMs - stateSince > PASS_WINDOW_MS) {
      resetToIdle();
    }
  }
  else if (passState == WAIT_CLEAR) { 
    if (!nearA && !nearB) resetToIdle();
  }

  //RGB สถานะ IR
  bool ir1High = (ir1Raw == HIGH);
  bool ir2High = (ir2Raw == HIGH);
  setRG(IR1_R, IR1_G, /*red*/ !ir1High, /*green*/ ir1High);
  setRG(IR2_R, IR2_G, /*red*/ !ir2High, /*green*/ ir2High);


  //Flame
  bool raw = digitalRead(FLAME_DO_PIN);
  bool readingDetected = (raw == LOW);
  static bool lastReading = false;
  if (readingDetected != lastReading) { lastChange = millis(); lastReading = readingDetected; }
  if (millis() - lastChange >= debounceMs) {
    if (flameDetected != readingDetected) {
      flameDetected = readingDetected;
    }
  }
  digitalWrite(BUZZER_PIN, flameDetected ? LOW : HIGH);
  flameState = flameDetected ? 1 : 0;

  //Publish JSON
  unsigned long now = millis();
  if(now - lastPublish > publishInterval){
    lastPublish = now;

    doc.clear();
    JsonObject ultra = doc.createNestedObject("ultra");
    ultra["entry"] = entry_count;
    ultra["exit"]  = exit_count;

    JsonArray ir = doc.createNestedArray("ir");
    ir.add(ir1);
    ir.add(ir2);

    doc["fire"]  = flameState;

    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    client.publish(TOPIC_SENSORS, buffer, n);
  }

  maybeLog(entry_count, exit_count, ir1, ir2, flameState, RELAY_state);
  delay(50);
}
