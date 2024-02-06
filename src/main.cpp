#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_MPU6050.h>

#define FLEX_PIN 32

#define POT_PIN 34

#define PUSH_PIN 35

#define LED_PIN 2

#define SDA_PIN 21
#define SCL_PIN 22

#define SSID "iPhone"
#define PASSWORD "senhasenha"
#define HOST "172.20.10.5"
#define PORT 1665

hw_timer_t *setTimer = NULL;

float_t pitch;

uint16_t setFreq = 1;
uint8_t preScaler = 160;

uint16_t potValue;
uint16_t flexValue;

uint32_t timerFrequency = 80000000 / preScaler;
uint32_t timerPeriod = timerFrequency / setFreq;

uint8_t buttonState;
uint8_t currentButtonState = HIGH;
uint8_t oldButtonState = HIGH;

WiFiUDP udp;

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;


bool sendFlag = false;
const int bufferSize = 17;
char incomingPacket[bufferSize];


void sendToPython(const char *s);
void IRAM_ATTR onTimer();

void setup() {
  
  Serial.begin(115200);
  pinMode(FLEX_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PUSH_PIN, INPUT_PULLUP);

  
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED);
    delay(1000);

  Serial.println("Conectado a rede...");

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin())
    while (1)
      delay(10);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  setTimer = timerBegin(0, preScaler, true);
  
  timerAttachInterrupt(setTimer, &onTimer, true);
  timerAlarmWrite(setTimer, timerPeriod, true);
  timerAlarmEnable(setTimer);

  udp.begin(PORT); 
}


void loop() {

  potValue = analogRead(POT_PIN);
  setFreq = map(potValue, 0, 4095, 1, 100);

  timerPeriod = timerFrequency / (2*setFreq);

  flexValue = analogRead(FLEX_PIN);

  mpu.getEvent(&a, &g, &temp);

  pitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;


  currentButtonState = digitalRead(PUSH_PIN);
  if(currentButtonState != oldButtonState && currentButtonState)
    buttonState = !buttonState;

  oldButtonState = currentButtonState;


  timerAlarmWrite(setTimer, timerPeriod, true);
  timerAlarmEnable(setTimer);
  
  if (sendFlag) {
    snprintf(incomingPacket, bufferSize, "%u;%u;%u;%f", flexValue, buttonState, setFreq, pitch);
    sendToPython(incomingPacket);
    sendFlag = false;
  }
}

void IRAM_ATTR onTimer(){
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));  
  sendFlag = true;
}

void sendToPython(const char *s) {
  udp.beginPacket(HOST, PORT);
  udp.print(s);
  udp.endPacket();
}