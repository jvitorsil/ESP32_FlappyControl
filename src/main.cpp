#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define FLEX_PIN 32
#define POT_PIN 34
#define LED_PIN 2

#define SSID "iPhone"
#define PASSWORD "senhasenha"
#define HOST "172.20.10.5"
#define PORT 1665

hw_timer_t *setTimer = NULL;

uint16_t setFreq = 1;
uint16_t saveFreq = 1;

uint8_t preScaler = 80;

uint16_t potValue;
uint16_t flexValue;

uint32_t timerFrequency = 80000000 / preScaler;
uint32_t timerPeriod = timerFrequency / setFreq;

WiFiUDP udp;

bool sendFlag = false;
const int bufferSize = 700;
char incomingPacket[bufferSize];

int packetCount = 0;


volatile unsigned long previousMicros = 0;
volatile unsigned long timerF = 0;

void sendToPython(const char *s);
void IRAM_ATTR onTimer();

void setup() {
  
  Serial.begin(115200);

  pinMode(FLEX_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  WiFi.begin(SSID, PASSWORD);

  Serial.println("Iniciando conexao na rede");

  while (WiFi.status() != WL_CONNECTED);
    delay(1000);

  Serial.println("Conectado a rede...");

  setTimer = timerBegin(0, preScaler, true);
  
  setCpuFrequencyMhz(80);

  timerAttachInterrupt(setTimer, &onTimer, true);
  timerAlarmWrite(setTimer, timerPeriod, true);
  timerAlarmEnable(setTimer);

  udp.begin(PORT); 
}


void loop() {

  potValue = analogRead(POT_PIN);
  setFreq = map(potValue, 0, 4095, 1, 550);

  timerPeriod = timerFrequency / setFreq;

  timerAlarmWrite(setTimer, timerPeriod, true);
  timerAlarmEnable(setTimer);
  
  // flexValue = analogRead(FLEX_PIN);
  flexValue = map(analogRead(FLEX_PIN), 0, 4095, 0, 550);

    // Serial.print(">f:");
    // Serial.println(flexValue);
    
    // Serial.print(">Freq:");
    // Serial.println(setFreq);


  if (sendFlag) {

    int charsWritten = snprintf(incomingPacket + strlen(incomingPacket), bufferSize - strlen(incomingPacket), "%s%u;%u", (packetCount == 0 ? "" : "/"), flexValue, setFreq);

    if (charsWritten >= 0 && charsWritten < bufferSize - strlen(incomingPacket))
      packetCount++;

    if(setFreq >= 70)
      saveFreq = 70;
    else
      saveFreq = setFreq;

    // Serial.print(">flexValue:");
    // Serial.println(flexValue);
    
    // Serial.print(">Freq:");
    // Serial.println(setFreq);

    if (packetCount >= saveFreq) {
      // unsigned long currentMicros = micros();
      // timerF = 1000000 / (currentMicros - previousMicros);
      // previousMicros = currentMicros;
      Serial.printf("FrequênciaEstimada: %u || ", timerF);
      Serial.printf("FrequênciaDefinida: %u || ", setFreq);
      Serial.printf("PacketCount: %i || ", packetCount);
      Serial.printf(incomingPacket);
      Serial.println(" ");

      sendToPython(incomingPacket);
      incomingPacket[0] = '\0';
      packetCount = 0;
    
    }
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