#include <Adafruit_ZeroTimer.h>
#include <Wire.h>
#include <BH1750.h>

#define BUTTON_PIN 2
#define TRIG_PIN 4
#define ECHO_PIN 5
#define LED_BUTTON 6
#define LED_ULTRA 7
#define LED_LIGHT 8

BH1750 lightMeter;
uint8_t bh1750Address = 0x23;  

Adafruit_ZeroTimer zt3 = Adafruit_ZeroTimer(3); 
volatile bool tickFlag = false;

volatile bool led1State = false;

void buttonISR() {
  led1State = !led1State;  
  Serial.print("Button pressed → LED1 ");
  Serial.println(led1State ? "ON" : "OFF");
}

void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3);
}

void myCallback() {
  tickFlag = true;
}

long readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long distanceCM = duration * 0.034 / 2;
  return distanceCM;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(500);

  Serial.println("System initialized...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUTTON, OUTPUT);
  pinMode(LED_ULTRA, OUTPUT);
  pinMode(LED_LIGHT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);  

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  zt3.configure(TC_CLOCK_PRESCALER_DIV1024,   
                TC_COUNTER_SIZE_16BIT,
                TC_WAVE_GENERATION_MATCH_FREQ);
  zt3.setCompare(0, 46875);  
  zt3.setCallback(true, TC_CALLBACK_CC_CHANNEL0, myCallback);
  zt3.enable(true);

  Wire.begin();
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
    bh1750Address = 0x23;
    Serial.println("BH1750 detected at 0x23");
  } else if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire)) {
    bh1750Address = 0x5C;
    Serial.println("BH1750 detected at 0x5C");
  } else {
    Serial.println("BH1750 sensor NOT found!");
  }
}

void loop() {

  digitalWrite(LED_BUTTON, led1State);

  long distance = readUltrasonicCM();
  if (distance > 0 && distance < 10) { 
    digitalWrite(LED_ULTRA, HIGH);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm | LED2: ON");
  } else {
    digitalWrite(LED_ULTRA, LOW);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm | LED2: OFF");
  }

  if (tickFlag) {
    tickFlag = false;

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.println("Timer tick → Toggled built-in LED");

    if (bh1750Address == 0x23 || bh1750Address == 0x5C) {
      float lux = lightMeter.readLightLevel();
      if (lux >= 0) {
        if (lux > 200) {  
          digitalWrite(LED_LIGHT, HIGH);
          Serial.print("Light: ");
          Serial.print(lux);
          Serial.println(" lux | LED3: ON");
        } else {
          digitalWrite(LED_LIGHT, LOW);
          Serial.print("Light: ");
          Serial.print(lux);
          Serial.println(" lux | LED3: OFF");
        }
      } else {
        Serial.println("BH1750 read failed!");
      }
    }
  }

  delay(2000); 
}
