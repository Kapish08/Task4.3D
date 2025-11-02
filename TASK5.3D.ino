#include <Adafruit_ZeroTimer.h>  // For using hardware timer on SAMD21
#include <Wire.h>                // For I2C communication
#include <BH1750.h>              // For BH1750 light intensity sensor

// Pin definitions
#define BUTTON_PIN 2      // Push button input pin
#define TRIG_PIN 4        // Ultrasonic trigger pin
#define ECHO_PIN 5        // Ultrasonic echo pin (via voltage divider)
#define LED_BUTTON 6      // LED controlled by button
#define LED_ULTRA 7       // LED indicates object proximity
#define LED_LIGHT 8       // LED indicates brightness

// Global objects and variables
BH1750 lightMeter;                    // Create BH1750 sensor object
uint8_t bh1750Address = 0x23;         // Default I2C address of BH1750

Adafruit_ZeroTimer zt3 = Adafruit_ZeroTimer(3);  // Use Timer Counter 3
volatile bool tickFlag = false;                   // Flag for timer event
volatile bool led1State = false;                  // State of button LED

// ISR for button press
void buttonISR() {
  led1State = !led1State;

  Serial.print("Button pressed → LED1 ");
  Serial.println(led1State ? "ON" : "OFF");
}

// Timer interrupt handler (TC3)
void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3);  // Handle TC3 interrupt
}

// Timer callback function
void myCallback() {
  tickFlag = true;  // Set flag when timer event occurs
}

// Function to measure distance using HC-SR04
long readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);      // Ensure trigger is LOW
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);     // Send 10 µs HIGH pulse
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure time for echo pulse to return (max 30ms)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  // Convert time to distance in centimeters
  long distanceCM = duration * 0.034 / 2;
  return distanceCM;
}

void setup() {
  Serial.begin(115200);      // Start Serial communication
  while (!Serial);           // Wait for serial monitor to open
  delay(500);

  Serial.println("System initialized...");

  // Configure pin modes
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Button input with pull-up
  pinMode(TRIG_PIN, OUTPUT);          // Trigger pin as output
  pinMode(ECHO_PIN, INPUT);           // Echo pin as input
  pinMode(LED_BUTTON, OUTPUT);        // LED1 (button)
  pinMode(LED_ULTRA, OUTPUT);         // LED2 (ultrasonic)
  pinMode(LED_LIGHT, OUTPUT);         // LED3 (light sensor)
  pinMode(LED_BUILTIN, OUTPUT);       // Built-in LED (timer activity)

  // Attach interrupt to button (FALLING = pressed)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  zt3.configure(
    TC_CLOCK_PRESCALER_DIV1024,    // Divide system clock for slower tick
    TC_COUNTER_SIZE_16BIT,         // 16-bit counter
    TC_WAVE_GENERATION_MATCH_FREQ  // Match frequency mode
  );

  zt3.setCompare(0, 46875);        // ≈1 second tick interval
  zt3.setCallback(true, TC_CALLBACK_CC_CHANNEL0, myCallback);
  zt3.enable(true);                // Start timer

  Wire.begin();  // Start I2C communication

  // Try address 0x23 first
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
    bh1750Address = 0x23;
    Serial.println("BH1750 detected at 0x23");
  }
  // If not found, try 0x5C
  else if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire)) {
    bh1750Address = 0x5C;
    Serial.println("BH1750 detected at 0x5C");
  }
  // If not found at all
  else {
    Serial.println("BH1750 sensor NOT found!");
  }
}

void loop() {

  digitalWrite(LED_BUTTON, led1State);  // Reflect button state

  long distance = readUltrasonicCM();   // Measure distance in cm

  if (distance > 0 && distance < 10) {  // If object within 10 cm
    digitalWrite(LED_ULTRA, HIGH);      // Turn on LED2
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm | LED2: ON");
  } else {                              // Otherwise turn it off
    digitalWrite(LED_ULTRA, LOW);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm | LED2: OFF");
  }

  if (tickFlag) {
    tickFlag = false;  // Reset timer flag

    // Toggle built-in LED each tick
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.println("Timer tick → Toggled built-in LED");

    if (bh1750Address == 0x23 || bh1750Address == 0x5C) {
      float lux = lightMeter.readLightLevel();  // Get lux value

      if (lux >= 0) {   // Valid reading
        if (lux > 200) {                      // If bright
          digitalWrite(LED_LIGHT, HIGH);      // Turn ON LED3
          Serial.print("Light: ");
          Serial.print(lux);
          Serial.println(" lux | LED3: ON");
        } else {                              // If dim
          digitalWrite(LED_LIGHT, LOW);       // Turn OFF LED3
          Serial.print("Light: ");
          Serial.print(lux);
          Serial.println(" lux | LED3: OFF");
        }
      } else {
        Serial.println("BH1750 read failed!");
      }
    }
  }

  delay(2000);  // Delay for stability (2 seconds)
}
