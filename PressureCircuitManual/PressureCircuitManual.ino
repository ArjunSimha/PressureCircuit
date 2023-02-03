// include libraries
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include <Adafruit_NeoPixel.h>

// pressure sensor setup
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// NeoPixel setup
#define LED_PIN 6 // pin connected to the NeoPixels
#define LED_COUNT 24 // number of NeoPixels
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// relay connection/signal pins
#define IN1 44
#define IN2 42
#define IN3 40
#define IN4 38
#define IN5 46
#define IN6 48
#define IN7 50
#define IN8 52

// Switch connection pins
#define SW1 43
#define SW2 41
#define SW3 39
#define SW4 37

// declare colors
uint32_t purple = strip.Color(191, 64, 191);

void setup() {
  // put your setup code here, to run once:
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(64); // Set BRIGHTNESS to about 1/4 (max = 255)

  Serial.begin(115200);
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");
  
  // initialize relay pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // initialize Switch pins
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  pinMode(SW4, INPUT);

  // turn off relay pins by default
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, HIGH);
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, HIGH);
}

void loop() {
  int prev; // temp var
  float pressure_hPa = mpr.readPressure();
  float pressure_PSI = pressure_hPa / 68.947572932;
  int pressureMapped = map(pressure_PSI, 0, 14, 0, 23); // map PSI pressure from 0 (absolute vaccuum) to 14 (atmospheric) to 0 to 23 (for the 24 LEDs)
  Serial.println(pressure_PSI); // for serial plotter

  // if pressure changes, change LED
  if (pressureMapped != prev) {
    strip.clear();
    strip.setPixelColor(pressureMapped, purple);
    strip.show();
  }
  prev = pressureMapped;

  if (digitalRead(SW1)) {
    digitalWrite(IN1, HIGH);
    // Serial.println("Solenoid 1 on");
  } else {
    digitalWrite(IN1, LOW);
    // Serial.println("Solenoid 1 off");
  }

  if (digitalRead(SW2)) {
    digitalWrite(IN2, HIGH);
    // Serial.println("Solenoid 2 on");
  } else {
    digitalWrite(IN2, LOW);
    // Serial.println("Solenoid 2 off");
  }

  if (digitalRead(SW3)) {
    digitalWrite(IN3, HIGH);
    // Serial.println("Solenoid 3 on");
  } else {
    digitalWrite(IN3, LOW);
    // Serial.println("Solenoid 3 off");
  }

  if (digitalRead(SW4)) {
    digitalWrite(IN4, HIGH);
    // Serial.println("Solenoid 4 on");
  } else {
    digitalWrite(IN4, LOW);
    //  Serial.println("Solenoid 4 off");
  }
}