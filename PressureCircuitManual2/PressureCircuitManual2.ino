// include libraries
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include <Adafruit_NeoPixel.h>

// pressure sensor setup
#define RESET_PIN -1 // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN -1 // set to any GPIO pin to read end-of-conversion by pin

Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN); // declare pressure sensor object

// NeoPixel setup
#define LED_PIN 6 // pin connected to the NeoPixels
#define LED_COUNT 24 // number of NeoPixels

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); // Declare NeoPixel strip object

// Relay connection/signal pins
#define IN1 44
#define IN2 42
#define IN3 40
#define IN4 38
#define IN5 46
#define IN6 48
#define IN7 50
#define IN8 52
#define IN1_2 22
#define IN2_2 24
#define IN3_2 26
#define IN4_2 28
#define IN5_2 30
#define IN6_2 32
#define IN7_2 34
#define IN8_2 36

// Switch connection pins
#define SW1 43
#define SW2 41
#define SW3 39
#define SW4 37
#define SW5 33
#define SW6 31
#define SW7 29
#define SW8 27
#define SW9 25
#define SW10 23

uint32_t purple = strip.Color(191, 64, 191); // Declare LED color

int pressureState = 0;

void setup() {
  // put your setup code here, to run once:
  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show(); // Turn OFF all pixels ASAP
  strip.setBrightness(100); // Set BRIGHTNESS (max = 255)

  Serial.begin(115200);
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
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
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(IN3_2, OUTPUT);
  pinMode(IN4_2, OUTPUT);
  pinMode(IN5_2, OUTPUT);
  pinMode(IN6_2, OUTPUT);
  pinMode(IN7_2, OUTPUT);
  pinMode(IN8_2, OUTPUT);

  // initialize Switch pins
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  pinMode(SW4, INPUT);
  pinMode(SW5, INPUT);
  pinMode(SW6, INPUT);
  pinMode(SW7, INPUT);
  pinMode(SW8, INPUT);
  pinMode(SW9, INPUT);
  pinMode(SW10, INPUT);

  // turn off relay pins (HIGH = off) by default
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, HIGH);
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, HIGH);
  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, HIGH);
  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, HIGH);
  digitalWrite(IN5_2, HIGH);
  digitalWrite(IN6_2, HIGH);
  digitalWrite(IN7_2, HIGH);
  digitalWrite(IN8_2, HIGH);
}

void loop() {
  pressureUpdate();
  solenoidManual();
}

void pressureUpdate() {
  float pressure_hPa = mpr.readPressure();
  float pressure_PSI = pressure_hPa / 68.947572932;
  int pressureMapped = map(pressure_PSI, 0, 14, 0, 23); // map PSI pressure from 0 (absolute vaccuum) to 14 (atmospheric) to 0 to 23 (for the 24 LEDs)

  // if pressure changes, change LED
  if (pressureMapped != pressureState) {
    Serial.print("Pressure: ");
    Serial.println(pressure_PSI);
    strip.clear();
    strip.setPixelColor(pressureMapped, purple);
    strip.show();
  }

  pressureState = pressureMapped; // update state
}

void solenoidManual() {
  digitalWrite(IN1, digitalRead(SW1));

  digitalWrite(IN2, digitalRead(SW2));

  digitalWrite(IN3, digitalRead(SW3));

  digitalWrite(IN4, digitalRead(SW4));

  digitalWrite(IN6_2, digitalRead(SW5));

  digitalWrite(IN5_2, digitalRead(SW6));

  digitalWrite(IN4_2, digitalRead(SW7));

  digitalWrite(IN3_2, digitalRead(SW8));

  digitalWrite(IN2_2, digitalRead(SW9));

  digitalWrite(IN1_2, digitalRead(SW10));
}