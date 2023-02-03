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

// Switch connection pins
#define SW1 43
#define SW2 41
#define SW3 39
#define SW4 37
#define SWAuto 35
#define buttonNext 53

uint32_t purple = strip.Color(191, 64, 191); // Declare LED color

unsigned long currentMillis;

unsigned long previousMillis = 0;

const unsigned long period_ms = 5000;

int pressureState = 0;

bool autoState = false;

bool promptAsked = false;

bool buttonPressed = false;

int stateCounter = 1;

long stepDelay = period_ms;

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

  // initialize Switch pins
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  pinMode(SW4, INPUT);
  pinMode(SWAuto, INPUT);
  pinMode(buttonNext, INPUT);

  // turn off relay pins (HIGH = off) by default
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
  pressureUpdate();
  currentMillis = millis();

  if (digitalRead(SWAuto) && (stateCounter <= 16)) { // automatic code
    if (currentMillis - previousMillis >= stepDelay) {
      if (!promptAsked) {
        Serial.print("Press button to begin state #");
        Serial.println(stateCounter);
        promptAsked = true;
      }
      if (digitalRead(buttonNext)) {
        stepDelay = solenoidActions(stateCounter);
        stateCounter++; // increment state
        promptAsked = false;
        previousMillis = currentMillis; // reset millis counter
      }
    }
  } else { // manual code
    solenoidManual();
    stateCounter = 1;
  }
}

// bool serialPrompt(int stateToDo) {
//   char read;
  
//   // Ask prompt
//   if (!promptAsked) {
//     Serial.print("Press y to start state #");
//     Serial.println(stateCounter);
//     promptAsked = true;
//   }

//   while (Serial.available() > 0) {
//     read = Serial.read();

//     while (read != 'y') {}

//     switch (read) {
//         case 'y':
//           Serial.println("Y pressed");
//           promptAsked = false;
//           return true;
//         break;
        
//         default:
//           Serial.println("Please enter a valid input.");
//           return false;
//         break;
//     }
//   }
// }

void pressureUpdate() {
  float pressure_hPa = mpr.readPressure();
  float pressure_PSI = pressure_hPa / 68.947572932;
  int pressureMapped = map(pressure_PSI, 0, 14, 0, 23); // map PSI pressure from 0 (absolute vaccuum) to 14 (atmospheric) to 0 to 23 (for the 24 LEDs)

  // if pressure changes, change LED
  if (pressureMapped != pressureState) {
    // Serial.println("Pressure: " + pressure_PSI); // for serial plotter
    strip.clear();
    strip.setPixelColor(pressureMapped, purple);
    strip.show();
  }

  pressureState = pressureMapped; // update state
}

long solenoidActions(int stateNumber) {
  Serial.print("Beginning State #");
  Serial.println(stateNumber);

  // case switch statements
  switch (stateNumber) {
    case 1:
      // do action: turn all on
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      return 1000;
    break;

    case 2:
      // do action:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      return 2000;
    break;
    
    case 3:
      // do action:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      return 3000;
    break;

    case 4:
      // do action:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      return 4000;
    break;

    case 5:
      // do action:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      return 5000;
    break;

    case 6:
      // do action:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      return 6000;
    break;

    case 7:
      // do action:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      return 7000;
    break;

    case 8:
      // do action:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      return 8000;
    break;

    case 9:
      // do action:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      return 9000;
    break;

    case 10:
      // do action:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      return 10000;
    break;

    case 11:
      // do action:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      return 11000;
    break;

    case 12:
      // do action:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      return 12000;
    break;

    case 13:
      // do action:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      return 13000;
    break;

    case 14:
      // do action:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      return 14000;
    break;

    case 15:
      // do action:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      return 15000;
    break;

    case 16:
      // do action: turn all off
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      return 16000;
    break;

    default:
      // by default, do action: turn all off
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      return period_ms;
    break;
  }
}

void solenoidManual() {
  if (digitalRead(SW1)) {
    digitalWrite(IN1, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    // Serial.println("Solenoid 1 on");
  }

  if (digitalRead(SW2)) {
    digitalWrite(IN2, HIGH);
    // Serial.println("Solenoid 2 on");
  } else {
    digitalWrite(IN2, LOW);
  }

  if (digitalRead(SW3)) {
    digitalWrite(IN3, HIGH);
    // Serial.println("Solenoid 3 on");
  } else {
    digitalWrite(IN3, LOW);
  }

  if (digitalRead(SW4)) {
    digitalWrite(IN4, HIGH);
    // Serial.println("Solenoid 4 on");
  } else {
    digitalWrite(IN4, LOW);
  }
}