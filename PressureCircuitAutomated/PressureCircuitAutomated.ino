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
#define SWAuto 35
#define buttonNext 53

uint32_t purple = strip.Color(191, 64, 191); // Declare LED color

unsigned long currentMillis;
unsigned long previousMillis = 0;

const unsigned long period_ms = 5000; // 5000 ms
const unsigned long default_delay = 500; // 500 ms
long stepDelay = period_ms;

int pressureState = 0;
bool autoState = false;
bool promptAsked = false;
bool buttonPressed = false;
int stateCounter = 1;

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
  currentMillis = millis();

  if (digitalRead(SWAuto) && (stateCounter <= 20)) { // automatic code
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
    Serial.print("Pressure: ");
    Serial.println(pressure_PSI);
    strip.clear();
    strip.setPixelColor(pressureMapped, purple);
    strip.show();
  }

  pressureState = pressureMapped; // update state
}

long solenoidActions(int stateNumber) {
  Serial.println(); // blank line
  Serial.print("Beginning State #");
  Serial.println(stateNumber);

  // case switch statements
  // Implementation:
  // LOW = solenoid on/vac
  // HIGH = solenoid off/vent
  // cases execute in order
  // number after return statement is delay for that step
  switch (stateNumber) {
    case 1:
      // do action: turn all off (vent)
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, HIGH); // vent
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      Serial.println("Connect card, start pump, then press enter to start cartridge run.");
      Serial.println("Place sample and buffer, then press enter.");
      return default_delay;
    break;

    case 2:
      // do action: Set S1 and S2 to vac
      digitalWrite(IN1, LOW); // vac
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      Serial.println("Press enter when ready to pull in sample to meter well.");
      return 10000; // delay 10000 ms
    break;
    
    case 3:
      // do action: Set S1 to vent
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      return 5000; // delay 5000 ms
    break;

    case 4:
      // do action: Set S3 to vac
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, LOW); // vac
      digitalWrite(IN4, HIGH); // vent
      Serial.println("Press enter to fill buffer.");
      return default_delay;
    break;

    case 5:
      // do action: Set S2 to vent
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, HIGH); // vent
      digitalWrite(IN3, LOW); // vac
      digitalWrite(IN4, HIGH); // vent
      Serial.println("Press enter to fill to mix well.");
      return default_delay;
    break;

    case 6:
      // do action: Set S2 to vac
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, LOW); // vac
      digitalWrite(IN4, HIGH); // vent
      return 2000; // delay 2000 ms
    break;

    case 7:
      // do action: Set S4 to vac
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, LOW); // vac
      digitalWrite(IN4, LOW); // vac
      return default_delay;
    break;

    case 8:
      // do action: Set S3 to vent
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, LOW); // vac
      Serial.println("Press enter when mixed.");
      return default_delay;
    break;

    case 9:
      // do action: Set S2 to vent
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, HIGH); // vent
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, LOW); // vac
      return default_delay;
    break;

    case 10:
      // do action: Set S4 to vent
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, HIGH); // vent
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      Serial.println("Move pneumatic lines 1 and 2 to position 5 and 6 respectively.");
      Serial.println("Close stage vent and then press enter to fill stage well.");
      return default_delay;
    break;

    case 11:
      // do action: Set S1 to vac
      digitalWrite(IN1, LOW); // vac
      digitalWrite(IN2, HIGH); // vent
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      Serial.println("Open stage vent and then press enter.");
      return default_delay;
    break;

    case 12:
      // do action: Set S4 to vac
      digitalWrite(IN1, LOW); // vac
      digitalWrite(IN2, HIGH); // vent
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, LOW); // vac
      Serial.println("Clear Lines then press enter to fill detection.");
      return 10000;
    break;

    case 13:
      // do action: Set S4 to vent
      digitalWrite(IN1, LOW); // vac
      digitalWrite(IN2, HIGH); // vent
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      return 13000;
    break;

    case 14:
      // do action: Set S2 to vac
      digitalWrite(IN1, LOW); // vac
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      return 1000;
    break;

    case 15:
      // do action: Set S1 to vent
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      return 10000;
    break;

    case 16:
      // do action: set S1 to vac
      digitalWrite(IN1, LOW); // vac
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      return 100;
    break;

    case 17:
      // do action: set S2 to vent
      digitalWrite(IN1, LOW); // vac
      digitalWrite(IN2, HIGH); // vent
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      return 10000;
    break;

    case 18:
      // do action: set S2 to vac
      digitalWrite(IN1, LOW); // vac
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      return 100;
    break;

    case 19:
      // do action: set S1 to vent
      digitalWrite(IN1, LOW); // vent
      digitalWrite(IN2, LOW); // vac
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      Serial.println("Press Enter to end script.");
      return default_delay;
    break;

    case 20:
      // do action: set all to vent
      digitalWrite(IN1, HIGH); // vent
      digitalWrite(IN2, HIGH); // vent
      digitalWrite(IN3, HIGH); // vent
      digitalWrite(IN4, HIGH); // vent
      return 100;
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
  switchStatus = digitalRead(SW1);
  digitalWrite(IN1, switchStatus);

  switchStatus = digitalRead(SW2);
  digitalWrite(IN2, switchStatus);

  switchStatus = digitalRead(SW3);
  digitalWrite(IN3, switchStatus);

  switchStatus = digitalRead(SW4);
  digitalWrite(IN4, switchStatus);

  switchStatus = digitalRead(SW5);
  digitalWrite(IN6_2, switchStatus);

 switchStatus = digitalRead(SW6);
  digitalWrite(IN5_2, switchStatus);

 switchStatus = digitalRead(SW7);
  digitalWrite(IN4_2, switchStatus);

  switchStatus = digitalRead(SW8);
  digitalWrite(IN3_2, switchStatus);

  switchStatus = digitalRead(SW9);
  digitalWrite(IN2_2, switchStatus);

  switchStatus = digitalRead(SW10);
  digitalWrite(IN1_2, switchStatus);
}