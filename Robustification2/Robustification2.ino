// ROBUST Pressure Actuated Microfluididc System (PAMS)

// include libraries
#include "Wire.h" // for I2C
#include "Adafruit_MPRLS.h" // pressure sensor

double atmosphere = 14.63; // define reference atmosphere pressure for computing relative pressure

// multiplexer setup
#define TCAADDR 0x70 // define multiplexer I2C address
// to select multiplexer output
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// Relay connection/signal pins
#define Solenoid1 38 // 3-way
#define Solenoid2 40 // 3-way
#define Solenoid3 42 // 3-way
#define Solenoid4 44 // 3-way
#define Solenoid5 32 // 2-way
#define Solenoid6 30 // 2-way
#define Solenoid7 28 // 2-way
#define Solenoid8 26 // 2-way
#define Solenoid9 24 // 2-way
#define Solenoid10 22 // 2-way

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
#define SWAuto 35 // to switch between manual and automated code
#define buttonNext 53 // to advance to next step
#define readyLED 7 // indicates if ready to go to next step

// pump PID pin
#define pumpPin 10

// pressure sensor setup
#define RESET_PIN -1 // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN -1 // set to any GPIO pin to read end-of-conversion by pin
// create pressure sensor objects
Adafruit_MPRLS mpr0 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr1 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr2 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr3 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr4 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr5 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr6 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr7 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
// create variables to store pressure sensor values 
double pressure0;
double pressure1;
double pressure2;
double pressure3;
double pressure4;
double pressure5;
double pressure6;
double pressure7;

// state machine variables
unsigned long currentMillis;
unsigned long previousMillis = 0;
#define numStates 16 // = number of states + 1
bool autoNext = false; // auto advance to next step

const unsigned long default_delay = 10;
unsigned long period_ms = default_delay;

int pressureState = 0;
bool autoState = false;
bool promptAsked = false;
bool buttonPressed = false;
int stateCounter = 1;

bool pumpState = false; // for pump bang bang control

// user input FSM
bool state = 0;
int ps = 0, ns = 0;

// auto pressure method variables
bool pressureNext = false;
double slack;

void setup() {
  // change valve PWM pin frequencies
  TCCR2B = TCCR2B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz on 9 and 10
  // TCCR3B = TCCR3B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz on 2, 3, and 5
  // TCCR4B = TCCR4B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz on 6, 7, and 8

  // I2C and Serial setup
  Wire.begin(); // I2C
  Serial.begin(9600);
  Serial1.begin(9600);

  // scan for I2C devices
  I2Cscan();

  // check if all 8 sensors are initialized
  sensorInitialization();

  // initialize relay pins
  pinMode(Solenoid1, OUTPUT);
  pinMode(Solenoid2, OUTPUT);
  pinMode(Solenoid3, OUTPUT);
  pinMode(Solenoid4, OUTPUT);
  pinMode(Solenoid5, OUTPUT);
  pinMode(Solenoid6, OUTPUT);
  pinMode(Solenoid7, OUTPUT);
  pinMode(Solenoid8, OUTPUT);
  pinMode(Solenoid9, OUTPUT);
  pinMode(Solenoid10, OUTPUT);
  digitalWrite(46, OUTPUT); // unused relay
  digitalWrite(48, OUTPUT); // unused relay
  digitalWrite(50, OUTPUT); // unused relay
  digitalWrite(52, OUTPUT); // unused relay
  digitalWrite(34, OUTPUT); // unused relay
  digitalWrite(36, OUTPUT); // unused relay

  // initialize ready LED
  pinMode(readyLED, OUTPUT);
  digitalWrite(readyLED, LOW);

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

  // turn off all relay pins (HIGH = off) by default
  digitalWrite(Solenoid1, HIGH);
  digitalWrite(Solenoid2, HIGH);
  digitalWrite(Solenoid3, HIGH);
  digitalWrite(Solenoid4, HIGH);
  digitalWrite(Solenoid5, HIGH);
  digitalWrite(Solenoid6, HIGH);
  digitalWrite(Solenoid7, HIGH);
  digitalWrite(Solenoid8, HIGH);
  digitalWrite(Solenoid9, HIGH);
  digitalWrite(Solenoid10, HIGH);
  digitalWrite(Solenoid10, HIGH);
  digitalWrite(46, HIGH); // unused relay
  digitalWrite(48, HIGH); // unused relay
  digitalWrite(50, HIGH); // unused relay
  digitalWrite(52, HIGH); // unused relay
  digitalWrite(34, HIGH); // unused relay
  digitalWrite(36, HIGH); // unused relay
}

void loop() {
  currentMillis = millis(); // update millis

  // pump control
  // pumpBangBang (6, 8);
  analogWrite(pumpPin, 255); // always on
  // analogWrite(pumpPin, 0); // always off

  pressureRead(); // pressure read

  userInput(); // update user input FSM

  pressureNext = autoPressure(stateCounter); // check pressures to determine eligibility to move to next step

  if (digitalRead(SWAuto) && (stateCounter <= numStates)) { // automatic code
    if (currentMillis - previousMillis >= period_ms) { // if delay has been reached
      if (!promptAsked) { // if prompt hasn't been asked yet, ask prompt
        Serial.print("Press button to begin state #");
        Serial.println(stateCounter);
        promptAsked = true;
        digitalWrite(readyLED, HIGH); // turn ready LED on
      }
      if ((state || autoNext) && pressureNext) { // if button is pushed OR autoNext is true, AND pressures are eligible, go to next step
        period_ms = solenoidActions(stateCounter);
        stateCounter++; // increment state
        promptAsked = false;
        previousMillis = currentMillis; // reset millis counter
      }
    } else {
      digitalWrite(readyLED, LOW); // turn ready LED off
    }
  } else { // manual code
    solenoidManual(); // update solenoids according to switches
    stateCounter = 1; // reset state
  }
}

// to scan for I2C devices connected to the multiplexer
void I2Cscan() {
  Serial.println("\nTCAScanner ready!");
  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;

      Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
        Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }
  Serial.println("\ndone");
}

// check if all 8 sensors are connected and initialized
void sensorInitialization() {
  tcaselect(0);
  if (!mpr0.begin()) {
    Serial.println("MPRLS sensor 0 failed");
  } else {
  Serial.println("Found MPRLS sensor 0");
  }

  tcaselect(1);
  if (!mpr1.begin()) {
    Serial.println("MPRLS sensor 1 failed");
  } else {
  Serial.println("Found MPRLS sensor 1");
  }

  tcaselect(2);
  if (!mpr2.begin()) {
    Serial.println("MPRLS sensor 2 failed");
  } else {
  Serial.println("Found MPRLS sensor 2");
  }

  tcaselect(3);
  if (!mpr3.begin()) {
    Serial.println("MPRLS sensor 3 failed");
  } else {
  Serial.println("Found MPRLS sensor 3");
  }

  tcaselect(4);
  if (!mpr4.begin()) {
    Serial.println("MPRLS sensor 4 failed");
  } else {
  Serial.println("Found MPRLS sensor 4");
  }

  tcaselect(5);
  if (!mpr5.begin()) {
    Serial.println("MPRLS sensor 5 failed");
  } else {
  Serial.println("Found MPRLS sensor 5");
  }

  tcaselect(6);
  if (!mpr6.begin()) {
    Serial.println("MPRLS sensor 6 failed");
  } else {
  Serial.println("Found MPRLS sensor 6");
  }

  tcaselect(7);
  if (!mpr7.begin()) {
    Serial.println("MPRLS sensor 7 failed");
  } else {
  Serial.println("Found MPRLS sensor 7");
  }
}

// automated solenoid control
long solenoidActions(int stateNumber) {
  // intro line
  Serial.println(); // blank line
  Serial.println("/////////////////");
  Serial.print("Beginning State #");
  Serial.println(stateNumber);

  // Implementation:
  // LOW = solenoid on/vac
  // HIGH = solenoid off/vent
  // cases execute in order
  // number after return statement is delay for that step

  // digitalWrite(Solenoid1, HIGH/LOW); // vent/vac -> 1
  // digitalWrite(Solenoid2, HIGH/LOW); // vent/vac -> 2
  // digitalWrite(Solenoid3, HIGH/LOW); // vent/vac -> 3
  // digitalWrite(Solenoid4, HIGH/LOW); // vent/vac -> 4
  // digitalWrite(Solenoid5, HIGH/LOW); // vent/vac -> 5
  // digitalWrite(Solenoid6, HIGH/LOW); // vent/vac -> 6
  // digitalWrite(Solenoid7, HIGH/LOW); // vent/vac -> 7
  // digitalWrite(Solenoid8, HIGH/LOW); // vent/vac -> 8
  // digitalWrite(Solenoid9, HIGH/LOW); // vent/vac -> 9
  // digitalWrite(Solenoid10, HIGH/LOW); // vent/vac -> 10
  switch (stateNumber) {
    case 1:
      // do action: turn all off, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      Serial.println("Press button to begin.");
      autoNext = true;
      return default_delay;
    break;

    case 2:
      // do action: 8 on, delay 3s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // vac -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      autoNext = true;
      return 3000;
    break;
    
    case 3:
      // do action: 1 on, pressure check
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // vac -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      // Serial.println("Check plasma fill, press button to continue.");
      autoNext = true;
      return 2000;
    break;

    case 4:
      // do action: Set 8 off, delay 0.5s
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      autoNext = true;
      return 500;
    break;

    case 5:
      // do action: Set 2 on, delay 5s
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      // Serial.println("Confirm removal of plasma from channel, press button to continue.");
      autoNext = true;
      return 5000;
    break;

    case 6:
      // do action: Set 3 and 5 on, pressure check
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, LOW); // vac -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, LOW); // vac -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      // Serial.println("Confirm dial filled completely, press button to continue.");
      autoNext = true;
      return 2000;
    break;

    case 7:
      // do action: Set 1 and 3 off, 9 on, delay 0.25s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, LOW); // vac -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, LOW); // vac -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      autoNext = true;
      return 250;
    break;

    case 8:
      // do action: Set 6 on, delay 0.25s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, LOW); // vac -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, LOW); // vac -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      autoNext = true;
      return 250;
    break;

    case 9:
      // do action: Set 5 off, pressure check
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, LOW); // vac -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      // Serial.println("Confirm well 6 filled completely, press button to continue.");
      autoNext = true;
      return 2000;
    break;

    case 10:
      // do action: Set 4 on, delay 6s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, LOW); // vac -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, LOW); // vac -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      // Serial.println("Confirm clear of buffer, press button to continue.");
      autoNext = true;
      return 6000;
    break;

    case 11:
      // do action: Set 9 off, delay 0.25s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, LOW); // vac -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      autoNext = true;
      return 250;
    break;

    case 12:
      // do action: Set 7 on, pressure check
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, LOW); // vac -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      // Serial.println("Confirm 16 wells filled completely, press button to continue.");
      autoNext = true;
      return 2000;
    break;

    case 13:
      // do action: Set 10 on, delay 0.5s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, LOW); // vac -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, LOW); // vac -> 10
      autoNext = true;
      return 500;
    break;

    case 14:
      // do action: Set 3, 5, and 6 on, delay 6s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, LOW); // vac -> 3
      digitalWrite(Solenoid4, LOW); // vac -> 4
      digitalWrite(Solenoid5, LOW); // vac -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, LOW); // vac -> 10
      // Serial.println("Confirm channel is cleared, press button to continue.");
      autoNext = true;
      return 6000;
    break;

    case 15:
      // do action: Set all except 7 off, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      Serial.println("Workflow complete! Robustification achieved.");
      Serial.println("////////////////////////////////////////////");
      Serial.println();      
      autoNext = false;
      return default_delay;
    break;

    default:
      // by default, do action: turn all off
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // vent -> 8
      digitalWrite(Solenoid9, HIGH); // vent -> 9
      digitalWrite(Solenoid10, HIGH); // vent -> 10
      autoNext = false;
      return default_delay;
    break;
  }
}

// check if pressures are at desired value, if so, return true
bool autoPressure(int stateNum) {  
  slack = 0.98 * pressure7;

  switch (stateNum) {
    case 4:
      if (pressure1 >= slack) {
        return true;
      } else {
        return false;
      }
    break;

    case 7:
      if (pressure4 >= slack) {
        return true;
      } else {
        return false;
      }
    break;

    case 10:
      if (pressure5 >= slack) {
        return true;
      } else {
        return false;
      }
    break;

    case 13:
      if (pressure6 >= slack) {
        return true;
      } else {
        return false;
      }
    break;

    default:
      return true;
    break;
  }
}

// control solenoids according to switches
void solenoidManual() {
  if (digitalRead(SW1)) {
    digitalWrite(Solenoid1, HIGH);
  } else {
    digitalWrite(Solenoid1, LOW);
    // Serial.println("Solenoid 1 on");
  }

  if (digitalRead(SW2)) {
    digitalWrite(Solenoid2, HIGH);
    // Serial.println("Solenoid 2 on");
  } else {
    digitalWrite(Solenoid2, LOW);
  }

  if (digitalRead(SW3)) {
    digitalWrite(Solenoid3, HIGH);
    // Serial.println("Solenoid 3 on");
  } else {
    digitalWrite(Solenoid3, LOW);
  }

  if (digitalRead(SW4)) {
    digitalWrite(Solenoid4, HIGH);
    // Serial.println("Solenoid 4 on");
  } else {
    digitalWrite(Solenoid4, LOW);
  }

  if (digitalRead(SW5)) {
    digitalWrite(Solenoid5, HIGH);
    // Serial.println("Solenoid 5 on");
  } else {
    digitalWrite(Solenoid5, LOW);
  }

  if (digitalRead(SW6)) {
    digitalWrite(Solenoid6, HIGH);
    // Serial.println("Solenoid 6 on");
  } else {
    digitalWrite(Solenoid6, LOW);
  }

  if (digitalRead(SW7)) {
    digitalWrite(Solenoid7, HIGH);
    // Serial.println("Solenoid 7 on");
  } else {
    digitalWrite(Solenoid7, LOW);
  }

  if (digitalRead(SW8)) {
    digitalWrite(Solenoid8, HIGH);
    // Serial.println("Solenoid 8 on");
  } else {
    digitalWrite(Solenoid8, LOW);
  }

  if (digitalRead(SW9)) {
    digitalWrite(Solenoid9, HIGH);
    // Serial.println("Solenoid 9 on");
  } else {
    digitalWrite(Solenoid9, LOW);
  }

  if (digitalRead(SW10)) {
    digitalWrite(Solenoid10, HIGH);
    // Serial.println("Solenoid 10 on");
  } else {
    digitalWrite(Solenoid10, LOW);
  }
}

// turn pump fully on or fully off in order to keep it between two setpoints
void pumpBangBang(int lower, int upper) {
  // read pressure in accumulator
  tcaselect(7); // select multiplexer output
  pressure7 = mpr7.readPressure() / 68.947572932; // read pressure, convert to PSI
  
  // keep pressure in accumulator between threshold values
  if (pumpState == false && pressure7 > upper) { // if pressure is above upper limit, turn pump on
    pumpState = true;
  } else if (pumpState == true && pressure7 < lower) { // if pump is on and pressure falls below lower limit, turn pump off
    pumpState = false;
  }

  if (pumpState) {
    analogWrite(pumpPin, 255); // enable pump
  } else {
    analogWrite(pumpPin, 0); // disable relay (off)
  }
}

// read pressure on all MPRLS sensors
void pressureRead() {
  // read from pressure sensors
  tcaselect(0); // select multiplexer output
  pressure0 = mpr0.readPressure() / 68.947572932; // read pressure, convert to PSI

  tcaselect(1); // select multiplexer output
  pressure1 = mpr1.readPressure() / 68.947572932; // read pressure, convert to PSI

  tcaselect(2); // select multiplexer output
  pressure2 = mpr2.readPressure() / 68.947572932; // read pressure, convert to PSI

  tcaselect(3); // select multiplexer output
  pressure3 = mpr3.readPressure() / 68.947572932; // read pressure, convert to PSI

  tcaselect(4); // select multiplexer output
  pressure4 = mpr4.readPressure() / 68.947572932; // read pressure, convert to PSI

  tcaselect(5); // select multiplexer output
  pressure5 = mpr5.readPressure() / 68.947572932; // read pressure, convert to PSI

  tcaselect(6); // select multiplexer output
  pressure6 = mpr6.readPressure() / 68.947572932; // read pressure, convert to PSI

  tcaselect(7); // select multiplexer output
  pressure7 = mpr7.readPressure() / 68.947572932; // read pressure, convert to PSI

  // convert absolute to relative pressure
  pressure0 = atmosphere - pressure0;
  pressure1 = atmosphere - pressure1;
  pressure2 = atmosphere - pressure2;
  pressure3 = atmosphere - pressure3;
  pressure4 = atmosphere - pressure4;
  pressure5 = atmosphere - pressure5;
  pressure6 = atmosphere - pressure6;
  pressure7 = atmosphere - pressure7;
  
  // print pressures to serial monitor/plotter
  Serial1.print(pressure0);
  Serial1.print(",");
  Serial1.print(pressure1);
  Serial1.print(",");
  Serial1.print(pressure2);
  Serial1.print(",");
  Serial1.print(pressure3);
  Serial1.print(",");
  Serial1.print(pressure4);
  Serial1.print(",");
  Serial1.print(pressure5);
  Serial1.print(",");
  Serial1.print(pressure6);
  Serial1.print(",");
  Serial1.print(pressure7);
  Serial1.print(",");

  // plot button pulse
  if (digitalRead(buttonNext)) {
    Serial1.println(1);
  } else {
    Serial1.println(0);
  }
}

// user input FSM
void userInput() {
  switch (ps) {
    case 0:
      if (digitalRead(buttonNext)) {
        ns = 1;
      } else {
        ns = 0;
      }  
    break;

    case 1:
      ns = 2;
    break;

    case 2:
      if (!digitalRead(buttonNext)) {
        ns = 0;
      } else {
        ns = 2;
      }
    break;

    default:
      ns = 0;
    break;
  }

  if (ps == 1) {
    state = 1;
  } else {
    state = 0;
  }

  // present state = next state
  ps = ns;
}