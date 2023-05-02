// ROBUSTificated Pressure Actuated Microfluididc System (PAMS)

// include libraries
#include "Wire.h" // for I2C
#include "Adafruit_MPRLS.h" // pressure sensor

double atmosphere = 14.63;

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
#define Solenoid5 32 // 3-way
#define Solenoid6 30 // 3-way
#define Solenoid7 28 // 3-way
#define Solenoid8 26 // 2-way
#define Solenoid9 24 // 2-way
#define Solenoid10 22 // 2-way
#define Solenoid11 46 // 2-way
#define Solenoid12 48 // spare channel

// Switch connection pins
#define SW1 43
#define SW2 41
#define SW3 39
#define SW4 37
#define SW5 35
#define SW6 31
#define SW7 29
#define SW8 27
#define SW9 25
#define SW10 23
#define SW11 45
#define SW12 47
#define SWAuto 33 // to switch between manual and automated code
#define buttonNext 53 // to advance to next step
#define readyLED 7 // to indicate if ready to advance to next step

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
#define numStates 17 // = number of states + 1 FOR NORMAL
//#define numStates 18 // = number of states + 1 FOR BUFFER STAGE
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
  pinMode(Solenoid11, OUTPUT);
  pinMode(Solenoid12, OUTPUT);
  pinMode(50, OUTPUT); // unused relay
  pinMode(52, OUTPUT); // unused relay
  pinMode(34, OUTPUT); // unused relay
  pinMode(36, OUTPUT); // unused relay

  // initialize ready LED
  pinMode(readyLED, OUTPUT);

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
  pinMode(SW11, INPUT);
  pinMode(SW12, INPUT);
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
  digitalWrite(Solenoid11, HIGH);
  digitalWrite(Solenoid12, HIGH);
  digitalWrite(50, HIGH); // unused relay
  digitalWrite(52, HIGH); // unused relay
  digitalWrite(34, HIGH); // unused relay
  digitalWrite(36, HIGH); // unused relay
}

void loop() {
  currentMillis = millis(); // update millis
  userInput(); // update user input FSM

  if (digitalRead(SWAuto) && (stateCounter <= numStates)) { // automatic code
    if (currentMillis - previousMillis >= period_ms) {
      if (!promptAsked) {
        Serial.print("Press button to begin state #");
        Serial.println(stateCounter);
        promptAsked = true;
        digitalWrite(readyLED, HIGH); // turn ready LED on
      }
      if (state || autoNext) {
        period_ms = solenoidActions(stateCounter); // FOR NORMAL
        //period_ms = solenoidActions_BufferStage(stateCounter); // FOR BUFFER STAGE
        stateCounter++; // increment state
        promptAsked = false;
        previousMillis = currentMillis; // reset millis counter
      }
    } else {
      digitalWrite(readyLED, LOW); // turn ready LED off
    }
  } else { // manual code
    solenoidManual(); // update solenoids according to switches
    digitalWrite(readyLED, LOW); // turn ready LED off
    stateCounter = 1;
  }

  // pump control
  // pumpBangBang (6, 8);
  analogWrite(pumpPin, 255); // always on
  // analogWrite(pumpPin, 0); // always off

  // pressure read
  pressureRead();
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
// normal cartridge
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

  // NOTE: 11 is linked to 2
  // vac = 11 LOW, 2 LOW
  // close = 11 HIGH, 2 HIGH
  // vent = 11 LOW, 2 HIGH

  // NOTE: 8 is linked to 4
  // vac = 8 LOW, 4 LOW
  // close = 8 HIGH, 4 HIGH
  // vent = 8 HIGH, 4 LOW

  // digitalWrite(Solenoid1, HIGH/LOW); // vent/vac -> 1 (port 1)
  // digitalWrite(Solenoid2, HIGH/LOW); // vent/vac -> 2 (port 2)
  // digitalWrite(Solenoid3, HIGH/LOW); // vent/vac -> 3 (port 3)
  // digitalWrite(Solenoid4, HIGH/LOW); // vent/vac -> 4 (port 4)
  // digitalWrite(Solenoid5, HIGH/LOW); // vent/vac -> 5 (port 5)
  // digitalWrite(Solenoid6, HIGH/LOW); // vent/vac -> 6 (port 6)
  // digitalWrite(Solenoid7, HIGH/LOW); // vent/vac -> 7 (port 7)
  // digitalWrite(Solenoid8, HIGH/LOW); // blocked/open -> 8 (port 4)
  // digitalWrite(Solenoid9, HIGH/LOW); // blocked/open -> 9 (port 8)
  // digitalWrite(Solenoid10, HIGH/LOW); // blocked/open -> 10 (port 9)
  // digitalWrite(Solenoid11, HIGH/LOW); // blocked/open -> 11 (port 2)
  // digitalWrite(Solenoid12, HIGH/LOW); // blocked/open -> 12 (port 10)

  switch (stateNumber) {
    case 1:
      // do action: turn all off, message, delay 0.5s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press button to begin.");
      autoNext = true;
      return default_delay;
    break;

    case 2:
      // do action: 1 on, delay 0.5s, message
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Check plasma fill, press button to open vent to help fill and continue.");
      autoNext = false;
      return default_delay;
    break;

    case 3:
      // do action: Set 11 on, delay 0.5s, message
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, LOW); // open -> 11
      Serial.println("Verify sample is full, press button to continue.");
      autoNext = false;
      return default_delay;
    break;

    case 4:
      // do action: Set 8 on, delay 2s, go to next step
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, LOW); // open -> 11
      autoNext = true;
      return 2000;
    break;

    case 5:
      // do action: Set 2 on, delay 0.5s, message
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
     digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, LOW); // open -> 11
      Serial.println("Verify sample is cleared, press button to continue.");
      autoNext = false;
      return default_delay;
    break;

    case 6:
      // do action: Set 2 off and 11 off, delay 2s, go to next step
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // open -> 11
      autoNext = true;
      return 2000;
    break;

    case 7:
      // do action: Set 3 on and 5 on, delay 0.5s, message
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, LOW); // vac -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, LOW); // vac -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Wait for dial to fill, press button to continue.");
      autoNext = false;
      return default_delay;
    break;

    case 8:
      // do action: Set 1 off and 3 off and 5 off and 9 on, delay 2s, go to next step
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      autoNext = true;
      return 2000;
    break;

    case 9:
      // do action: Set 6 on, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press buton to continue when dial is empty.");
      autoNext = false;
      return default_delay;
    break;

    case 10:
      // do action: Set 4 on, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, LOW); // vac -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Verify buffer is clear, press button to continue.");
      autoNext = false;
      return default_delay;
    break;

    case 11:
      // do action: Set 4 off, delay 2s, go to next step
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      autoNext = true;
      return 2000;
    break;

    case 12:
      // do action: Set 7 on, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press button when detection wells are filled.");
      autoNext = false;
      return default_delay;
    break;

    case 13:
      // do action: Set 6 off, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press button when detection wells are filled.");
      autoNext = false;
      return default_delay;
    break;

    case 14:
      // do action: Set 10 on, delay 2s, go to next step
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, LOW); // open -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      autoNext = true;
      return 2000;
    break;

    case 15:
      // do action: Set 6 on and 5 on and 3 on, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, LOW); // vac -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, LOW); // vac -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, LOW); // open -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press button to confirm detection clear.");
      autoNext = false;
      return default_delay;
    break;

    case 16:
      // do action: Set all except 7 and 10 off, message, delay 0.5s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, LOW); // open -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
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
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      autoNext = false;
      return default_delay;
    break;
  }
}

// automated solenoid control
// Buffer stage cartridge
long solenoidActions_BufferStage(int stateNumber) {
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

  // NOTE: 11 is linked to 2
  // vac = 11 LOW, 2 LOW
  // close = 11 HIGH, 2 HIGH
  // vent = 11 LOW, 2 HIGH

  // NOTE: 8 is linked to 4
  // vac = 8 LOW, 4 LOW
  // close = 8 HIGH, 4 HIGH
  // vent = 8 HIGH, 4 LOW

  // digitalWrite(Solenoid1, HIGH/LOW); // vent/vac -> 1 (port 1)
  // digitalWrite(Solenoid2, HIGH/LOW); // vent/vac -> 2 (port 2)
  // digitalWrite(Solenoid3, HIGH/LOW); // vent/vac -> 3 (port 3)
  // digitalWrite(Solenoid4, HIGH/LOW); // vent/vac -> 4 (port 4)
  // digitalWrite(Solenoid5, HIGH/LOW); // vent/vac -> 5 (port 5)
  // digitalWrite(Solenoid6, HIGH/LOW); // vent/vac -> 6 (port 6)
  // digitalWrite(Solenoid7, HIGH/LOW); // vent/vac -> 7 (port 7)
  // digitalWrite(Solenoid8, HIGH/LOW); // blocked/open -> 8 (port 4)
  // digitalWrite(Solenoid9, HIGH/LOW); // blocked/open -> 9 (port 8)
  // digitalWrite(Solenoid10, HIGH/LOW); // blocked/open -> 10 (port 9)
  // digitalWrite(Solenoid11, HIGH/LOW); // blocked/open -> 11 (port 2)
  // digitalWrite(Solenoid12, HIGH/LOW); // blocked/open -> 12 (port 10)

  switch (stateNumber) {
    case 1:
      // do action: turn all off, message, delay 0.5s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press button to begin.");
      autoNext = true;
      return default_delay;
    break;

    case 2:
      // do action: 1 on, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, LOW); // vac -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Pull Buffer to reservoir, press enter to continue");
      autoNext = false;
      return default_delay;
    break;

        case 3:
      // do action: 1 on, delay 0.5s, message
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Check plasma fill, press button to open vent to help fill and continue.");
      autoNext = false;
      return default_delay;
    break;

    case 4:
      // do action: Set 11 on, delay 0.5s, message
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, LOW); // open -> 11
      Serial.println("Verify sample is full, press button to continue.");
      autoNext = false;
      return default_delay;
    break;

    case 5:
      // do action: Set 8 on, delay 2s, go to next step
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, LOW); // open -> 11
      autoNext = true;
      return 2000;
    break;

    case 6:
      // do action: Set 2 on, delay 0.5s, message
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, LOW); // vac -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
     digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, LOW); // open -> 11
      Serial.println("Verify sample is cleared, press button to continue.");
      autoNext = false;
      return default_delay;
    break;

    case 7:
      // do action: Set 2 off and 11 off, delay 2s, go to next step
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      autoNext = true;
      return 2000;
    break;

    case 8:
      // do action: Set 3 on and 5 on, delay 0.5s, message
      digitalWrite(Solenoid1, LOW); // vac -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, LOW); // vac -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, LOW); // vac -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Wait for dial to fill, press button to continue.");
      autoNext = false;
      return default_delay;
    break;

    case 9:
      // do action: Set 1 off and 3 off and 5 off and 9 on, delay 2s, go to next step
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      autoNext = true;
      return 2000;
    break;

    case 10:
      // do action: Set 6 on, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press buton to continue when dial is empty.");
      autoNext = false;
      return default_delay;
    break;

    case 11:
      // do action: Set 4 on, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, LOW); // vac -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Verify buffer is clear, press button to continue.");
      autoNext = false;
      return default_delay;
    break;

    case 12:
      // do action: Set 4 off, delay 2s, go to next step
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, HIGH); // vent -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      autoNext = true;
      return 2000;
    break;

    case 13:
      // do action: Set 7 on, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press button when detection wells are filled.");
      autoNext = false;
      return default_delay;
    break;

    case 14:
      // do action: Set 6 off, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press button when detection wells are filled.");
      autoNext = false;
      return default_delay;
    break;

    case 15:
      // do action: Set 10 on, delay 2s, go to next step
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, LOW); // open -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      autoNext = true;
      return 2000;
    break;

    case 16:
      // do action: Set 6 on and 5 on and 3 on, delay 0.5s, message
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, LOW); // vac -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, LOW); // vac -> 5
      digitalWrite(Solenoid6, LOW); // vac -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, LOW); // open -> 8
      digitalWrite(Solenoid9, LOW); // open -> 9
      digitalWrite(Solenoid10, LOW); // open -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      Serial.println("Press button to confirm detection clear.");
      autoNext = false;
      return default_delay;
    break;

    case 17:
      // do action: Set all except 7 and 10 off, message, delay 0.5s
      digitalWrite(Solenoid1, HIGH); // vent -> 1
      digitalWrite(Solenoid2, HIGH); // vent -> 2
      digitalWrite(Solenoid3, HIGH); // vent -> 3
      digitalWrite(Solenoid4, HIGH); // vent -> 4
      digitalWrite(Solenoid5, HIGH); // vent -> 5
      digitalWrite(Solenoid6, HIGH); // vent -> 6
      digitalWrite(Solenoid7, LOW); // vac -> 7
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, LOW); // open -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
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
      digitalWrite(Solenoid8, HIGH); // blocked -> 8
      digitalWrite(Solenoid9, HIGH); // blocked -> 9
      digitalWrite(Solenoid10, HIGH); // blocked -> 10
      digitalWrite(Solenoid11, HIGH); // blocked -> 11
      autoNext = false;
      return default_delay;
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

  if (digitalRead(SW11)) {
    digitalWrite(Solenoid11, HIGH);
    // Serial.println("Solenoid 11 on");
  } else {
    digitalWrite(Solenoid11, LOW);
  }

  if (digitalRead(SW12)) {
    digitalWrite(Solenoid12, HIGH);
    // Serial.println("Solenoid 12 on");
  } else {
    digitalWrite(Solenoid12, LOW);
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

  // plot button pulse on serial channel 9
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