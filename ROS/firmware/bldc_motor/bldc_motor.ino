// BLDC motor firmware for the XMC1100 Boot Kit with the TLE9879QXA40 BLDC shield.
//
// The shield has its own microcontroller with field-oriented control (FOC).
// The XMC1100 only sends 16-bit SPI commands: speed, start and stop.
//
// Serial protocol to the ROS bridge (115200 baud), same as the other boards:
//   HELLO    -> "HELLO bldc 1" and READY / BUSY <id> / REARMING / FAULT <reason>
//   PING     -> PONG
//   RUN <id> -> STARTED <id>, the motor runs for 2 s, then DONE <id>
//   STOP     -> STOPPED <id>
// BLDC special case: the shield only starts reliably again after a reset.
// After every run, it is therefore reset in the background (~2.5 s):
//   REARMING -> reset started, RUN is answered with "ERROR <id> REARMING"
//   ARMED    -> the shield is ready again

#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>

#if !defined(XMC1100_Boot_Kit)
#error Select KIT_XMC11_BOOT_001 for this BLDC firmware.
#endif

const char MOTOR_NAME[] = "bldc";
const uint32_t RUN_MS = 2000;
const uint16_t MOTOR_RPM = 1000;

// SPI chip select of the shield. Pins 4-7 are the possible shield addresses.
const uint8_t SELECT_PIN = 4;

// SPI commands of the shield.
const uint16_t CMD_SET_SPEED = 0x0700;   // followed by the speed in rpm
const uint16_t CMD_START = 0x0800;
const uint16_t CMD_STOP = 0x0801;
const uint16_t CMD_CHECK_ERROR = 0x0C00;
const uint16_t CMD_ACK_CHECK = 0x1100;   // reads the acknowledgment (command + 0x1000)

// Waiting times after a shield reset and after requesting FOC mode.
const uint32_t RESET_WAIT_MS = 1500;
const uint32_t FOC_WAIT_MS = 1000;

// Background rearm after a run: RESET -> FOC -> IDLE (= ready again).
enum RearmStage : uint8_t { REARM_IDLE, REARM_RESET, REARM_FOC };
RearmStage rearmStage = REARM_IDLE;
uint32_t rearmAt = 0;
uint32_t rearmId = 0;

bool ready = false;                       // false = fault, no RUN accepted
const char* faultReason = "NOT_INITIALIZED";
char spiFault[72];                        // detailed text for SPI faults
bool motorRunning = false;
uint32_t activeId = 0;                    // job ID of the current run
uint32_t lastRunId = 0;                   // the same job is never started twice
uint32_t startedMs = 0;

// Buffer for the incoming command line.
const size_t LINE_SIZE = 48;
char inputLine[LINE_SIZE];
size_t inputLength = 0;
bool discardLine = false;

uint32_t lastLedMs = 0;
bool ledOn = false;

// --- Serial answers -----------------------------------------------------------

void reportId(const char* event, uint32_t id) {
  Serial.print(event);
  Serial.print(' ');
  Serial.println(static_cast<unsigned long>(id));
}

void reportError(uint32_t id, const char* reason) {
  Serial.print(F("ERROR "));
  Serial.print(static_cast<unsigned long>(id));
  Serial.print(' ');
  Serial.println(reason);
}

// --- SPI communication with the shield -------------------------------------------

// Sends one 16-bit word to the shield and returns its answer.
uint16_t exchange(uint16_t word) {
  digitalWrite(SELECT_PIN, LOW);
  delayMicroseconds(100);
  const uint16_t answer = SPI.transfer16(word);
  delayMicroseconds(100);
  digitalWrite(SELECT_PIN, HIGH);
  delayMicroseconds(100);
  return answer;
}

// Sends one word to all four possible shield addresses at once.
void broadcast(uint16_t word) {
  for (uint8_t pin = 4; pin <= 7; ++pin) {
    digitalWrite(pin, LOW);
  }
  delayMicroseconds(100);
  SPI.transfer16(word);
  delayMicroseconds(100);
  for (uint8_t pin = 4; pin <= 7; ++pin) {
    digitalWrite(pin, HIGH);
  }
  delayMicroseconds(100);
}

// Checks that the shield acknowledged a command. On failure, faultReason
// contains the command and the received answers.
bool confirmed(uint16_t command, const char* reason) {
  delay(50);
  const uint16_t duringCheck = exchange(CMD_ACK_CHECK);
  const uint16_t followingRead = exchange(0);
  // Depending on the shield version, the acknowledgment arrives in the first
  // or in the second answer.
  if (duringCheck == command + 0x1000 || followingRead == command + 0x1000) {
    return true;
  }
  snprintf(spiFault, sizeof(spiFault), "%s cmd=0x%04X rx=0x%04X/0x%04X",
           reason, static_cast<unsigned int>(command),
           static_cast<unsigned int>(duringCheck), static_cast<unsigned int>(followingRead));
  faultReason = spiFault;
  return false;
}

bool setMotorSpeed(uint16_t rpm) {
  exchange(CMD_SET_SPEED);
  exchange(rpm);
  return confirmed(CMD_SET_SPEED, "SPEED_UNCONFIRMED");
}

bool stopMotor() {
  exchange(CMD_STOP);
  return confirmed(CMD_STOP, "STOP_UNCONFIRMED");
}

// Reads the shield error register. 0x0000 followed by 0x1C00 means "no error".
bool shieldHealthy() {
  const uint16_t duringRequest = exchange(CMD_CHECK_ERROR);
  const uint16_t followingRead = exchange(0);
  return duringRequest == 0 && followingRead == 0x1C00;
}

// --- Shield initialization (reset -> FOC mode -> speed) -------------------------------

void resetShield() {
  broadcast(CMD_STOP);
  delay(10);
  broadcast(0x0902);
}

// Step 1 after the reset: ask the shield to switch to FOC mode.
bool requestFoc() {
  exchange(0x0901);
  if (exchange(0) != 0x1901) {
    faultReason = "NO_SHIELD";
    return false;
  }
  exchange(0x0103);
  return true;
}

// Step 2: check that FOC mode is active, then stop the motor and set the speed.
bool completeFoc() {
  exchange(0);
  exchange(0x0110);
  if (exchange(0) != 0x1113) {
    faultReason = "FOC_INACTIVE";
    return false;
  }
  if (!stopMotor()) {
    return false;
  }
  return setMotorSpeed(MOTOR_RPM);
}

// Blocking version, only used once in setup().
bool initializeMotor() {
  for (uint8_t pin = 4; pin <= 7; ++pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

  resetShield();
  delay(RESET_WAIT_MS);
  if (!requestFoc()) {
    return false;
  }
  delay(FOC_WAIT_MS);
  return completeFoc();
}

// Non-blocking version after each run, called from loop(). Serial commands
// (PING, STOP, HELLO) are still answered during the waiting times.
void serviceRearm() {
  if (rearmStage == REARM_IDLE) {
    return;
  }
  const uint32_t waitMs = rearmStage == REARM_RESET ? RESET_WAIT_MS : FOC_WAIT_MS;
  if (millis() - rearmAt < waitMs) {
    return;
  }
  bool ok;
  if (rearmStage == REARM_RESET) {
    ok = requestFoc();
    rearmStage = REARM_FOC;
    rearmAt = millis();
  } else {
    ok = completeFoc();
    rearmStage = REARM_IDLE;
    if (ok) {
      Serial.println(F("ARMED"));
    }
  }
  if (!ok) {
    rearmStage = REARM_IDLE;
    ready = false;
    reportError(rearmId, faultReason);
  }
}

// --- Running and stopping ------------------------------------------------------------

bool startMotor() {
  exchange(0x0110);
  if (exchange(0) != 0x1113) {
    faultReason = "MODE_LOST";
    return false;
  }
  if (!setMotorSpeed(MOTOR_RPM)) {
    return false;
  }
  shieldHealthy();  // part of the tested start sequence; the result is checked after the run
  startedMs = millis();
  exchange(CMD_START);
  if (!confirmed(CMD_START, "START_UNCONFIRMED")) {
    stopMotor();
    return false;
  }
  return true;
}

// Stops the motor, reports DONE or STOPPED and starts the background rearm.
void finishMotor(const char* event) {
  if (rearmStage != REARM_IDLE) {
    // The motor is already stopped; the shield is being reset.
    reportId(event, 0);
    return;
  }
  const uint32_t id = activeId;
  const bool wasRunning = motorRunning;
  const bool stopped = stopMotor();
  motorRunning = false;
  activeId = 0;
  const bool healthy = shieldHealthy();
  if (!stopped) {
    ready = false;
    reportError(id, faultReason);
    return;
  }
  if (wasRunning && ready) {
    // Never reset a shield that reports an error: the fault stays visible.
    if (!healthy) {
      ready = false;
      faultReason = "SHIELD_STATUS_UNCONFIRMED";
      reportError(id, faultReason);
      return;
    }
    resetShield();
    rearmStage = REARM_RESET;
    rearmAt = millis();
    rearmId = id;
    Serial.println(F("REARMING"));
  }
  reportId(event, id);
}

void checkDeadline() {
  if (motorRunning && millis() - startedMs >= RUN_MS) {
    finishMotor("DONE");
  }
}

// --- Command handling -----------------------------------------------------------

// Reads a decimal job ID (1 ... 4294967295).
bool parseId(const char* text, uint32_t& id) {
  if (*text == '\0') {
    return false;
  }
  id = 0;
  while (*text) {
    if (*text < '0' || *text > '9') {
      return false;
    }
    const uint32_t digit = *text++ - '0';
    if (id > (UINT32_MAX - digit) / 10) {
      return false;
    }
    id = id * 10 + digit;
  }
  return id != 0;
}

void handleRun(uint32_t id) {
  if (!ready) {
    reportError(id, faultReason);
  } else if (rearmStage != REARM_IDLE) {
    reportError(id, "REARMING");
  } else if (motorRunning) {
    // A repeated RUN for the running job is answered again; others are refused.
    if (id == activeId) {
      reportId("STARTED", id);
    } else {
      reportError(id, "BUSY");
    }
  } else if (id == lastRunId) {
    reportError(id, "DUPLICATE");
  } else {
    lastRunId = id;
    if (!startMotor()) {
      ready = false;
      reportError(id, faultReason);
      return;
    }
    activeId = id;
    motorRunning = true;
    reportId("STARTED", id);
  }
}

void handleCommand(const char* line) {
  if (strcmp(line, "HELLO") == 0) {
    Serial.print(F("HELLO "));
    Serial.print(MOTOR_NAME);
    Serial.println(F(" 1"));
    if (!ready) {
      Serial.print(F("FAULT "));
      Serial.println(faultReason);
    } else if (motorRunning) {
      reportId("BUSY", activeId);
    } else if (rearmStage != REARM_IDLE) {
      Serial.println(F("REARMING"));
    } else {
      Serial.println(F("READY"));
    }
  } else if (strcmp(line, "PING") == 0) {
    Serial.println(F("PONG"));
  } else if (strcmp(line, "STOP") == 0) {
    finishMotor("STOPPED");
  } else if (strncmp(line, "RUN ", 4) == 0) {
    uint32_t id;
    if (parseId(line + 4, id)) {
      handleRun(id);
    } else {
      reportError(0, "INVALID_ID");
    }
  } else {
    reportError(0, "UNKNOWN_COMMAND");
  }
}

// Collects characters until '\n'. Invalid or too long lines stop the motor.
void readSerial() {
  for (uint8_t count = 0; count < 32 && Serial.available(); ++count) {
    const int value = Serial.read();
    if (value == '\r') {
      continue;
    }
    if (value == '\n') {
      if (!discardLine && inputLength != 0) {
        inputLine[inputLength] = '\0';
        handleCommand(inputLine);
      }
      inputLength = 0;
      discardLine = false;
    } else if (!discardLine) {
      if (value < 32 || value > 126 || inputLength >= LINE_SIZE - 1) {
        if (motorRunning) {
          finishMotor("STOPPED");
        }
        reportError(0, "INVALID_LINE");
        inputLength = 0;
        discardLine = true;
      } else {
        inputLine[inputLength++] = static_cast<char>(value);
      }
    }
    checkDeadline();
  }
}

// --- Arduino entry points ---------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, !XMC_LED_ON);
  ready = initializeMotor();
  if (ready) {
    faultReason = nullptr;
  }
  handleCommand("HELLO");
}

void loop() {
  checkDeadline();
  serviceRearm();
  readSerial();

  // Blinking LED = firmware is alive.
  if (millis() - lastLedMs >= 500) {
    lastLedMs = millis();
    ledOn = !ledOn;
    digitalWrite(LED1, ledOn ? XMC_LED_ON : !XMC_LED_ON);
  }
}
