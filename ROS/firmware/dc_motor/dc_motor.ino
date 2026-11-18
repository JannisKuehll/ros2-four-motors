// DC motor firmware for the XMC4700 Relax Kit with the BTN9970LV motor shield.
//
// The board waits for text commands from the ROS bridge over USB (115200 baud):
//   HELLO    -> "HELLO dc 1" and READY / BUSY <id> / FAULT <reason>
//   PING     -> PONG
//   RUN <id> -> STARTED <id>, the motor runs for 2 s, then DONE <id>
//   STOP     -> STOPPED <id>
// The board stops the motor by itself after 2 s, even if the PC is gone.

#include <Arduino.h>

#if !defined(XMC4700_Relax_Kit)
#error Select KIT_XMC47_RELAX for this DC firmware.
#endif

const char MOTOR_NAME[] = "dc";
const uint32_t RUN_MS = 2000;

// BTN9970LV half-bridges: EN switches the bridge on, IN sets the direction.
const uint8_t EN_OUT1 = 3;
const uint8_t EN_OUT2 = 13;
const uint8_t IN_OUT1 = 10;
const uint8_t IN_OUT2 = 9;

bool ready = false;                       // false = fault, no RUN accepted
const char* faultReason = "NOT_INITIALIZED";
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

// --- Motor control --------------------------------------------------------------

bool stopMotor() {
  digitalWrite(EN_OUT1, LOW);
  digitalWrite(EN_OUT2, LOW);
  return true;
}

bool initializeMotor() {
  pinMode(EN_OUT1, OUTPUT);
  pinMode(EN_OUT2, OUTPUT);
  stopMotor();
  // OUT1 high, OUT2 low: the motor always turns in the same direction.
  pinMode(IN_OUT1, OUTPUT);
  pinMode(IN_OUT2, OUTPUT);
  digitalWrite(IN_OUT1, HIGH);
  digitalWrite(IN_OUT2, LOW);
  return true;
}

bool startMotor() {
  // Full supply voltage, no PWM.
  digitalWrite(EN_OUT1, HIGH);
  digitalWrite(EN_OUT2, HIGH);
  return true;
}

// Stops the motor and reports DONE or STOPPED with the job ID.
void finishMotor(const char* event) {
  const uint32_t id = activeId;
  const bool stopped = stopMotor();
  motorRunning = false;
  activeId = 0;
  if (!stopped) {
    ready = false;
    reportError(id, faultReason);
    return;
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
    startedMs = millis();
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
  readSerial();

  // Blinking LED = firmware is alive.
  if (millis() - lastLedMs >= 500) {
    lastLedMs = millis();
    ledOn = !ledOn;
    digitalWrite(LED1, ledOn ? XMC_LED_ON : !XMC_LED_ON);
  }
}
