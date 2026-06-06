#include <FlexCAN_T4.h>
#include <math.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// ======================================================
// CAN IDs
// ======================================================

constexpr uint16_t ID_JETSON_HEARTBEAT = 0x100;
constexpr uint16_t ID_MOTION_COMMAND   = 0x110;
constexpr uint16_t ID_ENABLE_COMMAND   = 0x120;
constexpr uint16_t ID_POSITION_COMMAND = 0x130;
constexpr uint16_t ID_STOP_COMMAND     = 0x140;
constexpr uint16_t ID_HOME_COMMAND     = 0x150;

constexpr uint16_t ID_INOS_HEARTBEAT   = 0x200;
constexpr uint16_t ID_INOS_STATUS      = 0x210;
constexpr uint16_t ID_INOS_POSITION    = 0x220;   // NEW: position report frame
constexpr uint16_t ID_INOS_FAULT       = 0x230;

// ======================================================
// STATES
// ======================================================

enum BoardState : uint8_t {
  STATE_BOOT    = 0,
  STATE_IDLE    = 1,
  STATE_ENABLED = 2,
  STATE_MOVING  = 3,
  STATE_HOMING  = 4,
  STATE_FAULT   = 5
};

enum FaultCode : uint8_t {
  FAULT_NONE               = 0,
  FAULT_HEARTBEAT_TIMEOUT  = 1,
  FAULT_LIMIT_HIT          = 2,
  FAULT_ESTOP_ACTIVE       = 3,
  FAULT_DRIVER_DISABLED    = 4,
  FAULT_HOMING_FAILED      = 5,
  FAULT_UNKNOWN_COMMAND    = 6,
  FAULT_AXIS_BUSY          = 7,
  FAULT_HOME_NOT_SUPPORTED = 8
};

enum MotionMode : uint8_t {
  MODE_IDLE     = 0,
  MODE_VELOCITY = 1,
  MODE_POSITION = 2,
  MODE_HOMING   = 3
};

// ======================================================
// HARDWARE PIN MAP
// Axis numbering: 1–6 (index 0 unused throughout)
// ======================================================

constexpr uint8_t AXIS_COUNT = 6;

constexpr int LED_PIN        = 13;
constexpr int MOTION_ALLOWED = 2;    // GLOBAL_EN_N control

constexpr int STEP_PINS[AXIS_COUNT + 1] = {
  -1,
  14,   // Axis 1 / J7  / STEP D14
  16,   // Axis 2 / J16 / STEP D16
  15,   // Axis 3 / J12 / STEP D15
  17,   // Axis 4 / J6  / STEP D17
  18,   // Axis 5 / J13 / STEP D18
  19    // Axis 6 / J20 / STEP D19
};

constexpr int DIR_PINS[AXIS_COUNT + 1] = {
  -1,
  24,   // Axis 1 / J7  / DIR D24
  26,   // Axis 2 / J16 / DIR D26
  25,   // Axis 3 / J12 / DIR D25
  27,   // Axis 4 / J6  / DIR D27
  28,   // Axis 5 / J13 / DIR D28
  29    // Axis 6 / J20 / DIR D29
};

constexpr int SENSOR_PINS[AXIS_COUNT + 1] = {
  -1,
  34,   // Axis 1 / J8  / LIM5 D34
  32,   // Axis 2 / J14 / LIM3 D32
  33,   // Axis 3 / J10 / LIM4 D33
  30,   // Axis 4 / J21 / LIM1 D30
  35,   // Axis 5 / J4  / LIM6 D35
  -1    // Axis 6 / no sensor yet
};

// Confirmed homing directions
constexpr uint8_t HOME_DIR[AXIS_COUNT + 1] = {
  0,
  0,    // Axis 1
  1,    // Axis 2
  0,    // Axis 3
  0,    // Axis 4
  0,    // Axis 5
  0     // Axis 6 unused
};

constexpr bool AXIS_HAS_SENSOR[AXIS_COUNT + 1] = {
  false,
  true,   // Axis 1
  true,   // Axis 2
  true,   // Axis 3
  true,   // Axis 4
  true,   // Axis 5
  false   // Axis 6 — no sensor installed yet
};

// Proximity adapter: no metal = LOW, screw/metal detected = HIGH
constexpr bool SENSOR_ACTIVE_LEVEL = HIGH;

// ======================================================
// SYSTEM SETTINGS
// ======================================================

constexpr uint32_t CAN_BAUDRATE                = 500000;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS        = 500;
constexpr uint32_t TX_HEARTBEAT_PERIOD_MS      = 100;
constexpr uint32_t TX_STATUS_PERIOD_MS         = 200;
constexpr uint32_t TX_POSITION_PERIOD_MS       = 200;   // NEW: position report cadence
constexpr uint16_t MAX_ALLOWED_SPEED_STEPS_SEC = 1000;

constexpr float    ACCEL_STEPS_PER_SEC2        = 300.0f;
constexpr uint32_t RAMP_UPDATE_MS              = 10;

// Minimum speed floor in position mode.
// Prevents stall near target but causes up to ~1-step overshoot at low speed.
// Acceptable for current bring-up; revisit when precise positioning is needed.
constexpr float POSITION_MIN_SPEED = 20.0f;

// ======================================================
// FORWARD DECLARATIONS
// Needed because setFault / startMotion reference functions
// defined later in the file.
// ======================================================

void sendFault(FaultCode code);
void applyEnableState();
void stopMotionOutput();

// ======================================================
// GLOBAL STATE
// ======================================================

bool jetsonAlive     = false;
bool systemEnabled   = false;
bool motionCommanded = false;
bool faultActive     = false;

BoardState currentState = STATE_BOOT;
FaultCode  currentFault = FAULT_NONE;
MotionMode motionMode   = MODE_IDLE;

uint8_t activeAxis      = 0;
uint8_t lastAxisCommand = 0;
uint8_t dirCommand      = 0;

uint16_t targetSpeed  = 0;
float    currentSpeed = 0.0f;

// Per-axis position counters (steps from home, signed)
int32_t currentPositionSteps[AXIS_COUNT + 1] = {0};

uint32_t remainingSteps = 0;
uint32_t commandedSteps = 0;

bool          stepOutputState  = false;
elapsedMicros stepTimer;
uint32_t      stepHalfPeriodUs = 0;

elapsedMillis heartbeatTimer;
elapsedMillis txHeartbeatTimer;
elapsedMillis txStatusTimer;
elapsedMillis txPositionTimer;   // NEW
elapsedMillis rampTimer;

uint8_t heartbeatCounter = 0;
uint8_t statusCounter    = 0;
uint8_t positionCounter  = 0;   // NEW

// NEW: tracks which axis to report position for in the rotating report
uint8_t positionReportAxis = 1;

// ======================================================
// HELPERS
// ======================================================

bool validAxis(uint8_t axis) {
  return axis >= 1 && axis <= AXIS_COUNT;
}

bool sensorActive(uint8_t axis) {
  if (!validAxis(axis))          return false;
  if (!AXIS_HAS_SENSOR[axis])    return false;
  return digitalRead(SENSOR_PINS[axis]) == SENSOR_ACTIVE_LEVEL;
}

uint8_t readSensorBitmask() {
  uint8_t bits = 0;
  for (uint8_t axis = 1; axis <= AXIS_COUNT; axis++) {
    if (AXIS_HAS_SENSOR[axis] && sensorActive(axis)) {
      bits |= (1 << (axis - 1));
    }
  }
  return bits;
}

// Force all STEP pins LOW except the active axis.
// GLOBAL_EN_N enables every connected TMC2209 simultaneously,
// so any floating STEP pin can cause unintended movement.
void lockInactiveAxes() {
  for (uint8_t axis = 1; axis <= AXIS_COUNT; axis++) {
    if (axis != activeAxis) {
      digitalWrite(STEP_PINS[axis], LOW);
    }
  }
}

void updateStepTimingFromCurrentSpeed() {
  if (currentSpeed <= 0.5f || activeAxis == 0) {
    stepHalfPeriodUs = 0;
    stepOutputState  = false;
    if (validAxis(activeAxis)) {
      digitalWrite(STEP_PINS[activeAxis], LOW);
    }
    return;
  }

  uint32_t fullPeriodUs = (uint32_t)(1000000.0f / currentSpeed);
  if (fullPeriodUs < 2) fullPeriodUs = 2;

  stepHalfPeriodUs = fullPeriodUs / 2;
  if (stepHalfPeriodUs < 1) stepHalfPeriodUs = 1;
}

// ======================================================
// MOTION STOP / FAULT / ENABLE
// ======================================================

void stopMotionOutput() {
  if (validAxis(activeAxis)) {
    digitalWrite(STEP_PINS[activeAxis], LOW);
  }

  motionCommanded  = false;
  motionMode       = MODE_IDLE;
  targetSpeed      = 0;
  currentSpeed     = 0.0f;
  stepHalfPeriodUs = 0;
  stepOutputState  = false;
  remainingSteps   = 0;
  commandedSteps   = 0;
  activeAxis       = 0;

  lockInactiveAxes();

  if      (faultActive)                        currentState = STATE_FAULT;
  else if (systemEnabled && jetsonAlive)        currentState = STATE_ENABLED;
  else                                          currentState = STATE_IDLE;
}

void setFault(FaultCode code) {
  faultActive    = true;
  currentFault   = code;
  currentState   = STATE_FAULT;
  systemEnabled  = false;

  stopMotionOutput();
  digitalWrite(MOTION_ALLOWED, LOW);

  Serial.print("FAULT SET: ");
  Serial.println((uint8_t)code);
}

void clearFault() {
  faultActive   = false;
  currentFault  = FAULT_NONE;
  systemEnabled = false;

  stopMotionOutput();
  digitalWrite(MOTION_ALLOWED, LOW);
  currentState = STATE_IDLE;

  Serial.println("FAULT CLEARED");
}

void applyEnableState() {
  if (systemEnabled && jetsonAlive && !faultActive) {
    digitalWrite(MOTION_ALLOWED, HIGH);

    if      (motionMode == MODE_HOMING && motionCommanded) currentState = STATE_HOMING;
    else if (motionCommanded && currentSpeed > 0.5f)       currentState = STATE_MOVING;
    else                                                   currentState = STATE_ENABLED;
  } else {
    digitalWrite(MOTION_ALLOWED, LOW);
    stopMotionOutput();
    if (!faultActive) currentState = STATE_IDLE;
  }
}

// ======================================================
// POSITION RAMP HELPER
// ======================================================

float computePositionDesiredSpeed(uint32_t stepsLeft) {
  float distance      = (float)stepsLeft;
  float maxSafeSpeed  = sqrtf(2.0f * ACCEL_STEPS_PER_SEC2 * distance);
  float commandedMax  = (float)targetSpeed;
  return (maxSafeSpeed < commandedMax) ? maxSafeSpeed : commandedMax;
}

// ======================================================
// MOTION START
// ======================================================

void startMotion(uint8_t axis, MotionMode mode, uint8_t dir, uint16_t speed, uint32_t steps) {
  if (!validAxis(axis)) {
    sendFault(FAULT_UNKNOWN_COMMAND);
    return;
  }

  if (activeAxis != 0 && activeAxis != axis) {
    Serial.println("Command ignored: another axis already active");
    sendFault(FAULT_AXIS_BUSY);
    return;
  }

  if (!jetsonAlive || faultActive || !systemEnabled) {
    Serial.println("Motion ignored: heartbeat missing, fault active, or system disabled");
    return;
  }

  if (speed > MAX_ALLOWED_SPEED_STEPS_SEC) speed = MAX_ALLOWED_SPEED_STEPS_SEC;

  if (speed == 0) {
    stopMotionOutput();
    applyEnableState();
    return;
  }

  activeAxis      = axis;
  lastAxisCommand = axis;
  dirCommand      = dir ? 1 : 0;

  digitalWrite(DIR_PINS[axis], dirCommand ? HIGH : LOW);

  motionMode       = mode;
  motionCommanded  = true;
  targetSpeed      = speed;
  currentSpeed     = 0.0f;
  stepHalfPeriodUs = 0;
  stepOutputState  = false;
  stepTimer        = 0;

  if (mode == MODE_POSITION) {
    remainingSteps = steps;
    commandedSteps = steps;
  } else {
    remainingSteps = 0;
    commandedSteps = 0;
  }

  digitalWrite(STEP_PINS[axis], LOW);
  lockInactiveAxes();
  applyEnableState();
}

// ======================================================
// SPEED RAMP UPDATE
// ======================================================

void updateSpeedRamp() {
  if (rampTimer < RAMP_UPDATE_MS) return;

  float dt = (float)rampTimer / 1000.0f;
  rampTimer = 0;

  float speedDelta = ACCEL_STEPS_PER_SEC2 * dt;

  if (!systemEnabled || !jetsonAlive || faultActive || !motionCommanded || activeAxis == 0) {
    currentSpeed = (currentSpeed > speedDelta) ? currentSpeed - speedDelta : 0.0f;
    updateStepTimingFromCurrentSpeed();
    return;
  }

  float desiredSpeed = 0.0f;

  if (motionMode == MODE_VELOCITY || motionMode == MODE_HOMING) {
    desiredSpeed = (float)targetSpeed;

  } else if (motionMode == MODE_POSITION) {
    if (remainingSteps == 0) {
      stopMotionOutput();
      updateStepTimingFromCurrentSpeed();
      return;
    }

    desiredSpeed = computePositionDesiredSpeed(remainingSteps);

    // Floor prevents stall near target.
    // NOTE: causes up to ~1-step overshoot. Revisit for precision positioning.
    if (desiredSpeed < POSITION_MIN_SPEED) {
      desiredSpeed = POSITION_MIN_SPEED;
      if (desiredSpeed > (float)targetSpeed) desiredSpeed = (float)targetSpeed;
    }
  }

  if      (currentSpeed < desiredSpeed) { currentSpeed += speedDelta; if (currentSpeed > desiredSpeed) currentSpeed = desiredSpeed; }
  else if (currentSpeed > desiredSpeed) { currentSpeed -= speedDelta; if (currentSpeed < desiredSpeed) currentSpeed = desiredSpeed; }

  updateStepTimingFromCurrentSpeed();

  if      (motionMode == MODE_HOMING && motionCommanded)  currentState = STATE_HOMING;
  else if (motionCommanded && currentSpeed > 0.5f)        currentState = STATE_MOVING;
  else if (systemEnabled && jetsonAlive && !faultActive)  currentState = STATE_ENABLED;
}

// ======================================================
// STEP GENERATOR
// ======================================================

void runMotionGenerator() {
  if (!motionCommanded)        return;
  if (!systemEnabled)          return;
  if (faultActive)             return;
  if (!jetsonAlive)            return;
  if (!validAxis(activeAxis))  return;
  if (stepHalfPeriodUs == 0)   return;

  if (stepTimer >= stepHalfPeriodUs) {
    stepTimer      = 0;
    stepOutputState = !stepOutputState;

    digitalWrite(STEP_PINS[activeAxis], stepOutputState);

    if (stepOutputState) {
      // Update signed position counter.
      // dirCommand=1 → positive direction, dirCommand=0 → negative.
      // Physical sign convention should be verified per axis after homing.
      if (dirCommand) currentPositionSteps[activeAxis]++;
      else            currentPositionSteps[activeAxis]--;

      if (motionMode == MODE_POSITION) {
        if (remainingSteps > 0) remainingSteps--;

        if (remainingSteps == 0) {
          stopMotionOutput();
          applyEnableState();
        }
      }
    }
  }
}

// ======================================================
// LIMIT / HOMING INPUT CHECK
// ======================================================

void checkLimitInputs() {
  if (!motionCommanded)              return;
  if (motionMode != MODE_HOMING)     return;
  if (!validAxis(activeAxis))        return;
  if (!AXIS_HAS_SENSOR[activeAxis])  return;

  if (sensorActive(activeAxis)) {
    Serial.print("AXIS ");
    Serial.print(activeAxis);
    Serial.println(" SENSOR ACTIVE -> homing complete, position zeroed");

    currentPositionSteps[activeAxis] = 0;
    stopMotionOutput();
    applyEnableState();
  }
}

// ======================================================
// LED BLINKER
// ======================================================

void blinkLed() {
  static uint32_t lastBlink = 0;
  static bool     ledState  = false;

  uint32_t interval = 600;
  if      (faultActive)                          interval = 100;
  else if (!jetsonAlive)                         interval = 300;
  else if (currentState == STATE_HOMING)         interval = 150;
  else if (systemEnabled)                        interval = 1000;

  if (millis() - lastBlink >= interval) {
    lastBlink = millis();
    ledState  = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}

// ======================================================
// CAN TX
// ======================================================

void sendHeartbeat() {
  CAN_message_t msg;
  msg.id     = ID_INOS_HEARTBEAT;
  msg.len    = 8;
  msg.buf[0] = heartbeatCounter++;
  msg.buf[1] = (uint8_t)currentState;
  msg.buf[2] = faultActive     ? 1 : 0;
  msg.buf[3] = systemEnabled   ? 1 : 0;
  msg.buf[4] = jetsonAlive     ? 1 : 0;
  msg.buf[5] = activeAxis;
  msg.buf[6] = (uint8_t)currentFault;
  msg.buf[7] = (uint8_t)motionMode;
  Can0.write(msg);
}

void sendStatus() {
  CAN_message_t msg;
  msg.id     = ID_INOS_STATUS;
  msg.len    = 8;

  uint16_t speedInt = (uint16_t)currentSpeed;

  msg.buf[0] = readSensorBitmask();
  msg.buf[1] = (uint8_t)motionMode;
  msg.buf[2] = systemEnabled ? 1 : 0;
  msg.buf[3] = activeAxis;
  msg.buf[4] = (uint8_t)(speedInt & 0xFF);
  msg.buf[5] = (uint8_t)((speedInt >> 8) & 0xFF);
  msg.buf[6] = motionCommanded ? 1 : 0;
  msg.buf[7] = statusCounter++;
  Can0.write(msg);
}

// NEW: Position report frame — rotates through axes one per TX period.
// Frame 0x220 layout:
//   buf[0]   axis number (1–6)
//   buf[1]   counter
//   buf[4:7] currentPositionSteps[axis] as int32 little-endian
//
// Report current axis FIRST, then advance — ensures Axis 1 is the
// first frame transmitted rather than being skipped on the first call.
void sendPosition() {
  int32_t pos = currentPositionSteps[positionReportAxis];

  CAN_message_t msg;
  msg.id     = ID_INOS_POSITION;
  msg.len    = 8;
  msg.buf[0] = positionReportAxis;
  msg.buf[1] = positionCounter++;
  msg.buf[2] = 0;
  msg.buf[3] = 0;
  msg.buf[4] = (uint8_t)( pos        & 0xFF);
  msg.buf[5] = (uint8_t)((pos >> 8)  & 0xFF);
  msg.buf[6] = (uint8_t)((pos >> 16) & 0xFF);
  msg.buf[7] = (uint8_t)((pos >> 24) & 0xFF);
  Can0.write(msg);

  // Advance after sending so next call reports the next axis
  positionReportAxis++;
  if (positionReportAxis > AXIS_COUNT) positionReportAxis = 1;
}

void sendFault(FaultCode code) {
  CAN_message_t msg;
  msg.id     = ID_INOS_FAULT;
  msg.len    = 8;
  msg.buf[0] = (uint8_t)code;
  msg.buf[1] = lastAxisCommand;
  msg.buf[2] = activeAxis;
  msg.buf[3] = (uint8_t)motionMode;
  msg.buf[4] = 0;
  msg.buf[5] = 0;
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can0.write(msg);
}

// ======================================================
// CAN RX HANDLERS
// ======================================================

void processHeartbeat(const CAN_message_t &msg) {
  heartbeatTimer = 0;

  if (faultActive && currentFault == FAULT_HEARTBEAT_TIMEOUT) {
    clearFault();
  }

  jetsonAlive = true;

  if (!faultActive) {
    currentState = systemEnabled ? STATE_ENABLED : STATE_IDLE;
  }
}

void processEnable(const CAN_message_t &msg) {
  uint8_t enableValue = msg.buf[0];

  Serial.print("ENABLE CMD RX: ");
  Serial.println(enableValue);

  if (enableValue == 1) {
    if (!faultActive && jetsonAlive) {
      systemEnabled = true;
    } else {
      Serial.println("Enable ignored: fault active or heartbeat missing");
    }
  } else {
    systemEnabled = false;
    stopMotionOutput();
  }

  applyEnableState();
}

void processMotion(const CAN_message_t &msg) {
  uint8_t  axis           = msg.buf[0];
  uint8_t  enable         = msg.buf[1];
  uint8_t  dir            = msg.buf[2];
  uint16_t requestedSpeed = ((uint16_t)msg.buf[5] << 8) | msg.buf[4];

  lastAxisCommand = axis;

  Serial.print("VELOCITY CMD RX axis=");
  Serial.print(axis);
  Serial.print(" enable=");
  Serial.print(enable);
  Serial.print(" dir=");
  Serial.print(dir);
  Serial.print(" speed=");
  Serial.println(requestedSpeed);

  if (!validAxis(axis)) {
    sendFault(FAULT_UNKNOWN_COMMAND);
    return;
  }

  if (enable == 0 || requestedSpeed == 0) {
    if (axis == activeAxis || activeAxis == 0) {
      stopMotionOutput();
      applyEnableState();
    }
    return;
  }

  startMotion(axis, MODE_VELOCITY, dir, requestedSpeed, 0);
}

void processPosition(const CAN_message_t &msg) {
  uint8_t  axis           = msg.buf[0];
  uint8_t  dir            = msg.buf[1];
  uint16_t requestedSpeed = ((uint16_t)msg.buf[3] << 8) | msg.buf[2];
  uint32_t steps =
    ((uint32_t)msg.buf[7] << 24) |
    ((uint32_t)msg.buf[6] << 16) |
    ((uint32_t)msg.buf[5] << 8)  |
    ((uint32_t)msg.buf[4]);

  lastAxisCommand = axis;

  Serial.print("POSITION CMD RX axis=");
  Serial.print(axis);
  Serial.print(" dir=");
  Serial.print(dir);
  Serial.print(" speed=");
  Serial.print(requestedSpeed);
  Serial.print(" steps=");
  Serial.println(steps);

  if (!validAxis(axis)) {
    sendFault(FAULT_UNKNOWN_COMMAND);
    return;
  }

  // FIXED: was silent drop. Now logs and sends fault for clarity.
  if (steps == 0) {
    Serial.print("POSITION CMD axis=");
    Serial.print(axis);
    Serial.println(" ignored: steps=0");
    sendFault(FAULT_UNKNOWN_COMMAND);
    return;
  }

  if (requestedSpeed == 0) {
    Serial.println("POSITION CMD ignored: speed=0");
    return;
  }

  startMotion(axis, MODE_POSITION, dir, requestedSpeed, steps);
}

void processHome(const CAN_message_t &msg) {
  uint8_t  axis           = msg.buf[0];
  uint8_t  dir            = msg.buf[1];
  uint16_t requestedSpeed = ((uint16_t)msg.buf[3] << 8) | msg.buf[2];

  lastAxisCommand = axis;

  Serial.print("HOME CMD RX axis=");
  Serial.print(axis);
  Serial.print(" dir=");
  Serial.print(dir);
  Serial.print(" speed=");
  Serial.println(requestedSpeed);

  if (!validAxis(axis)) {
    sendFault(FAULT_UNKNOWN_COMMAND);
    return;
  }

  if (!AXIS_HAS_SENSOR[axis]) {
    Serial.print("HOME CMD axis=");
    Serial.print(axis);
    Serial.println(" ignored: no sensor installed → FAULT_HOME_NOT_SUPPORTED");
    sendFault(FAULT_HOME_NOT_SUPPORTED);
    return;
  }

  if (activeAxis != 0 && activeAxis != axis) {
    Serial.println("HOME CMD ignored: another axis already active");
    sendFault(FAULT_AXIS_BUSY);
    return;
  }

  if (sensorActive(axis)) {
    Serial.print("Axis ");
    Serial.print(axis);
    Serial.println(" sensor already active — position set to zero, no motion needed");
    currentPositionSteps[axis] = 0;
    stopMotionOutput();
    applyEnableState();
    return;
  }

  if (requestedSpeed == 0) requestedSpeed = 100;
  if (requestedSpeed > 500) requestedSpeed = 500;

  // Use the firmware-confirmed homing direction, not the CAN-supplied dir.
  // HOME_DIR[] was validated physically per axis — trusting it prevents
  // a wrong-direction home if the sender passes an incorrect dir byte.
  startMotion(axis, MODE_HOMING, HOME_DIR[axis], requestedSpeed, 0);
}

void processStop(const CAN_message_t &msg) {
  Serial.println("STOP CMD RX");
  stopMotionOutput();
  applyEnableState();
}

// ======================================================
// CAN RX DISPATCHER
// ======================================================

void readCanMessages() {
  CAN_message_t msg;

  while (Can0.read(msg)) {
    switch (msg.id) {
      case ID_JETSON_HEARTBEAT:  processHeartbeat(msg); break;
      case ID_ENABLE_COMMAND:    processEnable(msg);    break;
      case ID_MOTION_COMMAND:    processMotion(msg);    break;
      case ID_POSITION_COMMAND:  processPosition(msg);  break;
      case ID_HOME_COMMAND:      processHome(msg);      break;
      case ID_STOP_COMMAND:      processStop(msg);      break;
      default:
        Serial.print("Unknown CAN ID: 0x");
        Serial.println(msg.id, HEX);
        break;
    }
  }
}

// ======================================================
// SAFETY
// ======================================================

void checkHeartbeatTimeout() {
  if (jetsonAlive && heartbeatTimer > HEARTBEAT_TIMEOUT_MS) {
    Serial.println("Heartbeat timeout");
    jetsonAlive = false;
    setFault(FAULT_HEARTBEAT_TIMEOUT);
    sendFault(FAULT_HEARTBEAT_TIMEOUT);
  }
}

// ======================================================
// SETUP / LOOP
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN,        OUTPUT);
  pinMode(MOTION_ALLOWED, OUTPUT);

  for (uint8_t axis = 1; axis <= AXIS_COUNT; axis++) {
    pinMode(STEP_PINS[axis], OUTPUT);
    pinMode(DIR_PINS[axis],  OUTPUT);

    digitalWrite(STEP_PINS[axis], LOW);
    digitalWrite(DIR_PINS[axis],  LOW);

    if (AXIS_HAS_SENSOR[axis]) {
      pinMode(SENSOR_PINS[axis], INPUT_PULLUP);
    }
  }

  digitalWrite(LED_PIN,        LOW);
  digitalWrite(MOTION_ALLOWED, LOW);

  lockInactiveAxes();

  Can0.begin();
  Can0.setBaudRate(CAN_BAUDRATE);

  heartbeatTimer  = 0;
  txHeartbeatTimer = 0;
  txStatusTimer   = 0;
  txPositionTimer = 0;
  rampTimer       = 0;

  currentState = STATE_IDLE;

  Serial.println("==============================================");
  Serial.println("INOS firmware v3.1.1 MULTI-AXIS");
  Serial.println("==============================================");
  Serial.println("Axis 1: J7  STEP D14 DIR D24 SENSOR D34 HOME DIR 0");
  Serial.println("Axis 2: J16 STEP D16 DIR D26 SENSOR D32 HOME DIR 1");
  Serial.println("Axis 3: J12 STEP D15 DIR D25 SENSOR D33 HOME DIR 0");
  Serial.println("Axis 4: J6  STEP D17 DIR D27 SENSOR D30 HOME DIR 0");
  Serial.println("Axis 5: J13 STEP D18 DIR D28 SENSOR D35 HOME DIR 0");
  Serial.println("Axis 6: J20 STEP D19 DIR D29 NO SENSOR / HOME DISABLED");
  Serial.println("----------------------------------------------");
  Serial.println("Only one axis moves at a time.");
  Serial.println("Unused STEP pins locked LOW (GLOBAL_EN_N safety).");
  Serial.println("Position frame: 0x220 rotating per axis.");
  Serial.println("CAN: 500 kbps");
  Serial.println("==============================================");
}

void loop() {
  lockInactiveAxes();      // Safety: keep unused STEP pins LOW every loop

  readCanMessages();
  checkHeartbeatTimeout();
  checkLimitInputs();
  updateSpeedRamp();
  runMotionGenerator();
  blinkLed();

  if (txHeartbeatTimer >= TX_HEARTBEAT_PERIOD_MS) {
    txHeartbeatTimer = 0;
    sendHeartbeat();
  }

  if (txStatusTimer >= TX_STATUS_PERIOD_MS) {
    txStatusTimer = 0;
    sendStatus();
  }

  // NEW: rotating per-axis position report
  if (txPositionTimer >= TX_POSITION_PERIOD_MS) {
    txPositionTimer = 0;
    sendPosition();
  }
}
