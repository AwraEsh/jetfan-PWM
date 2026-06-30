// ===== Pin definitions =====
const int PWM_PIN = 9;         // PWM output for motor speed control
const int LED_PIN = 13;        // Built-in LED for status
const int POT_PIN = A0;        // Potentiometer analog input (sensitivity in PC mode, PWM in manual)
const int TACH_IN_PIN = 2;     // Tachometer input from fan (must support interrupts)
const int TACH_OUT_PIN = 7;    // Tachometer output to laptop (add 1kOhm series resistor)

// ===== Mode and state variables =====
bool pcConnected = false;              // true = receiving valid PWM commands from PC
unsigned long lastPcCommand = 0;       // timestamp of last valid PC command
const unsigned long PC_TIMEOUT_MS = 3000; // fallback to manual after 3s without PC command

int currentPWMValue = 0;       // tracks the actual PWM output (0-255)
int potSensitivity = 128;      // last read pot value mapped to 0-255 (used in PC mode)

// ===== Blinking control =====
unsigned long previousMillis = 0;
bool ledState = LOW;
unsigned long blinkInterval = 0;

// ===== Look-up tables for original blink style =====
const unsigned long blinkIntervals[5] = {0, 1000, 500, 250, 100};

// ===== Potentiometer smoothing =====
const int POT_DEADBAND = 2;

// ===== RPM Measurement Variables =====
volatile unsigned long pulseCount = 0;
unsigned long lastRPMUpdate = 0;
const unsigned long RPM_UPDATE_INTERVAL = 500;
int currentRPM = 0;
const int PULSES_PER_REVOLUTION = 2;

// ===== Serial send interval =====
unsigned long lastSerialSend = 0;
const unsigned long SERIAL_SEND_INTERVAL = 300; // send pot/rpm to PC every 300ms

// ===== Serial line parser =====
char inputBuf[16];
int inputIdx = 0;

// ===== Function prototypes =====
void applyContinuousPWM(int pwmValue);
void readPotAndApply();
void updateRPM();
void sendSerialData();
void parseSerialLine();

// ===== Interrupt Service Routine (ISR) =====
void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TACH_IN_PIN, INPUT_PULLUP);
  pinMode(TACH_OUT_PIN, OUTPUT);
  digitalWrite(TACH_OUT_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(TACH_IN_PIN), countPulse, FALLING);

  // Start with pot-manual mode (safe: fan will run based on pot position)
  pcConnected = false;
  readPotAndApplyManual();

  Serial.println("=== JetFan v2 — Dual Mode (PC / Manual) ===");
  Serial.println("Mode: MANUAL (waiting for PC)");
  Serial.println("Protocol: PC sends PWM:xxx\\n, Arduino sends SENS:xxx\\n and RPM:xxx\\n");
}

void loop() {
  // ----- 1. Read serial commands from PC -----
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      inputBuf[inputIdx] = '\0';
      parseSerialLine();
      inputIdx = 0;
    } else if (inputIdx < (int)(sizeof(inputBuf) - 1)) {
      inputBuf[inputIdx++] = c;
    } else {
      // buffer full, discard partial line
      inputIdx = 0;
    }
  }

  // ----- 2. Check PC timeout -----
  if (pcConnected) {
    if (millis() - lastPcCommand >= PC_TIMEOUT_MS) {
      pcConnected = false;
      readPotAndApplyManual();
      Serial.println("MODE:MANUAL (PC timeout)");
    }
  }

  // ----- 3. Read pot and apply based on mode -----
  if (pcConnected) {
    // PC mode: read pot as sensitivity (0-255), send to PC
    int raw = analogRead(POT_PIN);
    int newSens = map(raw, 0, 1023, 0, 255);
    if (abs(newSens - potSensitivity) > POT_DEADBAND) {
      potSensitivity = newSens;
    }
  } else {
    // Manual mode: pot directly controls PWM
    readPotAndApplyManual();
  }

  // ----- 4. Blink LED according to current PWM -----
  if (currentPWMValue == 0) {
    digitalWrite(LED_PIN, LOW);
    ledState = LOW;
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }

  // ----- 5. Replicate tach signal to laptop -----
  bool tachState = digitalRead(TACH_IN_PIN);
  digitalWrite(TACH_OUT_PIN, tachState);

  // ----- 6. Update RPM and send serial data -----
  updateRPM();
  if (pcConnected) {
    sendSerialData();
  }
}

// ===== Parse a complete line from PC =====
void parseSerialLine() {
  // Expected format: "PWM:xxx" where xxx is 0-255
  if (strncmp(inputBuf, "PWM:", 4) == 0) {
    int val = atoi(inputBuf + 4);
    if (val >= 0 && val <= 255) {
      if (!pcConnected) {
        pcConnected = true;
        Serial.println("MODE:AUTO (PC connected)");
        // Read initial pot value as sensitivity
        potSensitivity = map(analogRead(POT_PIN), 0, 1023, 0, 255);
      }
      lastPcCommand = millis();
      applyContinuousPWM(val);
    }
  }
  // Other command formats can be added here in the future
}

// ===== Apply PWM value (0-255) =====
void applyContinuousPWM(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  currentPWMValue = pwmValue;
  analogWrite(PWM_PIN, pwmValue);
  blinkInterval = mapPWMtoBlinkInterval(pwmValue);
}

// ===== Manual mode: read pot and set PWM directly =====
void readPotAndApplyManual() {
  int raw = analogRead(POT_PIN);
  int newPWM = map(raw, 0, 1023, 0, 255);
  if (abs(newPWM - currentPWMValue) > POT_DEADBAND) {
    applyContinuousPWM(newPWM);
  }
}

// ===== Send pot sensitivity and RPM to PC =====
void sendSerialData() {
  unsigned long now = millis();
  if (now - lastSerialSend >= SERIAL_SEND_INTERVAL) {
    lastSerialSend = now;
    Serial.print("SENS:");
    Serial.println(potSensitivity);
    if (currentRPM > 0) {
      Serial.print("RPM:");
      Serial.println(currentRPM);
    }
  }
}

// ===== Calculate RPM from tachometer pulses =====
void updateRPM() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastRPMUpdate >= RPM_UPDATE_INTERVAL) {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    if (count > 0) {
      currentRPM = (count * 60000) / (PULSES_PER_REVOLUTION * RPM_UPDATE_INTERVAL);
    } else {
      currentRPM = 0;
    }

    lastRPMUpdate = currentMillis;
  }
}

// ===== Map PWM (0-255) to blink half-period in ms =====
unsigned long mapPWMtoBlinkInterval(int pwm) {
  if (pwm == 0) return 0;
  long mapped = map(pwm, 1, 255, 1000, 50);
  return constrain(mapped, 50, 1000);
}
