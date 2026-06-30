// ===== Pin definitions =====
const int PWM_PIN = 9;         // PWM output for motor speed control
const int LED_PIN = 13;        // Built-in LED for status
const int POT_PIN = A0;        // Potentiometer analog input
const int TACH_IN_PIN = 2;     // Tachometer input from fan (must support interrupts)
const int TACH_OUT_PIN = 7;    // Tachometer output to laptop (add 1kΩ series resistor)

// ===== Mode and state variables =====
bool numericMode = true;       // true = numeric (0–4), false = potentiometer
int currentState = 0;          // 0..4 used only in numeric mode
int currentPWMValue = 0;       // tracks the actual PWM output (0–255)

// ===== Blinking control =====
unsigned long previousMillis = 0;
bool ledState = LOW;
unsigned long blinkInterval = 0;   // milliseconds for half-period

// ===== Look-up tables for numeric mode =====
const int pwmValues[5] = {0, 64, 128, 192, 255};
const unsigned long blinkIntervals[5] = {0, 1000, 500, 250, 100};

// ===== Potentiometer smoothing =====
const int POT_DEADBAND = 2;  // ignore changes smaller than this

// ===== RPM Measurement Variables =====
volatile unsigned long pulseCount = 0;   // Number of pulses counted (volatile for ISR)
unsigned long lastRPMUpdate = 0;         // Last time RPM was calculated
const unsigned long RPM_UPDATE_INTERVAL = 500; // Calculate RPM every 500ms
int currentRPM = 0;                      // Store the calculated RPM
const int PULSES_PER_REVOLUTION = 2;     // Most PC fans generate 2 pulses per revolution

// ===== Function prototypes =====
void applyDiscreteState(int state);
void applyContinuousPWM(int pwmValue);
void readPotAndApply();
void printStatus();
unsigned long mapPWMtoBlinkInterval(int pwm);
void updateRPM();

// ===== Interrupt Service Routine (ISR) =====
void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);
  
  numericMode = false;

  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TACH_IN_PIN, INPUT_PULLUP);   // Pull-up to 5V (only if fan has NO pull-up to 12V)
  pinMode(TACH_OUT_PIN, OUTPUT);
  digitalWrite(TACH_OUT_PIN, HIGH);     // Default high (idle), matches open-drain idle state

  // Attach interrupt to count pulses on the tachometer pin (FALLING edge)
  attachInterrupt(digitalPinToInterrupt(TACH_IN_PIN), countPulse, FALLING);

  // Start with numeric mode, state 0
  applyDiscreteState(0);

  Serial.println("=== PWM Control: Numeric / Potentiometer with RPM & Tach Output ===");
  Serial.println("Default mode: Numeric (N)");
  Serial.println("Commands:");
  Serial.println("  0-4   : set discrete speed (numeric mode only)");
  Serial.println("  V/v   : switch to potentiometer (continuous) mode");
  Serial.println("  N/n   : switch back to numeric mode");
  Serial.println("Tachometer signal from fan is replicated on pin 7 for laptop.");
  printStatus();
}

void loop() {
  // ----- 1. Process serial commands -----
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == 'V' || input == 'v') {
      if (!numericMode) {
        Serial.println("⚠️ Already in potentiometer mode.");
      } else {
        numericMode = false;
        Serial.println("🔄 Switched to potentiometer (continuous) mode.");
        readPotAndApply();
        printStatus();
      }
    }
    else if (input == 'N' || input == 'n') {
      if (numericMode) {
        Serial.println("⚠️ Already in numeric mode.");
      } else {
        numericMode = true;
        Serial.println("🔄 Switched to numeric (discrete) mode.");
        applyDiscreteState(currentState);
        printStatus();
      }
    }
    else if (input >= '0' && input <= '4' && numericMode) {
      int newState = input - '0';
      if (newState != currentState) {
        applyDiscreteState(newState);
        Serial.println("---------------------------");
        Serial.print("✅ Numeric command received, new state: ");
        Serial.println(currentState);
        printStatus();
      }
    }
    else {
      if (!numericMode && input >= '0' && input <= '4') {
        Serial.println("❌ Numeric commands are ignored in potentiometer mode. Use the pot.");
      } else {
        Serial.print("❌ Invalid command: ");
        Serial.println(input);
        Serial.println("Allowed: 0-4, V, N");
      }
    }
    while (Serial.available() > 0) Serial.read(); // flush buffer
  }

  // ----- 2. Potentiometer mode: continuous reading -----
  if (!numericMode) {
    readPotAndApply();
  }

  // ----- 3. Blink the LED according to current PWM -----
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

  // ----- 4. Replicate tach signal to laptop -----
  // Read the actual pin state (HIGH/LOW) and output exactly the same.
  bool tachState = digitalRead(TACH_IN_PIN);
  digitalWrite(TACH_OUT_PIN, tachState);

  // ----- 5. Update and display RPM periodically -----
  updateRPM();
}

// ===== Function to calculate RPM =====
void updateRPM() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastRPMUpdate >= RPM_UPDATE_INTERVAL) {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    // RPM formula:
    // (pulses / PULSES_PER_REVOLUTION) * (60000 ms / interval ms)
    currentRPM = (count * 60000) / (PULSES_PER_REVOLUTION * RPM_UPDATE_INTERVAL);

    Serial.print("⚡ Fan Speed: ");
    Serial.print(currentRPM);
    Serial.println(" RPM");

    lastRPMUpdate = currentMillis;
  }
}

// ----- Apply discrete state (0-4) -----
void applyDiscreteState(int state) {
  currentState = state;
  int pwm = pwmValues[state];
  currentPWMValue = pwm;
  analogWrite(PWM_PIN, pwm);
  blinkInterval = blinkIntervals[state];
}

// ----- Apply continuous PWM (0-255) -----
void applyContinuousPWM(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  currentPWMValue = pwmValue;
  analogWrite(PWM_PIN, pwmValue);
  blinkInterval = mapPWMtoBlinkInterval(pwmValue);
}

// ----- Read potentiometer and update if changed -----
void readPotAndApply() {
  int raw = analogRead(POT_PIN);
  int newPWM = map(raw, 0, 1023, 0, 255);
  if (abs(newPWM - currentPWMValue) > POT_DEADBAND) {
    applyContinuousPWM(newPWM);
    Serial.println("---------------------------");
    Serial.print("🔹 Potentiometer changed, PWM set to: ");
    Serial.print(newPWM);
    Serial.print(" (");
    Serial.print((newPWM * 100) / 255);
    Serial.print("%) | Blink interval: ");
    Serial.print(blinkInterval);
    Serial.println(" ms");
  }
}

// ----- Map PWM (0-255) to blink half-period in ms -----
unsigned long mapPWMtoBlinkInterval(int pwm) {
  if (pwm == 0) return 0;
  long mapped = map(pwm, 1, 255, 1000, 50);
  return constrain(mapped, 50, 1000);
}

// ----- Print status to Serial -----
void printStatus() {
  Serial.print("  📊 Mode: ");
  Serial.print(numericMode ? "Numeric (discrete)" : "Potentiometer (continuous)");
  if (numericMode) {
    Serial.print(" | State: ");
    Serial.print(currentState);
    Serial.print(" | PWM: ");
    Serial.print(pwmValues[currentState]);
    Serial.print(" (");
    Serial.print((pwmValues[currentState] * 100) / 255);
    Serial.print("%) | Blink: ");
    if (currentState == 0) Serial.println("OFF");
    else {
      Serial.print(blinkIntervals[currentState]);
      Serial.println(" ms");
    }
  } else {
    Serial.print(" | PWM: ");
    Serial.print(currentPWMValue);
    Serial.print(" (");
    Serial.print((currentPWMValue * 100) / 255);
    Serial.print("%) | Blink: ");
    if (currentPWMValue == 0) Serial.println("OFF");
    else {
      Serial.print(blinkInterval);
      Serial.println(" ms");
    }
  }
}