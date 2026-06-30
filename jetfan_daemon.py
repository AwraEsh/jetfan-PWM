#!/usr/bin/env python3
"""
JetFan v2 — PC-side temperature-based fan controller for Arduino JetFan.

Reads CPU/GPU temperatures, computes target PWM with asymmetric PID-like control,
and sends it to Arduino over serial. Reads sensitivity pot from Arduino.

Protocol:
  PC  -> Arduino:  PWM:xxx\n   (0-255)
  Arduino -> PC:   SENS:xxx\n   (0-255, pot sensitivity)
                   RPM:xxx\n    (fan RPM)
                   MODE:...\n   (mode change status)

Fallback: if Arduino doesn't receive commands for 3s, it falls back to
manual (pot directly controls PWM). On reconnection, PC mode resumes.
"""

import os
import sys
import time
import serial
import logging
import logging.handlers
import signal
import subprocess
import glob
from pathlib import Path
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

# ─── Configuration ────────────────────────────────────────────────────
MAX_RPM = 13500
MAX_PWM = 255
FAN_STEP_RPM = 500
FAN_STEP_PWM = max(1, int(FAN_STEP_RPM * MAX_PWM / MAX_RPM))  # ~10
STEP_INTERVAL_S = 10          # seconds between step-down attempts
STABILITY_WINDOW_S = 15       # seconds of stable temp before step-down
CYCLE_INTERVAL = 0.25         # control loop runs at ~4Hz (250ms)
GPU_THRESHOLD = 55.0          # only consider GPU above this temp

# temperature -> target PWM curve (piecewise linear)
# Format: (temp_c, pwm) tuples
TEMP_CURVE = [
    # Gentle curve: fan should be barely audible at idle
    (0,   10),     # very cold → idle murmur
    (35,  20),     # idle
    (40,  30),     # warming
    (45,  45),     # getting warm
    (50,  60),     # moderate (~3176 RPM)
    (55,  80),     # warm (~4235 RPM)
    (60,  100),    # getting hot (~5294 RPM)
    (65,  120),    # hot (~6353 RPM)
    (70,  145),    # very hot (~7676 RPM)
    (75,  175),    # critical (~9265 RPM)
    (80,  210),    # very critical (~11118 RPM)
    (85,  255),    # MAX (13500 RPM)
]

# ─── Paths ────────────────────────────────────────────────────────────
PROJECT_DIR = Path(__file__).parent.resolve()
LOG_DIR = PROJECT_DIR / "logs"
LOG_FILE = LOG_DIR / "jetfan.log"
LATEST_FILE = LOG_DIR / "jetfan-latest.txt"
PID_FILE = Path("/tmp/jetfan-daemon.pid")

DEFAULT_SENSITIVITY = 128  # mid-range when Arduino not connected
PC_TIMEOUT_S = 2.0         # if no data from Arduino for 2s, use default sens

# ─── Logging setup ────────────────────────────────────────────────────
LOG_DIR.mkdir(parents=True, exist_ok=True)

logger = logging.getLogger("jetfan")
logger.setLevel(logging.DEBUG)

# File handler (verbose, rotates at 1MB)
file_handler = logging.handlers.RotatingFileHandler(
    LOG_FILE, maxBytes=1024 * 1024, backupCount=5
)
file_handler.setLevel(logging.DEBUG)
file_fmt = logging.Formatter(
    "%(asctime)s.%(msecs)03d|%(levelname)s|%(message)s",
    datefmt="%H:%M:%S"
)
file_handler.setFormatter(file_fmt)

# Stdout handler (for journald / direct run)
stdout_handler = logging.StreamHandler(sys.stdout)
stdout_handler.setLevel(logging.INFO)
stdout_fmt = logging.Formatter("%(asctime)s|%(message)s", datefmt="%H:%M:%S")
stdout_handler.setFormatter(stdout_fmt)

logger.addHandler(file_handler)
logger.addHandler(stdout_handler)


# ─── Helpers ──────────────────────────────────────────────────────────
def interpolate_curve(value: float, curve: list) -> int:
    """Interpolate Y from curve defined as [(x0,y0), (x1,y1), ...]."""
    if value <= curve[0][0]:
        return curve[0][1]
    if value >= curve[-1][0]:
        return curve[-1][1]
    for i in range(len(curve) - 1):
        x0, y0 = curve[i]
        x1, y1 = curve[i + 1]
        if x0 <= value <= x1:
            ratio = (value - x0) / (x1 - x0) if x1 != x0 else 0
            return int(y0 + ratio * (y1 - y0))
    return curve[-1][1]


def read_cpu_temp() -> Optional[float]:
    """Read CPU package temperature from sysfs."""
    try:
        with open("/sys/class/thermal/thermal_zone8/temp") as f:
            return int(f.read().strip()) / 1000.0
    except (FileNotFoundError, ValueError, OSError):
        # Fallback: try x86_pkg_temp
        try:
            for z in Path("/sys/class/thermal").glob("thermal_zone*"):
                typ = (z / "type").read_text().strip()
                if typ == "x86_pkg_temp":
                    return int((z / "temp").read_text().strip()) / 1000.0
        except (FileNotFoundError, ValueError, OSError):
            pass
        logger.warning("Cannot read CPU temperature")
        return None


def read_gpu_temp() -> Optional[float]:
    """Read NVIDIA GPU temperature via nvidia-smi."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=temperature.gpu",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            val = result.stdout.strip()
            if val:
                return float(val)
    except (FileNotFoundError, subprocess.TimeoutExpired, ValueError, OSError):
        pass
    return None


def find_arduino_port() -> Optional[str]:
    """Find Arduino by USB vendor ID in /dev/serial/by-id/."""
    by_id = Path("/dev/serial/by-id")
    if not by_id.exists():
        return None
    for link in by_id.glob("usb-Arduino*"):
        if link.is_symlink():
            target = link.resolve()
            if target.exists():
                return str(target)
    # Fallback: try common names
    for name in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
        if Path(name).exists():
            return name
    return None


def rpm_to_pwm(rpm: int) -> int:
    """Convert target RPM to PWM assuming linear relationship."""
    return max(0, min(MAX_PWM, int(rpm * MAX_PWM / MAX_RPM)))


def pwm_to_rpm(pwm: int) -> int:
    """Convert PWM to estimated RPM (linear approximation)."""
    return int(pwm * MAX_RPM / MAX_PWM)


def read_nvidia_persistence_mode() -> bool:
    """Check if nvidia persistence mode is on (makes nvidia-smi faster)."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "-pm", "1"],
            capture_output=True, text=True, timeout=5
        )
        return result.returncode == 0
    except:
        return False


# ─── Control State ────────────────────────────────────────────────────
@dataclass
class ControlState:
    """Persistent state for the control algorithm."""
    # Smoothed / averaged values
    smoothed_temp: float = 35.0
    temp_derivative_filtered: float = 0.0  # low-pass filtered derivative

    # Derivative history for smoothing
    deriv_buffer: deque = field(default_factory=lambda: deque(maxlen=8))

    # PID state
    integral: float = 0.0
    prev_temp: float = 35.0
    prev_time: float = 0.0

    # Current outputs
    target_pwm: int = 0
    current_pwm: int = 0

    # Sensitivity from Arduino pot
    sensitivity: int = DEFAULT_SENSITIVITY

    # Latest sensor readings
    cpu_temp: float = 0.0
    gpu_temp: float = 0.0
    arduino_rpm: int = 0

    # Step-down state
    step_down_enabled: bool = True
    last_step_time: float = 0.0
    steps_taken: int = 0
    temp_at_last_step: float = 0.0
    pwm_before_stepdown: int = 0
    stepdown_active: bool = False
    consecutive_rises: int = 0

    # Timing
    last_sensor_update: float = 0.0
    last_arduino_data: float = 0.0
    serial_connected: bool = False
    loop_count: int = 0

    # Temperature history for stability check
    temp_history: deque = field(default_factory=lambda: deque(maxlen=60))
    raw_temp_history: deque = field(default_factory=lambda: deque(maxlen=8))

    # Debug record
    effective_temp: float = 0.0
    base_pwm: int = 0
    pid_adjustment: float = 0.0
    stepdown_adjustment: int = 0
    alpha: float = 0.5
    derivative_raw: float = 0.0
    prev_filtered_deriv: float = 0.0


# ─── Serial Manager ──────────────────────────────────────────────────
class SerialManager:
    """Manages serial connection to Arduino with auto-reconnect."""

    def __init__(self):
        self.port: Optional[str] = None
        self.ser: Optional[serial.Serial] = None
        self.baud = 9600
        self._buf = ""

    def connect(self) -> bool:
        """Try to connect to Arduino. Returns True on success."""
        port = find_arduino_port()
        if port is None:
            if self.ser:
                self.disconnect()
            return False

        if self.ser and self.ser.is_open and port == self.port:
            return True  # already connected

        self.disconnect()
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self.baud,
                timeout=0.2,
                write_timeout=0.2
            )
            # Flush any stale data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.port = port
            logger.info(f"Serial connected: {port}")
            return True
        except (serial.SerialException, OSError) as e:
            logger.warning(f"Serial connect failed ({port}): {e}")
            self.ser = None
            self.port = None
            return False

    def disconnect(self):
        """Close serial connection."""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.port = None

    def read_data(self) -> list:
        """Read all available lines from serial. Returns list of parsed dicts."""
        results = []
        if not self.ser or not self.ser.is_open:
            return results

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline()
                try:
                    line_str = line.decode("utf-8", errors="replace").strip()
                except:
                    continue
                if not line_str:
                    continue

                if line_str.startswith("SENS:"):
                    try:
                        val = int(line_str[5:])
                        results.append({"type": "sensitivity", "value": val})
                    except ValueError:
                        pass
                elif line_str.startswith("RPM:"):
                    try:
                        val = int(line_str[4:])
                        results.append({"type": "rpm", "value": val})
                    except ValueError:
                        pass
                elif line_str.startswith("MODE:"):
                    results.append({"type": "mode", "value": line_str[5:]})
        except (serial.SerialException, OSError) as e:
            logger.error(f"Serial read error: {e}")
            self.disconnect()
        except Exception as e:
            logger.error(f"Serial read unexpected: {e}")

        return results

    def send_pwm(self, pwm: int):
        """Send PWM command to Arduino."""
        if not self.ser or not self.ser.is_open:
            return False
        try:
            cmd = f"PWM:{pwm}\n"
            self.ser.write(cmd.encode())
            return True
        except (serial.SerialException, OSError) as e:
            logger.error(f"Serial write error: {e}")
            self.disconnect()
            return False
        except Exception as e:
            logger.error(f"Serial write unexpected: {e}")
            return False

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open


# ─── Control Algorithm ──────────────────────────────────────────────
class FanController:
    """Core fan control algorithm."""

    def __init__(self):
        self.state = ControlState()
        self.serial_mgr = SerialManager()

    def compute_alpha(self) -> float:
        """Map sensitivity 0-255 to EMA alpha 0.05-0.95."""
        sens = max(0, min(255, self.state.sensitivity))
        # sensitivity=0 → alpha=0.05 (very slow, ~19 samples to 90%)
        # sensitivity=255 → alpha=0.95 (very fast, ~1 sample to 90%)
        return 0.05 + (sens / 255.0) * 0.90

    def compute_rise_alpha(self) -> float:
        """Alpha specifically for rising temperatures (always fast)."""
        sens = max(0, min(255, self.state.sensitivity))
        # Even at minimum sensitivity, rise is at least moderately fast
        return 0.4 + (sens / 255.0) * 0.55

    def compute_fall_alpha(self) -> float:
        """Alpha for falling temperatures (asymmetric — slower fall)."""
        sens = max(0, min(255, self.state.sensitivity))
        # Fall is always slower than rise
        base = 0.02 + (sens / 255.0) * 0.50
        return min(base, self.compute_rise_alpha() * 0.6)

    def get_temp_stability(self, temp: float) -> Tuple[float, float]:
        """Check if temperature is stable. Returns (max_variation, trend)."""
        history = list(self.state.temp_history)
        if len(history) < 10:
            return 99.0, 0.0  # not enough data → unstable

        recent = history[-10:]
        max_temp = max(recent)
        min_temp = min(recent)
        variation = max_temp - min_temp

        # Trend: positive = rising, negative = falling
        trend = recent[-1] - recent[0]
        return variation, trend

    def compute_base_pwm(self, temp: float) -> int:
        """Map temperature to base PWM via curve."""
        return interpolate_curve(temp, TEMP_CURVE)

    def compute_pid_adjustment(self, temp: float, dt: float) -> float:
        """
        PID fine-tuning across ALL temperatures.

        Uses the RATE of temperature change (derivative), not absolute temp,
        so it works from 30°C to 85°C identically. A slow 0.1°C/s rise
        gets the same gentle correction whether you're at 45°C or 75°C.
        A fast 3°C/s spike always gets an aggressive response.

        The derivative is low-pass filtered (averaged over 8 samples)
        to prevent sensor noise from causing fan oscillation.

        Sens pot (0-255) scales the PID aggressiveness:
          0   → PID barely reacts, temp must drift for a while
          255 → PID reacts instantly to any temp change
        """
        if dt <= 0:
            dt = 0.1

        Kp = 0.8          # proportional gain: reacts to rate of temp change
        Ki = 0.03         # integral gain: slow drift correction
        Kd = 0.5          # derivative gain: acceleration anticipation

        s = self.state

        # ── Smoothed derivative ──
        # Average the last N derivative readings to kill sensor noise
        s.deriv_buffer.append(s.derivative_raw)
        filtered_deriv = sum(s.deriv_buffer) / max(len(s.deriv_buffer), 1)
        s.temp_derivative_filtered = filtered_deriv

        # ── P: responds to the smoothed rate of change ──
        # +0.5°C/s → small +PWM. +3°C/s → big +PWM.
        P = Kp * filtered_deriv

        # ── I: catches slow drift ──
        # Integrate filtered derivative so sustained drift accumulates
        s.integral += filtered_deriv * dt * 0.3
        s.integral = max(-10, min(10, s.integral))  # anti-windup
        I = Ki * s.integral

        # ── D: acceleration (anticipation) ──
        # Is the temp rise accelerating or slowing down?
        acceleration = (filtered_deriv - s.prev_filtered_deriv) / max(dt, 0.1)
        s.prev_filtered_deriv = filtered_deriv
        D = Kd * acceleration * 0.5  # dampened

        # ── Rapid rise boost (only for real fast spikes) ──
        rise_boost = 0.0
        if filtered_deriv > 1.5:
            rise_boost = filtered_deriv * 4.0
            logger.debug(f"BOOST: dT/dt={filtered_deriv:.2f}C/s, boost={rise_boost:.0f}")

        # ── Scale by sensitivity ──
        # At sens=0: PID effect = 0.2x (barely any). At sens=255: 1.2x
        sens_factor = 0.2 + (s.sensitivity / 255.0) * 1.0

        adjustment = (P + I + D + rise_boost) * sens_factor
        return adjustment

    def step_down_cycle(self, temp: float, base_pwm: int) -> Tuple[int, int]:
        """
        When temp < 50°C and stable, gradually reduce PWM to find
        minimum viable fan speed. Returns (final_pwm, stepdown_adjustment).
        """
        s = self.state
        now = time.monotonic()

        # Step-down only kicks in below 50°C
        if temp >= 50.0:
            s.step_down_enabled = True
            s.stepdown_active = False
            s.steps_taken = 0
            return base_pwm, 0

        # Check stability
        variation, trend = self.get_temp_stability(temp)

        # If temp is rising, disable step-down temporarily
        if trend > 0.5:
            s.stepdown_active = False
            s.consecutive_rises += 1
            # Reverse some steps if temp keeps rising
            if s.consecutive_rises >= 2 and s.steps_taken > 0:
                s.steps_taken = max(0, s.steps_taken - 1)
                s.last_step_time = now
                logger.debug(f"STEP-DOWN: reversed 1 step due to rising temp (trend={trend:.1f})")
            return base_pwm, 0
        else:
            s.consecutive_rises = 0

        # Need stability to start stepping
        if variation > 1.5:
            s.stepdown_active = False
            return base_pwm, 0

        # Enough data collected, start stepping
        if not s.stepdown_active:
            s.stepdown_active = True
            s.pwm_before_stepdown = base_pwm
            s.temp_at_last_step = temp
            s.last_step_time = now
            s.steps_taken = 0
            logger.info(f"STEP-DOWN: starting (temp={temp:.1f}C, base_pwm={base_pwm})")
            return base_pwm, 0

        # Active step-down: check if enough time passed since last step
        elapsed = now - s.last_step_time
        if elapsed < STEP_INTERVAL_S:
            # Still waiting between steps
            pass

        # Check if temp rose since last step
        if s.steps_taken > 0 and (temp - s.temp_at_last_step) > 0.8:
            # Temp went up! Reverse this step
            s.steps_taken = max(0, s.steps_taken - 1)
            s.last_step_time = now
            s.temp_at_last_step = temp
            logger.debug(f"STEP-DOWN: reversed (temp rose {temp - s.temp_at_last_step:.1f}C)")
        elif elapsed >= STEP_INTERVAL_S:
            # Time to try another step
            s.steps_taken += 1
            s.last_step_time = now
            s.temp_at_last_step = temp
            logger.debug(f"STEP-DOWN: step {s.steps_taken} (reducing by {FAN_STEP_PWM} PWM)")

        adjustment = -(s.steps_taken * FAN_STEP_PWM)
        final_pwm = max(base_pwm + adjustment, 20)  # never go below minimum
        logger.debug(f"STEPDOWN: steps={s.steps_taken}, adjustment={adjustment}, final={final_pwm}")
        return final_pwm, adjustment

    def update(self) -> Tuple[int, int]:
        """
        Main control update. Returns (target_pwm, effective_temp).
        Both PWM values are 0-255.
        """
        s = self.state
        now = time.monotonic()
        dt = now - s.prev_time if s.prev_time > 0 else CYCLE_INTERVAL
        s.prev_time = now
        s.loop_count += 1

        # ── 1. Read sensors ──
        cpu_temp = read_cpu_temp()
        gpu_temp = read_gpu_temp()

        if cpu_temp is not None:
            s.cpu_temp = cpu_temp
        if gpu_temp is not None:
            s.gpu_temp = gpu_temp

        # ── 2. Determine effective temperature ──
        gpu_effective = s.gpu_temp if s.gpu_temp > GPU_THRESHOLD else 0
        s.effective_temp = max(s.cpu_temp, gpu_effective)
        temp = s.effective_temp

        # ── 3. Record temperature history ──
        s.temp_history.append(temp)

        # ── 4. Raw derivative (before filtering) ──
        s.raw_temp_history.append(temp)
        if len(s.raw_temp_history) >= 3:
            recent = list(s.raw_temp_history)
            dt_raw = (len(recent) - 1) * CYCLE_INTERVAL
            s.derivative_raw = (recent[-1] - recent[0]) / max(dt_raw, 0.1)
        else:
            s.derivative_raw = 0.0
        s.prev_temp = temp

        # ── 5. Compute smoothing alpha based on sensitivity ──
        s.alpha = self.compute_alpha()
        if temp > s.smoothed_temp:
            # Rising: use rise alpha (faster)
            s.alpha = self.compute_rise_alpha()
        elif temp < s.smoothed_temp:
            # Falling: use fall alpha (slower)
            s.alpha = self.compute_fall_alpha()

        # Apply smoothing
        s.smoothed_temp = s.alpha * temp + (1 - s.alpha) * s.smoothed_temp

        # ── 6. Base PWM from temperature curve ──
        s.base_pwm = self.compute_base_pwm(s.smoothed_temp)

        # ── 7. PID adjustment over the full temperature range ──
        s.pid_adjustment = self.compute_pid_adjustment(s.smoothed_temp, dt)

        # ── 8. Emergency: temp > 85 → max ──
        if s.smoothed_temp >= 85 or temp >= 85:
            s.target_pwm = MAX_PWM
            s.current_pwm = MAX_PWM
            s.stepdown_active = False
            logger.debug(f"EMERGENCY MAX: temp={temp:.1f}C / smoothed={s.smoothed_temp:.1f}C")
            return s.current_pwm, s.effective_temp

        # ── 9. Combine base + PID ──
        combined = s.base_pwm + int(s.pid_adjustment)
        combined = max(20, min(MAX_PWM, combined))

        # ── 10. Gradual step-down when stable below 50°C ──
        if s.step_down_enabled:
            stepped_pwm, step_adj = self.step_down_cycle(s.smoothed_temp, combined)
            s.stepdown_adjustment = step_adj
            combined = stepped_pwm

        s.target_pwm = max(20, min(MAX_PWM, combined))

        # ── 11. Apply asymmetric response to actual output ──
        # (Fast up, slow down — second layer beyond EMA)
        rise_alpha = 0.8
        fall_alpha = self.compute_fall_alpha()

        if s.target_pwm > s.current_pwm:
            # Rising: fast
            s.current_pwm = int(rise_alpha * s.target_pwm + (1 - rise_alpha) * s.current_pwm)
        elif s.target_pwm < s.current_pwm:
            # Falling: slow (unless emergency)
            if s.smoothed_temp >= 85:
                s.current_pwm = s.target_pwm  # instant drop from max
            else:
                s.current_pwm = int(fall_alpha * s.target_pwm + (1 - fall_alpha) * s.current_pwm)

        s.current_pwm = max(0, min(MAX_PWM, s.current_pwm))

        return s.current_pwm, s.effective_temp


# ─── Main Daemon ─────────────────────────────────────────────────────
class JetFanDaemon:
    """Main daemon orchestrating serial, sensors, and control."""

    def __init__(self):
        self.controller = FanController()
        self.serial = self.controller.serial_mgr
        self.running = True
        self.arduino_seen = False
        self.last_log_time = 0.0
        self.log_interval = 1.0  # log to file every 1s
        self.last_status_display = 0.0

        # Setup signal handlers
        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT, self._handle_signal)

    def _handle_signal(self, signum, frame):
        logger.info(f"Received signal {signum}, shutting down...")
        self.running = False

    def write_latest_status(self, line: str):
        """Write one-line status to a file for easy reading."""
        try:
            LATEST_FILE.write_text(line[:200])
        except OSError:
            pass

    def format_status(self, pwm: int, eff_temp: float) -> str:
        """Format one-line status string."""
        s = self.controller.state
        rpm_est = pwm_to_rpm(pwm)
        arpm = s.arduino_rpm
        return (
            f"CPU:{s.cpu_temp:.1f}C "
            f"GPU:{s.gpu_temp:.0f}C "
            f"MAX:{eff_temp:.1f}C "
            f"SM:{s.smoothed_temp:.1f}C "
            f"PWM:{pwm} ({pwm*100//MAX_PWM}%) "
            f"RPM:{rpm_est}/{arpm} "
            f"SENS:{s.sensitivity} "
            f"STP:{'ON' if s.stepdown_active else 'OFF'} "
            f"STEP:{s.steps_taken} "
            f"PID:{s.pid_adjustment:.0f} "
            f"ALPHA:{s.alpha:.2f}"
        )

    def run(self):
        """Main daemon loop."""
        logger.info("=== JetFan v2 Daemon starting ===")
        logger.info(f"Project dir: {PROJECT_DIR}")
        logger.info(f"Log file: {LOG_FILE}")

        # Enable nvidia persistence mode for faster GPU queries
        read_nvidia_persistence_mode()

        # Initial sensor reading
        initial_cpu = read_cpu_temp()
        initial_gpu = read_gpu_temp()
        if initial_cpu:
            self.controller.state.cpu_temp = initial_cpu
            self.controller.state.smoothed_temp = initial_cpu
            self.controller.state.prev_temp = initial_cpu
            logger.info(f"Initial CPU temp: {initial_cpu:.1f}C")
        if initial_gpu:
            self.controller.state.gpu_temp = initial_gpu
            logger.info(f"Initial GPU temp: {initial_gpu:.1f}C")

        # Enable nvidia persistence
        read_nvidia_persistence_mode()

        # Print summary log line periodically
        logger.info("Starting loop (Ctrl+C to stop)...")
        logger.info(
            "TIME|CPU|GPU|EFF|SMOOTH|PWM|PCT%|RPM|SENS|STEP|PID|ALPHA|MODE"
        )

        while self.running:
            cycle_start = time.monotonic()
            s = self.controller.state

            # ── A. Try serial connection ──
            serial_ok = self.serial.connect()
            s.serial_connected = serial_ok

            if serial_ok:
                # Read data from Arduino (sensitivity, RPM, mode)
                data = self.serial.read_data()
                for item in data:
                    if item["type"] == "sensitivity":
                        s.sensitivity = item["value"]
                        s.last_arduino_data = time.monotonic()
                        if not self.arduino_seen:
                            self.arduino_seen = True
                            logger.info(f"Arduino detected. Initial sensitivity: {s.sensitivity}")
                    elif item["type"] == "rpm":
                        s.arduino_rpm = item["value"]
                    elif item["type"] == "mode":
                        logger.info(f"Arduino mode: {item['value']}")

            # If Arduino data timed out, use default sensitivity
            if (time.monotonic() - s.last_arduino_data) > PC_TIMEOUT_S:
                s.sensitivity = DEFAULT_SENSITIVITY
                if self.arduino_seen:
                    self.arduino_seen = False

            # ── B. Compute target PWM ──
            target_pwm, eff_temp = self.controller.update()

            # ── C. Send to Arduino ──
            if serial_ok:
                self.serial.send_pwm(target_pwm)

            # ── D. Log status ──
            now = time.monotonic()
            if now - self.last_log_time >= self.log_interval:
                self.last_log_time = now

                rpm_est = pwm_to_rpm(target_pwm)
                status_line = self.format_status(target_pwm, eff_temp)

                # Detailed log line (CSV-like for parsing)
                log_line = (
                    f"{time.time():.1f}|"
                    f"{s.cpu_temp:.1f}|"
                    f"{s.gpu_temp:.0f}|"
                    f"{eff_temp:.1f}|"
                    f"{s.smoothed_temp:.1f}|"
                    f"{target_pwm}|"
                    f"{target_pwm*100//MAX_PWM}|"
                    f"{rpm_est}|"
                    f"{s.sensitivity}|"
                    f"{s.steps_taken}|"
                    f"{s.pid_adjustment:.0f}|"
                    f"{s.alpha:.2f}|"
                    f"{'AUTO' if serial_ok else 'MANUAL'}"
                )
                logger.info(log_line)

                # Latest one-line status
                self.write_latest_status(status_line)

            # ── E. Timing loop ──
            elapsed = time.monotonic() - cycle_start
            sleep_time = max(0, CYCLE_INTERVAL - elapsed)
            time.sleep(sleep_time)

        # Clean shutdown
        logger.info("Daemon shutting down...")
        # Set safe PWM before exit (send 0 if connected)
        if self.serial.is_connected():
            self.serial.send_pwm(0)
        self.serial.disconnect()
        logger.info("Bye!")
        sys.exit(0)


# ─── Entry Point ─────────────────────────────────────────────────────
def main():
    # Write PID file
    try:
        PID_FILE.write_text(str(os.getpid()))
    except OSError:
        pass

    daemon = JetFanDaemon()
    try:
        daemon.run()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt")
    except Exception as e:
        logger.exception(f"Fatal error: {e}")
        raise
    finally:
        # Cleanup PID file
        try:
            PID_FILE.unlink(missing_ok=True)
        except OSError:
            pass


if __name__ == "__main__":
    main()
