import time

import cv2
import serial
from ultralytics import YOLO


# ============================================================
# USER SETTINGS
# ============================================================

COM_PORT = "COM4"
BAUD_RATE = 115200

CAMERA_INDEX = 1
MODEL_FILE = "drone_brain_v1.pt"

CONFIDENCE_THRESHOLD = 0.50


# ============================================================
# CAMERA / OPTICS SETTINGS
# ============================================================

# Measure these for your actual camera/lens combo (check the
# datasheet, or measure empirically by pointing at two known
# reference points and computing the angle between them).
# Getting this right is what makes the gain physically meaningful
# instead of an arbitrary tuned number.
HORIZONTAL_FOV_DEG = 60.0
VERTICAL_FOV_DEG = 45.0


# ============================================================
# SERVO SETTINGS
# ============================================================

PAN_NEUTRAL = 375
TILT_NEUTRAL = 375

# Full configured range for the 270-degree servos.
MIN_PWM = 150
MAX_PWM = 600
SERVO_TRAVEL_DEG = 270.0
PWM_PER_DEGREE = (MAX_PWM - MIN_PWM) / SERVO_TRAVEL_DEG

# Corrected directions based on the previous test.
PAN_DIRECTION = -1
TILT_DIRECTION = -1


# ============================================================
# TRACKING SETTINGS
# ============================================================

# Ignore small errors to prevent slow endpoint drift.
# Kept tight since error is now measured in PWM-equivalent degrees,
# not raw pixels, so this is no longer resolution-dependent.
DEADZONE_PWM = 3.0

# Gains now operate on "PWM degrees of error" rather than raw pixels,
# and the derivative term is normalized by real elapsed time (dt),
# so these should need less re-tuning if camera or frame rate changes.
# START HERE: KD=0, raise KP until you see slight oscillation around
# the target, then back off ~30-40%. Only then start raising KD from
# 0 in small increments (e.g. 0.005 at a time).
KP = 0.15
KD = 0.0

# Derivative is low-pass filtered (see DERIVATIVE_FILTER_ALPHA below)
# before KD is applied, since raw frame-to-frame derivative divided
# by a small dt amplifies ordinary detection jitter into large kicks.
DERIVATIVE_FILTER_ALPHA = 0.3

# Step limit now ramps continuously between these two values based on
# how far off-target the error is, instead of snapping between two
# fixed speeds. This gives a true deceleration curve as the target
# nears center rather than cruising at a constant speed then halting.
MAX_STEP_NEAR = 1.5
MAX_STEP_FAR = 8.0
RAMP_START_PWM = DEADZONE_PWM   # where deceleration begins
RAMP_END_PWM = 30.0             # where full speed is reached

# Exponential smoothing factor for the detected target center.
# Higher = more responsive but jittery; lower = smoother but laggier.
SMOOTHING_ALPHA = 0.6


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def send_servo_positions(serial_port, pan_value, tilt_value):
    """
    Send:
    [header, pan high, pan low, tilt high, tilt low]
    """

    pan_value = int(clamp(pan_value, MIN_PWM, MAX_PWM))
    tilt_value = int(clamp(tilt_value, MIN_PWM, MAX_PWM))

    packet = bytes(
        [
            255,
            (pan_value >> 8) & 0xFF,
            pan_value & 0xFF,
            (tilt_value >> 8) & 0xFF,
            tilt_value & 0xFF,
        ]
    )

    serial_port.write(packet)


def pixel_error_to_pwm_degrees(error_px, frame_dimension_px, fov_deg):
    """
    Convert a pixel error into an equivalent PWM-tick error, using
    the camera's field of view and the servo's PWM-per-degree.
    This makes the error physically meaningful instead of an
    arbitrary pixel count, so gains transfer across resolutions,
    cameras, and lenses.
    """

    degrees_per_px = fov_deg / frame_dimension_px

    return error_px * degrees_per_px * PWM_PER_DEGREE


def ramped_step_limit(error_abs):
    """
    Continuously interpolate the allowed step size between
    MAX_STEP_NEAR (right at the edge of the deadzone) and
    MAX_STEP_FAR (once the error is large enough to need full speed).
    This produces a true deceleration curve as the target approaches
    center, instead of a sudden speed change at a fixed threshold.
    """

    if error_abs <= RAMP_START_PWM:
        return MAX_STEP_NEAR

    if error_abs >= RAMP_END_PWM:
        return MAX_STEP_FAR

    progress = (error_abs - RAMP_START_PWM) / (RAMP_END_PWM - RAMP_START_PWM)

    return MAX_STEP_NEAR + progress * (MAX_STEP_FAR - MAX_STEP_NEAR)


def calculate_servo_step(
    error_pwm,
    previous_error_pwm,
    previous_filtered_derivative,
    dt,
    direction,
):
    """
    Calculate one PD controller movement step.

    error_pwm: current error, already converted to PWM-equivalent
               degrees via pixel_error_to_pwm_degrees().
    dt:        seconds elapsed since the previous frame's error was
               computed. Normalizes the derivative term so gains
               don't drift with variable inference time.

    The raw derivative (error delta / dt) is extremely sensitive to
    ordinary detection jitter -- a few pixels of noise divided by a
    small dt produces a large spike. It's passed through a low-pass
    filter (DERIVATIVE_FILTER_ALPHA) before KD is applied, so only
    sustained motion trends drive the derivative term, not frame-to-
    frame noise.
    """

    if abs(error_pwm) <= DEADZONE_PWM:
        return 0.0, error_pwm, previous_filtered_derivative

    dt = max(dt, 1e-3)  # guard against div-by-zero / huge spikes

    raw_derivative = (error_pwm - previous_error_pwm) / dt

    filtered_derivative = (
        DERIVATIVE_FILTER_ALPHA * raw_derivative
        + (1 - DERIVATIVE_FILTER_ALPHA) * previous_filtered_derivative
    )

    step = direction * (
        (error_pwm * KP) +
        (filtered_derivative * KD)
    )

    step_limit = ramped_step_limit(abs(error_pwm))

    step = clamp(step, -step_limit, step_limit)

    return step, error_pwm, filtered_derivative


print("System booting...")

arduino = None
camera = None

try:
    # ========================================================
    # LOAD YOLO MODEL
    # ========================================================

    print(f"Loading model: {MODEL_FILE}")

    model = YOLO(MODEL_FILE)

    print("Model loaded.")

    # ========================================================
    # CONNECT TO ARDUINO
    # ========================================================

    try:
        arduino = serial.Serial(
            port=COM_PORT,
            baudrate=BAUD_RATE,
            timeout=0.1,
        )

        # Arduino may reset when the serial connection opens.
        time.sleep(2)

        arduino.reset_input_buffer()
        arduino.reset_output_buffer()

        # Center the turret before tracking begins.
        send_servo_positions(
            arduino,
            PAN_NEUTRAL,
            TILT_NEUTRAL,
        )

        time.sleep(1)

        print(f"Arduino connected on {COM_PORT}")

    except Exception as error:
        arduino = None

        print(
            f"WARNING: Could not connect to Arduino on "
            f"{COM_PORT}: {error}"
        )

        print("Running in vision-only mode.")

    # ========================================================
    # OPEN CAMERA
    # ========================================================

    camera = cv2.VideoCapture(
        CAMERA_INDEX,
        cv2.CAP_DSHOW,
    )

    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not camera.isOpened():
        raise RuntimeError(
            f"Could not open camera index {CAMERA_INDEX}"
        )

    # ========================================================
    # TRACKING STATE
    # ========================================================

    pan_pwm = float(PAN_NEUTRAL)
    tilt_pwm = float(TILT_NEUTRAL)

    previous_error_x_pwm = 0.0
    previous_error_y_pwm = 0.0
    filtered_derivative_x = 0.0
    filtered_derivative_y = 0.0
    previous_frame_time = time.time()

    smoothed_x = None
    smoothed_y = None

    print("Tracking active. Press Q to quit.")

    # ========================================================
    # MAIN TRACKING LOOP
    # ========================================================

    while True:
        frame_received, frame = camera.read()

        if not frame_received:
            print("Camera feed lost.")
            break

        # Use the actual frame size instead of assuming 1280x720.
        frame_height, frame_width = frame.shape[:2]

        frame_center_x = frame_width // 2
        frame_center_y = frame_height // 2

        results = model(
            frame,
            stream=True,
            verbose=False,
        )

        target_found = False
        best_confidence = 0.0

        target_x = 0
        target_y = 0
        target_box = None

        # Find the detection with the highest confidence.
        for result in results:
            for box in result.boxes:
                confidence = float(box.conf[0])

                if (
                    confidence >= CONFIDENCE_THRESHOLD
                    and confidence > best_confidence
                ):
                    best_confidence = confidence
                    target_found = True

                    coordinates = box.xyxy[0].tolist()

                    x1 = int(coordinates[0])
                    y1 = int(coordinates[1])
                    x2 = int(coordinates[2])
                    y2 = int(coordinates[3])

                    target_box = (
                        x1,
                        y1,
                        x2,
                        y2,
                    )

                    target_x = (x1 + x2) // 2
                    target_y = (y1 + y2) // 2

        # ====================================================
        # TIMING
        # ====================================================

        now = time.time()
        dt = now - previous_frame_time
        previous_frame_time = now

        # ====================================================
        # SERVO CONTROL
        # ====================================================

        if target_found:
            # Smooth the detected center to reduce box jitter from
            # feeding straight into the controller as raw error.
            if smoothed_x is None:
                smoothed_x = float(target_x)
                smoothed_y = float(target_y)
            else:
                smoothed_x = (
                    SMOOTHING_ALPHA * target_x
                    + (1 - SMOOTHING_ALPHA) * smoothed_x
                )
                smoothed_y = (
                    SMOOTHING_ALPHA * target_y
                    + (1 - SMOOTHING_ALPHA) * smoothed_y
                )

            error_x_px = smoothed_x - frame_center_x
            error_y_px = smoothed_y - frame_center_y

            # Convert pixel error into PWM-equivalent degrees using
            # the camera's real FOV, so gains are physically meaningful.
            error_x_pwm = pixel_error_to_pwm_degrees(
                error_x_px, frame_width, HORIZONTAL_FOV_DEG
            )
            error_y_pwm = pixel_error_to_pwm_degrees(
                error_y_px, frame_height, VERTICAL_FOV_DEG
            )

            pan_step, previous_error_x_pwm, filtered_derivative_x = (
                calculate_servo_step(
                    error_x_pwm,
                    previous_error_x_pwm,
                    filtered_derivative_x,
                    dt,
                    PAN_DIRECTION,
                )
            )

            tilt_step, previous_error_y_pwm, filtered_derivative_y = (
                calculate_servo_step(
                    error_y_pwm,
                    previous_error_y_pwm,
                    filtered_derivative_y,
                    dt,
                    TILT_DIRECTION,
                )
            )

            pan_pwm = clamp(
                pan_pwm + pan_step,
                MIN_PWM,
                MAX_PWM,
            )

            tilt_pwm = clamp(
                tilt_pwm + tilt_step,
                MIN_PWM,
                MAX_PWM,
            )

            if arduino is not None:
                send_servo_positions(
                    arduino,
                    pan_pwm,
                    tilt_pwm,
                )

            # =================================================
            # DRAW TARGET INFORMATION
            # =================================================

            x1, y1, x2, y2 = target_box

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 0, 255),
                2,
            )

            cv2.drawMarker(
                frame,
                (int(smoothed_x), int(smoothed_y)),
                (0, 255, 0),
                cv2.MARKER_CROSS,
                25,
                2,
            )

            cv2.line(
                frame,
                (frame_center_x, frame_center_y),
                (int(smoothed_x), int(smoothed_y)),
                (255, 255, 0),
                2,
            )

            cv2.putText(
                frame,
                f"LOCK: {best_confidence * 100:.1f}%",
                (x1, max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

            cv2.putText(
                frame,
                f"ERR PWM X: {error_x_pwm:.1f}  Y: {error_y_pwm:.1f}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )

        else:
            # Clear derivative memory and smoothing when the target
            # is lost, so re-acquisition doesn't inherit stale state.
            previous_error_x_pwm = 0.0
            previous_error_y_pwm = 0.0
            filtered_derivative_x = 0.0
            filtered_derivative_y = 0.0
            smoothed_x = None
            smoothed_y = None

            cv2.putText(
                frame,
                "SEARCHING...",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )

        # ====================================================
        # DRAW GENERAL HUD
        # ====================================================

        cv2.drawMarker(
            frame,
            (frame_center_x, frame_center_y),
            (255, 255, 255),
            cv2.MARKER_CROSS,
            30,
            2,
        )

        cv2.putText(
            frame,
            f"PAN: {int(pan_pwm)}  TILT: {int(tilt_pwm)}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        if arduino is None:
            cv2.putText(
                frame,
                "ARDUINO DISCONNECTED",
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        cv2.imshow(
            "AFWERX Turret HUD",
            frame,
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("Stopped by user.")

except Exception as error:
    print(f"ERROR: {error}")

finally:
    # ========================================================
    # SAFE SHUTDOWN
    # ========================================================

    if arduino is not None:
        try:
            print("Centering turret...")

            send_servo_positions(
                arduino,
                PAN_NEUTRAL,
                TILT_NEUTRAL,
            )

            time.sleep(0.75)
            arduino.close()

        except Exception:
            pass

    if camera is not None:
        camera.release()

    cv2.destroyAllWindows()

    print("System shut down.")
