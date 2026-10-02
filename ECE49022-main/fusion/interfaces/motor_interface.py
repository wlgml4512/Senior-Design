"""Motor-output interface for fusion.

This module is the final translation layer between fusion's motion command and
the embedded drivetrain backend. It does three jobs:
1. Normalize planner `linear` and `angular` motion into a [-1, 1] drive space.
2. Mix that motion into left/right side commands for the differential drive.
3. Adapt those side commands onto the embedded motor API, even if the embedded
   module evolves slightly over time.

Fusion intentionally avoids owning encoder resources here so the drivetrain
command path stays independent of separate telemetry/UI processes.
"""

from pathlib import Path
import inspect
import importlib
import os
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


DEFAULT_MAX_LINEAR_SPEED_MPS = 0.6
DEFAULT_MAX_ANGULAR_SPEED_RAD_S = 1.0
DEFAULT_TURN_MIX_GAIN = 0.75
DEFAULT_MIN_COMMAND_DEADBAND = 0.05
DEFAULT_PWM_FREQUENCY_HZ = 1000
_MOTOR_OUTPUT_ENABLED = False
MAX_LINEAR_SPEED_MPS = DEFAULT_MAX_LINEAR_SPEED_MPS
MAX_ANGULAR_SPEED_RAD_S = DEFAULT_MAX_ANGULAR_SPEED_RAD_S
TURN_MIX_GAIN = DEFAULT_TURN_MIX_GAIN
MIN_COMMAND_DEADBAND = DEFAULT_MIN_COMMAND_DEADBAND


def _clamp(value, lower, upper):
    """Clamp a numeric value into an inclusive range."""
    return max(lower, min(upper, value))


def configure_motor_mapping(
    *,
    max_linear_speed_mps=None,
    max_angular_speed_rad_s=None,
    turn_mix_gain=None,
    min_command_deadband=None,
):
    """Update the motion-to-motor scaling used by the drivetrain backend."""
    global MAX_LINEAR_SPEED_MPS
    global MAX_ANGULAR_SPEED_RAD_S
    global TURN_MIX_GAIN
    global MIN_COMMAND_DEADBAND

    if max_linear_speed_mps is not None:
        max_linear_speed_mps = float(max_linear_speed_mps)
        if max_linear_speed_mps <= 0.0:
            raise ValueError("max_linear_speed_mps must be greater than zero.")
        MAX_LINEAR_SPEED_MPS = max_linear_speed_mps

    if max_angular_speed_rad_s is not None:
        max_angular_speed_rad_s = float(max_angular_speed_rad_s)
        if max_angular_speed_rad_s <= 0.0:
            raise ValueError("max_angular_speed_rad_s must be greater than zero.")
        MAX_ANGULAR_SPEED_RAD_S = max_angular_speed_rad_s

    if turn_mix_gain is not None:
        TURN_MIX_GAIN = float(turn_mix_gain)

    if min_command_deadband is not None:
        min_command_deadband = float(min_command_deadband)
        if min_command_deadband < 0.0:
            raise ValueError("min_command_deadband must be non-negative.")
        MIN_COMMAND_DEADBAND = min_command_deadband


def _get_env_int(name):
    """Read an integer environment override, returning None when unset."""
    value = os.getenv(name)
    if value is None or value == "":
        return None
    try:
        return int(value)
    except ValueError:
        return None


def _get_module_int(module, name):
    """Read an integer constant from the embedded motor module."""
    value = getattr(module, name, None)
    if value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _get_env_bool(name, default=False):
    """Read a boolean environment override using common truthy strings."""
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _load_motor_module():
    """Import the embedded drivetrain backend if it is available."""
    try:
        return importlib.import_module("embedded.motors_encoders")
    except ModuleNotFoundError:
        return None


def _resolve_pin(motor_module, env_name):
    """Resolve one wiring constant from env overrides first, then module defaults."""
    value = _get_env_int(env_name)
    if value is not None:
        return value
    return _get_module_int(motor_module, env_name)


def _get_callable(obj, *names):
    """Return the first callable attribute that exists on an object."""
    for name in names:
        candidate = getattr(obj, name, None)
        if callable(candidate):
            return candidate
    return None


def _get_signature_kwargs(factory, alias_map):
    """Map known argument aliases onto a callable's signature.

    This lets fusion tolerate small naming changes in the embedded subsystem,
    such as `direction_pin` instead of `dir_pin`.
    """
    try:
        signature = inspect.signature(factory)
    except (TypeError, ValueError):
        return None

    kwargs = {}
    accepts_var_kwargs = any(
        parameter.kind == inspect.Parameter.VAR_KEYWORD
        for parameter in signature.parameters.values()
    )

    for parameter_name in signature.parameters:
        if parameter_name in {"self", "cls"}:
            continue
        if parameter_name in alias_map:
            kwargs[parameter_name] = alias_map[parameter_name]
            continue
        for alias, value in alias_map.items():
            if alias == parameter_name:
                kwargs[parameter_name] = value
                break

    if accepts_var_kwargs:
        for alias, value in alias_map.items():
            kwargs.setdefault(alias, value)

    return kwargs


def _build_motor(motor_module, handle, pwm_pin, dir_pin, freq):
    """Construct one embedded Motor using either keyword or positional forms."""
    alias_map = {
        "handle": handle,
        "chip_handle": handle,
        "gpiochip_handle": handle,
        "pwm_pin": pwm_pin,
        "dir_pin": dir_pin,
        "direction_pin": dir_pin,
        "freq": freq,
        "frequency": freq,
        "pwm_frequency_hz": freq,
    }

    motor_cls = motor_module.Motor
    kwargs = _get_signature_kwargs(motor_cls, alias_map)
    if kwargs is not None:
        try:
            return motor_cls(**kwargs)
        except TypeError:
            pass
    return motor_cls(handle, pwm_pin, dir_pin, freq=freq)


def _open_gpiochip(motor_module, chip_index=0):
    """Open the GPIO chip through either a module helper or raw lgpio access."""
    open_fn = _get_callable(motor_module, "open_gpiochip")
    if open_fn is not None:
        return open_fn(chip_index)

    lgpio_module = getattr(motor_module, "lgpio", None)
    if lgpio_module is not None:
        lgpio_open = _get_callable(lgpio_module, "gpiochip_open")
        if lgpio_open is not None:
            return lgpio_open(chip_index)

    raise AttributeError("Embedded motor module does not expose a gpiochip open helper.")


def _close_gpiochip(motor_module, handle):
    """Close the shared GPIO chip handle using the backend's preferred helper."""
    close_fn = _get_callable(motor_module, "close_gpiochip")
    if close_fn is not None:
        close_fn(handle)
        return

    lgpio_module = getattr(motor_module, "lgpio", None)
    if lgpio_module is not None:
        lgpio_close = _get_callable(lgpio_module, "gpiochip_close")
        if lgpio_close is not None:
            lgpio_close(handle)
            return

    raise AttributeError("Embedded motor module does not expose a gpiochip close helper.")


def _stop_motor_instance(motor):
    """Stop a motor regardless of which concrete drive API it exposes."""
    stop_fn = _get_callable(motor, "stop")
    if stop_fn is not None:
        stop_fn()
        return

    drive_fn = _get_callable(motor, "drive", "set_output", "set_command")
    if drive_fn is not None:
        drive_fn(0.0)
        return

    set_speed = _get_callable(motor, "set_speed")
    if set_speed is not None:
        set_speed(0.0)


def _cleanup_motor_instance(motor):
    """Release motor-side resources if the concrete backend exposes cleanup hooks."""
    cleanup_fn = _get_callable(motor, "cleanup", "close")
    if cleanup_fn is not None:
        cleanup_fn()


def _normalize_motion(cmd):
    """Convert planner units into normalized drive-space commands."""
    linear = float(cmd.get("linear", 0.0))
    angular = float(cmd.get("angular", 0.0))

    # Convert planner units into a normalized drive command before mixing into
    # left/right motor values.
    linear_norm = _clamp(linear / MAX_LINEAR_SPEED_MPS, -1.0, 1.0)
    angular_norm = _clamp(angular / MAX_ANGULAR_SPEED_RAD_S, -1.0, 1.0)
    return linear_norm, angular_norm


def _mix_to_side_commands(cmd):
    """Mix normalized linear/angular motion into left and right side commands."""
    linear_norm, angular_norm = _normalize_motion(cmd)

    # Differential/skid-steer mixing:
    # positive angular means turn left, so the left side slows down and the right speeds up.
    left = linear_norm - (TURN_MIX_GAIN * angular_norm)
    right = linear_norm + (TURN_MIX_GAIN * angular_norm)

    left = _clamp(left, -1.0, 1.0)
    right = _clamp(right, -1.0, 1.0)

    if abs(left) < MIN_COMMAND_DEADBAND:
        left = 0.0
    if abs(right) < MIN_COMMAND_DEADBAND:
        right = 0.0

    return {
        "left": left,
        "right": right,
        "linear_normalized": linear_norm,
        "angular_normalized": angular_norm,
    }


class _MotorSide:
    """Wrap one side of the drivetrain and hide backend-specific motor calls."""

    def __init__(self, motor, invert_direction=False):
        self.motor = motor
        self.invert_direction = invert_direction

    def drive(self, command):
        """Apply one normalized side command to the concrete motor object."""
        if abs(command) < MIN_COMMAND_DEADBAND:
            _stop_motor_instance(self.motor)
            return {"command": 0.0, "direction": None, "duty_cycle": 0.0}

        direction = 1 if command < 0 else 0
        if self.invert_direction:
            direction = 1 - direction

        duty_cycle = _clamp(abs(command), 0.0, 1.0)
        set_direction = _get_callable(self.motor, "set_direction")
        set_speed = _get_callable(self.motor, "set_speed")

        if set_direction is not None and set_speed is not None:
            # Older embedded code exposes separate direction and duty-cycle APIs.
            set_direction(direction)
            set_speed(duty_cycle)
        else:
            # Newer/refactored motor APIs may accept a single signed command.
            signed_command = -duty_cycle if direction == 1 else duty_cycle
            drive_fn = _get_callable(self.motor, "drive", "set_output", "set_command")
            if drive_fn is None:
                raise AttributeError("Embedded Motor object does not expose a supported drive API.")
            drive_fn(signed_command)

        return {"command": command, "direction": direction, "duty_cycle": duty_cycle}

    def stop(self):
        """Stop this motor side without tearing down the object."""
        _stop_motor_instance(self.motor)

    def cleanup(self):
        """Stop and release this motor side."""
        _stop_motor_instance(self.motor)
        _cleanup_motor_instance(self.motor)


class _DifferentialDriveBackend:
    """Lazy-initialized adapter around the embedded left/right drivetrain."""

    def __init__(self):
        self.motor_module = _load_motor_module()
        self.available = False
        self.initialized = False
        self.handle = None
        self.left = None
        self.right = None

        if self.motor_module is None or not hasattr(self.motor_module, "Motor"):
            return

        self.left_pwm = _resolve_pin(self.motor_module, "LEFT_MOTOR_PWM_PIN")
        self.left_dir = _resolve_pin(self.motor_module, "LEFT_MOTOR_DIR_PIN")
        self.right_pwm = _resolve_pin(self.motor_module, "RIGHT_MOTOR_PWM_PIN")
        self.right_dir = _resolve_pin(self.motor_module, "RIGHT_MOTOR_DIR_PIN")

        if None in (self.left_pwm, self.left_dir, self.right_pwm, self.right_dir):
            # If the robot-specific pin map is incomplete, keep the backend in
            # dry-run mode instead of trying to touch hardware.
            return

        self.available = True

    def _initialize(self):
        """Create hardware objects on first use instead of at import time."""
        if not self.available or self.initialized:
            return self.initialized

        left_motor = None
        right_motor = None
        try:
            self.handle = _open_gpiochip(self.motor_module, 0)

            # All low-level motor objects share the same GPIO chip handle.
            pwm_frequency_hz = _get_env_int("MOTOR_PWM_FREQUENCY_HZ") or DEFAULT_PWM_FREQUENCY_HZ
            left_motor = _build_motor(self.motor_module, self.handle, self.left_pwm, self.left_dir, pwm_frequency_hz)
            right_motor = _build_motor(self.motor_module, self.handle, self.right_pwm, self.right_dir, pwm_frequency_hz)

            self.left = _MotorSide(
                left_motor,
                invert_direction=_get_env_bool("LEFT_MOTOR_INVERT_DIRECTION"),
            )
            self.right = _MotorSide(
                right_motor,
                invert_direction=_get_env_bool("RIGHT_MOTOR_INVERT_DIRECTION"),
            )
            self.initialized = True
        except Exception:
            # Any setup failure drops back to dry-run behavior instead of
            # crashing the whole fusion loop.
            if left_motor is not None:
                try:
                    left_motor.cleanup()
                except Exception:
                    pass
            if right_motor is not None:
                try:
                    right_motor.cleanup()
                except Exception:
                    pass
            if self.handle is not None:
                try:
                    _close_gpiochip(self.motor_module, self.handle)
                except Exception:
                    pass
            self.initialized = False
            self.handle = None
            self.left = None
            self.right = None

        return self.initialized

    def drive(self, side_commands):
        """Send left/right commands to hardware or report a dry-run fallback."""
        if not self._initialize():
            return {
                "backend": "dry_run",
                "left": {"command": side_commands["left"]},
                "right": {"command": side_commands["right"]},
            }

        left_status = self.left.drive(side_commands["left"])
        right_status = self.right.drive(side_commands["right"])
        return {
            "backend": "embedded",
            "left": left_status,
            "right": right_status,
        }

    def stop(self):
        """Stop both sides without fully destroying the backend."""
        if self.left is not None:
            self.left.stop()
        if self.right is not None:
            self.right.stop()

    def shutdown(self):
        """Stop, clean up, and release the shared GPIO resources."""
        try:
            self.stop()
            if self.left is not None:
                self.left.cleanup()
            if self.right is not None:
                self.right.cleanup()
        finally:
            if self.handle is not None:
                _close_gpiochip(self.motor_module, self.handle)
            self.handle = None
            self.left = None
            self.right = None
            self.initialized = False


_DRIVE_BACKEND = _DifferentialDriveBackend()


def set_motor_output_enabled(enabled):
    """Arm or disarm real hardware writes from fusion."""
    global _MOTOR_OUTPUT_ENABLED
    # Hardware writes are intentionally gated so we can run fusion on the robot
    # for observation/logging before actually allowing motion.
    _MOTOR_OUTPUT_ENABLED = bool(enabled)
    if not _MOTOR_OUTPUT_ENABLED and _DRIVE_BACKEND.initialized:
        _DRIVE_BACKEND.shutdown()


def build_motor_command(cmd):
    """Convert a fused motion command into normalized left/right drive values."""
    side_commands = _mix_to_side_commands(cmd)
    # Keep both the original planner command and the per-side drive values so
    # logs can explain how a motion decision became motor output.
    return {
        "reason": cmd.get("reason"),
        "stop": bool(cmd.get("stop", False)),
        "linear": float(cmd.get("linear", 0.0)),
        "angular": float(cmd.get("angular", 0.0)),
        "left_motor": side_commands["left"],
        "right_motor": side_commands["right"],
        "linear_normalized": side_commands["linear_normalized"],
        "angular_normalized": side_commands["angular_normalized"],
    }


def send_motion_command(cmd):
    """Send the final fused motion command through the drivetrain backend."""
    motor_cmd = build_motor_command(cmd)

    # Preserve turn-in-place commands even when the planner marks the motion as a stop.
    should_force_zero = motor_cmd["stop"] and abs(motor_cmd["angular"]) < 1e-6
    if should_force_zero:
        motor_cmd["left_motor"] = 0.0
        motor_cmd["right_motor"] = 0.0

    backend_report = None
    if _DRIVE_BACKEND.available and _MOTOR_OUTPUT_ENABLED:
        # Only send real PWM updates when the embedded backend exists and the
        # current run explicitly armed motor output.
        backend_report = _DRIVE_BACKEND.drive(
            {
                "left": motor_cmd["left_motor"],
                "right": motor_cmd["right_motor"],
            }
        )
    else:
        # In dry-run/disarmed mode we still expose the exact command that would
        # have been sent to the drivetrain.
        backend_report = {
            "backend": "dry_run" if not _DRIVE_BACKEND.available else "disarmed",
            "left": {"command": motor_cmd["left_motor"]},
            "right": {"command": motor_cmd["right_motor"]},
        }

    # The console log is intentionally human-readable so on-robot tests can be
    # debugged without opening a trace file.
    print(
        "[MOTOR OUTPUT] "
        f"backend={backend_report['backend']} | "
        f"reason={motor_cmd['reason']} | "
        f"stop={motor_cmd['stop']} | "
        f"linear={motor_cmd['linear']:.2f} m/s | "
        f"angular={motor_cmd['angular']:+.2f} rad/s | "
        f"left={motor_cmd['left_motor']:.2f} | "
        f"right={motor_cmd['right_motor']:.2f}"
    )

    return {
        **motor_cmd,
        "backend_report": backend_report,
    }
