import collections
import logging
import threading

from configfile import ConfigWrapper
from nevermore import Nevermore

SERVO_SIGNAL_PERIOD = 0.020
AMBIENT_TEMP = 25.0
PID_PARAM_BASE = 255.0
SERVO_PROFILE_VERSION = 1
WATERMARK_PROFILE_OPTIONS = {
    "control": (str, "%s", "watermark", False),
    "max_delta": (float, "%.4f", 2.0, True),
    "reverse": (bool, "%s", False, True),
}
# (type, placeholder, default, can_be_none)
PID_PROFILE_OPTIONS = {
    "control": (str, "%s", "pid", False),
    "smooth_time": (float, "%.3f", None, True),
    "pid_kp": (float, "%.3f", None, False),
    "pid_ki": (float, "%.3f", None, False),
    "pid_kd": (float, "%.3f", None, False),
    "reverse": (bool, "%s", False, True),
}


SERVO_SIGNAL_PERIOD = 0.020
AMBIENT_TEMP = 25.0
PID_PARAM_BASE = 255.0
SERVO_PROFILE_VERSION = 1
WATERMARK_PROFILE_OPTIONS = {
    "control": (str, "%s", "watermark", False),
    "max_delta": (float, "%.4f", 2.0, True),
    "reverse": (bool, "%s", False, True),
}
# (type, placeholder, default, can_be_none)
PID_PROFILE_OPTIONS = {
    "control": (str, "%s", "pid", False),
    "smooth_time": (float, "%.3f", None, True),
    "pid_kp": (float, "%.3f", None, False),
    "pid_ki": (float, "%.3f", None, False),
    "pid_kd": (float, "%.3f", None, False),
}


class NevermoreServoControls:
    class ControlBangBang:
        @staticmethod
        def init_profile(config_section, name, pmgr=None):
            temp_profile = {}
            for key, (
                type,
                placeholder,
                default,
                can_be_none,
            ) in WATERMARK_PROFILE_OPTIONS.items():
                if key == "max_delta":
                    above = 0.0
                else:
                    above = None
                temp_profile[key] = pmgr._check_value_config(
                    key,
                    config_section,
                    type,
                    can_be_none,
                    default=default,
                    above=above,
                )
            if name != "default":
                profile_version = config_section.getint("profile_version", None)
                if SERVO_PROFILE_VERSION != profile_version:
                    logging.info(
                        "nevermore_servo_profile: Profile [%s] not compatible with this version\n"
                        "of nevermore_sensor_profile. Profile Version: %d Current Version: %d "
                        % (name, profile_version, SERVO_PROFILE_VERSION)
                    )
                    return None
            return temp_profile

        def __init__(self, profile, servo_controller):
            self.profile = profile
            self.servo_controller = servo_controller
            self.reverse = profile["reverse"]
            self.max_delta = profile["max_delta"]
            self.heating = False

        def temperature_callback(self, read_time, temp):
            current_temp, target_temp = (
                self.servo_controller.temperature_sensor.get_temp(read_time)
            )
            if self.heating != self.reverse and temp >= target_temp + self.max_delta:
                self.heating = self.reverse
            elif self.heating == self.reverse and temp <= target_temp - self.max_delta:
                self.heating = not self.reverse
            if self.heating:
                self.servo_controller.nevermore.set_vent_servo(0.0)
            else:
                self.servo_controller.nevermore.set_vent_servo(1.0)

        def get_type(self):
            return "watermark"

    class ControlPID:
        def __init__(self, profile, servo_controller):
            self.profile = profile
            self.servo_controller = servo_controller
            self.reverse = profile["reverse"]
            self.Kp = profile["pid_kp"] / PID_PARAM_BASE
            self.Ki = profile["pid_ki"] / PID_PARAM_BASE
            self.Kd = profile["pid_kd"] / PID_PARAM_BASE
            self.min_deriv_time = profile["pid_deriv_time"]
            imax = profile["pid_integral_max"]
            self.temp_integ_max = imax / self.Ki
            self.prev_temp = AMBIENT_TEMP
            self.prev_temp_time = 0.0
            self.prev_temp_deriv = 0.0
            self.prev_temp_integ = 0.0

        def temperature_callback(self, read_time, temp):
            current_temp, target_temp = (
                self.servo_controller.temperature_sensor.get_temp(read_time)
            )
            time_diff = read_time - self.prev_temp_time
            # Calculate change of temperature
            temp_diff = temp - self.prev_temp
            if time_diff >= self.min_deriv_time:
                temp_deriv = temp_diff / time_diff
            else:
                temp_deriv = (
                    self.prev_temp_deriv * (self.min_deriv_time - time_diff) + temp_diff
                ) / self.min_deriv_time
            # Calculate accumulated temperature "error"
            temp_err = target_temp - temp
            temp_integ = self.prev_temp_integ + temp_err * time_diff
            temp_integ = max(0.0, min(self.temp_integ_max, temp_integ))
            # Calculate output
            co = self.Kp * temp_err + self.Ki * temp_integ - self.Kd * temp_deriv
            bounded_co = max(0.0, min(1.0, co))
            if not self.reverse:
                self.servo_controller.nevermore.set_vent_servo(
                    max(0.0, 1.0 - bounded_co)
                )
            else:
                self.servo_controller.nevermore.set_vent_servo(max(0.0, bounded_co))
            # Store state for next measurement
            self.prev_temp = temp
            self.prev_temp_time = read_time
            self.prev_temp_deriv = temp_deriv
            if co == bounded_co:
                self.prev_temp_integ = temp_integ

        def get_type(self):
            return "pid"

    class ProfileManager:
        def __init__(self, servo_controller):
            self.servo_controller = servo_controller
            self.control_types = servo_controller.control_types
            self.profiles = {}
            self.incompatible_profiles = []
            # Fetch stored profiles from Config
            stored_profs = self.servo_controller.config.get_prefix_sections(
                "nevermore_sensor_profile %s" % self.servo_controller.name
            )
            for profile in stored_profs:
                if len(self.servo_controller.name.split(" ")) > 1:
                    name = profile.get_name().split(" ", 3)[-1]
                else:
                    name = profile.get_name().split(" ", 2)[-1]
                self._init_profile(profile, name)

        def _init_profile(self, config_section, name, force_control=None):
            control = self._check_value_config(
                "control", config_section, str, False, default=force_control
            )
            if control in self.control_types.keys():
                temp_profile = self.control_types[control].init_profile(
                    config_section, name, self
                )
                if temp_profile is not None:
                    temp_profile["name"] = name
                    temp_profile["control"] = control
                    self.profiles[name] = temp_profile
                return temp_profile
            else:
                raise self.servo_controller.printer.config_error(
                    "Unknown control type '%s' "
                    "in [nevermore_sensor_profile %s %s]."
                    % (control, self.servo_controller.name, name)
                )

        def _check_value_config(
            self,
            key,
            config_section,
            type,
            can_be_none,
            default=None,
            above=None,
            minval=None,
        ):
            if type is int:
                value = config_section.getint(key, default=default, minval=minval)
            elif type is float:
                value = config_section.getfloat(
                    key, default=default, minval=minval, above=above
                )
            elif type == "floatlist":
                value = config_section.getfloatlist(key, default=default)
            elif isinstance(type, tuple) and len(type) == 4 and type[0] == "lists":
                value = config_section.getlists(
                    key,
                    seps=type[1],
                    parser=type[2],
                    count=type[3],
                    default=default,
                )
            else:
                value = config_section.get(key, default=default)
            if not can_be_none and value is None:
                raise self.servo_controller.gcode.error(
                    "nevermore_sensor_profile: '%s' has to be "
                    "specified in [nevermore_sensor_profile %s %s]."
                    % (
                        key,
                        self.servo_controller.name,
                        config_section.get_name(),
                    )
                )
            return value

        def _compute_section_name(self, profile_name):
            return (
                self.servo_controller.name
                if profile_name == "default"
                else (
                    "nevermore_sensor_profile "
                    + self.servo_controller.name
                    + " "
                    + profile_name
                )
            )

        def _check_value_gcmd(
            self,
            name,
            default,
            gcmd,
            type,
            can_be_none,
            minval=None,
            maxval=None,
        ):
            if type is int:
                value = gcmd.get_int(name, default, minval=minval, maxval=maxval)
            elif type is float:
                value = gcmd.get_float(name, default, minval=minval, maxval=maxval)
            else:
                value = gcmd.get(name, default)
            if not can_be_none and value is None:
                raise gcmd.error(
                    "nevermore_sensor_profile: '%s' has to be specified." % name
                )
            return value.lower() if type == "lower" else value

        def init_default_profile(self):
            return self._init_profile(self.servo_controller.config, "default")

    def __init__(self, nevermore: Nevermore, config: ConfigWrapper) -> None:
        self.name = f"{nevermore.name}_servo"
        self.nevermore = nevermore
        self.printer = nevermore.printer
        self.gcode = self.printer.lookup_object("gcode")
        self.last_value = 0.0
        temperature_sensor_name = config.get("chamber_temperature_sensor", None)
        self.temperature_sensor = None
        if temperature_sensor_name is not None:
            try:
                self.temperature_sensor = self.printer.lookup_object(
                    temperature_sensor_name
                )
            except Exception:
                raise config.error(
                    f"Unknown ambient_temp_sensor '{temperature_sensor_name}' specified"
                )
        self.temperature_sensor.setup_callback(self.temperature_callback)

        self.lock = threading.Lock()
        self.control_types = collections.OrderedDict(
            {
                "watermark": self.ControlBangBang,
                "pid": self.ControlPID,
            }
        )
        self.pmgr = self.ProfileManager(self)
        self.control = self.lookup_control(self.pmgr.init_default_profile())
        if self.control is None:
            raise config.error("Default Nevermore_Servo Profile could not be loaded.")

    def load_control(self, gcmd, profile_name):
        profile = self.pmgr.profiles.get(profile_name, None)
        if profile is None:
            raise self.gcode.error(
                "Nevermore_Servo: Unknown profile [%s]" % profile_name
            )
        control = self.lookup_control(profile)
        self.set_control(control)

        gcmd.respond_info("Nevermore_Servo: Profile [%s] loaded" % profile["name"])

    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
        return old_control

    def lookup_control(self, profile):
        return self.control_types[profile["control"]](profile, self)

    def temperature_callback(self, read_time, temp):
        if self.control is not None:
            self.control.temperature_callback(read_time, temp)
