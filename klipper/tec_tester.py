import random

import asyncio
import datetime
import importlib
import importlib.util
import logging
import os
import os.path
import re
import sys
import threading
import time
import weakref
from abc import ABCMeta, abstractmethod
from dataclasses import dataclass
from enum import Enum
from threading import Thread
from typing import (
    Any,
    Callable,
    Coroutine,
    Dict,
    Generator,
    Iterable,
    List,
    Optional,
    Set,
    Tuple,
    Type,
    TypeVar,
    Union,
)
from uuid import UUID

import bleak
import janus
import serial
from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
from configfile import ConfigWrapper
from extras.heaters import Heater
from extras.led import LEDHelper
from gcode import GCodeCommand, GCodeDispatch
from klippy import Printer
from reactor import SelectReactor
from typing_extensions import override

# Commit war-crimes to load `/tools/nevermore_utilities.py`.
def _import_nevermore_utility():
    self_path = os.path.realpath(__file__)
    target_path = os.path.join(
        os.path.dirname(self_path), "..", "tools", "nevermore_utilities.py"
    )

    spec = importlib.util.spec_from_file_location("nevermore_utilities", target_path)
    assert spec, f"no spec for nevermore path {target_path}"
    assert spec.loader, f"no loader for {spec}"
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_import_nevermore_utility()
from nevermore_utilities import *

__all__ = [
    "load_config",
]


_A = TypeVar("_A")
_B = TypeVar("_B")


MAX_HEAT_TIME = 5.0

class TecTester:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        def opt(mk: Callable[[_A], _B], x: Optional[_A]):
            return mk(x) if x is not None else None


        self.min_temp_cold_side = opt(CmdConfigPeltierMinTempColdSide,
                                      config.getfloat("min_temp_cold_side", default=20))
        self.min_temp_hot_side = opt(CmdConfigPeltierMinTempHotSide,
                                     config.getfloat("min_temp_hot_side", default=20))
        self.max_temp_cold_side = opt(CmdConfigPeltierMaxTempColdSide,
                                      config.getfloat("max_temp_cold_side", default=80))
        self.max_temp_hot_side = opt(CmdConfigPeltierMaxTempHotSide,
                                     config.getfloat("max_temp_hot_side", default=80))
        self.hot_side_safety = opt(CmdConfigPeltierHotSideSafety,
                                   config.getfloat("hot_side_safety", default=10))
        self.max_deviation = opt(CmdConfigPeltierMaxDeviation,
                                 config.getfloat("max_deviation", default=60.0))
        self.dew_point_safety = opt(CmdConfigPeltierDewPointSafety,
                                    config.getfloat("dew_point_safety", default=5.0))
        self.dew_point_range = opt(CmdConfigPeltierDewPointRange,
                                   config.getfloat("dew_point_range", default=10))
        self.dew_point_base = opt(CmdConfigPeltierDewPointBase,
                                  config.getfloat("dew_point_base", default=30))
        self.target_temperature = opt(CmdConfigPeltierTargetTemperature,
                                      config.getfloat("target_temperature", default=50))
        self.enable_delay = opt(CmdConfigPeltierEnableDelay,
                                config.getfloat("enable_delay", 120))
        self.max_pwm = opt(CmdConfigPeltierMaxPwm,
                           config.getfloat("max_pwm", 1, minval=0, maxval=1))
        self.smooth_time = opt(CmdConfigPeltierSmoothTime,
                               config.getfloat("smooth_time", 1.0, above=0.0))
        self.kp = opt(CmdConfigPeltierKp,
                      config.getfloat("pid_kp", 1.0, above=0.0))
        self.ki = opt(CmdConfigPeltierKi,
                      config.getfloat("pid_ki", 1.0, above=0.0))
        self.kd = opt(CmdConfigPeltierKd,
                      config.getfloat("pid_kd", 1.0, above=0.0))

        self.enable = 0

        self.prev_err = 0.0
        self.prev_der = 0.0
        self.int_sum = 0.0
        self.prev_temp_time = 0.0
        self.prev_temp = 25.0

        self.temp_integ_max = 0.0
        if self.ki:
            self.temp_integ_max = self.max_pwm / self.ki

        self.cycle_time = opt(CmdConfigPeltierCycleTime,
                              config.getfloat(
                                  "pwm_cycle_time", 0.0004, above=0.0, maxval=0.25
                              ))

        self.use_pid = opt(CmdConfigPeltierUsePID,
                           config.get("control", "watermark") == "pid")

        self.temperature_sample_timer = self.reactor.register_timer(
            self.callback
        )

        self.last_value = 0
        self.last_enable_time = 0

        self.printer.add_object("heater_fan " + self.name, self)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_TEC_TESTER",
            "TEC_TESTER",
            self.name,
            self.cmd_SET_TEC_TESTER
        )

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready
        )

    def _handle_connect(self):
        self.sensor_cold = self.printer.lookup_object(self.sensor_cold_name)
        self.sensor_hot = self.printer.lookup_object(self.sensor_hot_name)

    def _handle_ready(self):
        self.reactor.update_timer(
            self.temperature_sample_timer, self.reactor.monotonic() + 1.0
        )

    def callback(self, eventtime):
        temp_cold = self.sensor_cold.get_status(eventtime)["temperature"]
        temp_hot = self.sensor_hot.get_status(eventtime)["temperature"]
        if temp_cold < self.min_temp_cold_side:
            self.printer.invoke_shutdown(
                "[%s]\n"
                "Cold side temp too low"
                % (
                    self.name,
                )
            )
        if temp_cold > self.max_temp_cold_side:
            self.printer.invoke_shutdown(
                "[%s]\n"
                "Cold side temp too high"
                % (
                    self.name,
                )
            )
        if temp_hot < self.min_temp_hot_side:
            self.printer.invoke_shutdown(
                "[%s]\n"
                "Hot side temp too low"
                % (
                    self.name,
                )
            )
        if temp_hot > self.max_temp_hot_side:
            self.printer.invoke_shutdown(
                "[%s]\n"
                "Hot side temp too high"
                % (
                    self.name,
                )
            )
        if abs(temp_cold - temp_hot) > self.max_deviation:
            if temp_cold < self.min_temp_cold_side:
                self.printer.invoke_shutdown(
                    "[%s]\n"
                    "Deviation between cold and hot too high"
                    % (
                        self.name,
                    )
                )

        if not self.enable:
            return self.callback_disabled()
        return self.callback_control(temp_cold, temp_hot)

    def callback_disabled(self):
        curtime = self.reactor.monotonic()
        read_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime)
        self.mcu_pwm.set_pwm(read_time, 0)
        return curtime + 0.25

    def callback_watermark(self, temp_cold, temp_hot, enabled):
        curtime = self.reactor.monotonic()
        dew_point = self.dew_point_base + random.randint(0, self.dew_point_range)
        dew_point = dew_point + self.dew_point_safety
        target_temp = self.target_temperature if self.target_temperature > dew_point else dew_point

        read_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime)
        if self.last_value == 0 and read_time < self.last_enable_time + self.enable_delay:
            return 0.25

        if temp_cold < target_temp or temp_hot >= (self.max_temp_cold_side - self.hot_side_safety) or not enabled:
            if self.last_value == 1:
                self.last_enable_time = read_time
            self.last_value = 0
            self.mcu_pwm.set_pwm(read_time, 0)
        else:
            self.last_value = self.max_pwm
            self.mcu_pwm.set_pwm(read_time, self.max_pwm)
        return curtime + 0.25

    def callback_pid(self, temp_cold, temp_hot, enabled):
        curtime = self.reactor.monotonic()
        read_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime)

        # calculate the error
        err = self.target_temperature - temp_cold
        # calculate the time difference
        dt = read_time - self.prev_temp_time
        # calculate the current integral amount using the Trapezoidal rule
        ic = ((self.prev_err + err) / 2.0) * dt
        i = self.int_sum + ic

        # calculate the current derivative using derivative on measurement,
        # to account for derivative kick when the set point changes
        # smooth the derivatives using a modified moving average
        # that handles unevenly spaced data points
        n = max(1.0, self.smooth_time / dt)
        dc = -(temp_cold - self.prev_temp) / dt
        dc = ((n - 1.0) * self.prev_der + dc) / n

        # calculate the output
        o = self.kp * err + self.ki * i + self.kd * dc
        # calculate the saturated output
        so = max(0.0, min(self.max_pwm, o))

        pwm = self.max_pwm - so

        # update the heater
        if temp_hot >= (self.max_temp_cold_side - self.hot_side_safety) or not enabled:
            pwm = 0.0
        self.mcu_pwm.set_pwm(read_time, pwm)
        # update the previous values
        self.prev_temp = temp_cold
        self.prev_temp_time = read_time
        self.prev_der = dc
        if temp_hot < (self.max_temp_cold_side - self.hot_side_safety) and enabled:
            self.prev_err = err
            if o == so:
                # not saturated so an update is allowed
                self.int_sum = i
            else:
                # saturated, so conditionally integrate
                if (o > 0.0) - (o < 0.0) != (ic > 0.0) - (ic < 0.0):
                    # the signs are opposite so an update is allowed
                    self.int_sum = i
        else:
            self.prev_err = 0.0
            self.int_sum = 0.0

        return curtime + 0.25


    def cmd_SET_TEC_TESTER(self, gcmd):
        self.target_temperature = gcmd.get_float("TARGET", self.target_temperature)
        self.enable = gcmd.get_int("ENABLE", self.enable, minval=0, maxval=1)
        gcmd.respond_info(f"TARGET_TEMP={self.target_temperature}")
        gcmd.respond_info(f"ENABLE={self.enable}")

    def get_status(self, eventtime):
        return {
            "speed": self.last_value,
            "pwm_value": self.last_value,
            "rpm": None,
        }

def load_config(config):
    return TecTester(config)

def load_config_prefix(config):
    return TecTester(config)
