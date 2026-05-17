#!/usr/bin/env python2

import math
from enum import IntEnum
from threading import Lock
import Jetson.GPIO as GPIO


class WheelDirection(IntEnum):
    FORWARD = 1
    REVERSE = -1


class WheelEncoderDriver(object):
    """Class handling communication with a wheel encoder on a Jetson Nano."""

    # Hardcoded resolution discovered from physical testing
    PPR = 146

    def __init__(self, gpio_pin):

        self._radius = 0.03 #wheel radius

        # Valid gpio_pin check for physical header limits
        if not 1 <= gpio_pin <= 40:
            raise ValueError("The pin number must be within the range [1, 40].")

        self._gpio_pin = gpio_pin
        
        # NOTE: Ensure GPIO.setmode is consistent across your entire project
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(gpio_pin, GPIO.IN)
        GPIO.add_event_detect(gpio_pin, GPIO.RISING, callback=self._cb)

        self._ticks = 0
        self._direction = WheelDirection.FORWARD
        
        # Thread lock to prevent race conditions during async interrupts
        self._lock = Lock()

    def get_direction(self):
        return self._direction

    def set_direction(self, direction):
        self._direction = direction

    def _cb(self, _):
        # Secure the lock while modifying the tick count
        with self._lock:
            self._ticks += self._direction.value

    def get_ticks(self):
        """Returns the current cumulative tick count safely."""
        with self._lock:
            return self._ticks

    def get_radians(self):
        """Converts the current tick count into radians."""
        with self._lock:
            return (float(self._ticks) / self.PPR) * (2.0 * math.pi)

    def reset(self):
        """Resets the angular position back to zero."""
        with self._lock:
            self._ticks = 0
    
    def get_distance_meters(self):
        """Calculates the total linear distance traveled in meters."""
        # Distance = Radius * Theta (in radians)
        return self._radius * self.get_radians()

    def shutdown(self):
        GPIO.remove_event_detect(self._gpio_pin)