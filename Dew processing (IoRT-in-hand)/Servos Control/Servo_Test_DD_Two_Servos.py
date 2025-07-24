#!/usr/bin/env python3

"""
Dual Servo Control on Raspberry Pi (GPIO 10 & 24)
-------------------------------------------------
This script controls two servos using hardware PWM via the RPi.GPIO library.
It moves them through a predefined sequence of angles and safely shuts down.

Connections:
  - Servo 1 (e.g. wrist) → GPIO pin 10
  - Servo 2 (e.g. base)  → GPIO pin 24

"""

import RPi.GPIO as GPIO
import time

# --------------------------- Configuration --------------------------- #
FREQ_HZ = 50  # Standard servo PWM frequency (Hz)

# GPIO pin assignments (BOARD mode)
SERVO1_PIN = 10  # e.g. wrist
SERVO2_PIN = 24  # e.g. base

# Predefined duty cycles (corresponding to angles)
DUTY_0   = 2   # ~0 degrees
DUTY_90  = 7   # ~90 degrees
DUTY_180 = 12  # ~180 degrees

# Time to wait after moving a servo (in seconds)
MOVE_DELAY = 0.5
PAUSE = 2.0
# --------------------------------------------------------------------- #


def set_servo_angle(servo, duty):
    """Send PWM signal to move servo and disable it after delay."""
    servo.ChangeDutyCycle(duty)
    time.sleep(MOVE_DELAY)
    servo.ChangeDutyCycle(0)  # stop sending pulses


def main():
    # Set pin numbering system
    GPIO.setmode(GPIO.BOARD)

    # Set up pins as outputs and initialize PWM
    GPIO.setup(SERVO1_PIN, GPIO.OUT)
    GPIO.setup(SERVO2_PIN, GPIO.OUT)

    servo1 = GPIO.PWM(SERVO1_PIN, FREQ_HZ)
    servo2 = GPIO.PWM(SERVO2_PIN, FREQ_HZ)

    servo1.start(0)
    servo2.start(0)

    try:
        print("[INFO] Moving servos through predefined sequence...")

        # Move servo1 to 90°
        set_servo_angle(servo1, DUTY_90)
        time.sleep(PAUSE)

        # Move servo2 to 90°, servo1 to 0°
        set_servo_angle(servo2, DUTY_90)
        set_servo_angle(servo1, DUTY_0)
        time.sleep(PAUSE)

        # Move servo2 to 180°, servo1 to 90°
        set_servo_angle(servo2, DUTY_180)
        set_servo_angle(servo1, DUTY_90)
        time.sleep(PAUSE)

        # Return both servos to 0°
        set_servo_angle(servo2, DUTY_0)
        set_servo_angle(servo1, DUTY_0)
        time.sleep(PAUSE)

    finally:
        # Always clean up GPIO and stop PWM signals
        print("[INFO] Stopping servos and cleaning up GPIO...")
        servo1.stop()
        servo2.stop()
        GPIO.cleanup()
        print("Goodbye")

if __name__ == "__main__":
    main()

