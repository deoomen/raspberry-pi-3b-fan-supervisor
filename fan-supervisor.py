#!/usr/bin/env python3
import RPi.GPIO as GPIO
from gpiozero import CPUTemperature
from time import sleep

class FanSupervisor:

    # fan control settings
    fan_gpio = 18  # GPIO
    fan_low  = 45  # fan OFF below this temp, fan_min% to fan_max% pwm above this
    fan_high = 60  # fan fan_max% pwm above this temp
    fan_time = 10  # sampling time in seconds
    fan_min  = 30  # minimum pwm duty cycle when running (stops stalling)
    fan_max  = 100  # maximum pwm duty cycle when running
    fan_freq = 100  # fan freq in Hz
    hysteresis = 2.0  # temp hysteresis, change fan speed only when edge values are reached

    # auxiliary variables
    dc = 0  # fan speed
    lastCheck = 0.0  # value of last temp check when calculating fan speed
    tempHigh = 0.0  # calculate fan speed when temp is higher than
    tempLow = 0.0  # calculate fan speed when temp is lower than

    def __init__(self) -> None:
        # setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.fan_gpio, GPIO.OUT)
        self.pwm = GPIO.PWM(self.fan_gpio, self.fan_freq)  # set pwm at fan_freq
        self.pwm.start(0)  # start pwm with 0 duty cycle (stopped)

    def __del__(self):
        GPIO.cleanup()

    def supervise(self) -> None:
        while True:
            self.checkTemperature()
            sleep(self.fan_time)

    def checkTemperature(self) -> None:
        temp = CPUTemperature().temperature
        newDc = self.dc

        if temp > self.tempHigh or temp < self.tempLow:
            newDc = self.calculateFanSpeed(temp)

        if self.lastCheck > self.fan_low and newDc != self.dc:
            self.dc = newDc
            self.pwm.ChangeDutyCycle(self.dc)
        elif self.lastCheck < self.fan_low:
            self.pwm.ChangeDutyCycle(0)

    def calculateFanSpeed(self, temp: float) -> int:
        self.lastCheck = temp
        self.tempHigh = temp + self.hysteresis
        self.tempLow = temp - self.hysteresis

        dc = int(((temp - self.fan_low)/(self.fan_high - self.fan_low)) * 100)
        dc = max(dc, self.fan_min)
        dc = min(dc, self.fan_max)

        return dc

if __name__ == '__main__':
    try:
        fanSupervisor = FanSupervisor()
        fanSupervisor.supervise()
    except KeyboardInterrupt as exception:
        pass
    except Exception as exception:
        print(exception)
